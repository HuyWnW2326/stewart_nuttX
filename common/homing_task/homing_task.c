/****************************************************************************
 * common/homing_task/homing_task.c
 *
 * Dua ba truc ve LIMIT_DOWN, chot moc encoder va nang len vi tri hoat
 * dong ban dau. Task cho limit co timeout de nhan ra EMERGENCY; neu bi
 * abort giua chung thi khong duoc dat homed=true, thong bao homing hoan
 * tat hay goi cut_all_son(), vi STEPIOC_ESTOP da cat SON.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "system_state.h"
#include "step_ioctl.h"
#include "stm32_sensorbtn.h"
#include "motor_pos.h"
#include "safety_task.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HOMING_MOTOR_COUNT          MOTOR_COUNT
#define HOMING_FREQ_HZ              100000UL   /* tan so xung khi homing */
#define HOMING_LIFT_FREQ_HZ         100000UL   /* tan so xung khi nang len */

#define HOMING_GEAR_RATIO           100.0f
#define HOMING_PPR                  10000.0f

#define HOMING_ZERO_CAPTURE_DELAY_US   150000UL
#define HOMING_ZERO_FRESH_MAX_AGE_MS   200UL
#define HOMING_ZERO_FRESH_RETRY_COUNT  5
#define HOMING_ZERO_FRESH_RETRY_US     50000UL


/* Tinh so pulse tu goc (do):
 * pulses = (deg / 360) * GEAR_RATIO * PPR
 * Vi du: 20 do -> (20/360) * 100 * 10000 = 55556 pulse
 */

#define HOMING_DEG_TO_PULSES(deg) \
  ((uint32_t)((deg) / 360.0f * HOMING_GEAR_RATIO * HOMING_PPR))

/* Thoi gian poll STEPIOC_STATUS giua cac lan kiem tra */

#define HOMING_STATUS_POLL_US       5000UL    /* 5ms */

/* Chu ky "thuc day" khi cho limit event o buoc 2 - moi lan het thoi
 * gian nay ma chua co event, task se tu kiem tra homing_is_aborted()
 * roi cho tiep, thay vi cho vo han tren semaphore.
 */

#define HOMING_ABORT_POLL_MS         200

/* Goc hoat dong ban dau (do) - tinh TU MOC 0 DO (khong phai tu limit
 * switch). Dung chung cho ca 3 truc. Sau khi homing xong, vi tri that
 * cua truc = HOMING_ACTIVE_DEG do trong he toa do moi (0 do = limit
 * down + margin cua tung truc, xem g_homing_margin_deg[] ben duoi).
 */

#define HOMING_ACTIVE_DEG            30.0f

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Margin (do) tu limit_down den moc 0 do - RIENG cho tung truc, dung
 * de calib phan cung (vi du sai so co khi giua cac truc khac nhau).
 * Dung boi motor_pos_capture_zero() khi chot moc 0.
 */

static const float g_homing_margin_deg[HOMING_MOTOR_COUNT] =
{
  15.0f,   /* motor 0 */
  15.0f,   /* motor 1 */
  16.0f,   /* motor 2 */
};

/* Goc that su can nang len TU LIMIT_DOWN (khong phai tu moc 0 do) de
 * dua truc den dung vi tri hoat dong ban dau = margin[i] + HOMING_ACTIVE_DEG.
 * Tinh trong homing_task_main() vi margin khac nhau tung truc, khong
 * the dung mot gia tri chung cho ca ba truc.
 */

/****************************************************************************
 * Name: homing_is_aborted
 *
 * Description:
 *   True neu system_state khong con o SYS_STATE_HOMING nua (vi du bi
 *   EMERGENCY chuyen sang SYS_STATE_ESTOP giua chung). Dung o ca buoc
 *   2 (cho limit) va buoc 3 (cho lift) de thoat som thay vi tiep tuc
 *   nhu khong co gi xay ra.
 ****************************************************************************/

static bool homing_is_aborted(void)
{
  return system_state_get() != SYS_STATE_HOMING;
}

/****************************************************************************
 * Name: open_step_dev
 *
 * Description:
 *   Mo /dev/stepN tuong ung motor_id (0->step0, 1->step1, 2->step2).
 *   Tra ve fd >= 0 neu thanh cong, < 0 neu loi.
 ****************************************************************************/

static int open_step_dev(int motor_id)
{
  char path[16];
  int  fd;

  snprintf(path, sizeof(path), "/dev/step%d", motor_id);

  fd = open(path, O_RDWR);

  if (fd < 0)
    {
      printf("[HOMING] ERROR: khong mo duoc %s (errno=%d)\n", path, errno);
      fflush(stdout);
    }

  return fd;
}

/****************************************************************************
 * Name: send_home
 *
 * Description:
 *   Gui STEPIOC_HOME cho 1 truc. Return true neu thanh cong.
 ****************************************************************************/

static bool send_home(int motor_id)
{
  struct step_home_s home;
  int                fd;
  int                ret;

  fd = open_step_dev(motor_id);
  if (fd < 0)
    {
      return false;
    }

  home.dir_up  = false;           /* xoay xuong tim LIMIT_DOWN */
  home.freq_hz = HOMING_FREQ_HZ;

  if (!safety_is_motor_allowed(motor_id, SAFETY_DIR_DOWN))
    {
      printf("[HOMING] ERROR: safety khong cho phep STEPIOC_HOME "
             "motor=%d\n", motor_id);
      fflush(stdout);
      close(fd);
      return false;
    }

  ret = ioctl(fd, STEPIOC_HOME, (unsigned long)&home);
  close(fd);

  if (ret < 0)
    {
      printf("[HOMING] ERROR: STEPIOC_HOME that bai motor=%d (errno=%d)\n",
             motor_id, errno);
      fflush(stdout);
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: send_lift
 *
 * Description:
 *   Gui STEPIOC_MOVE nang mot truc theo so xung da tinh.
 ****************************************************************************/

static bool send_lift(int motor_id, uint32_t pulses)
{
  struct step_move_s move;
  int                fd;
  int                ret;

  fd = open_step_dev(motor_id);
  if (fd < 0)
    {
      return false;
    }

  move.dir_up  = true;
  move.pulses  = pulses;
  move.freq_hz = HOMING_LIFT_FREQ_HZ;

  if (!safety_is_motor_allowed(motor_id,
                               move.dir_up ? SAFETY_DIR_UP :
                                             SAFETY_DIR_DOWN))
    {
      printf("[HOMING] ERROR: safety khong cho phep STEPIOC_MOVE "
             "(lift) motor=%d\n", motor_id);
      fflush(stdout);
      close(fd);
      return false;
    }

  ret = ioctl(fd, STEPIOC_MOVE, (unsigned long)&move);
  close(fd);

  if (ret < 0)
    {
      printf("[HOMING] ERROR: STEPIOC_MOVE (lift) that bai motor=%d "
             "(errno=%d)\n", motor_id, errno);
      fflush(stdout);
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: is_step_busy
 *
 * Description:
 *   Kiem tra motor_id co dang chay khong qua STEPIOC_STATUS.
 *   Tra ve true neu dang busy, false neu xong hoac loi.
 ****************************************************************************/

static bool is_step_busy(int motor_id)
{
  bool is_busy;
  int  fd;
  int  ret;

  fd = open_step_dev(motor_id);
  if (fd < 0)
    {
      return false;   /* coi nhu xong neu khong mo duoc */
    }

  ret = ioctl(fd, STEPIOC_STATUS, (unsigned long)&is_busy);
  close(fd);

  if (ret < 0)
    {
      return false;
    }

  return is_busy;
}

/****************************************************************************
 * Name: wait_all_lift_done
 *
 * Description:
 *   Poll STEPIOC_STATUS cua ca 3 truc den khi tat ca bao cao khong
 *   busy, HOAC den khi homing_is_aborted() tra ve true (EMERGENCY
 *   ngat giua chung - STEPIOC_ESTOP da tu tat xung, cac truc se tu
 *   het busy rat nhanh, nhung ham nay van chu dong thoat som thay vi
 *   cho vong lap kiem tra busy tu phat hien ra).
 *
 * Returned Value:
 *   true neu ca 3 truc da xong binh thuong (khong bi ngat).
 *   false neu bi ngat giua chung - caller KHONG duoc coi day la
 *   homing thanh cong.
 ****************************************************************************/

static bool wait_all_lift_done(void)
{
  bool any_busy;
  int  i;

  do
    {
      if (homing_is_aborted())
        {
          return false;
        }

      any_busy = false;

      for (i = 0; i < HOMING_MOTOR_COUNT; i++)
        {
          if (is_step_busy(i))
            {
              any_busy = true;
              break;
            }
        }

      if (any_busy)
        {
          usleep(HOMING_STATUS_POLL_US);
        }
    }
  while (any_busy);

  return true;
}

/****************************************************************************
 * Name: cut_all_son
 *
 * Description:
 *   Cat chan SON ca 3 truc sau khi lift hoan tat BINH THUONG (dem du
 *   so xung muc tieu qua slave timer). Trong truong hop nay khong co
 *   limit switch nao bi cham, nen duong tu dong cat SON trong
 *   stm32_steppulse_notify_limit() khong duoc kich hoat - phai cat
 *   thu cong tai day bang STEPIOC_SON_OFF cho tung truc.
 *
 *   KHONG goi ham nay tren nhanh bi EMERGENCY/abort - nhanh do da di
 *   qua STEPIOC_ESTOP (safety_task) cat SON ca 3 truc roi.
 ****************************************************************************/

static void cut_all_son(void)
{
  int i;
  int fd;

  for (i = 0; i < HOMING_MOTOR_COUNT; i++)
    {
      fd = open_step_dev(i);
      if (fd < 0)
        {
          continue;
        }

      if (ioctl(fd, STEPIOC_SON_OFF, 0) < 0)
        {
          printf("[HOMING] ERROR: STEPIOC_SON_OFF that bai motor=%d "
                 "(errno=%d)\n", i, errno);
          fflush(stdout);
        }

      close(fd);
    }
}

/****************************************************************************
 * Name: wait_motor_pos_fresh_for_zero
 *
 * Description:
 *   Sau khoang cho ban dau, kiem tra du lieu encoder da duoc Modbus cap
 *   nhat gan day chua. Retry ngan de tang co hoi lay dung mau moi; neu van
 *   stale thi caller van capture zero theo hanh vi cu.
 ****************************************************************************/

static bool wait_motor_pos_fresh_for_zero(int motor_id)
{
  int retry;

  for (retry = 0; retry < HOMING_ZERO_FRESH_RETRY_COUNT; retry++)
    {
      if (motor_pos_is_fresh(motor_id, HOMING_ZERO_FRESH_MAX_AGE_MS))
        {
          return true;
        }

      if (retry + 1 < HOMING_ZERO_FRESH_RETRY_COUNT)
        {
          usleep(HOMING_ZERO_FRESH_RETRY_US);
        }
    }

  printf("[HOMING] WARNING: motor=%d motor_pos van stale sau %d lan "
         "kiem tra, van capture zero\n",
         motor_id, HOMING_ZERO_FRESH_RETRY_COUNT);
  fflush(stdout);
  return false;
}

/****************************************************************************
 * Name: homing_task_main
 *
 * Description:
 *   Thuc hien chu trinh tim limit duoi, chot moc encoder, nang ba truc
 *   den goc hoat dong va cap nhat system_state khi hoan tat.
 *
 * Input Parameters:
 *   argc, argv - Khong su dung.
 *
 * Returned Value:
 *   0 khi homing hoan tat; -1 neu bi abort.
 ****************************************************************************/

int homing_task_main(int argc, char *argv[])
{
  uint32_t lift_pulses[HOMING_MOTOR_COUNT];
  int      i;
  int      need_event_count;
  int      motor_id;
  bool     is_up;
  bool     limit_down;

  (void)argc;
  (void)argv;

  for (i = 0; i < HOMING_MOTOR_COUNT; i++)
    {
      /* Goc that su can nang TU LIMIT_DOWN = margin (den moc 0 do) +
       * HOMING_ACTIVE_DEG (tu moc 0 do den vi tri hoat dong ban dau).
       * Vi vay khong dung truc tiep HOMING_ACTIVE_DEG lam goc nang.
       */

      float lift_deg_from_limit = g_homing_margin_deg[i] + HOMING_ACTIVE_DEG;

      lift_pulses[i] = HOMING_DEG_TO_PULSES(lift_deg_from_limit);

      printf("[HOMING] motor=%d: margin=%.2f do + active=%.2f do "
             "= nang %.2f do tu limit -> %lu pulse\n",
             i, (double)g_homing_margin_deg[i], (double)HOMING_ACTIVE_DEG,
             (double)lift_deg_from_limit, (unsigned long)lift_pulses[i]);
      fflush(stdout);
    }

  printf("[HOMING] Bat dau homing ca 3 truc\n");
  fflush(stdout);

  /*------------------------------------------------------------------------
   * Buoc 0: Kiem tra trang thai limit GPIO hien tai truoc khi gui lenh.
   * Xu ly ngay neu co truc da chạm LIMIT_DOWN tu truoc khi cap nguon.
   *------------------------------------------------------------------------*/

  need_event_count = 0;

  for (i = 0; i < HOMING_MOTOR_COUNT; i++)
    {
      limit_down = motorlimit_read_hw(i, false);

      if (limit_down)
        {
          printf("[HOMING] motor=%d da chạm LIMIT_DOWN tu truoc (skip home)\n", i);
          fflush(stdout);

          stm32_steppulse_notify_limit(i, false, true);
          system_state_set_limit_reached(i, true);

          /* Truc dung yen tu truoc khi cap nguon - van can cho de modbus_task
          * co it nhat 1 vong doc moi truoc khi lay lam moc 0 do.
          */
          usleep(HOMING_ZERO_CAPTURE_DELAY_US);
          wait_motor_pos_fresh_for_zero(i);

          if (motor_pos_capture_zero(i, g_homing_margin_deg[i]) < 0)
            {
              printf("[HOMING] ERROR: motor=%d capture zero that bai "
                    "(motor_pos chua co du lieu hop le)\n", i);
              fflush(stdout);
            }
        }
      else
        {
          /* Motor chua chạm - can gui STEPIOC_HOME va doi event */

          need_event_count++;
        }
    }

  /*------------------------------------------------------------------------
   * Buoc 1: Gui STEPIOC_HOME cho cac truc CHUA chạm limit.
   *------------------------------------------------------------------------*/

  for (i = 0; i < HOMING_MOTOR_COUNT; i++)
    {
      if (homing_is_aborted())
        {
          printf("[HOMING] Bi ngat khi dang gui STEPIOC_HOME - thoat\n");
          fflush(stdout);
          return -1;
        }

      if (!system_state_is_limit_reached(i))
        {
          printf("[HOMING] motor=%d: gui STEPIOC_HOME\n", i);
          fflush(stdout);
          send_home(i);
        }
    }

  /*------------------------------------------------------------------------
   * Buoc 2: Doi event LIMIT_DOWN cho dung so truc can doi, CO TIMEOUT
   * de tu phat hien bi ngat (EMERGENCY) thay vi treo vo han.
   * Bo qua: event LIMIT_UP, event LIMIT_DOWN cua truc da xu ly roi.
   *------------------------------------------------------------------------*/

  printf("[HOMING] Cho %d truc chạm LIMIT_DOWN...\n", need_event_count);
  fflush(stdout);

  while (need_event_count > 0)
    {
      struct timespec ts;
      int             ret;

      clock_gettime(CLOCK_REALTIME, &ts);

      ts.tv_nsec += (long)HOMING_ABORT_POLL_MS * 1000000L;
      if (ts.tv_nsec >= 1000000000L)
        {
          ts.tv_sec  += ts.tv_nsec / 1000000000L;
          ts.tv_nsec  = ts.tv_nsec % 1000000000L;
        }

      ret = motorlimit_timedwaitevent_id(&ts, &motor_id, &is_up);

      if (ret < 0)
        {
          /* Het thoi gian cho (hoac loi khac) - kiem tra co bi ngat
           * (EMERGENCY) khong. Neu khong, chi la timeout binh thuong,
           * quay lai cho tiep.
           */

          if (homing_is_aborted())
            {
              printf("[HOMING] Bi ngat (EMERGENCY) trong luc cho limit "
                     "- thoat, KHONG danh dau homing hoan tat\n");
              fflush(stdout);
              return -1;
            }

          continue;
        }

      if (is_up)
        {
          /* LIMIT_UP event - bo qua trong phase homing */

          continue;
        }

      if (system_state_is_limit_reached(motor_id))
        {
          /* LIMIT_DOWN cua truc nay da xu ly roi (event trung lap) */

          continue;
        }

      printf("[HOMING] motor=%d chạm LIMIT_DOWN\n", motor_id);
      fflush(stdout);

      system_state_set_limit_reached(motor_id, true);
      need_event_count--;


      /* Cho modbus_task doc it nhat 1 vong moi kip vi tri sau khi truc dung,
      * roi moi lay lam moc 0 do cho truc nay.
      */
      usleep(HOMING_ZERO_CAPTURE_DELAY_US);
      wait_motor_pos_fresh_for_zero(motor_id);

      if (motor_pos_capture_zero(motor_id, g_homing_margin_deg[motor_id]) < 0)
        {
          printf("[HOMING] ERROR: motor=%d capture zero that bai "
                "(motor_pos chua co du lieu hop le)\n", motor_id);
          fflush(stdout);
        }
    }

  printf("[HOMING] Ca 3 truc da chạm LIMIT_DOWN - bat dau nang len\n");
  fflush(stdout);

  /*------------------------------------------------------------------------
   * Buoc 3: Nang ca 3 truc len dong thoi.
   *------------------------------------------------------------------------*/

  for (i = 0; i < HOMING_MOTOR_COUNT; i++)
    {
      if (homing_is_aborted())
        {
          printf("[HOMING] Bi ngat khi dang gui STEPIOC_MOVE (lift) - thoat\n");
          fflush(stdout);
          return -1;
        }

      printf("[HOMING] motor=%d: nang len %lu pulse\n",
            i, (unsigned long)lift_pulses[i]);
      fflush(stdout);
      send_lift(i, lift_pulses[i]);
    }

  /* Doi ca 3 truc nang xong - hoac bi ngat giua chung */

  if (!wait_all_lift_done())
    {
      printf("[HOMING] Bi ngat (EMERGENCY) trong luc nang len - thoat, "
             "KHONG danh dau homing hoan tat\n");
      fflush(stdout);
      return -1;
    }

  printf("[HOMING] Ca 3 truc da nang len xong - homing hoan tat\n");
  fflush(stdout);

  /* Lift dung binh thuong (khong cham limit switch) - duong tu dong
   * cat SON trong stm32_steppulse_notify_limit() khong duoc kich
   * hoat, phai chu dong cat tai day truoc khi bao homing hoan tat.
   */

  cut_all_son();

  printf("[HOMING] Da cat SON ca 3 truc\n");
  fflush(stdout);

  /*------------------------------------------------------------------------
   * Buoc 4: Cap nhat system state, bao safety_task/system_state biet
   * homing da hoan tat thanh cong.
   *------------------------------------------------------------------------*/

  for (i = 0; i < HOMING_MOTOR_COUNT; i++)
    {
      system_state_set_homed(i, true);
    }

  system_state_notify_homing_complete();

  printf("[HOMING] State -> WAIT_START. Cho nhan nut START lan 2.\n");
  fflush(stdout);

  return 0;
}
