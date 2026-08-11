/****************************************************************************
 * common/homing_task/homing_task.c
 *
 * Quy trinh homing 3 truc dong co:
 *
 *   Buoc 0 - Kiem tra trang thai limit GPIO ngay khi khoi dong.
 *            Neu motor da chạm LIMIT_DOWN truoc khi cap nguon (khong co
 *            falling edge -> khong co ISR -> khong co event), xu ly ngay
 *            ma khong can doi event.
 *
 *   Buoc 1 - Gui STEPIOC_HOME (dir_up=false) cho cac truc CHUA chạm.
 *            Motor tu chay xuong cham, ISR tu cat xung khi chạm.
 *
 *   Buoc 2 - Doi event LIMIT_DOWN (co timeout) cho dung so truc can
 *            doi. Khi du ca 3 truc da chạm limit thi sang buoc 3.
 *
 *   Buoc 3 - Gui STEPIOC_MOVE (dir_up=true, pulse tinh tu goc nang)
 *            dong thoi cho ca 3 truc. Poll STEPIOC_STATUS den khi ca 3
 *            xong.
 *
 *   Buoc 4 - Set homed[0..2]=true, thong bao system_state homing da
 *            hoan tat (chuyen HOMING -> WAIT_START). Task tu thoat,
 *            cho safety_task xu ly nut START lan 2.
 *
 * Edge case da xu ly:
 *   - 1 hoac nhieu truc da chạm limit truoc khi cap nguon.
 *   - Event LIMIT_UP den trong luc dang doi LIMIT_DOWN (bi bo qua).
 *   - Event LIMIT_DOWN trung lap cua cung 1 truc (chi xu ly lan dau).
 *   - EMERGENCY duoc nhan giua chung (system_state chuyen HOMING ->
 *     ESTOP): task tu phat hien qua homing_is_aborted(), THOAT SOM
 *     thay vi treo vo han o buoc 2, va KHONG danh dau homed[]=true /
 *     KHONG bao homing da hoan tat neu bi ngat giua chung o buoc 2
 *     hoac buoc 3. STEPIOC_ESTOP (goi tu safety_task khi EMERGENCY) da
 *     tu tat xung o tang driver, o day chi can task nay tu biet duong
 *     ma thoat, khong con viec gi phai lam them de dam bao an toan.
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HOMING_MOTOR_COUNT          MOTOR_COUNT
#define HOMING_FREQ_HZ              50000UL   /* tan so xung khi homing */
#define HOMING_LIFT_FREQ_HZ         50000UL   /* tan so xung khi nang len */

#define HOMING_LIFT_DEFAULT_DEG     30.0f     /* goc nang mac dinh (do) */
#define HOMING_GEAR_RATIO           100.0f
#define HOMING_PPR                  10000.0f

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

 static const float g_homing_lift_deg[HOMING_MOTOR_COUNT] =
{
  30.0f,   /* motor 0 */
  30.0f,   /* motor 1 */
  30.0f,   /* motor 2 */
};

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
 *   Gui STEPIOC_MOVE di len ~20 do cho 1 truc. Return true neu thanh cong.
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
 * Public Functions
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
  (void)argv;   /* khong con dung argv nua, doc truc tiep tu g_homing_lift_deg */

  for (i = 0; i < HOMING_MOTOR_COUNT; i++)
    {
      lift_pulses[i] = HOMING_DEG_TO_PULSES(g_homing_lift_deg[i]);

      printf("[HOMING] motor=%d: goc nang=%.2f do -> %lu pulse\n",
             i, (double)g_homing_lift_deg[i], (unsigned long)lift_pulses[i]);
      fflush(stdout);
    }

  printf("[HOMING] Bat dau homing ca 3 truc\n");
  fflush(stdout);

  /* Khong con tu goi system_state_reset_homing()/set_phase() o day nua
   * - system_state_handle_btn_event() (goi boi safety_task truoc khi
   * spawn task nay) da tu reset homing progress va chuyen state sang
   * SYS_STATE_HOMING roi, nen luc task nay bat dau chay, state da
   * dung san.
   */

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
          /* Motor nay da chạm LIMIT_DOWN - khong co falling edge nao se
           * den nua cho trang thai nay. Xu ly ngay nhu ISR da noi.
           */

          printf("[HOMING] motor=%d da chạm LIMIT_DOWN tu truoc (skip home)\n",
                 i);
          fflush(stdout);

          /* Khoa chieu xuong (giong ISR lam) de steppulse biet - can
           * thiet vi khong co edge nao xay ra de ISR tu lam viec nay.
           */

          stm32_steppulse_notify_limit(i, false, true);

          system_state_set_limit_reached(i, true);
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
    }

  printf("[HOMING] Ca 3 truc da chạm LIMIT_DOWN - bat dau nang len\n");
  fflush(stdout);

  /*------------------------------------------------------------------------
   * Buoc 3: Nang ca 3 truc len dong thoi.
   *------------------------------------------------------------------------*/

  for (i = 0; i < HOMING_MOTOR_COUNT; i++)
    {
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