/****************************************************************************
 * common/motion_task/motion_task.c
 *
 * Runs a 100 Hz control loop: reads PWM setpoint, computes position error,
 * and sends STEPIOC_MOVE commands when system_state permits.
 *
 * Design notes:
 * - PWM range: 1000-2000 µs maps to 0-MOTION_ANGLE_MAX_DEG
 * - Step frequency: 100 kHz (constant for all three motors; TIM3 is shared)
 * - Only updates when motor_pos feedback is fresh (detects stalled Modbus)
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <math.h>

#include "system_state.h"
#include "step_ioctl.h"
#include "pwm_capture_ioctl.h"
#include "motor_pos.h"
#include "safety_task/safety_task.h"
#include "stm32_sensorbtn.h"   /* MOTOR_COUNT */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MOTION_MOTOR_COUNT       MOTOR_COUNT

#define MOTION_PERIOD_NS         100000000L   /* 100Hz = 10ms */

#define MOTION_PWM_MIN_US        1000.0f
#define MOTION_PWM_MAX_US        2000.0f
#define MOTION_ANGLE_MAX_DEG     60.0f       /* tam thoi - se cap nhat
                                               * sau khi do dac thuc te */

/* Tan so xung khi RUNNING - dung 1 gia tri co dinh chung cho ca 3
 * truc (TIM3 dung chung 1 period, xem stm32_steppulse.c). Theo dung
 * dai toc do da chot cho du   (300-500kHz), chon 400kHz lam mac
 * dinh - doi lai o day neu can so khac.
 */

#define MOTION_FREQ_MIN_HZ         1000UL
#define MOTION_FREQ_MAX_HZ         50000UL
#define MOTION_PULSES_FULL_SPEED   23500UL 

#define MOTION_TASK_PRIORITY     150   /* duoi safety_task, tren modbus_task */
#define MOTION_DEBUG_MOTOR_ID    1

#define MOTION_DEADBAND_PULSES   1500   /* bat dau tu day, tang dan den khi het dao dong */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_t g_motion_thread;
static int        g_step_fd[MOTION_MOTOR_COUNT];
static int        g_pwmcap_fd[MOTION_MOTOR_COUNT];
static clock_t g_last_pos_tick[MOTION_MOTOR_COUNT];   /* khoi tao 0 */
static float g_pulse_error[MOTION_MOTOR_COUNT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motion_pwm_to_deg
 *
 * Description:
 *   Quy doi pulse_width_us tu PWM capture sang goc hoat dong. Clamp ket
 *   qua vao [0, MOTION_ANGLE_MAX_DEG] de tranh lenh vuot bien khi PWM
 *   nam ngoai dai chuan.
 *
 * Returned Value:
 *   Goc dat theo don vi do.
 ****************************************************************************/

static float motion_pwm_to_deg(uint32_t pulse_width_us)
{
  float deg;

  deg = ((float)pulse_width_us - MOTION_PWM_MIN_US)
        / (MOTION_PWM_MAX_US - MOTION_PWM_MIN_US)
        * MOTION_ANGLE_MAX_DEG;

  if (deg < 0.0f)
    {
      deg = 0.0f;
    }
  else if (deg > MOTION_ANGLE_MAX_DEG)
    {
      deg = MOTION_ANGLE_MAX_DEG;
    }

  return deg;
}

/****************************************************************************
 * Name: motion_process_motor
 *
 * Description:
 *   Xu ly mot truc trong mot chu ky: doc PWM, tinh goc va so xung, kiem
 *   tra safety roi gui STEPIOC_MOVE. Neu mot buoc that bai, truc duoc bo
 *   qua den chu ky sau.
 *
 * Input Parameters:
 *   motor_id - Chi so truc can xu ly.
 ****************************************************************************/

static void motion_process_motor(int motor_id)
{
  struct pwmcap_result_s pwm;
  float                   target_deg;
  int32_t                 pulses;
  bool                    dir_up;
  uint32_t                abs_pulses;
  struct step_move_s      move;
  int                     ret;

  ret = ioctl(g_pwmcap_fd[motor_id], PWMCAPIOC_GET, (unsigned long)&pwm);
  if (ret < 0)
    {
      return;
    }

  // target_deg = 40.0f;

  target_deg = motion_pwm_to_deg(pwm.pulse_width_us);

  clock_t pos_tick = motor_pos_get_update_tick(motor_id);

  if (pos_tick == g_last_pos_tick[motor_id])
    {
      return;   /* chua co du lieu modbus moi, khong tinh/gui lenh lai */
    }

  g_last_pos_tick[motor_id] = pos_tick;

  ret = motor_pos_get_pulses(motor_id, target_deg, &pulses);
  if (ret < 0)
    {
      return;   /* motor_pos chua co du lieu (-EAGAIN) - bo qua chu ky nay */
    }

  printf("[MOTION] motor=%d target_deg=%.2f pulses=%ld\n",
         motor_id, (double)target_deg, (long)pulses);
  fflush(stdout);

  if (pulses == 0)
    {
      return;   /* khong can di - driver cung tu no-op voi pulses=0 */
    }

  dir_up     = (pulses > 0);
  abs_pulses = (uint32_t)(dir_up ? pulses : -pulses);

  g_pulse_error[motor_id] = abs_pulses;

  if (abs_pulses < MOTION_DEADBAND_PULSES)
    {
      ioctl(g_step_fd[motor_id], STEPIOC_SON_OFF, 0);
      return;   /* trong deadband, coi nhu da toi vi tri */
    }

  if (!safety_is_motor_allowed(motor_id,
                                dir_up ? SAFETY_DIR_UP : SAFETY_DIR_DOWN))
    {
      return;
    }

  /* TIM3 chi co 1 ARR dung chung cho ca 3 kenh - freq_hz PHAI tinh
   * theo sai so LON NHAT trong ca 3 truc (rang buoc phan cung), NHUNG
   * khong duoc vuot qua muc HOP LY rieng cho truc nay (dua theo sai so
   * CUA CHINH NO) - neu khong, truc da gan dich se bi ep chay nhanh
   * theo truc khac con xa, gay vot qua setpoint va dao dong.
   */
  {
    uint32_t max_err = g_pulse_error[0];
    uint32_t shared_freq;
    uint32_t own_freq;
    int i;

    for (i = 1; i < MOTION_MOTOR_COUNT; i++)
      {
        if (g_pulse_error[i] > max_err)
          {
            max_err = g_pulse_error[i];
          }
      }

    /* freq theo truc kho nhat - rang buoc phan cung TIM3 dung chung ARR */
    if (max_err >= MOTION_PULSES_FULL_SPEED)
      {
        shared_freq = MOTION_FREQ_MAX_HZ;
      }
    else if (max_err <= MOTION_DEADBAND_PULSES)
      {
        shared_freq = MOTION_FREQ_MIN_HZ;
      }
    else
      {
        // shared_freq = MOTION_FREQ_MIN_HZ +
        //   (uint32_t)((uint64_t)(MOTION_FREQ_MAX_HZ - MOTION_FREQ_MIN_HZ) *
        //               (max_err - MOTION_DEADBAND_PULSES) /
        //               (MOTION_PULSES_FULL_SPEED - MOTION_DEADBAND_PULSES));
        shared_freq = pow((max_err - MOTION_DEADBAND_PULSES), 2) / 10000 + MOTION_FREQ_MIN_HZ;
      }

    /* freq toi da hop ly rieng cho truc nay, dua theo sai so CUA CHINH NO */
    if (abs_pulses >= MOTION_PULSES_FULL_SPEED)
      {
        own_freq = MOTION_FREQ_MAX_HZ;
      }
    else if (abs_pulses <= MOTION_DEADBAND_PULSES)
      {
        own_freq = MOTION_FREQ_MIN_HZ;
      }
    else
      {
        // own_freq = MOTION_FREQ_MIN_HZ +
        //   (uint32_t)((uint64_t)(MOTION_FREQ_MAX_HZ - MOTION_FREQ_MIN_HZ) *
        //               (abs_pulses - MOTION_DEADBAND_PULSES) /
        //               (MOTION_PULSES_FULL_SPEED - MOTION_DEADBAND_PULSES));
        own_freq = pow((abs_pulses - MOTION_DEADBAND_PULSES), 2) / 10000 + MOTION_FREQ_MIN_HZ;
      }

    /* Lay gia tri NHO HON - khong bao gio chay nhanh hon muc ban than
     * truc nay can, du truc khac dang can toc do cao hon.
     */
    move.freq_hz = (shared_freq < own_freq) ? shared_freq : own_freq;

    printf("[MOTION] motor=%d pulses=%ld dir_up=%d freq=%lu "
           "shared_freq=%lu own_freq=%lu max_err=%lu\n",
           motor_id, (long)pulses, (int)dir_up,
           (unsigned long)move.freq_hz, (unsigned long)shared_freq,
           (unsigned long)own_freq, (unsigned long)max_err);
    fflush(stdout);
  }

  move.dir_up = dir_up;
  move.pulses = abs_pulses;

  ioctl(g_step_fd[motor_id], STEPIOC_MOVE, (unsigned long)&move);
}

/****************************************************************************
 * Name: motion_task_main
 *
 * Description:
 *   Mo cac device va chay vong dieu khien theo moc thoi gian tuyet doi
 *   voi chu ky 100 Hz khi he thong o SYS_STATE_RUNNING.
 ****************************************************************************/

static FAR void *motion_task_main(FAR void *arg)
{
  char devpath[16];
  int  i;
  int  ret;

  (void)arg;

  for (i = 0; i < MOTION_MOTOR_COUNT; i++)
    {
      snprintf(devpath, sizeof(devpath), "/dev/step%d", i);
      g_step_fd[i] = open(devpath, O_RDWR);
      if (g_step_fd[i] < 0)
        {
          printf("[MOTION] ERROR: khong mo duoc %s (errno=%d)\n",
                 devpath, errno);
          fflush(stdout);
        }

      snprintf(devpath, sizeof(devpath), "/dev/pwmcap%d", i);
      g_pwmcap_fd[i] = open(devpath, O_RDWR);
      if (g_pwmcap_fd[i] < 0)
        {
          printf("[MOTION] ERROR: khong mo duoc %s (errno=%d)\n",
                 devpath, errno);
          fflush(stdout);
        }
    }

  printf("[MOTION] motion_task bat dau, dan theo chu ky doc modbus\n");
  fflush(stdout);

  for (; ; )
  {
    int motor_id;

    /* Cho den khi co du lieu vi tri moi tu modbus_task, timeout 200ms
     * lam watchdog - neu modbus treo thi khong bi block vinh vien,
     * quay lai vong lap va de safety_task/logic khac xu ly.
     */

    ret = motor_pos_wait_update_id(200, &motor_id);

    if (ret != OK)
      {
        continue;   /* timeout, chua co du lieu moi */
      }

    if (system_state_get() == SYS_STATE_RUNNING)
      {
        motion_process_motor(motor_id);
      }
  }

  return NULL;
}

/****************************************************************************
 * Name: motion_task_start
 *
 * Description:
 *   Tao motion_task va dat ten cung priority cho thread.
 *
 * Returned Value:
 *   OK khi thanh cong; ma loi am neu khong tao duoc thread.
 ****************************************************************************/

int motion_task_start(void)
{
  int               ret;
  struct sched_param param;

  ret = pthread_create(&g_motion_thread, NULL, motion_task_main, NULL);
  if (ret != 0)
    {
      printf("[MOTION] ERROR: pthread_create failed: %d\n", ret);
      fflush(stdout);
      return -ret;
    }

  pthread_setname_np(g_motion_thread, "motion_task");

  param.sched_priority = MOTION_TASK_PRIORITY;
  ret = pthread_setschedparam(g_motion_thread, SCHED_FIFO, &param);
  if (ret != 0)
    {
      printf("[MOTION] WARNING: setschedparam failed: %d\n", ret);
      fflush(stdout);
    }

  return OK;
}
