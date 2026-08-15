/****************************************************************************
 * common/motion_task/motion_task.c
 *
 * Chay vong dieu khien 100 Hz, doc setpoint PWM, tinh sai lech vi tri
 * va gui lenh STEPIOC_MOVE khi system_state cho phep.
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

#define MOTION_PERIOD_NS         10000000L   /* 100Hz = 10ms */

#define MOTION_PWM_MIN_US        1000.0f
#define MOTION_PWM_MAX_US        2000.0f
#define MOTION_ANGLE_MAX_DEG     90.0f       /* tam thoi - se cap nhat
                                               * sau khi do dac thuc te */

/* Tan so xung khi RUNNING - dung 1 gia tri co dinh chung cho ca 3
 * truc (TIM3 dung chung 1 period, xem stm32_steppulse.c). Theo dung
 * dai toc do da chot cho du an (300-500kHz), chon 400kHz lam mac
 * dinh - doi lai o day neu can so khac.
 */
#define MOTION_STEP_FREQ_HZ      100000UL

#define MOTION_TASK_PRIORITY     150   /* duoi safety_task, tren modbus_task */
#define MOTION_DEBUG_MOTOR_ID    1

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_t g_motion_thread;
static int        g_step_fd[MOTION_MOTOR_COUNT];
static int        g_pwmcap_fd[MOTION_MOTOR_COUNT];
static clock_t g_last_pos_tick[MOTION_MOTOR_COUNT];   /* khoi tao 0 */

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

  // target_deg = motion_pwm_to_deg(pwm.pulse_width_us);
  target_deg = 50;

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

  if (motor_id == 0)
    {
      static int dbg_counter = 0;
      if (++dbg_counter % 30 == 0)
        {
          printf("[MOTION dbg] motor=%d target=%.1f pulses=%ld dir=%s\n",
                motor_id, (double)target_deg, (long)pulses,
                (pulses > 0) ? "UP(sau_fix)" : "DOWN(sau_fix)");
          fflush(stdout);
        }
    }

  if (pulses == 0)
    {
      return;   /* khong can di - driver cung tu no-op voi pulses=0 */
    }

  dir_up     = (pulses > 0);
  abs_pulses = (uint32_t)(dir_up ? pulses : -pulses);

  if (!safety_is_motor_allowed(motor_id,
                                dir_up ? SAFETY_DIR_UP : SAFETY_DIR_DOWN))
    {
      return;
    }

  move.dir_up  = dir_up;
  move.pulses  = abs_pulses;
  move.freq_hz = MOTION_STEP_FREQ_HZ;

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
  char            devpath[16];
  int             i;
  struct timespec next;

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

  printf("[MOTION] motion_task bat dau, 100Hz\n");
  fflush(stdout);

  clock_gettime(CLOCK_MONOTONIC, &next);

  for (; ; )
    {
      if (system_state_get() == SYS_STATE_RUNNING)
        {
          for (i = 0; i < MOTION_MOTOR_COUNT; i++)
            {
              motion_process_motor(i);
            }
        }

      next.tv_nsec += MOTION_PERIOD_NS;
      if (next.tv_nsec >= 1000000000L)
        {
          next.tv_sec  += next.tv_nsec / 1000000000L;
          next.tv_nsec  = next.tv_nsec % 1000000000L;
        }

      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
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
