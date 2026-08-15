/****************************************************************************
 * payloads/stewart/stewart_main.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdbool.h>
#include <time.h>

#include "motor_pos.h"
#include "system_state.h"
#include "homing_task.h"
#include "safety_task/safety_task.h"
#include "step_ioctl.h"
#include "pwm_capture_ioctl.h"   /* PWMCAPIOC_GET, struct pwmcap_result_s */
#include "motion_task.h"
#include "modbus_task.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TEST_GEAR_RATIO           100
#define TEST_MOTOR_PPR            50000
#define TEST_PULSES_PER_OUT_REV   (TEST_MOTOR_PPR * TEST_GEAR_RATIO) /* 1 vong truc ra = 1,000,000 xung */
#define TEST_FREQ_HZ              500000
#define TEST_PWM_PERIOD_NS   10000000L 

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Ra lenh quay dung 1 vong truc ra (sau hop so 1:100) tren ca 3 dong co,
 * o tan so TEST_FREQ_HZ, de kiem tra mat xung.
 */

static int test_spin_one_rev(void)
{
  static const char * const devpath[3] =
  {
    "/dev/step0", "/dev/step1", "/dev/step2"
  };

  int fd[3];
  int ret;
  int i;

  for (i = 0; i < 3; i++)
    {
      fd[i] = open(devpath[i], O_RDWR);
      if (fd[i] < 0)
        {
          printf("[TEST] mo %s that bai: %d\n", devpath[i], errno);

          while (--i >= 0)
            {
              close(fd[i]);
            }

          return -errno;
        }
    }

  struct step_move_s mv =
  {
    .dir_up  = true,
    .pulses  = TEST_PULSES_PER_OUT_REV,
    .freq_hz = TEST_FREQ_HZ,
  };

  /* Ra lenh MOVE lien tiep khong delay de ca 3 truc bat dau gan nhu
   * dong thoi.
   */

  for (i = 0; i < 3; i++)
    {
      ret = ioctl(fd[i], STEPIOC_MOVE, (unsigned long)&mv);
      if (ret < 0)
        {
          printf("[TEST] STEPIOC_MOVE fd[%d] that bai: %d\n", i, errno);
        }
    }

  printf("[TEST] da ra lenh quay 1 vong truc ra "
         "(%lu xung @ %u Hz) cho ca 3 truc\n",
         (unsigned long)TEST_PULSES_PER_OUT_REV, TEST_FREQ_HZ);
  fflush(stdout);

  /* Cho ca 3 truc bao done. STEPIOC_STATUS tra bool qua con tro arg -
   * gia dinh true = dang chay (busy), false = da xong. Dao lai neu
   * driver dinh nghia nguoc.
   */

  bool busy[3] = { true, true, true };

  while (busy[0] || busy[1] || busy[2])
    {
      for (i = 0; i < 3; i++)
        {
          if (busy[i])
            {
              ret = ioctl(fd[i], STEPIOC_STATUS, (unsigned long)&busy[i]);
              if (ret < 0)
                {
                  printf("[TEST] STEPIOC_STATUS fd[%d] that bai: %d\n",
                         i, errno);
                  busy[i] = false;
                }
            }
        }

      usleep(5000);
    }

  printf("[TEST] ca 3 truc da bao done\n");
  fflush(stdout);

  for (i = 0; i < 3; i++)
    {
      close(fd[i]);
    }

  return OK;
}


/* Doc /dev/pwmcap0..2 lien tuc o 100Hz, in ra pulse_width_us/period_us/
 * valid/stale cua ca 3 kenh. Dung clock_nanosleep absolute-wake giong
 * pattern se dung trong motion_task sau nay - test luon co che nay.
 * Chay vo han, Ctrl+C / power-cycle de dung (chi la code test).
 */

static int test_read_pwm(void)
{
  static const char * const devpath[3] =
    {
      "/dev/pwmcap0", "/dev/pwmcap1", "/dev/pwmcap2"
    };

  int             fd[3];
  int             ret;
  int             i;
  struct timespec next;

  for (i = 0; i < 3; i++)
    {
      fd[i] = open(devpath[i], O_RDWR);
      if (fd[i] < 0)
        {
          printf("[TEST-PWM] mo %s that bai: %d\n", devpath[i], errno);

          while (--i >= 0)
            {
              close(fd[i]);
            }

          return -errno;
        }
    }

  printf("[TEST-PWM] bat dau doc PWM @ 100Hz tren ca 3 kenh\n");
  fflush(stdout);

  clock_gettime(CLOCK_MONOTONIC, &next);

  for (; ; )
    {
      struct pwmcap_result_s res[3];

      for (i = 0; i < 3; i++)
        {
          ret = ioctl(fd[i], PWMCAPIOC_GET, (unsigned long)&res[i]);
          if (ret < 0)
            {
              printf("[TEST-PWM] PWMCAPIOC_GET fd[%d] that bai: %d\n",
                     i, errno);
              continue;
            }
        }

      printf("[TEST-PWM] "
             "ch0(pw=%lu us, T=%lu us, valid=%d, stale=%d) | "
             "ch1(pw=%lu us, T=%lu us, valid=%d, stale=%d) | "
             "ch2(pw=%lu us, T=%lu us, valid=%d, stale=%d)\n",
             (unsigned long)res[0].pulse_width_us,
             (unsigned long)res[0].period_us,
             (int)res[0].valid, (int)res[0].stale,
             (unsigned long)res[1].pulse_width_us,
             (unsigned long)res[1].period_us,
             (int)res[1].valid, (int)res[1].stale,
             (unsigned long)res[2].pulse_width_us,
             (unsigned long)res[2].period_us,
             (int)res[2].valid, (int)res[2].stale);
      fflush(stdout);

      next.tv_nsec += TEST_PWM_PERIOD_NS;
      if (next.tv_nsec >= 1000000000L)
        {
          next.tv_sec  += next.tv_nsec / 1000000000L;
          next.tv_nsec  = next.tv_nsec % 1000000000L;
        }

      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

  /* khong bao gio toi day - vong lap vo han */

  for (i = 0; i < 3; i++)
    {
      close(fd[i]);
    }

  return OK;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stewart_payload_main(int argc, FAR char *argv[])
{
  printf("[STEWART] khoi dong\n");

  motor_pos_init();
  system_state_init();

  modbus_task_start();
  safety_task_initialize();


  motion_task_start();          /* them dong nay */

  // test_spin_one_rev();
  
  for (;;)
    {
      usleep(1000000);
    }

  return OK;
}