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

#include "motor_pos.h"
#include "system_state.h"
#include "homing_task.h"
#include "safety_task/safety_task.h"
#include "step_ioctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TEST_GEAR_RATIO           100
#define TEST_MOTOR_PPR            10000
#define TEST_PULSES_PER_OUT_REV   (TEST_MOTOR_PPR * TEST_GEAR_RATIO) /* 1 vong truc ra = 1,000,000 xung */
#define TEST_FREQ_HZ              500000

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stewart_payload_main(int argc, FAR char *argv[])
{
  FAR char * const homing_argv[5] =
  {
    "homing_task",
    argc > 1 ? argv[1] : "20.0",
    argc > 2 ? argv[2] : "20.0",
    argc > 3 ? argv[3] : "20.0",
    NULL
  };

  printf("[STEWART] khoi dong\n");
  printf("[STEWART] goc nang: M0=%s M1=%s M2=%s do\n",
         homing_argv[1], homing_argv[2], homing_argv[3]);
  fflush(stdout);

  /* 1. Khoi tao state truoc - homing_task se doc/ghi vao day */
  // motor_pos_init();
  // system_state_init();

  // safety_task_initialize();

  /* Test: quay 1 vong truc ra @ 100kHz tren ca 3 dong co de kiem tra
   * mat xung. Thay cho luong hoat dong binh thuong trong luc test.
   */

  test_spin_one_rev();

  for (;;)
    {
      usleep(1000000);
    }

  return OK;
}