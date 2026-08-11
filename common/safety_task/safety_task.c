/****************************************************************************
 * common/safety_task/safety_task.c
 *
 * Xem safety_task.h de biet vai tro cua file nay trong kien truc moi -
 * toan bo state/business-logic nam trong common/system_state/.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <syslog.h>
#include <sys/ioctl.h>

#include "safety_task.h"
#include "system_state.h"     /* sys_state_t, sys_action_t, system_state_* */
#include "stm32_sensorbtn.h"  /* motorbtn_waitevent(), motorlimit_get() */
#include "step_ioctl.h"       /* STEPIOC_ESTOP */
#include "homing_task.h"      /* homing_task_main() */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SAFETY_STEP_DEV_FMT  "/dev/step%d"

#define SAFETY_HOMING_TASK_NAME       "homing_task"
#define SAFETY_HOMING_TASK_PRIORITY   100
#define SAFETY_HOMING_TASK_STACKSIZE  2048

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_t g_safety_thread;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Hard-cut ca 3 motor qua STEPIOC_ESTOP. Best-effort: log nhung khong
 * dung neu 1 device loi/khong mo duoc, muon "cham" duoc het cac truc
 * co the ngay ca khi 1 kenh dang gap van de.
 */

static void safety_hard_stop_all(void)
{
  char devpath[16];
  int  fd;
  int  i;

  for (i = 0; i < SAFETY_NUM_MOTORS; i++)
    {
      snprintf(devpath, sizeof(devpath), SAFETY_STEP_DEV_FMT, i);

      fd = open(devpath, O_RDWR);
      if (fd < 0)
        {
          syslog(LOG_ERR, "safety_task: failed to open %s for ESTOP\n",
                 devpath);
          continue;
        }

      if (ioctl(fd, STEPIOC_ESTOP, 0) < 0)
        {
          syslog(LOG_ERR, "safety_task: STEPIOC_ESTOP failed on %s\n",
                 devpath);
        }

      close(fd);
    }
}

/* Spawn homing_task khong tham so rieng -> homing_task_main() se tu
 * dung goc nang mac dinh cho ca 3 truc (xem homing_task.h).
 *
 * Luu y: khong co co che chong spawn-trung o day - theo dung bang
 * transition trong system_state.c thi SYS_ACTION_SPAWN_HOMING chi
 * duoc tra ve tu IDLE hoac STOPPED/ESTOP/FAULT (khong bao gio tu
 * chinh HOMING), nen ve mat logic khong the co 2 lan spawn chong
 * nhau tu duong nut bam. Neu sau nay co them nguon nao khac cung co
 * the trigger SYS_ACTION_SPAWN_HOMING thi can bo sung guard o day.
 */

static void safety_spawn_homing_task(void)
{
  int pid;

  pid = task_create(SAFETY_HOMING_TASK_NAME, SAFETY_HOMING_TASK_PRIORITY,
                     SAFETY_HOMING_TASK_STACKSIZE, homing_task_main, NULL);

  if (pid < 0)
    {
      syslog(LOG_ERR, "safety_task: failed to spawn homing_task: %d\n",
             pid);
    }
  else
    {
      syslog(LOG_INFO, "safety_task: homing_task spawned (pid=%d)\n", pid);
    }
}

/* Thuc hien I/O side-effect tuong ung voi 1 sys_action_t. Luon duoc
 * goi NGOAI mutex lock cua system_state (system_state_handle_btn_event()
 * / system_state_handle_fault_event() da tra ve TRUOC khi ham nay
 * chay).
 */

static void safety_dispatch_action(sys_action_t action)
{
  switch (action)
    {
      case SYS_ACTION_SPAWN_HOMING:
        safety_spawn_homing_task();
        break;

      case SYS_ACTION_HARD_STOP:
        syslog(LOG_WARNING, "safety_task: hard stop triggered\n");
        safety_hard_stop_all();
        break;

      case SYS_ACTION_NONE:
      default:
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *safety_task_main(void *arg)
{
  int          btn_id;
  int          level;
  sys_action_t action;

  UNUSED(arg);

  syslog(LOG_INFO, "safety_task: started\n");

  for (; ; )
    {
      /* Blocks until a button event occurs. Debounce da xu ly trong
       * ISR o board layer, moi event o day duoc coi la 1 canh sach.
       */

      motorbtn_waitevent(&btn_id, &level);

      action = system_state_handle_btn_event(btn_id, level);

      safety_dispatch_action(action);
    }

  return NULL;
}

int safety_task_initialize(void)
{
  int ret;

  ret = pthread_create(&g_safety_thread, NULL, safety_task_main, NULL);
  if (ret != 0)
    {
      syslog(LOG_ERR, "safety_task: pthread_create failed: %d\n", ret);
      return -ret;
    }

  pthread_setname_np(g_safety_thread, "safety_task");

  return 0;
}

sys_state_t safety_get_state(void)
{
  return system_state_get();
}

bool safety_is_motor_allowed(int motor_id, int direction)
{
  sys_state_t                 state;
  struct motor_limit_state_s  limit;

  if (motor_id < 0 || motor_id >= SAFETY_NUM_MOTORS)
    {
      return false;
    }

  state = system_state_get();

  if (state != SYS_STATE_RUNNING && state != SYS_STATE_HOMING)
    {
      return false;
    }

  /* Luu y: limit switch da hard-cut xung o tang ISR
   * (stm32_steppulse_notify_limit()) ngay khi cong tac tac dong, nen
   * check nay la lop kiem tra PHU - no ngan motion_task ngay ca thu
   * gui lenh vao chieu da bi chan, thay vi la lop cat an toan chinh.
   */

  motorlimit_get(motor_id, &limit);

  return (direction == SAFETY_DIR_DOWN) ? !limit.down : !limit.up;
}

void safety_report_fault(int motor_id, const char *reason)
{
  sys_action_t action;

  syslog(LOG_ERR, "safety_task: FAULT reported (motor %d): %s\n",
         motor_id, reason ? reason : "unspecified");

  action = system_state_handle_fault_event();

  safety_dispatch_action(action);
}