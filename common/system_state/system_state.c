/****************************************************************************
 * common/system_state/system_state.c
 *
 * Xem system_state.h de biet bang chuyen trang thai day du va nguyen
 * tac thiet ke (khong I/O ben trong lock).
 ****************************************************************************/

#include <nuttx/config.h>

#include <pthread.h>
#include <string.h>
#include <syslog.h>

#include "system_state.h"
#include "stm32_sensorbtn.h"   /* BTN_STARTSTOP / BTN_EMERGENCY / BTN_RESTART */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* level cua BTN_STARTSTOP - dat lai ten cho ro rang thay vi magic
 * number 0/1 rai rac trong file. Dung dung quy uoc trong
 * stm32_sensorbtn.c: 0 = START (relay dong), 1 = STOP (relay nha).
 */

#define SYS_LEVEL_START   0
#define SYS_LEVEL_STOP    1

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct
{
  pthread_mutex_t mutex;
  sys_state_t     state;

  bool            limit_reached[SYSTEM_STATE_MOTOR_COUNT];
  bool            homed[SYSTEM_STATE_MOTOR_COUNT];
} g_state;

/****************************************************************************
 * Private Functions
 *
 * Hai ham duoi day PHAI duoc goi trong khi dang giu g_state.mutex.
 ****************************************************************************/

static void system_state_set_locked(sys_state_t new_state)
{
  if (g_state.state != new_state)
    {
      syslog(LOG_INFO, "system_state: %d -> %d\n",
             (int)g_state.state, (int)new_state);
      g_state.state = new_state;
    }
}

static void system_state_reset_homing_locked(void)
{
  memset(g_state.limit_reached, false, sizeof(g_state.limit_reached));
  memset(g_state.homed,         false, sizeof(g_state.homed));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void system_state_init(void)
{
  pthread_mutex_init(&g_state.mutex, NULL);

  g_state.state = SYS_STATE_IDLE;

  memset(g_state.limit_reached, false, sizeof(g_state.limit_reached));
  memset(g_state.homed,         false, sizeof(g_state.homed));
}

sys_state_t system_state_get(void)
{
  sys_state_t state;

  pthread_mutex_lock(&g_state.mutex);
  state = g_state.state;
  pthread_mutex_unlock(&g_state.mutex);

  return state;
}

sys_action_t system_state_handle_btn_event(int btn_id, int level)
{
  sys_action_t action = SYS_ACTION_NONE;

  pthread_mutex_lock(&g_state.mutex);

  switch (btn_id)
    {
      case BTN_STARTSTOP:
        if (level == SYS_LEVEL_START)
          {
            if (g_state.state == SYS_STATE_IDLE)
              {
                /* Lan START dau tien: bat dau homing */

                system_state_reset_homing_locked();
                system_state_set_locked(SYS_STATE_HOMING);
                action = SYS_ACTION_SPAWN_HOMING;
              }
            else if (g_state.state == SYS_STATE_WAIT_START)
              {
                /* Lan START thu 2: homing da xong, bat dau can bang */

                system_state_set_locked(SYS_STATE_RUNNING);
              }
            else if (g_state.state == SYS_STATE_STOPPED)
              {
                /* Resume can bang, KHONG home lai */

                system_state_set_locked(SYS_STATE_RUNNING);
              }

            /* Cac state khac (HOMING, RUNNING, ESTOP, FAULT): bo qua */
          }
        else /* SYS_LEVEL_STOP */
          {
            if (g_state.state == SYS_STATE_RUNNING)
              {
                system_state_set_locked(SYS_STATE_STOPPED);
              }

            /* STOP KHONG co tac dung khi dang HOMING (yeu cau rieng)
             * va cac state khac cung bo qua.
             */
          }
        break;

      case BTN_EMERGENCY:
        /* IDLE: khong co gi dang chay de can dung khan cap - bo qua,
         * o lai IDLE, van cho START binh thuong. Da o ESTOP/FAULT thi
         * cung khong can lam gi them.
         */

        if (g_state.state != SYS_STATE_IDLE &&
            g_state.state != SYS_STATE_ESTOP &&
            g_state.state != SYS_STATE_FAULT)
          {
            system_state_set_locked(SYS_STATE_ESTOP);
            action = SYS_ACTION_HARD_STOP;
          }
        break;

      case BTN_RESTART:
        /* Chi hop le khi dang o 1 trong 3 trang thai "dung yen":
         * STOPPED (da STOP), ESTOP hoac FAULT (da hard-stop). Bi bo
         * qua khi dang RUNNING/HOMING/WAIT_START/IDLE.
         */

        if (g_state.state == SYS_STATE_STOPPED ||
            g_state.state == SYS_STATE_ESTOP   ||
            g_state.state == SYS_STATE_FAULT)
          {
            /* Reset toan bo ve nhu luc moi cap nguon, roi tu dong
             * home lai ngay - KHONG can doi them 1 lan START nua.
             */

            system_state_reset_homing_locked();
            system_state_set_locked(SYS_STATE_HOMING);
            action = SYS_ACTION_SPAWN_HOMING;
          }
        break;

      default:
        syslog(LOG_WARNING, "system_state: unknown btn_id %d\n", btn_id);
        break;
    }

  pthread_mutex_unlock(&g_state.mutex);

  return action;
}

void system_state_notify_homing_complete(void)
{
  pthread_mutex_lock(&g_state.mutex);

  if (g_state.state == SYS_STATE_HOMING)
    {
      system_state_set_locked(SYS_STATE_WAIT_START);
    }

  pthread_mutex_unlock(&g_state.mutex);
}

sys_action_t system_state_handle_fault_event(void)
{
  sys_action_t action = SYS_ACTION_NONE;

  pthread_mutex_lock(&g_state.mutex);

  if (g_state.state != SYS_STATE_ESTOP && g_state.state != SYS_STATE_FAULT)
    {
      system_state_set_locked(SYS_STATE_FAULT);
      action = SYS_ACTION_HARD_STOP;
    }

  pthread_mutex_unlock(&g_state.mutex);

  return action;
}

/****************************************************************************
 * Homing progress
 ****************************************************************************/

void system_state_set_limit_reached(uint8_t motor_id, bool reached)
{
  if (motor_id >= SYSTEM_STATE_MOTOR_COUNT)
    {
      return;
    }

  pthread_mutex_lock(&g_state.mutex);
  g_state.limit_reached[motor_id] = reached;
  pthread_mutex_unlock(&g_state.mutex);
}

bool system_state_is_limit_reached(uint8_t motor_id)
{
  bool reached;

  if (motor_id >= SYSTEM_STATE_MOTOR_COUNT)
    {
      return false;
    }

  pthread_mutex_lock(&g_state.mutex);
  reached = g_state.limit_reached[motor_id];
  pthread_mutex_unlock(&g_state.mutex);

  return reached;
}

bool system_state_all_limits_reached(void)
{
  bool all;
  int  i;

  pthread_mutex_lock(&g_state.mutex);

  all = true;
  for (i = 0; i < SYSTEM_STATE_MOTOR_COUNT; i++)
    {
      if (!g_state.limit_reached[i])
        {
          all = false;
          break;
        }
    }

  pthread_mutex_unlock(&g_state.mutex);

  return all;
}

void system_state_set_homed(uint8_t motor_id, bool homed)
{
  if (motor_id >= SYSTEM_STATE_MOTOR_COUNT)
    {
      return;
    }

  pthread_mutex_lock(&g_state.mutex);
  g_state.homed[motor_id] = homed;
  pthread_mutex_unlock(&g_state.mutex);
}

bool system_state_is_homed(uint8_t motor_id)
{
  bool homed;

  if (motor_id >= SYSTEM_STATE_MOTOR_COUNT)
    {
      return false;
    }

  pthread_mutex_lock(&g_state.mutex);
  homed = g_state.homed[motor_id];
  pthread_mutex_unlock(&g_state.mutex);

  return homed;
}

bool system_state_all_homed(void)
{
  bool all;
  int  i;

  pthread_mutex_lock(&g_state.mutex);

  all = true;
  for (i = 0; i < SYSTEM_STATE_MOTOR_COUNT; i++)
    {
      if (!g_state.homed[i])
        {
          all = false;
          break;
        }
    }

  pthread_mutex_unlock(&g_state.mutex);

  return all;
}

void system_state_reset_homing(void)
{
  pthread_mutex_lock(&g_state.mutex);
  system_state_reset_homing_locked();
  pthread_mutex_unlock(&g_state.mutex);
}