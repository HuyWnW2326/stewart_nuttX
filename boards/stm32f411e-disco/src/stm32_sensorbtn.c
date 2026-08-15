/****************************************************************************
 * boards/arm/stm32/stm32f411e-disco/src/stm32_sensorbtn.c
 *
 * Quan ly ngat, debounce va hang doi su kien cho sau limit switch va
 * ba nut START/STOP, EMERGENCY, RESTART.
 *
 * Wiring: limit switch active-HIGH. START/STOP dung pull-up active-LOW,
 * LOW la START va HIGH la STOP. EMERGENCY cung RESTART dung pull-up,
 * nhan nut tao canh xuong va duoc ma hoa voi level 0.
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <stdio.h>
#include <syslog.h>
#include <stdbool.h>
#include <semaphore.h>
#include <time.h>
#include <errno.h>

#include "stm32_gpio.h"
#include <arch/board/board.h>
#include "stm32_sensorbtn.h"
#include "step_ioctl.h"   /* stm32_steppulse_notify_limit() */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define DEBOUNCE_TICKS   MSEC2TICK(50)

static struct motor_limit_state_s g_limit[MOTOR_COUNT];
static clock_t  g_limit_last_tick[MOTOR_COUNT * 2];  /* 2 line/dong co */
static sem_t    g_limit_event_sem;

#define LIMIT_QUEUE_SIZE   8

static int      g_limit_queue[LIMIT_QUEUE_SIZE];
static int      g_limit_head = 0;
static int      g_limit_tail = 0;

/* Button queue: each entry encodes (btn_id << 1) | level, same scheme as
 * the limit switch queue. 'level' is only meaningful for BTN_STARTSTOP
 * (0 = LOW/START edge, 1 = HIGH/STOP edge); for BTN_EMERGENCY and
 * BTN_RESTART it is always 0 (press event, falling-edge only).
 */

#define BTN_QUEUE_SIZE   8

static int      g_btn_queue[BTN_QUEUE_SIZE];
static int      g_btn_head = 0;
static int      g_btn_tail = 0;
static clock_t  g_btn_last_tick[3];   /* one per physical button pin */
static sem_t    g_btn_event_sem;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: limit_isr
 *
 * Description:
 *   ISR dung chung cho 6 chan limit switch. Goi
 *   stm32_steppulse_notify_limit() ngay khi biet trang thai
 *   moi, truoc khi dung ring buffer/sem_post.
 ****************************************************************************/

static int limit_isr(int irq, FAR void *context, FAR void *arg)
{
  int code      = (int)(intptr_t)arg;
  int motor_id  = code >> 1;
  bool is_up    = (code & 1);
  int debounce_idx = code;
  clock_t now   = clock_systime_ticks();
  bool active;

  if ((now - g_limit_last_tick[debounce_idx]) < DEBOUNCE_TICKS)
    {
      return OK;
    }

  g_limit_last_tick[debounce_idx] = now;

  active = stm32_gpioread(is_up
                            ? (motor_id == 0 ? GPIO_MOTOR1_LIMIT_UP   :
                               motor_id == 1 ? GPIO_MOTOR2_LIMIT_UP   :
                                                GPIO_MOTOR3_LIMIT_UP)
                            : (motor_id == 0 ? GPIO_MOTOR1_LIMIT_DOWN :
                               motor_id == 1 ? GPIO_MOTOR2_LIMIT_DOWN :
                                                GPIO_MOTOR3_LIMIT_DOWN));

  syslog(LOG_INFO, "[LIMIT ISR] motor=%d %s active=%d\n",
       motor_id,
       is_up ? "UP" : "DOWN",
       (int)active);

  stm32_steppulse_notify_limit(motor_id, is_up, active);

  if (is_up)
    {
      g_limit[motor_id].up = active;
    }
  else
    {
      g_limit[motor_id].down = active;
    }

  int next_head = (g_limit_head + 1) % LIMIT_QUEUE_SIZE;
  if (next_head != g_limit_tail)
    {
      g_limit_queue[g_limit_head] = code;
      g_limit_head = next_head;
      sem_post(&g_limit_event_sem);
    }

  return OK;
}

/****************************************************************************
 * Name: btn_startstop_isr
 *
 * Description:
 *   Handles the single dual-edge START/STOP relay pin. Since both edges
 *   are registered, we read the pin level directly (post-edge) rather
 *   than trying to infer it from which edge fired -- this is more robust
 *   against missed/coalesced interrupts than tracking edge direction.
 *
 *   Debounce index 0 is reserved for this pin in g_btn_last_tick[].
 ****************************************************************************/

static int btn_startstop_isr(int irq, FAR void *context, FAR void *arg)
{
  clock_t now = clock_systime_ticks();
  bool level_high;
  int code;

  if ((now - g_btn_last_tick[BTN_STARTSTOP]) < DEBOUNCE_TICKS)
    {
      return OK;
    }

  g_btn_last_tick[BTN_STARTSTOP] = now;

  /* Pull-up, active-low: LOW = relay closed (START), HIGH = relay
   * released (STOP).
   */



  level_high = stm32_gpioread(GPIO_BTN_STARTSTOP);

  syslog(LOG_INFO, "[BTN_STARTSTOP ISR] level=%d", level_high);

  code = (BTN_STARTSTOP << 1) | (level_high ? 1 : 0);

  int next_head = (g_btn_head + 1) % BTN_QUEUE_SIZE;
  if (next_head != g_btn_tail)
    {
      g_btn_queue[g_btn_head] = code;
      g_btn_head = next_head;
      sem_post(&g_btn_event_sem);
    }

  return OK;
}

/****************************************************************************
 * Name: btn_momentary_isr
 *
 * Description:
 *   Shared ISR for EMERGENCY and RESTART -- both are falling-edge-only
 *   momentary buttons, no level needs to be reported (level bit always
 *   0). arg carries the btn_id (BTN_EMERGENCY or BTN_RESTART).
 ****************************************************************************/

static int btn_momentary_isr(int irq, FAR void *context, FAR void *arg)
{
  int btn_id  = (int)(intptr_t)arg;
  clock_t now = clock_systime_ticks();
  int code;

  if ((now - g_btn_last_tick[btn_id]) < DEBOUNCE_TICKS)
    {
      return OK;
    }

  g_btn_last_tick[btn_id] = now;

  syslog(LOG_INFO, "[BTN_MOMENTARY ISR] btn_id=%d (%s)\n", btn_id,
         btn_id == BTN_EMERGENCY ? "EMERGENCY" : "RESTART");

  code = (btn_id << 1) | 0;

  int next_head = (g_btn_head + 1) % BTN_QUEUE_SIZE;
  if (next_head != g_btn_tail)
    {
      g_btn_queue[g_btn_head] = code;
      g_btn_head = next_head;
      sem_post(&g_btn_event_sem);
    }

  return OK;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool motorlimit_read_hw(int motor_id, bool is_up)
{
  uint32_t gpio;

  if (is_up)
    {
      gpio = (motor_id == 0) ? GPIO_MOTOR1_LIMIT_UP   :
             (motor_id == 1) ? GPIO_MOTOR2_LIMIT_UP   :
                               GPIO_MOTOR3_LIMIT_UP;
    }
  else
    {
      gpio = (motor_id == 0) ? GPIO_MOTOR1_LIMIT_DOWN :
             (motor_id == 1) ? GPIO_MOTOR2_LIMIT_DOWN :
                               GPIO_MOTOR3_LIMIT_DOWN;
    }

  return stm32_gpioread(gpio);  
}

int stm32_sensorbtn_initialize(void)
{
  struct { uint32_t pinset; int code; const char *name; } limit_pins[] =
    {
      { GPIO_MOTOR1_LIMIT_DOWN, (0 << 1) | 0, "M1_DOWN" },
      { GPIO_MOTOR1_LIMIT_UP,   (0 << 1) | 1, "M1_UP"   },
      { GPIO_MOTOR2_LIMIT_DOWN, (1 << 1) | 0, "M2_DOWN" },
      { GPIO_MOTOR2_LIMIT_UP,   (1 << 1) | 1, "M2_UP"   },
      { GPIO_MOTOR3_LIMIT_DOWN, (2 << 1) | 0, "M3_DOWN" },
      { GPIO_MOTOR3_LIMIT_UP,   (2 << 1) | 1, "M3_UP"   },
    };

  int ret;

  sem_init(&g_limit_event_sem, 0, 0);
  sem_init(&g_btn_event_sem, 0, 0);

  for (size_t i = 0; i < sizeof(limit_pins) / sizeof(limit_pins[0]); i++)
    {
      ret = stm32_gpiosetevent(limit_pins[i].pinset, true, true, true,
                                limit_isr,
                                (void *)(intptr_t)limit_pins[i].code);
      printf("[SENSORBTN] LIMIT %-7s ret=%d\n", limit_pins[i].name, ret);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* START/STOP: single pin, both edges (risingedge=true, fallingedge=true). */

  ret = stm32_gpiosetevent(GPIO_BTN_STARTSTOP, true, true, true,
                            btn_startstop_isr, NULL);
  printf("[SENSORBTN] BTN STARTSTOP ret=%d\n", ret);
  if (ret < 0)
    {
      return ret;
    }

  /* EMERGENCY, RESTART: falling-edge only, momentary. */

  ret = stm32_gpiosetevent(GPIO_BTN_EMERGENCY, false, true, true,
                            btn_momentary_isr,
                            (void *)(intptr_t)BTN_EMERGENCY);
  printf("[SENSORBTN] BTN EMERGENCY ret=%d\n", ret);
  if (ret < 0)
    {
      return ret;
    }

  ret = stm32_gpiosetevent(GPIO_BTN_RESTART, false, true, true,
                            btn_momentary_isr,
                            (void *)(intptr_t)BTN_RESTART);
  printf("[SENSORBTN] BTN RESTART ret=%d\n", ret);
  if (ret < 0)
    {
      return ret;
    }

  fflush(stdout);
  return OK;
}

void motorlimit_get(int motor_id, struct motor_limit_state_s *out)
{
  *out = g_limit[motor_id];
}

void motorlimit_waitevent(void)
{
  sem_wait(&g_limit_event_sem);
}

int motorlimit_timedwaitevent(FAR const struct timespec *abstime)
{
  return sem_timedwait(&g_limit_event_sem, abstime);
}

void motorlimit_waitevent_id(FAR int *motor_id, FAR bool *is_up)
{
  int code;

  sem_wait(&g_limit_event_sem);

  irqstate_t flags = enter_critical_section();
  code = g_limit_queue[g_limit_tail];
  g_limit_tail = (g_limit_tail + 1) % LIMIT_QUEUE_SIZE;
  leave_critical_section(flags);

  *motor_id = code >> 1;
  *is_up    = (code & 1);
}

/****************************************************************************
 * Name: motorlimit_timedwaitevent_id
 *
 * Description:
 *   Giong motorlimit_waitevent_id(), nhung dung sem_timedwait() thay vi
 *   sem_wait() -- cho phep caller (homing_task) tinh dung khi het
 *   thoi gian cho ma chua co limit event nao, de tu kiem tra dieu kien
 *   huy (vi du system_state chuyen sang ESTOP do EMERGENCY) thay vi bi
 *   treo vo han.
 *
 * Returned Value:
 *   OK (0) va dien *motor_id va *is_up neu co event that su trong luc
 *   cho. Ma loi am (vi du -ETIMEDOUT tu sem_timedwait) neu het thoi
 *   gian cho - luc do *motor_id va *is_up KHONG duoc dong, khong nen doc.
 ****************************************************************************/

int motorlimit_timedwaitevent_id(FAR const struct timespec *abstime,
                                  FAR int *motor_id, FAR bool *is_up)
{
  int code;
  int ret;

  ret = sem_timedwait(&g_limit_event_sem, abstime);
  if (ret < 0)
    {
      return -get_errno();
    }

  irqstate_t flags = enter_critical_section();
  code = g_limit_queue[g_limit_tail];
  g_limit_tail = (g_limit_tail + 1) % LIMIT_QUEUE_SIZE;
  leave_critical_section(flags);

  *motor_id = code >> 1;
  *is_up    = (code & 1);

  return OK;
}

void motorbtn_waitevent(FAR int *btn_id, FAR int *level)
{
  int code;

  sem_wait(&g_btn_event_sem);

  irqstate_t flags = enter_critical_section();
  code = g_btn_queue[g_btn_tail];
  g_btn_tail = (g_btn_tail + 1) % BTN_QUEUE_SIZE;
  leave_critical_section(flags);

  *btn_id = code >> 1;
  *level  = code & 1;
}
