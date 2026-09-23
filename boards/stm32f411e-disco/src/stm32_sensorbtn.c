/****************************************************************************
 * boards/arm/stm32/stm32f411e-disco/src/stm32_sensorbtn.c
 *
 * Interrupt handling, debouncing, and event queue for limit switches and
 * three control buttons: START/STOP, EMERGENCY, RESTART.
 *
 * Wiring convention:
 * - Limit switches: active-HIGH (1 = limit reached)
 * - START/STOP button: pull-up, active-LOW; LOW=START edge, HIGH=STOP edge
 * - EMERGENCY / RESTART buttons: pull-up, active-LOW; falling-edge only
 *
 * Debounce strategy (IMPORTANT - differs per line):
 *
 * - Limit switches: "settle-time" (quiet period) debounce. The ISR does
 *   NOT process the edge - it only (re)arms a per-line watchdog. Every
 *   further edge on that line restarts the watchdog. Only once the line
 *   has been quiet for DEBOUNCE_TICKS does a work-queue job read the
 *   final pin level, call stm32_steppulse_notify_limit(), update
 *   g_limit[] and push the event.
 *
 * - RESTART button: same "settle-time" scheme as the limit switches
 *   (wdog + work queue) - RESTART triggers a real action (spawn homing)
 *   so it needs to be robust against a noisy first edge, same reasoning
 *   as limit switches.
 *
 * - START/STOP and EMERGENCY: "lockout" debounce (kept as before,
 *   unchanged). The ISR accepts the edge immediately and locks out
 *   further edges on that line for DEBOUNCE_TICKS.
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
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

#define DEBOUNCE_TICKS   MSEC2TICK(15)

static struct motor_limit_state_s g_limit[MOTOR_COUNT];
static sem_t    g_limit_event_sem;

#define LIMIT_QUEUE_SIZE   8

static int      g_limit_queue[LIMIT_QUEUE_SIZE];
static int      g_limit_head = 0;
static int      g_limit_tail = 0;

/* Settle-time debounce state per limit switch line. Each ISR does
 * nothing but (re)arm g_limit_debounce[code].wdog; the wdog only fires
 * if no further edge arrives on that line within DEBOUNCE_TICKS, at
 * which point limit_worker() runs (in work-queue context) to read the
 * settled pin level and do the real processing.
 *
 * code encoding matches the queue/event encoding used everywhere else
 * in this file: code = (motor_id << 1) | is_up.
 */

struct limit_debounce_s
{
  struct wdog_s wdog;
  struct work_s work;
  int           code;
};

static struct limit_debounce_s g_limit_debounce[MOTOR_COUNT * 2];

/* Button queue: each entry encodes (btn_id << 1) | level, same scheme as
 * the limit switch queue. 'level' is only meaningful for BTN_STARTSTOP
 * (0 = LOW/START edge, 1 = HIGH/STOP edge); for BTN_EMERGENCY and
 * BTN_RESTART it is always 0 (press event, falling-edge only).
 */

#define BTN_QUEUE_SIZE   8

static int      g_btn_queue[BTN_QUEUE_SIZE];
static int      g_btn_head = 0;
static int      g_btn_tail = 0;
static clock_t  g_btn_last_tick[3];   /* lockout debounce: STARTSTOP, EMERGENCY */
static sem_t    g_btn_event_sem;

/* Settle-time debounce state for RESTART only (same scheme as the limit
 * switches). START/STOP and EMERGENCY stay on the lockout scheme above.
 */

static struct wdog_s g_restart_wdog;
static struct work_s g_restart_work;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: limit_worker
 *
 * Description:
 *   Chay trong work queue (context binh thuong, duoc phep goi
 *   stm32_gpioread()/syslog()) sau khi g_limit_debounce[code].wdog het
 *   han ma khong bi huy/gia han them - nghia la duong day da "im lang"
 *   du DEBOUNCE_TICKS. Doc muc chan hien tai (da on dinh) va lam toan
 *   bo xu ly that su cua mot lan limit event.
 ****************************************************************************/

static void limit_worker(FAR void *arg)
{
  FAR struct limit_debounce_s *db = (FAR struct limit_debounce_s *)arg;
  int  code     = db->code;
  int  motor_id = code >> 1;
  bool is_up    = (code & 1);
  bool active;

  active = stm32_gpioread(is_up
                            ? (motor_id == 0 ? GPIO_MOTOR1_LIMIT_UP   :
                               motor_id == 1 ? GPIO_MOTOR2_LIMIT_UP   :
                                               GPIO_MOTOR3_LIMIT_UP)
                            : (motor_id == 0 ? GPIO_MOTOR1_LIMIT_DOWN :
                               motor_id == 1 ? GPIO_MOTOR2_LIMIT_DOWN :
                                               GPIO_MOTOR3_LIMIT_DOWN));

  syslog(LOG_INFO, "[LIMIT debounce] motor=%d %s active=%d\n",
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

  irqstate_t flags = enter_critical_section();
  int next_head = (g_limit_head + 1) % LIMIT_QUEUE_SIZE;
  if (next_head != g_limit_tail)
    {
      g_limit_queue[g_limit_head] = code;
      g_limit_head = next_head;
    }
  leave_critical_section(flags);

  sem_post(&g_limit_event_sem);
}

/****************************************************************************
 * Name: limit_wdog_timeout
 *
 * Description:
 *   Callback cua wdog, chay trong context ngat hen gio - khong goi
 *   stm32_gpioread()/syslog()/stm32_steppulse_notify_limit() truc tiep
 *   o day, chi day cong viec that su xuong work queue (limit_worker).
 ****************************************************************************/

static void limit_wdog_timeout(wdparm_t arg)  
{
  FAR struct limit_debounce_s *db = (FAR struct limit_debounce_s *)arg;

  work_queue(HPWORK, &db->work, limit_worker, db, 0);
}

/****************************************************************************
 * Name: limit_isr
 *
 * Description:
 *   ISR dung chung cho 6 chan limit switch. KHONG xu ly su kien tai
 *   day - chi (tai) khoi dong bo dem cua duong nay. Moi canh moi trong
 *   luc bo dem dang chay se huy bo dem cu va bat dau dem lai tu dau
 *   (wd_start goi lai se tu dong huy lan hen truoc do neu con dang
 *   cho), nen su kien chi thuc su duoc xac nhan va xu ly (doc muc chan,
 *   goi stm32_steppulse_notify_limit(), cap nhat g_limit[], day hang
 *   doi) khi duong day im lang lien tuc du DEBOUNCE_TICKS - xem
 *   limit_worker().
 ****************************************************************************/

static int limit_isr(int irq, FAR void *context, FAR void *arg)
{
  int code = (int)(intptr_t)arg;
  FAR struct limit_debounce_s *db = &g_limit_debounce[code];

  wd_start(&db->wdog, DEBOUNCE_TICKS, limit_wdog_timeout, (wdparm_t)db);

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
 * Name: btn_emergency_isr
 *
 * Description:
 *   EMERGENCY -- falling-edge-only momentary button, no level needs to
 *   be reported (level bit always 0). Lockout debounce, unchanged from
 *   before (kept separate from RESTART now that RESTART uses a
 *   different scheme -- see btn_restart_isr()).
 ****************************************************************************/

static int btn_emergency_isr(int irq, FAR void *context, FAR void *arg)
{
  clock_t now = clock_systime_ticks();
  int code;

  if ((now - g_btn_last_tick[BTN_EMERGENCY]) < DEBOUNCE_TICKS)
    {
      return OK;
    }

  g_btn_last_tick[BTN_EMERGENCY] = now;

  syslog(LOG_INFO, "[BTN_EMERGENCY ISR]\n");

  code = (BTN_EMERGENCY << 1) | 0;

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
 * Name: restart_worker
 *
 * Description:
 *   Chay trong work queue sau khi g_restart_wdog het han ma khong bi
 *   huy/gia han them - nghia la duong RESTART da "im lang" du
 *   DEBOUNCE_TICKS. Doc muc chan da on dinh; chi day su kien neu van
 *   con active (LOW), tranh nhan nham mot lan nhieu thoang qua.
 ****************************************************************************/

static void restart_worker(FAR void *arg)
{
  int code;

  if (stm32_gpioread(GPIO_BTN_RESTART))
    {
      /* Da tro lai HIGH (khong con active) truoc khi kip on dinh o
       * muc LOW - chi la nhieu thoang qua, khong phai mot lan nhan
       * that.
       */

      return;
    }

  syslog(LOG_INFO, "[BTN_RESTART debounce]\n");

  code = (BTN_RESTART << 1) | 0;

  irqstate_t flags = enter_critical_section();
  int next_head = (g_btn_head + 1) % BTN_QUEUE_SIZE;
  if (next_head != g_btn_tail)
    {
      g_btn_queue[g_btn_head] = code;
      g_btn_head = next_head;
    }
  leave_critical_section(flags);

  sem_post(&g_btn_event_sem);
}

/****************************************************************************
 * Name: restart_wdog_timeout
 *
 * Description:
 *   Callback cua wdog, chay trong context ngat hen gio - khong goi
 *   stm32_gpioread()/syslog() truc tiep o day, chi day cong viec that
 *   su xuong work queue (restart_worker).
 ****************************************************************************/

static void restart_wdog_timeout(wdparm_t arg)
{
  work_queue(HPWORK, &g_restart_work, restart_worker, NULL, 0);
}

/****************************************************************************
 * Name: btn_restart_isr
 *
 * Description:
 *   RESTART -- falling-edge-only momentary button. KHONG xu ly su kien
 *   tai day - chi (tai) khoi dong g_restart_wdog. Moi canh moi trong
 *   luc bo dem dang chay se huy bo dem cu va bat dau dem lai tu dau, nen
 *   su kien chi thuc su duoc xac nhan khi duong day im lang lien tuc du
 *   DEBOUNCE_TICKS - xem restart_worker(). RESTART trigger mot hanh
 *   dong that (spawn homing) nen dung cung kieu settle-time nhu limit
 *   switch thay vi lockout.
 ****************************************************************************/

static int btn_restart_isr(int irq, FAR void *context, FAR void *arg)
{
  wd_start(&g_restart_wdog, DEBOUNCE_TICKS, restart_wdog_timeout, 0);

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
      g_limit_debounce[limit_pins[i].code].code = limit_pins[i].code;

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

  /* EMERGENCY: falling-edge only, momentary, lockout debounce. */

  ret = stm32_gpiosetevent(GPIO_BTN_EMERGENCY, false, true, true,
                            btn_emergency_isr, NULL);
  printf("[SENSORBTN] BTN EMERGENCY ret=%d\n", ret);
  if (ret < 0)
    {
      return ret;
    }

  /* RESTART: falling-edge only, momentary, settle-time debounce. */

  ret = stm32_gpiosetevent(GPIO_BTN_RESTART, false, true, true,
                            btn_restart_isr, NULL);
  printf("[SENSORBTN] BTN RESTART ret=%d\n", ret);
  if (ret < 0)
    {
      return ret;
    }

  fflush(stdout);
  return OK;
}

void motorlimit_flush_events(void)
{
  irqstate_t flags = enter_critical_section();
  g_limit_head = 0;
  g_limit_tail = 0;
  leave_critical_section(flags);

  /* Rut can sem_post con du (drain khong block) */
  while (sem_trywait(&g_limit_event_sem) == 0)
    {
      /* discard */
    }
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