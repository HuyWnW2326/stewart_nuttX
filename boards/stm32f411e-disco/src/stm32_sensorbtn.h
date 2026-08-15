/****************************************************************************
 * boards/arm/stm32/stm32f411e-disco/src/stm32_sensorbtn.h
 *
 * Khai bao giao dien limit switch va nut dieu khien cua board.
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_STM32F411E_DISCO_SRC_STM32_SENSORBTN_H
#define __BOARDS_ARM_STM32_STM32F411E_DISCO_SRC_STM32_SENSORBTN_H

#include <nuttx/config.h>
#include <stdbool.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MOTOR_COUNT  3

/* Button identifiers used by motorbtn_waitevent(). Indexes into
 * g_btn_last_tick[] too, so keep values 0..2.
 */

#define BTN_STARTSTOP  0
#define BTN_EMERGENCY  1
#define BTN_RESTART    2

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct motor_limit_state_s
{
  bool up;
  bool down;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int  stm32_sensorbtn_initialize(void);

bool motorlimit_read_hw(int motor_id, bool is_up);
void motorlimit_get(int motor_id, FAR struct motor_limit_state_s *out);
void motorlimit_waitevent(void);
int  motorlimit_timedwaitevent(FAR const struct timespec *abstime);
void motorlimit_waitevent_id(FAR int *motor_id, FAR bool *is_up);

/* Giong motorlimit_waitevent_id(), nhung dung sem_timedwait() voi
 * abstime CLOCK_REALTIME. Timeout cho phep homing_task thoat som khi
 * EMERGENCY chuyen system_state sang ESTOP thay vi bi treo vo han tren
 * semaphore.
 *
 * Return OK va dien *motor_id va *is_up neu co event that su trong luc
 * cho. Return < 0 (vi du -ETIMEDOUT) neu het thoi gian cho ma chua co
 * event - luc do *motor_id va *is_up KHONG hop le, caller nen kiem tra
 * lai dieu kien huy (vi du system_state_get()) roi goi lai.
 */
int  motorlimit_timedwaitevent_id(FAR const struct timespec *abstime,
                                   FAR int *motor_id, FAR bool *is_up);

/* level meaning: for BTN_STARTSTOP, 0 = START edge, 1 = STOP edge.
 * For BTN_EMERGENCY / BTN_RESTART, always 0 (press event).
 */

void motorbtn_waitevent(FAR int *btn_id, FAR int *level);

#endif /* __BOARDS_ARM_STM32_STM32F411E_DISCO_SRC_STM32_SENSORBTN_H */
