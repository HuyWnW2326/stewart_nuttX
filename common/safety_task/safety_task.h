/****************************************************************************
 * common/safety_task/safety_task.h
 *
 * Xu ly nut START/STOP/EMERGENCY/RESTART, thuc thi cac I/O an toan theo
 * action cua system_state va kiem tra quyen chuyen dong theo limit switch.
 *
 * Luu y: limit switch da duoc hard-cut xung o tang ISR
 * (stm32_steppulse_notify_limit(), goi truc tiep tu stm32_sensorbtn.c)
 * - safety_task khong can tu goi STEPIOC_ESTOP khi co limit event;
 * safety_is_motor_allowed() chi la lop kiem tra phan mem PHU.
 ****************************************************************************/

#ifndef __COMMON_SAFETY_TASK_H
#define __COMMON_SAFETY_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "system_state.h"     /* sys_state_t */
#include "stm32_sensorbtn.h"  /* MOTOR_COUNT */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Huong di chuyen dung boi safety_is_motor_allowed(). Phai khop quy
 * uoc DIR pin dang dung trong stm32_steppulse.c / motion_task.
 */

#define SAFETY_DIR_UP    0
#define SAFETY_DIR_DOWN  1

#define SAFETY_NUM_MOTORS MOTOR_COUNT

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Khoi tao va tao thread safety_task. Goi 1 lan tu board bringup, SAU
 * khi stm32_sensorbtn_initialize() va system_state_init() da chay.
 *
 * Returns 0 neu thanh cong, ma loi am neu that bai.
 */

int safety_task_initialize(void);

/* Thread entry point. Thuong khong can goi truc tiep -
 * safety_task_initialize() da tu spawn no.
 */

void *safety_task_main(void *arg);

/* Doc state hien tai (wrapper mong cho system_state_get()). */

sys_state_t safety_get_state(void);

/* Tra ve true neu motion_task/homing_task duoc phep dieu khien
 * motor_id (0..2) di chuyen theo direction (SAFETY_DIR_UP/DOWN) NGAY
 * BAY GIO. Ket hop ca system state (phai la RUNNING hoac HOMING) va
 * trang thai limit switch song cua truc do.
 *
 * PHAI duoc goi truoc MOI lenh STEPIOC_MOVE/STEPIOC_HOME.
 */

bool safety_is_motor_allowed(int motor_id, int direction);

/* Goi boi task khac (vi du reconciliation task tuong lai) khi phat
 * hien loi nghiem trong (stall, mat feedback...). Chuyen he thong
 * sang SYS_STATE_FAULT va hard-stop ngay, giong het duong EMERGENCY.
 *
 * motor_id co the la -1 neu loi khong gan voi 1 truc cu the.
 * reason chi dung de log (syslog), an toan truyen 1 chuoi literal.
 */

void safety_report_fault(int motor_id, const char *reason);

#endif /* __COMMON_SAFETY_TASK_H */
