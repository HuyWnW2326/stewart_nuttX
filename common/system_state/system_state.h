/****************************************************************************
 * common/system_state/system_state.h
 *
 * Module dung chung (khong phai task) luu trang thai van hanh toan he
 * thong, GOP tu 2 state machine doc lap truoc day (system_state cu +
 * safety_task's state machine) thanh 1 nguon su that duy nhat.
 *
 * Nguyen tac thiet ke:
 *   - Module nay CHI la kho du lieu bi dong (mutex-protected data
 *     store) + logic transition THUAN (khong I/O, khong block lau).
 *     Nhieu task khac nhau (motion_task, homing_task, safety_task,
 *     tuong lai la reconciliation task) deu doc nhanh state o day, ke
 *     ca o 100Hz, nen KHONG duoc phep co bat ky loi goi I/O (ioctl,
 *     open, task_create...) ben trong khi dang giu mutex.
 *   - Task nao goi system_state_handle_btn_event() /
 *     system_state_handle_fault_event() se nhan ve 1 sys_action_t mo
 *     ta viec CAN lam tiep (spawn homing_task, hard-stop...) - viec
 *     thuc thi I/O do la trach nhiem cua caller (safety_task.c), luon
 *     thuc hien SAU KHI ham nay da return (ngoai lock).
 *
 * Thread-safe: moi ham deu tu giu mutex noi bo, caller khong can lock
 * them.
 ****************************************************************************/

#ifndef __COMMON_SYSTEM_STATE_H
#define __COMMON_SYSTEM_STATE_H

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYSTEM_STATE_MOTOR_COUNT    3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Trang thai tong the cua he thong.
 *
 * Luong binh thuong:
 *   IDLE --START--> HOMING --(homing xong)--> WAIT_START --START--> RUNNING
 *   RUNNING --STOP--> STOPPED --START--> RUNNING (resume, khong home lai)
 *   STOPPED --RESTART--> HOMING (reset toan bo, home lai tu dau)
 *
 * EMERGENCY: tu HOMING/WAIT_START/RUNNING/STOPPED -> ESTOP ngay. Rieng
 * o IDLE thi EMERGENCY KHONG co tac dung (khong co gi dang chay de can
 * dung khan cap, o lai IDLE, van cho START binh thuong).
 *
 * RESTART: chi hop le khi dang o STOPPED / ESTOP / FAULT -> HOMING
 * (reset toan bo ve nhu luc moi cap nguon, roi TU DONG chay homing
 * ngay, khong can cho THEM 1 lan START nua truoc khi homing bat dau).
 *
 * FAULT: hien tai CHUA co code path nao trigger - la state du tru cho
 * tuong lai (reconciliation task phat hien stall/lech vi tri, homing
 * timeout, mat Modbus keo dai, chan ALARM cua driver AC servo...).
 * Xu ly RESTART tu FAULT giong het tu ESTOP.
 */

typedef enum
{
  SYS_STATE_IDLE = 0,    /* Vua cap nguon / chua lam gi, cho START lan dau */
  SYS_STATE_HOMING,      /* Dang chay chu trinh homing (ca 3 truc ve
                           * limit duoi roi nang len vi tri home) */
  SYS_STATE_WAIT_START,  /* Homing xong, dung yen, cho START lan 2 de
                           * bat dau tu can bang */
  SYS_STATE_RUNNING,     /* Dang tu can bang theo PWM tu PX4 */
  SYS_STATE_STOPPED,     /* Tam dung can bang (do STOP), dung yen,
                           * START se resume, RESTART se home lai */
  SYS_STATE_ESTOP,       /* EMERGENCY da kich hoat, da hard-stop */
  SYS_STATE_FAULT        /* Loi nghiem trong (du tru, chua co trigger) */
} sys_state_t;

/* Viec ma caller (safety_task.c) CAN lam sau khi goi 1 ham xu ly
 * event, luon thuc hien NGOAI mutex lock.
 */

typedef enum
{
  SYS_ACTION_NONE = 0,      /* Khong can lam gi them */
  SYS_ACTION_SPAWN_HOMING,  /* Can task_create() homing_task_main() */
  SYS_ACTION_HARD_STOP      /* Can goi STEPIOC_ESTOP tren ca 3 /dev/stepN */
} sys_action_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Khoi tao - goi 1 lan duy nhat trong stewart_main truoc khi tao bat ky
 * task nao. State ban dau la SYS_STATE_IDLE.
 */

void system_state_init(void);

/* Doc state hien tai. Thread-safe, khong block lau. */

sys_state_t system_state_get(void);

/* Xu ly 1 event nut bam (goi tu safety_task's button-wait loop).
 *
 *   btn_id : BTN_STARTSTOP / BTN_EMERGENCY / BTN_RESTART
 *            (dinh nghia trong stm32_sensorbtn.h)
 *   level  : chi co y nghia voi BTN_STARTSTOP (0 = START, 1 = STOP);
 *            voi BTN_EMERGENCY / BTN_RESTART, gia tri nay bi bo qua.
 *
 * THUAN logic + cap nhat state duoi lock, KHONG goi bat ky I/O nao ben
 * trong. Tra ve sys_action_t caller can thuc hien SAU KHI ham nay
 * return.
 */

sys_action_t system_state_handle_btn_event(int btn_id, int level);

/* Goi boi homing_task khi hoan tat ca 3 truc (buoc cuoi cua
 * homing_task_main). Thuan cap nhat state (HOMING -> WAIT_START),
 * khong I/O.
 */

void system_state_notify_homing_complete(void);

/* Danh cho tuong lai: goi boi 1 task phat hien loi (vi du
 * reconciliation task so sanh vi tri lenh vs vi tri encoder thuc).
 * Chuyen sang SYS_STATE_FAULT tu bat ky state nao (tru khi da o
 * ESTOP/FAULT), tra ve SYS_ACTION_HARD_STOP de caller tu goi
 * STEPIOC_ESTOP. THUAN logic, khong I/O.
 */

sys_action_t system_state_handle_fault_event(void);

/****************************************************************************
 * Homing progress - buoc 1: truc nao da cham LIMIT_DOWN,
 *                   buoc 2: truc nao da nang len vi tri hoat dong xong
 ****************************************************************************/

void system_state_set_limit_reached(uint8_t motor_id, bool reached);
bool system_state_is_limit_reached(uint8_t motor_id);
bool system_state_all_limits_reached(void);

void system_state_set_homed(uint8_t motor_id, bool homed);
bool system_state_is_homed(uint8_t motor_id);
bool system_state_all_homed(void);

/* Reset limit_reached[]/homed[] ve false het. Da duoc goi tu BEN
 * TRONG system_state_handle_btn_event() moi khi bat dau 1 chu trinh
 * homing moi (IDLE->HOMING hoac RESTART->HOMING); van de public de
 * homing_task hoac code khac co the goi truc tiep neu can.
 */

void system_state_reset_homing(void);

#endif /* __COMMON_SYSTEM_STATE_H */