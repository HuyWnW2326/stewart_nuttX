/****************************************************************************
 * common/motor_pos/motor_pos.h 
 *
 * Luu feedback encoder va quy doi sai lech goc thanh so xung motor.
 ****************************************************************************/
#ifndef __COMMON_MOTOR_POS_MOTOR_POS_H
#define __COMMON_MOTOR_POS_MOTOR_POS_H

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_POS_COUNT               3

#define MOTOR_POS_GEAR_RATIO          100
#define MOTOR_POS_PULSE               10000     /* pulse per revolution (PPR) */
#define MOTOR_POS_ENCODER_RESOLUTION  131072     /* 2^17 - count/vong encoder */
#define MOTOR_POS_ZERO_MARGIN_DEG   15.0f 

void motor_pos_init(void);


/* Ghi lai vi tri encoder hien tai lam moc "0 do" cho motor_id -
 * goi 1 lan duy nhat khi truc vua cham LIMIT_DOWN trong luc homing.
 * Return -EAGAIN neu motor_pos chua co du lieu hop le nao (giong
 * dieu kien loi cua motor_pos_get_pulses()).
 */
int  motor_pos_capture_zero(int motor_id, float margin_deg);

/* Motion_task/homing_task co the check truoc khi RUNNING de chac
 * chan zero da duoc capture cho ca 3 truc.
 */
bool motor_pos_zero_captured(int motor_id);


/****************************************************************************
 * Name: motor_pos_update
 *
 * Description:
 *   Goi tu modbus_task moi lan doc thanh cong ca 3 thanh ghi cua 1
 *   dong co (31, 32, 36). Thread-safe.
 *
 * Input Parameters:
 *   motor_id     - 0..MOTOR_POS_COUNT-1
 *   encode_value - thanh ghi 31 (gia tri tho 1 vong)
 *   turn         - thanh ghi 32 (bo dem nua vong, dung cho unwrap)
 *   rev          - thanh ghi 36 (so vong day du, driver tu dem)
 ****************************************************************************/

void motor_pos_update(int motor_id, int32_t encode_value, int32_t turn,
                       int32_t rev);
clock_t motor_pos_get_update_tick(int motor_id);
/****************************************************************************
 * Name: motor_pos_is_fresh
 *
 * Description:
 *   Kiem tra du lieu cua 1 motor co con "moi" khong (da cap nhat
 *   trong vong max_age_ms gan day). motion_task nen goi ham nay
 *   TRUOC motor_pos_get_pulses() - neu slave Modbus im lang lien tuc,
 *   du lieu cu se bi coi la khong tin cay, tranh dung sai.
 ****************************************************************************/

bool motor_pos_is_fresh(int motor_id, uint32_t max_age_ms);

int motor_pos_get_pulses(int motor_id, float pos_des_deg,
                          int32_t *out_pulses);

#endif
