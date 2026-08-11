/****************************************************************************
 * common/motor_pos/motor_pos.h 
 ****************************************************************************/
#ifndef __COMMON_MOTOR_POS_MOTOR_POS_H
#define __COMMON_MOTOR_POS_MOTOR_POS_H

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_POS_COUNT               3

#define MOTOR_POS_GEAR_RATIO          100
#define MOTOR_POS_PULSE               10000     /* pulse per revolution (PPR) */
#define MOTOR_POS_ENCODER_RESOLUTION  131072     /* 2^17 - count/vong encoder */

void motor_pos_init(void);

/****************************************************************************
 * Name: motor_pos_update
 *
 * Description:
 *   Goi tu modbus_task moi lan doc thanh cong ca 3 thanh ghi cua 1
 *   dong co (31, 32, 35). Thread-safe.
 *
 * Input Parameters:
 *   motor_id     - 0..MOTOR_POS_COUNT-1
 *   encode_value - thanh ghi 31 (gia tri tho 1 vong)
 *   turn         - thanh ghi 32 (bo dem nua vong, dung cho unwrap)
 *   rev          - thanh ghi 35 (so vong day du, driver tu dem)
 ****************************************************************************/

void motor_pos_update(int motor_id, int32_t encode_value, int32_t turn,
                       int32_t rev);

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