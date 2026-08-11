/****************************************************************************
 * common/homing_task/homing_task.h
 ****************************************************************************/

#ifndef __COMMON_HOMING_TASK_H
#define __COMMON_HOMING_TASK_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Entry point cho task_create().
 * Block toi khi homing xong ca 3 truc va phase chuyen sang
 * SYS_PHASE_WAIT_START. Sau do task tu thoat.
 *
 * Tham so argv (optional, truyen tu stewart_payload_main):
 *   argv[0] = ten task (bo qua)
 *   argv[1] = goc nang motor 0 (do, float string, vi du "19.0")
 *   argv[2] = goc nang motor 1 (do, float string, vi du "20.0")
 *   argv[3] = goc nang motor 2 (do, float string, vi du "19.5")
 *
 * Neu khong truyen hoac truyen thieu, motor tuong ung dung
 * HOMING_LIFT_DEFAULT_DEG (20.0 do).
 *
 * Vi du goi tu NSH:
 *   nsh> stewart 19.0 20.0 19.5
 */

int homing_task_main(int argc, char *argv[]);

#endif /* __COMMON_HOMING_TASK_H */