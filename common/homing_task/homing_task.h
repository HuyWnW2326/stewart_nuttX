/****************************************************************************
 * common/homing_task/homing_task.h
 *
 * Khai bao entry point cua chu trinh homing ba truc.
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
 * Ham khong dung argc/argv. Goc nang cua tung truc duoc co dinh trong
 * mang g_homing_lift_deg[] khai bao trong homing_task.c.
 */

int homing_task_main(int argc, char *argv[]);

#endif /* __COMMON_HOMING_TASK_H */
