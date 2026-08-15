/****************************************************************************
 * common/motion_task/motion_task.h
 *
 * Khai bao giao dien khoi dong task dieu khien chuyen dong.
 ****************************************************************************/

#ifndef __COMMON_MOTION_TASK_H
#define __COMMON_MOTION_TASK_H

/****************************************************************************
 * Name: motion_task_start
 *
 * Description:
 *   Tao motion_task sau khi cac driver va system_state da khoi tao. Task
 *   chay lien tuc 100 Hz va khong gui lenh khi he thong khong RUNNING.
 *
 * Returned Value:
 *   OK khi thanh cong; ma loi am neu khong tao duoc task.
 ****************************************************************************/

int motion_task_start(void);

#endif /* __COMMON_MOTION_TASK_H */
