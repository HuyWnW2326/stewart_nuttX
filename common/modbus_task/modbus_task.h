/****************************************************************************
 * common/modbus_task/modbus_task.h
 *
 * Khai bao giao dien khoi dong task doc vi tri qua Modbus.
 ****************************************************************************/
#ifndef __COMMON_MODBUS_TASK_MODBUS_TASK_H
#define __COMMON_MODBUS_TASK_MODBUS_TASK_H

/****************************************************************************
 * Name: modbus_task_start
 *
 * Description:
 *   Khoi tao FreeModbus master (UART, poll thread engine), roi tao
 *   1 thread rieng lien tuc hoi tuan tu 3 slave (ID 1/2/3), doc
 *   sau thanh ghi lien tiep 31..36; dung reg31/reg32/reg36 de cap nhat
 *   motor_pos qua
 *   motor_pos_update(). Slave nao loi/timeout thi bo qua ngay, khong
 *   lam nghen 2 slave con lai; chi ghi motor_pos khi doc thanh cong.
 *
 *   Goi 1 lan duy nhat luc bringup, SAU motor_pos_init().
 *
 * Returned Value:
 *   OK neu khoi tao thanh cong, ma loi am neu that bai.
 ****************************************************************************/

int modbus_task_start(void);

#endif
