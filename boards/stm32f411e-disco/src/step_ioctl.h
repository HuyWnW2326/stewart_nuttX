/****************************************************************************
 * boards/arm/stm32/stm32f411e-disco/src/step_ioctl.h
 ****************************************************************************/
#ifndef __BOARDS_STM32F411E_DISCO_SRC_STEP_IOCTL_H
#define __BOARDS_STM32F411E_DISCO_SRC_STEP_IOCTL_H

#include <nuttx/fs/ioctl.h>
#include <stdint.h>
#include <stdbool.h>

#define STEPIOC_MOVE   _STEPIOC(0)  /* arg: FAR struct step_move_s * */
#define STEPIOC_HOME   _STEPIOC(1)  /* arg: FAR struct step_home_s *  */
#define STEPIOC_STATUS _STEPIOC(2)  /* arg: FAR bool *                */
#define STEPIOC_ESTOP  _STEPIOC(3)  /* arg: khong dung                */

struct step_move_s
{
  bool     dir_up;    /* true = xoay len, false = xoay xuong */
  uint32_t pulses;    /* so xung muc tieu (tuyet doi)         */
  uint32_t freq_hz;   /* tan so xung mong muon                */
};

struct step_home_s
{
  bool     dir_up;    /* thuong la false (xoay xuong tim limit duoi) */
  uint32_t freq_hz;   /* tan so cham hon binh thuong khi homing      */
};

/* Goi tu sensorbtn ISR moi khi mot cong tac limit doi trang thai.
 * An toan goi trong ISR context - chi doc/ghi thanh ghi, khong block.
 */
void stm32_steppulse_notify_limit(int motor_id, bool is_up, bool active);

int stm32_steppulse_initialize(void);

#endif