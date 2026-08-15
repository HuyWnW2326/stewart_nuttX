/****************************************************************************
 * boards/arm/stm32/stm32f411e-disco/src/stm32_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/debug.h>
#include <nuttx/arch.h>
#include <syslog.h>

#include "stm32.h"
#include "stm32f411e-disco.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* them moi: cho phan cung (GPIO, pull-up cam bien, nguon...) on dinh
 * truoc khi bat dau nhan ngat EXTI cho limit switch/nut bam. Neu
 * enable ngat ngay luc dien ap chan GPIO chua settle hoac dang trong
 * qua trinh cap nguon, EXTI co the bat 1 canh gia (spurious edge)
 * ngay tai thoi diem cau hinh, du chan khong thuc su doi muc.
 */

#define SENSORBTN_INIT_DELAY_MS   10000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_STM32_TIM3
  /* Register the step/dir pulse generator for the 3 servo motors
   * (TIM3 CH1/CH2/CH3) -> /dev/step0, /dev/step1, /dev/step2
   */

  ret = stm32_steppulse_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_steppulse_initialize() failed: %d\n",
             ret);
      return ret;
    }
#endif

  /* them moi: doi phan cung on dinh truoc khi bat dau cau hinh + bat
   * ngat EXTI cho limit switch/nut bam (xem SENSORBTN_INIT_DELAY_MS
   * o tren). stm32_sensorbtn_initialize() lam gop ca config GPIO va
   * enable interrupt trong cung 1 lenh stm32_gpiosetevent(), nen delay
   * dat truoc loi goi ham nay la du - khong can sua stm32_sensorbtn.c.
   */

  up_mdelay(SENSORBTN_INIT_DELAY_MS);

  /* Register EXTI interrupt handlers for the 6 motor limit switches
   * and 3 control buttons (start/stop/reset).
   */

  ret = stm32_sensorbtn_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_sensorbtn_initialize() failed: %d\n",
             ret);
      return ret;               /* <-- thêm return, trước đây thiếu */
    }

#ifdef CONFIG_STM32_TIM1
  /* Register PWM input-capture reader for PX4 setpoints
   * (TIM1 CH1/CH2/CH3) -> /dev/pwmcap0, /dev/pwmcap1, /dev/pwmcap2
   */

  ret = stm32_pwmcapture_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwmcapture_initialize() failed: %d\n",
             ret);
      return ret;
    }
#endif

  return ret;
}