/****************************************************************************
 * boards/arm/stm32/stm32f411e-disco/include/board.h
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

#ifndef __BOARDS_ARM_STM32_STM32F411E_DISCO_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_STM32F411E_DISCO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(192)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_4
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(8)

#define STM32_SYSCLK_FREQUENCY  96000000ul

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* TIM3 is on APB1 -> timer clock is 2x PCLK1 */

#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM3_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)

#define STM32_APB2_TIM1_CLKIN   (96000000ul)
#define BOARD_TIM1_FREQUENCY    (2 * STM32_PCLK2_FREQUENCY)

/* Alternate function pin selections ****************************************/

/* USART1: PB6(TX) / PB7(RX) */

#define GPIO_USART1_RX (GPIO_USART1_RX_2|GPIO_SPEED_100MHz)    /* PB7 */
#define GPIO_USART1_TX (GPIO_USART1_TX_2|GPIO_SPEED_100MHz)    /* PB6 */

/* USART2: PA2(TX) / PA3(RX) — console */

#define GPIO_USART2_RX   (GPIO_USART2_RX_1|GPIO_SPEED_100MHz)  /* PA3 */
#define GPIO_USART2_TX   (GPIO_USART2_TX_1|GPIO_SPEED_100MHz)  /* PA2 */

/* USART6: PC6(TX) / PC7(RX) */

#define GPIO_USART6_RX   (GPIO_USART6_RX_1|GPIO_SPEED_100MHz)  /* PC7 */
#define GPIO_USART6_TX   (GPIO_USART6_TX_1|GPIO_SPEED_100MHz)  /* PC6 */

/* Motor pulse outputs (TIM3) */

#define GPIO_TIM3_CH1OUT  (GPIO_TIM3_CH1OUT_2|GPIO_SPEED_50MHz)  /* PB4 - Motor1 pulse */
#define GPIO_TIM3_CH2OUT  (GPIO_TIM3_CH2OUT_1|GPIO_SPEED_50MHz)  /* PA7 - Motor2 pulse */
#define GPIO_TIM3_CH3OUT  (GPIO_TIM3_CH3OUT_1|GPIO_SPEED_50MHz)  /* PB0 - Motor3 pulse */

/* Motor DIR pins (plain GPIO output) */

#define GPIO_MOTOR1_DIR  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN4)  /* PC4 */
#define GPIO_MOTOR2_DIR  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)  /* PC1 */
#define GPIO_MOTOR3_DIR  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN2)  /* PC2 */

/* PWM input capture from PX4 (TIM1) */

#define GPIO_TIM1_CH1IN   GPIO_TIM1_CH1IN_1   /* PA8 */
#define GPIO_TIM1_CH2IN   GPIO_TIM1_CH2IN_2   /* PE11 */
#define GPIO_TIM1_CH3IN   GPIO_TIM1_CH3IN_1   /* PA10 */

/* Sensor EXTI inputs */

#define GPIO_MOTOR1_LIMIT_UP    (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTE|GPIO_PIN6)   /* PE6  */
#define GPIO_MOTOR1_LIMIT_DOWN  (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTC|GPIO_PIN7)   /* PC7  */
#define GPIO_MOTOR2_LIMIT_UP    (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTC|GPIO_PIN8)   /* PC8  */
#define GPIO_MOTOR2_LIMIT_DOWN  (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTC|GPIO_PIN9)   /* PC9  */
#define GPIO_MOTOR3_LIMIT_UP    (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTD|GPIO_PIN10)  /* PD10 */
#define GPIO_MOTOR3_LIMIT_DOWN  (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTD|GPIO_PIN11)  /* PD11 */

/* Buttons — manual EXTI */

#define GPIO_BTN_STARTSTOP  (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTB|GPIO_PIN12)  /* PB12 */
#define GPIO_BTN_EMERGENCY  (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)  /* PC13 */
#define GPIO_BTN_RESTART    (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTB|GPIO_PIN14)  /* PB14 */

/* Driver enable*/

#define GPIO_MOTOR1_SON  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN6)  /* PD6 */
#define GPIO_MOTOR2_SON  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN3)  /* PB3 */
#define GPIO_MOTOR3_SON  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)  /* PB5 */

#endif /* __BOARDS_ARM_STM32_STM32F411E_DISCO_INCLUDE_BOARD_H */