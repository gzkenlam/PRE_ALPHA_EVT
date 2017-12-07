/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#define DEBUG_IS_ON
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MTR_TRQ1_Pin GPIO_PIN_2
#define MTR_TRQ1_GPIO_Port GPIOE
#define MTR_SLEEP_Pin GPIO_PIN_3
#define MTR_SLEEP_GPIO_Port GPIOE
#define MTR_EN_Pin GPIO_PIN_4
#define MTR_EN_GPIO_Port GPIOE
#define MTR_STEP_Pin GPIO_PIN_5
#define MTR_STEP_GPIO_Port GPIOE
#define MTR_RST_N_Pin GPIO_PIN_6
#define MTR_RST_N_GPIO_Port GPIOE
#define NA_PC13_Pin GPIO_PIN_13
#define NA_PC13_GPIO_Port GPIOC
#define NA_PC14_Pin GPIO_PIN_14
#define NA_PC14_GPIO_Port GPIOC
#define NA_PC15_Pin GPIO_PIN_15
#define NA_PC15_GPIO_Port GPIOC
#define FEEDKEY_N_Pin GPIO_PIN_10
#define FEEDKEY_N_GPIO_Port GPIOF
#define NA_PC1_Pin GPIO_PIN_1
#define NA_PC1_GPIO_Port GPIOC
#define GAP_AD_Pin GPIO_PIN_0
#define GAP_AD_GPIO_Port GPIOA
#define PH_TEMP_AD_Pin GPIO_PIN_1
#define PH_TEMP_AD_GPIO_Port GPIOA
#define HEAD_OPEN_Pin GPIO_PIN_2
#define HEAD_OPEN_GPIO_Port GPIOA
#define HEAD_OPEN_AD_Pin GPIO_PIN_3
#define HEAD_OPEN_AD_GPIO_Port GPIOA
#define BLK_AD_Pin GPIO_PIN_4
#define BLK_AD_GPIO_Port GPIOA
#define P24V_AD_Pin GPIO_PIN_4
#define P24V_AD_GPIO_Port GPIOC
#define NA_PC5_Pin GPIO_PIN_5
#define NA_PC5_GPIO_Port GPIOC
#define PH_STR_1_Pin GPIO_PIN_0
#define PH_STR_1_GPIO_Port GPIOB
#define PH_LATCH_Pin GPIO_PIN_1
#define PH_LATCH_GPIO_Port GPIOB
#define LED_RED_N_Pin GPIO_PIN_10
#define LED_RED_N_GPIO_Port GPIOB
#define LED_GRN_N_Pin GPIO_PIN_11
#define LED_GRN_N_GPIO_Port GPIOB
#define NA_PB12_Pin GPIO_PIN_12
#define NA_PB12_GPIO_Port GPIOB
#define ID0_Pin GPIO_PIN_13
#define ID0_GPIO_Port GPIOB
#define ID1_Pin GPIO_PIN_14
#define ID1_GPIO_Port GPIOB
#define ID2_Pin GPIO_PIN_15
#define ID2_GPIO_Port GPIOB
#define NA_PD11_Pin GPIO_PIN_11
#define NA_PD11_GPIO_Port GPIOD
#define EMIT_PWM_Pin GPIO_PIN_12
#define EMIT_PWM_GPIO_Port GPIOD
#define NA_PD13_Pin GPIO_PIN_13
#define NA_PD13_GPIO_Port GPIOD
#define NA_PG3_Pin GPIO_PIN_3
#define NA_PG3_GPIO_Port GPIOG
#define NA_PG6_Pin GPIO_PIN_6
#define NA_PG6_GPIO_Port GPIOG
#define NA_PG7_Pin GPIO_PIN_7
#define NA_PG7_GPIO_Port GPIOG
#define PH_POWER_OFF_Pin GPIO_PIN_6
#define PH_POWER_OFF_GPIO_Port GPIOC
#define NA_PC7_Pin GPIO_PIN_7
#define NA_PC7_GPIO_Port GPIOC
#define WIFI_IRQ_N_Pin GPIO_PIN_8
#define WIFI_IRQ_N_GPIO_Port GPIOC
#define WIFI_CHIP_EN_Pin GPIO_PIN_9
#define WIFI_CHIP_EN_GPIO_Port GPIOC
#define PH_POWER_ON_Pin GPIO_PIN_8
#define PH_POWER_ON_GPIO_Port GPIOA
#define USB_EN_Pin GPIO_PIN_10
#define USB_EN_GPIO_Port GPIOA
#define WIFI_RST_N_Pin GPIO_PIN_4
#define WIFI_RST_N_GPIO_Port GPIOD
#define WIFI_WAKE_Pin GPIO_PIN_5
#define WIFI_WAKE_GPIO_Port GPIOD
#define NA_PD6_Pin GPIO_PIN_6
#define NA_PD6_GPIO_Port GPIOD
#define NA_PD7_Pin GPIO_PIN_7
#define NA_PD7_GPIO_Port GPIOD
#define MTR_USM0_Pin GPIO_PIN_9
#define MTR_USM0_GPIO_Port GPIOG
#define MTR_USM1_Pin GPIO_PIN_10
#define MTR_USM1_GPIO_Port GPIOG
#define NA_PB5_Pin GPIO_PIN_5
#define NA_PB5_GPIO_Port GPIOB
#define MTR_DIR_Pin GPIO_PIN_7
#define MTR_DIR_GPIO_Port GPIOB
#define MTR_HOME_Pin GPIO_PIN_8
#define MTR_HOME_GPIO_Port GPIOB
#define MTR_CURREF_Pin GPIO_PIN_9
#define MTR_CURREF_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
