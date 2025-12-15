/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define pwmi_speed_fl_Pin GPIO_PIN_6
#define pwmi_speed_fl_GPIO_Port GPIOF
#define pwmi_speed_fr_Pin GPIO_PIN_7
#define pwmi_speed_fr_GPIO_Port GPIOF
#define pwmi_sped_rl_Pin GPIO_PIN_8
#define pwmi_sped_rl_GPIO_Port GPIOF
#define pwmi_speed_rr_Pin GPIO_PIN_9
#define pwmi_speed_rr_GPIO_Port GPIOF
#define ai_Apps1_Pin GPIO_PIN_0
#define ai_Apps1_GPIO_Port GPIOC
#define ai_Apps2_Pin GPIO_PIN_1
#define ai_Apps2_GPIO_Port GPIOC
#define ai_BrakePress_Pin GPIO_PIN_2
#define ai_BrakePress_GPIO_Port GPIOC
#define ai_SteerAng_Pin GPIO_PIN_3
#define ai_SteerAng_GPIO_Port GPIOC
#define ai_WaterTemp1_Pin GPIO_PIN_4
#define ai_WaterTemp1_GPIO_Port GPIOA
#define ai_WaterTemp2_Pin GPIO_PIN_5
#define ai_WaterTemp2_GPIO_Port GPIOA
#define ai_WaterPress1_Pin GPIO_PIN_6
#define ai_WaterPress1_GPIO_Port GPIOA
#define ai_WaterPress2_Pin GPIO_PIN_7
#define ai_WaterPress2_GPIO_Port GPIOA
#define do_sdc_hs_Pin GPIO_PIN_11
#define do_sdc_hs_GPIO_Port GPIOF
#define di_SD_Detect_Pin GPIO_PIN_7
#define di_SD_Detect_GPIO_Port GPIOC
#define CSN_Pin GPIO_PIN_12
#define CSN_GPIO_Port GPIOG
#define CE_Pin GPIO_PIN_13
#define CE_GPIO_Port GPIOG
#define IRQ_Pin GPIO_PIN_14
#define IRQ_GPIO_Port GPIOG
#define IRQ_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
