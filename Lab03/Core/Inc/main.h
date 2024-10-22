/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define right_limit_switch_Pin GPIO_PIN_3
#define right_limit_switch_GPIO_Port GPIOF
#define YDirection_Pin GPIO_PIN_4
#define YDirection_GPIO_Port GPIOF
#define enableY_Pin GPIO_PIN_5
#define enableY_GPIO_Port GPIOF
#define Force_Sensor_Pin GPIO_PIN_0
#define Force_Sensor_GPIO_Port GPIOA
#define XPulses_Pin GPIO_PIN_3
#define XPulses_GPIO_Port GPIOA
#define debug_LED_Pin GPIO_PIN_0
#define debug_LED_GPIO_Port GPIOB
#define XDirection_Pin GPIO_PIN_7
#define XDirection_GPIO_Port GPIOE
#define left_limit_switch_Pin GPIO_PIN_11
#define left_limit_switch_GPIO_Port GPIOE
#define enableX_Pin GPIO_PIN_15
#define enableX_GPIO_Port GPIOE
#define enableZ_Pin GPIO_PIN_10
#define enableZ_GPIO_Port GPIOD
#define buttom_limit_switch_Pin GPIO_PIN_12
#define buttom_limit_switch_GPIO_Port GPIOD
#define top_limit_switch_Pin GPIO_PIN_13
#define top_limit_switch_GPIO_Port GPIOD
#define ZPulses_Pin GPIO_PIN_15
#define ZPulses_GPIO_Port GPIOD
#define ZDirection_Pin GPIO_PIN_5
#define ZDirection_GPIO_Port GPIOG
#define YPulses_Pin GPIO_PIN_4
#define YPulses_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
