/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM17_CH1_Pin GPIO_PIN_9
#define TIM17_CH1_GPIO_Port GPIOB
#define TRIG_Pin GPIO_PIN_15
#define TRIG_GPIO_Port GPIOC
#define TIM2_CH1_Pin GPIO_PIN_0
#define TIM2_CH1_GPIO_Port GPIOA
#define TIM2_CH2_Pin GPIO_PIN_1
#define TIM2_CH2_GPIO_Port GPIOA
#define TIM14_CH1_Pin GPIO_PIN_4
#define TIM14_CH1_GPIO_Port GPIOA
#define TEST_LED_Pin GPIO_PIN_5
#define TEST_LED_GPIO_Port GPIOA
#define TIM3_CH1_Pin GPIO_PIN_6
#define TIM3_CH1_GPIO_Port GPIOA
#define TIM3_CH2_Pin GPIO_PIN_7
#define TIM3_CH2_GPIO_Port GPIOA
#define TIM1_CH1_Pin GPIO_PIN_8
#define TIM1_CH1_GPIO_Port GPIOA
#define I2C2_SCL_Pin GPIO_PIN_11
#define I2C2_SCL_GPIO_Port GPIOA
#define SYS_SWDIO_Pin GPIO_PIN_13
#define SYS_SWDIO_GPIO_Port GPIOA
#define SYS_SWCLK_Pin GPIO_PIN_14
#define SYS_SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
