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
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define TRIG3_Pin GPIO_PIN_0
#define TRIG3_GPIO_Port GPIOB
#define TIM14_CH1_ECHO_Pin GPIO_PIN_1
#define TIM14_CH1_ECHO_GPIO_Port GPIOB
#define TIM1_CH1_HCSR1_ECHO_Pin GPIO_PIN_8
#define TIM1_CH1_HCSR1_ECHO_GPIO_Port GPIOA
#define TRIG1_Pin GPIO_PIN_9
#define TRIG1_GPIO_Port GPIOA
#define TRIG2_Pin GPIO_PIN_7
#define TRIG2_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define TIM3_CH1_Left_Motor_PWM_Pin GPIO_PIN_4
#define TIM3_CH1_Left_Motor_PWM_GPIO_Port GPIOB
#define TIM3_CH2_Right_Motor_PWM_Pin GPIO_PIN_5
#define TIM3_CH2_Right_Motor_PWM_GPIO_Port GPIOB
#define TIM17_CH1_ECHO_Pin GPIO_PIN_9
#define TIM17_CH1_ECHO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
