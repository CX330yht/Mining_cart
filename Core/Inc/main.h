/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define NRF_CSN_Pin GPIO_PIN_4
#define NRF_CSN_GPIO_Port GPIOE
#define ARM_LIMIT_MIN_Pin GPIO_PIN_7
#define ARM_LIMIT_MIN_GPIO_Port GPIOI
#define SERVO_B_PWM_Pin GPIO_PIN_5
#define SERVO_B_PWM_GPIO_Port GPIOD
#define ARM_LIMIT_MAX_Pin GPIO_PIN_2
#define ARM_LIMIT_MAX_GPIO_Port GPIOI
#define ARM_IN2_Pin GPIO_PIN_9
#define ARM_IN2_GPIO_Port GPIOI
#define ARM_ENC_A_Pin GPIO_PIN_0
#define ARM_ENC_A_GPIO_Port GPIOF
#define ARM_ENC_A_EXTI_IRQn EXTI0_IRQn
#define TB2_BIN1_Pin GPIO_PIN_0
#define TB2_BIN1_GPIO_Port GPIOI
#define TB1_BIN2_Pin GPIO_PIN_1
#define TB1_BIN2_GPIO_Port GPIOF
#define TB2_AIN1_Pin GPIO_PIN_12
#define TB2_AIN1_GPIO_Port GPIOH
#define ARM_ENC_B_Pin GPIO_PIN_10
#define ARM_ENC_B_GPIO_Port GPIOF
#define ARM_ENC_B_EXTI_IRQn EXTI15_10_IRQn
#define TB1_BIN1_Pin GPIO_PIN_15
#define TB1_BIN1_GPIO_Port GPIOD
#define NRF_CE_Pin GPIO_PIN_0
#define NRF_CE_GPIO_Port GPIOC
#define TB2_BIN2_Pin GPIO_PIN_1
#define TB2_BIN2_GPIO_Port GPIOC
#define TB1_STBY_Pin GPIO_PIN_2
#define TB1_STBY_GPIO_Port GPIOC
#define TB1_AIN2_Pin GPIO_PIN_3
#define TB1_AIN2_GPIO_Port GPIOC
#define TB1_AIN1_Pin GPIO_PIN_14
#define TB1_AIN1_GPIO_Port GPIOD
#define ARM_IN1_Pin GPIO_PIN_4
#define ARM_IN1_GPIO_Port GPIOA
#define TB2_STBY_Pin GPIO_PIN_4
#define TB2_STBY_GPIO_Port GPIOC
#define SERVO_A_PWM_Pin GPIO_PIN_2
#define SERVO_A_PWM_GPIO_Port GPIOA
#define ARM_STBY_Pin GPIO_PIN_5
#define ARM_STBY_GPIO_Port GPIOA
#define TB2_AIN2_Pin GPIO_PIN_5
#define TB2_AIN2_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
