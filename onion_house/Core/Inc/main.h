/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define KTIR_1_Pin GPIO_PIN_0
#define KTIR_1_GPIO_Port GPIOA
#define KTIR_2_Pin GPIO_PIN_1
#define KTIR_2_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOA
#define KTIR_3_Pin GPIO_PIN_6
#define KTIR_3_GPIO_Port GPIOA
#define KTIR_4_Pin GPIO_PIN_7
#define KTIR_4_GPIO_Port GPIOA
#define SERVO_RIGHT_Pin GPIO_PIN_0
#define SERVO_RIGHT_GPIO_Port GPIOB
#define SERVO_LEFT_Pin GPIO_PIN_1
#define SERVO_LEFT_GPIO_Port GPIOB
#define COLOR_RIGHT_SCL_Pin GPIO_PIN_10
#define COLOR_RIGHT_SCL_GPIO_Port GPIOB
#define MOTOR_LEFT_A_Pin GPIO_PIN_8
#define MOTOR_LEFT_A_GPIO_Port GPIOA
#define MOTOR_LEFT_B_Pin GPIO_PIN_9
#define MOTOR_LEFT_B_GPIO_Port GPIOA
#define MOTOR_RIGHT_A_Pin GPIO_PIN_10
#define MOTOR_RIGHT_A_GPIO_Port GPIOA
#define MOTOR_RIGHT_B_Pin GPIO_PIN_11
#define MOTOR_RIGHT_B_GPIO_Port GPIOA
#define COLOR_RIGHT_SDA_Pin GPIO_PIN_3
#define COLOR_RIGHT_SDA_GPIO_Port GPIOB
#define ENCODER_RIGHT_1_Pin GPIO_PIN_6
#define ENCODER_RIGHT_1_GPIO_Port GPIOB
#define ENCODER_RIGHT_2_Pin GPIO_PIN_7
#define ENCODER_RIGHT_2_GPIO_Port GPIOB
#define COLOR_LEFT_SCL_Pin GPIO_PIN_8
#define COLOR_LEFT_SCL_GPIO_Port GPIOB
#define COLOR_LEFT_SDA_Pin GPIO_PIN_9
#define COLOR_LEFT_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
