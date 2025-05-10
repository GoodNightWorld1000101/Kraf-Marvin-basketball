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
#include "stm32g4xx_hal.h"

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
#define IR_SENSOR_Pin GPIO_PIN_1
#define IR_SENSOR_GPIO_Port GPIOF
#define ENCODER2_Pin GPIO_PIN_0
#define ENCODER2_GPIO_Port GPIOA
#define ENCODER2B_Pin GPIO_PIN_1
#define ENCODER2B_GPIO_Port GPIOA
#define SERVO_CATCHER_Pin GPIO_PIN_2
#define SERVO_CATCHER_GPIO_Port GPIOA
#define SERVO_ANGLE_Pin GPIO_PIN_3
#define SERVO_ANGLE_GPIO_Port GPIOA
#define ENCODER3_Pin GPIO_PIN_4
#define ENCODER3_GPIO_Port GPIOA
#define MOTOR3_DIRECTION_Pin GPIO_PIN_5
#define MOTOR3_DIRECTION_GPIO_Port GPIOA
#define ENCODER3B_Pin GPIO_PIN_6
#define ENCODER3B_GPIO_Port GPIOA
#define MOTOR2_DIRECTION_Pin GPIO_PIN_7
#define MOTOR2_DIRECTION_GPIO_Port GPIOA
#define MOTOR1_DIRECTION_Pin GPIO_PIN_0
#define MOTOR1_DIRECTION_GPIO_Port GPIOB
#define MOTOR3_PWM_Pin GPIO_PIN_8
#define MOTOR3_PWM_GPIO_Port GPIOA
#define MOTOR2_PWM_Pin GPIO_PIN_9
#define MOTOR2_PWM_GPIO_Port GPIOA
#define MOTOR1_PWM_Pin GPIO_PIN_10
#define MOTOR1_PWM_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#define nSLEEP_Pin GPIO_PIN_4
#define nSLEEP_GPIO_Port GPIOB
#define encoder_1_Pin GPIO_PIN_6
#define encoder_1_GPIO_Port GPIOB
#define encoder_1B_Pin GPIO_PIN_7
#define encoder_1B_GPIO_Port GPIOB
#define THROWER_PWM_Pin GPIO_PIN_8
#define THROWER_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
