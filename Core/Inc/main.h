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
#define RECEIVEULTRASONICLEFT_Pin GPIO_PIN_1
#define RECEIVEULTRASONICLEFT_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_0
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_1
#define IN2_GPIO_Port GPIOA
#define RECEIVEULTRASONICFRONT_Pin GPIO_PIN_4
#define RECEIVEULTRASONICFRONT_GPIO_Port GPIOA
#define SENDULTRASONICFRONT_Pin GPIO_PIN_5
#define SENDULTRASONICFRONT_GPIO_Port GPIOA
#define IN3_Pin GPIO_PIN_6
#define IN3_GPIO_Port GPIOA
#define IN4_Pin GPIO_PIN_7
#define IN4_GPIO_Port GPIOA
#define RECEIVEULTRASONICRIGHT_Pin GPIO_PIN_0
#define RECEIVEULTRASONICRIGHT_GPIO_Port GPIOB
#define SENDULTRASONICLEFT_Pin GPIO_PIN_7
#define SENDULTRASONICLEFT_GPIO_Port GPIOC
#define IN1BACK_Pin GPIO_PIN_8
#define IN1BACK_GPIO_Port GPIOA
#define IN2BACK_Pin GPIO_PIN_9
#define IN2BACK_GPIO_Port GPIOA
#define SENDULTRASONICRIGHT_Pin GPIO_PIN_6
#define SENDULTRASONICRIGHT_GPIO_Port GPIOB
#define IN3BACK_Pin GPIO_PIN_8
#define IN3BACK_GPIO_Port GPIOB
#define IN4BACK_Pin GPIO_PIN_9
#define IN4BACK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
