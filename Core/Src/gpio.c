/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENDULTRASONICFRONT_GPIO_Port, SENDULTRASONICFRONT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENDULTRASONICLEFT_GPIO_Port, SENDULTRASONICLEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENDULTRASONICRIGHT_GPIO_Port, SENDULTRASONICRIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RECEIVEULTRASONICLEFT_Pin */
  GPIO_InitStruct.Pin = RECEIVEULTRASONICLEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RECEIVEULTRASONICLEFT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RECEIVEULTRASONICFRONT_Pin */
  GPIO_InitStruct.Pin = RECEIVEULTRASONICFRONT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RECEIVEULTRASONICFRONT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENDULTRASONICFRONT_Pin */
  GPIO_InitStruct.Pin = SENDULTRASONICFRONT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENDULTRASONICFRONT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RECEIVEULTRASONICRIGHT_Pin */
  GPIO_InitStruct.Pin = RECEIVEULTRASONICRIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RECEIVEULTRASONICRIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENDULTRASONICLEFT_Pin */
  GPIO_InitStruct.Pin = SENDULTRASONICLEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENDULTRASONICLEFT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENDULTRASONICRIGHT_Pin */
  GPIO_InitStruct.Pin = SENDULTRASONICRIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENDULTRASONICRIGHT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
