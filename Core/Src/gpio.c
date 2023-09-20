/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "motor_Control.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|Motor2_INA_Pin|Motor2_INB_Pin|Motor1_INB_Pin
                          |Motor1_INA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Motor3_INB_Pin|Motor3_INA_Pin|Motor4_INB_Pin|Motor4_INA_Pin
                          |Motor5_INA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor5_INB_GPIO_Port, Motor5_INB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                           PCPin */
  GPIO_InitStruct.Pin = LED1_Pin|Motor2_INA_Pin|Motor2_INB_Pin|Motor1_INB_Pin
                          |Motor1_INA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                           PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = Motor3_Encoder1_Pin|Motor3_Encoder2_Pin|Motor1_Encoder1_Pin|Motor1_Encoder2_Pin
                          |Motor5_Encoder2_Pin|Motor4_Encoder1_Pin|Motor4_Encoder2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = Motor2_Encoder1_Pin|Motor2_Encoder2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = Motor3_INB_Pin|Motor3_INA_Pin|Motor4_INB_Pin|Motor4_INA_Pin
                          |Motor5_INA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Motor5_Encoder1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Motor5_Encoder1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Motor5_INB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor5_INB_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
	//---------------- Pinky Encoder ----------------
		case Motor1_Encoder1_Pin:
			switch (Fingers_Status.Pinky.Direction) {
				case Open:
					Fingers_Status.Pinky.Encoder++;
					//for remove integration error in read encoder
					if(Fingers_Status.Pinky.Encoder>Max_Encoder_Pinky)
						Fingers_Status.Pinky.Encoder=Max_Encoder_Pinky;
						break;
				case Close:
					Fingers_Status.Pinky.Encoder--;
					//for remove integration error in read encoder
					if(Fingers_Status.Pinky.Encoder>65400 || Fingers_Status.Pinky.Encoder<100)
						Fingers_Status.Pinky.Encoder=0;
					break;
				default:
					break;
			}
			break;
		case Motor1_Encoder2_Pin:
			switch (Fingers_Status.Pinky.Direction) {
				case Open:
					Fingers_Status.Pinky.Encoder++;
					//for remove integration error in read encoder
					if(Fingers_Status.Pinky.Encoder>Max_Encoder_Pinky)
						Fingers_Status.Pinky.Encoder=Max_Encoder_Pinky;
					break;
				case Close:
					Fingers_Status.Pinky.Encoder--;
					//for remove integration error in read encoder
					if(Fingers_Status.Pinky.Encoder>65400 || Fingers_Status.Pinky.Encoder<100)
						Fingers_Status.Pinky.Encoder=0;
					break;
				default:
					break;
			}
			break;
	//---------------- Ring Encoder ----------------
		case Motor2_Encoder1_Pin:
			switch (Fingers_Status.Ring.Direction) {
				case Open:
					Fingers_Status.Ring.Encoder++;
					//for remove integration error in read encoder
					if(Fingers_Status.Ring.Encoder>Max_Encoder_Ring)
						Fingers_Status.Ring.Encoder=Max_Encoder_Ring;
					break;
				case Close:
					Fingers_Status.Ring.Encoder--;
					//for remove integration error in read encoder
					if(Fingers_Status.Ring.Encoder>65400 || Fingers_Status.Ring.Encoder<100)
						Fingers_Status.Ring.Encoder=0;
					break;
				default:
					break;
			}
			break;
		case Motor2_Encoder2_Pin:
			switch (Fingers_Status.Ring.Direction) {
				case Open:
					Fingers_Status.Ring.Encoder++;
					//for remove integration error in read encoder
					if(Fingers_Status.Ring.Encoder>Max_Encoder_Ring)
						Fingers_Status.Ring.Encoder=Max_Encoder_Ring;
					break;
				case Close:
					Fingers_Status.Ring.Encoder--;
					//for remove integration error in read encoder
					if(Fingers_Status.Ring.Encoder>65400 || Fingers_Status.Ring.Encoder<100)
						Fingers_Status.Ring.Encoder=0;
					break;
				default:
					break;
			}
			break;
	//---------------- Middle Encoder ----------------
		case Motor3_Encoder1_Pin:
			switch (Fingers_Status.Middle.Direction) {
				case Open:
					Fingers_Status.Middle.Encoder++;
					//for remove integration error in read encoder
					if(Fingers_Status.Middle.Encoder>Max_Encoder_Middle)
						Fingers_Status.Middle.Encoder=Max_Encoder_Middle;
					break;
				case Close:
					Fingers_Status.Middle.Encoder--;
					//for remove integration error in read encoder
					if(Fingers_Status.Middle.Encoder>65400 || Fingers_Status.Middle.Encoder<100)
					Fingers_Status.Middle.Encoder=0;
					break;
				default:
					break;
			}
			break;
		case Motor3_Encoder2_Pin:
			switch (Fingers_Status.Middle.Direction) {
				case Open:
					Fingers_Status.Middle.Encoder++;
					//for remove integration error in read encoder
					if(Fingers_Status.Middle.Encoder>Max_Encoder_Middle)
						Fingers_Status.Middle.Encoder=Max_Encoder_Middle;
					break;
				case Close:
					Fingers_Status.Middle.Encoder--;
					//for remove integration error in read encoder
					if(Fingers_Status.Middle.Encoder>65400 || Fingers_Status.Middle.Encoder<100)
					Fingers_Status.Middle.Encoder=0;
					break;
				default:
					break;
			}
			break;
	//---------------- Index Encoder ----------------
		case Motor4_Encoder1_Pin:
			switch (Fingers_Status.Index.Direction) {
				case Open:
					Fingers_Status.Index.Encoder++;
					//for remove integration error in read encoder
					if(Fingers_Status.Index.Encoder>Max_Encoder_Index)
						Fingers_Status.Index.Encoder=Max_Encoder_Index;
					break;
				case Close:
					Fingers_Status.Index.Encoder--;
					//for remove integration error in read encoder
					if(Fingers_Status.Index.Encoder>65400 || Fingers_Status.Index.Encoder<100)
					Fingers_Status.Index.Encoder=0;
					break;
				default:
					break;
			}
			break;
		case Motor4_Encoder2_Pin:
			switch (Fingers_Status.Index.Direction) {
				case Open:
					Fingers_Status.Index.Encoder++;
					//for remove integration error in read encoder
					if(Fingers_Status.Index.Encoder>Max_Encoder_Index)
						Fingers_Status.Index.Encoder=Max_Encoder_Index;
					break;
				case Close:
					Fingers_Status.Index.Encoder--;
					//for remove integration error in read encoder
					if(Fingers_Status.Index.Encoder>65400 || Fingers_Status.Index.Encoder<100)
					Fingers_Status.Index.Encoder=0;
					break;
				default:
					break;
			}
			break;
	//---------------- Thumb Encoder ----------------
		case Motor5_Encoder2_Pin:
			switch (Fingers_Status.Thumb.Direction) {
				case Open:
					Fingers_Status.Thumb.Encoder++;
					break;
				case Close:
					Fingers_Status.Thumb.Encoder--;
					//for remove integration error in read encoder
					if(Fingers_Status.Thumb.Encoder>65400 || Fingers_Status.Thumb.Encoder<100)
					Fingers_Status.Thumb.Encoder=0;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}
/* USER CODE END 2 */
