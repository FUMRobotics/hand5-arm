/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "motor_Control.h"
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

static uint32_t HAL_RCC_ADC_CLK_ENABLED=0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    HAL_RCC_ADC_CLK_ENABLED++;
    if(HAL_RCC_ADC_CLK_ENABLED==1){
      __HAL_RCC_ADC_CLK_ENABLE();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC2     ------> ADC1_IN3
    PC3     ------> ADC1_IN4
    PA2     ------> ADC1_IN7
    PA4     ------> ADC1_IN9
    PA5     ------> ADC1_IN10
    */
    GPIO_InitStruct.Pin = Iprop_Motor4_Pin|Iprop_Motor5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Iprop_Motor3_Pin|Iprop_Motor2_Pin|Iprop_Motor1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    HAL_RCC_ADC_CLK_ENABLED++;
    if(HAL_RCC_ADC_CLK_ENABLED==1){
      __HAL_RCC_ADC_CLK_ENABLE();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC0     ------> ADC2_IN1
    PC1     ------> ADC2_IN2
    PA1     ------> ADC2_IN6
    PA3     ------> ADC2_IN8
    PA6     ------> ADC2_IN11
    PA7     ------> ADC2_IN12
    */
    GPIO_InitStruct.Pin = ACS_Motor5_Pin|ACS_Motor4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ACS_Motor3_Pin|ACS_Motor2_Pin|ACS_Motor1_Pin|battery_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC_CLK_ENABLED--;
    if(HAL_RCC_ADC_CLK_ENABLED==0){
      __HAL_RCC_ADC_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PC2     ------> ADC1_IN3
    PC3     ------> ADC1_IN4
    PA2     ------> ADC1_IN7
    PA4     ------> ADC1_IN9
    PA5     ------> ADC1_IN10
    */
    HAL_GPIO_DeInit(GPIOC, Iprop_Motor4_Pin|Iprop_Motor5_Pin);

    HAL_GPIO_DeInit(GPIOA, Iprop_Motor3_Pin|Iprop_Motor2_Pin|Iprop_Motor1_Pin);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC_CLK_ENABLED--;
    if(HAL_RCC_ADC_CLK_ENABLED==0){
      __HAL_RCC_ADC_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PC0     ------> ADC2_IN1
    PC1     ------> ADC2_IN2
    PA1     ------> ADC2_IN6
    PA3     ------> ADC2_IN8
    PA6     ------> ADC2_IN11
    PA7     ------> ADC2_IN12
    */
    HAL_GPIO_DeInit(GPIOC, ACS_Motor5_Pin|ACS_Motor4_Pin);

    HAL_GPIO_DeInit(GPIOA, ACS_Motor3_Pin|ACS_Motor2_Pin|ACS_Motor1_Pin|battery_Pin);

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void ADC_Select_Pinky_CH (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_11;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  __disable_irq();
	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  __enable_irq();
}
void ADC_ReadCurrent_Pinky()
{
	ADC_Select_Pinky_CH();
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	Current_motor[1] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	Fingers_Status.Pinky.Current=(Fingers_Status.Pinky.Current+(0.05*Current_motor[1]))/1.05;
	Current_motor[1]=Fingers_Status.Pinky.Current;
	if(Fingers_Status.Pinky.Current>3800 || Fingers_Status.Pinky.Current<800)
		Fingers_Status.Pinky.Stuck_Finger=1;
	else
		Fingers_Status.Pinky.Stuck_Finger=0;
}
void ADC_Select_Ring_CH (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_8;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  __disable_irq();
	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  __enable_irq();
}
void ADC_ReadCurrent_Ring()
{
	ADC_Select_Ring_CH();
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	Current_motor[2] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	Fingers_Status.Ring.Current=(Fingers_Status.Ring.Current+(0.05*Current_motor[2]))/1.05;
	Current_motor[2]=Fingers_Status.Ring.Current;
	if(Fingers_Status.Ring.Current>3800 || Fingers_Status.Ring.Current<800)
		Fingers_Status.Ring.Stuck_Finger=1;
	else
		Fingers_Status.Ring.Stuck_Finger=0;
}
void ADC_Select_Middle_CH (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_6;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  __disable_irq();
	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  __enable_irq();
}
void ADC_ReadCurrent_Middle()
{
	ADC_Select_Middle_CH();
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	Current_motor[3] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	Fingers_Status.Middle.Current=(Fingers_Status.Middle.Current+(0.05*Current_motor[3]))/1.05;
	Current_motor[3]=Fingers_Status.Middle.Current;
	if(Fingers_Status.Middle.Current>3800 || Fingers_Status.Middle.Current<800)
		Fingers_Status.Middle.Stuck_Finger=1;
	else
		Fingers_Status.Middle.Stuck_Finger=0;
}
void ADC_Select_Index_CH (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  __disable_irq();
	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  __enable_irq();
}
void ADC_ReadCurrent_Index()
{
	ADC_Select_Index_CH();
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	Current_motor[4] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	Fingers_Status.Index.Current=(Fingers_Status.Index.Current+(0.05*Current_motor[4]))/1.05;
	Current_motor[4]=Fingers_Status.Index.Current;
	if(Fingers_Status.Index.Current>3800 || Fingers_Status.Index.Current<800)
		Fingers_Status.Index.Stuck_Finger=1;
	else
		Fingers_Status.Index.Stuck_Finger=0;
}
void ADC_Select_Thumb_CH (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  __disable_irq();
	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  __enable_irq();
}
void ADC_ReadCurrent_Thumb()
{
	ADC_Select_Thumb_CH();
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	Current_motor[5] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	Fingers_Status.Thumb.Current=(Fingers_Status.Thumb.Current+(0.05*Current_motor[5]))/1.05;
	Current_motor[5]=Fingers_Status.Thumb.Current;
	if(Fingers_Status.Thumb.Current>3800 || Fingers_Status.Thumb.Current<800)
		Fingers_Status.Thumb.Stuck_Finger=1;
	else
		Fingers_Status.Thumb.Stuck_Finger=0;
}
void ADC_Select_CH5 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_5;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  __disable_irq();
	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  __enable_irq();
}
/* USER CODE END 1 */
