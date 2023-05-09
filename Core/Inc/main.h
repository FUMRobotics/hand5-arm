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
#include "stm32f1xx_hal.h"

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
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;


extern UART_HandleTypeDef huart1;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Motor5_INB_Pin GPIO_PIN_10
#define Motor5_INB_GPIO_Port GPIOB
#define Motor5_INA_Pin GPIO_PIN_11
#define Motor5_INA_GPIO_Port GPIOB
#define Motor4_INB_Pin GPIO_PIN_12
#define Motor4_INB_GPIO_Port GPIOB
#define Motor4_INA_Pin GPIO_PIN_13
#define Motor4_INA_GPIO_Port GPIOB
#define Motor3_INB_Pin GPIO_PIN_14
#define Motor3_INB_GPIO_Port GPIOB
#define Motor3_INA_Pin GPIO_PIN_15
#define Motor3_INA_GPIO_Port GPIOB
#define Motor2_INA_Pin GPIO_PIN_6
#define Motor2_INA_GPIO_Port GPIOC
#define Motor2_INB_Pin GPIO_PIN_7
#define Motor2_INB_GPIO_Port GPIOC
#define Motor1_INA_Pin GPIO_PIN_8
#define Motor1_INA_GPIO_Port GPIOC
#define Motor1_INB_Pin GPIO_PIN_9
#define Motor1_INB_GPIO_Port GPIOC
#define Motor5_ENC_Pin GPIO_PIN_10
#define Motor5_ENC_GPIO_Port GPIOC
#define Motor5_ENC_EXTI_IRQn EXTI15_10_IRQn
#define Motor4_ENC_Pin GPIO_PIN_11
#define Motor4_ENC_GPIO_Port GPIOC
#define Motor4_ENC_EXTI_IRQn EXTI15_10_IRQn
#define Motor3_ENC_Pin GPIO_PIN_12
#define Motor3_ENC_GPIO_Port GPIOC
#define Motor3_ENC_EXTI_IRQn EXTI15_10_IRQn
#define Motor2_ENC_Pin GPIO_PIN_4
#define Motor2_ENC_GPIO_Port GPIOB
#define Motor2_ENC_EXTI_IRQn EXTI4_IRQn
#define Motor1_ENC_Pin GPIO_PIN_5
#define Motor1_ENC_GPIO_Port GPIOB
#define Motor1_ENC_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
