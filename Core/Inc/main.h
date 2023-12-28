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
#include "stm32l4xx_hal.h"

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
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define ACS_Motor5_Pin GPIO_PIN_0
#define ACS_Motor5_GPIO_Port GPIOC
#define ACS_Motor4_Pin GPIO_PIN_1
#define ACS_Motor4_GPIO_Port GPIOC
#define Iprop_Motor4_Pin GPIO_PIN_2
#define Iprop_Motor4_GPIO_Port GPIOC
#define Iprop_Motor5_Pin GPIO_PIN_3
#define Iprop_Motor5_GPIO_Port GPIOC
#define ACS_Motor3_Pin GPIO_PIN_1
#define ACS_Motor3_GPIO_Port GPIOA
#define Iprop_Motor3_Pin GPIO_PIN_2
#define Iprop_Motor3_GPIO_Port GPIOA
#define ACS_Motor2_Pin GPIO_PIN_3
#define ACS_Motor2_GPIO_Port GPIOA
#define Iprop_Motor2_Pin GPIO_PIN_4
#define Iprop_Motor2_GPIO_Port GPIOA
#define Iprop_Motor1_Pin GPIO_PIN_5
#define Iprop_Motor1_GPIO_Port GPIOA
#define ACS_Motor1_Pin GPIO_PIN_6
#define ACS_Motor1_GPIO_Port GPIOA
#define battery_Pin GPIO_PIN_7
#define battery_GPIO_Port GPIOA
#define Motor3_Encoder2_Pin GPIO_PIN_4
#define Motor3_Encoder2_GPIO_Port GPIOC
#define Motor3_Encoder2_EXTI_IRQn EXTI4_IRQn
#define Motor3_Encoder1_Pin GPIO_PIN_5
#define Motor3_Encoder1_GPIO_Port GPIOC
#define Motor3_Encoder1_EXTI_IRQn EXTI9_5_IRQn
#define Motor4_Encoder1_Pin GPIO_PIN_0
#define Motor4_Encoder1_GPIO_Port GPIOB
#define Motor4_Encoder1_EXTI_IRQn EXTI0_IRQn
#define Motor4_Encoder2_Pin GPIO_PIN_1
#define Motor4_Encoder2_GPIO_Port GPIOB
#define Motor4_Encoder2_EXTI_IRQn EXTI1_IRQn
#define Motor5_Encoder1_Pin GPIO_PIN_2
#define Motor5_Encoder1_GPIO_Port GPIOB
#define Motor5_Encoder1_EXTI_IRQn EXTI2_IRQn
#define IN1_Motor3_Pin GPIO_PIN_10
#define IN1_Motor3_GPIO_Port GPIOB
#define IN2_Motor3_Pin GPIO_PIN_11
#define IN2_Motor3_GPIO_Port GPIOB
#define Motor1_Encoder2_Pin GPIO_PIN_12
#define Motor1_Encoder2_GPIO_Port GPIOB
#define Motor1_Encoder2_EXTI_IRQn EXTI15_10_IRQn
#define Motor1_Encoder1_Pin GPIO_PIN_13
#define Motor1_Encoder1_GPIO_Port GPIOB
#define Motor1_Encoder1_EXTI_IRQn EXTI15_10_IRQn
#define Motor2_Encoder1_Pin GPIO_PIN_14
#define Motor2_Encoder1_GPIO_Port GPIOB
#define Motor2_Encoder1_EXTI_IRQn EXTI15_10_IRQn
#define Motor2_Encoder2_Pin GPIO_PIN_15
#define Motor2_Encoder2_GPIO_Port GPIOB
#define Motor2_Encoder2_EXTI_IRQn EXTI15_10_IRQn
#define IN2_Motor2_Pin GPIO_PIN_6
#define IN2_Motor2_GPIO_Port GPIOC
#define IN1_Motor2_Pin GPIO_PIN_7
#define IN1_Motor2_GPIO_Port GPIOC
#define IN2_Motor1_Pin GPIO_PIN_8
#define IN2_Motor1_GPIO_Port GPIOA
#define IN1_Motor1_Pin GPIO_PIN_9
#define IN1_Motor1_GPIO_Port GPIOA
#define Motor5_Encoder2_Pin GPIO_PIN_10
#define Motor5_Encoder2_GPIO_Port GPIOA
#define Motor5_Encoder2_EXTI_IRQn EXTI15_10_IRQn
#define IN2_Motor5_Pin GPIO_PIN_4
#define IN2_Motor5_GPIO_Port GPIOB
#define IN1_Motor5_Pin GPIO_PIN_5
#define IN1_Motor5_GPIO_Port GPIOB
#define IN2_Motor4_Pin GPIO_PIN_6
#define IN2_Motor4_GPIO_Port GPIOB
#define IN1_Motor4_Pin GPIO_PIN_7
#define IN1_Motor4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
