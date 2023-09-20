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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define Motor3_Encoder1_Pin GPIO_PIN_0
#define Motor3_Encoder1_GPIO_Port GPIOC
#define Motor3_Encoder1_EXTI_IRQn EXTI0_IRQn
#define Motor3_Encoder2_Pin GPIO_PIN_1
#define Motor3_Encoder2_GPIO_Port GPIOC
#define Motor3_Encoder2_EXTI_IRQn EXTI1_IRQn
#define Motor1_Current_Pin GPIO_PIN_0
#define Motor1_Current_GPIO_Port GPIOA
#define Motor2_Current_Pin GPIO_PIN_1
#define Motor2_Current_GPIO_Port GPIOA
#define Motor3_Current_Pin GPIO_PIN_2
#define Motor3_Current_GPIO_Port GPIOA
#define Motor4_Current_Pin GPIO_PIN_3
#define Motor4_Current_GPIO_Port GPIOA
#define Motor5_Current_Pin GPIO_PIN_4
#define Motor5_Current_GPIO_Port GPIOA
#define Motor2_Encoder1_Pin GPIO_PIN_6
#define Motor2_Encoder1_GPIO_Port GPIOA
#define Motor2_Encoder1_EXTI_IRQn EXTI9_5_IRQn
#define Motor2_Encoder2_Pin GPIO_PIN_7
#define Motor2_Encoder2_GPIO_Port GPIOA
#define Motor2_Encoder2_EXTI_IRQn EXTI9_5_IRQn
#define Motor1_Encoder1_Pin GPIO_PIN_4
#define Motor1_Encoder1_GPIO_Port GPIOC
#define Motor1_Encoder1_EXTI_IRQn EXTI4_IRQn
#define Motor1_Encoder2_Pin GPIO_PIN_5
#define Motor1_Encoder2_GPIO_Port GPIOC
#define Motor1_Encoder2_EXTI_IRQn EXTI9_5_IRQn
#define Motor3_INB_Pin GPIO_PIN_12
#define Motor3_INB_GPIO_Port GPIOB
#define Motor3_INA_Pin GPIO_PIN_13
#define Motor3_INA_GPIO_Port GPIOB
#define Motor4_INB_Pin GPIO_PIN_14
#define Motor4_INB_GPIO_Port GPIOB
#define Motor4_INA_Pin GPIO_PIN_15
#define Motor4_INA_GPIO_Port GPIOB
#define Motor2_INA_Pin GPIO_PIN_6
#define Motor2_INA_GPIO_Port GPIOC
#define Motor2_INB_Pin GPIO_PIN_7
#define Motor2_INB_GPIO_Port GPIOC
#define Motor1_INB_Pin GPIO_PIN_8
#define Motor1_INB_GPIO_Port GPIOC
#define Motor1_INA_Pin GPIO_PIN_9
#define Motor1_INA_GPIO_Port GPIOC
#define Motor1_PWM_Pin GPIO_PIN_8
#define Motor1_PWM_GPIO_Port GPIOA
#define Motor2_PWM_Pin GPIO_PIN_9
#define Motor2_PWM_GPIO_Port GPIOA
#define Motor3_PWM_Pin GPIO_PIN_10
#define Motor3_PWM_GPIO_Port GPIOA
#define Motor4_PWM_Pin GPIO_PIN_11
#define Motor4_PWM_GPIO_Port GPIOA
#define Motor5_Encoder1_Pin GPIO_PIN_12
#define Motor5_Encoder1_GPIO_Port GPIOA
#define Motor5_PWM_Pin GPIO_PIN_15
#define Motor5_PWM_GPIO_Port GPIOA
#define Motor5_Encoder2_Pin GPIO_PIN_10
#define Motor5_Encoder2_GPIO_Port GPIOC
#define Motor5_Encoder2_EXTI_IRQn EXTI15_10_IRQn
#define Motor4_Encoder1_Pin GPIO_PIN_11
#define Motor4_Encoder1_GPIO_Port GPIOC
#define Motor4_Encoder1_EXTI_IRQn EXTI15_10_IRQn
#define Motor4_Encoder2_Pin GPIO_PIN_12
#define Motor4_Encoder2_GPIO_Port GPIOC
#define Motor4_Encoder2_EXTI_IRQn EXTI15_10_IRQn
#define Motor5_INB_Pin GPIO_PIN_2
#define Motor5_INB_GPIO_Port GPIOD
#define Motor5_INA_Pin GPIO_PIN_3
#define Motor5_INA_GPIO_Port GPIOB
#define ESP_TX_Pin GPIO_PIN_6
#define ESP_TX_GPIO_Port GPIOB
#define ESP_RX_Pin GPIO_PIN_7
#define ESP_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
