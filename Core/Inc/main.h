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
typedef struct {
	float FeedBack;    	// Feedback
	float SetPoint;    	// SetPoint
	float Error;		// Calculated Error
	float DTerm;		// Calculated DTerm
	float ITerm;		// Calculated ITerm
	float I_Max;		//Maximum value of I Term - User parameter
	float I_Min;		//Minimum value of I term - User parameter
	float Out;			//Final PID Output
	float Out_Max;		//Maximum value of Output - User parameter
	float Out_Min;		//Minimum value of Output - User parameter
	float KP;			//Coefficient of P Term - User parameter
	float KI;			//Coefficient of I Term - User parameter
	float KD;			//Coefficient of D Term - User parameter
	float LoopPeriod;    //time of PID Control Loop - Not used in here
} PID;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void ParseTCP(uint8_t *txt);
void SetMotor(uint8_t motor_num, uint8_t motor_dir, uint8_t motor_speed);
void PID_Position_Defaults(PID *pid);
void ApplyPIDToMotor(PID *pid, uint8_t DesiredMotor);
void ComputePID(PID *PIDSystem);
void ADC_Select_CH0(void);
void ADC_Select_CH1(void);
void ADC_Select_CH2(void);
void ADC_Select_CH3(void);
void ADC_Select_CH4(void);
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
