/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_Control.h"
#include "ESP_UART.h"
#include <stdio.h>
#include <string.h>
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char uartTX[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	init_motor_controller();
	//start fingers calibration
	Fingers_Calibration();
	//feedback for end of calibration
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		Action_Motor_Video();
		//------------------------------| Thumb finger |----------------------------------------
		SetMotor(Thumb, &Fingers_Status.Thumb);
//		Control_Motor(Thumb, &Fingers_Status.Thumb);
		ADC_ReadCurrent_Thumb();
//		if(Fingers_Status.Thumb.Direction_motor==Stop)
//			Read_Encoder(Thumb, &Fingers_Status.Thumb);
		//------------------------------| Index finger |----------------------------------------
		ADC_ReadCurrent_Index();
		Control_Motor(Index,&Fingers_Status.Index);
		SetMotor(Index, &Fingers_Status.Index);
		if(Fingers_Status.Index.Direction_motor==Stop)
			Read_Encoder(Index, &Fingers_Status.Index);
		//------------------------------| Middle finger |----------------------------------------
		ADC_ReadCurrent_Middle();
		Control_Motor(Middle,&Fingers_Status.Middle);
		SetMotor(Middle, &Fingers_Status.Middle);
		if(Fingers_Status.Middle.Direction_motor==Stop)
			Read_Encoder(Middle, &Fingers_Status.Middle);
		//------------------------------| Ring finger |----------------------------------------
		SetMotor(Ring, &Fingers_Status.Ring);
//		Control_Motor(Ring,&Fingers_Status.Ring);
		ADC_ReadCurrent_Ring();
		if(Fingers_Status.Ring.Direction_motor==Stop)
			Read_Encoder(Ring, &Fingers_Status.Ring);
		//------------------------------| Pinky finger |----------------------------------------
		SetMotor(Pinky, &Fingers_Status.Pinky);
//		Control_Motor(Pinky,&Fingers_Status.Pinky);
		ADC_ReadCurrent_Pinky();
		if(Fingers_Status.Pinky.Direction_motor==Stop)
			Read_Encoder(Pinky, &Fingers_Status.Pinky);
		//------------------------------| Communication |----------------------------------------
		if(send_data_UART)
		{
			send_data_UART=0;
			uint16_t current_map[5];
			//map current pinky
			if(Fingers_Status.Pinky.Current<1772)
				current_map[Pinky]=((float)(1772-Fingers_Status.Pinky.Current)/(1772-1288))*1000;
			else
				current_map[Pinky]=((float)(Fingers_Status.Pinky.Current-1772)/(2298-1772))*1000;
			//map current ring
			if(Fingers_Status.Ring.Current<1781)
				current_map[Ring]=((float)(1781-Fingers_Status.Ring.Current)/(1781-1264))*1000;
			else
				current_map[Ring]=((float)(Fingers_Status.Ring.Current-1781)/(2243-1781))*1000;
			//map current middle
			if(Fingers_Status.Middle.Current<1750)
				current_map[Middle]=((float)(1750-Fingers_Status.Middle.Current)/(1750-1290))*1000;
			else
				current_map[Middle]=((float)(Fingers_Status.Middle.Current-1750)/(2291-1750))*1000;
			//map current index
			if(Fingers_Status.Index.Current<1688)
				current_map[Index]=((float)(1688-Fingers_Status.Index.Current)/(1688-1226))*1000;
			else
				current_map[Index]=((float)(Fingers_Status.Index.Current-1688)/(2239-1688))*1000;
			//map current thumb
			if(Fingers_Status.Thumb.Current<1701)
				current_map[Thumb]=((float)(1701-Fingers_Status.Thumb.Current)/(1701-1233))*1000;
			else
				current_map[Thumb]=((float)(Fingers_Status.Thumb.Current-1701)/(2279-1701))*1000;
			sprintf(uartTX,"{CP:%dCR:%dCM:%dCI:%dCT:%d}\n",current_map[Pinky],current_map[Ring],current_map[Middle],current_map[Index],current_map[Thumb]);
//			sprintf(uartTX,"{CP:%dCR:%dCM:%dCI:%dCT:%d}\n",Fingers_Status.Pinky.Current,Fingers_Status.Ring.Current,Fingers_Status.Middle.Current,Fingers_Status.Index.Current,Fingers_Status.Thumb.Current);
			HAL_UART_Transmit(&huart4, (uint8_t*)uartTX, strlen(uartTX), 5);
			HAL_Delay(1);
			sprintf(uartTX,"{PP:%dPR:%dPM:%dPI:%dPT:%d}\n",((uint16_t)(Fingers_Status.Pinky.position*100)),((uint16_t)(Fingers_Status.Ring.position*100)),((uint16_t)(Fingers_Status.Middle.position*100)),((uint16_t)(Fingers_Status.Index.position*100)),((uint16_t)(Fingers_Status.Thumb.position*100)));
			HAL_UART_Transmit(&huart4, (uint8_t*)uartTX, strlen(uartTX), 5);
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance==TIM7)
	{
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
