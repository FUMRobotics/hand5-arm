/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "motor_Control.h"
#include "usart.h"
#include "stdio.h"
#include "ESP_UART.h"
#include "string.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ThumbFinger_T */
osThreadId_t ThumbFinger_THandle;
uint32_t ThumbFinger_TBuffer[ 128 ];
osStaticThreadDef_t ThumbFinger_TControlBlock;
const osThreadAttr_t ThumbFinger_T_attributes = {
  .name = "ThumbFinger_T",
  .cb_mem = &ThumbFinger_TControlBlock,
  .cb_size = sizeof(ThumbFinger_TControlBlock),
  .stack_mem = &ThumbFinger_TBuffer[0],
  .stack_size = sizeof(ThumbFinger_TBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IndexFinger_T */
osThreadId_t IndexFinger_THandle;
uint32_t IndexFinger_TBuffer[ 128 ];
osStaticThreadDef_t IndexFinger_TControlBlock;
const osThreadAttr_t IndexFinger_T_attributes = {
  .name = "IndexFinger_T",
  .cb_mem = &IndexFinger_TControlBlock,
  .cb_size = sizeof(IndexFinger_TControlBlock),
  .stack_mem = &IndexFinger_TBuffer[0],
  .stack_size = sizeof(IndexFinger_TBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MiddleFinger_T */
osThreadId_t MiddleFinger_THandle;
uint32_t MiddleFinger_TBuffer[ 128 ];
osStaticThreadDef_t MiddleFinger_TControlBlock;
const osThreadAttr_t MiddleFinger_T_attributes = {
  .name = "MiddleFinger_T",
  .cb_mem = &MiddleFinger_TControlBlock,
  .cb_size = sizeof(MiddleFinger_TControlBlock),
  .stack_mem = &MiddleFinger_TBuffer[0],
  .stack_size = sizeof(MiddleFinger_TBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RingFinger_T */
osThreadId_t RingFinger_THandle;
uint32_t RingFinger_TBuffer[ 128 ];
osStaticThreadDef_t RingFinger_TControlBlock;
const osThreadAttr_t RingFinger_T_attributes = {
  .name = "RingFinger_T",
  .cb_mem = &RingFinger_TControlBlock,
  .cb_size = sizeof(RingFinger_TControlBlock),
  .stack_mem = &RingFinger_TBuffer[0],
  .stack_size = sizeof(RingFinger_TBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PinkyFinger_T */
osThreadId_t PinkyFinger_THandle;
uint32_t PinkyFinger_TBuffer[ 128 ];
osStaticThreadDef_t PinkyFinger_TControlBlock;
const osThreadAttr_t PinkyFinger_T_attributes = {
  .name = "PinkyFinger_T",
  .cb_mem = &PinkyFinger_TControlBlock,
  .cb_size = sizeof(PinkyFinger_TControlBlock),
  .stack_mem = &PinkyFinger_TBuffer[0],
  .stack_size = sizeof(PinkyFinger_TBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Communication */
osThreadId_t CommunicationHandle;
uint32_t CommunicationBuffer[ 128 ];
osStaticThreadDef_t CommunicationControlBlock;
const osThreadAttr_t Communication_attributes = {
  .name = "Communication",
  .cb_mem = &CommunicationControlBlock,
  .cb_size = sizeof(CommunicationControlBlock),
  .stack_mem = &CommunicationBuffer[0],
  .stack_size = sizeof(CommunicationBuffer),
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ThumbFinger(void *argument);
void IndexFinger(void *argument);
void MiddleFinger(void *argument);
void RingFinger(void *argument);
void PinkyFinger(void *argument);
void CommunicationTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ThumbFinger_T */
  ThumbFinger_THandle = osThreadNew(ThumbFinger, NULL, &ThumbFinger_T_attributes);

  /* creation of IndexFinger_T */
  IndexFinger_THandle = osThreadNew(IndexFinger, NULL, &IndexFinger_T_attributes);

  /* creation of MiddleFinger_T */
  MiddleFinger_THandle = osThreadNew(MiddleFinger, NULL, &MiddleFinger_T_attributes);

  /* creation of RingFinger_T */
  RingFinger_THandle = osThreadNew(RingFinger, NULL, &RingFinger_T_attributes);

  /* creation of PinkyFinger_T */
  PinkyFinger_THandle = osThreadNew(PinkyFinger, NULL, &PinkyFinger_T_attributes);

  /* creation of Communication */
  CommunicationHandle = osThreadNew(CommunicationTask, NULL, &Communication_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ThumbFinger */
/**
 * @brief  Function implementing the ThumbFinger_T thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_ThumbFinger */
void ThumbFinger(void *argument)
{
  /* USER CODE BEGIN ThumbFinger */
	/* Infinite loop */
	for(;;)
	{
		////if(Fingers_Status.Thumb.Stuck_Finger)
//			Fingers_Status.Thumb.Direction=Stop;
		SetMotor(Thumb, &Fingers_Status.Thumb);
		ADC_ReadCurrent_Thumb();
		osDelay(1);
	}
  /* USER CODE END ThumbFinger */
}

/* USER CODE BEGIN Header_IndexFinger */
/**
 * @brief Function implementing the IndexFinger_T thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IndexFinger */
void IndexFinger(void *argument)
{
  /* USER CODE BEGIN IndexFinger */
	/* Infinite loop */
	for(;;)
	{
		////if(Fingers_Status.Index.Stuck_Finger)
//			Fingers_Status.Index.Direction=Stop;
		SetMotor(Index, &Fingers_Status.Index);
		ADC_ReadCurrent_Index();
		osDelay(1);
	}
  /* USER CODE END IndexFinger */
}

/* USER CODE BEGIN Header_MiddleFinger */
/**
 * @brief Function implementing the MiddleFinger_T thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MiddleFinger */
void MiddleFinger(void *argument)
{
  /* USER CODE BEGIN MiddleFinger */
	/* Infinite loop */
	for(;;)
	{
		////if(Fingers_Status.Middle.Stuck_Finger)
//			Fingers_Status.Middle.Direction=Stop;
		SetMotor(Middle, &Fingers_Status.Middle);
		ADC_ReadCurrent_Middle();
		osDelay(1);
	}
  /* USER CODE END MiddleFinger */
}

/* USER CODE BEGIN Header_RingFinger */
/**
 * @brief Function implementing the RingFinger_T thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RingFinger */
void RingFinger(void *argument)
{
  /* USER CODE BEGIN RingFinger */
	/* Infinite loop */
	for(;;)
	{
		////if(Fingers_Status.Ring.Stuck_Finger)
//			Fingers_Status.Ring.Direction=Stop;
		SetMotor(Ring, &Fingers_Status.Ring);
		ADC_ReadCurrent_Ring();
		osDelay(1);
	}
  /* USER CODE END RingFinger */
}

/* USER CODE BEGIN Header_PinkyFinger */
/**
 * @brief Function implementing the PinkyFinger_T thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PinkyFinger */
void PinkyFinger(void *argument)
{
  /* USER CODE BEGIN PinkyFinger */
	/* Infinite loop */
	for(;;)
	{
		////if(Fingers_Status.Pinky.Stuck_Finger)
//			Fingers_Status.Pinky.Direction=Stop;
		SetMotor(Pinky, &Fingers_Status.Pinky);
		ADC_ReadCurrent_Pinky();
		osDelay(1);
	}
  /* USER CODE END PinkyFinger */
}

/* USER CODE BEGIN Header_CommunicationTask */
/**
 * @brief Function implementing the Communication thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CommunicationTask */
void CommunicationTask(void *argument)
{
  /* USER CODE BEGIN CommunicationTask */
	char uartTX[50];
	/* Infinite loop */
	for(;;)
	{
		if(send_data_UART)
		{
			send_data_UART=0;
			sprintf(uartTX,"{CP:%dCR:%dCM:%dCI:%dCT:%d}\n",Fingers_Status.Pinky.Current,Fingers_Status.Ring.Current,Fingers_Status.Middle.Current,Fingers_Status.Index.Current,Fingers_Status.Thumb.Current);
			HAL_UART_Transmit(&huart4, (uint8_t*)uartTX, strlen(uartTX), 5);
			osDelay(1);
			sprintf(uartTX,"{PP:%dPR:%dPM:%dPI:%dPT:%d}\n",((uint16_t)(Fingers_Status.Pinky.position*100)),((uint16_t)(Fingers_Status.Ring.position*100)),((uint16_t)(Fingers_Status.Middle.position*100)),((uint16_t)(Fingers_Status.Index.position*100)),((uint16_t)(Fingers_Status.Thumb.position*100)));
			HAL_UART_Transmit(&huart4, (uint8_t*)uartTX, strlen(uartTX), 5);
		}
		osDelay(1);
	}
  /* USER CODE END CommunicationTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

