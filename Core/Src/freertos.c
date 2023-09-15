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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ThumbFinger_Tas */
osThreadId_t ThumbFinger_TasHandle;
const osThreadAttr_t ThumbFinger_Tas_attributes = {
  .name = "ThumbFinger_Tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IndexFinger_Tas */
osThreadId_t IndexFinger_TasHandle;
const osThreadAttr_t IndexFinger_Tas_attributes = {
  .name = "IndexFinger_Tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MiddleFinger_Ta */
osThreadId_t MiddleFinger_TaHandle;
const osThreadAttr_t MiddleFinger_Ta_attributes = {
  .name = "MiddleFinger_Ta",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Ringfinger_Task */
osThreadId_t Ringfinger_TaskHandle;
const osThreadAttr_t Ringfinger_Task_attributes = {
  .name = "Ringfinger_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PinkyFinger_tas */
osThreadId_t PinkyFinger_tasHandle;
const osThreadAttr_t PinkyFinger_tas_attributes = {
  .name = "PinkyFinger_tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ThumbFinger(void *argument);
void IndexFinger(void *argument);
void MiddleFinger(void *argument);
void Ringfinger(void *argument);
void PinkyFinger(void *argument);

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
  /* creation of ThumbFinger_Tas */
  ThumbFinger_TasHandle = osThreadNew(ThumbFinger, NULL, &ThumbFinger_Tas_attributes);

  /* creation of IndexFinger_Tas */
  IndexFinger_TasHandle = osThreadNew(IndexFinger, NULL, &IndexFinger_Tas_attributes);

  /* creation of MiddleFinger_Ta */
  MiddleFinger_TaHandle = osThreadNew(MiddleFinger, NULL, &MiddleFinger_Ta_attributes);

  /* creation of Ringfinger_Task */
  Ringfinger_TaskHandle = osThreadNew(Ringfinger, NULL, &Ringfinger_Task_attributes);

  /* creation of PinkyFinger_tas */
  PinkyFinger_tasHandle = osThreadNew(PinkyFinger, NULL, &PinkyFinger_tas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ThumbFinger */
/**
 * @brief  Function implementing the ThumbFinger_Tas thread.
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
		SetMotor(Thumb, Fingers_Status.Thumb);
		Read_Encoder(&Fingers_Status.Thumb, Thumb);
		ADC_ReadCurrent_Thumb();
		osDelay(1);
	}
  /* USER CODE END ThumbFinger */
}

/* USER CODE BEGIN Header_IndexFinger */
/**
 * @brief Function implementing the IndexFinger_Tas thread.
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

		SetMotor(Index, Fingers_Status.Index);
		Read_Encoder(&Fingers_Status.Index, Index);
		ADC_ReadCurrent_Index();
		osDelay(1);
	}
  /* USER CODE END IndexFinger */
}

/* USER CODE BEGIN Header_MiddleFinger */
/**
 * @brief Function implementing the MiddleFinger_Ta thread.
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
		SetMotor(Middle, Fingers_Status.Middle);
		Read_Encoder(&Fingers_Status.Middle, Middle);
		ADC_ReadCurrent_Middle();
		osDelay(1);
	}
  /* USER CODE END MiddleFinger */
}

/* USER CODE BEGIN Header_Ringfinger */
/**
 * @brief Function implementing the Ringfinger_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Ringfinger */
void Ringfinger(void *argument)
{
  /* USER CODE BEGIN Ringfinger */
	/* Infinite loop */
	for(;;)
	{
		SetMotor(Ring, Fingers_Status.Ring);
		Read_Encoder(&Fingers_Status.Ring, Ring);
		ADC_ReadCurrent_Ring();
		osDelay(1);
	}
  /* USER CODE END Ringfinger */
}

/* USER CODE BEGIN Header_PinkyFinger */
/**
 * @brief Function implementing the PinkyFinger_tas thread.
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
		SetMotor(Pinky, Fingers_Status.Pinky);
		Read_Encoder(&Fingers_Status.Ring, Ring);
		ADC_ReadCurrent_Pinky();
		osDelay(1);
	}
  /* USER CODE END PinkyFinger */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

