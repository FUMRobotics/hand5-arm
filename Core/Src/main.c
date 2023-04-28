/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include "jsmn.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t dummy = 0;
//uart buffer
const char uartRecieveBuffer[300];
uint16_t uartCounter = 0;

uint16_t DATA[5] = { 0 };

#define MOTOR_CW 	 0
#define MOTOR_CCW 	 1
#define MOTOR_STOP 	 2
#define FINGER_Close MOTOR_CW
#define FINGER_Open  MOTOR_CCW
#define FINGER_Stop  MOTOR_STOP

#define Thumb_Motor  	5
#define Index_Motor  	4
#define Middle_Motor  	3
#define Ring_Motor  	2
#define Pinky_Motor  	1

uint8_t Thumb_ManualControl = FINGER_Stop;
uint8_t Index_ManualControl = FINGER_Stop;
uint8_t Middle_ManualControl = FINGER_Stop;
uint8_t Ring_ManualControl = FINGER_Stop;
uint8_t Pinky_ManualControl = FINGER_Stop;
uint8_t GestureManualControl = 0;
uint8_t ManualControlActive = 0;
uint8_t ManualControlPWM = 99;

int16_t Thumb_Position = 0;
int16_t Index_Position = 0;
int16_t Middle_Position = 0;
int16_t Ring_Position = 0;
int16_t Pinky_Position = 0;

int16_t Thumb_Position_SetPoint = 0;
int16_t Index_Position_SetPoint = 0;
int16_t Middle_Position_SetPoint = 0;
int16_t Ring_Position_SetPoint = 0;
int16_t Pinky_Position_SetPoint = 0;

uint8_t Thumb_LastDirection = 0;
uint8_t Index_LastDirection = 0;
uint8_t Middle_LastDirection = 0;
uint8_t Ring_LastDirection = 0;
uint8_t Pinky_LastDirection = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
// - Position PIDs
PID Thumb_Position_PID;
PID Index_Position_PID;
PID Middle_Position_PID;
PID Ring_Position_PID;
PID Pinky_Position_PID;
// = Current PIDs
PID Thumb_Current_PID;
PID Index_Current_PID;
PID Middle_Current_PID;
PID Ring_Current_PID;
PID Pinky_Current_PID;

float CRNT, SPD;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ComputePID(PID *PIDSystem) {
	/*
	 * >>> Using Procedure
	 * 0- Set output Max and Minimum
	 * 1- Set setpoint to desired one
	 * 2- Set Feedback
	 * 3- Call ComputePID
	 * <<< Using Procedure End
	 */
	PIDSystem->I_Max = PIDSystem->Out_Max / PIDSystem->KI;
	PIDSystem->I_Min = PIDSystem->Out_Min / PIDSystem->KI;

	PIDSystem->DTerm = PIDSystem->Error;    // Storing last Error
	PIDSystem->Error = PIDSystem->SetPoint - PIDSystem->FeedBack;
	// ----- Integral Term
	PIDSystem->ITerm = PIDSystem->ITerm + PIDSystem->Error;
	if (PIDSystem->ITerm > PIDSystem->I_Max)
		PIDSystem->ITerm = PIDSystem->I_Max;
	if (PIDSystem->ITerm < PIDSystem->I_Min)
		PIDSystem->ITerm = PIDSystem->I_Min;
	// ----- Derivative Term
	PIDSystem->DTerm = PIDSystem->Error - PIDSystem->DTerm;    // deltaError
	// ----- Output
	PIDSystem->Out = (PIDSystem->KP * PIDSystem->Error)
									+ (PIDSystem->KI * PIDSystem->ITerm)
									+ (PIDSystem->KD * PIDSystem->DTerm);
	if (PIDSystem->Out > PIDSystem->Out_Max)
		PIDSystem->Out = PIDSystem->Out_Max;
	if (PIDSystem->Out < PIDSystem->Out_Min)
		PIDSystem->Out = PIDSystem->Out_Min;
}
void PID_Position_Defaults(PID *pid) {
	pid->Out_Max = 99;
	pid->Out_Min = -99;
	pid->KP = 0;
	pid->KI = 0;
	pid->KD = 0;
	pid->I_Max = 50;
	pid->I_Min = -50;
	pid->LoopPeriod = 0.01;
}
void ApplyPIDToMotor(PID *pid, uint8_t DesiredMotor) {
	if (pid->Out == 0) {
		SetMotor(DesiredMotor, FINGER_Stop, pid->Out);
	} else if (pid->Out > 0) {
		SetMotor(DesiredMotor, FINGER_Open, pid->Out);
	} else {
		SetMotor(DesiredMotor, FINGER_Close, -pid->Out);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Motor1_ENC_Pin) {
		if (Pinky_LastDirection == FINGER_Open)
			Pinky_Position++;
		else
			Pinky_Position--;
	} else if (GPIO_Pin == Motor2_ENC_Pin) {
		if (Ring_LastDirection == FINGER_Open)
			Ring_Position++;
		else
			Ring_Position--;
	} else if (GPIO_Pin == Motor3_ENC_Pin) {
		if (Middle_LastDirection == FINGER_Open)
			Middle_Position++;
		else
			Middle_Position--;
	} else if (GPIO_Pin == Motor4_ENC_Pin) {
		if (Index_LastDirection == FINGER_Open)
			Index_Position++;
		else
			Index_Position--;
	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		// Thumb Position
		Thumb_Position_PID.SetPoint = Thumb_Position_SetPoint;
		Thumb_Position_PID.FeedBack = Thumb_Position;
		ComputePID(&Thumb_Position_PID);
		ApplyPIDToMotor(&Thumb_Position_PID, Thumb_Motor);
		// Index  Position
		Index_Position_PID.SetPoint = Index_Position_SetPoint;
		Index_Position_PID.FeedBack = Index_Position;
		ComputePID(&Index_Position_PID);
		ApplyPIDToMotor(&Index_Position_PID, Index_Motor);
		// Middle Position
		Middle_Position_PID.SetPoint = Middle_Position_SetPoint;
		Middle_Position_PID.FeedBack = Middle_Position;
		ComputePID(&Middle_Position_PID);
		ApplyPIDToMotor(&Middle_Position_PID, Middle_Motor);
		// Ring Position
		Ring_Position_PID.SetPoint = Ring_Position_SetPoint;
		Ring_Position_PID.FeedBack = Ring_Position;
		ComputePID(&Ring_Position_PID);
		ApplyPIDToMotor(&Ring_Position_PID, Ring_Motor);
		// Pinky Position
		Pinky_Position_PID.SetPoint = Pinky_Position_SetPoint;
		Pinky_Position_PID.FeedBack = Pinky_Position;
		ComputePID(&Pinky_Position_PID);
		ApplyPIDToMotor(&Pinky_Position_PID, Pinky_Motor);
	}
}
void SetMotor(uint8_t motor_num, uint8_t motor_dir, uint8_t motor_speed) {
	switch (motor_num) {
	case Pinky_Motor:
		Pinky_LastDirection = motor_dir;
		if (motor_dir == MOTOR_CW) {
			htim2.Instance->CCR1 = motor_speed;
			HAL_GPIO_WritePin(Motor1_INA_GPIO_Port, Motor1_INA_Pin, 0);
			HAL_GPIO_WritePin(Motor1_INB_GPIO_Port, Motor1_INB_Pin, 1);
		} else if (motor_dir == MOTOR_CCW) {
			htim2.Instance->CCR1 = motor_speed;
			HAL_GPIO_WritePin(Motor1_INA_GPIO_Port, Motor1_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor1_INB_GPIO_Port, Motor1_INB_Pin, 0);
		} else {
			htim2.Instance->CCR1 = 0;
			HAL_GPIO_WritePin(Motor1_INA_GPIO_Port, Motor1_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor1_INB_GPIO_Port, Motor1_INB_Pin, 1);
		}
		break;
	case Ring_Motor:
		Ring_LastDirection = motor_dir;
		if (motor_dir == MOTOR_CW) {
			htim1.Instance->CCR3 = motor_speed;
			HAL_GPIO_WritePin(Motor2_INA_GPIO_Port, Motor2_INA_Pin, 0);
			HAL_GPIO_WritePin(Motor2_INB_GPIO_Port, Motor2_INB_Pin, 1);
		} else if (motor_dir == MOTOR_CCW) {
			htim1.Instance->CCR3 = motor_speed;
			HAL_GPIO_WritePin(Motor2_INA_GPIO_Port, Motor2_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor2_INB_GPIO_Port, Motor2_INB_Pin, 0);
		} else {
			htim1.Instance->CCR3 = 0;
			HAL_GPIO_WritePin(Motor2_INA_GPIO_Port, Motor2_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor2_INB_GPIO_Port, Motor2_INB_Pin, 1);
		}
		break;
	case Middle_Motor:
		Middle_LastDirection = motor_dir;
		if (motor_dir == MOTOR_CW) {
			htim1.Instance->CCR4 = motor_speed;
			HAL_GPIO_WritePin(Motor3_INA_GPIO_Port, Motor3_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor3_INB_GPIO_Port, Motor3_INB_Pin, 0);
		} else if (motor_dir == MOTOR_CCW) {
			htim1.Instance->CCR4 = motor_speed;
			HAL_GPIO_WritePin(Motor3_INA_GPIO_Port, Motor3_INA_Pin, 0);
			HAL_GPIO_WritePin(Motor3_INB_GPIO_Port, Motor3_INB_Pin, 1);
		} else {
			htim1.Instance->CCR4 = 0;
			HAL_GPIO_WritePin(Motor3_INA_GPIO_Port, Motor3_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor3_INB_GPIO_Port, Motor3_INB_Pin, 1);
		}

		break;
	case Index_Motor:
		Index_LastDirection = motor_dir;
		if (motor_dir == MOTOR_CW) {
			htim1.Instance->CCR2 = motor_speed;
			HAL_GPIO_WritePin(Motor4_INA_GPIO_Port, Motor4_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor4_INA_GPIO_Port, Motor4_INB_Pin, 0);
		} else if (motor_dir == MOTOR_CCW) {
			htim1.Instance->CCR2 = motor_speed;
			HAL_GPIO_WritePin(Motor4_INA_GPIO_Port, Motor4_INA_Pin, 0);
			HAL_GPIO_WritePin(Motor4_INA_GPIO_Port, Motor4_INB_Pin, 1);
		} else {
			htim1.Instance->CCR2 = 0;
			HAL_GPIO_WritePin(Motor4_INA_GPIO_Port, Motor4_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor4_INA_GPIO_Port, Motor4_INB_Pin, 1);
		}
		break;
	case Thumb_Motor:
		Thumb_LastDirection = motor_dir;
		if (motor_dir == MOTOR_CW) {
			htim1.Instance->CCR1 = motor_speed;
			HAL_GPIO_WritePin(Motor5_INA_GPIO_Port, Motor5_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor5_INA_GPIO_Port, Motor5_INB_Pin, 0);
		} else if (motor_dir == MOTOR_CCW) {
			htim1.Instance->CCR1 = motor_speed;
			HAL_GPIO_WritePin(Motor5_INA_GPIO_Port, Motor5_INA_Pin, 0);
			HAL_GPIO_WritePin(Motor5_INA_GPIO_Port, Motor5_INB_Pin, 1);
		} else {
			htim1.Instance->CCR1 = 0;
			HAL_GPIO_WritePin(Motor5_INA_GPIO_Port, Motor5_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor5_INA_GPIO_Port, Motor5_INB_Pin, 1);
		}
		break;

	}
}


void ADC_Select_CH0(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC_Select_CH1(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC_Select_CH2(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC_Select_CH3(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC_Select_CH4(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
	if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0)
	{
		return 0;
	}
	return -1;
}

_Bool ProcessUartData(void)
{
	HandState_Typedef HandStruct[5]={0,};
	HandFinger_Typedef HandFinger;
	int i;
	int r;
	jsmn_parser p;
	jsmntok_t t[128]; /* We expect no more than 128 tokens */

	jsmn_init(&p);
	r = jsmn_parse(&p, uartRecieveBuffer, strlen(uartRecieveBuffer), t,
			sizeof(t) / sizeof(t[0]));
	if (r < 0)
	{
		//Failed to parse JSON
		return 1;
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT)
	{
		//Object expected
		return 1;
	}

	/* Loop over all keys of the root object */
	for (i = 1; i < r; i++)
	{

		if (jsoneq(uartRecieveBuffer, &t[i], "Thumb") == 0)
		{
			HandFinger = Thumb;
			i++;
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Middele") == 0)
		{
			HandFinger = Middel;
			i++;
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Index") == 0)
		{
			HandFinger = Index;
			i++;
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Ring") == 0)
		{
			HandFinger = Ring;
			i++;
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Pinky") == 0)
		{
			HandFinger = Pinky;
			i++;
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Status") == 0)
		{
			char FingerStateBuffer[100] = {0,};
			/* We may additionally check if the value is either "true" or "false" */
			strncpy(FingerStateBuffer, uartRecieveBuffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);
			i++;
			if (!strncmp("STOP", FingerStateBuffer, 4))
			{
				HandStruct[HandFinger - 1].HandStatuse = STOP;
			}
			if (!strncmp(FingerStateBuffer, "OPEN", 4))
			{
				HandStruct[HandFinger - 1].HandStatuse = OPEN;
			}
			if (!strncmp(FingerStateBuffer, "CLOSE", 5))
			{
				HandStruct[HandFinger - 1].HandStatuse = CLOSE;
			}
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Value") == 0)
		{
			char FingerValue[100] = {0,};
			strncpy(FingerValue, uartRecieveBuffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);
			i++;
			for (int counter = 0; counter < strlen(FingerValue); counter++)
			{
				switch (FingerValue[counter]) {
				case '0':
					HandStruct[HandFinger - 1].Value += 0 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '1':
					HandStruct[HandFinger - 1].Value += 1 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '2':
					HandStruct[HandFinger - 1].Value += 2 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '3':
					HandStruct[HandFinger - 1].Value += 3 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '4':
					HandStruct[HandFinger - 1].Value += 4 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '5':
					HandStruct[HandFinger - 1].Value += 5 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '6':
					HandStruct[HandFinger - 1].Value += 6 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '7':
					HandStruct[HandFinger - 1].Value += 7 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '8':
					HandStruct[HandFinger - 1].Value += 8 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '9':
					HandStruct[HandFinger - 1].Value += 9 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				default:
					break;
				}
			}
		}
		else
		{
			//Unexpected key
		}
	}
	//just for test code
	/*
	for(int counter=0;counter<5;counter++)
	{
		printf("--------finger number : %d---------\n",counter);
		switch (HandStruct[counter].HandStatuse)
		{
		case CLOSE:
			printf("finger status : CLOSE\n");
			break;
		case STOP:
			printf("finger status : STOP\n");
			break;
		case OPEN:
			printf("finger status : OPEN\n");
			break;
		default:
			break;
		}
		if(HandStruct[counter].Value/100)
		{
			HandStruct[counter].Value++;
		}
		printf("finger value : %d\n",HandStruct[counter].Value);
	}
	*/
	return EXIT_SUCCESS;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (uartRecieveBuffer[uartCounter] == '}'&& uartRecieveBuffer[uartCounter-1] == '}') {
			ProcessUartData();
		} else {
			uartCounter++;
			HAL_UART_Receive_IT(&huart1, (uint8_t*)&uartRecieveBuffer[uartCounter], 1);
		}
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	//start read data from ADC
	HAL_ADCEx_Calibration_Start(&hadc1);

	//start interrupt of UART for receive data from ESP32
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&uartRecieveBuffer[0], 1);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_ADC_MspInit(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) DATA, 5);
	// --------------- Current PID to Default
	PID_Position_Defaults(&Thumb_Current_PID);
	PID_Position_Defaults(&Index_Current_PID);
	PID_Position_Defaults(&Middle_Current_PID);
	PID_Position_Defaults(&Ring_Current_PID);
	PID_Position_Defaults(&Pinky_Current_PID);
	// --------------  Position PID To Default
	PID_Position_Defaults(&Thumb_Position_PID);
	PID_Position_Defaults(&Index_Position_PID);
	PID_Position_Defaults(&Middle_Position_PID);
	PID_Position_Defaults(&Ring_Position_PID);
	PID_Position_Defaults(&Pinky_Position_PID);
	HAL_UART_Receive_IT(&huart1, &dummy, 1);
	//	  	SetMotor(Index_Motor, FINGER_Open, 99);
	//	  	SetMotor(Middle_Motor, FINGER_Open, 99);
	//	  	SetMotor(Ring_Motor, FINGER_Open, 99);
	//	  	SetMotor(Pinky_Motor, FINGER_Open, 99);

	CRNT = 2500;
	SPD = 40;

	//	while (1) {
	//		if (DATA[0] > CRNT && DATA[1] > CRNT && DATA[2] > CRNT && DATA[3] > CRNT) {
	//			HAL_Delay(500);
	//			if (DATA[0] > CRNT && DATA[1] > CRNT && DATA[2] > CRNT && DATA[3] > CRNT) {
	//				break;
	//			}
	//
	//		}
	//	}

	//
	//	SetMotor(Index_Motor, FINGER_Stop, 99);
	//	SetMotor(Middle_Motor, FINGER_Stop, 99);
	//	SetMotor(Ring_Motor, FINGER_Stop, 99);
	//	SetMotor(Pinky_Motor, FINGER_Stop, 99);

	//		SetMotor(Index_Motor, FINGER_Open, SPD);
	//		HAL_Delay(5000);
	//		HAL_Delay(1);
	//		while(DATA[3]>0)
	//		{
	//			SetMotor(Index_Motor, FINGER_Close, SPD);
	//		}
	//		SetMotor(Index_Motor, FINGER_Stop, SPD);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//		ADC_Select_CH0();
		//		HAL_ADC_Start(&hadc1);
		//		HAL_ADC_PollForConversion(&hadc1, 100);
		//		DATA[0]=HAL_ADC_GetValue(&hadc1);
		//		HAL_ADC_Stop(&hadc1);
		//
		//		ADC_Select_CH1();
		//		HAL_ADC_Start(&hadc1);
		//		HAL_ADC_PollForConversion(&hadc1, 100);
		//		DATA[1]=HAL_ADC_GetValue(&hadc1);
		//		HAL_ADC_Stop(&hadc1);
		//
		//		ADC_Select_CH2();
		//		HAL_ADC_Start(&hadc1);
		//		HAL_ADC_PollForConversion(&hadc1, 100);
		//		DATA[2]=HAL_ADC_GetValue(&hadc1);
		//		HAL_ADC_Stop(&hadc1);
		//
		//		ADC_Select_CH3();
		//		HAL_ADC_Start(&hadc1);
		//		HAL_ADC_PollForConversion(&hadc1, 100);
		//		DATA[3]=HAL_ADC_GetValue(&hadc1);
		//		HAL_ADC_Stop(&hadc1);
		//
		//		ADC_Select_CH4();
		//		HAL_ADC_Start(&hadc1);
		//		HAL_ADC_PollForConversion(&hadc1, 100);
		//		DATA[4]=HAL_ADC_GetValue(&hadc1);
		//		HAL_ADC_Stop(&hadc1);

		if (ManualControlActive) {
			// ---------------------------  DANGER ---------------------------
			// After each Command You should manually set motor stop !!!! otherwise motor will burn !!!!!!!!!
			//			if (DATA[0] > CRNT && DATA[1] > CRNT && DATA[2] > CRNT && DATA[3] > CRNT)
			//			{
			//				HAL_Delay(10);
			//				if (DATA[0] > CRNT && DATA[1] > CRNT && DATA[2] > CRNT && DATA[3] > CRNT)
			//					GestureManualControl=0;
			//			}
			if (GestureManualControl == 0) {    // Stop All
				SetMotor(Index_Motor, FINGER_Stop, SPD);
				SetMotor(Middle_Motor, FINGER_Stop, SPD);
				SetMotor(Ring_Motor, FINGER_Stop, SPD);
				SetMotor(Pinky_Motor, FINGER_Stop, SPD);
				SetMotor(Thumb_Motor, FINGER_Stop, SPD);
				__NOP();
			} else if (GestureManualControl == 1) {    // Close all
				SetMotor(Index_Motor, FINGER_Close, SPD);
				SetMotor(Middle_Motor, FINGER_Close, SPD);
				SetMotor(Ring_Motor, FINGER_Close, SPD);
				SetMotor(Pinky_Motor, FINGER_Close, SPD);
				SetMotor(Thumb_Motor, FINGER_Close, SPD);
			} else if (GestureManualControl == 2) {    // Open all
				SetMotor(Index_Motor, FINGER_Open, SPD);
				SetMotor(Middle_Motor, FINGER_Open, SPD);
				SetMotor(Ring_Motor, FINGER_Open, SPD);
				SetMotor(Pinky_Motor, FINGER_Open, SPD);
				SetMotor(Thumb_Motor, FINGER_Open, SPD);
			} else if (GestureManualControl == 3) { // Close Index Finger, Stop other
				SetMotor(Index_Motor, FINGER_Close, SPD);
				SetMotor(Middle_Motor, FINGER_Stop, SPD);
				SetMotor(Ring_Motor, FINGER_Stop, SPD);
				SetMotor(Pinky_Motor, FINGER_Stop, SPD);
				SetMotor(Thumb_Motor, FINGER_Stop, SPD);
			} else if (GestureManualControl == 4) { // Open Index Finder, Stop Other
				SetMotor(Index_Motor, FINGER_Open, SPD);
				SetMotor(Middle_Motor, FINGER_Stop, SPD);
				SetMotor(Ring_Motor, FINGER_Stop, SPD);
				SetMotor(Pinky_Motor, FINGER_Stop, SPD);
				SetMotor(Thumb_Motor, FINGER_Stop, SPD);
			} else if (GestureManualControl == 5) {    // Independent Control
				SetMotor(Index_Motor, Index_ManualControl, ManualControlPWM);
				SetMotor(Middle_Motor, Middle_ManualControl, ManualControlPWM);
				SetMotor(Ring_Motor, Ring_ManualControl, ManualControlPWM);
				SetMotor(Pinky_Motor, Pinky_ManualControl, ManualControlPWM);
				SetMotor(Thumb_Motor, Thumb_ManualControl, ManualControlPWM);
			}

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 5;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 19;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 99;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 19;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 99;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 6399;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 99;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			LED_Pin | Motor2_INA_Pin | Motor2_INB_Pin | Motor1_INA_Pin | Motor1_INB_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			Motor5_INB_Pin | Motor5_INA_Pin | Motor4_INB_Pin | Motor4_INA_Pin
			| Motor3_INB_Pin | Motor3_INA_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_Pin Motor2_INA_Pin Motor2_INB_Pin Motor1_INA_Pin
	 Motor1_INB_Pin */
	GPIO_InitStruct.Pin = LED_Pin | Motor2_INA_Pin | Motor2_INB_Pin
			| Motor1_INA_Pin | Motor1_INB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Motor5_INB_Pin Motor5_INA_Pin Motor4_INB_Pin Motor4_INA_Pin
	 Motor3_INB_Pin Motor3_INA_Pin */
	GPIO_InitStruct.Pin = Motor5_INB_Pin | Motor5_INA_Pin | Motor4_INB_Pin
			| Motor4_INA_Pin | Motor3_INB_Pin | Motor3_INA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : Motor5_ENC_Pin Motor4_ENC_Pin Motor3_ENC_Pin */
	GPIO_InitStruct.Pin = Motor5_ENC_Pin | Motor4_ENC_Pin | Motor3_ENC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Motor2_ENC_Pin Motor1_ENC_Pin */
	GPIO_InitStruct.Pin = Motor2_ENC_Pin | Motor1_ENC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
