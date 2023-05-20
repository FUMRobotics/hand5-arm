/*
 * motorControl.c
 *
 *  Created on: May 9, 2023
 *      Author: NEW
 */
/*user include---------------------*/
#include "main.h"
#include "motorControl.h"
#include "ESP8266_UART.h"
#include "PID.h"
/*END user include-----------------*/
	// Declare de new object
qPID controller;
//uart buffer
char uartRecieveBuffer[150];
uint16_t uartCounter = 0;
uint8_t RXuart=0;
volatile HandState_Typedef HandStruct[5]={0,};

uint16_t DATA[5] = { 0 };
_Bool SaveData_EEprom=0;

uint8_t Thumb_ManualControl = FINGER_Stop;
uint8_t Index_ManualControl = FINGER_Stop;
uint8_t Middle_ManualControl = FINGER_Stop;
uint8_t Ring_ManualControl = FINGER_Stop;
uint8_t Pinky_ManualControl = FINGER_Stop;
uint8_t GestureManualControl = 0;
uint8_t ManualControlActive = 0;
uint8_t ManualControlPWM = 99;

volatile uint16_t Thumb_Position = 0;
volatile uint16_t Index_Position = 0;
volatile uint16_t Middle_Position = 0;
volatile uint16_t Ring_Position = 0;
volatile uint16_t Pinky_Position = 0;
volatile uint16_t Last_Thumb_Position = 0;
volatile uint16_t Last_Index_Position = 0;
volatile uint16_t Last_Middle_Position = 0;
volatile uint16_t Last_Ring_Position = 0;
volatile uint16_t Last_Pinky_Position = 0;

volatile uint32_t Check_Position_Counter=0;
float Index_PercentPosition=0;
float Middle_PercentPosition=0;
float Ring_PercentPosition=0;
float Pinky_PercentPosition=0;
float Thumb_PercentPosition=0;

uint16_t Thumb_Position_Stack =0;
uint16_t Index_Position_Stack =0;
uint16_t Middle_Position_Stack=0;
uint16_t Ring_Position_Stack  =0;
uint16_t Pinky_Position_Stack =0;
_Bool Thumb_Position_StackFlag =0;
_Bool Index_Position_StackFlag =0;
_Bool Middle_Position_StackFlag=0;
_Bool Ring_Position_StackFlag  =0;
_Bool Pinky_Position_StackFlag =0;

volatile uint8_t Thumb_LastDirection = FINGER_Stop;
volatile uint8_t Index_LastDirection = FINGER_Stop;
volatile uint8_t Middle_LastDirection = FINGER_Stop;
volatile uint8_t Ring_LastDirection = FINGER_Stop;
volatile uint8_t Pinky_LastDirection = FINGER_Stop;
float CRNT, SPD;

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

/* USER CODE END PV */

/*Functions------------------------*/

/*
 * Function1--------------------------
 */
void init_motor_controller(void)
{
	//start read data from ADC
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_ADC_MspInit(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) DATA, 5);
	HAL_TIM_Base_Start_IT(&htim4);

	// Configure settings
	controller.AntiWindup = ENABLED;
	controller.Bumpless = ENABLED;
	// Configure de output limits for clamping
	controller.OutputMax = 50.0;
	controller.OutputMin = -50.0;
	// Set the rate at the PID will run in seconds
	controller.Ts = 3;
	// More settings
	controller.b = 1.0;
	controller.c = 1.0;
	// Init de controller
	qPID_Init(&controller);
	// Set the tunning constants
	controller.K = 0.5;
	controller.Ti = 1/0.02;
	controller.Td = 1.0;
	controller.Nd = 3.0;
	// Set mode to auotmatic (otherwise it will be in manual mode)
	controller.Mode = AUTOMATIC;
}
/*
 * Function2--------------------------
 */
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
/*
 * Function3--------------------------
 */
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
/*
 * Function4--------------------------
 */
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
/*
 * Function5--------------------------
 */
void PID_Position_Defaults(PID *pid) {
	pid->Out_Max = 100;
	pid->Out_Min = 0;
	pid->KP = 0.01;
	pid->KI = 0.01;
	pid->KD = 0.001;
	pid->I_Max = 50;
	pid->I_Min = -50;
	pid->LoopPeriod = 0.01;
}
/*
 * Function6--------------------------
 */
void ApplyPIDToMotor(uint8_t DesiredMotor) {
	float output=0;
	switch (DesiredMotor) {
	case Thumb_Motor:
		output= qPID_Process(&controller, HandStruct[Thumb_Motor-1].Value, Thumb_PercentPosition);
		if((uint8_t)Thumb_PercentPosition==HandStruct[Thumb_Motor-1].Value)
		{
			output=0;
			SetMotor(Thumb_Motor, FINGER_Stop, output);
			Thumb_LastDirection=FINGER_Stop;
		}
		if(output>0)
		{
			SetMotor(Thumb_Motor, FINGER_Open, output);
			Thumb_LastDirection=FINGER_Open;
		}else
		{
			SetMotor(Thumb_Motor, FINGER_Close, -output);
			Thumb_LastDirection=FINGER_Close;
		}
		if(((Check_Position_Counter/10)/10)%2)
		{
			if(!Thumb_Position_StackFlag)
			{
				if(output!=0)
				{
					if(Thumb_Position_Stack!=Thumb_Position)
					{
						Thumb_Position_Stack=Thumb_Position;
					}else
					{
						Thumb_Position_StackFlag=1;
					}
				}
			}else
			{
				if(output!=0)
				{
					if(Thumb_Position_Stack!=Thumb_Position)
					{
						Thumb_Position_Stack=Thumb_Position;
						Thumb_Position_StackFlag=0;
					}else
					{
						output=0;
						SetMotor(Thumb_Motor, FINGER_Stop, output);
						Thumb_LastDirection=FINGER_Stop;
						Thumb_Position_StackFlag=1;
					}
				}
				else{
					Thumb_Position_StackFlag=0;
				}
			}
		}
		break;
	case Index_Motor:
		 output = qPID_Process(&controller, HandStruct[Index_Motor-1].Value, Index_PercentPosition);
		if((uint8_t)Index_PercentPosition==HandStruct[Index_Motor-1].Value)
		{
			output=0;
			SetMotor(Index_Motor, FINGER_Stop, output);
			Index_LastDirection=FINGER_Stop;
		}
		if(output>0)
		{
			SetMotor(Index_Motor, FINGER_Open, output);
			Index_LastDirection=FINGER_Open;
		}else
		{
			SetMotor(Index_Motor, FINGER_Close, -output);
			Index_LastDirection=FINGER_Close;
		}
		if((Check_Position_Counter/10)%2)
		{
			if(!Index_Position_StackFlag)
			{
				if(output!=0)
				{
					if(Index_Position_Stack!=Index_Position)
					{
						Index_Position_Stack=Index_Position;
					}else
					{
						Index_Position_StackFlag=1;
					}
				}
			}else
			{
				if(output!=0)
				{
					if(Index_Position_Stack!=Index_Position)
					{
						Index_Position_Stack=Index_Position;
						Index_Position_StackFlag=0;
					}else
					{
						output=0;
						SetMotor(Index_Motor, FINGER_Stop, output);
						Index_LastDirection=FINGER_Stop;
						Index_Position_StackFlag=1;
					}
				}
				else{
					Index_Position_StackFlag=0;
				}
			}
		}
		break;
	case Middle_Motor:
		output = qPID_Process(&controller, HandStruct[Middle_Motor-1].Value, Middle_PercentPosition);
		if((uint8_t)Middle_PercentPosition==HandStruct[Middle_Motor-1].Value)
		{
			output=0;
			SetMotor(Middle_Motor, FINGER_Stop, output);
			Middle_LastDirection=FINGER_Stop;
		}
		if(output>0)
		{
			SetMotor(Middle_Motor, FINGER_Open, output);
			Middle_LastDirection=FINGER_Open;
		}else
		{
			SetMotor(Middle_Motor, FINGER_Close, -output);
			Middle_LastDirection=FINGER_Close;
		}
		if((Check_Position_Counter/10)%2)
		{
			if(!Middle_Position_StackFlag)
			{
				if(output!=0)
				{
					if(Middle_Position_Stack!=Middle_Position)
					{
						Middle_Position_Stack=Middle_Position;
					}else
					{
						Middle_Position_StackFlag=1;
					}
				}
			}else
			{
				if(output!=0)
				{
					if(Middle_Position_Stack!=Middle_Position)
					{
						Middle_Position_Stack=Middle_Position;
						Middle_Position_StackFlag=0;
					}else
					{
						output=0;
						SetMotor(Middle_Motor, FINGER_Stop, output);
						Middle_LastDirection=FINGER_Stop;
						Middle_Position_StackFlag=1;
					}
				}
				else{
					Middle_Position_StackFlag=0;
				}
			}
		}

		break;
	case Ring_Motor:
		output = qPID_Process(&controller, HandStruct[Ring_Motor-1].Value, Ring_PercentPosition);
		if((uint8_t)Ring_PercentPosition==HandStruct[Ring_Motor-1].Value)
		{
			output=0;
			SetMotor(Ring_Motor, FINGER_Stop, output);
			Ring_LastDirection=FINGER_Stop;
		}
		if(output>0)
		{
			SetMotor(Ring_Motor, FINGER_Open, output);
			Ring_LastDirection=FINGER_Open;
		}else
		{
			SetMotor(Ring_Motor, FINGER_Close, -output);
			Ring_LastDirection=FINGER_Close;
		}
		if((Check_Position_Counter/20)%2)
		{
			if(!Ring_Position_StackFlag)
			{
				if(output!=0)
				{
					if(Ring_Position_Stack!=Ring_Position)
					{
						Ring_Position_Stack=Ring_Position;
					}else
					{
						Ring_Position_StackFlag=1;
					}
				}
			}else
			{
				if(output!=0)
				{
					if(Ring_Position_Stack!=Ring_Position)
					{
						Ring_Position_Stack=Ring_Position;
						Ring_Position_StackFlag=0;
					}else
					{
						output=0;
						SetMotor(Ring_Motor, FINGER_Stop, output);
						Ring_LastDirection=FINGER_Stop;
						Ring_Position_StackFlag=1;
					}
				}
				else{
					Ring_Position_StackFlag=0;
				}
			}
		}
		break;
	case Pinky_Motor:
		output = qPID_Process(&controller, HandStruct[Pinky_Motor-1].Value, Pinky_PercentPosition);
		if((uint8_t)Pinky_PercentPosition==HandStruct[Pinky_Motor-1].Value)
		{
			output=0;
			SetMotor(Pinky_Motor, FINGER_Stop, output);
			Pinky_LastDirection=FINGER_Stop;
		}
		if(output>0)
		{
			SetMotor(Pinky_Motor, FINGER_Open, output);
			Pinky_LastDirection=FINGER_Open;
		}else
		{
			SetMotor(Pinky_Motor, FINGER_Close, -output);
			Pinky_LastDirection=FINGER_Close;
		}
		if((Check_Position_Counter/10)%2)
		{
			if(!Pinky_Position_StackFlag)
			{
				if(output!=0)
				{
					if(Pinky_Position_Stack!=Pinky_Position)
					{
						Pinky_Position_Stack=Pinky_Position;
					}else
					{
						Pinky_Position_StackFlag=1;
					}
				}
			}else
			{
				if(output!=0)
				{
					if(Pinky_Position_Stack!=Pinky_Position)
					{
						Pinky_Position_Stack=Pinky_Position;
						Pinky_Position_StackFlag=0;
					}else
					{
						output=0;
						SetMotor(Pinky_Motor, FINGER_Stop, output);
						Pinky_LastDirection=FINGER_Stop;
						Pinky_Position_StackFlag=1;
					}
				}
				else{
					Pinky_Position_StackFlag=0;
				}
			}
		}

		break;
	default:
		break;
	}
}
/*END Functions--------------------*/
