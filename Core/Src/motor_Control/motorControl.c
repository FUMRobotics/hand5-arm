/*
 * motorControl.c
 *
 *  Created on: May 9, 2023
 *      Author: NEW
 */
/*user include---------------------*/
#include "main.h"
#include "motorControl.h"
/*END user include-----------------*/
/* USER CODE BEGIN PV */
uint8_t dummy = 0;
//uart buffer
char uartRecieveBuffer[300];
uint16_t uartCounter = 0;
uint8_t RXuart=0;
HandState_Typedef HandStruct[5]={0,};

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

uint16_t Thumb_Position = 0;
uint16_t Index_Position = 0;
uint16_t Middle_Position = 0;
uint16_t Ring_Position = 0;
uint16_t Pinky_Position = 0;

uint16_t Thumb_Position_SetPoint = 0;
uint16_t Index_Position_SetPoint = 0;
uint16_t Middle_Position_SetPoint = 0;
uint16_t Ring_Position_SetPoint = 0;
uint16_t Pinky_Position_SetPoint = 0;

uint8_t Thumb_LastDirection = 0;
uint8_t Index_LastDirection = 0;
uint8_t Middle_LastDirection = 0;
uint8_t Ring_LastDirection = 0;
uint8_t Pinky_LastDirection = 0;
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
	pid->Out_Max = 99;
	pid->Out_Min = -99;
	pid->KP = 0;
	pid->KI = 0;
	pid->KD = 0;
	pid->I_Max = 50;
	pid->I_Min = -50;
	pid->LoopPeriod = 0.01;
}
/*
 * Function6--------------------------
 */
void ApplyPIDToMotor(PID *pid, uint8_t DesiredMotor) {
	if (pid->Out == 0) {
		SetMotor(DesiredMotor, FINGER_Stop, pid->Out);
	} else if (pid->Out > 0) {
		SetMotor(DesiredMotor, FINGER_Open, pid->Out);
	} else {
		SetMotor(DesiredMotor, FINGER_Close, -pid->Out);
	}
}
/*END Functions--------------------*/
