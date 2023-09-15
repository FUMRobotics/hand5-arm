/*
 * motor_Control.c
 *
 *  Created on: Sep 14, 2023
 *      Author: Lenovo
 */

#include "motor_Control.h"
#include "PID.h"
#include "tim.h"
#include "adc.h"
//-------------- variable -------------------
Fingers_Struct Fingers_Status;
uint32_t Current_motor[6];
qPID controller;
uint8_t ManualControl=0;
//-------------- structure -------------------
//-------------- enumeration -------------------
//-------------- function -------------------
/*
 * Read Encoder Signals
 */
void Read_Encoder (Finger_Struct* FingerStruct,Fingers_Name_Enum FingerName)
{
	_Bool Signal_A;
	_Bool Signal_B;
	switch (FingerName) {
	case Thumb:
		Signal_A=HAL_GPIO_ReadPin(Motor5_Encoder1_GPIO_Port,Motor5_Encoder1_Pin);
		Signal_B=HAL_GPIO_ReadPin(Motor5_Encoder2_GPIO_Port,Motor5_Encoder2_Pin);
		if(Signal_A && Signal_B)
			FingerStruct->current_Encoder_State=Ahigh_Bhigh;
		else if (Signal_A && !Signal_B)
			FingerStruct->current_Encoder_State=Ahigh_Blow;
		else if (!Signal_A && Signal_B)
			FingerStruct->current_Encoder_State=Alow_Bhigh;
		else
			FingerStruct->current_Encoder_State=Alow_Blow;
		if(FingerStruct->current_Encoder_State != FingerStruct->Pre_Encoder_State)
		{
			FingerStruct->Encoder+=FingerStruct->Pre_Encoder_State-FingerStruct->current_Encoder_State;
			FingerStruct->Pre_Encoder_State=FingerStruct->current_Encoder_State;
		}
		break;
	case Index:
		Signal_A=HAL_GPIO_ReadPin(Motor4_Encoder1_GPIO_Port,Motor4_Encoder1_Pin);
		Signal_B=HAL_GPIO_ReadPin(Motor4_Encoder2_GPIO_Port,Motor4_Encoder2_Pin);
		if(Signal_A && Signal_B)
			FingerStruct->current_Encoder_State=Ahigh_Bhigh;
		else if (Signal_A && !Signal_B)
			FingerStruct->current_Encoder_State=Ahigh_Blow;
		else if (!Signal_A && Signal_B)
			FingerStruct->current_Encoder_State=Alow_Bhigh;
		else
			FingerStruct->current_Encoder_State=Alow_Blow;
		if(FingerStruct->current_Encoder_State != FingerStruct->Pre_Encoder_State)
		{
			FingerStruct->Encoder+=FingerStruct->Pre_Encoder_State-FingerStruct->current_Encoder_State;
			FingerStruct->Pre_Encoder_State=FingerStruct->current_Encoder_State;
		}
		break;
	case Middle:
		Signal_A=HAL_GPIO_ReadPin(Motor3_Encoder1_GPIO_Port,Motor3_Encoder1_Pin);
		Signal_B=HAL_GPIO_ReadPin(Motor3_Encoder2_GPIO_Port,Motor3_Encoder2_Pin);
		if(Signal_A && Signal_B)
			FingerStruct->current_Encoder_State=Ahigh_Bhigh;
		else if (Signal_A && !Signal_B)
			FingerStruct->current_Encoder_State=Ahigh_Blow;
		else if (!Signal_A && Signal_B)
			FingerStruct->current_Encoder_State=Alow_Bhigh;
		else
			FingerStruct->current_Encoder_State=Alow_Blow;
		if(FingerStruct->current_Encoder_State != FingerStruct->Pre_Encoder_State)
		{
			FingerStruct->Encoder+=FingerStruct->Pre_Encoder_State-FingerStruct->current_Encoder_State;
			FingerStruct->Pre_Encoder_State=FingerStruct->current_Encoder_State;
		}
		break;
	case Ring:
		Signal_A=HAL_GPIO_ReadPin(Motor2_Encoder1_GPIO_Port,Motor2_Encoder1_Pin);
		Signal_B=HAL_GPIO_ReadPin(Motor2_Encoder2_GPIO_Port,Motor2_Encoder2_Pin);
		if(Signal_A && Signal_B)
			FingerStruct->current_Encoder_State=Ahigh_Bhigh;
		else if (Signal_A && !Signal_B)
			FingerStruct->current_Encoder_State=Ahigh_Blow;
		else if (!Signal_A && Signal_B)
			FingerStruct->current_Encoder_State=Alow_Bhigh;
		else
			FingerStruct->current_Encoder_State=Alow_Blow;
		if(FingerStruct->current_Encoder_State != FingerStruct->Pre_Encoder_State)
		{
			FingerStruct->Encoder+=FingerStruct->Pre_Encoder_State-FingerStruct->current_Encoder_State;
			FingerStruct->Pre_Encoder_State=FingerStruct->current_Encoder_State;
		}
		break;
	case Pinky:
		Signal_A=HAL_GPIO_ReadPin(Motor1_Encoder1_GPIO_Port,Motor1_Encoder1_Pin);
		Signal_B=HAL_GPIO_ReadPin(Motor1_Encoder2_GPIO_Port,Motor1_Encoder2_Pin);
		if(Signal_A && Signal_B)
			FingerStruct->current_Encoder_State=Ahigh_Bhigh;
		else if (Signal_A && !Signal_B)
			FingerStruct->current_Encoder_State=Ahigh_Blow;
		else if (!Signal_A && Signal_B)
			FingerStruct->current_Encoder_State=Alow_Bhigh;
		else
			FingerStruct->current_Encoder_State=Alow_Blow;
		if(FingerStruct->current_Encoder_State != FingerStruct->Pre_Encoder_State)
		{
			FingerStruct->Encoder+=FingerStruct->Pre_Encoder_State-FingerStruct->current_Encoder_State;
			FingerStruct->Pre_Encoder_State=FingerStruct->current_Encoder_State;
		}
		break;
	default:
		break;
	}
}
/*
 * motor control
 */
void SetMotor(Fingers_Name_Enum name,Finger_Struct  FingerStruct) {
	switch (name) {
	case Thumb :
		if ( FingerStruct.Direction== Open) {
			htim2.Instance->CCR1 = FingerStruct.speed;
			HAL_GPIO_WritePin(Motor5_INA_GPIO_Port, Motor5_INA_Pin, 0);
			HAL_GPIO_WritePin(Motor5_INB_GPIO_Port, Motor5_INB_Pin, 1);
		} else if (FingerStruct.Direction == Close) {
			htim2.Instance->CCR1 = FingerStruct.speed;
			HAL_GPIO_WritePin(Motor5_INA_GPIO_Port, Motor5_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor5_INB_GPIO_Port, Motor5_INB_Pin, 0);
		} else {
			htim2.Instance->CCR1 = 0;
			HAL_GPIO_WritePin(Motor5_INA_GPIO_Port, Motor5_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor5_INB_GPIO_Port, Motor5_INB_Pin, 1);
		}
		break;
	case Index :
		if ( FingerStruct.Direction== Open) {
			htim1.Instance->CCR4 = FingerStruct.speed;
			HAL_GPIO_WritePin(Motor4_INA_GPIO_Port, Motor4_INA_Pin, 0);
			HAL_GPIO_WritePin(Motor4_INB_GPIO_Port, Motor4_INB_Pin, 1);
		} else if (FingerStruct.Direction == Close) {
			htim1.Instance->CCR4 = FingerStruct.speed;
			HAL_GPIO_WritePin(Motor4_INA_GPIO_Port, Motor4_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor4_INB_GPIO_Port, Motor4_INB_Pin, 0);
		} else {
			htim1.Instance->CCR4 = 0;
			HAL_GPIO_WritePin(Motor4_INA_GPIO_Port, Motor4_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor4_INB_GPIO_Port, Motor4_INB_Pin, 1);
		}
		break;
	case Middle :
		if ( FingerStruct.Direction== Open) {
			htim1.Instance->CCR3 = FingerStruct.speed;
			HAL_GPIO_WritePin(Motor3_INA_GPIO_Port, Motor3_INA_Pin, 0);
			HAL_GPIO_WritePin(Motor3_INB_GPIO_Port, Motor3_INB_Pin, 1);
		} else if (FingerStruct.Direction == Close) {
			htim1.Instance->CCR3 = FingerStruct.speed;
			HAL_GPIO_WritePin(Motor3_INA_GPIO_Port, Motor3_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor3_INB_GPIO_Port, Motor3_INB_Pin, 0);
		} else {
			htim1.Instance->CCR3 = 0;
			HAL_GPIO_WritePin(Motor3_INA_GPIO_Port, Motor3_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor3_INB_GPIO_Port, Motor3_INB_Pin, 1);
		}
		break;
	case Ring :
		if ( FingerStruct.Direction== Open) {
			htim1.Instance->CCR2 = FingerStruct.speed;
			HAL_GPIO_WritePin(Motor2_INA_GPIO_Port, Motor2_INA_Pin, 0);
			HAL_GPIO_WritePin(Motor2_INB_GPIO_Port, Motor2_INB_Pin, 1);
		} else if (FingerStruct.Direction == Close) {
			htim1.Instance->CCR2 = FingerStruct.speed;
			HAL_GPIO_WritePin(Motor2_INA_GPIO_Port, Motor2_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor2_INB_GPIO_Port, Motor2_INB_Pin, 0);
		} else {
			htim1.Instance->CCR2 = 0;
			HAL_GPIO_WritePin(Motor2_INA_GPIO_Port, Motor2_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor2_INB_GPIO_Port, Motor2_INB_Pin, 1);
		}
		break;
	case Pinky :
		if ( FingerStruct.Direction== Open) {
			htim1.Instance->CCR1 = FingerStruct.speed;
			HAL_GPIO_WritePin(Motor1_INA_GPIO_Port, Motor1_INA_Pin, 0);
			HAL_GPIO_WritePin(Motor1_INB_GPIO_Port, Motor1_INB_Pin, 1);
		} else if (FingerStruct.Direction == Close) {
			htim1.Instance->CCR1 = FingerStruct.speed;
			HAL_GPIO_WritePin(Motor1_INA_GPIO_Port, Motor1_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor1_INB_GPIO_Port, Motor1_INB_Pin, 0);
		} else {
			htim1.Instance->CCR1 = 0;
			HAL_GPIO_WritePin(Motor1_INA_GPIO_Port, Motor1_INA_Pin, 1);
			HAL_GPIO_WritePin(Motor1_INB_GPIO_Port, Motor1_INB_Pin, 1);
		}
		break;

	default:

		break;
	}
}
/*
 *
 */
void init_motor_controller(void)
{
	//start read data from ADC
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_ADC_MspInit(&hadc1);
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
