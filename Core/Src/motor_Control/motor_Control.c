/*
 * motor_Control.c
 *
 *  Created on: Sep 14, 2023
 *      Author: taji
 */
#include "motor_Control.h"
#include "ESP_UART.h"
#include "PID.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"
//-------------- variable -------------------
Fingers_Struct Fingers_Status;
uint32_t Current_motor[6];
qPID controller;
uint8_t ManualControl=0;
volatile uint16_t ADCData[6];
volatile uint16_t calibration_counter=0;
//-------------- structure -------------------
//-------------- enumeration -------------------
//-------------- function -------------------
/*
 * Read Encoder Signals
 */
void Read_Encoder (Finger_Struct* FingerStruct,Fingers_Name_Enum FingerName)
{
	//	_Bool Signal_A;
	//	_Bool Signal_B;
	switch (FingerName) {
	case Thumb:
		FingerStruct->SignalA=HAL_GPIO_ReadPin(Motor5_Encoder1_GPIO_Port,Motor5_Encoder1_Pin);
		FingerStruct->SignalB=HAL_GPIO_ReadPin(Motor5_Encoder2_GPIO_Port,Motor5_Encoder2_Pin);

		break;
	case Index:
		FingerStruct->SignalA=HAL_GPIO_ReadPin(Motor4_Encoder1_GPIO_Port,Motor4_Encoder1_Pin);
		FingerStruct->SignalB=HAL_GPIO_ReadPin(Motor4_Encoder2_GPIO_Port,Motor4_Encoder2_Pin);
		break;
	case Middle:
		FingerStruct->SignalA=HAL_GPIO_ReadPin(Motor3_Encoder1_GPIO_Port,Motor3_Encoder1_Pin);
		FingerStruct->SignalB=HAL_GPIO_ReadPin(Motor3_Encoder2_GPIO_Port,Motor3_Encoder2_Pin);
		break;
	case Ring:
		FingerStruct->SignalA=HAL_GPIO_ReadPin(Motor2_Encoder1_GPIO_Port,Motor2_Encoder1_Pin);
		FingerStruct->SignalB=HAL_GPIO_ReadPin(Motor2_Encoder2_GPIO_Port,Motor2_Encoder2_Pin);
		break;
	case Pinky:
		FingerStruct->SignalA=HAL_GPIO_ReadPin(Motor1_Encoder1_GPIO_Port,Motor1_Encoder1_Pin);
		FingerStruct->SignalB=HAL_GPIO_ReadPin(Motor1_Encoder2_GPIO_Port,Motor1_Encoder2_Pin);
		break;
	default:
		break;
	}
	if(FingerStruct->SignalA && FingerStruct->SignalB)
		FingerStruct->current_Encoder_State=Ahigh_Bhigh;
	else if (FingerStruct->SignalA && !FingerStruct->SignalB)
		FingerStruct->current_Encoder_State=Ahigh_Blow;
	else if (!FingerStruct->SignalA && FingerStruct->SignalB)
		FingerStruct->current_Encoder_State=Alow_Bhigh;
	else
		FingerStruct->current_Encoder_State=Alow_Blow;
	if(FingerStruct->current_Encoder_State != FingerStruct->Pre_Encoder_State)
	{
		//if(FingerStruct->Direction==Open)
		//	FingerStruct->Encoder++;
		//else if(FingerStruct->Direction==Close)
		//	FingerStruct->Encoder--;
		if(FingerStruct->Pre_Encoder_State==Ahigh_Blow && FingerStruct->current_Encoder_State==Alow_Blow)
			FingerStruct->Encoder++;
		else if(FingerStruct->Pre_Encoder_State==Alow_Blow && FingerStruct->current_Encoder_State==Ahigh_Blow)
			FingerStruct->Encoder--;
		else
			FingerStruct->Encoder+=FingerStruct->Pre_Encoder_State-FingerStruct->current_Encoder_State;
		FingerStruct->Pre_Encoder_State=FingerStruct->current_Encoder_State;
	}
}
/*
 * motor control
 */
void SetMotor(Fingers_Name_Enum name,Finger_Struct* FingerStruct) {
	//	Read_Encoder(FingerStruct, name);
	switch (name) {
	case Thumb :
		if ( FingerStruct->Direction== Close) {
			htim3.Instance->CCR1 = FingerStruct->speed;
			htim3.Instance->CCR2 =0;
		} else if (FingerStruct->Direction == Open) {
			htim3.Instance->CCR2 = FingerStruct->speed;
			htim3.Instance->CCR1 =0;
		} else {
			htim3.Instance->CCR1 =100;
			htim3.Instance->CCR2 =100;
		}
		//calculate position
		FingerStruct->position=((float)FingerStruct->Encoder/Max_Encoder_Thumb)*100;
		break;
	case Index :
		if ( FingerStruct->Direction== Close) {
			htim4.Instance->CCR1 = FingerStruct->speed;
			htim4.Instance->CCR2 =0;
		} else if (FingerStruct->Direction == Open) {
			htim4.Instance->CCR2 = FingerStruct->speed;
			htim4.Instance->CCR1 =0;
		} else {
			htim4.Instance->CCR1 =100;
			htim4.Instance->CCR2 =100;
		}
		FingerStruct->position=((float)FingerStruct->Encoder/Max_Encoder_Index)*100;
		break;
	case Middle :
		if ( FingerStruct->Direction== Close) {
			htim2.Instance->CCR4 = FingerStruct->speed;
			htim2.Instance->CCR3 =0;
		} else if (FingerStruct->Direction == Open) {
			htim2.Instance->CCR3 = FingerStruct->speed;
			htim2.Instance->CCR4 =0;
		} else {
			htim2.Instance->CCR3 =100;
			htim2.Instance->CCR4 =100;
		}
		FingerStruct->position=((float)FingerStruct->Encoder/Max_Encoder_Middle)*100;
		break;
	case Ring :
		if ( FingerStruct->Direction== Close) {
			htim8.Instance->CCR1 = FingerStruct->speed;
			htim8.Instance->CCR2 =0;
		} else if (FingerStruct->Direction == Open) {
			htim8.Instance->CCR2 = FingerStruct->speed;
			htim8.Instance->CCR1 =0;
		} else {
			htim8.Instance->CCR1 =100;
			htim8.Instance->CCR2 =100;
		}
		FingerStruct->position=((float)FingerStruct->Encoder/Max_Encoder_Ring)*100;
		break;
	case Pinky :
		if ( FingerStruct->Direction== Close) {
			htim1.Instance->CCR1 = FingerStruct->speed;
			htim1.Instance->CCR2 =0;
		} else if (FingerStruct->Direction == Open) {
			htim1.Instance->CCR2 = FingerStruct->speed;
			htim1.Instance->CCR1 =0;
		} else {
			htim1.Instance->CCR1 =100;
			htim1.Instance->CCR2 =100;
		}
		FingerStruct->position=((float)FingerStruct->Encoder/Max_Encoder_Pinky)*100;
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
	//motor5->thumb
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//motor4->index
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	//motor3->middle
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	//motor2->ring
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	//motor1->pinky
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	//initialization
	HAL_ADC_MspInit(&hadc1);
	HAL_ADC_MspInit(&hadc2);
	HAL_UART_MspInit(&huart4);
	HAL_UART_Receive_IT(&huart4, &RXuart, 1);
	HAL_UART_MspInit(&huart4);






	TX_State=idel;
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

void Fingers_Calibration(void)
{
	//read current until stable
	while(calibration_counter<500)
	{
		ADC_ReadCurrent_Thumb();
		ADC_ReadCurrent_Index();
		ADC_ReadCurrent_Middle();
		ADC_ReadCurrent_Ring();
		ADC_ReadCurrent_Pinky();
	}
	//------------------------------| Thumb finger |----------------------------------------
	Fingers_Status.Thumb.Direction=Open;
	Fingers_Status.Thumb.speed=60;
	calibration_counter=0;
	while(Fingers_Status.Thumb.Stuck_Finger==0 || calibration_counter<40)
	{
		SetMotor(Thumb, &Fingers_Status.Thumb);
		ADC_ReadCurrent_Thumb();
	}
	Fingers_Status.Thumb.Direction=Stop;
	Fingers_Status.Thumb.speed=0;
	SetMotor(Thumb, &Fingers_Status.Thumb);
	Fingers_Status.Thumb.Stuck_Finger=0;
	Fingers_Status.Thumb.Encoder=Max_Encoder_Thumb;
	//------------------------------| Index finger |----------------------------------------
	Fingers_Status.Index.Direction=Open;
	Fingers_Status.Index.speed=60;
	calibration_counter=0;
	while(Fingers_Status.Index.Stuck_Finger==0 || calibration_counter<50)
	{
		SetMotor(Index, &Fingers_Status.Index);
		ADC_ReadCurrent_Index();
	}
	Fingers_Status.Index.Direction=Stop;
	Fingers_Status.Index.speed=0;
	SetMotor(Index, &Fingers_Status.Index);
	Fingers_Status.Index.Stuck_Finger=0;
	Fingers_Status.Index.Encoder=Max_Encoder_Index;
	//------------------------------| Middle finger |----------------------------------------
	Fingers_Status.Middle.Direction=Open;
	Fingers_Status.Middle.speed=60;
	calibration_counter=0;
	while(Fingers_Status.Middle.Stuck_Finger==0 || calibration_counter<40)
	{
		SetMotor(Middle, &Fingers_Status.Middle);
		ADC_ReadCurrent_Middle();
	}
	Fingers_Status.Middle.Direction=Stop;
	Fingers_Status.Middle.speed=0;
	SetMotor(Middle, &Fingers_Status.Middle);
	Fingers_Status.Middle.Stuck_Finger=0;
	Fingers_Status.Middle.Encoder=Max_Encoder_Middle;
	//------------------------------| Ring finger |----------------------------------------
	Fingers_Status.Ring.Direction=Open;
	Fingers_Status.Ring.speed=60;
	calibration_counter=0;
	while(Fingers_Status.Ring.Stuck_Finger==0 || calibration_counter<40)
	{
	SetMotor(Ring, &Fingers_Status.Ring);
	ADC_ReadCurrent_Ring();
	}
	Fingers_Status.Ring.Direction=Stop;
	Fingers_Status.Ring.speed=0;
	SetMotor(Ring, &Fingers_Status.Ring);
	Fingers_Status.Ring.Stuck_Finger=0;
	Fingers_Status.Ring.Encoder=Max_Encoder_Middle;
	//------------------------------| Pinky finger |----------------------------------------
	Fingers_Status.Pinky.Direction=Open;
	Fingers_Status.Pinky.speed=60;
	calibration_counter=0;
	while(Fingers_Status.Pinky.Stuck_Finger==0 || calibration_counter<40)
	{
	SetMotor(Pinky, &Fingers_Status.Pinky);
	ADC_ReadCurrent_Pinky();
	}
	Fingers_Status.Pinky.Direction=Stop;
	Fingers_Status.Pinky.speed=0;
	SetMotor(Pinky, &Fingers_Status.Pinky);
	Fingers_Status.Pinky.Stuck_Finger=0;
	Fingers_Status.Pinky.Encoder=Max_Encoder_Middle;
}
