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
uint8_t ManualControl=0;
volatile uint16_t ADCData[6];
volatile uint16_t calibration_counter=0;
//-------------- structure -------------------
//-------------- enumeration -------------------
//-------------- function -------------------
/*
 * Read Encoder Signals
 */
void Read_Encoder (Fingers_Name_Enum FingerName,Finger_Struct* FingerStruct)
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
			FingerStruct->Encoder+=(FingerStruct->Pre_Encoder_State-FingerStruct->current_Encoder_State);
		FingerStruct->Pre_Encoder_State=FingerStruct->current_Encoder_State;
		if(FingerStruct->Encoder>65000)
			FingerStruct->Encoder=0;
	}
}
/*
 * set PWM and direction for each finger
 */
void SetMotor(Fingers_Name_Enum FingerName,Finger_Struct* FingerStruct) {
	//	Read_Encoder(FingerStruct, name);
	if(control_mode==speed_mode)
	{
		if(FingerStruct->SetPoint>0)
		{
			FingerStruct->speed=FingerStruct->SetPoint;
			FingerStruct->Direction_motor=Open;
			FingerStruct->Direction_Encoder=Open;
		}else if(FingerStruct->SetPoint<0)
		{
			FingerStruct->speed=FingerStruct->SetPoint*(-1);
			FingerStruct->Direction_motor=Close;
			FingerStruct->Direction_Encoder=Close;
		}else
		{
			FingerStruct->Direction_motor=Stop;
			FingerStruct->Direction_Encoder=Stop;
		}
	}
	switch (FingerName) {
	case Thumb :
		if ( FingerStruct->Direction_motor== Open) {
			htim3.Instance->CCR1 = (uint8_t) FingerStruct->speed;
			htim3.Instance->CCR2 =0;
		} else if (FingerStruct->Direction_motor == Close) {
			htim3.Instance->CCR2 =(uint8_t)  FingerStruct->speed;
			htim3.Instance->CCR1 =0;
		} else {
			htim3.Instance->CCR1 =100;
			htim3.Instance->CCR2 =100;
		}
		//calculate position
		FingerStruct->position=((float)FingerStruct->Encoder/Max_Encoder_Thumb)*100;
		break;
	case Index :
		if ( FingerStruct->Direction_motor== Close) {
			htim4.Instance->CCR1 =(uint8_t)  FingerStruct->speed;
			htim4.Instance->CCR2 =0;
		} else if (FingerStruct->Direction_motor == Open) {
			htim4.Instance->CCR2 = (uint8_t) FingerStruct->speed;
			htim4.Instance->CCR1 =0;
		} else {
			htim4.Instance->CCR1 =100;
			htim4.Instance->CCR2 =100;
		}
		FingerStruct->position=((float)FingerStruct->Encoder/Max_Encoder_Index)*100;
		break;
	case Middle :
		if ( FingerStruct->Direction_motor== Close) {
			htim2.Instance->CCR4 =(uint8_t)  FingerStruct->speed;
			htim2.Instance->CCR3 =0;
		} else if (FingerStruct->Direction_motor == Open) {
			htim2.Instance->CCR3 = (uint8_t) FingerStruct->speed;
			htim2.Instance->CCR4 =0;
		} else {
			htim2.Instance->CCR3 =100;
			htim2.Instance->CCR4 =100;
		}
		FingerStruct->position=((float)FingerStruct->Encoder/Max_Encoder_Middle)*100;
		break;
	case Ring :
		if ( FingerStruct->Direction_motor== Close) {
			htim8.Instance->CCR1 = (uint8_t) FingerStruct->speed;
			htim8.Instance->CCR2 =0;
		} else if (FingerStruct->Direction_motor == Open) {
			htim8.Instance->CCR2 =(uint8_t)  FingerStruct->speed;
			htim8.Instance->CCR1 =0;
		} else {
			htim8.Instance->CCR1 =100;
			htim8.Instance->CCR2 =100;
		}
		FingerStruct->position=((float)FingerStruct->Encoder/Max_Encoder_Ring)*100;
		break;
	case Pinky :
		if ( FingerStruct->Direction_motor== Close) {
			htim1.Instance->CCR1 =(uint8_t)  FingerStruct->speed;
			htim1.Instance->CCR2 =0;
		} else if (FingerStruct->Direction_motor == Open) {
			htim1.Instance->CCR2 =(uint8_t)  FingerStruct->speed;
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
	//-----------|start read data from ADC|---------------
	HAL_ADC_Start_DMA(&hadc2,(uint32_t *) ADCData, 6);
	//---------------|start PWM Timers|-------------------
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
	//---------------|initialization|----------------------
	HAL_ADC_MspInit(&hadc1);
	HAL_ADC_MspInit(&hadc2);
	HAL_UART_MspInit(&huart4);
	HAL_UART_Receive_IT(&huart4, &RXuart, 1);
	HAL_TIM_Base_Start_IT(&htim7);
	TX_State=idel;
	control_mode=position_mode;
	//-------------|Configure PID settings|----------------
	//********THUMB
	PID(&Fingers_Status.Thumb.PID_Struct, &Fingers_Status.Thumb.position, &Fingers_Status.Thumb.speed, &Fingers_Status.Thumb.SetPoint, 1, 3, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&Fingers_Status.Thumb.PID_Struct, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&Fingers_Status.Thumb.PID_Struct, 1);
	PID_SetOutputLimits(&Fingers_Status.Thumb.PID_Struct, 0, 100);
	//********INDEX
	PID(&Fingers_Status.Index.PID_Struct, &Fingers_Status.Index.position, &Fingers_Status.Index.speed, &Fingers_Status.Index.SetPoint, 1, 3, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&Fingers_Status.Index.PID_Struct, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&Fingers_Status.Index.PID_Struct, 1);
	PID_SetOutputLimits(&Fingers_Status.Index.PID_Struct, 0, 100);
	//********MIDDLE
	PID(&Fingers_Status.Middle.PID_Struct, &Fingers_Status.Middle.position, &Fingers_Status.Middle.speed, &Fingers_Status.Middle.SetPoint, 1, 3, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&Fingers_Status.Middle.PID_Struct, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&Fingers_Status.Middle.PID_Struct, 1);
	PID_SetOutputLimits(&Fingers_Status.Middle.PID_Struct, 0, 100);
	//********RING
	PID(&Fingers_Status.Ring.PID_Struct, &Fingers_Status.Ring.position, &Fingers_Status.Ring.speed, &Fingers_Status.Ring.SetPoint, 1, 3, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&Fingers_Status.Ring.PID_Struct, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&Fingers_Status.Ring.PID_Struct, 1);
	PID_SetOutputLimits(&Fingers_Status.Ring.PID_Struct, 0, 100);
	//********PINKY
	PID(&Fingers_Status.Pinky.PID_Struct, &Fingers_Status.Pinky.position, &Fingers_Status.Pinky.speed, &Fingers_Status.Pinky.SetPoint, 1, 3, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&Fingers_Status.Pinky.PID_Struct, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&Fingers_Status.Pinky.PID_Struct, 1);
	PID_SetOutputLimits(&Fingers_Status.Pinky.PID_Struct, 0, 100);
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
	//have mechanical problem

	//------------------------------| Thumb finger |----------------------------------------
	//	Fingers_Status.Thumb.Direction_motor=Open;
	//	calibration_counter=0;
	//	while(Fingers_Status.Thumb.Stuck_Finger==0 || calibration_counter<60)
	//	{
	//		Fingers_Status.Thumb.speed=60;
	//		SetMotor(Thumb, &Fingers_Status.Thumb);
	//		ADC_ReadCurrent_Thumb();
	//	}
	//	Fingers_Status.Thumb.Direction_motor=Stop;
	//	Fingers_Status.Thumb.speed=0;
	//	SetMotor(Thumb, &Fingers_Status.Thumb);
	//	Fingers_Status.Thumb.Stuck_Finger=0;
	//	Fingers_Status.Thumb.Encoder=Max_Encoder_Thumb;
	//	Fingers_Status.Thumb.SetPoint=100;
	//------------------------------| Index finger |----------------------------------------
	Fingers_Status.Index.Direction_motor=Open;
	calibration_counter=0;
	while(Fingers_Status.Index.Stuck_Finger==0 || calibration_counter<60)
	{
		Fingers_Status.Index.speed=60;
		SetMotor(Index, &Fingers_Status.Index);
		ADC_ReadCurrent_Index();
	}
	Fingers_Status.Index.Direction_motor=Stop;
	Fingers_Status.Index.speed=0;
	SetMotor(Index, &Fingers_Status.Index);
	Fingers_Status.Index.Stuck_Finger=0;
	Fingers_Status.Index.Encoder=Max_Encoder_Index;
	Fingers_Status.Index.SetPoint=100;
	//------------------------------| Middle finger |----------------------------------------
	Fingers_Status.Middle.Direction_motor=Open;
	calibration_counter=0;
	while(Fingers_Status.Middle.Stuck_Finger==0 || calibration_counter<60)
	{
		Fingers_Status.Middle.speed=60;
		SetMotor(Middle, &Fingers_Status.Middle);
		ADC_ReadCurrent_Middle();
	}
	Fingers_Status.Middle.Direction_motor=Stop;
	Fingers_Status.Middle.speed=0;
	SetMotor(Middle, &Fingers_Status.Middle);
	Fingers_Status.Middle.Stuck_Finger=0;
	Fingers_Status.Middle.Encoder=Max_Encoder_Middle;
	Fingers_Status.Middle.SetPoint=100;
	//------------------------------| Ring finger |----------------------------------------
	Fingers_Status.Ring.Direction_motor=Open;
	calibration_counter=0;
	while(Fingers_Status.Ring.Stuck_Finger==0 || calibration_counter<60)
	{
		Fingers_Status.Ring.speed=60;
		SetMotor(Ring, &Fingers_Status.Ring);
		ADC_ReadCurrent_Ring();
	}
	Fingers_Status.Ring.Direction_motor=Stop;
	Fingers_Status.Ring.speed=0;
	SetMotor(Ring, &Fingers_Status.Ring);
	Fingers_Status.Ring.Stuck_Finger=0;
	Fingers_Status.Ring.Encoder=Max_Encoder_Ring;
	Fingers_Status.Ring.SetPoint=100;
	//------------------------------| Pinky finger |----------------------------------------
	Fingers_Status.Pinky.Direction_motor=Open;
	calibration_counter=0;
	while(Fingers_Status.Pinky.Stuck_Finger==0 || calibration_counter<60)
	{
		Fingers_Status.Pinky.speed=60;
		SetMotor(Pinky, &Fingers_Status.Pinky);
		ADC_ReadCurrent_Pinky();
	}
	Fingers_Status.Pinky.Direction_motor=Stop;
	Fingers_Status.Pinky.speed=0;
	SetMotor(Pinky, &Fingers_Status.Pinky);
	Fingers_Status.Pinky.Stuck_Finger=0;
	Fingers_Status.Pinky.Encoder=Max_Encoder_Pinky;
	Fingers_Status.Pinky.SetPoint=100;
}
void Control_Motor(Fingers_Name_Enum FingerName,Finger_Struct* FingerStruct)
{
	if(control_mode==position_mode)
	{
		if(FingerStruct->SetPoint-FingerStruct->position>0.01)
		{
			FingerStruct->Direction_motor=Open;
			FingerStruct->Direction_Encoder=Open;
			FingerStruct->ChangeDirection=0;
			PID_SetControllerDirection(&FingerStruct->PID_Struct, _PID_CD_DIRECT);
		}
		else if (FingerStruct->SetPoint-FingerStruct->position<-0.01)
		{
			FingerStruct->Direction_motor=Close;
			FingerStruct->Direction_Encoder=Close;
			FingerStruct->ChangeDirection=0;
			PID_SetControllerDirection(&FingerStruct->PID_Struct, _PID_CD_REVERSE);
		}
		else
		{
			FingerStruct->Direction_motor=Stop;
		}
		if(FingerStruct->Stuck_Finger)
		{
			if( FingerStruct->Current_Counter>600)
			{
				if(FingerStruct->Direction_motor==Open)
				{
					switch (FingerName) {
					case Thumb:
						FingerStruct->Encoder=Max_Encoder_Thumb;
						break;
					case Index:
						FingerStruct->Encoder=Max_Encoder_Index;
						break;
					case Middle:
						FingerStruct->Encoder=Max_Encoder_Middle;
						break;
					case Ring:
						FingerStruct->Encoder=Max_Encoder_Ring;
						break;
					case Pinky:
						FingerStruct->Encoder=Max_Encoder_Pinky;
						break;
					default:
						break;
					}
				}
				else if(FingerStruct->Direction_motor==Close && FingerStruct->position<5 )
					FingerStruct->Encoder=0;
				FingerStruct->Direction_motor=Stop;
				FingerStruct->speed=0;
			}
		}else
			FingerStruct->Current_Counter=0;
	}
}
