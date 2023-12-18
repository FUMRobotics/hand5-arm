/*
 * motor_Control.h
 *
 *  Created on: Sep 14, 2023
 *      Author: taji
 */

#ifndef SRC_MOTOR_CONTROL_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_MOTOR_CONTROL_H_
#include "main.h"

//---------------- Defines ---------------------
#define Max_Encoder_Index               27300
#define Max_Encoder_Middle              28700
#define Max_Encoder_Ring                27600
#define Max_Encoder_Pinky               26100
#define Max_Encoder_Thumb               1	 //must be measured after mechanical part fixed
//-------------- enumeration -------------------
typedef enum
{
	Stop,
	Open,
	Close
}Move_Direction_Enum;
typedef enum
{
	Alow_Blow=1,
	Alow_Bhigh,
	Ahigh_Bhigh,
	Ahigh_Blow,
}Encoder_State_Enum;
typedef enum
{
	Thumb,
	Index,
	Middle,
	Ring,
	Pinky
}Fingers_Name_Enum;

//-------------- structure -------------------
typedef struct
{
	float SetPoint;
	float position;
	uint16_t Encoder;
	Move_Direction_Enum Direction;
	Encoder_State_Enum current_Encoder_State;
	Encoder_State_Enum Pre_Encoder_State;
	_Bool Stuck_Finger;
	uint8_t speed;
	uint16_t Current;
	volatile _Bool SignalA;
	volatile _Bool SignalB;
}Finger_Struct;
typedef struct
{
	Finger_Struct Thumb;
	Finger_Struct Index;
	Finger_Struct Middle;
	Finger_Struct Ring;
	Finger_Struct Pinky;
}Fingers_Struct;
//-------------- EXTERN variable -------------------
extern Fingers_Struct Fingers_Status;
extern uint8_t ManualControl;
extern uint32_t Current_motor[6];
//-------------- function prototype -------------------
void Read_Encoder (Finger_Struct* FingerStruct,Fingers_Name_Enum FingerName);
void SetMotor(Fingers_Name_Enum name,Finger_Struct*  FingerStruct);
void init_motor_controller(void);
#endif /* SRC_MOTOR_CONTROL_MOTOR_CONTROL_H_ */
