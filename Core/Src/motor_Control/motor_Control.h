/*
 * motor_Control.h
 *
 *  Created on: Sep 14, 2023
 *      Author: Lenovo
 */

#ifndef SRC_MOTOR_CONTROL_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_MOTOR_CONTROL_H_
#include "main.h"

//-------------- enumeration -------------------
typedef enum
{
	Stop,
	Open,
	Close
}Move_Direction_Enum;
typedef enum
{
	Ahigh_Bhigh=1,
	Ahigh_Blow,
	Alow_Bhigh,
	Alow_Blow,
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
	uint8_t SetPoint;
	uint8_t Encoder;
	Move_Direction_Enum Direction;
	Encoder_State_Enum current_Encoder_State;
	Encoder_State_Enum Pre_Encoder_State;
	_Bool Statck_Finger;
	uint8_t speed;
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
//-------------- function prototype -------------------
void Read_Encoder (Finger_Struct* FingerStruct,Fingers_Name_Enum FingerName);
void SetMotor(Fingers_Name_Enum name,Finger_Struct  FingerStruct);
void init_motor_controller(void);
#endif /* SRC_MOTOR_CONTROL_MOTOR_CONTROL_H_ */
