/*
 * motor_Control.h
 *
 *  Created on: Sep 14, 2023
 *      Author: taji
 */

#ifndef SRC_MOTOR_CONTROL_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_MOTOR_CONTROL_H_
#include "main.h"
#include "PID.h"
//---------------- Defines ---------------------
//---------------- Encoder ---------------------
#define Max_Encoder_Index               26500
#define Max_Encoder_Middle              25800
#define Max_Encoder_Ring                24400
#define Max_Encoder_Pinky               26100
#define Max_Encoder_Thumb               4647	 //(1 encoder pulse)
//------------- Stall Current -----------------(in 60% Speed)
#define Max_Current_Close_Index			1950
#define Max_Current_Close_Middle		1980
#define Max_Current_Close_Ring			1950
#define Max_Current_Close_Pinky			2000
#define Max_Current_Close_Thumb			1520	//must be measured after mechanical part fixed
#define Min_Current_Open_Index			1500
#define Min_Current_Open_Middle			1600
#define Min_Current_Open_Ring			1550
#define Min_Current_Open_Pinky			1600
#define Min_Current_Open_Thumb			1920	//must be measured after mechanical part fixed
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
	double SetPoint;
	double position;
	uint16_t Encoder;
	Move_Direction_Enum Direction_motor;
	Move_Direction_Enum Direction_Encoder;
	_Bool ChangeDirection;
	Encoder_State_Enum current_Encoder_State;
	Encoder_State_Enum Pre_Encoder_State;
	_Bool Stuck_Finger;
	double speed;
	uint16_t Current;
	volatile _Bool SignalA;
	volatile _Bool SignalB;
	volatile uint16_t Current_Counter;
	PID_TypeDef PID_Struct;
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
extern volatile uint16_t ADCData[6];
extern volatile uint16_t calibration_counter;
//-------------- function prototype -------------------
void Read_Encoder(Fingers_Name_Enum FingerName,Finger_Struct* FingerStruct);
void SetMotor(Fingers_Name_Enum name,Finger_Struct*  FingerStruct);
void init_motor_controller(void);
void Fingers_Calibration(void);
void Control_Motor(Fingers_Name_Enum FingerName,Finger_Struct* FingerStruct);
#endif /* SRC_MOTOR_CONTROL_MOTOR_CONTROL_H_ */
