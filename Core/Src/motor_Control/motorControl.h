/*
 * motorControl.h
 *
 *  Created on: May 9, 2023
 *      Author: NEW
 */

#ifndef SRC_MOTOR_CONTROL_MOTORCONTROL_H_
#define SRC_MOTOR_CONTROL_MOTORCONTROL_H_

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
/* USER CODE END PD */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef struct {
	float FeedBack;    	// Feedback
	float SetPoint;    	// SetPoint
	float Error;		// Calculated Error
	float DTerm;		// Calculated DTerm
	float ITerm;		// Calculated ITerm
	float I_Max;		//Maximum value of I Term - User parameter
	float I_Min;		//Minimum value of I term - User parameter
	float Out;			//Final PID Output
	float Out_Max;		//Maximum value of Output - User parameter
	float Out_Min;		//Minimum value of Output - User parameter
	float KP;			//Coefficient of P Term - User parameter
	float KI;			//Coefficient of I Term - User parameter
	float KD;			//Coefficient of D Term - User parameter
	float LoopPeriod;    //time of PID Control Loop - Not used in here
} PID;
typedef enum
{
  noValue,
  Thumb,
  Index,
  Middel,
  Ring,
  Pinky
} HandFinger_Typedef;
typedef enum
{
  STOP,
  OPEN,
  CLOSE
} HandStatuse_Typedef;
typedef struct
{
  HandStatuse_Typedef HandStatuse;
  int Value;
} HandState_Typedef;
/* USER CODE END EM */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE START PFP */
// - Position PIDs
extern PID Thumb_Position_PID;
extern PID Index_Position_PID;
extern PID Middle_Position_PID;
extern PID Ring_Position_PID;
extern PID Pinky_Position_PID;
 // = Current PIDs
extern PID Thumb_Current_PID;
extern PID Index_Current_PID;
extern PID Middle_Current_PID;
extern PID Ring_Current_PID;
extern PID Pinky_Current_PID;

void init_motor_controller(void);
void SetMotor(uint8_t motor_num, uint8_t motor_dir, uint8_t motor_speed);
void PID_Position_Defaults(PID *pid);
void ApplyPIDToMotor(PID *pid, uint8_t DesiredMotor);
void ComputePID(PID *PIDSystem);
void ADC_Select_CH0(void);
void ADC_Select_CH1(void);
void ADC_Select_CH2(void);
void ADC_Select_CH3(void);
void ADC_Select_CH4(void);

/* USER CODE END PFP */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE START PFP */
extern uint8_t dummy;
 //uart buffer
extern char uartRecieveBuffer[300];
extern uint16_t uartCounter;
extern uint8_t RXuart;
extern HandState_Typedef HandStruct[5];
extern uint16_t DATA[5] ;
extern _Bool SaveData_EEprom;

extern uint8_t Thumb_ManualControl ;
extern uint8_t Index_ManualControl ;
extern uint8_t Middle_ManualControl;
extern uint8_t Ring_ManualControl ;
extern uint8_t Pinky_ManualControl;
extern uint8_t GestureManualControl;
extern uint8_t ManualControlActive ;
extern uint8_t ManualControlPWM ;

extern uint16_t Thumb_Position ;
extern uint16_t Index_Position ;
extern uint16_t Middle_Position;
extern uint16_t Ring_Position ;
extern uint16_t Pinky_Position;

extern uint16_t Thumb_Position_SetPoint ;
extern uint16_t Index_Position_SetPoint ;
extern uint16_t Middle_Position_SetPoint;
extern uint16_t Ring_Position_SetPoint ;
extern uint16_t Pinky_Position_SetPoint;

extern uint8_t Thumb_LastDirection ;
extern uint8_t Index_LastDirection ;
extern uint8_t Middle_LastDirection;
extern uint8_t Ring_LastDirection ;
extern uint8_t Pinky_LastDirection;
extern float CRNT, SPD;
/* USER CODE END PFP */
#endif /* SRC_MOTOR_CONTROL_MOTORCONTROL_H_ */
