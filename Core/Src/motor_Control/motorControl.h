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

//this values is approximate
#define FINGER_INDEX_TRAVEL   7100
#define FINGER_MIDDEL_TRAVEL  7100
#define FINGER_RING_TRAVEL    6874
#define FINGER_PINKY_TRAVEL   6590

#define Max_Current_ADC       3500
#define MIN_Current_ADC       800
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
  Thumb= Thumb_Motor ,
  Index= Index_Motor ,
  Middel= Middle_Motor,
  Ring= Ring_Motor,
  Pinky= Pinky_Motor
} HandFinger_Typedef;
typedef struct
{
  uint8_t Value;
} HandState_Typedef;

//typedef enum
//{
//	stop,
//	open,
//	close
//}FingerOrder_Typedef;
//typedef struct
//{
//	FingerOrder_Typedef Pinky;
//	FingerOrder_Typedef Ring;
//	FingerOrder_Typedef Middle;
//	FingerOrder_Typedef Index;
//	FingerOrder_Typedef Thumb;
//}FingersOrder_Typedef;
/* USER CODE END EM */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE START PFP */

void init_motor_controller(void);
void SetMotor(uint8_t motor_num, uint8_t motor_dir, uint8_t motor_speed);
void PID_Position_Defaults(PID *pid);
uint8_t ApplyPIDToMotor(uint8_t DesiredMotor);
void ComputePID(PID *PIDSystem);
void ADC_Select_CH0(void);
void ADC_Select_CH1(void);
void ADC_Select_CH2(void);
void ADC_Select_CH3(void);
void ADC_Select_CH4(void);

void Calculate_ADC_Current(void);
/* USER CODE END PFP */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE START PFP */
extern uint8_t dummy;
 //uart buffer
extern char uartRecieveBuffer[150];
extern uint16_t uartCounter;
extern uint8_t RXuart;
extern volatile HandState_Typedef HandStruct[5];
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

extern volatile uint16_t Thumb_Position ;
extern volatile uint16_t Index_Position ;
extern volatile uint16_t Middle_Position;
extern volatile uint16_t Ring_Position ;
extern volatile uint16_t Pinky_Position;
extern volatile uint16_t Last_Thumb_Position;
extern volatile uint16_t Last_Index_Position;
extern volatile uint16_t Last_Middle_Position;
extern volatile uint16_t Last_Ring_Position;
extern volatile uint16_t Last_Pinky_Position;

extern volatile uint32_t Check_Position_Counter;
extern float Index_PercentPosition;
extern float Middle_PercentPosition;
extern float Ring_PercentPosition;
extern float Pinky_PercentPosition;

extern uint16_t Thumb_Position_Stack ;
extern uint16_t Index_Position_Stack ;
extern uint16_t Middle_Position_Stack;
extern uint16_t Ring_Position_Stack ;
extern uint16_t Pinky_Position_Stack;

extern volatile uint8_t Thumb_LastDirection ;
extern volatile uint8_t Index_LastDirection ;
extern volatile uint8_t Middle_LastDirection;
extern volatile uint8_t Ring_LastDirection ;
extern volatile uint8_t Pinky_LastDirection;
extern float CRNT, SPD;
/* USER CODE END PFP */
#endif /* SRC_MOTOR_CONTROL_MOTORCONTROL_H_ */
