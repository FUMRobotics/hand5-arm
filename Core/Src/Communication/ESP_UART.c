/*
 * ESP8266_UART.c
 *
 *  Created on: May 9, 2023
 *      Author: taji
 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ESP_UART.h"
#include "motor_Control.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t uartCounter = 0;
char uartRecieveBuffer[300]={0};
uint8_t RXuart=0;
UartTransmit_Enum TX_State;
send_Feedback_enum DataTypeFeedback;
_Bool send_data_UART=0;
volatile control_mode_Enum control_mode;
/* USER CODE END PV */


/*
 * Function1--------------------------
 */
void ProcessUartData(void)
{
	//remove noise in receive packet
	uint16_t counter=0;
	for(counter=uartCounter;counter>0;counter--)
	{
		if ((uartRecieveBuffer[counter] == '{'&&uartRecieveBuffer[counter+2] == 'P'&& uartRecieveBuffer[counter+3] == ':')&&(uartRecieveBuffer[counter+1] == 'P'||uartRecieveBuffer[counter+1] == 'S')) {
			for(uint16_t i=0 ; i<uartCounter ; i++)
			{
				uartRecieveBuffer[i]=uartRecieveBuffer[counter+i];
			}
			break;
		}
	}
	if(strstr(uartRecieveBuffer,"{SP:"))
	{
		control_mode=speed_mode;
	}
	else if(strstr(uartRecieveBuffer,"{PP:"))
	{
		control_mode=position_mode;
	}
	//parse receive data
	char* result;
	result= memchr(uartRecieveBuffer, ':', strlen(uartRecieveBuffer));
	Fingers_Status.Pinky.SetPoint = atof(result+1);
	if(Fingers_Status.Pinky.SetPoint>100)
		Fingers_Status.Pinky.SetPoint=100;
	result= memchr(uartRecieveBuffer, 'R', strlen(uartRecieveBuffer));
	Fingers_Status.Ring.SetPoint = atof(result+2);
	if(Fingers_Status.Ring.SetPoint>100)
		Fingers_Status.Ring.SetPoint=100;
	result= memchr(uartRecieveBuffer, 'M', strlen(uartRecieveBuffer));
	Fingers_Status.Middle.SetPoint= atof(result+2);
	if(Fingers_Status.Middle.SetPoint>100)
		Fingers_Status.Middle.SetPoint=100;
	result= memchr(uartRecieveBuffer, 'I', strlen(uartRecieveBuffer));
	Fingers_Status.Index.SetPoint= atof(result+2);
	if(Fingers_Status.Index.SetPoint>100)
		Fingers_Status.Index.SetPoint=100;
	result= memchr(uartRecieveBuffer, 'T', strlen(uartRecieveBuffer));
	Fingers_Status.Thumb.SetPoint= atof(result+2);
	if(Fingers_Status.Thumb.SetPoint>100)
		Fingers_Status.Thumb.SetPoint=100;
	send_data_UART=1;
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}
/*
 * Function2--------------------------
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART4) {
		uartRecieveBuffer[uartCounter]=RXuart;
		if(uartCounter>1)
		{
			if (uartRecieveBuffer[uartCounter] == '\n'&&uartRecieveBuffer[uartCounter-1] == '\r'&& uartRecieveBuffer[uartCounter-2] == '}') {
				ProcessUartData();
				ManualControl=1;
				uartCounter=-1;

//				for(uint16_t cleanCounter=0;cleanCounter<500;cleanCounter++)
//					uartRecieveBuffer[cleanCounter]=0;
			}
		}
		uartCounter++;
		HAL_UART_Receive_IT(&huart4, &RXuart, 1);
	}
}
/*
 * Function2--------------------------
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		TX_State=idel;
	}
}
/* USER CODE END PV */
