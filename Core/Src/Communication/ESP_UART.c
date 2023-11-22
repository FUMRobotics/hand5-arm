/*
 * ESP8266_UART.c
 *
 *  Created on: May 9, 2023
 *      Author: NEW
 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ESP_UART.h>
#include "main.h"
#include "stm32f1xx.h"
#include "string.h"
#include "motor_Control.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint16_t uartCounter = 0;
char uartRecieveBuffer[150];
uint8_t RXuart=0;
UartTransmit_Enum TX_State;
send_Feedback_enum DataTypeFeedback;
/* USER CODE END PV */


/*
 * Function1--------------------------
 */
void ProcessUartData(void)
{
	char* result;
	result= memchr(uartRecieveBuffer, 'P', strlen(uartRecieveBuffer));
	Fingers_Status.Pinky.SetPoint = atof(result+2);
	result= memchr(uartRecieveBuffer, 'R', strlen(uartRecieveBuffer));
	Fingers_Status.Ring.SetPoint = atof(result+2);
	result= memchr(uartRecieveBuffer, 'M', strlen(uartRecieveBuffer));
	Fingers_Status.Middle.SetPoint= atof(result+2);
	result= memchr(uartRecieveBuffer, 'I', strlen(uartRecieveBuffer));
	Fingers_Status.Index.SetPoint= atof(result+2);
	result= memchr(uartRecieveBuffer, 'T', strlen(uartRecieveBuffer));
	Fingers_Status.Thumb.SetPoint= atof(result+2);
}
/*
 * Function2--------------------------
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		uartRecieveBuffer[uartCounter]=RXuart;
		if (uartRecieveBuffer[uartCounter] == '\n'&&uartRecieveBuffer[uartCounter-1] == '\r'&& uartRecieveBuffer[uartCounter-2] == '}') {
			ProcessUartData();
			ManualControl=1;
			uartCounter=-1;

			for(uint16_t cleanCounter=0;cleanCounter<150;cleanCounter++)
				uartRecieveBuffer[cleanCounter]=0;
		}
		uartCounter++;
		HAL_UART_Receive_IT(&huart1, &RXuart, 1);
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
