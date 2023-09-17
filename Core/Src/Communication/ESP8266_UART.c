/*
 * ESP8266_UART.c
 *
 *  Created on: May 9, 2023
 *      Author: NEW
 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32f1xx.h"
#include "ESP8266_UART.h"
#include "jsmn.h"
#include "string.h"
#include "motor_Control.h"
#include "math.h"
#include "core_cm3.h"
#include "usart.h"
/* USER CODE END Includes */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint16_t uartCounter = 0;
char uartRecieveBuffer[150];
uint8_t RXuart=0;
volatile uint8_t HandStruct[5]={100,};
/* USER CODE END PV */


/*
 * Function1--------------------------
 */
static int jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
	if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0)
	{
		return 0;
	}
	return -1;
}
/*
 * Function2--------------------------
 */
_Bool ProcessUartData(void)
{

	Fingers_Name_Enum HandFinger;
	int i;
	int r;
	jsmn_parser p;
	jsmntok_t t[128]; /* We expect no more than 128 tokens */

	jsmn_init(&p);
	r = jsmn_parse(&p, uartRecieveBuffer, strlen(uartRecieveBuffer), t,
			sizeof(t) / sizeof(t[0]));
	if (r < 0)
	{
		//Failed to parse JSON
		return 1;
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT)
	{
		//Object expected
		return 1;
	}

	/* Loop over all keys of the root object */
	for (i = 1; i < r; i++)
	{

		if (jsoneq(uartRecieveBuffer, &t[i], "Thumb") == 0)
		{
			HandFinger = Thumb;
			i++;
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Middele") == 0)
		{
			HandFinger = Middle;
			i++;
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Index") == 0)
		{
			HandFinger = Index;
			i++;
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Ring") == 0)
		{
			HandFinger = Ring;
			i++;
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Pinky") == 0)
		{
			HandFinger = Pinky;
			i++;
		}
		else if (jsoneq(uartRecieveBuffer, &t[i], "Value") == 0)
		{
			char FingerValue[100] = {0,};
			strncpy(FingerValue, uartRecieveBuffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);
			i++;
			HandStruct[HandFinger]=0;
			for (int counter = 0; counter < strlen(FingerValue); counter++)
			{
				switch (FingerValue[counter]) {
				case '0':
					HandStruct[HandFinger] += 0 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '1':
					HandStruct[HandFinger] += 1 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '2':
					HandStruct[HandFinger] += 2 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '3':
					HandStruct[HandFinger] += 3 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '4':
					HandStruct[HandFinger]+= 4 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '5':
					HandStruct[HandFinger]+= 5 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '6':
					HandStruct[HandFinger]+= 6 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '7':
					HandStruct[HandFinger]+= 7 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '8':
					HandStruct[HandFinger]+= 8 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				case '9':
					HandStruct[HandFinger]+= 9 * pow(10,(strlen(FingerValue) - (counter+1)));

					break;
				default:
					break;
				}
			}
		}
		else
		{
			//Unexpected key
		}
	}

	//store receive data in finger structure
	Fingers_Status.Thumb.SetPoint=HandStruct[Thumb];
	Fingers_Status.Index.SetPoint=HandStruct[Index];
	Fingers_Status.Middle.SetPoint=HandStruct[Middle];
	Fingers_Status.Ring.SetPoint=HandStruct[Ring];
	Fingers_Status.Pinky.SetPoint=HandStruct[Pinky];
	//just for test code
	/*
	for(int counter=0;counter<5;counter++)
	{
		printf("--------finger number : %d---------\n",counter);
		switch (HandStruct[counter].HandStatuse)
		{
		case CLOSE:
			printf("finger status : CLOSE\n");
			break;
		case STOP:
			printf("finger status : STOP\n");
			break;
		case OPEN:
			printf("finger status : OPEN\n");
			break;
		default:
			break;
		}
		if(HandStruct[counter].Value/100)
		{
			HandStruct[counter].Value++;
		}
		printf("finger value : %d\n",HandStruct[counter].Value);
	}
	*/
	return EXIT_SUCCESS;
}
/*
 * Function3--------------------------
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

/* USER CODE END PV */
