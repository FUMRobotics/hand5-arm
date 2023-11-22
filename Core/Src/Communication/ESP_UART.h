/*
 * ESP8266_UART.h
 *
 *  Created on: May 9, 2023
 *      Author: NEW
 */

#ifndef SRC_COMMUNICATION_ESP_UART_H_
#define SRC_COMMUNICATION_ESP_UART_H_

#include "main.h"


typedef enum
{
	busy,
	idel
}UartTransmit_Enum;
typedef enum
{
	current,
	position
}send_Feedback_enum;
//extern variable************************************
extern char uartRecieveBuffer[150];
extern uint8_t RXuart;
extern UartTransmit_Enum TX_State;
extern send_Feedback_enum DataTypeFeedback;
//prototype functions************************************
void calculate_Position_Fingers(void);
void Fetch_Position_Fingers(void);

#endif /* SRC_COMMUNICATION_ESP_UART_H_ */
