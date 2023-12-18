/*
 * ESP8266_UART.h
 *
 *  Created on: May 9, 2023
 *      Author: taji
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
typedef enum
{
	speed_mode,
	position_mode
}control_mode_Enum;
//extern variable************************************
extern char uartRecieveBuffer[150];
extern uint8_t RXuart;
extern UartTransmit_Enum TX_State;
extern send_Feedback_enum DataTypeFeedback;
extern _Bool send_data_UART;
extern control_mode_Enum control_mode;
//prototype functions************************************
void calculate_Position_Fingers(void);
void Fetch_Position_Fingers(void);

#endif /* SRC_COMMUNICATION_ESP_UART_H_ */
