/*
 * terminal.c
 *
 *  Created on: Jan 19, 2025
 *      Author: jotto
 */

#include "terminal.h"

// Variables
UART_HandleTypeDef terminal_uart;

// Structures

// Functions
void terminal_init(UART_HandleTypeDef huart){
	terminal_uart = huart;
}
void terminal_receive(uint8_t *rxBuff, size_t size){
	// receive loop
	for(int i=0; i<size; i++){
	  HAL_UART_Receive(&terminal_uart, &rxBuff[i], 1, TIMEOUT);
	  if((rxBuff[i] == '\r') | (rxBuff[i] == '\n')){
		  rxBuff[i] = '\r';
		  rxBuff[i+1] = '\n';
		  break;
	  }
	}
}
void terminal_print(char *txBuff){
	// Has to be null terminated!
	size_t len = strlen((char*)txBuff);
	HAL_UART_Transmit(&terminal_uart, txBuff, len, TIMEOUT);
}
void terminal_clearBuff(char *buff){
	size_t len = strlen(buff);
	for(int i=0; i<len; i++){
		buff[i] = '\0';
	}
}
