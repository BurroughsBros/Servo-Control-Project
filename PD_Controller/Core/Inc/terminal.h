/*
 * terminal.h
 *
 *  Created on: Jan 19, 2025
 *      Author: jotto
 */

#ifndef INC_TERMINAL_H_
#define INC_TERMINAL_H_

// Included libraries
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Define constants
#define BUFF_SIZE	100				// Buffer Size
#define TIMEOUT		HAL_MAX_DELAY	// Wait a long time

// Function Prototypes
void terminal_init(UART_HandleTypeDef huart);
void terminal_receive(uint8_t *rxBuff, size_t size);
void terminal_print(char *txBuff);
void terminal_print_IT(char *txBuff);
void terminal_clearBuff(char *buff);

#endif /* INC_TERMINAL_H_ */

