/*
 * motorControl.h
 *
 *  Created on: Mar 14, 2025
 *      Author: jotto
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#include "main.h"

void motor_initEncoder(TIM_HandleTypeDef timmerHandler);
void motor_initPWM(TIM_HandleTypeDef timmerHandler);
void motor_PWMSetForward(int x);
void motor_PWMSetBackward(int x);
int motor_getCount(void);

#endif /* INC_MOTORCONTROL_H_ */
