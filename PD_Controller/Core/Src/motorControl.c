/*
 * motorControl.c
 *
 *  Created on: Mar 14, 2025
 *      Author: jotto
 */

#include "motorControl.h"
#include "main.h"

TIM_HandleTypeDef encoderHandle;
TIM_HandleTypeDef PWMHandle;
int timmerChannel1 = 0;
int timmerChannel2 = 0;

void motor_initEncoder(TIM_HandleTypeDef timmerHandler){
	encoderHandle = timmerHandler;
	HAL_TIM_Encoder_Start(&encoderHandle, 1);
	HAL_TIM_Encoder_Start(&encoderHandle, 2);
}
void motor_initPWM(TIM_HandleTypeDef timmerHandler){
	PWMHandle = timmerHandler;
	timmerChannel1 = TIM_CHANNEL_1;
	timmerChannel2 = TIM_CHANNEL_2;
	HAL_TIM_PWM_Start(&PWMHandle, timmerChannel1);
	HAL_TIM_PWM_Start(&PWMHandle, timmerChannel2);
}
void motor_PWMSetForward(int x){
	__HAL_TIM_SET_COMPARE(&PWMHandle, timmerChannel1, x);
	__HAL_TIM_SET_COMPARE(&PWMHandle, timmerChannel2, 0);
}
void motor_PWMSetBackward(int x){
	__HAL_TIM_SET_COMPARE(&PWMHandle, timmerChannel1, 0);
	__HAL_TIM_SET_COMPARE(&PWMHandle, timmerChannel2, x);
}
int motor_getCount(void){
	return __HAL_TIM_GET_COUNTER(&encoderHandle);
}

