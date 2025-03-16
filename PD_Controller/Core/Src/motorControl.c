/*
 * motorControl.c
 *
 *  Created on: Mar 14, 2025
 *      Author: jotto
 */

#include "motorControl.h"

// Variables
TIM_HandleTypeDef encoderHandle;
TIM_HandleTypeDef PWMHandle;
int timmerChannel1 = 0;
int timmerChannel2 = 0;

void motor_initEncoder(TIM_HandleTypeDef timmerHandler, uint32_t channel1, uint32_t channel2){
	encoderHandle = timmerHandler;
	HAL_TIM_Encoder_Start(&encoderHandle, channel1);
	HAL_TIM_Encoder_Start(&encoderHandle, channel2);
}
void motor_initPWM(TIM_HandleTypeDef timmerHandler, uint32_t channel1, uint32_t channel2){
	PWMHandle = timmerHandler;
	timmerChannel1 = channel1;
	timmerChannel2 = channel2;
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

