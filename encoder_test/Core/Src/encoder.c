/*
 * encoder.c
 *
 *  Created on: Nov 23, 2022
 *      Author: Hyperbook
 */


#include "encoder.h"

void ENCODER_init(Encoder* encoder, TIM_HandleTypeDef* htim){
	encoder->htim = htim;
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}
uint16_t ENCODER_get_value(Encoder* encoder){
	encoder->last_value = encoder->htim->Instance->CNT;
	return encoder->last_value;
}
int8_t ENCODER_get_change(Encoder* encoder){
	int TimerDif = encoder->htim->Instance->CNT - encoder->last_value;
	int8_t change = 0;
	if(TimerDif >= 4 || TimerDif <= -4)
	{
		TimerDif /= 4;
		change += (int8_t)TimerDif;
	}
	encoder->last_value = encoder->htim->Instance->CNT;
	return change;
}
