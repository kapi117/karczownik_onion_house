/*
 * encoder.h
 *
 *  Created on: Nov 23, 2022
 *      Author: Hyperbook
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f3xx.h"

typedef struct {
	TIM_HandleTypeDef *htim;
	uint16_t last_value;
} Encoder;

void ENCODER_init(Encoder* encoder, TIM_HandleTypeDef* htim);
uint16_t ENCODER_get_value(Encoder* encoder);
int8_t ENCODER_get_change(Encoder* encoder);

#endif /* INC_ENCODER_H_ */
