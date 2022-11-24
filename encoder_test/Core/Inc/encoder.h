/*
 * encoder.h
 *
 *  Created on: Nov 23, 2022
 *      Author: Hyperbook
 */

/**
 * USTAWIENIA W KONFIGURACJI SPRZĘTOWEJ:
 * - TIMERS > TIM > Combined Channels [Encoder Mode]
 * - auto reload preload [enable]
 * - Encoder mode [Encoder mode TI1 TI2]
 * - Input filter [10]
 * - Polarity [Falling edge]
 *
 * USAGE:
 * 	Encoder encoder;
 * 	ENCODER_init(&encoder, &htim);
 * 	uint16_t licz = ENCODER_get_value(&encoder);
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
