/*
 * servo.h
 *
 *  Created on: Nov 12, 2022
 *      Author: Hyperbook
 */

/**
 * USTAWIENIA W KONFIGURACJI SPRZĘTOWEJ:
 * - Timers > TIM > PWM Generation Output
 * - Prescaler [CLK częstotliwość - 1]
 * - Counter Period [20000-1]
 * - Pulse [1000]
 *
 * USAGE:
 *	Servo servo;
 *	servo_init(&servo, htim, TIM_CHANNEL_1);
 *	servo_set_angle(&servo, 60);
 *
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f4xx_hal.h"

#define SERVO_MAX_US		2500 //
#define SERVO_MIN_US		700 // 5%

#define SERVO_MAX_ANGLE		180
#define SERVO_MIN_ANGLE		0

typedef struct {
	TIM_HandleTypeDef *_pwm_tim;
	uint32_t _pwm_channel;
}Servo;

void servo_init(Servo* servo, TIM_HandleTypeDef *, uint32_t);
uint32_t servo_set_angle(Servo* servo, uint8_t);

#endif /* INC_SERVO_H_ */
