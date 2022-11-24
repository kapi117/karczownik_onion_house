/*
 * motor.h
 *
 *  Created on: Nov 17, 2022
 *      Author: Hyperbook
 */

/**
 * USTAWIENIA W KONFIGURACJI SPRZĘTOWEJ:
 * - TIMERS > PWM Generation Output (x4)
 * - Prescaler [częstotliwość CLK / 10 - 1]
 * - Counter [1000 - 1]
 *
 * USAGE:
 *
	#define LEFT_MOTOR_CHANNEL_A TIM_CHANNEL_4
	#define LEFT_MOTOR_CHANNEL_B TIM_CHANNEL_3
	Motor left_motor;
	motor_init(&left_motor, &htim1, LEFT_MOTOR_CHANNEL_A, LEFT_MOTOR_CHANNEL_B);
	motor_run(&left_motor, 10);
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define CLOCK_COUNTER_PERIOD 999

#include "stm32f4xx_hal.h"

typedef struct {
	uint32_t channel_A, channel_B;
	TIM_HandleTypeDef *pwm_tim;
} Motor;

HAL_StatusTypeDef motor_init(Motor*, TIM_HandleTypeDef*, uint32_t, uint32_t);
uint32_t motor_run(Motor*, int8_t);
void motor_brake(Motor*);



#endif /* INC_MOTOR_H_ */
