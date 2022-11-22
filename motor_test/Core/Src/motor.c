/*
 * motor.c
 *
 *  Created on: Nov 17, 2022
 *      Author: Hyperbook
 */

#include "motor.h"

HAL_StatusTypeDef motor_init(Motor* motor, TIM_HandleTypeDef* pwm_tim, uint32_t channel_A, uint32_t channel_B){
	motor->channel_A = channel_A;
	motor->channel_B = channel_B;
	motor->pwm_tim = pwm_tim;
	HAL_StatusTypeDef status = HAL_TIM_PWM_Start(pwm_tim, channel_A);
	HAL_TIM_PWM_Start(pwm_tim, channel_B);
	return status;
}

uint32_t motor_run(Motor* motor, int8_t speed){
	// speed <-100; 100>
	uint32_t duty;
	speed = speed > 100 ? 100 : (speed < -100 ? -100 : speed);
	if (speed >= 0){
		duty = speed * CLOCK_COUNTER_PERIOD / 100;
		__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel_B, 0);
		__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel_A, duty);
	} else {
		duty = (-speed) * CLOCK_COUNTER_PERIOD / 100;
		__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel_A, 0);
		__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel_B, duty);
	}
	return duty;
}

void motor_brake(Motor* motor){
	__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel_A, 0);
	__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel_B, 0);
}

