/*
 * servo.c
 *
 *  Created on: Nov 12, 2022
 *      Author: Hyperbook
 */

#include "servo.h"

TIM_HandleTypeDef *pwm_tim;
uint32_t pwm_channel;

void servo_init(TIM_HandleTypeDef *tim, uint32_t channel)
{
	pwm_tim = tim;
	pwm_channel = channel;

	__HAL_TIM_SET_COMPARE(pwm_tim, pwm_channel, SERVO_MAX_US);
	HAL_TIM_PWM_Start(pwm_tim, pwm_channel);
}

uint32_t servo_set_angle(uint8_t angle)
{
	if(angle < SERVO_MIN_ANGLE)
		angle = SERVO_MIN_ANGLE;
	else if(angle > SERVO_MAX_ANGLE)
		angle = SERVO_MAX_ANGLE;

	uint32_t pwm_duty_us;

	pwm_duty_us = SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US))/SERVO_MAX_ANGLE;

	__HAL_TIM_SET_COMPARE(pwm_tim, pwm_channel, pwm_duty_us);

	return pwm_duty_us;
}

