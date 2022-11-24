/*
 * servo.c
 *
 *  Created on: Nov 12, 2022
 *      Author: Hyperbook
 */

#include "servo.h"


void servo_init(Servo* servo, TIM_HandleTypeDef *tim, uint32_t channel)
{
	servo->_pwm_tim = tim;
	servo->_pwm_channel = channel;

	__HAL_TIM_SET_COMPARE(servo->_pwm_tim, servo->_pwm_channel, SERVO_MAX_US);
	HAL_TIM_PWM_Start(servo->_pwm_tim, servo->_pwm_channel);
}

uint32_t servo_set_angle(Servo* servo, uint8_t angle)
{
	if(angle < SERVO_MIN_ANGLE)
		angle = SERVO_MIN_ANGLE;
	else if(angle > SERVO_MAX_ANGLE)
		angle = SERVO_MAX_ANGLE;

	uint32_t pwm_duty_us;

	pwm_duty_us = SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US))/SERVO_MAX_ANGLE;

	__HAL_TIM_SET_COMPARE(servo->_pwm_tim, servo->_pwm_channel, pwm_duty_us);

	return pwm_duty_us;
}

