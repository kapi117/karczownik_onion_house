/*
 * motor.h
 *
 *  Created on: Nov 17, 2022
 *      Author: Hyperbook
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
