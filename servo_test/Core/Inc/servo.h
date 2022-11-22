/*
 * servo.h
 *
 *  Created on: Nov 12, 2022
 *      Author: Hyperbook
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f4xx_hal.h"

#define SERVO_MAX_US		2660 //
#define SERVO_MIN_US		700 // 5%

#define SERVO_MAX_ANGLE		180
#define SERVO_MIN_ANGLE		0

void servo_init(TIM_HandleTypeDef *, uint32_t);
uint32_t servo_set_angle(uint8_t);

#endif /* INC_SERVO_H_ */
