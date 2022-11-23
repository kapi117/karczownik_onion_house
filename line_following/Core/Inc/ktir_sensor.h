/*
 * ktir_sensor.h
 *
 *  Created on: Nov 23, 2022
 *      Author: Hyperbook
 */

#ifndef INC_KTIR_SENSOR_H_
#define INC_KTIR_SENSOR_H_

#include "stm32f3xx.h"

#define NUMBER_OF_SENSORS 4

volatile uint16_t*	_ktir_results;
ADC_HandleTypeDef* _hadc1;

void KTIR_Init(ADC_HandleTypeDef* hadc1, volatile uint16_t* ktir_results);
void KTIR_read();
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);


#endif /* INC_KTIR_SENSOR_H_ */
