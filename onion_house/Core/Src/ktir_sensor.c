/*
 * ktir_sensor.c
 *
 *  Created on: Nov 23, 2022
 *      Author: Hyperbook
 */

#include "ktir_sensor.h"

volatile uint8_t conversion_complete = 0;
ADC_HandleTypeDef* _hadc1;
volatile uint16_t* _ktir_results;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	conversion_complete = 1;
}

void KTIR_Init(ADC_HandleTypeDef* hadc1, volatile uint16_t* ktir_results){
	_hadc1 = hadc1;
	_ktir_results = ktir_results;
}

void KTIR_read(){
	HAL_ADC_Start_DMA(_hadc1, (uint32_t*) _ktir_results, NUMBER_OF_SENSORS);
}
