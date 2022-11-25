/*
 * ktir_sensor.h
 *
 *  Created on: Nov 23, 2022
 *      Author: Hyperbook
 */

/**
 * USTAWIENIA W KONFIGURACJI SPRZĘTOWEJ:
 * - Analog > ADC > IN Single Ended (x4)
 * - Number of conversions [4]
 * - Scan conversion mode [Enabled]
 * - dla każdego Rank odpowiedni Channel
 * - DMA Settings > ADD > ADC [Normal, Half word]
 *
 * USAGE:
 * 	volatile uint16_t ktir_results[NUMBER_OF_SENSORS];
 *	KTIR_Init(&hadc1, ktir_results);
 *	KTIR_read();
 *	printf("%d\n", ktir_reults[0]);
 */

#ifndef INC_KTIR_SENSOR_H_
#define INC_KTIR_SENSOR_H_

#include "main.h"

#define NUMBER_OF_SENSORS 4



void KTIR_Init(ADC_HandleTypeDef* hadc1, volatile uint16_t* ktir_results);
void KTIR_read();
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);


#endif /* INC_KTIR_SENSOR_H_ */
