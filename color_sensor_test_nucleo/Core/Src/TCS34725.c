/*
 * TCS34725.c
 *
 *  Created on: Nov 12, 2022
 *      Author: Hyperbook
 */

#include "TCS34725.h"

I2C_HandleTypeDef* _hi2c;


void TCS34725_init(I2C_HandleTypeDef *hi2c){
	_hi2c = hi2c;
}

HAL_StatusTypeDef readReg(uint8_t reg, uint8_t* value){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_hi2c, (TCS34725_ADDRESS << 1)|0x01, reg | 0x80, 1, value, sizeof(*value), HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef readRegword(uint8_t reg, uint16_t* value){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_hi2c, (TCS34725_ADDRESS << 1)|0x01, reg | 0x80, 1, (uint8_t*) value, sizeof(*value), HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef writeReg(uint8_t reg, uint8_t value){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(_hi2c, (TCS34725_ADDRESS << 1), reg | 0x80, 1, (uint8_t*)&value, sizeof(value), HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef lock(){
	uint8_t r;
	HAL_StatusTypeDef status = 0;
	status |= readReg(TCS34725_ENABLE, &r);
	r |= TCS34725_ENABLE_AIEN;
	status |= writeReg(TCS34725_ENABLE, r);
	return status;
}

HAL_StatusTypeDef unlock(){
	uint8_t r;
	HAL_StatusTypeDef status = 0;
	status |= readReg(TCS34725_ENABLE, &r);
	r &= ~TCS34725_ENABLE_AIEN;
	status |= writeReg(TCS34725_ENABLE, r);
	return status;
}
