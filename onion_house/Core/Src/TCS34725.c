/*
 * TCS34725.c
 *
 *  Created on: Nov 12, 2022
 *      Author: Hyperbook
 */

#include "TCS34725.h"


void TCS34725_init(ColorSensor *color_sensor, I2C_HandleTypeDef *hi2c){
	color_sensor->_hi2c = hi2c;
	while(HAL_I2C_IsDeviceReady(hi2c, 0x29 << 1, 10, HAL_MAX_DELAY)){
		  HAL_Delay(5);
	}
	// SET INTEGRETION TIME
	uint8_t data = TCS34725_INTEGRATIONTIME_50MS;
	writeReg(color_sensor, TCS34725_ATIME, data);

	// GAIN
	data = TCS34725_GAIN_4X;
	writeReg(color_sensor, TCS34725_CONTROL, data);

	// ENABLE
	data = TCS34725_ENABLE_PON;
	writeReg(color_sensor, TCS34725_ENABLE, data);
	data = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
	HAL_Delay(5);
	writeReg(color_sensor, TCS34725_ENABLE, data);
}

HAL_StatusTypeDef readReg(ColorSensor *color_sensor, uint8_t reg, uint8_t* value){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(color_sensor->_hi2c, (TCS34725_ADDRESS << 1)|0x01, reg | 0x80, 1, value, sizeof(*value), HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef readRegword(ColorSensor *color_sensor, uint8_t reg, uint16_t* value){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(color_sensor->_hi2c, (TCS34725_ADDRESS << 1)|0x01, reg | 0x80, 1, (uint8_t*) value, sizeof(*value), HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef writeReg(ColorSensor *color_sensor, uint8_t reg, uint8_t value){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(color_sensor->_hi2c, (TCS34725_ADDRESS << 1), reg | 0x80, 1, (uint8_t*)&value, sizeof(value), HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef get_red(ColorSensor *color_sensor, uint16_t* value){
	HAL_StatusTypeDef status = readRegword(color_sensor, TCS34725_RDATAL, value);
	return status;
}

HAL_StatusTypeDef get_green(ColorSensor *color_sensor, uint16_t* value){
	HAL_StatusTypeDef status = readRegword(color_sensor, TCS34725_GDATAL, value);
	return status;
}

HAL_StatusTypeDef get_blue(ColorSensor *color_sensor, uint16_t* value){
	HAL_StatusTypeDef status = readRegword(color_sensor, TCS34725_BDATAL, value);
	return status;
}

HAL_StatusTypeDef get_light(ColorSensor *color_sensor, uint16_t* value){
	HAL_StatusTypeDef status = readRegword(color_sensor, TCS34725_CDATAL, value);
	return status;
}

HAL_StatusTypeDef lock(ColorSensor *color_sensor){
	uint8_t r;
	HAL_StatusTypeDef status = 0;
	status |= readReg(color_sensor, TCS34725_ENABLE, &r);
	r |= TCS34725_ENABLE_AIEN;
	status |= writeReg(color_sensor, TCS34725_ENABLE, r);
	return status;
}

HAL_StatusTypeDef unlock(ColorSensor *color_sensor){
	uint8_t r;
	HAL_StatusTypeDef status = 0;
	status |= readReg(color_sensor, TCS34725_ENABLE, &r);
	r &= ~TCS34725_ENABLE_AIEN;
	status |= writeReg(color_sensor, TCS34725_ENABLE, r);
	return status;
}
