//////////////////////////////////////////////////////////////////////////
/*
 * Copyright (c) 2017
 *
 *  Created on: 02.05.2017
 *      Author: denis
 */
//////////////////////////////////////////////////////////////////////////

#ifndef DRIVER_INCLUDE_I2C_COMMON_H_
#define DRIVER_INCLUDE_I2C_COMMON_H_

#include <esp_common.h>

typedef enum
{
	Write = 0,
	Read = 1
} I2C_DataType;

#define RW_ADDR(ADDR, TYPE) (((ADDR) << 1)  | ((TYPE == Read) ? 0x01 : 0x00))

bool i2c_preamble_write(uint8 addr, uint8 reg);

#endif // DRIVER_INCLUDE_I2C_COMMON_H_
