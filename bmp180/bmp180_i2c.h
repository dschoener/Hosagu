//////////////////////////////////////////////////////////////////////////
/*
 * Copyright (c) 2017
 *
 *  Created on: 26.05.2017
 *      Author: denis
 */
//////////////////////////////////////////////////////////////////////////

#ifndef BMP180_BMP180_I2C_H_
#define BMP180_BMP180_I2C_H_

#include <esp_common.h>

/**
 * BMP180 result data.
 */
struct Bmp180Data
{
	uint16 temperature; ///< The temperature in steps of 0.1 deg Celsius.
	uint32 pressure;    ///< The value of pressure in steps of 1.0 Pa.
};

/* This function is an example for reading sensor data
 *	\param result Pointer to the structure which receives the resulting data.
 *	\return 0 if the read out succeeded, -1 otherwise.
 */
uint8 bmp180_data_readout(struct Bmp180Data* result);

#endif // BMP180_BMP180_I2C_H_
