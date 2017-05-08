/*
 ****************************************************************************
 * Copyright (C) 2014 - 2015 Bosch Sensortec GmbH
 *
 * bmp180_support.c
 * Date: 2015/07/16
 * Revision: 1.0.5 $
 *
 * Usage: Sensor Driver support file for BMP180
 *
 ****************************************************************************
 * License:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holder nor the names of the
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 **************************************************************************/
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "bmp180.h"
#include <i2c_master.h>
#include <i2c_common.h>
#include <logger.h>

/*----------------------------------------------------------------------------
 * The following functions are used for reading and writing of
 * sensor data using I2C or SPI communication
 ----------------------------------------------------------------------------*/
#ifdef BMP180_API
/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: I2C init routine
 */
s8 I2C_routine(void);
#endif
/********************End of I2C function declarations***********************/
/*	Brief : The delay routine
 *	\param : delay in ms
 */
void BMP180_delay_msek(u32 msek);
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bmp180_data_readout_template(void);
/*----------------------------------------------------------------------------
 struct bmp180_t parameters can be accessed by using bmp180
 *	bmp180_t having the following parameters
 *	Bus write function pointer: BMP180_WR_FUNC_PTR
 *	Bus read function pointer: BMP180_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *	Calibration parameters
 ---------------------------------------------------------------------------*/
struct bmp180_t bmp180;
/*----------------------------------------------------------------------------*/
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bmp180_data_readout_template(void)
{
	/* result of communication results*/
	s32 com_rslt = E_BMP_COMM_RES;
	u16 v_uncomp_temp_u16 = BMP180_INIT_VALUE;
	u32 v_uncomp_press_u32 = BMP180_INIT_VALUE;
	/*********************** START INITIALIZATION ************************/
	/**************Call the I2C init routine ***************/
#ifdef BMP180_API
	I2C_routine();
#endif
	/*--------------------------------------------------------------------------
	 *  This function used to assign the value/reference of
	 *	the following parameters
	 *	I2C address
	 *	Bus Write
	 *	Bus read
	 *	Chip id
	 *	Calibration values
	 -------------------------------------------------------------------------*/
	com_rslt = bmp180_init(&bmp180);
	// START CALIPRATION
	/*  This function used to read the calibration values of following
	 *	these values are used to calculate the true pressure and temperature
	 *	Parameter		MSB		LSB		bit
	 *		AC1			0xAA	0xAB	0 to 7
	 *		AC2			0xAC	0xAD	0 to 7
	 *		AC3			0xAE	0xAF	0 to 7
	 *		AC4			0xB0	0xB1	0 to 7
	 *		AC5			0xB2	0xB3	0 to 7
	 *		AC6			0xB4	0xB5	0 to 7
	 *		B1			0xB6	0xB7	0 to 7
	 *		B2			0xB8	0xB9	0 to 7
	 *		MB			0xBA	0xBB	0 to 7
	 *		MC			0xBC	0xBD	0 to 7
	 *		MD			0xBE	0xBF	0 to 7*/
	com_rslt += bmp180_get_calib_param();

	// START READ UNCOMPENSATED TEMPERATURE AND PRESSURE
	v_uncomp_temp_u16 = bmp180_get_uncomp_temperature();
	v_uncomp_press_u32 = bmp180_get_uncomp_pressure();

	// START READ TRUE TEMPERATURE AND PRESSURE
	com_rslt += bmp180_get_temperature(v_uncomp_temp_u16);
	com_rslt += bmp180_get_pressure(v_uncomp_press_u32);

	return com_rslt;
}

#ifdef BMP180_API
/*--------------------------------------------------------------------------*
 *	The following function is used to map the I2C bus read, write, delay and
 *	device address with global structure bmp180_t
 *-------------------------------------------------------------------------*/
s8 I2C_routine(void)
{
	/*--------------------------------------------------------------------------*
	 *  By using bmp180 the following structure parameter can be accessed
	 *	Bus write function pointer: BMP180_WR_FUNC_PTR
	 *	Bus read function pointer: BMP180_RD_FUNC_PTR
	 *	Delay function pointer: delay_msec
	 *	I2C address: dev_addr
	 *--------------------------------------------------------------------------*/
	bmp180.bus_write = BMP180_I2C_bus_write;
	bmp180.bus_read = BMP180_I2C_bus_read;
	bmp180.dev_addr = BMP180_I2C_ADDR;
	bmp180.delay_msec = BMP180_delay_msek;

	return BMP180_INIT_VALUE;
}

/************** I2C buffer length ******/

#define	I2C_BUFFER_LEN 8
#define I2C0 5
/*-------------------------------------------------------------------*
 *	This is a sample code for read and write the data by using I2C
 *	Configure the below code to your I2C driver
 *	The device address is defined in the bmp180.c file
 *-----------------------------------------------------------------------*/
/*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	for (; cnt > 0; cnt--)
	{
		const bool success = i2c_preamble_write(dev_addr, reg);
		if (!success)
		{
			return E_BMP_COMM_RES;;
		}

		// Write DATA
		i2c_master_writeByte(*reg_data);

		if (!i2c_master_checkAck())
		{
			log_error("missing slave ack (DATA)");
			i2c_master_stop();
			return E_BMP_COMM_RES;;
		}

		reg_data++;
		i2c_master_stop();
	}
	return (s8)BMP180_INIT_VALUE;
}

/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	const bool success = i2c_preamble_write(dev_addr, reg_addr);
	if (!success)
	{
		return E_BMP_COMM_RES;
	}

	// Start reading
	i2c_master_start();
	const u8 addr = RW_ADDR(dev_addr, Read);
	i2c_master_writeByte(addr);

	if (!i2c_master_checkAck())
	{
		log_error("missing slave ack (SAD+R)");
		i2c_master_stop();
		return E_BMP_COMM_RES;
	}

	// Read data sequence
	while (cnt > 0)
	{
		*reg_data = i2c_master_readByte();
		reg_data++;
		cnt--;
		if (cnt > 0)
		{
			i2c_master_send_ack();
		}
		else
		{
			i2c_master_send_nack();
		}
	}

	i2c_master_stop();
	return (s8)BMP180_INIT_VALUE;
}
/*	Brief : The delay routine
 *	\param : delay in ms
 */
void BMP180_delay_msek(u32 msek)
{
	i2c_master_wait(1000 * msek);
}

#endif
