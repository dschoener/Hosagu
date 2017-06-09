//////////////////////////////////////////////////////////////////////////
/*
 * Copyright (c) 2017
 *
 *  Created on: 20.04.2017
 *      Author: denis
 */
//////////////////////////////////////////////////////////////////////////

#include "lis331hh.h"

#include <driver/i2c_common.h>
#include <utils/logger.h>

bool lis331_write_register(uint8 reg, uint8 value)
{
	if ((reg < CTRL_REG1) || (reg > CTRL_REG5))
	{
		log_error("invalid register address: %d", reg);
		return false;
	}

	const bool success = i2c_preamble_write(LIS331_ADDR, reg);
	if (!success)
	{
		return false;
	}

	// Write DATA
	i2c_master_writeByte(value);

	if (!i2c_master_checkAck())
	{
		log_error("missing slave ack (DATA)");
		i2c_master_stop();
		return false;
	}

	i2c_master_stop();
	return true;
}

bool lis331_read_status(uint8 * status)
{
	const bool success = i2c_preamble_write(LIS331_ADDR, ((~SUB_AUTO_INC) & STATUS_REG));

	if (!success)
	{
		return false;
	}

	// Start reading
	i2c_master_start();
	const uint8 addr = RW_ADDR(LIS331_ADDR, Read);
	i2c_master_writeByte(addr);

	if (!i2c_master_checkAck())
	{
		log_error("missing slave ack (SAD+R)");
		i2c_master_stop();
		return false;
	}

	*status = i2c_master_readByte();
	i2c_master_send_nack();
	i2c_master_stop();

	return true;
}

bool lis331_read_data(struct AccelData* data)
{
	const bool success = i2c_preamble_write(LIS331_ADDR, (SUB_AUTO_INC | OUT_X_L));

	if (!success)
	{
		return false;
	}

	// Start reading
	i2c_master_start();
	const uint8 addr = RW_ADDR(LIS331_ADDR, Read);
	i2c_master_writeByte(addr);

	if (!i2c_master_checkAck())
	{
		log_error("missing slave ack (SAD+R)");
		i2c_master_stop();
		return false;
	}

	// read x axis
	uint16 low = i2c_master_readByte();
	i2c_master_send_ack();
	uint16 high = i2c_master_readByte();
	i2c_master_send_ack();
	data->xAccel = (sint16)((high << 12) | (low << 4));

	// read y axis
	low = i2c_master_readByte();
	i2c_master_send_ack();
	high = i2c_master_readByte();
	i2c_master_send_ack();
	data->yAccel = (sint16)((high << 12) | (low << 4));

	// read z axis
	low = i2c_master_readByte();
	i2c_master_send_ack();
	high = i2c_master_readByte();
	i2c_master_send_nack();
	data->zAccel = (sint16)((high << 12) | (low << 4));

	i2c_master_stop();

	return true;
}

bool lis331_init()
{
	const bool res = lis331_write_register(CTRL_REG1,
	CTRL_REG1_PM_NORM | CTRL_REG1_ODR_100 |
	CTRL_REG1_ZEN | CTRL_REG1_YEN | CTRL_REG1_XEN)
			&& lis331_write_register(CTRL_REG2, 0x00) && lis331_write_register(CTRL_REG3, 0x00)
			&& lis331_write_register(CTRL_REG4, CTRL_REG4_BDU_BLCK | CTRL_REG4_FS_6G)
			&& lis331_write_register(CTRL_REG5, 0x00);

	return res;
}
