//////////////////////////////////////////////////////////////////////////
/*
 * Copyright (c) 2017
 *
 *  Created on: 02.05.2017
 *      Author: denis
 */
//////////////////////////////////////////////////////////////////////////

#include <i2c_common.h>

#include <utils/logger.h>

bool i2c_preamble_write(uint8 addr, uint8 reg)
{
	// Write SAD + W
	i2c_master_start();
	const uint8 _addr = RW_ADDR(addr, Write);
	i2c_master_writeByte(_addr);

	if (!i2c_master_checkAck())
	{
		log_error("missing slave ack (SAD+W)");
		i2c_master_stop();
		return false;
	}

	// Write SUB
	i2c_master_writeByte(reg);

	if (!i2c_master_checkAck())
	{
		log_error("missing slave ack (REG)");
		i2c_master_stop();
		return false;
	}

	return true;
}
