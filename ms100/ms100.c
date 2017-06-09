/*
 * ms100.c
 *
 *  Created on: Jun 9, 2017
 *      Author: denis
 */

#include "ms100.h"

uint32 ms100_read_mv()
{
	return ((((uint32)system_adc_read()) * 1000) / 1024);
}
