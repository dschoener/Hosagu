//////////////////////////////////////////////////////////////////////////
/*
 * Author: Denis Schoener
 *
 * Copyright (c) 2017
 *
 *
 */
//////////////////////////////////////////////////////////////////////////
#include <esp_common.h>
#include <gpio.h>
#include <uart.h>

#include <ms100/ms100.h>
#include "lis331hh/lis331hh.h"
#include "bmp180/bmp180_i2c.h"
#include "utils/logger.h"
#include "wifi/wifi_control.h"

static os_timer_t timer;

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
 *******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
	flash_size_map size_map = system_get_flash_size_map();
	uint32 rf_cal_sec = 0;

	switch (size_map)
	{
	case FLASH_SIZE_4M_MAP_256_256:
		rf_cal_sec = 128 - 5;
		break;

	case FLASH_SIZE_8M_MAP_512_512:
		rf_cal_sec = 256 - 5;
		break;

	case FLASH_SIZE_16M_MAP_512_512:
	case FLASH_SIZE_16M_MAP_1024_1024:
		rf_cal_sec = 512 - 5;
		break;

	case FLASH_SIZE_32M_MAP_512_512:
	case FLASH_SIZE_32M_MAP_1024_1024:
		rf_cal_sec = 1024 - 5;
		break;

	default:
		rf_cal_sec = 0;
		break;
	}

	return rf_cal_sec;
}

LOCAL void ICACHE_FLASH_ATTR on_trigger_received()
{
	static uint32 last_time_ms = 0;

	const uint32 now_ms = system_get_time() / (uint32)1000;

	if (last_time_ms == 0)
	{
		last_time_ms = now_ms;
	}

	const uint32 duration_ms = now_ms - last_time_ms;

	if (duration_ms < 500)
	{
		GPIO_OUTPUT_SET(5, 1);
		//log_info("ADC: %d", system_adc_read());
	}
	else if (duration_ms < 1000)
	{
		GPIO_OUTPUT_SET(5, 0);
	}
	else
	{
		last_time_ms = now_ms;
	}

	struct Bmp180Data bmp180Data = { 0, 0 };
	if (bmp180_data_readout(&bmp180Data) == 0)
	{
		log_info("BMP180: %04d.0 Pa", bmp180Data.pressure);
		log_info("BMP180: %04d.%01d° C", (bmp180Data.temperature / 10), (bmp180Data.temperature % 10));
	}

	const uint32 mili_volt = ((((uint32)system_adc_read()) * 1000) / 1024);

	log_info("ADC: %01d.%03d V", (mili_volt / 1000), (mili_volt % 1000));

	uint8 status = 0;
	bool success = lis331_read_status(&status);

	if (success && SR_IS_ZDA(status) && SR_IS_YDA(status) && SR_IS_XDA(status))
	{
		struct AccelData accel =
		{ 0, 0, 0 };
		success = lis331_read_data(&accel);

		if (success)
		{
			log_info("LIS331 accel.: %6d; %6d; %6d", accel.xAccel, accel.yAccel,
					accel.zAccel);
		}
	}
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void user_init(void)
{
	UART_ResetFifo(UART0);
	uart_init_new();
	UART_SetBaudrate(UART0, 115200);
	printf("SDK version:%s\n", system_get_sdk_version());
	GPIO_AS_OUTPUT(5);
	GPIO_OUTPUT_SET(5, 1);
	GPIO_AS_OUTPUT(4);

	// Init I2C interface
	i2c_master_gpio_init();

	GPIO_OUTPUT_SET(4, 1);
	os_delay_us(10);
	GPIO_OUTPUT_SET(4, 0);

	// Init I2C device LIS331HH
	const bool success = lis331_init();
	if (!success)
	{
		log_error("failed to init device LIS331HH");
	}

	wifi_control_init();
	os_timer_setfn(&timer, (os_timer_func_t*) on_trigger_received, NULL);
	os_timer_arm(&timer, 200, 1);
}
