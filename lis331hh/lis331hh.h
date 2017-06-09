/*
 * Copyright (c) 2017
 *
 *  Created on: 20.04.2017
 *      Author: denis
 */

#ifndef __LIS331HH_H__
#define __LIS331HH_H__

#include <esp_common.h>
#include <i2c_master.h>

#include <driver/i2c_common.h>

#define LIS331_ADDR 0x18

/**
 * Contains the acceleration data
 */
struct AccelData
{
	sint16 xAccel; ///< acceleration x axis
	sint16 yAccel; ///< acceleration y axis
	sint16 zAccel; ///< acceleration z axis
};

// Registers of LIS331HH
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define HP_FILTER_RESET 0x25
#define REFERENCE 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

#define SUB_AUTO_INC 0x80

// CTRL_REG1: Power modes
#define REG_PM_OFF 5
#define CTRL_REG1_PM_DOWN     (0x00 << REG_PM_OFF)
#define CTRL_REG1_PM_NORM     (0x01 << REG_PM_OFF)
#define CTRL_REG1_PM_LOW_0_5  (0x02 << REG_PM_OFF)
#define CTRL_REG1_PM_LOW_1_0  (0x03 << REG_PM_OFF)
#define CTRL_REG1_PM_LOW_2_0  (0x04 << REG_PM_OFF)
#define CTRL_REG1_PM_LOW_5_0  (0x05 << REG_PM_OFF)
#define CTRL_REG1_PM_LOW_10_0 (0x06 << REG_PM_OFF)

// CTRL_REG1: Output data rate
#define ODR_REG_OFF 3
#define CTRL_REG1_ODR_50   (0x00 << ODR_REG_OFF)
#define CTRL_REG1_ODR_100  (0x01 << ODR_REG_OFF)
#define CTRL_REG1_ODR_400  (0x10 << ODR_REG_OFF)
#define CTRL_REG1_ODR_1000 (0x11 << ODR_REG_OFF)

// CTRL_REG1: Axis enable bits
#define ZEN_REG_OFF 2
#define YEN_REG_OFF 1
#define XEN_REG_OFF 0
#define CTRL_REG1_ZEN (0x01 << ZEN_REG_OFF)
#define CTRL_REG1_YEN (0x01 << YEN_REG_OFF)
#define CTRL_REG1_XEN (0x01 << XEN_REG_OFF)

// CTRL_REG4:
#define BDU_REG_OFF 6
#define CTRL_REG4_BDU_CONT (0x00 << BDU_REG_OFF)
#define CTRL_REG4_BDU_BLCK (0x01 << BDU_REG_OFF)

#define FS_REG_OFF 4
#define CTRL_REG4_FS_6G  (0x00 << FS_REG_OFF)
#define CTRL_REG4_FS_12G (0x01 << FS_REG_OFF)
#define CTRL_REG4_FS_24G (0x11 << FS_REG_OFF)

// Status register
#define SR_IS_ZYXOR(STAT) ((STAT & 0x80) > 0)
#define SR_IS_ZOR(STAT)   ((STAT & 0x40) > 0)
#define SR_IS_YOR(STAT)   ((STAT & 0x20) > 0)
#define SR_IS_XOR(STAT)   ((STAT & 0x10) > 0)
#define SR_IS_ZYXDA(STAT) ((STAT & 0x08) > 0)
#define SR_IS_ZDA(STAT)   ((STAT & 0x04) > 0)
#define SR_IS_YDA(STAT)   ((STAT & 0x02) > 0)
#define SR_IS_XDA(STAT)   ((STAT & 0x01) > 0)

/**
 * @brief Requests the ID of the chip.
 */
bool lis331_read_status(uint8 * status);

/**
 * @brief Reads the current acceleration data
 */
bool lis331_read_data(struct AccelData * data);

/**
 * @brief Writes a value to defined register
 * @param reg register identifier
 * @param value value to be sent
 */
bool lis331_write_register(uint8 reg, uint8 value);

/**
 * @brief Initializes the chip
 */
bool lis331_init();

#endif // __LIS331HH_H__
