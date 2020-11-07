/* BOSCH Pressure Sensor Driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 * VERSION: V1.4
 * History: V1.0 --- [2013.03.14]Driver creation
 *          V1.1 --- [2013.07.03]Re-write I2C function to fix the bug that
 *                    i2c access error on MT6589 platform.
 *          V1.2 --- [2013.07.04]Add self test function.
 *          V1.3 --- [2013.07.04]Support new chip id 0x57 and 0x58.
 *          V1.4 --- [2017.03.06]modify source code to integrate into MTK6795(andriod M).
 *          V1.5 --- [2017.03.08]Add I2C retry in initializition.
 */

#ifndef BOSCH_BARO_H
#define BOSCH_BARO_H

#include <linux/ioctl.h>
/*****************************************************
*|  sensor  |   chip id  |     7-bit i2c address      |
-----------------------------------------------------|
|  bmp280  |    0x56    |0x76(SDO:Low)|0x77(SDO:High)|
*****************************************************/

/* apply low pass filter on output */
/*#define CONFIG_BMP_LOWPASS*/
/*#define CONFIG_ID_TEMPERATURE*/
/*#define CONFIG_I2C_BASIC_FUNCTION*/

#define BMP_DRIVER_VERSION "V1.5"

#define BMP_DEV_NAME        "bmp280"

#define C_MAX_FIR_LENGTH (32)
#define MAX_SENSOR_NAME  (32)
#define BMP_DATA_NUM 1
#define BMP_PRESSURE         0
#define BMP_BUFSIZE			128

/* common definition */
#define BMP_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMP_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


#define BMP_CHIP_ID_REG	0xD0

/*********************************[BMP280]*************************************/
/* data type */
#define BMP280_U16_t u16
#define BMP280_S16_t s16
#define BMP280_U32_t u32
#define BMP280_S32_t s32
#define BMP280_U64_t u64
#define BMP280_S64_t s64

/* chip id */
#define BMP280_CHIP_ID1 0x56
#define BMP280_CHIP_ID2 0x57
#define BMP280_CHIP_ID3 0x58

/* i2c address */
/* 7-bit addr: 0x76 (SDO connected to GND); 0x77 (SDO connected to VDDIO) */
#define BMP280_I2C_ADDRESS 0x76

/* calibration data */
#define BMP280_CALIBRATION_DATA_START       0x88 /* BMP280_DIG_T1_LSB_REG */
#define BMP280_CALIBRATION_DATA_LENGTH		24

#define SHIFT_RIGHT_4_POSITION				 4
#define SHIFT_LEFT_2_POSITION                2
#define SHIFT_LEFT_4_POSITION                4
#define SHIFT_LEFT_5_POSITION                5
#define SHIFT_LEFT_8_POSITION                8
#define SHIFT_LEFT_12_POSITION               12
#define SHIFT_LEFT_16_POSITION               16

/* power mode */
#define BMP280_SLEEP_MODE                    0x00
#define BMP280_FORCED_MODE                   0x01
#define BMP280_NORMAL_MODE                   0x03

#define BMP280_CTRLMEAS_REG                  0xF4  /* Ctrl Measure Register */

#define BMP280_CTRLMEAS_REG_MODE__POS              0
#define BMP280_CTRLMEAS_REG_MODE__MSK              0x03
#define BMP280_CTRLMEAS_REG_MODE__LEN              2
#define BMP280_CTRLMEAS_REG_MODE__REG              BMP280_CTRLMEAS_REG

/* filter */
#define BMP280_FILTERCOEFF_OFF               0x00
#define BMP280_FILTERCOEFF_2                 0x01
#define BMP280_FILTERCOEFF_4                 0x02
#define BMP280_FILTERCOEFF_8                 0x03
#define BMP280_FILTERCOEFF_16                0x04

#define BMP280_CONFIG_REG                    0xF5  /* Configuration Register */

#define BMP280_CONFIG_REG_FILTER__POS              2
#define BMP280_CONFIG_REG_FILTER__MSK              0x1C
#define BMP280_CONFIG_REG_FILTER__LEN              3
#define BMP280_CONFIG_REG_FILTER__REG              BMP280_CONFIG_REG

/* oversampling */
#define BMP280_OVERSAMPLING_SKIPPED          0x00
#define BMP280_OVERSAMPLING_1X               0x01
#define BMP280_OVERSAMPLING_2X               0x02
#define BMP280_OVERSAMPLING_4X               0x03
#define BMP280_OVERSAMPLING_8X               0x04
#define BMP280_OVERSAMPLING_16X              0x05

#define BMP280_CTRLMEAS_REG_OSRST__POS             5
#define BMP280_CTRLMEAS_REG_OSRST__MSK             0xE0
#define BMP280_CTRLMEAS_REG_OSRST__LEN             3
#define BMP280_CTRLMEAS_REG_OSRST__REG             BMP280_CTRLMEAS_REG

#define BMP280_CTRLMEAS_REG_OSRSP__POS             2
#define BMP280_CTRLMEAS_REG_OSRSP__MSK             0x1C
#define BMP280_CTRLMEAS_REG_OSRSP__LEN             3
#define BMP280_CTRLMEAS_REG_OSRSP__REG             BMP280_CTRLMEAS_REG

/* data */
#define BMP280_PRESSURE_MSB_REG              0xF7  /* Pressure MSB Register */
#define BMP280_TEMPERATURE_MSB_REG           0xFA  /* Temperature MSB Reg */

/* i2c disable switch */
#define BMP280_I2C_DISABLE_SWITCH            0x87

#define	BMP280_INIT_VALUE					(0)
#define	BMP280_INVALID_DATA					(0)

/* right shift definitions*/
#define BMP280_SHIFT_BIT_POSITION_BY_01_BIT				 (1)
#define BMP280_SHIFT_BIT_POSITION_BY_02_BITS			(2)
#define BMP280_SHIFT_BIT_POSITION_BY_03_BITS			(3)
#define BMP280_SHIFT_BIT_POSITION_BY_04_BITS			(4)
#define BMP280_SHIFT_BIT_POSITION_BY_05_BITS			(5)
#define BMP280_SHIFT_BIT_POSITION_BY_08_BITS			(8)
#define BMP280_SHIFT_BIT_POSITION_BY_11_BITS			(11)
#define BMP280_SHIFT_BIT_POSITION_BY_12_BITS			(12)
#define BMP280_SHIFT_BIT_POSITION_BY_13_BITS			(13)
#define BMP280_SHIFT_BIT_POSITION_BY_14_BITS			(14)
#define BMP280_SHIFT_BIT_POSITION_BY_15_BITS			(15)
#define BMP280_SHIFT_BIT_POSITION_BY_16_BITS			(16)
#define BMP280_SHIFT_BIT_POSITION_BY_17_BITS			(17)
#define BMP280_SHIFT_BIT_POSITION_BY_18_BITS			(18)
#define BMP280_SHIFT_BIT_POSITION_BY_19_BITS			(19)
#define BMP280_SHIFT_BIT_POSITION_BY_25_BITS			(25)
#define BMP280_SHIFT_BIT_POSITION_BY_31_BITS			(31)
#define BMP280_SHIFT_BIT_POSITION_BY_33_BITS			(33)
#define BMP280_SHIFT_BIT_POSITION_BY_35_BITS			(35)
#define BMP280_SHIFT_BIT_POSITION_BY_47_BITS			(47)
#endif/* BOSCH_BARO_H */
