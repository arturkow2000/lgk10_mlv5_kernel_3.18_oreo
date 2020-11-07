/* SPL06 GOERTEK pressure sensor driver
*
*
* This software program is licensed subject to the GNU General Public License
* (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

* (C) Copyright 2017 GOERTEK INC.
* All Rights Reserved
* VERSION: V1.1
* History: V1.0 --- [2017.05.16]Driver creation
*          V1.1 --- [2017.11.06]modify source code to integrate into MTK6750S(andriod M).
*/

#ifndef SPL06_H
#define SPL06_H
#include <linux/ioctl.h>

//#define CONFIG_ID_TEMPERATURE 

#define SPL_DRIVER_VERSION "V1.1"

#define SPL_DEV_NAME        "spl06"

#define C_MAX_FIR_LENGTH (32)
#define MAX_SENSOR_NAME  (32)
#define SPL_DATA_NUM 1
#define SPL_PRESSURE         0
#define SPL_BUFSIZE			128

/* common definition */
#define SPL_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define SPL_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/* chip id */
#define SPL06_CHIP_ID                       0x10

/*********************************[SPL06]*************************************/
/* 7-bit addr: 0x76 (SDO connected to GND); 0x77 (SDO connected to VDDIO) */
#define SPL06_I2C_ADDRESS 					0x76

/* power mode */
#define SPL06_SLEEP_MODE                    0x00
#define SPL06_NORMAL_MODE                   0x07

/* sample rate*/
#define SPL06_SAMPLERATE_1               	(0x00)
#define SPL06_SAMPLERATE_2                  (0x01)
#define SPL06_SAMPLERATE_4                 	(0x02)
#define SPL06_SAMPLERATE_8                 	(0x03)
#define SPL06_SAMPLERATE_16                	(0x04)
#define SPL06_SAMPLERATE_32                	(0x05)
#define SPL06_SAMPLERATE_64                	(0x06)
#define SPL06_SAMPLERATE_128               	(0x07)

/* oversampling */
#define SPL06_OVERSAMPLING_1X          		(0x00)
#define SPL06_OVERSAMPLING_2X               (0x01)
#define SPL06_OVERSAMPLING_4X               (0x02)
#define SPL06_OVERSAMPLING_8X               (0x03)
#define SPL06_OVERSAMPLING_16X              (0x04)
#define SPL06_OVERSAMPLING_32X              (0x05)
#define SPL06_OVERSAMPLING_64X              (0x06)
#define SPL06_OVERSAMPLING_128X             (0x07)

/* work mode */
#define SPL06_LOW_POWER_MODE	            (0x00)
#define SPL06_STANDARD_RESOLUTION_MODE      (0x01)
#define SPL06_HIGH_RESOLUTION_MODE          (0x02)

#define SPL06_LOWPOWER_SAMPLERATE_PRESSURE	           		SPL06_SAMPLERATE_1
#define SPL06_LOWPOWER_OVERSAMPLING_PRESSURE	           	SPL06_OVERSAMPLING_2X
#define SPL06_LOWPOWER_SAMPLERATE_TEMPERATURE	       		SPL06_SAMPLERATE_1
#define SPL06_LOWPOWER_OVERSAMPLING_TEMPERATURE	           	SPL06_OVERSAMPLING_1X

#define SPL06_STANDARDRESOLUTION_SAMPLERATE_PRESSURE     	SPL06_SAMPLERATE_2
#define SPL06_STANDARDRESOLUTION_OVERSAMPLING_PRESSURE     	SPL06_OVERSAMPLING_16X
#define SPL06_STANDARDRESOLUTION_SAMPLERATE_TEMPERATURE  	SPL06_SAMPLERATE_1
#define SPL06_STANDARDRESOLUTION_OVERSAMPLING_TEMPERATURE  	SPL06_OVERSAMPLING_1X

#define SPL06_HIGHRESOLUTION_SAMPLERATE_PRESSURE         	SPL06_SAMPLERATE_4
#define SPL06_HIGHRESOLUTION_OVERSAMPLING_PRESSURE         	SPL06_OVERSAMPLING_64X
#define SPL06_HIGHRESOLUTION_SAMPLERATE_TEMPERATURE      	SPL06_SAMPLERATE_4
#define SPL06_HIGHRESOLUTION_OVERSAMPLING_TEMPERATURE      	SPL06_OVERSAMPLING_1X

#define SPL06_PRESSURE_SHIFT 				0x04
#define SPL06_TEMPERATURE_SHIFT     		0x08

#define SPL06_TMP_SOURCE_INT				0x00
#define SPL06_TMP_SOURCE_EXT				0x80

/* calibration data */
#define SPL06_CALIBRATION_DATA_START       	(0x10)
#define SPL06_CALIBRATION_DATA_LENGTH		(18)

/* register address */
#define SPL06_CHIP_ID_REG                   (0x0D)  /*Chip ID Register */
#define SPL06_RESET_REG                     (0x0C)  /*Softreset Register */
#define SPL06_INT_STATUS_REG                (0x0A)  /*Status Register */
#define SPL06_FIFO_STATUS_REG               (0x0B)  /*Status Register */
#define SPL06_PRS_CFG_REG                   (0x06)  /*Pressure Config Register */
#define SPL06_TMP_CFG_REG                   (0x07)  /*Temperature Config Register */
#define SPL06_CTRL_MEAS_REG                 (0x08)  /*Ctrl Measure Register */
#define SPL06_CONFIG_REG                    (0x09)  /*Configuration Register */

/* data */
#define SPL06_PRESSURE_MSB_REG              (0x00)  /*Pressure MSB Register */
#define SPL06_PRESSURE_LSB_REG              (0x01)  /*Pressure LSB Register */
#define SPL06_PRESSURE_XLSB_REG             (0x02)  /*Pressure XLSB Register */
#define SPL06_TEMPERATURE_MSB_REG           (0x03)  /*Temperature MSB Reg */
#define SPL06_TEMPERATURE_LSB_REG           (0x04)  /*Temperature LSB Reg */
#define SPL06_TEMPERATURE_XLSB_REG          (0x05)  /*Temperature XLSB Reg */

/* pressure oversampling bit definition */
#define SPL06_PRS_CFG_REG_OVERSAMPLING__POS   	(0)
#define SPL06_PRS_CFG_REG_OVERSAMPLING__MSK     (0x0F)
#define SPL06_PRS_CFG_REG_OVERSAMPLING__LEN     (4)
#define SPL06_PRS_CFG_REG_OVERSAMPLING__REG     (SPL06_PRS_CFG_REG)

/* temperature oversampling bit definition */
#define SPL06_TMP_CFG_REG_OVERSAMPLING__POS   	(0)
#define SPL06_TMP_CFG_REG_OVERSAMPLING__MSK   	(0x07)
#define SPL06_TMP_CFG_REG_OVERSAMPLING__LEN   	(3)
#define SPL06_TMP_CFG_REG_OVERSAMPLING__REG   	(SPL06_TMP_CFG_REG)

/* pressure samplerate bit definition */
#define SPL06_PRS_CFG_REG_SAMPLERATE__POS      	(4)
#define SPL06_PRS_CFG_REG_SAMPLERATE__MSK    	(0x70)
#define SPL06_PRS_CFG_REG_SAMPLERATE__LEN     	(3)
#define SPL06_PRS_CFG_REG_SAMPLERATE__REG   	(SPL06_PRS_CFG_REG)

/* temperature samplerate bit definition */
#define SPL06_TMP_CFG_REG_SAMPLERATE__POS      	(4)
#define SPL06_TMP_CFG_REG_SAMPLERATE__MSK      	(0x70)
#define SPL06_TMP_CFG_REG_SAMPLERATE__LEN      	(3)
#define SPL06_TMP_CFG_REG_SAMPLERATE__REG      	(SPL06_TMP_CFG_REG)

/* power mode bit definition */
#define SPL06_CTRL_MEAS_REG_POWER_MODE__POS   	(0)
#define SPL06_CTRL_MEAS_REG_POWER_MODE__MSK     (0x07)
#define SPL06_CTRL_MEAS_REG_POWER_MODE__LEN     (3)
#define SPL06_CTRL_MEAS_REG_POWER_MODE__REG     (SPL06_CTRL_MEAS_REG)

#endif
