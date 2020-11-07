/* linux/drivers/hwmon/K303B.c
 *
 * (C) Copyright 2008
 * MediaTek <www.mediatek.com>
 *
 * K303B driver for MT6516
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#ifndef K303B
#define K303B
#include <linux/ioctl.h>

#define	I2C_AUTO_INCREMENT	(0x80)

#define	K303B_REG_CTL0			0x1F
#define K303B_REG_DEVID			0x0F
#define	K303B_REG_BW_RATE			0x20
#define K303B_REG_DATA_FORMAT		0x23
#define K303B_REG_DATA_FILTER		0x21
#define K303B_REG_DATAX0			0x28
#define K303B_REG_REG5			0x24

#define K303B_REG_OFSX            0XFF

#define K303B_FIXED_DEVID			0x41

/* Accelerometer Sensor Full Scale */
#define	K303B_ACC_FS_MASK	(0x30)
#define K303B_ACC_FS_2G 	(0x00)	/* Full scale 2g */
#define K303B_ACC_FS_4G 	(0x20)	/* Full scale 4g */
#define K303B_ACC_FS_8G 	(0x30)	/* Full scale 8g */

/* Sensitivity */
#define SENSITIVITY_ACC_2G	(16*1024)	/**	60ug/LSB	*/
#define SENSITIVITY_ACC_4G	(8*1024)	/**	120ug/LSB	*/
#define SENSITIVITY_ACC_8G	(4*1024)	/**	240ug/LSB	*/
#define SENSITIVITY_ACC_16G	(2*1024)	/**	480ug/LSB	*/


//#define K303B_BW_200HZ			0x0C
//#define K303B_BW_100HZ			0x0B
//#define K303B_BW_50HZ				0x0A
/* ODR */
#define ODR_ACC_MASK		(0x70)	/* Mask for odr change on acc */
#define K303B_ACC_ODR_OFF	(0x00)  /* Power down */
#define K303B_ACC_ODR10	(0x10)  /* 25Hz output data rate */
#define K303B_ACC_ODR50	(0x20)  /* 50Hz output data rate */
#define K303B_ACC_ODR100	(0x30)  /* 100Hz output data rate */
#define K303B_ACC_ODR200	(0x40)  /* 200Hz output data rate */
#define K303B_ACC_ODR400	(0x50)  /* 400Hz output data rate */
#define K303B_ACC_ODR800	(0x60)  /* 1600Hz output data rate */
#define K303B_ACC_ODR_ENABLE (0x07)


/* Accelerometer Filter */
#define IF_ADD_INC						0x04
#define AAF_BW_50Hz						0xC0
#define AAF_BW_100Hz					0x80

#define K303B_ACC_FILTER_MASK	(0xC0)
#define FILTER_773		773
#define FILTER_362		362
#define FILTER_194		194
#define FILTER_50		50

#define K303B_SUCCESS						0
#define K303B_ERR_I2C						-1
#define K303B_ERR_STATUS					-3
#define K303B_ERR_SETUP_FAILURE			-4
#define K303B_ERR_GETGSENSORDATA			-5
#define K303B_ERR_IDENTIFICATION			-6
#define K303B_INIT_SUCC						(0)
#define K303B_INIT_FAIL						(-1)

#define K303B_BUFSIZE				256

#endif
