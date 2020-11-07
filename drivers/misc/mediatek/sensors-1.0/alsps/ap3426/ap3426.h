/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for AP3426 als/ps sensor chip.
 */
#ifndef __AP3426_H__
#define __AP3426_H__

#include <linux/ioctl.h>

/*ap3426 als/ps sensor register related macro*/
#define AP3426_FX                           1
#define AP3426_ALS_MIN_DATA                 0
#define AP3426_ALS_MAX_DATA                 65535
#define AP3426_ALS_READ_COUNT_MAX		10
#define AP3426_ALS_READ_COUNT_EN_PS		9
#define AP3426_ALS_AUTO_GAIN_HTHRES         ((3*AP3426_ALS_MAX_DATA)/4)
#define AP3426_ALS_AUTO_GAIN_LTHRES         ((2*AP3426_ALS_MAX_DATA)/16)
#define AP3426_ALS_AUTO_GAIN_LTHRES_L		(AP3426_ALS_MAX_DATA/100)
#define AP3426_ALS_LIGHT_RATIO_FLU		481
#define AP3426_ALS_LIGHT_RATIO_INC		591
#define AP3426_ALS_LIGHT_RATIO_R		39
#define AP3426_ALS_LIGHT_RATIO_R_FX		1000		// New Light Ratio Method:
#define AP3426_ALS_LIGHT_RATIO_NON		10000
#define AP3426_ALS_LIGHT_RATIO_FX		10000
#define AP3426_IR_DATA_MAX				1022

#define AP3426_ENABLE						0x00
#define AP3426_INT_STATUS					0x01
#define AP3426_INT_CTL                      0x02
#define AP3426_WAITING_TIME					0x06

/* ap3426 waiting time setting */
#define AP3426_ALSPS_WAITING_TIME			0X12
#define AP3426_PS_ONLY_WAITING_TIME			0x1c

#define AP3426_IRDATA_L						0x0A
#define AP3426_IRDATA_H						0x0B
#define AP3426_ADATA_L						0x0C
#define AP3426_ADATA_H						0x0D
#define AP3426_PDATA_L						0x0E
#define AP3426_PDATA_H						0x0F
#define AP3426_ALS_GAIN                     0x10
#define AP3426_INT_ALS_LOW_THD_LOW          0x1A
#define AP3426_INT_ALS_LOW_THD_HIGH         0x1B
#define AP3426_INT_ALS_HIGH_THD_LOW         0x1C
#define AP3426_INT_ALS_HIGH_THD_HIGH        0x1D
#define AP3426_CALIBRATION_LOW              0x28
#define AP3426_CALIBRATION_HIGH             0x29
#define AP3426_INT_PS_LOW_THD_LOW			0x2A
#define AP3426_INT_PS_LOW_THD_HIGH			0x2B
#define AP3426_INT_PS_HIGH_THD_LOW			0x2C
#define AP3426_INT_PS_HIGH_THD_HIGH			0x2D
#define AP3426_INT_PS_PRSIST				0x26
#define AP3426_PERSIST_EVERTIME				0x00
#define AP3426_PERSIST_2TIMES				0x02

#define AP3426_ALS_GAIN_D                   0x30
#define AP3426_ALS_GAIN_C                   0x20
#define AP3426_ALS_GAIN_B                   0x10
#define AP3426_ALS_GAIN_A                   0x00
#define AP3426_ALS_GAIN_D_RATION            (1) 	//x64
#define AP3426_ALS_GAIN_C_RATION            (4) 	//x16
#define AP3426_ALS_GAIN_B_RATION            (16)	//x4
#define AP3426_ALS_GAIN_A_RATION            (64)	//x1

/* System Trig status */
#define SYSTEM_TRIG_FIRST					1
#define SYSTEM_TRIG_SECOND					2

/*AP3426 related driver tag macro*/
#define AP3426_SUCCESS				        (0)
#define AP3426_ERR_I2C				        (-1)
#define AP3426_ERR_STATUS			        (-3)
#define AP3426_ERR_SETUP_FAILURE		    (-4)
#define AP3426_ERR_GETGSENSORDATA		    (-5)
#define AP3426_ERR_IDENTIFICATION		    (-6)

#endif

