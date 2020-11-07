/* 
 * ICM406XX sensor driver
 * Copyright (C) 2018 Invensense, Inc.
 *
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
 */

#ifndef ICM406XX_SHARE_INTERFACE_H
#define ICM406XX_SHARE_INTERFACE_H

#define ICM406XX_SUCCESS             		0
#define ICM406XX_ERR_BUS             		-1
#define ICM406XX_ERR_INVALID_PARAM     		-2
#define ICM406XX_ERR_STATUS          		-3
#define ICM406XX_ERR_SETUP_FAILURE   		-4

#define ICM406XX_DBG 1
#define ICM_TAG 	"[ICM406XX]"

#if ICM406XX_DBG
#define ACC_ERR(fmt, args...)	printk(KERN_ERR ICM_TAG "%s %d : "fmt, __func__, __LINE__, ##args)
#define GYRO_ERR(fmt, args...)	printk(KERN_ERR ICM_TAG "%s %d : "fmt, __func__, __LINE__, ##args)
#define STEP_C_ERR(fmt, args...)	printk(KERN_ERR ICM_TAG "%s %d : "fmt, __func__, __LINE__, ##args)
#else
#define ACC_ERR(fmt, args...)
#define GYRO_ERR(fmt, args...)
#endif

extern int icm406xx_share_read_register(u8 addr, u8 *data, u8 len);
extern int icm406xx_share_write_register(u8 addr, u8 *data, u8 len);

#endif /* ICM406XX_SHARE_INTERFACE_H */
