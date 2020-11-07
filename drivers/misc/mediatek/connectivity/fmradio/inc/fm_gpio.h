/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program
 * If not, see <http://www.gnu.org/licenses/>.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   fm_gpio.h
 *
 * Project:
 * --------
 *   FM Driver Gpio control
 *
 * Description:
 * ------------
 *   FM TDMB Switch gpio control
 *
 * Author:
 * -------
 *   
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/

#ifndef _FMDRV_GPIO_H_
#define _FMDRV_GPIO_H_

/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/


/*****************************************************************************
 *                         M A C R O
 *****************************************************************************/


/*****************************************************************************
 *                 FUNCTION       D E F I N I T I O N
 *****************************************************************************/
#if !defined(CONFIG_MTK_LEGACY)
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/device.h>
#else
#include <mt-plat/mt_gpio.h>
#endif

#if !defined(CONFIG_MTK_LEGACY)
enum fm_switch {
	FM_SEL=0,
	TDMB_SEL
};
void FmDrv_GPIO_probe(struct device* dev);
int FmDrv_GPIO_FM_TDMB_Default(void);
int FmDrv_GPIO_FM_TDMB_Select(int bValue);
#if defined(CONFIG_SND_FM_SW_USE_GPIO_EXPANDER)
void FmDrv_GPIO_Select_GPIO_Expander(int enable);
#endif
#endif

#endif
