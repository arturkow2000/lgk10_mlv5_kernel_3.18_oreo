/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
i * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/******************************************************************************
 * mt6575_vibrator.c - MT6575 Android Linux Vibrator Device Driver
 *
 * Copyright 2009-2010 MediaTek Co.,Ltd.
 *
 * DESCRIPTION:
 *     This file provid the other drivers vibrator relative functions
 *
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/types.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_hw.h>
#include "vibrator.h"

struct vibrator_hw *pvib_cust = NULL;

static int debug_enable_vib_hal = 1;
/* #define pr_fmt(fmt) "[vibrator]"fmt */
#define VIB_DEBUG(format, args...) do { \
	if (debug_enable_vib_hal) {\
		pr_debug(format, ##args);\
	} \
} while (0)

void vibr_Enable_HW(void)
{
#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
	pmic_set_register_value(PMIC_LDO_VIBR_EN, 1);     /* [bit 1]: VIBR_EN,  1=enable */
#else
	pmic_set_register_value(MT6351_PMIC_RG_VIBR_EN, 1);	/* [bit 1]: VIBR_EN,  1=enable */
#endif
	pr_info("vibrator_on\n");
}

void vibr_Disable_HW(void)
{
#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
	pmic_set_register_value(PMIC_LDO_VIBR_EN, 0);     /* [bit 1]: VIBR_EN,  1=enable */
#else
	pmic_set_register_value(MT6351_PMIC_RG_VIBR_EN, 0);	/* [bit 1]: VIBR_EN,  1=enable */
#endif
	pr_info("vibrator_off\n");
}

/******************************************
* Set RG_VIBR_VOSEL	Output voltage select
* hw->vib_vol:  Voltage selection
* 3'b000: 1.2V
* 3'b001: 1.3V
* 3'b010: 1.5V
* 3'b011: 1.8V
* 3'b100: 2.0V, if PMIC6353, 2.5V
* 3'b101: 2.8V
* 3'b110: 3.0V
* 3'b111: 3.3V
*******************************************/
struct vibrator_hw *get_cust_vibrator_dtsi(void)
{
	int ret;
	struct device_node *led_node = NULL;

	if (pvib_cust == NULL) {
		pvib_cust = kmalloc(sizeof(struct vibrator_hw), GFP_KERNEL);
		if (pvib_cust == NULL) {
			VIB_DEBUG("get_cust_vibrator_dtsi kmalloc fail\n");
			goto out;
		}

		led_node =
		    of_find_compatible_node(NULL, NULL, "mediatek,vibrator");
		if (!led_node) {
			VIB_DEBUG("Cannot find vibrator node from dts\n");
			kfree(pvib_cust);
			pvib_cust = NULL;
			goto out;
		} else {
			ret =
			    of_property_read_u32(led_node, "vib_timer",
						 &(pvib_cust->vib_timer));
			if (!ret) {
				VIB_DEBUG
				    ("The vibrator timer from dts is : %d\n",
				     pvib_cust->vib_timer);
			} else {
				pvib_cust->vib_timer = 25;
			}
#ifdef CUST_VIBR_LIMIT
			ret =
			    of_property_read_u32(led_node, "vib_limit",
						 &(pvib_cust->vib_limit));
			if (!ret) {
				VIB_DEBUG
				    ("The vibrator limit from dts is : %d\n",
				     pvib_cust->vib_limit);
			} else {
				pvib_cust->vib_limit = 9;
			}
#endif

#ifdef CUST_VIBR_VOL
			ret =
			    of_property_read_u32(led_node, "vib_vol",
						 &(pvib_cust->vib_vol));
			if (!ret) {
				VIB_DEBUG("The vibrator vol from dts is : %d\n",
					  pvib_cust->vib_vol);
			} else {
				pvib_cust->vib_vol = 0x05;
			}
#endif
		}
	}

 out:
	return pvib_cust;
}

void vibr_power_set(void)
{
#ifdef CUST_VIBR_VOL
	struct vibrator_hw *hw = get_cust_vibrator_dtsi();

	VIB_DEBUG("vibr_init: vibrator set voltage = %d\n", hw->vib_vol);
#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
	pmic_set_register_value(PMIC_RG_VIBR_VOSEL, hw->vib_vol);
#else
	pmic_set_register_value(MT6351_PMIC_RG_VIBR_VOSEL, hw->vib_vol);
#endif
#endif
}

void vibr_set_power_level(unsigned int val)
{
#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
	pmic_set_register_value(PMIC_RG_VIBR_VOSEL, val);
#else
	pmic_set_register_value(MT6351_PMIC_RG_VIBR_VOSEL, val);
#endif
}

struct vibrator_hw *mt_get_cust_vibrator_hw(void)
{
	struct vibrator_hw *hw = get_cust_vibrator_dtsi();
	return hw;
}
