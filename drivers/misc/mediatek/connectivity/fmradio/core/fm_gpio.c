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
 *   FmDrv_Gpio.h
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


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#if !defined(CONFIG_MTK_LEGACY)
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#else
#include <mt-plat/mt_gpio.h>
#endif
#include <linux/delay.h>
#include "fm_gpio.h"

#if !defined(CONFIG_MTK_LEGACY)

#ifdef CONFIG_MACH_MT6750S_CV5A
#include <soc/mediatek/lge/board_lge.h>
#endif

#if defined(CONFIG_SND_FM_SW_USE_GPIO_EXPANDER)
#ifdef CONFIG_TI_TCA6418
extern int ioexp_gpio_set_value(uint8_t gpio, uint8_t value);
#endif
#ifdef CONFIG_GPIO_PCAL6416A
extern int nxp_gpio_set_value(uint8_t gpio, uint8_t value);
#endif
#endif

struct pinctrl *pinctrlfm;
static int use_pinctrl;
static int fm_gpio;

enum fm_system_gpio_type {
	GPIO_FM_TDMB_SW_DEFAULT = 0,
	GPIO_FM_TDMB_SW_FM,
	GPIO_FM_TDMB_SW_TDMB,
	GPIO_NUM
};

struct fm_gpio_attr {
	const char *name;
	bool gpio_prepare;
	struct pinctrl_state *gpioctrl;
};

static struct fm_gpio_attr fm_gpios[GPIO_NUM] = {
	[GPIO_FM_TDMB_SW_DEFAULT] = {"fm_tdmb_sw_default", false, NULL},
	[GPIO_FM_TDMB_SW_FM] = {"fm_tdmb_sw_fm", false, NULL},
	[GPIO_FM_TDMB_SW_TDMB] = {"fm_tdmb_sw_tdmb", false, NULL},
};

void FmDrv_GPIO_probe(struct device* dev)
{
	int ret;
	int i = 0;

	pr_warn("%s\n", __func__);

	if(of_property_read_u32_index(dev->of_node, "lge,gpio-control", 0, &use_pinctrl)) {
			pr_err("lge,gpio-control is not exist.");
			use_pinctrl = 0;
	}

	if(use_pinctrl) {
		pinctrlfm = devm_pinctrl_get(dev);
		if (IS_ERR(pinctrlfm)) {
			ret = PTR_ERR(pinctrlfm);
			pr_err("Cannot find pinctrlfm!\n");
			return;
		}

		for (i = 0; i < ARRAY_SIZE(fm_gpios); i++) {
			fm_gpios[i].gpioctrl = pinctrl_lookup_state(pinctrlfm, fm_gpios[i].name);
			if (IS_ERR(fm_gpios[i].gpioctrl)) {
				ret = PTR_ERR(fm_gpios[i].gpioctrl);
				pr_err("%s pinctrl_lookup_state %s fail %d\n", __func__, fm_gpios[i].name,
			       	ret);
			} else {
				fm_gpios[i].gpio_prepare = true;
			}
		}
	} else {
#if defined(CONFIG_SND_FM_SW_USE_GPIO_EXPANDER)
		if(of_property_read_u32_index(dev->of_node, "lge,fm-gpio", 0, &fm_gpio)) {
			pr_err("Can not get lge,fm-gpio.");
			fm_gpio = -1;
		}
#else
		fm_gpio = of_get_named_gpio(dev->of_node, "lge,fm-gpio", 0);
#endif

	}
#ifndef CONFIG_SND_FM_SW_USE_GPIO_EXPANDER
	FmDrv_GPIO_FM_TDMB_Default();
#endif
}

#if defined(CONFIG_SND_FM_SW_USE_GPIO_EXPANDER)
void FmDrv_GPIO_Select_GPIO_Expander(int enable)
{
#ifdef CONFIG_MACH_MT6750S_CV5A
	if (lge_get_board_revno() == 0) {
		#if defined(CONFIG_TI_TCA6418)
		ioexp_gpio_set_value(fm_gpio, enable);
		#endif
	} else {
		#if defined(CONFIG_GPIO_PCAL6416A)
		nxp_gpio_set_value(fm_gpio, enable);
		#endif
	}
#else

#if defined(CONFIG_TI_TCA6418)
	ioexp_gpio_set_value(fm_gpio, enable);
#elif defined(CONFIG_GPIO_PCAL6416A)
	nxp_gpio_set_value(fm_gpio, enable);
#endif

#endif //CONFIG_MACH_MT6750S_CV5A
}
#endif //CONFIG_SND_FM_SW_USE_GPIO_EXPANDER

int FmDrv_GPIO_FM_TDMB_Default(void)
{
	int retval = 0;
	if(use_pinctrl) {
		if (fm_gpios[GPIO_FM_TDMB_SW_DEFAULT].gpio_prepare) {
			retval =
				pinctrl_select_state(pinctrlfm, fm_gpios[GPIO_FM_TDMB_SW_DEFAULT].gpioctrl);
			if (retval)
				pr_err("could not set fm_gpios[GPIO_FM_TDMB_SW_DEFAULT] pins\n");
		}
	} else {
		if(fm_gpio > -1) {
#if defined(CONFIG_SND_FM_SW_USE_GPIO_EXPANDER)
			FmDrv_GPIO_Select_GPIO_Expander(0);
#else
			gpio_set_value(fm_gpio, 0);
#endif
		} else {
			pr_err("Error : fm_gpio is %d\n", fm_gpio);
		}
	}
	return retval;
}

int FmDrv_GPIO_FM_TDMB_Select(int bValue)
{
	int retval = 0;

	if(use_pinctrl) {
		if (bValue == TDMB_SEL) { //TDMB
			if (fm_gpios[GPIO_FM_TDMB_SW_TDMB].gpio_prepare) {
				retval =
			    	pinctrl_select_state(pinctrlfm, fm_gpios[GPIO_FM_TDMB_SW_TDMB].gpioctrl);
					pr_warn("%s : TDMB_SEL\n", __func__);
				if (retval)
					pr_err("could not set fm_gpios[GPIO_FM_TDMB_SW_TDMB] pins\n");
			}
		} else {	//FM
			if (fm_gpios[GPIO_FM_TDMB_SW_FM].gpio_prepare) {
				retval =
			    	pinctrl_select_state(pinctrlfm, fm_gpios[GPIO_FM_TDMB_SW_FM].gpioctrl);
					pr_warn("%s : FM_SEL\n", __func__);
				if (retval)
					pr_err("could not set fm_gpios[GPIO_FM_TDMB_SW_FM] pins\n");
			}
		}
	} else {
		if (fm_gpio < 0) {
			pr_err("Error : FM Gpio is %d\n", fm_gpio);
			return -1;
		}

		if (bValue == TDMB_SEL) {
#if defined(CONFIG_SND_FM_SW_USE_GPIO_EXPANDER)
			FmDrv_GPIO_Select_GPIO_Expander(0);
#else
			gpio_set_value(fm_gpio, 0);
#endif
		} else {
#if defined(CONFIG_SND_FM_SW_USE_GPIO_EXPANDER)
			FmDrv_GPIO_Select_GPIO_Expander(1);
#else
			gpio_set_value(fm_gpio, 1);
#endif
		}
	}
	return retval;
}

#endif
