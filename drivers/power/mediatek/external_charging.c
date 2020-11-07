/*
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *    external_charging.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   08 Apr 2014 07:47:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/types.h>
#include <linux/kernel.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/battery_meter_hal.h>
#include <mt-plat/charging.h>
#include <mach/mt_charging.h>
#ifdef CONFIG_MTK_BOOT
#include <mt-plat/mt_boot.h>
#endif

#include "mtk_pep_intf.h"
#include "mtk_pep20_intf.h"

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mt-plat/diso.h>
#include <mach/mt_diso.h>
#endif

#ifdef CONFIG_MACH_LGE
#include <soc/mediatek/lge/board_lge.h>
#include <soc/mediatek/lge/lge_boot_mode.h>
#endif

#ifdef CONFIG_LGE_PM_USB_ID
#include <soc/mediatek/lge/lge_cable_id.h>
#endif

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
#include <soc/mediatek/lge/lge_pseudo_batt.h>
#endif

#ifdef CONFIG_LGE_PM_CHARGER_CONTROLLER
#include <linux/power/charger-controller.h>
#endif

#ifdef CONFIG_INPUT_EPACK
#include <linux/input/epack.h>
#endif

/* ============================================================ // */
/* define */
/* ============================================================ // */
#define STATUS_OK    0
#define STATUS_UNSUPPORTED    -1
#define STATUS_FAIL -2

/* ============================================================ // */
/* global variable */
/* ============================================================ // */
unsigned int g_bcct_flag = 0;
unsigned int g_bcct_value = 0;
/*input-output curent distinction*/
unsigned int g_bcct_input_flag = 0;
unsigned int g_bcct_input_value = 0;

/* Parallel Charger Detection */
static int portion_slave_icl = 50;	/* in percent */
static int portion_slave_fcc = 50;	/* in percent */
static int step_slave_icl = CHARGE_CURRENT_100_00_MA;
static int step_slave_fcc = CHARGE_CURRENT_100_00_MA;

static bool g_parallel_init = false;
static bool g_parallel_flag = false;
CHR_CURRENT_ENUM g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_CC_slave = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_slave = CHARGE_CURRENT_0_00_MA;
unsigned int g_usb_state = USB_UNCONFIGURED;
static bool usb_unlimited;
#if defined(CONFIG_MTK_HAFG_20)
#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
BATTERY_VOLTAGE_ENUM g_cv_voltage = BATTERY_VOLT_04_400000_V;
#else
BATTERY_VOLTAGE_ENUM g_cv_voltage = BATTERY_VOLT_04_200000_V;
#endif
unsigned int get_cv_voltage(void)
{
	return g_cv_voltage;
}
#endif
DEFINE_MUTEX(g_ichg_aicr_access_mutex);
DEFINE_MUTEX(g_aicr_access_mutex);
DEFINE_MUTEX(g_ichg_access_mutex);
DEFINE_MUTEX(g_hv_charging_mutex);
unsigned int g_aicr_upper_bound;
static bool g_enable_dynamic_cv = true;
static bool g_enable_hv_charging = true;

/* ============================================================ // */
/* function prototype */
/* ============================================================ // */


/* ============================================================ // */
/* extern variable */
/* ============================================================ // */


/* ============================================================ // */
/* extern function */
/* ============================================================ // */
#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
extern int get_AtCmdChargingModeOff(void);
#endif

// ============================================================ //
void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_log(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\n");
#else
	if ((usb_state_value < USB_SUSPEND) || (usb_state_value > USB_CONFIGURED)) {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] BAT_SetUSBState Fail! Restore to default value\n");
		g_usb_state = USB_UNCONFIGURED;
	} else {
		battery_log(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Success! Set %d\n",
				usb_state_value);
		g_usb_state = usb_state_value;
	}
#endif
}

unsigned int get_charging_setting_current(void)
{
	return g_temp_CC_value;
}

int mtk_get_dynamic_cv(unsigned int *cv)
{
	int ret = 0;
#ifdef CONFIG_MTK_BIF_SUPPORT
	u32 _cv;
	u32 vbat_bif = 0, vbat_auxadc = 0, vbat = 0;
	u32 retry_cnt = 0;
	u32 ircmp_volt_clamp = 0, ircmp_resistor = 0;

	if (!g_enable_dynamic_cv) {
		if (batt_cust_data.high_battery_voltage_support)
			_cv = BATTERY_VOLT_04_400000_V / 1000;
		else
			_cv = BATTERY_VOLT_04_200000_V / 1000;
		goto _out;
	}

	do {
		ret = battery_charging_control(CHARGING_CMD_GET_BIF_VBAT,
			&vbat_bif);
		vbat_auxadc = battery_meter_get_battery_voltage(KAL_TRUE);

		if (ret >= 0 && vbat_bif != 0 && vbat_bif < vbat_auxadc) {
			vbat = vbat_bif;
			battery_log(BAT_LOG_CRTI,
				"%s: use BIF vbat = %dmV, dV to auxadc = %dmV\n",
				__func__, vbat, vbat_auxadc - vbat_bif);
			break;
		}

		retry_cnt++;
	} while (retry_cnt < 5);

	if (retry_cnt == 5) {
		ret = 0;
		vbat = vbat_auxadc;
		battery_log(BAT_LOG_CRTI,
			"%s: use AUXADC vbat = %dmV, since BIF vbat = %d\n",
			__func__, vbat_auxadc, vbat_bif);
	}

	/* Adjust CV according to the obtained vbat */
	if (vbat >= 3400 && vbat < 4300) {
		_cv = 4550;
		battery_charging_control(CHARGING_CMD_SET_IRCMP_VOLT_CLAMP,
			&ircmp_volt_clamp);
		battery_charging_control(CHARGING_CMD_SET_IRCMP_RESISTOR,
			&ircmp_resistor);
	} else {
		if (batt_cust_data.high_battery_voltage_support)
			_cv = BATTERY_VOLT_04_400000_V / 1000;
		else
			_cv = BATTERY_VOLT_04_200000_V / 1000;

		/* Turn on IR compensation */
		ircmp_volt_clamp = 200;
		ircmp_resistor = 80;
		battery_charging_control(CHARGING_CMD_SET_IRCMP_VOLT_CLAMP,
			&ircmp_volt_clamp);
		battery_charging_control(CHARGING_CMD_SET_IRCMP_RESISTOR,
			&ircmp_resistor);

		/* Disable dynamic CV */
		g_enable_dynamic_cv = false;
	}

_out:
	*cv = _cv;
	battery_log(BAT_LOG_CRTI, "%s: CV = %dmV, enable dynamic cv = %d\n",
		__func__, _cv, g_enable_dynamic_cv);
#else
	ret = -ENOTSUPP;
#endif
	return ret;
}

bool get_usb_current_unlimited(void)
{
	if (BMT_status.charger_type == STANDARD_HOST || BMT_status.charger_type == CHARGING_HOST)
		return usb_unlimited;

	return false;
}

void set_usb_current_unlimited(bool enable)
{
	usb_unlimited = enable;
}

static void mtk_select_ichg_aicr(void);
unsigned int set_bat_charging_current_limit(int current_limit)
{
#ifdef CONFIG_LGE_PM_CHARGER_CONTROLLER
	chgctrl_thermal_limit(current_limit);
#else
	CHR_CURRENT_ENUM chr_type_ichg = 0;
	CHR_CURRENT_ENUM chr_type_aicr = 0;

	mutex_lock(&g_ichg_access_mutex);
	if (current_limit != -1) {
		g_bcct_flag = 1;
		switch (BMT_status.charger_type) {
		case STANDARD_HOST:
			chr_type_ichg = batt_cust_data.usb_charger_current;
			break;
		case NONSTANDARD_CHARGER:
			chr_type_ichg = batt_cust_data.non_std_ac_charger_current;
			break;
		case STANDARD_CHARGER:
			chr_type_ichg = batt_cust_data.ac_charger_current;
			mtk_pep20_set_charging_current(&chr_type_ichg, &chr_type_aicr);
			mtk_pep_set_charging_current(&chr_type_ichg, &chr_type_aicr);
			break;
		case CHARGING_HOST:
			chr_type_ichg = batt_cust_data.charging_host_charger_current;
			break;
		case APPLE_2_1A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_2_1a_charger_current;
			break;
		case APPLE_1_0A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_1_0a_charger_current;
			break;
		case APPLE_0_5A_CHARGER:
			chr_type_ichg = batt_cust_data.apple_0_5a_charger_current;
			break;
		default:
			chr_type_ichg = CHARGE_CURRENT_500_00_MA;
			break;
		}
		chr_type_ichg /= 100;
		if (current_limit > chr_type_ichg)
			current_limit = chr_type_ichg;
		g_bcct_value = current_limit;
	} else /* change to default current setting */
		g_bcct_flag = 0;

	mtk_select_ichg_aicr();

	battery_log(BAT_LOG_CRTI,
		"[BATTERY] set_bat_charging_current_limit (%d)\r\n",
		current_limit);

	mutex_unlock(&g_ichg_access_mutex);
#endif
	return g_bcct_flag;
}

int mtk_chr_reset_aicr_upper_bound(void)
{
	g_aicr_upper_bound = 0;
	return 0;
}

int mtk_chr_enable_hv_charging(bool en)
{
	battery_log(BAT_LOG_CRTI, "%s: en = %d\n", __func__, en);

	mutex_lock(&g_hv_charging_mutex);
	g_enable_hv_charging = en;
	mutex_unlock(&g_hv_charging_mutex);

	return 0;
}

bool mtk_chr_is_hv_charging_enable(void)
{
	return g_enable_hv_charging;
}

int set_chr_boost_current_limit(unsigned int current_limit)
{
	return battery_charging_control(CHARGING_CMD_SET_BOOST_CURRENT_LIMIT,
			&current_limit);
}

int set_chr_enable_otg(unsigned int enable)
{
	/* If the OTG cable is inserted on boot, the function is not prepared */
	/* sometimes. So it must be checked in order to avoid NULL */
	#ifdef CONFIG_LGE_PM
	if (battery_charging_control == NULL) {
		battery_charging_control = chr_control_interface;
		battery_log(BAT_LOG_CRTI, "%s: Set i/f avoiding except.\n", __func__);
	}
	#endif
	return battery_charging_control(CHARGING_CMD_ENABLE_OTG, &enable);
}

int mtk_chr_get_tchr(int *min_temp, int *max_temp)
{
	int ret = 0;
	int temp[2] = {0, 0};

	ret = battery_charging_control(CHARGING_CMD_GET_CHARGER_TEMPERATURE, temp);
	if (ret < 0)
		return ret;

	*min_temp = temp[0];
	*max_temp = temp[1];

	return ret;
}

int mtk_chr_get_soc(unsigned int *soc)
{
	if (BMT_status.SOC < 0) {
		*soc = 0;
		return -ENOTSUPP;
	}

	*soc = BMT_status.SOC;

	return 0;
}

int mtk_chr_get_ui_soc(unsigned int *ui_soc)
{
	/* UI_SOC2 is the one that shows on UI */
	if (BMT_status.UI_SOC2 < 0) {
		*ui_soc = 0;
		return -ENOTSUPP;
	}

	*ui_soc = BMT_status.UI_SOC2;

	return 0;
}

int mtk_chr_get_vbat(unsigned int *vbat)
{
	if (BMT_status.bat_vol < 0) {
		*vbat = 0;
		return -ENOTSUPP;
	}

	*vbat = BMT_status.bat_vol;

	return 0;
}

int mtk_chr_get_ibat(unsigned int *ibat)
{
	*ibat = BMT_status.IBattery / 10;
	return 0;
}

int mtk_chr_get_vbus(unsigned int *vbus)
{
	if (BMT_status.charger_vol < 0) {
		*vbus = 0;
		return -ENOTSUPP;
	}
	*vbus = BMT_status.charger_vol;

	return 0;
}

int mtk_chr_get_aicr(unsigned int *aicr)
{
	int ret = 0;
	u32 _aicr = 0; /* 10uA */

	ret = battery_charging_control(CHARGING_CMD_GET_INPUT_CURRENT, &_aicr);
	*aicr = _aicr / 100;

	return ret;
}

int mtk_chr_is_charger_exist(unsigned char *exist)
{
	*exist = (BMT_status.charger_exist ? 1 : 0);
	return 0;
}

static int detect_extchg_slave(void)
{
	kal_bool slave_exist = KAL_FALSE;

	battery_charging_control(CHARGING_CMD_GET_EXCHG_SLAVE, &slave_exist);
	battery_log(BAT_LOG_CRTI, "[BATTERY] %s : slave_exist = %d \n", __func__, slave_exist);

	g_parallel_init = true;

	if (slave_exist)
		g_parallel_flag = true;
	else
		g_parallel_flag = false;

	return 0;
}

static int charging_enabled_check(void)
{
	if (BMT_status.charging_enabled == KAL_FALSE)
		return 0;
#ifdef CONFIG_LGE_PM_CHARGER_CONTROLLER
	if (!chgctrl_charging_enabled())
		return 0;
#endif

	return 1;
}

static int battery_charging_enabled_check(void)
{
	if (BMT_status.bat_exist == KAL_FALSE)
		return 0;
	if (BMT_status.battery_charging_enabled == KAL_FALSE)
		return 0;
#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
	if (get_AtCmdChargingModeOff())
		return 0;
#endif
#ifdef CONFIG_LGE_PM_CHARGER_CONTROLLER
	if (!chgctrl_battery_charging_enabled())
		return 0;
#endif

	return 1;
}

static unsigned int charging_full_check(void)
{
	unsigned int status = KAL_FALSE;

	battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &status);

	return status;
}

#ifdef CONFIG_LGE_USB_TYPE_C
static bool is_typec_valid_for_pep(void)
{
	bool valid = true;

	if (lge_get_factory_boot())
		return valid;

	switch (BMT_status.typec_mode) {
	case POWER_SUPPLY_TYPEC_NONE:
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
	case POWER_SUPPLY_TYPEC_NON_COMPLIANT:
		valid = false;
		break;
	default:
		break;
	}

	return valid;
}
#endif

static bool mtk_is_pep_series_connect(void)
{
	if (!batt_cust_data.mtk_pump_express_plus_support)
		return false;

	if (mtk_pep20_get_is_connect() || mtk_pep_get_is_connect())
		return true;

	return false;
}

static bool mtk_is_pep_series_avaliable(void)
{
	if (!batt_cust_data.mtk_pump_express_plus_support)
		return false;

#ifdef CONFIG_LGE_PM_USB_ID
	if (lge_is_factory_cable_boot()) {
		battery_log(BAT_LOG_CRTI, "[PE+] Skip : factory_cable_boot\n");
		return false;
	}
#endif

	if (BMT_status.charger_type != STANDARD_CHARGER)
		return false;

	/* input current suspended */
	if (BMT_status.bat_charging_state == CHR_ERROR)
		return false;

	/* battery charging suspended */
	if (BMT_status.bat_charging_state == CHR_HOLD)
		return false;

	if (!chgctrl_hvdcp_enabled())
		return false;

	if (BMT_status.SOC >= 100) {
		battery_log(BAT_LOG_CRTI, "[PE+] Skip : Raw SOC over 100\n");
		return false;
	}

	return true;
}

static void mtk_pep_series_check_charger(void)
{
	if (mtk_is_pep_series_connect())
		return;

	if (!mtk_is_pep_series_avaliable())
		return;

#ifdef CONFIG_LGE_PM_USB_ID
	/* do not check pep when factory cable */
	if (lge_is_factory_cable()) {
		battery_log(BAT_LOG_CRTI, "[PE+] Skip : factory_cable detect\n");
		return;
	}
#endif

	if (!chgctrl_hvdcp_enabled()) {
		battery_log(BAT_LOG_CRTI, "[PE+] Skip : by charger_controller\n");
		return;
	}

#ifdef CONFIG_LGE_USB_TYPE_C
	if (!is_typec_valid_for_pep()) {
		battery_log(BAT_LOG_CRTI, "[PE+] Skip : Invalid USB_C mode\n");
		return;
	}
#endif

	if (BMT_status.bat_charging_state == CHR_CC) {
		if (BMT_status.UI_SOC2 >= 100 && BMT_status.bat_in_recharging_state == KAL_TRUE){
			battery_log(BAT_LOG_CRTI, "[PE+] Skip : UI_SOC2 100 and recharging\n");
			return;
		}
	}

	if (BMT_status.bat_charging_state == CHR_PRE) {
		/* enable input to make current pattern */
		battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH,
				&BMT_status.charger_exist);
	}

	/* enable to check charger */
	if (!mtk_pep20_get_is_enable()) {
		mtk_pep20_set_is_enable(true);
		mtk_pep20_set_to_check_chr_type(true);
	}
	if (!mtk_pep_get_is_enable()) {
		mtk_pep_set_is_enable(true);
		mtk_pep_set_to_check_chr_type(true);
	}

	mtk_pep20_check_charger();
	mtk_pep_check_charger();

	/* for faster update */
	if (mtk_is_pep_series_connect()) {
		battery_log(BAT_LOG_CRTI, "[PE+] End of detection, Fastchg update\n");
		mt_battery_update_status();
	}
}

#ifndef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
static int mtk_check_aicr_upper_bound(void)
{
	u32 aicr_upper_bound = 0; /* 10uA */

	if (mtk_is_pep_series_connect())
		return -EPERM;

	/* Check AICR upper bound gererated by AICL */
	aicr_upper_bound = g_aicr_upper_bound * 100;
	if (g_temp_input_CC_value > aicr_upper_bound && aicr_upper_bound > 0)
		g_temp_input_CC_value = aicr_upper_bound;

	return 0;
}
#endif

#ifdef CONFIG_LGE_USB_TYPE_C
static void select_charging_current_typec(void)
{
	if (mtk_is_pep_series_connect())
		return;

	switch (BMT_status.typec_mode) {
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		g_temp_input_CC_value = batt_cust_data.typec_high_input_current;
		g_temp_CC_value = batt_cust_data.typec_high_charging_current;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
		g_temp_input_CC_value = batt_cust_data.typec_medium_input_current;
		g_temp_CC_value = batt_cust_data.typec_medium_charging_current;
		break;
	default:
		break;
	}
}
#endif

static void select_charging_current(void)
{
#ifdef CONFIG_LGE_PM_USB_ID
	if (lge_is_factory_cable_boot()) {
		g_temp_input_CC_value = CHARGE_CURRENT_1500_00_MA;
		g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
		if (BMT_status.bat_exist == KAL_FALSE)
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
#ifdef CONFIG_LGE_USB_EMBEDDED_BATTERY
		if (BMT_status.bat_exist == KAL_TRUE && ((unsigned int) lge_read_cable_type() == LT_CABLE_910K))
			g_temp_input_CC_value = CHARGE_CURRENT_800_00_MA;
#endif
#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
		if (get_AtCmdChargingModeOff())
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
#endif
		return;
	}
#endif

	if (BMT_status.charger_type == STANDARD_HOST) {
		if (batt_cust_data.config_usb_if) {
			if (g_usb_state == USB_SUSPEND)
				g_temp_input_CC_value = batt_cust_data.usb_charger_current_suspend;
			else if (g_usb_state == USB_UNCONFIGURED)
				g_temp_input_CC_value = batt_cust_data.usb_charger_current_unconfigured;
			else if (g_usb_state == USB_CONFIGURED)
				g_temp_input_CC_value = batt_cust_data.usb_charger_current_configured;
			else
				g_temp_input_CC_value = batt_cust_data.usb_charger_current_unconfigured;

#ifdef CONFIG_MTK_BOOT
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
			/* If chargerlogo boot, usb current set 500mA */
			if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT ||
					get_boot_mode() == LOW_POWER_OFF_CHARGING_BOOT) {
				g_temp_input_CC_value = batt_cust_data.usb_charger_current;
			}
#endif
#ifdef CONFIG_LGE_PM_CHARGERLOGO
			/* If chargerlogo boot, usb current set 500mA */
			if (get_boot_mode() == CHARGER_BOOT) {
				g_temp_input_CC_value = batt_cust_data.usb_charger_current;
			}
#endif
#endif

			g_temp_CC_value = batt_cust_data.usb_charger_current;

			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] STANDARD_HOST CC mode charging : %d on %d state\r\n",
				    g_temp_CC_value, g_usb_state);
		} else {
			g_temp_input_CC_value = batt_cust_data.usb_charger_current;
			g_temp_CC_value = batt_cust_data.usb_charger_current;
		}

	} else if (BMT_status.charger_type == NONSTANDARD_CHARGER) {
		g_temp_input_CC_value = batt_cust_data.non_std_ac_charger_current;
		g_temp_CC_value = batt_cust_data.non_std_ac_charger_current;
	} else if (BMT_status.charger_type == STANDARD_CHARGER) {
		g_temp_input_CC_value = batt_cust_data.ac_charger_input_current;
		g_temp_CC_value = batt_cust_data.ac_charger_current;
		if (mtk_is_pep_series_connect()) {
			g_temp_input_CC_value = batt_cust_data.ta_ac_9v_input_current;
			g_temp_CC_value = batt_cust_data.ta_ac_charging_current;
		}
	} else if (BMT_status.charger_type == CHARGING_HOST) {
		g_temp_input_CC_value = batt_cust_data.charging_host_charger_current;
		g_temp_CC_value = batt_cust_data.charging_host_charger_current;
	} else if (BMT_status.charger_type == APPLE_2_1A_CHARGER) {
		g_temp_input_CC_value = batt_cust_data.apple_2_1a_charger_current;
		g_temp_CC_value = batt_cust_data.apple_2_1a_charger_current;
	} else if (BMT_status.charger_type == APPLE_1_0A_CHARGER) {
		g_temp_input_CC_value = batt_cust_data.apple_1_0a_charger_current;
		g_temp_CC_value = batt_cust_data.apple_1_0a_charger_current;
	} else if (BMT_status.charger_type == APPLE_0_5A_CHARGER) {
		g_temp_input_CC_value = batt_cust_data.apple_0_5a_charger_current;
		g_temp_CC_value = batt_cust_data.apple_0_5a_charger_current;
	} else if (BMT_status.charger_type == WIRELESS_CHARGER) {
		g_temp_input_CC_value = batt_cust_data.ac_charger_input_current;
		g_temp_CC_value = batt_cust_data.ac_charger_input_current;
#ifdef CONFIG_INPUT_EPACK
	} else if (BMT_status.charger_type == EPACK_CHARGER) {
		g_temp_input_CC_value = CHARGE_CURRENT_1000_00_MA;
		g_temp_CC_value = CHARGE_CURRENT_1000_00_MA;
#ifdef CONFIG_MTK_BOOT
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
		if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT ||
				get_boot_mode() == LOW_POWER_OFF_CHARGING_BOOT) {
			g_temp_input_CC_value = CHARGE_CURRENT_700_00_MA;
			g_temp_CC_value = CHARGE_CURRENT_700_00_MA;
		}
#endif
#ifdef CONFIG_LGE_PM_CHARGERLOGO
		if (get_boot_mode() == CHARGER_BOOT) {
			g_temp_input_CC_value = CHARGE_CURRENT_700_00_MA;
			g_temp_CC_value = CHARGE_CURRENT_700_00_MA;
		}
#endif
#endif
		if (epack_is_umsdev_connected()) {
			g_temp_input_CC_value = CHARGE_CURRENT_700_00_MA;
			g_temp_CC_value = CHARGE_CURRENT_700_00_MA;
		}
#endif
	} else {
		g_temp_input_CC_value = batt_cust_data.usb_charger_current;
		g_temp_CC_value = batt_cust_data.usb_charger_current;
	}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	if (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE) {
		g_temp_input_CC_value = batt_cust_data.ac_charger_current;
		g_temp_CC_value = batt_cust_data.ac_charger_current;
	}
#endif

#ifdef CONFIG_LGE_USB_TYPE_C
	select_charging_current_typec();
#endif

	if (!batt_cust_data.mtk_pump_express_plus_support)
		g_temp_CC_value = batt_cust_data.ac_charger_current;

#ifdef CONFIG_LGE_PM_CHARGER_CONTROLLER
	/* override charge setting refer to charger controller */
	chgctrl_select_charging_current((int*)&g_temp_input_CC_value,
					(int*)&g_temp_CC_value);
#endif

	if (!charging_enabled_check()) {
		g_temp_input_CC_value = 0;
		battery_log(BAT_LOG_CRTI, "IUSB charging disable\n");
	}
	if (!battery_charging_enabled_check()) {
		g_temp_CC_value = 0;
		battery_log(BAT_LOG_CRTI, "IBAT charging disable\n");
	}

#ifndef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	mtk_check_aicr_upper_bound();
#endif
}

static void select_charging_current_bcct(void)
{
#ifdef CONFIG_LGE_PM_CHARGER_CONTROLLER
	/* this will be done in charger-controller */
#else
	if ((BMT_status.charger_type == STANDARD_HOST) ||
	    (BMT_status.charger_type == NONSTANDARD_CHARGER)) {
		if (g_bcct_value < 100)
			g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 500)
			g_temp_input_CC_value = CHARGE_CURRENT_100_00_MA;
		else if (g_bcct_value < 800)
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		else if (g_bcct_value == 800)
			g_temp_input_CC_value = CHARGE_CURRENT_800_00_MA;
		else
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	} else if ((BMT_status.charger_type == STANDARD_CHARGER) ||
		   (BMT_status.charger_type == CHARGING_HOST)) {
		g_temp_input_CC_value = CHARGE_CURRENT_MAX;

		/* --------------------------------------------------- */
		/* set IOCHARGE */
		if (g_bcct_value < 550)
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 650)
			g_temp_CC_value = CHARGE_CURRENT_550_00_MA;
		else if (g_bcct_value < 750)
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		else if (g_bcct_value < 850)
			g_temp_CC_value = CHARGE_CURRENT_750_00_MA;
		else if (g_bcct_value < 950)
			g_temp_CC_value = CHARGE_CURRENT_850_00_MA;
		else if (g_bcct_value < 1050)
			g_temp_CC_value = CHARGE_CURRENT_950_00_MA;
		else if (g_bcct_value < 1150)
			g_temp_CC_value = CHARGE_CURRENT_1050_00_MA;
		else if (g_bcct_value < 1250)
			g_temp_CC_value = CHARGE_CURRENT_1150_00_MA;
		else if (g_bcct_value == 1250)
			g_temp_CC_value = CHARGE_CURRENT_1250_00_MA;
		else
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		/* --------------------------------------------------- */

	} else {
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	}

#ifndef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	mtk_check_aicr_upper_bound();
#endif
#endif
}

static bool is_slave_available(void)
{
	if (batt_cust_data.charger_configuration == CHARGER_CONFIG_SINGLE)
		return false;

	if (!g_parallel_flag)
		return false;

	if (g_temp_input_CC_value < CHARGE_CURRENT_1800_00_MA)
		return false;

	if (g_temp_CC_value <= CHARGE_CURRENT_1800_00_MA)
		return false;

	if (BMT_status.bat_charging_state == CHR_ERROR)
		return false;

	if (BMT_status.bat_charging_state == CHR_HOLD)
		return false;

	return true;
}

static void select_charging_current_slave(void)
{
	if (!is_slave_available()) {
		g_temp_CC_slave = 0;
		g_temp_input_CC_slave = 0;
		return;
	}

	g_temp_CC_slave = (g_temp_CC_value * portion_slave_fcc) / 100;
	g_temp_CC_slave -= (g_temp_CC_slave % step_slave_fcc);
	g_temp_CC_value -= g_temp_CC_slave;

	g_temp_input_CC_slave = (g_temp_input_CC_value * portion_slave_icl) / 100;
	g_temp_input_CC_slave -= (g_temp_input_CC_slave % step_slave_icl);
	if (batt_cust_data.charger_configuration == CHARGER_CONFIG_PARALLEL)
		g_temp_input_CC_value -= g_temp_input_CC_slave;
}

static void mtk_select_ichg_aicr(void)
{
	CHR_CURRENT_ENUM temp_input_CC_total;
	CHR_CURRENT_ENUM temp_CC_total;

	mutex_lock(&g_ichg_aicr_access_mutex);

	/* Set Ichg, AICR */
	if (get_usb_current_unlimited()) {
		if (batt_cust_data.ac_charger_input_current != 0)
			g_temp_input_CC_value = batt_cust_data.ac_charger_input_current;
		else
			g_temp_input_CC_value = batt_cust_data.ac_charger_current;

		g_temp_CC_value = batt_cust_data.ac_charger_current;
		battery_log(BAT_LOG_FULL,
			"USB_CURRENT_UNLIMITED, use batt_cust_data.ac_charger_current\n");
	} else {
		select_charging_current();
		if (g_bcct_flag == 1)
			select_charging_current_bcct(); // thermal mitigation charging current control by charger-controller

		select_charging_current_slave(); // Set Parallel Charging Current
	}

	temp_input_CC_total = g_temp_input_CC_value;
	if (batt_cust_data.charger_configuration == CHARGER_CONFIG_PARALLEL)
		temp_input_CC_total += g_temp_input_CC_slave;
	temp_CC_total = g_temp_CC_value + g_temp_CC_slave;

	battery_log(BAT_LOG_CRTI, "[BATTERY] CC mode current: "
				  "input=%dmA(%dmA, %dmA), charge=%dmA(%dmA, %dmA)\n",
			temp_input_CC_total / 100,
			g_temp_input_CC_value / 100, g_temp_input_CC_slave / 100,
			temp_CC_total / 100,
			g_temp_CC_value / 100, g_temp_CC_slave / 100);

	if (!batt_cust_data.mtk_pump_express_plus_support)
		goto set_ichg_aicr;

	/* check possible to draw more than 300mA from charger to enable pep */
	if (!mtk_is_pep_series_avaliable()) {
		if (mtk_pep20_get_is_enable()) {
			mtk_pep20_set_is_enable(false);
			if (mtk_pep20_get_is_connect())
				mtk_pep20_reset_ta_vchr();
		}
		if (mtk_pep_get_is_enable()) {
			mtk_pep_set_is_enable(false);
			if (mtk_pep_get_is_connect())
				mtk_pep_reset_ta_vchr();
		}
	} else {
		if (!mtk_pep20_get_is_enable()) {
			mtk_pep20_set_is_enable(true);
			mtk_pep20_set_to_check_chr_type(true);
		}
		if (!mtk_pep_get_is_enable()) {
			mtk_pep_set_is_enable(true);
			mtk_pep_set_to_check_chr_type(true);
		}

		/* PE+/PE+20 algorithm */
		if (mtk_is_pep_series_connect()) {
			mtk_pep20_start_algorithm();
			mtk_pep_start_algorithm();
		}
	}

set_ichg_aicr:
	if (g_temp_input_CC_value)
		battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,
				&g_temp_input_CC_value);

	if (g_temp_CC_value)
		battery_charging_control(CHARGING_CMD_SET_CURRENT,
				&g_temp_CC_value);

	if (g_temp_input_CC_slave)
		battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT_SLAVE,
				&g_temp_input_CC_slave);

	if (g_temp_CC_slave)
		battery_charging_control(CHARGING_CMD_SET_CURRENT_SLAVE,
				&g_temp_CC_slave);

	BMT_status.input_current_max = temp_input_CC_total;
	BMT_status.constant_charge_current_max = temp_CC_total;

	mutex_unlock(&g_ichg_aicr_access_mutex);
}

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_RT9460_NOISE_WA
#define NOISELESS_HEADROOM_MV 400
int rt9460_get_noise_wa_cv(unsigned int *cv)
{
	unsigned int voltage;
	unsigned int _cv;

	if (!BMT_status.bat_exist)
		return -ENODEV;

	voltage = battery_meter_get_battery_voltage(KAL_TRUE);

	_cv = voltage + NOISELESS_HEADROOM_MV;
	_cv = ((_cv + 50) / 100) * 100;	/* round up 100mV */
	_cv *= 1000;	/* unit change mV -> uV */

	if (_cv > BATTERY_VOLT_04_400000_V)
		_cv = BATTERY_VOLT_04_400000_V;
	if (_cv < BATTERY_VOLT_03_900000_V)
		_cv = BATTERY_VOLT_03_900000_V;

	*cv = _cv;

	return 0;
}
#endif

static void mtk_select_cv(void)
{
	int ret = 0;
	u32 dynamic_cv = 0;
	BATTERY_VOLTAGE_ENUM cv_voltage;
	BATTERY_VOLTAGE_ENUM cv_voltage_slave = 0;

	if (batt_cust_data.high_battery_voltage_support)
		cv_voltage = BATTERY_VOLT_04_400000_V;
	else
		cv_voltage = BATTERY_VOLT_04_200000_V;

	ret = mtk_get_dynamic_cv(&dynamic_cv);
	if (ret == 0) {
		cv_voltage = dynamic_cv * 1000;
		battery_log(BAT_LOG_FULL, "%s: set dynamic cv = %dmV\n",
			__func__, dynamic_cv);
	}

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_RT9460_NOISE_WA
	ret = rt9460_get_noise_wa_cv(&cv_voltage);
	if (ret == 0) {
		battery_log(BAT_LOG_CRTI, "%s: noise wa cv=%dmV\n",
				__func__, cv_voltage / 1000);
	}
#endif

#ifdef CONFIG_LGE_PM_CHARGER_CONTROLLER
	/* override charge setting refer to charger controller */
	chgctrl_select_cv((int*)&cv_voltage);
#endif
	cv_voltage_slave = cv_voltage;

	battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);
	battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE_SLAVE, &cv_voltage_slave);

	battery_log(BAT_LOG_FULL, "%s: Main Charger CV : %dmV, Slave Charger CV : %dmV\n",
				__func__, cv_voltage / 1000, cv_voltage_slave / 1000);

#if defined(CONFIG_MTK_HAFG_20)
	g_cv_voltage = cv_voltage;
#endif
	BMT_status.constant_charge_voltage_max = g_cv_voltage;
}

static void pchr_turn_on_charging(void)
{
	unsigned int charging_enable = true;
	unsigned int power_path_enable = true;
	unsigned int charging_enable_slave = true;

	if ((g_platform_boot_mode == META_BOOT) ||
			(g_platform_boot_mode == ADVMETA_BOOT)) {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] In meta or advanced meta mode, disable charging.\n");
		charging_enable = false;
	} else {
		/* HW initialization */
		battery_charging_control(CHARGING_CMD_INIT, NULL);
	}

	mtk_select_cv();

	mtk_select_ichg_aicr();

	if (!g_temp_CC_value)
		charging_enable = false;

	if (!g_temp_input_CC_value)
		power_path_enable = false;

	if (!g_temp_CC_slave || !g_temp_input_CC_slave)
		charging_enable_slave = false;

	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
	battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH, &power_path_enable);

	battery_charging_control(CHARGING_CMD_ENABLE_SLAVE, &charging_enable_slave);
	battery_charging_control(CHARGING_CMD_ENABLE_POWER_PATH_SLAVE, &charging_enable_slave);

	battery_log(BAT_LOG_FULL, "[BATTERY] %s: input %s, charge %s\n", __func__,
			power_path_enable ? "enabled" : "disabled",
			charging_enable ? "enabled" : "disabled");
}

static PMU_STATUS BAT_PreChargeModeAction(void)
{
#ifdef CONFIG_MTK_BIF_SUPPORT
	int ret = 0;
	bool bif_exist = false;
#endif
	unsigned int led_en = true;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Pre-CC mode charge, timer=%u on %u\n",
			BMT_status.PRE_charging_time, BMT_status.total_charging_time);

	BMT_status.bat_full = KAL_FALSE;
	BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

#ifdef CONFIG_MTK_BIF_SUPPORT
	/* If defined BIF but not BIF's battery, stop charging */
	ret = battery_charging_control(CHARGING_CMD_GET_BIF_IS_EXIST,
		&bif_exist);
	if (!bif_exist) {
		battery_log(BAT_LOG_CRTI,
			"%s: define BIF but no BIF battery, disable charging\n",
			__func__);
		BMT_status.bat_charging_state = CHR_ERROR;
		return PMU_STATUS_OK;
	}
#endif

	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	pchr_turn_on_charging();

	if (charging_full_check()) {
		BMT_status.bat_charging_state = CHR_BATFULL;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	} else if (BMT_status.bat_vol > V_PRE2CC_THRES) {
		BMT_status.bat_charging_state = CHR_CC;
	}

	return PMU_STATUS_OK;
}

static PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
	unsigned int led_en = true;

	battery_log(BAT_LOG_CRTI, "[BATTERY] CC mode charge, timer=%u on %u\n",
			BMT_status.CC_charging_time, BMT_status.total_charging_time);

	BMT_status.bat_full = KAL_FALSE;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time += BAT_TASK_PERIOD;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	pchr_turn_on_charging();

	if (charging_full_check() == KAL_TRUE) {
		BMT_status.bat_charging_state = CHR_BATFULL;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	}

	return PMU_STATUS_OK;
}

static PMU_STATUS BAT_TopOffModeAction(void)
{
	unsigned int led_en = true;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Top Off mode charge, timer=%d on %d !!\n\r",
			BMT_status.TOPOFF_charging_time, BMT_status.total_charging_time);

	BMT_status.bat_full = KAL_FALSE;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time += BAT_TASK_PERIOD;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	pchr_turn_on_charging();

	if (charging_full_check()) {
		BMT_status.bat_charging_state = CHR_BATFULL;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	}

	return PMU_STATUS_OK;
}

static PMU_STATUS BAT_BatteryFullAction(void)
{
	unsigned int led_en = false;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Battery full\n");

	/* enable 100% tracking if vfloat is high enough */
	if (g_cv_voltage >= BATTERY_VOLT_04_200000_V)
		BMT_status.bat_full = KAL_TRUE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

	battery_log(BAT_LOG_FULL, "[BATTERY] Turn off PWRSTAT LED\n");
	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	pchr_turn_on_charging();

	if (charging_full_check() == KAL_FALSE) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Re-charging\n");

		BMT_status.bat_full = KAL_FALSE;
		BMT_status.bat_in_recharging_state = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_CC;
#ifndef CONFIG_MTK_HAFG_20
		battery_meter_reset();
#endif
		if (batt_cust_data.mtk_pump_express_plus_support) {
			mtk_pep20_set_to_check_chr_type(true);
			mtk_pep_set_to_check_chr_type(true);
		}
		g_enable_dynamic_cv = true;
	}

	return PMU_STATUS_OK;
}

static PMU_STATUS BAT_BatteryHoldAction(void)
{
	unsigned int led_en = true;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Charge Suspend\n");

	BMT_status.bat_full = KAL_FALSE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

static PMU_STATUS BAT_BatteryStatusFailAction(void)
{
	unsigned int led_en = false;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Input Suspend\n");

	BMT_status.bat_full = KAL_FALSE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

	battery_log(BAT_LOG_FULL, "[BATTERY] Turn off PWRSTAT LED\n");
	battery_charging_control(CHARGING_CMD_SET_PWRSTAT_LED_EN, &led_en);

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

static void mt_battery_charging_check_enable(void)
{
	signed int state = BMT_status.bat_charging_state;

	/* if suspended, try to recharge */
	if (state == CHR_ERROR || state == CHR_HOLD)
		state = CHR_PRE;

	if (!battery_charging_enabled_check())
		state = CHR_HOLD;
	if (!charging_enabled_check())
		state = CHR_ERROR;

	/* update next state */
	BMT_status.bat_charging_state = state;
}

void mt_battery_charging_algorithm(void)
{
	battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

	if (!g_parallel_init)
		detect_extchg_slave();

	/* check charging enabled here for fast response */
	mt_battery_charging_check_enable();

	/* check PE+ is connected */
	mtk_pep_series_check_charger();

	switch (BMT_status.bat_charging_state) {
	case CHR_PRE :		/* Default State */
		BAT_PreChargeModeAction();
		break;

	case CHR_CC :		/* Constant Current Charging */
		BAT_ConstantCurrentModeAction();
		break;

	case CHR_TOP_OFF :	/* Constant Voltage Charging : Not Used */
		BAT_TopOffModeAction();
		break;

	case CHR_BATFULL:	/* End of Charging */
		BAT_BatteryFullAction();
		break;

	case CHR_HOLD:		/* Charging Suspended */
		BAT_BatteryHoldAction();
		break;

	case CHR_ERROR:		/* Input Suspended */
		BAT_BatteryStatusFailAction();
		break;

	default:
		battery_log(BAT_LOG_CRTI, "[BATTERY] invalid charging state\n");
		BMT_status.bat_charging_state = CHR_PRE;
		break;
	}

	battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
	battery_charging_control(CHARGING_CMD_DUMP_REGISTER_SLAVE, NULL);
}

