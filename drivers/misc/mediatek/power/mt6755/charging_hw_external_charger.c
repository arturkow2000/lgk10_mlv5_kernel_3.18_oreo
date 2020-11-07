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

#include <linux/types.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>

#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <mt-plat/mt_gpio.h>

#ifdef CONFIG_MACH_LGE
#include <soc/mediatek/lge/board_lge.h>
#endif

#include <mach/mt_sleep.h>
#include <mach/mt_spm.h>
#ifdef CONFIG_INPUT_EPACK
#include <linux/input/epack.h>
#endif
/* ============================================================ // */
/* define */
/* ============================================================ // */
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define STATUS_FAIL -2

#define PATTERN_HIGH	500000	/* uA */
#define PATTERN_LOW	100000	/* uA */
#define PATTERN_ZERO	0	/* uA */
/* ============================================================ */
/* global variable */
/* ============================================================ */
kal_bool chargin_hw_init_done = KAL_FALSE;

/* ============================================================ */
/* internal variable */
/* ============================================================ */
static struct power_supply *extchg;
static struct power_supply *extchg_slave;
static struct regulator *otgreg;
static kal_bool charging_type_det_done = KAL_TRUE;
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;

static int g_charging_enabled = -EINVAL;
static int g_battery_charging_enabled = -EINVAL;
static int g_charging_current = -EINVAL;
static int g_charging_current_limit = -EINVAL;
static int g_charging_setting_changed = -EINVAL;
static int g_otg_enabled = -EINVAL;

/* ============================================================ */
/* extern variable */
/* ============================================================ */

/* ============================================================ */
/* extern function */
/* ============================================================ */

/* ============================================================ */
/* internal function : power_supply */
/* ============================================================ */
static int get_property(enum power_supply_property prop, int *data)
{
	union power_supply_propval val;
	int rc = 0;

	rc = extchg->get_property(extchg, prop, &val);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to get property %d\n", prop);
		*(int *)data = 0;
		return rc;
	}

	*(int *)data = val.intval;
	battery_log(BAT_LOG_FULL, "[CHARGER] get property %d as %d\n", prop, val.intval);

	return rc;
}

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_BQ25898S_SLAVE_SUPPORT
static int get_property_slave(enum power_supply_property prop, int *data)
{
	union power_supply_propval val;
	int rc = 0;

	rc = extchg_slave->get_property(extchg_slave, prop, &val);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to get property as slave %d\n", prop);
		*(int *)data = 0;
		return rc;
	}

	*(int *)data = val.intval;
	battery_log(BAT_LOG_FULL, "[CHARGER] get property as slave %d to %d\n", prop, val.intval);

	return rc;
}
#endif

static int get_property_string(enum power_supply_property prop, const char **data)
{
	union power_supply_propval val;
	int rc = 0;

	rc = extchg->get_property(extchg, prop, &val);
	if (rc) {
		*data = NULL;
		return rc;
	}

	*data = val.strval;

	return rc;
}

static int get_property_string_slave(enum power_supply_property prop, const char **data)
{
#if 1
	return 1;
#else
	union power_supply_propval val;
	int rc = 0;

	rc = extchg_slave->get_property(extchg_slave, prop, &val);
	if (rc) {
		*data = NULL;
		return rc;
	}

	*data = val.strval;

	return rc;
#endif
}

static int set_property(enum power_supply_property prop, int data)
{
	union power_supply_propval val;
	int rc = 0;

	val.intval = data;
	rc = extchg->set_property(extchg, prop, &val);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to set property %d\n", prop);
		return rc;
	}

	battery_log(BAT_LOG_FULL, "[CHARGER] set property %d to %d\n", prop, data);

	return rc;
}

static int set_property_slave(enum power_supply_property prop, int data)
{
#if 1
	return 1;
#else
	union power_supply_propval val;
	int rc = 0;

	val.intval = data;
	rc = extchg_slave->set_property(extchg_slave, prop, &val);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to set property as slave %d\n", prop);
		return rc;
	}

	battery_log(BAT_LOG_FULL, "[CHARGER] set property as slave %d to %d\n", prop, data);

	return rc;
#endif
}

/* ============================================================ */
/* internal function : interface */
/* ============================================================ */
static int charging_hw_init(void *data)
{
	int status = STATUS_OK;
	static int hw_initialized;

	if (hw_initialized)
		return STATUS_OK;

	hw_initialized = 1;
	battery_log(BAT_LOG_FULL, "[CHARGER] initialized.\n");

	return status;
}

static int charging_dump_register(void *data)
{
	int status = STATUS_OK;
	int rc = 0;
	const char *buffer;

	rc = get_property_string(POWER_SUPPLY_PROP_CHARGER_DUMP, &buffer);

	if (rc < 0)
		return rc;

	if (buffer == NULL)
		return rc;

	return status;
}

static int charging_dump_register_slave(void *data)
{
	int status = STATUS_OK;
	int rc = 0;
	const char *buffer = NULL;

	rc = get_property_string_slave(POWER_SUPPLY_PROP_CHARGER_DUMP, &buffer);

	if (rc < 0)
		return rc;

	if (buffer == NULL)
		return rc;

	return status;
}

static int charging_enable(void *data)
{
	int status = STATUS_OK;
	int enable = 0;
	int rc = 0;

	enable = *(unsigned int *)(data);

#ifndef SWCHR_POWER_PATH
	/*
	 * Do not disable charging when battery disconnected.
	 * If power path / charging path is not separated,
	 * this cause unintentional powerdown
	 */
	if (!enable && pmic_get_register_value(PMIC_RGS_BATON_UNDET))
		return STATUS_OK;
#endif

	battery_log(BAT_LOG_CRTI, "[CHARGER] %s battery charging.\n", (enable ? "start" : "stop"));

	rc = set_property(POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, enable);
	if (rc) {
		battery_log(BAT_LOG_CRTI,
			    "[CHARGER] failed to %s battery charging.(%d)\n",
			    (enable ? "start" : "stop"), rc);
		return STATUS_UNSUPPORTED;
	}

	g_battery_charging_enabled = enable;

	/* clear charging setting */
	if (!g_battery_charging_enabled) {
		g_charging_current = 0;
		g_charging_current_limit = 0;
	}

	g_charging_setting_changed = 0;

	return status;
}

static int charging_enable_slave(void *data)
{
	int status = STATUS_OK;
	int enable = 0;
	int rc = 0;

	enable = *(unsigned int *)(data);

	battery_log(BAT_LOG_CRTI, "[CHARGER] %s battery charging as slave.\n",
		    (enable ? "start" : "stop"));

	rc = set_property_slave(POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, enable);
	if (rc) {
		battery_log(BAT_LOG_CRTI,
			    "[CHARGER] failed to %s battery charging as slave.(%d)\n",
			    (enable ? "start" : "stop"), rc);
		return STATUS_UNSUPPORTED;
	}

	return status;
}

static int charging_set_cv_voltage(void *data)
{
	int voltage = *(int *)(data);
	int rc = 0;

	battery_log(BAT_LOG_CRTI, "[CHARGER] Set CV Voltage to %duV\n", voltage);

	rc = set_property(POWER_SUPPLY_PROP_VOLTAGE_MAX, voltage);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] Set CV Voltage failed.(%d)\n", rc);
		return STATUS_UNSUPPORTED;
	}

	return STATUS_OK;
}

static int charging_set_cv_voltage_slave(void *data)
{
	int voltage = *(int *)(data);
	int rc = 0;

	battery_log(BAT_LOG_CRTI, "[CHARGER] Set CV Voltage to %duV as slave\n", voltage);

	rc = set_property_slave(POWER_SUPPLY_PROP_VOLTAGE_MAX, voltage);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] Set CV Voltage failed as slave.(%d)\n", rc);
		return STATUS_UNSUPPORTED;
	}

	return STATUS_OK;
}

static int charging_get_current(void *data)
{
	int main_cur = 0;
	int slave_cur = 0;
	int cur = 0;
	int enable = 0;
#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_BQ25898S_SLAVE_SUPPORT
	int slave_enable = 0;
#endif
	int rc = 0;

	if (!g_battery_charging_enabled) {
		*(int *)data = 0;
		return STATUS_OK;
	}

	rc = get_property(POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &enable);
	if (rc) {
		*(int *)data = 0;
		return rc;
	}

	if (!enable) {
		*(int *)data = 0;
		return STATUS_OK;
	}

	rc = get_property(POWER_SUPPLY_PROP_CURRENT_MAX, &cur);
	if (rc) {
		*(int *)data = 0;
		return rc;
	}

	main_cur = cur;

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_BQ25898S_SLAVE_SUPPORT
	if (extchg_slave) {
		rc = get_property_slave(POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &slave_enable);
		if (rc)
			slave_enable = 0;

		if (slave_enable) {
			rc = get_property_slave(POWER_SUPPLY_PROP_CURRENT_MAX, &slave_cur);
			if (rc)
				slave_cur = 0;

			cur = main_cur + slave_cur;
		}
	}
#endif
	battery_log(BAT_LOG_CRTI,
		    "[CHARGER] Total Charging Current to %dmA ( Main: %d, Slave: %d)\n", cur,
		    main_cur, slave_cur);

	/* match unit with CHR_CURRENT_ENUM */
	*(int *)data = cur * 100;

	return rc;
}

static int charging_set_current(void *data)
{
	int cur = *(int *)(data);
	int rc = 0;

	/* convert unit to uA */
	cur = cur * 10;

	battery_log(BAT_LOG_CRTI, "[CHARGER] Set Current to %duA\n", cur);

	rc = set_property(POWER_SUPPLY_PROP_CURRENT_MAX, cur);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] Set Current failed.(%d)\n", rc);
		return rc;
	}

	g_charging_current = cur;

	return rc;
}

static int charging_set_current_slave(void *data)
{
	int cur = *(int *)(data);
	int rc = 0;

	/* convert unit to uA */
	cur = cur * 10;

	battery_log(BAT_LOG_CRTI, "[CHARGER] Set Slave Charging Current to %duA\n", cur);

	rc = set_property_slave(POWER_SUPPLY_PROP_CURRENT_MAX, cur);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] Set Slave Current failed.(%d)\n", rc);
		return rc;
	}

	g_charging_current = cur;

	return rc;
}

static int charging_set_input_current(void *data)
{
	int cur = *(int *)(data);
	int rc = 0;

	/* convert unit to uA */
	cur = cur * 10;

	battery_log(BAT_LOG_CRTI, "[CHARGER] Set Current Limit to %duA\n", cur);

	rc = set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, cur);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] Set Current Limit failed.(%d)\n", rc);
		return rc;
	}

	g_charging_current_limit = cur;

	return rc;
}

static int charging_get_charging_status(void *data)
{
	int status;
	int rc = 0;

	rc = get_property(POWER_SUPPLY_PROP_STATUS, &status);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to get charging status.(%d)\n", rc);
		return rc;
	}

	if (status == POWER_SUPPLY_STATUS_FULL)
		*(int *)data = 1;
	else
		*(int *)data = 0;

	battery_log(BAT_LOG_CRTI, "[CHARGER] End of Charging : %d\n", *(int *)data);

	return rc;
}

static int charging_get_charging_status_slave(void *data)
{
#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_BQ25898S_SLAVE_SUPPORT
	int status;
	int rc = 0;

	rc = get_property_slave(POWER_SUPPLY_PROP_STATUS, &status);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to get charging status as slave.(%d)\n",
			    rc);
		return rc;
	}

	if (status == POWER_SUPPLY_STATUS_FULL)
		*(int *)data = 1;
	else
		*(int *)data = 0;

	battery_log(BAT_LOG_CRTI, "[CHARGER] End of Charging as Slave : %d\n", *(int *)data);

	return rc;
#else
	*(int *)data = 0;
	return 0;
#endif
}

static int charging_reset_watch_dog_timer(void *data)
{
	/* nothing to do */
	return 0;
}

static int charging_set_hv_threshold(void *data)
{
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	unsigned int vcdt_vth[] = {
		/* 50mV step for 0000b ~ 1000b */
		4200, 4250, 4300, 4350, 4400, 4450, 4500, 4550, 4600,
		/* 500mV step for 1001b ~ 1100b */
		6000, 6500, 7000, 7500,
		/* 1000mV step for 1101b ~ 1111b */
		8500, 9500, 10500,
	};
	unsigned int mv = *(unsigned int *)data;
	int i;

	for (i = 0; i < ARRAY_SIZE(vcdt_vth); i++) {
		if (vcdt_vth[i] >= mv)
			break;
	}
	if (i == ARRAY_SIZE(vcdt_vth))
		i = ARRAY_SIZE(vcdt_vth) - 1;

	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, i);
#endif

	return 0;
}

static int charging_get_hv_status(void *data)
{
	if (pmic_get_register_value(PMIC_RGS_VCDT_HV_DET))
		*(kal_bool *) data = KAL_TRUE;
	else
		*(kal_bool *) data = KAL_FALSE;

	return 0;
}

static int charging_get_battery_status(void *data)
{
	pmic_set_register_value(PMIC_BATON_TDET_EN, 1);
	pmic_set_register_value(PMIC_RG_BATON_EN, 1);

	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);

	return STATUS_OK;
}

static int charging_get_ex_charger_det_status(void)
{
	int status = 0;
	int rc = 0;

	rc = get_property(POWER_SUPPLY_PROP_ONLINE, &status);
	if (rc) {
		battery_log(BAT_LOG_CRTI,
			    "[CHARGER] failed to get VBUS from External charger IC.(%d)\n", rc);
		return status;
	}

	battery_log(BAT_LOG_FULL, "[CHARGER] VBUS (Ex-charger IC) OK = %d\n", status);

	return status;
}

static int charging_get_charger_det_status(void *data)
{

	if (pmic_get_register_value(PMIC_RGS_CHRDET) == 0) {
		*(kal_bool *) (data) = KAL_FALSE;

		/* Check VBUS by external charger ic */
		/* Prevent disconnection by abnormal VBUS drop */
		if (extchg_slave) {
			if (charging_get_ex_charger_det_status())
				*(kal_bool *) (data) = KAL_TRUE;
			else
				*(kal_bool *) (data) = KAL_FALSE;
		}
	} else {
		*(kal_bool *) (data) = KAL_TRUE;
	}

	return STATUS_OK;
}

static int charging_get_exchg_slave(void *data)
{
	unsigned int status = STATUS_OK;

#if 0
	rc = get_property_slave(POWER_SUPPLY_PROP_ONLINE, &status);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] Slave charger is not online\n");
		*(kal_bool *) (data) = KAL_FALSE;
		return status;
	}
#endif

	if (extchg_slave)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;

	return status;
}

/* extern int hw_charging_get_charger_type(void); */

static int charging_get_charger_type(void *data)
{
#if defined(CONFIG_POWER_EXT)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
	if (pmic_get_register_value(PMIC_RGS_CHRDET) == 0) {
		g_charger_type = CHARGER_UNKNOWN;
		*(CHARGER_TYPE *) (data) = g_charger_type;
		battery_log(BAT_LOG_CRTI, "[CHARGER] return %d!\n", g_charger_type);
		return STATUS_OK;
	}

	charging_type_det_done = KAL_FALSE;

	g_charger_type = hw_charging_get_charger_type();

	charging_type_det_done = KAL_TRUE;

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT)
	battery_log(BAT_LOG_CRTI, "%s First Charger_Type = %d\n", __func__, g_charger_type);
	if (g_charger_type == NONSTANDARD_CHARGER || g_charger_type == STANDARD_HOST) {
		g_charger_type = hw_charging_get_charger_type();
		battery_log(BAT_LOG_CRTI, "%s Second Charger_Type = %d\n", __func__,
			    g_charger_type);
	}
#endif

	*(CHARGER_TYPE *) (data) = g_charger_type;
#endif

	return STATUS_OK;
}

static int charging_get_is_pcm_timer_trigger(void *data)
{
	unsigned int status = STATUS_OK;

	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;

	battery_log(BAT_LOG_CRTI, "[CHARGER] slp_get_wake_reason=%d\n", slp_get_wake_reason());

	return status;
}

static int charging_set_platform_reset(void *data)
{
	battery_log(BAT_LOG_CRTI, "[CHARGER] charging_set_platform_reset\n");

	kernel_restart("battery service reboot system");

	return STATUS_OK;
}

static int charging_get_platform_boot_mode(void *data)
{
	*(unsigned int *)(data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "[CHARGER] get_boot_mode=%d\n", get_boot_mode());

	return STATUS_OK;
}

static int charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;

	battery_log(BAT_LOG_CRTI, "[CHARGER] charging_set_power_off\n");

	kernel_power_off();

	return status;
}

static int charging_get_power_source(void *data)
{
	unsigned int status = STATUS_OK;

	*(kal_bool *) data = KAL_FALSE;

	return status;
}

static int charging_get_csdac_full_flag(void *data)
{
	unsigned int status = STATUS_OK;

	*(kal_bool *) data = KAL_FALSE;

	return status;
}

static int charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

static int charging_diso_init(void *data)
{
	unsigned int status = STATUS_OK;
#if defined(MTK_DUAL_INPUT_CHARGER_SUPPORT)
	struct device_node *node;
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *) data;
	int ret;

	/* Initialization DISO Struct */
	pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;

	pDISO_data->diso_state.pre_otg_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vdc_state = DISO_OFFLINE;

	pDISO_data->chr_get_diso_state = KAL_FALSE;

	pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;

	/* Initial AuxADC IRQ */
	DISO_IRQ.vdc_measure_channel.number = AP_AUXADC_DISO_VDC_CHANNEL;
	DISO_IRQ.vusb_measure_channel.number = AP_AUXADC_DISO_VUSB_CHANNEL;
	DISO_IRQ.vdc_measure_channel.period = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vusb_measure_channel.period = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vdc_measure_channel.debounce = AUXADC_CHANNEL_DEBOUNCE;
	DISO_IRQ.vusb_measure_channel.debounce = AUXADC_CHANNEL_DEBOUNCE;

	/* use default threshold voltage, if use high voltage,maybe refine */
	DISO_IRQ.vusb_measure_channel.falling_threshold = VBUS_MIN_VOLTAGE / 1000;
	DISO_IRQ.vdc_measure_channel.falling_threshold = VDC_MIN_VOLTAGE / 1000;
	DISO_IRQ.vusb_measure_channel.rising_threshold = VBUS_MIN_VOLTAGE / 1000;
	DISO_IRQ.vdc_measure_channel.rising_threshold = VDC_MIN_VOLTAGE / 1000;

	node = of_find_compatible_node(NULL, NULL, "mediatek,AUXADC");
	if (!node) {
		battery_log(BAT_LOG_CRTI, "[diso_adc]: of_find_compatible_node failed!!\n");
	} else {
		pDISO_data->irq_line_number = irq_of_parse_and_map(node, 0);
		battery_log(BAT_LOG_FULL, "[diso_adc]: IRQ Number: 0x%x\n",
			    pDISO_data->irq_line_number);
	}

	mt_irq_set_sens(pDISO_data->irq_line_number, MT_EDGE_SENSITIVE);
	mt_irq_set_polarity(pDISO_data->irq_line_number, MT_POLARITY_LOW);

	ret = request_threaded_irq(pDISO_data->irq_line_number, diso_auxadc_irq_handler,
				   pDISO_data->irq_callback_func, IRQF_ONESHOT, "DISO_ADC_IRQ",
				   NULL);

	if (ret) {
		battery_log(BAT_LOG_CRTI, "[diso_adc]: request_irq failed.\n");
	} else {
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		battery_log(BAT_LOG_FULL, "[diso_adc]: diso_init success.\n");
	}

#if defined(MTK_DISCRETE_SWITCH) && defined(MTK_DSC_USE_EINT)
	battery_log(BAT_LOG_CRTI, "[diso_eint]vdc eint irq registitation\n");
	mt_eint_set_hw_debounce(CUST_EINT_VDC_NUM, CUST_EINT_VDC_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_VDC_NUM, CUST_EINTF_TRIGGER_LOW, vdc_eint_handler, 0);
	mt_eint_mask(CUST_EINT_VDC_NUM);
#endif
#endif

	return status;
}

static int charging_get_diso_state(void *data)
{
#if defined(MTK_DUAL_INPUT_CHARGER_SUPPORT)
	int diso_state = 0x0;
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *) data;

	_get_diso_interrupt_state();
	diso_state = g_diso_state;
	battery_log(BAT_LOG_FULL, "[do_chrdet_int_task] current diso state is %s!\n",
		    DISO_state_s[diso_state]);
	if (((diso_state >> 1) & 0x3) != 0x0) {
		switch (diso_state) {
		case USB_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
#ifdef MTK_DISCRETE_SWITCH
#ifdef MTK_DSC_USE_EINT
			mt_eint_unmask(CUST_EINT_VDC_NUM);
#else
			set_vdc_auxadc_irq(DISO_IRQ_ENABLE, 1);
#endif
#endif
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_RISING);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_USB:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_OTG:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			break;
		default:	/* OTG only also can trigger vcdt IRQ */
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			battery_log(BAT_LOG_FULL, " switch load vcdt irq triggerd by OTG Boost!\n");
			break;	/* OTG plugin no need battery sync action */
		}
	}

	if (DISO_ONLINE == pDISO_data->diso_state.cur_vdc_state)
		pDISO_data->hv_voltage = VDC_MAX_VOLTAGE;
	else
		pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;

	return 0;
#else
	return -ENOTSUPP;
#endif
}

static int charging_set_vindpm(void *data)
{
	/* should return 0 to support pe/pe+ */
	return 0;
}

static int charging_set_vbus_ovp_en(void *data)
{
	unsigned int e = *(unsigned int *)data;

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_set_register_value(PMIC_RG_VCDT_HV_EN, e);
#else
	pmic_set_register_value(MT6351_PMIC_RG_VCDT_HV_EN, e);
#endif

	return 0;
}

static int charging_get_bif_vbat(void *data)
{
	*(unsigned int *)(data) = 0;

	return -ENOTSUPP;
}

static int charging_set_chrind_ck_pdn(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int pwr_dn;

	pwr_dn = *(unsigned int *)data;

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_set_register_value(PMIC_CLK_DRV_CHRIND_CK_PDN, pwr_dn);
#else
	pmic_set_register_value(PMIC_RG_DRV_CHRIND_CK_PDN, pwr_dn);
#endif

	return status;
}

static int charging_sw_init(void *data)
{
	return 0;
}

static int charging_enable_safetytimer(void *data)
{
	return 0;
}

static int charging_set_hiz_swchr(void *data)
{
	return -ENOTSUPP;
}

static int charging_get_bif_tbat(void *data)
{
	*(unsigned int *)(data) = 0;

	return -ENOTSUPP;
}

/* For check PE+ 2.0 current patten */
static struct timespec ta20_ptime[13];
static int ta20_cptime[13][2];

static int dtime(int i)
{
	struct timespec time;

	time = timespec_sub(ta20_ptime[i], ta20_ptime[i - 1]);
	return time.tv_nsec / 1000000;
}

void pep_schedule_hrtimeout(u64 msec)
{
	ktime_t to = ms_to_ktime(msec);

	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_hrtimeout(&to, HRTIMER_MODE_REL);
}

/* PE+ 1.0 current patten */
static int charging_set_ta_current_pattern(void *data)
{
	unsigned int increase = *(unsigned int *)(data);
	unsigned int status = STATUS_OK;

	battery_log(BAT_LOG_CRTI, "[CHARGER] set_ta_curret_pattern %s\n",
		    increase ? "increase" : "decrease");

	pep_schedule_hrtimeout(50);

	set_property(POWER_SUPPLY_PROP_CURRENT_MAX, PATTERN_HIGH);

	if (increase) {
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
		pep_schedule_hrtimeout(85);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);
		pep_schedule_hrtimeout(85);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
		pep_schedule_hrtimeout(85);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);
		pep_schedule_hrtimeout(85);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
		pep_schedule_hrtimeout(85);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);
		pep_schedule_hrtimeout(281);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
		pep_schedule_hrtimeout(85);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);
		pep_schedule_hrtimeout(281);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
		pep_schedule_hrtimeout(85);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);
		pep_schedule_hrtimeout(281);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
		pep_schedule_hrtimeout(85);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);
		pep_schedule_hrtimeout(485);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
		pep_schedule_hrtimeout(50);
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);
		pep_schedule_hrtimeout(200);
	} else {
		set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
		set_property(POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, true);
		if (extchg_slave)
			set_property_slave(POWER_SUPPLY_PROP_CURRENT_MAX, PATTERN_ZERO);
	}

	return status;
}

/* PE+ 2.0 current patten */
static int charging_set_ta20_reset(void *data)
{
	set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
	set_property(POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, true);
	if (extchg_slave) {
		set_property_slave(POWER_SUPPLY_PROP_CURRENT_MAX, PATTERN_ZERO);
	}
/*	msleep(250); */
/*	set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH); */
	return STATUS_OK;
}

#define TA20_PEOFFTIME 45
#define TA20_PEONTIME 98	/* For High patten(charging) */

static int charging_set_ta20_current_pattern(void *data)
{
	int value;		/*9V patten=00111=0x07 ,8.5V patten=00110=0x06 */
	int i = 0;		/*bit 4~0 */
	int j = 0;		/*bit duty cycle check */
	int flag;
	CHR_VOLTAGE_ENUM chr_vol = *(CHR_VOLTAGE_ENUM *) data;

/*	usleep_range(1000, 1200); */
	value = (chr_vol - CHR_VOLT_05_500000_V) / CHR_VOLT_00_500000_V;

	set_property(POWER_SUPPLY_PROP_CURRENT_MAX, PATTERN_HIGH);

	battery_log(BAT_LOG_CRTI, "[%s] start\n", __func__);

	set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
	/* msleep(70); */
	pep_schedule_hrtimeout(70);

	battery_log(BAT_LOG_CRTI, "%s : value = %d\n", __func__, value);
	get_monotonic_boottime(&ta20_ptime[j++]);
	for (i = 4; i >= 0; i--) {
		flag = value & (1 << i);

		if (flag == 0) {
			battery_log(BAT_LOG_CRTI, "%s : bit: %d = Low\n", __func__, i);
			set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);
			/* usleep_range(TA20_PEOFFTIME * 1000, TA20_PEOFFTIME * 1000); */
			pep_schedule_hrtimeout(TA20_PEOFFTIME);

			get_monotonic_boottime(&ta20_ptime[j]);
			ta20_cptime[j][0] = TA20_PEOFFTIME;
			ta20_cptime[j][1] = dtime(j);
			if (ta20_cptime[j][1] < 30 || ta20_cptime[j][1] > 65) {
				battery_log(BAT_LOG_CRTI,
					    "%s : fail1: idx:%d target:%d actual:%d\n",
					    __func__, i, TA20_PEOFFTIME, ta20_cptime[j][1]);
				return STATUS_FAIL;
			}
			j++;
			set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
			/* usleep_range(TA20_PEONTIME * 1000, TA20_PEONTIME * 1000); */
			pep_schedule_hrtimeout(TA20_PEONTIME);

			get_monotonic_boottime(&ta20_ptime[j]);
			ta20_cptime[j][0] = TA20_PEONTIME;
			ta20_cptime[j][1] = dtime(j);
			if (ta20_cptime[j][1] < 88 || ta20_cptime[j][1] > 115) {
				battery_log(BAT_LOG_CRTI,
					    "%s : fail2: idx:%d target:%d actual:%d\n",
					    __func__, i, TA20_PEONTIME, ta20_cptime[j][1]);
				return STATUS_FAIL;
			}
			j++;

		} else {
			battery_log(BAT_LOG_CRTI, "%s : bit: %d = High\n", __func__, i);
			set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);
			/* usleep_range(TA20_PEONTIME * 1000, TA20_PEONTIME * 1000); */
			pep_schedule_hrtimeout(TA20_PEONTIME);

			get_monotonic_boottime(&ta20_ptime[j]);
			ta20_cptime[j][0] = TA20_PEONTIME;
			ta20_cptime[j][1] = dtime(j);
			if (ta20_cptime[j][1] < 88 || ta20_cptime[j][1] > 115) {
				battery_log(BAT_LOG_CRTI,
					    "%s : fail3: idx:%d target:%d actual:%d\n",
					    __func__, i, TA20_PEONTIME, ta20_cptime[j][1]);
				return STATUS_FAIL;
			}
			j++;
			set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
			/* usleep_range(TA20_PEOFFTIME * 1000, TA20_PEOFFTIME * 1000); */
			pep_schedule_hrtimeout(TA20_PEOFFTIME);

			get_monotonic_boottime(&ta20_ptime[j]);
			ta20_cptime[j][0] = TA20_PEOFFTIME;
			ta20_cptime[j][1] = dtime(j);
			if (ta20_cptime[j][1] < 30 || ta20_cptime[j][1] > 65) {
				battery_log(BAT_LOG_CRTI,
					    "%s : fail4: idx:%d target:%d actual:%d\n",
					    __func__, i, TA20_PEOFFTIME, ta20_cptime[j][1]);
				return STATUS_FAIL;
			}
			j++;
		}
	}

	set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);
	/* msleep(160); */
	pep_schedule_hrtimeout(160);
	get_monotonic_boottime(&ta20_ptime[j]);
	ta20_cptime[j][0] = 160;
	ta20_cptime[j][1] = dtime(j);
	if (ta20_cptime[j][1] < 150 || ta20_cptime[j][1] > 240) {
		battery_log(BAT_LOG_CRTI,
			    "%s : fail5: idx:%d target:%d actual:%d\n",
			    __func__, i, TA20_PEOFFTIME, ta20_cptime[j][1]);
		return STATUS_FAIL;
	}
	j++;

	set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_LOW);
	/* msleep(30); */
	pep_schedule_hrtimeout(30);
	set_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, PATTERN_HIGH);

	battery_log(BAT_LOG_CRTI,
		    "[%s] target: chr_vol:%d time:%d %d %d %d %d %d %d %d %d %d %d !!\n",
		    __func__, chr_vol, ta20_cptime[1][0], ta20_cptime[2][0], ta20_cptime[3][0],
		    ta20_cptime[4][0], ta20_cptime[5][0], ta20_cptime[6][0], ta20_cptime[7][0],
		    ta20_cptime[8][0], ta20_cptime[9][0], ta20_cptime[10][0], ta20_cptime[11][0]);

	battery_log(BAT_LOG_CRTI,
		    "[%s] actual: chr_vol:%d time:%d %d %d %d %d %d %d %d %d %d %d !!\n",
		    __func__, chr_vol, ta20_cptime[1][1], ta20_cptime[2][1], ta20_cptime[3][1],
		    ta20_cptime[4][1], ta20_cptime[5][1], ta20_cptime[6][1], ta20_cptime[7][1],
		    ta20_cptime[8][1], ta20_cptime[9][1], ta20_cptime[10][1], ta20_cptime[11][1]);

	battery_log(BAT_LOG_CRTI, "[%s] end\n", __func__);

	return STATUS_OK;
}

static int charging_set_dp(void *data)
{
	unsigned int status = STATUS_OK;
#ifndef CONFIG_POWER_EXT
	unsigned int en;

	en = *(int *)data;
#ifdef CONFIG_LGE_PM
	/* intentionally blocked for PEP.
	 *  PEP will not re-enable with blocked code.
	 */
#else
	hw_charging_enable_dp_voltage(en);
#endif
#endif
	return status;
}

static int charging_get_charger_temperature(void *data)
{
	return STATUS_UNSUPPORTED;
}

static int charging_set_boost_current_limit(void *data)
{
	int ret = 0;
	u32 current_limit = 0;

	current_limit = *((u32 *) data);
	/* TODO */

	return ret;
}

static int charging_enable_otg(void *data)
{
	int enable = *(int *)(data);
	int rc;

	if (!otgreg) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] otg boost not ready\n");
		return -ENXIO;
	}

	if (enable)
		rc = regulator_enable(otgreg);
	else
		rc = regulator_disable(otgreg);

	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to %s otg boost\n",
			    enable ? "enable" : "disable");
		return rc;
	}

	g_otg_enabled = enable;

	battery_log(BAT_LOG_CRTI, "[CHARGER] otg boost %s\n", enable ? "enabled" : "disabled");

	return rc;
}

static int charging_enable_power_path(void *data)
{
	int enable = 0;
	int rc = STATUS_OK;

	enable = *((unsigned int *)data);

	/* Do not disable power path when battery disconnected */
	if (!enable && pmic_get_register_value(PMIC_RGS_BATON_UNDET))
		return STATUS_OK;

	battery_log(BAT_LOG_CRTI, "[CHARGER] %s charging.\n", (enable ? "start" : "stop"));

	rc = set_property(POWER_SUPPLY_PROP_CHARGING_ENABLED, enable);
	if (rc) {
		battery_log(BAT_LOG_CRTI,
			    "[CHARGER] failed to %s charging.(%d)\n",
			    (enable ? "start" : "stop"), rc);
		return rc;
	}

	g_charging_enabled = enable;

	/* clear charging setting */
	if (!g_charging_enabled) {
		g_charging_current = 0;
		g_charging_current_limit = 0;
	}

	return rc;
}

static int charging_enable_power_path_slave(void *data)
{
	int enable = 0;
	int rc = STATUS_OK;

	enable = *((unsigned int *)data);

	/*
	 * Do not disable charging when battery disconnected
	 */
	if (!enable && pmic_get_register_value(PMIC_RGS_BATON_UNDET))
		return STATUS_OK;

	battery_log(BAT_LOG_CRTI, "[CHARGER] %s charging as slave\n", (enable ? "start" : "stop"));

	rc = set_property_slave(POWER_SUPPLY_PROP_CHARGING_ENABLED, enable);
	if (rc) {
		battery_log(BAT_LOG_CRTI,
			    "[CHARGER] failed to %s charging.(%d) as slave\n",
			    (enable ? "start" : "stop"), rc);
		return rc;
	}

	return rc;
}

static int charging_get_bif_is_exist(void *data)
{
	int bif_exist = 0;

	bif_exist = false;
	*(bool *) data = bif_exist;

	return 0;
}

static int charging_get_input_current(void *data)
{
	int rc = 0;
	int input_current;

	rc = get_property(POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, &input_current);
	if (rc)
		return rc;

	/* mA -> 10uA */
	*(int *)data = input_current * 100;

	return 0;
}

static int charging_enable_direct_charge(void *data)
{
	return -ENOTSUPP;
}

static int charging_get_is_power_path_enable(void *data)
{
	return -ENOTSUPP;
}

static int charging_get_ibus(void *data)
{
	return -ENOTSUPP;
}

static int charging_get_vbus(void *data)
{
	return -ENOTSUPP;
}

static int charging_reset_dc_watch_dog_timer(void *data)
{
	return -ENOTSUPP;
}

static int charging_run_aicl(void *data)
{
	return -ENOTSUPP;
}

int chr_control_register(struct power_supply *psy)
{
	char otgreg_name[30];

	if (!psy) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] invalid charger\n");
		return -EINVAL;
	}

	if (extchg) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] charger already registered\n");
		return -EPERM;
	}

	snprintf(otgreg_name, 30, "%s_otg_supply", psy->name);
	otgreg = regulator_get(NULL, otgreg_name);

	extchg = psy;
	chargin_hw_init_done = KAL_TRUE;

	battery_log(BAT_LOG_CRTI, "[CHARGER] %s registered\n", psy->name);

	return 0;
}
EXPORT_SYMBOL(chr_control_register);

int chr_control_register_slave(struct power_supply *psy)
{
	if (!psy) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] invalid slave charger\n");
		return -EINVAL;
	}

	if (extchg_slave) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] Slave charger already registered\n");
		return -EPERM;
	}

	extchg_slave = psy;

	battery_log(BAT_LOG_CRTI, "[CHARGER] %s registered as slave\n", psy->name);

	return 0;
}
EXPORT_SYMBOL(chr_control_register_slave);

/* this must be aligned with CHARGING_CTRL_CMD */
static int (*charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
	charging_hw_init,	/* CHARGING_CMD_INIT */
	    charging_dump_register,	/* CHARGING_CMD_DUMP_REGISTER */
	    charging_dump_register_slave,	/* CHARGING_CMD_DUMP_REGISTER_SLAVE */
	    charging_enable,	/* CHARGING_CMD_ENABLE */
	    charging_enable_slave,	/* CHARGING_CMD_ENABLE_SLAVE */
	    charging_set_cv_voltage,	/* CHARGING_CMD_SET_CV_VOLTAGE */
	    charging_set_cv_voltage_slave,	/* CHARGING_CMD_SET_CV_VOLTAGE_SLAVE */
	    charging_get_current,	/* CHARGING_CMD_GET_CURRENT */
	    charging_set_current,	/* CHARGING_CMD_SET_CURRENT */
	    charging_set_current_slave,	/* CHARGING_CMD_SET_CURRENT_SLAVE */
	    charging_set_input_current,	/* CHARGING_CMD_SET_INPUT_CURRENT */
	    charging_get_charging_status,	/* CHARGING_CMD_GET_CHARGING_STATUS */
	    charging_get_charging_status_slave,	/* CHARGING_CMD_GET_CHARGING_STATUS_SLAVE */
	    charging_reset_watch_dog_timer,	/* CHARGING_CMD_RESET_WATCH_DOG_TIMER */
	    charging_set_hv_threshold,	/* CHARGING_CMD_SET_HV_THRESHOLD */
	    charging_get_hv_status,	/* CHARGING_CMD_GET_HV_STATUS */
	    charging_get_battery_status,	/* CHARGING_CMD_GET_BATTERY_STATUS */
	    charging_get_charger_det_status,	/* CHARGING_CMD_GET_CHARGER_DET_STATUS */
	    charging_get_charger_type,	/* CHARGING_CMD_GET_CHARGER_TYPE */
	    charging_get_is_pcm_timer_trigger,	/* CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER */
	    charging_set_platform_reset,	/* CHARGING_CMD_SET_PLATFORM_RESET */
	    charging_get_platform_boot_mode,	/* CHARGING_CMD_GET_PLATFORM_BOOT_MODE */
	    charging_set_power_off,	/* CHARGING_CMD_SET_POWER_OFF */
	    charging_get_power_source,	/* CHARGING_CMD_GET_POWER_SOURCE */
	    charging_get_csdac_full_flag,	/* CHARGING_CMD_GET_CSDAC_FALL_FLAG */
	    charging_set_ta_current_pattern,	/* CHARGING_CMD_SET_TA_CURRENT_PATTERN */
	    charging_set_error_state,	/* CHARGING_CMD_SET_ERROR_STATE */
	    charging_diso_init,	/* CHARGING_CMD_DISO_INIT */
	    charging_get_diso_state,	/* CHARGING_CMD_GET_DISO_STATE */
	    charging_set_vindpm,	/* CHARGING_CMD_SET_VINDPM */
	    charging_set_vbus_ovp_en,	/* CHARGING_CMD_SET_VBUS_OVP_EN */
	    charging_get_bif_vbat,	/* CHARGING_CMD_GET_BIF_VBAT */
	    charging_set_chrind_ck_pdn,	/* CHARGING_CMD_SET_CHRIND_CK_PDN */
	    charging_sw_init,	/* CHARGING_CMD_SW_INIT */
	    charging_enable_safetytimer,	/* CHARGING_CMD_ENABLE_SAFETY_TIMER */
	    charging_set_hiz_swchr,	/* CHARGING_CMD_SET_HIZ_SWCHR */
	    charging_get_bif_tbat,	/* CHARGING_CMD_GET_BIF_TBAT */
	    charging_set_ta20_reset,	/* CHARGING_CMD_SET_TA20_RESET */
	    charging_set_ta20_current_pattern,	/* CHARGING_CMD_SET_TA20_CURRENT_PATTERN */
	    charging_set_dp,	/* CHARGING_CMD_SET_DP */
	    charging_get_charger_temperature,	/* CHARGING_CMD_GET_CHARGER_TEMPERATURE (not supported) */
	    charging_set_boost_current_limit,	/* CHARGING_CMD_SET_BOOST_CURRENT_LIMIT */
	    charging_enable_otg,	/* CHARGING_CMD_ENABLE_OTG */
	    charging_enable_power_path,	/* CHARGING_CMD_ENABLE_POWER_PATH */
	    charging_enable_power_path_slave,	/* CHARGING_CMD_ENABLE_POWER_PATH_SLAVE */
	    charging_get_exchg_slave,	/* CHARGING_CMD_GET_EXCHG_SLAVE */
	    charging_get_bif_is_exist,	/* CHARGING_CMD_GET_BIF_IS_EXIST */
	    charging_get_input_current,	/* CHARGING_CMD_GET_INPUT_CURRENT (not supported) */
	    charging_enable_direct_charge,	/* CHARGING_CMD_ENABLE_DIRECT_CHARGE */
	    charging_get_is_power_path_enable,	/* CHARGING_CMD_GET_IS_POWER_PATH_ENABLE */
	    NULL,		/* CHARGING_CMD_GET_IS_SAFETY_TIMER_ENABLE */
	    NULL,		/* CHARGING_CMD_SET_PWRSTAT_LED_EN */
	    charging_get_ibus,	/* CHARGING_CMD_GET_IBUS (not supported) */
	    charging_get_vbus,	/* CHARGING_CMD_GET_VBUS (not supported) */
	    charging_reset_dc_watch_dog_timer,	/* CHARGING_CMD_RESET_DC_WATCH_DOG_TIMER (not supported) */
	    charging_run_aicl,	/* CHARGING_CMD_RUN_AICL (not supported) */
	    NULL,		/* CHARGING_CMD_SET_IRCMP_RESISTOR, */
	    NULL,		/* CHARGING_CMD_SET_IRCMP_VOLT_CLAMP, */
};

/* below commands do not need external charger */
static CHARGING_CTRL_CMD internal_hw_cmd[] = {
	CHARGING_CMD_SET_HV_THRESHOLD,
	CHARGING_CMD_GET_HV_STATUS,
	CHARGING_CMD_GET_BATTERY_STATUS,
	CHARGING_CMD_GET_CHARGER_DET_STATUS,
	CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER,
	CHARGING_CMD_SET_PLATFORM_RESET,
	CHARGING_CMD_GET_PLATFORM_BOOT_MODE,
	CHARGING_CMD_SET_POWER_OFF,
	CHARGING_CMD_SW_INIT,
	CHARGING_CMD_GET_BIF_VBAT,
	CHARGING_CMD_GET_BIF_TBAT,
	CHARGING_CMD_GET_BIF_IS_EXIST,
};

static int is_internal_hw_cmd(CHARGING_CTRL_CMD cmd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(internal_hw_cmd); i++) {
		if (cmd == internal_hw_cmd[i])
			return 1;
	}

	return 0;
}

int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	if (is_internal_hw_cmd(cmd))
		goto run_cmd_chr_control;

	if (!extchg) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] charger not registered.\n");
		return STATUS_UNSUPPORTED;
	}

	if (cmd >= CHARGING_CMD_NUMBER)
		return STATUS_UNSUPPORTED;

	if (!charging_func[cmd])
		return STATUS_UNSUPPORTED;

run_cmd_chr_control:
	return charging_func[cmd] (data);
}
