/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>

#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/switch.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>

#include <mt-plat/charging.h>
#include <mach/mt_charging.h>
#include <mt_gpio.h>
#include <mach/gpio_const.h>

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
#include "charging_hw_external_charger.h"
#endif

#include "bq25898s_reg.h"

enum bq2589x_part_no {
	BQ25898  = 0x00,
	BQ25898S = 0x01,
	BQ25898D = 0x02,
};


struct bq2589x_config {
	bool enable_auto_dpdm;

	int charge_voltage;
	int charge_current;

	int iindpm_threshold;
	int vindpm_threshold;

	bool enable_term;
	int term_current;

	bool use_absolute_vindpm;
};


struct bq2589x {
	struct device *dev;
	struct i2c_client *client;
	enum bq2589x_part_no part_no;
	int revision;

	bool prechg;
	struct bq2589x_config cfg;
	struct work_struct irq_work;

	int ibat;
	int vreg;
	int rsoc;
	struct power_supply *batt_psy;
	struct power_supply psy;

/* debugfs */
	struct dentry *debugfs;
	u32 debug_addr;
};

struct bq2589x *g_bq2589x;

bool bq25898s_charger_en=0;


static DEFINE_MUTEX(bq2589x_i2c_lock);

static int bq2589x_read_byte(struct bq2589x *bq, u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&bq2589x_i2c_lock);
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI,"[bq25898s] %s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
		mutex_unlock(&bq2589x_i2c_lock);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&bq2589x_i2c_lock);

	return 0;
}

static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&bq2589x_i2c_lock);
	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI,"[bq25898s] %s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
		mutex_unlock(&bq2589x_i2c_lock);
		return ret;
	}

	mutex_unlock(&bq2589x_i2c_lock);
	return ret;
}

static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;


	ret = bq2589x_read_byte(bq, &tmp, reg);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2589x_write_byte(bq, reg, tmp);
}

static int bq2589x_enable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ25898S_CHG_ENABLE << BQ25898S_CHG_CONFIG_SHIFT;

	battery_log(BAT_LOG_CRTI, "[bq2589x] %s \n", __func__);

	ret = bq2589x_update_bits(bq, BQ25898S_REG_03, BQ25898S_CHG_CONFIG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_charger);

static int bq2589x_disable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ25898S_CHG_DISABLE << BQ25898S_CHG_CONFIG_SHIFT;

	battery_log(BAT_LOG_CRTI, "[bq2589x] %s \n", __func__);

	ret = bq2589x_update_bits(bq, BQ25898S_REG_03, BQ25898S_CHG_CONFIG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_charger);

static int bq2589x_enable_Q4(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ25898S_Q4_ENABLE << BQ25898S_Q4_SHIFT;

	battery_log(BAT_LOG_CRTI, "[bq2589x] %s \n", __func__);

	ret = bq2589x_update_bits(bq, BQ25898S_REG_09, BQ25898S_Q4_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_Q4);

static int bq2589x_disable_Q4(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ25898S_Q4_DISABLE << BQ25898S_Q4_SHIFT;

	battery_log(BAT_LOG_CRTI, "[bq2589x] %s \n", __func__);

	ret = bq2589x_update_bits(bq, BQ25898S_REG_09, BQ25898S_Q4_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_Q4);

static int bq2589x_enable_term(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	battery_log(BAT_LOG_CRTI, "[bq2589x] %s \n", __func__);

	if (enable)
		val = BQ25898S_TERM_ENABLE << BQ25898S_EN_TERM_SHIFT;
	else
		val = BQ25898S_TERM_DISABLE << BQ25898S_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ25898S_REG_07, BQ25898S_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_term);

int bq2589x_adc_start(struct bq2589x *bq, bool oneshot)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_02);
	if (ret < 0) {
		dev_err(bq->dev, "%s failed to read register 0x02:%d\n", __func__, ret);
		return ret;
	}

	if (((val & BQ25898S_CONV_RATE_MASK) >> BQ25898S_CONV_RATE_SHIFT)
		       == BQ25898S_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/

	battery_log(BAT_LOG_CRTI, "[bq2589x] %s \n", __func__);

	if (oneshot)
		//ADC Conversion Not Active
		ret = bq2589x_update_bits(bq, BQ25898S_REG_02, BQ25898S_CONV_START_MASK,
				BQ25898S_CONV_START << BQ25898S_CONV_START_SHIFT);
	else
		//ADC Conversion Active (Continuous)
		ret = bq2589x_update_bits(bq, BQ25898S_REG_02, BQ25898S_CONV_RATE_MASK,
				BQ25898S_ADC_CONTINUE_ENABLE << BQ25898S_CONV_RATE_SHIFT);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_start);

int bq2589x_adc_stop(struct bq2589x *bq)
{
	battery_log(BAT_LOG_CRTI, "[bq2589x] %s \n", __func__);
	return bq2589x_update_bits(bq, BQ25898S_REG_02, BQ25898S_CONV_RATE_MASK,
			BQ25898S_ADC_CONTINUE_DISABLE << BQ25898S_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_adc_stop);

int bq2589x_adc_read_battery_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_0E);
	if (ret < 0) {
		dev_err(bq->dev, "read battery voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ25898S_BATV_BASE
			+ ((val & BQ25898S_BATV_MASK) >> BQ25898S_BATV_SHIFT) * BQ25898S_BATV_LSB ;
		battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,volt);
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_battery_volt);

int bq2589x_adc_read_sys_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_0F);
	if (ret < 0) {
		dev_err(bq->dev, "read system voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ25898S_SYSV_BASE
			+ ((val & BQ25898S_SYSV_MASK) >> BQ25898S_SYSV_SHIFT) * BQ25898S_SYSV_LSB ;
		battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,volt);
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt);

int bq2589x_adc_read_vbus_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_11);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ25898S_VBUSV_BASE
			+ ((val & BQ25898S_VBUSV_MASK) >> BQ25898S_VBUSV_SHIFT) * BQ25898S_VBUSV_LSB ;
		battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,volt);
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_vbus_volt);

int bq2589x_adc_read_charge_current(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_12);
	if (ret < 0) {
		dev_err(bq->dev, "read charge current failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(BQ25898S_ICHGR_BASE
			+ ((val & BQ25898S_ICHGR_MASK) >> BQ25898S_ICHGR_SHIFT) * BQ25898S_ICHGR_LSB) ;
		battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,volt);
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_charge_current);

int bq2589x_set_chargecurrent(struct bq2589x *bq, int curr)
{
	u8 ichg;

	ichg = (curr - BQ25898S_ICHG_BASE)/BQ25898S_ICHG_LSB;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,curr);

	return bq2589x_update_bits(bq, BQ25898S_REG_04, BQ25898S_ICHG_MASK, ichg << BQ25898S_ICHG_SHIFT);

}
EXPORT_SYMBOL_GPL(bq2589x_set_chargecurrent);

int bq2589x_set_ibat(int curr)
{
	struct bq2589x *bq = g_bq2589x;
	bq2589x_set_chargecurrent(bq, curr);
	return 0;
}

int bq2589x_set_term_current(struct bq2589x *bq, int curr)
{
	u8 iterm;

	iterm = curr / BQ25898S_ITERM_LSB;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,curr);

	return bq2589x_update_bits(bq, BQ25898S_REG_05, BQ25898S_ITERM_MASK, iterm << BQ25898S_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_term_current);

int bq2589x_set_prechg_current(struct bq2589x *bq, int curr)
{
	u8 iprechg;

	iprechg = (curr - BQ25898S_IPRECHG_BASE) / BQ25898S_IPRECHG_LSB;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,curr);

	return bq2589x_update_bits(bq, BQ25898S_REG_05, BQ25898S_IPRECHG_MASK, iprechg << BQ25898S_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_prechg_current);

int bq2589x_set_chargevoltage(struct bq2589x *bq, int volt)
{
	u8 val;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,volt);
	val = (volt - BQ25898S_VREG_BASE)/BQ25898S_VREG_LSB;

	return bq2589x_update_bits(bq, BQ25898S_REG_06, BQ25898S_VREG_MASK, val << BQ25898S_VREG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_chargevoltage);

int bq2589x_set_recharge_voltage(struct bq2589x *bq, int offset)
{
	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,offset);

	return bq2589x_update_bits(bq, BQ25898S_REG_06, BQ25898S_VRECHG_MASK, offset << BQ25898S_VRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_recharge_voltage);

int bq2589x_set_input_volt_limit(struct bq2589x *bq, int volt)
{
	u8 val;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,volt);
	val = (volt - BQ25898S_VINDPM_BASE) / BQ25898S_VINDPM_LSB;

	return bq2589x_update_bits(bq, BQ25898S_REG_0D, BQ25898S_VINDPM_MASK, val << BQ25898S_VINDPM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_volt_limit);

int bq2589x_set_input_current_limit(struct bq2589x *bq, int curr)
{
	u8 val;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,curr);
	val = (curr - BQ25898S_IINLIM_BASE) / BQ25898S_IINLIM_LSB;

	return bq2589x_update_bits(bq, BQ25898S_REG_00, BQ25898S_IINLIM_MASK, val << BQ25898S_IINLIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_current_limit);

int bq2589x_set_vindpm_offset(struct bq2589x *bq, int offset)
{
	u8 val;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,offset);

	if (offset == 400)
		val = BQ25898S_VINDPMOS_400MV;
	else
		val = BQ25898S_VINDPMOS_600MV;

	return bq2589x_update_bits(bq, BQ25898S_REG_01, BQ25898S_VINDPMOS_MASK, val << BQ25898S_VINDPMOS_SHIFT);

}
EXPORT_SYMBOL_GPL(bq2589x_set_vindpm_offset);

int bq2589x_get_charging_status(struct bq2589x *bq)
{
	u8 read_reg;
	int ret;
	int chrg_stat;

	ret = bq2589x_read_byte(bq, &read_reg, BQ25898S_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "%s Failed to read register 0x0b:%d\n", __func__, ret);
		return ret;
	}

	chrg_stat = ((read_reg & BQ25898S_CHRG_STAT_MASK) >> BQ25898S_CHRG_STAT_SHIFT);

	pr_info("[bq2589x] %s: REG_0B : 0x%02X CHRG_STAT = %d\n", __func__, read_reg, chrg_stat);

	return chrg_stat;
}
EXPORT_SYMBOL_GPL(bq2589x_get_charging_status);

int bq2589x_set_watchdog_timer(struct bq2589x *bq, u8 timeout)
{

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,timeout);

	return bq2589x_update_bits(bq, BQ25898S_REG_07, BQ25898S_WDT_MASK,
			(u8)((timeout - BQ25898S_WDT_BASE) / BQ25898S_WDT_LSB) << BQ25898S_WDT_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_watchdog_timer);

int bq2589x_disable_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ25898S_WDT_DISABLE << BQ25898S_WDT_SHIFT;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	return bq2589x_update_bits(bq, BQ25898S_REG_07, BQ25898S_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_disable_watchdog_timer);

int bq2589x_reset_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ25898S_WDT_RESET << BQ25898S_WDT_RESET_SHIFT;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	return bq2589x_update_bits(bq, BQ25898S_REG_03, BQ25898S_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_reset_watchdog_timer);

int bq2589x_reset_chip(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ25898S_RESET << BQ25898S_RESET_SHIFT;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	ret = bq2589x_update_bits(bq, BQ25898S_REG_14, BQ25898S_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_reset_chip);

int bq2589x_enter_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ25898S_HIZ_ENABLE << BQ25898S_ENHIZ_SHIFT;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	return bq2589x_update_bits(bq, BQ25898S_REG_00, BQ25898S_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_enter_hiz_mode);

int bq2589x_exit_hiz_mode(struct bq2589x *bq)
{

	u8 val = BQ25898S_HIZ_DISABLE << BQ25898S_ENHIZ_SHIFT;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	return bq2589x_update_bits(bq, BQ25898S_REG_00, BQ25898S_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_exit_hiz_mode);

int bq2589x_get_hiz_mode(struct bq2589x *bq, u8 *state)
{
	u8 val;
	int ret;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_00);
	if (ret)
		return ret;
	*state = (val & BQ25898S_ENHIZ_MASK) >> BQ25898S_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_get_hiz_mode);

static int bq2589x_enable_auto_dpdm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,enable);

	if (enable)
		val = BQ25898S_AUTO_DPDM_ENABLE << BQ25898S_AUTO_DPDM_EN_SHIFT;
	else
		val = BQ25898S_AUTO_DPDM_DISABLE << BQ25898S_AUTO_DPDM_EN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ25898S_REG_02, BQ25898S_AUTO_DPDM_EN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_auto_dpdm);

static int bq2589x_set_absolute_vindpm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : %d \n",__func__,enable);

	if (enable)
		val = BQ25898S_FORCE_VINDPM_ENABLE << BQ25898S_FORCE_VINDPM_SHIFT;
	else
		val = BQ25898S_FORCE_VINDPM_DISABLE << BQ25898S_FORCE_VINDPM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ25898S_REG_0D, BQ25898S_FORCE_VINDPM_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_set_absolute_vindpm);

int bq2589x_dump_register(void)
{
	u8 addr;
	u8 val;
	int ret = 0;
	unsigned char reg[0x14] = { 0 };

	for (addr = 0x0; addr < 0x14; addr++) {
		ret = bq2589x_read_byte(g_bq2589x, &val, addr);
		if (ret < 0) {
			battery_log(BAT_LOG_CRTI, "[bq2589x_reg@]read fail\n");
			return ret;
		} else{
			reg[addr]=val;
		}
	}

/*	Reduce register debug log
	battery_log(BAT_LOG_CRTI,"[bq2589x_reg@][0x00]=0x%02x ,[0x01]=0x%02x ,[0x02]=0x%02x ,[0x03]=0x%02x ,[0x04]=0x%02x\n",reg[0],reg[1],reg[2],reg[3],reg[4]);
	battery_log(BAT_LOG_CRTI,"[bq2589x_reg@][0x05]=0x%02x ,[0x06]=0x%02x ,[0x07]=0x%02x ,[0x08]=0x%02x ,[0x09]=0x%02x\n",reg[5],reg[6],reg[7],reg[8],reg[9]);
	battery_log(BAT_LOG_CRTI,"[bq2589x_reg@][0x0b]=0x%02x ,[0x0c]=0x%02x ,[0x0d]=0x%02x ,[0x0e]=0x%02x ,[0x0f]=0x%02x\n",reg[0xb],reg[0xc],reg[0xd],reg[0xe],reg[0xf]);
	battery_log(BAT_LOG_CRTI,"[bq2589x_reg@][0x10]=0x%02x ,[0x11]=0x%02x ,[0x12]=0x%02x ,[0x13]=0x%02x\n",reg[0x10],reg[0x11],reg[0x12],reg[0x13]);
*/
	battery_log(BAT_LOG_CRTI,"[DUMP@][0x00]0x%02x,[0x02]0x%02x,[0x03]0x%02x,[0x04]0x%02x,[0x05]0x%02x\n",reg[0],reg[2],reg[3],reg[4],reg[5]);
	battery_log(BAT_LOG_CRTI,"[DUMP@][0x06]0x%02x,[0x07]0x%02x,[0x0b]0x%02x,[0x0c]0x%02x,[0x0e]0x%02x,[0x12]0x%02x\n",reg[6],reg[7],reg[0xb],reg[0xc],reg[0xe],reg[0x12]);

	return 0;
}

static enum power_supply_property bq2589x_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
};

static int bq2589x_get_vbus_gd(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_11);
	if (ret)
		return ret;

	val &= BQ25898S_VBUS_GD_MASK;
	val >>= BQ25898S_VBUS_GD_SHIFT;

	return val;
}

static int bq2589x_status(struct bq2589x *bq)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;

	if (!bq2589x_get_vbus_gd(bq))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	switch (bq2589x_get_charging_status(bq)) {
	case 0:
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case 1:
	case 2:
		status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case 3:
		status = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		break;
	}

	return status;
}

static int bq2589x_present(struct bq2589x *bq)
{
	if (bq2589x_get_vbus_gd(bq) > 0)
		return 1;

	return 0;
}

static int bq2589x_get_pg_stat(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_0B);
	if (ret)
		return ret;

	val &= BQ25898S_PG_STAT_MASK;
	val >>= BQ25898S_PG_STAT_SHIFT;

	return val;
}

static int bq2589x_get_iterm(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_05);
	if (ret)
		return ret;

	val &= BQ25898S_ITERM_MASK;

	return (val + 1) * 64;
}

static int bq2589x_get_ichg(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_04);
	if (ret)
		return ret;

	val &= BQ25898S_ICHG_MASK;

	return val * 64;
}

static int bq2589x_get_ilimit(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_00);
	if (ret)
		return ret;

	val &= BQ25898S_IINLIM_MASK;

	return (val * 50) + 100;
}

static int bq2589x_get_vreg(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_06);
	if (ret)
		return ret;

	val &= BQ25898S_VREG_MASK;
	val >>= BQ25898S_VREG_SHIFT;

	return val * 16 + 3840;
}

static int bq2589x_get_en_hiz(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_00);
	if (ret)
		return ret;

	val &= BQ25898S_ENHIZ_MASK;
	val >>= BQ25898S_ENHIZ_SHIFT;

	return val;
}

static int bq2589x_get_en_ilim(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_00);
	if (ret)
		return ret;

	val &= BQ25898S_ENILIM_MASK;
	val >>= BQ25898S_ENILIM_SHIFT;

	return val;
}

static int bq2589x_get_chg_enable(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_03);
	if (ret)
		return ret;

	val &= BQ25898S_CHG_CONFIG_MASK;
	val >>= BQ25898S_CHG_CONFIG_SHIFT;

	return val;
}

static int bq2589x_get_auto_dpdm_enable(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_02);
	if (ret)
		return ret;

	val &= BQ25898S_AUTO_DPDM_EN_MASK;
	val >>= BQ25898S_AUTO_DPDM_EN_SHIFT;

	return val;
}

static int bq2589x_get_force_dpdm_enable(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_02);
	if (ret)
		return ret;

	val &= BQ25898S_FORCE_DPDM_MASK;
	val >>= BQ25898S_FORCE_DPDM_SHIFT;

	return val;
}

static int bq2589x_get_charging_term_enable(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_07);
	if (ret)
		return ret;

	val &= BQ25898S_EN_TERM_MASK;
	val >>= BQ25898S_EN_TERM_SHIFT;

	return val;
}

static int bq2589x_get_stat_dis_enable(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_07);
	if (ret)
		return ret;

	val &= BQ25898S_STAT_DIS_MASK;
	val >>= BQ25898S_STAT_DIS_SHIFT;

	return val;
}

static int bq2589x_get_WATCHDOG_Timer(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_07);
	if (ret)
		return ret;

	val &= BQ25898S_WDT_MASK;
	val >>= BQ25898S_WDT_SHIFT;

	/* 0 = Disable watchdog timer
	   1 = 40s (default)
	   2 = 80s
	   3 = 160s
	*/

	return val;
}

static int bq2589x_get_charging_safety_timer_enable(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ25898S_REG_07);
	if (ret)
		return ret;

	val &= BQ25898S_EN_TIMER_MASK;
	val >>= BQ25898S_EN_TIMER_SHIFT;

	return val;
}

static int bq2589x_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct bq2589x *bq = container_of(psy, struct bq2589x, psy);
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2589x_status(bq);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq2589x_present(bq);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq2589x_get_pg_stat(bq);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = bq2589x_get_iterm(bq);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = bq2589x_get_ichg(bq) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = bq2589x_get_ilimit(bq) * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = bq2589x_get_vreg(bq) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq2589x_get_en_hiz(bq);
		if (val->intval)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = bq2589x_get_chg_enable(bq);
		break;
	case POWER_SUPPLY_PROP_CHARGER_DUMP:
		val->intval = bq2589x_dump_register();
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int bq2589x_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct bq2589x *bq = container_of(psy, struct bq2589x, psy);
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		bq->ibat = val->intval;
		bq2589x_set_ibat(bq->ibat / 1000);
		battery_log(BAT_LOG_CRTI, "Set ibat = %d\n", bq->ibat / 1000);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		rc = bq2589x_set_input_current_limit(bq, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		bq->vreg = val->intval;
		rc = bq2589x_set_chargevoltage(bq, bq->vreg / 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (val->intval)
			bq2589x_exit_hiz_mode(bq);
		else
			bq2589x_enter_hiz_mode(bq);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		if (val->intval)
			bq2589x_adapter_in_handler();
		else
			bq2589x_adapter_out_handler();
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

int bq2589x_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static ssize_t bq2589x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "Charger");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(g_bq2589x, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,"Reg[0x%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}


static DEVICE_ATTR(registers, S_IRUGO, bq2589x_show_registers, NULL);

static struct attribute *bq2589x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};

static int bq2589x_parse_dt(struct device *dev, struct bq2589x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	//bq->cfg.enable_auto_dpdm = of_property_read_bool(np, "ti,bq2589x,enable-auto-dpdm");
	//bq->cfg.enable_term = of_property_read_bool(np, "ti,bq2589x,enable-termination");
	//bq->cfg.use_absolute_vindpm = of_property_read_bool(np, "ti,bq2589x,use-absolute-vindpm");

	ret = of_property_read_u32(np, "ti,bq2589x,charge-voltage",&bq->cfg.charge_voltage);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current",&bq->cfg.charge_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,term-current",&bq->cfg.term_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,input-current-limit",&bq->cfg.iindpm_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,input-voltage-limit",&bq->cfg.vindpm_threshold);
	if (ret)
		return ret;
	return 0;
}

static int bq2589x_detect_device(struct bq2589x *bq)
{
	int ret;
	u8 data;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	ret = bq2589x_read_byte(bq, &data, BQ25898S_REG_14);
	if (ret == 0) {
		bq->part_no = (data & BQ25898S_PN_MASK) >> BQ25898S_PN_SHIFT;
		bq->revision = (data & BQ25898S_DEV_REV_MASK) >> BQ25898S_DEV_REV_SHIFT;
	}

	return ret;
}

// defined but not used
#if 0
static int bq2589x_read_batt_rsoc(struct bq2589x *bq)
{
	union power_supply_propval ret = {0,};

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	if (!bq->batt_psy)
		bq->batt_psy = power_supply_get_by_name("battery");

	if (bq->batt_psy) {
		bq->batt_psy->get_property(bq->batt_psy,POWER_SUPPLY_PROP_CAPACITY,&ret);
		return ret.intval;
	}
	else {
		return 50;
	}
}
#endif

static int bq2589x_adjust_absolute_vindpm(struct bq2589x *bq)
{
	u16 vbus_volt;
	u16 vindpm_volt;
	int ret;

	vbus_volt = bq2589x_adc_read_vbus_volt(bq);

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s: %d\n",__func__,vbus_volt);

	if (vbus_volt < 0){
		dev_err(bq->dev, "%s:Failed to read vbus voltage:%d\n", __func__, ret);
		return ret;
	}
	if (vbus_volt < 6000)
		vindpm_volt = vbus_volt - 600;
	else
		vindpm_volt = vbus_volt - 1200;
	ret = bq2589x_set_input_volt_limit(bq, vindpm_volt); /* REG0D [6~0] */
	if (ret < 0) {
		dev_err(bq->dev, "%s:Set absolute vindpm threshold %d Failed:%d\n", __func__, vindpm_volt, ret);
		return ret;
	}
	else
		battery_log(BAT_LOG_CRTI, "%s:Set absolute vindpm threshold %d successfully\n", __func__, vindpm_volt);

	return 0;
}

static int bq2589x_charger_gpio_init(struct bq2589x *bq)
{

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	// GPIO setting
	mt_set_gpio_mode(GPIO9|0x80000000, GPIO_MODE_GPIO);
	mt_set_gpio_pull_enable(GPIO9|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO9|0x80000000, GPIO_PULL_UP);
	mt_set_gpio_dir(GPIO9|0x80000000, GPIO_DIR_IN);
	return 0;
}


static int bq2589x_init_device(struct bq2589x *bq)
{
	int ret;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);

	ret = bq2589x_disable_watchdog_timer(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable watchdog timer:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_enable_auto_dpdm(bq, false);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable auto dpdm:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_enable_term(bq, true);
	//ret = bq2589x_enable_term(bq, bq->cfg.enable_term);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to enable/disable termination:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_absolute_vindpm(bq,true);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to enable absolute vindpm:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_disable_charger(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable charger:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_enable_Q4(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to bq2589x_enable_Q4:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_adc_start(bq,true); //ADC Conversion Not Active
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to start ADC:%d\n", __func__, ret);
	}

	ret = bq2589x_set_chargevoltage(bq, bq->cfg.charge_voltage);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set charge voltage:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_chargecurrent(bq, bq->cfg.charge_current);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set charge current:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_term_current(bq, bq->cfg.term_current);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set termination current:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_input_current_limit(bq, bq->cfg.iindpm_threshold);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set input current limit:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_recharge_voltage(bq, BQ25898S_VRECHG_200MV);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set 200mV recharge voltage:%d\n", __func__, ret);
		return ret;
	}

	return ret;
}


#if 0
int bq2589x_set_charge_profile(struct bq2589x *bq)
{
	int ret;


	battery_log(BAT_LOG_CRTI,"[bq2589x_set_charge_profile]\n");

	ret = bq2589x_set_chargevoltage(bq, bq->cfg.charge_voltage); /* In dtsi VBATREG 4400mV REG06 [BIT 2~7] */
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set charge voltage:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_chargecurrent(bq, bq->cfg.charge_current); /* In dtsi ICHG 1300mA REG04 [Bit 5 ~ 0]  */
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set charge current:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_term_current(bq, bq->cfg.term_current); /* In dtsi TERM current 128mA REG05 [3~0] */
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set termination current:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_input_current_limit(bq, bq->cfg.iindpm_threshold); /* In dtsi ILIMIT 1300mA REG00 [5~0]*/
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set input current limit:%d\n", __func__, ret);
		return ret;
	}

	return bq2589x_adjust_absolute_vindpm(bq);
}
#endif

void bq2589x_adapter_in_handler(void)
{
	struct bq2589x *bq = g_bq2589x;
	int ret;
//	int vbat; //not use

	if(bq25898s_charger_en==1){
		battery_log(BAT_LOG_CRTI, "%s already enabled!!!\n", __func__);
		return ;
	}

	if (!bq) {
		battery_log(BAT_LOG_CRTI, "BQ25898S driver not loaded!");
		return;
	}

	ret = bq2589x_adc_start(bq,false); //ADC Conversion Active(continuous)
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to start ADC:%d\n", __func__, ret);
	}

	//ret = bq2589x_set_charge_profile(bq);
	ret = bq2589x_adjust_absolute_vindpm(bq);
	if (ret)
		return;

#if (0)
	vbat = bq2589x_adc_read_battery_volt(bq);
	if (vbat < 0){
		dev_err(bq->dev, "Failed to read battery voltage");
		return;
	}
	else if (vbat < 3500){
		bq->prechg = true;
		return;
	}

	/* check if battery is near full, if so, no need to turn on slave charge */
	/* Need to tuning : Over 95% SOC, stop charing by slave charger */
	bq->rsoc = bq2589x_read_batt_rsoc(bq);
	if (bq->rsoc > 95 ) {
		battery_log(BAT_LOG_CRTI, "%s:RSOC=%d, no need start slavce charger\n", __func__, bq->rsoc);
		return;
	}
#endif
	bq2589x_set_ibat(500);
	ret = bq2589x_enable_charger(bq);
	bq2589x_set_ibat(bq->ibat/1000);
	/* happend peck charging current when charging enable */
	/* After set ibat(500), charging enable and reset ibat(1300) */
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to enable charging:%d\n", __func__, ret);
		return ;
	}
	else {
		battery_log(BAT_LOG_CRTI, "%s:slave charge start charging\n", __func__);
	}

	bq25898s_charger_en = 1;

}

void bq2589x_adapter_out_handler(void)
{
	struct bq2589x *bq = g_bq2589x;
	int ret;

	if(bq25898s_charger_en==0){
		battery_log(BAT_LOG_CRTI, "%s already disabled!!!\n", __func__);
		return ;
	}

	battery_log(BAT_LOG_CRTI,"[bq2589x_adapter_out_handler]\n");

	if (!bq) {
		battery_log(BAT_LOG_CRTI,"BQ25898S driver not loaded!\n");
		return;
	}

	bq2589x_set_ibat(0);
//	bq2589x_write_byte(bq, BQ25898S_REG_04, 0x00);

	ret = bq2589x_disable_charger(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable charger:%d\n", __func__, ret);
	} else {
		battery_log(BAT_LOG_CRTI, "%s:slave charge stopped\n", __func__);
		ret = bq2589x_adc_start(bq,true); //ADC Conversion Not Active
		if (ret < 0) {
			dev_err(bq->dev, "%s:Failed to start ADC:%d\n", __func__, ret);
		}
	}

	bq25898s_charger_en=0;

}

static void bq2589x_charger_irq_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	u8 charge_status = 0;
	int ret;

	msleep(5);

	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ25898S_REG_0B);
	if (ret){
		battery_log(BAT_LOG_CRTI,"[bq2589x] %s: irq i2c read error\n", __func__);
		return;
	}

	ret = bq2589x_read_byte(bq, &fault, BQ25898S_REG_0C);
	if (ret){
		battery_log(BAT_LOG_CRTI,"[bq2589x] %s: irq i2c read error\n", __func__);
		return;

}

	charge_status = (status & BQ25898S_CHRG_STAT_MASK) >> BQ25898S_CHRG_STAT_SHIFT;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s : bq2589x_charger_irq_workfunc %d \n",__func__,charge_status);

	if (charge_status == BQ25898S_CHRG_STAT_CHGDONE){
		battery_log(BAT_LOG_CRTI, "%s:charge done!\n", __func__);
		bq2589x_dump_register();
		//bq2589x_adapter_out_handler();
	}

#if (0)
	if (charge_status == BQ25898S_CHRG_STAT_IDLE)
		battery_log(BAT_LOG_CRTI, "%s:not charging\n", __func__);
	else if (charge_status == BQ25898S_CHRG_STAT_PRECHG)
		battery_log(BAT_LOG_CRTI, "%s:precharging\n", __func__);
	else if (charge_status == BQ25898S_CHRG_STAT_FASTCHG)
		battery_log(BAT_LOG_CRTI, "%s:fast charging\n", __func__);
	else if (charge_status == BQ25898S_CHRG_STAT_CHGDONE){
		battery_log(BAT_LOG_CRTI, "%s:charge done!\n", __func__);
		bq2589x_disable_charger(bq);
	}
#endif
	if (fault)
		battery_log(BAT_LOG_CRTI, "%s:charge fault:%02x\n", __func__,fault);
}


static irqreturn_t bq2589x_charger_interrupt(int irq, void *data)
{
	struct bq2589x *bq = data;

	schedule_work(&bq->irq_work);
	return IRQ_HANDLED;
}

static int is_parallel_slave(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;

	return of_property_read_bool(node, "lge,parallel-charger");
}

#define GPIO_IRQ    9
static int bq2589x_main_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2589x *bq;
	int irqn;

	int ret;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s \n",__func__);
	bq = devm_kzalloc(&client->dev, sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);

	ret = bq2589x_detect_device(bq);
	if (!ret && bq->part_no == BQ25898S) {
		battery_log(BAT_LOG_CRTI, "%s: charger device bq25898S detected, revision:%d\n", __func__, bq->revision);
	} else {
		battery_log(BAT_LOG_CRTI, "%s: no bq25898S charger device found:%d\n", __func__, ret);
		return -ENODEV;
	}

	bq->batt_psy = power_supply_get_by_name("battery");

	bq2589x_charger_gpio_init(bq);

	g_bq2589x = bq;

	if (client->dev.of_node)
		bq2589x_parse_dt(&client->dev, bq);

	ret = bq2589x_init_device(bq);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_0;
	}

/*(
	ret = gpio_request(GPIO_IRQ, "bq2589x irq pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, GPIO_IRQ);
		goto err_0;
	}
	gpio_direction_input(GPIO_IRQ);
*/
	irqn = gpio_to_irq(GPIO_IRQ);
	if (irqn < 0) {
		dev_err(bq->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		ret = irqn;
		goto err_1;
	}
	client->irq = irqn;

	INIT_WORK(&bq->irq_work, bq2589x_charger_irq_workfunc);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_irq;
	}

	ret = request_irq(client->irq, bq2589x_charger_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "bq2589x_charger1_irq", bq);
	if (ret) {
		dev_err(bq->dev, "%s:Request IRQ %d failed: %d\n", __func__, client->irq, ret);
		goto err_irq;
	} else {
		battery_log(BAT_LOG_CRTI, "%s:irq = %d\n", __func__, client->irq);
	}

	return 0;

err_irq:
	cancel_work_sync(&bq->irq_work);
err_1:
	gpio_free(GPIO_IRQ);
err_0:
	g_bq2589x = NULL;
	return ret;
}

static int debugfs_get_data(void *data, u64 *val)
{
	struct bq2589x *chip = data;
	int rc;
	u8 temp;

	rc = bq2589x_read_byte(chip, &temp, chip->debug_addr);
	if (rc)
		return -EAGAIN;

	*val = temp;
	battery_log(BAT_LOG_FULL, "%s: [bq2589x_read_byte] addr : 0x%02x, data : 0x%02x\n", __func__, chip->debug_addr, temp);

	return 0;
}

static int debugfs_set_data(void *data, u64 val)
{
	struct bq2589x *chip = data;
	int rc;
	u8 temp;

	temp = (u8)val;
	rc = bq2589x_write_byte(chip, temp, chip->debug_addr);
	if (rc)
		return -EAGAIN;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(data_debugfs_ops,
	debugfs_get_data, debugfs_set_data, "0x%02llx\n");


const static int regs_to_dump[] = {
	BQ25898S_REG_00,
	BQ25898S_REG_01,
	BQ25898S_REG_02,
	BQ25898S_REG_03,
	BQ25898S_REG_04,
	BQ25898S_REG_05,
	BQ25898S_REG_06,
	BQ25898S_REG_07,
	BQ25898S_REG_08,
	BQ25898S_REG_09,
	BQ25898S_REG_0A,
	BQ25898S_REG_0B,
	BQ25898S_REG_0C,
	BQ25898S_REG_0D,
	BQ25898S_REG_0E,
	BQ25898S_REG_0F,
	BQ25898S_REG_11,
	BQ25898S_REG_12,
	BQ25898S_REG_13,
	BQ25898S_REG_14,
};

static int dump_debugfs_show(struct seq_file *m, void *start)
{
	struct bq2589x *bq = m->private;
	u8 data;
	int rc;
	int i;

	for (i = 0; i < ARRAY_SIZE(regs_to_dump); i++) {
		rc = bq2589x_read_byte(bq, &data, regs_to_dump[i]);
		if (rc) {
			seq_printf(m, "0x%02x=error\n", regs_to_dump[i]);
		}

		seq_printf(m, "0x%02x=0x%02x\n", regs_to_dump[i], data);
	}

	return 0;
}

static int dump_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2589x *bq = inode->i_private;

	return single_open(file, dump_debugfs_show, bq);
}

static const struct file_operations dump_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= dump_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int info_debugfs_show(struct seq_file *m, void *start)
{
	struct bq2589x *bq = m->private;

	seq_printf(m, "--- Enable ---\n");
	seq_printf(m, "Charger EN = %d\n", bq2589x_get_chg_enable(bq));
	seq_printf(m, "HIZ EN = %d\n", bq2589x_get_en_hiz(bq));
	seq_printf(m, "ILIM EN = %d\n", bq2589x_get_en_ilim(bq));
	seq_printf(m, "Force DPDM EN = %d\n", bq2589x_get_force_dpdm_enable(bq));
	seq_printf(m, "AUTO DPDM EN= %d\n", bq2589x_get_auto_dpdm_enable(bq));
	seq_printf(m, "Charging Termination EN= %d\n", bq2589x_get_charging_term_enable(bq));
	seq_printf(m, "STAT pin Disable = %d\n", bq2589x_get_stat_dis_enable(bq));
	seq_printf(m, "charging Safety Timer EN = %d\n", bq2589x_get_charging_safety_timer_enable(bq));
	seq_printf(m, "\n");
	seq_printf(m, "--- Charge Mode ---\n");
	seq_printf(m, "VREG = %d mV\n", bq2589x_get_vreg(bq));
	seq_printf(m, "Ilimit = %d mA\n", bq2589x_get_ilimit(bq));
	seq_printf(m, "ICHG = %d mA\n", bq2589x_get_ichg(bq));
	seq_printf(m, "ITERM = %d mA\n", bq2589x_get_iterm(bq));
	seq_printf(m, "CHRG_STAT = %d (0 : Not charging, 1 : Pre charge, 2: Fast charging, 3: charging term done)\n", bq2589x_status(bq));
	seq_printf(m, "PG_STAT = %d\n", bq2589x_get_pg_stat(bq));
	seq_printf(m, "Watchdog Timer= %d\n", bq2589x_get_WATCHDOG_Timer(bq));
	seq_printf(m, "\n");

	return 0;
}

static int info_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2589x *bq = inode->i_private;

	return single_open(file, info_debugfs_show, bq);
}

static const struct file_operations info_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= info_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int create_debugfs_entries(struct bq2589x *chip)
{
	struct dentry *ent;

	chip->debugfs = debugfs_create_dir("bq2589x", NULL);
	if (!chip->debugfs) {
		dev_err(chip->dev, "failed to create debugfs\n");
		return -ENODEV;
	}

	ent = debugfs_create_x32("addr", S_IFREG | S_IWUSR | S_IRUGO,
		chip->debugfs, &chip->debug_addr);
	if (!ent)
		dev_err(chip->dev, "failed to create addr debugfs\n");

	ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
		chip->debugfs, chip, &data_debugfs_ops);
	if (!ent)
		dev_err(chip->dev, "failed to create data debugfs\n");

	ent = debugfs_create_file("dump", S_IFREG | S_IRUGO,
		chip->debugfs, chip, &dump_debugfs_ops);
	if (!ent)
		dev_err(chip->dev, "failed to create dump debugfs\n");

	ent = debugfs_create_file("info", S_IFREG | S_IRUGO,
		chip->debugfs, chip, &info_debugfs_ops);
	if (!ent)
		dev_err(chip->dev, "failed to create info debugfs\n");

	return 0;
}

static int bq2589x_parallel_slave_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2589x *bq;
	int ret;

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s Start\n",__func__);
	bq = devm_kzalloc(&client->dev, sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);

	ret = bq2589x_detect_device(bq);
	if (!ret && bq->part_no == BQ25898S) {
		battery_log(BAT_LOG_CRTI, "%s: charger device bq25898S detected, revision:%d\n", __func__, bq->revision);
	} else {
		battery_log(BAT_LOG_CRTI, "%s: no bq25898S charger device found:%d\n", __func__, ret);
		return -ENODEV;
	}

	bq->psy.name = "usb-parallel";
	bq->psy.type = POWER_SUPPLY_TYPE_USB_PARALLEL;
	bq->psy.properties = bq2589x_properties;
	bq->psy.num_properties = ARRAY_SIZE(bq2589x_properties);
	bq->psy.get_property = bq2589x_get_property;
	bq->psy.set_property = bq2589x_set_property;
	bq->psy.property_is_writeable = bq2589x_property_is_writeable;

	bq2589x_charger_gpio_init(bq);

	g_bq2589x = bq;

	if (client->dev.of_node)
		bq2589x_parse_dt(&client->dev, bq);

	ret = bq2589x_init_device(bq);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_0;
	}

	ret = power_supply_register(bq->dev, &bq->psy);
	if (ret) {
		dev_err(bq->dev, "failed to register power_supply, rc=%d\n", ret);
		return ret;
	}

	create_debugfs_entries(bq);

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	chr_control_register_slave(&bq->psy);
#endif

	battery_log(BAT_LOG_CRTI,"[bq2589x] %s End\n",__func__);

	return 0;

err_0:
	g_bq2589x = NULL;
	return ret;
}

static int bq2589x_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	if (is_parallel_slave(client))
		return bq2589x_parallel_slave_probe(client, id);
	else
		return bq2589x_main_charger_probe(client, id);
}

static int bq2589x_remove(struct i2c_client *client)
{
	struct bq2589x *bq = g_bq2589x;

	battery_log(BAT_LOG_CRTI, "%s: remove\n", __func__);

	power_supply_unregister(&bq->psy);

	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, bq);
	}

	kfree(bq);

	return 0;
}

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
extern int detect_vbus_ok(void);
#endif
static void bq2589x_shutdown(struct i2c_client *client)
{
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	struct bq2589x *bq = g_bq2589x;
#endif
	battery_log(BAT_LOG_CRTI, "%s: shutdown\n", __func__);

	bq2589x_adapter_out_handler();

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	if (!detect_vbus_ok()) {
		bq2589x_adc_stop(bq);
		bq2589x_disable_Q4(bq);
	}
#endif
	return;

/*
	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);
	cancel_work_sync(&bq->irq_work);

	free_irq(bq->client->irq, bq);
	gpio_free(GPIO_IRQ);
	g_bq2589x = NULL;
*/

}

static const struct of_device_id bq2589x_charger_match_table[] = {
	{.compatible = "ti,bq25898s",},
	{},
};


static const struct i2c_device_id bq2589x_charger_id[] = {
	{"bq25898s", 0},
	{},
};

static struct i2c_driver bq2589x_charger_driver = {
	.probe = bq2589x_charger_probe,
	.remove = bq2589x_remove,
	.shutdown = bq2589x_shutdown,
	.driver		= {
			.name = "bq25898s",
			.of_match_table = bq2589x_charger_match_table,
	},
	.id_table = bq2589x_charger_id,
};

static int __init bq25898s_init(void)
{
	battery_log(BAT_LOG_CRTI, "[bq25898s_init] init start\n");

	if (i2c_add_driver(&bq2589x_charger_driver) != 0) {
		battery_log(BAT_LOG_CRTI,
			    "[bq25898s_init] failed to register bq25898s i2c driver.\n");
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[bq25898s_init] Success to register bq25898s i2c driver.\n");
	}

	return 0;
}

static void __exit bq25898s_exit(void)
{
	i2c_del_driver(&bq2589x_charger_driver);
}

module_init(bq25898s_init);
module_exit(bq25898s_exit);

/*
module_i2c_driver(bq2589x_charger_driver);

MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
*/
