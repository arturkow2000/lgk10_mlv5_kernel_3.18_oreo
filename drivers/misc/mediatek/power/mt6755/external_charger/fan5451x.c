/*
* fan5451x.c -- 3.2A Dual Input, Switch Mode Charger device driver
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
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
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

#include <mt-plat/charging.h>
#include <mach/mt_charging.h>
#include <mt_gpio.h>
#include <mach/gpio_const.h>

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
#include "charging_hw_external_charger.h"
#endif

#include "fan5451x.h"

typedef unsigned char BYTE;

#define IBUS_MIN_MA	100
#define IBUS_MAX_MA	3000
#define IBUS_STEP_MA	25

#define IOCHRG_MIN_MA	200
#define IOCHRG_MAX_MA	3200
#define IOCHRG_STEP_MA	50

#define VFLOAT_MIN_MV	3300
#define VFLOAT_MAX_MV	4720
#define VFLOAT_STEP_MV	10

static struct i2c_client *new_client = NULL;

static int init_gpio_number = (8 | 0x80000000);
static int en_gpio_number = (24 | 0x80000000);

struct fan5451x_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct fan5451x_info {
	struct i2c_client *i2c;
	struct device *dev;
	int irq;
	struct power_supply fan_psy;
	struct fan5451x_regulator otg_vreg;

	int iochrg;
	int vfloat;
	int ibus;

	int status;
	int suspend;
	int battery_charging_enabled;
	bool otg_boost_enabled;
};

static int fan5451x_read_reg(struct i2c_client *i2c, BYTE reg, BYTE *dest)
{
	int ret;
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}
	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int fan5451x_write_reg(struct i2c_client *i2c, BYTE reg, BYTE value)
{
	int ret;
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
	return ret;
}

static int fan5451x_update_reg(struct i2c_client *i2c, u8 reg, BYTE val, BYTE mask)
{
	int ret;
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret >= 0) {
		BYTE old_val = ret & 0xff;
		BYTE new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	return ret;
}

static int fan5451x_get_vbuspwr(struct fan5451x_info *info)
{
	u8 read_reg;

	fan5451x_read_reg(info->i2c, REG_STAT0, &read_reg);
	if (read_reg & STAT0_VBUSPWR)
		return 1;
	else
		return 0;
}

static int fan5451x_get_chgcmp(struct fan5451x_info *info)
{
	u8 read_reg_stat0;
	u8 read_reg_stat1;
	int chgon_ret;
	int topoff_ret;
	int done_ret;

	fan5451x_read_reg(info->i2c, REG_STAT0, &read_reg_stat0); /* status 0 stat bit */
	fan5451x_read_reg(info->i2c, REG_STAT1, &read_reg_stat1); /* status 1 chgcmp/tochg bit */

	chgon_ret = ((read_reg_stat0 & STAT0_STAT) >> STAT0_STAT_SHIFT);
	topoff_ret = ((read_reg_stat1 & STAT1_TOCHG) >> STAT1_TOCHG_SHIFT);
	done_ret = ((read_reg_stat1 & STAT1_CHGCMP) >> STAT1_CHGCMP_SHIFT);

	pr_info("[fan] %s: STATUS0[0x%02X] STATUS1[0x%02X] CHGON = %d TOPOFF = %d DONE = %d\n",
				__func__, read_reg_stat0, read_reg_stat1, chgon_ret, topoff_ret, done_ret);

	if (done_ret)
		return 1;
	else if (chgon_ret || topoff_ret)
		return 2;
	else
		return 0;
}

static int fan5451x_get_status(struct fan5451x_info *info)
{
	int status = 0;

	switch (fan5451x_get_chgcmp(info)) {
	case 0:
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case 1:
		status = POWER_SUPPLY_STATUS_FULL;
		break;
	case 2:
		status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	default:
		break;
	}

	return status;
}

static int fan5451x_set_iochrg(struct fan5451x_info *info, int ma)
{
	int reg_val;

	if (ma < IOCHRG_MIN_MA)
		ma = IOCHRG_MIN_MA;
	else if(ma > IOCHRG_MAX_MA)
		ma = IOCHRG_MAX_MA;

	reg_val = (ma - IOCHRG_MIN_MA)/IOCHRG_STEP_MA;
	info->iochrg = reg_val*IOCHRG_STEP_MA + IOCHRG_MIN_MA;
	return fan5451x_write_reg(info->i2c, REG_IOCHRG, reg_val);
}

static int fan5451x_set_vfloat(struct fan5451x_info *info, int mv)
{
	int reg_val;

	if (mv < VFLOAT_MIN_MV)
		mv = VFLOAT_MIN_MV;
	else if(mv > VFLOAT_MAX_MV)
		mv = VFLOAT_MAX_MV;

	reg_val = (mv - VFLOAT_MIN_MV) / VFLOAT_STEP_MV;
	info->vfloat = reg_val * VFLOAT_STEP_MV + VFLOAT_MIN_MV;
	return fan5451x_write_reg(info->i2c, REG_VFLOAT, reg_val);
}

static int fan5451x_set_ibus(struct fan5451x_info *info, int ma)
{
	int reg_val;

	if (ma < IBUS_MIN_MA)
		ma = IBUS_MIN_MA;
	else if(ma > IBUS_MAX_MA)
		ma = IBUS_MAX_MA;

	reg_val = (ma - IBUS_MIN_MA) / IBUS_STEP_MA;
	info->ibus = reg_val * IBUS_STEP_MA + IBUS_MIN_MA;
	return fan5451x_write_reg(info->i2c, REG_IBUS, reg_val);
}

// need to check from FAIRCHILD
static int fan5451x_enable(struct fan5451x_info *info, int enable)
{
	if (enable) {
		fan5451x_update_reg(info->i2c, REG_CON3, 0, CON3_CE);
	} else {
		fan5451x_update_reg(info->i2c, REG_CON3, 1, CON3_CE);
	}
	battery_log(BAT_LOG_CRTI, "[fan5451x] enable : %d\n", enable);
	return 0;
}

static int fan5451x_get_suspend(struct fan5451x_info *info)
{
	u8 read_reg;

	fan5451x_read_reg(info->i2c, REG_CON2, &read_reg);
	if (read_reg & CON2_HZMOD)
		return 1;
	else
		return 0;
}

static int fan5451x_suspend(struct fan5451x_info *info, int suspend)
{
	if (suspend) {
		fan5451x_update_reg(info->i2c, REG_CON2, 0 << CON2_HZMOD_SHIFT, CON2_HZMOD);
	} else {
		fan5451x_update_reg(info->i2c, REG_CON2, 1 << CON2_HZMOD_SHIFT, CON2_HZMOD);
	}
	battery_log(BAT_LOG_CRTI, "[fan5451x] suspend : %d\n", suspend);
	return 0;
}

int fan5451x_dump_register(void)
{
	BYTE config[7], stat[3], con[4], itr[3], mnt[2];
	int ret = 0;
	u8 read_reg;

	ret = i2c_smbus_read_i2c_block_data(new_client, REG_STAT0, 3, stat);
	if (ret < 0){
		printk("%s: i2c read error\n", __func__);
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[DUMP@]STATUS0:0x%02x STATUS1:0x%02x STATUS2:0x%02x\n",stat[0],stat[1],stat[2]);

	ret = i2c_smbus_read_i2c_block_data(new_client, REG_INT0, 3, itr);
	if (ret < 0){
		printk("%s: i2c read error\n", __func__);
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[DUMP@]INT0:0x%02x INT1:0x%02x INT:0x%02x\n",itr[0],itr[1],itr[2]);

	ret = i2c_smbus_read_i2c_block_data(new_client, REG_MNT0, 2, mnt);
	if (ret < 0){
		printk("%s: i2c read error\n", __func__);
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[DUMP@]MONITOR0:0x%02x MONITOR1:0x%02x\n",stat[0],stat[1]);

	ret = fan5451x_read_reg(new_client, REG_BOOST, &read_reg);
	if (ret < 0){
		printk("%s: i2c read error\n", __func__);
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[DUMP@]BOOST:0x%02x\n",read_reg);

	ret = i2c_smbus_read_i2c_block_data(new_client, REG_CON0, 4, con);
	if (ret < 0){
		printk("%s: i2c read error\n", __func__);
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[DUMP@]CON0:0x%02x CON1:0x%02x CON2:0x%02x CON3:0x%02x\n",con[0],con[1],con[2],con[3]);

	ret = i2c_smbus_read_i2c_block_data(new_client, REG_VFLOAT, 7, config);
	if (ret < 0){
		printk("%s: i2c read error\n", __func__);
		return ret;
	}
	battery_log(BAT_LOG_CRTI,
				"[DUMP@]VFLOAT:0x%02x IOCHRG:0x%02x IBAT:0x%02x IBUS:0x%02x VBUS:0x%02x IIN:0x%02x VIN:0x%02x\n",
				config[0],config[1],config[2],config[3],config[4],config[5],config[6]);
//	battery_log(BAT_LOG_CRTI,
//				"[DUMP@]VFLOAT:%d IOCHRG:%d IBAT:%d IBUS:%d VBUS:%d IIN:%d VIN:%d\n",
//				config[0],config[1],config[2],config[3],config[4],config[5],config[6]);
	return 0;
}

static int fan5451x_charger_get_property(struct power_supply *psy, enum power_supply_property prop, union power_supply_propval *val)
{
	struct fan5451x_info *info = container_of(psy, struct fan5451x_info, fan_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fan5451x_get_status(info);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = fan5451x_get_vbuspwr(info);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = info->iochrg * 1000;
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s:iochrg(%d)\n", __func__, info->iochrg);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = info->ibus * 1000;
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s:ibus(%d)\n", __func__, info->ibus);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = info->vfloat * 1000;
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s:vfloat(%d)\n", __func__, info->vfloat);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = fan5451x_get_suspend(info);
		if (val->intval)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = fan5451x_get_chgcmp(info);
		if (val->intval == 2)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGER_DUMP:
		val->intval = fan5451x_dump_register();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int fan5451x_charger_set_property(struct power_supply *psy, enum power_supply_property prop, const union power_supply_propval *val)
{
	int ret = 0;
	struct fan5451x_info *info = container_of(psy, struct fan5451x_info, fan_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		fan5451x_set_iochrg(info, val->intval / 1000);
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s:iochrg(%d)\n", __func__, info->iochrg);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		fan5451x_set_ibus(info, val->intval / 1000);
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s:ibus(%d)\n", __func__, info->ibus);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		fan5451x_set_vfloat(info, val->intval / 1000);
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s:vfloat(%d)\n", __func__, info->vfloat);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		info->suspend = val->intval;
		fan5451x_suspend(info, info->suspend);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		info->battery_charging_enabled = val->intval;
		fan5451x_enable(info, info->battery_charging_enabled);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int fan5451x_charger_property_is_writeable(struct power_supply *psy, enum power_supply_property prop)
{
	int rc = 0;
	switch (prop) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
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

static enum power_supply_property fan5451x_charger_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
};

static struct power_supply fan5451x_psy = {
	.name = "fan5451x",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties	= fan5451x_charger_properties,
	.num_properties = ARRAY_SIZE(fan5451x_charger_properties),
	.get_property = fan5451x_charger_get_property,
	.set_property = fan5451x_charger_set_property,
	.property_is_writeable = fan5451x_charger_property_is_writeable,
};

int fan5451x_enable_safety_timer(int enable)
{
	if (enable)
		fan5451x_write_reg(new_client, REG_TIMER, 0x1f); // Safety Timer On (16hour)
	else
		fan5451x_write_reg(new_client, REG_TIMER, 0x18); // Safety Timer Off

	return 0;
}

int fan5451x_set_otg_boost_enable(int enable)
{
	if (enable) {
		fan5451x_update_reg(new_client, REG_BOOST, 1 << BOOST_EN_SHIFT, BOOST_EN);
		fan5451x_update_reg(new_client, REG_BOOST, 1 << BOOST_OTG_SHIFT, BOOST_OTG);
	} else {
		fan5451x_update_reg(new_client, REG_BOOST, 0 << BOOST_EN_SHIFT, BOOST_EN);
		fan5451x_update_reg(new_client, REG_BOOST, 0 << BOOST_OTG_SHIFT, BOOST_OTG);
	}

	battery_log(BAT_LOG_CRTI, "[fan5451x] OTG_BOOST_Enable : %d\n", enable);

	return 0;
}

static int fan5451x_gpio_init(struct fan5451x_info *info)
{
	// Charger_INT GPIO setting
	mt_set_gpio_mode(init_gpio_number, GPIO_MODE_GPIO);
	mt_set_gpio_pull_enable(init_gpio_number, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(init_gpio_number, GPIO_PULL_UP);
	// mt_set_gpio_dir(init_gpio_number, GPIO_DIR_OUT);
	// mt_set_gpio_out(init_gpio_number, GPIO_OUT_ONE);
	mt_set_gpio_dir(init_gpio_number, GPIO_DIR_IN);

	// Charger_disable GPIO setting
	mt_set_gpio_mode(en_gpio_number, GPIO_MODE_GPIO);
	mt_set_gpio_pull_enable(en_gpio_number, GPIO_PULL_DISABLE);
	mt_set_gpio_pull_select(en_gpio_number, GPIO_NO_PULL);
	mt_set_gpio_dir(en_gpio_number, GPIO_DIR_OUT);
	mt_set_gpio_out(en_gpio_number, GPIO_OUT_ZERO);

	return 0;
}

static void fan5451x_initialization(struct fan5451x_info *info)
{
	fan5451x_update_reg(info->i2c, REG_CON0, 0x03, CON0_VBATMIN); // 3V voltage threshold on BAT pin above which Fast charging begins.
	fan5451x_update_reg(info->i2c, REG_CON2, 0, CON2_TOEN); // Disable Post charging
	fan5451x_write_reg(info->i2c, REG_VFLOAT, 0x6e); // 4.4V Charger output 'float' voltage
	fan5451x_write_reg(info->i2c, REG_IOCHRG, 0x1a); // 1.5A IBAT current
	fan5451x_write_reg(info->i2c, REG_IBAT,(0x5 << IBAT_ITERM_SHIFT)| 0x05); // ITERM(150), PRECHG(300)
	fan5451x_write_reg(info->i2c, REG_IBUS, 0x4C); // 2A IBUS
	fan5451x_update_reg(info->i2c, REG_VBUS, 1 << VBUS_OVP_SHIFT, VBUS_OVP);  // 10.5V VBUS OVP threshold
	fan5451x_update_reg(info->i2c, REG_VIN, 0, VIN_OVP); // 6.5V VIN OVP threshold
	fan5451x_write_reg(info->i2c, REG_TIMER, 0x1f); // Safety Timer On (16hour)

	fan5451x_dump_register();

	info->iochrg = 1500;
	info->vfloat = 4400;
	info->ibus = 3000;
}

/*block for bringup*/
#if 0
static irqreturn_t fan5451x_irq_thread(int irq, void *handle)
{
	BYTE intr[3];
	int ret;
	struct fan5451x_info *info = (struct fan5451x_info *)handle;

	ret = i2c_smbus_read_i2c_block_data(info->i2c,REG_INT0,3,intr);
	if (ret < 0){
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s: i2c read error\n", __func__);
		return ret;
	}
	battery_log(BAT_LOG_CRTI,"[fan5451x] %s: INT0:0x%02x, INT1:0x%02x, INT2:0x%02x\n", __func__,intr[0],intr[1],intr[2]);

	/*    INT0    */
	if (intr[0] & INT0_BATINT)
	{
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s: INT0_BATINT\n", __func__);
	}
	else if (intr[0] & INT0_WKBAT)
	{
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s: INT0_WKBAT\n", __func__);
	}
	else if (intr[0] & INT0_CHGMOD)
	{
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s: INT0_CHGMOD\n", __func__);
	}
	else if (intr[0] & INT0_CHGEND)
	{
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s: Normal Charge Cycle completed.\n", __func__);
	}
	/*	  INT1	  */
	else if (intr[1] & INT1_VBATLV)
	{
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s: INT1_VBATLV\n", __func__);
	}
	else if (intr[1] & INT1_BSTWDTTO)
	{
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s: INT1_BSTWDTTO\n", __func__);
	}
	else if (intr[1] & INT1_BSTFAIL)
	{
		msleep(100);
		fan5451x_set_otg_boost_enable(1);
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s: INT1_BSTFAIL\n", __func__);
	}
	/*	  INT2	  */
	else if (intr[2] & INT2_TIMER)
	{
		fan5451x_initialization(info);
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s: INT2_TIMER\n", __func__);
	}
	else if (intr[2] & INT2_BATOCP)
	{
		battery_log(BAT_LOG_CRTI,"[fan5451x] %s: INT2_BATOCP\n", __func__);
	}
	return IRQ_HANDLED;
}
#endif

static int fan5451x_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct fan5451x_info *info = rdev_get_drvdata(rdev);
	int rc = 0;

	battery_log(BAT_LOG_CRTI, "[fan5451x] OTG_BOOST Enable\n");

	info->otg_boost_enabled = true;

	fan5451x_update_reg(new_client, REG_BOOST, 1 << BOOST_EN_SHIFT, BOOST_EN);
	fan5451x_update_reg(new_client, REG_BOOST, 1 << BOOST_OTG_SHIFT, BOOST_OTG);

	return rc;
}

static int fan5451x_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct fan5451x_info *info = rdev_get_drvdata(rdev);
	int rc = 0;

	fan5451x_update_reg(new_client, REG_BOOST, 0 << BOOST_EN_SHIFT, BOOST_EN);
	fan5451x_update_reg(new_client, REG_BOOST, 0 << BOOST_OTG_SHIFT, BOOST_OTG);

	info->otg_boost_enabled = false;

	battery_log(BAT_LOG_CRTI, "[fan5451x] OTG_BOOST Disable\n");

	return rc;
}

static int fan5451x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct fan5451x_info *info = rdev_get_drvdata(rdev);

	return info->otg_boost_enabled ? 1 : 0;
}

struct regulator_ops fan5451x_otg_regulator_ops = {
	.enable		= fan5451x_otg_regulator_enable,
	.disable	= fan5451x_otg_regulator_disable,
	.is_enabled	= fan5451x_otg_regulator_is_enable,
};

static int fan5451x_regulator_init(struct fan5451x_info *info)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(info->dev, info->dev->of_node);
	if (!init_data) {
		battery_log(BAT_LOG_CRTI, "[fan5451x] failed to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		info->otg_vreg.rdesc.owner = THIS_MODULE;
		info->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		info->otg_vreg.rdesc.ops = &fan5451x_otg_regulator_ops;
		info->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = info->dev;
		cfg.init_data = init_data;
		cfg.driver_data = info;
		cfg.of_node = info->dev->of_node;

		info->otg_vreg.rdev =
			devm_regulator_register(info->dev, &info->otg_vreg.rdesc, &cfg);
		if (IS_ERR(info->otg_vreg.rdev)) {
			rc = PTR_ERR(info->otg_vreg.rdev);
			if (rc != -EPROBE_DEFER)
				battery_log(BAT_LOG_CRTI, "[fan5451x] failed to register regulator, rc=%d\n", rc);
		}
	}

	return rc;
}

static int fan5451x_parse_dt(struct device_node *dev_node, struct fan5451x_info *info)
{
	int ret;

	ret = of_property_read_u32(dev_node, "vfloat", &info->vfloat);
	if (ret) {
		battery_log(BAT_LOG_CRTI,"vfloat not defined\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ibus", &info->ibus);
	if (ret) {
		battery_log(BAT_LOG_CRTI,"vfloat not defined\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "iochrg", &info->iochrg);
	if (ret) {
		battery_log(BAT_LOG_CRTI,"vfloat not defined\n");
		return ret;
	}

	return 0;
}

static int fan5451x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	struct fan5451x_info *info;
	struct device_node *dev_node = client->dev.of_node;

	battery_log(BAT_LOG_CRTI,"[fan5451x] %s Start\n",__func__);

	info = (struct fan5451x_info*)kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "memory allocation failed.\n");
		return -ENOMEM;
	}

	if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
		return -ENOMEM;
	}
	memset(new_client, 0, sizeof(struct i2c_client));
	new_client = client;

	i2c_set_clientdata(client, info);
	info->i2c = client;
	info->dev = &client->dev;
/*block for bringup*/
#if 0
	info->irq = gpio_to_irq(8);
	if (info->irq == -EINVAL)
		goto enable_irq_failed;
#endif

	if (dev_node) {
		rc = fan5451x_parse_dt(dev_node, info);
		if (rc) {
			dev_err(&client->dev, "Failed to parse dt\n");
		}
	}

	rc = fan5451x_regulator_init(info);
	if (rc)
		return rc;

	fan5451x_gpio_init(info);

	info->fan_psy = fan5451x_psy;

	rc = power_supply_register(&client->dev, &info->fan_psy);

	if(rc < 0){
		dev_err(&client->dev, "power supply register failed.\n");
		return rc;
	}

/*block for bringup*/
#if 0
	rc = request_threaded_irq(info->irq, NULL, fan5451x_irq_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT, "fan5451x_irq", info);

	if (rc){
		dev_err(&client->dev, "failed to reqeust IRQ\n");
		goto request_irq_failed;
	}
#endif

	fan5451x_initialization(info);

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	chr_control_register(&info->fan_psy);
#endif

	battery_log(BAT_LOG_CRTI,"[fan5451x] %s End\n",__func__);
	return 0;

/*block for bringup*/
#if 0
enable_irq_failed:
	free_irq(info->irq,NULL);
request_irq_failed:
	i2c_set_clientdata(client, NULL);
	kfree(info);
#endif

	return rc;
}

static int fan5451x_remove(struct i2c_client *client)
{
	struct fan5451x_info *info = i2c_get_clientdata(client);

	power_supply_unregister(&info->fan_psy);

	return 0;
}

/* for otg supply, if otg on, regulator disable call */
static void fan5451x_shutdown(struct i2c_client *client)
{
	struct fan5451x_info *info = i2c_get_clientdata(client);

	if (info->otg_boost_enabled) {
		fan5451x_update_reg(new_client, REG_BOOST, 0 << BOOST_EN_SHIFT, BOOST_EN);
		fan5451x_update_reg(new_client, REG_BOOST, 0 << BOOST_OTG_SHIFT, BOOST_OTG);
		battery_log(BAT_LOG_CRTI, "[fan5451x] OTG_BOOST Disable (poweroff)\n");
	}

	return;
}

static const struct of_device_id fan5451x_of_match[] = {
	{
		.compatible = "fc,fan5451x",
	},
};

static const struct i2c_device_id fan5451x_i2c_id[] = {
	{
		.name = "fan5451x",
		.driver_data = 0,
	},
};

static struct i2c_driver fan5451x_driver = {
	.probe = fan5451x_probe,
	.remove = fan5451x_remove,
	.shutdown = fan5451x_shutdown,
	.driver = {
		.name = "fan5451x",
		.of_match_table = fan5451x_of_match,
	},
	.id_table = fan5451x_i2c_id,
};

static int __init fan5451x_init(void)
{
	return i2c_add_driver(&fan5451x_driver);
}

static void __exit fan5451x_exit(void)
{
	i2c_del_driver(&fan5451x_driver);
}

module_init(fan5451x_init);
module_exit(fan5451x_exit);
