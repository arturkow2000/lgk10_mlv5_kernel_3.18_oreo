/*
* sm5424.c -- 3.5A Input, Switch Mode Charger device driver
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
#include <linux/debugfs.h>

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
#include "charging_hw_external_charger.h"
#endif

#include "sm5424.h"

typedef unsigned char BYTE;

#define VBUSLIMIT_MIN_MA	100
#define VBUSLIMIT_1ST_MA	375
#define VBUSLIMIT_MAX_MA	3525
#define VBUSLIMIT_STEP_MA	25

#define FASTCHG_MIN_MA	350
#define FASTCHG_MAX_MA	3500
#define FASTCHG_STEP_MA	50

#define BATREG_MIN_MV	3990
#define BATREG_MAX_MV	4620
#define BATREG_STEP_MV	10

#define TOPOFF_MIN_MV	100
#define TOPOFF_MAX_MV	475
#define TOPOFF_STEP_MV	25

/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK    0
#define STATUS_UNSUPPORTED    -1
#define STATUS_FAIL -2


static struct i2c_client *new_client = NULL;

static int init_gpio_number = (8 | 0x80000000);
static int en_gpio_number = (24 | 0x80000000);

struct sm5424_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct sm5424_info {
	struct i2c_client *i2c;
	struct device *dev;
	struct mutex mutex;
	int irq;
	struct power_supply sm_psy;
	struct sm5424_regulator otg_vreg;
	struct class *fan_class;

	int dis_set;

	int fastchg;
	int batreg;
	int vbuslimit;
	int total_chr_current;

	int status;
	int enable;
	int suspend;
	bool otg_boost_enabled;

	/* debugfs */
	struct dentry *debugfs;
	u32 debug_addr;
};


static int sm5424_read_reg(struct i2c_client *i2c, BYTE reg, BYTE *dest)
{
	int ret;
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI,"[sm5424] %s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}
	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int sm5424_write_reg(struct i2c_client *i2c, BYTE reg, BYTE value)
{
	int ret;
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI,"[sm5424] %s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
	return ret;
}

static int sm5424_update_reg(struct i2c_client *i2c, u8 reg, BYTE val, BYTE mask)
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

static int sm5424_get_vbusok(struct sm5424_info *info)
{
	u8 read_reg_00;
	BYTE ret = 0;

	sm5424_read_reg(info->i2c, SM5424_STATUS1, &read_reg_00);
	ret = ((read_reg_00 & SM5424_STATUS1_VBUSPOK) >> SM5424_STATUS1_SHIFT);

	pr_debug("[sm5424] %s: STATUS1 : 0x%02X VBUSPOK = %d\n", __func__, read_reg_00, ret);

	if (ret)
		return 1;
	else
		return 0;
}

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
int detect_vbus_ok(void)
{
	u8 read_reg_00;
	u8 read_reg_01;
	int ret_vbus = 0;
	int ret_chgon = 0;

	sm5424_read_reg(new_client, SM5424_STATUS1, &read_reg_00);
	sm5424_read_reg(new_client, SM5424_STATUS2, &read_reg_01);

	ret_vbus = (read_reg_00 & (SM5424_STATUS1_VBUSPOK << SM5424_STATUS1_SHIFT));
	ret_chgon = (read_reg_01 & (SM5424_STATUS2_CHGON << SM5424_STATUS2_SHIFT));

/*
	pr_debug("[sm5424] %s: STATUS1 : 0x%02X STATUS2 : 0x%02X \n", __func__, read_reg_00, read_reg_01);
	pr_debug("[sm5424] %s: VBUSPOK = %d CHGON = %d\n", __func__, ret_vbus, ret_chgon);
*/
	return ret_vbus;
}
#endif

static int sm5424_get_en_hiz(struct sm5424_info *info)
{
	u8 read_reg;
	int ret;

	sm5424_read_reg(info->i2c, SM5424_CNTL, &read_reg);

	ret = ((read_reg & SM5424_CNTL_SUSPEND_MASK) >> SM5424_CNTL_SUSPEND_SHIFT);

	pr_debug("[sm5424] %s: SM5424_CNTL : 0x%02X Suspend = %d\n", __func__, read_reg, ret);

	if (ret)
		return 1;
	else
		return 0;
}

static int sm5424_get_chgcmp(struct sm5424_info *info)
{
	u8 read_reg;
	int chgon_ret;
	int topoff_ret;
	int done_ret;

	sm5424_read_reg(info->i2c, SM5424_STATUS2, &read_reg);

	chgon_ret = ((read_reg & SM5424_STATUS2_CHGON) >> SM5424_STATUS2_SHIFT);
	topoff_ret = ((read_reg & SM5424_STATUS2_TOPOFF) >> SM5424_STATUS2_TOPOFF_SHIFT);
	done_ret = ((read_reg & SM5424_STATUS2_DONE) >> SM5424_STATUS2_DONE_SHIFT);

	pr_info("[sm5424] %s: STATUS2 : 0x%02X CHGON = %d TOPOFF = %d DONE = %d\n", __func__, read_reg, chgon_ret, topoff_ret, done_ret);

	if (topoff_ret || done_ret)
		return 1;
	else if (chgon_ret)
		return 2;
	else
		return 0;
}

static int sm5424_get_status(struct sm5424_info *info)
{
	int status = 0;

	switch (sm5424_get_chgcmp(info)) {
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

static int sm5424_set_fastchg(struct sm5424_info *info, int ma)
{
	int reg_val;

	if (ma < FASTCHG_MIN_MA)
		ma = FASTCHG_MIN_MA;
	else if(ma > FASTCHG_MAX_MA)
		ma = FASTCHG_MAX_MA;

	reg_val = (ma - FASTCHG_MIN_MA)/FASTCHG_STEP_MA;
	info->fastchg = reg_val*FASTCHG_STEP_MA + FASTCHG_MIN_MA;

	return sm5424_update_reg(info->i2c, SM5424_CHGCNTL2,
			 (reg_val << SM5424_CHGCNTL2_FASTCHG_SHIFT),
			 (SM5424_CHGCNTL2_FASTCHG_MASK << SM5424_CHGCNTL2_FASTCHG_SHIFT));
}

static int sm5424_set_batreg(struct sm5424_info *info, int mv)
{
	int reg_val;

	if (mv < BATREG_MIN_MV)
		mv = BATREG_MIN_MV;
	else if(mv > BATREG_MAX_MV)
		mv = BATREG_MAX_MV;

	reg_val = (mv - BATREG_MIN_MV) / BATREG_STEP_MV;
	info->batreg = reg_val * BATREG_STEP_MV + BATREG_MIN_MV;

	return sm5424_write_reg(info->i2c, SM5424_CHGCNTL3, reg_val);
}

static int sm5424_set_vbuslimit(struct sm5424_info *info, int ma)
{
	int reg_val;

	if (ma < VBUSLIMIT_MIN_MA)
		ma = VBUSLIMIT_MIN_MA;
	else if(ma > VBUSLIMIT_MAX_MA)
		ma = VBUSLIMIT_MAX_MA;

	if (ma < VBUSLIMIT_1ST_MA) {
		reg_val = VBUSLIMIT_100mA;
		info->vbuslimit = VBUSLIMIT_MIN_MA;
	} else {
		reg_val = ((ma - VBUSLIMIT_1ST_MA) / VBUSLIMIT_STEP_MA) + 1;
		info->vbuslimit = VBUSLIMIT_1ST_MA
				+ ((reg_val - 1) * VBUSLIMIT_STEP_MA);
	}

	return sm5424_write_reg(info->i2c, SM5424_VBUSCNTL, reg_val);
}

static int sm5424_enable(struct sm5424_info *info, int enable)
{
	int val = enable ? 1 : 0;

	sm5424_update_reg(info->i2c, SM5424_CNTL, val << SM5424_CNTL_CHGEN_SHIFT,
			SM5424_CNTL_CHGEN_MASK << SM5424_CNTL_CHGEN_SHIFT);

	return 0;
}

static int sm5424_suspend(struct sm5424_info *info, int suspend)
{
	int val = suspend ? 1 : 0;

	sm5424_update_reg(info->i2c, SM5424_CNTL, val << SM5424_CNTL_SUSPEND_SHIFT,
			SM5424_CNTL_SUSPEND_MASK << SM5424_CNTL_SUSPEND_SHIFT);

	return 0;
}

int sm5424_dump_register(struct sm5424_info *info)
{
	unsigned char sm5424_reg[SM5424_REG_NUM] = { 0 };
	int i = 6;
	int ret = 0;

	for (i = 6; i < SM5424_REG_NUM - 1; i++) {
		ret = i2c_smbus_read_i2c_block_data(info->i2c, i, 1, &sm5424_reg[i]);
		if (ret < 0) {
			battery_log(BAT_LOG_CRTI, "[sm5424_reg@]read fail\n");
			pr_err("%s: i2c read error\n", __func__);
			return ret;
		}
	}

	battery_log(BAT_LOG_CRTI,
		    "[DUMP@][0x06]0x%02x,[0x07]0x%02x,[0x08]0x%02x,[0x09]0x%02x,[0x0A]0x%02x\n",
		    sm5424_reg[6], sm5424_reg[7], sm5424_reg[8], sm5424_reg[9], sm5424_reg[10]);

	battery_log(BAT_LOG_CRTI,
		    "[DUMP@][0x0B]0x%02x,[0x0C]0x%02x,[0x0D]0x%02x,[0x0E]0x%02x,[0x0F]0x%02x,[0x10]0x%02x\n",
		    sm5424_reg[11], sm5424_reg[12], sm5424_reg[13], sm5424_reg[14], sm5424_reg[15], sm5424_reg[16]);

	return 0;
}

static inline unsigned char _calc_topoff_current_offset_to_mA(unsigned short mA)
{
	unsigned char offset;

	if (mA < TOPOFF_MIN_MV) {
		offset = TOPOFF_100mA;               /* Topoff = 100mA */
	} else if (mA < TOPOFF_MAX_MV) {
		offset = ((mA - TOPOFF_MIN_MV) / TOPOFF_STEP_MV) & SM5424_CHGCNTL4_TOPOFF_MASK;   /* Topoff = 125mA ~ 450mA in 25mA steps */
	} else {
		offset = TOPOFF_475mA;              /* Topoff = 475mA */
	}

	return offset;
}

static int sm5424_set_TOPOFF(struct sm5424_info *info, u8 val)
{
 //   u8 offset = _calc_topoff_current_offset_to_mA(topoff_mA);

	sm5424_update_reg(info->i2c, SM5424_CHGCNTL4,
			(val & SM5424_CHGCNTL4_TOPOFF_MASK) << SM5424_CHGCNTL4_TOPOFF_SHIFT,
			SM5424_CHGCNTL4_TOPOFF_MASK << SM5424_CHGCNTL4_TOPOFF_SHIFT);

	battery_log(BAT_LOG_CRTI, "[sm5424] set_TOPOFF : 0x%x\n", val);

	return 0;
}

static int sm5424_set_AICLEN(struct sm5424_info *info, bool enable)
{
	sm5424_update_reg(info->i2c, SM5424_CHGCNTL1,
			(enable & SM5424_CHGCNTL1_AICLEN_MASK) << SM5424_CHGCNTL1_AICLEN_SHIFT,
			SM5424_CHGCNTL1_AICLEN_MASK << SM5424_CHGCNTL1_AICLEN_SHIFT);

	battery_log(BAT_LOG_CRTI, "[sm5424] set_AICLEN : %d\n", enable);

	return 0;
}

static int sm5424_set_AICLTH(struct sm5424_info *info, u8 val)
{
	sm5424_update_reg(info->i2c, SM5424_CHGCNTL1,
			(val & SM5424_CHGCNTL1_AICLTH_MASK) << SM5424_CHGCNTL1_AICLTH_SHIFT,
			SM5424_CHGCNTL1_AICLTH_MASK << SM5424_CHGCNTL1_AICLTH_SHIFT);

	battery_log(BAT_LOG_CRTI, "[sm5424] set_AICLTH : %d\n", val);

	return 0;
}

static int sm5424_set_RECHG(struct sm5424_info *info, bool enable)
{
    sm5424_update_reg(info->i2c, SM5424_CHGCNTL2,
			(enable & SM5424_CHGCNTL2_RECHG_MASK) << SM5424_CHGCNTL2_RECHG_SHIFT,
			SM5424_CHGCNTL2_RECHG_MASK << SM5424_CHGCNTL2_RECHG_SHIFT);

	battery_log(BAT_LOG_CRTI, "[sm5424] set_RECHG : %d\n", enable);

	return 0;
}

static int sm5424_set_AUTOSTOP(struct sm5424_info *info, bool enable)
{
	sm5424_update_reg(info->i2c, SM5424_CNTL,
			(enable & SM5424_CNTL_AUTOSTOP_MASK) << SM5424_CNTL_AUTOSTOP_SHIFT,
			SM5424_CNTL_AUTOSTOP_MASK << SM5424_CNTL_AUTOSTOP_SHIFT);

	battery_log(BAT_LOG_CRTI, "[sm5424] set_AUTOSTOP : %d\n", enable);

	return 0;
}

static int sm5424_set_OTGCURRENT(struct sm5424_info *info, u8 val)
{
	sm5424_update_reg(info->i2c, SM5424_CHGCNTL5,
			(val & SM5424_CHGCNTL5_OTGCURRENT_MASK) << SM5424_CHGCNTL5_OTGCURRENT_SHIFT,
			SM5424_CHGCNTL5_OTGCURRENT_MASK << SM5424_CHGCNTL5_OTGCURRENT_SHIFT);

	battery_log(BAT_LOG_CRTI, "[sm5424] set_OTGCURRENT : 0x%x\n", val);

	return 0;
}

static int sm5424_set_BST_IQ3LIMIT(struct sm5424_info *info, u8 val)
{
	sm5424_update_reg(info->i2c, SM5424_CHGCNTL5,
			(val & SM5424_CHGCNTL5_BST_IQ3LIMIT_MASK) << SM5424_CHGCNTL5_BST_IQ3LIMIT_SHIFT,
			SM5424_CHGCNTL5_BST_IQ3LIMIT_MASK << SM5424_CHGCNTL5_BST_IQ3LIMIT_SHIFT);

	battery_log(BAT_LOG_CRTI, "[sm5424] set_BST_IQ3LIMIT : 0x%x\n", val);

	return 0;
}

static int sm5424_set_VOTG(struct sm5424_info *info, u8 val)
{
	sm5424_update_reg(info->i2c, SM5424_CHGCNTL6,
			(val & SM5424_CHGCNTL6_VOTG_MASK) << SM5424_CHGCNTL6_VOTG_SHIFT,
			SM5424_CHGCNTL6_VOTG_MASK << SM5424_CHGCNTL6_VOTG_SHIFT);

	battery_log(BAT_LOG_CRTI, "[sm5424] set_VOTG : 0x%x\n", val);

	return 0;
}

static int sm5424_set_TOPOFFTIMER_timer(struct sm5424_info *info, int topoff_min)
{
	sm5424_update_reg(info->i2c, SM5424_CHGCNTL5,
			topoff_min << TOPOFFTIMER_SHIFT,
			TOPOFFTIMER_MASK << TOPOFFTIMER_SHIFT);

	battery_log(BAT_LOG_CRTI, "[sm5424] set_TOPOFFTIMER_timer : 0x%x\n", topoff_min);

	return 0;
}

static int sm5424_set_FASTTIMER_timer(struct sm5424_info *info, int fast_hours)
{
	sm5424_update_reg(info->i2c, SM5424_CHGCNTL5,
			fast_hours << FASTTIMER_SHIFT,
			FASTTIMER_MASK << FASTTIMER_SHIFT); // Safety(FASTTIMER) Timer On

	battery_log(BAT_LOG_CRTI, "[sm5424] set_FASTTIMER_timer : 0x%x\n", fast_hours);

	return 0;
}

static int sm5424_charger_get_property(struct power_supply *psy, enum power_supply_property prop, union power_supply_propval *val)
{
	struct sm5424_info *info = container_of(psy, struct sm5424_info, sm_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = sm5424_get_status(info);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sm5424_get_vbusok(info);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sm5424_get_vbusok(info);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = info->vbuslimit * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = info->fastchg * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = info->batreg * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = sm5424_get_en_hiz(info);
		if (val->intval)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = sm5424_get_chgcmp(info);
		if (val->intval == 2)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGER_DUMP:
		val->intval = sm5424_dump_register(info);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sm5424_charger_set_property(struct power_supply *psy, enum power_supply_property prop, const union power_supply_propval *val)
{
	int ret = 0;
	struct sm5424_info *info = container_of(psy, struct sm5424_info, sm_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		info->enable = val->intval;
		sm5424_enable(info, info->enable);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		sm5424_set_vbuslimit(info, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		sm5424_set_fastchg(info, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		sm5424_set_batreg(info, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (val->intval)
			info->suspend = 0;
		else
			info->suspend = 1;
		sm5424_suspend(info, info->suspend);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int sm5424_charger_property_is_writeable(struct power_supply *psy, enum power_supply_property prop)
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

static enum power_supply_property sm5424_charger_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
};

static char *sm5424_supplied_to[] = {
	"ac",
};

static struct power_supply sm5424_psy = {
	.name = "sm5424",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties	= sm5424_charger_properties,
	.num_properties = ARRAY_SIZE(sm5424_charger_properties),
	.supplied_to = sm5424_supplied_to,
	.num_supplicants = ARRAY_SIZE(sm5424_supplied_to),
	.get_property = sm5424_charger_get_property,
	.set_property = sm5424_charger_set_property,
	.property_is_writeable = sm5424_charger_property_is_writeable,
};

static struct power_supply sm5424_parallel_psy = {
	.name = "usb-parallel",
	.type = POWER_SUPPLY_TYPE_USB_PARALLEL,
	.properties	= sm5424_charger_properties,
	.num_properties = ARRAY_SIZE(sm5424_charger_properties),
	.get_property = sm5424_charger_get_property,
	.set_property = sm5424_charger_set_property,
	.property_is_writeable = sm5424_charger_property_is_writeable,
};

static int sm5424_gpio_init(struct sm5424_info *info)
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

static void sm5424_initialization(struct sm5424_info *info)
{
/* IRQ read & clear for nINT pin state initialization */
	u8 read_reg_00;
	u8 read_reg_01;
	u8 read_reg_02;

	sm5424_read_reg(info->i2c, SM5424_INT1, &read_reg_00);
	sm5424_read_reg(info->i2c, SM5424_INT2, &read_reg_01);
	sm5424_read_reg(info->i2c, SM5424_INT3, &read_reg_02);
/* IRQ read & clear for nINT pin state initialization */

	sm5424_write_reg(info->i2c, SM5424_INTMASK1, 0xE8); // BATOVP | VBUSOVP | VBUSUVLO | VBUSPOK
	sm5424_write_reg(info->i2c, SM5424_INTMASK2, 0xF2); // DONE | TOPOFF | CHGON
	sm5424_write_reg(info->i2c, SM5424_INTMASK3, 0xD8); // FASTTMROFF | OTGFAIL | THEMSHDN | THEMREG

	/*Switching Frequency 1.5MHz*/
	sm5424_update_reg(info->i2c, SM5424_CHGCNTL4,
			(0x02<< SM5424_CHGCNTL4_FREQSEL_SHIFT),
			(SM5424_CHGCNTL4_FREQSEL_MASK << SM5424_CHGCNTL4_FREQSEL_SHIFT));

	sm5424_set_AICLTH(info, AICL_THRESHOLD_4_6_V);
	sm5424_set_AICLEN(info, AICL_EN);
	sm5424_set_batreg(info, 4400); // 4.4V Charger output 'float' voltage
	sm5424_set_RECHG(info, RECHG_100mV); //Recharging voltage VBATREG - 100mV
	sm5424_set_BST_IQ3LIMIT(info, BSTIQ3LIMIT_4P0A);
	sm5424_set_AUTOSTOP(info, AUTOSTOP_EN);
	sm5424_set_TOPOFF(info, TOPOFF_150mA);
	sm5424_set_fastchg(info, 500); // 1.5A  typical charging current during Fast charging
//	sm5424_set_vbuslimit(info, VBUSLIMIT_3000mA); // Input current 3A
	sm5424_set_TOPOFFTIMER_timer(info, TOPOFFTIMER_10MIN);
//	sm5424_set_FASTTIMER_timer(FASTTIMER_8_0_HOUR);// FAST Timer On (8hour)
	sm5424_set_FASTTIMER_timer(info, FASTTIMER_DISABLED);
	sm5424_set_OTGCURRENT(info, OTGCURRENT_900mA); //OTG current
	sm5424_set_VOTG(info, VOTG_5_0_V); // OTG voltage
	sm5424_dump_register(info);
}

static void sm5424_parallel_initialization(struct sm5424_info *info)
{
/* IRQ read & clear for nINT pin state initialization */
//	u8 read_reg_00;
//	u8 read_reg_01;
//	u8 read_reg_02;

//	sm5424_read_reg(info->i2c, SM5424_INT1, &read_reg_00);
//	sm5424_read_reg(info->i2c, SM5424_INT2, &read_reg_01);
//	sm5424_read_reg(info->i2c, SM5424_INT3, &read_reg_02);
/* IRQ read & clear for nINT pin state initialization */

//	sm5424_write_reg(info->i2c, SM5424_INTMASK1, 0xC8); // AICL | BATOVP | VBUSOVP | VBUSUVLO | VBUSPOK
//	sm5424_write_reg(info->i2c, SM5424_INTMASK2, 0xF2); // DONE | TOPOFF | CHGON
//	sm5424_write_reg(info->i2c, SM5424_INTMASK3, 0xD8); // FASTTMROFF | OTGFAIL | THEMSHDN | THEMREG

	/*Switching Frequency 1.5MHz*/
	sm5424_update_reg(info->i2c, SM5424_CHGCNTL4,
			(0x02<< SM5424_CHGCNTL4_FREQSEL_SHIFT),
			(SM5424_CHGCNTL4_FREQSEL_MASK << SM5424_CHGCNTL4_FREQSEL_SHIFT));

	sm5424_set_AICLTH(info, AICL_THRESHOLD_4_6_V);
	sm5424_set_AICLEN(info, AICL_EN);
	sm5424_set_batreg(info, 4370); // 4.4V Charger output 'float' voltage
	sm5424_set_RECHG(info, RECHG_100mV); //Recharging voltage VBATREG - 100mV
	sm5424_set_BST_IQ3LIMIT(info, BSTIQ3LIMIT_4P0A);
	sm5424_set_AUTOSTOP(info, AUTOSTOP_EN);
	sm5424_set_TOPOFF(info, TOPOFF_450mA);
	sm5424_set_fastchg(info, 500); // 1.5A  typical charging current during Fast charging
//	sm5424_set_vbuslimit(info, VBUSLIMIT_3000mA); // Input current 3A
	sm5424_set_TOPOFFTIMER_timer(info, TOPOFFTIMER_10MIN);
//	sm5424_set_FASTTIMER_timer(FASTTIMER_8_0_HOUR);// FAST Timer On (8hour)
	sm5424_set_FASTTIMER_timer(info, FASTTIMER_DISABLED);
//	sm5424_set_OTGCURRENT(info, OTGCURRENT_900mA); //OTG current
//	sm5424_set_VOTG(info, VOTG_5_0_V); // OTG voltage
	sm5424_dump_register(info);
}

static char *sm5424_int1_msg[] = {
	"A valid VBUS detected",
	"VBUS UVLO detected",
	"VBUS OVP detected",
	"VBUS current limit detected",
	"BAT OVP detected",
	"AICL threshold detected",
	"No Battery detected",
};

static char *sm5424_int2_msg[] = {
	"The charger is ON with charging current",
	"The Q4 FET operates in full-on mode",
	"Top-Off condition detected",
	"Top-Off timer expired",
	"Boost POK threshold on a boost regulation voltage detected",
};

static char *sm5424_int3_msg[] = {
	"Thremal regultation threshold detected",
	"Thermal shutdown detected",
	"VBUS output dropped below the UVLO in OTG due to abnormal conditions",
	"Current Limit threshold detected in discharge mode and no valid input supply",
	"Pre-charge timer expired",
	"Fast charge timer expired",
	"Hot threshold detected",
	"Cold threshold detected",
};

static irqreturn_t sm5424_irq_thread(int irq, void *handle)
{
	struct sm5424_info *info = (struct sm5424_info *)handle;
	bool notify = false;
	u8 int1, int2, int3;
	u8 status1, status2, status3;
	int i;

	/* IRQ read & clear for nINT pin state initialization */
	sm5424_read_reg(info->i2c, SM5424_INT1, &int1);
	sm5424_read_reg(info->i2c, SM5424_INT2, &int2);
	sm5424_read_reg(info->i2c, SM5424_INT3, &int3);
	pr_info("[sm5424] %s: INT1 : 0x%02X INT2 : 0x%02X INT3 : 0x%02X\n",
		__func__, int1, int2, int3);

	sm5424_read_reg(info->i2c, SM5424_STATUS1, &status1);
	sm5424_read_reg(info->i2c, SM5424_STATUS2, &status2);
	sm5424_read_reg(info->i2c, SM5424_STATUS3, &status3);
	pr_debug("[sm5424] %s: STATUS1 : 0x%02X STATUS2 : 0x%02X STATUS3 : 0x%02X\n",
		__func__, status1, status2, status3);

	/* show interrupt message */
	for (i = 0; i < ARRAY_SIZE(sm5424_int1_msg); i++) {
		if (int1 & (1U << i)) {
			pr_info("[sm5424] %s: %s\n", __func__,
					sm5424_int1_msg[i]);
		}
	}
	for (i = 0; i < ARRAY_SIZE(sm5424_int2_msg); i++) {
		if (int2 & (1U << i)) {
			pr_info("[sm5424] %s: %s\n", __func__,
					sm5424_int2_msg[i]);
		}
	}
	for (i = 0; i < ARRAY_SIZE(sm5424_int3_msg); i++) {
		if (int3 & (1U << i)) {
			pr_info("[sm5424] %s: %s\n", __func__,
					sm5424_int3_msg[i]);
		}
	}

	if (int1 & SM5424_INT1_VBUSPOK)
		notify = true;
	if (int1 & SM5424_INT1_VBUSUVLO)
		notify = true;
	if (int1 & SM5424_INT1_VBUSOVP)
		notify = true;

	if (notify)
		power_supply_changed(&info->sm_psy);

	return IRQ_HANDLED;
}

// bringup build error
#if 0
static ssize_t show_sm5424_addr_register(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", att_addr);
}

static ssize_t store_sm5424_addr_register(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int ret=0;
	//struct sm5424_info *info = dev_get_drvdata(dev);

	if (!buf)
		return 0;

	if (size == 0)
		return 0;

	if (kstrtouint(buf, 10, &ret) == 0) {
		att_addr = ret;
		return size;
	}

	return size;
}
static DEVICE_ATTR(sm5424_addr_register, 0664, show_sm5424_addr_register, store_sm5424_addr_register);

static ssize_t show_sm5424_status_register(struct device *dev,struct device_attribute *attr, char *buf)
{
	BYTE ret=0;
	struct sm5424_info *info = dev_get_drvdata(dev);

	sm5424_read_reg(info->i2c, att_addr, &ret);
	return sprintf(buf, "%d\n", ret);
}

static ssize_t store_sm5424_status_register(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	BYTE ret=0;
	struct sm5424_info *info = dev_get_drvdata(dev);

	if (!buf)
		return 0;

	if (size == 0)
		return 0;
	if (kstrtouint(buf, 10, &ret) == 0) {
		sm5424_write_reg(info->i2c, att_addr, ret);
		return size;
	}
	return size;
}
static DEVICE_ATTR(sm5424_status_register, 0664, show_sm5424_status_register, store_sm5424_status_register);
#endif

static int sm5424_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct sm5424_info *info = rdev_get_drvdata(rdev);
	int rc = 0;

	battery_log(BAT_LOG_CRTI, "[sm5424] OTG_BOOST Enable\n");

	info->otg_boost_enabled = true;

	sm5424_suspend(info, SUSPEND_DIS);
	sm5424_enable(info, CHARGE_DIS);
	sm5424_update_reg(info->i2c, SM5424_CNTL, ENBOOST_EN << SM5424_CNTL_ENBOOST_SHIFT, SM5424_CNTL_ENBOOST_MASK << SM5424_CNTL_ENBOOST_SHIFT);

	return rc;
}

static int sm5424_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct sm5424_info *info = rdev_get_drvdata(rdev);
	int rc = 0;

	sm5424_update_reg(info->i2c, SM5424_CNTL, ENBOOST_DIS << SM5424_CNTL_ENBOOST_SHIFT, SM5424_CNTL_ENBOOST_MASK << SM5424_CNTL_ENBOOST_SHIFT);

	info->otg_boost_enabled = false;

	battery_log(BAT_LOG_CRTI, "[sm5424] OTG_BOOST Disable\n");

	return rc;
}

static int sm5424_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct sm5424_info *info = rdev_get_drvdata(rdev);

	return info->otg_boost_enabled ? 1 : 0;
}

struct regulator_ops sm5424_otg_regulator_ops = {
	.enable		= sm5424_otg_regulator_enable,
	.disable	= sm5424_otg_regulator_disable,
	.is_enabled	= sm5424_otg_regulator_is_enable,
};

static int sm5424_regulator_init(struct sm5424_info *info)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(info->dev, info->dev->of_node);
	if (!init_data) {
		battery_log(BAT_LOG_CRTI, "[sm5424] failed to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		info->otg_vreg.rdesc.owner = THIS_MODULE;
		info->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		info->otg_vreg.rdesc.ops = &sm5424_otg_regulator_ops;
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
				battery_log(BAT_LOG_CRTI, "[sm5424] failed to register regulator, rc=%d\n", rc);
		}
	}

	return rc;
}

static int debugfs_get_data(void *data, u64 *val)
{
	struct sm5424_info *info = data;
	int rc;
	u8 temp;

	rc = sm5424_read_reg(info->i2c, info->debug_addr, &temp);
	if (rc)
		return -EAGAIN;

	*val = temp;

	return 0;
}

static int debugfs_set_data(void *data, u64 val)
{
	struct sm5424_info *info = data;
	int rc;
	u8 temp;

	temp = (u8)val;
	rc = sm5424_write_reg(info->i2c, info->debug_addr, temp);
	if (rc)
		return -EAGAIN;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(data_debugfs_ops,
	debugfs_get_data, debugfs_set_data, "0x%02llx\n");

const static int regs_to_dump[] = {
	SM5424_INT1,
	SM5424_INT2,
	SM5424_INT3,
	SM5424_INTMASK1,
	SM5424_INTMASK2,
	SM5424_INTMASK3,
	SM5424_STATUS1,
	SM5424_STATUS2,
	SM5424_STATUS3,
	SM5424_CNTL,
	SM5424_VBUSCNTL,
	SM5424_CHGCNTL1,
	SM5424_CHGCNTL2,
	SM5424_CHGCNTL3,
	SM5424_CHGCNTL4,
	SM5424_CHGCNTL5,
	SM5424_CHGCNTL6,
	SM5424_DEVICEID,
};

static int dump_debugfs_show(struct seq_file *m, void *start)
{
	struct sm5424_info *info = m->private;
	u8 data;
	int rc;
	int i;

	for (i = 0; i < ARRAY_SIZE(regs_to_dump); i++) {
		rc = sm5424_read_reg(info->i2c, regs_to_dump[i], &data);
		if (rc) {
			seq_printf(m, "0x%02x=error\n", regs_to_dump[i]);
		}

		seq_printf(m, "0x%02x=0x%02x\n", regs_to_dump[i], data);
	}

	return 0;
}

static int dump_debugfs_open(struct inode *inode, struct file *file)
{
	struct sm5424_info *info = inode->i_private;

	return single_open(file, dump_debugfs_show, info);
}

static const struct file_operations dump_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= dump_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int create_debugfs_entries(struct sm5424_info *info)
{
	struct dentry *ent;

	info->debugfs = debugfs_create_dir(info->sm_psy.name, NULL);
	if (!info->debugfs) {
		dev_err(info->dev, "failed to create debugfs\n");
		return -ENODEV;
	}

	ent = debugfs_create_x32("addr", S_IFREG | S_IWUSR | S_IRUGO,
		info->debugfs, &info->debug_addr);
	if (!ent)
		dev_err(info->dev, "failed to create addr debugfs\n");

	ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
		info->debugfs, info, &data_debugfs_ops);
	if (!ent)
		dev_err(info->dev, "failed to create data debugfs\n");

	ent = debugfs_create_file("dump", S_IFREG | S_IRUGO,
		info->debugfs, info, &dump_debugfs_ops);
	if (!ent)
		dev_err(info->dev, "failed to create dump debugfs\n");

	return 0;
}

static int sm5424_parse_dt(struct device_node *dev_node, struct sm5424_info *info)
{
	int ret;

	ret = of_property_read_u32(dev_node, "batreg", &info->batreg);
	if (ret) {
		battery_log(BAT_LOG_CRTI,"batreg not defined\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "vbuslimit", &info->vbuslimit);
	if (ret) {
		battery_log(BAT_LOG_CRTI,"vbuslimit not defined\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "fastchg", &info->fastchg);
	if (ret) {
		battery_log(BAT_LOG_CRTI,"fastchg not defined\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "dis_set", &info->dis_set);
	if (ret) {
		battery_log(BAT_LOG_CRTI,"dis_set not defined\n");
		return ret;
	}
	return 0;
}

static int is_parallel_slave(struct i2c_client *client)
{
	struct device_node *dev_node = client->dev.of_node;

	return of_property_read_bool(dev_node, "lge,parallel-charger");
}

static int sm5424_main_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct sm5424_info *info;
	struct device_node *dev_node = client->dev.of_node;

	battery_log(BAT_LOG_CRTI,"[sm5424] %s Start\n",__func__);
	info = (struct sm5424_info*)kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "memory allocation failed.\n");
		return -ENOMEM;
	}

	new_client = client;

	i2c_set_clientdata(client, info);
	info->i2c = client;
	info->dev = &client->dev;
	info->irq = gpio_to_irq(8);
	mutex_init(&info->mutex);
	if (info->irq == -EINVAL)
		goto enable_irq_failed;

	if (dev_node) {
		ret = sm5424_parse_dt(dev_node, info);
		if (ret) {
			dev_err(&client->dev, "Failed to parse dt\n");
		}
	}

	ret = sm5424_regulator_init(info);
	if (ret)
		return ret;

	sm5424_gpio_init(info);

	info->sm_psy = sm5424_psy;

	ret = power_supply_register(&client->dev, &info->sm_psy);

	if(ret < 0){
		dev_err(&client->dev, "power supply register failed.\n");
		return ret;
	}

	ret = request_threaded_irq(info->irq, NULL, sm5424_irq_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT, "sm5424_irq", info);

	if (ret){
		dev_err(&client->dev, "failed to reqeust IRQ\n");
		goto request_irq_failed;
	}

	sm5424_initialization(info);
	create_debugfs_entries(info);

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	chr_control_register(&info->sm_psy);
#endif

// bringup build error
#if 0
	device_create_file(&(client->dev), &dev_attr_sm5424_addr_register);
	device_create_file(&(client->dev), &dev_attr_sm5424_status_register);
#endif

	battery_log(BAT_LOG_CRTI,"[sm5424] %s End\n",__func__);
	return 0;

enable_irq_failed:
	free_irq(info->irq,NULL);
request_irq_failed:
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
	kfree(info);

	return ret;
}

static int sm5424_parallel_slave_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct sm5424_info *info;
	struct device_node *dev_node = client->dev.of_node;

	battery_log(BAT_LOG_CRTI,"[sm5424] %s Start\n",__func__);
	info = (struct sm5424_info*)kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "memory allocation failed.\n");
		return -ENOMEM;
	}

	new_client = client;

	i2c_set_clientdata(client, info);
	info->i2c = client;
	info->dev = &client->dev;
	mutex_init(&info->mutex);

	if (dev_node) {
		ret = sm5424_parse_dt(dev_node, info);
		if (ret) {
			dev_err(&client->dev, "Failed to parse dt\n");
		}
	}

	/* slave charger do not used irq and charging_diable GPIO */
	//sm5424_gpio_init(info);

	info->sm_psy = sm5424_parallel_psy;

	ret = power_supply_register(&client->dev, &info->sm_psy);

	if(ret < 0){
		dev_err(&client->dev, "power supply register failed.\n");
		return ret;
	}

	sm5424_parallel_initialization(info);

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	chr_control_register_slave(&info->sm_psy);
#endif

// bringup build error
#if 0
	device_create_file(&(client->dev), &dev_attr_sm5424_addr_register);
	device_create_file(&(client->dev), &dev_attr_sm5424_status_register);
#endif

	battery_log(BAT_LOG_CRTI,"[sm5424] %s End\n",__func__);
	return 0;
}

static int sm5424_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	if (is_parallel_slave(client))
		return sm5424_parallel_slave_probe(client, id);
	else
		return sm5424_main_probe(client, id);
}

static int sm5424_remove(struct i2c_client *client)
{
	struct sm5424_info *info = i2c_get_clientdata(client);

	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, info);
	}
	if (info)
		kfree(info);

	return 0;
}

/* for otg supply, if otg on, regulator disable call */
void sm5424_shutdown(struct i2c_client *client)
{
	struct sm5424_info *info = i2c_get_clientdata(client);

	if (info->otg_boost_enabled) {
		sm5424_update_reg(info->i2c, SM5424_CNTL, ENBOOST_DIS << SM5424_CNTL_ENBOOST_SHIFT, SM5424_CNTL_ENBOOST_MASK << SM5424_CNTL_ENBOOST_SHIFT);
		battery_log(BAT_LOG_CRTI, "[sm5424] OTG_BOOST Disable (poweroff)\n");
	}
}

static const struct i2c_device_id sm5424_i2c_id[] = {
	{"sm5424", 0},
	{},
};

static const struct of_device_id sm5424_of_match[] = {
	{.compatible = "sm,sm5424",},
	{},
};

static struct i2c_driver sm5424_driver = {
	.probe = sm5424_probe,
	.remove = sm5424_remove,
	.shutdown = sm5424_shutdown,
	.driver = {
		.name = "sm5424",
		.of_match_table = sm5424_of_match,
	},
	.id_table = sm5424_i2c_id,
};

static int __init sm5424_init(void)
{
	return i2c_add_driver(&sm5424_driver);
}

static void __exit sm5424_exit(void)
{
	i2c_del_driver(&sm5424_driver);
}

module_init(sm5424_init);
module_exit(sm5424_exit);
