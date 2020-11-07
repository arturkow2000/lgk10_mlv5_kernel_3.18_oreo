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

#include <mt-plat/charging.h>
#include <mach/mt_charging.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>

#include "bq24262.h"

typedef unsigned char BYTE;

#define bq24262_DEV_NAME "bq24262"

/* Not Use */
#define IBUS_MIN_MA	100
#define IBUS_MAX_MA	3000
#define IBUS_STEP_MA	25

#define IOCHRG_MIN_MA	500
#define IOCHRG_MAX_MA	3000
#define IOCHRG_STEP_MA	100

#define VFLOAT_MIN_MV	3500
#define VFLOAT_MAX_MV	4440
#define VFLOAT_STEP_MV	20

#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

#ifdef CONFIG_OF
#else
static struct i2c_board_info i2c_bq24262 __initdata = { I2C_BOARD_INFO(bq24262_DEV_NAME, 0x6B) };
#endif

static struct i2c_client *new_client;
static unsigned int att_addr = bq24262_CON4;
int en_gpio_number = (14 | 0x80000000);
unsigned char bq24262_reg[bq24262_REG_NUM] = { 0 };

const unsigned int INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA, CHARGE_CURRENT_150_00_MA, CHARGE_CURRENT_500_00_MA,
	CHARGE_CURRENT_900_00_MA,
	CHARGE_CURRENT_1500_00_MA, CHARGE_CURRENT_1950_00_MA, CHARGE_CURRENT_2500_00_MA,
	CHARGE_CURRENT_2000_00_MA
};



static const struct i2c_device_id bq24262_i2c_id[] = { {bq24262_DEV_NAME, 0}, {} };

static const struct of_device_id bq24262_of_match[] = { {.compatible =
							 "mediatek,SWITHING_CHARGER",}, {},
};

static enum power_supply_property bq24262_charger_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX
};

struct bq24262_info {
	struct i2c_client *i2c;
	struct mutex mutex;
	int irq;
	struct power_supply fan_psy;
	struct class *fan_class;

	int dis_set;

	int iochrg;
	int vfloat;
	int ibus;

	int status;
	int enable;
};

static int bq24262_read_reg(struct i2c_client *i2c, BYTE reg, BYTE *dest)
{
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI, "[bq24262] %s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}
	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int bq24262_write_reg(struct i2c_client *i2c, BYTE reg, BYTE value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	if (ret < 0)
		battery_log(BAT_LOG_CRTI, "[bq24262] %s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
	return ret;
}

static int bq24262_update_reg(struct i2c_client *i2c, u8 reg, BYTE val, BYTE mask)
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

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number,
					   unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				battery_log(2, "zzf_%d<=%d     i=%d\n", pList[i], level, i);
				return pList[i];
			}
		}
		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}
		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

unsigned int charging_parameter_to_value(const unsigned int *parameter,
					 const unsigned int array_size, const unsigned int val)
{
	unsigned int i;

	battery_log(BAT_LOG_FULL, "array_size = %d \r\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	battery_log(BAT_LOG_CRTI, "NO register value match \r\n");
	/* TODO: ASSERT(0);    // not find the value */
	return 0;
}

static int bq24262_get_chgset(struct bq24262_info *info)
{
	unsigned char ret = 0;
	unsigned char char_stats = 0;

	bq24262_read_reg(info->i2c, bq24262_CON0, &ret);

	char_stats = ((ret >> CON0_CHRG_STAT_SHIFT) & CON0_CHRG_STAT);

	return (int)char_stats;
}

static int bq24262_get_chgcmp(struct bq24262_info *info)
{

	if ((2 == bq24262_get_chgset(info)))
		return 1;
	else
		return 0;
}

static int bq24262_get_status(struct bq24262_info *info)
{
	int status = 0;

	if ((1 == bq24262_get_chgset(info))) {
		status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (bq24262_get_chgcmp(info))
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	return status;
}

static int bq24262_set_iochrg(struct bq24262_info *info, int ma)
{
	int reg_val;

	if (ma < IOCHRG_MIN_MA)
		ma = IOCHRG_MIN_MA;
	else if (ma > IOCHRG_MAX_MA)
		ma = IOCHRG_MAX_MA;

	reg_val = (ma - IOCHRG_MIN_MA) / IOCHRG_STEP_MA;
	info->iochrg = reg_val * IOCHRG_STEP_MA + IOCHRG_MIN_MA;
	pr_info("[JJB] %s : Setting chg_mA = %d, set_ibat = %d, reg_val = 0x%02x\n", __func__, ma,
		info->iochrg, reg_val);
	return bq24262_update_reg(info->i2c, bq24262_CON4, (reg_val << CON4_ICHG_SHIFT),
				  (CON4_ICHG << CON4_ICHG_SHIFT));
}

static int bq24262_set_vfloat(struct bq24262_info *info, int mv)
{
	int reg_val;

	if (mv < VFLOAT_MIN_MV)
		mv = VFLOAT_MIN_MV;
	else if (mv > VFLOAT_MAX_MV)
		mv = VFLOAT_MAX_MV;

	reg_val = (mv - VFLOAT_MIN_MV) / VFLOAT_STEP_MV;
	info->vfloat = reg_val * VFLOAT_STEP_MV + VFLOAT_MIN_MV;
	pr_info("[JJB] %s :req_vbat = %d set_vbat = %d reg_val = 0x%02x\n", __func__, mv,
		info->vfloat, reg_val);
	return bq24262_update_reg(info->i2c, bq24262_CON2, (reg_val << CON2_VBREG_SHIFT),
				  (CON2_VBREG << CON2_VBREG_SHIFT));
}

static int bq24262_set_ibus(struct bq24262_info *info, int ma)
{
	unsigned int current_value = (unsigned int)(ma * 100);
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	array_size = GETARRAYNUM(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size, set_chr_current);

	info->ibus = INPUT_CS_VTH[register_value];

	pr_info("[JJB] %s :req_ibus = %d set_ibus = %d reg_val = 0x%02x\n", __func__, ma,
		info->ibus, register_value);
	return bq24262_update_reg(info->i2c, bq24262_CON1, (register_value << CON1_IINLIM_SHIFT),
				  (CON1_IINLIM << CON1_IINLIM_SHIFT));
}

static int bq24262_remove_set_reg(struct bq24262_info *info)
{
	u8 read_reg_00;
	u8 read_reg_01;

	pr_info("%s: I_limit 500mA set!!\n", __func__);
	bq24262_update_reg(info->i2c, bq24262_CON1,
			   (0x02 << CON1_IINLIM_SHIFT), (CON1_IINLIM << CON1_IINLIM_SHIFT));

	bq24262_read_reg(info->i2c, bq24262_CON0, &read_reg_00);
	bq24262_read_reg(info->i2c, bq24262_CON1, &read_reg_01);

	pr_info("%s: check reg[0x00] : 0x%02X [0x01] : 0x%02X\n", __func__, read_reg_00,
		read_reg_01);
	return 0;
}

static void bq24262_enable(struct bq24262_info *info, int enable)
{
	if (enable) {
		battery_log(BAT_LOG_CRTI, "[bq24262]charging enable\n");
		bq24262_update_reg(info->i2c, bq24262_CON1,
				   (!enable << CON1_CHG_CONFIG_SHIFT),
				   (CON1_CHG_CONFIG << CON1_CHG_CONFIG_SHIFT));
	} else {
		battery_log(BAT_LOG_CRTI, "[bq24262]charging disable\n");
		bq24262_update_reg(info->i2c, bq24262_CON1,
				   (!enable << CON1_CHG_CONFIG_SHIFT),
				   (CON1_CHG_CONFIG << CON1_CHG_CONFIG_SHIFT));
	}
}

/*Boost mode enable for OTG*/
#ifdef CONFIG_LGE_PM_OTG_BOOST_MODE
static void bq24262_otg_boost_mode(bool enable)
{
	u8 read_reg_00;
	u8 read_reg_01;

	if (enable) {
		pr_info("[OTG] %s: enable\n", __func__);
		bq24262_update_reg(new_client, bq24262_CON0,
				   (enable << CON0_BOOST_MODE_SHIFT),
				   (CON0_BOOST_MODE << CON0_BOOST_MODE_SHIFT));
		bq24262_update_reg(new_client, bq24262_CON6,
				   (!enable << CON6_BOOST_ILIM_SHIFT),
				   (CON6_BOOST_ILIM << CON6_BOOST_ILIM_SHIFT));
	} else {
		pr_info("[OTG] %s: disable\n", __func__);
		bq24262_update_reg(new_client, bq24262_CON0,
				   (enable << CON0_BOOST_MODE_SHIFT),
				   (CON0_BOOST_MODE << CON0_BOOST_MODE_SHIFT));
		bq24262_update_reg(new_client, bq24262_CON6,
				   (enable << CON6_BOOST_ILIM_SHIFT),
				   (CON6_BOOST_ILIM << CON6_BOOST_ILIM_SHIFT));
	}

	bq24262_read_reg(new_client, bq24262_CON0, &read_reg_00);
	bq24262_read_reg(new_client, bq24262_CON6, &read_reg_01);

	pr_info("[OTG] %s: reg[0x00] : 0x%02X [0x06] : 0x%02X\n", __func__, read_reg_00,
		read_reg_01);
}

int detect_otg_mode(int enable)
{
	pr_info("[OTG] %s: boost_enable = %d\n", __func__, enable);

	if (enable) {
		pr_info("[OTG] %s: enable\n", __func__);
		bq24262_otg_boost_mode(1);
	} else {
		pr_info("[OTG] %s: disable\n", __func__);
		bq24262_otg_boost_mode(0);
	}
}
#endif

static int bq24262_charger_get_property(struct power_supply *psy, enum power_supply_property prop,
					union power_supply_propval *val)
{
	struct bq24262_info *info = container_of(psy, struct bq24262_info, fan_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq24262_get_status(info);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = info->ibus;
		battery_log(BAT_LOG_CRTI, "[bq24262] %s:ibus(%d)\n", __func__, info->ibus);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = info->iochrg;
		battery_log(BAT_LOG_CRTI, "[bq24262] %s:iochrg(%d)\n", __func__, info->iochrg);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = info->vfloat;
		battery_log(BAT_LOG_CRTI, "[bq24262] %s:vfloat(%d)\n", __func__, info->vfloat);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq24262_charger_set_property(struct power_supply *psy, enum power_supply_property prop,
					const union power_supply_propval *val)
{
	int ret = 0;
	struct bq24262_info *info = container_of(psy, struct bq24262_info, fan_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_CHARGING:
			info->enable = 1;
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			info->enable = 0;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		if (!ret)
			bq24262_enable(info, info->enable);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		bq24262_set_ibus(info, val->intval);
		battery_log(BAT_LOG_CRTI, "[bq24262] %s:ibus(%d)\n", __func__, info->ibus);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		bq24262_set_iochrg(info, val->intval);
		battery_log(BAT_LOG_CRTI, "[bq24262] %s:iochrg(%d)\n", __func__, info->iochrg);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		bq24262_set_vfloat(info, val->intval / 1000);
		battery_log(BAT_LOG_CRTI, "[bq24262] %s:vfloat(%d)\n", __func__, info->vfloat);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int bq24262_charger_property_is_writeable(struct power_supply *psy,
						 enum power_supply_property prop)
{
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static struct power_supply bq24262_psy = {
	.name = "bq24262",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = bq24262_charger_properties,
	.num_properties = ARRAY_SIZE(bq24262_charger_properties),
	.get_property = bq24262_charger_get_property,
	.set_property = bq24262_charger_set_property,
	.property_is_writeable = bq24262_charger_property_is_writeable,
};

int bq24262_dump_register(void)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < bq24262_REG_NUM; i++) {
		ret = i2c_smbus_read_i2c_block_data(new_client, i, 1, &bq24262_reg[i]);
		if (ret < 0) {
			battery_log(BAT_LOG_CRTI, "[bq24262 reg@]read fail\n");
			pr_err("%s: i2c read error\n", __func__);
			return ret;
		}
	}
	battery_log(BAT_LOG_CRTI,
		    "[bq24262 reg@][0x00]=0x%02x ,[0x01]=0x%02x ,[0x02]=0x%02x ,[0x03]=0x%02x ,[0x04]=0x%02x ,[0x05]=0x%02x ,[0x06]=0x%02x\n",
		    bq24262_reg[0], bq24262_reg[1], bq24262_reg[2], bq24262_reg[3], bq24262_reg[4],
		    bq24262_reg[5], bq24262_reg[6]);
	return 0;
}


static int bq24262_gpio_init(struct bq24262_info *info)
{
	/* GPIO setting */
	mt_set_gpio_mode(en_gpio_number, GPIO_MODE_GPIO);
	mt_set_gpio_pull_enable(en_gpio_number, GPIO_PULL_DISABLE);
	mt_set_gpio_pull_select(en_gpio_number, GPIO_NO_PULL);
	mt_set_gpio_dir(en_gpio_number, GPIO_DIR_OUT);

	mt_set_gpio_out(en_gpio_number, GPIO_OUT_ZERO);	/* low disable */
	/*GPIO_OUT_ONE : high enable */
/*
	if (gpio_get_value(info->en_set)) {
	pinctrl_select_state(info->pin, info->not_charging);
	} else {
	pinctrl_select_state(info->pin, info->charging);
	}
*/
	return 0;
}

static irqreturn_t bq24262_irq_thread(int irq, void *handle)
{
	BYTE intr;
	int ret;
	unsigned char sys_fault = 0;
	unsigned char chrg_fault = 0;
	struct bq24262_info *info = (struct bq24262_info *)handle;

	mutex_lock(&info->mutex);


	ret = i2c_smbus_read_i2c_block_data(info->i2c, bq24262_CON0, 1, &intr);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI, "[bq24262] %s: i2c read error\n", __func__);
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[bq24262] %s: REG00:0x%02x\n", __func__, intr);

	sys_fault = (intr & (CON0_CHRG_STAT << CON0_CHRG_STAT_SHIFT));
	chrg_fault = (intr & (CON0_CHG_FAULT << CON0_CHG_FAULT_SHIFT));

	if (intr & (CON0_CHRG_STAT << CON0_CHRG_STAT_SHIFT)) {
		if (sys_fault == 1)
			battery_log(BAT_LOG_CRTI, "[bq24262] %s: STAT Fault\n", __func__);
	} else if (intr & (CON0_CHG_FAULT << CON0_CHG_FAULT_SHIFT)) {
		if (chrg_fault == 1)
			battery_log(BAT_LOG_CRTI, "[bq24262] %s: VIN OVP or Boost Mode OVP\n",
				    __func__);
		else if (chrg_fault == 2)
			battery_log(BAT_LOG_CRTI,
				    "[bq24262] %s: Low Supply connected or Boost Mode OverCurrent\n",
				    __func__);
		else if (chrg_fault == 3)
			battery_log(BAT_LOG_CRTI, "[bq24262] %s: Thermal Shutdown\n", __func__);
		else if (chrg_fault == 4)
			battery_log(BAT_LOG_CRTI, "[bq24262] %s: Battery Temperature Fault\n",
				    __func__);
		else if (chrg_fault == 5)
			battery_log(BAT_LOG_CRTI,
				    "[bq24262] %s: Timer Fault (Watchdog or Safety)\n", __func__);
		else if (chrg_fault == 6)
			battery_log(BAT_LOG_CRTI, "[bq24262] %s: Battery OVP\n", __func__);
		else if (chrg_fault == 7)
			battery_log(BAT_LOG_CRTI, "[bq24262] %s: No Battery connected\n", __func__);
	}

	ret = i2c_smbus_read_i2c_block_data(info->i2c, bq24262_CON6, 1, &intr);
	if (ret < 0) {
		battery_log(BAT_LOG_CRTI, "[bq24262] %s: i2c read error\n", __func__);
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[bq24262] %s: REG06:0x%02x\n", __func__, intr);

	ret = bq24262_get_chgset(info);
	pr_info("[bq24262] %s: bq24262_get_chgset = %d\n", __func__, ret);
	if ((3 == bq24262_get_chgset(info))) {
		pr_info("[bq24262] %s: charger NOT exist !\n", __func__);
		bq24262_remove_set_reg(info);
	}

	bq24262_dump_register();

	mutex_unlock(&info->mutex);
	return IRQ_HANDLED;
}


static ssize_t show_bq24262_addr_register(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	return sprintf(buf, "0x%x\n", att_addr);
}

static ssize_t store_bq24262_addr_register(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	int ret = 0;
	/* struct bq24262_info *info = dev_get_drvdata(dev); */

	if (!buf)
		return 0;

	if (size == 0)
		return 0;

	if (kstrtouint(buf, 10, &ret) == 0) {
		att_addr = ret;
		return size;
	} else
		return 0;
}

static DEVICE_ATTR(bq24262_addr_register, 0664, show_bq24262_addr_register,
		   store_bq24262_addr_register);

static ssize_t show_bq24262_status_register(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	unsigned char ret = 0;
	struct bq24262_info *info = dev_get_drvdata(dev);

	bq24262_read_reg(info->i2c, att_addr, &ret);
	return sprintf(buf, "%d\n", ret);
}

static ssize_t store_bq24262_status_register(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	int ret = 0;
	struct bq24262_info *info = dev_get_drvdata(dev);

	if (!buf)
		return 0;

	if (size == 0)
		return 0;
	if (kstrtouint(buf, 10, &ret) == 0) {
		bq24262_write_reg(info->i2c, att_addr, ret);
		return size;
	}

	return 0;
}

static DEVICE_ATTR(bq24262_status_register, 0664, show_bq24262_status_register,
		   store_bq24262_status_register);
static void bq24262_initialization(struct bq24262_info *info)
{
	bq24262_update_reg(info->i2c, bq24262_CON1,
			   (0x00 << CON1_HZ_MODE_SHIFT), (CON1_HZ_MODE << CON1_HZ_MODE_SHIFT));

	bq24262_update_reg(info->i2c, bq24262_CON6,
			   (0x00 << CON6_VINDPM_OFF_SHIFT),
			   (CON6_VINDPM_OFF << CON6_VINDPM_OFF_SHIFT));
	/* VIN DPM off 4.20V */
	bq24262_update_reg(info->i2c, bq24262_CON5,
			   (0x01 << CON5_DPM_STATUS_SHIFT),
			   (CON5_DPM_STATUS << CON5_DPM_STATUS_SHIFT));
	/* VIN DPM 4.2V */
	bq24262_update_reg(info->i2c, bq24262_CON5,
			   (0x04 << CON5_VINDPM_SHIFT), (CON5_VINDPM << CON5_VINDPM_SHIFT));
	/* VIN DPM plus 8% = 4.536V */

	bq24262_update_reg(info->i2c, bq24262_CON1,
			   (0x00 << CON1_REG_RST_SHIFT), (CON1_REG_RST << CON1_REG_RST_SHIFT));

	bq24262_update_reg(info->i2c, bq24262_CON5,
			   (0x00 << CON5_MINSYS_STATUS_SHIFT),
			   (CON5_MINSYS_STATUS << CON5_MINSYS_STATUS_SHIFT));
	/* Minimum system voltage 3.5V disable */

	bq24262_update_reg(info->i2c, bq24262_CON4,
			   (0x05 << CON4_ITERM_SHIFT), (CON4_ITERM << CON4_ITERM_SHIFT));
	/* Termination current 300mA */

	bq24262_update_reg(info->i2c, bq24262_CON2,
			   (0x2D << CON2_VBREG_SHIFT), (CON2_VBREG << CON2_VBREG_SHIFT));
	/* VREG 4.4V */

	bq24262_update_reg(info->i2c, bq24262_CON6,
			   (0x00 << CON6_XTMR_EN_SHIFT), (CON6_XTMR_EN << CON6_XTMR_EN_SHIFT));
	/* Timer not slowed */
	bq24262_update_reg(info->i2c, bq24262_CON0,
			   (0x00 << CON0_TMR_RST_SHIFT), (CON0_TMR_RST << CON0_TMR_RST_SHIFT));
	/* Disable WDT */

#ifdef CONFIG_LGE_PM_OTG_BOOST_MODE
	pr_info("[OTG] %s: bq24262_probe otg_boost_mode init\n", __func__);
	detect_otg_mode(0);
#endif

	bq24262_dump_register();
}

static int bq24262_parse_dt(struct device_node *dev_node, struct bq24262_info *info)
{
	int ret = 0;

	ret = of_property_read_u32(dev_node, "vfloat", &info->vfloat);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "vfloat not defined\n");
		info->vfloat = 4350;
	}
	ret = of_property_read_u32(dev_node, "ibus", &info->ibus);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "ibus not defined\n");
		info->ibus = 1500;
	}
	ret = of_property_read_u32(dev_node, "iochrg", &info->iochrg);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "iochrg not defined\n");
		info->iochrg = 1500;
	}

	pr_info("[JJB] %s :vfloat = %d ibus = %d iochrg = %d\n", __func__, info->vfloat, info->ibus,
		info->iochrg);
	return 0;
}

static int bq24262_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct bq24262_info *info;
	struct device_node *dev_node = client->dev.of_node;

	battery_log(BAT_LOG_CRTI, "[bq24262_PROBE_START]\n");
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "memory allocation failed.\n");
		return -ENOMEM;
	}

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!new_client)
		return -ENOMEM;

	memset(new_client, 0, sizeof(struct i2c_client));

	new_client = client;

	i2c_set_clientdata(client, info);
	info->i2c = client;
	info->irq = gpio_to_irq(8);
	mutex_init(&info->mutex);

	if (dev_node) {
		ret = bq24262_parse_dt(dev_node, info);
		if (ret)
			dev_err(&client->dev, "Failed to parse dt\n");
	}

	bq24262_gpio_init(info);

	info->fan_psy = bq24262_psy;

	ret = power_supply_register(&client->dev, &info->fan_psy);

	if (ret < 0) {
		dev_err(&client->dev, "power supply register failed.\n");
		return ret;
	}

	ret = request_threaded_irq(info->irq, NULL, bq24262_irq_thread,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				   "bq24262_irq", info);

	if (ret) {
		dev_err(&client->dev, "failed to request IRQ\n");
		goto request_irq_failed;
	}

	bq24262_initialization(info);
	chargin_hw_init_done = KAL_TRUE;

	device_create_file(&(client->dev), &dev_attr_bq24262_addr_register);
	device_create_file(&(client->dev), &dev_attr_bq24262_status_register);

	battery_log(BAT_LOG_CRTI, "[bq24262_PROBE_SUCCESS]\n");
	return 0;

request_irq_failed:
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
	kfree(info);

	return ret;
}

static int bq24262_remove(struct i2c_client *client)
{
	struct bq24262_info *info = i2c_get_clientdata(client);

	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, info);
	}
	kfree(info);

	return 0;
}

static struct i2c_driver bq24262_driver = {
	.probe = bq24262_probe,
	.remove = bq24262_remove,
	.driver = {
		   .name = "bq24262",
#ifdef CONFIG_OF
		   .of_match_table = bq24262_of_match,
#endif
		   },
	.id_table = bq24262_i2c_id,
};

static int __init bq24262_init(void)
{
	battery_log(BAT_LOG_CRTI, "[bq24262_init] init start\n");

	/* i2c registeration using DTS instead of boardinfo */
#ifdef CONFIG_OF
	battery_log(BAT_LOG_CRTI, "[bq24262_init] init start with i2c DTS");
#else
	battery_log(BAT_LOG_CRTI, "[bq24262_init] init start. ch=%d\n", bq24262_BUSNUM);
	i2c_register_board_info(2, &i2c_bq24262, 1);
#endif

	if (i2c_add_driver(&bq24262_driver) != 0) {
		battery_log(BAT_LOG_CRTI,
			    "[bq24262_init] failed to register bq24262 i2c driver.\n");
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[bq24262_init] Success to register bq24262 i2c driver.\n");
	}

	return 0;
}

static void __exit bq24262_exit(void)
{
	i2c_del_driver(&bq24262_driver);
}
module_init(bq24262_init);
module_exit(bq24262_exit);
