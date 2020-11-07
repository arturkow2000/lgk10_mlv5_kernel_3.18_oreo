/*
* bq25600.c -- reference charger driver controlled by i2c
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
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>

#include <mt-plat/charging.h>
#include <mach/mt_charging.h>
#include <mt_gpio.h>
#include <mach/gpio_const.h>

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
#include "charging_hw_external_charger.h"
#endif

#define BQ25600_REG00	0x00
#define EN_HIZ	0x80
#define EN_HIZ_SHIFT 7
#define EN_ICHG_MON	0x60
#define EN_ICHG_MON_SHIFT 5
#define IINDPM	0x1F

#define BQ25600_REG01	0x01
#define PFM_DIS	0x80
#define PFM_DIS_SHIFT 7
#define WD_RST	0x40
#define WD_RST_SHIFT 6
#define OTG_CONFIG 0x20
#define OTG_CONFIG_SHIFT 5
#define CHG_CONFIG 0x10
#define CHG_CONFIG_SHIFT 4
#define SYS_MIN 0x0E
#define SYS_MIN_SHIFT 1
#define MIN_VBAT_SEL 0x01
#define MIN_VBAT_SEL_SHIFT 0

#define BQ25600_REG02	0x02
#define BOOST_LIM 0x80
#define BOOST_LIM_SHIFT 7
#define Q1_FULLON	0x40
#define Q1_FULLON_SHIFT 6
#define ICHG 0x3F
#define ICHG_SHIFT 0

#define BQ25600_REG03	0x03
#define IPRECHG 0xF0
#define IPRECHG_SHIFT 4
#define ITERM 0x0F
#define ITERM_SHIFT 0

#define BQ25600_REG04	0x04
#define VREG 0xF8
#define VREG_SHIFT 3
#define TOPOFF_TIMER 0x06
#define TOPOFF_TIMER_SHIFT 1
#define VRECHG 0x01
#define VRECHG_SHIFT 0

#define BQ25600_REG05	0x05
#define EN_TERM 0x80
#define EN_TERM_SHIFT 7
#define OVPFET_DIS	0x40
#define OVPFET_DIS_SHIFT 6
#define WATCHDOG 0x30
#define WATCHDOG_SHIFT 4
#define EN_TIMER 0x08
#define EN_TIMER_SHIFT 3
#define CHG_TIMER 0x04
#define CHG_TIMER_SHIFT 2
#define TREG 0x02
#define JEITA_ISET 0x01

#define BQ25600_REG06	0x06
#define OVP 0xC0
#define OVP_SHIFT 6
#define BOOSTV 0x30
#define BOOSTV_SHIFT 4
#define VINDPM 0x0F

#define BQ25600_REG07	0x07
#define IINDET_EN 0x80
#define IINDET_EN_SHIFT 7
#define TMR2X_EN 0x40
#define TMR2X_EN_SHIFT 6
#define BATFET_DIS 0x20
#define BATFET_DIS_SHIFT 5
#define BATFET_DLY 0x08
#define BATFET_DLY_SHIFT 3
#define BATFET_RST_EN 0x04
#define BATFET_RST_EN_SHIFT 2
#define VDPM_BAT_TRACK 0x03
#define VDPM_BAT_TRACK_SHIFT 0

#define BQ25600_REG08	0x08
#define VBUS_STAT 0xE0
#define VBUS_STAT_SHIFT 5
#define CHRG_STAT 0x18
#define CHRG_STAT_SHIFT 3
#define PG_STAT 0x04
#define PG_STAT_SHIFT 2
#define THERM_STAT 0x02
#define THERM_STAT_SHIFT 1
#define VSYS_STAT 0x01
#define VSYS_STAT_SHIFT 0

#define BQ25600_REG09	0x09
#define WATCHDOG_FAULT 0x80
#define BOOST_FAULT 0x40
#define CHRG_FAULT 0x30
#define CHRG_FAULT_SHIFT 4
#define BAT_FAULT 0x08
#define NTC_FAULT 0x07
#define NTC_FAULT_SHIFT 0

#define BQ25600_REG0A	0x0A
#define VBUS_GD 0x80
#define VBUS_GD_SHIFT 7
#define VINDPM_STAT 0x40
#define VINDPM_STAT_SHIFT 6
#define IINDPM_STAT 0x20
#define IINDPM_STAT_SHIFT 5
#define TOPOFF_ACTIVE 0x08
#define TOPOFF_ACTIVE_SHIFT 3
#define ACOV_STAT 0x04
#define ACOV_STAT_SHIFT 2
#define VINDPM_INT_MASK 0x02
#define VINDPM_INT_MASK_SHIFT 1
#define IINDPM_INT_MASK 0x01
#define IINDPM_INT_MASK_SHIFT 0

#define BQ25600_REG0B	0x0B
#define REG_RST 0x80
#define REG_RST_SHIFT 7
#define PN 0x78
#define PN_SHIFT 3
#define DEV_REV 0x03
#define DEV_REV_SHIFT 0

#define DUMP_SIZE 256

enum {
	VBUS_STAT_NO_INPUT = 0,
	VBUS_STAT_SDP = 1,
	VBUS_STAT_OTG = 7,
};

static int init_gpio_number = (8 | 0x80000000);
static int en_gpio_number = (24 | 0x80000000);

struct bq25600_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct bq25600 {
	struct i2c_client *client;
	struct device *dev;

	struct power_supply psy;
	struct bq25600_regulator otg_vreg;
	struct mutex lock;

	bool enabled;
	bool battery_charging_enabled;
	bool otg_boost_enabled;
	unsigned int input_ua;
	unsigned int battery_ua;
	unsigned int battery_uv;

	/* device tree */
	bool is_slave;
	bool ship_mode;
	bool ship_mode_enabled;
	int irq_gpio;
	int iterm_ma;
	int vindpm_mv;
	int ovp_mv;
	int watchdog_sec;
	bool vindpm_int;
	bool iindpm_int;
	int chg_timer_hrs;
	bool chg_timer_disabled;
	int vreg_mv;
	int vrechg_mv;

	/* debugfs */
	struct dentry *debugfs;
	u32 debug_addr;

	/* dump buffer */
	char buffer[DUMP_SIZE];
};

const static int regs_to_dump[] = {
	BQ25600_REG00,
	BQ25600_REG01,
	BQ25600_REG02,
	BQ25600_REG03,
	BQ25600_REG04,
	BQ25600_REG05,
	BQ25600_REG06,
	BQ25600_REG07,
	BQ25600_REG08,
	BQ25600_REG09,
	BQ25600_REG0A,
	BQ25600_REG0B,
};

static int bq25600_read(struct i2c_client *client, u8 reg, u8 *data)
{
	int rc;

	rc = i2c_smbus_read_byte_data(client, reg);
	if (rc < 0)
		dev_err(&client->dev, "failed to read. rc=%d\n", rc);

	*data = (rc & 0xFF);

	return rc < 0 ? rc : 0;
}

static int bq25600_write(struct i2c_client *client, u8 reg, u8 data)
{
	int rc;

	rc = i2c_smbus_write_byte_data(client, reg, data);
	if (rc < 0)
		dev_err(&client->dev, "failed to write. rc=%d\n", rc);

	return rc;
}

static int bq25600_masked_write(struct i2c_client *client, u8 reg, u8 data, u8 mask)
{
	struct bq25600 *chip = i2c_get_clientdata(client);
	u8 tmp;
	int rc = 0;

	mutex_lock(&chip->lock);

	rc = bq25600_read(client, reg, &tmp);
	if (rc < 0)
		goto out;

	tmp = (data & mask) | (tmp & (~mask));
	rc = bq25600_write(client, reg, tmp);

out:
	mutex_unlock(&chip->lock);
	return rc;
}

static int bq25600_get_en_hiz(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG00, &data);
	if (rc)
		return rc;

	data &= EN_HIZ;
	data >>= EN_HIZ_SHIFT;

	return data;
}

static int bq25600_set_en_hiz(struct bq25600 *chip, int en)
{
	u8 data = 0;

	if (en)
		data = 1;

	data <<= EN_HIZ_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG00, data, EN_HIZ);
}

static int bq25600_get_iindpm(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG00, &data);
	if (rc)
		return rc;

	data &= IINDPM;
	return (data * 100) + 100;
}

static int bq25600_set_iindpm(struct bq25600 *chip, int ma)
{
	u8 data;

	if (ma < 100)
		ma = 100;
	else if (ma > 3200)
		ma = 3200;

	data = (ma - 100) / 100;

	return bq25600_masked_write(chip->client,
			BQ25600_REG00, data, IINDPM);
}

static int bq25600_get_otg_config(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG01, &data);
	if (rc)
		return rc;

	data &= OTG_CONFIG;
	data >>= OTG_CONFIG_SHIFT;
	return data;
}

static int bq25600_set_otg_config(struct bq25600 *chip, int en)
{
	u8 data = 0;

	if (en)
		data = 1;

	data <<= OTG_CONFIG_SHIFT;

	battery_log(BAT_LOG_CRTI, "[bq25600] bq25600_set_otg_config\n");

	return bq25600_masked_write(chip->client,
			BQ25600_REG01, data, OTG_CONFIG);
}

static int bq25600_get_chg_config(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG01, &data);
	if (rc)
		return rc;

	data &= CHG_CONFIG;
	data >>= CHG_CONFIG_SHIFT;

	return data;
}

static int bq25600_set_chg_config(struct bq25600 *chip, int en)
{
	u8 data = 0;

	if (en)
		data = 1;

	data <<= CHG_CONFIG_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG01, data, CHG_CONFIG);
}

static int bq25600_get_boost_lim(struct bq25600 *chip)
{
	u8 data = 0;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG02, &data);
	if (rc)
		return rc;

	data &= BOOST_LIM;
	data >>= BOOST_LIM_SHIFT;

	if (data)
		return 1200;

	return 500;
}

static int bq25600_set_boost_lim(struct bq25600 *chip, int ma)
{
	u8 data = 1;

	if (ma <= 500)
		data = 0;

	data <<= BOOST_LIM_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG02, data, BOOST_LIM);
}

static int bq25600_get_ichg(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG02, &data);
	if (rc)
		return rc;

	data &= ICHG;
	return data * 60;
}

static int bq25600_set_ichg(struct bq25600 *chip, int ma)
{
	u8 data;

	if (ma < 0)
		ma = 0;
	else if (ma > 3000)
		ma = 3000;

	data = ma / 60;

	return bq25600_masked_write(chip->client,
			BQ25600_REG02, data, ICHG);
}

static int bq25600_get_iterm(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG03, &data);
	if (rc)
		return rc;

	data &= ITERM;
	return (data + 1) * 60;
}

static int bq25600_set_iterm(struct bq25600 *chip, int ma)
{
	u8 data;

	if (ma < 60)
		ma = 60;
	else if (ma > 780)
		ma = 780;

	data = (ma - 60) / 60;

	return bq25600_masked_write(chip->client,
			BQ25600_REG03, data, ITERM);
}

static int bq25600_get_vreg(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG04, &data);
	if (rc)
		return rc;

	data &= VREG;
	data >>= VREG_SHIFT;

	/* special value */
	if (data == 0xFF)
		return 4352;

	return data * 32 + 3856;
}

static int bq25600_set_vreg(struct bq25600 *chip, int mv)
{
	u8 data;

	if (mv < 3856)
		mv = 3856;
	else if (mv > 4624)
		mv = 4624;

	data = (mv - 3856) / 32;
	data <<= VREG_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG04, data, VREG);
}

static int bq25600_set_vrechg(struct bq25600 *chip, int mv)
{
	u8 data = 0;

	/* if invalid value came, set as default */
	if (mv != 100 && mv != 200)
		mv = 100;

	if (mv == 200)
		data = 1;

	return bq25600_masked_write(chip->client,
			BQ25600_REG04, data, VRECHG);
}

static int bq25600_set_watchdog(struct bq25600 *chip, int sec)
{
	u8 data;

	if (sec < 0)
		sec = 0;
	if (sec > 160)
		sec = 160;

	data = sec / 40;
	data <<= WATCHDOG_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG05, data, WATCHDOG);
}

static int bq25600_get_en_timer(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG05, &data);
	if (rc)
		return rc;

	data &= EN_TIMER;
	data >>= EN_TIMER_SHIFT;
	return data;
}

static int bq25600_set_en_timer(struct bq25600 *chip, int en)
{
	u8 data;

	if (en)
		data = 1;
	else
		data = 0;
	data <<= EN_TIMER_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG05, data, EN_TIMER);
}

static int bq25600_set_chg_timer(struct bq25600 *chip, int hrs)
{
	u8 data;

	if (hrs == 5)
		data = 0;
	else
		data = 1;
	data <<= CHG_TIMER_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG05, data, CHG_TIMER);
}

static int bq25600_set_ovp(struct bq25600 *chip, int mv)
{
	const int ovp_mv[] = {
		5500,
		6200,
		10500,
		14300,
	};
	u8 data;
	int i;

	for (i = 0; i < ARRAY_SIZE(ovp_mv); i++) {
		if (ovp_mv[i] == mv)
			break;
	}
	if (i == ARRAY_SIZE(ovp_mv))
		i = 1;
	data = i << OVP_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG06, data, OVP);
}

static int bq25600_set_boostv(struct bq25600 *chip, int mv)
{
	u8 data;

	if (mv < 4850)
		mv = 4850;
	if (mv > 5300)
		mv = 5300;

	data = (mv - 4850) / 150;
	data <<= BOOSTV_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG06, data, BOOSTV);
}

static int bq25600_set_vindpm(struct bq25600 *chip, int mv)
{
	u8 data;

	if (mv < 3900)
		mv = 3900;
	if (mv > 5400)
		mv = 5400;

	data = (mv - 3900) / 100;

	return bq25600_masked_write(chip->client,
			BQ25600_REG06, data, VINDPM);
}

static int bq25600_get_vbus_stat(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG08, &data);
	if (rc)
		return rc;

	data &= VBUS_STAT;
	data >>= VBUS_STAT_SHIFT;

	return data;
}

static int bq25600_get_chrg_stat(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG08, &data);
	if (rc)
		return rc;

	data &= CHRG_STAT;
	data >>= CHRG_STAT_SHIFT;

	return data;
}
static int bq25600_get_pg_stat(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG08, &data);
	if (rc)
		return rc;

	data &= PG_STAT;
	data >>= PG_STAT_SHIFT;

	return data;
}

static int bq25600_get_vbus_gd(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG0A, &data);
	if (rc)
		return rc;

	data &= VBUS_GD;
	data >>= VBUS_GD_SHIFT;

	return data;
}

static int bq25600_set_vindpm_int(struct bq25600 *chip, int en)
{
	u8 data = 0;

	if (!en)
		data = 1 << VINDPM_INT_MASK_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG0A, data, VINDPM_INT_MASK);
}

static int bq25600_set_iindpm_int(struct bq25600 *chip, int en)
{
	u8 data = 0;

	if (!en)
		data = 1 << IINDPM_INT_MASK_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG0A, data, IINDPM_INT_MASK);
}

static int bq25600_get_part_number(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG0B, &data);
	if (rc)
		return rc;

	data &= PN;
	data >>= PN_SHIFT;

	return data;
}

static int bq25600_get_device_revision(struct bq25600 *chip)
{
	u8 data;
	int rc;

	rc = bq25600_read(chip->client, BQ25600_REG0B, &data);
	if (rc)
		return rc;

	data &= DEV_REV;
	data >>= DEV_REV_SHIFT;

	return data;
}

static int bq25600_get_register_dump(struct bq25600 *chip)
{
	u8 data;
	int rc;
	int i;
	char* buf = chip->buffer;

	buf[0] = '\0';

	snprintf(buf + strnlen(buf, DUMP_SIZE), DUMP_SIZE,
		 "[DUMP@] : ");

	for (i = 0; i < ARRAY_SIZE(regs_to_dump); i++) {
		rc = bq25600_read(chip->client, regs_to_dump[i], &data);
		if (rc) {
			dev_err(chip->dev, "dump_err: 0x%02x is error\n", regs_to_dump[i]);
		}
		snprintf(buf + strnlen(buf, DUMP_SIZE), DUMP_SIZE, "0x%02x=0x%02x ",
			regs_to_dump[i], data);
	}
	dev_info(chip->dev, "%s\n", buf);

	return 0;
}

static int bq25600_set_batfet_dis(struct bq25600 *chip, int en)
{
	u8 data = 0;

	if (en)
		data = 1;

	data <<= BATFET_DIS_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG07, data, BATFET_DIS);
}

static int bq25600_set_batfet_dly(struct bq25600 *chip, int en)
{
	u8 data = 0;

	if (en)
		data = 1;

	data <<= BATFET_DLY_SHIFT;

	return bq25600_masked_write(chip->client,
			BQ25600_REG07, data, BATFET_DLY);
}

static int bq25600_handle_watchdog_fault(struct bq25600 *chip)
{
	dev_info(chip->dev, "fault : watchdog\n");

	return 0;
}

static int bq25600_handle_boost_fault(struct bq25600 *chip)
{
	dev_info(chip->dev, "fault : boost\n");

	if (chip->otg_boost_enabled) {
		dev_info(chip->dev, "force disable otg boost\n");

		bq25600_set_otg_config(chip, 0);
		chip->otg_boost_enabled = false;
	}

	return 0;
}

static int bq25600_handle_chrg_fault(struct bq25600 *chip, int type)
{
	switch (type) {
	case 1:
		dev_info(chip->dev, "fault : input fault\n");
		break;
	case 2:
		dev_info(chip->dev, "fault : thermal shutdown\n");
		break;
	case 3:
		dev_info(chip->dev, "fault : safety timer\n");
		break;
	default:
		break;
	}

	return 0;
}

static int bq25600_handle_bat_fault(struct bq25600 *chip)
{
	dev_info(chip->dev, "fault : battery overvoltage\n");

	return 0;
}

static int bq25600_handle_ntc_fault(struct bq25600 *chip, int type)
{
	switch (type) {
	case 0:
		break;
	case 2:
		dev_info(chip->dev, "fault : warm\n");
		break;
	case 3:
		dev_info(chip->dev, "fault : cool\n");
		break;
	case 5:
		dev_info(chip->dev, "fault : cold\n");
		break;
	case 6:
		dev_info(chip->dev, "fault : hot\n");
		break;
	default:
		dev_info(chip->dev, "fault : ntc unknown\n");
		break;
	}

	return 0;
}

static irqreturn_t bq25600_irq_handler(int irq, void *data)
{
	struct bq25600 *chip = data;
	int rc;
	u8 stat;
	u8 fault;

	dev_info(chip->dev, "bq25600_irq_handler\n");

	rc = bq25600_read(chip->client, BQ25600_REG08, &stat);
	if (rc)
		return IRQ_NONE;
	rc = bq25600_read(chip->client, BQ25600_REG09, &fault);
	if (rc)
		return IRQ_NONE;

	dev_info(chip->dev, "stat=0x%02x, fault=0x%02x\n", stat, fault);

	if (fault & WATCHDOG_FAULT)
		bq25600_handle_watchdog_fault(chip);

	if (fault & BOOST_FAULT)
		bq25600_handle_boost_fault(chip);

	if (fault & CHRG_FAULT)
		bq25600_handle_chrg_fault(chip,
			(fault & CHRG_FAULT) >> CHRG_FAULT_SHIFT);

	if (fault & BAT_FAULT)
		bq25600_handle_bat_fault(chip);

	if (fault & NTC_FAULT)
		bq25600_handle_ntc_fault(chip,
			(fault & NTC_FAULT) >> NTC_FAULT_SHIFT);

	/* notify to battery driver */
	if (!chip->is_slave)
		power_supply_changed(&chip->psy);

	return IRQ_HANDLED;
}

static enum power_supply_property bq25600_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE,
};

static char *bq25600_supplied_to[] = {
	"ac",
};

static int bq25600_status(struct bq25600 *chip)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;

	if (!bq25600_get_vbus_gd(chip))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	switch (bq25600_get_chrg_stat(chip)) {
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

static int bq25600_present(struct bq25600 *chip)
{
	int vbus_stat = bq25600_get_vbus_stat(chip);

	if (vbus_stat < 0)
		return 0;

	if (vbus_stat && vbus_stat < VBUS_STAT_OTG)
		return 1;

	return 0;
}

static int bq25600_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct bq25600 *chip = container_of(psy, struct bq25600, psy);
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq25600_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq25600_present(chip);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq25600_get_pg_stat(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = bq25600_get_iterm(chip);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = bq25600_get_ichg(chip);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = bq25600_get_iindpm(chip);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = bq25600_get_vreg(chip);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq25600_get_en_hiz(chip);
		if (val->intval)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = bq25600_get_chg_config(chip);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		val->intval = bq25600_get_en_timer(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGER_DUMP:
		val->intval = bq25600_get_register_dump(chip);
		break;
	case POWER_SUPPLY_PROP_SHIP_MODE:
		val->intval = chip->ship_mode;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int bq25600_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct bq25600 *chip = container_of(psy, struct bq25600, psy);
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		rc = bq25600_set_iterm(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		chip->battery_ua = val->intval;
		rc = bq25600_set_ichg(chip, chip->battery_ua / 1000);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		chip->input_ua = val->intval;
		rc = bq25600_set_iindpm(chip, chip->input_ua / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		chip->battery_uv = val->intval;
		rc = bq25600_set_vreg(chip, chip->battery_uv / 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		chip->enabled = val->intval;
		rc = bq25600_set_en_hiz(chip, chip->enabled ? 0 : 1);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		chip->battery_charging_enabled = val->intval;
		rc = bq25600_set_chg_config(chip,
				chip->battery_charging_enabled);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		rc = bq25600_set_en_timer(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_SHIP_MODE:
		chip->ship_mode = val->intval;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

int bq25600_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
	case POWER_SUPPLY_PROP_SHIP_MODE:
		rc = 1;
		break;
	default:
		break;
	}

	return rc;
}

static int bq25600_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct bq25600 *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	battery_log(BAT_LOG_CRTI, "[bq25600] OTG_BOOST Enable\n");

	chip->otg_boost_enabled = true;

	bq25600_set_boost_lim(chip, 1200);
	bq25600_set_boostv(chip, 5000);
	bq25600_set_otg_config(chip, 1);

	return rc;
}

static int bq25600_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct bq25600 *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	battery_log(BAT_LOG_CRTI, "[bq25600] OTG_BOOST Disable\n");

	bq25600_set_otg_config(chip, 0);

	chip->otg_boost_enabled = false;

	return rc;
}

static int bq25600_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct bq25600 *chip = rdev_get_drvdata(rdev);
	int enabled;

	enabled = bq25600_get_otg_config(chip);
	if (enabled < 0)
		enabled = 0;

	return enabled;
}

static int bq25600_otg_regulator_get_current_limit(struct regulator_dev *rdev)
{
	struct bq25600 *chip = rdev_get_drvdata(rdev);
	int limit = bq25600_get_boost_lim(chip);

	if (limit < 0)
		return 0;

	return limit * 1000;
}

static int bq25600_otg_regulator_set_current_limit(struct regulator_dev *rdev, int min, int max)
{
	struct bq25600 *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = bq25600_set_boost_lim(chip, max / 1000);

	return rc;
}

struct regulator_ops bq25600_otg_regulator_ops = {
	.enable	= bq25600_otg_regulator_enable,
	.disable = bq25600_otg_regulator_disable,
	.is_enabled = bq25600_otg_regulator_is_enable,
	.get_current_limit = bq25600_otg_regulator_get_current_limit,
	.set_current_limit = bq25600_otg_regulator_set_current_limit,
};

static int bq25600_regulator_init(struct bq25600 *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &bq25600_otg_regulator_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		chip->otg_vreg.rdev =
			devm_regulator_register(chip->dev, &chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev, "failed to register regulator, rc=%d\n", rc);
		}
	}

	return rc;
}

static int debugfs_get_data(void *data, u64 *val)
{
	struct bq25600 *chip = data;
	int rc;
	u8 temp;

	rc = bq25600_read(chip->client, chip->debug_addr, &temp);
	if (rc)
		return -EAGAIN;

	*val = temp;

	return 0;
}

static int debugfs_set_data(void *data, u64 val)
{
	struct bq25600 *chip = data;
	int rc;
	u8 temp;

	temp = (u8)val;
	rc = bq25600_write(chip->client, chip->debug_addr, temp);
	if (rc)
		return -EAGAIN;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(data_debugfs_ops,
	debugfs_get_data, debugfs_set_data, "0x%02llx\n");


static int dump_debugfs_show(struct seq_file *m, void *start)
{
	struct bq25600 *chip = m->private;
	u8 data;
	int rc;
	int i;

	for (i = 0; i < ARRAY_SIZE(regs_to_dump); i++) {
		rc = bq25600_read(chip->client, regs_to_dump[i], &data);
		if (rc) {
			seq_printf(m, "0x%02x=error\n", regs_to_dump[i]);
			continue;
		}

		seq_printf(m, "0x%02x=0x%02x\n", regs_to_dump[i], data);
	}

	return 0;
}

static int dump_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq25600 *chip = inode->i_private;

	return single_open(file, dump_debugfs_show, chip);
}

static const struct file_operations dump_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= dump_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int create_debugfs_entries(struct bq25600 *chip)
{
	struct dentry *ent;

	chip->debugfs = debugfs_create_dir(chip->psy.name, NULL);
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

	return 0;
}

static int bq25600_gpio_init(struct bq25600 *chip)
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

static int bq25600_hw_init(struct bq25600 *chip)
{
	int rc;

	if (gpio_is_valid(chip->irq_gpio)) {
		rc = devm_gpio_request_one(chip->dev, chip->irq_gpio, GPIOF_DIR_IN,
					"bq25600");
		if (rc) {
			dev_err(chip->dev, "failed to request gpio, rc=%d\n", rc);
			return rc;
		}
	}

	rc = bq25600_set_batfet_dis(chip, 0);
	if (rc)
		dev_err(chip->dev, "failed to set batfet, rc=%d\n", rc);

	rc = bq25600_get_en_hiz(chip);
	if (rc) {
		rc = bq25600_set_en_hiz(chip, 0);
		if (rc)
			dev_err(chip->dev, "failed to set en_hiz, rc=%d\n", rc);
	}

	rc = bq25600_set_iterm(chip, chip->iterm_ma);
	if (rc)
		dev_err(chip->dev, "failed to set iterm, rc=%d\n", rc);

	rc = bq25600_set_vindpm(chip, chip->vindpm_mv);
	if (rc)
		dev_err(chip->dev, "failed to set vindpm, rc=%d\n", rc);

	rc = bq25600_set_ovp(chip, chip->ovp_mv);
	if (rc)
		dev_err(chip->dev, "failed to set ovp, rc=%d\n", rc);

	rc = bq25600_set_watchdog(chip, chip->watchdog_sec);
	if (rc)
		dev_err(chip->dev, "failed to set watchdog, rc=%d\n", rc);

	rc = bq25600_set_vindpm_int(chip, chip->vindpm_int);
	if (rc)
		dev_err(chip->dev, "failed to set vindpm_int, rc=%d\n", rc);

	rc = bq25600_set_iindpm_int(chip, chip->iindpm_int);
	if (rc)
		dev_err(chip->dev, "failed to set iindpm_int, rc=%d\n", rc);

	rc = bq25600_set_chg_timer(chip, chip->chg_timer_hrs);
	if (rc)
		dev_err(chip->dev, "failed to set chg_timer, rc=%d\n", rc);

	rc = bq25600_set_en_timer(chip, chip->chg_timer_disabled ? 0 : 1);
	if (rc)
		dev_err(chip->dev, "failed to set en_timer, rc=%d\n", rc);

	rc = bq25600_set_vreg(chip, chip->vreg_mv);
	if (rc)
		dev_err(chip->dev, "failed to set vreg, rc=%d\n", rc);

	rc = bq25600_set_vrechg(chip, chip->vrechg_mv);
	if (rc)
		dev_err(chip->dev, "failed to set vrechg, rc=%d\n", rc);

	return 0;
}

static int bq25600_parse_dt(struct bq25600 *chip)
{
	struct device_node *np = chip->dev->of_node;
	int rc = 0;

	if (!np)
		return -ENODEV;

	chip->is_slave = of_property_read_bool(np, "lge,parallel-charger");

	chip->ship_mode_enabled = of_property_read_bool(np, "ship_mode_enabled");

	chip->irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0, NULL);
	if (!gpio_is_valid(chip->irq_gpio)) {
		dev_err(chip->dev, "failed to read irq-gpio\n");
		chip->irq_gpio = -EINVAL;
	}

	rc = of_property_read_u32(np, "ovp", &chip->ovp_mv);
	if (rc)
		chip->ovp_mv = 10500;

	rc = of_property_read_u32(np, "vindpm", &chip->vindpm_mv);
	if (rc)
		chip->vindpm_mv = 4500;

	rc = of_property_read_u32(np, "vreg", &chip->vreg_mv);
	if (rc)
		chip->vreg_mv = 4400;

	rc = of_property_read_u32(np, "vrechg", &chip->vrechg_mv);
	if (rc)
		chip->vrechg_mv = 100;

	rc = of_property_read_u32(np, "iterm", &chip->iterm_ma);
	if (rc)
		chip->iterm_ma = 100;

	rc = of_property_read_u32(np, "chg_timer", &chip->chg_timer_hrs);
	if (rc)
		chip->chg_timer_hrs = 10;

	chip->chg_timer_disabled = of_property_read_bool(np, "chg_timer_disabled");

	rc = of_property_read_u32(np, "watchdog", &chip->watchdog_sec);
	if (rc)
		chip->watchdog_sec = 40;

	chip->vindpm_int = of_property_read_bool(np, "vindpm-int");
	chip->iindpm_int = of_property_read_bool(np, "iindpm-int");

	return 0;
}

static int bq25600_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bq25600 *chip;
	int rc;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

	chip->enabled = true;
	chip->battery_charging_enabled = true;
	chip->otg_boost_enabled = false;
	mutex_init(&chip->lock);

	chip->psy.name = "bq25600";
	chip->psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->psy.properties = bq25600_properties;
	chip->psy.num_properties = ARRAY_SIZE(bq25600_properties);
	chip->psy.supplied_to = bq25600_supplied_to;
	chip->psy.num_supplicants = ARRAY_SIZE(bq25600_supplied_to);
	chip->psy.get_property = bq25600_get_property;
	chip->psy.set_property = bq25600_set_property;
	chip->psy.property_is_writeable = bq25600_property_is_writeable;

	rc = bq25600_parse_dt(chip);
	if (rc)
		return rc;

	if (chip->is_slave) {
		chip->psy.name = "usb-parallel";

		/* check device exist */
		if (bq25600_get_part_number(chip) < 0)
			return -ENODEV;
		if (bq25600_get_device_revision(chip) < 0)
			return -ENODEV;
	}

	if (!chip->is_slave) {
		rc = bq25600_regulator_init(chip);
		if (rc)
			return rc;
	}

	bq25600_gpio_init(chip);

	rc = bq25600_hw_init(chip);
	if (rc)
		return rc;

	rc = power_supply_register(chip->dev, &chip->psy);
	if (rc) {
		dev_err(chip->dev, "failed to register power_supply, rc=%d\n", rc);
		return rc;
	}

	if (chip->irq_gpio != -EINVAL) {
		rc = devm_request_threaded_irq(chip->dev, gpio_to_irq(chip->irq_gpio),
				NULL, bq25600_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, chip->psy.name, chip);
		if (rc) {
			dev_err(chip->dev, "failed to request irq, rc=%d\n", rc);
			return rc;
		}
	}

	create_debugfs_entries(chip);

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	if (chip->is_slave)
		chr_control_register_slave(&chip->psy);
	else
		chr_control_register(&chip->psy);
#endif

	return rc;
}

static int bq25600_remove(struct i2c_client *client)
{
	struct bq25600 *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->psy);

	return 0;
}

static void bq25600_shutdown(struct i2c_client *client)
{
	struct bq25600 *chip = i2c_get_clientdata(client);
	int ship_mode = 0;

	if (chip->otg_boost_enabled) {
		bq25600_set_otg_config(chip, 0);
		chip->otg_boost_enabled = false;
	}

	if (chip->is_slave)
		ship_mode = 1;

	if (chip->ship_mode_enabled && chip->ship_mode)
		ship_mode = 1;

	if (ship_mode) {
		bq25600_set_batfet_dis(chip, 1);
		bq25600_set_batfet_dly(chip, 0);
	}

	return;
}

static const struct of_device_id bq25600_of_match[] = {
	{
		.compatible = "ti,bq25600",
	},
};

static const struct i2c_device_id bq25600_i2c_id[] = {
	{
		.name = "bq25600",
		.driver_data = 0,
	},
};

static struct i2c_driver bq25600_driver = {
	.probe = bq25600_probe,
	.remove = bq25600_remove,
	.shutdown = bq25600_shutdown,
	.driver = {
		.name = "bq25600",
		.of_match_table = bq25600_of_match,
	},
	.id_table = bq25600_i2c_id,
};

static int __init bq25600_init(void)
{
	return i2c_add_driver(&bq25600_driver);
}

static void __exit bq25600_exit(void)
{
	i2c_del_driver(&bq25600_driver);
}

module_init(bq25600_init);
module_exit(bq25600_exit);
