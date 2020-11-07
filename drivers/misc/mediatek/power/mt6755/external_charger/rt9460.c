/*
* rt9460-charger.c -- charger driver controlled by i2c
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
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/delay.h>

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
#include "charging_hw_external_charger.h"
#endif

#ifdef CONFIG_LGE_BOOT_MODE
#include <soc/mediatek/lge/lge_boot_mode.h>
#endif

/*ctrl1*/
#define RT9460_SWFREQ_MASK		0x80
#define RT9460_SWFREQ_SHFT		7
#define RT9460_ENSTAT_MASK		0x40
#define RT9460_ENSTAT_SHFT		6
#define RT9460_CHGSTAT_MASK		0x30
#define RT9460_CHGSTAT_SHFT		4
#define RT9460_BOOST_MASK		0x08
#define RT9460_BOOST_SHFT		3
#define RT9460_POWRRDY_MASK		0x04
#define RT9460_POWRRDY_SHFT		2
#define RT9460_OTGPINP_MASK		0x02
#define RT9460_OTGPINP_SHFT		1
#define RT9460_MIVRSTAT_MASK	0x01
#define RT9460_MIVRSTAT_SHFT	0

/*ctrl2*/
#define RT9460_IEOC_MASK		0xe0
#define RT9460_IEOC_SHFT		5
#define RT9460_OCP_MASK			0x10
#define RT9460_OCP_SHFT			4
#define RT9460_TEEN_MASK		0x08
#define RT9460_TEEN_SHFT		3
#define RT9460_IININT_MASK		0x04
#define RT9460_IININT_SHFT		2
#define RT9460_HZ_MASK			0x02
#define RT9460_HZ_SHFT			1
#define RT9460_OPAMODE_MASK		0x01
#define RT9460_OPAMODE_SHFT		0

/*ctrl3*/
#define RT9460_VOREG_MASK		0xfc
#define RT9460_VOREG_SHFT		2
#define RT9460_OTGPL_MASK		0x02
#define RT9460_OTGPL_SHFT		1
#define RT9460_OTGEN_MASK		0x01
#define RT9460_OTGEN_SHFT		0

/*ctrl5*/
#define RT9460_ENTMR_MASK		0x80
#define RT9460_ENTMR_SHFT		7
#define RT9460_OTGOC_MASK		0x40
#define RT9460_OTGOC_SHFT		6
#define RT9460_SYSMIN_MASK		0x30
#define RT9460_SYSMIN_SHFT		4
#define RT9460_IPREC_MASK		0x0F
#define RT9460_IPREC_SHFT		0

/*ctrl6*/
#define RT9460_ICHRG_MASK		0xf0
#define RT9460_ICHRG_SHFT		4
#define RT9460_OSCSS_MASK		0x08
#define RT9460_OSCSS_SHFT		3
#define RT9460_VPREC_MASK		0x07
#define RT9460_VPREC_SHFT		0

/*ctrl7*/
#define RT9460_CCJEITA_MASK		0x80
#define RT9460_CCJEITA_SHFT		7
#define RT9460_BATDEN_MASK		0x40
#define RT9460_BATDEN_SHFT		6
#define RT9460_CHIPEN_MASK		0x20
#define RT9460_CHIPEN_SHFT		5
#define RT9460_CHGEN_MASK		0x10
#define RT9460_CHGEN_SHFT		4
#define RT9460_VMREG_MASK		0x0f
#define RT9460_VMREG_SHFT		0
#define RT9460_TSHC_MASK		0x09
#define RT9460_TSWC_MASK		0x06

/*ctrl9*/
#define RT9460_ENPPCTRL_MASK	0x20
#define RT9460_ENPPCTRL_SHFT	5
#define RT9460_MIVREN_MASK		0x10
#define RT9460_MIVREN_SHFT		4
#define RT9460_MIVRLVL_MASK		0x0f
#define RT9460_MIVRLVL_SHFT		0

/*ctrl10*/
#define RT9460_CLRDP_MASK		0x80
#define RT9460_CLRDP_SHFT		7
#define RT9460_DPSTAT_MASK		0x40
#define RT9460_DPSTAT_SHFT		6
#define RT9460_WTFC_MASK		0x38
#define RT9460_WTFC_SHFT		3
#define RT9460_WTPRC_MASK		0x06
#define RT9460_WTPRC_SHFT		1
#define RT9460_TMRP_MASK		0x01
#define RT9460_TMRP_SHFT		0

/*ctrl11*/
#define RT9460_CHGAICR_MASK		0xf8
#define RT9460_CHGAICR_SHFT		3
#define RT9460_DEADBAT_MASK		0x07
#define RT9460_DEADBAT_SHFT		0

/*ctrl12*/
#define RT9460_EOCTMR_MASK		0xC0
#define RT9460_EOCTMR_SHFT		6
#define RT9460_WUTMR_MASK		0x38
#define RT9460_WUTMR_SHFT		3
#define RT9460_WUTMREN_MASK		0x04
#define RT9460_WUTMREN_SHFT		2
#define RT9460_IRQPUL_MASK		0x02
#define RT9460_IRQPUL_SHFT		1
#define RT9460_IRQREZ_MASK		0x01
#define RT9460_IRQREZ_SHFT		0

/*ctrl13*/
#define RT9460_WDTEN_MASK		0x80
#define RT9460_WDTEN_SHFT		7

/*DPDM*/
#define RT9460_CHGTYPE_MASK		0xe0
#define RT9460_CHGTYPE_SHFT		5
#define RT9460_IICSEL_MASK		0x18
#define RT9460_IICSEL_SHFT		3
#define RT9460_CHG2DET_MASK		0x04
#define RT9460_CHG2DET_SHFT		2
#define RT9460_CHG1DET_MASK		0x02
#define RT9460_CHG1DET_SHFT		1
#define RT9460_CHGRUN_MASK		0x01
#define RT9460_CHGRUN_SHFT		0

/*MASK1*/
#define RT9460_CHTERMTMRIM_MASK		0x04

/*stat irq mask*/
#define RT9460_MIVRI_MASK	0x04
#define RT9460_PWRRDYI_MASK	0x08
#define RT9460_TSWCI_MASK	0x60
#define RT9460_TSHCI_MASK	0x90
#define RT9460_TSEVENT_MASK	0xf0

enum {
	RT9460_REG_CTRL1,
	RT9460_REG_RANGE1_START = RT9460_REG_CTRL1,
	RT9460_REG_CTRL2,
	RT9460_REG_CTRL3,
	RT8460_REG_ID,
	RT9460_REG_CTRL4,
	RT9460_REG_CTRL5,
	RT9460_REG_CTRL6,
	RT9460_REG_CTRL7,
	RT9460_REG_IRQ1,
	RT9460_REG_IRQ2,
	RT9460_REG_IRQ3,
	RT9460_REG_MASK1,
	RT9460_REG_MASK2,
	RT9460_REG_MASK3,
	RT9460_REG_DPDM = 0x0e,
	RT9460_REG_RANGE1_END = RT9460_REG_DPDM,

/* hidden bit for otg mode */
	RT9460_REG_1C = 0x1c,
	RT9460_REG_HIDDEN = 0x18,

	RT9460_REG_CTRL9 = 0x21,
	RT9460_REG_RANGE2_START = RT9460_REG_CTRL9,
	RT9460_REG_CTRL10,
	RT9460_REG_CTRL11,
	RT9460_REG_CTRL12,
	RT9460_REG_CTRL13,
	RT9460_REG_STATIRQ,
	RT9460_REG_STATIRQMASK,
	RT9460_REG_RANGE2_END = RT9460_REG_STATIRQMASK,
	RT9450_REG_31 = 0x31,
};

/* irq event */
enum {
	CHGEVENT_BSTLOWVI = 5,
	CHGEVENT_BSTOLI,
	CHGEVENT_BSTVINOVI,

	CHGEVENT_SYSWAKEUPI,
	CHGEVENT_CHTREGI,
	CHGEVENT_CHTMRI,
	CHGEVENT_CHRCHGI,
	CHGEVENT_CHTERMI,
	CHGEVENT_CHBATOVI,
	CHGEVENT_CHBADI,
	CHGEVENT_CHRVPI = 15,

	CHGEVENT_BATABI,
	CHGEVENT_SYSUVPI,
	CHGEVENT_CHTERMTMRI,
	CHGEVENT_WATCHDOGI = 20,
	CHGEVENT_WAKEUPI,
	CHGEVENT_VIMOVPI,
	CHGEVENT_TSDI,
	CHGEVENT_MAX,
};
#define RT9460_RESERVED_IRQ_MASK 0x8001F
#define DUMP_SIZE 256

struct rt9460_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct rt9460 {
	struct i2c_client *client;
	struct device *dev;

	struct power_supply psy;
	struct rt9460_regulator otg_vreg;
	struct mutex lock;
	struct mutex voreg_lock;

	bool enabled;
	bool battery_charging_enabled;
	bool otg_boost_enabled;

	/* device tree */
	bool is_slave;
	int irq_gpio;
	int freq;
	bool te_en;
	bool iin_int;
	int vchg_mv;
	int vbst_mv;
	int ieoc_ma;
	int mivr;
	bool mivr_en;

	/* debugfs */
	struct dentry *debugfs;
	u32 debug_addr;

	/* dump buffer */
	char buffer[DUMP_SIZE];
};

static int rt9460_read(struct i2c_client *client, u8 reg, u8 *data)
{
	int rc;

	rc = i2c_smbus_read_byte_data(client, reg);
	if (rc < 0)
		dev_err(&client->dev, "failed to read. rc=%d\n", rc);

	*data = (rc & 0xFF);

	return rc < 0 ? rc : 0;
}

static int rt9460_write(struct i2c_client *client, u8 reg, u8 data)
{
	int rc;

	rc = i2c_smbus_write_byte_data(client, reg, data);
	if (rc < 0)
		dev_err(&client->dev, "failed to write. rc=%d\n", rc);

	return rc;
}

static int rt9460_masked_write(struct i2c_client *client, u8 reg, u8 data, u8 mask)
{
	struct rt9460 *chip = i2c_get_clientdata(client);
	u8 tmp;
	int rc = 0;

	mutex_lock(&chip->lock);

	rc = rt9460_read(client, reg, &tmp);
	if (rc < 0)
		goto out;

	tmp = (data & mask) | (tmp & (~mask));
	rc = rt9460_write(client, reg, tmp);

out:
	mutex_unlock(&chip->lock);
	return rc;
}

/* TODO : do additional work here [START] */
static int rt9460_get_hiz_en(struct rt9460 *chip)
{
        u8 regval = 0;
        int rc;

        rc = rt9460_read(chip->client, RT9460_REG_CTRL2, &regval);
        if (rc)
                return rc;

        regval &= RT9460_HZ_MASK;

        if (regval)
                return 1;

        return 0;
}

static int rt9460_set_hiz_en(struct rt9460 *chip, int en)
{
        u8 regval = 0;

        if (en)
                regval = 1;

        regval <<= RT9460_HZ_SHFT;

        return rt9460_masked_write(chip->client,
                                RT9460_REG_CTRL2, regval, RT9460_HZ_MASK);
}

static void rt9460_plug_irq_handler(void *data)
{
	struct rt9460 *chip = data;
	int rc;
	u8 pwrrdy;

	rc = rt9460_read(chip->client, RT9460_REG_CTRL1, &pwrrdy);

	if (rc) {
		dev_err(chip->dev, "rt9460 i2c read fail\n");
	}

	if (pwrrdy & RT9460_POWRRDY_MASK) {
		dev_info(chip->dev, "cable in\n");
		rt9460_masked_write(chip->client, RT9460_REG_CTRL2, RT9460_IININT_MASK, RT9460_IININT_MASK);

		if (!chip->enabled)
			rt9460_set_hiz_en(chip, !chip->enabled);
	} else {
		dev_info(chip->dev, "cable out\n");
	}
}

static void rt9460_irq_event_handler(void *data, int eventno)
{
	struct rt9460 *chip = data;

	dev_info(chip->dev, "IRQ : eventno=%02d\n", eventno);

	switch (eventno) {
	case CHGEVENT_CHRCHGI:
		dev_info(chip->dev, "IRQ : recharging event occured\n");
		break;
	case CHGEVENT_CHTERMI:
		dev_info(chip->dev, "IRQ : charging terminated\n");
		break;
	case CHGEVENT_CHTERMTMRI:
		dev_info(chip->dev, "IRQ : EOC occured\n");
		break;
	case CHGEVENT_WATCHDOGI:
		dev_info(chip->dev, "IRQ : watchdog timer\n");
		rt9460_masked_write(chip->client, RT9460_REG_CTRL13, 0, RT9460_WDTEN_MASK);
		rt9460_masked_write(chip->client, RT9460_REG_CTRL13, RT9460_WDTEN_MASK, RT9460_WDTEN_MASK);
		break;
	case CHGEVENT_CHTMRI:
		dev_info(chip->dev, "IRQ : safety timeout\n");
		break;
	default:
		dev_err(chip->dev, "Unexpected IRQ\n");
		break;
	}
}

static inline int rt9460_irq_release(struct rt9460 *chip)
{
	int rc;

	/* Enable wake-up timer to trigger internal osc to be turnned on */
	rc = rt9460_masked_write(chip->client, RT9460_REG_CTRL12, RT9460_WUTMREN_MASK, RT9460_WUTMREN_MASK);

	msleep(10);

	/* set irq_rez bit to trigger irq pin to be released */
	rc = rt9460_masked_write(chip->client, RT9460_REG_CTRL12, RT9460_IRQREZ_MASK, RT9460_IRQREZ_MASK);

	/* Disable wake-up timer */
	return rt9460_masked_write(chip->client, RT9460_REG_CTRL12, 0, RT9460_WUTMREN_MASK);
}
/* TODO : do additional work here [END] */

static irqreturn_t rt9460_irq_handler(int irq, void *data)
{
	struct rt9460 *chip = data;
	int rc, i;
	u8 stat;
	u8 regval[3];
	u32 irq_event;

	dev_info(chip->dev, "rt9460_irq_handler\n");

	rc = rt9460_read(chip->client, RT9460_REG_STATIRQ, &stat);
	if (rc)
		goto irq_fin;

	rc = rt9460_read(chip->client, RT9460_REG_IRQ1, &regval[0]);
	if (rc)
		goto irq_fin;
	rc = rt9460_read(chip->client, RT9460_REG_IRQ2, &regval[1]);
	if (rc)
		goto irq_fin;
	rc = rt9460_read(chip->client, RT9460_REG_IRQ3, &regval[2]);
	if (rc)
		goto irq_fin;
	irq_event = regval[0] << 16 | regval[1] << 8 | regval[2];
	irq_event &= ~RT9460_RESERVED_IRQ_MASK;

	dev_info(chip->dev, "stat=0x%x, irq=0x%x\n", stat, irq_event);

	if (stat & RT9460_PWRRDYI_MASK)
		rt9460_plug_irq_handler(chip);

	if (irq_event) {
		for (i = 5; i < CHGEVENT_MAX; i++) {
			if (irq_event & (1 << i))
				rt9460_irq_event_handler(chip, i);
		}
	}

irq_fin:
	rc = rt9460_irq_release(chip);
	if (rc)
		dev_err(chip->dev, "write irq release bit fail\n");

	return IRQ_HANDLED;
}

static enum power_supply_property rt9460_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
};

static int rt9460_get_ieoc(struct rt9460 *chip)
{
	u8 regval = 0;
	int rc;

	rc = rt9460_read(chip->client, RT9460_REG_CTRL2, &regval);
	if (rc)
		return rc;

	regval &= RT9460_IEOC_MASK;
	regval >>= RT9460_IEOC_SHFT;

	return (regval + 2) * 50;
}

static int rt9460_set_ieoc(struct rt9460 *chip, int ma)
{
	u8 regval = 0;

	if (ma < 100)
		ma = 100;
	else if (ma > 450)
		ma = 450;

	regval = (ma / 50) - 2;
	regval <<= RT9460_IEOC_SHFT;

	return rt9460_masked_write(chip->client,
				RT9460_REG_CTRL2, regval, RT9460_IEOC_MASK);
}

#define ICHRG_MIN_MA 1250
#define ICHRG_MAX_MA 3125
#define ICHRG_STEP_MA 125
static int rt9460_get_ichrg(struct rt9460 *chip)
{
	u8 regval = 0;
	int rc;

	rc = rt9460_read(chip->client, RT9460_REG_CTRL6, &regval);
	if (rc)
		return rc;

	regval &= RT9460_ICHRG_MASK;
	regval >>= RT9460_ICHRG_SHFT;

	return (regval * ICHRG_STEP_MA) + ICHRG_MIN_MA;
}

static int rt9460_set_ichrg(struct rt9460 *chip, int ma)
{
	u8 regval = 0;

	if (ma < ICHRG_MIN_MA)
		ma = ICHRG_MIN_MA;
	if (ma > ICHRG_MAX_MA)
		ma = ICHRG_MAX_MA;

	regval = (ma - ICHRG_MIN_MA) / ICHRG_STEP_MA;
	regval <<= RT9460_ICHRG_SHFT;

	return rt9460_masked_write(chip->client,
				RT9460_REG_CTRL6, regval, RT9460_ICHRG_MASK);
}

static int rt9460_get_iaicr(struct rt9460 *chip)
{
	u8 regval = 0;
	int rc;

	rc = rt9460_read(chip->client, RT9460_REG_CTRL11, &regval);
	if (rc)
		return rc;

	regval &= RT9460_CHGAICR_MASK;
	regval >>= RT9460_CHGAICR_SHFT;

	switch (regval) {
	case 0:
	case 1:
		return 100;
	case 2:
	case 3:
		return 150;
	case 4:
	case 5:
		return 500;
	default:
		return regval * 100;
	}
}

static int rt9460_set_iaicr(struct rt9460 *chip, int ma)
{
	u8 regval = 0;

	if (ma <= 100)
		regval = 0;
	else if (ma <= 150)
		regval = 2;
	else if (ma <= 500)
		regval = 4;
	else if (ma > 3000)
		regval = 30;
	else
		regval = ma / 100;

	regval <<= RT9460_CHGAICR_SHFT;

	return rt9460_masked_write(chip->client,
				RT9460_REG_CTRL11, regval, RT9460_CHGAICR_MASK);
}

#define VOREG_MIN_MV 3500
#define VOREG_MAX_MV 4620
#define VOREG_STEP_MV 20
#define VOREG_BOOST_MIN_MV 4425
#define VOREG_BOOST_MAX_MV 5825
#define VOREG_BOOST_STEP_MV 25
static int rt9460_get_voreg(struct rt9460 *chip)
{
	u8 regval = 0;
	int mv;
	int rc;

	rc = rt9460_read(chip->client, RT9460_REG_CTRL3, &regval);
	if (rc)
		return rc;

	regval &= RT9460_VOREG_MASK;
	regval >>= RT9460_VOREG_SHFT;

	mv = (regval * VOREG_STEP_MV) + VOREG_MIN_MV;
	if (mv > VOREG_MAX_MV)
		mv = VOREG_MAX_MV;

	return mv;
}

static int rt9460_set_voreg(struct rt9460 *chip, int mv, bool boost)
{
	u8 regval = 0;

	if (boost) {
		if (mv < VOREG_BOOST_MIN_MV)
			mv = VOREG_BOOST_MIN_MV;
		if (mv > VOREG_BOOST_MAX_MV)
			mv = VOREG_BOOST_MAX_MV;
		regval = (mv - VOREG_BOOST_MIN_MV) / VOREG_BOOST_STEP_MV;
	} else {
		if (mv < VOREG_MIN_MV)
			mv = VOREG_MIN_MV;
		if (mv > VOREG_MAX_MV)
			mv = VOREG_MAX_MV;
		regval = (mv - VOREG_MIN_MV) / VOREG_STEP_MV;
	}

	regval <<= RT9460_VOREG_SHFT;

	return rt9460_masked_write(chip->client,
				RT9460_REG_CTRL3, regval, RT9460_VOREG_MASK);
}

static int rt9460_get_chg_en(struct rt9460 *chip)
{
	u8 regval = 0;
	int rc;

	rc = rt9460_read(chip->client, RT9460_REG_CTRL7, &regval);
	if (rc)
		return rc;

	regval &= RT9460_CHGEN_MASK;

	if (regval)
		return 1;

	return 0;
}

static int rt9460_set_chg_en(struct rt9460 *chip, int en)
{
	u8 regval = 0;

	if (en)
		regval = 1;

	regval <<= RT9460_CHGEN_SHFT;

	return rt9460_masked_write(chip->client,
				RT9460_REG_CTRL7, regval, RT9460_CHGEN_MASK);
}

static int rt9460_present(struct rt9460 *chip)
{
	u8 regval = 0;
	int rc;

	rc = rt9460_read(chip->client, RT9460_REG_CTRL1, &regval);
	if (rc)
		return rc;

	if (regval & RT9460_POWRRDY_MASK)
		return 1;

	return 0;
}

static int rt9460_get_status(struct rt9460 *chip)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	int rc;
	u8 regval = 0;

	if (!rt9460_present(chip))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	rc = rt9460_read(chip->client, RT9460_REG_CTRL1, &regval);
	if (rc)
		return rc;

	regval &= RT9460_CHGSTAT_MASK;
	regval >>= RT9460_CHGSTAT_SHFT;

	if (regval == 1)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else if (regval == 2)
		status = POWER_SUPPLY_STATUS_FULL;
	else
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return status;
}

static int rt9460_online(struct rt9460 *chip)
{
	if (rt9460_present(chip) && !rt9460_get_hiz_en(chip) &&
			rt9460_get_chg_en(chip))
		return 1;

	return 0;
}

static int rt9460_get_otg_config(struct rt9460 *chip)
{
	u8 regval = 0;
	int rc;

	rc = rt9460_read(chip->client, RT9460_REG_CTRL2, &regval);
	if (rc)
		return rc;

	regval &= RT9460_OPAMODE_MASK;
	regval >>= RT9460_OPAMODE_SHFT;

	return regval;
}

static int rt9460_set_otg_config(struct rt9460 *chip, int en)
{
	u8 regval = 0;

	if (en)
		regval = 1;

	regval <<= RT9460_OPAMODE_SHFT;

	return rt9460_masked_write(chip->client,
				RT9460_REG_CTRL2, regval, RT9460_OPAMODE_MASK);
}

static int rt9460_set_freq(struct rt9460 *chip, int freq)
{
	u8 regval = 0;

	if (freq)
		regval = 1;

	regval <<= RT9460_SWFREQ_SHFT;

	return rt9460_masked_write(chip->client,
				RT9460_REG_CTRL1, regval, RT9460_SWFREQ_MASK);
}

static int rt9460_set_mivr_en(struct rt9460 *chip, int en)
{
	u8 regval = 1;

#ifdef CONFIG_LGE_BOOT_MODE
	if (lge_get_factory_boot()) {
		dev_info(chip->dev, "MIVR : factory mode -> disable MIVR\n");
		return 0;
	}
#endif

	if (en)
		regval = 0;

	regval <<= RT9460_MIVREN_SHFT;

	return rt9460_masked_write(chip->client,
				RT9460_REG_CTRL9, regval, RT9460_MIVREN_MASK);
}

static int rt9460_set_mivr(struct rt9460 *chip, int mivr)
{
	if (mivr < 0)
		mivr = 0;
	else if (mivr > 15)
		mivr = 15;

	mivr <<= RT9460_MIVRLVL_SHFT;

	return rt9460_masked_write(chip->client,
				RT9460_REG_CTRL9, mivr, RT9460_MIVRLVL_MASK);
}

static int rt9460_get_register_dump(struct rt9460 *chip)
{
	u8 regval = 0;
	int rc;
	int i;
	char* buf = chip->buffer;

	buf[0] = '\0';

	snprintf(buf +strnlen(buf, DUMP_SIZE), DUMP_SIZE,
			"rt9460_get_register_dump : ");

	for (i = RT9460_REG_RANGE1_START; i <= RT9460_REG_RANGE1_END; i++) {
		rc = rt9460_read(chip->client, i, &regval);
		if (rc) {
			dev_err(chip->dev, "dump_err: 0x%02x is error\n", i);
			continue;
		}
		snprintf(buf + strnlen(buf, DUMP_SIZE), DUMP_SIZE, "0x%02x=0x%02x ",
				i, regval);
	}

	for (i = RT9460_REG_RANGE2_START; i <= RT9460_REG_RANGE2_END; i++) {
		rc = rt9460_read(chip->client, i, &regval);
		if (rc) {
			dev_err(chip->dev, "dump_err: 0x%02x is error\n", i);
			continue;
		}
		snprintf(buf + strnlen(buf, DUMP_SIZE), DUMP_SIZE, "0x%02x=0x%02x ",
				i, regval);
	}

	dev_info(chip->dev, "%s\n", buf);

	return 0;
}

static int rt9460_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct rt9460 *chip = container_of(psy, struct rt9460, psy);
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = rt9460_get_status(chip);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = rt9460_online(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = rt9460_get_ieoc(chip);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = rt9460_get_ichrg(chip);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = rt9460_get_iaicr(chip);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = rt9460_get_voreg(chip);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = rt9460_get_hiz_en(chip);
		if (val->intval)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = rt9460_get_chg_en(chip);
		break;
#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	case POWER_SUPPLY_PROP_CHARGER_DUMP:
		val->intval = rt9460_get_register_dump(chip);
		break;
#endif
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int rt9460_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct rt9460 *chip = container_of(psy, struct rt9460, psy);
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		rc = rt9460_set_ieoc(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = rt9460_set_ichrg(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		rc = rt9460_set_iaicr(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		mutex_lock(&chip->voreg_lock);

		/* do not set voreg here when boost enabled */
		if (rt9460_get_otg_config(chip)) {
			mutex_unlock(&chip->voreg_lock);
			break;
		}

		rc = rt9460_set_voreg(chip, val->intval / 1000, false);

		mutex_unlock(&chip->voreg_lock);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		chip->enabled = val->intval;
		rc = rt9460_set_hiz_en(chip, chip->enabled ? 0 : 1);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		chip->battery_charging_enabled = val->intval;
		rc = rt9460_set_chg_en(chip, chip->battery_charging_enabled);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

int rt9460_property_is_writeable(struct power_supply *psy,
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
		rc = 1;
		break;
	default:
		break;
	}

	return rc;
}

static int rt9460_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct rt9460 *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chip->voreg_lock);

	chip->otg_boost_enabled = true;

//	rt9460_write(chip->client, RT9460_REG_HIDDEN, 0x32);
	rt9460_set_voreg(chip, chip->vbst_mv, true);
	rt9460_set_otg_config(chip, 1);
//	rt9460_write(chip->client, RT9460_REG_HIDDEN, 0x0e);

	mutex_unlock(&chip->voreg_lock);

	return rc;
}

static int rt9460_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct rt9460 *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chip->voreg_lock);

	rt9460_set_otg_config(chip, 0);
	rt9460_set_voreg(chip, chip->vchg_mv, false);
//	rt9460_write(chip->client, RT9460_REG_HIDDEN, 0x32);

	chip->otg_boost_enabled = false;

	mutex_unlock(&chip->voreg_lock);

	return rc;
}

static int rt9460_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct rt9460 *chip = rdev_get_drvdata(rdev);

	return chip->otg_boost_enabled;
}

struct regulator_ops rt9460_otg_regulator_ops = {
	.enable		= rt9460_otg_regulator_enable,
	.disable	= rt9460_otg_regulator_disable,
	.is_enabled	= rt9460_otg_regulator_is_enable,
};

static int rt9460_regulator_init(struct rt9460 *chip)
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
		chip->otg_vreg.rdesc.ops = &rt9460_otg_regulator_ops;
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
	struct rt9460 *chip = data;
	int rc;
	u8 temp;

	rc = rt9460_read(chip->client, chip->debug_addr, &temp);
	if (rc)
		return -EAGAIN;

	*val = temp;

	return 0;
}

static int debugfs_set_data(void *data, u64 val)
{
	struct rt9460 *chip = data;
	int rc;
	u8 temp;

	temp = (u8)val;
	rc = rt9460_write(chip->client, chip->debug_addr, temp);
	if (rc)
		return -EAGAIN;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(data_debugfs_ops,
	debugfs_get_data, debugfs_set_data, "0x%02llx\n");

static int dump_debugfs_show(struct seq_file *m, void *start)
{
	struct rt9460 *chip = m->private;
	u8 data;
	int rc;
	int i;

	for (i = RT9460_REG_RANGE1_START; i <= RT9460_REG_RANGE1_END; i++) {
		rc = rt9460_read(chip->client, i, &data);
		if (rc) {
			seq_printf(m, "0x%02x is error\n", i);
			continue;
		}
		seq_printf(m, "0x%02x=0x%02x\n", i, data);
	}

	for (i = RT9460_REG_RANGE2_START; i <= RT9460_REG_RANGE2_END; i++) {
		rc = rt9460_read(chip->client, i, &data);
		if (rc) {
			seq_printf(m, "0x%02x is error\n", i);
			continue;
		}
		seq_printf(m, "0x%02x=0x%02x\n", i, data);
	}

	return 0;
}

static int dump_debugfs_open(struct inode *inode, struct file *file)
{
	struct rt9460 *chip = inode->i_private;

	return single_open(file, dump_debugfs_show, chip);
}

static const struct file_operations dump_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= dump_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int create_debugfs_entries(struct rt9460 *chip)
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

static int rt9460_hw_init(struct rt9460 *chip)
{
	int rc;

	rc = devm_gpio_request_one(chip->dev, chip->irq_gpio, GPIOF_DIR_IN,
				"rt9460");
	if (rc)
		dev_err(chip->dev, "failed to request gpio, rc=%d\n", rc);

	/* set bit 7 at reg31 to prevent i2c transaction error */
	rc = rt9460_masked_write(chip->client, RT9450_REG_31, 0x80, 0x80);
	if (rc)
		dev_err(chip->dev, "failed to set i2c transaction w/a, rc=%d\n", rc);

	rc = rt9460_set_freq(chip, chip->freq);
	if (rc)
		dev_err(chip->dev, "failed to set freq, rc=%d\n", rc);

	rc = rt9460_set_ieoc(chip, chip->ieoc_ma);
	if (rc)
		dev_err(chip->dev, "failed to set ieoc, rc=%d\n", rc);

	rc = rt9460_set_mivr_en(chip, chip->mivr_en);
	if (rc)
		dev_err(chip->dev, "failed to set mivr_en, rc=%d\n", rc);

	rc = rt9460_set_mivr(chip, chip->mivr);
	if (rc)
		dev_err(chip->dev, "failed to set vmivr, rc=%d\n", rc);

	return rc;
}

static int rt9460_parse_dt(struct rt9460 *chip)
{
	struct device_node *np = chip->dev->of_node;
	int rc = 0;

	if (!np)
		return -ENODEV;

	chip->is_slave = of_property_read_bool(np, "slave-charger");

	chip->irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0, NULL);
	if (!gpio_is_valid(chip->irq_gpio)) {
		dev_err(chip->dev, "failed to read irq-gpio\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "freq", &chip->freq);
	if (rc)
		chip->freq = 0;

	rc = of_property_read_u32(np, "vchg", &chip->vchg_mv);
	if (rc)
		chip->vchg_mv = 4400;

	rc = of_property_read_u32(np, "vbst", &chip->vbst_mv);
	if (rc)
		chip->vbst_mv = 5000;

	rc = of_property_read_u32(np, "ieoc", &chip->ieoc_ma);
	if (rc)
		chip->ieoc_ma = 200;

	rc = of_property_read_u32(np, "mivr", &chip->mivr);
	if (rc)
		chip->mivr = 2;

	chip->te_en = of_property_read_bool(np, "te_en");
	chip->iin_int = of_property_read_bool(np, "iin_int");
	chip->mivr_en = of_property_read_bool(np, "mivr_en");
	return 0;
}

static int rt9460_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct rt9460 *chip;
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
	mutex_init(&chip->voreg_lock);

	chip->psy.name = "rt9460";
	chip->psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->psy.properties = rt9460_properties;
	chip->psy.num_properties = ARRAY_SIZE(rt9460_properties);
	chip->psy.get_property = rt9460_get_property;
	chip->psy.set_property = rt9460_set_property;
	chip->psy.property_is_writeable = rt9460_property_is_writeable;

	rc = rt9460_parse_dt(chip);
	if (rc)
		return rc;

	if (!chip->is_slave) {
		rc = rt9460_regulator_init(chip);
		if (rc)
			return rc;
	}

	rc = rt9460_hw_init(chip);
	if (rc)
		return rc;

	rc = devm_request_threaded_irq(chip->dev, gpio_to_irq(chip->irq_gpio),
			NULL, rt9460_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "rt9460", chip);
	if (rc) {
		dev_err(chip->dev, "failed to request irq, rc=%d\n", rc);
		return rc;
	}

	rc = power_supply_register(chip->dev, &chip->psy);
	if (rc) {
		dev_err(chip->dev, "failed to register power_supply, rc=%d\n", rc);
		return rc;
	}

	create_debugfs_entries(chip);

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	chr_control_register(&chip->psy);
#endif

	return rc;
}

static int rt9460_remove(struct i2c_client *client)
{
	struct rt9460 *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->psy);

	return 0;
}

static void rt9460_shutdown(struct i2c_client *client)
{
	struct rt9460 *chip = i2c_get_clientdata(client);

	if (chip->otg_boost_enabled) {
		chip->otg_boost_enabled = false;
		rt9460_set_otg_config(chip, 0);
	}

	return;
}

static const struct of_device_id rt9460_of_match[] = {
	{
		.compatible = "rt,rt9460",
	},
};

static const struct i2c_device_id rt9460_i2c_id[] = {
	{
		.name = "rt9460",
		.driver_data = 0,
	},
};

static struct i2c_driver rt9460_driver = {
	.probe = rt9460_probe,
	.remove = rt9460_remove,
	.shutdown = rt9460_shutdown,
	.driver = {
		.name = "rt9460",
		.of_match_table = rt9460_of_match,
	},
	.id_table = rt9460_i2c_id,
};

static int __init rt9460_init(void)
{
	return i2c_add_driver(&rt9460_driver);
}

static void __exit rt9460_exit(void)
{
	i2c_del_driver(&rt9460_driver);
}

module_init(rt9460_init);
module_exit(rt9460_exit);
