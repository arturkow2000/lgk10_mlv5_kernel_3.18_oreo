/*
* refcharger.c -- reference charger driver controlled by i2c
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

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
#include "charging_hw_external_charger.h"
#endif

/* TODO : define register here [START] */

/* TODO : define register here [END] */

#define SIZE_DUMP 256

const static int regs_to_dump[] = {
	/* TODO : fill register number to dump */
};

struct refcharger_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct refcharger {
	struct i2c_client *client;
	struct device *dev;

	struct power_supply psy;
	struct refcharger_regulator otg_vreg;
	struct mutex lock;

	bool enabled;
	bool battery_charging_enabled;
	bool otg_boost_enabled;

	unsigned int input_ua;
	unsigned int battery_ua;
	unsigned int battery_uv;

	char dump[SIZE_DUMP];

	/* device tree */
	bool is_slave;
	int irq_gpio;

	/* debugfs */
	struct dentry *debugfs;
	u32 debug_addr;
};

static int refcharger_read(struct i2c_client *client, u8 reg, u8 *data)
{
	int rc;

	rc = i2c_smbus_read_byte_data(client, reg);
	if (rc < 0)
		dev_err(&client->dev, "failed to read. rc=%d\n", rc);

	*data = (rc & 0xFF);

	return rc < 0 ? rc : 0;
}

static int refcharger_write(struct i2c_client *client, u8 reg, u8 data)
{
	int rc;

	rc = i2c_smbus_write_byte_data(client, reg, data);
	if (rc < 0)
		dev_err(&client->dev, "failed to write. rc=%d\n", rc);

	return rc;
}

static int refcharger_masked_write(struct i2c_client *client, u8 reg, u8 data, u8 mask)
{
	struct refcharger *chip = i2c_get_clientdata(client);
	u8 tmp;
	int rc = 0;

	mutex_lock(&chip->lock);

	rc = refcharger_read(client, reg, &tmp);
	if (rc < 0)
		goto out;

	tmp = (data & mask) | (tmp & (~mask));
	rc = refcharger_write(client, reg, tmp);

out:
	mutex_unlock(&chip->lock);
	return rc;
}

/* TODO : do additional work here [START] */

/* TODO : do additional work here [END] */

static irqreturn_t refcharger_irq_handler(int irq, void *data)
{
	struct refcharger *chip = data;

	dev_info(chip->dev, "refcharger_irq_handler\n");

	/* TODO : do something useful work here */

	return IRQ_HANDLED;
}

static enum power_supply_property refcharger_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	/*
	 * charger exist
	 * - handle this only if charging ic can report vbus exist
	 */
	POWER_SUPPLY_PROP_PRESENT,
	/*
	 * charger online
	 * - charger exist and charging enabled
	 */
	POWER_SUPPLY_PROP_ONLINE,
	/*
	 * battery charging current
	 * - fcc: fast charging current, constant charging current
	 */
	POWER_SUPPLY_PROP_CURRENT_MAX,
	/*
	 * input current
	 * - icl: input current limit
	 */
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	/*
	 * battery charging voltage
	 * - vfloat: floating voltage, regulation voltage, constant voltage
	 */
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
};

static char *refcharger_supplied_to[] = {
	"ac",
};

static int refcharger_status(struct refcharger *chip)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;

	/* TODO : get status and convert to POWER_SUPPLY */

	return status;
}

static int refcharger_present(struct refcharger *chip)
{
	int present = 0;

	/* TODO : get present */

	return present;
}

static int refcharger_online(struct refcharger *chip)
{
	int online = 0;

	/* TODO : get online */

	return online;
}

static int refcharger_dump(struct refcharger *chip)
{
	char *buf = chip->dump;
	int size = SIZE_DUMP;
	int written = 0;
	u8 data;
	int rc;
	int i;

	for (i = 0; i < ARRAY_SIZE(regs_to_dump); i++) {
		rc = refcharger_read(chip->client, regs_to_dump[i], &data);
		if (rc)
			continue;

		written = snprintf(buf, size, "0x%02x=0x%02x ",
				regs_to_dump[i], data);
		if (written < 0)
			continue;

		buf += written;
		size -= written;
	}

	dev_info(chip->dev, "dump: %s\n", buf);

	return 0;
}

static int refcharger_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct refcharger *chip = container_of(psy, struct refcharger, psy);
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = refcharger_status(chip);
		break;
	/*
	 * handle this only if charger presence can be repoerted.
	 * otherwise, return rc as non-zero.
	 */
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = refcharger_present(chip);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = refcharger_online(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->battery_ua;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = chip->input_ua;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = chip->battery_uv;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->enabled;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = chip->battery_charging_enabled;
		break;
#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
	case POWER_SUPPLY_PROP_CHARGER_DUMP:
		val->intval = refcharger_dump(chip);
		break;
#endif
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int refcharger_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct refcharger *chip = container_of(psy, struct refcharger, psy);
	int rc = 0;

	switch(psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		chip->battery_ua = val->intval;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		chip->input_ua = val->intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		chip->battery_uv = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		chip->enabled = val->intval;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		chip->battery_charging_enabled = val->intval;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

int refcharger_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc = 0;

	switch(psp) {
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

static int refcharger_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct refcharger *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	chip->otg_boost_enabled = true;

	/* TODO : turn on boost */

	return rc;
}

static int refcharger_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct refcharger *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	/* TODO : turn off boost */

	chip->otg_boost_enabled = false;

	return rc;
}

static int refcharger_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct refcharger *chip = rdev_get_drvdata(rdev);

	return chip->otg_boost_enabled;
}

struct regulator_ops refcharger_otg_regulator_ops = {
	.enable		= refcharger_otg_regulator_enable,
	.disable	= refcharger_otg_regulator_disable,
	.is_enabled	= refcharger_otg_regulator_is_enable,
};

static int refcharger_regulator_init(struct refcharger *chip)
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
		chip->otg_vreg.rdesc.ops = &refcharger_otg_regulator_ops;
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
	struct refcharger *chip = data;
	int rc;
	u8 temp;

	rc = refcharger_read(chip->client, chip->debug_addr, &temp);
	if (rc)
		return -EAGAIN;

	*val = temp;

	return 0;
}

static int debugfs_set_data(void *data, u64 val)
{
	struct refcharger *chip = data;
	int rc;
	u8 temp;

	temp = (u8)val;
	rc = refcharger_write(chip->client, chip->debug_addr, temp);
	if (rc)
		return -EAGAIN;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(data_debugfs_ops,
	debugfs_get_data, debugfs_set_data, "0x%02llx\n");

static int dump_debugfs_show(struct seq_file *m, void *start)
{
	struct refcharger *chip = m->private;
	u8 data;
	int rc;
	int i;

	for (i = 0; i < ARRAY_SIZE(regs_to_dump); i++) {
		rc = refcharger_read(chip->client, regs_to_dump[i], &data);
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
	struct refcharger *chip = inode->i_private;

	return single_open(file, dump_debugfs_show, chip);
}

static const struct file_operations dump_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= dump_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int create_debugfs_entries(struct refcharger *chip)
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

static int refcharger_hw_init(struct refcharger *chip)
{
	int rc;

	rc = devm_gpio_request_one(chip->dev, chip->irq_gpio, GPIOF_DIR_IN,
				"refcharger");
	if (rc)
		dev_err(chip->dev, "failed to request gpio, rc=%d\n", rc);

	/* TODO : init settings */

	return rc;
}

static int refcharger_parse_dt(struct refcharger *chip)
{
	struct device_node *np = chip->dev->of_node;
	int rc = 0;

	if (!np)
		return -ENODEV;

	chip->is_slave = of_property_read_bool(np, "lge,parallel-charger");

	chip->irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0, NULL);
	if (!gpio_is_valid(chip->irq_gpio)) {
		dev_err(chip->dev, "failed to read irq-gpio\n");
		return -EINVAL;
	}

	return rc;
}

static int refcharger_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct refcharger *chip;
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

	rc = refcharger_parse_dt(chip);
	if (rc)
		return rc;


	chip->psy.name = "refcharger";
	chip->psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->psy.properties = refcharger_properties;
	chip->psy.num_properties = ARRAY_SIZE(refcharger_properties);
	chip->psy.supplied_to = refcharger_supplied_to;
	chip->psy.num_supplicants = ARRAY_SIZE(refcharger_supplied_to);
	chip->psy.get_property = refcharger_get_property;
	chip->psy.set_property = refcharger_set_property;
	chip->psy.property_is_writeable = refcharger_property_is_writeable;

	/* if charger is slave, register as usb-parallel */
	if (chip->is_slave)
		chip->psy.name = "usb-parallel";

	if (!chip->is_slave) {
		/* Only Main Charger IC For OTG regulator*/
		rc = refcharger_regulator_init(chip);
		if (rc)
			return rc;
	}

	rc = refcharger_hw_init(chip);
	if (rc)
		return rc;

	rc = devm_request_threaded_irq(chip->dev, gpio_to_irq(chip->irq_gpio),
			NULL, refcharger_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "refcharger", chip);
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
	if (chip->is_slave)
		chr_control_register_slave(&chip->psy);
	else
		chr_control_register(&chip->psy);
#endif

	return rc;
}

static int refcharger_remove(struct i2c_client *client)
{
	struct refcharger *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->psy);

	return 0;
}

static void refcharger_shutdown(struct i2c_client *client)
{
	struct refcharger *chip = i2c_get_clientdata(client);

	if (chip->otg_boost_enabled) {
		/* TODO : force off boost here */
		chip->otg_boost_enabled = false;
	}

	return;
}

static const struct of_device_id refcharger_of_match[] = {
	{
		.compatible = "lge,refcharger",
	},
};

static const struct i2c_device_id refcharger_i2c_id[] = {
	{
		.name = "refcharger",
		.driver_data = 0,
	},
};

static struct i2c_driver refcharger_driver = {
	.probe = refcharger_probe,
	.remove = refcharger_remove,
	.shutdown = refcharger_shutdown,
	.driver = {
		.name = "refcharger",
		.of_match_table = refcharger_of_match,
	},
	.id_table = refcharger_i2c_id,
};

static int __init refcharger_init(void)
{
	return i2c_add_driver(&refcharger_driver);
}

static void __exit refcharger_exit(void)
{
	i2c_del_driver(&refcharger_driver);
}

module_init(refcharger_init);
module_exit(refcharger_exit);
