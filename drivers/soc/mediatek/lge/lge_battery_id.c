/*
 *  lge_battery_id.c
 *
 *  LGE Battery Charger Interface Driver
 *
 *  Copyright (C) 2011 LG Electronics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#ifdef CONFIG_LGE_PM_USB_ID
#include <soc/mediatek/lge/lge_cable_id.h>
#endif
#include <soc/mediatek/lge/lge_battery_id.h>

#define MODULE_NAME "lge_battery_id"

struct battery_id_t {
	int id;
	char *name;
};

struct lge_battery_id_info {
	struct device *dev;
	struct power_supply psy;

	int id;
	char *name;
	int valid;

	/* common information */
	const char *model_name;
	int charge_full_design;

	/* cell information */
	const char *manufacturer;
};

static char *battery_id_name = NULL;

static const struct battery_id_t ids[] = {
	{ BATT_ID_UNKNOWN, "UNKNOWN" },
	{ BATT_ID_MISSING, "MISSING" },
	/* ADC */
	{ BATT_ID_LGC, "LGC" },
	{ BATT_ID_TOCAD, "TOCAD" },
	{ BATT_ID_ATL, "ATL" },
	{ BATT_ID_BYD, "BYD" },
	{ BATT_ID_LISHEN, "LISHEN" },
	/* Authentication IC */
	{ BATT_ID_DS2704_N, "DS2704_N" },
	{ BATT_ID_DS2704_L, "DS2704_L" },
	{ BATT_ID_DS2704_C, "DS2704_C" },
	{ BATT_ID_ISL6296_N, "ISL6296_N" },
	{ BATT_ID_ISL6296_L, "ISL6296_L" },
	{ BATT_ID_ISL6296_C, "ISL6296_C" },
	{ BATT_ID_ISL6296A_N, "ISL6296A_N" },
	{ BATT_ID_ISL6296A_L, "ISL6296A_L" },
	{ BATT_ID_ISL6296A_C, "ISL6296A_C" },
	{ BATT_ID_RA4301_VC0, "RA4301_VC0" },
	{ BATT_ID_RA4301_VC1, "RA4301_VC1" },
	{ BATT_ID_RA4301_VC2, "RA4301_VC2" },
	{ BATT_ID_SW3800_VC0, "SW3800_VC0" },
	{ BATT_ID_SW3800_VC1, "SW3800_VC1" },
	{ BATT_ID_SW3800_VC2, "SW3800_VC2" },
};

bool lge_battery_check(void)
{
	struct power_supply *psy;
	union power_supply_propval prop = {0,};
	int valid = 0;

	psy = power_supply_get_by_name("battery_id");
	if (psy) {
		psy->get_property(psy, POWER_SUPPLY_PROP_VALID_BATT_ID, &prop);
		valid = prop.intval;
	}

	return valid;
}

static int lge_battery_id_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct lge_battery_id_info *info = container_of(psy,
			struct lge_battery_id_info,
			psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_BATTERY_ID:
		val->intval = info->id;
		break;
	case POWER_SUPPLY_PROP_VALID_BATT_ID:
		val->intval = info->valid;
#ifdef CONFIG_LGE_PM_USB_ID
		if (lge_is_factory_cable_boot())
			val->intval = 1;
#endif
		break;
	case POWER_SUPPLY_PROP_CHECK_BATT_ID_FOR_AAT:
		val->intval = info->valid;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = info->charge_full_design;
		break;
	/* Properties of type `const char *' */
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = info->model_name;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = info->manufacturer;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property lge_battery_id_properties[] = {
	POWER_SUPPLY_PROP_BATTERY_ID,
	POWER_SUPPLY_PROP_VALID_BATT_ID,
	POWER_SUPPLY_PROP_CHECK_BATT_ID_FOR_AAT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	/* Properties of type `const char *' */
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static struct power_supply lge_battery_id_psy = {
	.name = "battery_id",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = lge_battery_id_get_property,
	.properties = lge_battery_id_properties,
	.num_properties = ARRAY_SIZE(lge_battery_id_properties),
};

static struct of_device_id lge_battery_id_match_table[] = {
	{ .compatible = "lge,battery-id" },
	{}
};

static const char *lge_battery_id_get_vendor(struct lge_battery_id_info *info)
{
	struct device_node *np = info->dev->of_node;
	struct property *prop;
	char *name = info->name;
	const char *cp;

	if (!name)
		return NULL;

	prop = of_find_property(np, "manufacturer", NULL);
	if (!prop)
		return NULL;

	for (cp = of_prop_next_string(prop, NULL); cp;
			cp = of_prop_next_string(prop, cp)) {
		if (!strcmp(name, cp))
			break;

		/* skip next string - this will be manufacturer */
		cp = of_prop_next_string(prop, cp);
	}

	if (cp)
		return of_prop_next_string(prop, cp);

	return NULL;
}

static int lge_battery_id_find_id(struct lge_battery_id_info *info)
{
	char *name = info->name;
	int i;

	if (!name)
		return BATT_ID_UNKNOWN;

	for (i = 0; i < ARRAY_SIZE(ids); i++) {
		if (!strcmp(name, ids[i].name))
			break;
	}

	if (i >= ARRAY_SIZE(ids))
		return BATT_ID_UNKNOWN;

	return ids[i].id;
}

static int lge_battery_id_parse_dt(struct lge_battery_id_info *info)
{
	struct device_node *np = info->dev->of_node;
	int rc;

	rc = of_property_read_string(np, "model-name",
			&info->model_name);
	if (rc)
		info->model_name = "BL-Xxx";

	rc = of_property_read_u32(np, "charge-full-design",
			&info->charge_full_design);
	if (rc)
		info->charge_full_design = 3000;
	info->charge_full_design *= 1000;	/* mAh -> uAh */

	info->name = battery_id_name;
	info->id = lge_battery_id_find_id(info);
	info->manufacturer = lge_battery_id_get_vendor(info);

	info->valid = info->manufacturer ? 1 : 0;

	return 0;
}

static int lge_battery_id_probe(struct platform_device *pdev)
{
	struct lge_battery_id_info *info;
	/*struct power_supply *psy;*/
	int ret = 0;

	info = devm_kzalloc(&pdev->dev, sizeof(struct lge_battery_id_info),
			GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "failed: allocation memory\n");
		return -ENOMEM;
	}

	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

	ret = lge_battery_id_parse_dt(info);
	if (ret)
		goto err_probe;

	info->psy = lge_battery_id_psy;
	ret = power_supply_register(&pdev->dev, &info->psy);
	if (ret) {
		dev_err(&pdev->dev, "failed: power supply register\n");
		goto err_probe;
	}

	dev_info(&pdev->dev, "Battery: %s, Vendor: %s, ID: %s\n",
			(info->model_name ? info->model_name : "unknown"),
			(info->manufacturer ? info->manufacturer : "unknown"),
			(info->name ? info->name : "none"));

	return ret;

err_probe:
	return ret;
}

static int lge_battery_id_remove(struct platform_device *pdev)
{
	struct lge_battery_id_info *info = platform_get_drvdata(pdev);

	power_supply_unregister(&info->psy);

	return 0;
}

static struct platform_driver lge_battery_id_driver = {
	.driver = {
		.name   = MODULE_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = lge_battery_id_match_table,
	},
	.probe  = lge_battery_id_probe,
	.remove = lge_battery_id_remove,
};

static int __init battery_id_setup(char *name)
{
        battery_id_name = name;

        return 0;
}
early_param("lge.battid", battery_id_setup);

static int __init lge_battery_id_init(void)
{
	return platform_driver_register(&lge_battery_id_driver);
}

static void __exit lge_battery_id_exit(void)
{
	platform_driver_unregister(&lge_battery_id_driver);
}

module_init(lge_battery_id_init);
module_exit(lge_battery_id_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cowboy");
MODULE_DESCRIPTION("LGE Battery ID Checker");
