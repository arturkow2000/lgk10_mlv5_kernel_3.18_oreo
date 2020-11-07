/* Copyright (c) 2013-2014, LG Eletronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif

#include <soc/mediatek/lge/lge_power_sysfs.h>

#define MODULE_NAME "power-sysfs"

#define PWR_SYSFS_MANDATORY_MAX_NUM 5

/* Set sysfs node for non-using DT */
__weak struct power_sysfs_array lge_power_sysfs_path[] = {
	/* ADC */
	{"adc", "batt_id", "/sys/class/power_supply/battery_id/valid_batt_id"},
	{"adc", "batt_therm", "/sys/class/power_supply/battery/temp"},
	{"adc", "thermal", "/sys/class/thermal/"},
	{"adc", "usb_id", "/sys/devices/platform/battery/LGBM_AtCmdUsbidadc"},
	{"adc", "xo_therm", "NULL"},
#ifdef CONFIG_LGE_PM_VTS
	{"adc", "vts" , "/sys/class/thermal/thermal_zone8/temp"},
#else
	{"adc", "vts" , "NULL"},
#endif
	{"adc", "usb_in" , "/sys/class/power_supply/usb/voltage_now"},
	/* Battery */
	{"battery", "battery_type", "/sys/class/power_supply/battery_id/manufacturer"},
	{"battery", "capacity", "/sys/class/power_supply/battery/capacity"},
	{"battery", "cycle", "/sys/class/power_supply/battery/battery_cycle"},
	{"battery", "raw_capacity", "/sys/class/power_supply/battery/raw_capacity"},
	{"battery", "health", "/sys/class/power_supply/battery/health"},
	{"battery", "present", "/sys/class/power_supply/battery/present"},
	{"battery", "pseudo_batt", "/sys/class/power_supply/charger_controller/pseudo_batt"},
	{"battery", "temp", "/sys/class/power_supply/battery/temp"},
	{"battery", "valid_batt_id", "/sys/class/power_supply/battery_id/valid_batt_id"},
	{"battery", "check_batt_id_for_aat", "/sys/class/power_supply/battery_id/check_batt_id_for_aat"},
	{"battery", "voltage_now", "/sys/class/power_supply/battery/voltage_now"},
	{"battery", "status", "/sys/class/power_supply/battery/status"},
	/* Charger */
	{"charger", "ac_online", "/sys/class/power_supply/ac/online"},
	{"charger", "usb_online", "/sys/class/power_supply/usb/online"},
	{"charger", "wlc_online", "/sys/class/power_supply/wireless/online"},
	{"charger", "present", "/sys/class/power_supply/ac/present"},
	{"charger", "wlc_online", "NULL"},
	{"charger", "ibat_current", "/sys/class/power_supply/battery/current_now"},
	{"charger", "time_out", "/sys/class/power_supply/battery/safety_timer_enabled"},
	{"charger", "charging_enabled", "/sys/class/power_supply/battery/charging_enabled"},
	{"charger", "usb_parallel_charging_enabled", "/sys/class/power_supply/usb/fastchg"},
	{"charger", "ichg_current", "/sys/class/power_supply/usb/current_max"},
	{"charger", "chgstep", "/sys/class/power_supply/charger_controller/charge_type"},
	{"charger", "call_state", "/sys/class/power_supply/charger_controller/call_charging_mode"},
	{"charger", "restricted_charging", "/sys/module/charger_controller/parameters/restricted_charging"},
	{"charger", "store_demo_enabled", "/sys/class/power_supply/charger_controller/store_demo_enabled"},
	{"charger", "fastchg", "/sys/class/power_supply/usb/fastchg"},
	{"charger", "fastchg_support", "/sys/class/power_supply/usb/fastchg_support"},
	{"charger", "incompatible", "/sys/class/power_supply/usb/incompatible_chg"},
	#if defined(CONFIG_LGE_USB_TYPE_C) && defined(CONFIG_LGE_USB_MOISTURE_DETECTION)
	{"charger", "moisture_detection", "/sys/class/power_supply/battery/input_suspend"},
	#endif
	/* LCD */
	{"lcd", "brightness", "/sys/class/leds/lcd-backlight/brightness"},
	{"lcd", "max_brightness", "/sys/class/leds/lcd-backlight/max_brightness"},
	/* Key LED */
	{"key_led", "red_brightness", "/sys/class/leds/red/brightness"},
	{"key_led", "setting", "/sys/class/leds/red/pattern_id"},
	{"key_led", "green_brightness", "NULL"},
	{"key_led", "blue_brightness", "NULL"},
	{"key_led", "button_backlight1_brightness", "NULL"},
	/* CPU */
	#ifdef CONFIG_LGE_PM_MTK_GAME_MODE
	{"cpu", "game_mode", "/proc/ppm/policy/forcelimit_fix_game"},
	#endif
	#ifdef CONFIG_LGE_PM_MTK_PRM
	{"cpu", "prmlimit_maxfreq", "/proc/ppm/policy/prmlimit_max_cpu_freq"},
	{"cpu", "prmlimt_gpu", "/proc/gpufreq/gpufreq_opp_max_freq"},
	#endif
	/* GPU */
	{"gpu", "gpu_converted_working_time", "/proc/mali/gpu_converted_working_time"},
	{"gpu", "max_gpuclk", "/proc/gpufreq/gpufreq_max_freq"},
	{"gpu", "gpuclk", "/proc/gpufreq/gpufreq_cur_freq"},
	{"gpu", "gpubusy", "/proc/mali/utilization"},
	{"gpu", "gpu_pwr_state", "/proc/mali/utilization"},
	{"gpu", "gpu_loading", "/sys/module/ged/parameters/gpu_loading"},
	/* Platform */
	{"platform", "pcb_rev", "NULL"},	/* should be defined */
	{"platform", "smpl_boot", "/sys/bootinfo/powerup_by_smpl"},
	/* Perf Service feature */
	{"psfeature", "vfps", "/sys/class/graphics/fb0/vfps"},
	{"psfeature", "vfps_fcnt", "/sys/class/graphics/fb0/vfps_fcnt"},
	{"psfeature", "vfps_ratio", "/sys/class/graphics/fb0/vfps_ratio"},
	/* TestMode */
	{"testmode", "voltage_now", "/sys/class/power_supply/battery/voltage_now"},
	{"testmode", "usb_id", "/sys/devices/platform/battery/LGBM_AtCmdUsbidadc"},
	{"testmode", "temp", "/sys/devices/platform/battery/LGBM_AtCmdBatmp"},
	{"testmode", "fuelrst", "/sys/class/power_supply/battery/present"},
	{"testmode", "fuelval", "/sys/class/power_supply/battery/capacity"},
	{"testmode", "batt_id", "/sys/class/power_supply/battery_id/batt_id"},
	{"testmode", "chcomp", "/sys/devices/platform/battery/LGBM_AtCmdChcomp"},
	{"testmode", "charge", "/sys/devices/platform/battery/LGBM_AtCmdCharge"},
	{"testmode", "chgmodeoff", "/sys/devices/platform/battery/LGBM_AtCmdChargingModeOff"},
	{"testmode", "battexist", "/sys/class/power_supply/battery/present"},
	{"testmode", "valid_batt_id", "/sys/class/power_supply/battery_id/check_batt_id_for_aat"},
	{"testmode", "usbin", "/sys/class/power_supply/usb/voltage_now"}, // not used Minios4
	{"testmode", "current_now", "/sys/class/power_supply/battery/current_now"},
	{"testmode", "current_max", "/sys/class/power_supply/usb/current_max"},
};

static const struct power_sysfs_array mandatory_paths[] = {
	/* Battery */
	{"battery", "voltage_now"},
	/* TestMode */
	{"testmode", "charge"},
	{"testmode", "chcomp"},
};

static const char *group_names[] = {
	"adc",
	"battery",
	"charger",
	"lcd",
	"key_led",
	"cpu",
	"gpu",
	"platform",
	"psfeature",
	"testmode"
};

static struct power_sysfs_array *arr;
static int arr_cnt;

static int check_mandatory_path(int arr_num)
{
	int mandatory_num;
	int array_size = ARRAY_SIZE(mandatory_paths);

	for (mandatory_num = 0; mandatory_num < array_size; mandatory_num++) {
		if (!strcmp(arr[arr_num].group, mandatory_paths[mandatory_num].group))
			if (!strcmp(arr[arr_num].user_node, mandatory_paths[mandatory_num].user_node))
				return 1;
	}
	return 0;
}

#ifdef CONFIG_OF
static int power_sysfs_parse_dt(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int arr_num, ret, i;

	arr_cnt = of_property_count_strings(node, "sysfs,node") / 3;
	if (arr_cnt > 0)
		pr_info("%s : Total sysfs node is %d\n", __func__, arr_cnt);
	else {
		pr_err("%s : ERROR sysfs node isn't exist\n", __func__);
		return 0;
	}

	arr = devm_kzalloc(&pdev->dev, arr_cnt *
			sizeof(struct power_sysfs_array), GFP_KERNEL);
	if (!arr) {
		pr_err("%s : ERROR get sysfs array\n", __func__);
		return -1;
	}

	for (arr_num = 0, i = 0; arr_num < arr_cnt; arr_num++) {
		ret = of_property_read_string_index(node, "sysfs,node",
				i++, &arr[arr_num].group);
		if (ret) {
			pr_err("%s : ERROR get %ith group\n", __func__, arr_num);
			goto err_get_array;
		}
		ret = of_property_read_string_index(node, "sysfs,node",
				i++, &arr[arr_num].user_node);
		if (ret) {
			pr_err("%s : ERROR get %ith user_node\n", __func__, arr_num);
			goto err_get_array;
		}
		ret = of_property_read_string_index(node, "sysfs,node",
				i++, &arr[arr_num].kernel_node);
		if (ret) {
			pr_err("%s : ERROR get %ith kernel_node\n", __func__, arr_num);
			goto err_get_array;
		} else if (check_mandatory_path(arr_num)) {
			if (!strcmp(arr[arr_num].kernel_node, "NULL")) {
				pr_err("%s : ERROR get mandatory path %s\n", __func__,
						arr[arr_num].user_node);
				goto err_get_array;
			}
			pr_info("%s : %s path is mandatory \n", __func__,
					arr[arr_num].user_node);
		}
	}

	/* debug */
	for (arr_num = 0; arr_num < arr_cnt; arr_num++)
		pr_err("%s : get %dth node is %s, %s, %s\n", __func__, arr_num,
				arr[arr_num].group, arr[arr_num].user_node,
				arr[arr_num].kernel_node);

	return 0;

err_get_array:
	return -1;
}
#else
static int power_sysfs_parse_dt(struct platform_device *pdev)
{
	pr_err("%s : ERROR CONFIG_OF isn't set\n", __func__);
	return -1;
}
#endif
static int power_sysfs_make_path(struct platform_device *pdev)
{
	int arr_num, group_num;
	struct proc_dir_entry *p, *temp_p;
	struct proc_dir_entry **groups_p;

	groups_p = devm_kzalloc(&pdev->dev, ARRAY_SIZE(group_names) *
			sizeof(struct proc_dir_entry *), GFP_KERNEL);
	if (!groups_p) {
		pr_err("%s : ERROR make groups pointer \n", __func__);
		return -ENOMEM;
	}

	/* Set Power Sysfs root directory */
	p = proc_mkdir("lge_power", NULL);
	if (p == NULL) {
		pr_err("%s : ERROR make root sysfs \n", __func__);
		return -ENOMEM;
	}

	/* Set Power Sysfs group directory */
	for (group_num = 0; group_num < ARRAY_SIZE(group_names); group_num++) {
		groups_p[group_num] = proc_mkdir(group_names[group_num], p);
		if (groups_p[group_num] == NULL) {
			pr_err("%s : ERROR make %s group \n", __func__,
					group_names[group_num]);
			return -ENOMEM;
		}
	}

	/* Set Power Sysfs Path */
	for (arr_num = 0; arr_num < arr_cnt; arr_num++) {
		for (group_num = 0; group_num < ARRAY_SIZE(group_names); group_num++)
			if (!strcmp(arr[arr_num].group, group_names[group_num]))
				break;

		if (!strcmp(arr[arr_num].kernel_node, "NULL")) {
			pr_info("%s : %s user node didn't have kernel node \n",
					__func__, arr[arr_num].user_node);
			continue;
		} else {
			temp_p = proc_symlink(arr[arr_num].user_node, groups_p[group_num],
					arr[arr_num].kernel_node);
			if (temp_p == NULL) {
				pr_err("%s : ERROR make %ith sysfs path(%s, %s, %s)\n",
						__func__, arr_num, arr[arr_num].group,
						arr[arr_num].kernel_node, arr[arr_num].user_node);
				return -ENOMEM;
			}
		}

	}

	return 0;
}

static int read_sysfs_path(struct platform_device *pdev)
{
	int arr_num;

	arr_cnt = ARRAY_SIZE(lge_power_sysfs_path);

	arr = devm_kzalloc(&pdev->dev, arr_cnt *
			sizeof(struct power_sysfs_array), GFP_KERNEL);
	if (!arr) {
		pr_err("%s : ERROR get sysfs array\n", __func__);
		return -1;
	}

	for (arr_num = 0; arr_num < arr_cnt; arr_num++) {
		arr[arr_num].group = lge_power_sysfs_path[arr_num].group;
		arr[arr_num].user_node = lge_power_sysfs_path[arr_num].user_node;
		arr[arr_num].kernel_node = lge_power_sysfs_path[arr_num].kernel_node;
		if (arr[arr_num].kernel_node == NULL) {
			pr_err("%s : ERROR get %ith kernel_node\n", __func__, arr_num);
			goto err_get_array;
		} else if (check_mandatory_path(arr_num)) {
			if (!strcmp(arr[arr_num].kernel_node, "NULL")) {
				pr_err("%s : ERROR get mandatory path %s\n", __func__,
						arr[arr_num].user_node);
				goto err_get_array;
			}
			pr_info("%s : %s path is mandatory \n", __func__,
					arr[arr_num].user_node);
		}
	}

	/* debug */
	for (arr_num = 0; arr_num < arr_cnt; arr_num++)
		pr_err("%s : get %dth node is %s, %s, %s\n", __func__, arr_num,
				arr[arr_num].group, arr[arr_num].user_node,
				arr[arr_num].kernel_node);
	return 0;

err_get_array:
	return -1;
}

static int power_sysfs_probe(struct platform_device *pdev)
{
	int ret;

	if (pdev->dev.of_node) {
		ret = power_sysfs_parse_dt(pdev);
		if (ret < 0) {
			pr_err("%s : ERROR Parsing DT\n", __func__);
			return ret;
		}
	} else {
		ret = read_sysfs_path(pdev);
		if (ret < 0) {
			pr_err("%s : ERROR Parsing data\n", __func__);
			return ret;
		}
	}

	ret = power_sysfs_make_path(pdev);
	if (ret != 0) {
		pr_err("%s : ERROR make sysfs path\n", __func__);
		return ret;
	}

	pr_info("%s : Success Power sysfs Init\n", __func__);

	return ret;
}

static int power_sysfs_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id power_sysfs_match_table[] = {
	{ .compatible = "lge,power-sysfs" },
	{}
};
#endif

static struct platform_driver power_sysfs_driver = {
	.probe = power_sysfs_probe,
	.remove = power_sysfs_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = power_sysfs_match_table,
#endif
	},
};

static int __init power_sysfs_init(void)
{
	return platform_driver_register(&power_sysfs_driver);
}

static void power_sysfs_exit(void)
{
	platform_driver_unregister(&power_sysfs_driver);
}

#ifdef CONFIG_OF
static struct platform_device power_sysfs_platform_device = {
	.name   = "power-sysfs",
	.id = 0,
};

static int __init power_sysfs_device_init(void)
{
	pr_err("%s st\n", __func__);
	return platform_device_register(&power_sysfs_platform_device);
}

static void power_sysfs_device_exit(void)
{
	platform_device_unregister(&power_sysfs_platform_device);
}
#endif

late_initcall(power_sysfs_init);
module_exit(power_sysfs_exit);
#ifdef CONFIG_OF
late_initcall(power_sysfs_device_init);
module_exit(power_sysfs_device_exit);
#endif
MODULE_DESCRIPTION("LGE Power sysfs driver");
MODULE_LICENSE("GPL v2");
