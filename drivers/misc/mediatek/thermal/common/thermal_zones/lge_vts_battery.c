/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "[Power/VTS_Battery_Thermal] %s: " fmt, __func__

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/seq_file.h>
#include <linux/uidgid.h>
#include <linux/slab.h>
#include "mt-plat/mtk_thermal_monitor.h"
#include "mach/mt_thermal.h"
#include <soc/mediatek/lge/board_lge.h>
#include <linux/lge_vts.h>

//#define DEBUG_LOG

#define VTS_NAME "battery_vts"
#define VTS_BATTERY_TEMP_CRIT (800)	/* 80.0 degree Celsius */
#define VTS_BATTERY_TEMP_DEFAULT (200)	/* 20.0 degree Celsius */
#define VTS_TEMP_SCALING (100)
#define VTS_WEIGHT_SCALING (1000)

struct vts_param {
	char tz[THERMAL_NAME_LENGTH];
	int scaling;	/* multplier to convert to milli degree Celsius */
	int weight;
};

struct vts_trip {
	char cooler[THERMAL_NAME_LENGTH];
	int temp;
	int type;
};

static struct thermal_zone_device *thz_dev;
static DEFINE_SEMAPHORE(sem_mutex);
static int thermal_mode;
static struct delayed_work update;
static int vts_battery_temp = 250;

#define VTS_BATTERY_PARAMS 3
static struct vts_param params[VTS_BATTERY_PARAMS] = {
	{
		.tz = "mtktsbattery",
		.scaling = 1,
		.weight = VTS_WEIGHT_SCALING,
	},
	{
		.tz = "no-tz",
		.scaling = 0,
		.weight = 0,
	},
	{
		.tz = "no-tz",
		.scaling = 0,
		.weight = 0,
	},
};
static int vts_constant = 0;	/* milli degree Celsius */
static int cmode_constant = 0;

#define VTS_BATTERY_TRIPS 10
static struct vts_trip trips[VTS_BATTERY_TRIPS] = {
	{"no-cooler", 1200, THERMAL_TRIP_ACTIVE},
	{"no-cooler", 1100, THERMAL_TRIP_ACTIVE},
	{"no-cooler", 1000, THERMAL_TRIP_ACTIVE},
	{"no-cooler",  900, THERMAL_TRIP_ACTIVE},
	{"no-cooler",  800, THERMAL_TRIP_ACTIVE},
	{"no-cooler",  700, THERMAL_TRIP_ACTIVE},
	{"no-cooler",  600, THERMAL_TRIP_ACTIVE},
	{"no-cooler",  500, THERMAL_TRIP_ACTIVE},
	{"no-cooler",  400, THERMAL_TRIP_ACTIVE},
	{"no-cooler",  300, THERMAL_TRIP_ACTIVE},
};
static int num_trip = 0;
static unsigned int interval = 5000;	/* mseconds, 0 : no auto polling */

int lge_vts_get_battery_temp(void)
{
	return vts_battery_temp;
}
EXPORT_SYMBOL(lge_vts_get_battery_temp);

static int vts_battery_calculate_temp(void)
{
	struct thermal_zone_device *dev;
	unsigned long tz_temp;
	int temp = 0;
	int temp_old = 0;
	int rc;
	int i;

#ifdef DEBUG_LOG
	int debug_scaling[VTS_BATTERY_PARAMS];
	int debug_weight[VTS_BATTERY_PARAMS];
	int debug_temp[VTS_BATTERY_PARAMS];
	int debug_accumulate_temp[VTS_BATTERY_PARAMS];
#endif

	for (i = 0; i < VTS_BATTERY_PARAMS; i++) {
		if (!params[i].weight || !params[i].scaling)
			continue;

		dev = thermal_zone_get_zone_by_name(params[i].tz);
		rc = thermal_zone_get_temp(dev, &tz_temp);
		if (rc)
			continue;

#ifdef DEBUG_LOG
		debug_temp[i] = tz_temp;
#endif

		/* Convert to milli degree Celsius */
		tz_temp *= params[i].scaling;

		/* accumulate temp with weight */
		temp += tz_temp * params[i].weight;

#ifdef DEBUG_LOG
		debug_scaling[i] = params[i].scaling;
		debug_weight[i] = params[i].weight;
		debug_accumulate_temp[i] = temp;
#endif
	}

#ifdef DEBUG_LOG
	for (i = 0; i < VTS_BATTERY_PARAMS; i++ ) {
		pr_info("[DEBUG] accumulate_temp(%d) : temp(%d)*scaling(%d)*weight(%d)\n",
			debug_accumulate_temp[i]/1000,
			debug_temp[i],
			debug_scaling[i],
			debug_weight[i]
			);
	}
#endif

	/* Convert to milli degree Celsius */
	temp /= VTS_WEIGHT_SCALING;

	temp_old = temp;

	/* Add constant to adjust temp */
	temp += vts_constant;

#ifdef DEBUG_LOG
	pr_info("[DEBUG] temp(%d)=temp(%d)+vts_constant(%d)\n",
				temp, temp_old, vts_constant);
#endif

	/* Compensation for LCD Off fast charging */
	if (0) {
		temp_old = temp;
		temp += cmode_constant;
		pr_info("[DEBUG] : temp(%d) = temp(%d) + c_mode(%d)\n",
				temp, temp_old, cmode_constant);
	}

	/* Convert milli degree Celsius to tenths of degree Celsius */
	return temp / VTS_TEMP_SCALING;
}

static int vts_battery_get_temp(struct thermal_zone_device *thermal,
				unsigned long *t)
{
	*t = vts_battery_temp;

	return 0;
}

static int vts_battery_bind(struct thermal_zone_device *thermal,
			    struct thermal_cooling_device *cdev)
{
	int i;

	for (i = 0; i < VTS_BATTERY_TRIPS; i++) {
		if (!strcmp(cdev->type, trips[i].cooler))
			break;
	}

	if (i >= VTS_BATTERY_TRIPS)
		return 0;

	if (mtk_thermal_zone_bind_cooling_device(thermal, i, cdev))
		return -EINVAL;

	return 0;
}

static int vts_battery_unbind(struct thermal_zone_device *thermal,
			      struct thermal_cooling_device *cdev)
{
	int i;

	for (i = 0; i < VTS_BATTERY_TRIPS; i++) {
		if (!strcmp(cdev->type, trips[i].cooler))
			break;
	}

	if (i >= VTS_BATTERY_TRIPS)
		return 0;

	if (thermal_zone_unbind_cooling_device(thermal, i, cdev))
		return -EINVAL;

	return 0;
}

static int vts_battery_get_mode(struct thermal_zone_device *thermal,
				enum thermal_device_mode *mode)
{
	*mode = (thermal_mode) ?
		THERMAL_DEVICE_ENABLED : THERMAL_DEVICE_DISABLED;

	return 0;
}

static int vts_battery_set_mode(struct thermal_zone_device *thermal,
				enum thermal_device_mode mode)
{
	thermal_mode = mode;

	return 0;
}

static int vts_battery_get_trip_type(struct thermal_zone_device *thermal,
				     int trip, enum thermal_trip_type *type)
{
	*type = trips[trip].type;

	return 0;
}

static int vts_battery_get_trip_temp(struct thermal_zone_device *thermal,
				     int trip, unsigned long *temp)
{
	*temp = trips[trip].temp;

	return 0;
}

static int vts_battery_get_crit_temp(struct thermal_zone_device *thermal,
				     unsigned long *temperature)
{
	*temperature = VTS_BATTERY_TEMP_CRIT;

	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops vts_battery_dev_ops = {
	.bind = vts_battery_bind,
	.unbind = vts_battery_unbind,
	.get_temp = vts_battery_get_temp,
	.get_mode = vts_battery_get_mode,
	.set_mode = vts_battery_set_mode,
	.get_trip_type = vts_battery_get_trip_type,
	.get_trip_temp = vts_battery_get_trip_temp,
	.get_crit_temp = vts_battery_get_crit_temp,
};

static void vts_battery_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);

	if (!lge_vts_get_enabled())
		return;

	vts_battery_temp = vts_battery_calculate_temp();

	if (interval)
		schedule_delayed_work(dwork, msecs_to_jiffies(interval));
}

static void vts_battery_enable(bool en)
{
	if (!en) {
		cancel_delayed_work_sync(&update);
		return;
	}

	schedule_delayed_work(&update, 0);
}

static int vts_battery_register_thermal(void)
{
	if (thz_dev)
		return 0;

	thz_dev = mtk_thermal_zone_device_register(VTS_NAME, num_trip,
			NULL, &vts_battery_dev_ops, 0, 0, 0, interval);

	return 0;
}

static void vts_battery_unregister_thermal(void)
{
	if (!thz_dev)
		return;

	cancel_delayed_work(&update);

	mtk_thermal_zone_device_unregister(thz_dev);
	thz_dev = NULL;
}

static int vts_battery_read(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < VTS_BATTERY_TRIPS; i++) {
		seq_printf(m, "[%d] cooler: %s temp: %d type: %d\n", i,
				trips[i].cooler,
				trips[i].temp, trips[i].type);
	}
	seq_printf(m, "time: %d\n", interval);

	return 0;
}

static ssize_t vts_battery_write(struct file *file, const char __user *buffer,
				 size_t count, loff_t *data)
{
	struct vts_trip trip[VTS_BATTERY_TRIPS];
	int polling_time;
	char *buf;
	int rc;
	int i;

	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return 0;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return 0;
	}
	buf[count] = '\0';

	rc = sscanf(buf, "%d"
			" %d %d %19s %d %d %19s %d %d %19s %d %d %19s"
			" %d %d %19s %d %d %19s %d %d %19s %d %d %19s"
			" %d %d %19s %d %d %19s"
			" %d",
			&num_trip,
			&trip[0].temp, &trip[0].type, trip[0].cooler,
			&trip[1].temp, &trip[1].type, trip[1].cooler,
			&trip[2].temp, &trip[2].type, trip[2].cooler,
			&trip[3].temp, &trip[3].type, trip[3].cooler,
			&trip[4].temp, &trip[4].type, trip[4].cooler,
			&trip[5].temp, &trip[5].type, trip[5].cooler,
			&trip[6].temp, &trip[6].type, trip[6].cooler,
			&trip[7].temp, &trip[7].type, trip[7].cooler,
			&trip[8].temp, &trip[8].type, trip[8].cooler,
			&trip[9].temp, &trip[9].type, trip[9].cooler,
			&polling_time);
	kfree(buf);

	if (!rc)
		return -EINVAL;

	down(&sem_mutex);
	vts_battery_unregister_thermal();

	if (num_trip < 0 || num_trip > 10) {
		up(&sem_mutex);
		return -EINVAL;
	}

	for (i = 0; i < VTS_BATTERY_TRIPS; i++) {
		trips[i].temp = trip[i].temp;
		trips[i].type = trip[i].type;
		strncpy(trips[i].cooler, trip[i].cooler, THERMAL_NAME_LENGTH);
		trips[i].cooler[THERMAL_NAME_LENGTH - 1] = '\0';
	}

	interval = polling_time;

	vts_battery_register_thermal();
	up(&sem_mutex);

	return count;
}

static int vts_battery_open(struct inode *inode, struct file *file)
{
	return single_open(file, vts_battery_read, NULL);
}

static const struct file_operations vts_battery_fops = {
	.owner = THIS_MODULE,
	.open = vts_battery_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = vts_battery_write,
	.release = single_release,
};

static int vts_battery_param_read(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < VTS_BATTERY_PARAMS; i++) {
		seq_printf(m, "[%d] tz: %s weight: %d scale: %d\n", i,
				params[i].tz,
				params[i].weight, params[i].scaling);
	}

	seq_printf(m, "vts_constant: %d\n", vts_constant);
	seq_printf(m, "c_mode: %d\n", cmode_constant);

	return 0;
}

static ssize_t vts_battery_param_write(struct file *file, const char __user *buffer,
				       size_t count, loff_t *data)
{
	struct vts_param param[VTS_BATTERY_PARAMS];
	int constant;
	int cmode;
	char *buf;
	int rc;
	int i;

	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return 0;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return 0;
	}
	buf[count] = '\0';

	rc = sscanf(buf, "%d %d %19s %d %d %19s %d %d %19s %d %d",
			&constant, &cmode,
			param[0].tz, &param[0].scaling, &param[0].weight,
			param[1].tz, &param[1].scaling, &param[1].weight,
			param[2].tz, &param[2].scaling, &param[2].weight);
	kfree(buf);

	if (!rc)
		return -EINVAL;

#if defined (CONFIG_MACH_MT6750S_CV5A)
	if (HW_REV_B > lge_get_board_revno())
		return count;
#endif
#if defined (CONFIG_MACH_MT6750S_CV7A)
	if (HW_REV_A > lge_get_board_revno())
		return count;
#endif
	vts_constant = constant;
	cmode_constant = cmode;
	for (i = 0; i < VTS_BATTERY_PARAMS; i++) {
		strncpy(params[i].tz, param[i].tz, THERMAL_NAME_LENGTH);
		params[i].tz[THERMAL_NAME_LENGTH - 1] = '\0';
		/* battery -> mtktsbattery */
		if (!strcmp(params[i].tz, "battery"))
			strcpy(params[i].tz, "mtktsbattery");
		params[i].weight = param[i].weight;
		params[i].scaling = param[i].scaling;
	}

	return count;
}

static int vts_battery_param_open(struct inode *inode, struct file *file)
{
	return single_open(file, vts_battery_param_read, NULL);
}

static const struct file_operations vts_battery_param_fops = {
	.owner = THIS_MODULE,
	.open = vts_battery_param_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = vts_battery_param_write,
	.release = single_release,
};

static kuid_t uid = KUIDT_INIT(0);
static kgid_t gid = KGIDT_INIT(1000);
static int __init vts_battery_init(void)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *vts_battery_dir = NULL;

	INIT_DELAYED_WORK(&update, vts_battery_update_work);

	vts_battery_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!vts_battery_dir)
		return 0;

	entry = proc_create("tzbattery_vts", S_IRUGO | S_IWUSR | S_IWGRP,
			vts_battery_dir, &vts_battery_fops);
	if (entry)
		proc_set_user(entry, uid, gid);

	entry = proc_create("tzbattery_vts_param", S_IRUGO | S_IWUSR | S_IWGRP,
			vts_battery_dir, &vts_battery_param_fops);
	if (entry)
		proc_set_user(entry, uid, gid);

	lge_vts_register("vts_battery", vts_battery_enable);

	return 0;
}

static void __exit vts_battery_exit(void)
{
	vts_battery_unregister_thermal();
}
module_init(vts_battery_init);
module_exit(vts_battery_exit);
