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

#define pr_fmt(fmt) "[Power/VTS] %s: " fmt, __func__

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
#include <linux/lge_vts.h>

struct lge_vts_module {
	char *name;
	void (*enable)(bool en);
};

#define LGE_VTS_MAX 5
static struct lge_vts_module vts_modules[LGE_VTS_MAX] = {
	{
		.name = NULL,
		.enable = NULL,
	},
	{
		.name = NULL,
		.enable = NULL,
	},
	{
		.name = NULL,
		.enable = NULL,
	},
	{
		.name = NULL,
		.enable = NULL,
	},
	{
		.name = NULL,
		.enable = NULL,
	},
};

static bool vts_enabled = false;

bool lge_vts_get_enabled(void)
{
	return vts_enabled;
}
EXPORT_SYMBOL(lge_vts_get_enabled);

void lge_vts_register(char *name, void (*enable)(bool en))
{
	int i;

	if (!name || !enable)
		return;

	for (i = 0; i < LGE_VTS_MAX; i++) {
		if (vts_modules[i].name || vts_modules[i].enable)
			continue;
		vts_modules[i].name = name;
		vts_modules[i].enable = enable;
		break;
	}

	pr_info("%s registered at index %d\n", name, i);
}
EXPORT_SYMBOL(lge_vts_register);

#define THERMAL_LOGGING_DELAY 10000
static char *zone_name[] = {
	"mtktscpu",
	"mtktsAP",
	"tsAP2",
	"mtktsbattery",
	"battery_vts",
	"skin_vts",
};

static struct delayed_work lge_thermal_monitor;

static void thermal_information_enable(bool en)
{
	if (!en) {
		cancel_delayed_work_sync(&lge_thermal_monitor);
		pr_info("VTS Not Ready\n");
		return;
	}

	schedule_delayed_work(&lge_thermal_monitor, msecs_to_jiffies(1000));
}

static void thermal_information(struct work_struct *work)
{
	int ret = 0;
	int i = 0;
	long temp = 0;
	long log_t[6] = { 0 };
	struct thermal_zone_device *tz = NULL;

	if (!lge_vts_get_enabled()) {
		pr_info("VTS Not Ready\n");
		return;
	}

	for (; i < ARRAY_SIZE(zone_name); i++) {
		tz = thermal_zone_get_zone_by_name(zone_name[i]);
		ret = thermal_zone_get_temp(tz, &temp);
		if (ret < 0)
			continue;
		log_t[i] = temp;
	}

#ifdef CONFIG_MACH_MT6750_CV3
	temp = 190000 + log_t[3]*35 + log_t[1]*27 + log_t[2]*34;
	pr_info("CPU( %li ) AP( %li ) AP2( %li ) Raw_Batt( %li ) VTS_B( %li ) VTS_S( %li ) VTS_Camera( %li )\n",
				log_t[0]/100, log_t[1]/100, log_t[2]/100, log_t[3]/100, log_t[4], log_t[5], temp/10000);
#else
	pr_info("CPU( %li ) AP( %li ) AP2( %li ) Raw_Batt( %li ) VTS_B( %li ) VTS_S( %li )\n",
				log_t[0]/100, log_t[1]/100, log_t[2]/100, log_t[3]/100, log_t[4], log_t[5]);
#endif

	schedule_delayed_work(to_delayed_work(work), msecs_to_jiffies(THERMAL_LOGGING_DELAY));
}

static void lge_vts_enable(bool en)
{
	int i;

	for (i = 0; i < LGE_VTS_MAX; i++) {
		if (!vts_modules[i].enable)
			continue;

		vts_modules[i].enable(en);
		pr_info("%s %s\n", vts_modules[i].name,
				en ? "enabled" : "disabled");
	}
}

static int lge_vts_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", vts_enabled ? "Enabled" : "Disabled");

	return 0;
}

static ssize_t lge_vts_write(struct file *file, const char __user *buffer,
			      size_t count, loff_t *data)
{
	char *buf;
	bool en = false;
	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return 0;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return 0;
	}
	buf[count] = '\0';

	if (!strncmp(buf, "1", 1)
			|| !strncmp(buf, "enable", 6)
			|| !strncmp(buf, "Enable", 6))
		en = true;

	kfree(buf);

	if (vts_enabled == en)
		return count;

	vts_enabled = en;

	lge_vts_enable(en);

	return count;
}

static int lge_vts_open(struct inode *inode, struct file *file)
{
	return single_open(file, lge_vts_read, NULL);
}

static const struct file_operations lge_vts_fops = {
	.owner = THIS_MODULE,
	.open = lge_vts_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = lge_vts_write,
	.release = single_release,
};

static kuid_t uid = KUIDT_INIT(0);
static kgid_t gid = KGIDT_INIT(1000);
static int __init lge_vts_init(void)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *lge_vts_dir = NULL;

	lge_vts_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!lge_vts_dir) {
		pr_info("failed to get dir for vts\n");
		return 0;
	}

	entry = proc_create("tz_vts", S_IRUGO | S_IWUSR | S_IWGRP,
			lge_vts_dir, &lge_vts_fops);
	if (entry)
		proc_set_user(entry, uid, gid);

	INIT_DELAYED_WORK(&lge_thermal_monitor, thermal_information);

	lge_vts_register("monitor", thermal_information_enable);

	return 0;
}

static void __exit lge_vts_exit(void)
{
	cancel_delayed_work(&lge_thermal_monitor);
	return;
}
module_init(lge_vts_init);
module_exit(lge_vts_exit);
