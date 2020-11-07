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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/timer.h>

#include "lg_partition.h"


/* ENABLE_TEST should be disabled in release source */
/* it is just for the MTK internal testing of the new platform */
#if 0
#define ENABLE_TEST
#endif

#define TIMER_DELAY 20

static struct timer_list tc1_test_timer;

#ifdef ENABLE_TEST
static void tc1_interface_test_timer(unsigned long data)
{
	del_timer(&tc1_test_timer);
	pr_err("start test kernel TC1 API...\n");
	LGE_API_test();
	pr_err("end test kernel TC1 API...\n");
}
#endif

static int __init test_init(void)
{
#ifdef ENABLE_TEST
	init_timer(&tc1_test_timer);
	tc1_test_timer.function = (void *)&tc1_interface_test_timer;
	tc1_test_timer.expires = jiffies + TIMER_DELAY * HZ;
	add_timer(&tc1_test_timer);
#endif

	return 1;
}

/* should never be called */
static void __exit test_exit(void)
{
	del_timer(&tc1_test_timer);
}

late_initcall(test_init);
module_exit(test_exit);

MODULE_AUTHOR("kai.zhu@mediatek.com");
MODULE_DESCRIPTION("test module");
MODULE_LICENSE("GPL");
