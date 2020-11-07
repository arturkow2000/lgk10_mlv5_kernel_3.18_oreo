/* production_test.c
 *
 * Copyright (C) 2015 LGE.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/time.h>

#include <touch_hwif.h>
#include <touch_core.h>

#include "touch_nt11206.h"

static void write_file(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu \n",
			my_date.tm_mon + 1,my_date.tm_mday,
			my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
			(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	TOUCH_I("write open %s, fd : %d\n", (fd >= 0)? "success":"fail",fd);
	if (fd >= 0) {
		if (time > 0) {
			sys_write(fd, time_string, strlen(time_string));
			TOUCH_I("Time write success.\n");
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);

}
static ssize_t show_sd(struct device *dev, char *buf)
{
	int ret = 0;
	u8 mode = 0;
	ret = nt11206_selftest(dev, buf, mode);
	write_file(SELF_TEST_FILE_PATH, buf, 0);
	return ret;
}
static ssize_t show_lpwgsd(struct device *dev, char *buf)
{
	int ret = 0;
	u8 mode = 1;
	ret = nt11206_selftest(dev, buf, mode);
	write_file(SELF_TEST_FILE_PATH, buf, 0);
	return ret;
}
static ssize_t show_delta(struct device *dev, char *buf)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	u8 x_num=0;
	u8 y_num=0;
	u16 *xdata = NULL;

	xdata = (u16*)kmalloc(2 * 2048, GFP_KERNEL);
	if(xdata == NULL) {
		TOUCH_E("xdata Alloc Fail\n");
		return -1;
	}
	nvt_change_mode(dev, TEST_MODE_2);

	if(nvt_check_fw_status(dev) != 0) {
		TOUCH_E("FW_STATUS FAIL\n");
		if(xdata)
			kfree(xdata);
		return -EAGAIN;
	}
	nvt_get_fw_info(dev);

	if(nvt_get_fw_pipe(dev) == 0)
		nvt_read_mdata(dev, DIFF_PIPE0_ADDR);
	else
		nvt_read_mdata(dev, DIFF_PIPE1_ADDR);

	nvt_change_mode(dev, MODE_CHANGE_NORMAL_MODE);

	if(xdata) {
		memset(xdata, 0, 2048 * 2);
		nvt_get_mdata(xdata, &x_num, &y_num);
	}
	ret = snprintf(buf, PAGE_SIZE, "======== Deltadata ========\n");
	for(i=0; i<y_num; i++)
	{
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i);
		for(j=0; j<x_num; j++)
		{
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "%5d ", (short)xdata[i*x_num+j]);
		}
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");

	write_file(DELTA_FILE_PATH, buf, 0);
	if(xdata)
		kfree(xdata);
	return ret;
}
static ssize_t show_rawdata(struct device *dev, char *buf)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	u16* xdata = NULL;
	u8 x_num=0;
	u8 y_num=0;

	xdata = (u16*)kmalloc(2 * 2048, GFP_KERNEL);
	if(xdata == NULL) {
		TOUCH_E("xdata Alloc Fail\n");
		return -1;
	}
	nvt_change_mode(dev, TEST_MODE_1);

	if(nvt_check_fw_status(dev) != 0) {
		TOUCH_E("FW_STATUS FAIL\n");
		if(xdata)
			kfree(xdata);
		return -EAGAIN;
	}
	nvt_get_fw_info(dev);

	if(nvt_get_fw_pipe(dev) == 0)
		nvt_read_mdata(dev, RAW_PIPE0_ADDR);
	else
		nvt_read_mdata(dev, RAW_PIPE1_ADDR);

	nvt_change_mode(dev, MODE_CHANGE_NORMAL_MODE);
	if(xdata) {
		memset(xdata, 0, 2048 * 2);
		nvt_get_mdata(xdata, &x_num, &y_num);
	}

	for(i=0; i<y_num; i++)
	{
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i);
		for(j=0; j<x_num; j++)
		{
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "%5d", (short)xdata[i*x_num+j]);
		}
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");

	write_file(RAWDATA_FILE_PATH, buf, 0);
	if(xdata)
		kfree(xdata);
	return ret;
}
static ssize_t show_baseline(struct device *dev, char *buf)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	u16* xdata = NULL;
	u8 x_num=0;
	u8 y_num=0;

	xdata = (u16*)kmalloc(2 * 2048, GFP_KERNEL);
	if(xdata == NULL) {
		TOUCH_E("xdata Alloc Fail\n");
		return -1;
	}
	nvt_change_mode(dev, TEST_MODE_1);

	if(nvt_check_fw_status(dev) != 0) {
		if(xdata)
			kfree(xdata);
		TOUCH_E("FW_STATUS FAIL\n");
		return -EAGAIN;
	}
	nvt_get_fw_info(dev);

	nvt_read_mdata(dev, BASELINE_ADDR);

	nvt_change_mode(dev, MODE_CHANGE_NORMAL_MODE);
	if(xdata) {
		memset(xdata, 0, 2048 * 2);
		nvt_get_mdata(xdata, &x_num, &y_num);
	}
	for(i=0; i<y_num; i++)
	{
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i);
		for(j=0; j<x_num; j++)
		{
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "%5d", (short)xdata[i*x_num+j]);
		}
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	write_file(BASELINE_FILE_PATH, buf, 0);
	if(xdata)
		kfree(xdata);
	return ret;
}
static ssize_t show_pinstate(struct device *dev, char *buf)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	ret = snprintf(buf, PAGE_SIZE,
			"RST:%d, INT:%d, SCL:%d, SDA:%d, VDD:%d, VIO:%d\n",
			gpio_get_value(ts->reset_pin), gpio_get_value(ts->int_pin),
			gpio_get_value(890 + 11), gpio_get_value(890 + 10),
			gpio_get_value(ts->vdd_pin), gpio_get_value(ts->vio_pin));
	TOUCH_I("%s() buf:%s",__func__, buf);
	return ret;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwgsd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(baseline, show_baseline, NULL);
static TOUCH_ATTR(pinstate, show_pinstate, NULL);

static struct attribute *nt11206_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_pinstate.attr,
	&touch_attr_baseline.attr,
	NULL,
};

static const struct attribute_group nt11206_attribute_group = {
	.attrs = nt11206_attribute_list,
};

int nt11206_touch_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &nt11206_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return ret;
}
