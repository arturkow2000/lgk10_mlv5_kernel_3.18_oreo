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
#include <linux/firmware.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_lr388k6.h"
#include "touch_lr388k6_prd.h"


static char w_buf[BUF_SIZE];

static void log_file_size_check(struct device *dev)
{
	TOUCH_TRACE();

	return;
}

void write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();
	int mfts_mode = 0;

	set_fs(KERNEL_DS);

	mfts_mode = touch_boot_mode_check(dev);

	switch (mfts_mode) {
	case MFTS_NONE:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MFTS_FOLDER:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mfts_mode\n", __func__);
		break;
	}

	if (fname) {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);

	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		if (write_time == TIME_INFO_WRITE) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec,
					sys_tz.tz_minuteswest * 60 * (-1),
					&my_date);
			snprintf(time_string, 64,
				"\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
				my_date.tm_mon + 1,
				my_date.tm_mday, my_date.tm_hour,
				my_date.tm_min, my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	} else {
		TOUCH_I("File open failed\n");
	}
	set_fs(old_fs);
}

static int sdcard_spec_file_read(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int spec_file_read(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int sharp_get_limit(struct device *dev, char *breakpoint,
			unsigned char Tx, unsigned char Rx)
{
	TOUCH_TRACE();
	sdcard_spec_file_read(dev);
	spec_file_read(dev);

	return 0;
}

static void firmware_version_log(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	int ret = 0;
	unsigned char buffer[LOG_BUF_SIZE] = {0,};
	int mfts_mode = 0;

	TOUCH_I("%s\n", __func__);

	mfts_mode = touch_boot_mode_check(dev);
	if (mfts_mode)
		ret = sharp_ic_info(dev);

	ret = snprintf(buffer, LOG_BUF_SIZE,
			"======== Firmware Info ========\n");

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"product_id : %d\n", d->fw[0].moduleMakerID);

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"version : v%d.%02d\n",
			d->fw[0].isOfficial, d->fw[0].version);

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"===============================\n\n");

	write_file(dev, buffer, TIME_INFO_SKIP);
}

static int sharp_cmd_calibration(struct device *dev)
{
	u8 value = 0;
	int count = 0;

	TOUCH_I("%s\n", __func__);

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_COMMAND);

	/* Set Command */
	sharp_write(dev, SHTSC_ADDR_COMMAND, CMD_EXECCALIBRATION, CMD_EXECCALIBRATION_LEN);

	/* Set Indicator */
	sharp_set_indicator(dev, SHTSC_IND_CMD);

	sharp_read(dev, SHTSC_ADDR_INT0, &value, sizeof(value));
	TOUCH_I("Value check value %d, SHTSC_STATUS_COMMAND_RESULT & value %d\n",
			value, (SHTSC_STATUS_COMMAND_RESULT & value));

	/* waiting the result of get_property */
	while (!(SHTSC_STATUS_COMMAND_RESULT & value)) {
		touch_msleep(5);
		count++;

		if (count > 100)
			break;
	}

	sharp_clear_interrupt(dev, SHTSC_STATUS_COMMAND_RESULT);
	TOUCH_I("Calibration Done!!!\n");

	return 0;
}

static ssize_t show_calibration(struct device *dev, char *buf)
{
	struct timeval start;
	struct timeval finish;

	int ret = 0;
	long elapsed_time = 0;

	TOUCH_I("%s\n", __func__);

	do_gettimeofday(&start);
	sharp_cmd_calibration(dev);
	do_gettimeofday(&finish);

	if(finish.tv_sec == start.tv_sec) {
		elapsed_time = finish.tv_usec - start.tv_usec;
	} else if ( finish.tv_sec > start.tv_sec) {
		elapsed_time = (finish.tv_usec + 1000000) - start.tv_usec;
	} else {
		TOUCH_E("Time wrong. check!\n");
	}

	ret = snprintf(buf + ret, PAGE_SIZE - ret,
				"Calbiration done(Time : %ld)\n", elapsed_time);
	TOUCH_I("Calbiration done(Time : %ld)\n", elapsed_time);

	return ret;
}

static int sharp_cmd_sd(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	u8 value = 0;
	int count = 0;

	TOUCH_I("%s\n", __func__);

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_COMMAND);

	/* Set Command */
	sharp_write(dev, SHTSC_ADDR_COMMAND, CMD_SELFDIAG, CMD_SELFDIAG_LEN);

	/* Set Indicator */
	sharp_set_indicator(dev, SHTSC_IND_CMD);
	touch_msleep(100);

	sharp_read(dev, SHTSC_ADDR_INT0, &value, sizeof(value));
	TOUCH_I("Value check value %d, SHTSC_STATUS_COMMAND_RESULT & value %d \n",
			value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (!(SHTSC_STATUS_COMMAND_RESULT & value)) {
		touch_msleep(5);
		count++;

		if (count > 100)
			break;
	}

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_COMMAND_RESULT);

	/* Read Result */
	sharp_read(dev, 0x08, d->cmd_buf, MAX_COMMAND_RESULT_LEN);

	/* Clear Interrupt */
	sharp_clear_interrupt(dev, SHTSC_STATUS_COMMAND_RESULT);

	return 0;
}

static int sharp_read_sd_result(struct device *dev)
{
	unsigned char ram_addr[3];
	unsigned int vram_addr = 0x0177E8;	/* the first address of th map */
	unsigned int read_size = NUM_DRIVE * NUM_SENSE * 2;
	int bytes = read_size;
	int size;
	int index = 0;
	int ret = 0;
	int i = 0, j = 0;

	u8 *buffer = NULL;
	int row_size = NUM_DRIVE;
	int col_size = NUM_SENSE;

	TOUCH_I("%s\n", __func__);

	buffer = kzalloc(sizeof(u8) * 1536, GFP_KERNEL);
	if (buffer == NULL) {
		TOUCH_E("memory allocate fail.\n");
		return -EIO;
	}

	memset(buffer, 0x0, sizeof(u8) * 1536);

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_DCMAP);

	/* read 120 bytes from Bank5, address 8 */
	/* read loop required */
	while (bytes > 0) {
		/* 16bit signed value for each point = read two bytes for one point */
		ram_addr[0] = vram_addr & 0xff;			/* set address to read (Lower) */
		ram_addr[1] = (vram_addr & 0xff00) >> 8;	/* set address to read (Middle) */
		ram_addr[2] = (vram_addr & 0xff0000) >> 16;	/* set address to read (Higher) */

		sharp_write(dev, 0x06, ram_addr, 2);
		/* Set Bank */
		sharp_set_bank(dev, SHTSC_BANK_SYSTEM_HA);
		sharp_write(dev, 0x43, &(ram_addr[2]), sizeof(ram_addr[2]));
		/* Set Bank */
		sharp_set_bank(dev, SHTSC_BANK_DCMAP);

		size = (bytes >= 120) ? 120 : bytes;
		TOUCH_I("size : %d\n", size);

		/* 120 bytes = 120/2=60 elements (signed 16bit little endian) */
		sharp_read(dev, 0x08, &(buffer[index]), size);
		index += size;
		bytes -= size;
		vram_addr += size;
		TOUCH_I("size : %04X\n", vram_addr);
	}

	ret += snprintf(w_buf + ret, sizeof(w_buf) - ret,
				"Raw Data - Row(%d), Col(%d)\n",
				row_size, col_size);

	ret += snprintf(w_buf + ret, sizeof(w_buf) - ret,
				"==========================================================================================================");

	ret += snprintf(w_buf + ret, sizeof(w_buf) - ret,
				"====================================================================================\n   :");

	for (i = 0 ; i < row_size ; i++)
		ret += snprintf(w_buf + ret, sizeof(w_buf) - ret, "%5d ", i + 1);

	ret += snprintf(w_buf + ret, sizeof(w_buf) - ret,
				"\n----------------------------------------------------------------------------------------------------------");

	ret += snprintf(w_buf + ret, sizeof(w_buf) - ret,
				"------------------------------------------------------------------------------------\n");

	for (i = 0 ; i < col_size ; i++) {
		ret += snprintf(w_buf + ret, sizeof(w_buf) - ret,
					"%2d : ", i + 1);
		for (j = 0 ; j < row_size ; j++) {
			ret += snprintf(w_buf + ret, sizeof(w_buf) - ret, "%5d ",
					buffer[(col_size * i + j) * 2 + 1] << 8
					| buffer[(col_size * i + j) * 2]);
		}
		ret += snprintf(w_buf + ret, sizeof(w_buf) - ret, "\n");
	}
	ret += snprintf(w_buf + ret, sizeof(w_buf) - ret,
				"----------------------------------------------------------------------------------------------------------");

	ret += snprintf(w_buf + ret, sizeof(w_buf) - ret,
				"------------------------------------------------------------------------------------\n");

	write_file(dev, w_buf, TIME_INFO_SKIP);
	TOUCH_I("Raw Data Test - Completed\n");
	kfree(buffer);

	return 0;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sharp_data *d = to_sharp_data(dev);

	int ret = 0;
	int rawdata_ret = 0;
	int chstatus_ret = 0;

	TOUCH_I("%s\n", __func__);

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"power_state[suspend]. Can not sd.\n");
		return ret;
	}

	/* file create , time log */
	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test Start\n");

	firmware_version_log(dev);

	mutex_lock(&ts->lock);
	touch_disable_irq(ts->irq);

	sharp_get_limit(dev, "sharp_limit", NUM_DRIVE, NUM_SENSE);
	sharp_cmd_calibration(dev);
	sharp_cmd_sd(dev);

	if ((d->cmd_buf[0x08 - 0x08] == 0xDA) &&
		(d->cmd_buf[0x09 - 0x08] == 0x00)) {
		if (d->cmd_buf[0x0a - 0x08] == 0x00) {
			rawdata_ret = 1;
			chstatus_ret = 1;
			TOUCH_I("SD command test passed\n");
		} else if (d->cmd_buf[0x0a - 0x08] == 0x01) {
			rawdata_ret = 0;
			chstatus_ret = 0;
			TOUCH_E("SD command test failed\n");
		} else if (d->cmd_buf[0x0a - 0x08] == 0xff) {
			rawdata_ret = 0;
			chstatus_ret = 0;
			TOUCH_E("SD command test abnormal termination\n");
		}
	} else if (d->cmd_buf[0x09 - 0x08] != 0x00) {
		TOUCH_E("SD ErrorCode(%02X)\n", d->cmd_buf[0x09 - 0x08]);
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"SD ErrorCode(%02X), Can not sd.\n",
				d->cmd_buf[0x09 - 0x08]);
		mutex_unlock(&ts->lock);
		return ret;
	}
	sharp_read_sd_result(dev);
	sharp_init(dev);
	touch_enable_irq(ts->irq);
	mutex_unlock(&ts->lock);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"\n========RESULT=======\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : %s",
				(chstatus_ret == 1) ? "Pass\n" : "Fail\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : %s",
				(rawdata_ret == 1) ? "Pass\n" : "Fail\n");

	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test End\n");
	log_file_size_check(dev);

	return ret;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	TOUCH_TRACE();

	return 0;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	TOUCH_TRACE();

	return 0;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	TOUCH_TRACE();
	
	return 0;
}

static TOUCH_ATTR(calibration, show_calibration, NULL);
static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);

static struct attribute *prd_attribute_list[] = {
	&touch_attr_calibration.attr,
	&touch_attr_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_lpwg_sd.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

int lr388k6_prd_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return ret;
}
