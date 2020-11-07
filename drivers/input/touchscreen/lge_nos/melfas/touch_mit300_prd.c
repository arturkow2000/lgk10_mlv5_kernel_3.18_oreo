/* touch_mit300_sysfs.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: PH1-BSP-Touch@lge.com
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
#include <linux/delay.h>

#include <touch_core.h>
#include <touch_hwif.h>

#include "touch_mit300.h"
#include "touch_mit300_prd.h"

extern void touch_interrupt_control(struct device *dev, int on_off);

static char line[BUF_SIZE];
bool test_busy = false;

static void log_file_size_check(struct device *dev)
{
	char *fname = NULL;
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n",
				__func__);
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n",
				__func__, PTR_ERR(file), fname);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n",
			__func__, fname, file_size);

	filp_close(file, 0);

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
				__func__, fname, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				sprintf(buf1, "%s", fname);
			else
				sprintf(buf1, "%s.%d", fname, i);

			ret = sys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n",
						__func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n",
								__func__, buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n",
							__func__, buf1);
				} else {
					sprintf(buf2, "%s.%d",
							fname,
							(i + 1));

					if (sys_rename(buf1, buf2) < 0) {
						TOUCH_E("%s : failed to rename file [%s] -> [%s]\n",
								__func__, buf1, buf2);
						goto error;
					}

					TOUCH_I("%s : rename file [%s] -> [%s]\n",
							__func__, buf1, buf2);
				}
			} else {
				TOUCH_I("%s : file [%s] does not exist (ret = %d)\n",
						__func__, buf1, ret);
			}
		}
	}

error:
	set_fs(old_fs);
	return;
}

static void write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/mnt/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
		sys_chmod(fname, 0666);
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

static void write_excel_file(struct device *dev, char *data)
{
	int fd = 0;
	char *fname = NULL;
	mm_segment_t old_fs = get_fs();
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	TOUCH_I("write_excel_file start \n");
	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/mnt/sdcard/touch_self_test.csv";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.csv";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.csv";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		sys_write(fd, data, strlen(data));
		sys_close(fd);
		TOUCH_I("%s saved \n", fname);
	} else {
		TOUCH_E("File open failed\n");
	}
	set_fs(old_fs);

	TOUCH_I("write_excel_file end \n");

}

static int sdcard_spec_file_read(struct device *dev)
{
	int ret = 0;
	int fd = 0;
	char *path[2] = { "/mnt/sdcard/ph1_limit.txt",
		"/mnt/sdcard/ph1_limit_mfts.txt" };
	int boot_mode = 0;

	mm_segment_t old_fs = get_fs();

	if (touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER)
		boot_mode = 1;
	else
		boot_mode = 0;

	set_fs(KERNEL_DS);

	fd = sys_open(path[boot_mode], O_RDONLY, 0);
	if (fd >= 0) {
		sys_read(fd, line, sizeof(line));
		sys_close(fd);
		TOUCH_I("%s file existing\n", path[boot_mode]);
		ret = 1;
	}
	set_fs(old_fs);

	return ret;
}

static int spec_file_read(struct device *dev)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fwlimit = NULL;
	const char *path[2] = { ts->panel_spec, ts->panel_spec_mfts };
	int boot_mode = 0;

	if (touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER)
		boot_mode = 1;
	else
		boot_mode = 0;

	if (ts->panel_spec == NULL || ts->panel_spec_mfts == NULL) {
		TOUCH_I("panel_spec_file name is null\n");
		ret = -1;
		goto error;
	}

	if (request_firmware(&fwlimit, path[boot_mode], dev) < 0) {
		TOUCH_I("request ihex is failed in normal mode\n");
		ret = -1;
		goto error;
	}

	if (fwlimit->data == NULL) {
		ret = -1;
		TOUCH_I("fwlimit->data is NULL\n");
		goto error;
	}

	strlcpy(line, fwlimit->data, sizeof(line));

error:
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}

static int get_limit(struct device *dev, char *breakpoint)
{
	int p = 0;
	int q = 0;
	int cipher = 1;
	int ret = 0;
	char *found;
	int boot_mode = 0;
	int file_exist = 0;
	int limit_data = 0;


	if (breakpoint == NULL) {
		ret = -1;
		goto error;
	}

	boot_mode = touch_boot_mode_check(dev);
	TOUCH_I("boot_mode : %d\n", boot_mode);

	if (boot_mode > MINIOS_MFTS_CURVED
			|| boot_mode < TOUCH_NORMAL_BOOT) {
		ret = -1;
		goto error;
	}

	file_exist = sdcard_spec_file_read(dev);
	if (!file_exist) {
		ret = spec_file_read(dev);
		if (ret == -1)
			goto error;
	}

	if (line == NULL) {
		ret =  -1;
		goto error;
	}

	found = strnstr(line, breakpoint, sizeof(line));
	if (found != NULL) {
		q = found - line;
	} else {
		TOUCH_I(
			"failed to find breakpoint. The panel_spec_file is wrong");
		ret = -1;
		goto error;
	}

	while (1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') &&
					(line[q - p] <= '9'); p++) {
				limit_data += ((line[q - p] - '0') * cipher);
				cipher *= 10;

			}
			TOUCH_I("[limit_data] %s = %d\n", breakpoint, limit_data);
			ret = limit_data;
			break;
		}
		q++;
	}

	TOUCH_I("panel_spec_file scanning is success\n");
	return ret;

error:
	return ret;
}

static void firmware_version_log(struct device *dev)
{
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;
	unsigned char buffer[LOG_BUF_SIZE] = {0,};
	int boot_mode = 0;

	boot_mode = touch_boot_mode_check(dev);
	if (boot_mode)
		ret = mit300_ic_info(dev);

	ret = snprintf(buffer, LOG_BUF_SIZE,
			"======== Firmware Info ========\n");
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"version : v%d.%02d\n",
			d->module.version[0], d->module.version[1]);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"product : %s\n", d->module.product_code);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"row(node_y) : %d, col(node_x) : %d\n", d->dev.row_num, d->dev.col_num);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"ic name : %s\n", d->module.ic_name);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"max_x : %d max_y :%d\n", d->dev.x_resolution, d->dev.y_resolution);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"===============================\n\n");

	write_file(dev, buffer, TIME_INFO_SKIP);
}
/**
* Process table data
*/
static int mip_proc_table_data(struct device *dev, u8 size, u8 data_type_size, u8 data_type_sign, u8 buf_addr_h, u8 buf_addr_l, u8 row_num, u8 col_num, u8 buf_col_num, u8 rotate, u8 key_num)
{
	struct mit_data *d = to_mit_data(dev);
	char data[10];
	int i_col, i_row;
	int i_x, i_y;
	int lim_x, lim_y;
	int lim_col, lim_row;
	int max_x = 0;
	int max_y = 0;
	bool flip_x = false;
	int sValue = 0;
	unsigned int uValue = 0;
	int value = 0;
	u8 wbuf[8];
	u8 rbuf[512];
	unsigned int buf_addr;
	int offset;
	int data_size = data_type_size;
	int data_sign = data_type_sign;
	int has_key = 0;

	memset(data, 0, 10);

	/* set axis */
	if (rotate == 0) {
		max_x = col_num;
		max_y = row_num;
		if (key_num > 0) {
			max_y += 1;
			has_key = 1;
		}
		flip_x = false;
	} else if (rotate == 1) {
		max_x = row_num;
		max_y = col_num;
		if (key_num > 0) {
			max_y += 1;
			has_key = 1;
		}
		flip_x = true;
	} else {
		TOUCH_E("[ERROR] rotate [%d]\n", rotate);
		goto ERROR;
	}

	/* get table data */
	lim_row = row_num + has_key;
	for (i_row = 0; i_row < lim_row; i_row++) {
		/* get line data */
		offset = buf_col_num * data_type_size;
		size = col_num * data_type_size;

		buf_addr = (buf_addr_h << 8) | buf_addr_l | (offset * i_row);
		wbuf[0] = (buf_addr >> 8) & 0xFF;
		wbuf[1] = buf_addr & 0xFF;

		if (mit300_reg_read(dev, wbuf,2, rbuf, size)) {
			TOUCH_E("[ERROR] Read data buffer\n");
			goto ERROR;
		}

		/* save data */
		if ((key_num > 0) && (i_row == (lim_row - 1))) {
			lim_col = key_num;
		} else {
			lim_col = col_num;
		}
		for (i_col = 0; i_col < lim_col; i_col++) {
			if (data_sign == 0){
				/* unsigned */
				if (data_size == 1){
					uValue = (u8)rbuf[i_col];
				}
				else if (data_size == 2){
					uValue = (u16)(rbuf[data_size * i_col]
							| (rbuf[data_size * i_col + 1] << 8));
				}
				else if (data_size == 4){
					uValue = (u32)(rbuf[data_size * i_col]
							| (rbuf[data_size * i_col + 1] << 8)
							| (rbuf[data_size * i_col + 2] << 16)
							| (rbuf[data_size * i_col + 3] << 24));
				}
				else {
					TOUCH_E("[ERROR] data_size [%d]\n", data_size);
					goto ERROR;
				}
				value = (int)uValue;
			} else {
				/* signed */
				if (data_size == 1) {
					sValue = (s8)rbuf[i_col];
				} else if (data_size == 2) {
					sValue = (s16)(rbuf[data_size * i_col]
							| (rbuf[data_size * i_col + 1] << 8));
				} else if (data_size == 4) {
					sValue = (s32)(rbuf[data_size * i_col]
							| (rbuf[data_size * i_col + 1] << 8)
							| (rbuf[data_size * i_col + 2] << 16)
							| (rbuf[data_size * i_col + 3] << 24));
				} else {
					TOUCH_E("[ERROR] data_size [%d]\n", data_size);
					goto ERROR;
				}
				value = (int)sValue;
			}

			switch (rotate) {
				case 0:
					d->mit_data[i_row][i_col] = value;
					d->intensity_data[i_row][i_col] = value;
					break;
				case 1:
					if((key_num > 0) && (i_row == (lim_row - 1))) {
						d->mit_data[i_row][i_col] = value;
						d->intensity_data[i_row][i_col] = value;
					} else {
						d->mit_data[i_col][i_row] = value;
						d->intensity_data[i_col][i_row] = value;
					}
					break;
				default:
					TOUCH_E("[ERROR] rotate [%d]\n", rotate);
					goto ERROR;
					break;
			}
		}
	}

	/* print table header */
	sprintf(data, "    ");
	memset(data, 0, 10);

	switch(data_size) {
		case 1:
			for (i_x = 0; i_x < max_x; i_x++) {
				sprintf(data, "[%2d]", i_x);
				memset(data, 0, 10);
			}
			break;
		case 2:
			for (i_x = 0; i_x < max_x; i_x++) {
				sprintf(data, "[%4d]", i_x);
				memset(data, 0, 10);
			}
			break;
		case 4:
			for (i_x = 0; i_x < max_x; i_x++) {
				sprintf(data, "[%5d]", i_x);
				memset(data, 0, 10);
			}
			break;
		default:
			TOUCH_E("[ERROR] data_size [%d]\n", data_size);
			goto ERROR;
			break;
	}

	sprintf(data, "\n");
	memset(data, 0, 10);

	/* print table */
	lim_y = max_y;
	for (i_y = 0; i_y < lim_y; i_y++) {
		/* print line header */
		if ((key_num > 0) && (i_y == (lim_y -1))) {
			sprintf(data, "[TK]");
		} else {
			sprintf(data, "[%2d]", i_y);
		}
		memset(data, 0, 10);

		/* print line */
		if ((key_num > 0) && (i_y == (lim_y - 1))) {
			lim_x = key_num;
		} else {
			lim_x = max_x;
		}

		sprintf(data, "\n");
		memset(data, 0, 10);
	}

	sprintf(data, "\n");
	memset(data, 0, 10);

	return 0;

ERROR:
	TOUCH_E("[ERROR]\n");
	return 1;
}


/**
* Get ready status
*/
int mip_get_ready_status(struct device *dev)
{
	u8 wbuf[16];
	u8 rbuf[16] = {0};
	int ret = 0;

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_READY_STATUS;
	if (mit300_reg_read(dev, wbuf, 2, rbuf, 1) < 0) {
		TOUCH_E("[ERROR] mip_i2c_read\n");
		goto ERROR;
	}
	ret = rbuf[0];

	/* check status */
	if ((ret == MIP_CTRL_STATUS_NONE) || (ret == MIP_CTRL_STATUS_LOG) || (ret == MIP_CTRL_STATUS_READY)) {
		if (ret == 0xA0)
			TOUCH_I("%s - status [0x%02X]\n", __func__, ret);
	} else {
		TOUCH_E("[ERROR] Unknown status [0x%02X]\n", ret);
		goto ERROR;
	}

	if (ret == MIP_CTRL_STATUS_LOG) {
		/* skip log event */
		wbuf[0] = MIP_R0_LOG;
		wbuf[1] = MIP_R1_LOG_TRIGGER;
		wbuf[2] = 0;
		if( mit300_reg_write(dev, wbuf, 3)) {
			TOUCH_E("[ERROR] mip_i2c_write\n");
		}
	}

	return ret;

ERROR:
	TOUCH_E("[ERROR]\n");
	return -1;
}

int mip_get_image(struct device *dev, u8 image_type)
{
    int busy_cnt = 100;
	int wait_cnt = 100;
	u8 wbuf[8];
	u8 rbuf[512] = {0};
	u8 size = 0;
	u8 row_num;
	u8 col_num;
	u8 buffer_col_num;
	u8 rotate;
	u8 key_num;
	u8 data_type;
	u8 data_type_size;
	u8 data_type_sign;
	u8 buf_addr_h;
	u8 buf_addr_l;

	TOUCH_I("%s [START]\n", __func__);
	TOUCH_I("%s - image_type[%d]\n", __func__, image_type);

	/* check busy status */
	while (busy_cnt--) {
		if (test_busy == false) {
			break;
		}
		TOUCH_I("%s - busy_cnt[%d]\n", __func__, busy_cnt);
		touch_msleep(5);
	}

	test_busy = true;

	/* check image type */
	switch (image_type) {
		case MIP_IMG_TYPE_INTENSITY:
			TOUCH_I("=== Intensity Image ===\n");
			break;
		case MIP_IMG_TYPE_RAWDATA:
			TOUCH_I("=== Rawdata Image ===\n");
			break;
		default:
			TOUCH_E( "[ERROR] Unknown image type\n");
			goto ERROR;
			break;
	}
	/* set event interrupt type none(=set disable interrupt)(0x0610) */
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_EVENT_TRIGGER_TYPE;
	wbuf[2] = MIP_TRIGGER_TYPE_NONE;
	if (mit300_reg_write(dev, wbuf, 3) < 0) {
		TOUCH_E("[ERROR] Write interrupt none\n");
		goto ERROR;
	}

	/* set image type(TEST_CS_CONTROL_SET)(0x0C10) */
	wbuf[0] = MIP_R0_IMAGE;
	wbuf[1] = MIP_R1_IMAGE_TYPE;
	wbuf[2] = image_type;
	if (mit300_reg_write(dev, wbuf, 3) < 0) {
		TOUCH_E("[ERROR] Write image type\n");
		goto ERROR;
	}

	TOUCH_I("%s - set image type\n", __func__);

	/* wait ready status(CHECK PACKET SIZE & MARKER) */
	wait_cnt = 100;
	while (wait_cnt--) {
		if (mip_get_ready_status(dev) == MIP_CTRL_STATUS_READY) {
			break;
		}
		touch_msleep(10);

		TOUCH_I("%s - wait [%d]\n", __func__, wait_cnt);
	}

	if (wait_cnt <= 0) {
		TOUCH_E("[ERROR] Wait timeout\n");
		goto ERROR;
	}

	TOUCH_I("%s - ready\n", __func__);

	/* get data format(Read TEST CS DATA FORMAT)(0x0A20) */
	wbuf[0] = MIP_R0_IMAGE;
	wbuf[1] = MIP_R1_IMAGE_DATA_FORMAT;
	if (mit300_reg_read(dev, wbuf,2, rbuf, 6) < 0) {
		TOUCH_E("[ERROR] Read data format\n");
		goto ERROR;
	}
	row_num = rbuf[0];
	col_num = rbuf[1];
	buffer_col_num = rbuf[2];
	rotate = rbuf[3];
	key_num = rbuf[4];
	data_type = rbuf[5];

	data_type_sign = (data_type & 0x80) >> 7;
	data_type_size = data_type & 0x7F;

	TOUCH_I("%s - row_num[%d] col_num[%d] buffer_col_num[%d] rotate[%d] key_num[%d]\n", __func__, row_num, col_num, buffer_col_num, rotate, key_num);
	TOUCH_I("%s - data_type[0x%02X] data_sign[%d] data_size[%d]\n", __func__, data_type, data_type_sign, data_type_size);

	/* get buf addr(Read TEST CS TEST INFO)(0x0A00) */
	wbuf[0] = MIP_R0_IMAGE;
	wbuf[1] = MIP_R1_IMAGE_BUF_ADDR;

	if (mit300_reg_read(dev, wbuf, 2,rbuf, 2) < 0) {
		TOUCH_E("[ERROR] Read buf addr\n");
		goto ERROR;
	}

	buf_addr_l = rbuf[0];
	buf_addr_h = rbuf[1];
	TOUCH_I("%s - buf_addr[0x%02X 0x%02X]\n", __func__, buf_addr_h, buf_addr_l);

	/* print data */
	if (mip_proc_table_data(dev, size, data_type_size, data_type_sign, buf_addr_h, buf_addr_l, row_num, col_num, buffer_col_num, rotate, key_num )) {
		TOUCH_E("[ERROR] mip_proc_table_data\n");
		goto ERROR;
	}

	/* clear image type(0x0C10) */
	wbuf[0] = MIP_R0_IMAGE;
	wbuf[1] = MIP_R1_IMAGE_TYPE;
	wbuf[2] = MIP_IMG_TYPE_NONE;

	if  (mit300_reg_write(dev, wbuf, 3) < 0) {
		TOUCH_E("[ERROR] Clear image type\n");
		goto ERROR;
	}

	/* set event interrupt type(0x0611) */
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_EVENT_TRIGGER_TYPE;
	wbuf[2] = MIP_TRIGGER_TYPE_INTR;
	if (mit300_reg_write(dev, wbuf, 3) < 0) {
		TOUCH_E("[ERROR] Write interrupt mode\n");
		goto ERROR;
	}

	/* exit */
	test_busy = false;

	TOUCH_I("%s [DONE]\n", __func__);
	return 0;

ERROR:
	test_busy = false;

	TOUCH_E("[ERROR]\n");
	return 1;
}

static int get_intensity(struct device *dev)
{
  if (mip_get_image(dev, MIP_IMG_TYPE_INTENSITY)) {
	  TOUCH_E("[ERROR]\n");
	  return -1;
  }
  return 0;
}

static int print_intensity(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int col = 0;
	int row = 0;
	int ret = 0;

	ret += sprintf(buf + ret,"===== INTENSITY TEST =====\n");

	for (row = 0; row < d->dev.row_num; row++) {
		printk("[Touch] [%2d]  ", row);
		ret += sprintf(buf + ret,"[%2d]  ", row);
		for (col = 0; col < d->dev.col_num; col++) {
			ret += sprintf(buf + ret,"%4d ", d->intensity_data[row][d->dev.col_num - col - 1]);
			printk("%4d ", d->intensity_data[row][d->dev.col_num - col - 1]);
		}
		printk("\n");
		ret += sprintf(buf + ret,"\n");
	}

	return ret;
}

static int get_rawdata(struct device *dev)
{
	if (mip_get_image(dev, MIP_IMG_TYPE_RAWDATA)) {
		TOUCH_E("[ERROR]\n");
		return -1;
	}
	return 0;
}

static int print_rawdata(struct device *dev, char *buf,int type)
{
	struct mit_data *d = to_mit_data(dev);
	int col = 0;
	int row = 0;
	int ret = 0;
	u32 limit_upper = 0;
	u32 limit_lower = 0;
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	d->r_min = d->mit_data[0][0];
	d->r_max = d->mit_data[0][0];


	snprintf(lower_str, sizeof(lower_str),"Raw_data_min");
	snprintf(upper_str, sizeof(upper_str),"Raw_data_max");

	d->limit->raw_data_min = get_limit(dev, lower_str);
	d->limit->raw_data_max = get_limit(dev, upper_str);

	limit_upper = d->limit->raw_data_max;
	limit_lower = d->limit->raw_data_min;

	ret += sprintf(buf + ret,"\n===== RAWDATA TEST =====\n");

	for (row = 0; row < d->dev.row_num; row++) {
		if (type == RAW_DATA_SHOW) {
			ret += sprintf(buf + ret,"[%2d]  ",row);
			printk("[Touch] [%2d]  ",row);
		}

		for (col = 0; col < d->dev.col_num; col++) {
			if ((d->mit_data[row][col] >= d->limit->raw_data_min) && (d->mit_data[row][col] <= d->limit->raw_data_max)) {
				printk("%5d ", d->mit_data[row][col]);
				ret += sprintf(buf + ret,"%5d ", d->mit_data[row][col]);
			} else {
				printk("!%4d ", d->mit_data[row][col]);
				ret += sprintf(buf + ret,"!%4d ", d->mit_data[row][col]);
			}
			if (type == RAW_DATA_STORE) {
				ret += sprintf(buf + ret,",");
			}

			d->r_min = (d->r_min > d->mit_data[row][col]) ? d->mit_data[row][col] : d->r_min;
			d->r_max = (d->r_max < d->mit_data[row][col]) ? d->mit_data[row][col] : d->r_max;

		}

		if (type == RAW_DATA_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == RAW_DATA_SHOW) {

		for (row = 0; row < d->dev.row_num; row++) {
			for (col = 0; col < d->dev.col_num; col++) {
				if (d->mit_data[row][col] < limit_lower || d->mit_data[row][col] > limit_upper)
					d->selfdiagnostic_state[SD_RAWDATA] = 0;
			}
		}

		printk("\n");
		ret += sprintf(buf + ret,"\n");

		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n",d->r_max , d->r_min, d->r_max - d->r_min);
		TOUCH_I("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n",d->r_max , d->r_min, d->r_max - d->r_min);

		TOUCH_I("RAW DATA SPEC (UPPER : %d  LOWER : %d)\n",
				limit_upper, limit_lower);
		ret += sprintf(buf+ret,"RAW DATA SPEC (UPPER : %d  LOWER : %d)\n",
				limit_upper, limit_lower);

		if (d->selfdiagnostic_state[SD_RAWDATA] == 1) {
			TOUCH_I("RAW DATA Test : Pass\n");
			ret += sprintf(buf + ret,"RAWDATA Test : PASS\n");
		} else {
			TOUCH_I("RAW DATA Test : Fail\n");
			ret += sprintf(buf + ret,"RAW DATA Test : FAIL\n");
		}
	}

	return ret;
}

static int print_excel_rawdata(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int col = 0;
	int row = 0;
	int ret = 0;

	TOUCH_I("print_rawdata start \n");

	for (row = 0; row < d->dev.row_num; row++) {

		for (col = 0; col < d->dev.col_num; col++) {
			printk("%5d ", d->mit_data[row][col]);
			ret += sprintf(buf + ret,"%5d ", d->mit_data[row][col]);
			ret += sprintf(buf + ret,",");

		}

	}

	ret += sprintf(buf + ret,"\n");
	write_excel_file(dev, buf);
	touch_msleep(30);

	TOUCH_I("print_rawdata end \n");

	memset(buf, 0, PAGE_SIZE);


	return ret;
}

static int print_cm_delta_data(struct device *dev, char *buf, int type)
{
	struct mit_data *d = to_mit_data(dev);
	int col = 0;
	int row = 0;
	int ret = 0;
	char cmdelta_str[64] = {0, };
	d->d_min = d->mit_data[0][0];
	d->d_max = d->mit_data[0][0];

	snprintf(cmdelta_str, sizeof(cmdelta_str),"Cm_delta");

	d->limit->cm_delta = get_limit(dev, cmdelta_str);

	ret += sprintf(buf + ret,"\n===== CM DELTA TEST =====\n");

	for (row = 0; row < d->dev.row_num; row++) {
		if (type == CM_DELTA_SHOW) {
			printk("[Touch] [%2d]  ", row);
			ret += sprintf(buf + ret,"[%2d]  ", row);
		}

		for (col = 0; col < d->dev.col_num; col++) {
			if (d->mit_data[row][col] >= d->limit->cm_delta) {
				printk("%5d ", d->mit_data[row][col]);
				ret += sprintf(buf + ret,"%5d ", d->mit_data[row][col]);
			} else {
				printk("!%4d ", d->mit_data[row][col]);
				ret += sprintf(buf + ret,"!%4d ", d->mit_data[row][col]);
			}

			if (type == CM_DELTA_STORE) {
				ret += sprintf(buf + ret,",");
			}

			d->d_min = (d->d_min > d->mit_data[row][col]) ? d->mit_data[row][col] : d->d_min;
			d->d_max = (d->d_max < d->mit_data[row][col]) ? d->mit_data[row][col] : d->d_max;

		}

		if (type == CM_DELTA_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == CM_DELTA_SHOW) {

		if (d->d_min < d->limit->cm_delta)
			d->selfdiagnostic_state[SD_CM_DELTA] = 0;

		printk("\n");
		ret += sprintf(buf + ret,"\n");

		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n",d->d_max , d->d_min, d->d_max - d->d_min);
		TOUCH_I("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n",d->d_max , d->d_min, d->d_max - d->d_min);

		TOUCH_I("CM DELTA TEST SPEC : %d\n", d->limit->cm_delta);
		ret += sprintf(buf + ret,"CM DELTA TEST SPEC : %d\n", d->limit->cm_delta);
		if (d->selfdiagnostic_state[SD_CM_DELTA] == 1) {
			TOUCH_I("CM DELTA Test : Pass\n");
			ret += sprintf(buf + ret,"CM DELTA Test : PASS\n");
		} else {
			TOUCH_I("CM DELTA Test : Fail\n");
			ret += sprintf(buf + ret,"CM DELTA Test : FAIL\n");
		}
	}
	return ret;
}

static int  print_cm_jitter_data(struct device *dev, char *buf, int type)
{
	struct mit_data *d = to_mit_data(dev);
	int col = 0;
	int row = 0;
	int ret = 0;
	char cmjitter_str[64] = {0, };
	d->j_min = d->mit_data[0][0];
	d->j_max = d->mit_data[0][0];

	snprintf(cmjitter_str, sizeof(cmjitter_str),"Cm_jitter");

	d->limit->cm_jitter = get_limit(dev, cmjitter_str);

	ret += sprintf(buf + ret,"\n===== CM JITTER TEST =====\n");

	for (row = 0; row < d->dev.row_num; row++) {
		if (type == CM_JITTER_SHOW) {
			printk("[Touch] [%2d]  ", row);
			ret += sprintf(buf + ret,"[%2d]  ", row);
		}

		for (col = 0 ; col < d->dev.col_num ; col++) {
			if (d->mit_data[row][col] <= d->limit->cm_jitter) {
				printk("%5d ", d->mit_data[row][col]);
				ret += sprintf(buf + ret,"%5d ", d->mit_data[row][col]);
			} else {
				printk("!%4d ", d->mit_data[row][col]);
				ret += sprintf(buf + ret,"!%4d ", d->mit_data[row][col]);
			}

			if (type == CM_JITTER_STORE) {
				ret += sprintf(buf + ret,",");
			}

			d->j_min = (d->j_min > d->mit_data[row][col]) ? d->mit_data[row][col] : d->j_min;
			d->j_max = (d->j_max < d->mit_data[row][col]) ? d->mit_data[row][col] : d->j_max;

		}

		if (type == CM_JITTER_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == CM_JITTER_SHOW) {

		if (d->j_max > d->limit->cm_jitter)
			d->selfdiagnostic_state[SD_CM_JITTER] = 0;

		printk("\n");
		ret += sprintf(buf + ret,"\n");

		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n",d->j_max , d->j_min, d->j_max - d->j_min);
		TOUCH_I("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n",d->j_max , d->j_min, d->j_max - d->j_min);

		TOUCH_I("CM JITTER TEST SPEC: %d\n", d->limit->cm_jitter);
		ret += sprintf(buf + ret,"CM JITTER TEST SPEC: %d\n", d->limit->cm_jitter);
		if (d->selfdiagnostic_state[SD_CM_JITTER] == 1) {
			TOUCH_I("CM JITTER Test : Pass\n");
			ret += sprintf(buf + ret,"CM JITTER Test : PASS\n");
		} else {
			TOUCH_I("CM JITTER Test : Fail\n");
			ret += sprintf(buf + ret,"CM JITTER Test : FAIL\n");
		}
	}
	return ret;
}

static int  print_openshort_data(struct device *dev, char *buf, int type)
{
	struct mit_data *d = to_mit_data(dev);
	int col = 0;
	int row = 0;
	int ret = 0;
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	d->o_min = d->mit_data[0][0];
	d->o_max = d->mit_data[0][0];

	snprintf(lower_str, sizeof(lower_str),"Open_short_min");
	snprintf(upper_str, sizeof(upper_str),"Open_short_max");

	d->limit->open_short_min = get_limit(dev, lower_str);
	d->limit->open_short_max = get_limit(dev, upper_str);

	ret += sprintf(buf + ret,"===== OPEN SHORT TEST =====\n");

	for (row = 0; row < d->dev.row_num; row++) {
		if (type == OPENSHORT) {
			printk("[Touch] [%2d]  ", row);
			ret += sprintf(buf + ret,"[%2d]  ", row);
		}

		for (col = 0; col < d->dev.col_num; col++) {
			if ((d->mit_data[row][col] >= d->limit->open_short_min) && (d->mit_data[row][col] <= d->limit->open_short_max)) {
				printk("%5d ", d->mit_data[row][col]);
				ret += sprintf(buf + ret,"%5d ", d->mit_data[row][col]);
			} else {
				printk("!%4d ", d->mit_data[row][col]);
				ret += sprintf(buf + ret,"!%4d ", d->mit_data[row][col]);
			}
			if (type == OPENSHORT_STORE) {
				ret += sprintf(buf + ret,",");
			}

			d->o_min = (d->o_min > d->mit_data[row][col]) ? d->mit_data[row][col] : d->o_min;
			d->o_max = (d->o_max < d->mit_data[row][col]) ? d->mit_data[row][col] : d->o_max;

		}

		if (type == OPENSHORT) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == OPENSHORT) {
		if ((d->o_min < d->limit->open_short_min) || (d->o_max > d->limit->open_short_max))
			d->selfdiagnostic_state[SD_OPENSHORT] = 0;

		printk("\n");
		ret += sprintf(buf + ret,"\n");

		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n",d->o_max , d->o_min, d->o_max - d->o_min);
		TOUCH_I("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n",d->o_max , d->o_min, d->o_max - d->o_min);

		TOUCH_I("OPEN / SHORT TEST SPEC(MIN : %d, MAX : %d)\n", d->limit->open_short_min, d->limit->open_short_max);
		ret += sprintf(buf + ret,"OPEN / SHORT TEST SPEC(MIN : %d, MAX : %d)\n", d->limit->open_short_min, d->limit->open_short_max);
		if (d->selfdiagnostic_state[SD_OPENSHORT] == 1) {
			TOUCH_I("OpenShort Test : Pass\n");
			ret += sprintf(buf + ret,"OpenShort Test : PASS\n");
		} else {
			TOUCH_I("OpenShort Test : Fail\n");
			ret += sprintf(buf + ret,"OpenShort Test : FAIL\n");
		}
	}
	return ret;
}


static int  print_muxshort_data(struct device *dev, char *buf, int type)
{
	struct mit_data *d = to_mit_data(dev);
	int col = 0;
	int row = 0;
	int ret = 0;
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	d->m_min = d->mit_data[0][0];
	d->m_max = d->mit_data[0][0];

	snprintf(lower_str, sizeof(lower_str),"Mux_short_min");
	snprintf(upper_str, sizeof(upper_str),"Mux_short_max");

	d->limit->mux_short_min = get_limit(dev, lower_str);
	d->limit->mux_short_max = get_limit(dev, upper_str);

	ret += sprintf(buf + ret,"\n===== MUX SHORT TEST =====\n");

	for (row = 0; row < d->dev.row_num; row++) {
		if (type == MUXSHORT_SHOW) {
			printk("[Touch] [%2d]  ", row);
			ret += sprintf(buf + ret,"[%2d]  ", row);
		}

		for (col = 0; col < d->dev.col_num; col++) {
			if ((d->mit_data[row][col] >= d->limit->mux_short_min) && (d->mit_data[row][col] <= d->limit->mux_short_max)) {
				printk("%5d ", d->mit_data[row][col]);
				ret += sprintf(buf + ret,"%5d ", d->mit_data[row][col]);
			} else {
				printk("!%4d ", d->mit_data[row][col]);
				ret += sprintf(buf + ret,"!%4d ", d->mit_data[row][col]);
			}

			d->m_min = (d->m_min > d->mit_data[row][col]) ? d->mit_data[row][col] : d->m_min;
			d->m_max = (d->m_max < d->mit_data[row][col]) ? d->mit_data[row][col] : d->m_max;
		}

		if (type == MUXSHORT_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == MUXSHORT_SHOW) {
		if ((d->m_min < d->limit->mux_short_min) || (d->m_max > d->limit->mux_short_max))
			d->selfdiagnostic_state[SD_MUXSHORT] = 0;

		printk("\n");
		ret += sprintf(buf + ret,"\n");

		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n",d->m_max , d->m_min, d->m_max - d->m_min);
		TOUCH_I("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n",d->m_max , d->m_min, d->m_max - d->m_min);

		TOUCH_I("MUX SHORT TEST SPEC(MIN : %d, MAX : %d)\n", d->limit->mux_short_min, d->limit->mux_short_max);
		ret += sprintf(buf + ret,"MUX SHORT TEST SPEC(MIN : %d, MAX : %d)\n", d->limit->mux_short_min, d->limit->mux_short_max);
		if (d->selfdiagnostic_state[SD_MUXSHORT] == 1) {
			TOUCH_I("Mux Short Test : Pass\n");
			ret += sprintf(buf + ret,"Mux Short Test : PASS\n");
		} else {
			TOUCH_I("Mux Short Test : Fail\n");
			ret += sprintf(buf + ret,"Mux Short Test : FAIL\n");
		}
	}
	return ret;
}

static int  print_lpwg_jitter_data(struct device *dev, char *buf, int type)
{
	struct mit_data *d = to_mit_data(dev);
	int col = 0;
	int row = 0;
	int ret = 0;
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	d->l_j_min = d->mit_data[0][0];
	d->l_j_max = d->mit_data[0][0];

	snprintf(lower_str, sizeof(lower_str),"Lpwg_jitter_min");
	snprintf(upper_str, sizeof(upper_str),"Lpwg_jitter_max");

	d->limit->lpwg_jitter_min = get_limit(dev, lower_str);
	d->limit->lpwg_jitter_max = get_limit(dev, upper_str);

	ret += sprintf(buf + ret,"===== LPWG JITTER TEST =====\n");

	for (row = 0; row < d->dev.row_num; row++) {
		if (type == LPWG_JITTER_SHOW) {
			printk("[Touch] [%2d]  ", row);
			ret += sprintf(buf + ret,"[%2d]  ", row);
		}

		for (col = 0; col < d->dev.col_num; col++) {
			if (col >= (d->dev.col_num - 8)) {
				printk(" XXX ");
				ret += sprintf(buf + ret," XXX ");
			} else {
				if ((d->mit_data[row][col] >= d->limit->lpwg_jitter_min) && (d->mit_data[row][col] <= d->limit->lpwg_jitter_max)) {
					printk("%5d ", d->mit_data[row][col]);
					ret += sprintf(buf + ret,"%5d ", d->mit_data[row][col]);
				} else {
					printk("!%4d ", d->mit_data[row][col]);
					ret += sprintf(buf + ret,"!%4d ", d->mit_data[row][col]);
				}
			}

			if (col < (d->dev.col_num - 8)) {
				d->l_j_min = (d->l_j_min > d->mit_data[row][col]) ? d->mit_data[row][col] : d->l_j_min;
				d->l_j_max = (d->l_j_max < d->mit_data[row][col]) ? d->mit_data[row][col] : d->l_j_max;
			}
		}

		if (type == LPWG_JITTER_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == LPWG_JITTER_SHOW) {
		if ((d->l_j_min < d->limit->lpwg_jitter_min) || (d->l_j_max > d->limit->lpwg_jitter_max))
			d->lpwg_selfdiagnostic_state[SD_LPWG_JITTER] = 0;

		printk("\n");
		ret += sprintf(buf + ret,"\n");

		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n",d->l_j_max , d->l_j_min, d->l_j_max - d->l_j_min);
		TOUCH_I("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n",d->l_j_max , d->l_j_min, d->l_j_max - d->l_j_min);

		TOUCH_I("LPWG JITTER TEST SPEC(MIN : %d, MAX : %d)\n", d->limit->lpwg_jitter_min, d->limit->lpwg_jitter_max);
		ret += sprintf(buf + ret,"LPWG JITTER TEST SPEC(MIN : %d, MAX : %d)\n", d->limit->lpwg_jitter_min, d->limit->lpwg_jitter_max);
		if (d->lpwg_selfdiagnostic_state[SD_LPWG_JITTER] == 1) {
			TOUCH_I("Lpwg Jitter Test : Pass\n");
			ret += sprintf(buf + ret,"Lpwg Jitter Test : PASS\n");
		} else {
			TOUCH_I("Lpwg Jitter Test : Fail\n");
			ret += sprintf(buf + ret,"Lpwg Jitter Test : FAIL\n");
		}
	}
	return ret;
}

static int  print_lpwg_abs_data(struct device *dev, char *buf, int type)
{
	struct mit_data *d = to_mit_data(dev);
	int col = 0;
	int row = 0;
	int ret = 0;
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	d->l_a_min = d->mit_data[1][1];
	d->l_a_max = d->mit_data[1][1];

	snprintf(lower_str, sizeof(lower_str),"Lpwg_abs_min");
	snprintf(upper_str, sizeof(upper_str),"Lpwg_abs_max");

	d->limit->lpwg_abs_min = get_limit(dev, lower_str);
	d->limit->lpwg_abs_max = get_limit(dev, upper_str);

	ret += sprintf(buf + ret,"\n===== LPWG ABS TEST =====\n");

	for (row = 0; row < d->dev.row_num; row++) {
		if (type == LPWG_ABS_SHOW) {
			printk("[Touch] [%2d]  ", row);
			ret += sprintf(buf + ret,"[%2d]  ", row);
		}

		for (col = 0; col < d->dev.col_num; col++) {
			if ((col == 0) || (col == d->dev.col_num-1)) {
				printk(" XXX ");
				ret += sprintf(buf + ret," XXX ");
			} else {
				if ((d->mit_data[row][col] >= d->limit->lpwg_abs_min) && (d->mit_data[row][col] <= d->limit->lpwg_abs_max)) {
					printk("%5d ", d->mit_data[row][col]);
					ret += sprintf(buf + ret,"%5d ", d->mit_data[row][col]);
				} else {
					printk("!%4d ", d->mit_data[row][col]);
					ret += sprintf(buf + ret,"!%4d ", d->mit_data[row][col]);
				}
			}

			if (col > 0 && col < d->dev.col_num-1) {
				d->l_a_min = (d->l_a_min > d->mit_data[row][col]) ? d->mit_data[row][col] : d->l_a_min;
				d->l_a_max = (d->l_a_max < d->mit_data[row][col]) ? d->mit_data[row][col] : d->l_a_max;
			}
		}

		if (type == LPWG_ABS_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == LPWG_ABS_SHOW) {
		if ((d->l_a_min < d->limit->lpwg_abs_min) || (d->l_a_max > d->limit->lpwg_abs_max))
			d->lpwg_selfdiagnostic_state[SD_LPWG_ABS] = 0;

		printk("\n");
		ret += sprintf(buf + ret,"\n");

		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n",d->l_a_max , d->l_a_min, d->l_a_max - d->l_a_min);
		TOUCH_I("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n",d->l_a_max , d->l_a_min, d->l_a_max - d->l_a_min);

		TOUCH_I("LPWG ABS TEST SPEC(MIN : %d, MAX : %d)\n", d->limit->lpwg_abs_min, d->limit->lpwg_abs_max);
		ret += sprintf(buf + ret,"LPWG ABS TEST SPEC(MIN : %d, MAX : %d)\n", d->limit->lpwg_abs_min, d->limit->lpwg_abs_max);
		if (d->lpwg_selfdiagnostic_state[SD_LPWG_ABS] == 1) {
			TOUCH_I("Lpwg Abs Test : Pass\n");
			ret += sprintf(buf + ret,"Lpwg Abs Test : PASS\n");
		} else {
			TOUCH_I("Lpwg Abs Test : Fail\n");
			ret += sprintf(buf + ret,"Lpwg Abs Test : FAIL\n");
		}
	}
	return ret;
}
int mip_run_test(struct device *dev, u8 type)
{
	int busy_cnt = 50;
	int wait_cnt = 50;
	u8 wbuf[8];
	u8 rbuf[512] = {0};
	u8 test_type = 0;
	u8 size = 0;
	u8 row_num;
	u8 col_num;
	u8 buffer_col_num;
	u8 rotate;
	u8 key_num;
	u8 data_type;
	u8 data_type_size;
	u8 data_type_sign;
	u8 buf_addr_h;
	u8 buf_addr_l;

	TOUCH_I("%s [START]\n", __func__);
	TOUCH_I("%s - type[%d]\n", __func__, type);


	/* check test type */
	switch (type) {
		case CM_DELTA_SHOW:
		case CM_DELTA_STORE:
			TOUCH_I("=== Cm Delta Test ===\n");
			test_type = MIP_TEST_TYPE_CM_DELTA;
			break;
		case CM_JITTER_SHOW:
		case CM_JITTER_STORE:
		case LPWG_JITTER_SHOW:
			TOUCH_I("=== Jitter Test ===\n");
			test_type = MIP_TEST_TYPE_CM_JITTER;
			break;
		case OPENSHORT:
		case OPENSHORT_STORE:
			TOUCH_I("=== Short Test ===\n");
			test_type = MIP_TEST_TYPE_SHORT;
			break;
		case LPWG_ABS_SHOW:
			TOUCH_I("=== Abs Test ===\n");
			test_type = MIP_TEST_TYPE_CM_ABS;
			break;
		case MUXSHORT_SHOW:
			TOUCH_I("=== MUX Short Test ===\n");
			test_type = MIP_TEST_TYPE_SHORT2;
			break;
		default:
			TOUCH_E("[ERROR] Unknown test type\n");
			goto ERROR;
			break;
	}
	TOUCH_I("%s - test_type[%d]\n", __func__, test_type);

	while (busy_cnt--) {
		if (test_busy == false) {
			break;
			TOUCH_I("%s - busy_cnt = %d\n", __func__, busy_cnt);
		}
		touch_msleep(10);
	}

	test_busy = true;

	/* set event interrupt type none(=set disable interrupt)(0x0610) */
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_EVENT_TRIGGER_TYPE;
	wbuf[2] = MIP_TRIGGER_TYPE_NONE;
	if (mit300_reg_write(dev, wbuf, 3) < 0) {
		TOUCH_E("[ERROR] Write interrupt none\n");
		goto ERROR;
	}

	/* set test mode(SET CS TEST MODE)(0x0610) */
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_MODE;
	wbuf[2] = MIP_CTRL_MODE_TEST_CM;
	if (mit300_reg_write(dev, wbuf, 3) < 0) {
		TOUCH_E("[ERROR] Write test mode\n");
		goto ERROR;
	}
	/* wait ready status(CHECK PACKET SIZE & MARKER)(0x0600) */
	wait_cnt = 100;
	while (wait_cnt--) {
		if (mip_get_ready_status(dev) == MIP_CTRL_STATUS_READY) {
			TOUCH_I("%s - wait 1_cnt : %d\n", __func__, wait_cnt);
			break;
		}
		touch_msleep(50);
	}

	if (wait_cnt <= 0) {
		TOUCH_E("%s [ERROR] Wait timeout\n", __func__);
		goto ERROR;
	}
	TOUCH_I("%s - set control mode\n", __func__);

	/* set test type (TEST CS CONTROL SET)(0x0A10) */
	wbuf[0] = MIP_R0_TEST;
	wbuf[1] = MIP_R1_TEST_TYPE;
	wbuf[2] = test_type;
	if (mit300_reg_write(dev, wbuf, 3)) {
		TOUCH_E("%s [ERROR] Write test type\n", __func__);
		goto ERROR;
	}
	TOUCH_I("%s - set test type\n", __func__);

	if (type == LPWG_JITTER_SHOW) {
		touch_msleep(1000);
		TOUCH_I("%s - touch_msleep(1000)\n", __func__);
	}
	/* wait ready status(CHECK PACKET SIZE & MARKER)(0x0600) */
	wait_cnt = 100;
	while (wait_cnt--) {
		if (mip_get_ready_status(dev) == MIP_CTRL_STATUS_READY) {
			TOUCH_I("%s - wait 2_cnt :  %d\n", __func__, wait_cnt);
			break;
		}
		touch_msleep(50);

	}

	if (wait_cnt <= 0) {
		TOUCH_E("[ERROR] Wait timeout\n");
		goto ERROR;
	}
	TOUCH_I("%s - ready\n", __func__);

	/* get data format(Read TEST CS DATA FORMAT)(0x0A20) */
	wbuf[0] = MIP_R0_TEST;
	wbuf[1] = MIP_R1_TEST_DATA_FORMAT;
	if (mit300_reg_read(dev, wbuf, 2, rbuf, 6) < 0) {
		TOUCH_E("[ERROR] Read data format\n");
		goto ERROR;
	}
	row_num = rbuf[0];
	col_num = rbuf[1];
	buffer_col_num = rbuf[2];
	rotate = rbuf[3];
	key_num = rbuf[4];
	data_type = rbuf[5];

	data_type_sign = (data_type & 0x80) >> 7;
	data_type_size = data_type & 0x7F;

	TOUCH_I("%s - row_num[%d] col_num[%d] buffer_col_num[%d] rotate[%d] key_num[%d]\n", __func__, row_num, col_num, buffer_col_num, rotate, key_num);
	TOUCH_I("%s - data_type[0x%02X] data_sign[%d] data_size[%d]\n", __func__, data_type, data_type_sign, data_type_size);

	/* get buf addr(Read TEST CS TEST INFO)(0x0A00) */
	wbuf[0] = MIP_R0_TEST;
	wbuf[1] = MIP_R1_TEST_BUF_ADDR;
	if (mit300_reg_read(dev, wbuf, 2, rbuf, 2) < 0) {
		TOUCH_E("[ERROR] Read buf addr\n");
		goto ERROR;
	}

	buf_addr_l = rbuf[0];
	buf_addr_h = rbuf[1];
	TOUCH_I("%s - buf_addr[0x%02X 0x%02X]\n", __func__, buf_addr_h, buf_addr_l);

	/* read rmi buffer */
	if (mip_proc_table_data(dev, size, data_type_size, data_type_sign, buf_addr_h, buf_addr_l, row_num, col_num, buffer_col_num, rotate, key_num)) {
		TOUCH_E("[ERROR] mip_proc_table_data\n");
		goto ERROR;
	}

	/* set normal(sensing) mode */
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_MODE;
	wbuf[2] = MIP_CTRL_MODE_NORMAL;
	if (mit300_reg_write(dev, wbuf, 3)) {
		TOUCH_E("[ERROR] mip_i2c_write\n");
		goto ERROR;
	}
	/* wait ready status */
	wait_cnt = 100;
	while (wait_cnt--) {
		if (mip_get_ready_status(dev) == MIP_CTRL_STATUS_READY) {
			TOUCH_I("%s - wait 3_cnt = %d\n", __func__, wait_cnt);
			break;
		}
		touch_msleep(50);
	}

	if (wait_cnt <= 0) {
		TOUCH_E("[ERROR] Wait timeout\n");
		goto ERROR;
	}

	TOUCH_I("%s - set normal mode\n", __func__);

	/* set event interrupt type(0x0611) */
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_EVENT_TRIGGER_TYPE;
	wbuf[2] = MIP_TRIGGER_TYPE_INTR;
	if (mit300_reg_write(dev, wbuf, 3) < 0) {
		TOUCH_E("[ERROR] Write interrupt mode\n");
		goto ERROR;
	}

	TOUCH_I("%s - set interrupt mode\n", __func__);

	/* exit */
	test_busy = false;

	return 0;
ERROR:
	test_busy = false;

	TOUCH_E("[ERROR]\n");
	return 1;
}

ssize_t mit_get_test_result(struct device *dev, char *buf, int type)
{
	struct mit_data *d = to_mit_data(dev);
	char temp_buf[255] = {0,};
	int i = 0;
	int ret = 0;
	int fd = 0;
	char data_path[64] = {0,};
	char *read_buf = NULL;
	int retry_max = 3;
	int retry_count = 0;

	mm_segment_t old_fs = get_fs();

	for ( i = 0; i < d->dev.row_num; i++) {
		memset(d->mit_data[i], 0, sizeof(uint16_t) * d->dev.col_num);
	}

	read_buf = kzalloc(sizeof(u8) * 4096,GFP_KERNEL);
	if (read_buf == NULL) {
		TOUCH_E("read_buf mem_error\n");
		goto mem_error;
	}

	d->test_mode = type;


	if (d->test_mode == RAW_DATA_SHOW || d->test_mode == RAW_DATA_STORE ) {
		retry_count = 0;
		while (retry_count++ < retry_max) {
			if (get_rawdata(dev) == -1) {
				TOUCH_E("getting raw data failed\n");
				ret = snprintf(buf, PAGE_SIZE, "%s\n", "ERROR");
				TOUCH_E("retry (%d/%d)\n", retry_count, retry_max);
				mit_power_reset(dev);
				touch_msleep(100);
			} else break;
			if (retry_count >= retry_max) {
				TOUCH_E("all retry failed\n");
				goto error;
			}
		}
	} else {
		retry_count = 0;
		while (retry_count++ < retry_max) {
			if (mip_run_test(dev, type)) {
				TOUCH_E("[ERROR] mip_run_test = %d\n", type);
				ret = snprintf(buf, PAGE_SIZE, "%s\n", "ERROR");
				TOUCH_E("retry (%d/%d)\n", retry_count, retry_max);
				mit_power_reset(dev);
				touch_msleep(100);
			} else break;
			if (retry_count >= retry_max) {
				TOUCH_E("all retry failed\n");
				goto error;
			}
		}
	}


	switch (type) {
		case RAW_DATA_SHOW:
			if (d->use_excel) {
				print_excel_rawdata(dev, buf);
			}
			ret = print_rawdata(dev, buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print raw data\n");
				d->selfdiagnostic_state[SD_RAWDATA] = 0;
				goto error;
			}
			break;
		case RAW_DATA_STORE:
			snprintf(temp_buf, strlen(buf), "%s", buf);
			sprintf(data_path, "/sdcard/%s.csv", temp_buf);

			ret = print_rawdata(dev, read_buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print raw data\n");
				d->selfdiagnostic_state[SD_RAWDATA] = 0;
				goto error;
			}

			set_fs(KERNEL_DS);
			fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
			if (fd >= 0) {
				sys_write(fd, read_buf, 4096);
				sys_close(fd);
				TOUCH_I("%s saved \n", data_path);
			} else {
				TOUCH_E("%s open failed \n", data_path);
			}
			set_fs(old_fs);
			break;
		case OPENSHORT :
			if (d->check_openshort == 1)
				ret = print_openshort_data(dev, buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print open short data\n");
				d->selfdiagnostic_state[SD_OPENSHORT] = 0;
				goto error;
			}
			break;
		case MUXSHORT_SHOW :
			if (d->check_openshort == 1)
				ret = print_muxshort_data(dev, buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print mux short data\n");
				d->selfdiagnostic_state[SD_MUXSHORT] = 0;
				goto error;
			}
			break;
		case OPENSHORT_STORE :
			snprintf(temp_buf,strlen(buf),"%s", buf);
			sprintf(data_path,"/sdcard/%s_openshort.csv", temp_buf);
			if (d->check_openshort == 1)
				ret = print_openshort_data(dev, read_buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print open short data\n");
				d->selfdiagnostic_state[SD_OPENSHORT] = 0;
				goto error;
			}

			set_fs(KERNEL_DS);
			fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
			if (fd >= 0) {
				sys_write(fd, read_buf, 4096);
				sys_close(fd);
				TOUCH_I("%s saved \n", data_path);
			} else {
				TOUCH_E("%s open failed \n", data_path);
			}
			set_fs(old_fs);
			break;

		case CM_DELTA_SHOW:
			ret = print_cm_delta_data(dev, buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print open short data\n");
				d->selfdiagnostic_state[SD_CM_DELTA] = 0;
				goto error;
			}
			break;
		case CM_DELTA_STORE :
			snprintf(temp_buf,strlen(buf),"%s", buf);
			sprintf(data_path,"/sdcard/%s_cmdelta.csv", temp_buf);
			ret = print_cm_delta_data(dev, read_buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print open short data\n");
				d->selfdiagnostic_state[SD_CM_DELTA] = 0;
				goto error;
			}

			set_fs(KERNEL_DS);
			fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
			if (fd >= 0) {
				sys_write(fd, read_buf, 4096);
				sys_close(fd);
				TOUCH_I("%s saved \n", data_path);
			} else {
				TOUCH_E("%s open failed \n", data_path);
			}
			set_fs(old_fs);
			break;
		case CM_JITTER_SHOW:
			ret = print_cm_jitter_data(dev, buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print open short data\n");
				d->selfdiagnostic_state[SD_CM_JITTER] = 0;
				goto error;
			}
			break;
		case CM_JITTER_STORE :
			snprintf(temp_buf,strlen(buf),"%s", buf);
			sprintf(data_path,"/sdcard/%s_cmjitter.csv", temp_buf);
			ret = print_cm_jitter_data(dev, read_buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print open short data\n");
				d->selfdiagnostic_state[SD_CM_JITTER] = 0;
				goto error;
			}

			set_fs(KERNEL_DS);
			fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
			if (fd >= 0) {
				sys_write(fd, read_buf, 4096);
				sys_close(fd);
				TOUCH_I("%s saved \n", data_path);
			} else {
				TOUCH_E("%s open failed \n", data_path);
			}
			set_fs(old_fs);
			break;
		case LPWG_JITTER_SHOW :
			ret = print_lpwg_jitter_data(dev, buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print lpwg jitter data\n");
				d->lpwg_selfdiagnostic_state[SD_LPWG_JITTER] = 0;
				goto error;
			}
			break;
		case LPWG_ABS_SHOW :
			ret = print_lpwg_abs_data(dev, buf, type);
			if (ret < 0) {
				TOUCH_E("fail to print lpwg abs data\n");
				d->lpwg_selfdiagnostic_state[SD_LPWG_ABS] = 0;
				goto error;
			}
			break;

		default :
			TOUCH_I("type = default[%d]\n", type);
			break;
		}

	if (read_buf != NULL)
		kfree(read_buf);

	return ret;

error :
	if (read_buf != NULL)
		kfree(read_buf);

	return -1;

mem_error :
	if (read_buf != NULL)
		kfree(read_buf);
	return -1;


}

static ssize_t mit_rawdata_show(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;
	TOUCH_TRACE();

	d->selfdiagnostic_state[SD_RAWDATA] = 1;	/* rawdata */

	ret = mit_get_test_result(dev, buf, RAW_DATA_SHOW);
	if (ret < 0) {
		memset(buf, 0, PAGE_SIZE);
		ret = snprintf(buf, PAGE_SIZE, "failed to get raw data\n");
	}

	return ret;
}
static ssize_t mit_rawdata_store(struct device *dev, const char *buf)
{
	int ret = 0;
	char temp_buf[255] = {0};
	TOUCH_TRACE();
	if (strlen(buf) < 254) {
		strlcpy(temp_buf,buf,sizeof(temp_buf));
	} else {
		TOUCH_E("buf size is more than 255\n");
	}
	ret = mit_get_test_result(dev, temp_buf, RAW_DATA_STORE);

	return ret;
}

static ssize_t mit_cm_delta_show(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;
	d->selfdiagnostic_state[SD_CM_DELTA] = 1;	/* rawdata */

	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);

	ret = mit_get_test_result(dev, buf, CM_DELTA_SHOW);
	if (ret < 0) {
		memset(buf, 0, PAGE_SIZE);
		ret = snprintf(buf, PAGE_SIZE, "failed to get raw data\n");
	}
	return ret;
}

static ssize_t mit_cm_delta_store(struct device *dev, const char *buf)
{
	int ret = 0;
	char temp_buf[255] = {0};
	TOUCH_TRACE();
	if (strlen(buf) < 254) {
		strlcpy(temp_buf,buf,sizeof(temp_buf));
	} else {
		TOUCH_E("buf size is more than 255\n");
	}
	ret = mit_get_test_result(dev, temp_buf, CM_DELTA_STORE);

	return ret;
}

static ssize_t mit_cm_jitter_show(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;
	d->selfdiagnostic_state[SD_CM_JITTER] = 1;	/* rawdata */

	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);

	ret = mit_get_test_result(dev, buf, CM_JITTER_SHOW);
	if (ret < 0) {
		memset(buf, 0, PAGE_SIZE);
		ret = snprintf(buf, PAGE_SIZE, "failed to get raw data\n");
	}
	return ret;
}

static ssize_t mit_cm_jitter_store(struct device *dev, const char *buf)
{
	int ret = 0;
	char temp_buf[255] = {0};
	TOUCH_TRACE();
	if (strlen(buf) < 254) {
		strlcpy(temp_buf,buf,sizeof(temp_buf));
	} else {
		TOUCH_E("buf size is more than 255\n");
	}
	ret = mit_get_test_result(dev, temp_buf, CM_JITTER_STORE);

	return ret;
}

static ssize_t mit_chstatus_show(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;
	int len = 0;

	d->selfdiagnostic_state[SD_OPENSHORT] = 1;	/* openshort */

	TOUCH_TRACE();
	TOUCH_I("mit_chstatus_show\n");

	ret = mit_get_test_result(dev, buf, OPENSHORT);
	memset(buf, 0, PAGE_SIZE);
	if (ret < 0) {
		TOUCH_E("Failed to get OPEN SHORT Test result. \n");
		ret = snprintf(buf, PAGE_SIZE, "failed to OPEN SHORT data\n");
		goto error;
	}

	len = snprintf(buf, PAGE_SIZE - len, "Firmware Version : %X.%02X \n", d->module.version[0], d->module.version[1]);
	len += snprintf(buf + len, PAGE_SIZE - len, "FW Product : %s \n", d->module.product_code);
	len += snprintf(buf + len, PAGE_SIZE - len, "=======RESULT========\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "OPEN /  SHORT Test : %s\n", d->selfdiagnostic_state[SD_OPENSHORT]==1 ? "PASS" : "FAIL");

	return len;

error:
	return ret;
}

static ssize_t mit_chstatus_store(struct device *dev, const char *buf)
{
	int ret = 0;
	char temp_buf[255] = {0};
	TOUCH_TRACE();
	strlcpy(temp_buf,buf,strlen(temp_buf));

	ret = mit_get_test_result(dev, temp_buf, OPENSHORT_STORE);

	return ret;
}

static int mit_delta_show(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int i = 0;
	int ret = 0;

	TOUCH_TRACE();

	for ( i = 0 ; i < d->dev.row_num ; i++) {
		memset(d->intensity_data[i], 0, sizeof(uint16_t) * d->dev.col_num);
	}

	touch_interrupt_control(dev, INTERRUPT_DISABLE);

	if (get_intensity(dev) == -1) {
		TOUCH_E("intensity printf failed\n");
		goto error;
	}

	ret = print_intensity(dev, buf);

	if ( ret < 0) {
		TOUCH_E("fail to print intensity data\n");
		goto error;
	}

	touch_interrupt_control(dev, INTERRUPT_ENABLE);
	return ret;

error :
	touch_interrupt_control(dev, INTERRUPT_ENABLE);
	return -1;
}

static ssize_t mit_self_diagnostic_show(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int len = 0;
	int ret = 0;
	d->selfdiagnostic_state[SD_RAWDATA] = 1;	/* rawdata */
	d->selfdiagnostic_state[SD_OPENSHORT] = 1;	/* openshort */
	d->selfdiagnostic_state[SD_CM_DELTA] = 1;
	d->selfdiagnostic_state[SD_CM_JITTER] = 1;
	d->selfdiagnostic_state[SD_MUXSHORT] = 1;

	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test Start\n");

	firmware_version_log(dev);
	touch_msleep(30);

	d->selfdiagnostic_state[SD_OPENSHORT] = 1;	/* openshort */
	ret = mit_get_test_result(dev, buf, OPENSHORT);
	if (ret < 0) {
		TOUCH_E("failed to get open short data\n");
		memset(buf, 0, PAGE_SIZE);
		d->o_max = 0;
		d->o_min = 0;
		d->selfdiagnostic_state[SD_OPENSHORT] = 0;
		len = snprintf(buf, PAGE_SIZE, "failed to get open short data\n");
	}

	write_file(dev, buf, TIME_INFO_SKIP);
	touch_msleep(30);

	memset(buf, 0, PAGE_SIZE);
	d->selfdiagnostic_state[SD_MUXSHORT] = 1;	/* muxshort */
	ret = mit_get_test_result(dev, buf, MUXSHORT_SHOW);
	if (ret < 0) {
		TOUCH_E("failed to get mux short data\n");
		memset(buf, 0, PAGE_SIZE);
		d->m_max = 0;
		d->d_min = 0;
		d->selfdiagnostic_state[SD_MUXSHORT] = 0;
		len = snprintf(buf, PAGE_SIZE, "failed to get cm delta data\n");
	}

	write_file(dev, buf, TIME_INFO_SKIP);
	touch_msleep(30);

	memset(buf, 0, PAGE_SIZE);
	d->selfdiagnostic_state[SD_CM_DELTA] = 1;	/* CM_DELTA */
	ret = mit_get_test_result(dev, buf, CM_DELTA_SHOW);
	if (ret < 0) {
		TOUCH_E("failed to get cm delta data\n");
		memset(buf, 0, PAGE_SIZE);
		d->d_max = 0;
		d->d_min = 0;
		d->selfdiagnostic_state[SD_CM_DELTA] = 0;
		len = snprintf(buf, PAGE_SIZE, "failed to get cm delta data\n");
	}

	write_file(dev, buf, TIME_INFO_SKIP);
	touch_msleep(30);

	memset(buf, 0, PAGE_SIZE);
	d->selfdiagnostic_state[SD_CM_JITTER] = 1;	/* CM_JITTER */
	ret = mit_get_test_result(dev, buf, CM_JITTER_SHOW);
	if (ret < 0) {
		TOUCH_E("failed to get cm delta data\n");
		memset(buf, 0, PAGE_SIZE);
		d->j_max = 0;
		d->j_min = 0;
		d->selfdiagnostic_state[SD_CM_JITTER] = 0;
		len = snprintf(buf, PAGE_SIZE, "failed to get cm delta data\n");
	}

	write_file(dev, buf, TIME_INFO_SKIP);
	touch_msleep(30);

	memset(buf, 0, PAGE_SIZE);
	d->selfdiagnostic_state[SD_RAWDATA] = 1;	/* rawdata */
	ret = mit_get_test_result(dev, buf, RAW_DATA_SHOW);
	if (ret < 0) {
		TOUCH_E("failed to get raw data\n");
		memset(buf, 0, PAGE_SIZE);
		d->r_max = 0;
		d->r_min = 0;
		d->selfdiagnostic_state[SD_RAWDATA] = 0;
		len = snprintf(buf, PAGE_SIZE, "failed to get raw data\n");
	}

	write_file(dev, buf, TIME_INFO_SKIP);
	touch_msleep(30);

	TOUCH_I("Firmware Version : %X.%02X \n", d->module.version[0], d->module.version[1]);
	TOUCH_I("FW Product : %s \n", d->module.product_code);

	TOUCH_I("=========================\n");
	if (d->check_openshort) {
		TOUCH_I("OpenShort : %5d , %5d\n", d->o_max, d->o_min);
		TOUCH_I("MuxShort : %5d , %5d\n", d->m_max, d->m_min);
	}
	TOUCH_I("CmDelta   : %5d , %5d\n", d->d_max, d->d_min);
	TOUCH_I("CmJitter  : %5d , %5d\n", d->j_max, d->j_min);
	TOUCH_I("Rawdata   : %5d , %5d\n", d->r_max, d->r_min);
	TOUCH_I("=========================\n\n");
	TOUCH_I("=========RESULT==========\n");
	TOUCH_I("Channel Status : %s\n", (d->selfdiagnostic_state[SD_OPENSHORT]
				* d->selfdiagnostic_state[SD_CM_DELTA]
				* d->selfdiagnostic_state[SD_CM_JITTER]
				* d->selfdiagnostic_state[SD_MUXSHORT]) == 1 ? "PASS" : "FAIL");
	TOUCH_I("Raw Data : %s\n", d->selfdiagnostic_state[SD_RAWDATA] == 1 ? "PASS" : "FAIL");

	memset(buf, 0, PAGE_SIZE);
	len += snprintf(buf, PAGE_SIZE , "\n=========================\n");
	if (d->check_openshort) {
		len += snprintf(buf + len, PAGE_SIZE - len, "OpenShort : %5d , %5d\n", d->o_max, d->o_min);
		len += snprintf(buf + len, PAGE_SIZE - len, "MuxShort : %5d , %5d\n", d->m_max, d->m_min);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "CmDelta   : %5d , %5d\n", d->d_max, d->d_min);
	len += snprintf(buf + len, PAGE_SIZE - len, "CmJitter  : %5d , %5d\n", d->j_max, d->j_min);
	len += snprintf(buf + len, PAGE_SIZE - len, "Rawdata   : %5d , %5d\n", d->r_max, d->r_min);
	len += snprintf(buf + len, PAGE_SIZE - len, "=========================\n\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "=========RESULT==========\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "Channel Status : %s\n", ((d->selfdiagnostic_state[SD_OPENSHORT]
							* d->selfdiagnostic_state[SD_CM_DELTA]
							* d->selfdiagnostic_state[SD_CM_JITTER]
							* d->selfdiagnostic_state[SD_MUXSHORT]) == 1)? "PASS" : "FAIL");
	len += snprintf(buf + len, PAGE_SIZE - len, "Raw Data : %s\n", d->selfdiagnostic_state[SD_RAWDATA] == 1 ? "PASS" : "FAIL");
	write_file(dev, buf, TIME_INFO_SKIP);

	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);
	log_file_size_check(dev);
	TOUCH_I("Show_sd Test End\n");

	return len;
}


static ssize_t mit_lpwg_self_diagnostic_show(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int len = 0;
	int ret = 0;
	d->lpwg_selfdiagnostic_state[SD_LPWG_JITTER] = 1;	/* lpwg jitter */
	d->lpwg_selfdiagnostic_state[SD_LPWG_ABS] = 1;	/* lpwg abs */

	mip_lpwg_enable_sensing(dev, 1);
	touch_msleep(1000);

	tci_control(dev, LPWG_PANEL_DEBUG_CTRL, 1);
	touch_msleep(10);

	mip_lpwg_start(dev);
	touch_msleep(10);

	write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_lpwg_sd Test Start\n");

	firmware_version_log(dev);
	touch_msleep(30);

	d->lpwg_selfdiagnostic_state[SD_LPWG_JITTER] = 1;	/* lpwg jitter */
	ret = mit_get_test_result(dev, buf, LPWG_JITTER_SHOW);
	if (ret < 0) {
		TOUCH_E("failed to get lpwg jitter data\n");
		memset(buf, 0, PAGE_SIZE);
		d->l_j_max = 0;
		d->l_j_min = 0;
		d->lpwg_selfdiagnostic_state[SD_LPWG_JITTER] = 0;
		len = snprintf(buf, PAGE_SIZE, "failed to get lpwg jitter data\n");
	}

	write_file(dev, buf, TIME_INFO_SKIP);
	touch_msleep(30);

	memset(buf, 0, PAGE_SIZE);
	d->lpwg_selfdiagnostic_state[SD_LPWG_ABS] = 1;	/* lpwg abs */
	ret = mit_get_test_result(dev, buf, LPWG_ABS_SHOW);
	if (ret < 0) {
		TOUCH_E("failed to get lpwg abs data\n");
		memset(buf, 0, PAGE_SIZE);
		d->l_a_max = 0;
		d->l_a_min = 0;
		d->lpwg_selfdiagnostic_state[SD_LPWG_ABS] = 0;
		len = snprintf(buf, PAGE_SIZE, "failed to get lpwg abs data\n");
	}

	write_file(dev, buf, TIME_INFO_SKIP);
	touch_msleep(30);


	TOUCH_I("Firmware Version : %X.%02X \n", d->module.version[0], d->module.version[1]);
	TOUCH_I("FW Product : %s \n", d->module.product_code);

	TOUCH_I("=========================\n");
	TOUCH_I("LpwgJitter: %5d , %5d\n", d->l_j_max, d->l_j_min);
	TOUCH_I("LpwgAbs   : %5d , %5d\n", d->l_a_max, d->l_a_min);
	TOUCH_I("=========================\n\n");
	TOUCH_I("=========RESULT==========\n");
	TOUCH_I("LPWG RawData : %s\n", (d->lpwg_selfdiagnostic_state[SD_LPWG_JITTER]
				* d->lpwg_selfdiagnostic_state[SD_LPWG_ABS]) == 1 ? "PASS" : "FAIL");

	memset(buf, 0, PAGE_SIZE);
	len += snprintf(buf, PAGE_SIZE , "\n=========================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "LpwgJitter: %5d , %5d\n", d->l_j_max, d->l_j_min);
	len += snprintf(buf + len, PAGE_SIZE - len, "LpwgAbs   : %5d , %5d\n", d->l_a_max, d->l_a_min);
	len += snprintf(buf + len, PAGE_SIZE - len, "=========================\n\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "=========RESULT==========\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "LPWG RawData : %s\n", (d->lpwg_selfdiagnostic_state[SD_LPWG_JITTER]
							* d->lpwg_selfdiagnostic_state[SD_LPWG_ABS]) == 1? "PASS" : "FAIL");
	write_file(dev, buf, TIME_INFO_SKIP);

	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
    log_file_size_check(dev);
	TOUCH_I("Show_lpwg_sd Test End\n");

	tci_control(dev, LPWG_PANEL_DEBUG_CTRL, 0);
	touch_msleep(10);

	mip_lpwg_start(dev);

	return len;
}

int mit_sysfs(struct device *dev, char *buf1, const char *buf2, u32 code)
{
	int ret = 0;

	TOUCH_TRACE();

	switch (code) {
		case SYSFS_CHSTATUS_SHOW:
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			ret = mit_chstatus_show(dev, buf1);
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			break;
		case SYSFS_CHSTATUS_STORE:
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			ret = mit_chstatus_store(dev, buf2);
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			break;
		case SYSFS_RAWDATA_SHOW:
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			ret = mit_rawdata_show(dev, buf1);
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			break;
		case SYSFS_RAWDATA_STORE:
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			ret = mit_rawdata_store(dev, buf2);
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			break;
		case SYSFS_CMDELTA_SHOW:
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			ret = mit_cm_delta_show(dev, buf1);
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			break;
		case SYSFS_CMDELTA_STORE:
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			ret = mit_cm_delta_store(dev, buf2);
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			break;
		case SYSFS_CMJITTER_SHOW:
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			ret = mit_cm_jitter_show(dev, buf1);
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			break;
		case SYSFS_CMJITTER_STORE:
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			ret = mit_cm_jitter_store(dev, buf2);
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			break;
		case SYSFS_DELTA_SHOW:
			ret = mit_delta_show(dev, buf1);
			break;
		case SYSFS_SELF_DIAGNOSTIC_SHOW:
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			ret = mit_self_diagnostic_show(dev, buf1);
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			break;
	}
	return ret;
}


static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;

	if(d->lpwg_debug_enable == 0 && !ts->lpwg.screen) {
		ret = snprintf(buf,PAGE_SIZE,"If you want to check delta, please turn on the LCD.\n");
		TOUCH_I("If you want to check delta, please turn on the LCD.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, buf, 0, SYSFS_DELTA_SHOW);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;

	if(d->lpwg_debug_enable == 0 && !ts->lpwg.screen) {
		ret = snprintf(buf,PAGE_SIZE,"If you want to check raw data, please turn on the LCD.\n");
		TOUCH_I("If you want to check raw data, please turn on the LCD.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, buf, 0, SYSFS_RAWDATA_SHOW);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t store_rawdata(struct device *dev,const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	char cmd[NAME_BUFFER_SIZE] = {0};
	if( count > NAME_BUFFER_SIZE )
	{
		return count;
	}

	if(!ts->lpwg.screen) {
		TOUCH_I("If you want to check delta, please turn on the LCD.\n");
		return count;
	}

	if (sscanf(buf, "%127s", cmd) != 1)
			return -EINVAL;

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, 0, buf, SYSFS_RAWDATA_STORE);
	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t show_openshort(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	int value = 0;
	int ret = 0;

	if(!ts->lpwg.screen) {
		ret = snprintf(buf,PAGE_SIZE,"If you want to check open short, please turn on the LCD.\n");
		TOUCH_I("If you want to check open short, please turn on the LCD.\n");
		return ret;
	}

	value = d->check_openshort;
	d->check_openshort = 1;

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, buf, 0, SYSFS_CHSTATUS_SHOW);
	mutex_unlock(&ts->lock);

	d->check_openshort = value;
	return ret;
}

static ssize_t store_openshort(struct device *dev,const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;
	int value = 0;
	char cmd[NAME_BUFFER_SIZE] = {0};

	if( count > NAME_BUFFER_SIZE )
	{
		return count;
	}

	if(!ts->lpwg.screen) {
		TOUCH_I("If you want to check open short, please turn on the LCD.\n");
		return count;
	}

	if (sscanf(buf, "%127s", cmd) != 1)
			return -EINVAL;

	value = d->check_openshort;
	d->check_openshort = 1;

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, 0, buf, SYSFS_CHSTATUS_STORE);
	mutex_unlock(&ts->lock);

	d->check_openshort = value;

	return count;
}

static ssize_t show_cmdelta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if(!ts->lpwg.screen) {
		ret = snprintf(buf,PAGE_SIZE,"If you want to check cm delta, please turn on the LCD.\n");
		TOUCH_I("If you want to check open cd delta, please turn on the LCD.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, buf, 0, SYSFS_CMDELTA_SHOW);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t store_cmdelta(struct device *dev,const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	char cmd[NAME_BUFFER_SIZE] = {0};

	if( count > NAME_BUFFER_SIZE )
	{
		return count;
	}

	if(!ts->lpwg.screen) {
		TOUCH_I("If you want to check open cd delta, please turn on the LCD.\n");
		return count;
	}

	if (sscanf(buf, "%127s", cmd) != 1)
			return -EINVAL;

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, 0, buf, SYSFS_CMDELTA_STORE);
	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t show_cmjitter(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if(!ts->lpwg.screen) {
		ret = snprintf(buf,PAGE_SIZE,"If you want to check cm jitter, please turn on the LCD.\n");
		TOUCH_I("If you want to check open cd jitter, please turn on the LCD.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, buf, 0, SYSFS_CMJITTER_SHOW);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t store_cmjitter(struct device *dev,const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	char cmd[NAME_BUFFER_SIZE] = {0};

	if( count > NAME_BUFFER_SIZE )
	{
		return count;
	}

	if(!ts->lpwg.screen) {
		TOUCH_I("If you want to check open cd jitter, please turn on the LCD.\n");
		return count;
	}

	if (sscanf(buf, "%127s", cmd) != 1)
			return -EINVAL;

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, 0, buf, SYSFS_CMJITTER_STORE);
	mutex_unlock(&ts->lock);
	return count;
}

static ssize_t show_chstatus(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if(!ts->lpwg.screen) {
		ret = snprintf(buf,PAGE_SIZE,"If you want to check chstatus, please turn on the LCD.\n");
		TOUCH_I("If you want to check open chstatus, please turn on the LCD.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, buf, 0, SYSFS_CHSTATUS_SHOW);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret;

	if(!ts->lpwg.screen) {
		ret = snprintf(buf,PAGE_SIZE,"If you want to check self diagnostic, please turn on the LCD.\n");
		TOUCH_I("If you want to check open self diagnostic, please turn on the LCD.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	ret = mit_sysfs(dev, buf, 0, SYSFS_SELF_DIAGNOSTIC_SHOW);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret;

	mutex_lock(&ts->lock);
	touch_interrupt_control(dev, INTERRUPT_DISABLE);

	ret = mit_lpwg_self_diagnostic_show(dev, buf);

	touch_interrupt_control(dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret;
}
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, store_rawdata);
static TOUCH_ATTR(openshort, show_openshort, store_openshort);
static TOUCH_ATTR(cmdelta, show_cmdelta, store_cmdelta);
static TOUCH_ATTR(cmjitter, show_cmjitter, store_cmjitter);
static TOUCH_ATTR(chstatus, show_chstatus, NULL);
static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);

static struct attribute *prd_attribute_list[] = {
	&touch_attr_delta.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_openshort.attr,
	&touch_attr_cmdelta.attr,
	&touch_attr_cmjitter.attr,
	&touch_attr_chstatus.attr,
	&touch_attr_sd.attr,
	&touch_attr_lpwg_sd.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

int mit300_prd_register_sysfs(struct device *dev)
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

