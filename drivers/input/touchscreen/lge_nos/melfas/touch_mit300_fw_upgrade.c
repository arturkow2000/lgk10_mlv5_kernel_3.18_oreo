/* touch_mit300_fw_upgrade.c
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
#include <touch_core.h>
#include <touch_hwif.h>

#include "touch_mit300.h"

/* ISC Info */
#define ISC_PAGE_SIZE				128

/* ISC Command */
#define ISC_CMD_ENTER				{0xFB,0x4A,0x00,0x65,0x00,0x00,0x00,0x00}
#define ISC_CMD_READ_STATUS			{0xFB,0x4A,0x36,0xC2,0x00,0x00,0x00,0x00}
#define ISC_CMD_ERASE_PAGE			{0xFB,0x4A,0x00,0x8F,0x00,0x00,0x00,0x00}
#define ISC_CMD_PROGRAM_PAGE		{0xFB,0x4A,0x00,0x54,0x00,0x00,0x00,0x00}
#define ISC_CMD_READ_PAGE			{0xFB,0x4A,0x00,0xC2,0x00,0x00,0x00,0x00}
#define ISC_CMD_EXIT				{0xFB,0x4A,0x00,0x66,0x00,0x00,0x00,0x00}

/* ISC Status */
#define ISC_STATUS_BUSY				0x96
#define ISC_STATUS_DONE				0xAD

#define MAX_ITERATOR				30000

#define MIP_BIN_TAIL_MARK			{0x4D, 0x42, 0x54, 0x01}	/* M B T 0x01 */
#define MIP_BIN_TAIL_SIZE			64

/* Firmware update */
#define MIP_FW_UPDATE_DEBUG			0	/* 0 (defualt) or 1 */
#define CHIP_NAME					"MIT300"
#define CHIP_FW_CODE				"T3H0"

int is_probed = 0;

/**
* Firmware binary tail info
*/
struct mip_bin_tail {
	u8 tail_mark[4];
	char chip_name[4];
	u32 bin_start_addr;
	u32 bin_length;

	u16 ver_boot;
	u16 ver_core;
	u16 ver_app;
	u16 ver_param;
	u8 boot_start;
	u8 boot_end;
	u8 core_start;
	u8 core_end;
	u8 app_start;
	u8 app_end;
	u8 param_start;
	u8 param_end;

	u8 checksum_type;
	u8 hw_category;
	u16 param_id;
	u32 param_length;
	u32 build_date;
	u32 build_time;

	u32 reserved1;
	u32 reserved2;
	u16 reserved3;
	u16 tail_size;
	u32 crc;
} __attribute__ ((packed));

/**
* Read ISC status
*/
static int mip_isc_read_status(struct device *dev)
{
	u8 cmd[8] =  ISC_CMD_READ_STATUS;
	u8 result = 0;
	int cnt = 100;
	int ret = 0;

	TOUCH_TRACE();

	do {
		if (mit300_reg_read(dev, cmd, 8, &result, 1)) {
			TOUCH_E("[ERROR] i2c_transfer\n");
			return -1;
		}

		if (result == ISC_STATUS_DONE) {
			ret = 0;
			break;
		} else if (result == ISC_STATUS_BUSY) {
			ret = -1;
			//touch_msleep(1);
		} else {
			TOUCH_E("wrong value [0x%02X]\n", result);
			ret = -1;
			touch_msleep(1);
		}
	} while (--cnt);

	if (!cnt) {
		TOUCH_E("[ERROR] count overflow - cnt [%d] status [0x%02X]\n", cnt, result);
		goto ERROR;
	}

	return ret;

ERROR:
	return ret;
}

/**
* Command : Erase Page
*/
static int mip_isc_erase_page(struct device *dev, int offset)
{
	u8 write_buf[8] = ISC_CMD_ERASE_PAGE;

	TOUCH_TRACE();

	write_buf[4] = (u8)((offset >> 24) & 0xFF);
	write_buf[5] = (u8)((offset >> 16) & 0xFF);
	write_buf[6] = (u8)((offset >> 8) & 0xFF);
	write_buf[7] = (u8)(offset & 0xFF);

	if (mit300_reg_write(dev, write_buf, 8)) {
		TOUCH_E("[ERROR] i2c_transfer\n");
		goto ERROR;
	}

	if (mip_isc_read_status(dev) != 0) {
		goto ERROR;
	}

	return 0;

ERROR:
	return -1;
}

/**
* Command : Read Page
*/
int mip_isc_read_page(struct device *dev, int offset, u8 *data)
{
	u8 write_buf[8] = ISC_CMD_READ_PAGE;

	TOUCH_TRACE();

	write_buf[4] = (u8)((offset >> 24) & 0xFF);
	write_buf[5] = (u8)((offset >> 16) & 0xFF);
	write_buf[6] = (u8)((offset >> 8) & 0xFF);
	write_buf[7] = (u8)(offset & 0xFF);

	if (mit300_reg_read(dev, write_buf, 8, data, ISC_PAGE_SIZE)) {
		TOUCH_E("[ERROR] i2c_transfer\n");
		goto ERROR;
	}

	return 0;

ERROR:
	return -1;
}

/**
* Command : Program Page
*/
static int mip_isc_program_page(struct device *dev, int offset, const u8 *data, int length)
{
	u8 write_buf[8 + ISC_PAGE_SIZE] = ISC_CMD_PROGRAM_PAGE;

	TOUCH_TRACE();

	if (length > ISC_PAGE_SIZE) {
		TOUCH_E("[ERROR] page length overflow\n");
		goto ERROR;
	}

	write_buf[4] = (u8)((offset >> 24) & 0xFF);
	write_buf[5] = (u8)((offset >> 16) & 0xFF);
	write_buf[6] = (u8)((offset >> 8) & 0xFF);
	write_buf[7] = (u8)(offset & 0xFF);

	memcpy(&write_buf[8], data, length);

	if (mit300_reg_write(dev, write_buf, (length + 8))) {
		TOUCH_E("[ERROR] i2c_master_send\n");
		goto ERROR;
	}

	if (mip_isc_read_status(dev) != 0) {
		goto ERROR;
	}

	return 0;

ERROR:
	return -1;
}

/**
* Command : Enter ISC
*/
static int mip_isc_enter(struct device *dev)
{
	u8 write_buf[8] = ISC_CMD_ENTER;

	TOUCH_TRACE();

	if (mit300_reg_write(dev, write_buf, 8)) {
		TOUCH_E("[ERROR] i2c_master_send\n");
		goto ERROR;
	}

	if (mip_isc_read_status(dev) != 0) {
		goto ERROR;
	}

	return 0;

ERROR:
	return -1;
}

/**
* Command : Exit ISC
*/
int mip_isc_exit(struct device *dev)
{
	u8 write_buf[8] = ISC_CMD_EXIT;

	TOUCH_TRACE();

	if (mit300_reg_write(dev, write_buf, 8)) {
		TOUCH_E("[ERROR] i2c_master_send\n");
		goto ERROR;
	}

	return 0;

ERROR:
	return -1;
}

/**
* Read chip firmware version
*/
int mip_get_fw_version(struct device *dev, u8 *ver_buf)
{
	u8 rbuf[2] = {0};
	u8 wbuf[2];

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_CUSTOM;
	if (mit300_reg_read(dev, wbuf, 2, rbuf, 2)) {
		TOUCH_E("[ERROR]\n");
		memset(ver_buf, 0xFF, (int) sizeof(ver_buf));
		return 1;
	};

	ver_buf[0] = rbuf[1];
	ver_buf[1] = rbuf[0];

	return 0;
}

/**
* Read chip firmware version for u16
*/
int mip_get_fw_version_u16(struct device *dev, u16 *ver_buf_u16)
{
	u8 rbuf[2];

	if (mip_get_fw_version(dev, rbuf)) {
		goto ERROR;
	}

	ver_buf_u16[0] = (rbuf[0] << 8) | rbuf[1];

	return 0;

ERROR:
	memset(ver_buf_u16, 0xFFFF, (int) sizeof(ver_buf_u16));

	TOUCH_E("[ERROR]\n");
	return 1;
}

/**
* Reset gpio pin
*/
int mit_power_reset(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

    TOUCH_I("Power Reset \n");

    touch_gpio_direction_output(ts->reset_pin, 0);
    TOUCH_I("power: reset_pin low\n");
    touch_msleep(2);

    touch_gpio_direction_output(ts->reset_pin, 1);
    TOUCH_I("power: reset_pin high\n");
    touch_msleep(50);

	return 0;
}

/**
* Reboot chip
*
* Caution : IRQ must be disabled before mip_reboot and enabled after mip_reboot.
*/
void mip_reboot(struct device *dev)
{
	TOUCH_I("%s : [START]\n", __func__);

	mit_power_reset(dev);

	TOUCH_I("%s : [DONE]\n", __func__);
}

int mit300_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	struct mip_bin_tail *bin_info = NULL;
	u8 dev_major;
	u8 dev_minor;
	u16 tail_size = 0;
	u8 bin_major;
	u8 bin_minor;
	int update = 0;

	TOUCH_I("%s [START]\n", __func__);

	dev_major = d->module.version[0];
	dev_minor = d->module.version[1];

	/* Check tail size */
	tail_size = (fw->data[fw->size - 5] << 8) | fw->data[fw->size - 6];

	/* Read bin info */
    bin_info = (struct mip_bin_tail *)&fw->data[fw->size - tail_size];

    TOUCH_I("%s - is_probed [%d]\n", __func__, is_probed);
    if(!is_probed){
        d->module.bin_version[0] = (bin_info->ver_app >> 8) & 0xFF;
        d->module.bin_version[1] =  bin_info->ver_app;
        strncpy(d->module.bin_chip_name, bin_info->chip_name, strlen(bin_info->chip_name));
        is_probed = 1;
    }

    bin_major = d->module.bin_version[0];
    bin_minor = d->module.bin_version[1];


	if (ts->force_fwup) {
		update = 1;
	} else if (bin_major && dev_major) {
		if (bin_minor != dev_minor)
			update = 1;
	} else if (bin_major ^ dev_major) {
		update = 1;
	} else if (!bin_major && !dev_major) {
		if (bin_minor != dev_minor)
			update = 1;
	}
	TOUCH_I(
		"bin-ver: %x.%02x , dev-ver: %x.%02x -> update: %d, force_fwup: %d\n",
		bin_major, bin_minor, dev_major, dev_minor,
		update, ts->force_fwup);

	TOUCH_I("%s [END]\n", __func__);
	return update;
}

/**
* Flash chip firmware (main function)
*/
int mip_flash_fw(struct device *dev, const u8 *fw_data, size_t fw_size, bool force, bool section)
{
	struct mip_bin_tail *bin_info = NULL;
	int ret = 0;
	int offset = 0;
	int offset_start = 0;
	int bin_size = 0;
	u8 *bin_data;
	u16 tail_size = 0;
	u8 tail_mark[4] = MIP_BIN_TAIL_MARK;
	u16 ver_chip;

	TOUCH_I("%s [START]\n", __func__);

	/* Check tail size */
	tail_size = (fw_data[fw_size - 5] << 8) | fw_data[fw_size - 6];
	if (tail_size != MIP_BIN_TAIL_SIZE) {
		TOUCH_E("[ERROR] wrong tail size [%d]\n", tail_size);
		ret = fw_err_file_type;
		goto ERROR;
	}

	/* Check bin format */
	if (memcmp(&fw_data[fw_size - tail_size], tail_mark, 4)) {
		TOUCH_E("[ERROR] wrong tail mark\n");
		ret = fw_err_file_type;
		goto ERROR;
	}

	/* Read bin info */
	bin_info = (struct mip_bin_tail *)&fw_data[fw_size - tail_size];

	TOUCH_D(FW_UPGRADE, "%s - bin_info : bin_len[%d] hw_cat[0x%2X] date[%4X] time[%4X] tail_size[%d]\n", __func__, bin_info->bin_length, bin_info->hw_category, bin_info->build_date, bin_info->build_time, bin_info->tail_size);

#if MIP_FW_UPDATE_DEBUG
	print_hex_dump(KERN_ERR, MIP_DEVICE_NAME " Bin Info : ", DUMP_PREFIX_OFFSET, 16, 1, bin_info, tail_size, false);
#endif

	/* Check chip code */
	if (memcmp(bin_info->chip_name, CHIP_FW_CODE, 4)) {
		TOUCH_E("[ERROR] F/W file is not for %s\n", CHIP_NAME);
		ret = fw_err_file_type;
		goto ERROR;
	}

	/* Read bin data */
	bin_size = bin_info->bin_length;
	bin_data = kzalloc(sizeof(u8) * (bin_size), GFP_KERNEL);
	memcpy(bin_data, fw_data, bin_size);

	/* Enter ISC mode */
	TOUCH_D(FW_UPGRADE, "Enter ISC mode\n");
	ret = mip_isc_enter(dev);
	if (ret != 0) {
		TOUCH_E("[ERROR] mip_isc_enter\n");
		ret = fw_err_download;
		goto ERROR;
	}

	/* Erase first page */
	offset = 0;
	TOUCH_D(FW_UPGRADE, "%s - Erase first page : Offset[0x%04X]\n", __func__, offset);
	ret = mip_isc_erase_page(dev, offset);
	if (ret != 0) {
		TOUCH_E("[ERROR] mip_isc_erase_page\n");
		ret = fw_err_download;
		goto ERROR;
	}

	/* Program & Verify */
	TOUCH_D(FW_UPGRADE, "%s - Program & Verify\n", __func__);

	offset_start = 0;
	offset = bin_size - ISC_PAGE_SIZE;
	while (offset >= offset_start) {
		/* Program */
		if (mip_isc_program_page(dev, offset, &bin_data[offset], ISC_PAGE_SIZE)) {
			TOUCH_E("[ERROR] mip_isc_program_page : offset[0x%04X]\n", offset);
			ret = fw_err_download;
			goto ERROR;
		}
		TOUCH_D(FW_UPGRADE, "%s - mip_isc_program_page : offset[0x%04X]\n", __func__, offset);

		offset -= ISC_PAGE_SIZE;
	}

	/* Exit ISC mode */
	TOUCH_D(FW_UPGRADE, "%s - Exit\n", __func__);
	mip_isc_exit(dev);

	/* Reset chip */
	mip_reboot(dev);

	touch_msleep(100);

	/* Check chip firmware version */
	if (mip_get_fw_version_u16(dev, &ver_chip)) {
		TOUCH_E("[ERROR] Unknown chip firmware version\n");
		ret = fw_err_download;
		goto ERROR;
	} else {
		if ((ver_chip == bin_info->ver_app)) {
			TOUCH_I("%s - Version check OK\n", __func__);
		} else {
			TOUCH_E("[ERROR] Version mismatch after flash. Chip[0x%04X] File[0x%04X]\n", ver_chip, bin_info->ver_app);
			ret = fw_err_download;
			goto ERROR;
		}
	}

	goto EXIT;

ERROR:
	/* Reset chip */
	mip_reboot(dev);

	TOUCH_E("[ERROR]\n");

EXIT:
	TOUCH_I("%s [DONE]\n", __func__);

	return ret;
}


int mit_isc_fwupdate(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int retires = 3;
	int ret;

	do {
		ret = mip_flash_fw(dev, fw->data, fw->size, ts->force_fwup, true);
		if (ret >= fw_err_none) {
			break;
		}
	} while (--retires);

	if (!retires) {
		TOUCH_E("%s [ERROR] mip_flash_fw failed\n", __func__);
		ret = -1;
	}

	if (ret < 0) {
		goto ERROR;
	}

	TOUCH_D(FW_UPGRADE, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	TOUCH_E("%s [ERROR]\n", __func__);
	return -1;
}
