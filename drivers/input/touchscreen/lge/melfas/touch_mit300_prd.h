/* lge_touch_melfas.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: sangyeol.ryu@lge.com
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

/*
 * Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_mit300.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

#define MIP_R0_TEST				0x0A
#define MIP_R1_TEST_BUF_ADDR			0x00
#define MIP_R1_TEST_PROTOCOL			0x02
#define MIP_R1_TEST_TYPE			0x10
#define MIP_R1_TEST_DATA_FORMAT			0x20
#define MIP_R1_TEST_ROW_NUM			0x20
#define MIP_R1_TEST_COL_NUM			0x21
#define MIP_R1_TEST_BUFFER_COL_NUM		0x22
#define MIP_R1_TEST_COL_AXIS			0x23
#define MIP_R1_TEST_KEY_NUM			0x24
#define MIP_R1_TEST_DATA_TYPE			0x25

#define MIP_R0_IMAGE				0x0C
#define MIP_R1_IMAGE_BUF_ADDR			0x00
#define MIP_R1_IMAGE_PROTOCOL_ID		0x04
#define MIP_R1_IMAGE_TYPE			0x10
#define MIP_R1_IMAGE_DATA_FORMAT		0x20
#define MIP_R1_IMAGE_ROW_NUM			0x20
#define MIP_R1_IMAGE_COL_NUM			0x21
#define MIP_R1_IMAGE_BUFFER_COL_NUM		0x22
#define MIP_R1_IMAGE_COL_AXIS			0x23
#define MIP_R1_IMAGE_KEY_NUM			0x24
#define MIP_R1_IMAGE_DATA_TYPE			0x25
#define MIP_R1_IMAGE_FINGER_NUM			0x30
#define MIP_R1_IMAGE_FINGER_AREA		0x31

#define MIP_CTRL_STATUS_NONE			0x05
#define MIP_CTRL_STATUS_READY			0xA0
#define MIP_CTRL_STATUS_LOG			0x77

#define MIP_CTRL_MODE_NORMAL			0
#define MIP_CTRL_MODE_PARAM			1
#define MIP_CTRL_MODE_TEST_CM			2

#define MIP_TEST_TYPE_NONE			0
#define MIP_TEST_TYPE_CM_DELTA			1
#define MIP_TEST_TYPE_CM_ABS			2
#define MIP_TEST_TYPE_CM_JITTER			3
#define MIP_TEST_TYPE_SHORT			4
#define MIP_TEST_TYPE_INTR_H			5
#define MIP_TEST_TYPE_INTR_L			6
#define MIP_TEST_TYPE_SHORT2			7

#define MIP_IMG_TYPE_NONE			0
#define MIP_IMG_TYPE_INTENSITY			1
#define MIP_IMG_TYPE_RAWDATA			2
#define MIP_IMG_TYPE_WAIT			255

#define MIP_TRIGGER_TYPE_NONE			0
#define MIP_TRIGGER_TYPE_INTR			1
#define MIP_TRIGGER_TYPE_REG			2

#define LOG_BUF_SIZE		256
#define BUF_SIZE (PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

enum {
	RAW_DATA_SHOW	= 0,
	RAW_DATA_STORE,
	OPENSHORT,
	OPENSHORT_STORE,
	CM_DELTA_SHOW,
	CM_DELTA_STORE,
	CM_JITTER_SHOW,
	CM_JITTER_STORE,
	MUXSHORT_SHOW,
	LPWG_JITTER_SHOW,
	LPWG_ABS_SHOW,
};

enum {
	SD_RAWDATA = 0,
	SD_OPENSHORT,
	SD_CM_DELTA,
	SD_CM_JITTER,
	SD_MUXSHORT,
};

enum {
	SD_LPWG_JITTER = 0,
	SD_LPWG_ABS,
};

enum {
	SYSFS_REG_CONTROL_STORE,
	SYSFS_LPWG_TCI_STORE,
	SYSFS_CHSTATUS_SHOW,
	SYSFS_CHSTATUS_STORE,
	SYSFS_OPENSHORT_SHOW,
	SYSFS_RAWDATA_SHOW,
	SYSFS_RAWDATA_STORE,
	SYSFS_CMDELTA_SHOW,
	SYSFS_CMDELTA_STORE,
	SYSFS_CMJITTER_SHOW,
	SYSFS_CMJITTER_STORE,
	SYSFS_DELTA_SHOW,
	SYSFS_SELF_DIAGNOSTIC_SHOW,
	SYSFS_VERSION_SHOW,
	SYSFS_FIRMWARE_SHOW,
	SYSFS_TESTMODE_VERSION_SHOW,
	SYSFS_SENSING_ALL_BLOCK_CONTROL,
	SYSFS_SENSING_BLOCK_CONTROL,
	SYSFS_FW_DUMP,
	SYSFS_LPWG_SHOW,
	SYSFS_LPWG_STORE,
	SYSFS_KEYGUARD_STORE,
	SYSFS_LPWG_DEBUG_STORE,
	SYSFS_LPWG_REASON_STORE,
	SYSFS_LPWG_LCD_STATUS_SHOW,
};

enum {
	UNUSE_STORE_EXCEL = 0,
	USE_STORE_EXCEL,
};

extern void touch_msleep(unsigned int msecs);
int mit300_prd_register_sysfs(struct device *dev);

#endif

