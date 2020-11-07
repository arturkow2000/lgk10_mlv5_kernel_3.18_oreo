/* touch_sharp_lr388k6.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: keunyoung1.park@lge.com
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

#ifndef LGE_TOUCH_SHARP_H
#define LGE_TOUCH_SHARP_H

#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/firmware.h>
#include <linux/input/lge_touch_notify.h>

#define MAX_COMMAND_RESULT_LEN	(64 - 8)

/* LPWG Address */
#define REPORT_RATE_CTRL_REG		0x08
#define SENSITIVITY_CTRL_REG		0x0A

#define ACTIVE_AREA_X1_CTRL_REG		0x0C
#define ACTIVE_AREA_Y1_CTRL_REG		0x0E
#define ACTIVE_AREA_X2_CTRL_REG		0x10
#define ACTIVE_AREA_Y2_CTRL_REG		0x12

#define TOUCH_SLOP_CTRL_REG		0x14
#define TAP_DISTANCE_CTRL_REG		0x16
#define MIN_INTERTAP_CTRL_REG		0x18
#define MAX_INTERTAP_CTRL_REG		0x1A
#define TAP_COUNT_CTRL_REG		0x1C
#define INTERRUPT_DELAY_CTRL_REG	0x1E

#define TOUCH_SLOP_CTRL2_REG		0x20
#define TAP_DISTANCE_CTRL2_REG		0x22
#define MIN_INTERTAP_CTRL2_REG		0x24
#define MAX_INTERTAP_CTRL2_REG		0x26
#define TAP_COUNT_CTRL2_REG		0x28
#define INTERRUPT_DELAY_CTRL2_REG	0x2A

#define FAILURE_REASON_REG		0x30
#define FAILURE_INT_ENABLE_REG		0x31
#define FAILURE_INT_ENABLE2_REG		0x33
#define FAILURE_INT_STATUS_REG		0x35
#define FAILURE_INT_STATUS2_REG		0x37

/* INT STATUS */
#define SHTSC_STATUS_TOUCH_READY	(1 << 0)
#define SHTSC_STATUS_POWER_UP		(1 << 1)
#define SHTSC_STATUS_RESUME_PROX	(1 << 2)
#define SHTSC_STATUS_WDT		(1 << 3)
#define SHTSC_STATUS_DCMAP_READY	(1 << 4)
#define SHTSC_STATUS_COMMAND_RESULT	(1 << 5)
#define SHTSC_STATUS_LG_LPWG_TCI1	(1 << 6)
#define SHTSC_STATUS_LG_LPWG_TCI2	(1 << 7)
#define SHTSC_STATUS_FLASH_LOAD_ERROR	(1 << 8)
#define SHTSC_STATUS_PLL_UNLOCK		(1 << 9)

/* DONE IND */
#define SHTSC_IND_CMD			0x20
#define SHTSC_IND_TOUCH			0x01
#define SHTSC_IND_DCMAPDONE		0x10

/* BANK Address */
#define SHTSC_BANK_TOUCH_REPORT		0x00
#define SHTSC_BANK_LPWG_DATA		0x00
#define SHTSC_BANK_LPWG_PARAM		0x01
#define SHTSC_BANK_COMMAND		0x02
#define SHTSC_BANK_COMMAND_RESULT	0x03
#define SHTSC_BANK_DCMAP		0x05
#define SHTSC_BANK_SYSTEM_HA		0x1C

/* Common Register Address */
#define SHTSC_ADDR_INT0		0x00
#define SHTSC_ADDR_INTMASK0	0x01
#define SHTSC_ADDR_BANK		0x02
#define SHTSC_ADDR_IND		0x03
#define SHTSC_ADDR_INT1		0x04
#define SHTSC_ADDR_INTMASK1	0x05

/* Touch Report Register Address */
#define SHTSC_ADDR_TOUCH_NUM	0x08
#define SHTSC_ADDR_RESUME_PROX	0x09
#define SHTSC_ADDR_TOUCH_REPORT	0x10
#define SHTSC_ADDR_LPWG_REPORT	0x10

/* Touch Parmeters */
#define SHTSC_MAX_FINGERS	10
#define SHTSC_MAX_TOUCH_1PAGE	10
#define SHTSC_LENGTH_OF_TOUCH	8
#define SHTSC_LENGTH_OF_LPWG	4

/* Touch Status */
#define SHTSC_F_TOUCH		((u8)0x01)
#define SHTSC_F_TOUCH_OUT	((u8)0x03)
//#define SHTSC_P_TOUCH ((u8)0x02)
//#define SHTSC_P_TOUCH_OUT ((u8)0x04)

#define SHTSC_TOUCHOUT_STATUS	((u8)0x80)

#define SHTSC_ADDR_COMMAND	0x08

/* Command Type */
#define CMD_GETPROPERTY 		"\xE0\x00\x00\x11"
#define CMD_GETPROPERTY_LEN 		4

#define CMD_SETSYSTEMSTATE_SLEEP 	"\x02\x00\x01\x00\x00"
#define CMD_SETSYSTEMSTATE_SLEEP_LEN 	5
#define CMD_SETSYSTEMSTATE_DEEPIDLE 	"\x02\x00\x01\x00\x04"
#define CMD_SETSYSTEMSTATE_DEEPIDLE_LEN	5
#define CMD_SETSYSTEMSTATE_IDLE 	"\x02\x00\x01\x00\x03"
#define CMD_SETSYSTEMSTATE_IDLE_LEN	5
#define CMD_GETSYSTEMSTATE		"\x03\x00\x00\x01"
#define CMD_GETSYSTEMSTATE_LEN		4

#define CMD_SETSYSTEMSTATE_PEN_MODE	"\x02\x00\x01\x00\x08"
#define CMD_SETSYSTEMSTATE_PEN_MODE_LEN	5

#define CMD_SETSYSTEMSTATE_NORMAL_MODE	"\x02\x00\x01\x00\x03"
#define CMD_SETSYSTEMSTATE_NORMAL_MODE_LEN	5

#define CMD_EXECCALIBRATION "\x0F\x00\x03\x00\x05\x20\0x03"
#define CMD_EXECCALIBRATION_LEN 7

#define CMD_SELFDIAG "\xDA\x00\x00\x01"
#define CMD_SELFDIAG_LEN 4


#define CMD_DELAY	16
#define MAX_16BIT	0xffff
#define MAX_12BIT	0xfff
#define MAX_DCMAP_SIZE	(37 * 37 *2)
#define LOW_LEVEL	0
#define HIGH_LEVEL	1
#define SHTSC_DEVBUF_SIZE	1500
#define KNOCKDATA_SIZE	(40)

/* KNOCK ON/CODE FAILURE STATUS */
#define SHTSC_KNOCKON_FAILURE_DISTANCE_INTER_TAP	(1 << 0)
#define SHTSC_KNOCKON_FAILURE_DISTANCE_TOUCHSLOP	(1 << 1)
#define SHTSC_KNOCKON_FAILURE_TIMEOUT_INTERTAP		(1 << 2)
#define SHTSC_KNOCKON_FAILURE_MULTI_FINGER		(1 << 3)
#define SHTSC_KNOCKON_FAILURE_DELAY_TIME		(1 << 4)
#define SHTSC_KNOCKON_FAILURE_PALM_STATE		(1 << 5)

#define KNOCK_CODE_FAILURE_REASONS (\
                                    SHTSC_KNOCKON_FAILURE_DISTANCE_INTER_TAP | \
                                    SHTSC_KNOCKON_FAILURE_DISTANCE_TOUCHSLOP | \
                                    SHTSC_KNOCKON_FAILURE_TIMEOUT_INTERTAP   | \
                                    SHTSC_KNOCKON_FAILURE_MULTI_FINGER       | \
                                    SHTSC_KNOCKON_FAILURE_DELAY_TIME         | \
                                    SHTSC_KNOCKON_FAILURE_PALM_STATE         )

/* Software reset delay */
#define SHTSC_RESET_TIME	250		/* msec */
#define SHTSC_SLEEP_TIME	100		/* msec */

/* ======== EEPROM command ======== */
#define	FLASH_CMD_WRITE_EN	(0x06)		/* Write enable command */
#define	FLASH_CMD_PAGE_WR	(0x02)		/* Page write command */
#define	FLASH_CMD_READ_ST	(0x05)		/* Read status command */
#define	FLASH_CMD_SECTOR_ERASE	(0xD7)		/* Sector erase command */
#define	FLASH_CMD_READ		(0x03)		/* Read command */

/* ======== EEPROM status ======== */
#define	FLASH_ST_BUSY		(1 << 0)	/* Busy with a write operation */

#define CMD_SETSYSTEMSTATE_LEN	5
#define CMD_SETSYSTEMSTATE	"\x02\x00\x01\x00\xff"

/* ======== Version information  ======== */
#define DEVICE_CODE_LR388K6	2
#define VERSION_YEAR		15
#define VERSION_MONTH		6
#define VERSION_DAY		12
#define VERSION_SERIAL_NUMBER	20
#define VERSION_MODEL_CODE	DEVICE_CODE_LR388K6
#define DRIVER_VERSION_LEN	5
#define DRIVER_NAME 		"shtsc"

/* ======= SelfDiagnosis ====== */
#define RESULT_DATA_MAX_LEN	(8 * 1024)
#define FILE_PATH_RESULT	"/mnt/sdcard/touch_self_test.txt"
#define NUM_DRIVE		31
#define NUM_SENSE		18

/* FLASH TIME IMPROVEMENT FEATURE */
#define FLASH_TIME_ZERO_WAIT
#define FLASH_MULTI_WRITE
#define FLASH_NO_VERIFY

#ifdef FLASH_NO_VERIFY
#define FLASH_VERIFY	0
#else
#define FLASH_VERIFY	1
#endif

#ifdef FLASH_TIME_ZERO_WAIT
#define FLASH_WAIT 0
#else
#define FLASH_WAIT 500
#endif

#define RETRY_COUNT			(2000 * 10)
#define FLASH_PAGE_SIZE			(4 << 10)	/* 4k block for each page */
#define FLASH_VERIFY_SIZE		512
#define FLASH_PHYSICAL_PAGE_SIZE 	256	/* can write 256bytes at a time */

enum {
	WAIT_NONE = 0,
	WAIT_CMD,
	WAIT_RESET,
};

enum {
	ABS_MODE = 0,
	KNOCK_1,
	KNOCK_2,
};

struct sharp_touch_data {
	u8 status;
	u8 id;
	u8 size;
	u8 type;
	u16 x;
	u16 y;
	u16 z;
};

struct sharp_touch_info {
	u16 irq_status;
	u16 irq_mask;
	u8 wake_type;
	u8 object_count;
	u8 reg_data[11];
	u8 buf[128];
	struct sharp_touch_data data[10];
};

struct sharp_fw_info {
	u16 moduleMakerID;
	u16 moduleVersion;
	u16 modelID;
	u16 isOfficial;
	u16 version;
	unsigned firm_ver;
	unsigned param_ver;
};

struct sharp_data {
	struct sharp_touch_info info;
	struct sharp_fw_info fw[2];
	u8 cmd;
	u8 wait_state;
	bool wait_result;
	bool reset_done;
	u8 pen_mode;
	int update;
	unsigned char resume_state;
	unsigned char cmd_buf[MAX_COMMAND_RESULT_LEN];
	unsigned char verify_buf[FLASH_VERIFY_SIZE];
};

int sharp_read(struct device *dev, u8 addr, void *data, int size);
int sharp_write(struct device *dev, u8 addr, void *data, int size);
int sharp_ic_info(struct device *dev);
int sharp_init(struct device *dev);
int sharp_set_bank(struct device *dev, u8 bank);
int sharp_set_indicator(struct device *dev, u8 bank);
void sharp_clear_interrupt(struct device *dev, u16 val);

/* extern */
extern int sharp_fw_update(struct device *dev, const struct firmware *fw);


static inline struct sharp_data *to_sharp_data(struct device *dev)
{
	return (struct sharp_data *)touch_get_device(to_touch_core(dev));
}

#endif /* LGE_TOUCH_SHARP_H */
