/* touch_mit300.c
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
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>


#include <touch_core.h>
#include <touch_hwif.h>

#include "touch_mit300.h"
#include "touch_mit300_prd.h"


/*extern int cradle_smart_cover_status(void);*/
extern void touch_interrupt_control(struct device *dev, int on_off);

static int change_cover_func(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 write_buf[4] = {0};

	write_buf[0] = MIP_R0_CTRL;
	write_buf[1] = MIP_R1_CTRL_WINDOW_MODE;
	write_buf[2] = ts->lpwg.qcover;

	if (mit300_reg_write(dev, write_buf, 3)) {
		TOUCH_I("change_cover_func failed\n");
		return -EIO;
	} else {
		TOUCH_I("mit300_set_covermode status = %d(%s)\n",
				ts->lpwg.qcover, ts->lpwg.qcover? "CLOSE": "OPEN");
	}
	return 0;
}

static void mit300_init_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 0;
	ts->tci.info[TCI_1].max_intertap = 700;
	ts->tci.info[TCI_1].touch_slop = 10;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].tap_count = 2;
	ts->tci.info[TCI_2].min_intertap = 0;
	ts->tci.info[TCI_2].max_intertap = 700;
	ts->tci.info[TCI_2].touch_slop = 10;
	ts->tci.info[TCI_2].tap_distance = 65535;
	ts->tci.info[TCI_2].intr_delay = 68;
}

static int mit300_tci_active_area(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	char write_buf[255] = {0};

	TOUCH_I("%s [START]\n", __func__);

	write_buf[0] = MIP_R0_LPWG;
	write_buf[1] = MIP_R1_LPWG_ACTIVE_AREA;
	write_buf[2] = (ts->tci.area.x1 + d->dev.active_area_gap) & 0xFF;		/* MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal start low byte) */
	write_buf[3] = ((ts->tci.area.x1 + d->dev.active_area_gap) >> 8) & 0xFF;	/* MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal start low byte) */
	write_buf[4] = (ts->tci.area.y1 + d->dev.active_area_gap) & 0xFF;		/* MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical start low byte) */
	write_buf[5] = ((ts->tci.area.y1 + d->dev.active_area_gap) >> 8) & 0xFF;	/* MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical start high byte) */
	write_buf[6] = (ts->tci.area.x2 - d->dev.active_area_gap) & 0xFF;		/* MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal end low byte) */
	write_buf[7] = ((ts->tci.area.x2 - d->dev.active_area_gap) >> 8) & 0xFF;	/* MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal end high byte) */
	write_buf[8] = (ts->tci.area.y2 - d->dev.active_area_gap) & 0xFF;		/* MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical end low byte) */
	write_buf[9] = ((ts->tci.area.y2 - d->dev.active_area_gap) >> 8) & 0xFF;	/* MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical end high byte) */

	if (mit300_reg_write(dev, write_buf, 10)) {
		TOUCH_E("MIP_R1_LPWG_ACTIVE_AREA write error \n");
		return -EIO;
	} else {
		TOUCH_I("MIP_R1_LPWG_ACTIVE_AREA\n");
	}
	TOUCH_I("%s [DONE]\n", __func__);

	return 0;
}

int tci_control(struct device *dev, int type, u16 value)
{
	char write_buf[255] = {0};
	/* Common Reg */
	switch (type) {
		case IDLE_REPORTRATE_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_IDLE_REPORTRATE;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_IDLE_REPORTRATE write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_IDLE_REPORTRATE\n");
			}
			break;
		case ACTIVE_REPORTRATE_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_ACTIVE_REPORTRATE;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_ACTIVE_REPORTRATE write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_ACTIVE_REPORTRATE\n");
			}
			break;
		case SENSITIVITY_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_SENSITIVITY;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_SENSITIVITY write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_SENSITIVITY = %d \n", write_buf[2]);
			}
			break;
		/* TCI1 reg */
		case TCI_ENABLE_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_ENABLE;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_ENABLE write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_ENABLE = %d \n", write_buf[2]);
			}
			break;
		case TAP_COUNT_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_WAKEUP_TAP_COUNT;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_WAKEUP_TAP_COUNT write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_WAKEUP_TAP_COUNT\n");
			}
			break;
		case TOUCH_SLOP_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_TOUCH_SLOP;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_TOUCH_SLOP write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_TOUCH_SLOP\n");
			}
			break;
		case TAP_MIN_DISTANCE_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_MIN_INTERTAP_DISTANCE;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_MIN_INTERTAP_DISTANCE write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_MIN_INTERTAP_DISTANCE\n");
			}
			break;
		case TAP_MAX_DISTANCE_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_MAX_INTERTAP_DISTANCE;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_MAX_INTERTAP_DISTANCE write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_MAX_INTERTAP_DISTANCE\n");
			}
			break;
		case MIN_INTERTAP_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_MIN_INTERTAP_TIME;
			write_buf[2] = (value >> 8);
			write_buf[3] = (value & 0xFF);
			if (mit300_reg_write(dev, write_buf, 4)) {
				TOUCH_I("MIP_R1_LPWG_MIN_INTERTAP_TIME write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_MIN_INTERTAP_TIME\n");
			}
			break;
		case MAX_INTERTAP_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_MAX_INTERTAP_TIME;
			write_buf[2] = (value >> 8);
			write_buf[3] = (value & 0xFF);
			if (mit300_reg_write(dev, write_buf, 4)) {
				TOUCH_I("MIP_R1_LPWG_MAX_INTERTAP_TIME write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_MAX_INTERTAP_TIME\n");
			}
			break;
		case INTERRUPT_DELAY_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_INT_DELAY_TIME;
			write_buf[2] = ((value ? KNOCKON_DELAY : 0) >> 8);
			write_buf[3] = ((value ? KNOCKON_DELAY : 0) & 0xFF);
			if (mit300_reg_write(dev, write_buf, 4)) {
				TOUCH_I("MIP_R1_LPWG_INT_DELAY_TIME write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_INT_DELAY_TIME\n");
			}
			break;
		/* TCI2 reg */
		case TCI_ENABLE_CTRL2:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_ENABLE2;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_ENABLE2 write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_ENABLE2 = %d\n", write_buf[2]);
			}
			break;
		case TAP_COUNT_CTRL2:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_WAKEUP_TAP_COUNT2;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_WAKEUP_TAP_COUNT2 write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_WAKEUP_TAP_COUNT2 = %d\n", write_buf[2]);
			}
			break;
		case TOUCH_SLOP_CTRL2:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_TOUCH_SLOP2;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_TOUCH_SLOP2 write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_TOUCH_SLOP2\n");
			}
			break;
		case TAP_MIN_DISTANCE_CTRL2:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_MIN_INTERTAP_DISTANCE2;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_MIN_INTERTAP_DISTANCE2 write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_MIN_INTERTAP_DISTANCE2\n");
			}
			break;
		case TAP_MAX_DISTANCE_CTRL2:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_MAX_INTERTAP_DISTANCE2;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_MAX_INTERTAP_DISTANCE2 write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_MAX_INTERTAP_DISTANCE2\n");
			}
			break;
		case MIN_INTERTAP_CTRL2:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_MIN_INTERTAP_TIME2;
			write_buf[2] = (value >> 8);
			write_buf[3] = (value & 0xFF);
			if (mit300_reg_write(dev, write_buf, 4)) {
				TOUCH_I("MIP_R1_LPWG_MIN_INTERTAP_TIME2 write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_MIN_INTERTAP_TIME2\n");
			}
			break;
		case MAX_INTERTAP_CTRL2:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_MAX_INTERTAP_TIME2;
			write_buf[2] = (value >> 8);
			write_buf[3] = (value & 0xFF);
			if (mit300_reg_write(dev, write_buf, 4)) {
				TOUCH_I("MIP_R1_LPWG_MAX_INTERTAP_TIME2 write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_MAX_INTERTAP_TIME2\n");
			}
			break;
		case INTERRUPT_DELAY_CTRL2:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_INT_DELAY_TIME2;
			write_buf[2] = ((value ? KNOCKON_DELAY : 0) >> 8);
			write_buf[3] = ((value ? KNOCKON_DELAY : 0) & 0xFF);
			if (mit300_reg_write(dev, write_buf, 4)) {
				TOUCH_I("MIP_R1_LPWG_INT_DELAY_TIME2 write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_INT_DELAY_TIME2\n");
			}
			break;
		case LPWG_STORE_INFO_CTRL:
			TOUCH_I("not used\n");
			break;
		case LPWG_START_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_START;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_START write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_START\n");
			}
			break;
		case LPWG_PANEL_DEBUG_CTRL:
			write_buf[0] = MIP_R0_CTRL;
			write_buf[1] = MIP_R1_CTRL_LPWG_DEBUG_ENABLE;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_CTRL_LPWG_DEBUG_ENABLE write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_CTRL_LPWG_DEBUG_ENABLE = %d \n", write_buf[2]);
			}
			break;
		case LPWG_FAIL_REASON_CTRL:
			write_buf[0] = MIP_R0_LPWG;
			write_buf[1] = MIP_R1_LPWG_FAIL_REASON;
			write_buf[2] = value;
			if (mit300_reg_write(dev, write_buf, 3)) {
				TOUCH_I("MIP_R1_LPWG_FAIL_REASON write error \n");
				return -EIO;
			} else {
				TOUCH_I("MIP_R1_LPWG_FAIL_REASON = %d \n", write_buf[2]);
			}
			break;
		default:
			break;
	}

	return 0;
}

/**
* LPWG Config Public
*/
int mip_lpwg_config(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	u8 wbuf[32];

	TOUCH_I("%s [START]\n", __func__);

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_IDLE_REPORTRATE;
	wbuf[2] = 20;			                                                /* MIP_ADDR_SET_LPWG_IDLE_REPORTRATE */
	wbuf[3] = 40;			                                                /* MIP_ADDR_SET_LPWG_ACTIVE_REPORTRATE */
	wbuf[4] = 30;                                               		        /* MIP_ADDR_SET_LPWG_SENSITIVITY */
	wbuf[5] = (ts->tci.area.x1 + d->dev.active_area_gap) & 0xFF;		        /* MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal start low byte) */
	wbuf[6] = ((ts->tci.area.x1 + d->dev.active_area_gap) >> 8) & 0xFF;	        /* MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal start low byte) */
	wbuf[7] = (ts->tci.area.y1 + d->dev.active_area_gap) & 0xFF;		        /* MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical start low byte) */
	wbuf[8] = ((ts->tci.area.y1 + d->dev.active_area_gap) >> 8) & 0xFF;	        /* MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical start high byte) */
	wbuf[9] = (ts->tci.area.x2 - d->dev.active_area_gap) & 0xFF;		        /* MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal end low byte) */
	wbuf[10] = ((ts->tci.area.x2 - d->dev.active_area_gap) >> 8) & 0xFF;	        /* MIP_ADDR_SET_LPWG_ACTIVE_AREA (horizontal end high byte) */
	wbuf[11] = (ts->tci.area.y2 - d->dev.active_area_gap) & 0xFF;			/* MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical end low byte) */
	wbuf[12] = ((ts->tci.area.y2 - d->dev.active_area_gap) >> 8) & 0xFF;		/* MIP_ADDR_SET_LPWG_ACTIVE_AREA (vertical end high byte) */
	wbuf[13] = 1;                                           			/* MIP_ADDR_SET_LPWG_FAIL_REASON */

	TOUCH_I("    [Hz] : idle(%4d), active(%4d)\n", wbuf[2], wbuf[3]);
	TOUCH_I("    sensitivity(%4d)\n", wbuf[4]);
	TOUCH_I("    [area start] hori.(%4d) vert.(%4d)\n", (wbuf[6]<<8)|wbuf[5], (wbuf[8]<<8)|wbuf[7]);
	TOUCH_I("    [area end  ] hori.(%4d) vert.(%4d)\n", (wbuf[10]<<8)|wbuf[9], (wbuf[12]<<8)|wbuf[11]);
	TOUCH_I("    fail reason(%d)\n", wbuf[13]);

	if (mit300_reg_write(dev, wbuf, 14)) {
		TOUCH_E("%s [ERROR] mip_i2c_write\n", __func__);
		return 1;
	}

	TOUCH_I("%s [DONE]\n", __func__);
	return 0;
}

/**
* LPWG Config Knock-on
*/
int mip_lpwg_config_knock_on(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 wbuf[32];

	TOUCH_I("%s [START]\n", __func__);

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE;
	wbuf[2] = 1;							                /* MIP_ADDR_SET_LPWG_ENABLE */
	wbuf[3] = 2;							                /* MIP_ADDR_SET_LPWG_TAP_COUNT */
	wbuf[4] = ts->tci.info[TCI_1].touch_slop & 0xFF;		                /* MIP_ADDR_SET_LPWG_TOUCH_SLOP (low byte) */
	wbuf[5] = ts->tci.info[TCI_1].touch_slop >>8 & 0xFF;		                /* MIP_ADDR_SET_LPWG_TOUCH_SLOP (high byte) */
	wbuf[6] = (0) & 0xFF;		                                                /* MIP_ADDR_SET_LPWG_MIN_DISTANCE (low byte) */
	wbuf[7] = (0) >>8 & 0xFF;	                                                /* MIP_ADDR_SET_LPWG_MIN_DISTANCE (high byte) */
	wbuf[8] = (ts->tci.info[TCI_1].tap_distance) & 0xFF;		                /* MIP_ADDR_SET_LPWG_MAX_DISTANCE (low byte) */
	wbuf[9] = (ts->tci.info[TCI_1].tap_distance) >>8 & 0xFF;	                /* MIP_ADDR_SET_LPWG_MAX_DISTANCE (high byte) */
	wbuf[10] = (ts->tci.info[TCI_1].min_intertap) & 0xFF;		                /* MIP_ADDR_SET_LPWG_MIN_INTERTAP_TIME (low byte) */
	wbuf[11] = (ts->tci.info[TCI_1].min_intertap) >>8 & 0xFF;	                /* MIP_ADDR_SET_LPWG_MIN_INTERTAP_TIME (high byte) */
	wbuf[12] = (ts->tci.info[TCI_1].max_intertap) & 0xFF;		                /* MIP_ADDR_SET_LPWG_MAX_INTERTAP_TIME (low byte) */
	wbuf[13] = (ts->tci.info[TCI_1].max_intertap) >>8 & 0xFF;	                /* MIP_ADDR_SET_LPWG_MAX_INTERTAP_TIME (high byte) */
	wbuf[14] = (ts->tci.double_tap_check ? KNOCKON_DELAY : 0) & 0xFF;		/* MIP_ADDR_SET_LPWG_MAX_INTERTAP_DELAY (low byte) */
	wbuf[15] = ((ts->tci.double_tap_check ? KNOCKON_DELAY : 0) >> 8) & 0xFF;	/* MIP_ADDR_SET_LPWG_MAX_INTERTAP_DELAY (high byte) */

	TOUCH_I("    enable(%4d), tap_count(%4d) touch_slope(%4d)\n", wbuf[2], wbuf[3], (wbuf[5]<<8)|wbuf[4]);
	TOUCH_I("    [intertap distance] min(%4d), max(%4d)\n", (wbuf[7]<<8)|wbuf[6], (wbuf[9]<<8)|wbuf[8]);
	TOUCH_I("    [intertap time    ] min(%4d), max(%4d)\n", (wbuf[11]<<8)|wbuf[10], (wbuf[13]<<8)|wbuf[12]);
	TOUCH_I("    interrupt_delay(%4d)\n", (wbuf[15]<<8)|wbuf[14]);

	if (mit300_reg_write(dev, wbuf, 16)) {
		TOUCH_E("[ERROR] mip_i2c_write\n");
		return 1;
	}

	TOUCH_I("%s [DONE]\n", __func__);
	return 0;
}

/**
* LPWG Config Knock-code
*/
int mip_lpwg_config_knock_code(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 wbuf[32];

	TOUCH_I("%s [START]\n", __func__);

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE2;
	wbuf[2] = 1;							                /* MIP_ADDR_SET_LPWG_ENABLE */
	wbuf[3] = ts->tci.info[TCI_2].tap_count;                    			/* MIP_ADDR_SET_LPWG_TAP_COUNT */
	wbuf[4] = ts->tci.info[TCI_2].touch_slop & 0xFF;		                /* MIP_ADDR_SET_LPWG_TOUCH_SLOP (low byte) */
	wbuf[5] = ts->tci.info[TCI_2].touch_slop >>8 & 0xFF;	                        /* MIP_ADDR_SET_LPWG_TOUCH_SLOP (high byte) */
	wbuf[6] = (0) & 0xFF;		                                                /* MIP_ADDR_SET_LPWG_MIN_DISTANCE (low byte) */
	wbuf[7] = (0) >>8 & 0xFF;	                                                /* MIP_ADDR_SET_LPWG_MIN_DISTANCE (high byte) */
	wbuf[8] = (ts->tci.info[TCI_2].tap_distance) & 0xFF;		                /* MIP_ADDR_SET_LPWG_MAX_DISTANCE (low byte) */
	wbuf[9] = (ts->tci.info[TCI_2].tap_distance) >>8 & 0xFF;	                /* MIP_ADDR_SET_LPWG_MAX_DISTANCE (high byte) */
	wbuf[10] = (ts->tci.info[TCI_2].min_intertap) & 0xFF;	                        /* MIP_ADDR_SET_LPWG_MIN_INTERTAP_TIME (low byte) */
	wbuf[11] = (ts->tci.info[TCI_2].min_intertap) >>8 & 0xFF;	                /* MIP_ADDR_SET_LPWG_MIN_INTERTAP_TIME (high byte) */
	wbuf[12] = (ts->tci.info[TCI_2].max_intertap) & 0xFF;	                        /* MIP_ADDR_SET_LPWG_MAX_INTERTAP_TIME (low byte) */
	wbuf[13] = (ts->tci.info[TCI_2].max_intertap) >>8 & 0xFF;	                /* MIP_ADDR_SET_LPWG_MAX_INTERTAP_TIME (high byte) */
	wbuf[14] = (250) & 0xFF;	                                                /* MIP_ADDR_SET_LPWG_MAX_INTERTAP_DELAY (low byte) */
	wbuf[15] = (250) >>8 & 0xFF;	                                                /* MIP_ADDR_SET_LPWG_MAX_INTERTAP_DELAY (high byte) */

	TOUCH_I("    enable(%4d), tap_count(%4d) touch_slope(%4d)\n", wbuf[2], wbuf[3], (wbuf[5]<<8)|wbuf[4]);
	TOUCH_I("    [intertap distance] min(%4d), max(%4d)\n", (wbuf[7]<<8)|wbuf[6], (wbuf[9]<<8)|wbuf[8]);
	TOUCH_I("    [intertap time    ] min(%4d), max(%4d)\n", (wbuf[11]<<8)|wbuf[10], (wbuf[13]<<8)|wbuf[12]);
	TOUCH_I("    interrupt_delay(%4d)\n", (wbuf[15]<<8)|wbuf[14]);

	if (mit300_reg_write(dev, wbuf, 16)) {
		TOUCH_E("[ERROR] mip_i2c_write\n");
		return 1;
	}

	TOUCH_I("%s [DONE]\n", __func__);
	return 0;
}

int mip_lpwg_enable_sensing(struct device *dev, bool bEnable)
{
	u8 wbuf[4];

	TOUCH_I("%s [START]\n", __func__);

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE_SENSING;
	wbuf[2] = bEnable;

	if (mit300_reg_write(dev, wbuf, 3)) {
			TOUCH_E("%s [ERROR] mip_i2c_write\n", __func__);
			return 1;
	}

	TOUCH_I("%s [DONE]\n", __func__);
	return 0;
}

/**
* LPWG Control Start
*/
int mip_lpwg_start(struct device *dev)
{
	u8 wbuf[4];

	TOUCH_I("%s [START]\n", __func__);

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_START;
	wbuf[2] = 1;

	if (mit300_reg_write(dev, wbuf, 3)) {
		TOUCH_E("%s [ERROR] mip_i2c_write\n", __func__);
		return 1;
	}

	TOUCH_I("%s [DONE]\n", __func__);
	return 0;
}

static int mit300_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("%s mode=%d\n", __func__, mode);

	switch (mode) {
		case LPWG_DOUBLE_TAP:
			ts->tci.mode = LPWG_DOUBLE_TAP;
			mip_lpwg_config(dev);
			mip_lpwg_config_knock_on(dev);
			break;

		case LPWG_PASSWORD:
			ts->tci.mode = LPWG_PASSWORD;
			mip_lpwg_config(dev);
			mip_lpwg_config_knock_on(dev);
			mip_lpwg_config_knock_code(dev);
			break;

		default:
			ts->tci.mode = 0;
			tci_control(dev, TCI_ENABLE_CTRL, 0);
			tci_control(dev, TCI_ENABLE_CTRL2, 0);
			break;
	}

	return 0;
}

static int mit300_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg) {
			d->dev.active_area_gap = 0;
			TOUCH_I("%s(%d) - not set active area gap : %d\n",
				__func__, __LINE__, d->dev.active_area_gap);
			mit300_lpwg_control(dev, LPWG_DOUBLE_TAP);
			mip_lpwg_enable_sensing(dev, LPWG_ENABLE_SENSING);
			mip_lpwg_start(dev);
			return 0;
		}
		if (ts->lpwg.mode == LPWG_NONE) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep\n",
				__func__, __LINE__);
			mit300_lpwg_control(dev, LPWG_NONE);
			mip_lpwg_enable_sensing(dev, LPWG_DISABLE_SENSING);
			mip_lpwg_start(dev);
		} else if (ts->lpwg.screen) {
			TOUCH_I("%s(%d) - FB_SUSPEND & screen on -> skip\n",
				__func__, __LINE__);
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			return 0;
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep by prox\n",
				__func__, __LINE__);
			mit300_lpwg_control(dev, LPWG_NONE);
			mip_lpwg_enable_sensing(dev, LPWG_DISABLE_SENSING);
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			/* hall near */
			TOUCH_I("%s(%d) - deep sleep by hole_near\n",
				__func__, __LINE__);
			mit300_lpwg_control(dev, LPWG_NONE);
			mip_lpwg_enable_sensing(dev, LPWG_DISABLE_SENSING);
			mip_lpwg_start(dev);
		} else {
			/* knock on/code */
			TOUCH_I("%s(%d) - knock %d\n",
				__func__, __LINE__, ts->lpwg.mode);
			mit300_lpwg_control(dev, ts->lpwg.mode);
			mip_lpwg_enable_sensing(dev, LPWG_ENABLE_SENSING);
			mip_lpwg_start(dev);
		}
		return 0;
	}

	/* resume */
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("%s(%d) - normal\n",
				__func__, __LINE__);
	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* normal */
		TOUCH_I("%s(%d) - normal on screen off\n",
				__func__, __LINE__);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		/* wake up */
		TOUCH_I("%s(%d) - wake up on screen off and prox\n",
				__func__, __LINE__);
		TOUCH_I("%s - wake up is not ready\n", __func__);
	} else {
		/* partial */
		touch_report_all_event(ts);
		TOUCH_I("%s(%d) - partial mode\n",
				__func__, __LINE__);
		TOUCH_I("%s - partial is not ready\n", __func__);
	}

	return 0;
}

int mit300_ic_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);

	u8 wbuf[8];
	u8 rbuf[64] = {0};

	TOUCH_TRACE();

	/* Product name */
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;
	if (mit300_reg_read(dev, wbuf, 2, rbuf, 16)) {
		TOUCH_E("[ERROR] read product name\n");
		return -EIO;
	}
	memcpy((u8 *) &d->module.product_code, rbuf, 16);

	/* Ic name */
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_IC_NAME;
	if (mit300_reg_read(dev, wbuf, 2, rbuf, 4)) {
		TOUCH_E("[ERROR] read ic name\n");
		return -EIO;
	}
	memcpy((u8 *) &d->module.ic_name, rbuf, 4);

	/* Firmware version */
	if (mip_get_fw_version(dev, rbuf)) {
		TOUCH_E("[ERROR] get fw version\n");
		return -EIO;
	}
	memcpy((u8 *) &d->module.version, rbuf, 2);

	/* Resolution */
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
	if (mit300_reg_read(dev, wbuf, 2, rbuf, 7)) {
		TOUCH_E("[ERROR] get resulution\n");
		d->dev.x_resolution = 720;
		d->dev.x_resolution = 1280;
		return -EIO;
	} else {
		d->dev.x_resolution = (rbuf[0]) | (rbuf[1] << 8);
		d->dev.y_resolution = (rbuf[2]) | (rbuf[3] << 8);
	}

	/* Non-active area gap */
	d->dev.active_area_gap = 60;
	TOUCH_I("set active area gap = %d\n", d->dev.active_area_gap);

	/* Node info */
	d->dev.col_num = rbuf[4];
	d->dev.row_num = rbuf[5];
	d->dev.key_num = rbuf[6];

	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_SUPPORTED_FUNC;
	if (mit300_reg_read(dev, wbuf, 2, rbuf, 7)) {
		TOUCH_E("[ERROR] read node info\n");
		return -EIO;
	}
	d->event_format = (rbuf[4]) | (rbuf[5] << 8);
	d->event_size = rbuf[6];

	if (ts->lpwg.screen) {
		TOUCH_I("====== LCD  ON  ======\n");
	} else {
		TOUCH_I("====== LCD  OFF ======\n");
	}

	TOUCH_I("======================\n");
	TOUCH_I("F/W Version : %X.%02X \n", d->module.version[0], d->module.version[1]);
	TOUCH_I("F/W Product : %s \n", d->module.product_code);
	TOUCH_I("F/W Row(node_y) : %d, Col(node_x) : %d \n", d->dev.row_num, d->dev.col_num);
	TOUCH_I("IC Name : %s \n", d->module.ic_name);
	TOUCH_I("max_x[%d] max_y[%d]\n", d->dev.x_resolution, d->dev.y_resolution);
	TOUCH_I("======================\n");

	return 0;
}

static int mit300_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = NULL;
	int i = 0;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate mit300 data\n");
		return -ENOMEM;
	}
	d->d_dev = dev;
	touch_set_device(ts, d);

	d->limit = kzalloc(sizeof(struct touch_limit_value), GFP_KERNEL);

	for (i = 0; i < MAX_ROW; i++) {
		d->mit_data[i] = kzalloc(sizeof(uint16_t) * MAX_COL, GFP_KERNEL);
		if (d->mit_data[i] == NULL) {
			TOUCH_E("mit_data kzalloc error\n");
			return -ENOMEM;
		}
		d->intensity_data[i] = kzalloc(sizeof(uint16_t) * MAX_COL, GFP_KERNEL);
		if (d->intensity_data[i] == NULL) {
			TOUCH_E("intensity_data kzalloc error\n");
			return -ENOMEM;
		}
	}

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 1);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, 4096);

	mit300_init_tci_info(dev);
	mit300_ic_info(dev);
	mit300_debugging(dev);

	return 0;
}

static int mit300_remove(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int mit300_suspend(struct device *dev)
{
	TOUCH_TRACE();
	mit300_lpwg_mode(dev);

	return 0;
}

static int mit300_resume(struct device *dev)
{
	TOUCH_TRACE();
	mit300_lpwg_mode(dev);
	return 0;
}

static int mit300_init(struct device *dev)
{
	struct mit_data *d = to_mit_data(dev);

	TOUCH_TRACE();

	d->check_openshort = 1;

	return 0;
}

int mit300_reg_read(struct device *dev, u8 *addr, int tx_size, void *data, int rx_size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct touch_bus_msg msg;
	int ret = 0;
	int i = 0;

	for (i = 0; i < tx_size; i++) {
		ts->tx_buf[i] = addr[i];
		ts->tx_buf[i] = addr[i];
	}

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = tx_size;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = rx_size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], rx_size);
	return 0;
}

int mit300_reg_write(struct device *dev, u8 *addr, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	ts->tx_buf[0] = addr[0];
	ts->tx_buf[1] = addr[1];
	memcpy(&ts->tx_buf[2], &addr[2], size-2);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		return ret;
	}

	return 0;
}

static int mit_get_packet(struct device *dev, u8 *category)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);

	u8 sz = 0;
	u8 wbuf[8] = {0};
	u8 rbuf[256] = {0};
	int size = 0;

	TOUCH_TRACE();

	/* send Dummy packet(lpwg mode) */
	if (ts->tci.mode) {
		if (mit300_reg_write(dev, wbuf, 2)) {
			TOUCH_E("[ERROR] Dummy packet\n");
			return -EIO;
		}
	}
	/* Read packet info */
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
	if (mit300_reg_read(dev, wbuf, 2, rbuf, 1)) {
		TOUCH_E("[ERROR] Read packet info\n");
		return -EIO;
	}
	size = (rbuf[0] & 0x7F);
	*category = ((rbuf[0] >> 7) & 0x1);
	sz = ((rbuf[0] >> 7) & 0x1);
	TOUCH_D(GET_DATA, "%s - packet info : size[%d] category[%d], type[%d]\n", __func__, size, *category, sz);

	/* Check size */
	if (size <= 0) {
		TOUCH_E("[ERROR] Packet size [%d]\n", size);
		return -EIO;
	}

	/* Read packet data */
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
	if (mit300_reg_read(dev, wbuf, 2, d->buf, size)) {
		TOUCH_E("%s [ERROR] Read packet data\n", __func__);
		return -EIO;
	}

	return size;
}

static int mit_touch_event(struct device *dev, u8 *buf, int sz)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	struct touch_data *tdata;

	u8 touch_count = 0;
	int i = 0;
	int id = 0, x = 0, y = 0;
	int pressure = 0;
	int touch_major = 0, touch_minor = 0;
	int palm = 0;
	int size = 0;
    u8 event_format = d->event_format;
    int event_size = d->event_size;

	TOUCH_TRACE();

	for (i = 0; i < sz; i += event_size) {
		u8 *tmp = &buf[i];

		/* Report input data */
		if ((tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0) {
			/* Touchkey Event */
			TOUCH_I("use software key\n");
		} else {
			/* Touchscreen Event Protocol Type */
			if(event_format == 0) {
				id = (tmp[0] & 0xf) - 1;
				x = tmp[2] | ((tmp[1] & 0xf) << 8);
				y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
				pressure = tmp[4];
				touch_major = tmp[5];
				palm = (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4;
			} else if(event_format == 1) {
				id = (tmp[0] & 0xf) - 1;
				x = tmp[2] | ((tmp[1] & 0xf) << 8);
				y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
				pressure = tmp[4];
				size = tmp[5];
				touch_major = tmp[6];
				touch_minor = tmp[7];
				palm = (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4;
			} else if(event_format == 2) {
				id = (tmp[0] & 0xf) - 1;
				x = tmp[2] | ((tmp[1] & 0xf) << 8);
				y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
				pressure = tmp[4];
				touch_major = tmp[5];
				touch_minor = tmp[6];
				palm = (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4;
			} else {
				TOUCH_E("%s [ERROR] Unknown event format [%d]\n", __func__, event_format);
				return -EIO;
			}

			if ((id > MAX_FINGER - 1) || (id < 0)) {
				TOUCH_E("%s [ERROR] Abnormal id [%d]\n", __func__, id);
				return -EIO;
			}

			/* quickcover filter */
			if (hallic_is_pouch_closed() && d->use_quick_window) {
				if (ts->lpwg.qcover == QUICKCOVER_OPEN) {
					TOUCH_I("Skip irq while quickcover is closed!! \n");
					ts->intr_status = TOUCH_IRQ_NONE;
					touch_report_all_event(ts);
					return 0;
				} else {
					if (x < ts->tci.area.x1 || x > ts->tci.area.x2
							|| y < ts->tci.area.y1 || y > ts->tci.area.y2) {
						TOUCH_D(QUICKCOVER, "Quickcover is closed, Irq of left area is skipped!!\n");
						tmp[0] &= ~(1 << 7u);
					}
				}
			}

			if((tmp[0] & MIP_EVENT_INPUT_PRESS) == 0) {
				if (d->ispalm) {
					TOUCH_I("Palm released : %d \n", pressure);
					ts->is_cancel = 0;
					ts->tcount = 0;
					ts->intr_status = TOUCH_IRQ_FINGER;
					d->ispalm = 0;
					ts->new_mask = 0;
					return 0;
				}
				ts->new_mask &= ~(1 << id);
			} else {
				if (palm) {
					TOUCH_I("Palm detected : %d \n", pressure);
					ts->is_cancel = 1;
					ts->tcount = 0;
					ts->intr_status = TOUCH_IRQ_FINGER;
					d->ispalm = 1;
					return 0;
				}
				ts->new_mask |= (1 << id);
				tdata = ts->tdata + id;

				tdata->id = id;
				tdata->type = 1; /* FINGER */
				tdata->x = x;
				tdata->y = y;

				/* check pressure on Finger Touch */
				if ( pressure < 1 ) { pressure = 1; }
				else if (pressure > MAX_PRESSURE - 1) { pressure = MAX_PRESSURE - 1; }

				tdata->pressure = pressure;
				tdata->width_major = touch_major;
				tdata->width_minor = touch_minor;
				tdata->orientation = 0;

				TOUCH_D(ABS, "tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
					tdata->id,
					tdata->type,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor,
					tdata->orientation);
			}
		}
	}

	for(i = 0; i < MAX_FINGER; i++) {
		if(ts->new_mask & (1 << i))
			touch_count++;
	}

	ts->tcount = touch_count;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return 0;
}

static int mit_lpwg_event(struct device *dev, u8 *buf, int sz)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 wbuf[4];
	u8 gesture_code = buf[1];
	int i = 0, x = 0, y = 0;

	TOUCH_I("%s - gesture_code[%d]\n", __func__, gesture_code);
	if (ts->lpwg.screen == 0 && ts->lpwg.mode) {
		switch(gesture_code){
			case MIP_EVENT_GESTURE_DOUBLE_TAP:
				ts->intr_status |= TOUCH_IRQ_KNOCK;
				for(i = 2; i < sz; i += LPWG_EVENT_SZ){
					u8 *tmp = &buf[i];
					x = tmp[1] | ((tmp[0] & 0xf) << 8);
					y = tmp[2] | (((tmp[0] >> 4) & 0xf) << 8);
					TOUCH_I("LPWG[%d] - %d TAP x[%3d] y[%3d] \n", gesture_code, (i+1)/LPWG_EVENT_SZ, x, y);
					ts->lpwg.code[((i + 1) / LPWG_EVENT_SZ) - 1].x = x;
					ts->lpwg.code[((i + 1) / LPWG_EVENT_SZ) - 1].y = y;
					ts->lpwg.code_num++;
				}
				ts->lpwg.code[((i + 1) / LPWG_EVENT_SZ) - 1].x = -1;
				ts->lpwg.code[((i + 1) / LPWG_EVENT_SZ) - 1].y = -1;
				break;
			case MIP_EVENT_GESTURE_MULTI_TAP:
				ts->intr_status |= TOUCH_IRQ_PASSWD;
				for(i = 2; i < sz; i += LPWG_EVENT_SZ){
					u8 *tmp = &buf[i];
					x = tmp[1] | ((tmp[0] & 0xf) << 8);
					y = tmp[2] | (((tmp[0] >> 4) & 0xf) << 8);
					if ((ts->lpwg.mode == LPWG_PASSWORD) && (ts->role.hide_coordinate))
						TOUCH_I("LPWG data xxx, xxx\n");
					else
						TOUCH_I("LPWG[%d] - %d TAP x[%3d] y[%3d] \n", gesture_code, (i+1)/LPWG_EVENT_SZ, x, y);
					ts->lpwg.code[((i + 1) / LPWG_EVENT_SZ) - 1].x = x;
					ts->lpwg.code[((i + 1) / LPWG_EVENT_SZ) - 1].y = y;
					ts->lpwg.code_num++;
				}
				ts->lpwg.code[((i + 1) / LPWG_EVENT_SZ) - 1].x = -1;
				ts->lpwg.code[((i + 1) / LPWG_EVENT_SZ) - 1].y = -1;
				break;
/*
			case MIP_EVENT_GESTURE_SWIPE:
				TOUCH_INFO_MSG("SWIPE EVENT - [%d] \n", gesture_code);
				ts->pdata->send_lpwg = LPWG_SWIPE_DOWN;
				for(i = 2; i < sz; i += LPWG_EVENT_SZ){
					u8 *tmp = &buf[i];
					x = tmp[1] | ((tmp[0] & 0xf) << 8);
					y = tmp[2] | (((tmp[0] >> 4) & 0xf) << 8);
					if (ts->pdata->role->use_security_all || (ts->pdata->role->use_security_mode && lockscreen_stat_mit300 == 1)) {
						TOUCH_INFO_MSG("LPWG[%d] - %d TAP x[XXX] y[XXX] \n", gesture_code, (i+1)/LPWG_EVENT_SZ);
					} else {
						TOUCH_INFO_MSG("LPWG[%d] - %d TAP x[%3d] y[%3d] \n", gesture_code, (i+1)/LPWG_EVENT_SZ, x, y);
					}
					ts->pdata->lpwg_x[((i + 1) / LPWG_EVENT_SZ) - 1] = x;
					ts->pdata->lpwg_y[((i + 1) / LPWG_EVENT_SZ) - 1] = y;
					ts->pdata->lpwg_size++;
				}
				wakeup_by_swipe_mit300 = true;
				TOUCH_INFO_MSG("[%s] - wakeup_by_swipe_mit300 [%d] \n", __func__, wakeup_by_swipe_mit300);
				mip_swipe_disable(ts->client);
	#if 0 // not yet TW
				if (ts->lpwg_ctrl.password_enable || ts->pdata->role->use_security_all) {
					TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "LPWG Swipe Gesture: "
						"start(xxxx,xxxx) end(xxxx,xxxx) "
						"swipe_fail_reason(%d) swipe_time(%dms)\n",
						swipe_fail_reason, swipe_time);
				} else {
					TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "LPWG Swipe Gesture: "
						"start(%4d,%4d) end(%4d,%4d) "
						"swipe_fail_reason(%d) swipe_time(%dms)\n",
						swipe_start_x, swipe_start_y,
						swipe_end_x, swipe_end_y,
						swipe_fail_reason, swipe_time);
				}
	#endif
				break;
*/
			default:
				/* Re-enter nap mode */
				wbuf[0] = MIP_R0_CTRL;
				wbuf[1] = MIP_R1_CTRL_POWER_STATE;
				wbuf[2] = MIP_CTRL_POWER_LOW;
				if (mit300_reg_write(dev, wbuf, 3)) {
					TOUCH_E("%s [ERROR] mip_i2c_write\n", __func__);
					return -EIO;
				}
				break;
		}
	} else {
		TOUCH_E("LPWG[%d] - abnormal wakeup!!!\n", gesture_code);
		return -EIO;
	}

	return 0;
}

static int mit_fail_reason_event(struct device *dev, u8 *buf, int sz)
{
	u8 wbuf[4];
	u8 gesture_code = buf[1];
	TOUCH_I("%s - gesture_code[%d]\n", __func__, gesture_code);
	switch (gesture_code) {
		case MIP_LPWG_EVENT_TYPE_FAIL:
			switch (buf[2]) {
				case FAIL_OUT_OF_AREA:            TOUCH_I("LPWG FAIL REASON = Out of Area\n");                break;
				case FAIL_PALM:                   TOUCH_I("LPWG FAIL REASON = Palm\n");                       break;
				case FAIL_DELAY_TIME:             TOUCH_I("LPWG FAIL REASON = Delay Time\n");                 break;
				case FAIL_TAP_TIME:               TOUCH_I("LPWG FAIL REASON = Tap Time\n");                   break;
				case FAIL_TAP_DISTACE:            TOUCH_I("LPWG FAIL REASON = Tap Distance\n");               break;
				case FAIL_TOUCH_SLOPE:            TOUCH_I("LPWG FAIL REASON = Touch Slope\n");                break;
				case FAIL_MULTI_TOUCH:            TOUCH_I("LPWG FAIL REASON = Multi Touch\n");                break;
				case FAIL_LONG_PRESS:             TOUCH_I("LPWG FAIL REASON = Long Press\n");                 break;
				case FAIL_SWIPE_FINGER_RELEASE:   TOUCH_I("LPWG FAIL REASON = Swipe - Finger releas\n");      break;
				case FAIL_SWIPE_MULTIPLE_FINGERS: TOUCH_I("LPWG FAIL REASON = Swipe - Multiple Fingers\n");   break;
				case FAIL_SWIPE_TOO_FAST:         TOUCH_I("LPWG FAIL REASON = Swipe - Too Fast\n");           break;
				case FAIL_SWIPE_TOO_SLOW:         TOUCH_I("LPWG FAIL REASON = Swipe - Too Slow\n");           break;
				case FAIL_SWIPE_UPWARD:           TOUCH_I("LPWG FAIL REASON = Swipe - Upward\n");             break;
				case FAIL_SWIPE_RATIO_EXECESS:    TOUCH_I("LPWG FAIL REASON = Swipe - Ratio Execess\n");      break;
				default	:                         TOUCH_I("LPWG FAIL REASON = Unknown Reason\n");             break;
			}
			break;
		default:
			/* Re-enter nap mode */
			wbuf[0] = MIP_R0_CTRL;
			wbuf[1] = MIP_R1_CTRL_POWER_STATE;
			wbuf[2] = MIP_CTRL_POWER_LOW;
			if (mit300_reg_write(dev, wbuf, 3)) {
				TOUCH_E("%s [ERROR] mip_i2c_write\n", __func__);
				return -EIO;
			}
			break;
	}

	return 0;
}

static int mit300_irq_handler(struct device *dev)
{
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;
	int sz = 0;
	u8 alert_type = 0;
	u8 category = 0;

	TOUCH_TRACE();

	sz = mit_get_packet(dev, &category);
	if (sz == 0) {
		return 0;
	}
	if ((sz) < 0) {
		return -EIO;
	}

	if (category == 0) {
		mit_touch_event(dev, d->buf, sz);
	} else {
		alert_type = d->buf[0];
		TOUCH_I("%s - alert type [%d] sz[%d]\n", __func__, alert_type, sz);
		if(alert_type == MIP_ALERT_WAKEUP){
			if (mit_lpwg_event(dev, d->buf, sz) < 0)
				goto err_event_type;
		} else if(alert_type == MIP_ALERT_F1) {
			if (mit_fail_reason_event(dev, d->buf, sz) < 0)
				goto err_event_type;
		} else if(alert_type == MIP_ALERT_ESD){
			TOUCH_I("%s - MIP_ALERT_ESD\n", __func__);
			ret = -ERESTART;
		} else {
			goto err_event_type;
		}
	}

	return ret;

err_event_type:
	TOUCH_E("%s Unkown, event type(alert_type) 0x%x\n", __func__, alert_type);
	return -EIO;
}

static int mit300_power(struct device *dev, int ctrl)
{
	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		TOUCH_I("%s, off\n", __func__);
		break;

	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		break;

	case POWER_SLEEP:
		TOUCH_I("%s, sleep\n", __func__);
		break;

	case POWER_WAKE:
		TOUCH_I("%s, wake\n", __func__);
		break;
	}

	return 0;
}

static int mit300_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fw = NULL;
	char fwpath[256] = {'\0',};
	int ret = 0;
	int cut_ver = 0;
	cut_ver = lge_get_db7400_cut_ver();

	if(!ts->lpwg.screen) {
		TOUCH_I("If you want to upgrade firmware, please turn on the LCD.\n");
		return -EPERM;
	}

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath : %s\n", fwpath);
	} else if (ts->def_fwcnt) {
		if (cut_ver >= CUT7) {
			TOUCH_I("DDIC cut revision is CUT7!\n");
			memcpy(fwpath, ts->def_fwpath[2], sizeof(fwpath));
			fwpath[strlen(ts->def_fwpath[2])] = '\0';
		} else if (cut_ver == CUT6) {
			TOUCH_I("DDIC cut revision is CUT6!\n");
			memcpy(fwpath, ts->def_fwpath[1], sizeof(fwpath));
			fwpath[strlen(ts->def_fwpath[1])] = '\0';
		} else {
			TOUCH_I("DDIC cut revision is CUT5!\n");
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
			fwpath[strlen(ts->def_fwpath[0])] = '\0';
		}
		TOUCH_I("get fwpath from def_fwpath : %s\n", fwpath);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	if (fwpath == NULL) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);

	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n",
			fwpath, ret);

		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

    if(mit300_fw_compare(dev,fw)){
        ret = mit_isc_fwupdate(dev, fw);
    } else {
        release_firmware(fw);
        return -EPERM;
    }
	release_firmware(fw);

    mit300_ic_info(dev);

	return ret;
}

static int mit300_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int *value = (int *)param;

	TOUCH_TRACE();

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->tci.area.x1 = value[0];
		ts->tci.area.x2 = value[1];
		ts->tci.area.y1 = value[2];
		ts->tci.area.y2 = value[3];
		TOUCH_I("LPWG_ACTIVE_AREA: x0[%d], x1[%d], x2[%d], x3[%d]\n",
			value[0], value[1], value[2], value[3]);
		if ( ts->lpwg.screen == 0 ) {
			mit300_tci_active_area(dev);
		} else {
			TOUCH_I("DO NOT SET LPWG_ACTIVE_AREA because screen is ON\n");
		}
		break;

	case LPWG_TAP_COUNT:
		ts->tci.info[TCI_2].tap_count = value[0];
		TOUCH_I("LPWG_TAP_COUNT: ts->tci.info[TCI_2].tap_count = %d\n", value[0]);
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		ts->tci.double_tap_check = value[0];
		TOUCH_I("LPWG_DOUBLE_TAP_CHECK: ts->tci.double_tap_check = %d\n", value[0]);
		break;

	case LPWG_UPDATE_ALL:
		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];
		TOUCH_I(
			"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			ts->lpwg.mode,
			ts->lpwg.screen ? "ON" : "OFF",
			ts->lpwg.sensor ? "FAR" : "NEAR",
			ts->lpwg.qcover ? "CLOSE" : "OPEN");
		mit300_lpwg_mode(dev);
		change_cover_func(dev);
		break;
	}

	return 0;
}

static int mit300_notify(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int on_off;
	int ret =0;

	TOUCH_TRACE();

	switch(event) {
		case NOTIFY_TOUCH_RESET:
			on_off = *(int *)data;
			if(!on_off) {
				touch_disable_irq(ts->irq);
				touch_gpio_direction_output(ts->reset_pin, on_off);
				touch_msleep(2);
				TOUCH_I("%s\n", on_off ? "power: reset_pin high" : "power: reset_pin low");
			} else {
				touch_gpio_direction_output(ts->reset_pin, on_off);
				touch_msleep(50);
				touch_enable_irq(ts->irq);
				TOUCH_I("%s\n", on_off ? "power: reset_pin high" : "power: reset_pin low");
				change_cover_func(dev);
			}
			ret = NOTIFY_STOP;
			break;

		default:
			break;
	}
	return ret;
}


static ssize_t mit_fw_dump_show(struct device *dev, char *buf)
{
	int len = 0;
	u8 *pDump = NULL;
	int readsize = 0;
	int addr = 0;
	int retrycnt = 0;
	int fd = 0;
	char *dump_path = "/sdcard/touch_dump.fw";
	mm_segment_t old_fs = get_fs();

	TOUCH_I("F/W Dumping... \n");

	touch_interrupt_control(dev, INTERRUPT_DISABLE);

	pDump = kzalloc(FW_MAX_SIZE, GFP_KERNEL);

RETRY :
	readsize = 0;
	retrycnt++;
	mit_power_reset(dev);
	touch_msleep(50);

	for (addr = 0; addr < FW_MAX_SIZE; addr += FW_BLOCK_SIZE ) {
		if ( mip_isc_read_page(dev, addr, &pDump[addr]) ) {
			TOUCH_E("F/W Read failed \n");
			if (retrycnt > 10) {
				len += snprintf(buf + len, PAGE_SIZE - len, "dump failed \n");
				goto EXIT;
			} else
				goto RETRY;
		}

		readsize += FW_BLOCK_SIZE;
		if (readsize % (FW_BLOCK_SIZE * 20) == 0) {
			TOUCH_I("\t Dump %5d / %5d bytes\n", readsize, FW_MAX_SIZE);
		}
	}

	TOUCH_I("\t Dump %5d / %5d bytes\n", readsize, FW_MAX_SIZE);

	set_fs(KERNEL_DS);
	fd = sys_open(dump_path, O_WRONLY|O_CREAT, 0666);
	if (fd >= 0) {
		sys_write(fd, pDump, FW_MAX_SIZE);
		sys_close(fd);
		len += snprintf(buf + len, PAGE_SIZE - len, "%s saved \n", dump_path);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "%s open failed \n", dump_path);
	}

	set_fs(old_fs);

EXIT :
	kfree(pDump);

	mip_isc_exit(dev);

	mit_power_reset(dev);

	touch_interrupt_control(dev, INTERRUPT_ENABLE);

	return len;
}

static int mit_get_lpwg_lcd_status(struct device *dev)
{
	struct mit_data *d = to_mit_data(dev);
	u8 wbuf[2] = {0};
	u8 rbuf[1] = {0};

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_LCD_STATUS;
	if (mit300_reg_read(dev, wbuf, 2, rbuf, 1)) {
		TOUCH_E("[ERROR] read LCD Status Register\n");
		return -EIO;
	}
	d->lpwg_lcd_status = rbuf[0];
	return 0;
}

static ssize_t show_pen_support(struct device *dev, char *buf)
{
	int pen_support = 0;   /* 1: Support , 0: Not support */
	int ret = 0;

	pen_support = 1;

	TOUCH_I("Read Pen Support : %d\n", pen_support);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", pen_support);

	return ret;
}

static ssize_t show_lpwg_debug(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;
	ret = sprintf(buf, "lpwg_debug_enable : [%d]\n", d->lpwg_debug_enable);

	return ret;
}

static ssize_t store_lpwg_debug(struct device *dev, const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	int cmd = 0;

	sscanf(buf, "%d", &cmd);

	mutex_lock(&ts->lock);

	d->lpwg_debug_enable = cmd;
	tci_control(dev, LPWG_PANEL_DEBUG_CTRL, d->lpwg_debug_enable);

	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t show_lpwg_fail_reason(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;
	ret = sprintf(buf, "lpwg_fail_reason : [%d]\n", d->lpwg_fail_reason);
	return ret;
}

static ssize_t store_lpwg_fail_reason(struct device *dev, const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	int cmd = 0;

	sscanf(buf, "%d", &cmd);

	mutex_lock(&ts->lock);

	d->lpwg_fail_reason = cmd;
	tci_control(dev, LPWG_FAIL_REASON_CTRL, d->lpwg_fail_reason);

	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t show_lpwg_lcd_status(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;

	mutex_lock(&ts->lock);

	mit_get_lpwg_lcd_status(dev);

	mutex_unlock(&ts->lock);

	ret = sprintf(buf, "lpwg LCD status : [%d]\n", d->lpwg_lcd_status);

	return ret;
}

static ssize_t show_fw_dump(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	mutex_lock(&ts->lock);

	mit_fw_dump_show(dev, buf);

	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t store_reg_ctrl(struct device *dev,
				const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int reg_addr[2] = {0};
	char command[6] = {0};
	int value = 0;
	u8 write_buf[50] = {0};
	u8 read_buf[50] = {0};
	int i = 0;
	int len = 2;

	if ( sscanf(buf, "%5s %x %x %d", command, &reg_addr[0], &reg_addr[1], &value) != 4) {
		TOUCH_I("data parsing fail.\n");
		return -EINVAL;
	}
	TOUCH_I("%s, 0x%x, 0x%x, %d\n", command, reg_addr[0], reg_addr[1], value);

	mutex_lock(&ts->lock);

	if (!strcmp(command, "read")) {
			write_buf[0] = reg_addr[0];
			write_buf[1] = reg_addr[1];
			if(mit300_reg_read(dev, write_buf, len, read_buf, value) )
			{
				TOUCH_E("store_reg_control failed\n");
			}
			TOUCH_I("address=[0x%x%x]\n",write_buf[0], write_buf[1]);

			for (i = 0; i < value; i ++) {
				TOUCH_I("read_buf=[%02x]\n",read_buf[i]);
			}
	} else if (!strcmp(command, "write")) {
			write_buf[0] = reg_addr[0];
			write_buf[1] = reg_addr[1];
			if (value >= 256) {
				write_buf[2] = (value & 0xFF);
				write_buf[3] = (value >> 8);
				len = len + 2;
			} else {
				write_buf[2] = value;
				len++;
			}

			if (mit300_reg_write(dev, write_buf, len) < 0 ) {
				TOUCH_E("store_reg_control failed\n");
			}
	} else {
		TOUCH_I("usage: echo [read|write], [reg address0], [reg address1], [length(read)|value(write)] > reg_control\n");
	}

	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t store_use_quick_window(struct device *dev, const char *buf, size_t count)
{
	struct mit_data *d = to_mit_data(dev);
	int value = 0;

	sscanf(buf, "%d", &value);

	if ((value == 1) && (d->use_quick_window == COVER_SETTING_OFF)) {
		d->use_quick_window = COVER_SETTING_ON;
	} else if ((value == 0) && (d->use_quick_window == COVER_SETTING_ON)) {
		d->use_quick_window = COVER_SETTING_OFF;
	} else {
		return count;
	}

	TOUCH_I("quick cover setting = %s\n", (d->use_quick_window == COVER_SETTING_ON) ? "ON" : "OFF");

	return count;
}

static ssize_t store_use_excel(struct device *dev, const char *buf, size_t count)
{
	struct mit_data *d = to_mit_data(dev);
	int cmd = 0;

	sscanf(buf, "%d", &cmd);

	d->use_excel = cmd;
	TOUCH_I("use_excel  = %s\n", (d->use_excel == USE_STORE_EXCEL) ? "USE" : "UNUSE");

	return count;
}

static ssize_t show_use_excel(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int ret = 0;

	ret = sprintf(buf, "use_excel status : [%d]\n", d->use_excel);
	TOUCH_I("use_excel  = %s\n", (d->use_excel == USE_STORE_EXCEL) ? "USE" : "UNUSE");

	return ret;
}

static TOUCH_ATTR(pen_support, show_pen_support, NULL);
static TOUCH_ATTR(lpwg_debug, show_lpwg_debug, store_lpwg_debug);
static TOUCH_ATTR(lpwg_fail_reason, show_lpwg_fail_reason, store_lpwg_fail_reason);
static TOUCH_ATTR(lpwg_lcd_status, show_lpwg_lcd_status, NULL);
static TOUCH_ATTR(fw_dump, show_fw_dump, NULL);
static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(use_quick_window, NULL, store_use_quick_window);
static TOUCH_ATTR(use_excel, show_use_excel, store_use_excel);

static struct attribute *mit300_attribute_list[] = {
	&touch_attr_pen_support.attr,
	&touch_attr_lpwg_debug.attr,
	&touch_attr_lpwg_fail_reason.attr,
	&touch_attr_lpwg_lcd_status.attr,
	&touch_attr_fw_dump.attr,
	&touch_attr_reg_ctrl.attr,
	&touch_attr_use_quick_window.attr,
	&touch_attr_use_excel.attr,
	NULL,
};

static const struct attribute_group mit300_attribute_group = {
	.attrs = mit300_attribute_list,
};

static int mit300_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &mit300_attribute_group);
	if (ret < 0)
		TOUCH_E("mit300 sysfs register failed\n");

	mit300_prd_register_sysfs(dev);

	return 0;
}

static int mit300_get_cmd_version(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int offset = 0;
	int ret = 0;

	ret = mit300_ic_info(dev);

	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset += sprintf(buf + offset, "======== IC Firmware Info ========\n");
	offset += sprintf(buf + offset, "F/W Version : %X.%02X \n", d->module.version[0], d->module.version[1]);
	offset += sprintf(buf + offset, "F/W Product : %s \n", d->module.product_code);
	offset += sprintf(buf + offset, "F/W Row : %d, Col : %d\n", d->dev.row_num, d->dev.col_num);
	offset += sprintf(buf + offset, "IC Name : [%s] \n", d->module.ic_name);
	offset += sprintf(buf + offset, "\n======== BIN Firmware Info ========\n");
	offset += sprintf(buf + offset, "FW Version: %X.%02X\n", d->module.bin_version[0], d->module.bin_version[1]);
	offset += sprintf(buf + offset, "IC Name : %s \n", d->module.bin_chip_name);

	return offset;
}

static int mit300_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct mit_data *d = to_mit_data(dev);
	int offset = 0;
	int ret = 0;

	ret = mit300_ic_info(dev);

	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = sprintf(buf + offset, "v%X.%02X \n", d->module.version[0], d->module.version[1]);

	return offset;
}

static int mit300_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int mit300_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = mit300_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = mit300_get_cmd_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}

struct touch_driver touch_driver = {
	.probe = mit300_probe,
	.remove = mit300_remove,
	.suspend = mit300_suspend,
	.resume = mit300_resume,
	.init = mit300_init,
	.irq_handler = mit300_irq_handler,
	.power = mit300_power,
	.upgrade = mit300_upgrade,
	.lpwg = mit300_lpwg,
	.notify = mit300_notify,
	.register_sysfs = mit300_register_sysfs,
	.set = mit300_set,
	.get = mit300_get,
};

#define MATCH_NAME	"melfas,mit300"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = touch_match_ids,
};

static int __init touch_device_init(void)
{
	TOUCH_TRACE();
	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);
