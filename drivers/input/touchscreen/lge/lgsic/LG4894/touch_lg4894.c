/* touch_lg4894.c
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <soc/mediatek/lge/board_lge.h>
/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

/*
 *  Include to Local Header File
 */
#include "touch_lg4894.h"
#include "touch_lg4894_abt.h"
#include "touch_lg4894_prd.h"

// Definitions for Debugging Failure Reason in LPWG
enum {
	TCI_DEBUG_DISABLE = 0,
	TCI_DEBUG_ALWAYS,
	TCI_DEBUG_BUFFER, // NA
	TCI_DEBUG_BUFFER_ALWAYS,  // NA
};

static const char *debug_type[] = {
	"Disable Type",
	"Always Report Type"
};
#define TCI_FAIL_NUM 10
static const char const *tci_debug_str[TCI_FAIL_NUM] = {
	"SUCESS",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"MINTIMEOUT_INTER_TAP",
	"MAXTIMEOUT_INTER_TAP",
	"LONGPRESS_TIME_OUT",
	"MULTI_FINGER",
	"DELAY_TIME",/* It means Over Tap */
	"PALM_STATE",
	"OUTOF_AREA",
};

#if 1
#define SWIPE_FAIL_NUM 11
static const char const *swipe_debug_str[SWIPE_FAIL_NUM] = {
	"ERROR",
	"FINGER_FAST_RELEASE",
	"MULTI_FINGER",
	"FAST_SWIPE",
	"SLOW_SWIPE",
	"WRONG_DIRECTION",
	"RATIO_FAIL",
	"OUT_OF_START_AREA",
	"OUT_OF_ACTIVE_AREA",
	"INITIAL_RATIO_FAIL",
	"PALM_STATE",
};
#endif

#define IC_DEBUG_INFO_NUM 15
static const char *ic_debug_info_str[] = {
	[0] = "NONE",
	[1] = "DBG_TG_FAULT",
	[2] = "DBG_ESD_FAULT",
	[3] = "DBG_WATDOG_TIMEOUT",
	[4] = "DBG_TC_DRV_MISMATCH",
	[5] = "DBG_TC_INVALID_TIME_DRV_REQ",
	[6] = "DBG_AFE_TUNE_FAIL",
	[7] = "DBG_DBG_MSG_FULL",
	[8] = "DBG_PRE_MA_OVF_ERR",
	[9] = "DBG_ADC_OVF_ERR",
	[10] = "DBG_CM3_FAULT",
	[11] = "DBG_DBG_TC_DRV_START_SKIP",
	[12] = "DBG_DBG_TEST_MSG1",
	[13] = "DBG_DBG_TEST_MSG2",
	[14] = "DBG_DBG_TEST_MSG3",
};

#ifdef LG4894_TCL_OFF_VIA_MIPI
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
extern int set_touch_osc(int enable);
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
extern int primary_display_set_touch_osc(int enable);
#endif
#endif

static int lg4894_condition_wait(struct device *dev,
				    u16 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry);

#if defined(CONFIG_LGE_MODULE_DETECT)
int lg4894_panel_type;
#endif /* CONFIG_LGE_MODULE_DETECT */

#ifdef LG4894_ESD_SKIP_WHILE_TOUCH_ON
static int finger_cnt = 0;

bool lg4894_check_finger(void)
{
	return finger_cnt==0? false:true;
}
EXPORT_SYMBOL(lg4894_check_finger);
#endif

int lg4894_xfer_msg(struct device *dev, struct touch_xfer_msg *xfer)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	struct touch_xfer_data_t *tx = NULL;
	struct touch_xfer_data_t *rx = NULL;
	int ret = 0;
	int i = 0;

	mutex_lock(&d->spi_lock);

	for (i = 0; i < xfer->msg_count; i++) {
		tx = &xfer->data[i].tx;
		rx = &xfer->data[i].rx;

		if (rx->size) {
			tx->data[0] = ((rx->size > 4) ? 0x20 : 0x00);
			tx->data[0] |= ((rx->addr >> 8) & 0x0f);
			tx->data[1] = (rx->addr & 0xff);
			tx->data[2] = 0;
			tx->data[3] = 0;
			rx->size += R_HEADER_SIZE;
		} else {
			if (tx->size > (MAX_XFER_BUF_SIZE - W_HEADER_SIZE)) {
				TOUCH_E("buffer overflow\n");
				mutex_unlock(&d->spi_lock);
				return -EOVERFLOW;
			}

			tx->data[0] = ((tx->size == 1) ? 0x60 : 0x40);
			tx->data[0] |= ((tx->addr >> 8) & 0x0f);
			tx->data[1] = (tx->addr  & 0xff);
			memcpy(&tx->data[W_HEADER_SIZE], tx->buf, tx->size);
			tx->size += W_HEADER_SIZE;
		}
	}

	ret = touch_bus_xfer(dev, xfer);
	if (ret) {
		TOUCH_E("touch bus error : %d\n", ret);
		mutex_unlock(&d->spi_lock);
		return ret;
	}

	for (i = 0; i < xfer->msg_count; i++) {
		rx = &xfer->data[i].rx;

		if (rx->size) {
			memcpy(rx->buf, rx->data + R_HEADER_SIZE,
				(rx->size - R_HEADER_SIZE));
		}
	}

	mutex_unlock(&d->spi_lock);

	return 0;
}

int lg4894_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->spi_lock);
	ts->tx_buf[0] = ((size > 4) ? 0x20 : 0x00);
	ts->tx_buf[0] |= ((addr >> 8) & 0x0f);
	ts->tx_buf[1] = (addr & 0xff);
	//ts->tx_buf[2] = 0;
	//ts->tx_buf[3] = 0;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = W_HEADER_SIZE;
	msg.rx_buf = ts->rx_buf;
	msg.rx_size = R_HEADER_SIZE + size;
	//msg.bits_per_word = 8;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus error : %d\n", ret);
		mutex_unlock(&d->spi_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[R_HEADER_SIZE], size);
	mutex_unlock(&d->spi_lock);
	return 0;
}

int lg4894_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->spi_lock);
	ts->tx_buf[0] = ((size > 4) ? 0x60 : 0x40);
	ts->tx_buf[0] |= ((addr >> 8) & 0x0f);
	ts->tx_buf[1] = (addr  & 0xff);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = W_HEADER_SIZE + size;
	msg.rx_buf = NULL;
	msg.rx_size = 0;
	//msg.bits_per_word = 8;

	memcpy(&ts->tx_buf[W_HEADER_SIZE], data, size);

	ret = touch_bus_write(dev, &msg);
	mutex_unlock(&d->spi_lock);

	if (ret < 0) {
		TOUCH_E("touch bus error : %d\n", ret);
		return ret;
	}

	return 0;
}


static int lg4894_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			TOUCH_I("FB_UNBLANK\n");
		else if (*blank == FB_BLANK_POWERDOWN)
			TOUCH_I("FB_BLANK\n");
	}

	return 0;
}
#if 0 //no use 4894
static int lg4894_cmd_write(struct device *dev, u8 cmd)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	struct touch_bus_msg msg;
	u8 input[2] = {0, };
	int ret = 0;

	input[0] = cmd;
	input[1] = 0;

	msg.tx_buf = input;
	msg.tx_size = 2;

	msg.rx_buf = NULL;
	msg.rx_size = 0;

	mutex_lock(&d->spi_lock);

	ret = touch_bus_write(dev, &msg);

	mutex_unlock(&d->spi_lock);

	if (ret) {
		TOUCH_E("touch bus error : %d\n", ret);
		return ret;
	}

	return 0;
}
#endif

static int lg4894_sw_reset(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;
	/******************************************************
	* Siliconworks does not recommend to use SW reset    *
	* due to its limitation in stability in LG4894.      *
	******************************************************/
	TOUCH_I("%s : SW Reset\n", __func__);
	lg4894_write_value(dev, spr_rst_ctl, 7);
	lg4894_write_value(dev, spr_rst_ctl, 0);
	/* Boot Start */
	lg4894_write_value(dev, spr_boot_ctl, 1);
	/* firmware boot done check */
	ret = lg4894_condition_wait(dev, tc_flash_dn_sts, NULL,
				    FLASH_BOOTCHK_VALUE, 0xFFFFFFFF, 10, 200);
	if (ret < 0) {
		TOUCH_E("failed : \'boot check\'\n");
		return -EPERM;
	}
	atomic_set(&d->init, IC_INIT_NEED);
	queue_delayed_work(ts->wq, &ts->init_work,
				msecs_to_jiffies(ts->caps.sw_reset_delay));

	return ret;
}

int lg4894_hw_reset(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);

	TOUCH_I("%s : HW Reset(%d)\n", __func__, mode);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_msleep(1);

	touch_gpio_direction_output(ts->reset_pin, 1);
	atomic_set(&d->init, IC_INIT_NEED);

	if (mode == HW_RESET_ASYNC){
		queue_delayed_work(ts->wq, &ts->init_work,
				msecs_to_jiffies(ts->caps.hw_reset_delay));
	} else if(mode == HW_RESET_SYNC) {
		touch_msleep(ts->caps.hw_reset_delay);
		ts->driver->init(dev);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	} else {
		TOUCH_E("%s Invalid HW reset mode!!\n", __func__);
	}

	return 0;
}

int lg4894_reset_ctrl(struct device *dev, int ctrl)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	//struct lg4894_data *d = to_lg4894_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
	default :
	case SW_RESET:
		lg4894_sw_reset(dev);
		break;

	case HW_RESET_ASYNC:
	case HW_RESET_SYNC:
		lg4894_hw_reset(dev, ctrl);
		break;
	}

	//atomic_set(&d->watch.state.rtc_status, RTC_CLEAR);

	return 0;
}

static int lg4894_power(struct device *dev, int ctrl)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	//struct lg4894_data *d = to_lg4894_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		TOUCH_I("%s, off\n", __func__);
		//touch_gpio_direction_output(ts->reset_pin, 0);
		//touch_power_vio(dev, 0);
		//touch_power_vdd(dev, 0);
		//touch_msleep(1);
		//atomic_set(&d->watch.state.font_status, FONT_EMPTY);
		break;

	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		//touch_power_vdd(dev, 1);
		//touch_power_vio(dev, 1);
		//touch_gpio_direction_output(ts->reset_pin, 1);
		break;


	case POWER_SLEEP:
		TOUCH_I("%s, sleep\n", __func__);
		break;

	case POWER_WAKE:
		TOUCH_I("%s, wake\n", __func__);
		break;
	case POWER_HW_RESET:
		TOUCH_I("%s, HW reset\n", __func__);
		lg4894_reset_ctrl(dev, HW_RESET_ASYNC);
		break;
	case POWER_SW_RESET:
		TOUCH_I("%s, SW reset\n", __func__);
		lg4894_reset_ctrl(dev, SW_RESET);
		break;
	}

	return 0;
}

static void lg4894_get_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 6;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 100;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 6;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 100;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 20;
}

static void lg4894_get_swipe_info(struct device *dev)
{
	struct lg4894_data *d = to_lg4894_data(dev);

	d->swipe.info.distance = 15; //20mm -> 15mm
	d->swipe.info.ratio_thres = 150;
	d->swipe.info.min_time = 4;
	d->swipe.info.max_time = 150;
	d->swipe.info.area.x1 = 0; /* 401 */;
	d->swipe.info.area.y1 = 0;
	d->swipe.info.area.x2 = 719;
	d->swipe.info.area.y2 = 1279;
	d->swipe.info.start.x1 = 230;
	d->swipe.info.start.y1 = 1061; //20mm
	d->swipe.info.start.x2 = 490;
	d->swipe.info.start.y2 = 1279;
	d->swipe.info.wrong_dir_thes = 5;
	d->swipe.info.init_rat_chk_dist = 4;
	d->swipe.info.init_rat_thres = 100;

	d->swipe.mode = 0;
}

int lg4894_ic_info(struct device *dev)
{
	struct lg4894_data *d = to_lg4894_data(dev);

	int ret = 0;
	u32 version = 0;
	u32 revision = 0;
	u32 bootmode = 0;
	u32 lcdrevision = 0;
	u32 product[2] = {0};
	char rev_str[32] = {0};

	ret = lg4894_reg_read(dev, tc_version, &version, sizeof(version));
	if (ret < 0) {
		TOUCH_D(BASE_INFO, "version : %x\n", version);
		return ret;
	}

	ret = lg4894_reg_read(dev, info_chip_revision, &revision, sizeof(revision));
	ret = lg4894_reg_read(dev, tc_product_id1, &product[0], sizeof(product));
	ret = lg4894_reg_read(dev, spr_boot_st, &bootmode, sizeof(bootmode));
	ret = lg4894_reg_read(dev, info_lcd_revision, &lcdrevision, sizeof(lcdrevision));

	d->fw.version[0] = ((version >> 8) & 0xFF);
	d->fw.version[1] = version & 0xFF;
	d->fw.revision = revision & 0xFF;
	d->fw.lcd_fpcb_revision = lcdrevision;
	memcpy(&d->fw.product_id[0], &product[0], sizeof(product));

	if (d->fw.revision == 0xFF)
		snprintf(rev_str, 32, "revision: Flash Erased(0xFF)");
	else
		snprintf(rev_str, 32, "revision: %d", d->fw.revision);

	TOUCH_I("version : v%d.%02d, chip : %d, protocol : %d\n"
		,d->fw.version[0], d->fw.version[1], (version >> 16) & 0xFF, (version >> 24) & 0xFF);
	TOUCH_I("%s\n", rev_str);
	TOUCH_I("product id : %s\n", d->fw.product_id);
	TOUCH_I("flash boot : %s, %s, crc : %s\n",
		(bootmode >> 1 & 0x1) ? "BUSY" : "idle",
		(bootmode >> 2 & 0x1) ? "done" : "booting",
		(bootmode >> 3 & 0x1) ? "ERROR" : "ok");

	TOUCH_D(BASE_INFO, "lcd fpcb revision : %d\n", d->fw.lcd_fpcb_revision);

	if ((((version >> 16) & 0xFF) != 4) || (((version >> 24) & 0xFF) != 4)) {
		TOUCH_I("FW is in abnormal state because of ESD or something.\n");
		lg4894_reset_ctrl(dev, HW_RESET_ASYNC);
	}

	return ret;
}

static int lg4894_get_tci_data(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	u8 i = 0;
	u32 rdata[MAX_LPWG_CODE];

	if (!count)
		return 0;

	ts->lpwg.code_num = count;

	memcpy(&rdata, d->info.data, sizeof(u32) * count);

	for (i = 0; i < count; i++) {
		ts->lpwg.code[i].x = rdata[i] & 0xffff;
		ts->lpwg.code[i].y = (rdata[i] >> 16) & 0xffff;

		if (ts->lpwg.mode >= LPWG_PASSWORD)
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n",
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static int lg4894_get_swipe_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	u32 rdata[3];
	int count = 1;

	/* swipe_info */
	/* start (X, Y), end (X, Y), time = 2bytes * 5 = 10 bytes */
	memcpy(&rdata, d->info.data, sizeof(u32) * 3);

	TOUCH_I("Swipe Gesture: start(%4d,%4d) end(%4d,%4d) swipe_time(%dms)\n",
			rdata[0] & 0xffff, rdata[0] >> 16,
			rdata[1] & 0xffff, rdata[1] >> 16,
			rdata[2] & 0xffff);

	ts->lpwg.code_num = count;
	ts->lpwg.code[0].x = rdata[1] & 0xffff;
	ts->lpwg.code[0].x = rdata[1]  >> 16;

	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static void set_debug_reason(struct device *dev, int type)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	u32 wdata = 0;
	//u32 wdata1 = 0;

	if(type == SWIPE){
		wdata = d->swipe_debug_type;
		TOUCH_I("SWIPE debug-type:%d\n", wdata);
	} else {
		wdata = d->tci_debug_type;
		TOUCH_I("TCI%d debug-type:%d\n", type + 1, wdata);
	}

	lg4894_reg_write(dev, LPWG_DEBUG_CTRL, &wdata, sizeof(wdata));

	return;
}

static int lg4894_tci_knock(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u32 lpwg_data[7];

	if ((d->tci_debug_type != 0) && (ts->lpwg.mode==LPWG_DOUBLE_TAP))
		set_debug_reason(dev, TCI_1);

	lpwg_data[0] = ts->tci.mode;
	lpwg_data[1] = info1->tap_count | (info2->tap_count << 16);
	lpwg_data[2] = info1->min_intertap | (info2->min_intertap << 16);
	lpwg_data[3] = info1->max_intertap | (info2->max_intertap << 16);
	lpwg_data[4] = info1->touch_slop | (info2->touch_slop << 16);
	lpwg_data[5] = info1->tap_distance | (info2->tap_distance << 16);
	lpwg_data[6] = info1->intr_delay | (info2->intr_delay << 16);

	return lg4894_reg_write(dev, TCI_ENABLE_W,
			&lpwg_data[0], sizeof(lpwg_data));
}

static int lg4894_tci_password(struct device *dev)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	if (d->tci_debug_type != 0)
		set_debug_reason(dev, TCI_2);

	return lg4894_tci_knock(dev);
}

static int lg4894_tci_active_area(struct device *dev,
		u32 x1, u32 y1, u32 x2, u32 y2)
{
	int ret = 0, i;
	u32 active_area[4] = {x1+ACT_SENSELESS_AREA_W, y1+ACT_SENSELESS_AREA_W, \
						  x2-ACT_SENSELESS_AREA_W, y2-ACT_SENSELESS_AREA_W};

	TOUCH_I("%s: x1[%d], y1[%d], x2[%d], y2[%d]\n", __func__,
				active_area[0], active_area[1], active_area[2], active_area[3]);

	for (i=0; i < sizeof(active_area)/sizeof(u32); i++)
		active_area[i] = (active_area[i]) | (active_area[i] << 16);

	ret = lg4894_reg_write(dev, ACT_AREA_X1_W,
			&active_area[0], sizeof(u32));
	ret = lg4894_reg_write(dev, ACT_AREA_Y1_W,
			&active_area[1], sizeof(u32));
	ret = lg4894_reg_write(dev, ACT_AREA_X2_W,
			&active_area[2], sizeof(u32));
	ret = lg4894_reg_write(dev, ACT_AREA_Y2_W,
			&active_area[3], sizeof(u32));

	return ret;
}

static int lg4894_tci_control(struct device *dev, int type)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u32 lpwg_data;
	int ret = 0;

	switch (type) {
	case ENABLE_CTRL:
		lpwg_data = ts->tci.mode;
		ret = lg4894_reg_write(dev, TCI_ENABLE_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case TAP_COUNT_CTRL:
		lpwg_data = info1->tap_count | (info2->tap_count << 16);
		ret = lg4894_reg_write(dev, TAP_COUNT_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case MIN_INTERTAP_CTRL:
		lpwg_data = info1->min_intertap | (info2->min_intertap << 16);
		ret = lg4894_reg_write(dev, MIN_INTERTAP_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case MAX_INTERTAP_CTRL:
		lpwg_data = info1->max_intertap | (info2->max_intertap << 16);
		ret = lg4894_reg_write(dev, MAX_INTERTAP_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case TOUCH_SLOP_CTRL:
		lpwg_data = info1->touch_slop | (info2->touch_slop << 16);
		ret = lg4894_reg_write(dev, TOUCH_SLOP_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case TAP_DISTANCE_CTRL:
		lpwg_data = info1->tap_distance | (info2->tap_distance << 16);
		ret = lg4894_reg_write(dev, TAP_DISTANCE_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case INTERRUPT_DELAY_CTRL:
		lpwg_data = info1->intr_delay | (info2->intr_delay << 16);
		ret = lg4894_reg_write(dev, INT_DELAY_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case ACTIVE_AREA_CTRL:
		ret = lg4894_tci_active_area(dev, 0, 0, 720, 1280); /* 720 x 1280 */
		/*
				ts->tci.area.x1,
				ts->tci.area.y1,
				ts->tci.area.x2,
				ts->tci.area.y2);
		*/
		break;

	case ACTIVE_AREA_RESET_CTRL:
		/*
		ret = lg4894_tci_active_area(dev,
				(65 | 65 << 16),
				(1374 | 1374 << 16),
				(65 | 65 << 16),
				(2494 | 2494 << 16));
		*/
		break;

	default:
		break;
	}

	return ret;
}

static int lg4894_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	int ret = 0;

#ifdef LG4894_TCL_OFF_VIA_MIPI
	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("Not Ready, Need to turn on clock\n");
		return 0;
	}
#endif

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		info1->intr_delay = 0;
		info1->tap_distance = 10;

		ret = lg4894_tci_control(dev, ACTIVE_AREA_CTRL);
		ret = lg4894_tci_knock(dev);
		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x01 | (0x01 << 16);
		info1->intr_delay = ts->tci.double_tap_check ? 68 : 0;
		info1->tap_distance = 7;

		ret = lg4894_tci_control(dev, ACTIVE_AREA_CTRL);
		ret = lg4894_tci_password(dev);
		break;

	case LPWG_PASSWORD_ONLY:
		TOUCH_I("lg4894_lpwg_control LPWG_PASSWORD_ONLY\n");
		ts->tci.mode = 0x01 << 16;
		info1->intr_delay = 0;
		info1->tap_distance = 10;

		ret = lg4894_tci_control(dev, ACTIVE_AREA_CTRL);
		ret = lg4894_tci_password(dev);
		break;

	default:
		ts->tci.mode = 0;
		ret = lg4894_tci_control(dev, ENABLE_CTRL);
		break;
	}

	TOUCH_I("lg4894_lpwg_control mode = %d\n", mode);

	return ret;
}

static int lg4894_swipe_active_area(struct device *dev)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	struct lg4894_swipe_info *up = &d->swipe.info;

	u32 active_area[4] = {0x0, };
	int ret = 0;

	active_area[0] = up->area.x1;
	active_area[1] = up->area.y1;
	active_area[2] = up->area.x2;
	active_area[3] = up->area.y2;

	ret = lg4894_reg_write(dev, SWIPE_ACT_AREA_X1_W,
			active_area, sizeof(active_area));

	return ret;
}

static int lg4894_swipe_control(struct device *dev, int type)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	struct lg4894_swipe_info *up = &d->swipe.info;
	u32 swipe_data = 0;
	int ret = 0;

	switch (type) {
	case SWIPE_ENABLE_CTRL:
		swipe_data = d->swipe.mode;
		ret = lg4894_reg_write(dev, SWIPE_ENABLE_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_DISABLE_CTRL:
		swipe_data = 0;
		ret = lg4894_reg_write(dev, SWIPE_ENABLE_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_DIST_CTRL:
		swipe_data = up->distance;
		ret = lg4894_reg_write(dev, SWIPE_DIST_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_RATIO_THR_CTRL:
		swipe_data = up->ratio_thres;
		ret = lg4894_reg_write(dev, SWIPE_RATIO_THR_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_RATIO_PERIOD_CTRL:
//		swipe_data = (right->ratio_period) | (left->ratio_period << 16);
//		ret = lg4894_reg_write(dev, SWIPE_RATIO_PERIOD_W,
//				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_RATIO_DIST_CTRL:
//		swipe_data = (right->ratio_distance) |
//				(left->ratio_distance << 16);
//		ret = lg4894_reg_write(dev, SWIPE_RATIO_DIST_W,
//				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_TIME_MIN_CTRL:
		swipe_data = up->min_time;
		ret = lg4894_reg_write(dev, SWIPE_TIME_MIN_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_TIME_MAX_CTRL:
		swipe_data = up->max_time;
		ret = lg4894_reg_write(dev, SWIPE_TIME_MAX_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_AREA_CTRL:
		ret = lg4894_swipe_active_area(dev);
		break;
	default:
		break;
	}

	return ret;
}

static int lg4894_swipe_mode(struct device *dev, u8 lcd_mode)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	struct lg4894_swipe_info *up = &d->swipe.info;
	u32 swipe_data[16] = {0x0, };
	int ret = 0;

	if (!d->swipe.mode)
		return ret;

	if (lcd_mode == LCD_MODE_U3) {
		ret = lg4894_swipe_control(dev, SWIPE_DISABLE_CTRL);
		TOUCH_I("swipe disable\n");
	} else {
		TOUCH_I("swipe mode is [%d]\n", d->swipe.mode);
		swipe_data[0] = d->swipe.mode;
		swipe_data[1] = up->distance;
		swipe_data[2] = up->ratio_thres;
		swipe_data[3] = up->min_time;
		swipe_data[4] = up->max_time;

		swipe_data[5] = up->area.x1;
		swipe_data[6] = up->area.y1;
		swipe_data[7] = up->area.x2;
		swipe_data[8] = up->area.y2;

		swipe_data[9] = up->start.x1;
		swipe_data[10] = up->start.y1;
		swipe_data[11] = up->start.x2;
		swipe_data[12] = up->start.y2;

		swipe_data[13] = up->wrong_dir_thes;
		swipe_data[14] = up->init_rat_chk_dist;
		swipe_data[15] = up->init_rat_thres;

		ret = lg4894_reg_write(dev, SWIPE_ENABLE_W,
			&swipe_data[0], sizeof(swipe_data));

		if ((d->swipe_debug_type != 0))
			set_debug_reason(dev, SWIPE);

		TOUCH_I("swipe enable\n");
	}

	return ret;
}


static int lg4894_clock(struct device *dev, bool onoff)
{
#ifdef LG4894_TCL_OFF_VIA_MIPI
	struct touch_core_data *ts = to_touch_core(dev);

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
	set_touch_osc(onoff); //0 : osc off / 1 : osc on
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
	int status = atomic_read(&ts->state.sleep);

	/*********************************************************
	 * Add exception control for the scene proximity updates *
	 * repeatedly in a short period of time.                 *
	 *********************************************************/
	 if ((primary_display_changing | primary_display_esd_checking) || (status ^ onoff)) {
		TOUCH_I("skip oscillator control(ic_status=%s, oscillator=%s,lcd_mode=%d)\n",
					status? "DEEP_SLEEP":"NORMAL", onoff? "on":"off",
					primary_display_changing | primary_display_esd_checking);
		return 0;
	}

	primary_display_set_touch_osc(onoff); //0 : osc off / 1 : osc on
#endif
	if (onoff) {
		touch_msleep(60);
		atomic_set(&ts->state.sleep, IC_NORMAL);
	} else {
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
	}
	TOUCH_I("set_touch_osc = %s\n", (onoff == 0) ? "0 (disable)" : "1 (enable)");
#else
	/* Lg4894 needs to control touch oscillator clock via MIPI script. */
#endif
	return 0;
}

int lg4894_tc_driving(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u32 ctrl = 0;
	u8 rdata;

#ifdef LG4894_TCL_OFF_VIA_MIPI
	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("Not Ready, Need to turn on clock\n");
		return 0;
	}
#endif

	switch (mode) {
	case LCD_MODE_U0:
		ctrl = 0x01;
		break;

	case LCD_MODE_U2:
		return 0;
		ctrl = 0x101;
		break;

	case LCD_MODE_U3:
		if(atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_1)
			ctrl = 0x181;
		else
			ctrl = 0x185;
		break;

	case LCD_MODE_U3_PARTIAL:
		return 0;
		if(atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_1)
			ctrl = 0x381;
		else
			ctrl = 0x385;
		break;

	case LCD_MODE_STOP:
		ctrl = 0x02;
		break;
	}

	/* When quick cover is in CLOSE state, set 1 in 10th bit of tc_drive_ctl. */
	if (ts->lpwg.qcover == 1)
		ctrl |= (1 << 10);

	/* swipe set */
	lg4894_swipe_mode(dev, mode);

	TOUCH_I("lg4894_tc_driving = %d\n", mode);
	lg4894_reg_read(dev, spr_subdisp_st, (u8 *)&rdata, sizeof(u32));
	TOUCH_I("DDI Display Mode = %d\n", rdata);
	lg4894_reg_write(dev, tc_drive_ctl, &ctrl, sizeof(ctrl));
	touch_msleep(20);

	return 0;
}

static void lg4894_deep_sleep(struct device *dev)
{
	lg4894_tc_driving(dev, LCD_MODE_STOP);
	lg4894_clock(dev, 0);
}

static void lg4894_debug_tci(struct device *dev)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	u32 rdata = 0;
	u32 rdata1 = 0;
	u16 bit_mask = 0;

	if (!d->tci_debug_type)
		return;

	lg4894_reg_read(dev, LPWG_DEBUG_FAIL_STATUS, &rdata1, sizeof(rdata1));
	lg4894_reg_read(dev, TCI_DEBUG_FAILREASON_BUFFER, &rdata, sizeof(rdata));
	TOUCH_I("[LPWG_DEBUG_FAIL_STATUS=[0x%x] TCI_DEBUG_FAILREASON_BUFFER = [0x%x]\n", rdata1, rdata);
	if (rdata1&0x1) {
		bit_mask = (rdata & 0xffff);
		if (bit_mask < TCI_FAIL_NUM)
			TOUCH_I("[TCI_ON] TCI_DEBUG_FAILREASON_BUFFER = [0x%x]%s\n", rdata, tci_debug_str[bit_mask]);
	}

	if (rdata1&(0x1<<1)) {
		bit_mask = (rdata >> 16) & 0xffff;
		if (bit_mask < TCI_FAIL_NUM)
			TOUCH_I("[TCI_CODE] TCI_DEBUG_FAILREASON_BUFFER = [0x%x]%s\n", rdata, tci_debug_str[bit_mask]);
	}
}

static void lg4894_debug_swipe(struct device *dev)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	u32 rdata = 0;
	u32 rdata1 = 0;
	u16 bit_mask = 0;

	if (!d->swipe_debug_type)
		return;
	lg4894_reg_read(dev, LPWG_DEBUG_FAIL_STATUS, &rdata1, sizeof(rdata1));
	lg4894_reg_read(dev, SWIPE_DEBUG_FAILREASON_BUFFER, &rdata, sizeof(rdata));
	TOUCH_I("SWIPE_DEBUG_FAIL_STATUS=[0x%x] SWIPE_DEBUG_FAILREASON_BUFFER = [0x%x]\n", rdata1, rdata);

	if (rdata1&(0x1<<2)) {
		bit_mask = (rdata & 0xff);
		if (bit_mask < SWIPE_FAIL_NUM)
			TOUCH_I("[SWIPE] SWIPE_DEBUG_FAILREASON_BUFFER = [0x%x]%s\n", rdata, swipe_debug_str[bit_mask]);
	}
}


static int lg4894_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);

	if (atomic_read(&d->init) == IC_INIT_NEED) {
		TOUCH_I("Not Ready, Need IC init\n");
		return 0;
	}

#ifdef LG4894_TCL_OFF_VIA_MIPI
	if (atomic_read(&d->hw_reset) == LCD_EVENT_TOUCH_RESET_START) {
		TOUCH_I("Touch IC is being reset. skip lpwg mode update\n");
		return 0;
	}
#endif

	if (atomic_read(&d->skip_update) == 1) {
		TOUCH_I("Device in Resume/Suspend. Skip Update\n");
		return 0;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg) {
			lg4894_lpwg_control(dev, LPWG_DOUBLE_TAP);
			lg4894_tc_driving(dev, d->lcd_mode);
			return 0;
		}
		if (ts->lpwg.screen) {
			TOUCH_I("Skip lpwg_mode\n");
#ifdef LG4894_TCL_OFF_VIA_MIPI
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
				TOUCH_I("Not Ready, Need to turn on clock\n");
				return 0;
			}
#endif

#if 0 // not yet non RT debug
			lg4894_debug_tci(dev);
			lg4894_debug_swipe(dev);
#endif
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("suspend sensor == PROX_NEAR\n");
			if (atomic_read(&ts->state.sleep) != IC_DEEP_SLEEP){
				lg4894_deep_sleep(dev);
			}
		} else if (ts->lpwg.qcover == HALL_NEAR) {
			/* knock on/code disable */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				lg4894_clock(dev, 1);

			lg4894_lpwg_control(dev, LPWG_NONE);
			lg4894_tc_driving(dev, d->lcd_mode);
		} else {
			/* knock on/code */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				lg4894_clock(dev, 1);

			lg4894_lpwg_control(dev, ts->lpwg.mode);
			if (ts->lpwg.mode == LPWG_NONE && !d->swipe.mode &&
					d->lcd_mode == LCD_MODE_U0) {
				/* knock on/code disable */
				TOUCH_I("LCD_MODE_U0 - DeepSleep\n");
				lg4894_deep_sleep(dev);
			} else {
				lg4894_tc_driving(dev, d->lcd_mode);
			}
		}
		return 0;
	}

	/* resume */
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("resume ts->lpwg.screen\n");
		lg4894_lpwg_control(dev, LPWG_NONE);
		lg4894_tc_driving(dev, d->lcd_mode);
	}else if (ts->lpwg.sensor == PROX_NEAR) {
		TOUCH_I("resume ts->lpwg.sensor == PROX_NEAR\n");
		lg4894_tc_driving(dev, LCD_MODE_STOP);
	} else {
		/* partial */
		TOUCH_I("resume Partial-Do not set\n");
		/*
		if (ts->lpwg.qcover == HALL_NEAR)
			lg4894_lpwg_control(dev, LPWG_NONE);
		else
			lg4894_lpwg_control(dev, ts->lpwg.mode);
		lg4894_tc_driving(dev, LCD_MODE_U3_PARTIAL);
		*/
	}

	return 0;
}

static int lg4894_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
#ifdef LG4894_TCL_OFF_VIA_MIPI
	struct lg4894_data *d = to_lg4894_data(dev);
#endif
	int *value = (int *)param;

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->tci.area.x1 = value[0];
		ts->tci.area.x2 = value[1];
		ts->tci.area.y1 = value[2];
		ts->tci.area.y2 = value[3];
		TOUCH_I("LPWG_ACTIVE_AREA: x0[%d], x1[%d], x2[%d], x3[%d]\n",
			value[0], value[1], value[2], value[3]);
		break;

	case LPWG_TAP_COUNT:
		ts->tci.info[TCI_2].tap_count = value[0];
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		ts->tci.double_tap_check = value[0];
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
#ifdef LG4894_TCL_OFF_VIA_MIPI
		if (atomic_read(&d->init) == IC_INIT_NEED) {
			TOUCH_I("Skip lpwg update until IC is intialized!!\n");
			return 0;
		}
#endif
		lg4894_lpwg_mode(dev);

		break;

	case LPWG_REPLY:
		break;

	}

	return 0;
}

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
static void lg4894_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int charger_state = atomic_read(&ts->state.connect);
	int wireless_state = atomic_read(&ts->state.wireless);

	TOUCH_TRACE();

	d->charger = 0;
	/* wire */
	if (charger_state == CONNECT_INVALID)
		d->charger = CONNECT_NONE;
	else if ((charger_state == CONNECT_DCP)
			|| (charger_state == CONNECT_PROPRIETARY))
		d->charger = CONNECT_TA;
	else if (charger_state == CONNECT_HUB)
		d->charger = CONNECT_OTG;
	else
		d->charger = CONNECT_USB;

	/* wireless */
	if (wireless_state)
		d->charger = d->charger | CONNECT_WIRELESS;

	TOUCH_I("%s: write charger_state = 0x%02X\n", __func__, d->charger);
	if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
		TOUCH_I("DEV_PM_SUSPEND - Don't try I2C\n");
		return;
	}
	lg4894_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u32));
}
#elif defined(CONFIG_LGE_TOUCH_CORE_MTK)
static void lg4894_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int charger_state = atomic_read(&ts->state.connect);
	//int wireless_state = atomic_read(&ts->state.wireless);

	TOUCH_TRACE();

	d->charger = 0;
	/* wire */
	if (charger_state == CONNECT_CHARGER_UNKNOWN)
		d->charger = CONNECT_NONE;
	else if (charger_state == CONNECT_STANDARD_HOST
				|| charger_state == CONNECT_CHARGING_HOST)
		d->charger = CONNECT_USB;
	else if (charger_state == CONNECT_WIRELESS_CHARGER)
		d->charger = CONNECT_WIRELESS;
	else
		d->charger = CONNECT_TA;

	TOUCH_I("%s: write charger_state = 0x%02X\n", __func__, d->charger);
	if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
		TOUCH_I("DEV_PM_SUSPEND - Don't try I2C\n");
		return;
	}
	lg4894_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u32));
}
#else
static void lg4894_connect(struct device *dev)
{
	TOUCH_E("%s: Check your configuration!!\n");
}
#endif

static void lg4894_lcd_mode(struct device *dev, u32 mode)
{
	struct lg4894_data *d = to_lg4894_data(dev);

	TOUCH_I("lcd_mode: %d (prev: %d)\n", mode, d->lcd_mode);
#if 0 //no use 4894
	if (d->lcd_mode == LCD_MODE_U2 && d->watch.ext_wdata.time.disp_waton)
		ext_watch_get_current_time(dev, NULL, NULL);
#endif
	if (mode == LCD_MODE_U2_UNBLANK)
		mode = LCD_MODE_U2;

	d->lcd_mode = mode;
}

static int lg4894_usb_status(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("TA Type: %d\n", atomic_read(&ts->state.connect));
	lg4894_connect(dev);
	return 0;
}

static int lg4894_wireless_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Wireless charger: 0x%02X\n", atomic_read(&ts->state.wireless));
	lg4894_connect(dev);
	return 0;
}

static int lg4894_earjack_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Earjack Type: 0x%02X\n", atomic_read(&ts->state.earjack));
	return 0;
}

static int lg4894_debug_tool(struct device *dev, u32 value)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (value == DEBUG_TOOL_ENABLE) {
		ts->driver->irq_handler = lg4894_sic_abt_irq_handler;
	} else {
		ts->driver->irq_handler = lg4894_irq_handler;
	}

	return 0;
}

static void lg4894_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct lg4894_data *d =
			container_of(to_delayed_work(fb_notify_work),
				struct lg4894_data, fb_notify_work);
	int ret = 0;

	if (d->lcd_mode == LCD_MODE_U0 || d->lcd_mode == LCD_MODE_U2)
		ret = FB_SUSPEND;
	else
		ret = FB_RESUME;

	touch_notifier_call_chain(NOTIFY_FB, &ret);
}

static int lg4894_notify_charger(struct device *dev, ulong event, void *data)
{
struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	switch (event) {

	case LCD_EVENT_TOUCH_RESET_START:
		atomic_set(&d->hw_reset, event);
		atomic_set(&d->skip_update, 1);
		TOUCH_I("LCD_EVENT_TOUCH_RESET_START!\n");
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		touch_gpio_direction_output(ts->reset_pin, 0);
		break;

	case LCD_EVENT_TOUCH_RESET_END:
		atomic_set(&d->hw_reset, event);

		TOUCH_I("LCD_EVENT_TOUCH_RESET_END!\n");
		touch_gpio_direction_output(ts->reset_pin, 1);

		queue_delayed_work(ts->wq, &ts->init_work,
					msecs_to_jiffies(ts->caps.hw_reset_delay));
		break;

	default:
		TOUCH_E("%lu is not supported in charger mode\n", event);
		break;
	}

	return ret;
}

static int lg4894_notify_normal(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	switch (event) {
	case NOTIFY_TOUCH_RESET:
		if(atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_1)
			ret = 1;
		else
			ret = 0;
		TOUCH_I("NOTIFY_TOUCH_RESET! return = %d\n", ret);
		//atomic_set(&d->watch.state.font_status, FONT_EMPTY);
		//atomic_set(&d->block_watch_cfg, BLOCKED);
		break;
	case LCD_EVENT_TOUCH_RESET_START:
		atomic_set(&d->hw_reset, event);
		atomic_set(&d->skip_update, 1);
		TOUCH_I("LCD_EVENT_TOUCH_RESET_START!\n");
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		touch_gpio_direction_output(ts->reset_pin, 0);
		break;
	case LCD_EVENT_TOUCH_RESET_END:
		atomic_set(&d->hw_reset, event);

		TOUCH_I("LCD_EVENT_TOUCH_RESET_END!\n");
		touch_gpio_direction_output(ts->reset_pin, 1);

		atomic_set(&d->init, IC_INIT_NEED);
		queue_delayed_work(ts->wq, &ts->init_work,
					msecs_to_jiffies(ts->caps.hw_reset_delay));
		break;
	case LCD_EVENT_LCD_MODE:
		TOUCH_I("LCD_EVENT_LCD_MODE!\n");
		atomic_set(&d->skip_update, 0);
		lg4894_lcd_mode(dev, *(u32 *)data);
		queue_delayed_work(ts->wq, &d->fb_notify_work, 0);
		break;
	case NOTIFY_CONNECTION:
		TOUCH_I("NOTIFY_CONNECTION!\n");
		ret = lg4894_usb_status(dev, *(u32 *)data);
		break;
	case NOTIFY_WIRELEES:
		TOUCH_I("NOTIFY_WIRELEES!\n");
		ret = lg4894_wireless_status(dev, *(u32 *)data);
		break;
	case NOTIFY_EARJACK:
		TOUCH_I("NOTIFY_EARJACK!\n");
		ret = lg4894_earjack_status(dev, *(u32 *)data);
		break;
	case NOTIFY_IME_STATE:
		TOUCH_I("NOTIFY_IME_STATE!\n");
		ret = lg4894_reg_write(dev, REG_IME_STATE,
			(u32*)data, sizeof(u32));
		break;
	case NOTIFY_DEBUG_TOOL:
		ret = lg4894_debug_tool(dev, *(u32 *)data);
		TOUCH_I("NOTIFY_DEBUG_TOOL!\n");
		break;
	case NOTIFY_CALL_STATE:
		TOUCH_I("NOTIFY_CALL_STATE!\n");
		ret = lg4894_reg_write(dev, REG_CALL_STATE,
			(u32*)data, sizeof(u32));
		break;
	case NOTIFY_QMEMO_STATE:
		TOUCH_I("NOTIFY_QMEMO_STATE!\n");
		ret = lg4894_reg_write(dev, REG_QMEMO_STATE,
			(u32*)data, sizeof(u32));
		break;
	default:
		TOUCH_E("%lu is not supported\n", event);
		break;
	}

	return ret;
}

static int lg4894_notify(struct device *dev, ulong event, void *data)
{
	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("CHARGER MODE notify\n");
		return lg4894_notify_charger(dev,event,data);
	}

	return lg4894_notify_normal(dev,event,data);
}

static void lg4894_init_works(struct lg4894_data *d)
{
	INIT_DELAYED_WORK(&d->fb_notify_work, lg4894_fb_notify_work_func);
}

static void lg4894_init_locks(struct lg4894_data *d)
{
	mutex_init(&d->spi_lock);
}

static int lg4894_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = NULL;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate synaptics data\n");
		return -ENOMEM;
	}

	d->dev = dev;
	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	//touch_gpio_direction_output(ts->reset_pin, 0);
	touch_gpio_direction_output(ts->reset_pin, 1);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);
if (0) {
	touch_gpio_init(ts->maker_id_pin, "touch_make_id");
	touch_gpio_direction_input(ts->maker_id_pin);
}
	/******************************************************
	 * Display driver does control the power in LG4894 IC *
	 * due to its design from 1-chip. Here we skip the    *
	 * control.                                           *
	 ******************************************************/
	//touch_power_init(dev);
	touch_bus_init(dev, MAX_XFER_BUF_SIZE);

	lg4894_init_works(d);
	lg4894_init_locks(d);

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		touch_gpio_init(ts->reset_pin, "touch_reset");
		touch_gpio_direction_output(ts->reset_pin, 1);
		/* Deep Sleep */
		lg4894_deep_sleep(dev);
		return 0;
	}

	lg4894_get_tci_info(dev);
	lg4894_get_swipe_info(dev);

	d->lcd_mode = LCD_MODE_U3;
	d->tci_debug_type = TCI_DEBUG_DISABLE;
	d->err_cnt = 0;
	lg4894_sic_abt_probe();

	return 0;
}

static int lg4894_remove(struct device *dev)
{
	TOUCH_TRACE();
	lg4894_sic_abt_remove();
	//lg4894_watch_remove(dev);

	return 0;
}

static int lg4894_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	u8 dev_major = d->fw.version[0];
	u8 dev_minor = d->fw.version[1];
	u32 bin_ver_offset = *((u32 *)&fw->data[0xe8]);
	u32 bin_pid_offset = *((u32 *)&fw->data[0xf0]);
	char pid[12] = {0};
	u8 bin_major;
	u8 bin_minor;
	int update = 0;

	if ((bin_ver_offset > FLASH_FW_SIZE) || (bin_pid_offset > FLASH_FW_SIZE)) {
		TOUCH_I("INVALID OFFSET\n");
		return -1;
	}

	bin_major = fw->data[bin_ver_offset];
	bin_minor = fw->data[bin_ver_offset + 1];
	memcpy(pid, &fw->data[bin_pid_offset], 8);

	if (ts->force_fwup) {
		update = 1;
	} else if (bin_major != dev_major) {
			update = 1;
	} else  {
		if (bin_minor != dev_minor)
			update = 1;
	}

	if(dev_major == 0 && dev_minor == 0){
		TOUCH_E("Touch FW cannot be 0.0!! Check your panel connection!!\n");
		update = 0;
	}

	TOUCH_I(
		"bin-ver: %d.%02d (%s), dev-ver: %d.%02d -> update: %d, force_fwup: %d\n",
		bin_major, bin_minor, pid, dev_major, dev_minor,
		update, ts->force_fwup);

	return update;
}

static int lg4894_condition_wait(struct device *dev,
				    u16 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry)
{
	u32 data = 0;

	do {
		touch_msleep(delay);
		lg4894_read_value(dev, addr, &data);

		if ((data & mask) == expect) {
			if (value)
				*value = data;
			TOUCH_I(
				"%d, addr[%04x] data[%08x], mask[%08x], expect[%08x]\n",
				retry, addr, data, mask, expect);
			return 0;
		}
	} while (--retry);

	if (value)
		*value = data;

	TOUCH_I("%s addr[%04x], expect[%x], mask[%x], data[%x]\n",
		__func__, addr, expect, mask, data);

	return -EPERM;
}

static int lg4894_fw_upgrade(struct device *dev,
			     const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 *fwdata = (u8 *) fw->data;
	u32 data;
	u32 conf_dn_addr;
	int ret;
	int i = 0;

	TOUCH_I("%s - START\n", __func__);
	/* enable SPI between RAM and ROM */
	lg4894_write_value(dev, 0x15, 0);

	/* CM3 hold */
	lg4894_write_value(dev, spr_rst_ctl, 2);

	/* sram write enable */
	lg4894_write_value(dev, spr_sram_ctl, 3);

	for (i = 0 ; i < FLASH_FW_SIZE ; i += MAX_RW_SIZE) {

		/* code sram base address write */
		lg4894_write_value(dev, spr_code_offset, i / 4);

		/* first 60KB firmware image download to code sram */
		lg4894_reg_write(dev, code_access_addr, &fwdata[i], MAX_RW_SIZE);
	}

#if 0
	/* code sram base address write */
	lg4894_write_value(dev, spr_code_offset, 0);

	/* first 60KB firmware image download to code sram */
	lg4894_reg_write(dev, code_access_addr, &fwdata[0], MAX_RW_SIZE);

	/* code sram base address write */
	lg4894_write_value(dev, spr_code_offset, MAX_RW_SIZE / 4);

	/* last 12KB firmware image download to code sram */
	lg4894_reg_write(dev, code_access_addr, &fwdata[MAX_RW_SIZE], FLASH_FW_SIZE - MAX_RW_SIZE);
#endif

	/* CM3 Release*/
	lg4894_write_value(dev, spr_rst_ctl, 0);

	/* Boot Start */
	lg4894_write_value(dev, spr_boot_ctl, 1);

	/* firmware boot done check */
	ret = lg4894_condition_wait(dev, tc_flash_dn_sts, NULL,
				    FLASH_BOOTCHK_VALUE, 0xFFFFFFFF, 10, 200);
	if (ret < 0) {
		TOUCH_E("failed : \'boot check\'\n");
		return -EPERM;
	}
	/* Firmware Download Start */
	lg4894_write_value(dev, tc_flash_dn_ctl, (FLASH_KEY_CODE_CMD << 16) | 1);
	touch_msleep(ts->caps.hw_reset_delay);
	/* download check */
	ret = lg4894_condition_wait(dev, tc_flash_dn_sts, &data,
				    FLASH_CODE_DNCHK_VALUE, 0xFFFFFFFF, 10, 200);
	if (ret < 0) {
		TOUCH_E("failed : \'code check\'\n");
		return -EPERM;
	}
if (0) {
	/* conf base address read */
	lg4894_reg_read(dev, tc_confdn_base_addr, (u8 *)&data, sizeof(u32));
	conf_dn_addr = ((data >> 16) & 0xFFFF);
	TOUCH_I("conf_dn_addr : %08x data: %08x \n", conf_dn_addr, data);
	if (conf_dn_addr >= (0x1200) || conf_dn_addr < (0x8C0)) {
		TOUCH_E("failed : \'conf base invalid \'\n");
		return -EPERM;
	}

	/* conf sram base address write */
	lg4894_write_value(dev, spr_data_offset, conf_dn_addr);

	/* Conf data download to conf sram */
	lg4894_reg_write(dev, data_access_addr, &fwdata[FLASH_FW_SIZE], FLASH_CONF_SIZE);

	/* Conf Download Start */
	lg4894_write_value(dev, tc_flash_dn_ctl, (FLASH_KEY_CONF_CMD << 16) | 2);


	/* Conf check */
	ret = lg4894_condition_wait(dev, tc_flash_dn_sts, &data,
				    FLASH_CONF_DNCHK_VALUE, 0xFFFFFFFF, 10, 200);
	if (ret < 0) {
		TOUCH_E("failed : \'conf check\'\n");
		return -EPERM;
	}
}
	/*
	   if want to upgrade ic (not configure section writed)
	   do write register:0xC04 value:7
	   delay 1s
	*/
	TOUCH_I("===== Firmware download Okay =====\n");

	return 0;
}

static int lg4894_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	int ret = 0;
	int i = 0;

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n",
			&ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from def_fwpath : %s, lcd_revision: %d\n",
					fwpath, d->fw.lcd_fpcb_revision);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	fwpath[sizeof(fwpath) - 1] = '\0';

	if (strlen(fwpath) <= 0) {
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

	if (lg4894_fw_compare(dev, fw)) {
		ret = -EINVAL;

		/*
		 * TC_STOP before FW upgrade
		 */
		lg4894_write_value(dev, tc_drive_ctl, 0x02);

		touch_msleep(200);
		for (i = 0; i < 2 && ret; i++)
			ret = lg4894_fw_upgrade(dev, fw);
	} else {
		release_firmware(fw);
		return 0;
	}
	if (!ret) {
		d->err_cnt = 0;
		TOUCH_I("FW upgrade retry err_cnt clear\n");
	}

	release_firmware(fw);

	return ret;
}

static int lg4894_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int mfts_mode = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE)
		return -EPERM;

	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
		lg4894_power(dev, POWER_OFF);
		return -EPERM;
	} else {
		TOUCH_I("%s : touch_suspend start\n", __func__);
#if 0 //no use 4894
		if (d->lcd_mode == LCD_MODE_U2 &&
			atomic_read(&d->watch.state.rtc_status) == RTC_RUN &&
			d->watch.ext_wdata.time.disp_waton)
				ext_watch_get_current_time(dev, NULL, NULL);
#endif
	}

	if (atomic_read(&d->init) == IC_INIT_DONE)
		lg4894_lpwg_mode(dev);
	else /* need init */
		ret = 1;

	return ret;
}

static int lg4894_resume(struct device *dev)
{
#if 0 // FW-upgrade not working at MFTS mode
	struct touch_core_data *ts = to_touch_core(dev);
	int mfts_mode = 0;
#endif

	TOUCH_TRACE();

#if 0 // FW-upgrade not working at MFTS mode
	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		lg4894_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		lg4894_ic_info(dev);
		if (lg4894_upgrade(dev) == 0) {
			lg4894_power(dev, POWER_OFF);
			lg4894_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);
		}
	}
#endif
	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		lg4894_deep_sleep(dev);
		return -EPERM;
	}

	return 0;
}

static int lg4894_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	u32 data = 1;
	int ret = 0;

	TOUCH_TRACE();

#ifdef LG4894_TCL_OFF_VIA_MIPI
	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("TC clock is off. Turn it on before init\n");
		lg4894_clock(dev, 1);
	}
#endif

	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		TOUCH_I("fb_notif change\n");
		fb_unregister_client(&ts->fb_notif);
		ts->fb_notif.notifier_call = lg4894_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
	}

	TOUCH_I("%s: charger_state = 0x%02X\n", __func__, d->charger);

	if (atomic_read(&ts->state.debug_tool) == DEBUG_TOOL_ENABLE)
		lg4894_sic_abt_init(dev);

	ret = lg4894_ic_info(dev);
	if (ret < 0) {
		TOUCH_E("lg4894_ic_info failed %d\n", ret);
		return ret;
	}

	ret = lg4894_reg_write(dev, tc_device_ctl, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'tc_device_ctrl\', ret:%d\n", ret);

	ret = lg4894_reg_write(dev, tc_interrupt_ctl, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'tc_interrupt_ctrl\', ret:%d\n", ret);

	ret = lg4894_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u32));
	if (ret)
		TOUCH_E("failed to write \'spr_charger_sts\', ret:%d\n", ret);

	data = atomic_read(&ts->state.ime);
	ret = lg4894_reg_write(dev, REG_IME_STATE, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'reg_ime_state\', ret:%d\n", ret);

	/* Apply valid production info value depend on panel duality */

	if ((atomic_read(&ts->state.core) == CORE_PROBE)
		|| (atomic_read(&ts->state.core) == CORE_UPGRADE)) {
		TOUCH_I("Need to get prd_info cuz Driver is %s\n",
			atomic_read(&ts->state.core) == CORE_PROBE ? "Probing" : "FW upgraing");
		prd_info_set(dev);
	} else if ((atomic_read(&ts->state.mfts) != MFTS_NONE)) {
		TOUCH_I("Need to get prd_info in mfts\n");
		prd_info_set(dev);
	}

	atomic_set(&d->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);

	/*
	data = atomic_read(&d->qmemo);
	ret = lg4894_reg_write(dev, REG_QMEMO_STATE, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'reg_qmemo_state\', ret:%d\n", ret);
	*/

	lg4894_lpwg_mode(dev);
	if (ret)
		TOUCH_E("failed to lpwg_control, ret:%d\n", ret);
	return 0;
}

/* (1 << 5)|(1 << 6)|(1 << 7)|(1 << 9)|(1 << 10) */
/*#define INT_RESET_CLR_BIT   0x6C0*/  /*Global reset use only*/
/* (1 << 10) | (1 << 31)*/  /* Global Reset bit */
#define INT_GLOBAL_RESET_CLR_BIT 0x80000400
/* (1 << 6)|(1 << 7)|(1 << 9) */  /* Touch HW Reset bit */
#define INT_HW_RESET_CLR_BIT 0x2C0
/* LG4894 not set */
#define INT_SW_RESET_CLR_BIT 0x0
/* (1 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_LOGGING_CLR_BIT	 0x50A000
/* LG4894 not set */
#define INT_FWU_CLR_BIT      0x0
/* (1 << 5) |(1 << 6) |(1 << 7)|(0 << 9)|(0 << 10)|(0 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_NORMAL_MASK		 0x5080E0
#define IC_DEBUG_SIZE		 16	/* byte */

int lg4894_check_status(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;
	u32 status = d->info.device_status;//0x201
	u32 ic_status = d->info.ic_status;//0x200
	u32 debugging_num = 0;
	u32 debugging_mask = 0x0;
	u32 status_mask = 0x0;
	int checking_log_flag = 0;
	const int checking_log_size = 1024;
	char *checking_log = NULL;
	int length = 0;

	status_mask = status ^ INT_NORMAL_MASK;
	debugging_mask = ((status >> 16) & 0xF);
	if ((status_mask & INT_GLOBAL_RESET_CLR_BIT) || (ic_status & (1 << 0)) || (ic_status & (1 << 3))) {
		TOUCH_I("%s : Need Global Reset, status = %x, ic_status = %x\n",
			__func__, status, ic_status);
		if (atomic_read(&ts->state.fb) == FB_SUSPEND)
			ret = -EHWRESET;
		else
			ret = -EGLOBALRESET;
	} else if (status_mask & INT_HW_RESET_CLR_BIT) {
		TOUCH_I("%s : Need Touch HW Reset, status = %x, ic_status = %x\n",
			__func__, status, ic_status);
		ret = -EHWRESET;
	} else if (status_mask & INT_SW_RESET_CLR_BIT) {
		TOUCH_I("%s : Need Touch HW Reset, status = %x, ic_status = %x\n",
			__func__, status, ic_status);
		ret = -ESWRESET;
	} else if ((status_mask & INT_LOGGING_CLR_BIT) || (debugging_mask == 0x4)) {
		TOUCH_I("%s : Need Logging, status = %x, ic_status = %x\n",
			__func__, status, ic_status);
		ret = -ERANGE;
	} else if (status_mask & INT_FWU_CLR_BIT) {
		if (d->err_cnt >= 3) {
			ret = -ERANGE;
		} else {
			d->err_cnt++;
			ret = -EUPGRADE;
		}
		TOUCH_I("%s : Need FW Upgrade, status = %x, ic_status = %x err_cnt = %d %s\n",
			__func__, status, ic_status, d->err_cnt,
			d->err_cnt>=3 ? " skip upgrade":"");
	}

	if (ret != 0) {
		checking_log = kzalloc(sizeof(*checking_log) * checking_log_size, GFP_KERNEL);
		if(checking_log == NULL) {
			TOUCH_E("Failed to allocate mem for checking_log\n");
			ret = -ENOMEM;
			goto error;
		}
		if (ic_status & (1 << 0)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[IC_STATUS] [0]ESD detection");
		}
		if (ic_status & (1 << 3)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[IC_STATUS] [3]Watchdog Exception");
		}
		if (!(status & (1 << 5))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[TC_STATUS] [5]Device_ctl not Set");
		}
		if (!(status & (1 << 6))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[TC_STATUS] [6]Code CRC Invalid");
		}
		if (!(status & (1 << 7))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[TC_STATUS] [7]CFG CRC Invalid");
		}
		if (status & (1 << 9)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[TC_STATUS] [9]Abnormal status Detected");
		}
		if (status & (1 << 10)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[TC_STATUS] [10]System Error Detected");
		}
		if (status & (1 << 13)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[TC_STATUS] [13]Display mode Mismatch");
		}
		if (!(status & (1 << 15))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[TC_STATUS] [15]Interrupt_Pin Invalid");
		}
		if (!(status & (1 << 20))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[TC_STATUS] [20]Touch interrupt status Invalid");
		}
		if (!(status & (1 << 22))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[TC_STATUS] [22]TC driving Invalid");
		}
		if (status & (1 << 31)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[TC_STATUS] [31]DSI_Error");
		}

		if (checking_log_flag) {
			TOUCH_E("%s, status = %x, ic_status = %x\n",
					checking_log, status, ic_status);
		}
		if(checking_log != NULL)
			kfree(checking_log);
	}

/*
 *	0x3 abnormal Error
 *	0x4 debugging INT
 */
	if (debugging_mask == 0x3 || debugging_mask == 0x4) {
		lg4894_reg_read(dev, ic_debug_info_addr, &d->info.debug[0].ic_debug_info,
			sizeof(d->info.debug[0].ic_debug_info));
		debugging_num = d->info.debug[0].ic_debug_info;
		if (debugging_num < IC_DEBUG_INFO_NUM )
			TOUCH_E("[IC_DEBUG_INFO] [%d]%s \n", debugging_num,
				ic_debug_info_str[debugging_num]);
	}
error:
	return ret;
}

int lg4894_debug_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	struct lg4894_touch_debug *debug = d->info.debug;
	int ret = 0;
	int id = 0;
	int i = 0;

	u16 debug_change_mask = 0;
	u16 press_mask = 0;
	id = DEBUG_PROTOCOL_PACKET_NUM - 1;

	debug_change_mask = ts->old_mask ^ ts->new_mask;
	press_mask = ts->new_mask & debug_change_mask;

	/* check protocol ver */
	if (debug[id].protocol_ver < 0)
		return ret;

	/* check debugger status */
	if ((atomic_read(&ts->state.earjack) == EARJACK_DEBUG) ||
		(gpio_get_value(126) < 1))
			return ret;

	if (debug_change_mask && press_mask) {
		u8 buf[DEBUG_BUF_SIZE] = {0,};
		ret += snprintf(buf + ret, DEBUG_BUF_SIZE - ret,
				"[%d] %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				ts->tcount - 1,
				debug[id].protocol_ver,
				debug[id].frame_cnt,
				debug[id].rn_max_bfl,
				debug[id].rn_max_afl,
				debug[id].rn_min_bfl,
				debug[id].rn_min_afl,
				debug[id].rn_max_afl_x,
				debug[id].rn_max_afl_y,
				debug[id].seg1_cnt,
				debug[id].seg2_cnt,
				debug[id].seg1_thr,
				debug[id].rn_pos_cnt,
				debug[id].rn_neg_cnt,
				debug[id].rn_pos_sum,
				debug[id].rn_neg_sum,
				debug[id].rn_stable
			       );

		for (i = 0 ; i < ts->tcount ; i++) {
			if (i < 1)
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"[Touch]	tb:");
			ret += snprintf(buf + ret, DEBUG_BUF_SIZE - ret,
					"%2d ",	debug[id].track_bit[i]);
		}

		for (i = 0 ; i < sizeof(debug[id].rn_max_tobj) ; i++) {
			if (debug[id].rn_max_tobj[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" to:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug[id].rn_max_tobj[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug[id].rebase) ; i++) {
			if (debug[id].rebase[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" re:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug[id].rebase[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug[id].noise_detect) ; i++) {
			if (debug[id].noise_detect[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" nd:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug[id].noise_detect[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug[id].lf_oft) ; i++) {
			if (debug[id].lf_oft[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" lf:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2x ",	debug[id].lf_oft[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug[id].palm) ; i++) {
			if (debug[id].palm[i] > 0) {
				if (i < 1) {
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" pa:");
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							"%2d ",
							debug[id].palm[i]);
				}
				} else {
					break;
				}
			}
		TOUCH_I("%s\n", buf);
	}
	return ret;
}
static int lg4894_irq_abs_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	struct lg4894_touch_data *data = d->info.data;
	struct touch_data *tdata;
	u32 touch_count = 0;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;

	touch_count = d->info.touch_cnt;
	ts->new_mask = 0;

	/* check if palm detected */
	if (data[0].track_id == PALM_ID) {
		if (data[0].event == TOUCHSTS_DOWN) {
			ts->is_cancel = 1;
			TOUCH_I("Palm Detected\n");
		} else if (data[0].event == TOUCHSTS_UP) {
			ts->is_cancel = 0;
			TOUCH_I("Palm Released\n");
		}
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

#if defined(TARGET_MT6735_K6P) && defined(CONFIG_CORNER_QUICK_COVER)
	if (Touch_Quick_Cover_Closed && !ts->lpwg.qcover) {
		TOUCH_D(QUICKCOVER, "Skip irq while quickcover is closed\n");
		ts->intr_status = TOUCH_IRQ_NONE;
		touch_report_all_event(ts);
		return ret;
	}
#elif defined(CONFIG_MSM8909_K6P)
	if (get_smartcover_status() && !ts->lpwg.qcover) {
		TOUCH_D(QUICKCOVER, "Skip irq while quickcover is closed\n");
		ts->intr_status = TOUCH_IRQ_NONE;
		touch_report_all_event(ts);
		return ret;
	}
#endif

	for (i = 0; i < touch_count; i++) {
		if ((data[i].track_id >= MAX_FINGER) || (data[i].track_id < 0))
			continue;

		if (data[i].event == TOUCHSTS_DOWN
			|| data[i].event == TOUCHSTS_MOVE) {
			ts->new_mask |= (1 << data[i].track_id);
			tdata = ts->tdata + data[i].track_id;

			tdata->id = data[i].track_id;
			tdata->type = data[i].tool_type;
			tdata->x = data[i].x;
			tdata->y = data[i].y;
			tdata->pressure = data[i].pressure;
			tdata->width_major = data[i].width_major;
			tdata->width_minor = data[i].width_minor;

			if (data[i].width_major == data[i].width_minor)
				tdata->orientation = 1;
			else
				tdata->orientation = data[i].angle;

			finger_index++;

#if defined(CONFIG_LGE_TOUCH_HALL_IC_COVER)
			if (cradle_smart_cover_status() && d->use_qcover) {
				if (tdata->y > 290) {
					TOUCH_D(QUICKCOVER, "Skip irq while quickcover is closed\n");
					ts->intr_status = TOUCH_IRQ_NONE;
					return ret;
				}
			}
#endif

			TOUCH_D(ABS,
				"tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
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

#ifdef LG4894_ESD_SKIP_WHILE_TOUCH_ON
	if (finger_index != finger_cnt) {
		TOUCH_D(ABS, "finger cnt changed from %d to %d\n", finger_cnt, finger_index);
		finger_cnt = finger_index;
	}
#endif

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return ret;
}

int lg4894_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);

	/* check if touch cnt is valid */
	if (d->info.touch_cnt == 0 || d->info.touch_cnt > ts->caps.max_id)
		return -ERANGE;

	return lg4894_irq_abs_data(dev);
}

int lg4894_irq_lpwg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;

	if (d->info.wakeup_type == KNOCK_1) {
		if (ts->lpwg.mode != LPWG_NONE) {
			lg4894_get_tci_data(dev,
				ts->tci.info[TCI_1].tap_count);
			ts->intr_status = TOUCH_IRQ_KNOCK;
		}
	} else if (d->info.wakeup_type == KNOCK_2) {
		if (ts->lpwg.mode >= LPWG_PASSWORD) {
			lg4894_get_tci_data(dev,
				ts->tci.info[TCI_2].tap_count);
			ts->intr_status = TOUCH_IRQ_PASSWD;
		}
	} else if (d->info.wakeup_type == SWIPE_UP) {
		TOUCH_I("SWIPE_UP\n");
		lg4894_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_UP;
	} else if (d->info.wakeup_type == SWIPE_DOWN) {
		TOUCH_I("SWIPE_DOWN\n");
		lg4894_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_DOWN;
	} else if (d->info.wakeup_type == SWIPE_LEFT) {
		TOUCH_I("SWIPE_LEFT\n");
		lg4894_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_LEFT;
	} else if (d->info.wakeup_type == SWIPE_RIGHT) {
		TOUCH_I("SWIPE_RIGHT\n");
		lg4894_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_RIGHT;
	} else if (d->info.wakeup_type == KNOCK_OVERTAP) {
		TOUCH_I("LPWG wakeup_type is Overtap\n");
		lg4894_get_tci_data(dev, 1);
		ts->intr_status = TOUCH_IRQ_PASSWD;
	} else if (d->info.wakeup_type == CUSTOM_DEBUG) {
		TOUCH_I("LPWG wakeup_type is CUSTOM_DEBUG\n");
		lg4894_debug_tci(dev);
		lg4894_debug_swipe(dev);
	} else {
		TOUCH_I("LPWG wakeup_type is not support type![%d]\n",
			d->info.wakeup_type);
	}

	return ret;
}

int lg4894_irq_handler(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;

	if (atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_2) {
		ret = lg4894_reg_read(dev, tc_ic_status, &d->info,
				sizeof(d->info));
	} else {
		ret = lg4894_reg_read(dev, tc_ic_status, &d->info,
				sizeof(d->info) - sizeof(d->info.debug));
	}

	if (ret < 0) {
		if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
			ret = -EPERM;
			TOUCH_E("%s : read status error but touch ic sleep. Do nothing.\n", __func__);
		} else {
			ret = -EHWRESET;
			TOUCH_E("%s : read status error. Need to H/W reset.\n", __func__);
		}
		goto error;
	}

	ret = lg4894_check_status(dev);
	if (ret < 0)
		goto error;
	if (d->info.wakeup_type == ABS_MODE)
		ret = lg4894_irq_abs(dev);
	else
		ret = lg4894_irq_lpwg(dev);

	if (atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_2)
		lg4894_debug_info(dev);
error:
	return ret;
}

static ssize_t show_swipe_enable(struct device *dev, char *buf)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;

	ret = snprintf(buf + ret, PAGE_SIZE, "SWIPE mode = 0x%X\n",
		d->swipe.mode);

	TOUCH_I("Swipe mode = %s\n",
		d->swipe.mode == SWIPE_UP_BIT ? "SWIPE_UP" : "Disable");

	return ret;
}

static ssize_t store_swipe_enable(struct device *dev,
	const char *buf, size_t count)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if(value > 1 || value < 0) {
		TOUCH_I("Set Swipe mode wrong, 0, 1 only\n");
		return count;
	}

	d->swipe.mode = value ? SWIPE_UP_BIT : 0;

	return count;
}


static ssize_t store_reg_ctrl(struct device *dev,
				const char *buf, size_t count)
{
	char command[6] = {0};
	u32 reg = 0;
	int value = 0;
	u32 data = 1;
	u16 reg_addr;

	if (sscanf(buf, "%5s %x %x", command, &reg, &value) <= 0)
		return count;

	reg_addr = reg;
	if (!strcmp(command, "write")) {
		data = value;
		if (lg4894_reg_write(dev, reg_addr, &data, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x write fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else if (!strcmp(command, "read")) {
		if (lg4894_reg_read(dev, reg_addr, &data, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x read fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else {
		TOUCH_D(BASE_INFO, "Usage\n");
		TOUCH_D(BASE_INFO, "Write reg value\n");
		TOUCH_D(BASE_INFO, "Read reg\n");
	}
	return count;
}

static ssize_t show_tci_debug(struct device *dev, char *buf)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;
	u32 rdata = -1;

	if (lg4894_reg_read(dev, LPWG_DEBUG_CTRL,
				(u8 *)&rdata, sizeof(rdata)) < 0) {
		TOUCH_I("Fail to Read TCI Debug Reason type\n");
		return ret;
	}

	ret = snprintf(buf + ret, PAGE_SIZE,
			"Read TCI Debug Reason type[IC] = %s\n",
			debug_type[(rdata & 0x1 ? 1 : 0)]);
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Read TCI Debug Reason type[Driver] = %s\n",
			debug_type[d->tci_debug_type]);
	TOUCH_I("Read TCI Debug Reason type = %s\n",
			debug_type[d->tci_debug_type]);

	return ret;
}

static ssize_t store_tci_debug(struct device *dev,
						const char *buf, size_t count)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value > 1 || value < 0) {
		TOUCH_I("SET TCI debug reason wrong, 0, 1 only\n");
		return count;
	}

	d->tci_debug_type = (u8)value;
	TOUCH_I("SET TCI Debug reason type = %s\n", debug_type[value]);

	return count;
}

static ssize_t show_swipe_debug(struct device *dev, char *buf)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;
	u32 rdata = -1;

	if (lg4894_reg_read(dev, LPWG_DEBUG_CTRL,
				(u8 *)&rdata, sizeof(rdata)) < 0) {
		TOUCH_I("Fail to Read SWIPE Debug Reason type\n");
		return ret;
	}

	ret = snprintf(buf + ret, PAGE_SIZE,
			"Read SWIPE Debug Reason type[IC] = %s\n",
			debug_type[(rdata & 0x1 ? 1 : 0)]);
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Read SWIPE Debug Reason type[Driver] = %s\n",
			debug_type[d->swipe_debug_type]);
	TOUCH_I("Read SWIPE Debug Reason type = %s\n",
			debug_type[d->swipe_debug_type]);

	return ret;
}

static ssize_t store_swipe_debug(struct device *dev,
						const char *buf, size_t count)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value > 1 || value < 0) {
		TOUCH_I("SET SWIPE debug reason wrong, 0, 1 only\n");
		return count;
	}

	d->swipe_debug_type = (u8)value;
	TOUCH_I("Write SWIPE Debug reason type = %s\n", debug_type[value]);

	return count;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	lg4894_reset_ctrl(dev, value);

	return count;
}

static TOUCH_ATTR(swipe_enable, show_swipe_enable, store_swipe_enable);
static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(tci_debug, show_tci_debug, store_tci_debug);
static TOUCH_ATTR(swipe_debug, show_swipe_debug, store_swipe_debug);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);

static struct attribute *lg4894_attribute_list[] = {
	&touch_attr_swipe_enable.attr,
	&touch_attr_reg_ctrl.attr,
	&touch_attr_tci_debug.attr,
	&touch_attr_swipe_debug.attr,
	&touch_attr_reset_ctrl.attr,
	NULL,
};

static const struct attribute_group lg4894_attribute_group = {
	.attrs = lg4894_attribute_list,
};

static int lg4894_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &lg4894_attribute_group);
	if (ret < 0)
		TOUCH_E("lg4894 sysfs register failed\n");

	//lg4894_watch_register_sysfs(dev);
	lg4894_prd_register_sysfs(dev);
	lg4894_sic_abt_register_sysfs(&ts->kobj);

	return 0;
}

static int lg4894_get_cmd_version(struct device *dev, char *buf)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	int offset = 0;
	int ret = 0;
	u32 rdata[4] = {0};

	ret = lg4894_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = snprintf(buf + offset, PAGE_SIZE - offset, "version : v%d.%02d\n",
		d->fw.version[0], d->fw.version[1]);

	if (d->fw.revision == 0xFF) {
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"revision : Flash Erased(0xFF)\n");
	} else {
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"revision : %d\n", d->fw.revision);
	}

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
		"product id : [%s]\n", d->fw.product_id);

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
		"lcd fpcb revision : %d\n\n", d->fw.lcd_fpcb_revision);

	lg4894_reg_read(dev, info_lot_num, (u8 *)&rdata, sizeof(rdata));
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "lot : %d\n", rdata[0]);
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "serial : 0x%X\n", rdata[1]);
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "date : 0x%X 0x%X\n",
		rdata[2], rdata[3]);
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "date : %04d.%02d.%02d " \
		"%02d:%02d:%02d Site%d\n",
		rdata[2] & 0xFFFF, (rdata[2] >> 16 & 0xFF), (rdata[2] >> 24 & 0xFF),
		rdata[3] & 0xFF, (rdata[3] >> 8 & 0xFF), (rdata[3] >> 16 & 0xFF),
		(rdata[3] >> 24 & 0xFF));

	return offset;
}

static int lg4894_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct lg4894_data *d = to_lg4894_data(dev);
	int offset = 0;
	int ret = 0;

	ret = lg4894_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = snprintf(buf, PAGE_SIZE, "%s_v%d.%02d\n",
		d->fw.product_id, d->fw.version[0], d->fw.version[1]);

	return offset;
}

static int lg4894_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int lg4894_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = lg4894_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = lg4894_get_cmd_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}

static int lg4894_shutdown(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static struct touch_driver touch_driver = {
	.probe = lg4894_probe,
	.remove = lg4894_remove,
	.shutdown = lg4894_shutdown,
	.suspend = lg4894_suspend,
	.resume = lg4894_resume,
	.init = lg4894_init,
	.irq_handler = lg4894_irq_handler,
	.power = lg4894_power,
	.upgrade = lg4894_upgrade,
	.lpwg = lg4894_lpwg,
	.notify = lg4894_notify,
	.register_sysfs = lg4894_register_sysfs,
	.set = lg4894_set,
	.get = lg4894_get,
};

#define MATCH_NAME			"lge,lg4894"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{},
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
};

static int __init touch_device_init(void)
{
	TOUCH_I("touch_device_init func\n");
	TOUCH_TRACE();

    TOUCH_I("4894__[%s] touch_bus_device_init\n", __func__);
	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("PH1-BSP-Touch@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
