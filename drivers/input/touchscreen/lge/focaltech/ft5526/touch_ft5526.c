/* touch_ft5526.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hyokmin.kwon@lge.com
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
#define TS_MODULE "[ft5526]"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/async.h>

//#include <mach/board_lge.h>
//#include <linux/i2c.h>
//#include <touch_i2c.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

/*
 *  Include to Local Header File
 */
#include "touch_ft5526.h"
#include "touch_ft5526_prd.h"


// Definitions for Debugging Failure Reason in LPWG
enum {
	TCI_DEBUG_DISABLE = 0,
	TCI_DEBUG_ALWAYS,
	TCI_DEBUG_BUFFER,
	TCI_DEBUG_BUFFER_ALWAYS,
};



static const char *tci_debug_type_str[] = {
	"Disable Type",
	"Always Report Type",
	"Buffer Type",
	"Buffer and Always Report Type"
};

#define TCI_FR_BUF_LEN	10
#define TCI_FR_NUM		7

static const char const *tci_debug_str[TCI_FR_NUM + 1] = {
	"NONE",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP",
	"MULTI_FINGER",
	"DELAY_TIME", /* It means Over Tap */
	"PALM_STATE",
	"Reserved" // Invalid data
};

int apk_allocate_flag = 1;
int fw_upgrade_flag = 0;
u16 pre_pressure[FTS_MAX_POINTS] = {0,};


#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
u8 *tpd_i2c_dma_va = NULL;
dma_addr_t tpd_i2c_dma_pa = 0;
#endif

#ifdef CONFIG_MTK_I2C_EXTENSION
u8 *g_dma_buff_va = NULL;
dma_addr_t g_dma_buff_pa = 0;
#endif

#ifdef CONFIG_MTK_I2C_EXTENSION
	static void msg_dma_alloct(struct device *dev)
	{
		struct touch_core_data *ts = to_touch_core(dev);

	    if (NULL == g_dma_buff_va)
    		{
       		 ts->input->dev.coherent_dma_mask = DMA_BIT_MASK(32);
       		 g_dma_buff_va = (u8 *)dma_alloc_coherent(&ts->input->dev, 128, &g_dma_buff_pa, GFP_KERNEL);
    		}

	    	if(!g_dma_buff_va)
		{
	        	TOUCH_E("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	    	}
	}
/*
	static void msg_dma_release(void){
		if(g_dma_buff_va)
		{
	     		dma_free_coherent(NULL, 128, g_dma_buff_va, g_dma_buff_pa);
	        	g_dma_buff_va = NULL;
	        	g_dma_buff_pa = 0;
			TOUCH_E("[DMA][release] Allocate DMA I2C Buffer release!\n");
	    	}
	}
	*/
#endif

#ifdef FT5526_ESD_SKIP_WHILE_TOUCH_ON
static int finger_cnt = 0;

bool ft5526_check_finger(void)
{
	return finger_cnt==0? false:true;
}
EXPORT_SYMBOL(ft5526_check_finger);
#endif

int ft5526_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->rw_lock);

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);
	mutex_unlock(&d->rw_lock);
	return 0;

}

int ft5526_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->rw_lock);

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size + 1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	mutex_unlock(&d->rw_lock);

	return 0;
}

int ft5526_cmd_read(struct device *dev, void *cmd_data, int cmd_len, void *read_buf, int read_len)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->rw_lock);

	memcpy(&ts->tx_buf[0], cmd_data, cmd_len);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = cmd_len;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = read_len;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(read_buf, &ts->rx_buf[0], read_len);
	mutex_unlock(&d->rw_lock);
	return 0;

}

int fts_i2c_read_cmd(struct device *dev, char *writebuf, int writelen, char *readbuf, int readlen)
{
	struct ft5526_data *d = to_ft5526_data(dev);
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	mutex_lock(&d->rw_lock);

	if(readlen > 0)
	{
		if (writelen > 0) {
			ret = i2c_master_send(client, writebuf, writelen);
			if (ret < 0)
				TOUCH_E("touch bus read error : %d\n", ret);

			touch_msleep(1);
			ret = i2c_master_recv(client, readbuf, readlen);
			if (ret < 0)
				TOUCH_E("touch bus read error : %d\n", ret);
		} else {
			struct i2c_msg msgs[] = {
				{
					 .addr = client->addr,
					 .flags = I2C_M_RD,
					 .len = readlen,
					 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0)
				TOUCH_E("touch bus read error : %d\n", ret);
		}
	}

	mutex_unlock(&d->rw_lock);

	return ret;
}

static int ft5526_reset_ctrl(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
//	struct ft5526_data *d = to_ft5526_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
	default :
	case SW_RESET:
		TOUCH_I("%s : SW Reset\n", __func__);
//		ft5526_cmd_write(dev, CMD_ENA);
//		ft5526_cmd_write(dev, CMD_RESET_LOW);
		touch_msleep(5);
//		ft5526_cmd_write(dev, CMD_RESET_HIGH);
//		ft5526_cmd_write(dev, CMD_DIS);
		//touch_msleep(ts->caps.sw_reset_delay);
		touch_msleep(300);
		break;

	case HW_RESET:
		TOUCH_I("%s : HW Reset\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		//touch_msleep(ts->caps.hw_reset_delay);
		touch_msleep(300);
		break;
	}

	return 0;
}

static int ft5526_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
//	struct ft5526_data *d = to_ft5526_data(dev);

	TOUCH_TRACE();

	switch (ctrl) {

	case POWER_OFF:
		TOUCH_I("%s, off\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		break;

	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(300);
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

static void ft5526_get_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 0;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 100;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 0;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 100;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 20;
}

static struct ft5526_ic_info *ext_ic_info;

int ft5526_chipid_info(void)
{
	int ret = 0;

	if (!ext_ic_info) {
		return -EINVAL;
	}

	if ((ext_ic_info->chip_id == 0x86)
			&& (ext_ic_info->chip_id_low == 0x07)) {
		ret = 7;
	} else if ((ext_ic_info->chip_id == 0x86)
			&& (ext_ic_info->chip_id_low == 0x06)) {
		ret = 6;
	} else {
		ret = -EINVAL;
	}

	return ret;
}

int ft5526_ic_info(struct device *dev)
{
//	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	int ret = 0;
	u8 chip_id = 0;
	u8 chip_id_low = 0;
	u8 is_official = 0;
	u8 fw_version = 0;
	u8 fw_version_minor = 0;
	u8 fw_version_sub_minor = 0;
	u8 fw_vendor_id = 0;
	int i;
//	u8 rdata = 0;

	// In LPWG, i2c can be failed because of i2c sleep mode
	if (d->state != TC_STATE_ACTIVE) {
		TOUCH_E("Cannot get ic info in NOT ACTIVE mode\n");
		return -EPERM; // Do nothing in caller
	}

	// If it is failed to get info without error, just return error
	for (i = 0; i < 2; i++) {
		ret |= ft5526_reg_read(dev, FTS_REG_ID, (u8 *)&chip_id, 1);
		ret |= ft5526_reg_read(dev, FTS_REG_ID_LOW, (u8 *)&chip_id_low, 1);
		ret |= ft5526_reg_read(dev, FTS_REG_FW_VER, (u8 *)&fw_version, 1);
		ret |= ft5526_reg_read(dev, FTS_REG_FW_VER_MINOR, (u8 *)&fw_version_minor, 1);
		ret |= ft5526_reg_read(dev, FTS_REG_FW_VER_SUB_MINOR, (u8 *)&fw_version_sub_minor, 1);
		ret |= ft5526_reg_read(dev, FTS_REG_FW_VENDOR_ID, (u8 *)&fw_vendor_id, 1);
		ret |= ft5526_reg_read(dev, FTS_REG_PRODUCT_ID_H, d->ic_info.product_id, 2);

		if (ret == 0) {
			TOUCH_I("Success to get ic info data\n");
			break;
		}
	}

	if (i >= 2) {
		TOUCH_E("Failed to get ic info data, (need to recover it?)\n");
		return -EPERM; // Do nothing in caller
	}

	is_official = (fw_version & 0x80) >> 7;
	fw_version &= 0x7F;
	d->ic_info.version.major = fw_version;
	d->ic_info.version.minor = fw_version_minor;
	d->ic_info.version.sub_minor = fw_version_sub_minor;

	d->ic_info.chip_id = chip_id; // Device ID
	d->ic_info.chip_id_low = chip_id_low;
	d->ic_info.is_official = is_official;
	d->ic_info.fw_version = fw_version; // Major
	d->ic_info.fw_vendor_id = fw_vendor_id; // Vendor ID

	d->ic_info.info_valid = 1;

	ext_ic_info = &d->ic_info;

	TOUCH_I("==============================\n");
	TOUCH_I("chip_id: %x / chip_id_low : %x\n", d->ic_info.chip_id, d->ic_info.chip_id_low);
	TOUCH_I("IC_fw_version: v%d.%02d\n", d->ic_info.is_official, d->ic_info.fw_version);
	TOUCH_I("fw_vendor_id: %x\n", d->ic_info.fw_vendor_id);
	TOUCH_I("Product_id: FT%x%x\n", d->ic_info.product_id[0], d->ic_info.product_id[1]);
	TOUCH_I("=============================\n");

	return ret;

#if 0
	Flash erased
	LOT
#endif
}

#if 0
static void set_debug_reason(struct device *dev, int type)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4946_data *d = to_lg4946_data(dev);
	u32 wdata[2] = {0, };
	u32 start_addr = 0x0;
	int i = 0;

	wdata[0] = (u32)type;
	wdata[0] |= (d->tci_debug_type == 1) ? 0x01 << 2 : 0x01 << 3;
	wdata[1] = TCI_DEBUG_ALL;
	TOUCH_I("TCI%d-type:%d\n", type + 1, wdata[0]);

	lg4946_xfer_msg_ready(dev, 2);
	start_addr = TCI_FAIL_DEBUG_W;
	for (i = 0; i < 2; i++) {
		ts->xfer->data[i].tx.addr = start_addr + i;
		ts->xfer->data[i].tx.buf = (u8 *)&wdata[i];
		ts->xfer->data[i].tx.size = sizeof(u32);
	}

	lg4946_xfer_msg(dev, ts->xfer);

	return;
}
#endif

static int ft5526_tci_control(struct device *dev, int type)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u8 data;
	int ret = 0;


	switch (type) {
	case TCI_CTRL_SET:
#if 0
		if (d->tci_debug_type || d->en_i2c_lpwg) { // LPWG I2C Enable
			data = 0x01; // Stand-by mode
			TOUCH_I("Enable i2c block in LPWG mode\n");
		}
		else {
			data = 0x00; // Stop mode
		}
		d->en_i2c_lpwg = 0; // One-time flag for lpwg_sd
		ret = ft5526_reg_write(dev, 0xF6, &data, 1);
#endif
		data = ts->tci.mode;
		ret = ft5526_reg_write(dev, 0xD0, &data, 1);
		break;
#if 1
	case TCI_CTRL_CONFIG_COMMON:
		data = (((d->tci_debug_type) & 0x02) << 3) | ((d->tci_debug_type) & 0x01);
		ret = ft5526_reg_write(dev, 0xE5, &data, 1);	// Fail Reason Debug Function Enable
#if 0
		data = 0x0A;
		ret |= ft5526_reg_write(dev, 0x80, &data, 1);	// Touch Differ Threshold (32 * 0x0A = 320)
		data = 0x21;
		ret |= ft5526_reg_write(dev, 0x89, &data, 1);	// Frame Rate for LPWG (30 Hz)
#endif
		data = 0x2D;
		ret |= ft5526_reg_write(dev, 0xB4, &data, 1);	// Active Area LSB of X1
		data = 0x00;
		ret |= ft5526_reg_write(dev, 0xB5, &data, 1);	// Active Area MSB of X1 (45)
		data = 0xF3;
		ret |= ft5526_reg_write(dev, 0xB6, &data, 1);	// Active Area LSB of X2
		data = 0x02;
		ret |= ft5526_reg_write(dev, 0xB7, &data, 1);	// Active Area MSB of X2 (800)
		data = 0x2D;
		ret |= ft5526_reg_write(dev, 0xB8, &data, 1);	// Active Area LSB of Y1
		data = 0x00;
		ret |= ft5526_reg_write(dev, 0xB9, &data, 1);	// Active Area MSB of Y1 (45)
		data = 0xD3;
		ret |= ft5526_reg_write(dev, 0xBA, &data, 1);	// Active Area LSB of Y2
		data = 0x04;
		ret |= ft5526_reg_write(dev, 0xBB, &data, 1);	// Active Area MSB of Y2 (1280)
		break;

	case TCI_CTRL_CONFIG_TCI_1:
		data = 0x0A;
		ret = ft5526_reg_write(dev, 0xBC, &data, 1);	// Touch Slop (10mm)
		data = 0x0A;
		ret |= ft5526_reg_write(dev, 0xC4, &data, 1);	// Touch Distance (10mm)
		data = 0x46; //0x32;
		ret |= ft5526_reg_write(dev, 0xC6, &data, 1);	// Time Gap Max (700ms)
		data = 0x02;
		ret |= ft5526_reg_write(dev, 0xCA, &data, 1);	// Total Tap Count (2)
		data = info1->intr_delay; //0x32;
		ret |= ft5526_reg_write(dev, 0xCC, &data, 1);	// Interrupt Delay (700ms or 0ms)
		break;

	case TCI_CTRL_CONFIG_TCI_2:
		data = 0x0A;
		ret = ft5526_reg_write(dev, 0xBD, &data, 1);	// Touch Slop (10mm)
		data = 0xFF; //0xC8;
		ret |= ft5526_reg_write(dev, 0xC5, &data, 1);	// Touch Distance (255mm)
		data = 0x46;
		ret |= ft5526_reg_write(dev, 0xC7, &data, 1);	// Time Gap Max (700ms)
		data = info2->tap_count;
		ret |= ft5526_reg_write(dev, 0xCB, &data, 1);	// Total Tap Count (6~10)
		data = 0x32;
		ret |= ft5526_reg_write(dev, 0xCD, &data, 1);	// Interrupt Delay (500ms)
		break;
#endif

	default:
		break;
	}

	return ret;
}


static int ft5526_lpwg_control(struct device *dev, u8 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	//struct ft5526_data *d = to_ft5526_data(dev);
	int ret = 0;

	switch (mode) {

	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		info1->intr_delay = 0;

		ret = ft5526_tci_control(dev, TCI_CTRL_CONFIG_TCI_1);
		ret |= ft5526_tci_control(dev, TCI_CTRL_CONFIG_COMMON);
		ret |= ft5526_tci_control(dev, TCI_CTRL_SET);

		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x03;
		info1->intr_delay = ts->tci.double_tap_check ? 70 : 0;

		ret = ft5526_tci_control(dev, TCI_CTRL_CONFIG_TCI_1);
		ret |= ft5526_tci_control(dev, TCI_CTRL_CONFIG_TCI_2);
		ret |= ft5526_tci_control(dev, TCI_CTRL_CONFIG_COMMON);
		ret |= ft5526_tci_control(dev, TCI_CTRL_SET);

		break;

	case LPWG_PASSWORD_ONLY:
		ts->tci.mode = 0x02;
		info1->intr_delay = 0;

//		ret = ft5526_tci_control(dev, TCI_CTRL_CONFIG_TCI_1);
		ret |= ft5526_tci_control(dev, TCI_CTRL_CONFIG_TCI_2);
		ret |= ft5526_tci_control(dev, TCI_CTRL_CONFIG_COMMON);
		ret |= ft5526_tci_control(dev, TCI_CTRL_SET);

		break;

	default:
		ts->tci.mode = 0;

		ret = ft5526_tci_control(dev, TCI_CTRL_SET);

		break;
	}

	TOUCH_I("ft5526_lpwg_control mode = %d\n", mode);

	return ret;
}


static int ft5526_deep_sleep(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 data;

	TOUCH_I("ft5526_deep_sleep = %d\n", mode);

	if(mode) {
		if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
			return 0;
		data = 0x03;
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
		return ft5526_reg_write(dev, 0xA5, &data, 1);
	}
	else {
		if (atomic_read(&ts->state.sleep) == IC_NORMAL)
			return 0;
		// Do something
		atomic_set(&ts->state.sleep, IC_NORMAL);
		return 0;
	}
}

#if 0
static void ft5526_debug_tci(struct device *dev)
{
	struct ft5526_data *d = to_ft5526_data(dev);
	u8 debug_reason_buf[TCI_MAX_NUM][TCI_DEBUG_MAX_NUM];
	u32 rdata[9] = {0, };
	u8 count[2] = {0, };
	u8 count_max = 0;
	u32 i, j = 0;
	u8 buf = 0;

	if (!d->tci_debug_type)
		return;

	ft5526_reg_read(dev, TCI_DEBUG_R, &rdata, sizeof(rdata));

	count[TCI_1] = (rdata[0] & 0xFFFF);
	count[TCI_2] = ((rdata[0] >> 16) & 0xFFFF);
	count_max = (count[TCI_1] > count[TCI_2]) ? count[TCI_1] : count[TCI_2];

	if (count_max == 0)
		return;

	if (count_max > TCI_DEBUG_MAX_NUM) {
		count_max = TCI_DEBUG_MAX_NUM;
		if (count[TCI_1] > TCI_DEBUG_MAX_NUM)
			count[TCI_1] = TCI_DEBUG_MAX_NUM;
		if (count[TCI_2] > TCI_DEBUG_MAX_NUM)
			count[TCI_2] = TCI_DEBUG_MAX_NUM;
	}

	for (i = 0; i < ((count_max-1)/4)+1; i++) {
		memcpy(&debug_reason_buf[TCI_1][i*4], &rdata[i+1], sizeof(u32));
		memcpy(&debug_reason_buf[TCI_2][i*4], &rdata[i+5], sizeof(u32));
	}

	TOUCH_I("TCI count_max = %d\n", count_max);
	for (i = 0; i < TCI_MAX_NUM; i++) {
		TOUCH_I("TCI count[%d] = %d\n", i, count[i]);
		for (j = 0; j < count[i]; j++) {
			buf = debug_reason_buf[i][j];
			TOUCH_I("TCI_%d - DBG[%d/%d]: %s\n",
				i + 1, j + 1, count[i],
				(buf > 0 && buf < TCI_FAIL_NUM) ?
					tci_debug_str[buf] :
					tci_debug_str[0]);
		}
	}
}
#endif

int ft5526_lpwg_reset(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("ft5526_lpwg_reset\n");
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);

	touch_msleep(1);
	LCD_RESET_H;
	touch_msleep(1);
	LCD_RESET_L;
	touch_msleep(1);
	LCD_RESET_H;

	touch_msleep(200);
	return 0;
}

static int ft5526_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	u8 next_state;
	int ret = 0, mfts_mode = 0;


	TOUCH_I(
		"ft5526_lpwg_mode : mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
		ts->lpwg.mode,
		ts->lpwg.screen ? "ON" : "OFF",
		ts->lpwg.sensor ? "FAR" : "NEAR",
		ts->lpwg.qcover ? "CLOSE" : "OPEN");
	TOUCH_I("d->state[%d]\n", d->state);


	if (atomic_read(&d->init) == IC_INIT_NEED) {
		TOUCH_I("Not Ready, Need IC init\n");
		return 0;
	}

	// Check MFTS mode to use POWER_OFF state
	if ((atomic_read(&ts->state.mfts) >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		TOUCH_I("MINIOS_MFTS [%d]\n", mfts_mode);
		if (d->state == TC_STATE_ACTIVE && ts->lpwg.screen == 0) { // Int disable, Touch/LCD Reset 0, DSV Off, VDDI Off
			next_state = TC_STATE_POWER_OFF;
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			touch_gpio_direction_output(ts->reset_pin, 0);
			} else if (d->state == TC_STATE_POWER_OFF && ts->lpwg.screen == 1) { // VDDI On, 1ms, DSV On, Touch/LCD Reset 1, Int enable
			next_state = TC_STATE_ACTIVE;

			touch_gpio_direction_output(ts->reset_pin, 1);
			touch_msleep(105); // lmh add

			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			touch_msleep(300); // ??????? Check with LCD on delay
		} else
			next_state = d->state;
		goto RET;
	}

	// NORMAL Case
	if (d->state == TC_STATE_ACTIVE) {
		if (ts->lpwg.screen == 0) {
			if((ts->lpwg.mode == 0) && ts->role.mfts_lpwg == 0) {
				next_state = TC_STATE_DEEP_SLEEP;
				ret = ft5526_deep_sleep(dev, 1);
				TOUCH_I("4 0\n");
			} else if (ts->lpwg.sensor == 1) {
				next_state = TC_STATE_LPWG;
				TOUCH_I("3\n");
				ret = ft5526_lpwg_control(dev, ts->lpwg.mode); // LCD SLP In or LCD Deep Sleep, 100ms -> Touch LPWG
			}
			else {
				next_state = TC_STATE_DEEP_SLEEP;
				ret = ft5526_deep_sleep(dev, 1); // LCD Deep Sleep, 100ms -> Touch Deep Sleep, DSV Off
				TOUCH_I("4\n");
			}
		}
		else {
			next_state = TC_STATE_ACTIVE; // Do nothing
		}
	} else if (d->state == TC_STATE_LPWG) {
		if (ts->lpwg.screen == 0) {
			if (ts->lpwg.sensor == 0) {
				next_state = TC_STATE_DEEP_SLEEP; // Touch Reset, Deep Sleep, DSV Off
				touch_gpio_direction_output(ts->reset_pin, 0);
				touch_msleep(5);
				touch_gpio_direction_output(ts->reset_pin, 1);
				touch_msleep(105); // 105-> 300
				ret = ft5526_deep_sleep(dev, 1);
				TOUCH_I("5\n");
			}
			else {
				next_state = TC_STATE_LPWG; // Do nothing
			}
		}
		else {
			next_state = TC_STATE_ACTIVE; // Touch Reset -> LCD Reset, SLP Out
			if(d->tci_debug_type & 0x02)
				ft5526_report_tci_fr_buffer(dev); // Report fr before touch IC reset
			touch_report_all_event(ts);
			touch_gpio_direction_output(ts->reset_pin, 0);
			TOUCH_I("6\n");
			touch_msleep(5);
			touch_gpio_direction_output(ts->reset_pin, 1);
//			touch_msleep(105); //  case resume version 0.0
		}
	} else if (d->state == TC_STATE_DEEP_SLEEP) {
		if (ts->lpwg.screen == 0) {
			if (ts->lpwg.sensor == 1) {
				next_state = TC_STATE_LPWG;
				TOUCH_I("7\n");
				touch_gpio_direction_output(ts->reset_pin, 0);
				touch_msleep(5);
				touch_gpio_direction_output(ts->reset_pin, 1);
				touch_msleep(200);
				ret = ft5526_deep_sleep(dev, 0);
				ret = ft5526_lpwg_control(dev, ts->lpwg.mode);
			}
			else {
				next_state = TC_STATE_DEEP_SLEEP; // Do nothing
			}
		}
		else {
			next_state = TC_STATE_ACTIVE; //  Touch DSV On, Reset
			TOUCH_I("8\n");

			touch_gpio_direction_output(ts->reset_pin, 0);

			touch_msleep(5);
			touch_gpio_direction_output(ts->reset_pin, 1);
			touch_msleep(105);

			ret = ft5526_deep_sleep(dev, 0);
			touch_msleep(15);
		}
	}
	else {
		next_state = d->state;
	}

RET:

	TOUCH_I("State changed from [%d] to [%d]\n", d->state, next_state);

	d->state = next_state;

	return ret;

}

#if 0
static int ft5526_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);

	if (atomic_read(&d->init) == IC_INIT_NEED) {
		TOUCH_I("Not Ready, Need IC init\n");
		return 0;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg) {
			ft5526_lpwg_control(dev, LPWG_DOUBLE_TAP);
			//ft5526_tc_driving(dev, d->lcd_mode);
			return 0;
		}
		if (ts->lpwg.mode == LPWG_NONE) {
			/* deep sleep */
			TOUCH_I("LPWG_NONE in FB_SUSPEND => Deep Sleep (PROX_NEAR??)\n");
			ft5526_deep_sleep(dev, 1);
		} else if (ts->lpwg.screen) {
			TOUCH_I("Skip lpwg_mode\n");
			ft5526_debug_tci(dev);
			ft5526_debug_swipe(dev);
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			/* knock on/code disable */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				ft5526_clock(dev, 1);

			ft5526_lpwg_control(dev, LPWG_NONE);
			ft5526_tc_driving(dev, d->lcd_mode);
		} else {
			/* knock on/code */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				//ft5526_clock(dev, 1);
				ft5526_deep_sleep(dev, 0);

			ft5526_lpwg_control(dev, ts->lpwg.mode);
			//ft5526_tc_driving(dev, d->lcd_mode);
		}
		return 0;
	}

	/* resume */
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("resume ts->lpwg.screen\n");
		ft5526_lpwg_control(dev, LPWG_NONE);
		ft5526_tc_driving(dev, d->lcd_mode);
	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* wake up */
		TOUCH_I("resume ts->lpwg.mode == LPWG_NONE\n");
		ft5526_tc_driving(dev, LCD_MODE_STOP);
	} else {
		/* partial */
		TOUCH_I("resume Partial\n");
		if (ts->lpwg.qcover == HOLE_NEAR)
			ft5526_lpwg_control(dev, LPWG_NONE);
		else
			ft5526_lpwg_control(dev, ts->lpwg.mode);
		ft5526_tc_driving(dev, LCD_MODE_U3_PARTIAL);
	}

	return 0;
}
#endif

#if 0
static int synaptics_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->lpwg.mode == LPWG_NONE) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep\n",
				__func__, __LINE__);
			synaptics_lpwg_control(dev, LPWG_NONE);
		} else if (ts->lpwg.screen) {
			TOUCH_I("%s(%d) - FB_SUSPEND & screen on -> skip\n",
				__func__, __LINE__);
			return 0;
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep by prox\n",
				__func__, __LINE__);
			synaptics_lpwg_control(dev, LPWG_DOUBLE_TAP);
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			/* knock on */
			TOUCH_I("%s(%d) - knock on by hole\n",
				__func__, __LINE__);
			synaptics_lpwg_control(dev, LPWG_DOUBLE_TAP);
		} else {
			/* knock on/code */
			TOUCH_I("%s(%d) - knock %d, screen %d, proxy %d, qcover %d\n",
				__func__, __LINE__,
				ts->lpwg.mode, ts->lpwg.screen, ts->lpwg.sensor, ts->lpwg.qcover);
			synaptics_lpwg_control(dev, ts->lpwg.mode);
		}
		return 0;
	}

	/* resume */
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("%s(%d) - normal\n",
				__func__, __LINE__);
		synaptics_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* normal */
		TOUCH_I("%s(%d) - normal on screen off\n",
				__func__, __LINE__);
		synaptics_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		/* wake up */
		TOUCH_I("%s(%d) - wake up on screen off and prox\n",
				__func__, __LINE__);
		TOUCH_I("%s - wake up is not ready\n", __func__);
		synaptics_lpwg_control(dev, LPWG_NONE);
	} else {
		/* partial */
		TOUCH_I("%s(%d) - partial mode(knock %d, screen %d, proxy %d, qcover %d)\n",
				__func__, __LINE__,
				ts->lpwg.mode, ts->lpwg.screen, ts->lpwg.sensor, ts->lpwg.qcover);
		TOUCH_I("%s - partial is not ready\n", __func__);
		synaptics_lpwg_control(dev, LPWG_NONE);
	}

	return 0;
}
#endif

static int ft5526_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
//	struct ft5526_data *d = to_ft5526_data(dev);
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
		TOUCH_I("LPWG_TAP_COUNT: [%d]\n", value[0]);
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		ts->tci.double_tap_check = value[0];
		TOUCH_I("LPWG_DOUBLE_TAP_CHECK: [%d]\n", value[0]);
		break;

	case LPWG_UPDATE_ALL:
		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];

		TOUCH_I(
			"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			value[0],
			value[1] ? "ON" : "OFF",
			value[2] ? "FAR" : "NEAR",
			value[3] ? "CLOSE" : "OPEN");
#if 0
		// Screen change is done in fb callback, not here
		if(ts->lpwg.screen != value[1]) {
			ts->lpwg.screen = value[1];
			TOUCH_I("Screen state changed => Ignore LPWG_UPDATE_ALL\n");
		}
		else {
#endif
			ft5526_lpwg_mode(dev);
//		}

		break;

	case LPWG_REPLY:
		break;

	}

	return 0;
}

static void ft5526_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	int charger_state = atomic_read(&ts->state.connect);
//	int wireless_state = atomic_read(&ts->state.wireless);

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

	/* code for TA simulator */
	if (atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_4) {
		TOUCH_I("TA Simulator mode, Set CONNECT_TA\n");
		d->charger = CONNECT_TA;
	}

	/* wireless */
	/*
	if (wireless_state)
		d->charger = d->charger | CONNECT_WIRELESS;
	*/

	/* Distinguish just TA state or not. */
	if (d->charger == CONNECT_TA || d->charger == CONNECT_OTG || d->charger == CONNECT_USB)
		d->charger = 1;
	else
		d->charger = 0;

	if (ts->lpwg.screen != 0 ) {

		TOUCH_I("%s: write charger_state = 0x%02X\n", __func__, d->charger);
		if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
			TOUCH_I("DEV_PM_SUSPEND - Don't try SPI\n");
			return;
		}

		ft5526_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u8));
	}

}

#if 0
static void ft5526_lcd_event_read_reg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u32 rdata[5] = {0};

//	ft5526_xfer_msg_ready(dev, 5);

	ts->xfer->data[0].rx.addr = tc_ic_status;
	ts->xfer->data[0].rx.buf = (u8 *)&rdata[0];
	ts->xfer->data[0].rx.size = sizeof(rdata[0]);

	ts->xfer->data[1].rx.addr = tc_status;
	ts->xfer->data[1].rx.buf = (u8 *)&rdata[1];
	ts->xfer->data[1].rx.size = sizeof(rdata[1]);

	ts->xfer->data[2].rx.addr = spr_subdisp_st;
	ts->xfer->data[2].rx.buf = (u8 *)&rdata[2];
	ts->xfer->data[2].rx.size = sizeof(rdata[2]);

	ts->xfer->data[3].rx.addr = tc_version;
	ts->xfer->data[3].rx.buf = (u8 *)&rdata[3];
	ts->xfer->data[3].rx.size = sizeof(rdata[3]);

	ts->xfer->data[4].rx.addr = 0x0;
	ts->xfer->data[4].rx.buf = (u8 *)&rdata[4];
	ts->xfer->data[4].rx.size = sizeof(rdata[4]);

//	ft5526_xfer_msg(dev, ts->xfer);

	TOUCH_I(
		"reg[%x] = 0x%x reg[%x] = 0x%x reg[%x] = 0x%x reg[%x] = 0x%x reg[%x] = 0x%x\n",
		tc_ic_status, rdata[0], tc_status, rdata[1],
		spr_subdisp_st, rdata[2], tc_version, rdata[3],
		0x0, rdata[4]);
	TOUCH_I("v%d.%02d\n", (rdata[3] >> 8) & 0xF, rdata[3] & 0xFF);
}

void ft5526_xfer_msg_ready(struct device *dev, u8 msg_cnt)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);

	mutex_lock(&d->spi_lock);

	ts->xfer->msg_count = msg_cnt;
}
#endif

static int ft5526_usb_status(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("TA Type: %d\n", atomic_read(&ts->state.connect));
	ft5526_connect(dev);
	return 0;
}

#if 0
static int ft5526_wireless_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Wireless charger: 0x%02X\n", atomic_read(&ts->state.wireless));
	ft5526_connect(dev);
	return 0;
}

static int ft5526_earjack_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Earjack Type: 0x%02X\n", atomic_read(&ts->state.earjack));
	return 0;
}

static int ft5526_debug_tool(struct device *dev, u32 value)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (value == DEBUG_TOOL_ENABLE) {
//		ts->driver->irq_handler = ft5526_sic_abt_irq_handler;
	} else {
		ts->driver->irq_handler = ft5526_irq_handler;
	}

	return 0;
}
static int ft5526_debug_option(struct device *dev, u32 *data)
{
	u32 chg_mask = data[0];
	u32 enable = data[1];

	switch (chg_mask) {
	case DEBUG_OPTION_0:
		TOUCH_I("Debug Option 0 %s\n", enable ? "Enable" : "Disable");
		break;
	case DEBUG_OPTION_1:
//		if (enable)	/* ASC */
//			ft5526_asc_control(dev, ASC_ON);
//		else
//			ft5526_asc_control(dev, ASC_OFF);
		break;
	case DEBUG_OPTION_2:
		TOUCH_I("Debug Info %s\n", enable ? "Enable" : "Disable");
		break;
	case DEBUG_OPTION_3:
		TOUCH_I("Debug Info Depth 10 %s\n", enable ? "Enable" : "Disable");
		break;
	case DEBUG_OPTION_4:
		TOUCH_I("TA Simulator mode %s\n", enable ? "Enable" : "Disable");
		ft5526_connect(dev);
		break;
	default:
		TOUCH_E("Not supported debug option\n");
		break;
	}

	return 0;
}

static void lg4946_debug_info_work_func(struct work_struct *debug_info_work)
{
	struct lg4946_data *d =
			container_of(to_delayed_work(debug_info_work),
				struct lg4946_data, debug_info_work);
	struct touch_core_data *ts = to_touch_core(d->dev);

	int status = 0;

	status = lg4946_debug_info(d->dev, 1);

	if (status > 0) {
		queue_delayed_work(d->wq_log, &d->debug_info_work , DEBUG_WQ_TIME);
	} else if (status < 0) {
		TOUCH_I("debug info log stop\n");
		atomic_set(&ts->state.debug_option_mask, DEBUG_OPTION_2);
	}
}
static void ft5526_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct ft5526_data *d =
			container_of(to_delayed_work(fb_notify_work),
				struct ft5526_data, fb_notify_work);
	int ret = 0;

	if (d->lcd_mode == LCD_MODE_U3)
		ret = FB_RESUME;
	else
		ret = FB_SUSPEND;

	touch_notifier_call_chain(NOTIFY_FB, &ret);
}
#endif

static int ft5526_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *ev = (struct fb_event *)data;
	//struct touch_core_data *ts = container_of(self, struct touch_core_data, fb_notif);
	//struct ft5526_data *d = (struct ft5526_data *)touch_get_device(ts);
//	struct ft5526_data *d = container_of(self, struct ft5526_data, fb_notif);
	//struct touch_core_data *ts = to_touch_core(d->dev);
	int event_data;

	if(event != FB_EVENT_BLANK && event != FB_EARLY_EVENT_BLANK && event != FB_R_EARLY_EVENT_BLANK) {
		return 0;
	}

	if (ev && ev->data) {
		event_data = *((int *)ev->data);
		if (event == FB_EVENT_BLANK) {
			if (event_data == FB_BLANK_POWERDOWN) {
				TOUCH_I("FB_EVENT_BLANK / FB_BLANK_POWERDOWN\n");
//				ft5526_lpwg_mode(d->dev);
//				mutex_unlock(&d->fb_lock);
				//TOUCH_I("ft5526_fb_notifier_callback mutex [-]\n");
			}
			else if (event_data == FB_BLANK_UNBLANK) {
//				mutex_unlock(&d->fb_lock);
				//TOUCH_I("ft5526_fb_notifier_callback mutex [-]\n");
			}
		}
		else if (event == FB_EARLY_EVENT_BLANK) {
			if (event_data == FB_BLANK_UNBLANK) {
				//TOUCH_I("ft5526_fb_notifier_callback mutex [+]\n");
//				mutex_lock(&d->fb_lock);
				TOUCH_I("FB_EARLY_EVENT_BLANK / FB_BLANK_UNBLANK\n");
//				ft5526_lpwg_mode(d->dev);
			}
			else if (event_data == FB_BLANK_POWERDOWN) {
				//TOUCH_I("ft5526_fb_notifier_callback mutex [+]\n");
//				mutex_lock(&d->fb_lock);
			}
		}
		else if (event == FB_R_EARLY_EVENT_BLANK) {
			if (event_data == FB_BLANK_POWERDOWN) {
				TOUCH_I("FB_R_EARLY_EVENT_BLANK / FB_BLANK_POWERDOWN\n");
//				mutex_unlock(&d->fb_lock);
				//TOUCH_I("ft5526_fb_notifier_callback mutex [-]\n");
			}
			else if (event_data == FB_BLANK_UNBLANK) {
				TOUCH_I("FB_R_EARLY_EVENT_BLANK / FB_BLANK_UNBLANK\n");
//				ft5526_lpwg_mode(d->dev);
//				mutex_unlock(&d->fb_lock);
				//TOUCH_I("ft5526_fb_notifier_callback mutex [-]\n");
			}
		}
	}

#if 0
	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			TOUCH_I("FB_UNBLANK\n");
		else if (*blank == FB_BLANK_POWERDOWN)
			TOUCH_I("FB_BLANK\n");
	}
#endif

#if 0
	if (ev && ev->data) {
		int *blank = (int *)ev->data;

		switch(event) {
			case FB_EARLY_EVENT_BLANK:
				TOUCH_I("ft5526_fb_notifier_callback : FB_EARLY_EVENT_BLANK : [%d]\n", *blank);
				break;

			case FB_EVENT_BLANK:
				TOUCH_I("ft5526_fb_notifier_callback : FB_EVENT_BLANK : [%d]\n", *blank);
				break;

			case FB_R_EARLY_EVENT_BLANK:
				TOUCH_I("ft5526_fb_notifier_callback : FB_R_EARLY_EVENT_BLANK : [%d]\n", *blank);
				break;

			default:
				TOUCH_I("ft5526_fb_notifier_callback : Others [%ld] : [%d]\n", event, *blank);
				break;

		}

	}
#endif

	return 0;
}

static int ft5526_notify(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
//	struct ft5526_data *d = to_ft5526_data(dev);
	u8 status = 0x00;
	int ret = 0;

	TOUCH_TRACE();
	TOUCH_I("%s event=0x%x\n", __func__, (unsigned int)event);
	switch (event) {
#if 0
	case NOTIFY_TOUCH_RESET:
		TOUCH_I("NOTIFY_TOUCH_RESET! return = %d\n", ret);
		atomic_set(&d->init, IC_INIT_NEED);
//		atomic_set(&d->watch.state.rtc_status, RTC_CLEAR);
		break;
	case LCD_EVENT_LCD_MODE:
		TOUCH_I("LCD_EVENT_LCD_MODE!\n");
		ft5526_lcd_mode(dev, *(u32 *)data);
		ret = ft5526_check_mode(dev);
		if (ret == 0)
			queue_delayed_work(ts->wq, &d->fb_notify_work, 0);
		else
			ret = 0;
		break;
	case LCD_EVENT_READ_REG:
		TOUCH_I("LCD_EVENT_READ_REG\n");
		ft5526_lcd_event_read_reg(dev);
		break;
#endif
	case NOTIFY_CONNECTION:
		TOUCH_I("NOTIFY_CONNECTION!\n");
		ret = ft5526_usb_status(dev, *(u32 *)data);
//		if (ft5526_asc_usable(dev))	/* ASC */
//			ft5526_asc_toggle_delta_check(dev);
		break;
#if 0
	case NOTIFY_WIRELEES:
		TOUCH_I("NOTIFY_WIRELEES!\n");
		ret = ft5526_wireless_status(dev, *(u32 *)data);
//		if (ft5526_asc_usable(dev))	/* ASC */
//			ft5526_asc_toggle_delta_check(dev);
		break;
	case NOTIFY_EARJACK:
		TOUCH_I("NOTIFY_EARJACK!\n");
		ret = ft5526_earjack_status(dev, *(u32 *)data);
		break;
#endif
	case NOTIFY_IME_STATE:
		TOUCH_I("NOTIFY_IME_STATE!\n");
		status = atomic_read(&ts->state.ime);
		ret = ft5526_reg_write(dev, REG_IME_STATE, &status, sizeof(status));
		if (ret)
			TOUCH_E("failed to write reg_ime_state, ret : %d\n", ret);
		break;
#if 0
	case NOTIFY_DEBUG_TOOL:
		ret = ft5526_debug_tool(dev, *(u32 *)data);
		TOUCH_I("NOTIFY_DEBUG_TOOL!\n");
		break;
	case NOTIFY_CALL_STATE:
		TOUCH_I("NOTIFY_CALL_STATE!\n");
		ret = ft5526_reg_write(dev, REG_CALL_STATE,
			(u32 *)data, sizeof(u32));
//		if (ft5526_asc_usable(dev))	/* ASC */
//			ft5526_asc_toggle_delta_check(dev);
		break;
	case NOTIFY_DEBUG_OPTION:
		TOUCH_I("NOTIFY_DEBUG_OPTION!\n");
		ret = ft5526_debug_option(dev, (u32 *)data);
		break;
	case NOTIFY_ONHAND_STATE:
		TOUCH_I("NOTIFY_ONHAND_STATE!\n");
//		if (ft5526_asc_usable(dev)) {	/* ASC */
//			ft5526_asc_toggle_delta_check(dev);
//			ft5526_asc_write_onhand(dev, *(u32 *)data);
//		}
		break;
#endif
/*	case NOTIFY_QUICKCOVER_STATE:
		TOUCH_I("NOTIFY_QUICKCOVER_STATE!: %d\n", *(u8 *)data);
		ret = ft5526_reg_write(dev, SPR_QUICKCOVER_STS, (u8 *)data, sizeof(u8));
		break; */
	default:
		TOUCH_E("%lu is not supported\n", event);
		break;
	}

	return ret;
}

#if 0
static void ft5526_init_works(struct ft5526_data *d)
{
	d->wq_log = create_singlethread_workqueue("touch_wq_log");

	if (!d->wq_log)
		TOUCH_E("failed to create workqueue log\n");
	else
		INIT_DELAYED_WORK(&d->debug_info_work, lg4946_debug_info_work_func);

//	INIT_DELAYED_WORK(&d->font_download_work, lg4946_font_download);
	INIT_DELAYED_WORK(&d->fb_notify_work, ft5526_fb_notify_work_func);
}
#endif

static void ft5526_init_locks(struct ft5526_data *d)
{
	mutex_init(&d->rw_lock);
	mutex_init(&d->fb_lock);
}

static int ft5526_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = NULL;
	int ret = 0;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate ft5526 data\n");
		return -ENOMEM;
	}

	d->dev = dev;
	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_gpio_init(ts->maker_id_pin, "touch_make_id");
	touch_gpio_direction_input(ts->maker_id_pin);

	touch_power_init(dev);
	touch_bus_init(dev, MAX_BUF_SIZE);

//	ft5526_init_works(d);
	ft5526_init_locks(d);

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		touch_gpio_init(ts->reset_pin, "touch_reset");
		touch_gpio_direction_output(ts->reset_pin, 1);
		/* Deep Sleep */
		touch_msleep(100); // ???????????????????????
		ft5526_deep_sleep(dev, 1);
		return 0;
	}

	ts->vdd = regulator_get(dev, "vdd");
	ret = regulator_set_voltage(ts->vdd, 2800000, 2800000);
	ret = regulator_enable(ts->vdd);
	if (ret != 0) {
		TOUCH_E("Failed to set regulator voltage: %d\n", ret);
	}

	ft5526_get_tci_info(dev);

	d->tci_debug_type = TCI_DEBUG_BUFFER;
	atomic_set(&ts->state.debug_option_mask, DEBUG_OPTION_2);

	// To be implemented.....
#ifdef FTS_CTL_IIC
	fts_rw_iic_drv_init(to_i2c_client(dev));
#endif
#ifdef FTS_SYSFS_DEBUG
	fts_create_sysfs(to_i2c_client(dev));
#endif
#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(to_i2c_client(dev));
#endif

	return 0;
}

static int ft5526_remove(struct device *dev)
{
	TOUCH_TRACE();
#ifdef FTS_APK_DEBUG
		fts_release_apk_debug_channel();
#endif

#ifdef FTS_SYSFS_DEBUG
		fts_remove_sysfs(to_i2c_client(dev));
#endif

#ifdef FTS_CTL_IIC
		fts_rw_iic_drv_exit();
#endif
	return 0;
}

static int ft5526_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	u8 ic_fw_version = d->ic_info.fw_version;
	u8 ic_is_official = d->ic_info.is_official;
	u8 bin_fw_version = 0;
	u8 bin_is_official = 0;
	int update = 0;
	u32 fw_size = (u32)(fw->size);

	if(d->ic_info.info_valid == 0) { // Failed to get ic info
		TOUCH_I("invalid ic info, skip fw upgrade\n");
		return 0;
	}

	bin_fw_version = fw->data[fw_size-2];
	bin_is_official = (bin_fw_version & 0x80) >> 7;
	bin_fw_version &= 0x7F;

	if(d->ic_info.fw_version_bin == 0 && d->ic_info.is_official_bin == 0) { // IF fw ver of bin is not initialized
		d->ic_info.fw_version_bin = bin_fw_version;
		d->ic_info.is_official_bin = bin_is_official;
	}

//	u32 bin_ver_offset = *((u32 *)&fw->data[0xe8]);
//	u32 bin_pid_offset = *((u32 *)&fw->data[0xf0]);

//	if ((bin_ver_offset > FLASH_FW_SIZE) || (bin_pid_offset > FLASH_FW_SIZE)) {
//		TOUCH_I("%s : invalid offset\n", __func__);
//		return -1;
//	}

	if (ts->force_fwup) {
		update = 1;
	} else if ((ic_is_official != bin_is_official) || (ic_fw_version != bin_fw_version)) {
		update = 1;
	}

	TOUCH_I("%s : binary[V%d.%d] device[V%d.%d]" \
		" -> update: %d, force: %d\n", __func__,
		bin_is_official, bin_fw_version, ic_is_official, ic_fw_version,
		update, ts->force_fwup);

	return update;
}

static int ft5526_fwboot_upgrade(struct device *dev, const struct firmware *fw_boot)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const u8 *fw_data = fw_boot->data;
	u32 fw_size = (u32)(fw_boot->size);
	u8 *fw_check_buf = NULL;
	u8 i2c_buf[FTS_PACKET_LENGTH + 12] = {0,};
	int ret;
	int packet_num, i, j, packet_addr, packet_len;
	u8 pramboot_ecc;

	TOUCH_I("%s - START\n", __func__);

	if(fw_size > 0x10000 || fw_size == 0)
		return -EIO;

	fw_check_buf = kmalloc(fw_size+1, GFP_ATOMIC);
	if(fw_check_buf == NULL)
		return -ENOMEM;

	for (i = 12; i <= 30; i++) {
		// Reset CTPM
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(50);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(i);

		// Set Upgrade Mode
		ret = ft5526_reg_write(dev, 0x55, i2c_buf, 0);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
		touch_msleep(1);

		// Check ID
		TOUCH_I("%s - Set Upgrade Mode and Check ID : %d ms\n", __func__, i);
		ret = ft5526_reg_read(dev, 0x90, i2c_buf, 2);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}

		TOUCH_I("Check ID : 0x%x , 0x%x\n", i2c_buf[0], i2c_buf[1]);
		if(i2c_buf[0] == 0x54&& i2c_buf[1] == 0x22){
			touch_msleep(50);
			break;
		}
	}

	if (i > 30) {
		TOUCH_E("timeout to set upgrade mode\n");
		goto FAIL;
	}

	// Write F/W (Pramboot) Binary to CTPM
	TOUCH_I("%s - Write F/W (Pramboot)\n", __func__);
	pramboot_ecc = 0;
	packet_num = (fw_size-8 + FTS_PACKET_LENGTH - 1) / FTS_PACKET_LENGTH;   // FW SIZE -8
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[0] = 0; //(u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
		for (j = 0; j < packet_len; j++) {
			i2c_buf[5 + j] = fw_data[packet_addr + j];
			//i2c_buf[5 + (j/4)*4 + (3 - (j%4))] = fw_data[packet_addr + j];        //Kylin20170315 for LGE
			pramboot_ecc ^= i2c_buf[5 + j];
		}
		//TOUCH_I("#%d : Writing to %d , %d bytes\n", i, packet_addr, packet_len); //kjh
		ret = ft5526_reg_write(dev, 0xAE, i2c_buf, packet_len + 5);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
	}

	packet_addr = 0xD7F8;
	i2c_buf[0] = 0;;
	i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
	i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
	packet_len = 8;
	i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
	i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
	for (j = 0; j < packet_len; j++) {
		i2c_buf[5 + j] = fw_data[(fw_size - 8) + j];
		pramboot_ecc ^= i2c_buf[5 + j];
	}
	//TOUCH_I("#%d : Writing to %d , %d bytes\n", i, packet_addr, packet_len);
	ret = ft5526_reg_write(dev, 0xAE, i2c_buf, packet_len + 5);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	touch_msleep(20);

	// Read out Checksum
#if 1  // Verify method 1: Check sum
	ret = ft5526_reg_read(dev, 0xCC, i2c_buf, 1);

	TOUCH_I("Make ecc: 0x%x ,Read ecc 0x%x\n", pramboot_ecc, i2c_buf[0]);

	if(i2c_buf[0] != pramboot_ecc){
		TOUCH_I("Pramboot Verify Failed !!\n" );
		goto FAIL;
	}
	TOUCH_I("%s - Pramboot write Verify OK !!\n", __func__);
#else  // Verify method 2: All data read and compare.
	TOUCH_I("%s - Verify\n", __func__);
	for (i = 0; i < packet_num; i++) {
		i2c_buf[0] = 0x85;
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[1] = 0; //(u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[2] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[3] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		//TOUCH_I("#%d : Reading from %d , %d bytes\n", i, packet_addr, packet_len);
		ret = ft5526_cmd_read(dev, i2c_buf, 4, fw_check_buf+packet_addr, packet_len);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
	}
	for (i = 0; i < fw_size; i++) {
		if(fw_check_buf[i] != fw_data[i]) {
			TOUCH_I("%s - Verify Failed !!\n", __func__);
			goto FAIL;
		}
	}
	TOUCH_I("%s - Pramboot write Verify OK !!\n", __func__);
#endif

	// Start App
	TOUCH_I("%s - Start App\n", __func__);
	ret = ft5526_reg_write(dev, 0x08, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	touch_msleep(10);
	if(fw_check_buf)
		kfree(fw_check_buf);

	TOUCH_I("===== Firmware (Pramboot) download Okay =====\n");

	return 0;

FAIL :

	if(fw_check_buf)
		kfree(fw_check_buf);

	TOUCH_I("===== Firmware (Pramboot) download FAIL!!! =====\n");

	return -EIO;

}

static int ft5526_fw_upgrade(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const u8 *fw_data = fw->data;
	u32 fw_size = (u32)(fw->size);
	u8 i2c_buf[FTS_PACKET_LENGTH + 12];
	int ret;
	int packet_num, retry, i, j, packet_addr, packet_len;
	u8 fw_ecc;
	//u8 retrim_val[4] = {0,};

	TOUCH_I("%s - START\n", __func__);

	// Reset CTPM
	TOUCH_I("%s - Reset CTPM\n", __func__);
	touch_msleep(50);

	i2c_buf[0] = 0xAA;
	ret = ft5526_reg_write(dev, 0xFC, i2c_buf, 1);
	if(ret < 0) {
	         TOUCH_E("i2c error\n");
	         return -EIO;
	}
	mdelay(5);

	i2c_buf[0] = 0x55;
	ret = ft5526_reg_write(dev, 0xFC, i2c_buf, 1);
	if(ret < 0) {
	         TOUCH_E("i2c error\n");
	         return -EIO;
	}
	touch_msleep(200);

	// Enter Upgrade Mode and ID check
	for (i = 0; i < 30; i++) {
		// Enter Upgrade Mode
		TOUCH_I("%s - Enter Upgrade Mode and Check ID\n", __func__);

		i2c_buf[0] = 0xAA;
		ret = ft5526_reg_write(dev, 0x55, i2c_buf, 1);

		if(ret < 0) {
			TOUCH_E("i2c error\n");
			return -EIO;
		}
		touch_msleep(1);

		// Check ID

		ret = ft5526_reg_read(dev, 0x90, i2c_buf, 2);

		if(ret < 0) {
			TOUCH_E("i2c error\n");
			return -EIO;
		}

		TOUCH_I("Check ID [%d] : 0x%x , 0x%x\n", i, i2c_buf[0], i2c_buf[1]);
		if(i2c_buf[0] == 0x54 && i2c_buf[1] == 0x2B)
			break;

		touch_msleep(10);
	}

	if (i == 30) {
		TOUCH_E("timeout to set upgrade mode\n");
		goto FAIL;
	}


// Change to write flash set range
	i2c_buf[0] = 0x0A;
	ret = ft5526_reg_write(dev, 0x09, i2c_buf, 1);
//- 0x0A : All
//- 0x0B : App
//- 0x0C : Lcd

	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	touch_msleep(50);

	// Erase start
	TOUCH_I("%s - Erase All  Area\n", __func__);

	ret = ft5526_reg_write(dev, 0x61, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	touch_msleep(1000);
	retry = 300;
	i = 0;
	do {
		ret = ft5526_reg_read(dev, 0x6A, i2c_buf, 2);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}

		if(i2c_buf[0] == 0xF0 && i2c_buf[1] == 0xAA)
		{
			TOUCH_I("Erase Done : %d \n", i);
			break;
		}
		i++;
		//touch_msleep(50);
		mdelay(10);
	} while (--retry);

	//  Write F/W (All or App) Binary to CTPM
	TOUCH_I("%s - Write F/W (App)\n", __func__);
	fw_ecc = 0;
	packet_num = (fw_size + FTS_PACKET_LENGTH - 1) / FTS_PACKET_LENGTH;
	for (i = 0; i < packet_num; i++) {
		packet_addr = i * FTS_PACKET_LENGTH;
		i2c_buf[0] = (u8)(((u32)(packet_addr) & 0x00FF0000) >> 16);
		i2c_buf[1] = (u8)(((u32)(packet_addr) & 0x0000FF00) >> 8);
		i2c_buf[2] = (u8)((u32)(packet_addr) & 0x000000FF);
		if(packet_addr + FTS_PACKET_LENGTH > fw_size)
			packet_len = fw_size - packet_addr;
		else
			packet_len = FTS_PACKET_LENGTH;
		i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
		i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
		for (j = 0; j < packet_len; j++) {
			i2c_buf[5 + j] = fw_data[packet_addr + j];
			fw_ecc ^= i2c_buf[5 + j];
		}
#if 0
		TOUCH_I("#%d : Writing to %d , %d bytes..[0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", i, packet_addr, packet_len, \
			i2c_buf[0], i2c_buf[1], i2c_buf[2], i2c_buf[3], i2c_buf[4]);
#endif
		ret = ft5526_reg_write(dev, 0xBF, i2c_buf, packet_len + 5);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
		//touch_msleep(10);

		retry = 100;
		do {
			ret = ft5526_reg_read(dev, 0x6A, i2c_buf, 2);
			if(ret < 0) {
				TOUCH_E("i2c error\n");
				goto FAIL;
			}

			if((u32)(i + 0x1000) == (((u32)(i2c_buf[0]) << 8) | ((u32)(i2c_buf[1]))))
			{
				if((i & 0x007F) == 0) {
					TOUCH_I("Write Done : %d / %d\n", i+1, packet_num);
				}
				break;
			}
			//touch_msleep(1);
			mdelay(1);
		} while (--retry);
		if(retry == 0) {
			TOUCH_I("Write Fail : %d / %d : [0x%02x] , [0x%02x]\n", i+1, packet_num, i2c_buf[0], i2c_buf[1]);
			//goto FAIL;
		}
	}
	TOUCH_I("Write Finished : Total %d\n", packet_num);

	touch_msleep(50);

	// Read out Checksum
	TOUCH_I("%s - Read out checksum (App) for %d bytes\n", __func__, fw_size);
	ret = ft5526_reg_write(dev, 0x64, i2c_buf, 0);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	touch_msleep(5);

	packet_len=0;
	i2c_buf[0] = (u8)(((u32)(packet_len) & 0x00FF0000) >> 16);
	i2c_buf[1] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
	i2c_buf[2] = (u8)((u32)(packet_len) & 0x000000FF);
	packet_len = fw_size;
	i2c_buf[3] = (u8)(((u32)(packet_len) & 0x0000FF00) >> 8);
	i2c_buf[4] = (u8)((u32)(packet_len) & 0x000000FF);
	ret = ft5526_reg_write(dev, 0x65, i2c_buf, 5);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	touch_msleep(200);

	retry = 200;
	do {
		ret = ft5526_reg_read(dev, 0x6A, i2c_buf, 2);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}

		if(i2c_buf[0] == 0xF0 && i2c_buf[1] == 0x55)
		{
			TOUCH_I("Checksum Calc. Done\n");
			break;
		}
		touch_msleep(10);
	} while (--retry);

	ret = ft5526_reg_read(dev, 0x66, i2c_buf, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	TOUCH_I("Reg 0x66 : 0x%x\n", i2c_buf[0]);

	if(i2c_buf[0] != fw_ecc)
	{
		TOUCH_E("Checksum ERROR : Reg 0x66 [0x%x] , fw_ecc [0x%x]\n", i2c_buf[0], fw_ecc);
		goto FAIL;
	}

	TOUCH_I("Checksum OK : Reg 0x66 [0x%x] , fw_ecc [0x%x]\n", i2c_buf[0], fw_ecc);

	TOUCH_I("===== Firmware download OK!!! =====\n");

	// reset the new FW
#if 0
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(50);
	touch_gpio_direction_output(ts->reset_pin, 1);
	//touch_msleep(ts->caps.hw_reset_delay);
#endif
	ret = ft5526_reg_write(dev, 0x07, i2c_buf, 0);  // SW_Reset
	msleep(200);

	return 0;

FAIL:

	TOUCH_I("===== Firmware download FAIL!!! =====\n");

	// Reset Anyway
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(50);
	touch_gpio_direction_output(ts->reset_pin, 1);
	//touch_msleep(ts->caps.hw_reset_delay);
	touch_msleep(300);

	ret = ft5526_reg_write(dev, 0x07, i2c_buf, 0);  // SW_Reset
	msleep(200);

	return -EIO;

}


static int ft5526_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	const struct firmware *fw = NULL;
	const struct firmware *fw_boot = NULL;
	char fwpath[256] = {0};
	int ret = 0;
//	int i = 0;

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if (ts->test_fwpath[0] && (ts->test_fwpath[0] != 't' && ts->test_fwpath[1] != 0)) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n", &ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		if(d->ic_info.chip_id_low == 0x06) {
			memcpy(fwpath, ts->def_fwpath[2], sizeof(fwpath)); // 0 : pramboot bin, 1 : 8607 all bin, 2: 8606 all bin
		} else {
			memcpy(fwpath, ts->def_fwpath[1], sizeof(fwpath)); // 0 : pramboot bin, 1 : 8607 all bin, 2: 8606 all bin
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
		TOUCH_I("fail to request_firmware fwpath: %s (ret:%d)\n", fwpath, ret);
		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	if (ft5526_fw_compare(dev, fw)) {

//		ret = -EINVAL;
//		touch_msleep(200);
//		for (i = 0; i < 2 && ret; i++)
//			ret = ft5526_fw_upgrade(dev, fw);

		TOUCH_I("fwpath_boot[%s]\n", ts->def_fwpath[0]);  // PARAMBOOT.BIN
		ret = request_firmware(&fw_boot, ts->def_fwpath[0], dev);
		if (ret < 0) {
			TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", ts->def_fwpath[0], ret);
			release_firmware(fw);
			return ret;
		}

		//touch_msleep(200);

		ret = ft5526_fwboot_upgrade(dev, fw_boot);
		if(ret < 0) {
			TOUCH_E("fail to upgrade f/w (pramboot) : %d\n", ret);
			release_firmware(fw);
			release_firmware(fw_boot);
			return -EPERM;
		}

		ret = ft5526_fw_upgrade(dev, fw);
		if(ret < 0) {
			TOUCH_E("fail to upgrade f/w : %d\n", ret);
			release_firmware(fw);
			release_firmware(fw_boot);
			return -EPERM;
		}



		TOUCH_I("f/w upgrade complete\n");

		release_firmware(fw_boot);

	}

	release_firmware(fw);

	return -EPERM; // Return non-zero to not reset
}

static int ft5526_suspend(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	//struct ft5526_data *d = to_ft5526_data(dev);
	//int mfts_mode = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE)
		return -EPERM;
#if 0
	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
		touch_interrupt_control(dev, INTERRUPT_DISABLE);
		ft5526_power(dev, POWER_OFF);
		return -EPERM;
	} else {
		TOUCH_I("%s : touch_suspend start\n", __func__);
//		if (d->lcd_mode == LCD_MODE_U2 &&
//			atomic_read(&d->watch.state.rtc_status) == RTC_RUN &&
//			d->watch.ext_wdata.time.disp_waton)
//				ext_watch_get_current_time(dev, NULL, NULL);
	}

//	if (atomic_read(&d->init) == IC_INIT_DONE)
//		ft5526_lpwg_mode(dev);
//	else /* need init */
//		ret = 1;
#endif

	return ret;
}

static int ft5526_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	int mfts_mode = 0;

	TOUCH_TRACE();

	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		#if 0
		ft5526_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		ft5526_ic_info(dev);
		if (ft5526_upgrade(dev) == 0) {
			ft5526_power(dev, POWER_OFF);
			ft5526_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);
		}
		//return -EPERM;
		#endif
		// Activate IC is done in fb callback EARLY BLANK (POWER_OFF => ACTIVE)
		// Return 0 to do init in touch_resume
		if (d->state == TC_STATE_ACTIVE)
			return 0;
		else
			return -EPERM;
	}
	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		//ft5526_deep_sleep(dev, 1);
		return -EPERM;
	}

//	if (atomic_read(&d->init) == IC_INIT_DONE)
//		ft5526_lpwg_mode(dev);
//	else /* need init */
//		ret = 1;

	return 0;
}

static int ft5526_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
//	u32 data = 1;
	u8 data = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		TOUCH_I("ft5526 fb_notif register\n");
		//fb_unregister_client(&ts->fb_notif);
		d->fb_notif.notifier_call = ft5526_fb_notifier_callback;
		d->fb_notif.priority = ts->fb_notif.priority + 1; // To call ft5526 fb notif in advance
		fb_register_client(&d->fb_notif);
		//fb_register_client(&ts->fb_notif);
	}

	touch_msleep(50);
	TOUCH_I("%s: charger_state = 0x%02X\n", __func__, d->charger);

//	if (atomic_read(&ts->state.debug_tool) == DEBUG_TOOL_ENABLE)
//		ft5526_sic_abt_init(dev);

	ret = ft5526_ic_info(dev);
	if (ret < 0) {
		TOUCH_I("failed to get ic_info, ret:%d\n", ret);
		atomic_set(&d->init, IC_INIT_DONE); // Nothing to init, anyway DONE init

		return 0;
	}

	ret = ft5526_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u8));
	if (ret)
		TOUCH_E("failed to write \'spr_charger_sts\', ret:%d\n", ret);

	data = atomic_read(&ts->state.ime);
	ret = ft5526_reg_write(dev, REG_IME_STATE, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'reg_ime_state\', ret:%d\n", ret);

	atomic_set(&d->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);
	d->state = TC_STATE_ACTIVE;

//	ft5526_lpwg_mode(dev, 1);
	ft5526_lpwg_mode(dev);
	if (ret)
		TOUCH_E("failed to lpwg_control, ret:%d\n", ret);

	if(apk_allocate_flag == 1) {

#ifdef CONFIG_MTK_I2C_EXTENSION
		msg_dma_alloct(dev);
#endif


#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT

		if (NULL == tpd_i2c_dma_va)
		{
		touch_msleep(3000);
			if (ts->input != NULL ) {

				dma_set_coherent_mask(&ts->input->dev, DMA_BIT_MASK(32));
				ts->input->dev.coherent_dma_mask = DMA_BIT_MASK(32);
				tpd_i2c_dma_va = (u8 *)dma_alloc_coherent(&ts->input->dev, 250, &tpd_i2c_dma_pa, GFP_KERNEL);
				//tpd_i2c_dma_va = (u8 *)dma_alloc_coherent(NULL, 250, &tpd_i2c_dma_pa, GFP_KERNEL);
			} else {
				TOUCH_E ( "ts->input is NULL\n");
			}
		}
		if (!tpd_i2c_dma_va)
			TOUCH_E("dma_alloc_coherent error!\n");
		else
			TOUCH_I("dma_alloc_coherent success!\n");

#endif


#ifdef CONFIG_MTK_I2C_EXTENSION
		//msg_dma_release();
#endif

		apk_allocate_flag = 0;

	}

	return 0;
}

#if 0
 ////// USED IN IRQ HANDLER

/* (1 << 5) | (1 << 9)|(1 << 10) */
#define INT_RESET_CLR_BIT	0x620
/* (1 << 6) | (1 << 7)|(1 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_LOGGING_CLR_BIT	0x50A0C0
/* (1 << 5) |(1 << 6) |(1 << 7)|(0 << 9)|(0 << 10)|(0 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_NORMAL_MASK		0x5080E0
#define IC_DEBUG_SIZE		16	/* byte */

int ft5526_check_status(struct device *dev)
{
	struct ft5526_data *d = to_ft5526_data(dev);
	int ret = 0;
	u32 status = d->info.device_status;
	u32 ic_status = d->info.ic_status;
	u32 debugging_mask = 0x0;
	u8 debugging_length = 0x0;
	u32 debugging_type = 0x0;
	u32 status_mask = 0x0;
	int checking_log_flag = 0;
	const int checking_log_size = DEBUG_BUF_SIZE;
	char checking_log[DEBUG_BUF_SIZE] = {0};
	int length = 0;

	status_mask = status ^ INT_NORMAL_MASK;
	if (status_mask & INT_RESET_CLR_BIT) {
		TOUCH_I("%s : Need Reset, status = %x, ic_status = %x\n",
			__func__, status, ic_status);
		ret = -ERESTART;
	} else if (status_mask & INT_LOGGING_CLR_BIT) {
		TOUCH_I("%s : Need Logging, status = %x, ic_status = %x\n",
			__func__, status, ic_status);
		ret = -ERANGE;
	}

	if (!(status & (1 << 5))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[5]Device_ctl not Set");
	}
	if (!(status & (1 << 6))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[6]Code CRC Invalid");
	}
	if (!(status & (1 << 7))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[7]CFG CRC Invalid");
	}
	if (status & (1 << 9)) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[9]Abnormal status Detected");
	}
	if (status & (1 << 10)) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[10]System Error Detected");
	}
	if (status & (1 << 13)) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[13]Display mode Mismatch");
	}
	if (!(status & (1 << 15))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[15]Interrupt_Pin Invalid");
	}
	if (!(status & (1 << 20))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[20]Touch interrupt status Invalid");
	}
	if (!(status & (1 << 22))) {
		checking_log_flag = 1;
		length += snprintf(checking_log + length,
			checking_log_size - length, "[22]TC driving Invalid");
	}

	if (checking_log_flag) {
		TOUCH_E("%s, status = %x, ic_status = %x\n",
				checking_log, status, ic_status);
	}

	if ((ic_status & 1) || (ic_status & (1 << 3))) {
		TOUCH_I("%s : Watchdog Exception - status : %x, ic_status : %x\n",
			__func__, status, ic_status);
		ret = -ERESTART;
	}

	if (ret == -ERESTART)
		return ret;

	debugging_mask = ((status >> 16) & 0xF);
	if (debugging_mask == 0x2) {
		TOUCH_I("TC_Driving OK\n");
		ret = -ERANGE;
	} else if (debugging_mask == 0x3 || debugging_mask == 0x4) {
		debugging_length = ((d->info.debug.ic_debug_info >> 24) & 0xFF);
		debugging_type = (d->info.debug.ic_debug_info & 0x00FFFFFF);
		TOUCH_I(
			"%s, INT_TYPE:%x,Length:%d,Type:%x,Log:%x %x %x\n",
			__func__, debugging_mask,
			debugging_length, debugging_type,
			d->info.debug.ic_debug[0], d->info.debug.ic_debug[1],
			d->info.debug.ic_debug[2]);
		ret = -ERANGE;
	}

	return ret;
}
#endif

#if 0
int ft5526_debug_info(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	struct ft5526_touch_debug *debug = &d->info.debug;
	int ret = 0;
	int i = 0;
	u8 buf[DEBUG_BUF_SIZE] = {0,};

	u16 debug_change_mask = 0;
	u16 press_mask = 0;
	u16 release_mask = 0;
	debug_change_mask = ts->old_mask ^ ts->new_mask;
	press_mask = ts->new_mask & debug_change_mask;
	release_mask = ts->old_mask & debug_change_mask;

	/* check protocol ver */
	if ((debug->protocol_ver < 0) && !(atomic_read(&d->init) == IC_INIT_DONE))
		return ret;

	/* check debugger status */
	if ((atomic_read(&ts->state.earjack) == EARJACK_DEBUG) ||
		(gpio_get_value(126) < 1))
		return ret;

#if 0
	if (!mode) {
		if ((debug_change_mask && press_mask)
				|| (debug_change_mask && release_mask)) {
			if (debug_detect_filter(dev, ts->tcount)) {
				/* disable func in irq handler */
				atomic_set(&ts->state.debug_option_mask,
						(atomic_read(&ts->state.debug_option_mask)
						 ^ DEBUG_OPTION_2));
				d->frame_cnt = debug->frame_cnt;
				queue_delayed_work(d->wq_log, &d->debug_info_work, DEBUG_WQ_TIME);
			}
			if ((debug->rebase[0] > 0)
					&& (debug_change_mask && press_mask)
					&& (ts->tcount < 2))
				goto report_debug_info;
		}
	}

	if (mode) {
		u8 debug_data[264];

		if (lg4946_reg_read(dev, tc_ic_status, &debug_data[0],
					sizeof(debug_data)) < 0) {
			TOUCH_I("debug data read fail\n");
		} else {
			memcpy(&d->info.debug, &debug_data[132], sizeof(d->info.debug));
		}

		if (debug->frame_cnt - d->frame_cnt > DEBUG_FRAME_CNT) {
			TOUCH_I("frame cnt over\n");
			return -1;
		}

		goto report_debug_info;
	}
#endif
	return ret;

//report_debug_info:
		ret += snprintf(buf + ret, DEBUG_BUF_SIZE - ret,
				"[%d] %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
				ts->tcount - 1,
				debug->protocol_ver,
				debug->frame_cnt,
				debug->rn_max_bfl,
				debug->rn_max_afl,
				debug->rn_min_bfl,
				debug->rn_min_afl,
				debug->rn_max_afl_x,
				debug->rn_max_afl_y,
				debug->seg1_cnt,
				debug->seg2_cnt,
				debug->seg1_thr,
				debug->rn_pos_cnt,
				debug->rn_neg_cnt,
				debug->rn_pos_sum,
				debug->rn_neg_sum,
				debug->rn_stable
			       );

		for (i = 0 ; i < ts->tcount ; i++) {
			if (i < 1)
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						" tb:");
			ret += snprintf(buf + ret, DEBUG_BUF_SIZE - ret,
					"%2d ",	debug->track_bit[i]);
		}

		for (i = 0 ; i < sizeof(debug->rn_max_tobj) ; i++) {
			if (debug->rn_max_tobj[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" to:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug->rn_max_tobj[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug->rebase) ; i++) {
			if (debug->rebase[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" re:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug->rebase[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug->noise_detect) ; i++) {
			if (debug->noise_detect[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" nd:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug->noise_detect[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug->lf_oft) ; i++) {
			if (debug->lf_oft[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" lf:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2x ",	debug->lf_oft[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug->palm) ; i++) {
			if (debug->palm[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" pa:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",	debug->palm[i]);
			} else {
				break;
			}
		}
		TOUCH_I("%s\n", buf);

		return ret;
}
#endif

static int ft5526_irq_abs_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	struct ft5526_touch_data *data = d->info.data;
	struct touch_data *tdata;
	u32 touch_count = 0;
	u8 finger_index = 0;
//	int ret = 0;
	int i = 0;
	u8 touch_id, event, palm;

	touch_count = d->info.touch_cnt;
	ts->new_mask = 0;
/*
	if (get_smartcover_status() && !ts->lpwg.qcover) {
		TOUCH_D(QUICKCOVER, "Skip irq while quickcover is closed\n");
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_NONE;

		touch_report_all_event(ts);
		return 0;
	}
*/
	for (i = 0; i < FTS_MAX_POINTS; i++) {
		touch_id = (u8)(data[i].yh) >> 4;
		if (touch_id >= FTS_MAX_ID) {
			break; // ??
		}

		event = (u8)(data[i].xh) >> 6;
		palm = ((u8)(data[i].xh) >> 4) & 0x01;

		if (palm) {
			if (event == FTS_TOUCH_CONTACT) { // FTS_TOUCH_DOWN
				ts->is_cancel = 1;
				TOUCH_I("Palm Detected\n");
			}
			else if (event == FTS_TOUCH_UP) {
				ts->is_cancel = 0;
				TOUCH_I("Palm Released\n");
			}
			ts->tcount = 0;
			ts->intr_status = TOUCH_IRQ_FINGER;
			return 0;
		}

		if(event == FTS_TOUCH_DOWN || event == FTS_TOUCH_CONTACT) {
			ts->new_mask |= (1 << touch_id);
			tdata = ts->tdata + touch_id;

			tdata->id = touch_id;
			tdata->type = MT_TOOL_FINGER;
			tdata->x = ((u16)(data[i].xh & 0x0F))<<8 | (u16)(data[i].xl);
			tdata->y = ((u16)(data[i].yh & 0x0F))<<8 | (u16)(data[i].yl);
			tdata->pressure = (u8)(data[i].weight);
			if((event == FTS_TOUCH_DOWN) && (tdata->pressure==0x29)) {
				tdata->pressure = 0x30;
			}
			tdata->width_major = (u8)((data[i].area)>>4);
			tdata->width_minor = 0;
			tdata->orientation = 0;

			if (0 == tdata->width_major)
				tdata->width_major = 0x09;
			if (0 == tdata->pressure)
				tdata->width_major = 0x3f;
			if (pre_pressure[tdata->id] == tdata->pressure) {
				tdata->pressure++;
			}
			pre_pressure[tdata->id] = tdata->pressure;

			finger_index++;

			TOUCH_D(ABS, "tdata [id:%d e:%d x:%d y:%d z:%d - %d,%d,%d]\n",\
					tdata->id,\
					event,\
					tdata->x,\
					tdata->y,\
					tdata->pressure,\
					tdata->width_major,\
					tdata->width_minor,\
					tdata->orientation);

		}
	}

#ifdef FT5526_ESD_SKIP_WHILE_TOUCH_ON
	if (finger_index != finger_cnt) {
//		TOUCH_I("finger cnt changed from %d to %d\n", finger_cnt, finger_index);
		finger_cnt = finger_index;
	}
#endif

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return 0;

#if 0

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

	for (i = 0; i < touch_count; i++) {
		if (data[i].track_id >= MAX_FINGER)
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

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return ret;
#endif

}

int ft5526_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	struct ft5526_touch_data *data = d->info.data;

	u8 point_buf[POINT_READ_BUF] = { 0, };
	int ret = -1;

	ret = ft5526_reg_read(dev, 0, point_buf, POINT_READ_BUF);

	if (ret < 0) {
		TOUCH_E("Fail to read point regs.\n");
		return ret;
	}

	/* check if touch cnt is valid */
	if (/*point_buf[FTS_TOUCH_P_NUM] == 0 || */point_buf[FTS_TOUCH_P_NUM] > ts->caps.max_id) {
		TOUCH_I("%s : touch cnt is invalid - %d\n",
			__func__, point_buf[FTS_TOUCH_P_NUM]);
		return -ERANGE;
	}

	d->info.touch_cnt = point_buf[FTS_TOUCH_P_NUM];

	memcpy(data, point_buf+FTS_TOUCH_EVENT_POS, FTS_ONE_TCH_LEN * FTS_MAX_POINTS);

	return ft5526_irq_abs_data(dev);


#if 0

	/* check if touch cnt is valid */
	if (d->info.touch_cnt == 0 || d->info.touch_cnt > ts->caps.max_id) {
		TOUCH_I("%s : touch cnt is invalid - %d\n",
			__func__, d->info.touch_cnt);
		return -ERANGE;
	}

	return ft5526_irq_abs_data(dev);
#endif
}

int ft5526_irq_lpwg(struct device *dev, int tci)
{
	struct touch_core_data *ts = to_touch_core(dev);
	//struct ft5526_data *d = to_ft5526_data(dev);
	int ret = 0, tap_count, i, j;
	u8 tci_data_buf[MAX_TAP_COUNT*4 + 2];

	if(ts->lpwg.mode == LPWG_NONE || (ts->lpwg.mode == LPWG_DOUBLE_TAP && tci == TCI_2)) {
		TOUCH_I("lpwg irq is invalid!!\n");
		return -1;
	}

	ret = ft5526_reg_read(dev, 0xD3, tci_data_buf, 2);
	if (ret < 0) {
		TOUCH_E("Fail to read tci data\n");
		return ret;
	}

	TOUCH_I("TCI Data : TCI[%d], Result[%d], TapCount[%d]\n", tci, tci_data_buf[0], tci_data_buf[1]);

	// Validate tci data
	if (!((tci_data_buf[0] == 0x01 && tci == TCI_1) || (tci_data_buf[0] == 0x02 && tci == TCI_2))
		|| tci_data_buf[1] == 0 || tci_data_buf[1] > MAX_TAP_COUNT) {
		TOUCH_I("tci data is invalid!!\n");
		return -1;
	}

	tap_count = tci_data_buf[1];

	ret = ft5526_reg_read(dev, 0xD3, tci_data_buf, tap_count*4 + 2);
	if (ret < 0) {
		TOUCH_E("Fail to read tci data\n");
		return ret;
	}

	ts->lpwg.code_num = tap_count;
	for (i = 0; i < tap_count; i++) {
		j = i*4+2;
		ts->lpwg.code[i].x = ((int)tci_data_buf[j] << 8) | (int)tci_data_buf[j+1];
		ts->lpwg.code[i].y = ((int)tci_data_buf[j+2] << 8) | (int)tci_data_buf[j+3];

		if ( (ts->lpwg.mode >= LPWG_PASSWORD) && (ts->role.hide_coordinate) )
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n", ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[tap_count].x = -1;
	ts->lpwg.code[tap_count].y = -1;

	if(tci == TCI_1)
		ts->intr_status = TOUCH_IRQ_KNOCK;
	else if(tci == TCI_2)
		ts->intr_status = TOUCH_IRQ_PASSWD;
	else
		ts->intr_status = TOUCH_IRQ_NONE;

	return ret;
}

int ft5526_irq_report_tci_fr(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	int ret = 0;
	u8 data, tci_1_fr, tci_2_fr;

	if (d->tci_debug_type != TCI_DEBUG_ALWAYS && d->tci_debug_type != TCI_DEBUG_BUFFER_ALWAYS) {
		TOUCH_I("tci debug in real time is disabled!!\n");
		return 0;
	}

	ret = ft5526_reg_read(dev, 0xE6, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	tci_1_fr = data & 0x0F; // TCI_1
	tci_2_fr = (data & 0xF0) >> 4; // TCI_2

	if (tci_1_fr < TCI_FR_NUM) {
		TOUCH_I("Knock-on Failure Reason Reported : [%s]\n", tci_debug_str[tci_1_fr]);
	}
	else {
		TOUCH_I("Knock-on Failure Reason Reported : [%s]\n", tci_debug_str[TCI_FR_NUM]);
	}

	if (tci_2_fr < TCI_FR_NUM) {
		TOUCH_I("Knock-code Failure Reason Reported : [%s]\n", tci_debug_str[tci_2_fr]);
	}
	else {
		TOUCH_I("Knock-code Failure Reason Reported : [%s]\n", tci_debug_str[TCI_FR_NUM]);
	}

	return ret;
}

int ft5526_report_tci_fr_buffer(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft5526_data *d = to_ft5526_data(dev);
	int ret = 0, i;
	u8 tci_fr_buffer[1 + TCI_FR_BUF_LEN], tci_fr_cnt, tci_fr;

	if (d->tci_debug_type != TCI_DEBUG_BUFFER && d->tci_debug_type != TCI_DEBUG_BUFFER_ALWAYS) {
		TOUCH_I("tci debug in buffer is disabled!!\n");
		return 0;
	}

	// Knock-on
	for (i = 0; i < 25; i++) {
		ret = ft5526_reg_read(dev, 0xE7, &tci_fr_buffer, sizeof(tci_fr_buffer));
		if (!ret) {
			break;
		}
		msleep(2);
	}
	if (i == 25) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	tci_fr_cnt = tci_fr_buffer[0];
	if (tci_fr_cnt > TCI_FR_BUF_LEN) {
		TOUCH_I("Knock-on Failure Reason Buffer Count Invalid\n");
	}
	else if (tci_fr_cnt == 0) {
		TOUCH_I("Knock-on Failure Reason Buffer NONE\n");
	}
	else {
		for (i = 0; i < tci_fr_cnt; i++) {
			tci_fr = tci_fr_buffer[1 + i];
			if (tci_fr < TCI_FR_NUM) {
				TOUCH_I("Knock-on Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[tci_fr]);
			}
			else {
				TOUCH_I("Knock-on Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[TCI_FR_NUM]);
			}
		}
	}

	// Knock-code (Same as knock-on case except for reg addr)
	for (i = 0; i < 25; i++) {
		ret = ft5526_reg_read(dev, 0xE9, &tci_fr_buffer, sizeof(tci_fr_buffer));
		if (!ret) {
			break;
		}
		msleep(2);
	}
	if (i == 25) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	tci_fr_cnt = tci_fr_buffer[0];
	if (tci_fr_cnt > TCI_FR_BUF_LEN) {
		TOUCH_I("Knock-code Failure Reason Buffer Count Invalid\n");
	}
	else if (tci_fr_cnt == 0) {
		TOUCH_I("Knock-code Failure Reason Buffer NONE\n");
	}
	else {
		for (i = 0; i < tci_fr_cnt; i++) {
			tci_fr = tci_fr_buffer[1 + i];
			if (tci_fr < TCI_FR_NUM) {
				TOUCH_I("Knock-code Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[tci_fr]);
			}
			else {
				TOUCH_I("Knock-code Failure Reason Buffer [%02d] : [%s]\n", i+1, tci_debug_str[TCI_FR_NUM]);
			}
		}
	}

	return ret;
}


int ft5526_irq_handler(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	//struct ft5526_data *d = to_ft5526_data(dev);
	int ret = 0;
	u8 int_status = 0;

	ret = ft5526_reg_read(dev, 0x01, &int_status, 1);
	if (ret < 0)
		return ret;

//	ret = ft5526_irq_abs(dev);

	if (int_status == 0x01) { // Finger
		ret = ft5526_irq_abs(dev);
	}
	else if (int_status == 0x02) { // TCI_1
		ret = ft5526_irq_lpwg(dev, TCI_1);
	}
	else if (int_status == 0x03) { // TCI_2
		ret = ft5526_irq_lpwg(dev, TCI_2);
	}
	else if (int_status == 0x04) { // LPWG Fail Reason Report (RT)
		ret = ft5526_irq_report_tci_fr(dev);
	}
/*
	else if (int_status == 0x05) { // ESD
		TOUCH_I("ESD interrupt !!\n");
#if defined(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
		ret = -ERESTART;
#endif
	}
*/
	else if (int_status == 0x00) {
		TOUCH_I("No interrupt status\n");
	}
	else {
		TOUCH_E("Invalid interrupt status : %d\n", int_status);
#if defined(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
		ret = -ERESTART;
#endif

	}

	return ret;

}

static ssize_t store_reg_ctrl(struct device *dev,
				const char *buf, size_t count)
{
	char command[6] = {0};
	int reg = 0;
	int value = 0;
	u8 data = 0;
	u8 reg_addr;

	if (sscanf(buf, "%5s %x %x", command, &reg, &value) <= 0)
		return count;

	reg_addr = (u8)reg;
	if (!strcmp(command, "write")) {
		data = (u8)value;
		if (ft5526_reg_write(dev, reg_addr, &data, sizeof(u8)) < 0)
			TOUCH_E("reg addr 0x%x write fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else if (!strcmp(command, "read")) {
		if (ft5526_reg_read(dev, reg_addr, &data, sizeof(u8)) < 0)
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
	struct ft5526_data *d = to_ft5526_data(dev);
	int ret = 0;

	ret = snprintf(buf + ret, PAGE_SIZE,
			"Current TCI Debug Type = %s\n",
			(d->tci_debug_type < 4) ? tci_debug_type_str[d->tci_debug_type] : "Invalid");

	TOUCH_I("Current TCI Debug Type = %s\n",
			(d->tci_debug_type < 4) ? tci_debug_type_str[d->tci_debug_type] : "Invalid");

	return ret;
}

static ssize_t store_tci_debug(struct device *dev,
						const char *buf, size_t count)
{
	struct ft5526_data *d = to_ft5526_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0 || value < 0 || value > 3) {
		TOUCH_I("Invalid TCI Debug Type, please input 0~3\n");
		return count;
	}

	d->tci_debug_type = (u8)value;

	TOUCH_I("Set TCI Debug Type = %s\n", (d->tci_debug_type < 4) ? tci_debug_type_str[d->tci_debug_type] : "Invalid");

	return count;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	touch_interrupt_control(dev, INTERRUPT_DISABLE);
	ft5526_reset_ctrl(dev, value);

	ft5526_init(dev);
	touch_interrupt_control(dev, INTERRUPT_ENABLE);

	return count;
}

static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(tci_debug, show_tci_debug, store_tci_debug);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);

static struct attribute *ft5526_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_tci_debug.attr,
	&touch_attr_reset_ctrl.attr,
	NULL,
};

static const struct attribute_group ft5526_attribute_group = {
	.attrs = ft5526_attribute_list,
};

static int ft5526_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &ft5526_attribute_group);
	if (ret < 0)
		TOUCH_E("ft5526 sysfs register failed\n");

//	ft5526_watch_register_sysfs(dev);
	ft5526_prd_register_sysfs(dev);
//	ft5526_asc_register_sysfs(dev);	/* ASC */
//	ft5526_sic_abt_register_sysfs(&ts->kobj);

	return 0;
}

static int ft5526_get_cmd_version(struct device *dev, char *buf)
{
#if 1
	struct ft5526_data *d = to_ft5526_data(dev);
	int offset = 0;
	int ret = 0;

	ret = ft5526_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = snprintf(buf + offset, PAGE_SIZE - offset, "IC firmware info\n");
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "Version : V%d.%02d\n\n",
		d->ic_info.is_official, d->ic_info.fw_version);

	offset += snprintf(buf + offset, PAGE_SIZE - offset, "Bin firmware info\n");
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "Version : V%d.%02d\n\n",
		d->ic_info.is_official_bin, d->ic_info.fw_version_bin);

	offset += snprintf(buf + offset, PAGE_SIZE - offset, "Product_id : [FT%x%x]\n\n", d->ic_info.product_id[0], d->ic_info.product_id[1]);

	return offset;
#else
	struct ft5526_data *d = to_ft5526_data(dev);
	int offset = 0;
	int ret = 0;
	u32 rdata[4] = {0};

	ret = ft5526_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	if (d->ic_info.version.build) {
		offset = snprintf(buf + offset, PAGE_SIZE - offset, "version : v%d.%02d.%d\n",
			d->ic_info.version.major, d->ic_info.version.minor, d->ic_info.version.build);
	} else {
		offset = snprintf(buf + offset, PAGE_SIZE - offset, "version : v%d.%02d\n",
			d->ic_info.version.major, d->ic_info.version.minor);
	}

	if (d->ic_info.revision == 0xFF) {
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"revision : Flash Erased(0xFF)\n");
	} else {
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"revision : %d\n", d->ic_info.revision);
	}

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
		"fpc : %d, cg : %d, wfr : %d\n", d->ic_info.fpc, d->ic_info.cg, d->ic_info.wfr);

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
		"product id : [%s]\n\n", d->ic_info.product_id);

	ft5526_reg_read(dev, info_lot_num, (u8 *)&rdata, sizeof(rdata));
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "lot : %d\n", rdata[0]);
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "serial : 0x%X\n", rdata[1]);
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "date : %04d.%02d.%02d " \
		"%02d:%02d:%02d Site%d\n",
		rdata[2] & 0xFFFF, (rdata[2] >> 16 & 0xFF), (rdata[2] >> 24 & 0xFF),
		rdata[3] & 0xFF, (rdata[3] >> 8 & 0xFF), (rdata[3] >> 16 & 0xFF),
		(rdata[3] >> 24 & 0xFF));

	return offset;
#endif
}

static int ft5526_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct ft5526_data *d = to_ft5526_data(dev);
	int offset = 0;
	int ret = 0;

	ret = ft5526_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = snprintf(buf, PAGE_SIZE, "V%d.%02d\n",
		d->ic_info.is_official, d->ic_info.fw_version);

	return offset;
}

static int ft5526_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int ft5526_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = ft5526_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = ft5526_get_cmd_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}

static struct touch_driver touch_driver = {
	.probe = ft5526_probe,
	.remove = ft5526_remove,
	.suspend = ft5526_suspend,
	.resume = ft5526_resume,
	.init = ft5526_init,
	.irq_handler = ft5526_irq_handler,
	.power = ft5526_power,
	.upgrade = ft5526_upgrade,
	.lpwg = ft5526_lpwg,
	.notify = ft5526_notify,
	.register_sysfs = ft5526_register_sysfs,
	.set = ft5526_set,
	.get = ft5526_get,
};

#define MATCH_NAME			"focaltech,ft5526"

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
/*
static bool ft5526_get_device_type(void)
{
	bool bdevice = false;
	enum lge_panel_type panel_type = touch_get_device_type();

	TOUCH_I("%s [lge_get_panel_type] = [%d]\n", __func__, panel_type);

	switch(panel_type) {
		case LV3_TIANMA:
			bdevice = true;
			break;
		default:
			break;
	}

	return bdevice;
}
*/

static void __init touch_device_async_init(void *data, async_cookie_t cookie)
{
       touch_bus_device_init(&hwif, &touch_driver);
       TOUCH_TRACE();
}

static int __init touch_device_init(void)
{
	TOUCH_TRACE();
/*
	if (!ft5526_get_device_type()) {
		TOUCH_I("%s, ft5526 not found\n", __func__);
		return 0;
	}
*/
	//return touch_bus_device_init(&hwif, &touch_driver);
	return async_schedule(touch_device_async_init, NULL);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
//late_initcall(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("hyokmin.kwon@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
