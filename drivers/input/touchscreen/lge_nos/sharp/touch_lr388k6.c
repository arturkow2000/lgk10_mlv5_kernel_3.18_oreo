/* touch_sharp.c
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>

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

int sharp_read(struct device *dev, u8 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);
	return 0;
}

int sharp_write(struct device *dev, u8 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size + 1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		return ret;
	}

	return 0;
}

int sharp_set_bank(struct device *dev, u8 bank)
{
	u8 value = bank;

	sharp_write(dev, SHTSC_ADDR_BANK, &value, sizeof(value));
	TOUCH_I("set bank %d\n", value);

	return 0;
}

int sharp_set_indicator(struct device *dev, u8 bank)
{
	u8 value = bank;

	sharp_write(dev, SHTSC_ADDR_IND, &value, sizeof(value));
	TOUCH_I("set indicator %d\n", value);

	return 0;
}

void sharp_clear_interrupt(struct device *dev, u16 val)
{
	struct sharp_data *d = to_sharp_data(dev);

	u8 value = 0;

	TOUCH_I("%s %04X \n", __func__, val);

	if (val & 0x00FF) {
		value = val & 0x00FF;
		sharp_write(dev, SHTSC_ADDR_INT0, &value, sizeof(value));
	}

	if (val & 0xFF00) {
		value = (val & 0xFF00) >> 8;
		sharp_write(dev, SHTSC_ADDR_INT1, &value, sizeof(value));
	}

	d->info.irq_status &= ~val;
}

int sharp_ic_info(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	u8 value = 0;
	int count = 0;

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_COMMAND);

	/* Set Command */
	sharp_write(dev, SHTSC_ADDR_COMMAND, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	/* Set Incdicator */
	sharp_set_indicator(dev, SHTSC_IND_CMD);
	touch_msleep(100);

	sharp_read(dev, SHTSC_ADDR_INT0, &value, sizeof(value));
	TOUCH_I("1st Value check value %d, SHTSC_STATUS_COMMAND_RESULT & value %d \n",
			value, (SHTSC_STATUS_COMMAND_RESULT & value));

	/* waiting the result of get_property */
	while (!(SHTSC_STATUS_COMMAND_RESULT & value)) {
		touch_msleep(5);
		count++;

		if (count > 10)
			break;
	}

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_COMMAND_RESULT);

	/* Read Result */
	sharp_read(dev, 0x08, d->cmd_buf, MAX_COMMAND_RESULT_LEN);
	TOUCH_I("Cmd result(GetProperty[E0]), Operation code: %02X, Error code:%02X\n",
			d->cmd_buf[0], d->cmd_buf[1]);

	d->fw[0].firm_ver = ((d->cmd_buf[0x0a - 0x08 + 3] << 24) |
			(d->cmd_buf[0x0a - 0x08 + 2] << 16) |
			(d->cmd_buf[0x0a - 0x08 + 1] << 8) |
			(d->cmd_buf[0x0a - 0x08 + 0] << 0));
	d->fw[0].param_ver = ((d->cmd_buf[0x12 - 0x08 + 3] << 24) |
			(d->cmd_buf[0x12 - 0x08 + 2] << 16) |
			(d->cmd_buf[0x12 - 0x08 + 1] << 8) |
			(d->cmd_buf[0x12 - 0x08 + 0] << 0));
	TOUCH_I("Firmware %08X, Parameter %08X\n", d->fw[0].firm_ver, d->fw[0].param_ver);

	/* Clear Interrupt */
	sharp_clear_interrupt(dev, SHTSC_STATUS_COMMAND_RESULT);

	d->fw[0].moduleMakerID = 2;
	d->fw[0].moduleVersion = 0;
	d->fw[0].modelID = 3;
	d->fw[0].isOfficial = 1;
	d->fw[0].version = d->cmd_buf[0x12 - 0x08];

	TOUCH_I("IC f/w version %d \n", d->fw[0].version);

	if(d->fw[0].version == 0) {
		touch_msleep(50);
		d->fw[0].param_ver = ((d->cmd_buf[0x12-0x08 +3] << 24) |
				(d->cmd_buf[0x12-0x08 +2] << 16) |
				(d->cmd_buf[0x12-0x08 +1] << 8) |
				(d->cmd_buf[0x12-0x08 +0] << 0));
		TOUCH_I("paramver %08X \n", d->fw[0].param_ver);
	}

	return 0;
}

static int sharp_wait_async(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	int i;

	TOUCH_I("%s\n", __func__);

	for (i = 0 ; i < 50 ; i++) {
		touch_msleep(CMD_DELAY);	/* 16ms */
		switch(d->wait_state) {
		case WAIT_RESET:
			break;
		case WAIT_CMD:
			break;
		case WAIT_NONE:
			if (d->wait_result == true) {
				TOUCH_I("shtsc: wait state change: success\n");
				return 0;
			} else {
				TOUCH_E("shtsc: wait state change: fail\n");
				return -EIO;
			}
		default:
			break;
		}
	}

	return -EIO;
}

static int sharp_set_command(struct device *dev,
					unsigned char *cmd, unsigned int len)
{
	struct sharp_data *d = to_sharp_data(dev);

	int ret = 0;

	TOUCH_I("%s\n", __func__);

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_COMMAND);

	/* Set Command */
	sharp_write(dev, SHTSC_ADDR_COMMAND, cmd, len);
	TOUCH_I("set command (%x)\n", cmd[0]);

	d->cmd = cmd[0];
	d->wait_state = WAIT_CMD;
	d->wait_result = true;

	/* Set Indicator */
	sharp_set_indicator(dev, SHTSC_IND_CMD);

	ret = sharp_wait_async(dev);

	return ret;
}

static int sharp_tci_control(struct device *dev, u8 addr, unsigned int value)
{
	u8 data = 0;
	u8 buffer[2];
	int ret;

	TOUCH_I("%s\n", __func__);

	if (addr == KNOCK_1) {
		sharp_read(dev, SHTSC_ADDR_INTMASK0, &data, sizeof(data));
		if (value)
			data &= ~SHTSC_STATUS_LG_LPWG_TCI1;
		else
			data |= SHTSC_STATUS_LG_LPWG_TCI1;
		sharp_read(dev, SHTSC_ADDR_INTMASK0, &data, sizeof(data));
		return 0;
	} else if (addr == KNOCK_2) {
		sharp_read(dev, SHTSC_ADDR_INTMASK0, &data, sizeof(data));
		if (value)
			data &= ~SHTSC_STATUS_LG_LPWG_TCI2;
		else
			data |= SHTSC_STATUS_LG_LPWG_TCI2;
		sharp_read(dev, SHTSC_ADDR_INTMASK0, &data, sizeof(data));
		return 0;
	}

	buffer[0] = value & 0xFF;
	buffer[1] = (value >> 8) & 0xFF;

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_LPWG_PARAM);

	sharp_write(dev, addr, buffer, sizeof(buffer));
	TOUCH_I("tci set LPWG_PARM command (0x%x) (0x%x) \n", buffer[0], buffer[1]);

	return ret;
}

static int sharp_tci_active_area(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	u8 buffer[2];

	TOUCH_I("%s\n", __func__);
	TOUCH_I("LPWG AREA (%d,%d)-(%d,%d)\n",
			ts->lpwg.area[0].x, ts->lpwg.area[0].y,
			ts->lpwg.area[1].x, ts->lpwg.area[1].y);

	/* Set bank */
	sharp_set_bank(dev, SHTSC_BANK_LPWG_PARAM);

	buffer[0] = (ts->lpwg.area[0].x >> 0) & 0xff;
	buffer[1] = (ts->lpwg.area[0].x >> 8) & 0xff;
	sharp_write(dev, ACTIVE_AREA_X1_CTRL_REG, buffer, sizeof(buffer));

	buffer[0] = (ts->lpwg.area[0].y >> 0) & 0xff;
	buffer[1] = (ts->lpwg.area[0].y >> 8) & 0xff;
	sharp_write(dev, ACTIVE_AREA_Y1_CTRL_REG, buffer, sizeof(buffer));

	buffer[0] = (ts->lpwg.area[1].x >> 0) & 0xff;
	buffer[1] = (ts->lpwg.area[1].x >> 8) & 0xff;
	sharp_write(dev, ACTIVE_AREA_X2_CTRL_REG, buffer, sizeof(buffer));

	buffer[0] = (ts->lpwg.area[1].y >> 0) & 0xff;
	buffer[1] = (ts->lpwg.area[1].y >> 8) & 0xff;
	sharp_write(dev, ACTIVE_AREA_Y2_CTRL_REG, buffer, sizeof(buffer));

	return 0;
}

static int sharp_tci_knock(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info_1 = &ts->tci.info[0];

	TOUCH_I("%s\n", __func__);

	if (mode == 0x02)
		info_1->intr_delay = 255;

	sharp_tci_control(dev, TAP_COUNT_CTRL_REG, info_1->tap_count);
	sharp_tci_control(dev, MIN_INTERTAP_CTRL_REG, info_1->min_intertap);
	sharp_tci_control(dev, MAX_INTERTAP_CTRL_REG, info_1->max_intertap);
	sharp_tci_control(dev, TOUCH_SLOP_CTRL_REG, info_1->touch_slop);
	sharp_tci_control(dev, TAP_DISTANCE_CTRL_REG, info_1->tap_distance);
	sharp_tci_control(dev, INTERRUPT_DELAY_CTRL_REG, info_1->intr_delay);
	sharp_tci_active_area(dev);
	sharp_tci_control(dev, FAILURE_INT_ENABLE_REG, KNOCK_CODE_FAILURE_REASONS);

	return 0;
}

static int sharp_tci_password(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info_2 = &ts->tci.info[1];

	TOUCH_I("%s\n", __func__);

	info_2->tap_count = ts->lpwg.code_num;

	sharp_tci_knock(dev, ts->lpwg.mode);

	sharp_tci_control(dev, TAP_COUNT_CTRL2_REG, info_2->tap_count);
	sharp_tci_control(dev, MIN_INTERTAP_CTRL2_REG, info_2->min_intertap);
	sharp_tci_control(dev, MAX_INTERTAP_CTRL2_REG, info_2->max_intertap);
	sharp_tci_control(dev, TOUCH_SLOP_CTRL2_REG, info_2->touch_slop);
	sharp_tci_control(dev, TAP_DISTANCE_CTRL2_REG, info_2->tap_distance);
	sharp_tci_control(dev, INTERRUPT_DELAY_CTRL2_REG, info_2->intr_delay);
	sharp_tci_control(dev, FAILURE_INT_ENABLE2_REG, KNOCK_CODE_FAILURE_REASONS);

	return 0;
}

static int sharp_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("sharp_lpwg_control mode=%d\n", mode);

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		sharp_tci_knock(dev, ts->tci.mode);
		sharp_tci_control(dev, KNOCK_1, 1);
		sharp_tci_control(dev, KNOCK_2, 0);
		sharp_set_command(dev, CMD_SETSYSTEMSTATE_DEEPIDLE,
					CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x02;
		sharp_tci_password(dev, ts->tci.mode);
		sharp_tci_control(dev, KNOCK_1, 1);
		sharp_tci_control(dev, KNOCK_2, 1);
		sharp_set_command(dev, CMD_SETSYSTEMSTATE_DEEPIDLE,
					CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
		break;

	default:
		ts->tci.mode = 0;
		break;
	}

	return 0;
}

static int sharp_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->lpwg.mode == LPWG_NONE) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep\n",
				__func__, __LINE__);
			sharp_lpwg_control(dev, LPWG_NONE);
		} else if (ts->lpwg.screen) {
			TOUCH_I("%s(%d) - FB_SUSPEND & screen on -> skip\n",
				__func__, __LINE__);
			return 0;
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep by prox\n",
				__func__, __LINE__);
			sharp_lpwg_control(dev, LPWG_NONE);
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			/* knock on */
			TOUCH_I("%s(%d) - knock on by hole\n",
				__func__, __LINE__);
			sharp_lpwg_control(dev, LPWG_DOUBLE_TAP);
		} else {
			/* knock on/code */
			TOUCH_I("%s(%d) - knock %d\n",
				__func__, __LINE__, ts->lpwg.mode);
			sharp_lpwg_control(dev, ts->lpwg.mode);
		}
		return 0;
	}

	/* resume */
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("%s(%d) - normal\n",
				__func__, __LINE__);
		sharp_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* normal */
		TOUCH_I("%s(%d) - normal on screen off\n",
				__func__, __LINE__);
		sharp_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		/* wake up */
		TOUCH_I("%s(%d) - wake up on screen off and prox\n",
				__func__, __LINE__);
		TOUCH_I("%s - wake up is not ready\n", __func__);
		sharp_lpwg_control(dev, LPWG_NONE);
	} else {
		/* partial */
		touch_report_all_event(ts);
		TOUCH_I("%s(%d) - parial mode\n",
				__func__, __LINE__);
		TOUCH_I("%s - partial is not ready\n", __func__);
		sharp_lpwg_control(dev, LPWG_NONE);
	}

	return 0;
}

static void sharp_init_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("%s\n", __func__);

	ts->tci.info[0].tap_count = 2;
	ts->tci.info[0].min_intertap = 0;
	ts->tci.info[0].max_intertap = 500;
	ts->tci.info[0].touch_slop = 100;
	ts->tci.info[0].tap_distance = 100;
	ts->tci.info[0].intr_delay = 0;

	ts->tci.info[1].tap_count = 3;
	ts->tci.info[1].min_intertap = 0;
	ts->tci.info[1].max_intertap = 500;
	ts->tci.info[1].touch_slop = 100;
	ts->tci.info[1].tap_distance = 1700;
	ts->tci.info[1].intr_delay = 0;
}

static int sharp_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sharp_data *d;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate sharp data\n");
		return -ENOMEM;
	}

	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 1);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, 4096);

	sharp_init_tci_info(dev);

	d->info.irq_mask = 0xffff;
	d->wait_state = WAIT_NONE;
	d->wait_result = false;

	return 0;
}

static int sharp_remove(struct device *dev)
{
	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);
	return 0;
}


static int sharp_suspend(struct device *dev)
{
	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);
	sharp_lpwg_mode(dev);

	return 0;
}

static int sharp_resume(struct device *dev)
{
	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);
	sharp_lpwg_mode(dev);

	return 0;
}

static int sharp_get_object_count(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sharp_data *d = to_sharp_data(dev);

	int ret = 0;

	if (d->info.reg_data[SHTSC_ADDR_BANK] != SHTSC_BANK_TOUCH_REPORT) {
		/* Set Bank */
		sharp_set_bank(dev, SHTSC_BANK_TOUCH_REPORT);
		sharp_read(dev, SHTSC_ADDR_TOUCH_NUM,
				d->info.reg_data + SHTSC_ADDR_TOUCH_NUM, 3);
	}
	d->info.object_count = d->info.reg_data[SHTSC_ADDR_TOUCH_NUM];

	if (d->info.object_count > ts->caps.max_id || d->info.object_count < 0)
		return -EINVAL;

	ret = sharp_read(dev, SHTSC_ADDR_TOUCH_REPORT, d->info.buf,
				SHTSC_LENGTH_OF_TOUCH * d->info.object_count);

	if (ret < 0) {
		TOUCH_E("faied to read finger data\n");
		return ret;
	}

	return 0;
}

static int sharp_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sharp_data *d = to_sharp_data(dev);
	struct sharp_touch_data *data = d->info.data;
	struct touch_data *tdata;

	u8 finger_index = 0;
	u8 state = 0;
	int i = 0;

	ts->new_mask = 0;
	sharp_get_object_count(dev);

	if (d->info.object_count > 0) {
		finger_index = 0;

		for (i = 0 ; i < d->info.object_count; i++) {
			state = d->info.buf[i * SHTSC_LENGTH_OF_TOUCH] & 0xF0;
			data[i].id = d->info.buf[i * SHTSC_LENGTH_OF_TOUCH] & 0x0F;
			data[i].size = d->info.buf[i * SHTSC_LENGTH_OF_TOUCH + 1] & 0x3F;
			data[i].type = (d->info.buf[i * SHTSC_LENGTH_OF_TOUCH + 1] & 0xC0) >> 6;
			data[i].x = (d->info.buf[i * SHTSC_LENGTH_OF_TOUCH + 2] << 0) |
					(d->info.buf[i * SHTSC_LENGTH_OF_TOUCH + 3] << 8);
			data[i].y = (d->info.buf[i * SHTSC_LENGTH_OF_TOUCH + 4] << 0) |
					(d->info.buf[i * SHTSC_LENGTH_OF_TOUCH + 5] << 8);
			data[i].z = (d->info.buf[i * SHTSC_LENGTH_OF_TOUCH + 6] << 0) |
					(d->info.buf[i * SHTSC_LENGTH_OF_TOUCH + 7] << 8);

			if (state & SHTSC_TOUCHOUT_STATUS) {
				data[i].status = SHTSC_F_TOUCH_OUT;
				continue;
			}

			ts->new_mask |= (1 << i);
			tdata = ts->tdata + i;

			tdata->id = data[i].id;
			tdata->type = data[i].type;
			tdata->x = data[i].x;
			tdata->y = data[i].y;
			tdata->pressure = ((data[i].z) > 4095)
						? (0xff) : ((data[i].z & 0x0ff0) >> 4);
			tdata->width_major = data[i].size;
			tdata->width_minor = data[i].size;

			finger_index++;

			TOUCH_D(ABS,
				"tdata [id:%d, s:%x, t:%d, x:%d, t:%d, z:%d-%d,%d\n",
					tdata->id,
					state,
					tdata->type,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor);
		}
		ts->tcount = finger_index;
	}

	ts->intr_status = TOUCH_IRQ_FINGER;

	/* Clear Interrupt */
	d->info.irq_status &= ~SHTSC_STATUS_TOUCH_READY;
	d->info.reg_data[SHTSC_ADDR_INT0] = SHTSC_STATUS_TOUCH_READY;
	d->info.reg_data[SHTSC_ADDR_BANK] = SHTSC_BANK_TOUCH_REPORT;
	d->info.reg_data[SHTSC_ADDR_IND] = SHTSC_IND_TOUCH;
	sharp_write(dev, SHTSC_ADDR_INT0, d->info.reg_data, 4);

	return 0;
}

static void sharp_irq_power_up(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	/* Clear Interrupt */
	sharp_clear_interrupt(dev, SHTSC_STATUS_POWER_UP);

	if (d->wait_state == WAIT_RESET) {
		d->wait_state = WAIT_NONE;
		d->wait_result = true;
	}
}

static void sharp_irq_resume_prox(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_TOUCH_REPORT);
	sharp_read(dev, SHTSC_ADDR_RESUME_PROX,
			&(d->resume_state), sizeof(d->resume_state));

	/* Clear Interrupt */
	sharp_clear_interrupt(dev, SHTSC_STATUS_RESUME_PROX);
}


static void sharp_irq_wdt(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	/* Clear Interrupt */
	sharp_clear_interrupt(dev, SHTSC_STATUS_WDT);
	d->info.irq_mask = 0xffff;
}

static void sharp_irq_dcmap_ready(struct device *dev)
{
	/* Clear Interrupt */
	sharp_clear_interrupt(dev, SHTSC_STATUS_DCMAP_READY);
}

static void sharp_irq_cmd_result(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_COMMAND_RESULT);

	sharp_read(dev, 0x08, d->cmd_buf, sizeof(d->cmd_buf));

	/* Clear Interrupt */
	sharp_clear_interrupt(dev, SHTSC_STATUS_COMMAND_RESULT);

	if (d->wait_state == WAIT_CMD) {
		if ((d->cmd_buf[0] != d->cmd) || (d->cmd_buf[1] != 0)) {
			d->wait_state = WAIT_NONE;
			d->wait_result = false;
		} else {
			d->wait_state = WAIT_NONE;
			d->wait_result = true;
		}
	}
}

static int sharp_tci_getdata(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);

	u8 buf[KNOCKDATA_SIZE];
	int i, ret;

	/* set bank */
	sharp_set_bank(dev, SHTSC_BANK_LPWG_DATA);

	ret = sharp_read(dev, SHTSC_ADDR_LPWG_REPORT, buf, sizeof(buf));

	if (ret < 0)
		return ret;

	for (i = 0; i < count; i++) {
		ts->lpwg.code[i].x = (buf[i * SHTSC_LENGTH_OF_LPWG] << 0)
			| (buf[i * SHTSC_LENGTH_OF_LPWG + 1] << 8);
		ts->lpwg.code[i].y = (buf[i * SHTSC_LENGTH_OF_LPWG + 2] << 0)
			| (buf[i * SHTSC_LENGTH_OF_LPWG + 3] << 8);

		if ((ts->lpwg.mode == LPWG_PASSWORD) && (ts->role.hide_coordinate))
			TOUCH_I("LPWG data xxx, xxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n",
					ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}

	ts->lpwg.code[i].x = -1;
	ts->lpwg.code[i].y = -1;

	return 0;
}

static int sharp_irq_lpwg(struct device *dev, u8 type)
{
	struct touch_core_data *ts = to_touch_core(dev);

	u8 value;
	u8 result;

	/* set bank */
	sharp_set_bank(dev, SHTSC_BANK_LPWG_PARAM);

	sharp_read(dev, FAILURE_REASON_REG, &value, sizeof(value));

	if (type == KNOCK_1) {
		result = value & 0x0F;
	} else if (type == KNOCK_2) {
		result = (value & 0xF0) >> 4;
	}

	TOUCH_I("TCI_%d fail reason [%d]\n", type, result);

	switch (result) {
	case 0:
		TOUCH_I("TCI_%d SUCCESS\n", type);
		if (type == KNOCK_1) {
			sharp_tci_getdata(dev, 2);
			ts->intr_status = TOUCH_IRQ_KNOCK;
		} else if (type == KNOCK_2) {
			sharp_tci_getdata(dev, 4);
			ts->intr_status = TOUCH_IRQ_PASSWD;
		}
		break;
	case 1:
		TOUCH_E("TCI_%d FAIL - DISTANCE_INTER_TAP\n", type);
		break;
	case 2:
		TOUCH_E("TCI_%d FAIL - DISTANCE_TOUCHSLOP\n", type);
		break;
	case 3:
		TOUCH_E("TCI_%d FAIL - TIMEOUT_INTERTAP\n", type);
		break;
	case 4:
		TOUCH_E("TCI_%d FAIL - MULTI_FINGER\n", type);
		break;
	case 5:
		TOUCH_E("TCI_%d FAIL - DELAY_TIME\n", type);
		break;
	case 6:
		TOUCH_E("TCI_%d FAIL - PALM_STATE\n", type);
		break;
	default:
		TOUCH_E("TCI_%d FAIL - RESERVED\n", type);
		break;
	}

	if (type == KNOCK_1) {
		/* Clear Interrupt */
		sharp_clear_interrupt(dev, SHTSC_STATUS_LG_LPWG_TCI1);
	} else if (type == KNOCK_2) {
		/* Clear Interrupt */
		sharp_clear_interrupt(dev, SHTSC_STATUS_LG_LPWG_TCI2);
	}

	return 0;
}

static void sharp_irq_flash_error(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	if (d->wait_state == WAIT_RESET) {
		d->wait_state = WAIT_NONE;
		d->wait_result = true;
	}

	d->info.irq_mask &= ~SHTSC_STATUS_FLASH_LOAD_ERROR;

	/* Clear Interrupt */
	sharp_clear_interrupt(dev, SHTSC_STATUS_FLASH_LOAD_ERROR);
}

static void sharp_irq_pll_unlock(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	u8 value;

	value = 0x03;
	sharp_write(dev, SHTSC_ADDR_INTMASK1, &value, sizeof(value));
	d->info.reg_data[SHTSC_ADDR_INTMASK1] = 0x03;
	d->info.irq_mask &= ~SHTSC_STATUS_PLL_UNLOCK;

	/* Clear Interrupt */
	sharp_clear_interrupt(dev, SHTSC_STATUS_PLL_UNLOCK);
}

static void sharp_irq_unknown_state(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	u8 value;

	value = 0xFF;
	sharp_write(dev, SHTSC_ADDR_INTMASK0, &value, sizeof(value));
	value = 0x03;
	sharp_write(dev, SHTSC_ADDR_INTMASK1, &value, sizeof(value));

	d->info.irq_status = 0;
}

static int sharp_irq_handler(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	int ret = 0;

	ret = sharp_read(dev, SHTSC_ADDR_INT0,
				d->info.reg_data, sizeof(d->info.reg_data));

	if (ret < 0) {
		TOUCH_E("faied to read reg_data\n");
		return ret;
	}

	d->info.irq_status = (d->info.reg_data[SHTSC_ADDR_INT1] << 8)
			| d->info.reg_data[SHTSC_ADDR_INT0];

	d->info.irq_status &= d->info.irq_mask;

	if (d->info.irq_status & SHTSC_STATUS_TOUCH_READY) {
		sharp_irq_abs(dev);
	}

	if (d->info.irq_status & SHTSC_STATUS_POWER_UP) {
		TOUCH_I("shtsc_irq power-up\n");
		sharp_irq_power_up(dev);
	}

	if (d->info.irq_status & SHTSC_STATUS_RESUME_PROX) {
		TOUCH_I("shtsc_irq resume from deep_idle or prox\n");
		sharp_irq_resume_prox(dev);
	}

	if (d->info.irq_status & SHTSC_STATUS_WDT) {
		TOUCH_I("shtsc_irq WDT\n");
		sharp_irq_wdt(dev);
	}

	if (d->info.irq_status & SHTSC_STATUS_DCMAP_READY) {
		TOUCH_I("shtsc_irq dcmap ready\n");
		sharp_irq_dcmap_ready(dev);
	}

	if (d->info.irq_status & SHTSC_STATUS_COMMAND_RESULT) {
		TOUCH_I("shtsc_irq command reasult\n");
		sharp_irq_cmd_result(dev);
	}

	if (d->info.irq_status & SHTSC_STATUS_LG_LPWG_TCI1) {
		TOUCH_I("shtsc_irq knock on\n");
		sharp_irq_lpwg(dev, KNOCK_1);
	}

	if (d->info.irq_status & SHTSC_STATUS_LG_LPWG_TCI2) {
		TOUCH_I("shtsc_irq knock code");
		sharp_irq_lpwg(dev, KNOCK_2);
	}

	if (d->info.irq_status & SHTSC_STATUS_FLASH_LOAD_ERROR) {
		TOUCH_I("shtsc_irq flash load error\n");
		sharp_irq_flash_error(dev);
	}

	if (d->info.irq_status & SHTSC_STATUS_PLL_UNLOCK) {
		TOUCH_I("shtsc_irq PLL_UNLOCK: %04X\n", d->info.irq_status);
		sharp_irq_pll_unlock(dev);
	}

	if (d->info.irq_status != 0) {
		TOUCH_I("shtsc_irq unknown irq status %04X\n", d->info.irq_status);
		sharp_irq_unknown_state(dev);
	}

	return 0;
}

static int sharp_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sharp_data *d = to_sharp_data(dev);

	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);

	switch (ctrl) {
	case POWER_OFF:
		TOUCH_I("%s, off\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(50);
		touch_power_vio(dev, 0);
		touch_power_vdd(dev, 0);
		break;

	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		touch_power_vdd(dev, 1);
		touch_msleep(10);
		touch_power_vio(dev, 1);
		if (d->update)
			touch_msleep(250);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(200);
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

int sharp_init(struct device *dev)
{
	struct sharp_data *d = to_sharp_data(dev);

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	sharp_ic_info(dev);

	sharp_power(dev, POWER_OFF);
	d->info.irq_mask = 0xffff;
	sharp_power(dev, POWER_ON);
	d->update = 0;

	return 0;
}

static int sharp_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int *value = (int *)param;

	TOUCH_TRACE();

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->lpwg.area[0].x = value[0];
		ts->lpwg.area[0].y = value[2];
		ts->lpwg.area[1].x = value[1];
		ts->lpwg.area[1].y = value[3];
		TOUCH_I("LPWG AREA (%d,%d)-(%d,%d)\n",
			ts->lpwg.area[0].x, ts->lpwg.area[0].y,
			ts->lpwg.area[1].x, ts->lpwg.area[1].y);
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
		sharp_lpwg_mode(dev);
		break;

	case LPWG_TAP_COUNT:
		ts->lpwg.code_num = value[0];
		TOUCH_I("LPWG_TAP_COUNT (%d)\n", ts->lpwg.code_num);
		break;
	}

	return 0;
}

static int sharp_fw_compare(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sharp_data *d = to_sharp_data(dev);
	struct sharp_fw_info *ic = &d->fw[0];
	struct sharp_fw_info *bin = &d->fw[1];

	int update = 0;

	TOUCH_I("%s\n", __func__);

	if (ts->force_fwup) {
		update = 1;
	} else {
		if (ic->isOfficial == 0) {
			TOUCH_I("IC f/w version is test f/w[V%d.%02d]\n",
				ic->isOfficial, ic->version);
			update = 0;
		} else if (ic->isOfficial == 1) {
			if (ic->version != bin->version) {
				TOUCH_I("f/w version mismatch(ic[V%d.%02d], bin[V%d.%02d])\n",
					ic->isOfficial, ic->version,
					bin->isOfficial, bin->version);
				update = 1;
			} else {
				TOUCH_I("f/w version match(ic[V%d.%02d], bin[V%d.%02d])\n",
					ic->isOfficial, ic->version,
					bin->isOfficial, bin->version);
				update = 0;
			}
		}
	}

	return update;
}

static int sharp_upgrade(struct device *dev)
{

	struct touch_core_data *ts = to_touch_core(dev);
	struct sharp_data *d = to_sharp_data(dev);

	int ret = 0;
	char fwpath[256] = {0};
	const struct firmware *fw = NULL;
	const u8 *firmware = NULL;
	unsigned long image_size = 0;

	TOUCH_I("%s\n", __func__);

	if (!ts->force_fwup) {
		ts->test_fwpath[0] = '\0';
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n",
				&ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
		TOUCH_I("BIN firmware\n");
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

	firmware = fw->data;
	image_size = fw->size;

	TOUCH_I("success from image to buffer size %lu\n", image_size);
	TOUCH_I("file fw ver = %02x%02x%02x%02x, param ver = %02x%02x%02x%02x\n",
			firmware[0x23 + 7], firmware[0x22 + 7],
			firmware[0x21 + 7], firmware[0x20 + 7],
			firmware[image_size - 6], firmware[image_size - 7],
			firmware[image_size - 8], firmware[image_size - 9]);

	d->fw[1].moduleMakerID = 2;
	d->fw[1].moduleVersion = 0;
	d->fw[1].modelID = 3;
	d->fw[1].isOfficial = 1;
	d->fw[1].version = firmware[image_size - 9];
	TOUCH_I("BIN f/w version %d \n", d->fw[1].version);

	d->update = sharp_fw_compare(dev);

	if (d->update) {
		/* fw_upgrade */
		ret = sharp_fw_update(dev, fw);

		if (ret < 0) {
			release_firmware(fw);
			return ret;
		}
	} else {
		TOUCH_I("need not fw version upgrade\n");
	}

	release_firmware(fw);

	return 0;
}

static int sharp_get_cmd_version(struct device *dev, char *buf)
{
	struct sharp_data *d = to_sharp_data(dev);
	struct sharp_fw_info *ic = &d->fw[0];
	struct sharp_fw_info *bin = &d->fw[1];

	int offset = 0;

	TOUCH_I("%s\n", __func__);

	offset = snprintf(buf + offset, PAGE_SIZE - offset,
				"\n======== Firmware Info ========\n");

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"=== ic_fw_version info ===\n");

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"FW Version: %d.%02d\n\n",
				ic->isOfficial, ic->version);

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"=== bin_fw_version info ===\n");

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"BIN Version: %d.%02d\n\n",
				bin->isOfficial, bin->version);

	return offset;
}

static int sharp_get_atcmd_version(struct device *dev, char *buf)
{
	struct sharp_data *d = to_sharp_data(dev);
	struct sharp_fw_info *ic = &d->fw[0];

	int offset = 0;

	TOUCH_I("%s\n", __func__);

	offset = snprintf(buf + offset, PAGE_SIZE - offset,
				"V%d.%02d(%d/%d/%d)\n",
				ic->isOfficial, ic->version,
				ic->moduleMakerID, ic->moduleVersion,
				ic->modelID);

	return offset;
}

static int sharp_notify(struct device *dev, ulong event, void *data)
{
	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);
	return 0;
}

static ssize_t store_reg_ctrl(struct device *dev,
					const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);

	char command[6] = {0};
	int reg = 0;
	int value = 0, data = 0;
	int i;

	if (sscanf(buf, "%5s %d %d", command, &reg, &value) <= 0)
		return count;

	mutex_lock(&ts->lock);
	if (!strcmp(command, "write")) {
		sharp_write(dev, reg, &value, sizeof(value));
	} else if (!strcmp(command, "read")) {
		if (value == 0)
			value = 1;

		for (i = 0 ; i < value ; i++) {
			sharp_read(dev, reg, &data, sizeof(data));
			TOUCH_I("read to register (reg : 0x%04X, data : %d)\n",
					reg, data);
		}
	} else {
		TOUCH_E("Usage\n");
		TOUCH_E("Write reg value\n");
		TOUCH_E("Read reg value\n");
	}
	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t show_property(struct device *dev, char *buf)
{
	struct sharp_data *d = to_sharp_data(dev);

	int ret = 0;
	unsigned firmver = 0;
	unsigned paramver = 0;

	TOUCH_I("%s\n", __func__);

	/* Set Command */
	sharp_set_command(dev, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	firmver = ((d->cmd_buf[0x0a - 0x08 + 3] << 24) |
			(d->cmd_buf[0x0a - 0x08 + 2] << 16) |
			(d->cmd_buf[0x0a - 0x08 + 1] << 8) |
			(d->cmd_buf[0x0a - 0x08 + 0] << 0));
	paramver = ((d->cmd_buf[0x12 - 0x08 + 3] << 24) |
			(d->cmd_buf[0x12 - 0x08 + 2] << 16) |
			(d->cmd_buf[0x12 - 0x08 + 1] << 8) |
			(d->cmd_buf[0x12 - 0x08 + 0] << 0));

	ret = snprintf(buf + ret, PAGE_SIZE - ret,
				"Firmware %08X, Parameter %08X\n", firmver, paramver);

	TOUCH_I("Firmware %08X, Parameter %08X\n", firmver, paramver);

	return ret;
}

static int sharp_cmd_pen_mode(struct device *dev, int mode)
{
	u8 value = 0;
	int count = 0;

	TOUCH_I("%s\n", __func__);

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_COMMAND);

	/* Set Command */
	if (mode) {
		sharp_write(dev, SHTSC_ADDR_COMMAND, CMD_SETSYSTEMSTATE_PEN_MODE,
				CMD_SETSYSTEMSTATE_PEN_MODE_LEN);
	} else {
		sharp_write(dev, SHTSC_ADDR_COMMAND, CMD_SETSYSTEMSTATE_NORMAL_MODE,
				CMD_SETSYSTEMSTATE_NORMAL_MODE_LEN);
	}

	/* Set Indicator */
	sharp_set_indicator(dev, SHTSC_IND_CMD);

	sharp_read(dev, SHTSC_ADDR_INT0, &value, sizeof(value));
	TOUCH_I("Value check value %d, SHTSC_STATUS_COMMAND_RESULT & value %d \n",
			value, (SHTSC_STATUS_COMMAND_RESULT & value));

	/* waiting the result of get_property */
	while (!(SHTSC_STATUS_COMMAND_RESULT & value)) {
		touch_msleep(5);
		count++;

		if (count > 10)
			break;
	}

	return 0;
}

static ssize_t show_pen_mode(struct device *dev, char *buf)
{
	struct sharp_data *d = to_sharp_data(dev);

	int ret = 0;

	TOUCH_I("%s\n", __func__);

	ret = snprintf(buf + ret, PAGE_SIZE - ret,
				"PEN_MODE 1, NORMAL_MODE 0 : %d\n", d->pen_mode);

	TOUCH_I("PEN_MODE 1, NORMAL_MODE 0 : %d\n", d->pen_mode);

	return ret;
}

static ssize_t store_pen_mode(struct device *dev,
					const char *buf, size_t count)
{
	struct sharp_data *d = to_sharp_data(dev);

	int value = 0;

	TOUCH_I("%s\n", __func__);

	sscanf(buf, "%d", &value);

	if (value != 0 || value != 1) {
		TOUCH_I("invalid value %d\n", value);
		return count;
	}

	TOUCH_I("current mode : %d, change mode : %d\n", d->pen_mode, value);

	if (value) {
		if (!d->pen_mode) {
			sharp_cmd_pen_mode(dev, value);
			d->pen_mode = value;
		}
	} else {
		if (d->pen_mode) {
			sharp_cmd_pen_mode(dev, value);
			d->pen_mode = value;
		}
	}

	return count;
}

static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(property, show_property, NULL);
static TOUCH_ATTR(pen_mode, show_pen_mode, store_pen_mode);

static struct attribute *lr388k6_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_property.attr,
	&touch_attr_pen_mode.attr,
	NULL,
};

static const struct attribute_group lr388k6_attribute_group = {
	.attrs = lr388k6_attribute_list,
};

static int sharp_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int ret = 0;

	TOUCH_I("%s\n", __func__);

	ret = sysfs_create_group(&ts->kobj, &lr388k6_attribute_group);
	if (ret < 0)
		TOUCH_E("lr388k6 sysfs register failed\n");

	lr388k6_prd_register_sysfs(dev);

	return 0;
}

static int sharp_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int sharp_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_TRACE();
	TOUCH_I("%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = sharp_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = sharp_get_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}

static struct touch_driver touch_driver = {
	.probe = sharp_probe,
	.remove = sharp_remove,
	.suspend = sharp_suspend,
	.resume = sharp_resume,
	.init = sharp_init,
	.irq_handler = sharp_irq_handler,
	.power = sharp_power,
	.lpwg = sharp_lpwg,
	.notify = sharp_notify,
	.upgrade = sharp_upgrade,
	.register_sysfs = sharp_register_sysfs,
	.set = sharp_set,
	.get = sharp_get,
};


#define MATCH_NAME			"sharp,lr388k6"

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

MODULE_AUTHOR("keunyoung1.park@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
