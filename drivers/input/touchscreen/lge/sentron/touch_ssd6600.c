/* touch_synaptics.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
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
#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/file.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

/*
 *  Include to Local Header File
 */
#include "touch_ssd6600.h"

int m_power_status = LPM_RESUME;        // 0:resume , 1:suspend

int ssd6600_reg_read(struct device *dev, u16 addr, u8 *data, u16 rx_size)
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct touch_bus_msg msg;
        int ret = 0;
        int retry = 0;

        ts->tx_buf[0] = (addr & 0x00ff);
        ts->tx_buf[1] = (addr & 0xff00) >> 8;

        msg.tx_buf = ts->tx_buf;
        msg.tx_size = 2;

        msg.rx_buf = ts->rx_buf;
        msg.rx_size = rx_size;

        do{
                ret = touch_bus_read(dev, &msg);

                if (ret < 0) {
                        TOUCH_E("touch bus read error : %d\n", ret);
                } else {
                        if (retry)
                        TOUCH_I("touch bus read success : retry[%d]\n",retry);
                        break;
                }
        }while(++retry < 5);

        if(ret < 0)
                return ret;
    
        memcpy(data, &ts->rx_buf[0], rx_size);

        udelay(200);

        return 0;
}

int ssd6600_reg_read_ex(struct device *dev, u8 *addr, u16 tx_size, u8 *data, u16 rx_size)
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct touch_bus_msg msg;
        int ret = 0;
        int i = 0;
        int retry = 0;

        for (i = 0; i < tx_size; i++) {
                ts->tx_buf[i] = addr[i];
        }

        msg.tx_buf = ts->tx_buf;
        msg.tx_size = tx_size;

        msg.rx_buf = ts->rx_buf;
        msg.rx_size = rx_size;

        do{
                ret = touch_bus_read(dev, &msg);

                if (ret < 0) {
                        TOUCH_E("touch bus read error : %d\n", ret);
                } else {
                        if (retry)
                                TOUCH_I("touch bus read success : retry[%d]\n",retry);
                        break;
                }
        }while(++retry < 5);

        if(ret < 0)
                return ret;

        memcpy(data, &ts->rx_buf[0], rx_size);
    
        udelay(200);

        return 0;
}

int ssd6600_reg_write(struct device *dev, u16 addr, u8 *data, u16 size)
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct touch_bus_msg msg;
        int ret = 0;
        int retry = 0;

        ts->tx_buf[0] = (addr & 0x00ff);
        ts->tx_buf[1] = (addr & 0xff00) >> 8;
        memcpy(&ts->tx_buf[2], data, size);

        msg.tx_buf = ts->tx_buf;
        msg.tx_size = size+2;
        msg.rx_buf = NULL;
        msg.rx_size = 0;

        do{
                ret = touch_bus_write(dev, &msg);

                if (ret < 0) {
                        TOUCH_E("touch bus write error : %d\n", ret);
                } else {
                        if (retry)
                                TOUCH_I("touch bus write success : retry[%d]\n",retry);
                        break;
                }
        }while(++retry < 5);

        if(ret < 0)
                return ret;

        udelay(200);

        return 0;
}

static void write_file(struct device *dev, char *data)
{
	int fd = 0;
	mm_segment_t old_fs = get_fs();
	char *fname = "/mnt/sdcard/touch_self_test.txt";

	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);

	if (fd >= 0) {
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	} else {
		TOUCH_I("File open failed\n");
	}
	set_fs(old_fs);
}

static int ssd6600_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);	
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	int ret = 0;
	
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
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
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

        if (ts->force_fwup) {
                ret = SSD6600_firmware_update_byArr(dev, fw);
        } else {
                SSD6600_firmware_pre_boot_up_check(dev, fw);
        }

	release_firmware(fw);

	return ret;
}

static inline s32 ts_read_gesture(struct device *dev, u16 *gesture, u16 length)
{
        int err = 0;

        err = ssd6600_reg_read(dev, SENTRON_GET_GESTURE, (u8 *)(gesture), length);
        if ( err < 0 ) {
                TOUCH_E("error : read Gesture");
                return -EAGAIN;
        }

        TOUCH_I("read gesture : 0x%04x", *gesture);
        return err;
}

static int sentron_read_gesture( struct device *dev )
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct sentron_point points[SENTRON_MAX_POINT]={{0},};
        int err = 0;
        int i = 0;
        u16 arrGesture[GESTURE_READ_SIZE/2] = {0,};
        u16 gesture = 0;
        u16 TCI_length  = 0;
        u16 x, y;

        if (ts->lpwg.screen == 0 && ts->lpwg.mode) {
                if( (err = ts_read_gesture(dev, arrGesture, GESTURE_READ_SIZE)) >= 0 ) {
                        gesture = arrGesture[0];
                        TOUCH_I("length %d\n", arrGesture[1]);

                        if( (gesture & GESTURE_STATUS_TCI1) > 0 ) {
                                TCI_length = arrGesture[1];
                                ts->intr_status |= TOUCH_IRQ_PASSWD;
                                memcpy( (u8 *)(&points), (u8 *)(arrGesture+2), TCI_length*2*2 );
			
				for (i = 0; i < TCI_length; i++) {
					x = (points[i].id_x  & 0x0fff);
					y = (points[i].w_y  & 0x0fff);

					if ((ts->lpwg.mode == LPWG_PASSWORD) && (ts->role.hide_coordinate))
						TOUCH_I("LPWG data xxx, xxx\n");
					else
						TOUCH_I("LPWG[0x%04x] - %d TAP x[%3d] y[%3d] \n", gesture, i, x, y);

					ts->lpwg.code[i].x = x;
					ts->lpwg.code[i].y = y;
					ts->lpwg.code_num++;
				}
				ts->lpwg.code[i].x = -1;
				ts->lpwg.code[i].y = -1;
                        }   else if( (gesture & GESTURE_STATUS_TCI2) > 0 ) {
                                TCI_length = arrGesture[1];
                                ts->intr_status |= TOUCH_IRQ_KNOCK;
                                memcpy( (u8 *)(&points), (u8 *)(arrGesture+2), TCI_length*2*2 );

                                for (i = 0; i < TCI_length; i++) {
                                        x = (points[i].id_x  & 0x0fff);
                                        y = (points[i].w_y  & 0x0fff);

                                        TOUCH_I("LPWG[0x%04x] - %d TAP x[%3d] y[%3d] \n", gesture, i, x, y);

                                        ts->lpwg.code[i].x = x;
                                        ts->lpwg.code[i].y = y;
                                        ts->lpwg.code_num++;
                                }
                                ts->lpwg.code[i].x = -1;
                                ts->lpwg.code[i].y = -1;
                        } else {
				return -EFAULT;
			}
		}
	} else {
		TOUCH_E("LPWG - abnormal wakeup!!!\n");
		return -EIO;
        }

        return err;
}

static s32 int_pin_check(struct device *dev, int retry)
{
        struct touch_core_data *ts = to_touch_core(dev);
        int ret=0;

        if( dev == NULL ) return 0;

        if( retry == 0 ) return 0;

        do {
                if (gpio_get_value(ts->int_pin) == 0 ) break;
                mdelay(1);
        } while( (retry--) > 0 );

        if( retry < 1 ) ret = -1;

        return ret;
}

static inline s32 lpm_end(struct device *dev)
{
        s32 ret = 0;
        u32 temp_flag = 0x0000;
        int retry1 = 5;

again :
        if( (ret = ssd6600_reg_write(dev, DS_CUP_CONTROL, (u8 *)&temp_flag, 2)) < 0 ) {
                TOUCH_E("0x%04X i2c write fail(2)!!", DS_CUP_CONTROL);
        }

        retry1--;
        if( ret < 0 && retry1 > 0 ) goto again;

        return ret;
}

static inline s32 int_clear_cmd(struct device *dev)
{
        int val = 0x00;

        return ssd6600_reg_write(dev, SENTRON_INT_CLEAR_CMD, (u8 *)&val, 2);
}

static inline s32 lpm_end_clear(struct device *dev)
{
        int err = 0;
        int retry = 500;

        err = int_pin_check(dev, retry);
        int_clear_cmd(dev);

        return err;
}

static inline s32 lpm_end2(struct device *dev)
{
        s32 ret = 0;
        u32 temp_flag = 0x0000;
        int retry = 5;

        TOUCH_I("lpm end2\n");
again :
        ssd6600_reg_write(dev, DS_CUP_CONTROL, (u8 *)&temp_flag, 2);
        mdelay(1);
        if( (ret = ssd6600_reg_write(dev, DS_CUP_CONTROL, (u8 *)&temp_flag, 2)) < 0 ) {
                TOUCH_E("0x%04X i2c write fail(2)!!\n", DS_CUP_CONTROL);
        }

        retry--;
        if( ret < 0 && retry > 0 ) goto again;

        return ret;
}

static s32 sentron_run_mode(struct device *dev, u16 mode)
{
        int err=0x00;
    
        err = ssd6600_reg_write(dev, SENTRON_RUN_MODE, (u8 *)&(mode), 2);

        if (err < 0) {
                TOUCH_E("Fail to set SENTRON_RUN_MODE(%d).\n", mode);
        }

        return err;
}

static int sentron_set_tci_mode(struct device *dev, u16 mode)
{
        int err=0x00;

        err = ssd6600_reg_write(dev, SENTRON_LPWG_STATUS, (u8 *)&(mode), 2);
    
        if (err < 0) {
                TOUCH_E("Fail to set SENTRON_LPWG_STATUS.\n");
        }

        return err;
}

static int ssd6600_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("ssd6600_lpwg_control mode=%d, %s\n", mode, __func__);

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = LPWG_DOUBLE_TAP;
                sentron_set_tci_mode(dev, SENTRON_LPWG_STATUS_KNOCK_ON_ONLY);
        break;

	case LPWG_PASSWORD:
		ts->tci.mode = LPWG_PASSWORD;
                sentron_set_tci_mode(dev, SENTRON_LPWG_STATUS_KNOCK_ON_AND_CODE);
        break;

	default:
		ts->tci.mode = 0;
		sentron_set_tci_mode(dev, SENTRON_LPWG_STATUS_OFF);
        break;
	}

	return 0;
}

static int ssd6600_tci_active_area(struct device *dev)
{
        return 0;
}

static int ssd6600_set_lpm(struct device *dev)
{
        sentron_run_mode(dev, RUN_MODE_LPM);
        m_power_status = LPM_SUSPEND;

        return 0;
}

static int ssd6600_set_nm(struct device *dev)
{
        int err=0;

        if( m_power_status == LPM_SUSPEND ) {
                if( lpm_end2(dev) >= 0 )
                        m_power_status = LPM_SUSPEND_DELAY;
        }

        err = lpm_end_clear(dev);

        if ( err < 0 ) {
                err = -EAGAIN;
                TOUCH_E("fail to write reset command");
        }

        m_power_status = LPM_RESUME;

        return 0;
}

static int ssd6600_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->lpwg.mode == LPWG_NONE) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep\n",
				__func__, __LINE__);
			ssd6600_lpwg_control(dev, LPWG_NONE);
		} else if (ts->lpwg.screen) {
			TOUCH_I("%s(%d) - FB_SUSPEND & screen on -> skip\n",
				__func__, __LINE__);
			return 0;
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep by prox\n",
				__func__, __LINE__);
			ssd6600_lpwg_control(dev, LPWG_NONE);
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			/* knock on */
			TOUCH_I("%s(%d) - knock on by hole\n",
				__func__, __LINE__);
			ssd6600_lpwg_control(dev, LPWG_DOUBLE_TAP);
		} else {
			/* knock on/code */
			TOUCH_I("%s(%d) - knock %d\n",
				__func__, __LINE__, ts->lpwg.mode);
			ssd6600_lpwg_control(dev, ts->lpwg.mode);
		}
		return 0;
	}

	/* resume */
	if (ts->lpwg.screen) {
        ssd6600_set_nm(dev);
		/* normal */
		TOUCH_I("%s(%d) - normal\n",
				__func__, __LINE__);
		ssd6600_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* normal */
		TOUCH_I("%s(%d) - normal on screen off\n",
				__func__, __LINE__);
		ssd6600_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		/* wake up */
		TOUCH_I("%s(%d) - wake up on screen off and prox\n",
				__func__, __LINE__);
		TOUCH_I("%s - wake up is not ready\n", __func__);
		ssd6600_lpwg_control(dev, LPWG_NONE);
	} else {
		/* partial */
		touch_report_all_event(ts);
		TOUCH_I("%s(%d) - parial mode\n",
				__func__, __LINE__);
		TOUCH_I("%s - partial is not ready\n", __func__);
		ssd6600_lpwg_control(dev, LPWG_NONE);
	}

	return 0;
}

static void synaptic_init_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[0].tap_count = 2;
	ts->tci.info[0].min_intertap = 6;
	ts->tci.info[0].max_intertap = 70;
	ts->tci.info[0].touch_slop = 100;
	ts->tci.info[0].tap_distance = 10;
	ts->tci.info[0].intr_delay = 0;

	ts->tci.info[1].tap_count = 2;
	ts->tci.info[1].min_intertap = 6;
	ts->tci.info[1].max_intertap = 70;
	ts->tci.info[1].touch_slop = 100;
	ts->tci.info[1].tap_distance = 255;
	ts->tci.info[1].intr_delay = 68;
}

int ds_read_boot_st(struct device *dev, u16 *value)
{
        int ret = 0;
        u8 wd[10];
        u8 rd[10];

        wd[0] = 0x00;
        wd[1] = 0x00;
        wd[2] = 0x00;
        wd[3] = 0x00;

        if( (ret = ssd6600_reg_read_ex(dev, wd, 4, rd, 2)) < 0 ) {
                TOUCH_E("read boot st i2c read fail(1)!!" );
                return ret;
        }

        *value = rd[0] | (rd[1] << 8);

        return ret;
}

static int ds_init_code(struct device *dev)
{
        int ret = 0;
        u16 rd = 0;

        // I2C address change   
        if( ds_clear_int(dev) < 0 ) return -1;
        if( ds_read_boot_st(dev, (u16 *)&rd) < 0 ) return -1;
        TOUCH_I(">>>>> read boot st read(2) : 0x%04x", rd );
        if( (ret = ds_eflash_write(dev, 0xE000, 0x0003)) < 0 ) return ret;
        if( (ret = ds_eflash_read(dev, 0xE000, (u8 *)&rd, 2)) < 0 ) return ret;
        TOUCH_I(">>>>> 0xE000 read : 0x%04x", rd );
        if( (ret = ds_eflash_write(dev, 0xE009, 0x0000)) < 0 ) return ret;
        if( (ret = ds_eflash_read(dev, 0xE009, (u8 *)&rd, 2)) < 0 ) return ret;
        TOUCH_I(">>>>> 0xE009 read : 0x%04x", rd );
        if( (ret = ds_eflash_write(dev, 0xE00A, 0x0001)) < 0 ) return ret;
        if( (ret = ds_eflash_read(dev, 0xE00A, (u8 *)&rd, 2)) < 0 ) return ret;
        TOUCH_I(">>>>> 0xE00A read : 0x%04x", rd );

        if( (ret = ds_eflash_write(dev, 0xF000, 0x0003)) < 0 ) return ret;
        if( (ret = ds_eflash_read(dev, 0xF000, (u8 *)&rd, 2)) < 0 ) return ret;
        TOUCH_I(">>>>> 0xF000 read : 0x%04x", rd );

        if( (ret = ds_eflash_write(dev, 0x85FF, 0x0200)) < 0 ) return ret;
        if( (ret = ds_eflash_read(dev, 0x85FF, (u8 *)&rd, 2)) < 0 ) return ret;
        TOUCH_I(">>>>> 0x85FF read : 0x%04x", rd );

        if( (ret = ds_eflash_write(dev, 0xA003, 0x0200)) < 0 ) return ret;
        if( (ret = ds_eflash_read(dev, 0xA003, (u8 *)&rd, 2)) < 0 ) return ret;
        TOUCH_I(">>>>> 0xA003 read : 0x%04x", rd );

        if( (ret = ds_eflash_write(dev, 0xA004, 0x0070)) < 0 ) return ret;
        if( (ret = ds_eflash_read(dev, 0xA004, (u8 *)&rd, 2)) < 0 ) return ret;
        TOUCH_I(">>>>> 0xA004 read : 0x%04x", rd );

        return ret;
}

static int sint_int_mask(struct device *dev)
{
        s32 ret = 0;
        u32 temp_flag = 0x00;
        u8 pkt[10] = {0,};

        pkt[0] = DS_EFLASH_READ & 0xff;
        pkt[1] = (DS_EFLASH_READ >> 8 ) & 0xff;
        pkt[2] = DS_COMMAND_01 & 0xff;
        pkt[3] = (DS_COMMAND_01 >> 8 ) & 0xff;

        if( (ret = ssd6600_reg_read_ex(dev, pkt, 4, (u8 *)&temp_flag, 2)) < 0 ) {
                TOUCH_E("0x%04X i2c read fail(1)!!", DS_COMMAND_01);
                return ret;
        }
        TOUCH_I("temp_flag = 0x%04X", temp_flag);
        temp_flag = 0x00E5;
        temp_flag = (temp_flag << 16)|DS_COMMAND_01;
        if( (ret = ssd6600_reg_write(dev, DS_EFLASH_WRITE, (u8 *)&temp_flag, 4)) < 0 ) {
                TOUCH_E("0x%04X i2c read fail(2)!!", DS_COMMAND_01);
                return ret;
        }
        temp_flag = 0;
        if( (ret = ds_eflash_read(dev, DS_COMMAND_01, (u8 *)&temp_flag, 2)) < 0 ) return ret;
        TOUCH_I(">>>>> 0x%04x read : 0x%04x", DS_COMMAND_01, temp_flag );

        temp_flag = (0x20<<16)|DS_COMMAND_02;
        if( (ret = ssd6600_reg_write(dev, DS_EFLASH_WRITE, (u8 *)&temp_flag, 4)) < 0 ) {
                TOUCH_E("0x%04X i2c write fail(2)!!", DS_COMMAND_02);
                return ret;
        }
        temp_flag = 0;
        if( (ret = ds_eflash_read(dev, DS_COMMAND_02, (u8 *)&temp_flag, 2)) < 0 ) return ret;
        TOUCH_I(">>>>> 0x%04x read : 0x%04x", DS_COMMAND_02, temp_flag );

        return ret;
}

static int sint_unstall(struct device *dev)
{
        s32 ret = 0;
        u32 temp_flag = 0x00;

        temp_flag = (0x0000<<16)|DS_COMMAND_031;
        if( (ret = ssd6600_reg_write(dev, DS_EFLASH_WRITE, (u8 *)&temp_flag, 4)) < 0 ) {
                TOUCH_E("0x%04X i2c write fail(2)!!", DS_COMMAND_031);
                return ret;
        }

        temp_flag = (0x0002<<16)|DS_COMMAND_03;
        if( (ret = ssd6600_reg_write(dev, DS_EFLASH_WRITE, (u8 *)&temp_flag, 4)) < 0 ) {
                TOUCH_E("0x%04X i2c write fail(2)!!", DS_COMMAND_03);
                return ret;
        }

        if( (ret = ds_eflash_read(dev, DS_COMMAND_03, (u8 *)&temp_flag, 2)) < 0 ) return ret;
        TOUCH_I(">>>>> 0x%04x read : 0x%04x", DS_COMMAND_03, temp_flag );

        temp_flag = 0x0000;
        if( (ret = ssd6600_reg_write(dev, DS_CUP_CONTROL, (u8 *)&temp_flag, 2)) < 0 ) {
                TOUCH_E("0x%04X i2c write fail(2)!!", DS_CUP_CONTROL);
                return ret;
        }

        return ret;
}

int sentron_reset(struct device *dev)
{
        struct touch_core_data *ts = to_touch_core(dev);

        int retry = 100;

        touch_gpio_direction_output(ts->reset_pin, 0);
        udelay(300);
        touch_gpio_direction_output(ts->reset_pin, 1);

        int_pin_check(dev, retry*2);
              
        return 0;
}

int ds_eflash_write(struct device *dev, int addr, u16 data)
{
        int ret=0;
        u8 wd[10];
        wd[0] = addr & 0xFF;
        wd[1] = (addr >> 8) & 0xFF;
        wd[2] = data & 0xFF;
        wd[3] = (data >> 8) & 0xFF;

        if( (ret = ssd6600_reg_write(dev, DS_EFLASH_WRITE, wd, 4)) < 0 ) {
                TOUCH_E("0x%04X i2c read fail(2)!!", DS_COMMAND_01);
        return ret;
        }

        return ret;
}

int ds_eflash_read(struct device *dev, int addr, u8 *rd, int rLen)
{
        int ret=0;
        u8 wd[10];
        wd[0] = DS_EFLASH_READ & 0xff;
        wd[1] = (DS_EFLASH_READ >> 8 ) & 0xff;
        wd[2] = addr & 0xff;
        wd[3] = (addr >> 8 ) & 0xff;

        if( (ret = ssd6600_reg_read_ex(dev, wd, 4, rd, rLen)) < 0 ) {
                TOUCH_E("0x%04X i2c read fail(1)!!", addr);
                return ret;
        }

        return ret;
}

int ds_clear_int(struct device *dev)
{
        int ret = 0;
        u8 wd[10];
        wd[0] = 0x00;
        wd[1] = 0x00;

        if( (ret = ssd6600_reg_write(dev, DS_CLEAR_INT, wd, 2)) < 0 ) {
                TOUCH_E("0x%04X i2c write fail(2)!!", DS_CLEAR_INT);
                return ret;
        }

        return ret;
}

int fw_ds_view_fw_info(struct device *dev)
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct sentron_device *ftdev = (struct sentron_device *)touch_get_device(ts);
                TOUCH_I("==========================================\n");
                if( ftdev->fw_info->haveflag ) {
                        TOUCH_I("F/W Version : %d.%d\n", ftdev->fw_info->version1[1], ftdev->fw_info->version1[0] );
                        TOUCH_I("F/W Product : %s%s\n", ftdev->fw_info->prdID1, ftdev->fw_info->prdID2 );
                        TOUCH_I("F/W Row(node_Y) : %d, Col(node_x) : %d\n", ftdev->ftconfig->x_node, ftdev->ftconfig->y_node);
                        TOUCH_I("IC Name : %s%s\n", ftdev->fw_info->ICName1, ftdev->fw_info->ICName2);
                        TOUCH_I("MAX_X[%d] MAX_Y[%d]\n", ftdev->ftconfig->max_x, ftdev->ftconfig->max_y);
                } else {
                        TOUCH_I("Fail to read the F/W Info!!!!");
                }
                TOUCH_I("==========================================\n");
    
        return 0;
}

static int ssd6600_get_cmd_version(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sentron_device *ftdev = (struct sentron_device *)touch_get_device(ts);
	int offset = 0;

	if (ftdev == NULL) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset += sprintf(buf + offset, "======== IC Firmware Info ========\n");
	if( ftdev->fw_info->haveflag ) {
		offset += sprintf(buf + offset, "F/W Version : %d.%d \n", ftdev->fw_info->version1[1], ftdev->fw_info->version1[0]);
		offset += sprintf(buf + offset, "F/W Product : %s%s \n", ftdev->fw_info->prdID1, ftdev->fw_info->prdID2 );
		offset += sprintf(buf + offset, "F/W Row : %d, Col : %d\n", ftdev->ftconfig->x_node, ftdev->ftconfig->y_node);
		offset += sprintf(buf + offset, "IC Name : [%s%s] \n", ftdev->fw_info->ICName1, ftdev->fw_info->ICName2);
	} else {
		TOUCH_I("Fail to read the F/W Info!!!!");
	}
	
	offset += sprintf(buf + offset, "\n======== BIN Firmware Info ========\n");
	if( ftdev->bin_info->haveflag ) {
		offset += sprintf(buf + offset, "FW Version: %d.%d\n", ftdev->bin_info->version1[1], ftdev->bin_info->version1[0]);
		offset += sprintf(buf + offset, "IC Name : %s%s \n", ftdev->bin_info->ICName1, ftdev->bin_info->ICName2);
	} else {
		TOUCH_I("Fail to read the BIN file Info!!!!");
	}

	return offset;
}

int sentron_read_hw_info(struct device *dev, u16 *info)
{
	int err = 0;

	err = ssd6600_reg_read(dev, SENTRON_HW_CAL_INFO, (u8 *)(info), 2);
	if ( err < 0) {
		TOUCH_E("error read HW cal info using i2c.-");
	}

	return err;
}

int sentron_write_hw_info(struct device *dev)
{
	int err = 0;
	u16 info = 0x01;

	err = ssd6600_reg_write(dev, SENTRON_HW_CALIBRATION, (u8 *)&(info), 2 );

	if( err < 0 ) {
		TOUCH_E("error HW cal info write (0x%04x)!!", info);
	}

	return err;
}

int sentron_HW_check(struct device *dev)
{
	int err = 0;
	int retry1=3;
	int limit=0;
	u16 info = 0;

	err = sentron_read_hw_info(dev, &info);

	if( err >= 0 ) {
		TOUCH_I("HW auto tune info = %d", info);

		if( info == 0 ) {
			do {
				limit = 5;
				err = sentron_write_hw_info(dev);
				if( err >= 0 ) {
					do {
						sentron_read_hw_info(dev, &info);
						if( info != 0 ) break;
						mdelay(10);	// 100 -> 10
					} while( limit-- > 0 );
				}
			} while( info == 0 && retry1-- > 0 );

			if( info == 1 ) TOUCH_I("HW auto tune Success!!");
			else TOUCH_I("HW auto tune Fail!!");
		}
	}

	return 0;
}

static int sentron_init_config(struct device *dev)
{
        struct touch_core_data *ts = to_touch_core(dev);
        int i, err = 0;
        u16 val = 0x0000;

        struct sentron_device *ftdev = (struct sentron_device *)touch_get_device(ts);
        struct sentron_config *ftconfig = ftdev->ftconfig;

        val = TOUCH_POINT_MODE;
        err = ssd6600_reg_write(dev, SENTRON_TOUCH_MODE, (u8 *)&(val), 2);

        if (err < 0)
                TOUCH_E("Fail to set TOUCH_MODE %d.", val);

        ftconfig->x_node = SENTRON_MAX_X_NODE;
        ftconfig->y_node = SENTRON_MAX_Y_NODE;
        ftconfig->max_x = SENTRON_X_MAX;
        ftconfig->max_y = SENTRON_Y_MAX;
        ftconfig->using_point = SENTRON_MAX_POINT;

        i=10;

        while( (i--)>0 ) {
                int_clear_cmd(dev);
                udelay(10);
                if (gpio_get_value(ts->int_pin)) break;
        }

        return err;
}

int sentron_pre_init(struct device *dev)
{
        int ret = 0;

        if( ds_init_code(dev) < 0 ) return -1;
        if( sint_int_mask(dev) < 0 ) return -1;
        if( sint_unstall(dev) < 0 ) return -1;

        ret = int_pin_check(dev, 100);

        return ret;
}

int sentron_init(struct device *dev)
{
        int ret = 0;
        int retry = 3;
        u16 data = 0;

        do {
                sentron_reset(dev);
                ret = sentron_pre_init(dev);

                if ( ret < 0) {
                        TOUCH_E("Pre init failed\n");
                        break;
                }

                ret = ssd6600_reg_read(dev, SENTRON_STATUS_LENGTH, (u8 *)(&data), 2);
                if ( ret < 0) {
                        TOUCH_E("error read point info using i2c.\n");
                        break;
                }
                TOUCH_I("TEST : status & length = 0x%04x", data);
        
                ret = sentron_init_config(dev);
                if ( ret < 0) {
                        TOUCH_E("Read config failed\n");
                        break;
                }

                fw_ds_view_fw_info(dev);
                if( (data&0xFF00) == 0 ) break;
        } while( (retry--) > 0 );

        return ret;
}

static int sentron_probe_init(struct device *dev)
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct sentron_device *ftdev = NULL;
        struct sentron_data *ftdata = NULL;
        struct sentron_config *ftconfig = NULL;
        struct sentron_info *fw_info = NULL;
        struct sentron_info *bin_info = NULL;
        int err = 0;

        ftdev = (struct sentron_device *)touch_get_device(ts);
        ftdata = (struct sentron_data *)kzalloc( \
                sizeof(struct sentron_data), GFP_KERNEL);
        if ( !ftdata ) {
                TOUCH_E("Create sentron data failed\n");
                err = -ENOMEM;
                goto create_data_failed;
        }
        ftdata->queue_front = 0;
        ftdata->queue_rear = 0;
#ifdef SUPPORT_KEY_BUTTON
        ftdata->keydata = 0x00;
#endif
        ftconfig = (struct sentron_config *)kzalloc( \
                        sizeof(struct sentron_config), GFP_KERNEL);
        if ( !ftconfig ) {
                TOUCH_E("Create sentron config failed\n");
                err = -ENOMEM;
                goto create_config_failed;
        }

        fw_info = (struct sentron_info *)kzalloc( \
                        sizeof(struct sentron_info), GFP_KERNEL);
        if ( !fw_info ) {
                TOUCH_E("Create sentron fw_info failed");
                err = -ENOMEM;
                goto create_fw_info_failed;
        }
        bin_info = (struct sentron_info *)kzalloc( \
                        sizeof(struct sentron_info), GFP_KERNEL);
        if ( !bin_info ) {
                TOUCH_E("Create sentron bin_info failed\n");
                err = -ENOMEM;
                goto create_bin_info_failed;
        }

        ftdev->touch_mode = TOUCH_POINT_MODE;
        ftdev->ftconfig = ftconfig;
        ftdev->ftdata = ftdata;
        ftdev->work_procedure = TS_NO_WORK;
        ftdev->fw_info = fw_info;
        ftdev->bin_info = bin_info;

        sema_init(&ftdev->work_procedure_lock, 1); 
        SSD6600_get_boot_fw_info(dev);

        return 0;

create_bin_info_failed:
        kfree(fw_info);
create_fw_info_failed:
        kfree(ftconfig);
create_config_failed:
        kfree(ftdata);
create_data_failed:
        kfree(ftdev);
    
        return err;
}

static int ssd6600_probe(struct device *dev)
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct sentron_device *ftdev = NULL;

	TOUCH_TRACE();

        ftdev = (struct sentron_device *)kzalloc( sizeof(struct sentron_device), GFP_KERNEL);
        if( !ftdev ) {
                TOUCH_E("Create sentron device failed");
                return -ENOMEM;
        }
        touch_set_device(ts, ftdev);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 1);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, 4096);

        synaptic_init_tci_info(dev);
        sentron_probe_init(dev);
	return 0;
}

static int ssd6600_remove(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int ssd6600_suspend(struct device *dev)
{
	TOUCH_TRACE();

        ssd6600_set_lpm(dev);
        ssd6600_lpwg_mode(dev);

	return 0;
}

static int ssd6600_resume(struct device *dev)
{
	TOUCH_TRACE();

        ssd6600_lpwg_mode(dev);
	return 0;
}

static int sentron_report(struct device *dev)
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct sentron_device *ftdev = (struct sentron_device *)touch_get_device(ts);
        struct sentron_data *data = ftdev->ftdata;
        struct touch_data *tdata = NULL;

        int i = 0;
        u8 status, length, id, w;
        u16 x, y;
        u8 finger_index=0;

        ts->new_mask = 0;

        if( !ftdev || !data )
        return -EFAULT;

        status = (data->point_info & 0xff00) >> 8;
        length = (data->point_info & 0x00ff);

        if( length == 0 ) {
                // TODO : need release function
        } else {
                data->lastValidCnt = 0;
                finger_index = 0;

                for (i = 0; i < length/2; i++) {
                        id = (data->points[i].id_x  & 0xf000) >> 12;
                        x = (data->points[i].id_x  & 0x0fff);
                        w = (data->points[i].w_y  & 0xf000) >> 12;
                        y = (data->points[i].w_y  & 0x0fff);

			if(w) { 
				data->lastValidCnt++;
				finger_index++;

				ts->new_mask |= (1 << id);
				tdata = ts->tdata + id;

				tdata->id = id;
				tdata->type = 1;
				tdata->x = x;
				tdata->y = y;
				tdata->pressure = w * 10;
				tdata->width_major = w;
				tdata->width_minor = w;
				tdata->orientation = 1;

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

        ts->tcount = finger_index;
        }

        ts->intr_status |= TOUCH_IRQ_FINGER;

        return 0;
}

static int sentron_read_lpwg( struct device *dev)
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct sentron_device *ftdev = (struct sentron_device *)touch_get_device(ts);
        struct sentron_data *data = ftdev->ftdata;
        int err = 0;
        u8 status=0x00, length=0x00;

        TOUCH_TRACE();

        if ( !dev )
                return -EFAULT;

        TOUCH_I("sentron_read_lpwg\n");

        if (gpio_get_value(ts->int_pin)) {
                /*interrupt pin is high, not valid data.*/
                TOUCH_E("read points... interrupt pin is high\n");
                return -EAGAIN;
        }

        err = ssd6600_reg_read(dev, SENTRON_STATUS_LENGTH, (u8 *)(&data->point_info), 2);
        if ( err < 0) {
                TOUCH_E("error read point info using i2c.-\n");
        goto out;
        }

        if( data->point_info == 0x0000 ) {
                goto out;
        }

        status = (data->point_info & 0xff00) >> 8;
        length = (data->point_info & 0x00ff) << 1;

        TOUCH_D(ABS, "status : 0x%02x, 0x%02x\n", status, length);

        if( (status&STATUS_CHECK_PALM_GESTURE) == STATUS_CHECK_PALM_GESTURE ) {
                if( sentron_read_gesture(dev) >= 0 ) {
                        TOUCH_I("knock on detect\n");
                }
        }
out:
        int_clear_cmd(dev);

        return err;
}

static int sentron_read_points( struct device *dev)
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct sentron_device *ftdev = (struct sentron_device *)touch_get_device(ts);
        struct sentron_data *data = ftdev->ftdata;
        int err = 0;
	u8 status=0x00, length=0x00;

	TOUCH_TRACE();

        if ( !dev )
                return -EFAULT;

        if (gpio_get_value(ts->int_pin)) {
                /*interrupt pin is high, not valid data.*/
                TOUCH_E("read points... interrupt pin is high\n");
                return -EAGAIN;
        }

        err = ssd6600_reg_read(dev, SENTRON_STATUS_LENGTH, (u8 *)(&data->point_info), 2);
        if ( err < 0) {
                TOUCH_E("error read point info using i2c.-\n");
                goto out;
        }

        if( data->point_info == 0x0000 ) {
                goto out;
        }

        status = (data->point_info & 0xff00) >> 8;
        length = (data->point_info & 0x00ff) << 1;

        TOUCH_D(ABS, "status : 0x%02x, 0x%02x\n", status, length);

        if( length > 0 && ((status&0x0F) > 0) ) {
                err = ssd6600_reg_read(dev,
                        SENTRON_POINT_DATA, (u8 *)(&data->points), length);
        if (err < 0) {
                TOUCH_E("error read point info using i2c.-\n");
                goto out;
        }
        TOUCH_D(ABS, "point : 0x%x, 0x%x\n", 
                data->points[0].id_x, data->points[0].w_y);
    }

out:
        int_clear_cmd(dev);

        if( err >= 0 && length > 0 && ((status&0x0F) > 0) ) {
                sentron_report(dev);
        }
        return err;
}

static int ssd6600_irq_handler(struct device *dev)
{
        int ret;
    
	TOUCH_TRACE();

        if( m_power_status == LPM_SUSPEND ) {
                ret = sentron_read_lpwg(dev);
        } else if( m_power_status == LPM_RESUME ) {
                ret = sentron_read_points(dev);
        }

	return ret;
}


static int ssd6600_init(struct device *dev)
{
	TOUCH_TRACE();
	
        sentron_init(dev);

	return 0;
}

static int ssd6600_power(struct device *dev, int ctrl)
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


static int ssd6600_lpwg(struct device *dev, u32 code, void *param)
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
                        ssd6600_tci_active_area(dev);
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
                TOUCH_I("LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
                        ts->lpwg.mode,
                        ts->lpwg.screen ? "ON" : "OFF",
                        ts->lpwg.sensor ? "FAR" : "NEAR",
                        ts->lpwg.qcover ? "CLOSE" : "OPEN");
                ssd6600_lpwg_mode(dev);
                break;
        }

        return 0;
}

static int ssd6600_notify(struct device *dev, ulong event, void *data)
{
	TOUCH_TRACE();

	return 0;
}

static int ssd6600_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int ssd6600_get(struct device *dev, u32 cmd, void *input, void *output)
{
        int ret = 0;
        TOUCH_TRACE();

        TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

        switch (cmd) {
                case CMD_VERSION:
                        ret = ssd6600_get_cmd_version(dev, (char *)output);
                        break;

                case CMD_ATCMD_VERSION:
                        //ret = ssd6600_get_cmd_atcmd_version(dev, (char *)output);
                        break;

                default:
                        break;
        }

        return ret;
}

static int sentron_get_rawdata(struct device *dev, int16_t *buf, u32 touchmode)
{
        struct touch_core_data *ts = to_touch_core(dev);
        struct sentron_device *ftdev = (struct sentron_device *)touch_get_device(ts);
        int err=0;
        int garbage=10;
        int sz=0;
        u16 point_info=0x00;
        int len = MAX_RAWDATA_BUFFER_SIZE;
        u16 ptr[SENTRON_MAX_NODE] = {0,};
        u8 *temp=NULL;
        int retry = 5;
        u16 val=0;
	
        do {
                err = ssd6600_reg_write(dev, SENTRON_TOUCH_MODE, (u8 *)&(touchmode), 2);
                if (err < 0) {
                        TOUCH_E("Fail to set TOUCH_MODE %d.", touchmode);
                        goto out;
                }

                if( ssd6600_reg_read(dev, SENTRON_TOUCH_MODE, (u8 *)&val, 2) < 0 ) goto out;

                if( val == touchmode ) break;
	
                mdelay(1);
        }while( (retry--) > 0 );

        if( retry <= 0 ) {
                TOUCH_E("Touch mode[%d] change fail!!\n", touchmode );
                goto out;
        }
		
        do {
                err = -1;
                retry = 20;
                do {
                        if( gpio_get_value(ts->int_pin) == 0 ) {
                                err = 0;
                                break;
                        }
                        mdelay(1);
                } while( (retry--) > 0 );

                if( err < 0 ) goto out_clear;
                if( (garbage--) <= 0 ) break;
						
                int_clear_cmd(dev);
        }while( 1 );

        err = ssd6600_reg_read(dev, SENTRON_STATUS_LENGTH, (u8 *)(&point_info), 2);
        if ( err < 0) {
                TOUCH_E("error read point info using i2c.-");
                goto out_clear;
        }

        sz = SENTRON_MAX_NODE * 2;
        temp = (u8 *)(ptr);		

        while( sz > 0 ) {
                if( sz <= MAX_RAWDATA_BUFFER_SIZE ) len = sz;

                sz -= MAX_RAWDATA_BUFFER_SIZE;
                err = ssd6600_reg_read(dev, SENTRON_RAW_DATA, temp, len);
                if ( err < 0) {
                        TOUCH_E("error : read raw data");
                        goto out_clear;
                }

                temp = temp+len;
        }
	
        for( retry=0; retry<SENTRON_MAX_NODE; retry++ ) {
                buf[retry] = ptr[SENTRON_MAX_NODE-1-retry];
        }
out_clear :
        int_clear_cmd(dev);
        touchmode = TOUCH_POINT_MODE;
        err = ssd6600_reg_write(dev, SENTRON_TOUCH_MODE, (u8 *)&(touchmode), 2);

        if (err < 0) {
                TOUCH_E("Fail to set TOUCH_MODE %d.", ftdev->touch_mode);
                goto out;
        }
	
out :
	
        return err;
}

static ssize_t get_data(struct device *dev, int16_t *buf, u32 wdata)
{
        TOUCH_I("======== get data(mode : %d) ========\n", wdata);

        if( sentron_get_rawdata(dev, buf, wdata) < 0 ) return 1;
        else return 0;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
        struct touch_core_data *ts = to_touch_core(dev);
        int ret = 0;
	int ret2 = 0;
	int16_t *rawdata = NULL;
	char * w_buf = NULL;
	int w_len = 0;
	int i = 0;
	int j = 0;

	mutex_lock(&ts->lock);
	touch_disable_irq(ts->irq);

	rawdata = kzalloc(sizeof(int16_t) * (SENTRON_MAX_X_NODE*SENTRON_MAX_Y_NODE), GFP_KERNEL);
	w_buf = kzalloc(BUF_SIZE * sizeof(char), GFP_KERNEL);

	if (rawdata == NULL || w_buf == NULL) {
		TOUCH_E("mem_error\n");
		return ret;
	}

	ret2 = get_data(dev, rawdata, 1);  /* 2 == deltadata */
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== rawdata ========\n");
	w_len += snprintf(w_buf + w_len, BUF_SIZE - w_len, "======== rawdata ========\n");

        for (i = 0 ; i < SENTRON_MAX_Y_NODE ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"[%2d]  ", i);
		w_len += snprintf(w_buf + w_len, BUF_SIZE - w_len,
			"[%2d] ", i);

        for (j = 0 ; j < SENTRON_MAX_X_NODE ; j++) {
            ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%5d ",
				rawdata[i * SENTRON_MAX_X_NODE + j]);
			log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"%5d ",
				rawdata[i * SENTRON_MAX_X_NODE + j]);
			w_len += snprintf(w_buf + w_len, BUF_SIZE - w_len,
				"%5d ", rawdata[i * SENTRON_MAX_X_NODE + j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
		w_len += snprintf(w_buf + w_len, BUF_SIZE - w_len, "\n");
	}

	write_file(dev, w_buf);
	touch_msleep(10);

error:
	if (rawdata != NULL)
		kfree(rawdata);

	if (w_buf != NULL)
		kfree(w_buf);

	sentron_init(dev);
	touch_enable_irq(ts->irq);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret2 = 0;
	int16_t *rawdata = NULL;
	int i = 0;
	int j = 0;

	mutex_lock(&ts->lock);
	touch_disable_irq(ts->irq);

	rawdata = kzalloc(sizeof(int16_t) * (SENTRON_MAX_X_NODE*SENTRON_MAX_Y_NODE), GFP_KERNEL);

	if (rawdata == NULL) {
		TOUCH_E("mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== deltadata ========\n");

	ret2 = get_data(dev, rawdata, 4);  /* 4 == deltadata */
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

        for (i = 0 ; i < SENTRON_MAX_Y_NODE ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"[%2d]  ", i);

        for (j = 0 ; j < SENTRON_MAX_X_NODE ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%5d ",
				rawdata[i * SENTRON_MAX_X_NODE + j]);
			log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"%5d ",
				rawdata[i * SENTRON_MAX_X_NODE + j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (rawdata != NULL)
		kfree(rawdata);

	sentron_init(dev);
	touch_enable_irq(ts->irq);
	mutex_unlock(&ts->lock);

	return ret;
}

static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(delta,   show_delta, NULL);

static struct attribute *ssd6600_attribute_list[] = {
	&touch_attr_rawdata.attr,
	&touch_attr_delta.attr,
	NULL,
};

static const struct attribute_group ssd6600_attribute_group = {
        .attrs = ssd6600_attribute_list,
};

static int ssd6600_register_sysfs(struct device *dev)
{
        struct touch_core_data *ts = to_touch_core(dev);
        int ret = 0;
        TOUCH_TRACE();

        ret = sysfs_create_group(&ts->kobj, &ssd6600_attribute_group);
        if (ret < 0)
                TOUCH_E("ssd6600 sysfs register failed\n");

        return 0;
}

static struct touch_driver touch_driver = {
	.probe = ssd6600_probe,
	.remove = ssd6600_remove,
	.suspend = ssd6600_suspend,
	.resume = ssd6600_resume,
	.init = ssd6600_init,
	.irq_handler = ssd6600_irq_handler,
	.power = ssd6600_power,
	.upgrade = ssd6600_upgrade,
	.lpwg = ssd6600_lpwg,
	.notify = ssd6600_notify,
	.register_sysfs = ssd6600_register_sysfs,
	.set = ssd6600_set,
	.get = ssd6600_get,
};


#define MATCH_NAME "lge,sentron_ssd6600"

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

MODULE_AUTHOR("hoyeon.jang@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
