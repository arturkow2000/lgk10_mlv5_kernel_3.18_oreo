/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   s5k3p8sx_pdafotp.c
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   CMOS sensor source file
 *
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/slab.h>


#define PFX "S5K3P8_pdafotp"
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)
#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k3p8sxmipiraw_Sensor.h"

#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define S5K3P8_DEBUG 0
#define S5K3P8_EEPROM_READ_ID  0xB1
#define S5K3P8_EEPROM_WRITE_ID   0xB0
#define S5K3P8_I2C_SPEED        100
#define S5K3P8_MAX_OFFSET		0xFFFF
#define S5K3P8_START_ADDR 0x0C50

#define S5K3P8_DATA_SIZE 2048//1404
BYTE s5k3P8_eeprom_data[S5K3P8_DATA_SIZE] = { 0 };

static kal_uint16 read_eeprom(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	if (iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, S5K3P8_EEPROM_WRITE_ID) < 0)
		LOG_INF("read eeprom failed!!!\n");
	return get_byte;
}

/*
static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > S5K3P8_MAX_OFFSET)
	return false;
	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, S5K3P8_EEPROM_WRITE_ID)<0)
		return false;
    return true;
}
*/

static bool _read_3P8_eeprom(kal_uint16 addr, kal_uint32 size)
{
	int i = 0;
#ifdef S5K3P8_DEBUG
	int read_tmp = 0;
#endif
	for (i = 0; i < size; i++) {
		s5k3P8_eeprom_data[i] = read_eeprom(addr + i);
		//LOG_INF("read_eeprom 0x%0x %d\n", addr, s5k3P8_eeprom_data[i]);
	}
#ifdef S5K3P8_DEBUG
	read_tmp = read_eeprom(0xBE0);//VendorID
	LOG_INF("read_eeprom VendorID : 0x%x\n", read_tmp);
	read_tmp = read_eeprom(0xBE2);//EEPROM_MAP
	LOG_INF("read_eeprom EEPROM_MAP : 0x%x\n", read_tmp);
#endif
	return true;
}

bool read_3P8_eeprom(kal_uint16 addr, BYTE *data, kal_uint32 size)
{
#ifdef S5K3P8_DEBUG
	int i = 0;
	MUINT32 idx = 0;
#endif
	addr = S5K3P8_START_ADDR;
	size = S5K3P8_DATA_SIZE;
	/* BYTE header[9]= {0}; */
	/* _read_3P3_eeprom(0x0000, header, 9); */

	LOG_INF("Read 3P8 EEPROM, addr = 0x%x, size = 0x%d\n", addr, size);

	/*
	   if(!get_done || last_size != size || last_offset != addr) {
	   if(!_read_3P8_eeprom(addr, s5k3P8_eeprom_data, size)){
	   get_done = 0;
	   last_size = 0;
	   last_offset = 0;
	   return false;
	   }
	   }
	 */
	if (!_read_3P8_eeprom(addr, size))
		LOG_INF("Read 3P8 EEPROM Failed!!!!\n");
	LOG_INF("Read 3P8 EEPROM success!\n");
#ifdef S5K3P8_DEBUG
	for(i = 0; i < S5K3P8_DATA_SIZE; i += 4)
	{
		LOG_INF(" %d / 0x%x    0x%x    0x%x    0x%x\n",
		i, s5k3P8_eeprom_data[i], s5k3P8_eeprom_data[i+1], s5k3P8_eeprom_data[i+2], s5k3P8_eeprom_data[i+3]);
	}

	for (idx = 0; idx < S5K3P8_DATA_SIZE; idx += 16)
	{
		LOG_INF(" %d / 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", idx,
		*(s5k3P8_eeprom_data+idx), *(s5k3P8_eeprom_data+idx+1), *(s5k3P8_eeprom_data+idx+2), *(s5k3P8_eeprom_data+idx+3),
		*(s5k3P8_eeprom_data+idx+4), *(s5k3P8_eeprom_data+idx+5), *(s5k3P8_eeprom_data+idx+6), *(s5k3P8_eeprom_data+idx+7),
		*(s5k3P8_eeprom_data+idx+8), *(s5k3P8_eeprom_data+idx+9), *(s5k3P8_eeprom_data+idx+10), *(s5k3P8_eeprom_data+idx+11),
		*(s5k3P8_eeprom_data+idx+12), *(s5k3P8_eeprom_data+idx+13), *(s5k3P8_eeprom_data+idx+14), *(s5k3P8_eeprom_data+idx+15));
	}

#endif	
	memcpy(data, s5k3P8_eeprom_data, size);
	LOG_INF("Copy 3P8 EEPROM success!\n");
	return true;
}
