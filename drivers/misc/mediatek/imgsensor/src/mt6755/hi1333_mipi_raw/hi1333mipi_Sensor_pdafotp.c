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
 *   hi1333mipi_Sensor_pdafotp.c
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/types.h>


#define PFX "HI1333_pdafotp"
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "hi1333_mipi_raw_Sensor.h"
#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#if 1
#define HI1333_DEBUG 0
#define HI1333_EEPROM_READ_ID  0xA1
#define HI1333_EEPROM_WRITE_ID   0xA0
#define HI1333_I2C_SPEED        100
#define HI1333_MAX_OFFSET		0xFFFF
#define HI1333_START_ADDR 0x0C50

#define HI1333_DATA_SIZE 2048//1404
BYTE hi1333_eeprom_data[HI1333_DATA_SIZE]= {0};

static kal_uint16 read_eeprom(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	if(iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, HI1333_EEPROM_WRITE_ID)<0)
		LOG_INF("read eeprom failed!!!\n");
	return get_byte;
}

/*
static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > HI1333_MAX_OFFSET)
        return false;
	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, HI1333_EEPROM_WRITE_ID)<0)
		return false;
    return true;
}
*/

static bool _read_HI1333_eeprom(kal_uint16 addr, kal_uint32 size ){
	int i = 0;
#ifdef HI1333_DEBUG
	int read_tmp = 0;
#endif
	for(i = 0; i < size; i++) {
		hi1333_eeprom_data[i] = read_eeprom(addr + i);
		//LOG_INF("read_eeprom 0x%0x %d\n",addr, hi1333_eeprom_data[i]);
	}
#ifdef HI1333_DEBUG
	read_tmp = read_eeprom(0xBE0);//VendorID
	LOG_INF("read_eeprom VendorID : 0x%x\n", read_tmp);
	read_tmp = read_eeprom(0xBE2);//EEPROM_MAP
	LOG_INF("read_eeprom EEPROM_MAP : 0x%x\n", read_tmp);
#endif
    return true;
}

bool read_HI1333_eeprom( kal_uint16 addr, BYTE *data, kal_uint32 size){
#ifdef HI1333_DEBUG
	int i = 0;
	MUINT32 idx = 0;
#endif
	addr = HI1333_START_ADDR;
	size = HI1333_DATA_SIZE;
	//BYTE header[9]= {0};
	//_read_3P3_eeprom(0x0000, header, 9);

	LOG_INF("Read HI1333 addr = 0x%x, size = 0x%d\n",addr, size);

	/* 
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_HI1333_eeprom(addr, hi1333_eeprom_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	*/
	if(!_read_HI1333_eeprom(addr, size))
		LOG_INF("Read HI1333 EEPROM Failed!!!!\n");
	LOG_INF("Read HI1333 EEPROM success!\n");
	
#ifdef HI1333_DEBUG
	for(i = 0; i < HI1333_DATA_SIZE; i += 4)
	{
		LOG_INF(" %d / 0x%x    0x%x    0x%x    0x%x\n",
		i, hi1333_eeprom_data[i], hi1333_eeprom_data[i+1], hi1333_eeprom_data[i+2], hi1333_eeprom_data[i+3]);
	}

	for (idx = 0; idx < HI1333_DATA_SIZE; idx += 16)
	{
		LOG_INF(" %d / 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", idx,
		*(hi1333_eeprom_data+idx), *(hi1333_eeprom_data+idx+1), *(hi1333_eeprom_data+idx+2), *(hi1333_eeprom_data+idx+3),
		*(hi1333_eeprom_data+idx+4), *(hi1333_eeprom_data+idx+5), *(hi1333_eeprom_data+idx+6), *(hi1333_eeprom_data+idx+7),
		*(hi1333_eeprom_data+idx+8), *(hi1333_eeprom_data+idx+9), *(hi1333_eeprom_data+idx+10), *(hi1333_eeprom_data+idx+11),
		*(hi1333_eeprom_data+idx+12), *(hi1333_eeprom_data+idx+13), *(hi1333_eeprom_data+idx+14), *(hi1333_eeprom_data+idx+15));
	}

#endif
	memcpy(data, hi1333_eeprom_data, size);
	LOG_INF("Copy HI1333 EEPROM success!\n");
    return true;
}

#if 0
bool getPDAFCalDataFromFile(void)
{
	bool Flag = false;
	int fd;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	fd = sys_open("/data/pdaf.txt", O_RDONLY, 777);

	if (fd < 0) {
		LOG_INF("KYM PDAF FILE READ FAIL\n");
		goto RESULT;
	} else {
/* if( sys_read(fd, (char *)&hi1333_eeprom_data[0], 1372) ) */
		if (sys_read(fd, (char *)&hi1333_eeprom_data[0], 1404)) {
			LOG_INF("KYM PDAF FILE READ PASS\n");
			Flag = true;
		}
	}

RESULT:
	sys_close(fd);
	set_fs(old_fs);
	return Flag;
}

bool read_hi1333_eeprom_fromfile(kal_uint16 addr, BYTE *data, kal_uint32 size)
{
	bool Flag;

	addr = 0xC50;		/* 0x0800; */
	size = 1404;


	Flag = getPDAFCalDataFromFile();
	if (Flag)
		memcpy(data, hi1333_eeprom_data, size);

	return Flag;
}
#endif

#else
#define HI1333_EEPROM_READ_ID  0xA1	/* 0xA2 */
#define HI1333_EEPROM_WRITE_ID   0xA0	/* 0xA3 */
#define HI1333_I2C_SPEED        100
#define HI1333_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048

#define READ_EEPROM_ID	0xB0
#define	I2C_SPEED	100

BYTE hi1333_eeprom_data[DATA_SIZE] = { 0 };

static bool get_done;
static int last_size;
static int last_offset;


BYTE eeprom_data[DATA_SIZE] = { 0 };

bool getPDAFCalDataFromFile(void)
{
	bool Flag = false;
	int fd;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	fd = sys_open("/data/pdaf.txt", O_RDONLY, 777);

	if (fd < 0) {
		LOG_INF("KYM PDAF FILE READ FAIL\n");
		goto RESULT;
	} else {
/* if( sys_read(fd, (char *)&hi1333_eeprom_data[0], 1372) ) */
		if (sys_read(fd, (char *)&hi1333_eeprom_data[0], 1404)) {
			LOG_INF("KYM PDAF FILE READ PASS\n");
			Flag = true;
		}
	}

RESULT:
	sys_close(fd);
	set_fs(old_fs);
	return Flag;
}

bool read_hi1333_eeprom(kal_uint16 addr, BYTE *data, kal_uint32 size)
{
	bool Flag;

	addr = 0xC50;		/* 0x0800; */
	size = 1404;


	Flag = getPDAFCalDataFromFile();
	if (Flag)
		memcpy(data, hi1333_eeprom_data, size);

	return Flag;
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////// */
/* read EEPROM In VCM */
/* //////////////////////////////////////////////////////////////////////////////////////////////////////////// */
#if 1
static bool selective_read_eeprom(kal_uint16 addr, BYTE *data)
{

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	if (addr > HI1333_MAX_OFFSET)
		return false;

	/* kdSetI2CSpeed(I2C_SPEED); */

	if (iReadRegI2C(pu_send_cmd, 2, (u8 *) data, 1, READ_EEPROM_ID) < 0) {
		LOG_INF("VCM E2PROM READ fail\n");
		return false;
	}
	/* LOG_INF( "read eeprom : 0x%d\n", (u8*)data ); */

	return true;
}

bool _read_eeprom(kal_uint16 addr, BYTE *data, kal_uint32 size)
{
	int i = 0;
	int index = 0;
	int offset = addr;

	for (i = 0; i < 496; i++) {
		if (!selective_read_eeprom(offset, &data[i])) {
			return false;
		}
		/* LOG_INF("VCM read_eeprom 0x%0x %d\n",offset, data[i]); */
		offset++;
		index++;
	}

	offset = addr + 1024;

	for (i = 0; i < 908 - 12; i++) {
		/* for(i = 0; i < 908; i++) { //ok */
		if (!selective_read_eeprom(offset, &data[index])) {
			return false;
		}
		/* LOG_INF("VCM read_eeprom 0x%0x %d\n",offset, data[i]); */
		offset++;
		index++;
	}

/* position Code */
	data[index++] = 0x63;
	data[index++] = 0x40;
	data[index++] = 0x6F;
	data[index++] = 0x40;
	data[index++] = 0x3F;
	data[index++] = 0x54;
	data[index++] = 0x53;
	data[index++] = 0x54;
	data[index++] = 0x5F;
	data[index++] = 0x54;
	data[index++] = 0x73;
	data[index++] = 0x54;
/*  */


	get_done = true;
	last_size = size;
	last_offset = addr;
	return true;
}

bool read_eeprom(kal_uint16 addr, BYTE *data, kal_uint32 size)
{
	size = 0x057c;
	addr = 0x0900;
	if (!get_done || last_size != size || last_offset != addr) {
		if (!_read_eeprom(addr, eeprom_data, size)) {
			get_done = 0;
			last_size = 0;
			last_offset = 0;
			return false;
		}
	}
	memcpy(data, eeprom_data, size);
	return true;
}
#endif
/* //////////////////////////////////////////////////////////////////////////////////////////////////////////// */
#endif
