/*!file sx9310.c
 *brief  SX9310 Driver
 *
 * Driver for the SX9310
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>

#include <linux/device.h>
#include <linux/interrupt.h>

#include <mach/irqs.h>
#include <linux/kthread.h>
#include <linux/async.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

/* #include <linux/rtpm_prio.h> */
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_HQ_HARDWARE_INFO	//WSP add for hardwareinfo
#include <linux/hardware_info.h>
#endif
#define DEBUG
#define DRIVER_NAME "SX9310_2"

#define MAX_WRITE_ARRAY_SIZE 32
static int sar_debug_en = 1;

#include "sx9310.h"		/* main struct, interrupt,init,pointers */

#define IDLE 0
#define ACTIVE 1
#define THRESHOLD 0x5
#define TIME 30
#define SENSOR_CAL_FILENAME "/persist-lg/sensor/data.txt"

char temp_str_2[20];
unsigned int channel2_thre=0;
unsigned int channel3_thre=0;
unsigned int temp_thre2=0;
static u8 thre_flag2 = 0;
static int sar_sensor_enable=0;
static u8 sensor_flag = 0;
static unsigned int channel_thre=0;
static unsigned char threshold=0;
static u8 old_status=0x50;
static u8 sar_flag=0;
static int thermal_value2=0;
static ktime_t ktime2;
static ktime_t ktime_cal2;
/*! struct sx9310
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */

static struct _buttonInfo psmtcButtons[] = {
	{
	 .keycode = KEY_SAR_B_0,
	 .mask = SX9310_TCHCMPSTAT_TCHSTAT0_FLAG,
	 },
};

/* Define Registers that need to be initialized to values different than
 * default
 */
static struct smtc_reg_data sx9310_i2c_reg_setup[] = {
	{
	 .reg = SX9310_IRQ_ENABLE_REG,
	 .val = 0x00,  //disable all interrupt
	 },
	{
	 .reg = SX9310_IRQFUNC_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_CPS_CTRL1_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_CPS_CTRL2_REG,
	 .val = 0x04,
	 },
	{
	 .reg = SX9310_CPS_CTRL3_REG,
	 .val = 0x0A,
	 },
	{
	 .reg = SX9310_CPS_CTRL4_REG,
	 .val = 0x0D,
	 },
	{
	 .reg = SX9310_CPS_CTRL5_REG,
	 .val = 0xC1,
	 },
	{
	 .reg = SX9310_CPS_CTRL6_REG,
	 .val = 0x10,
	 },
	{
	 .reg = SX9310_CPS_CTRL7_REG,
	 .val = 0x4C,
	 },
	{
	 .reg = SX9310_CPS_CTRL8_REG,  //cs0 TH
	 .val = 0x78,
	 },
	{
	 .reg = SX9310_CPS_CTRL9_REG, //cs1 TH
	 .val = 0x48,
	 },
	{
	 .reg = SX9310_CPS_CTRL10_REG,
	 .val = 0x10,
	 },
	{
	 .reg = SX9310_CPS_CTRL11_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_CPS_CTRL12_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_CPS_CTRL13_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_CPS_CTRL14_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_CPS_CTRL15_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_CPS_CTRL16_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_CPS_CTRL17_REG,
	 .val = 0x04,
	 },
	{
	 .reg = SX9310_CPS_CTRL18_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_CPS_CTRL19_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_SAR_CTRL0_REG,
	 .val = 0x00,
	 },
	{
	 .reg = SX9310_SAR_CTRL1_REG,
	 .val = 0x80,
	 },
	{
	 .reg = SX9310_SAR_CTRL2_REG,
	 .val = 0x0C,
	 },
	{
	 .reg = SX9310_CPS_CTRL0_REG,
	 .val = 0x50,
	 },
};

static struct input_dev *sx9310_input_device;
static unsigned int sar_sensor_irq = 0;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static unsigned int sar_sensor_flag = 0;
static struct task_struct *thread_sar_sensor;
static struct sx9310_data *psx9310_data = 0;
static void input_poll_work_func(struct work_struct *work);
static int sx9310_irq_registration(void);
static int read_calibration(void);

static void ForcetoTouched(struct sx9310_data *this)
{
	struct input_dev *input = NULL;

	if (this && (sx9310_input_device)) {
		SAR_INFO("ForcetoTouched()\n");
		input = sx9310_input_device;
		input_report_abs(input, psmtcButtons[0].keycode, 0);
		input_sync(input);
		psmtcButtons[0].state = ACTIVE;
		SAR_INFO("Leaving ForcetoTouched()\n");
	}
}

/*! fn static int write_register(struct sx9310_data * this, u8 address, u8 value)
 *  brief Sends a write register to the device
 *  param this Pointer to main parent struct
 *  param address 8-bit register address
 *  param value   8-bit register value to write to address
 *  return Value from i2c_master_send
 */
static int write_register(struct sx9310_data *this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;

	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;
	if (this && this->bus) {
		i2c = this->bus;
		returnValue = i2c_master_send(i2c, buffer, 2);
		SAR_INFO("write_register Address: 0x%x Value: 0x%x Return: %d\n", address, value, returnValue);
	}
	if (returnValue < 0) {
		ForcetoTouched(this);
		SAR_INFO("Write_register-ForcetoTouched()\n");
	}
	return returnValue;
}

/*! fn static int read_register(struct sx9310_data * this, u8 address, u8 *value)
* brief Reads a register's value from the device
* param this Pointer to main parent struct
* param address 8-Bit address to read from
* param value Pointer to 8-bit value to save register value to
* return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(struct sx9310_data *this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;

	if (this && value && this->bus) {
		i2c = this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c, address);
		SAR_INFO("read_register Address: 0x%x Return: 0x%x\n", address, returnValue);
		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} else {
			return returnValue;
		}
	}
	ForcetoTouched(this);
	SAR_INFO("read_register-ForcetoTouched()\n");
	return -ENOMEM;
}

static void apply_calibration_value(void)
{
	if(channel2_thre<=0) {
		write_register(psx9310_data, SX9310_CPS_CTRL9_REG, 48);
		channel2_thre = 48;
	}
	else
		write_register(psx9310_data, SX9310_CPS_CTRL9_REG, channel2_thre);
	if(channel3_thre<=0) {
		write_register(psx9310_data, SX9310_CPS_CTRL8_REG, 78);
		channel3_thre = 78;
		temp_thre2 = channel3_thre;
	}
	else{
		temp_thre2 = channel3_thre;
		write_register(psx9310_data, SX9310_CPS_CTRL8_REG, channel3_thre);
	}
	return;
}
/*********************************************************************/
/*!brief Perform a manual offset calibration
*param this Pointer to main parent struct
*return Value return value from the write register
 */
static int manual_offset_calibration(struct sx9310_data *this)
{
	s32 returnValue = 0;

	returnValue = write_register(this, SX9310_IRQSTAT_REG, 0xFF);
	return returnValue;
}

/*!brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
 #if 0
static ssize_t manual_offset_calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	struct sx9310_data *this = dev_get_drvdata(dev);

	SAR_INFO("Reading IRQSTAT_REG\n");
	read_register(this, SX9310_IRQSTAT_REG, &reg_value);
	return sprintf(buf, "%d\n", reg_value);
}

/*!brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9310_data *this = dev_get_drvdata(dev);
	/* unsigned long val; */
	/* if (val) { */
	SAR_INFO("Performing manual_offset_calibration()\n");
	manual_offset_calibration(this);
	/* } */
	return count;
}
#endif
static ssize_t noise_register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 noise_reg_high = 0;
	u8 noise_reg_low = 0;
	u16 noise_val = 0;
	struct sx9310_data *this = dev_get_drvdata(dev);

	read_register(this, SX9310_DIFFMSB, &noise_reg_high);
	read_register(this, SX9310_DIFFLSB, &noise_reg_low);
	noise_val = ((u16)noise_reg_high << 8) + noise_reg_low;
	return sprintf(buf, "%u\n", noise_val);
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	int i, j = 0;
	struct sx9310_data *this = dev_get_drvdata(dev);

	for (i = 0; i < 64; i++) {
		read_register(this, i, &reg_value);
		j += snprintf(buf + j, 4096, "reg:0x%02x is 0x%02x\n", i, reg_value);
	}
	return j;
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9310_data *this = dev_get_drvdata(dev);
	int reg_address = 0;
	int reg_val = 0;
	int ret;

	ret = sscanf(buf, "%x %x", &reg_address, &reg_val);
	if (ret == 2)
		write_register(this, reg_address, reg_val);
	return count;
}

static ssize_t onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(psmtcButtons[0].state == ACTIVE)
	return sprintf(buf,"0\n");
	else
	return sprintf(buf,"1\n");
}

static ssize_t onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;
	sscanf(buf, "%d", &value);
	read_calibration();
	apply_calibration_value();
	if(value ==1) {
			write_register(psx9310_data, SX9310_IRQ_ENABLE_REG, 0x70);
			write_register(psx9310_data, SX9310_CPS_CTRL0_REG, 0x51);
			msleep(50);
			write_register(psx9310_data, SX9310_IRQSTAT_REG, 0xff);
	}
	else {
		write_register(psx9310_data, SX9310_IRQ_ENABLE_REG, 0x00);
		write_register(psx9310_data, SX9310_CPS_CTRL0_REG, 0x50);
	}

	return count;
}

 static ssize_t regproxdata_show(struct device *dev, struct device_attribute *attr, char *buf)
 {
	u8 data = 0;
	u8 high = 0;
	u8 low = 0;
	u16 diff_val = 0;
	u16 avg_val = 0;
	u16 offset_val = 0;
	unsigned int diff_data=0;
	int j=0;
	u8 count=0;
	for(j=0;j<5;j++)
	{
	do
	{read_register(psx9310_data, SX9310_DIFFMSB, &high);
	read_register(psx9310_data, SX9310_DIFFLSB, &low);
	diff_val = ((u16)high << 8) + low;
	count++;
	msleep(TIME);
	}while(diff_val > 4095 && count<20); // >4095 is negative
	if(count==20)
		break;
	else{
		count=0;
		diff_data += diff_val;
	}
	}
	if(count==20)
		diff_val=0;
	else
		diff_val = diff_data/5;

	read_register(psx9310_data, SX9310_AVGMSB, &high);
	read_register(psx9310_data, SX9310_AVGLSB, &low);
	avg_val = ((u16)high << 8) + low;

	read_register(psx9310_data, SX9310_OFFSETMSB, &high);
	read_register(psx9310_data, SX9310_OFFSETLSB, &low);
	avg_val = ((u16)high << 8) + low;

	SAR_INFO("Register stat SAR Sensor 2\n");
	read_register(psx9310_data, 0x31, &data);
	read_register(psx9310_data, 0x32, &data);
	read_register(psx9310_data, 0x33, &data);
	read_register(psx9310_data, 0x34, &data);
	read_register(psx9310_data, 0x35, &data);
	read_register(psx9310_data, 0x36, &data);
	read_register(psx9310_data, 0x37, &data);
	read_register(psx9310_data, 0x38, &data);

	return sprintf(buf, "avg:0x%x diff:0x%x offset:0x%x\n", avg_val, diff_val, offset_val);
 }
 
//static DEVICE_ATTR(calibrate_1, 0660, a_manual_offset_calibration_show, a_manual_offset_calibration_store);
static DEVICE_ATTR(noise_2, 0440, noise_register_show, NULL);
static DEVICE_ATTR(reg_2, 0660, reg_show, reg_store);
static DEVICE_ATTR(onoff, 0660, onoff_show, onoff_store);
static DEVICE_ATTR(regproxdata, 0660, regproxdata_show, NULL);
//static struct attribute *sx9310_attributes[] = { &dev_attr_calibrate_1.attr,  &dev_attr_reg_1.attr, &dev_attr_noise_1.attr
//, NULL,};
static struct attribute *sx9310_debug_attributes[] = {&dev_attr_reg_2.attr, &dev_attr_noise_2.attr, NULL,};
static struct attribute_group sx9310_attr_group = {.attrs = sx9310_debug_attributes,
};

/*********************************************************************/

/*!fn static int read_regStat(struct sx9310_data * this)
 *brief Shortcut to read what caused interrupt.
 *details This is to keep the drivers a unified
 * function that will read whatever register(s)
 * provide information on why the interrupt was caused.
 *param this Pointer to main parent struct
 *return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(struct sx9310_data *this)
{
	u8 data = 0;

	if (this) {
		read_register(psx9310_data, 0x31, &data);
		read_register(psx9310_data, 0x32, &data);
		read_register(psx9310_data, 0x33, &data);
		read_register(psx9310_data, 0x34, &data);
		read_register(psx9310_data, 0x35, &data);
		read_register(psx9310_data, 0x36, &data);
		read_register(psx9310_data, 0x37, &data);
		read_register(psx9310_data, 0x38, &data);
		if (read_register(this, SX9310_IRQSTAT_REG, &data) == 0)
			return (data & 0x00FF);
	}
	return 0;
}

static int read_calibration(void)
{
	int fd;
	int res;
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = sys_open(SENSOR_CAL_FILENAME, O_RDONLY, 0);

	if(fd < 0)
	{
		SAR_ERROR("ERROR!! Cal data Not Found!!!\n");
		sys_close(fd);
		return -EINVAL;
	}

	else
	{
		memset(temp_str_2, 0x00, sizeof(temp_str_2));
		res = sys_read(fd, temp_str_2, sizeof(temp_str_2));
		if(res < 0)
		{
			SAR_ERROR("ERROR!! Cal data read fail!!!\n");
			sys_close(fd);
			return -EINVAL;
		}
		sys_close(fd);
		set_fs(old_fs);

		sscanf(temp_str_2,"0x%x\n0x%x",&channel2_thre,&channel3_thre);
		SAR_INFO("thre1 : %x thre2 : %x\n",channel2_thre,channel3_thre);

		return 0;
	}
}

struct sx9310_entry {
	 struct attribute attr;
	 ssize_t (*show)(struct kobject *kobj, char *page);
	 ssize_t (*store)(struct kobject *kobj, const char *page, size_t size);
 };
static struct sx9310_sysobj {
	 struct kobject kobj;
	 atomic_t enable;
 } sx9310_sysobj = {
	 .enable = ATOMIC_INIT(0),
 };
static ssize_t sx9310_attr_show(struct kobject *kobj, struct attribute *attr, char *buffer)
 {
	 struct sx9310_entry *entry = container_of(attr, struct sx9310_entry, attr);
	 return entry->show(kobj, buffer);
 }
 static ssize_t sx9310_attr_store(struct kobject *kobj, struct attribute *attr, const char *buffer, size_t size)
 {
	 struct sx9310_entry *entry = container_of(attr, struct sx9310_entry, attr);
	 return entry->store(kobj, buffer, size);
 }
 static ssize_t sx9310_enable_show(struct kobject *kobj, char *buffer)
 {
	return sprintf(buffer,"%d,0x%x\n",sar_sensor_enable,channel_thre); 	
 }
 static ssize_t sx9310_enable_store(struct kobject *kobj, const char *buffer, size_t size)
 {
 	u8 ret=0;
 	ret = sscanf(buffer, "%d,%x",&sar_sensor_enable,&channel_thre);
 	printk("sar sensor:%d,0x%x,%d",sar_sensor_enable,channel_thre,ret);
	if (ret == 2)
	{
		if(sar_sensor_enable==1) //open cs0 cs1 channels
			{
			if(channel_thre<=0)
				write_register(psx9310_data, SX9310_CPS_CTRL8_REG, 78);
			else
				write_register(psx9310_data, SX9310_CPS_CTRL8_REG, channel_thre);
			write_register(psx9310_data, SX9310_IRQ_ENABLE_REG, 0x70);  //enable all interrput
			write_register(psx9310_data, SX9310_CPS_CTRL0_REG, 0x51);
			msleep(50);
			write_register(psx9310_data, SX9310_IRQSTAT_REG, 0xff);
			}
		else
			{
			write_register(psx9310_data, SX9310_IRQ_ENABLE_REG, 0x00);  //disable all interrput
			write_register(psx9310_data, SX9310_CPS_CTRL0_REG, 0x50);
			}
	}
	return size;
 }
 
 static ssize_t sx9310_calibration_show(struct kobject *kobj, char *buffer)
 {
        return sprintf(buffer,"0x%x\n",threshold);
 }
 static ssize_t sx9310_diff_value_show(struct kobject *kobj, char *buffer)
 {
	u8 noise_reg_high = 0;
	u8 noise_reg_low = 0;
	u16 noise_val = 0;
	unsigned int diff_data=0;
	int j=0;
	u8 count=0;
	for(j=0;j<5;j++)
	{
	do
	{read_register(psx9310_data, SX9310_DIFFMSB, &noise_reg_high);
	read_register(psx9310_data, SX9310_DIFFLSB, &noise_reg_low);
	noise_val = ((u16)noise_reg_high << 8) + noise_reg_low;
	count++;
	msleep(TIME);
	}while(noise_val > 4095 && count<20); // >4095 is negative
	if(count==20)
		break;
	else{
		count=0;
		diff_data += noise_val;
	}
	}
	if(count==20)
		noise_val=0;
	else
		noise_val = diff_data/5;
	return sprintf(buffer, "0x%x\n", noise_val);
 }
 static ssize_t sx9310_id_show(struct kobject *kobj, char *buffer)
 {
	//u8 sensor_id = 0;
	//read_register(psx9310_data, SX9310_ID, &sensor_id);
	SAR_INFO("sensor_id:%d\n", sensor_flag);
	return sprintf(buffer, "%d\n", sensor_flag);
 }
 static ssize_t sx9310_calibration(void)
 {
 	u8 noise_reg_high = 0;
	u8 noise_reg_low = 0;
	u16 noise_val = 0;
	int j;
	unsigned int cali_data=0;
	u8 count=0;
	for(j=0;j<10;j++)
		{
			do
			{read_register(psx9310_data, SX9310_DIFFMSB, &noise_reg_high);
			read_register(psx9310_data, SX9310_DIFFLSB, &noise_reg_low);
			noise_val = ((u16)noise_reg_high << 8) + noise_reg_low;
			count++;
			msleep(TIME);
			}while(noise_val > 4095 && count<20);  // >4095 is negative
			if(count==20)
				break;
			else{
			count=0;
			cali_data += noise_val;
			}
		}
	if(count==20)
			noise_val=0;
	else
			noise_val = cali_data/10;
		//return sprintf(buf, "%u\n", noise_val);
	if(noise_val<24 && noise_val>=20)
			threshold=(THRESHOLD+0)<<3;   //0x18;  //12
	else if(noise_val<28 && noise_val>=24)
			threshold=(THRESHOLD+1)<<3;//0x20;  //16
	else if(noise_val<32 && noise_val>=28)
			threshold=(THRESHOLD+2)<<3;//0x28;  //20
	else if(noise_val<40 && noise_val>=32)
			threshold=(THRESHOLD+3)<<3;//0x30;  //24
	else if(noise_val<48 && noise_val>=40)
			threshold=(THRESHOLD+4)<<3;//0x38;  //28
	else if(noise_val<56 && noise_val>=48)
			threshold=(THRESHOLD+5)<<3;//0x40;  //32
	else if(noise_val<64 && noise_val>=56)
			threshold=(THRESHOLD+6)<<3;//0x48;  //40
	else if(noise_val<72 && noise_val>=64)
			threshold=(THRESHOLD+7)<<3;//0x50;  //48
	else if(noise_val<80 && noise_val>=72)
			threshold=(THRESHOLD+8)<<3;//0x50;  //48
	else if(noise_val<88 && noise_val>=80)
			threshold=(THRESHOLD+9)<<3;//0x58;  //56
	else if(noise_val<96 && noise_val>=88)
			threshold=(THRESHOLD+10)<<3;//0x60;  //64
	else if(noise_val<112 && noise_val>=96)
			threshold=(THRESHOLD+11)<<3;//0x68;  //72
	else if(noise_val<128 && noise_val>=112)
			threshold=(THRESHOLD+12)<<3;//0x70;  //80
	else if(noise_val<144 && noise_val>=128)
			threshold=(THRESHOLD+13)<<3;//0x78;  //88
	else if(noise_val<160 && noise_val>=144)
			threshold=(THRESHOLD+14)<<3;//0x80;  //96
	else if(noise_val<192 && noise_val>160)
			threshold=(THRESHOLD+15)<<3;//0x88;  //112
	else if(noise_val<224 && noise_val>=192)
			threshold=(THRESHOLD+16)<<3;//0x90;  //128
	else if(noise_val<256 && noise_val>=224)
			threshold=(THRESHOLD+17)<<3;//0x98;  //144
	else if(noise_val<320 && noise_val>=256)
			threshold=(THRESHOLD+18)<<3;//0xa0;  //160
	else if(noise_val<384 && noise_val>=320)
			threshold=(THRESHOLD+19)<<3;//0xa8;  //192
	else if(noise_val<512 && noise_val>=384)
			threshold=(THRESHOLD+20)<<3;//0xb0;  //224
	else if(noise_val<640 && noise_val>=512)
			threshold=(THRESHOLD+21)<<3;//0xb8;  //256
	else if(noise_val<768 && noise_val>=640)
			threshold=(THRESHOLD+22)<<3;//0xc0;  //320
	else if(noise_val<1024 && noise_val>=768)
			threshold=(THRESHOLD+23)<<3;//0xc8;  //384
	else if(noise_val<1536 && noise_val>=1024)
			threshold=(THRESHOLD+24)<<3;//0xd0;  //512
	else
			threshold=0x00;
	write_register(psx9310_data, SX9310_CPS_CTRL8_REG, threshold);
	return 0;
 }
 static ssize_t sx9310_calibration_store(struct kobject *kobj, const char *buffer, size_t size)
 {

	int value=0;
 	sscanf(buffer, "%d", &value);
 	
	if (value==1 && sar_flag==0)
	{
		//struct sx9310_data *this = dev_get_drvdata(dev);
		/* unsigned long val; */
		/* if (val) { */
		sar_flag=1;
		read_register(psx9310_data, SX9310_CPS_CTRL0_REG, &old_status);
		write_register(psx9310_data, SX9310_IRQ_ENABLE_REG, 0x00);  //disable all interrput
		write_register(psx9310_data, SX9310_CPS_CTRL0_REG, 0x21);  //enable channel 0 30ms
		write_register(psx9310_data, SX9310_CPSRD, 0x00);
		SAR_INFO("Performing manual_offset_calibration()\n");
		msleep(50);
		write_register(psx9310_data, SX9310_IRQSTAT_REG, 0xff);
		//manual_offset_calibration(psx9310_data);
	}
	else if (value==2 && sar_flag==1)
	{	
		sar_flag=0;
		sx9310_calibration();
		write_register(psx9310_data, SX9310_CPS_CTRL0_REG, old_status);
        }
        else
        	;
        
        return size;
 }
 
  static ssize_t sx9310_read_cal_show(struct kobject *kobj, char *buffer)
 {
	int value = read_calibration();
	if (value < 0)
	{
		return sprintf(buffer, "fail\n");
	}
	else
	{
		return sprintf(buffer, "%s\n",temp_str_2);
	}
 }
 
 static void sx9310_sysfs_release(struct kobject *kobj)
 {
	 struct sx9310_sysobj		 *ge_sysfs;

	 ge_sysfs=container_of(kobj, struct sx9310_sysobj, kobj);

	 kfree(ge_sysfs);
 }

 /*---------------------------------------------------------------------------*/
static struct sysfs_ops sx9310_sysfs_ops = {
	 .show	 = sx9310_attr_show,
	 .store  = sx9310_attr_store,
 };
 /*---------------------------------------------------------------------------*/
 /*---------------------------------------------------------------------------*/
 static struct sx9310_entry sx9310_enable_entry = {
	 { .name = "sx9310_enable", .mode = 0664 },
	 sx9310_enable_show,
	 sx9310_enable_store,
 };
 /*---------------------------------------------------------------------------*/
 static struct sx9310_entry sx9310_calibration_entry = {
	 { .name = "sx9310_calibration", .mode = 0664},
	 sx9310_calibration_show,
	 sx9310_calibration_store,
 };
 /*---------------------------------------------------------------------------*/

 /*---------------------------------------------------------------------------*/
 static struct sx9310_entry sx9310_diff_value_entry = {
	 { .name = "sx9310_diff_value", .mode = 0664},
	 sx9310_diff_value_show,
 };
 /*---------------------------------------------------------------------------*/
 static struct sx9310_entry sx9310_read_cal_entry = {
	 { .name = "sx9310_read_cal", .mode = 0664 },
	 sx9310_read_cal_show,
 };
  /*---------------------------------------------------------------------------*/
 static struct sx9310_entry sx9310_id_entry = {
	 { .name = "sx9310_id", .mode = 0664},
	 sx9310_id_show,
 };
 /*---------------------------------------------------------------------------*/
 static struct attribute *sx9310_attributes[] = {
	 &sx9310_enable_entry.attr,  /*enable setting*/
	 &sx9310_calibration_entry.attr,
	 &sx9310_diff_value_entry.attr,
	 &sx9310_id_entry.attr,
	 &sx9310_read_cal_entry.attr,
	 &dev_attr_onoff.attr,
	 NULL,
 };
 /*---------------------------------------------------------------------------*/
 static struct attribute *sx9310_attributes_input[] = {
	 &dev_attr_onoff.attr,
	 &dev_attr_regproxdata.attr,
	 &dev_attr_reg_2.attr,
	 NULL,
 };
  /*---------------------------------------------------------------------------*/
 static struct attribute_group sx9310_attr_group_input = {
	 .attrs = sx9310_attributes_input,
	 NULL,
 };
 /*---------------------------------------------------------------------------*/
 static struct kobj_type sx9310_ktype = {
	 .sysfs_ops = &sx9310_sysfs_ops,
	 .release =sx9310_sysfs_release,
	 .default_attrs = sx9310_attributes,
 };
 /*---------------------------------------------------------------------------*/

 /*---------------------------------------------------------------------------*/
 static int sx9310_sysfs(struct i2c_client * client)
 {
	 struct sx9310_sysobj *obj = &sx9310_sysobj;

	 memset(&obj->kobj, 0x00, sizeof(obj->kobj));

	 atomic_set(&obj->enable, 0);

	 obj->kobj.parent = kernel_kobj;
	 if (kobject_init_and_add(&obj->kobj, &sx9310_ktype, NULL, "sx9310_2")) {
		 kobject_put(&obj->kobj);
		 printk("error sx9310_sysfs ");
		 return -ENOMEM;
	 }
	 kobject_uevent(&obj->kobj, KOBJ_ADD);
	 return 0;
 }
/*!brief  initialize I2C config from platform data
 *param this Pointer to main parent struct
 */
static void hw_init(struct sx9310_data *this)
{
	int i = 0;

	/* configure device */
	SAR_INFO("Going to Setup I2C Registers\n");
	if (this) {
		while (i < ARRAY_SIZE(sx9310_i2c_reg_setup)) {

			/* Write all registers/values contained in i2c_reg */
			SAR_INFO("Going to Write Reg: 0x%x Value: 0x%x\n", sx9310_i2c_reg_setup[i].reg, sx9310_i2c_reg_setup[i].val);

/* msleep(3); */
			write_register(this, sx9310_i2c_reg_setup[i].reg, sx9310_i2c_reg_setup[i].val);
			i++;
		}
	} else {

		/* Force to touched if error */
		ForcetoTouched(this);
		SAR_INFO("Hardware_init-ForcetoTouched()\n");
	}
}

/*********************************************************************/

/*!fn static int initialize(struct sx9310_data * this)
 *brief Performs all initialization needed to configure the device
 *param this Pointer to main parent struct
 *return Last used command's return value (negative if error)
 */
static int initialize(struct sx9310_data *this)
{
	if (this) {

		/* prepare reset by disabling any irq handling */
		this->irq_disabled = 1;
		disable_irq(sar_sensor_irq);

		/* perform a reset */
		write_register(this, SX9310_SOFTRESET_REG, SX9310_SOFTRESET);

		/* wait until the reset has finished by monitoring NIRQ */
		SAR_INFO("Sent Software Reset. Waiting until device is back from reset to continue.\n");

		/* just sleep for awhile instead of using a loop with reading irq status */
		msleep(50);

		hw_init(this);
		msleep(50);	/* make sure everything is running */
		manual_offset_calibration(this);

		/* re-enable interrupt handling */
		enable_irq(sar_sensor_irq);
		this->irq_disabled = 0;

		/* make sure no interrupts are pending since enabling irq will only
		 * work on next falling edge */
		read_regStat(this);
		INIT_WORK(&this->input_poll_work, input_poll_work_func);
		SAR_INFO("Exiting initialize().\n");
		return 0;
	}
	return -ENOMEM;
}

static void compensation_process(struct sx9310_data *this)
{
	/* User released button */
	SAR_INFO("compensation interrupt enter\n");
	input_report_abs(sx9310_input_device, psmtcButtons[0].keycode, 1);
	input_sync(sx9310_input_device);
	psmtcButtons[0].state = IDLE;
	SAR_INFO("Leaving compensation interrupt\n");
}
/*!
 *brief Handle what to do when a touch occurs
 *param this Pointer to main parent struct
 */
static void touchProcess(struct sx9310_data *this)
{
	int counter = 0;
	u8 i = 0;
	int numberOfButtons = 0;
	struct _buttonInfo *buttons = NULL;
	struct input_dev *input = NULL;
	struct _buttonInfo *pCurrentButton = NULL;
	SAR_INFO("Inside touchProcess()\n");
	if (this) {
		SAR_INFO("Inside touchProcess()\n");
		read_register(this, SX9310_STAT0_REG, &i);
		buttons = psmtcButtons;
		input = sx9310_input_device;
		numberOfButtons = ARRAY_SIZE(psmtcButtons);
		if (unlikely((buttons == NULL) || (input == NULL))) {
			SAR_ERROR("ERROR!! buttons or input NULL!!!\n");
			return;
		}
		for (counter = 0; counter < numberOfButtons; counter++) {
			pCurrentButton = &buttons[counter];
			if (pCurrentButton == NULL) {
				SAR_ERROR("ERROR!! current button at index: %d NULL!!!\n", counter);
				return;	/* ERRORR!!!! */
			}
			SAR_ERROR("pCurrentButton[%d]->state: %d \n", counter,pCurrentButton->state);
			switch (pCurrentButton->state) {
			case IDLE:	/* Button is not being touched! */
				if (((i & pCurrentButton->mask) == pCurrentButton->mask)) {

					/* User pressed button */
					SAR_INFO("cap button %d touched\n", counter);
					hrtimer_start( &this->hr_timer, ktime2, HRTIMER_MODE_REL );
					input_report_abs(input, pCurrentButton->keycode, 0);
					input_sync(input);
					pCurrentButton->state = ACTIVE;
				} else {
					SAR_INFO("Button %d already released.\n", counter);
				}
				break;
			case ACTIVE:	/* Button is being touched! */
				if (((i & pCurrentButton->mask) != pCurrentButton->mask)) {

					/* User released button */
					SAR_INFO("cap button %d released\n", counter);
					hrtimer_cancel( &this->hr_timer );
					channel3_thre = temp_thre2;
					thre_flag2 = 0;
					write_register(this, SX9310_CPS_CTRL8_REG, channel3_thre);
					input_report_abs(input, pCurrentButton->keycode, 1);
					input_sync(input);
					pCurrentButton->state = IDLE;
				} else {
					SAR_INFO("Button %d still touched.\n", counter);
				}
				break;
			default:	/* Shouldn't be here, device only allowed ACTIVE or IDLE */
				break;
			};
		}
		SAR_INFO("Leaving touchProcess()\n");
	}
}

static int read_thermal(char const* path)
{
    int fd;
    int ret = 0;

    if (path == NULL)
    	return 0;

    fd = sys_open(path, O_RDONLY, 0);
    if (fd >= 0) {
    	char buffer[20];
    	int len = sys_read(fd, buffer, 19);
		if (len < 0) {
            sys_close(fd);
            return 0;
		}

    	if (sscanf(buffer, "%d", &ret))
    	{
    	    sys_close(fd);
    	    // success
    	    return ret;
    	}
    	sys_close(fd);
    	return 0;
    }
    return 0;
}

static void input_poll_work_func(struct work_struct *work)
{
	struct sx9310_data *stat;
	u8 data = 0;
	thermal_value2 = read_thermal("sys/class/thermal/thermal_zone0/temp");
	thermal_value2 = thermal_value2/1000;
	stat = container_of((struct work_struct *) work,
			struct sx9310_data, input_poll_work);
	read_register(stat, 0x35, &data);
	SAR_INFO("state %d, data %d, thre %d, thermal %d SAR Controller 2\n",psmtcButtons[0].state,data,thre_flag2,thermal_value2);
	if( psmtcButtons[0].state == ACTIVE && data > 4 && thre_flag2 == 0 && ( thermal_value2 > 50 || thermal_value2 < 0) ) {

		thre_flag2 = 1;
		temp_thre2 = channel3_thre;
		channel3_thre = 0xD1;
		SAR_INFO("Compensation SAR Controller 1\n");
		msleep(50);
		write_register(stat, SX9310_IRQSTAT_REG, 0xFF);
		hrtimer_cancel( &stat->hr_timer );
	}
}

enum hrtimer_restart check_threshold2( struct hrtimer *timer )
{
  SAR_INFO("watching SAR Controller 2()\n");

  queue_work(psx9310_data->hr_timer_poll_work_queue, &psx9310_data->input_poll_work);
  hrtimer_start( &psx9310_data->hr_timer, ktime_cal2, HRTIMER_MODE_REL );
  return HRTIMER_NORESTART;
}

static irqreturn_t sx9310_eint_interrupt_handler(int irq, void *dev_id)
{
	sar_sensor_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

/*****************************************************************************
*  Name: sp9310_event_handler
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int sx9310_event_handler(void *unused)
{
	int status = 0;
	int counter = 0;
	struct sched_param param = {.sched_priority = 4 };	/* RTPM_PRIO_TPD */

	sched_setscheduler(current, SCHED_RR, &param);

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, sar_sensor_flag != 0);

		sar_sensor_flag = 0;

		set_current_state(TASK_RUNNING);

		status = psx9310_data->refreshStatus(psx9310_data);
		counter = -1;
		SAR_INFO("Worker - Refresh Status %d\n", status);
		while ((++counter) < MAX_NUM_STATUS_BITS) {	/* counter start from MSB */
			SAR_INFO("Looping Counter %d\n", counter);
			if (((status >> counter) & 0x01) && (psx9310_data->statusFunc[counter])) {
				SAR_INFO("Function Pointer Found. Calling\n");
				psx9310_data->statusFunc[counter] (psx9310_data);
			}
		}

	} while (!kthread_should_stop());

	return 0;
}

static int sx9310_init(struct sx9310_data *this)
{
	int ret=0;

	if (this) {
		ret=sx9310_irq_registration();
		if(ret<0)
		   return -1;
		if (this->init)
			return this->init(this);
		SAR_ERROR("No init function!!!!\n");
	}
	return -ENOMEM;
}

static int sx9310_remove(struct sx9310_data *this)
{
	if (this) {

		kfree(this);
		return 0;
	}
	return -ENOMEM;
}

/*!fn static int sx9310_probe(struct i2c_client *client, const struct i2c_device_id *id)
 *brief Probe function
 *param client pointer to i2c_client
 *param id pointer to i2c_device_id
 *return Whether probe was successful
 */
static int sx9310_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i = 0;
	int retval = 0;
	int reset_count=0;
	struct input_dev *input = NULL;

	SAR_INFO("sx9310_probe()\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;
reset_proc:
	retval =i2c_smbus_read_byte_data(client,0x42);
	SAR_INFO("sarsensor_i2c:data:%d\n", retval);
	if(retval!=1)// reg0 data running state is 0; other state is not 0
	{
		if ( ++reset_count < SENSOR_ID_COUNT )
		{
			goto reset_proc;
		}
		return -1;
	}
	retval=sx9310_sysfs(client);
	if(retval<0)
		return -1;
	psx9310_data = kzalloc(sizeof(struct sx9310_data), GFP_KERNEL);	/* create memory for main struct */
	SAR_INFO("\t Initialized Main Memory: 0x%p\n", psx9310_data);
	if (psx9310_data == NULL) {
		SAR_ERROR("\t sx9310_data kzalloc Fail\n");
		return -1;
	}

	/* In case we need to reinitialize data
	 * (e.q. if suspend reset device) */
	psx9310_data->init = initialize;

	/* shortcut to read status of interrupt */
	psx9310_data->refreshStatus = read_regStat;

	/* pointer to function from platform data to get pendown
	 * (1->NIRQ=0, 0->NIRQ=1) */
	/* psx9310_data->get_nirq_low = pplatData->get_is_nirq_low; */

	/* save irq in case we need to reference it */
	psx9310_data->irq = client->irq;


	/* Setup function to call on corresponding reg irq source bit */
	if (MAX_NUM_STATUS_BITS >= 8) {
		psx9310_data->statusFunc[0] = 0;	/* TXEN_STAT */
		psx9310_data->statusFunc[1] = 0;	/* UNUSED */
		psx9310_data->statusFunc[2] = 0;	/* UNUSED */
		psx9310_data->statusFunc[3] = 0;	/* CONV_STAT */
		psx9310_data->statusFunc[4] = compensation_process;	/* COMP_STAT */
		psx9310_data->statusFunc[5] = touchProcess;	/* RELEASE_STAT */
		psx9310_data->statusFunc[6] = touchProcess;	/* TOUCH_STAT  */
		psx9310_data->statusFunc[7] = 0;	/* RESET_STAT */
	}

	/* setup i2c communication */
	psx9310_data->bus = client;
	i2c_set_clientdata(client, psx9310_data);

	/* record device struct */
	psx9310_data->pdev = &client->dev;

	/* for accessing items in user data (e.g. calibrate) */
	if (sysfs_create_group(&client->dev.kobj, &sx9310_attr_group) != 0)
		return -ENOMEM;

	/* Create the input device */
	input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	/* Set all the keycodes */
	__set_bit(EV_ABS, input->evbit);
	input_set_abs_params(input, ABS_DISTANCE, 0, 1, 0, 0);
	for (i = 0; i < ARRAY_SIZE(psmtcButtons); i++) {
		//__set_bit(psmtcButtons[i].keycode, input->keybit);
		psmtcButtons[i].state = IDLE;
	}

	/* save the input pointer and finish initialization */
	sx9310_input_device = input;
	input->name = "lge_sar_rf";
	input->id.bustype = BUS_I2C;
	input->dev.init_name = "lge_sar_rf";
	input_set_drvdata(input,psx9310_data);

	if (input_register_device(input)) {
		SAR_ERROR("[TPD]Failed to create sx9310 input device!");
		return -ENOMEM;
	}

	if(sysfs_create_group(&input->dev.kobj, &sx9310_attr_group_input)) {
		SAR_ERROR("[TPD]Failed to create input sysfs create fail!");
		return -ENOMEM;
	}

	thread_sar_sensor = kthread_run(sx9310_event_handler, 0, DRIVER_NAME);
	if (IS_ERR(thread_sar_sensor)) {
		retval = PTR_ERR(thread_sar_sensor);
		SAR_ERROR("[TPD]Failed to create kernel thread_sar_sensor,ret=%d!", retval);
		return -1;
	}

	retval=sx9310_init(psx9310_data);
	if(retval<0)
		return -1;
	sensor_flag=1;
	#ifdef CONFIG_HQ_HARDWARE_INFO	//WSP add for hardwareinfo
		get_hardware_info_data(HWID_SAR_SENSOR_3, DRIVER_NAME);
	#endif

	psx9310_data->hr_timer_poll_work_queue = 0;
	psx9310_data->hr_timer_poll_work_queue = create_workqueue("hr_timer_poll_wq2");
	hrtimer_init( &psx9310_data->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	ktime2 = ktime_set( 300, 0 );
	ktime_cal2 = ktime_set( 120, 0 );
	psx9310_data->hr_timer.function = &check_threshold2;

	return 0;

}

/*!fn static int sx9310_remove(struct i2c_client *client)
 *brief Called when device is to be removed
 *param client Pointer to i2c_client struct
 *return Value from sx9310_i2c_remove()
 */
static int sx9310_i2c_remove(struct i2c_client *client)
{
	struct sx9310_data *this = i2c_get_clientdata(client);
	if (this->hr_timer_poll_work_queue) {
	flush_workqueue(this->hr_timer_poll_work_queue);
	destroy_workqueue(this->hr_timer_poll_work_queue);
	}
	if (this) {
		input_unregister_device(sx9310_input_device);
		sysfs_remove_group(&client->dev.kobj, &sx9310_attr_group);
	}
	return sx9310_remove(this);
}


/*====================================================*/

static const struct i2c_device_id sx9310_i2c_id[] = { {DRIVER_NAME, 0}, {} };

static const struct of_device_id sx9310_dt_match[] = {
	{.compatible = "mediatek,sarsensor_2"}, {},
};

MODULE_DEVICE_TABLE(of, sx9310_dt_match);
static struct i2c_driver sx9310_i2c_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = of_match_ptr(sx9310_dt_match),
		   },
	.probe = sx9310_i2c_probe,
	.remove = sx9310_i2c_remove,
	.id_table = sx9310_i2c_id,
};


/*================sar sensor platform driver====================================*/
static const struct of_device_id sar_sensor_dt_match[] = {
	{.compatible = "mediatek,sar_sensor_2"}, {},
};

MODULE_DEVICE_TABLE(of, sar_sensor_dt_match);
static int sx9310_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	SAR_FUNC_ENTER();
	node = of_find_matching_node(node, sar_sensor_dt_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		sar_sensor_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(sar_sensor_irq, sx9310_eint_interrupt_handler, IRQF_TRIGGER_FALLING, "Sar Sensor-eint 2", NULL);
		if (ret !=0)
			{
			return -1;
			SAR_ERROR("sar sensor request_irq IRQ LINE NOT AVAILABLE!.");
			}
		else
			SAR_INFO("IRQ request succussfully, irq=%d", sar_sensor_irq);
	} else {
		SAR_ERROR("Can not find touch eint device node!");
		return -1;
	}
	SAR_FUNC_EXIT();
	return 0;
}
static int sar_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	/*configure to GPIO function, external interrupt */
	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		SAR_ERROR("Cannot find sar_sensor pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default_B");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		SAR_ERROR("Cannot find sar_senser pinctrl default!\n");
	}
	pinctrl_select_state(pinctrl, pins_default);

	return i2c_add_driver(&sx9310_i2c_driver);
}

static int sar_sensor_remove(struct platform_device *pdev)
{
	SAR_FUNC_ENTER();
	i2c_del_driver(&sx9310_i2c_driver);
	SAR_FUNC_EXIT();
	return 0;
}





MODULE_DEVICE_TABLE(of, sx9310_dt_match);
static const struct dev_pm_ops sar_sensor_pm_ops = {
	.suspend = NULL,
	.resume = NULL,
};

static struct platform_driver sar_sensor_driver = {
	.remove = sar_sensor_remove,
	.shutdown = NULL,
	.probe = sar_sensor_probe,
	.driver = {
		   .name = "sar sensor 2",
		   .pm = &sar_sensor_pm_ops,
		   .owner = THIS_MODULE,
		   .of_match_table = sar_sensor_dt_match,
		   },
};

static void async_sx9310_platform_driver_init(void *data, async_cookie_t cookie)
{
	if (platform_driver_register(&sar_sensor_driver) != 0)
		SAR_ERROR("Failed to register platform driver\n");
	return;
}

static int __init sx9310_platform_driver_init(void)
{
	async_schedule(async_sx9310_platform_driver_init, NULL);
	return 0;
}

static void __exit sx9310_platform_driver_exit(void)
{

	platform_driver_unregister(&sar_sensor_driver);
}

late_initcall(sx9310_platform_driver_init);

module_exit(sx9310_platform_driver_exit);
MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9310 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");