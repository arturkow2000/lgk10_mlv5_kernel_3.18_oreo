/*!
* @section LICENSE
 * (C) Copyright 2011~2017 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
*
* @filename bhy_acc.c
* @date     "Wed Aug 17 17:00:04 2016 +0800"
* @id       "3b8d32d"
*
* @brief
* The implementation file for BHy driver core
*/

#define DRIVER_VERSION "1.3.18.0"

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/swab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/firmware.h>
#include <linux/hrtimer.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
//#include <mach/mt_gpio.h>

//#include <linux/hwmsensor.h>
//#include <linux/hwmsen_dev.h>
//#include <linux/sensors_io.h>
#include <hwmsensor.h>
#include <sensors_io.h>
//#include <linux/hwmsen_helper.h>
#include <hwmsen_helper.h>
#include <accel.h>
#include <cust_acc.h>

#include "bhy_core.h"
#include "bhy_host_interface.h"
#include "bs_log.h"
#include "default_fw.h"

//#define FW_DEBUG      //for BHI160 firmware debug info print
#define SW_CALIBRATION

extern int bhy_gesture_notify(void);

static unsigned int bhy_irq = 0;
struct bhy_client_data *obj_i2c_data;
struct i2c_client *bmi160_acc_i2c_client;
static struct acc_init_info bmi160_acc_init_info;
static bool sensor_power = true;
//static struct GSENSOR_VECTOR3D gsensor_gain;

static int bmi160_acc_init_flag = -1;
struct acc_hw accel_cust;
//static struct acc_hw *hw = &accel_cust;

static const struct i2c_device_id bmi160_acc_i2c_id[] = {
	{BMI160_DEV_NAME, 0}, {}
};

static struct data_resolution bmi160_acc_data_resolution[] = {
	/*8 combination by {FULL_RES,RANGE} */
	{{0, 6}, 16384},	/*+/-2g  in 16-bit resolution:  0.06 mg/LSB */
	{{0, 12}, 8192},	/*+/-4g  in 16-bit resolution:  0.12 mg/LSB */
	{{0, 24}, 4096},	/*+/-8g  in 16-bit resolution:  0.24 mg/LSB */
	{{0, 5}, 2048},		/*+/-16g in 16-bit resolution:  0.49 mg/LSB */
};

/*static struct data_resolution bmi160_acc_offset_resolution = {{0, 12}, 8192};*/

int bhy_gesture_status = BHY_STATUS_DISABLE;
static s16 acc_data[BMI160_ACC_AXES_NUM] = { 0 };
s16 gyro_data[BMI160_ACC_AXES_NUM] = { 0 };

#define BHY_MAX_RETRY_I2C_XFER		10
#define BHY_I2C_WRITE_DELAY_TIME	1000
#define BHY_I2C_MAX_BURST_WRITE_LEN	64

#define BHY_SUPPORT_I2C_DMA 1
#if BHY_SUPPORT_I2C_DMA
#include <linux/dma-mapping.h>
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
#define BHY_DMA_MAX_TRANSACTION_LENGTH  511
#define BHY_DMA_MAX_ALLOCATE  512
#define BHY_DMA_DIVIDED
#endif
static ssize_t bhy_store_mapping_matrix(struct device_driver *ddri, const char *buf, size_t count);
/* Ellen add for foc part1 bug fix */
static ssize_t bhy_store_mapping_matrix_acc(struct device_driver *ddri, const char *buf, size_t count);

static s32 bhy_i2c_read_internal(struct i2c_client *client,
		u8 reg, u8 *data, u16 len)
{
#if 0
	int ret, retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	if (len <= 0)
		return -EINVAL;

	for (retry = 0; retry < BHY_MAX_RETRY_I2C_XFER; retry++) {
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (ret >= 0)
			break;
		usleep_range(BHY_I2C_WRITE_DELAY_TIME,
				BHY_I2C_WRITE_DELAY_TIME);
	}

	return ret;
	/*int ret;
	if ((ret = i2c_master_send(client, &reg, 1)) < 0)
		return ret;
	return i2c_master_recv(client, data, len);*/
#else
	int ret,i,retry;
	
//printk("bhy_i2c_read_internal,data=%p,len=%d\n",data,len);
	
#if 0//for normal I2c transfer
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
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
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
#else// for DMA I2c transfer
	if(1)
	{
		//DMA Write
		if(1)//if(writelen < 8  )
		{
			for (retry = 0; retry < 2; ++retry)
			{
				ret= i2c_master_send(client, &reg, 1);
				
				if (ret >= 0)
	        	{
	        		break;
	        	}else
	       		{
	        		printk("@@@@@@@@@@@@DMA  read ERROR@@@@@@,retry=%d,ret=%d\n",retry,ret);
	       		}
			}
			
		}
		else
		{
			for(i = 0 ; i < len; i++)
			{
				gpDMABuf_va[i] = data[i];
			}

			client->addr = ((client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG);
		
			if((ret=i2c_master_send(client, (u8 *)(uintptr_t)gpDMABuf_pa, len))!=len)
				printk("### DMA ERROR %s i2c write len=%d\n", __func__,ret);

			client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);

		}
	}
	//DMA Read 
	if(len!=0)
	{
		if(0)//if (readlen <8) {
		{
			ret = i2c_master_recv(client, (unsigned char *)data, len);
		}
		else
		{

#ifdef BHY_DMA_DIVIDED
	int last_len = len;
	int copyPtr = 0;
	client->addr = ((client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG);
	while(last_len > 0){
		ret = i2c_master_recv(client, (u8 *)(uintptr_t)gpDMABuf_pa, min(last_len, (int)BHY_DMA_MAX_TRANSACTION_LENGTH));
		if(ret < 0){
			printk("bhy_i2c_read_internal read fail : %d, total read: %d\n", ret, copyPtr);
			client->addr = ((client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG));
			return ret;
		}
		else{
			for(i = 0; i < ret; i++)
			{
				data[copyPtr+i] = gpDMABuf_va[i];
			}
			last_len -= ret;
			copyPtr += ret;
		}
			
		
	}
	client->addr = ((client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG));

		
#else
			client->addr = ((client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG);
			ret = i2c_master_recv(client, (u8 *)(uintptr_t)gpDMABuf_pa, len);

			for(i = 0; i < len; i++)
	        {
	            data[i] = gpDMABuf_va[i];
	        }
		client->addr = ((client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG));
#endif


		}
	}
	#endif
	return ret;
#endif
}

static s32 bhy_i2c_write_internal(struct i2c_client *client,
		u8 reg, const u8 *data, u16 len)
{
#if 0
	int ret, retry;
	u8 buf[BHY_I2C_MAX_BURST_WRITE_LEN + 1];
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
	};

	if (len <= 0 || len > BHY_I2C_MAX_BURST_WRITE_LEN)
		return -EINVAL;

	buf[0] = reg;
	memcpy(&buf[1], data, len);
	msg.len = len + 1;
	msg.buf = buf;

	for (retry = 0; retry < BHY_MAX_RETRY_I2C_XFER; retry++) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret >= 0)
			break;
		usleep_range(BHY_I2C_WRITE_DELAY_TIME,
				BHY_I2C_WRITE_DELAY_TIME);
	}

	return ret;
#else
	int ret,retry;
	int i = 0;

   client->addr = client->addr & I2C_MASK_FLAG;
  // client->ext_flag |= I2C_DIRECTION_FLAG; 
  // client->timing = 100;
    #if 0
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);
	#else
	
	if(len >= BHY_DMA_MAX_TRANSACTION_LENGTH){
		printk("bhy_i2c_write_internal overflow fail : %d\n", len);
		dump_stack();
	}
		
	if(0)//if(writelen < 8)
	{
		
		//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
		ret = i2c_master_send(client, data, len);
	}
	else
	{
		gpDMABuf_va[0] = reg;
		for(i = 0 ; i < len; i++)
		{
			gpDMABuf_va[i+1] = data[i];
		}

		client->addr = ((client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG);

	for (retry = 0; retry < 2; ++retry)
	{
		if((ret=i2c_master_send(client, (u8 *)(uintptr_t)gpDMABuf_pa, len+1))!=len+1)
		{
		//	printk("### error  ###%s i2c write len=%d,buffaddr=%x\n", __func__,ret,gpDMABuf_pa);
		}else
		{
		//	printk("bhy_i2c_write_internal,ret=%d\n",ret);
		}
		 if (ret >= 0)
        {
        	break;
        }else
        {
        	printk("@@@@@@@@@@@@DMA write ERROR@@@@@@,retry=%d,ret=%d\n",retry,ret);
        }
		 usleep_range(BHY_I2C_WRITE_DELAY_TIME,
				BHY_I2C_WRITE_DELAY_TIME);
	}
	client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	} 
	#endif

	return ret;
#endif
}

static s32 bhy_i2c_read(struct device *dev, u8 reg, u8 *data, u16 len)
{
	struct i2c_client *client;
	//printk("bhy_i2c_read %p, %p, %d......\n", dev, data, len);
	client = to_i2c_client(dev);
	//printk(", client %p......\n", client);
	return bhy_i2c_read_internal(client, reg, data, len);
}

static s32 bhy_i2c_write(struct device *dev, u8 reg, const u8 *data, u16 len)
{
	struct i2c_client *client;
	client = to_i2c_client(dev);
	return bhy_i2c_write_internal(client, reg, data, len);
}

static int bhy_read_reg(struct bhy_client_data *client_data,
		u8 reg, u8 *data, u16 len)
{
	if (client_data == NULL)
		return -EIO;
	
	//printk("bhy_read_reg start %p, %p, %d......\n", client_data->data_bus.dev, data, len);
	return client_data->data_bus.read(client_data->data_bus.dev,
		reg, data, len);
}
int bhy_read_bus_reg(u8 reg, u8 *data, u16 len)
{
	return bhy_read_reg(obj_i2c_data, reg, data, len);
}
EXPORT_SYMBOL(bhy_read_bus_reg);

static int bhy_write_reg(struct bhy_client_data *client_data,
		u8 reg, const u8 *data, u16 len)
{
	if (client_data == NULL)
		return -EIO;
	return client_data->data_bus.write(client_data->data_bus.dev,
		reg, data, len);
}
int bhy_write_bus_reg(u8 reg, u8 *data, u16 len)
{
	return bhy_write_reg(obj_i2c_data, reg, data, len);
}
EXPORT_SYMBOL(bhy_write_bus_reg);

/* Dump BHy registers from 0x32 to 0xB5 */
static int bhy_dump_registers(struct bhy_client_data *client_data, char *buf,
	const char *line_hint)
{
	u8 reg_val[132];
	int i;
	int len;
	int ret;

	ret = bhy_read_reg(client_data, 0x32, reg_val, sizeof(reg_val));
	if (ret < 0) {
		snprintf(buf, 32, "Dump register failed");
		PERR("Dump register failed");
		return -EIO;
	}

	len = 0;
	len += snprintf(buf + len, 128, "BHy host register dump\n");
	len += snprintf(buf + len, 128,
		"%s> ## 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F",
		line_hint);
	for (i = 0x30; i <= 0xB5; ++i) {
		if (i % 0x10 == 0)
			len += snprintf(buf + len, 16, "\n%s> %02X",
				line_hint, i);
		if (i >= 0x32)
			len += snprintf(buf + len, 16, " %02X",
				reg_val[i - 0x32]);
		else
			len += snprintf(buf + len, 16, " **");
	}
	len += snprintf(buf + len, 16, "\n");

	return len;
}

int bhy_dump_hub_registers(char *buf, const char *line_hint)
{
	
	return(bhy_dump_registers(obj_i2c_data, buf, line_hint));
}
EXPORT_SYMBOL(bhy_dump_hub_registers);

static int bhy_read_parameter(struct bhy_client_data *client_data,
		u8 page_num, u8 param_num, u8 *data, u8 len)
{
	int ret, ret2;
	int retry;
	u8 ack, u8_val;
	char dump[256 * 3];

	/* Select page */
	ret = bhy_write_reg(client_data, BHY_REG_PARAM_PAGE_SEL, &page_num, 1);
	if (ret < 0) {
		PERR("Write page request failed");
		goto bhy_read_parameter_exit;
	}
	/* Select param */
	ret = bhy_write_reg(client_data, BHY_REG_PARAM_REQ, &param_num, 1);
	if (ret < 0) {
		PERR("Write param request failed");
		goto bhy_read_parameter_exit;
	}
	/* Wait for ack */
	retry = BHY_PARAM_ACK_WAIT_RETRY;
	while (retry--) {
		ret = bhy_read_reg(client_data, BHY_REG_PARAM_ACK, &ack, 1);
		if (ret < 0) {
			PERR("Read ack reg failed");
			goto bhy_read_parameter_exit;
		}
		if (ack == 0x80) {
			PERR("Param is not accepted");
			ret = -EINVAL;
			goto bhy_read_parameter_exit;
		}
		if (ack == param_num)
			break;
		usleep_range(10000, 20000);
	}
	if (retry == -1) {
		PERR("Wait for ack failed[%d, %d]", page_num, param_num);
		ret = -EBUSY;
		goto bhy_read_parameter_exit;
	}
	/* Fetch param data */
	ret = bhy_read_reg(client_data, BHY_REG_SAVED_PARAM_0, data, len);
	if (ret < 0) {
		PERR("Read saved parameter failed");
		goto bhy_read_parameter_exit;
	}
bhy_read_parameter_exit:
	if (ret < 0) {
		bhy_dump_registers(client_data, dump, "BHY");
		PDEBUG("%s", dump);
	}
	/* Clear up */
	u8_val = 0;
	ret2 = bhy_write_reg(client_data, BHY_REG_PARAM_PAGE_SEL, &u8_val, 1);
	if (ret2 < 0) {
		PERR("Write page sel failed on clear up");
		return ret2;
	}
	u8_val = 0;
	ret2 = bhy_write_reg(client_data, BHY_REG_PARAM_REQ, &u8_val, 1);
	if (ret2 < 0) {
		PERR("Write param_req failed on clear up");
		return ret2;
	}
	retry = BHY_PARAM_ACK_WAIT_RETRY;
	while (retry--) {
		ret2 = bhy_read_reg(client_data, BHY_REG_PARAM_ACK, &ack, 1);
		if (ret2 < 0) {
			PERR("Read ack reg failed");
			return ret2;
		}
		if (ack == 0)
			break;
		msleep(10);
	}
	if (retry == 0)
		PWARN("BHY_REG_PARAM_ACK cannot revert to 0 after clear up");
	if (ret < 0)
		return ret;
	return len;
}

static int bhy_write_parameter(struct bhy_client_data *client_data,
		u8 page_num, u8 param_num, const u8 *data, u8 len)
{
	int ret, ret2;
	int retry = BHY_PARAM_ACK_WAIT_RETRY;
	u8 param_num_mod, ack, u8_val;
	char dump[256 * 3];

	/* Write param data */
	ret = bhy_write_reg(client_data, BHY_REG_LOAD_PARAM_0, data, len);
	if (ret < 0) {
		PERR("Write load parameter failed");
		goto bhy_write_parameter_exit;
	}
	/* Select page */
	ret = bhy_write_reg(client_data, BHY_REG_PARAM_PAGE_SEL, &page_num, 1);
	if (ret < 0) {
		PERR("Write page request failed");
		goto bhy_write_parameter_exit;
	}
	/* Select param */
	param_num_mod = param_num | 0x80;
	ret = bhy_write_reg(client_data, BHY_REG_PARAM_REQ, &param_num_mod, 1);
	if (ret < 0) {
		PERR("Write param request failed");
		goto bhy_write_parameter_exit;
	}
	/* Wait for ack */
	while (retry--) {
		ret = bhy_read_reg(client_data, BHY_REG_PARAM_ACK, &ack, 1);
		if (ret < 0) {
			PERR("Read ack reg failed");
			goto bhy_write_parameter_exit;
		}
		if (ack == 0x80) {
			PERR("Param is not accepted");
			ret = -EINVAL;
			goto bhy_write_parameter_exit;
		}
		if (ack == param_num_mod)
			break;
		usleep_range(10000, 20000);
	}
	if (retry == -1) {
		PERR("Wait for ack failed[%d, %d]", page_num, param_num);
		ret = -EBUSY;
		goto bhy_write_parameter_exit;
	}
bhy_write_parameter_exit:
	if (ret < 0) {
		bhy_dump_registers(client_data, dump, "BHY");
		PDEBUG("%s", dump);
	}
	/* Clear up */
	u8_val = 0;
	ret2 = bhy_write_reg(client_data, BHY_REG_PARAM_PAGE_SEL, &u8_val, 1);
	if (ret2 < 0) {
		PERR("Write page sel failed on clear up");
		return ret2;
	}
	u8_val = 0;
	ret2 = bhy_write_reg(client_data, BHY_REG_PARAM_REQ, &u8_val, 1);
	if (ret2 < 0) {
		PERR("Write param_req failed on clear up");
		return ret2;
	}
	retry = BHY_PARAM_ACK_WAIT_RETRY;
	while (retry--) {
		ret2 = bhy_read_reg(client_data, BHY_REG_PARAM_ACK, &ack, 1);
		if (ret2 < 0) {
			PERR("Read ack reg failed");
			return ret2;
		}
		if (ack == 0)
			break;
		msleep(10);
	}
	if (retry == 0)
		PWARN("BHY_REG_PARAM_ACK cannot revert to 0 after clear up");
	if (ret < 0)
		return ret;
	return len;
}

/* Soft pass thru op, support max length of 4 */
static int bhy_soft_pass_thru_read_reg(struct bhy_client_data *client_data,
	u8 slave_addr, u8 reg, u8 *data, u8 len)
{
	int ret;
	u8 temp[8];
	int retry = BHY_SOFT_PASS_THRU_READ_RETRY;

	if (len > 4 || len <= 0) {
		PERR("Unsupported read len %d", len);
		return -EINVAL;
	}
	temp[0] = slave_addr;
	temp[1] = reg;
	temp[2] = len;
	ret = bhy_write_parameter(client_data, BHY_PAGE_SOFT_PASS_THRU,
		BHY_PARAM_SOFT_PASS_THRU_READ, temp, 8);
	if (ret < 0) {
		PERR("Write BHY_PARAM_SOFT_PASS_THRU_READ parameter failed");
		return -EIO;
	}
	do {
		udelay(50);
		ret = bhy_read_parameter(client_data, BHY_PAGE_SOFT_PASS_THRU,
			BHY_PARAM_SOFT_PASS_THRU_READ, temp, 8);
		if (ret < 0) {
			PERR("Read SOFT_PASS_THRU_READ parameter failed");
			return -EIO;
		}
		if (temp[3])
			break;
	} while (--retry);
	if (retry == 0) {
		PERR("Soft pass thru reg read timed out");
		return -EIO;
	}
	memcpy(data, temp + 4, len);

	return 0;
}

static int bhy_soft_pass_thru_write_reg(struct bhy_client_data *client_data,
	u8 slave_addr, u8 reg, u8 *data, u8 len)
{
	int ret;
	u8 temp[8];
	int retry = BHY_SOFT_PASS_THRU_READ_RETRY;

	if (len > 4 || len <= 0) {
		PERR("Unsupported write len %d", len);
		return -EINVAL;
	}
	temp[0] = slave_addr;
	temp[1] = reg;
	temp[2] = len;
	memcpy(temp + 4, data, len);
	ret = bhy_write_parameter(client_data, BHY_PAGE_SOFT_PASS_THRU,
		BHY_PARAM_SOFT_PASS_THRU_WRITE, temp, 8);
	if (ret < 0) {
		PERR("Write BHY_PARAM_SOFT_PASS_THRU_WRITE parameter failed");
		return -EIO;
	}
	do {
		udelay(50);
		ret = bhy_read_parameter(client_data, BHY_PAGE_SOFT_PASS_THRU,
			BHY_PARAM_SOFT_PASS_THRU_WRITE, temp, 8);
		if (ret < 0) {
			PERR("Read SOFT_PASS_THRU_WRITE parameter failed");
			return -EIO;
		}
		if (temp[3])
			break;
	} while (--retry);
	if (retry == 0) {
		PERR("Soft pass thru reg read timed out");
		return -EIO;
	}

	return 0;
}

static int bhy_soft_pass_thru_read_reg_m(struct bhy_client_data *client_data,
	u8 slave_addr, u8 reg, u8 *data, u8 len)
{
	int i;
	int ret;
	for (i = 0; i < len; ++i) {
		ret = bhy_soft_pass_thru_read_reg(client_data, slave_addr,
			reg + i, &data[i], 1);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int bhy_soft_pass_thru_write_reg_m(struct bhy_client_data *client_data,
	u8 slave_addr, u8 reg, u8 *data, u8 len)
{
	int i;
	int ret;
	for (i = 0; i < len; ++i) {
		ret = bhy_soft_pass_thru_write_reg(client_data, slave_addr,
			reg + i, &data[i], 1);
		if (ret < 0)
			return ret;
	}
	return 0;
}

/* Soft pass thru op(non-bust version), support max length of 4 */
#ifdef BHY_RESERVE_FOR_LATER_USE
static int bhy_soft_pass_thru_read_reg_nb(struct bhy_client_data *client_data,
	u8 slave_addr, u8 reg, u8 *data, u8 len)
{
	int ret;
	u8 temp[8];
	int retry = BHY_SOFT_PASS_THRU_READ_RETRY;

	if (len > 4 || len <= 0) {
		PERR("Unsupported read len %d", len);
		return -EINVAL;
	}
	temp[0] = slave_addr;
	temp[1] = reg;
	temp[2] = len;
	ret = bhy_write_parameter(client_data, BHY_PAGE_SOFT_PASS_THRU,
		BHY_PARAM_SOFT_PASS_THRU_READ_NONBURST, temp, 8);
	if (ret < 0) {
		PERR("Write BHY_PARAM_SOFT_PASS_THRU_READ parameter failed");
		return -EIO;
	}
	do {
		udelay(50);
		ret = bhy_read_parameter(client_data, BHY_PAGE_SOFT_PASS_THRU,
			BHY_PARAM_SOFT_PASS_THRU_READ_NONBURST, temp, 8);
		if (ret < 0) {
			PERR("Read SOFT_PASS_THRU_READ parameter failed");
			return -EIO;
		}
		if (temp[3])
			break;
	} while (--retry);
	if (retry == 0) {
		PERR("Soft pass thru reg read timed out");
		return -EIO;
	}
	memcpy(data, temp + 4, len);

	return 0;
}
#endif /*~ BHY_RESERVE_FOR_LATER_USE */

#ifdef BHY_RESERVE_FOR_LATER_USE
/* Still not working for now */
static int bhy_soft_pass_thru_write_reg_nb(struct bhy_client_data *client_data,
	u8 slave_addr, u8 reg, u8 *data, u8 len)
{
	int ret;
	u8 temp[8];
	int retry = BHY_SOFT_PASS_THRU_READ_RETRY;

	if (len > 4 || len <= 0) {
		PERR("Unsupported write len %d", len);
		return -EINVAL;
	}
	temp[0] = slave_addr;
	temp[1] = reg;
	temp[2] = len;
	memcpy(temp + 4, data, len);
	ret = bhy_write_parameter(client_data, BHY_PAGE_SOFT_PASS_THRU,
		BHY_PARAM_SOFT_PASS_THRU_WRITE_NONBURST, temp, 8);
	if (ret < 0) {
		PERR("Write BHY_PARAM_SOFT_PASS_THRU_WRITE parameter failed");
		return -EIO;
	}
	do {
		udelay(50);
		ret = bhy_read_parameter(client_data, BHY_PAGE_SOFT_PASS_THRU,
			BHY_PARAM_SOFT_PASS_THRU_WRITE_NONBURST, temp, 8);
		if (ret < 0) {
			PERR("Read SOFT_PASS_THRU_WRITE parameter failed");
			return -EIO;
		}
		if (temp[3])
			break;
	} while (--retry);
	if (retry == 0) {
		PERR("Soft pass thru reg read timed out");
		return -EIO;
	}

	return 0;
}
#endif /*~ BHY_RESERVE_FOR_LATER_USE */

static int bmi160_read_reg(struct bhy_client_data *client_data,
	u8 reg, u8 *data, u16 len)
{
	if (client_data == NULL)
		return -EIO;
	return bhy_soft_pass_thru_read_reg(client_data, BHY_SLAVE_ADDR_BMI160,
		reg, data, len);
}

static int bmi160_write_reg(struct bhy_client_data *client_data,
	u8 reg, u8 *data, u16 len)
{
	if (client_data == NULL)
		return -EIO;
	return bhy_soft_pass_thru_write_reg_m(client_data,
		BHY_SLAVE_ADDR_BMI160, reg, data, len);
}

static int bma2x2_read_reg(struct bhy_client_data *client_data,
	u8 reg, u8 *data, u16 len)
{
	if (client_data == NULL)
		return -EIO;
	return bhy_soft_pass_thru_read_reg(client_data, BHY_SLAVE_ADDR_BMA2X2,
		reg, data, len);
}

static int bma2x2_write_reg(struct bhy_client_data *client_data,
	u8 reg, u8 *data, u16 len)
{
	if (client_data == NULL)
		return -EIO;
	return bhy_soft_pass_thru_write_reg_m(client_data,
		BHY_SLAVE_ADDR_BMA2X2, reg, data, len);
}

static void bhy_get_ap_timestamp(s64 *ts_ap)
{
	struct timespec ts;
	get_monotonic_boottime(&ts);
	*ts_ap = ts.tv_sec;
	*ts_ap = *ts_ap * 1000000000 + ts.tv_nsec;
}

static void bhy_clear_flush_queue(struct flush_queue *q)
{
	q->head = q->tail = 0;
	q->cur = -1;
}

static void bhy_advance_flush_queue_position(int *pos)
{
	if (*pos == BHY_FLUSH_QUEUE_SIZE - 1)
		*pos = 0;
	else
		++*pos;
}

static void bhy_advance_n_flush_queue_position(int *pos, int n)
{
	if (*pos + n >= BHY_FLUSH_QUEUE_SIZE)
		*pos = (*pos + n) % BHY_FLUSH_QUEUE_SIZE;
	else if (*pos + n < 0)
		*pos = (*pos + n) % BHY_FLUSH_QUEUE_SIZE + BHY_FLUSH_QUEUE_SIZE;
	else
		*pos += n;
}

static int bhy_hardware_flush(struct bhy_client_data *client_data, u8 sel)
{
	int ret = 0;
	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	if (sel <= 0 || (sel > BHY_SENSOR_HANDLE_MAX
		&& sel != BHY_FLUSH_DISCARD_ALL
		&& sel != BHY_FLUSH_FLUSH_ALL)) {
		PERR("Invalid sensor sel for flush: %d", sel);
		return -EINVAL;
	}
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_reg(client_data, BHY_REG_FIFO_FLUSH, &sel, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write flush sensor reg error");
		return ret;
	}
	return ret;
}

static int bhy_enqueue_flush(struct bhy_client_data *client_data, u8 sensor_sel)
{
	int ret = 0;
	struct flush_queue *q = NULL;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	q = &client_data->flush_queue;
	mutex_lock(&q->lock);
	if (q->cur == -1) {
		ret = bhy_hardware_flush(client_data, sensor_sel);
		if (ret < 0) {
			mutex_unlock(&q->lock);
			PERR("Write sensor flush failed");
			return ret;
		}
		q->cur = sensor_sel;
	} else {
		q->queue[q->head] = sensor_sel;
		q->head = q->head == BHY_FLUSH_QUEUE_SIZE - 1 ? 0 : q->head + 1;
		if (q->head == q->tail) {
			bhy_clear_flush_queue(q);
			mutex_unlock(&q->lock);
			PERR("Flush queue full!!!");
			return -EIO;
		}
	}
	mutex_unlock(&q->lock);

	return ret;
}

static void bhy_dequeue_flush(struct bhy_client_data *client_data,
	u8 sensor_sel, int *extra_event)
{
	struct flush_queue *q = NULL;
	int i, temp_index;

	*extra_event = 0;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return;
	}

	q = &client_data->flush_queue;
	mutex_lock(&q->lock);
	if (q->cur != sensor_sel) { /* Flush logic error */
		bhy_clear_flush_queue(q);
		mutex_unlock(&q->lock);
		PERR("Flush logic error: q->cur != sensor_sel");
		return;
	}

	if (q->head == q->tail) { /* Flush queue empty */
		q->cur = -1;
		mutex_unlock(&q->lock);
		return;
	}

	/* Remove & count all the same type sensor in queue */
	for (i = q->tail; i != q->head; bhy_advance_flush_queue_position(&i)) {
		if (q->queue[i] == q->cur) {
			++*extra_event;
			continue;
		}
		if (*extra_event > 0) {
			temp_index = i;
			bhy_advance_n_flush_queue_position(&temp_index,
				-*extra_event);
			q->queue[temp_index] = q->queue[i];
		}
	}
	if (*extra_event > 0)
		bhy_advance_n_flush_queue_position(&q->head, -*extra_event);
	if (q->head == q->tail) {
		q->cur = -1;
		mutex_unlock(&q->lock);
		return;
	}

	/* Pop tail (dequeue) */
	q->cur = q->queue[q->tail];
	q->tail = q->tail == BHY_FLUSH_QUEUE_SIZE - 1 ? 0 : q->tail + 1;

	if (q->cur == -1) {
		mutex_unlock(&q->lock);
		return;
	}

	if (bhy_hardware_flush(client_data, (u8)q->cur) < 0) {
		bhy_clear_flush_queue(q);
		mutex_unlock(&q->lock);
		PERR("Flush next sensor failed");
		return;
	}
	mutex_unlock(&q->lock);

	return;
}

static int bhy_check_chip_id(struct bhy_data_bus *data_bus)
{
	int ret;
	u8 prod_id;
	ret = data_bus->read(data_bus->dev, BHY_REG_PRODUCT_ID, &prod_id,
		sizeof(u8));
	printk("product id = 0X%2X.\n", prod_id);
	if (ret < 0) {
		ACC_PR_ERR("Read prod id failed");
		return ret;
	}
	switch (prod_id) {
	case BST_FPGA_PRODUCT_ID_7181:
		PINFO("BST FPGA 7181 detected");
		break;
	case BHY_C1_PRODUCT_ID:
		PINFO("BHy C1 sample detected");
		break;
	case BST_FPGA_PRODUCT_ID_7183:
		PINFO("BST FPGA 7183 detected");
		break;
	default:
		PERR("Unknown product ID: 0X%02X", prod_id);
		return -ENODEV;
	}
	return 0;
}

static int bhy_request_firmware(struct bhy_client_data *client_data)
{
	ssize_t ret;
	u8 u8_val;
	__le16 le16_val;
	__le32 le32_val;
	u16 u16_val;
	int retry = BHY_RESET_WAIT_RETRY;
	int reset_flag_copy;
	struct ram_patch_header header;
	struct  ram_patch_cds cds;
	u32 cds_offset;
	ssize_t read_len;
	char data_buf[BHY_DMA_MAX_ALLOCATE-32]; /* Must be less than burst write max buf */
	u16 remain;
	int i;
	const struct firmware *fw;
	size_t pos;
	u8 ref_rom_version = 0;

	mutex_lock(&client_data->mutex_bus_op);
	/* Reset BHy */
	reset_flag_copy = atomic_read(&client_data->reset_flag);
	if (reset_flag_copy != RESET_FLAG_READY) {
		atomic_set(&client_data->reset_flag, RESET_FLAG_TODO);
		u8_val = 1;
		ret = bhy_write_reg(client_data, BHY_REG_RESET_REQ, &u8_val,
			sizeof(u8));
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write reset reg failed");
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return ret;
		}
#ifdef BHY_LEVEL_TRIGGERED_IRQ_SUPPORT
		/* Enable IRQ for reset detection */
		if (client_data->irq_enabled == BHY_FALSE) {
			client_data->irq_enabled = BHY_TRUE;
			enable_irq(client_data->data_bus.irq);
		}
#endif /*~ BHY_LEVEL_TRIGGERED_IRQ_SUPPORT */
		while (retry--) {
			reset_flag_copy = RESET_FLAG_READY;//atomic_read(&client_data->reset_flag);
			if (reset_flag_copy == RESET_FLAG_READY)
				break;
			msleep(10);
		}
		if (retry <= 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Reset ready status wait failed");
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EIO;
		}
		printk("BHy reset successfully");
	}

	/* Check chip status */
	retry = 1000;
	while (retry--) {
		ret = bhy_read_reg(client_data, BHY_REG_CHIP_STATUS,
			&u8_val, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read chip status failed");
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EIO;
		}
		if (u8_val & BHY_CHIP_STATUS_BIT_FIRMWARE_IDLE)
			break;
		usleep_range(10000, 10000);
	}
	if (retry <= 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Chip status error after reset: %d", u8_val);
		atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
		return -EIO;
	}

	/* Init upload addr */
	le16_val = __cpu_to_le16(0);
	if (bhy_write_reg(client_data, BHY_REG_UPLOAD_ADDR_0,
		(u8 *)&le16_val, 2) < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Init upload addr failed");
		atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
		return -EIO;
	}
	/* Write upload request */
	u8_val = BHY_CHIP_CTRL_BIT_UPLOAD_ENABLE;
	if (bhy_write_reg(client_data, BHY_REG_CHIP_CTRL, &u8_val, 1) < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Set chip ctrl failed");
		atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
		return -EIO;
	}
	/* Request firmware data from filesystem*/
	ret = request_firmware(&fw, "ram_patch.fw", client_data->data_bus.dev);
	if (ret < 0) {
		printk("load firmware from flash failed");
		//use default ram patch buffer to bootup phone
		memcpy(&header, default_ram_patch, sizeof(header));       
		pos += sizeof(header);
		u16_val = le16_to_cpu(header.magic);
		if (u16_val != BHY_RAM_PATCH_HEADER_MAGIC) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Header magic mismatch: %d vs %d", u16_val,
				BHY_RAM_PATCH_HEADER_MAGIC);
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EINVAL;
		}
		u16_val = le16_to_cpu(header.flags);
		u16_val &= BHY_FLAG_EXP_ROM_VER_MASK;
		u16_val >>= BHY_FLAG_EXP_ROM_VER_SHIFT;
		ret = bhy_read_reg(client_data, BHY_REG_REV_ID, &ref_rom_version, 1);
		if (ret < 0) {
			PERR("Read product revision failed");
			mutex_unlock(&client_data->mutex_bus_op);
			return -EINVAL;
		}
		printk("installing ram match rom version: %d, expected rom version %d",
			u16_val, ref_rom_version);
		if (u16_val != ref_rom_version) {
			mutex_unlock(&client_data->mutex_bus_op);
			printk("Expected rom version mismatch: %d vs %d", u16_val,
				ref_rom_version);
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EINVAL;
		}
		remain = le16_to_cpu(header.data_length);
		if (remain % 4 != 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			printk("data length cannot be divided by 4");
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EINVAL;
		}
		if (sizeof(default_ram_patch) < (size_t)(sizeof(header) + remain)) {
			mutex_unlock(&client_data->mutex_bus_op);
			printk("firmware size error: %lld vs %lld", sizeof(default_ram_patch),
					(long long int)(sizeof(header) + remain + sizeof(cds)));
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EINVAL;
		}
		le32_val = *(__le32 *)(default_ram_patch + BHY_CDS_OFFSET_POS);
		cds_offset = le32_to_cpu(le32_val);
		/* Evan's proposal to distinguish old firmware */
		cds.sig = 0;
		if (cds_offset >= BHY_CDS_INVALID_OFFSET) {
			printk("Old firmware format detected.");
			memcpy(&cds, default_ram_patch + sizeof(default_ram_patch) - sizeof(cds), sizeof(cds));
		} else if (sizeof(default_ram_patch) < (size_t)(sizeof(header) +
			cds_offset + sizeof(cds))) {
			printk("cds_offset is invalid.\n");
		}
		else
			memcpy(&cds, default_ram_patch + sizeof(header)+cds_offset, sizeof(cds));
		u16_val = le16_to_cpu(cds.sig);

		if (u16_val != BHY_CDS_SIGNATURE)
			printk("CDS signature mismatch: %d vs %d", u16_val,
				BHY_CDS_SIGNATURE);
		else
			printk("Ram version read from patch is %d",
				le16_to_cpu(cds.ram_version));
		while (remain > 0) {
			read_len = remain > sizeof(data_buf) ? sizeof(data_buf) :
				remain;	

			memcpy(data_buf, default_ram_patch + pos, read_len);
			pos += read_len;

			for (i = 0; i < read_len; i += 4)
				*(u32 *)(data_buf + i) = swab32(*(u32 *)(data_buf + i));
			if (bhy_write_reg(client_data, BHY_REG_UPLOAD_DATA,
				(u8 *)data_buf, read_len) < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				printk("Write ram patch data failed");
				atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
				return -EIO;
			}
			remain -= read_len;
			}
	}
	else //load ram_patch.fw from filesystem successfully
	{
		pos = 0;
		printk("Firmware size is %u", (unsigned int)fw->size);
		/* Upload data */
		if (fw->size < sizeof(header)) {
			release_firmware(fw);
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("firmware size error");
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EINVAL;
		}
		memcpy(&header, fw->data, sizeof(header));
		pos += sizeof(header);
		u16_val = le16_to_cpu(header.magic);
		if (u16_val != BHY_RAM_PATCH_HEADER_MAGIC) {
			release_firmware(fw);
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Header magic mismatch: %d vs %d", u16_val,
				BHY_RAM_PATCH_HEADER_MAGIC);
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EINVAL;
		}
		u16_val = le16_to_cpu(header.flags);
		u16_val &= BHY_FLAG_EXP_ROM_VER_MASK;
		u16_val >>= BHY_FLAG_EXP_ROM_VER_SHIFT;
		ret = bhy_read_reg(client_data, BHY_REG_REV_ID, &ref_rom_version, 1);
		if (ret < 0) {
			PERR("Read product revision failed");
			mutex_unlock(&client_data->mutex_bus_op);
			return -EINVAL;
		}
		printk("installing ram match rom version: %d, expected rom version %d",
			u16_val, ref_rom_version);

		if (u16_val != ref_rom_version) {
			release_firmware(fw);
			mutex_unlock(&client_data->mutex_bus_op);
			printk("Expected rom version mismatch: %d vs %d", u16_val,
				ref_rom_version);
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EINVAL;
		}
		remain = le16_to_cpu(header.data_length);
		if (remain % 4 != 0) {
			release_firmware(fw);
			mutex_unlock(&client_data->mutex_bus_op);
			printk("data length cannot be divided by 4");
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EINVAL;
		}
		if (fw->size < (size_t)(sizeof(header) + remain)) {
			release_firmware(fw);
			mutex_unlock(&client_data->mutex_bus_op);
			printk("firmware size error: %lld vs %lld", (long long int)fw->size,
					(long long int)(sizeof(header) + remain + sizeof(cds)));
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EINVAL;
		}
		le32_val = *(__le32 *)(fw->data + BHY_CDS_OFFSET_POS);
		cds_offset = le32_to_cpu(le32_val);
		/* Evan's proposal to distinguish old firmware */
		cds.sig = 0;
		if (cds_offset >= BHY_CDS_INVALID_OFFSET) {
			printk("Old firmware format detected.");
			memcpy(&cds, fw->data + fw->size - sizeof(cds), sizeof(cds));
		} else if (fw->size < (size_t)(sizeof(header) +
			cds_offset + sizeof(cds))) {
			printk("cds_offset is invalid.\n");
			/* PWARN("cds_offset is invalid: %d vs %d", fw->size,
				sizeof(header) + cds_offset + sizeof(cds)); */
		}
		else
			memcpy(&cds, fw->data + sizeof(header)+cds_offset, sizeof(cds));
		u16_val = le16_to_cpu(cds.sig);

		if (u16_val != BHY_CDS_SIGNATURE)
			printk("CDS signature mismatch: %d vs %d", u16_val,
				BHY_CDS_SIGNATURE);
		else
			printk("Ram version read from patch is %d",
				le16_to_cpu(cds.ram_version));

		struct timespec ts;
	    ts = current_kernel_time();
		printk("begin_uploading ram_patch %ld %ld\n", ts.tv_sec, ts.tv_nsec);
		while (remain > 0) {
			read_len = remain > sizeof(data_buf) ? sizeof(data_buf) :
				remain;	

			memcpy(data_buf, fw->data + pos, read_len);
			pos += read_len;
		
			for (i = 0; i < read_len; i += 4)
				*(u32 *)(data_buf + i) = swab32(*(u32 *)(data_buf + i));
			if (bhy_write_reg(client_data, BHY_REG_UPLOAD_DATA,
				(u8 *)data_buf, read_len) < 0) {
				release_firmware(fw);
				mutex_unlock(&client_data->mutex_bus_op);
				printk("Write ram patch data failed");
				atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
				return -EIO;
			}
			remain -= read_len;
		}
			
	    ts = current_kernel_time();
	    printk("done uploading ram_patch %ld %ld\n", ts.tv_sec, ts.tv_nsec);
		/* Release firmware */
		release_firmware(fw);
	}
	/* Check CRC */
	if (bhy_read_reg(client_data, BHY_REG_DATA_CRC_0,
		(u8 *)&le32_val, 4) < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read CRC failed");
		atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
		return -EIO;
	}
	if (le32_val != header.crc) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("CRC mismatch 0X%08X vs 0X%08X", le32_to_cpu(le32_val),
			le32_to_cpu(header.crc));
		atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
		return -EIO;
	}
	/* Clear upload mode bit */
	u8_val = 0;
	if (bhy_write_reg(client_data, BHY_REG_CHIP_CTRL, &u8_val, 1) < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write chip ctrl reg failed");
		atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
		return -EIO;
	}
	udelay(50);
	/* Check chip status */
	retry = 1000;
	while (retry--) {
		ret = bhy_read_reg(client_data, BHY_REG_CHIP_STATUS,
			&u8_val, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read chip status failed");
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EIO;
		}
		if (u8_val & BHY_CHIP_STATUS_BIT_FIRMWARE_IDLE)
			break;
		usleep_range(10000, 10000);
	}
	if (retry <= 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Chip status error after upload patch");
		atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
		return -EIO;
	}
	/* Enable cpu run */
	u8_val = BHY_CHIP_CTRL_BIT_CPU_RUN_REQ;
	if (bhy_write_reg(client_data, BHY_REG_CHIP_CTRL, &u8_val, 1) < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write chip ctrl reg failed #2");
		atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
		return -EIO;
	}

	/* Check chip status */
	retry = 1000;
	while (retry--) {
		ret = bhy_read_reg(client_data, BHY_REG_CHIP_STATUS,
			&u8_val, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read chip status failed");
			atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
			return -EIO;
		}
		if (!(u8_val & BHY_CHIP_STATUS_BIT_FIRMWARE_IDLE))
			break;
		usleep_range(10000, 10000);
	}
	if (retry <= 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Chip status error after CPU run request");
		atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
		return -EIO;
	}

	mutex_unlock(&client_data->mutex_bus_op);
	printk("Ram patch loaded successfully.");

#ifdef BHY_LEVEL_TRIGGERED_IRQ_SUPPORT
	/* Enable IRQ to read data after ram patch loaded */
	if (client_data->irq_enabled == BHY_FALSE) {
		client_data->irq_enabled = BHY_TRUE;
		enable_irq(client_data->data_bus.irq);
	}
#endif /*~ BHY_LEVEL_TRIGGERED_IRQ_SUPPORT */

	return 0;
}

static int bhy_reinit(struct bhy_client_data *client_data)
{
	int retry;
	u8 reg_data;
	int ret;

	PDEBUG("Reinit after self-test");
	mutex_lock(&client_data->mutex_bus_op);

	reg_data = 0;
	ret = bhy_write_reg(client_data, BHY_REG_CHIP_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write chip control reg failed");
		return -EIO;
	}
	retry = 1000;
	do {
		ret = bhy_read_reg(client_data, BHY_REG_CHIP_STATUS,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read chip status failed");
			return -EIO;
		}
		if (reg_data & BHY_CHIP_STATUS_BIT_FIRMWARE_IDLE)
			break;
		usleep_range(10000, 10000);
	} while (--retry);
	if (retry == 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Wait for chip idle status timed out");
		return -EBUSY;
	}
	/* Clear self test bit */
	ret = bhy_read_reg(client_data, BHY_REG_HOST_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read host ctrl reg failed");
		return -EIO;
	}
	reg_data &= ~HOST_CTRL_MASK_SELF_TEST_REQ;
	ret = bhy_write_reg(client_data, BHY_REG_HOST_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write host ctrl reg failed");
		return -EIO;
	}
	/* Enable CPU run from chip control */
	reg_data = 1;
	ret = bhy_write_reg(client_data, BHY_REG_CHIP_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write chip control reg failed");
		return -EIO;
	}
	retry = 1000;
	do {
		ret = bhy_read_reg(client_data, BHY_REG_CHIP_STATUS,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read chip status failed");
			return -EIO;
		}
		if (!(reg_data & BHY_CHIP_STATUS_BIT_FIRMWARE_IDLE))
			break;
		usleep_range(10000, 10000);
	} while (--retry);
	if (retry == 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Wait for chip running status timed out");
		return -EBUSY;
	}

	mutex_unlock(&client_data->mutex_bus_op);

	return 0;
}

static int bhy_get_sensor_conf(struct bhy_client_data *client_data,
	int handle, u8 *conf)
{
	int i;
	__le16 swap_data;
	u8 data[8];
	int ret;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_SENSOR,
		BHY_PARAM_SENSOR_CONF_0 + client_data->sensor_sel,
		data, 8);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read parameter error");
		return ret;
	}

	for (i = 0; i < 4; ++i) {
		swap_data = cpu_to_le16(*(u16 *)(data + i * 2));
		memcpy(conf + i * 2, &swap_data, sizeof(swap_data));
	}

	return 8;
}

static int bhy_set_sensor_conf(struct bhy_client_data *client_data,
	int handle, const u8 *conf)
{
	int i;
	__le16 swap_data;
	u8 data[8];
	int ret;
	for (i = 0; i < 4; ++i) {
		swap_data = cpu_to_le16(*(u16 *)(conf + i * 2));
		memcpy(data + i * 2, &swap_data, sizeof(swap_data));
	}
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_SENSOR,
		BHY_PARAM_SENSOR_CONF_0 + handle,
		(u8 *)data, 8);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		printk("Write parameter error");
		return ret;
	}
	printk("Set sensor[%d] conf: %02X %02X %02X %02X %02X %02X %02X %02X",
		handle, data[0], data[1], data[2], data[3], data[4], data[5],
		data[6], data[7]);

	return 0;
}

static int bhy_set_mapping_matrix(struct bhy_client_data *client_data,
	int index)
{
	u8 data[8] = { 0, };
	int i;
	struct physical_sensor_context *pct;
	int handle;
	int ret;

	switch (index) {
	case PHYSICAL_SENSOR_INDEX_ACC:
		handle = BHY_SENSOR_HANDLE_ACCELEROMETER;
		break;
	case PHYSICAL_SENSOR_INDEX_MAG:
		handle = BHY_SENSOR_HANDLE_MAGNETIC_FIELD_UNCALIBRATED;
		break;
	case PHYSICAL_SENSOR_INDEX_GYRO:
		handle = BHY_SENSOR_HANDLE_GYROSCOPE_UNCALIBRATED;
		break;
	default:
		return -EINVAL;
	}

	pct = &client_data->ps_context[index];
	for (i = 0; i < 5; ++i) {
		switch (pct->mapping_matrix[2 * i]) {
		case 0:
			data[i] = 0;
			break;
		case 1:
			data[i] = 1;
			break;
		case -1:
			data[i] = 0xF;
			break;
		default:
			return -EINVAL;
		}
		if (i == 4)
			break;
		switch (pct->mapping_matrix[2 * i + 1]) {
		case 0:
			break;
		case 1:
			data[i] |= 0x10;
			break;
		case -1:
			data[i] |= 0xF0;
			break;
		default:
			return -EINVAL;
		}
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_SYSTEM,
		BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_DETAIL_0 + handle,
		data, sizeof(data));
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write mapping matrix failed");
		return ret;
	}

	return 0;
}

static int bhy_set_meta_event_ctrl(struct bhy_client_data *client_data,
	int type, int for_wake_up, int event_en, int irq_en)
{
	int num, bit;
	u8 param;
	u8 data[8];
	int ret;

	/*get the num for META_EVENT_CONTORL_BYTE*/
	num = (type - 1) / 4;
	/*get the bit for META_EVENT_CONTROL_BYTE*/
	bit = (type - 1) % 4;
	param = for_wake_up == BHY_TRUE ?
		BHY_PARAM_SYSTEM_WAKE_UP_META_EVENT_CTRL :
		BHY_PARAM_SYSTEM_META_EVENT_CTRL;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_SYSTEM, param,
		data, 8);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read meta event failed");
		return -EIO;
	}

	if (event_en == BHY_TRUE)
		data[num] |= (1 << (bit * 2 + 1));
	else
		data[num] &= ~(1 << (bit * 2 + 1));
	if (irq_en == BHY_TRUE)
		data[num] |= (1 << (bit * 2));
	else
		data[num] &= ~(1 << (bit * 2));
	ret = bhy_write_parameter(client_data, BHY_PAGE_SYSTEM, param,
		data, 8);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write meta event ctrl failed");
		return -EIO;
	}
	mutex_unlock(&client_data->mutex_bus_op);

	return 0;
}

static int bhy_set_fifo_ctrl(struct bhy_client_data *client_data, u8 *buf)
{
	int ret;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_SYSTEM,
		BHY_PARAM_SYSTEM_FIFO_CTRL, buf, BHY_FIFO_CTRL_PARAM_LEN);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write fifo ctrl failed");
		return -EIO;
	}

	return ret;
}

static int bhy_set_calib_profile(struct bhy_client_data *client_data,
	const u8 *buf)
{
	int ret;
	u8 param_num;

	mutex_lock(&client_data->mutex_bus_op);
#ifdef BHY_CALIB_PROFILE_OP_IN_FUSER_CORE
	switch (client_data->sensor_sel) {
	case BHY_SENSOR_HANDLE_ACCELEROMETER:
		param_num = BHY_PARAM_OFFSET_ACC_2;
		break;
	case BHY_SENSOR_HANDLE_GEOMAGNETIC_FIELD:
		param_num = BHY_PARAM_OFFSET_MAG_2;
		break;
	case BHY_SENSOR_HANDLE_GYROSCOPE:
		param_num = BHY_PARAM_OFFSET_GYRO_2;
		break;
	default:
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Invalid sensor sel");
		return -EINVAL;
	}
#else
	switch (client_data->sensor_sel) {
	case BHY_SENSOR_HANDLE_ACCELEROMETER:
		param_num = BHY_PARAM_OFFSET_ACC;
		break;
	case BHY_SENSOR_HANDLE_GEOMAGNETIC_FIELD:
		param_num = BHY_PARAM_OFFSET_MAG;
		break;
	case BHY_SENSOR_HANDLE_GYROSCOPE:
		param_num = BHY_PARAM_OFFSET_GYRO;
		break;
	default:
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Invalid sensor sel");
		return -EINVAL;
	}
#endif /*~ BHY_CALIB_PROFILE_OP_IN_FUSER_CORE */
	ret = bhy_write_parameter(client_data, BHY_PAGE_ALGORITHM,
		param_num, (u8 *)buf, 8);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write parameter error");
		return ret;
	}

	return 0;
}

/* Returns whether hw watchdog was detected */
static int check_watchdog_reset(struct bhy_client_data *client_data)
{
	int ret;
	u8 reg_data;
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, BHY_REG_HOST_STATUS,
		&reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read host status failed");
		return BHY_FALSE;
	}
	if (!(reg_data & BHY_HOST_STATUS_MASK_RESET)) {
		mutex_unlock(&client_data->mutex_bus_op);
		PDEBUG("Host status is still good");
		return BHY_FALSE;
	}
	ret = bhy_read_reg(client_data, BHY_REG_CHIP_CTRL,
		&reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read chip control failed");
		return BHY_FALSE;
	}
	if (reg_data & BHY_CHIP_CTRL_BIT_CPU_RUN_REQ) {
		mutex_unlock(&client_data->mutex_bus_op);
		PDEBUG("Chip control indicates CPU still running");
		return BHY_FALSE;
	}
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("Hardware watch dog bark!!!");
	atomic_set(&client_data->reset_flag, RESET_FLAG_READY);
	bhy_request_firmware(client_data);
	client_data->recover_from_disaster = BHY_TRUE;

	return BHY_TRUE;
}

#ifdef BHY_CHECK_MCU_INCONSISTENCY
/* Check if irq status/bytes remaining register update inconsistency happened.
   returns BHY_TRUE if this inconsistency happened and abort stucked data */
static int check_mcu_inconsistency(struct bhy_client_data *client_data)
{
	int ret;
	u8 reg_data;
	char dump[256 * 3];
	mutex_lock(&client_data->mutex_bus_op);
	/* Check IRQ status */
	ret = bhy_read_reg(client_data, BHY_REG_INT_STATUS, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read irq status failed");
		return BHY_FALSE;
	}
	if (reg_data == 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		return BHY_FALSE;
	}
	bhy_dump_registers(client_data, dump, "BHY");
	PDEBUG("%s", dump);
	reg_data = HOST_CTRL_MASK_ABORT_TRANSFER;
	ret = bhy_write_reg(client_data, BHY_REG_HOST_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write host control failed");
		return BHY_FALSE;
	}
	msleep(10);
	reg_data = 0; /* Clear host control reg */
	ret = bhy_write_reg(client_data, BHY_REG_HOST_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write host control failed");
		return BHY_FALSE;
	}
	mutex_unlock(&client_data->mutex_bus_op);
	return BHY_TRUE;
}
#endif /*~ BHY_CHECK_MCU_INCONSISTENCY */

static void bhy_init_sensor_context(struct bhy_client_data *client_data)
{
	int i;
	struct bhy_sensor_context *ct;

	ct = client_data->sensor_context;
	for (i = 1; i <= BHY_SENSOR_HANDLE_REAL_MAX; ++i) {
		ct[i].handlle = i;
		ct[i].data_len = -1;
		ct[i].type = SENSOR_TYPE_INVALID;
		ct[i].is_wakeup = BHY_FALSE;
		ct[i].sample_rate = 0;
		ct[i].report_latency = 0;
#ifdef BHY_AR_HAL_SUPPORT
		ct[i].for_ar_hal = BHY_FALSE;
#endif /*~ BHY_AR_HAL_SUPPORT */
	}
	for (i = BHY_SENSOR_HANDLE_WAKEUP_BEGIN; i <= BHY_SENSOR_HANDLE_MAX;
		++i)
		ct[i].is_wakeup = BHY_TRUE;
	ct[BHY_SENSOR_HANDLE_ACCELEROMETER].data_len =
		BHY_SENSOR_DATA_LEN_ACCELEROMETER;
	ct[BHY_SENSOR_HANDLE_GEOMAGNETIC_FIELD].data_len =
		BHY_SENSOR_DATA_LEN_GEOMAGNETIC_FIELD;
	ct[BHY_SENSOR_HANDLE_ORIENTATION].data_len =
		BHY_SENSOR_DATA_LEN_ORIENTATION;
	ct[BHY_SENSOR_HANDLE_GYROSCOPE].data_len =
		BHY_SENSOR_DATA_LEN_GYROSCOPE;
	ct[BHY_SENSOR_HANDLE_LIGHT].data_len =
		BHY_SENSOR_DATA_LEN_LIGHT;
	ct[BHY_SENSOR_HANDLE_PRESSURE].data_len =
		BHY_SENSOR_DATA_LEN_PRESSURE;
	ct[BHY_SENSOR_HANDLE_TEMPERATURE].data_len =
		BHY_SENSOR_DATA_LEN_TEMPERATURE;
	ct[BHY_SENSOR_HANDLE_PROXIMITY].data_len =
		BHY_SENSOR_DATA_LEN_PROXIMITY;
	ct[BHY_SENSOR_HANDLE_GRAVITY].data_len =
		BHY_SENSOR_DATA_LEN_GRAVITY;
	ct[BHY_SENSOR_HANDLE_LINEAR_ACCELERATION].data_len =
		BHY_SENSOR_DATA_LEN_LINEAR_ACCELERATION;
	ct[BHY_SENSOR_HANDLE_ROTATION_VECTOR].data_len =
		BHY_SENSOR_DATA_LEN_ROTATION_VECTOR;
	ct[BHY_SENSOR_HANDLE_RELATIVE_HUMIDITY].data_len =
		BHY_SENSOR_DATA_LEN_RELATIVE_HUMIDITY;
	ct[BHY_SENSOR_HANDLE_AMBIENT_TEMPERATURE].data_len =
		BHY_SENSOR_DATA_LEN_AMBIENT_TEMPERATURE;
	ct[BHY_SENSOR_HANDLE_MAGNETIC_FIELD_UNCALIBRATED].data_len =
		BHY_SENSOR_DATA_LEN_MAGNETIC_FIELD_UNCALIBRATED;
	ct[BHY_SENSOR_HANDLE_GAME_ROTATION_VECTOR].data_len =
		BHY_SENSOR_DATA_LEN_GAME_ROTATION_VECTOR;
	ct[BHY_SENSOR_HANDLE_GYROSCOPE_UNCALIBRATED].data_len =
		BHY_SENSOR_DATA_LEN_GYROSCOPE_UNCALIBRATED;
	ct[BHY_SENSOR_HANDLE_SIGNIFICANT_MOTION].data_len =
		BHY_SENSOR_DATA_LEN_SIGNIFICANT_MOTION;
	ct[BHY_SENSOR_HANDLE_STEP_DETECTOR].data_len =
		BHY_SENSOR_DATA_LEN_STEP_DETECTOR;
	ct[BHY_SENSOR_HANDLE_STEP_COUNTER].data_len =
		BHY_SENSOR_DATA_LEN_STEP_COUNTER;
	ct[BHY_SENSOR_HANDLE_GEOMAGNETIC_ROTATION_VECTOR].data_len =
		BHY_SENSOR_DATA_LEN_GEOMAGNETIC_ROTATION_VECTOR;
	ct[BHY_SENSOR_HANDLE_HEART_RATE].data_len =
		BHY_SENSOR_DATA_LEN_HEART_RATE;
	ct[BHY_SENSOR_HANDLE_ACCELEROMETER_WU].data_len =
		BHY_SENSOR_DATA_LEN_ACCELEROMETER_WU;
	ct[BHY_SENSOR_HANDLE_GEOMAGNETIC_FIELD_WU].data_len =
		BHY_SENSOR_DATA_LEN_GEOMAGNETIC_FIELD_WU;
	ct[BHY_SENSOR_HANDLE_ORIENTATION_WU].data_len =
		BHY_SENSOR_DATA_LEN_ORIENTATION_WU;
	ct[BHY_SENSOR_HANDLE_GYROSCOPE_WU].data_len =
		BHY_SENSOR_DATA_LEN_GYROSCOPE_WU;
	ct[BHY_SENSOR_HANDLE_LIGHT_WU].data_len =
		BHY_SENSOR_DATA_LEN_LIGHT_WU;
	ct[BHY_SENSOR_HANDLE_PRESSURE_WU].data_len =
		BHY_SENSOR_DATA_LEN_PRESSURE_WU;
	ct[BHY_SENSOR_HANDLE_TEMPERATURE_WU].data_len =
		BHY_SENSOR_DATA_LEN_TEMPERATURE_WU;
	ct[BHY_SENSOR_HANDLE_PROXIMITY_WU].data_len =
		BHY_SENSOR_DATA_LEN_PROXIMITY_WU;
	ct[BHY_SENSOR_HANDLE_GRAVITY_WU].data_len =
		BHY_SENSOR_DATA_LEN_GRAVITY_WU;
	ct[BHY_SENSOR_HANDLE_LINEAR_ACCELERATION_WU].data_len =
		BHY_SENSOR_DATA_LEN_LINEAR_ACCELERATION_WU;
	ct[BHY_SENSOR_HANDLE_ROTATION_VECTOR_WU].data_len =
		BHY_SENSOR_DATA_LEN_ROTATION_VECTOR_WU;
	ct[BHY_SENSOR_HANDLE_RELATIVE_HUMIDITY_WU].data_len =
		BHY_SENSOR_DATA_LEN_RELATIVE_HUMIDITY_WU;
	ct[BHY_SENSOR_HANDLE_AMBIENT_TEMPERATURE_WU].data_len =
		BHY_SENSOR_DATA_LEN_AMBIENT_TEMPERATURE_WU;
	ct[BHY_SENSOR_HANDLE_MAGNETIC_FIELD_UNCALIBRATED_WU].data_len =
		BHY_SENSOR_DATA_LEN_MAGNETIC_FIELD_UNCALIBRATED_WU;
	ct[BHY_SENSOR_HANDLE_GAME_ROTATION_VECTOR_WU].data_len =
		BHY_SENSOR_DATA_LEN_GAME_ROTATION_VECTOR_WU;
	ct[BHY_SENSOR_HANDLE_GYROSCOPE_UNCALIBRATED_WU].data_len =
		BHY_SENSOR_DATA_LEN_GYROSCOPE_UNCALIBRATED_WU;
	ct[BHY_SENSOR_HANDLE_STEP_DETECTOR_WU].data_len =
		BHY_SENSOR_DATA_LEN_STEP_DETECTOR_WU;
	ct[BHY_SENSOR_HANDLE_STEP_COUNTER_WU].data_len =
		BHY_SENSOR_DATA_LEN_STEP_COUNTER_WU;
	ct[BHY_SENSOR_HANDLE_GEOMAGNETIC_ROTATION_VECTOR_WU].data_len =
		BHY_SENSOR_DATA_LEN_GEOMAGNETIC_ROTATION_VECTOR_WU;
	ct[BHY_SENSOR_HANDLE_HEART_RATE_WU].data_len =
		BHY_SENSOR_DATA_LEN_HEART_RATE_WU;
	ct[BHY_SENSOR_HANDLE_TILT_DETECTOR].data_len =
		BHY_SENSOR_DATA_LEN_TILT_DETECTOR;
	ct[BHY_SENSOR_HANDLE_WAKE_GESTURE].data_len =
		BHY_SENSOR_DATA_LEN_WAKE_GESTURE;
	ct[BHY_SENSOR_HANDLE_GLANCE_GESTURE].data_len =
		BHY_SENSOR_DATA_LEN_GLANCE_GESTURE;
	ct[BHY_SENSOR_HANDLE_PICK_UP_GESTURE].data_len =
		BHY_SENSOR_DATA_LEN_PICK_UP_GESTURE;
	ct[BHY_SENSOR_HANDLE_ACTIVITY_RECOGNITION].data_len =
		BHY_SENSOR_DATA_LEN_ACTIVITY_RECOGNITION;
	ct[BHY_SENSOR_HANDLE_BSX_C].data_len =
		BHY_SENSOR_DATA_LEN_BSX_C;
	ct[BHY_SENSOR_HANDLE_BSX_B].data_len =
		BHY_SENSOR_DATA_LEN_BSX_B;
	ct[BHY_SENSOR_HANDLE_BSX_A].data_len =
		BHY_SENSOR_DATA_LEN_BSX_A;
	ct[BHY_SENSOR_HANDLE_TIMESTAMP_LSW].data_len =
		BHY_SENSOR_DATA_LEN_TIMESTAMP_LSW;
	ct[BHY_SENSOR_HANDLE_TIMESTAMP_MSW].data_len =
		BHY_SENSOR_DATA_LEN_TIMESTAMP_MSW;
	ct[BHY_SENSOR_HANDLE_META_EVENT].data_len =
		BHY_SENSOR_DATA_LEN_META_EVENT;
	ct[BHY_SENSOR_HANDLE_TIMESTAMP_LSW_WU].data_len =
		BHY_SENSOR_DATA_LEN_TIMESTAMP_LSW_WU;
	ct[BHY_SENSOR_HANDLE_TIMESTAMP_MSW_WU].data_len =
		BHY_SENSOR_DATA_LEN_TIMESTAMP_MSW_WU;
	ct[BHY_SENSOR_HANDLE_META_EVENT_WU].data_len =
		BHY_SENSOR_DATA_LEN_META_EVENT_WU;
	ct[BHY_SENSOR_HANDLE_DEBUG].data_len =
		BHY_SENSOR_DATA_LEN_DEBUG;
	ct[BHY_SENSOR_HANDLE_CUSTOM_1].data_len =
		BHY_SENSOR_DATA_LEN_CUSTOM_1;
	ct[BHY_SENSOR_HANDLE_CUSTOM_2].data_len =
		BHY_SENSOR_DATA_LEN_CUSTOM_2;
	ct[BHY_SENSOR_HANDLE_CUSTOM_3].data_len =
		BHY_SENSOR_DATA_LEN_CUSTOM_3;
	ct[BHY_SENSOR_HANDLE_CUSTOM_4].data_len =
		BHY_SENSOR_DATA_LEN_CUSTOM_4;
	ct[BHY_SENSOR_HANDLE_CUSTOM_5].data_len =
		BHY_SENSOR_DATA_LEN_CUSTOM_5;
	ct[BHY_SENSOR_HANDLE_CUSTOM_1_WU].data_len =
		BHY_SENSOR_DATA_LEN_CUSTOM_1_WU;
	ct[BHY_SENSOR_HANDLE_CUSTOM_2_WU].data_len =
		BHY_SENSOR_DATA_LEN_CUSTOM_2_WU;
	ct[BHY_SENSOR_HANDLE_CUSTOM_3_WU].data_len =
		BHY_SENSOR_DATA_LEN_CUSTOM_3_WU;
	ct[BHY_SENSOR_HANDLE_CUSTOM_4_WU].data_len =
		BHY_SENSOR_DATA_LEN_CUSTOM_4_WU;
	ct[BHY_SENSOR_HANDLE_CUSTOM_5_WU].data_len =
		BHY_SENSOR_DATA_LEN_CUSTOM_5_WU;

	ct[BHY_SENSOR_HANDLE_ACCELEROMETER].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_GEOMAGNETIC_FIELD].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_ORIENTATION].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_GYROSCOPE].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_LIGHT].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_PRESSURE].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_TEMPERATURE].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_PROXIMITY].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_GRAVITY].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_LINEAR_ACCELERATION].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_ROTATION_VECTOR].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_RELATIVE_HUMIDITY].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_AMBIENT_TEMPERATURE].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_MAGNETIC_FIELD_UNCALIBRATED].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_GAME_ROTATION_VECTOR].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_GYROSCOPE_UNCALIBRATED].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_SIGNIFICANT_MOTION].type =
		SENSOR_TYPE_ONE_SHOT;
	ct[BHY_SENSOR_HANDLE_STEP_DETECTOR].type =
		SENSOR_TYPE_SPECIAL;
	ct[BHY_SENSOR_HANDLE_STEP_COUNTER].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_GEOMAGNETIC_ROTATION_VECTOR].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_HEART_RATE].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_ACCELEROMETER_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_GEOMAGNETIC_FIELD_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_ORIENTATION_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_GYROSCOPE_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_LIGHT_WU].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_PRESSURE_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_TEMPERATURE_WU].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_PROXIMITY_WU].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_GRAVITY_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_LINEAR_ACCELERATION_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_ROTATION_VECTOR_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_RELATIVE_HUMIDITY_WU].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_AMBIENT_TEMPERATURE_WU].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_MAGNETIC_FIELD_UNCALIBRATED_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_GAME_ROTATION_VECTOR_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_GYROSCOPE_UNCALIBRATED_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_STEP_DETECTOR_WU].type =
		SENSOR_TYPE_SPECIAL;
	ct[BHY_SENSOR_HANDLE_STEP_COUNTER_WU].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_GEOMAGNETIC_ROTATION_VECTOR_WU].type =
		SENSOR_TYPE_CONTINUOUS;
	ct[BHY_SENSOR_HANDLE_HEART_RATE_WU].type =
		SENSOR_TYPE_ON_CHANGE;
	ct[BHY_SENSOR_HANDLE_TILT_DETECTOR].type =
		SENSOR_TYPE_SPECIAL;
	ct[BHY_SENSOR_HANDLE_WAKE_GESTURE].type =
		SENSOR_TYPE_ONE_SHOT;
	ct[BHY_SENSOR_HANDLE_GLANCE_GESTURE].type =
		SENSOR_TYPE_ONE_SHOT;
	ct[BHY_SENSOR_HANDLE_PICK_UP_GESTURE].type =
		SENSOR_TYPE_ONE_SHOT;
	ct[BHY_SENSOR_HANDLE_ACTIVITY_RECOGNITION].type =
		SENSOR_TYPE_ON_CHANGE;
#ifdef BHY_AR_HAL_SUPPORT
	ct[BHY_SENSOR_HANDLE_TIMESTAMP_LSW].for_ar_hal = BHY_TRUE;
	ct[BHY_SENSOR_HANDLE_TIMESTAMP_MSW].for_ar_hal = BHY_TRUE;
	ct[BHY_SENSOR_HANDLE_META_EVENT].for_ar_hal = BHY_TRUE;
	ct[BHY_SENSOR_HANDLE_TIMESTAMP_LSW_WU].for_ar_hal = BHY_TRUE;
	ct[BHY_SENSOR_HANDLE_TIMESTAMP_MSW_WU].for_ar_hal = BHY_TRUE;
	ct[BHY_SENSOR_HANDLE_META_EVENT_WU].for_ar_hal = BHY_TRUE;
	ct[BHY_SENSOR_HANDLE_ACTIVITY_RECOGNITION].for_ar_hal = BHY_TRUE;
#endif /*~ BHY_AR_HAL_SUPPORT */
}

#ifdef BHY_DEBUG
static void bhy_dump_fifo_data(const u8 *data, int len)
{
	int i, j;
	char buf[256];
	int line_char = 0;
	const int bytes_per_line = 8;
	printk(" Data is ");
	for (i = j = 0; i < len; ++i) {
		j += snprintf(buf + j, 16, "%02X ", *(data + i));
		if (++line_char == bytes_per_line) {
			buf[j - 1] = '\0';
			printk("%s", buf);
			line_char = 0;
			j = 0;
		}
	}
	if (line_char > 0) {
		buf[j - 1] = '\0';
		printk("%s", buf);
	}
}
#endif /*~ BHY_DEBUG */

static void bhy_recover_sensor_activity(struct bhy_client_data *client_data)
{
	int i;
	struct bhy_sensor_context *ct;
	u8 conf[8];
	struct bhy_meta_event_context *mct, *wmct;

	mutex_lock(&client_data->mutex_sw_watchdog);
	/* Recover fifo control settings */
	bhy_set_fifo_ctrl(client_data, client_data->fifo_ctrl_cfg);
	/* Recover step counter */
	client_data->step_count_base = client_data->step_count_latest;
	/* Recover axis mapping */
	for (i = 0; i < PHYSICAL_SENSOR_COUNT; ++i) {
		if (client_data->ps_context[i].use_mapping_matrix == BHY_TRUE)
			bhy_set_mapping_matrix(client_data, i);
	}
	/* Recover meta event */
	mct = client_data->me_context;
	wmct = client_data->mew_context;
	for (i = 1; i <= BHY_META_EVENT_MAX; ++i) {
		if (mct[i].event_en != BHY_STATUS_DEFAULT)
			bhy_set_meta_event_ctrl(client_data, i, BHY_FALSE,
			mct[i].event_en == BHY_STATUS_ENABLED ?
		BHY_TRUE : BHY_FALSE,
				   mct[i].irq_en == BHY_STATUS_ENABLED ?
			   BHY_TRUE : BHY_FALSE);
		if (wmct[i].event_en != BHY_STATUS_DEFAULT)
			bhy_set_meta_event_ctrl(client_data, i, BHY_TRUE,
			wmct[i].event_en == BHY_STATUS_ENABLED ?
		BHY_TRUE : BHY_FALSE,
				   wmct[i].irq_en == BHY_STATUS_ENABLED ?
			   BHY_TRUE : BHY_FALSE);
	}
	/* Recover calibration profile */
	mutex_lock(&client_data->mutex_bus_op);
	client_data->sensor_sel = BHY_SENSOR_HANDLE_ACCELEROMETER;
	mutex_unlock(&client_data->mutex_bus_op);
	bhy_set_calib_profile(client_data, client_data->calibprofile_acc);
	mutex_lock(&client_data->mutex_bus_op);
	client_data->sensor_sel = BHY_SENSOR_HANDLE_GEOMAGNETIC_FIELD;
	mutex_unlock(&client_data->mutex_bus_op);
	bhy_set_calib_profile(client_data, client_data->calibprofile_mag);
	mutex_lock(&client_data->mutex_bus_op);
	client_data->sensor_sel = BHY_SENSOR_HANDLE_GYROSCOPE;
	mutex_unlock(&client_data->mutex_bus_op);
	bhy_set_calib_profile(client_data, client_data->calibprofile_gyro);
	/* Recover sensor activity */
	ct = client_data->sensor_context;
	for (i = 1; i <= BHY_SENSOR_HANDLE_MAX; ++i) {
		if (ct[i].type != SENSOR_TYPE_INVALID &&
			ct[i].sample_rate > 0) {
			memcpy(conf, &ct[i].sample_rate, 2);
			memcpy(conf + 2, &ct[i].report_latency, 2);
			memset(conf + 4, 0, 4);
			bhy_set_sensor_conf(client_data, i, conf);
		}
	}
	mutex_unlock(&client_data->mutex_sw_watchdog);
}

static void bhy_advance_fifo_queue_position(int *pos)
{
	if (*pos == BHY_FRAME_SIZE - 1)
		*pos = 0;
	else
		++*pos;
}

static void bhy_advance_fifo_queue_head(struct frame_queue *q)
{
	bhy_advance_fifo_queue_position(&q->head);
	if (q->head == q->tail) {
		bhy_advance_fifo_queue_position(&q->tail);
	}
}

static void bhy_read_timestamp_sync(struct bhy_client_data *client_data)
{
	int ret;
	u8 timestamp_fw[4];
	struct frame_queue *q = &client_data->data_queue;
#ifdef BHY_AR_HAL_SUPPORT
	struct frame_queue *qa = &client_data->data_queue_ar;
#endif /*~ BHY_AR_HAL_SUPPORT */

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, BHY_REG_HOST_IRQ_TIMESTAMP_1,
		timestamp_fw, 4);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0)
		PERR("Get firmware timestamp failed");
	mutex_lock(&q->lock);
	q->frames[q->head].handle = BHY_SENSOR_HANDLE_TIMESTAMP_SYNC;
	memcpy(q->frames[q->head].data,
		&client_data->timestamp_irq, sizeof(u64));
	memcpy(q->frames[q->head].data + sizeof(u64),
		&timestamp_fw, sizeof(timestamp_fw));
#ifdef BHY_TS_LOGGING_SUPPORT
	++client_data->irq_count;
	memcpy(q->frames[q->head].data + sizeof(u64)+sizeof(timestamp_fw),
		&client_data->irq_count, sizeof(u32));
#endif /*~ BHY_TS_LOGGING_SUPPORT */
	bhy_advance_fifo_queue_head(q);
	mutex_unlock(&q->lock);
#ifdef BHY_AR_HAL_SUPPORT
	mutex_lock(&qa->lock);
	qa->frames[qa->head].handle = BHY_SENSOR_HANDLE_TIMESTAMP_SYNC;
	memcpy(qa->frames[qa->head].data,
		&client_data->timestamp_irq, sizeof(u64));
	memcpy(qa->frames[qa->head].data + sizeof(u64),
		&timestamp_fw, sizeof(timestamp_fw));
	if (qa->head == BHY_FRAME_SIZE_AR - 1)
		qa->head = 0;
	else
		++qa->head;
	if (qa->head == qa->tail) {
		if (qa->tail == BHY_FRAME_SIZE_AR - 1)
			qa->tail = 0;
		else
			++qa->tail;
	}
	mutex_unlock(&qa->lock);
#endif /*~ BHY_AR_HAL_SUPPORT */
}
#if 1//Ellen for remapping
static void bhy_get_mapping_value(int hw_postion, char* acc_remapping, char* gyro_remapping)
{
/*
P0: (0 1 0 -1 0 0 0 0 1); P1: (1 0 0 0 1 0 0 0 1); P2: (-1 0 0 0 -1 0 0 0 1); P3: (0 -1 0 1 0 0 0 0 1);
P4: (-1 0 0 0 1 0 0 0 -1); P5: (0 1 0 1 0 0 0 0 -1); P6: (0 -1 0 -1 0 0 0 0 -1); P7: (1 0 0 0 -1 0 0 0 -1).
*/
  char mapping_value[8][25] = {"0 1 0 -1 0 0 0 0 1","1 0 0 0 1 0 0 0 1","-1 0 0 0 -1 0 0 0 1",
                               "0 -1 0 1 0 0 0 0 1","-1 0 0 0 1 0 0 0 -1","0 1 0 1 0 0 0 0 -1",
                               "0 -1 0 -1 0 0 0 0 -1","1 0 0 0 -1 0 0 0 -1"};

  GSE_FUN();
  snprintf(acc_remapping, 30, "1 %s", mapping_value[hw_postion]);
  snprintf(gyro_remapping, 30, "16 %s", mapping_value[hw_postion]);

}
#endif
static void getRawDataFromQueue(struct bhy_client_data *client_data)
{

	u8 *pbufer = NULL;

	u16 handle = client_data->data_queue.frames[client_data->data_queue.head].handle;
	//ACC_LOG("getRawDataFromQueue handle = %d\n", handle);
	if (BHY_SENSOR_HANDLE_ACCELEROMETER == handle) {
		pbufer = client_data->data_queue.frames[client_data->data_queue.head].data;
		/* Convert sensor raw data to 16-bit integer */
		acc_data[BMI160_ACC_AXIS_X] = (s16) ((((s32)((s8)pbufer[1]))
				<< BMI160_SHIFT_8_POSITION) | (pbufer[0]));
		acc_data[BMI160_ACC_AXIS_Y] = (s16) ((((s32)((s8)pbufer[3]))
				<< BMI160_SHIFT_8_POSITION) | (pbufer[2]));
		acc_data[BMI160_ACC_AXIS_Z] = (s16) ((((s32)((s8)pbufer[5]))
				<< BMI160_SHIFT_8_POSITION) | (pbufer[4]));
/*		ACC_LOG("raw acc data: x = %d, y = %d, z = %d\n",
				acc_data[0], acc_data[1], acc_data[2]); */
	} 
	if(BHY_SENSOR_HANDLE_GYROSCOPE == handle) {
		pbufer = client_data->data_queue.frames[client_data->data_queue.head].data;
		/* Convert sensor raw data to 16-bit integer */
		gyro_data[BMI160_ACC_AXIS_X] = (s16) ((((s32)((s8)pbufer[1]))
				<< BMI160_SHIFT_8_POSITION) | (pbufer[0]));
		gyro_data[BMI160_ACC_AXIS_Y] = (s16) ((((s32)((s8)pbufer[3]))
				<< BMI160_SHIFT_8_POSITION) | (pbufer[2]));
		gyro_data[BMI160_ACC_AXIS_Z] = (s16) ((((s32)((s8)pbufer[5]))
				<< BMI160_SHIFT_8_POSITION) | (pbufer[4]));
/*		ACC_LOG("raw gyro data: x = %d, y = %d, z = %d\n",
				gyro_data[0], gyro_data[1], gyro_data[2]);*/
	}
	if(BHY_SENSOR_HANDLE_WAKE_GESTURE == handle) {
		ACC_LOG("WAKE_GESTURE data!\n");
		bhy_gesture_status = BHY_STATUS_ENABLE;
	}
#ifdef FW_DEBUG	
	if(BHY_SENSOR_HANDLE_DEBUG == handle) {
		printk("[FW_DEBUG]");
		pbufer = client_data->data_queue.frames[client_data->data_queue.head].data;
		printk("%s",pbufer+1);
	}
#endif
}

static void getCompensateAccData(struct bhy_client_data *obj, char *buffer)
{
	
	int acc[BMI160_ACC_AXES_NUM] = { 0 };

#if 0 /*bhy is sensor hub, so remapping/calibration had been done inside sensorhub*/
	/* compensate data */
	acc_data[BMI160_ACC_AXIS_X] += obj->cali_sw[BMI160_ACC_AXIS_X];
	acc_data[BMI160_ACC_AXIS_Y] += obj->cali_sw[BMI160_ACC_AXIS_Y];
	acc_data[BMI160_ACC_AXIS_Z] += obj->cali_sw[BMI160_ACC_AXIS_Z];
	/* remap coordinate */
	acc[obj->cvt.map[BMI160_ACC_AXIS_X]] =
			obj->cvt.sign[BMI160_ACC_AXIS_X] * acc_data[BMI160_ACC_AXIS_X];
	acc[obj->cvt.map[BMI160_ACC_AXIS_Y]] =
			obj->cvt.sign[BMI160_ACC_AXIS_Y] * acc_data[BMI160_ACC_AXIS_Y];
	acc[obj->cvt.map[BMI160_ACC_AXIS_Z]] =
			obj->cvt.sign[BMI160_ACC_AXIS_Z] * acc_data[BMI160_ACC_AXIS_Z];

	acc[obj->cvt.map[BMI160_ACC_AXIS_X]] = acc_data[BMI160_ACC_AXIS_X];
	acc[obj->cvt.map[BMI160_ACC_AXIS_Y]] = acc_data[BMI160_ACC_AXIS_Y];
	acc[obj->cvt.map[BMI160_ACC_AXIS_Z]] = acc_data[BMI160_ACC_AXIS_Z];	
	/* Output the mg */
	/*ACC_LOG("GRAVITY = %d, sensitivity = %d.\n",
			GRAVITY_EARTH_1000, obj->reso->sensitivity);*/
	acc[BMI160_ACC_AXIS_X] = acc[BMI160_ACC_AXIS_X] *
			GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[BMI160_ACC_AXIS_Y] = acc[BMI160_ACC_AXIS_Y] *
			GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[BMI160_ACC_AXIS_Z] = acc[BMI160_ACC_AXIS_Z] *
			GRAVITY_EARTH_1000 / obj->reso->sensitivity;
#endif	


	acc[BMI160_ACC_AXIS_X] = acc_data[BMI160_ACC_AXIS_X] *
			GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[BMI160_ACC_AXIS_Y] = acc_data[BMI160_ACC_AXIS_Y] *
			GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[BMI160_ACC_AXIS_Z] = acc_data[BMI160_ACC_AXIS_Z] *
			GRAVITY_EARTH_1000 / obj->reso->sensitivity;

	/*ACC_LOG("compensate acc data: x = %d, y = %d, z = %d\n",
			acc[0], acc[1], acc[2]);*/
	snprintf(buffer, 96, "%04x %04x %04x",
			acc[BMI160_ACC_AXIS_X],
			acc[BMI160_ACC_AXIS_Y],
			acc[BMI160_ACC_AXIS_Z]);
}

static void bhy_read_fifo_data(struct bhy_client_data *client_data,
	int reset_flag)
{
	int ret;
	u8 *data = client_data->fifo_buf;
	u8 event = 0;
	u8 sensor_sel = 0;
	u16 bytes_remain;
	int sensor_type;
	int parse_index, data_len;
	struct frame_queue *q = &client_data->data_queue;
	int idx; /* For self test index */
	int init_event_detected = BHY_FALSE;
	int self_result_detected = BHY_FALSE;
	int sigmo_detected = BHY_FALSE;
	int wake_gesture_detected = BHY_FALSE;
	int glance_gesture_detected = BHY_FALSE;
	int pick_up_gesture_detected = BHY_FALSE;
	struct bhy_sensor_context *ct;
	__le16 le16_val;
	int extra_flush;
	/*Ellen add for remapping : debug code, so write hard code here. 
	Actually, we can dynamically get the remapping based on position*/
    char acc_remapping[30]; 
	char gyro_remapping[30];//Ellen add for remapping
#ifdef BHY_AR_HAL_SUPPORT
	struct frame_queue *qa = &client_data->data_queue_ar;
#endif /*~ BHY_AR_HAL_SUPPORT */
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, BHY_REG_BYTES_REMAIN_0,
		(u8 *)&bytes_remain, 2);
	
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read bytes remain reg failed");
		return;
	}
#ifdef BHY_DEBUG
	if (client_data->enable_irq_log)
		PDEBUG("Fifo length: %d", bytes_remain);
#endif /*~ BHY_DEBUG */
	if (bytes_remain == 0) {
		printk("Zero length FIFO detected");
		if (client_data->hw_watchdog_disabled == BHY_TRUE)
			return;
		if (check_watchdog_reset(client_data) == BHY_TRUE)
			return;
#ifdef BHY_CHECK_MCU_INCONSISTENCY
		if (check_mcu_inconsistency(client_data) == BHY_TRUE)
			PERR("MCU inconsistency happened.");
#endif /*~ BHY_CHECK_MCU_INCONSISTENCY */
		return;
	}

	if (bytes_remain >= BHY_FIFO_LEN_MAX) {
		printk("bytes_remain(%d) exceeds maxium FIFO size", bytes_remain);
		return;
	}

	/* Feed the software watchdog */
	mutex_lock(&client_data->mutex_sw_watchdog);
	client_data->inactive_count = 0;
	mutex_unlock(&client_data->mutex_sw_watchdog);

	mutex_lock(&client_data->mutex_bus_op);
	
	ret = bhy_read_reg(client_data, BHY_REG_FIFO_BUFFER_0,
		data, bytes_remain);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read fifo data failed");
		return;
	}
	mutex_unlock(&client_data->mutex_bus_op);
#ifdef BHY_DEBUG
	if (client_data->enable_fifo_log)
		bhy_dump_fifo_data(data, bytes_remain);
#endif /*~ BHY_DEBUG */

	mutex_lock(&q->lock);
#ifdef BHY_AR_HAL_SUPPORT
	mutex_lock(&qa->lock);
#endif /*~ BHY_AR_HAL_SUPPORT */
	for (parse_index = 0; parse_index < bytes_remain;
		parse_index += data_len + 1) {
		sensor_type = data[parse_index];
		/* FIFO parsing should end with a 0 sensor type */
		if (sensor_type == 0)
			break;
		data_len = client_data->sensor_context[sensor_type].data_len;
		if (data_len < 0)
			break;
		if (parse_index + data_len >= bytes_remain) {
			PERR("Invalid FIFO data detected for sensor_type %d",
				sensor_type);
			break;
		}
		if (reset_flag == RESET_FLAG_READY) {
			if ((sensor_type == BHY_SENSOR_HANDLE_META_EVENT ||
				sensor_type ==
				BHY_SENSOR_HANDLE_META_EVENT_WU) &&
				data[parse_index + 1] ==
				META_EVENT_INITIALIZED) {
				atomic_set(&client_data->reset_flag,
					RESET_FLAG_INITIALIZED);
				init_event_detected = BHY_TRUE;
			}
		} else if (reset_flag == RESET_FLAG_SELF_TEST) {
			if (sensor_type == BHY_SENSOR_HANDLE_META_EVENT &&
				data[parse_index + 1] ==
				META_EVENT_SELF_TEST_RESULTS) {
				idx = -1;
				switch (data[parse_index + 2]) {
				case BHY_SENSOR_HANDLE_ACCELEROMETER:
					idx = PHYSICAL_SENSOR_INDEX_ACC;
					break;
				case BHY_SENSOR_HANDLE_MAG_UNCAL:
					idx = PHYSICAL_SENSOR_INDEX_MAG;
					break;
				case BHY_SENSOR_HANDLE_GYROSCOPE_UNCALIBRATED:
					idx = PHYSICAL_SENSOR_INDEX_GYRO;
					break;
				}
				if (idx != -1)
					client_data->self_test_result[idx] =
					(s8)data[parse_index + 3];
				self_result_detected = BHY_TRUE;
			}
		}
		switch (sensor_type) {
		case BHY_SENSOR_HANDLE_SIGNIFICANT_MOTION:
			sigmo_detected = BHY_TRUE;
			break;
		case BHY_SENSOR_HANDLE_WAKE_GESTURE:
			wake_gesture_detected = BHY_TRUE;
			ACC_LOG("bhy wake_gesture_detected\n");
			break;
		case BHY_SENSOR_HANDLE_GLANCE_GESTURE:
			glance_gesture_detected = BHY_TRUE;
			break;
		case BHY_SENSOR_HANDLE_PICK_UP_GESTURE:
			pick_up_gesture_detected = BHY_TRUE;
			break;
		case BHY_SENSOR_HANDLE_META_EVENT:
		case BHY_SENSOR_HANDLE_META_EVENT_WU:
			event = data[parse_index + 1];
			sensor_sel = data[parse_index + 2];
			if(event == META_EVENT_INITIALIZED)
			{
#if 1 //Ellen add for remapping
		       if(client_data->hw.direction <= 7 && client_data->hw.direction >= 0)
			   {
			    bhy_get_mapping_value(client_data->hw.direction,acc_remapping,gyro_remapping); 
				bhy_store_mapping_matrix(&(bmi160_acc_init_info.platform_diver_addr->driver),(const char*)acc_remapping,(size_t)50);			   
				msleep(2);
				bhy_store_mapping_matrix(&(bmi160_acc_init_info.platform_diver_addr->driver),(const char*)gyro_remapping,(size_t)50);
			   }
				/* Ellen for FOC part1 bug fix */
			    bhy_store_mapping_matrix_acc(&(bmi160_acc_init_info.platform_diver_addr->driver),
						"1", (size_t)10);
#endif /* Ellen add for remapping */
			}
			if (event != META_EVENT_FLUSH_COMPLETE)
				break;
			bhy_dequeue_flush(client_data, sensor_sel,
				&extra_flush);
			while (extra_flush--) {
				q->frames[q->head].handle = sensor_type;
				memcpy(q->frames[q->head].data,
					&data[parse_index + 1],	data_len);
				bhy_advance_fifo_queue_head(q);
			}
			break;
		case BHY_SENSOR_HANDLE_STEP_COUNTER:
		case BHY_SENSOR_HANDLE_STEP_COUNTER_WU:
			client_data->step_count_latest =
				client_data->step_count_base +
				le16_to_cpu(*(__le16 *)&data[parse_index + 1]);
			le16_val = cpu_to_le16(client_data->step_count_latest);
			memcpy(&data[parse_index + 1], &le16_val, 2);
			break;
		}
		q->frames[q->head].handle = sensor_type;
		memcpy(q->frames[q->head].data, &data[parse_index + 1],
			data_len);

		getRawDataFromQueue(client_data);
		bhy_advance_fifo_queue_head(q);

#ifdef BHY_AR_HAL_SUPPORT
		if (client_data->sensor_context[sensor_type].for_ar_hal ==
			BHY_TRUE) {
			qa->frames[qa->head].handle = sensor_type;
			memcpy(qa->frames[qa->head].data,
				&client_data->fifo_buf[parse_index + 1],
				data_len);
			if (qa->head == BHY_FRAME_SIZE_AR - 1)
				qa->head = 0;
			else
				++qa->head;
			if (qa->head == qa->tail) {
				if (qa->tail == BHY_FRAME_SIZE_AR - 1)
					qa->tail = 0;
				else
					++qa->tail;
			}
		}
#endif /*~ BHY_AR_HAL_SUPPORT */
	}
#ifdef BHY_AR_HAL_SUPPORT
	mutex_unlock(&qa->lock);
#endif /*~ BHY_AR_HAL_SUPPORT */
	mutex_unlock(&q->lock);

	//getRawDataFromQueue(client_data);

	/* Fix status for one shot sensor */
	ct = client_data->sensor_context;
	if (sigmo_detected == BHY_TRUE) {
		mutex_lock(&client_data->mutex_sw_watchdog);
		ct[BHY_SENSOR_HANDLE_SIGNIFICANT_MOTION].sample_rate = 0;
		ct[BHY_SENSOR_HANDLE_SIGNIFICANT_MOTION].enable = 0;
		client_data->enabled_counter--;
		mutex_unlock(&client_data->mutex_sw_watchdog);
	}
	if (wake_gesture_detected == BHY_TRUE) {
		mutex_lock(&client_data->mutex_sw_watchdog);
		ct[BHY_SENSOR_HANDLE_WAKE_GESTURE].sample_rate = 0;
		ct[BHY_SENSOR_HANDLE_WAKE_GESTURE].enable = 0;
		client_data->enabled_counter--;
		mutex_unlock(&client_data->mutex_sw_watchdog);
	}
	if (glance_gesture_detected == BHY_TRUE) {
		mutex_lock(&client_data->mutex_sw_watchdog);
		ct[BHY_SENSOR_HANDLE_GLANCE_GESTURE].sample_rate = 0;
		ct[BHY_SENSOR_HANDLE_GLANCE_GESTURE].enable = 0;
		client_data->enabled_counter--;
		mutex_unlock(&client_data->mutex_sw_watchdog);
	}
	if (pick_up_gesture_detected == BHY_TRUE) {
		mutex_lock(&client_data->mutex_sw_watchdog);
		ct[BHY_SENSOR_HANDLE_PICK_UP_GESTURE].sample_rate = 0;
		ct[BHY_SENSOR_HANDLE_PICK_UP_GESTURE].enable = 0;
		client_data->enabled_counter--;
		mutex_unlock(&client_data->mutex_sw_watchdog);
	}

	/* Recovery on disaster */
	if (init_event_detected == BHY_TRUE &&
		client_data->recover_from_disaster == BHY_TRUE) {
		bhy_recover_sensor_activity(client_data);
		client_data->recover_from_disaster = BHY_FALSE;
	}

	/* Re-init sensors */
	if (self_result_detected == BHY_TRUE) {
		bhy_reinit(client_data);
		client_data->recover_from_disaster = BHY_TRUE;
		atomic_set(&client_data->reset_flag, RESET_FLAG_READY);
	}

}

static irqreturn_t bhy_irq_handler(int irq, void *handle)
{
	struct bhy_client_data *client_data = handle;
	/*ktime_t ktime;*/

//	ACC_PR_ERR("bhy_irq_handler triggered!!\n");

	if (client_data == NULL)
		return IRQ_HANDLED;

#ifdef BHY_ON_NEXUS5
	if (g_rtc_suspend_flag) {
		schedule_delayed_work(&client_data->fiforead_work,
			msecs_to_jiffies(20));
	} else {
		bhy_get_ap_timestamp(&client_data->timestamp_irq);
		schedule_work(&client_data->irq_work);
	}
#else
	bhy_get_ap_timestamp(&client_data->timestamp_irq);
	schedule_work(&client_data->irq_work);
#endif

#ifdef BHY_LEVEL_TRIGGERED_IRQ_SUPPORT
	/* Disable IRQ to prevent additional entry */
	if (client_data->irq_enabled == BHY_TRUE) {
		client_data->irq_enabled = BHY_FALSE;
		disable_irq_nosync(client_data->data_bus.irq);
	}
#endif /*~ BHY_LEVEL_TRIGGERED_IRQ_SUPPORT */

	return IRQ_HANDLED;
}

#ifdef BHY_ON_NEXUS5
static void bhy_irq_work_func_rtc(struct work_struct *work)
{
	struct bhy_client_data *client_data =
		container_of(work, struct bhy_client_data,
		fiforead_work.work);
	int reset_flag_copy;
	int ret;
#ifdef BHY_DEBUG
	u8 irq_status;
#endif /*~ BHY_DEBUG */

	PDEBUG("irq_rtc() rtc %d\n", g_rtc_suspend_flag);

	if (client_data->irq_normal_run_flag) {
		PINFO("normalirq handling, not run rtcirq handling\n");
		return;
	}

	wake_lock(&client_data->wlock);
	client_data->irq_rtc_run_flag = 1; /* Set flag */
	if (g_rtc_suspend_flag) {
		schedule_delayed_work(&client_data->fiforead_work,
			msecs_to_jiffies(20));
		client_data->irq_rtc_run_flag = 0; /* Clear flag */
		wake_unlock(&client_data->wlock);
		return;
	}
	/* Detect reset event */
	reset_flag_copy = atomic_read(&client_data->reset_flag);
	if (reset_flag_copy == RESET_FLAG_TODO) {
		atomic_set(&client_data->reset_flag, RESET_FLAG_READY);
		client_data->irq_rtc_run_flag = 0; /* Clear flag */
		wake_unlock(&client_data->wlock);
		PERR("in irq, RESET_FLAG_TODO\n");
		return;
	}

#ifdef BHY_DEBUG
	if (client_data->enable_irq_log) {
		irq_status = 0;
		mutex_lock(&client_data->mutex_bus_op);
		ret = bhy_read_reg(client_data, BHY_REG_INT_STATUS,
			&irq_status, 1);
		mutex_unlock(&client_data->mutex_bus_op);
		if (ret < 0)
			PERR("Read IRQ status failed");
	}
#endif /*~ BHY_DEBUG */

	bhy_get_ap_timestamp(&client_data->timestamp_irq);
	bhy_read_timestamp_sync(client_data);

	/* Read FIFO data */
	bhy_read_fifo_data(client_data, reset_flag_copy);

	input_event(client_data->input, EV_MSC, MSC_RAW, 0);
	input_sync(client_data->input);

#ifdef BHY_AR_HAL_SUPPORT
	input_event(client_data->input_ar, EV_MSC, MSC_RAW, 0);
	input_sync(client_data->input_ar);
#endif /*~ BHY_AR_HAL_SUPPORT */

#ifdef BHY_LEVEL_TRIGGERED_IRQ_SUPPORT
	/* Enable IRQ after data processing */
	if (client_data->irq_enabled == BHY_FALSE) {
		client_data->irq_enabled = BHY_TRUE;
		enable_irq(client_data->data_bus.irq);
	}
#endif /*~ BHY_LEVEL_TRIGGERED_IRQ_SUPPORT */
	client_data->irq_rtc_run_flag = 0;  /* Clear flag */
	wake_unlock(&client_data->wlock);
}
#endif


static void bhy_gesture_interrupt_handle(void)
{
	bhy_gesture_notify();
	bhy_gesture_status = BHY_STATUS_DISABLE;
	ACC_LOG("bhy gesture handler done.\n");
}

static void bhy_irq_work_func(struct work_struct *work)
{
	struct bhy_client_data *client_data = container_of(
		work, struct bhy_client_data, irq_work);
	int reset_flag_copy;
	int ret;
#ifdef BHY_DEBUG
	u8 irq_status;
#endif /*~ BHY_DEBUG */

	wake_lock(&client_data->wlock);
#ifdef BHY_ON_NEXUS5
	if (client_data->irq_rtc_run_flag) {
		PINFO("rtcirq handling, not run normalirq\n");
		bhy_get_ap_timestamp(&client_data->timestamp_irq);
		schedule_work(&client_data->irq_work);
		wake_unlock(&client_data->wlock);
		return;
	}
	client_data->irq_normal_run_flag = 1; /* Set flag */
#endif
	/* Detect reset event */
	reset_flag_copy = atomic_read(&client_data->reset_flag);

	if (reset_flag_copy == RESET_FLAG_TODO) {
		atomic_set(&client_data->reset_flag, RESET_FLAG_READY);
#ifdef BHY_ON_NEXUS5
		client_data->irq_normal_run_flag = 0; /* Clear flag */
#endif
		//enable_irq(bhy_irq);
		wake_unlock(&client_data->wlock);
		printk("in irq, RESET_FLAG_TODO\n");
		return;
	}

#ifdef BHY_DEBUG
	if (client_data->enable_irq_log) {
		irq_status = 0;
		mutex_lock(&client_data->mutex_bus_op);
		ret = bhy_read_reg(client_data, BHY_REG_INT_STATUS,
			&irq_status, 1);
		mutex_unlock(&client_data->mutex_bus_op);
		if (ret < 0)
			PERR("Read IRQ status failed");
		PDEBUG("In IRQ, timestamp: %llu, irq_type: 0x%02X",
			client_data->timestamp_irq, irq_status);
	}
#endif /*~ BHY_DEBUG */

	bhy_read_timestamp_sync(client_data);

	/* Read FIFO data */
	bhy_read_fifo_data(client_data, reset_flag_copy);

	//input_event(client_data->input, EV_MSC, MSC_RAW, 0);
	//input_sync(client_data->input);

#ifdef BHY_AR_HAL_SUPPORT
	input_event(client_data->input_ar, EV_MSC, MSC_RAW, 0);
	input_sync(client_data->input_ar);
#endif /*~ BHY_AR_HAL_SUPPORT */

#ifdef BHY_LEVEL_TRIGGERED_IRQ_SUPPORT
	/* Enable IRQ after data processing */
	if (client_data->irq_enabled == BHY_FALSE) {
		client_data->irq_enabled = BHY_TRUE;
		enable_irq(client_data->data_bus.irq);
	}
#endif /*~ BHY_LEVEL_TRIGGERED_IRQ_SUPPORT */

#ifdef BHY_ON_NEXUS5
	client_data->irq_normal_run_flag = 0; /* Clear flag */
#endif
	wake_unlock(&client_data->wlock);

	if (BHY_STATUS_ENABLE == bhy_gesture_status) {
		bhy_gesture_interrupt_handle();
	}

}

static int bhy_request_irq(struct bhy_client_data *client_data)
{
	struct bhy_data_bus *data_bus = &client_data->data_bus;
	int ret;
	u32 ints[2] = {0, 0};

#if 0
	int err = 0;
	/* u8 reset_req; */
	/*for mtk6795*/
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek, ALS-eint");
	if (node) {
		data_bus->irq = irq_of_parse_and_map(node, 0);
		err = request_irq(data_bus->irq, bhy_irq_handler,
			IRQF_TRIGGER_RISING, "bhy_int", client_data);//data_bus
		if (err > 0)
			printk("bhy request_irq failed");
	} else
		printk("[%s] can not find!", __func__);
	/*for mtk6795 end*/
	bhy_irq = data_bus->irq;
#endif
	INIT_WORK(&client_data->irq_work, bhy_irq_work_func);

if (client_data->irq_node) {
	of_property_read_u32_array(client_data->irq_node, "debounce", ints, ARRAY_SIZE(ints));
	gpio_set_debounce(ints[0], ints[1]);
	ACC_PR_ERR("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

	data_bus->irq = irq_of_parse_and_map(client_data->irq_node, 0);
	if (data_bus->irq  < 0) {
		ACC_PR_ERR("alsps request_irq IRQ LINE NOT AVAILABLE!.");
		return -1;
	}
	bhy_irq = data_bus->irq;
	ACC_PR_ERR("bhy->irq = %d\n", bhy_irq);
	if (!bhy_irq) {
		ACC_PR_ERR("irq_of_parse_and_map fail!!\n");
		return -EINVAL;
	}
} else {
	ACC_PR_ERR("null irq node!!\n");
	return -EINVAL;
}


#ifdef BHY_LEVEL_TRIGGERED_IRQ_SUPPORT
	client_data->irq_enabled = BHY_TRUE;
	ret = request_irq(bhy_irq, (irq_handler_t)bhy_irq_handler, IRQ_TYPE_LEVEL_HIGH,
		SENSOR_NAME, client_data);
	PINFO("Use level triggered IRQ");
#else
	
	ret = request_irq(data_bus->irq, bhy_irq_handler, IRQF_TRIGGER_RISING,
		SENSOR_NAME, client_data);
	
#endif /*~ BHY_LEVEL_TRIGGERED_IRQ_SUPPORT */
	if (ret < 0)
		return ret;
	ret = device_init_wakeup(data_bus->dev, 1);
	if (ret < 0) {
		PDEBUG("Init device wakeup failed");
		return ret;
	}
	printk("bhy_request_irq IRQ_num = %d.\n", data_bus->irq);
	return 0;
}

static int bhy_init_input_dev(struct bhy_client_data *client_data)
{
	struct input_dev *dev;
	int ret;

	dev = input_allocate_device();
	if (dev == NULL) {
		PERR("Allocate input device failed");
		return -ENOMEM;
	}

	dev->name = SENSOR_INPUT_DEV_NAME;
	dev->id.bustype = client_data->data_bus.bus_type;

	input_set_capability(dev, EV_MSC, MSC_RAW);
	input_set_drvdata(dev, client_data);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		PERR("Register input device failed");
		return ret;
	}
	client_data->input = dev;

#ifdef BHY_AR_HAL_SUPPORT
	dev = input_allocate_device();
	if (dev == NULL) {
		PERR("Allocate input device failed for AR");
		return -ENOMEM;
	}

	dev->name = SENSOR_AR_INPUT_DEV_NAME;
	dev->id.bustype = client_data->data_bus.bus_type;

	input_set_capability(dev, EV_MSC, MSC_RAW);
	input_set_drvdata(dev, client_data);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		PERR("Register input device for AR failed");
		return ret;
	}
	client_data->input_ar = dev;
#endif /*~ BHY_AR_HAL_SUPPORT */

	return 0;
}

static int bmi160_acc_read_chipInfo(
	struct i2c_client *client, char *buf, int bufsize)
{
	snprintf(buf, PAGE_SIZE, "bmi160_acc");
	return 0;
}

static int bmi160_acc_read_sensor_data(struct bhy_client_data *obj)
{
	char buff[BMI160_BUFSIZE];
	getCompensateAccData(obj, buff);
	return 0;
}

#if 0
static int bmi160_acc_read_raw_data(struct i2c_client *client, char *buf)
{
	snprintf(buf, PAGE_SIZE, "bmi160_acc_read_raw_data %04x %04x %04x",
			acc_data[BMI160_ACC_AXIS_X],
			acc_data[BMI160_ACC_AXIS_Y],
			acc_data[BMI160_ACC_AXIS_Z]);
	return 0;
}
#endif

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[BMI160_BUFSIZE];
	struct i2c_client *client = bmi160_acc_i2c_client;
	bmi160_acc_read_chipInfo(client, strbuf, BMI160_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	char strbuf[BMI160_BUFSIZE] = {0};
	struct bhy_client_data *client_data = obj_i2c_data;
	bmi160_acc_read_sensor_data(client_data);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "not support\n");
}

static ssize_t store_firlen_value( struct device_driver *ddri,
		const char *buf, size_t count)
{
	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct bhy_client_data *obj = obj_i2c_data;

	len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			obj->hw.i2c_num, obj->hw.direction,
			obj->hw.power_id, obj->hw.power_vol);
	return len;
}

static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t ret;
	if (sensor_power)
		ACC_LOG("G sensor in work mode, sensor_power = %d\n", sensor_power);
	else
		ACC_LOG("G sensor in standby mode, sensor_power = %d\n", sensor_power);

	ret = snprintf(buf, 32, "%d\n", (int)sensor_power);
	return 0;
}

static ssize_t bhy_show_rom_id(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	__le16 reg_data;
	u16 rom_id;
	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, BHY_REG_ROM_VERSION_0,
		(u8 *)&reg_data, 2);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret < 0)
		return ret;
	rom_id = __le16_to_cpu(reg_data);
	ret = snprintf(buf, 32, "%d\n", (int)rom_id);

	return ret;
}

static ssize_t bhy_show_ram_id(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	__le16 reg_data;
	u16 ram_id;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, BHY_REG_RAM_VERSION_0,
		(u8 *)&reg_data, 2);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret < 0)
		return ret;
	ram_id = __le16_to_cpu(reg_data);
	ret = snprintf(buf, 32, "%d\n", (int)ram_id);

	return ret;
}

static ssize_t bhy_show_status_bank(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	int i;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	for (i = BHY_PARAM_SYSTEM_STAUS_BANK_0;
			i <= BHY_PARAM_SYSTEM_STAUS_BANK_3; ++i) {
		ret = bhy_read_parameter(client_data, BHY_PAGE_SYSTEM, i,
				(u8 *)(buf + (i - BHY_PARAM_SYSTEM_STAUS_BANK_0)
				* 16), 16);
		if (ret < 0) {
			PERR("Read BHY_PARAM_SYSTEM_STAUS_BANK_%d error",
					i - BHY_PARAM_SYSTEM_STAUS_BANK_0);
			mutex_unlock(&client_data->mutex_bus_op);
			return ret;
		}
	}
	mutex_unlock(&client_data->mutex_bus_op);

	return BHY_SENSOR_STATUS_BANK_LEN;
}

static ssize_t bhy_store_sensor_sel(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	client_data->sensor_sel = buf[0];

	return count;
}

static ssize_t bhy_show_sensor_info(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	if (client_data->sensor_sel <= 0 ||
			client_data->sensor_sel > BHY_SENSOR_HANDLE_MAX) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Invalid sensor sel");
		return -EINVAL;
	}
	ret = bhy_read_parameter(client_data, BHY_PAGE_SENSOR,
			BHY_PARAM_SENSOR_INFO_0 + client_data->sensor_sel,
			(u8 *)buf, 16);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read parameter error");
		return ret;
	}

	return 16;
}

static ssize_t bhy_show_sensor_conf(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int sel;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	sel = client_data->sensor_sel;
	mutex_unlock(&client_data->mutex_bus_op);
	if (sel <= 0 || sel > BHY_SENSOR_HANDLE_MAX) {
		PERR("Invalid sensor sel");
		return -EINVAL;
	}

	return bhy_get_sensor_conf(client_data, sel, buf);
}

static ssize_t bhy_store_sensor_conf(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int sel;
	struct bhy_sensor_context *ct;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	sel = client_data->sensor_sel;
	mutex_unlock(&client_data->mutex_bus_op);
	if (sel <= 0 || sel > BHY_SENSOR_HANDLE_MAX) {
		PERR("Invalid sensor sel");
		return -EINVAL;
	}

	mutex_lock(&client_data->mutex_sw_watchdog);
	ct = client_data->sensor_context;
	ct[sel].sample_rate = *(u16 *)buf;
	ct[sel].report_latency = *(u16 *)(buf + 2);
	client_data->inactive_count = 0;
	mutex_unlock(&client_data->mutex_sw_watchdog);
	if (ct[sel].sample_rate) {
		if (ct[sel].enable == 0) {
			client_data->enabled_counter++;
			if (ct[sel].type == SENSOR_TYPE_CONTINUOUS) {
				client_data->cont_sensor_count++;
				if (client_data->cont_sensor_count == 1)
					client_data->data_check_flag = 1;
			}
		}
		ct[sel].enable = 1;
	} else {
		if (ct[sel].enable == 1) {
			client_data->enabled_counter--;
			if (ct[sel].type == SENSOR_TYPE_CONTINUOUS) {
				client_data->cont_sensor_count++;
				if (client_data->cont_sensor_count == 0)
					client_data->data_check_flag = 0;
			}
		}
		ct[sel].enable = 0;
	}

	/* Clear flush queue if all sensor are de-activated */
	if (client_data->enabled_counter == 0) {
		mutex_lock(&client_data->flush_queue.lock);
		if (client_data->flush_queue.head !=
			client_data->flush_queue.tail) {
			PDEBUG("Flush queue not empty on all sensor close.");
			bhy_clear_flush_queue(&client_data->flush_queue);
		}
		mutex_unlock(&client_data->flush_queue.lock);
	}
	if (bhy_set_sensor_conf(client_data, sel, buf) < 0)
		return -EIO;

	ACC_LOG("bhy_store_sensor_conf successfully.");
	return count;
}

static ssize_t bhy_store_sensor_flush(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	u8 sensor_sel = buf[0];

	ret = bhy_enqueue_flush(client_data, sensor_sel);
	if (ret < 0) {
		PERR("Write sensor flush failed");
		return ret;
	}

	return count;
}

static ssize_t bhy_show_calib_profile(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	u8 param_num;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
#ifdef BHY_CALIB_PROFILE_OP_IN_FUSER_CORE
	switch (client_data->sensor_sel) {
	case BHY_SENSOR_HANDLE_ACCELEROMETER:
		param_num = BHY_PARAM_OFFSET_ACC_2;
		break;
	case BHY_SENSOR_HANDLE_GEOMAGNETIC_FIELD:
		param_num = BHY_PARAM_OFFSET_MAG_2;
		break;
	case BHY_SENSOR_HANDLE_GYROSCOPE:
		param_num = BHY_PARAM_OFFSET_GYRO_2;
		break;
	default:
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Invalid sensor sel");
		return -EINVAL;
	}
#else
	switch (client_data->sensor_sel) {
	case BHY_SENSOR_HANDLE_ACCELEROMETER:
		param_num = BHY_PARAM_OFFSET_ACC;
		break;
	case BHY_SENSOR_HANDLE_GEOMAGNETIC_FIELD:
		param_num = BHY_PARAM_OFFSET_MAG;
		break;
	case BHY_SENSOR_HANDLE_GYROSCOPE:
		param_num = BHY_PARAM_OFFSET_GYRO;
		break;
	default:
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Invalid sensor sel");
		return -EINVAL;
	}
#endif /*~ BHY_CALIB_PROFILE_OP_IN_FUSER_CORE */
	ret = bhy_read_parameter(client_data, BHY_PAGE_ALGORITHM,
		param_num, (u8 *)buf, BHY_CALIB_PROFILE_LEN);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read parameter error");
		return ret;
	}

	return BHY_CALIB_PROFILE_LEN;
}

static ssize_t bhy_store_calib_profile(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = bhy_set_calib_profile(client_data, (const u8 *)buf);
	if (ret < 0) {
		PERR("bhy_set_calib_profile failed");
		return ret;
	}

	/* Save calibration profile for disaster recovery */
	mutex_lock(&client_data->mutex_bus_op);
	switch (client_data->sensor_sel) {
	case BHY_SENSOR_HANDLE_ACCELEROMETER:
		memcpy(client_data->calibprofile_acc, buf, 8);
		break;
	case BHY_SENSOR_HANDLE_GEOMAGNETIC_FIELD:
		memcpy(client_data->calibprofile_mag, buf, 8);
		break;
	case BHY_SENSOR_HANDLE_GYROSCOPE:
		memcpy(client_data->calibprofile_gyro, buf, 8);
		break;
	}
	mutex_unlock(&client_data->mutex_bus_op);

	return count;
}

static ssize_t bhy_show_sic_matrix(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	u8 data[36];
	int i;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	for (i = BHY_PARAM_SIC_MATRIX_0_1; i <= BHY_PARAM_SIC_MATRIX_8; ++i) {
		ret = bhy_read_parameter(client_data, BHY_PAGE_ALGORITHM,
				i, (u8 *)(data + (i - 1) * 8),
				i == BHY_PARAM_SIC_MATRIX_8 ? 4 : 8);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read parameter error");
			return ret;
		}
	}
	mutex_unlock(&client_data->mutex_bus_op);
	ret = 0;
	for (i = 0; i < 9; ++i)
		ret += snprintf(buf + ret, 16, "%02X %02X %02X %02X\n",
				data[i * 4], data[i * 4 + 1],
				data[i * 4 + 2], data[i * 4 + 3]);

	return ret;
}

static ssize_t bhy_store_sic_matrix(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	int i;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	for (i = BHY_PARAM_SIC_MATRIX_0_1; i <= BHY_PARAM_SIC_MATRIX_8; ++i) {
		ret = bhy_write_parameter(client_data, BHY_PAGE_ALGORITHM,
				i, (u8 *)(buf + (i - 1) * 8),
				i == BHY_PARAM_SIC_MATRIX_8 ? 4 : 8);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write parameter error");
			return ret;
		}
	}
	mutex_unlock(&client_data->mutex_bus_op);

	return count;
}

static ssize_t bhy_show_meta_event_ctrl(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	u8 data[8];
	int i, j;
	ssize_t len;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_SYSTEM,
			BHY_PARAM_SYSTEM_META_EVENT_CTRL, data, 8);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read meta event ctrl failed");
		return -EIO;
	}
	len = 0;
	len += snprintf(buf + len, 64, "Non wake up meta event\n");
	for (i = 0; i < 8; ++i) {
		for (j = 0; j < 4; ++j)
			len += snprintf(buf + len, 64,
					"Meta event #%d: event_en=%d, irq_en=%d\n",
					i * 4 + j + 1,
					(data[i] >> (j * 2 + 1)) & 1,
					(data[i] >> (j * 2)) & 1);
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_SYSTEM,
			BHY_PARAM_SYSTEM_WAKE_UP_META_EVENT_CTRL, data, 8);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read wake up meta event ctrl failed");
		return -EIO;
	}
	len += snprintf(buf + len, 64, "Wake up meta event\n");
	for (i = 0; i < 8; ++i) {
		for (j = 0; j < 4; ++j)
			len += snprintf(buf + len, 64,
					"Meta event #%d: event_en=%d, irq_en=%d\n",
					i * 4 + j + 1,
					(data[i] >> (j * 2 + 1)) & 1,
					(data[i] >> (j * 2)) & 1);
	}

	return len;
}

/* Byte0: meta event type; Byte1: event enable; Byte2: IRQ enable;
   Byte3: 0 for non-wakeup, 1 for wakeup */
static ssize_t bhy_store_meta_event_ctrl(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	int type, event_en, irq_en, for_wake_up;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	type = buf[0];
	if (type <= 0 || type > BHY_META_EVENT_MAX) {
		PERR("Invalid meta event type");
		return -EINVAL;
	}
	event_en = buf[1] & 0x1 ? BHY_TRUE : BHY_FALSE;
	irq_en = buf[2] & 0x1 ? BHY_TRUE : BHY_FALSE;
	for_wake_up = buf[3] ? BHY_TRUE : BHY_FALSE;

	ret = bhy_set_meta_event_ctrl(client_data, type, for_wake_up,
		event_en, irq_en);
	if (ret < 0)
		return -EIO;

	if (for_wake_up == BHY_TRUE) {
		client_data->mew_context[type].event_en =
			event_en == BHY_TRUE ? BHY_STATUS_ENABLED :
			BHY_STATUS_DISABLED;
		client_data->mew_context[type].irq_en =
			irq_en == BHY_TRUE ? BHY_STATUS_ENABLED :
			BHY_STATUS_DISABLED;
	} else {
		client_data->me_context[type].event_en =
			event_en == BHY_TRUE ? BHY_STATUS_ENABLED :
			BHY_STATUS_DISABLED;
		client_data->me_context[type].irq_en =
			irq_en == BHY_TRUE ? BHY_STATUS_ENABLED :
			BHY_STATUS_DISABLED;
	}

	return count;
}

static ssize_t bhy_show_fifo_ctrl(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_SYSTEM,
			BHY_PARAM_SYSTEM_FIFO_CTRL, buf,
			BHY_FIFO_CTRL_PARAM_LEN);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read fifo ctrl failed");
		return -EIO;
	}

	return BHY_FIFO_CTRL_PARAM_LEN;
}

static ssize_t bhy_store_fifo_ctrl(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = bhy_set_fifo_ctrl(client_data, (u8 *)buf);
	if (ret < 0) {
		PERR("Set fifo ctrl failed");
		return ret;
	}

	/* Save fifo control cfg for future recovery */
	memcpy(client_data->fifo_ctrl_cfg, buf, BHY_FIFO_CTRL_PARAM_LEN);

	return count;
}

#ifdef BHY_AR_HAL_SUPPORT
static ssize_t bhy_store_activate_ar_hal(struct device *dev
		, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bhy_client_data *client_data = input_get_drvdata(input);
	ssize_t ret;
	long req;
	struct frame_queue *qa = &client_data->data_queue_ar;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = kstrtol(buf, 10, &req);
	if (ret < 0 || req != 1) {
		PERR("Invalid request");
		return -EINVAL;
	}

	mutex_lock(&qa->lock);
	qa->frames[qa->head].handle = BHY_AR_ACTIVATE;
	if (qa->head == BHY_FRAME_SIZE_AR - 1)
		qa->head = 0;
	else
		++qa->head;
	if (qa->head == qa->tail) {
		if (qa->tail == BHY_FRAME_SIZE_AR - 1)
			qa->tail = 0;
		else
			++qa->tail;
	}
	mutex_unlock(&qa->lock);

	input_event(client_data->input_ar, EV_MSC, MSC_RAW, 0);
	input_sync(client_data->input_ar);
	PDEBUG("AR HAL activate message sent");

	return count;
}
#endif /*~ BHY_AR_HAL_SUPPORT */

static ssize_t bhy_show_reset_flag(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int reset_flag_copy;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	reset_flag_copy = atomic_read(&client_data->reset_flag);
	buf[0] = 0x30 + (u8)reset_flag_copy;
	buf[1] = '\n';
	buf[2] = '\0';

	return 2;
}

/* 16-bit working mode value */
static ssize_t bhy_show_working_mode(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_ALGORITHM,
			BHY_PARAM_WORKING_MODE_ENABLE, buf, 2);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read working mode mask failed");
		return -EIO;
	}

	return BHY_FIFO_CTRL_PARAM_LEN;
}

static ssize_t bhy_store_working_mode(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_ALGORITHM,
			BHY_PARAM_WORKING_MODE_ENABLE, (u8 *)buf, 2);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write working mode mask failed");
		return -EIO;
	}

	return count;
}

static ssize_t bhy_show_op_mode(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	u8 data[2];
	char op_mode[64];
	int ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_ALGORITHM,
			BHY_PARAM_OPERATING_MODE, data, 2);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read op mode failed");
		return -EIO;
	}

	switch (data[1]) {
	case 0:
		strlcpy(op_mode, "SLEEP", 64);
		break;
	case 1:
		strlcpy(op_mode, "ACCONLY", 64);
		break;
	case 2:
		strlcpy(op_mode, "GYROONLY", 64);
		break;
	case 3:
		strlcpy(op_mode, "MAGONLY", 64);
		break;
	case 4:
		strlcpy(op_mode, "ACCGYRO", 64);
		break;
	case 5:
		strlcpy(op_mode, "ACCMAG", 64);
		break;
	case 6:
		strlcpy(op_mode, "MAGGYRO", 64);
		break;
	case 7:
		strlcpy(op_mode, "AMG", 64);
		break;
	case 8:
		strlcpy(op_mode, "IMUPLUS", 64);
		break;
	case 9:
		strlcpy(op_mode, "COMPASS", 64);
		break;
	case 10:
		strlcpy(op_mode, "M4G", 64);
		break;
	case 11:
		strlcpy(op_mode, "NDOF", 64);
		break;
	case 12:
		strlcpy(op_mode, "NDOF_FMC_OFF", 64);
		break;
	case 13:
		strlcpy(op_mode, "NDOF_GEORV", 64);
		break;
	case 14:
		strlcpy(op_mode, "NDOF_GEORV_FMC_OFF", 64);
		break;
	default:
		snprintf(op_mode, 64, "Unrecoginized op mode[%d]",
				data[1]);
		break;
	}

	ret = snprintf(buf, 128, "Current op mode: %s, odr: %dHz\n",
			op_mode, data[0]);

	return ret;
}

static ssize_t bhy_show_bsx_version(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	u8 data[8];
	int ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_ALGORITHM,
			BHY_PARAM_BSX_VERSION, data, 8);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read BSX version failed");
		return -EIO;
	}

	ret = snprintf(buf, 128, "%d.%d.%d.%d\n",
			*(u16 *)data, *(u16 *)(data + 2),
			*(u16 *)(data + 4), *(u16 *)(data + 6));

	return ret;
}

static ssize_t bhy_show_driver_version(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = snprintf(buf, 128, "Driver version: %s\n",
			DRIVER_VERSION);

	return ret;
}

#ifdef BHY_AR_HAL_SUPPORT
static ssize_t bhy_show_fifo_frame_ar(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	struct frame_queue *qa = &client_data->data_queue_ar;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&qa->lock);
	if (qa->tail == qa->head) {
		mutex_unlock(&qa->lock);
		return 0;
	}
	memcpy(buf, &qa->frames[qa->tail], sizeof(struct fifo_frame));
	if (qa->tail == BHY_FRAME_SIZE_AR - 1)
		qa->tail = 0;
	else
		++qa->tail;
	mutex_unlock(&qa->lock);

	return sizeof(struct fifo_frame);
}
#endif /*~ BHY_AR_HAL_SUPPORT */

static ssize_t bhy_show_bmi160_foc_offset_acc(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	u8 data[3];

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bmi160_read_reg(client_data, BMI160_REG_ACC_OFFSET_X,
		data, sizeof(data));
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read hw reg failed");
		return ret;
	}

	return snprintf(buf, 64, "%11d %11d %11d\n",
		*(s8 *)data, *(s8 *)(data + 1), *(s8 *)(data + 2));
}

static ssize_t bhy_store_bmi160_foc_offset_acc(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int temp[3];
	s8 data[3];
	u8 val;
	int ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = sscanf(buf, "%11d %11d %11d", &temp[0], &temp[1], &temp[2]);
	if (ret != 3) {
		PERR("Invalid input");
		return -EINVAL;
	}
	data[0] = temp[0];
	data[1] = temp[1];
	data[2] = temp[2];
	mutex_lock(&client_data->mutex_bus_op);
	/* Write offset */
	ret = bmi160_write_reg(client_data, BMI160_REG_ACC_OFFSET_X,
		(u8 *)data, sizeof(data));
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write hw reg failed");
		return ret;
	}
	/* Write offset enable bit */
	ret = bmi160_read_reg(client_data, BMI160_REG_OFFSET_6,
		&val, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read hw reg for enable bit failed");
		return ret;
	}
	val |= BMI160_OFFSET_6_BIT_ACC_EN;
	ret = bmi160_write_reg(client_data, BMI160_REG_OFFSET_6,
		&val, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write hw reg for enable bit failed");
		return ret;
	}
	mutex_unlock(&client_data->mutex_bus_op);

	return count;
}

static ssize_t bhy_show_bmi160_foc_offset_gyro(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	s8 data[4];
	s16 x, y, z, h;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bmi160_read_reg(client_data, BMI160_REG_GYRO_OFFSET_X,
		(u8 *)data, sizeof(data));
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read hw reg failed");
		return ret;
	}

	/* left shift 6 bits to make sign bit msb, then shift back */
	h = (data[3] & BMI160_OFFSET_6_MASK_GYRO_X) >>
		BMI160_OFFSET_6_OFFSET_GYRO_X;
	x = ((h << 8) | data[0]) << 6;
	x >>= 6;
	h = (data[3] & BMI160_OFFSET_6_MASK_GYRO_Y) >>
		BMI160_OFFSET_6_OFFSET_GYRO_Y;
	y = ((h << 8) | data[1]) << 6;
	y >>= 6;
	h = (data[3] & BMI160_OFFSET_6_MASK_GYRO_Z) >>
		BMI160_OFFSET_6_OFFSET_GYRO_Z;
	z = ((h << 8) | data[2]) << 6;
	z >>= 6;

	return snprintf(buf, 64, "%11d %11d %11d\n", x, y, z);
}

static ssize_t bhy_store_bmi160_foc_offset_gyro(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int x, y, z;
	u8 data[3];
	u8 val;
	int ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = sscanf(buf, "%11d %11d %11d", &x, &y, &z);
	if (ret != 3) {
		PERR("Invalid input");
		return -EINVAL;
	}

	data[0] = x & 0xFF;
	data[1] = y & 0xFF;
	data[2] = z & 0xFF;
	mutex_lock(&client_data->mutex_bus_op);
	/* Set low 8-bit */
	ret = bmi160_write_reg(client_data, BMI160_REG_GYRO_OFFSET_X,
		data, sizeof(data));
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write hw reg failed");
		return ret;
	}
	/* Set high bit, extract 9th bit and 10th bit from x, y, z
	* Set enable bit */
	ret = bmi160_read_reg(client_data, BMI160_REG_OFFSET_6,
		&val, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read hw reg for enable bit failed");
		return ret;
	}
	val &= ~BMI160_OFFSET_6_MASK_GYRO_X;
	val |= ((x >> 8) & 0x03) << BMI160_OFFSET_6_OFFSET_GYRO_X;
	val &= ~BMI160_OFFSET_6_MASK_GYRO_Y;
	val |= ((y >> 8) & 0x03) << BMI160_OFFSET_6_OFFSET_GYRO_Y;
	val &= ~BMI160_OFFSET_6_MASK_GYRO_Z;
	val |= ((z >> 8) & 0x03) << BMI160_OFFSET_6_OFFSET_GYRO_Z;
	val |= BMI160_OFFSET_6_BIT_GYRO_EN;
	ret = bmi160_write_reg(client_data, BMI160_REG_OFFSET_6,
		&val, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write hw reg for enable bit failed");
		return ret;
	}
	mutex_unlock(&client_data->mutex_bus_op);

	return count;
}

static ssize_t bhy_show_bmi160_foc_conf(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int x, y, z, g;
	int out[3], in[3], i;
	const char *disp[4] = {
		"disabled",
		"1g",
		"-1g",
		"0"
	};
	u8 conf;
	ssize_t ret = 0;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	conf = client_data->bmi160_foc_conf;

	x = (conf & BMI160_FOC_CONF_MASK_ACC_X) >> BMI160_FOC_CONF_OFFSET_ACC_X;
	y = (conf & BMI160_FOC_CONF_MASK_ACC_Y) >> BMI160_FOC_CONF_OFFSET_ACC_Y;
	z = (conf & BMI160_FOC_CONF_MASK_ACC_Z) >> BMI160_FOC_CONF_OFFSET_ACC_Z;
	g = (conf & BMI160_FOC_CONF_MASK_GYRO) >> BMI160_FOC_CONF_OFFSET_GYRO;

	out[0] = x;
	out[1] = y;
	out[2] = z;
	for (i = 0; i < 3; ++i) {
		in[i] = out[0] * client_data->mapping_matrix_acc_inv[0][i] +
			out[1] * client_data->mapping_matrix_acc_inv[1][i] +
			out[2] * client_data->mapping_matrix_acc_inv[2][i];
		switch (in[i]) {
		case -1:
			in[i] = 2;
			break;
		case -2:
			in[i] = 1;
			break;
		case -3:
			in[i] = 3;
			break;
		default:
			break;
		}
	}

	ret += snprintf(buf + ret, 128, "Acc conf: %s %s %s Gyro: %s\n",
		disp[x], disp[y], disp[z], g ? "enabled" : "disabled");
	ret += snprintf(buf + ret, 128, "Original acc conf: %s %s %s\n",
		disp[in[0]], disp[in[1]], disp[in[2]]);

	return ret;
}

static ssize_t bhy_store_bmi160_foc_conf(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int i;
	int mask, offset;
	u8 conf = 0;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	for (i = 0; i < count; ++i) {
		mask = 0;
		switch (buf[i]) {
		case 'x':
		case 'X':
			mask = BMI160_FOC_CONF_MASK_ACC_X;
			offset = BMI160_FOC_CONF_OFFSET_ACC_X;
			break;
		case 'y':
		case 'Y':
			mask = BMI160_FOC_CONF_MASK_ACC_Y;
			offset = BMI160_FOC_CONF_OFFSET_ACC_Y;
			break;
		case 'z':
		case 'Z':
			mask = BMI160_FOC_CONF_MASK_ACC_Z;
			offset = BMI160_FOC_CONF_OFFSET_ACC_Z;
			break;
		case 'g':
		case 'G':
			mask = BMI160_FOC_CONF_MASK_GYRO;
			offset = BMI160_FOC_CONF_OFFSET_GYRO;
			break;
		}
		if (mask == 0)
			continue;
		if (i >= count - 1)
			break;
		conf &= ~mask;
		++i;
		switch (buf[i]) {
		case 'x': /* Set to disable */
		case 'X':
			conf |= BMI160_FOC_CONF_DISABLE << offset;
			break;
		case 'g': /* set to 1g, enable for gyro */
		case 'G':
			conf |= BMI160_FOC_CONF_1G << offset;
			break;
		case 'n': /* set to -1g */
		case 'N':
			if (offset == BMI160_FOC_CONF_OFFSET_GYRO)
				break;
			conf |= BMI160_FOC_CONF_N1G << offset;
			break;
		case '0': /* set to 0 */
			if (offset == BMI160_FOC_CONF_OFFSET_GYRO)
				break;
			conf |= BMI160_FOC_CONF_0 << offset;
			break;
		}
	}
	client_data->bmi160_foc_conf = conf;

	return count;
}

static ssize_t bhy_show_bmi160_foc_exec(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	return snprintf(buf, 64,
		"Use echo 1 > bmi160_foc_exec to begin foc\n");
}

static ssize_t bhy_store_bmi160_foc_exec(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	long req;
	int for_acc, for_gyro;
	int pmu_status_acc = 0, pmu_status_gyro = 0;
	u8 conf;
	u8 reg_data;
	int retry;
	int in[3], out[3], i;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtol(buf, 16, &req);
	if (ret < 0 || req != 1) {
		PERR("Invalid input");
		return -EINVAL;
	}
	conf = client_data->bmi160_foc_conf;

	/* Recalc acc conf according to real axis mapping */
	out[0] = (conf & BMI160_FOC_CONF_MASK_ACC_X) >>
		BMI160_FOC_CONF_OFFSET_ACC_X;
	out[1] = (conf & BMI160_FOC_CONF_MASK_ACC_Y) >>
		BMI160_FOC_CONF_OFFSET_ACC_Y;
	out[2] = (conf & BMI160_FOC_CONF_MASK_ACC_Z) >>
		BMI160_FOC_CONF_OFFSET_ACC_Z;
	for (i = 0; i < 3; ++i) {
		in[i] = out[0] * client_data->mapping_matrix_acc_inv[0][i] +
			out[1] * client_data->mapping_matrix_acc_inv[1][i] +
			out[2] * client_data->mapping_matrix_acc_inv[2][i];
		switch (in[i]) {
		case -1:
			in[i] = 2;
			break;
		case -2:
			in[i] = 1;
			break;
		case -3:
			in[i] = 3;
			break;
		default:
			break;
		}
	}
	conf &= ~BMI160_FOC_CONF_MASK_ACC_X;
	conf |= in[0] << BMI160_FOC_CONF_OFFSET_ACC_X;
	conf &= ~BMI160_FOC_CONF_MASK_ACC_Y;
	conf |= in[1] << BMI160_FOC_CONF_OFFSET_ACC_Y;
	conf &= ~BMI160_FOC_CONF_MASK_ACC_Z;
	conf |= in[2] << BMI160_FOC_CONF_OFFSET_ACC_Z;

	for_acc = (conf & 0x3F) ? 1 : 0;
	for_gyro = (conf & 0xC0) ? 1 : 0;
	if (for_acc == 0 && for_gyro == 0) {
		PERR("No need to do foc");
		return -EINVAL;
	}

	mutex_lock(&client_data->mutex_bus_op);
	/* Set acc range to 4g */
	if (for_acc) {
		ret = bmi160_read_reg(client_data, BMI160_REG_ACC_RANGE,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read acc range failed");
			return -EIO;
		}
		if ((reg_data & BMI160_ACC_RANGE_MASK) != BMI160_ACC_RANGE_4G) {
			reg_data = BMI160_ACC_RANGE_4G;
			ret = bmi160_write_reg(client_data,
				BMI160_REG_ACC_RANGE,
				&reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Write acc range failed");
				return -EIO;
			}
			retry = BMI160_OP_RETRY;
			do {
				ret = bmi160_read_reg(client_data,
					BMI160_REG_ACC_RANGE, &reg_data, 1);
				if (ret < 0) {
					mutex_unlock(
						&client_data->mutex_bus_op);
					PERR("Read acc range #2 failed");
					return -EIO;
				}
				if ((reg_data & BMI160_ACC_RANGE_MASK) ==
					BMI160_ACC_RANGE_4G)
					break;
				udelay(50);
			} while (--retry);
			if (retry == 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Wait for acc 4g range failed");
				return -EBUSY;
			}
		}
	}
	/* Set normal power mode */
	ret = bmi160_read_reg(client_data, BMI160_REG_PMU_STATUS,
		&reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read acc pmu status failed");
		return -EIO;
	}
	pmu_status_acc = (reg_data & BMI160_PMU_STATUS_MASK_ACC)
		>> BMI160_PMU_STATUS_OFFSET_ACC;
	pmu_status_gyro = (reg_data & BMI160_PMU_STATUS_MASK_GYRO)
		>> BMI160_PMU_STATUS_OFFSET_GYRO;
	if (for_acc && pmu_status_acc != BMI160_PMU_STATUS_NORMAL) {
		reg_data = BMI160_CMD_PMU_BASE_ACC + BMI160_PMU_STATUS_NORMAL;
		ret = bmi160_write_reg(client_data, BMI160_REG_CMD,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write acc pmu cmd failed");
			return -EIO;
		}
		retry = BMI160_OP_RETRY;
		do {
			ret = bmi160_read_reg(client_data,
				BMI160_REG_PMU_STATUS, &reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Read acc pmu status #2 failed");
				return -EIO;
			}
			reg_data = (reg_data & BMI160_PMU_STATUS_MASK_ACC)
				>> BMI160_PMU_STATUS_OFFSET_ACC;
			if (reg_data == BMI160_PMU_STATUS_NORMAL)
				break;
			udelay(50);
		} while (--retry);
		if (retry == 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Wait for acc normal mode status failed");
			return -EBUSY;
		}
	}
	if (for_gyro && pmu_status_gyro != BMI160_PMU_STATUS_NORMAL) {
		reg_data = BMI160_CMD_PMU_BASE_GYRO + BMI160_PMU_STATUS_NORMAL;
		ret = bmi160_write_reg(client_data, BMI160_REG_CMD,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write gyro pmu cmd failed");
			return -EIO;
		}
		retry = BMI160_OP_RETRY;
		do {
			ret = bmi160_read_reg(client_data,
				BMI160_REG_PMU_STATUS, &reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Read gyro pmu status #2 failed");
				return -EIO;
			}
			reg_data = (reg_data & BMI160_PMU_STATUS_MASK_GYRO)
				>> BMI160_PMU_STATUS_OFFSET_GYRO;
			if (reg_data == BMI160_PMU_STATUS_NORMAL)
				break;
			udelay(50);
		} while (--retry);
		if (retry == 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Wait for gyro normal mode status failed");
			return -EBUSY;
		}
	}
	/* Write offset enable bits */
	ret = bmi160_read_reg(client_data, BMI160_REG_OFFSET_6, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read offset config failed");
		return -EIO;
	}
	if (for_acc)
		reg_data |= BMI160_OFFSET_6_BIT_ACC_EN;
	if (for_gyro)
		reg_data |= BMI160_OFFSET_6_BIT_GYRO_EN;
	ret = bmi160_write_reg(client_data, BMI160_REG_OFFSET_6, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write offset enable failed");
		return ret;
	}
	/* Write configuration status */
	ret = bmi160_write_reg(client_data, BMI160_REG_FOC_CONF, &conf, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write configuration status failed");
		return ret;
	}
	/* Execute FOC command */
	reg_data = BMI160_CMD_START_FOC;
	ret = bmi160_write_reg(client_data, BMI160_REG_CMD, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Execute FOC failed");
		return ret;
	}
	reg_data = 0;
	retry = BMI160_OP_RETRY;
	do {
		ret = bmi160_read_reg(client_data, BMI160_REG_STATUS,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read status after exec FOC failed");
			return ret;
		}
		if (reg_data & BMI160_STATUS_BIT_FOC_RDY)
			break;
		usleep_range(2000, 2000);
	} while (--retry);
	if (retry == 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Cannot read the right status after exec FOC");
		return -EBUSY;
	}
	/* Restore old power mode */
	if (for_acc && pmu_status_acc != BMI160_PMU_STATUS_NORMAL) {
		reg_data = BMI160_CMD_PMU_BASE_ACC
			+ pmu_status_acc;
		ret = bmi160_write_reg(client_data, BMI160_REG_CMD,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write acc pmu cmd #2 failed");
			return -EIO;
		}
		retry = BMI160_OP_RETRY;
		do {
			ret = bmi160_read_reg(client_data,
				BMI160_REG_PMU_STATUS, &reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Read acc pmu status #2 failed");
				return -EIO;
			}
			reg_data = (reg_data & BMI160_PMU_STATUS_MASK_ACC)
				>> BMI160_PMU_STATUS_OFFSET_ACC;
			if (reg_data == pmu_status_acc)
				break;
			udelay(50);
		} while (--retry);
		if (retry == 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Wait for acc normal mode status #2 failed");
			return -EBUSY;
		}
	}
	if (for_gyro && pmu_status_gyro != BMI160_PMU_STATUS_NORMAL) {
		reg_data = BMI160_CMD_PMU_BASE_GYRO
			+ pmu_status_gyro;
		ret = bmi160_write_reg(client_data, BMI160_REG_CMD,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write gyro pmu cmd #2 failed");
			return -EIO;
		}
		retry = BMI160_OP_RETRY;
		do {
			ret = bmi160_read_reg(client_data,
				BMI160_REG_PMU_STATUS, &reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Read gyro pmu status #2 failed");
				return -EIO;
			}
			reg_data = (reg_data & BMI160_PMU_STATUS_MASK_GYRO)
				>> BMI160_PMU_STATUS_OFFSET_GYRO;
			if (reg_data == pmu_status_gyro)
				break;
			udelay(50);
		} while (--retry);
		if (retry == 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Wait for gyro normal mode status #2 failed");
			return -EBUSY;
		}
	}
	mutex_unlock(&client_data->mutex_bus_op);
	/* Reset foc conf*/
	client_data->bmi160_foc_conf = 0;

	PERR("FOC executed successfully");

	return count;
}

static ssize_t bhy_show_bmi160_foc_save_to_nvm(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	return snprintf(buf, 64,
		"Use echo 1 > bmi160_foc_save_to_nvm to save to nvm\n");
}

static ssize_t bhy_store_bmi160_foc_save_to_nvm(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	long req;
	u8 reg_data;
	int retry;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtol(buf, 16, &req);
	if (ret < 0 || req != 1) {
		PERR("Invalid input");
		return -EINVAL;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bmi160_read_reg(client_data, BMI160_REG_CONF, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read conf failed");
		return ret;
	}
	reg_data |= BMI160_CONF_BIT_NVM;
	ret = bmi160_write_reg(client_data, BMI160_REG_CONF, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Enable NVM writing failed");
		return ret;
	}
	reg_data = BMI160_CMD_PROG_NVM;
	ret = bmi160_write_reg(client_data, BMI160_REG_CMD, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Execute NVM prog failed");
		return ret;
	}
	reg_data = 0;
	retry = BMI160_OP_RETRY;
	do {
		ret = bmi160_read_reg(client_data, BMI160_REG_STATUS,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read status after exec FOC failed");
			return ret;
		}
		if (reg_data & BMI160_STATUS_BIT_NVM_RDY)
			break;
		usleep_range(2000, 2000);
	} while (--retry);
	if (retry == 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Cannot read the right status after write to NVM");
		return -EBUSY;
	}
	ret = bmi160_read_reg(client_data, BMI160_REG_CONF, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read conf after exec nvm prog failed");
		return ret;
	}
	reg_data &= ~BMI160_CONF_BIT_NVM;
	ret = bmi160_write_reg(client_data, BMI160_REG_CONF, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Disable NVM writing failed");
		return ret;
	}
	mutex_unlock(&client_data->mutex_bus_op);

	PINFO("NVM successfully written");

	return count;
}

static ssize_t bhy_show_bma2x2_foc_offset(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	s8 data[3];

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bma2x2_read_reg(client_data, BMA2X2_REG_OFC_OFFSET_X,
		(u8 *)data, sizeof(data));
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read hw reg failed");
		return ret;
	}

	return snprintf(buf, 64, "%11d %11d %11d\n", data[0], data[1], data[2]);
}

static ssize_t bhy_store_bma2x2_foc_offset(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int x, y, z;
	s8 data[3];
	int ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = sscanf(buf, "%11d %11d %11d", &x, &y, &z);
	if (ret != 3) {
		PERR("Invalid input");
		return -EINVAL;
	}
	data[0] = x & 0xFF;
	data[1] = y & 0xFF;
	data[2] = z & 0xFF;
	mutex_lock(&client_data->mutex_bus_op);
	ret = bma2x2_write_reg(client_data, BMA2X2_REG_OFC_OFFSET_X,
		(u8 *)data, sizeof(data));
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write hw reg failed");
		return ret;
	}

	return count;
}

static ssize_t bhy_show_bma2x2_foc_conf(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int x, y, z;
	int out[3], in[3], i;
	const char *disp[4] = {
		"disabled",
		"1g",
		"-1g",
		"0"
	};
	u8 conf;
	ssize_t ret = 0;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	conf = client_data->bma2x2_foc_conf;

	x = (conf & BMA2X2_OFC_CONF_MASK_X) >> BMA2X2_OFC_CONF_OFFSET_X;
	y = (conf & BMA2X2_OFC_CONF_MASK_Y) >> BMA2X2_OFC_CONF_OFFSET_Y;
	z = (conf & BMA2X2_OFC_CONF_MASK_Z) >> BMA2X2_OFC_CONF_OFFSET_Z;

	out[0] = x;
	out[1] = y;
	out[2] = z;
	for (i = 0; i < 3; ++i) {
		in[i] = out[0] * client_data->mapping_matrix_acc_inv[0][i] +
			out[1] * client_data->mapping_matrix_acc_inv[1][i] +
			out[2] * client_data->mapping_matrix_acc_inv[2][i];
		switch (in[i]) {
		case -1:
			in[i] = 2;
			break;
		case -2:
			in[i] = 1;
			break;
		case -3:
			in[i] = 3;
			break;
		default:
			break;
		}
	}

	ret += snprintf(buf + ret, 128, "Acc conf: %s %s %s\n",
		disp[x], disp[y], disp[z]);
	ret += snprintf(buf + ret, 128, "Original acc conf: %s %s %s\n",
		disp[in[0]], disp[in[1]], disp[in[2]]);

	return ret;
}

static ssize_t bhy_store_bma2x2_foc_conf(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int i;
	int mask, offset;
	u8 conf = 0;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	for (i = 0; i < count; ++i) {
		mask = 0;
		switch (buf[i]) {
		case 'x':
		case 'X':
			mask = BMA2X2_OFC_CONF_MASK_X;
			offset = BMA2X2_OFC_CONF_OFFSET_X;
			break;
		case 'y':
		case 'Y':
			mask = BMA2X2_OFC_CONF_MASK_Y;
			offset = BMA2X2_OFC_CONF_OFFSET_Y;
			break;
		case 'z':
		case 'Z':
			mask = BMA2X2_OFC_CONF_MASK_Z;
			offset = BMA2X2_OFC_CONF_OFFSET_Z;
			break;
		}
		if (mask == 0)
			continue;
		if (i >= count - 1)
			break;
		conf &= ~mask;
		++i;
		switch (buf[i]) {
		case 'x': /* Set to disable */
		case 'X':
			conf |= BMA2X2_OFC_CONF_DISABLE << offset;
			break;
		case 'g': /* set to 1g, enable for gyro */
		case 'G':
			conf |= BMA2X2_OFC_CONF_1G << offset;
			break;
		case 'n': /* set to -1g */
		case 'N':
			conf |= BMA2X2_OFC_CONF_N1G << offset;
			break;
		case '0': /* set to 0 */
			conf |= BMA2X2_OFC_CONF_0 << offset;
			break;
		}
	}
	client_data->bma2x2_foc_conf = conf;

	return count;
}

static ssize_t bhy_show_bma2x2_foc_exec(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	return snprintf(buf, 64,
		"Use echo 1 > bma2x2_foc_exec to begin foc\n");
}

static ssize_t bhy_store_bma2x2_foc_exec(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	long req;
	u8 pmu_status_old;
	u8 conf;
	u8 reg_data;
	int retry;
	int in[3], out[3], i;
	u8 trigger_axis[3] = {
		BMA2X2_CAL_TRIGGER_X,
		BMA2X2_CAL_TRIGGER_Y,
		BMA2X2_CAL_TRIGGER_Z
	};

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtol(buf, 16, &req);
	if (ret < 0 || req != 1) {
		PERR("Invalid input");
		return -EINVAL;
	}
	conf = client_data->bma2x2_foc_conf;

	/* Recalc acc conf according to real axis mapping */
	out[0] = (conf & BMA2X2_OFC_CONF_MASK_X) >>
		BMA2X2_OFC_CONF_OFFSET_X;
	out[1] = (conf & BMA2X2_OFC_CONF_MASK_Y) >>
		BMA2X2_OFC_CONF_OFFSET_Y;
	out[2] = (conf & BMA2X2_OFC_CONF_MASK_Z) >>
		BMA2X2_OFC_CONF_OFFSET_Z;
	for (i = 0; i < 3; ++i) {
		in[i] = out[0] * client_data->mapping_matrix_acc_inv[0][i] +
			out[1] * client_data->mapping_matrix_acc_inv[1][i] +
			out[2] * client_data->mapping_matrix_acc_inv[2][i];
		switch (in[i]) {
		case -1:
			in[i] = 2;
			break;
		case -2:
			in[i] = 1;
			break;
		case -3:
			in[i] = 3;
			break;
		default:
			break;
		}
	}
	conf &= ~BMA2X2_OFC_CONF_MASK_X;
	conf |= in[0] << BMA2X2_OFC_CONF_OFFSET_X;
	conf &= ~BMA2X2_OFC_CONF_MASK_Y;
	conf |= in[1] << BMA2X2_OFC_CONF_OFFSET_Y;
	conf &= ~BMA2X2_OFC_CONF_MASK_Z;
	conf |= in[2] << BMA2X2_OFC_CONF_OFFSET_Z;

	/* Set 2g range */
	mutex_lock(&client_data->mutex_bus_op);
	ret = bma2x2_read_reg(client_data, BMA2X2_REG_PMU_RANGE,
		&reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read acc pmu range failed");
		return -EIO;
	}
	if (reg_data != BMA2X2_PMU_RANGE_2G) {
		reg_data = BMA2X2_PMU_RANGE_2G;
		ret = bma2x2_write_reg(client_data, BMA2X2_REG_PMU_RANGE,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write acc pmu range failed");
			return -EIO;
		}
		retry = BMA2X2_OP_RETRY;
		do {
			ret = bma2x2_read_reg(client_data, BMA2X2_REG_PMU_RANGE,
				&reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Read acc pmu range failed");
				return -EIO;
			}
			if (reg_data == BMA2X2_PMU_RANGE_2G)
				break;
			udelay(50);
		} while (--retry);
		if (retry == 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Wait for 4g range failed");
			return -EBUSY;
		}
	}
	mutex_unlock(&client_data->mutex_bus_op);

	/* Set BW to 62.5Hz if it is greater than 125Hz */
	mutex_lock(&client_data->mutex_bus_op);
	ret = bma2x2_read_reg(client_data, BMA2X2_REG_PMU_BW,
		&reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read acc pmu range failed");
		return -EIO;
	}
	if (reg_data > BMA2X2_PMU_BW_125) {
		reg_data = BMA2X2_PMU_BW_62_5;
		ret = bma2x2_write_reg(client_data, BMA2X2_REG_PMU_BW,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write acc pmu bw failed");
			return -EIO;
		}
		retry = BMA2X2_OP_RETRY;
		do {
			ret = bma2x2_read_reg(client_data, BMA2X2_REG_PMU_BW,
				&reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Read acc pmu range failed");
				return -EIO;
			}
			if (reg_data == BMA2X2_PMU_BW_62_5)
				break;
			udelay(50);
		} while (--retry);
		if (retry == 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Wait for 62.5Hz BW failed");
			return -EBUSY;
		}
	}
	mutex_unlock(&client_data->mutex_bus_op);

	/* Set normal power mode */
	mutex_lock(&client_data->mutex_bus_op);
	ret = bma2x2_read_reg(client_data, BMA2X2_REG_PMU_LPW,
		&reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read acc pmu status failed");
		return -EIO;
	}
	pmu_status_old = reg_data;
	reg_data &= BMA2X2_PMU_CONF_MASK;
	if (reg_data != BMA2X2_PMU_CONF_NORMAL) {
		reg_data = BMA2X2_PMU_CONF_NORMAL;
		ret = bma2x2_write_reg(client_data, BMA2X2_REG_PMU_LPW,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write acc pmu cmd failed");
			return -EIO;
		}
		retry = BMA2X2_OP_RETRY;
		do {
			ret = bma2x2_read_reg(client_data,
				BMA2X2_REG_PMU_LPW, &reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Read acc pmu status #2 failed");
				return -EIO;
			}
			reg_data &= BMA2X2_PMU_CONF_MASK;
			if (reg_data == BMA2X2_PMU_CONF_NORMAL)
				break;
			udelay(50);
		} while (--retry);
		if (retry == 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Wait for acc normal mode status failed");
			return -EBUSY;
		}
	}
	/* Write configuration status */
	ret = bma2x2_write_reg(client_data, BMA2X2_REG_OFC_SETTING, &conf, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write configuration status failed");
		return ret;
	}
	/* Execute FOC command */
	ret = bma2x2_read_reg(client_data,
		BMA2X2_REG_OFC_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read ofc_ctrl failed");
		return -EIO;
	}
	if ((reg_data & BMA2X2_CAL_RDY_MASK) == 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("OFC cal rdy status error!");
		return -EIO;
	}
	for (i = 0; i < 3; ++i) {
		if (in[i] == 0) /* disabled */
			continue;
		reg_data = trigger_axis[i];
		ret = bma2x2_write_reg(client_data, BMA2X2_REG_OFC_CTRL,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Execute FOC failed");
			return ret;
		}
		reg_data = 0;
		retry = BMA2X2_OP_RETRY;
		do {
			ret = bma2x2_read_reg(client_data,
				BMA2X2_REG_OFC_CTRL, &reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Read ofc_ctrl failed");
				return -EIO;
			}
			if (reg_data & BMA2X2_CAL_RDY_MASK)
				break;
			usleep_range(2000, 2000);
		} while (--retry);
		if (retry == 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Cannot read the right status after exec FOC");
			return -EBUSY;
		}
	}
	/* Restore old power mode */
	reg_data = pmu_status_old;
	reg_data &= BMA2X2_PMU_CONF_MASK;
	if (reg_data != BMA2X2_PMU_CONF_NORMAL) {
		reg_data = pmu_status_old;
		ret = bma2x2_write_reg(client_data, BMA2X2_REG_PMU_LPW,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write acc pmu cmd #2 failed");
			return -EIO;
		}
		retry = BMA2X2_OP_RETRY;
		do {
			ret = bma2x2_read_reg(client_data,
				BMA2X2_REG_PMU_LPW, &reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Read acc pmu status #2 failed");
				return -EIO;
			}
			if (reg_data == pmu_status_old)
				break;
			udelay(50);
		} while (--retry);
		if (retry == 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Wait for acc normal mode status #2 failed");
			return -EBUSY;
		}
	}
	mutex_unlock(&client_data->mutex_bus_op);

	/* Restore 4g range */
	mutex_lock(&client_data->mutex_bus_op);
	ret = bma2x2_read_reg(client_data, BMA2X2_REG_PMU_RANGE,
		&reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read acc pmu range failed");
		return -EIO;
	}
	if (reg_data != BMA2X2_PMU_RANGE_4G) {
		reg_data = BMA2X2_PMU_RANGE_4G;
		ret = bma2x2_write_reg(client_data, BMA2X2_REG_PMU_RANGE,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Write acc pmu range failed");
			return -EIO;
		}
		retry = BMA2X2_OP_RETRY;
		do {
			ret = bma2x2_read_reg(client_data, BMA2X2_REG_PMU_RANGE,
				&reg_data, 1);
			if (ret < 0) {
				mutex_unlock(&client_data->mutex_bus_op);
				PERR("Read acc pmu range failed");
				return -EIO;
			}
			if (reg_data == BMA2X2_PMU_RANGE_4G)
				break;
			udelay(50);
		} while (--retry);
		if (retry == 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Wait for 4g range failed");
			return -EBUSY;
		}
	}
	mutex_unlock(&client_data->mutex_bus_op);

	/* Reset foc conf*/
	client_data->bma2x2_foc_conf = 0;

	PINFO("FOC executed successfully");

	return count;
}

static ssize_t bhy_show_self_test(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	return snprintf(buf, 64,
		"Use echo 1 > self_test to do self-test\n");
}

static ssize_t bhy_store_self_test(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	long req;
	u8 reg_data;
	int retry;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtol(buf, 16, &req);
	if (ret < 0 || req != 1) {
		PERR("Invalid input");
		return -EINVAL;
	}

	atomic_set(&client_data->reset_flag, RESET_FLAG_SELF_TEST);

	mutex_lock(&client_data->mutex_bus_op);
	/* Disable CPU run from chip control */
	reg_data = 0;
	ret = bhy_write_reg(client_data, BHY_REG_CHIP_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write chip control reg failed");
		return -EIO;
	}
	retry = 1000;
	do {
		ret = bhy_read_reg(client_data, BHY_REG_CHIP_STATUS,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read chip status failed");
			return -EIO;
		}
		if (reg_data & BHY_CHIP_STATUS_BIT_FIRMWARE_IDLE)
			break;
		usleep_range(10000, 10000);
	} while (--retry);
	if (retry == 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Wait for chip idle status timed out");
		return -EBUSY;
	}
	/* Write self test bit */
	ret = bhy_read_reg(client_data, BHY_REG_HOST_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read host ctrl reg failed");
		return -EIO;
	}
	reg_data |= HOST_CTRL_MASK_SELF_TEST_REQ;
	/*reg_data &= ~HOST_CTRL_MASK_ALGORITHM_STANDBY;*/
	ret = bhy_write_reg(client_data, BHY_REG_HOST_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write host ctrl reg failed");
		return -EIO;
	}
	/* Enable CPU run from chip control */
	reg_data = 1;
	ret = bhy_write_reg(client_data, BHY_REG_CHIP_CTRL, &reg_data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write chip control reg failed");
		return -EIO;
	}
	retry = 1000;
	do {
		ret = bhy_read_reg(client_data, BHY_REG_CHIP_STATUS,
			&reg_data, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Read chip status failed");
			return -EIO;
		}
		if (!(reg_data & BHY_CHIP_STATUS_BIT_FIRMWARE_IDLE))
			break;
		usleep_range(10000, 10000);
	} while (--retry);
	if (retry == 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Wait for chip running status timed out");
		return -EBUSY;
	}
	mutex_unlock(&client_data->mutex_bus_op);

	return count;
}

static ssize_t bhy_show_self_test_result(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int i, handle, counts;
	ssize_t ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = 0;
	counts = 0;
	for (i = 0; i < PHYSICAL_SENSOR_COUNT; ++i) {
		if (client_data->self_test_result[i] != -1) {
			switch (i) {
			case PHYSICAL_SENSOR_INDEX_ACC:
				handle = BHY_PHYS_HANDLE_ACC;
				break;
			case PHYSICAL_SENSOR_INDEX_MAG:
				handle = BHY_PHYS_HANDLE_MAG;
				break;
			case PHYSICAL_SENSOR_INDEX_GYRO:
				handle = BHY_PHYS_HANDLE_GYRO;
				break;
			}
			ret += snprintf(buf + ret, 64,
				"Result for sensor[%d]: %d\n",
				handle, client_data->self_test_result[i]);
			++counts;
		}
	}
	ret += snprintf(buf + ret, 64, "Totally %d sensor(s) tested.\n", counts);

	return ret;
}

static ssize_t bhy_store_update_device_info(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	int i;
	u8 id[4];

	/* Set device type */
	for (i = 0; i < sizeof(client_data->dev_type) - 1 && buf[i]; ++i)
		client_data->dev_type[i] = buf[i];
	client_data->dev_type[i] = '\0';
	/* Set rom & ram ID */
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, BHY_REG_ROM_VERSION_0, id, 4);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read rom id failed");
		return -EIO;
	}
	client_data->rom_id = *(u16 *)id;
	client_data->ram_id = *((u16 *)id + 1);

	return count;
}

static ssize_t bhy_show_mapping_matrix_acc(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int i, j;
	ssize_t ret = 0;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret += snprintf(buf + ret, 64, "Matrix:\n");
	for (i = 0; i < 3; ++i) {
		for (j = 0; j < 3; ++j)
			ret += snprintf(buf + ret, 16, "%d ",
			client_data->mapping_matrix_acc[i][j]);
		buf[ret++] = '\n';
	}

	ret += snprintf(buf + ret, 64, "Inverse:\n");
	for (i = 0; i < 3; ++i) {
		for (j = 0; j < 3; ++j)
			ret += snprintf(buf + ret, 16, "%d ",
			client_data->mapping_matrix_acc_inv[i][j]);
		buf[ret++] = '\n';
	}
	buf[ret++] = '\0';

	return ret;
}

static ssize_t bhy_store_mapping_matrix_acc(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	long req;
	u8 data[16];
	int i, j, k;
	s8 m[3][6], tmp;

	GSE_FUN();

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtol(buf, 16, &req);
	if (ret < 0 || req != 1) {
		PERR("Invalid input");
		return -EINVAL;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_SYSTEM,
		BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_DETAIL_ACC,
		data, sizeof(data));
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read param failed");
		return ret;
	}
	for (i = 0; i < 3; ++i)
		for (j = 0; j < 3; ++j) {
			k = i * 3 + j;
			client_data->mapping_matrix_acc[i][j] =
				k % 2 == 0 ? data[11 + k / 2] & 0xF :
				data[11 + k / 2] >> 4;
			if (client_data->mapping_matrix_acc[i][j] == 0xF)
				client_data->mapping_matrix_acc[i][j] = -1;
		}

	for (i = 0; i < 3; ++i)
		for (j = 0; j < 3; ++j) {
			m[i][j] = client_data->mapping_matrix_acc[i][j];
			m[i][j + 3] = i == j ? 1 : 0;
		}
	for (i = 0; i < 3; ++i) {
		if (m[i][i] == 0) {
			for (j = i + 1; j < 3; ++j) {
				if (m[j][i]) {
					for (k = 0; k < 6; ++k) {
						tmp = m[j][k];
						m[j][k] = m[i][k];
						m[i][k] = tmp;
					}
					break;
				}
			}
			if (j >= 3) { /* Fail check */
				PERR("Matrix invalid");
				break;
			}
		}
		if (m[i][i] < 0) {
			for (j = 0; j < 6; ++j)
				m[i][j] = -m[i][j];
		}
	}

	for (i = 0; i < 3; ++i)
		for (j = 0; j < 3; ++j)
			client_data->mapping_matrix_acc_inv[i][j] = m[i][j + 3];

	return count;
}

static ssize_t bhy_store_sensor_data_size(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int handle;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	handle = (u8)buf[0];
	if (handle >= 255) {
		PERR("Invalid handle");
		return -EIO;
	}
	client_data->sensor_context[handle].data_len = (s8)buf[1];

	return count;
}

static ssize_t bhy_show_mapping_matrix(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int i, j, k, ii;
	int ret;
	ssize_t len = 0;
	u8 data[16];
	u8 map[32];
	u8 handle[3] = {
		BHY_SENSOR_HANDLE_ACCELEROMETER,
		BHY_SENSOR_HANDLE_MAGNETIC_FIELD_UNCALIBRATED,
		BHY_SENSOR_HANDLE_GYROSCOPE_UNCALIBRATED,
	};
	char *name[3] = {
		"Accelerometer",
		"Magnetometer",
		"Gyroscope"
	};
	u8 param;
	s8 mapping[3][3];

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	/* Check sensor existance */
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_SYSTEM,
		BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_PRESENT,
		data, sizeof(data));
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read param failed");
		return ret;
	}
	for (i = 0; i < 4; ++i) {
		for (j = 0; j < 8; ++j) {
			if (data[i] & (1 << j))
				map[i * 8 + j] = 1;
			else
				map[i * 8 + j] = 0;
		}
	}

	/* Get orientation matrix */
	for (ii = 0; ii < 3; ++ii) {
		if (!map[handle[ii]])
			continue;
		param = BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_DETAIL_0 + handle[ii];
		mutex_lock(&client_data->mutex_bus_op);
		ret = bhy_read_parameter(client_data, BHY_PAGE_SYSTEM,
			param, data, sizeof(data));
		mutex_unlock(&client_data->mutex_bus_op);
		if (ret < 0) {
			PERR("Read param failed #2");
			return ret;
		}
		for (i = 0; i < 3; ++i)
			for (j = 0; j < 3; ++j) {
				k = i * 3 + j;
				mapping[i][j] =
					k % 2 == 0 ? data[11 + k / 2] & 0xF :
					data[11 + k / 2] >> 4;
				if (mapping[i][j] == 0xF)
					mapping[i][j] = -1;
			}
		len += snprintf(buf + len, 128, "Matrix for %s:\n", name[ii]);
		for (i = 0; i < 3; ++i) {
			for (j = 0; j < 3; ++j)
				len += snprintf(buf + len, 16, "%d ",
					mapping[i][j]);
			buf[len++] = '\n';
		}
		buf[len++] = '\n';
		buf[len++] = '\0';
	}

	return len;
}

static ssize_t bhy_store_mapping_matrix(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int i;
	int handle;
	int matrix[9];
	int ret;
	int index;

/*
	struct hwmsen_convert map[] = {
		{ { 1, 1, 1}, {0, 1, 2} },
		{ {-1, 1, 1}, {1, 0, 2} },
		{ {-1, -1, 1}, {0, 1, 2} },
		{ { 1, -1, 1}, {1, 0, 2} },
	
		{ {-1, 1, -1}, {0, 1, 2} },
		{ { 1, 1, -1}, {1, 0, 2} },
		{ { 1, -1, -1}, {0, 1, 2} },
		{ {-1, -1, -1}, {1, 0, 2} },

struct hwmsen_convert {
	s8 sign[C_MAX_HWMSEN_EVENT_NUM];
	u8 map[C_MAX_HWMSEN_EVENT_NUM];
};

		
*/

	GSE_FUN();

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = sscanf(buf, "%11d %11d %11d %11d %11d %11d %11d %11d %11d %11d",
		&handle, &matrix[0], &matrix[1], &matrix[2], &matrix[3],
		&matrix[4], &matrix[5], &matrix[6], &matrix[7], &matrix[8]);
	if (ret != 10) {
		PERR("Invalid input for matrix");
		return -EINVAL;
	}

	for (i = 0; i < 9; ++i) {
		if (matrix[i] < -1 || matrix[i] > 1) {
			PERR("Invalid matrix data: %d", matrix[i]);
			return -EINVAL;
		}
	}

	switch (handle) {
	case BHY_SENSOR_HANDLE_ACCELEROMETER:
		index = PHYSICAL_SENSOR_INDEX_ACC;
		break;
	case BHY_SENSOR_HANDLE_MAGNETIC_FIELD_UNCALIBRATED:
		index = PHYSICAL_SENSOR_INDEX_MAG;
		break;
	case BHY_SENSOR_HANDLE_GYROSCOPE_UNCALIBRATED:
		index = PHYSICAL_SENSOR_INDEX_GYRO;
		break;
	default:
		PERR("Invalid sensor handle: %d", handle);
		return -EINVAL;
	}
	for (i = 0; i < 9; ++i)
		client_data->ps_context[index].mapping_matrix[i] = matrix[i];

	if (bhy_set_mapping_matrix(client_data, index) < 0)
		return -EIO;

	client_data->ps_context[index].use_mapping_matrix = BHY_TRUE;

	return count;
}

static ssize_t bhy_show_custom_version(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	int ret;
	ssize_t len = 0;
	u8 data[16];
	int custom_version;
	int year, month, day, hour, minute, second;
	__le16 reg_data;
	int rom_id, ram_id;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, BHY_REG_ROM_VERSION_0,
		(u8 *)&reg_data, 2);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read rom version failed");
		return ret;
	}
	rom_id = (u16)__le16_to_cpu(reg_data);
	len += snprintf(buf + len, 64, "Rom version: %d\n", rom_id);

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, BHY_REG_RAM_VERSION_0,
		(u8 *)&reg_data, 2);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read ram version failed");
		return ret;
	}
	ram_id = (u16)__le16_to_cpu(reg_data);
	len += snprintf(buf + len, 64, "Ram version: %d\n", ram_id);

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_SYSTEM,
		BHY_PARAM_SYSTEM_CUSTOM_VERSION, data, sizeof(data));
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read custom version failed");
		return ret;
	}

	reg_data = *(__le16 *)data;
	custom_version = (u16)__le16_to_cpu(reg_data);
	reg_data = *(__le16 *)(data + 2);
	year = (u16)__le16_to_cpu(reg_data);
	month = *(u8 *)(data + 4);
	day = *(u8 *)(data + 5);
	hour = *(u8 *)(data + 6);
	minute = *(u8 *)(data + 7);
	second = *(u8 *)(data + 8);

	len += snprintf(buf + len, 64, "Custom version: %d\n", custom_version);
	len += snprintf(buf + len, 64,
		"Build date: %04d-%02d-%02d %02d:%02d:%02d\n",
		year, month, day, hour, minute, second);

	return len;
}

static ssize_t bhy_store_req_fw(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	long req;
	int ret;
	int retry_count = 0;
	ACC_LOG("bhy_store_req_fw start.");

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = kstrtol(buf, 10, &req);
	if (ret < 0 || req != 1) {
		PERR("Invalid request");
		atomic_set(&client_data->reset_flag, RESET_FLAG_ERROR);
		return -EINVAL;
	}
#if 0
	atomic_set(&client_data->reset_flag, RESET_FLAG_TODO);
	ret = bhy_request_firmware(client_data);
	if (ret < 0)
		return ret;
#else
	ret = -1;
	while (ret < 0) {
		ret = bhy_request_firmware(client_data);
		retry_count++;
		printk("send fw %d times, ret = %d.\n", retry_count, ret);
		if(retry_count > 3 && ret < 0) {
			return ret;
		}
	}
#endif
	return count;
}

#ifdef BHY_DEBUG

static ssize_t bhy_show_reg_sel(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	return snprintf(buf, 64, "reg=0X%02X, len=%d\n",
		client_data->reg_sel, client_data->reg_len);
}

static ssize_t bhy_store_reg_sel(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = sscanf(buf, "%11X %11d",
		&client_data->reg_sel, &client_data->reg_len);
	if (ret != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	return count;
}

static ssize_t bhy_show_reg_val(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	u8 reg_data[128], i;
	int pos;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	if (client_data->reg_len <= 0) {
		PERR("Invalid register length");
		return -EINVAL;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, client_data->reg_sel,
		reg_data, client_data->reg_len);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Reg op failed");
		return ret;
	}

	pos = 0;
	for (i = 0; i < client_data->reg_len; ++i) {
		pos += snprintf(buf + pos, 16, "%02X", reg_data[i]);
		buf[pos++] = (i + 1) % 16 == 0 ? '\n' : ' ';
	}
	if (buf[pos - 1] == ' ')
		buf[pos - 1] = '\n';

	return pos;
}

static ssize_t bhy_store_reg_val(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	u8 reg_data[32] = { 0, };
	int i, j, status, digit;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	if (client_data->reg_len <= 0) {
		PERR("Invalid register length");
		return -EINVAL;
	}

	status = 0;
	for (i = j = 0; i < count && j < client_data->reg_len; ++i) {
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\t' ||
			buf[i] == '\r') {
			status = 0;
			++j;
			continue;
		}
		digit = buf[i] & 0x10 ? (buf[i] & 0xF) : ((buf[i] & 0xF) + 9);
		/*PDEBUG("digit is %d", digit);*/
		switch (status) {
		case 2:
			++j; /* Fall thru */
			if (j >= (int)sizeof(reg_data))
				break;
		case 0:
			reg_data[j] = digit;
			status = 1;
			break;
		case 1:
			reg_data[j] = reg_data[j] * 16 + digit;
			status = 2;
			break;
		}
	}
	if (status > 0)
		++j;
	if (j > client_data->reg_len)
		j = client_data->reg_len;
	else if (j < client_data->reg_len) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	/*PDEBUG("Reg data read as");*/
	for (i = 0; i < j; ++i)
		PDEBUG("%d", reg_data[i]);

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_reg(client_data, client_data->reg_sel,
		reg_data, client_data->reg_len);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Reg op failed");
		return ret;
	}

	return count;
}

static ssize_t bhy_show_param_sel(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	return snprintf(buf, 64, "Page=%d, param=%d\n",
		client_data->page_sel, client_data->param_sel);
}

static ssize_t bhy_store_param_sel(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = sscanf(buf, "%11d %11d",
		&client_data->page_sel, &client_data->param_sel);
	if (ret != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	return count;
}

static ssize_t bhy_show_param_val(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	u8 data[16];
	int pos, i;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, client_data->page_sel,
		client_data->param_sel, data, sizeof(data));
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read param failed");
		return ret;
	}

	pos = 0;
	for (i = 0; i < 16; ++i) {
		pos += snprintf(buf + pos, 16, "%02X", data[i]);
		buf[pos++] = ' ';
	}
	if (buf[pos - 1] == ' ')
		buf[pos - 1] = '\n';

	return pos;
}

static ssize_t bhy_store_param_val(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	u8 data[8];
	int i, j, status, digit;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	status = 0;
	for (i = j = 0; i < count && j < 8; ++i) {
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\t' ||
			buf[i] == '\r') {
			status = 0;
			++j;
			continue;
		}
		digit = buf[i] & 0x10 ? (buf[i] & 0xF) : ((buf[i] & 0xF) + 9);
		switch (status) {
		case 2:
			++j; /* Fall thru */
			if (j >= (int)sizeof(data))
				break;
		case 0:
			data[j] = digit;
			status = 1;
			break;
		case 1:
			data[j] = data[j] * 16 + digit;
			status = 2;
			break;
		}
	}
	if (status > 0)
		++j;
	if (j == 0) {
		PERR("Invalid argument");
		return -EINVAL;
	} else if (j > 8)
		j = 8;
	/* Alway write 8 bytes, the bytes is 0 if not provided*/
	for (i = j; i < 8; ++i)
		data[i] = 0;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, client_data->page_sel,
		client_data->param_sel, data, 8);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write param failed");
		return ret;
	}

	return count;
}

static ssize_t bhy_store_log_raw_data(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	long req;
	u8 param_data[8];
	struct frame_queue *q;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtol(buf, 10, &req);
	if (ret < 0) {
		PERR("Invalid request");
		return -EINVAL;
	}
	q = &client_data->data_queue;

	memset(param_data, 0, sizeof(param_data));
	if (req)
		param_data[0] = param_data[1] = param_data[2] = 1;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_ALGORITHM,
			BHY_PARAM_VIRTUAL_BSX_ENABLE, param_data, 8);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write raw data cfg failed");
		return ret;
	}

	mutex_lock(&q->lock);
	q->frames[q->head].handle = BHY_SENSOR_HANDLE_DATA_LOG_TYPE;
	q->frames[q->head].data[0] = BHY_DATA_LOG_TYPE_RAW;
	bhy_advance_fifo_queue_head(q);
	mutex_unlock(&q->lock);

	input_event(client_data->input, EV_MSC, MSC_RAW, 0);
	input_sync(client_data->input);

	return count;
}

static ssize_t bhy_store_log_input_data_gesture(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	long req;
	u8 param_data[8];
	struct frame_queue *q;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtol(buf, 10, &req);
	if (ret < 0) {
		PERR("Invalid request");
		return -EINVAL;
	}
	q = &client_data->data_queue;

	memset(param_data, 0, sizeof param_data);
	if (req)
		param_data[3] = param_data[4] = param_data[5] = 1;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_ALGORITHM,
			BHY_PARAM_VIRTUAL_BSX_ENABLE, param_data, 8);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write raw data cfg failed");
		return ret;
	}

	mutex_lock(&q->lock);
	q->frames[q->head].handle = BHY_SENSOR_HANDLE_DATA_LOG_TYPE;
	q->frames[q->head].data[0] = BHY_DATA_LOG_TYPE_INPUT_GESTURE;
	bhy_advance_fifo_queue_head(q);
	mutex_unlock(&q->lock);

	input_event(client_data->input, EV_MSC, MSC_RAW, 0);
	input_sync(client_data->input);

	return count;
}

static ssize_t bhy_store_log_input_data_tilt_ar(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	long req;
	u8 param_data[8];
	struct frame_queue *q;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtol(buf, 10, &req);
	if (ret < 0) {
		PERR("Invalid request");
		return -EINVAL;
	}
	q = &client_data->data_queue;

	memset(param_data, 0, sizeof param_data);
	if (req)
		param_data[6] = param_data[7] = 1;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_ALGORITHM,
			BHY_PARAM_VIRTUAL_BSX_ENABLE, param_data, 8);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write raw data cfg failed");
		return ret;
	}

	mutex_lock(&q->lock);
	q->frames[q->head].handle = BHY_SENSOR_HANDLE_DATA_LOG_TYPE;
	q->frames[q->head].data[0] = BHY_DATA_LOG_TYPE_INPUT_TILT_AR;
	bhy_advance_fifo_queue_head(q);
	mutex_unlock(&q->lock);

	input_event(client_data->input, EV_MSC, MSC_RAW, 0);
	input_sync(client_data->input);

	return count;
}

static ssize_t bhy_store_log_fusion_data(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	long req;
	struct frame_queue *q;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtol(buf, 10, &req);
	if (ret < 0) {
		PERR("Invalid request");
		return -EINVAL;
	}
	q = &client_data->data_queue;

	mutex_lock(&q->lock);
	q->frames[q->head].handle = BHY_SENSOR_HANDLE_LOG_FUSION_DATA;
	q->frames[q->head].data[0] = req ? BHY_FUSION_DATA_LOG_ENABLE :
		BHY_FUSION_DATA_LOG_NONE;
	bhy_advance_fifo_queue_head(q);
	mutex_unlock(&q->lock);

	input_event(client_data->input, EV_MSC, MSC_RAW, 0);
	input_sync(client_data->input);

	return count;
}

static ssize_t bhy_store_enable_pass_thru(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	u8 u8_val;
	int enable;
	int retry;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtoint(buf, 10, &enable);
	if (ret < 0) {
		PERR("invalid input");
		return ret;
	}

	mutex_lock(&client_data->mutex_bus_op);

	if (enable) {
		/* Make algorithm standby */
		ret = bhy_read_reg(client_data, BHY_REG_HOST_CTRL, &u8_val, 1);
		if (ret < 0) {
			PERR("Read algorithm standby reg failed");
			goto _exit;
		}
		u8_val |= HOST_CTRL_MASK_ALGORITHM_STANDBY;
		ret = bhy_write_reg(client_data, BHY_REG_HOST_CTRL, &u8_val, 1);
		if (ret < 0) {
			PERR("Write algorithm standby reg failed");
			goto _exit;
		}
		retry = 10;
		do {
			ret = bhy_read_reg(client_data, BHY_REG_HOST_STATUS,
					&u8_val, 1);
			if (ret < 0) {
				PERR("Read host status again failed");
				goto _exit;
			}
			if (u8_val & BHY_HOST_STATUS_MASK_ALGO_STANDBY)
				break;
			msleep(1000);
		} while (--retry);
		if (retry == 0) {
			ret = -EIO;
			PERR("Algo standby does not take effect");
			goto _exit;
		}

		/* Enable pass thru mode */
		u8_val = 1;
		ret = bhy_write_reg(client_data, BHY_REG_PASS_THRU_CFG,
				&u8_val, 1);
		if (ret < 0) {
			PERR("Write pass thru cfg reg failed");
			goto _exit;
		}
		retry = 1000;
		do {
			ret = bhy_read_reg(client_data, BHY_REG_PASS_THRU_READY,
					&u8_val, 1);
			if (ret < 0) {
				PERR("Read pass thru ready reg failed");
				goto _exit;
			}
			if (u8_val & 1)
				break;
			usleep_range(1000, 1000);
		} while (--retry);
		if (retry == 0) {
			ret = -EIO;
			PERR("Pass thru does not take effect");
			goto _exit;
		}
	} else {
		/* Disable pass thru mode */
		u8_val = 0;
		ret = bhy_write_reg(client_data, BHY_REG_PASS_THRU_CFG,
				&u8_val, 1);
		if (ret < 0) {
			PERR("Write pass thru cfg reg failed");
			goto _exit;
		}
		retry = 1000;
		do {
			ret = bhy_read_reg(client_data, BHY_REG_PASS_THRU_READY,
					&u8_val, 1);
			if (ret < 0) {
				PERR("Read pass thru ready reg failed");
				goto _exit;
			}
			if (!(u8_val & 1))
				break;
			usleep_range(1000, 1000);
		} while (--retry);
		if (retry == 0) {
			ret = -EIO;
			PERR("Pass thru disable does not take effect");
			goto _exit;
		}

		/* Make algorithm standby */
		ret = bhy_read_reg(client_data, BHY_REG_HOST_CTRL, &u8_val, 1);
		if (ret < 0) {
			PERR("Read algorithm standby reg failed");
			goto _exit;
		}
		u8_val &= ~HOST_CTRL_MASK_ALGORITHM_STANDBY;
		ret = bhy_write_reg(client_data, BHY_REG_HOST_CTRL,
				&u8_val, 1);
		if (ret < 0) {
			PERR("Write algorithm standby reg failed");
			goto _exit;
		}
		retry = 10;
		do {
			ret = bhy_read_reg(client_data, BHY_REG_HOST_STATUS,
					&u8_val, 1);
			if (ret < 0) {
				PERR("Read host status again failed");
				goto _exit;
			}
			if (!(u8_val & BHY_HOST_STATUS_MASK_ALGO_STANDBY))
				break;
			msleep(1000);
		} while (--retry);
		if (retry == 0) {
			ret = -EIO;
			PERR("Pass thru enable does not take effect");
			goto _exit;
		}
	}

	ret = count;

_exit:

	mutex_unlock(&client_data->mutex_bus_op);
	return ret;
}

static ssize_t bhy_store_enable_irq_log(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	int enable;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtoint(buf, 10, &enable);
	if (ret < 0) {
		PERR("invalid input");
		return ret;
	}
	client_data->enable_irq_log = enable;

	return count;
}

static ssize_t bhy_store_enable_fifo_log(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	int enable;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = kstrtoint(buf, 10, &enable);
	if (ret < 0) {
		PERR("invalid input");
		return ret;
	}
	client_data->enable_fifo_log = enable;

	return count;
}

static ssize_t bhy_show_hw_reg_sel(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	return snprintf(buf, 64, "slave_addr=0X%02X, reg=0X%02X, len=%d\n",
		client_data->hw_slave_addr, client_data->hw_reg_sel,
		client_data->hw_reg_len);
}

static ssize_t bhy_store_hw_reg_sel(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = sscanf(buf, "%11X %11X %11d", &client_data->hw_slave_addr,
		&client_data->hw_reg_sel, &client_data->hw_reg_len);
	if (ret != 3) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	return count;
}

static ssize_t bhy_show_hw_reg_val(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	u8 reg_data[128], i;
	int pos;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	if (client_data->hw_reg_len <= 0) {
		PERR("Invalid register length");
		return -EINVAL;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_soft_pass_thru_read_reg_m(client_data,
		client_data->hw_slave_addr, client_data->hw_reg_sel,
		reg_data, client_data->hw_reg_len);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Reg op failed");
		return ret;
	}

	pos = 0;
	for (i = 0; i < client_data->hw_reg_len; ++i) {
		pos += snprintf(buf + pos, 16, "%02X", reg_data[i]);
		buf[pos++] = (i + 1) % 16 == 0 ? '\n' : ' ';
	}
	if (buf[pos - 1] == ' ')
		buf[pos - 1] = '\n';

	return pos;
}

static ssize_t bhy_store_hw_reg_val(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	u8 reg_data[32] = { 0, };
	int i, j, status, digit;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	if (client_data->hw_reg_len <= 0) {
		PERR("Invalid register length");
		return -EINVAL;
	}

	status = 0;
	for (i = j = 0; i < count && j < client_data->hw_reg_len; ++i) {
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\t' ||
			buf[i] == '\r') {
			status = 0;
			++j;
			continue;
		}
		digit = buf[i] & 0x10 ? (buf[i] & 0xF) : ((buf[i] & 0xF) + 9);
		PDEBUG("digit is %d", digit);
		switch (status) {
		case 2:
			++j; /* Fall thru */
		case 0:
			reg_data[j] = digit;
			status = 1;
			break;
		case 1:
			reg_data[j] = reg_data[j] * 16 + digit;
			status = 2;
			break;
		}
	}
	if (status > 0)
		++j;
	if (j > client_data->hw_reg_len)
		j = client_data->hw_reg_len;
	else if (j < client_data->hw_reg_len) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	PDEBUG("Reg data read as");
	for (i = 0; i < j; ++i)
		PDEBUG("%d", reg_data[i]);

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_soft_pass_thru_write_reg_m(client_data,
		client_data->hw_slave_addr, client_data->hw_reg_sel,
		reg_data, client_data->hw_reg_len);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Reg op failed");
		return ret;
	}

	return count;
}

static ssize_t bhy_show_sw_watchdog_enabled(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_sw_watchdog);
	if (client_data->sw_watchdog_disabled == BHY_TRUE)
		ret = snprintf(buf, 64, "disabled\n");
	else
		ret = snprintf(buf, 64, "enabled\n");
	mutex_unlock(&client_data->mutex_sw_watchdog);

	return ret;
}

static ssize_t bhy_store_sw_watchdog_enabled(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	int enable;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = kstrtoint(buf, 10, &enable);
	if (ret < 0) {
		PERR("invalid input");
		return ret;
	}

	mutex_lock(&client_data->mutex_sw_watchdog);
	if (enable) {
		client_data->sw_watchdog_disabled = BHY_FALSE;
		client_data->inactive_count = 0;
	} else
		client_data->sw_watchdog_disabled = BHY_TRUE;
	mutex_unlock(&client_data->mutex_sw_watchdog);

	return count;
}

static ssize_t bhy_show_hw_watchdog_enabled(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	if (client_data->hw_watchdog_disabled == BHY_TRUE)
		ret = snprintf(buf, 64, "disabled\n");
	else
		ret = snprintf(buf, 64, "enabled\n");

	return ret;
}

static ssize_t bhy_store_hw_watchdog_enabled(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	int enable;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = kstrtoint(buf, 10, &enable);
	if (ret < 0) {
		PERR("invalid input");
		return ret;
	}

	if (enable) {
		client_data->hw_watchdog_disabled = BHY_FALSE;
		client_data->inactive_count = 0;
	} else
		client_data->hw_watchdog_disabled = BHY_TRUE;

	return count;
}

static ssize_t bhy_store_trigger_sw_watchdog(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t ret;
	int req;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	ret = kstrtoint(buf, 10, &req);
	if (ret < 0) {
		PERR("invalid input");
		return ret;
	}
	if (req != 1)
		return -EINVAL;

	PDEBUG("Manual triggered software watchdog!!!");
	client_data->recover_from_disaster = BHY_TRUE;
	bhy_request_firmware(client_data);

	return count;
}

static ssize_t bhy_show_dump_registers(struct device_driver *ddri, char *buf)
{
	struct bhy_client_data *client_data = obj_i2c_data;
	ssize_t len;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&client_data->mutex_bus_op);
	len = bhy_dump_registers(client_data, buf, "");
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%s", buf);

	return len;
}

#endif /*~ BHY_DEBUG */

static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO,
		show_firlen_value, store_firlen_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value, NULL);
static DRIVER_ATTR(rom_id, S_IRUGO, bhy_show_rom_id, NULL);
static DRIVER_ATTR(ram_id, S_IRUGO,
	bhy_show_ram_id, NULL);
static DRIVER_ATTR(status_bank, S_IRUGO,
	bhy_show_status_bank, NULL);
static DRIVER_ATTR(sensor_sel, S_IWUSR | S_IWGRP,
	NULL, bhy_store_sensor_sel);
static DRIVER_ATTR(sensor_info, S_IRUGO,
	bhy_show_sensor_info, NULL);
static DRIVER_ATTR(sensor_conf, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_sensor_conf, bhy_store_sensor_conf);
static DRIVER_ATTR(sensor_flush, S_IWUSR | S_IWGRP,
	NULL, bhy_store_sensor_flush);
static DRIVER_ATTR(calib_profile, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_calib_profile, bhy_store_calib_profile);
static DRIVER_ATTR(sic_matrix, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_sic_matrix, bhy_store_sic_matrix);
static DRIVER_ATTR(meta_event_ctrl, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_meta_event_ctrl, bhy_store_meta_event_ctrl);
static DRIVER_ATTR(fifo_ctrl, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_fifo_ctrl, bhy_store_fifo_ctrl);
#ifdef BHY_AR_HAL_SUPPORT
static DRIVER_ATTR(activate_ar_hal, S_IWUSR | S_IWGRP,
	NULL, bhy_store_activate_ar_hal);
#endif /*~ BHY_AR_HAL_SUPPORT */
static DRIVER_ATTR(reset_flag, S_IRUGO,
	bhy_show_reset_flag, NULL);
static DRIVER_ATTR(working_mode, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_working_mode, bhy_store_working_mode);
static DRIVER_ATTR(op_mode, S_IRUGO,
	bhy_show_op_mode, NULL);
static DRIVER_ATTR(bsx_version, S_IRUGO,
	bhy_show_bsx_version, NULL);
static DRIVER_ATTR(driver_version, S_IRUGO,
	bhy_show_driver_version, NULL);
#ifdef BHY_AR_HAL_SUPPORT
static DRIVER_ATTR(fifo_frame_ar, S_IRUGO,
	bhy_show_fifo_frame_ar, NULL);
#endif /*~ BHY_AR_HAL_SUPPORT */
static DRIVER_ATTR(bmi160_foc_offset_acc, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_bmi160_foc_offset_acc, bhy_store_bmi160_foc_offset_acc);
static DRIVER_ATTR(bmi160_foc_offset_gyro,
	S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_bmi160_foc_offset_gyro, bhy_store_bmi160_foc_offset_gyro);
static DRIVER_ATTR(bmi160_foc_conf, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_bmi160_foc_conf, bhy_store_bmi160_foc_conf);
static DRIVER_ATTR(bmi160_foc_exec, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_bmi160_foc_exec, bhy_store_bmi160_foc_exec);
static DRIVER_ATTR(bmi160_foc_save_to_nvm,
	S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_bmi160_foc_save_to_nvm, bhy_store_bmi160_foc_save_to_nvm);
static DRIVER_ATTR(bma2x2_foc_offset,
	S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_bma2x2_foc_offset, bhy_store_bma2x2_foc_offset);
static DRIVER_ATTR(bma2x2_foc_conf, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_bma2x2_foc_conf, bhy_store_bma2x2_foc_conf);
static DRIVER_ATTR(bma2x2_foc_exec, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_bma2x2_foc_exec, bhy_store_bma2x2_foc_exec);
static DRIVER_ATTR(self_test, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_self_test, bhy_store_self_test);
static DRIVER_ATTR(self_test_result, S_IRUGO,
	bhy_show_self_test_result, NULL);
static DRIVER_ATTR(update_device_info, S_IWUSR | S_IWGRP,
	NULL, bhy_store_update_device_info);
static DRIVER_ATTR(mapping_matrix_acc, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_mapping_matrix_acc, bhy_store_mapping_matrix_acc);
static DRIVER_ATTR(sensor_data_size, S_IWUSR | S_IWGRP,
	NULL, bhy_store_sensor_data_size);
static DRIVER_ATTR(mapping_matrix, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_mapping_matrix, bhy_store_mapping_matrix);
static DRIVER_ATTR(custom_version, S_IRUGO,
	bhy_show_custom_version, NULL);
static DRIVER_ATTR(req_fw, S_IWUSR | S_IWGRP,
	NULL, bhy_store_req_fw);
#ifdef BHY_DEBUG
static DRIVER_ATTR(reg_sel, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_reg_sel, bhy_store_reg_sel);
static DRIVER_ATTR(reg_val, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_reg_val, bhy_store_reg_val);
static DRIVER_ATTR(param_sel, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_param_sel, bhy_store_param_sel);
static DRIVER_ATTR(param_val, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_param_val, bhy_store_param_val);
static DRIVER_ATTR(log_raw_data, S_IWUSR | S_IWGRP,
	NULL, bhy_store_log_raw_data);
static DRIVER_ATTR(log_input_data_gesture, S_IWUSR | S_IWGRP,
	NULL, bhy_store_log_input_data_gesture);
static DRIVER_ATTR(log_input_data_tilt_ar, S_IWUSR | S_IWGRP,
	NULL, bhy_store_log_input_data_tilt_ar);
static DRIVER_ATTR(log_fusion_data, S_IWUSR | S_IWGRP,
	NULL, bhy_store_log_fusion_data);
static DRIVER_ATTR(enable_pass_thru, S_IWUSR | S_IWGRP,
	NULL, bhy_store_enable_pass_thru);
static DRIVER_ATTR(enable_irq_log, S_IWUSR | S_IWGRP,
	NULL, bhy_store_enable_irq_log);
static DRIVER_ATTR(enable_fifo_log, S_IWUSR | S_IWGRP,
	NULL, bhy_store_enable_fifo_log);
static DRIVER_ATTR(hw_reg_sel, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_hw_reg_sel, bhy_store_hw_reg_sel);
static DRIVER_ATTR(hw_reg_val, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_hw_reg_val, bhy_store_hw_reg_val);
static DRIVER_ATTR(sw_watchdog_enabled, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_sw_watchdog_enabled, bhy_store_sw_watchdog_enabled);
static DRIVER_ATTR(hw_watchdog_enabled, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_show_hw_watchdog_enabled, bhy_store_hw_watchdog_enabled);
static DRIVER_ATTR(trigger_sw_watchdog, S_IWUSR | S_IWGRP,
	NULL, bhy_store_trigger_sw_watchdog);
static DRIVER_ATTR(dump_registers, S_IRUGO,
	bhy_show_dump_registers, NULL);
#endif /*~ BHY_DEBUG */

static struct driver_attribute *bmi160_acc_attr_list[] = {
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_firlen,
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_rom_id,
	&driver_attr_ram_id,
	&driver_attr_status_bank,
	&driver_attr_sensor_sel,
	&driver_attr_sensor_info,
	&driver_attr_sensor_conf,
	&driver_attr_sensor_flush,
	&driver_attr_calib_profile,
	&driver_attr_sic_matrix,
	&driver_attr_meta_event_ctrl,
	&driver_attr_fifo_ctrl,
#ifdef BHY_AR_HAL_SUPPORT
	&driver_attr_activate_ar_hal,
#endif /*~ BHY_AR_HAL_SUPPORT */
	&driver_attr_reset_flag,
	&driver_attr_working_mode,
	&driver_attr_op_mode,
	&driver_attr_bsx_version,
	&driver_attr_driver_version,
	&driver_attr_bmi160_foc_offset_acc,
	&driver_attr_bmi160_foc_offset_gyro,
	&driver_attr_bmi160_foc_conf,
	&driver_attr_bmi160_foc_exec,
	&driver_attr_bmi160_foc_save_to_nvm,
	&driver_attr_bma2x2_foc_offset,
	&driver_attr_bma2x2_foc_conf,
	&driver_attr_bma2x2_foc_exec,
	&driver_attr_self_test,
	&driver_attr_self_test_result,
	&driver_attr_update_device_info,
	&driver_attr_mapping_matrix_acc,
	&driver_attr_sensor_data_size,
	&driver_attr_mapping_matrix,
	&driver_attr_custom_version,
	&driver_attr_req_fw,
#ifdef BHY_DEBUG
	&driver_attr_reg_sel,
	&driver_attr_reg_val,
	&driver_attr_param_sel,
	&driver_attr_param_val,
	&driver_attr_log_raw_data,
	&driver_attr_log_input_data_gesture,
	&driver_attr_log_input_data_tilt_ar,
	&driver_attr_log_fusion_data,
	&driver_attr_enable_pass_thru,
	&driver_attr_enable_irq_log,
	&driver_attr_enable_fifo_log,
	&driver_attr_hw_reg_sel,
	&driver_attr_hw_reg_val,
	&driver_attr_sw_watchdog_enabled,
	&driver_attr_hw_watchdog_enabled,
	&driver_attr_trigger_sw_watchdog,
	&driver_attr_dump_registers,
#endif /*~ BHY_DEBUG */
};

#ifdef BHY_AR_HAL_SUPPORT
static struct attribute *input_ar_attributes[] = {
	&dev_attr_rom_id.attr,
	&dev_attr_status_bank.attr,
	&dev_attr_sensor_sel.attr,
	&dev_attr_sensor_conf.attr,
	&dev_attr_sensor_flush.attr,
	&dev_attr_meta_event_ctrl.attr,
	&dev_attr_reset_flag.attr,
	&dev_attr_fifo_frame_ar.attr,
	NULL
};
#endif /*~ BHY_AR_HAL_SUPPORT */

static ssize_t bhy_show_fifo_frame(struct file *file
	, struct kobject *kobj, struct bin_attribute *attr,
	char *buffer, loff_t pos, size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct input_dev *input = to_input_dev(dev);
	struct bhy_client_data *client_data = input_get_drvdata(input);
	struct frame_queue *q = &client_data->data_queue;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	mutex_lock(&q->lock);
	if (q->tail == q->head) { /* Queue empty */
		mutex_unlock(&q->lock);
		return 0;
	}
	memcpy(buffer, &q->frames[q->tail], sizeof(struct fifo_frame));
	if (q->tail == BHY_FRAME_SIZE - 1)
		q->tail = 0;
	else
		++q->tail;
	mutex_unlock(&q->lock);

	return sizeof(struct fifo_frame);
}

static struct bin_attribute bin_attr_fifo_frame = {
	.attr = {
		.name = "fifo_frame",
		.mode = S_IRUGO,
	},
	.size = 0,
	.read = bhy_show_fifo_frame,
	.write = NULL,
};

static ssize_t bhy_bst_show_rom_id(struct device *dev
	, struct device_attribute *attr, char *buf)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;

	ret = snprintf(buf, 32, "%d\n", client_data->rom_id);

	return ret;
}

static ssize_t bhy_bst_show_ram_id(struct device *dev
	, struct device_attribute *attr, char *buf)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;

	ret = snprintf(buf, 32, "%d\n", client_data->ram_id);

	return ret;
}

static ssize_t bhy_bst_show_dev_type(struct device *dev
	, struct device_attribute *attr, char *buf)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;

	ret = snprintf(buf, 32, "%s\n", client_data->dev_type);

	return ret;
}

#ifdef BHY_DSC_SENSOR_SUPPORT

static ssize_t bhy_bst_show_height(struct device *dev
	, struct device_attribute *attr, char *buf)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;
	u8 reg_val;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_CUSTOM,
		BHY_PARAM_HEIGHT, &reg_val, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read from parameter failed");
		return -EIO;
	}
	ret = snprintf(buf, 32, "%d\n", reg_val);

	return ret;
}

static ssize_t bhy_bst_store_height(struct device *dev
	, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;
	long l;
	u8 reg_val;

	ret = kstrtol(buf, 10, &l);
	if (ret < 0) {
		PERR("Convert to integer failed");
		return ret;
	}
	reg_val = (u8)l;
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_CUSTOM,
		BHY_PARAM_HEIGHT, &reg_val, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write to parameter failed");
		return ret;
	}
	PDEBUG("%d\n", reg_val);

	return count;
}

static ssize_t bhy_bst_show_weight(struct device *dev
	, struct device_attribute *attr, char *buf)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;
	u8 reg_val;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_CUSTOM,
		BHY_PARAM_WEIGHT, &reg_val, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read from parameter failed");
		return -EIO;
	}
	ret = snprintf(buf, 32, "%d\n", reg_val);

	return ret;
}

static ssize_t bhy_bst_store_weight(struct device *dev
	, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;
	long l;
	u8 reg_val;

	ret = kstrtol(buf, 10, &l);
	if (ret < 0) {
		PERR("Convert to integer failed");
		return ret;
	}
	reg_val = (u8)l;
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_CUSTOM,
		BHY_PARAM_WEIGHT, &reg_val, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write to parameter failed");
		return ret;
	}
	PDEBUG("%d\n", reg_val);

	return count;
}

static ssize_t bhy_bst_show_gender(struct device *dev
	, struct device_attribute *attr, char *buf)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;
	u8 reg_val;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_CUSTOM,
		BHY_PARAM_GENDER, &reg_val, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read from parameter failed");
		return -EIO;
	}
	ret = snprintf(buf, 32, "%d\n", reg_val);

	return ret;
}

static ssize_t bhy_bst_store_gender(struct device *dev
	, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;
	long l;
	u8 reg_val;

	ret = kstrtol(buf, 10, &l);
	if (ret < 0) {
		PERR("Convert to integer failed");
		return ret;
	}
	reg_val = (u8)l;
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_CUSTOM,
		BHY_PARAM_GENDER, &reg_val, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write to parameter failed");
		return ret;
	}
	PDEBUG("%d\n", reg_val);

	return count;
}

static ssize_t bhy_bst_show_step_len(struct device *dev
	, struct device_attribute *attr, char *buf)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;
	u8 reg_val;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_CUSTOM,
		BHY_PARAM_STEP_LEN, &reg_val, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read from parameter failed");
		return -EIO;
	}
	ret = snprintf(buf, 32, "%d\n", reg_val);

	return ret;
}

static ssize_t bhy_bst_store_step_len(struct device *dev
	, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;
	long l;
	u8 reg_val;

	ret = kstrtol(buf, 10, &l);
	if (ret < 0) {
		PERR("Convert to integer failed");
		return ret;
	}
	reg_val = (u8)l;
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_CUSTOM,
		BHY_PARAM_STEP_LEN, &reg_val, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write to parameter failed");
		return ret;
	}
	PDEBUG("%d\n", reg_val);

	return count;
}

static ssize_t bhy_bst_show_age(struct device *dev
	, struct device_attribute *attr, char *buf)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;
	u8 reg_val;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_parameter(client_data, BHY_PAGE_CUSTOM,
		BHY_PARAM_AGE, &reg_val, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Read from parameter failed");
		return -EIO;
	}
	ret = snprintf(buf, 32, "%d\n", reg_val);

	return ret;
}

static ssize_t bhy_bst_store_age(struct device *dev
	, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct bst_dev *bst_dev = to_bst_dev(dev);
	struct bhy_client_data *client_data = bst_get_drvdata(bst_dev);
	ssize_t ret;
	long l;
	u8 reg_val;

	ret = kstrtol(buf, 10, &l);
	if (ret < 0) {
		PERR("Convert to integer failed");
		return ret;
	}
	reg_val = (u8)l;
	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_CUSTOM,
		BHY_PARAM_AGE, &reg_val, 1);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret < 0) {
		PERR("Write to parameter failed");
		return ret;
	}
	PDEBUG("%d\n", reg_val);

	return count;
}

#endif /*~ BHY_DSC_SENSOR_SUPPORT */

static DEVICE_ATTR(bhy_rom_id, S_IRUGO,
	bhy_bst_show_rom_id, NULL);
static DEVICE_ATTR(bhy_ram_id, S_IRUGO,
	bhy_bst_show_ram_id, NULL);
static DEVICE_ATTR(bhy_dev_type, S_IRUGO,
	bhy_bst_show_dev_type, NULL);
#ifdef BHY_DSC_SENSOR_SUPPORT
static DEVICE_ATTR(height, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_bst_show_height, bhy_bst_store_height);
static DEVICE_ATTR(weight, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_bst_show_weight, bhy_bst_store_weight);
static DEVICE_ATTR(gender, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_bst_show_gender, bhy_bst_store_gender);
static DEVICE_ATTR(step_len, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_bst_show_step_len, bhy_bst_store_step_len);
static DEVICE_ATTR(age, S_IRUGO | S_IWUSR | S_IWGRP,
	bhy_bst_show_age, bhy_bst_store_age);
#endif /*~ BHY_DSC_SENSOR_SUPPORT */

static struct attribute *bst_attributes[] = {
	&dev_attr_bhy_rom_id.attr,
	&dev_attr_bhy_ram_id.attr,
	&dev_attr_bhy_dev_type.attr,
#ifdef BHY_DSC_SENSOR_SUPPORT
	&dev_attr_height.attr,
	&dev_attr_weight.attr,
	&dev_attr_gender.attr,
	&dev_attr_step_len.attr,
	&dev_attr_age.attr,
#endif /*~ BHY_DSC_SENSOR_SUPPORT */
	NULL
};

static void bhy_sw_watchdog_work_func(struct work_struct *work)
{
	struct bhy_client_data *client_data = container_of(work
		, struct bhy_client_data, sw_watchdog_work);
	int in_suspend_copy;
	int i;
	struct bhy_sensor_context *ct;
	int sensor_on = BHY_FALSE;
	int continuous_sensor_on = BHY_FALSE;
	int ret;
	u8 data[16];
	__le16 le16_data;
	u16 byte_remain;
	u8 irq_status;
	char dump[256 * 3];

	in_suspend_copy = atomic_read(&client_data->in_suspend);
	mutex_lock(&client_data->mutex_sw_watchdog);
	if (client_data->sw_watchdog_disabled == BHY_TRUE) {
		mutex_unlock(&client_data->mutex_sw_watchdog);
		return;
	}
	ct = client_data->sensor_context;
	for (i = 1; i <= BHY_SENSOR_HANDLE_MAX; ++i) {
		if (in_suspend_copy && ct[i].is_wakeup == BHY_FALSE)
			continue;
		if (ct[i].sample_rate <= 0)
			continue;
		if (ct[i].type == SENSOR_TYPE_CONTINUOUS &&
			ct[i].report_latency == 0) {
			sensor_on = BHY_TRUE;
			continuous_sensor_on = BHY_TRUE;
			break;
		}
		if (sensor_on == BHY_FALSE)
			sensor_on = BHY_TRUE;
	}
	if (sensor_on == BHY_TRUE)
		++client_data->inactive_count;
	/*PDEBUG("inactive_count is %d", client_data->inactive_count);*/
	if (client_data->inactive_count < BHY_SW_WATCHDOG_EXPIRE_COUNT) {
		mutex_unlock(&client_data->mutex_sw_watchdog);
		return;
	}
	/* Reset inactive count */
	client_data->inactive_count = 0;
	mutex_unlock(&client_data->mutex_sw_watchdog);

	if (continuous_sensor_on == BHY_TRUE) {
		mutex_lock(&client_data->mutex_bus_op);
		ret = bhy_read_reg(client_data, BHY_REG_BYTES_REMAIN_0,
			(u8 *)&le16_data, 2);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Fatal error: I2C communication failed");
			return;
		}
		byte_remain = le16_to_cpu(le16_data);
		ret = bhy_read_reg(client_data, BHY_REG_INT_STATUS,
			(u8 *)&irq_status, 1);
		if (ret < 0) {
			mutex_unlock(&client_data->mutex_bus_op);
			PERR("Fatal error: I2C communication failed");
			return;
		}
		mutex_unlock(&client_data->mutex_bus_op);
		if (byte_remain > 0 && irq_status != 0) {
			/* Retrigger data reading */
			schedule_work(&client_data->irq_work);
			return;
		}
	} else {
		mutex_lock(&client_data->mutex_bus_op);
		ret = bhy_read_parameter(client_data, BHY_PAGE_SYSTEM,
			BHY_PARAM_SYSTEM_HOST_IRQ_TIMESTAMP, data,
			sizeof(data));
		mutex_unlock(&client_data->mutex_bus_op);
		if (ret != -EBUSY) {
			PDEBUG("Check point triggered but passed");
			return;
		}
	}

	PDEBUG("Live lock detected!!!");
	PDEBUG("Dump register after live lock condition");
	bhy_dump_registers(client_data, dump, "BHY");
	PDEBUG("%s", dump);
	client_data->recover_from_disaster = BHY_TRUE;
	bhy_request_firmware(client_data);
}

static enum hrtimer_restart bhy_sw_watchdog_timer_callback(
	struct hrtimer *timer)
{
	struct bhy_client_data *client_data = container_of(timer
		, struct bhy_client_data, sw_watchdog_timer);
	ktime_t now, interval;
	now = hrtimer_cb_get_time(timer);
	interval = ktime_set(0, BHY_SW_WATCHDOG_TIMER_INTERVAL);
	hrtimer_forward(timer, now, interval);
	schedule_work(&client_data->sw_watchdog_work);
	return HRTIMER_RESTART;
}

static void bhy_clear_up(struct bhy_client_data *client_data)
{
	if (client_data != NULL) {
		mutex_destroy(&client_data->mutex_bus_op);
		mutex_destroy(&client_data->data_queue.lock);
		mutex_destroy(&client_data->flush_queue.lock);
#ifdef BHY_AR_HAL_SUPPORT
		mutex_destroy(&client_data->data_queue_ar.lock);
#endif /*~ BHY_AR_HAL_SUPPORT */
		if (client_data->input_attribute_group != NULL) {
			sysfs_remove_group(&client_data->input->dev.kobj,
				client_data->input_attribute_group);
			kfree(client_data->input_attribute_group);
			client_data->input_attribute_group = NULL;
		}
		if(client_data->input){
			sysfs_remove_bin_file(&client_data->input->dev.kobj,
				&bin_attr_fifo_frame);
			if (client_data->input != NULL) {
				input_unregister_device(client_data->input);
				input_free_device(client_data->input);
				client_data->input = NULL;
			}
#ifdef BHY_AR_HAL_SUPPORT
			if (client_data->input_ar_attribute_group != NULL) {
				sysfs_remove_group(&client_data->input_ar->dev.kobj,
					client_data->input_ar_attribute_group);
				kfree(client_data->input_ar_attribute_group);
				client_data->input_ar_attribute_group = NULL;
			}
			if (client_data->input_ar != NULL) {
				input_unregister_device(client_data->input_ar);
				input_free_device(client_data->input_ar);
				client_data->input_ar = NULL;
			}
#endif /*~ BHY_AR_HAL_SUPPORT */
		}
		if(client_data->bst_dev){
			if (client_data->bst_attribute_group != NULL) {
				sysfs_remove_group(&client_data->bst_dev->dev.kobj,
					client_data->bst_attribute_group);
				kfree(client_data->bst_attribute_group);
				client_data->bst_attribute_group = NULL;
			}
			bst_unregister_device(client_data->bst_dev);
			bst_free_device(client_data->bst_dev);
			client_data->bst_dev = NULL;
		}
		if (client_data->data_bus.irq != -1)
			free_irq(client_data->data_bus.irq, client_data);
		if (client_data->fifo_buf != NULL) {
			kfree(client_data->fifo_buf);
			client_data->fifo_buf = NULL;
		}
		if (client_data->data_queue.frames != NULL) {
			kfree(client_data->data_queue.frames);
			client_data->data_queue.frames = NULL;
		}
#ifdef BHY_AR_HAL_SUPPORT
		if (client_data->data_queue_ar.frames != NULL) {
			kfree(client_data->data_queue_ar.frames);
			client_data->data_queue_ar.frames = NULL;
		}
#endif /*~ BHY_AR_HAL_SUPPORT */
		wake_lock_destroy(&client_data->wlock);
		/*move to another clear up function*/
		//hrtimer_cancel(&client_data->sw_watchdog_timer);
		//mutex_destroy(&client_data->mutex_sw_watchdog);
		//cancel_delayed_work_sync(&client_data->fiforead_work);
		//kfree(client_data); will be kfree outside this function
	}
}

static void bhy_clear_up_late(struct bhy_client_data *client_data)
{
	if (client_data != NULL) {
		//wake_lock_destroy(&client_data->wlock);
		hrtimer_cancel(&client_data->sw_watchdog_timer);
		mutex_destroy(&client_data->mutex_sw_watchdog);
		cancel_delayed_work_sync(&client_data->fiforead_work);
		//kfree(client_data); will be kfree outside this function
	}
}


static int bmi160_acc_create_attr(struct device_driver *driver)
{
	int err = 0;
	int idx = 0;
	int num =
	(int)(sizeof(bmi160_acc_attr_list)/sizeof(bmi160_acc_attr_list[0]));

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, bmi160_acc_attr_list[idx]);
		if (err) {
			ACC_PR_ERR("create driver file (%s) failed.\n",
			bmi160_acc_attr_list[idx]->attr.name);
			break;
		}
	}
	return err;
}

static int bmi160_acc_delete_attr(struct device_driver *driver)
{
	int idx = 0;
	int err = 0;
	int num =
	(int)(sizeof(bmi160_acc_attr_list)/sizeof(bmi160_acc_attr_list[0]));
	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, bmi160_acc_attr_list[idx]);
	return err;
}

/*!
 * @brief bmi160 initialization
 *
 * @param[in] client the pointer of i2c_client
 * @param[in] int reset calibration value
 *
 * @return zero success, non-zero failed
 */
 #if 0
static int bmi160_acc_init_client(struct bhy_client_data *obj)
{
	gsensor_gain.x = gsensor_gain.y =
		gsensor_gain.z = obj->reso->sensitivity;
	ACC_LOG("bmi160 acc init OK.\n");
	return 0;
}
#endif
/*
static int bmi160_acc_read_offset(
	struct i2c_client *client, s8 ofs[BMI160_ACC_AXES_NUM])
{
	int err = 0;
#ifdef SW_CALIBRATION
	ofs[0] = ofs[1] = ofs[2] = 0x0;
#else

#endif
	return err;
}
*/
#if 0
static int bmi160_acc_read_calibrationEx(struct i2c_client *client,
		int act[BMI160_ACC_AXES_NUM], int raw[BMI160_ACC_AXES_NUM])
{
	/* raw: the raw calibration data; act: the actual calibration data */
	int mul;
	struct bhy_client_data *obj = obj_i2c_data;
#ifdef SW_CALIBRATION
	/* only SW Calibration, disable  Calibration */
	mul = 0;
#else
	/*
	int err;
	err = bmi160_acc_read_offset(client, obj->offset);
	if (err) {
		ACC_PR_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity/bmi160_acc_offset_resolution.sensitivity;
	*/
#endif
	raw[BMI160_ACC_AXIS_X] =
			obj->offset[BMI160_ACC_AXIS_X]*mul + obj->cali_sw[BMI160_ACC_AXIS_X];
	raw[BMI160_ACC_AXIS_Y] =
			obj->offset[BMI160_ACC_AXIS_Y]*mul + obj->cali_sw[BMI160_ACC_AXIS_Y];
	raw[BMI160_ACC_AXIS_Z] =
			obj->offset[BMI160_ACC_AXIS_Z]*mul + obj->cali_sw[BMI160_ACC_AXIS_Z];

	act[obj->cvt.map[BMI160_ACC_AXIS_X]] =
			obj->cvt.sign[BMI160_ACC_AXIS_X]*raw[BMI160_ACC_AXIS_X];
	act[obj->cvt.map[BMI160_ACC_AXIS_Y]] =
			obj->cvt.sign[BMI160_ACC_AXIS_Y]*raw[BMI160_ACC_AXIS_Y];
	act[obj->cvt.map[BMI160_ACC_AXIS_Z]] =
			obj->cvt.sign[BMI160_ACC_AXIS_Z]*raw[BMI160_ACC_AXIS_Z];

	return 0;
}

static int bmi160_acc_write_calibration(
	struct i2c_client *client, int dat[BMI160_ACC_AXES_NUM])
{
	int err = 0;
	int cali[BMI160_ACC_AXES_NUM] = {0};
	int raw[BMI160_ACC_AXES_NUM] = {0};
	struct bhy_client_data *obj = obj_i2c_data;
#ifndef SW_CALIBRATION
	int lsb = bmi160_acc_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;
#endif
	err = bmi160_acc_read_calibrationEx(client, cali, raw);
	/* offset will be updated in obj->offset */
	if (err) {
		ACC_PR_ERR("read offset fail, %d\n", err);
		return err;
	}
	ACC_LOG("OLDOFF:(%+3d%+3d%+3d):(%+3d%+3d %+3d)/(%+3d%+3d%+3d)\n",
		raw[BMI160_ACC_AXIS_X], raw[BMI160_ACC_AXIS_Y], raw[BMI160_ACC_AXIS_Z],
		obj->offset[BMI160_ACC_AXIS_X], obj->offset[BMI160_ACC_AXIS_Y],
		obj->offset[BMI160_ACC_AXIS_Z],
		obj->cali_sw[BMI160_ACC_AXIS_X], obj->cali_sw[BMI160_ACC_AXIS_Y],
		obj->cali_sw[BMI160_ACC_AXIS_Z]);
	/* calculate the real offset expected by caller */
	cali[BMI160_ACC_AXIS_X] += dat[BMI160_ACC_AXIS_X];
	cali[BMI160_ACC_AXIS_Y] += dat[BMI160_ACC_AXIS_Y];
	cali[BMI160_ACC_AXIS_Z] += dat[BMI160_ACC_AXIS_Z];
	ACC_LOG("UPDATE: (%+3d %+3d %+3d)\n",
	dat[BMI160_ACC_AXIS_X], dat[BMI160_ACC_AXIS_Y], dat[BMI160_ACC_AXIS_Z]);
#ifdef SW_CALIBRATION
	obj->cali_sw[BMI160_ACC_AXIS_X] = obj->cvt.sign[BMI160_ACC_AXIS_X]
		* (cali[obj->cvt.map[BMI160_ACC_AXIS_X]]);
	obj->cali_sw[BMI160_ACC_AXIS_Y] = obj->cvt.sign[BMI160_ACC_AXIS_Y]
		* (cali[obj->cvt.map[BMI160_ACC_AXIS_Y]]);
	obj->cali_sw[BMI160_ACC_AXIS_Z] = obj->cvt.sign[BMI160_ACC_AXIS_Z]
		* (cali[obj->cvt.map[BMI160_ACC_AXIS_Z]]);
#else
	obj->offset[BMI160_ACC_AXIS_X] = (s8)(obj->cvt.sign[BMI160_ACC_AXIS_X]
	* (cali[obj->cvt.map[BMI160_ACC_AXIS_X]])/(divisor));
	obj->offset[BMI160_ACC_AXIS_Y] = (s8)(obj->cvt.sign[BMI160_ACC_AXIS_Y]
	* (cali[obj->cvt.map[BMI160_ACC_AXIS_Y]])/(divisor));
	obj->offset[BMI160_ACC_AXIS_Z] = (s8)(obj->cvt.sign[BMI160_ACC_AXIS_Z]
	* (cali[obj->cvt.map[BMI160_ACC_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[BMI160_ACC_AXIS_X] = obj->cvt.sign[BMI160_ACC_AXIS_X]
	* (cali[obj->cvt.map[BMI160_ACC_AXIS_X]])%(divisor);
	obj->cali_sw[BMI160_ACC_AXIS_Y] = obj->cvt.sign[BMI160_ACC_AXIS_Y]
	* (cali[obj->cvt.map[BMI160_ACC_AXIS_Y]])%(divisor);
	obj->cali_sw[BMI160_ACC_AXIS_Z] = obj->cvt.sign[BMI160_ACC_AXIS_Z]
	* (cali[obj->cvt.map[BMI160_ACC_AXIS_Z]])%(divisor);

	ACC_LOG("NEWOFF:(%+3d %+3d %+3d):(%+3d %+3d %+3d)/(%+3d %+3d %+3d)\n",
	obj->offset[BMI160_ACC_AXIS_X]*divisor +
	obj->cali_sw[BMI160_ACC_AXIS_X],
	obj->offset[BMI160_ACC_AXIS_Y]*divisor +
	obj->cali_sw[BMI160_ACC_AXIS_Y],
	obj->offset[BMI160_ACC_AXIS_Z]*divisor +
	obj->cali_sw[BMI160_ACC_AXIS_Z],
	obj->offset[BMI160_ACC_AXIS_X], obj->offset[BMI160_ACC_AXIS_Y],
	obj->offset[BMI160_ACC_AXIS_Z],
	obj->cali_sw[BMI160_ACC_AXIS_X], obj->cali_sw[BMI160_ACC_AXIS_Y],
	obj->cali_sw[BMI160_ACC_AXIS_Z]);

	err = bma_i2c_write_block(obj->client,
	BMI160_USER_OFFSET_0_ADDR, obj->offset, BMI160_ACC_AXES_NUM);
	if (err) {
		ACC_PR_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}
#endif

#if 0

/*!
 * @brief Reset calibration for acc
 *
 * @param[in] client the pointer of i2c_client
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_reset_calib(struct i2c_client *client)
{
	int err = 0;
	struct bhy_client_data *obj = obj_i2c_data;
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;
}

static int bmi160_acc_read_calibration(
	struct i2c_client *client, int dat[BMI160_ACC_AXES_NUM])
{
	struct bhy_client_data *obj = obj_i2c_data;
	int err = 0;
	int mul;
#ifdef SW_CALIBRATION
	mul = 0;/*only SW Calibration, disable HW Calibration*/
#else

#endif

	dat[obj->cvt.map[BMI160_ACC_AXIS_X]] = obj->cvt.sign[BMI160_ACC_AXIS_X]
		*(obj->offset[BMI160_ACC_AXIS_X]*mul + obj->cali_sw[BMI160_ACC_AXIS_X]);
	dat[obj->cvt.map[BMI160_ACC_AXIS_Y]] = obj->cvt.sign[BMI160_ACC_AXIS_Y]
		 *(obj->offset[BMI160_ACC_AXIS_Y]*mul + obj->cali_sw[BMI160_ACC_AXIS_Y]);
	dat[obj->cvt.map[BMI160_ACC_AXIS_Z]] = obj->cvt.sign[BMI160_ACC_AXIS_Z]
		 *(obj->offset[BMI160_ACC_AXIS_Z]*mul + obj->cali_sw[BMI160_ACC_AXIS_Z]);

	return err;
}


static int bmi160_acc_open(struct inode *inode, struct file *file)
{
	file->private_data = bmi160_acc_i2c_client;
	if (file->private_data == NULL) {
		ACC_PR_ERR("file->private_data is null pointer.\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int bmi160_acc_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long bmi160_acc_unlocked_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg)
{
	char strbuf[BMI160_BUFSIZE] = {0};
	void __user *data;
	SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3] = {0};
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct bhy_client_data *obj = obj_i2c_data;
	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(
		VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(
		VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}
	if (err) {
		ACC_PR_ERR("access error: %08X, (%2d, %2d)\n",
		cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}
	ACC_LOG("bmi160_acc_unlocked_ioctl, cmd = 0x%x\n", cmd);
	switch (cmd) {
	case GSENSOR_IOCTL_INIT:
		bmi160_acc_init_client(obj);
		break;
	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *) arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		bmi160_acc_read_chipInfo(client, strbuf, BMI160_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf)+1))
			err = -EFAULT;
		break;
	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *) arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		bmi160_acc_read_sensor_data(obj);
		if (copy_to_user(data, strbuf, strlen(strbuf)+1))
			err = -EFAULT;
		break;
	case GSENSOR_IOCTL_READ_GAIN:
		data = (void __user *) arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_to_user(data,
			&gsensor_gain, sizeof(GSENSOR_VECTOR3D))) {
			err = -EFAULT;
		}
		break;
	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *) arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		bmi160_acc_read_raw_data(client, strbuf);
		if (copy_to_user(data, &strbuf, strlen(strbuf)+1))
			err = -EFAULT;
		break;
	case GSENSOR_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		if (atomic_read(&obj->suspend)) {
			ACC_PR_ERR("can't calibration in suspend\n");
			err = -EINVAL;
		} else {
			cali[BMI160_ACC_AXIS_X] = sensor_data.x *
					obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[BMI160_ACC_AXIS_Y] = sensor_data.y *
					obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[BMI160_ACC_AXIS_Z] = sensor_data.z *
					obj->reso->sensitivity / GRAVITY_EARTH_1000;
			//TODO need do calibration in bhy
			/*err = bmi160_acc_write_calibration(client, cali);*/
		}
		break;
	case GSENSOR_IOCTL_CLR_CALI:
		err = bmi160_acc_reset_calib(client);
		break;
	case GSENSOR_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		err = bmi160_acc_read_calibration(client, cali);
		if (err) {
			ACC_PR_ERR("read calibration failed.\n");
			break;
		}
		sensor_data.x = cali[BMI160_ACC_AXIS_X] *
				GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.y = cali[BMI160_ACC_AXIS_Y] *
				GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.z = cali[BMI160_ACC_AXIS_Z] *
				GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			err = -EFAULT;
		break;
	default:
		ACC_PR_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}
	return err;
}
static const struct file_operations bmi160_acc_fops = {
	/* .owner = THIS_MODULE, */
	.open = bmi160_acc_open,
	.release = bmi160_acc_release,
	.unlocked_ioctl = bmi160_acc_unlocked_ioctl,
};

static struct miscdevice bmi160_acc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &bmi160_acc_fops,
};
#endif

/*!
 * @brief if use this type of enable,
 * Gsensor should report inputEvent(x, y, z, status, div) to HAL
 *
 * @param[in] int open true or false
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_open_report_data(int open)
{
	return 0;
}

/*!
 * @brief If use this type of enable, Gsensor only enabled but not report inputEvent to HAL
 *
 * @param[in] int enable true or false
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_enable_nodata(int en)
{
	int err = 0;
	u8 conf[8];
	u16 tmp;
	struct bhy_client_data *obj;
	bool power = false;
	if (1 == en) {
		power = true;
		sensor_power = true;
	} else {
		power =  false;
		sensor_power = false;
	}
	obj = obj_i2c_data;
	ACC_LOG("bmi160_acc_enable_nodata enable = %d.\n", en);
	tmp = (u16)en;
	memcpy(conf, &tmp, 2);
	memset(conf + 2, 0, 2);
	memset(conf + 4, 0, 4);
	/* 1-> Acc, 4 -> Gyro */
	err = bhy_set_sensor_conf(obj, 1, conf);
	if (err < 0) {
		ACC_PR_ERR("bmi160_acc_set_power_mode failed.\n");
		return err;
	}
	ACC_LOG("bmi160_acc_enable_nodata ok!\n");
	return err;
}

/*!
 * @brief set the delay value for acc
 *
 * @param[in] u64 ns for dealy
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_set_delay(u64 ns)
{
	int value = 0;
	u16 sample_delay = 0;
	int err = 0;
	u8 conf[8];
	struct bhy_client_data *obj = obj_i2c_data;
	value = (int)ns/1000/1000;
	/* Currently, set max data rate to 100Hz as MTK platform */
	if(value <= 10)
	{
	  sample_delay = 100;
	}
	else
	{
	  sample_delay = 1000 / value;
	}

	ACC_LOG("bmi160_acc_set_delay = (%d Hz)\n", sample_delay);

	memcpy(conf, &sample_delay, 2);
	memset(conf + 2, 0, 2);
	memset(conf + 4, 0, 4);

	err = bhy_set_sensor_conf(obj, 1, conf);
	if (err < 0) {
		ACC_PR_ERR("set delay parameter error!\n");
		return err;
	}
	return 0;
}

/*!
 * @brief get the raw data for gsensor
 *
 * @param[in] int x axis value
 * @param[in] int y axis value
 * @param[in] int z axis value
 * @param[in] int status
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_get_data(int *x, int *y, int *z, int *status)
{
	char buff[BMI160_BUFSIZE];
	getCompensateAccData(obj_i2c_data, buff);
	sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return 0;
}

static int gsensor_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs/1000/1000;

	ACC_LOG("gsensor_batch period=(%d ms)\n", value);
	return bmi160_acc_set_delay(samplingPeriodNs);
}
static int gsensor_flush(void)
{
	return acc_flush_report();
}

static int bmi160_acc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct i2c_client *new_client;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};

	struct bhy_client_data *client_data = NULL;
	int ret = 0;
	ktime_t ktime;
	int i;
	struct bhy_data_bus data_bus = {
		.read = bhy_i2c_read,
		.write = bhy_i2c_write,
		.dev = &client->dev,
		.irq = client->irq,
		.bus_type = BUS_I2C,
	};

	GSE_FUN();

	ACC_LOG("%s: is begin, %p, %d, %p, %d.\n", __func__, client, client->addr, id, bmi160_acc_init_flag);

	if(bmi160_acc_init_flag == 0)
		return 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		return -EIO;
	}

	/* init client_data */
	client_data = kzalloc(sizeof(struct bhy_client_data), GFP_KERNEL);
	if (client_data == NULL) {
		PERR("no memory available for struct bhy_client_data");
		ret = -ENOMEM;
		goto err_exit;
	}
	memset(client_data, 0, sizeof(struct bhy_client_data));
	err = get_accel_dts_func(client->dev.of_node, &client_data->hw);
	if (err < 0) {
		ACC_PR_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit_kfree;
	}


	err = hwmsen_get_convert(client_data->hw.direction, &client_data->cvt);
		printk("[Harry] hw.direction: = %d\n",	client_data->hw.direction);	
	if (err) {
		ACC_PR_ERR("invalid direction: %d\n", client_data->hw.direction);
		goto exit;
	}
	obj_i2c_data = client_data;
	obj_i2c_data->client = client;
	bmi160_acc_i2c_client = new_client = client_data->client;
	i2c_set_clientdata(new_client, client_data);
	atomic_set(&client_data->trace, 0);
	atomic_set(&client_data->suspend, 0);
	mutex_init(&client_data->lock);

#ifdef BHY_SUPPORT_I2C_DMA
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	gpDMABuf_va = (u8 *)dma_alloc_coherent(&client->dev, BHY_DMA_MAX_ALLOCATE,&gpDMABuf_pa, GFP_KERNEL);
	if (!gpDMABuf_va) {
		printk("%s Allocate DMA I2C Buffer failed!\n", __func__);
		err = -EFAULT;
		goto exit_kfree;
	}else
	{
		printk("DMA BUFF Allocate OK.\n");
	}	
 	memset(gpDMABuf_va, 0, BHY_DMA_MAX_ALLOCATE);
#endif
	
	/* check chip id */
	ret = bhy_check_chip_id(&data_bus);
	if (ret < 0) {
		ACC_PR_ERR("Bosch Sensortec Device not found, chip id mismatch");
		goto err_exit;
	}
	ACC_LOG("Bosch Sensortec Device %s detected", SENSOR_NAME);


	err = bmi160_acc_create_attr(
		&(bmi160_acc_init_info.platform_diver_addr->driver));
	if (err) {
		ACC_PR_ERR("create attribute failed.\n");
		goto err_exit;
	}

	ctl.open_report_data = bmi160_acc_open_report_data;
	ctl.enable_nodata = bmi160_acc_enable_nodata;
	//ctl.set_delay = bmi160_acc_set_delay;
	ctl.batch = gsensor_batch;
	ctl.flush = gsensor_flush;
	ctl.is_report_input_direct = false;
	err = acc_register_control_path(&ctl);
	if (err) {
		ACC_PR_ERR("register acc control path error.\n");
		goto err_exit;
	}
	data.get_data = bmi160_acc_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		ACC_PR_ERR("register acc data path error.\n");
		goto err_exit;
	}

	client_data->reso = &bmi160_acc_data_resolution[1];
	dev_set_drvdata(data_bus.dev, client_data);
	client_data->data_bus = data_bus;
	mutex_init(&client_data->mutex_bus_op);
	mutex_init(&client_data->data_queue.lock);
	mutex_init(&client_data->flush_queue.lock);
#ifdef BHY_AR_HAL_SUPPORT
	mutex_init(&client_data->data_queue_ar.lock);
#endif /*~ BHY_AR_HAL_SUPPORT */
	client_data->rom_id = 0;
	client_data->ram_id = 0;
	client_data->dev_type[0] = '\0';
	bhy_clear_flush_queue(&client_data->flush_queue);
	memset(client_data->self_test_result, -1, PHYSICAL_SENSOR_COUNT);
#ifdef BHY_TS_LOGGING_SUPPORT
	client_data->irq_count = 0;
#endif /*~ BHY_TS_LOGGING_SUPPORT */

#ifdef BHY_ON_NEXUS5
	client_data->irq_normal_run_flag = 0;
	client_data->irq_rtc_run_flag = 0;
#endif

	wake_lock_init(&client_data->wlock, WAKE_LOCK_SUSPEND, "bhy");

	client_data->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, gse_1-eint");

	ret = bhy_request_irq(client_data);
	if (ret < 0) {
		ACC_PR_ERR("Request IRQ failed : %d", ret);
		goto err_exit;
	}

	/* init input devices */
	ret = bhy_init_input_dev(client_data);
	if (ret < 0) {
		PERR("Init input dev failed");
		goto err_exit;
	}
	/* sysfs input node creation */
	/*
	client_data->input_attribute_group =
		kzalloc(sizeof(struct attribute_group), GFP_KERNEL);
	if (client_data->input_attribute_group == NULL) {
		ret = -ENOMEM;
		PERR("No mem for input_attribute_group");
		goto err_exit;
	}
	client_data->input_attribute_group->attrs = input_attributes;
	ret = sysfs_create_group(&client_data->input->dev.kobj,
		client_data->input_attribute_group);
	if (ret < 0) {
		kfree(client_data->input_attribute_group);
		client_data->input_attribute_group = NULL;
		goto err_exit;
	}
	 */
	ret = sysfs_create_bin_file(&client_data->input->dev.kobj,
		&bin_attr_fifo_frame);
	if (ret < 0) {
		//sysfs_remove_bin_file(&client_data->input->dev.kobj,
			//&bin_attr_fifo_frame);
		goto err_exit;
	}
	
#ifdef BHY_AR_HAL_SUPPORT
	/* sysfs input node for AR creation */
	client_data->input_ar_attribute_group =
		kzalloc(sizeof(struct attribute_group), GFP_KERNEL);
	if (client_data->input_ar_attribute_group == NULL) {
		ret = -ENOMEM;
		PERR("No mem for input_ar_attribute_group");
		goto err_exit;
	}
	client_data->input_ar_attribute_group->attrs = input_ar_attributes;
	ret = sysfs_create_group(&client_data->input_ar->dev.kobj,
		client_data->input_ar_attribute_group);
	if (ret < 0) {
		kfree(client_data->input_ar_attribute_group);
		client_data->input_ar_attribute_group = NULL;
		goto err_exit;
	}
#endif /*~ BHY_AR_HAL_SUPPORT */

	/* bst device creation */
	client_data->bst_dev = bst_allocate_device();
	if (!client_data->bst_dev) {
		PERR("Allocate bst device failed");
		goto err_exit;
	}
	client_data->bst_dev->name = SENSOR_NAME;
	bst_set_drvdata(client_data->bst_dev, client_data);
	ret = bst_register_device(client_data->bst_dev);
	if (ret < 0) {
		bst_free_device(client_data->bst_dev);
		client_data->bst_dev = NULL;
		PERR("Register bst device failed");
		goto err_exit;
	}
	client_data->bst_attribute_group =
		kzalloc(sizeof(struct attribute_group), GFP_KERNEL);
	if (client_data->bst_attribute_group == NULL) {
		ret = -ENOMEM;
		PERR("No mem for bst_attribute_group");
		goto err_exit;
	}
	client_data->bst_attribute_group->attrs = bst_attributes;
	ret = sysfs_create_group(&client_data->bst_dev->dev.kobj,
		client_data->bst_attribute_group);
	if (ret < 0) {
		PERR("Create sysfs nodes for bst device failed");
		goto err_exit;
	}
	
	client_data->fifo_buf = kmalloc(BHY_FIFO_LEN_MAX, GFP_KERNEL);
	if (!client_data->fifo_buf) {
		ACC_PR_ERR("Allocate FIFO buffer failed");
		ret = -ENOMEM;
		goto err_exit;
	}

	client_data->data_queue.frames = kmalloc(BHY_FRAME_SIZE *
			sizeof(struct fifo_frame), GFP_KERNEL);
	if (!client_data->data_queue.frames) {
		ACC_PR_ERR("Allocate FIFO frame buffer failed");
		ret = -ENOMEM;
		goto err_exit;
	}
	client_data->data_queue.head = 0;
	client_data->data_queue.tail = 0;

#ifdef BHY_AR_HAL_SUPPORT
	client_data->data_queue_ar.frames = kmalloc(BHY_FRAME_SIZE_AR *
			sizeof(struct fifo_frame), GFP_KERNEL);
	if (!client_data->data_queue_ar.frames) {
		PERR("Allocate ar FIFO frame buffer failed");
		ret = -ENOMEM;
		goto err_exit;
	}
	client_data->data_queue_ar.head = 0;
	client_data->data_queue_ar.tail = 0;
#endif /*~ BHY_AR_HAL_SUPPORT */
	bhy_init_sensor_context(client_data);
	ktime = ktime_set(0, BHY_SW_WATCHDOG_TIMER_INTERVAL);
	hrtimer_init(&client_data->sw_watchdog_timer, CLOCK_MONOTONIC,
		HRTIMER_MODE_REL);
	client_data->sw_watchdog_timer.function =
		&bhy_sw_watchdog_timer_callback;
	INIT_WORK(&client_data->sw_watchdog_work, bhy_sw_watchdog_work_func);

#ifdef BHY_ON_NEXUS5
	INIT_DELAYED_WORK(&client_data->fiforead_work, bhy_irq_work_func_rtc);
#endif
#ifdef BHY_SW_WATCHDOG_SUPPORT
	hrtimer_start(&client_data->sw_watchdog_timer, ktime, HRTIMER_MODE_REL);
#endif /* BHY_SW_WATCHDOG_SUPPORT */

	client_data->need_wait_rtc = false;
	mutex_init(&client_data->mutex_sw_watchdog);
	client_data->inactive_count = 0;
	client_data->recover_from_disaster = BHY_FALSE;
	client_data->sw_watchdog_disabled = BHY_FALSE;
	client_data->hw_watchdog_disabled = BHY_FALSE;
	for (i = 0; i < PHYSICAL_SENSOR_COUNT; ++i) {
		client_data->ps_context[i].index = i;
		client_data->ps_context[i].use_mapping_matrix = BHY_FALSE;
	}
	for (i = 1; i <= BHY_META_EVENT_MAX; ++i) {
		client_data->me_context[i].index = i;
		client_data->me_context[i].event_en = BHY_STATUS_DEFAULT;
		client_data->me_context[i].irq_en = BHY_STATUS_DEFAULT;
		client_data->mew_context[i].index = i;
		client_data->mew_context[i].event_en = BHY_STATUS_DEFAULT;
		client_data->mew_context[i].irq_en = BHY_STATUS_DEFAULT;
	}
	bmi160_acc_init_flag = 0;
	atomic_set(&client_data->reset_flag, RESET_FLAG_TODO);

	ACC_LOG("sensor %s probed successfully", SENSOR_NAME);

	return 0;
//err_exit2:
	bhy_clear_up_late(client_data);

err_exit:
	bhy_clear_up(client_data);
exit_kfree:
	kfree(client_data);
	client_data = NULL;
exit:
	ACC_PR_ERR("%s: err = %d\n", __func__, err);
	bmi160_acc_init_flag = -1;
	return ret;
}

static int bmi160_acc_i2c_remove(struct i2c_client *client)
{
	int err = 0;
	err = bmi160_acc_delete_attr(
		&(bmi160_acc_init_info.platform_diver_addr->driver));
	if (err)
		ACC_PR_ERR("delete device attribute failed.\n");

	bmi160_acc_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(obj_i2c_data);
	return 0;
}



//EXPORT_SYMBOL(bhy_probe);

int bhy_remove(struct device *dev)
{
	struct bhy_client_data *client_data = dev_get_drvdata(dev);
	bhy_clear_up(client_data);
	return 0;
}
EXPORT_SYMBOL(bhy_remove);

#ifdef CONFIG_PM
int bhy_suspend(struct device *dev)
{
	struct bhy_client_data *client_data = dev_get_drvdata(dev);
	int ret;
	u8 data;
#ifdef BHY_TS_LOGGING_SUPPORT
	struct frame_queue *q = &client_data->data_queue;
#endif /*~ BHY_TS_LOGGING_SUPPORT */

	PINFO("Enter suspend");

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, BHY_REG_HOST_CTRL, &data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read host ctrl reg failed");
		return -EIO;
	}
	data |= HOST_CTRL_MASK_AP_SUSPENDED;
	ret = bhy_write_reg(client_data, BHY_REG_HOST_CTRL, &data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write host ctrl reg failed");
		return -EIO;
	}
	mutex_unlock(&client_data->mutex_bus_op);

	enable_irq_wake(client_data->data_bus.irq);

	atomic_set(&client_data->in_suspend, 1);
	client_data->need_wait_rtc = true;

#ifdef BHY_TS_LOGGING_SUPPORT
	mutex_lock(&q->lock);
	q->frames[q->head].handle = BHY_SENSOR_HANDLE_AP_SLEEP_STATUS;
	q->frames[q->head].data[0] = BHY_AP_STATUS_SUSPEND;
	bhy_advance_fifo_queue_head(q);
	mutex_unlock(&q->lock);

	input_event(client_data->input, EV_MSC, MSC_RAW, 0);
	input_sync(client_data->input);
#endif /*~ BHY_TS_LOGGING_SUPPORT */

	return 0;
}
EXPORT_SYMBOL(bhy_suspend);

int bhy_resume(struct device *dev)
{
	struct bhy_client_data *client_data = dev_get_drvdata(dev);
	int ret;
	u8 data;
#ifdef BHY_TS_LOGGING_SUPPORT
	struct frame_queue *q = &client_data->data_queue;
#endif /*~ BHY_TS_LOGGING_SUPPORT */

	PINFO("Enter resume");

	disable_irq_wake(client_data->data_bus.irq);

	/* Wait for 50ms in case we cannot receive IRQ */
	msleep(50);

	mutex_lock(&client_data->mutex_bus_op);
	ret = bhy_read_reg(client_data, BHY_REG_HOST_CTRL, &data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Read host ctrl reg failed");
		return -EIO;
	}
	data &= ~HOST_CTRL_MASK_AP_SUSPENDED;
	ret = bhy_write_reg(client_data, BHY_REG_HOST_CTRL, &data, 1);
	if (ret < 0) {
		mutex_unlock(&client_data->mutex_bus_op);
		PERR("Write host ctrl reg failed");
		return -EIO;
	}
	mutex_unlock(&client_data->mutex_bus_op);

	atomic_set(&client_data->in_suspend, 0);

	/* Flush after resume */
#ifdef BHY_FLUSH_AFTER_RESUME
	ret = bhy_enqueue_flush(client_data, BHY_FLUSH_FLUSH_ALL);
	if (ret < 0) {
		PERR("Write sensor flush failed");
		return ret;
	}
#endif /*~ BHY_FLUSH_AFTER_RESUME */

#ifdef BHY_TS_LOGGING_SUPPORT
	client_data->irq_count = 0;
	mutex_lock(&q->lock);
	q->frames[q->head].handle = BHY_SENSOR_HANDLE_AP_SLEEP_STATUS;
	q->frames[q->head].data[0] = BHY_AP_STATUS_RESUME;
	bhy_advance_fifo_queue_head(q);
	mutex_unlock(&q->lock);

	input_event(client_data->input, EV_MSC, MSC_RAW, 0);
	input_sync(client_data->input);
#endif /*~ BHY_TS_LOGGING_SUPPORT */

	return 0;
}
EXPORT_SYMBOL(bhy_resume);
#endif /*~ CONFIG_PM */

#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
//	{ .compatible = "mediatek,bmi160_acc", },
	{ .compatible = "mediatek,gsensor", },
	{},
};
#endif

#ifndef CONFIG_HAS_EARLYSUSPEND
static int bmi160_acc_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct bhy_client_data *obj = obj_i2c_data;

	GSE_FUN();
	
	if (msg.event == PM_EVENT_SUSPEND)
		atomic_set(&obj->suspend, 1);
	return 0;
}

static int bmi160_acc_resume(struct i2c_client *client)
{
	struct bhy_client_data *obj = obj_i2c_data;

	GSE_FUN();
	
	atomic_set(&obj->suspend, 0);
	return 0;
}

#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/

static void bmi160_acc_early_suspend(struct early_suspend *h)
{
	struct bhy_client_data *obj =
		container_of(h, struct bhy_client_data, early_drv);
	atomic_set(&obj->suspend, 1);
	return;
}

static void bmi160_acc_late_resume(struct early_suspend *h)
{
	struct bhy_client_data *obj =
		container_of(h, struct bhy_client_data, early_drv);
	atomic_set(&obj->suspend, 0);
	return;
}

#endif /*CONFIG_HAS_EARLYSUSPEND*/


static struct i2c_driver bmi160_acc_i2c_driver = {
	.driver = {
		.name = BMI160_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = gsensor_of_match,
#endif
	},
	.probe = bmi160_acc_i2c_probe,
	.remove = bmi160_acc_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = bmi160_acc_suspend,
	.resume = bmi160_acc_resume,
#endif
	.id_table = bmi160_acc_i2c_id,
};



int  bmi160_acc_local_init(void)
{
	int err = 0;

	GSE_FUN();

#if 0
	/*need define in the DTS side for bmi160_acc*/
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info;
	adapter = i2c_get_adapter(3);
#endif

	ACC_PR_ERR("bmi160_acc_local_init enter\n");

#if 0
	if (adapter == NULL) {
		printk(KERN_ERR "gsensor_local_init error");
		err = -1;
		return err;
	}
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = 0x28;
	strlcpy(info.type, "bmi160_acc", I2C_NAME_SIZE);
	client = i2c_new_device(adapter, &info);
	strlcpy(client->name, "bmi160_acc", I2C_NAME_SIZE);
#endif
	if (i2c_add_driver(&bmi160_acc_i2c_driver)) {
		ACC_PR_ERR("add driver error\n");
		err = -1;
		return err;
	}
	if (-1 == bmi160_acc_init_flag) {
		ACC_LOG("bmi160 acc local init failed.\n");
		err = -1;
		return err;
	}
	ACC_LOG("bmi160 acc local init.\n");
	return 0;
}

static int bmi160_acc_remove(void)
{
	GSE_FUN();
	i2c_del_driver(&bmi160_acc_i2c_driver);
	return 0;
}

static struct acc_init_info bmi160_acc_init_info = {
	.name = BMI160_DEV_NAME,
	.init = bmi160_acc_local_init,
	.uninit = bmi160_acc_remove,
};

static int __init bmi160_acc_init(void)
{
	GSE_FUN();
	acc_driver_add(&bmi160_acc_init_info);
	return 0;
}

static void __exit bmi160_acc_exit(void)
{
	GSE_FUN();
}

module_init(bmi160_acc_init);
module_exit(bmi160_acc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMI160_ACC I2C driver");
MODULE_AUTHOR("contact@bosch-sensortec.com>");
