/*!
 * @section LICENSE
 * (C) Copyright 2011~2016 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 * VERSION: V2.0
 * Date: 2016/12/08

 * History: V1.0 --- [2013.01.29]Driver creation
 *          V1.1 --- [2013.03.28]
 *                   1.Instead late_resume, use resume to make sure
 *                     driver resume is ealier than processes resume.
 *                   2.Add power mode setting in read data.
 *          V1.2 --- [2013.06.28]Add self test function.
 *          V1.3 --- [2013.07.26]Fix the bug of wrong axis remapping
 *                   in rawdata inode.
 V2.0  change the driver node
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/module.h>

#include <gyroscope.h>
#include <cust_gyro.h>
#include <hwmsensor.h>
//#include <linux/hwmsensor.h>
//#include <linux/hwmsen_dev.h>
//#include <linux/sensors_io.h>
#include <sensors_io.h>
//#include <linux/hwmsen_helper.h>
#include <hwmsen_helper.h>
#include <cust_acc.h>
#include "bhy_core.h"
#include "bhy_gyro.h"

/* debug infomation flags */
enum GYRO_TRC {
	GYRO_TRC_FILTER  = 0x01,
	GYRO_TRC_RAWDATA = 0x02,
	GYRO_TRC_IOCTL   = 0x04,
	GYRO_TRC_CALI	= 0x08,
	GYRO_TRC_INFO	= 0x10,
};

/* s/w data filter */
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][BMG_AXES_NUM];
	int sum[BMG_AXES_NUM];
	int num;
	int idx;
};

/*struct bhy_data_bus {
	struct device *dev;
	s32(*read)(struct device *dev, u8 reg, u8 *data, u16 len);
	s32(*write)(struct device *dev, u8 reg, const u8 *data, u16 len);
	int irq;
	int bus_type;
};*/

struct bmg_i2c_data {
	struct i2c_client *client;
	struct gyro_hw hw;
	struct hwmsen_convert   cvt;
	struct bhy_data_bus data_bus;
	/* sensor info */
	u8 sensor_name[MAX_SENSOR_NAME];
	int datarate;
	/* sensitivity = 2^bitnum/range
	[+/-2000 = 4000; +/-1000 = 2000;
	 +/-500 = 1000; +/-250 = 500;
	 +/-125 = 250 ] */
	u16 sensitivity;
	/*misc*/
	struct mutex mutex_bus_op;
	struct mutex lock;
	atomic_t	trace;
	atomic_t	suspend;
	atomic_t	filter;
	/* unmapped axis value */
	s16	cali_sw[BMG_AXES_NUM+1];
	/* hw offset +1:for 4-byte alignment */
	s8	offset[BMG_AXES_NUM+1];

#if defined(CONFIG_BMG_LOWPASS)
	atomic_t	firlen;
	atomic_t	fir_en;
	struct data_filter	fir;
#endif
};

//static struct bhy_client_data *obj_i2c_data;
extern struct bhy_client_data *obj_i2c_data;
//struct gyro_hw gyro_cust;
//static struct gyro_hw *hw = &gyro_cust;
static struct gyro_init_info bmi160_gyro_init_info;
static int bmi160_gyro_init_flag = -1;
static struct i2c_driver bmg_i2c_driver;
static struct bmg_i2c_data *obj_i2c_data_bhy_gyro;
static const struct i2c_device_id bmi160_gyro_i2c_id[] = {
	{BMG_DEV_NAME, 0},
	{}
};

static bool sensor_power = true;

#define BHY_MAX_RETRY_I2C_XFER		10
#define BHY_I2C_WRITE_DELAY_TIME	1000
#define BHY_I2C_MAX_BURST_WRITE_LEN	64

#define BHY_SUPPORT_I2C_DMA 1
#if BHY_SUPPORT_I2C_DMA
    #include <linux/dma-mapping.h>
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
#define BHY_DMA_MAX_TRANSACTION_LENGTH  63
#define BHY_DMA_MAX_ALLOCATE  64

#endif

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

			client->addr = ((client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG);
			ret = i2c_master_recv(client, (u8 *)(uintptr_t)gpDMABuf_pa, len);

			for(i = 0; i < len; i++)
	        {
	            data[i] = gpDMABuf_va[i];
	        }
		client->addr = ((client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG));

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
	client = to_i2c_client(dev);
	return bhy_i2c_read_internal(client, reg, data, len);
}

static s32 bhy_i2c_write(struct device *dev, u8 reg, const u8 *data, u16 len)
{
	struct i2c_client *client;
	client = to_i2c_client(dev);
	return bhy_i2c_write_internal(client, reg, data, len);
}

static int bhy_read_reg(struct bmg_i2c_data *client_data,
		u8 reg, u8 *data, u16 len)
{
	if (client_data == NULL)
		return -EIO;
	return client_data->data_bus.read(client_data->data_bus.dev,
		reg, data, len);
}

static int bhy_write_reg(struct bmg_i2c_data *client_data,
		u8 reg, const u8 *data, u16 len)
{
	if (client_data == NULL)
		return -EIO;
	return client_data->data_bus.write(client_data->data_bus.dev,
		reg, data, len);
}

/* Dump BHy registers from 0x32 to 0xB5 */
static int bhy_dump_registers(struct bmg_i2c_data *client_data, char *buf,
	const char *line_hint)
{
	u8 reg_val[132];
	int i;
	int len;
	int ret;

	ret = bhy_read_reg(client_data, 0x32, reg_val, sizeof(reg_val));
	if (ret < 0) {
		snprintf(buf, 32, "Dump register failed");
		GYRO_PR_ERR("Dump register failed");
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

static int bhy_write_parameter(struct bmg_i2c_data *client_data,
		u8 page_num, u8 param_num, const u8 *data, u8 len)
{
	int ret, ret2;
	int retry = BHY_PARAM_ACK_WAIT_RETRY;
	u8 param_num_mod, ack, u8_val;
	char dump[256 * 3];

	/* Write param data */
	ret = bhy_write_reg(client_data, BHY_REG_LOAD_PARAM_0, data, len);
	if (ret < 0) {
		GYRO_PR_ERR("Write load parameter failed");
		goto bhy_write_parameter_exit;
	}
	/* Select page */
	ret = bhy_write_reg(client_data, BHY_REG_PARAM_PAGE_SEL, &page_num, 1);
	if (ret < 0) {
		GYRO_PR_ERR("Write page request failed");
		goto bhy_write_parameter_exit;
	}
	/* Select param */
	param_num_mod = param_num | 0x80;
	ret = bhy_write_reg(client_data, BHY_REG_PARAM_REQ, &param_num_mod, 1);
	if (ret < 0) {
		GYRO_PR_ERR("Write param request failed");
		goto bhy_write_parameter_exit;
	}
	/* Wait for ack */
	while (retry--) {
		ret = bhy_read_reg(client_data, BHY_REG_PARAM_ACK, &ack, 1);
		if (ret < 0) {
			GYRO_PR_ERR("Read ack reg failed");
			goto bhy_write_parameter_exit;
		}
		if (ack == 0x80) {
			GYRO_PR_ERR("Param is not accepted");
			ret = -EINVAL;
			goto bhy_write_parameter_exit;
		}
		if (ack == param_num_mod)
			break;
		usleep_range(10000, 20000);
	}
	if (retry == -1) {
		GYRO_PR_ERR("Wait for ack failed[%d, %d]", page_num, param_num);
		ret = -EBUSY;
		goto bhy_write_parameter_exit;
	}
bhy_write_parameter_exit:
	if (ret < 0) {
		bhy_dump_registers(client_data, dump, "BHY");
		GYRO_LOG("%s", dump);
	}
	/* Clear up */
	u8_val = 0;
	ret2 = bhy_write_reg(client_data, BHY_REG_PARAM_PAGE_SEL, &u8_val, 1);
	if (ret2 < 0) {
		GYRO_PR_ERR("Write page sel failed on clear up");
		return ret2;
	}
	u8_val = 0;
	ret2 = bhy_write_reg(client_data, BHY_REG_PARAM_REQ, &u8_val, 1);
	if (ret2 < 0) {
		GYRO_PR_ERR("Write param_req failed on clear up");
		return ret2;
	}
	retry = BHY_PARAM_ACK_WAIT_RETRY;
	while (retry--) {
		ret2 = bhy_read_reg(client_data, BHY_REG_PARAM_ACK, &ack, 1);
		if (ret2 < 0) {
			GYRO_PR_ERR("Read ack reg failed");
			return ret2;
		}
		if (ack == 0)
			break;
		msleep(10);
	}
	if (retry == 0)
		GYRO_LOG("BHY_REG_PARAM_ACK cannot revert to 0 after clear up");
	if (ret < 0)
		return ret;
	return len;
}

static int bhy_set_sensor_conf(struct bmg_i2c_data *client_data,
	int handle, const u8 *conf)
{
	int i;
	__le16 swap_data;
	u8 data[8];
	int ret;
	GYRO_LOG("bhy_set_sensor_conf for gyro.\n");
	for (i = 0; i < 4; ++i) {
		swap_data = cpu_to_le16(*(u16 *)(conf + i * 2));
		memcpy(data + i * 2, &swap_data, sizeof(swap_data));
	}
	mutex_lock(&obj_i2c_data->mutex_bus_op);
	ret = bhy_write_parameter(client_data, BHY_PAGE_SENSOR,
		BHY_PARAM_SENSOR_CONF_0 + handle,
		(u8 *)data, 8);
	mutex_unlock(&obj_i2c_data->mutex_bus_op);
	if (ret < 0) {
		GYRO_PR_ERR("Write parameter error");
		return ret;
	}
	printk("Set sensor[%d] conf: %02X %02X %02X %02X %02X %02X %02X %02X",
		handle, data[0], data[1], data[2], data[3], data[4], data[5],
		data[6], data[7]);

	return 0;
}


#if 0
/* I2C operation functions */
static int bmg_i2c_read_block(struct i2c_client *client, u8 addr,
				u8 *data, u8 len)
{
	int err;
	u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &beg
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		}
	};
	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
		GYRO_PR_ERR("length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		GYRO_PR_ERR("i2c_transfer error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	return err;
}

static int bmg_i2c_write_block(struct i2c_client *client, u8 addr,
				u8 *data, u8 len)
{
	/*
	*because address also occupies one byte,
	*the maximum length for write is 7 bytes
	*/
	int err = 0;
	int idx = 0;
	int num = 0;
	char buf[C_I2C_FIFO_SIZE];
	if (!client)
		return -EINVAL;
	else if (len >= C_I2C_FIFO_SIZE) {
		GYRO_PR_ERR("length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];
	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		GYRO_PR_ERR("send command error.\n");
		return -EFAULT;
	} else {
		err = 0;
	}
	return err;
}
#endif
static int bmg_read_raw_data(struct i2c_client *client, s16 data[BMG_AXES_NUM])
{
	int err = 0;
#ifdef CONFIG_BMG_LOWPASS
	struct bmg_i2c_data *priv = obj_i2c_data_bhy_gyro;
#endif

	data[BMG_AXIS_X] = gyro_data[0];
	data[BMG_AXIS_Y] = gyro_data[1];
	data[BMG_AXIS_Z] = gyro_data[2];
	GYRO_LOG("[16bit raw] [%08X %08X %08X] => [%5d %5d %5d]\n",
		data[BMG_AXIS_X], data[BMG_AXIS_Y], data[BMG_AXIS_Z],
		data[BMG_AXIS_X], data[BMG_AXIS_Y], data[BMG_AXIS_Z]);

#ifdef CONFIG_BMG_LOWPASS
/*
*Example: firlen = 16, filter buffer = [0] ... [15],
*when 17th data come, replace [0] with this new data.
*Then, average this filter buffer and report average value to upper layer.
*/
	if (atomic_read(&priv->filter)) {
		if (atomic_read(&priv->fir_en) &&
				 !atomic_read(&priv->suspend)) {
			int idx, firlen = atomic_read(&priv->firlen);
			if (priv->fir.num < firlen) {
				priv->fir.raw[priv->fir.num][BMG_AXIS_X] =
						data[BMG_AXIS_X];
				priv->fir.raw[priv->fir.num][BMG_AXIS_Y] =
						data[BMG_AXIS_Y];
				priv->fir.raw[priv->fir.num][BMG_AXIS_Z] =
						data[BMG_AXIS_Z];
				priv->fir.sum[BMG_AXIS_X] += data[BMG_AXIS_X];
				priv->fir.sum[BMG_AXIS_Y] += data[BMG_AXIS_Y];
				priv->fir.sum[BMG_AXIS_Z] += data[BMG_AXIS_Z];
				if (atomic_read(&priv->trace)&GYRO_TRC_FILTER) {
					GYRO_LOG("add [%2d]\n\n"
					"[%5d %5d %5d] => [%5d %5d %5d]\n",
					priv->fir.num,
					priv->fir.raw
					[priv->fir.num][BMG_AXIS_X],
					priv->fir.raw
					[priv->fir.num][BMG_AXIS_Y],
					priv->fir.raw
					[priv->fir.num][BMG_AXIS_Z],
					priv->fir.sum[BMG_AXIS_X],
					priv->fir.sum[BMG_AXIS_Y],
					priv->fir.sum[BMG_AXIS_Z]);
				}
				priv->fir.num++;
				priv->fir.idx++;
			} else {
				idx = priv->fir.idx % firlen;
				priv->fir.sum[BMG_AXIS_X] -=
					priv->fir.raw[idx][BMG_AXIS_X];
				priv->fir.sum[BMG_AXIS_Y] -=
					priv->fir.raw[idx][BMG_AXIS_Y];
				priv->fir.sum[BMG_AXIS_Z] -=
					priv->fir.raw[idx][BMG_AXIS_Z];
				priv->fir.raw[idx][BMG_AXIS_X] =
					data[BMG_AXIS_X];
				priv->fir.raw[idx][BMG_AXIS_Y] =
					data[BMG_AXIS_Y];
				priv->fir.raw[idx][BMG_AXIS_Z] =
					data[BMG_AXIS_Z];
				priv->fir.sum[BMG_AXIS_X] +=
					data[BMG_AXIS_X];
				priv->fir.sum[BMG_AXIS_Y] +=
					data[BMG_AXIS_Y];
				priv->fir.sum[BMG_AXIS_Z] +=
					data[BMG_AXIS_Z];
				priv->fir.idx++;
				data[BMG_AXIS_X] =
					priv->fir.sum[BMG_AXIS_X]/firlen;
				data[BMG_AXIS_Y] =
					priv->fir.sum[BMG_AXIS_Y]/firlen;
				data[BMG_AXIS_Z] =
					priv->fir.sum[BMG_AXIS_Z]/firlen;
				if (atomic_read(&priv->trace)&GYRO_TRC_FILTER) {
					GYRO_LOG("add [%2d]\n\n"
					"[%5d %5d %5d] =>\n\n"
					"[%5d %5d %5d] : [%5d %5d %5d]\n", idx,
					priv->fir.raw[idx][BMG_AXIS_X],
					priv->fir.raw[idx][BMG_AXIS_Y],
					priv->fir.raw[idx][BMG_AXIS_Z],
					priv->fir.sum[BMG_AXIS_X],
					priv->fir.sum[BMG_AXIS_Y],
					priv->fir.sum[BMG_AXIS_Z],
					data[BMG_AXIS_X],
					data[BMG_AXIS_Y],
					data[BMG_AXIS_Z]);
				}
			}
		}
	}
#endif
	return err;
}

#ifndef SW_CALIBRATION
/* get hardware offset value from chip register */
static int bmg_get_hw_offset(struct i2c_client *client,
		s8 offset[BMG_AXES_NUM + 1])
{
	int err = 0;
	/* HW calibration is under construction */
	GYRO_LOG("hw offset x=%x, y=%x, z=%x\n",
	offset[BMG_AXIS_X], offset[BMG_AXIS_Y], offset[BMG_AXIS_Z]);
	return err;
}
#endif

#ifndef SW_CALIBRATION
/* set hardware offset value to chip register*/
static int bmg_set_hw_offset(struct i2c_client *client,
		s8 offset[BMG_AXES_NUM + 1])
{
	int err = 0;
	/* HW calibration is under construction */
	GYRO_LOG("hw offset x=%x, y=%x, z=%x\n",
	offset[BMG_AXIS_X], offset[BMG_AXIS_Y], offset[BMG_AXIS_Z]);
	return err;
}
#endif

static int bmg_reset_calibration(struct i2c_client *client)
{
	int err = 0;
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
#ifdef SW_CALIBRATION

#else
	err = bmg_set_hw_offset(client, obj->offset);
	if (err) {
		GYRO_PR_ERR("read hw offset failed, %d\n", err);
		return err;
	}
#endif
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;
}

static int bmg_read_calibration(struct i2c_client *client,
	int act[BMG_AXES_NUM], int raw[BMG_AXES_NUM])
{
	/*
	*raw: the raw calibration data, unmapped;
	*act: the actual calibration data, mapped
	*/
	int err = 0;
	int mul;
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;

#ifdef SW_CALIBRATION
	/* only sw calibration, disable hw calibration */
	mul = 0;
#else
	err = bmg_get_hw_offset(client, obj->offset);
	if (err) {
		GYRO_PR_ERR("read hw offset failed, %d\n", err);
		return err;
	}
	mul = 1; /* mul = sensor sensitivity / offset sensitivity */
#endif

	raw[BMG_AXIS_X] =
			obj->offset[BMG_AXIS_X]*mul + obj->cali_sw[BMG_AXIS_X];
	raw[BMG_AXIS_Y] =
			obj->offset[BMG_AXIS_Y]*mul + obj->cali_sw[BMG_AXIS_Y];
	raw[BMG_AXIS_Z] =
			obj->offset[BMG_AXIS_Z]*mul + obj->cali_sw[BMG_AXIS_Z];

	act[obj->cvt.map[BMG_AXIS_X]] =
			obj->cvt.sign[BMG_AXIS_X]*raw[BMG_AXIS_X];
	act[obj->cvt.map[BMG_AXIS_Y]] =
			obj->cvt.sign[BMG_AXIS_Y]*raw[BMG_AXIS_Y];
	act[obj->cvt.map[BMG_AXIS_Z]] =
			obj->cvt.sign[BMG_AXIS_Z]*raw[BMG_AXIS_Z];
	return err;
}

static int bmg_write_calibration(struct i2c_client *client,
	int dat[BMG_AXES_NUM])
{
	/* dat array : Android coordinate system, mapped, unit:LSB */
	int err = 0;
	int cali[BMG_AXES_NUM] = {0};
	int raw[BMG_AXES_NUM] = {0};
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	/*offset will be updated in obj->offset */
	err = bmg_read_calibration(client, cali, raw);
	if (err) {
		GYRO_PR_ERR("read offset fail, %d\n", err);
		return err;
	}
	/* calculate the real offset expected by caller */
	cali[BMG_AXIS_X] += dat[BMG_AXIS_X];
	cali[BMG_AXIS_Y] += dat[BMG_AXIS_Y];
	cali[BMG_AXIS_Z] += dat[BMG_AXIS_Z];
	GYRO_LOG("UPDATE: add mapped data(%+3d %+3d %+3d)\n",
		dat[BMG_AXIS_X], dat[BMG_AXIS_Y], dat[BMG_AXIS_Z]);

#ifdef SW_CALIBRATION
	/* obj->cali_sw array : chip coordinate system, unmapped,unit:LSB */
	obj->cali_sw[BMG_AXIS_X] =
		obj->cvt.sign[BMG_AXIS_X]*(cali[obj->cvt.map[BMG_AXIS_X]]);
	obj->cali_sw[BMG_AXIS_Y] =
		obj->cvt.sign[BMG_AXIS_Y]*(cali[obj->cvt.map[BMG_AXIS_Y]]);
	obj->cali_sw[BMG_AXIS_Z] =
		obj->cvt.sign[BMG_AXIS_Z]*(cali[obj->cvt.map[BMG_AXIS_Z]]);
#else
	/* divisor = sensor sensitivity / offset sensitivity */
	int divisor = 1;
	obj->offset[BMG_AXIS_X] = (s8)(obj->cvt.sign[BMG_AXIS_X]*
		(cali[obj->cvt.map[BMG_AXIS_X]])/(divisor));
	obj->offset[BMG_AXIS_Y] = (s8)(obj->cvt.sign[BMG_AXIS_Y]*
		(cali[obj->cvt.map[BMG_AXIS_Y]])/(divisor));
	obj->offset[BMG_AXIS_Z] = (s8)(obj->cvt.sign[BMG_AXIS_Z]*
		(cali[obj->cvt.map[BMG_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[BMG_AXIS_X] = obj->cvt.sign[BMG_AXIS_X]*
		(cali[obj->cvt.map[BMG_AXIS_X]])%(divisor);
	obj->cali_sw[BMG_AXIS_Y] = obj->cvt.sign[BMG_AXIS_Y]*
		(cali[obj->cvt.map[BMG_AXIS_Y]])%(divisor);
	obj->cali_sw[BMG_AXIS_Z] = obj->cvt.sign[BMG_AXIS_Z]*
		(cali[obj->cvt.map[BMG_AXIS_Z]])%(divisor);
	/* HW calibration is under construction */
	err = bmg_set_hw_offset(client, obj->offset);
	if (err) {
		GYRO_PR_ERR("read hw offset failed.\n");
		return err;
	}
#endif
	return err;
}

static int bmg_get_chip_type(void)
{
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	strlcpy(obj->sensor_name, BMG_DEV_NAME, 48);
	return 0;
}

static int bmg_set_datarate(int datarate)
{
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	obj->datarate = datarate;
	return 0;
}

static int bmg_init_client(void)
{
	int err = 0;
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	err = bmg_get_chip_type();
	err = bmg_set_datarate(100);
	obj->sensitivity = 16;
	if (0 != err) {
		GYRO_PR_ERR("bmg init client failed.\n");
		return err;
	}
#ifdef CONFIG_BMG_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif
	return 0;
}

/*
*Returns compensated and mapped value. unit is :degree/second
*/
static int bmg_read_sensor_data(struct i2c_client *client,
		char *buf, int bufsize)
{
	int gyro[BMG_AXES_NUM] = {0};
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	//printk("read raw data, x = %d, y = %d, z = %d.\n",gyro_data[0],gyro_data[1],gyro_data[2]);

#if 0 /*bhy is sensor hub, so remapping/calibration had been done inside sensorhub*/
	/* compensate data */
	gyro_data[BMG_AXIS_X] += obj->cali_sw[BMG_AXIS_X];
	gyro_data[BMG_AXIS_Y] += obj->cali_sw[BMG_AXIS_Y];
	gyro_data[BMG_AXIS_Z] += obj->cali_sw[BMG_AXIS_Z];

	/* remap coordinate */
	gyro[obj->cvt.map[BMG_AXIS_X]] =
		obj->cvt.sign[BMG_AXIS_X]*gyro_data[BMG_AXIS_X];
	gyro[obj->cvt.map[BMG_AXIS_Y]] =
		obj->cvt.sign[BMG_AXIS_Y]*gyro_data[BMG_AXIS_Y];
	gyro[obj->cvt.map[BMG_AXIS_Z]] =
		obj->cvt.sign[BMG_AXIS_Z]*gyro_data[BMG_AXIS_Z];
#endif
	//gyro[obj->cvt.map[BMG_AXIS_X]] = gyro_data[BMG_AXIS_X];
	//gyro[obj->cvt.map[BMG_AXIS_Y]] = gyro_data[BMG_AXIS_Y];
	//gyro[obj->cvt.map[BMG_AXIS_Z]] = gyro_data[BMG_AXIS_Z];

	gyro[BMG_AXIS_X] = gyro_data[BMG_AXIS_X];
	gyro[BMG_AXIS_Y] = gyro_data[BMG_AXIS_Y];
	gyro[BMG_AXIS_Z] = gyro_data[BMG_AXIS_Z];
	/* convert: LSB -> degree/second(o/s) */
	gyro[BMG_AXIS_X] = (gyro[BMG_AXIS_X] * 1000) / obj->sensitivity;
	gyro[BMG_AXIS_Y] = (gyro[BMG_AXIS_Y] * 1000) / obj->sensitivity;
	gyro[BMG_AXIS_Z] = (gyro[BMG_AXIS_Z] * 1000) / obj->sensitivity;
	/*GYRO_LOG("gyro final xyz data: %d,%d,%d, sensitivity:%d\n",
		gyro[BMG_AXIS_X], gyro[BMG_AXIS_Y], gyro[BMG_AXIS_Z], obj->sensitivity);*/
	snprintf(buf, 96, "%04x %04x %04x",
		gyro[BMG_AXIS_X], gyro[BMG_AXIS_Y], gyro[BMG_AXIS_Z]);
	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	return snprintf(buf, PAGE_SIZE, "%s\n", obj->sensor_name);
}

/*
* sensor data format is hex, unit:degree/second
*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	char strbuf[BMG_BUFSIZE] = {0};
	bmg_read_sensor_data(obj->client, strbuf, BMG_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*
* raw data format is s16, unit:LSB, axis mapped
*/
static ssize_t show_rawdata_value(struct device_driver *ddri, char *buf)
{
	s16 databuf[BMG_AXES_NUM] = {0};
	s16 dataraw[BMG_AXES_NUM] = {0};
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	bmg_read_raw_data(obj->client, dataraw);
	/*remap coordinate*/
	databuf[obj->cvt.map[BMG_AXIS_X]] =
			obj->cvt.sign[BMG_AXIS_X]*dataraw[BMG_AXIS_X];
	databuf[obj->cvt.map[BMG_AXIS_Y]] =
			obj->cvt.sign[BMG_AXIS_Y]*dataraw[BMG_AXIS_Y];
	databuf[obj->cvt.map[BMG_AXIS_Z]] =
			obj->cvt.sign[BMG_AXIS_Z]*dataraw[BMG_AXIS_Z];
	return snprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
			databuf[BMG_AXIS_X],
			databuf[BMG_AXIS_Y],
			databuf[BMG_AXIS_Z]);
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	int err = 0;
	int len = 0;
	int mul = 0;
	int act[BMG_AXES_NUM] = {0};
	int raw[BMG_AXES_NUM] = {0};
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;

	mul = len;
	err = bmg_read_calibration(obj->client, act, raw);
	if (err)
		return -EINVAL;
	else {
		mul = 1; /* mul = sensor sensitivity / offset sensitivity */
		len += snprintf(buf+len, PAGE_SIZE-len,
			"[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n",
			mul,
			obj->offset[BMG_AXIS_X], obj->offset[BMG_AXIS_Y], obj->offset[BMG_AXIS_Z],
			obj->offset[BMG_AXIS_X], obj->offset[BMG_AXIS_Y], obj->offset[BMG_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
			obj->cali_sw[BMG_AXIS_X], obj->cali_sw[BMG_AXIS_Y], obj->cali_sw[BMG_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len,
			"[ALL]unmapped(%+3d, %+3d, %+3d), mapped(%+3d, %+3d, %+3d)\n",
			raw[BMG_AXIS_X], raw[BMG_AXIS_Y], raw[BMG_AXIS_Z],
			act[BMG_AXIS_X], act[BMG_AXIS_Y], act[BMG_AXIS_Z]);
		return len;
	}
}

static ssize_t store_cali_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	int err = 0;
	unsigned int dat[BMG_AXES_NUM] = {0};
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	if (!strncmp(buf, "rst", 3)) {
		err = bmg_reset_calibration(obj->client);
		if (err)
			GYRO_PR_ERR("reset offset err = %d\n", err);
	} else if (BMG_AXES_NUM == sscanf(buf, "0x%02X 0x%02X 0x%02X",
		&dat[BMG_AXIS_X], &dat[BMG_AXIS_Y], &dat[BMG_AXIS_Z])) {
		err = bmg_write_calibration(obj->client, dat);
		if (err) {
			GYRO_PR_ERR("bmg write calibration failed, err = %d\n",
				err);
		}
	} else {
		GYRO_PR_ERR("invalid format\n");
	}
	return count;
}

static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "not support\n");
}

static ssize_t store_firlen_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	unsigned int trace;
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	if (1 == sscanf(buf, "0x%2x", &trace))
		atomic_set(&obj->trace, trace);
	else
		GYRO_PR_ERR("invalid content: '%s'\n", buf);
	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	//if (obj->hw)
		len += snprintf(buf+len, PAGE_SIZE-len,
			"CUST: %d %d (%d %d)\n",
			obj->hw.i2c_num, obj->hw.direction,
			obj->hw.power_id, obj->hw.power_vol);

	len += snprintf(buf+len, PAGE_SIZE-len, "i2c addr:%#x,ver:%s\n",
		obj->client->addr, BMG_DRIVER_VERSION);
	return len;
}

static ssize_t show_power_mode_value(struct device_driver *ddri, char *buf)
{
	if (sensor_power)
		GYRO_LOG("G sensor in work mode, sensor_power = %d\n", sensor_power);
	else
		GYRO_LOG("G sensor in suspend mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, 32, "%d\n", sensor_power);
}

static ssize_t show_run_fast_calibration(struct device_driver *ddri, char *buf)
{

	u8 data = 0x00;

	GYRO_LOG("BHI160_GYRO show_run_fast_calibration_value ");

	return sprintf(buf, "%d\n", data);
}

static ssize_t store_run_fast_calibration(struct device_driver *ddri,
						const char *buf, size_t count)
{

	GYRO_LOG("BHI160_GYRO store_run_fast_calibration_value ");

	return count;
}

static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(rawdata, S_IWUSR | S_IRUGO, show_rawdata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO,
show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO,
show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(gyro_op_mode, S_IWUSR | S_IRUGO,
show_power_mode_value, NULL);
static DRIVER_ATTR(run_fast_calibration, S_IRUGO | S_IWUSR | S_IWGRP,
		   show_run_fast_calibration, store_run_fast_calibration);


static struct driver_attribute *bmg_attr_list[] = {
	/* chip information */
	&driver_attr_chipinfo,
	/* dump sensor data */
	&driver_attr_sensordata,
	/* dump raw data */
	&driver_attr_rawdata,
	/* show calibration data */
	&driver_attr_cali,
	/* filter length: 0: disable, others: enable */
	&driver_attr_firlen,
	/* trace flag */
	&driver_attr_trace,
	/* get hw configuration */
	&driver_attr_status,
	/* get power mode */
	&driver_attr_gyro_op_mode,
	/* calibration */
	&driver_attr_run_fast_calibration,
};

static int bmg_create_attr(struct device_driver *driver)
{
	int idx = 0;
	int err = 0;
	int num = (int)(sizeof(bmg_attr_list)/sizeof(bmg_attr_list[0]));
	if (driver == NULL)
		return -EINVAL;
	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, bmg_attr_list[idx]);
		if (err) {
			GYRO_PR_ERR("driver_create_file (%s) = %d\n",
				bmg_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int bmg_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(bmg_attr_list)/sizeof(bmg_attr_list[0]));
	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, bmg_attr_list[idx]);
	return err;
}

#if 0
static int bmg_open(struct inode *inode, struct file *file)
{
	file->private_data = obj_i2c_data_bhy_gyro;
	if (file->private_data == NULL) {
		GYRO_PR_ERR("file->private_data is null pointer.\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int bmg_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long bmg_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int err = 0;
	char strbuf[BMG_BUFSIZE] = {0};
	int raw_offset[BMG_BUFSIZE] = {0};
	void __user *data;
	SENSOR_DATA sensor_data;
	int cali[BMG_AXES_NUM] = {0};
	struct bmg_i2c_data *obj = (struct bmg_i2c_data *)file->private_data;
	struct i2c_client *client = obj->client;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) {
		GYRO_PR_ERR("access error: %08x, (%2d, %2d)\n",
			cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}
	switch (cmd) {
	case GYROSCOPE_IOCTL_INIT:
		bmg_init_client();
		break;
	case GYROSCOPE_IOCTL_READ_SENSORDATA:
		data = (void __user *) arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		bmg_read_sensor_data(client, strbuf, BMG_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;
	case GYROSCOPE_IOCTL_SET_CALI:
		/* data unit is degree/second */
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
			GYRO_PR_ERR("perform calibration in suspend mode\n");
			err = -EINVAL;
		} else {
			/* convert: degree/second -> LSB */
			cali[BMG_AXIS_X] = sensor_data.x * obj->sensitivity;
			cali[BMG_AXIS_Y] = sensor_data.y * obj->sensitivity;
			cali[BMG_AXIS_Z] = sensor_data.z * obj->sensitivity;
			/* err = bmg_write_calibration(client, cali); */
		}
		break;
	case GYROSCOPE_IOCTL_CLR_CALI:
		err = bmg_reset_calibration(client);
		break;
	case GYROSCOPE_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		err = bmg_read_calibration(client, cali, raw_offset);
		if (err)
			break;

		sensor_data.x = cali[BMG_AXIS_X] * obj->sensitivity;
		sensor_data.y = cali[BMG_AXIS_Y] * obj->sensitivity;
		sensor_data.z = cali[BMG_AXIS_Z] * obj->sensitivity;
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		break;
	default:
		GYRO_PR_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}
	return err;
}

static const struct file_operations bmg_fops = {
	.owner = THIS_MODULE,
	.open = bmg_open,
	.release = bmg_release,
	.unlocked_ioctl = bmg_unlocked_ioctl,
};

static struct miscdevice bmg_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gyroscope",
	.fops = &bmg_fops,
};
#endif

static int bmi160_gyro_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;
	int err = 0;
	if (msg.event == PM_EVENT_SUSPEND)
		atomic_set(&obj->suspend, 1);
	return err;
}

static int bmi160_gyro_resume(struct i2c_client *client)
{
	struct bmg_i2c_data *obj = obj_i2c_data_bhy_gyro;

	atomic_set(&obj->suspend, 0);
	return 0;
}

static int bmi160_gyro_i2c_detect(struct i2c_client *client,
		struct i2c_board_info *info)
{
	strlcpy(info->type, BMG_DEV_NAME, 48);
	return 0;
}

static int bmi160_gyro_open_report_data(int open)
{
	return 0;
}

static int bmi160_gyro_enable_nodata(int en)
{
	int err = 0;
	u8 conf[8];
	u16 tmp;

	if (1 == en)
		sensor_power = true;
	else
		sensor_power = false;
	GYRO_LOG("bmi160_gyro_enable_nodata en = %d.",en);

	tmp = (u16)en;
	memcpy(conf, &tmp, 2);
	memset(conf + 2, 0, 2);
	memset(conf + 4, 0, 4);

	/* 1-> Acc, 4 -> Gyro */
	err = bhy_set_sensor_conf(obj_i2c_data_bhy_gyro, 4, conf);
	if (err < 0)
		GYRO_LOG("bmi160_gyro_SetPowerMode fail!\n");
	else
		GYRO_LOG("bmi160_gyro_SetPowerMode ok.\n");

	return err;
}

static int bmi160_gyro_set_delay(u64 ns)
{
	int err;
	u8 conf[8];
	u16 sample_delay = 0;
	int value = (int)ns / 1000 / 1000;	
	struct bmg_i2c_data *priv = obj_i2c_data_bhy_gyro;
	/* Currently, set max data rate to 100Hz as MTK platform */
	if(value <= 10)
	{
	  sample_delay = 100;
	}
	else
	{
	  sample_delay = 1000 / value;
	}
	GYRO_LOG("bmi160_gyro_set_delay = (%d Hz)\n",sample_delay);

	memcpy(conf, &sample_delay, 2);
	memset(conf + 2, 0, 2);
	memset(conf + 4, 0, 4);

	err = bhy_set_sensor_conf(obj_i2c_data_bhy_gyro, 4, conf);
	if (err < 0)
		GYRO_PR_ERR("set data rate failed.\n");
	if (value >= 40)
		atomic_set(&priv->filter, 0);
	else {
#if defined(CONFIG_BMG_LOWPASS)
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[BMG_AXIS_X] = 0;
		priv->fir.sum[BMG_AXIS_Y] = 0;
		priv->fir.sum[BMG_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
#endif
	}
	return 0;
}

static int gyroscope_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs/1000/1000;

	GYRO_LOG("gyroscope_batch period=(%d ms)\n", value);
	return bmi160_gyro_set_delay(samplingPeriodNs);
}
static int gyroscope_flush(void)
{
	return gyro_flush_report();
}

static int bmi160_gyro_get_data(int *x, int *y, int *z, int *status)
{
	char buff[BMG_BUFSIZE] = {0};
	bmg_read_sensor_data(obj_i2c_data_bhy_gyro->client, buff, BMG_BUFSIZE);
	sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return 0;
}

static int bmi160_gyro_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	struct bmg_i2c_data *obj;
	struct gyro_control_path ctl = {0};
	struct gyro_data_path data = {0};
	struct bhy_data_bus data_bus;

	GYRO_LOG("%s: is begin, %p, %d, %p, %d\n", __func__, client, client->addr, id, bmi160_gyro_init_flag);

	if(bmi160_gyro_init_flag == 0)
		return 0;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	err = get_gyro_dts_func(client->dev.of_node, &obj->hw);
	if (err < 0) {
		GYRO_PR_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit_init_failed;
	}

	data_bus.read = bhy_i2c_read;
	data_bus.write = bhy_i2c_write;
	data_bus.dev = &bhy_bmi160_acc_i2c_client->dev;
	data_bus.irq = client->irq;
	data_bus.bus_type = BUS_I2C;

	err = hwmsen_get_convert(obj->hw.direction, &obj->cvt);
	if (err) {
		GYRO_PR_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit_hwmsen_get_convert_failed;
	}
	obj_i2c_data_bhy_gyro = obj;
	obj->client = bhy_bmi160_acc_i2c_client;
	i2c_set_clientdata(obj->client, obj);
	mutex_init(&obj->mutex_bus_op);

	dev_set_drvdata(data_bus.dev, obj);
	obj->data_bus = data_bus;

#ifdef BHY_SUPPORT_I2C_DMA
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&client->dev, BHY_DMA_MAX_ALLOCATE,&gpDMABuf_pa, GFP_KERNEL);
	if (!gpDMABuf_va) {
		printk("%s Allocate DMA I2C Buffer failed!\n", __func__);
		err = -EFAULT;
		goto exit_init_failed;
	}else
	{
		printk("DMA BUFF Allocate OK.\n");
	}	
 	memset(gpDMABuf_va, 0, BHY_DMA_MAX_ALLOCATE);
#endif
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	mutex_init(&obj->lock);
#ifdef CONFIG_BMG_LOWPASS
	if (obj->hw.firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw.firlen);

	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);
#endif
	bmg_init_client();
	err = bmg_create_attr(
		&bmi160_gyro_init_info.platform_diver_addr->driver);
	if (err) {
		GYRO_PR_ERR("create attribute failed, err = %d\n", err);
		goto exit_create_attr_failed;
	}
	ctl.open_report_data = bmi160_gyro_open_report_data;
	ctl.enable_nodata = bmi160_gyro_enable_nodata;
	//ctl.set_delay  = bmi160_gyro_set_delay;
	ctl.batch = gyroscope_batch;
	ctl.flush = gyroscope_flush;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw.is_batch_supported;
	err = gyro_register_control_path(&ctl);
	if (err) {
		GYRO_PR_ERR("register gyro control path err\n");
		goto exit_create_attr_failed;
	}
	data.get_data = bmi160_gyro_get_data;
	data.vender_div = DEGREE_TO_RAD;
	err = gyro_register_data_path(&data);
	if (err) {
		GYRO_PR_ERR("gyro_register_data_path fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	bmi160_gyro_init_flag = 0;
	GYRO_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:


exit_init_failed:

exit_hwmsen_get_convert_failed:
	kfree(obj);
	obj = NULL;
exit:
	bmi160_gyro_init_flag = -1;
	GYRO_PR_ERR("err = %d\n", err);
	return err;
}

static int bmi160_gyro_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = bmg_delete_attr(
		&bmi160_gyro_init_info.platform_diver_addr->driver);
	if (err)
		GYRO_PR_ERR("bmg_delete_attr failed, err = %d\n", err);


	obj_i2c_data_bhy_gyro = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gyro_of_match[] = {
	{.compatible = "mediatek,bmi160_gyro"},
	{},
};
#endif

static struct i2c_driver bmg_i2c_driver = {
	.driver = {
		.name = BMG_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = gyro_of_match,
#endif
	},
	.probe = bmi160_gyro_i2c_probe,
	.remove	= bmi160_gyro_i2c_remove,
	.detect	= bmi160_gyro_i2c_detect,
	.suspend = bmi160_gyro_suspend,
	.resume = bmi160_gyro_resume,
	.id_table = bmi160_gyro_i2c_id,
};

static int bmi160_gyro_remove(void)
{
	i2c_del_driver(&bmg_i2c_driver);
	return 0;
}

static int bmi160_gyro_local_init(struct platform_device *pdev)
{
	int err = 0;

#if 0
	/*need define in the DTS side for bmi160_gyro*/
	
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info;
#endif

	GYRO_LOG("bmi160_gyro_local_init start.\n");
#if 0
	adapter = i2c_get_adapter(3);
	if (adapter == NULL)
		printk(KERN_ERR"gsensor  error");
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = 0x61;
	strlcpy(info.type, "bmi160_gyro", I2C_NAME_SIZE);
	client = i2c_new_device(adapter, &info);
	strlcpy(client->name, "bmi160_gyro", I2C_NAME_SIZE);
#endif	
	if (i2c_add_driver(&bmg_i2c_driver)) {
		GYRO_PR_ERR("add gyro driver error.\n");
		err = -1;
		return err;
	}
	if (-1 == bmi160_gyro_init_flag) {
		err = -1;
		return err;
	}
	GYRO_LOG("bmi160 gyro init ok.\n");
	return 0;
}

static struct gyro_init_info bmi160_gyro_init_info = {
		.name = BMG_DEV_NAME,
		.init = bmi160_gyro_local_init,
		.uninit = bmi160_gyro_remove,
};

static int __init bmi160_gyro_init(void)
{
	GYRO_LOG("%s: bosch gyroscope driver version: %s\n",
	__func__, BMG_DRIVER_VERSION);
	gyro_driver_add(&bmi160_gyro_init_info);
	return 0;
}

static void __exit bmi160_gyro_exit(void)
{
	GYRO_FUN();
}

module_init(bmi160_gyro_init);
module_exit(bmi160_gyro_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMI160 GYRO Driver");
MODULE_AUTHOR("contact@bosch-sensortec.com>");
