/* BMI160_ACC motion sensor driver
 *
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 *
 * VERSION: V1.5
 * HISTORY: V1.0 --- Driver creation
 *          V1.1 --- Add share I2C address function
 *          V1.2 --- Fix the bug that sometimes sensor is stuck after system resume.
 *          V1.3 --- Add FIFO interfaces.
 *          V1.4 --- Use basic i2c function to read fifo data instead of i2c DMA mode.
 *          V1.5 --- Add compensated value performed by MTK acceleration calibration process.
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/module.h>
#include <hwmsensor.h>
//#include <hwmsen_dev.h>
#include <sensors_io.h>
//#include <hwmsen_helper.h>
#include <accel.h>
#include <cust_acc.h>

#define CALIBRATION_TO_FILE
#ifdef CALIBRATION_TO_FILE
#include "../../sensor_cal/sensor_cal_file_io.h"
#endif
#include <linux/of_irq.h>
#include <mt_gpio.h>
#include <linux/gpio.h>
#include <mach/gpio_const.h>


#include "bmi160_acc.h"

#define BMI160_SELFTEST //LGE_CHANGE
#ifdef CONFIG_LGE_SENSOR_SLOPE_DETECTOR
#define STEP_DETECT
#define ENABLE_INT1
#endif
#define SW_CALIBRATION
#define FIFO_READ_USE_DMA_MODE_I2C
#define BYTES_PER_LINE						(16)
#define ACCEL_DMA_MAX_TRANSACTION_LENGTH 	1003

#ifdef CONFIG_LGE_SENSOR_SLOPE_DETECTOR
extern int bmi160_stc_notify(void);
#endif

static const struct i2c_device_id bmi160_acc_i2c_id[] = {{BMI160_DEV_NAME,0},{}};

static int bmi160_set_command_register(u8 cmd_reg);
static int bmi160_acc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int bmi160_acc_i2c_remove(struct i2c_client *client);
static int bmi160_acc_local_init(void);
static int bmi160_acc_remove(void);
static int bmi160_acc_read_sensor_data(struct i2c_client *client, char *buf, int bufsize);


struct scale_factor{
	u8  whole;
	u8  fraction;
};

struct data_resolution {
	struct scale_factor scalefactor;
	int	sensitivity;
};

struct bmi160_acc_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert   cvt;
	struct bmi160_t device;
	/*misc*/
	struct data_resolution *reso;
	atomic_t	trace;
	atomic_t	suspend;
	atomic_t	selftest_result;
	atomic_t	filter;
	atomic_t	fast_calib_rslt;
#ifdef ENABLE_INT1
	struct device_nod *irq_node;
	int irq;
	struct work_struct irq_work;
#endif
#ifdef STEP_DETECT
	atomic_t step_detect_enable;
	int step_value;
	struct mutex enable_step_mutex;
#endif
	/* +5 for 4-byte alignment and calibraiton old value*/
	int	cali_sw[BMI160_ACC_AXES_NUM+5];
	int	cal_update;
	struct mutex lock;
	/* +1: for 4-byte alignment */
	s8		offset[BMI160_ACC_AXES_NUM+1];
	s16		data[BMI160_ACC_AXES_NUM+1];
	u8		fifo_count;

	u8 		fifo_head_en;
	u8  	fifo_data_sel;
	u16 	fifo_bytecount;
	struct 	odr_t odr;
	u64 	fifo_time;
	atomic_t 	layout;   

	struct delayed_work work;
	struct input_dev *input;
	struct hrtimer timer;
	int is_timer_running;
	struct work_struct report_data_work;
	ktime_t work_delay_kt;
	uint64_t time_odr;
	struct mutex mutex_ring_buf;
	atomic_t delay;
	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend    early_drv;
#endif
};

struct bmi160_axis_data_t {
	s16 x;
	s16 y;
	s16 z;
};

enum BMI_AXIS_TYPE{
	X_AXIS = 0,
	Y_AXIS,
	Z_AXIS,
	AXIS_MAX
};

struct i2c_client *bmi160_acc_i2c_client = NULL;
static struct acc_init_info bmi160_acc_init_info;
static struct bmi160_acc_i2c_data *obj_i2c_data;
static bool sensor_power = false;
static bool accel_enable_status = false;
static struct GSENSOR_VECTOR3D gsensor_gain;

/* 0=ok, -1=fail */
static int bmi160_acc_init_flag = -1;
struct bmi160_t *p_bmi160;

static struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;

static struct data_resolution bmi160_acc_data_resolution[] = {
	/*8 combination by {FULL_RES,RANGE} */
	{{0, 6}, 16384},	/*+/-2g  in 16-bit resolution:  0.06 mg/LSB */
	{{0, 12}, 8192},	/*+/-4g  in 16-bit resolution:  0.12 mg/LSB */
	{{0, 24}, 4096},	/*+/-8g  in 16-bit resolution:  0.24 mg/LSB */
	{{0, 5}, 2048},		/*+/-16g in 16-bit resolution:  0.49 mg/LSB */
};

static struct data_resolution bmi160_acc_offset_resolution = {{0, 12}, 8192};

#ifdef FIFO_READ_USE_DMA_MODE_I2C
#include <linux/dma-mapping.h>
#ifndef I2C_MASK_FLAG
#define I2C_MASK_FLAG   (0x00ff)
#define I2C_DMA_FLAG    (0x2000)
#define I2C_ENEXT_FLAG  (0x0200)
#endif

static u8 *gpDMABuf_va;
static dma_addr_t gpDMABuf_pa;

#if 0
static int i2c_dma_read_fifo(struct i2c_client *client,
			     uint8_t regaddr, uint8_t *readbuf, int32_t readlen)
{
	int ret;
	struct i2c_msg msg;

	ret = i2c_master_send(client, &regaddr, 1);
	if (ret < 0) {
		GSE_ERR("send command error!!\n");
		return -EFAULT;
	}
	msg.addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = readlen;
	msg.buf = (char *)I2CDMABuf_pa;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		GSE_ERR("dma receive data error!!\n");
		return -EFAULT;
	}
	memcpy(readbuf, I2CDMABuf_va, readlen);
	return ret;
}
#endif

static s32 i2c_dma_read(struct i2c_client *client, uint8_t addr, uint8_t *readbuf, int32_t readlen)
{
	int ret = 0;
	s32 retry = 0;
	u8 buffer[2];
	struct i2c_msg msg[2] = {
		{
			 .addr = (client->addr & I2C_MASK_FLAG),
			 .flags = 0,
			 .buf = buffer,
			 .len = 2,
			 /* .timing = I2C_MASTER_CLOCK */
		 },
		{
			 .addr = (client->addr & I2C_MASK_FLAG),
			 .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			 .flags = I2C_M_RD,
			 .buf = (u8 *)(uintptr_t) gpDMABuf_pa,
			 .len = readlen,
			 /* .timing = I2C_MASTER_CLOCK */
		 },
	};
	buffer[1] = (addr >> 8) & 0xFF;
	buffer[0] = addr & 0xFF;
	if (NULL == readbuf) {
		return -1;
	}
	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
			continue;
		memcpy(readbuf, gpDMABuf_va, readlen);
		return 0;
	}
	GSE_ERR("DMA I2C read error: 0x%04X, %d byte(s), err-code: %d", addr, readlen, ret);
	return ret;
}
#endif

/* I2C operation functions */
static int bma_i2c_read_block(struct i2c_client *client,
		u8 addr, u8 *data, u8 len)
{
	int err;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	if (!client)
		return -EINVAL;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

#if 0
	else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
#endif
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",
				addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;/*no error*/
	}
	return err;
}

static int bma_i2c_write_block(struct i2c_client *client, u8 addr,
		u8 *data, u8 len)
{
	/*
	 *because address also occupies one byte,
	 *the maximum length for write is 7 bytes
	 */
	int err, idx = 0, num = 0;
	char buf[32];
	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++) {
		buf[num++] = data[idx];
	}
	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		GSE_ERR("send command error!!\n");
		return -EFAULT;
	} else {
		err = 0;
	}
	return err;
}

s8 bmi_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err = 0;
	err = bma_i2c_read_block(bmi160_acc_i2c_client, reg_addr, data, len);
	if (err < 0) {
		GSE_ERR("read bmi160 i2c failed.\n");
	}
	return err;
}

s8 bmi_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err = 0;
	err = bma_i2c_write_block(bmi160_acc_i2c_client, reg_addr, data, len);
	if (err < 0) {
		GSE_ERR("read bmi160 i2c failed.\n");
	}
	return err;
}

static void bmi_delay(u32 msec)
{
	mdelay(msec);
}

/*!
 * @brief Set data resolution
 *
 * @param[in] client the pointer of bmi160_acc_i2c_data
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_set_resolution(struct bmi160_acc_i2c_data *obj)
{
	int res = 0;
	u8 dat, reso;
	mutex_lock(&obj->lock);
	res = bma_i2c_read_block(obj->client, BMI160_USER_ACC_RANGE__REG, &dat, 1);
	mutex_unlock(&obj->lock);
	if (res) {
		GSE_ERR("read data format fail!!\n");
		return res;
	}
	dat = dat & BMI160_USER_ACC_RANGE__MSK;
	reso = 0xFF;
	switch (dat) {
		case BMI160_ACCEL_RANGE_2G:
			reso = 0;
			break;
		case BMI160_ACCEL_RANGE_4G:
			reso = 1;
			break;
		case BMI160_ACCEL_RANGE_8G:
			reso = 2;
			break;
		case BMI160_ACCEL_RANGE_16G:
			reso = 3;
			break;
		default:
			break;
	}
	if (reso < sizeof(bmi160_acc_data_resolution) / sizeof(bmi160_acc_data_resolution[0])) {
		obj->reso = &bmi160_acc_data_resolution[reso];
		GSE_LOG("bmi160_acc_set_resolution: %d\n", obj->reso->sensitivity);
		return 0;
	} else {
		return -EINVAL;
	}
}

static int bmi160_acc_read_data(struct i2c_client *client, s16 data[BMI160_ACC_AXES_NUM])
{
	int err = 0;
	u8 addr = BMI160_USER_DATA_14_ACC_X_LSB__REG;
	u8 buf[BMI160_ACC_DATA_LEN] = {0};
	err = bma_i2c_read_block(client, addr, buf, BMI160_ACC_DATA_LEN);
	if(err) {
		GSE_ERR("read data failed.\n");
	}
	else {
		/* Convert sensor raw data to 16-bit integer */
		data[BMI160_ACC_AXIS_X] = (s16) ((((s32)((s8)buf[1]))
					<< BMI160_SHIFT_8_POSITION) | (buf[0]));
		data[BMI160_ACC_AXIS_Y] = (s16) ((((s32)((s8)buf[3]))
					<< BMI160_SHIFT_8_POSITION) | (buf[2]));
		data[BMI160_ACC_AXIS_Z] = (s16) ((((s32)((s8)buf[5]))
					<< BMI160_SHIFT_8_POSITION) | (buf[4]));
	}
	return err;
}

static int bmi160_acc_read_offset(struct i2c_client *client, s8 ofs[BMI160_ACC_AXES_NUM])
{
	int err = 0;
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#else
	err = bma_i2c_read_block(client, BMI160_ACC_REG_OFSX, ofs, BMI160_ACC_AXES_NUM);
	if(err) {
		GSE_ERR("error: %d\n", err);
	}
#endif
	return err;
}

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
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
#ifdef SW_CALIBRATION

#else
	u8 ofs[4]={0,0,0,0};
	err = bma_i2c_write_block(client, BMI160_ACC_REG_OFSX, ofs, 4);
	if(err) {
		GSE_ERR("write offset failed.\n");
	}
#endif
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;
}

static int bmi160_acc_read_calibration(struct i2c_client *client, int dat[BMI160_ACC_AXES_NUM])
{
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	int err = 0;
	int mul;
#ifdef SW_CALIBRATION
	mul = 0;//only SW Calibration, disable HW Calibration
#else
	err = bmi160_acc_read_offset(client, obj->offset);
	if(err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity/bmi160_acc_offset_resolution.sensitivity;
#endif

	dat[obj->cvt.map[BMI160_ACC_AXIS_X]] = obj->cvt.sign[BMI160_ACC_AXIS_X]
				*(obj->offset[BMI160_ACC_AXIS_X]*mul + obj->cali_sw[BMI160_ACC_AXIS_X]);
	dat[obj->cvt.map[BMI160_ACC_AXIS_Y]] = obj->cvt.sign[BMI160_ACC_AXIS_Y]
				*(obj->offset[BMI160_ACC_AXIS_Y]*mul + obj->cali_sw[BMI160_ACC_AXIS_Y]);
	dat[obj->cvt.map[BMI160_ACC_AXIS_Z]] = obj->cvt.sign[BMI160_ACC_AXIS_Z]
				*(obj->offset[BMI160_ACC_AXIS_Z]*mul + obj->cali_sw[BMI160_ACC_AXIS_Z]);

	return err;
}

static int bmi160_acc_read_calibrationEx(struct i2c_client *client,
		int act[BMI160_ACC_AXES_NUM], int raw[BMI160_ACC_AXES_NUM])
{
	/* raw: the raw calibration data; act: the actual calibration data */
	int mul;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
#ifdef SW_CALIBRATION
	/* only SW Calibration, disable  Calibration */
	mul = 0;
#else
	int err;
	err = bmi160_acc_read_offset(client, obj->offset);
	if(err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity/bmi160_acc_offset_resolution.sensitivity;
#endif

	raw[BMI160_ACC_AXIS_X] = obj->offset[BMI160_ACC_AXIS_X]*mul + obj->cali_sw[BMI160_ACC_AXIS_X];
	raw[BMI160_ACC_AXIS_Y] = obj->offset[BMI160_ACC_AXIS_Y]*mul + obj->cali_sw[BMI160_ACC_AXIS_Y];
	raw[BMI160_ACC_AXIS_Z] = obj->offset[BMI160_ACC_AXIS_Z]*mul + obj->cali_sw[BMI160_ACC_AXIS_Z];

	act[obj->cvt.map[BMI160_ACC_AXIS_X]] = obj->cvt.sign[BMI160_ACC_AXIS_X]*raw[BMI160_ACC_AXIS_X];
	act[obj->cvt.map[BMI160_ACC_AXIS_Y]] = obj->cvt.sign[BMI160_ACC_AXIS_Y]*raw[BMI160_ACC_AXIS_Y];
	act[obj->cvt.map[BMI160_ACC_AXIS_Z]] = obj->cvt.sign[BMI160_ACC_AXIS_Z]*raw[BMI160_ACC_AXIS_Z];

	return 0;
}

static int bmi160_acc_write_calibration(struct i2c_client *client, int dat[BMI160_ACC_AXES_NUM])
{
	int err = 0;
	int cali[BMI160_ACC_AXES_NUM] = {0};
	int raw[BMI160_ACC_AXES_NUM] = {0};
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
#ifndef SW_CALIBRATION
	int lsb = bmi160_acc_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;
#endif
	err = bmi160_acc_read_calibrationEx(client, cali, raw);
	/* offset will be updated in obj->offset */
	if(err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
			raw[BMI160_ACC_AXIS_X], raw[BMI160_ACC_AXIS_Y], raw[BMI160_ACC_AXIS_Z],
			obj->offset[BMI160_ACC_AXIS_X], obj->offset[BMI160_ACC_AXIS_Y], obj->offset[BMI160_ACC_AXIS_Z],
			obj->cali_sw[BMI160_ACC_AXIS_X], obj->cali_sw[BMI160_ACC_AXIS_Y], obj->cali_sw[BMI160_ACC_AXIS_Z]);
	/* calculate the real offset expected by caller */
	cali[BMI160_ACC_AXIS_X] += dat[BMI160_ACC_AXIS_X];
	cali[BMI160_ACC_AXIS_Y] += dat[BMI160_ACC_AXIS_Y];
	cali[BMI160_ACC_AXIS_Z] += dat[BMI160_ACC_AXIS_Z];
	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n",
			dat[BMI160_ACC_AXIS_X], dat[BMI160_ACC_AXIS_Y], dat[BMI160_ACC_AXIS_Z]);
#ifdef SW_CALIBRATION
	obj->cali_sw[BMI160_ACC_AXIS_X] = obj->cvt.sign[BMI160_ACC_AXIS_X]*(cali[obj->cvt.map[BMI160_ACC_AXIS_X]]);
	obj->cali_sw[BMI160_ACC_AXIS_Y] = obj->cvt.sign[BMI160_ACC_AXIS_Y]*(cali[obj->cvt.map[BMI160_ACC_AXIS_Y]]);
	obj->cali_sw[BMI160_ACC_AXIS_Z] = obj->cvt.sign[BMI160_ACC_AXIS_Z]*(cali[obj->cvt.map[BMI160_ACC_AXIS_Z]]);
#else
	obj->offset[BMI160_ACC_AXIS_X] = (s8)(obj->cvt.sign[BMI160_ACC_AXIS_X]*(cali[obj->cvt.map[BMI160_ACC_AXIS_X]])/(divisor));
	obj->offset[BMI160_ACC_AXIS_Y] = (s8)(obj->cvt.sign[BMI160_ACC_AXIS_Y]*(cali[obj->cvt.map[BMI160_ACC_AXIS_Y]])/(divisor));
	obj->offset[BMI160_ACC_AXIS_Z] = (s8)(obj->cvt.sign[BMI160_ACC_AXIS_Z]*(cali[obj->cvt.map[BMI160_ACC_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[BMI160_ACC_AXIS_X] = obj->cvt.sign[BMI160_ACC_AXIS_X]*(cali[obj->cvt.map[BMI160_ACC_AXIS_X]])%(divisor);
	obj->cali_sw[BMI160_ACC_AXIS_Y] = obj->cvt.sign[BMI160_ACC_AXIS_Y]*(cali[obj->cvt.map[BMI160_ACC_AXIS_Y]])%(divisor);
	obj->cali_sw[BMI160_ACC_AXIS_Z] = obj->cvt.sign[BMI160_ACC_AXIS_Z]*(cali[obj->cvt.map[BMI160_ACC_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
			obj->offset[BMI160_ACC_AXIS_X]*divisor + obj->cali_sw[BMI160_ACC_AXIS_X],
			obj->offset[BMI160_ACC_AXIS_Y]*divisor + obj->cali_sw[BMI160_ACC_AXIS_Y],
			obj->offset[BMI160_ACC_AXIS_Z]*divisor + obj->cali_sw[BMI160_ACC_AXIS_Z],
			obj->offset[BMI160_ACC_AXIS_X], obj->offset[BMI160_ACC_AXIS_Y], obj->offset[BMI160_ACC_AXIS_Z],
			obj->cali_sw[BMI160_ACC_AXIS_X], obj->cali_sw[BMI160_ACC_AXIS_Y], obj->cali_sw[BMI160_ACC_AXIS_Z]);

	err = bma_i2c_write_block(obj->client, BMI160_ACC_REG_OFSX, obj->offset, BMI160_ACC_AXES_NUM);
	if(err) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}
/*!
 * @brief check chip id
 *
 * @param[in] client the pointer of i2c_client
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_check_chip_id(struct i2c_client *client)
{
	int err = 0;
	u8 databuf[2] = { 0 };
	err = bma_i2c_read_block(client, BMI160_USER_CHIP_ID__REG, databuf, 1);
	err = bma_i2c_read_block(client, BMI160_USER_CHIP_ID__REG, databuf, 1);
	if (err < 0) {
		GSE_ERR("read chip id failed.\n");
		return BMI160_ACC_ERR_I2C;
	}
	switch (databuf[0]) {
	case SENSOR_CHIP_ID_BMI:
	case SENSOR_CHIP_ID_BMI_C2:
	case SENSOR_CHIP_ID_BMI_C3:
		GSE_LOG("check chip id %d successfully.\n", databuf[0]);
		break;
	default:
		GSE_LOG("check chip id %d failed.\n", databuf[0]);
		break;
	}
	return err;
}

/*!
 * @brief Set power mode for acc
 *
 * @param[in] client the pointer of i2c_client
 * @param[in] enable
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_set_power_mode(struct i2c_client *client, bool enable)
{
	int err = 0;
	u8 databuf[2] = { 0 };
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	u8 errorcode = 0;
	int retry = 0;

	mutex_lock(&obj->lock);
	if (enable == true) {
		databuf[0] = CMD_PMU_ACC_NORMAL;
	} else {
		databuf[0] = CMD_PMU_ACC_SUSPEND;
	}
	err = bma_i2c_write_block(client, BMI160_CMD_COMMANDS__REG, &databuf[0], 1);
	err = bma_i2c_read_block(client, 0x02, &errorcode, 1);

	while( !((errorcode ^ 0x00) == 0) ) {
		GSE_ERR("code : %x Retry for guarantee execution time.\n", errorcode);
		mdelay(10);
		err = bma_i2c_write_block(client, BMI160_CMD_COMMANDS__REG, &databuf[0], 1);
		err = bma_i2c_read_block(client, 0x02, &errorcode, 1);
		retry++;
		if(retry == 2)
		{
			GSE_ERR("retry 2 times but write power mode value to register failed.\n");
			mutex_unlock(&obj->lock);
			return BMI160_ACC_ERR_I2C;
		}
	}
	if (err < 0) {
		GSE_ERR("write power mode value to register failed.\n");
		mutex_unlock(&obj->lock);
		return BMI160_ACC_ERR_I2C;
	}

	sensor_power = enable;
	mdelay(1);

	mutex_unlock(&obj->lock);

	GSE_LOG("set power mode enable = %d ok!\n", enable);

	return 0;
}

/*!
 * @brief Set range value for acc
 *
 * @param[in] client the pointer of i2c_client
 * @param[in] range value
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_set_data_range(struct i2c_client *client, u8 dataformat)
{
	int res = 0;
	u8 databuf[2] = { 0 };
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	mutex_lock(&obj->lock);
	res = bma_i2c_read_block(client, BMI160_USER_ACC_RANGE__REG, &databuf[0], 1);
	databuf[0] = BMI160_SET_BITSLICE(databuf[0], BMI160_USER_ACC_RANGE, dataformat);
	res += bma_i2c_write_block(client, BMI160_USER_ACC_RANGE__REG, &databuf[0], 1);
	mdelay(1);
	if (res < 0) {
		GSE_ERR("set range failed.\n");
		mutex_unlock(&obj->lock);
		return BMI160_ACC_ERR_I2C;
	}
	mutex_unlock(&obj->lock);
	GSE_LOG("bmi160_acc_set_data_range(0x%x)\n", dataformat);
	return bmi160_acc_set_resolution(obj);
}

/*!
 * @brief Set bandwidth for acc
 *
 * @param[in] client the pointer of i2c_client
 * @param[in] bandwidth value
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_set_odr(struct i2c_client *client, u8 bwrate)
{
	int err = 0;
	u8 databuf[2] = { 0 };
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;

	if (bwrate == 0)
		return 0;
	mutex_lock(&obj->lock);
	err = bma_i2c_read_block(client, BMI160_USER_ACC_CONF_ODR__REG, &databuf[0], 1);
	databuf[0] = BMI160_SET_BITSLICE(databuf[0], BMI160_USER_ACC_CONF_ODR, bwrate);
	err += bma_i2c_write_block(client, BMI160_USER_ACC_CONF_ODR__REG, &databuf[0], 1);
	mdelay(20);
	if (err < 0) {
		GSE_ERR("set bandwidth failed, res = %d\n", err);
		mutex_unlock(&obj->lock);
		return BMI160_ACC_ERR_I2C;
	}
	GSE_LOG("set bandwidth = %d ok.\n", bwrate);
	mutex_unlock(&obj->lock);
	return err;
}

/*!
 * @brief Set OSR for acc
 *
 * @param[in] client the pointer of i2c_client
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_set_osr4(struct i2c_client *client)
{
	int err = 0;
	uint8_t databuf[2] = {0};
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	uint8_t bandwidth = BMI160_ACCEL_OSR4_AVG1;
	uint8_t accel_undersampling_parameter = 0;
	mutex_lock(&obj->lock);
	err = bma_i2c_read_block(client,
			BMI160_USER_ACC_CONF_ODR__REG, &databuf[0], 1);
	databuf[0] = BMI160_SET_BITSLICE(databuf[0],
			BMI160_USER_ACC_CONF_ACC_BWP, bandwidth);
	databuf[0] = BMI160_SET_BITSLICE(databuf[0],
			BMI160_USER_ACC_CONF_ACC_UNDER_SAMPLING,
			accel_undersampling_parameter);
	err += bma_i2c_write_block(client,
			BMI160_USER_ACC_CONF_ODR__REG, &databuf[0], 1);
	mdelay(10);
	if(err < 0) {
		GSE_ERR("set OSR failed.\n");
		mutex_unlock(&obj->lock);
		return BMI160_ACC_ERR_I2C;
	}
	GSE_LOG("[%s] acc_bmp = %d, acc_us = %d ok.\n", __func__,
				bandwidth, accel_undersampling_parameter);
	mutex_unlock(&obj->lock);
	return err;
}

/*!
 * @brief Set interrupt enable
 *
 * @param[in] client the pointer of i2c_client
 * @param[in] enable for interrupt
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_set_int_enable(struct i2c_client *client, u8 enable)
{
	int err = 0;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	mutex_lock(&obj->lock);
	err = bma_i2c_write_block(client, BMI160_USER_INT_EN_0_ADDR, &enable, 0x01);
	mdelay(1);
	if(err < 0) {
		mutex_unlock(&obj->lock);
		return err;
	}
	err = bma_i2c_write_block(client, BMI160_USER_INT_EN_1_ADDR, &enable, 0x01);
	mdelay(1);
	if(err < 0) {
		mutex_unlock(&obj->lock);
		return err;
	}
	err = bma_i2c_write_block(client, BMI160_USER_INT_EN_2_ADDR, &enable, 0x01);
	mdelay(1);
	if(err < 0) {
		mutex_unlock(&obj->lock);
		return err;
	}
	mutex_unlock(&obj->lock);
	GSE_LOG("bmi160 set interrupt enable = %d ok.\n", enable);
	return 0;
}

/*!
 * @brief bmi160 initialization
 *
 * @param[in] client the pointer of i2c_client
 * @param[in] int reset calibration value
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_init_client(struct i2c_client *client, int reset_cali)
{
	int err = 0;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;

	err = bmi160_acc_check_chip_id(client);
	if (err < 0) {
		return err;
	}

	/* soft reset */
	err = bmi160_set_command_register(0xB6);
	if (err < 0) {
		return err;
	}
	mdelay(5);

	err = bmi160_acc_set_odr(client, BMI160_ACCEL_ODR_200HZ);
	if(err < 0) {
		return err;
	}
	err = bmi160_acc_set_osr4(client);
	if(err < 0) {
		return err;
	}
	err = bmi160_acc_set_data_range(client, BMI160_ACCEL_RANGE_4G);
	if(err < 0) {
		return err;
	}
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;
	err = bmi160_acc_set_int_enable(client, 0);
	if(err < 0) {
		return err;
	}
	err = bmi160_acc_set_power_mode(client, false);
	if(err < 0) {
		return err;
	}
	if(0 != reset_cali) {
		/* reset calibration only in power on */
		err = bmi160_acc_reset_calib(client);
		if(err < 0) {
			return err;
		}
	}
	GSE_LOG("bmi160 acc init OK.\n");
	return 0;
}

static int bmi160_acc_read_chipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	sprintf(buf, "bmi160_acc");
	return 0;
}

/*!
 * @brief get the compensated data for gsensor
 *
 * @param[in] i2c_client client
 * @param[out] char save value
 * @param[in] int bufsize
 *
 * @return zero success, non-zero failed
 */
static int bmi160_acc_get_cps_data(struct i2c_client *client, char *buf, int bufsize)
{
	int res = 0;
	int acc[BMI160_ACC_AXES_NUM] = { 0 };
	s16 databuf[BMI160_ACC_AXES_NUM] = { 0 };
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	if (sensor_power == false) {
		res = bmi160_acc_set_power_mode(client, true);
		if (res) {
			GSE_ERR("Power on bmi160_acc error %d!\n", res);
		}
	}
	res = bmi160_acc_read_data(client, databuf);
	if (res) {
		GSE_ERR("read acc data failed.\n");
		return -3;
	} else {
		/* Add compensated value performed by MTK calibration process */
		databuf[BMI160_ACC_AXIS_X] += obj->cali_sw[BMI160_ACC_AXIS_X];
		databuf[BMI160_ACC_AXIS_Y] += obj->cali_sw[BMI160_ACC_AXIS_Y];
		databuf[BMI160_ACC_AXIS_Z] += obj->cali_sw[BMI160_ACC_AXIS_Z];
		/*remap coordinate */
		acc[obj->cvt.map[BMI160_ACC_AXIS_X]] =
		    obj->cvt.sign[BMI160_ACC_AXIS_X] * databuf[BMI160_ACC_AXIS_X];
		acc[obj->cvt.map[BMI160_ACC_AXIS_Y]] =
		    obj->cvt.sign[BMI160_ACC_AXIS_Y] * databuf[BMI160_ACC_AXIS_Y];
		acc[obj->cvt.map[BMI160_ACC_AXIS_Z]] =
		    obj->cvt.sign[BMI160_ACC_AXIS_Z] * databuf[BMI160_ACC_AXIS_Z];
		sprintf(buf, "%d %d %d",
				(s16)acc[BMI160_ACC_AXIS_X],
				(s16)acc[BMI160_ACC_AXIS_Y],
				(s16)acc[BMI160_ACC_AXIS_Z]);
	}
	return 0;
}

static int bmi160_acc_read_sensor_data(struct i2c_client *client, char *buf, int bufsize)
{
	int err = 0;
	int acc[BMI160_ACC_AXES_NUM] = { 0 };
	s16 databuf[BMI160_ACC_AXES_NUM] = { 0 };
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
#if 0
	if (sensor_power == false) {
		err = bmi160_acc_set_power_mode(client, true);
		if (err) {
			GSE_ERR("set power on acc failed.\n");
			return err;
		}
	}
#endif
	err = bmi160_acc_read_data(client, databuf);
	if (err) {
		GSE_ERR("read acc data failed.\n");
		return err;
	} else {
		databuf[BMI160_ACC_AXIS_X] += obj->cali_sw[BMI160_ACC_AXIS_X];
		databuf[BMI160_ACC_AXIS_Y] += obj->cali_sw[BMI160_ACC_AXIS_Y];
		databuf[BMI160_ACC_AXIS_Z] += obj->cali_sw[BMI160_ACC_AXIS_Z];
		/* remap coordinate */
		acc[obj->cvt.map[BMI160_ACC_AXIS_X]] =
		    obj->cvt.sign[BMI160_ACC_AXIS_X] * databuf[BMI160_ACC_AXIS_X];
		acc[obj->cvt.map[BMI160_ACC_AXIS_Y]] =
		    obj->cvt.sign[BMI160_ACC_AXIS_Y] * databuf[BMI160_ACC_AXIS_Y];
		acc[obj->cvt.map[BMI160_ACC_AXIS_Z]] =
		    obj->cvt.sign[BMI160_ACC_AXIS_Z] * databuf[BMI160_ACC_AXIS_Z];
		/* Output the mg */
		acc[BMI160_ACC_AXIS_X] = acc[BMI160_ACC_AXIS_X]
		    * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMI160_ACC_AXIS_Y] = acc[BMI160_ACC_AXIS_Y]
		    * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMI160_ACC_AXIS_Z] = acc[BMI160_ACC_AXIS_Z]
		    * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		//GSE_LOG("acc final xyz data: %d,%d,%d, sens:%d\n", acc[0], acc[1], acc[2], obj->reso->sensitivity);
		sprintf(buf, "%04x %04x %04x",
				acc[BMI160_ACC_AXIS_X],
				acc[BMI160_ACC_AXIS_Y],
				acc[BMI160_ACC_AXIS_Z]);
	}
	return 0;
}

static int bmi160_acc_read_raw_data(struct i2c_client *client, char *buf)
{
	int err = 0;
	s16 databuf[BMI160_ACC_AXES_NUM] = {0};
	err = bmi160_acc_read_data(client, databuf);
	if(err) {
		GSE_ERR("read acc raw data failed.\n");
		return EIO;
	}else {
		sprintf(buf, "bmi160_acc_read_raw_data %04x %04x %04x",
				databuf[BMI160_ACC_AXIS_X],
				databuf[BMI160_ACC_AXIS_Y],
				databuf[BMI160_ACC_AXIS_Z]);
	}
	return 0;
}

static int bmi160_do_calibration(struct i2c_client *client)
{
	int err = 0;
	s16 data[BMI160_ACC_AXES_NUM] = {0,};
	s32 sum[BMI160_ACC_AXES_NUM] = {0,};
	s16 cali[6] = {0,};
	int i = 0;
	unsigned int timeout_shaking = 0;
	unsigned int tilt_check_count = 0;
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	struct bmi160_axis_data_t bmi160_udata_pre;
	struct bmi160_axis_data_t bmi160_udata;

	GSE_FUN();

	/*Back up previous calibration data*/
	for(i = 0 ; i < 6 ; i++){
		cali[i] = client_data->cali_sw[i];
	}

	err = bmi160_acc_reset_calib(client);
	if(err < 0) {
		GSE_ERR("Fail reset current caibration data\n");
		return err;
	}
	GSE_LOG("Success reset current calibration data\n");

	err = bmi160_acc_read_data(client, data);
	if(err < 0) {
		GSE_ERR("Fail read read data\n");
		return err;
	}

	bmi160_udata_pre.x = data[BMI160_ACC_AXIS_X];
	bmi160_udata_pre.y = data[BMI160_ACC_AXIS_Y];
	bmi160_udata_pre.z = data[BMI160_ACC_AXIS_Z];

	if(client_data->cvt.sign[BMI160_ACC_AXIS_Z] * bmi160_udata_pre.z < 0) {
		GSE_ERR("Fail device is upside down. data : %d!\n", bmi160_udata_pre.z);
		return BMI160_ACC_CAL_FAIL;
	}

	do{
		mdelay(20);
		err = bmi160_acc_read_data(client, data);
		if(err < 0)
			return BMI160_ACC_CAL_FAIL;

		bmi160_udata.x = data[BMI160_ACC_AXIS_X];
		bmi160_udata.y = data[BMI160_ACC_AXIS_Y];
		bmi160_udata.z = data[BMI160_ACC_AXIS_Z];

		GSE_LOG("===========moved x, y, z============== timeout = %d\n", timeout_shaking);
		GSE_LOG("(%d, %d, %d) (%d, %d, %d)\n",
				bmi160_udata_pre.x,bmi160_udata_pre.y,bmi160_udata_pre.z,
				bmi160_udata.x,bmi160_udata.y, bmi160_udata.z);

		if((abs(bmi160_udata.x - bmi160_udata_pre.x) > BMI160_SHAKING_DETECT_THRESHOLD)
				||(abs(bmi160_udata.y - bmi160_udata_pre.y) > BMI160_SHAKING_DETECT_THRESHOLD)
				||(abs(bmi160_udata.z - bmi160_udata_pre.z) > BMI160_SHAKING_DETECT_THRESHOLD)){
			GSE_ERR("==============shaking x, y, z=============\n");
			return BMI160_ACC_CAL_FAIL;
		} else {
			bmi160_udata_pre.x = bmi160_udata.x;
			bmi160_udata_pre.y = bmi160_udata.y;
			bmi160_udata_pre.z = bmi160_udata.z;

			sum[BMI160_ACC_AXIS_X] += bmi160_udata.x;
			sum[BMI160_ACC_AXIS_Y] += bmi160_udata.y;
			sum[BMI160_ACC_AXIS_Z] += bmi160_udata.z;

			if(abs(bmi160_udata.y)>BMI160_FAST_CALIB_TILT_LIMIT
					||abs(bmi160_udata.x)>BMI160_FAST_CALIB_TILT_LIMIT)
				tilt_check_count++;
			if(tilt_check_count > 5){
				GSE_ERR("========= tilted over 15 degree ==========\n");
				return BMI160_ACC_FLAT_FAIL;
			}
		}

		timeout_shaking++;

	} while(timeout_shaking < CALIBRATION_DATA_AMOUNT);
	    GSE_LOG("===========complete shaking x, y, z check==============\n");

	/* zero-g offset check */
	if(abs(sum[BMI160_ACC_AXIS_X]/CALIBRATION_DATA_AMOUNT) > BMI160_DEFECT_LIMIT_XY ||
			abs(sum[BMI160_ACC_AXIS_Y]/CALIBRATION_DATA_AMOUNT) > BMI160_DEFECT_LIMIT_XY ||
			((abs(sum[BMI160_ACC_AXIS_Z]/CALIBRATION_DATA_AMOUNT) < BMI160_DEFECT_LIMIT_Z1) ||
				(abs(sum[BMI160_ACC_AXIS_Z]/CALIBRATION_DATA_AMOUNT) > BMI160_DEFECT_LIMIT_Z2))){
		GSE_ERR("========= Calibration zero-g offset check failed (%d %d %d) ==========\n",
			sum[BMI160_ACC_AXIS_X]/CALIBRATION_DATA_AMOUNT,
			sum[BMI160_ACC_AXIS_Y]/CALIBRATION_DATA_AMOUNT,
			sum[BMI160_ACC_AXIS_Z]/CALIBRATION_DATA_AMOUNT);

		return BMI160_ACC_FLAT_FAIL;
	}

	/*Set new calibration*/
	client_data->cali_sw[BMI160_ACC_AXIS_X] = -(s16)(sum[BMI160_ACC_AXIS_X]/CALIBRATION_DATA_AMOUNT);
	client_data->cali_sw[BMI160_ACC_AXIS_Y] = -(s16)(sum[BMI160_ACC_AXIS_Y]/CALIBRATION_DATA_AMOUNT);
	if(sum[BMI160_ACC_AXIS_Z]/CALIBRATION_DATA_AMOUNT > 0)
		client_data->cali_sw[BMI160_ACC_AXIS_Z] = -(s16)(sum[BMI160_ACC_AXIS_Z]/CALIBRATION_DATA_AMOUNT) + 8192;
	else
		client_data->cali_sw[BMI160_ACC_AXIS_Z] = -(s16)(sum[BMI160_ACC_AXIS_Z]/CALIBRATION_DATA_AMOUNT) - 8192;

	client_data->cali_sw[BMI160_ACC_AXIS_OLD_X] = cali[BMI160_ACC_AXIS_X];
	client_data->cali_sw[BMI160_ACC_AXIS_OLD_Y] = cali[BMI160_ACC_AXIS_Y];
	client_data->cali_sw[BMI160_ACC_AXIS_OLD_Z] = cali[BMI160_ACC_AXIS_Z];

	client_data->cal_update = 1;
#ifdef CALIBRATION_TO_FILE
#ifdef CONFIG_CUSTOM_KERNEL_SENSOR_CAL
	err = sensor_calibration_save(ID_ACCELEROMETER, client_data->cali_sw);
#endif
	if(err)
			return BMI160_ACC_CAL_FAIL;
#endif

	return BMI160_ACC_SUCCESS;
}

int bmi160_acc_get_mode(struct i2c_client *client, unsigned char *mode)
{
	int comres = 0;
	u8 v_data_u8r = C_BMI160_ZERO_U8X;
	comres = bma_i2c_read_block(client,
			BMI160_USER_ACC_PMU_STATUS__REG, &v_data_u8r, 1);
	*mode = BMI160_GET_BITSLICE(v_data_u8r,
			BMI160_USER_ACC_PMU_STATUS);
	return comres;
}

static int bmi160_acc_set_range(struct i2c_client *client, unsigned char range)
{
	return bmi160_acc_set_data_range(client, (u8) range);
/*
	int comres = 0;
	unsigned char data[2] = {BMI160_USER_ACC_RANGE__REG};
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;

	if (client == NULL)
	{
		return -1;
	}
	mutex_lock(&obj->lock);
	comres = bma_i2c_read_block(client,
			BMI160_USER_ACC_RANGE__REG, data+1, 1);

	data[1]  = BMI160_SET_BITSLICE(data[1],
			BMI160_USER_ACC_RANGE, range);

	comres = i2c_master_send(client, data, 2);
	mutex_unlock(&obj->lock);
	if(comres <= 0)
	{
		return BMI160_ACC_ERR_I2C;
	}
	else
	{
		return comres;
	}
*/
}

static int bmi160_acc_get_range(struct i2c_client *client, u8 *range)
{
	int comres = 0;
	u8 data;
	comres = bma_i2c_read_block(client, BMI160_USER_ACC_RANGE__REG,	&data, 1);
	*range = BMI160_GET_BITSLICE(data, BMI160_USER_ACC_RANGE);
	return comres;
}

static int bmi160_acc_set_bandwidth(struct i2c_client *client, unsigned char bandwidth)
{
	int comres = 0;
	unsigned char data[2] = {BMI160_USER_ACC_CONF_ODR__REG};
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	GSE_LOG("[%s] bandwidth = %d\n", __func__, bandwidth);
	mutex_lock(&obj->lock);
	comres = bma_i2c_read_block(client,
			BMI160_USER_ACC_CONF_ODR__REG, data+1, 1);
	data[1]  = BMI160_SET_BITSLICE(data[1],
			BMI160_USER_ACC_CONF_ODR, bandwidth);
	comres = i2c_master_send(client, data, 2);
	mdelay(1);
	mutex_unlock(&obj->lock);
	if(comres <= 0) {
		return BMI160_ACC_ERR_I2C;
	}else {
		return comres;
	}
}

static int bmi160_acc_get_bandwidth(struct i2c_client *client, unsigned char *bandwidth)
{
	int comres = 0;
	comres = bma_i2c_read_block(client, BMI160_USER_ACC_CONF_ODR__REG, bandwidth, 1);
	*bandwidth = BMI160_GET_BITSLICE(*bandwidth, BMI160_USER_ACC_CONF_ODR);
	return comres;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[BMI160_BUFSIZE];
	struct i2c_client *client = bmi160_acc_i2c_client;
	bmi160_acc_read_chipInfo(client, strbuf, BMI160_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_acc_op_mode_value(struct device_driver *ddri, char *buf)
{
	int err = 0;
	u8 data = 0;
	err = bmi160_acc_get_mode(bmi160_acc_i2c_client, &data);
	if(err < 0 ) {
		GSE_ERR("get acc op mode failed.\n");
		return err;
	}
	return sprintf(buf, "%d\n", data);
}

static ssize_t store_acc_op_mode_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;

	err = kstrtoul(buf, 10, &data);
	if (err) {
		return err;
	}
	GSE_ERR("store_acc_op_mode_value = %d .\n", (int)data);
	if (data == BMI160_ACC_MODE_NORMAL) {
		err = bmi160_acc_set_power_mode(bmi160_acc_i2c_client, true);
		sensor_power = true;
	}else {
#ifdef STEP_DETECT
		if(!atomic_read(&obj->step_detect_enable))
#endif
		err = bmi160_acc_set_power_mode(bmi160_acc_i2c_client, false);
		sensor_power = false;
	}
	if(err < 0) {
		GSE_ERR("set acc op mode = %d failed.\n", (int)data);
		return err;
	}
	return count;
}

static ssize_t show_acc_range_value(struct device_driver *ddri, char *buf)
{
	int err = 0;
	u8 data;
	err = bmi160_acc_get_range(bmi160_acc_i2c_client, &data);
	if(err < 0 ) {
		GSE_ERR("get acc range failed.\n");
		return err;
	}
	return sprintf(buf, "%d\n", data);
}

static ssize_t store_acc_range_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int err;
	err = kstrtoul(buf, 10, &data);
	if (err) {
		return err;
	}
	err = bmi160_acc_set_range(bmi160_acc_i2c_client, (u8)data);
	if(err < 0 ) {
		GSE_ERR("set acc range = %d failed.\n", (int)data);
		return err;
	}
	return count;
}

static ssize_t show_acc_odr_value(struct device_driver *ddri, char *buf)
{
	int err;
	u8 data;
	err = bmi160_acc_get_bandwidth(bmi160_acc_i2c_client, &data);
	if(err < 0 ) {
		GSE_ERR("get acc odr failed.\n");
		return err;
	}
	return sprintf(buf, "%d\n", data);
}

static ssize_t store_acc_odr_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	err = kstrtoul(buf, 10, &data);
	if (err) {
		return err;
	}
	err = bmi160_acc_set_bandwidth(bmi160_acc_i2c_client, (u8)data);
	if(err < 0 ) {
		GSE_ERR("set acc bandwidth failed.\n");
		return err;
	}
	client_data->odr.acc_odr = data;
	return count;
}

static ssize_t show_cpsdata_value(struct device_driver *ddri, char *buf)
{
	char strbuf[BMI160_BUFSIZE] = {0};
	struct i2c_client *client = bmi160_acc_i2c_client;
	bmi160_acc_get_cps_data(client, strbuf, BMI160_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	char strbuf[BMI160_BUFSIZE] = {0};
	struct i2c_client *client = bmi160_acc_i2c_client;
	bmi160_acc_read_sensor_data(client, strbuf, BMI160_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	int err = 0;
	int len = 0;
	int mul;
	int tmp[BMI160_ACC_AXES_NUM] = {0};
	struct bmi160_acc_i2c_data *obj;
	struct i2c_client *client = bmi160_acc_i2c_client;
	obj = obj_i2c_data;
	err = bmi160_acc_read_offset(client, obj->offset);
	if(err) {
		return -EINVAL;
	}
	err = bmi160_acc_read_calibration(client, tmp);
	if(err) {
		return -EINVAL;
	}else {
		mul = obj->reso->sensitivity/bmi160_acc_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
				obj->offset[BMI160_ACC_AXIS_X], obj->offset[BMI160_ACC_AXIS_Y], obj->offset[BMI160_ACC_AXIS_Z],
				obj->offset[BMI160_ACC_AXIS_X], obj->offset[BMI160_ACC_AXIS_Y], obj->offset[BMI160_ACC_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
				obj->cali_sw[BMI160_ACC_AXIS_X], obj->cali_sw[BMI160_ACC_AXIS_Y], obj->cali_sw[BMI160_ACC_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
				obj->offset[BMI160_ACC_AXIS_X]*mul + obj->cali_sw[BMI160_ACC_AXIS_X],
				obj->offset[BMI160_ACC_AXIS_Y]*mul + obj->cali_sw[BMI160_ACC_AXIS_Y],
				obj->offset[BMI160_ACC_AXIS_Z]*mul + obj->cali_sw[BMI160_ACC_AXIS_Z],
				tmp[BMI160_ACC_AXIS_X], tmp[BMI160_ACC_AXIS_Y], tmp[BMI160_ACC_AXIS_Z]);

		return len;
	}
}

static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int err, x, y, z;
	int dat[BMI160_ACC_AXES_NUM] = {0};
	struct i2c_client *client = bmi160_acc_i2c_client;
	if(!strncmp(buf, "rst", 3)) {
		err = bmi160_acc_reset_calib(client);
		if(err) {
			GSE_ERR("reset offset err = %d\n", err);
		}
	} else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
		dat[BMI160_ACC_AXIS_X] = x;
		dat[BMI160_ACC_AXIS_Y] = y;
		dat[BMI160_ACC_AXIS_Z] = z;
		err = bmi160_acc_write_calibration(client, dat);
		if(err) {
			GSE_ERR("write calibration err = %d\n", err);
		}
	}else {
		GSE_ERR("set calibration value by invalid format.\n");
	}
	return count;
}

static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "not support\n");
}

static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	int err;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	err = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return err;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int trace;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	if(1 == sscanf(buf, "0x%x", &trace)) {
		atomic_set(&obj->trace, trace);
	}else {
		GSE_ERR("invalid content: '%s'\n", buf);
	}
	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	if(obj->hw) {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
				obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);
	}else {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;
}

static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	if(sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return 0;
}

static int bmi160_fifo_length(uint32_t *fifo_length)
{
	int comres=0;
	struct i2c_client *client = bmi160_acc_i2c_client;
	uint8_t a_data_u8r[2] = {0, 0};
	comres += bma_i2c_read_block(client, BMI160_USER_FIFO_BYTE_COUNTER_LSB__REG, a_data_u8r, 2);
	a_data_u8r[1] = BMI160_GET_BITSLICE(a_data_u8r[1], BMI160_USER_FIFO_BYTE_COUNTER_MSB);
	*fifo_length = (uint32_t)(((uint32_t)((uint8_t)(a_data_u8r[1])<<BMI160_SHIFT_8_POSITION)) | a_data_u8r[0]);

	return comres;
}

int bmi160_set_fifo_time_enable(u8 v_fifo_time_enable_u8)
{
	/* variable used for return the status of communication result*/
	int com_rslt = -1;
	u8 v_data_u8 = 0;
	struct i2c_client *client = bmi160_acc_i2c_client;

	if (v_fifo_time_enable_u8 <= 1) {
		/* write the fifo sensor time*/
		com_rslt = bma_i2c_read_block(client,
				BMI160_USER_FIFO_TIME_ENABLE__REG, &v_data_u8, 1);
		if (com_rslt == 0) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
					BMI160_USER_FIFO_TIME_ENABLE,
					v_fifo_time_enable_u8);
			com_rslt += bma_i2c_write_block(client,
					BMI160_USER_FIFO_TIME_ENABLE__REG, &v_data_u8, 1);
		}
	} else {
		com_rslt = -2;
	}
	return com_rslt;
}

int bmi160_set_fifo_header_enable(u8 v_fifo_header_enable_u8)
{
	/* variable used for return the status of communication result*/
	int com_rslt = -1;
	u8 v_data_u8 = 0;
	struct i2c_client *client = bmi160_acc_i2c_client;
	if (v_fifo_header_enable_u8 <= 1) {
		/* read the fifo sensor header enable*/
		com_rslt = bma_i2c_read_block(client,
				BMI160_USER_FIFO_HEADER_EN__REG, &v_data_u8, 1);
		if (com_rslt == 0) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
					BMI160_USER_FIFO_HEADER_EN,
					v_fifo_header_enable_u8);
			com_rslt += bma_i2c_write_block(client,
					BMI160_USER_FIFO_HEADER_EN__REG, &v_data_u8, 1);
		}
	} else {
		com_rslt = -2;
	}
	return com_rslt;
}

int bmi160_get_fifo_time_enable(u8 *v_fifo_time_enable_u8)
{
	/* variable used for return the status of communication result*/
	int com_rslt = -1;
	u8 v_data_u8 = 0;
	struct i2c_client *client = bmi160_acc_i2c_client;
	com_rslt = bma_i2c_read_block(client,
			BMI160_USER_FIFO_TIME_ENABLE__REG, &v_data_u8, 1);
	*v_fifo_time_enable_u8 = BMI160_GET_BITSLICE(v_data_u8,
				BMI160_USER_FIFO_TIME_ENABLE);
	return com_rslt;
}

static int bmi160_get_fifo_header_enable(u8 *v_fifo_header_u8)
{
	/* variable used for return the status of communication result*/
	int com_rslt = -1;
	u8 v_data_u8 = 0;
	struct i2c_client *client = bmi160_acc_i2c_client;
	com_rslt = bma_i2c_read_block(client,
			BMI160_USER_FIFO_HEADER_EN__REG, &v_data_u8, 1);
	*v_fifo_header_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_HEADER_EN);
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_write_reg(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	/* check the p_bmi160 structure as NULL*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
		} else {
			/* write data from register*/
			com_rslt =
			p_bmi160->BMI160_BUS_WRITE_FUNC(p_bmi160->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_init(struct bmi160_t *bmi160)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	u8 v_pmu_data_u8 = BMI160_INIT_VALUE;
	/* assign bmi160 ptr */
	p_bmi160 = bmi160;
	com_rslt =
	p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr, BMI160_USER_CHIP_ID__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
	/* read Chip Id */
	p_bmi160->chip_id = v_data_u8;
	/* To avoid gyro wakeup it is required to write 0x00 to 0x6C*/
	com_rslt += bmi160_write_reg(BMI160_USER_PMU_TRIGGER_ADDR,
	&v_pmu_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
	return com_rslt;
}

static int bmi160_set_command_register(u8 cmd_reg)
{
	int comres = 0;
	struct i2c_client *client = bmi160_acc_i2c_client;
	comres += bma_i2c_write_block(client, BMI160_CMD_COMMANDS__REG, &cmd_reg, 1);
	return comres;
}

static ssize_t bmi160_fifo_bytecount_show(struct device_driver *ddri, char *buf)
{
	int comres=0;
	uint32_t fifo_bytecount = 0;
	uint8_t a_data_u8r[2] = {0,0};
	struct i2c_client *client = bmi160_acc_i2c_client;
	comres += bma_i2c_read_block(client, BMI160_USER_FIFO_BYTE_COUNTER_LSB__REG, a_data_u8r, 2);
	a_data_u8r[1] = BMI160_GET_BITSLICE(a_data_u8r[1], BMI160_USER_FIFO_BYTE_COUNTER_MSB);
	fifo_bytecount = (uint32_t)(((uint32_t)((uint8_t)(a_data_u8r[1])<<BMI160_SHIFT_8_POSITION)) | a_data_u8r[0]);
	comres = sprintf(buf, "%u\n", fifo_bytecount);
	return comres;
}

static ssize_t bmi160_fifo_bytecount_store(struct device_driver *ddri, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	client_data->fifo_bytecount = (u16) data;
	return count;
}

static int bmi160_fifo_data_sel_get(struct bmi160_acc_i2c_data *client_data)
{
	int err = 0;
	struct i2c_client *client = bmi160_acc_i2c_client;
	unsigned char data;
	unsigned char fifo_acc_en, fifo_gyro_en, fifo_mag_en;
	unsigned char fifo_datasel;


	err = bma_i2c_read_block(client, BMI160_USER_FIFO_ACC_EN__REG, &data, 1);
	fifo_acc_en = BMI160_GET_BITSLICE(data, BMI160_USER_FIFO_ACC_EN);

	err += bma_i2c_read_block(client, BMI160_USER_FIFO_GYRO_EN__REG, &data, 1);
	fifo_gyro_en = BMI160_GET_BITSLICE(data, BMI160_USER_FIFO_GYRO_EN);

	err += bma_i2c_read_block(client, BMI160_USER_FIFO_MAG_EN__REG, &data, 1);
	fifo_mag_en = BMI160_GET_BITSLICE(data, BMI160_USER_FIFO_MAG_EN);

	if (err)
		return err;

	fifo_datasel = (fifo_acc_en << BMI_ACC_SENSOR) |
		(fifo_gyro_en << BMI_GYRO_SENSOR) |
		(fifo_mag_en << BMI_MAG_SENSOR);

	client_data->fifo_data_sel = fifo_datasel;

	return err;
}

static ssize_t bmi160_fifo_data_sel_show(struct device_driver *ddri, char *buf)
{
	int err = 0;
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	err = bmi160_fifo_data_sel_get(client_data);
	if (err)
		return -EINVAL;
	return sprintf(buf, "%d\n", client_data->fifo_data_sel);
}

static ssize_t bmi160_fifo_data_sel_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	struct i2c_client *client = bmi160_acc_i2c_client;
	int err;
	unsigned long data;
	unsigned char fifo_datasel;
	unsigned char fifo_acc_en, fifo_gyro_en, fifo_mag_en;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	/* data format: aimed 0b0000 0x(m)x(g)x(a), x:1 enable, 0:disable*/
	if (data > 7)
		return -EINVAL;

	fifo_datasel = (unsigned char)data;
	fifo_acc_en = fifo_datasel & (1 << BMI_ACC_SENSOR) ? 1 : 0;
	fifo_gyro_en = fifo_datasel & (1 << BMI_GYRO_SENSOR) ? 1 : 0;
	fifo_mag_en = fifo_datasel & (1 << BMI_MAG_SENSOR) ? 1 : 0;	
	
	err += bma_i2c_read_block(client, BMI160_USER_FIFO_ACC_EN__REG, &fifo_datasel, 1);
	fifo_datasel = BMI160_SET_BITSLICE(fifo_datasel, BMI160_USER_FIFO_ACC_EN, fifo_acc_en);
	err += bma_i2c_write_block(client, BMI160_USER_FIFO_ACC_EN__REG, &fifo_datasel, 1);
	udelay(500);
	err += bma_i2c_read_block(client, BMI160_USER_FIFO_GYRO_EN__REG, &fifo_datasel, 1);
	fifo_datasel = BMI160_SET_BITSLICE(fifo_datasel, BMI160_USER_FIFO_GYRO_EN, fifo_gyro_en);
	err += bma_i2c_write_block(client, BMI160_USER_FIFO_GYRO_EN__REG, &fifo_datasel, 1);
	udelay(500);
	err += bma_i2c_read_block(client, BMI160_USER_FIFO_MAG_EN__REG, &fifo_datasel, 1);
	fifo_datasel = BMI160_SET_BITSLICE(fifo_datasel, BMI160_USER_FIFO_MAG_EN, fifo_mag_en);
	err += bma_i2c_write_block(client, BMI160_USER_FIFO_MAG_EN__REG, &fifo_datasel, 1);
	udelay(500);
	if (err)
		return -EIO;

	client_data->fifo_data_sel = (u8)data;
	GSE_LOG("FIFO fifo_data_sel %d, A_en:%d, G_en:%d, M_en:%d\n",
			client_data->fifo_data_sel, fifo_acc_en, fifo_gyro_en, fifo_mag_en);
	return count;
}

static ssize_t bmi160_fifo_data_out_frame_show(struct device_driver *ddri, char *buf)
{
	int err = 0;
	uint32_t fifo_bytecount = 0;
	struct i2c_client *client = bmi160_acc_i2c_client;
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	err = bmi160_fifo_length(&fifo_bytecount);
	if(err < 0) {
		GSE_ERR("read fifo length error.\n");
		return -EINVAL;
	}
	if(0 == fifo_bytecount) {
		return 0;
	}
	client_data->fifo_bytecount = fifo_bytecount;
#ifdef FIFO_READ_USE_DMA_MODE_I2C
	err = i2c_dma_read(client, BMI160_USER_FIFO_DATA__REG, buf, client_data->fifo_bytecount);
#else
	err = bma_i2c_read_block(client, BMI160_USER_FIFO_DATA__REG, buf, fifo_bytecount);
#endif
	if (err < 0) {
		GSE_ERR("read fifo data error.\n");
		return sprintf(buf, "Read byte block error.");
	}

	return client_data->fifo_bytecount;
}

static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct bmi160_acc_i2c_data *data = obj_i2c_data;

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
			data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
			data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);            
}

static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bmi160_acc_i2c_data *data = obj_i2c_data;
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			GSE_ERR( "HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			GSE_ERR( "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			GSE_ERR( "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		GSE_ERR( "invalid format = '%s'\n", buf);
	}

	return count;            
}

static void bmi_dump_reg(struct bmi160_acc_i2c_data *client_data)
{
#if 0
	int i;
	u8 dbg_buf0[REG_MAX0];
	u8 dbg_buf1[REG_MAX1];
	u8 dbg_buf_str0[REG_MAX0 * 3 + 1] = "";
	u8 dbg_buf_str1[REG_MAX1 * 3 + 1] = "";
	struct i2c_client *client = bmi160_acc_i2c_client;
	GSE_LOG("\nFrom 0x00:\n");
	bma_i2c_read_block(client, BMI160_USER_CHIP_ID__REG, dbg_buf0, REG_MAX0);
	for (i = 0; i < REG_MAX0; i++) {
		sprintf(dbg_buf_str0 + i * 3, "%02x%c", dbg_buf0[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	GSE_LOG("%s\n", dbg_buf_str0);

	bma_i2c_read_block(client, BMI160_USER_ACCEL_CONFIG_ADDR, dbg_buf1, REG_MAX1);
	GSE_LOG("\nFrom 0x40:\n");
	for (i = 0; i < REG_MAX1; i++) {
		sprintf(dbg_buf_str1 + i * 3, "%02x%c", dbg_buf1[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	GSE_LOG("\n%s\n", dbg_buf_str1);
#endif

	struct i2c_client *client = bmi160_acc_i2c_client;

	u8 addr;
	int i, j;

	GSE_LOG("bmi_register_show ~! \n");

	for (i=0; i<7; i++)
	{
		for (j=0; j<16 ; j++)
		{
			bma_i2c_read_block(client, 0x00 + j + (16 * i), &addr, 1);
			GSE_LOG(" 0x%02x : 0x%02x \n", 0x00 + j + (16 * i), addr);
			addr = 0x00;
		}
		GSE_LOG("\n\n");
	}

	for (i=0; i<15; i++)
	{
		bma_i2c_read_block(client, 0x70 + i, &addr, 1);
		GSE_LOG(" 0x%02x : 0x%02x \n", 0x70 + i, addr);
		addr = 0x00;
	}
	GSE_LOG("\n\n");
}

static ssize_t bmi_register_show(struct device_driver *ddri, char *buf)
{
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	bmi_dump_reg(client_data);
	return sprintf(buf, "Dump OK\n");
}

static ssize_t bmi_register_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	int err;
	int reg_addr = 0;
	int data;
	u8 write_reg_add = 0;
	u8 write_data = 0;
	struct i2c_client *client = bmi160_acc_i2c_client;

	err = sscanf(buf, "%3d %3d", &reg_addr, &data);
	if (err < 2)
		return err;

	if (data > 0xff)
		return -EINVAL;

	write_reg_add = (u8)reg_addr;
	write_data = (u8)data;
	err += bma_i2c_write_block(client, write_reg_add, &write_data, 1);

	if (!err) {
		GSE_ERR("write reg 0x%2x, value= 0x%2x\n", reg_addr, data);
	} else {
		GSE_ERR("write reg fail\n");
		return err;
	}
	return count;
}

static ssize_t bmi160_bmi_value_show(struct device_driver *ddri, char *buf)
{
	int err;
	u8 raw_data[12] = {0};
	struct i2c_client *client = bmi160_acc_i2c_client;
	memset(raw_data, 0, sizeof(raw_data));
	err = bma_i2c_read_block(client, BMI160_USER_DATA_8_GYRO_X_LSB__REG, raw_data, 12);
	if (err)
		return err;
	/* output:gyro x y z acc x y z */
	return sprintf(buf, "%hd %d %hd %hd %hd %hd\n",
			(s16)(raw_data[1] << 8 | raw_data[0]),
			(s16)(raw_data[3] << 8 | raw_data[2]),
			(s16)(raw_data[5] << 8 | raw_data[4]),
			(s16)(raw_data[7] << 8 | raw_data[6]),
			(s16)(raw_data[9] << 8 | raw_data[8]),
			(s16)(raw_data[11] << 8 | raw_data[10]));
}

static int bmi160_get_fifo_wm(u8 *v_fifo_wm_u8)
{
	/* variable used for return the status of communication result*/
	int com_rslt = -1;
	u8 v_data_u8 = 0;
	struct i2c_client *client = bmi160_acc_i2c_client;
	/* check the p_bmi160 structure as NULL*/
	/* read the fifo water mark level*/
	com_rslt = bma_i2c_read_block(client, 
			BMI160_USER_FIFO_WM__REG, &v_data_u8, 1);
	*v_fifo_wm_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_FIFO_WM);
	return com_rslt;
}

static int bmi160_set_fifo_wm(u8 v_fifo_wm_u8)
{
	/* variable used for return the status of communication result*/
	int com_rslt = -1;
	struct i2c_client *client = bmi160_acc_i2c_client;
	/* write the fifo water mark level*/
	com_rslt = bma_i2c_write_block(client,
			BMI160_USER_FIFO_WM__REG, &v_fifo_wm_u8, 1);
	return com_rslt;
}

static ssize_t bmi160_fifo_watermark_show(struct device_driver *ddri, char *buf)
{
	int err;
	u8 data = 0xff;
	err = bmi160_get_fifo_wm(&data);
	if (err)
		return err;
	return sprintf(buf, "%d\n", data);
}

static ssize_t bmi160_fifo_watermark_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	int err;
	unsigned long data;
	u8 fifo_watermark;
	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	fifo_watermark = (u8)data;
	err = bmi160_set_fifo_wm(fifo_watermark);
	if (err)
		return -EIO;

	GSE_LOG("set fifo watermark = %d ok.", (int)fifo_watermark);
	return count;
}

static ssize_t bmi160_delay_show(struct device_driver *ddri, char *buf)
{
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	return sprintf(buf, "%d\n", atomic_read(&client_data->delay));
}

static ssize_t bmi160_delay_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	if (data < BMI_DELAY_MIN)
		data = BMI_DELAY_MIN;

	atomic_set(&client_data->delay, (unsigned int)data);
	return count;
}

static ssize_t bmi160_fifo_flush_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	int err;
	unsigned long enable;
	err = kstrtoul(buf, 10, &enable);
	if (err)
		return err;
	if (enable)
		err = bmi160_set_command_register(CMD_CLR_FIFO_DATA);
	if (err)
		GSE_ERR("fifo flush failed!\n");
	return count;
}

static ssize_t bmi160_fifo_header_en_show(struct device_driver *ddri, char *buf)
{
	int err;
	u8 data = 0xff;
	err = bmi160_get_fifo_header_enable(&data);
	if (err)
		return err;
	return sprintf(buf, "%d\n", data);
}

static ssize_t bmi160_fifo_header_en_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	int err;
	unsigned long data;
	u8 fifo_header_en;
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	if (data > 1)
		return -ENOENT;
	fifo_header_en = (u8)data;
	err = bmi160_set_fifo_header_enable(fifo_header_en);
	if (err)
		return -EIO;
	client_data->fifo_head_en = fifo_header_en;
	return count;
}

static ssize_t show_run_fast_calibration_value(struct device_driver *ddri, char *buf)
{
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	return sprintf(buf, "%d\n", atomic_read(&client_data->fast_calib_rslt));
}

static ssize_t store_run_fast_calibration_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	int res = 0;
	struct i2c_client *client = bmi160_acc_i2c_client;
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	int enabled = sensor_power; //atomic_read(&client_data->accel_enable);

	GSE_FUN();

	atomic_set(&client_data->fast_calib_rslt, BMI160_ACC_CAL_FAIL);

	if (!enabled)
	{
		GSE_LOG("%s sensor is disabled. so enable... \n", __func__);
		res = bmi160_acc_set_power_mode(bmi160_acc_i2c_client, true);
		if (res)
		{
			GSE_ERR("%s set power mode to true, failed err=%d\n", __func__, res);
			return res;
		}
		mdelay(100);
	}

	res = bmi160_do_calibration(client);
	if( res != BMI160_ACC_SUCCESS) {
		GSE_ERR("Calibration Fail err=%d\n", res);
		if( res == BMI160_ACC_FLAT_FAIL )
			atomic_set(&client_data->fast_calib_rslt, BMI160_ACC_FLAT_FAIL);
	} else {
		atomic_set(&client_data->fast_calib_rslt, BMI160_ACC_SUCCESS);
		GSE_LOG("Calibraiton Success\n");
	}

	if (!enabled)
	{
		GSE_LOG("%s sensor is disabled. so disable... \n", __func__);
		res = bmi160_acc_set_power_mode(bmi160_acc_i2c_client, false);
		if (res)
		{
			GSE_ERR("%s set power mode to false, failed err=%d\n", __func__, res);
			return res;
		}
	}

	return count;
}

#ifdef BMI160_SELFTEST //LGE_CHANGE
static int bmi160_set_acc_selftest_amp(u8 acc_selftest_amp)
{
	struct i2c_client *client = bmi160_acc_i2c_client;
	int comres = SUCCESS;
	u8 v_data_u8r = C_BMI160_ZERO_U8X;

	if(acc_selftest_amp < C_BMI160_TWO_U8X){
		comres = bma_i2c_read_block(client,
			BMI160_USER_ACC_SELF_TEST_AMP__REG, &v_data_u8r, 1);
		if(comres == SUCCESS){
			v_data_u8r = BMI160_SET_BITSLICE(v_data_u8r,
					BMI160_USER_ACC_SELF_TEST_AMP, acc_selftest_amp);
			comres += bma_i2c_write_block(client,
				BMI160_USER_ACC_SELF_TEST_AMP__REG, &v_data_u8r, 1);
		}
	}else{
		comres = E_BMI160_OUT_OF_RANGE;
	}
	return comres;
}

static int bmi160_set_acc_selftest_sign(u8 acc_selftest_sign)
{
	struct i2c_client *client = bmi160_acc_i2c_client;
	int comres = SUCCESS;
	u8 v_data_u8r = C_BMI160_ZERO_U8X;

	if(acc_selftest_sign < C_BMI160_TWO_U8X){
		comres = bma_i2c_read_block(client,
			BMI160_USER_ACC_SELF_TEST_SIGN__REG, &v_data_u8r, 1);
		if(comres == SUCCESS){
			v_data_u8r = BMI160_SET_BITSLICE(v_data_u8r,
					BMI160_USER_ACC_SELF_TEST_SIGN, acc_selftest_sign);
			comres += bma_i2c_write_block(client,
				BMI160_USER_ACC_SELF_TEST_SIGN__REG, &v_data_u8r, 1);
		}
	}else{
		comres = E_BMI160_OUT_OF_RANGE;
	}
	return comres;
}

static int bmi160_get_acc_selftest_sign(u8 *acc_selftest_amp)
{
	struct i2c_client *client = bmi160_acc_i2c_client;
	int comres = SUCCESS;
	u8 v_data_u8r = C_BMI160_ZERO_U8X;

	comres = bma_i2c_read_block(client,
		BMI160_USER_ACC_SELF_TEST_AMP__REG, &v_data_u8r, 1);
	if(comres == SUCCESS){
		*acc_selftest_amp = BMI160_GET_BITSLICE(v_data_u8r,BMI160_USER_ACC_SELF_TEST_AMP);
	}
	return comres;
}

static int bmi160_set_acc_selftest_axis(u8 acc_selftest_axis)
{
	struct i2c_client *client = bmi160_acc_i2c_client;
	int comres = SUCCESS;
	u8 v_data_u8r = C_BMI160_ZERO_U8X;
	
	if(acc_selftest_axis <= C_BMI160_THREE_U8X){
		comres += bma_i2c_read_block(client,
			BMI160_USER_ACC_SELF_TEST_AXIS__REG, &v_data_u8r, 1);
		if(comres == SUCCESS){
			v_data_u8r = BMI160_SET_BITSLICE(v_data_u8r,
					BMI160_USER_ACC_SELF_TEST_AXIS, acc_selftest_axis);
			comres += bma_i2c_write_block(client,
					BMI160_USER_ACC_SELF_TEST_AXIS__REG, &v_data_u8r, 1);
		}
	}else{
		comres = E_BMI160_OUT_OF_RANGE;
	}
	return comres;
}

static ssize_t bmi160_selftest_show(struct device_driver *ddri, char *buf)
{
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	return sprintf(buf, "0x%x\n", atomic_read(&client_data->selftest_result));
}

static ssize_t bmi160_selftest_store(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bmi160_acc_i2c_client;
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	s16 databuf[BMI160_ACC_AXES_NUM] = {0};
	int err = 0;
	int i = 0;
	u8 acc_selftest = 0;
	s16 axis_p_value, axis_n_value;
	u16 diff_axis[3] = {0xff, 0xff, 0xff};
	u8 bandwidth, range, acc_selftest_sign;

	GSE_ERR("Selftest for BMI16x starting!\n");
	err = bmi160_set_command_register(CMD_RESET_USER_REG);
	msleep(70);
	err += bmi160_set_command_register(CMD_PMU_ACC_NORMAL);
	/*acc_odr = 1600hz, acc_bwp=2, acc_us=0*/
	err += bma_i2c_read_block(client,
			BMI160_USER_ACC_CONF_ODR__REG, (u8 *)&databuf[0], 1);
	databuf[0] = BMI160_SET_BITSLICE(databuf[0],
			BMI160_USER_ACC_CONF_ACC_BWP, 2);
	databuf[0] = BMI160_SET_BITSLICE(databuf[0],
			BMI160_USER_ACC_CONF_ACC_UNDER_SAMPLING,0);
	err += bma_i2c_write_block(client,
			BMI160_USER_ACC_CONF_ODR__REG, (u8 *)&databuf[0], 1);
	err += bmi160_acc_set_odr(client,BMI160_ACCEL_ODR_1600HZ);

	/*set to 8G range*/
	err += bmi160_acc_set_range(client,BMI160_ACCEL_RANGE_8G);
	/*set to self amp high*/
	err += bmi160_set_acc_selftest_amp(1);

	err += bmi160_acc_get_bandwidth(client,&bandwidth);
	err += bmi160_acc_get_range(client,&range);
	GSE_ERR("acc_odr: %d, acc_range:%d!\n", bandwidth, range);

	for(i= X_AXIS; i<BMI160_ACC_AXES_NUM; i++){
		axis_n_value = 0;
		axis_p_value = 0;
		/* set every selftest axis */
		err += bmi160_set_acc_selftest_axis(i+1);
		msleep(50);
		
		switch(i){
			case X_AXIS:
				/*set negative sign*/
				err += bmi160_set_acc_selftest_sign(0);
				err += bmi160_get_acc_selftest_sign(&acc_selftest_sign);

				msleep(60);
				err += bmi160_acc_read_data(client, databuf);
				axis_n_value = databuf[X_AXIS];
				//err += bmi160_read_acc_x(&axis_n_value);
				
				/*set positive sign*/
				err += bmi160_set_acc_selftest_sign(1);
				err += bmi160_get_acc_selftest_sign(&acc_selftest_sign);
		
				msleep(60);
				err += bmi160_acc_read_data(client, databuf);
				axis_p_value = databuf[X_AXIS];
				//err += bmi160_read_acc_x(&axis_p_value);
				diff_axis[i] = abs(axis_p_value - axis_n_value);
				break;

			case Y_AXIS:
				/*set negative sign*/
				err += bmi160_set_acc_selftest_sign(0);
				err += bmi160_get_acc_selftest_sign(&acc_selftest_sign);

				msleep(60);
				err += bmi160_acc_read_data(client, databuf);
				axis_n_value = databuf[Y_AXIS];
				//err += bmi160_read_acc_y(&axis_n_value);
				
				/*set positive sign*/
				err += bmi160_set_acc_selftest_sign(1);
				err += bmi160_get_acc_selftest_sign(&acc_selftest_sign);
				
				msleep(60);
				err += bmi160_acc_read_data(client, databuf);
				axis_p_value = databuf[Y_AXIS];
				//err += bmi160_read_acc_y(&axis_p_value);
				diff_axis[i] = abs(axis_p_value - axis_n_value);
				break;

			case Z_AXIS:
				/*set negative sign*/
				err += bmi160_set_acc_selftest_sign(0);
				err += bmi160_get_acc_selftest_sign(&acc_selftest_sign);

				msleep(60);
				err += bmi160_acc_read_data(client, databuf);
				axis_n_value = databuf[Z_AXIS];
				//err += bmi160_read_acc_z(&axis_n_value);
				
				/*set positive sign*/
				err += bmi160_set_acc_selftest_sign(1);
				err += bmi160_get_acc_selftest_sign(&acc_selftest_sign);
				
				msleep(60);
				err += bmi160_acc_read_data(client, databuf);
				axis_p_value = databuf[Z_AXIS];
				//err += bmi160_read_acc_z(&axis_p_value);
				diff_axis[i] = abs(axis_p_value - axis_n_value);
				break;

			default:
				err += -EINVAL;
				break;
		}

		if(err) {
			GSE_ERR("Failed selfteset axis:%d, p_val=%d, n_val=%d!\n", i, axis_p_value, axis_n_value);
			return -EINVAL;
		}

		/*400mg for acc z axis*/
		if(Z_AXIS == i){
			if(diff_axis[i] < 1639){
				acc_selftest |= 1 << i;
				GSE_ERR("Over selftest minimum for z, p_val=%d, n_val=%d!\n", axis_p_value, axis_n_value);
			}
		}else{
			/*800mg for x or y axis*/
			if(diff_axis[i] < 3277){
				acc_selftest |= 1 <<i;
				GSE_ERR("Over selftest minimum for %d, p_val=%d, n_val=%d!\n", i, axis_p_value, axis_n_value);
			}
		}

		GSE_ERR("axis:%d, p_val:%d, n_val:%d\n", i, axis_p_value, axis_n_value);
	}
	acc_selftest &= 0x0f;
	atomic_set(&client_data->selftest_result, acc_selftest);

	/*soft reset*/
	err = bmi160_set_command_register(CMD_RESET_USER_REG);
	if(err)
		return err;
	msleep(50);
	err = bmi160_acc_set_data_range(client, BMI160_ACCEL_RANGE_4G);
	if(err)
		return err;
	GSE_ERR("Selfteset for BMI16x finished!\n");
	return count;
}
#endif
static ssize_t show_cal_update(struct device_driver *ddri, char *buf)
{
	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	return sprintf(buf, "%d", client_data->cal_update);
}

static ssize_t store_cal_update(struct device_driver *ddri, const char *buf, size_t count){

	struct bmi160_acc_i2c_data *client_data = obj_i2c_data;
	int check_cal=0;
	
	if(1 == sscanf(buf, "%d", &check_cal)){
		client_data->cal_update = check_cal;
	}
	else
	{
		printk(KERN_ERR "invalid content: '%s', length = %zu\n", buf, count);
	}

	return count;
}

static DRIVER_ATTR(cal_update, S_IWUSR| S_IRUGO, show_cal_update, store_cal_update);
static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(cpsdata, 	 S_IWUSR | S_IRUGO, show_cpsdata_value, NULL);
static DRIVER_ATTR(acc_op_mode,  S_IWUSR | S_IRUGO, show_acc_op_mode_value, store_acc_op_mode_value);
static DRIVER_ATTR(acc_range, 	 S_IWUSR | S_IRUGO, show_acc_range_value, store_acc_range_value);
static DRIVER_ATTR(acc_odr, S_IWUSR | S_IRUGO, show_acc_odr_value, store_acc_odr_value);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(status,     S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value, NULL);

static DRIVER_ATTR(fifo_bytecount, S_IRUGO | S_IWUSR, bmi160_fifo_bytecount_show, bmi160_fifo_bytecount_store);
static DRIVER_ATTR(fifo_data_sel, S_IRUGO | S_IWUSR, bmi160_fifo_data_sel_show, bmi160_fifo_data_sel_store);
static DRIVER_ATTR(fifo_data_frame, S_IRUGO, bmi160_fifo_data_out_frame_show, NULL);
static DRIVER_ATTR(layout, S_IRUGO | S_IWUSR, show_layout_value, store_layout_value );
static DRIVER_ATTR(register, S_IRUGO|S_IWUSR, bmi_register_show, bmi_register_store);
static DRIVER_ATTR(bmi_value, S_IRUGO, bmi160_bmi_value_show, NULL);
static DRIVER_ATTR(fifo_watermark, S_IRUGO|S_IWUSR, bmi160_fifo_watermark_show, bmi160_fifo_watermark_store);
static DRIVER_ATTR(delay, S_IRUGO|S_IWUSR, bmi160_delay_show, bmi160_delay_store);
static DRIVER_ATTR(fifo_flush, S_IWUSR | S_IRUGO, NULL, bmi160_fifo_flush_store);
static DRIVER_ATTR(fifo_header_en, S_IWUSR | S_IRUGO, bmi160_fifo_header_en_show, bmi160_fifo_header_en_store);
static DRIVER_ATTR(run_fast_calibration, S_IRUGO|S_IWUSR|S_IWGRP, show_run_fast_calibration_value,  store_run_fast_calibration_value);
static DRIVER_ATTR(selftest, S_IRUGO|S_IWUSR|S_IWGRP, bmi160_selftest_show,  bmi160_selftest_store);

static struct driver_attribute *bmi160_acc_attr_list[] = {
	&driver_attr_cal_update,
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_cpsdata,		/*g sensor data for compass tilt compensation*/
	&driver_attr_acc_op_mode,	/*g sensor opmode for compass tilt compensation*/
	&driver_attr_acc_range,		/*g sensor range for compass tilt compensation*/
	&driver_attr_acc_odr,		/*g sensor bandwidth for compass tilt compensation*/

	&driver_attr_fifo_bytecount,
	&driver_attr_fifo_data_sel,
	&driver_attr_fifo_data_frame,
	&driver_attr_layout,
	&driver_attr_register,
	&driver_attr_bmi_value,
	&driver_attr_fifo_watermark,
	&driver_attr_delay,
	&driver_attr_fifo_flush,
	&driver_attr_fifo_header_en,
	&driver_attr_run_fast_calibration,
	&driver_attr_selftest,
};


static int bmi160_acc_create_attr(struct device_driver *driver)
{
	int err = 0;
	int idx = 0;
	int num = (int)(sizeof(bmi160_acc_attr_list)/sizeof(bmi160_acc_attr_list[0]));
	for(idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, bmi160_acc_attr_list[idx]);
		if(err) {
			GSE_ERR("create driver file (%s) failed.\n", bmi160_acc_attr_list[idx]->attr.name);
			break;
		}
	}
	return err;
}

static int bmi160_acc_delete_attr(struct device_driver *driver)
{
	int idx = 0;
	int err = 0;
	int num = (int)(sizeof(bmi160_acc_attr_list)/sizeof(bmi160_acc_attr_list[0]));
	for(idx = 0; idx < num; idx++) {
		driver_remove_file(driver, bmi160_acc_attr_list[idx]);
	}
	return err;
}

static int bmi160_acc_open(struct inode *inode, struct file *file)
{
	file->private_data = bmi160_acc_i2c_client;
	if(file->private_data == NULL) {
		GSE_ERR("file->private_data is null pointer.\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int bmi160_acc_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long bmi160_acc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	char strbuf[BMI160_BUFSIZE] = {0};
	void __user *data;
	struct SENSOR_DATA sensor_data;
	int err = 0;
	//uint32_t enable = 0;
#ifdef CALIBRATION_TO_FILE
	int cali[6] = {0};
#else
    int cali[3] = {0};
#endif
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	if(_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}else if(_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}
	if(err) {
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd) {
		case GSENSOR_IOCTL_INIT:
			bmi160_acc_init_client(client, 0);
			break;
		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL) {
				err = -EINVAL;
				break;
			}
			bmi160_acc_read_chipInfo(client, strbuf, BMI160_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				err = -EFAULT;
			}
			break;
		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL) {
				err = -EINVAL;
				break;
			}
			bmi160_acc_read_sensor_data(client, strbuf, BMI160_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				err = -EFAULT;
			}
			break;
		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL) {
				err = -EINVAL;
				break;
			}
			if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D))) {
				err = -EFAULT;
			}
			break;
		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL) {
				err = -EINVAL;
				break;
			}
			bmi160_acc_read_raw_data(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1)) {
				err = -EFAULT;
			}
			break;
		case GSENSOR_IOCTL_SET_CALI:
#ifdef CALIBRATION_TO_FILE
#ifdef CONFIG_CUSTOM_KERNEL_SENSOR_CAL
			err = sensor_calibration_read(ID_ACCELEROMETER, obj->cali_sw);
#endif
			if(err){
				GSE_ERR("Read Cal Fail from file !!!\n");
				break;
			}
#endif
			if(atomic_read(&obj->suspend)) {
				GSE_ERR("can't perform calibration in suspend state.\n");
				err = -EINVAL;
			}else {
#ifndef CALIBRATION_TO_FILE
				data = (void __user*)arg;
				if(data == NULL) {
					err = -EINVAL;
					break;
				}
				if(copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
					err = -EFAULT;
					break;
				}
				cali[BMI160_ACC_AXIS_X] = sensor_data.x
						* obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMI160_ACC_AXIS_Y] = sensor_data.y
						* obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMI160_ACC_AXIS_Z] = sensor_data.z
						* obj->reso->sensitivity / GRAVITY_EARTH_1000;
				err = bmi160_acc_write_calibration(client, cali);
#endif
			}
			break;
		case GSENSOR_IOCTL_CLR_CALI:
			err = bmi160_acc_reset_calib(client);
			break;
		case GSENSOR_IOCTL_GET_CALI:
			GSE_LOG("IOCTL_GET_CALI.\n");
			break;
#if 0
		case GSENSOR_IOCTL_SET_ENABLE:
			data = (void __user *) arg;
			if(data == NULL) {
				err = -EINVAL;
				break;
			}
			if(copy_from_user(&enable, data, sizeof(enable))) {
				return -EFAULT;
			} else {
				GSE_LOG("GSENSOR_IOCTL_SET_ENABLE %d\n",(int)enable);
				if(enable == 1)	{
					err = bmi160_acc_set_power_mode(client, true);
				} else if(enable == 0) {
					err = bmi160_acc_set_power_mode(client, false);
				}
			}
			break;
#endif
		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
	}
	return err;
}

static struct file_operations bmi160_acc_fops = {
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

#ifndef CONFIG_HAS_EARLYSUSPEND

static int bmi160_acc_suspend(struct i2c_client *client, pm_message_t msg)
{
	int err = 0;
	u8 op_mode = 0;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;

	GSE_FUN();

	err = bmi160_acc_get_mode(client, &op_mode);
	if(msg.event == PM_EVENT_SUSPEND) {
		atomic_set(&obj->suspend, 1);
#ifdef STEP_DETECT
		if(op_mode != BMI160_ACC_MODE_SUSPEND && !atomic_read(&obj->step_detect_enable)) {
			err = bmi160_acc_set_power_mode(obj->client, false);
		}
#else
		if(op_mode != BMI160_ACC_MODE_SUSPEND) {
			err = bmi160_acc_set_power_mode(obj->client, false);
		}
#endif
		if(err) {
			GSE_ERR("write power control failed.\n");
			return err;
		}
	}
	return err;
}

static int bmi160_acc_resume(struct i2c_client *client)
{
	int err;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;

	GSE_FUN();

	if(accel_enable_status)
	{
	    err = bmi160_acc_set_power_mode(obj->client, true);
		if(err) {
			GSE_ERR("bmi160_acc_resume write power control failed.\n");
			return err;
		}
	}

	atomic_set(&obj->suspend, 0);

	return 0;
}

#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/

static void bmi160_acc_early_suspend(struct early_suspend *h)
{
	struct bmi160_acc_i2c_data *obj = container_of(h, struct bmi160_acc_i2c_data, early_drv);
	int err;
	atomic_set(&obj->suspend, 1);
	err = bmi160_acc_set_power_mode(obj->client, false);
	if(err) {
		GSE_ERR("write power control fail!!\n");
		return;
	}
}

static void bmi160_acc_late_resume(struct early_suspend *h)
{
	struct bmi160_acc_i2c_data *obj = container_of(h, struct bmi160_acc_i2c_data, early_drv);
	int err;
	err = bmi160_acc_init_client(obj->client, 0);
	if(err) {
		GSE_ERR("initialize client fail!!\n");
		return;
	}
	atomic_set(&obj->suspend, 0);
}

#endif /*CONFIG_HAS_EARLYSUSPEND*/

#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{ .compatible = "mediatek,bmi160_acc", },
	{},
};
#endif


static struct i2c_driver bmi160_acc_i2c_driver = {
	.driver = {
		.name           = BMI160_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = gsensor_of_match,
#endif
	},
	.probe      		= bmi160_acc_i2c_probe,
	.remove    			= bmi160_acc_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend            = bmi160_acc_suspend,
	.resume             = bmi160_acc_resume,
#endif
	.id_table = bmi160_acc_i2c_id,
};

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
	int err =0;
	bool power = false;
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;

	if(1 == en)
	{
		power = true;
		err = bmi160_acc_set_power_mode(obj_i2c_data->client, power);
		if(err < 0) {
			GSE_ERR("bmi160_acc_set_power_mode failed.\n");
			return -1;
		}

		GSE_LOG("bmi160_acc_enable_nodata set odr BMI160_ACCEL_ODR_1600HZ \n");
		err = bmi160_acc_set_odr(obj_i2c_data->client, BMI160_ACCEL_ODR_1600HZ);
		if (err < 0) {
			GSE_ERR("set delay parameter error!\n");
			return -1;
		}
		accel_enable_status = true;
	}
	else
	{
		power =false;
#ifdef STEP_DETECT
		if(atomic_read(&obj->step_detect_enable) == false)
#endif
		err = bmi160_acc_set_power_mode(obj_i2c_data->client, power);
		if(err < 0) {
			GSE_ERR("bmi160_acc_set_power_mode failed.\n");
			return -1;
		}
		accel_enable_status = false;
	}

	GSE_LOG("bmi160_acc_enable_nodata ok!\n");
	return err;
}

/*!
 * @brief set ODR rate value for acc
 *
 * @param[in] u64 ns for sensor delay
 *
 * @return zero success, non-zero failed
 */

static int bmi160_acc_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;
	int sample_delay = 0;
	int err = 0;

	value = (int)samplingPeriodNs/1000/1000;
	if (value <= 5)
		sample_delay = BMI160_ACCEL_ODR_400HZ;
	else if (value <= 10)
		sample_delay = BMI160_ACCEL_ODR_200HZ;
	else
		sample_delay = BMI160_ACCEL_ODR_100HZ;
	err = bmi160_acc_set_odr(obj_i2c_data->client, sample_delay);
	if (err < 0) {
		GSE_ERR("set delay parameter error!\n");
		return -1;
	}
	GSE_LOG("bmi160 acc set delay = (%d) ok.\n", value);
	return 0;
}

static int bmi160_acc_flush(void)
{
	return acc_flush_report();
}

#if 0
static int bmi160_acc_set_delay(u64 ns)
{
	int value = 0;
	int sample_delay = 0;
	int err = 0;
	value = (int)ns/1000/1000;
	if(value <= 5) {
		sample_delay = BMI160_ACCEL_ODR_400HZ;
	}
	else if(value <= 10) {
		sample_delay = BMI160_ACCEL_ODR_200HZ;
	}
	else {
		sample_delay = BMI160_ACCEL_ODR_100HZ;
	}
	err = bmi160_acc_set_odr(obj_i2c_data->client, sample_delay);
	if(err < 0) {
		GSE_ERR("set delay parameter error!\n");
		return -1;
	}
	GSE_LOG("bmi160 acc set delay = (%d) ok.\n", value);
	return 0;
}
#endif
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
static int bmi160_acc_get_data(int* x ,int* y,int* z, int* status)
{
	int err = 0;
	char buff[BMI160_BUFSIZE];
	err = bmi160_acc_read_sensor_data(obj_i2c_data->client, buff, BMI160_BUFSIZE);
	if(err < 0) {
		GSE_ERR("bmi160_acc_get_data failed.\n");
		return err;
	}
	sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_HIGH;
	return 0;
}
//LGE

#ifdef ENABLE_INT1
static void bmi160_set_interrupt(void)
{
	u8 int_out, ex_int, int1_latch = 0;
	int err = 0;

	int_out = 0x9; // int1_output_en =1, int1_edge_ctrl = 1
	int1_latch = 0x12; //int1_input_en =1, temporary 625us
	ex_int = 1; //step_detect map
	
	err = bma_i2c_write_block(obj_i2c_data->client, BMI160_USER_INTR_OUT_CTRL_ADDR, &int_out, 1);	// 0x53
	err += bma_i2c_write_block(obj_i2c_data->client, BMI160_USER_INTR_LATCH_ADDR,&int1_latch,1);	// 0x54
	err += bma_i2c_write_block(obj_i2c_data->client, BMI160_USER_INTR_MAP_0_ADDR, &ex_int, 1);		// 0x55

	if(err)
	{
		GSE_ERR("interrupt set error\n");
	}

}
#endif

#ifdef STEP_DETECT
static void bmi160_set_step_mode(int mode)
{
	u8 step_enable, config_0, config_1, data= 0;
	int err = 0;
	
	switch(mode){
		case STEP_NORMAL_MODE:
			step_enable = 1<<3; //enable step detect engine
			config_0 = 0x13;
			config_1 = 0x03;
			break;
			
		case STEP_SENSITIVE_MODE:
			step_enable = 1<<3;
			config_0 = 0x2D;
			config_1 = 0x00;
			break;
			
		case STEP_ROBUST_MODE:
			step_enable = 1<<3;
			config_0 = 0x1D;
			config_1 = 0x07;
			break;

		case STEP_MODE_OFF:
			step_enable = 0;
			config_0 = 0;
			config_1 = 0;
			break;

		default:
			break;
	}

	err += bma_i2c_write_block(obj_i2c_data->client, BMI160_USER_INT_EN_2_ADDR, &step_enable, 1);		// 0x52
	err += bma_i2c_write_block(obj_i2c_data->client, BMI160_USER_STEP_CONFIG_0_ADDR, &config_0, 1);		// 0x7A
	err += bma_i2c_write_block(obj_i2c_data->client, BMI160_USER_STEP_CONFIG_1_ADDR, &config_1, 1);		// 0x7B

	if(err)
		GSE_ERR("set step mode error\n");
}

int bmi160_set_step_enable(int enable)
{
	struct bmi160_acc_i2c_data *obj = obj_i2c_data;
	int err = 0;

    if (bmi160_acc_init_flag != 0)
    {
        GSE_LOG("return by bmi160 accel probe fail \n");
        return -1;
    }


	mutex_lock(&obj->enable_step_mutex);
	
	if(enable){
		if(!atomic_read(&obj->step_detect_enable))
		{
			atomic_set(&obj->step_detect_enable, 1);
			GSE_LOG("step_enable\n");
			bmi160_set_interrupt();
			bmi160_set_step_mode(STEP_NORMAL_MODE);
			if(sensor_power == false){
				err = bmi160_set_command_register(CMD_PMU_ACC_LP1);
			}
		}
	}else{
		if(atomic_read(&obj->step_detect_enable)){
			GSE_LOG("step_disable\n");
			atomic_set(&obj->step_detect_enable, 0);
				
			bmi160_set_step_mode(STEP_MODE_OFF);
			if(sensor_power == false){
				err = bmi160_set_command_register(CMD_PMU_ACC_SUSPEND);
			}
			else
			{
				err = bmi160_set_command_register(CMD_PMU_ACC_NORMAL);
			}

			cancel_work_sync(&obj->irq_work);
			flush_work(&obj->irq_work);
		}
	}

	mutex_unlock(&obj->enable_step_mutex);
}
EXPORT_SYMBOL(bmi160_set_step_enable);

int step_sensor_operate(void* slef, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	struct hwm_sensor_data* gsensor_data;
	int value;
	struct bmi160_acc_i2c_data *priv = obj_i2c_data;

	GSE_LOG("step detector operate cmd %d\n", command);

	switch(command)
	{
		case SENSOR_DELAY:
			//pass
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter err!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				GSE_LOG("STEP SENSOR_ENABLE %d\n", value);
				//bmi160_set_step_enable(priv->client, value);
			}
			break;
		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter err!\n");
				err = -EINVAL;
			}
			else{
				gsensor_data = (struct hwm_sensor_data *)buff_out;
				gsensor_data->values[0] = priv->step_value;
				gsensor_data->status = SENSOR_STATUS_ACCURACY_HIGH;
				gsensor_data->value_divide = 1;
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d\n",command);
			err = -1;
			break;
	}
	return err;
}
#endif

#ifdef ENABLE_INT1
static void bmi160_irq_work_func(struct work_struct *work)
{
	int err = 0;
	u8 databuf = 0;
	struct hwm_sensor_data sensor_data;
	struct i2c_client *client = bmi160_acc_i2c_client;

//	GSE_LOG("bmi160_irq_work_func\n");
	//disable_irq_nosync(obj_i2c_data->irq);

	err = bma_i2c_read_block(client,
			BMI160_USER_INTR_STAT_0_STEP_INTR__REG, &databuf, 1);

	GSE_LOG("databuf %d\n", databuf);

	if(err != SUCCESS){
		GSE_ERR("read step_int_status register err!\n");
	}

	if( databuf & 0x01){
		GSE_LOG("step occurred!!!\n");
#ifdef CONFIG_CUSTOM_KERNEL_SITUATION
		bmi160_stc_notify();
#endif
#if 0
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		sensor_data.values[0] = 1;
		obj_i2c_data->step_value = sensor_data.values[0];
		hwmsen_get_interrupt_data(ID_SLOPE_DETECTOR, &sensor_data);
#endif
	}
	enable_irq(obj_i2c_data->irq);
	enable_irq_wake(obj_i2c_data->irq);
}

static irqreturn_t bmi160_eint_handler(int irq, void *handle)
{
	struct bmi160_acc_i2c_data *data = obj_i2c_data;

	if(data == NULL)
		return IRQ_HANDLED;
	if(data->client == NULL)
		return IRQ_HANDLED;

	schedule_work(&data->irq_work);
	disable_irq_nosync(data->irq);

	return IRQ_HANDLED;
}

static int bmi160_setup_eint(void)
{
	int ret = 0;
	u32 ints[2] = {0,0};

	GSE_LOG("setup eint\n");
	obj_i2c_data->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, gse_1-eint");
	if(obj_i2c_data->irq_node){
		of_property_read_u32_array(obj_i2c_data->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		GSE_ERR("accel request_irq IRQ %d!.", ints[0]);
		obj_i2c_data->irq = irq_of_parse_and_map(obj_i2c_data->irq_node, 0);
		if(obj_i2c_data->irq < 0){
			GSE_ERR("accel request_irq IRQ LINE NOT AVAILABLE!\n");
			return -1;
		}
		GSE_LOG("accel int_gpio_number: %d irq: %d !! \n", ints[0], obj_i2c_data->irq);

		ret = request_irq(obj_i2c_data->irq, bmi160_eint_handler, IRQF_TRIGGER_RISING, "gse_1-eint", NULL);
		if(ret > 0){
			GSE_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}

	}else{
		GSE_ERR("null irq node!!\n");
		return -EINVAL;
	}

	return ret;
}
#endif

static int bmi160_acc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct i2c_client *new_client;
	struct bmi160_acc_i2c_data *obj;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};
#if 0
#ifdef STEP_DETECT
	struct hwmsen_object sldobj;
#endif
#endif

	GSE_FUN();

	err = get_accel_dts_func(client->dev.of_node, hw);
	if (err < 0) {
		GSE_ERR("get dts info fail !! err = %d\n", err);
		err = -EFAULT;
		goto exit;
	}

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(struct bmi160_acc_i2c_data));
	obj->hw = hw;
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if(err) {
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}
	obj_i2c_data = obj;
	client->addr = *hw->i2c_addr;
	obj->client = client;
	bmi160_acc_i2c_client = new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
#ifdef STEP_DETECT
	atomic_set(&obj->step_detect_enable, 0);
#endif
	mutex_init(&obj->lock);
	err = bmi160_acc_init_client(new_client, 1);
	if(err) {
		goto exit_init_failed;
	}
#ifdef STEP_DETECT	
	bmi160_setup_eint();
	INIT_WORK(&obj->irq_work, bmi160_irq_work_func);
	mutex_init(&obj->enable_step_mutex);
#endif
#ifdef FIFO_READ_USE_DMA_MODE_I2C
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	client->dev.dma_mask = &client->dev.coherent_dma_mask;
	gpDMABuf_va = (u8 *) dma_alloc_coherent(&client->dev, ACCEL_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL | GFP_DMA);	/* DMA size for customer */
	if (!gpDMABuf_va) {
		GSE_ERR("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	}
	memset(gpDMABuf_va, 0, ACCEL_DMA_MAX_TRANSACTION_LENGTH);
#endif
	err = misc_register(&bmi160_acc_device);
	if(err) {
		GSE_ERR("bmi160_acc_device register failed.\n");
		goto exit_misc_device_register_failed;
	}
	err = bmi160_acc_create_attr(&(bmi160_acc_init_info.platform_diver_addr->driver));
	if(err) {
		GSE_ERR("create attribute failed.\n");
		goto exit_create_attr_failed;
	}
	ctl.open_report_data = bmi160_acc_open_report_data;
	ctl.enable_nodata = bmi160_acc_enable_nodata;
//	ctl.set_delay = bmi160_acc_set_delay;
	ctl.batch = bmi160_acc_batch;
	ctl.flush = bmi160_acc_flush;
	ctl.is_report_input_direct = false;
	err = acc_register_control_path(&ctl);
	if (err < 0) {
		GSE_ERR("register acc control path err = %d\n", err);
		goto exit_kfree;
	}
	data.get_data = bmi160_acc_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path error.\n");
		goto exit_kfree;
	}
#if 0
#ifdef STEP_DETECT
	sldobj.self = obj;
	sldobj.polling = 0;
	sldobj.sensor_operate = step_sensor_operate;
	if(err = hwmsen_attach(ID_SLOPE_DETECTOR, &sldobj))
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}
	mutex_init(&obj->enable_step_mutex);
#endif
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend = bmi160_acc_early_suspend,
	obj->early_drv.resume = bmi160_acc_late_resume,
	register_early_suspend(&obj->early_drv);
#endif

	/* h/w init */
	obj->device.bus_read = bmi_i2c_read_wrapper;
	obj->device.bus_write = bmi_i2c_write_wrapper;
	obj->device.delay_msec = bmi_delay;
	bmi160_init(&obj->device);
	obj->cal_update =1;
#ifdef ENABLE_INT1
	bmi160_set_interrupt();
#endif

	/* fifo setting*/
	bmi160_set_fifo_header_enable(ENABLE);
	udelay(500);
	bmi160_set_fifo_time_enable(DISABLE);
	udelay(500);
	bmi160_acc_init_flag =0;
	GSE_LOG("%s: is ok.\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&bmi160_acc_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	bmi160_acc_init_flag =-1;
	return err;
}

static int bmi160_acc_i2c_remove(struct i2c_client *client)
{
	int err = 0;
	err = bmi160_acc_delete_attr(&(bmi160_acc_init_info.platform_diver_addr->driver));
	if(err) {
		GSE_ERR("delete device attribute failed.\n");
	}
	err = misc_deregister(&bmi160_acc_device);
	if(err) {
		GSE_ERR("misc_deregister fail: %d\n", err);
	}
#if 0
#ifdef STEP_DETECT
	if(err = hwmsen_detach(ID_SLOPE_DETECTOR))
	{
		GSE_ERR("hwmsen_detach fail : %d\n", err);
	}
#endif
#endif

	bmi160_acc_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(obj_i2c_data);
	return 0;
}

static int  bmi160_acc_local_init(void)
{
	if(i2c_add_driver(&bmi160_acc_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}
	if(-1 == bmi160_acc_init_flag) {
		return -1;
	}
	GSE_LOG("bmi160 acc local init.\n");
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
MODULE_AUTHOR("xiaogang.fan@cn.bosch.com");
