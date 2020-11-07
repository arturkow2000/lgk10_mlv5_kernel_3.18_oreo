/* SPL06 GOERTEK pressure sensor driver
 *
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2017 GOERTEK INC.
 * All Rights Reserved
* VERSION: V1.1
* History: V1.0 --- [2013.05.16]Driver creation
*          V1.1 --- [2017.11.06]modify source code to integrate into MTK6750S(andriod M).
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
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/math64.h>

#include <cust_baro.h>
//#include <linux/hwmsensor.h>
//#include <linux/hwmsen_dev.h>
//#include <linux/sensors_io.h>
#include "spl06.h"
//#include <linux/hwmsen_helper.h>
//#include <linux/proc_fs.h>
//#include <linux/batch.h>

//#include <mach/mt_typedefs.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>

#include "barometer.h"

#define BAR_TAG                  "[spl06] "
#define BAR_FUN(f)               printk(BAR_TAG"%s\n", __func__)
#define BAR_ERR(fmt, args...) \
	printk(BAR_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define BAR_LOG(fmt, args...)    printk(BAR_TAG fmt, ##args)

/*!
 * @brief list all the measurment time
 * that could be set[unit:0.1ms]
*/
static const u32 meastime_list[] = {
	36, 52, 84, 148, 276, 532, 1044, 2068};
/*!
 * @brief list all the sample rate
 * that could be set[unit:Hz]
*/
static const u32 samplerate_list[] = {
	1, 2, 4, 8, 16, 32, 64, 128};

/* sensor type */
enum SENSOR_TYPE_ENUM {
	SPL06_TYPE = 0x0,

	INVALID_TYPE = 0xff
};

/* power mode */
enum SPL_POWERMODE_ENUM {
	SPL_SUSPEND_MODE = 0x0,
	SPL_NORMAL_MODE,

	SPL_UNDEFINED_POWERMODE = 0xff
};

/* filter */
enum SPL_SAMPLERATE_ENUM 
{
	SPL_SAMPLERATE_1 = 0x0,
	SPL_SAMPLERATE_2,
	SPL_SAMPLERATE_4,
	SPL_SAMPLERATE_8,
	SPL_SAMPLERATE_16,
	SPL_SAMPLERATE_32,
	SPL_SAMPLERATE_64,
	SPL_SAMPLERATE_128,

	SPL_UNDEFINED_SAMPLERATE = 0xff
};

/* oversampling */
enum SPL_OVERSAMPLING_ENUM 
{
	SPL_OVERSAMPLING_1X = 0x0,
	SPL_OVERSAMPLING_2X,
	SPL_OVERSAMPLING_4X,
	SPL_OVERSAMPLING_8X,
	SPL_OVERSAMPLING_16X,
	SPL_OVERSAMPLING_32X,
	SPL_OVERSAMPLING_64X,
	SPL_OVERSAMPLING_128X,

	SPL_UNDEFINED_OVERSAMPLING = 0xff
};

/* trace */
enum BAR_TRC {
	BAR_TRC_READ 	= 0x01,
	BAR_TRC_RAWDATA = 0x02,
	BAR_TRC_IOCTL 	= 0x04,
	BAR_TRC_FILTER 	= 0x08,
	BAR_TRC_INFO 	= 0x10,
};

/* s/w filter */
struct data_filter {
	u32 raw[C_MAX_FIR_LENGTH][SPL_DATA_NUM];
	int sum[SPL_DATA_NUM];
	int num;
	int idx;
};

/* spl06 calibration */
struct spl06_calibration_data {	
    s16 c0;
    s16 c1;
    s32 c00;
    s32 c10;
    s16 c01;
    s16 c11;
    s16 c20;
    s16 c21;
    s16 c30;       
};


/* spl i2c client data */
struct spl_i2c_data {
	struct i2c_client *client;
	struct baro_hw hw;

	/* sensor info */
	u8 sensor_name[MAX_SENSOR_NAME];
	enum SENSOR_TYPE_ENUM sensor_type;
	enum SPL_POWERMODE_ENUM power_mode;
	u8 hw_filter;
	u32 i32kP;
	u32 i32kT;
  	u8 oversampling_p;
  	u8 oversampling_t;	
	u8 samplerate_p;
	u8 samplerate_t;
	struct spl06_calibration_data spl06_cali;

	/*misc*/
	struct mutex lock;
	atomic_t trace;
	atomic_t suspend;
	atomic_t filter;

#if defined(CONFIG_SPL_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
};


#ifdef CONFIG_MTK_LEGACY
static struct i2c_board_info spl_i2c_info __initdata = {
	I2C_BOARD_INFO(SPL_DEV_NAME, SPL06_I2C_ADDRESS)
};
#endif

static int spl_local_init(void);
static int  spl_remove(void);
static int spl_init_flag = -1;
static struct baro_init_info spl_init_info = {
	.name = "spl06",
	.init = spl_local_init,
	.uninit = spl_remove,
};

#if 0
/* Maintain  cust info here */
struct baro_hw baro_cust;
static struct baro_hw *hw = &baro_cust;
/* For baro driver get cust info */
struct baro_hw *get_cust_baro(void)
{
	return &baro_cust;
}
#endif

static int spl_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int spl_i2c_remove(struct i2c_client *client);
static int spl_i2c_detect(struct i2c_client *client,
	struct i2c_board_info *info);
static int spl_suspend(struct i2c_client *client, pm_message_t msg);
static int spl_resume(struct i2c_client *client);

static int spl_open_report_data(int open);
static int spl_enable_nodata(int en);
static int spl_set_delay(u64 ns);
static int spl_compute_meas_time(u32 *meastime);
static int spl_get_data(int *value, int *status);

#ifdef CONFIG_ID_TEMPERATURE
int temperature_operate(void *self, u32 command, void *buff_in,
	int size_in, void *buff_out, int size_out, int *actualout);
#endif

static int spl_init_client(struct i2c_client *client);
static int spl_set_powermode(struct i2c_client *client,
	enum SPL_POWERMODE_ENUM power_mode);
static int spl_get_pressure(struct i2c_client *client, char *buf, int bufsize);
static int spl_get_temperature(struct i2c_client *client,
	char *buf, int bufsize);
static int spl_get_chip_type(struct i2c_client *client);
static int spl_get_calibration_data(struct i2c_client *client);
static int spl_set_oversampling_p(struct i2c_client *client,
	enum SPL_OVERSAMPLING_ENUM oversampling_p);
static int spl_set_oversampling_t(struct i2c_client *client,
	enum SPL_OVERSAMPLING_ENUM oversampling_t);
static int spl_set_samplerate_p(struct i2c_client *client,
	enum SPL_SAMPLERATE_ENUM samplerate_p);
static int spl_set_samplerate_t(struct i2c_client *client,
	enum SPL_SAMPLERATE_ENUM samplerate_t);
static int spl_read_raw_pressure(struct i2c_client *client, s32 *pressure);
static int spl_read_raw_temperature(struct i2c_client *client,
	s32 *temperature);

static int spl_open(struct inode *inode, struct file *file);
static int spl_release(struct inode *inode, struct file *file);
static long spl_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg);
#if IS_ENABLED(CONFIG_COMPAT)
static long compat_spl_unlocked_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg);
#endif

static struct i2c_driver spl_i2c_driver;
static struct spl_i2c_data *obj_i2c_data;
static const struct i2c_device_id spl_i2c_id[] = {
	{ SPL_DEV_NAME, 0 },
	{}
};

/* I2C operation functions */
static int spl_i2c_read_block(struct i2c_client *client,
	u8 addr, u8 *data, u8 len)
{
	u8 *rxbuf = data;
	u8 left = len;
	u8 retry;
	u8 offset = 0;

	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &addr,
			.len = 1,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
		},
	};

	if (rxbuf == NULL)
		return -1;

	while (left > 0) {
		retry = 0;
		msg[1].buf = &rxbuf[offset];

		if (left > C_I2C_FIFO_SIZE) {
			msg[1].len = C_I2C_FIFO_SIZE;
			left -= C_I2C_FIFO_SIZE;
			offset += C_I2C_FIFO_SIZE;
		} else {
			msg[1].len = left;
			left = 0;
		}

		while (i2c_transfer(client->adapter, &msg[0], 2) != 2) {
			retry++;

			if (retry == 20) {
				BAR_ERR("i2c read register=%#x length=%d failed\n", addr + offset, len);
				return -EIO;
			}
		}
	}

	return 0;
}

static int spl_i2c_write_block(struct i2c_client *client, u8 addr,
	u8 *data, u8 len)
{
	u8 buffer[C_I2C_FIFO_SIZE];
	u8 *txbuf = data;
	u8 left = len;
	u8 offset = 0;
	u8 retry = 0;

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.buf = buffer,
	};

	if (txbuf == NULL)
		return -1;

	while (left > 0) {
		retry = 0;
		/* register address */
		buffer[0] = addr + offset;

		if (left >= C_I2C_FIFO_SIZE) {
			memcpy(&buffer[1], &txbuf[offset], C_I2C_FIFO_SIZE - 1);
			msg.len = C_I2C_FIFO_SIZE;
			left -= C_I2C_FIFO_SIZE - 1;
			offset += C_I2C_FIFO_SIZE - 1;
		} else {
			memcpy(&buffer[1], &txbuf[offset], left);
			msg.len = left + 1;
			left = 0;
		}

		while (i2c_transfer(client->adapter, &msg, 1) != 1) {
			retry++;

			if (retry == 20) {
				BAR_ERR("i2c write register=%#x length=%d failed\n", buffer[0], len);
				return -EIO;
			}

			BAR_LOG("i2c write addr %#x, retry %d\n",
				buffer[0], retry);
		}
	}

	return 0;
}

static void spl_power(struct baro_hw *hw, unsigned int on)
{

}

/*----------------------------------------------------------------------------*/
static const struct file_operations spl_fops = {
	.open = spl_open,
	.release = spl_release,
	.unlocked_ioctl = spl_unlocked_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_spl_unlocked_ioctl,
#endif
};

static struct miscdevice spl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "barometer",
	.fops = &spl_fops,
};

#ifdef CONFIG_OF
static const struct of_device_id baro_of_match[] = {

	{ .compatible = "mediatek,baro" },
	{},
};
#endif

static struct i2c_driver spl_i2c_driver = {
	.driver = {
	.name = SPL_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = baro_of_match,
#endif
},
.probe = spl_i2c_probe,
.remove = spl_i2c_remove,
.detect = spl_i2c_detect,
.suspend = spl_suspend,
.resume = spl_resume,
.id_table = spl_i2c_id,
};

static int spl_open(struct inode *inode, struct file *file)
{
	file->private_data = obj_i2c_data;

	if (file->private_data == NULL) {
		BAR_ERR("null pointer\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int spl_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long spl_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct spl_i2c_data *obj = (struct spl_i2c_data *)file->private_data;
	struct i2c_client *client = obj->client;
	char strbuf[SPL_BUFSIZE];
	u32 dat = 0;
	void __user *data;
	int err = 0;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
		(void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
		(void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		BAR_ERR("access error: %08X, (%2d, %2d)\n",
			cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case BAROMETER_IOCTL_INIT:
		spl_init_client(client);
		err = spl_set_powermode(client, SPL_NORMAL_MODE);
		if (err) {
			err = -EFAULT;
			break;
		}
		break;

	case BAROMETER_IOCTL_READ_CHIPINFO:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		strncpy(strbuf, obj->sensor_name, sizeof(strbuf));
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case BAROMETER_GET_PRESS_DATA:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		spl_get_pressure(client, strbuf, SPL_BUFSIZE);
		if (sscanf(strbuf, "%x", &dat) != 1)
			BAR_ERR("sscanf parsing fail\n");
		if (copy_to_user(data, &dat, sizeof(dat))) {
			err = -EFAULT;
			break;
		}
		break;

	case BAROMETER_GET_TEMP_DATA:
		data = (void __user *) arg;
		if (NULL == data) {
			err = -EINVAL;
			break;
		}
		spl_get_temperature(client, strbuf, SPL_BUFSIZE);
		if (sscanf(strbuf, "%x", &dat) != 1)
			BAR_ERR("sscanf parsing fail\n");
		if (copy_to_user(data, &dat, sizeof(dat))) {
			err = -EFAULT;
			break;
		}
		break;

	default:
		BAR_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long compat_spl_unlocked_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{

	BAR_FUN();

	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		BAR_ERR("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	//case COMPAT_BAROMETER_IOCTL_INIT:
	//case COMPAT_BAROMETER_IOCTL_READ_CHIPINFO:
	//case COMPAT_BAROMETER_GET_PRESS_DATA:
	//case COMPAT_BAROMETER_GET_TEMP_DATA: {
		BAR_LOG("compat_ion_ioctl : BAROMETER_IOCTL_XXX command is 0x%x\n", cmd);
		return filp->f_op->unlocked_ioctl(filp, cmd,
			(unsigned long)compat_ptr(arg));
	//}
	default:
		BAR_ERR("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif

/* spl setting initialization */
static int spl_init_client(struct i2c_client *client)
{
	int err = 0;

	/* BAR_FUN(); */

	err = spl_get_chip_type(client);
	if (err < 0) {
		BAR_ERR("get chip type failed, err = %d\n", err);
		return err;
	}

	err = spl_get_calibration_data(client);
	if (err < 0) {
		BAR_ERR("get calibration data failed, err = %d\n", err);
		return err;
	}

	err = spl_set_powermode(client, SPL_SUSPEND_MODE);
	if (err < 0) {
		BAR_ERR("set power mode failed, err = %d\n", err);
		return err;
	}

	err = spl_set_samplerate_p(client, SPL_SAMPLERATE_2);
	if (err < 0) {
		BAR_ERR("set pressure samplerate failed, err = %d\n", err);
		return err;
	}

	err = spl_set_samplerate_t(client, SPL_SAMPLERATE_1);
	if (err < 0) {
		BAR_ERR("set temperature samplerate failed, err = %d\n", err);
		return err;
	}

	err = spl_set_oversampling_p(client, SPL_OVERSAMPLING_16X);
	if (err < 0) {
		BAR_ERR("set pressure oversampling failed, err = %d\n", err);
		return err;
	}

	err = spl_set_oversampling_t(client, SPL_OVERSAMPLING_1X);
	if (err < 0) {
		BAR_ERR("set temperature oversampling failed, err = %d\n", err);
		return err;
	}

	return 0;
}

static int spl_set_powermode(struct i2c_client *client,
	enum SPL_POWERMODE_ENUM power_mode)
{
	struct spl_i2c_data *obj = i2c_get_clientdata(client);
	u8 err = 0, data = 0, actual_power_mode = 0;

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		BAR_LOG("[%s] power_mode = %d, old power_mode = %d\n", __func__,
			power_mode, obj->power_mode);

	if (power_mode == obj->power_mode)
		return 0;

	mutex_lock(&obj->lock);

	if (obj->sensor_type == SPL06_TYPE) {/* SPL06 */
		if (power_mode == SPL_SUSPEND_MODE) {
			actual_power_mode = SPL06_SLEEP_MODE;
		} else if (power_mode == SPL_NORMAL_MODE) {
			actual_power_mode = SPL06_NORMAL_MODE;
		} else {
			err = -EINVAL;
			BAR_ERR("invalid power mode = %d\n", power_mode);
			mutex_unlock(&obj->lock);
			return err;
		}
		err = spl_i2c_read_block(client,
			SPL06_CTRL_MEAS_REG_POWER_MODE__REG, &data, 1);
		data = SPL_SET_BITSLICE(data,
			SPL06_CTRL_MEAS_REG_POWER_MODE, actual_power_mode);
		err += spl_i2c_write_block(client,
			SPL06_CTRL_MEAS_REG_POWER_MODE__REG, &data, 1);
	}

	if (err < 0)
		BAR_ERR("set power mode failed, err = %d, sensor name = %s\n",
			err, obj->sensor_name);
	else
		obj->power_mode = power_mode;

	mutex_unlock(&obj->lock);
	return err;
}

#ifdef ENABLE_FLOAT

/*get compensated temperature
 *unit:0.01 degree*/
static int spl_get_temperature(struct i2c_client *client,
	char *buf, int bufsize)
{  
	struct spl_i2c_data *obj;
	int status;
	s32 utemp = 0;/* uncompensated temperature */
	s32 temperature = 0;
	float fTemperature = 0.0;
    float fTsc;

	BAR_ERR("spl_get_temperature start\n");

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	obj = i2c_get_clientdata(client);

	status = spl_read_raw_temperature(client, &utemp);
	if (status != 0)
		return status;

	if (obj->sensor_type == SPL06_TYPE) {/* SPL06 */	
    	fTsc = utemp / (float)obj->i32kT;
    	fTemperature = obj->spl06_cali.c0 * 0.5 + obj->spl06_cali.c1 * fTsc;
		/* The result temperature unit should be 0.01deg */
		temperature = (s32)(fTemperature*100);
	}
		
	sprintf(buf, "%08x", temperature);
	if (atomic_read(&obj->trace) & BAR_TRC_IOCTL) {
		BAR_LOG("temperature: %d\n", temperature);
		BAR_LOG("temperature/100: %d\n", temperature / 100);
		BAR_LOG("compensated temperature value: %s\n", buf);
	}

	BAR_ERR("aa temperature: %d\n", temperature);
	BAR_ERR("bb temperature/100: %d\n", temperature / 100);

	return status;
}

/*
*get compensated pressure
*unit: hectopascal(Pa)
*/
static int spl_get_pressure(struct i2c_client *client, 
	char *buf, int bufsize)
{
	struct spl_i2c_data *obj;
	int status;
	s32 utemp = 0, upressure = 0, pressure = 0;

	float fTsc, fPsc;
    float qua2, qua3;
	float fPressure;

	BAR_ERR("spl_get_pressure start\n");

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	obj = i2c_get_clientdata(client);

	status = spl_read_raw_temperature(client, &utemp);
	if (status != 0)
		goto exit;

	status = spl_read_raw_pressure(client, &upressure);
	if (status != 0)
		goto exit;

	if (obj->sensor_type == SPL06_TYPE) {/* SPL06 */
    	fTsc = utemp / (float)obj->i32kT;
    	fPsc = upressure / (float)obj->i32kP;
    	qua2 = obj->spl06_cali.c10 + fPsc * (obj->spl06_cali.c20 + fPsc* obj->spl06_cali.c30);
    	qua3 = fTsc * fPsc * (obj->spl06_cali.c11 + fPsc * obj->spl06_cali.c21);

    	fPressure = obj->spl06_cali.c00 + fPsc * qua2 + fTsc * obj->spl06_cali.c01 + qua3;
		pressure = (s32)fPressure;
	}
	sprintf(buf, "%08x", pressure);
	if (atomic_read(&obj->trace) & BAR_TRC_IOCTL) {
		BAR_LOG("pressure: %d\n", pressure);
		BAR_LOG("pressure/100: %d\n", pressure / 100);
		BAR_LOG("compensated pressure value: %s\n", buf);
	}
exit:
	return status;
}

#else

/*get compensated temperature
 *unit:0.01 degree*/
static int spl_get_temperature(struct i2c_client *client,
	char *buf, int bufsize)
{
	struct spl_i2c_data *obj;
	int status;
	s32 utemp = 0;/* uncompensated temperature */
	s32 temperature = 0;
	s32 fTsc;
	s64 utemp2;

	BAR_ERR("spl_get_temperature start\n");

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	obj = i2c_get_clientdata(client);

	status = spl_read_raw_temperature(client, &utemp);
	if (status != 0)
		return status;

	utemp2 = (s64)utemp << 16;
	if (obj->sensor_type == SPL06_TYPE) {/* SPL06 */
		fTsc = (s32)(div64_long(utemp2, obj->i32kT));
//		fTsc = (s32)(((s64)utemp << 16) / obj->i32kT);
//		fTsc = (s32)((utemp << 16) / obj->i32kT);
		/* Actual temperature should be divided by 256*/
		temperature = ((s32)obj->spl06_cali.c0 << 7) + ((s32)((s32)obj->spl06_cali.c1 * fTsc) >> 8);
		/* The result temperature unit should be 0.01deg */
		temperature = (temperature*100) >> 8;
	}

	sprintf(buf, "%08x", temperature);
	if (atomic_read(&obj->trace) & BAR_TRC_IOCTL) {
		BAR_LOG("temperature: %d\n", temperature);
		BAR_LOG("temperature/100: %d\n", temperature / 100);
		BAR_LOG("compensated temperature value: %s\n", buf);
	}

	BAR_ERR("aa temperature: %d\n", temperature);
	BAR_ERR("bb temperature/100: %d\n", temperature / 100);

	return status;
}

/*
*get compensated pressure
*unit: hectopascal(Pa)
*/
static int spl_get_pressure(struct i2c_client *client, 
	char *buf, int bufsize)
{
	struct spl_i2c_data *obj;
	int status;
	s32 utemp = 0, upressure = 0, pressure = 0;
	s64 utemp2 = 0, upressure2 = 0;
//	char temp_buf[SPL_BUFSIZE];

	s32 fTsc, fPsc;
    s32 qua2, qua3;

//	BAR_ERR("spl_get_pressure start\n");

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	obj = i2c_get_clientdata(client);
	
	status = spl_read_raw_temperature(client, &utemp);
	if (status != 0)
		goto exit;

	utemp2 = (s64)utemp << 16; // hun

	status = spl_read_raw_pressure(client, &upressure);
	if (status != 0)
		goto exit;

	upressure2 = (s64)upressure << 16; // hun

	if (obj->sensor_type == SPL06_TYPE) {/* SPL06 */
//    	fTsc = (s32)(((s64)utemp << 16) / obj->i32kT);
		fTsc = (s32)(div64_long(utemp2, obj->i32kT));

//    	fPsc = (s32)(((s64)upressure << 16) / obj->i32kP);
        fPsc = (s32)(div64_long(upressure2, (s32)obj->i32kP));

//    	fTsc = (s32)((utemp << 16) / obj->i32kT);
//   	fPsc = (s32)((upressure << 16) / obj->i32kP);
	
		qua2 = ((s32)obj->spl06_cali.c20 << 8) + ((s32)((s32)obj->spl06_cali.c30 * fPsc) >> 8);
		qua3 = ((s32)obj->spl06_cali.c10 << 8) + (s32)(((s64)qua2 * (s64)fPsc) >> 16);
		pressure = ((s32)obj->spl06_cali.c00 << 8) + (s32)(((s64)qua3 * (s64)fPsc) >> 16);
		pressure += ((s32)((s32)obj->spl06_cali.c01 * fTsc) >> 8);

		qua2 = ((s32)obj->spl06_cali.c11 << 8) + ((s32)((s32)obj->spl06_cali.c21 * fPsc) >> 8);
		qua3 = (s32)(((s64)qua2 * (s64)fPsc) >> 16);	
		pressure += (s32)(((s64)qua3 * (s64)fTsc) >> 16);
		/* Actual temperature should be divided by 256*/
    	pressure = pressure >> 8;
	
	}	
	sprintf(buf, "%08x", pressure);
	if (atomic_read(&obj->trace) & BAR_TRC_IOCTL) {
		BAR_LOG("pressure: %d\n", pressure);
		BAR_LOG("pressure/100: %d\n", pressure / 100);
		BAR_LOG("compensated pressure value: %s\n", buf);
	}
exit:
	return status;
}
#endif

/* get chip type */
static int spl_get_chip_type(struct i2c_client *client)
{
	int err = 0;
	u8 chip_id = 0;
	struct spl_i2c_data *obj = i2c_get_clientdata(client);
	int count = 0;
	/* BAR_FUN(f); */

	err = spl_i2c_read_block(client, SPL06_CHIP_ID_REG, &chip_id, 0x01);
	count++;
	udelay(1000);
	while ((count < 3) && (err < 0)) {
		err = spl_i2c_read_block(client, SPL06_CHIP_ID_REG, &chip_id, 0x01);
		count++;
		udelay(1000);
	}
	if (err != 0)
		return err;

	if (chip_id == SPL06_CHIP_ID) {
		obj->sensor_type = SPL06_TYPE;
		strncpy(obj->sensor_name, "spl06", sizeof(obj->sensor_name));
	} else {
		obj->sensor_type = INVALID_TYPE;
		strncpy(obj->sensor_name, "unknown sensor", sizeof(obj->sensor_name));
	}

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		BAR_LOG("[%s]chip id = %#x, sensor name = %s\n", __func__,
			chip_id, obj->sensor_name);

	if (obj->sensor_type == INVALID_TYPE) {
		BAR_ERR("unknown pressure sensor\n");
		return -1;
	}
	return 0;
}

static int spl_get_calibration_data(struct i2c_client *client)
{
	struct spl_i2c_data *obj =
		(struct spl_i2c_data *)i2c_get_clientdata(client);
	int err = 0;
	u8 a_data_u8r[3] = {0};

	if (obj->sensor_type == SPL06_TYPE) {
		err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START, &a_data_u8r[0], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+1, &a_data_u8r[1], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    obj->spl06_cali.c0 = (s16)a_data_u8r[0]<<4 | a_data_u8r[1]>>4;
	    obj->spl06_cali.c0 = (obj->spl06_cali.c0&0x0800)?(0xF000|obj->spl06_cali.c0):obj->spl06_cali.c0;

		err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+1, &a_data_u8r[0], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+2, &a_data_u8r[1], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    obj->spl06_cali.c1 = (s16)(a_data_u8r[0]&0x0F)<<8 | a_data_u8r[1];
	    obj->spl06_cali.c1 = (obj->spl06_cali.c1&0x0800)?(0xF000|obj->spl06_cali.c1):obj->spl06_cali.c1;
		
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+3, &a_data_u8r[0], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+4, &a_data_u8r[1], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+5, &a_data_u8r[2], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    obj->spl06_cali.c00 = (s32)a_data_u8r[0]<<12 | (s32)a_data_u8r[1]<<4 | (s32)a_data_u8r[2]>>4;
	    obj->spl06_cali.c00 = (obj->spl06_cali.c00&0x080000)?(0xFFF00000|obj->spl06_cali.c00):obj->spl06_cali.c00;
		
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+5, &a_data_u8r[0], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+6, &a_data_u8r[1], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}

	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+7, &a_data_u8r[2], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    obj->spl06_cali.c10 = (s32)(a_data_u8r[0]&0x0F)<<16 | (s32)a_data_u8r[1]<<8 | a_data_u8r[2];
	    obj->spl06_cali.c10 = (obj->spl06_cali.c10&0x080000)?(0xFFF00000|obj->spl06_cali.c10):obj->spl06_cali.c10;
		
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+8, &a_data_u8r[0], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+9, &a_data_u8r[1], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    obj->spl06_cali.c01 = (s16)a_data_u8r[0]<<8 | a_data_u8r[1];
		
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+10, &a_data_u8r[0], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+11, &a_data_u8r[1], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    obj->spl06_cali.c11 = (s16)a_data_u8r[0]<<8 | a_data_u8r[1];
		
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+12, &a_data_u8r[0], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+13, &a_data_u8r[1], 1);
		if (err < 0){
		 	BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    obj->spl06_cali.c20 = (s16)a_data_u8r[0]<<8 | a_data_u8r[1];
		
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+14, &a_data_u8r[0], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+15, &a_data_u8r[1], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    obj->spl06_cali.c21 = (s16)a_data_u8r[0]<<8 | a_data_u8r[1];
		
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+16, &a_data_u8r[0], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    err = spl_i2c_read_block(client, SPL06_CALIBRATION_DATA_START+17, &a_data_u8r[1], 1);
		if (err < 0) {
			BAR_ERR("read calibration_data failed, err = %d\n", err);
			return err;
		}
	    obj->spl06_cali.c30 = (s16)a_data_u8r[0]<<8 | a_data_u8r[1];
		
   	}	
	return 0;
}

static int spl_set_samplerate_p(struct i2c_client *client,
	enum SPL_SAMPLERATE_ENUM samplerate_p)
{
	struct spl_i2c_data *obj = i2c_get_clientdata(client);
	u8 err = 0, data = 0, actual_samplerate_p = 0;

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		BAR_LOG("[%s] samplerate_p = %d, old samplerate_p = %d\n", __func__,
			samplerate_p, obj->samplerate_p);

	if (samplerate_p == obj->samplerate_p)
		return 0;

	mutex_lock(&obj->lock);

	if (obj->sensor_type == SPL06_TYPE) {/* SPL06 */
		if (samplerate_p == SPL_SAMPLERATE_1)
			actual_samplerate_p = SPL06_SAMPLERATE_1;
		else if (samplerate_p == SPL_SAMPLERATE_2)
			actual_samplerate_p = SPL06_SAMPLERATE_2;
		else if (samplerate_p == SPL_SAMPLERATE_4)
			actual_samplerate_p = SPL06_SAMPLERATE_4;
		else if (samplerate_p == SPL_SAMPLERATE_8)
			actual_samplerate_p = SPL06_SAMPLERATE_8;
		else if (samplerate_p == SPL_SAMPLERATE_16)
			actual_samplerate_p = SPL06_SAMPLERATE_16;
		else if (samplerate_p == SPL_SAMPLERATE_32)
			actual_samplerate_p = SPL06_SAMPLERATE_32;
		else if (samplerate_p == SPL_SAMPLERATE_64)
			actual_samplerate_p = SPL06_SAMPLERATE_64;
		else if (samplerate_p == SPL_SAMPLERATE_128)
			actual_samplerate_p = SPL06_SAMPLERATE_128;		
		else {
			err = -EINVAL;
			BAR_ERR("invalid samplerate_p = %d\n",
				samplerate_p);
			mutex_unlock(&obj->lock);
			return err;
		}
		err = spl_i2c_read_block(client,
			SPL06_PRS_CFG_REG_SAMPLERATE__REG, &data, 1);
		data = SPL_SET_BITSLICE(data,
			SPL06_PRS_CFG_REG_SAMPLERATE, actual_samplerate_p);
		err += spl_i2c_write_block(client,
			SPL06_PRS_CFG_REG_SAMPLERATE__REG, &data, 1);
	}

	if (err < 0)
		BAR_ERR("set pressure samplerate failed, err = %d,sensor name = %s\n", err, obj->sensor_name);
	else
		obj->samplerate_p = samplerate_p;

	mutex_unlock(&obj->lock);
	return err;
}

static int spl_set_samplerate_t(struct i2c_client *client,
	enum SPL_SAMPLERATE_ENUM samplerate_t)
{
	struct spl_i2c_data *obj = i2c_get_clientdata(client);
	u8 err = 0, data = 0, actual_samplerate_t = 0;

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		BAR_LOG("[%s] samplerate_t = %d, old samplerate_t = %d\n", __func__,
			samplerate_t, obj->samplerate_t);

	if (samplerate_t == obj->samplerate_t)
		return 0;

	mutex_lock(&obj->lock);

	if (obj->sensor_type == SPL06_TYPE) {/* SPL06 */
		if (samplerate_t == SPL_SAMPLERATE_1)
			actual_samplerate_t = SPL06_SAMPLERATE_1;
		else if (samplerate_t == SPL_SAMPLERATE_2)
			actual_samplerate_t = SPL06_SAMPLERATE_2;
		else if (samplerate_t == SPL_SAMPLERATE_4)
			actual_samplerate_t = SPL06_SAMPLERATE_4;
		else if (samplerate_t == SPL_SAMPLERATE_8)
			actual_samplerate_t = SPL06_SAMPLERATE_8;
		else if (samplerate_t == SPL_SAMPLERATE_16)
			actual_samplerate_t = SPL06_SAMPLERATE_16;
		else if (samplerate_t == SPL_SAMPLERATE_32)
			actual_samplerate_t = SPL06_SAMPLERATE_32;
		else if (samplerate_t == SPL_SAMPLERATE_64)
			actual_samplerate_t = SPL06_SAMPLERATE_64;
		else if (samplerate_t == SPL_SAMPLERATE_128)
			actual_samplerate_t = SPL06_SAMPLERATE_128;		
		else {
			err = -EINVAL;
			BAR_ERR("invalid samplerate_t = %d\n",
				samplerate_t);
			mutex_unlock(&obj->lock);
			return err;
		}
		err = spl_i2c_read_block(client,
			SPL06_TMP_CFG_REG_SAMPLERATE__REG, &data, 1);
		data = SPL_SET_BITSLICE(data,
			SPL06_TMP_CFG_REG_SAMPLERATE, actual_samplerate_t);
		err += spl_i2c_write_block(client,
			SPL06_TMP_CFG_REG_SAMPLERATE__REG, &data, 1); 
	}

	if (err < 0)
		BAR_ERR("set temperature samplerate failed, err = %d,sensor name = %s\n", err, obj->sensor_name);
	else
		obj->samplerate_t = samplerate_t;

	mutex_unlock(&obj->lock);
	return err;
}

static int spl_set_oversampling_p(struct i2c_client *client,
	enum SPL_OVERSAMPLING_ENUM oversampling_p)
{
	struct spl_i2c_data *obj = i2c_get_clientdata(client);
	u8 err = 0, data = 0, actual_oversampling_p = 0;

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		BAR_LOG("[%s] oversampling_p = %d, old oversampling_p = %d\n", __func__,
			oversampling_p, obj->oversampling_p);

	if (oversampling_p == obj->oversampling_p)
		return 0;

	mutex_lock(&obj->lock);

	if (obj->sensor_type == SPL06_TYPE) {/* SPL06 */
		if (oversampling_p == SPL_OVERSAMPLING_1X) {
			actual_oversampling_p = SPL06_OVERSAMPLING_1X;
			obj->i32kP = 524288;
		}
		else if (oversampling_p == SPL_OVERSAMPLING_2X) {
			actual_oversampling_p = SPL06_OVERSAMPLING_2X;
			obj->i32kP = 1572864;
		}
		else if (oversampling_p == SPL_OVERSAMPLING_4X) {
			actual_oversampling_p = SPL06_OVERSAMPLING_4X;
			obj->i32kP = 3670016;
		}
		else if (oversampling_p == SPL_OVERSAMPLING_8X) {
			actual_oversampling_p = SPL06_OVERSAMPLING_8X;
			obj->i32kP = 7864320;
		}
		else if (oversampling_p == SPL_OVERSAMPLING_16X) {
			actual_oversampling_p = SPL06_OVERSAMPLING_16X;
			obj->i32kP = 253952;
		}
		else if (oversampling_p == SPL_OVERSAMPLING_32X) {
			actual_oversampling_p = SPL06_OVERSAMPLING_32X;
			obj->i32kP = 516096;
		}
		else if (oversampling_p == SPL_OVERSAMPLING_64X) {
			actual_oversampling_p = SPL06_OVERSAMPLING_64X;
			obj->i32kP = 1040384;
		}
		else if (oversampling_p == SPL_OVERSAMPLING_128X) {
			actual_oversampling_p = SPL06_OVERSAMPLING_128X;
			obj->i32kP = 2088960;
		}
		else {
			err = -EINVAL;
			BAR_ERR("invalid oversampling_p = %d\n",
				oversampling_p);
			mutex_unlock(&obj->lock);
			return err;
		}
		err = spl_i2c_read_block(client,
			SPL06_PRS_CFG_REG_OVERSAMPLING__REG, &data, 1);
		data = SPL_SET_BITSLICE(data,
			SPL06_PRS_CFG_REG_OVERSAMPLING, actual_oversampling_p);
		err += spl_i2c_write_block(client,
			SPL06_PRS_CFG_REG_OVERSAMPLING__REG, &data, 1);
	}

	if (err < 0)
		BAR_ERR("set pressure oversampling failed, err = %d,sensor name = %s\n", err, obj->sensor_name);
	else
		obj->oversampling_p = oversampling_p;

	if(oversampling_p > SPL_OVERSAMPLING_8X) {		 
		err = spl_i2c_read_block(client, 
			SPL06_CONFIG_REG, &data, 1);
		data |= SPL06_PRESSURE_SHIFT;		 
		err += spl_i2c_write_block(client, 
			SPL06_CONFIG_REG, &data, 1);
	} else {		 
		err = spl_i2c_read_block(client, 
			SPL06_CONFIG_REG, &data, 1);
		data &= (~SPL06_PRESSURE_SHIFT);		
		err += spl_i2c_write_block(client, 
			SPL06_CONFIG_REG, &data, 1);
	}
	if (err < 0)
		BAR_ERR("set pressure configuration failed, err = %d, sensor name = %s\n", err, obj->sensor_name);

	mutex_unlock(&obj->lock);
	return err;
}

static int spl_set_oversampling_t(struct i2c_client *client,
	enum SPL_OVERSAMPLING_ENUM oversampling_t)
{
	struct spl_i2c_data *obj = i2c_get_clientdata(client);
	u8 err = 0, data = 0, actual_oversampling_t = 0;

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		BAR_LOG("[%s] oversampling_t = %d, old oversampling_t = %d\n", __func__,
			oversampling_t, obj->oversampling_t);

	if (oversampling_t == obj->oversampling_t)
		return 0;

	mutex_lock(&obj->lock);

	if (obj->sensor_type == SPL06_TYPE) {/* SPL06 */
		if (oversampling_t == SPL_OVERSAMPLING_1X) {
			actual_oversampling_t = SPL06_OVERSAMPLING_1X;
			obj->i32kT = 524288;
		}
		else if (oversampling_t == SPL_OVERSAMPLING_2X) {
			actual_oversampling_t = SPL06_OVERSAMPLING_2X;
			obj->i32kT = 1572864;
		}
		else if (oversampling_t == SPL_OVERSAMPLING_4X) {
			actual_oversampling_t = SPL06_OVERSAMPLING_4X;
			obj->i32kT = 3670016;
		}
		else if (oversampling_t == SPL_OVERSAMPLING_8X) {
			actual_oversampling_t = SPL06_OVERSAMPLING_8X;
			obj->i32kT = 7864320;
		}
		else if (oversampling_t == SPL_OVERSAMPLING_16X) {
			actual_oversampling_t = SPL06_OVERSAMPLING_16X;
			obj->i32kT = 253952;
		}
		else if (oversampling_t == SPL_OVERSAMPLING_32X) {
			actual_oversampling_t = SPL06_OVERSAMPLING_32X;
			obj->i32kT = 516096;
		}
		else if (oversampling_t == SPL_OVERSAMPLING_64X) {
			actual_oversampling_t = SPL06_OVERSAMPLING_64X;
			obj->i32kT = 1040384;
		}
		else if (oversampling_t == SPL_OVERSAMPLING_128X) {
			actual_oversampling_t = SPL06_OVERSAMPLING_128X;
			obj->i32kT = 2088960;
		}
		else {
			err = -EINVAL;
			BAR_ERR("invalid oversampling_t = %d\n",
				oversampling_t);
			mutex_unlock(&obj->lock);
			return err;
		}
		err = spl_i2c_read_block(client,
			SPL06_TMP_CFG_REG_OVERSAMPLING__REG, &data, 1);
		data = SPL_SET_BITSLICE(data,
			SPL06_TMP_CFG_REG_OVERSAMPLING, actual_oversampling_t);
		data |= SPL06_TMP_SOURCE_EXT;
		err += spl_i2c_write_block(client,
			SPL06_TMP_CFG_REG_OVERSAMPLING__REG, &data, 1);
	}

	if (err < 0)
		BAR_ERR("set temperature oversampling failed, err = %d, sensor name = %s\n", err, obj->sensor_name);
	else
		obj->oversampling_t = oversampling_t;

	if(oversampling_t > SPL_OVERSAMPLING_8X)	  
	{		 
		err = spl_i2c_read_block(client, 
			SPL06_CONFIG_REG, &data, 1);
		data |= SPL06_TEMPERATURE_SHIFT;		 
		err += spl_i2c_write_block(client, 
			SPL06_CONFIG_REG, &data, 1);
	}	 
	else	
	{		 
		err = spl_i2c_read_block(client, 
			SPL06_CONFIG_REG, &data, 1);
		data &= (~SPL06_TEMPERATURE_SHIFT);		
		err += spl_i2c_write_block(client, 
			SPL06_CONFIG_REG, &data, 1);
	} 
	if (err < 0)
		BAR_ERR("set temperature configuration failed, err = %d, sensor name = %s\n", err, obj->sensor_name);

	mutex_unlock(&obj->lock);
	return err;
}

static int spl_read_raw_temperature(struct i2c_client *client,
	s32 *temperature)
{
	struct spl_i2c_data *obj;
	s32 err = 0;
	s32 utemp = 0;
	u8 a_data_u8r[3] = {0};

	if (NULL == client) {
		err = -EINVAL;
		return err;
	}

	obj = i2c_get_clientdata(client);

	mutex_lock(&obj->lock);

	if (obj->sensor_type == SPL06_TYPE) {/* SPL06 */	
	    err  = spl_i2c_read_block(client, 
			SPL06_TEMPERATURE_MSB_REG, &a_data_u8r[0], 1);
	    err += spl_i2c_read_block(client, 
			SPL06_TEMPERATURE_LSB_REG, &a_data_u8r[1], 1);
	    err += spl_i2c_read_block(client, 
			SPL06_TEMPERATURE_XLSB_REG, &a_data_u8r[2], 1);
		if (err < 0) {
			BAR_ERR("read raw temperature failed, err = %d\n", err);
			mutex_unlock(&obj->lock);
			return err;
		}

		utemp = (s32)a_data_u8r[0]<<16 | (s32)a_data_u8r[1]<<8 | (s32)a_data_u8r[2];    
		*temperature = (utemp&0x800000) ? (0xFF000000|utemp) : utemp;
	}	
	mutex_unlock(&obj->lock);

	return err;
}

static int spl_read_raw_pressure(struct i2c_client *client, s32 *pressure)
{
	struct spl_i2c_data *priv;
	s32 err = 0;
	s32 upressure = 0;
	u8 a_data_u8r[3] = {0};

	if (NULL == client) {
		err = -EINVAL;
		return err;
	}

	priv = i2c_get_clientdata(client);

	mutex_lock(&priv->lock);

	if (priv->sensor_type == SPL06_TYPE) {/* SPL06 */
	    err  = spl_i2c_read_block(client, 
			SPL06_PRESSURE_MSB_REG, &a_data_u8r[0], 1);
	    err += spl_i2c_read_block(client, 
			SPL06_PRESSURE_LSB_REG, &a_data_u8r[1], 1);
	    err += spl_i2c_read_block(client, 
			SPL06_PRESSURE_XLSB_REG, &a_data_u8r[2], 1);
		if (err < 0) {
			BAR_ERR("read raw pressure failed, err = %d\n", err);
			mutex_unlock(&priv->lock);
			return err;
		}

		upressure = (s32)a_data_u8r[0]<<16 | (s32)a_data_u8r[1]<<8 | (s32)a_data_u8r[2];	
		*pressure = (upressure&0x800000) ? (0xFF000000|upressure) : upressure;
	}	

#ifdef CONFIG_SPL_LOWPASS
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
				priv->fir.raw[priv->fir.num][SPL_PRESSURE] =
					*pressure;
				priv->fir.sum[SPL_PRESSURE] += *pressure;
				if (atomic_read(&priv->trace) &
					BAR_TRC_FILTER) {
					BAR_LOG("add [%2d] [%5d] => [%5d]\n",
						priv->fir.num,
						priv->fir.raw
						[priv->fir.num][SPL_PRESSURE],
						priv->fir.sum[SPL_PRESSURE]);
				}
				priv->fir.num++;
				priv->fir.idx++;
			} else {
				idx = priv->fir.idx % firlen;
				priv->fir.sum[SPL_PRESSURE] -=
					priv->fir.raw[idx][SPL_PRESSURE];
				priv->fir.raw[idx][SPL_PRESSURE] = *pressure;
				priv->fir.sum[SPL_PRESSURE] += *pressure;
				priv->fir.idx++;
				*pressure = priv->fir.sum[SPL_PRESSURE] / firlen;
				if (atomic_read(&priv->trace) &
					BAR_TRC_FILTER) {
					BAR_LOG("add [%2d][%5d]=>[%5d]:[%5d]\n",
						idx,
						priv->fir.raw[idx][SPL_PRESSURE],
						priv->fir.sum[SPL_PRESSURE],
						*pressure);
				}
			}
		}
	}
#endif

	mutex_unlock(&priv->lock);
	return err;
}

static int spl_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

static int spl_enable_nodata(int en)
{
	//struct spl_i2c_data *obj = i2c_get_clientdata(obj_i2c_data->client);
	int res = 0;
	int retry = 0;
	bool power = false;

	if (1 == en)
		power = true;

	if (0 == en)
		power = false;


	for (retry = 0; retry < 3; retry++) {
		res = spl_set_powermode(obj_i2c_data->client, (enum SPL_POWERMODE_ENUM)(!!power));
		if (res == 0) {
			BAR_LOG("spl_set_powermode done\n");
			break;
		}
		BAR_ERR("spl_set_powermode fail\n");
	}

	if (res != 0) {
		BAR_ERR("spl_set_powermode fail!\n");
		return -1;
	}
	BAR_LOG("spl_set_powermode OK!\n");
	return 0;
}

static int spl_set_delay(u64 ns)
{
	return 0;
}

static int spl_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return spl_set_delay(samplingPeriodNs);
}

static int spl_flush(void)
{
	return baro_flush_report();
}

static int spl_compute_meas_time(u32 *meastime)
{
	/* variable used to return communication result*/
	int err = 0;

	*meastime = (samplerate_list[obj_i2c_data->samplerate_t]
	 * meastime_list[obj_i2c_data->oversampling_t] 
	 + samplerate_list[obj_i2c_data->samplerate_p]
	 * meastime_list[obj_i2c_data->oversampling_p])/10;

	return err;
}

static int spl_get_data(int *value, int *status)
{
	char buff[SPL_BUFSIZE];
	int err = 0;

	err = spl_get_pressure(obj_i2c_data->client, buff, SPL_BUFSIZE);
	if (err) {
		BAR_ERR("get compensated pressure value failed, err = %d\n", err);
		return -1;
	}
	if (sscanf(buff, "%x", value) != 1)
		BAR_ERR("sscanf parsing fail\n");
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int spl_check_calib_param(struct spl_i2c_data *obj)
{
	struct spl06_calibration_data *cali = &(obj->spl06_cali);

	/* verify that not all calibration parameters are 0 */
	if (cali->c0 == 0 && cali->c1 == 0 && cali->c00 == 0
		&& cali->c10== 0 && cali->c01 == 0
		&& cali->c11 == 0 && cali->c20 == 0
		&& cali->c21 == 0 && cali->c30 == 0) {
		BAR_ERR("all calibration parameters are zero\n");
		return -2;
	}

	BAR_LOG("calibration parameters are OK\n");
	return 0;
}

static int spl_check_pt(struct spl_i2c_data *obj)
{
	int err = 0;
	unsigned int temperature;
	unsigned int pressure;
	u32 meas_time;
	char t[SPL_BUFSIZE] = "", p[SPL_BUFSIZE] = "";

	err = spl_set_powermode(obj->client, SPL_NORMAL_MODE);
	if (err < 0) {
		BAR_ERR("set power mode failed, err = %d\n", err);
		return -15;
	}

	mdelay(50);
	
	spl_compute_meas_time(&meas_time);
	if (meas_time > 1000) {
		BAR_ERR("measurment time is out of range:%dms\n",
			meas_time);
		return -1;
	}

	/* check ut and t */
	spl_get_temperature(obj->client, t, SPL_BUFSIZE);
	if (sscanf(t, "%x", &temperature) != 1)
		BAR_ERR("sscanf parsing fail\n");
	if (temperature <= -40 * 100 || temperature >= 85 * 100) {
		BAR_ERR("temperature value is out of range:%d*0.01degree\n",
			temperature);
		return -16;
	}

	/* check up and p */
	spl_get_pressure(obj->client, p, SPL_BUFSIZE);
	if (sscanf(p, "%x", &pressure) != 1)
		BAR_ERR("sscanf parsing fail\n");
	if (pressure <= 800 * 100 || pressure >= 1100 * 100) {
		BAR_ERR("pressure value is out of range:%d Pa\n",
			pressure);
		return -17;
	}

	BAR_LOG("spl06 temperature and pressure values are OK\n");
	return 0;
}

static int spl_do_selftest(struct spl_i2c_data *obj)
{
	int err = 0;
	/* 0: failed, 1: success */
	u8 selftest;

	err = spl_check_calib_param(obj);
	if (err) {
		selftest = 0;
		BAR_ERR("spl_check_calib_param:err=%d\n", err);
		goto exit;
	}

	err = spl_check_pt(obj);
	if (err) {
		selftest = 0;
		BAR_ERR("spl_check_pt:err=%d\n", err);
		goto exit;
	}

	/* selftest is OK */
	selftest = 1;
	BAR_LOG("spl06 self test is OK\n");
exit:
	return selftest;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct spl_i2c_data *obj = obj_i2c_data;

	if (NULL == obj) {
		BAR_ERR("spl i2c data pointer is null\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", obj->sensor_name);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct spl_i2c_data *obj = obj_i2c_data;
	char strbuf[SPL_BUFSIZE] = "";

	if (NULL == obj) {
		BAR_ERR("spl i2c data pointer is null\n");
		return 0;
	}

	spl_get_pressure(obj->client, strbuf, SPL_BUFSIZE);

	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct spl_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		BAR_ERR("spl i2c data pointer is null\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf,
	size_t count)
{
	struct spl_i2c_data *obj = obj_i2c_data;
	unsigned int trace;

	if (obj == NULL) {
		BAR_ERR("i2c_data obj is null\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		BAR_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct spl_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		BAR_ERR("spl i2c data pointer is null\n");
		return 0;
	}

	if (&obj->hw)
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
			obj->hw.i2c_num,
			obj->hw.direction,
			obj->hw.power_id,
			obj->hw.power_vol);
	else
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");

	len += snprintf(buf + len, PAGE_SIZE - len, "i2c addr:%#x,ver:%s\n",
		obj->client->addr, SPL_DRIVER_VERSION);

	return len;
}

static ssize_t show_power_mode_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct spl_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		BAR_ERR("spl i2c data pointer is null\n");
		return 0;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "%s mode\n",
		obj->power_mode == SPL_NORMAL_MODE ? "normal" : "suspend");

	return len;
}

static ssize_t store_power_mode_value(struct device_driver *ddri,
	const char *buf, size_t count)
{
	struct spl_i2c_data *obj = obj_i2c_data;
	unsigned long power_mode;
	int err;

	if (obj == NULL) {
		BAR_ERR("spl i2c data pointer is null\n");
		return 0;
	}

	err = kstrtoul(buf, 10, &power_mode);

	if (err == 0) {
		err = spl_set_powermode(obj->client,
			(enum SPL_POWERMODE_ENUM)(!!(power_mode)));
		if (err)
			return err;
		return count;
	}
	return err;
}

static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	struct spl_i2c_data *obj = obj_i2c_data;

	if (NULL == obj) {
		BAR_ERR("spl i2c data pointer is null\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", spl_do_selftest(obj));
}

static ssize_t show_cal_data(struct device_driver *ddri, char *buf)
{
	struct spl_i2c_data *obj = obj_i2c_data;

	if (NULL == obj) {
		BAR_ERR("spl i2c data pointer is null\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "c0:%d\n, c1:%d\n, c00:%d\n, c10:%d\n, c01:%d\n, c11:%d\n, c20:%d\n, c21:%d\n, c30:%d\n", 
		obj->spl06_cali.c0, obj->spl06_cali.c1, obj->spl06_cali.c00, obj->spl06_cali.c10,
		obj->spl06_cali.c01, obj->spl06_cali.c11, obj->spl06_cali.c20, obj->spl06_cali.c21, obj->spl06_cali.c30);
}

static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO,
	show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powermode, S_IWUSR | S_IRUGO,
	show_power_mode_value, store_power_mode_value);
static DRIVER_ATTR(selftest, S_IRUGO, show_selftest_value, NULL);
static DRIVER_ATTR(cal_data, S_IRUGO, show_cal_data, NULL);


static struct driver_attribute *spl_attr_list[] = {
	&driver_attr_chipinfo,	/* chip information*/
	&driver_attr_sensordata,/* dump sensor data*/
	&driver_attr_trace,	/* trace log*/
	&driver_attr_status,	/* cust setting */
	&driver_attr_powermode,	/* power mode */
	&driver_attr_selftest,	/* self test */
	&driver_attr_cal_data,	/* calibration data */
};

static int spl_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(spl_attr_list) / sizeof(spl_attr_list[0]));

	if (NULL == driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, spl_attr_list[idx]);
		if (err) {
			BAR_ERR("driver_create_file (%s) = %d\n",
				spl_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int spl_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(spl_attr_list) / sizeof(spl_attr_list[0]));

	if (NULL == driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, spl_attr_list[idx]);

	return err;
}

#if 0
#ifdef CONFIG_ID_TEMPERATURE
int temperature_operate(void *self, u32 command, void *buff_in,
	int size_in, void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	char buff[SPL_BUFSIZE];
	hwm_sensor_data *temperature_data;

	struct spl_i2c_data *priv = (struct spl_i2c_data *)self;

	switch (command) {
	case SENSOR_DELAY:
		/* under construction */
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			BAR_ERR("enable sensor parameter error\n");
			err = -EINVAL;
		} else {
			/* value:[0--->suspend, 1--->normal] */
			value = *(int *)buff_in;
			BAR_LOG("sensor enable/disable command: %s\n",
				value ? "enable" : "disable");

			err = spl_set_powermode(priv->client,
				(enum SPL_POWERMODE_ENUM)(!!value));
			if (err)
				BAR_ERR("set power mode failed, err = %d\n", err);
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) ||
			(size_out < sizeof(hwm_sensor_data))) {
			BAR_ERR("get sensor data parameter error\n");
			err = -EINVAL;
		} else {
			temperature_data = (hwm_sensor_data *)buff_out;
			err = spl_get_temperature(priv->client, buff, SPL_BUFSIZE);
			if (err) {
				BAR_ERR("get compensated temperature value failed,err = %d\n", err);
				return -1;
			}
			if (sscanf(buff, "%x", &temperature_data->values[0]) != 1)
				BAR_ERR("sscanf parsing fail\n");
			temperature_data->values[1] = temperature_data->values[2] = 0;
			temperature_data->status = SENSOR_STATUS_ACCURACY_HIGH;
			temperature_data->value_divide = 100;
		}
		break;

	default:
		BAR_ERR("temperature operate function no this parameter %d\n",
			command);
		err = -1;
		break;
	}

	return err;
}
#endif/* CONFIG_ID_TEMPERATURE */
#endif

static int spl_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct spl_i2c_data *obj;
	struct baro_control_path ctl = { 0 };
	struct baro_data_path data = { 0 };
#ifdef CONFIG_ID_TEMPERATURE
//	struct hwmsen_object sobj_t;
#endif
	int err = 0;

	BAR_FUN();
//	BAR_ERR(" start\n");

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	err = get_baro_dts_func(client->dev.of_node, &obj->hw);
	if (err < 0) {
		BAR_ERR("get cust_baro dts info fail\n");
		goto exit_init_client_failed;
	}

	obj_i2c_data = obj;
	obj->client = client;
	i2c_set_clientdata(client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	obj->power_mode = SPL_UNDEFINED_POWERMODE;
	obj->i32kP = 524288;
	obj->i32kT = 524288;
	obj->oversampling_p = SPL_UNDEFINED_OVERSAMPLING;
	obj->oversampling_t = SPL_UNDEFINED_OVERSAMPLING;
	obj->samplerate_p = SPL_UNDEFINED_SAMPLERATE;
	obj->samplerate_t = SPL_UNDEFINED_SAMPLERATE;	
	mutex_init(&obj->lock);

#ifdef CONFIG_SPL_LOWPASS
	if (obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);

	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);
#endif

	err = spl_init_client(client);
	if (err)
		goto exit_init_client_failed;

	err = misc_register(&spl_device);
	if (err) {
		BAR_ERR("misc device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}

	err = spl_create_attr(&(spl_init_info.platform_diver_addr->driver));
	if (err) {
		BAR_ERR("create attribute failed, err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.is_use_common_factory = false;
	ctl.open_report_data = spl_open_report_data;
	ctl.enable_nodata = spl_enable_nodata;
	ctl.set_delay = spl_set_delay;

	ctl.batch = spl_batch;
	ctl.flush = spl_flush;
	
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw.is_batch_supported;

	err = baro_register_control_path(&ctl);
	if (err) {
		BAR_ERR("register baro control path err\n");
		goto exit_hwmsen_attach_pressure_failed;
	}

	data.get_data = spl_get_data;
	data.vender_div = 100;
	err = baro_register_data_path(&data);
	if (err) {
		BAR_ERR("baro_register_data_path failed, err = %d\n", err);
		goto exit_hwmsen_attach_pressure_failed;
	}

#if 0
	err = batch_register_support_info(ID_PRESSURE, obj->hw->is_batch_supported, data.vender_div, 0);
	if (err) {
		BAR_ERR("register baro batch support err = %d\n", err);
		goto exit_hwmsen_attach_pressure_failed;
	}

	BAR_ERR(" 88\n");
#endif

#if 0
#ifdef CONFIG_ID_TEMPERATURE
	sobj_t.self = obj;
	sobj_t.polling = 1;
	sobj_t.sensor_operate = temperature_operate;
	err = hwmsen_attach(ID_TEMPRERATURE, &sobj_t);
	if (err) {
		BAR_ERR("hwmsen attach failed, err = %d\n", err);
		goto exit_hwmsen_attach_temperature_failed;
	}
#endif/* CONFIG_ID_TEMPERATURE */
#endif

	spl_init_flag = 0;
	BAR_LOG("%s: OK\n", __func__);
	return 0;

//#ifdef CONFIG_ID_TEMPERATURE
//exit_hwmsen_attach_temperature_failed:
//	hwmsen_detach(ID_PRESSURE);
//#endif/* CONFIG_ID_TEMPERATURE */
exit_hwmsen_attach_pressure_failed:
	spl_delete_attr(&(spl_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
	misc_deregister(&spl_device);
exit_misc_device_register_failed:
exit_init_client_failed:
	kfree(obj);
exit:
	BAR_ERR("err = %d\n", err);
	spl_init_flag = -1;
	return err;
}

static int spl_i2c_remove(struct i2c_client *client)
{
	int err = 0;

//	err = hwmsen_detach(ID_PRESSURE);
//	if (err)
//		BAR_ERR("hwmsen_detach ID_PRESSURE failed, err = %d\n", err);

#if 0
#ifdef CONFIG_ID_TEMPERATURE
	err = hwmsen_detach(ID_TEMPRERATURE);
	if (err)
		BAR_ERR("hwmsen_detach ID_TEMPRERATURE failed, err = %d\n",
			err);
#endif
#endif

	err = spl_delete_attr(&(spl_init_info.platform_diver_addr->driver));
	if (err)
		BAR_ERR("spl_delete_attr failed, err = %d\n", err);

	err = misc_deregister(&spl_device);
	if (err)
		BAR_ERR("misc_deregister failed, err = %d\n", err);

	obj_i2c_data = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static int spl_i2c_detect(struct i2c_client *client,
	struct i2c_board_info *info)
{
	strncpy(info->type, SPL_DEV_NAME, sizeof(info->type));
	return 0;
}

static int spl_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct spl_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	if (NULL == obj) {
		BAR_ERR("null pointer\n");
		return -EINVAL;
	}

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		BAR_FUN();

	if (msg.event == PM_EVENT_SUSPEND) {

		atomic_set(&obj->suspend, 1);
		err = spl_set_powermode(obj->client, SPL_SUSPEND_MODE);
		if (err) {
			BAR_ERR("spl set suspend mode failed, err = %d\n", err);
			return err;
		}
		spl_power(&obj->hw, 0);
	}
	return err;
}

static int spl_resume(struct i2c_client *client)
{
	struct spl_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	if (NULL == obj) {
		BAR_ERR("null pointer\n");
		return -EINVAL;
	}

	if (atomic_read(&obj->trace) & BAR_TRC_INFO)
		BAR_FUN();

	spl_power(&obj->hw, 1);

	err = spl_init_client(obj->client);
	if (err) {
		BAR_ERR("initialize client fail\n");
		return err;
	}

	err = spl_set_powermode(obj->client, SPL_NORMAL_MODE);
	if (err) {
		BAR_ERR("spl set normal mode failed, err = %d\n", err);
		return err;
	}

#ifdef CONFIG_SPL_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	atomic_set(&obj->suspend, 0);
	return 0;
}



static int  spl_local_init(void)
{
//	struct baro_hw *hw = get_cust_baro();

	BAR_ERR(" start\n");

	//spl_power(hw, 1);

	if (i2c_add_driver(&spl_i2c_driver)) {
		BAR_ERR("add driver error\n");
		return -1;
	}

	if (-1 == spl_init_flag) {
		BAR_ERR("spl_init_flag is -1\n");
		return -1;
	}

	BAR_ERR("end\n");

	return 0;
}

static int spl_remove(void)
{
//	struct baro_hw *hw = get_cust_baro();

	BAR_FUN();
//	spl_power(hw, 0);
	i2c_del_driver(&spl_i2c_driver);
	return 0;
}


static int __init spl_init(void)
{
//	const char *name = "mediatek,spl06";

	//hw = get_baro_dts_func(name, hw);
//	hw = get_cust_baro_hw();
//	if (!hw)
//		BAR_ERR("get cust_baro dts info fail\n");

	BAR_FUN();
#ifdef CONFIG_MTK_LEGACY
	i2c_register_board_info(hw->i2c_num, &spl_i2c_info, 1);
#endif
	baro_driver_add(&spl_init_info);

	BAR_ERR(" end\n");
	return 0;
}

static void __exit spl_exit(void)
{
	BAR_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(spl_init);
module_exit(spl_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SPL06 PRESSURE SENSOR DRIVER");
MODULE_AUTHOR("xuwei.yang@goertek.com");
