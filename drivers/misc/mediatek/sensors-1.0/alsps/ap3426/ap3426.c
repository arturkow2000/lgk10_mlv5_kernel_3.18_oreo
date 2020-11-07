/*
 * Author: yucong xiong <yucong.xion@mediatek.com>
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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/time.h>
#include <cust_alsps.h>
#include "ap3426.h"
#include <alsps.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

//#include <linux/atomic.h>

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define AP3426_DEV_NAME     "ap3426"
/*----------------------------------------------------------------------------*/
#define USES_ALS_WAKELOCK
#define AP3426_DBG 1
#define AP3426_ALS_AUTO_GAIN_EN 1
#define AP3426_PS_AUTO_CALI_EN	0
#define AP3426_LIGHT_RATIO_EN 	1
#define CALIBRATION_TO_FILE 1

#if defined(CALIBRATION_TO_FILE)
#include "../../sensor_cal/sensor_cal_file_io.h"
#endif

#define CALB_TIMES 3

#define APS_TAG                  "[AP3426] "

#if AP3426_DBG
#define APS_FUN(f)				printk(KERN_INFO APS_TAG "%s\n", __func__)
#define APS_ERR(fmt, args...)	printk(KERN_ERR APS_TAG "%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)	printk(KERN_INFO APS_TAG "%s %d:"fmt, __func__, __LINE__, ##args)
#define APS_DBG(fmt, args...)   printk(KERN_DEBUG APS_TAG "%s %d: "fmt, __func__, __LINE__, ##args) /* ((void)0) */
#else
#define APS_FUN(f)
#define APS_ERR(fmt, args...)
#define APS_LOG(fmt, args...)
#define APS_DBG(fmt, args...)
#endif

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

#define ALS_INT 0x01
#define PS_INT 0x02

int ps_offset = 200;
int ps_hysteresis = 100;
int ps_integrated_time = 0x05;
int als_threshold_high = 200;
int als_threshold_low = 100;
int als_light_ratio_flu = AP3426_ALS_LIGHT_RATIO_FLU;
int als_light_ratio_inc = AP3426_ALS_LIGHT_RATIO_INC;
/******************************************************************************
 * extern functions
*******************************************************************************/
static int ap3426_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ap3426_i2c_remove(struct i2c_client *client);
static int ap3426_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int ap3426_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ap3426_i2c_resume(struct i2c_client *client);

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ap3426_i2c_id[] = { {AP3426_DEV_NAME, 0}, {} };

static unsigned long long int_top_time;
/* Maintain alsps cust info here */
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
struct platform_device *alspsPltFmDev;

/* For alsp driver get cust info */
struct alsps_hw *get_cust_alsps(void)
{
	return &alsps_cust;
}

/*----------------------------------------------------------------------------*/
struct ap3426_priv {
	struct alsps_hw *hw;
	struct i2c_client *client;
	struct work_struct eint_work;

	/*misc */
	u16 als_modulus;
	atomic_t i2c_retry;
	atomic_t als_suspend;
	atomic_t als_debounce;	/*debounce time after enabling als */
	atomic_t als_deb_on;	/*indicates if the debounce is on */
	atomic_t als_auto_gain_deb;
#ifdef CONFIG_64BIT
	atomic64_t als_deb_end;	/*the jiffies representing the end of debounce */
    atomic64_t als_auto_gain_deb_end;
#else
	atomic_t als_deb_end;	/*the jiffies representing the end of debounce */
    atomic_t als_auto_gain_deb_end;
#endif
	atomic_t ps_mask;	/*mask ps: always return far away */
	atomic_t ps_debounce;	/*debounce time after enabling ps */
	atomic_t ps_deb_on;	/*indicates if the debounce is on */
#ifdef CONFIG_64BIT
	atomic64_t ps_deb_end;	/*the jiffies representing the end of debounce */
#else
	atomic_t ps_deb_end;	/*the jiffies representing the end of debounce */
#endif
	atomic_t ps_suspend;
	atomic_t trace;
	atomic_t init_done;
#if defined(CONFIG_FB)
	struct notifier_block	 fb_notif;
#endif
	struct device_node *irq_node;
	int irq;

	/*data */
	u16 als;
	u32 bef_lux;
	bool als_first_data_flag;
	bool als_auto_gain_trig;
	bool als_change_light_ratio_trig;
	int als_read_count;
	u32 als_current_ratio;
	u16 ps;
	/* u8                   _align; */
	u8  als_gain;
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[C_CUST_ALS_LEVEL - 1];
	u32 als_value[C_CUST_ALS_LEVEL];
	unsigned int ps_cali;
	int ps_last_cali_status;
	bool ps_first_data;
	bool ps_bef_enable;
	bool als_bef_enable;
	u8 system_first_trig_status;

	atomic_t als_cmd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_cmd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_high;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_low;	/*the cmd value can't be read, stored in ram */
	atomic_t als_thd_val_high;	/*the cmd value can't be read, stored in ram */
	atomic_t als_thd_val_low;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val;
	ulong enable;		/*enable mask */
	ulong pending_intr;	/*pending interrupt */
#ifdef USES_ALS_WAKELOCK
	unsigned int als_wake_delay;
	struct wake_lock als_wl;
#endif
};
/*----------------------------------------------------------------------------*/

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,ap3426"},
	{},
};
#endif

static struct i2c_driver ap3426_i2c_driver = {
	.probe = ap3426_i2c_probe,
	.remove = ap3426_i2c_remove,
	.detect = ap3426_i2c_detect,
	.suspend = ap3426_i2c_suspend,
	.resume = ap3426_i2c_resume,
	.id_table = ap3426_i2c_id,
	.driver = {
		   .name = AP3426_DEV_NAME,
#ifdef CONFIG_OF
		   .of_match_table = alsps_of_match,
#endif
		   },
};

/*----------------------------------------------------------------------------*/
struct PS_CALI_DATA_STRUCT {
	int close;
	int far_away;
	int valid;
};

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static struct i2c_client *ap3426_i2c_client;
static struct ap3426_priv *ap3426_obj;
static int intr_flag = 1;	/* hw default away after enable. */
static u32 als_gain_ration[4] = {AP3426_ALS_GAIN_A_RATION, AP3426_ALS_GAIN_B_RATION,
	                             AP3426_ALS_GAIN_C_RATION, AP3426_ALS_GAIN_D_RATION};

static int ap3426_local_init(void);
static int ap3426_remove(void);
static int ap3426_do_calibration(struct i2c_client *client);
static int set_alssensor_threshold(struct i2c_client * client, int thd_low, int thd_high);
static int ap3426_init_flag = -1;	/* 0<==>OK -1 <==> fail */
static struct alsps_init_info ap3426_init_info = {
	.name = "ap3426",
	.init = ap3426_local_init,
	.uninit = ap3426_remove,
};

/*----------------------------------------------------------------------------*/
static DEFINE_MUTEX(ap3426_mutex);
/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS = 1,
	CMC_BIT_PS = 2,
} CMC_BIT;
/*-----------------------------CMC for debugging-------------------------------*/
typedef enum {
	CMC_TRC_ALS_DATA = 0x0001,
	CMC_TRC_PS_DATA = 0x0002,
	CMC_TRC_EINT = 0x0004,
	CMC_TRC_IOCTL = 0x0008,
	CMC_TRC_I2C = 0x0010,
	CMC_TRC_CVT_ALS = 0x0020,
	CMC_TRC_CVT_PS = 0x0040,
	CMC_TRC_CVT_AAL = 0x0080,
	CMC_TRC_DEBUG = 0x8000,
} CMC_TRC;
/*-----------------------------------------------------------------------------*/

int AP3426_i2c_master_operate(struct i2c_client *client, char *buf, int count, int i2c_flag)
{
	int res = 0;

	mutex_lock(&ap3426_mutex);
	switch (i2c_flag) {
	case I2C_FLAG_WRITE:
		res = i2c_master_send(client, buf, count);
		if (res <= 0) {
		    APS_ERR("ap3426_write_reg res=%d\n", res);
		    goto EXIT_ERR;
	    }
		break;
	case I2C_FLAG_READ:
		res = i2c_master_send(client, buf, count);
		if (res <= 0) {
		    APS_ERR("ap3426_read_reg 1 res=%x\n", res);
		    goto EXIT_ERR;
	    }
		res = i2c_master_recv(client, buf, count);
		if (res <= 0) {
		    APS_ERR("ap3426_read_reg 2 res=%d\n", res);
		    goto EXIT_ERR;
	    }
		break;
	default:
		APS_LOG("AP3426_i2c_master_operate i2c_flag command not support!\n");
		break;
	}

	if (res < 0)
		goto EXIT_ERR;

	mutex_unlock(&ap3426_mutex);
	return res;
EXIT_ERR:
	mutex_unlock(&ap3426_mutex);
	APS_ERR("AP3426_i2c_master_operate fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static void ap3426_power(struct alsps_hw *hw, unsigned int on)
{
}

/********************************************************************/
int ap3426_als_enable_ps(struct i2c_client *client, int enable)
{
	struct ap3426_priv *obj = i2c_get_clientdata(client);
	int res = 1;
	u8 databuf[3] = {'\0'};

	if (enable == 1) {
		APS_DBG("ap3426_als_enable_ps enable_ps\n");
		databuf[0] = AP3426_ENABLE;
		res = AP3426_i2c_master_operate(client, databuf, 0x01, I2C_FLAG_READ);
		if (res < 0) {
			APS_ERR("read AP3426_ENABLE err: %d\n", databuf[0]);
			goto ENABLE_PS_EXIT_ERR;
		}

		APS_DBG("AP3426_ENABLE read value = 0x%02x\n", databuf[0]);
		databuf[1] = databuf[0] | 0x02;
		databuf[0] = AP3426_ENABLE;

		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
		if (res < 0) {
			APS_ERR("write AP3426_ENABLE err: %d\n", res);
			goto ENABLE_PS_EXIT_ERR;
		}

	} else {
		APS_DBG("ap3426_enable_ps disable_ps\n");
		databuf[0] = AP3426_ENABLE;
		res = AP3426_i2c_master_operate(client, databuf, 0x01, I2C_FLAG_READ);
		if (res < 0) {
			APS_ERR("read AP3426_ENABLE err: %d\n", res);
			goto ENABLE_PS_EXIT_ERR;
		}

		APS_LOG("AP3426_ENABLE value = 0x%02x\n", databuf[0]);
		databuf[1] = databuf[0] & 0xFD;
		databuf[0] = AP3426_ENABLE;
		res = AP3426_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res < 0) {
			APS_ERR("write AP3426_ENABLE err: %d\n", res);
			goto ENABLE_PS_EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);

	}

	return 0;
ENABLE_PS_EXIT_ERR:
	return res;
}


int ap3426_set_waiting_time(struct i2c_client *client, u8 waiting_time)
{
	int res=1;
	u8 databuf[3] = {'\0'};
	//struct ap3426_priv *obj = i2c_get_clientdata(client);

	databuf[1] = waiting_time;
	databuf[0] = AP3426_WAITING_TIME;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
        if(res < 0){
		APS_ERR("write AP3426_WAITING_TIME err: %d\n", res);
			return res;
	}
	return 0;
}

int ap3426_set_persist(struct i2c_client *client, u8 persist)
{
	int res=1;
	u8 databuf[3] = {'\0'};

	databuf[1] = persist;
	databuf[0] = AP3426_INT_PS_PRSIST;
	APS_LOG("write AP3426_SET_PERSIST : %d\n", persist);
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
        if(res < 0){
		APS_ERR("write AP3426_SET_PERSIST err: %d\n", res);
			return res;
	}
	return 0;
}

int ap3426_clr_int_flag(struct i2c_client *client)
{
	//struct ap3426_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3] = {'\0'};
	int res = 1;

	databuf[1] = 0x00;
	databuf[0] = AP3426_INT_STATUS;

	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res < 0) {
		APS_ERR("write AP3426_INT_STATUS err: %d\n", res);
		return res;
	}

	databuf[0] = AP3426_PDATA_L;
	res = AP3426_i2c_master_operate(client, databuf, 0x01, I2C_FLAG_READ);
	if (res < 0) {
		APS_ERR("read AP3426_PDATA_L err: %d\n", databuf[0]);
		return res;
	}

	return 0;
}

/********************************************************************/
int ap3426_enable_ps(struct i2c_client *client, int enable)
{
	struct ap3426_priv *obj = i2c_get_clientdata(client);
	int res = 1;
	u8 databuf[3] = {'\0'};
	u8 databuf1[2] = {'\0'};

	if (enable == 1) {
		APS_LOG("ap3426_enable_ps enable_ps\n");
		databuf[0] = AP3426_ENABLE;
		res = AP3426_i2c_master_operate(client, databuf, 0x01, I2C_FLAG_READ);
		if (res < 0) {
			APS_ERR("read AP3426_ENABLE err: %d\n", databuf[0]);
			goto ENABLE_PS_EXIT_ERR;
		}

	/*	Change waiting time : */

		databuf1[0] = AP3426_WAITING_TIME;

		if (obj->als_bef_enable){

			res = ap3426_set_waiting_time(client, AP3426_ALSPS_WAITING_TIME);  // 90 ms
			if (res<0)	{
				APS_ERR("set waiting time err: %d\n", res);
				goto ENABLE_PS_EXIT_ERR;
			}
		}
		else
		{
			res = ap3426_set_waiting_time(client, AP3426_PS_ONLY_WAITING_TIME); //140 ms
			if (res<0)	{
				APS_ERR("set waiting time err: %d\n", res);
				goto ENABLE_PS_EXIT_ERR;
			}
		}
		obj->ps_first_data = true;
		res = ap3426_set_persist(client, AP3426_PERSIST_EVERTIME);
		if (res<0)	{
			APS_ERR("set persist err: %d\n", res);
			goto ENABLE_PS_EXIT_ERR;
		}

		APS_LOG("AP3426_ENABLE read value = 0x%02x\n", databuf[0]);
		databuf[1] = databuf[0] | 0x02;
		databuf[0] = AP3426_ENABLE;

		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
		if (res < 0) {
			APS_ERR("write AP3426_ENABLE err: %d\n", res);
			goto ENABLE_PS_EXIT_ERR;
		}

		obj->ps_bef_enable =true; // Change PXY befor status is true
		intr_flag = 1;	/* reset hw status to away after enable. */
		msleep(3);
	} else {
		APS_LOG("ap3426_enable_ps disable_ps\n");
		databuf[0] = AP3426_ENABLE;
		res = AP3426_i2c_master_operate(client, databuf, 0x01, I2C_FLAG_READ);
		if (res < 0) {
			APS_ERR("read AP3426_ENABLE err: %d\n", res);
			goto ENABLE_PS_EXIT_ERR;
		}

		databuf1[0]=AP3426_WAITING_TIME;
        databuf1[1]=AP3426_ALSPS_WAITING_TIME; // 90ms
        res = AP3426_i2c_master_operate(client, databuf1, 0x02, I2C_FLAG_WRITE);
            if(res < 0){
				APS_ERR("write AP3426_WAITING_TIME err: %d\n", res);
                goto ENABLE_PS_EXIT_ERR;
            }

		APS_LOG("AP3426_ENABLE value = 0x%02x\n", databuf[0]);
		databuf[1] = databuf[0] & 0xFD;
		databuf[0] = AP3426_ENABLE;
		res = AP3426_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res < 0) {
			APS_ERR("write AP3426_ENABLE err: %d\n", res);
			goto ENABLE_PS_EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);

		/*------------Clear Interrupt flag----------------*/
		res = ap3426_clr_int_flag(client);
		if (res < 0) {
			APS_ERR("clear interrupt flag err: %d\n", res);
			goto ENABLE_PS_EXIT_ERR;
		}
		/*-------------------------------------------*/

		obj->ps_bef_enable=false; // Change PXY befor status is true
	}

	return 0;
ENABLE_PS_EXIT_ERR:
	return res;
}

/********************************************************************/
int ap3426_enable_als(struct i2c_client *client, int enable)
{
	struct ap3426_priv *obj = i2c_get_clientdata(client);
	int res = 1;
/*	int value, status = 0; */
	u8 databuf[3] = {'\0'};

	if (enable == 1) {
		APS_LOG("ap3426_enable_als enable_als\n");
		obj->als_auto_gain_trig = false;
		obj->system_first_trig_status = SYSTEM_TRIG_FIRST;
		obj->als_read_count = 0;
		/*------------Waiting time: 0 Gain: 64 ----------------*/
		/* System Waiting time*/
		databuf[1] = 0x0; 	//0ms
		databuf[0] = AP3426_WAITING_TIME;
		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
		if (res <= 0) {
			APS_ERR("write System Waiting Time err: %d\n", res);
			goto ENABLE_ALS_EXIT_ERR;
		}

		/* ALS gain */
		databuf[0] = 0x10;
		databuf[1] = 0x30;  	// 64x gain
		obj->als_gain = databuf[1];
		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
		if (res <= 0) {
			APS_ERR("write als gain err: %d\n", res);
			goto ENABLE_ALS_EXIT_ERR;
		}

		/*------------------------------------------------*/

		databuf[0] = AP3426_ENABLE;
		res = AP3426_i2c_master_operate(client, databuf, 0x01, I2C_FLAG_READ);
		if (res < 0) {
			APS_ERR("read AP3426_ENABLE err:%d\n", res);
			goto ENABLE_ALS_EXIT_ERR;
		}

		APS_LOG("AP3426_ENABLE value=0x%02x\n", databuf[0]);

		databuf[1] = databuf[0] | 0x01;
		databuf[0] = AP3426_ENABLE;
		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
		if (res < 0) {
			APS_ERR("write AP3426_ENABLE err:%d\n", res);
			goto ENABLE_ALS_EXIT_ERR;
		}
		obj->als_first_data_flag = true;
		obj->als_bef_enable=true;
        msleep(3);
/* If you want to update fastest data of ALS after ALS enable, use this. */
#if(0)
		if(obj->system_first_trig_status == SYSTEM_TRIG_FIRST) {
			msleep(100);
			als_get_data(&value, &status);
			als_data_report(value, status);
			obj->system_first_trig_status = SYSTEM_TRIG_SECOND;
		}
#endif
	} else {
		APS_LOG("ap3426_enable_als disable_als\n");

		/* System Waiting time*/
		if(obj->ps_bef_enable)
			databuf[1] = AP3426_PS_ONLY_WAITING_TIME; 	//140ms
		else
			databuf[1] = AP3426_ALSPS_WAITING_TIME;	// 90ms

		databuf[0] = AP3426_WAITING_TIME;
		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
		if (res <= 0) {
			APS_ERR("write System Waiting Time err: %d\n", res);
			goto ENABLE_ALS_EXIT_ERR;
		}

		databuf[0] = AP3426_ENABLE;
		res = AP3426_i2c_master_operate(client, databuf, 0x01, I2C_FLAG_READ);
		if (res < 0) {
			APS_ERR("read AP3426_ENABLE err:%d\n", res);
			goto ENABLE_ALS_EXIT_ERR;
		}

		APS_LOG("AP3426_ENABLE value value_low=0x%02x\n", databuf[0]);
		databuf[1] = databuf[0] & 0xFE;
		databuf[0] = AP3426_ENABLE;
		res = AP3426_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res < 0) {
			APS_ERR("write AP3426_ENABLE err:%d\n", res);
			goto ENABLE_ALS_EXIT_ERR;
		}
		atomic_set(&obj->als_deb_on, 0);
		obj->als_bef_enable=false;
	}
	return 0;
ENABLE_ALS_EXIT_ERR:
	return res;
}

/********************************************************************/
#if AP3426_LIGHT_RATIO_EN
/* Add get IR data function for light ratio function - Kaku 20171109*/
static int ap3426_get_ir_data(struct i2c_client *client)
{
	u16 ir_value = 0;
	u8 lsb = 0, msb = 0;
	u8 databuf[3] = {'\0'};
	int res = 0;

	databuf[0] = AP3426_IRDATA_L;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_READ);
	if (res < 0) {
		APS_ERR("read als raw data err: %d\n", res);
		return res;
	}

	lsb = databuf[0];
	msb = databuf[1];
	ir_value =( (msb&0x03)<<8) | lsb;

	APS_DBG("AP3426_IR_DATA: %d",ir_value);

	return ir_value;
}
#endif
/********************************************************************/

long ap3426_read_ps(struct i2c_client *client, u16 *data)
{
	int res = 1;
	u8 databuf[3] = {'\0'};
	struct ap3426_priv *obj = i2c_get_clientdata(client);

	databuf[0] = AP3426_PDATA_L;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_READ);
	if (res < 0) {
		APS_ERR("read ps raw data err:%d\n", res);
		goto READ_PS_EXIT_ERR;
	}

	if (atomic_read(&obj->trace) & CMC_TRC_DEBUG) {
		APS_LOG("AP3426_REG_PS_DATA value value_low=%x, value_high=%x\n", databuf[0],
			databuf[1] & 0x03);
	}

	*data = (((databuf[1] & 0x03) << 8) | databuf[0]);

	return 0;
READ_PS_EXIT_ERR:
	return res;
}

bool ap3426_als_auto_gain(struct i2c_client *client, u16 *alsrawdata)
{
    struct ap3426_priv *obj = i2c_get_clientdata(client);
    u8 databuf[3] = {'\0'};
    int res = 0;

	databuf[0] = AP3426_ALS_GAIN;
	res = AP3426_i2c_master_operate(client, databuf, 0x01, I2C_FLAG_READ);
	if (res < 0) {
		APS_ERR("read AP3426_ALS_GAIN err:%d\n", res);
		ap3426_set_waiting_time(client, AP3426_ALSPS_WAITING_TIME);
		return false;
	}
    obj->als_gain = databuf[0];
//	APS_ERR("current gain:0x%02x\n", obj->als_gain);
#if 0
	/*------------ Reduce auto gain waiting time: Lux Low to High --------------*/

	if((*alsrawdata == AP3426_ALS_MAX_DATA) && (obj->als_gain > AP3426_ALS_GAIN_A)){

#ifdef CONFIG_64BIT
		atomic64_set(&obj->als_auto_gain_deb_end, jiffies + atomic_read(&obj->als_auto_gain_deb) / (1000 / HZ));
#else
		atomic_set(&obj->als_auto_gain_deb_end, jiffies + atomic_read(&obj->als_auto_gain_deb) / (1000 / HZ));
#endif
		APS_DBG("als raw data:%d, als_auto_gain_hthres:%d\n", *alsrawdata, AP3426_ALS_MAX_DATA);
		obj->als_gain = AP3426_ALS_GAIN_A;  //Change gain x64 -> x1
		APS_DBG("next als gain:0x%02x\n", obj->als_gain);
		databuf[1] = obj->als_gain;
		databuf[0] = AP3426_ALS_GAIN;
		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	    if (res < 0) {
		    APS_ERR("write AP3426_ALS_GAIN err:%d\n", res);
			ap3426_set_waiting_time(client, AP3426_ALSPS_WAITING_TIME);
			return false;
		}
		ap3426_set_waiting_time(client, 0x00);
		return true;
	}
	/*------------ Reduce auto gain waiting time: Lux High to Low --------------*/
	if((*alsrawdata < AP3426_ALS_AUTO_GAIN_LTHRES_L) && (obj->als_gain < AP3426_ALS_GAIN_D)){
#ifdef CONFIG_64BIT
		atomic64_set(&obj->als_auto_gain_deb_end, jiffies + atomic_read(&obj->als_auto_gain_deb) / (1000 / HZ));
#else
        atomic_set(&obj->als_auto_gain_deb_end, jiffies + atomic_read(&obj->als_auto_gain_deb) / (1000 / HZ));
#endif
		APS_DBG("als raw data:%d, als_auto_gain_hthres:%d\n", *alsrawdata, AP3426_ALS_MAX_DATA);
		obj->als_gain = AP3426_ALS_GAIN_D;
		APS_DBG("next als gain:0x%02x\n", obj->als_gain);
		databuf[1] = obj->als_gain;
		databuf[0] = AP3426_ALS_GAIN;
		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	    if (res < 0) {
		    APS_ERR("write AP3426_ALS_GAIN err:%d\n", res);
		    ap3426_set_waiting_time(client, AP3426_ALSPS_WAITING_TIME);
		    return false;
	    }
		ap3426_set_waiting_time(client, 0x00);
		return true;
	}

	/*----------------------------------------------------------------------------------*/

#endif
	if ((*alsrawdata >= AP3426_ALS_AUTO_GAIN_HTHRES) && (obj->als_gain > 0x00)) {
/*
#ifdef CONFIG_64BIT
	atomic64_set(&obj->als_auto_gain_deb_end, jiffies + atomic_read(&obj->als_auto_gain_deb) / (1000 / HZ));
#else
        atomic_set(&obj->als_auto_gain_deb_end, jiffies + atomic_read(&obj->als_auto_gain_deb) / (1000 / HZ));
#endif
*/
        APS_DBG("als raw data:%d, als_auto_gain_hthres:%d\n", *alsrawdata, AP3426_ALS_AUTO_GAIN_HTHRES);
		obj->als_gain = obj->als_gain - 0x10;
		APS_DBG("next als gain:0x%02x\n", obj->als_gain);
		databuf[1] = obj->als_gain;
		databuf[0] = AP3426_ALS_GAIN;
		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	    if (res < 0) {
		    APS_ERR("write AP3426_ALS_GAIN err:%d\n", res);
		    ap3426_set_waiting_time(client, AP3426_ALSPS_WAITING_TIME);
		    return false;
	    }
	} else if ((*alsrawdata <= AP3426_ALS_AUTO_GAIN_LTHRES) && (obj->als_gain < 0x30)) {
/*
#ifdef CONFIG_64BIT
		atomic64_set(&obj->als_auto_gain_deb_end, jiffies + atomic_read(&obj->als_auto_gain_deb) / (1000 / HZ));
#else
        atomic_set(&obj->als_auto_gain_deb_end, jiffies + atomic_read(&obj->als_auto_gain_deb) / (1000 / HZ));
#endif
*/
        APS_DBG("als raw data:%d, als_auto_gain_lthres:%d\n", *alsrawdata, AP3426_ALS_AUTO_GAIN_LTHRES);
		obj->als_gain = obj->als_gain + 0x10;
		APS_DBG("next als gain:0x%02x\n", obj->als_gain);
		databuf[1] = obj->als_gain;
		databuf[0] = AP3426_ALS_GAIN;
		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	    if (res < 0) {
		    APS_ERR("write AP3426_ALS_GAIN err:%d\n", res);
		    ap3426_set_waiting_time(client, AP3426_ALSPS_WAITING_TIME);
			return false;
		}
	} else {
	ap3426_set_waiting_time(client, AP3426_ALSPS_WAITING_TIME);
		return false;
	}
    ap3426_set_waiting_time(client, 0x00);
    return true;
}

/********************************************************************/
long ap3426_read_als(struct i2c_client *client, u16 *data)
{
    struct ap3426_priv *obj = i2c_get_clientdata(client);
	int res = 1;
	u8 databuf[3] = {'\0'};

#if AP3426_ALS_AUTO_GAIN_EN
//    unsigned long endt = atomic_read(&obj->als_auto_gain_deb_end);
//    APS_DBG("endt:%lu, als_auto_gain_trig:%d\n", endt, obj->als_auto_gain_trig);
/*
    if (obj->als_auto_gain_trig) {
	    if (time_before(jiffies, endt)){
            APS_DBG("als auto gain deb is not expire\n");
			return 0;
	    }
    }
*/
#endif

	databuf[0] = AP3426_ADATA_L;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_READ);
	if (res < 0) {
		APS_ERR("read als raw data err: %d\n", res);
		goto READ_ALS_EXIT_ERR;
	}

	*data = ((databuf[1] << 8) | databuf[0]);

//	APS_DBG("als raw data: %d\n", *data);

	if(obj->als_first_data_flag){
		/* System Waiting time*/
		if(obj->ps_bef_enable) {
			databuf[1] = AP3426_ALSPS_WAITING_TIME;  //90ms
			databuf[0] = AP3426_WAITING_TIME;
			res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
			if (res <= 0) {
			APS_ERR("Write System Waiting Time err: %d\n", res);
			goto READ_ALS_EXIT_ERR;
			}
		}

		obj->als_first_data_flag=false;
	}

#if AP3426_ALS_AUTO_GAIN_EN
	obj->als_auto_gain_trig = ap3426_als_auto_gain(client, data);
#endif

	if (atomic_read(&obj->trace) & CMC_TRC_DEBUG)
		APS_LOG("als raw data:%d\n", *data);

	return 0;
READ_ALS_EXIT_ERR:
	return res;
}

/********************************************************************/
static int ap3426_get_ps_value(struct ap3426_priv *obj, u16 ps)
{
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;

	val = intr_flag;	/* value between high/low threshold should sync. with hw status. */

	if (ps > atomic_read(&obj->ps_thd_val_high))
		val = 0;	/*close */
	else if (ps < atomic_read(&obj->ps_thd_val_low))
		val = 1;	/*far away */

	if (atomic_read(&obj->ps_suspend)) {
		invalid = 1;
	} else if (1 == atomic_read(&obj->ps_deb_on)) {
#ifdef CONFIG_64BIT
		unsigned long endt = atomic64_read(&obj->ps_deb_end);
#else
		unsigned long endt = atomic_read(&obj->ps_deb_end);
#endif
		if (time_after(jiffies, endt))
			atomic_set(&obj->ps_deb_on, 0);

		if (1 == atomic_read(&obj->ps_deb_on))
			invalid = 1;
	}

	if (!invalid) {
		if (unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS)) {
			if (mask)
				APS_DBG("PS:  %05d => %05d [M]\n", ps, val);
			else
				APS_DBG("PS:  %05d => %05d\n", ps, val);
		}
		if (0 == test_bit(CMC_BIT_PS, &obj->enable)) {
			/* if ps is disable do not report value */
			APS_DBG("PS: not enable and do not report this value\n");
			return -1;
		} else {
			return val;
		}

	} else {
		if (unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);
		return -1;
	}
}

/********************************************************************/
static int ap3426_get_als_value(struct ap3426_priv *obj, u16 als)
{
	u32 lux = 0;
#if AP3426_LIGHT_RATIO_EN
	u32 ir_value =0;
#endif
#if AP3426_ALS_AUTO_GAIN_EN
//    if (obj->als_auto_gain_trig || (obj->system_first_trig_status == SYSTEM_TRIG_SECOND)) {
	if (obj->als_auto_gain_trig) {
		if (obj->als_auto_gain_trig == 1) {
			APS_DBG("before lux:%d, als_auto_gain_trig:%d\n", obj->bef_lux, obj->als_auto_gain_trig);
		}
		//obj->system_first_trig_status = SYSTEM_TRIG_FIRST;
        //return obj->bef_lux;
        return -1;
    }
#endif

	lux = (als_gain_ration[(obj->als_gain >> 4)] * als);
	//obj->system_first_trig_status = SYSTEM_TRIG_SECOND;

#if AP3426_LIGHT_RATIO_EN
/*------------------------------------------------------------------*/
	if ((obj->als_read_count == AP3426_ALS_READ_COUNT_EN_PS)){
			ap3426_als_enable_ps(obj->client, 1); //Before check light ratio, enable PS first(AP3426_ALS_READ_COUNT_EN_PS)(Reduce delay time)
	}

	if (obj->als_read_count >= AP3426_ALS_READ_COUNT_MAX)  // Check light ratio 1 time after read ALS data 10 times
	{
		if (!obj->ps_bef_enable){
			ir_value = ap3426_get_ir_data(obj->client);
			ap3426_als_enable_ps(obj->client, 0); //Disable PXY
		}
		else
		{
			ir_value = ap3426_get_ir_data(obj->client);
			APS_DBG("IR_DATA :%d\n", ir_value);
		}

		if(lux==0){

				if(ir_value>0)
					obj->als_current_ratio = als_light_ratio_inc;
				else
					obj->als_current_ratio = als_light_ratio_flu;
			}
		else
		{
				if ((ir_value >= AP3426_IR_DATA_MAX) || ((ir_value*AP3426_ALS_LIGHT_RATIO_R_FX) > (AP3426_ALS_LIGHT_RATIO_R*lux))) {	// ir_value : lux = 0.039 or Ir_value is MAX

					obj->als_current_ratio = als_light_ratio_inc;  	// The light ratio Incandescent
				}
				else
					obj->als_current_ratio = als_light_ratio_flu;  	// The light ratio for Fluorescent
		}
		APS_DBG("als_current_ratio : %d", obj->als_current_ratio);
		obj->als_read_count = 0;
	}
	else
	{
		obj->als_read_count++;
	}

	lux = (lux * obj->als_current_ratio)/AP3426_ALS_LIGHT_RATIO_FX;
	/*---------------------------------------------------*/

#else
	lux = ((lux * AP3426_ALS_LIGHT_RATIO_NON)/AP3426_ALS_LIGHT_RATIO_FX);  // The coefficient is for non-Light ratio function
#endif
	lux = lux/AP3426_FX;

#if AP3426_ALS_AUTO_GAIN_EN
    obj->bef_lux = lux;
#endif

    APS_DBG("lux:%d, als raw data:%d, als gain ratio:%d, als_gain:0x%02x\n", lux, als, als_gain_ration[(obj->als_gain >> 4)], obj->als_gain);
	return lux;
}

/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t ap3426_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n",
		       atomic_read(&ap3426_obj->i2c_retry),
		       atomic_read(&ap3426_obj->als_debounce), atomic_read(&ap3426_obj->ps_mask),
		       atomic_read(&ap3426_obj->ps_thd_val),
		       atomic_read(&ap3426_obj->ps_debounce));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}

	if (5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb)) {
		atomic_set(&ap3426_obj->i2c_retry, retry);
		atomic_set(&ap3426_obj->als_debounce, als_deb);
		atomic_set(&ap3426_obj->ps_mask, mask);
		atomic_set(&ap3426_obj->ps_thd_val, thres);
		atomic_set(&ap3426_obj->ps_debounce, ps_deb);
	} else {
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ap3426_obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int trace;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&ap3426_obj->trace, trace);
	else
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_show_als(struct device_driver *ddri, char *buf)
{
	int res;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}
	res = ap3426_read_als(ap3426_obj->client, &ap3426_obj->als);
	if (res)
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	else
		return snprintf(buf, PAGE_SIZE, "0x%04x\n", ap3426_obj->als);
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	u8 databuf[3] = {'\0'};

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}

	databuf[0] = AP3426_ENABLE;
	res = AP3426_i2c_master_operate(ap3426_obj->client, databuf, 1, I2C_FLAG_READ);
	if (res <= 0) {
		APS_ERR("write AP3426_ENABLE err: %d\n", (int)res);
		return -1;
	}
	if (!(databuf[0] & 0x02))
		return snprintf(buf, PAGE_SIZE, "Read AP3426_EN value: 0x%02x, Please enable ps first\n", databuf[0]);

	res = ap3426_read_ps(ap3426_obj->client, &ap3426_obj->ps);
	if (res)
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", (int)res);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", ap3426_obj->ps);
}

static ssize_t ap3426_show_reg(struct device_driver *ddri, char *buf)
{
	u16 idx = 0;
	int count = 0;
	int res = 1;
	u8 databuf[2] = { 0 };

	u8 ap3426_reg[] = {
		0x00, 0x01, 0x02, 0x06, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
		0x10, 0x14, 0x1A, 0x1B, 0x1C, 0x1D,
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};

	u8 ap3426_NUM_CACHABLE_REGS = sizeof(ap3426_reg)/sizeof(ap3426_reg[0]);

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		count += sprintf(buf+count, "ap3426_obj is null!!\n");
		return count;
	}
	for (idx = 0; idx < ap3426_NUM_CACHABLE_REGS; idx++) {
		databuf[0] = ap3426_reg[idx];
		res = AP3426_i2c_master_operate(ap3426_obj->client, databuf, 0x001, I2C_FLAG_READ);
		if (res < 0) {
			count += snprintf(buf+count, PAGE_SIZE - count, "i2c read_reg err\n");
			return count;
		}
		count += snprintf(buf+count, PAGE_SIZE - count, "[0x%02x]=0x%02x\n", ap3426_reg[idx], databuf[0]);
	}

	return count;
}

static ssize_t ap3426_store_reg(struct device_driver *ddri, const char *buf, size_t count)
{

	unsigned int addr, val;
	int ret = 0;
	int res;
	u8 databuf[2] = { 0 };
	char msg[30] = {'\n'};

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return -1;
	}

	ret = sscanf(buf, "%x %x", &addr, &val);

	APS_LOG("Reg[%x].Write [%x]..\n", addr, val);
	databuf[0] = addr;
	databuf[1] = val;
	res = AP3426_i2c_master_operate(ap3426_obj->client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res < 0) {
		count += snprintf(msg+count, PAGE_SIZE - count, "i2c store reg err: %d\n", res);
		return count;
	}

	return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t ap3426_show_send(struct device_driver *ddri, char *buf)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int addr, cmd;
	u8 dat;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	} else if (2 != sscanf(buf, "%x %x", &addr, &cmd)) {
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8) cmd;

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_show_recv(struct device_driver *ddri, char *buf)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	int ret;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}
	ret = kstrtoint(buf, 16, &addr);
	if (ret < 0) {
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}

	if (ap3426_obj->hw) {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d, (%d %d)\n",
				ap3426_obj->hw->i2c_num, ap3426_obj->hw->power_id,
				ap3426_obj->hw->power_vol);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "REGS: %02X %02X %02X %02lX %02lX\n",
			atomic_read(&ap3426_obj->als_cmd_val),
			atomic_read(&ap3426_obj->ps_cmd_val),
			atomic_read(&ap3426_obj->ps_thd_val), ap3426_obj->enable,
			ap3426_obj->pending_intr);

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "MISC: %d %d\n",
		     atomic_read(&ap3426_obj->als_suspend), atomic_read(&ap3426_obj->ps_suspend));

	return len;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ap3426_priv *obj, const char *buf, size_t count, u32 data[],
			     int len)
{
	int idx = 0;
	int ret;
	char *cur = (char *)buf, *end = (char *)(buf + count);

	while (idx < len) {
		while ((cur < end) && IS_SPACE(*cur))
			cur++;

		ret = kstrtoint(cur, 10, &data[idx]);
		if (ret < 0)
			break;

		idx++;
		while ((cur < end) && !IS_SPACE(*cur))
			cur++;
	}
	return idx;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}

	for (idx = 0; idx < ap3426_obj->als_level_num; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%d ", ap3426_obj->hw->als_level[idx]);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	} else if (!strcmp(buf, "def")) {
		memcpy(ap3426_obj->als_level, ap3426_obj->hw->als_level,
		       sizeof(ap3426_obj->als_level));
	} else if (ap3426_obj->als_level_num !=
		   read_int_from_buf(ap3426_obj, buf, count, ap3426_obj->hw->als_level,
				     ap3426_obj->als_level_num)) {
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}

	for (idx = 0; idx < ap3426_obj->als_value_num; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%d ", ap3426_obj->hw->als_value[idx]);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t ap3426_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	} else if (!strcmp(buf, "def")) {
		memcpy(ap3426_obj->als_value, ap3426_obj->hw->als_value,
		       sizeof(ap3426_obj->als_value));
	} else if (ap3426_obj->als_value_num !=
		   read_int_from_buf(ap3426_obj, buf, count, ap3426_obj->hw->als_value,
				     ap3426_obj->als_value_num)) {
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}

/*---------------------------------------------------------------------------------------*/
static ssize_t ap3426_show_cali(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = ap3426_i2c_client;
	struct ap3426_priv *obj = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", obj->ps_last_cali_status );
}

/*---------------------------------------------------------------------------------------*/
static ssize_t ap3426_store_cali(struct device_driver *dev, const char *buf, size_t count)
{
	struct i2c_client *client = ap3426_i2c_client;
	struct ap3426_priv *obj = i2c_get_clientdata ( client );
	int err = ap3426_do_calibration (client);

	obj->ps_last_cali_status = err;

	return count;
}
/*---------------------------------------------------------------------------------------*/
static ssize_t ap3426_show_ps_crosstalk ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = ap3426_i2c_client;
	struct ap3426_priv *obj = i2c_get_clientdata ( client );

	APS_ERR("ap3426_show_ps_crosstalk = %d\n", obj->ps_cali);

	return sprintf ( buf, "%d\n", obj->ps_cali );
}
/*---------------------------------------------------------------------------------------*/
static ssize_t ap3426_show_coefficent(struct device_driver *dev, char *buf)
{
	return sprintf ( buf, "FLU:%d, INC:%d \n", als_light_ratio_flu, als_light_ratio_inc);
}

/*---------------------------------------------------------------------------------------*/
static ssize_t ap3426_store_coefficent(struct device_driver *ddri, const char *buf, size_t count)
{
	int flu, inc;

	if (2 == sscanf(buf, "%d %d", &flu, &inc)) {
		als_light_ratio_flu = flu;
		als_light_ratio_inc = inc;
	} else {
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;
}
/*---------------------------------------------------------------------------------------*/

static DRIVER_ATTR(als, S_IWUSR | S_IRUGO, ap3426_show_als, NULL);
static DRIVER_ATTR(ps, S_IWUSR | S_IRUGO, ap3426_show_ps, NULL);
static DRIVER_ATTR(config, S_IWUSR | S_IRUGO, ap3426_show_config, ap3426_store_config);
static DRIVER_ATTR(alslv, S_IWUSR | S_IRUGO, ap3426_show_alslv, ap3426_store_alslv);
static DRIVER_ATTR(alsval, S_IWUSR | S_IRUGO, ap3426_show_alsval, ap3426_store_alsval);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, ap3426_show_trace, ap3426_store_trace);
static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, ap3426_show_status, NULL);
static DRIVER_ATTR(send, S_IWUSR | S_IRUGO, ap3426_show_send, ap3426_store_send);
static DRIVER_ATTR(recv, S_IWUSR | S_IRUGO, ap3426_show_recv, ap3426_store_recv);
static DRIVER_ATTR(reg, S_IWUSR | S_IRUGO, ap3426_show_reg, ap3426_store_reg);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, ap3426_show_cali, ap3426_store_cali);
static DRIVER_ATTR(ps_crosstalk, S_IWUSR | S_IRUGO , ap3426_show_ps_crosstalk, NULL );
static DRIVER_ATTR(als_coefficent, S_IWUSR | S_IRUGO , ap3426_show_coefficent, ap3426_store_coefficent );



/*----------------------------------------------------------------------------*/
static struct driver_attribute *ap3426_attr_list[] = {
	&driver_attr_als,
	&driver_attr_ps,
	&driver_attr_trace,	/*trace log */
	&driver_attr_config,
	&driver_attr_alslv,
	&driver_attr_alsval,
	&driver_attr_status,
	&driver_attr_send,
	&driver_attr_recv,
	&driver_attr_reg,
	&driver_attr_cali,
	&driver_attr_ps_crosstalk,
	&driver_attr_als_coefficent,
};

/*----------------------------------------------------------------------------*/
static int ap3426_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ap3426_attr_list) / sizeof(ap3426_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, ap3426_attr_list[idx]);
		if (err) {
			APS_ERR("driver_create_file (%s) = %d\n", ap3426_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int ap3426_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ap3426_attr_list) / sizeof(ap3426_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, ap3426_attr_list[idx]);

	return err;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------interrupt functions--------------------------------*/
static int ap3426_check_intr(struct i2c_client *client)
{
	struct ap3426_priv *obj = i2c_get_clientdata(client);
	int res = 1;
	u8 databuf[3] = {'\0'};
	int rawdata = 0;
	int obj_distance;

//	APS_ERR("start\n");

	databuf[0] = AP3426_INT_STATUS;
	res = AP3426_i2c_master_operate(client, databuf, 0x01, I2C_FLAG_READ);
	if (res < 0) {
		APS_ERR("read AP3426_INT_STATUS err: %d\n", res);
		goto EXIT_ERR;
	}

	APS_LOG("AP3426_INT_STATUS value = 0x%02x\n", databuf[0]);

	if(databuf[0] & PS_INT) {
		obj_distance = !(((int)databuf[0] & 0x10) >> 4); //In android, 0 for near, 1 for far

		/* clear ps interrupt flag */
		databuf[0] = AP3426_PDATA_L;
		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_READ);
		if (res < 0) {
			APS_ERR("read ps raw data err: %d\n", res);
			goto EXIT_ERR;
		}
		APS_LOG("ps raw data value = %d, obj_distance: %d\n", (((databuf[1] & 0x03) << 8) | databuf[0]), obj_distance);

		intr_flag = obj_distance;
		APS_LOG("intr_flag: %d, obj distance = %s\n", intr_flag, intr_flag==0?"near":"far");
		res = ps_report_interrupt_data(intr_flag);
		if (obj->ps_first_data){
		obj->ps_first_data = false;
		ap3426_set_persist(obj->client, AP3426_PERSIST_2TIMES);
		}
	}
	if(databuf[0] & ALS_INT) {
		/* clear als interrupt flag */
		databuf[0] = AP3426_ADATA_L;
		res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_READ);
		if (res < 0) {
			APS_ERR("read ps raw data err: %d\n", res);
			goto EXIT_ERR;
		}
		rawdata = databuf[0] | (databuf[1] << 8);
		if (rawdata >= atomic_read(&obj->als_thd_val_low)) {
			set_alssensor_threshold(client, atomic_read(&obj->als_thd_val_low), AP3426_ALS_MAX_DATA);
		} else if(rawdata <= atomic_read(&obj->als_thd_val_high)) {
			set_alssensor_threshold(client, AP3426_ALS_MIN_DATA, atomic_read(&obj->als_thd_val_high));
		}
#ifdef USES_ALS_WAKELOCK
		wake_lock_timeout(&obj->als_wl, msecs_to_jiffies(obj->als_wake_delay));
#endif
	}
	return 0;
EXIT_ERR:
	APS_ERR("ap3426_check_intr dev: %d\n", res);
	return res;
}

/*----------------------------------------------------------------------------*/
static void ap3426_eint_work(struct work_struct *work)
{
	struct ap3426_priv *obj =
	    (struct ap3426_priv *)container_of(work, struct ap3426_priv, eint_work);
	int res = 0;

//	APS_ERR("start\n");

	APS_LOG("ap3426_eint_work ap3426 int top half time = %lld\n", int_top_time);

	res = ap3426_check_intr(obj->client);
	if (res < 0) {
		APS_LOG("obj distance err: %d\n", res);
		goto EXIT_INTR_ERR;
	}

    enable_irq(obj->irq);
	return;
EXIT_INTR_ERR:
#if defined(CONFIG_OF)
	enable_irq(obj->irq);
#else
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif
	APS_ERR("err: %d\n", res);
}

/*----------------------------------------------------------------------------*/
static void ap3426_eint_func(void)
{
	struct ap3426_priv *obj = ap3426_obj;

//    APS_ERR("start\n");

	if (!obj)
		return;
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}

#if defined(CONFIG_OF)
static irqreturn_t ap3426_eint_handler(int irq, void *desc)
{
    APS_DBG("start\n");

    ap3426_eint_func();
	disable_irq_nosync(ap3426_obj->irq);

	return IRQ_HANDLED;
}
#endif

/*----------------------------------------------------------------------------*/
int ap3426_setup_eint(struct i2c_client *client)
{
#if defined(CONFIG_OF)
	u32 ints[2] = {0};
	int err = 0;
//	int ret;
//	struct pinctrl *pinctrl;
//	struct pinctrl_state *pins_default;
//	struct pinctrl_state *pins_cfg;
#endif

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return 0;
	}
#if 0
#if defined(CONFIG_OF)
	alspsPltFmDev = get_alsps_platformdev();

	/* gpio setting */
	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
		return ret;
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		APS_ERR("Cannot find alsps pinctrl default!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
		return ret;
	}
#else
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
#endif
#endif

#if defined(CONFIG_OF)
	if (ap3426_obj->irq_node) {
		of_property_read_u32_array(ap3426_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));

		//gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
//		pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0]=%d, ints[1]=%d\n", ints[0], ints[1]);

		ap3426_obj->irq = irq_of_parse_and_map(ap3426_obj->irq_node, 0);
		APS_LOG("ap3426_obj->irq = %d\n", ap3426_obj->irq);
		if (!ap3426_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		err = request_irq(ap3426_obj->irq, ap3426_eint_handler, IRQF_TRIGGER_FALLING, "ALS-eint", NULL);

		if (err != 0) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		//enable_irq(ap3426_obj->irq);
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}
#else
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, ap3426_eint_func, 0);

	mt_eint_mask(CUST_EINT_ALS_NUM);
#endif
	return 0;
}

/*-------------------------------MISC device related------------------------------------------*/



/************************************************************/
static int ap3426_open(struct inode *inode, struct file *file)
{
	file->private_data = ap3426_i2c_client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/************************************************************/
static int ap3426_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/************************************************************/

/*	Set PS threshold */

static int set_psensor_threshold(struct i2c_client *client)
{
	struct ap3426_priv *obj = i2c_get_clientdata(client);
	int res = 0;
	u8 databuf[3];

	APS_ERR("set_psensor_threshold function high: %d, low: %d\n",
		atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));
	databuf[0] = AP3426_INT_PS_LOW_THD_LOW;
	databuf[1] = atomic_read(&obj->ps_thd_val_low) & 0xFF;
	res = AP3426_i2c_master_operate(client, databuf, 2, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write AP3426_INT_PS_LOW_THD_LOW err: %d\n", res);
		return -1;
	}
	databuf[0] = AP3426_INT_PS_LOW_THD_HIGH;
	databuf[1] = atomic_read(&obj->ps_thd_val_low) >> 8;
	res = AP3426_i2c_master_operate(client, databuf, 2, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write AP3426_INT_PS_LOW_THD_HIGH err: %d\n", res);
		return -1;
	}

	databuf[0] = AP3426_INT_PS_HIGH_THD_LOW;
	databuf[1] = atomic_read(&obj->ps_thd_val_high) & 0xFF;
	res = AP3426_i2c_master_operate(client, databuf, 2, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write AP3426_INT_PS_HIGH_THD_LOW err: %d\n", res);
		return -1;
	}
	databuf[0] = AP3426_INT_PS_HIGH_THD_HIGH;
	databuf[1] = atomic_read(&obj->ps_thd_val_high) >> 8;
	res = AP3426_i2c_master_operate(client, databuf, 2, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write AP3426_INT_PS_HIGH_THD_HIGH err: %d\n", res);
		return -1;
	}
	return 0;

}

/*	Set ALS threshold */

static int set_alssensor_threshold(struct i2c_client *client, int thd_low, int thd_high)
{
	int res = 0;
	u8 databuf[3];

	APS_ERR("set_alssensor_threshold function low: 0x%x, high:0x%x\n",
		thd_low, thd_high);
	databuf[0] = AP3426_INT_ALS_LOW_THD_LOW;
	databuf[1] = thd_low & 0xFF;
	res = AP3426_i2c_master_operate(client, databuf, 2, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("read AP3426_INT_ALS_LOW_THD_LOW err: %d\n", res);
		return -1;
	}
	databuf[0] = AP3426_INT_ALS_LOW_THD_HIGH;
	databuf[1] = thd_low >> 8;
	res = AP3426_i2c_master_operate(client, databuf, 2, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("read AP3426_INT_ALS_LOW_THD_HIGH err: %d\n", res);
		return -1;
	}

	databuf[0] = AP3426_INT_ALS_HIGH_THD_LOW;
	databuf[1] = thd_high & 0xFF;
	res = AP3426_i2c_master_operate(client, databuf, 2, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("read AP3426_INT_ALS_HIGH_THD_LOW err: %d\n", res);
		return -1;
	}
	databuf[0] = AP3426_INT_ALS_HIGH_THD_HIGH;
	databuf[1] = thd_high >> 8;
	res = AP3426_i2c_master_operate(client, databuf, 2, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("read AP3426_INT_ALS_HIGH_THD_HIGH err: %d\n", res);
		return -1;
	}
	return 0;

}

/* Add calibration & Auto calibration - Kaku 20171109*/
static int ap3426_set_pcrosstalk(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	u8 databuf[3];
	msb = val >> 8;
	lsb = val & 0xFF;

	databuf[0] = AP3426_CALIBRATION_LOW;
	databuf[1] = lsb;
	err = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);

	if (err <= 0)
		{
			APS_ERR("write Low calibration register err: %d\n", err);
			return err;
		}

	databuf[0] = AP3426_CALIBRATION_HIGH;
	databuf[1] = msb;

	err = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);

	if (err <= 0)
		{
			APS_ERR("write High calibration register err: %d\n", err);
			return err;
		}

	return 0;
}

/* Add clear calibration register - Kaku 20171110*/
static int ap3426_clr_pcrosstalk(struct i2c_client *client)
{
	int err;
	u8 databuf [3];

	databuf[0] = AP3426_CALIBRATION_LOW;
	databuf[1] = 0;

	err = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (err < 0)
	{
		APS_ERR("Write Low calibration register err: %d\n", err);
		return err;
	}

	databuf[0] = AP3426_CALIBRATION_HIGH;
	databuf[1] = 0;

	err = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);

	if (err <= 0)
	{
		APS_ERR("write High calibration register err: %d\n", err);
		return err;
	}

	return 0;
}
static unsigned int ap3426_calc_calibration ( struct i2c_client *client )
{
    struct ap3426_priv *obj = i2c_get_clientdata(client);
	int err;
	int i = 0;
    u8 databuf[3] ={0};
	u8 ps_als_status = 0;
	u16 ui_ps_data = 0;
	u16 ps_crosstalk[20] = {0,};
	u16 ps_data_avg = 0;


    databuf[0] = AP3426_ENABLE;
    err = AP3426_i2c_master_operate(obj->client, databuf, 0x01, I2C_FLAG_READ);

    if (err < 0){
		APS_ERR("read AP3426_SYSTEM_MODE err: %d\n", err);
		goto err_out;
    }
	ps_als_status = databuf[0];

    ap3426_enable_als(obj->client, 0);
	ap3426_enable_ps(obj->client, 0);
	msleep(100);

    ap3426_clr_pcrosstalk(obj->client);
	ap3426_enable_ps(obj->client, 1);

	for (i=0; i<20; i++){
		msleep(30);
		err = ap3426_read_ps(obj->client, &obj->ps);
		if (err != 0){
			goto err_out;
		}
		if (atomic_read(&obj->trace) & CMC_TRC_CVT_PS){
			APS_LOG("AP3426 ps =%d\n", obj->ps);
		}
		ps_crosstalk[i] = obj->ps;
	}

	for(i=1; i<20; i++){
		ui_ps_data += ps_crosstalk[i];
	}
	ps_data_avg = ui_ps_data/19;

	if (atomic_read(&obj->trace) & CMC_TRC_CVT_PS){
		APS_LOG("AP3426 ps_data2 =%d\n", ps_data_avg);
	}

	if(ps_als_status & 0x01) {
		ap3426_enable_als(client,1);
	}
	if(ps_als_status == 0x00) {
		ap3426_enable_ps(client,0);
	}

	return ps_data_avg;

err_out:
	if(ps_als_status & 0x01) {
		ap3426_enable_als(client,1);
	}
	if(ps_als_status == 0x00) {
		ap3426_enable_ps(client,0);
	}
	APS_ERR("ap3xx6_read_ps fail\n");
	return -1;
}

static int ap3426_do_calibration(struct i2c_client *client)
{
    struct ap3426_priv *obj = i2c_get_clientdata(client);
	unsigned int calib_data;
	int err;
	int i;

	for (i=0; i< CALB_TIMES; i++){
		calib_data = ap3426_calc_calibration(client);
		if ( calib_data <= 500 /*200*/ /*25*/ /*870*/ ) {
			APS_ERR("ps_cross_talk save : %d\n", calib_data);
			obj->ps_cali = calib_data;
			break;
		}
		if(i==2 && calib_data > 500){
			APS_ERR("ps_cross_talk Fail Pdata: %d\n", calib_data);
			return -1;
		}
	}
#if defined(CALIBRATION_TO_FILE)
	sensor_calibration_save(ID_PROXIMITY, &obj->ps_cali);
#endif
	atomic_set(&obj->ps_thd_val_high, obj->ps_cali + ps_offset);
	atomic_set(&obj->ps_thd_val_low, (obj->ps_cali + ps_offset) - ps_hysteresis);
	//ap3426_set_pcrosstalk(obj->client, calib_data);
	err = set_psensor_threshold(client);
	if (err < 0) {
		APS_ERR("set ps threshold err: %d\n", err);
		return err;
	}
	msleep(70);

    return 0;
}

#if AP3426_PS_AUTO_CALI_EN
static int ap3426_calibration_every_time(struct i2c_client *client)
{
	int err;
	int i = 0;
	u16 ui_ps_data = 0;
	u8 databuf[3] ={0};
	u8 ps_als_status = 0;

	/* struct i2c_client *client = (struct i2c_client*)file->private_data; */
	struct ap3426_priv *obj = i2c_get_clientdata(client);

	u16 ps_crosstalk[5] = {0,0,0,0,0};
	u16 ps_data_avg = 0;
	databuf[0] = AP3426_ENABLE;
	err = AP3426_i2c_master_operate(obj->client, databuf, 0x01, I2C_FLAG_READ);

	if (err < 0)
		{
			APS_ERR("read AP3426_SYSTEM_MODE err: %d\n", err);
			goto err_out;
		}

	ps_als_status = databuf[0];
	databuf[0] = AP3426_ENABLE;
	err = AP3426_i2c_master_operate(obj->client, databuf, 0x02, I2C_FLAG_WRITE);

	if (err < 0)
		{
			APS_ERR("write AP3426_SYSTEM_MODE err: %d\n", err);
			goto err_out;
		}
	if (err<= 0)
		{
			goto err_out;
		}

	msleep(100);

	ap3426_set_pcrosstalk(obj->client, 0);
	ap3426_enable_ps(obj->client, 1);

	for (i = 0; i < 5; i++)
		{
			msleep(30);
			err = ap3426_read_ps(obj->client, &obj->ps);
			if (err != 0)
				{
					goto err_out;
				}
			if (atomic_read(&obj->trace) & CMC_TRC_CVT_PS)
				{
					APS_LOG("AP3426 ps =%d\n", obj->ps);
				}
			ps_crosstalk[i] = obj->ps;
		}

	for(i = 1;i < 5;i ++)
		{
			ui_ps_data += ps_crosstalk[i];
		}
	ps_data_avg = ui_ps_data/4;

	if (atomic_read(&obj->trace) & CMC_TRC_CVT_PS)
		{
			APS_LOG("AP3426 ps_data2 =%d\n", ps_data_avg);
			}
	if(ps_data_avg > 511)
		{
			ap3426_set_pcrosstalk(obj->client, 511);
		}

	else	{
			ap3426_set_pcrosstalk(obj->client, ps_data_avg);
		}	msleep(70);

	if(ps_als_status & 0x01)
		{
			ap3426_enable_als(client,1);
		}

	return 0;

err_out:
		if(ps_als_status & 0x01)
		{
			ap3426_enable_als(client,1);
		}
		APS_ERR("ap3xx6_read_ps fail\n");
		return -1;
}

#endif

static long ap3426_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct ap3426_priv *obj = i2c_get_clientdata(client);
	long err = 0;
	void __user *ptr = (void __user *)arg;
	int dat;
	uint32_t enable;
	int ps_result;
	int ps_cali;
	int threshold[2];

	switch (cmd) {

	case ALSPS_SET_PS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		if (enable) {
			err = ap3426_enable_ps(obj->client, 1);
			if (err) {
				APS_ERR("enable ps fail: %ld\n", err);
				goto err_out;
			}

			set_bit(CMC_BIT_PS, &obj->enable);
		} else {
			err = ap3426_enable_ps(obj->client, 0);
			if (err) {
				APS_ERR("disable ps fail: %ld\n", err);
				goto err_out;
			}
			clear_bit(CMC_BIT_PS, &obj->enable);
		}
		break;

	case ALSPS_GET_PS_MODE:
		enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_DATA:
		err = ap3426_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;

		dat = ap3426_get_ps_value(obj, obj->ps);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_RAW_DATA:
		err = ap3426_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;

		dat = obj->ps;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_SET_ALS_MODE:

		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		if (enable) {
			err = ap3426_enable_als(obj->client, 1);
			if (err) {
				APS_ERR("enable als fail: %ld\n", err);
				goto err_out;
			}
			set_bit(CMC_BIT_ALS, &obj->enable);
		} else {
			err = ap3426_enable_als(obj->client, 0);
			if (err) {
				APS_ERR("disable als fail: %ld\n", err);
				goto err_out;
			}
			clear_bit(CMC_BIT_ALS, &obj->enable);
		}
		break;

	case ALSPS_GET_ALS_MODE:
		enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_DATA:
		err = ap3426_read_als(obj->client, &obj->als);
		if (err)
			goto err_out;

		dat = ap3426_get_als_value(obj, obj->als);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_RAW_DATA:
		err = ap3426_read_als(obj->client, &obj->als);
		if (err)
			goto err_out;

		dat = obj->als;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

		/*----------------------------------for factory mode test---------------------------------------*/
	case ALSPS_GET_PS_TEST_RESULT:
		err = ap3426_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;

		if (obj->ps > atomic_read(&obj->ps_thd_val_low))
			ps_result = 0;
		else
			ps_result = 1;

		if (copy_to_user(ptr, &ps_result, sizeof(ps_result))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_IOCTL_CLR_CALI:
		if (copy_from_user(&dat, ptr, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		if (dat == 0)
			obj->ps_cali = 0;

		break;

	case ALSPS_IOCTL_GET_CALI:
		ps_cali = obj->ps_cali;
		if (copy_to_user(ptr, &ps_cali, sizeof(ps_cali))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_IOCTL_SET_CALI:
		APS_ERR ( "CMD = ALSPS_IOCTL_SET_CALI\n" );

#if defined(CALIBRATION_TO_FILE)
		ps_cali = 0;
		err = sensor_calibration_read(ID_PROXIMITY, &ps_cali);
		if (err != 0){
			APS_ERR("Read Cal Fail from file !!!\n");
			atomic_set(&obj->ps_thd_val_high, obj->hw->ps_threshold_high);
			atomic_set(&obj->ps_thd_val_low, obj->hw->ps_threshold_low);
			err = set_psensor_threshold(client);
			if (err < 0) {
				APS_ERR("set ps threshold err: %ld\n", err);
				goto err_out;
			}
			msleep(70);

			break;
		}
#else
		if (copy_from_user(&ps_cali, ptr, sizeof(ps_cali))) {
			err = -EFAULT;
			goto err_out;
		}
#endif
		obj->ps_cali = ps_cali;
		//ap3426_set_pcrosstalk(obj->client, obj->ps_cali);
		atomic_set(&obj->ps_thd_val_high, obj->ps_cali + ps_offset);
		atomic_set(&obj->ps_thd_val_low, (obj->ps_cali + ps_offset) - ps_hysteresis);
		err = set_psensor_threshold(client);
		if (err < 0) {
			APS_ERR("set ps threshold err: %ld\n", err);
			goto err_out;
		}
		msleep(70);

		break;

	case ALSPS_SET_PS_THRESHOLD:
		if (copy_from_user(threshold, ptr, sizeof(threshold))) {
			err = -EFAULT;
			goto err_out;
		}
		APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],
			threshold[1]);
		atomic_set(&obj->ps_thd_val_high, (threshold[0] + obj->ps_cali));
		atomic_set(&obj->ps_thd_val_low, (threshold[1] + obj->ps_cali));	/* need to confirm */

		set_psensor_threshold(obj->client);

		break;

	case ALSPS_GET_PS_THRESHOLD_HIGH:
		threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
		APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
		if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_THRESHOLD_LOW:
		threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
		APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
		if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
			err = -EFAULT;
			goto err_out;
		}
		break;
		/*------------------------------------------------------------------------------------------*/

	default:
		APS_ERR("%s not supported = 0x%04x", __func__, cmd);
		err = -ENOIOCTLCMD;
		break;
	}

err_out:
	return err;
}

#ifdef CONFIG_COMPAT
static long compat_ap3426_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	APS_FUN();

	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		APS_ERR("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case COMPAT_ALSPS_SET_PS_MODE:
	case COMPAT_ALSPS_GET_PS_MODE:
	case COMPAT_ALSPS_GET_PS_DATA:
	case COMPAT_ALSPS_GET_PS_RAW_DATA:
	case COMPAT_ALSPS_SET_ALS_MODE:
	case COMPAT_ALSPS_GET_ALS_MODE:
	case COMPAT_ALSPS_GET_ALS_DATA:
	case COMPAT_ALSPS_GET_ALS_RAW_DATA:
	case COMPAT_ALSPS_GET_PS_TEST_RESULT:
	case COMPAT_ALSPS_GET_ALS_TEST_RESULT:
	case COMPAT_ALSPS_GET_PS_THRESHOLD_HIGH:
	case COMPAT_ALSPS_GET_PS_THRESHOLD_LOW:
	case COMPAT_ALSPS_GET_ALS_THRESHOLD_HIGH:
	case COMPAT_ALSPS_GET_ALS_THRESHOLD_LOW:
	case COMPAT_ALSPS_IOCTL_CLR_CALI:
	case COMPAT_ALSPS_IOCTL_GET_CALI:
	case COMPAT_ALSPS_IOCTL_SET_CALI:
	case COMPAT_ALSPS_SET_PS_THRESHOLD:
	case COMPAT_ALSPS_SET_ALS_THRESHOLD:
	case COMPAT_AAL_SET_ALS_MODE:
	case COMPAT_AAL_GET_ALS_MODE:
	case COMPAT_AAL_GET_ALS_DATA:
		return filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
	default:
		APS_ERR("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static const struct file_operations ap3426_fops = {
	.owner = THIS_MODULE,
	.open = ap3426_open,
	.release = ap3426_release,
	.unlocked_ioctl = ap3426_unlocked_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_ap3426_unlocked_ioctl,
#endif
};

static struct miscdevice ap3426_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ap3426_fops,
};

/*--------------------------------------------------------------------------------*/
static int ap3426_parse_dt(const char* name)
{
	struct device_node *node = NULL;
	int ret;
	u32 nv_ps_offset[] = {0};
	u32 nv_ps_hysteresis[] = {0};
	u32 nv_ps_integrated_time[] = {0};
	u32 nv_als_threshold_high[] = {0};
	u32 nv_als_threshold_low[] = {0};
	u32 nv_als_light_ratio_flu[] = {0};
	u32 nv_als_light_ratio_inc[] = {0};

	node = of_find_compatible_node(NULL, NULL, name);

	if(node)
	{
		ret = of_property_read_u32_array(node, "ps_offset", nv_ps_offset, ARRAY_SIZE(nv_ps_offset));
		if (ret == 0)
			ps_offset = nv_ps_offset[0];
		ret = of_property_read_u32_array(node, "ps_hysteresis", nv_ps_hysteresis, ARRAY_SIZE(nv_ps_hysteresis));
		if (ret == 0)
			ps_hysteresis = nv_ps_hysteresis[0];
		ret = of_property_read_u32_array(node, "ps_integrated_time", nv_ps_integrated_time, ARRAY_SIZE(nv_ps_integrated_time));
		if (ret == 0)
			ps_integrated_time = nv_ps_integrated_time[0];
		ret = of_property_read_u32_array(node, "als_threshold_high", nv_als_threshold_high, ARRAY_SIZE(nv_als_threshold_high));
		if (ret == 0)
			als_threshold_high = nv_als_threshold_high[0];
		ret = of_property_read_u32_array(node, "als_threshold_low", nv_als_threshold_low, ARRAY_SIZE(nv_als_threshold_low));
		if (ret == 0)
			als_threshold_low = nv_als_threshold_low[0];
		ret = of_property_read_u32_array(node, "als_light_ratio_flu", nv_als_light_ratio_flu, ARRAY_SIZE(nv_als_light_ratio_flu));
		if (ret == 0)
			als_light_ratio_flu = nv_als_light_ratio_flu[0];
		ret = of_property_read_u32_array(node, "als_light_ratio_inc", nv_als_light_ratio_inc, ARRAY_SIZE(nv_als_light_ratio_inc));
		if (ret == 0)
			als_light_ratio_inc = nv_als_light_ratio_inc[0];
	} else
    {
        APS_ERR("Device Tree: can not find alsps node!. Go to use old cust info\n");
		return -1;
    }

	return 0;
}

/*--------------------------------------------------------------------------------*/
static int ap3426_init_client(struct i2c_client *client)
{
	struct ap3426_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3] = {'\0'};
	int res = 0;

	APS_FUN();

	databuf[0] = AP3426_ENABLE;
	databuf[1] = 0x00;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write AP3426_ENABLE err: %d\n", res);
		goto EXIT_ERR;
	}

	/* Interrupt flag */
	databuf[0] = 0x01;
	databuf[1] = 0x00;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write interrupt flag err: %d\n", res);
		goto EXIT_ERR;
	}

	/* System Waiting time*/
	databuf[0] = AP3426_WAITING_TIME;
	databuf[1] = AP3426_ALSPS_WAITING_TIME;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write System Waiting Time err: %d\n", res);
		goto EXIT_ERR;
	}

	/* Interrupt control */
	databuf[0] = AP3426_INT_CTL;
	databuf[1] = 0x00;
	if (0 == obj->hw->polling_mode_ps) {
        databuf[1] |= 0x80;
		/* set ps threshold */
		res = set_psensor_threshold(client);
		if (res < 0) {
			APS_ERR("set ps threshold err: %d\n", res);
			goto EXIT_ERR;
		}
	}
	if (0 == obj->hw->polling_mode_als) {
        databuf[1] |= 0x08;
		/* set ALS threshold */
		res = set_alssensor_threshold(client, atomic_read(&obj->als_thd_val_low), atomic_read(&obj->als_thd_val_high));
		if (res < 0) {
			APS_ERR("set als threshold err: %d\n", res);
			goto EXIT_ERR;
		}
	}
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write interrupt contrl err: %d\n", res);
		goto EXIT_ERR;
	}

    /* ALS gain */
    databuf[0] = 0x10;
	databuf[1] = 0x30;
	obj->als_gain = databuf[1];
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write als gain err: %d\n", res);
		goto EXIT_ERR;
	}

	/* PS gain */
    	databuf[0] = 0x20;
	databuf[1] = 0x00;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write ps gain err: %d\n", res);
		goto EXIT_ERR;
	}

	/* PS LED DRIVER */
    	databuf[0] = 0x21;
	databuf[1] = 0x03;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write ps led driver err: %d\n", res);
		goto EXIT_ERR;
	}

	/* PS Mean Time */
    	databuf[0] = 0x23;
	databuf[1] = 0x01;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write ps mean time err: %d\n", res);
		goto EXIT_ERR;
	}

	/* PS Integrated Time */
    	databuf[0] = 0x25;
	databuf[1] = ps_integrated_time;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write ps integrated time err: %d\n", res);
		goto EXIT_ERR;
	}

	/* PS Persis */
	databuf[0] = 0x26;
	databuf[1] = 0x02;
	res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
	if (res <= 0) {
		APS_ERR("write ps mean time err: %d\n", res);
		goto EXIT_ERR;
	}

	/* Default Light Ratio */
	obj->als_current_ratio = als_light_ratio_flu;
	obj->als_read_count = AP3426_ALS_READ_COUNT_MAX;  //First time check light ratio

	/* Clear PS calibration register */
	res = ap3426_clr_pcrosstalk(client);
	if(res !=0) {
		APS_ERR("PS set clear pcrosstalk: %d\n", res);
		return res;
	}

	/* Set PS crosstalk*/
	res = ap3426_set_pcrosstalk(client, 0);//Initial crosstalk setting
	if(res != 0){
		APS_ERR("PS set pcrosstalk err: %d\n", res);
		return res;
	}

	res = ap3426_setup_eint(client);
	if (res != 0) {
		APS_ERR("setup eint: %d\n", res);
		return res;
	}

	return AP3426_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/*--------------------------------------------------------------------------------*/

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int als_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int als_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("ap3426_obj als enable value = %d\n", en);

	mutex_lock(&ap3426_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &ap3426_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &ap3426_obj->enable);
	mutex_unlock(&ap3426_mutex);

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return -1;
	}

	res = ap3426_enable_als(ap3426_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}

	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

int als_get_data(int *value, int *status)
{
	int err = 0;
	struct ap3426_priv *obj = NULL;

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return -1;
	}
	obj = ap3426_obj;
	err = ap3426_read_als(obj->client, &obj->als);
	if (err) {
		err = -1;
	} else {
		*value = ap3426_get_als_value(obj, obj->als);
		if (*value < 0){
			err = -1;
		}
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int ps_get_data(int *value, int *status)
{
        int err = 0;

        if (!ap3426_obj) {
                APS_ERR("ap3426_obj is null!!\n");
                return -1;
        }

        err = ap3426_read_ps(ap3426_obj->client, &ap3426_obj->ps);
        if (err) {
                err = -1;
        } else {
                *value = ap3426_get_ps_value(ap3426_obj, ap3426_obj->ps);
                if (*value < 0)
                        err = -1;
                *status = SENSOR_STATUS_ACCURACY_MEDIUM;
        }

        return err;
}

static int ps_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("ap3426_obj ps enable value = %d\n", en);

	mutex_lock(&ap3426_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &ap3426_obj->enable);

	else
		clear_bit(CMC_BIT_PS, &ap3426_obj->enable);

	mutex_unlock(&ap3426_mutex);

	if (!ap3426_obj) {
		APS_ERR("ap3426_obj is null!!\n");
		return -1;
	}

#if AP3426_PS_AUTO_CALI_EN
	if(en)
	{
		res= ap3426_calibration_every_time(ap3426_obj->client);
	}
	else
	{
		res = ap3426_enable_ps(ap3426_obj->client, en)
	}
#else
	res = ap3426_enable_ps(ap3426_obj->client, en);
#endif

	if (res) {
		APS_ERR("ps_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	APS_FUN();

	value = (int)samplingPeriodNs/1000/1000;
	/*FIX  ME */
#ifdef USES_ALS_WAKELOCK
	ap3426_obj->als_wake_delay = value;
#endif
	APS_LOG("ap3426 als set delay = (%d) ok.\n", value);
	return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	APS_FUN();

	return als_flush_report();
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	APS_FUN();

	value = (int)samplingPeriodNs/1000/1000;
	/*FIX  ME */

	APS_LOG("ap3426 ps set delay = (%d) ok.\n", value);
	return 0;
}

static int ps_flush(void)
{
	APS_FUN();

	return ps_flush_report();
}

#if defined(CONFIG_FB)
static int ap3426_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct ap3426_priv *obj = ap3426_obj;
	struct i2c_client *client = obj->client;
    struct fb_event *evdata = (struct fb_event *)data;
    int *blank = NULL;
	u8 databuf[3] = {'\0'};
	int res = 0;

    if (evdata && evdata->data)
        blank = (int *)evdata->data;
    else
        return 0;

    if (event == FB_EVENT_BLANK) {
        if (*blank == FB_BLANK_POWERDOWN) {
			/* Interrupt control */
			databuf[0] = AP3426_INT_CTL;
			if (0 == obj->hw->polling_mode_als) {
				databuf[1] |= 0x88;
				/* set ALS threshold */
				res = set_alssensor_threshold(client, atomic_read(&obj->als_thd_val_low), atomic_read(&obj->als_thd_val_high));
				if (res < 0) {
					APS_ERR("set als threshold err: %d\n", res);
					return res;
				}
				res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
				if (res <= 0) {
					APS_ERR("write interrupt contrl err: %d\n", res);
					return res;
				}
			}
            APS_ERR("[IN] LCD Sleep\n");
        } else if (*blank == FB_BLANK_UNBLANK) {
			/* Interrupt control */
			databuf[0] = AP3426_INT_CTL;
			databuf[1] |= 0x80;
			res = AP3426_i2c_master_operate(client, databuf, 0x02, I2C_FLAG_WRITE);
				if (res <= 0) {
					APS_ERR("write interrupt contrl err: %d\n", res);
					return res;
				}
            APS_ERR("[OUT] LCD Sleep\n");
        }
    }
    return 0;
}
#endif

/*-----------------------------------i2c operations----------------------------------*/
static int ap3426_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ap3426_priv *obj;

	int err = 0;
	struct als_control_path als_ctl = { 0 };
	struct als_data_path als_data = { 0 };
	struct ps_control_path ps_ctl = { 0 };
	struct ps_data_path ps_data = { 0 };
	//struct regulator *vmch = NULL;

	APS_FUN();

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	err = get_alsps_dts_func(client->dev.of_node, hw);
	if (err < 0) {
		APS_ERR("get dts info fail\n");
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ap3426_obj = obj;

	//vmch = regulator_get(&client->dev, "vmch");
	//regulator_set_voltage(vmch, 3300000, 3300000);
	//err = regulator_enable(vmch);
	//msleep(30);

	obj->hw = hw;

	INIT_WORK(&obj->eint_work, ap3426_eint_work);

	obj->client = client;
	i2c_set_clientdata(client, obj);

	/*-----------------------------value need to be confirmed-----------------------------------------*/
	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_auto_gain_deb, 150);
#ifdef CONFIG_64BIT
	atomic64_set(&obj->als_deb_end, 0);
    atomic64_set(&obj->als_auto_gain_deb_end, 0);
#else
	atomic_set(&obj->als_deb_end, 0);
    atomic_set(&obj->als_auto_gain_deb_end, 0);
#endif
	atomic_set(&obj->ps_debounce, 200);
	atomic_set(&obj->ps_deb_on, 0);
#ifdef CONFIG_64BIT
	atomic64_set(&obj->ps_deb_end, 0);
#else
	atomic_set(&obj->ps_deb_end, 0);
#endif
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val, 0xC1);
	atomic_set(&obj->ps_thd_val_high, obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low, obj->hw->ps_threshold_low);
	atomic_set(&obj->als_thd_val_high, als_threshold_high);
	atomic_set(&obj->als_thd_val_low, als_threshold_low);
	atomic_set(&obj->init_done, 0);
	atomic_set(&obj->trace, 1);
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
    if(obj->irq_node ){
        if(obj->irq_node->name){
            APS_LOG("irq_node name = %s\n", obj->irq_node->name);
        }
        else{
            APS_LOG("irq_node name is NULL \n");
        }
    }else{
        APS_LOG("irq_node is NULL \n");
    }

	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
	obj->ps_bef_enable=false;
	obj->als_level_num = sizeof(obj->hw->als_level) / sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value) / sizeof(obj->hw->als_value[0]);
	/*-----------------------------value need to be confirmed-----------------------------------------*/

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);

	ap3426_i2c_client = client;

	err = ap3426_init_client(client);
	if (err)
		goto exit_init_failed;
	APS_LOG("ap3426_init_client() OK!\n");


	err = misc_register(&ap3426_device);
	if (err) {
		APS_ERR("ap3426_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	als_ctl.is_use_common_factory = false;
	ps_ctl.is_use_common_factory = false;
	APS_LOG("ap3426_device misc_register OK!\n");

	/*------------------------ap3426 attribute file for debug--------------------------------------*/
	err = ap3426_create_attr(&(ap3426_init_info.platform_diver_addr->driver));
	if (err) {
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------ap3426 attribute file for debug--------------------------------------*/
	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay = als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;
	als_ctl.is_polling_mode = obj->hw->polling_mode_als;

	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}


	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	ps_ctl.is_polling_mode = obj->hw->polling_mode_ps;

	err = ps_register_control_path(&ps_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
#if defined(CONFIG_FB)
	obj->fb_notif.notifier_call = ap3426_fb_notifier_callback;
	fb_register_client(&obj->fb_notif);
#endif
#ifdef USES_ALS_WAKELOCK
	wake_lock_init(&obj->als_wl, WAKE_LOCK_SUSPEND, "ap3426_als_wl");
#endif
#if 0
	err = batch_register_support_info(ID_LIGHT, als_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register light batch support err = %d\n", err);

	err = batch_register_support_info(ID_PROXIMITY, ps_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register proximity batch support err = %d\n", err);
#endif
	ap3426_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
	misc_deregister(&ap3426_device);
exit_init_failed:
	kfree(obj);
exit:
	ap3426_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	ap3426_init_flag = -1;
	return err;
}

static int ap3426_i2c_remove(struct i2c_client *client)
{
	struct ap3426_priv *obj = i2c_get_clientdata(client);
	int err;
	/*------------------------ap3426 attribute file for debug--------------------------------------*/
	err = ap3426_delete_attr(&(ap3426_init_info.platform_diver_addr->driver));
	if (err)
		APS_ERR("ap3426_delete_attr fail: %d\n", err);
	/*----------------------------------------------------------------------------------------*/

	err = misc_deregister(&ap3426_device);
	if (err)
		APS_ERR("misc_deregister fail: %d\n", err);

#if defined(CONFIG_FB)
	fb_unregister_client(&obj->fb_notif);
#endif
#ifdef USES_ALS_WAKELOCK
	wake_lock_destroy(&obj->als_wl);
#endif
	ap3426_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;

}

static int ap3426_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strncpy(info->type, AP3426_DEV_NAME, sizeof(info->type));
	return 0;

}

static int ap3426_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct ap3426_priv *obj = i2c_get_clientdata(client);
	//int err;

	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return 0;
	}

	atomic_set(&obj->als_suspend, 1);

#if 0
	err = ap3426_enable_als(obj->client, 0);
	if (err)
		APS_ERR("disable als fail: %d\n", err);
#endif

	return 0;
}

static int ap3426_i2c_resume(struct i2c_client *client)
{

	struct ap3426_priv *obj = i2c_get_clientdata(client);
//	int err;
//	struct hwm_sensor_data sensor_data;

//	memset(&sensor_data, 0, sizeof(sensor_data));
	APS_FUN();
	if (!obj) {
		APS_ERR("null pointer!!\n");
		return 0;
	}

	atomic_set(&obj->als_suspend, 0);

#if 0
	if (test_bit(CMC_BIT_ALS, &obj->enable)) {
		err = ap3426_enable_als(obj->client, 1);
		if (err)
			APS_ERR("enable als fail: %d\n", err);
	}
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int ap3426_remove(void)
{
	ap3426_power(hw, 0);

	i2c_del_driver(&ap3426_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/

static int ap3426_local_init(void)
{
	APS_FUN();

	ap3426_power(hw, 1);
	if (i2c_add_driver(&ap3426_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}
	if (-1 == ap3426_init_flag)
		return -1;

	return 0;
}


/*----------------------------------------------------------------------------*/
static int __init ap3426_init(void)
{
	/*const char *name = "mediatek,ap3426";

	hw = get_alsps_dts_func(name, hw);
	if (!hw) {
		APS_ERR("get_alsps_dts_func fail\n");
		return 0;
	}*/
	ap3426_parse_dt("mediatek,ap3426");
	APS_FUN();
	alsps_driver_add(&ap3426_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit ap3426_exit(void)
{
	APS_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(ap3426_init);
module_exit(ap3426_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dyna-Image");
MODULE_DESCRIPTION("AP3426 driver");
MODULE_LICENSE("GPL");
