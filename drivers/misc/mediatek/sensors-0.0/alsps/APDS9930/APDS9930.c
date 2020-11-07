/* drivers/hwmon/mt6516/amit/APDS9930.c - APDS9930 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

#include <alsps.h>

#include <hwmsensor.h>
#include <cust_alsps.h>
#include "APDS9930.h"
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/regulator/consumer.h>


#define CALIBRATION_TO_FILE 1

#if defined(CALIBRATION_TO_FILE)
#define CAL_TO_PERSIST
#include "../../sensor_cal/sensor_cal_file_io.h"
#endif

//#define APS_DEBUG

struct regulator *reg = NULL;
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define APDS9930_DEV_NAME     "apds9930"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               pr_debug(APS_TAG"%s\n", __func__)
#define APS_ERR(fmt, args...)    pr_err(APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    pr_debug(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    pr_debug(APS_TAG fmt, ##args)

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

#define APDS9930_I2CADDR (0x39)

#define I2C_MAX_TRY 3

#define APDS9930_ALS_THRESHOLD_HSYTERESIS    1    /* % */
#define APDS9930_DF    52

bool on_boot = false;
unsigned int alsps_int_gpio_number = 0;
static int of_get_APDS9930_platform_data(struct device *dev);
struct alsps_hw alsps_cust;
struct apds9930_hw local_alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
static struct apds9930_hw *proxi_hw = &local_alsps_cust;
struct platform_device *alspsPltFmDev;

/* For alsp driver get cust info */
struct alsps_hw *get_cust_alsps(void)
{
	return &alsps_cust;
}

struct apds9930_hw *get_cust_local_alsps(void)
{
	return &local_alsps_cust;
}

struct apds9930_hw {
	u32 near_offset;
	u32 far_offset;
	u32 crosstalk_max;
	u32 als_ga;
	u32 als_coe_b;
	u32 als_coe_c;
	u32 als_coe_d;

	unsigned int ppcount;
	unsigned int ps_led_current;

	u32 als_threshold;
	char* vdd;
};



/*----------------------------------------------------------------------------*/
static struct i2c_client *APDS9930_i2c_client;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id APDS9930_i2c_id[] = { {APDS9930_DEV_NAME, 0}, {} };

/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int APDS9930_i2c_remove(struct i2c_client *client);
static int APDS9930_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int APDS9930_i2c_resume(struct i2c_client *client);
static int APDS9930_remove(void);
static int APDS9930_local_init(void);

static int APDS9930_init_client(struct i2c_client *client);
static int light_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
//static int ps_get_data(int *value, int *status);

static int set_psensor_threshold(struct i2c_client *client, uint16_t pilt, uint16_t piht);
static int APDS9930_init_flag = -1; /* 0<==>OK -1 <==> fail*/
static int forced_far = 0;
static struct alsps_init_info APDS9930_init_info = {
		.name = "apds9930",
		.init = APDS9930_local_init,
		.uninit = APDS9930_remove,
};

/*calibration*/
#define CALB_BOX_TIMES 20
#define CALB_REMOVAL_TIME 5
#define CALB_TIMES 3

static DEFINE_MUTEX(APDS9930_mutex);


static struct APDS9930_priv *g_APDS9930_ptr;
static unsigned int alsps_irq;

static unsigned long long int_top_time;

//static uint8_t apds9930_als_atime_tb[] = {0xF6, 0XED, 0XDB};
static uint32_t apds9930_als_integration_tb[] = {2720, 5168, 10064};
static uint16_t apds9930_als_res_tb[] = { 10240, 19456, 37888 };
static uint8_t apds9930_als_again_tb[] = {1, 8, 16, 120};
#ifdef CONFIG_LGE_SENSOR_DYNAMIC_AGAIN
static uint8_t apds9930_als_again_bit_tb[] = { 0x00, 0x01, 0x02, 0x03};
#endif

enum{
	APDS9930_ALS_RES_10240 = 0,
	APDS9930_ALS_RES_19456 = 1,
	APDS9930_ALS_RES_37888 = 2
};

enum{
	APDS9930_ALS_GAIN_1X = 0,
	APDS9930_ALS_GAIN_8X = 1,
	APDS9930_ALS_GAIN_16X = 2,
	APDS9930_ALS_GAIN_120X = 3
};

enum {
	CMC_BIT_ALS = 1,
	CMC_BIT_PS = 2,
};

enum {
	PS_INT = 1,
	ALS_INT = 2,
	PS_ALS_INT = 3,
};

enum {
	PS_CALI_FAILED = -1,
	PS_CALI_SUCCESS = 0,
	PS_CALI_NONE = 1,
};

/*----------------------------------------------------------------------------*/
struct APDS9930_i2c_addr {	/*define a series of i2c slave address */
	u8 write_addr;
	u8 ps_thd;		/*PS INT threshold */
};
/*----------------------------------------------------------------------------*/
struct APDS9930_priv {
	struct alsps_hw *hw;
	struct apds9930_hw *local_hw;
	struct i2c_client *client;
	struct work_struct irq_work;

	/*i2c address group */
	struct APDS9930_i2c_addr addr;

	/*misc */
	u16 als_modulus;
	atomic_t als_suspend;
	atomic_t als_debounce;	/*debounce time after enabling als */
	atomic_t als_deb_on;	/*indicates if the debounce is on */
	atomic_t als_deb_end;	/*the jiffies representing the end of debounce */
	atomic_t ps_mask;	/*mask ps: always return far away */
	atomic_t ps_debounce;	/*debounce time after enabling ps */
	atomic_t ps_deb_on;	/*indicates if the debounce is on */
	atomic_t ps_deb_end;	/*the jiffies representing the end of debounce */
	atomic_t ps_suspend;


	/*data */
	u16 als;
	u16 ps;
	u8 _align;
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[C_CUST_ALS_LEVEL - 1];
	u32 als_value[C_CUST_ALS_LEVEL];
	unsigned int ps_cali;
	bool is_psat_enabled;
	int ps_last_cali_status;

	u16 als_atime_index;
	u16 als_again_index;

	atomic_t als_cmd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_cmd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_high;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_low;	/*the cmd value can't be read, stored in ram */
	ulong enable;		/*enable mask */
	ulong pending_intr;	/*pending interrupt */
#ifdef CONFIG_LGE_SENSOR_DYNAMIC_AGAIN
    u16 als_reduce;
#endif

	/*early suspend */
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif

#if defined(CONFIG_FB)
	struct notifier_block    fb_notif;
#endif
};

static atomic_t driver_suspend_flag = ATOMIC_INIT(0);
static unsigned int PS_DEFAULT_CAL = 200; /* NEED TO FIX */
static unsigned int ALS_DEFAULT_GAIN = APDS9930_ALS_GAIN_8X;

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,apds9930"},
	{},
};
#endif
/*----------------------------------------------------------------------------*/
static struct i2c_driver APDS9930_i2c_driver = {
	.probe = APDS9930_i2c_probe,
	.remove = APDS9930_i2c_remove,
	.detect = APDS9930_i2c_detect,
	.suspend = APDS9930_i2c_suspend,
	.resume = APDS9930_i2c_resume,
	.id_table = APDS9930_i2c_id,
	.driver = {
		.name = APDS9930_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
	},
};

static struct APDS9930_priv *APDS9930_obj;
/*------------------------i2c function for 89-------------------------------------*/
int APDS9930_i2c_master_operate(struct i2c_client *client, char *buf, int count, int i2c_flag)
{
	int res = 0;
	int retry = 0;

	mutex_lock(&APDS9930_mutex);
	switch (i2c_flag) {
	case I2C_FLAG_WRITE:
		/* client->addr &= I2C_MASK_FLAG; */
		for(retry=0; retry < I2C_MAX_TRY; retry++){
			res = i2c_master_send(client, buf, count);
			if(res < 0){
				APS_ERR("APDS9930 transfer error, retry:%d\n", retry+1);
				msleep(20);
			}else
				break;
		}
		/* client->addr &= I2C_MASK_FLAG; */
		break;

	case I2C_FLAG_READ:
		/*
		   client->addr &= I2C_MASK_FLAG;
		   client->addr |= I2C_WR_FLAG;
		   client->addr |= I2C_RS_FLAG;
		 */
		for(retry=0; retry < I2C_MAX_TRY; retry++){
			res = i2c_master_send(client, buf, count & 0xFF);
			/* client->addr &= I2C_MASK_FLAG; */
			res = i2c_master_recv(client, buf, count >> 0x08);
			if(res < 0){
				APS_ERR("APDS9930 transfer error, retry:%d\n", retry+1);
				msleep(20);
			}else
				break;
		}
		break;
	default:
		APS_ERR("APDS9930_i2c_master_operate i2c_flag command not support!\n");
		break;
	}
	if (res <= 0)
		goto EXIT_ERR;

	mutex_unlock(&APDS9930_mutex);
	return res;
 EXIT_ERR:
	mutex_unlock(&APDS9930_mutex);
	APS_ERR("APDS9930_i2c_transfer fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
int APDS9930_get_addr(struct alsps_hw *hw, struct APDS9930_i2c_addr *addr)
{
	if (!hw || !addr)
		return -EFAULT;

	addr->write_addr = hw->i2c_addr[0];
	return 0;
}

/*----------------------------------------------------------------------------*/
static void APDS9930_power(struct apds9930_hw *local_hw,struct alsps_hw *hw, unsigned int on)
{
	struct i2c_client *client = APDS9930_i2c_client;
	int poweron;

	if(local_hw->vdd == NULL){
		//don't control the power line
	} else {
		if(reg == NULL)
			reg = regulator_get(NULL, local_hw->vdd);

		poweron = regulator_is_enabled(reg);
		APS_ERR("Power %d, %d !\n", poweron, on);

		if(on){
			APS_ERR("Power on!\n");
			if(regulator_set_voltage(reg, hw->power_vol, hw->power_vol) != 0)
				APS_ERR("Power on fail!\n");

			if(regulator_enable(reg) != 0)
				APS_ERR("Power on fail!\n");

			//re-init setting
			if(on_boot == false){
				msleep(50);
				APDS9930_init_client(client);
			}
			on_boot = false;

		}else if(poweron && !on){
			APS_ERR("Power off!\n");
			if(regulator_disable(reg) != 0){
				APS_ERR("Power off fail!\n");
			}
		}
	}
}

/*____________________________________________________________________________*/
struct apds9930_hw *get_apds9930_dts_func(const char *name, struct apds9930_hw *hw)
{
	int ret;
	u32 near_offset[] = {0};
	u32 far_offset[] = {0};
	u32 crosstalk_max[] = {0};
	u32 ppcount[] = {0};
	u32 ps_led_current[] = {0};
	u32 als_threshold[] = {0};
	u32 als_ga[] = {0};
	u32 als_coe_b[] = {0};
	u32 als_coe_c[] = {0};
	u32 als_coe_d[] = {0};
	char* vdd;

	struct device_node *node = NULL;

	if (name == NULL)
		return NULL;

	node = of_find_compatible_node(NULL, NULL, name);
	if (node) {
		ret = of_property_read_u32_array(node, "near_offset", near_offset, ARRAY_SIZE(near_offset));
		if (ret == 0)
			hw->near_offset = near_offset[0];

		ret = of_property_read_u32_array(node, "far_offset", far_offset, ARRAY_SIZE(far_offset));
		if (ret == 0)
			hw->far_offset = far_offset[0];

		ret = of_property_read_u32_array(node, "crosstalk_max", crosstalk_max, ARRAY_SIZE(crosstalk_max));
		if (ret == 0)
			hw->crosstalk_max = crosstalk_max[0];

		ret = of_property_read_u32_array(node, "ppcount", ppcount, ARRAY_SIZE(ppcount));
		if (ret == 0)
			hw->ppcount = ppcount[0];

		ret = of_property_read_u32_array(node, "ps_led_current", ps_led_current, ARRAY_SIZE(ps_led_current));
		if (ret == 0)
			hw->ps_led_current = ps_led_current[0];

		ret = of_property_read_u32_array(node, "als_threshold", als_threshold, ARRAY_SIZE(als_threshold));
		if (ret == 0)
			hw->als_threshold =	als_threshold[0];

		ret = of_property_read_u32_array(node, "als_ga", als_ga, ARRAY_SIZE(als_ga));
		if (ret == 0)
			hw->als_ga = als_ga[0];

		ret = of_property_read_u32_array(node, "als_coe_b", als_coe_b, ARRAY_SIZE(als_coe_b));
		if (ret == 0)
			hw->als_coe_b = als_coe_b[0];
		ret = of_property_read_u32_array(node, "als_coe_c", als_coe_c, ARRAY_SIZE(als_coe_c));
		if (ret == 0)
			hw->als_coe_c = als_coe_c[0];

		ret = of_property_read_u32_array(node, "als_coe_d", als_coe_d, ARRAY_SIZE(als_coe_d));
		if (ret == 0)
			hw->als_coe_d = als_coe_d[0];

		ret = of_property_read_string(node, "vdd", (const char **)&vdd);
		if (ret == 0)
			hw->vdd = vdd;

	} else {
		APS_ERR("Device Tree: can not find alsps node!. Go to use old cust info\n");
		return NULL;
	}
	return hw;
}

/*----------------------------------------------------------------------------*/
static int set_asensor_threshold(struct i2c_client *client, uint16_t ailt, uint16_t aiht)
{
	//struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3];
	int res = 0;

	databuf[0] = APDS9930_CMM_AINT_LOW_THD_LOW;
	databuf[1] = (u8)(ailt & 0x00FF);

	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		return -1;

	databuf[0] = APDS9930_CMM_AINT_LOW_THD_HIGH;
	databuf[1] = (u8)((ailt & 0xFF00) >> 8);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		return -1;

	databuf[0] = APDS9930_CMM_AINT_HIGH_THD_LOW;
	databuf[1] = (u8)(aiht & 0x00FF);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		return -1;

	databuf[0] = APDS9930_CMM_AINT_HIGH_THD_HIGH;
	databuf[1] = (u8)((aiht & 0xFF00) >> 8);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		return -1;

	return 0;
}

/*----------------------------------------------------------------------------*/
static int set_psensor_threshold(struct i2c_client *client, uint16_t pilt, uint16_t piht)
{
	//struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3];
	int res = 0;

	databuf[0] = APDS9930_CMM_PINT_LOW_THD_LOW;
	databuf[1] = (u8)(pilt & 0x00FF);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		return -1;

	databuf[0] = APDS9930_CMM_PINT_LOW_THD_HIGH;
	databuf[1] = (u8)((pilt & 0xFF00) >> 8);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		return -1;

	databuf[0] = APDS9930_CMM_PINT_HIGH_THD_LOW;
	databuf[1] = (u8)(piht & 0x00FF);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		return -1;

	databuf[0] = APDS9930_CMM_PINT_HIGH_THD_HIGH;
	databuf[1] = (u8)((piht & 0xFF00) >> 8);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		return -1;

	return 0;
}

#if defined(CONFIG_FB)
static int light_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	struct fb_event *evdata = (struct fb_event *)data;
	int *blank = NULL;
//    uint16_t low_threshold;
//    uint16_t  high_threshold;

	if (evdata && evdata->data)
		blank = (int *)evdata->data;
	else
		return APDS9930_SUCCESS;

	if (event == FB_EVENT_BLANK) {
		if (*blank == FB_BLANK_POWERDOWN) {
			atomic_set(&driver_suspend_flag, 1);
			APS_ERR("[IN] LCD Sleep\n");
		} else if (*blank == FB_BLANK_UNBLANK) {
			atomic_set(&driver_suspend_flag, 0);
			APS_ERR("[OUT] LCD Sleep\n");
			if(test_bit(CMC_BIT_ALS, &obj->enable)){
				set_asensor_threshold(client, 0xFFFF, 0);
			}
		}
	}
	return APDS9930_SUCCESS;
}
#endif


/*----------------------------------------------------------------------------*/
static long APDS9930_enable_als(struct i2c_client *client, int enable)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	long res = 0;

	if(!test_bit(CMC_BIT_PS, &obj->enable) && enable)
			APDS9930_power(proxi_hw, hw, 1);

	databuf[0] = APDS9930_CMM_ENABLE;
	res = APDS9930_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if (res <= 0)
		goto EXIT_ERR;

	if (enable) {
		if(obj->hw->polling_mode_als == 1){
			databuf[1] = databuf[0] | 0x03;
		}else{
			set_asensor_threshold(client, 0xFFFF,0);
			databuf[1] = databuf[0] | 0x13;
		}

		databuf[0] = APDS9930_CMM_ENABLE;
		APS_ERR("APDS9930_CMM_ENABLE enable als value = %x\n", databuf[1]);
		res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0)
			goto EXIT_ERR;

		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end,
			   jiffies + atomic_read(&obj->als_debounce) / (1000 / HZ));
		msleep(100);
	} else {
		if (test_bit(CMC_BIT_PS, &obj->enable))
			databuf[1] = databuf[0] & 0xED;
		else
			databuf[1] = databuf[0] & 0xE8;

		databuf[0] = APDS9930_CMM_ENABLE;
		APS_ERR("APDS9930_CMM_ENABLE disable als value = %x\n", databuf[1]);
		res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0)
			goto EXIT_ERR;

		if(!test_bit(CMC_BIT_PS, &obj->enable))
			APDS9930_power(proxi_hw,hw, 0);

	}
	return 0;

 EXIT_ERR:
	APS_ERR("APDS9930_enable_als fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static long APDS9930_enable_ps(struct i2c_client *client, int enable)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	long res = 0;

	if(!test_bit(CMC_BIT_ALS, &obj->enable) && enable)
		APDS9930_power(proxi_hw,hw, 1);

	databuf[0] = APDS9930_CMM_ENABLE;
	res = APDS9930_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if (res <= 0)
		goto EXIT_ERR;

	if (enable) {
		databuf[1] = databuf[0] | 0x05;
		databuf[0] = APDS9930_CMM_ENABLE;
		APS_LOG("APDS9930_CMM_ENABLE enable ps value = %x\n", databuf[1]);
		res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0)
			goto EXIT_ERR;

		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end,
			   jiffies + atomic_read(&obj->ps_debounce) / (1000 / HZ));
	} else {
		if (test_bit(CMC_BIT_ALS, &obj->enable))
			databuf[1] = databuf[0] & 0xFB;
		else
			databuf[1] = databuf[0] & 0xF8;

		databuf[0] = APDS9930_CMM_ENABLE;
		APS_LOG("APDS9930_CMM_ENABLE disable ps value = %x\n", databuf[1]);
		res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0)
			goto EXIT_ERR;

		/*fix bug */

		set_psensor_threshold(client, obj->hw->ps_threshold_low, obj->hw->ps_threshold_high);
		/*fix bug */

		if(!test_bit(CMC_BIT_ALS, &obj->enable))
			APDS9930_power(proxi_hw,hw, 0);

	}
	return 0;

 EXIT_ERR:
	APS_ERR("APDS9930_enable_ps fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
/*for interrupt work mode support -- by liaoxl.lenovo 12.08.2011*/
static int APDS9930_check_and_clear_intr(struct i2c_client *client)
{
	int res, intp, intl;
	u8 buffer[2];

	gpio_direction_input(alsps_int_gpio_number);
	if (gpio_get_value(alsps_int_gpio_number) == 1)	/*skip if no interrupt */
		return 0;

	buffer[0] = APDS9930_CMM_STATUS;
	res = APDS9930_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if (res <= 0)
		goto EXIT_ERR;

	res = 0;
	intp = 0;
	intl = 0;
	if (0 != (buffer[0] & 0x20)) {
		res = 1;
		intp = 1;
	}
	if (0 != (buffer[0] & 0x10)) {
		res = 1;
		intl = 1;
	}

	if (1 == res) {
		if ((1 == intp) && (0 == intl))
			buffer[0] = (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | 0x05);
		else if ((0 == intp) && (1 == intl))
			buffer[0] = (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | 0x06);
		else
			buffer[0] = (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | 0x07);


		res = APDS9930_i2c_master_operate(client, buffer, 0x1, I2C_FLAG_WRITE);
		if (res <= 0)
			goto EXIT_ERR;
		else
			res = 0;
	}

	return res;

 EXIT_ERR:
	APS_ERR("APDS9930_check_and_clear_intr fail\n");
	return 1;
}

/*----------------------------------------------------------------------------*/

/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int APDS9930_check_intr(struct i2c_client *client)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 buffer[2];
	int res;
	obj->is_psat_enabled = false;

	gpio_direction_input(alsps_int_gpio_number);
	if (gpio_get_value(alsps_int_gpio_number) == 1)	/*skip if no interrupt */
		return 0;

	buffer[0] = APDS9930_CMM_STATUS;
	res = APDS9930_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		APS_ERR("APDS9930_check_intr fail\n");
		return -1;
	}

	if (0 != (buffer[0] & 0x40))
		obj->is_psat_enabled = true;

	if (0 != (buffer[0] & 0x20))  /* only PS is interrupted */
		return PS_INT;

	if (0 != (buffer[0] & 0x10))  /* only ALS is interrupted */
		return ALS_INT;

	if (0 != (buffer[0] & 0x30))  /* both PS and ALS are interruped */
		return PS_ALS_INT;

	return -1;
}

static int APDS9930_clear_intr(struct i2c_client *client)
{
	int res;
	u8 buffer[2];

	buffer[0] = (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | 0x07);
	res = APDS9930_i2c_master_operate(client, buffer, 0x1, I2C_FLAG_WRITE);
	if (res <= 0)
		goto EXIT_ERR;
	else
		res = 0;

	return res;

 EXIT_ERR:
	APS_ERR("APDS9930_check_and_clear_intr fail\n");
	return 1;
}



static irqreturn_t alsps_interrupt_handler(int irq, void *dev_id)
{
	struct APDS9930_priv *obj = g_APDS9930_ptr;

	if (!obj)
		return IRQ_HANDLED;

	disable_irq_nosync(alsps_irq);
	int_top_time = sched_clock();
	schedule_work(&obj->irq_work);
	return IRQ_HANDLED;
}


int APDS9930_irq_registration(struct i2c_client *client)
{
	int ret = -1;

	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	/*struct task_struct *thread = NULL;*/

	g_APDS9930_ptr = obj;


	gpio_direction_input(alsps_int_gpio_number);

	ret = request_irq(alsps_irq, alsps_interrupt_handler, IRQF_TRIGGER_FALLING, "ALS-eint", NULL);

	if (ret > 0) {
		APS_ERR("alsps request_irq IRQ LINE NOT AVAILABLE!.");
		return ret;
	}

 /*   disable_irq_nosync(alsps_irq);*/
 /*   enable_irq(alsps_irq);*/

	return ret;

}


/*----------------------------------------------------------------------------*/

static int APDS9930_init_client(struct i2c_client *client)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	databuf[0] = (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | 0x00);
	res = APDS9930_i2c_master_operate(client, databuf, 0x1, I2C_FLAG_WRITE);
	if (res <= 0)
		goto EXIT_ERR;

	databuf[0] = APDS9930_CMM_ENABLE;
	if (obj->hw->polling_mode_ps == 1)
		databuf[1] = 0x08;
	if (obj->hw->polling_mode_ps == 0)
		databuf[1] = 0x28;

	if (obj->hw->polling_mode_als == 0)
		databuf[1] |= 0x10;

	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		goto EXIT_ERR;

	databuf[0] = APDS9930_CMM_ATIME;
	databuf[1] = 0xF6;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		goto EXIT_ERR;

	databuf[0] = APDS9930_CMM_PTIME;
	databuf[1] = 0xFF;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		goto EXIT_ERR;

	databuf[0] = APDS9930_CMM_WTIME;
	databuf[1] = 0xEE;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		goto EXIT_ERR;

	/*for interrupt work mode support -- by liaoxl.lenovo 12.08.2011 */
	if (0 == obj->hw->polling_mode_ps) {
		set_psensor_threshold(client, obj->hw->ps_threshold_low, obj->hw->ps_threshold_high);
	}

	databuf[0] = APDS9930_CMM_Persistence;
	databuf[1] = 0x22;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		goto EXIT_ERR;

	databuf[0] = APDS9930_CMM_CONFIG;
	databuf[1] = 0x00;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		goto EXIT_ERR;


	/*Lenovo-sw chenlj2 add 2011-06-03,modified pulse 2  to 4 */
	databuf[0] = APDS9930_CMM_PPCOUNT;
	databuf[1] = obj->local_hw->ppcount;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		goto EXIT_ERR;


	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
	databuf[0] = APDS9930_CMM_CONTROL;
	databuf[1] = obj->local_hw->ps_led_current;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0)
		goto EXIT_ERR;


	return APDS9930_SUCCESS;

 EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/******************************************************************************
 * Function Configuration
******************************************************************************/
int APDS9930_read_als(struct i2c_client *client, u16 *data)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u16 c0_value, c1_value;
	int IAC, IAC1, IAC2;
	u8 buffer[2];
	int res = 0;
	int luxValue = 0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0] = APDS9930_CMM_C0DATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if (res <= 0)
		goto EXIT_ERR;


	c0_value = buffer[0] | (buffer[1] << 8);

	buffer[0] = APDS9930_CMM_C1DATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if (res <= 0)
		goto EXIT_ERR;


	c1_value = buffer[0] | (buffer[1] << 8);

#ifdef CONFIG_LGE_SENSOR_DYNAMIC_AGAIN
	if(c0_value >= apds9930_als_res_tb[obj->als_atime_index] ||
		c1_value >= apds9930_als_res_tb[obj->als_atime_index]){

		if(obj->als_again_index != APDS9930_ALS_GAIN_1X) {
			return 0;
		} else {
			*data = 30000;
			return 0;
		}
	}
#else
	if(c0_value >= apds9930_als_res_tb[obj->als_atime_index] ||
		c1_value >= apds9930_als_res_tb[obj->als_atime_index]){
		*data = 30000;
		return 0;
	}
#endif

	IAC1 = c0_value - (obj->local_hw->als_coe_b * c1_value) / 100;
	IAC2 = obj->local_hw->als_coe_c * c0_value/100 - obj->local_hw->als_coe_d*c1_value/100;
#ifdef APS_DEBUG
	APS_ERR("c0:%d, c1:%d, IAC1:%d, IAC2:%d\n", c0_value, c1_value, IAC1, IAC2);
#endif
	if(IAC1 > IAC2)
		IAC = IAC1;
	else if(IAC1 <= IAC2)
		IAC = IAC2;
	else
		IAC = 0;

#ifdef CONFIG_LGE_SENSOR_DYNAMIC_AGAIN
	if(IAC1<0 && IAC2<0){
		return 0;
	}
#else
	if(IAC1<0 && IAC2<0){
		IAC = 0;
	}
#endif

	luxValue = ((IAC * obj->local_hw->als_ga * APDS9930_DF)/100 / (apds9930_als_integration_tb[obj->als_atime_index]/100 *apds9930_als_again_tb[obj->als_again_index]));

#ifdef CONFIG_LGE_SENSOR_DYNAMIC_AGAIN
    if(obj->als_reduce){
        luxValue = (((IAC * obj->local_hw->als_ga * APDS9930_DF)/100) *60)/10 / (apds9930_als_integration_tb[obj->als_atime_index]/100 *apds9930_als_again_tb[obj->als_again_index]);
    }
#endif

#ifdef APS_DEBUG
	APS_ERR("als_res_index:%d, als_gain_index:%d, data:%d \n", obj->als_atime_index, obj->als_again_index, *data);
	APS_ERR("luxValue = %d, obj->als_reduce=%d \n", luxValue, obj->als_reduce);
#endif

	*data = (luxValue < 30000) ? luxValue : 30000;

	return 0;

 EXIT_ERR:
	APS_ERR("APDS9930_read_ps fail\n");
	return res;
}

int APDS9930_read_als_ch0(struct i2c_client *client, u16 *data)
{
	/* struct APDS9930_priv *obj = i2c_get_clientdata(client); */
	u16 c0_value;
	u8 buffer[2];
	int res = 0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
/* get adc channel 0 value */
	buffer[0] = APDS9930_CMM_C0DATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if (res <= 0)
		goto EXIT_ERR;

	c0_value = buffer[0] | (buffer[1] << 8);
	*data = c0_value;
	/* APS_LOG("c0_value=%d\n", c0_value); */
	return 0;



 EXIT_ERR:
	APS_ERR("APDS9930_read_ps fail\n");
	return res;
}

int APDS9930_read_als_ch1(struct i2c_client *client, u16 *data)
{
	/* struct APDS9930_priv *obj = i2c_get_clientdata(client); */
	u16 c1_value;
	u8 buffer[2];
	int res = 0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
/* get adc channel 1 value */
	buffer[0] = APDS9930_CMM_C1DATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if (res <= 0)
		goto EXIT_ERR;

	c1_value = buffer[0] | (buffer[1] << 8);
	*data = c1_value;
	/* APS_LOG("c1_value=%d\n", c1_value); */
	return 0;



 EXIT_ERR:
	APS_ERR("APDS9930_read_ps fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
long APDS9930_read_ps(struct i2c_client *client, u16 *data)
{
//	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 buffer[2];
	u16 temp_data;
	long res = 0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0] = APDS9930_CMM_PDATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if (res <= 0)
		goto EXIT_ERR;


	temp_data = buffer[0] | (buffer[1] << 8);
		*data = temp_data;
	return 0;

 EXIT_ERR:
	APS_ERR("APDS9930_read_ps fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static int APDS9930_get_ps_value(struct APDS9930_priv *obj, u16 ps)
{
	int val;		/* mask = atomic_read(&obj->ps_mask); */
	int invalid = 0;

	APS_ERR("APDS9930_get_value ps=%d psat=%d\n", ps, obj->is_psat_enabled);

	if ( obj->is_psat_enabled ) {
		val = 1;	/* forced far */
		forced_far = 1;
	} else if (ps > atomic_read(&obj->ps_thd_val_high)){
		val = 0;
	} else {
		val = 1;
		forced_far = 0;
	}
#if 0
	if ((ps > atomic_read(&obj->ps_thd_val_high))&& !obj->is_psat_enabled ) {
		val = 0;	/*close */
	} else if (ps < atomic_read(&obj->ps_thd_val_low)) {
		val = 1;	/*far away */
	} else
		val = 1;
#endif
	if (atomic_read(&obj->ps_suspend)) {
		invalid = 1;
	} else if (1 == atomic_read(&obj->ps_deb_on)) {
		unsigned long endt = atomic_read(&obj->ps_deb_end);

		if (time_after(jiffies, endt)) {
			atomic_set(&obj->ps_deb_on, 0);
		}

		if (1 == atomic_read(&obj->ps_deb_on)) {
			invalid = 1;
		}

	} else if (obj->als > 45000) {
		/* invalid = 1; */
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;	/*far away */
	}

	if (!invalid) {
		/* APS_DBG("PS:  %05d => %05d\n", ps, val); */
		return val;
	} else {
		return -1;
	}
}

/*calibration*/
void APDS9930_swap(int *x, int *y)
{
	int temp = *x;
	*x = *y;
	*y = temp;
}

static unsigned int APDS9930_calc_calibration ( struct i2c_client *client )
{
    unsigned int value;
	int temp_pdata[CALB_BOX_TIMES] = {0,};
//	int temp_state[CALB_BOX_TIMES] = {0,};
	unsigned int i = 0;
	unsigned int j = 0;
    unsigned int sum_of_pdata = 0;
    int result;
	struct APDS9930_priv *obj = i2c_get_clientdata(client);

	/* Enable PS and Mask interrupt */

	if (!test_bit(CMC_BIT_PS, &obj->enable)) {
		result = APDS9930_enable_ps(client, 1);
		if (result != APDS9930_SUCCESS) {
			return (result);
		}
	}
	mdelay ( 10 );

	/* Read pdata */
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		APDS9930_read_ps(client, (u16 *) &(temp_pdata[i]));
		mdelay ( 10 );
	}

#if defined ( APS_DEBUG )
	APS_LOG ( "Read Value = " );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		APS_ERR ( "%d ", temp_pdata[i] );
	}
	APS_ERR ( "\n" );
#endif

	/* sort pdata */
	for ( i = 0 ; i < CALB_BOX_TIMES - 1 ; i++ )
	{
		for ( j = i + 1 ; j < CALB_BOX_TIMES ; j++ )
		{
			if ( temp_pdata[i] > temp_pdata[j] )
			{
				APDS9930_swap( temp_pdata+i, temp_pdata+j );
			}
		}
	}

#if defined(APS_DEBUG)
	APS_LOG ( "Read Value = " );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		APS_ERR ( "%d ", temp_pdata[i] );
	}
	APS_ERR ( "\n" );
#endif

	/* take ten middle data only */
	for ( i = CALB_REMOVAL_TIME ; i < (CALB_BOX_TIMES - CALB_REMOVAL_TIME) ; i++ )
	{
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	/* calculate average */
    value = sum_of_pdata / (CALB_BOX_TIMES - (CALB_REMOVAL_TIME * 2));
	APS_ERR ( "New calibrated cross talk = %d\n", value );

    return (value);
}

static int APDS9930_do_calibration(struct i2c_client *client)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	unsigned int calib_data;
	unsigned int old_enable = 0;
//	unsigned char old_modectl;
	int result, i;

	old_enable  = obj->enable;

	for (i=0; i< CALB_TIMES; i++){
		calib_data = APDS9930_calc_calibration(client);
		if ( calib_data <= obj->local_hw->crosstalk_max /*200*/ /*25*/ /*870*/ ) {
			APS_ERR("ps_cross_talk save : %d\n", calib_data);
			obj->ps_cali = calib_data;
			break;
		}
	}

	if (i >= CALB_TIMES) {
		APS_ERR ("failed to calibrate cross talk \n");
		return -1;
	} else {
#if defined(CALIBRATION_TO_FILE)
		sensor_calibration_save(ID_PROXIMITY, &obj->ps_cali);
#endif
		atomic_set(&obj->ps_thd_val_low, (obj->local_hw->near_offset + obj->ps_cali));
		atomic_set(&obj->ps_thd_val_high,(obj->local_hw->far_offset + obj->ps_cali));
		obj->hw->ps_threshold_high = obj->local_hw->far_offset + obj->ps_cali;
		obj->hw->ps_threshold_low = obj->local_hw->near_offset + obj->ps_cali;
	}

	result = APDS9930_enable_ps( client, old_enable );
    if (result != APDS9930_SUCCESS) {
        return (result);
    }

	/* we should store it to storage ( it should be free from factory reset ) but ATCI Demon will store it through LGP Demon */
	return 0;
}

#ifdef CONFIG_LGE_SENSOR_DYNAMIC_AGAIN
static int APDS9930_change_gain(u16 als_ch0){
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	int change_again;
	u8 databuf[3];
	int res = 0;

	if (als_ch0 >= ((apds9930_als_res_tb[obj->als_atime_index] * 90) / 100)) {
		if (obj->als_again_index != APDS9930_ALS_GAIN_1X) {
			obj->als_again_index--;
			change_again = 1;
		} else if (!obj->als_reduce) {
			databuf[0] = APDS9930_CMM_CONFIG;
			databuf[1] = APDS9930_ALS_REDUCE;
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
#ifdef APS_DEBUG
			APS_ERR("APDS9930_change_gain als_ch0 = %d, als_gain = 0x%0x\n", als_ch0, databuf[1] & 0x03);
#endif
			if (res >= 0)
				obj->als_reduce = 1;
		}
	} else if (als_ch0 <= apds9930_als_res_tb[obj->als_atime_index] * 10 / 100) {
		if (obj->als_reduce) {
			databuf[0] = APDS9930_CMM_CONFIG;
			databuf[1] = 0x00;
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
#ifdef APS_DEBUG
			APS_ERR("APDS9930_change_gain als_ch0 = %d, als_gain = 0x%0x\n", als_ch0, databuf[1] & 0x03);
#endif
			if (res >= 0)
				obj->als_reduce = 0;
        }
        else if (obj->als_again_index != APDS9930_ALS_GAIN_120X) {
			obj->als_again_index++;
			change_again = 1;
		}
	}

	if (change_again) {
		databuf[0] = APDS9930_CMM_CONTROL;
		res = APDS9930_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
		if ( res < 0 )
			return -1;

		databuf[1] = (databuf[0] & 0xFC) | apds9930_als_again_bit_tb[obj->als_again_index];
		databuf[0] = APDS9930_CMM_CONTROL;
		res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
#ifdef APS_DEBUG
		APS_ERR("APDS9930_change_gain als_ch0 = %d, als_gain = 0x%0x\n", als_ch0, databuf[1] & 0x03);
#endif

		if ( res < 0 )
			return -1;
	}

	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
/*for interrupt work mode support -- by liaoxl.lenovo 12.08.2011*/
/* #define DEBUG_APDS9930 */
static void APDS9930_irq_work(struct work_struct *work)
{
	struct APDS9930_priv *obj =
	    (struct APDS9930_priv *)container_of(work, struct APDS9930_priv, irq_work);
	int err;
	struct hwm_sensor_data sensor_data;
//	u8 databuf[3];
	u16 als_ch0 = 0;
	u16 als_ch1 = 0;
	int res = 0;
	unsigned int ps_state, adata = 0;
	unsigned int als_low_threshold, als_high_threshold, als_pocket_threshold;

	/* disable_irq_nosync(alsps_irq); */

	res = APDS9930_check_intr(obj->client);
	APDS9930_clear_intr(obj->client);

    APS_ERR("cali =%d\n", obj->ps_cali);
    APS_ERR("ps_threshold_high =%d\n", obj->hw->ps_threshold_high) ;
    APS_ERR("ps_threshold_low =%d\n", obj->hw->ps_threshold_low );

	if (res == PS_INT || res == PS_ALS_INT) {
		/* get raw data */
		APDS9930_read_ps(obj->client, &obj->ps);
		//APDS9930_read_als_ch0(obj->client, &obj->als);
		//APS_ERR("APDS9930_irq_work rawdata ps=%d als_ch0=%d!\n", obj->ps, obj->als);
		APS_ERR("APDS9930_irq_work rawdata ps=%d\n", obj->ps);
#ifdef APS_DEBUG
		APS_ERR("APDS9930 int top half time = %lld\n", int_top_time);
#endif
#if 0
		if (obj->als > 40000) {
			APS_ERR("APDS9930_irq_work ALS too large may under lighting als_ch0=%d!\n",
				obj->als);
			return;
		}
#endif

		ps_state = APDS9930_get_ps_value(obj, obj->ps);
		sensor_data.values[0] = ps_state;
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

/*signal interrupt function add*/
		if (ps_state) {  //far
			if (forced_far == 1) {
				set_psensor_threshold(obj->client, 0, 1022);
				APS_ERR("FAR! \n");
			} else {
				set_psensor_threshold(obj->client, 0, obj->hw->ps_threshold_high);
				APS_ERR("FAR \n");
			}

		} else {
			if (forced_far == 1) {
				set_psensor_threshold(obj->client, 1022, 1023);
				APS_ERR("NEAR! \n");
			} else {
				set_psensor_threshold(obj->client, obj->hw->ps_threshold_low, 1023);
				APS_ERR("NEAR \n");
			}
		}

		err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);
		if (err)
			APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);

	}
	if (res == ALS_INT || res == PS_ALS_INT) {
		APDS9930_read_als(obj->client, &obj->als);
		APDS9930_read_als_ch0(obj->client, &als_ch0);
		APDS9930_read_als_ch1(obj->client, &als_ch1);

		adata = obj->als;
		sensor_data.values[0] = adata;
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

		APS_ERR("APDS9930_irq_work als=%d!, ch0 = %d\n", adata, als_ch0);

		err = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data);
		if (err)
			APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);

		//todo threshold
		if (atomic_read(&driver_suspend_flag) == 1) {
            als_pocket_threshold = obj->local_hw->als_threshold * apds9930_als_again_tb[obj->als_again_index]/apds9930_als_again_tb[ALS_DEFAULT_GAIN];
			APS_ERR("One threshold detect mode\n");
			if (als_ch0 <= als_pocket_threshold) {
				APS_ERR("als_data(%d)<THRESHOLD(%d), Change bright to dark\n", als_ch0, als_pocket_threshold);
				set_asensor_threshold(obj->client, 0, als_pocket_threshold);
			} else {
				APS_ERR("als_data(%d)>THRESHOLD(%d), Change dark to bright\n", als_ch0, als_pocket_threshold);
				set_asensor_threshold(obj->client, als_pocket_threshold, 0xFFFF);
			}
		} else if (atomic_read(&driver_suspend_flag) == 0) {
				APS_ERR("normal mode\n");
			als_low_threshold = (als_ch0 * (100- APDS9930_ALS_THRESHOLD_HSYTERESIS)) / 100;
			als_high_threshold = (als_ch0 * (100+ APDS9930_ALS_THRESHOLD_HSYTERESIS)) / 100;

			if (als_high_threshold >= apds9930_als_res_tb[obj->als_atime_index])
				als_high_threshold = apds9930_als_res_tb[obj->als_atime_index];

#ifdef CONFIG_LGE_SENSOR_DYNAMIC_AGAIN
			if ( (als_ch0 >= apds9930_als_res_tb[obj->als_atime_index] ||
				als_ch1 >= apds9930_als_res_tb[obj->als_atime_index]) &&
				(obj->als_again_index != APDS9930_ALS_GAIN_1X) ) {
				set_asensor_threshold(obj->client, 0xFFFF, 0);
			} else {
				set_asensor_threshold(obj->client, als_low_threshold, als_high_threshold);
			}
#else
			set_asensor_threshold(obj->client, als_low_threshold, als_high_threshold);
#endif

			if (1 == als_ch0)
				set_asensor_threshold(obj->client, 1, 1);
#ifdef CONFIG_LGE_SENSOR_DYNAMIC_AGAIN
		APDS9930_change_gain(als_ch0);
#endif
		}
	}

 /*   disable_irq_nosync(alsps_irq);*/
	enable_irq(alsps_irq);
	enable_irq_wake(alsps_irq);
	return;
#if 0
 EXIT_ERR:
	APDS9930_clear_intr(obj->client);
 /*   disable_irq_nosync(alsps_irq);*/
	enable_irq(alsps_irq);
	enable_irq_wake(alsps_irq);
	APS_ERR("i2c_transfer error = %d\n", res);
#endif
}

static int APDS9930_get_deivceid( struct i2c_client *client, unsigned char *Data )
{
	int ret = 0;
	u8 databuf[2];
	
	databuf[0] = APDS9930_CMM_ID;
	ret = APDS9930_i2c_master_operate(client, databuf, 0x01, I2C_FLAG_READ);
	if (ret <= 0)
		return ret;
	else {
		*Data = databuf[0] | (databuf[1] << 8);
		APS_DBG ( "DEVICEID=0x%02x\n", *Data );
	}

	return ret;
}

/******************************************************************************
 * ADB Shell command function
******************************************************************************/
static ssize_t APDS9930_show_cali(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", obj->ps_last_cali_status );
}

static ssize_t APDS9930_store_cali(struct device_driver *dev, const char *buf, size_t count)
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );
	int err = APDS9930_do_calibration (client);

	obj->ps_last_cali_status = err;

	return count;
}
static ssize_t APDS9930_show_ps_led ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%x\n", data->local_hw->ps_led_current );
}

static ssize_t APDS9930_store_ps_led(struct device_driver *dev, const char *buf, size_t count)
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;
	u8 databuf[2];

	databuf[0] = APDS9930_CMM_CONTROL;
	databuf[1] = (unsigned int)val;
	ret = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);

	if ( ret < 0 )
		return ret;

	obj->local_hw->ps_led_current = (unsigned int)val;

	return count;
}

static ssize_t APDS9930_show_ps_pulse ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%x\n", data->local_hw->ppcount  );
}

static ssize_t APDS9930_store_ps_pulse(struct device_driver *dev, const char *buf, size_t count)
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;
	u8 databuf[2];

	databuf[0] = APDS9930_CMM_PPCOUNT;
	databuf[1] = (unsigned int)val;
	ret = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);

	if ( ret < 0 )
		return ret;

	obj->local_hw->ppcount = (unsigned int)val;

	return count;
}

static ssize_t APDS9930_show_pilt ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->hw->ps_threshold_low );
}

static ssize_t APDS9930_store_pilt ( struct device_driver *dev, const char *buf, size_t count )
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = set_psensor_threshold(client,(uint16_t)val,obj->hw->ps_threshold_high);

	if ( ret < 0 )
		return ret;

	atomic_set(&obj->ps_thd_val_low, val);
	obj->hw->ps_threshold_low = (unsigned int)val;

	return count;
}

static ssize_t APDS9930_show_piht ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->hw->ps_threshold_high );
}

static ssize_t APDS9930_store_piht ( struct device_driver *dev, const char *buf, size_t count )
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = set_psensor_threshold(client,obj->hw->ps_threshold_low,(uint16_t)val);

	if ( ret < 0 )
		return ret;

	atomic_set(&obj->ps_thd_val_high, val);
	obj->hw->ps_threshold_high = (unsigned int)val;

	return count;
}

static ssize_t APDS9930_show_pdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );

    APDS9930_read_ps( client, &obj->ps );

    return sprintf ( buf, "%d\n", obj->ps );
}

static ssize_t APDS9930_show_alsdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );
	int err = 0;
	err = APDS9930_read_als( client, &obj->als);

	if( err != APDS9930_SUCCESS)
	{
		APS_ERR("Can't access register of alsdata.");
	}
    return sprintf ( buf, "%d\n", obj->als );
}

static ssize_t APDS9930_show_luxdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );
    unsigned short data = 0;
	int err = 0;

	err = APDS9930_read_als( client, &obj->als);
	if( err != APDS9930_SUCCESS)
	{
		APS_ERR("Can't access register of alsdata.");
	}

	data = obj->als;

    return sprintf ( buf, "%d\n", data );
}

static ssize_t APDS9930_show_ps_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *data = i2c_get_clientdata ( client );
	int enable = test_bit(CMC_BIT_PS, &data->enable)?(1):(0);

    switch(enable) {
          case 0:
			  return sprintf ( buf, "%s\n", "Proximity Disabled");
          case 1:
               return sprintf ( buf, "%s\n", "Proximity Enabled" );

           default:
               return sprintf ( buf, "%s\n", "Proximity Error" );
	}
}

static ssize_t APDS9930_store_ps_enable ( struct device_driver *dev, const char *buf, size_t count )
{
	struct i2c_client *client = APDS9930_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret=0;

    switch(val) {
          case 0:
			  ret = APDS9930_enable_ps(client, 0);
            break;
          case 1:
			  ret = APDS9930_enable_ps(client, 1);
			break;

           default:
			break;
	}

	if ( ret < 0 )
		return ret;

	if (val)
		set_bit(CMC_BIT_PS, &APDS9930_obj->enable);

	else
		clear_bit(CMC_BIT_PS, &APDS9930_obj->enable);


	return count;
}

static ssize_t APDS9930_show_als_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *data = i2c_get_clientdata ( client );
	int enable = test_bit(CMC_BIT_ALS, &data->enable)?(1):(0);

    switch(enable) {
          case 0:
			  return sprintf ( buf, "%s\n", "Light Disabled");
          case 1:
               return sprintf ( buf, "%s\n", "Light Enabled" );

           default:
               return sprintf ( buf, "%s\n", "Light Error" );
	}
}

static ssize_t APDS9930_store_als_enable ( struct device_driver *dev, const char *buf, size_t count )
{
	struct i2c_client *client = APDS9930_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret=0;

    switch(val) {
          case 0:
            ret = APDS9930_enable_als( client, 0 );
            break;
          case 1:
            ret = APDS9930_enable_als( client, 1 );
            break;

           default:
			break;
	}

	if ( ret < 0 )
		return ret;

	if (val)
		set_bit(CMC_BIT_ALS, &APDS9930_obj->enable);

	else
		clear_bit(CMC_BIT_ALS, &APDS9930_obj->enable);


	return count;
}
// jin.joo 16/03/21 This func is to check Als c0, c1, gain through sysfs
static ssize_t  APDS9930_show_als_tunningdata(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );
	
	u16 c0_value, c1_value, als_gain;
	u8 buffer[2];
	int res = 0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0] = APDS9930_CMM_C0DATA_L;
	res = APDS9930_i2c_master_operate(obj->client, buffer, 0x201, I2C_FLAG_READ);

	if (res <= 0)
		goto EXIT_ERR;

	c0_value = buffer[0] | (buffer[1] << 8);


	buffer[0] = APDS9930_CMM_C1DATA_L;
	res = APDS9930_i2c_master_operate(obj->client, buffer, 0x201, I2C_FLAG_READ);

	if (res <= 0)
		goto EXIT_ERR;

	c1_value = buffer[0] | (buffer[1] << 8);


	buffer[0] = APDS9930_CMM_CONTROL;
	res = APDS9930_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);

	if (res <= 0)
		goto EXIT_ERR;

	als_gain = buffer[0] & 0x03;

#ifdef APS_DEBUG
	APS_ERR("APDS9930_show_als_tunningdata CONTROL register = 0x%x, als_gain = 0x%x\n", buffer[0], als_gain);
#endif
	return sprintf(buf, "c0:%d, c1:%d, gain:%dx\n", c0_value, c1_value, apds9930_als_again_tb[als_gain]);

 EXIT_ERR:
	APS_ERR("APDS9930_show_als_tunningdata fail\n");

	return res;
}

static ssize_t APDS9930_show_ps_crosstalk ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = APDS9930_i2c_client;
	struct APDS9930_priv *obj = i2c_get_clientdata ( client );

	APS_ERR("APDS9930_show_ps_crosstalk = %d\n", obj->ps_cali);

	return sprintf ( buf, "%d\n", obj->ps_cali );
}


static DRIVER_ATTR ( cali, S_IWUSR| S_IRUGO, APDS9930_show_cali, APDS9930_store_cali );
static DRIVER_ATTR ( ps_led, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_ps_led, APDS9930_store_ps_led );
static DRIVER_ATTR ( ps_pulse, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_ps_pulse, APDS9930_store_ps_pulse );
static DRIVER_ATTR ( pilt, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_pilt, APDS9930_store_pilt );
static DRIVER_ATTR ( piht, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_piht, APDS9930_store_piht );
static DRIVER_ATTR ( pdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_pdata, NULL );
static DRIVER_ATTR ( alsdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_alsdata, NULL );
static DRIVER_ATTR ( luxdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_luxdata, NULL );
static DRIVER_ATTR ( enable_ps, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_ps_enable, APDS9930_store_ps_enable );
static DRIVER_ATTR ( enable_als, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_als_enable, APDS9930_store_als_enable );
static DRIVER_ATTR ( als_enable, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_als_enable, APDS9930_store_als_enable );
static DRIVER_ATTR ( als_tunningdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, APDS9930_show_als_tunningdata, NULL ); //jin.joo temp code
static DRIVER_ATTR ( ps_crosstalk, S_IWUSR | S_IRUGO , APDS9930_show_ps_crosstalk, NULL );


static struct driver_attribute *apds9930_attr_list[] = {
	&driver_attr_cali,
	&driver_attr_ps_led,
	&driver_attr_ps_pulse,
	&driver_attr_pilt,
	&driver_attr_piht,
	&driver_attr_pdata,
	&driver_attr_alsdata,
	&driver_attr_luxdata,
	&driver_attr_enable_ps,
	&driver_attr_enable_als,
	&driver_attr_als_enable,
	&driver_attr_als_tunningdata,
	&driver_attr_ps_crosstalk,
};

static int APDS9930_create_attr ( struct device_driver *driver)
{
	int idx;
	int err = 0;
	int num = (int)(sizeof(apds9930_attr_list) / sizeof (apds9930_attr_list[0]));

	if( driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, apds9930_attr_list[idx]);
		if (err)
		{
			APS_ERR("driver_create_file ");
			break;
		}
	}

	return err;
}

static int APDS9930_delete_attr (struct device_driver *driver)
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( apds9930_attr_list ) / sizeof ( apds9930_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		driver_remove_file ( driver, apds9930_attr_list[idx] );
	}

	return err;
}


/******************************************************************************
 * Function Configuration
******************************************************************************/
static int APDS9930_open(struct inode *inode, struct file *file)
{
	file->private_data = APDS9930_i2c_client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int APDS9930_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}


/*----------------------------------------------------------------------------*/
static long APDS9930_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	long err = 0;
	void __user *ptr = (void __user *)arg;
	int dat;
	uint32_t enable;
//	int ps_result;
	uint32_t ps_cali;
	int threshold[2];

	switch (cmd) {
	case ALSPS_SET_PS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		if (enable) {
			err = APDS9930_enable_ps(obj->client, 1);
			if (err) {
				APS_ERR("enable ps fail: %ld\n", err);
				goto err_out;
			}

			set_bit(CMC_BIT_PS, &obj->enable);
		} else {
			err = APDS9930_enable_ps(obj->client, 0);
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
		err = APDS9930_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;


		dat = APDS9930_get_ps_value(obj, obj->ps);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_RAW_DATA:
		err = APDS9930_read_ps(obj->client, &obj->ps);
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
			err = APDS9930_enable_als(obj->client, 1);
			if (err) {
				APS_ERR("enable als fail: %ld\n", err);
				goto err_out;
			}
			set_bit(CMC_BIT_ALS, &obj->enable);
		} else {
			err = APDS9930_enable_als(obj->client, 0);
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
		err = APDS9930_read_als(obj->client, &obj->als);
		if (err)
			goto err_out;

		dat = obj->als;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_RAW_DATA:
		err = APDS9930_read_als(obj->client, &obj->als);
		if (err)
			goto err_out;


		dat = obj->als;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;
		/*----------------------------------for factory mode test---------------------------------------*/
/*	case ALSPS_GET_PS_TEST_RESULT:
		err = APDS9930_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;

		if (obj->ps > atomic_read(&obj->ps_thd_val_high))
			ps_result = 0;
		else
			ps_result = 1;

		if (copy_to_user(ptr, &ps_result, sizeof(ps_result))) {
			err = -EFAULT;
			goto err_out;
		}
		break;
*/
	case ALSPS_IOCTL_CLR_CALI:
		if (copy_from_user(&dat, ptr, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		if (dat == 0)
			obj->ps_cali = 0;
		break;

	case ALSPS_IOCTL_GET_CALI:
		APS_ERR ( "CMD = ALSPS_IOCTL_GET_CALI\n" );
		err = APDS9930_do_calibration(obj->client);
		if(err == 0)
		{
			ps_cali = obj->ps_cali;
			if (copy_to_user(ptr, &ps_cali, sizeof(ps_cali))) {
				err = -EFAULT;
				goto err_out;
			}
		}
		break;

	case ALSPS_IOCTL_SET_CALI:
		APS_ERR ( "CMD = ALSPS_IOCTL_SET_CALI\n" );

#if defined(CALIBRATION_TO_FILE)
		err = sensor_calibration_read(ID_PROXIMITY, &ps_cali);
		if (err != 0){
			APS_ERR("Read Cal Fail from file !!!\n");
			break;
		}
#else
		if (copy_from_user(&ps_cali, ptr, sizeof(ps_cali))) {
			err = -EFAULT;
			goto err_out;
		}
#endif

		if ( 0 == ps_cali || 65535 == ps_cali )
		{
			obj->ps_cali = PS_DEFAULT_CAL; // default calibration value
		}
		else
		{
			obj->ps_cali = ps_cali;
		}
		obj->hw->ps_threshold_high = obj->local_hw->far_offset + obj->ps_cali;
		obj->hw->ps_threshold_low = obj->local_hw->near_offset + obj->ps_cali;
		atomic_set(&obj->ps_thd_val_high,obj->hw->ps_threshold_high);
		atomic_set(&obj->ps_thd_val_low, obj->hw->ps_threshold_low);
		set_psensor_threshold(client, obj->hw->ps_threshold_low ,obj->hw->ps_threshold_high);
		break;
	case ALSPS_GET_DEVICEID:
        APS_LOG ( "CMD = ALSPS_GET_DEVICEID\n" );
		err = APDS9930_get_deivceid ( obj->client, (unsigned char*) &dat );
		if (err)
		{
			goto err_out;
		}
		if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
		{
			err = -EFAULT;
			goto err_out;
		}
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

		obj->hw->ps_threshold_high = threshold[0] + obj->ps_cali;
		obj->hw->ps_threshold_low = threshold[1] + obj->ps_cali;

		set_psensor_threshold(obj->client, obj->hw->ps_threshold_low,obj->hw->ps_threshold_high);

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

/*----------------------------------------------------------------------------*/
static const struct file_operations APDS9930_fops = {
	.owner = THIS_MODULE,
	.open = APDS9930_open,
	.release = APDS9930_release,
	.unlocked_ioctl = APDS9930_unlocked_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice APDS9930_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &APDS9930_fops,
};

/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void APDS9930_early_suspend(struct early_suspend *h)
{				/*early_suspend is only applied for ALS */
	struct APDS9930_priv *obj = container_of(h, struct APDS9930_priv, early_drv);
	int err;

	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return;
	}
#if 1
	atomic_set(&obj->als_suspend, 1);
	if (test_bit(CMC_BIT_ALS, &obj->enable)) {
		err = APDS9930_enable_als(obj->client, 0);
		if (err)
			APS_ERR("disable als fail: %d\n", err);

	}
#endif
}

/*----------------------------------------------------------------------------*/
static void APDS9930_late_resume(struct early_suspend *h)
{				/*early_suspend is only applied for ALS */
	struct APDS9930_priv *obj = container_of(h, struct APDS9930_priv, early_drv);
	int err;

	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return;
	}
#if 1
	atomic_set(&obj->als_suspend, 0);
	if (test_bit(CMC_BIT_ALS, &obj->enable)) {
		err = APDS9930_enable_als(obj->client, 1);
		if (err)
			APS_ERR("enable als fail: %d\n", err);

	}
#endif
}
#endif

/*----------------------------------------------------------------------------*/
//static int temp_als;
//static int ALS_FLAG;

int APDS9930_ps_operate(void *self, uint32_t command, void *buff_in, int size_in,
			void *buff_out, int size_out, int *actualout)
{
	int value;
	int err = 0;

	struct hwm_sensor_data *sensor_data;
	struct APDS9930_priv *obj = (struct APDS9930_priv *)self;

	APS_ERR("ps_operate : %d\n", command);
	/* APS_FUN(f); */
	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			APS_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		}
		/* Do nothing */
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			APS_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value) {
				err = APDS9930_enable_ps(obj->client, 1);
				if (err) {
					APS_ERR("enable ps fail: %d\n", err);
					return -1;
				}
				set_bit(CMC_BIT_PS, &obj->enable);
#if 0
				if (!test_bit(CMC_BIT_ALS, &obj->enable)) {
					ALS_FLAG = 1;
					err = APDS9930_enable_als(obj->client, 1);
					if (err) {
						APS_ERR("enable als fail: %d\n", err);
						return -1;
					}
				}
#endif
			} else {
				err = APDS9930_enable_ps(obj->client, 0);
				if (err) {
					APS_ERR("disable ps fail: %d\n", err);
					return -1;
				}
				clear_bit(CMC_BIT_PS, &obj->enable);
#if 0
				if (ALS_FLAG == 1) {
					err = APDS9930_enable_als(obj->client, 0);
					if (err) {
						APS_ERR("disable als fail: %d\n", err);
						return -1;
					}
					ALS_FLAG = 0;
				}
#endif
			}
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			APS_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			sensor_data = (struct hwm_sensor_data *) buff_out;
			APDS9930_read_ps(obj->client, &obj->ps);
			sensor_data->values[0] = APDS9930_get_ps_value(obj, obj->ps);
			sensor_data->value_divide = 1;
			sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		}
		break;
	default:
		APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}


int APDS9930_als_operate(void *self, uint32_t command, void *buff_in, int size_in,
			 void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *sensor_data;
	struct APDS9930_priv *obj = (struct APDS9930_priv *)self;

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			APS_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		}
		/* Do nothing */
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			APS_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value) {
				err = APDS9930_enable_als(obj->client, 1);
				if (err) {
					APS_ERR("enable als fail: %d\n", err);
					return -1;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			} else {
				err = APDS9930_enable_als(obj->client, 0);
				if (err) {
					APS_ERR("disable als fail: %d\n", err);
					return -1;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}

		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			APS_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			sensor_data = (struct hwm_sensor_data *) buff_out;
			/*yucong MTK add for fixing known issue */
			APDS9930_read_als(obj->client, &obj->als);
			sensor_data->values[0] = obj->als;
			sensor_data->value_divide = 1;
			sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		}
		break;
	default:
		APS_ERR("light sensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}


/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, APDS9930_DEV_NAME);
	return 0;
}

#if 0
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
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	APS_ERR("APDS9930_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (atomic_read(&APDS9930_obj->init_done)) {
		req.activate_req.sensorType = ID_LIGHT;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = en;
		len = sizeof(req.activate_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
	} else
		APS_ERR("sensor hub has not been ready!!\n");

	mutex_lock(&APDS9930_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &APDS9930_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &APDS9930_obj->enable);
	mutex_unlock(&APDS9930_mutex);
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	mutex_lock(&APDS9930_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &APDS9930_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &APDS9930_obj->enable);
	mutex_unlock(&APDS9930_mutex);
	if (!APDS9930_obj) {
		APS_ERR("APDS9930_obj is null!!\n");
		return -1;
	}
	res = APDS9930_enable_als(APDS9930_obj->client, en);
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */
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

static int als_get_data(int *value, int *status)
{
	int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else
	struct APDS9930_priv *obj = NULL;
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (atomic_read(&APDS9930_obj->init_done)) {
		req.get_data_req.sensorType = ID_LIGHT;
		req.get_data_req.action = SENSOR_HUB_GET_DATA;
		len = sizeof(req.get_data_req);
		err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err)
		APS_ERR("SCP_sensorHub_req_send fail!\n");
	else {
		*value = req.get_data_rsp.int16_Data[0];
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	if (atomic_read(&APDS9930_obj->trace) & CMC_TRC_PS_DATA)
		APS_ERR("value = %d\n", *value);
	else {
		APS_ERR("sensor hub hat not been ready!!\n");
		err = -1;
	}
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	if (!APDS9930_obj) {
		APS_ERR("APDS9930_obj is null!!\n");
		return -1;
	}
	obj = APDS9930_obj;
	err = APDS9930_read_als(obj->client, &obj->als);
	if (err)
		err = -1;
	else {
		*value = obj->als;
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	return err;
}

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int ps_enable_nodata(int en)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	APS_ERR("APDS9930_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (atomic_read(&APDS9930_obj->init_done)) {
		req.activate_req.sensorType = ID_PROXIMITY;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = en;
		len = sizeof(req.activate_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
	} else
		APS_ERR("sensor hub has not been ready!!\n");

	mutex_lock(&APDS9930_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &APDS9930_obj->enable);
	else
		clear_bit(CMC_BIT_PS, &APDS9930_obj->enable);
	mutex_unlock(&APDS9930_mutex);
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	mutex_lock(&APDS9930_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &APDS9930_obj->enable);

	else
		clear_bit(CMC_BIT_PS, &APDS9930_obj->enable);

	mutex_unlock(&APDS9930_mutex);
	if (!APDS9930_obj) {
		APS_ERR("APDS9930_obj is null!!\n");
		return -1;
	}
	res = APDS9930_enable_ps(APDS9930_obj->client, en);
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int *value, int *status)
{
	int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (atomic_read(&APDS9930_obj->init_done)) {
		req.get_data_req.sensorType = ID_PROXIMITY;
		req.get_data_req.action = SENSOR_HUB_GET_DATA;
		len = sizeof(req.get_data_req);
		err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err) {
		APS_ERR("SCP_sensorHub_req_send fail!\n");
		*value = -1;
		err = -1;
	} else {
		*value = req.get_data_rsp.int16_Data[0];
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	if (atomic_read(&APDS9930_obj->trace) & CMC_TRC_PS_DATA)
		APS_ERR("value = %d\n", *value)
	else {
		APS_ERR("sensor hub has not been ready!!\n");
		err = -1;
	}
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	if (!APDS9930_obj) {
		APS_ERR("APDS9930_obj is null!!\n");
		return -1;
	}

	err = APDS9930_read_ps(APDS9930_obj->client, &APDS9930_obj->ps);
	if (err)
		err = -1;
	else {
		*value = APDS9930_get_ps_value(APDS9930_obj, APDS9930_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	return err;
}

#endif
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct APDS9930_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
/*	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	struct ps_control_path ps_ctl = {0};
	struct ps_data_path ps_data = {0};*/
	int err = 0;

	APS_FUN();

	//APS_ERR("request_interrupt~~~\n");
	of_get_APDS9930_platform_data(&client->dev);
	/* configure the gpio pins */
	err = gpio_request_one(alsps_int_gpio_number, GPIOF_IN,
				 "alsps_int");
	if (err < 0) {
		APS_ERR("Unable to request gpio int_pin\n");
		return -1;
	}
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!(obj)) {
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	APDS9930_obj = obj;
	obj->hw = hw;
	obj->local_hw = proxi_hw;
	APDS9930_get_addr(obj->hw, &obj->addr);

	/*for interrupt work mode support -- by liaoxl.lenovo 12.08.2011 */
	INIT_WORK(&obj->irq_work, APDS9930_irq_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);
	atomic_set(&obj->als_debounce, 50);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val, 0xC1);
	atomic_set(&obj->ps_thd_val_high, obj->local_hw->far_offset);
	atomic_set(&obj->ps_thd_val_low, obj->local_hw->near_offset);
    obj->hw->ps_threshold_high = obj->local_hw->far_offset;
    obj->hw->ps_threshold_low = obj->local_hw->near_offset;
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->is_psat_enabled = false;
#ifdef CONFIG_LGE_SENSOR_DYNAMIC_AGAIN
    obj->als_reduce=0;
#endif
	obj->als_level_num = sizeof(obj->hw->als_level) / sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value) / sizeof(obj->hw->als_value[0]);
	obj->als_again_index = ALS_DEFAULT_GAIN;
	obj->als_atime_index = APDS9930_ALS_RES_10240;
	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 according to actual thing */
	/* (1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value */
	/* (400)/16*2.72 here is amplify *100 / *16 */
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	obj->ps_cali = 0;
	obj->ps_last_cali_status = PS_CALI_NONE;

	APDS9930_i2c_client = client;

	if (1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	} else {
		obj_ps.polling = 0;
	}

	err = APDS9930_init_client(client);
	if (err)
		goto exit_init_failed;
	APS_ERR("APDS9930_init_client() OK!\n");

	//irq register
	err = APDS9930_irq_registration(client);
	if (err != 0) {
		APS_ERR("registration failed: %d\n", err);
		goto exit_init_failed;
	}

	err = APDS9930_check_and_clear_intr(client);
	if (err) {
		APS_ERR("check/clear intr: %d\n", err);
		goto exit_init_failed;
	}


	err = misc_register(&APDS9930_device);
	if (err) {
		APS_ERR("APDS9930_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	//als_ctl.is_use_common_factory = false;
	//ps_ctl.is_use_common_factory = false;

	err = APDS9930_create_attr (&(APDS9930_init_info.platform_diver_addr->driver) );
	if (err)
	{
		APS_ERR("create_attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	obj_ps.self = APDS9930_obj;

	obj_ps.sensor_operate = APDS9930_ps_operate;
	err = hwmsen_attach(ID_PROXIMITY, &obj_ps);
	if (err) {
		APS_ERR("attach fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	obj_als.self = APDS9930_obj;
	if (1 == obj->hw->polling_mode_als)
	{
		obj_als.polling = 1;
	} else {
		obj_als.polling = 0;
	}
	obj_als.sensor_operate = APDS9930_als_operate;
	err = hwmsen_attach(ID_LIGHT, &obj_als);
	if (err) {
		APS_ERR("attach fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

#if defined(CONFIG_FB)
	obj->fb_notif.notifier_call = light_fb_notifier_callback;
	fb_register_client(&obj->fb_notif);
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
		obj->early_drv.suspend = APDS9930_early_suspend,
		obj->early_drv.resume = APDS9930_late_resume, register_early_suspend(&obj->early_drv);
#endif
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = obj->hw->is_batch_supported_als;
#endif
	APDS9930_init_flag = 0;
	APS_ERR("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
	misc_deregister(&APDS9930_device);
exit_misc_device_register_failed:
exit_init_failed:
	/* i2c_detach_client(client); */
	/* exit_kfree: */
	kfree(obj);
exit:
	gpio_free(alsps_int_gpio_number);
	APDS9930_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	APDS9930_init_flag = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_remove(struct i2c_client *client)
{
	int err;
	struct APDS9930_priv *obj = i2c_get_clientdata(client);

	err = misc_deregister(&APDS9930_device);
	if (err)
		APS_ERR("misc_deregister fail: %d\n", err);

#if defined(CONFIG_FB)
	fb_unregister_client(&obj->fb_notif);
#endif

	err = APDS9930_delete_attr (&(APDS9930_init_info.platform_diver_addr->driver));

	if (err)
	{
		APS_ERR("apds9930_delete_attr fail: %d\n", err);
	}
	gpio_free(alsps_int_gpio_number);
	APDS9930_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
static struct i2c_board_info __initdata i2c_APDS9930 = {I2C_BOARD_INFO(APDS9930_DEV_NAME, APDS9930_I2CADDR)};
/*----------------------------------------------------------------------------*/
static int  APDS9930_local_init(void)
{

	//APS_FUN();
#if defined(TARGET_MT6753_K7)
	on_boot=true;
	APDS9930_power(proxi_hw,hw, 1);
#endif
	i2c_register_board_info(hw->i2c_num, &i2c_APDS9930, 1);

	if (i2c_add_driver(&APDS9930_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}

	APS_ERR("init_flag %d\n", APDS9930_init_flag);

	if (-1 == APDS9930_init_flag)
		return -1;

	return 0;
}

/*----------------------------------------------------------------------------*/
static int APDS9930_remove(void)
{
	APS_FUN();
#if defined(TARGET_MT6753_K7)
	APDS9930_power(proxi_hw,hw, 0);
#endif
	i2c_del_driver(&APDS9930_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/

static int of_get_APDS9930_platform_data(struct device *dev)
{
	struct device_node *node = NULL;
	u32 ints[2] = { 0, 0 };

	node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		alsps_int_gpio_number = ints[0];

		alsps_irq = irq_of_parse_and_map(node, 0);
		if (alsps_irq < 0) {
			APS_ERR("alsps request_irq IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		APS_ERR("alsps_int_gpio_number %d; alsps_irq : %d\n", alsps_int_gpio_number, alsps_irq);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init APDS9930_init(void)
{
	const char *name ="mediatek,apds9930";

	APS_FUN();

	hw =get_alsps_dts_func(name, hw);
	if (!hw) {
		APS_ERR("get dts info fail\n");
		return -1;
	}

	proxi_hw = get_apds9930_dts_func(name, proxi_hw);
	if(!proxi_hw)
		APS_ERR("get local_hw dts info fail\n");

	alsps_driver_add(&APDS9930_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit APDS9930_exit(void)
{
	APS_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(APDS9930_init);
module_exit(APDS9930_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("APDS9930 driver");
MODULE_LICENSE("GPL");
