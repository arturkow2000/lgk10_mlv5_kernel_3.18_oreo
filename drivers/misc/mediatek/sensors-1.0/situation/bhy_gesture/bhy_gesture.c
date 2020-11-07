/*!
 * @section LICENSE
 * (C) Copyright 2011~2016 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 * VERSION: V2.0
 * Date: 2016/12/08
 */

//#include "bhy_gesture.h"

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
#include <linux/notifier.h>
#include <cust_acc.h>

#include <hwmsen_helper.h>
//#include <SCP_sensorHub.h>
#include <hwmsensor.h>
#include <sensors_io.h>

//#include "scp_helper.h"
#include "bhy_core.h"

#include "bhy_gesture.h"
#include "situation.h"
#include "bs_log.h"

#define BHY_MAX_RETRY_I2C_XFER		10
#define BHY_I2C_WRITE_DELAY_TIME	1000
#define BHY_I2C_MAX_BURST_WRITE_LEN	64

extern struct i2c_client *bmi160_acc_i2c_client;
extern struct bhy_client_data *obj_i2c_data;

extern int bhy_dump_hub_registers(char *buf, const char *line_hint);

struct acc_hw acc_cust;
//static struct acc_hw *hw = &acc_cust;

static struct bhy_gesture_i2c_data *obj_i2c_data_bhy_gesture;

static const struct i2c_device_id bhy_gesture_i2c_id[] = {
	{GESTUER_DEV_NAME, 0},
	{}
};

/*i2c traffic owned by acc, using acc's i2c channel for communication*/
extern int bhy_read_bus_reg(u8 reg, u8 *data, u16 len);
extern int bhy_write_bus_reg(u8 reg, u8 *data, u16 len);


int bhy_gesture_notify(void)
{
#ifdef CONFIG_LGE_PHONE_GESTURE
	return situation_notify(ID_PHONE_TILT_GESTURE);
#else
	return situation_notify(ID_WAKE_GESTURE);
#endif

}
EXPORT_SYMBOL(bhy_gesture_notify);


static int bhy_write_parameter(struct bhy_gesture_i2c_data *client_data,
		u8 page_num, u8 param_num, u8 *data, u8 len)
{
	int ret, ret2;
	int retry = BHY_PARAM_ACK_WAIT_RETRY;
	u8 param_num_mod, ack, u8_val;
	char dump[256 * 3];

	/* Write param data */
	ret = bhy_write_bus_reg( BHY_REG_LOAD_PARAM_0, data, len);
	if (ret < 0) {
		WAKEHUB_ERR("Write load parameter failed");
		goto bhy_write_parameter_exit;
	}
	/* Select page */
	ret = bhy_write_bus_reg( BHY_REG_PARAM_PAGE_SEL, &page_num, 1);
	if (ret < 0) {
		WAKEHUB_ERR("Write page request failed");
		goto bhy_write_parameter_exit;
	}
	/* Select param */
	param_num_mod = param_num | 0x80;
	ret = bhy_write_bus_reg( BHY_REG_PARAM_REQ, &param_num_mod, 1);
	if (ret < 0) {
		WAKEHUB_ERR("Write param request failed");
		goto bhy_write_parameter_exit;
	}
	/* Wait for ack */
	while (retry--) {
		ret = bhy_read_bus_reg( BHY_REG_PARAM_ACK, &ack, 1);
		if (ret < 0) {
			WAKEHUB_ERR("Read ack reg failed");
			goto bhy_write_parameter_exit;
		}
		if (ack == 0x80) {
			WAKEHUB_ERR("Param is not accepted");
			ret = -EINVAL;
			goto bhy_write_parameter_exit;
		}
		if (ack == param_num_mod)
			break;
		usleep_range(10000, 20000);
	}
	if (retry == -1) {
		WAKEHUB_ERR("Wait for ack failed[%d, %d]", page_num, param_num);
		ret = -EBUSY;
		goto bhy_write_parameter_exit;
	}
bhy_write_parameter_exit:
	if (ret < 0) {
		bhy_dump_hub_registers(dump, "BHY");
		PDEBUG("%s", dump);
	}
	/* Clear up */
	u8_val = 0;
	ret2 = bhy_write_bus_reg( BHY_REG_PARAM_PAGE_SEL, &u8_val, 1);
	if (ret2 < 0) {
		WAKEHUB_ERR("Write page sel failed on clear up");
		return ret2;
	}
	u8_val = 0;
	ret2 = bhy_write_bus_reg( BHY_REG_PARAM_REQ, &u8_val, 1);
	if (ret2 < 0) {
		WAKEHUB_ERR("Write param_req failed on clear up");
		return ret2;
	}
	retry = BHY_PARAM_ACK_WAIT_RETRY;
	while (retry--) {
		ret2 = bhy_read_bus_reg( BHY_REG_PARAM_ACK, &ack, 1);
		if (ret2 < 0) {
			WAKEHUB_ERR("Read ack reg failed");
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

static int bhy_set_sensor_conf(struct bhy_gesture_i2c_data *client_data,
	int handle, u8 *conf)
{
	int i;
	__le16 swap_data;
	u8 data[8];
	int ret;
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
		printk("Write parameter error");
		return ret;
	}
	printk("Set sensor[%d] conf: %02X %02X %02X %02X %02X %02X %02X %02X",
		handle, data[0], data[1], data[2], data[3], data[4], data[5],
		data[6], data[7]);

	return 0;
}

/*
static int bhy_gesture_recv_data(struct data_unit_t *event, void *reserved)
{
	WAKEHUB_LOG("bhy_gesture_recv_data start.\n");
	if (event->flush_action == FLUSH_ACTION)
		WAKEHUB_ERR("wake_gesture do not support flush.\n");
	else if (event->flush_action == DATA_ACTION)
		situation_notify(wake_gesture);
	return 0;
}
*/

static int bhy_gesture_get_data(int *probability, int *status)
{
	return 0;
}

static int bhy_gesture_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;
	int err = 0;
	u8 conf[8];
	u16 sample_delay = 0;
	value = (int)samplingPeriodNs/1000/1000;
	if (value <= 5)
		sample_delay = 400;
	else if (value <= 10)
		sample_delay = 200;
	else
		sample_delay = 100;

	memcpy(conf, &sample_delay, 2);
	memset(conf + 2, 0, 2);
	memset(conf + 4, 0, 4);

	err = bhy_set_sensor_conf(obj_i2c_data_bhy_gesture, BHY_SENSOR_HANDLE_WAKE_GESTURE, conf);
	if (err < 0) {
		WAKEHUB_ERR("set delay parameter error!\n");
		return err;
	}

	return 0;
}

/**
 * enable/disable gesture
 */
static int bhy_gesture_open_report_data(int open)
{
	int err = 0;
	u8 conf[8];
	u16 tmp;

	WAKEHUB_LOG("%s : enable=%d\n", __func__, open);
	tmp = (u16)open;
	memcpy(conf, &tmp, 2);
	memset(conf + 2, 0, 2);
	memset(conf + 4, 0, 4);
	err = bhy_set_sensor_conf(obj_i2c_data_bhy_gesture, BHY_SENSOR_HANDLE_WAKE_GESTURE, conf);
	if (err < 0) {
		WAKEHUB_ERR("enable bhy gesture failed.\n");
		return err;
	}
	return err;
}


static int wakehub_local_init(void)
{
	/*
	if(i2c_add_driver(&bhy_gesture_i2c_driver)) {
		WAKEHUB_ERR("add driver error.\n");
		return -1;
	}
	*/
	struct situation_control_path ctl = {0};
	struct situation_data_path data = {0};
	int err = 0;
	struct bhy_gesture_i2c_data *obj;

	WAKEHUB_LOG("bhy wakehub_local_init start.\n");
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	obj_i2c_data_bhy_gesture = obj;

	mutex_init(&obj->mutex_bus_op);

	ctl.open_report_data = bhy_gesture_open_report_data;
	ctl.batch = bhy_gesture_batch;
	ctl.is_support_batch = false;
	ctl.is_support_wake_lock = false;

#ifdef CONFIG_LGE_PHONE_GESTURE
	err = situation_register_control_path(&ctl, ID_PHONE_TILT_GESTURE);
#else
	err = situation_register_control_path(&ctl, ID_WAKE_GESTURE);
#endif

	if (err) {
		WAKEHUB_ERR("register wake_gesture control path err\n");
		goto exit;
	}

	data.get_data = bhy_gesture_get_data;

#ifdef CONFIG_LGE_PHONE_GESTURE	
	err = situation_register_data_path(&data, ID_PHONE_TILT_GESTURE);
#else
	err = situation_register_data_path(&data, ID_WAKE_GESTURE);
#endif

	if (err) {
		WAKEHUB_ERR("register wake_gesture data path err\n");
		goto exit;
	}
	WAKEHUB_LOG("bhy_gesture_i2c_probe done.\n");

	return 0;
exit:
	return -1;

	
}

static int wakehub_local_uninit(void)
{
	return 0;
}

static struct situation_init_info wakehub_init_info = {
	.name = "bhy_gesture",
	.init = wakehub_local_init,
	.uninit = wakehub_local_uninit,
};

static int __init wakehub_init(void)
{
	/*
	hw = get_accel_dts_func(COMPATIABLE_NAME, hw);
	if (!hw) {
		WAKEHUB_ERR("get dts information failed.\n");
		return -1;
	}*/
	WAKEHUB_ERR("bhy wakehub_init\n");
	
#ifdef CONFIG_LGE_PHONE_GESTURE
	situation_driver_add(&wakehub_init_info, ID_PHONE_TILT_GESTURE);
#else
	situation_driver_add(&wakehub_init_info, ID_WAKE_GESTURE);
#endif

	return 0;
}

static void __exit wakehub_exit(void)
{
	WAKEHUB_FUN();
}

module_init(wakehub_init);
module_exit(wakehub_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BHY GESTURE I2C Driver");
MODULE_AUTHOR("contact@bosch-sensortec.com");
