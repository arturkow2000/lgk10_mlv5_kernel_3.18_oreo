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
#include <hwmsensor.h>
#include <sensors_io.h>

#include "bmi160_stc.h"
#include "situation.h"

extern struct i2c_client *bmi160_acc_i2c_client;
extern struct bmi160_stc_client_data *obj_i2c_data;

extern int bmi160_set_step_enable(int);

static struct bmi160_stc_i2c_data *obj_i2c_data_bmi160_stc;

int bmi160_stc_notify(void)
{
	return situation_notify(ID_SLOPE_DETECTOR);
}
EXPORT_SYMBOL(bmi160_stc_notify);


static int bmi160_stc_get_data(int *probability, int *status)
{
	return 0;
}

static int bmi160_stc_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{

	return 0;
}

/**
 * enable/disable gesture
 */
static int bmi160_stc_open_report_data(int open)
{
	int err = 0;

	BMI160_STC_LOG("%s : enable=%d\n", __func__, open);

	mutex_lock(&obj_i2c_data_bmi160_stc->mutex_bus_op);
	err = bmi160_set_step_enable(open);
	mutex_unlock(&obj_i2c_data_bmi160_stc->mutex_bus_op);

	return err;
}


static int bmi160_stc_local_init(void)
{
	struct situation_control_path ctl = {0};
	struct situation_data_path data = {0};
	int err = 0;
	struct bmi160_stc_i2c_data *obj;

	BMI160_STC_LOG("bmi160_stc_local_init start.\n");
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	obj_i2c_data_bmi160_stc = obj;

	mutex_init(&obj->mutex_bus_op);

	ctl.open_report_data = bmi160_stc_open_report_data;
	ctl.batch = bmi160_stc_batch;
	ctl.is_support_batch = false;
	ctl.is_support_wake_lock = false;

	err = situation_register_control_path(&ctl, ID_SLOPE_DETECTOR);

	if (err) {
		BMI160_STC_ERR("register bmi160_stc control path err\n");
		goto exit;
	}

	data.get_data = bmi160_stc_get_data;

	err = situation_register_data_path(&data, ID_SLOPE_DETECTOR);

	if (err) {
		BMI160_STC_ERR("register bmi160_stc data path err\n");
		goto exit;
	}
	BMI160_STC_LOG("bmi160_stc_i2c_probe done.\n");

	return 0;
exit:
	return -1;
}

static int bmi160_stc_local_uninit(void)
{
	return 0;
}

static struct situation_init_info bmi160_stc_init_info = {
	.name = "bmi160_stc",
	.init = bmi160_stc_local_init,
	.uninit = bmi160_stc_local_uninit,
};

static int __init bmi160_stc_init(void)
{
	BMI160_STC_ERR("bmi160_stc_init\n");

	situation_driver_add(&bmi160_stc_init_info, ID_SLOPE_DETECTOR);
	return 0;
}

static void __exit bmi160_stc_exit(void)
{
	BMI160_STC_FUN();
}

module_init(bmi160_stc_init);
module_exit(bmi160_stc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMI160 STC I2C Driver");
MODULE_AUTHOR("contact@bosch-sensortec.com");
