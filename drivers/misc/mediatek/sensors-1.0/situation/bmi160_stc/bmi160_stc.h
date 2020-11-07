/*!
 * @section LICENSE
 * (C) Copyright 2017 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 * VERSION: V2.0
 * Date: 2017/09/19
 */

#ifndef BMI160_STC_H
#define BMI160_STC_H

#include <linux/ioctl.h>

#define GESTUER_DEV_NAME		"bmi160_stc"

extern struct i2c_client *bmi160_acc_i2c_client;

struct bmi160_stc_i2c_data {
	struct i2c_client *client;
	struct mutex mutex_bus_op;
};

#define BMI160_STC_TAG                  "[bmi160_stc] "
#define BMI160_STC_FUN(f)               printk(BMI160_STC_TAG"%s\n", __func__)
#define BMI160_STC_ERR(fmt, args...)    printk(BMI160_STC_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define BMI160_STC_LOG(fmt, args...)    printk(BMI160_STC_TAG fmt, ##args)

#endif/* BMI160_STC_H */
