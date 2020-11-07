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

#ifndef BHY_GESTURE_H
#define BHY_GESTURE_H

#include <linux/ioctl.h>

#define GESTUER_DEV_NAME		"bhy_gesture"

extern struct i2c_client *bhy_bmi160_acc_i2c_client;

/*
struct bhy_data_bus {
	struct device *dev;
	s32(*read)(struct device *dev, u8 reg, u8 *data, u16 len);
	s32(*write)(struct device *dev, u8 reg, const u8 *data, u16 len);
	int irq;
	int bus_type;
};
*/

struct bhy_gesture_i2c_data {
	struct i2c_client *client;
	struct mutex mutex_bus_op;
	struct bhy_data_bus data_bus;
};
#define WAKEHUB_TAG                  "[bhy] "
#define WAKEHUB_FUN(f)               printk(WAKEHUB_TAG"%s\n", __func__)
#define WAKEHUB_ERR(fmt, args...)    printk(WAKEHUB_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define WAKEHUB_LOG(fmt, args...)    printk(WAKEHUB_TAG fmt, ##args)

#define BHY_SENSOR_HANDLE_WAKE_GESTURE	55

/* sensor page */
#define BHY_PAGE_SENSOR	3
#define BHY_PARAM_SENSOR_INFO_0	0
#define BHY_PARAM_SENSOR_CONF_0	64

#define BHY_PARAM_ACK_WAIT_RETRY	100
#define BHY_REG_LOAD_PARAM_0	0x5C	/* 0x5C through 0x63 */
#define BHY_REG_PARAM_PAGE_SEL	0x54
#define BHY_REG_PARAM_REQ	0x64
#define BHY_REG_PARAM_ACK	0x3A

#endif/* BHY_GESTURE_H */
