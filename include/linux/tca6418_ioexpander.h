/* include/linux/i2c/tca6418_ioexpander.h
 *
 * Copyright (C) 2009 HTC Corporation.
 * Author: htc
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

#define FINGER_LDO_EN      1
#define TDMB_FM_SW         2
#define HIFI_MODE2         3
#define VT_CAM_VIO_EN      4
#define NFC_MODE           6
#define FLASH_CNTL_EN      7
#define DMB_EN             8
#define CAM_AVDD_LDO_EN    10
#define HIFI_LDO_SW        11
#define VT_CAM_DVDD_EN     12
#define SPK_AMP_EN         13
#define LCD_LDO_EN         15
#define HIFI_SW_SEL        16
#define VCONN_BOOST_EN     17

enum {
	IOEXP_INPUT,
	IOEXP_OUTPUT,
};

enum {
	IOEXP_PULLDOWN,
	IOEXP_NOPULL,
};

#ifndef _LINUX_ATMEGA_MICROP_H
#define _LINUX_ATMEGA_MICROP_H

int ioexp_gpio_set_value(uint8_t gpio, uint8_t value);
int ioexp_gpio_get_value(uint8_t gpio);
int ioexp_gpio_get_direction(uint8_t gpio);
int ioexp_read_gpio_status(uint8_t *data);
int ioexp_gpio_set_cfg(uint8_t gpio, uint8_t direction, uint8_t pulldown_disable, uint8_t level);
void ioexp_print_gpio_status(void);

#endif

