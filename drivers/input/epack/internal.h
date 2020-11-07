/***************************************************************************
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
 *    File	:
 *    Author(s)   :
 *    Description :
 *
 ***************************************************************************/
#ifndef __EPACK_INTERNAL_H__
#define __EPACK_INTERNAL_H__

/****************************************************************************
* Nested Include Files
****************************************************************************/
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/input/epack.h>

/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/
#define EPACK_PACKET_SIZE	64

/* for upgrading firmware*/
#define CMD_GET_FWVER			0xA6
#define CMD_GET_HWVER			0xA7
#define CMD_GET_FLASHMODE 	0xCA
#define CMD_RUN_LDROM			0xAC
#define CMD_UPDATE_DATE_FLASH	0xC3
#define CMD_WRITE_CHECKSUM 	0xC9
#define CMD_RUN_APROM			0xAB

/* for monitoring */
#define CMD_GET_BAT_VOLTAGE	0xD1
#define CMD_GET_BAT_TEMP		0xD3
#define CMD_GET_CHG_STATUS	0xD5
#define CMD_GET_FAULT_STATUS	0xD7
#define CMD_GET_OUTPUT_STATUS	0xD9

/* for vbus control */
#define CMD_VBUS_CONTROL 	0xDB

/* Audio */
#define EPACK_AUD_ENABLE (1)
#define EPACK_AUD_DISABLE (0)

/**********************************************************
* Driver Capability
**********************************************************/

/****************************************************************************
* Type Definitions
****************************************************************************/
enum {
	EPACK_TRIGGER_EPID = 0x01,
	EPACK_TRIGGER_VINAVA = 0x02,
	EPACK_TRIGGER_VCHGDET = 0x04,
	EPACK_TRIGGER_FIRMWARE = 0x08,
	EPACK_TRIGGER_MONITOR = 0x10,
	EPACK_TRIGGER_PMIC = 0x20,
	EPACK_TRIGGER_INIT = 0x40,
};

struct epack {
	struct i2c_client *client;
	struct device *dev;

	struct mutex i2c_rw_lock;

#ifdef CONFIG_MTK_I2C_EXTENSION
	dma_addr_t i2c_dma;
	u8 *i2c_buf;
#endif

	struct delayed_work upgrade;
	struct delayed_work update;
	struct delayed_work monitor;
	struct delayed_work init;

	unsigned int irq_epid;
	unsigned int irq_vinava;
	unsigned int irq_vchgdet;

	int gpio_epid;
	int gpio_vinava;
	int gpio_vchgdet;

	int epid_status;
	int vinava_status;
	int vchgdet_status;

	struct pinctrl *pin;

	int usb_path;
	int pwr_path;

	bool is_present;
	bool need_checking;
	int pwr_status;
	int trigger_src;

	struct notifier_block usb_nb;
	int otgdev_cnt;
	int umsdev_cnt;
	int otg_device;

	struct switch_dev aud_state;

	int fw_status;
	char fw_path[256];
	unsigned int fw_ver;
	unsigned int hw_ver;

	struct dentry *debugfs;
};


/****************************************************************************
* Exported Variables
****************************************************************************/

/****************************************************************************
* Macros
****************************************************************************/

/****************************************************************************
* Global Function Prototypes
****************************************************************************/
extern int epack_read_dbg_info(struct epack *ep, u16 *volt, u16 *temp, u16 *chg, u16 *fault, u16 *output);
extern bool epack_need_fw_upgrade(struct epack *ep);
extern int epack_do_fw_upgrade(struct epack *, char *path);
extern int epack_fw_init(struct epack *);
extern int epack_control_vbus(struct epack *ep, int enable);


#endif /* __EPACK_INTERNAL_H__ */

/* End Of File */
