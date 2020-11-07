/* Goodix's GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF816M/GF3208/GF5216
 *  fingerprint sensor linux driver for TEE
 *
 * 2010 - 2015 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef __GF_SPI_TEE_H
#define __GF_SPI_TEE_H

#include <linux/types.h>
#include <linux/netlink.h>
#include <linux/cdev.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include "mt_spi.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

/**************************debug******************************/
#define ERR_LOG  (0)
#define INFO_LOG (1)
#define DEBUG_LOG (2)


#define gf_debug(level, fmt, args...) do { \
	if (g_debug_level >= level) {\
		printk(KERN_INFO "[gf] " fmt, ##args); \
	} \
} while (0)

#define FUNC_ENTRY() gf_debug(DEBUG_LOG, "%s: enter\n", __func__)
#define FUNC_EXIT()  gf_debug(DEBUG_LOG, "%s: exit\n", __func__)

/**********************IO Magic**********************/
#define GF_IOC_MAGIC	'g'

#define GF_NAV_INPUT_UP			KEY_UP
#define GF_NAV_INPUT_DOWN		KEY_DOWN
#define GF_NAV_INPUT_LEFT		KEY_LEFT
#define GF_NAV_INPUT_RIGHT		KEY_RIGHT
#define GF_NAV_INPUT_CLICK		KEY_VOLUMEDOWN
#define GF_NAV_INPUT_DOUBLE_CLICK	KEY_VOLUMEUP
#define GF_NAV_INPUT_LONG_PRESS	KEY_SEARCH
#define GF_NAV_INPUT_HEAVY		KEY_CHAT

#define GF_KEY_INPUT_HOME		KEY_HOMEPAGE
#define GF_KEY_INPUT_MENU		KEY_MENU
#define GF_KEY_INPUT_BACK		KEY_BACK
#define GF_KEY_INPUT_POWER		KEY_POWER
#define GF_KEY_INPUT_CAMERA		KEY_CAMERA

typedef enum gf_nav_event {
	GF_NAV_NONE = 0,
	GF_NAV_FINGER_UP,
	GF_NAV_FINGER_DOWN,
	GF_NAV_UP,
	GF_NAV_DOWN,
	GF_NAV_LEFT,
	GF_NAV_RIGHT,
	GF_NAV_CLICK,
	GF_NAV_HEAVY,
	GF_NAV_LONG_PRESS,
	GF_NAV_DOUBLE_CLICK,
} gf_nav_event_t;

typedef enum gf_key_event {
	GF_KEY_NONE = 0,
	GF_KEY_HOME,
	GF_KEY_POWER,
	GF_KEY_MENU,
	GF_KEY_BACK,
	GF_KEY_CAMERA,
} gf_key_event_t;

struct gf_key {
	enum gf_key_event key;
	uint32_t value;   /* key down = 1, key up = 0 */
};

enum gf_netlink_cmd {
	GF_NETLINK_TEST = 0,
	GF_NETLINK_IRQ = 1,
	GF_NETLINK_SCREEN_OFF,
	GF_NETLINK_SCREEN_ON
};

struct gf_ioc_transfer {
	u8 cmd;    /* spi read = 0, spi  write = 1 */
	u8 reserved;
	u16 addr;
	u32 len;
	u8 *buf;
};

struct gf_ioc_transfer_raw {
    u32 len;
    u8 *read_buf;
    u8 *write_buf;
    uint32_t high_time;
    uint32_t low_time;
};

struct gf_ioc_chip_info {
	u8 vendor_id;
	u8 mode;
	u8 operation;
	u8 reserved[5];
};

/* define for gf_spi_cfg_t->speed_hz */
#define GF_SPI_SPEED_LOW 1
#define GF_SPI_SPEED_MEDIUM 6
#define GF_SPI_SPEED_HIGH 9

enum gf_spi_cpol {
    GF_SPI_CPOL_0,
    GF_SPI_CPOL_1
};

enum gf_spi_cpha {
    GF_SPI_CPHA_0,
    GF_SPI_CPHA_1
};

typedef struct {
    unsigned int cs_setuptime;
    unsigned int cs_holdtime;
    unsigned int cs_idletime;
    unsigned int speed_hz; //spi clock rate
    unsigned int duty_cycle; //The time ratio of active level in a period. Default value is 50. that means high time and low time is same.
    enum gf_spi_cpol cpol;
    enum gf_spi_cpol cpha;
} gf_spi_cfg_t;

struct mt_spi_t {
	struct platform_device *pdev;
	void __iomem *regs;
	int irq;
	int running;
	u32 pad_macro;
	struct wake_lock wk_lock;
	struct mt_chip_conf *config;
	struct spi_master *master;

	struct spi_transfer *cur_transfer;
	struct spi_transfer *next_transfer;

	spinlock_t lock;
	struct list_head queue;
#if !defined(CONFIG_MTK_CLKMGR)
	struct clk *clk_main;   /* main clock for spi bus */
#endif                          /* !defined(CONFIG_MTK_LEGACY) */
};

// need to sync up gf_hal_device.c
/* define commands */
#define GF_IOC_INIT             _IOR(GF_IOC_MAGIC, 0, u8)
#define GF_IOC_EXIT             _IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET            _IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ       _IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ		_IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_SPI_CLK   _IOW(GF_IOC_MAGIC, 5, uint32_t)
#define GF_IOC_DISABLE_SPI_CLK  _IO(GF_IOC_MAGIC, 6)
#define GF_IOC_ENABLE_POWER     _IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER    _IO(GF_IOC_MAGIC, 8)
#define GF_IOC_INPUT_KEY_EVENT  _IOW(GF_IOC_MAGIC, 9, struct gf_key)
/* fp chip has change to sleep mode while screen off */
#define GF_IOC_ENTER_SLEEP_MODE _IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO      _IOR(GF_IOC_MAGIC, 11, u8)
#define GF_IOC_REMOVE           _IO(GF_IOC_MAGIC, 12)
#define GF_IOC_CHIP_INFO        _IOW(GF_IOC_MAGIC, 13, struct gf_ioc_chip_info)
#define GF_IOC_NAV_EVENT        _IOW(GF_IOC_MAGIC, 14, gf_nav_event_t)
/* for SPI REE transfer */
#define GF_IOC_TRANSFER_CMD     _IOWR(GF_IOC_MAGIC, 15, struct gf_ioc_transfer)
#define GF_IOC_TRANSFER_RAW_CMD _IOWR(GF_IOC_MAGIC, 16, struct gf_ioc_transfer_raw)
#define GF_IOC_SPI_INIT_CFG_CMD _IOW(GF_IOC_MAGIC, 17, gf_spi_cfg_t)

#define  GF_IOC_MAXNR    18  /* THIS MACRO IS NOT USED NOW... */

struct gf_device {
	dev_t devno;
	struct cdev cdev;
	struct device *device;
	struct class *class;
	struct spi_device *spi;
	int device_count;
	struct mt_chip_conf spi_mcc;
	struct regulator *reg;
	spinlock_t spi_lock;
	struct list_head device_entry;

	struct input_dev *input;

	u8 *spi_buffer;  /* only used for SPI transfer internal */
	struct mutex buf_lock;
	struct mutex release_lock;
	u8 buf_status;

	/* for netlink use */
	struct sock *nl_sk;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#else
	struct notifier_block notifier;
#endif

	u8 probe_finish;
	u8 irq_count;

	/* bit24-bit32 of signal count */
	/* bit16-bit23 of event type, 1: key down; 2: key up; 3: fp data ready; 4: home key */
	/* bit0-bit15 of event type, buffer status register */
	u32 event_type;
	u8 sig_count;
	u8 is_sleep_mode;
	u8 system_status;

	u32 cs_gpio;
	u32 reset_gpio;
	u32 irq_gpio;
	u32 irq_num;
	u8  need_update;
	u32 irq;
	int netlink_route;

#ifdef CONFIG_OF
	struct pinctrl *pinctrl_gpios;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_miso_spi, *pins_miso_pullhigh, *pins_miso_pulllow;
	struct pinctrl_state *pins_reset_high, *pins_reset_low;
	struct pinctrl_state *pins_mosi_spi, *pins_mosi_pullhigh, *pins_mosi_pulllow;
	struct pinctrl_state *pins_cs_spi,*pins_cs_pullhigh, *pins_cs_pulllow;
	struct pinctrl_state *pins_clk_spi, *pins_clk_pullhigh, *pins_clk_pulllow;
	struct pinctrl_state *pins_ldo_high, *pins_ldo_low;
#endif
};

/**************************REE SPI******************************/
#ifndef SUPPORT_REE_SPI
#define SUPPORT_REE_SPI
#endif

#ifdef SUPPORT_REE_SPI
#define HIGH_SPEED 6
#define LOW_SPEED  1

#define ERR_NO_SENSOR    111
#define ERR_FW_DESTROY   112
#define ERR_PREPARE_FAIL 113

#define GF32XX_REG_CHIP_ID_LO  0x0000
#define GF32XX_REG_CHIP_ID_HI  0x0002
#define GF3208_REG_IRQ_CTRL0   0x0120
#define GF3208_REG_IRQ_CLEAR   0x0124
#define GF3208_REG_IRQ_STATUS  0x0126
#define GF3208_REG_IRQ_CTRL4   0x0128

#define GF3208_RST_DATA (0x0100)
#define GF3228_RST_DATA (0x0800)

#define GF3208_CHIP_ID  (0x002202)
#define GF3228_CHIP_ID  (0x00220A)
#define GF3258_CHIP_ID  (0x00220D)

/**********************function defination**********************/
void gf_spi_setup_conf_ree(struct gf_device *gf_dev, u32 speed, enum spi_transfer_mode mode);
int gf_spi_read_bytes_ree(struct gf_device *gf_dev, u16 addr, u32 data_len, u8 *rx_buf);
int gf_spi_write_bytes_ree(struct gf_device *gf_dev, u16 addr, u32 data_len, u8 *tx_buf);
int gf_spi_read_byte_ree(struct gf_device *gf_dev, u16 addr, u8 *value);
int gf_spi_write_byte_ree(struct gf_device *gf_dev, u16 addr, u8 value);
int gf_ioctl_transfer_raw_cmd(struct gf_device *gf_dev, unsigned long arg,unsigned int bufsiz);
int gf_ioctl_spi_init_cfg_cmd(struct mt_chip_conf *mcc, unsigned long arg);

#endif

#endif	/* __GF_SPI_TEE_H */
