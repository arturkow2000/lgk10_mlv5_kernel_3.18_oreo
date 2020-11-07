/* Goodix's GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF816M/GF3208/GF5206/GF5216/GF5208
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

#include <linux/device.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif

#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/async.h>

/* MTK header */
#include "mt_spi.h"
#include "mt_spi_hal.h"
#include "mt_gpio.h"
#include "mach/gpio_const.h"

#include "gf_spi_tee.h"

/**************************defination******************************/
#define GF_DEV_NAME "goodix_fp"
#define GF_DEV_MAJOR 0	/* assigned */

#define GF_CLASS_NAME "goodix_fp"
#define GF_INPUT_NAME "gf-keys"

#define GF_LINUX_VERSION "V1.01.04"

//#define GF_NETLINK_ROUTE 29   /* for GF test temporary, need defined in include/uapi/linux/netlink.h */
#define MAX_NL_MSG_LEN 16

#define MAX_RETRY_HW_CHECK_COUNT 5

#define USES_FPS_WAKE_UP_ENABLE

/*************************************************************/
/* debug log setting */
//static const u8 g_debug_level =   ERR_LOG; // 0
static const u8 g_debug_level =  INFO_LOG; // 1
//static const u8 g_debug_level = DEBUG_LOG; // 2

/* align=2, 2 bytes align */
/* align=4, 4 bytes align */
/* align=8, 8 bytes align */
#define ROUND_UP(x, align)		((x+(align-1))&~(align-1))

/*************************************************************/
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned int bufsiz = (25 * 1024);
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "maximum data bytes for SPI message");

#ifdef CONFIG_OF
static const struct of_device_id gf_of_match[] = {
	{ .compatible = "mediatek,goodix-fp", },
	{ .compatible = "goodix,goodix-fp", },
	{},
};
MODULE_DEVICE_TABLE(of, gf_of_match);
#endif

/* for netlink use */
static int pid = 0;

static u8 g_vendor_id = 0;

static ssize_t gf_debug_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t gf_debug_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, gf_debug_show, gf_debug_store);

static struct attribute *gf_debug_attrs[] = {
	&dev_attr_debug.attr,
	NULL
};

static const struct attribute_group gf_debug_attr_group = {
	.attrs = gf_debug_attrs,
	.name = "debug"
};

const struct mt_chip_conf spi_ctrdata = {
	.setuptime = 10,
	.holdtime = 10,
	.high_time = 50, /* 1MHz */
	.low_time = 50,
	.cs_idletime = 10,
	.ulthgh_thrsh = 0,

	.cpol = SPI_CPOL_0,
	.cpha = SPI_CPHA_0,

	.rx_mlsb = SPI_MSB,
	.tx_mlsb = SPI_MSB,

	.tx_endian = SPI_LENDIAN,
	.rx_endian = SPI_LENDIAN,

	.com_mod = FIFO_TRANSFER,
	/* .com_mod = DMA_TRANSFER, */

	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

#ifdef USES_FPS_WAKE_UP_ENABLE
#define TTW_HOLD_TIME 1000
static struct wake_lock ttw_wl;
#endif

/* -------------------------------------------------------------------- */
/* timer function								*/
/* -------------------------------------------------------------------- */
#define TIME_START	   0
#define TIME_STOP	   1

#if 0
static long int prev_time, cur_time;

static long int kernel_time(unsigned int step)
{
	cur_time = ktime_to_us(ktime_get());
	if (step == TIME_START) {
		prev_time = cur_time;
		return 0;
	} else if (step == TIME_STOP) {
		gf_debug(DEBUG_LOG, "%s: use: %ld us\n", __func__, (cur_time - prev_time));
		return cur_time - prev_time;
	}
	prev_time = cur_time;
	return -1;
}
#endif

/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration								  */
/* -------------------------------------------------------------------- */
static int gf_get_sensor_dts_info(struct gf_device *gf_dev)
{
#ifdef CONFIG_OF
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;
	int ret;
	int value;

	FUNC_ENTRY();

	// get node from device tree
	node = of_find_compatible_node(NULL, NULL, "mediatek,goodix-fp");
	if (node == NULL) {
		gf_debug(ERR_LOG, "%s failed to get dts node!\n", __func__);
		return -ENODEV;
	}

	// get gpio pin from device tree
	of_property_read_u32(node, "cs_gpio", &value);
	gf_debug(INFO_LOG, "%s: get cs_gpio [%d] from dts\n", __func__, value);
	gf_dev->cs_gpio = value;

	of_property_read_u32(node, "reset_gpio", &value);
	gf_debug(INFO_LOG, "%s: get reset_gpio [%d] from dts\n", __func__, value);
	gf_dev->reset_gpio = value;

	of_property_read_u32(node, "irq_gpio", &value);
	gf_debug(INFO_LOG, "%s: get irq_gpio [%d] from dts\n", __func__, value);
	gf_dev->irq_gpio = value;

	of_property_read_u32(node, "netlink-event", &value);
	gf_debug(INFO_LOG, "%s: get netlink event [%d] from dts\n", __func__, value);
	gf_dev->netlink_route = value;

	// bind pinctrl
	pdev = of_find_device_by_node(node);
	if (pdev == NULL) {
		gf_debug(ERR_LOG, "%s platform device is null\n", __func__);
		return -ENODEV;
	}

	gf_dev->pinctrl_gpios = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gf_dev->pinctrl_gpios)) {
		ret = PTR_ERR(gf_dev->pinctrl_gpios);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl\n", __func__);
		return ret;
	}

	/* it's normal that get "default" will failed */
	gf_dev->pins_default = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "default");
	if (IS_ERR(gf_dev->pins_default)) {
		ret = PTR_ERR(gf_dev->pins_default);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl default\n", __func__);
		return ret;
	}

	gf_dev->pins_miso_spi = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "miso_spi");
	if (IS_ERR(gf_dev->pins_miso_spi)) {
		ret = PTR_ERR(gf_dev->pins_miso_spi);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl miso_spi\n", __func__);
		return ret;
	}

	gf_dev->pins_miso_pullhigh = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "miso_pullhigh");
	if (IS_ERR(gf_dev->pins_miso_pullhigh)) {
		ret = PTR_ERR(gf_dev->pins_miso_pullhigh);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl miso_pullhigh\n", __func__);
		return ret;
	}

	gf_dev->pins_miso_pulllow = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "miso_pulllow");
	if (IS_ERR(gf_dev->pins_miso_pulllow)) {
		ret = PTR_ERR(gf_dev->pins_miso_pulllow);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl miso_pulllow\n", __func__);
		return ret;
	}

	gf_dev->pins_reset_high = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "reset_high");
	if (IS_ERR(gf_dev->pins_reset_high)) {
		ret = PTR_ERR(gf_dev->pins_reset_high);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl reset_high\n", __func__);
		return ret;
	}

	gf_dev->pins_reset_low = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "reset_low");
	if (IS_ERR(gf_dev->pins_reset_low)) {
		ret = PTR_ERR(gf_dev->pins_reset_low);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl reset_low\n", __func__);
		return ret;
	}

	gf_dev->pins_mosi_spi = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "mosi_spi");
	if (IS_ERR(gf_dev->pins_mosi_spi)) {
		ret = PTR_ERR(gf_dev->pins_mosi_spi);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl pins_mosi_spi\n", __func__);
		return ret;
	}

	gf_dev->pins_mosi_pullhigh = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "mosi_pullhigh");
	if (IS_ERR(gf_dev->pins_mosi_pullhigh)) {
		ret = PTR_ERR(gf_dev->pins_mosi_pullhigh);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl mosi_pullhigh\n", __func__);
		return ret;
	}

	gf_dev->pins_mosi_pulllow = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "mosi_pulllow");
	if (IS_ERR(gf_dev->pins_mosi_pulllow)) {
		ret = PTR_ERR(gf_dev->pins_mosi_pulllow);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl mosi_pulllow\n", __func__);
		return ret;
	}

	gf_dev->pins_cs_spi = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "cs_spi");
	if (IS_ERR(gf_dev->pins_cs_spi)) {
		ret = PTR_ERR(gf_dev->pins_cs_spi);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl pins_cs_spi\n", __func__);
		return ret;
	}

	gf_dev->pins_cs_pullhigh = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "cs_pullhigh");
	if (IS_ERR(gf_dev->pins_cs_pullhigh)) {
		ret = PTR_ERR(gf_dev->pins_cs_pullhigh);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl cs_pullhigh\n", __func__);
		return ret;
	}

	gf_dev->pins_cs_pulllow = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "cs_pulllow");
	if (IS_ERR(gf_dev->pins_cs_pulllow)) {
		ret = PTR_ERR(gf_dev->pins_cs_pulllow);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl cs_pulllow\n", __func__);
		return ret;
	}

	gf_dev->pins_clk_spi = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "clk_spi");
	if (IS_ERR(gf_dev->pins_clk_spi)) {
		ret = PTR_ERR(gf_dev->pins_clk_spi);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl pins_clk_spi\n", __func__);
		return ret;
	}

	gf_dev->pins_clk_pullhigh = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "clk_pullhigh");
	if (IS_ERR(gf_dev->pins_clk_pullhigh)) {
		ret = PTR_ERR(gf_dev->pins_clk_pullhigh);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl clk_pullhigh\n", __func__);
		return ret;
	}

	gf_dev->pins_clk_pulllow = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "clk_pulllow");
	if (IS_ERR(gf_dev->pins_clk_pulllow)) {
		ret = PTR_ERR(gf_dev->pins_clk_pulllow);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl clk_pulllow\n", __func__);
		return ret;
	}

	gf_dev->pins_ldo_high = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "ldo_high");
	if (IS_ERR(gf_dev->pins_ldo_high)) {
		ret = PTR_ERR(gf_dev->pins_ldo_high);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl reset_high\n", __func__);
		return ret;
	}

	gf_dev->pins_ldo_low = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "ldo_low");
	if (IS_ERR(gf_dev->pins_ldo_low)) {
		ret = PTR_ERR(gf_dev->pins_ldo_low);
		gf_debug(ERR_LOG, "%s can't find fingerprint pinctrl reset_low\n", __func__);
		return ret;
	}

	gf_debug(DEBUG_LOG, "%s: get pinctrl success!\n", __func__);
#endif

	FUNC_EXIT();

	return 0;
}

static void gf_hw_power_enable(struct gf_device *gf_dev, u8 onoff)
{
	/* TODO: LDO configure */
	static int enable = 1;

	if (onoff && enable) {
		/* TODO:  set power  according to actual situation  */
		/* hwPowerOn(MT6331_POWER_LDO_VIBR, VOL_2800, "fingerprint"); */
		enable = 0;
#ifdef CONFIG_OF
		pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_low);
		mdelay(15);
		pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_high);
#endif
	} else if (!onoff && !enable) {
		/* hwPowerDown(MT6331_POWER_LDO_VIBR, "fingerprint"); */
		enable = 1;
	}
}

static void gf_spi_clk_enable(struct gf_device *gf_dev, u8 bonoff)
{
	static int count = 0;
#ifdef CONFIG_MTK_CLKMGR
	if (bonoff && (count == 0)) {
		gf_debug(DEBUG_LOG, "%s: start to enable spi clk && count = %d.\n", __func__, count);
		enable_clock(MT_CG_PERI_SPI0, "spi");
		count = 1;
	} else if ((count > 0) && (bonoff == 0)) {
		gf_debug(DEBUG_LOG, "%s: start to disable spi clk&& count = %d.\n", __func__, count);
		disable_clock(MT_CG_PERI_SPI0, "spi");
		count = 0;
	}
#else
	/* changed after MT6797 platform */
	struct mt_spi_t *ms = NULL;

	ms = spi_master_get_devdata(gf_dev->spi->master);

	if (bonoff && (count == 0)) {
		clk_enable(ms->clk_main);
		count = 1;
	} else if ((count > 0) && (bonoff == 0)) {
		clk_disable(ms->clk_main);
		count = 0;
	}
#endif
}

static void gf_bypass_flash_gpio_cfg(void)
{
	/* TODO: by pass flash IO config, default connect to GND */
}

/* pull high miso, or change to SPI mode */
static void gf_miso_gpio_cfg(struct gf_device *gf_dev, u8 pullhigh)
{
#ifdef CONFIG_OF
	if (pullhigh)
		pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_miso_pullhigh);
	else
		pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_miso_spi);
#endif

}

static void gf_mosi_gpio_cfg(struct gf_device *gf_dev, u8 pullhigh)
{
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_mosi_spi);
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_cs_spi);
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_clk_spi);
}

static void gf_irq_gpio_cfg(struct gf_device *gf_dev)
{
#ifdef CONFIG_OF
	struct device_node *node;

	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_default);

	node = of_find_compatible_node(NULL, NULL, "mediatek,goodix-fp");
	if (node == NULL) {
		gf_debug(ERR_LOG, "%s: can't find compatible node\n", __func__);
		return ;
	}

	gf_dev->irq_num = irq_of_parse_and_map(node, 0);
	gf_debug(INFO_LOG, "%s: irq_num = %d\n", __func__, gf_dev->irq_num);
	gf_dev->irq = gf_dev->irq_num;
#endif
}

static void gf_reset_gpio_cfg(struct gf_device *gf_dev)
{
#ifdef CONFIG_OF
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_high);
#endif
}

/* delay ms after reset */
static void gf_hw_reset(struct gf_device *gf_dev, u8 delay)
{
#ifdef CONFIG_OF
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_low);
	mdelay(5);
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_high);
#endif

	if (delay) {
		/* delay is configurable */
		mdelay(delay);
	}
}

static void gf_enable_irq(struct gf_device *gf_dev)
{
	if (1 == gf_dev->irq_count) {
		gf_debug(ERR_LOG, "%s: irq already enabled\n", __func__);
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_count = 1;
		gf_debug(DEBUG_LOG, "%s enable interrupt!\n", __func__);
	}
}

static void gf_disable_irq(struct gf_device *gf_dev)
{
	if (0 == gf_dev->irq_count) {
		gf_debug(ERR_LOG, "%s: irq already disabled\n", __func__);
	} else {
		disable_irq(gf_dev->irq);
		gf_dev->irq_count = 0;
		gf_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
	}
}

/* -------------------------------------------------------------------- */
/* netlink functions                 */
/* -------------------------------------------------------------------- */
static void gf_netlink_send(struct gf_device *gf_dev, const int command)
{
	struct nlmsghdr *nlh = NULL;
	struct sk_buff *skb = NULL;
	int ret;

	FUNC_ENTRY();

	gf_debug(DEBUG_LOG, "%s: send command %d\n", __func__, command);
	if (NULL == gf_dev->nl_sk) {
		gf_debug(ERR_LOG, "%s: invalid socket\n", __func__);
		return;
	}

	if (0 == pid) {
		gf_debug(ERR_LOG, "%s: invalid native process pid\n", __func__);
		return;
	}

	/*alloc data buffer for sending to native*/
	/*malloc data space at least 1500 bytes, which is ethernet data length*/
	skb = alloc_skb(MAX_NL_MSG_LEN, GFP_ATOMIC);
	if (skb == NULL) {
		gf_debug(ERR_LOG, "%s: skb_get return NULL\n", __func__);
		return;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
	if (!nlh) {
		gf_debug(ERR_LOG, "%s: nlmsg_put failed\n", __func__);
		kfree_skb(skb);
		return;
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	*(char *)NLMSG_DATA(nlh) = command;
	ret = netlink_unicast(gf_dev->nl_sk, skb, pid, MSG_DONTWAIT);
	if (ret == 0) {
		gf_debug(ERR_LOG, "%s: send failed\n", __func__);
		return;
	}

	gf_debug(DEBUG_LOG, "%s: send done, data length is %d\n", __func__, ret);

	FUNC_EXIT();
}

static void gf_netlink_recv(struct sk_buff *__skb)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	char str[128];

	FUNC_ENTRY();

	skb = skb_get(__skb);
	if (skb == NULL) {
		gf_debug(ERR_LOG, "%s: skb_get return NULL\n", __func__);
		return;
	}

	/* presume there is 5byte payload at leaset */
	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		memcpy(str, NLMSG_DATA(nlh), sizeof(str));
		pid = nlh->nlmsg_pid;
		gf_debug(DEBUG_LOG, "%s: pid: %d, msg: %s\n", __func__, pid, str);

	} else {
		gf_debug(ERR_LOG, "%s: not enough data length\n", __func__);
	}

	kfree_skb(skb);

	FUNC_EXIT();
}

static int gf_netlink_init(struct gf_device *gf_dev)
{
	struct netlink_kernel_cfg cfg;

	memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
	cfg.input = gf_netlink_recv;

	gf_dev->nl_sk = netlink_kernel_create(&init_net, gf_dev->netlink_route, &cfg);
	if (gf_dev->nl_sk == NULL) {
		gf_debug(ERR_LOG, "%s: netlink create failed\n", __func__);
		return -1;
	}

	gf_debug(DEBUG_LOG, "%s: netlink create success\n", __func__);

	return 0;
}

static int gf_netlink_destroy(struct gf_device *gf_dev)
{
	if (gf_dev->nl_sk == NULL) {
		gf_debug(ERR_LOG, "%s: no netlink socket yet\n", __func__);
		return -1;
	}

	netlink_kernel_release(gf_dev->nl_sk);
	gf_dev->nl_sk = NULL;

	return 0;
}

/* -------------------------------------------------------------------- */
/* early suspend callback and suspend/resume functions          */
/* -------------------------------------------------------------------- */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void gf_early_suspend(struct early_suspend *handler)
{
	struct gf_device *gf_dev = NULL;

	gf_dev = container_of(handler, struct gf_device, early_suspend);

	gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
}

static void gf_late_resume(struct early_suspend *handler)
{
	struct gf_device *gf_dev = NULL;

	gf_dev = container_of(handler, struct gf_device, early_suspend);

	gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
}
#else

static int gf_fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct gf_device *gf_dev = NULL;
	struct fb_event *evdata = data;
	unsigned int blank;
	int retval = 0;
	FUNC_ENTRY();

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */)
		return 0;

	gf_dev = container_of(self, struct gf_device, notifier);
	blank = *(int *)evdata->data;

	gf_debug(DEBUG_LOG, "%s: blank=0x%x\n", __func__, blank);

	switch (blank) {
	case FB_BLANK_UNBLANK:
		gf_debug(INFO_LOG, "%s: FB_BLANK_UNBLANK\n", __func__);
		gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
		break;

	case FB_BLANK_POWERDOWN:
		gf_debug(INFO_LOG, "%s: FB_BLANK_POWERDOWN\n", __func__);
		gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
		break;

	default:
		gf_debug(ERR_LOG, "%s: ignore other blank event\n", __func__);
	}
	FUNC_EXIT();
	return retval;
}

#endif /* CONFIG_HAS_EARLYSUSPEND */

/* -------------------------------------------------------------------- */
/* file operation function                                              */
/* -------------------------------------------------------------------- */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;

#if 0//#ifdef SUPPORT_REE_SPI
	struct gf_device *gf_dev = NULL;
	u8 status;
	u8 *transfer_buf = NULL;
	u16 checksum = 0;
	int i;

	FUNC_ENTRY();
	gf_dev = (struct gf_device *)filp->private_data;

	gf_spi_read_byte_ree(gf_dev, 0x8140, &status);
	if ((status & 0xF0) != 0xC0) {
		gf_debug(ERR_LOG, "%s: no image data available\n", __func__);
		return 0;
	} else {
		if ((count > bufsiz) || (count == 0)) {
			gf_debug(ERR_LOG, "%s: request transfer length larger than maximum buffer\n", __func__);
			return -EINVAL;
		} else {
			transfer_buf = kzalloc((count + 10), GFP_KERNEL);
			if (transfer_buf == NULL) {
				gf_debug(ERR_LOG, "%s: failed to allocate transfer buffer\n", __func__);
				return -EMSGSIZE;
			}
		}
	}

	/* set spi to high speed */
	gf_spi_setup_conf_ree(gf_dev, HIGH_SPEED, DMA_TRANSFER);

	gf_spi_read_bytes_ree(gf_dev, 0x8140, count + 10, transfer_buf);

	/* check checksum */
	checksum = 0;
	for (i = 0; i < (count + 6); i++) {
		checksum += *(transfer_buf + 2 + i);
	}
	if (checksum != ((*(transfer_buf + count + 8) << 8) | *(transfer_buf + count + 9))) {
		gf_debug(ERR_LOG, "%s: raw data checksum check failed, cal[0x%x], recevied[0x%x]\n", __func__,
				checksum, ((*(transfer_buf + count + 8) << 8) | *(transfer_buf + count + 9)));
		retval = 0;
	} else {
		gf_debug(INFO_LOG, "%s: checksum check passed[0x%x], copy_to_user\n", __func__, checksum);
		if (copy_to_user(buf, transfer_buf + 8, count)) {
			gf_debug(ERR_LOG, "%s: Failed to copy gf_ioc_transfer from kernel to user\n", __func__);
			retval = -EFAULT;
		} else {
			retval = count;
		}
	}

	/* restore to low speed */
	gf_spi_setup_conf_ree(gf_dev, LOW_SPEED, FIFO_TRANSFER);

	kfree(transfer_buf);

	FUNC_EXIT();
#endif /* SUPPORT_REE_SPI */
	return retval;
}

static ssize_t gf_write(struct file *filp, const char __user *buf,
			size_t count, loff_t *f_pos)
{
	gf_debug(ERR_LOG, "%s: Not support write opertion in TEE mode\n", __func__);
	return -EFAULT;
}

static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_device *gf_dev = (struct gf_device *)handle;
	FUNC_ENTRY();

#ifdef USES_FPS_WAKE_UP_ENABLE
	wake_lock_timeout(&ttw_wl, msecs_to_jiffies(TTW_HOLD_TIME));
#endif

	gf_netlink_send(gf_dev, GF_NETLINK_IRQ);
	gf_dev->sig_count++;

	gf_debug(INFO_LOG, "%s: sig_count = %d\n", __func__, gf_dev->sig_count);

	FUNC_EXIT();
	return IRQ_HANDLED;
}


static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_device *gf_dev = NULL;
#ifdef CONFIG_LGE_FINGERPRINT_KEYMODE
	struct gf_key gf_key;
	gf_nav_event_t nav_event = GF_NAV_NONE;
	uint32_t nav_input = 0;
	uint32_t key_input = 0;
#endif // CONFIG_LGE_FINGERPRINT_KEYMODE
#if 0//#ifdef SUPPORT_REE_SPI
	struct gf_ioc_transfer ioc;
	u8 *transfer_buf = NULL;
#endif
	int retval = 0;
	u8  buf    = 0;
	u8 netlink_route;
	struct gf_ioc_chip_info info;

	FUNC_ENTRY();

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC) {
		gf_debug(ERR_LOG, "%s: incorrect magic number\n", __func__);
		return -EINVAL;
	}

	/* Check access direction once here; don't repeat below.
	* IOC_DIR is from the user perspective, while access_ok is
	* from the kernel perspective; so they look reversed.
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));

	if (retval == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (retval) {
		gf_debug(ERR_LOG, "%s: ioctl error ret = %d\n", __func__, retval);
		return -EINVAL;
	}

	gf_dev = (struct gf_device *)filp->private_data;
	if (!gf_dev) {
		gf_debug(ERR_LOG, "%s: gf_dev IS NULL ======\n", __func__);
		return -EINVAL;
	}
	netlink_route = gf_dev->netlink_route;

	gf_debug(DEBUG_LOG, "%s: === IOCTL cmd = 0x%08x\n", __func__, cmd);

	switch (cmd) {
	case GF_IOC_INIT:
		gf_debug(INFO_LOG, "%s: GF_IOC_INIT gf init======\n", __func__);
		gf_debug(INFO_LOG, "%s: Linux Version %s\n", __func__, GF_LINUX_VERSION);

		if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
			gf_debug(ERR_LOG, "Failed to copy netlink_rount number from kernel to user\n");
			retval = -EFAULT;
			break;
		}

		if (gf_dev->system_status) {
			gf_debug(INFO_LOG, "%s: system re-started======\n", __func__);
			break;
		}

		gf_irq_gpio_cfg(gf_dev);
		retval = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, "gf_irq", gf_dev);
		if (retval)
			gf_debug(ERR_LOG, "%s: irq thread request failed, retval=%d\n", __func__, retval);

		gf_dev->irq_count = 1;
		gf_disable_irq(gf_dev);

#if defined(CONFIG_HAS_EARLYSUSPEND)
		gf_debug(DEBUG_LOG, "%s: register_early_suspend\n", __func__);
		gf_dev->early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
		gf_dev->early_suspend.suspend = gf_early_suspend,
		gf_dev->early_suspend.resume = gf_late_resume,
		register_early_suspend(&gf_dev->early_suspend);
#else
		/* register screen on/off callback */
		gf_dev->notifier.notifier_call = gf_fb_notifier_callback;
		fb_register_client(&gf_dev->notifier);
#endif

		gf_dev->sig_count = 0;
		gf_dev->system_status = 1;

		gf_debug(INFO_LOG, "%s: gf init finished======\n", __func__);
		break;

	case GF_IOC_CHIP_INFO:
		gf_debug(INFO_LOG, "%s: GF_IOC_CHIP_INFO ======\n", __func__);

		if (copy_from_user(&info, (struct gf_ioc_chip_info *)arg, sizeof(struct gf_ioc_chip_info))) {
			gf_debug(ERR_LOG, "Failed to copy chip info from user to kernel\n");
			retval = -EFAULT;
			break;
		}
		g_vendor_id = info.vendor_id;

		gf_debug(INFO_LOG, "%s: vendor_id 0x%x\n", __func__, g_vendor_id);
		gf_debug(INFO_LOG, "%s: mode 0x%x\n", __func__, info.mode);
		gf_debug(INFO_LOG, "%s: operation 0x%x\n", __func__, info.operation);
		break;

	case GF_IOC_EXIT:
		gf_debug(INFO_LOG, "%s: GF_IOC_EXIT ======\n", __func__);
		gf_disable_irq(gf_dev);
		if (gf_dev->irq) {
			free_irq(gf_dev->irq, gf_dev);
			gf_dev->irq_count = 0;
			gf_dev->irq = 0;
		}

#ifdef CONFIG_HAS_EARLYSUSPEND
		if (gf_dev->early_suspend.suspend)
			unregister_early_suspend(&gf_dev->early_suspend);
#else
		fb_unregister_client(&gf_dev->notifier);
#endif

		gf_dev->system_status = 0;
		gf_debug(INFO_LOG, "%s: gf exit finished ======\n", __func__);
		break;

	case GF_IOC_RESET:
		gf_debug(INFO_LOG, "%s: GF_IOC_RESET ======\n", __func__);
		gf_hw_reset(gf_dev, 5);
		break;

	case GF_IOC_ENABLE_IRQ:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_IRQ ======\n", __func__);
		gf_enable_irq(gf_dev);
		break;

	case GF_IOC_DISABLE_IRQ:
		gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_IRQ ======\n", __func__);
		gf_disable_irq(gf_dev);
		break;

	case GF_IOC_ENABLE_SPI_CLK:
#if 0
		gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_SPI_CLK ======\n", __func__);
#endif
		gf_spi_clk_enable(gf_dev, 1);
		break;

	case GF_IOC_DISABLE_SPI_CLK:
#if 0
		gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_SPI_CLK ======\n", __func__);
#endif
		gf_spi_clk_enable(gf_dev, 0);
		break;

	case GF_IOC_ENABLE_POWER:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_POWER ======\n", __func__);
		gf_hw_power_enable(gf_dev, 1);
		break;

	case GF_IOC_DISABLE_POWER:
		gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_POWER ======\n", __func__);
		gf_hw_power_enable(gf_dev, 0);
		break;

#ifdef CONFIG_LGE_FINGERPRINT_KEYMODE
	case GF_IOC_INPUT_KEY_EVENT:
		gf_debug(INFO_LOG, "%s: GF_IOC_INPUT_KEY_EVENT ======\n", __func__);

		if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
			gf_debug(ERR_LOG, "Failed to copy input key event from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		if (GF_KEY_HOME == gf_key.key) {
			key_input = GF_KEY_INPUT_HOME;
		} else if (GF_KEY_POWER == gf_key.key) {
			key_input = GF_KEY_INPUT_POWER;
		} else if (GF_KEY_CAMERA == gf_key.key) {
			key_input = GF_KEY_INPUT_CAMERA;
		} else {
			/* add special key define */
			key_input = gf_key.key;
		}
		gf_debug(INFO_LOG, "%s: received key event=%d, gf_key=%d, value=%d\n",
				__func__, key_input, gf_key.key, gf_key.value);

		if ((GF_KEY_POWER == gf_key.key || GF_KEY_CAMERA == gf_key.key) && (gf_key.value == 1)) {
			input_report_key(gf_dev->input, key_input, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_input, 0);
			input_sync(gf_dev->input);
		}

		if (GF_KEY_HOME == gf_key.key) {
			input_report_key(gf_dev->input, key_input, gf_key.value);
			input_sync(gf_dev->input);
		}
		break;

	case GF_IOC_NAV_EVENT:
		gf_debug(INFO_LOG, "%s: GF_IOC_NAV_EVENT ======\n", __func__);

		if (copy_from_user(&nav_event, (gf_nav_event_t *)arg, sizeof(gf_nav_event_t))) {
			gf_debug(ERR_LOG, "Failed to copy nav event from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		switch (nav_event) {
		case GF_NAV_FINGER_DOWN:
			gf_debug(INFO_LOG, "%s: GF_NAV_FINGER_DOWN\n", __func__);
#ifdef CONFIG_LGE_FINGERPRINT_KEYMODE
			nav_input = GF_KEY_INPUT_HOME;
			input_report_key(gf_dev->input, nav_input, 1);
			input_sync(gf_dev->input);
#endif // CONFIG_LGE_FINGERPRINT_KEYMODE
			break;

		case GF_NAV_FINGER_UP:
			gf_debug(INFO_LOG, "%s: GF_NAV_FINGER_UP\n", __func__);
#ifdef CONFIG_LGE_FINGERPRINT_KEYMODE
			nav_input = GF_KEY_INPUT_HOME;
			input_report_key(gf_dev->input, nav_input, 0);
			input_sync(gf_dev->input);
#endif // CONFIG_LGE_FINGERPRINT_KEYMODE
			break;

		case GF_NAV_DOWN:
			gf_debug(INFO_LOG, "%s: GF_NAV_DOWN\n", __func__);
			nav_input = GF_NAV_INPUT_DOWN;
			break;

		case GF_NAV_UP:
			gf_debug(INFO_LOG, "%s: GF_NAV_UP\n", __func__);
			nav_input = GF_NAV_INPUT_UP;
			break;

		case GF_NAV_LEFT:
			gf_debug(INFO_LOG, "%s: GF_NAV_LEFT\n", __func__);
			nav_input = GF_NAV_INPUT_LEFT;
			break;

		case GF_NAV_RIGHT:
			gf_debug(INFO_LOG, "%s: GF_NAV_RIGHT\n", __func__);
			nav_input = GF_NAV_INPUT_RIGHT;
			break;

		case GF_NAV_CLICK:
			gf_debug(INFO_LOG, "%s: GF_NAV_CLICK\n", __func__);
			nav_input = GF_NAV_INPUT_CLICK;
			break;

		case GF_NAV_HEAVY:
			gf_debug(INFO_LOG, "%s: GF_NAV_HEAVY\n", __func__);
			nav_input = GF_NAV_INPUT_HEAVY;
			break;

		case GF_NAV_LONG_PRESS:
			gf_debug(INFO_LOG, "%s: GF_NAV_LONG_PRESS\n", __func__);
			nav_input = GF_NAV_INPUT_LONG_PRESS;
			break;

		case GF_NAV_DOUBLE_CLICK:
			gf_debug(INFO_LOG, "%s: GF_NAV_DOUBLE_CLICK\n", __func__);
			nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
			break;

		default:
			gf_debug(ERR_LOG, "%s: not support nav event nav_event: %d ======\n", __func__, nav_event);
			break;
		}

		if ((nav_event != GF_NAV_FINGER_DOWN) && (nav_event != GF_NAV_FINGER_UP)) {
			input_report_key(gf_dev->input, nav_input, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, nav_input, 0);
			input_sync(gf_dev->input);
		}
		break;
#endif // CONFIG_LGE_FINGERPRINT_KEYMODE

	case GF_IOC_ENTER_SLEEP_MODE:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENTER_SLEEP_MODE ======\n", __func__);
		break;

	case GF_IOC_GET_FW_INFO:
		gf_debug(INFO_LOG, "%s: GF_IOC_GET_FW_INFO ======\n", __func__);
		buf = gf_dev->need_update;

		gf_debug(DEBUG_LOG, "%s: firmware info 0x%x\n", __func__, buf);
		if (copy_to_user((void __user *)arg, (void *)&buf, sizeof(u8))) {
			gf_debug(ERR_LOG, "Failed to copy data to user\n");
			retval = -EFAULT;
		}
		break;

	case GF_IOC_REMOVE:
		gf_debug(INFO_LOG, "%s: GF_IOC_REMOVE ======\n", __func__);

		gf_netlink_destroy(gf_dev);

		mutex_lock(&gf_dev->release_lock);
		if (gf_dev->input == NULL) {
			mutex_unlock(&gf_dev->release_lock);
			break;
		}
#ifdef CONFIG_LGE_FINGERPRINT_KEYMODE
		input_unregister_device(gf_dev->input);
		gf_dev->input = NULL;
#endif // CONFIG_LGE_FINGERPRINT_KEYMODE
		mutex_unlock(&gf_dev->release_lock);

		cdev_del(&gf_dev->cdev);
		sysfs_remove_group(&gf_dev->spi->dev.kobj, &gf_debug_attr_group);
		device_destroy(gf_dev->class, gf_dev->devno);
		list_del(&gf_dev->device_entry);
		unregister_chrdev_region(gf_dev->devno, 1);
		class_destroy(gf_dev->class);
		gf_hw_power_enable(gf_dev, 0);
		gf_spi_clk_enable(gf_dev, 0);

		mutex_lock(&gf_dev->release_lock);
		if (gf_dev->spi_buffer != NULL) {
			kfree(gf_dev->spi_buffer);
			gf_dev->spi_buffer = NULL;
		}
		mutex_unlock(&gf_dev->release_lock);

		spi_set_drvdata(gf_dev->spi, NULL);
		gf_dev->spi = NULL;
		mutex_destroy(&gf_dev->buf_lock);
		mutex_destroy(&gf_dev->release_lock);
		break;

#if 0//#ifdef SUPPORT_REE_SPI
	case GF_IOC_TRANSFER_CMD:
		if (copy_from_user(&ioc, (struct gf_ioc_transfer *)arg, sizeof(struct gf_ioc_transfer))) {
			gf_debug(ERR_LOG, "%s: Failed to copy gf_ioc_transfer from user to kernel\n", __func__);
			retval = -EFAULT;
			break;
		}

		if ((ioc.len > bufsiz) || (ioc.len == 0)) {
			gf_debug(ERR_LOG, "%s: request transfer length larger than maximum buffer\n", __func__);
			retval = -EINVAL;
			break;
		} else {
			transfer_buf = kzalloc(ioc.len, GFP_KERNEL);
			if (transfer_buf == NULL) {
				gf_debug(ERR_LOG, "%s: failed to allocate transfer buffer\n", __func__);
				retval = -EMSGSIZE;
				break;
			}
		}

		mutex_lock(&gf_dev->buf_lock);
		if (ioc.cmd) {
			/* spi write operation */
			gf_debug(DEBUG_LOG, "%s: write data to 0x%x, len = 0x%x\n", __func__, ioc.addr, ioc.len);
			if (copy_from_user(transfer_buf, ioc.buf, ioc.len)) {
				gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer from user to kernel\n");
				retval = -EFAULT;
			} else {
				gf_spi_write_bytes_ree(gf_dev, ioc.addr, ioc.len, transfer_buf);
			}
		} else {
			/* spi read operation */
			gf_debug(DEBUG_LOG, "%s: read data from 0x%x, len = 0x%x\n", __func__, ioc.addr, ioc.len);
			gf_spi_read_bytes_ree(gf_dev, ioc.addr, ioc.len, transfer_buf);
			if (copy_to_user(ioc.buf, transfer_buf, ioc.len)) {
				gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer from kernel to user\n");
				retval = -EFAULT;
			}
		}
		kfree(transfer_buf);
		mutex_unlock(&gf_dev->buf_lock);
		break;

	case GF_IOC_TRANSFER_RAW_CMD:
		retval = gf_ioctl_transfer_raw_cmd(gf_dev,arg,bufsiz);
		break;

	case GF_IOC_SPI_INIT_CFG_CMD:
		retval = gf_ioctl_spi_init_cfg_cmd(&gf_dev->spi_mcc,arg);
		break;

#endif /* SUPPORT_REE_SPI */
	default:
		gf_debug(ERR_LOG, "gf doesn't support this command(%x)\n", cmd);
		break;
	}

	FUNC_EXIT();
	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	FUNC_ENTRY();

	retval = filp->f_op->unlocked_ioctl(filp, cmd, arg);

	FUNC_EXIT();
	return retval;
}
#endif

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
	gf_debug(ERR_LOG, "Not support poll opertion in TEE version\n");
	return -EFAULT;
}

/* -------------------------------------------------------------------- */
/* devfs                                                              */
/* -------------------------------------------------------------------- */
static ssize_t gf_debug_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	gf_debug(INFO_LOG, "%s: Show debug_level = 0x%x\n", __func__, g_debug_level);
	return sprintf(buf, "vendor id 0x%x\n", g_vendor_id);
}

static ssize_t gf_debug_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct gf_device *gf_dev =  dev_get_drvdata(dev);
	int retval = 0;
	u8 flag = 0;

	if (!strncmp(buf, "-8", 2)) {
		gf_debug(INFO_LOG, "%s: parameter is -8, enable spi clock test===============\n", __func__);
		gf_spi_clk_enable(gf_dev, 1);

	} else if (!strncmp(buf, "-9", 2)) {
		gf_debug(INFO_LOG, "%s: parameter is -9, disable spi clock test===============\n", __func__);
		gf_spi_clk_enable(gf_dev, 0);

	} else if (!strncmp(buf, "-10", 3)) {
		gf_debug(INFO_LOG, "%s: parameter is -10, gf init start===============\n", __func__);

		gf_irq_gpio_cfg(gf_dev);
		retval = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(&(gf_dev->spi->dev)), gf_dev);

		if (retval)
			gf_debug(ERR_LOG, "%s: irq thread request failed, retval=%d\n", __func__, retval);

		gf_dev->irq_count = 1;
		gf_disable_irq(gf_dev);

#if defined(CONFIG_HAS_EARLYSUSPEND)
		gf_debug(INFO_LOG, "%s: register_early_suspend\n", __func__);
		gf_dev->early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
		gf_dev->early_suspend.suspend = gf_early_suspend,
		gf_dev->early_suspend.resume = gf_late_resume,
		register_early_suspend(&gf_dev->early_suspend);
#else
		/* register screen on/off callback */
		gf_dev->notifier.notifier_call = gf_fb_notifier_callback;
		fb_register_client(&gf_dev->notifier);
#endif

		gf_dev->sig_count = 0;

		gf_debug(INFO_LOG, "%s: gf init finished======\n", __func__);

	} else if (!strncmp(buf, "-11", 3)) {
		gf_debug(INFO_LOG, "%s: parameter is -11, enable irq===============\n", __func__);
		gf_enable_irq(gf_dev);

	} else if (!strncmp(buf, "-12", 3)) {
		gf_debug(INFO_LOG, "%s: parameter is -12, GPIO test===============\n", __func__);
		gf_reset_gpio_cfg(gf_dev);

#ifdef CONFIG_OF
		if (flag == 0) {
			pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_miso_pulllow);
			gf_debug(INFO_LOG, "%s: set miso PIN to low\n", __func__);
			flag = 1;
		} else {
			pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_miso_pullhigh);
			gf_debug(INFO_LOG, "%s: set miso PIN to high\n", __func__);
			flag = 0;
		}
#endif

	} else if (!strncmp(buf, "-13", 3)) {
		gf_debug(INFO_LOG, "%s: parameter is -13, Vendor ID test --> 0x%x\n", __func__, g_vendor_id);
	} else {
		gf_debug(ERR_LOG, "%s: wrong parameter!===============\n", __func__);
	}

	return count;
}

/* -------------------------------------------------------------------- */
/* device function								  */
/* -------------------------------------------------------------------- */
static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = NULL;
	int status = -ENXIO;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devno == inode->i_rdev) {
			gf_debug(INFO_LOG, "%s: Found\n", __func__);
			status = 0;
			break;
		}
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		filp->private_data = gf_dev;
		nonseekable_open(inode, filp);
		gf_debug(INFO_LOG, "%s: Success to open device. irq = %d\n", __func__, gf_dev->irq);
	} else {
		gf_debug(ERR_LOG, "%s: No device for minor %d\n", __func__, iminor(inode));
	}
	FUNC_EXIT();
	return status;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = NULL;
	int    status = 0;

	FUNC_ENTRY();
	gf_dev = filp->private_data;
	if (gf_dev->irq)
		gf_disable_irq(gf_dev);
	gf_dev->need_update = 0;
	FUNC_EXIT();
	return status;
}

#ifdef SUPPORT_REE_SPI
/* -------------------------------------------------------------------- */
/* normal world SPI read/write function                 */
/* -------------------------------------------------------------------- */

/* gf_spi_setup_conf_ree, configure spi speed and transfer mode in REE mode
  *
  * speed: 1, 4, 6, 8 unit:MHz
  * mode: DMA mode or FIFO mode
  */
void gf_spi_setup_conf_ree(struct gf_device *gf_dev, u32 speed, enum spi_transfer_mode mode)
{
	struct mt_chip_conf *mcc = &gf_dev->spi_mcc;

	switch (speed) {
	case 1:
		/* set to 1MHz clock */
		mcc->high_time = 50;
		mcc->low_time = 50;
		break;
	case 4:
		/* set to 4MHz clock */
		mcc->high_time = 15;
		mcc->low_time = 15;
		break;
	case 6:
		/* set to 6MHz clock */
		mcc->high_time = 10;
		mcc->low_time = 10;
		break;
	case 8:
		/* set to 8MHz clock */
		mcc->high_time = 8;
		mcc->low_time = 8;
		break;
	default:
		/* default set to 1MHz clock */
		mcc->high_time = 50;
		mcc->low_time = 50;
	}

	if ((mode == DMA_TRANSFER) || (mode == FIFO_TRANSFER)) {
		mcc->com_mod = mode;
	} else {
		/* default set to FIFO mode */
		mcc->com_mod = FIFO_TRANSFER;
	}

	if (spi_setup(gf_dev->spi))
		gf_debug(ERR_LOG, "%s: failed to setup spi conf\n", __func__);
}

int gf_spi_read_bytes_ree(struct gf_device *gf_dev, u16 addr, u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;
	u8 *tmp_buf = NULL;
	u32 package, reminder, retry;

	package = (data_len + 2) / 1024;
	reminder = (data_len + 2) % 1024;

	if ((package > 0) && (reminder != 0)) {
		xfer = kzalloc(sizeof(*xfer) * 4, GFP_KERNEL);
		retry = 1;
	} else {
		xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
		retry = 0;
	}
	if (xfer == NULL) {
		gf_debug(ERR_LOG, "%s: no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}

	tmp_buf = gf_dev->spi_buffer;

	/* switch to DMA mode if transfer length larger than 32 bytes */
	if ((data_len + 1) > 32) {
		gf_dev->spi_mcc.com_mod = DMA_TRANSFER;
		spi_setup(gf_dev->spi);
	}
	spi_message_init(&msg);
	*tmp_buf = 0xF0;
	*(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
	*(tmp_buf + 2) = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tmp_buf;
	xfer[0].len = 3;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	spi_message_init(&msg);
	/* memset((tmp_buf + 4), 0x00, data_len + 1); */
	/* 4 bytes align */
	*(tmp_buf + 4) = 0xF1;
	xfer[1].tx_buf = tmp_buf + 4;
	xfer[1].rx_buf = tmp_buf + 4;

	if (retry)
		xfer[1].len = package * 1024;
	else
		xfer[1].len = data_len + 1;

	xfer[1].delay_usecs = 5;
	spi_message_add_tail(&xfer[1], &msg);
	spi_sync(gf_dev->spi, &msg);

	/* copy received data */
	if (retry)
		memcpy(rx_buf, (tmp_buf + 5), (package * 1024 - 1));
	else
		memcpy(rx_buf, (tmp_buf + 5), data_len);

	/* send reminder SPI data */
	if (retry) {
		addr = addr + package * 1024 - 2;
		spi_message_init(&msg);

		*tmp_buf = 0xF0;
		*(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
		*(tmp_buf + 2) = (u8)(addr & 0xFF);
		xfer[2].tx_buf = tmp_buf;
		xfer[2].len = 3;
		xfer[2].delay_usecs = 5;
		spi_message_add_tail(&xfer[2], &msg);
		spi_sync(gf_dev->spi, &msg);

		spi_message_init(&msg);
		*(tmp_buf + 4) = 0xF1;
		xfer[3].tx_buf = tmp_buf + 4;
		xfer[3].rx_buf = tmp_buf + 4;
		xfer[3].len = reminder + 1;
		xfer[3].delay_usecs = 5;
		spi_message_add_tail(&xfer[3], &msg);
		spi_sync(gf_dev->spi, &msg);

		memcpy((rx_buf + package * 1024 - 1), (tmp_buf + 6), (reminder - 1));
	}

	/* restore to FIFO mode if has used DMA */
	if ((data_len + 1) > 32) {
		gf_dev->spi_mcc.com_mod = FIFO_TRANSFER;
		spi_setup(gf_dev->spi);
	}
	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

int gf_spi_write_bytes_ree(struct gf_device *gf_dev, u16 addr, u32 data_len, u8 *tx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;
	u8 *tmp_buf = NULL;
	u32 package, reminder, retry;

	package = (data_len + 3) / 1024;
	reminder = (data_len + 3) % 1024;

	if ((package > 0) && (reminder != 0)) {
		xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
		retry = 1;
	} else {
		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
		retry = 0;
	}
	if (xfer == NULL) {
		gf_debug(ERR_LOG, "%s: no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}
	tmp_buf = gf_dev->spi_buffer;

	/* switch to DMA mode if transfer length larger than 32 bytes */
	if ((data_len + 3) > 32) {
		gf_dev->spi_mcc.com_mod = DMA_TRANSFER;
		spi_setup(gf_dev->spi);
	}
	spi_message_init(&msg);
	*tmp_buf = 0xF0;
	*(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
	*(tmp_buf + 2) = (u8)(addr & 0xFF);
	*(tmp_buf + 3) = (u8)((data_len >> 8) & 0xFF);
	*(tmp_buf + 4) = (u8)(data_len & 0xFF);
	if (retry) {
		memcpy(tmp_buf + 5, tx_buf, (package * 1024 - 5));
		xfer[0].len = package * 1024;
	} else {
		memcpy(tmp_buf + 5, tx_buf, data_len);
		xfer[0].len = data_len + 5;
	}
	xfer[0].tx_buf = tmp_buf;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	if (retry) {
		addr = addr + package * 1024 - 5;
		spi_message_init(&msg);
		*tmp_buf = 0xF0;
		*(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
		*(tmp_buf + 2) = (u8)(addr & 0xFF);
		*(tmp_buf + 3) = (u8)((data_len >> 8) & 0xFF);
		*(tmp_buf + 4) = (u8)(data_len & 0xFF);
		memcpy(tmp_buf + 5, (tx_buf + package * 1024 - 5), reminder);
		xfer[1].tx_buf = tmp_buf;
		xfer[1].len = reminder + 5;
		xfer[1].delay_usecs = 5;
		spi_message_add_tail(&xfer[1], &msg);
		spi_sync(gf_dev->spi, &msg);
	}

	/* restore to FIFO mode if has used DMA */
	if ((data_len + 3) > 32) {
		gf_dev->spi_mcc.com_mod = FIFO_TRANSFER;
		spi_setup(gf_dev->spi);
	}
	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

int gf_spi_read_word_ree(struct gf_device *gf_dev, u16 addr, u16 *value)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;

	xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
	if (xfer == NULL) {
		gf_debug(ERR_LOG, "%s: no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}

	spi_message_init(&msg);
	*gf_dev->spi_buffer = 0xF0;
	*(gf_dev->spi_buffer + 1) = (u8)((addr >> 8) & 0xFF);
	*(gf_dev->spi_buffer + 2) = (u8)(addr & 0xFF);

	xfer[0].tx_buf = gf_dev->spi_buffer;
	xfer[0].len = 3;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	spi_message_init(&msg);
	/* 4 bytes align */
	*(gf_dev->spi_buffer + 4) = 0xF1;
	xfer[1].tx_buf = gf_dev->spi_buffer + 4;
	xfer[1].rx_buf = gf_dev->spi_buffer + 4;
	xfer[1].len = 3;
	xfer[1].delay_usecs = 5;
	spi_message_add_tail(&xfer[1], &msg);
	spi_sync(gf_dev->spi, &msg);

	*value = (*(gf_dev->spi_buffer + 5) << 8) | (*(gf_dev->spi_buffer + 6));

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

int gf_spi_write_word_ree(struct gf_device *gf_dev, u16 addr, u16 value)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;

	xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	if (xfer == NULL) {
		gf_debug(ERR_LOG, "%s: no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}

	spi_message_init(&msg);
	*gf_dev->spi_buffer = 0xF0;
	*(gf_dev->spi_buffer + 1) = (u8)((addr >> 8) & 0xFF);
	*(gf_dev->spi_buffer + 2) = (u8)(addr & 0xFF);
	*(gf_dev->spi_buffer + 3) = 0x00;
	*(gf_dev->spi_buffer + 4) = 0x02;
	*(gf_dev->spi_buffer + 5) = (u8)((value >> 8) & 0xFF);
	*(gf_dev->spi_buffer + 6) =  (u8)(value & 0xFF);

	xfer[0].tx_buf = gf_dev->spi_buffer;
	xfer[0].len = 7;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}
#endif /* SUPPORT_REE_SPI */

static const struct file_operations gf_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	* gets more complete API coverage.	It'll simplify things
	* too, except for the locking.
	*/
	.write =	gf_write,
	.read =		gf_read,
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif
	.open =		gf_open,
	.release =	gf_release,
	.poll	= gf_poll,
};

/*-------------------------------------------------------------------------*/
static int gf_read_chip_id(struct gf_device *gf_dev)
{
	int i;
	u16 chip_id_high, chip_id_low;
	u32 chip_id;
	int result = -1;

	for (i = 0; i < 5; i++) {
		gf_spi_read_word_ree(gf_dev, GF32XX_REG_CHIP_ID_HI, &chip_id_high);
		gf_spi_read_word_ree(gf_dev, GF32XX_REG_CHIP_ID_LO, &chip_id_low);
		gf_debug(INFO_LOG, "%s: i = %d, CHIP_ID_HI = 0x%04x, CHIP_ID_LO = 0x%04x\n",
				__func__, i, chip_id_high, chip_id_low);

		chip_id = ( (chip_id_high << 16) | (chip_id_low) ) >> 8;
		if (chip_id == GF3208_CHIP_ID
				|| chip_id == GF3228_CHIP_ID
				|| chip_id == GF3258_CHIP_ID
		) {
			gf_debug(INFO_LOG, "%s: succeed to read chip id. 0x%08x\n", __func__, chip_id);
			result = 0;
			break;
		}

		gf_hw_reset(gf_dev, 5);
	}

	return result;
}

static int gf_check_irq(struct gf_device *gf_dev)
{
	int i, j;
	int gpio_status_prev = 0;
	int gpio_status = 0;
	int result = -1;

	FUNC_ENTRY();

	gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CLEAR, 0x01FF);   //clear irq
	for (i = 0; i < 3; i++) {
		gpio_status_prev = gpio_get_value(gf_dev->irq_gpio);
		gf_debug(INFO_LOG, "%s: rst retry count = %d, gpio prev status = %d\n", __func__, i, gpio_status_prev);

		gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CTRL0, 0xFFFF); //GF3208_REG_IRQ_CTRL0 set irq high time
		gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CTRL4, 0x0002);

		gpio_status = gpio_get_value(gf_dev->irq_gpio);

		if (gpio_status_prev == 0 && gpio_status == 0) {
			gf_debug(INFO_LOG, "%s: start irq pin check.... gpio status = %d\n", __func__, gpio_status);

			for (j = 0; j < 10; j++) {
				mdelay(10);
				gpio_status = gpio_get_value(gf_dev->irq_gpio);
				gf_debug(INFO_LOG, "%s: irq retry count = %d, gpio status = %d\n", __func__, j, gpio_status);
				if (gpio_status == 1) {
					gf_debug(INFO_LOG, "irq_gpio status is high ...\n");
					break;
				}
			}
			if (gpio_status_prev == 0 && gpio_status == 1) {
				gf_debug(INFO_LOG, "%s: succeed to check irq gpio\n", __func__);
				result = 0;
				break;
			} else {
				gf_debug(ERR_LOG, "%s: reset and retry irq gpio check\n", __func__);
				gf_hw_reset(gf_dev, 5);
				gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CLEAR, 0x01FF);   //clear irq
			}
		} else if (gpio_status_prev == 0 && gpio_status == 1) {
			gf_debug(INFO_LOG, "%s: irq_gpio status is high..... gpio status = %d w/o delay\n", __func__, gpio_status);
			gf_debug(INFO_LOG, "%s: succeed to check irq gpio\n", __func__);
			result = 0;
			break;
		} else {
			gf_debug(INFO_LOG, "%s: irq_gpio_prev status was high..... gpio status = %d\n", __func__, gpio_status);
			gf_hw_reset(gf_dev, 5);
			gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CLEAR, 0x01FF);  //clear all irq
		}
	}

	gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CLEAR, 0xFFFF); //GF3208_REG_IRQ_CLEAR set irq clear
	mdelay(10);
	gpio_status = gpio_get_value(gf_dev->irq_gpio);

	if (gpio_status == 0) {
		gf_debug(INFO_LOG, "irq_gpio clear success\n");
	} else {
		gf_debug(ERR_LOG, "irq_gpio clear fail\n");
		result = -1;
	}

	FUNC_EXIT();

	return result;
}

static int gf_check_rst(struct gf_device *gf_dev)
{
	int i;
	int ret;
	unsigned short reset_data;

	gf_spi_write_word_ree(gf_dev,GF3208_REG_IRQ_CLEAR, 0x01FF); /*0x0124,0x0400 clean reset INT*/

	for (i = 0; i < MAX_RETRY_HW_CHECK_COUNT; i++)
	{
		gf_hw_reset(gf_dev, 5);

		if (gpio_get_value(gf_dev->reset_gpio) == 0)
		{
			gf_debug(ERR_LOG, "%s: rst pin check fail.\n", __func__);
			continue;
		}

		gf_debug(INFO_LOG, "%s: rst pin is high.\n", __func__);
		gf_spi_read_word_ree(gf_dev,GF3208_REG_IRQ_STATUS, &reset_data);
		gf_debug(INFO_LOG, "%s: reset_irq_data = 0x%x \n", __func__, reset_data);

		if (reset_data == GF3208_RST_DATA
				|| reset_data == GF3228_RST_DATA
		) {
			ret = 0;
			gf_debug(INFO_LOG, "%s: succeed to check chip reset.\n", __func__);
			gf_spi_write_word_ree(gf_dev,GF3208_REG_IRQ_CLEAR, reset_data); /*0x0124,0x0400 clean reset INT*/
			return ret;
		}

		ret = gf_read_chip_id(gf_dev);
		gf_debug(ERR_LOG, "%s: chip reset check fail. ret = %d\n", __func__, ret);
		msleep(20);
	}
	ret = -1;

	return ret;
}

static int gf_probe(struct spi_device *spi)
{
	struct gf_device *gf_dev = NULL;
	int status = -EINVAL;
//	int retval = 0;
#if 0//def SUPPORT_REE_SPI
	u32 retry_times = 0;
	u8 tempbuf[4] = {0};
	u32 chipid = 0;
	u16 irq_type = 0;
#endif
	FUNC_ENTRY();

	/* Allocate driver data */
	gf_dev = kzalloc(sizeof(struct gf_device), GFP_KERNEL);
	if (!gf_dev) {
		status = -ENOMEM;
		goto err;
	}

	spin_lock_init(&gf_dev->spi_lock);
	mutex_init(&gf_dev->buf_lock);
	mutex_init(&gf_dev->release_lock);

	INIT_LIST_HEAD(&gf_dev->device_entry);

	gf_dev->device_count     = 0;
	gf_dev->probe_finish     = 0;
	gf_dev->system_status    = 0;
	gf_dev->need_update      = 0;

	/*setup gf configurations.*/
	gf_debug(INFO_LOG, "%s: Setting gf device configuration==========\n", __func__);

	/* Initialize the driver data */
	gf_dev->spi = spi;

	/* setup SPI parameters */
	/* CPOL=CPHA=0, speed 1MHz */
	gf_dev->spi->mode = SPI_MODE_0;
	gf_dev->spi->bits_per_word = 8;
	gf_dev->spi->max_speed_hz = 1 * 1000 * 1000;
	memcpy(&gf_dev->spi_mcc, &spi_ctrdata, sizeof(struct mt_chip_conf));
	gf_dev->spi->controller_data = (void *)&gf_dev->spi_mcc;

	spi_setup(gf_dev->spi);
	gf_dev->irq = 0;
	spi_set_drvdata(spi, gf_dev);

	/* allocate buffer for SPI transfer */
	gf_dev->spi_buffer = kzalloc(bufsiz, GFP_KERNEL);
	if (gf_dev->spi_buffer == NULL) {
		gf_debug(ERR_LOG, "%s: can't alloc memory spi_buffer\n", __func__);
		status = -ENOMEM;
		goto err_buf;
	}

	/* get gpio info from dts or defination */
	status = gf_get_sensor_dts_info(gf_dev);
	if (status < 0) {
		goto err_buf;
	}

#if 0 // use always on power
	/*enable the power*/
	gf_debug(INFO_LOG, "%s: enable power start--->.\n", __func__);
	gf_dev->reg =  regulator_get(&gf_dev->spi->dev, "vfp");
	retval = regulator_set_voltage(gf_dev->reg, 2800000, 3300000);
	retval = regulator_enable(gf_dev->reg);
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_ldo_high);
	gf_hw_power_enable(1);
	gf_debug(INFO_LOG, "%s: enable power end--->.\n", __func__);
#endif

	gf_bypass_flash_gpio_cfg();
	gf_mosi_gpio_cfg(gf_dev, 1);
	gf_miso_gpio_cfg(gf_dev, 1);
	gf_hw_reset(gf_dev, 0);
	udelay(100);
	gf_miso_gpio_cfg(gf_dev, 0);

	/* run oem test */
	gf_spi_clk_enable(gf_dev, 1);

#if 0//def SUPPORT_REE_SPI
	/* read chip id */
	for (retry_times = 0; retry_times < MAX_RETRY_HW_CHECK_COUNT; retry_times++) {
		gf_spi_read_bytes_ree(gf_dev, GF3208_REG_CHIP_ID, 4, tempbuf);
		gf_debug(INFO_LOG,"%s: GF3208_REG_CHIP_ID read is 0x%02x, 0x%02x, 0x%02x, 0x%02x", __func__,
				tempbuf[0], tempbuf[1], tempbuf[2], tempbuf[3]);
		chipid = (tempbuf[2] << 16) | (tempbuf[3] << 8) | tempbuf[0];
		if (chipid == GF3208_CHIP_ID) {
			gf_debug(INFO_LOG, "%s: chip id read success, chipid = 0x%06x!\n", __func__, chipid);
			break;
		}
		gf_hw_reset(gf_dev, 100);
	}

	if (MAX_RETRY_HW_CHECK_COUNT == retry_times) {
		gf_debug(ERR_LOG, "%s: chip id read failed, chipid = 0x%06x!\n", __func__, chipid);
		status = -ENODEV;
		goto err_class;
	}

	/*INT pin check */
	gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CLEAR, 0x01FF);   //clear irq
	gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CTRL0, 0xFFFF);   //enable all irq
	mdelay(1);
	for (retry_times = 0; retry_times < MAX_RETRY_HW_CHECK_COUNT; retry_times++) {
		gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CTRL4, 0x0002);
		mdelay(1);
		gf_spi_read_word_ree(gf_dev, GF3208_REG_IRQ_STATUS, &irq_type);   //read irq type
		gf_debug(INFO_LOG, "%s: int pin test read irq type = 0x%04x!\n", __func__, irq_type);
		if (gpio_get_value(gf_dev->irq_gpio)) {
			gf_debug(INFO_LOG, "%s: int pin test sucess!\n", __func__);
			break;
		}
		mdelay(100);
	}
	if (MAX_RETRY_HW_CHECK_COUNT == retry_times) {
		gf_debug(ERR_LOG, "%s: int pin check failed!\n", __func__);
		status = -ENODEV;
		goto err_class;
	}

	/* rst pin check*/
	irq_type = 0x00;
	gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CLEAR, 0x01FF);  //clear all irq
	mdelay(1);
	for (retry_times = 0; retry_times < MAX_RETRY_HW_CHECK_COUNT; retry_times++) {
		gf_hw_reset(gf_dev, 10);
		gf_spi_read_word_ree(gf_dev, GF3208_REG_IRQ_STATUS, &irq_type);
		gf_debug(INFO_LOG, "%s: rst pin test read irq type = 0x%04x!\n", __func__, irq_type);
		gf_spi_write_word_ree(gf_dev, GF3208_REG_IRQ_CLEAR, irq_type);
		if (irq_type & 0x100) {
			gf_debug(INFO_LOG, "%s: rst pin check success!\n", __func__);
			break;
		}
		mdelay(100);
	}

	if (MAX_RETRY_HW_CHECK_COUNT == retry_times) {
		gf_debug(INFO_LOG, "%s: rst pin check failed!\n", __func__);
		status = -ENODEV;
		goto err_class;
	}
#else
	status = gf_read_chip_id(gf_dev);
	if (status < 0) {
		gf_debug(ERR_LOG, "%s: read chip id failed!\n", __func__);
		goto err_class;
	}

	status = gf_check_rst(gf_dev);
	if (status < 0) {
		gf_debug(ERR_LOG, "%s: rst pin check failed!\n", __func__);
		goto err_class;
	}

	status = gf_check_irq(gf_dev);
	if (status < 0) {
		gf_debug(ERR_LOG, "%s: irq pin check failed!\n", __func__);
		goto err_class;
	}
#endif /* SUPPORT_REE_SPI */
	gf_spi_clk_enable(gf_dev, 0);

	/* create class */
	gf_dev->class = class_create(THIS_MODULE, GF_CLASS_NAME);
	if (IS_ERR(gf_dev->class)) {
		gf_debug(ERR_LOG, "%s: Failed to create class.\n", __func__);
		status = -ENODEV;
		goto err_class;
	}

	/* get device no */
	if (GF_DEV_MAJOR > 0) {
		gf_dev->devno = MKDEV(GF_DEV_MAJOR, gf_dev->device_count++);
		status = register_chrdev_region(gf_dev->devno, 1, GF_DEV_NAME);
	} else {
		status = alloc_chrdev_region(&gf_dev->devno, gf_dev->device_count++, 1, GF_DEV_NAME);
	}
	if (status < 0) {
		gf_debug(ERR_LOG, "%s: Failed to alloc devno.\n", __func__);
		goto err_devno;
	} else {
		gf_debug(DEBUG_LOG, "%s: major=%d, minor=%d\n", __func__, MAJOR(gf_dev->devno), MINOR(gf_dev->devno));
	}

	/* create device */
	gf_dev->device = device_create(gf_dev->class, &spi->dev, gf_dev->devno, gf_dev, GF_DEV_NAME);
	if (IS_ERR(gf_dev->device)) {
		gf_debug(ERR_LOG, "%s: Failed to create device.\n", __func__);
		status = -ENODEV;
		goto err_device;
	} else {
		mutex_lock(&device_list_lock);
		list_add(&gf_dev->device_entry, &device_list);
		mutex_unlock(&device_list_lock);
	}

	/* create sysfs */
	status = sysfs_create_group(&spi->dev.kobj, &gf_debug_attr_group);
	if (status) {
		gf_debug(ERR_LOG, "%s: Failed to create sysfs file.\n", __func__);
		status = -ENODEV;
		goto err_sysfs;
	}

	/* cdev init and add */
	cdev_init(&gf_dev->cdev, &gf_fops);
	gf_dev->cdev.owner = THIS_MODULE;
	status = cdev_add(&gf_dev->cdev, gf_dev->devno, 1);
	if (status) {
		gf_debug(ERR_LOG, "%s: Failed to add cdev.\n", __func__);
		goto err_cdev;
	}

	gf_dev->input = NULL;
#ifdef CONFIG_LGE_FINGERPRINT_KEYMODE
	/*register device within input system.*/
	gf_dev->input = input_allocate_device();
	if (gf_dev->input == NULL) {
		gf_debug(ERR_LOG, "%s: Failed to allocate input device.\n", __func__);
		status = -ENOMEM;
		goto err_input;
	}

	__set_bit(EV_KEY, gf_dev->input->evbit);
	__set_bit(GF_KEY_INPUT_HOME, gf_dev->input->keybit);

	__set_bit(GF_KEY_INPUT_MENU, gf_dev->input->keybit);
	__set_bit(GF_KEY_INPUT_BACK, gf_dev->input->keybit);
	__set_bit(GF_KEY_INPUT_POWER, gf_dev->input->keybit);
	__set_bit(GF_KEY_INPUT_CAMERA, gf_dev->input->keybit);

	__set_bit(GF_NAV_INPUT_UP, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_DOWN, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_RIGHT, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_LEFT, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_CLICK, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_DOUBLE_CLICK, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_LONG_PRESS, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_HEAVY, gf_dev->input->keybit);

	gf_dev->input->name = GF_INPUT_NAME;
	if (input_register_device(gf_dev->input)) {
		gf_debug(ERR_LOG, "%s: Failed to register input device.\n", __func__);
		status = -ENODEV;
		goto err_input_2;
	}
#endif // CONFIG_LGE_FINGERPRINT_KEYMODE

	/* netlink interface init */
	status = gf_netlink_init(gf_dev);
	if (status == -1) {
#ifdef CONFIG_LGE_FINGERPRINT_KEYMODE
		mutex_lock(&gf_dev->release_lock);
		input_unregister_device(gf_dev->input);
		gf_dev->input = NULL;
		mutex_unlock(&gf_dev->release_lock);
#endif // CONFIG_LGE_FINGERPRINT_KEYMODE
		goto err_input;
	}

	gf_dev->probe_finish = 1;
	gf_dev->is_sleep_mode = 0;
	gf_debug(INFO_LOG, "%s: probe finished\n", __func__);

#ifdef USES_FPS_WAKE_UP_ENABLE
	enable_irq_wake(gf_dev->irq);
	wake_lock_init(&ttw_wl, WAKE_LOCK_SUSPEND, "gf_ttw_wl");
#endif

	FUNC_EXIT();
	return 0;

#ifdef CONFIG_LGE_FINGERPRINT_KEYMODE
err_input_2:
	mutex_lock(&gf_dev->release_lock);
	input_free_device(gf_dev->input);
	gf_dev->input = NULL;
	mutex_unlock(&gf_dev->release_lock);
#endif // CONFIG_LGE_FINGERPRINT_KEYMODE

err_input:
	cdev_del(&gf_dev->cdev);

err_cdev:
	sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);

err_sysfs:
	device_destroy(gf_dev->class, gf_dev->devno);
	list_del(&gf_dev->device_entry);

err_device:
	unregister_chrdev_region(gf_dev->devno, 1);

err_devno:
	class_destroy(gf_dev->class);

err_class:
	gf_hw_power_enable(gf_dev, 0);
	gf_spi_clk_enable(gf_dev, 0);
	kfree(gf_dev->spi_buffer);
	if (gf_dev->pinctrl_gpios != NULL) {
		devm_pinctrl_put(gf_dev->pinctrl_gpios);
		gf_dev->pinctrl_gpios = NULL;
	}

err_buf:
	mutex_destroy(&gf_dev->buf_lock);
	mutex_destroy(&gf_dev->release_lock);
	spi_set_drvdata(spi, NULL);
	gf_dev->spi = NULL;
	kfree(gf_dev);
	gf_dev = NULL;

err:
	FUNC_EXIT();

	gf_debug(ERR_LOG, "%s error code = %d\n", __func__, status);

	return status;
}

static int gf_remove(struct spi_device *spi)
{
	struct gf_device *gf_dev = spi_get_drvdata(spi);

	FUNC_ENTRY();

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq) {
		free_irq(gf_dev->irq, gf_dev);
		gf_dev->irq_count = 0;
		gf_dev->irq = 0;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (gf_dev->early_suspend.suspend)
		unregister_early_suspend(&gf_dev->early_suspend);
#else
	fb_unregister_client(&gf_dev->notifier);
#endif

	mutex_lock(&gf_dev->release_lock);
	if (gf_dev->input == NULL) {
		kfree(gf_dev);
		mutex_unlock(&gf_dev->release_lock);
		FUNC_EXIT();
		return 0;
	}
#ifdef CONFIG_LGE_FINGERPRINT_KEYMODE
	input_unregister_device(gf_dev->input);
	gf_dev->input = NULL;
#endif // CONFIG_LGE_FINGERPRINT_KEYMODE
	mutex_unlock(&gf_dev->release_lock);

	mutex_lock(&gf_dev->release_lock);
	if (gf_dev->spi_buffer != NULL) {
		kfree(gf_dev->spi_buffer);
		gf_dev->spi_buffer = NULL;
	}
	mutex_unlock(&gf_dev->release_lock);

	gf_netlink_destroy(gf_dev);
	cdev_del(&gf_dev->cdev);
	sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);
	device_destroy(gf_dev->class, gf_dev->devno);
	list_del(&gf_dev->device_entry);

	unregister_chrdev_region(gf_dev->devno, 1);
	class_destroy(gf_dev->class);
	gf_hw_power_enable(gf_dev, 0);
	gf_spi_clk_enable(gf_dev, 0);

	spin_lock_irq(&gf_dev->spi_lock);
	spi_set_drvdata(spi, NULL);
	gf_dev->spi = NULL;
	spin_unlock_irq(&gf_dev->spi_lock);

	mutex_destroy(&gf_dev->buf_lock);
	mutex_destroy(&gf_dev->release_lock);
#ifdef USES_FPS_WAKE_UP_ENABLE
	wake_lock_destroy(&ttw_wl);
#endif

	kfree(gf_dev);
	FUNC_EXIT();
	return 0;
}

/*-------------------------------------------------------------------------*/
static struct spi_driver gf_spi_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gf_of_match,
#endif
	},
	.probe = gf_probe,
	.remove = gf_remove,
};

#ifdef CONFIG_LGE_FINGERPRINT_ONE_BINARY
static int is_finger_support = 0;

static int __init get_lge_finger_support(char *str)
{
	gf_debug(INFO_LOG, "%s: lge.fingerprint=%s\n", __func__, str);

	if (str != NULL && str[0] == '1')
	{
		is_finger_support = 1;
	}

	return 0;
}

__setup("lge.fingerprint=", get_lge_finger_support);
#endif // CONFIG_LGE_FINGERPRINT_ONE_BINARY

static void async_gf_init(void *data, async_cookie_t cookie)
{
	int status = 0;

	FUNC_ENTRY();

	status = spi_register_driver(&gf_spi_driver);
	if (status < 0) {
		gf_debug(ERR_LOG, "%s: Failed to register SPI driver. status = %d\n", __func__, status);
	}

	FUNC_EXIT();

	return;
}

static int __init gf_init(void)
{
	FUNC_ENTRY();

#ifdef CONFIG_LGE_FINGERPRINT_ONE_BINARY
	if (!is_finger_support) {
		gf_debug(ERR_LOG, "%s: doesn't support fingerprint.", __func__);
		return -ENODEV;
	}
#endif // CONFIG_LGE_FINGERPRINT_ONE_BINARY

	async_schedule(async_gf_init, NULL);

	FUNC_EXIT();

	return 0;
}

static void __exit gf_exit(void)
{
	FUNC_ENTRY();

	spi_unregister_driver(&gf_spi_driver);

	FUNC_EXIT();
}

module_init(gf_init);
module_exit(gf_exit);

MODULE_AUTHOR("goodix");
MODULE_DESCRIPTION("Goodix Fingerprint chip GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF816M/GF3208/GF5206/GF5216/GF5208 TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf_spi");
