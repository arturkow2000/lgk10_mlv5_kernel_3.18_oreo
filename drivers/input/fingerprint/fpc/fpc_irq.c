/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */


#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/spi/spi.h>

#include "mt_spi.h"
#include "mt_spi_hal.h"
#include <mt-plat/mt_gpio.h>

#ifndef FPC_CONFIG_SPI_SPEED_HZ
#define FPC_CONFIG_SPI_SPEED_HZ 5000000
#endif

#define FPC1020_RESET_LOW_US 5000
#define FPC1020_RESET_HIGH1_US 100
#define FPC1020_RESET_HIGH2_US 5000

#define FPC_TTW_HOLD_TIME 1000

#define FPC_CMD_SOFT_RESET					0xF8
#define FPC_IRQ_REG_BITS_REBOOT				0xFF
#define FPC_REG_READ_INTERRUPT_WITH_CLEAR	0x1C

#define GPIO_CONFIGURE_IN(pin) gpio_direction_input(pin)
#define GPIO_CONFIGURE_OUT(pin, default) gpio_direction_output(pin, default)
#define GPIO_SET(pin, data) gpio_set_value(pin, data)
#define GPIO_GET(pin) gpio_get_value(pin)

#define DEBUG

#ifdef DEBUG
    #define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
    #define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt "\n", __FUNCTION__, __LINE__, ##args)
    #define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt "\n", __FUNCTION__,##args)
    #define SENSOR_DBG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[D]""%s : "fmt "\n", __FUNCTION__,##args)
#else
    #define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
    #define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt "\n", __FUNCTION__, __LINE__, ##args)
    #define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt "\n", __FUNCTION__,##args)
    #define SENSOR_DBG(fmt, args...)    (void)0
#endif

#define SENSOR_TAG	"[Fingerprint]"

static const char * const pctl_names[] = {
	"fpc1020_spi_active",
	"fpc1020_reset_reset",
	"fpc1020_reset_active",
	"fpc1020_cs_low",
	"fpc1020_cs_high",
	"fpc1020_cs_active",
	"fpc1020_irq_active",
};

/* fingerprint chip hardware configuration from device tree */
enum sensor_dt_entry_status {
	DT_REQUIRED,
	DT_SUGGESTED,
	DT_OPTIONAL,
};

enum sensor_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
	DT_PINCTL,
};

struct sensor_dt_to_pdata_map {
	const char	*dt_name;
	void		*ptr_data;
	enum sensor_dt_entry_status	status;
	enum sensor_dt_entry_type	type;
	int			default_val;
};
/* fingerprint chip hardware configuration device tree */

static struct mt_chip_conf spi_chip_config;

struct fpc1020_data {
	struct device *dev;
	struct spi_device *spi;
	struct input_dev *input;
	struct regulator *vreg;
	struct input_dev *idev;
	char idev_name[32];
	int event_type;
	int event_code;
	struct mutex lock;
	bool prepared;
	bool wakeup_enabled;
	bool clocks_enabled;
	struct wake_lock ttw_wl;

	int irq_gpio;
	int irq_num;
	int cs_gpio;
	int rst_gpio;
	int pinctrl;
	char *vdd;

#ifdef CONFIG_OF
	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
#endif
};

static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name);

static void fpc_spi_config(struct mt_chip_conf *configuration)
{
	configuration->setuptime = 15;
	configuration->holdtime = 15;
	configuration->high_time = 11;
	configuration->low_time = 11;

	configuration->cs_idletime = 20;
	configuration->ulthgh_thrsh = 0;

	configuration->cpol = 0;
	configuration->cpha = 0;

	configuration->rx_mlsb = 1;
	configuration->tx_mlsb = 1;

	configuration->tx_endian = 0;
	configuration->rx_endian = 0 ;

	configuration->com_mod = FIFO_TRANSFER;
	configuration->pause = 0;
	configuration->finish_intr = 1;
	configuration->deassert = 0;
	configuration->ulthigh = 0;
	configuration->tckdly = 0;
};

static int set_clks(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;
	mutex_lock(&fpc1020->lock);

	if (enable == fpc1020->clocks_enabled)
		goto out;

	if (enable) {
		mt_spi_enable_master_clk(fpc1020->spi);
		fpc1020->clocks_enabled = true;
		SENSOR_LOG("enable spi clk");
	} else {
		mt_spi_disable_master_clk(fpc1020->spi);
		fpc1020->clocks_enabled = false;
		SENSOR_LOG("disable spi clk");
	}

out:
	mutex_unlock(&fpc1020->lock);
	return rc;
}

#ifdef CONFIG_FPC_CHECK_CONTACT
static int fpc_sw_reset(struct spi_device *spidev)
{
	int rc = 0;
	/* 4bytes aligned */
	u8 txrx_buf[4] = { FPC_CMD_SOFT_RESET };

	struct spi_transfer t = {
		.tx_buf	= txrx_buf,
		.len	= 1,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	rc = spi_sync(spidev, &m);
	if (rc != 0) {
		SENSOR_ERR("spi transfer fail");
		goto exit;
	}
exit:
	return rc;
}

static int fpc_check_irq_after_reset(struct spi_device *spidev)
{
	int rc = 0;
	/* 4bytes aligned */
	u8 txrx_buf[4] = {FPC_REG_READ_INTERRUPT_WITH_CLEAR, 0x00};

	struct spi_transfer t = {
		.tx_buf	= txrx_buf,
		.rx_buf	= txrx_buf,
		.len	= 2,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	rc = spi_sync(spidev, &m);
	if (rc != 0) {
		SENSOR_ERR("spi transfer fail");
		goto exit;
	}

	if (txrx_buf[1] != FPC_IRQ_REG_BITS_REBOOT) {
		SENSOR_ERR("***IRQ register, expected 0x%x, got 0x%x***",
			FPC_IRQ_REG_BITS_REBOOT,
			txrx_buf[1]);
		//rc = -EIO;
	}

exit:
	return rc;
}
#endif

/*
static int hw_reset(struct fpc1020_data *fpc1020)
{
	int irq_gpio;

	GPIO_SET(fpc1020->rst_gpio, 0);
	usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 100);

	GPIO_SET(fpc1020->rst_gpio, 1);
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	irq_gpio = GPIO_GET(fpc1020->irq_gpio);
	SENSOR_DBG("IRQ after reset %d", irq_gpio);

	return 0;
}

void fpc1020_hw_reset(struct fpc1020_data *fpc1020)
{
	SENSOR_FUN();

	GPIO_SET(fpc1020->rst_gpio, GPIO_OUT_ONE);
	udelay(FPC1020_RESET_HIGH1_US);

	GPIO_SET(fpc1020->rst_gpio, GPIO_OUT_ZERO);
	udelay(FPC1020_RESET_LOW_US);
	udelay(FPC1020_RESET_LOW_US);

	GPIO_SET(fpc1020->rst_gpio, GPIO_OUT_ONE);
	udelay(FPC1020_RESET_HIGH2_US);

	GPIO_SET(fpc1020->rst_gpio, GPIO_OUT_ONE);
	udelay(FPC1020_RESET_HIGH1_US);

	GPIO_SET(fpc1020->rst_gpio, GPIO_OUT_ZERO);
	udelay(FPC1020_RESET_LOW_US);
	udelay(FPC1020_RESET_LOW_US);

	GPIO_SET(fpc1020->rst_gpio, GPIO_OUT_ONE);
	udelay(FPC1020_RESET_HIGH2_US);
}
*/

static int fpc1020_hw_reset(struct  fpc1020_data *fpc1020)
{
	int irq_gpio;
	int rc = 0;

	SENSOR_FUN();
	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	SENSOR_DBG("IRQ before reset %d", irq_gpio);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_HIGH2_US, FPC1020_RESET_HIGH2_US + 100);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	if (irq_gpio == 0)
		SENSOR_DBG("***IRQ after reset %d *** The Interrupt don't occur.", irq_gpio);
exit:
	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset"))) {
		rc = fpc1020_hw_reset(fpc1020);
	} else {
		return -EINVAL;
	}
	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc1020->wakeup_enabled = true;
		smp_wmb();
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc1020->wakeup_enabled = false;
		smp_wmb();
	} else {
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

/**
 * sysfs node for sending event to make the system interactive,
 * i.e. waking up
 */
static ssize_t do_wakeup_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (count > 0) {
		input_report_key(fpc1020->idev, KEY_POWER, 1);
		input_report_key(fpc1020->idev, KEY_POWER, 0);
		input_sync(fpc1020->idev);
	} else {
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(do_wakeup, S_IWUSR, NULL, do_wakeup_set);

static ssize_t clk_enable_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	return set_clks(fpc1020, (*buf == '1')) ? : count;
}
static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
			     struct device_attribute *attribute,
			     char *buffer)
{
	int irq;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);

	irq = gpio_get_value(fpc1020->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}


/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
			     struct device_attribute *attribute,
			     const char *buffer, size_t count)
{
	SENSOR_FUN();
	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static struct attribute *attributes[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_do_wakeup.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	SENSOR_LOG("fpc1020->irq_num = %d", gpio_to_irq(fpc1020->irq_gpio));

	/* Make sure 'wakeup_enabled' is updated before using it
	** since this is interrupt context (other thread...) */
	smp_rmb();

	if (fpc1020->wakeup_enabled) {
		wake_lock_timeout(&fpc1020->ttw_wl,
			msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}

	sysfs_notify(&fpc1020->input->dev.kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	size_t i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
					fpc1020->pinctrl_state[i]);
			if (rc)
				SENSOR_ERR("cannot select '%s'", name);
			else
				SENSOR_DBG("Selected '%s'", name);
			goto exit;
		}
	}

	rc = -EINVAL;
	SENSOR_ERR("'%s' not found", name);

exit:
	return rc;
}
/*
static int device_prepare(struct  fpc1020_data *fpc1020, bool enable)
{
	int rc;

	SENSOR_FUN();

	mutex_lock(&fpc1020->lock);
	if (enable && !fpc1020->prepared) {
		spi_bus_lock(fpc1020->spi->master);
		fpc1020->prepared = true;
		select_pin_ctl(fpc1020, "fpc1020_reset_reset");

		usleep_range(100, 1000);

		(void)select_pin_ctl(fpc1020, "fpc1020_cs_high");
		(void)select_pin_ctl(fpc1020, "fpc1020_reset_active");

		usleep_range(100, 200);

		(void)select_pin_ctl(fpc1020, "fpc1020_cs_active");

	} else if (!enable && fpc1020->prepared) {
		rc = 0;

		(void)select_pin_ctl(fpc1020, "fpc1020_cs_high");
		(void)select_pin_ctl(fpc1020, "fpc1020_reset_reset");

		usleep_range(100, 1000);

		(void)select_pin_ctl(fpc1020, "fpc1020_cs_low");

		fpc1020->prepared = false;
		spi_bus_unlock(fpc1020->spi->master);
	} else {
		rc = 0;
	}

	mutex_unlock(&fpc1020->lock);

	return rc;
}
*/
static int fpc1020_get_gpio_dts_info(struct fpc1020_data *fpc1020)
{
	int ret,err = 0;
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
		{"fpc,gpio_irq",	&fpc1020->irq_gpio,				DT_REQUIRED,	DT_U32,		0},
		{"fpc,gpio_rst",	&fpc1020->rst_gpio,				DT_REQUIRED,	DT_U32,		0},
		{"fpc,gpio_cs",		&fpc1020->cs_gpio,				DT_REQUIRED,	DT_U32,		0},
		{"fpc,pinctrl",		&fpc1020->fingerprint_pinctrl,	DT_REQUIRED,	DT_PINCTL,	0},
		{NULL,				NULL,							0,				0,			0},
	};

	if (!np) {
		SENSOR_ERR("no of node found");
		return -EINVAL;
	}

	for (itr = map; itr->dt_name ; ++itr) {
		switch (itr->type) {
			case DT_GPIO:
				ret = of_get_named_gpio(np, itr->dt_name, 0);
				if (ret >= 0) {
					*((int *) itr->ptr_data) = ret;
					ret = 0;
				}
				break;
			case DT_U32:
				ret = of_property_read_u32(np, itr->dt_name,
						(u32 *) itr->ptr_data);
				break;
			case DT_BOOL:
				*((bool *) itr->ptr_data) =
					of_property_read_bool(np, itr->dt_name);
				ret = 0;
				break;
			case DT_PINCTL:
				fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
				if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
					ret = PTR_ERR(fpc1020->fingerprint_pinctrl);
					SENSOR_ERR("can't find fingerprint pinctrl");
					return ret;
				}
				ret = 0;
				break;
			default:
				SENSOR_ERR("%d is an unknown DT entry type", itr->type);
				ret = -EBADE;
		}

		//SENSOR_DBG("DT entry ret:%d name:%s val:%d", ret, itr->dt_name, *((int *)itr->ptr_data));

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;
			if (itr->status < DT_OPTIONAL) {
				SENSOR_LOG("Missing '%s' DT entry", itr->dt_name);
				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}
	return err;
}

static int fpc1020_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int rc = 0;
	int irqf;
	size_t i;
	struct fpc1020_data *fpc1020;

	SENSOR_FUN();

	fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020) {
		SENSOR_ERR("failed to allocate memory for struct fpc1020_data");
		rc = -ENOMEM;
		goto exit;
	}

	mutex_init(&fpc1020->lock);

	fpc1020->dev = dev;
	dev_set_drvdata(dev, fpc1020);
	fpc1020->spi = spi;
	fpc1020->spi->mode = SPI_MODE_0;
	fpc1020->spi->bits_per_word = 8;
	fpc1020->spi->max_speed_hz = FPC_CONFIG_SPI_SPEED_HZ;

	fpc_spi_config(&spi_chip_config);
	fpc1020->spi->controller_data = (void *)&spi_chip_config;

	rc = spi_setup(fpc1020->spi);
	if (rc < 0) {
		SENSOR_ERR("spi setup failed.");
		goto exit;
	}

	rc = fpc1020_get_gpio_dts_info(fpc1020);
	if (rc < 0) {
		SENSOR_ERR("fpc1020_get_gpio_dts_info failed.");
		return rc;
	}

	fpc1020->vdd = NULL;
	// get vdd name
	rc = of_property_read_string(dev->of_node, "fpc,vdd", (const char**) &fpc1020->vdd);
	if (rc) {
		SENSOR_ERR("can't get property fpc,vdd");
	}

	// if fps doesn't use always on power, then turn on power rail
	if (fpc1020->vdd != NULL)
	{
		SENSOR_LOG("Using %s as VDD.", fpc1020->vdd);

		fpc1020->vreg = regulator_get(dev, fpc1020->vdd);
		if (IS_ERR(fpc1020->vreg)) {
			SENSOR_ERR("could not get vreg");
			goto exit;
		}

		regulator_set_voltage(fpc1020->vreg, 1800000UL, 1800000UL);
		rc = regulator_enable(fpc1020->vreg);
		if (rc) {
			SENSOR_ERR("could not enable vreg");
			goto exit;
		}
	}
	else
	{
		SENSOR_LOG("use always on power");
	}

	SENSOR_DBG("Using GPIO#%d as IRQ.", fpc1020->irq_gpio);
	SENSOR_DBG("Using GPIO#%d as RST.", fpc1020->rst_gpio);

	/* Configure the direction of the gpios */
	rc = GPIO_CONFIGURE_IN(fpc1020->irq_gpio);
	if (rc < 0) {
		SENSOR_ERR("gpio_direction_input failed for INT.");
		return rc;
	}

	rc = GPIO_CONFIGURE_OUT(fpc1020->rst_gpio, 1);
	if (rc < 0) {
		SENSOR_ERR("gpio_direction_output failed for RST.");
		return rc;
	}

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			SENSOR_ERR("cannot find '%s'", n);
			rc = -EINVAL;
			goto exit;
		}
		SENSOR_DBG("found pin control %s", n);
		fpc1020->pinctrl_state[i] = state;
	}

	rc = select_pin_ctl(fpc1020, "fpc1020_cs_low");
	if (rc)
		goto exit;
	rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
	if (rc)
		goto exit;

	rc = select_pin_ctl(fpc1020, "fpc1020_cs_high");
	if (rc)
		goto exit;
	rc = select_pin_ctl(fpc1020, "fpc1020_cs_active");
	if (rc)
		goto exit;

	// Check Connection Status by HW reset
	rc = fpc1020_hw_reset(fpc1020);
	if (rc)
		goto exit;
	// Check Connection Status by SW reset
	rc = select_pin_ctl(fpc1020, "fpc1020_spi_active");
	if (rc)
		goto exit;

	set_clks(fpc1020, true);

#ifdef CONFIG_FPC_CHECK_CONTACT
	rc = fpc_sw_reset(fpc1020->spi);
	if (rc)
		goto exit;

	usleep_range(FPC1020_RESET_HIGH2_US, FPC1020_RESET_HIGH2_US + 100);

	rc = gpio_get_value(fpc1020->irq_gpio);
	if (rc != 1) {
		SENSOR_ERR("***IRQ still low after sw_reset***");
		//goto exit;
	}

	rc = fpc_check_irq_after_reset(fpc1020->spi);
	if (rc)
		goto exit;
#endif

	set_clks(fpc1020, false);
/*
	fpc1020->irq_num = fpc1020_get_irqnum();
	printk(KERN_INFO "irq_num= %d\n", fpc1020->irq_num);

	rc = of_property_read_u32(np, "fpc,event-type", &val);
	fpc1020->event_type = rc < 0 ? EV_MSC : val;
	printk(KERN_INFO "fpc,event-type %d\n", fpc1020->event_type);

	rc = of_property_read_u32(np, "fpc,event-code", &val);
	fpc1020->event_code = rc < 0 ? MSC_SCAN : val;
	printk(KERN_INFO "fpc,event-event_code %d\n", fpc1020->event_code;

	fpc1020->idev = devm_input_allocate_device(dev);
	if (!fpc1020->idev) {
		printk(KERN_INFO "failed to allocate input device\n");
		rc = -ENOMEM;
		goto exit;
	}
	input_set_capability(fpc1020->idev,
		fpc1020->event_type, fpc1020->event_code);

	if (!of_property_read_string(np, "input-device-name", &idev_name)) {
		fpc1020->idev->name = idev_name;
	} else {
		snprintf(fpc1020->idev_name, sizeof(fpc1020->idev_name),
			"fpc_irq@%s", dev_name(dev));
		fpc1020->idev->name = fpc1020->idev_name;
	}

	set_bit(EV_KEY,	fpc1020->idev->evbit);
	set_bit(EV_PWR,	fpc1020->idev->evbit);
	set_bit(KEY_WAKEUP, fpc1020->idev->keybit);
	set_bit(KEY_POWER, fpc1020->idev->keybit);
	rc = input_register_device(fpc1020->idev);
	fpc1020->wakeup_enabled = false;
	if (rc) {
		printk(KERN_INFO "failed to register input device\n");
		goto exit;
	}
*/
	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}
	/* register input device */
	fpc1020->input = input_allocate_device();
	if(!fpc1020->input) {
		dev_err(dev, "input_allocate_deivce failed.");
		goto exit;
	}

	fpc1020->input->name = "fingerprint";
	fpc1020->input->dev.init_name = "lge_fingerprint";

	input_set_drvdata(fpc1020->input, fpc1020);
	rc = input_register_device(fpc1020->input);
	if(rc) {
		dev_err(dev, "input_register_device failed.");
		input_free_device(fpc1020->input);
		fpc1020->input = NULL;
		goto exit;
	}

	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
				NULL, fpc1020_irq_handler, irqf,
				dev_name(dev), fpc1020);
	if (rc) {
		SENSOR_ERR("could not request irq %d", gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
	}

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
	wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");

	rc = sysfs_create_group(&fpc1020->input->dev.kobj, &attribute_group);
	if (rc) {
		SENSOR_ERR("could not create sysfs");
		goto exit;
	}

	//(void)device_prepare(fpc1020, true);

	SENSOR_LOG("ok");

	return rc;

exit:
	if (fpc1020 != NULL) {
		if (fpc1020->fingerprint_pinctrl != NULL) {
			SENSOR_ERR("clear pinctrl");
			devm_pinctrl_put(fpc1020->fingerprint_pinctrl);
			fpc1020->fingerprint_pinctrl = NULL;
		}

		if (fpc1020->input != NULL) {
			SENSOR_ERR("clear input device");
			input_unregister_device(fpc1020->input);
			input_free_device(fpc1020->input);
			fpc1020->input = NULL;
		}
		set_clks(fpc1020, false);
    }

	return rc;
}

static int fpc1020_remove(struct spi_device *spi)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(&spi->dev);

	SENSOR_FUN();

	sysfs_remove_group(&fpc1020->input->dev.kobj, &attribute_group);
	mutex_destroy(&fpc1020->lock);
	wake_lock_destroy(&fpc1020->ttw_wl);
	return 0;
}

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{ .compatible = "mediatek,fpc-fp", },
	{}
};

MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct spi_driver spi_driver = {
	.driver = {
		.name	= "fpc1020",
		.owner	= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
		.bus	= &spi_bus_type,
	},
	.probe	= fpc1020_probe,
	.remove	= fpc1020_remove
};

static int __init fpc1020_init(void)
{
	SENSOR_FUN();

	if (spi_register_driver(&spi_driver)) {
		SENSOR_ERR("register spi driver fail");
		return -EINVAL;
	}

	return 0;
}

static void __exit fpc1020_exit(void)
{
	SENSOR_FUN();
	spi_unregister_driver(&spi_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
