/*
 * Copyright (C) 2012 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/tca6418_ioexpander.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>

#include <linux/leds.h>
#include <linux/i2c.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/input.h>
#include <linux/list.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>

#define IOEXPANDER_I2C_NAME "tca6418"

#define TCA6418E_Device			0x68
#define TCA6418E_Reg_GPIO_DAT_STAT1	0x14
#define TCA6418E_Reg_GPIO_DAT_STAT2	0x15
#define TCA6418E_Reg_GPIO_DAT_STAT3	0x16
#define TCA6418E_Reg_GPIO_DAT_OUT1	0x17
#define TCA6418E_Reg_GPIO_DAT_OUT2	0x18
#define TCA6418E_Reg_GPIO_DAT_OUT3	0x19
#define TCA6418E_Reg_GPIO_DIR1		0x23
#define TCA6418E_Reg_GPIO_DIR2		0x24
#define TCA6418E_Reg_GPIO_DIR3		0x25
#define TCA6418E_Reg_GPIO_PULLDOWN1	0x2C
#define TCA6418E_Reg_GPIO_PULLDOWN2	0x2D
#define TCA6418E_Reg_GPIO_PULLDOWN3	0x2E
#define TCA6418E_GPIO_NUM		14

struct ioexp_i2c_platform_data {
	struct platform_device *ioexp_devices;
	int			num_devices;
	int			reset_gpio;
	void 			*dev_id;
	void (*setup_gpio)(void);
	void (*reset_chip)(void);
};

struct ioexp_i2c_client_data {
	struct mutex ioexp_i2c_rw_mutex;
	struct mutex ioexp_set_gpio_mutex;
	uint16_t version;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	atomic_t ioexp_is_suspend;
};

struct ioexp_ops {
	int (*init_ioexp_func)(struct i2c_client *);
};

int ioexp_i2c_read(uint8_t addr, uint8_t *data, int length);
int ioexp_i2c_write(uint8_t addr, uint8_t *data, int length);
void ioexp_register_ops(struct ioexp_ops *ops);

#define I2C_READ_RETRY_TIMES			10
#define I2C_WRITE_RETRY_TIMES			10
#define IOEXP_I2C_WRITE_BLOCK_SIZE		4

static struct i2c_client *private_ioexp_client;

static int ioexp_rw_delay;

static char *hex2string(uint8_t *data, int len)
{
	static char buf[IOEXP_I2C_WRITE_BLOCK_SIZE*4];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

static int i2c_read_ioexpreg(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;

	struct ioexp_i2c_client_data *cdata;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	cdata = i2c_get_clientdata(client);
	mutex_lock(&cdata->ioexp_i2c_rw_mutex);


	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msgs, 2) == 2)
			break;
		msleep(ioexp_rw_delay);
	}

	mutex_unlock(&cdata->ioexp_i2c_rw_mutex);
	dev_info(&client->dev, "R [%02X] = %s\n",
			addr, hex2string(data, length));

	if (retry > I2C_READ_RETRY_TIMES) {
		printk(KERN_ERR "%s(), [IOEXP_ERR] i2c_read_ioexpreg retry over %d\n", __func__, I2C_READ_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_write_ioexpreg(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	uint8_t buf[6];
	int i;
	struct ioexp_i2c_client_data *cdata;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	dev_info(&client->dev, "W [%02X] = %s\n", addr, hex2string(data, length));


	cdata = i2c_get_clientdata(client);


	if (length + 1 > IOEXP_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "[IOEXP_ERR] i2c_write_ioexpreg length too long\n");
		return -E2BIG;
	}


	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	mutex_lock(&cdata->ioexp_i2c_rw_mutex);

	printk(KERN_INFO "%s(), i2c_transfer(addr:0x%x, len:%d)\n", __func__, msg->addr, msg->len);


	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(ioexp_rw_delay);
	}


	if (retry > I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "[IOEXP_ERR] i2c_write_ioexpreg retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
		mutex_unlock(&cdata->ioexp_i2c_rw_mutex);
		return -EIO;
	}

	mutex_unlock(&cdata->ioexp_i2c_rw_mutex);

	return 0;
}

int ioexp_i2c_read(uint8_t addr, uint8_t *data, int length)
{
	struct i2c_client *client = private_ioexp_client;



	if (!client)	{
		printk(KERN_ERR "[IOEXP_ERR] %s: dataset: client is empty\n", __func__);
		return -EIO;
	}



	if (i2c_read_ioexpreg(client, addr, data, length) < 0)	{
		dev_err(&client->dev, "[IOEXP_ERR] %s: write ioexp i2c fail\n", __func__);
		return -EIO;
	}

	return 0;
}

EXPORT_SYMBOL(ioexp_i2c_read);

int ioexp_i2c_write(uint8_t addr, uint8_t *data, int length)
{
	struct i2c_client *client = private_ioexp_client;



	if (!client) {
		printk(KERN_ERR "[IOEXP_ERR] %s: dataset: client is empty\n", __func__);
		return -EIO;
	}


	if (i2c_write_ioexpreg(client, addr, data, length) < 0)	{
		dev_err(&client->dev, "[IOEXP_ERR] %s: write ioexp i2c fail\n", __func__);
		return -EIO;
	}

	return 0;
}

EXPORT_SYMBOL(ioexp_i2c_write);

int ioexp_gpio_set_cfg(uint8_t gpio, uint8_t direction, uint8_t pulldown_disable, uint8_t level)
{
	uint8_t addr;
	uint8_t rdata = 0;
	uint8_t wdata = 0;

	unsigned char TCA6418_GPIO_BITMAP[18]={7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7,0,1};

	struct ioexp_i2c_client_data *cdata;
	struct i2c_client *client = private_ioexp_client;


	if (!client)	{
		printk(KERN_ERR "[IOEXP_ERR] %s: dataset: client is empty\n", __func__);
		return -EIO;
	}


	cdata = i2c_get_clientdata(client);
	mutex_lock(&cdata->ioexp_set_gpio_mutex);



	if ( (gpio >= 0) && (gpio <= 7) )
		addr = TCA6418E_Reg_GPIO_DAT_OUT1;
	else if ( (gpio >= 8) && (gpio <= 15) )
		addr = TCA6418E_Reg_GPIO_DAT_OUT2;
	else if ( (gpio >= 16) && (gpio <= 17) )
		addr = TCA6418E_Reg_GPIO_DAT_OUT3;
	else {
		printk(KERN_ERR "[IOEXP_ERR] %s: Pin not support!\n", __func__);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	}


	if (ioexp_i2c_read(addr, &rdata, 1)) {
		printk(KERN_ERR "[IOEXP_ERR] %s: readdata error, addr:0x%x\n", __func__, addr);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	}


	if (level)
		wdata = rdata | (1 << TCA6418_GPIO_BITMAP[gpio]);
	else
		wdata = rdata & ~(1 << TCA6418_GPIO_BITMAP[gpio]);


	if ( ioexp_i2c_write(addr, &wdata, 1) ) {
		printk(KERN_ERR "[IOEXP_ERR] %s: writedata failed, addr:0x%x, data:0x%x\n", __func__, addr, wdata);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	} else
		printk(KERN_INFO "[IOEXP_INFO] %s: OK, addr:0x%x, data=0x%x\n", __func__, addr, wdata);



	rdata = 0;
	wdata = 0;
	if ( (gpio >= 0) && (gpio <= 7) )
		addr = TCA6418E_Reg_GPIO_DIR1;
	else if ( (gpio >= 8) && (gpio <= 15) )
		addr = TCA6418E_Reg_GPIO_DIR2;
	else if ( (gpio >= 16) && (gpio <= 17) )
		addr = TCA6418E_Reg_GPIO_DIR3;
	else {
		printk(KERN_ERR "[IOEXP_ERR] %s: Pin not support!\n", __func__);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	}


	if (ioexp_i2c_read(addr, &rdata, 1)) {
		printk(KERN_ERR "[IOEXP_ERR] %s: readdata error, addr:0x%x\n", __func__, addr);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	}


	if (direction)
		wdata = rdata | (1 << TCA6418_GPIO_BITMAP[gpio]);
	else
		wdata = rdata & ~(1 << TCA6418_GPIO_BITMAP[gpio]);


	if ( ioexp_i2c_write(addr, &wdata, 1) ) {
		printk(KERN_ERR "[IOEXP_ERR] %s: writedata failed, addr:0x%x, data:0x%x\n", __func__, addr, wdata);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	} else
		printk(KERN_INFO "[IOEXP_INFO] %s: OK, addr:0x%x, data=0x%x\n", __func__, addr, wdata);



	rdata = 0;
	wdata = 0;
	if ( (gpio >= 0) && (gpio <= 7) )
		addr = TCA6418E_Reg_GPIO_PULLDOWN1;
	else if ( (gpio >= 8) && (gpio <= 15) )
		addr = TCA6418E_Reg_GPIO_PULLDOWN2;
	else if ( (gpio >= 16) && (gpio <= 17) )
		addr = TCA6418E_Reg_GPIO_PULLDOWN3;
	else {
		printk(KERN_ERR "[IOEXP_ERR] %s: Pin not support!\n", __func__);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	}


	if (ioexp_i2c_read(addr, &rdata, 1)) {
		printk(KERN_ERR "[IOEXP_ERR] %s: readdata error, addr:0x%x\n", __func__, addr);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	}


	if (pulldown_disable)
		wdata = rdata | (1 << TCA6418_GPIO_BITMAP[gpio]);
	else
		wdata = rdata & ~(1 << TCA6418_GPIO_BITMAP[gpio]);


	if ( ioexp_i2c_write(addr, &wdata, 1) ) {
		printk(KERN_ERR "[IOEXP_ERR] %s: writedata failed, addr:0x%x, data:0x%x\n", __func__, addr, wdata);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	} else
		printk(KERN_INFO "[IOEXP_INFO] %s: OK, addr:0x%x, data=0x%x\n", __func__, addr, wdata);

	mutex_unlock(&cdata->ioexp_set_gpio_mutex);
    return 0;
}

EXPORT_SYMBOL(ioexp_gpio_set_cfg);

int ioexp_gpio_set_value(uint8_t gpio, uint8_t value)
{
	uint8_t addr;
	uint8_t rdata = 0;
	uint8_t wdata = 0;

	unsigned char TCA6418_GPIO_BITMAP[18]={7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7,0,1};

	struct ioexp_i2c_client_data *cdata;
	struct i2c_client *client = private_ioexp_client;


	if (!client)	{
		printk(KERN_ERR "[IOEXP_ERR] %s: dataset: client is empty\n", __func__);
		return -EIO;
	}


	cdata = i2c_get_clientdata(client);
	mutex_lock(&cdata->ioexp_set_gpio_mutex);


	if ( (gpio >= 0) && (gpio <= 7) )
		addr = TCA6418E_Reg_GPIO_DAT_OUT1;
	else if ( (gpio >= 8) && (gpio <= 15) )
		addr = TCA6418E_Reg_GPIO_DAT_OUT2;
	else if ( (gpio >= 16) && (gpio <= 17) )
		addr = TCA6418E_Reg_GPIO_DAT_OUT3;
	else {
		printk(KERN_ERR "[IOEXP_ERR] %s: Pin not support!\n", __func__);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	}


	if (ioexp_i2c_read(addr, &rdata, 1)) {
		printk(KERN_ERR "[IOEXP_ERR] %s: readdata error, addr:0x%x\n", __func__, addr);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	}


	if (value)
		wdata = rdata | (1 << TCA6418_GPIO_BITMAP[gpio]);
	else
		wdata = rdata & ~(1 << TCA6418_GPIO_BITMAP[gpio]);


	if ( ioexp_i2c_write(addr, &wdata, 1) ) {
		printk(KERN_ERR "[IOEXP_ERR] %s: writedata failed, addr:0x%x, data:0x%x\n", __func__, addr, wdata);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	} else
		printk(KERN_INFO "[IOEXP_INFO] %s: OK, addr:0x%x, data=0x%x\n", __func__, addr, wdata);

	mutex_unlock(&cdata->ioexp_set_gpio_mutex);
	return 0;
}

EXPORT_SYMBOL(ioexp_gpio_set_value);


int ioexp_gpio_get_value(uint8_t gpio)
{
	uint8_t addr;
	uint8_t rdata = 0;
	int get_value = 0;

	unsigned char TCA6418_GPIO_BITMAP[18]={7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7,0,1};

	struct ioexp_i2c_client_data *cdata;
	struct i2c_client *client = private_ioexp_client;
	if (!client)	{
		printk(KERN_ERR "[IOEXP_ERR] %s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	cdata = i2c_get_clientdata(client);
	mutex_lock(&cdata->ioexp_set_gpio_mutex);


	if ( (gpio >= 0) && (gpio <= 7) )
		addr = TCA6418E_Reg_GPIO_DAT_STAT1;
	else if ( (gpio >= 8) && (gpio <= 15) )
		addr = TCA6418E_Reg_GPIO_DAT_STAT2;
	else if ( (gpio >= 16) && (gpio <= 17) )
		addr = TCA6418E_Reg_GPIO_DAT_STAT3;
	else {
		printk(KERN_ERR "[IOEXP_ERR] %s: pin not support!\n", __func__);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	}


	if (ioexp_i2c_read(addr, &rdata, 1)) {

		printk(KERN_ERR "[IOEXP_ERR] %s: readdata error, addr:0x%x\n", __func__, addr);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	} else {

		get_value = (rdata >> TCA6418_GPIO_BITMAP[gpio]) & 0x1;
		printk(KERN_INFO "[IOEXP_INFO] %s: OK, addr:0x%x, rdata = %x, get_value:%d\n", __func__, addr, rdata, get_value);
	}

	mutex_unlock(&cdata->ioexp_set_gpio_mutex);
	return get_value;
}

EXPORT_SYMBOL(ioexp_gpio_get_value);

int ioexp_gpio_get_direction(uint8_t gpio)
{
	uint8_t addr;
	uint8_t rdata = 0;
	int get_direction = 0;

	unsigned char TCA6418_GPIO_BITMAP[18]={7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7,0,1};

	struct ioexp_i2c_client_data *cdata;
	struct i2c_client *client = private_ioexp_client;
	if (!client)	{
		printk(KERN_ERR "[IOEXP_ERR] %s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	cdata = i2c_get_clientdata(client);
	mutex_lock(&cdata->ioexp_set_gpio_mutex);


	if ( (gpio >= 0) && (gpio <= 7) )
		addr = TCA6418E_Reg_GPIO_DIR1;
	else if ( (gpio >= 8) && (gpio <= 15) )
		addr = TCA6418E_Reg_GPIO_DIR2;
	else if ( (gpio >= 16) && (gpio <= 17) )
		addr = TCA6418E_Reg_GPIO_DIR3;
	else {
		printk(KERN_ERR "[IOEXP_ERR] %s: pin not support!\n", __func__);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	}


	if (ioexp_i2c_read(addr, &rdata, 1)) {

		printk(KERN_ERR "[IOEXP_ERR] %s: readdata error, addr:0x%x\n", __func__, addr);
		mutex_unlock(&cdata->ioexp_set_gpio_mutex);
		return -1;
	} else {

		get_direction = (rdata >> TCA6418_GPIO_BITMAP[gpio]) & 0x1;

	}

	mutex_unlock(&cdata->ioexp_set_gpio_mutex);
	return get_direction;
}

EXPORT_SYMBOL(ioexp_gpio_get_direction);

void ioexp_print_gpio_status()
{
	int i = 0;
	int value;
	int direction;

	for (i = 0; i < TCA6418E_GPIO_NUM; i++)
	{
		value = ioexp_gpio_get_value(i);
		direction = ioexp_gpio_get_direction(i);
		printk(KERN_INFO "EXP_GPIO_%d: oe=%s, value=%d", i, direction ? "OU" : "IN", value);
	}

}

EXPORT_SYMBOL( ioexp_print_gpio_status);

static int ioexp_i2c_remove(struct i2c_client *client)
{
	struct ioexp_i2c_platform_data *pdata;
	struct ioexp_i2c_client_data *cdata;

	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cdata->early_suspend);
#endif

	if (client->irq)
		free_irq(client->irq, &client->dev);

	if (pdata->reset_gpio != 0)
		gpio_free(pdata->reset_gpio);

	kfree(cdata);

	return 0;
}

#if 0
static int tca6418_parse_dt(struct device *dev, struct ioexp_i2c_platform_data *pdata)
{
	struct device_node *dt = dev->of_node;

	if (pdata == NULL) {
		printk(KERN_ERR "%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}
	pdata->reset_gpio = of_get_named_gpio_flags(dt, "tca6418,reset_gpio",
				0, NULL);
	if (pdata->reset_gpio < 0) {
		printk(KERN_ERR "%s: tca6418,reset_gpio not found", __func__);
		return pdata->reset_gpio;
	}
	else
		printk(KERN_INFO "%s: reset_gpio = %d\n", __func__, pdata->reset_gpio);

	return 0;
}
#endif

static void register_ioexp_devices(struct platform_device *devices, int num)
{
	int i;
	printk(KERN_INFO "%s()+, num:%d, addr:0x%x\n", __func__, num, private_ioexp_client->addr);

	for (i = 0; i < num; i++) {
		platform_device_register(devices + i);
		dev_set_drvdata(&(devices + i)->dev, private_ioexp_client);
	}

	printk(KERN_INFO "%s()-, num:%d, i:%d\n", __func__, num, i);
}

static void ioexp_gpio_init(void)
{
	ioexp_gpio_set_cfg(FINGER_LDO_EN,	IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(TDMB_FM_SW,		IOEXP_OUTPUT, 	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(HIFI_MODE2,		IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(VT_CAM_VIO_EN,	IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(NFC_MODE,		IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(FLASH_CNTL_EN,	IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(DMB_EN,	        IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(CAM_AVDD_LDO_EN,	IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(HIFI_LDO_SW,		IOEXP_OUTPUT, 	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(VT_CAM_DVDD_EN,	IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(SPK_AMP_EN,		IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(LCD_LDO_EN,		IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(HIFI_SW_SEL,		IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
	ioexp_gpio_set_cfg(VCONN_BOOST_EN,	IOEXP_OUTPUT,	IOEXP_PULLDOWN, 0);
}

static int ioexp_i2c_probe(struct i2c_client *client
	, const struct i2c_device_id *id)
{
	struct ioexp_i2c_platform_data *pdata;
	struct ioexp_i2c_client_data *cdata;
	int ret = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "i2c_check_functionality error\n");
		goto err_exit;
	}

	printk(KERN_INFO "%s()+\n", __func__);

	cdata = kzalloc(sizeof(struct ioexp_i2c_client_data), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		dev_err(&client->dev, "[IOEXP_PROBE_ERR] failed on allocate cdata\n");
		goto err_cdata;
	}

	i2c_set_clientdata(client, cdata);

	mutex_init(&cdata->ioexp_i2c_rw_mutex);
	mutex_init(&cdata->ioexp_set_gpio_mutex);

	private_ioexp_client = client;



	pdata = kzalloc(sizeof(struct ioexp_i2c_platform_data), GFP_KERNEL);;

	if (!pdata) {
		ret = -EBUSY;
		dev_err(&client->dev, "[IOEXP_PROBE_ERR] failed to allocate pdata\n");
		goto err_exit;
	}
	pdata->dev_id = (void *)&client->dev;

	ioexp_rw_delay = 5;
#if 0
	if (client->dev.of_node) {
		printk(KERN_INFO "Device Tree parsing.");

		ret = tca6418_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "%s: tca6418_parse_dt "
					"for pdata failed. err = %d",
					__func__, ret);
			goto err_exit;
		}
	} else {
		if (client->dev.platform_data != NULL) {
			memcpy(pdata, client->dev.platform_data,
			       sizeof(*pdata));
		}
	}

	if (pdata->reset_gpio >= 0) {
		ret = gpio_request(pdata->reset_gpio, "tca6418-reset-gpio");
		if (ret < 0) {
			dev_err(&client->dev, "[IOEXP_PROBE_ERR] failed on request reset gpio\n");
			goto err_exit;
		}
		else {
			if(gpio_get_value(pdata->reset_gpio) != 1) {
				printk(KERN_INFO "%s(), reset_gpio is low. Set it to high.\n", __func__);
				gpio_set_value(pdata->reset_gpio, 1);
				printk(KERN_INFO "%s(), pdata->reset_gpio:%d\n", __func__, gpio_get_value(pdata->reset_gpio));
			}
		}
	}
#endif
	atomic_set(&cdata->ioexp_is_suspend, 0);

	register_ioexp_devices(pdata->ioexp_devices, pdata->num_devices);

	printk(KERN_INFO "%s()-, OK\n", __func__);

	ioexp_gpio_init();

	return 0;

err_exit:

err_cdata:
	printk(KERN_ERR "%s()-, FAIL!\n", __func__);
	return ret;

}

#ifdef CONFIG_PM
static int ioexp_i2c_suspend(struct device *dev)
{
	return 0;
}
static int ioexp_i2c_resume(struct device *dev)
{
	return 0;
}
#else
#define ioexp_i2c_suspend	NULL
#define ioexp_i2c_resume	NULL
#endif

static const struct dev_pm_ops ioexp_i2c_pm_ops = {
#ifdef CONFIG_PM
	.suspend = ioexp_i2c_suspend,
	.resume = ioexp_i2c_resume,
#endif
};

static const struct i2c_device_id ioexp_i2c_id[] = {
	{ IOEXPANDER_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ioexp_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id ioexp_i2c_match_table[] = {
	{.compatible = "ti,tca6418" },
	{},
};
#else
#define ioexp_i2c_match_table NULL
#endif

static struct i2c_driver ioexp_i2c_driver = {
	.driver = {
		.name		= IOEXPANDER_I2C_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = ioexp_i2c_match_table,
#ifdef CONFIG_PM
		.pm		= &ioexp_i2c_pm_ops,
#endif
	},
	.probe		= ioexp_i2c_probe,
	.remove		= ioexp_i2c_remove,
	.id_table	= ioexp_i2c_id,
};
module_i2c_driver(ioexp_i2c_driver);

MODULE_AUTHOR("Joner Lin <Joner_Lin@htc.com>");
MODULE_DESCRIPTION("IO Expander driver");
MODULE_LICENSE("GPL");

