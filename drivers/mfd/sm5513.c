

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/sm5513-private.h>
#include <linux/regulator/machine.h>
#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#endif /* CONFIG_OF */

#define I2C_ADDR_SM5513	    (0x42 >> 1)
#define I2C_WR_RETRY_CNT    2

/* In sm5513, switching mux is available by gpio or register	*
   If you want to use gpio(nOEPIN, SELPIN), set the feature		*/
#define USE_SM5513_SYS_NODE

#ifdef USE_SM5513_SYS_NODE
static struct kobject *debug_kobj;
#endif

static struct mfd_cell sm5513_devs[] = {
#if defined(CONFIG_USB_C_SWITCH_SM5513)
	{ .name = "sm5513-ccic", },
#endif
};

static struct sm5513_dev *g_sm5513 = NULL;
struct pinctrl *sm5513_pinctrl;
struct pinctrl_state *sm5513_int_cfg;

int sm5513_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct sm5513_dev *sm5513 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&sm5513->i2c_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&sm5513->i2c_lock);
	if (ret < 0) {
		sm5513_err_msg("%s:%s reg(0x%x), ret(%d)\n", MFD_DEV_NAME, __func__, reg, ret);
		return ret;
	}
	*dest = (ret & 0xff);

    return 0;
}
EXPORT_SYMBOL_GPL(sm5513_read_reg);

int sm5513_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct sm5513_dev *sm5513 = i2c_get_clientdata(i2c);
    int retry = I2C_WR_RETRY_CNT;
	int ret;

RETRY_P:
	mutex_lock(&sm5513->i2c_lock);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&sm5513->i2c_lock);
	if (ret < 0) {
		sm5513_err_msg("%s:%s reg(0x%x), ret(%d)\n", MFD_DEV_NAME, __func__, reg, ret);
        if (retry-- > 0) {
            goto RETRY_P;
        }
    }

	return ret;
}
EXPORT_SYMBOL_GPL(sm5513_write_reg);

int sm5513_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct sm5513_dev *sm5513 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&sm5513->i2c_lock);
	ret = i2c_smbus_read_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&sm5513->i2c_lock);
	if (ret < 0)
        sm5513_err_msg("%s:%s reg(0x%x), cnt(%d), ret(%d)\n", MFD_DEV_NAME, __func__, reg, count, ret);
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(sm5513_bulk_read);

int sm5513_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct sm5513_dev *sm5513 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&sm5513->i2c_lock);
	ret = i2c_smbus_write_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&sm5513->i2c_lock);
	if (ret < 0)
        sm5513_err_msg("%s:%s reg(0x%x), cnt(%d), ret(%d)\n", MFD_DEV_NAME, __func__, reg, count, ret);
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(sm5513_bulk_write);

int sm5513_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct sm5513_dev *sm5513 = i2c_get_clientdata(i2c);
    int retry = I2C_WR_RETRY_CNT;
	int ret;

RETRY_P:
	mutex_lock(&sm5513->i2c_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
        if (ret < 0) {
            sm5513_err_msg("%s:%s - fail to write byte: reg(0x%x), ret(%d)\n", MFD_DEV_NAME, __func__, reg, ret);
        }
	} else {
        sm5513_err_msg("%s:%s - fail to read byte: reg(0x%x), ret(%d)\n", MFD_DEV_NAME, __func__, reg, ret);
    }
	mutex_unlock(&sm5513->i2c_lock);

    if (ret < 0 && retry-- > 0) {
        goto RETRY_P;
    }

    return ret;
}
EXPORT_SYMBOL_GPL(sm5513_update_reg);

/**
 * Suppot MUX Switch control interface functions
 */
 int sm5513_set_mux_channel(int channel)
{
    int ret;

//	sm5513_dbg_msg("%s: %s - channel %d\n", MFD_DEV_NAME, __func__, channel);

	if (channel == MUX_CH_1)
		channel	= DPDM_CON_HOST1;
	else if (channel == MUX_CH_2)
		channel	= DPDM_CON_HOST2;
	else
		channel	= DPDM_CON_OPEN;

    ret = sm5513_update_reg(g_sm5513->i2c, SM5513_REG_SWCNTL, (channel << 3), SM5513_SWCNTL_DPDM_CON_SW);

    return ret;
}

u8 sm5513_get_mux_channel(void)
{
	u8 reg = 0;
	u8 channel;

	sm5513_read_reg(g_sm5513->i2c, SM5513_REG_SWCNTL , &reg);
	reg	= (reg & SM5513_SWCNTL_DPDM_CON_SW) >> 3;

	if (reg	== DPDM_CON_HOST1)
		channel	= MUX_CH_1;
	else if (reg == DPDM_CON_HOST2)
		channel	= MUX_CH_2;
	else if (reg == DPDM_CON_OPEN)
		channel	= MUX_CH_0;
	else
		channel	= MUX_CH_FAULT;

	/* sm5513_dbg_msg("%s: %s - channel %d\n", MFD_DEV_NAME, __func__, channel); */

    return channel;
}

#if defined(CONFIG_OF)
static int of_sm5513_dt(struct device *dev, struct sm5513_platform_data *pdata)
{
	struct device_node *np_sm5513 = dev->of_node;

	if(!np_sm5513)
		return -EINVAL;

    of_property_read_u32(np_sm5513, "sm5513,irq-gpio", &pdata->irq_gpio);
    pdata->irq = irq_of_parse_and_map(np_sm5513, 0);
    pdata->wakeup = of_property_read_bool(np_sm5513, "sm5513,wakeup");

    pdata->irq = gpio_to_irq(pdata->irq_gpio);

	sm5513_dbg_msg("%s:%s: irq=%d, irq-gpio=%d\n",MFD_DEV_NAME,  __func__, pdata->irq, pdata->irq_gpio);
	
	return 0;
}

#else

static int of_sm5513_dt(struct device *dev, struct sm5513_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

static int sm5513_gpio_init(struct sm5513_dev *sm5513)
{
	sm5513_pinctrl = NULL;

	sm5513_pinctrl = devm_pinctrl_get(sm5513->dev);
	if (IS_ERR(sm5513_pinctrl)) {
		sm5513_err_msg("%s:%s::error, fail to get pinctrl\n",MFD_DEV_NAME, __func__);
		return -EINVAL;
	}

	sm5513_int_cfg = pinctrl_lookup_state(sm5513_pinctrl, "sm5513_int_cfg");
	if (IS_ERR(sm5513_int_cfg)) {
		sm5513_err_msg("%s:%s::error, fail to get sm5513_int_cfg\n",MFD_DEV_NAME, __func__);
		return -EINVAL;
	}
	pinctrl_select_state(sm5513_pinctrl, sm5513_int_cfg);

    return 0;
}

#ifdef USE_SM5513_SYS_NODE
static ssize_t show_mux_channel(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 reg_data	= 0;

	if (sm5513_read_reg(g_sm5513->i2c, SM5513_REG_SWCNTL, &reg_data) < 0) {
		sm5513_err_msg("%s:%s read failed\n", MFD_DEV_NAME, __func__);
		return sprintf(buf, "Mux Channel Could not read\n");
	} else {
		if (reg_data & (SM5513_SWCNTL_ENnOEPIN|SM5513_SWCNTL_ENnOEPIN)) {
			return sprintf(buf, "Mux Channel is not set for i2c\n");
		}
		reg_data	= (reg_data & SM5513_SWCNTL_DPDM_CON_SW) >> 3;

		if (reg_data == DPDM_CON_HOST1)
			return sprintf(buf, "Mux Channel Host1\n");
		else if (reg_data == DPDM_CON_HOST2)
			return sprintf(buf, "Mux Channel Host2\n");
		else if (reg_data == DPDM_CON_OPEN)
			return sprintf(buf, "Mux Channel Open\n");
		else
			return sprintf(buf, "Mux Channel Reserved\n");
	}
}

static ssize_t store_mux_channel(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int channel	= 0;
	if(buf != NULL && size != 0)
		channel = (int)(buf[0]) - 0x30;

	sm5513_set_mux_channel(channel);

	return size;
}
static ssize_t show_register_rw(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int i;

	u8 reg = 0;

	for (i = SM5513_REG_INTMASK1; i <= SM5513_REG_DEVICE_ID; i++) {
		sm5513_read_reg(g_sm5513->i2c, i, &reg);
		pr_err("[USB-CC] Register 0x%x = 0x%x\n", i, reg);
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[USB-CC] Register 0x%x = 0x%x\n", i, reg);
	}

	return ret;
}


static ssize_t store_register_rw(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int address;
	unsigned int value;

	if (sscanf(buf, "%x %x", &address, &value) < 2)
		return size;

	pr_err("[USB-CC] Register write 0x%x = 0x%x\n", address, value);
	sm5513_write_reg(g_sm5513->i2c, (u8)address, (u8)value);
	
	return size;
}

static ssize_t show_enable_cc(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 reg = 0;
	int value_cc_state = 0;
	int value_op_mode = 0;

	sm5513_read_reg(g_sm5513->i2c, SM5513_REG_CC_CNTL4 , &reg);
	value_cc_state = reg & SM5513_CC_CNTL4_CC_STATE;
	sm5513_read_reg(g_sm5513->i2c, SM5513_REG_CC_CNTL1 , &reg);
	value_op_mode = reg & SM5513_CC_CNTL1_OP_MODE;

	return sprintf(buf, "Enable CC state: %d op mode: %d\n", value_cc_state, value_op_mode);
}

static ssize_t store_enable_cc(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	if (sscanf(buf, "%d", &value) < 1)
		return size;

	pr_err("[USB-CC] enable_cc setting to %d\n", value);
	sm5513_enable_cc_detection((bool)value);

	return size;
}

static ssize_t store_reset_sw(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	int rev_id = 0;

	if (sscanf(buf, "%d", &value) < 1)
		return size;


	sm5513_write_reg(g_sm5513->i2c, SM5513_REG_CNTL, SM5513_CNTL_SW_RESET);
	pr_err("[USB-CC] enable SW_RESET\n");

/* configure system initalize setting */

	rev_id = (g_sm5513->device_id >> 4) & 0xf;

	if (rev_id > 0) {
		sm5513_write_reg(g_sm5513->i2c, SM5513_REG_INTMASK1, 0x3C);
	} else {
		sm5513_write_reg(g_sm5513->i2c, SM5513_REG_INTMASK1, 0xFC);
	}
	sm5513_write_reg(g_sm5513->i2c, SM5513_REG_INTMASK2, 0x80);

	sm5513_write_reg(g_sm5513->i2c, SM5513_REG_CC_CNTL2, 0x38);

	/* restore current optimize */
	sm5513_update_reg(g_sm5513->i2c, 0x21, (0x0 << 6), (0x1 << 6));
	sm5513_update_reg(g_sm5513->i2c, SM5513_REG_CC_CNTL1, CC_OP_MODE_TRY_SNK, 0xf);

	return size;
}

static DEVICE_ATTR(mux_channel, S_IWUSR | S_IRUGO, show_mux_channel, store_mux_channel);
static DEVICE_ATTR(register_rw, S_IWUSR | S_IRUGO, show_register_rw, store_register_rw);
static DEVICE_ATTR(enable_cc, S_IWUSR | S_IRUGO, show_enable_cc, store_enable_cc);
static DEVICE_ATTR(reset_sw, S_IWUSR | S_IRUGO, NULL, store_reset_sw);


static struct attribute *sm5513_attrs[]	= {
	&dev_attr_mux_channel.attr,
	&dev_attr_register_rw.attr,
	&dev_attr_enable_cc.attr,
	&dev_attr_reset_sw.attr,
	NULL
};

static const struct attribute_group sm5513_attr_group = {
	.name	= "sm5513",
	.attrs	= sm5513_attrs,
};
#endif

static int sm5513_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *dev_id)
{
	struct sm5513_dev *sm5513;
	struct sm5513_platform_data *pdata = i2c->dev.platform_data;

	u8 reg_data;
	int ret = 0;

	sm5513_dbg_msg("%s:%s\n", MFD_DEV_NAME, __func__);

	sm5513 = kzalloc(sizeof(struct sm5513_dev), GFP_KERNEL);
	if (!sm5513) {
		sm5513_err_msg("%s: fail to alloc mem for sm5513\n", MFD_DEV_NAME);
		return -ENOMEM;
	}

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(&i2c->dev, sizeof(struct sm5513_platform_data), GFP_KERNEL);
		if (!pdata) {
			sm5513_err_msg("%s: fail to allocate memory \n", MFD_DEV_NAME);
			ret = -ENOMEM;
			goto err;
		}

		ret = of_sm5513_dt(&i2c->dev, pdata);
		if (ret < 0){
			sm5513_err_msg("%s: fail to get device of_node \n", MFD_DEV_NAME);
			goto err;
		}

		i2c->dev.platform_data = pdata;
	}

	sm5513->dev = &i2c->dev;
	sm5513->i2c = i2c;

	if (pdata) {
        sm5513_gpio_init(sm5513);
		sm5513->pdata = pdata;
	} else {
		ret = -EINVAL;
		goto err;
	}

	mutex_init(&sm5513->i2c_lock);
	i2c_set_clientdata(i2c, sm5513);

    /* Check vender ID */
	if (sm5513_read_reg(i2c, SM5513_REG_DEVICE_ID, &reg_data) < 0) {
		sm5513_err_msg("%s:%s device not found on this channel (this is not an error)\n", MFD_DEV_NAME, __func__);
		ret = -ENODEV;
		goto err_w_lock;
	}
    sm5513->device_id = reg_data;
	sm5513_err_msg("%s:%s device_id is 0x%x\n", MFD_DEV_NAME, __func__, sm5513->device_id);

    sm5513_write_reg(i2c, SM5513_REG_BOOSTCNTL, 0x1A);  /* Prevent BOOSTCNTL default value. */

	ret = mfd_add_devices(sm5513->dev, -1, sm5513_devs, ARRAY_SIZE(sm5513_devs), NULL, 0, NULL);
	if (ret < 0) {
		goto err_mfd;
    }

	device_init_wakeup(sm5513->dev, pdata->wakeup);

#ifdef USE_SM5513_SYS_NODE
	debug_kobj	= kobject_create_and_add("sm5513_debug", NULL);
	if (!debug_kobj)
		return -ENOMEM;
	ret	=	sysfs_create_group(debug_kobj, &sm5513_attr_group);
	if (ret) {
		kobject_put(debug_kobj);
		return ret;
	}
#endif

    g_sm5513 = sm5513;

    sm5513_err_msg("%s: probe done (device_id=0x%x)\n", MFD_DEV_NAME, reg_data);

	return ret;

err_mfd:
	mfd_remove_devices(sm5513->dev);
err_w_lock:
	mutex_destroy(&sm5513->i2c_lock);
err:
	kfree(sm5513);

    return ret;
}

static int sm5513_i2c_remove(struct i2c_client *i2c)
{
	struct sm5513_dev *sm5513 = i2c_get_clientdata(i2c);

	mfd_remove_devices(sm5513->dev);

    mutex_destroy(&sm5513->i2c_lock);

    kfree(sm5513);

	return 0;
}

static const struct i2c_device_id sm5513_i2c_id[] = {
	{ MFD_DEV_NAME, TYPE_SM5513 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5513_i2c_id);

#if defined(CONFIG_OF)
static struct of_device_id sm5513_i2c_dt_ids[] = {
	{ .compatible = "siliconmitus,sm5513" },
	{ },
};
MODULE_DEVICE_TABLE(of, sm5513_i2c_dt_ids);
#endif /* CONFIG_OF */

#if defined(CONFIG_PM)
static int sm5513_suspend(struct device *dev)
{
	//sm5513_dbg_msg("%s:%s\n", MFD_DEV_NAME, __func__);

	return 0;
}

static int sm5513_resume(struct device *dev)
{
	//sm5513_dbg_msg("%s:%s\n", MFD_DEV_NAME, __func__);

	return 0;
}
#else
#define sm5513_suspend	    NULL
#define sm5513_resume		NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops sm5513_pm = {
	.suspend    = sm5513_suspend,
	.resume     = sm5513_resume,
};

static struct i2c_driver sm5513_i2c_driver = {
	.driver		= {
		.name	= MFD_DEV_NAME,
		.owner	= THIS_MODULE,
#if defined(CONFIG_PM)
		.pm	    = &sm5513_pm,
#endif /* CONFIG_PM */
#if defined(CONFIG_OF)
		.of_match_table	= sm5513_i2c_dt_ids,
#endif /* CONFIG_OF */
	},
	.probe		= sm5513_i2c_probe,
	.remove		= sm5513_i2c_remove,
	.id_table	= sm5513_i2c_id,
};

static int __init sm5513_i2c_init(void)
{
	sm5513_dbg_msg("%s:%s\n", MFD_DEV_NAME, __func__);
	return i2c_add_driver(&sm5513_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(sm5513_i2c_init);


