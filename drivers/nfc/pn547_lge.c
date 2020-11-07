/*
 *
 * Copyright (C) 2010 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

/*****************************************************************************
 * Include
 *****************************************************************************/
#include <linux/nfc/pn547_lge.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif


#include <linux/async.h>
#include <linux/wakelock.h>

#include <mt-plat/mt_gpio.h>
#define PN547_DRV_NAME "pn547"
//#define I2C_DEVICE_ADDR 0x28
//#define NFC_I2C_BUSNUM  1
//static struct i2c_board_info nfc_board_info __initdata = { I2C_BOARD_INFO(PN547_DRV_NAME, I2C_DEVICE_ADDR) };

#ifdef CONFIG_LGE_NFC_DEBUG_MESSAGE
#define NFC_LOG(func_name, log_message) pr_err("%s - %s\n", func_name, log_message)
#define NFC_LOG1(func_name, log_message, x) pr_err("%s - %s : %d\n", func_name, log_message, x)
#else
#define NFC_LOG(func_name, log_message)
#define NFC_LOG1(func_name, log_message, x)
#endif

#define MAX_BUFFER_SIZE    512
#define NFC_POWER_OFF    false
#define NFC_POWER_ON    true
#define NFC_TIMEOUT_MS 2000

#define NFC_CLIENT_TIMING 400	/* I2C speed */

#define GET_GPIO(x) mt_get_gpio_in(x)
#define SET_GPIO(x,y) mt_set_gpio_out(x,y)
#define MTK_GPIO(x) (x | 0x80000000)

#define READ_IRQ_MODIFY //NFC-6953

#ifdef READ_IRQ_MODIFY
bool do_reading = 0;
static bool cancel_read = false;
#endif

struct pn547_dev g_pn547_dev;
struct wake_lock nfc_wake_lock;
static bool sPowerState = NFC_POWER_OFF;
static bool sIsWakeLocked = false;
static bool sIrqState = false;


/* For DMA */
static char *I2CDMAWriteBuf;	/*= NULL;*//* unnecessary initialise */
static dma_addr_t I2CDMAWriteBuf_pa;
static char *I2CDMAReadBuf;	/*= NULL;*//* unnecessary initialise */
static dma_addr_t I2CDMAReadBuf_pa;	/* = NULL; */

#ifdef CONFIG_GPIO_PCAL6416A
extern int nxp_gpio_set_cfg(uint8_t gpio, uint8_t direction, uint8_t pull, uint8_t drvstr);
extern int nxp_gpio_set_value(uint8_t gpio, uint8_t value);
extern int nxp_gpio_get_value(uint8_t gpio);
bool expander_mode = false;
#endif


static void pn547_disable_irq(struct pn547_dev *pn547_dev)
{
    unsigned long flags;

    spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
    if (pn547_dev->irq_enabled) {
        disable_irq_nosync(pn547_dev->client->irq);
        disable_irq_wake(pn547_dev->client->irq);
        pn547_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static void pn547_enable_irq(struct pn547_dev *pn547_dev)
{
    unsigned long flags;
    spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
    if (!pn547_dev->irq_enabled) {
        enable_irq(pn547_dev->client->irq);
        enable_irq_wake(pn547_dev->client->irq);
        pn547_dev->irq_enabled = true;
    }
    spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static long pn547_dev_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct pn547_dev *pn547_dev = filp->private_data;
    unsigned long flags;

    switch (cmd) {
    case pn547_SET_PWR:
        if (arg == 2) {
            /* power on with firmware download (requires hw reset) */
            dprintk(PN547_DRV_NAME ":%s power on with firmware\n", __func__);
            SET_GPIO(pn547_dev->ven_gpio, 1);

#ifdef CONFIG_GPIO_PCAL6416A
            //NFC_LOG(__func__, "CONFIG_GPIO_PCAL6416A");
            if (expander_mode) {
                nxp_gpio_set_value(pn547_dev->firm_gpio, 1);
            } else {
                SET_GPIO(pn547_dev->firm_gpio, 1);
            }
#else
            SET_GPIO(pn547_dev->firm_gpio, 1);
#endif

            msleep(10);
            SET_GPIO(pn547_dev->ven_gpio, 0);
            msleep(10);
            SET_GPIO(pn547_dev->ven_gpio, 1);
            msleep(10);
        } else if (arg == 1) {
            /* power on */
            NFC_LOG(__func__, "power on");
            if (sPowerState == NFC_POWER_OFF) {

#ifdef CONFIG_GPIO_PCAL6416A
                if (expander_mode) {
                    nxp_gpio_set_value(pn547_dev->firm_gpio, 0);
                } else {
                    SET_GPIO(pn547_dev->firm_gpio, 0);
                }
#else
                SET_GPIO(pn547_dev->firm_gpio, 0);
#endif

                SET_GPIO(pn547_dev->ven_gpio, 1);
                msleep(10);
                spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
                if (sIrqState == false) {
                    irq_set_irq_wake(pn547_dev->client->irq,1);
                    sIrqState = true;
                    NFC_LOG(__func__, "enable IRQ");
                }
                else {
                    pr_err("%s IRQ is already enabled!\n", __func__);
                }
                NFC_LOG(__func__, "NFC_POWER_ON");
                sPowerState = NFC_POWER_ON;
                spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
            }
            else {
                pr_err("%s NFC is alread On!\n", __func__);
            }
        } else  if (arg == 0) {
            /* power off */
            NFC_LOG(__func__, "power off");
            if (sPowerState == NFC_POWER_ON) {

#ifdef CONFIG_GPIO_PCAL6416A
                if (expander_mode) {
                    nxp_gpio_set_value(pn547_dev->firm_gpio, 0);
                } else {
                    SET_GPIO(pn547_dev->firm_gpio, 0);
                }
#else
                SET_GPIO(pn547_dev->firm_gpio, 0);
#endif

                SET_GPIO(pn547_dev->ven_gpio, 0);
                msleep(10);
                spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
                if (sIrqState == true) {
                    irq_set_irq_wake(pn547_dev->client->irq,0);
                    sIrqState = false;
                    NFC_LOG(__func__, "disable IRQ");
                }
                else {
                    pr_err("%s IRQ is already disabled!\n", __func__);
                }
                if (sIsWakeLocked == true) {
                    NFC_LOG(__func__, "Release Wake_Lock");
                    wake_unlock(&nfc_wake_lock);
                    sIsWakeLocked = false;
                }
                NFC_LOG(__func__, "NFC_POWER_OFF");
                sPowerState = NFC_POWER_OFF;
                spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
            }
            else {
                pr_err("%s NFC is already off\n", __func__);
            }
#ifdef READ_IRQ_MODIFY
        } else if (arg == 3) {
            pr_err("%s Read Cancel\n", __func__);
            cancel_read = true;
            do_reading = 1;
            wake_up(&pn547_dev->read_wq);
#endif
        } else {
                pr_err("%s bad arg %ld\n", __func__, arg);
            return -EINVAL;
        }
        break;
/*    case pn547_HW_REVISION:
        {
            return pn547_get_hw_revision();
        }*/
    default:
        pr_err("%s bad ioctl %d\n", __func__, cmd);
        return -EINVAL;
    }

    return 0;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn547_dev  *pn547_dev;
    int ret;

    pn547_dev = filp->private_data;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    if (copy_from_user(I2CDMAWriteBuf, buf, count)) {
        pr_err(PN547_DRV_NAME ":%s : failed to copy from user space\n", __func__);
        return -EFAULT;
    }
    //NFC_LOG1(__func__, "writing bytes", count);
    mutex_lock(&pn547_dev->read_mutex);

    /* Write data */
    pn547_dev->client->addr = (pn547_dev->client->addr & I2C_MASK_FLAG);
    pn547_dev->client->ext_flag |= I2C_DMA_FLAG;
    pn547_dev->client->timing = NFC_CLIENT_TIMING;

    ret = i2c_master_send(pn547_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, count);

    mutex_unlock(&pn547_dev->read_mutex);

    if (ret != count) {
        pr_err("%s : i2c_master_send returned %d (one time is ok)\n", __func__, ret);
        ret = -EIO;
    }
    return ret;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn547_dev *pn547_dev = filp->private_data;
    int ret;
    static bool isFirstPacket = true;
    unsigned long flags;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;
    //NFC_LOG1(__func__, "reading bytes. count", count);

#ifdef READ_IRQ_MODIFY
    if (!GET_GPIO(pn547_dev->irq_gpio))
        do_reading = 0;
#endif

    if (isFirstPacket == false) {
#ifdef READ_IRQ_MODIFY
        ret = wait_event_interruptible_timeout(pn547_dev->read_wq, do_reading, msecs_to_jiffies(NFC_TIMEOUT_MS));
#else
        ret = wait_event_interruptible_timeout(pn547_dev->read_wq, GET_GPIO(pn547_dev->irq_gpio), msecs_to_jiffies(NFC_TIMEOUT_MS));
#endif
        if (ret == 0) {
            NFC_LOG1(__func__, "no more interrupt after 2s.  IRQ", GET_GPIO(pn547_dev->irq_gpio));
            spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
            if (sIsWakeLocked == true) {
                wake_unlock(&nfc_wake_lock);
                sIsWakeLocked = false;
            }
            spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
            //NFC_LOG(__func__, "wake_unlock");
            isFirstPacket = true;
        }
    }

    if (isFirstPacket == true)
    {
#ifdef READ_IRQ_MODIFY
        ret = wait_event_interruptible(pn547_dev->read_wq, do_reading);
#else
        ret = wait_event_interruptible(pn547_dev->read_wq, GET_GPIO(pn547_dev->irq_gpio));
#endif
        if (ret == 0)
            isFirstPacket = false;
    }

    if (ret == -ERESTARTSYS) {
        NFC_LOG(__func__, "pass wait_event_interruptible by signal. Skip");
        return -ERESTARTSYS;
    }
    else {
        //NFC_LOG1(__func__, "pass wait_event_interruptible by condition. IRQ", GET_GPIO(pn547_dev->irq_gpio));
    }

#ifdef READ_IRQ_MODIFY
    if (cancel_read == true) {
        cancel_read = false;
        ret = -1;
        goto fail;
    }
#endif

    /* Read data */
    mutex_lock(&pn547_dev->read_mutex);

	pn547_dev->client->addr = (pn547_dev->client->addr & I2C_MASK_FLAG);
	pn547_dev->client->ext_flag |= I2C_DMA_FLAG;
	pn547_dev->client->timing = NFC_CLIENT_TIMING;

    ret = i2c_master_recv(pn547_dev->client, (unsigned char *)(uintptr_t) I2CDMAReadBuf_pa, count);

    mutex_unlock(&pn547_dev->read_mutex);

    if (count == 0) {
        pr_err("%s: reading 0 bytes! skip! (%d)\n", __func__, ret);
        return ret;
    }
    if (ret < 0) {
        pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        return ret;
    }
    if (ret > count) {
        pr_err("%s: received too many bytes from i2c (%d)\n",
            __func__, ret);
        return -EIO;
    }
    if (copy_to_user(buf, I2CDMAReadBuf, ret)) {
        pr_warning("%s : failed to copy to user space\n", __func__);
        return -EFAULT;
    }
#ifdef READ_IRQ_MODIFY
    fail:
#endif

    //NFC_LOG1(__func__, "i2c_master_recv success.  ret", ret);
    return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
    struct pn547_dev *pn547_dev = &g_pn547_dev;//i2c_get_clientdata(g_pn547_dev.client);
    filp->private_data = pn547_dev;
    pn547_enable_irq(pn547_dev);
    pr_err("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));
    return 0;
}

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
    struct pn547_dev *pn547_dev = dev_id;
    unsigned long flags;
    unsigned int irq_gpio_val;
    irq_gpio_val = GET_GPIO(pn547_dev->irq_gpio);
    if (irq_gpio_val == 0) {
        pr_err("%s: False Interrupt!\n", __func__);
        return IRQ_HANDLED;
    }
    if (sPowerState == NFC_POWER_ON) {
        spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
#ifdef READ_IRQ_MODIFY
        do_reading = 1;
#endif
        /* Wake up waiting readers */
        wake_up(&pn547_dev->read_wq);
        if (sIsWakeLocked == false) {
            wake_lock(&nfc_wake_lock);
            sIsWakeLocked = true;
        }
        else {
            //NFC_LOG(__func__, "already wake locked!");
        }
        spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
        //NFC_LOG1(__func__, "wake_lock.  IRQ", GET_GPIO(pn547_dev->irq_gpio));
    }
    else {
         pr_err("%s, NFC IRQ Triggered during NFC OFF\n", __func__);
    }
    return IRQ_HANDLED;
}


static int pn547_parse_dt(struct pn547_dev *pn547_dev)
{
    struct device_node *node;
#ifdef CONFIG_GPIO_PCAL6416A
    unsigned int iarray[2];
#endif

	node = of_find_compatible_node(NULL, NULL, "nxp,pn547");

	if (node) {
		of_property_read_u32_array(node, "nxp,gpio_ven",
					   &(pn547_dev->ven_gpio), 1);
		of_property_read_u32_array(node, "nxp,gpio_mode",
					   &(pn547_dev->firm_gpio), 1);
		of_property_read_u32_array(node, "nxp,gpio_irq",
					   &(pn547_dev->irq_gpio), 1);
#ifdef CONFIG_GPIO_PCAL6416A
        if (pn547_dev->firm_gpio == 999) {
            pr_info(PN547_DRV_NAME ": FIRM : %d : use expander GPIO\n", pn547_dev->firm_gpio);
            of_property_read_u32_array(node, "nxp,gpio_mode", iarray, 2);
            pn547_dev->firm_gpio = iarray[1];
            expander_mode = true;
            pr_info(PN547_DRV_NAME ": FIRM : use expander GPIO %d\n", pn547_dev->firm_gpio);
        }
#endif
	} else {
		pr_debug("%s : get gpio num err.\n", __func__);
		return 0;
	}
    pr_info(PN547_DRV_NAME ": VEN : %d\n", pn547_dev->ven_gpio);
    pr_info(PN547_DRV_NAME ": FIRM : %d\n", pn547_dev->firm_gpio);
    pr_info(PN547_DRV_NAME ": IRQ: %d\n", pn547_dev->irq_gpio);

	return 0;
}


static const struct file_operations pn547_dev_fops = {
    .owner  = THIS_MODULE,
    .llseek = no_llseek,
    .read   = pn547_dev_read,
    .write  = pn547_dev_write,
    .open   = pn547_dev_open,
    .unlocked_ioctl = pn547_dev_unlocked_ioctl,
    .compat_ioctl = pn547_dev_unlocked_ioctl,
};
static int pn547_dma_free(struct i2c_client *client){
	if (I2CDMAWriteBuf) {
#ifdef CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
				  I2CDMAWriteBuf_pa);
#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
				  I2CDMAWriteBuf_pa);
#endif
		I2CDMAWriteBuf = NULL;
		I2CDMAWriteBuf_pa = 0;
	}

	if (I2CDMAReadBuf) {
#ifdef CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				  I2CDMAReadBuf_pa);
#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				  I2CDMAReadBuf_pa);
#endif
		I2CDMAReadBuf = NULL;
		I2CDMAReadBuf_pa = 0;
	}
    return 0;
}

static int pn547_dma_alloc(struct i2c_client *client){
#ifdef CONFIG_64BIT
	I2CDMAWriteBuf =
	    (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAWriteBuf_pa, GFP_KERNEL);
#else
	I2CDMAWriteBuf =
	    (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAWriteBuf_pa, GFP_KERNEL);
#endif

	if (I2CDMAWriteBuf == NULL) {
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		return 1;
	}
#ifdef CONFIG_64BIT
	I2CDMAReadBuf =
	    (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAReadBuf_pa, GFP_KERNEL);
#else
	I2CDMAReadBuf =
	    (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAReadBuf_pa, GFP_KERNEL);
#endif

	if (I2CDMAReadBuf == NULL) {
		pr_err("%s : failed to allocate dma buffer\n", __func__);
        return 1;
	}
	pr_debug("%s :done\n", __func__);
    return 0;
}

static int pn547_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct pn547_dev *pn547_dev = &g_pn547_dev;
    int ret = 0;
    pr_info(PN547_DRV_NAME ": pn547_probe() start\n");

    ret = pn547_dma_alloc(client);
    if (ret) {
        pn547_dma_free(client);
        pr_err(PN547_DRV_NAME ": pn547_probe() end with error!\n");
        return ret;
    }

    pn547_parse_dt(pn547_dev);
    pn547_dev->client = client;
    pn547_dev->client->irq = gpio_to_irq(pn547_dev->irq_gpio);

    pn547_dev->irq_gpio = MTK_GPIO(pn547_dev->irq_gpio);
    pn547_dev->ven_gpio = MTK_GPIO(pn547_dev->ven_gpio);
    pn547_dev->firm_gpio = MTK_GPIO(pn547_dev->firm_gpio);

    /* irq_gpio setup */
    pr_info("%s : irq_gpio setup", __func__);
    mt_set_gpio_mode(pn547_dev->irq_gpio, GPIO_MODE_00);
    mt_set_gpio_dir(pn547_dev->irq_gpio, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(pn547_dev->irq_gpio, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(pn547_dev->irq_gpio, GPIO_PULL_DOWN);

    /* ven_gpio setup */
    pr_info("%s : ven_gpio setup", __func__);
    mt_set_gpio_mode(pn547_dev->ven_gpio, GPIO_MODE_00);
    mt_set_gpio_dir(pn547_dev->ven_gpio, GPIO_DIR_OUT);

    /* mode_gpio setup */
    pr_info("%s : mode_gpio setup", __func__);

#ifdef CONFIG_GPIO_PCAL6416A
    if (expander_mode) {
        pr_info("%s : nxp_gpio_set_cfg setup", __func__);
        nxp_gpio_set_cfg(pn547_dev->firm_gpio, 1, 0, 0);
    } else {
        mt_set_gpio_mode(pn547_dev->firm_gpio, GPIO_MODE_00);
        mt_set_gpio_dir(pn547_dev->firm_gpio, GPIO_DIR_OUT);
    }
#else
    mt_set_gpio_mode(pn547_dev->firm_gpio, GPIO_MODE_00);
    mt_set_gpio_dir(pn547_dev->firm_gpio, GPIO_DIR_OUT);
#endif

    /* init mutex and queues */
    init_waitqueue_head(&pn547_dev->read_wq);
    mutex_init(&pn547_dev->read_mutex);
    spin_lock_init(&pn547_dev->irq_enabled_lock);
    wake_lock_init(&nfc_wake_lock, WAKE_LOCK_SUSPEND, "NFCWAKE");


    pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
    pn547_dev->pn547_device.name = PN547_DRV_NAME;
    pn547_dev->pn547_device.fops = &pn547_dev_fops;

    pr_info(PN547_DRV_NAME ": misc_register\n");
    ret = misc_register(&pn547_dev->pn547_device);
    if (ret) {
        pr_err("%s : misc_register failed %d\n", __FILE__, ret);
        goto err_exit;
    }

    pr_info("%s : requesting IRQ %s irq : %d",__func__, client->name, client->irq);
    ret = request_irq(client->irq, pn547_dev_irq_handler,
            IRQF_TRIGGER_RISING, client->name, pn547_dev);
    if (ret) {
        pr_err("%s : request_irq failed %d\n", __func__, ret);
        goto err_exit;
    }
    //pn547_disable_irq
    pn547_dev->irq_enabled = true;
    enable_irq_wake(pn547_dev->client->irq);
    pn547_disable_irq(pn547_dev);

    i2c_set_clientdata(client, pn547_dev);

    pr_info(PN547_DRV_NAME ": pn547_probe() end\n");
    return 0;

err_exit:
    mutex_destroy(&pn547_dev->read_mutex);

    gpio_free(pn547_dev->firm_gpio);
    gpio_free(pn547_dev->ven_gpio);
    gpio_free(pn547_dev->irq_gpio);

    pr_info(PN547_DRV_NAME ": misc_register minor %d\n", pn547_dev->pn547_device.minor);
    if (pn547_dev->pn547_device.minor != MISC_DYNAMIC_MINOR && pn547_dev->pn547_device.minor != 0) {
        misc_deregister(&pn547_dev->pn547_device);
    }
    free_irq(client->irq, pn547_dev);
    pr_err(PN547_DRV_NAME ": pn547_probe() end with error!\n");
    return ret;
}

static int pn547_remove(struct i2c_client *client)
{
    struct pn547_dev *pn547_dev;
    pr_info(PN547_DRV_NAME ": remove\n");

    pn547_dev = i2c_get_clientdata(client);
    pn547_dma_free(client);
    free_irq(client->irq, pn547_dev);
    misc_deregister(&pn547_dev->pn547_device);
    mutex_destroy(&pn547_dev->read_mutex);
    gpio_free(pn547_dev->firm_gpio);
    gpio_free(pn547_dev->ven_gpio);
    gpio_free(pn547_dev->irq_gpio);

    return 0;
}

static void pn547_shutdown(struct i2c_client *client)
{
    pr_info(PN547_DRV_NAME ": shutdown\n");
    return;
}

static const struct i2c_device_id pn547_id[] = {
    { PN547_DRV_NAME, 0 },
    { }
};

#ifdef CONFIG_OF
static const struct of_device_id pn547_match_table[] = {
    { .compatible = "nxp,pn547",},
    { },
};
#endif

static struct i2c_driver pn547_driver = {
    .driver = {
        .name = "pn547",
#ifdef CONFIG_OF
        .of_match_table = pn547_match_table,
#endif
    },
    .probe = pn547_probe,
    .remove = pn547_remove,
    .shutdown   = pn547_shutdown,
    .id_table = pn547_id,
};

/*
 * module load/unload record keeping
 */
static void async_dev_init(void *data, async_cookie_t cookie)
{
    int ret = 0;
    pr_info(PN547_DRV_NAME ": Start async init\n");
    pr_info(PN547_DRV_NAME ": i2c_add_driver\n");

    ret = i2c_add_driver(&pn547_driver);
    if (ret < 0) {
        pr_err("[NFC]failed to i2c_add_driver\n");
    }
    pr_info(PN547_DRV_NAME ": Loading PN547 driver Success! \n");
    return;
}

static int __init pn547_dev_init(void)
{
    pr_info("Loading PN547 driver 2.0\n");
//    i2c_register_board_info(1, &nfc_board_info, 1);

    async_schedule(async_dev_init, NULL);

//    i2c_add_driver(&pn547_driver);
    return 0;
}

static void __exit pn547_dev_exit(void)
{
    pr_info("Unloading PN547 driver\n");
    i2c_del_driver(&pn547_driver);
}

MODULE_DEVICE_TABLE(of, pn547_match_table);
MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN547 driver");
MODULE_LICENSE("GPL");

//late_initcall(pn547_dev_init);
module_init(pn547_dev_init);
module_exit(pn547_dev_exit);
