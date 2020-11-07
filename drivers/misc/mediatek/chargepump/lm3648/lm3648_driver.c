/*
* This software program is licensed subject to the GNU General Public License
* (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

* (C) Copyright 2011 Bosch Sensortec GmbH
* All Rights Reserved
*/


/* file lm3648.c
brief This file contains all function implementations for the lm3648 in linux
this source file refer to MT6572 platform
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <mt_gpio.h>
#include <mach/gpio_const.h>
#include <../inc/lm3648.h>
#include <linux/platform_device.h>

#include <linux/ctype.h>
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
#include <soc/mediatek/lge/board_lge.h>
extern int lge_get_maker_id(void);
#endif

#include <linux/leds.h>
#define FLASH_NAME "strobe_sub"


#define LM3648_DEV_NAME "lm3648"

#define CPD_TAG                  "[sub_flash] "
#define CPD_FUN(f)               printk(CPD_TAG"%s\n", __FUNCTION__)
#define CPD_ERR(fmt, args...)    printk(CPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define CPD_LOG(fmt, args...)    printk(CPD_TAG fmt, ##args)

//GPIO
#ifndef GPIO_FRONT_FLASH_EN
#define GPIO_FRONT_FLASH_EN        (GPIO19 | 0x80000000)
#define GPIO_FRONT_FLASH_EN_M_GPIO   GPIO_MODE_00
#endif
// I2C variable
static struct i2c_client *new_client_lm3648 = NULL;
static const struct i2c_device_id lm3648_i2c_id[] = {{LM3648_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_lm3648={ I2C_BOARD_INFO(LM3648_DEV_NAME, 0x63)};

// Flash control
unsigned char lm3648_strobe_ctrl;
unsigned char lm3648_flash_ctrl=0; // flash_en register(0x0A) setting position change.

struct semaphore lm3648_lock; // Add semaphore for lcd and flash i2c communication.

/* generic */
#define LM3648_MAX_RETRY_I2C_XFER (100)
#define LM3648_I2C_WRITE_DELAY_TIME 1

/* i2c read routine for API*/
static char lm3648_i2c_read(struct i2c_client *client, u8 reg_addr,
            u8 *data, u8 len)
{
        #if !defined BMA_USE_BASIC_I2C_FUNC
        s32 dummy;
		client->addr = 0x63;
        if (NULL == client)
            return -1;

        while (0 != len--) {
            #ifdef BMA_SMBUS
            dummy = i2c_smbus_read_byte_data(client, reg_addr);
            if (dummy < 0) {
                CPD_ERR("i2c bus read error\n");
                return -1;
            }
            *data = (u8)(dummy & 0xff);
            #else
            dummy = i2c_master_send(client, (char *)&reg_addr, 1);
            if (dummy < 0)
            {
                CPD_ERR("send dummy is %d\n", dummy);
                return -1;
            }

            dummy = i2c_master_recv(client, (char *)data, 1);
            if (dummy < 0)
            {
                CPD_ERR("recv dummy is %d\n", dummy);
                return -1;
            }
            #endif
            reg_addr++;
            data++;
        }
        return 0;
        #else
        int retry;

        struct i2c_msg msg[] = {
                {
                    .addr = 0x63,
                    .flags = 0,
                    .len = 1,
                    .buf = &reg_addr,
                },

                {
                    .addr = 0x63,
                    .flags = I2C_M_RD,
                    .len = len,
                    .buf = data,
                },
        };

        for (retry = 0; retry < LM3648_MAX_RETRY_I2C_XFER; retry++) {
            if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
                break;
            else
                mdelay(LM3648_I2C_WRITE_DELAY_TIME);
        }

        if (LM3648_MAX_RETRY_I2C_XFER <= retry) {
            CPD_ERR("I2C xfer error\n");
            return -EIO;
        }

        return 0;
        #endif
}

/* i2c write routine for */
static char lm3648_i2c_write(struct i2c_client *client, u8 reg_addr,
            u8 *data, u8 len)
{

        #if !defined BMA_USE_BASIC_I2C_FUNC
        s32 dummy;

		client->addr = 0x63;
        if (NULL == client)
            return -1;

        while (0 != len--) {
            dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);

            reg_addr++;
            data++;
            if (dummy < 0) {
                return -1;
            }
        }

        #else
        u8 buffer[2];
        int retry;
        struct i2c_msg msg[] = {
                {
                    .addr = 0x63,
                    .flags = 0,
                    .len = 2,
                    .buf = buffer,
                },
        };

        while (0 != len--) {
            buffer[0] = reg_addr;
            buffer[1] = *data;
            for (retry = 0; retry < LM3648_MAX_RETRY_I2C_XFER; retry++) {
                if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0) {
                    break;
                }
                else {
                    mdelay(LM3648_I2C_WRITE_DELAY_TIME);
                }
            }
            if (LM3648_MAX_RETRY_I2C_XFER <= retry) {
                return -EIO;
            }
            reg_addr++;
            data++;
        }
        #endif
        //CPD_LOG("lm3648_i2c_write \n");
        return 0;
}

static int lm3648_smbus_read_byte(struct i2c_client *client,
            unsigned char reg_addr, unsigned char *data)
{
        return lm3648_i2c_read(client,reg_addr,data,1);
}

static int lm3648_smbus_write_byte(struct i2c_client *client,
            unsigned char reg_addr, unsigned char *data)
{
        int ret_val = 0;
        int i = 0;
		

        ret_val = lm3648_i2c_write(client,reg_addr,data,1);

        for ( i = 0; i < 5; i++)
        {
            if (ret_val != 0)
                lm3648_i2c_write(client,reg_addr,data,1);
#if 0
            else
                return ret_val;
#endif
            else
            {
                pr_warn("[lm3648] : lm3648_smbus_write_byte fail: %d\n",ret_val);
                CPD_LOG("[lm3648] : lm3648_smbus_write_byte fail: %d\n",ret_val);
                return ret_val;
            }
        }
        return ret_val;
}

#if 0
static int lm3648_smbus_read_byte_block(struct i2c_client *client,
unsigned char reg_addr, unsigned char *data, unsigned char len)
{
        return lm3648_i2c_read(client,reg_addr,data,len);
}
#endif


static int lm3648_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
       CPD_FUN();

       new_client_lm3648 = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);


       memset(new_client_lm3648, 0, sizeof(struct i2c_client));

       new_client_lm3648 = client;

       return 0;
}

#if 0
static int lm3648_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
        new_client_lm3648 = client;

        CPD_FUN();
        if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
            CPD_LOG("i2c_check_functionality error\n");
            return -1;
        }

        sema_init(&lm3648_lock, 1); // Add semaphore for lcd and flash i2c communication.

        if (client == NULL) 
            printk("%s client is NULL\n", __func__);
        else
        {
            //CPD_LOG("%s %x %x %x\n", __func__, client->adapter, client->addr, client->flags);
            CPD_LOG("%s is OK\n", __func__);
        }
        return 0;
}
#endif

static int lm3648_remove(struct i2c_client *client)
{
        new_client_lm3648 = NULL;
        return 0;
}

#if 0
static int lm3648_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{ 
    return 0;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id lm3648_of_match[] = {
    {.compatible = "mediatek,strobe_main",},
    {},
};

MODULE_DEVICE_TABLE(of, lm3648_of_match);
#endif

static struct i2c_driver lm3648_i2c_driver = {
        .driver = {
        .owner  = THIS_MODULE,
        .name   = LM3648_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = lm3648_of_match,
#endif
        },
        .probe      = lm3648_driver_probe,
        .remove     = lm3648_remove,
        //  .detect     = lm3648_detect,
        .id_table   = lm3648_i2c_id,
        //  .address_data = &lm3648250_i2c_addr_data,
};

static int lm3648_pd_probe(struct platform_device *pdev) 
{
        if(i2c_add_driver(&lm3648_i2c_driver))
        {
            CPD_ERR("add driver error\n");
            return -1;
        }
        else
        {
            CPD_LOG("i2c_add_driver OK\n");       
        }
        return 0;
}

static int lm3648_pd_remove(struct platform_device *pdev)
{
        CPD_FUN();
        i2c_del_driver(&lm3648_i2c_driver);
        return 0;
}


static struct platform_driver lm3648_backlight_driver = {
        .probe      = lm3648_pd_probe,
        .remove     = lm3648_pd_remove,
        .driver     = {
                .name  = FLASH_NAME,
                .owner = THIS_MODULE,
        }
};      

static int __init lm3648_init(void)
{
        CPD_FUN();

		sema_init(&lm3648_lock, 1);
		
		mt_set_gpio_mode(GPIO_FRONT_FLASH_EN, GPIO_FRONT_FLASH_EN_M_GPIO);
		mt_set_gpio_pull_enable(GPIO_FRONT_FLASH_EN, GPIO_PULL_ENABLE);
		mt_set_gpio_dir(GPIO_FRONT_FLASH_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_FRONT_FLASH_EN, GPIO_OUT_ONE);

		i2c_register_board_info(2, &i2c_lm3648, 1);
		CPD_LOG("lm3648_i2c_master_addr = 3\n");

		if (i2c_add_driver(&lm3648_i2c_driver) != 0)
		{
			CPD_ERR("Failed to register rt4832 driver");
}

		
		
        return 0;
}

static void __exit lm3648_exit(void)
{
        platform_driver_unregister(&lm3648_backlight_driver);
}

void lm3648_flash_strobe_prepare(char OnOff,char ActiveHigh)
{
        int err = 0;

        err = down_interruptible(&lm3648_lock); // Add semaphore for lcd and flash i2c communication.
        err = lm3648_smbus_read_byte(new_client_lm3648, 0x01, &lm3648_strobe_ctrl);
        lm3648_strobe_ctrl &= 0x0B;
        lm3648_flash_ctrl = OnOff;
        err = lm3648_smbus_write_byte(new_client_lm3648, 0x01, &lm3648_strobe_ctrl);

        up(&lm3648_lock); // Add semaphore for lcd and flash i2c communication.
}

//strobe enable
void lm3648_flash_strobe_en(void)
{
        int err = 0;
        int flash_OnOff=0;
        err = down_interruptible(&lm3648_lock); // Add semaphore for lcd and flash i2c communication.
        err = lm3648_smbus_read_byte(new_client_lm3648, 0x01, (unsigned char *)&flash_OnOff);

        if(lm3648_flash_ctrl == 1) { //strobe
	        flash_OnOff |= 0x2F;
              CPD_ERR("[LGE] Strobe mode");
        }
        else if(lm3648_flash_ctrl == 2){  //torch
	        flash_OnOff |= 0x0B;
              CPD_ERR("[LGE] Torch mode");
        }
        else
	        flash_OnOff &= 0x00;
        err = lm3648_smbus_write_byte(new_client_lm3648, 0x01, (unsigned char *)&flash_OnOff);
        up(&lm3648_lock); // Add semaphore for lcd and flash i2c communication.
}

//LGE_CHANGE_S, set flash current feature, Himchan.Lee
void lm3648_flash_torch_level(int level)
{
    unsigned char torch_level;
    int err = 0;
    err = down_interruptible(&lm3648_lock);

    if (level == 1)
        torch_level = 0x83;
    else if (level == 2)
        torch_level = 0x88;
    else if (level == 3)
        torch_level = 0x8C;
    else if (level == 4)
        torch_level = 0x91;
    else if (level == 5)
        torch_level = 0x95;
    else if (level == 6)
        torch_level = 0x9A;
    else if (level == 7)
        torch_level = 0x9E;
    else if (level == 8)
        torch_level = 0xA3;
    else if (level == 9)
        torch_level = 0xA7;
    else if (level == 10)
        torch_level = 0xAB;
    else if (level == 11)
        torch_level = 0xB0;
    else if (level == 12)
        torch_level = 0xB4;
    else if (level == 13)
        torch_level = 0xB9;
    else if (level == 14)
        torch_level = 0xBD;
    else if (level == 15)
        torch_level = 0xC2;
    else
        torch_level = 0xBF; //178mA

    err = lm3648_smbus_write_byte(new_client_lm3648, 0x05, &torch_level);
    up(&lm3648_lock);
}
//LGE_CHANGE_E, set flash current feature, Himchan.Lee

//strobe level
void lm3648_flash_strobe_level(char level)
{
        int err = 0;
        unsigned char data1=0;
//        unsigned char data2=0;
    unsigned char torch_level;
	unsigned char strobe_timeout = 0x1F;

    err = down_interruptible(&lm3648_lock); // Add semaphore for lcd and flash i2c communication.

	torch_level = 0xBF; //178mA

    err = lm3648_smbus_read_byte(new_client_lm3648, 0x05, &data1);

	strobe_timeout = 0x1F;	/* Flash Timing Tuning */
	if (level < 0)
		data1 = 0x10;//400
	else if (level == 1)
		data1 = 0x10;//400
	else if (level == 2)
		data1 = 0x19;//600
	else if (level == 3)
		data1 =  0x26;//900
	else if (level == 4)
		data1 =  0x2E;//1100
	else
		data1 = 0x2E;//1100
        err |= lm3648_smbus_write_byte(new_client_lm3648, 0x03, &data1);
        err = lm3648_smbus_write_byte(new_client_lm3648, 0x05, &torch_level);
       
        err |= lm3648_smbus_write_byte(new_client_lm3648, 0x08, &strobe_timeout);
        up(&lm3648_lock); // Add semaphore for lcd and flash i2c communication.
}
EXPORT_SYMBOL(lm3648_flash_strobe_en);
EXPORT_SYMBOL(lm3648_flash_strobe_prepare);
EXPORT_SYMBOL(lm3648_flash_strobe_level);
MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("lm3648 driver");
MODULE_LICENSE("GPL");

module_init(lm3648_init);
module_exit(lm3648_exit);
