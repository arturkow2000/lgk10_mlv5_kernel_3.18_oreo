/*
* This software program is licensed subject to the GNU General Public License
* (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

* (C) Copyright 2011 Bosch Sensortec GmbH
* All Rights Reserved
*/


/* file dw8767.c
brief This file contains all function implementations for the dw8767 in linux
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
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/leds.h>

#include <mt_gpio.h>
#include <mach/gpio_const.h>

#define DW8767_MIN_VALUE_SETTINGS 10
#define DW8767_MAX_VALUE_SETTINGS 255

#define MIN_MAX_SCALE(x) (((x)<DW8767_MIN_VALUE_SETTINGS) ? DW8767_MIN_VALUE_SETTINGS : (((x)>DW8767_MAX_VALUE_SETTINGS) ? DW8767_MAX_VALUE_SETTINGS:(x)))

#define BACKLIHGT_NAME          "charge-pump"
#define DW8767_DEV_NAME         "dw8767"
#define CPD_TAG                 "[chargepump] "

#define CPD_FUN(f)              printk(CPD_TAG"%s\n", __FUNCTION__)
#define CPD_ERR(fmt, args...)   printk(CPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define CPD_LOG(fmt, args...)   printk(CPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

#define GPIO_LCD_BL_EN          (GPIO100 | 0x80000000)
#define GPIO_LCD_BL_EN_MODE     GPIO_MODE_00

#define LED_MODE_CUST_LCM       4
#define LED_MODE_CUST_BLS_PWM   5

#define ENABLE  1
#define DISABLE 0

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id dw8767_i2c_id[] = {{DW8767_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_dw8767={ I2C_BOARD_INFO(DW8767_DEV_NAME, 0x11)};

int old_bl_level;
static int led_mode = LED_MODE_CUST_LCM;

// Flash control
unsigned char strobe_ctrl;
unsigned char flash_ctrl=0; // flash_en register(0x0A) setting position change.
unsigned char mapping_level;

#if 0
unsigned int bright_arr[] = {
    7,7,7,7,7,7,7,7,7,8,7,7,8,9,9,10,11,12,12,13,                                                   // 19
    14,14,15,16,17,17,18,19,19,20,21,22,22,23,24,25,25,26,27,27,                                    // 39
    28,29,30,30,31,32,32,33,34,35,35,36,37,37,38,39,40,40,41,42,                                    // 59
    43,45,47,50,52,54,57,59,62,64,66,69,71,74,76,78,81,83,86,88,                                    // 79
    90,93,95,98,100,102,105,107,110,112,114,117,119,122,124,126,129,131,134,136,                    // 99
    138,141,143,146,148,150,153,155,158,162,166,170,174,178,182,187,191,195,199,203,                // 119
    207,212,216,220,224,228,232,237,241,245,249,253,257,262,266,270,274,278,282,287,                // 139
    291,295,299,303,307,312,316,320,324,328,332,337,341,345,349,353,357,362,368,375,                // 159
    382,389,396,403,410,417,424,431,438,445,452,459,466,473,479,486,493,500,507,514,                // 179
    521,528,535,542,549,556,563,570,577,584,590,597,604,611,618,625,632,639,646,653,                // 199
    660,667,674,681,688,695,702,712,722,732,742,752,762,773,783,793,803,813,823,833,                // 219
    844,854,864,874,884,894,904,915,925,935,945,955,965,975,986,996,1006,1016,1026,1036,            // 239
    1046,1057,1067,1077,1087,1097,1107,1117,1128,1138,1148,1158,1168,1178,1188,1199                 // 255
};
#endif

unsigned int bright_arr[] = {
    7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 10, 11,
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 22, 23, 24, 25, 26,
    27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 38, 39, 40, 41,
    42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 57, 60, 64,
    67, 70, 74, 77, 81, 84, 87, 91, 94, 98, 101, 104, 108, 111, 115, 118,
    121, 125, 128, 132, 135, 138, 142, 145, 149, 152, 155, 159, 162, 166, 169, 172,
    176, 179, 183, 186, 189, 193, 196, 200, 203, 206, 210, 213, 217, 223, 229, 235,
    241, 247, 253, 259, 265, 271, 278, 284, 290, 296, 302, 308, 314, 320, 326, 332,
    339, 345, 351, 357, 363, 369, 375, 381, 387, 393, 400, 406, 412, 418, 424, 430,
    436, 442, 448, 454, 461, 467, 473, 479, 485, 491, 497, 503, 509, 516, 525, 534,
    543, 552, 561, 570, 579, 588, 597, 607, 616, 625, 634, 643, 652, 661, 670, 679,
    688, 698, 707, 716, 725, 734, 743, 752, 761, 770, 779, 789, 798, 807, 816, 825,
    834, 843, 852, 861, 870, 880, 889, 898, 907, 916, 925, 934, 943, 952, 962, 974,
    986, 998, 1010, 1022, 1034, 1046, 1058, 1070, 1082, 1094, 1106, 1118, 1130, 1142, 1154, 1166,
    1178, 1190, 1202, 1214, 1226, 1238, 1250, 1262, 1274, 1286, 1298, 1310, 1322, 1334, 1346, 1358,
    1370, 1382, 1394, 1406, 1418, 1430, 1442, 1454, 1466, 1478, 1490, 1502, 1514, 1526, 1538, 1550
};


#ifdef CONFIG_HAS_EARLYSUSPEND
static unsigned char current_brightness = 0;
#endif
static unsigned char is_suspend = DISABLE;

// Add semaphore for lcd and flash i2c communication.
struct semaphore dw8767_lock;

// generic
#define DW8767_MAX_RETRY_I2C_XFER (100)
#define DW8767_I2C_WRITE_DELAY_TIME 1

static char dw8767_i2c_read(struct i2c_client *client, u8 reg_addr, u8 *data, u8 len)
{
#if !defined BMA_USE_BASIC_I2C_FUNC
    s32 dummy;
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
#else
    int retry;

    struct i2c_msg msg[] = {
            {
                .addr = client->addr,
                .flags = 0,
                .len = 1,
                .buf = &reg_addr,
            },

            {
                .addr = client->addr,
                .flags = I2C_M_RD,
                .len = len,
                .buf = data,
            },
    };

    for (retry = 0; retry < DW8767_MAX_RETRY_I2C_XFER; retry++) {
        if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
            break;
        else
            mdelay(DW8767_I2C_WRITE_DELAY_TIME);
    }

    if (DW8767_MAX_RETRY_I2C_XFER <= retry) {
        CPD_ERR("I2C xfer error\n");
        return -EIO;
    }
#endif

    return 0;
}

static char dw8767_i2c_write(struct i2c_client *client, u8 reg_addr, u8 *data, u8 len)
{
#if !defined BMA_USE_BASIC_I2C_FUNC
    s32 dummy;

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
                .addr = client->addr,
                .flags = 0,
                .len = 2,
                .buf = buffer,
            },
    };

    while (0 != len--) {
        buffer[0] = reg_addr;
        buffer[1] = *data;
        for (retry = 0; retry < DW8767_MAX_RETRY_I2C_XFER; retry++) {
            if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0) {
                break;
            }
            else {
                mdelay(DW8767_I2C_WRITE_DELAY_TIME);
            }
        }
        if (DW8767_MAX_RETRY_I2C_XFER <= retry) {
            return -EIO;
        }
        reg_addr++;
        data++;
    }
#endif

    return 0;
}

static int dw8767_smbus_read_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
{
    return dw8767_i2c_read(client,reg_addr,data,1);
}

static int dw8767_smbus_write_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
{
    int ret_val = 0;
    int i = 0;

    ret_val = dw8767_i2c_write(client,reg_addr,data,1);

    for ( i = 0; i < 5; i++)
    {
        if (ret_val != 0)
            dw8767_i2c_write(client,reg_addr,data,1);
#if 0
        else
            return ret_val;
#endif
        else
        {
            //pr_warn("1","[dw8767] : dw8767_smbus_write_byte fail: %d\n",ret_val);
            CPD_LOG("[dw8767] : dw8767_smbus_write_byte fail: %d\n",ret_val);
            return ret_val;
        }
    }
    return ret_val;
}

int chargepump_set_backlight_level(unsigned int level)
{
    unsigned int level_data = 0;
    unsigned int results = 0;
    unsigned char reg_data = 0x00;
    unsigned char lsb_data = 0x00;// 3bit
    unsigned char msb_data = 0x00;// 8bit
    
    old_bl_level = level;

    CPD_LOG("%s, level : %d\n",__func__,level);

    if (level) {
        level = MIN_MAX_SCALE(level);
        level_data = bright_arr[level];
        mapping_level = level_data;

        CPD_LOG("%s, level_data : %d\n",__func__,level_data);

        if (is_suspend == ENABLE){
            CPD_LOG( "backlight on\n");
            is_suspend = DISABLE;
            mdelay(10);

            results = down_interruptible(&dw8767_lock);

            reg_data = 0x00;
            dw8767_smbus_write_byte(new_client, 0x03, &reg_data);

            lsb_data = (level_data) & 0x07;// 3bit LSB
            msb_data = (level_data >> 3) & 0xFF;// 8bit MSB

            dw8767_smbus_write_byte(new_client, 0x04, &lsb_data);
            dw8767_smbus_write_byte(new_client, 0x05, &msb_data);
            CPD_LOG("brightness Setting[reg0x04][LSB:0x%x][reg0x05][MSB:0x%x]\n",lsb_data,msb_data);

            dw8767_smbus_read_byte(new_client, 0x0A, &reg_data);
            // BLED12_EN, BL_EN enable
            reg_data |= 0x09;
            dw8767_smbus_write_byte(new_client, 0x0A, &reg_data);

            // 500KHZ, LED ramping time 50ms, PWM samling freq 4MHZ
            reg_data = 0x4C;
            dw8767_smbus_write_byte(new_client, 0x03, &reg_data);

            up(&dw8767_lock);
        }

        results = down_interruptible(&dw8767_lock);

        lsb_data = (level_data) & 0x07;
        msb_data = (level_data >> 3) & 0xFF;

        dw8767_smbus_write_byte(new_client, 0x04, &lsb_data);// 3bit LSB
        dw8767_smbus_write_byte(new_client, 0x05, &msb_data);// 8bit MSB
        CPD_LOG("brightness Setting[reg0x04][LSB:0x%x][reg0x05][MSB:0x%x]\n",lsb_data,msb_data);

        up(&dw8767_lock);
    } else {
        if(is_suspend == DISABLE){
            CPD_LOG( "backlight off\n");
            results = down_interruptible(&dw8767_lock);

            reg_data = 0x00;
            dw8767_smbus_write_byte(new_client, 0x04, &reg_data);
            dw8767_smbus_write_byte(new_client, 0x05, &reg_data);

            dw8767_smbus_read_byte(new_client, 0x0A, &reg_data);
            // BLED1_EN 0, BL_EN disable
            reg_data &= 0xE6;
            dw8767_smbus_write_byte(new_client, 0x0A, &reg_data);

            is_suspend = ENABLE;
            up(&dw8767_lock);
        }
    }

    return 0;
}

#if defined(CONFIG_LGE_PM_BACKLIGHT_CHG_CONTROL) || defined(CONFIG_LGE_PM_MTK_C_MODE)
unsigned int get_cur_main_lcd_level(void)
{
    return old_bl_level;
}
EXPORT_SYMBOL(get_cur_main_lcd_level);
#endif

unsigned char get_backlight_level(void)
{
    return mapping_level;
}

void chargepump_DSV_on(void)
{
    unsigned char data = 0x00;

    // OVP 32V, linear mode, active high PWM input
    data = 0x90;
    dw8767_smbus_write_byte(new_client, 0x02, &data);

    // 500KHZ, LED ramping time 50ms, PWM samling freq 4MHZ
    data = 0x44;
    dw8767_smbus_write_byte(new_client, 0x03, &data);

    // VPOS_EN, VNEG_EN enable
    data = 0x1E;
    dw8767_smbus_write_byte(new_client, 0x0C, &data);

    // DSV Vboost voltage 5.6V
    data = 0x16;
    dw8767_smbus_write_byte(new_client, 0x0D, &data);

    // VPOS +5.5v
    data = 0x1E;
    dw8767_smbus_write_byte(new_client, 0x0E, &data);

    // VNEG -5.5v
    data = 0x1E;
    dw8767_smbus_write_byte(new_client, 0x0F, &data);

#ifdef CONFIG_MTK_AAL_SUPPORT
    if(led_mode==LED_MODE_CUST_BLS_PWM)
        data = 0x40; // PWN enable
    else
        data = 0x00; // PWN disable
#endif
    dw8767_smbus_write_byte(new_client, 0x09, &data);

    CPD_LOG("%s\n",__func__);
}

void chargepump_DSV_off(void)
{
    unsigned char data = 0;

    data=0x18;
    dw8767_smbus_write_byte(new_client, 0x0C, &data);

    CPD_LOG("%s\n",__func__);
}

void dw8767_dsv_ctrl(unsigned int enable)
{
    if(enable)
        chargepump_DSV_on();
    else
        chargepump_DSV_off();

    CPD_LOG("%s : %s\n",__func__,(enable)? "enable":"disable");
}

#ifdef CONFIG_MTK_AAL_SUPPORT
#define REG_BL_ENABLE   0x09

void dw8767_reg_bl_en_ctrl(unsigned int enable)
{
    unsigned char reg_data = 0x00;
    unsigned char bl_en_data = 0x00;
    unsigned char write_data = 0x00;

    dw8767_smbus_read_byte(new_client, 0x0A, &reg_data);

    bl_en_data = reg_data;

    if(enable)
        bl_en_data |= REG_BL_ENABLE;
    else
        bl_en_data &= (~REG_BL_ENABLE);

    write_data = bl_en_data;
    dw8767_smbus_write_byte(new_client, 0x0A, &write_data);

    CPD_LOG("%s : bl_en %s(0x%x)\n",__func__,(enable)? "enable":"disable",write_data);
}

void dw8767_check_pwm_enable(void)
{
    unsigned char pwm_enable = 0;

    dw8767_smbus_read_byte(new_client, 0x09, &pwm_enable);
    pwm_enable |= 0x40;
    dw8767_smbus_write_byte(new_client, 0x09, &pwm_enable);

    CPD_LOG("%s for ESS : 0x%x\n",__func__,pwm_enable);
}
#endif

void dw8767_set_led_mode(int mode)
{
    led_mode = mode;
    CPD_LOG("%s : %s\n",__func__,(mode!=LED_MODE_CUST_LCM)? "LED_MODE_CUST_BLS_PWM(5)":"LED_MODE_CUST_LCM(4)");
}

static unsigned int get_backlight_rawdata(void)
{
    unsigned char dw8767_msb = 0;
    unsigned char dw8767_lsb = 0;
    unsigned int dw8767_level = 0;

    dw8767_smbus_read_byte(new_client, 0x04, &dw8767_lsb);
    dw8767_smbus_read_byte(new_client, 0x05, &dw8767_msb);

    dw8767_level |= ((dw8767_lsb & 0x07));// 3bit LSB8
    dw8767_level |= ((dw8767_msb & 0xFF) << 3);// 8bit MSB

    return dw8767_level;
}

static void set_backlight_rawdata(unsigned int level)
{
    unsigned char dw8767_msb = 0;
    unsigned char dw8767_lsb = 0;

    dw8767_lsb = (level) & 0x07;// 3bit LSB
    dw8767_msb = (level >> 3) & 0xFF;// 8bit MSB

    dw8767_smbus_write_byte(new_client, 0x04, &dw8767_lsb);
    dw8767_smbus_write_byte(new_client, 0x05, &dw8767_msb);
}

static ssize_t lcd_backlight_show_blmap(struct device *dev, struct device_attribute *attr, char *buf)
{
    int i, j;


    buf[0] = '{';

    for (i = 0, j = 2; i < 256 && j < PAGE_SIZE; ++i) {
            if (!(i % 15)) {
                    buf[j] = '\n';
                    ++j;
            }

            sprintf(&buf[j], "%d, ", bright_arr[i]);
            if (bright_arr[i] < 10)
                    j += 3;
            else if (bright_arr[i] < 100)
                    j += 4;
            else
                    j += 5;
    }

    buf[j] = '\n';
    ++j;
    buf[j] = '}';
    ++j;

    return j;
}

static ssize_t lcd_backlight_store_blmap(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int i;
    int j;
    int value, ret;

    if (count < 1)
            return count;

    if (buf[0] != '{')
            return -EINVAL;


    for (i = 1, j = 0; i < count && j < 256; ++i) {
            if (!isdigit(buf[i]))
                    continue;

            ret = sscanf(&buf[i], "%d", &value);
            if (ret < 1)
                    pr_err("read error\n");
            bright_arr[j] = (unsigned int)value;

            while (isdigit(buf[i]))
                    ++i;
            ++j;
    }

    return count;
}

static ssize_t show_rawdata(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "dw8767 brightness code : %d\n",get_backlight_rawdata());
}

static ssize_t store_rawdata(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char *pvalue = NULL;
    unsigned int level = 0;
    size_t size = 0;

    level = simple_strtoul(buf,&pvalue,10);
    size = pvalue - buf;

    if (*pvalue && isspace(*pvalue))
            size++;

    printk("[DW8767] %s : [%d] \n",__func__,level);
    set_backlight_rawdata(level);

    return count;
}

DEVICE_ATTR(brightness_code, 0644, show_rawdata, store_rawdata);
DEVICE_ATTR(bl_blmap, 0644, lcd_backlight_show_blmap, lcd_backlight_store_blmap);

static int dw8767_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
   CPD_FUN();

   new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);

   device_create_file(&client->dev, &dev_attr_bl_blmap);
   device_create_file(&client->dev, &dev_attr_brightness_code);

   memset(new_client, 0, sizeof(struct i2c_client));

   new_client = client;

   return 0;
}

#if 0
static int dw8767_smbus_read_byte_block(struct i2c_client *client, unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    return dw8767_i2c_read(client,reg_addr,data,len);
}

static int dw8767_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret;
    new_client = client;
    int err;

    CPD_FUN();

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        CPD_LOG("i2c_check_functionality error\n");
        return -1;
    }

    sema_init(&dw8767_lock, 1); // Add semaphore for lcd and flash i2c communication.

    if (client == NULL)
        CPD_LOG("%s client is NULL\n", __func__);
    else
        CPD_LOG("%s is OK\n", __func__);

    return 0;
}

static int dw8767_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    return 0;
}
#endif

static int dw8767_remove(struct i2c_client *client)
{
    new_client = NULL;
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dw8767_of_match[] = {
    {.compatible = "mediatek,i2c_lcd_bias",},
    {},
};

MODULE_DEVICE_TABLE(of, dw8767_of_match);
#endif

static struct i2c_driver dw8767_i2c_driver = {
    .driver = {
        .name   = DW8767_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = dw8767_of_match,
#endif
    },
    .probe      = dw8767_driver_probe,
    .remove     = dw8767_remove,
    .id_table   = dw8767_i2c_id,
};

static int dw8767_pd_probe(struct platform_device *pdev)
{
    if(i2c_add_driver(&dw8767_i2c_driver)){
        CPD_ERR("add driver error\n");
        return -1;
    }else{
        CPD_LOG("i2c_add_driver OK\n");
    }

    return 0;
}

static int dw8767_pd_remove(struct platform_device *pdev)
{
    CPD_FUN();
    i2c_del_driver(&dw8767_i2c_driver);

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void dw8767_early_suspend(struct early_suspend *h)
{
    int err = 0;
    unsigned char data;
    unsigned int results = 0;

    results = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
    data = 0x00; //backlight2 brightness 0
    err = dw8767_smbus_write_byte(new_client, 0x04, &data);
    err = dw8767_smbus_write_byte(new_client, 0x05, &data);

    err = dw8767_smbus_read_byte(new_client, 0x0A, &data);
    data &= 0x06;  // FLASH_MODE, FLASH Enable, AND operation

    err = dw8767_smbus_write_byte(new_client, 0x0A, &data);
    up(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
    CPD_LOG("early_suspend  [%d]",data);
}

static void dw8767_late_resume(struct early_suspend *h)
{
    int err = 0;
    unsigned char data1;
    unsigned int results = 0;

    mdelay(50);
    results = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.

    data1 = 0x07;  // Backlight brightness LSB 3bits, 0b111
    dw8767_smbus_write_byte(new_client, 0x04, &data1);
    err = dw8767_smbus_write_byte(new_client, 0x05, &current_brightness);

    err = dw8767_smbus_read_byte(new_client, 0x0A, &data1);
    data1 |= 0x09;  // BLED12_EN, BL_EN enable

    err = dw8767_smbus_write_byte(new_client, 0x0A, &data1);
    up(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
    CPD_LOG("dw8767_late_resume  [%d]",data1);

    mt_set_gpio_out(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
}

static struct early_suspend dw8767_early_suspend_desc = {
    .level      = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend    = dw8767_early_suspend,
    .resume     = dw8767_late_resume,
};
#endif

static struct platform_driver dw8767_backlight_driver = {
    .probe      = dw8767_pd_probe,
    .remove     = dw8767_pd_remove,
    .driver     = {
            .name  = BACKLIHGT_NAME,
            .owner = THIS_MODULE,
    }
};

static int __init dw8767_init(void)
{
    CPD_FUN();

    sema_init(&dw8767_lock, 1);

    mt_set_gpio_mode(GPIO_LCD_BL_EN, GPIO_LCD_BL_EN_MODE);
    mt_set_gpio_pull_enable(GPIO_LCD_BL_EN, GPIO_PULL_ENABLE);
    mt_set_gpio_dir(GPIO_LCD_BL_EN, GPIO_DIR_OUT);

    i2c_register_board_info(3, &i2c_dw8767, 1);

    if (i2c_add_driver(&dw8767_i2c_driver) != 0)
        CPD_ERR("Failed to register dw8767 driver");

    return 0;
}

static void __exit dw8767_exit(void)
{
    platform_driver_unregister(&dw8767_backlight_driver);
}

void dw8767_flash_strobe_prepare(char OnOff,char ActiveHigh)
{
    int err = 0;
    unsigned int results = 0;

    results = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.

    err = dw8767_smbus_read_byte(new_client, 0x09, &strobe_ctrl);

    // flash_en register(0x0A) setting position change.
    strobe_ctrl &= 0xF3;
    flash_ctrl = OnOff;

    if(ActiveHigh)
        strobe_ctrl |= 0x20;
    else
        strobe_ctrl &= 0xDF;

    if(OnOff == 1){
        CPD_LOG("Strobe mode On\n");
        strobe_ctrl |= 0x10;
    }else if(OnOff == 2){
        CPD_LOG("Torch mode On\n");
        strobe_ctrl |= 0x10;
    }else{
        CPD_LOG("Flash Off\n");
        strobe_ctrl &= 0xEF;
    }
    // flash_en register(0x0A) setting position change.
    err = dw8767_smbus_write_byte(new_client, 0x09, &strobe_ctrl);

    up(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
}

//strobe enable
void dw8767_flash_strobe_en(void)
{
    // flash_en register(0x0A) setting position change.
    int err = 0;
    unsigned char flash_OnOff = 0;
    unsigned int results = 0;

    results = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.

    err = dw8767_smbus_read_byte(new_client, 0x0A, &flash_OnOff);
#if 0
    if(flash_ctrl == 1)
        flash_OnOff |= 0x66;
    else if(flash_ctrl == 2)
        flash_OnOff |= 0x62;
    else
        flash_OnOff &= 0x99;
#endif
            
    if(flash_ctrl == 1)
        flash_OnOff |= 0x06;
    else if(flash_ctrl == 2)
        flash_OnOff |= 0x02;
    else
        flash_OnOff &= 0xF9;

    err = dw8767_smbus_write_byte(new_client, 0x0A, &flash_OnOff);

    up(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
    // flash_en register(0x0A) setting position change.
}

//[LGE_CHANGE_S] [Himchan.Lee@lge.com] set torch level
void dw8767_flash_torch_level(int level)
{
    unsigned char torch_level = 0;
    unsigned char rdata = 0;
    unsigned char wdata = 0;
    int err = 0;
    err = down_interruptible(&dw8767_lock);

    err = dw8767_smbus_read_byte(new_client, 0x06, &rdata);
    CPD_LOG("dw8767_flash_torch_level read =0x%x\n", rdata);

    if (level == 1)
        torch_level = 0x00;
    else if (level == 2)
        torch_level = 0x10;
    else if (level == 3)
        torch_level = 0x20;
    else if (level == 4)
        torch_level = 0x30;
    else if (level == 5)
        torch_level = 0x40;
    else if (level == 6)
        torch_level = 0x50;
    else if (level == 7)
        torch_level = 0x60;
    else if (level == 8)
        torch_level = 0x70;
    else if (level == 9)
        torch_level = 0x80;
    else if (level == 10)
        torch_level = 0x90;
    else if (level == 11)
        torch_level = 0xA0;
    else if (level == 12)
        torch_level = 0xB0;
    else if (level == 13)
        torch_level = 0xC0;
    else if (level == 14)
        torch_level = 0xD0;
    else if (level == 15)
        torch_level = 0xE0;
    else {
        torch_level = 0x50; //125mA
    }

    wdata = torch_level | (0x0F & rdata);
    err = dw8767_smbus_write_byte(new_client, 0x06, &wdata);
    CPD_LOG("dw8767_flash_torch_level write =0x%x\n", wdata);
    up(&dw8767_lock);
}
//[LGE_CHANGE_E] [Himchan.Lee@lge.com] set torch level


//strobe level
void dw8767_flash_strobe_level(char level)
{
    int err = 0;
    unsigned char data1=0;
    unsigned char data2=0;
    unsigned char torch_level;
    unsigned char strobe_timeout = 0x1F;
    unsigned int results = 0;

    results = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
#if 0 // Add Main Flash current(4 Step)
    if( level == 1)
    {
        torch_level = 0x20;
    }
    else
    {
        torch_level = 0x50;
    }

    err = dw8767_smbus_read_byte(new_client, 0x06, &data1);

    if(31 < level)
    {
        data1= torch_level | 0x0A;
        strobe_timeout = 0x0F;
    }
    else if(level < 0)
    {
        data1= torch_level ;
    }
    else
    {
        data1= torch_level | level;
    }
    // Add Main Flash current(4 Step)
#else
    torch_level = 0x50;

    err = dw8767_smbus_read_byte(new_client, 0x06, &data1);

    strobe_timeout = 0x1F; // Flash Timing Tuning
    if(level < 0)
        data1= torch_level;
    else if(level == 1)
        data1= torch_level | 0x02; //300mA
    else if(level == 2)
        data1= torch_level | 0x04; //500mA
    else if(level == 3)
        data1= torch_level | 0x07; //800mA
    else if(level == 4)
        data1= torch_level | 0x09; //1000mA
    else
        data1= torch_level | 0x09;
#endif
    // Add Main Flash current(4 Step)
    /*
    if(0)
    {
        CPD_LOG("Batt temp=%d\n", BMT_status.temperature );

        torch_level = 0xF0 & data1;
        level = 0x0F & data1;
        torch_level = 0xF0 & (torch_level >> 2);
        level = 0x0F & (level >> 2);

        data1 = torch_level | level;
    }
    */
    CPD_LOG("Flash Level =0x%x\n", data1);
    err = dw8767_smbus_write_byte(new_client, 0x06, &data1);

    data2 = 0x40 | strobe_timeout;
    CPD_LOG("Storbe Timeout =0x%x\n", data2);
    err |= dw8767_smbus_write_byte(new_client, 0x07, &data2);
    up(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
}
EXPORT_SYMBOL(dw8767_flash_strobe_en);
EXPORT_SYMBOL(dw8767_flash_strobe_prepare);
EXPORT_SYMBOL(dw8767_flash_strobe_level);
MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("dw8767 driver");
MODULE_LICENSE("GPL");

module_init(dw8767_init);
module_exit(dw8767_exit);
