/*
* This software program is licensed subject to the GNU General Public License
* (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
* (C) Copyright 2011 Bosch Sensortec GmbH
* All Rights Reserved
*/
/* file lm3632.c
brief This file contains all function implementations for the lm3632 in linux
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
//#include <mt_typedefs.h>
#include <mt_gpio.h>
//#include <cust_gpio_usage.h>
#include <mach/gpio_const.h>
#include <../inc/lm3632.h>
#include <linux/platform_device.h>
//#include <cust_acc.h>
//#include <linux/hwmsensor.h>
//#include <linux/hwmsen_dev.h>
//#include <linux/sensors_io.h>
//#include <linux/hwmsen_helper.h>
#include <linux/ctype.h>
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
#include <soc/mediatek/lge/board_lge.h>
extern int lge_get_maker_id(void);
#endif
#include <linux/leds.h>
#define LM3632_DSV_VPOS_EN  GPIO_DSV_AVEE_EN
#define LM3632_DSV_VPOS_EN_MODE GPIO_DSV_AVEE_EN_M_GPIO
#define LM3632_DSV_VNEG_EN GPIO_DSV_AVDD_EN
#define LM3632_DSV_VNEG_EN_MODE GPIO_DSV_AVDD_EN_M_GPIO
//#define LCD_LED_MAX 0x7F
//#define LCD_LED_MIN 0
//#define DEFAULT_BRIGHTNESS 0x73 //for 20mA
#define LM3632_MIN_VALUE_SETTINGS 10 /* value leds_brightness_set*/
#define LM3632_MAX_VALUE_SETTINGS 255 /* value leds_brightness_set*/
#define MIN_MAX_SCALE(x) (((x)<LM3632_MIN_VALUE_SETTINGS) ? LM3632_MIN_VALUE_SETTINGS : (((x)>LM3632_MAX_VALUE_SETTINGS) ? LM3632_MAX_VALUE_SETTINGS:(x)))
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
#define LED_MODE_CUST_LCM 4
#define LED_MODE_CUST_BLS_PWM 5
#endif
#define BACKLIHGT_NAME "charge-pump"
#define LM3632_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)
#define LM3632_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))
//#define LM3632_DEV_NAME "charge-pump"
#define LM3632_DEV_NAME "lm3632"
#define CPD_TAG                  "[chargepump] "
#define CPD_FUN(f)               printk(CPD_TAG"%s\n", __FUNCTION__)
#define CPD_ERR(fmt, args...)    printk(CPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define CPD_LOG(fmt, args...)    printk(CPD_TAG fmt, ##args)
//GPIO
#ifndef GPIO_LCD_BL_EN
#define GPIO_LCD_BL_EN         (GPIO100 | 0x80000000)
#define GPIO_LCD_BL_EN_M_GPIO   GPIO_MODE_00
#endif
// I2C variable
static struct i2c_client *new_client = NULL;
static const struct i2c_device_id lm3632_i2c_id[] = {{LM3632_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_lm3632={ I2C_BOARD_INFO(LM3632_DEV_NAME, 0x11)};
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV7) || defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
int lm3632_old_bl_level;
static int led_mode = LED_MODE_CUST_LCM;
#endif
// Flash control
unsigned char lm3632_strobe_ctrl;
unsigned char lm3632_flash_ctrl=0; // flash_en register(0x0A) setting position change.
unsigned char lm3632_mapping_level;
#if 0
unsigned char lm3632_bright_arr[] = {  // array index max 100, value under 255
        20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 22, 22, 22, 22, 24, 24, 24,  // 19
        26, 26, 26, 28, 28, 30, 30, 32, 32, 34, 34, 36, 38, 38, 40, 42, 42, 44, 36, 48,  // 39
        48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 76, 78, 80, 82, 86, 88, 90,  // 59
        94, 96, 98, 102, 104, 108, 110, 114, 118, 120, 124, 128, 130, 134, 138, 142, 146, 148, 152, 156,  //79
        160, 164, 168, 172, 178, 182, 186, 190, 194, 200, 204, 208, 212, 218, 222, 228, 232, 238, 242, 248, 255  // 100
};
#endif
#define LGE_DISPLAY_BACKLIGHT_256_TABLE
unsigned int lm3632_bright_arr[] = {
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 12, 13, 13, 14, 15, 16, 17,                                             // 19
    17, 18, 19, 20, 20, 21, 22, 23, 24, 24, 25, 26, 27, 27, 28, 29, 30, 31, 31, 32,                                             // 39
    33, 34, 34, 35, 36, 37, 38, 38, 39, 40, 41, 41, 42, 43, 44, 45, 45, 46, 47, 48,                                             // 59
    49, 51, 54, 57, 60, 63, 66, 69, 72, 74, 77, 80, 83, 86, 89, 92, 95, 97, 100, 103,                                           // 79
    106, 109, 112, 115, 118, 120, 123, 126, 129, 132, 135, 138, 141, 143, 146, 149, 152, 155, 158, 161,                         // 99
    164, 166, 169, 172, 175, 178, 181, 184, 187, 192, 197, 202, 208, 213, 218, 224, 229, 234, 240, 245,                         // 119
    250, 256, 261, 266, 272, 277, 282, 288, 293, 298, 304, 309, 314, 320, 325, 330, 336, 341, 346, 352,                         // 139
    357, 362, 368, 373, 378, 384, 389, 394, 400, 405, 410, 416, 421, 426, 432, 437, 442, 448, 456, 465,                         // 159
    473, 482, 505, 520, 535, 550, 560, 579, 585, 600, 600, 600, 607, 610, 614, 619, 622, 624, 626, 635,                         // 179
    702, 708, 714, 720, 726, 732, 738, 744, 750, 756, 762, 768, 774, 780, 786, 792, 798, 804, 810, 816,                         // 199
    822, 828, 830, 839, 847, 856, 865, 877, 890, 903, 916, 929, 942, 955, 968, 980, 993, 1006, 1019, 1032,                      // 219
    1045, 1058, 1071, 1083, 1096, 1109, 1122, 1135, 1148, 1161, 1174, 1186, 1199, 1212, 1225, 1238, 1251, 1264, 1277, 1289,     // 239
    1302, 1315, 1328, 1341, 1354, 1367, 1380, 1392, 1405, 1418, 1431, 1444, 1457, 1470, 1483, 1496                              // 255
};
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
unsigned int lm3632_bright_arr_tovis[] = {
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 12, 13, 13, 14, 15, 16, 17,                                             // 19
    17, 18, 19, 20, 20, 21, 22, 23, 24, 24, 25, 26, 27, 27, 28, 29, 30, 31, 31, 32,                                             // 39
    33, 34, 34, 35, 36, 37, 38, 38, 39, 40, 41, 41, 42, 43, 44, 45, 45, 46, 47, 48,                                             // 59
    49, 51, 54, 57, 60, 63, 66, 69, 72, 74, 77, 80, 83, 86, 89, 92, 95, 97, 100, 103,                                           // 79
    106, 109, 112, 115, 118, 120, 123, 126, 129, 132, 135, 138, 141, 143, 146, 149, 152, 155, 158, 161,                         // 99
    164, 166, 169, 172, 175, 178, 181, 184, 187, 192, 197, 202, 208, 213, 218, 224, 229, 234, 240, 245,                         // 119
    250, 256, 261, 266, 272, 277, 282, 288, 293, 298, 304, 309, 314, 320, 325, 330, 336, 341, 346, 352,                         // 139
    357, 362, 368, 373, 378, 384, 389, 394, 400, 405, 410, 419, 426, 435, 444, 453, 462, 468, 474, 482,                         // 159
    490, 499, 517, 532, 547, 562, 572, 591, 600, 600, 610, 620, 627, 630, 635, 636, 642, 646, 651, 659,                         // 179
    720, 726, 732, 738, 744, 751, 758, 765, 772, 779, 786, 793, 800, 807, 814, 821, 829, 840, 852, 863,                         // 199
    873, 882, 891, 900, 909, 918, 928, 937, 944, 957, 970, 983, 996, 1009, 1022, 1035, 1048, 1061, 1074, 1087,                  // 219
    1100, 1113, 1126, 1139, 1152, 1165, 1178, 1191, 1204, 1217, 1230, 1243, 1257, 1271, 1285, 1299, 1313, 1327, 1341, 1355,     // 239
    1369, 1383, 1397, 1411, 1425, 1439, 1453, 1467, 1481, 1500, 1519, 1533, 1555, 1575, 1595, 1610                              // 255
};
#endif
static unsigned char is_suspend = 0;
struct semaphore lm3632_lock; // Add semaphore for lcd and flash i2c communication.
/* generic */
#define LM3632_MAX_RETRY_I2C_XFER (100)
#define LM3632_I2C_WRITE_DELAY_TIME 1
/* i2c read routine for API*/
static char lm3632_i2c_read(struct i2c_client *client, u8 reg_addr,
            u8 *data, u8 len)
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
        return 0;
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
        for (retry = 0; retry < LM3632_MAX_RETRY_I2C_XFER; retry++) {
            if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
                break;
            else
                mdelay(LM3632_I2C_WRITE_DELAY_TIME);
        }
        if (LM3632_MAX_RETRY_I2C_XFER <= retry) {
            CPD_ERR("I2C xfer error\n");
            return -EIO;
        }
        return 0;
        #endif
}
/* i2c write routine for */
static char lm3632_i2c_write(struct i2c_client *client, u8 reg_addr,
            u8 *data, u8 len)
{
        #if !defined BMA_USE_BASIC_I2C_FUNC
        s32 dummy;
        if (NULL == client)
            return -1;
        while (0 != len--) {
            #if 1
            dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
            #else
            buffer[0] = reg_addr;
            buffer[1] = *data;
            dummy = i2c_master_send(client, (char *)buffer, 2);
            #endif
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
            for (retry = 0; retry < LM3632_MAX_RETRY_I2C_XFER; retry++) {
                if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0) {
                    break;
                }
                else {
                    mdelay(LM3632_I2C_WRITE_DELAY_TIME);
                }
            }
            if (LM3632_MAX_RETRY_I2C_XFER <= retry) {
                return -EIO;
            }
            reg_addr++;
            data++;
        }
        #endif
        //CPD_LOG("lm3632_i2c_write \n");
        return 0;
}
static int lm3632_smbus_read_byte(struct i2c_client *client,
            unsigned char reg_addr, unsigned char *data)
{
        return lm3632_i2c_read(client,reg_addr,data,1);
}
static int lm3632_smbus_write_byte(struct i2c_client *client,
            unsigned char reg_addr, unsigned char *data)
{
        int ret_val = 0;
        int i = 0;
        ret_val = lm3632_i2c_write(client,reg_addr,data,1);
        for ( i = 0; i < 5; i++)
        {
            if (ret_val != 0)
                lm3632_i2c_write(client,reg_addr,data,1);
            else
                return ret_val;
        }
        return ret_val;
}
#if 0
bool check_charger_pump_vendor()
{
        int err = 0;
        unsigned char data = 0;
        err = lm3632_smbus_read_byte(new_client,0x01,&data);
        if(err < 0)
            CPD_ERR("%s read charge-pump vendor id fail\n", __func__);
        CPD_ERR("%s vendor is 0x%x\n", __func__, data&0x03);
        if((data&0x03) == 0x03) //Richtek
            return FALSE;
        else
            return TRUE;
}
#endif
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
int lm3632_chargepump_set_backlight_level(unsigned int level)
{
#else
int chargepump_set_backlight_level(unsigned int level)
{
#endif
    unsigned int data = 0;
    unsigned char data1 = 0;
    unsigned char lsb_data = 0x00; // 3bit
    unsigned char msb_data = 0x00; // 8bit
    int ret = 0;
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV7) || defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
    lm3632_old_bl_level = level;
#endif
    CPD_LOG("chargepump_set_backlight_level  [%d]\n",level);
    if (level == 0) {
        if(is_suspend == false) {
            CPD_LOG( "backlight off\n");
            ret = down_interruptible(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
            data1 = 0x00; //backlight2 brightness 0
            lm3632_smbus_write_byte(new_client, 0x04, &data1);  // LSB 3bit all 0
            lm3632_smbus_write_byte(new_client, 0x05, &data1);  // MSB 3bit all 0
            lm3632_smbus_read_byte(new_client, 0x0A, &data1);
            // data1 &= 0x16;  // BLED1_EN, FLASH_MODE, FLASH Enable, AND operation
            data1 &= 0xE6;  // BLED1_EN 0, BL_EN disable
            lm3632_smbus_write_byte(new_client, 0x0A, &data1);
            is_suspend = true;// Move backlight suspend setting position into semaphore
            up(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        }
    } else {
        level = MIN_MAX_SCALE(level);
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
        if (lge_get_maker_id() == 0)
            data = lm3632_bright_arr[level]; // LGD
        else
            data = lm3632_bright_arr_tovis[level]; // TOVIS
        //CPD_LOG("[%s] %s data = %d\n",(lge_get_maker_id()!=0)? "TOVIS":"LGD",__func__, data);
#else
        data = lm3632_bright_arr[level];
        //CPD_LOG("%s data = %d\n",__func__, data);
#endif
        lm3632_mapping_level = data;
        if (is_suspend == true) {
            is_suspend = false;
            mdelay(10);
            ret = down_interruptible(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
            data1 = 0x70; // OVP 29V, Linear mapping mode
            lm3632_smbus_write_byte(new_client, 0x02, &data1);
            data1 = 0xCD; // 1Mhz, ramp rate : 100ms
            lm3632_smbus_write_byte(new_client, 0x03, &data1);
            lsb_data = (data) & 0x07; // 3bit LSB
            msb_data = (data >> 3) & 0xFF; // 8bit MSB
            lm3632_smbus_write_byte(new_client, 0x04, &lsb_data); // LSB
            lm3632_smbus_write_byte(new_client, 0x05, &msb_data); // MSB
            CPD_LOG("[LM3632]-backlight brightness Setting[reg0x04][MSB:0x%x]\n",lsb_data);
            CPD_LOG("[LM3632]-backlight brightness Setting[reg0x05][LSB:0x%x]\n",msb_data);
            lm3632_smbus_read_byte(new_client, 0x0A, &data1);
            data1 |= 0x11;  // BLED1_EN, BL_EN enable
            lm3632_smbus_write_byte(new_client, 0x0A, &data1);
            up(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        }
        if (level != 0) {
            ret = down_interruptible(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
            lsb_data = (data) & 0x07; // 3bit LSB
            msb_data = (data >> 3) & 0xFF; // 8bit MSB
            lm3632_smbus_write_byte(new_client, 0x04, &lsb_data); // LSB
            lm3632_smbus_write_byte(new_client, 0x05, &msb_data); // MSB
            CPD_LOG("[LM3632]-backlight brightness Setting[reg0x04][MSB:0x%x]\n",lsb_data);
            CPD_LOG("[LM3632]-backlight brightness Setting[reg0x05][LSB:0x%x]\n",msb_data);
            up(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        } // Move backlight suspend setting position into semaphore
    }
    return 0;
}
#if defined(CONFIG_LGE_PM_BACKLIGHT_CHG_CONTROL) || defined(CONFIG_LGE_PM_MTK_C_MODE)
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
unsigned int lm3632_get_cur_main_lcd_level(void)
{
#else
unsigned int get_cur_main_lcd_level(void)
{
#endif
    return lm3632_old_bl_level;
}
EXPORT_SYMBOL(get_cur_main_lcd_level);
#endif
int chargepump_backlight_level_test(unsigned int level)
{
        unsigned char data = 0;
        if(level > 255)
        {
                level = 255;
        }
        data = 0x70;// OVP 29V, Linear mapping mode
        lm3632_smbus_write_byte(new_client, 0x02, &data);
        data = 0x07;  // Backlight brightness LSB 3bits, 0b111
        lm3632_smbus_write_byte(new_client, 0x04, &data);
        data = level;
        lm3632_smbus_write_byte(new_client, 0x05, &data);  // MSB 8 bit control
        CPD_LOG("backlight brightness Test[reg0x05][value:0x%x]\n",data);
        lm3632_smbus_read_byte(new_client, 0x0A, &data);
        data |= 0x11;  // BLED1_EN, BL_EN enable
        lm3632_smbus_write_byte(new_client, 0x0A, &data);
        return 0;
}
unsigned char get_lm3632_backlight_level(void)
{
        return lm3632_mapping_level;
}
void chargepump_DSV_on(void)
{
        unsigned char data = 0;
		int ret = 0;
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
	ret = down_interruptible(&lm3632_lock);
	if (lge_get_maker_id() == 0) { // LGD
		data=0x70;
		lm3632_smbus_write_byte(new_client, 0x02, &data);// OVP 29V, linear mode, TI recommand
		data=0x21;
		lm3632_smbus_write_byte(new_client, 0x0D, &data);// LV5 LGD panel DSV Vboost voltage 6.15V
		data=0x1E;
		lm3632_smbus_write_byte(new_client, 0x0C, &data);// VPOS_EN=1, VNEG_EN=1, EXT_EN=0(I2C Control)
		data=0x1A;
		lm3632_smbus_write_byte(new_client, 0x0E, &data);// Vpos voltage setting as +5.3v
		data=0x28;
		lm3632_smbus_write_byte(new_client, 0x0F, &data);// Vneg voltage setting as -6.0v
		if(led_mode==LED_MODE_CUST_BLS_PWM){
			lm3632_smbus_read_byte(new_client, 0x09, &data);
			CPD_LOG("[lm3632] pwm enable : 0x09 reg value : 0x%x\n",data);
			data=0x40;
			lm3632_smbus_write_byte(new_client, 0x09, &data);// pwm enable
		}
	} else {
		data=0x70;
		lm3632_smbus_write_byte(new_client, 0x02, &data);// OVP 29V, linear mode, TI recommand
		data=0x18;
		lm3632_smbus_write_byte(new_client, 0x0D, &data); // DSV Vboost voltage 5.7V
		data=0x1E;
		lm3632_smbus_write_byte(new_client, 0x0E, &data);// Vpos voltage setting as +5.5v
		data=0x1E;
		lm3632_smbus_write_byte(new_client, 0x0F, &data);// Vneg voltage setting as -5.5v
		//data=0x1E;
		data=0x04;
		lm3632_smbus_write_byte(new_client, 0x0C, &data);// VPOS_EN=1, EXT_EN=0(I2C Control)
		mdelay(10);
		data=0x06;
		lm3632_smbus_write_byte(new_client, 0x0C, &data);// VNEG_EN=1, EXT_EN=0(I2C Control)
	}
	up(&lm3632_lock);
#else
        data=0x70;
        lm3632_smbus_write_byte(new_client, 0x02, &data);// OVP 29V, linear mode, TI recommand
        // LCM bias(Vpos, Vneg) controlled by external pin, not I2C
        data=0x18;
        lm3632_smbus_write_byte(new_client, 0x0D, &data);  // LV5 LGD panel DSV Vboost voltage 5.7V
        data=0x1E;
        lm3632_smbus_write_byte(new_client, 0x0C, &data);// VPOS_EN=1, VNEG_EN=1, EXT_EN=0(I2C Control)
        // Vpos voltage setting as +5.5v
        data=0x1E;
        lm3632_smbus_write_byte(new_client, 0x0E, &data);
        // Vneg voltage setting as -5.5v
        data=0x1E;
        lm3632_smbus_write_byte(new_client, 0x0F, &data);
#endif
        CPD_LOG("[lm3632]chargepump DSV on\n");
}
void chargepump_DSV_off(void)
{
        unsigned char data = 0;
        data=0x18;
        lm3632_smbus_write_byte(new_client, 0x0C, &data);// VPOS_EN=0, VNEG_EN=0, EXT_EN=1
        CPD_LOG("[lm3632]chargepump DSV off\n");
}
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
void lm3632_dsv_ctrl(int enable)
{
	if(enable==1)
		chargepump_DSV_on();
	else
		chargepump_DSV_off();
}
void lm3632_check_pwm_enable(void){
	/* Prevent unsetting pwm enable */
	unsigned char data = 0;
	unsigned char n_data = 0;
	int ret = 0;
	ret = down_interruptible(&lm3632_lock);
	lm3632_smbus_read_byte(new_client, 0x09, &data);
	n_data = (data & 0x40);
	if(n_data!=0x40){
		data|=0x40;
		lm3632_smbus_write_byte(new_client, 0x09, &data);// pwm enable
	}
	lm3632_smbus_read_byte(new_client, 0x03, &data);
	n_data = (data & 0xCD);
	if(n_data!=0xCD){
		data |= 0xCD;
		lm3632_smbus_write_byte(new_client, 0x03, &data);
	}
	lm3632_smbus_read_byte(new_client, 0x0A, &data);
	n_data = (data & 0x11);
	if(n_data!=0x11){
		data |= 0x11;
		lm3632_smbus_write_byte(new_client, 0x0A, &data);
	}
	up(&lm3632_lock);
}
void lm3632_set_led_mode(int mode){
	/* Distinguish rev E, rev F, rev 1.0 */
	led_mode = mode;
	CPD_LOG("[lm3632] LED mode %s\n",(mode!=LED_MODE_CUST_LCM)? "LED_MODE_CUST_BLS_PWM(5)":"LED_MODE_CUST_LCM(4)");
}
#endif
static unsigned int get_lm3632_backlight_rawdata(void)
{
        unsigned char lm3632_msb = 0;
        unsigned char lm3632_lsb = 0;
        unsigned int lm3632_level = 0;
        lm3632_smbus_read_byte(new_client, 0x04, &lm3632_lsb);
        lm3632_smbus_read_byte(new_client, 0x05, &lm3632_msb);
        lm3632_level |= ((lm3632_msb & 0xFF) << 3); // 8bit MSB
        lm3632_level |= ((lm3632_lsb & 0x07)); // 3bit LSB8
        return lm3632_level;
}
static void set_lm3632_backlight_rawdata(unsigned int level)
{
        unsigned char lm3632_msb = 0;
        unsigned char lm3632_lsb = 0;
       lm3632_lsb = (level) & 0x07; // 3bit LSB
       lm3632_msb = (level >> 3) & 0xFF; // 8bit MSB
        lm3632_smbus_write_byte(new_client, 0x04, &lm3632_lsb);
        lm3632_smbus_write_byte(new_client, 0x05, &lm3632_msb);
}
static ssize_t lm3632_lcd_backlight_show_blmap(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        int i, j;
        buf[0] = '{';
        for (i = 0, j = 2; i < 256 && j < PAGE_SIZE; ++i) {
                if (!(i % 15)) {
                        buf[j] = '\n';
                        ++j;
                }
                sprintf(&buf[j], "%d, ", lm3632_bright_arr[i]);
                if (lm3632_bright_arr[i] < 10)
                        j += 3;
                else if (lm3632_bright_arr[i] < 100)
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
static ssize_t lm3632_lcd_backlight_store_blmap(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
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
                lm3632_bright_arr[j] = (unsigned int)value;
                while (isdigit(buf[i]))
                        ++i;
                ++j;
        }
        return count;
}
static ssize_t show_lm3632_rawdata(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "lm3632 brightness code : %d\n",get_lm3632_backlight_rawdata());
}
static ssize_t store_lm3632_rawdata(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        char *pvalue = NULL;
        unsigned int level = 0;
        size_t size = 0;
        level = simple_strtoul(buf,&pvalue,10);
        size = pvalue - buf;
        if (*pvalue && isspace(*pvalue))
                size++;
        printk("[LM3632] store_lm3632_rawdata : [%d] \n",level);
        set_lm3632_backlight_rawdata(level);
        return count;
}
DEVICE_ATTR(lm3632_brightness_code, 0644, show_lm3632_rawdata, store_lm3632_rawdata);
DEVICE_ATTR(lm3632_bl_blmap, 0644, lm3632_lcd_backlight_show_blmap, lm3632_lcd_backlight_store_blmap);
static int lm3632_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
       CPD_FUN();
       new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
       device_create_file(&client->dev, &dev_attr_lm3632_bl_blmap);
       device_create_file(&client->dev, &dev_attr_lm3632_brightness_code);
       memset(new_client, 0, sizeof(struct i2c_client));
       new_client = client;
       return 0;
}
static int lm3632_remove(struct i2c_client *client)
{
        new_client = NULL;
        return 0;
}
#ifdef CONFIG_OF
static const struct of_device_id lm3632_of_match[] = {
    {.compatible = "mediatek,i2c_lcd_bias",},
    {},
};
MODULE_DEVICE_TABLE(of, lm3632_of_match);
#endif
static struct i2c_driver lm3632_i2c_driver = {
        .driver = {
        //      .owner  = THIS_MODULE,
        .name   = LM3632_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = lm3632_of_match,
#endif
        },
        .probe      = lm3632_driver_probe,
        .remove     = lm3632_remove,
        //  .detect     = lm3632_detect,
        .id_table   = lm3632_i2c_id,
        //  .address_data = &lm3632250_i2c_addr_data,
};
static int lm3632_pd_probe(struct platform_device *pdev) 
{
        if(i2c_add_driver(&lm3632_i2c_driver))
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
static int lm3632_pd_remove(struct platform_device *pdev)
{
        CPD_FUN();
        i2c_del_driver(&lm3632_i2c_driver);
        return 0;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3632_early_suspend(struct early_suspend *h)
{
        int err = 0;
        unsigned char data;
		int ret = 0;
        ret = down_interruptible(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        data = 0x00; //backlight2 brightness 0
        err = lm3632_smbus_write_byte(new_client, 0x04, &data); 
        err = lm3632_smbus_write_byte(new_client, 0x05, &data);
        err = lm3632_smbus_read_byte(new_client, 0x0A, &data);
        data &= 0x06;  // FLASH_MODE, FLASH Enable, AND operation    
        err = lm3632_smbus_write_byte(new_client, 0x0A, &data);
        up(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        CPD_LOG("early_suspend  [%d]",data);
        //mt_set_gpio_out(GPIO_LCM_BL_EN,GPIO_OUT_ZERO);
}
static void lm3632_late_resume(struct early_suspend *h)
{
        int err = 0;
        unsigned char data1;
		int ret = 0;
        //mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ONE);
        mdelay(50);
        ret = down_interruptible(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        data1 = 0x07;  // Backlight brightness LSB 3bits, 0b111
        lm3632_smbus_write_byte(new_client, 0x04, &data1);        	
        err = lm3632_smbus_write_byte(new_client, 0x05, &current_brightness);
        err = lm3632_smbus_read_byte(new_client, 0x0A, &data1);
        data1 |= 0x11;  // BLED1_EN, BL_EN enable        
        err = lm3632_smbus_write_byte(new_client, 0x0A, &data1);
        up(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        CPD_LOG("lm3632_late_resume  [%d]",data1);
        mt_set_gpio_out(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
}
static struct early_suspend lm3632_early_suspend_desc = {
        .level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
        .suspend	= lm3632_early_suspend,
        .resume		= lm3632_late_resume,
};
#endif
static struct platform_driver lm3632_backlight_driver = {
        .probe      = lm3632_pd_probe,
        .remove     = lm3632_pd_remove,
        .driver     = {
                .name  = BACKLIHGT_NAME,
                .owner = THIS_MODULE,
        }
};      
static int __init lm3632_init(void)
{
        CPD_FUN();
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
			if(lge_get_board_revno() < HW_REV_C){
				CPD_LOG("do not lm3632 init\n");
			}
			else{
		        sema_init(&lm3632_lock, 1);
		        mt_set_gpio_mode(GPIO_LCD_BL_EN, GPIO_LCD_BL_EN_M_GPIO);
		        mt_set_gpio_pull_enable(GPIO_LCD_BL_EN, GPIO_PULL_ENABLE);
		        mt_set_gpio_dir(GPIO_LCD_BL_EN, GPIO_DIR_OUT);
		        i2c_register_board_info(3, &i2c_lm3632, 1); 
		        CPD_LOG("lm3632_i2c_master_addr = 3\n");
		        if (i2c_add_driver(&lm3632_i2c_driver) != 0)
		            CPD_ERR("Failed to register rt4832 driver");
			}
#else
		sema_init(&lm3632_lock, 1);
		
		mt_set_gpio_mode(GPIO_LCD_BL_EN, GPIO_LCD_BL_EN_M_GPIO);
		mt_set_gpio_pull_enable(GPIO_LCD_BL_EN, GPIO_PULL_ENABLE);
		mt_set_gpio_dir(GPIO_LCD_BL_EN, GPIO_DIR_OUT);
		
		i2c_register_board_info(3, &i2c_lm3632, 1); 
		CPD_LOG("lm3632_i2c_master_addr = 3\n");
		
		//#ifndef	CONFIG_MTK_LEDS
		//register_early_suspend(&lm3632_early_suspend_desc);
		//#endif	
		/*
		if(platform_driver_register(&lm3632_backlight_driver))
		{
			CPD_ERR("failed to register driver");
			return -1;
		}*/
		if (i2c_add_driver(&lm3632_i2c_driver) != 0)
			CPD_ERR("Failed to register rt4832 driver");
#endif
        return 0;
}
static void __exit lm3632_exit(void)
{
        platform_driver_unregister(&lm3632_backlight_driver);
}
void lm3632_flash_strobe_prepare(char OnOff,char ActiveHigh)
{
        int err = 0;
		int ret = 0;
        ret = down_interruptible(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        err = lm3632_smbus_read_byte(new_client, 0x09, &lm3632_strobe_ctrl);        
        // flash_en register(0x0A) setting position change.
        lm3632_strobe_ctrl &= 0xF3;
        lm3632_flash_ctrl = OnOff;
        if(ActiveHigh)
        {
            lm3632_strobe_ctrl |= 0x20;
        }
        else
        {
            lm3632_strobe_ctrl &= 0xDF;
        }
        if(OnOff == 1)
        {
            CPD_LOG("Strobe mode On\n");
            lm3632_strobe_ctrl |= 0x10;		
        }
        else if(OnOff == 2)
        {
            CPD_LOG("Torch mode On\n");
            lm3632_strobe_ctrl |= 0x10;
        }
        else
        {
            CPD_LOG("Flash Off\n");
            lm3632_strobe_ctrl &= 0xEF;
        }
        // flash_en register(0x0A) setting position change.
        err = lm3632_smbus_write_byte(new_client, 0x09, &lm3632_strobe_ctrl);
        up(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
}
//strobe enable
void lm3632_flash_strobe_en(void)
{
        // flash_en register(0x0A) setting position change.
        int err = 0;
        int flash_OnOff=0;
		int ret = 0;
        ret = down_interruptible(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        err = lm3632_smbus_read_byte(new_client, 0x0A, (unsigned char *)&flash_OnOff);
#if 0
        if(lm3632_flash_ctrl == 1)
            flash_OnOff |= 0x66;
        else if(lm3632_flash_ctrl == 2)
            flash_OnOff |= 0x62;
        else
            flash_OnOff &= 0x99;
#endif
            
	if(lm3632_flash_ctrl == 1)
		flash_OnOff |= 0x06;
	else if(lm3632_flash_ctrl == 2)
		flash_OnOff |= 0x02;
	else
		flash_OnOff &= 0xF9;
        err = lm3632_smbus_write_byte(new_client, 0x0A, (unsigned char *)&flash_OnOff);
        up(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        // flash_en register(0x0A) setting position change.
}

void lm3632_flash_torch_level(int level)
{
    unsigned char torch_level = 0;
    unsigned char rdata = 0;
    unsigned char wdata = 0;
    int err = 0;
    err = down_interruptible(&lm3632_lock);
    err = lm3632_smbus_read_byte(new_client, 0x06, &rdata);
    CPD_LOG("lm3632_flash_torch_level read =0x%x\n", rdata);

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
    err = lm3632_smbus_write_byte(new_client, 0x06, &wdata);
    CPD_LOG("lm3632_flash_torch_level write =0x%x\n", wdata);
    up(&lm3632_lock);
}
//strobe level
void lm3632_flash_strobe_level(char level)
{
        int err = 0;
        unsigned char data1=0;
        unsigned char data2=0;
        unsigned char torch_level;
        unsigned char strobe_timeout = 0x1F;
		int ret = 0;
        ret = down_interruptible(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
        #if 0 // Add Main Flash current(4 Step)
        if( level == 1) 
        {
            torch_level = 0x20;
        }
        else
        {
            torch_level = 0x50;
        }
        err = lm3632_smbus_read_byte(new_client, 0x06, &data1);
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
        err = lm3632_smbus_read_byte(new_client, 0x06, &data1);
        strobe_timeout = 0x1F; // Flash Timing Tuning
        if(level < 0)
            data1= torch_level;
        else if(level == 1)
            data1= torch_level | 0x03;
        else if(level == 2)
            data1= torch_level | 0x05;
        else if(level == 3)
            data1= torch_level | 0x08;
        else if(level == 4)
            data1= torch_level | 0x0A;
        else
            data1= torch_level | level;
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
        err = lm3632_smbus_write_byte(new_client, 0x06, &data1);
        data2 = 0x40 | strobe_timeout;
        CPD_LOG("Storbe Timeout =0x%x\n", data2);
        err |= lm3632_smbus_write_byte(new_client, 0x07, &data2);
        up(&lm3632_lock); // Add semaphore for lcd and flash i2c communication.
}
EXPORT_SYMBOL(lm3632_flash_strobe_en);
EXPORT_SYMBOL(lm3632_flash_strobe_prepare);
EXPORT_SYMBOL(lm3632_flash_strobe_level);
MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("lm3632 driver");
MODULE_LICENSE("GPL");
module_init(lm3632_init);
module_exit(lm3632_exit);