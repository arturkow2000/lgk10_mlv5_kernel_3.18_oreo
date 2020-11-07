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

#define DW8767_DSV_VPOS_EN  GPIO_DSV_AVEE_EN
#define DW8767_DSV_VPOS_EN_MODE GPIO_DSV_AVEE_EN_M_GPIO

#define DW8767_DSV_VNEG_EN GPIO_DSV_AVDD_EN
#define DW8767_DSV_VNEG_EN_MODE GPIO_DSV_AVDD_EN_M_GPIO

#define MIN_VALUE_SETTINGS 10 /* value leds_brightness_set*/
#define MAX_VALUE_SETTINGS 255 /* value leds_brightness_set*/
#define MIN_MAX_SCALE(x) (((x)<MIN_VALUE_SETTINGS) ? MIN_VALUE_SETTINGS : (((x)>MAX_VALUE_SETTINGS) ? MAX_VALUE_SETTINGS:(x)))

#define BACKLIHGT_NAME "charge-pump"
#define DW8767_DEV_NAME "dw8767"

#define CPD_TAG                  "[chargepump] "
#define CPD_FUN(f)               printk(CPD_TAG"%s\n", __FUNCTION__)
#define CPD_ERR(fmt, args...)    printk(CPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define CPD_LOG(fmt, args...)    printk(CPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

#ifndef GPIO_LCD_BL_EN
#define GPIO_LCD_BL_EN         (GPIO100 | 0x80000000)
#define GPIO_LCD_BL_EN_M_GPIO   GPIO_MODE_00
#endif

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id dw8767_i2c_id[] = {{DW8767_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_dw8767={ I2C_BOARD_INFO(DW8767_DEV_NAME, 0x11)};

#define LED_MODE_CUST_LCM 4
#define LED_MODE_CUST_BLS_PWM 5
static int led_mode = LED_MODE_CUST_BLS_PWM;
int old_bl_level;

#if defined(CONFIG_LEDS_OPR_MODE_RECOVERY)
#define I2C_SET (0x00)
#define PWM_SET (0x01 << 6)
#define PWM_CLEAR (~(0x01 << 6))
static char pwm_en;
#endif

// Flash control
unsigned char strobe_ctrl;
unsigned char flash_ctrl=0; // flash_en register(0x0A) setting position change.
unsigned char mapping_level;

#if defined(CONFIG_SW49106_FHDPLUS_DSI_VDO_LGD_CV5)
unsigned int bright_arr[] = {
	7,7,7,7,7,7,7,7,7,8,7,7,8,9,9,10,11,12,12,13,													// 19
	14,14,15,16,17,17,18,19,19,20,21,22,22,23,24,25,25,26,27,27,									// 39
	28,29,30,30,31,32,32,33,34,35,35,36,37,37,38,39,40,40,41,42,									// 59
	43,45,47,50,52,54,57,59,62,64,66,69,71,74,76,78,81,83,86,88,									// 79
	90,93,95,98,100,102,105,107,110,112,114,117,119,122,124,126,129,131,134,136, 				    // 99
	138,141,143,146,148,150,153,155,158,162,166,170,174,178,182,187,191,195,199,203, 			    // 119
	207,212,216,220,224,228,232,237,241,245,249,253,257,262,266,270,274,278,282,287, 			    // 139
	291,295,299,303,307,312,316,320,324,328,332,337,341,345,349,353,357,362,368,375, 			    // 159
	382,389,396,403,410,417,424,431,438,445,452,459,466,473,479,486,493,500,507,514,				// 179
	521,528,535,542,549,556,563,570,577,584,590,597,604,611,618,625,632,639,646,653, 			    // 199
	660,667,674,681,688,695,702,712,722,732,742,752,762,773,783,793,803,813,823,833,				// 219
	844,854,864,874,884,894,904,915,925,935,945,955,965,975,986,996,1006,1016,1026,1036,			// 239
	1046,1057,1067,1077,1087,1097,1107,1117,1128,1138,1148,1158,1168,1178,1188,1700					// 255
};
#elif defined(CONFIG_TOVIS_INCELL_TD4100_HD_LV7) || defined(CONFIG_TOVIS_INCELL_LG4894_HD_CV3_SPR)
unsigned int bright_arr[] = {
        7,7,7,7,7,7,7,7,7,7,7,7,8,9,10,11,                                                      // 0-15
        12,13,13,14,15,16,17,18,19,19,20,21,22,23,24,25,                                        // 16-31
        25,26,27,28,29,30,31,31,32,33,34,35,36,37,37,38,                                        // 32-47
        39,40,41,42,43,43,44,45,46,47,48,49,50,53,56,59,                                        // 48-63
        63,66,69,72,76,79,82,85,89,92,95,99,102,105,108,112,                                    // 64-79
        115,118,121,125,128,131,135,138,141,144,148,151,154,157,161,164,                        // 80-95
        167,171,174,177,180,184,187,190,193,197,200,203,207,213,219,225,                        // 96-111
        231,237,243,249,255,262,268,274,280,286,292,298,304,311,317,323,                        // 112-127
        329,335,341,347,353,360,366,372,378,384,390,396,402,409,415,421,                        // 128-143
        427,433,439,445,451,458,464,470,476,482,488,494,500,507,516,526,                        // 144-159
        535,545,554,564,573,583,592,602,612,621,631,640,650,659,669,678,                        // 160-175
        688,698,707,717,726,736,745,755,764,774,783,793,803,812,822,831,                        //176-191
        841,850,860,869,879,889,898,908,917,927,936,946,955,965,975,988,                        // 192-207
        1002,1016,1029,1043,1057,1070,1084,1098,1111,1125,1139,1152,1166,1180,1193,1207,        // 208-223
        1221,1234,1248,1262,1275,1289,1303,1316,1330,1344,1357,1371,1385,1398,1412,1426,        // 224-239
        1439,1453,1467,1480,1494,1508,1521,1535,1549,1562,1576,1590,1603,1617,1631,1645         // 240-255
};
#else
unsigned char bright_arr[] = {  // array index max 100, value under 255
	20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 22, 22, 22, 22, 24, 24, 24,  // 19
	26, 26, 26, 28, 28, 30, 30, 32, 32, 34, 34, 36, 38, 38, 40, 42, 42, 44, 36, 48,  // 39
	48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 76, 78, 80, 82, 86, 88, 90,  // 59
	94, 96, 98, 102, 104, 108, 110, 114, 118, 120, 124, 128, 130, 134, 138, 142, 146, 148, 152, 156,  //79
	160, 164, 168, 172, 178, 182, 186, 190, 194, 200, 204, 208, 212, 218, 222, 228, 232, 238, 242, 248, 255  // 100
};
#endif

//static unsigned char current_brightness = 0;
static unsigned char is_suspend = 0;

struct semaphore dw8767_lock; // Add semaphore for lcd and flash i2c communication.

/* generic */
#define DW8767_MAX_RETRY_I2C_XFER (100)
#define DW8767_I2C_WRITE_DELAY_TIME 1

/* i2c read routine for API*/
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

		return 0;
#endif
}

/* i2c write routine for */
static char dw8767_i2c_write(struct i2c_client *client, u8 reg_addr, u8 *data, u8 len)
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
	//CPD_LOG("dw8767_i2c_write \n");
	return 0;
}

static int dw8767_smbus_read_byte(struct i2c_client *client,
            unsigned char reg_addr, unsigned char *data)
{
	return dw8767_i2c_read(client,reg_addr,data,1);
}

static int dw8767_smbus_write_byte(struct i2c_client *client,
            unsigned char reg_addr, unsigned char *data)
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
			CPD_ERR("[dw8767] : dw8767_smbus_write_byte fail: %d\n",ret_val);
			return ret_val;
		}
	}
	return ret_val;
}

#if 0
bool check_charger_pump_vendor()
{
	int err = 0;
	unsigned char data = 0;

	err = dw8767_smbus_read_byte(new_client,0x01,&data);

	if(err < 0)
		CPD_ERR("%s read charge-pump vendor id fail\n", __func__);

	CPD_ERR("%s vendor is 0x%x\n", __func__, data&0x03);

	if((data&0x03) == 0x03) //Richtek
		return FALSE;
	else
		return TRUE;
}
#endif
//int current_level;

int chargepump_set_backlight_level(unsigned int level)
{
	int ret;
	unsigned int data = 0;
	unsigned char data1 = 0;
#if defined(CONFIG_LEDS_OPR_MODE_RECOVERY)
	unsigned char reg_status = 0;
#endif
	//unsigned char data = 0;
	//unsigned int bright_per = 0;

	unsigned char lsb_data = 0x00; // 3bit
	unsigned char msb_data = 0x00; // 8bit

	old_bl_level = level;

	CPD_LOG("chargepump_set_backlight_level  [%d]\n",level);

	if (level == 0){
		if(is_suspend == false){
			CPD_LOG( "backlight off\n");
			ret = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
			data1 = 0x00; //backlight2 brightness 0
			dw8767_smbus_write_byte(new_client, 0x04, &data1);  // LSB 3bit all 0
			dw8767_smbus_write_byte(new_client, 0x05, &data1);  // MSB 3bit all 0
			dw8767_smbus_read_byte(new_client, 0x0A, &data1);
			// data1 &= 0x16;  // BLED1_EN, FLASH_MODE, FLASH Enable, AND operation
			data1 &= 0xE6;  // BLED1_EN 0, BL_EN disable
			dw8767_smbus_write_byte(new_client, 0x0A, &data1);

			is_suspend = true;// Move backlight suspend setting position into semaphore
			up(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
		}
	}else{
		level = MIN_MAX_SCALE(level);
		data = bright_arr[level];
		CPD_LOG("%s data = %d\n", __func__, data);
		//bright_per = (level - (unsigned int)10) *(unsigned int)100 / (unsigned int)245; // 10 ~255
		//current_level = bright_per;
		//CPD_LOG("%s bright_per = %d, data = %d\n", __func__, bright_per, data);

		mapping_level = data;
		if (is_suspend == true){
			is_suspend = false;
			//printk( "------    backlight_level resume-----\n");
			ret = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.

			data1 = 0x70;// OVP 29V, Linear mapping mode
			dw8767_smbus_write_byte(new_client, 0x02, &data1);

#if defined(CONFIG_TOVIS_INCELL_LG4894_HD_CV3_SPR)
			data1 = 0x04;//500kHz
#else
			data1 = 0x84;//4Mhz
#endif
			dw8767_smbus_write_byte(new_client, 0x03, &data1);

			lsb_data = (data) & 0x07; // 3bit LSB
			msb_data = (data >> 3) & 0xFF; // 8bit MSB

			dw8767_smbus_write_byte(new_client, 0x04, &lsb_data); // LSB
			dw8767_smbus_write_byte(new_client, 0x05, &msb_data); // MSB

			CPD_LOG("[DW8767]-backlight brightness Setting[reg0x04][MSB:0x%x]\n",lsb_data);
			CPD_LOG("[DW8767]-backlight brightness Setting[reg0x05][LSB:0x%x]\n",msb_data);

#if defined(CONFIG_LEDS_OPR_MODE_RECOVERY)
			dw8767_smbus_read_byte(new_client, 0x09, &data1);

			reg_status = data1 & PWM_SET;

			if(pwm_en != reg_status) /*abnormal status*/
			{
				if(led_mode == LED_MODE_CUST_BLS_PWM)
					data1 |= PWM_SET;
				else
					data1 &= PWM_CLEAR;
				dw8767_smbus_write_byte(new_client, 0x09, &data1);
				CPD_LOG("[DW8767]-PWM_EN bit recovery. [reg0x09][value:0x%x]\n", data1);
			}
#endif
			dw8767_smbus_read_byte(new_client, 0x0A, &data1);
			data1 |= 0x09;  // BLED12_EN, BL_EN enable
			dw8767_smbus_write_byte(new_client, 0x0A, &data1);

			// Move backlight suspend setting position into semaphore
			up(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
		}

		if (level != 0) // Move backlight suspend setting position into semaphore
		{
			ret = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
			if(0)  // blocking useless code
			{
				unsigned char read_data = 0;
				dw8767_smbus_read_byte(new_client, 0x02, &read_data);
				CPD_LOG("[DW8767]-OVP[0x%x]\n",read_data);
			}

			lsb_data = (data) & 0x07; // 3bit LSB
			msb_data = (data >> 3) & 0xFF; // 8bit MSB

			dw8767_smbus_write_byte(new_client, 0x04, &lsb_data); // LSB
			dw8767_smbus_write_byte(new_client, 0x05, &msb_data); // MSB

			CPD_LOG("[DW8767]-backlight brightness Setting[reg0x04][MSB:0x%x]\n",lsb_data);
			CPD_LOG("[DW8767]-backlight brightness Setting[reg0x05][LSB:0x%x]\n",msb_data);

			up(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
		} // Move backlight suspend setting position into semaphore
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

int chargepump_backlight_level_test(unsigned int level)
{
	unsigned char data = 0;

	if(level > 255)
			level = 255;

	data = 0x70;// OVP 29V, Linear mapping mode
	dw8767_smbus_write_byte(new_client, 0x02, &data);

	data = 0x07;  // Backlight brightness LSB 3bits, 0b111
	dw8767_smbus_write_byte(new_client, 0x04, &data);

	data = level;
	dw8767_smbus_write_byte(new_client, 0x05, &data);  // MSB 8 bit control
	CPD_LOG("backlight brightness Test[reg0x05][value:0x%x]\n",data);

	dw8767_smbus_read_byte(new_client, 0x0A, &data);
	data |= 0x19;  // BLED12_EN, BL_EN enable
	dw8767_smbus_write_byte(new_client, 0x0A, &data);

	return 0;
}

unsigned char get_backlight_level(void)
{
        return mapping_level;
}

void dw8767_set_led_mode(int mode){
    led_mode = mode;
#if defined(CONFIG_LEDS_OPR_MODE_RECOVERY)
    pwm_en = (mode == LED_MODE_CUST_BLS_PWM ? PWM_SET : I2C_SET);
#endif
    CPD_LOG("[dw8767] LED mode (%d) LED_MODE_CUST_BLS_PWM(5), LED_MODE_CUST_LCM(4)\n", led_mode);
}

void chargepump_DSV_VPOS_on(void){
    unsigned char data = 0;
    data=0x18;
    dw8767_smbus_write_byte(new_client, 0x0D, &data); // DSV Vboost voltage 5.7V
    data=0x1C;
    dw8767_smbus_write_byte(new_client, 0x0C, &data);// VPOS_EN=1
#if defined(CONFIG_TOVIS_INCELL_LG4894_HD_CV3_SPR)
	data=0x1E;
#else
    data=0x16;
#endif
    dw8767_smbus_write_byte(new_client, 0x0E, &data);// Vpos voltage setting +5.1v
}

void chargepump_DSV_VNEG_on(void){
    unsigned char data = 0;
    data=0x18;
    dw8767_smbus_write_byte(new_client, 0x0D, &data); // DSV Vboost voltage 5.7V
    data=0x1E;
    dw8767_smbus_write_byte(new_client, 0x0C, &data);// VNEG_EN=1
#if defined(CONFIG_TOVIS_INCELL_LG4894_HD_CV3_SPR)
    data=0x1E;
#else
	data=0x16;
#endif
    dw8767_smbus_write_byte(new_client, 0x0F, &data);// Vneg voltage setting as -5.1v
}

void chargepump_DSV_EXT_mode_on(void){
    unsigned char data = 0;
    data=0x87;
    dw8767_smbus_write_byte(new_client, 0x0C, &data); // DSV Toggle mode
}

void chargepump_DSV_I2C_mode_on(void){
    unsigned char data = 0;
    data=0x06;
    dw8767_smbus_write_byte(new_client, 0x0C, &data);
    mdelay(8);
    data=0x1E;
    dw8767_smbus_write_byte(new_client, 0x0C, &data);

}

void chargepump_DSV_fast_discharge(void)
{
    unsigned char data = 0;
    data=0x18;
    dw8767_smbus_write_byte(new_client, 0x0C, &data);
}

void chargepump_DSV_floating_mode(void)
{
    unsigned char data = 0;
    data=0x00;
    dw8767_smbus_write_byte(new_client, 0x0C, &data);
}

void chargepump_DSV_on(void)
{
	unsigned char data = 0;

	data=0x70;
	dw8767_smbus_write_byte(new_client, 0x02, &data);// OVP 29V, linear mode, TI recommand
	data=0x18;
	dw8767_smbus_write_byte(new_client, 0x0D, &data); // DSV Vboost voltage 5.7V
	data=0x1E;
	dw8767_smbus_write_byte(new_client, 0x0C, &data);// VPOS_EN=1, VNEG_EN=1, EXT_EN=0(I2C Control)
	data=0x1E;
	dw8767_smbus_write_byte(new_client, 0x0E, &data);// Vpos voltage setting as +5.5v
	data=0x1E;
	dw8767_smbus_write_byte(new_client, 0x0F, &data);// Vneg voltage setting as -5.5v

	if( led_mode == LED_MODE_CUST_BLS_PWM){
		/* PWM enable */
		data = 0x40;
		dw8767_smbus_write_byte(new_client, 0x09, &data);
	}else if ( led_mode == LED_MODE_CUST_LCM) {
		data = 0x00;
		dw8767_smbus_write_byte(new_client, 0x09, &data);
	}

	CPD_LOG("[dw8767]chargepump DSV on\n");
}

void chargepump_DSV_off(void)
{
   unsigned char data = 0;

   data=0x18;
   dw8767_smbus_write_byte(new_client, 0x0C, &data);// VPOS_EN=0
#if defined(CONFIG_TOVIS_INCELL_LG4894_HD_CV3_SPR)
   mdelay(3);
#else
   mdelay(10);
#endif
   data=0x00;
   dw8767_smbus_write_byte(new_client, 0x0C, &data);// VPOS_EN=0

   CPD_LOG("[dw8767]chargepump DSV off\n");
}

void dw8767_dsv_ctrl(int enable)
{
    int ret = 0;
	ret = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
	if(enable==1)
		chargepump_DSV_on();
	else
		chargepump_DSV_off();
	up(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
	CPD_LOG("[dw8767]dw8767_dsv_ctrl %s\n",(enable)? "enable":"disable");
}

static unsigned int get_backlight_rawdata(void)
{
	unsigned char dw8767_msb = 0;
	unsigned char dw8767_lsb = 0;
	unsigned int dw8767_level = 0;

	dw8767_smbus_read_byte(new_client, 0x04, &dw8767_lsb);
	dw8767_smbus_read_byte(new_client, 0x05, &dw8767_msb);

	dw8767_level |= ((dw8767_msb & 0xFF) << 3); // 8bit MSB
	dw8767_level |= ((dw8767_lsb & 0x07)); // 3bit LSB8

	return dw8767_level;
}

static void set_backlight_rawdata(unsigned int level)
{
	unsigned char dw8767_msb = 0;
	unsigned char dw8767_lsb = 0;

	dw8767_lsb = (level) & 0x07; // 3bit LSB
	dw8767_msb = (level >> 3) & 0xFF; // 8bit MSB

	dw8767_smbus_write_byte(new_client, 0x04, &dw8767_lsb);
	dw8767_smbus_write_byte(new_client, 0x05, &dw8767_msb);
}

static ssize_t lcd_backlight_show_blmap(struct device *dev,
                struct device_attribute *attr, char *buf)
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

static ssize_t lcd_backlight_store_blmap(struct device *dev,
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
		bright_arr[j] = (unsigned int)value;

		while (isdigit(buf[i]))
			++i;
		++j;
	}

	return count;
}

static ssize_t show_rawdata(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "dw8767 brightness code : %d\n",get_backlight_rawdata());
}

static ssize_t store_rawdata(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
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

static ssize_t sysfs_show_dsv(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	int n = 0;
	int err = 0;
	unsigned char flash_OnOff=0;

	err = dw8767_smbus_read_byte(new_client, 0x01, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x01 %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x02, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x02 %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x03, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x03 %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x04, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x04 %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x05, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x05 %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x06, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x06 %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x07, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x07 %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x08, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x08 %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x09, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x09 %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x0A, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x0A %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x0B, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x0B %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x0C, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x0C %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x0D, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x0D %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x0E, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x0E %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x0F, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x0F %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x0E, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x0E %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x10, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x10 %d \n",flash_OnOff);

	err = dw8767_smbus_read_byte(new_client, 0x11, &flash_OnOff);
	n +=  sprintf(buf+n, "FLASH 0x11 %d \n",flash_OnOff);
	return n;
}

static ssize_t sysfs_store_dsv(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	int addr;
	int err = 0;
	sscanf(buf, "%d %d",&addr, &value);

	err = dw8767_smbus_write_byte(new_client, addr, (unsigned char*)&value);
	return count;
}

DEVICE_ATTR(dw8767_reg, 0644, sysfs_show_dsv, sysfs_store_dsv);
DEVICE_ATTR(brightness_code, 0644, show_rawdata, store_rawdata);
DEVICE_ATTR(bl_blmap, 0644, lcd_backlight_show_blmap, lcd_backlight_store_blmap);


static int dw8767_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	CPD_FUN();

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);

	device_create_file(&client->dev, &dev_attr_bl_blmap);
	device_create_file(&client->dev, &dev_attr_brightness_code);
	device_create_file(&client->dev, &dev_attr_dw8767_reg);

	memset(new_client, 0, sizeof(struct i2c_client));

	new_client = client;
	return 0;
}

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
	// .owner  = THIS_MODULE,
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
	} else{
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

	err = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
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

	mdelay(50);
	err = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.

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
	.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend	= dw8767_early_suspend,
	.resume		= dw8767_late_resume,
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

	mt_set_gpio_mode(GPIO_LCD_BL_EN, GPIO_LCD_BL_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCD_BL_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCD_BL_EN, GPIO_DIR_OUT);

#if defined(CONFIG_TOVIS_INCELL_LG4894_HD_CV3_SPR)
	i2c_register_board_info(1, &i2c_dw8767, 1);
#else
	i2c_register_board_info(4, &i2c_dw8767, 1);
#endif

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

	err = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.

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

	err = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
	err = dw8767_smbus_read_byte(new_client, 0x0A, &flash_OnOff);

#if 0
	if(flash_ctrl == 1)
		flash_OnOff |= 0x66;
	else if(flash_ctrl == 2)
		flash_OnOff |= 0x62;
	else
		flash_OnOff &= 0x99;
#endif

    CPD_LOG("Flash %u\n", flash_OnOff);

if(flash_ctrl == 1)
	flash_OnOff |= 0x06;
else if(flash_ctrl == 2)
	flash_OnOff |= 0x02;
else
	flash_OnOff &= 0xF9;

    flash_OnOff |= 0x09; // BLED12_EN, BL_EN enable

    CPD_LOG("flash_OnOff %u \n", flash_OnOff);
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

	err = down_interruptible(&dw8767_lock); // Add semaphore for lcd and flash i2c communication.
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
