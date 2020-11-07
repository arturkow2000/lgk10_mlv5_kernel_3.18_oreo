/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/delay.h>
#ifdef CONFIG_MTK_AAL_SUPPORT
#include <ddp_aal.h>
#endif
#include <ddp_gamma.h>

#include <mt-plat/mt_pwm.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_hw.h>

#include "leds_sw.h"
#include "leds_hal.h"
#include "ddp_pwm.h"
#include "mtkfb.h"

#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
#include <soc/mediatek/lge/board_lge.h>
#endif
#ifdef CONFIG_LGE_LCD_OFF_DIMMING
#include <soc/mediatek/lge/board_lge.h>
#include <soc/mediatek/lge/lge_handle_panic.h>
extern bool fb_blank_called;
#endif

#ifdef CONFIG_MTK_AAL_SUPPORT
extern struct semaphore qpps_lock;
extern int down_interruptible(struct semaphore *sem);
extern void up(struct semaphore *sem);
#endif

/* for LED&Backlight bringup, define the dummy API */
#ifndef CONFIG_MTK_PMIC
u16 pmic_set_register_value(u32 flagname, u32 val)
{
	return 0;
}
#endif

int __weak mtkfb_set_backlight_level(unsigned int level)
{
	return 0;
}

#ifndef CONFIG_MTK_PWM
s32 pwm_set_spec_config(struct pwm_spec_config *conf)
{
	return 0;
}

void mt_pwm_disable(u32 pwm_no, u8 pmic_pad)
{
}
#endif

static DEFINE_MUTEX(leds_mutex);
static DEFINE_MUTEX(leds_pmic_mutex);

/****************************************************************************
 * variables
 ***************************************************************************/
/* struct cust_mt65xx_led* bl_setting_hal = NULL; */
static unsigned int bl_brightness_hal = 102;
static unsigned int bl_duty_hal = 21;
static unsigned int bl_div_hal = CLK_DIV1;
static unsigned int bl_frequency_hal = 32000;
/* for button led don't do ISINK disable first time */
static int button_flag_isink0;
static int button_flag_isink1;
static int button_flag_isink2;
static int button_flag_isink3;

struct wake_lock leds_suspend_lock;

char *leds_name[MT65XX_LED_TYPE_TOTAL] = {
	"red",
	"green",
	"blue",
	"jogball-backlight",
	"keyboard-backlight",
	"button-backlight",
	"lcd-backlight",
};

struct cust_mt65xx_led *pled_dtsi = NULL;
/****************************************************************************
 * DEBUG MACROS
 ***************************************************************************/
static int debug_enable_led_hal = 1;
#define LEDS_DEBUG(format, args...) do { \
	if (debug_enable_led_hal) {	\
		pr_debug("[LED]"format, ##args);\
	} \
} while (0)

/*****************PWM *************************************************/
#define PWM_DIV_NUM 8
static int time_array_hal[PWM_DIV_NUM] = {
	256, 512, 1024, 2048, 4096, 8192, 16384, 32768 };
static unsigned int div_array_hal[PWM_DIV_NUM] = {
	1, 2, 4, 8, 16, 32, 64, 128 };

static unsigned int backlight_PWM_div_hal = CLK_DIV1;	/* this para come from cust_leds. */

/****************************************************************************
 * func:return global variables
 ***************************************************************************/
static unsigned long long current_time, last_time;
static int count;
static char buffer[4096] = "[BL] Set Backlight directly ";

#if defined(CONFIG_TOVIS_INCELL_LG4894_HD_CV3_SPR)
unsigned int bright_arr_pwm[]={
	0, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13,                     // 19
	13, 14, 14, 15, 15, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,                    // 39
	17, 17, 17, 20, 22, 24, 26, 27, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,                    // 59
	32, 32, 32, 49, 49, 49, 49, 49, 49, 49, 49, 49, 55, 59, 64, 68, 72, 76, 78, 79,                    // 79
	79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 100, 100, 100, 100, 100, 100, 100, 104, 106, 110,                    // 99
	114, 118, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 125, 135, 145, 155, 165, 175, 185, 188,           // 119
	191, 194, 197, 200, 203, 206, 209, 209, 209, 209, 209, 209, 209, 209, 213, 217, 221, 225, 229, 235,           // 139
	240, 245, 250, 255, 260, 265, 270, 275, 280, 285, 290, 295, 295, 295, 295, 295, 295, 295, 295, 295,           // 159
	295, 295, 304, 304, 304, 304, 306, 314, 322, 330, 338, 346, 354, 362, 370, 378, 386, 394, 402, 410,           // 179
	418, 426, 434, 442, 450, 458, 462, 465, 465, 465, 470, 480, 484, 500, 510, 520, 530, 540, 550, 555,           // 199
	560, 565, 565, 565, 565, 565, 565, 565, 565, 565, 565, 573, 582, 591, 600, 609, 618, 627, 636, 645,           // 219
	654, 663, 672, 681, 690, 699, 708, 717, 726, 735, 744, 753, 762, 771, 780, 789, 798, 807, 816, 825,           // 239
	834, 843, 852, 861, 870, 879, 888, 897, 906, 915, 924, 933, 942, 951, 955, 960                             // 255
};
#endif

static void backlight_debug_log(int level, int mappingLevel)
{
	unsigned long long temp_time, rem;

	current_time = sched_clock();
	temp_time = current_time;
	rem = do_div(temp_time, 1000000000);
	do_div(rem, 1000000);

	sprintf(buffer + strlen(buffer), "T:%lld.%lld,L:%d map:%d    ",
		temp_time, rem, level, mappingLevel);

	count++;

	if (level == 0 || count == 5 || (current_time - last_time) > 1000000000) {
		LEDS_DEBUG("%s", buffer);
		count = 0;
		buffer[strlen("[BL] Set Backlight directly ")] = '\0';
	}

	last_time = sched_clock();
}

void mt_leds_wake_lock_init(void)
{
	wake_lock_init(&leds_suspend_lock, WAKE_LOCK_SUSPEND, "leds wakelock");
}

unsigned int mt_get_bl_brightness(void)
{
	return bl_brightness_hal;
}

unsigned int mt_get_bl_duty(void)
{
	return bl_duty_hal;
}

unsigned int mt_get_bl_div(void)
{
	return bl_div_hal;
}

unsigned int mt_get_bl_frequency(void)
{
	return bl_frequency_hal;
}

unsigned int *mt_get_div_array(void)
{
	return &div_array_hal[0];
}

void mt_set_bl_duty(unsigned int level)
{
	bl_duty_hal = level;
}

void mt_set_bl_div(unsigned int div)
{
	bl_div_hal = div;
}

void mt_set_bl_frequency(unsigned int freq)
{
	bl_frequency_hal = freq;
}

struct cust_mt65xx_led *get_cust_led_dtsi(void)
{
	struct device_node *led_node = NULL;
	bool isSupportDTS = false;
	int i, ret;
	int mode, data;
	int pwm_config[5] = { 0 };

	/* LEDS_DEBUG("get_cust_led_dtsi: get the leds info from device tree\n"); */
	if (pled_dtsi == NULL) {
		/* this can allocat an new struct array */
		pled_dtsi = kmalloc(MT65XX_LED_TYPE_TOTAL *
						      sizeof(struct
							     cust_mt65xx_led),
						      GFP_KERNEL);
		if (pled_dtsi == NULL) {
			LEDS_DEBUG("get_cust_led_dtsi kmalloc fail\n");
			goto out;
		}

		for (i = 0; i < MT65XX_LED_TYPE_TOTAL; i++) {

			char node_name[32] = "mediatek,";
			if (strlen(node_name) + strlen(leds_name[i]) + 1 > sizeof(node_name)) {
				LEDS_DEBUG("buffer for %s%s not enough\n", node_name, leds_name[i]);
				pled_dtsi[i].mode = 0;
				pled_dtsi[i].data = -1;
				continue;
			}

			pled_dtsi[i].name = leds_name[i];

			led_node =
			    of_find_compatible_node(NULL, NULL,
						    strncat(node_name,
							   leds_name[i], sizeof(node_name) - strlen(node_name) - 1));
			if (!led_node) {
				LEDS_DEBUG("Cannot find LED node from dts\n");
				pled_dtsi[i].mode = 0;
				pled_dtsi[i].data = -1;
			} else {
				isSupportDTS = true;
				ret =
				    of_property_read_u32(led_node, "led_mode",
							 &mode);
				if (!ret) {
					pled_dtsi[i].mode = mode;
					LEDS_DEBUG
					    ("The %s's led mode is : %d\n",
					     pled_dtsi[i].name,
					     pled_dtsi[i].mode);
				} else {
					LEDS_DEBUG
					    ("led dts can not get led mode");
					pled_dtsi[i].mode = 0;
				}
#if defined(CONFIG_LEDS_DW8767) || defined(CONFIG_LEDS_DW8767_CV7A) || defined(CONFIG_LEDS_DW8767_CV1)
				dw8767_set_led_mode(mode);
#endif
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
				lm3632_set_led_mode(mode);
#endif
				ret =
				    of_property_read_u32(led_node, "data",
							 &data);
				if (!ret) {
					pled_dtsi[i].data = data;
					LEDS_DEBUG
					    ("The %s's led data is : %ld\n",
					     pled_dtsi[i].name,
					     pled_dtsi[i].data);
				} else {
					LEDS_DEBUG
					    ("led dts can not get led data");
					pled_dtsi[i].data = -1;
				}

				ret =
				    of_property_read_u32_array(led_node,
							       "pwm_config",
							       pwm_config,
							       ARRAY_SIZE
							       (pwm_config));
				if (!ret) {
					LEDS_DEBUG
					    ("The %s's pwm config data is %d %d %d %d %d\n",
					     pled_dtsi[i].name, pwm_config[0],
					     pwm_config[1], pwm_config[2],
					     pwm_config[3], pwm_config[4]);
					pled_dtsi[i].config_data.clock_source =
					    pwm_config[0];
					pled_dtsi[i].config_data.div =
					    pwm_config[1];
					pled_dtsi[i].config_data.low_duration =
					    pwm_config[2];
					pled_dtsi[i].config_data.High_duration =
					    pwm_config[3];
					pled_dtsi[i].config_data.pmic_pad =
					    pwm_config[4];

				} else
					LEDS_DEBUG
					    ("led dts can not get pwm config data.\n");

				switch (pled_dtsi[i].mode) {
				case MT65XX_LED_MODE_CUST_LCM:
#ifdef CONFIG_CUSTOM_KERNEL_CHARGEPUMP
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
					if(lge_get_board_revno() < HW_REV_C)
					    pled_dtsi[i].data = (long)rt4832_chargepump_set_backlight_level;
					else
					    pled_dtsi[i].data = (long)lm3632_chargepump_set_backlight_level;
#else
					pled_dtsi[i].data = (long)chargepump_set_backlight_level;
#endif
#else
					pled_dtsi[i].data =
					    (long)mtkfb_set_backlight_level;
					LEDS_DEBUG
					    ("kernel:the backlight hw mode is LCM.\n");
#endif
					break;
				case MT65XX_LED_MODE_CUST_BLS_PWM:
					pled_dtsi[i].data =
					    (long)disp_bls_set_backlight;
					LEDS_DEBUG
					    ("kernel:the backlight hw mode is BLS.\n");
					break;
				default:
					break;
				}
			}
		}

		if (!isSupportDTS) {
			kfree(pled_dtsi);
			pled_dtsi = NULL;
		}
	}
 out:
	return pled_dtsi;
}

struct cust_mt65xx_led *mt_get_cust_led_list(void)
{
	struct cust_mt65xx_led *cust_led_list = get_cust_led_dtsi();
	return cust_led_list;
}

/****************************************************************************
 * internal functions
 ***************************************************************************/
static int brightness_mapto64(int level)
{
	if (level < 30)
		return (level >> 1) + 7;
	else if (level <= 120)
		return (level >> 2) + 14;
	else if (level <= 160)
		return level / 5 + 20;
	else
		return (level >> 3) + 33;
}

static int find_time_index(int time)
{
	int index = 0;

	while (index < 8) {
		if (time < time_array_hal[index])
			return index;
		index++;
	}
	return PWM_DIV_NUM - 1;
}

int mt_led_set_pwm(int pwm_num, struct nled_setting *led)
{
	struct pwm_spec_config pwm_setting = { .pwm_no = pwm_num};
	int time_index = 0;

	pwm_setting.pwm_no = pwm_num;
	pwm_setting.mode = PWM_MODE_OLD;

	LEDS_DEBUG("led_set_pwm: mode=%d,pwm_no=%d\n", led->nled_mode,
		   pwm_num);
	/* We won't choose 32K to be the clock src of old mode because of system performance. */
	/* The setting here will be clock src = 26MHz, CLKSEL = 26M/1625 (i.e. 16K) */
	pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;

	switch (led->nled_mode) {
	/* Actually, the setting still can not to turn off NLED. We should disable PWM to turn off NLED. */
	case NLED_OFF:
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = 0;
		pwm_setting.clk_div = CLK_DIV1;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100 / 2;
		break;

	case NLED_ON:
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = 30 / 2;
		pwm_setting.clk_div = CLK_DIV1;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100 / 2;
		break;

	case NLED_BLINK:
		LEDS_DEBUG("LED blink on time = %d offtime = %d\n",
			   led->blink_on_time, led->blink_off_time);
		time_index =
		    find_time_index(led->blink_on_time + led->blink_off_time);
		LEDS_DEBUG("LED div is %d\n", time_index);
		pwm_setting.clk_div = time_index;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH =
		    (led->blink_on_time +
		     led->blink_off_time) * MIN_FRE_OLD_PWM /
		    div_array_hal[time_index];
		pwm_setting.PWM_MODE_OLD_REGS.THRESH =
		    (led->blink_on_time * 100) / (led->blink_on_time +
						  led->blink_off_time);
	}

	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;
	pwm_set_spec_config(&pwm_setting);

	return 0;
}

/************************ led breath function*****************************/
/*************************************************************************
//func is to swtich to breath mode from PWM mode of ISINK
//para: enable: 1 : breath mode; 0: PWM mode;
*************************************************************************/
#if 0
static int led_switch_breath_pmic(enum mt65xx_led_pmic pmic_type,
				  struct nled_setting *led, int enable)
{
	/* int time_index = 0; */
	/* int duty = 0; */
	LEDS_DEBUG("led_blink_pmic: pmic_type=%d\n", pmic_type);

	if ((pmic_type != MT65XX_LED_PMIC_NLED_ISINK0
	     && pmic_type != MT65XX_LED_PMIC_NLED_ISINK1
	     && pmic_type != MT65XX_LED_PMIC_NLED_ISINK2
	     && pmic_type != MT65XX_LED_PMIC_NLED_ISINK3)
	    || led->nled_mode != NLED_BLINK) {
		return -1;
	}
	if (1 == enable) {
		switch (pmic_type) {
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH0_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TRF_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TR2_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TF1_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TF2_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL, 0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL, 0x03);
			pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_ON); */
			break;
		case MT65XX_LED_PMIC_NLED_ISINK1:
			pmic_set_register_value(PMIC_ISINK_CH1_MODE, ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH1_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TRF_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TR1_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TR2_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TF1_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TF2_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TON_SEL, 0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TOFF_SEL, 0x03);
			pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH1_EN,NLED_ON); */
			break;
		case MT65XX_LED_PMIC_NLED_ISINK2:
			pmic_set_register_value(PMIC_ISINK_CH4_MODE, ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH4_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH4_TRF_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH4_TR1_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH4_TR2_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH4_TF1_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH4_TF2_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH4_TON_SEL, 0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH4_TOFF_SEL, 0x03);
			pmic_set_register_value(PMIC_ISINK_DIM4_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM4_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH4_EN,NLED_ON); */
			break;
		case MT65XX_LED_PMIC_NLED_ISINK3:
			pmic_set_register_value(PMIC_ISINK_CH5_MODE, ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH5_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH5_TRF_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH5_TR1_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH5_TR2_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH5_TF1_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH5_TF2_SEL, 0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH5_TON_SEL, 0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH5_TOFF_SEL, 0x03);
			pmic_set_register_value(PMIC_ISINK_DIM5_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM5_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH5_EN,NLED_ON); */
			break;
		default:
			break;
		}
	} else {
		switch (pmic_type) {
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH0_MODE,
						ISINK_PWM_MODE);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK1:
			pmic_set_register_value(PMIC_ISINK_CH1_MODE,
						ISINK_PWM_MODE);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK2:
			pmic_set_register_value(PMIC_ISINK_CH4_MODE,
						ISINK_PWM_MODE);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK3:
			pmic_set_register_value(PMIC_ISINK_CH5_MODE,
						ISINK_PWM_MODE);
			break;
		default:
			break;
		}
	}
	return 0;

}
#endif

#define PMIC_PERIOD_NUM 8
/* 100 * period, ex: 0.01 Hz -> 0.01 * 100 = 1 */
int pmic_period_array[] = { 250, 500, 1000, 1250, 1666, 2000, 2500, 10000 };

/* int pmic_freqsel_array[] = {99999, 9999, 4999, 1999, 999, 499, 199, 4, 0}; */
int pmic_freqsel_array[] = { 0, 4, 199, 499, 999, 1999, 1999, 1999 };

static int find_time_index_pmic(int time_ms)
{
	int i;

	for (i = 0; i < PMIC_PERIOD_NUM; i++) {
		if (time_ms <= pmic_period_array[i])
			return i;
	}
	return PMIC_PERIOD_NUM - 1;
}

int mt_led_blink_pmic(enum mt65xx_led_pmic pmic_type, struct nled_setting *led)
{
#ifdef CONFIG_MTK_PMIC
	int time_index = 0;
	int duty = 0;

	LEDS_DEBUG("led_blink_pmic: pmic_type=%d\n", pmic_type);

	if (led->nled_mode != NLED_BLINK)
		return -1;

	LEDS_DEBUG("LED blink on time = %d offtime = %d\n",
		   led->blink_on_time, led->blink_off_time);
	time_index =
	    find_time_index_pmic(led->blink_on_time + led->blink_off_time);
	LEDS_DEBUG("LED index is %d  freqsel=%d\n", time_index,
		   pmic_freqsel_array[time_index]);
	duty =
	    32 * led->blink_on_time / (led->blink_on_time +
				       led->blink_off_time);
	if (pmic_type > MT65XX_LED_PMIC_NLED_ISINK_MIN && pmic_type < MT65XX_LED_PMIC_NLED_ISINK_MAX) {
		/* pmic_set_register_value(PMIC_RG_G_DRV_2M_CK_PDN(0X0); // DISABLE POWER DOWN ,Indicator no need) */
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		#else
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		#endif
	}

	switch (pmic_type) {
	case MT65XX_LED_PMIC_NLED_ISINK0:
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_set_register_value(PMIC_CLK_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(PMIC_ISINK_CH0_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, duty);
		pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,
					pmic_freqsel_array[time_index]);
		pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_ON);
		#else
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK0_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_DUTY, duty);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_FSEL,
					pmic_freqsel_array[time_index]);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_ON);
		#endif
		break;
	case MT65XX_LED_PMIC_NLED_ISINK1:
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_set_register_value(PMIC_CLK_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH1_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(PMIC_ISINK_CH1_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, duty);
		pmic_set_register_value(PMIC_ISINK_DIM1_FSEL,
					pmic_freqsel_array[time_index]);
		pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_ON);
		#else
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK1_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM1_DUTY, duty);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM1_FSEL,
					pmic_freqsel_array[time_index]);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_ON);
		#endif
		break;
	case MT65XX_LED_PMIC_NLED_ISINK2:
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_set_register_value(PMIC_CLK_DRV_ISINK2_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH2_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(PMIC_ISINK_CH2_EN, NLED_ON);
		#else
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK4_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK4_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH4_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH4_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM4_DUTY, duty);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM4_FSEL,
					pmic_freqsel_array[time_index]);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH4_EN, NLED_ON);
		#endif
		break;
	case MT65XX_LED_PMIC_NLED_ISINK3:
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_set_register_value(PMIC_CLK_DRV_ISINK3_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH3_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(PMIC_ISINK_CH3_EN, NLED_ON);
		#else
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK5_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK5_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH5_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH5_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM5_DUTY, duty);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM5_FSEL,
					pmic_freqsel_array[time_index]);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH5_EN, NLED_ON);
		#endif
		break;
	default:
		LEDS_DEBUG("[LEDS] pmic_type %d is not handled\n", pmic_type);
		break;
	}
#endif /* End of #ifdef CONFIG_MTK_PMIC */
	return 0;
}

int mt_backlight_set_pwm(int pwm_num, u32 level, u32 div,
			 struct PWM_config *config_data)
{
	struct pwm_spec_config pwm_setting = { .pwm_no = pwm_num};
	unsigned int BacklightLevelSupport =
	    Cust_GetBacklightLevelSupport_byPWM();
	pwm_setting.pwm_no = pwm_num;

	if (BacklightLevelSupport == BACKLIGHT_LEVEL_PWM_256_SUPPORT)
		pwm_setting.mode = PWM_MODE_OLD;
	else
		pwm_setting.mode = PWM_MODE_FIFO;	/* New mode fifo and periodical mode */

	pwm_setting.pmic_pad = config_data->pmic_pad;

	if (config_data->div) {
		pwm_setting.clk_div = config_data->div;
		backlight_PWM_div_hal = config_data->div;
	} else
		pwm_setting.clk_div = div;

	if (BacklightLevelSupport == BACKLIGHT_LEVEL_PWM_256_SUPPORT) {
		if (config_data->clock_source)
			pwm_setting.clk_src = PWM_CLK_OLD_MODE_BLOCK;
		else
			pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;	/* actually.
			it's block/1625 = 26M/1625 = 16KHz @ MT6571 */

		pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
		pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
		pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
		pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 255;	/* 256 level */
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = level;

		LEDS_DEBUG("[LEDS][%d]backlight_set_pwm:duty is %d/%d\n",
			   BacklightLevelSupport, level,
			   pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
		LEDS_DEBUG("[LEDS][%d]backlight_set_pwm:clk_src/div is %d%d\n",
			   BacklightLevelSupport, pwm_setting.clk_src,
			   pwm_setting.clk_div);
		if (level > 0 && level < 256) {
			pwm_set_spec_config(&pwm_setting);
			LEDS_DEBUG
			    ("[LEDS][%d]backlight_set_pwm: old mode: thres/data_width is %d/%d\n",
			     BacklightLevelSupport,
			     pwm_setting.PWM_MODE_OLD_REGS.THRESH,
			     pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
		} else {
			LEDS_DEBUG("[LEDS][%d]Error level in backlight\n",
				   BacklightLevelSupport);
			mt_pwm_disable(pwm_setting.pwm_no,
				       config_data->pmic_pad);
		}
		return 0;

	} else {
		if (config_data->clock_source) {
			pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		} else {
			pwm_setting.clk_src =
			    PWM_CLK_NEW_MODE_BLOCK_DIV_BY_1625;
		}

		if (config_data->High_duration && config_data->low_duration) {
			pwm_setting.PWM_MODE_FIFO_REGS.HDURATION =
			    config_data->High_duration;
			pwm_setting.PWM_MODE_FIFO_REGS.LDURATION =
			    pwm_setting.PWM_MODE_FIFO_REGS.HDURATION;
		} else {
			pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 4;
			pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 4;
		}

		pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
		pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
		pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 31;
		pwm_setting.PWM_MODE_FIFO_REGS.GDURATION =
		    (pwm_setting.PWM_MODE_FIFO_REGS.HDURATION + 1) * 32 - 1;
		pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;

		LEDS_DEBUG("[LEDS]backlight_set_pwm:duty is %d\n", level);
		LEDS_DEBUG
		    ("[LEDS]backlight_set_pwm:clk_src/div/high/low is %d%d%d%d\n",
		     pwm_setting.clk_src, pwm_setting.clk_div,
		     pwm_setting.PWM_MODE_FIFO_REGS.HDURATION,
		     pwm_setting.PWM_MODE_FIFO_REGS.LDURATION);

		if (level > 0 && level <= 32) {
			pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
			pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 =
			    (1 << level) - 1;
			pwm_set_spec_config(&pwm_setting);
		} else if (level > 32 && level <= 64) {
			pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 1;
			level -= 32;
			pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 =
			    (1 << level) - 1;
			pwm_set_spec_config(&pwm_setting);
		} else {
			LEDS_DEBUG("[LEDS]Error level in backlight\n");
			mt_pwm_disable(pwm_setting.pwm_no,
				       config_data->pmic_pad);
		}

		return 0;

	}
}

void mt_led_pwm_disable(int pwm_num)
{
	struct cust_mt65xx_led *cust_led_list = get_cust_led_dtsi();

	mt_pwm_disable(pwm_num, cust_led_list->config_data.pmic_pad);
}

void mt_backlight_set_pwm_duty(int pwm_num, u32 level, u32 div,
			       struct PWM_config *config_data)
{
	mt_backlight_set_pwm(pwm_num, level, div, config_data);
}

void mt_backlight_set_pwm_div(int pwm_num, u32 level, u32 div,
			      struct PWM_config *config_data)
{
	mt_backlight_set_pwm(pwm_num, level, div, config_data);
}

void mt_backlight_get_pwm_fsel(unsigned int bl_div, unsigned int *bl_frequency)
{

}

void mt_store_pwm_register(unsigned int addr, unsigned int value)
{

}

unsigned int mt_show_pwm_register(unsigned int addr)
{
	return 0;
}

#define PMIC_BACKLIGHT_LEVEL    80
int mt_brightness_set_pmic(enum mt65xx_led_pmic pmic_type, u32 level, u32 div)
{
	static bool first_time = true;
#if 0
	int tmp_level = level;
	static bool backlight_init_flag;
	static unsigned char duty_mapping[PMIC_BACKLIGHT_LEVEL] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
		12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19,
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
		21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 24,
		25, 26, 27, 28, 29, 30, 31, 25, 26, 27, 28, 29,
		30, 31, 26, 27, 28, 29, 30, 31,
	};
	static unsigned char current_mapping[PMIC_BACKLIGHT_LEVEL] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3,
		3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4,
		4, 4, 5, 5, 5, 5, 5, 5,
	};
#endif
	LEDS_DEBUG("PMIC#%d:%d\n", pmic_type, level);
	mutex_lock(&leds_pmic_mutex);
	if (pmic_type == MT65XX_LED_PMIC_LCD_ISINK) {
#if 0
		if (backlight_init_flag == false) {
			pmic_set_register_value(PMIC_RG_G_DRV_2M_CK_PDN, 0x0);	/* Disable power down */

			/* For backlight: Current: 24mA, PWM frequency: 20K */
			/* Duty: 20~100, Soft start: off, Phase shift: on */
			/* ISINK0 */
			pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN, 0x0);	/* Disable power down */
			pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL, 0x1);	/* Freq = 1Mhz for Backlight */
			pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_PWM_MODE);
			pmic_set_register_value(PMIC_ISINK_CH0_STEP, ISINK_5);	/* 24mA */
			pmic_set_register_value(PMIC_ISINK_SFSTR0_EN, 0x0);	/* Disable soft start */
			pmic_set_register_value(PMIC_ISINK_PHASE_DLY_TC, 0x0);	/* TC = 0.5us */
			pmic_set_register_value(PMIC_ISINK_PHASE0_DLY_EN, 0x1);	/* Enable phase delay */
			pmic_set_register_value(PMIC_ISINK_CHOP0_EN, 0x1);	/* Enable CHOP clk */
			/* ISINK1 */
			pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_PDN, 0x0);	/* Disable power down */
			pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_CKSEL, 0x1);	/* Freq = 1Mhz for Backlight */
			pmic_set_register_value(PMIC_ISINK_CH1_MODE, ISINK_PWM_MODE);
			pmic_set_register_value(PMIC_ISINK_CH1_STEP, ISINK_3);	/* 24mA */
			pmic_set_register_value(PMIC_ISINK_SFSTR1_EN, 0x0);	/* Disable soft start */
			pmic_set_register_value(PMIC_ISINK_PHASE1_DLY_EN, 0x1);	/* Enable phase delay */
			pmic_set_register_value(PMIC_ISINK_CHOP1_EN, 0x1);	/* Enable CHOP clk */
			/* ISINK2 */
			pmic_set_register_value(PMIC_RG_DRV_ISINK2_CK_PDN, 0x0);	/* Disable power down */
			pmic_set_register_value(PMIC_RG_DRV_ISINK2_CK_CKSEL, 0x1);	/* Freq = 1Mhz for Backlight */
			pmic_set_register_value(PMIC_ISINK_CH2_STEP, ISINK_3);	/* 24mA */
			pmic_set_register_value(PMIC_ISINK_CHOP2_EN, 0x1);	/* Enable CHOP clk */
			/* ISINK3 */
			pmic_set_register_value(PMIC_RG_DRV_ISINK3_CK_PDN, 0x0);	/* Disable power down */
			pmic_set_register_value(PMIC_RG_DRV_ISINK3_CK_CKSEL, 0x1);	/* Freq = 1Mhz for Backlight */
			pmic_set_register_value(PMIC_ISINK_CH3_STEP, ISINK_3);	/* 24mA */
			pmic_set_register_value(PMIC_ISINK_CHOP3_EN, 0x1);	/* Enable CHOP clk */
			/* ISINK4 */
			pmic_set_register_value(PMIC_RG_DRV_ISINK4_CK_PDN, 0x0);	/* Disable power down */
			pmic_set_register_value(PMIC_RG_DRV_ISINK4_CK_CKSEL, 0x1);	/* Freq = 1Mhz for Backlight */
			pmic_set_register_value(PMIC_ISINK_CH4_MODE, ISINK_PWM_MODE);
			pmic_set_register_value(PMIC_ISINK_CH4_STEP, ISINK_5);	/* 24mA */
			pmic_set_register_value(PMIC_ISINK_SFSTR4_EN, 0x0);	/* Disable soft start */
			pmic_set_register_value(PMIC_ISINK_PHASE4_DLY_EN, 0x1);	/* Enable phase delay */
			pmic_set_register_value(PMIC_ISINK_CHOP4_EN, 0x1);	/* Enable CHOP clk */
			/* ISINK5 */
			pmic_set_register_value(PMIC_RG_DRV_ISINK5_CK_PDN, 0x0);	/* Disable power down */
			pmic_set_register_value(PMIC_RG_DRV_ISINK5_CK_CKSEL, 0x1);	/* Freq = 1Mhz for Backlight */
			pmic_set_register_value(PMIC_ISINK_CH5_MODE, ISINK_PWM_MODE);
			pmic_set_register_value(PMIC_ISINK_CH5_STEP, ISINK_3);	/* 24mA */
			pmic_set_register_value(PMIC_ISINK_SFSTR5_EN, 0x0);	/* Disable soft start */
			pmic_set_register_value(PMIC_ISINK_PHASE5_DLY_EN, 0x1);	/* Enable phase delay */
			pmic_set_register_value(PMIC_ISINK_CHOP5_EN, 0x1);	/* Enable CHOP clk */
			/* ISINK6 */
			pmic_set_register_value(PMIC_RG_DRV_ISINK6_CK_PDN, 0x0);	/* Disable power down */
			pmic_set_register_value(PMIC_RG_DRV_ISINK6_CK_CKSEL, 0x1);	/* Freq = 1Mhz for Backlight */
			pmic_set_register_value(PMIC_ISINK_CH6_STEP, ISINK_3);	/* 24mA */
			pmic_set_register_value(PMIC_ISINK_CHOP6_EN, 0x1);	/* Enable CHOP clk */
			/* ISINK7 */
			pmic_set_register_value(PMIC_RG_DRV_ISINK7_CK_PDN, 0x0);	/* Disable power down */
			pmic_set_register_value(PMIC_RG_DRV_ISINK7_CK_CKSEL, 0x1);	/* Freq = 1Mhz for Backlight */
			pmic_set_register_value(PMIC_ISINK_CH7_STEP, ISINK_3);	/* 24mA */
			pmic_set_register_value(PMIC_ISINK_CHOP7_EN, 0x1);	/* Enable CHOP clk */
			backlight_init_flag = true;
		}

		if (level) {
			level = brightness_mapping(tmp_level);
			if (level == ERROR_BL_LEVEL)
				level = 255;
			if (level == 255)
				level = PMIC_BACKLIGHT_LEVEL;
			else
				level = ((level * PMIC_BACKLIGHT_LEVEL) / 255) + 1;

			LEDS_DEBUG("[LED]Level Mapping = %d\n", level);
			LEDS_DEBUG("[LED]ISINK DIM Duty = %d\n", duty_mapping[level - 1]);
			LEDS_DEBUG("[LED]ISINK Current = %d\n", current_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,
						duty_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_DIM1_DUTY,
						duty_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_DIM4_DUTY,
						duty_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_DIM5_DUTY,
						duty_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_CH0_STEP,
						current_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_CH1_STEP,
						current_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_CH2_STEP,
						current_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_CH3_STEP,
						current_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_CH4_STEP,
						current_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_CH5_STEP,
						current_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_CH6_STEP,
						current_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_CH7_STEP,
						current_mapping[level - 1]);
			pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, ISINK_2M_20KHZ);	/* 20Khz */
			pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, ISINK_2M_20KHZ);	/* 20Khz */
			pmic_set_register_value(PMIC_ISINK_DIM4_FSEL, ISINK_2M_20KHZ);	/* 20Khz */
			pmic_set_register_value(PMIC_ISINK_DIM5_FSEL, ISINK_2M_20KHZ);	/* 20Khz */
			pmic_set_register_value(PMIC_ISINK_CH0_EN, 0x1);	/* Turn on ISINK Channel 0 */
			pmic_set_register_value(PMIC_ISINK_CH1_EN, 0x1);	/* Turn on ISINK Channel 1 */
			pmic_set_register_value(PMIC_ISINK_CH2_EN, 0x1);	/* Turn on ISINK Channel 2 */
			pmic_set_register_value(PMIC_ISINK_CH3_EN, 0x1);	/* Turn on ISINK Channel 3 */
			pmic_set_register_value(PMIC_ISINK_CH4_EN, 0x1);	/* Turn on ISINK Channel 4 */
			pmic_set_register_value(PMIC_ISINK_CH5_EN, 0x1);	/* Turn on ISINK Channel 5 */
			pmic_set_register_value(PMIC_ISINK_CH6_EN, 0x1);	/* Turn on ISINK Channel 6 */
			pmic_set_register_value(PMIC_ISINK_CH7_EN, 0x1);	/* Turn on ISINK Channel 7 */
		} else {
			pmic_set_register_value(PMIC_ISINK_CH0_EN, 0x0);	/* Turn off ISINK Channel 0 */
			pmic_set_register_value(PMIC_ISINK_CH1_EN, 0x0);	/* Turn off ISINK Channel 1 */
			pmic_set_register_value(PMIC_ISINK_CH2_EN, 0x0);	/* Turn off ISINK Channel 2 */
			pmic_set_register_value(PMIC_ISINK_CH3_EN, 0x0);	/* Turn off ISINK Channel 3 */
			pmic_set_register_value(PMIC_ISINK_CH4_EN, 0x0);	/* Turn on ISINK Channel 4 */
			pmic_set_register_value(PMIC_ISINK_CH5_EN, 0x0);	/* Turn on ISINK Channel 5 */
			pmic_set_register_value(PMIC_ISINK_CH6_EN, 0x0);	/* Turn on ISINK Channel 6 */
			pmic_set_register_value(PMIC_ISINK_CH7_EN, 0x0);	/* Turn on ISINK Channel 7 */
		}
#endif				/* No Support */
		mutex_unlock(&leds_pmic_mutex);
		return 0;
	} else if (pmic_type == MT65XX_LED_PMIC_NLED_ISINK0) {
		if ((button_flag_isink0 == 0) && (first_time == true)) {
			/* button flag ==0, means this ISINK is not for button backlight */
			if (button_flag_isink1 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH1_EN,
				 NLED_OFF);	/* sw workround for sync leds status */
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN,
				 NLED_OFF);	/* sw workround for sync leds status */
				#endif
			if (button_flag_isink2 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH2_EN, NLED_OFF);
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH2_EN,
							NLED_OFF);
				#endif
			if (button_flag_isink3 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH3_EN, NLED_OFF);
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH3_EN,
							NLED_OFF);
				#endif
			first_time = false;
		}
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_CLK_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_PWM_MODE);
#ifdef CONFIG_LGE_LEDS_BREATHMODE
		pmic_set_register_value(PMIC_ISINK_CH0_STEP, ISINK_1);	/* 8mA */
		pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, level / 8); /* 0~31 duty ratio */
#else
		pmic_set_register_value(PMIC_ISINK_CH0_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, 15);
#endif
		pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, ISINK_1KHZ);
		#else
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK0_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_MODE, ISINK_PWM_MODE);
#ifdef CONFIG_LGE_LEDS_BREATHMODE
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_STEP, ISINK_1);	/* 8mA */
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_DUTY, level / 8); /* 0~31 duty ratio */
#else
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_DUTY, 15);
#endif

		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_FSEL, ISINK_1KHZ);	/* 1KHz */
		#endif
		if (level) {
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
			pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_ON);
			#else
			pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
			pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_ON);
			#endif
		} else {
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
			pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_OFF);
			#else
			pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_OFF);
			#endif
		}
		mutex_unlock(&leds_pmic_mutex);
		return 0;
	} else if (pmic_type == MT65XX_LED_PMIC_NLED_ISINK1) {
		if ((button_flag_isink1 == 0) && (first_time == true)) {
			/* button flag ==0, means this ISINK is not for button backlight */
			if (button_flag_isink0 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH0_EN,
				 NLED_OFF);	/* sw workround for sync leds status */
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN,
				 NLED_OFF);	/* sw workround for sync leds status */
				#endif
			if (button_flag_isink2 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH2_EN,
				 NLED_OFF);	/* sw workround for sync leds status */
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH4_EN,
							NLED_OFF);
				#endif
			if (button_flag_isink3 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH3_EN,
				 NLED_OFF);	/* sw workround for sync leds status */
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH5_EN,
							NLED_OFF);
				#endif
			first_time = false;
		}
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_CLK_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH1_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(PMIC_ISINK_CH1_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, 15);
		pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, ISINK_1KHZ);
		#else
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK1_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM1_DUTY, 15);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM1_FSEL, ISINK_1KHZ);	/* 1KHz */
		#endif
		if (level) {
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
			pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_ON);
			#else
			pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_ON);
			#endif
		} else {
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
			pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_OFF);
			#else
			pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_OFF);
			#endif
		}
		mutex_unlock(&leds_pmic_mutex);
		return 0;
	} else if (pmic_type == MT65XX_LED_PMIC_NLED_ISINK2) {
		if ((button_flag_isink2 == 0) && (first_time == true)) {
			/* button flag ==0, means this ISINK is not for button backlight */
			if (button_flag_isink0 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_OFF);
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_OFF);
				#endif
			if (button_flag_isink1 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_OFF);
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_OFF);
				#endif
			if (button_flag_isink3 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH3_EN, NLED_OFF);
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH5_EN, NLED_OFF);
				#endif
			first_time = false;
		}
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_CLK_DRV_ISINK2_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH2_STEP, ISINK_3);	/* 16mA */
		#else
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK4_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK4_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH4_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH4_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM4_DUTY, 15);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM4_FSEL, ISINK_1KHZ);	/* 1KHz */
		#endif
		if (level) {
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
			pmic_set_register_value(PMIC_ISINK_CH2_EN, NLED_ON);
			#else
			pmic_set_register_value(MT6351_PMIC_ISINK_CH4_EN, NLED_ON);
			#endif
		} else {
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
			pmic_set_register_value(PMIC_ISINK_CH2_EN, NLED_OFF);
			#else
			pmic_set_register_value(MT6351_PMIC_ISINK_CH4_EN, NLED_OFF);
			#endif
		}
	} else if (pmic_type == MT65XX_LED_PMIC_NLED_ISINK3) {
		if ((button_flag_isink3 == 0) && (first_time == true)) {
			/* button flag ==0, means this ISINK is not for button backlight */
			if (button_flag_isink0 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH0_EN,
				 NLED_OFF);	/* sw workround for sync leds status */
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN,
				 NLED_OFF);	/* sw workround for sync leds status */
				#endif
			if (button_flag_isink1 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH1_EN,
				 NLED_OFF);	/* sw workround for sync leds status */
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN,
							NLED_OFF);
				#endif
			if (button_flag_isink2 == 0)
				#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
				pmic_set_register_value(PMIC_ISINK_CH2_EN,
				 NLED_OFF);	/* sw workround for sync leds status */
				#else
				pmic_set_register_value(MT6351_PMIC_ISINK_CH4_EN,
							NLED_OFF);
				#endif
			first_time = false;
		}
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_CLK_DRV_ISINK3_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH3_STEP, ISINK_3);	/* 16mA */
		#else
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK5_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK5_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH5_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH5_STEP, ISINK_3);	/* 16mA */
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM5_DUTY, 15);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM5_FSEL, ISINK_1KHZ);	/* 1KHz */
		#endif
		if (level) {
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
			pmic_set_register_value(PMIC_ISINK_CH3_EN, NLED_ON);
			#else
			pmic_set_register_value(MT6351_PMIC_ISINK_CH5_EN, NLED_ON);
			#endif
		} else {
			#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
			pmic_set_register_value(PMIC_ISINK_CH3_EN, NLED_OFF);
			#else
			pmic_set_register_value(MT6351_PMIC_ISINK_CH5_EN, NLED_OFF);
			#endif
		}
		mutex_unlock(&leds_pmic_mutex);
		return 0;
	}
	mutex_unlock(&leds_pmic_mutex);
	return -1;
}

int mt_brightness_set_pmic_duty_store(u32 level, u32 div)
{
	return -1;
}

int mt_mt65xx_led_set_cust(struct cust_mt65xx_led *cust, int level)
{
	struct nled_setting led_tmp_setting = { 0, 0, 0 };
	int tmp_level = level;
	static bool button_flag;
	unsigned int BacklightLevelSupport =
	    Cust_GetBacklightLevelSupport_byPWM();

	switch (cust->mode) {

	case MT65XX_LED_MODE_PWM:
		if (strcmp(cust->name, "lcd-backlight") == 0) {
			bl_brightness_hal = level;
			if (level == 0) {
				mt_pwm_disable(cust->data,
					       cust->config_data.pmic_pad);

			} else {

				if (BacklightLevelSupport ==
				    BACKLIGHT_LEVEL_PWM_256_SUPPORT)
					level = brightness_mapping(tmp_level);
				else
					level = brightness_mapto64(tmp_level);
				mt_backlight_set_pwm(cust->data, level,
						     bl_div_hal,
						     &cust->config_data);
			}
			bl_duty_hal = level;

		} else {
			if (level == 0) {
				led_tmp_setting.nled_mode = NLED_OFF;
				mt_led_set_pwm(cust->data, &led_tmp_setting);
				mt_pwm_disable(cust->data,
					       cust->config_data.pmic_pad);
			} else {
				led_tmp_setting.nled_mode = NLED_ON;
				mt_led_set_pwm(cust->data, &led_tmp_setting);
			}
		}
		return 1;

	case MT65XX_LED_MODE_GPIO:
		LEDS_DEBUG("brightness_set_cust:go GPIO mode!!!!!\n");
		return ((cust_set_brightness) (cust->data)) (level);

	case MT65XX_LED_MODE_PMIC:
		/* for button baclight used SINK channel, when set button ISINK,
			don't do disable other ISINK channel */
		if ((strcmp(cust->name, "button-backlight") == 0)) {
			if (button_flag == false) {
				switch (cust->data) {
				case MT65XX_LED_PMIC_NLED_ISINK0:
					button_flag_isink0 = 1;
					break;
				case MT65XX_LED_PMIC_NLED_ISINK1:
					button_flag_isink1 = 1;
					break;
				case MT65XX_LED_PMIC_NLED_ISINK2:
					button_flag_isink2 = 1;
					break;
				case MT65XX_LED_PMIC_NLED_ISINK3:
					button_flag_isink3 = 1;
					break;
				default:
					break;
				}
				button_flag = true;
			}
		}
		return mt_brightness_set_pmic(cust->data, level, bl_div_hal);

	case MT65XX_LED_MODE_CUST_LCM:
		if (strcmp(cust->name, "lcd-backlight") == 0)
#ifdef CONFIG_LGE_LCD_OFF_DIMMING
		{
			static int read_done_flag=0;
			static int lge_boot_reason = 0;

			if (read_done_flag == 0) {
				lge_boot_reason = lge_get_bootreason();
				printk("%s : lge_boot_reason : 0x%x \n",__func__, lge_boot_reason);
				read_done_flag = 1;
			}

			if (!fb_blank_called && (level != 0)) {
				if (lge_boot_reason == LGE_REBOOT_REASON_FOTA_LCD_OFF ||
							lge_boot_reason == LGE_REBOOT_REASON_FOTA_OUT_LCD_OFF ||
							lge_boot_reason == LGE_REBOOT_REASON_LCD_OFF){
					level = 10;
					printk("%s : lcd dimming mode! minimum brightness. \n", __func__);
				}
			}
			bl_brightness_hal = level;
		}
#else
		bl_brightness_hal = level;
#endif
		LEDS_DEBUG("brightness_set_cust:backlight control by LCM\n");
		/* warning for this API revork */
		return ((cust_brightness_set) (cust->data)) (level, bl_div_hal);

	case MT65XX_LED_MODE_CUST_BLS_PWM:
		if (strcmp(cust->name, "lcd-backlight") == 0)
			bl_brightness_hal = level;

		return ((cust_set_brightness) (cust->data)) (level);

	case MT65XX_LED_MODE_NONE:
	default:
		break;
	}
	return -1;
}

void mt_mt65xx_led_work(struct work_struct *work)
{
	struct mt65xx_led_data *led_data =
	    container_of(work, struct mt65xx_led_data, work);

	LEDS_DEBUG("%s:%d\n", led_data->cust.name, led_data->level);
	mutex_lock(&leds_mutex);
	mt_mt65xx_led_set_cust(&led_data->cust, led_data->level);
	mutex_unlock(&leds_mutex);
}

#ifdef CONFIG_MTK_AAL_SUPPORT
#define MAX_LEVEL 255
/* Gamma 2.2 Mapping Table */
#ifdef CONFIG_SW49107_LGD_FHDPLUS_DSI_CMD_CV7A
unsigned int Gamma[] = {
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11,
    11, 12, 12, 13, 13, 13, 14, 15, 15, 16, 17, 17, 18, 19, 19, 21,
    22, 23, 24, 26, 27, 28, 29, 31, 32, 33, 35, 35, 36, 37, 39, 40,
    41, 43, 44, 46, 47, 48, 50, 51, 53, 54, 56, 58, 60, 62, 64, 65,
    67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 90, 92, 94, 96, 98,
    100, 103, 105, 108, 111, 114, 117, 120, 123, 126, 129, 132, 135, 137, 140, 143,
    146, 148, 151, 154, 157, 159, 162, 165, 168, 171, 175, 179, 183, 187, 191, 195,
    199, 203, 207, 211, 215, 220, 223, 227, 231, 235, 239, 243, 247, 251, 255, 259,
    263, 267, 271, 275, 279, 283, 287, 291, 295, 299, 303, 307, 311, 315, 320, 325,
    331, 336, 341, 347, 352, 358, 363, 368, 374, 379, 385, 390, 396, 401, 407, 413,
    418, 424, 429, 435, 441, 446, 452, 458, 465, 472, 479, 486, 493, 500, 507, 514,
    521, 528, 535, 540, 545, 551, 556, 561, 567, 572, 578, 583, 588, 594, 599, 605,
    614, 623, 632, 641, 650, 660, 669, 678, 687, 696, 705, 715, 721, 728, 734, 741,
    747, 754, 760, 767, 773, 780, 786, 793, 800, 807, 815, 823, 830, 838, 846, 853,
    861, 869, 876, 884, 892, 900, 911, 923, 938, 952, 965, 978, 988, 997, 1015, 1019
};
#else
unsigned int Gamma[] = {
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11,
11, 12, 12, 13, 13, 13, 14, 15, 15, 16, 17, 17, 18, 19, 19, 21,
22, 23, 24, 26, 27, 28, 29, 31, 32, 33, 35, 35, 36, 37, 39, 40,
41, 43, 44, 46, 47, 48, 50, 51, 53, 54, 56, 58, 60, 62, 64, 65,
67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 90, 92, 94, 96, 98,
100, 103, 105, 108, 111, 114, 117, 120, 123, 126, 129, 132, 135, 137, 140, 143,
146, 148, 151, 154, 157, 159, 162, 165, 168, 171, 175, 179, 183, 187, 191, 195,
199, 203, 207, 211, 215, 220, 223, 227, 231, 235, 239, 243, 247, 251, 255, 259,
263, 267, 271, 275, 279, 283, 287, 291, 295, 299, 303, 307, 311, 315, 320, 325,
331, 336, 341, 347, 352, 358, 363, 368, 374, 379, 385, 390, 396, 401, 407, 413,
418, 424, 429, 435, 441, 446, 452, 458, 465, 472, 479, 486, 493, 500, 507, 514,
521, 528, 535, 540, 545, 551, 556, 561, 567, 572, 578, 583, 588, 594, 599, 605,
614, 623, 632, 641, 650, 660, 669, 678, 687, 696, 705, 715, 721, 728, 734, 741,
747, 754, 760, 767, 773, 780, 786, 793, 800, 807, 815, 823, 830, 838, 846, 853,
861, 869, 876, 884, 892, 900, 911, 923, 935, 947, 958, 970, 982, 994, 1005, 1017
};
#endif
#endif

void mt_mt65xx_led_set(struct led_classdev *led_cdev, enum led_brightness level)
{
#ifdef CONFIG_MTK_AAL_SUPPORT
	unsigned int level10_mapping = 10;
	unsigned int results = 0;
#endif
	struct mt65xx_led_data *led_data =
	    container_of(led_cdev, struct mt65xx_led_data, cdev);

#ifdef CONFIG_MTK_AAL_SUPPORT
#define AAL_MIN_BRIGHTNESS 108 /* under 40% brightness level, ESS is not running */
	if (led_data->level != level) {
		led_data->level = level;
		if (strcmp(led_data->cust.name, "lcd-backlight") != 0) {
			LEDS_DEBUG("Set NLED directly %d at time %lu\n",
				   led_data->level, jiffies);
			schedule_work(&led_data->work);
		} else {
			if (level != 0
			    && level * CONFIG_LIGHTNESS_MAPPING_VALUE < 255) {
				level = 1;
			} else {
				level =
				    (level * CONFIG_LIGHTNESS_MAPPING_VALUE) /
				    255;
			}

                       if (level < AAL_MIN_BRIGHTNESS && disp_aal_get_ess_en()) {
				disp_aal_set_ess_en(false);
				LEDS_DEBUG("[AAL] level: %d, ESS turn off!!\n", level);
                       } else if (level >= AAL_MIN_BRIGHTNESS && !disp_aal_get_ess_en()) {
				disp_aal_set_ess_en(true);
				LEDS_DEBUG("[AAL] level: %d, ESS turn on!!\n", level);
                       }

			backlight_debug_log(led_data->level, level);

			if(led_data->cust.mode == MT65XX_LED_MODE_CUST_BLS_PWM){
				if (level != 0){
					results = down_interruptible(&qpps_lock);
					level10_mapping = Gamma[level];
					if (level10_mapping > Gamma[MAX_LEVEL])
						level10_mapping = Gamma[MAX_LEVEL];
					up(&qpps_lock);
				}
				else{
					level10_mapping = 0;
				}

				LEDS_DEBUG("level = %d, level10_mapping = %d\n",level,level10_mapping);
				disp_pq_notify_backlight_changed(level10_mapping);
				disp_aal_notify_backlight_changed(level10_mapping);
			}else{
				mt_mt65xx_led_set_cust(&led_data->cust, level);
			}
		}
	}
#else
	/* do something only when level is changed */
	if (led_data->level != level) {
		led_data->level = level;
		if (strcmp(led_data->cust.name, "lcd-backlight") != 0) {
			LEDS_DEBUG("Set NLED directly %d at time %lu\n",
				   led_data->level, jiffies);
			schedule_work(&led_data->work);
		} else {
			if (level != 0
			    && level * CONFIG_LIGHTNESS_MAPPING_VALUE < 255) {
				level = 1;
			} else {
				level =
				    (level * CONFIG_LIGHTNESS_MAPPING_VALUE) /
				    255;
			}
			/* LEDS_DEBUG
			    ("Set Backlight directly %d at time %lu, mapping level is %d\n",
			     led_data->level, jiffies, level); */
			backlight_debug_log(led_data->level, level);
			disp_pq_notify_backlight_changed((((1 <<
							     MT_LED_INTERNAL_LEVEL_BIT_CNT)
							    - 1) * level +
							   127) / 255);
			if (MT65XX_LED_MODE_CUST_BLS_PWM == led_data->cust.mode) {
#if defined(CONFIG_TOVIS_INCELL_LG4894_HD_CV3_SPR)
				mt_mt65xx_led_set_cust(&led_data->cust, bright_arr_pwm[level]);
#else
				mt_mt65xx_led_set_cust(&led_data->cust,
						       ((((1 <<
							   MT_LED_INTERNAL_LEVEL_BIT_CNT)
							  - 1) * level +
							 127) / 255));
#endif
			} else {
				mt_mt65xx_led_set_cust(&led_data->cust, level);
			}
		}
	}
	/* spin_unlock_irqrestore(&leds_lock, flags); */
#endif
/* if(0!=aee_kernel_Powerkey_is_press()) */
/* aee_kernel_wdt_kick_Powkey_api("mt_mt65xx_led_set",WDT_SETBY_Backlight); */
}

int mt_mt65xx_blink_set(struct led_classdev *led_cdev,
			unsigned long *delay_on, unsigned long *delay_off)
{
	struct mt65xx_led_data *led_data =
	    container_of(led_cdev, struct mt65xx_led_data, cdev);
	static int got_wake_lock;
	struct nled_setting nled_tmp_setting = { 0, 0, 0 };

	/* only allow software blink when delay_on or delay_off changed */
	if (*delay_on != led_data->delay_on
	    || *delay_off != led_data->delay_off) {
		led_data->delay_on = *delay_on;
		led_data->delay_off = *delay_off;
		if (led_data->delay_on && led_data->delay_off) {	/* enable blink */
			led_data->level = 255;	/* when enable blink  then to set the level  (255) */
			/* AP PWM all support OLD mode */
			if (led_data->cust.mode == MT65XX_LED_MODE_PWM) {
				nled_tmp_setting.nled_mode = NLED_BLINK;
				nled_tmp_setting.blink_off_time =
				    led_data->delay_off;
				nled_tmp_setting.blink_on_time =
				    led_data->delay_on;
				mt_led_set_pwm(led_data->cust.data,
					       &nled_tmp_setting);
				return 0;
			} else if ((led_data->cust.mode == MT65XX_LED_MODE_PMIC)
				   && (led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK0
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK1
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK2
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK3)) {
				nled_tmp_setting.nled_mode = NLED_BLINK;
				nled_tmp_setting.blink_off_time =
				    led_data->delay_off;
				nled_tmp_setting.blink_on_time =
				    led_data->delay_on;
				mt_led_blink_pmic(led_data->cust.data,
						  &nled_tmp_setting);
				return 0;
			} else if (!got_wake_lock) {
				wake_lock(&leds_suspend_lock);
				got_wake_lock = 1;
			}
		} else if (!led_data->delay_on && !led_data->delay_off) {	/* disable blink */
			/* AP PWM all support OLD mode */
			if (led_data->cust.mode == MT65XX_LED_MODE_PWM) {
				nled_tmp_setting.nled_mode = NLED_OFF;
				mt_led_set_pwm(led_data->cust.data,
					       &nled_tmp_setting);
				return 0;
			} else if ((led_data->cust.mode == MT65XX_LED_MODE_PMIC)
				   && (led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK0
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK1
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK2
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK3)) {
				mt_brightness_set_pmic(led_data->cust.data, 0,
						       0);
				return 0;
			} else if (got_wake_lock) {
				wake_unlock(&leds_suspend_lock);
				got_wake_lock = 0;
			}
		}
		return -1;
	}
	/* delay_on and delay_off are not changed */
	return 0;
}

#ifdef CONFIG_LGE_LEDS_BREATHMODE
#define kal_uint32 unsigned int

typedef struct {
	kal_uint32 isink1_step;
	kal_uint32 isink1_dim1_duty;
	kal_uint32 isink1_dim1_fsel;
	kal_uint32 isink1_breath1_tr1_sel;
	kal_uint32 isink1_breath1_tr2_sel;
	kal_uint32 isink1_breath1_tf1_sel;
	kal_uint32 isink1_breath1_tf2_sel;
	kal_uint32 isink1_breath1_ton_sel;
	kal_uint32 isink1_breath1_toff_sel;
} ISINK1_LED_Breathmode_set_table;

static ISINK1_LED_Breathmode_set_table led_scene_setting[] = {
	{0x1, 31, 999,  7, 2, 7, 4, 3, 0},// 0.charging
	{0x1, 31, 999,  7, 2, 7, 4, 3, 0},// 1.Power on
	{0x1, 31, 999,  7, 2, 7, 4, 3, 0},// 2.Power off
	{0x1, 31, 999,  0, 0, 0, 0, 0, 0},// 3.Alarm
	{0x1, 31, 999,  2, 0, 0, 0, 0, 0},// 4.Incomming call
	{0x1, 31, 999,  0, 0, 0, 0, 0, 0},// 5.Urgent Incomming call
	{0x1, 31, 999,  4, 0, 0, 0, 3, 2},// 6.In call
	{0x1, 31, 999,  0, 0, 0, 0, 4, 0},// 7.Knock_on / Knock_code_fail / Bluetooth connect / Bluetooth disconnect
	{0x1, 31, 999,  0, 0, 0, 0, 0, 4},// 8.Missed call / Urgent missed call
	{0x1, 31, 999,  7, 2, 7, 4, 3, 0},// 9.default
};
void mt_mt65xx_breath_mode(unsigned int patternid)
{
	/* sangcheol.seo@lge.com 20140919*/
	// MT65XX_LED_PMIC_NLED_ISINK0
	LEDS_DEBUG(" patternid = %d\n", patternid);

	LEDS_DEBUG(" ch1_step = 0x%x, dim1_duty = %d, dim1_fsel = %d, breath1_tr1_sel = %d, breath1_tr2_sel = %d, breath1_tf1_sel = %d, \
	breath1_tf2_sel = %d, breath1_ton_sel = %d, breath1_toff_sel = %d\n",\
		led_scene_setting[patternid].isink1_step, led_scene_setting[patternid].isink1_dim1_duty, led_scene_setting[patternid].isink1_dim1_fsel, \
		led_scene_setting[patternid].isink1_breath1_tr1_sel, led_scene_setting[patternid].isink1_breath1_tr2_sel, \
		led_scene_setting[patternid].isink1_breath1_tf1_sel, led_scene_setting[patternid].isink1_breath1_tf2_sel, \
		led_scene_setting[patternid].isink1_breath1_ton_sel, led_scene_setting[patternid].isink1_breath1_toff_sel);

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_set_register_value(PMIC_CLK_DRV_ISINK0_CK_PDN, 0);
	pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);
	pmic_set_register_value(PMIC_ISINK_CH0_STEP, led_scene_setting[patternid].isink1_step);
	pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, led_scene_setting[patternid].isink1_dim1_duty);
	pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, led_scene_setting[patternid].isink1_dim1_fsel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL, led_scene_setting[patternid].isink1_breath1_tr1_sel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TR2_SEL, led_scene_setting[patternid].isink1_breath1_tr2_sel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TF1_SEL, led_scene_setting[patternid].isink1_breath1_tf1_sel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TF2_SEL, led_scene_setting[patternid].isink1_breath1_tf2_sel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL, led_scene_setting[patternid].isink1_breath1_ton_sel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL, led_scene_setting[patternid].isink1_breath1_toff_sel);
	pmic_set_register_value(PMIC_ISINK_CH0_EN, 0x01);
#else
	pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN, 0);
	pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL, 0);
	pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);
	pmic_set_register_value(PMIC_ISINK_CH0_STEP, led_scene_setting[patternid].isink1_step);
	pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, led_scene_setting[patternid].isink1_dim1_duty);
	pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, led_scene_setting[patternid].isink1_dim1_fsel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL, led_scene_setting[patternid].isink1_breath1_tr1_sel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TR2_SEL, led_scene_setting[patternid].isink1_breath1_tr2_sel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TF1_SEL, led_scene_setting[patternid].isink1_breath1_tf1_sel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TF2_SEL, led_scene_setting[patternid].isink1_breath1_tf2_sel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL, led_scene_setting[patternid].isink1_breath1_ton_sel);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL, led_scene_setting[patternid].isink1_breath1_toff_sel);
	pmic_set_register_value(PMIC_ISINK_CH0_EN, 0x01);
#endif
}
void mt_mt65xx_breath_mode_2(int freq, int duty, int trf, int ton,int toff)
{
	/* sangcheol.seo@lge.com 20140919*/
	// MT65XX_LED_PMIC_NLED_ISINK0
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_set_register_value(PMIC_CLK_DRV_ISINK0_CK_PDN, 0);
	pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);
	pmic_set_register_value(PMIC_ISINK_CH0_STEP, led_scene_setting[0].isink1_step);
	pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, duty);
	pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, freq);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL, trf);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL, ton);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL, toff);
	pmic_set_register_value(PMIC_ISINK_CH0_EN, 0x01);
#else
	pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN, 0);
	pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL, 0);
	pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);
	pmic_set_register_value(PMIC_ISINK_CH0_STEP, led_scene_setting[0].isink1_step);
	pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, duty);
	pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, freq);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL, trf);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL, ton);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL, toff);
	pmic_set_register_value(PMIC_ISINK_CH0_EN, 0x01);
#endif
}
void mt_mt65xx_led_set_to_breathmode(void)
{
	/* sangcheol.seo@lge.com 20140919*/
	// MT65XX_LED_PMIC_NLED_ISINK0
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_set_register_value(PMIC_CLK_DRV_ISINK0_CK_PDN, 0);
	pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);
	pmic_set_register_value(PMIC_ISINK_CH0_STEP, led_scene_setting[0].isink1_step);
	pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, 31);
	pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, 999);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL, 0);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL, 0);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL, 0);
	pmic_set_register_value(PMIC_ISINK_CH0_EN, 0);
#else
	pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN, 0);
	pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL, 0);
	pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);
	pmic_set_register_value(PMIC_ISINK_CH0_STEP, led_scene_setting[0].isink1_step);
	pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, 31);
	pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, 999);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL, 0);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL, 0);
	pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL, 0);
	pmic_set_register_value(PMIC_ISINK_CH0_EN, 0);
#endif
}
#endif
