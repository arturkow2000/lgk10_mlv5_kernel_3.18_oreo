/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2016
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifdef BUILD_LK
#include <string.h>
#include <mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "upmu_common.h"
#include <linux/string.h>
#include "mt_gpio.h"
#include <mach/gpio_const.h>
#endif
#include "lcm_drv.h"
#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#if defined(BUILD_LK)
#include <boot_mode.h>
#else
#include <linux/types.h>
#include <upmu_hw.h>
#if defined CONFIG_LEDS_RT4832_K6PNA
#include "rt4832.h"
#endif
#endif
/* #include <soc/mediatek/lge/board_lge.h> */
#include <linux/input/lge_touch_notify.h>
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
/* pixel */
#define FRAME_WIDTH			(720)
#define FRAME_HEIGHT			(1280)
/* physical dimension */
#define PHYSICAL_WIDTH        (62)
#define PHYSICAL_HEIGHT         (110)
#define LCM_ID       (0xb9)
#define LCM_DSI_CMD_MODE		0
#define REGFLAG_DELAY 0xAB
#define REGFLAG_END_OF_TABLE 0xAA	/* END OF REGISTERS MARKER */
/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */
static LCM_UTIL_FUNCS lcm_util = { 0 };
#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n)	(lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmdq_V3(para_tbl, size, force_update)\
lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)\
lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)\
lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)\
lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)\
lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)\
lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)\
lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
static unsigned int need_set_lcm_addr = 1;
#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST         (GPIO158 | 0x80000000)
#define GPIO_LCM_RST_M_GPIO   GPIO_MODE_00
#define GPIO_LCM_RST_M_LCM_RST   GPIO_MODE_01
#endif
#ifndef GPIO_TOUCH_RESET
#define GPIO_TOUCH_RESET         (GPIO10 | 0x80000000)
#define GPIO_TOUCH_RESET_M_GPIO   GPIO_MODE_00
#define GPIO_TOUCH_RESET_M_LCM_RST   GPIO_MODE_01
#endif
#ifndef GPIO_DSV_ENP_EN
#define GPIO_DSV_ENP_EN (GPIO11 | 0x80000000)
#define GPIO_DSV_ENP_EN_M_GPIO GPIO_MODE_00
#define GPIO_DSV_ENP_EN_M_KROW GPIO_MODE_06
#define GPIO_DSV_ENP_EN_M_PWM GPIO_MODE_05
#endif
#ifndef GPIO_DSV_ENN_EN
#define GPIO_DSV_ENN_EN (GPIO12 | 0x80000000)
#define GPIO_DSV_ENN_EN_M_GPIO GPIO_MODE_00
#define GPIO_DSV_ENN_EN_M_KROW GPIO_MODE_06
#define GPIO_DSV_ENN_EN_M_PWM GPIO_MODE_05
#endif
#ifndef GPIO_TOUCH_EN
#define GPIO_TOUCH_EN         (GPIO43 | 0x80000000)
#define GPIO_TOUCH_EN_M_GPIO   GPIO_MODE_00
#define GPIO_TOUCH_EN_M_LCM_RST   GPIO_MODE_01
#endif
/* extern u16 pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname, u32 val); */
/* extern void rt4832_dsv_ctrl(int enable); */
#define LGE_LPWG_SUPPORT
struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};
static LCM_setting_table_V3 lg4894_initial_command[] = {
	{0x15, 0xB0, 1, {0xAC} },
	{0x39, 0xB1, 4, {0x10, 0x30, 0x16, 0x00} },
	{0x39, 0xB2, 12, {0x01, 0x00, 0xA1, 0x20, 0x58, 0x00, 0xCF, 0x10, 0x1F, 0x1F, 0x28, 0x28} },
	{0x39, 0xB4, 3, {0x00, 0x9F, 0x00} },
	{0x39, 0xB5, 5, {0x42, 0xC0, 0x80, 0x00, 0x00} },
	{0x39, 0xB6, 3, {0x77, 0x34, 0x48} },
	{0x39, 0xBD, 9, {0xD0, 0x02, 0x18, 0x02, 0x04, 0x04, 0x12, 0x20, 0x22} },
	{0x39, 0xBE, 7, {0xE0, 0xE0, 0xC9, 0xC9, 0xC8, 0xF8, 0x00} },
	{0x39, 0xC1, 6, {0x01, 0xE8, 0xD0, 0xC2, 0xC1, 0x00} },
	{0x39, 0xC2, 3, {0x66, 0x13, 0x13} },
	{0x39, 0xC3, 6, {0x15, 0x2F, 0x2F, 0x00, 0xA6, 0x21} },
	{0x39, 0xC4, 3, {0x51, 0x00, 0x37} },
	{0x39, 0xC5, 5, {0x25, 0x20, 0x20, 0x14, 0x14} },
	{0x39, 0xC7, 12, {0x10, 0x22, 0x00, 0x28, 0x00, 0x62, 0x62, 0x62, 0x00, 0xB1, 0x01, 0x00} },
	{0x39, 0xD0, 12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x51, 0x00, 0x00, 0x00, 0x42, 0x46, 0x03} },
	{0x39, 0xD1, 12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x51, 0x00, 0x00, 0x00, 0x42, 0x46, 0x03} },
	{0x39, 0xD2, 12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x51, 0x00, 0x00, 0x00, 0x42, 0x46, 0x03} },
	{0x39, 0xD3, 12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x51, 0x00, 0x00, 0x00, 0x42, 0x46, 0x03} },
	{0x39, 0xD4, 12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x51, 0x00, 0x00, 0x00, 0x42, 0x46, 0x03} },
	{0x39, 0xD5, 12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x51, 0x00, 0x00, 0x00, 0x42, 0x46, 0x03} },
	{0x39, 0xE4, 21,
	 {0x2F, 0xC3, 0x2F, 0x60, 0x41, 0xC5, 0xC6, 0xCC, 0xCC, 0xCB, 0x2F, 0xCB, 0xCA, 0xCA, 0xC9,
	  0xC9, 0xC7, 0xC8, 0x2F, 0x41, 0x2F} },
	{0x39, 0xE5, 12, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03} },
	{0x05, 0x11, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};
static LCM_setting_table_V3 lcm_initial_disp_on[] = {
	{0x05, 0x29, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};
static LCM_setting_table_V3 lcm_initial_for_sleep_in[] = {
	{0x05, 0x28, 1, {0x00} },
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 100, {} },
	{0x05, 0x10, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};
static struct LCM_setting_table __attribute__ ((unused)) lcm_backlight_level_setting[] = {
	{
		0x51, 1, {
	0xFF} }, {
		REGFLAG_END_OF_TABLE, 0x00, {
	} }
};
/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy((void *)&lcm_util, (void *)util, sizeof(LCM_UTIL_FUNCS));
}
static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	/* physical size */
	params->physical_width = PHYSICAL_WIDTH;
	params->physical_height = PHYSICAL_HEIGHT;
	params->dsi.mode = BURST_VDO_MODE;	/* SYNC_EVENT_VDO_MODE; //BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE; */
	/* enable tearing-free */
	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* params->dsi.intermediat_buffer_num = 0; */
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.cont_clock = 1;
	params->dsi.vertical_sync_active = 1;
	params->dsi.vertical_backporch = 20;
	params->dsi.vertical_frontporch = 536;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 16;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.PLL_CLOCK = 280;
}
static void init_lcm_registers(void)
{
	dsi_set_cmdq_V3(lg4894_initial_command,
			sizeof(lg4894_initial_command) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(120);
	dsi_set_cmdq_V3(lcm_initial_disp_on,
			sizeof(lcm_initial_disp_on) / sizeof(LCM_setting_table_V3), 1);
}
static void init_lcm_registers_sleep(void)
{
	dsi_set_cmdq_V3(lcm_initial_for_sleep_in,
			sizeof(lcm_initial_for_sleep_in) / sizeof(LCM_setting_table_V3), 1);
	/* MDELAY(150); */
	LCM_PRINT("[LCD] init_lcm_registers_sleep\n");
}
#if 0				/* no need */
static void init_lcm_registers_sleep_2nd(void)
{
	/* unsigned int data_array[1]; */
	/* dsi_set_cmdq_V3(lcm_initialization_for_sleep_in_post,
	sizeof(lcm_initialization_for_sleep_in_post) / sizeof(LCM_setting_table_V3), 1); */
	LCM_PRINT("[LCD] init_lcm_registers_sleep 2nd\n");
}
#endif
/* 1.8v LDO is always on */
static void ldo_1v8io_on(void)
{
#if 1
	pmic_set_register_value(PMIC_RG_VGP3_VOSEL, 6);
	pmic_set_register_value(PMIC_RG_VGP3_EN, 1);
#else
	hwPowerOn(MT6351_POWER_LDO_VGP3, VOL_1800, "LCD");
#endif
}
/* 1.8v LDO is always on */
static void ldo_1v8io_off(void)
{
#if 1
	pmic_set_register_value(PMIC_RG_VGP3_EN, 0);
#else
	hwPowerDown(MT6351_POWER_LDO_VGP3, "LCD");
#endif
}
/* VGP1 3.0v LDO enable */
static void ldo_3v0_on(void)
{
#if 0
	mt6328_set_register_value(PMIC_RG_VGP1_VOSEL, 6);
	mt6328_set_register_value(PMIC_RG_VGP1_EN, 1);
#else
/* hwPowerOn(MT6328_POWER_LDO_VGP1, VOL_3000, "LCD"); */
	mt_set_gpio_mode(GPIO_TOUCH_EN, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_TOUCH_EN, GPIO_OUT_ONE);
#endif
}
/* VGP1 3.0v LDO disable */
static void ldo_3v0_off(void)
{
#if 0
	mt6328_set_register_value(PMIC_RG_VGP1_EN, 0);
#else
/* hwPowerDown(MT6328_POWER_LDO_VGP1, "LCD"); */
	mt_set_gpio_mode(GPIO_TOUCH_EN, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_TOUCH_EN, GPIO_OUT_ZERO);
#endif
}
/* DSV power +5.8V,-5.8v */
static void ldo_p5m5_dsv_5v5_on(void)
{
	rt4832_dsv_ctrl(1);
}
/* DSV power +5.8V,-5.8v */
static void ldo_p5m5_dsv_5v5_off(void)
{
	rt4832_dsv_ctrl(0);
}
static void reset_lcd_module(unsigned char reset)
{
	mt_set_gpio_mode(GPIO_LCM_RST, GPIO_LCM_RST_M_GPIO);
	/* mt_set_gpio_pull_enable(GPIO_LCM_RST, GPIO_PULL_ENABLE); */
	mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
	if (reset) {
		mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
		LCM_PRINT("[LCD] Reset High\n");
	} else {
		mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
		LCM_PRINT("[LCD] Reset Low\n");
	}
}
#if 0
static void touch_reset_pin(int mode)
{
	mt_set_gpio_mode(GPIO_TOUCH_RESET, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_RESET, GPIO_DIR_OUT);
	if (mode == 1)
		mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ONE);
	else if (mode == 0)
		mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ZERO);
}
#endif
static void lcm_init(void)
{
	ldo_p5m5_dsv_5v5_on();
	MDELAY(5);		/* min 5ms */
	reset_lcd_module(1);
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);
	MDELAY(5);		/* min 5ms */
	init_lcm_registers();
	MDELAY(50);
	need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_init\n");
}
static void lcm_suspend(void)
{
	init_lcm_registers_sleep();
#if 1
#else
	ldo_p5m5_dsv_5v5_off();
	MDELAY(10);
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
	reset_lcd_module(0);
	MDELAY(10);
	ldo_3v0_off();
	ldo_1v8io_off();
	MDELAY(10);
#endif
	LCM_PRINT("[LCD] lcm_suspend\n");
}
static void lcm_resume_power(void)
{
#if 1
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
	reset_lcd_module(0);
	ldo_1v8io_on();
	ldo_3v0_on();
	MDELAY(5);
#else
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
	reset_lcd_module(0);
	MDELAY(10);
	ldo_p5m5_dsv_5v5_off();
	/* ldo_3v0_off(); */
	/* ldo_1v8io_off(); */
	MDELAY(20);
	ldo_1v8io_on();
	ldo_3v0_on();
	MDELAY(10);		/* Hiz or DDVDH>=VPNL for min 10ms */
#endif
	LCM_PRINT("[LCD] lcm_resume_power\n");
}
static void lcm_resume(void)
{
	lcm_init();
	need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_resume\n");
}
#if 0				/* moved to lcm_suspend */
static void lcm_shutdown(void)
{
	ldo_p5m5_dsv_5v5_off();
	MDELAY(10);
	reset_lcd_module(0);
	MDELAY(10);
	ldo_3v0_off();
	ldo_1v8io_off();
	MDELAY(10);
	LCM_PRINT("[LCD] lcm_shutdown\n");
}
#endif
#if 0
static void lcm_esd_recover(void)
{
	lcm_suspend();
	lcm_resume();
	LCM_PRINT("[LCD] lcm_esd_recover\n");
}
#endif
/* --------------------------------------------------------------------------- */
/* Get LCM Driver Hooks */
/* --------------------------------------------------------------------------- */
LCM_DRIVER lg4894_hd720_dsi_vdo_lcm_drv = {
	.name = "LGD_LG4894",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.resume_power = lcm_resume_power,
/*	.suspend_2nd_cmd = init_lcm_registers_sleep_2nd, */
/*	.shutdown = lcm_shutdown, */
};