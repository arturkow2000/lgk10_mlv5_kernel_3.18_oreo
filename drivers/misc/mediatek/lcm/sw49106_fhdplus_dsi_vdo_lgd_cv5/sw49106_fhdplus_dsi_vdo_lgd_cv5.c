/*
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
#include <soc/mediatek/lge/board_lge.h>
#endif

#include "dw8767.h"
#if defined(CONFIG_SW49106_FHDPLUS_DSI_VDO_LGD_CV5)
#include <linux/input/lge_touch_notify.h>
#endif

#define FRAME_WIDTH              (1080)
#define FRAME_HEIGHT             (2160)

#define PHYSICAL_WIDTH        (62)
#define PHYSICAL_HEIGHT         (124)

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#define UDELAY(n)                                             (lcm_util.udelay(n))
#define MDELAY(n)                                             (lcm_util.mdelay(n))

#define dsi_set_cmdq_V3(para_tbl, size, force_update)       lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)            lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                        lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                    lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                            lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

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

#ifndef GPIO_LCD_LDO_EN_0_1
#define GPIO_LCD_LDO_EN_0_1         (GPIO13 | 0x80000000)
#define GPIO_LCD_LDO_EN_M_GPIO   GPIO_MODE_00
#endif

#ifndef GPIO_LCD_LDO_EN_A_1
#define GPIO_LCD_LDO_EN_A_1         (GPIO21 | 0x80000000)
#endif

#include "ddp_path.h"
extern int ddp_dsi_power_on(DISP_MODULE_ENUM module, void *cmdq_handle);
extern int ddp_dsi_power_off(DISP_MODULE_ENUM module, void *cmdq_handle);

extern void chargepump_DSV_VPOS_on(void);
extern void chargepump_DSV_VNEG_on(void);
extern void chargepump_DSV_EXT_mode_on(void);
extern void chargepump_DSV_I2C_mode_on(void);
extern void dw8767_dsv_ctrl(int enable);
extern void chargepump_DSV_fast_discharge(void);
extern void chargepump_DSV_floating_mode(void);

static bool flag_deep_sleep_ctrl_available = false;
static bool flag_is_panel_deep_sleep = false;

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static LCM_setting_table_V3 lcm_initialization_setting_V3[] = {
       {0x05, 0x35, 1,   {0x00}},
       {0x15, 0x36, 1,   {0x00}},
       {0x15, 0x51, 1,   {0xFF}},
       {0x15, 0x53, 1,   {0x24}},
       {0x15, 0x55, 1,   {0x80}},
       {0x15, 0xB0, 1,   {0xAC}},
       {0x29, 0xB1, 5,   {0x46, 0x00, 0x80, 0x14, 0x85}},
       {0x29, 0xB3, 7,   {0x05, 0x08, 0x14, 0x00, 0x1C, 0x00, 0x02}},
       {0x29, 0xB4, 15,  {0x83, 0x08, 0x00, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
       {0x29, 0xB5, 18,  {0x03, 0x1E, 0x0B, 0x02, 0x29, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x24, 0x00, 0x10, 0x10, 0x10, 0x10,0x00}},
       {0x29, 0xB6, 9,   {0x00, 0x72, 0x39, 0x13, 0x08, 0x67, 0x00, 0x60, 0x46}},
       {0x29, 0xB7, 4,   {0x00, 0x50, 0x37, 0x04}},
       {0x29, 0xB8, 11,  {0x70, 0x38, 0x14, 0xED, 0x08, 0x04, 0x00, 0x01, 0xCC, 0xC8, 0x8C}},
       {0x29, 0xC0, 5,   {0x8A, 0x8F, 0x18, 0xC1, 0x12}},
       {0x29, 0xC1, 6,   {0x01, 0x00, 0x30, 0xC2, 0xC7, 0x0F}},
       {0x29, 0xC2, 2,   {0x2A, 0x00}},
       {0x29, 0xC3, 6,   {0x05, 0x0E, 0x0E, 0x50, 0x88, 0x09}},
       {0x29, 0xC4, 3,   {0xA2, 0xF3, 0xF2}},
       {0x29, 0xC5, 4,   {0xC2, 0x2A, 0x49, 0x07}},
       {0x29, 0xC6, 2,   {0x15, 0x01}},
       {0x29, 0xCA, 6,   {0x00, 0x00, 0x03, 0x84, 0x55, 0xF5}},
       {0x29, 0xCB, 2,   {0x3F, 0xA0}},
       {0x29, 0xCC, 8,   {0xF0, 0x03, 0x10, 0x55, 0x11, 0xFC, 0x34, 0x34}},
       {0x29, 0xCD, 6,   {0x11, 0x50, 0x50, 0x90, 0x00, 0xF3}},
       {0x29, 0xCE, 6,   {0xA0, 0x28, 0x28, 0x34, 0x00, 0xAB}},
       {0x29, 0xD0, 15,  {0x0D, 0x16, 0x20, 0x2C, 0x33, 0x41, 0x4B, 0x57, 0x47, 0x3C, 0x2C, 0x1C, 0x09, 0x00, 0x82}},
       {0x29, 0xD1, 15,  {0x0D, 0x16, 0x20, 0x2C, 0x33, 0x41, 0x4B, 0x57, 0x47, 0x3C, 0x2C, 0x1C, 0x09, 0x00, 0x82}},
       {0x29, 0xD2, 15,  {0x0D, 0x16, 0x20, 0x2D, 0x35, 0x42, 0x4C, 0x57, 0x47, 0x3C, 0x2C, 0x1C, 0x09, 0x00, 0x82}},
       {0x29, 0xD3, 15,  {0x0D, 0x16, 0x20, 0x2D, 0x35, 0x42, 0x4C, 0x57, 0x47, 0x3C, 0x2C, 0x1C, 0x09, 0x00, 0x82}},
       {0x29, 0xD4, 15,  {0x0D, 0x16, 0x20, 0x2B, 0x32, 0x42, 0x4C, 0x58, 0x46, 0x3C, 0x2D, 0x1D, 0x08, 0x00, 0x82}},
       {0x29, 0xD5, 15,  {0x0D, 0x16, 0x20, 0x2B, 0x32, 0x42, 0x4C, 0x58, 0x46, 0x3C, 0x2D, 0x1D, 0x08, 0x00, 0x82}},
       {0x29, 0xE5, 12,  {0x24, 0x23, 0x11, 0x10, 0x00, 0x0A, 0x08, 0x06, 0x04, 0x11, 0x0E, 0x23}},
       {0x29, 0xE6, 12,  {0x24, 0x23, 0x11, 0x10, 0x01, 0x0B, 0x09, 0x07, 0x05, 0x11, 0x0E, 0x23}},
       {0x29, 0xE7, 6,   {0x15, 0x16, 0x17, 0x18, 0x19, 0x1A}},
       {0x29, 0xE8, 6,   {0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20}},
       {0x29, 0xED, 4,   {0x00, 0x01, 0x53, 0x0C}},
       {0x29, 0xF0, 2,   {0x82, 0x00}},
       {0x29, 0xF2, 4,   {0x01, 0x00, 0x17, 0x00}},
       {0x29, 0xF3, 6,   {0x00, 0x56, 0x96, 0xCF, 0x00, 0x01}},
       {0x29, 0xB2, 3,   {0x77, 0x48, 0x4C}}, //bias current ratio 1.25
};

/* It should be synced with lcm_initialization_setting_V3 */
static LCM_setting_table_V3 lcm_lpwg_on_setting_V3[] = {
       {0x15, 0xB0, 1,   {0xAC}},
       {0x29, 0xC1, 6,   {0x01, 0x00, 0x30, 0xC2, 0xC7, 0x0F}},
       {0x29, 0xC3, 6,   {0x05, 0x0E, 0x0E, 0x50, 0x88, 0x09}},
       {0x29, 0xC6, 2,   {0x15, 0x01}},
       {0x29, 0xCC, 8,   {0xF0, 0x03, 0x10, 0x55, 0x11, 0xFC, 0x34, 0x34}},
       {0x29, 0xCE, 6,   {0xA0, 0x28, 0x28, 0x34, 0x00, 0xAB}},
       {0x29, 0xE5, 12,  {0x24, 0x23, 0x11, 0x10, 0x00, 0x0A, 0x08, 0x06, 0x04, 0x11, 0x0E, 0x23}},
       {0x29, 0xE6, 12,  {0x24, 0x23, 0x11, 0x10, 0x01, 0x0B, 0x09, 0x07, 0x05, 0x11, 0x0E, 0x23}},
       {0x29, 0xED, 4,   {0x00, 0x01, 0x53, 0x0C}},
};

static LCM_setting_table_V3 lcm_initialization_setting_V3_Exit_Sleep[] = {
	{0x05, 0x11, 1, {0x00}},
};

static LCM_setting_table_V3 lcm_initialization_setting_V3_Disp_On[] = {
	{0x05, 0x29, 1, {0x00}},
};

static LCM_setting_table_V3 lcm_initialization_setting_V3_Display_Off[] = {
	{0x05, 0x28, 1, {0x00}},
};

static LCM_setting_table_V3 lcm_initialization_setting_V3_Enter_Sleep[] = {
	{0x05, 0x10, 1, {0x00}},
};

static struct LCM_setting_table __attribute__ ((unused)) lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
};


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy((void*)&lcm_util, (void*)util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS * params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// physical size
	params->physical_width = PHYSICAL_WIDTH;
	params->physical_height = PHYSICAL_HEIGHT;

	params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_EVENT_VDO_MODE; //BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE;
	// enable tearing-free
	params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM                    = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format          = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;
	//params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.cont_clock = 1;

	params->dsi.vertical_sync_active = 1;
	params->dsi.vertical_backporch = 92;
	params->dsi.vertical_frontporch = 170;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 4;
	params->dsi.horizontal_backporch				= 8;
	params->dsi.horizontal_frontporch				= 8;
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 520;
	params->dsi.ssc_disable	= 1;
}

static void init_lcm_registers(void)
{
	LCM_PRINT("[LCD] %s : +\n", __func__);
	dsi_set_cmdq_V3(lcm_initialization_setting_V3_Exit_Sleep, sizeof(lcm_initialization_setting_V3_Exit_Sleep) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(100);
	dsi_set_cmdq_V3(lcm_initialization_setting_V3_Disp_On, sizeof(lcm_initialization_setting_V3_Disp_On) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(5);
	LCM_PRINT("[LCD] %s : -\n", __func__);
}

static void init_lcm_registers_sleep(void)
{
	LCM_PRINT("[LCD] %s : +\n", __func__);
	dsi_set_cmdq_V3(lcm_initialization_setting_V3_Display_Off, sizeof(lcm_initialization_setting_V3_Display_Off) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(50);
	dsi_set_cmdq_V3(lcm_initialization_setting_V3_Enter_Sleep, sizeof(lcm_initialization_setting_V3_Enter_Sleep) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(120);
	LCM_PRINT("[LCD] %s : -\n", __func__);
}

static void touch_reset_pin (int mode)
{
    mt_set_gpio_mode(GPIO_TOUCH_RESET, GPIO_TOUCH_RESET_M_GPIO);
    mt_set_gpio_dir(GPIO_TOUCH_RESET, GPIO_DIR_OUT);

    if(mode == 1)
    {
        mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ONE);
    }
    else if(mode == 0)
    {
        mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ZERO);
    }
}

/* VCAMD 1.8v LDO enable */
static void ldo_1v8io_on(void)
{
	hw_rev_type hw_rev = lge_get_board_revno();

        if(hw_rev >= HW_REV_1_0)
        {
                mt_set_gpio_mode(GPIO_LCD_LDO_EN_A_1, GPIO_LCD_LDO_EN_M_GPIO);
                mt_set_gpio_dir(GPIO_LCD_LDO_EN_A_1, GPIO_DIR_OUT);
                mt_set_gpio_out(GPIO_LCD_LDO_EN_A_1, GPIO_OUT_ONE);
	}
        else if(hw_rev >= HW_REV_C)
                return;
        else if(hw_rev >= HW_REV_A)
        {
                mt_set_gpio_mode(GPIO_LCD_LDO_EN_A_1, GPIO_LCD_LDO_EN_M_GPIO);
                mt_set_gpio_dir(GPIO_LCD_LDO_EN_A_1, GPIO_DIR_OUT);
                mt_set_gpio_out(GPIO_LCD_LDO_EN_A_1, GPIO_OUT_ONE);
        }
        else if(hw_rev == HW_REV_0_1)
        {
                mt_set_gpio_mode(GPIO_LCD_LDO_EN_0_1, GPIO_LCD_LDO_EN_M_GPIO);
                mt_set_gpio_dir(GPIO_LCD_LDO_EN_0_1, GPIO_DIR_OUT);
                mt_set_gpio_out(GPIO_LCD_LDO_EN_0_1, GPIO_OUT_ONE);
        }
        else
        {
                pmic_set_register_value(PMIC_LDO_VCN18_EN, 1);
        }
}

/* VCAMD 1.8v LDO disable */
static void ldo_1v8io_off(void)
{
        hw_rev_type hw_rev = lge_get_board_revno();

        if(hw_rev >= HW_REV_1_0)
        {
                mt_set_gpio_mode(GPIO_LCD_LDO_EN_A_1, GPIO_LCD_LDO_EN_M_GPIO);
                mt_set_gpio_dir(GPIO_LCD_LDO_EN_A_1, GPIO_DIR_OUT);
                mt_set_gpio_out(GPIO_LCD_LDO_EN_A_1, GPIO_OUT_ZERO);
        }
        else if(hw_rev >= HW_REV_C)
                return;
        else if(hw_rev >= HW_REV_A)
        {
                mt_set_gpio_mode(GPIO_LCD_LDO_EN_A_1, GPIO_LCD_LDO_EN_M_GPIO);
                mt_set_gpio_dir(GPIO_LCD_LDO_EN_A_1, GPIO_DIR_OUT);
                mt_set_gpio_out(GPIO_LCD_LDO_EN_A_1, GPIO_OUT_ZERO);
        }
        else if(hw_rev == HW_REV_0_1)
        {
                mt_set_gpio_mode(GPIO_LCD_LDO_EN_0_1, GPIO_LCD_LDO_EN_M_GPIO);
                mt_set_gpio_dir(GPIO_LCD_LDO_EN_0_1, GPIO_DIR_OUT);
                mt_set_gpio_out(GPIO_LCD_LDO_EN_0_1, GPIO_OUT_ONE);
        }
        else
        {
                pmic_set_register_value(PMIC_LDO_VCN18_EN, 0);
        }
}

static void ldo_p5m5_dsv_5v5_off(void)
{
#if defined(BUILD_LK)
    chargepump_DSV_off();
#else
    dw8767_dsv_ctrl(0);
#endif
}

static void reset_lcd_module(unsigned char reset)
{
    mt_set_gpio_mode(GPIO_LCM_RST, GPIO_LCM_RST_M_GPIO);
    mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);

    if(reset)
	    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    else
	    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);

    LCM_PRINT("LCD Reset %s \n",(reset)? "High":"Low");
}

static void lcm_init(void)
{
	LCM_PRINT("[LCD] %s : +\n", __func__);
#if defined(CONFIG_SW49106_FHDPLUS_DSI_VDO_LGD_CV5)
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
#endif
	reset_lcd_module(0);

	ldo_1v8io_on();
        MDELAY(15);

        chargepump_DSV_VPOS_on();
        MDELAY(3);

        reset_lcd_module(1);
#if defined(CONFIG_SW49106_FHDPLUS_DSI_VDO_LGD_CV5)
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);
#endif
        MDELAY(12);
        dsi_set_cmdq_V3(lcm_initialization_setting_V3, sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(1);

        chargepump_DSV_VNEG_on();
        MDELAY(5);

        init_lcm_registers();
	LCM_PRINT("[LCD] %s : -\n", __func__);
}

static void lcm_suspend_mfts(void)
{
    LCM_PRINT("[LCD] %s : +\n", __func__);
    MDELAY(6);
#if defined(CONFIG_SW49106_FHDPLUS_DSI_VDO_LGD_CV5)
        touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
#endif
    reset_lcd_module(0);
    MDELAY(2);
    LCM_PRINT("[LCD] %s : -\n", __func__);
}
static void lcm_suspend(void)
{
    chargepump_DSV_EXT_mode_on();
    init_lcm_registers_sleep();
    flag_deep_sleep_ctrl_available = true;
    LCM_PRINT("[LCD] lcm_suspend \n");
}

void lge_panel_enter_deep_sleep(void)
{
    if(flag_deep_sleep_ctrl_available)
    {
	MDELAY(6);
	chargepump_DSV_fast_discharge();
	MDELAY(10);
	chargepump_DSV_floating_mode();
	MDELAY(1);
	touch_reset_pin(0);
	MDELAY(1);
	reset_lcd_module(0);
	MDELAY(2);
	ldo_1v8io_off();
	MDELAY(1);
	flag_is_panel_deep_sleep = true;
	LCM_PRINT("[LCD] %s\n", __func__);
    }
}

void lge_panel_exit_deep_sleep(void)
{
    if(flag_deep_sleep_ctrl_available)
    {
	ddp_path_top_clock_on();
	ddp_dsi_power_on(DISP_MODULE_DSI0, NULL);
	ldo_1v8io_on();
	MDELAY(2);
	chargepump_DSV_VPOS_on();
	MDELAY(2);
	reset_lcd_module(1);
	MDELAY(6);
	touch_reset_pin(1);
	MDELAY(7);
	dsi_set_cmdq_V3(lcm_lpwg_on_setting_V3, sizeof(lcm_lpwg_on_setting_V3) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(50);
	chargepump_DSV_EXT_mode_on();
	ddp_dsi_power_off(DISP_MODULE_DSI0, NULL);
	ddp_path_top_clock_off();
	flag_is_panel_deep_sleep = false;
	LCM_PRINT("[LCD] %s\n", __func__);
    }
}

static void lcm_resume_mfts(void)
{
	ldo_1v8io_on();
        MDELAY(15);
        chargepump_DSV_VPOS_on();
        MDELAY(3);
}

static void lcm_resume(void)
{
    flag_deep_sleep_ctrl_available = false;
    if(flag_is_panel_deep_sleep)
    {
	lcm_init();
    }
    else
    {
#if defined(CONFIG_SW49106_FHDPLUS_DSI_VDO_LGD_CV5)
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
#endif
	reset_lcd_module(0);
	MDELAY(1);
	reset_lcd_module(1);
#if defined(CONFIG_SW49106_FHDPLUS_DSI_VDO_LGD_CV5)
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);
#endif
	MDELAY(7);
	chargepump_DSV_I2C_mode_on();
	MDELAY(1);
	dsi_set_cmdq_V3(lcm_initialization_setting_V3, sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);
	init_lcm_registers();
    }
    LCM_PRINT("[LCD] lcm_resume \n");
}


static void lcm_shutdown(void)
{
    MDELAY(6);
    ldo_p5m5_dsv_5v5_off();
    MDELAY(10);
    ldo_1v8io_off();

    LCM_PRINT("[LCD] lcm_shutdown \n");
}

LCM_DRIVER sw49106_fhdplus_dsi_vdo_lgd_cv5_drv = {
    .name = "LGD-SW49106",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
	.suspend_mfts = lcm_suspend_mfts,
	.resume_mfts = lcm_resume_mfts,
    .shutdown = lcm_shutdown,
};
