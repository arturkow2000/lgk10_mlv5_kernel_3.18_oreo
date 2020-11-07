/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
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
#endif
#include <linux/input/lge_touch_notify.h>
#include <soc/mediatek/lge/board_lge.h>

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

// pixel
#define FRAME_WIDTH             (720)
#define FRAME_HEIGHT            (1280)

// physical dimension
#define PHYSICAL_WIDTH          (66)
#define PHYSICAL_HEIGHT         (117)

#define LCM_ID                  (0xb9)
#define LCM_DSI_CMD_MODE        0

#define REGFLAG_DELAY           0xAB
#define REGFLAG_END_OF_TABLE    0xAA // END OF REGISTERS MARKER

#define LGE_LPWG_SUPPORT

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};
static unsigned int need_set_lcm_addr = 1;

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V3(para_tbl, size, force_update)       lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                       lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

extern void rt4832_dsv_ctrl(int enable);
extern void lm3632_dsv_ctrl(int enable);
//extern void chargepump_DSV_on(void);
//extern void chargepump_DSV_off(void);
//extern void SetNextState_For_Touch(int lcdState);

/* touch irq handle according to display power drop in lv5 */
extern bool mfts_check_shutdown;

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

// ---------------------------------------------------------------------------
//  GPIO Set
// ---------------------------------------------------------------------------

#ifndef GPIO_TOUCH_RESET
#define GPIO_TOUCH_RESET            (GPIO10 | 0x80000000)
#define GPIO_TOUCH_RESET_M_GPIO     GPIO_MODE_00
#endif

#ifndef GPIO_TOUCH_EN
#define GPIO_TOUCH_EN               (GPIO43 | 0x80000000)
#define GPIO_TOUCH_EN_M_GPIO        GPIO_MODE_00
#endif

#ifndef GPIO_LCD_LDO_EN
#define GPIO_LCD_LDO_EN             (GPIO16 | 0x80000000)
#define GPIO_LCD_LDO_EN_M_GPIO      GPIO_MODE_00
#endif

#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST                (GPIO158 | 0x80000000)
#define GPIO_LCM_RST_M_GPIO         GPIO_MODE_00
#endif

// ---------------------------------------------------------------------------
//  command set
// ---------------------------------------------------------------------------

static LCM_setting_table_V3 lcm_initialization_setting_V3_Exit_Sleep[] = {
    {0x05, 0x11,    1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_initialization_setting_V3_Disp_On[] = {
    {0x05, 0x29,    1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_initialization_setting_V3_Display_Off[] = {
    {0x05, 0x28,    1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_initialization_setting_V3_Enter_Sleep[] = {
    {0x05, 0x10,    1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static struct LCM_setting_table __attribute__ ((unused)) lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if defined(CONFIG_LGE_READER_MODE)
static LCM_setting_table_V3 lcm_reader_mode_off[] = {
    //reader_mode off command
    {0x15, 0x84, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};
static LCM_setting_table_V3 lcm_reader_mode_low[] = {
    //6200K
    {0x15, 0x84, 1, {0x50}},   //cold setting(temp)
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};
static LCM_setting_table_V3 lcm_reader_mode_mid[] = {
    //5500K
    {0x15, 0x84, 1, {0x70}}, //neutral
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};
static LCM_setting_table_V3 lcm_reader_mode_high[] = {
    //5300K
    {0x15, 0x84, 1, {0x7F}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};
static LCM_setting_table_V3 lcm_reader_mode_mono[] = {
    //5500K
    {0x15, 0x84, 1, {0x70}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static void lcm_reader_mode(LCM_READER_MODE mode)
{
    switch(mode)
    {
        case READER_MODE_OFF:
            dsi_set_cmdq_V3(lcm_reader_mode_off,sizeof(lcm_reader_mode_off)/sizeof(LCM_setting_table_V3),1);
            break;
        case READER_MODE_LOW:
            dsi_set_cmdq_V3(lcm_reader_mode_low,sizeof(lcm_reader_mode_low)/sizeof(LCM_setting_table_V3),1);
            break;
        case READER_MODE_MID:
            dsi_set_cmdq_V3(lcm_reader_mode_mid,sizeof(lcm_reader_mode_mid)/sizeof(LCM_setting_table_V3),1);
            break;
        case READER_MODE_HIGH:
            dsi_set_cmdq_V3(lcm_reader_mode_high,sizeof(lcm_reader_mode_high)/sizeof(LCM_setting_table_V3),1);
            break;
        case READER_MODE_MONO:
            dsi_set_cmdq_V3(lcm_reader_mode_mono,sizeof(lcm_reader_mode_mono)/sizeof(LCM_setting_table_V3),1);
            break;
        default:
            break;
    }

    LCM_PRINT("READER MODE : %d\n", mode);
}
#endif

#ifdef CONFIG_LGE_COMFORT_VIEW
#define COMFORT_VIEW_CMD_CNT 5
#define COMFORT_VIEW_OFF_CMD_CNT 6

static LCM_setting_table_V3 lcm_comfort_view_off[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x39, 0xC8,  55, {0x01, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0xFC, 0x00, 0x00, 0x04, 0xFE, 0x03, 0xFC, 0x00, 0x00, 0x04, 0xFE, 0x03, 0xF8, 0x00, 0x00,
			           0xFE, 0x02, 0xFB, 0x00, 0x00, 0x00, 0x04, 0xFE, 0x03, 0xFC, 0x00, 0x00, 0x04, 0xFE, 0x02, 0xF3,
			           0x00, 0x00, 0xFF, 0x01, 0xFA, 0x84, 0x00}},
	{0x39, 0xC9,  19, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0xFC, 0x00}},
	{0x39, 0xCA,   4, {0x1C, 0xFC, 0xFC, 0xFC}},
	{0x15, 0xB0,   1, {0x03}},
	{0x15, 0x84,   1, {0x00}},
};

// 7300K
static LCM_setting_table_V3 lcm_comfort_view_step_1[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x15, 0xC8,   1, {0x11}},
	{0x39, 0xC9,  18, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0x85}},
	{0x39, 0xCA,   4, {0x1D, 0xFC, 0xFC, 0xFC}},
	{0x15, 0xB0,   1, {0x03}},
};

// 7000K
static LCM_setting_table_V3 lcm_comfort_view_step_2[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x15, 0xC8,   1, {0x11}},
	{0x39, 0xC9,  18, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x99, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0x70}},
	{0x39, 0xCA,   4, {0x1D, 0xFC, 0xFC, 0xFC}},
	{0x15, 0xB0,   1, {0x03}},
};

// 6650K
static LCM_setting_table_V3 lcm_comfort_view_step_3[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x15, 0xC8,   1, {0x11}},
	{0x39, 0xC9,  18, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x91, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0x4F}},
	{0x39, 0xCA,   4, {0x1D, 0xFC, 0xFC, 0xFC}},
	{0x15, 0xB0,   1, {0x03}},
};

// 6300K
static LCM_setting_table_V3 lcm_comfort_view_step_4[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x15, 0xC8,   1, {0x11}},
	{0x39, 0xC9,  18, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0x2E}},
	{0x39, 0xCA,   4, {0x1D, 0xFC, 0xFC, 0xFC}},
	{0x15, 0xB0,   1, {0x03}},
};

// 6000K
static LCM_setting_table_V3 lcm_comfort_view_step_5[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x15, 0xC8,   1, {0x11}},
	{0x39, 0xC9,  18, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0x12}},
	{0x39, 0xCA,   4, {0x1D, 0xFC, 0xFC, 0xFC}},
	{0x15, 0xB0,   1, {0x03}},
};

// 5650K
static LCM_setting_table_V3 lcm_comfort_view_step_6[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x15, 0xC8,   1, {0x11}},
	{0x39, 0xC9,  18, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0x00}},
	{0x39, 0xCA,   4, {0x1D, 0xFC, 0xF4, 0xDA}},
	{0x15, 0xB0,   1, {0x03}},
};

// 5300K
static LCM_setting_table_V3 lcm_comfort_view_step_7[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x15, 0xC8,   1, {0x11}},
	{0x39, 0xC9,  18, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0x00}},
	{0x39, 0xCA,   4, {0x1D, 0xFC, 0xDB, 0xA1}},
	{0x15, 0xB0,   1, {0x03}},
};

// 5000K
static LCM_setting_table_V3 lcm_comfort_view_step_8[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x15, 0xC8,   1, {0x11}},
	{0x39, 0xC9,  18, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0x00}},
	{0x39, 0xCA,   4, {0x1D, 0xFC, 0xC9, 0x71}},
	{0x15, 0xB0,   1, {0x03}},
};

// 4800K
static LCM_setting_table_V3 lcm_comfort_view_step_9[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x15, 0xC8,   1, {0x11}},
	{0x39, 0xC9,  18, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0x00}},
	{0x39, 0xCA,   4, {0x1D, 0xFC, 0xBB, 0x4D}},
	{0x15, 0xB0,   1, {0x03}},
};

// 4650K
static LCM_setting_table_V3 lcm_comfort_view_step_10[] = {
	{0x15, 0xB0,   1, {0x04}},
	{0x15, 0xC8,   1, {0x11}},
	{0x39, 0xC9,  18, {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00,
			           0x00, 0x00}},
	{0x39, 0xCA,   4, {0x1D, 0xFC, 0xAF, 0x2B}},
	{0x15, 0xB0,   1, {0x03}},
};

// step 0 ~ 10
LCM_setting_table_V3 *lcm_comfort_view_cmd_tovis[] = {
    lcm_comfort_view_off,
    lcm_comfort_view_step_1,lcm_comfort_view_step_2,
    lcm_comfort_view_step_3,lcm_comfort_view_step_4,
    lcm_comfort_view_step_5,lcm_comfort_view_step_6,
    lcm_comfort_view_step_7,lcm_comfort_view_step_8,
    lcm_comfort_view_step_9,lcm_comfort_view_step_10,
};

static void lcm_comfort_view(unsigned int mode)
{
	unsigned int len = COMFORT_VIEW_CMD_CNT;

	if(!mode)
		len = COMFORT_VIEW_OFF_CMD_CNT;

	dsi_set_cmdq_V3(lcm_comfort_view_cmd_tovis[mode], len, 1);

    LCM_PRINT("[LCD] %s : %d\n",__func__,mode);
}
#endif

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy((void*)&lcm_util, (void*)util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS * params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type = LCM_TYPE_DSI;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // physical size
    params->physical_width = PHYSICAL_WIDTH;
    params->physical_height = PHYSICAL_HEIGHT;

    params->dsi.mode = SYNC_PULSE_VDO_MODE;
    // enable tearing-free
    params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    params->dsi.packet_size = 256;
    //params->dsi.intermediat_buffer_num = 0;

    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.cont_clock = 1;

    params->dsi.vertical_sync_active = 127; //20150824 , 1->2
    params->dsi.vertical_backporch = 127;
    params->dsi.vertical_frontporch = 127;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 4;
    params->dsi.horizontal_backporch = 32;
    params->dsi.horizontal_frontporch = 78;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 272;//for 60fps
    //params->dsi.ssc_disable   = 1;
}

static void init_lcm_registers(void)
{
    dsi_set_cmdq_V3(lcm_initialization_setting_V3_Exit_Sleep, sizeof(lcm_initialization_setting_V3_Exit_Sleep) / sizeof(LCM_setting_table_V3), 1);
    MDELAY(120);

    dsi_set_cmdq_V3(lcm_initialization_setting_V3_Disp_On, sizeof(lcm_initialization_setting_V3_Disp_On) / sizeof(LCM_setting_table_V3), 1);

    LCM_PRINT("[LCD] init_lcm_registers \n");
}



static void init_lcm_registers_sleep(void)
{
    dsi_set_cmdq_V3(lcm_initialization_setting_V3_Display_Off, sizeof(lcm_initialization_setting_V3_Display_Off) / sizeof(LCM_setting_table_V3), 1);
    MDELAY(20);

    dsi_set_cmdq_V3(lcm_initialization_setting_V3_Enter_Sleep, sizeof(lcm_initialization_setting_V3_Enter_Sleep) / sizeof(LCM_setting_table_V3), 1);
    MDELAY(50);
#if 0
    LCM_PRINT("[LCD][TOUCH] SetNextState_Touch called.....here.......start\n");
    SetNextState_For_Touch(0);
    LCM_PRINT("[LCD][TOUCH] SetNextState_Touch called.....here.......end\n");
#endif
    LCM_PRINT("[LCD] init_lcm_registers_sleep\n");
}

#if 0
static void init_lcm_registers_sleep(void)
{
    unsigned int data_array[1];
    MDELAY(10);
    data_array[0] = 0x00280500; //Display Off
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(30);
    data_array[0] = 0x00100500; //enter sleep
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    LCM_PRINT("[LCD] init_lcm_registers_sleep \n");
}

static void init_lcm_registers_sleep_out(void)
{
    unsigned int data_array[1];

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(50);
    LCM_PRINT("[LCD] init_lcm_registers_sleep \n");
}
#endif

/* VCAMD 1.8v LDO enable */
static void ldo_1v8io_on(void)
{
    mt_set_gpio_mode(GPIO_LCD_LDO_EN, GPIO_LCD_LDO_EN_M_GPIO);
    mt_set_gpio_dir(GPIO_LCD_LDO_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_LDO_EN, GPIO_OUT_ONE);

    LCM_PRINT("[LCD] ldo_1v8io_on\n");
}

/* VCAMD 1.8v LDO disable */
static void ldo_1v8io_off(void)
{
    mt_set_gpio_mode(GPIO_LCD_LDO_EN, GPIO_LCD_LDO_EN_M_GPIO);
    mt_set_gpio_dir(GPIO_LCD_LDO_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_LDO_EN, GPIO_OUT_ZERO);

    LCM_PRINT("[LCD] ldo_1v8io_off\n");
}

/* DSV power 5V,-5v */
static void ldo_p5m5_dsv_5v5_on(void)
{
    if(lge_get_board_revno() < HW_REV_C)
        rt4832_dsv_ctrl(1);
    else
        lm3632_dsv_ctrl(1);

    LCM_PRINT("[LCD] ldo_p5m5_dsv_5v5_on\n");
}

static void ldo_p5m5_dsv_5v5_off(void)
{
    if(lge_get_board_revno() < HW_REV_C)
        rt4832_dsv_ctrl(0);
    else
        lm3632_dsv_ctrl(0);

    LCM_PRINT("[LCD] ldo_p5m5_dsv_5v5_off\n");
}

static void reset_lcd_module(unsigned int reset)
{
    mt_set_gpio_mode(GPIO_LCM_RST, GPIO_LCM_RST_M_GPIO);
    mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);

    if(reset)
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    else
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);

    LCM_PRINT("LCD Reset %s \n",(reset)? "High":"Low");
}

static void lcm_resume_power(void)
{
    LCM_PRINT("[LCD][TOVIS] lcm_init_power : do nothing\n");
}

static void lcm_init(void)
{
    reset_lcd_module(0);

    ldo_1v8io_on();
    MDELAY(3);

    reset_lcd_module(1);
    MDELAY(20);

    ldo_p5m5_dsv_5v5_on();
    MDELAY(130);

    init_lcm_registers();

    need_set_lcm_addr = 1;

    LCM_PRINT("[LCD] lcm_init\n");
}

static void lcm_suspend_mfts(void)
{
    mfts_check_shutdown = true;
    reset_lcd_module(0);
    MDELAY(10);

    ldo_p5m5_dsv_5v5_off();

    ldo_1v8io_off();
    MDELAY(20);
    mfts_check_shutdown = false;

    LCM_PRINT("[LCD][TOVIS] lcm_suspend_for_mfts\n");
}

static void lcm_suspend(void)
{
    init_lcm_registers_sleep();

    LCM_PRINT("[LCD] lcm_suspend \n");
}

static void lcm_resume_mfts(void)
{
    LCM_PRINT("[LCD][TOVIS] lcm_resume_mfts : do nothing\n");
}

static void lcm_resume(void)
{
    lcm_init();

    need_set_lcm_addr = 1;

    LCM_PRINT("[LCD] lcm_resume\n");
}

static void lcm_shutdown(void)
{
    reset_lcd_module(0);
    MDELAY(5);

    ldo_p5m5_dsv_5v5_off();
    MDELAY(20);

    ldo_1v8io_off();
    MDELAY(10);

    LCM_PRINT("[LCD] lcm_shutdown\n");
}

#if 0
static void lcm_esd_recover(void)
{
    lcm_suspend();
    lcm_resume();

    LCM_PRINT("lcm_esd_recover \n");
}

static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);

    unsigned int data_array[16];

    // need update at the first time
    if(need_set_lcm_addr)
    {
        data_array[0]= 0x00053902;
        data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
        data_array[2]= (x1_LSB);
        dsi_set_cmdq(data_array, 3, 1);

        data_array[0]= 0x00053902;
        data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
        data_array[2]= (y1_LSB);
        dsi_set_cmdq(data_array, 3, 1);
        need_set_lcm_addr = 0;
    }

    data_array[0]= 0x002c3909;
   dsi_set_cmdq(data_array, 1, 0);
    LCM_PRINT("lcm_update \n");
}
#endif

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------

LCM_DRIVER td4100_hd720_dsi_vdo_tovis_lv5_drv = {
    .name = "TOVIS_TD4100",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .resume_power = lcm_resume_power,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .suspend_mfts = lcm_suspend_mfts,
    .resume_mfts = lcm_resume_mfts,
    .shutdown = lcm_shutdown,
#if defined(CONFIG_LGE_READER_MODE)
    .reader_mode = lcm_reader_mode,
#endif
#ifdef CONFIG_LGE_COMFORT_VIEW
    .comfort_view = lcm_comfort_view,
#endif
};