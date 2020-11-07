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

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

/* pixel */
#define FRAME_WIDTH             (720)
#define FRAME_HEIGHT            (1280)

/* physical dimension */
#define PHYSICAL_WIDTH          (66)
#define PHYSICAL_HEIGHT         (117)

#define LCM_ID                  (0xb9)
#define LCM_DSI_CMD_MODE        0

#define REGFLAG_DELAY           (0xAB)
#define REGFLAG_END_OF_TABLE    (0xAA) // END OF REGISTERS MARKER

#define ENABLE  1
#define DISABLE 0

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)        (lcm_util.set_reset_pin((v)))
#define UDELAY(n)               (lcm_util.udelay(n))
#define MDELAY(n)               (lcm_util.mdelay(n))

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

#ifdef CONFIG_LEDS_DW8767_CV1
extern void dw8767_dsv_ctrl(unsigned int enable);
#endif
/* touch irq handle according to display suspend in MFTS */
extern bool mfts_check_shutdown;
extern unsigned int gSetShutdown;

/* --------------------------------------------------------------------------- */
/* GPIO Set */
/* --------------------------------------------------------------------------- */

//dir out : mode 00 dir in : mode 01

#define GPIO_LCD_LDO_EN             (GPIO16 | 0x80000000)
#define GPIO_LCD_LDO_EN_MODE        GPIO_MODE_00

#define GPIO_LCD_RESET_N            (GPIO158 | 0x80000000)
#define GPIO_LCD_RESET_N_MODE       GPIO_MODE_00

#if 0
#define GPIO_TOUCH_EN               (GPIO19 | 0x80000000)
#define GPIO_TOUCH_EN_MODE          GPIO_MODE_00
#endif

#define GPIO_TOUCH_RESET_N          (GPIO2 | 0x80000000)
#define GPIO_TOUCH_RESET_N_MODE     GPIO_MODE_00

#define LGE_LPWG_SUPPORT

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static LCM_setting_table_V3 lcm_initial_sleep_out[] = {
    {0x05, 0x11, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

static LCM_setting_table_V3 lcm_initial_disp_on[] = {
    {0x05, 0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

static LCM_setting_table_V3 lcm_initial_sleep_in[] = {
    {0x05, 0x10, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

static LCM_setting_table_V3 lcm_initial_display_off[] = {
    {0x05, 0x28, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

static LCM_setting_table_V3 lcm_deep_standby_mode[] = {
    {0x39, 0x50,  2, {0x5A,0x24}},
    {0x15, 0x90,  1, {0xA5}},
    {0x39, 0x04,  2, {0xA5,0xA5}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

#ifdef CONFIG_LGE_COMFORT_VIEW
static LCM_setting_table_V3 lcm_initial_comfort_view_cmd[] = {
    {0x39, 0x50,  2, {0x5A,0x18}},
    {0x39, 0xB1, 13, {0x00,0xAC,0x58,0x59,0x00,0xA8,0x3F,0xE8,0x01,0x70,0x77,0x4E,0x03}},
    {0x15, 0x51,  1, {0x23}},
    {0x15, 0x8E,  1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};
#endif

static void init_lcm_registers_sleep_in(void)
{
    dsi_set_cmdq_V3(lcm_initial_display_off, sizeof(lcm_initial_display_off) / sizeof(LCM_setting_table_V3), 1);
    MDELAY(20);
    dsi_set_cmdq_V3(lcm_initial_sleep_in, sizeof(lcm_initial_sleep_in) / sizeof(LCM_setting_table_V3), 1);
    dsi_set_cmdq_V3(lcm_deep_standby_mode, sizeof(lcm_deep_standby_mode) / sizeof(LCM_setting_table_V3), 1);
    MDELAY(50);

    LCM_PRINT("[LCD] %s\n",__func__);
}

static void init_lcm_registers_sleep_out(void)
{
    dsi_set_cmdq_V3(lcm_initial_sleep_out, sizeof(lcm_initial_sleep_out) / sizeof(LCM_setting_table_V3), 1);
    MDELAY(120);

#ifdef CONFIG_LGE_COMFORT_VIEW
    dsi_set_cmdq_V3(lcm_initial_comfort_view_cmd, sizeof(lcm_initial_comfort_view_cmd) / sizeof(LCM_setting_table_V3), 1);
    dsi_set_cmdq_V3(lcm_initial_disp_on, sizeof(lcm_initial_disp_on) / sizeof(LCM_setting_table_V3), 1);
    MDELAY(10);
#else
    dsi_set_cmdq_V3(lcm_initial_disp_on, sizeof(lcm_initial_disp_on) / sizeof(LCM_setting_table_V3), 1);
#endif

    LCM_PRINT("[LCD] %s\n",__func__);
}

#ifdef CONFIG_LGE_COMFORT_VIEW
#define COMFORT_VIEW_CMD_CNT 3

static LCM_setting_table_V3 lcm_comfort_view_off[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// 7001K
static LCM_setting_table_V3 lcm_comfort_view_step_1[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x7D}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// 6612K
static LCM_setting_table_V3 lcm_comfort_view_step_2[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x75}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// 6223K
static LCM_setting_table_V3 lcm_comfort_view_step_3[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x6D}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// 5834K
static LCM_setting_table_V3 lcm_comfort_view_step_4[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x61}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// 5445K
static LCM_setting_table_V3 lcm_comfort_view_step_5[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x55}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// 5056K
static LCM_setting_table_V3 lcm_comfort_view_step_6[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x4B}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// 4667K
static LCM_setting_table_V3 lcm_comfort_view_step_7[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x3B}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// 4278K
static LCM_setting_table_V3 lcm_comfort_view_step_8[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x2D}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// 3889K
static LCM_setting_table_V3 lcm_comfort_view_step_9[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x1D}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// 3500K
static LCM_setting_table_V3 lcm_comfort_view_step_10[] = {
    {0x39, 0x50,  2, {0x5A,0x23}},
    {0x15, 0x8E,  1, {0x0B}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

// step 0 ~ 10
LCM_setting_table_V3 *lcm_comfort_view_cmd[] = {
    lcm_comfort_view_off,
    lcm_comfort_view_step_1,lcm_comfort_view_step_2,
    lcm_comfort_view_step_3,lcm_comfort_view_step_4,
    lcm_comfort_view_step_5,lcm_comfort_view_step_6,
    lcm_comfort_view_step_7,lcm_comfort_view_step_8,
    lcm_comfort_view_step_9,lcm_comfort_view_step_10,
};

static void lcm_comfort_view(unsigned int mode)
{
    dsi_set_cmdq_V3(lcm_comfort_view_cmd[mode], COMFORT_VIEW_CMD_CNT, 1);

    LCM_PRINT("[LCD] %s : %d\n",__func__,mode);
}
#endif

#ifdef CONFIG_LGE_CHECK_SURFACE_TEMPERATURE
static LCM_setting_table_V3 lcm_gamma_cmd_on[] = {
    {0x39, 0x50,  2, {0x5A,0x0A}},
    {0x39, 0x80, 16, {0x73,0xE7,0x02,0x17,0x30,0x44,0x59,0x6F,0x82,0x89,0xB9,0xBC,0xED,0x52,0x26,0x73}},
    {0x39, 0x90, 16, {0x4A,0x48,0x39,0x2A,0x1D,0x15,0x0F,0x0D,0x30,0x44,0x59,0x6F,0x82,0x89,0xB9,0xBC}},
    {0x39, 0xA0, 12, {0xED,0x52,0x26,0x73,0x4A,0x48,0x39,0x2A,0x1D,0x15,0x0F,0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

static LCM_setting_table_V3 lcm_gamma_cmd_off[] = {
    {0x39, 0x50,  2, {0x5A,0x0A}},
    {0x39, 0x80, 16, {0x73,0xE7,0x02,0x16,0x30,0x44,0x59,0x6F,0x82,0x89,0xB9,0xBC,0xED,0x52,0x26,0x73}},
    {0x39, 0x90, 16, {0x4A,0x48,0x39,0x2A,0x1D,0x15,0x17,0x0E,0x30,0x44,0x59,0x6F,0x82,0x89,0xB9,0xBC}},
    {0x39, 0xA0, 12, {0xED,0x52,0x26,0x73,0x4A,0x48,0x39,0x2A,0x1D,0x15,0x08,0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1, {}},
};

static void lcm_set_gamma_cmd(unsigned int enable)
{
    if(enable)
        dsi_set_cmdq_V3(lcm_gamma_cmd_on, sizeof(lcm_gamma_cmd_on) / sizeof(LCM_setting_table_V3), 1);
    else
        dsi_set_cmdq_V3(lcm_gamma_cmd_off, sizeof(lcm_gamma_cmd_off) / sizeof(LCM_setting_table_V3), 1);

    LCM_PRINT("[LCD] %s : %s\n",__func__,enable? "enable":"disable");
}
#endif

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

    params->dsi.mode = SYNC_PULSE_VDO_MODE;
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

    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.cont_clock = 1;

    params->dsi.vertical_sync_active = 8;
    params->dsi.vertical_backporch = 32;
    params->dsi.vertical_frontporch = 226;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 16;
    params->dsi.horizontal_backporch = 60;
    params->dsi.horizontal_frontporch = 68;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 253; // mipi clk : 506Mhz
    params->dsi.ssc_disable = 1;

    LCM_PRINT("[LCD] %s\n",__func__);
}

static void ldo_1v8io_ctrl(unsigned int enable)
{
    mt_set_gpio_mode(GPIO_LCD_LDO_EN, GPIO_LCD_LDO_EN_MODE);
    mt_set_gpio_dir(GPIO_LCD_LDO_EN, GPIO_DIR_OUT);

    if(enable)
        mt_set_gpio_out(GPIO_LCD_LDO_EN, GPIO_OUT_ONE);
    else
        mt_set_gpio_out(GPIO_LCD_LDO_EN, GPIO_OUT_ZERO);

    LCM_PRINT("[LCD] %s : %s\n",__func__,(enable)? "ON":"OFF");
}

static void lcm_dsv_ctrl(unsigned int enable)
{
#ifdef CONFIG_LEDS_DW8767_CV1
    dw8767_dsv_ctrl(enable);
#endif

    LCM_PRINT("[LCD] %s : %s\n",__func__,(enable)? "ON":"OFF");
}

static void reset_lcd_module(unsigned int reset)
{
    mt_set_gpio_mode(GPIO_LCD_RESET_N, GPIO_LCD_RESET_N_MODE);
    mt_set_gpio_dir(GPIO_LCD_RESET_N, GPIO_DIR_OUT);

    if(reset)
        mt_set_gpio_out(GPIO_LCD_RESET_N, GPIO_OUT_ONE);
    else
        mt_set_gpio_out(GPIO_LCD_RESET_N, GPIO_OUT_ZERO);

    LCM_PRINT("[LCD] LCD Reset %s \n",(reset)? "High":"Low");
}

static void touch_reset_pin(unsigned int mode)
{
    mt_set_gpio_mode(GPIO_TOUCH_RESET_N, GPIO_TOUCH_RESET_N_MODE);
    mt_set_gpio_dir(GPIO_TOUCH_RESET_N, GPIO_DIR_OUT);

    if(mode)
        mt_set_gpio_out(GPIO_TOUCH_RESET_N, GPIO_OUT_ONE);
    else
        mt_set_gpio_out(GPIO_TOUCH_RESET_N, GPIO_OUT_ZERO);

    LCM_PRINT("Touch Reset %s \n",(mode)? "High":"Low");
}

static void lcm_suspend(void)
{
    if(gSetShutdown == ENABLE)
      MDELAY(50);

    init_lcm_registers_sleep_in();

    LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_suspend_mfts(void)
{
    mfts_check_shutdown=true;
    lcm_dsv_ctrl(DISABLE);
    MDELAY(5);

    touch_reset_pin(DISABLE);
    reset_lcd_module(DISABLE);

    ldo_1v8io_ctrl(ENABLE);
    MDELAY(5);
    mfts_check_shutdown=false;

    LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_resume(void)
{
    // reset toggle for wake up from deep standby mode
    touch_reset_pin(DISABLE);
    MDELAY(10);
    reset_lcd_module(DISABLE);
    MDELAY(10);

    touch_reset_pin(ENABLE);
    MDELAY(10);
    reset_lcd_module(ENABLE);
    MDELAY(35);

    init_lcm_registers_sleep_out();

    need_set_lcm_addr = 1;

    LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_resume_mfts(void)
{
    mfts_check_shutdown=true;
    ldo_1v8io_ctrl(DISABLE);
    MDELAY(2);

    lcm_dsv_ctrl(ENABLE);
    MDELAY(3);

    touch_reset_pin(ENABLE);
    MDELAY(10);
    reset_lcd_module(ENABLE);
    MDELAY(35);
    mfts_check_shutdown=false;

    LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_shutdown(void)
{
    MDELAY(150);

    lcm_dsv_ctrl(DISABLE);
    MDELAY(5);

    touch_reset_pin(DISABLE);
    reset_lcd_module(DISABLE);

    if(lge_get_board_revno() < HW_REV_A)
        ldo_1v8io_ctrl(DISABLE);

    LCM_PRINT("[LCD] %s\n",__func__);
}

/* --------------------------------------------------------------------------- */
/* Get LCM Driver Hooks */
/* --------------------------------------------------------------------------- */
LCM_DRIVER ft8006m_hd720_dsi_vdo_tcl_cv1_lcm_drv = {
    .name = "TCL_FT8006M",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .shutdown = lcm_shutdown,
    .suspend_mfts = lcm_suspend_mfts,
    .resume_mfts = lcm_resume_mfts,
#ifdef CONFIG_LGE_COMFORT_VIEW
    .comfort_view = lcm_comfort_view,
#endif
#ifdef CONFIG_LGE_CHECK_SURFACE_TEMPERATURE
    .set_gamma_cmd = lcm_set_gamma_cmd,
#endif
};
