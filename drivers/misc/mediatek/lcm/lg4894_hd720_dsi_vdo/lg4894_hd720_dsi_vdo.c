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
#if defined CONFIG_LGD_INCELL_DB7400_HD_PH1
#include "rt4832.h"
#endif
#endif
#include <linux/input/lge_touch_notify.h>
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
#include <soc/mediatek/lge/board_lge.h>
#endif
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
/* pixel */
#define FRAME_WIDTH			(720)
#define FRAME_HEIGHT			(1280)

// physical dimension
#define PHYSICAL_WIDTH        (66)
#define PHYSICAL_HEIGHT         (117)

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
//static unsigned int need_revision_check = 1;

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

#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
#ifndef GPIO_LCD_LDO_EN
#define GPIO_LCD_LDO_EN         (GPIO16 | 0x80000000)
#define GPIO_LCD_LDO_EN_M_GPIO   GPIO_MODE_00
#endif
#endif

extern u16 pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname, u32 val);
extern void rt4832_dsv_ctrl(int enable);
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
extern void lm3632_dsv_ctrl(int enable);
#endif
extern int lge_get_lcm_revision(void);
#define LGE_LPWG_SUPPORT
//static unsigned char panel_rev = 0xFF;

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static LCM_setting_table_V3 lg4894_initial_command[] = {
    {0x15, 0xB0,  1,  {0xAC}},
    {0x39, 0xB1,  4,  {0x10, 0x30, 0x16, 0x00}},
    {0x39, 0xB2,  12, {0x01, 0x00, 0x21, 0x20, 0x58, 0x00, 0xCF, 0x10, 0x1F, 0x1F, 0x34, 0x34}},
    {0x39, 0xB4,  3,  {0x00, 0x9F, 0x00}},
    {0x39, 0xB5,  5,  {0x42, 0xC0, 0x80, 0x00, 0x00}},
    {0x39, 0xB6,  3,  {0x77, 0x34, 0x48}},
    {0x39, 0xBD,  9,  {0xD0, 0x02, 0xF4, 0x01, 0x04, 0x04, 0x12, 0x20, 0x22}},
    {0x39, 0xBE,  7,  {0xE0, 0xE0, 0xC9, 0xC9, 0xC8, 0xF8, 0x00}},
    {0x39, 0xC1,  6,  {0x01, 0xE8, 0xD8, 0xC2, 0xC1, 0x00}},
    {0x39, 0xC2,  3,  {0x3B, 0x13, 0x13}},
    {0x39, 0xC3,  6,  {0x15, 0x2F, 0x2F, 0x00, 0x66, 0x21}},
    {0x39, 0xC4,  3,  {0x51, 0x00, 0x37}},
    {0x39, 0xC5,  5,  {0x25, 0x20, 0x20, 0x14, 0x14}},
    {0x39, 0xC7,  12, {0x10, 0x22, 0x00, 0x28, 0x00, 0x62, 0x62, 0x62, 0x00, 0xB1, 0x01, 0x00}},
    {0x39, 0xC8,  4,  {0x01, 0x00, 0x03, 0x8C}},
    {0x39, 0xCC,  14, {0x22, 0x2F, 0x11, 0x26, 0x21, 0x24, 0x02, 0x62, 0x62, 0x62, 0x00, 0xB1, 0x01, 0x00}},
    {0x39, 0xD0,  12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x51, 0x00, 0x00, 0x00, 0x42, 0x46, 0x03}},
    {0x39, 0xD1,  12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x51, 0x00, 0x00, 0x00, 0x42, 0x46, 0x03}},
    {0x39, 0xD2,  12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x51, 0x00, 0x00, 0x00, 0x42, 0x46, 0x03}},
    {0x39, 0xD3,  12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x51, 0x00, 0x00, 0x00, 0x42, 0x46, 0x03}},
    {0x39, 0xD4,  12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x70, 0x0F, 0x0F, 0x00, 0x21, 0x46, 0x03}},
    {0x39, 0xD5,  12, {0x10, 0x67, 0x43, 0x43, 0x32, 0x70, 0x0F, 0x0F, 0x00, 0x21, 0x46, 0x03}},
    {0x39, 0xE4,  21, {0x2F, 0xC3, 0x2F, 0x60, 0x41, 0xC5, 0xC6, 0xCC, 0xCC, 0xCB, 0x2F, 0xCB, 0xCA, 0xCA, 0xC9, 0xC9, 0xC7, 0xC8, 0x2F, 0x41, 0x2F}},
    {0x39, 0xE5,  12, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x13, 0xEF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03}},
    {0x05, 0x11,  1,  {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_initial_disp_on[] = {
    {0x05, 0x29,	1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_touch_osc_on_cmd[] = {
    {0x15, 0xBF,    1, {0x20}},
    {0x15, 0xBF,    1, {0x60}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_touch_osc_off_cmd[] = {
    {0x15, 0xBF,    1, {0x20}},
    {0x15, 0xBF,    1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

#if defined(CONFIG_LGE_READER_MODE)
static LCM_setting_table_V3 lcm_reader_mode_off[] = {
    {0x15, 0xED,    1, {0x00}},  // reader mode disable
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_reader_mode_low[] = {
    // 5850K
    {0x39, 0xF5,   8, {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}},
    {0x39, 0xF6,   8, {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}},
    {0x39, 0xF7,   8, {0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34}},
    {0x15, 0xED,    1, {0x04}},  // reader mode enable
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_reader_mode_mid[] = {
    // 5500K
    {0x39, 0xF5,   8, {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}},
    {0x39, 0xF6,   8, {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}},
    {0x39, 0xF7,   8, {0x3B, 0x2F, 0x2F, 0x2F, 0x2F, 0x2F, 0x2F, 0x2F}},
    {0x15, 0xED,    1, {0x04}},  // reder mode enable
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_reader_mode_high[] = {
    // 5350K
    {0x39, 0xF5,   8, {0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F}},
    {0x39, 0xF6,   8, {0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F}},
    {0x39, 0xF7,   8, {0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D}},
    {0x15, 0xED,    1, {0x04}},  // reder mode enable
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_reader_mode_mono[] = {
    {0x15, 0xED,    1, {0x00}},  // reader mode disable, DDIC monochrome mode not supported
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static void lcm_reader_mode(LCM_READER_MODE mode)
{
    switch(mode)
    {
        case READER_MODE_OFF :
            dsi_set_cmdq_V3(lcm_reader_mode_off, sizeof(lcm_reader_mode_off) / sizeof(LCM_setting_table_V3), 1);
            break;
        case READER_MODE_LOW :
            dsi_set_cmdq_V3(lcm_reader_mode_low, sizeof(lcm_reader_mode_low) / sizeof(LCM_setting_table_V3), 1);
            break;
        case READER_MODE_MID :
            dsi_set_cmdq_V3(lcm_reader_mode_mid, sizeof(lcm_reader_mode_mid) / sizeof(LCM_setting_table_V3), 1);
            break;
        case READER_MODE_HIGH :
            dsi_set_cmdq_V3(lcm_reader_mode_high, sizeof(lcm_reader_mode_high) / sizeof(LCM_setting_table_V3), 1);
            break;
        case READER_MODE_MONO :
            dsi_set_cmdq_V3(lcm_reader_mode_mono, sizeof(lcm_reader_mode_mono) / sizeof(LCM_setting_table_V3), 1);
            break;
        default :
            break;
    }
    LCM_PRINT("[LCD] READER MODE : %d\n", mode);
}
#endif

#ifdef CONFIG_LGE_COMFORT_VIEW
#define COMFORT_VIEW_CMD_CNT 5
#define COMFORT_VIEW_OFF_CMD_CNT 1

static LCM_setting_table_V3 lcm_comfort_view_off[] = {
	{0x15, 0xED,   1, {0x00}},
};

// 7300K
static LCM_setting_table_V3 lcm_comfort_view_step_1[] = {
	{0x15, 0x55,   1, {0x80}},
	{0x15, 0xED,   1, {0x04}},
	{0x39, 0xF5,  16, {0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF6,  16, {0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF7,  16, {0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

// 7000K
static LCM_setting_table_V3 lcm_comfort_view_step_2[] = {
	{0x15, 0x55,   1, {0x80}},
	{0x15, 0xED,   1, {0x04}},
	{0x39, 0xF5,  16, {0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF6,  16, {0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF7,  16, {0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

// 6650K
static LCM_setting_table_V3 lcm_comfort_view_step_3[] = {
	{0x15, 0x55,   1, {0x80}},
	{0x15, 0xED,   1, {0x04}},
	{0x39, 0xF5,  16, {0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF6,  16, {0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF7,  16, {0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

// 6300K
static LCM_setting_table_V3 lcm_comfort_view_step_4[] = {
	{0x15, 0x55,   1, {0x80}},
	{0x15, 0xED,   1, {0x04}},
	{0x39, 0xF5,  16, {0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF6,  16, {0x3A, 0x3A, 0x3A, 0x3A, 0x3A, 0x3A, 0x3A, 0x3A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF7,  16, {0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

// 6000K
static LCM_setting_table_V3 lcm_comfort_view_step_5[] = {
	{0x15, 0x55,   1, {0x80}},
	{0x15, 0xED,   1, {0x04}},
	{0x39, 0xF5,  16, {0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF6,  16, {0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF7,  16, {0x29, 0x29, 0x29, 0x29, 0x29, 0x29, 0x29, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

// 5650K
static LCM_setting_table_V3 lcm_comfort_view_step_6[] = {
	{0x15, 0x55,   1, {0x80}},
	{0x15, 0xED,   1, {0x04}},
	{0x39, 0xF5,  16, {0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF6,  16, {0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF7,  16, {0x26, 0x26, 0x26, 0x26, 0x26, 0x26, 0x26, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

// 5300K
static LCM_setting_table_V3 lcm_comfort_view_step_7[] = {
	{0x15, 0x55,   1, {0x80}},
	{0x15, 0xED,   1, {0x04}},
	{0x39, 0xF5,  16, {0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF6,  16, {0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF7,  16, {0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

// 5000K
static LCM_setting_table_V3 lcm_comfort_view_step_8[] = {
	{0x15, 0x55,   1, {0x80}},
	{0x15, 0xED,   1, {0x04}},
	{0x39, 0xF5,  16, {0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF6,  16, {0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF7,  16, {0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

// 4800K
static LCM_setting_table_V3 lcm_comfort_view_step_9[] = {
	{0x15, 0x55,   1, {0x80}},
	{0x15, 0xED,   1, {0x04}},
	{0x39, 0xF5,  16, {0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF6,  16, {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF7,  16, {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

// 4650K
static LCM_setting_table_V3 lcm_comfort_view_step_10[] = {
	{0x15, 0x55,   1, {0x80}},
	{0x15, 0xED,   1, {0x04}},
	{0x39, 0xF5,  16, {0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF6,  16, {0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0x39, 0xF7,  16, {0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
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
	unsigned int len = COMFORT_VIEW_CMD_CNT;

	if(!mode)
		len = COMFORT_VIEW_OFF_CMD_CNT;

	dsi_set_cmdq_V3(lcm_comfort_view_cmd[mode], len, 1);

    LCM_PRINT("[LCD] %s : %d\n",__func__,mode);
}
#endif

static LCM_setting_table_V3 lcm_initial_for_sleep_in[] =
{
	{0x05, 0x10, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_initial_for_display_off[] =
{
        {0x05, 0x28, 1, {0x00}},
        {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static struct LCM_setting_table __attribute__ ((unused)) lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
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

	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	// enable tearing-free
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
	params->dsi.vertical_backporch = 15;
	params->dsi.vertical_frontporch = 500;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 36;
	params->dsi.horizontal_frontporch = 12;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 272;

        /* ESD recovery */
        params->dsi.esd_check_enable = 0;
        params->dsi.customization_esd_check_enable = 1;
        params->dsi.lcm_esd_check_table[0].cmd = 0xDA;
        params->dsi.lcm_esd_check_table[0].count = 0x01;
        params->dsi.lcm_esd_check_table[0].para_list[0] = 0x00;

        params->dsi.HS_TRAIL = 6;
        params->dsi.HS_PRPR = 6;
}

static int lcm_set_touch_osc(int enable)
{
    if(enable)
    {
        dsi_set_cmdq_V3(lcm_touch_osc_on_cmd, sizeof(lcm_touch_osc_on_cmd) / sizeof(LCM_setting_table_V3), 1);
    }
    else
    {
        dsi_set_cmdq_V3(lcm_touch_osc_off_cmd, sizeof(lcm_touch_osc_off_cmd) / sizeof(LCM_setting_table_V3), 1);
    }
    LCM_PRINT("%s : enable=%d\n", __func__, enable);

    return 0;
}

static void init_lcm_registers(void)
{
    dsi_set_cmdq_V3(lg4894_initial_command, sizeof(lg4894_initial_command) / sizeof(LCM_setting_table_V3), 1);
    MDELAY(100);
    dsi_set_cmdq_V3(lcm_initial_disp_on, sizeof(lcm_initial_disp_on) / sizeof(LCM_setting_table_V3), 1);
#if 0
    if(panel_rev <= 0x26)
        LCM_PRINT("[LCD] init_lcm_registers for OLD panel\n");
    else
        LCM_PRINT("[LCD] init_lcm_registers \n");
#endif
    LCM_PRINT("[LCD] init_lcm_registers \n");
}

static void init_lcm_registers_sleep(void)
{
        dsi_set_cmdq_V3(lcm_initial_for_display_off, sizeof(lcm_initial_for_display_off) / sizeof(LCM_setting_table_V3), 1);
        MDELAY(100);
        dsi_set_cmdq_V3(lcm_initial_for_sleep_in, sizeof(lcm_initial_for_sleep_in) / sizeof(LCM_setting_table_V3), 1);
        MDELAY(120);
	LCM_PRINT("[LCD] init_lcm_registers_sleep\n");
}

static void init_lcm_registers_sleep_2nd(void)
{
	//unsigned int data_array[1];
	//dsi_set_cmdq_V3(lcm_initialization_for_sleep_in_post, sizeof(lcm_initialization_for_sleep_in_post) / sizeof(LCM_setting_table_V3), 1);
	LCM_PRINT("[LCD] init_lcm_registers_sleep 2nd \n");
}

/* 1.8v LDO is always on */
static void ldo_1v8io_on(void)
{
	//pmic_set_register_value(PMIC_LDO_VIO18_EN_ADDR, 1);
	mt_set_gpio_mode(GPIO_LCD_LDO_EN, GPIO_LCD_LDO_EN_M_GPIO);
	mt_set_gpio_dir(GPIO_LCD_LDO_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LDO_EN, GPIO_OUT_ONE);
	LCM_PRINT("[LCD] ldo_1v8io_on \n");
}

/* 1.8v LDO is always on */
static void ldo_1v8io_off(void)
{
	//pmic_set_register_value(PMIC_LDO_VIO18_EN_ADDR, 0);
	mt_set_gpio_mode(GPIO_LCD_LDO_EN, GPIO_LCD_LDO_EN_M_GPIO);
	mt_set_gpio_dir(GPIO_LCD_LDO_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LDO_EN, GPIO_OUT_ZERO);
	LCM_PRINT("[LCD] ldo_1v8io_off \n");
}

/* VGP1 3.0v LDO enable */
static void ldo_3v0_on(void)
{
	mt_set_gpio_mode(GPIO_TOUCH_EN, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_TOUCH_EN, GPIO_OUT_ONE);
	LCM_PRINT("[LCD] ldo_3v0_on \n");
}

/* VGP1 3.0v LDO disable */
static void ldo_3v0_off(void)
{
	mt_set_gpio_mode(GPIO_TOUCH_EN, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_TOUCH_EN, GPIO_OUT_ZERO);
	LCM_PRINT("[LCD] ldo_3v0_off \n");
}


/* DSV power +5.8V,-5.8v */
static void ldo_p5m5_dsv_5v5_on(void)
{
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
	if(lge_get_board_revno() < HW_REV_C)
		rt4832_dsv_ctrl(1);
	else
		lm3632_dsv_ctrl(1);
#else
	rt4832_dsv_ctrl(1);
#endif
	LCM_PRINT("[LCD] ldo_p5m5_dsv_5v5_on \n");
}


/* DSV power +5.8V,-5.8v */
static void ldo_p5m5_dsv_5v5_off(void)
{
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
	if(lge_get_board_revno() < HW_REV_C)
		rt4832_dsv_ctrl(0);
	else
		lm3632_dsv_ctrl(0);
#else
	rt4832_dsv_ctrl(0);
#endif
	LCM_PRINT("[LCD] ldo_p5m5_dsv_5v5_off \n");
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

static void lcm_init(void)
{
#if 0
	if(need_revision_check == 1)
	{
		LCM_setting_table_V3 old_b2h = {0x39, 0xB2,  12, {0x01, 0x00, 0xA1, 0x20, 0x58, 0x00, 0xCF, 0x10, 0x1F, 0x1F, 0x34, 0x34}};

		panel_rev = lge_get_lcm_revision();
        if(panel_rev <= 0x26)  // parameter change for rev0.6 under panel
        {
		memcpy((void*)&lg4894_initial_command[2], (void*)&old_b2h, sizeof(old_b2h));
        }
		need_revision_check = 0;
		LCM_PRINT("[LCD] PANEL REVID  ECh : [0x%x]\n", panel_rev);
	}
#endif
	reset_lcd_module(1);
	MDELAY(5);
	lcm_set_touch_osc(1);
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);
	ldo_p5m5_dsv_5v5_on();
	MDELAY(5);
	init_lcm_registers();
	//MDELAY(20);
	need_set_lcm_addr = 1;

	LCM_PRINT("[LCD] lcm_init\n");
}

static void lcm_suspend(void)
{
	init_lcm_registers_sleep();

	LCM_PRINT("[LCD] lcm_suspend\n");
}

static void lcm_suspend_mfts(void)
{
        touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
        reset_lcd_module(0);
        MDELAY(10);
        ldo_p5m5_dsv_5v5_off();
        ldo_3v0_off();
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
        if(lge_get_board_revno()!= HW_REV_A)
            ldo_1v8io_off();
#else
        ldo_1v8io_off();
#endif
        MDELAY(20);

        LCM_PRINT("[LCD] lcm_suspend_mfts \n");
}

static void lcm_resume_power(void)
 {
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
	reset_lcd_module(0);
	//MDELAY(10);
	//ldo_p5m5_dsv_5v5_off();
	//ldo_3v0_off();
	//ldo_1v8io_off();
	//MDELAY(20);
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
	if(lge_get_board_revno()!= HW_REV_A)
		ldo_1v8io_on();
#else
	ldo_1v8io_on();
#endif
	MDELAY(3);
	ldo_3v0_on();
	MDELAY(5);

	LCM_PRINT("[LCD] lcm_resume_power \n");
}


static void lcm_resume(void)
{
	lcm_init();
	need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_resume\n");
}

static void lcm_resume_mfts(void)
{
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
    if(lge_get_board_revno()!= HW_REV_A)
        ldo_1v8io_on();
#else
    ldo_1v8io_on();
#endif
    MDELAY(5);
    ldo_3v0_on();

    LCM_PRINT("[LCD] lcm_resume_mfts \n");
}


static void lcm_shutdown(void)
{
	ldo_p5m5_dsv_5v5_off();
	MDELAY(10);
	reset_lcd_module(0);
	touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
	MDELAY(5);
	ldo_3v0_off();
	MDELAY(1);
#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
    if(lge_get_board_revno()!= HW_REV_A)
        ldo_1v8io_off();
#else
    ldo_1v8io_off();
#endif

	LCM_PRINT("[LCD] lcm_shutdown \n");
}

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

LCM_DRIVER lg4894_hd720_dsi_vdo_drv = {
	.name = "LGD_LG4894",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.resume_power = lcm_resume_power,
	.suspend_2nd_cmd = init_lcm_registers_sleep_2nd,
	.shutdown = lcm_shutdown,
	.suspend_mfts = lcm_suspend_mfts,
	.resume_mfts = lcm_resume_mfts,
#if defined(CONFIG_LGE_READER_MODE)
	.reader_mode = lcm_reader_mode,
#endif
#ifdef CONFIG_LGE_COMFORT_VIEW
    .comfort_view = lcm_comfort_view,
#endif
	.set_touch_osc = lcm_set_touch_osc,
};