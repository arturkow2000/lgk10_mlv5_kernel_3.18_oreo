#ifdef BUILD_LK
#include <sys/types.h>
#else
#include <linux/string.h>
#include <soc/mediatek/lge/board_lge.h>
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#ifdef BUILD_LK
    #define LCM_PRINT printf
#else
    #define LCM_PRINT printk
#endif

#ifndef FALSE
  #define FALSE   0
#endif

#ifndef TRUE
  #define TRUE    1
#endif

#define REGFLAG_DELAY             0xFD
#define REGFLAG_END_OF_TABLE      0xFE   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
#define FRAME_WIDTH               (800)
#define FRAME_HEIGHT              (1280)

#define PHYSICAL_WIDTH            (108)
#define PHYSICAL_HEIGHT           (172)

#define LCM_MODULE_NAME    "HX8394A_GX_INX_B026"

#ifdef BUILD_LK
#define LCM_ID                    (0x941B)
//#define AUX_ADC_EXTERNAL_CHANNEL                (12)
#ifdef AUX_ADC_EXTERNAL_CHANNEL
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
#endif
#endif

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)                                   lcm_util.set_reset_pin((v))
#define UDELAY(n)                                          lcm_util.udelay(n)
#define MDELAY(n)                                          lcm_util.mdelay(n)
#define dsi_set_cmdq(pdata, queue_size, force_update)      lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)   lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define read_reg_v2(cmd, buffer, buffer_size)              lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#if 0   //Open relevant code based on requirement
#define wrtie_cmd(cmd)                                     lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                 lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                      lcm_util.dsi_dcs_read_lcm_reg(cmd)
#endif

#ifdef CONFIG_HQ_SET_LCD_BIAS
#define SET_LCD_BIAS_VSPN(value)                                  lcm_util.set_lcd_bias_vspn(value)
#define SET_LCD_BIAS_VSPN_EN(en, seq)                                  lcm_util.set_lcd_bias_vspn_en(en, seq)
#endif

#if 0   //HX8394A IC need use the dsi_set_cmdq_V3 method instead of dsi_set_cmdq_V2 to send the initial code
struct LCM_setting_table {
    unsigned char cmd;
    unsigned int  count;
    unsigned char para_list[64];
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    unsigned char cmd;

    for(i = 0; i < count; i++) {

        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                    MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
}
#endif

static LCM_setting_table_V3 lcm_patch_code[] = {

    {0x39,0xB9,3,{0xFF,0x83,0x94}},

    {0x39,0xB1,16,{0x7C,0x00,0x04,0x0F,0x03,0x0E,0xEE,0x21,0x29,0x19,
              0x19,0x57,0x12,0x00,0xE6,0xE2}},

};

#if defined(CONFIG_LGE_READER_MODE)
static LCM_setting_table_V3 lcm_reader_mode_off[] = {
    //reader_mode off command
    {0x15, 0xC1, 1, {0x00}},
};
static LCM_setting_table_V3 lcm_reader_mode_low[] = {
    //6200K
    {0x39, 0xC1, 16, {0x01, 0x00, 0x0D, 0x14, 0x1E, 0x28, 0x2E, 0x35, 0x3E, 0x47,
                      0x4F, 0x5A, 0x63, 0x6B, 0x71, 0x79}},
    {0x29, 0xC1, 16, {0x80, 0x85, 0x8C, 0x93, 0x9A, 0xA0, 0xA5, 0xAC, 0xB3, 0xBA,
                      0xC3, 0xCC, 0xD7, 0xE2, 0xEF, 0xFA}},
    {0x29, 0xC1, 16, {0xFF, 0xFF, 0x61, 0x6F, 0x44, 0x74, 0xC4, 0xB7, 0x36, 0x00,
                      0xC0, 0x00, 0x0D, 0x14, 0x1E, 0x28}},
    {0x29, 0xC1, 16, {0x2E, 0x35, 0x3E, 0x47, 0x4F, 0x5A, 0x63, 0x6B, 0x71, 0x79,
                      0x80, 0x85, 0x8C, 0x93, 0x9A, 0xA0}},
    {0x29, 0xC1, 16, {0xA5, 0xAC, 0xB3, 0xBA, 0xC3, 0xCC, 0xD7, 0xE2, 0xEF, 0xFA,
                      0xFF, 0xFF, 0x61, 0x6F, 0x44, 0x74}},
    {0x29, 0xC1, 16, {0xC4, 0xB7, 0x36, 0x00, 0xC0, 0x00, 0x0C, 0x11, 0x18, 0x1F,
		      0x27, 0x2D, 0x33, 0x3A, 0x41, 0x47}},
    {0x29, 0xC1, 16, {0x4F, 0x57, 0x5F, 0x66, 0x6C, 0x71, 0x78, 0x7E, 0x82, 0x88,
                      0x8D, 0x92, 0x98, 0x9D, 0xA2, 0xA7}},
    {0x29, 0xC1, 15, {0xAC, 0xB2, 0xB9, 0xBE, 0xC5, 0xCE, 0x4C, 0xFC, 0x0E, 0x3E,
		      0xD7, 0x43, 0x03, 0x6C, 0xC0}},
};
static LCM_setting_table_V3 lcm_reader_mode_mid[] = {
    //5500K
    {0x39, 0xC1, 16, {0x01, 0x00, 0x0D, 0x14, 0x1E, 0x28, 0x2E, 0x35, 0x3E, 0x47,
                      0x4F, 0x5A, 0x63, 0x6B, 0x71, 0x79}},
    {0x29, 0xC1, 16, {0x80, 0x85, 0x8C, 0x93, 0x9A, 0xA0, 0xA5, 0xAC, 0xB3, 0xBA,
                      0xC3, 0xCC, 0xD7, 0xE2, 0xEF, 0xFA}},
    {0x29, 0xC1, 16, {0xFF, 0xFF, 0x61, 0x6F, 0x44, 0x74, 0xC4, 0xB7, 0x36, 0x00,
                      0xC0, 0x00, 0x0D, 0x14, 0x1E, 0x28}},
    {0x29, 0xC1, 16, {0x2E, 0x35, 0x3E, 0x47, 0x4F, 0x5A, 0x63, 0x6B, 0x71, 0x79,
                      0x80, 0x85, 0x8C, 0x93, 0x9A, 0xA0}},
    {0x29, 0xC1, 16, {0xA5, 0xAC, 0xB3, 0xBA, 0xC3, 0xCC, 0xD7, 0xE2, 0xEF, 0xFA,
                      0xFF, 0xFF, 0x61, 0x6F, 0x44, 0x74}},
    {0x29, 0xC1, 16, {0xC4, 0xB7, 0x36, 0x00, 0xC0, 0x00, 0x09, 0x10, 0x15, 0x1C,
                      0x23, 0x29, 0x2E, 0x33, 0x39, 0x3F}},
    {0x29, 0xC1, 16, {0x46, 0x4B, 0x52, 0x5A, 0x61, 0x66, 0x6C, 0x70, 0x75, 0x7B,
                      0x80, 0x84, 0x88, 0x8D, 0x92, 0x97}},
    {0x29, 0xC1, 15, {0x9C, 0xA0, 0xA4, 0xA8, 0xAE, 0xB2, 0x4A, 0x4A, 0x3D, 0x06,
                      0xDF, 0x07, 0xC0, 0x0E, 0xC0}},
};
static LCM_setting_table_V3 lcm_reader_mode_high[] = {
    //5000K
    {0x39, 0xC1, 16, {0x01, 0x00, 0x0D, 0x14, 0x1E, 0x28, 0x2E, 0x35, 0x3E, 0x47,
                      0x4F, 0x5A, 0x63, 0x6B, 0x71, 0x79}},
    {0x29, 0xC1, 16, {0x80, 0x85, 0x8C, 0x93, 0x9A, 0xA0, 0xA5, 0xAC, 0xB3, 0xBA,
                      0xC3, 0xCC, 0xD7, 0xE2, 0xEF, 0xFA}},
    {0x29, 0xC1, 16, {0xFF, 0xFF, 0x61, 0x6F, 0x44, 0x74, 0xC4, 0xB7, 0x36, 0x00,
                      0xC0, 0x00, 0x0D, 0x14, 0x1E, 0x28}},
    {0x29, 0xC1, 16, {0x2E, 0x35, 0x3E, 0x47, 0x4F, 0x5A, 0x63, 0x6B, 0x71, 0x79,
                      0x80, 0x85, 0x8C, 0x93, 0x9A, 0xA0}},
    {0x29, 0xC1, 16, {0xA5, 0xAC, 0xB3, 0xBA, 0xC3, 0xCC, 0xD7, 0xE2, 0xEF, 0xFA,
                      0xFF, 0xFF, 0x61, 0x6F, 0x44, 0x74}},
    {0x29, 0xC1, 16, {0xC4, 0xB7, 0x36, 0x00, 0xC0, 0x00, 0x07, 0x0F, 0x13, 0x19,
		      0x1F, 0x24, 0x2B, 0x2F, 0x32, 0x37}},
    {0x29, 0xC1, 16, {0x3D, 0x42, 0x48, 0x4C, 0x53, 0x5A, 0x5F, 0x65, 0x6A, 0x6D,
                      0x72, 0x77, 0x7C, 0x7F, 0x83, 0x87}},
    {0x29, 0xC1, 15, {0x8B, 0x8F, 0x93, 0x98, 0x9C, 0x9F, 0x50, 0x5D, 0x4E, 0x12,
		      0x79, 0xC5, 0xA3, 0xFC, 0xC0}},
};
static LCM_setting_table_V3 lcm_reader_mode_mono[] = {
    //off
    {0x15, 0xC1, 1, {0x00}},
};
static void lcm_reader_mode(LCM_READER_MODE mode)
{
	switch (mode) {
	case READER_MODE_OFF:
		dsi_set_cmdq_V3(lcm_reader_mode_off,
				sizeof(lcm_reader_mode_off) / sizeof(LCM_setting_table_V3), 1);
		break;
	case READER_MODE_LOW:
		dsi_set_cmdq_V3(lcm_reader_mode_low,
				sizeof(lcm_reader_mode_low) / sizeof(LCM_setting_table_V3), 1);
		break;
	case READER_MODE_MID:
		dsi_set_cmdq_V3(lcm_reader_mode_mid,
				sizeof(lcm_reader_mode_mid) / sizeof(LCM_setting_table_V3), 1);
		break;
	case READER_MODE_HIGH:
		dsi_set_cmdq_V3(lcm_reader_mode_high,
				sizeof(lcm_reader_mode_high) / sizeof(LCM_setting_table_V3), 1);
		break;
	case READER_MODE_MONO:
		dsi_set_cmdq_V3(lcm_reader_mode_mono,
				sizeof(lcm_reader_mode_mono) / sizeof(LCM_setting_table_V3), 1);
		break;
	default:
		break;
	}
	LCM_PRINT("READER MODE : %d\n", mode);
}
#endif
static LCM_setting_table_V3 lcm_initialization_setting[] = {

//HX8394A01+INX8

    {0x39,0xB9, 3,{0xFF,0x83,0x94}},

    {0x39,0xB1,17,{0x01,0x00,0x04,0xC3,0x03,0x12,0xF1,0x1C,0x24,0x3F,
                   0x3F,0x57,0x02,0x00,0xE6,0xE2,0xA6}},

    {0x39,0xB2, 6,{0x00,0xC8,0x0E,0x30,0x00,0x81}},

    {0x39,0xB4,31,{0x80,0x04,0x32,0x10,0x08,0x54,0x15,0x0F,0x22,0x10,
                   0x08,0x47,0x43,0x44,0x0A,0x4B,0x43,0x44,0x02,0x55,
                   0x55,0x02,0x06,0x44,0x06,0x5F,0x0A,0x6B,0x70,0x05,
                   0x08}},

    //{0x15,0xB6,1,{0x48}},  //for OTP

    {0x39,0xD5,54,{0x00,0x00,0x00,0x00,0x0A,0x00,0x01,0x22,0x00,0x22,
                   0x66,0x11,0x01,0x01,0x23,0x45,0x67,0x9A,0xBC,0xCC,
                   0xDD,0x45,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x67,
                   0x88,0x88,0x08,0x81,0x29,0x83,0x88,0x08,0x48,0x81,
                   0x85,0x28,0x68,0x83,0x87,0x88,0x48,0x85,0x00,0x00,
                   0x00,0x00,0x3C,0x10}},

    {0x15,0xCC, 1,{0x09}},

    {0x39,0xBF, 4,{0x06,0x02,0x10,0x04}},

    {0x39,0xC7, 4,{0x00,0x10,0x00,0x10}},

    {0x39,0xE0,42,{0x0C,0x11,0x10,0x2D,0x34,0x3F,0x0E,0x3E,0x02,0x0A,
                   0x0E,0x10,0x13,0x10,0x12,0x12,0x1A,0x0C,0x11,0x10,
                   0x38,0x3C,0x3F,0x0E,0x3E,0x02,0x0A,0x0E,0x10,0x13,
                   0x10,0x12,0x12,0x1A,0x07,0x12,0x07,0x13,0x07,0x12,
                   0x07,0x13}},
    {0x39,0xC1,127,{0x01,0x00,0x0D,0x15,0x1D,0x25,0x2D,0x35,0x3D,0x45,
		    0x4D,0x55,0x5D,0x65,0x6D,0x75,0x7D,0x85,0x8D,0x95,
		    0x9D,0xA5,0xAD,0xB5,0xBD,0xC5,0xCD,0xD5,0xDD,0xE5,
		    0xED,0xF5,0xFC,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
		    0x00,0x00,0x00,0x00,0x08,0x10,0x18,0x20,0x28,0x30,
		    0x38,0x40,0x48,0x50,0x58,0x60,0x68,0x70,0x78,0x80,
		    0x88,0x90,0x98,0xA0,0xA8,0xB0,0xB8,0xC0,0xC8,0xD0,
		    0xD8,0xE0,0xE8,0xF0,0xF8,0xFF,0x00,0x00,0x00,0x00,
		    0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x10,0x18,0x20,
		    0x28,0x30,0x38,0x40,0x48,0x50,0x58,0x60,0x68,0x70,
		    0x78,0x80,0x88,0x90,0x98,0xA0,0xA8,0xB0,0xB8,0xC0,
		    0xC8,0xD0,0xD8,0xE0,0xE8,0xF0,0xF8,0xFF,0x00,0x00,
		    0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x39,0xC0, 2,{0x0C,0x17}},

    {0x39,0xC6, 2,{0x08,0x08}},

    {0x15,0xD4, 1,{0x32}},

    //{0x15,0x35, 1,{0x00}},  //Open TE

    {0x05,0x11, 0,{}},  // Sleep-Out
    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,120,{}},

    {0x39,0xBA,17,{0x73,0x42,0x00,0x16,0xD5,0x40,0x10,0x00,0x00,0x24,
                   0x03,0x21,0x24,0x25,0x20,0x08,0x31}},

    {0x05,0x29, 0,{}},  // Display On
    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,20,{}},
};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.mode                       = SYNC_PULSE_VDO_MODE;   //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
    params->dsi.data_format.format         = LCM_DSI_FORMAT_RGB888;
    params->dsi.PS                         = LCM_PACKED_PS_24BIT_RGB888;
    params->type                           = LCM_TYPE_DSI;
    params->width                          = FRAME_WIDTH;
    params->height                         = FRAME_HEIGHT;

    // DSI
    params->dsi.LANE_NUM                   = LCM_FOUR_LANE;
    params->dsi.vertical_sync_active                       = 4;
    params->dsi.vertical_backporch                         = 12;
    params->dsi.vertical_frontporch                        = 16;   //10
    params->dsi.vertical_active_line       = FRAME_HEIGHT;
    params->dsi.horizontal_sync_active                     = 20;
    params->dsi.horizontal_backporch                       = 88;   //84
    params->dsi.horizontal_frontporch                      = 88;   //60
    params->dsi.horizontal_active_pixel    = FRAME_WIDTH;
    params->dsi.PLL_CLOCK                                  = 258;   //244

    params->physical_width                 = PHYSICAL_WIDTH;
    params->physical_height                = PHYSICAL_HEIGHT;

    // Non-continuous clock
    params->dsi.noncont_clock = TRUE;
    //params->dsi.noncont_clock_period = 2;   //Unit : frames
    params->dsi.clk_lp_per_line_enable              = 1;

    params->dsi.ssc_disable = 0;   //default enable SSC
    //params->dsi.ssc_range = 5;

    params->dsi.esd_check_enable                    = 1;
    params->dsi.customization_esd_check_enable      = 1;

    params->dsi.lcm_esd_check_table[0].cmd          = 0x45;
    params->dsi.lcm_esd_check_table[0].count        = 2;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x05;
    params->dsi.lcm_esd_check_table[0].para_list[1] = 0x1E;

    params->dsi.lcm_esd_check_table[1].cmd          = 0x09;
    params->dsi.lcm_esd_check_table[1].count        = 3;
    params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;
    params->dsi.lcm_esd_check_table[1].para_list[1] = 0x73;
    params->dsi.lcm_esd_check_table[1].para_list[2] = 0x04;

    params->dsi.lcm_esd_check_table[2].cmd          = 0xB1;   //0x01,0x00,0x04,0xC3,0x03,0x12,0xF1,0x1C,0x24,0x3F,
    params->dsi.lcm_esd_check_table[2].count        = 10;
    params->dsi.lcm_esd_check_table[2].para_list[0] = 0x7C;
    params->dsi.lcm_esd_check_table[2].para_list[1] = 0x00;
    params->dsi.lcm_esd_check_table[2].para_list[2] = 0x04;
    params->dsi.lcm_esd_check_table[2].para_list[3] = 0xC3;
    params->dsi.lcm_esd_check_table[2].para_list[4] = 0x03;
    params->dsi.lcm_esd_check_table[2].para_list[5] = 0x12;
    params->dsi.lcm_esd_check_table[2].para_list[6] = 0xF1;
    params->dsi.lcm_esd_check_table[2].para_list[7] = 0x1C;
    params->dsi.lcm_esd_check_table[2].para_list[8] = 0x24;
    params->dsi.lcm_esd_check_table[2].para_list[9] = 0x3F;

    params->dsi.lcm_esd_check_table[3].cmd          = 0xB4;   //0x80,0x04,0x32,0x10,0x08,0x54,0x15,0x0F,0x22,0x10,
    params->dsi.lcm_esd_check_table[3].count        = 10;
    params->dsi.lcm_esd_check_table[3].para_list[0] = 0x80;
    params->dsi.lcm_esd_check_table[3].para_list[1] = 0x04;
    params->dsi.lcm_esd_check_table[3].para_list[2] = 0x32;
    params->dsi.lcm_esd_check_table[3].para_list[3] = 0x10;
    params->dsi.lcm_esd_check_table[3].para_list[4] = 0x08;
    params->dsi.lcm_esd_check_table[3].para_list[5] = 0x54;
    params->dsi.lcm_esd_check_table[3].para_list[6] = 0x15;
    params->dsi.lcm_esd_check_table[3].para_list[7] = 0x0F;
    params->dsi.lcm_esd_check_table[3].para_list[8] = 0x22;
    params->dsi.lcm_esd_check_table[3].para_list[9] = 0x10;

}

static void lcm_reset(unsigned int ms1, unsigned int ms2, unsigned int ms3)
{
    if (ms1) {
        SET_RESET_PIN(1);
        MDELAY(ms1);
    }

    if (ms2) {
        SET_RESET_PIN(0);
        MDELAY(ms2);
    }

    if (ms3) {
        SET_RESET_PIN(1);
        MDELAY(ms3);
    }
}

static void lcm_init(void)
{
	if( lge_get_board_revno() <= HW_REV_A ){
		lcm_reset(5, 10, 50);
		dsi_set_cmdq_V3(lcm_patch_code, ARRAY_SIZE(lcm_patch_code), 1);
		MDELAY(10);
	}

#ifdef CONFIG_HQ_SET_LCD_BIAS
    SET_LCD_BIAS_VSPN(5200);    //Unit: mv
	if( lge_get_board_revno() <= HW_REV_A )
		SET_LCD_BIAS_VSPN_EN(ON, VSN_FIRST_VSP_AFTER);
	else
		SET_LCD_BIAS_VSPN_EN(ON, VSP_FIRST_VSN_AFTER);
    MDELAY(10);
#endif
	if( lge_get_board_revno() > HW_REV_A )
		lcm_reset(5, 10, 50);

    dsi_set_cmdq_V3(lcm_initialization_setting, ARRAY_SIZE(lcm_initialization_setting), 1);
}

static void lcm_suspend(void)
{
    lcm_reset(5, 10, 120);

#ifdef CONFIG_HQ_SET_LCD_BIAS
    SET_LCD_BIAS_VSPN_EN(OFF, VSN_FIRST_VSP_AFTER);
#endif
}

static void lcm_resume(void)
{
    LCM_PRINT("%s Init Start!\n", LCM_MODULE_NAME);

    lcm_init();

    LCM_PRINT("%s Init End!\n", LCM_MODULE_NAME);
}

#ifdef BUILD_LK
static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned int lcm_vol = 0;
    unsigned char buffer[3] = {0};
    unsigned int data_array[16] = {0};

	if( lge_get_board_revno() <= HW_REV_A ){
		lcm_reset(1, 5, 10);
		dsi_set_cmdq_V3(lcm_patch_code, ARRAY_SIZE(lcm_patch_code), 1);
		MDELAY(5);
	}

#ifdef CONFIG_HQ_SET_LCD_BIAS
    SET_LCD_BIAS_VSPN(5200);    //Unit: mv
	if( lge_get_board_revno() <= HW_REV_A )
		SET_LCD_BIAS_VSPN_EN(ON, VSN_FIRST_VSP_AFTER);
	else
		SET_LCD_BIAS_VSPN_EN(ON, VSP_FIRST_VSN_AFTER);
    MDELAY(10);
#endif
	if( lge_get_board_revno() > HW_REV_A )
		lcm_reset(1, 5, 20);

    data_array[0] = 0x00033700;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);

    read_reg_v2(0x04, buffer, 3);   //DA -> 83; DB -> 94; DC -> 1A
    id = (buffer[1] << 8) | buffer[2];

#ifdef AUX_ADC_EXTERNAL_CHANNEL
    unsigned int data[4] = {0, 0, 0,0};
    unsigned int rawdata = 0;
    int res = 0;

    res = IMM_GetOneChannelValue(AUX_ADC_EXTERNAL_CHANNEL, data, &rawdata);
    if(res < 0) {
        LCM_PRINT("[%s]:%s get gpio status error!\n", LCM_MODULE_NAME, __func__);
        return FALSE;
    }

    lcm_vol = data[0] * 1000 + data[1] * 10;
#endif

    LCM_PRINT("[%s]:%s, buf:0x%x, 0x%x, 0x%x, lcm_vol:0x%x\n", LCM_MODULE_NAME, __func__, buffer[0], buffer[1], buffer[2], lcm_vol);

    return (LCM_ID == id) ? TRUE : FALSE;
}
#endif

#if defined(CONFIG_LGE_INIT_CMD_TUNING)
static LCM_setting_table_V3* lcm_init_cmd_addr(void)
{
	return lcm_initialization_setting;
}

static int lcm_init_cmd_size(void)
{
	return (sizeof(lcm_initialization_setting)/sizeof(LCM_setting_table_V3));
}
#endif

LCM_DRIVER hx8394a_guoxian_inx_b026_zal2017_lcm_drv =
{
    .name           = LCM_MODULE_NAME,
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
#if defined(CONFIG_LGE_INIT_CMD_TUNING)
	.get_init_cmd_addr = lcm_init_cmd_addr,
	.get_init_cmd_size = lcm_init_cmd_size,
#endif
#ifdef BUILD_LK
    .compare_id     = lcm_compare_id,
#endif
#ifdef CONFIG_LGE_READER_MODE
    .reader_mode = lcm_reader_mode,
#endif
};
