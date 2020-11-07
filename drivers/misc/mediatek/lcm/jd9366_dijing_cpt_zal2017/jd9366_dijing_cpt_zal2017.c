#ifdef BUILD_LK
#include <sys/types.h>
#else
#include <linux/string.h>
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

#define LCM_MODULE_NAME    "JD9366_DJ"

#ifdef BUILD_LK
#define LCM_ID                    (0x9366)
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

static struct LCM_setting_table lcm_initialization_setting[] = {
//------JD9366+CPT 8.0(080WQ) initial code------//

    //Page0
    {0xE0,1,{0x00}},

    //Password
    {0xE1,1,{0x93}},
    {0xE2,1,{0x65}},
    {0xE3,1,{0xF8}},
    {0x80,1,{0x03}},

    //Page1
    {0xE0,1,{0x01}},

    //Set Vcom
    //{0x00,1,{0x00}},
    //{0x01,1,{0x9C}},//0xB7

    //Set Gamma Power, VGMP,VGMN,VGSP,VGSN
    {0x17,1,{0x00}},
    {0x18,1,{0xAF}},//4.3V
    {0x19,1,{0x01}},
    {0x1A,1,{0x00}},
    {0x1B,1,{0xAF}},
    {0x1C,1,{0x01}},

    //Set Gate Power
    {0x1F,1,{0x3E}},
    {0x20,1,{0x28}},
    {0x21,1,{0x28}},
    {0x22,1,{0x0E}},

    {0x24,1,{0xC8}},//0x38, [7]:VGH_EN=1, [6]:VGL_EN=1,[5]:AVDD_EN=0,[4]:AVEE_EN=0

    //Set RgbCyc
    {0x37,1,{0x29}},//[5:4]ENZ[1:0]=10, [3]SS=1, [0]BGR=1
    {0x38,1,{0x05}},//JDT=101 Zigzag inversion
    {0x39,1,{0x08}},//RGB_N_EQ1, modify 20140806
    {0x3A,1,{0x12}},//RGB_N_EQ2, modify 20140806
    {0x3C,1,{0x78}},//SET EQ3 for TE_H
    {0x3D,1,{0xFF}},//SET CHGEN_ON, modify 20140827
    {0x3E,1,{0xFF}},//SET CHGEN_OFF, modify 20140827
    {0x3F,1,{0xFF}},//SET CHGEN_OFF2, modify 20140827

    //Set Tcom
    {0x40,1,{0x06}},//RSO=800 Pixels
    {0x41,1,{0xA0}},//LN=640->1280 line
    {0x43,1,{0x15}},//VFP
    {0x44,1,{0x12}},//VBP
    {0x45,1,{0x50}},//HBP
    {0x4B,1,{0x04}},

    //Power Voltage
    {0x55,1,{0x0F}},//DCDCM=1111, External pwoer ic
    {0x56,1,{0x01}},
    {0x57,1,{0x89}},
    {0x58,1,{0x0A}},
    {0x59,1,{0x2A}},//VCL = -2.9V
    {0x5A,1,{0x31}},//VGH = 19V
    {0x5B,1,{0x15}},//VGL = -11V

    //S-Gamma
    {0x5D,1,{0x7F}},
    {0x5E,1,{0x61}},
    {0x5F,1,{0x4D}},
    {0x60,1,{0x44}},
    {0x61,1,{0x46}},
    {0x62,1,{0x34}},
    {0x63,1,{0x36}},
    {0x64,1,{0x1E}},
    {0x65,1,{0x30}},
    {0x66,1,{0x27}},
    {0x67,1,{0x1F}},
    {0x68,1,{0x36}},
    {0x69,1,{0x1F}},
    {0x6A,1,{0x28}},
    {0x6B,1,{0x26}},
    {0x6C,1,{0x23}},
    {0x6D,1,{0x18}},
    {0x6E,1,{0x09}},
    {0x6F,1,{0x00}},

    {0x70,1,{0x7F}},
    {0x71,1,{0x61}},
    {0x72,1,{0x4D}},
    {0x73,1,{0x44}},
    {0x74,1,{0x46}},
    {0x75,1,{0x34}},
    {0x76,1,{0x36}},
    {0x77,1,{0x1E}},
    {0x78,1,{0x30}},
    {0x79,1,{0x27}},
    {0x7A,1,{0x1F}},
    {0x7B,1,{0x36}},
    {0x7C,1,{0x1F}},
    {0x7D,1,{0x28}},
    {0x7E,1,{0x26}},
    {0x7F,1,{0x23}},
    {0x80,1,{0x18}},
    {0x81,1,{0x09}},
    {0x82,1,{0x00}},

    //Page2, for GIP
    {0xE0,1,{0x02}},

    //GIP_L Pin mapping
    {0x00,1,{0x00}},
    {0x01,1,{0x04}},
    {0x02,1,{0x08}},
    {0x03,1,{0x05}},
    {0x04,1,{0x09}},
    {0x05,1,{0x06}},
    {0x06,1,{0x0A}},
    {0x07,1,{0x07}},
    {0x08,1,{0x0B}},
    {0x09,1,{0x1F}},
    {0x0A,1,{0x1F}},
    {0x0B,1,{0x1F}},
    {0x0C,1,{0x1F}},
    {0x0D,1,{0x1F}},
    {0x0E,1,{0x1F}},
    {0x0F,1,{0x17}},
    {0x10,1,{0x37}},
    {0x11,1,{0x10}},
    {0x12,1,{0x1F}},
    {0x13,1,{0x1F}},
    {0x14,1,{0x1F}},
    {0x15,1,{0x1F}},

    //GIP_R Pin mapping
    {0x16,1,{0x00}},
    {0x17,1,{0x04}},
    {0x18,1,{0x08}},
    {0x19,1,{0x05}},
    {0x1A,1,{0x09}},
    {0x1B,1,{0x06}},
    {0x1C,1,{0x0A}},
    {0x1D,1,{0x07}},
    {0x1E,1,{0x0B}},
    {0x1F,1,{0x1F}},
    {0x20,1,{0x1F}},
    {0x21,1,{0x1F}},
    {0x22,1,{0x1F}},
    {0x23,1,{0x1F}},
    {0x24,1,{0x1F}},
    {0x25,1,{0x17}},
    {0x26,1,{0x37}},
    {0x27,1,{0x10}},
    {0x28,1,{0x1F}},
    {0x29,1,{0x1F}},
    {0x2A,1,{0x1F}},
    {0x2B,1,{0x1F}},

    //GIP Timing
    {0x58,1,{0x01}},
    {0x59,1,{0x00}},
    {0x5A,1,{0x00}},
    {0x5B,1,{0x00}},
    {0x5C,1,{0x0C}},//STV_S0
    {0x5D,1,{0x60}},
    {0x5E,1,{0x00}},
    {0x5F,1,{0x00}},
    {0x60,1,{0x30}},
    {0x61,1,{0x00}},
    {0x62,1,{0x00}},
    {0x63,1,{0x03}},//STV_ON
    {0x64,1,{0x6A}},//STV_OFF
    {0x65,1,{0x45}},
    {0x66,1,{0x14}},//for G1280 1 line delay with RST
    {0x67,1,{0x73}},
    {0x68,1,{0x10}},//STV_S0
    {0x69,1,{0x06}},//CKV_ON
    {0x6A,1,{0x6A}},//CKV_OFF
    {0x6B,1,{0x00}},
    {0x6C,1,{0x00}},
    {0x6D,1,{0x03}},
    {0x6E,1,{0x00}},
    {0x6F,1,{0x08}},
    {0x70,1,{0x00}},
    {0x71,1,{0x00}},
    {0x72,1,{0x06}},
    {0x73,1,{0x7B}},
    {0x74,1,{0x00}},
    {0x75,1,{0x80}},
    {0x76,1,{0x00}},
    {0x77,1,{0x05}},
    {0x78,1,{0x1B}},
    {0x79,1,{0x00}},
    {0x7A,1,{0x00}},
    {0x7B,1,{0x00}},
    {0x7C,1,{0x00}},
    {0x7D,1,{0x03}},
    {0x7E,1,{0x7B}},

    //Page4
    {0xE0,1,{0x04}},
    {0x09,1,{0x10}},
    {0x2B,1,{0x2B}},
    {0x2E,1,{0x44}},
    //{0x2D,1,{0x03}},

    //Page0
    {0xE0,1,{0x00}},
    {0xE6,1,{0x02}},
    {0xE7,1,{0x06}},

    //Open TE
    //{0x35,1,{0x00}},

    //Sleep Out
    {0x11,1,{0x00}},
    {REGFLAG_DELAY,120,{}},

    //Display On
    {0x29,1,{0x00}},
    {REGFLAG_DELAY,5,{}},

    //End of Table
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
    // Display off sequence
    {REGFLAG_DELAY, 1, {}},
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
};
#endif

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
    params->dsi.vertical_backporch                         = 15;
    params->dsi.vertical_frontporch                        = 21;
    params->dsi.vertical_active_line       = FRAME_HEIGHT;
    params->dsi.horizontal_sync_active                     = 40;
    params->dsi.horizontal_backporch                       = 40;
    params->dsi.horizontal_frontporch                      = 40;
    params->dsi.horizontal_active_pixel    = FRAME_WIDTH;
    params->dsi.PLL_CLOCK                                  = 250;

    params->physical_width                 = PHYSICAL_WIDTH;
    params->physical_height                = PHYSICAL_HEIGHT;

    // Non-continuous clock
    params->dsi.noncont_clock = TRUE;
    //params->dsi.noncont_clock_period = 2; // Unit : frames
    params->dsi.clk_lp_per_line_enable              = 1;

    params->dsi.ssc_disable = 0;//default enable SSC
    //params->dsi.ssc_range = 5;

    params->dsi.esd_check_enable                    = 1;
    params->dsi.customization_esd_check_enable      = 1;

    params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
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
#ifdef CONFIG_HQ_SET_LCD_BIAS
    SET_LCD_BIAS_VSPN(5200);    //Unit: mv
    SET_LCD_BIAS_VSPN_EN(ON, VSP_FIRST_VSN_AFTER);
    MDELAY(10);
#endif

    lcm_reset(5, 10, 120);

    push_table(lcm_initialization_setting, ARRAY_SIZE(lcm_initialization_setting), 1);
}

static void lcm_suspend(void)
{
    lcm_reset(0, 120, 0);
    //push_table(lcm_sleep_in_setting, ARRAY_SIZE(lcm_sleep_in_setting), 1);

#ifdef CONFIG_HQ_SET_LCD_BIAS
    SET_LCD_BIAS_VSPN_EN(OFF, VSN_FIRST_VSP_AFTER);
#endif
}

static void lcm_resume(void)
{
    LCM_PRINT("%s Init Start!\n", LCM_MODULE_NAME);

    lcm_init();
#if 0
#ifdef CONFIG_HQ_SET_LCD_BIAS
    SET_LCD_BIAS_VSPN(5200);
    SET_LCD_BIAS_VSPN_EN(ON, VSP_FIRST_VSN_AFTER);
    MDELAY(10);
#endif
    push_table(lcm_sleep_out_setting, ARRAY_SIZE(lcm_sleep_out_setting), 1);
#endif

    LCM_PRINT("%s Init End!\n", LCM_MODULE_NAME);
}

#ifdef BUILD_LK
static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned int lcm_vol = 0;
    unsigned char buffer[3] = {0};
    unsigned int data_array[16] = {0};

    lcm_reset(1, 5, 20);

    data_array[0] = 0x00033700;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);

    read_reg_v2(0x04, buffer, 3);

    id = (buffer[0] << 8) | buffer[1];

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

LCM_DRIVER jd9366_dijing_cpt_zal2017_lcm_drv =
{
    .name           = "JD9366_DJ_CPT",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
#ifdef BUILD_LK
    .compare_id     = lcm_compare_id,
#endif
};
