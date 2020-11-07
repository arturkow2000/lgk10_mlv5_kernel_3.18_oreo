/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 HI1333mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi1333_mipi_raw_Sensor.h"

#define PFX "HI1333_camera_sensor"

#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#define LOG_DEBUG(format, args...)	pr_err(PFX "[%s] " format, __FUNCTION__, ##args) /*LGE_CHANGE, 2017-12-11, added camera log, soojong.jin@lge.com*/
#define ENABLE_PDAF 1
#define e2prom 1

bool pip_mode = 0;

// [LGE_UPDATE_S] [kyunghun.oh@lge.com] [2017-09-18] Creation of node for camera module identification.
extern int iReadReg(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId);
extern char rear_sensor_name[20];
extern int g_mainSensor; 
// [LGE_UPDATE_E] [kyunghun.oh@lge.com] [2017-09-18] Creation of node for camera module identification.
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI1333_SENSOR_ID,

	.checksum_value = 0xabaa55c3,	/* 0x6d01485c // Auto Test Mode ÃßÈÄ.. */
	.pre = {
		.pclk = 600000000,	/* record different mode's pclk */
		.linelength = 5952,	/* record different mode's linelength */
		.framelength = 3360,	/* 3300,                     //record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 2104,	/* 2104,               //record different mode's width of grabwindow */
		.grabwindow_height = 1560,	/* record different mode's height of grabwindow */
		/*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		.mipi_data_lp2hs_settle_dc = 85,
		/*       following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
		},
	.cap = {
		.pclk = 600000000,
		.linelength = 5952,
		.framelength = 3360,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		},
	/* need to setting */
	.cap1 = {
		 /* capture for PIP 24fps relative information, capture1 mode must use same framelength,
		    linelength with Capture mode for shutter calculate */
		 .pclk = 300000000,
		 .linelength = 5952,
		 .framelength = 3360,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 4208,
		 .grabwindow_height = 3120,
		 .mipi_data_lp2hs_settle_dc = 85,
		 .max_framerate = 150,
		 },
	.normal_video = {
			 .pclk = 600000000,
			 .linelength = 5952,
			 .framelength = 3360,
			 .startx = 0,
			 .starty = 0,
			 .grabwindow_width = 4208,
			 .grabwindow_height = 3120,
			 .mipi_data_lp2hs_settle_dc = 85,
			 .max_framerate = 300,
			 },
	.hs_video = {
		     .pclk = 600000000,
		     .linelength = 5952,
		     .framelength = 840,	/* 832, */
		     .startx = 0,
		     .starty = 0,
		     .grabwindow_width = 640,
		     .grabwindow_height = 480,
		     .mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		     .max_framerate = 1200,
		     },
	.slim_video = {
		       .pclk = 600000000,
		       .linelength = 5952,
		       .framelength = 840,	/* 832, */
		       .startx = 0,
		       .starty = 0,
		       .grabwindow_width = 1280,
		       .grabwindow_height = 720,
		       .mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		       .max_framerate = 1200,	/* 1200, */
		       },
	/* .margin = 6, */
	.margin = 7,
	.min_shutter = 7,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 5,	/* support sensor mode num */

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 3,	/* enter slim video delay frame num */

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	/* .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B, */
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x40, 0xff},
	.i2c_speed = 400,
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	   INIT, Preview, Capture, Video,High Speed Video, Slim Video */
	.shutter = 0x0100,	/* current shutter */
	.gain = 0xe0,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,
	/* auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker */
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	/* current scenario id */
	.ihdr_en = 0,
	.i2c_write_id = 0x40,
};

static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{4240, 3124, 0, 0, 4240, 3124, 2120, 1562, 16, 2, 2104, 1560, 0, 0, 2104, 1560},
	/* preview (2104 x 1560) */
	{4240, 3124, 0, 0, 4240, 3124, 4240, 3124, 16, 2, 4208, 3120, 0, 0, 4208, 3120},
	/* capture (4208 x 3120) */
	{4240, 3124, 0, 0, 4240, 3124, 4240, 3124, 16, 2, 4208, 3120, 0, 0, 4208, 3120},
	/* VIDEO (4208 x 3120) */
	{4240, 3124, 0, 110, 4240, 2904, 704, 484, 34, 2, 640, 480, 0, 0, 640, 480},
	/* hight speed video (640 x 480) */
	{4240, 3124, 0, 476, 4240, 2172, 1408, 724, 66, 2, 1280, 720, 0, 0, 1280, 720},
	/* slim video (1280 x 720) */
};

#if ENABLE_PDAF

/* VC1 for HDR(N/A) VC2 for PDAF(VC1, DT=2B, unit : 10bit */
SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting */
	{0x02,			/* VC_Num */
	 0x0a,			/* VC_PixelNum */
	 0x00, /* ModeSelect    0:auto 1:direct */
	 0x00, /* EXPO_Ratio    1/1, 1/2, 1/4, 1/8 */
	 0x00, /* 0DValue    0D Value */
	 0x00, /* RG_STATSMODE  STATS divistion mode 0:16x16  1:8x8  2:4x4  3:1x1 */
	 0x00, 0x2b, 0x0838, 0x0618,	/* VC0 Maybe image data? */
	 0x00, 0x00, 0x0000, 0x0000,	/* VC1 MVHDR */
	 0x01, 0x2B, 0x0100, 0x0180,	/* VC2 PDAF */
	 0x00, 0x00, 0x0000, 0x0000},	/* VC3 ?? */
	/* Capture mode setting */
	{0x02,			/* VC_Num */
	 0x0a,			/* VC_PixelNum */
	 0x00, /* ModeSelect    0:auto 1:direct */
	 0x00, /* EXPO_Ratio    1/1, 1/2, 1/4, 1/8 */
	 0x00, /* 0DValue               0D Value */
	 0x00, /* RG_STATSMODE  STATS divistion mode 0:16x16  1:8x8  2:4x4  3:1x1 */
	 0x00, 0x2b, 0x1070, 0x0C30,	/* VC0 Maybe image data? */
	 0x00, 0x00, 0x0000, 0x0000,	/* VC1 MVHDR */
	 0x01, 0x2B, 0x0100, 0x0180,	/* VC2 PDAF */
	 0x00, 0x00, 0x0000, 0x0000},	/* VC3 ?? */
	/* Video mode setting */
	{0x02,			/* VC_Num */
	 0x0a,			/* VC_PixelNum */
	 0x00, /* ModeSelect    0:auto 1:direct */
	 0x00, /* EXPO_Ratio    1/1, 1/2, 1/4, 1/8 */
	 0x00, /* 0DValue               0D Value */
	 0x00, /* RG_STATSMODE  STATS divistion mode 0:16x16  1:8x8  2:4x4  3:1x1 */
	 0x00, 0x2b, 0x1070, 0x0C30,	/* VC0 Maybe image data? */
	 0x00, 0x00, 0x0000, 0x0000,	/* VC1 MVHDR */
	 0x01, 0x2B, 0x0100, 0x0180,	/* VC2 PDAF */
	 0x00, 0x00, 0x0000, 0x0000},	/* VC3 ?? */
};

static SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 56,
	.i4OffsetY = 24,
	.i4PitchX = 64,
	.i4PitchY = 64,
	.i4PairNum = 16,
	.i4SubBlkW = 16,
	.i4SubBlkH = 16,
	.i4PosL = {
		   {67, 36}, {79, 36}, {99, 36}, {111, 36},
		   {63, 48}, {83, 48}, {95, 48}, {115, 48},
		   {67, 68}, {79, 68}, {99, 68}, {111, 68},
		   {63, 80}, {83, 80}, {95, 80}, {115, 80}
		   },
	.i4PosR = {
		   {67, 32}, {79, 32}, {99, 32}, {111, 32},
		   {63, 52}, {83, 52}, {95, 52}, {115, 52},
		   {67, 64}, {79, 64}, {99, 64}, {111, 64},
		   {63, 84}, {83, 84}, {95, 84}, {115, 84}
		   },
	.i4BlockNumX = 64,
	.i4BlockNumY = 48,
};

static SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning = {
	.i4OffsetX = 28,
	.i4OffsetY = 12,
	.i4PitchX = 32,
	.i4PitchY = 32,
	.i4PairNum = 16,
	.i4SubBlkW = 8,
	.i4SubBlkH = 8,
	.i4PosL = {
		   {35, 18}, {39, 18}, {51, 18}, {55, 18},
		   {31, 24}, {43, 24}, {47, 24}, {59, 24},
		   {35, 34}, {39, 34}, {51, 34}, {55, 34},
		   {31, 40}, {43, 40}, {47, 40}, {59, 40}
		   },
	.i4PosR = {
		   {35, 16}, {39, 16}, {51, 16}, {55, 16},
		   {31, 26}, {43, 26}, {47, 26}, {59, 26},
		   {35, 32}, {39, 32}, {51, 32}, {55, 32},
		   {31, 42}, {43, 42}, {47, 42}, {59, 42}
		   },
	.i4BlockNumX = 64,
	.i4BlockNumY = 48,
};
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	kdSetI2CSpeed(imgsensor_info.i2c_speed);	/* Add this func to set i2c speed by each sensor */
	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {
		(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)
	};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor1D(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0006, imgsensor.frame_length);

	if (pip_mode)
		write_cmos_sensor(0x0008, imgsensor.line_length / 4);
	else
		write_cmos_sensor(0x0008, imgsensor.line_length / 8);

}				/*      set_dummy  */

// [LGE_UPDATE_S] [kyunghun.oh@lge.com] [2017-09-18] Creation of node for camera module identification.
static int hi1333_read_eepromVendorID(void)
{
    int retry = 3;
    u8 vendorID_readFrom_rear_imageSensor = 0;
    for(; retry > 0; retry--) {
        if(iReadReg(0x0be0, &vendorID_readFrom_rear_imageSensor, 0xa0) == 0)
            break;
        mdelay(1);
    }
    printk("[hi1333_read_eepromVendorID] read eeprom id = %d\n", vendorID_readFrom_rear_imageSensor);
    if(vendorID_readFrom_rear_imageSensor != 0x14)
    {
        printk("[hi1333_read_eepromVendorID] matching fail!! It should be 0x01 but read %d\n", vendorID_readFrom_rear_imageSensor);
        return -1;
    }
    else
    {
        printk("[hi1333_read_eepromVendorID] matching OK\n");
        strcpy(rear_sensor_name, "sl1333_imtech");
        return 0;
    }
}
// [LGE_UPDATE_E] [kyunghun.oh@lge.com] [2017-09-18] Creation of node for camera module identification.
static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/* kal_int16 dummy_line; */
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
	    (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();

}				/*      set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	/* kal_uint32 frame_length = 0; */

	LOG_INF("E\n");
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	/* OV Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter =
	    (shutter >
	     (imgsensor_info.max_frame_length -
	      imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
					 imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps =
		    imgsensor.pclk * 10 / (imgsensor.line_length * imgsensor.frame_length);
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {

			LOG_INF("Enable autoflicker\n");
			write_cmos_sensor(0x0006, imgsensor.frame_length);
		}
	} else {
		LOG_INF("Disable autoflicker\n");
		/* Extend frame length */
		write_cmos_sensor(0x0006, imgsensor.frame_length);
	}

	write_cmos_sensor1D(0x0073, (shutter & 0x0F0000) >> 16);
	write_cmos_sensor1D(0x0074, (shutter & 0x00FF00) >> 8);
	write_cmos_sensor1D(0x0075, (shutter & 0x0000FF));

	LOG_INF("frame_length = %d , shutter = %d\n", imgsensor.frame_length, shutter);


}				/*      write_shutter  */



/*************************************************************************
* FUNCTION
*	enable set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}				/*      set_shutter */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = gain / 4 - 16;

	return (kal_uint16) reg_gain;

}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	LOG_INF("set_gain is %d\n", gain);

	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;	/* set current gain to global */
	spin_unlock(&imgsensor_drv_lock);

	write_cmos_sensor(0x0077, reg_gain);

	return gain;
}				/*      set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	return;
}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}				/*      night_mode      */


/*************************************************************************
* FUNCTION
*	Initial, preview, capture(normal_video), hs_video, slim_video
*
* DESCRIPTION
*	Normal write setting
*
* RETURNS
*	None
*
*************************************************************************/

void hi1333_init_setting_normal(void)
{
    /* //// */
    /* Initialize */
    /* /// */

//write_cmos_sensor(0x0a00, 0x0000);  //stream off
////////////////////////////////
//---------TG firmware--------//
////////////////////////////////

// Firmware tg_pt
write_cmos_sensor(0x0a00, 0x0000); //stream off
write_cmos_sensor(0x2ffe, 0xd800);
write_cmos_sensor(0x0e04, 0x0013);
write_cmos_sensor(0x3048, 0x5020);
write_cmos_sensor(0x0f30, 0x001f);
write_cmos_sensor(0x0f36, 0x001f);
write_cmos_sensor(0x0c00, 0x11d8);
write_cmos_sensor(0x0c02, 0x0011);
write_cmos_sensor(0x0c04, 0x5000);
write_cmos_sensor(0x0c06, 0x01eb);
write_cmos_sensor(0x0c10, 0x0040);
write_cmos_sensor(0x0c12, 0x0040);
write_cmos_sensor(0x0c14, 0x0040);
write_cmos_sensor(0x0c16, 0x0040);
write_cmos_sensor(0x0c18, 0x8000);
write_cmos_sensor(0x0c62, 0x0194);
write_cmos_sensor(0x0c64, 0x0286);
write_cmos_sensor(0x0c66, 0x0294);
write_cmos_sensor(0x0c68, 0x0100);
write_cmos_sensor(0x0cb2, 0x0200);
write_cmos_sensor(0x0714, 0xe8e8);
write_cmos_sensor(0x0716, 0xede8);
write_cmos_sensor(0x000e, 0x0100);
write_cmos_sensor(0x0a10, 0x400c);
write_cmos_sensor(0x003e, 0x0000);
write_cmos_sensor(0x0074, 0x0d18);
write_cmos_sensor(0x0a04, 0x036a);
write_cmos_sensor(0x0076, 0x0000);
write_cmos_sensor(0x0724, 0x0f1f);
write_cmos_sensor(0x0068, 0x0703);
write_cmos_sensor(0x0060, 0x0008);
write_cmos_sensor(0x0062, 0x0200);
write_cmos_sensor(0x075e, 0x0535);
write_cmos_sensor(0x0012, 0x0fcd);
write_cmos_sensor(0x0806, 0x0002);
write_cmos_sensor(0x0900, 0x0300);
write_cmos_sensor(0x0902, 0xc319);
write_cmos_sensor(0x095a, 0x0099);
write_cmos_sensor(0x095c, 0x1111);
write_cmos_sensor(0x095e, 0xbac0);
write_cmos_sensor(0x0960, 0x5dae);
write_cmos_sensor(0x0a38, 0x080c);
write_cmos_sensor(0x0a3a, 0x140c);
write_cmos_sensor(0x0a3c, 0x280c);
write_cmos_sensor(0x0a3e, 0x340c);
write_cmos_sensor(0x0a40, 0x0418);
write_cmos_sensor(0x0a42, 0x1818);
write_cmos_sensor(0x0a44, 0x2418);
write_cmos_sensor(0x0a46, 0x3818);
write_cmos_sensor(0x0a48, 0x082c);
write_cmos_sensor(0x0a4a, 0x142c);
write_cmos_sensor(0x0a4c, 0x282c);
write_cmos_sensor(0x0a4e, 0x342c);
write_cmos_sensor(0x0a50, 0x0438);
write_cmos_sensor(0x0a52, 0x1838);
write_cmos_sensor(0x0a54, 0x2438);
write_cmos_sensor(0x0a56, 0x3838);
write_cmos_sensor(0x0a58, 0x0808);
write_cmos_sensor(0x0a5a, 0x1408);
write_cmos_sensor(0x0a5c, 0x2808);
write_cmos_sensor(0x0a5e, 0x3408);
write_cmos_sensor(0x0a60, 0x041c);
write_cmos_sensor(0x0a62, 0x181c);
write_cmos_sensor(0x0a64, 0x241c);
write_cmos_sensor(0x0a66, 0x381c);
write_cmos_sensor(0x0a68, 0x0828);
write_cmos_sensor(0x0a6a, 0x1428);
write_cmos_sensor(0x0a6c, 0x2828);
write_cmos_sensor(0x0a6e, 0x3428);
write_cmos_sensor(0x0a70, 0x043c);
write_cmos_sensor(0x0a72, 0x183c);
write_cmos_sensor(0x0a74, 0x243c);
write_cmos_sensor(0x0a76, 0x383c);
write_cmos_sensor(0x0404, 0x015c);
write_cmos_sensor(0x0406, 0x0138);
write_cmos_sensor(0x040a, 0x0115);
write_cmos_sensor(0x040c, 0x022a);
write_cmos_sensor(0x040e, 0xbc52);
write_cmos_sensor(0x0410, 0x0056);
write_cmos_sensor(0x0412, 0x00ac);
write_cmos_sensor(0x0414, 0x6907);
write_cmos_sensor(0x0422, 0x0011);
write_cmos_sensor(0x0424, 0x0023);
write_cmos_sensor(0x0426, 0x2f15);
write_cmos_sensor(0x0428, 0x0016);
write_cmos_sensor(0x042a, 0x002b);
write_cmos_sensor(0x042c, 0x3483);
write_cmos_sensor(0x0324, 0x0100);
write_cmos_sensor(0x0600, 0x0c3e);
write_cmos_sensor(0x0a78, 0x0400);

}

void hi1333_preview_setting_normal(void)
{

    write_cmos_sensor(0x0a00, 0x0000);	/* stream off */

//Sensor Information////////////////////////////
//Sensor	  : SL-1333
//Date		  : 2018-01-19
//Customer        : LGE
//Image size	  : 2104x1560
//MCLK		  : 24MHz
//MIPI speed(Mbps): 726Mbps x 4Lane | Continuous mode
//Frame Length	  : 3359
//Line Length 	  : 5952
//Max Fps 	  : 30.0fps
//Pixel order 	  : Green 1st (=GB)
//X/Y-flip	  : X-flip
//BLC offse	  : 64code
////////////////////////////////////////////////
write_cmos_sensor(0x0f38, 0x0379);
write_cmos_sensor(0x0f3a, 0x4107);
write_cmos_sensor(0x093e, 0x0100);
write_cmos_sensor(0x0920, 0xc103);
write_cmos_sensor(0x0922, 0x030d);
write_cmos_sensor(0x0924, 0x0203);
write_cmos_sensor(0x0926, 0x0606);
write_cmos_sensor(0x0928, 0x0704);
write_cmos_sensor(0x092a, 0x0505);
write_cmos_sensor(0x092c, 0x0a00);
write_cmos_sensor(0x0910, 0x033e);
write_cmos_sensor(0x0912, 0x0054);
write_cmos_sensor(0x0914, 0x0002);
write_cmos_sensor(0x0916, 0x0002);
write_cmos_sensor(0x091a, 0x0001);
write_cmos_sensor(0x0938, 0x4100);	/* 0x4100); //mipi_virtual_channel_ctrl */
write_cmos_sensor(0x0904, 0x2bab);
write_cmos_sensor(0x0942, 0x0108);	/* 0x0108); //mipi_pd_sep_ctrl1, mipi_pd_sep_ctrl2 */
write_cmos_sensor(0x0944, 0x0100);
write_cmos_sensor(0x0946, 0x0004);
write_cmos_sensor(0x0520, 0x0100);
write_cmos_sensor(0x0532, 0x0100);
write_cmos_sensor(0x0a2a, 0x8060);
write_cmos_sensor(0x0a2c, 0x2020);
write_cmos_sensor(0x0a32, 0x0301);
write_cmos_sensor(0x0a26, 0x0048);
write_cmos_sensor(0x0a28, 0x001c);
write_cmos_sensor(0x0a36, 0x0000);
write_cmos_sensor(0x0408, 0x0000);
write_cmos_sensor(0x0418, 0x0000);
write_cmos_sensor(0x0800, 0x0400);
write_cmos_sensor(0x0008, 0x02e8);
write_cmos_sensor(0x000c, 0x000c);
write_cmos_sensor(0x0804, 0x0008);
write_cmos_sensor(0x0026, 0x003c);
write_cmos_sensor(0x002c, 0x0c71);
write_cmos_sensor(0x005c, 0x0204);
write_cmos_sensor(0x002e, 0x1111);
write_cmos_sensor(0x0032, 0x3311);
write_cmos_sensor(0x0006, 0x0d1f);
write_cmos_sensor(0x0074, 0x0d18);
write_cmos_sensor(0x0a0e, 0x0002);
write_cmos_sensor(0x0a12, 0x0838);
write_cmos_sensor(0x0a14, 0x0618);
write_cmos_sensor(0x075c, 0x0100);
write_cmos_sensor(0x0050, 0x4300);
write_cmos_sensor(0x0722, 0x0700);
write_cmos_sensor(0x004c, 0x0100);


write_cmos_sensor(0x0a00, 0x0100);  // stream on
};


void hi1333_capture_setting_normal(void)
{
//Sensor Information////////////////////////////
//Sensor	  : SL-1333
//Date		  : 2018-01-19
//Customer        : LGE
//Image size	  : 4208x3120
//MCLK		  : 24MHz
//MIPI speed(Mbps): 1.452Gbps x 4Lane | Continuous mode
//Frame Length	  : 3359
//Line Length 	  : 5952
//Max Fps 	  : 30.0fps
//Pixel order 	  : Green 1st (=GB)
//X/Y-flip	  : X-flip
//BLC offse	  : 64code
////////////////////////////////////////////////
write_cmos_sensor(0x0a00, 0x0000);  //stream off
write_cmos_sensor(0x0f38, 0x0379);
write_cmos_sensor(0x0f3a, 0x4007);
write_cmos_sensor(0x093e, 0x0000);
write_cmos_sensor(0x0920, 0xc106);
write_cmos_sensor(0x0922, 0x061a);
write_cmos_sensor(0x0924, 0x0207);
write_cmos_sensor(0x0926, 0x0b09);
write_cmos_sensor(0x0928, 0x0c08);
write_cmos_sensor(0x092a, 0x0a06);
write_cmos_sensor(0x092c, 0x1600);
write_cmos_sensor(0x0910, 0x0696);
write_cmos_sensor(0x0912, 0x00bb);
write_cmos_sensor(0x0914, 0x002d);
write_cmos_sensor(0x0916, 0x002d);
write_cmos_sensor(0x091a, 0x002a);
write_cmos_sensor(0x0938, 0x4100);
write_cmos_sensor(0x0904, 0x2bab);
write_cmos_sensor(0x0942, 0x0108);
write_cmos_sensor(0x0944, 0x0100);
write_cmos_sensor(0x0946, 0x0004);
write_cmos_sensor(0x0520, 0x0100);
write_cmos_sensor(0x0532, 0x0100);
write_cmos_sensor(0x0a2a, 0x8060);
write_cmos_sensor(0x0a2c, 0x2020);
write_cmos_sensor(0x0a32, 0x0301);
write_cmos_sensor(0x0a26, 0x0048);
write_cmos_sensor(0x0a28, 0x001a);
write_cmos_sensor(0x0a36, 0x0000);
write_cmos_sensor(0x0408, 0x0202);
write_cmos_sensor(0x0418, 0x0000);
write_cmos_sensor(0x0800, 0x0000);
write_cmos_sensor(0x0008, 0x02e8);
write_cmos_sensor(0x000c, 0x000c);
write_cmos_sensor(0x0804, 0x0010);
write_cmos_sensor(0x0026, 0x003e);
write_cmos_sensor(0x002c, 0x0c71);
write_cmos_sensor(0x005c, 0x0202);
write_cmos_sensor(0x002e, 0x1111);
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0006, 0x0d1f);
write_cmos_sensor(0x0074, 0x0d18);
write_cmos_sensor(0x0a0e, 0x0001);
write_cmos_sensor(0x0a12, 0x1070);
write_cmos_sensor(0x0a14, 0x0c30);
write_cmos_sensor(0x075c, 0x0100);
write_cmos_sensor(0x0050, 0x4300);
write_cmos_sensor(0x0722, 0x0700);
write_cmos_sensor(0x004c, 0x0100);


write_cmos_sensor(0x0a00, 0x0100);  // stream on
}

void hi1333_capture_setting_pip(void)
{
/* 4208x3120 15fps */
/* PDAF Type2 */
/* /////////////////////////////////////////////////// */
	write_cmos_sensor(0x0a00, 0x0000);	/* stream off */

	write_cmos_sensor(0x0f38, 0x077d);	/* pll_cfg_mipi1 b7-0: 4C mipi_mdiv */
	write_cmos_sensor(0x0f3a, 0x4007);	/* pll_cfg_mipi2 */
	write_cmos_sensor(0x093e, 0x0000);	/* mipi_tx_col_read_ctrl */
	write_cmos_sensor(0x0920, 0xc103);	/* mipi_exit_seq, tlpx */
	write_cmos_sensor(0x0922, 0x030d);	/* tclk_prepare, tclk_zero */
	write_cmos_sensor(0x0924, 0x0204);	/* tclk_pre, ths_prepare */
	write_cmos_sensor(0x0926, 0x0606);	/* ths_zero, ths_trail */
	write_cmos_sensor(0x0928, 0x0704);	/* tclk_post, tclk_trail */
	write_cmos_sensor(0x092a, 0x0505);	/* texit, tsync */
	write_cmos_sensor(0x092c, 0x0a00);	/* tpd_sync */
	write_cmos_sensor(0x0910, 0x036a);	/* mipi_vblank_delay */
	write_cmos_sensor(0x0918, 0x0365);	/* mipi_pd_vblank_delay */
	write_cmos_sensor(0x0912, 0x00eb);	/* mipi_hblank_delay */
	write_cmos_sensor(0x0914, 0x004e);	/* mipi_hblank_short_delay1 */
	write_cmos_sensor(0x0916, 0x004e);	/* mipi_hblank_short_delay2 */
	write_cmos_sensor(0x091a, 0x004a);	/* mipi_pd_hblank_delay */
	write_cmos_sensor(0x0938, 0x4100);	/* mipi_virtual_channel_ctrl */
	write_cmos_sensor(0x0904, 0x2bab);	/* mipi_data_id_ctrl, mipi_pd_data_id_ctrl */
	write_cmos_sensor(0x0942, 0x0108);	/* mipi_pd_sep_ctrl1, mipi_pd_sep_ctrl2 */
	write_cmos_sensor(0x0944, 0x0100);	/* mipi_pd_col_size */
	write_cmos_sensor(0x0946, 0x0004);	/* mipi_pd_row_size */
	write_cmos_sensor(0x0a2a, 0x8060);	/* PDAF patch x/y num normal */
	write_cmos_sensor(0x0a2c, 0x2020);	/* PDAF patch x/y size normal */
	write_cmos_sensor(0x0a32, 0x0301);	/* PDAF cnt012 normal */
	write_cmos_sensor(0x0a26, 0x0048);	/* PDAF x patch offset normal */
	write_cmos_sensor(0x0a28, 0x001a);	/* PDAF y patch offset normal */
	write_cmos_sensor(0x0a36, 0x0000);	/* PDAF win loc sx/sy normal */
	write_cmos_sensor(0x0408, 0x0202);	/* lsc spare */
	write_cmos_sensor(0x0418, 0x0000);	/* lsc win_h */
	write_cmos_sensor(0x0c74, 0x00ed);	/* act_win_ystart */
	write_cmos_sensor(0x0c78, 0x0adb);	/* act_win_yend */
	write_cmos_sensor(0x0a7e, 0x0005);	/* lsc shift */
	write_cmos_sensor(0x0800, 0x0000);	/* fmt ctrl */
	write_cmos_sensor(0x0008, 0x05d0);	/* line length pck */
	write_cmos_sensor(0x000c, 0x000c);	/* colgen start */
	write_cmos_sensor(0x0804, 0x0010);	/* fmt x cropping */
	write_cmos_sensor(0x0026, 0x003e);	/* y addr start active */
	write_cmos_sensor(0x002c, 0x0c71);	/* y addr end active */
	write_cmos_sensor(0x005c, 0x0202);	/* y dummy size */
	write_cmos_sensor(0x002e, 0x1111);	/* y even/odd inc tobp */
	write_cmos_sensor(0x0032, 0x1111);	/* y even/odd inc active */
	write_cmos_sensor(0x0006, 0x0d20);	/* frame length lines */
	write_cmos_sensor(0x0074, 0x0d19);	/* coarse integ time */
	write_cmos_sensor(0x0a0e, 0x0001);	/* image mode/digial binning mode */
	write_cmos_sensor(0x0a12, 0x1070);	/* x output size */
	write_cmos_sensor(0x0a14, 0x0c30);	/* y output size */
	write_cmos_sensor(0x075c, 0x0100);
	/* OTP ctrl b5:lsc_flag b4:lsc_checksum b3:pdlsc_en b2:dga_en b1:lsc_en b0:adpc_en */
	write_cmos_sensor(0x0050, 0x4300);
	/* analog control b7:vblank_analog_off b6:ag_cal b5:pd_rst2 b4:rdo_set_off b3:
	   pat_addr_en b2:sub3 b1:sreg re-load b0:sreg write */
	write_cmos_sensor(0x0722, 0x0700);	/* d2a_pxl_drv_pwr/d2a_row_binning_en */
	write_cmos_sensor(0x004c, 0x0100);	/* tg enable,hdr off */

	write_cmos_sensor(0x0a00, 0x0100);	/* stream on */
}

void hi1333_hs_video_setting_normal(void)
{

	write_cmos_sensor(0x0a00, 0x0000);
	/* setting ver5.0 None PD 640x480 120fps */
	write_cmos_sensor(0x0f38, 0x037d);	/* pll_cfg_mipi1 */
	write_cmos_sensor(0x0f3a, 0x4407);	/* pll_cfg_mipi2 b10-8:100 mipi 1/6 */
	write_cmos_sensor(0x093e, 0x0500);	/* mipi_tx_col_read_ctrl */
	write_cmos_sensor(0x0920, 0xc101);	/* mipi_exit_seq, tlpx */
	write_cmos_sensor(0x0922, 0x0105);	/* tclk_prepare, tclk_zero */
	write_cmos_sensor(0x0924, 0x0201);	/* tclk_pre, ths_prepare */
	write_cmos_sensor(0x0926, 0x0103);	/* ths_zero, ths_trail */
	write_cmos_sensor(0x0928, 0x0602);	/* tclk_post, tclk_trail */
	write_cmos_sensor(0x092a, 0x0203);	/* texit, tsync */
	write_cmos_sensor(0x092c, 0x0100);	/* tpd_sync */
	write_cmos_sensor(0x0910, 0x007b);	/* mipi_vblank_delay */
	write_cmos_sensor(0x0918, 0x0077);	/* mipi_pd_vblank_delay */
	write_cmos_sensor(0x0912, 0x001f);	/* mipi_hblank_delay */
	write_cmos_sensor(0x0938, 0x0000);	/* 0x4100); //mipi_virtual_channel_ctrl  // ADD ODIN */
	write_cmos_sensor(0x0942, 0x0008);
	/* mipi_pd_sep_ctrl1, mipi_pd_sep_ctrl2 0x0108 -> PD / 0x0008 -> nonPD */
	write_cmos_sensor(0x0a2a, 0x4030);	/* PDAF patch x/y num vga */
	write_cmos_sensor(0x0a2c, 0x4040);	/* PDAF patch x/y size vga */
	write_cmos_sensor(0x0a32, 0x030c);	/* PDAF cnt012 vga */
	write_cmos_sensor(0x0a26, 0x0048);	/* PDAF x patch offset vga */
	write_cmos_sensor(0x0a28, 0x0000);	/* PDAF y patch offset vga */
	write_cmos_sensor(0x0a36, 0x0002);	/* PDAF win loc sx/sy vga */
	write_cmos_sensor(0x0408, 0x0200);	/* lsc spare */
	write_cmos_sensor(0x0418, 0x006c);	/* lsc win_h */
	write_cmos_sensor(0x0c74, 0x0052);	/* act_win_ystart */
	write_cmos_sensor(0x0c78, 0x018f);	/* act_win_yend */
	write_cmos_sensor(0x0a7e, 0x0002);	/* lsc shift */
	write_cmos_sensor(0x0800, 0x1400);	/* fmt ctrl */
	write_cmos_sensor(0x0008, 0x02e8);	/* line length pck */
	write_cmos_sensor(0x000c, 0x000c);	/* colgen start */
	write_cmos_sensor(0x0804, 0x0022);	/* fmt x cropping */
	write_cmos_sensor(0x0026, 0x00ac);	/* y addr start active */
	write_cmos_sensor(0x002c, 0x0bff);	/* y addr end active */
	write_cmos_sensor(0x005c, 0x040c);	/* y dummy size */
	write_cmos_sensor(0x002e, 0x3311);	/* y even/odd inc tobp */
	write_cmos_sensor(0x0032, 0x5577);	/* y even/odd inc active */
	write_cmos_sensor(0x0006, 0x0348);	/* frame length lines */
	write_cmos_sensor(0x0074, 0x0341);	/* coarse integ time */
	write_cmos_sensor(0x0a0e, 0x0006);	/* image mode/digial binning mode */
	write_cmos_sensor(0x0a12, 0x0280);	/* x output size */
	write_cmos_sensor(0x0a14, 0x01e0);	/* y output size */
	write_cmos_sensor(0x075c, 0x0100);
	/* OTP ctrl b5:lsc_flag b4:lsc_checksum b3:pdlsc_en b2:dga_en b1:lsc_en b0:adpc_en */
	write_cmos_sensor(0x0050, 0x4720);
	/* analog control b7:vblank_analog_off b6:ag_cal b5:pd_rst2 b4:rdo_set_off b3:
	   pat_addr_en b2:sub3 b1:sreg re-load b0:sreg write */
	write_cmos_sensor(0x0722, 0x0702);	/* d2a_pxl_drv_pwr/d2a_row_binning_en - on */
	write_cmos_sensor(0x004c, 0x0100);	/* tg enable,hdr off */

	write_cmos_sensor(0x0a00, 0x0100);
}

void hi1333_slim_video_setting_normal(void)
{
/* //////////////////////////////// */
/* HD 1280x720 120fps */
/*  */
/* //////////////////////////////// */

	write_cmos_sensor(0x0a00, 0x0000);
	write_cmos_sensor(0x0f38, 0x037d);	/* pll_cfg_mipi1 */
	write_cmos_sensor(0x0f3a, 0x4207);	/* pll_cfg_mipi2 b10-8:010 mipi 1/3 */
	write_cmos_sensor(0x093e, 0x0200);	/* mipi_tx_col_read_ctrl */
	write_cmos_sensor(0x0920, 0xc102);	/* mipi_exit_seq, tlpx */
	write_cmos_sensor(0x0922, 0x0209);	/* tclk_prepare, tclk_zero */
	write_cmos_sensor(0x0924, 0x0203);	/* tclk_pre, ths_prepare */
	write_cmos_sensor(0x0926, 0x0304);	/* ths_zero, ths_trail */
	write_cmos_sensor(0x0928, 0x0703);	/* tclk_post, tclk_trail */
	write_cmos_sensor(0x092a, 0x0304);	/* texit, tsync */
	write_cmos_sensor(0x092c, 0x0400);	/* tpd_sync */
	write_cmos_sensor(0x0910, 0x0112);	/* mipi_vblank_delay */
	write_cmos_sensor(0x0918, 0x010d);	/* mipi_pd_vblank_delay */
	write_cmos_sensor(0x0912, 0x0057);	/* mipi_hblank_delay */
	write_cmos_sensor(0x0938, 0x0000);	/* 0x4100);  //mipi_virtual_channel_ctrl */

	write_cmos_sensor(0x0942, 0x0008);	/* 0x0108);  //mipi_pd_sep_ctrl1, mipi_pd_sep_ctrl2 */
	write_cmos_sensor(0x0a2a, 0x4030);	/* PDAF patch x/y num hd */
	write_cmos_sensor(0x0a2c, 0x4040);	/* PDAF patch x/y size hd */
	write_cmos_sensor(0x0a32, 0x1300);	/* PDAF cnt012 hd */
	write_cmos_sensor(0x0a26, 0x0048);	/* PDAF x patch offset hd */
	write_cmos_sensor(0x0a28, 0x0000);	/* PDAF y patch offset hd */
	write_cmos_sensor(0x0a36, 0x0000);	/* PDAF win loc sx/sy hd */
	write_cmos_sensor(0x0408, 0x0200);	/* lsc spare */
	write_cmos_sensor(0x0418, 0x01a2);	/* lsc win_h */
	write_cmos_sensor(0x0c74, 0x002b);	/* act_win_ystart */
	write_cmos_sensor(0x0c78, 0x017a);	/* act_win_yend */
	write_cmos_sensor(0x0a7e, 0x0003);	/* lsc shift */
	write_cmos_sensor(0x0800, 0x0800);	/* fmt ctrl */
	write_cmos_sensor(0x0008, 0x02e8);	/* line length pck */
	write_cmos_sensor(0x000c, 0x000c);	/* colgen start */
	write_cmos_sensor(0x0804, 0x0042);	/* fmt x cropping */
	write_cmos_sensor(0x0026, 0x021a);	/* y addr start active */
	write_cmos_sensor(0x002c, 0x0a93);	/* y addr end active */
	write_cmos_sensor(0x005c, 0x0206);	/* y dummy size */
	write_cmos_sensor(0x002e, 0x1111);	/* y even/odd inc tobp */
	write_cmos_sensor(0x0032, 0x3333);	/* y even/odd inc active */
	write_cmos_sensor(0x0006, 0x0348);	/* frame length lines */
	write_cmos_sensor(0x0074, 0x0341);	/* coarse integ time */
	write_cmos_sensor(0x0a0e, 0x0003);	/* image mode/digial binning mode */
	write_cmos_sensor(0x0a12, 0x0500);	/* x output size */
	write_cmos_sensor(0x0a14, 0x02d0);	/* y output size */
	write_cmos_sensor(0x075c, 0x0100);
	/* OTP ctrl b5:lsc_flag b4:lsc_checksum b3:pdlsc_en b2:dga_en b1:lsc_en b0:adpc_en */
	write_cmos_sensor(0x0050, 0x4720);
	/* analog control b7:vblank_analog_off b6:ag_cal b5:pd_rst2 b4:rdo_set_off b3:pat_addr_en b2:
	   sub3b1:sreg re-load b0:sreg write */
	write_cmos_sensor(0x0722, 0x0700);	/* d2a_pxl_drv_pwr/d2a_row_binning_en */
	write_cmos_sensor(0x004c, 0x0100);	/* tg enable,hdr off */

	write_cmos_sensor(0x0a00, 0x0100);


}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
    UINT32 temp = 0;
	LOG_DEBUG("E\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));
            temp = ((read_cmos_sensor(0x0064) << 8) | read_cmos_sensor(0x0065));
            printk("KYUNGHUN_0064 = 0x%x\n",temp);
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_DEBUG("i2c write id  : 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
// [LGE_UPDATE_S] [kyunghun.oh@lge.com] [2017-09-18] Creation of node for camera module identification.
                    if(hi1333_read_eepromVendorID() != 0)
                        printk("[HI1333] vendor id is different!! you should check the EEPROM or Module hardware\n");
                    g_mainSensor = *sensor_id;
// [LGE_UPDATE_E] [kyunghun.oh@lge.com] [2017-09-18] Creation of node for camera module identification.
				return ERROR_NONE;
			}
			LOG_DEBUG("get_imgsensor_id Read sensor id fail, id: 0x%x 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	LOG_DEBUG("E\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {

		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("SP\n");
		do {
			sensor_id = ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_DEBUG("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);

				break;
			}
			LOG_DEBUG("Read sensor id fail, write id:0x%x id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}

	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;



	/* initail sequence write in  */
	LOG_DEBUG("hi1333_find sensor pass!! Start initial setting\n");
	hi1333_init_setting_normal();
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;

	imgsensor.pdaf_mode = 1;

	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*      open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_DEBUG("E\n");

	/*No Need to implement this function */

	return ERROR_NONE;
}				/*      close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	hi1333_preview_setting_normal();

	pip_mode = 0;

	return ERROR_NONE;
}				/*      preview   */


static void capture_setting(kal_uint16 currentfps)
{
	LOG_DEBUG(" capture, currefps = %d\n", currentfps);

	LOG_INF("burst disable\n");
	if (currentfps < 300)
		hi1333_capture_setting_pip();
	else
		hi1333_capture_setting_normal();

}

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {	/* 30fps */
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;

		pip_mode = 0;
	} else {		/* PIP capture: 15fps for 13M */

		if (imgsensor.current_fps != imgsensor_info.cap1.max_framerate)
			LOG_DEBUG
			    ("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",
			     imgsensor.current_fps, imgsensor_info.cap1.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;

		pip_mode = 1;
	}

	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E");
	LOG_INF("Normal_Video\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	/* normal_video_setting(imgsensor.current_fps); */

	hi1333_capture_setting_normal();

	pip_mode = 0;

	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("hs videoE\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	hi1333_hs_video_setting_normal();

	pip_mode = 0;

	return ERROR_NONE;
}				/*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	hi1333_slim_video_setting_normal();

	pip_mode = 0;


	return ERROR_NONE;
}				/*    slim_video     */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");

	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}				/*    get_resolution    */


static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	/* sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; not use */
	    /* sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; not use */
	    /* imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; not use */
	    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
/* test */
	/* sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type; */
	/* sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode; */

	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	/* #ifdef ENABLE_PDAF */
	/* sensor_info->PDAF_Support = 1; // PDAF Type3 */
	/* #else */
	/* sensor_info->PDAF_Support = 0; //none PDAF */

/* jens test */
	sensor_info->PDAF_Support = 2;	/* PDAF Type2  //VC Type */
	/* #endif */

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("[contrlo]scenario_id = %d", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
		/* case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: */
		/* capture(image_window, sensor_config_data); */
		/* break; */
		/* 2016.09.09 Coby */
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	case MSDK_SCENARIO_ID_CAMERA_ZSD:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);

	if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = 10 * framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.pre.framelength) ? (frame_length -
							imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length =
		    imgsensor_info.normal_video.pclk / framerate * 10 /
		    imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.normal_video.framelength) ? (frame_length -
								 imgsensor_info.normal_video.
								 framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length =
			    imgsensor_info.cap1.pclk / framerate * 10 /
			    imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			    (frame_length >
			     imgsensor_info.cap1.framelength) ? (frame_length -
								 imgsensor_info.cap1.
								 framelength) : 0;
			imgsensor.frame_length =
			    imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_DEBUG
				    ("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				     framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length =
			    imgsensor_info.cap.pclk / framerate * 10 /
			    imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			    (frame_length >
			     imgsensor_info.cap.framelength) ? (frame_length -
								imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
			    imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length =
		    imgsensor_info.hs_video.pclk / framerate * 10 /
		    imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.hs_video.framelength) ? (frame_length -
							     imgsensor_info.hs_video.
							     framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		LOG_INF("HIGHT_SPPED VIDEO : Frame: %dn", imgsensor.frame_length);
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length =
		    imgsensor_info.slim_video.pclk / framerate * 10 /
		    imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.slim_video.framelength) ? (frame_length -
							       imgsensor_info.slim_video.
							       framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();

		break;
	default:		/* coding with  preview scenario by default */
		frame_length =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.pre.framelength) ? (frame_length -
							imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();

		LOG_DEBUG("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						    MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	UINT16 enable_TP = 0;

	enable_TP = ((read_cmos_sensor(0x0A04) << 8) | read_cmos_sensor(0x0A05));

	LOG_INF("enable: %d", enable);

	if (enable) {
		write_cmos_sensor(0x0938, 0x0000);
		write_cmos_sensor(0x0942, 0x0008);
		write_cmos_sensor(0x0A04, 0x0141);
		write_cmos_sensor(0x020A, 0x0200);

	} else {
		write_cmos_sensor(0x0938, 0x4100);
		write_cmos_sensor(0x0942, 0x0108);
		write_cmos_sensor(0x0A04, 0x03C1);
		write_cmos_sensor(0x020A, 0x0000);
	}
	return ERROR_NONE;
}



static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;

#if ENABLE_PDAF
	SET_PD_BLOCK_INFO_T *PDAFinfo;	/* KYM ADD 13M */
	SENSOR_VC_INFO_STRUCT *pvcinfo;
#endif

	unsigned long long *feature_data = (unsigned long long *)feature_para;

	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) * feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *feature_data,
					      *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *(feature_data),
						  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", (UINT32) *feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (BOOL) * feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL) * feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32) *feature_data);

		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;

#if ENABLE_PDAF

#if 1				/* test */
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16) *feature_data);
		pvcinfo = (SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:			
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
#endif
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("KYM_PDAF GET_PDAF_DATA EEPROM\n");

		/* read from e2prom */
#ifdef e2prom
		read_HI1333_eeprom((kal_uint16) (*feature_data),
				(char *)(uintptr_t) (*(feature_data + 1)),
				(kal_uint32) (*(feature_data + 2)));
#else
		/* read from file */

		LOG_INF("READ PDCAL DATA\n");
#if 0
		read_hi1333_eeprom_fromfile((kal_uint16) (*feature_data),
				   (char *)(uintptr_t) (*(feature_data + 1)),
				   (kal_uint32) (*(feature_data + 2)));
#endif
#endif
		break;

	case SENSOR_FEATURE_GET_PDAF_INFO:
		/* LOG_INF("KYM_PDAF SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT32)*feature_data); */
		PDAFinfo = (SET_PD_BLOCK_INFO_T *) (uintptr_t) (*(feature_data + 1));	/* coby */
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
			       sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_binning,
			       sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			break;

		}
		break;


	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		/* LOG_INF("KYM_PDAF SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", *feature_data); */
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_PDAF:
		imgsensor.pdaf_mode = *feature_data_16;
		break;

#endif
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
		ihdr_write_shutter_gain((UINT16) *feature_data, (UINT16) *(feature_data + 1),
					(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
		ihdr_write_shutter_gain((UINT16) *feature_data, (UINT16) *(feature_data + 1),
					(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:
		{
			uintptr_t *pHorFOV = (uintptr_t *)(uintptr_t)(*(feature_data));
			uintptr_t *pVerFOV = (uintptr_t *)(uintptr_t)(*(feature_data+1));
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:\n");
			*pHorFOV = (0x3F)<<16 | (0x3F); // 16:9 = 63, 4:3 = 63 // (63)<<16 | (63)   //63 degree
			*pVerFOV = (0x26)<<16 | (0x26);
			*feature_para_len = sizeof(uintptr_t) * 2;
		}
		break;
	default:
		break;
	}

	return ERROR_NONE;
}				/*  feature_control()  */


SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI1333_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*      HI1333_MIPI_RAW_SensorInit      */
