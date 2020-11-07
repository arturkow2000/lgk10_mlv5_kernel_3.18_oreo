/* touch_ssd6600.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* sentron header  */
#ifndef __SENTRON_6600_H
#define __SENTRON_6600_H

#define SUPPORT_LPM			// for support of LPM
#define SUPPORT_ESD_CHECKSUM		// For ESD checksum. only Status & Length and point info
#define SUPPORT_AUX			// for support of AUX bit of SnL
#define SUPPORT_KEY_BUTTON		// for key H/W button

#define SUPPORT_GESTURE_DEMO		// for gesture demo. relative MainTAG

#define SSD6600_UNSTALL_DELAY	5
#define SSD6600_READ_DELAY	300	// I2C packet delay

#define TOUCH_POINT_MODE	0
#define RAW_DATA_MODE		1
#define MAX_RAWDATA_BUFFER_SIZE	512

#define UPDATE_FILE_NAME		"/sdcard/sentron_fw.hex"
#define SENTRON_NAME			( "ssd6600" )			// I2C device name
#define SENTRON_I2C_ADDR		( 0x53 )			// I2C slave address

#define SENTRON_RESET_PIN		( EXYNOS5410_GPF1(1) )		// reset gpio
#define SENTRON_X_MAX			( 480 )				// resolution X
#define SENTRON_Y_MAX			( 854 )				// resolution Y
#define SENTRON_MAX_X_NODE	( 16 )					// number of NODE X
#define SENTRON_MAX_Y_NODE	( 25 )					// number of NODE Y
#define LOG_BUF_SIZE    256

#define SENTRON_MAX_POINT	( 10 )					// The maximum supported number of fingers.
#define SENTRON_MAX_NODE	( SENTRON_MAX_X_NODE * SENTRON_MAX_Y_NODE )

#define SENTRON_MAX_RAWDATA_QUEUE	10

#define SENTRON_INT_PIN		( EXYNOS5410_GPX0(2) )			// touch interrupt gpio
#define SENTRON_IRQ			( IRQ_EINT(2) )			// touch irq

#define DELAY_FOR_SIGNAL_DELAY		30	/*us*/

#define DELAY_FOR_TRANSCATION		SSD6600_READ_DELAY
#define DELAY_FOR_POST_TRANSCATION	SSD6600_READ_DELAY

#define RETRY_TIME_INT_PIN	200

#define I2C_SUCCESS		0
#define I2C_FAIL		1

//#define POWER_ON		0
//#define POWER_OFF		1
#define POWER_ON_SEQUENCE	2

// for DS16
#define DS_EFLASH_WRITE		        0x0004
#define DS_EFLASH_READ		        0x0005
#define DS_COMMAND_01		        0xF002
#define DS_COMMAND_02		        0xE008
#define DS_COMMAND_031		        0xA001
#define DS_COMMAND_03		        0xA002
#define DS_CUP_CONTROL		        0x0002
#define DS_CLEAR_INT		        0x0001
#define DS_ERASE_MACRO		        0x000B

#define DS_EFLASH_READ_01	        0x0009
#define DS_EFLASH_WRITE_01	        0x0008
#define DS_EFLASH_DOWNLOAD_ADDR1	0x7E00
#define DS_EFLASH_DOWNLOAD_ADDR2	0x0000

#define DS_WRITE_PTR		        0x9004
#define DS_READ_PTR			0x9000

//-----------------------------------------------------
// Debug msg
//-----------------------------------------------------
#define sentron_debug		0
#define sentron_warnning	1
#define sentron_timecheck	0	// only use check the boot time

extern int debug_switch;

#if sentron_debug
#define SENTRON_DEBUG(fmt, args...) \
	if ( debug_switch)  \
		printk(KERN_INFO "[SENTRON-INFO : %-18s] "fmt"\n", \
		 __func__, ##args)
#else
#define SENTRON_DEBUG(fmt, args...) \
	do {} while(0)
#endif

#if  sentron_warnning
#define SENTRON_WARNNING(fmt, args...) \
	printk(KERN_INFO "[SENTRON-WARN : %-18s] "fmt"\n", 	\
		 __func__, ##args)
#else
#define SENTRON_WARNNING(fmt, args...) \
	do {} while(0)
#endif

#if  sentron_timecheck
#define SENTRON_TIME(fmt, args...) \
	printk(KERN_INFO ""fmt"\n", 	\
		 ##args)
#else
#define SENTRON_TIME(fmt, args...) \
	do {} while(0)
#endif
//-----------------------------------------------------
// ESD TIMER
//-----------------------------------------------------
#define ESD_TIMER_ENABLE		0	// Support ESD Timer when value is 1.

#define	SENTRON_ESD_INTERVAL		1
#define SENTRON_SCAN_RATE_HZ		60
#define SENTRON_CHECK_ESD_TIMER		3


//-----------------------------------------------------
// CMD & Reg Addr
//-----------------------------------------------------
#define SENTRON_POWER_MODE		0x002F

#define SENTRON_SWRESET_CMD		0x0044

#define SENTRON_INT_CLEAR_CMD		0x0043
#define SENTRON_SW_CALIBRATION		0x0040
#define SENTRON_TEST_MODE		0x0045
#define SENTRON_HW_CALIBRATION		0x0046

#define SENTRON_TOUCH_MODE		0x0050

#define SENTRON_INT_FLAG		0x0051
#define SENTRON_DATA_VERSION		0x0052
#define SENTRON_TOTAL_NODE		0x0053
#define SENTRON_TOTAL_Y_NODE		0x0054
#define SENTRON_TOTAL_X_NODE		0x0055
#define SENTRON_X_RESOLUTION		0x0056
#define SENTRON_Y_RESOLUTION		0x0057
#define SENTRON_ORIENTATION		0x0058
#define SENTRON_USING_POINT_NUM		0x0059
#define SENTRON_SENSITIVITY		0x005c
#define SENTRON_ESD_INT_INTERVAL	0x006a

#define SENTRON_HW_CAL_INFO		0x00E4

#define SENTRON_STATUS_LENGTH		0x00f0
#define SENTRON_POINT_DATA		0x00f1
#define SENTRON_RAW_DATA		0x00f2
#define SENTRON_DEBUG_MODE		0x00f3
#define SENTRON_GRAPH_MODE		0x00f4

#define SENTRON_GET_KEYDATA		0x00F5
#define SENTRON_GET_GESTURE		0x00F6
#define SENTRON_AUX			0x00f8

#define SENTRON_DOWNLOAD_MODE		0x00f9

#define SENTRON_FW_VER_WRITE		0x00fa
#define SENTRON_DATA_VER_WRITE		0x00fb

#define SENTRON_SAVE_REG		0x00fd

// 20151115 added by howard
#define SENTRON_RUN_MODE		0x0100
#define SENTRON_TCI_ENABLE		0x0101
#define SENTRON_TCI_STATUS		0x0102
#define SENTRON_TCI1_FAIL_REASON	0x0103
#define SENTRON_TCI2_FAIL_REASON	0x0104

//-----------------------------------------------------

//-----------------------------------------------------
// RUN MODE
//-----------------------------------------------------
#define RUN_MODE_NM		        0x0000
#define RUN_MODE_LPM	                0x0001


#ifdef SUPPORT_LPM
#define CHECK_GESTURE_KNOCK_ON	        0x0400

#define STATUS_CHECK_PALM_GESTURE	0x10
#define STATUS_CHECK_KEY		0x20
#define STATUS_CHECK_AUX		0x40

#define GESTURE_STATUS_TCI2		0x0800
#define GESTURE_STATUS_TCI1		0x0400
#define GESTURE_STATUS_PALM_REJECT	0x0200
#define GESTURE_STATUS_LARGE_PALM	0x0100
#define GESTURE_STATUS_GESTURES		0x00FF

#define GESTURE_STATUS_KNOCK_ALL	(GESTURE_STATUS_TCI2|GESTURE_STATUS_TCI1|GESTURE_STATUS_GESTURES)

#define TCI_ENABLE_TCI1		        0x0001
#define TCI_ENABLE_TCI2		        0x0002

#define TCI_STATUS_TCI1		        0x0001
#define TCI_STATUS_TCI2		        0x0002

#define TCI_FAIL_REASON_COUNT	        6
#define GESTURE_READ_SIZE		40	//gesture status(2bytes) + point data(2bytes);
#endif


//-----------------------------------------------------
// DOWNLOAD
//-----------------------------------------------------
#define LDM_RUN			0x00
#define APM_RUN			0x02
#define	FW_LOAD_FINISH		0x04
#define	FW_LOAD_START		0x01
#define	FW_READ_START		0x08
#define DATA_LOAD_FINISH	0x07
#define DATA_LOAD_START		0x05
#define	DATA_READ_START		0x09

//-----------------------------------------------------


//-----------------------------------------------------
// IOCTL
//-----------------------------------------------------
#define TOUCH_IOCTL_BASE			0xbc
#define TOUCH_IOCTL_GET_FW_VERSION		_IO(TOUCH_IOCTL_BASE, 0)
#define TOUCH_IOCTL_GET_DATA_VERSION		_IO(TOUCH_IOCTL_BASE, 1)
#define TOUCH_IOCTL_GET_X_NODE_NUM		_IO(TOUCH_IOCTL_BASE, 2)
#define TOUCH_IOCTL_GET_Y_NODE_NUM		_IO(TOUCH_IOCTL_BASE, 3)
#define TOUCH_IOCTL_GET_TOTAL_NODE_NUM		_IO(TOUCH_IOCTL_BASE, 4)
#define TOUCH_IOCTL_SET_TOUCH_MODE		_IO(TOUCH_IOCTL_BASE, 5)
#define TOUCH_IOCTL_GET_RAW_DATA		_IO(TOUCH_IOCTL_BASE, 6)
#define TOUCH_IOCTL_GET_X_RESOLUTION		_IO(TOUCH_IOCTL_BASE, 7)
#define TOUCH_IOCTL_GET_Y_RESOLUTION		_IO(TOUCH_IOCTL_BASE, 8)
#define TOUCH_IOCTL_GET_REG			_IO(TOUCH_IOCTL_BASE, 9)
#define TOUCH_IOCTL_SET_REG			_IO(TOUCH_IOCTL_BASE, 10)
#define TOUCH_IOCTL_SET_DOWNLOAD		_IO(TOUCH_IOCTL_BASE, 11)
#define TOUCH_IOCTL_GET_GRAPH_DATA		_IO(TOUCH_IOCTL_BASE, 12)
#define TOUCH_IOCTL_QUEUE_CLEAR			_IO(TOUCH_IOCTL_BASE, 13)
#define TOUCH_IOCTL_GET_GESTURE			_IO(TOUCH_IOCTL_BASE, 14)
//-----------------------------------------------------

int sentron_reset(struct device *dev);
int ssd6600_reg_read(struct device *dev, u16 reg, u8 *values, u16 length);
int ssd6600_reg_read_ex(struct device *dev, u8 * reg, u16 regLen, u8 *values, u16 length);
int ssd6600_reg_write(struct device *dev, u16 addr, u8 *data, u16 size);
int ds_read_boot_st(struct device *dev, u16 *value);
int ds_clear_int(struct device *dev);
int ds_eflash_write(struct device *dev, int addr, u16 data);
int ds_eflash_read(struct device *dev, int addr, u8 *rd, int rLen);

int sentron_suspend_ex(void);
int sentron_resume_ex(void);

////////////////////////////////////////////
////// firmware update /////////////////////
////////////////////////////////////////////
#define SUPPORT_TEST_MODE

#define CONTENT_HEADER_SIZE		12	// bytes

#define FW_MAX_RETRY_COUNT		3
#define FW_MAX_I2C_DATA_COUNT	        256
#define FW_MAX_DATA_INFO_SIZE	        8
#define FW_ERASE_ALL_PAGENUM	        128

#define BOOT_UPDATE_ALL			1
#define BOOT_UPDATE_EACH		0

#define BOOT_UPDATE_OK			1
#define BOOT_UPDATE_NONE		0

#define EFLAH_ADDR_CPU			0x0000
#define EFLAH_ADDR_MP_FPM		0x6600
#define EFLAH_ADDR_MP_FDM		0x6E00
#define EFLAH_ADDR_DELTACAL		0x7100
#define EFLAH_ADDR_FPM			0x7200
#define EFLAH_ADDR_FDM			0x7A00
#define EFLAH_ADDR_TMC_REG		0x7C00
#define EFLAH_ADDR_FW_INFO		0x7D00
#define EFLAH_ADDR_CPU_CFG		0x7E00
#define EFLAH_ADDR_SYS_CFG		0x7F00
#define EFLAH_ADDR_INFO			0x7FFF
#define EFLAH_ADDR_INFO_OSC_TUNE	0x8000
#define EFLAH_ADDR_INFO_DCO32K_CAL	0x8001
#define EFLAH_ADDR_INFO_LDORDAC_CAL	0x8002

// boot status failure bit
#define BOOT_STATUS_ERR_CPUCFG_CPUINST_CHECKSUM_FAIL	0x0100
#define BOOT_STATUS_ERR_CPUCFG_CPUDM_CHECKSUM_FAIL	0x0200
#define BOOT_STATUS_ERR_CPUCFG_TABLE_CHECK_FAIL		0x0400
#define BOOT_STATUS_ERR_SYS_CFG_FAIL			0x0800
#define BOOT_STATUS_ERR_INF_LDORDAC_INVALID		0x1000
#define BOOT_STATUS_ERR_INF_IDCO32K_INVALID		0x2000
#define BOOT_STATUS_ERR_INF_OSC_TRIM_INVALID		0x4000

#define BOOT_STATUS_ERR_CPUCFG_ALL 	(BOOT_STATUS_ERR_CPUCFG_CPUINST_CHECKSUM_FAIL|BOOT_STATUS_ERR_CPUCFG_CPUDM_CHECKSUM_FAIL|BOOT_STATUS_ERR_CPUCFG_TABLE_CHECK_FAIL)
#define BOOT_STATUS_ERR_INF_ALL		(BOOT_STATUS_ERR_INF_LDORDAC_INVALID|BOOT_STATUS_ERR_INF_IDCO32K_INVALID|BOOT_STATUS_ERR_INF_OSC_TRIM_INVALID)
#define BOOT_STATUS_ERR_ALL		(BOOT_STATUS_ERR_CPUCFG_ALL|BOOT_STATUS_ERR_CPUCFG_SYS_CFG_FAIL|BOOT_STATUS_ERR_INF_ALL)


// EFLASH FLAG
#define FW_EFLASH_FLAG_CPU_ONLY				0x0001
#define FW_EFLASH_FLAG_MP_FPM				0x0002
#define FW_EFLASH_FLAG_MP_FDM				0x0004
#define FW_EFLASH_FLAG_HW_CAL				0x0008
#define FW_EFLASH_FLAG_FPM				0x0010
#define FW_EFLASH_FLAG_FDM				0x0020
#define FW_EFLASH_FLAG_TMC_REG				0x0040
#define FW_EFLASH_FLAG_SW_CAL				0x0080
#define FW_EFLASH_FLAG_CPU_CFG				0x0100
#define FW_EFLASH_FLAG_SYS_CFG				0x0200
#define FW_EFLASH_FLAG_INFO				0x0400
#define FW_EFLASH_FLAG_ALL		(FW_EFLASH_FLAG_CPU_ONLY|FW_EFLASH_FLAG_MP_FPM|FW_EFLASH_FLAG_MP_FDM|FW_EFLASH_FLAG_HW_CAL|FW_EFLASH_FLAG_FPM|FW_EFLASH_FLAG_FDM|FW_EFLASH_FLAG_TMC_REG|FW_EFLASH_FLAG_SW_CAL|FW_EFLASH_FLAG_CPU_CFG|FW_EFLASH_FLAG_SYS_CFG|FW_EFLASH_FLAG_INFO)


// ERROR NUMBER
#define ERROR_TYPE_PARSING           0x81000000
#define ERROR_TYPE_UPDATE            0x82000000
#define ERROR_TYPE_VERSION           0x84000000
#define ERROR_TYPE_VERIFY            0x88000000
#define ERROR_TYPE_EFLASH            0x90000000
#define ERROR_TYPE_SYSTEM            0xA0000000

#define ERROR_SUCCESS                0
#define ERROR_PARSING_FILENAME_IS_NULL                  (ERROR_TYPE_PARSING|0x00000001)
#define ERROR_PARSING_FILE_OPEN_FAIL                    (ERROR_TYPE_PARSING|0x00000002)
#define ERROR_PARSING_FORMAT_INVALID                    (ERROR_TYPE_PARSING|0x00000003)
#define ERROR_PARSING_CHECKSUM_FAIL                     (ERROR_TYPE_PARSING|0x00000004)
#define ERROR_PARSING_MALLOC_FAIL                       (ERROR_TYPE_PARSING|0x00000005)
#define ERROR_PARSING_CONTENT_SIZE_FAIL                 (ERROR_TYPE_PARSING|0x00000006)
#define ERROR_PARSING_DATA_CNT_FAIL                     (ERROR_TYPE_PARSING|0x00000007)
#define ERROR_PARSING_HEADER_DATA_INVALID_LENGTH        (ERROR_TYPE_PARSING|0x00000008)
#define ERROR_PARSING_INVALID_DATATYPE                  (ERROR_TYPE_PARSING|0x00000009)

#define ERROR_UPDATE_INIT_FAIL				(ERROR_TYPE_UPDATE|0x00000001)
#define ERROR_UPDATE_ERASE_FAIL				(ERROR_TYPE_UPDATE|0x00000002)
#define ERROR_UPDATE_WRITE_FAIL				(ERROR_TYPE_UPDATE|0x00000003)
#define ERROR_UPDATE_READ_FAIL				(ERROR_TYPE_UPDATE|0x00000004)
#define ERROR_UPDATE_VERIFY_FAIL			(ERROR_TYPE_UPDATE|0x00000005)

#define ERROR_EFLAH_ERASE_FAIL				(ERROR_TYPE_EFLASH|0x00000001)
#define ERROR_EFLAH_WRITE_FAIL				(ERROR_TYPE_EFLASH|0x00000002)
#define ERROR_EFLAH_READ_FAIL				(ERROR_TYPE_EFLASH|0x00000003)
#define ERROR_EFLAH_VERIFY_FAIL	        		(ERROR_TYPE_EFLASH|0x00000004)

#define ERROR_SYSTEM_FAIL				(ERROR_TYPE_SYSTEM|0x00000001)

#define ERROR_VERSION_CHECK_FAIL			(ERROR_TYPE_VERSION|0x00000001)

#define ERROR_VERIFY_VERIFY_FAIL			(ERROR_TYPE_VERIFY|0x00000001)

int SSD6600_firmware_update_byfile(struct device *dev, char *filename);
int SSD6600_firmware_pre_boot_up_check(struct device *dev, const struct firmware *fw);
int SSD6600_get_fw_info(struct device *dev);
int SSD6600_firmware_update_byArr(struct device *dev, const struct firmware *fw);
u8* SSD6600_get_version(void);
int SSD6600_get_boot_fw_info(struct device *dev);

///////////////////////////////////////////////
///// key dat /////////////////////////////////
///////////////////////////////////////////////
#ifdef SUPPORT_KEY_BUTTON
#define STR_KEYDATA_BACK_DOWN		0x0001
#define STR_KEYDATA_MENU_DOWN		0x0002
#define STR_KEYDATA_HOME_DOWN		0x0004
#define STR_KEYDATA_HOMEPAGE_DOWN	0x0008

#define STR_KEYDATA_BACK_UP		0x0100
#define STR_KEYDATA_MENU_UP		0x0200
#define STR_KEYDATA_HOME_UP		0x0400
#define STR_KEYDATA_HOMEPAGE_UP		0x0800
#endif	// SUPPORT_KEY_BUTTON

// LPWG
#define POWER_STATUS_NM                 0x8000
#define POWER_STATUS_LPM                0x4000
#define LPM_RESUME                      (POWER_STATUS_NM | 0x0001)
#define LPM_SUSPEND                     (POWER_STATUS_LPM | 0x0001)
#define LPM_SUSPEND_DELAY               (POWER_STATUS_LPM | 0x0002)

#define SENTRON_LPWG_STATUS             0x00CA

// LPWG MODE
#define SENTRON_LPWG_TCI1_ENABLE                0x0001
#define SENTRON_LPWG_TCI2_ENABLE                0x0002
#define SENTRON_LPWG_STATUS_OFF                 0x0000
#define SENTRON_LPWG_STATUS_KNOCK_ON_ONLY       (SENTRON_LPWG_TCI2_ENABLE)
#define SENTRON_LPWG_STATUS_KNOCK_ON_AND_CODE   (SENTRON_LPWG_TCI2_ENABLE|SENTRON_LPWG_TCI1_ENABLE)


//write log
#define BUF_SIZE (PAGE_SIZE * 2)

struct sentron_info {
        u8 version1[4];
        u8 version2[4];
        char prdID1[5];
        char prdID2[5];
        char ICName1[5];
        char ICName2[5];
        u8 haveflag;
};

// sentron
struct sentron_point {
        u16 id_x;
        u16 w_y;
};

struct sentron_data {
        u16 point_info;
        struct sentron_point points[SENTRON_MAX_POINT];
        s16 rawData[SENTRON_MAX_RAWDATA_QUEUE][SENTRON_MAX_NODE + 4*SENTRON_MAX_POINT + 2];
        int queue_front;
        int queue_rear;
        int validPointCnt;
        int lastValidCnt;
#ifdef SUPPORT_KEY_BUTTON
        u32 keydata;
#endif
};

struct sentron_config {
        u8 *fw_ver;
        u16 data_ver;
        u16 max_x;      // x resolution
        u16 max_y;      // y resolution
        u16 max_weight;
        u16 using_point;
        u16 x_node;
        u16 y_node;
        bool check;
};

enum _ts_work_procedure {
        TS_NO_WORK = 0,
        TS_NORMAL_WORK,
        TS_ESD_TIMER_WORK,
        TS_IN_EALRY_SUSPEND,
        TS_IN_SUSPEND,
        TS_IN_RESUME,
        TS_IN_LATE_RESUME,
        TS_IN_UPGRADE,
        TS_REMOVE_WORK,
        TS_SET_MODE,
        TS_GET_RAW_DATA,
};

struct sentron_device {
        struct input_dev *input_dev;
        struct i2c_client *client;
        struct atc260x_dev *atc260x;
        struct sentron_data *ftdata;
        struct sentron_config *ftconfig;
        struct sentron_info *fw_info;
        struct sentron_info *bin_info;
        struct work_struct work;
        struct workqueue_struct *workqueue;
        struct mutex lock;
#ifdef CONFIG_HAS_EARLYSUSPEND  
        struct early_suspend es;
#endif
        int irq;
        int int_pin;
        u16 touch_mode;
        u8 update;
        struct semaphore    raw_data_lock;
        u8 work_procedure;
        struct semaphore work_procedure_lock;

    //ESD TIMER
#if ESD_TIMER_ENABLE
        struct work_struct tmr_work;
        struct workqueue_struct *tmr_workqueue;
        u8 use_esd_tmr;
        bool in_esd_tmr;
        struct timer_list esd_tmr;
        struct timer_list *p_esd_tmr;
        unsigned long esd_check_time;
#endif
};

static inline struct sentron_device *to_sentron_data(struct device *dev)
{
        return (struct sentron_device *)touch_get_device(to_touch_core(dev));
}

#endif /* LGE_TOUCH_SENTRON_H */

