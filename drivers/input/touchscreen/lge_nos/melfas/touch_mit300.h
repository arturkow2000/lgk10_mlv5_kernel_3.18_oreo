/* lge_touch_melfas.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: sangyeol.ryu@lge.com
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
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/firmware.h>

#ifndef LGE_TOUCH_MIT300_H
#define LGE_TOUCH_MIT300_H

#ifndef MIP_USE_DEV
#define MIP_USE_DEV 0
#endif

#define FW_BLOCK_SIZE		128
#define FW_MAX_SIZE		(64 * 1024)

#define FINGER_EVENT_SZ		6
#define LPWG_EVENT_SZ		3
#define MAX_PRESSURE		255

#define SECTION_NUM		3
#define PAGE_HEADER		3
#define PAGE_DATA		1024
#define PAGE_CRC		2
#define PACKET_SIZE		(PAGE_HEADER + PAGE_DATA + PAGE_CRC)

#define KNOCKON_DELAY   	700

/*Number of channel*/
#define MAX_COL			18
#define MAX_ROW			32

#define NAME_BUFFER_SIZE 	128

#define MIP_R0_INFO				0x01
#define MIP_R1_VENDOR_INFO			0x00
#define MIP_R1_INFO_VERSION_CUSTOM		0x24
#define MIP_R1_INFO_PRODUCT_NAME		0x00
#define MIP_R1_INFO_IC_NAME			0x71
#define MIP_R1_INFO_RESOLUTION_X		0x10
#define MIP_R1_INFO_RESOLUTION_Y		0x12
#define MIP_R1_INFO_NODE_NUM_X			0x14
#define MIP_R1_INFO_NODE_NUM_Y			0x15
#define MIP_R1_INFO_KEY_NUM			0x16
#define MIP_R1_INFO_VERSION_BOOT		0x20
#define MIP_R1_INFO_VERSION_CORE		0x22
#define MIP_R1_INFO_VERSION_CUSTOM		0x24
#define MIP_R1_INFO_VERSION_PARAM		0x26
#define MIP_R1_INFO_SECT_BOOT_START		0x30
#define MIP_R1_INFO_SECT_BOOT_END		0x31
#define MIP_R1_INFO_SECT_CORE_START		0x32
#define MIP_R1_INFO_SECT_CORE_END		0x33
#define MIP_R1_INFO_SECT_CUSTOM_START		0x34
#define MIP_R1_INFO_SECT_CUSTOM_END		0x35
#define MIP_R1_INFO_SECT_PARAM_START		0x36
#define MIP_R1_INFO_SECT_PARAM_END		0x37
#define MIP_R1_INFO_BUILD_DATE			0x40
#define MIP_R1_INFO_BUILD_TIME			0x44
#define MIP_R1_INFO_CHECKSUM_PRECALC		0x48
#define MIP_R1_INFO_CHECKSUM_REALTIME		0x4A
#define MIP_R1_INFO_CHECKSUM_CALC		0x4C
#define MIP_R1_INFO_PROTOCOL_NAME		0x50
#define MIP_R1_INFO_PROTOCOL_VERSION		0x58
#define MIP_R1_INFO_IC_ID			0x70
#define MIP_R1_INFO_IC_NAME			0x71

#define MIP_R0_EVENT				0x02
#define MIP_R1_EVENT_SUPPORTED_FUNC		0x00
#define MIP_R1_EVENT_FORMAT			0x04
#define MIP_R1_EVENT_SIZE			0x06
#define MIP_R1_EVENT_PACKET_INFO		0x10
#define MIP_R1_EVENT_PACKET_DATA		0x11

#define MIP_R0_CTRL				0x06
#define MIP_R1_CTRL_READY_STATUS		0x00
#define MIP_R1_CTRL_EVENT_READY			0x01
#define MIP_R1_CTRL_MODE			0x10
#define MIP_R1_CTRL_EVENT_TRIGGER_TYPE		0x11
#define MIP_R1_CTRL_RECALIBRATE			0x12
#define MIP_R1_CTRL_POWER_STATE			0x13
#define MIP_R1_CTRL_GESTURE_TYPE		0x14
#define MIP_R1_CTRL_DISABLE_ESD_ALERT		0x18
#define MIP_R1_CTRL_CHARGER_MODE		0x19
#define MIP_R1_CTRL_GLOVE_MODE			0x1A
#define MIP_R1_CTRL_WINDOW_MODE			0x1B
#define MIP_R1_CTRL_PALM_REJECTION		0x22
#define MIP_R1_CTRL_EDGE_EXPAND			0x1D
#define MIP_R1_CTRL_LPWG_DEBUG_ENABLE		0x1F

#define MIP_R0_PARAM				0x08
#define MIP_R1_PARAM_BUFFER_ADDR		0x00
#define MIP_R1_PARAM_PROTOCOL			0x04
#define MIP_R1_PARAM_MODE			0x10

#define MIP_R0_LOG				0x10
#define MIP_R1_LOG_TRIGGER			0x14

/* Value */
#define MIP_EVENT_INPUT_PRESS			0x80
#define MIP_EVENT_INPUT_SCREEN			0x40
#define MIP_EVENT_INPUT_HOVER			0x20
#define MIP_EVENT_INPUT_PALM			0x10
#define MIP_EVENT_INPUT_ID			0x0F

#define MIP_EVENT_GESTURE_DOUBLE_TAP		24
#define MIP_EVENT_GESTURE_MULTI_TAP		25
#define MIP_EVENT_GESTURE_SWIPE			26
#define MIP_EVENT_GESTURE_ALL			0xFFFFFFFF

#define MIP_ALERT_ESD				1
#define MIP_ALERT_WAKEUP			2
#define MIP_ALERT_F1				0xF1

#define MIP_CTRL_POWER_ACTIVE			0
#define MIP_CTRL_POWER_LOW			1

/* LPWG Register map */
#define MIP_R0_LPWG				0x0E
/* Control */
#define MIP_R1_LPWG_START 			0x10
#define MIP_R1_LPWG_ENABLE_SENSING		0x11
#define MIP_R1_SET_WAKEUP_BY_SWIPE		0x12
/* Common */
#define MIP_R1_LPWG_LCD_STATUS			0x20
#define MIP_R1_LPWG_IDLE_REPORTRATE		0x21
#define MIP_R1_LPWG_ACTIVE_REPORTRATE 		0x22
#define MIP_R1_LPWG_SENSITIVITY			0x23
#define MIP_R1_LPWG_ACTIVE_AREA 		0x24
#define MIP_R1_LPWG_FAIL_REASON 		0x2C
/* Knock On */
#define MIP_R1_LPWG_ENABLE 			0x40
#define MIP_R1_LPWG_WAKEUP_TAP_COUNT		0x41
#define MIP_R1_LPWG_TOUCH_SLOP	 		0x42
#define MIP_R1_LPWG_MIN_INTERTAP_DISTANCE	0x44
#define MIP_R1_LPWG_MAX_INTERTAP_DISTANCE	0x46
#define MIP_R1_LPWG_MIN_INTERTAP_TIME		0x48
#define MIP_R1_LPWG_MAX_INTERTAP_TIME		0x4A
#define MIP_R1_LPWG_INT_DELAY_TIME		0x4C
/* Knock Code */
#define MIP_R1_LPWG_ENABLE2			0x50
#define MIP_R1_LPWG_WAKEUP_TAP_COUNT2		0x51
#define MIP_R1_LPWG_TOUCH_SLOP2	 		0x52
#define MIP_R1_LPWG_MIN_INTERTAP_DISTANCE2	0x54
#define MIP_R1_LPWG_MAX_INTERTAP_DISTANCE2	0x56
#define MIP_R1_LPWG_MIN_INTERTAP_TIME2		0x58
#define MIP_R1_LPWG_MAX_INTERTAP_TIME2		0x5A
#define MIP_R1_LPWG_INT_DELAY_TIME2		0x5C
/* Swipe */
#define MIP_R1_SWIPE_ENABLE                 	0x60
#define MIP_R1_SWIPE_DISTANCE_THRESHOLD     	0x61
#define MIP_R1_SWIPE_RATIO_THRESHOLD        	0x62
#define MIP_R1_SWIPE_RATIO_CHECK_PERIOD     	0x63
#define MIP_R1_SWIPE_MIN_TIME_THRESHOLD     	0x64
#define MIP_R1_SWIPE_MAX_TIME_THRESHOLD     	0x66
#define MIP_R1_SWIPE_INTERRUPT_STATUS       	0x68
#define MIP_R1_SWIPE_TOUCH_TIME             	0x69

/* Event types */
#define MIT_LOG_EVENT				0xD
#define MIT_LPWG_EVENT				0xE
#define MIT_ERROR_EVENT				0xF
#define MIT_TOUCH_KEY_EVENT			0x40
#define MIT_REQUEST_THERMAL_INFO		0xB
#define MIT_ERRORCODE_FAIL_REASON		0x14

/* LPWG Control Value */
#define IDLE_REPORTRATE_CTRL    		1
#define ACTIVE_REPORTRATE_CTRL  		2
#define SENSITIVITY_CTRL        		3

#define TCI_ENABLE_CTRL         		11
#define TOUCH_SLOP_CTRL         		12
#define TAP_MIN_DISTANCE_CTRL   		13
#define TAP_MAX_DISTANCE_CTRL   		14
#define MIN_INTERTAP_CTRL       		15
#define MAX_INTERTAP_CTRL       		16
#define TAP_COUNT_CTRL          		17
#define INTERRUPT_DELAY_CTRL    		18

#define TCI_ENABLE_CTRL2        		21
#define TOUCH_SLOP_CTRL2        		22
#define TAP_MIN_DISTANCE_CTRL2  		23
#define TAP_MAX_DISTANCE_CTRL2  		24
#define MIN_INTERTAP_CTRL2      		25
#define MAX_INTERTAP_CTRL2      		26
#define TAP_COUNT_CTRL2         		27
#define INTERRUPT_DELAY_CTRL2   		28

#define LPWG_STORE_INFO_CTRL    		31
#define LPWG_START_CTRL         		32
#define LPWG_PANEL_DEBUG_CTRL   		33
#define LPWG_FAIL_REASON_CTRL   		34

#define MIP_LPWG_EVENT_TYPE_FAIL		1

#define I2C_RETRY_COUNT				3

enum {
	FAIL_OUT_OF_AREA = 1,
	FAIL_PALM,
	FAIL_DELAY_TIME,
	FAIL_TAP_TIME,
	FAIL_TAP_DISTACE,
	FAIL_TOUCH_SLOPE,
	FAIL_MULTI_TOUCH,
	FAIL_LONG_PRESS,
	FAIL_SWIPE_FINGER_RELEASE,
	FAIL_SWIPE_MULTIPLE_FINGERS,
	FAIL_SWIPE_TOO_FAST,
	FAIL_SWIPE_TOO_SLOW,
	FAIL_SWIPE_UPWARD,
	FAIL_SWIPE_RATIO_EXECESS
};

enum {
        LPWG_DISABLE_SENSING = 0,
        LPWG_ENABLE_SENSING = 1,
};

enum {
	LOG_TYPE_U08	= 2,
	LOG_TYPE_S08,
	LOG_TYPE_U16,
	LOG_TYPE_S16,
	LOG_TYPE_U32	= 8,
	LOG_TYPE_S32,
};

enum {
	CUT5 = 5,
	CUT6,
	CUT7,
};
struct mit_dev {
	u16 x_resolution;
	u16 y_resolution;
	u16 active_area_gap;
	u8 contact_on_event_thres;
	u8 moving_event_thres;
	u8 active_report_rate;
	u8 operation_mode;
	u8 tx_ch_num;
	u8 rx_ch_num;
	u8 row_num;
	u8 col_num;
	u8 key_num;
	u8 lcd_status;
};

struct touch_limit_value {
	u32	raw_data_max;
	u32	raw_data_min;
	u32	raw_data_margin;
	u32	open_short_max;
	u32	open_short_min;
	u32	cm_delta;
	u32	cm_jitter;
	u32 	mux_short_max;
	u32	mux_short_min;
	u32 	lpwg_jitter_max;
	u32	lpwg_jitter_min;
	u32	lpwg_abs_max;
	u32 	lpwg_abs_min;
};

struct mit_section {
	u8 version;
	u8 compatible_version;
	u8 start_addr;
	u8 end_addr;
	int offset;
	u32 crc;
};

struct mit_module {
	u8 product_code[16];
	u8 version[2];
	u8 ic_name[5];
	u8 bin_version[2];
	char bin_chip_name[5];
};

struct mit_log {
	u8 *data;
	int cmd;
};

struct mit_bin_hdr {
	char	tag[8];
	u16	core_version;
	u16	section_num;
	u16	contains_full_binary;
	u16	reserved0;

	u32	binary_offset;
	u32	binary_length;

	u32	extention_offset;
	u32	reserved1;
} __attribute__ ((packed));

struct mit_fw_img {
	u16	type;
	u16	version;

	u16	start_page;
	u16	end_page;

	u32	offset;
	u32	length;

} __attribute__ ((packed));

/**
* Firmware update error code
*/
enum fw_update_errno{
	fw_err_file_read = -4,
	fw_err_file_open = -3,
	fw_err_file_type = -2,
	fw_err_download = -1,
	fw_err_none = 0,
	fw_err_uptodate = 1,
};

#define TOUCH_PWR_NUM	1

struct mit_data {
	struct device *d_dev;
	bool probed;
	struct i2c_client *client;
	struct touch_platform_data *pdata;
	struct regulator *vdd_regulator[TOUCH_PWR_NUM];
	struct lpwg_tci_data *lpwg_data;
	struct mit_dev dev;
	bool need_update[SECTION_NUM];
	struct mit_section ts_section[SECTION_NUM];
	struct mit_bin_hdr *fw_hdr;
	struct mit_fw_img* fw_img[SECTION_NUM];
	struct mit_module module;
	char buf[PACKET_SIZE];
	struct mit_log log;
	uint16_t *mit_data[MAX_ROW];
	s16 *intensity_data[MAX_ROW];
	u8 test_mode;
	int count_short;
	int thermal_info_send_block;
	int r_max;
	int r_min;
	int o_max;
	int o_min;
	int d_max;
	int d_min;
	int j_max;
	int j_min;
	int m_max;
	int m_min;
	int l_j_max;
	int l_j_min;
	int l_a_max;
	int l_a_min;

	/* MIP_USE_DEV(for melfas debugging) */
	struct class *class;
	dev_t mip_dev;
	struct cdev cdev;
	u8 *dev_fs_buf;

	bool	selfdiagnostic_state[5];
	bool	lpwg_selfdiagnostic_state[2];
	struct touch_limit_value	*limit;
	int check_openshort;
	u8 lpwg_debug_enable;
	u8 lpwg_fail_reason;
	u8 lpwg_lcd_status;
	u8 use_quick_window;
	u8 use_excel;
	u8 event_size;
	int event_format;
	int ispalm;
};

struct mit_log_pkt {
	u8	marker;
	u8	log_info;
	u8	code;
	u8	element_sz;
	u8	row_sz;
} __attribute__ ((packed));

enum {
	NO_SUPPORT_SWIPE = 0,
	SUPPORT_SWIPE,
};

enum {
	SWIPE_DISABLE = 0,
	SWIPE_ENABLE,
};

static inline struct mit_data *to_mit_data(struct device *dev)
{
	return (struct mit_data *)touch_get_device(to_touch_core(dev));
}

int mit300_reg_read(struct device *dev, u8* addr, int tx_size, void *data, int rx_size);
int mit300_reg_write(struct device *dev, u8 *addr, int size);
int tci_control(struct device *dev, int type, u16 value);
void reset_pin_ctrl(int on_off, int delay);
int mit300_ic_info(struct device *dev);
int mip_get_fw_version(struct device *dev, u8 *ver_buf);
int mit_sysfs(struct device *dev, char *buf1, const char *buf2, u32 code);
int mit_power_reset(struct device *dev);
int mit300_fw_compare(struct device *dev, const struct firmware *fw);
int mit_isc_fwupdate(struct device *dev, const struct firmware *fw);
int mip_isc_read_page(struct device *dev, int offset, u8 *data);
int mip_lpwg_enable_sensing(struct device *dev, bool bEnable);
int mip_isc_exit(struct device *dev);
int mip_lpwg_start(struct device *dev);
int mit300_debugging(struct device *dev);
extern int lge_get_db7400_cut_ver(void);
extern int hallic_is_pouch_closed(void);
#endif // LGE_TOUCH_MIT300_H

