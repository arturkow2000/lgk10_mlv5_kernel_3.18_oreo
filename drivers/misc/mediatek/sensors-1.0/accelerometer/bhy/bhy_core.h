/*!
* @section LICENSE
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
*
* @filename bhy_core.h
* @date     "Wed Aug 17 17:00:04 2016 +0800"
* @id       "3b8d32d"
*
* @brief
* The header file for BHy driver core
*/

#ifndef BHY_CORE_H

#include <linux/types.h>
#include <linux/wakelock.h>
#include "bstclass.h"
#include "bhy_host_interface.h"

#ifndef MODULE_TAG
#define MODULE_TAG "BHY"
#endif

#define BMI160_BUFSIZE		256
#define BMI160_ACC_AXIS_X	0
#define BMI160_ACC_AXIS_Y	1
#define BMI160_ACC_AXIS_Z	2

/* Define this macro for nexus5 platform */
/* #define BHY_ON_NEXUS5 */

#ifdef BHY_ON_NEXUS5
/* on Nexus5 platform, in some case, when system wakeup by sensor hub interrupt,
* rtc module don't resume when read AP time in hub interrupt handle, AP time
* does not contain system suspend time in this case, so need to wait rtc resume,
* then read AP time.
* Definition is in $kernel_root/drivers/rtc/class.c
*/
extern u8 g_rtc_suspend_flag;
#endif

#define BHY_TRUE	(1)
#define BHY_FALSE	(0)

/*#define BHY_LEVEL_TRIGGERED_IRQ_SUPPORT*/

#define BHY_DEBUG
/*
 * BHY_RESERVE_FOR_LATER_USE:
 * We need this funtion for future use
 */

/*#define BHY_AR_HAL_SUPPORT*/
/*#define BHY_SW_WATCHDOG_SUPPORT*/
/*#define BHY_DSC_SENSOR_SUPPORT*/
#ifdef BHY_DSC_SENSOR_SUPPORT
#define BHY_PARAM_HEIGHT 20
#define BHY_PARAM_WEIGHT 21
#define BHY_PARAM_GENDER 22
#define BHY_PARAM_STEP_LEN 23
#define BHY_PARAM_AGE 24
#endif /*~ BHY_DSC_SENSOR_SUPPORT */

/* Support for timestamp logging for analysis */
/*#define BHY_TS_LOGGING_SUPPORT*/

#ifdef BHY_TS_LOGGING_SUPPORT
#define BHY_SENSOR_HANDLE_AP_SLEEP_STATUS	128
#define BHY_AP_STATUS_SUSPEND	1
#define BHY_AP_STATUS_RESUME	2
#endif /*~ BHY_TS_LOGGING_SUPPORT */

#ifdef BHY_DEBUG
#define BHY_SENSOR_HANDLE_DATA_LOG_TYPE	129
#define BHY_DATA_LOG_TYPE_NONE	0
#define BHY_DATA_LOG_TYPE_RAW	1
#define BHY_DATA_LOG_TYPE_INPUT_GESTURE	2
#define BHY_DATA_LOG_TYPE_INPUT_TILT_AR	3
#define BHY_SENSOR_HANDLE_LOG_FUSION_DATA	130
#define BHY_FUSION_DATA_LOG_NONE	0
#define BHY_FUSION_DATA_LOG_ENABLE	1
#endif /*~ BHY_DEBUG */

/* Supporting calib profile loading in fuser core */
#define BHY_CALIB_PROFILE_OP_IN_FUSER_CORE

#define GSE_TAG		"[bhi160]"
#define GSE_FUN(f)	printk(KERN_ERR GSE_TAG"%s\n", __func__)
#define SENSOR_NAME					"bhy"
#define SENSOR_INPUT_DEV_NAME		SENSOR_NAME
#ifdef BHY_AR_HAL_SUPPORT
#define SENSOR_AR_INPUT_DEV_NAME	"bhy_ar"
#endif /*~ BHY_AR_HAL_SUPPORT */

#define BHY_STATUS_ENABLE	(1)
#define BHY_STATUS_DISABLE	(0)

#define COMPATIABLE_NAME	"mediatek,bmi160_acc"
#define BMI160_DEV_NAME		"bmi160_acc"

#define BMI160_ACC_AXIS_X          0
#define BMI160_ACC_AXIS_Y          1
#define BMI160_ACC_AXIS_Z          2
#define BMI160_ACC_AXES_NUM        3
#define BMI160_ACC_DATA_LEN        6

#define BMI160_SHIFT_8_POSITION    8

#define BHY_FLUSH_QUEUE_SIZE 128

#ifdef BHY_ON_NEXUS5
extern uint8_t g_rtc_suspend_flag;
#endif

struct scale_factor {
	u8  whole;
	u8  fraction;
};

struct data_resolution {
	struct scale_factor scalefactor;
	int	sensitivity;
};

struct bhy_data_bus {
	struct device *dev;
	s32(*read)(struct device *dev, u8 reg, u8 *data, u16 len);
	s32(*write)(struct device *dev, u8 reg, const u8 *data, u16 len);
	int irq;
	int bus_type;
};

struct __attribute__((__packed__)) fifo_frame {
	u16 handle;
	u8 data[20];
};

#define BHY_FRAME_SIZE		20000
#define BHY_FRAME_SIZE_AR	50

struct frame_queue {
	struct fifo_frame *frames;
	int head;
	int tail;
	struct mutex lock;
};

enum {
	PHYSICAL_SENSOR_INDEX_ACC = 0,
	PHYSICAL_SENSOR_INDEX_MAG,
	PHYSICAL_SENSOR_INDEX_GYRO,
	PHYSICAL_SENSOR_COUNT
};

enum {
	SENSOR_TYPE_CONTINUOUS = 0,
	SENSOR_TYPE_ON_CHANGE,
	SENSOR_TYPE_ONE_SHOT,
	SENSOR_TYPE_SPECIAL,
	SENSOR_TYPE_INVALID
};

struct bhy_sensor_context {
	u16 handlle;
	u8  enable;
	u16 sample_rate;
	u16 report_latency;
	int type;
	int is_wakeup;
	s8 data_len;
#ifdef BHY_AR_HAL_SUPPORT
	int for_ar_hal;
#endif /*~ BHY_AR_HAL_SUPPORT */
};

struct physical_sensor_context {
	int index;
	int use_mapping_matrix;
	int mapping_matrix[9];
};

enum {
	BHY_STATUS_DEFAULT = 0,
	BHY_STATUS_ENABLED,
	BHY_STATUS_DISABLED,
};


struct bhy_meta_event_context {
	int index;
	int event_en;
	int irq_en;
};

#define BHY_SW_WATCHDOG_TIMER_INTERVAL	100000000L /* 100ms timer */
#define BHY_SW_WATCHDOG_EXPIRE	5000000000L
#define BHY_SW_WATCHDOG_EXPIRE_COUNT \
	(BHY_SW_WATCHDOG_EXPIRE / BHY_SW_WATCHDOG_TIMER_INTERVAL)

struct flush_queue {
	u8 queue[BHY_FLUSH_QUEUE_SIZE];
	int head;
	int tail;
	int cur;
	struct mutex lock;
};

struct bhy_client_data {
	struct i2c_client *client;
	struct acc_hw hw;
	struct hwmsen_convert cvt;
	atomic_t	trace;
	atomic_t	suspend;
	atomic_t	filter;
	struct mutex lock;
	struct data_resolution *reso;
	s16 cali_sw[BMI160_ACC_AXES_NUM+1];
	s8	offset[BMI160_ACC_AXES_NUM+1];

	struct device_node *irq_node;
	struct mutex mutex_bus_op;
	struct bhy_data_bus data_bus;
	struct work_struct irq_work;
	struct delayed_work fiforead_work;
	struct input_dev *input;
#ifdef BHY_AR_HAL_SUPPORT
	struct input_dev *input_ar;
#endif /*~ BHY_AR_HAL_SUPPORT */
	struct attribute_group *input_attribute_group;
#ifdef BHY_AR_HAL_SUPPORT
	struct attribute_group *input_ar_attribute_group;
#endif /*~ BHY_AR_HAL_SUPPORT */
	struct attribute_group *bst_attribute_group;
	atomic_t reset_flag;
	int sensor_sel;
	s64 timestamp_irq;
	atomic_t in_suspend;
	struct wake_lock wlock;
	u8 *fifo_buf;
	struct frame_queue data_queue;
#ifdef BHY_AR_HAL_SUPPORT
	struct frame_queue data_queue_ar;
#endif /*~ BHY_AR_HAL_SUPPORT */
	u8 bmi160_foc_conf;
	u8 bma2x2_foc_conf;
	struct bst_dev *bst_dev;
	u16 rom_id;
	u16 ram_id;
	char dev_type[16];
	s8 mapping_matrix_acc[3][3];
	s8 mapping_matrix_acc_inv[3][3];
	s8 self_test_result[PHYSICAL_SENSOR_COUNT];
	struct hrtimer sw_watchdog_timer;
	struct work_struct sw_watchdog_work;
	struct bhy_sensor_context
	sensor_context[BHY_SENSOR_HANDLE_REAL_MAX + 1];
	struct mutex mutex_sw_watchdog;
	u16 inactive_count;
	int recover_from_disaster;
	int sw_watchdog_disabled;
	int hw_watchdog_disabled;
	struct physical_sensor_context ps_context[PHYSICAL_SENSOR_COUNT];
	struct bhy_meta_event_context me_context[BHY_META_EVENT_MAX + 1];
	struct bhy_meta_event_context mew_context[BHY_META_EVENT_MAX + 1];
	struct flush_queue flush_queue;
	u16 step_count_latest; /* Save stc for disaster recovery */
	u16 step_count_base;
	u8 fifo_ctrl_cfg[BHY_FIFO_CTRL_PARAM_LEN];
	u8 calibprofile_acc[8];
	u8 calibprofile_mag[8];
	u8 calibprofile_gyro[8];
	bool need_wait_rtc;
#ifdef BHY_DEBUG
	int reg_sel;
	int reg_len;
	int page_sel;
	int param_sel;
	int enable_irq_log;
	int enable_fifo_log;
	int hw_slave_addr;
	int hw_reg_sel;
	int hw_reg_len;
#endif /*~ BHY_DEBUG */
#ifdef BHY_TS_LOGGING_SUPPORT
	u32 irq_count;
#endif /*~ BHY_TS_LOGGING_SUPPORT */
#ifdef BHY_LEVEL_TRIGGERED_IRQ_SUPPORT
	int irq_enabled;
#endif /*~ BHY_LEVEL_TRIGGERED_IRQ_SUPPORT */
#ifdef BHY_ON_NEXUS5
	u8 irq_normal_run_flag;
	u8 irq_rtc_run_flag;
#endif
	u16 enabled_counter;
	u8 cont_sensor_count;
	u8 data_check_flag;
	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend  early_drv;
#endif
};


int bhy_suspend(struct device *dev);
int bhy_resume(struct device *dev);
int bhy_probe(struct bhy_data_bus *data_bus);
int bhy_remove(struct device *dev);

#ifdef CONFIG_PM
int bhy_suspend(struct device *dev);
int bhy_resume(struct device *dev);
#endif

extern int bhy_gesture_notify(void);

#endif /** BHY_CORE_H */
