/*
* lge_tuning.h
*
* Copyright (c) 2017 LGE.
*
* author : woonghwan.lee@lge.com and junil.cho@lge.com
*
* This software is licensed under the terms of the GNU general Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#ifndef __LGE_SETTING_H__
#define __LGE_SETTING_H__

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <linux/ctype.h>

#include "disp_utils.h"
#include "mtkfb.h"
#include "ddp_hal.h"
#include "ddp_dump.h"
#include "ddp_path.h"
#include "ddp_drv.h"
#include "ddp_reg.h"
#include "disp_session.h"
#include "primary_display.h"
#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"
#include "ddp_manager.h"
#include "mtkfb_fence.h"
#include "mtkfb_debug.h"
#include "disp_recorder.h"
#include "fbconfig_kdebug.h"
#include "ddp_mmp.h"
#include "mtk_sync.h"
#include "ddp_irq.h"
#include "disp_session.h"
#include "disp_helper.h"
#include "ddp_reg.h"
#include "mtk_disp_mgr.h"
#include "ddp_dsi.h"
#include "mtkfb_console.h"
#include "disp_recovery.h"

#define TOTAL_MIPI_SETTING_TYPE 20
#define FB_SYSFS_FLAG_ATTR 1

#define DSI_OUTREG32(cmdq, addr, val) DISP_REG_SET(cmdq, addr, val)
#define DSI_BACKUPREG32(cmdq, hSlot, idx, addr) DISP_REG_BACKUP(cmdq, hSlot, idx, addr)
#define DSI_POLLREG32(cmdq, addr, mask, value) DISP_REG_CMDQ_POLLING(cmdq, addr, value, mask)
#define DSI_INREG32(type, addr) INREG32(addr)
#define DSI_READREG32(type, dst, src) mt_reg_sync_writel(INREG32(src), dst)
//#define OUTREG32(x, y)  WRITE_REGISTER_UINT32((uint32_t *)((void *)(x)),(uint32_t)(y))
#define AS_UINT32(x)    (*(uint32_t *)((void *)x))
#define DSI_MASKREG32(cmdq, REG, MASK, VALUE)	DISP_REG_MASK((cmdq), (REG), (VALUE), (MASK))
#define DSI_OUTREGBIT(cmdq, TYPE, REG, bit, value)  \
	{\
		do {\
			TYPE r;\
			TYPE v;\
			if (cmdq) {\
				*(unsigned int *)(&r) = ((unsigned int)0x00000000); \
				r.bit = ~(r.bit);  \
				*(unsigned int *)(&v) = ((unsigned int)0x00000000); \
				v.bit = value; \
				DISP_REG_MASK(cmdq, &REG, AS_UINT32(&v), AS_UINT32(&r)); \
			} else { \
				mt_reg_sync_writel(INREG32(&REG), &r); \
				r.bit = (value); \
				DISP_REG_SET(cmdq, &REG, INREG32(&r)); \
			} \
		} while (0);\
	}

//#define REGFLAG_ESCAPE_ID  0x00
//variables
#if defined(CONFIG_LGE_DSI_PARAM_TUNING)
typedef struct total_mipi_setting_value{
	uint32_t hs_prpr;
	uint32_t hs_zero;
	uint32_t hs_trail;
	uint32_t ta_go;
	uint32_t ta_sure;
	uint32_t ta_get;
	uint32_t da_hs_exit;
	uint32_t clk_zero;
	uint32_t clk_trail;
	uint32_t cont_det;
	uint32_t clk_hs_prpr;
	uint32_t clk_hs_post;
	uint32_t clk_hs_exit;
	uint32_t hfp;
	uint32_t hsa;
	uint32_t hbp;
	uint32_t vsa;
	uint32_t vfp;
	uint32_t vbp;
	uint32_t lpx;
	uint32_t ssc_en;
}total_mipi_setting_value;

typedef enum{
	LGE_PLLCLK =0,
	LGE_SSC,
	LGE_VFP,
	LGE_VBP,
	LGE_VSA,
	LGE_HFP,
	LGE_HBP,
	LGE_HSA,
	LGE_ESD_ON_OFF,
}LGE_DISP_CONFIG;

typedef struct {
	DISP_POWER_STATE state;
	unsigned int lcm_fps;
	int max_layer;
	int need_trigger_overlay;
	int need_trigger_ovl1to2;
	int need_trigger_dcMirror_out;
	DISP_PRIMARY_PATH_MODE mode;
	unsigned int session_id;
	int session_mode;
	int ovl1to2_mode;
	unsigned int last_vsync_tick;
	unsigned long framebuffer_mva;
	unsigned long framebuffer_va;
	struct mutex lock;
	struct mutex capture_lock;
	struct mutex switch_dst_lock;
	disp_lcm_handle *plcm;
	cmdqRecHandle cmdq_handle_config_esd;
	cmdqRecHandle cmdq_handle_config;
	disp_path_handle dpmgr_handle;
	disp_path_handle ovl2mem_path_handle;
	cmdqRecHandle cmdq_handle_ovl1to2_config;
	cmdqRecHandle cmdq_handle_trigger;
	char *mutex_locker;
	int vsync_drop;
	unsigned int dc_buf_id;
	unsigned int dc_buf[DISP_INTERNAL_BUFFER_COUNT];
	unsigned int freeze_buf;
	unsigned int force_fps_keep_count;
	unsigned int force_fps_skip_count;
	cmdqBackupSlotHandle cur_config_fence;
	cmdqBackupSlotHandle subtractor_when_free;
	cmdqBackupSlotHandle rdma_buff_info;
	cmdqBackupSlotHandle ovl_status_info;
	cmdqBackupSlotHandle ovl_config_time;

	int is_primary_sec;
} display_primary_path_context;

static total_mipi_setting_value dsi_total_val;
static display_primary_path_context *lge_pgc;
static int esd_on_flag = 1;
#endif

#if defined(CONFIG_LGE_INIT_CMD_TUNING)
static int reg_num_idx = -1;
static unsigned char init_cmd;
#endif

//extern function and variables
#if defined(CONFIG_LGE_DSI_PARAM_TUNING)
extern uint32_t PanelMaster_get_dsi_timing(uint32_t dsi_index, MIPI_SETTING_TYPE type);
extern void PanelMaster_DSI_set_timing(uint32_t dsi_index, MIPI_TIMING timing);
extern void *primary_get_pgc(void);
#endif

#if defined(CONFIG_LGE_INIT_CMD_TUNING)
extern LCM_setting_table_V3* get_lcm_init_cmd_structure(void);
extern int get_init_cmd_str_size(void);
#endif

#if defined(CONFIG_LGE_BACKLIGHT_BRIGHTNESS_TUNING)
extern unsigned int get_backlight_brightness_rawdata(void);
extern void set_backlight_brightness_rawdata(unsigned int level);
#if defined(CONFIG_LGE_LCM_BACKLIGHT_PWM)
extern unsigned int bright_arr_pwm[];
#else
extern unsigned int bright_arr[];
#endif
#endif

#if defined(CONFIG_BACKLIGHT_DRIVER_REG_TUNING)
extern unsigned char get_bl_ic_reg_val(void);
extern void set_bl_ic_reg_addr(unsigned char reg_addr);
extern void set_bl_ic_reg_val(unsigned char reg_val);
#endif


//function
#if defined(CONFIG_LGE_DSI_PARAM_TUNING)
static void lge_set_dsi_value(uint32_t dsi_index, LGE_DISP_CONFIG mode,int value);
static uint32_t lge_get_dsi_value(uint32_t dsi_index, LGE_DISP_CONFIG mode);
static void _lge_path_lock(const char *caller);
static void _lge_path_unlock(const char *caller);
static int lge_mtk_display_config_dsi(LGE_DISP_CONFIG mode, uint32_t config_value);
int lge_mtk_dsi_config_entry(LGE_DISP_CONFIG mode, void *config_value);
int lge_mtk_set_disp_config(unsigned int new_value, LGE_DISP_CONFIG mode);
LCM_PARAMS* lge_display_get_lcm_params(void);
void init_mipi_total_setting_value(int pm_dsi_mode);
#endif

//sysfs
#if defined(CONFIG_LGE_DSI_PARAM_TUNING)
static ssize_t lge_disp_config_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t lge_disp_config_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t lge_disp_config_add_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t lge_disp_config_add_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
#endif

#if defined(CONFIG_LGE_INIT_CMD_TUNING)
static ssize_t mtkfb_lge_init_cmd_find_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t mtkfb_lge_init_cmd_find_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t mtkfb_lge_init_cmd_edit_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t show_lcd_init_cmd_full(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_lcd_init_cmd_full(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
#endif

#if defined(CONFIG_LGE_BACKLIGHT_BRIGHTNESS_TUNING)
static ssize_t store_driver_brightness(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t size);
static ssize_t show_driver_brightness(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t lcd_backlight_show_blmap(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t lcd_backlight_store_blmap(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
#endif

#if defined(CONFIG_BACKLIGHT_DRIVER_REG_TUNING)
static ssize_t store_bl_ic_reg_addr(struct kobject *kobj,struct kobj_attribute *attr, const char *buf,size_t size);
static ssize_t show_bl_ic_reg_val(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t store_bl_ic_reg_val(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,size_t size);
#endif

#endif
