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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include <soc/mediatek/lge/board_lge.h>//LGE_UPDATE [kyunghun.oh@lge.com] 2016/10/06, add model define and revision checking for flash
#include "kd_camera_hw.h" //LGE_UPDATE [yonghwan.lym@lge.com] 2016/09/08, combine the camera GPIO function
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
//	#define PK_DBG PK_DBG_FUNC
    #define PK_DBG printk
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

   //[LGE_UPDATE_S] [kipo.yun@lge.com] [2014-05-10] Enable Flash Driver
   static u32 strobe_width = 0; /* 0 is disable */

   static const MUINT32 strobeLevelLUT[32] = {1,1,1,1,3,3,3,3,3,3,3,2,4,4,4,4,5,5,5,2,5,5,5,5,5,5,5,5,5,5,5,5};
  //[LGE_UPDATE_E] [kipo.yun@lge.com] [2014-05-10] Enable Flash Driver

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0x11


static struct work_struct workTimeOut;


/*****************************************************************************
Device Tree GPIO Control Function
*****************************************************************************/
/* GPIO Pin control*/
extern int mtkcam_gpio_set(int PinIdx, int PwrType, int Val); //LGE_UPDATE [yonghwan.lym@lge.com] 2016/09/08, combine the camera GPIO function

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void dummy_flash_strobe_en(void)
{
    return;
}
static void dummy_flash_strobe_prepare(char OnOff,char ActiveHigh)
{
    return;
}
static void dummy_flash_strobe_level(char level)
{
    return;
}

#if defined(CONFIG_LEDS_LM3632)
extern void lm3632_flash_strobe_en(void);
extern void lm3632_flash_strobe_prepare(char OnOff,char ActiveHigh);
extern void lm3632_flash_strobe_level(char level);
#endif

#if defined(CONFIG_LEDS_LM3648)
extern void lm3648_flash_strobe_en(void);
extern void lm3648_flash_strobe_prepare(char OnOff,char ActiveHigh);
extern void lm3648_flash_strobe_level(char level);
#if defined(CONFIG_MACH_MT6750_LV7)
extern void lm3648_flash_torch_level(int level);
#endif
#endif

#if defined(CONFIG_LEDS_RT4832)
extern void rt4832_flash_strobe_en(void);
extern void rt4832_flash_strobe_prepare(char OnOff,char ActiveHigh);
extern void rt4832_flash_strobe_level(char level);
#endif
static void (* flash_strobe_en)(void) = NULL;
static void (* flash_strobe_prepare)(char OnOff,char ActiveHigh) = NULL;
static void (* flash_strobe_level)(char level) = NULL;
#if defined(CONFIG_MACH_MT6750_LV7)
static void (* flash_torch_level)(int level) = NULL;
#endif

static void work_timeOutFunc(struct work_struct *data);

static int FL_Enable(void)
{
    mtkcam_gpio_set(0,CAM_FLASH_FRONT_EN,1); //LGE_UPDATE [yonghwan.lym@lge.com] 2016/09/08, combine the camera GPIO function
    flash_strobe_prepare(2,1); // Torch only
    flash_strobe_level(1);

    flash_strobe_en();
    return 0;
}

#if defined(CONFIG_MACH_MT6750_LV7)
static int FL_Set_Flash_Torch_Level(kal_uint32 level)
{
    flash_torch_level(level);
    return 0;
}
#endif

static int FL_Disable(void)
{
    flash_strobe_prepare(0,0);
    flash_strobe_en();

    mtkcam_gpio_set(0,CAM_FLASH_FRONT_EN,0); //LGE_UPDATE [yonghwan.lym@lge.com] 2016/09/08, combine the camera GPIO function

    PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    g_duty =  duty;
    flash_strobe_level((char)duty);
    return 0;
}


static int FL_Init(void)
{
//LGE_UPDATE_S [kyunghun.oh@lge.com] 2016/10/06, add model defines and revision checking for flash
	flash_strobe_en = dummy_flash_strobe_en;
	flash_strobe_prepare = dummy_flash_strobe_prepare;
	flash_strobe_level = dummy_flash_strobe_level;

	#if defined(CONFIG_LEDS_LM3648)
		flash_strobe_en = lm3648_flash_strobe_en;
		flash_strobe_prepare = lm3648_flash_strobe_prepare;
		flash_strobe_level = lm3648_flash_strobe_level;
	#if defined(CONFIG_MACH_MT6750_LV7)
        flash_torch_level = lm3648_flash_torch_level;
	#endif
	#endif
//LGE_UPDATE_E [kyunghun.oh@lge.com] 2016/10/06, add model define and revision checking for flash

    mtkcam_gpio_set(0,CAM_FLASH_FRONT_EN,0); //LGE_UPDATE [yonghwan.lym@lge.com] 2016/09/08, combine the camera GPIO function



    INIT_WORK(&workTimeOut, work_timeOutFunc);
    flash_strobe_prepare(1,1);
    flash_strobe_level((char)32);
#if defined(CONFIG_MACH_MT6750_LV7)
    flash_torch_level(0);
#endif
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


static int FL_Uninit(void)
{
    FL_Disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
static void timerInit(void)
{
    g_timeOutTimeMs=1000; //1s
    hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
    g_timeOutTimer.function=ledTimeOutCallback;

}



static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
   int iFlashType = (int)FLASHLIGHT_NONE;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("RT4505 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("SUB_FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("SUB_FLASHLIGHT_DUTY: %d\n", (int)arg);
		strobe_width = arg;
            //FL_Init(); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-20] Flash is fired more than 4 times.
            FL_dim_duty(arg);
            break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("SUB_FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("SUB_FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
         case FLASHLIGHTIOC_G_FLASHTYPE:
                 iFlashType = FLASHLIGHT_LED_CONSTANT;
                 if(copy_to_user((void __user *) arg , (void*)&iFlashType , _IOC_SIZE(cmd)))
                 {
                         PK_DBG(" ioctl copy to user failed\n");
                         return -EFAULT;
                 }
                 break;
         case FLASHLIGHTIOC_T_STATE:
                 break;
     //LGE_CHANGE_S, set flash current feature, Himchan.Lee
    case FLASH_IOC_SET_FLASH_TORCH_LEVEL:
#if defined(CONFIG_MACH_MT6750_LV7)
        FL_Set_Flash_Torch_Level(arg);
#endif
        break;
    //LGE_CHANGE_E, set flash current feature, Himchan.Lee
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}

static int sub_strobe_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("sub_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("sub_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("sub_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
	PK_DBG("sub_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}

FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}
