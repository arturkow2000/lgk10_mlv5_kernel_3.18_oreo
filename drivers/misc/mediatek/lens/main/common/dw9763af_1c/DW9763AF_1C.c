/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*
 * DW9763AF_1C voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "DW9763AF_1C_DRV"
#define AF_I2C_SLAVE_ADDR        0x1C

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

typedef enum
{
    AF_disabled,
    AF_initializing,
    AF_driving,
    AF_releasing
}STATUS_NUM;
static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;
static char status_AF; //[LGE_UPDATE] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time
static char is_register_setted;

static int i2c_read(u8 a_u2Addr, u8 *a_puBuff)
{
    int i4RetValue = 0;
    char puReadCmd[1] = { (char)(a_u2Addr) };

    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puReadCmd, 1);

    if (i4RetValue != 1) {
        LOG_INF(" I2C write failed!!\n");
        return -1;
    }

    i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, (char *)a_puBuff, 1);
    if (i4RetValue != 1) {
        LOG_INF(" I2C read failed!!\n");
        return -1;
    }

    return 0;
}

#if 0
static u8 read_data(u8 addr)
{
    u8 get_byte = 0;

    i2c_read(addr, &get_byte);

    return get_byte;
}

static int s4DW9763AF_1C_ReadReg(unsigned short *a_pu2Result)
{
    *a_pu2Result = (read_data(0x03) << 8) + (read_data(0x04) & 0xff);

    return 0;
}
#endif
static int s4AF_WriteReg(u16 a_u2Data)
{
    int i4RetValue = 0;
    u8 is_busy = 0;
    char puSendCmd[3] = { 0x03, (char)(a_u2Data >> 8), (char)(a_u2Data & 0xFF) };

    g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

    g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);

    if(1)
    {
        int busy_count = 0;
        while(1)
        {
            i2c_read(0x05,&is_busy);
            if(is_busy & 0x01)
            {
                LOG_INF("[DW9763AF] I2C is busy now wait 5ms\n");
                mdelay(5);
                busy_count++;
                if(busy_count > 10)
                    break;
            }
            else
                break;
        }
    }
    if (i4RetValue < 0) {
        LOG_INF("I2C send failed!!\n");
        return -1;
    }

    return 0;
}

static inline int getAFInfo(__user stAF_MotorInfo * pstMotorInfo)
{
    stAF_MotorInfo stMotorInfo;

    stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
    stMotorInfo.u4InfPosition = g_u4AF_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR = 1;

    stMotorInfo.bIsMotorMoving = 1;

    if (*g_pAF_Opened >= 1)
        stMotorInfo.bIsMotorOpen = 1;
    else
        stMotorInfo.bIsMotorOpen = 0;

    if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
        LOG_INF("copy to user failed when getting motor information\n");

    return 0;
}
static void set_reg_drive(void)
{

}

static void set_reg_init(void)
{
    char puSendCmd1[2] = {0x02, 0x02};	//Active mode
    char puSendCmd2[2] = {0x06, 0x21};	// SAC2, clock = x1(defualt)
    char puSendCmd3[2] = {0x07, 0x20};	//operation time = (32 * 0.1ms + 6.3ms) * 1(clock) = 9.5ms

    i2c_master_send(g_pstAF_I2Cclient, puSendCmd1, 2); mdelay(1);
    i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2); mdelay(1);
    i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2); mdelay(1);
}
static void set_reg_release(void)
{
}

static inline int moveAF(unsigned long a_u4Position)
{
//    int ret = 0;
    int thresh_hold_position = 0;
    if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
        LOG_INF("out of range\n");
        return -EINVAL;
    }
    if (*g_pAF_Opened == 1) {
//        unsigned short InitPos;
        status_AF = AF_initializing;//[LGE_UPDATE] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time
#if 0
        ret = s4DW9763AF_1C_ReadReg(&InitPos);
        if (ret == 0) {
            LOG_INF("Init Pos %6d\n", InitPos);
            spin_lock(g_pAF_SpinLock);
            g_u4CurrPosition = (unsigned long)InitPos;
            spin_unlock(g_pAF_SpinLock);
        } else {
            spin_lock(g_pAF_SpinLock);
            g_u4CurrPosition = 0;
            spin_unlock(g_pAF_SpinLock);
        }
#endif
        spin_lock(g_pAF_SpinLock);
        *g_pAF_Opened = 2;
        spin_unlock(g_pAF_SpinLock);
    }

    if (g_u4CurrPosition == a_u4Position && status_AF != AF_initializing)
        return 0;
    printk("[DW9763AF] TargetPosition = %lu\n", a_u4Position);
    spin_lock(g_pAF_SpinLock);
    g_u4TargetPosition = a_u4Position;
    spin_unlock(g_pAF_SpinLock);

    if(status_AF == AF_initializing && is_register_setted != AF_initializing)
    {
        printk("[DW9763AF] set_initializing_mode\n");
        set_reg_init();
        is_register_setted = 0;
        if(g_u4TargetPosition == 0)
        {
            status_AF = AF_driving;
            return 0;
        }
        is_register_setted = AF_initializing;
    }
    else if(status_AF == AF_releasing && is_register_setted != AF_releasing)
    {
        printk("[DW9763AF] set_releasing_mode\n");
        set_reg_release();
        is_register_setted = AF_releasing;
    }
    else if(status_AF == AF_driving && is_register_setted != AF_driving)
    {
        printk("[DW9763AF] set_driving_mode\n");
        set_reg_drive();
        is_register_setted = AF_driving;
    }

#if 1
    while(abs((int)g_u4CurrPosition - (int)g_u4TargetPosition) > 7 && (g_u4CurrPosition < 320 || g_u4TargetPosition < 320) && status_AF == AF_driving)
    {
        if(((int)g_u4TargetPosition - (int)g_u4CurrPosition) > 0)
            thresh_hold_position = 7;
        else
            thresh_hold_position = -7;
        if (s4AF_WriteReg((unsigned short)(g_u4CurrPosition + thresh_hold_position)) == 0) {
        } else {
            LOG_INF("set I2C failed when moving the motor\n");
        }
        //[LGE_UPDATE_S] [kyunghun.oh@lge.com] [2017-09-22] Calculates the current position even if an i2c error has occurred.
        spin_lock(g_pAF_SpinLock);
        g_u4CurrPosition = (unsigned long)(g_u4CurrPosition + thresh_hold_position);
        spin_unlock(g_pAF_SpinLock);
        //[LGE_UPDATE_E] [kyunghun.oh@lge.com] [2017-09-22] Calculates the current position even if an i2c error has occurred.
        msleep(3);
    }
#endif

#if 1
    while(abs((int)g_u4CurrPosition - (int)g_u4TargetPosition) > 120 && status_AF == AF_initializing)
    {
        if(((int)g_u4TargetPosition - (int)g_u4CurrPosition) > 0)
            thresh_hold_position = 120;
        else
            thresh_hold_position = -120;
        if (s4AF_WriteReg((unsigned short)(g_u4CurrPosition + thresh_hold_position)) == 0) {
        } else {
            LOG_INF("set I2C failed when moving the motor\n");
        }

        //[LGE_UPDATE_S] [kyunghun.oh@lge.com] [2017-09-22] Calculates the current position even if an i2c error has occurred.
        spin_lock(g_pAF_SpinLock);
        g_u4CurrPosition = (unsigned long)(g_u4CurrPosition + thresh_hold_position);
        spin_unlock(g_pAF_SpinLock);
        //[LGE_UPDATE_E] [kyunghun.oh@lge.com] [2017-09-22] Calculates the current position even if an i2c error has occurred.
        msleep(10);
    }
#endif
    if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
    } else {
        LOG_INF("set I2C failed when moving the motor\n");
    }
    //[LGE_UPDATE_S] [kyunghun.oh@lge.com] [2017-09-22] Calculates the current position even if an i2c error has occurred.
    spin_lock(g_pAF_SpinLock);
    g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
    spin_unlock(g_pAF_SpinLock);
    //[LGE_UPDATE_E] [kyunghun.oh@lge.com] [2017-09-22] Calculates the current position even if an i2c error has occurred.
    if(status_AF == AF_initializing)
        status_AF = AF_driving;//[LGE_UPDATE] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time

    return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
    spin_lock(g_pAF_SpinLock);
    g_u4AF_INF = a_u4Position;
    spin_unlock(g_pAF_SpinLock);
    return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
    spin_lock(g_pAF_SpinLock);
    g_u4AF_MACRO = a_u4Position;
    spin_unlock(g_pAF_SpinLock);
    return 0;
}

/* ////////////////////////////////////////////////////////////// */
long DW9763AF_1C_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch (a_u4Command) {
        case AFIOC_G_MOTORINFO:
            i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
            break;

        case AFIOC_T_MOVETO:
            i4RetValue = moveAF(a_u4Param);
            break;

        case AFIOC_T_SETINFPOS:
            i4RetValue = setAFInf(a_u4Param);
            break;

        case AFIOC_T_SETMACROPOS:
            i4RetValue = setAFMacro(a_u4Param);
            break;

        default:
            LOG_INF("No CMD\n");
            i4RetValue = -EPERM;
            break;
    }

    return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int DW9763AF_1C_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
    char puSendCmd5[2] = {0x02, 0x01};  //power down mode
    LOG_INF("Start\n");
    status_AF = AF_releasing;//[LGE_UPDATE] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time

    if (*g_pAF_Opened == 2)
        LOG_INF("Wait\n");

    if (*g_pAF_Opened) {
        LOG_INF("Free\n");
        spin_lock(g_pAF_SpinLock);
        *g_pAF_Opened = 0;
        spin_unlock(g_pAF_SpinLock);
    }

    if (*g_pAF_Opened == 0)
    {
        unsigned long release_position;
        while(g_u4CurrPosition > 350)
        {
            release_position = g_u4CurrPosition;
            release_position -= 50;
            moveAF(release_position);
            msleep(1);
        }
        while(g_u4CurrPosition > 250)
        {
            release_position = g_u4CurrPosition;
            release_position -= 10;
            moveAF(release_position);
            msleep(1);
        }
        while(g_u4CurrPosition > 180)
        {
            release_position = g_u4CurrPosition;
            release_position -= 50;
            moveAF(release_position);
            msleep(1);
        }
         while(g_u4CurrPosition > 70)
        {
            release_position = g_u4CurrPosition;
            release_position -= 10;
            moveAF(release_position);
            msleep(1);
        }
         while(g_u4CurrPosition > 30)
        {
            release_position = g_u4CurrPosition;
            release_position -= 30;
            moveAF(release_position);
            msleep(1);
        }
        msleep(3);
        status_AF = AF_disabled;//[LGE_UPDATE] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time
        g_u4CurrPosition = 0;
        i2c_master_send(g_pstAF_I2Cclient, puSendCmd5, 2);	//Power down mode
    }
    LOG_INF("End\n");

    return 0;
}

void DW9763AF_1C_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock,
        int *pAF_Opened)
{
    g_pstAF_I2Cclient = pstAF_I2Cclient;
    g_pAF_SpinLock = pAF_SpinLock;
    g_pAF_Opened = pAF_Opened;
}
