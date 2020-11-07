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
 * DW9763AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "DW9763AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;
static int g_init_flag = 0; // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/16, reduce the AF Open/Release Time

static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int i2c_read(u8 a_u2Addr, u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[1] = { (char)(a_u2Addr) };

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puReadCmd, 1);
// LGE_UPDATE_S [yonghwan.lym@lge.com] 2016/03/23, Reduce the AF Tick Noise - Add busy check routine
#if 0
	if (i4RetValue != 2) {
#else
	if (i4RetValue != 1) {
#endif
// LGE_UPDATE_E [yonghwan.lym@lge.com] 2016/03/23, Reduce the AF Tick Noise - Add busy check routine
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

static u8 read_data(u8 addr)
{
	u8 get_byte = 0;

	i2c_read(addr, &get_byte);

	return get_byte;
}

static int s4DW9763AF_ReadReg(unsigned short *a_pu2Result)
{
	*a_pu2Result = (read_data(0x03) << 8) + (read_data(0x04) & 0xff);

	return 0;
}

// LGE_UPDATE_S [yonghwan.lym@lge.com] 2016/03/23, Reduce the AF Tick Noise - Add busy check routine
static int s4DW9763AF_ReadReg_FlagBit(unsigned short * a_pu2Result)
{
    *a_pu2Result = (read_data(0x05)&0x01);

    //LOG_INF("[DW9763AF][CHECK_BUSY]  s4DW9763AF_ReadReg %d \n",  *a_pu2Result);
    return 0;
}

static int s4DW9763AF_CheckFlagBit(void)
{
    unsigned short move_Flag;
    int check_count = 0;
    while (1)
    {
        s4DW9763AF_ReadReg_FlagBit(&move_Flag);
        if (move_Flag == 0)
        {
            //LOG_INF("[DW9763AF] CHECK_BUSY [%d] ***NOT SLEEP(BREAK)**** check_count(%d)\n", move_Flag, check_count);
            break;
        }
        else
        {
            //LOG_INF("[DW9763AF] CHECK_BUSY [%d] ***SLEEP**** check_count(%d)\n", move_Flag, check_count);
            mdelay(3);
        }
        check_count++;
        if (check_count > 5)
        {
            //LOG_INF("[DW9763AF] CHECK_BUSY [%d] check_count(%d) (break by check_count)\n", move_Flag, check_count);
            break;
        }
    }
    return 0;
}
// LGE_UPDATE_E [yonghwan.lym@lge.com] 2016/03/23, Reduce the AF Tick Noise - Add busy check routine

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[3] = { 0x03, (char)(a_u2Data >> 8), (char)(a_u2Data & 0xFF) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);

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

// LGE_UPDATE_S [yonghwan.lym@lge.com] 2016/02/16, reduce the AF Open/Release Time
static void initdrv_sac3(void)
{
	char puSendCmd[2] = { 0x06, 0x61 };	// SAC3 mode
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);
}

static void initdrv_sac2(void)
{
	char puSendCmdArray[6][2] = {
		{0x02, 0x01}, // Power down mode operating
		{0x02, 0x00}, // Normal operating
		{0xFE, 0xFE}, // delay
		{0x02, 0x02}, // Ringing control enable
		{0x06, 0x21}, // SAC2 mode, PRESC[2:0]=b'001
		{0x07, 0x25}  // Tvib 10ms
	};
	unsigned char cmd_number;
#if 0
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
#endif

	LOG_INF("InitDrv[1] %p, %p\n", &(puSendCmdArray[1][0]), puSendCmdArray[1]);
	LOG_INF("InitDrv[2] %p, %p\n", &(puSendCmdArray[2][0]), puSendCmdArray[2]);

	for (cmd_number = 0; cmd_number < 6; cmd_number++) {
		if (puSendCmdArray[cmd_number][0] != 0xFE)
			i2c_master_send(g_pstAF_I2Cclient, puSendCmdArray[cmd_number], 2);
		else
			udelay(100);
	}
}

static void exitdrv(void)
{
	char puSendCmd[2] = {0x02, 0x00}; // Ringing control disable (direct mode)
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);
}
// LGE_UPDATE_E [yonghwan.lym@lge.com] 2016/02/16, reduce the AF Open/Release Time

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;
	unsigned long TempPosition = 0;
	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

// LGE_UPDATE_S [yonghwan.lym@lge.com] 2016/02/16, reduce the AF Open/Release Time
	if(g_init_flag == 0 && a_u4Position != 0) {
		initdrv_sac2();
		g_init_flag = 1;
	}
	else if( g_init_flag == 1 ) {
		initdrv_sac3();
		g_init_flag = 2;
	}
// LGE_UPDATE_E [yonghwan.lym@lge.com] 2016/02/16, reduce the AF Open/Release Time

	if (*g_pAF_Opened == 1) {
		unsigned short InitPos;

		//initdrv(); // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/16, reduce the AF Open/Release Time
		ret = s4DW9763AF_ReadReg(&InitPos);

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

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	/* LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); */
    TempPosition = abs(g_u4CurrPosition - g_u4TargetPosition);
    if((g_init_flag == 2) && (TempPosition > 40))
    {
        TempPosition = g_u4TargetPosition - (TempPosition/2);
        if (s4AF_WriteReg((unsigned short)TempPosition) == 0) {
            spin_lock(g_pAF_SpinLock);
            g_u4CurrPosition = (unsigned long)TempPosition;
            spin_unlock(g_pAF_SpinLock);
        } else {
            LOG_INF("set I2C failed when moving the motor\n");
        }
        s4DW9763AF_CheckFlagBit();
    }

// LGE_UPDATE_S [yonghwan.lym@lge.com] 2016/03/23, Reduce the AF Tick Noise - Add busy check routine
    if((g_init_flag == 1) && (g_u4TargetPosition > 100))
    {
        TempPosition = 100;
        while(TempPosition < g_u4TargetPosition)
        {
            if (s4AF_WriteReg((unsigned short)TempPosition) == 0) {
                spin_lock(g_pAF_SpinLock);
                g_u4CurrPosition = (unsigned long)TempPosition;
                spin_unlock(g_pAF_SpinLock);
            } else {
                LOG_INF("set I2C failed when moving the motor\n");
            }
            s4DW9763AF_CheckFlagBit();
            TempPosition += 100;
        }
    }
// LGE_UPDATE_E [yonghwan.lym@lge.com] 2016/03/23, Reduce the AF Tick Noise - Add busy check routine

	if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
	}

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
long DW9763AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
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
int DW9763AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	unsigned long release_position = g_u4CurrPosition; //LGE_CHANGE: [2015-12-03] yonghwan.lym@lge.com, Code for AF de-noise during the camera exit
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2)
		LOG_INF("Wait\n");

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");
// LGE_UPDATE_S [yonghwan.lym@lge.com] 2016/02/16, reduce the AF Open/Release Time
#if 0
//LGE_CHANGE_S: [2015-12-03] yonghwan.lym@lge.com, Code for AF de-noise during the camera exit
		while(release_position > 60/*40*/) // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/12, improve the swiching time.(Main1 Camera -> Other Camera)
		{
			release_position -= 50/*30*/; // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/12, improve the swiching time.(Main1 Camera -> Other Camera)
			if (s4AF_WriteReg((unsigned short)release_position) != 0) {
				LOG_INF("set I2C failed when moving the motor\n");
				break;
			}
			msleep(5);
		}
//LGE_CHANGE_E: [2015-12-03] yonghwan.lym@lge.com, Code for AF de-noise during the camera exit
#else
		if(release_position > 300)
		{
			while(release_position > 360) // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/12, improve the swiching time.(Main1 Camera -> Other Camera)
			{
				release_position -= 50; // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/12, improve the swiching time.(Main1 Camera -> Other Camera)
				s4DW9763AF_CheckFlagBit(); // LGE_UPDATE [yonghwan.lym@lge.com] 2016/03/23, Reduce the AF Tick Noise - Add busy check routine
				if (s4AF_WriteReg((unsigned short)release_position) != 0) {
					LOG_INF("set I2C failed when moving the motor\n");
					break;
				}
			}

			s4DW9763AF_CheckFlagBit(); // LGE_UPDATE [yonghwan.lym@lge.com] 2016/03/23, Reduce the AF Tick Noise - Add busy check routine
			s4AF_WriteReg(300);
			release_position = 300;
		}

		exitdrv();
		while(release_position > 10) // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/12, improve the swiching time.(Main1 Camera -> Other Camera)
		{
			release_position -= 5; // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/12, improve the swiching time.(Main1 Camera -> Other Camera)
			s4DW9763AF_CheckFlagBit(); // LGE_UPDATE [yonghwan.lym@lge.com] 2016/03/23, Reduce the AF Tick Noise - Add busy check routine
			if (s4AF_WriteReg((unsigned short)release_position) != 0) {
				LOG_INF("set I2C failed when moving the motor\n");
				break;
			}
			udelay(500);
		}
#endif
// LGE_UPDATE_E [yonghwan.lym@lge.com] 2016/02/16, reduce the AF Open/Release Time
		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
		g_init_flag = 0; // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/16, reduce the AF Open/Release Time
	}

	LOG_INF("End\n");

	return 0;
}

void DW9763AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock,
			    int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}
