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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/kernel.h>       /* printk() */
#include <linux/slab.h>         /* kmalloc() */
#include <linux/fs.h>           /* everything... filp_open */
#include <linux/errno.h>        /* error codes */
#include <linux/types.h>        /* size_t */
#include <linux/proc_fs.h>      /*proc */
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <linux/aio.h>
#include <linux/uaccess.h>        /*set_fs get_fs mm_segment_t */
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/unistd.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>
#include "partition.h"
#include "lg_partition.h"

#define PART_NAME "misc2"

int g_init_write_size = 1;

static char misc2_path[64];

static int get_misc2_path(void)
{
	struct hd_struct *part = NULL;

	if (misc2_path[0])
		return 0;

	part = get_part(PART_NAME);
	if (!part) {
		pr_err("Not find partition %s\n", PART_NAME);
		return 1;
	}

#ifdef CONFIG_MTK_EMMC_SUPPORT
	snprintf(misc2_path, sizeof(misc2_path), "/dev/block/mmcblk0p%d", part->partno);
#elif defined(CONFIG_MTK_UFS_BOOTING)
	snprintf(misc2_path, sizeof(misc2_path), "/dev/block/sdc%d", part->partno);
#else
#error can not copy misc2 path
#endif

	put_part(part);

	return 0;
}

bool _LGE_GENERIC_WRITE_FUN(unsigned char *buff, unsigned int offset, unsigned int length)
{
	int ret;
	struct file *filp;
	unsigned char *tmp;
	mm_segment_t curr_fs;

	ret = get_misc2_path();
	if (ret == 1) {
		pr_err("get MISC2 partition info fail!\n");
		return -1;
	}

	filp = filp_open(misc2_path, O_RDWR, 0660);
	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		pr_err("misc2_path : %s\n", misc2_path);
		pr_err("Open MISC2 partition fail! errno=%d\n", ret);
		return -1;
	}
	pr_err("WRITE - Open MISC2 partition OK!\n");
	if (g_init_write_size == 0) {
		struct mtd_info_user info;

		if (filp->f_op->unlocked_ioctl)
			filp->f_op->unlocked_ioctl(filp, MEMGETINFO, (unsigned long) &info);
		else if (filp->f_op->compat_ioctl)
			filp->f_op->compat_ioctl(filp, MEMGETINFO, (unsigned long) &info);

		if (info.writesize != EMMC_BLOCK_SIZE) {
			pr_err("write size error!info.writesize=%d,EMMC_BLOCK_SIZE=%d\n",
				info.writesize, EMMC_BLOCK_SIZE);
			g_init_write_size = 0;
			filp_close(filp, NULL);
			return false;
		}
		g_init_write_size = 1;
	}

	filp->f_op->llseek(filp, offset * EMMC_BLOCK_SIZE, SEEK_SET);
	tmp = kzalloc(EMMC_BLOCK_SIZE, GFP_KERNEL);
	if (!tmp) {
		pr_err("malloc memory fail!\n");
		filp_close(filp, NULL);
		return false;
	}
	memset(tmp, 0x0, EMMC_BLOCK_SIZE);
	curr_fs = get_fs();
	set_fs(KERNEL_DS);
	memcpy(tmp, buff, length);
	ret = filp->f_op->write(filp, tmp, EMMC_BLOCK_SIZE, &(filp->f_pos));
	if (EMMC_BLOCK_SIZE != ret) {
		pr_err("write fail!errno=%d\n", ret);
		filp_close(filp, NULL);
		kfree(tmp);
		set_fs(curr_fs);
		return false;
	}
	set_fs(curr_fs);
	kfree(tmp);
	filp_close(filp, NULL);
	pr_debug("WRITE -  OK!\n");

	return true;
}

bool _LGE_GENERIC_READ_FUN(unsigned char *buff, unsigned int offset, unsigned int length)
{

	int ret;
	struct file *filp;
	unsigned char *tmp;
	mm_segment_t curr_fs;

	ret = get_misc2_path();
	if (ret == 1) {
		pr_err("get MISC2 partition info fail!\n");
		return -1;
	}

	filp = filp_open(misc2_path, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		pr_err("misc2_path : %s\n", misc2_path);
		pr_err("Open MISC2 partition fail! errno=%d\n", ret);
		return -1;
	}
	pr_err("READ - Open MISC2 partition OK!\n");
	if (g_init_write_size == 0) {
		struct mtd_info_user info;

		if (filp->f_op->unlocked_ioctl)
			filp->f_op->unlocked_ioctl(filp, MEMGETINFO, (unsigned long) &info);
		else if (filp->f_op->compat_ioctl)
			filp->f_op->compat_ioctl(filp, MEMGETINFO, (unsigned long) &info);

		if (info.writesize != EMMC_BLOCK_SIZE) {
			pr_err("write size error!info.writesize=%d,EMMC_BLOCK_SIZE=%d\n",
				info.writesize, EMMC_BLOCK_SIZE);
			g_init_write_size = 0;
			filp_close(filp, NULL);
			return false;
		}
		g_init_write_size = 1;
	}

	filp->f_op->llseek(filp, offset * EMMC_BLOCK_SIZE, SEEK_SET);
	tmp = kzalloc(EMMC_BLOCK_SIZE, GFP_KERNEL);
	if (!tmp) {
		pr_err("malloc memory fail!\n");
		filp_close(filp, NULL);
		return false;
	}
	memset(tmp, 0x0, EMMC_BLOCK_SIZE);
	curr_fs = get_fs();
	set_fs(KERNEL_DS);

	ret = filp->f_op->read(filp, tmp, EMMC_BLOCK_SIZE, &(filp->f_pos));
	if (EMMC_BLOCK_SIZE != ret) {
		pr_err("read fail!errno=%d\n", ret);
		filp_close(filp, NULL);
		kfree(tmp);
		set_fs(curr_fs);
		return false;
	}
	memcpy(buff, tmp, length);
	set_fs(curr_fs);
	kfree(tmp);
	filp_close(filp, NULL);
	pr_debug("READ -  OK!\n");

	return true;
}

unsigned char calchecksum(unsigned char *data, int leng)
{
	unsigned char csum;

	csum = 0xFF;

	for ( ; leng > 0; leng--)
		csum += *data++;

	return ~csum;
}

extern int of_scan_meid_node(char *meid, int len);

unsigned char tc1_read_meid(unsigned char *meid)
{
	bool ret;

#ifdef CONFIG_MACH_LGE
	ret = of_scan_meid_node(&meid[1], LGE_FAC_MEID_LEN);
#else
	ret = LGE_FacReadMeid(0 , meid);
#endif
	return (unsigned char) ret;
}

#define DEF_TC1_SYNC_MAKER      0x26
#define DEF_TC1_SYNC_LENG       16

unsigned char tc1_read_meid_syncform(unsigned char *meid, int leng)
{
	bool ret;
	/*
	sync format
	Maker (1 byte) : 0x26
	Meid  (14 bytes) : from LSB to MSB
	Checksum (1 byte) : using "calchecksum" function.
	*/

	if (leng != DEF_TC1_SYNC_LENG)
		return false;

	meid[0]  = DEF_TC1_SYNC_MAKER;

#ifdef CONFIG_MACH_LGE
	ret = of_scan_meid_node(&meid[1], LGE_FAC_MEID_LEN);
#else
	ret = LGE_FacReadMeid(0, &meid[1]);
#endif
	meid[DEF_TC1_SYNC_LENG-1] = calchecksum(&meid[1], LGE_FAC_MEID_LEN);

	return (unsigned char) ret;
}

bool LGE_FacWriteWifiMacAddr(unsigned char *wifiMacAddr, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN(wifiMacAddr, LGE_FAC_WIFI_MAC_ADDR_OFFSET, LGE_FAC_WIFI_MAC_ADDR_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteWifiMacAddr);

bool LGE_FacReadWifiMacAddr(unsigned char *wifiMacAddr)
{
	return _LGE_GENERIC_READ_FUN(wifiMacAddr, LGE_FAC_WIFI_MAC_ADDR_OFFSET, LGE_FAC_WIFI_MAC_ADDR_LEN);
}
EXPORT_SYMBOL(LGE_FacReadWifiMacAddr);

bool LGE_FacWriteBtAddr(unsigned char *btAddr, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN(btAddr, LGE_FAC_BT_ADDR_OFFSET, LGE_FAC_BT_ADDR_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteBtAddr);

bool LGE_FacReadBtAddr(unsigned char *btAddr)
{
	return _LGE_GENERIC_READ_FUN(btAddr, LGE_FAC_BT_ADDR_OFFSET, LGE_FAC_BT_ADDR_LEN);
}
EXPORT_SYMBOL(LGE_FacReadBtAddr);

const unsigned int imei_mapping_table[4] = {LGE_FAC_IMEI_1_OFFSET,
										LGE_FAC_IMEI_0_OFFSET,
										LGE_FAC_IMEI_2_OFFSET,
										LGE_FAC_IMEI_3_OFFSET};

bool LGE_FacWriteImei(unsigned char imei_type, unsigned char *imei, bool needFlashProgram)
{
	if (imei_mapping_table[imei_type] == LGE_FAC_IMEI_ENDMARK)
		return false;

	return _LGE_GENERIC_WRITE_FUN(imei, imei_mapping_table[imei_type], LGE_FAC_IMEI_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteImei);

bool LGE_FacReadImei(unsigned char imei_type, unsigned char *imei)
{
	if (imei_mapping_table[imei_type] == LGE_FAC_IMEI_ENDMARK)
		return false;

	return _LGE_GENERIC_READ_FUN(imei, imei_mapping_table[imei_type], LGE_FAC_IMEI_LEN);
}
EXPORT_SYMBOL(LGE_FacReadImei);

bool LGE_FacWriteSimLockType(unsigned char simLockType, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN(&simLockType, LGE_FAC_SIM_LOCK_TYPE_OFFSET, LGE_FAC_SIM_LOCK_TYPE_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteSimLockType);

bool LGE_FacReadSimLockType(unsigned char *simLockType)
{
	return _LGE_GENERIC_READ_FUN(simLockType, LGE_FAC_SIM_LOCK_TYPE_OFFSET, LGE_FAC_SIM_LOCK_TYPE_LEN);
}
EXPORT_SYMBOL(LGE_FacReadSimLockType);

bool LGE_FacWriteNetworkCodeListNum(unsigned short *networkCodeListNum, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN((unsigned char *)&networkCodeListNum,
				LGE_FAC_NETWORK_CODE_LIST_NUM_OFFSET, LGE_FAC_NETWORK_CODE_LIST_NUM_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteNetworkCodeListNum);

bool LGE_FacReadNetworkCodeListNum(unsigned short *networkCodeListNum)
{
	return _LGE_GENERIC_READ_FUN((unsigned char *)networkCodeListNum,
				LGE_FAC_NETWORK_CODE_LIST_NUM_OFFSET, LGE_FAC_NETWORK_CODE_LIST_NUM_LEN);
}
EXPORT_SYMBOL(LGE_FacReadNetworkCodeListNum);

bool LGE_FacWriteUnlockCodeVerifyFailCount(unsigned char failCount, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN(&failCount,
			LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteUnlockCodeVerifyFailCount);

bool LGE_FacReadUnlockCodeVerifyFailCount(unsigned char *failCount)
{
	return _LGE_GENERIC_READ_FUN(failCount,
			LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_LEN);
}
EXPORT_SYMBOL(LGE_FacReadUnlockCodeVerifyFailCount);

#ifndef CONFIG_MACH_LGE
bool LGE_FacWriteUnlockFailCount(unsigned char simLockType, unsigned char failCount, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN(&failCount,
				LGE_FAC_UNLOCK_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_FAIL_COUNT_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteUnlockFailCount);

bool LGE_FacReadUnlockFailCount(unsigned char simLockType, unsigned char *failCount)
{
	return _LGE_GENERIC_READ_FUN(failCount,
				LGE_FAC_UNLOCK_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_FAIL_COUNT_LEN);
}
EXPORT_SYMBOL(LGE_FacReadUnlockFailCount);
#endif

bool LGE_FacWriteUnlockCode(struct FactoryUnlockCode *unlockCode, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN((unsigned char *)unlockCode,
				LGE_FAC_UNLOCK_CODE_OFFSET, LGE_FAC_UNLOCK_CODE_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteUnlockCode);

bool LGE_FacVerifyUnlockCode(unsigned char simLockType, unsigned char *unlockCode, bool *isOk)
{
	*isOk = true;
	return true;
}
EXPORT_SYMBOL(LGE_FacVerifyUnlockCode);

bool LGE_FacCheckUnlockCodeValidness(bool *isValid)
{
	*isValid = true;
	return true;
}
EXPORT_SYMBOL(LGE_FacCheckUnlockCodeValidness);

bool LGE_FacWriteNetworkCode(struct FactoryNetworkCode *networkCode,
					unsigned short networkCodeListNum, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN((unsigned char *)networkCode,
				LGE_FAC_NETWORK_CODE_OFFSET, LGE_FAC_NETWORK_CODE_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteNetworkCode);

bool LGE_FacReadNetworkCode(struct FactoryNetworkCode *networkCode, unsigned short networkCodeListNum)
{
	return _LGE_GENERIC_READ_FUN((unsigned char *)networkCode,
				LGE_FAC_NETWORK_CODE_OFFSET, LGE_FAC_NETWORK_CODE_LEN);
}
EXPORT_SYMBOL(LGE_FacReadNetworkCode);

bool LGE_FacCheckNetworkCodeValidness(unsigned char simLockType, bool *isValid)
{
	*isValid = true;
	return true;
}
EXPORT_SYMBOL(LGE_FacCheckNetworkCodeValidness);

bool LGE_FacInitSimLockData(void)
{
	return true;
}
EXPORT_SYMBOL(LGE_FacInitSimLockData);

bool LGE_FacReadFusgFlag(unsigned char *fusgFlag)
{
	return _LGE_GENERIC_READ_FUN(fusgFlag, LGE_FAC_FUSG_FLAG_OFFSET, LGE_FAC_FUSG_FLAG_LEN);
}
EXPORT_SYMBOL(LGE_FacReadFusgFlag);

bool LGE_FacWriteFusgFlag(unsigned char fusgFlag, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN(&fusgFlag, LGE_FAC_FUSG_FLAG_OFFSET, LGE_FAC_FUSG_FLAG_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteFusgFlag);

bool LGE_FacReadDataVersion(unsigned char *dataVersion)
{
	return _LGE_GENERIC_READ_FUN(dataVersion, LGE_FAC_DATA_VERSION_OFFSET, LGE_FAC_DATA_VERSION_LEN);
}
EXPORT_SYMBOL(LGE_FacReadDataVersion);

bool LGE_FacWriteDataVersion(unsigned char *dataVersion, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN(dataVersion, LGE_FAC_DATA_VERSION_OFFSET, LGE_FAC_DATA_VERSION_LEN);
}
EXPORT_SYMBOL(LGE_FacWriteDataVersion);

bool LGE_FacReadPid(unsigned char *pid)
{
	return _LGE_GENERIC_READ_FUN(pid, LGE_FAC_PID_OFFSET, LGE_FAC_PID_LEN);
}
EXPORT_SYMBOL(LGE_FacReadPid);

bool LGE_FacWritePid(unsigned char *pid, bool needFlashProgram)
{
	return _LGE_GENERIC_WRITE_FUN(pid, LGE_FAC_PID_OFFSET, LGE_FAC_PID_LEN);
}
EXPORT_SYMBOL(LGE_FacWritePid);

void LGE_FacGetSoftwareversion(bool isOriginalVersion, unsigned char *pVersion)
{
	int index = 0;

	for (index = 0; index < LGE_FAC_SV_LEN; index++)
		pVersion[index] = index;
}
EXPORT_SYMBOL(LGE_FacGetSoftwareversion);

/* sbp_interface */
bool LGE_FacReadNetworkCode_SBP(struct FactoryNetworkCode *networkCode, unsigned short networkCodeListNum, unsigned int length)
{
	int i = 0;
	bool result;
	char *temp_buf = NULL;

	if (length != LGE_FAC_NETWORK_CODE_SIZE_SBP)
	{
		pr_err("LGE_FAC_NETWORK_CODE_SIZE_SBP length Error!\n");
		return 0;
	}

	temp_buf = (char *)kmalloc(LGE_FAC_NETWORK_CODE_SIZE_SBP, GFP_KERNEL);

	if (temp_buf == NULL)
	{
		pr_err("LGE_FAC_NETWORK_CODE_SIZE_SBP alloc fail!\n");
		return 0;
	}

	if (( networkCode == 0) ||(networkCodeListNum > LGE_FAC_MAX_NTCODE_COUNT_SBP ))
	{
		pr_err("networkCodeListNum=%d is too big!\n", networkCodeListNum);
		return 0;
	}

	result = _LGE_GENERIC_READ_FUN(temp_buf, LGE_FAC_NETWORK_CODE_IDX01, (sizeof(struct FactoryNetworkCode) * networkCodeListNum));

	if( result != 1 )
	{
		pr_err("LGE_FAC_NETWORK_CODE_IDX01 read fail\n");
		kfree(temp_buf);
		return 0;
	}

	for(i = 0; i < LGE_FAC_MAX_NTCODE_COUNT_SBP; i++)
	{
		memcpy(&networkCode[i], &temp_buf[sizeof(struct FactoryNetworkCode)*i], sizeof(struct FactoryNetworkCode));
	}

	kfree(temp_buf);

	return 1;
}
EXPORT_SYMBOL(LGE_FacReadNetworkCode_SBP);

bool LGE_FacReadOneBinaryHWInfo(unsigned char *data)
{
    bool result = 0;

    result = _LGE_GENERIC_READ_FUN(data, LGE_ONE_BINARY_HWINFO_IDX, LGE_ONE_BINARY_HWINFO_SIZE);

    if( result == 1 )
    {
        pr_err("LGE_ONE_BINARY_HWINFO_IDX read OK, OneBinaryHWInfo=%s\n", data);
    }
	else
	{
		pr_err("LGE_ONE_BINARY_HWINFO_IDX read fail\n");
	}

    return result;
}
EXPORT_SYMBOL(LGE_FacReadOneBinaryHWInfo);

bool LGE_FacWriteOneBinaryHWInfo(unsigned char *data, bool needFlashProgram)
{
    bool result = 0;

    result = _LGE_GENERIC_WRITE_FUN(data, LGE_ONE_BINARY_HWINFO_IDX, LGE_ONE_BINARY_HWINFO_SIZE);

    if( result == 1 )
    {
        pr_err("LGE_ONE_BINARY_HWINFO_IDX write OK, OneBinaryHWInfo=%s\n", data);
    }
	else
	{
		pr_err("LGE_ONE_BINARY_HWINFO_IDX write fail!");
	}

    return result;
}
EXPORT_SYMBOL(LGE_FacWriteOneBinaryHWInfo);

bool LGE_FacReadSVN_SBP (unsigned char *svn)
{
	bool result = 0;

	result = _LGE_GENERIC_READ_FUN (svn, LGE_FAC_IMEI_SVN_IDX, LGE_FAC_IMEI_SVN_SIZE);

	if( result == 1 )
	{
		pr_debug("LGE_FAC_IMEI_SVN_IDX read OK, svn=%s\n", svn);
	}
	else
	{
		pr_debug("LGE_FAC_IMEI_SVN_IDX read fail\n");
		return 0;
	}

	return 1;
}
/* sbp_interface */

int LGE_API_test(void)
{
	unsigned char buff[EMMC_BLOCK_SIZE];
	unsigned char temp[EMMC_BLOCK_SIZE];
	int index = 0;

	memset(buff, 0xa, EMMC_BLOCK_SIZE);
	for (index = 0; index < LGE_FAC_WIFI_MAC_ADDR_LEN; index++)
		buff[index] = LGE_FAC_WIFI_MAC_ADDR_OFFSET+index;

	LGE_FacWriteWifiMacAddr(buff, true);
	memset(buff, 0xb, EMMC_BLOCK_SIZE);
	LGE_FacReadWifiMacAddr(buff);

	snprintf(temp, 25, "%02X %02X %02X %02X %02X %02X %02X",
		buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6]);

	pr_err("kernel wifi data : %s\n", temp);

#if 0
	memset(buff, 0xc, EMMC_BLOCK_SIZE);
	for (index = 0; index < LGE_FAC_BT_ADDR_LEN; index++)
		buff[index] = LGE_FAC_BT_ADDR_OFFSET+0xc;

	LGE_FacWriteBtAddr(buff, true);
	memset(buff, 0xd, EMMC_BLOCK_SIZE);
	LGE_FacReadBtAddr(buff);
	pr_err("kernel BT data:");
	for (index = 0; index < 100; index++)
		pr_debug(" 0x%x", buff[index]);

	pr_err("\n");
#endif

	return 0;
}
EXPORT_SYMBOL(LGE_API_test);

MODULE_AUTHOR("Kai Zhu@mediatek.com");
MODULE_DESCRIPTION("access partition API");
MODULE_LICENSE("GPL");
