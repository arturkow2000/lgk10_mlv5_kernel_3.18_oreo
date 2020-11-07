#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

#include "STR_Firmware.h"
#include "touch_ssd6600.h"

// structure for update
struct sentron_fw_group {
        struct sentron_fw section;
        struct sentron_fw_group *next;
};
// for version & verify check
struct sentron_fw_version {
		struct sentron_fw *section;
                u16 version_address;
		unsigned int update_flag;
		unsigned int verify_flag;
};
// level of state for parsing
typedef enum {
        LEVEL_NONE = 0,
        LEVEL_ADDRESS,
        LEVEL_LENGTH,
        LEVEL_ERASE_SIZE,
        LEVEL_VERSION,
        LEVEL_CHECKSUM,
        LEVEL_RESERVE_01,
        LEVEL_RESERVE_02,
        LEVEL_RESERVE_03,
        LEVEL_CONTENT,
} VAL_LEVEL;

// level of state for dollar 
typedef enum {
	LEVEL_DOLLAR_NONE = 0,
	LEVEL_DOLLAR_ERASE_TYPE,
} VAL_DOLLAR_LEVEL;

struct sentron_fw_version m_fw_version[] = {
	{&STR_FW, 0x0000, FW_EFLASH_FLAG_CPU_ONLY, FW_EFLASH_FLAG_CPU_ONLY},
	{&STR_FW_CFG, 0x0000, FW_EFLASH_FLAG_CPU_CFG, FW_EFLASH_FLAG_CPU_CFG},
	{&STR_SYS_CFG, 0x0000, FW_EFLASH_FLAG_SYS_CFG, FW_EFLASH_FLAG_SYS_CFG},
	{&STR_MPFPM, 0x0000, FW_EFLASH_FLAG_MP_FPM, FW_EFLASH_FLAG_MP_FPM},
	{&STR_MPFDM, 0x0000, FW_EFLASH_FLAG_MP_FDM, FW_EFLASH_FLAG_MP_FDM},
	{NULL, 0x0000, FW_EFLASH_FLAG_HW_CAL, FW_EFLASH_FLAG_HW_CAL},
	{&STR_FPM, 0x0000, FW_EFLASH_FLAG_FPM, FW_EFLASH_FLAG_FPM},
	{&STR_FDM, 0x0000, FW_EFLASH_FLAG_FDM, FW_EFLASH_FLAG_FDM},
	{&STR_TMC_REG, 0x0000, FW_EFLASH_FLAG_TMC_REG, FW_EFLASH_FLAG_TMC_REG},
	{&STR_DCSW, 0x0000, FW_EFLASH_FLAG_SW_CAL, FW_EFLASH_FLAG_SW_CAL},
	{NULL, 0x0000, FW_EFLASH_FLAG_INFO, FW_EFLASH_FLAG_INFO},
	{&STR_FW_CFG, 0x7E10, FW_EFLASH_FLAG_CPU_ONLY|FW_EFLASH_FLAG_CPU_CFG|FW_EFLASH_FLAG_SYS_CFG, 0x00000000 },
};

u8 m_all_version[8]={0,};
u32 m_display_version=0;
u32 m_hidden_version=0;

#define VERSION_DISPLAY_ADDR    0x7E11
#define VERSION_HIDDEN_ADDR     0x7E12

static int SSD6600_verify_checksum_only(struct device *dev);

static int convHexStr2Int(unsigned char hex[], int hexLen, unsigned int *iRet)
{
        int ret = 0;
	int hexIdx = 0;
	int temp = 0;
	int i=0;

	if( hexLen < 0 ) return ERROR_PARSING_INVALID_DATATYPE;

	hexIdx = hexLen - 1;
	*iRet = 0;

	for( i=0; i<hexLen; i++ ) {
		if( hex[i] >= 0x30 && hex[i] <= 0x39 ) temp = hex[i] - 0x30;
		else if( hex[i] >= 0x41 && hex[i] <= 0x46 ) temp = hex[i] - 0x37;
		else if( hex[i] >= 0x61 && hex[i] <= 0x66 ) temp = hex[i] - 0x57;
		else { ret = -2; break; }

		*iRet += (temp << (4*hexIdx));
		hexIdx--;
	}

	return ret;
}

int fw_calc_checksum(int len, unsigned short *tmpContent, unsigned int *checksum)
{
        int i=0;
        int ret = ERROR_SUCCESS;
        unsigned short sum = 0x00;
        unsigned short xor = 0x00;
    
        if( tmpContent == NULL ) return ERROR_PARSING_CHECKSUM_FAIL;

        for( i=0; i<len; i++ ) {
                sum += tmpContent[i];
                xor ^= tmpContent[i];
        }
    
        *checksum = (xor<<16)|sum;
    
        TOUCH_E(">>>> sum:0x%04x, xor:0x%04x, checksum:0x%08x\n", sum, xor, *checksum);
        return ret;
}

int fw_checksum(struct sentron_fw sec)
{
        int ret = ERROR_SUCCESS;
        unsigned int checksum = 0x00;
	unsigned short *tmpContent = NULL;
	int len = 0;

	if( sec.address == 0x0000 || sec.address == 0x7E00 || sec.address == 0x7F00 ) {
		len = sec.byte_cnt/2+(sec.byte_cnt&0x01);
		tmpContent = (unsigned short *)sec.content;
	} else {
		len = (sec.byte_cnt-CONTENT_HEADER_SIZE)/2+((sec.byte_cnt-CONTENT_HEADER_SIZE)&0x01);
		tmpContent = (unsigned short *)(sec.content+CONTENT_HEADER_SIZE);	
	}
		
        ret = fw_calc_checksum(len, tmpContent, &checksum);

        if( ret == ERROR_SUCCESS ) {
                if( sec.checksum != checksum ) ret = ERROR_PARSING_CHECKSUM_FAIL;
        }

        return ret;
}

int fw_free(struct sentron_fw_group *fw, int data_free)
{
    int i=30;
    struct sentron_fw_group *ptr = fw;
    
    while( fw != NULL ) {
           if( (i--) < 1 ) break;
           
           ptr = fw->next;
           
           if( data_free == 1 && fw->section.content != NULL ) kfree(fw->section.content);
           kfree(fw);
           fw = ptr;
    }
    
    return 0;
}

static int fw_ds_eflash_write(struct device *dev, u16 addr, int efaddr, u8 *wr, int wr_count, int dsaddr)
{
	u8 data[FW_MAX_I2C_DATA_COUNT+32];
	int nByte = 0;
	int ret = 0;

	memset(data, 0, sizeof(data));

	data[0] = (efaddr&0xFF);
	data[1] = ((efaddr>>8)&0xFF);
	data[2] = (wr_count&0xFF);
	data[3] = ((wr_count>>8)&0xFF);
	data[4] = (dsaddr&0xFF);
	data[5] = ((dsaddr>>8)&0xFF);

	memcpy((u8 *)&data[6], wr, wr_count);

	nByte = wr_count + 6;

	ret = ssd6600_reg_write(dev, addr, data , nByte);
	if (ret < 0)
	{
		TOUCH_E("I2C eflash WRITE FAIL : 0x%x", addr);
		return ret;
	}

	return ret;
}

static int fw_ds_eflash_read(struct device *dev, u16 addr, int efaddr, u8 *rd, int rd_count, int dsaddr)
{
	u8 data[FW_MAX_I2C_DATA_COUNT+32];
	int ret = 0;

	memset(data, 0, sizeof(data));

	data[0] = (addr&0xFF);
	data[1] = ((addr>>8)&0xFF);
	data[2] = (efaddr&0xFF);
	data[3] = ((efaddr>>8)&0xFF);
	data[4] = (rd_count&0xFF);
	data[5] = ((rd_count>>8)&0xFF);
	data[6] = (dsaddr&0xFF);
	data[7] = ((dsaddr>>8)&0xFF);

	if( (ret = ssd6600_reg_read_ex(dev, data, 8, rd, rd_count)) < 0 ) {
		TOUCH_E("0x%04X | 0x%04X i2c eflash read fail(1)!!", addr, efaddr);
		return ret;
	}

	return ret;
}

static int fw_ds_read_version(struct device *dev, u16 eflashAddr, int *version )
{
	int ret = ERROR_EFLAH_READ_FAIL;
	int retry = FW_MAX_RETRY_COUNT;

	do {
		if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, eflashAddr, (u8 *)version, 4, DS_READ_PTR)) >= 0 ) {
			ret = ERROR_SUCCESS;
			break;
		}
	} while( (retry--) > 0 );

	TOUCH_E("ret : 0x%08x \t VERSION : 0x%08x", ret, *version);

	return ret;
}

static int fw_ds_read_version_all(struct device *dev)
{
	int ver=0;
	int ret=0;

	// FW/FW_CFG/SYS_CFG version
	if( (ret=fw_ds_read_version(dev, EFLAH_ADDR_CPU_CFG+0x10, &ver)) < ERROR_SUCCESS ) goto out;
	TOUCH_E("FW/FW_CFG/SYS_CFG Read VERSION : 0x%08x", ver );
	m_all_version[0] = (ver>>8)&0xFF;
	m_all_version[1] = ver&0xFF;

	// TMC_REG version
	if( (ret=fw_ds_read_version(dev, EFLAH_ADDR_TMC_REG, &ver)) < ERROR_SUCCESS ) goto out;
	TOUCH_E("TMC_REG Read VERSION : 0x%08x", ver );
	m_all_version[2] = ver&0xFF;

	// DCSW	version
	if( (ret=fw_ds_read_version(dev, EFLAH_ADDR_FW_INFO, &ver)) < ERROR_SUCCESS ) goto out;
	TOUCH_E("DSCW Read VERSION : 0x%08x", ver );
	m_all_version[3] = ver&0xFF;

	// FPM version
	if( (ret=fw_ds_read_version(dev, EFLAH_ADDR_FPM, &ver)) < ERROR_SUCCESS ) goto out;
	TOUCH_E("FPM Read VERSION : 0x%08x", ver );
	m_all_version[4] = ver&0xFF;

	// FDM version
	if( (ret=fw_ds_read_version(dev, EFLAH_ADDR_FDM, &ver)) < ERROR_SUCCESS ) goto out;
	TOUCH_E("FDM Read VERSION : 0x%08x", ver );
	m_all_version[5] = ver&0xFF;

	// MP_FPM version
	if( (ret=fw_ds_read_version(dev, EFLAH_ADDR_MP_FPM, &ver)) < ERROR_SUCCESS ) goto out;
	TOUCH_E("MP_FPM Read VERSION : 0x%08x", ver );
	m_all_version[6] = ver&0xFF;

	// MP_FDM version
	if( (ret=fw_ds_read_version(dev, EFLAH_ADDR_MP_FDM, &ver)) < ERROR_SUCCESS ) goto out;
	TOUCH_E("MP_FDM Read VERSION : 0x%08x", ver );
	m_all_version[7] = ver&0xFF;

	TOUCH_E("ALL VERSION : 0x%02x %02x %02x %02x %02x %02x %02x %02x ", m_all_version[0], m_all_version[1], m_all_version[2], m_all_version[3], m_all_version[4], m_all_version[5], m_all_version[6], m_all_version[7] );

	// m_display_version
	if( (ret=fw_ds_read_version(dev, EFLAH_ADDR_CPU_CFG+0x11, &m_display_version)) < ERROR_SUCCESS ) goto out;
	TOUCH_E("DISPLAY VERSION : 0x%08x", ver );
	// m_hidden_version
	if( (ret=fw_ds_read_version(dev, EFLAH_ADDR_CPU_CFG+0x12, &m_hidden_version)) < ERROR_SUCCESS ) goto out;
	TOUCH_E("HIDDEN VERSION : 0x%08x", ver );
out :
	return ret;
}

static int fw_ds_eflash_erase_all(struct device *dev)
{
	int ret = 0;
	u16 rd = 0x00;
	int i=0;
	int verify_data = 0xFFFF;
	u16 startAddr = 0x0000;
	int pageNum = FW_ERASE_ALL_PAGENUM;

	TOUCH_E("EFLASH ERASE ALL START ");

	if( (ret = ds_eflash_read(dev, 0x0000, (u8 *)&rd, 2)) < 0 ) return ERROR_EFLAH_ERASE_FAIL;	// The address send to 0x0000
	if( (ret = ds_eflash_write(dev, 0xA008, 0x0028)) < 0 ) return ERROR_EFLAH_ERASE_FAIL;
	mdelay(60);

	for( i=0; i<pageNum; i++ ) {
#if (!sentron_timecheck)
		TOUCH_E(".");
#endif
		if( (ret = ds_eflash_write(dev, 0xA005, startAddr+(i*0x100))) < 0 ) return ERROR_EFLAH_ERASE_FAIL;
		if( (ret = ds_eflash_write(dev, 0xA008, 0x0031)) < 0 ) return ERROR_EFLAH_ERASE_FAIL;

		if( (ret = ds_eflash_read(dev, 0xA006, (u8 *)&rd, 2)) < 0 ) return ERROR_EFLAH_ERASE_FAIL;
		verify_data &= rd;

		if( (ret = ds_eflash_read(dev, 0xA007, (u8 *)&rd, 2)) < 0 ) return ERROR_EFLAH_ERASE_FAIL;
		verify_data &= rd;
	}
	if( verify_data != 0xFFFF ) {
		TOUCH_E("VERIFY FAIL!!\n");
		ret = ERROR_EFLAH_ERASE_FAIL;
	} else {
		TOUCH_E("VERIFY SUCCESS!!\n");
	}

	TOUCH_E("EFLASH ERASE ALL END(0x%08x)",ret);

	return ret;
}

static int fw_ds_eflash_erase(struct device *dev, u16 startAddr, int pageNum)
{
	int ret = 0;
	int i=0;
	int verify_data = 0xFFFF;
	u16 rd = 0x00;

	TOUCH_E("EFLASH ERASE START ");
	for( i=0; i<pageNum; i++ ) {
		if( (ret = ds_eflash_write(dev, 0xA005, startAddr+(i*0x100))) < 0 ) return ERROR_EFLAH_ERASE_FAIL;
		if( (ret = ds_eflash_write(dev, 0xA008, 0x0024)) < 0 ) return ERROR_EFLAH_ERASE_FAIL;
		mdelay(100);
	}
	TOUCH_E("END\n");
	TOUCH_E("EFLASH ERASE VERIFY START ");

	for( i=0; i<pageNum; i++ ) {
		if( (ret = ds_eflash_write(dev, 0xA005, startAddr+(i*0x100))) < 0 ) return ERROR_EFLAH_ERASE_FAIL;
		if( (ret = ds_eflash_write(dev, 0xA008, 0x0031)) < 0 ) return ERROR_EFLAH_ERASE_FAIL;

		if( (ret = ds_eflash_read(dev, 0xA006, (u8 *)&rd, 2)) < 0 ) return ERROR_EFLAH_ERASE_FAIL;
		verify_data &= rd;

		if( (ret = ds_eflash_read(dev, 0xA007, (u8 *)&rd, 2)) < 0 ) return ERROR_EFLAH_ERASE_FAIL;
		verify_data &= rd;
	}
	TOUCH_E("END\n");
	if( verify_data != 0xFFFF ) {
		TOUCH_E("VERIFY FAIL!!\n");
		ret = ERROR_EFLAH_ERASE_FAIL;
	} else {
		TOUCH_E("VERIFY SUCCESS!!\n");
	}
	
	return ret;
}

static int fw_SSD6600_init(struct device *dev)
{
	int ret = 0;
	u16 rd = 0x00;

	TOUCH_E("initialize start >>>>>>>>>>>>>>>");
	if( ds_read_boot_st(dev, (u16 *)&rd) < 0 ) return ERROR_SYSTEM_FAIL;

	if( (rd&0xFFFF) == 0xFFFF ) {
		TOUCH_E("DS read boot status(0x%04x) fail!!", rd);
		return ERROR_SYSTEM_FAIL;
	}

	if( ds_clear_int(dev) < 0 ) return ERROR_SYSTEM_FAIL;

	if( (ret = ds_eflash_write(dev, 0xE000, 0x0000)) < 0 ) return ERROR_SYSTEM_FAIL;
	if( (ret = ds_eflash_read(dev, 0xE000, (u8 *)&rd, 2)) < 0 ) return ERROR_SYSTEM_FAIL;
	TOUCH_E(">>>>> 0xE000 read : 0x%04x", rd );
	if( (rd&0xFFFF) == 0xFFFF ) return ERROR_SYSTEM_FAIL;

	if( (ret = ds_eflash_write(dev, 0xE009, 0x0000)) < 0 ) return ERROR_SYSTEM_FAIL;
	if( (ret = ds_eflash_read(dev, 0xE009, (u8 *)&rd, 2)) < 0 ) return ERROR_SYSTEM_FAIL;
	TOUCH_E(">>>>> 0xE009 read : 0x%04x", rd );
	if( (rd&0xFFFF) == 0xFFFF ) return ERROR_SYSTEM_FAIL;

	if( (ret = ds_eflash_write(dev, 0xE00A, 0x0001)) < 0 ) return ERROR_SYSTEM_FAIL;
	if( (ret = ds_eflash_read(dev, 0xE00A, (u8 *)&rd, 2)) < 0 ) return ERROR_SYSTEM_FAIL;
	TOUCH_E(">>>>> 0xE00A read : 0x%04x", rd );
	if( (rd&0xFFFF) == 0xFFFF ) return ERROR_SYSTEM_FAIL;

	if( (ret = ds_eflash_write(dev, 0xF000, 0x0003)) < 0 ) return ERROR_SYSTEM_FAIL;
	if( (ret = ds_eflash_read(dev, 0xF000, (u8 *)&rd, 2)) < 0 ) return ERROR_SYSTEM_FAIL;
	TOUCH_E(">>>>> 0xF000 read : 0x%04x", rd );
	if( (rd&0xFFFF) == 0xFFFF ) return ERROR_SYSTEM_FAIL;

        if( (ret = ds_eflash_write(dev, 0xA003, 0x00FF)) < 0 ) return ERROR_SYSTEM_FAIL;
	if( (ret = ds_eflash_read(dev, 0xA003, (u8 *)&rd, 2)) < 0 ) return ERROR_SYSTEM_FAIL;
	TOUCH_E(">>>>> 0xA003 read : 0x%04x", rd );
	if( (rd&0xFFFF) == 0xFFFF ) return ERROR_SYSTEM_FAIL;

	if( (ret = ds_eflash_write(dev, 0xA004, 0x00FF)) < 0 ) return ERROR_SYSTEM_FAIL;
	if( (ret = ds_eflash_read(dev, 0xA004, (u8 *)&rd, 2)) < 0 ) return ERROR_SYSTEM_FAIL;
	TOUCH_E(">>>>> 0xA004 read : 0x%04x", rd );
	if( (rd&0xFFFF) == 0xFFFF ) return ERROR_SYSTEM_FAIL;
	TOUCH_E("<<<<<<<<<<<< initialize end(%d)", ret);
	return ret;
}

static int fw_SSD6600_verify_checksum(struct device *dev, int address)
{
	int ret = 0;
	int nBlock_cnt;
	int byte_cnt=0, tByte_cnt=0;
	int len = 0;
	int i=0;
	u8 *verify_data = NULL;
	unsigned short *tmpContent = NULL;
	unsigned int checksum = 0x00;
	unsigned int header[3] = {0,};
	unsigned int hChecksum = 0x00;
	int tAddress = 0;

	// header read 12bytes
	if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, address, (u8 *)header, CONTENT_HEADER_SIZE, DS_READ_PTR)) < 0 ) {
		ret = ERROR_EFLAH_READ_FAIL;
	} else {
		TOUCH_E("READ eFlash Header(12bytes) : 0x%08x 0x%08x 0x%08x", header[0], header[1], header[2] );
		tByte_cnt = byte_cnt = header[1];
		hChecksum = header[2];
		TOUCH_E("Converted byte_cnt = %d(0x%08x), checksum : 0x%08x", byte_cnt, byte_cnt, hChecksum);

		nBlock_cnt = byte_cnt/FW_MAX_I2C_DATA_COUNT;
		verify_data = (u8 *)kmalloc(byte_cnt+2, GFP_KERNEL);
		tAddress = address + (CONTENT_HEADER_SIZE/4);
		TOUCH_E("CHECKSUM Address : 0x%08x", address);
		TOUCH_E("CHECKSUM READ START ");
		for( i=0; i<(nBlock_cnt+1); i++ ) {
			if( byte_cnt >= FW_MAX_I2C_DATA_COUNT ) {
				if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, tAddress+i*(FW_MAX_I2C_DATA_COUNT/4), verify_data+i*FW_MAX_I2C_DATA_COUNT, FW_MAX_I2C_DATA_COUNT, DS_READ_PTR)) < 0 ) {ret = ERROR_EFLAH_READ_FAIL; break;}
			} else if( byte_cnt > 0 ) {
				if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, tAddress+i*(FW_MAX_I2C_DATA_COUNT/4), verify_data+i*FW_MAX_I2C_DATA_COUNT, byte_cnt, DS_READ_PTR)) < 0 ) {ret = ERROR_EFLAH_READ_FAIL; break;}
			}

			byte_cnt -= FW_MAX_I2C_DATA_COUNT;
		}

		TOUCH_E("CHECKSUM READ END\n");
		if( ret >= 0 ) {
			len = tByte_cnt/2+(tByte_cnt&0x01);
			tmpContent = (unsigned short *)verify_data;
			ret = fw_calc_checksum(len, tmpContent, &checksum);

			if( ret >= ERROR_SUCCESS ) {
				if( hChecksum != checksum ) ret = ERROR_PARSING_CHECKSUM_FAIL;
			}
		}

		kfree(verify_data);
	}

	if( ret >= 0 ) TOUCH_E("CHECKSUM SUCCESS ");
	else TOUCH_E("CHECKSUM FAIL ");

	return ret;	
}

static int fw_SSD6600_verify(struct device *dev, struct sentron_fw *fw)
{
	int ret = ERROR_SUCCESS;
	int nBlock_cnt;
	int byte_cnt;
	int i=0;
	u8 *verify_data = NULL;

	if( fw == NULL ) ret = ERROR_UPDATE_INIT_FAIL;
	else {
		nBlock_cnt = fw->byte_cnt/FW_MAX_I2C_DATA_COUNT;
		verify_data = (u8 *)kmalloc(fw->byte_cnt+2, GFP_KERNEL);
		byte_cnt = fw->byte_cnt;
		TOUCH_E("VERIFY READ START (address : 0x%04x) ", fw->address);
		for( i=0; i<(nBlock_cnt+1); i++ ) {
			if( byte_cnt >= FW_MAX_I2C_DATA_COUNT ) {
				if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, fw->address+i*(FW_MAX_I2C_DATA_COUNT/4), verify_data+i*FW_MAX_I2C_DATA_COUNT, FW_MAX_I2C_DATA_COUNT, DS_READ_PTR)) < 0 ) {ret = ERROR_EFLAH_READ_FAIL; break;}
			} else if( byte_cnt > 0 ) {
				if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, fw->address+i*(FW_MAX_I2C_DATA_COUNT/4), verify_data+i*FW_MAX_I2C_DATA_COUNT, byte_cnt, DS_READ_PTR)) < 0 ) {ret = ERROR_EFLAH_READ_FAIL; break;}
			}

			byte_cnt -= FW_MAX_I2C_DATA_COUNT;
		}

		TOUCH_E("VERIFY READ END\n");
		if( ret >= 0 ) {
			for( i=0; i<fw->byte_cnt; i++ ) {
				if( fw->content[i] != verify_data[i] ) {
					ret = ERROR_UPDATE_VERIFY_FAIL;
					TOUCH_E("VERIFY Fail(%d) org data : 0x%02x, read data : 0x%02x", i, fw->content[i], verify_data[i]);
					break;
				}
			}
			TOUCH_E("\n");

			if( ret >= 0 ) {
				ret = ERROR_SUCCESS;
				TOUCH_E("VERIFY SUCCESS ");
			} else TOUCH_E("VERIFY FAIL(1) ");
		} else TOUCH_E("VERIFY FAIL(2) ");

		kfree(verify_data);
	}

	return ret;	
}

static int fw_SSD6600_update(struct device *dev, struct sentron_fw *fw)
{
	int ret = 0;
	int nBlock_cnt;
	int byte_cnt;
	int i=0;
	u16 rd = 0x00;

	TOUCH_E("update start >>>>>>>>>>>>>>>");
	
	if( ds_read_boot_st(dev, (u16 *)&rd) < 0 ) return ERROR_UPDATE_INIT_FAIL;
	TOUCH_E("boot status : 0x%04x", rd);

	nBlock_cnt = fw->byte_cnt/FW_MAX_I2C_DATA_COUNT;
	byte_cnt = fw->byte_cnt;
	TOUCH_E("UPDATE START(%d)(%d) 0x%02x 0x%02x 0x%02x 0x%02x ", nBlock_cnt, byte_cnt, fw->content[0], fw->content[1], fw->content[2], fw->content[3]);
	for( i=0; i<(nBlock_cnt+1); i++ ) {
		if( byte_cnt >= FW_MAX_I2C_DATA_COUNT ) {
			if( (ret = fw_ds_eflash_write(dev, DS_EFLASH_WRITE_01, fw->address+i*(FW_MAX_I2C_DATA_COUNT/4), fw->content+i*FW_MAX_I2C_DATA_COUNT, FW_MAX_I2C_DATA_COUNT, DS_WRITE_PTR)) < 0 ) return ERROR_UPDATE_WRITE_FAIL;
		} else  if( byte_cnt > 0 ) {
			if( (ret = fw_ds_eflash_write(dev, DS_EFLASH_WRITE_01, fw->address+i*(FW_MAX_I2C_DATA_COUNT/4), fw->content+i*FW_MAX_I2C_DATA_COUNT, byte_cnt, DS_WRITE_PTR)) < 0 ) return ERROR_UPDATE_WRITE_FAIL;
		}

		byte_cnt -= FW_MAX_I2C_DATA_COUNT;
		mdelay(1);
	}
	TOUCH_E("UPDATE END\n");
	ret = fw_SSD6600_verify(dev, fw);
	TOUCH_E("<<<<<<<<<<<< update end");

	return ret;
}

static int fw_ds_read_info_by_bin(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sentron_device *ftdev = (struct sentron_device *)touch_get_device(ts);
	u32 val=0;
	u8 buff[10]={0,};
	const u8 *data = fw->data;
	struct sentron_info *bin_info = ftdev->bin_info;
	
	if( fw->size < 66 ) {
		TOUCH_E("file size error. size : %d\n", (int)fw->size);
		goto out;
	}
	
	memset( bin_info, 0x00, sizeof(struct sentron_info) );
	// VERSION 1
	memcpy( buff, data+1, 8);	// $+[version 8bytes]
	if( (convHexStr2Int( buff, 8, &val ) ) < ERROR_SUCCESS ) goto out;
	memcpy( bin_info->version1, (u8 *)&val, sizeof(u32) );
	TOUCH_I("VERSION 1 : %d.%d\n", bin_info->version1[1], bin_info->version1[0] );
	
	// VERSION 2
	memcpy( buff, data+12, 8);	// $+[version 8bytes]
	if( (convHexStr2Int( buff, 8, &val ) ) < ERROR_SUCCESS ) goto out;
	memcpy( bin_info->version2, (u8 *)&val, sizeof(u32) );
	TOUCH_I("VERSION 2 : %d.%d\n", bin_info->version2[1], bin_info->version2[0] );
	
	// Product ID 1
	memcpy( buff, data+23, 8);	// $+[version 8bytes]
	if( (convHexStr2Int( buff, 8, &val ) ) < ERROR_SUCCESS ) goto out;
	bin_info->prdID1[0] = (val>>24)&0xFF;
	bin_info->prdID1[1] = (val>>16)&0xFF;
	bin_info->prdID1[2] = (val>>8)&0xFF;
	bin_info->prdID1[3] = (val)&0xFF;
	
	// Product ID 2
	memcpy( buff, data+34, 8);	// $+[version 8bytes]
	if( (convHexStr2Int( buff, 8, &val ) ) < ERROR_SUCCESS ) goto out;
	bin_info->prdID2[0] = (val>>24)&0xFF;
	bin_info->prdID2[1] = (val>>16)&0xFF;
	bin_info->prdID2[2] = (val>>8)&0xFF;
	bin_info->prdID2[3] = (val)&0xFF;
	TOUCH_I("Product ID : %s%s\n", bin_info->prdID1, bin_info->prdID2 );
	
	// IC Name 1
	memcpy( buff, data+45, 8);	// $+[version 8bytes]
	if( (convHexStr2Int( buff, 8, &val ) ) < ERROR_SUCCESS ) goto out;
	bin_info->ICName1[0] = (val>>24)&0xFF;
	bin_info->ICName1[1] = (val>>16)&0xFF;
	bin_info->ICName1[2] = (val>>8)&0xFF;
	bin_info->ICName1[3] = (val)&0xFF;
	
	// Product ID 2
	memcpy( buff, data+56, 8);	// $+[version 8bytes]
	if( (convHexStr2Int( buff, 8, &val ) ) < ERROR_SUCCESS ) goto out;
	bin_info->ICName2[0] = (val>>24)&0xFF;
	bin_info->ICName2[1] = (val>>16)&0xFF;
	bin_info->ICName2[2] = (val>>8)&0xFF;
	bin_info->ICName2[3] = (val)&0xFF;
	TOUCH_I("IC Name : %s%s\n", bin_info->ICName1, bin_info->ICName2 );
	
	bin_info->haveflag = 1;
	
	return 0;
out :
	return -1;
}

static int fw_ds_read_info_by_fw(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sentron_device *ftdev = (struct sentron_device *)touch_get_device(ts);
	struct sentron_info *fw_info = ftdev->fw_info;
	u32 val=0;
	int ret=0;
	
	TOUCH_I("Read F/W information START");
	memset( ftdev->fw_info, 0x00, sizeof(struct sentron_info) );
	// version 1
	if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, EFLAH_ADDR_CPU_CFG+0x11, (u8 *)&(val), 4, DS_READ_PTR)) < ERROR_SUCCESS ) goto out;
	memcpy( fw_info->version1, (u8 *)&val, sizeof(u32) );
	TOUCH_I("VERSION 1 : %d.%d\n", fw_info->version1[1], fw_info->version1[0] );
	
	// version 2
	if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, EFLAH_ADDR_CPU_CFG+0x12, (u8 *)&(val), 4, DS_READ_PTR)) < ERROR_SUCCESS ) goto out;
	memcpy( fw_info->version2, (u8 *)&val, sizeof(u32) );
	TOUCH_I("VERSION w : %d.%d\n", fw_info->version2[1], fw_info->version2[0] );
	
	// Product ID 1
	if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, EFLAH_ADDR_CPU_CFG+0x13, (u8 *)&(val), 4, DS_READ_PTR)) < ERROR_SUCCESS ) goto out;
	fw_info->prdID1[0] = (val>>24)&0xFF;
	fw_info->prdID1[1] = (val>>16)&0xFF;
	fw_info->prdID1[2] = (val>>8)&0xFF;
	fw_info->prdID1[3] = (val)&0xFF;
	
	// Product ID 2
	if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, EFLAH_ADDR_CPU_CFG+0x14, (u8 *)&(val), 4, DS_READ_PTR)) < ERROR_SUCCESS ) goto out;
	fw_info->prdID2[0] = (val>>24)&0xFF;
	fw_info->prdID2[1] = (val>>16)&0xFF;
	fw_info->prdID2[2] = (val>>8)&0xFF;
	fw_info->prdID2[3] = (val)&0xFF;
	TOUCH_I("Product ID : %s%s\n", fw_info->prdID1, fw_info->prdID2 );
	
	// IC Name 1
	if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, EFLAH_ADDR_CPU_CFG+0x15, (u8 *)&(val), 4, DS_READ_PTR)) < ERROR_SUCCESS ) goto out;
	fw_info->ICName1[0] = (val>>24)&0xFF;
	fw_info->ICName1[1] = (val>>16)&0xFF;
	fw_info->ICName1[2] = (val>>8)&0xFF;
	fw_info->ICName1[3] = (val)&0xFF;
	
	// IC Name 2
	if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, EFLAH_ADDR_CPU_CFG+0x16, (u8 *)&(val), 4, DS_READ_PTR)) < ERROR_SUCCESS ) goto out;
	fw_info->ICName2[0] = (val>>24)&0xFF;
	fw_info->ICName2[1] = (val>>16)&0xFF;
	fw_info->ICName2[2] = (val>>8)&0xFF;
	fw_info->ICName2[3] = (val)&0xFF;
	TOUCH_I("IC Name : %s%s\n", fw_info->ICName1, fw_info->ICName2 );
	fw_info->haveflag = 1;
out :
	return ret;
}

static int SSD6600_firmware_update(struct device *dev, struct sentron_fw_group *fw, int all)
{
	int ret=0;
	int retry = FW_MAX_RETRY_COUNT;
	int retry2 = FW_MAX_RETRY_COUNT;
	struct sentron_fw_group *gPtr = NULL;
	struct sentron_fw *ptr = NULL;
	// TODO : erase all or erase each
	TOUCH_E("update start >>>>>>>>>>>>>>>");

	if( ret >= 0 ) {
		if( all == BOOT_UPDATE_ALL ) {
			retry2 = FW_MAX_RETRY_COUNT;
			do {
				if( (ret = fw_ds_eflash_erase_all(dev)) >= 0 ) break;
				mdelay(1);
			} while( (retry2--) > 1 );
		}

		if( ret >=  ERROR_SUCCESS ) {
			gPtr = fw;

			while( gPtr != NULL ) {
				ptr = &(gPtr->section);
				TOUCH_E("\n\n\n\n");
				TOUCH_E("EFLASH ADDRESS : 0x%04x", ptr->address);
				TOUCH_E("EFLASH ERASE PAGE NUM : 0x%04x", ptr->erase_page_cnt);
				TOUCH_E("EFLASH DATA COUNT : 0x%04x", ptr->byte_cnt);

				if( ptr->byte_cnt > 0 && ptr->content != NULL ) {
					if( all == BOOT_UPDATE_EACH ) {
						retry2 = FW_MAX_RETRY_COUNT;
						do {
							if( (ret = fw_ds_eflash_erase(dev, ptr->address, ptr->erase_page_cnt)) >= 0 ) break;
							mdelay(1);
						} while( (retry2--) > 1 );
					}
					retry = FW_MAX_RETRY_COUNT;
					do {
						if( (ret = fw_SSD6600_update(dev, ptr)) >= 0 ) break;
						mdelay(1);	// 10 -> 1
					} while( (retry--) > 1 );

					if( ret < 0 ) break;
				}

				gPtr = gPtr->next;
			}
		}
	}
	TOUCH_E("<<<<<<<<<<<< update end ret=0x%08x", ret);

	return ret;
}

static int SSD6600_Boot_sys_cfg_check(struct device *dev)
{
	int ret=0;
	int retry2=0, retry=0;
	do {
		retry2 = FW_MAX_RETRY_COUNT;
		do {
			if( (ret = fw_ds_eflash_erase(dev, STR_SYS_CFG.address, STR_SYS_CFG.erase_page_cnt)) >= 0 ) break;
			mdelay(1);
		} while( (retry2--) > 1 );
		
		if( ret < 0 ) break;

		if( (ret = fw_SSD6600_update(dev, &STR_SYS_CFG)) >= 0 ) break;
		mdelay(1);
	} while( (retry--) > 1 );

	return ret;
}

static int SSD6600_Boot_inf_check(struct device *dev, u16 val)
{
	unsigned int D0, D1, D2;
	int ret = 0;
	int retry = 0;

	if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, EFLAH_ADDR_INFO_OSC_TUNE, (u8 *)&D0, 4, DS_READ_PTR)) < 0 ) return ret;

	if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, EFLAH_ADDR_INFO_DCO32K_CAL, (u8 *)&D1, 4, DS_READ_PTR)) < 0 ) return ret;

	if( (ret = fw_ds_eflash_read(dev, DS_EFLASH_READ_01, EFLAH_ADDR_INFO_LDORDAC_CAL, (u8 *)&D2, 4, DS_READ_PTR)) < 0 ) return ret;

	TOUCH_E("D0=0x%08x , D1=0x%08x, D2=0x%08x", D0, D1, D2);
#if 1
	if( (val&BOOT_STATUS_ERR_INF_OSC_TRIM_INVALID) == BOOT_STATUS_ERR_INF_OSC_TRIM_INVALID ) D0 = 0x00040004;

	if( (val&BOOT_STATUS_ERR_INF_IDCO32K_INVALID) == BOOT_STATUS_ERR_INF_IDCO32K_INVALID ) D1 = 0x00050005;

	if( (val&BOOT_STATUS_ERR_INF_LDORDAC_INVALID) == BOOT_STATUS_ERR_INF_LDORDAC_INVALID ) D2 = 0x00000000;

	retry = FW_MAX_RETRY_COUNT;

	do {
		if( (ret = fw_ds_eflash_erase(dev, EFLAH_ADDR_INFO_OSC_TUNE, 1)) >= 0 ) break;
		mdelay(10);
	} while( (retry--) > 1 );
	
	if( ret < 0 ) return ret;

	if( (ret = fw_ds_eflash_write(dev, DS_EFLASH_WRITE_01, EFLAH_ADDR_INFO_OSC_TUNE, (u8 *)&D0, 4, DS_WRITE_PTR)) < 0 ) return ret;

	if( (ret = fw_ds_eflash_write(dev, DS_EFLASH_WRITE_01, EFLAH_ADDR_INFO_DCO32K_CAL, (u8 *)&D1, 4, DS_WRITE_PTR)) < 0 ) return ret;

	if( (ret = fw_ds_eflash_write(dev, DS_EFLASH_WRITE_01, EFLAH_ADDR_INFO_LDORDAC_CAL, (u8 *)&D2, 4, DS_WRITE_PTR)) < 0 ) return ret;
#endif
	return ret;
}

static inline int fw_SSD6600_make_link(struct sentron_fw_group **head, struct sentron_fw_group **tail, struct sentron_fw_group *ptr)
{
	int ret = 0;

	if( ptr != NULL ) {
		if( *head == NULL ) { 
			*head = *tail = ptr;
		} else {
			(*tail)->next = ptr;
			*tail = ptr;
		}
	}

	return ret;
}

static int fw_update_by_header_all(struct device *dev)
{
	int ret = ERROR_SUCCESS;
	int all=BOOT_UPDATE_ALL;
	struct sentron_fw_group *head=NULL, *tail = NULL;
	struct sentron_fw_group gSTR_FW, gSTR_MPFPM, gSTR_MPFDM, gSTR_FW_CFG, gSTR_SYS_CFG, gSTR_TMC_REG, gSTR_FPM, gSTR_FDM, gSTR_MUX_CAL;

	gSTR_FW.section = STR_FW;
	gSTR_FW.next = NULL;
	fw_SSD6600_make_link(&head, &tail, &gSTR_FW);

	gSTR_MPFPM.section = STR_MPFPM;
	gSTR_MPFPM.next = NULL;
	fw_SSD6600_make_link(&head, &tail, &gSTR_MPFPM);

	gSTR_MPFDM.section = STR_MPFDM;
	gSTR_MPFDM.next = NULL;
	fw_SSD6600_make_link(&head, &tail, &gSTR_MPFDM);

	gSTR_FPM.section = STR_FPM;
	gSTR_FPM.next = NULL;
	fw_SSD6600_make_link(&head, &tail, &gSTR_FPM);

	gSTR_FDM.section = STR_FDM;
	gSTR_FDM.next = NULL;
	fw_SSD6600_make_link(&head, &tail, &gSTR_FDM);

	gSTR_TMC_REG.section = STR_TMC_REG;
	gSTR_TMC_REG.next = NULL;
	fw_SSD6600_make_link(&head, &tail, &gSTR_TMC_REG);

	gSTR_MUX_CAL.section = STR_DCSW;
	gSTR_MUX_CAL.next = NULL;
	fw_SSD6600_make_link(&head, &tail, &gSTR_MUX_CAL);

	gSTR_FW_CFG.section = STR_FW_CFG;
	gSTR_FW_CFG.next = NULL;
	fw_SSD6600_make_link(&head, &tail, &gSTR_FW_CFG);

	gSTR_SYS_CFG.section = STR_SYS_CFG;
	gSTR_SYS_CFG.next = NULL;
	fw_SSD6600_make_link(&head, &tail, &gSTR_SYS_CFG);

	if( head != NULL ) {
		ret = SSD6600_firmware_update(dev, head, all);
	}

	return ret;
}

static int fw_update_by_header(struct device *dev, unsigned short eflash_flag)
{
	int ret = ERROR_SUCCESS;
	int all=BOOT_UPDATE_EACH;
	struct sentron_fw_group *head=NULL, *tail = NULL;
	struct sentron_fw_group gSTR_FW, gSTR_MPFPM, gSTR_MPFDM,gSTR_FW_CFG, gSTR_SYS_CFG, gSTR_TMC_REG, gSTR_FPM, gSTR_FDM, gSTR_MUX_CAL;

	if( (eflash_flag & FW_EFLASH_FLAG_CPU_ONLY) == FW_EFLASH_FLAG_CPU_ONLY ) {
		gSTR_FW.section = STR_FW;
		gSTR_FW.next = NULL;
		fw_SSD6600_make_link(&head, &tail, &gSTR_FW);
	}
	if( (eflash_flag & FW_EFLASH_FLAG_MP_FPM) == FW_EFLASH_FLAG_MP_FPM	) {
		gSTR_MPFPM.section = STR_MPFPM;
		gSTR_MPFPM.next = NULL;
		fw_SSD6600_make_link(&head, &tail, &gSTR_MPFPM);
	}
	if( (eflash_flag & FW_EFLASH_FLAG_MP_FDM) == FW_EFLASH_FLAG_MP_FDM	) {
		gSTR_MPFDM.section = STR_MPFDM;
		gSTR_MPFDM.next = NULL;
		fw_SSD6600_make_link(&head, &tail, &gSTR_MPFDM);
	}

	if( (eflash_flag & FW_EFLASH_FLAG_FPM) == FW_EFLASH_FLAG_FPM ) {
		gSTR_FPM.section = STR_FPM;
		gSTR_FPM.next = NULL;
		fw_SSD6600_make_link(&head, &tail, &gSTR_FPM);
	}
	if( (eflash_flag & FW_EFLASH_FLAG_FDM) == FW_EFLASH_FLAG_FDM ) {
		gSTR_FDM.section = STR_FDM;
		gSTR_FDM.next = NULL;
		fw_SSD6600_make_link(&head, &tail, &gSTR_FDM);
	}
	if( (eflash_flag & FW_EFLASH_FLAG_TMC_REG) == FW_EFLASH_FLAG_TMC_REG ) {
		gSTR_TMC_REG.section = STR_TMC_REG;
		gSTR_TMC_REG.next = NULL;
		fw_SSD6600_make_link(&head, &tail, &gSTR_TMC_REG);
	}
	if( (eflash_flag & FW_EFLASH_FLAG_SW_CAL) == FW_EFLASH_FLAG_SW_CAL ) {
		gSTR_MUX_CAL.section = STR_DCSW;
		gSTR_MUX_CAL.next = NULL;
		fw_SSD6600_make_link(&head, &tail, &gSTR_MUX_CAL);
	}
	if( (eflash_flag & FW_EFLASH_FLAG_CPU_CFG) == FW_EFLASH_FLAG_CPU_CFG ) {
		gSTR_FW_CFG.section = STR_FW_CFG;
		gSTR_FW_CFG.next = NULL;
		fw_SSD6600_make_link(&head, &tail, &gSTR_FW_CFG);
	}
	if( (eflash_flag & FW_EFLASH_FLAG_SYS_CFG) == FW_EFLASH_FLAG_SYS_CFG ) {
		gSTR_SYS_CFG.section = STR_SYS_CFG;
		gSTR_SYS_CFG.next = NULL;
		fw_SSD6600_make_link(&head, &tail, &gSTR_SYS_CFG);
	}
	if( (eflash_flag & FW_EFLASH_FLAG_INFO) == FW_EFLASH_FLAG_INFO ) {
	}

	if( head != NULL ) {
		ret = SSD6600_firmware_update(dev, head, all);
	}

	return ret;
}


static int SSD6600_firmware_check(struct device *dev, int *updated)
{
	int eDisplayVersion=0, eHiddenVersion=0;
	int hDisplayVersion=0, hHiddenVersino=0;
	int ret=ERROR_SUCCESS;

	if( updated != NULL ) *updated = BOOT_UPDATE_NONE;	//BOOT_UPDATE_OK

	if( (ret=fw_ds_read_version(dev, VERSION_DISPLAY_ADDR, &eDisplayVersion)) < ERROR_SUCCESS ) goto out;	// read eFlash display version
	hDisplayVersion = STR_FW_CFG.content[68]|(STR_FW_CFG.content[69]<<8)|(STR_FW_CFG.content[70]<<16)|(STR_FW_CFG.content[71]<<24);
	TOUCH_E("DISPLAY VERSION  EFLASH : 0x%08x\t0x%08x", eDisplayVersion, hDisplayVersion);
	if( (ret=fw_ds_read_version(dev, VERSION_HIDDEN_ADDR, &eHiddenVersion)) < ERROR_SUCCESS ) goto out;	// read eFlash hidden version
	hHiddenVersino = STR_FW_CFG.content[72]|(STR_FW_CFG.content[73]<<8)|(STR_FW_CFG.content[74]<<16)|(STR_FW_CFG.content[75]<<24);
	TOUCH_E("HIDDEN VERSION  EFLASH : 0x%08x\tHEADER : 0x%08x", eHiddenVersion, hHiddenVersino);

	if( eDisplayVersion == 0xFFFFFFFF && hDisplayVersion != 0xFFFFFFFF ) goto update;

	if( eDisplayVersion < hDisplayVersion ) goto update;

	else if( eDisplayVersion == hDisplayVersion ) {
		if( eHiddenVersion < hHiddenVersino ) {
			goto update;
		} else {
			if( SSD6600_verify_checksum_only(dev) < 0 ) goto update;
		}
	} else {
		if( SSD6600_verify_checksum_only(dev) < 0 ) goto update;
	}
	goto out;
update :
	fw_update_by_header_all(dev);
	if( updated != NULL ) *updated = BOOT_UPDATE_OK;
out : 
	return ret;
}

static int SSD6600_firmware_version_check(struct device *dev, int all, int *upgraded, int *verify_flag, int *checksum_flag)
{
	int ret=ERROR_SUCCESS;
	int ver=0x00;
	int len=0, i=0;
	unsigned int update_flag=0x00;

	if( verify_flag != NULL ) *verify_flag = 0x00;

	if( checksum_flag != NULL ) *checksum_flag = 0x00;

	if( upgraded != NULL ) *upgraded = BOOT_UPDATE_NONE;	//BOOT_UPDATE_OK

	if( all == BOOT_UPDATE_ALL ) {
		update_flag = FW_EFLASH_FLAG_ALL;
	} else {
		len = sizeof( m_fw_version ) / sizeof(struct sentron_fw_version);

		TOUCH_E("ARRAY SIZE : %d \t STRUCT SIZE : %d \t LENGTH : %d", (int) sizeof( m_fw_version ), (int) sizeof(struct sentron_fw_version), len );
		for( i=3; i<len; i++ ) {
			if( m_fw_version[i].section == NULL ) continue;

			if( m_fw_version[i].section->content == NULL || m_fw_version[i].section->byte_cnt == 0 ) continue;

			if( (ret=fw_ds_read_version(dev, m_fw_version[i].version_address, &ver)) < ERROR_SUCCESS ) break;
			TOUCH_E("address : 0x%08x \t H VERSION : 0x%08x \t R VERSION : 0x%08x", m_fw_version[i].version_address, m_fw_version[i].section->version, ver );
			if( m_fw_version[i].section->version > ver ) {
				update_flag |= m_fw_version[i].update_flag;
			} else if( m_fw_version[i].section->version == ver ) {
				if( verify_flag != NULL ) *verify_flag |= m_fw_version[i].verify_flag;
			} else {
				if( checksum_flag != NULL ) *checksum_flag |= m_fw_version[i].verify_flag;
			}
TOUCH_E("update flag : 0x%08x \t verify_flag : 0x%08x \t checksum_falgh : 0x%08x", update_flag, *verify_flag, *checksum_flag);
		}
	}

	if( ret >= ERROR_SUCCESS && update_flag > 0x00 ) {
		ret = fw_update_by_header(dev, update_flag);
		if( upgraded != NULL ) *upgraded = BOOT_UPDATE_OK;	//
	}

TOUCH_E("update flag : 0x%08x \t verify_flag : 0x%08x", update_flag, *verify_flag);
	return ret;
}

static int Sentron_boot_up_check(struct device *dev, int *updated)
{
	u16 rd = 0;
	int ret = 0;

	if( updated != NULL ) *updated = BOOT_UPDATE_NONE;

	if( ds_read_boot_st(dev, (u16 *)&rd) < 0 ) return -1;
	TOUCH_E(">>>>> read boot st read(2) : 0x%04x", rd );
	if( (rd & BOOT_STATUS_ERR_INF_ALL) > 0 ) {
		TOUCH_E(">>>>> Sentron DS16 INF error routine start!!");
		SSD6600_Boot_inf_check(dev, rd);
		TOUCH_E(">>>>> Sentron DS16S INF error routine end!!");
		if( updated != NULL ) *updated = BOOT_UPDATE_OK;
	}

	if( (rd & BOOT_STATUS_ERR_CPUCFG_ALL) > 0 ) {
		TOUCH_E(">>>>> Sentron DS16 CPU CFG error routine start!!");
		fw_update_by_header(dev, (FW_EFLASH_FLAG_CPU_ONLY|FW_EFLASH_FLAG_CPU_CFG|FW_EFLASH_FLAG_SYS_CFG));	// booting time;
		TOUCH_E(">>>>> Sentron DS16 CPU CFG error routine end!!");
		if( updated != NULL ) *updated = BOOT_UPDATE_OK;
	}

	if( (rd & BOOT_STATUS_ERR_SYS_CFG_FAIL) > 0 ) {
		TOUCH_E(">>>>> Sentron DS16 SYS CFG error routine start!!");
		SSD6600_Boot_sys_cfg_check(dev);
		TOUCH_E(">>>>> Sentron DS16S SYS CFG error routine end!!");
		if( updated != NULL ) *updated = BOOT_UPDATE_OK;
	}
	
TOUCH_E(">>>>> END <<<<<");
	return ret;
}

static int SSD6600_verify_checksum_only(struct device *dev)
{
	int ret=ERROR_SUCCESS;
	int i=0, len=0;

	TOUCH_E("START VERIFY CHECKSUM ONLY!!");

	len = sizeof(m_fw_version) / sizeof(struct sentron_fw_version)-1;

	TOUCH_E("ARRAY SIZE : %d \t STRUCT SIZE : %d \t LENGTH : %d", (int) sizeof( m_fw_version ), (int) sizeof(struct sentron_fw_version), len );
	for( i=3; i<len; i++ ) {
		if( m_fw_version[i].section == NULL ) continue;

		if( m_fw_version[i].section->content == NULL || m_fw_version[i].section->byte_cnt == 0 ) continue;

		if( m_fw_version[i].verify_flag == 0x00 ) continue;
		
		if( (ret=fw_SSD6600_verify_checksum(dev, m_fw_version[i].section->address)) < 0 ) break;
	}

	TOUCH_E("END VERIFY CHECK ONLY!!");
	return ret;
}

static struct sentron_fw_group *parsHexFile2IntArr(char *filename, int *errnum, int *all)
{
        struct file *src;
        mm_segment_t oldfs;
        struct sentron_fw_group *head=NULL, *tail=NULL, *ptr=NULL;
        unsigned char buff[128] = {0,};
        int buff_idx = 0;
        int ret=ERROR_SUCCESS;
        unsigned char ch;
        int bComment = 0;
        VAL_LEVEL level = LEVEL_NONE;
        VAL_DOLLAR_LEVEL dollar_level = LEVEL_DOLLAR_NONE;
        int iRet = 0;
       
        if( filename == NULL ) {
                ret = ERROR_PARSING_FILENAME_IS_NULL;
                goto out;
        }
       
        oldfs = get_fs();
        set_fs(KERNEL_DS);

        src = filp_open(filename, O_RDONLY, S_IRUSR|S_IRGRP|S_IROTH);
        if(IS_ERR(src)){
                TOUCH_E("[%s] file open error!!", filename);
                ret = ERROR_PARSING_FILE_OPEN_FAIL;
        		set_fs(oldfs);
                goto out;
        }
       
        do {
                memset( buff, 0, sizeof(buff) );
                buff_idx = 0;
                bComment = 0;
          
                do {
                        ret = vfs_read( src, &ch, 1, &src->f_pos);
               
                        if( ret > 0 ) {
                                if( ch == '\n' ) break;
                                if( ch == '\r' || ch == '\t' || ch == ' ' || ch == 0x09 ) {}
                                else if(  ch == ';' ) bComment = 1;
                                else if( bComment == 0 ) buff[buff_idx++] = ch;
                        }
                } while( ret > 0 && buff_idx < 128 );
           
                if( ret < 0 ) { ret = ERROR_PARSING_FORMAT_INVALID; break; }
                if( ret == 0 ) break;
       
                if( strlen(buff) < 1 ) continue;

                if( buff[0] == '$' ) {
                        if( dollar_level == LEVEL_DOLLAR_NONE ) {
                                if( (ret = convHexStr2Int( (buff+1), 4, &iRet )) < ERROR_SUCCESS ) {break;}
                                *all = iRet;
                                TOUCH_E(">>>>> ALL Update flag : %d", *all);
                                dollar_level = LEVEL_DOLLAR_ERASE_TYPE;
                        }
                        continue;
                }
           
                if( buff[0] == '#' ) {
                        if( strlen(buff+1) < 1 ) { ret = ERROR_PARSING_HEADER_DATA_INVALID_LENGTH; break; }

                        if( level != LEVEL_NONE ) {
                                ret = ERROR_PARSING_FORMAT_INVALID;
                                break;
                        }

                        ptr = (struct sentron_fw_group *)kmalloc( sizeof(struct sentron_fw_group), GFP_KERNEL );
                
                        if( ptr == NULL ) {
                                ret = ERROR_PARSING_MALLOC_FAIL;
                                break;
                        }
                
                        memset( ptr, 0x00, sizeof(struct sentron_fw_group) );
                
                        if( (ret = convHexStr2Int( (buff+1), strlen(buff)-1, &iRet )) < ERROR_SUCCESS ) {break;}
                
                        ptr->section.address = iRet;
                        TOUCH_E(">>> address : 0x%08x\n", ptr->section.address);
                        if( head == NULL ) head = tail = ptr;
                        else {
                                tail->next = ptr;
                                tail = ptr;
                        }
                
                        level = LEVEL_ADDRESS;
                } else if( buff[0] == '*' ) {
                        if( strlen(buff+1) < 1 ) { ret = ERROR_PARSING_HEADER_DATA_INVALID_LENGTH; break; }
                        if( level < LEVEL_ADDRESS ) { ret = ERROR_PARSING_FORMAT_INVALID; break; }
                        if( (ret = convHexStr2Int( (buff+1), strlen(buff)-1, &iRet )) < ERROR_SUCCESS ) {break;}
                        if( level == LEVEL_ADDRESS ) {
                                if( iRet < 1 ) { ret = ERROR_PARSING_DATA_CNT_FAIL; break; }
                                ptr->section.byte_cnt = iRet;
                                TOUCH_E(">>> byte_cnt : 0x%08x\n", ptr->section.byte_cnt);
                                level = LEVEL_LENGTH;
                        } else if( level == LEVEL_LENGTH ) {
                                if( iRet < 1 ) { ret = ERROR_PARSING_DATA_CNT_FAIL; break; }
                                ptr->section.erase_page_cnt = iRet;
                                TOUCH_E(">>> erase_page_cnt : 0x%08x\n", ptr->section.erase_page_cnt);
                                level = LEVEL_ERASE_SIZE;
                        } else if( level == LEVEL_ERASE_SIZE ) {
                                ptr->section.version = iRet;
                                TOUCH_E(">>> version : 0x%08x\n", ptr->section.version);
                                level = LEVEL_VERSION;
                        } else if( level == LEVEL_VERSION ) {
                                ptr->section.checksum = (unsigned int)iRet;
                                TOUCH_E(">>> checksum : 0x%08x\n", ptr->section.checksum);
                                level = LEVEL_CHECKSUM;
                        } else if( level == LEVEL_CHECKSUM ) {
                                ptr->section.reserved_01 = iRet;
                                TOUCH_E(">>> reserved_01 : 0x%08x\n", ptr->section.reserved_01);
                                level = LEVEL_RESERVE_01;
                        } else if( level == LEVEL_RESERVE_01 ) {
                                ptr->section.reserved_02 = iRet;
                                TOUCH_E(">>> reserved_02 : 0x%08x\n", ptr->section.reserved_03);
                                level = LEVEL_RESERVE_02;
                        } else if( level == LEVEL_RESERVE_02 ) {
                                ptr->section.reserved_03 = iRet;
                                TOUCH_E(">>> reserved_03 : 0x%08x\n", ptr->section.reserved_03);
                                ptr->section.content = (unsigned char *)kmalloc(ptr->section.byte_cnt, GFP_KERNEL);

                                if( ptr->section.content == NULL ) { ret = ERROR_PARSING_MALLOC_FAIL; break; }

                                ret = vfs_read( src, ptr->section.content, ptr->section.byte_cnt, &src->f_pos );

                                TOUCH_E(" SIZE %d %d\n", ptr->section.byte_cnt, ret );
                                if( ret != ptr->section.byte_cnt ) { ret = ERROR_PARSING_CONTENT_SIZE_FAIL; break; }

                                if( (ret=fw_checksum(ptr->section)) < ERROR_SUCCESS ) { break; }
                                level = LEVEL_NONE;
                        }
                } else {
                        ret = ERROR_PARSING_FORMAT_INVALID;
                }
           
        } while(ret >= 0);

        filp_close(src, NULL);
        set_fs(oldfs);
out :
        *errnum = ret;

        if( ret < 0 ) { 
                fw_free(head, 1);
                head = NULL;
        }
        return head; 
}

static struct sentron_fw_group *parsHexFWarr2IntArr(const struct firmware *fw, int *errnum, int *all)
{
        struct sentron_fw_group *head=NULL, *tail=NULL, *ptr=NULL;
        unsigned char buff[128] = {0,};
        int buff_idx = 0;
        int ret=ERROR_SUCCESS;
        unsigned char ch;
        int bComment = 0;
        VAL_LEVEL level = LEVEL_NONE;
        VAL_DOLLAR_LEVEL dollar_level = LEVEL_DOLLAR_NONE;
        int iRet = 0;
        int dollar_count = 0;
        const u8 *data = fw->data;
        long dataSize = fw->size;
        long dataIdx = 0;
       
        do {     
                TOUCH_I("[sentron] %ld %ld\n", dataIdx, dataSize);
                if( dataIdx > dataSize ) { ret = ERROR_PARSING_FORMAT_INVALID; break; }
                if( dataIdx == dataSize ) break;
        	
                memset( buff, 0, sizeof(buff) );
                buff_idx = 0;
                bComment = 0;
	       
                do {
                        ch = *data;
                        dataIdx++;
                        data++;

                        if( ch == '\n' ) break;
                        if( ch == '\r' || ch == '\t' || ch == ' ' || ch == 0x09 ) {}
                        else if(  ch == ';' ) bComment = 1;
                        else if( bComment == 0 ) buff[buff_idx++] = ch;
                } while( dataIdx < dataSize && buff_idx < 128 );

                if( strlen(buff) < 1 ) continue;

                TOUCH_I("[sentron] buff : %s\n", buff);
                if( buff[0] == '$' ) {
                        dollar_count++;
                        if( dollar_count == 9 && dollar_level == LEVEL_DOLLAR_NONE ) {
                                if( (ret = convHexStr2Int( (buff+1), 4, &iRet )) < ERROR_SUCCESS ) {break;}
                                *all = iRet;
                                TOUCH_I(">>>>> ALL Update flag : %d", *all);
                                dollar_level = LEVEL_DOLLAR_ERASE_TYPE;
                        }
                        continue;
                }
           
                if( buff[0] == '#' ) {
                        if( strlen(buff+1) < 1 ) { ret = ERROR_PARSING_HEADER_DATA_INVALID_LENGTH; break; }

                        if( level != LEVEL_NONE ) {
                                ret = ERROR_PARSING_FORMAT_INVALID;
                                break;
                        }

                        ptr = (struct sentron_fw_group *)kmalloc( sizeof(struct sentron_fw_group), GFP_KERNEL );

                        if( ptr == NULL ) {
                                ret = ERROR_PARSING_MALLOC_FAIL;
                                break;
                        }
                
                        memset( ptr, 0x00, sizeof(struct sentron_fw_group) );

                        if( (ret = convHexStr2Int( (buff+1), strlen(buff)-1, &iRet )) < ERROR_SUCCESS ) {break;}

                        ptr->section.address = iRet;
                        TOUCH_I(">>> address : 0x%08x\n", ptr->section.address);
                        if( head == NULL ) head = tail = ptr;
                        else {
                                tail->next = ptr;
                                tail = ptr;
                        }

                        level = LEVEL_ADDRESS;
                } else if( buff[0] == '*' ) {
                        if( strlen(buff+1) < 1 ) { ret = ERROR_PARSING_HEADER_DATA_INVALID_LENGTH; break; }

                        if( level < LEVEL_ADDRESS ) { ret = ERROR_PARSING_FORMAT_INVALID; break; }

                        if( (ret = convHexStr2Int( (buff+1), strlen(buff)-1, &iRet )) < ERROR_SUCCESS ) {break;}
                  
                        if( level == LEVEL_ADDRESS ) {
                                if( iRet < 1 ) { ret = ERROR_PARSING_DATA_CNT_FAIL; break; }
                                ptr->section.byte_cnt = iRet;
                                TOUCH_I(">>> byte_cnt : 0x%08x\n", ptr->section.byte_cnt);
                                level = LEVEL_LENGTH;
                        } else if( level == LEVEL_LENGTH ) {
                                if( iRet < 1 ) { ret = ERROR_PARSING_DATA_CNT_FAIL; break; }
                                ptr->section.erase_page_cnt = iRet;
                                TOUCH_I(">>> erase_page_cnt : 0x%08x\n", ptr->section.erase_page_cnt);
                                level = LEVEL_ERASE_SIZE;
                        } else if( level == LEVEL_ERASE_SIZE ) {
                                ptr->section.version = iRet;
                                TOUCH_I(">>> version : 0x%08x\n", ptr->section.version);
                                level = LEVEL_VERSION;
                        } else if( level == LEVEL_VERSION ) {
                                ptr->section.checksum = (unsigned int)iRet;
                                TOUCH_I(">>> checksum : 0x%08x\n", ptr->section.checksum);
                                level = LEVEL_CHECKSUM;
                        } else if( level == LEVEL_CHECKSUM ) {
                                ptr->section.reserved_01 = iRet;
                                TOUCH_I(">>> reserved_01 : 0x%08x\n", ptr->section.reserved_01);
                                level = LEVEL_RESERVE_01;
                        } else if( level == LEVEL_RESERVE_01 ) {
                                ptr->section.reserved_02 = iRet;
                                TOUCH_I(">>> reserved_02 : 0x%08x\n", ptr->section.reserved_03);
                                level = LEVEL_RESERVE_02;
                        } else if( level == LEVEL_RESERVE_02 ) {
                                ptr->section.reserved_03 = iRet;
                                TOUCH_I(">>> reserved_03 : 0x%08x\n", ptr->section.reserved_03);
                                TOUCH_I(" SIZE %d %ld\n", ptr->section.byte_cnt, dataIdx );
                                if( ptr->section.byte_cnt > (dataSize-dataIdx) ) { ret = ERROR_PARSING_CONTENT_SIZE_FAIL; break; }

                                ptr->section.content = (unsigned char *)kmalloc(ptr->section.byte_cnt, GFP_KERNEL);

                                if( ptr->section.content == NULL ) { ret = ERROR_PARSING_MALLOC_FAIL; break; }

                                memcpy( ptr->section.content, data, ptr->section.byte_cnt );

                                dataIdx += ptr->section.byte_cnt;
                                data += ptr->section.byte_cnt;

                                if( (ret=fw_checksum(ptr->section)) < ERROR_SUCCESS ) { break; }
                                        level = LEVEL_NONE;
                       }
                } else {
                        ret = ERROR_PARSING_FORMAT_INVALID;
                }
        } while(ret >= 0);

        *errnum = ret;
    
        if( ret < 0 ) { 
                fw_free(head, 1);
                head = NULL;
        }
        return head; 
}

int SSD6600_firmware_update_byfile(struct device *dev, char *filename)
{
	int errnum = 0;
	int retry=FW_MAX_RETRY_COUNT;
	struct sentron_fw_group *head=NULL;
	int all = BOOT_UPDATE_EACH;

	head = parsHexFile2IntArr(filename, &errnum, &all);
	if( head != NULL ) {
		sentron_reset(dev);

		do {
			if( (errnum = fw_SSD6600_init(dev)) >= 0 ) break;
			mdelay(10);
		} while( (retry--) > 1 );

		errnum = SSD6600_firmware_update(dev, head, all);
		fw_ds_read_version_all(dev);
	}

	return errnum;
}

int SSD6600_firmware_update_byArr(struct device *dev, const struct firmware *fw)
{
	int errnum = 0;
	int retry=FW_MAX_RETRY_COUNT;
	struct sentron_fw_group *head=NULL;
	int all = BOOT_UPDATE_EACH;

	head = parsHexFWarr2IntArr(fw, &errnum, &all);
	if( head != NULL ) {
		sentron_reset(dev);

		do {
			if( (errnum = fw_SSD6600_init(dev)) >= 0 ) break;
			mdelay(10);
		} while( (retry--) > 1 );

		errnum = SSD6600_firmware_update(dev, head, all);
	    fw_ds_read_info_by_fw(dev);
    }

	return errnum;
}

static int Sentron_firmware_boot_up_no_bin(struct device *dev)
{
	int retry=FW_MAX_RETRY_COUNT;
	sentron_reset(dev);

	do {
		if( fw_SSD6600_init(dev) >= 0 ) break;
		mdelay(10);
	} while( (retry--) > 0 );
	
	if( fw_ds_read_info_by_fw(dev) < 0 ) {
		TOUCH_E("F/W info read error!!");
		goto out;
	}
	
	return 0;
out :
	return -1;
}

static int Sentron_firmware_boot_up(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sentron_device *ftdev = (struct sentron_device *)touch_get_device(ts);
	int retry=FW_MAX_RETRY_COUNT;
	u32 fwinfo, bininfo;
	int errnum = 0;
        struct sentron_fw_group *head=NULL;
        int all = BOOT_UPDATE_EACH;
        u16 rd = 0;

        TOUCH_I(">>>>> START <<<<<");

	sentron_reset(dev);

	do {
		if( fw_SSD6600_init(dev) >= 0 ) break;
		mdelay(10);
	} while( (retry--) > 0 );
	
	if( retry <= 0 ) goto out;
		
	if( fw_ds_read_info_by_fw(dev) < 0 ) {
		TOUCH_E("F/W info read error!!");
		goto out;
	}
	if( fw_ds_read_info_by_bin(dev, fw) < 0 ) {
		TOUCH_E("BIN info read error!!");
		goto out;
	}

        // ADD FW AUTO
        if( ftdev->fw_info->version1[0] == 0xFF && ftdev->fw_info->version1[1] == 0xFF && ftdev->fw_info->version1[2] == 0xFF && ftdev->fw_info->version1[3] == 0xFF ) goto upgrade;

        // IC Name check
        if( (memcmp(ftdev->fw_info->ICName1, ftdev->bin_info->ICName1, 4) != 0) || (memcmp(ftdev->fw_info->ICName2, ftdev->bin_info->ICName2, 4) != 0) ) {
        TOUCH_E("IC Name is not equal. F/W IC Name : %s%s \t BIN IC Name : %s%s\n", ftdev->fw_info->ICName1, ftdev->fw_info->ICName2, ftdev->bin_info->ICName1, ftdev->bin_info->ICName2);
                goto out;
        }
        // Product ID check
        if( (memcmp(ftdev->fw_info->prdID1, ftdev->bin_info->prdID1, 4) != 0) || (memcmp(ftdev->fw_info->prdID2, ftdev->bin_info->prdID2, 4) != 0) ) {
        TOUCH_E("Product ID is not equal. F/W Product ID : %s%s \t BIN Product ID : %s%s\n", ftdev->fw_info->prdID1, ftdev->fw_info->prdID2, ftdev->bin_info->prdID1, ftdev->bin_info->prdID2);
                goto out;
        }

	TOUCH_I("F/W version 1 = 0x%02x%02x%02x%02x", ftdev->fw_info->version1[3], ftdev->fw_info->version1[2], ftdev->fw_info->version1[1], ftdev->fw_info->version1[0] );
	TOUCH_I("BIN version 1 = 0x%02x%02x%02x%02x", ftdev->bin_info->version1[3], ftdev->bin_info->version1[2], ftdev->bin_info->version1[1], ftdev->bin_info->version1[0] );
	fwinfo = (ftdev->fw_info->version1[1]) << 8 |(ftdev->fw_info->version1[0]);
	bininfo = (ftdev->bin_info->version1[1]) << 8 |(ftdev->bin_info->version1[0]);
	
	if( fwinfo < bininfo ) {
		goto upgrade;
	} else {
		TOUCH_I("F/W version 2 = 0x%02x%02x%02x%02x", ftdev->fw_info->version2[3], ftdev->fw_info->version2[2], ftdev->fw_info->version2[1], ftdev->fw_info->version2[0] );
		TOUCH_I("BIN version 2 = 0x%02x%02x%02x%02x", ftdev->bin_info->version2[3], ftdev->bin_info->version2[2], ftdev->bin_info->version2[1], ftdev->bin_info->version2[0] );
		fwinfo = (ftdev->fw_info->version2[1]) << 8 |(ftdev->fw_info->version2[0]);
		bininfo = (ftdev->bin_info->version2[1]) << 8 |(ftdev->bin_info->version2[0]);
		
		if(  fwinfo < bininfo ) {
			goto upgrade;
		}
		
		if( ds_read_boot_st(dev, (u16 *)&rd) < 0 ) return -1;
		
		if( (rd & BOOT_STATUS_ERR_CPUCFG_ALL) > 0 ) {
			TOUCH_I(">>>>> Sentron DS16 CPU CFG Checksum fail!!");
			goto upgrade;
		}
	
		if( (rd & BOOT_STATUS_ERR_SYS_CFG_FAIL) > 0 ) {
			TOUCH_I(">>>>> Sentron DS16 SYS CFG Checksum fail!!");
			goto upgrade;
		}
		if( SSD6600_verify_checksum_only(dev) < 0 ) {
			TOUCH_I(">>>>>> Sentron Else reg section Checksum fail!!");
			goto upgrade;
		}
		
		TOUCH_I(">>>>> read boot st read(2) : 0x%04x", rd );
		if( (rd & BOOT_STATUS_ERR_INF_ALL) > 0 ) {
			TOUCH_I(">>>>> Sentron DS16 INF tunning routine start!!");
			SSD6600_Boot_inf_check(dev, rd);
		}
	}
	
	return 0;
upgrade :
    head = parsHexFWarr2IntArr(fw, &errnum, &all);
    if( head != NULL ) {
	    errnum = SSD6600_firmware_update(dev, head, all); // firmware upate
    }
out :
	fw_ds_read_info_by_fw(dev);
	return errnum;
}

int SSD6600_firmware_pre_boot_up_check(struct device *dev, const struct firmware *fw)
{
    int ret=0;

    if (fw == NULL) {
        TOUCH_E("error get fw NULL\n");
        goto no_bin;
    }

    ret = Sentron_firmware_boot_up(dev, fw);

    return ret;

no_bin :
    return Sentron_firmware_boot_up_no_bin(dev);
}

u8* SSD6600_get_version(void)
{
	return m_all_version;
}

int SSD6600_get_display_version(void)
{
    if (0) {
        SSD6600_firmware_check(0, 0);
        SSD6600_firmware_version_check(0, 0, 0, 0, 0);
        Sentron_boot_up_check(0, 0);
    }
	return m_display_version;
}

int SSD6600_get_hidden_version(void)
{
	return m_hidden_version;
}

int SSD6600_get_fw_info(struct device *dev)
{
        return fw_ds_read_info_by_fw(dev);
}

int SSD6600_get_boot_fw_info(struct device *dev)
{
        int retry=FW_MAX_RETRY_COUNT;

        sentron_reset(dev);

        do {
                if( fw_SSD6600_init(dev) >= 0 ) break;
                mdelay(10);
        } while( (retry--) > 0 );

        if( retry <= 0 ) goto out;

        if( fw_ds_read_info_by_fw(dev) < 0 ) {
                TOUCH_E("F/W info boot read error!!");
                goto out;
        }
        return 0;
out:
        return -1;
}
