/* sharp_fw_upgrade.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author : keunyoung1.park@lge.com
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_lr388k6.h"

static int sharp_flash_access_start(struct device *dev)
{
	unsigned char value;

	/* Mask Everything */
	value = 0xFF;
	sharp_write(dev, SHTSC_ADDR_INTMASK0, &value, sizeof(value));
	value = 0x03;
	sharp_write(dev, SHTSC_ADDR_INTMASK1, &value, sizeof(value));

	touch_msleep(100);

	/* TRIM_OSC = 0 */
	value = 0x18;
	sharp_write(dev, 0x02, &value, sizeof(value));
	value = 0x19;
	sharp_write(dev, 0x11, &value, sizeof(value));

	return 0;
}

static int sharp_flash_erase_page(struct device *dev, int page)
{
	unsigned char r_data;
	unsigned char value;
	int retry = 0;

	/* Banc Change */
	value = 0x1C;
	sharp_write(dev, 0x02, &value, sizeof(value));

	/* CLKON_CTL_0 */
	value = 0x22;
	sharp_write(dev, 0x14, &value, sizeof(value));

	/* FLC_CTL CS_HIGH, WP_DISABLE */
	value = 0x14;
	sharp_write(dev, 0x3C, &value, sizeof(value));

	/* FLC_CTL CS_LOW, WP_DISABLE */
	value = 0x16;
	sharp_write(dev, 0x3C, &value, sizeof(value));

	/* FLC_Tx_DATA WRITE_ENABLE */
	value = FLASH_CMD_WRITE_EN;
	sharp_write(dev, 0x3D, &value, sizeof(value));

	/* FLC_CTL CS_HIGH, WP_DISABLE */
	value = 0x14;
	sharp_write(dev, 0x3C, &value, sizeof(value));

	/* FLC_CTL CS_LOW, WP_DISABLE */
	value = 0x16;
	sharp_write(dev, 0x3C, &value, sizeof(value));

	/* FLC_Tx_DATA CHIP_ERASE_COMMAND */
	/* not a chip erase, but a sector erase for the backward compatibility!! */
	value = FLASH_CMD_SECTOR_ERASE;
	sharp_write(dev, 0x3D, &value, sizeof(value));

 	/* 24bit address. 4kByte=001000H. 00x000H:x=0-f -> 4kB*16=64kB */
	/* for K6 or more than 64kB, 0(0x page)000 : page(00H-1FH) for 128kB */
	value = (page >> 4) & 0xFF;
	sharp_write(dev, 0x3D, &value, sizeof(value));
	value = (page << 4) & 0xFF;
	sharp_write(dev, 0x3D, &value, sizeof(value));
	value = 0x00;
	sharp_write(dev, 0x3D, &value, sizeof(value));

	/* FLC_CTL CS_HIGH, WP_DISABLE */
	value = 0x14;
	sharp_write(dev, 0x3C, &value, sizeof(value));

	/* wait until 'BUSY == LOW' */
	do {
		retry++;

		if (retry > RETRY_COUNT) {
			TOUCH_E("flash_erase_page retry %d times for page %d - FAILED!\n",
					retry, page);
			return 1;
		}

		/* FLC_CTL CS_LOW, WP_DISABLE */
		value = 0x16;
		sharp_write(dev, 0x3C, &value, sizeof(value));

		/* FLC_TxDATA READ_STATUS_COMMAND */
		value = FLASH_CMD_READ_ST;
		sharp_write(dev, 0x3D, &value, sizeof(value));

		/* Dummy Data */
		value = 0x00;
		sharp_write(dev, 0x3D, &value, sizeof(value));

		/* FLC_RxDATA */
		sharp_read(dev, 0x3F, &r_data, sizeof(r_data));

		/* FLC_CTL CS_HIGH, WP_DISABLE */
		value = 0x14;
		sharp_write(dev, 0x3C, &value, sizeof(value));
	} while(r_data & FLASH_ST_BUSY);	/* Check busy bit */

	return 0;
}

static int sharp_flash_write_page(struct device *dev, int page, unsigned char *image)
{
	unsigned char r_data;
	unsigned char value;
	unsigned page_addr;	/* Address(32 or 64KB area) */
	int retry = 0;
	int i, idx;
	char temp_buf[512];

	/* Bank Change */
	value = 0x1C;
	sharp_write(dev, 0x02, &value, sizeof(value));

	/* CLKON_CTL_0 */
	value = 0x22;
	sharp_write(dev, 0x14, &value, sizeof(value));

	/* FLC_CTL CS_HIGH, WP_DISABLE */
	value = 0x14;
	sharp_write(dev, 0x3C, &value, sizeof(value));

	/* 256 bytes / Flash write page, 4kByte / logical(virtual) flash page */
	for (i = 0 ; i < (FLASH_PAGE_SIZE / FLASH_PHYSICAL_PAGE_SIZE) ; i++) {
		/* 4k page offset + in-page offset. 4k * n + 256 * m */
		page_addr = (page * FLASH_PAGE_SIZE)
				+ (i * FLASH_PHYSICAL_PAGE_SIZE);

		/* FLC_CTL CS_LOW, WP_DISABLE */
		value = 0x16;
		sharp_write(dev, 0x3C, &value, sizeof(value));

		/* FLC_Tx_DATA WRITE_ENABLE */
		value = FLASH_CMD_WRITE_EN;
		sharp_write(dev, 0x3D, &value, sizeof(value));
		
		/* FLC_CTL CS_HIGH, WP_DISABLE */
		value = 0x14;
		sharp_write(dev, 0x3C, &value, sizeof(value));
		
		/* FLC_CTL CS_LOW, WP_DISABLE */
		value = 0x16;
		sharp_write(dev, 0x3C, &value, sizeof(value));
		
		/* FLC_Tx_DATA PAGE_PROGRAM_COMMAND */
		value = FLASH_CMD_PAGE_WR;
		sharp_write(dev, 0x3D, &value, sizeof(value));
		
		/* FLC_Tx_DATA Address(bit16~23) */
		value = (page_addr >> 16) & 0xFF;
		sharp_write(dev, 0x3D, &value, sizeof(value));

		/* FLC_Tx_DATA Address(bit8~15) */
		value = (page_addr >> 8) & 0xFF;
		sharp_write(dev, 0x3D, &value, sizeof(value));

		/* FLC_Tx_DATA Address(bit0~7) */
		value = page_addr & 0xFF;
		sharp_write(dev, 0x3D, &value, sizeof(value));

		/* Random access mode to speed up */
		{
			for (idx = 0 ; idx < 256 ; idx++) {
				temp_buf[idx * 2] = 0x80 | 0x3D;
				temp_buf[idx * 2 + 1] = image[i * FLASH_PHYSICAL_PAGE_SIZE + idx];
			}
			sharp_write(dev, temp_buf[0], &(temp_buf[1]), 511);
		}

		/* FLC_CTL CS_HIGH, WP_DISABLE */
		value = 0x14;
		sharp_write(dev, 0x3C, &value, sizeof(value));

		/* wait until 'BUSY == LOW' */
		do {
			retry++;

			if (retry > RETRY_COUNT) {
				TOUCH_E("flash_write_page retry %d times for page %d, addr %04X - FAILED!\n",
						retry, page, page_addr);
				return 1;
			}

			/* FLC_CTL CS_LOW, WP_DISABLE */
			value = 0x16;
			sharp_write(dev, 0x3C, &value, sizeof(value));

			/* FLC_TxDATA READ_STATUS_COMMAND */
			value = FLASH_CMD_READ_ST;
			sharp_write(dev, 0x3D, &value, sizeof(value));

			/* Dummy Data */
			value = 0x00;
			sharp_write(dev, 0x3D, &value, sizeof(value));

			/* FLC_RxDATA */
			sharp_read(dev, 0x3F, &r_data, sizeof(r_data));

			/* FLC_CTL CS_HIGH, WP_DISABLE */
			value = 0x14;
			sharp_write(dev, 0x3C, &value, sizeof(value));
		} while(r_data & FLASH_ST_BUSY);	/* Check busy bit */
	}

	return 0;
}

static int sharp_flash_verify_page(struct device *dev, int page, unsigned char *image)
{
	struct sharp_data *d = to_sharp_data(dev);

	unsigned char value;
	unsigned page_addr;
	int i;

	/* Banc Change */
	value = 0x1C;
	sharp_write(dev, 0x02, &value, sizeof(value));

	/* CLKON_CTL_0 */
	value = 0x22;
	sharp_write(dev, 0x14, &value, sizeof(value));

	/* FLC_CTL CS_HIGH, WP_ENABLE */
	value = 0x10;
	sharp_write(dev, 0x3C, &value, sizeof(value));

	/* FLC_CTL CS_LOW, WP_ENABLE */
	value = 0x12;
	sharp_write(dev, 0x3C, &value, sizeof(value));

	/* FLC_Tx_DATA READ_COMMAND */
	value = FLASH_CMD_READ;
	sharp_write(dev, 0x3D, &value, sizeof(value));

	/* FLC_Tx_DATA Address(bit16~23) */
	value = (page >> 4) & 0xFF;
	sharp_write(dev, 0x3D, &value, sizeof(value));

	/* FLC_Tx_DATA Address(bit8~15) */
	value = (page << 4) & 0xFF;
	sharp_write(dev, 0x3D, &value, sizeof(value));

	/* FLC_Tx_DATA Address(bit0~7) */
	value = 0x00;
	sharp_write(dev, 0x3D, &value, sizeof(value));

	/* Dummy Data */
	value = 0x00;
	sharp_write(dev, 0x3D, &value, sizeof(value));

	for (page_addr = 0 ; page_addr < FLASH_PAGE_SIZE ; page_addr += FLASH_VERIFY_SIZE) {
		for (i = 0 ; i < FLASH_VERIFY_SIZE ; i++) {
			/* FLC_RxDATA */
			sharp_read(dev, 0x3F, &(d->verify_buf[i]), sizeof(d->verify_buf[i]));
		}

		if (memcmp((unsigned char *)&(image[page_addr]),
				(unsigned char *)d->verify_buf, FLASH_VERIFY_SIZE)) {
			/* FLC_CTL CS_HIGH, WP_ENABLE */
			value = 0x10;
			sharp_write(dev, 0x3C, &value, sizeof(value));
			TOUCH_E("flash_verify_page %d - FAILED!\n", page);
			return 1;
		}
	}

	/* FLC_CTL CS_HIGH, WP_ENABLE */
	value = 0x10;
	sharp_write(dev, 0x3C, &value, sizeof(value));

	return 0;
}

static int sharp_cmd_system_state_sleep(struct device *dev)
{
	u8 value = 0;

	/* Set Bank */
	sharp_set_bank(dev, SHTSC_BANK_COMMAND);

	/* Set Command */
	sharp_write(dev, SHTSC_ADDR_COMMAND, CMD_SETSYSTEMSTATE_SLEEP, CMD_GETPROPERTY_LEN);

	/* Set Incdicator */
	sharp_set_indicator(dev, SHTSC_IND_CMD);

	sharp_read(dev, SHTSC_ADDR_INT0, &value, sizeof(value));
	TOUCH_I("Value check value %d, SHTSC_STATUS_COMMAND_RESULT & value %d \n",
			value, (SHTSC_STATUS_COMMAND_RESULT & value));

	/* Clear Interrupt */
	sharp_clear_interrupt(dev, SHTSC_STATUS_COMMAND_RESULT);

	return 0;
}

static int sharp_update_flash(struct device *dev, unsigned char *image, unsigned int size)
{
	int page;

	TOUCH_I("%s start\n", __func__);
	TOUCH_I("f/w updating......\n");

	sharp_flash_access_start(dev);

	for (page = 0 ; page < (size / FLASH_PAGE_SIZE) ; page++) {
		touch_msleep(FLASH_WAIT);
		sharp_flash_erase_page(dev, page);
		TOUCH_I("flash_erase_page done: page %d\n", page);

		touch_msleep(FLASH_WAIT);
		sharp_flash_write_page(dev, page, &(image[page * FLASH_PAGE_SIZE]));
		TOUCH_I("flash_write_page done: page %d\n", page);

		if (FLASH_VERIFY) {
			touch_msleep(FLASH_WAIT);
			sharp_flash_verify_page(dev, page, &(image[page * FLASH_PAGE_SIZE]));
			TOUCH_I("flash_verify_page done: page %d\n", page);
		}
	}
	TOUCH_I("f/w updating...... done!!!\n");

	return 0;
}

int sharp_fw_update(struct device *dev, const struct firmware *fw)
{
	u8 *image = NULL;
	unsigned long image_size = fw->size;

	TOUCH_I("%s\n", __func__);

	image = kzalloc(sizeof(char) * (image_size + 1), GFP_KERNEL);
	if (image == NULL) {
		TOUCH_E("Can not allocate memory\n");
		return -EIO;
	}

	memcpy(image, fw->data, image_size);
	TOUCH_E("success from image to buffer size %lu", image_size);

	sharp_cmd_system_state_sleep(dev);
	sharp_update_flash(dev, image, image_size);

	kfree(image);

	return 0;
}

