/***************************************************************************
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
 *    File  :
 *    Author(s)   :
 *    Description :
 *
 ***************************************************************************/

/****************************************************************************
* Include files
****************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/switch.h>
#include <linux/firmware.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/input/epack.h>
#include "internal.h"

/****************************************************************************
* Local function Prototypes
****************************************************************************/

/****************************************************************************
* Manifest constants / Defines
****************************************************************************/
#define EPACK_FW_PATH_REV_05 "epack/th8/EM-P100_0412_050F_AP.bin"
#define EPACK_FW_PATH_REV_06 "epack/th8/EM-P100_0412_060F_AP.bin"

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
* Type definitions
****************************************************************************/

/****************************************************************************
* Variables
****************************************************************************/

/****************************************************************************
* Extern functions prototypes
****************************************************************************/

/****************************************************************************
* Local functions
****************************************************************************/
static char *epack_get_fw_path(unsigned int hw_ver)
{
	char *path;

	switch(hw_ver) {
	case 0x05:
		path = EPACK_FW_PATH_REV_05;
		break;
	case 0x06:
		path = EPACK_FW_PATH_REV_06;
		break;
	default:
		path = NULL;
		break;
	}

	return path;
}

static void epack_prepare_packet(u8 *buf, unsigned int cmd, unsigned int cnt)
{
	memset(buf, 0, EPACK_PACKET_SIZE);
	memcpy(buf, &cmd, sizeof(cmd));
	memcpy(buf + 4, &cnt, sizeof(cnt));
}

static unsigned int epack_cmd_delay(unsigned int cmd)
{
	int delay;

	switch (cmd) {
	case CMD_GET_BAT_VOLTAGE:
	case CMD_GET_BAT_TEMP:
	case CMD_GET_CHG_STATUS:
	case CMD_GET_FAULT_STATUS:
	case CMD_GET_OUTPUT_STATUS:
	case CMD_VBUS_CONTROL:
		delay = 50;
		break;
	case CMD_RUN_APROM:
	case CMD_RUN_LDROM:
		delay = 500;
		break;
	default:
		delay = 0;
		break;
	}

	return delay;
}

static int epack_check_packet_cnt(u8 *buf, unsigned int cnt)
{
	unsigned int packet_cnt;

	memcpy(&packet_cnt, buf + 4, sizeof(packet_cnt));

	if (packet_cnt != cnt)
		return -EINVAL;

	return 0;
}

int epack_read(struct i2c_client *client, unsigned int cmd, u8 *data, unsigned int len)
{
	struct epack *ep = i2c_get_clientdata(client);
	u8 packet[EPACK_PACKET_SIZE];
	struct i2c_msg send = {
		.addr = client->addr,
		.flags = client->flags,
		.len = EPACK_PACKET_SIZE,
		.buf = packet,
	};
	struct i2c_msg recv = {
		.addr = client->addr,
		.flags = client->flags | I2C_M_RD,
		.len = EPACK_PACKET_SIZE,
		.buf = packet,
	};
	u8 *buffer = packet;
	unsigned int packet_cnt = 1;
	int rc;

	mutex_lock(&ep->i2c_rw_lock);

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (ep->i2c_buf) {
		send.ext_flag |= I2C_ENEXT_FLAG | I2C_DMA_FLAG;
		send.buf = (u8 *)(unsigned int)ep->i2c_dma;

		recv.ext_flag |= I2C_ENEXT_FLAG | I2C_DMA_FLAG;
		recv.buf = (u8 *)(unsigned int)ep->i2c_dma;

		buffer = ep->i2c_buf;
	}
#endif
	epack_prepare_packet(buffer, cmd, packet_cnt);

	rc = i2c_transfer(client->adapter, &send, 1);
	if (rc < 0) {
		dev_err(&client->dev, "failed to send. rc=%d\n", rc);
		goto out_read;
	}

	packet_cnt++;
	msleep(epack_cmd_delay(cmd));

	rc = i2c_transfer(client->adapter, &recv, 1);
	if (rc < 0) {
		dev_err(&client->dev, "failed to recv. rc=%d\n", rc);
		goto out_read;
	}

	if (epack_check_packet_cnt(buffer, packet_cnt)) {
		dev_err(&client->dev, "packet count not match.");
		rc = -EIO;
		goto out_read;
	}

	memcpy(data, buffer + 8, len);

out_read:
	mutex_unlock(&ep->i2c_rw_lock);

	return rc < 0 ? rc : 0;
}

int epack_write(struct i2c_client *client, unsigned int cmd, u8 *data, unsigned int len)
{
	struct epack *ep = i2c_get_clientdata(client);
	u8 packet[EPACK_PACKET_SIZE];
	struct i2c_msg send = {
		.addr = client->addr,
		.flags = client->flags,
		.len = EPACK_PACKET_SIZE,
		.buf = packet,
	};
	u8 *buffer = packet;
	unsigned int packet_cnt = 1;
	int rc;

	mutex_lock(&ep->i2c_rw_lock);

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (ep->i2c_buf) {
		send.ext_flag |= I2C_ENEXT_FLAG | I2C_DMA_FLAG;
		send.buf = (u8 *)(unsigned int)ep->i2c_dma;

		buffer = ep->i2c_buf;
	}
#endif
	epack_prepare_packet(buffer, cmd, packet_cnt);

	/* fill data */
	if (data && len)
		memcpy(buffer + 8, data, len);

	rc = i2c_transfer(client->adapter, &send, 1);
	if (rc < 0) {
		dev_err(&client->dev, "failed to send. rc=%d\n", rc);
		goto out_write;
	}

	packet_cnt++;
	msleep(epack_cmd_delay(cmd));

out_write:
	mutex_unlock(&ep->i2c_rw_lock);

	return rc < 0 ? rc : 0;
}

static int epack_read_fw_ver(struct epack *ep, u16 *value)
{
	int rc;
	u8 version;

	rc = epack_read(ep->client, CMD_GET_FWVER, &version, 1);
	if (rc) {
		dev_err(ep->dev, "fail to read firmware version ( err = %d )\n", rc);
		return rc;
	}

	*value = (u16)version;

	return 0;
}

static int epack_read_hw_ver(struct epack *ep, u16 *value)
{
	int rc;
	u8 version;

	rc = epack_read(ep->client, CMD_GET_HWVER, &version, 1);
	if (rc) {
		dev_err(ep->dev, "fail to read hardware version ( err = %d )\n", rc);
		return rc;
	}

	*value = (u16)version;

	return 0;
}

static int epack_read_voltage(struct epack *ep, u16 *value)
{
	int rc;
	u16 voltage = 0;

	rc = epack_read(ep->client, CMD_GET_BAT_VOLTAGE, (u8 *)&voltage, sizeof(voltage));
	if (rc) {
		dev_err(ep->dev, "fail to read battery voltage ( err = %d )\n", rc);
		return rc;
	}

	*value = voltage;

	return 0;
}

static int epack_read_temperature(struct epack *ep, u16 *value)
{
	int rc;
	u16 temp = 0;

	rc = epack_read(ep->client, CMD_GET_BAT_TEMP, (u8 *)&temp, sizeof(temp));
	if (rc) {
		dev_err(ep->dev, "fail to read temperature ( err = %d )\n", rc);
		return rc;
	}

	*value = temp;

	return 0;
}

static int epack_read_chg_status(struct epack *ep, u16 *value)
{
	int rc;
	u8 chg_stat;

	rc = epack_read(ep->client, CMD_GET_CHG_STATUS, (u8 *)&chg_stat, sizeof(chg_stat));
	if (rc) {
		dev_err(ep->dev, "fail to read charging status ( err = %d )\n", rc);
		return rc;
	}

	*value = (u16)chg_stat;

	return 0;
}

static int epack_read_fault(struct epack *ep, u16 *value)
{
	int rc;
	u8 fault;

	rc = epack_read(ep->client, CMD_GET_FAULT_STATUS, (u8 *)&fault, sizeof(fault));
	if (rc) {
		dev_err(ep->dev, "fail to read fault status ( err = %d )\n", rc);
		return rc;
	}

	*value = (u16)fault;

	return 0;
}

static int epack_read_output(struct epack *ep, u16 *value)
{
	int rc;
	u8 output;

	rc = epack_read(ep->client, CMD_GET_OUTPUT_STATUS, (u8 *)&output, sizeof(output));
	if (rc) {
		dev_err(ep->dev, "fail to read Vbus output status ( err = %d )\n", rc);
		return rc;
	}

	*value = (u16)output;

	return 0;
}

static int epack_read_version_from_bin(struct epack *ep, char* path, int *fw_ver, int *hw_ver)
{
	const struct firmware *fw;
	int rc = 0;

	rc = request_firmware(&fw, path, ep->dev);
	if (rc < 0) {
		dev_err(ep->dev, "fail to request firmware ( path: %s, err = %d )\n", path, rc);
		return rc;
	}

	*fw_ver = fw->data[(fw->size)-1];
	*hw_ver = fw->data[(fw->size)-2];

	release_firmware(fw);

	return 0;
}

#define PAGE_SIZE_EPACK 0x200
static int epack_fw_checksum(unsigned char *buf, int len)
{
	int c = 0;
	int i;

	for (i = 0; i < len; i++) {
		c += buf[i];
	}

	return (c);
}

static unsigned short epack_fw_get_checksum(u8 *buf, int len)
{
	u8 aprom_buf[PAGE_SIZE_EPACK];
	unsigned int length = PAGE_SIZE_EPACK;
	unsigned short lcksum = 0;
	int i;

	memset(aprom_buf, 0, PAGE_SIZE_EPACK);

	for(i = 0; i < len; i += PAGE_SIZE_EPACK) {
		memcpy(aprom_buf, buf + i, PAGE_SIZE_EPACK);

		length = PAGE_SIZE_EPACK;
		if (len - i < length)
			length = len - i;

		lcksum += epack_fw_checksum(aprom_buf, length);
	}

	return lcksum;
}

static int epack_aprom_upgrade(struct epack *ep, char *path)
{
	struct i2c_client *client = ep->client;
	u8 packet[EPACK_PACKET_SIZE];
	struct i2c_msg send = {
		.addr = ep->client->addr,
		.flags = ep->client->flags,
		.len = EPACK_PACKET_SIZE,
		.buf = packet,
	};
	struct i2c_msg recv = {
		.addr = ep->client->addr,
		.flags = ep->client->flags | I2C_M_RD,
		.len = EPACK_PACKET_SIZE,
		.buf = packet,
	};
	u8 *buffer = packet;

	unsigned int cmd = CMD_UPDATE_DATE_FLASH;
	const struct firmware *fw;
	u8 *data;
	unsigned int size;
	unsigned int fw_checksum, dev_checksum;

	unsigned int data_size;
	unsigned int sent = 0;
	unsigned int packet_cnt = 1;
	int rc;

	dev_info(ep->dev, "begin aprom upgrade\n");

	if (!path) {
		dev_err(ep->dev, "firmware path invalid\n");
		return -EINVAL;
	}

	rc = request_firmware(&fw, path, ep->dev);
	if (rc < 0) {
		dev_err(ep->dev, "fail to get firmware. path: %s rc=%d\n", path, rc);
		return rc;
	}

	dev_info(ep->dev, "firmware 0x%02X is ready for hw 0x%02X",
			fw->data[(fw->size)-1], fw->data[(fw->size)-2]);
	data = (u8 *)fw->data;
	size = fw->size - 2;
	fw_checksum = epack_fw_get_checksum(data, size);

	mutex_lock(&ep->i2c_rw_lock);

	if (fw->data[(fw->size)-2] != ep->hw_ver) {
		rc = -EINVAL;
		dev_err(&client->dev, "WARN : hardware version is mis-matched\n");
		goto out_upgrade_aprom;
	}

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (ep->i2c_buf) {
		send.ext_flag |= I2C_ENEXT_FLAG | I2C_DMA_FLAG;
		send.buf = (u8 *)(unsigned int)ep->i2c_dma;

		recv.ext_flag |= I2C_ENEXT_FLAG | I2C_DMA_FLAG;
		recv.buf = (u8 *)(unsigned int)ep->i2c_dma;

		buffer = ep->i2c_buf;
	}
#endif

	/* prepare send packet */
	epack_prepare_packet(buffer, cmd, packet_cnt);

	/* fill data */
	data_size = EPACK_PACKET_SIZE - 16;
	memset(buffer + 8, 0, sizeof(unsigned int));	// fill start address as 0
	memcpy(buffer + 12, &size, sizeof(size));	// fill firmware size
	memcpy(buffer + 16, data, data_size);

	do {
		rc = i2c_transfer(client->adapter, &send, 1);
		if (rc < 0) {
			dev_err(&client->dev, "failed to send. rc=%d\n", rc);
			goto out_upgrade_aprom;
		}

		sent += data_size;
		packet_cnt++;

		rc = i2c_transfer(client->adapter, &recv, 1);
		if (rc < 0) {
			dev_err(&client->dev, "failed to recv. rc=%d\n", rc);
			goto out_upgrade_aprom;
		}

		if (epack_check_packet_cnt(buffer, packet_cnt)) {
			dev_err(&client->dev, "packet count not match.");
			rc = -EIO;
			goto out_upgrade_aprom;
		}

		packet_cnt++;
		cmd = 0;
		data_size = EPACK_PACKET_SIZE - 8;
		if (size - sent < data_size)
			data_size = size - sent;

		/* fill next data */
		if (data_size) {
			epack_prepare_packet(buffer, cmd, packet_cnt);
			memcpy(buffer + 8, data + sent, data_size);
		}
	} while (data_size);

	/* compare checksum */
	memcpy(&dev_checksum, buffer + 8, sizeof(dev_checksum));
	if (fw_checksum != dev_checksum) {
		dev_err(ep->dev, "checksum mismatch. fw=0x%04x, dev=0x%04x",
				fw_checksum, dev_checksum);
		rc = -EIO;
		goto out_upgrade_aprom;
	}

#if 0 /* branden.you_20170522 */
	dev_info(ep->dev, "skip write checksum\n");
	goto out_upgrade_aprom;
#endif
	dev_info(ep->dev, "begin writing checksum\n");

	/* write checksum */
	packet_cnt = 1;
	cmd = CMD_WRITE_CHECKSUM;
	epack_prepare_packet(buffer, cmd, packet_cnt);
	memcpy(buffer + 8, &size, sizeof(size));
	memcpy(buffer + 12, &fw_checksum, sizeof(fw_checksum));

	rc = i2c_transfer(client->adapter, &send, 1);
	if (rc < 0) {
		dev_err(&client->dev, "failed to send. rc=%d\n", rc);
		goto out_upgrade_aprom;
	}

	sent += data_size;
	packet_cnt++;
	msleep(200);

	rc = i2c_transfer(client->adapter, &recv, 1);
	if (rc < 0) {
		dev_err(&client->dev, "failed to recv. rc=%d\n", rc);
		goto out_upgrade_aprom;
	}

	if (epack_check_packet_cnt(buffer, packet_cnt)) {
		dev_err(&client->dev, "packet count not match.");
		rc = -EIO;
		goto out_upgrade_aprom;
	}

	dev_info(ep->dev, "done writing checksum\n");
	rc = 0;
	msleep(1000);

out_upgrade_aprom:
	mutex_unlock(&ep->i2c_rw_lock);

	release_firmware(fw);

	dev_info(ep->dev, "done aprom upgrade\n");

	return rc < 0 ? rc : 0;
}



/****************************************************************************
* Global functions
****************************************************************************/
int epack_read_dbg_info(struct epack *ep, u16 *volt, u16 *temp, u16 *chg, u16 *fault, u16 *output)
{
	int rc;

	rc = epack_read_voltage(ep, volt);
	if (rc)
		return rc;

	rc = epack_read_temperature(ep, temp);
	if (rc)
		return rc;

	rc = epack_read_chg_status(ep, chg);
	if (rc)
		return rc;

	rc = epack_read_fault(ep, fault);
	if (rc)
		return rc;

	rc = epack_read_output(ep, output);
	if (rc)
		return rc;

	return 0;
}
EXPORT_SYMBOL(epack_read_dbg_info);

bool epack_need_fw_upgrade(struct epack *ep)
{
	int rc = 0;
	u16 hw_ver = 0x05;
	u16 fw_ver = 0x00;
	unsigned int bin_hw_ver, bin_fw_ver;
	char *path = NULL;

	rc = epack_read_hw_ver(ep, &hw_ver);
	if (rc)
		return false;

	rc = epack_read_fw_ver(ep, &fw_ver);
	if (rc)
		return false;

	dev_info(ep->dev, "fw_ver=0x%02X, hw_ver=0x%02X\n", fw_ver, hw_ver);
	ep->fw_ver = fw_ver;
	ep->hw_ver = hw_ver;

	path = epack_get_fw_path(hw_ver);
	if (!path) {
		dev_err(ep->dev, "WARN : there is no matched hardware version\n");
		return false;
	}

	rc = epack_read_version_from_bin(ep, path, &bin_fw_ver, &bin_hw_ver);
	if (rc) {
		dev_err(ep->dev, "fail to read binary firmware info ( err = %d )\n", rc);
		return false;
	}
	dev_info(ep->dev, "bin_fw_ver = 0x%02X, bin_hw_ver = 0x%02X\n", bin_fw_ver, bin_hw_ver);

	if (hw_ver!=bin_hw_ver) {
		dev_err(ep->dev, "WARN : hardware version is mismatched\n");
		return false;
	}

	if ((fw_ver&0xF0) == 0xF0) {
		dev_info(ep->dev, "firmware is test version\n");
		return false;
	}

	if (fw_ver >= bin_fw_ver) {
		dev_info(ep->dev, "firmware version is latest\n");
		return false;
	}

	dev_info(ep->dev, "need to upgrade firmware\n");
	return true;

}
EXPORT_SYMBOL(epack_need_fw_upgrade);

int epack_do_fw_upgrade(struct epack *ep, char *path)
{
	char *filepath = NULL;

	if (ep->fw_status != EPACK_FW_OK) {
		dev_err(ep->dev, "duplicated fw upgrade request\n");
		dump_stack();
		return -EPERM;
	}

	memset(ep->fw_path, 0x00, sizeof(ep->fw_path));

	if (path) {
		dev_err(ep->dev, "firmware path is too long ( len = %d )\n", strlen(path));
		if (strlen(path) > sizeof(ep->fw_path)) {
			dev_err(ep->dev, "firmware path is too long ( len = %d )\n", strlen(path));
			return -1;
		}
		strcpy(ep->fw_path, path);
	} else {
		filepath = epack_get_fw_path(ep->hw_ver);
		if (!filepath) {
			dev_err(ep->dev, "WARN : there is no matched hardware version\n");
			return -1;
		} else {
			strcpy(ep->fw_path, filepath);
		}
	}

	ep->fw_status = EPACK_FW_UPGRADE;
	dev_info(ep->dev, "firmware : %s\n", ep->fw_path);
	schedule_delayed_work(&ep->upgrade, msecs_to_jiffies(1));

	return 0;
}
EXPORT_SYMBOL(epack_do_fw_upgrade);

int epack_control_vbus(struct epack *ep, int enable)
{
	int rc;

	rc = epack_write(ep->client, CMD_VBUS_CONTROL, (u8 *)&enable, sizeof(enable));
	if (rc) {
		dev_err(ep->dev, "fail to write vbus control ( err = %d )\n", rc);
		return rc;
	}

	return 0;
}
EXPORT_SYMBOL(epack_control_vbus);


/****************************************************************************
* Event triggered handler functions
****************************************************************************/
static void epack_fw_upgrade(struct work_struct *work)
{
	struct epack *ep = container_of(to_delayed_work(work), struct epack, upgrade);
	int mode = EPACK_APROM_MODE;
	int rc;

	dev_info(ep->dev, "begin firmware upgrade\n");

	/* check epack online */
	if (!ep->is_present) {
		dev_err(ep->dev, "can't upgrade ( epack is not attached )\n");
		goto out_upgrade;
	}

	/* check epack mode */
	rc = epack_read(ep->client, CMD_GET_FLASHMODE, (u8 *)&mode, sizeof(mode));
	if (rc) {
		dev_err(ep->dev, "can't upgrade ( fail to read flash mode )\n");
		goto out_upgrade;
	}

	if (mode == EPACK_LDROM_MODE)
		goto do_upgrade;

	/* set epack mode to LDROM */
	dev_info(ep->dev, "run LDROM\n");
	rc = epack_write(ep->client, CMD_RUN_LDROM, NULL, 0);
	if (rc) {
		dev_err(ep->dev, "can't upgrade ( fail to run LDROM )\n");
		goto out_upgrade;
	}

	rc = epack_read(ep->client, CMD_GET_FLASHMODE, (u8 *)&mode, sizeof(mode));
	if (rc) {
		dev_err(ep->dev, "can't upgrade ( fail to read flash mode )\n");
		goto out_upgrade;
	}

	if (mode != EPACK_LDROM_MODE) {
		dev_err(ep->dev, "can't upgrade ( fail to run LDROM ( mode = %d ) )\n", mode);
		goto out_upgrade;
	}

do_upgrade:
	/* update epack aprom */
	rc = epack_aprom_upgrade(ep, ep->fw_path);
	if (rc) {
		dev_err(ep->dev, "fail to upgrade firmware\n");
		goto out_upgrade;
	}

	//ep->fw_path = NULL;

out_upgrade:

	if (mode == EPACK_LDROM_MODE) {
		dev_info(ep->dev, "run APROM\n");
		rc = epack_write(ep->client, CMD_RUN_APROM, NULL, 0);
		if (rc) {
			dev_err(ep->dev, "fail to rum APROM\n");
		}
	}

	ep->fw_status = EPACK_FW_OK;
	ep->pwr_status = EPACK_POWER_NA;
	//ep->need_checking = true;
	ep->trigger_src |= EPACK_TRIGGER_FIRMWARE;
	schedule_delayed_work(&ep->update, msecs_to_jiffies(400));

	dev_info(ep->dev, "done firmware upgrade\n");
	return;

}

/****************************************************************************
* Device driver functions
****************************************************************************/
int epack_fw_init(struct epack *ep)
{
	ep->fw_status = EPACK_FW_OK;

	INIT_DELAYED_WORK(&ep->upgrade, epack_fw_upgrade);

	return 0;
}

/* End Of File */
