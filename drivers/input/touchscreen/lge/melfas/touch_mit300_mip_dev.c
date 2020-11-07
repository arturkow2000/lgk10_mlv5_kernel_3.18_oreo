/* touch_mit300_mip_dev.c
 *
 * Copyright (C) 2016 LGE.
 *
 * Author: PH1-BSP-Touch@lge.com
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

#include <touch_core.h>
#include <touch_hwif.h>

#include "touch_mit300.h"


/**
* Dev node output to user
*/
static ssize_t mip_dev_fs_read(struct file *fp, char *rbuf, size_t cnt, loff_t *fpos)
{
	struct mit_data *d = fp->private_data;
	int ret = 0;

	ret = copy_to_user(rbuf, d->dev_fs_buf, cnt);

	return ret;
}

/**
* Dev node input from user
*/
static ssize_t mip_dev_fs_write(struct file *fp, const char *wbuf, size_t cnt, loff_t *fpos)
{
	struct mit_data *d = fp->private_data;
	struct touch_core_data *ts = to_touch_core(d->d_dev);
	u8 *buf;
	int ret = 0;
	int cmd = 0;

	buf = kzalloc(cnt + 1, GFP_KERNEL);

	if ((buf == NULL) || copy_from_user(buf, wbuf, cnt)) {
		TOUCH_E("copy_from_user\n");
		ret = -EIO;
		goto EXIT;
	}

	cmd = buf[cnt - 1];

	if(cmd == 1){

		if(mit300_reg_read(ts->dev, buf, (cnt - 2), d->dev_fs_buf, buf[cnt - 2]) ){
			TOUCH_E("Mit300_I2C_Read\n");
		}
	}
	else if(cmd == 2){
		if(mit300_reg_write(ts->dev, buf, (cnt - 1)) ){
			TOUCH_E("Mit300_I2C_Write\n");
		}
	}
	else{
		goto EXIT;
	}

EXIT:
	kfree(buf);

	return ret;
}

/**
* Open dev node
*/
static int mip_dev_fs_open(struct inode *node, struct file *fp)
{
	struct mit_data *d = container_of(node->i_cdev, struct mit_data, cdev);

	fp->private_data = d;

	d->dev_fs_buf = kzalloc(1024 * 4, GFP_KERNEL);

	return 0;
}

/**
* Close dev node
*/
static int mip_dev_fs_release(struct inode *node, struct file *fp)
{
	struct mit_data *d = fp->private_data;

	kfree(d->dev_fs_buf);

	return 0;
}

/**
* Dev node info
*/
static struct file_operations mip_dev_fops = {
	.owner	= THIS_MODULE,
	.open	= mip_dev_fs_open,
	.release	= mip_dev_fs_release,
	.read	= mip_dev_fs_read,
	.write	= mip_dev_fs_write,
};

/**
* Create dev node
*/
int mip_dev_create(struct mit_data *d)
{
	int ret = 0;

	TOUCH_TRACE();
	TOUCH_I("%s [START]\n", __func__);

	if (alloc_chrdev_region(&d->mip_dev, 0, 1, "lge_touch")) {
		TOUCH_E("alloc_chrdev_region\n");
		ret = -ENOMEM;
		goto ERROR;
	}

	cdev_init(&d->cdev, &mip_dev_fops);
	d->cdev.owner = THIS_MODULE;

	if (cdev_add(&d->cdev, d->mip_dev, 1)) {
		TOUCH_E("cdev_add\n");
		ret = -EIO;
		goto ERROR;
	}

	TOUCH_I("%s [DONE]\n", __func__);
	return 0;

ERROR:
	TOUCH_E("mip_dev_create\n");
	return 0;
}


int mit300_debugging(struct device *dev)
{
#if MIP_USE_DEV

	struct mit_data *d = to_mit_data(dev);

	TOUCH_TRACE();

	d->lpwg_debug_enable = 1;

	/* Create dev node (optional) */
	if(mip_dev_create(d)){
		TOUCH_E("mip_dev_create\n");
	}

	/* Create dev */
	d->class = class_create(THIS_MODULE, "lge_touch");
	device_create(d->class, NULL, d->mip_dev, NULL, "lge_touch");

#endif
	return 0;
}

