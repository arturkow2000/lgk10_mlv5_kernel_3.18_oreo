/* Copyright (c) 2013-2014, LG Eletronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#include <linux/mutex.h>
#include <linux/fs_struct.h>
#include <linux/sched.h>
#include <linux/path.h>

#include "mt-plat/partition.h"
#include "mt-plat/mt_devinfo.h"
#include "soc/mediatek/lge/lge_efuse_access.h"

int get_partition_path(char *partition_name, char *path_name, int path_size)
{
	struct hd_struct *part = NULL;

	part = get_part(partition_name);
	if (!part) {
		pr_err("Not find partition %s\n", partition_name);
		return RET_ERR;
	}
	snprintf(path_name, path_size, "/dev/block/mmcblk0p%d", part->partno);
	put_part(part);
	return RET_OK;
}

int get_efuse_partition_data(unsigned char *result)
{
	int ret;
	struct file *filp;
	struct path root;
	mm_segment_t curr_fs;
	char part_path[64];

	ret = get_partition_path(EFUSE_PARTITION_NAME, part_path, sizeof(part_path));
	if (ret == RET_ERR) {
		pr_err("get EFUSE partition info fail!\n");
		return ret;
	}

	task_lock(&init_task);
	get_fs_root(init_task.fs, &root);
	task_unlock(&init_task);
	filp = file_open_root(root.dentry, root.mnt, part_path, O_RDONLY, 0);
	path_put(&root);
	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		pr_err("Open EFUSE partition fail! errno=%d\n", ret);
		return ret;
	}

	filp->f_op->llseek(filp, 0x0, SEEK_SET);
	curr_fs = get_fs();
	set_fs(KERNEL_DS);

	ret = filp->f_op->read(filp, result, EFUSE_PARTITION_USED_SIZE, &(filp->f_pos));
	filp_close(filp, NULL);
	set_fs(curr_fs);

	if (ret != EFUSE_PARTITION_USED_SIZE) {
		pr_err("read fail!errno=%d\n", ret);
		return RET_ERR;
	}

	return RET_OK;
}

u32 read_efuse_partition(void)
{
	struct hash_desc desc;
	struct scatterlist sg;
	unsigned char efuse_partition_all[EFUSE_PARTITION_USED_SIZE] = {0x0, }, *p_offset;
	u32 efuse_magic1, efuse_magic2;

	printk(KERN_INFO "[EFUSE]%s start\n", __func__);

	// get efuse partition hash
	if(b_efuse_read){
		printk(KERN_INFO "[EFUSE]%s : efuse partition already loaded \n", __func__);
		return RET_OK;
	}
	printk(KERN_INFO "[EFUSE]%s : efuse partition first read. \n", __func__);
	mutex_lock(&secdat_lock);

	// read efuse partition all
	if (get_efuse_partition_data(efuse_partition_all) != RET_OK) {
		printk(KERN_ERR "[EFUSE]%s :efuse partition read fail\n", __func__);
		goto err_part;
	}

	// Overwrite magic-code
	efuse_magic1 = EFUSE_MAGIC_CODE1;
	efuse_magic2 = EFUSE_MAGIC_CODE2;
	memcpy(efuse_partition_all, (unsigned char *)&efuse_magic1, 4);
	memcpy(efuse_partition_all + 4, (unsigned char *)&efuse_magic2, 4);

	// compute partition hash value
	sg_init_one(&sg, efuse_partition_all, sizeof(efuse_partition_all));

	desc.flags=0;
	desc.tfm = crypto_alloc_hash("sha256", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(desc.tfm)){
		printk(KERN_ERR "[EFUSE]%s :hash alloc error\n", __func__);
		goto err_mem;
	}

	if (crypto_hash_init(&desc) != 0){
		printk(KERN_ERR "[EFUSE]%s : hash init error\n", __func__);
		goto err_mem;
	}

	if(crypto_hash_digest(&desc, &sg, sizeof(efuse_partition_all), hash_p) != 0){
		printk(KERN_ERR "[EFUSE]%s : hash_digest error\n", __func__);
		goto err_mem;
	}
	crypto_free_hash(desc.tfm);

	// copy partition values to efuse_data struct
	p_offset = efuse_partition_all + EFUSE_PARTITION_OFFSET_SBC_PUBK_HASH;
	memcpy(efuse_data.pubk_hash, p_offset, sizeof(efuse_data.pubk_hash));

	p_offset = efuse_partition_all + EFUSE_PARTITION_OFFSET_CONFIG;
	memcpy(efuse_data.config, p_offset, sizeof(efuse_data.config));

	b_efuse_read = true;
	mutex_unlock(&secdat_lock);

	printk(KERN_INFO "[EFUSE]%s end\n", __func__);
	return RET_OK;

err_mem:
	if (desc.tfm)
		crypto_free_hash(desc.tfm);
err_part:
	mutex_unlock(&secdat_lock);
	printk(KERN_INFO "[EFUSE]%s end\n", __func__);
	return RET_ERR;
}

u32 read_efuse_memory(u32 index)
{
	u32 ret;
	ret = get_devinfo_with_index(index);
	return ret;
}

u32 get_efuse_values(u32 *result_verify, u32 *result_part)
{
	int i, t;
	u32 value_part = 0, value_memory = 0;
	u32 temp_same = 0, temp_mtk = 0, temp_lge = 0;
	unsigned char hash_k[SHA256_HASH_SIZE] = {0x0, };
	unsigned char temp_buf[4] = {0};

// ---------------------------------
// Read efuse partition if not read yet.
// ---------------------------------
	if (read_efuse_partition() != RET_OK) {
		printk(KERN_ERR "[EFUSE]%s : Partition read fail\n", __func__);
		return false;
	}

// ---------------------------------
// Check partition hash
// ---------------------------------
#ifndef CONFIG_LGE_QFPROM_SECHASH
	printk(KERN_ERR "[EFUSE]%s : CONFIG_LGE_QFPROM_SECHASH is not exist\n", __func__);
	return RET_ERR;
#else
	// read kernel config hash
	for(i = 0; i < SHA256_HASH_SIZE; i++){
		memset(temp_buf, 0, 4);
		memcpy(temp_buf, CONFIG_LGE_QFPROM_SECHASH + (i*2), 2);
		sscanf(temp_buf, "%x", &t);
		hash_k[i] = t;
	}

	if(memcmp(hash_k, hash_p, SHA256_HASH_SIZE) != 0){
		printk(KERN_ERR "[EFUSE]%s : sec hash different\n", __func__);
		printk(KERN_ERR "[EFUSE]%s : kernel - %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", __func__,
			hash_k[0],hash_k[1],hash_k[2],hash_k[3],hash_k[4],hash_k[5],hash_k[6],hash_k[7],
			hash_k[8],hash_k[9],hash_k[10],hash_k[11],hash_k[12],hash_k[13],hash_k[14],hash_k[15],
			hash_k[16],hash_k[17],hash_k[18],hash_k[19],hash_k[20],hash_k[21],hash_k[22],hash_k[23],
			hash_k[24],hash_k[25],hash_k[26],hash_k[27],hash_k[28],hash_k[29],hash_k[30],hash_k[31]);
		printk(KERN_ERR "[EFUSE]%s : partition - %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", __func__,
			hash_p[0],hash_p[1],hash_p[2],hash_p[3],hash_p[4],hash_p[5],hash_p[6],hash_p[7],
			hash_p[8],hash_p[9],hash_p[10],hash_p[11],hash_p[12],hash_p[13],hash_p[14],hash_p[15],
			hash_p[16],hash_p[17],hash_p[18],hash_p[19],hash_p[20],hash_p[21],hash_p[22],hash_p[23],
			hash_p[24],hash_p[25],hash_p[26],hash_p[27],hash_p[28],hash_p[29],hash_p[30],hash_p[31]);
		return RET_ERR;
	}
#endif

	// ---------------------------------
	// Check sbc pubk hash & default key
	// ---------------------------------
	printk(KERN_INFO "[EFUSE]%s: Check sbc pubk hash\n", __func__);
	printk(KERN_INFO "[EFUSE]%s: Read pubk hash. Memory(index, value), Partition(seq, value)\n", __func__);
	for (i = 0; i < ARRAY_SIZE(pubk_list); i++) {
		value_memory = read_efuse_memory(pubk_list[i].k_index);
		value_part = efuse_data.pubk_hash[i];
		printk(KERN_INFO "[EFUSE]%s: Memory(%d, 0x%08X), Partition(%d, 0x%08X)\n", __func__, pubk_list[i].k_index, value_memory, i, value_part);

		if (value_part == value_memory)
			temp_same |= (0x1 << i);
		if (value_part == pubk_list[i].mtk_key)
			temp_mtk |= (0x1 << i);
		if (value_part == pubk_list[i].lge_default_key)
			temp_lge |= (0x1 << i);
	}

	// lge_default key (none mtk key)
	printk(KERN_INFO "[EFUSE]%s: check lgek\n", __func__);
	if (temp_mtk != 0xFF)
		*result_part |= (0x1 << EFUSE_RESULT_SBC_PUBK_HASH);
	if (temp_mtk != 0xFF && temp_same == 0xFF)
		*result_verify |= (0x1 << EFUSE_RESULT_SBC_PUBK_HASH);
	printk(KERN_INFO "[EFUSE]%s: Current result (verification,result) = (%x, %x)\n", __func__, *result_verify, *result_part);

	// none lge default key (model custom key)
	printk(KERN_INFO "[EFUSE]%s: check customk\n", __func__);
	if (temp_mtk != 0xFF && temp_lge != 0xFF)
		*result_part |= (0x1 << EFUSE_RESULT_CUSTOM_PUB_KEY);
	if (temp_mtk != 0xFF && temp_lge != 0xFF && temp_same == 0xFF)
		*result_verify |= (0x1 << EFUSE_RESULT_CUSTOM_PUB_KEY);
	printk(KERN_INFO "[EFUSE]%s: Current result (verification,result) = (%x, %x)\n", __func__, *result_verify, *result_part);

	// ---------------------------------
	// Check remind fuses
	// ---------------------------------
	printk(KERN_INFO "[EFUSE]%s: Check fuse list\n", __func__);
	printk(KERN_INFO "[EFUSE]%s: Read pubk hash. Memory(index, value), Partition(seq, value)\n", __func__);
	for (i = 0; i < ARRAY_SIZE(fuse_list); i++) {
		printk(KERN_INFO "[EFUSE]%s: ---------------------------------------------------------------------)\n", __func__);
		value_memory = read_efuse_memory(fuse_list[i].k_index);
		value_part = efuse_data.config[fuse_list[i].idx];
		printk(KERN_INFO "[EFUSE]%s: Before masking - Memory(%d, 0x%08X), Partition(%d, 0x%08X)\n", __func__, fuse_list[i].k_index, value_memory, i, value_part);
		value_memory &= fuse_list[i].b_mask;
		value_part &= fuse_list[i].b_mask;
		printk(KERN_INFO "[EFUSE]%s: After masking(0x%08X) - Memory(%d, 0x%08X), Partition(%d, 0x%08X)\n", __func__, fuse_list[i].b_mask, fuse_list[i].k_index, value_memory, i, value_part);

		if (value_part > 0x0)
			*result_part |= (0x1 << fuse_list[i].r_type);
		if (value_memory > 0x0 && value_part == value_memory)
			*result_verify |= (0x1 << fuse_list[i].r_type);
		printk(KERN_INFO "[EFUSE]%s: Current result (verification,result) = (%x, %x)\n", __func__, *result_verify, *result_part);
	}

	if (*result_part > 0x0 && *result_part == *result_verify)
		return RET_OK;
	else
		return RET_ERR;
}

static ssize_t efuse_sbc_check_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 value;

	printk(KERN_INFO "[EFUSE]%s start\n", __func__);
	value = read_efuse_memory(SBC_KERNEL_INDEX);
	printk(KERN_INFO "[EFUSE]%s - raw value : %02x\n", __func__, value);
	value &= SBC_KERNEL_MASKING;
	printk(KERN_INFO "[EFUSE]%s - masking & making value : %02x, %02x\n", __func__, SBC_KERNEL_MASKING, value);
	printk(KERN_INFO "[EFUSE]%s end\n", __func__);

	return sprintf(buf, "%x\n", (value > 0)?1:0);
}
static DEVICE_ATTR(qfusing, S_IWUSR | S_IRUGO, efuse_sbc_check_show, NULL);

static ssize_t efuse_check_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 ret_b = RET_ERR, ret_v = 0x0, ret_r = 0x0;

	printk(KERN_INFO "[EFUSE]%s start\n", __func__);
	ret_b = get_efuse_values(&ret_v, &ret_r);
	printk(KERN_INFO "[EFUSE]%s end\n", __func__);

	return sprintf(buf, "%x\n", (ret_b == RET_OK)?1:0);
}
static DEVICE_ATTR(qfusing_check, S_IWUSR | S_IRUGO, efuse_check_show, NULL);

static ssize_t efuse_verification_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 ret_v = 0x0, ret_r = 0x0;

	printk(KERN_INFO "[EFUSE]%s start\n", __func__);
	get_efuse_values(&ret_v, &ret_r);
	printk(KERN_INFO "[EFUSE]%s end\n", __func__);

	return sprintf(buf, "%x\n", ret_v);
}
static DEVICE_ATTR(qfusing_verification, S_IWUSR | S_IRUGO, efuse_verification_show, NULL);

static ssize_t efuse_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 ret_v = 0x0, ret_r = 0x0;

	printk(KERN_INFO "[EFUSE]%s start\n", __func__);
	get_efuse_values(&ret_v, &ret_r);
	printk(KERN_INFO "[EFUSE]%s end\n", __func__);

	return sprintf(buf, "%x\n", ret_r);
}
static DEVICE_ATTR(qresult, S_IWUSR | S_IRUGO, efuse_result_show, NULL);

static ssize_t sec_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = RET_ERR;

	printk(KERN_INFO "[EFUSE]%s start\n", __func__);
	ret = read_efuse_partition();
	printk(KERN_ERR "[EFUSE]%s end\n", __func__);

	return sprintf(buf, "%d\n", (ret == RET_OK)?1:0);
}

static ssize_t sec_read_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	printk(KERN_INFO "[EFUSE]%s start\n", __func__);
	read_efuse_partition();
	printk(KERN_INFO "[EFUSE]%s end\n", __func__);

	return count;
}
static DEVICE_ATTR(sec_read, S_IWUSR | S_IRUGO, sec_read_show, sec_read_store);

static ssize_t efuse_antirollback_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[EFUSE]%s : MT6755 anti rollback is enabled\n", __func__);
	return sprintf(buf, "%d\n", 1);
}

static DEVICE_ATTR(antirollback, S_IWUSR | S_IRUGO, efuse_antirollback_show, NULL);

static struct attribute *efuse_attributes[] = {
	&dev_attr_qfusing.attr,
	&dev_attr_qfusing_check.attr,
	&dev_attr_qfusing_verification.attr,
	&dev_attr_qresult.attr,
	&dev_attr_sec_read.attr,
	&dev_attr_antirollback.attr,
	NULL
};

static const struct attribute_group efuse_attribute_group = {
	.attrs = efuse_attributes,
};

u32 get_real_version(u32 iv)
{
	int i, rv = 0;
	for (i = 0; i < 32; i += 2) {
		if (((iv >> i) & 0x1) != 0 || ((iv >> (i + 1)) & 0x1) != 0) {
			rv = i / 2 + 1;
		}
	}
	return rv;
}

static ssize_t efuse_read_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 m1, m2, v1, v2;

	printk(KERN_INFO "[EFUSE]%s : Read anti rollback version\n", __func__);
	m1 = read_efuse_memory(IDX_ARB_GROUP_0);
	m2 = read_efuse_memory(IDX_ARB_GROUP_1);

	v1 = get_real_version(m1);
	v2 = get_real_version(m2);

	if (strcmp(attr->attr.name, "group0") == 0) {
		return sprintf(buf, "%d\n", v1);
	}

	if (strcmp(attr->attr.name, "group1") == 0) {
		return sprintf(buf, "%d\n", v2);
	}

	if (strcmp(attr->attr.name, "appsbl") == 0) {
		return sprintf(buf, "%d\n", v1);
	}

	return sprintf(buf, "-1\n");
}

static DEVICE_ATTR(group0, S_IWUSR | S_IRUGO, efuse_read_version_show, NULL);
static DEVICE_ATTR(group1, S_IWUSR | S_IRUGO, efuse_read_version_show, NULL);
static DEVICE_ATTR(appsbl, S_IWUSR | S_IRUGO, efuse_read_version_show, NULL);

static struct attribute *efuse_version_attributes[] = {
	&dev_attr_group0.attr,
	&dev_attr_group1.attr,
	&dev_attr_appsbl.attr,
	NULL
};

static const struct attribute_group efuse_version_attribute_group = {
	.name = "versions",
	.attrs = efuse_version_attributes,
};

static int lge_efuse_probe(struct platform_device *pdev)
{
	int err;
	printk(KERN_INFO "[EFUSE]%s : efuse init\n", __func__);
	mutex_init(&secdat_lock);
	err = sysfs_create_group(&pdev->dev.kobj, &efuse_attribute_group);
	if (err < 0) {
		printk(KERN_ERR "[EFUSE]%s: cant create lge-efuse attribute group!\n", __func__);
		return err;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &efuse_version_attribute_group);
	if (err < 0) {
	  printk(KERN_ERR "[EFUSE]%s: cant create version attribute group\n", __func__);
	}
	return err;
}

static int lge_efuse_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id lge_efuse_match_table[] = {
	{ .compatible = MODULE_TABLE_NAME },
	{}
};
#endif

static struct platform_driver lge_efuse_driver = {
	.probe = lge_efuse_probe,
	.remove = lge_efuse_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lge_efuse_match_table,
#endif
	},
};

static int __init lge_efuse_interface_init(void)
{
	return platform_driver_register(&lge_efuse_driver);
}

static void lge_efuse_interface_exit(void)
{
	platform_driver_unregister(&lge_efuse_driver);
}


#ifdef CONFIG_OF
static struct platform_device lge_efuse_interface_platform_device = {
	.name = MODULE_NAME,
	.id = -1,
};

static int __init lge_efuse_interface_device_init(void)
{
	pr_err("%s st\n", __func__);
	return platform_device_register(&lge_efuse_interface_platform_device);
}

static void lge_efuse_interface_device_exit(void)
{
	platform_device_unregister(&lge_efuse_interface_platform_device);
}
#endif

late_initcall(lge_efuse_interface_init);
module_exit(lge_efuse_interface_exit);
#ifdef CONFIG_OF
late_initcall(lge_efuse_interface_device_init);
module_exit(lge_efuse_interface_device_exit);
#endif
MODULE_DESCRIPTION("LGE EFUSE interface driver");
MODULE_LICENSE("GPL v2");

