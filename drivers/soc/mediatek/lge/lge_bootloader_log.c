/*
 * arch/arm/mach-msm/lge/lge_bootloader_log.c
 *
 * Copyright (C) 2012 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/memblock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <asm/page.h>
#include <asm/setup.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#endif

#define LOGBUF_SIG	0x6c6f6762

struct log_buffer {
	uint32_t    sig;
	uint32_t    start;
	uint32_t    size;
	uint8_t     data[0];
};

unsigned long boot_logbuf_paddr;
unsigned long boot_logbuf_size;

struct log_buffer *boot_logbuf_vaddr;


static int __init logbuf_parse_dt(unsigned long node, const char *uname, int depth, void *data)
{
	int len;
	const __be32 *prop;

	if (depth != 1 || (strcmp(uname, "chosen") != 0 && strcmp(uname, "chosen@0") != 0))
		return 0;

	prop = of_get_flat_dt_prop(node, "lge,log_buffer_phy_addr", &len);
	if (!prop)
		return 0;

	boot_logbuf_paddr = of_read_number(prop, len/4);

	prop = of_get_flat_dt_prop(node, "lge,log_buffer_size", &len);
	if (!prop)
		return 0;

	boot_logbuf_size = of_read_number(prop, len/4);

	return 1;
}

static int __init logbuf_of_init(void)
{
	of_scan_flat_dt(logbuf_parse_dt, NULL);
	return 0;
}
static void *logbuf_ram_vmap(phys_addr_t start, size_t size,
		unsigned int memtype)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	if (memtype)
		prot = pgprot_noncached(PAGE_KERNEL);
	else
		prot = pgprot_writecombine(PAGE_KERNEL);

	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n",
		       __func__, page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);

	return vaddr;
}

static void *logbuf_ram_iomap(phys_addr_t start, size_t size,
		unsigned int memtype)
{
	void *va;

	if (!request_mem_region(start, size, "persistent_ram")) {
		pr_err("request mem region (0x%llx@0x%llx) failed\n",
			(unsigned long long)size, (unsigned long long)start);
		return NULL;
	}

	if (memtype)
		va = ioremap(start, size);
	else
		va = ioremap_wc(start, size);

	return va;
}

static int __init lge_bootlog_init(void)
{
	char *buffer;
	char *token;
	char *ct = "\n";

	if (pfn_valid(boot_logbuf_paddr >> PAGE_SHIFT)) {
		boot_logbuf_vaddr = logbuf_ram_vmap(boot_logbuf_paddr, boot_logbuf_size, 0);
	} else {
		boot_logbuf_vaddr = logbuf_ram_iomap(boot_logbuf_paddr, boot_logbuf_size, 0);
	}

	if (!boot_logbuf_vaddr) {
		pr_err("%s: Failed to map 0x%llx pages at 0x%llx\n", __func__,
			(unsigned long long)boot_logbuf_size, (unsigned long long)boot_logbuf_paddr);
		return -ENOMEM;
	}

	pr_info("%s: logbuf paddr=0x%lx\n",__func__, boot_logbuf_paddr);
	pr_info("%s: logbuf sig=%x\n",__func__, boot_logbuf_vaddr->sig);
	pr_info("%s: logbuf size=%d/%d\n",__func__, boot_logbuf_vaddr->start, boot_logbuf_vaddr->size);
	pr_info("--------------------------------------------------------------\n");
	pr_info("below logs are got from bootloader \n");
	pr_info("--------------------------------------------------------------\n");
	pr_info("\n");
	buffer = (char *)boot_logbuf_vaddr->data;

	while (1) {
		token = strsep(&buffer, ct);
		if (!token) {
			pr_info("%s: token %p\n",__func__, token);
			break;
		}
		pr_info("%s\n", token);
	}
	pr_info("--------------------- bootloader log end ---------------------\n");

	if (boot_logbuf_vaddr) {
		if (pfn_valid(boot_logbuf_paddr>> PAGE_SHIFT)) {
			vunmap(boot_logbuf_vaddr);
		} else {
			iounmap(boot_logbuf_vaddr);
			release_mem_region(boot_logbuf_paddr, boot_logbuf_size);
		}
		boot_logbuf_vaddr = NULL;
	}

	return 0;
}

static void __exit lge_bootlog_exit(void)
{
	return;
}

#ifdef CONFIG_OF
early_initcall(logbuf_of_init);
#endif
module_init(lge_bootlog_init);
module_exit(lge_bootlog_exit);

MODULE_DESCRIPTION("LGE bootloader log driver");
MODULE_AUTHOR("SungEun Kim <cleaneye.kim@lge.com>");
MODULE_LICENSE("GPL");
