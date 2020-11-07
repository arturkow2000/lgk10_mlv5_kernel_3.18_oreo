/*
 * arch/arm/mach-msm/lge/lge_handle_panic.c
 *
 * Copyright (C) 2010 LGE, Inc
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

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kdebug.h>
#include <asm/setup.h>
#include <linux/module.h>

#ifdef CONFIG_CPU_CP15_MMU
#include <linux/ptrace.h>
#endif

#include <soc/mediatek/lge/board_lge.h>
#include <soc/mediatek/lge/lge_handle_panic.h>
#include <linux/input.h>

#ifdef CONFIG_OF_RESERVED_MEM
#include <linux/memblock.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#endif

#define PANIC_HANDLER_NAME	"panic-handler"

unsigned int lge_crash_reason_magic;

static DEFINE_SPINLOCK(lge_panic_lock);


static struct _lge_crash_footprint *crash_fp;
static int dummy_arg;

#ifdef CONFIG_LGE_HANDLE_PANIC_BY_KEY
#define GEN_KEY_PANIC_TIMEOUT 3000
static struct delayed_work lge_gen_key_panic_timout_work;
static struct delayed_work lge_long_key_time_work;
static int key_crash_cnt = 0;
static int crash_combi[] = {KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_VOLUMEUP, KEY_VOLUMEUP, KEY_VOLUMEDOWN,
			    KEY_POWER,
			    KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_VOLUMEUP, KEY_VOLUMEUP, KEY_VOLUMEDOWN};

static int crash_key_detect = 0;

static void lge_gen_key_panic_timeout(struct work_struct *work)
{
	pr_info("Ready to panic: timeout\n");
	key_crash_cnt = 0;
}

static void lge_long_key_time(struct work_struct *work)
{
     crash_key_detect = 1;
}

void lge_gen_key_panic(int key, int value)
{
	if (!!value)
	{
		if (lge_get_crash_handle_status() != 1)
			return;

		if (key != crash_combi[key_crash_cnt]) {
			cancel_delayed_work(&lge_gen_key_panic_timout_work);
			key_crash_cnt = 0;
			crash_key_detect = 0;

			return;
		} else if (key_crash_cnt == 0) {
			schedule_delayed_work(&lge_gen_key_panic_timout_work,
					msecs_to_jiffies(GEN_KEY_PANIC_TIMEOUT));
		} else {
			pr_info("Ready to panic: cnt = %d\n", key_crash_cnt);
		}

		if (++key_crash_cnt == ARRAY_SIZE(crash_combi)) {
			schedule_delayed_work(&lge_long_key_time_work,
                     msecs_to_jiffies(5000));
		}
	}
	else
	{
		if (crash_key_detect && (key_crash_cnt == 0)) {
			panic("%s: Generate panic by key!\n", __func__);
		}
		else {
			cancel_delayed_work(&lge_long_key_time_work);
			crash_key_detect = 0;
		}
	}
}
#endif

static int gen_bug(const char *val, struct kernel_param *kp)
{
	BUG();

	return 0;
}
module_param_call(gen_bug, gen_bug,
		param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

static int gen_panic(const char *val, struct kernel_param *kp)
{
	panic("LGE Panic Handler Panic Test");

	return 0;
}
module_param_call(gen_panic, gen_panic,
		param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

static int gen_null_pointer_exception(const char *val, struct kernel_param *kp)
{
	int *panic_test;

	panic_test = 0;
	*panic_test = 0xa11dead;

	return 0;
}
module_param_call(gen_null_pointer_exception, gen_null_pointer_exception,
		param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

static DEFINE_SPINLOCK(watchdog_timeout_lock);
static int gen_wdog(const char *val, struct kernel_param *kp)
{
	unsigned long flags;

	spin_lock_irqsave(&watchdog_timeout_lock, flags);
	while(1);

	return 0;
}
module_param_call(gen_wdog, gen_wdog, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

extern void die(const char *str, struct pt_regs *regs, int err);
static int gen_die(const char *val, struct kernel_param *kp)
{
	struct pt_regs *reg;

	reg = (struct pt_regs *)crash_fp->apinfo.regs;

	die("test die", reg, 0);

	return 0;
}
module_param_call(gen_die, gen_die, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

#ifdef CONFIG_LGE_DRAM_WR_BIAS_OFFSET
static int wr_bias_offset;
static int wr_bias_set_offset(const char *val, struct kernel_param *kp)
{
	int wr_bias_for_passzone = 0;
	if (!param_set_int(val, kp))
		wr_bias_for_passzone = *(int *)kp->arg;
	else
		return -EINVAL;

	crash_fp->wr_bias_for_passzone = wr_bias_for_passzone;
	return 0;
}

static int wr_bias_get_offset(char *val, struct kernel_param *kp)
{
	return sprintf(val, "%d", lge_get_wr_bias_offset());
}
module_param_call(wr_bias_offset, wr_bias_set_offset,
		wr_bias_get_offset, &wr_bias_offset, S_IWUSR | S_IWGRP | S_IRUGO);
#endif

static u32 _lge_get_reboot_reason(void)
{
	return crash_fp->reboot_reason;
}

u32 lge_get_reboot_reason(void)
{
	return _lge_get_reboot_reason();
}

static void _lge_set_reboot_reason(u32 rr)
{
	crash_fp->reboot_reason = rr;

	return;
}

void lge_set_reboot_reason(u32 rr)
{
	u32 prev_rr;

	prev_rr = _lge_get_reboot_reason();

	if (prev_rr == LGE_CRASH_UNKNOWN ||
			prev_rr == LGE_REBOOT_REASON_NORMAL) {
		_lge_set_reboot_reason(rr);
	} else {
		/* pr_warning("block to set boot reason : "
				"prev_rr = 0x%x, new rr = 0x%x\n", prev_rr, rr); */
	}

	return;
}

void lge_set_modem_info(u32 modem_info)
{
	crash_fp->modeminfo.modem_info = modem_info;

	return;
}

u32 lge_get_modem_info(void)
{
	return crash_fp->modeminfo.modem_info;
}

void lge_set_modem_will(const char *modem_will)
{
	memset(crash_fp->modeminfo.modem_will, 0x0, LGE_MODEM_WILL_SIZE);
	strncpy(crash_fp->modeminfo.modem_will,
			modem_will, LGE_MODEM_WILL_SIZE - 1);

	return;
}

static int crash_handle_status = 0;

void lge_set_crash_handle_status(u32 enable)
{
	unsigned long flags;

	spin_lock_irqsave(&lge_panic_lock, flags);

	if (enable) {
		crash_handle_status = 1;
	} else {
		crash_handle_status = 0;
	}
	lge_crash_reason_magic = 0x6D630000;
	spin_unlock_irqrestore(&lge_panic_lock, flags);
}

int lge_get_crash_handle_status(void)
{
	return crash_handle_status;
}

static int read_crash_handle_enable(char *buffer, const struct kernel_param *kp)
{
	return sprintf(buffer, "%d", crash_handle_status);
}

module_param_call(crash_handle_enable, NULL,
		read_crash_handle_enable, &dummy_arg, S_IRUGO);


void lge_set_atf_info(const u32 buf_addr, const u32 buf_size)
{
	crash_fp->atfinfo.atf_buf_addr = buf_addr;
	crash_fp->atfinfo.atf_buf_size = buf_size;
}

void lge_set_ram_console_addr(unsigned int addr, unsigned int size)
{
	crash_fp->oopsinfo.ramconsole_paddr = addr;
	crash_fp->oopsinfo.ramconsole_size = size;
}

#ifdef CONFIG_OF_RESERVED_MEM

static void lge_panic_handler_free_page(unsigned long mem_addr,
					unsigned long size)
{
	unsigned long pfn_start, pfn_end, pfn_idx;

	pfn_start = mem_addr >> PAGE_SHIFT;
	pfn_end = (mem_addr + size) >> PAGE_SHIFT;

	for (pfn_idx = pfn_start; pfn_idx < pfn_end; pfn_idx++) {
		free_reserved_page(pfn_to_page(pfn_idx));
	}
}

static void lge_panic_handler_reserve_cleanup(unsigned long addr,
						unsigned long size)
{
	pr_info("reserved-memory free[@0x%lx+@0x%lx)\n", addr, size);

	if (addr == 0 || size == 0)
		return;

	memblock_free(addr, size);

	lge_panic_handler_free_page(addr, size);
}

static u32 sraminfo_address = 0;
static u32 sraminfo_size = 0;
static u32 atfinfo_address = 0;
static u32 atfinfo_size = 0;
static u32 atflogbackup_address = 0;
static u32 atflogbackup_size = 0;
static u32 busdbg_address = 0;
static u32 busdbg_size = 0;
static u32 preloader_mem_addr = 0;
static u32 preloader_mem_size = 0;
static u32 lk_mem_addr = 0;
static u32 lk_mem_size = 0;

static void _lge_set_reserved_4sram(u32 addr, u32 size)
{
	printk(KERN_INFO "%s : addr = 0x%x, size = 0x%x\n", __func__, addr, size);

	crash_fp->sraminfo.addr = addr;
	crash_fp->sraminfo.size = size;
}

static int lge_get_reserved_4sram(struct reserved_mem *rmem)
{
	sraminfo_address = (u32) rmem->base;
	sraminfo_size = (u32) rmem->size;

	return 0;
}

static void _lge_set_reserved_4atf(u32 addr, u32 size)
{
	printk(KERN_INFO "%s : addr = 0x%x, size = 0x%x\n", __func__, addr, size);

	crash_fp->atfinfo.atf_backup_addr = addr;
	crash_fp->atfinfo.atf_backup_size = size;
}

static int lge_get_reserved_4atf(struct reserved_mem *rmem)
{
	atfinfo_address = (u32) rmem->base;
	atfinfo_size = (u32) rmem->size;

	return 0;
}

static void _lge_set_reserved_4atflog(u32 addr, u32 size)
{
	printk(KERN_INFO "%s : addr = 0x%x, size = 0x%x\n", __func__, addr, size);

	crash_fp->atfinfo.atflog_backup_addr = addr;
	crash_fp->atfinfo.atflog_backup_size = size;
}

static int lge_get_reserved_4atflog(struct reserved_mem *rmem)
{
	atflogbackup_address = (u32) rmem->base;
	atflogbackup_size = (u32) rmem->size;

	return 0;
}

static void _lge_set_reserved_4busdbg(u32 addr, u32 size)
{
	printk(KERN_INFO "%s : addr = 0x%x, size = 0x%x\n", __func__, addr, size);

	crash_fp->busdbginfo.addr = addr;
	crash_fp->busdbginfo.size = size;
}

static int lge_get_reserved_4busdbg(struct reserved_mem *rmem)
{
	busdbg_address = (u32) rmem->base;
	busdbg_size = (u32) rmem->size;

	return 0;
}

static int lge_get_preloader_reserve_mem(struct reserved_mem *rmem)
{
	preloader_mem_addr = rmem->base;
	preloader_mem_size = rmem->size;
	return 0;
}

static int lge_get_lk_reserve_mem(struct reserved_mem *rmem)
{
	lk_mem_addr = rmem->base;
	lk_mem_size = rmem->size;
	return 0;
}

RESERVEDMEM_OF_DECLARE(lge_reserved_4sram_init,
		"lge,smembackup",lge_get_reserved_4sram);
RESERVEDMEM_OF_DECLARE(lge_reserved_4atf_init,
		"lge,atfbackup",lge_get_reserved_4atf);
RESERVEDMEM_OF_DECLARE(lge_reserved_4atflog_init,
		"lge,atflogbackup",lge_get_reserved_4atflog);
RESERVEDMEM_OF_DECLARE(lge_reserved_4systacker_init,
		"lge,busdbgbackup",lge_get_reserved_4busdbg);
RESERVEDMEM_OF_DECLARE(lge_panic_handler_pre_reserved_memory,
		"mediatek,preloader", lge_get_preloader_reserve_mem);
RESERVEDMEM_OF_DECLARE(lge_panic_handler_lk_reserved_memory,
		"mediatek,lk", lge_get_lk_reserve_mem);

static void release_reserved_mem(void)
{
	if (lge_get_crash_handle_status() != 0) {
		pr_info("Crash handler enabled..\n");
		return;
	}

	if (sraminfo_address != 0 && sraminfo_size != 0) {
		lge_panic_handler_reserve_cleanup(sraminfo_address,
				sraminfo_size);
		sraminfo_address = 0;
		sraminfo_size = 0;

		_lge_set_reserved_4sram(sraminfo_address, sraminfo_size);
	}

	if (atfinfo_address != 0 && atfinfo_size != 0) {
		lge_panic_handler_reserve_cleanup(atfinfo_address,
				atfinfo_size);
		atfinfo_address = 0;
		atfinfo_size = 0;

		_lge_set_reserved_4atf(atfinfo_address, atfinfo_size);
	}

	if (atflogbackup_address != 0 && atflogbackup_size != 0) {
		lge_panic_handler_reserve_cleanup(atflogbackup_address,
				atflogbackup_size);
		atflogbackup_address = 0;
		atflogbackup_size = 0;

		_lge_set_reserved_4atf(atflogbackup_address, atflogbackup_size);
	}

	if (busdbg_address != 0 && busdbg_size != 0) {
		lge_panic_handler_reserve_cleanup(busdbg_address,
				busdbg_size);
		busdbg_address = 0;
		busdbg_size = 0;

		_lge_set_reserved_4atf(busdbg_address, busdbg_size);
	}

	if (preloader_mem_addr != 0 &&  preloader_mem_size != 0) {
		lge_panic_handler_reserve_cleanup(preloader_mem_addr,
				preloader_mem_size);
		preloader_mem_addr = 0;
		preloader_mem_size = 0;
	}

	if (lk_mem_addr != 0 && lk_mem_addr != 0) {
		lge_panic_handler_reserve_cleanup(lk_mem_addr, lk_mem_size);
		lk_mem_addr = 0;
		lk_mem_size = 0;
	}
}

static int reserved_mem_check(const char *val, struct kernel_param *kp)
{
	int ret;
	unsigned long flags;

	ret = param_set_int(val, kp);
	if (ret) {
		return ret;
	}

	spin_lock_irqsave(&lge_panic_lock, flags);

	release_reserved_mem();

	spin_unlock_irqrestore(&lge_panic_lock, flags);

	return 0;
}
module_param_call(reserved_mem_check, reserved_mem_check,
		param_get_int, &dummy_arg, S_IWUSR | S_IWGRP | S_IRUGO);
#endif

static inline void lge_save_ctx(struct pt_regs *regs)
{
	unsigned int sctrl, ttbr0, ttbr1, ttbcr;
	struct pt_regs context;
	int id, current_cpu;

	asm volatile ("mrc p15, 0, %0, c0, c0, 5 @ Get CPUID\n":"=r"(id));
	current_cpu = (id & 0x3) + ((id & 0xF00) >> 6);

	crash_fp->apinfo.fault_cpu = current_cpu;

	if (regs == NULL) {

		asm volatile ("stmia %1, {r0 - r15}\n\t"
				"mrs %0, cpsr\n"
				:"=r" (context.uregs[16])
				: "r"(&context)
				: "memory");

		/* save cpu register for simulation */
		crash_fp->apinfo.regs[0] = context.ARM_r0;
		crash_fp->apinfo.regs[1] = context.ARM_r1;
		crash_fp->apinfo.regs[2] = context.ARM_r2;
		crash_fp->apinfo.regs[3] = context.ARM_r3;
		crash_fp->apinfo.regs[4] = context.ARM_r4;
		crash_fp->apinfo.regs[5] = context.ARM_r5;
		crash_fp->apinfo.regs[6] = context.ARM_r6;
		crash_fp->apinfo.regs[7] = context.ARM_r7;
		crash_fp->apinfo.regs[8] = context.ARM_r8;
		crash_fp->apinfo.regs[9] = context.ARM_r9;
		crash_fp->apinfo.regs[10] = context.ARM_r10;
		crash_fp->apinfo.regs[11] = context.ARM_fp;
		crash_fp->apinfo.regs[12] = context.ARM_ip;
		crash_fp->apinfo.regs[13] = context.ARM_sp;
		crash_fp->apinfo.regs[14] = context.ARM_lr;
		crash_fp->apinfo.regs[15] = context.ARM_pc;
		crash_fp->apinfo.regs[16] = context.ARM_cpsr;
	} else {
		memcpy(crash_fp->apinfo.regs, regs, sizeof(unsigned int) * 17);
	}


	/*
	 * SCTRL, TTBR0, TTBR1, TTBCR
	 */
	asm volatile ("mrc p15, 0, %0, c1, c0, 0\n" : "=r" (sctrl));
	asm volatile ("mrc p15, 0, %0, c2, c0, 0\n" : "=r" (ttbr0));
	asm volatile ("mrc p15, 0, %0, c2, c0, 1\n" : "=r" (ttbr1));
	asm volatile ("mrc p15, 0, %0, c2, c0, 2\n" : "=r" (ttbcr));

	printk(KERN_INFO "SCTRL: %08x  TTBR0: %08x\n", sctrl, ttbr0);
	printk(KERN_INFO "TTBR1: %08x  TTBCR: %08x\n", ttbr1, ttbcr);

	/* save mmu register for simulation */
	crash_fp->apinfo.regs[17] = sctrl;
	crash_fp->apinfo.regs[18] = ttbr0;
	crash_fp->apinfo.regs[19] = ttbr1;
	crash_fp->apinfo.regs[20] = ttbcr;

	return;
}

static int lge_handler_panic(struct notifier_block *this,
			unsigned long event,
			void *ptr)
{
	unsigned long flags;

	spin_lock_irqsave(&lge_panic_lock, flags);

	printk(KERN_CRIT "%s called\n", __func__);

	lge_save_ctx(NULL);

	if (_lge_get_reboot_reason() == LGE_CRASH_UNKNOWN ||
		_lge_get_reboot_reason() == LGE_REBOOT_REASON_NORMAL) {
		_lge_set_reboot_reason(LGE_CRASH_KERNEL_PANIC);
	}

	spin_unlock_irqrestore(&lge_panic_lock, flags);

	return NOTIFY_DONE;
}

static int lge_handler_die(struct notifier_block *self,
			unsigned long cmd,
			void *ptr)
{
	struct die_args *dargs = (struct die_args *) ptr;
	unsigned long flags;

	spin_lock_irqsave(&lge_panic_lock, flags);

	printk(KERN_CRIT "%s called\n", __func__);

	lge_save_ctx(dargs->regs);

	/*
	 * reboot reason setting..
	 */
	if (_lge_get_reboot_reason() == LGE_CRASH_UNKNOWN ||
		_lge_get_reboot_reason() == LGE_REBOOT_REASON_NORMAL) {
		_lge_set_reboot_reason(LGE_CRASH_KERNEL_OOPS);
	}

	spin_unlock_irqrestore(&lge_panic_lock, flags);

	return NOTIFY_DONE;
}

static int lge_handler_reboot(struct notifier_block *self,
			unsigned long cmd,
			void *ptr)
{
	unsigned long flags;

	spin_lock_irqsave(&lge_panic_lock, flags);

	printk(KERN_CRIT "%s called\n", __func__);

	if (_lge_get_reboot_reason() == LGE_CRASH_UNKNOWN) {
		_lge_set_reboot_reason(LGE_REBOOT_REASON_NORMAL);
	}

	spin_unlock_irqrestore(&lge_panic_lock, flags);

	return NOTIFY_DONE;
}

static struct notifier_block lge_panic_blk = {
	.notifier_call  = lge_handler_panic,
	.priority	= 1004,
};

static struct notifier_block lge_die_blk = {
	.notifier_call	= lge_handler_die,
	.priority	= 1004,
};

static struct notifier_block lge_reboot_blk = {
	.notifier_call	= lge_handler_reboot,
	.priority	= 1004,
};

static void lge_print_mtk_poweroff_reason(void * buffer)
{
	struct _lge_crash_footprint *local_crash_fp = (struct _lge_crash_footprint *) buffer;

	if( local_crash_fp != NULL )
	{
		/*1.UVLO off*/
		pr_err("[pmic_status] TOP_RST_STATUS=0x%x\n", local_crash_fp->pmic_poweroff_reason.TOP_RST_STATUS);
#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
		if (local_crash_fp->pmic_poweroff_reason.TOP_RST_STATUS == 0x0 || local_crash_fp->pmic_poweroff_reason.TOP_RST_STATUS == 0x1 )
			pr_err("[pmic_status] power off_reason is SPAR or battery is removed ");
		if (local_crash_fp->pmic_poweroff_reason.TOP_RST_STATUS == 0xb)
			pr_err("[pmic_status] power off_reason is UVLO");
		if (local_crash_fp->pmic_poweroff_reason.TOP_RST_STATUS == 0x9)
			pr_err("[pmic_status] power off_reason is long press shutdown");
#endif
		/*2.thermal shutdown 150*/
		pr_err("[pmic_status] THERMALSTATUS =0x%x\n",  local_crash_fp->pmic_poweroff_reason.THERMALSTATUS);
		/*3.power not good*/
		pr_err("[pmic_status] PGSTATUS0=0x%x\n",  local_crash_fp->pmic_poweroff_reason.PGSTATUS0 );
		/*4.buck oc*/
		pr_err("[pmic_status] OCSTATUS1=0x%x\n", local_crash_fp->pmic_poweroff_reason.OCSTATUS1);
		pr_err("[pmic_status] OCSTATUS2=0x%x\n",  local_crash_fp->pmic_poweroff_reason.OCSTATUS2);
		/*5.long press shutdown*/
		pr_err("[pmic_status]  STRUP_CON4=0x%x\n", local_crash_fp->pmic_poweroff_reason.STRUP_CON4);
		/*6.WDTRST*/
		pr_err("][pmic_status] TOP_RST_MISC=0x%x\n", local_crash_fp->pmic_poweroff_reason.TOP_RST_MISC);
	}
}
static int __init lge_panic_handler_early_init(void)
{
	size_t start;
	size_t size;
	void *buffer;

	start = LGE_BSP_RAM_CONSOLE_PHY_ADDR;
	size = LGE_BSP_RAM_CONSOLE_SIZE;

	pr_info("LG console start addr : 0x%x\n", (unsigned int) start);
	pr_info("LG console end addr : 0x%x\n", (unsigned int) (start + size));

	buffer = ioremap(start, size);
	if (buffer == NULL) {
		pr_err("lge_panic_handler: failed to map memory\n");
		return -ENOMEM;
	}

	lge_print_mtk_poweroff_reason(buffer);

	memset(buffer, 0x0, size);
	crash_fp = (struct _lge_crash_footprint *) buffer;
	crash_fp->magic = LGE_CONSOLE_MAGIC_KEY;
	_lge_set_reboot_reason(LGE_CRASH_UNKNOWN);

#ifdef CONFIG_OF_RESERVED_MEM
	_lge_set_reserved_4sram(sraminfo_address, sraminfo_size);
	_lge_set_reserved_4atf(atfinfo_address, atfinfo_size);
	_lge_set_reserved_4atflog(atflogbackup_address, atflogbackup_size);
	_lge_set_reserved_4busdbg(busdbg_address, busdbg_size);
#endif

	atomic_notifier_chain_register(&panic_notifier_list, &lge_panic_blk);
	register_die_notifier(&lge_die_blk);
	register_reboot_notifier(&lge_reboot_blk);
#ifdef CONFIG_LGE_HANDLE_PANIC_BY_KEY
	INIT_DELAYED_WORK(&lge_gen_key_panic_timout_work, lge_gen_key_panic_timeout);
	INIT_DELAYED_WORK(&lge_long_key_time_work, lge_long_key_time);
#endif

	return 0;
}
early_initcall(lge_panic_handler_early_init);

static int __init lge_panic_handler_probe(struct platform_device *pdev)
{
	int ret = 0;

	return ret;
}

static int lge_panic_handler_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver panic_handler_driver __refdata = {
	.probe	= lge_panic_handler_probe,
	.remove = lge_panic_handler_remove,
	.driver = {
		.name = PANIC_HANDLER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init lge_panic_handler_init(void)
{
	return platform_driver_register(&panic_handler_driver);
}

static void __exit lge_panic_handler_exit(void)
{
	platform_driver_unregister(&panic_handler_driver);
}

module_init(lge_panic_handler_init);
module_exit(lge_panic_handler_exit);

MODULE_DESCRIPTION("LGE panic handler driver");
MODULE_AUTHOR("SungEun Kim <cleaneye.kim@lge.com>");
MODULE_LICENSE("GPL");
