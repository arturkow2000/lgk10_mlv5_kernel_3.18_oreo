/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#include <linux/atomic.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <asm-generic/sizes.h>
#include <linux/mtk_rtb.h>

#define SENTINEL_BYTE_1 0xFF
#define SENTINEL_BYTE_2 0xAA
#define SENTINEL_BYTE_3 0xFF

#define RTB_COMPAT_STR	"mtk,mtk-rtb"

/* Write
 * 1) 3 bytes sentinel
 * 2) 1 bytes of log type
 * 3) 8 bytes of where the caller came from
 * 4) 4 bytes index
 * 4) 8 bytes extra data from the caller
 * 5) 8 bytes of timestamp
 *
 * Total = 32 bytes.
 */
struct mtk_rtb_layout {
	unsigned char sentinel[3];
	unsigned char log_type;
	uint32_t idx;
	uint64_t caller;
	uint64_t data;
	uint64_t timestamp;
} __attribute__ ((__packed__));


struct mtk_rtb_state {
	struct mtk_rtb_layout *rtb;
	phys_addr_t phys;
	int nentries;
	int size;
	int enabled;
	int initialized;
	uint32_t filter;
	int step_size;
};

#if defined(CONFIG_LGE_MTK_RTB_SEPARATE_CPUS)
DEFINE_PER_CPU(atomic_t, mtk_rtb_idx_cpu);
#else
static atomic_t mtk_rtb_idx;
#endif

static struct mtk_rtb_state mtk_rtb = {
	.filter = 1 << LOGK_LOGBUF,
	.enabled = 1,
};

module_param_named(filter, mtk_rtb.filter, uint, 0644);
module_param_named(enable, mtk_rtb.enabled, int, 0644);

static int mtk_rtb_panic_notifier(struct notifier_block *this,
					unsigned long event, void *ptr)
{
	mtk_rtb.enabled = 0;
	return NOTIFY_DONE;
}

static struct notifier_block mtk_rtb_panic_blk = {
	.notifier_call  = mtk_rtb_panic_notifier,
};

void notrace mtk_rtb_set_filter(uint filter)
{
	mtk_rtb.filter = filter;
}

int notrace mtk_rtb_event_should_log(enum logk_event_type log_type)
{
	return mtk_rtb.initialized && mtk_rtb.enabled &&
		((1 << (log_type & ~LOGTYPE_NOPC)) & mtk_rtb.filter);
}
EXPORT_SYMBOL(mtk_rtb_event_should_log);

static void mtk_rtb_emit_sentinel(struct mtk_rtb_layout *start)
{
	start->sentinel[0] = SENTINEL_BYTE_1;
	start->sentinel[1] = SENTINEL_BYTE_2;
	start->sentinel[2] = SENTINEL_BYTE_3;
}

static void mtk_rtb_write_type(enum logk_event_type log_type,
			struct mtk_rtb_layout *start)
{
	start->log_type = (char)log_type;
}

static void mtk_rtb_write_caller(uint64_t caller, struct mtk_rtb_layout *start)
{
	start->caller = caller;
}

static void mtk_rtb_write_idx(uint32_t idx,
				struct mtk_rtb_layout *start)
{
	start->idx = idx;
}

static void mtk_rtb_write_data(uint64_t data, struct mtk_rtb_layout *start)
{
	start->data = data;
}

static void mtk_rtb_write_timestamp(struct mtk_rtb_layout *start)
{
	start->timestamp = sched_clock();
}

static void uncached_logk_pc_idx(enum logk_event_type log_type, uint64_t caller,
				 uint64_t data, int idx)
{
	struct mtk_rtb_layout *start;

	start = &mtk_rtb.rtb[idx & (mtk_rtb.nentries - 1)];

	mtk_rtb_emit_sentinel(start);
	mtk_rtb_write_type(log_type, start);
	mtk_rtb_write_caller(caller, start);
	mtk_rtb_write_idx(idx, start);
	mtk_rtb_write_data(data, start);
	mtk_rtb_write_timestamp(start);
	mb();

	return;
}

static void uncached_logk_timestamp(int idx)
{
	unsigned long long timestamp;

	timestamp = sched_clock();
	uncached_logk_pc_idx(LOGK_TIMESTAMP|LOGTYPE_NOPC,
			(uint64_t)lower_32_bits(timestamp),
			(uint64_t)upper_32_bits(timestamp), idx);
}

#if defined(CONFIG_LGE_MTK_RTB_SEPARATE_CPUS)
/*
 * Since it is not necessarily true that nentries % step_size == 0,
 * must make appropriate adjustments to the index when a "wraparound"
 * occurs to ensure that mtk_rtb.rtb[x] always belongs to the same cpu.
 * It is desired to give all cpus the same number of entries; this leaves
 * (nentries % step_size) dead space at the end of the buffer.
 */
static int mtk_rtb_get_idx(void)
{
	int cpu, i, offset;
	atomic_t *index;
	unsigned long flags;
	u32 unused_buffer_size = mtk_rtb.nentries % mtk_rtb.step_size;
	int adjusted_size;

	/*
	 * ideally we would use get_cpu but this is a close enough
	 * approximation for our purposes.
	 */
	cpu = raw_smp_processor_id();

	index = &per_cpu(mtk_rtb_idx_cpu, cpu);

	local_irq_save(flags);
	i = atomic_add_return(mtk_rtb.step_size, index);
	i -= mtk_rtb.step_size;

	/*
	 * Check if index has wrapped around or is in the unused region at the
	 * end of the buffer
	 */
	adjusted_size = atomic_read(index) + unused_buffer_size;
	offset = (adjusted_size & (mtk_rtb.nentries - 1)) -
		 ((adjusted_size - mtk_rtb.step_size) & (mtk_rtb.nentries - 1));
	if (offset < 0) {
		uncached_logk_timestamp(i);
		i = atomic_add_return(mtk_rtb.step_size + unused_buffer_size,
									index);
		i -= mtk_rtb.step_size;
	}
	local_irq_restore(flags);

	return i;
}
#else
static int mtk_rtb_get_idx(void)
{
	int i, offset;

	i = atomic_inc_return(&mtk_rtb_idx);
	i--;

	/* Check if index has wrapped around */
	offset = (i & (mtk_rtb.nentries - 1)) -
		 ((i - 1) & (mtk_rtb.nentries - 1));
	if (offset < 0) {
		uncached_logk_timestamp(i);
		i = atomic_inc_return(&mtk_rtb_idx);
		i--;
	}

	return i;
}
#endif

int notrace uncached_logk_pc(enum logk_event_type log_type, void *caller,
				void *data)
{
	int i;

	if (!mtk_rtb_event_should_log(log_type))
		return 0;

	i = mtk_rtb_get_idx();
	uncached_logk_pc_idx(log_type, (uint64_t)((unsigned long) caller),
				(uint64_t)((unsigned long) data), i);

	return 1;
}
EXPORT_SYMBOL(uncached_logk_pc);

noinline int notrace uncached_logk(enum logk_event_type log_type, void *data)
{
	return uncached_logk_pc(log_type, __builtin_return_address(0), data);
}
EXPORT_SYMBOL(uncached_logk);

static int mtk_rtb_probe(struct platform_device *pdev)
{
	struct mtk_rtb_platform_data *d = pdev->dev.platform_data;
#if defined(CONFIG_LGE_MTK_RTB_SEPARATE_CPUS)
	unsigned int cpu;
#endif
	int ret;

	if (!pdev->dev.of_node) {
		mtk_rtb.size = d->size;
	} else {
		u64 size;
		struct device_node *pnode;

		pnode = of_parse_phandle(pdev->dev.of_node,
						"linux,contiguous-region", 0);
		if (pnode != NULL) {
			const u32 *addr;

			addr = of_get_address(pnode, 0, &size, NULL);
			if (!addr) {
				of_node_put(pnode);
				return -EINVAL;
			}
			of_node_put(pnode);
		} else {
			ret = of_property_read_u32(pdev->dev.of_node,
					"mtk,rtb-size",
					(u32 *)&size);
			if (ret < 0)
				return ret;

		}

		mtk_rtb.size = size;
	}

	if (mtk_rtb.size <= 0 || mtk_rtb.size > SZ_1M)
		return -EINVAL;

	mtk_rtb.rtb = dma_alloc_coherent(&pdev->dev, mtk_rtb.size,
						&mtk_rtb.phys,
						GFP_KERNEL);

	if (!mtk_rtb.rtb)
		return -ENOMEM;

	mtk_rtb.nentries = mtk_rtb.size / sizeof(struct mtk_rtb_layout);

	/* Round this down to a power of 2 */
	mtk_rtb.nentries = __rounddown_pow_of_two(mtk_rtb.nentries);

	memset(mtk_rtb.rtb, 0, mtk_rtb.size);


#if defined(CONFIG_LGE_MTK_RTB_SEPARATE_CPUS)
	for_each_possible_cpu(cpu) {
		atomic_t *a = &per_cpu(mtk_rtb_idx_cpu, cpu);
		atomic_set(a, cpu);
	}
	mtk_rtb.step_size = num_possible_cpus();
#else
	atomic_set(&mtk_rtb_idx, 0);
	mtk_rtb.step_size = 1;
#endif

	atomic_notifier_chain_register(&panic_notifier_list,
						&mtk_rtb_panic_blk);
	mtk_rtb.initialized = 1;
	return 0;
}

static struct of_device_id mtk_match_table[] = {
	{.compatible = RTB_COMPAT_STR},
	{},
};

static struct platform_driver mtk_rtb_driver = {
	.driver         = {
		.name = "mtk_rtb",
		.owner = THIS_MODULE,
		.of_match_table = mtk_match_table
	},
};

static int __init mtk_rtb_init(void)
{
	return platform_driver_probe(&mtk_rtb_driver, mtk_rtb_probe);
}

static void __exit mtk_rtb_exit(void)
{
	platform_driver_unregister(&mtk_rtb_driver);
}
module_init(mtk_rtb_init)
module_exit(mtk_rtb_exit)
