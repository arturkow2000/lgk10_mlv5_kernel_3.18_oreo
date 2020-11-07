#undef TRACE_SYSTEM
#define TRACE_SYSTEM readahead

#if !defined(_TRACE_READAHEAD_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_READAHEAD_H

#include <linux/types.h>
#include <linux/tracepoint.h>
#include <linux/mm.h>
#include <linux/memcontrol.h>
#include <linux/device.h>
#include <linux/kdev_t.h>

DECLARE_EVENT_CLASS(mm_readahead_ondemand_readahead,

	TP_PROTO(unsigned long ra_size,
		unsigned long tot_ra_size,
		unsigned long tot_ref_size,
		unsigned int ref_ratio,
		const char* file_name),

	TP_ARGS(ra_size, tot_ra_size, tot_ref_size, ref_ratio, file_name),

	TP_STRUCT__entry(
		__field(unsigned long, ra_size)
		__field(unsigned long, tot_ra_size)
		__field(unsigned long, tot_ref_size)
		__field(unsigned int, ref_ratio)
		__field(const char*, file_name)
	),

	TP_fast_assign(
		__entry->ra_size = ra_size;
		__entry->tot_ra_size = tot_ra_size;
		__entry->tot_ref_size = tot_ref_size;
		__entry->ref_ratio = ref_ratio;
		__entry->file_name = file_name;
	),

	TP_printk("[PSW] ra_size %ld tot_ra_size %ld tot_ref_size %ld ref_ratio %d file %s",
		__entry->ra_size,
		__entry->tot_ra_size,
		__entry->tot_ref_size,
		__entry->ref_ratio,
		__entry->file_name)
);

DEFINE_EVENT(mm_readahead_ondemand_readahead, ondemand_sequential_readahead,
	TP_PROTO(unsigned long ra_size,
		unsigned long tot_ra_size,
		unsigned long tot_ref_size,
		unsigned int ref_ratio,
		const char* file_name),

	TP_ARGS(ra_size, tot_ra_size, tot_ref_size, ref_ratio, file_name)
);

DEFINE_EVENT(mm_readahead_ondemand_readahead, ondemand_random_readahead,
	TP_PROTO(unsigned long ra_size,
		unsigned long tot_ra_size,
		unsigned long tot_ref_size,
		unsigned int ref_ratio,
		const char* file_name),

	TP_ARGS(ra_size, tot_ra_size, tot_ref_size, ref_ratio, file_name)
);

#endif /* _TRACE_READAHEAD_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
