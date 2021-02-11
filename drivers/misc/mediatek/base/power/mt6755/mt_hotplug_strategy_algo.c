/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

/**
* @file    mt_hotplug_strategy_algo.c
* @brief   hotplug strategy(hps) - algo
*/

/*============================================================================*/
/* Include files */
/*============================================================================*/
/* system includes */
#include <linux/kernel.h>	/* printk */
#include <linux/module.h>	/* MODULE_DESCRIPTION, MODULE_LICENSE */
#include <linux/init.h>		/* module_init, module_exit */
#include <linux/cpu.h>		/* cpu_up */
#include <linux/kthread.h>	/* kthread_create */
#include <linux/wakelock.h>	/* wake_lock_init */
#include <linux/delay.h>	/* msleep */
#include <asm-generic/bug.h>	/* BUG_ON */

/* local includes */
#include "mt_hotplug_strategy_internal.h"

/* forward references */

/*============================================================================*/
/* Macro definition */
/*============================================================================*/
/*
 * static
 */
#define STATIC
/* #define STATIC static */

/*
 * config
 */

/*============================================================================*/
/* Local type definition */
/*============================================================================*/

/*============================================================================*/
/* Local function declarition */
/*============================================================================*/

/*============================================================================*/
/* Local variable definition */
/*============================================================================*/

/*============================================================================*/
/* Global variable definition */
/*============================================================================*/

/*============================================================================*/
/* Local function definition */
/*============================================================================*/

/*============================================================================*/
/* Gobal function definition */
/*============================================================================*/

/* power on_off cpus of cluster*/
/*Use for Fast hotplug.*/
#ifdef CONFIG_CPU_ISOLATION
void hps_algo_cpu_cluster_action(int online_cores, int target_cores,
				 int cpu_id_min, int cpu_id_max,
				 int cluster)
{
	int cpu;
	struct cpumask target_cpu_up_cpumask, target_cpu_down_cpumask, target_cpu_isolate_mask;

	cpumask_clear(&target_cpu_up_cpumask);
	cpumask_clear(&target_cpu_down_cpumask);
	cpumask_copy(&target_cpu_isolate_mask, cpu_isolate_mask);

	if (target_cores > online_cores) {	/*Power upcpus */
		for (cpu = cpu_id_min; cpu <= cpu_id_max; ++cpu) {
			if (!cpumask_test_cpu(cpu, cpu_online_mask)) {	/* For CPU offline */
				cpumask_set_cpu(cpu, &target_cpu_up_cpumask);
				++online_cores;
			} else if (cpumask_test_cpu(cpu, cpu_isolate_mask)) {
				cpumask_clear_cpu(cpu, &target_cpu_isolate_mask);
				++online_cores;
			}
			if (target_cores == online_cores)
				break;
		}
		cpu_up_by_mask(&target_cpu_up_cpumask);
		cpu_isolate_set(&target_cpu_isolate_mask);

	} else {		/*Power downcpus */

		for (cpu = cpu_id_max; cpu >= cpu_id_min; --cpu) {
			if (cpumask_test_cpu(cpu, cpu_online_mask)) {
				if (!cpumask_test_cpu(cpu, cpu_isolate_mask))
					cpumask_set_cpu(cpu, &target_cpu_isolate_mask);
				else
					continue;
				--online_cores;
			}
			if (target_cores == online_cores)
				break;
		}
		cpu_isolate_set(&target_cpu_isolate_mask);
	}
}
#else
/*
*power on/off cpus of cluster
*For legacy hotplug use
*/
static void hps_algo_cpu_cluster_action(int online_cores, int target_cores,
					int cpu_id_min, int cpu_id_max,
					int cluster)
{
	int cpu, rc;

	if ((online_cores < 0) || (target_cores < 0) || (cpu_id_min < 0)
		|| (cpu_id_max < 0) || (cluster < 0))
		return;
	if (target_cores > online_cores) {	/*Power up cpus */
		for (cpu = cpu_id_min; cpu <= cpu_id_max; ++cpu) {
			if (!cpu_online(cpu)) {	/* For CPU offline */
				if (cpu % 4 == 0) {
					rc = hps_cpu_up(cpu);
					if (rc)
						return;
				} else {
					rc  = cpu_up(cpu);
					if (rc)
						return;
				}

				++online_cores;
			}
			if (target_cores == online_cores)
				break;
		}

	} else {		/*Power downcpus */

		for (cpu = cpu_id_max; cpu >= cpu_id_min; --cpu) {
			if (cpu_online(cpu)) {
				rc = cpu_down(cpu);
				if (rc)
					return;

				--online_cores;
			}
			if (target_cores == online_cores)
				break;
		}
	}
}
#endif

/* =============================================================================================== */
/*
* New hotpug strategy
*/

void hps_cal_cores(struct hps_func_data *hps_func)
{
	int cpu;

	switch (hps_func->target_root_cpu) {
	case 0:
		for (cpu = hps_func->base_LL;
		     cpu < hps_func->limit_LL; cpu++, hps_func->target_LL++, hps_func->cores--) {
			if (hps_func->cores <= 0)
				break;
		}
		if (hps_func->cores > 0) {
			for (cpu = hps_func->base_L; cpu < hps_func->limit_L; cpu++) {
				hps_func->target_L++;
				hps_func->cores--;
			}
		}
		set_bit(hps_func->action_LL, (unsigned long *)&hps_ctxt.action);
		break;

	case 4:
		for (cpu = hps_func->base_L;
		     cpu < hps_func->limit_L; cpu++, hps_func->target_L++, hps_func->cores--) {
			if (hps_func->cores <= 0)
				break;
		}
		if (hps_func->cores > 0) {
			for (cpu = hps_func->base_LL; cpu < hps_func->limit_LL; cpu++) {
				hps_func->target_LL++;
				hps_func->cores--;
			}
		}
		set_bit(hps_func->action_L, (unsigned long *)&hps_ctxt.action);
		break;
	default:
		break;
	}
}

#ifdef CONFIG_POWER_AWARENESS_HPS

/* variable for history buffer */
#define LOAD_HISTORY_NUM	32

static unsigned int load_history[LOAD_HISTORY_NUM];
static unsigned int cur_index = 0;
static unsigned int cpufreq_history[LOAD_HISTORY_NUM];

/* variable to determine valid history buffer size */
#define MAX_CORE_LOADS	100
#define MAX_LITTLE_CORE_NUM	4
#define MAX_BIG_CORE_NUM	4
#define MAX_CORE_NUM	(MAX_LITTLE_CORE_NUM + MAX_BIG_CORE_NUM)
#define MAX_LOADS	(MAX_CORE_LOADS * MAX_CORE_NUM)

#define MIN_LOAD_HISTORY_COUNT 8
#define MAX_LOAD_HISTORY_COUNT 64

static unsigned int load_history_count = 32;

/* variable for calculated load */
static unsigned int current_loads;
static unsigned int load_average;
static unsigned int max_loads;
static unsigned int min_loads;

/* variable to determine down treshold value */
#define MIN_HISTORY_DOWN_THRESHOLD (100 - DEF_CPU_DOWN_THRESHOLD)
#define MAX_HISTORY_DOWN_THRESHOLD DEF_CPU_DOWN_THRESHOLD

static unsigned int history_down_threshold = (DEF_CPU_DOWN_THRESHOLD / 2);

/* variable for cpu core frequency */
#define MAX_LITTLE_CORE_FREQUENCY	1001000
#define MAX_BIG_CORE_FREQUENCY		1508000
#define CPUFREQ_NORMALIZATION_FACTOR	100
static unsigned int cpufreq_average;

/* Only tunable value */
#define HOTPLUG_COST_CRITERIA_MS	800

enum down_feedback_value {
	NO_FEEDBACK,
	DOWN,
	NO_DOWN_BY_MAX,
	NO_DOWN_BY_AVERAGE,
};

struct down_feedback_data {
	enum down_feedback_value feedback_value;
	unsigned long feedback_time;
};

static struct down_feedback_data down_feedback;

static inline int get_norm_cpufreq(void)
{
	int i;
	unsigned int norm_little_cpufreq;
	unsigned int norm_big_cpufreq;

	norm_little_cpufreq = 0;

	for (i = 0; i < MAX_LITTLE_CORE_NUM; ++i) {
		if (cpu_online(i)) {
			norm_little_cpufreq = CPUFREQ_NORMALIZATION_FACTOR * cpufreq_quick_get(i) / MAX_LITTLE_CORE_FREQUENCY;
			break;
		}
	}

	norm_big_cpufreq = 0;

	for (i = MAX_LITTLE_CORE_NUM; i < MAX_CORE_NUM; ++i) {
		if (cpu_online(i)) {
			norm_big_cpufreq = CPUFREQ_NORMALIZATION_FACTOR * cpufreq_quick_get(i) / MAX_BIG_CORE_FREQUENCY;
			break;
		}
	}

	if (0 < norm_little_cpufreq && 0 < norm_big_cpufreq)
		return (norm_little_cpufreq + norm_big_cpufreq) / 2;
	else if (0 < norm_little_cpufreq)
		return norm_little_cpufreq;
	else if (0 < norm_big_cpufreq)
		return norm_big_cpufreq;

	return 0;
}

static inline void calculate_load(unsigned int history_num)
{
	unsigned int index;
	unsigned int history_count;

	/* calculate average */
	index = cur_index;

	if (index == 0)
		index = LOAD_HISTORY_NUM - 1;
	else
		--index;

	load_average = load_history[index];
	max_loads = load_history[index];
	min_loads = load_history[index];

	cpufreq_average = cpufreq_history[index];

	history_count = history_num - 1;

	while (index != cur_index && history_count) {
		if (index == 0)
			index = LOAD_HISTORY_NUM - 1;
		else
			--index;

		load_average += load_history[index];

		if (max_loads < load_history[index])
			max_loads = load_history[index];

		if (load_history[index] < min_loads)
			min_loads = load_history[index];

		cpufreq_average += cpufreq_history[index];

		--history_count;
	}

	load_average /= history_num;
	cpufreq_average /= history_num;
}

static void insert_load(unsigned int cur_loads)
{
	unsigned int history_num;

	current_loads = cur_loads;

	load_history[cur_index] = cur_loads;
	cpufreq_history[cur_index] = get_norm_cpufreq();
	++cur_index;
	if (cur_index == LOAD_HISTORY_NUM)
		cur_index = 0;

	history_num = load_average * load_history_count / MAX_LOADS;
	if (history_num < 1)
		history_num = 1;
	else if (LOAD_HISTORY_NUM < history_num)
		history_num = LOAD_HISTORY_NUM;

	calculate_load(history_num);
}

static int judge_down(int little_online_cores, int big_online_cores, int online_cores)
{
	unsigned long current_time;

	if (hps_ctxt.up_loads_sum > hps_ctxt.up_threshold * hps_ctxt.up_times * online_cores) {
		down_feedback.feedback_value = NO_FEEDBACK;
		down_feedback.feedback_time = 0;
		return 0;
	}

	if (online_cores <= 1 && !big_online_cores) {
	       down_feedback.feedback_value = NO_FEEDBACK;
	       down_feedback.feedback_time = 0;
	       return 1;
        }

	/* 1. cpu cluster */
	if (online_cores == MAX_LITTLE_CORE_NUM) {
		down_feedback.feedback_value = NO_FEEDBACK;
		down_feedback.feedback_time = 0;
		return 1;
	}

	current_time = jiffies;

	if (current_loads < min_loads && current_loads + MAX_CORE_LOADS < load_average) {
		if (down_feedback.feedback_value == NO_DOWN_BY_MAX &&
			jiffies_to_msecs(current_time - down_feedback.feedback_time) < HOTPLUG_COST_CRITERIA_MS) {
			--load_history_count;
			if (MIN_LOAD_HISTORY_COUNT < load_history_count)
				load_history_count = MIN_LOAD_HISTORY_COUNT;
		}

		if (down_feedback.feedback_value == NO_DOWN_BY_AVERAGE &&
			jiffies_to_msecs(current_time - down_feedback.feedback_time) < HOTPLUG_COST_CRITERIA_MS) {
			++history_down_threshold;
			if (MAX_HISTORY_DOWN_THRESHOLD < history_down_threshold)
				history_down_threshold = MAX_HISTORY_DOWN_THRESHOLD;
		}

		down_feedback.feedback_value = NO_FEEDBACK;
		down_feedback.feedback_time = 0;

		return 1;
	}

	/* 2. low cpu core frequency */
	if (hps_ctxt.up_threshold * online_cores < max_loads) {
		if (down_feedback.feedback_value != NO_DOWN_BY_MAX &&
			down_feedback.feedback_value != NO_DOWN_BY_AVERAGE)
			down_feedback.feedback_time = current_time;

		down_feedback.feedback_value = NO_DOWN_BY_MAX;

		return 0;
	}

	if (history_down_threshold * online_cores *
		2 * (online_cores) / (online_cores + 1) *
		(cpufreq_average * cpufreq_average) /
		(CPUFREQ_NORMALIZATION_FACTOR * CPUFREQ_NORMALIZATION_FACTOR) <= load_average) {
		if (down_feedback.feedback_value != NO_DOWN_BY_MAX &&
			down_feedback.feedback_value != NO_DOWN_BY_AVERAGE)
			down_feedback.feedback_time = current_time;

		down_feedback.feedback_value = NO_DOWN_BY_AVERAGE;

		return 0;
	}

	down_feedback.feedback_value = DOWN;
	down_feedback.feedback_time = current_time;

	return 1;
}

#ifdef CONFIG_SCHED_NEURAL_NETWORKS
extern void get_cpu_data(unsigned int load);
extern void training_neural_networks(int little_online_cores, int big_online_cores);
extern int run_neural_networks(int little_online_cores, int big_online_cores);

static int judge_down_with_sched_nn(int little_online_cores, int big_online_cores, int online_cores)
{
	int value;

	if (hps_ctxt.up_loads_sum > hps_ctxt.up_threshold * hps_ctxt.up_times * online_cores)
		return 0;

	/* 1. cpu cluster */
	if (MAX_LITTLE_CORE_NUM == online_cores)
		return 1;

	value = run_neural_networks(little_online_cores, big_online_cores);

	if (value < online_cores * MAX_CORE_LOADS)
		return 1;

	return 0;
}
#endif

#endif

void hps_algo_amp(void)
{
	int val, base_val, target_little_cores, target_big_cores;
	struct cpumask little_online_cpumask;
	struct cpumask big_online_cpumask;
	int little_num_base, little_num_limit, little_num_online;
	int big_num_base, big_num_limit, big_num_online;
	int target_root_cpu, state_tran_active;
	struct hps_func_data hps_func;
	/*
	 * run algo or not by hps_ctxt.enabled
	 */
	if (!hps_ctxt.enabled) {
		atomic_set(&hps_ctxt.is_ondemand, 0);
		return;
	}

	/*
	 * algo - begin
	 */
	mutex_lock(&hps_ctxt.lock);
	hps_ctxt.action = ACTION_NONE;
	atomic_set(&hps_ctxt.is_ondemand, 0);

	/*
	 * algo - get boundary
	 */
	little_num_limit = hps_ctxt.little_num_limit_power_serv;
	little_num_base = hps_ctxt.little_num_base_perf_serv;
	cpumask_and(&little_online_cpumask, &hps_ctxt.little_cpumask, cpu_online_mask);

#ifdef CONFIG_CPU_ISOLATION
	cpumask_andnot(&little_online_cpumask, &little_online_cpumask, cpu_isolate_mask);
#endif
	little_num_online = cpumask_weight(&little_online_cpumask);

	big_num_limit = hps_ctxt.big_num_limit_power_serv;
	big_num_base = hps_ctxt.big_num_base_perf_serv;

	cpumask_and(&big_online_cpumask, &hps_ctxt.big_cpumask, cpu_online_mask);

#ifdef CONFIG_CPU_ISOLATION
	cpumask_andnot(&big_online_cpumask, &big_online_cpumask, cpu_isolate_mask);
#endif
	big_num_online = cpumask_weight(&big_online_cpumask);

	base_val = little_num_base + big_num_base;
	target_little_cores = little_num_online;
	target_big_cores = big_num_online;

#ifdef CONFIG_POWER_AWARENESS_HPS
	if (hps_ctxt.pa_hps_enabled == ENABLE_PA_HPS) {
		insert_load(hps_ctxt.cur_loads);
	}
#ifdef CONFIG_SCHED_NEURAL_NETWORKS
	else if (hps_ctxt.pa_hps_enabled == ENABLE_PA_HPS_SCHED_NN) {
		if (little_num_base < 2 && big_num_base < 2) {
			get_cpu_data(hps_ctxt.cur_loads);
			training_neural_networks(little_num_online, big_num_online);
		}
	}
#endif
#endif

	if (hps_ctxt.cur_dump_enabled) {
		hps_debug(
		"loads(%u), hvy_tsk(%u), tlp(%u), iowait(%u), limit_t(%u)(%u), limit_lb(%u)(%u), limit_ups(%u)(%u), limit_pos(%u)(%u), base_pes(%u)(%u)\n",
		     hps_ctxt.cur_loads, hps_ctxt.cur_nr_heavy_task, hps_ctxt.cur_tlp,
		     hps_ctxt.cur_iowait, hps_ctxt.little_num_limit_thermal,
		     hps_ctxt.big_num_limit_thermal, hps_ctxt.little_num_limit_low_battery,
		     hps_ctxt.big_num_limit_low_battery,
		     hps_ctxt.little_num_limit_ultra_power_saving,
		     hps_ctxt.big_num_limit_ultra_power_saving,
		     hps_ctxt.little_num_limit_power_serv, hps_ctxt.big_num_limit_power_serv,
		     hps_ctxt.little_num_base_perf_serv, hps_ctxt.big_num_base_perf_serv);
	}
	/* Determine target_root_cpu */
	target_root_cpu = hps_ctxt.root_cpu;
	state_tran_active = 0;
	if ((hps_ctxt.root_cpu == 0) && (little_num_base == 0) && (big_num_base > 0))
		target_root_cpu = 4;
	if ((hps_ctxt.root_cpu == 4) && (big_num_base == 0) && (little_num_base > 0))
		target_root_cpu = 0;

	if ((hps_ctxt.root_cpu == 0) && (little_num_base == 0) && (little_num_limit == 0))
		target_root_cpu = 4;
	if ((hps_ctxt.root_cpu == 4) && (big_num_base == 0) && (big_num_limit == 0))
		target_root_cpu = 0;
	if (hps_ctxt.root_cpu != target_root_cpu)
		state_tran_active = 1;
	/*
	 * update history - tlp
	 */
	val = hps_ctxt.tlp_history[hps_ctxt.tlp_history_index];
	hps_ctxt.tlp_history[hps_ctxt.tlp_history_index] = hps_ctxt.cur_tlp;
	hps_ctxt.tlp_sum += hps_ctxt.cur_tlp;
	hps_ctxt.tlp_history_index =
	    (hps_ctxt.tlp_history_index + 1 ==
	     hps_ctxt.tlp_times) ? 0 : hps_ctxt.tlp_history_index + 1;
	++hps_ctxt.tlp_count;
	if (hps_ctxt.tlp_count > hps_ctxt.tlp_times) {
		BUG_ON(hps_ctxt.tlp_sum < val);
		hps_ctxt.tlp_sum -= val;
		hps_ctxt.tlp_avg = hps_ctxt.tlp_sum / hps_ctxt.tlp_times;
	} else {
		hps_ctxt.tlp_avg = hps_ctxt.tlp_sum / hps_ctxt.tlp_count;
	}
	if (hps_ctxt.stats_dump_enabled)
		hps_ctxt_print_algo_stats_tlp(0);

	hps_func.limit_LL = hps_ctxt.little_num_limit_power_serv;
	hps_func.limit_L = hps_ctxt.big_num_limit_power_serv;
	hps_func.base_LL = hps_ctxt.little_num_base_perf_serv;
	hps_func.base_L = hps_ctxt.big_num_base_perf_serv;
	hps_func.target_root_cpu = target_root_cpu;
/* ALGO_RUSH_BOOST: */
	/*
	 * algo - rush boost
	 */
	if (hps_ctxt.rush_boost_enabled) {
		if (hps_ctxt.cur_loads >
		    hps_ctxt.rush_boost_threshold * (little_num_online + big_num_online))
			++hps_ctxt.rush_count;
		else
			hps_ctxt.rush_count = 0;
		if (hps_ctxt.rush_boost_times == 1)
			hps_ctxt.tlp_avg = hps_ctxt.cur_tlp;
		if ((hps_ctxt.rush_count >= hps_ctxt.rush_boost_times) &&
		    ((little_num_online + big_num_online) * 100 < hps_ctxt.tlp_avg)) {
			val = hps_ctxt.tlp_avg / 100 + (hps_ctxt.tlp_avg % 100 ? 1 : 0);

			BUG_ON(!(val > little_num_online + big_num_online));
			if (val > num_possible_cpus())
				val = num_possible_cpus();
			target_little_cores = target_big_cores = 0;
			val -= base_val;

			hps_func.cores = val;
			hps_func.action_LL = ACTION_RUSH_BOOST_LITTLE;
			hps_func.action_L = ACTION_RUSH_BOOST_BIG;
			hps_func.target_LL = hps_func.target_L = 0;
			hps_cal_cores(&hps_func);
			target_little_cores = hps_func.target_LL;
			target_big_cores = hps_func.target_L;
#ifdef CONFIG_POWER_AWARENESS_HPS
			if (hps_ctxt.pa_hps_enabled == ENABLE_PA_HPS) {
				if (down_feedback.feedback_value == DOWN &&
					jiffies_to_msecs(jiffies - down_feedback.feedback_time) < HOTPLUG_COST_CRITERIA_MS) {
					++load_history_count;
					if (MAX_LOAD_HISTORY_COUNT < load_history_count)
						load_history_count = MAX_LOAD_HISTORY_COUNT;

					--history_down_threshold;
					if (history_down_threshold < MIN_HISTORY_DOWN_THRESHOLD)
						history_down_threshold = MIN_HISTORY_DOWN_THRESHOLD;
				}

				down_feedback.feedback_value = NO_FEEDBACK;
				down_feedback.feedback_time = 0;
			}
#endif
		}
	}			/* if (hps_ctxt.rush_boost_enabled) */
	if (hps_ctxt.action) {
		target_little_cores += hps_ctxt.little_num_base_perf_serv;
		target_big_cores += hps_ctxt.big_num_base_perf_serv;
		if (!((little_num_online == target_little_cores)
		      && (big_num_online == target_big_cores)))
			goto ALGO_END_WITH_ACTION;
		else
			hps_ctxt.action = ACTION_NONE;
	}

/* ALGO_UP: */
	/*
	 * algo - cpu up
	 */
	if ((little_num_online + big_num_online) < num_possible_cpus()) {

		/*
		 * update history - up
		 */
		val = hps_ctxt.up_loads_history[hps_ctxt.up_loads_history_index];
		hps_ctxt.up_loads_history[hps_ctxt.up_loads_history_index] = hps_ctxt.cur_loads;
		hps_ctxt.up_loads_sum += hps_ctxt.cur_loads;
		hps_ctxt.up_loads_history_index =
		    (hps_ctxt.up_loads_history_index + 1 ==
		     hps_ctxt.up_times) ? 0 : hps_ctxt.up_loads_history_index + 1;
		++hps_ctxt.up_loads_count;
		/* XXX: use >= or >, which is benifit? use > */
		if (hps_ctxt.up_loads_count > hps_ctxt.up_times) {
			BUG_ON(hps_ctxt.up_loads_sum < val);
			hps_ctxt.up_loads_sum -= val;
		}
		if (hps_ctxt.stats_dump_enabled)
			hps_ctxt_print_algo_stats_up(0);
		if (hps_ctxt.up_times == 1)
			hps_ctxt.up_loads_sum = hps_ctxt.cur_loads;
		if (hps_ctxt.up_loads_count >= hps_ctxt.up_times) {
			target_little_cores = target_big_cores = 0;
			if (hps_ctxt.up_loads_sum >
			    hps_ctxt.up_threshold * hps_ctxt.up_times * (little_num_online +
									 big_num_online)) {
				val = little_num_online + big_num_online + 1;
				target_little_cores = target_big_cores = 0;
				val -= base_val;

				hps_func.cores = val;
				hps_func.action_LL = ACTION_UP_LITTLE;
				hps_func.action_L = ACTION_UP_BIG;
				hps_func.target_LL = hps_func.target_L = 0;
				hps_cal_cores(&hps_func);
				target_little_cores = hps_func.target_LL;
				target_big_cores = hps_func.target_L;
#ifdef CONFIG_POWER_AWARENESS_HPS
				if (hps_ctxt.pa_hps_enabled == ENABLE_PA_HPS) {
					if (down_feedback.feedback_value == DOWN &&
						jiffies_to_msecs(jiffies - down_feedback.feedback_time) < HOTPLUG_COST_CRITERIA_MS) {
						++load_history_count;
						if (MAX_LOAD_HISTORY_COUNT < load_history_count)
							load_history_count = MAX_LOAD_HISTORY_COUNT;

						--history_down_threshold;
						if (history_down_threshold < MIN_HISTORY_DOWN_THRESHOLD)
							history_down_threshold = MIN_HISTORY_DOWN_THRESHOLD;
					}

					down_feedback.feedback_value = NO_FEEDBACK;
					down_feedback.feedback_time = 0;
				}
#endif
			}
		}		/* if (hps_ctxt.up_loads_count >= hps_ctxt.up_times) */
	}
	/* if ((little_num_online + big_num_online) < num_possible_cpus()) */
	if (hps_ctxt.action) {
		target_little_cores += hps_ctxt.little_num_base_perf_serv;
		target_big_cores += hps_ctxt.big_num_base_perf_serv;
		if (!((little_num_online == target_little_cores)
		      && (big_num_online == target_big_cores)))
			goto ALGO_END_WITH_ACTION;
		else {
			if (hps_ctxt.up_loads_count > hps_ctxt.up_times)
				goto ALGO_END_WITH_ACTION;
			else
				hps_ctxt.action = ACTION_NONE;
		}
	}
/* ALGO_DOWN: */
	/*
	 * algo - cpu down (inc. quick landing)
	 */
	if (little_num_online + big_num_online > 1) {
		/*
		 * update history - down
		 */
		val = hps_ctxt.down_loads_history[hps_ctxt.down_loads_history_index];
		hps_ctxt.down_loads_history[hps_ctxt.down_loads_history_index] = hps_ctxt.cur_loads;
		hps_ctxt.down_loads_sum += hps_ctxt.cur_loads;
		hps_ctxt.down_loads_history_index =
		    (hps_ctxt.down_loads_history_index + 1 ==
		     hps_ctxt.down_times) ? 0 : hps_ctxt.down_loads_history_index + 1;
		++hps_ctxt.down_loads_count;
		/* XXX: use >= or >, which is benifit? use > */
		if (hps_ctxt.down_loads_count > hps_ctxt.down_times) {
			/*BUG_ON(hps_ctxt.down_loads_sum < val);*/
			hps_ctxt.down_loads_sum -= val;
		}
		if (hps_ctxt.stats_dump_enabled)
			hps_ctxt_print_algo_stats_down(0);
		if (hps_ctxt.down_times == 1)
			hps_ctxt.down_loads_sum = hps_ctxt.cur_loads;
		if (hps_ctxt.down_loads_count >= hps_ctxt.down_times) {
			unsigned int down_threshold = hps_ctxt.down_threshold * hps_ctxt.down_times;

			val = little_num_online + big_num_online;
#ifdef CONFIG_POWER_AWARENESS_HPS
			if (hps_ctxt.pa_hps_enabled == ENABLE_PA_HPS) {
				while ((hps_ctxt.down_loads_sum < down_threshold * (val - 1)) &&
					judge_down(little_num_online, big_num_online, val - 1))
					--val;
			}
#ifdef CONFIG_SCHED_NEURAL_NETWORKS
			else if (hps_ctxt.pa_hps_enabled == ENABLE_PA_HPS_SCHED_NN) {
				while ((hps_ctxt.down_loads_sum < down_threshold * (val - 1)) &&
					((1 < little_num_base || 1 < big_num_base) ||
					judge_down_with_sched_nn(little_num_online, big_num_online, val - 1)))
					--val;
			}
#endif
			else {
				while (hps_ctxt.down_loads_sum < down_threshold * (val - 1))
					--val;
			}
#else
			while (hps_ctxt.down_loads_sum < down_threshold * (val - 1))
				--val;
#endif
			BUG_ON(val < 0);
			target_little_cores = target_big_cores = 0;
			val -= base_val;

			hps_func.cores = val;
			hps_func.action_LL = ACTION_DOWN_LITTLE;
			hps_func.action_L = ACTION_DOWN_BIG;
			hps_func.target_LL = hps_func.target_L = 0;
			hps_cal_cores(&hps_func);
			target_little_cores = hps_func.target_LL;
			target_big_cores = hps_func.target_L;

		}		/* if (hps_ctxt.down_loads_count >= hps_ctxt.down_times) */
	}

	/* if (little_num_online + big_num_online > 1) */
	if (hps_ctxt.action) {
		target_little_cores += hps_ctxt.little_num_base_perf_serv;
		target_big_cores += hps_ctxt.big_num_base_perf_serv;
		if (!((little_num_online == target_little_cores)
		      && (big_num_online == target_big_cores)))
			goto ALGO_END_WITH_ACTION;
		else {
			if (hps_ctxt.down_loads_count > hps_ctxt.down_times)
				goto ALGO_END_WITH_ACTION;
			else
				hps_ctxt.action = ACTION_NONE;
		}
	}
/*ACTION_ROOT_TRAN: */
	/* Process "ONLY" root cpu transition */
	if (state_tran_active) {
		target_little_cores = target_big_cores = 0;
		val = little_num_online + big_num_online;
		val -= base_val;

		hps_func.cores = val;
		hps_func.action_LL = ACTION_ROOT_2_LITTLE;
		hps_func.action_L = ACTION_ROOT_2_BIG;
		hps_func.target_LL = hps_func.target_L = 0;
		hps_cal_cores(&hps_func);
		target_little_cores = hps_func.target_LL;
		target_big_cores = hps_func.target_L;
		state_tran_active = 0;
	}

	if (hps_ctxt.action) {
		target_little_cores += hps_ctxt.little_num_base_perf_serv;
		target_big_cores += hps_ctxt.big_num_base_perf_serv;
		if (!((little_num_online == target_little_cores)
		      && (big_num_online == target_big_cores)))
			goto ALGO_END_WITH_ACTION;
		else
			hps_ctxt.action = ACTION_NONE;
	}

/*Base and limit check*/
	if (target_little_cores < hps_ctxt.little_num_base_perf_serv)
		target_little_cores = hps_ctxt.little_num_base_perf_serv;
	if (target_little_cores > hps_ctxt.little_num_limit_power_serv)
		target_little_cores = hps_ctxt.little_num_limit_power_serv;

	if (target_big_cores < hps_ctxt.big_num_base_perf_serv)
		target_big_cores = hps_ctxt.big_num_base_perf_serv;
	if (target_big_cores > hps_ctxt.big_num_limit_power_serv)
		target_big_cores = hps_ctxt.big_num_limit_power_serv;

	if (!((little_num_online == target_little_cores) && (big_num_online == target_big_cores)))
		goto ALGO_END_WITH_ACTION;
	else
		hps_ctxt.action = ACTION_NONE;


	if (!hps_ctxt.action)
		goto ALGO_END_WO_ACTION;


	/*
	 * algo - end
	 */
ALGO_END_WITH_ACTION:

/*Base and limit check*/
	if (target_little_cores < hps_ctxt.little_num_base_perf_serv)
		target_little_cores = hps_ctxt.little_num_base_perf_serv;
	if (target_little_cores > hps_ctxt.little_num_limit_power_serv)
		target_little_cores = hps_ctxt.little_num_limit_power_serv;

	if (target_big_cores < hps_ctxt.big_num_base_perf_serv)
		target_big_cores = hps_ctxt.big_num_base_perf_serv;
	if (target_big_cores > hps_ctxt.big_num_limit_power_serv)
		target_big_cores = hps_ctxt.big_num_limit_power_serv;

#ifdef CONFIG_MACH_LGE
	{
		extern unsigned int setup_max_cpus;

		// if macpus is used, use only cpus under maxcpus.
		if (NR_CPUS != setup_max_cpus) {
			int lcpus_cnt = cpumask_weight(&hps_ctxt.little_cpumask);

			target_root_cpu = 0;
			if (setup_max_cpus <= lcpus_cnt) {
				// use only little cores
				target_little_cores = setup_max_cpus;
				target_big_cores = 0;
			} else {
				// use all little cores and some big cores
				target_little_cores = lcpus_cnt;
				target_big_cores = setup_max_cpus - lcpus_cnt;
			}
		}
	}
#endif

	if (target_root_cpu == 4) {
		if (target_big_cores == 0)
			target_big_cores++;	/*Root cpu must alive!! */
		/*Process big cluster */
		if (big_num_online != target_big_cores)
			hps_algo_cpu_cluster_action(big_num_online, target_big_cores,
						    hps_ctxt.big_cpu_id_min,
						    hps_ctxt.big_cpu_id_max, 1);

		/*process little cluster */
		if (little_num_online != target_little_cores)
			hps_algo_cpu_cluster_action(little_num_online, target_little_cores,
						    hps_ctxt.little_cpu_id_min,
						    hps_ctxt.little_cpu_id_max, 0);
	} else if (target_root_cpu == 0) {
		if (target_little_cores == 0)
			target_little_cores++;	/*Root cpu must alive!! */

		/*Process little cluster */
		if (little_num_online != target_little_cores)
			hps_algo_cpu_cluster_action(little_num_online, target_little_cores,
						    hps_ctxt.little_cpu_id_min,
						    hps_ctxt.little_cpu_id_max, 0);

		/*process big cluster */
		if (big_num_online != target_big_cores)
			hps_algo_cpu_cluster_action(big_num_online, target_big_cores,
						    hps_ctxt.big_cpu_id_min,
						    hps_ctxt.big_cpu_id_max, 1);
	} else
		hps_warn("ERROR! root cpu %d\n", target_root_cpu);
	if (!get_suspend_status()) {
		if (!((little_num_online == target_little_cores) && (big_num_online == target_big_cores))) {
			hps_debug(
			"END :(%04lx)(%u)(%u) action end(%u)(%u)(%u)(%u) (%u)(%u)(%u)(%u) (%u)(%u)(%u) (%u)(%u)(%u) (%u)(%u)(%u)(%u)(%u) (%u)(%u)(%u)\n",
		hps_ctxt.action, little_num_online, big_num_online, hps_ctxt.cur_loads, hps_ctxt.cur_tlp,
		hps_ctxt.cur_iowait, hps_ctxt.cur_nr_heavy_task, hps_ctxt.little_num_limit_power_serv,
		hps_ctxt.big_num_limit_power_serv, hps_ctxt.little_num_base_perf_serv, hps_ctxt.big_num_base_perf_serv,
		hps_ctxt.up_loads_sum, hps_ctxt.up_loads_count, hps_ctxt.up_loads_history_index,
		hps_ctxt.down_loads_sum, hps_ctxt.down_loads_count, hps_ctxt.down_loads_history_index,
		hps_ctxt.rush_count, hps_ctxt.tlp_sum, hps_ctxt.tlp_count, hps_ctxt.tlp_history_index,
		hps_ctxt.tlp_avg, target_root_cpu, target_little_cores,	target_big_cores);
		}
	}
	hps_ctxt_reset_stas_nolock();

ALGO_END_WO_ACTION:
	if ((hps_ctxt.up_times == 1) || (hps_ctxt.down_times==1))
		hps_ctxt_reset_stas_nolock();
	mutex_unlock(&hps_ctxt.lock);
}
