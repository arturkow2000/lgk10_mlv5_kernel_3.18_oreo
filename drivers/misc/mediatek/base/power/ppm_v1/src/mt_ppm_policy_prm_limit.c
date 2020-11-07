/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>

#include "mt_ppm_internal.h"


static enum ppm_power_state ppm_prmlimit_get_power_state_cb(enum ppm_power_state cur_state);
static void ppm_prmlimit_update_limit_cb(enum ppm_power_state new_state);
static void ppm_prmlimit_status_change_cb(bool enable);
static void ppm_prmlimit_mode_change_cb(enum ppm_mode mode);

/* other members will init by ppm_main */
static struct ppm_policy_data prmlimit_policy = {
	.name			= __stringify(PPM_POLICY_PRM_LIMIT),
	.lock			= __MUTEX_INITIALIZER(prmlimit_policy.lock),
	.policy			= PPM_POLICY_PRM_LIMIT,
	.priority		= PPM_POLICY_PRIO_PRM_SPECIFY_BASE,
	.get_power_state_cb	= ppm_prmlimit_get_power_state_cb,
	.update_limit_cb	= ppm_prmlimit_update_limit_cb,
	.status_change_cb	= ppm_prmlimit_status_change_cb,
	.mode_change_cb		= ppm_prmlimit_mode_change_cb,
};

struct ppm_prmlimit_data prmlimit_data = {
	.is_freq_limited_by_prm = false,
	.is_core_limited_by_prm = false,
};


/* MUST in lock */
bool ppm_prmlimit_is_policy_active(void)
{
	if (!prmlimit_data.is_freq_limited_by_prm && !prmlimit_data.is_core_limited_by_prm)
		return false;
	else
		return true;
}

static enum ppm_power_state ppm_prmlimit_get_power_state_cb(enum ppm_power_state cur_state)
{
	if (prmlimit_data.is_core_limited_by_prm)
		return ppm_judge_state_by_prm_limit(cur_state, prmlimit_data);
	else
		return cur_state;
}

static void ppm_prmlimit_update_limit_cb(enum ppm_power_state new_state)
{
	unsigned int i;
	struct ppm_policy_req *req = &prmlimit_policy.req;

	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: prmlimit policy update limit for new state = %s\n",
		__func__, ppm_get_power_state_name(new_state));

	if (prmlimit_data.is_freq_limited_by_prm || prmlimit_data.is_core_limited_by_prm) {
		ppm_hica_set_default_limit_by_state(new_state, &prmlimit_policy);

		for (i = 0; i < req->cluster_num; i++) {
			req->limit[i].min_cpu_core = (prmlimit_data.limit[i].min_core_num == -1)
				? req->limit[i].min_cpu_core
				: prmlimit_data.limit[i].min_core_num;
			req->limit[i].max_cpu_core = (prmlimit_data.limit[i].max_core_num == -1)
				? req->limit[i].max_cpu_core
				: prmlimit_data.limit[i].max_core_num;
			req->limit[i].min_cpufreq_idx = (prmlimit_data.limit[i].min_freq_idx == -1)
				? req->limit[i].min_cpufreq_idx
				: prmlimit_data.limit[i].min_freq_idx;
			req->limit[i].max_cpufreq_idx = (prmlimit_data.limit[i].max_freq_idx == -1)
				? req->limit[i].max_cpufreq_idx
				: prmlimit_data.limit[i].max_freq_idx;
		}

		ppm_limit_check_for_prm_limit(new_state, req, prmlimit_data);

		/* error check */
		for (i = 0; i < req->cluster_num; i++) {
			if (req->limit[i].max_cpu_core < req->limit[i].min_cpu_core)
				req->limit[i].min_cpu_core = req->limit[i].max_cpu_core;
			if (req->limit[i].max_cpufreq_idx > req->limit[i].min_cpufreq_idx)
				req->limit[i].min_cpufreq_idx = req->limit[i].max_cpufreq_idx;
		}
	}

	FUNC_EXIT(FUNC_LV_POLICY);
}

static void ppm_prmlimit_status_change_cb(bool enable)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: prmlimit policy status changed to %d\n", __func__, enable);

	FUNC_EXIT(FUNC_LV_POLICY);
}

static void ppm_prmlimit_mode_change_cb(enum ppm_mode mode)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: ppm mode changed to %d\n", __func__, mode);

	FUNC_EXIT(FUNC_LV_POLICY);
}

unsigned int mt_ppm_prmlimit_cpu_core(unsigned int cluster_num, struct ppm_limit_data *data)
{
	int i = 0;
	int min_core, max_core;
	bool is_limit = false;

	/* Error check */
	if (cluster_num > NR_PPM_CLUSTERS) {
		ppm_err("@%s: Invalid cluster num = %d\n", __func__, cluster_num);
		return -1;
	}

	if (!data) {
		ppm_err("@%s: limit data is NULL!\n", __func__);
		return -1;
	}

	for (i = 0; i < cluster_num; i++) {
		min_core = data[i].min;
		max_core = data[i].max;

		/* invalid input check */
		if (min_core != -1 && min_core < (int)get_cluster_min_cpu_core(i)) {
			ppm_err("@%s: Invalid input! min_core for cluster %d = %d\n", __func__, i, min_core);
			return -1;
		}
		if (max_core != -1 && max_core > (int)get_cluster_max_cpu_core(i)) {
			ppm_err("@%s: Invalid input! max_core for cluster %d = %d\n", __func__, i, max_core);
			return -1;
		}

#ifdef PPM_IC_SEGMENT_CHECK
		if (!max_core) {
			if ((i == 0 && ppm_main_info.fix_state_by_segment == PPM_POWER_STATE_LL_ONLY)
					|| (i == 1 && ppm_main_info.fix_state_by_segment == PPM_POWER_STATE_L_ONLY)) {
				ppm_err("@%s: Cannot disable cluster %d due to fix_state_by_segment is %s\n",
					__func__, i, ppm_get_power_state_name(ppm_main_info.fix_state_by_segment));
				return -1;
			}
		}
#endif

		/* check is all limit clear or not */
		if (min_core != -1 || max_core != -1)
			is_limit = true;

		/* sync to max_core if min > max */
		if (min_core != -1 && max_core != -1 && min_core > max_core)
			data[i].min = data[i].max;
	}

	ppm_lock(&prmlimit_policy.lock);
	if (!prmlimit_policy.is_enabled) {
		ppm_warn("@%s: prmlimit policy is not enabled!\n", __func__);
		ppm_unlock(&prmlimit_policy.lock);
		return -1;
	}

	/* update policy data */
	for (i = 0; i < cluster_num; i++) {
		min_core = data[i].min;
		max_core = data[i].max;

		if (min_core != prmlimit_data.limit[i].min_core_num ||
				max_core != prmlimit_data.limit[i].max_core_num) {
			prmlimit_data.limit[i].min_core_num = min_core;
			prmlimit_data.limit[i].max_core_num = max_core;
			ppm_dbg(ALL, "update prmlimit min/max core for cluster %d = %d/%d\n",
					i, min_core, max_core);
		}
	}

	prmlimit_data.is_core_limited_by_prm = is_limit;
	prmlimit_policy.is_activated = ppm_prmlimit_is_policy_active();

	ppm_unlock(&prmlimit_policy.lock);
	mt_ppm_main();

	return 0;
}

unsigned int mt_ppm_prmlimit_cpu_freq(unsigned int cluster_num, struct ppm_limit_data *data)
{
	int i = 0;
	int min_freq, max_freq, min_freq_idx, max_freq_idx;
	bool is_limit = false;

	/* Error check */
	if (cluster_num > NR_PPM_CLUSTERS) {
		ppm_err("@%s: Invalid cluster num = %d\n", __func__, cluster_num);
		return -1;
	}

	if (!data) {
		ppm_err("@%s: limit data is NULL!\n", __func__);
		return -1;
	}

	ppm_lock(&prmlimit_policy.lock);
	if (!prmlimit_policy.is_enabled) {
		ppm_warn("@%s: prmlimit policy is not enabled!\n", __func__);
		ppm_unlock(&prmlimit_policy.lock);
		return -1;
	}

	/* update policy data */
	for (i = 0; i < cluster_num; i++) {
		min_freq = data[i].min;
		max_freq = data[i].max;

		/* check is all limit clear or not */
		if (min_freq != -1 || max_freq != -1)
			is_limit = true;

		/* freq to idx */
		min_freq_idx = (min_freq == -1) ? -1
			: ppm_main_freq_to_idx(i, min_freq, CPUFREQ_RELATION_L);
		max_freq_idx = (max_freq == -1) ? -1
			: ppm_main_freq_to_idx(i, max_freq, CPUFREQ_RELATION_H);

		/* sync to max_freq if min < max */
		if (min_freq_idx != -1 && max_freq_idx != -1 && min_freq_idx < max_freq_idx)
			min_freq_idx = max_freq_idx;

		/* write to policy data */
		if (min_freq_idx != prmlimit_data.limit[i].min_freq_idx ||
				max_freq_idx != prmlimit_data.limit[i].max_freq_idx) {
			prmlimit_data.limit[i].min_freq_idx = min_freq_idx;
			prmlimit_data.limit[i].max_freq_idx = max_freq_idx;
			ppm_dbg(ALL, "update prm limit min/max freq for cluster %d = %d(%d)/%d(%d)\n",
					i, min_freq, min_freq_idx, max_freq, max_freq_idx);
		}
	}

	prmlimit_data.is_freq_limited_by_prm = is_limit;
	prmlimit_policy.is_activated = ppm_prmlimit_is_policy_active();

	ppm_unlock(&prmlimit_policy.lock);
	mt_ppm_main();

	return 0;
}

static int ppm_prmlimit_min_cpu_core_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < prmlimit_policy.req.cluster_num; i++) {
		seq_printf(m, "cluster %d: min_core_num = %d, max_core_num = %d\n",
			i, prmlimit_data.limit[i].min_core_num, prmlimit_data.limit[i].max_core_num);
	}

	return 0;
}

static ssize_t ppm_prmlimit_min_cpu_core_proc_write(struct file *file, const char __user *buffer,
					size_t count,	loff_t *pos)
{
	int id, min_core, i;
	bool is_limit = false;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &id, &min_core) == 2) {
		if (id < 0 || id >= ppm_main_info.cluster_num) {
			ppm_err("@%s: Invalid cluster id: %d\n", __func__, id);
			goto out;
		}

		if (min_core != -1 && min_core < (int)get_cluster_min_cpu_core(id)) {
			ppm_err("@%s: Invalid input! min_core = %d\n", __func__, min_core);
			goto out;
		}

		ppm_lock(&prmlimit_policy.lock);

		if (!prmlimit_policy.is_enabled) {
			ppm_warn("@%s: prmlimit policy is not enabled!\n", __func__);
			ppm_unlock(&prmlimit_policy.lock);
			goto out;
		}

		/* error check */
		if (prmlimit_data.limit[id].max_core_num != -1
			&& min_core != -1
			&& min_core > prmlimit_data.limit[id].max_core_num) {
			ppm_warn("@%s: min_core(%d) > max_core(%d), sync to max core!\n",
				__func__, min_core, prmlimit_data.limit[id].max_core_num);
			min_core = prmlimit_data.limit[id].max_core_num;
		}

		if (min_core != prmlimit_data.limit[id].min_core_num) {
			prmlimit_data.limit[id].min_core_num = min_core;
			ppm_dbg(ALL, "prm limit min_core_num = %d for cluster %d\n", min_core, id);
		}

		/* check is core limited or not */
		for (i = 0; i < prmlimit_policy.req.cluster_num; i++) {
			if (prmlimit_data.limit[i].min_core_num != -1
				|| prmlimit_data.limit[i].max_core_num != -1) {
				is_limit = true;
				break;
			}
		}
		prmlimit_data.is_core_limited_by_prm = is_limit;

		prmlimit_policy.is_activated = ppm_prmlimit_is_policy_active();

		ppm_unlock(&prmlimit_policy.lock);
		ppm_task_wakeup();
	} else
		ppm_err("@%s: Invalid input!\n", __func__);

out:
	free_page((unsigned long)buf);
	return count;
}

static int ppm_prmlimit_max_cpu_core_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < prmlimit_policy.req.cluster_num; i++) {
		seq_printf(m, "cluster %d: min_core_num = %d, max_core_num = %d\n",
			i, prmlimit_data.limit[i].min_core_num, prmlimit_data.limit[i].max_core_num);
	}

	return 0;
}

static ssize_t ppm_prmlimit_max_cpu_core_proc_write(struct file *file, const char __user *buffer,
					size_t count,	loff_t *pos)
{
	int id, max_core, i;
	bool is_limit = false;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &id, &max_core) == 2) {
		if (id < 0 || id >= ppm_main_info.cluster_num) {
			ppm_err("@%s: Invalid cluster id: %d\n", __func__, id);
			goto out;
		}

		if (max_core != -1 && max_core > (int)get_cluster_max_cpu_core(id)) {
			ppm_err("@%s: Invalid input! max_core = %d\n", __func__, max_core);
			goto out;
		}

#ifdef PPM_IC_SEGMENT_CHECK
		if (!max_core) {
			if ((id == 0 && ppm_main_info.fix_state_by_segment == PPM_POWER_STATE_LL_ONLY)
				|| (id == 1 && ppm_main_info.fix_state_by_segment == PPM_POWER_STATE_L_ONLY)) {
				ppm_err("@%s: Cannot disable cluster %d due to fix_state_by_segment is %s\n",
					__func__, id, ppm_get_power_state_name(ppm_main_info.fix_state_by_segment));
				goto out;
			}
		}
#endif

		ppm_lock(&prmlimit_policy.lock);

		if (!prmlimit_policy.is_enabled) {
			ppm_warn("@%s: prmlimit policy is not enabled!\n", __func__);
			ppm_unlock(&prmlimit_policy.lock);
			goto out;
		}

		/* error check */
		if (prmlimit_data.limit[id].min_core_num != -1
			&& max_core != -1
			&& max_core < prmlimit_data.limit[id].min_core_num) {
			ppm_warn("@%s: max_core(%d) < min_core(%d), overwrite min core!\n",
				__func__, max_core, prmlimit_data.limit[id].min_core_num);
			prmlimit_data.limit[id].min_core_num = max_core;
		}

		if (max_core != prmlimit_data.limit[id].max_core_num) {
			prmlimit_data.limit[id].max_core_num = max_core;
			ppm_dbg(ALL, "prm limit max_core_num = %d for cluster %d\n", max_core, id);
		}

		/* check is core limited or not */
		for (i = 0; i < prmlimit_policy.req.cluster_num; i++) {
			if (prmlimit_data.limit[i].min_core_num != -1
				|| prmlimit_data.limit[i].max_core_num != -1) {
				is_limit = true;
				break;
			}
		}
		prmlimit_data.is_core_limited_by_prm = is_limit;

		prmlimit_policy.is_activated = ppm_prmlimit_is_policy_active();

		ppm_unlock(&prmlimit_policy.lock);
		ppm_task_wakeup();
	} else
		ppm_err("@%s: Invalid input!\n", __func__);

out:
	free_page((unsigned long)buf);
	return count;
}

static int ppm_prmlimit_min_cpu_freq_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < prmlimit_policy.req.cluster_num; i++) {
		seq_printf(m, "cluster %d: min_freq_idx = %d, max_freq_idx = %d\n",
			i, prmlimit_data.limit[i].min_freq_idx, prmlimit_data.limit[i].max_freq_idx);
	}

	return 0;
}

static ssize_t ppm_prmlimit_min_cpu_freq_proc_write(struct file *file, const char __user *buffer,
					size_t count, loff_t *pos)
{
	int id, min_freq, idx, i;
	bool is_limit = false;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &id, &min_freq) == 2) {
		if (id < 0 || id >= ppm_main_info.cluster_num) {
			ppm_err("@%s: Invalid cluster id: %d\n", __func__, id);
			goto out;
		}

		ppm_lock(&prmlimit_policy.lock);

		if (!prmlimit_policy.is_enabled) {
			ppm_warn("@%s: prmlimit policy is not enabled!\n", __func__);
			ppm_unlock(&prmlimit_policy.lock);
			goto out;
		}

		idx = (min_freq == -1) ? -1 : ppm_main_freq_to_idx(id, min_freq, CPUFREQ_RELATION_L);

		/* error check, sync to max idx if min freq > max freq */
		if (prmlimit_data.limit[id].max_freq_idx != -1
			&& idx != -1
			&& idx < prmlimit_data.limit[id].max_freq_idx)
			idx = prmlimit_data.limit[id].max_freq_idx;

		if (idx != prmlimit_data.limit[id].min_freq_idx) {
			prmlimit_data.limit[id].min_freq_idx = idx;
			ppm_dbg(ALL, "prm limit min_freq = %d KHz(idx = %d) for cluster %d\n",
				min_freq, idx, id);
		}

		/* check is freq limited or not */
		for (i = 0; i < prmlimit_policy.req.cluster_num; i++) {
			if (prmlimit_data.limit[i].min_freq_idx != -1
				|| prmlimit_data.limit[i].max_freq_idx != -1) {
				is_limit = true;
				break;
			}
		}
		prmlimit_data.is_freq_limited_by_prm = is_limit;

		prmlimit_policy.is_activated = ppm_prmlimit_is_policy_active();

		ppm_unlock(&prmlimit_policy.lock);
		ppm_task_wakeup();
	} else
		ppm_err("@%s: Invalid input!\n", __func__);

out:
	free_page((unsigned long)buf);
	return count;
}

static int ppm_prmlimit_max_cpu_freq_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < prmlimit_policy.req.cluster_num; i++) {
		seq_printf(m, "cluster %d: min_freq_idx = %d, max_freq_idx = %d\n",
			i, prmlimit_data.limit[i].min_freq_idx, prmlimit_data.limit[i].max_freq_idx);
	}

	return 0;
}

static ssize_t ppm_prmlimit_max_cpu_freq_proc_write(struct file *file, const char __user *buffer,
					size_t count, loff_t *pos)
{
	int id, max_freq, idx, i;
	bool is_limit = false;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &id, &max_freq) == 2) {
		if (id < 0 || id >= ppm_main_info.cluster_num) {
			ppm_err("@%s: Invalid cluster id: %d\n", __func__, id);
			goto out;
		}

		ppm_lock(&prmlimit_policy.lock);

		if (!prmlimit_policy.is_enabled) {
			ppm_warn("@%s: prmlimit policy is not enabled!\n", __func__);
			ppm_unlock(&prmlimit_policy.lock);
			goto out;
		}

		idx = (max_freq == -1) ? -1 : ppm_main_freq_to_idx(id, max_freq, CPUFREQ_RELATION_H);

		/* error check, sync to max idx if max freq < min freq */
		if (prmlimit_data.limit[id].min_freq_idx != -1
			&& idx != -1
			&& idx > prmlimit_data.limit[id].min_freq_idx)
			prmlimit_data.limit[id].min_freq_idx = idx;

		if (idx != prmlimit_data.limit[id].max_freq_idx) {
			prmlimit_data.limit[id].max_freq_idx = idx;
			ppm_dbg(ALL, "prm limit max_freq = %d KHz(idx = %d) for cluster %d\n",
				max_freq, idx, id);
		}

		/* check is freq limited or not */
		for (i = 0; i < prmlimit_policy.req.cluster_num; i++) {
			if (prmlimit_data.limit[i].min_freq_idx != -1
				|| prmlimit_data.limit[i].max_freq_idx != -1) {
				is_limit = true;
				break;
			}
		}
		prmlimit_data.is_freq_limited_by_prm = is_limit;

		if(max_freq == 0)
			prmlimit_policy.is_activated = false;
		else
			prmlimit_policy.is_activated = ppm_prmlimit_is_policy_active();

		ppm_unlock(&prmlimit_policy.lock);
		ppm_task_wakeup();
	} else
		ppm_err("@%s: Invalid input!\n", __func__);

out:
	free_page((unsigned long)buf);
	return count;
}

PROC_FOPS_RW(prmlimit_min_cpu_core);
PROC_FOPS_RW(prmlimit_max_cpu_core);
PROC_FOPS_RW(prmlimit_min_cpu_freq);
PROC_FOPS_RW(prmlimit_max_cpu_freq);

static int __init ppm_prmlimit_policy_init(void)
{
	int i, ret = 0;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(prmlimit_min_cpu_core),
		PROC_ENTRY(prmlimit_max_cpu_core),
		PROC_ENTRY(prmlimit_min_cpu_freq),
		PROC_ENTRY(prmlimit_max_cpu_freq),
	};

	FUNC_ENTER(FUNC_LV_POLICY);

	/* create procfs */
	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create(entries[i].name, S_IRUGO | S_IWUSR | S_IWGRP, policy_dir, entries[i].fops)) {
			ppm_err("%s(), create /proc/ppm/policy/%s failed\n", __func__, entries[i].name);
			ret = -EINVAL;
			goto out;
		}
	}

	prmlimit_data.limit = kcalloc(ppm_main_info.cluster_num, sizeof(*prmlimit_data.limit), GFP_KERNEL);
	if (!prmlimit_data.limit) {
		ret = -ENOMEM;
		goto out;
	}

	/* init prmlimit_data */
	for_each_ppm_clusters(i) {
		prmlimit_data.limit[i].min_freq_idx = -1;
		prmlimit_data.limit[i].max_freq_idx = -1;
		prmlimit_data.limit[i].min_core_num = -1;
		prmlimit_data.limit[i].max_core_num = -1;
	}

	if (ppm_main_register_policy(&prmlimit_policy)) {
		ppm_err("@%s: prmlimit policy register failed\n", __func__);
		kfree(prmlimit_data.limit);
		ret = -EINVAL;
		goto out;
	}

	ppm_info("@%s: register %s done!\n", __func__, prmlimit_policy.name);

out:
	FUNC_EXIT(FUNC_LV_POLICY);

	return ret;
}

static void __exit ppm_prmlimit_policy_exit(void)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	kfree(prmlimit_data.limit);

	ppm_main_unregister_policy(&prmlimit_policy);

	FUNC_EXIT(FUNC_LV_POLICY);
}

module_init(ppm_prmlimit_policy_init);
module_exit(ppm_prmlimit_policy_exit);

