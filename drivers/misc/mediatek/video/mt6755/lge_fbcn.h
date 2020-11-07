/*
 * FBCN: Frame Based Clock Normalizer.
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

#ifndef LGE_FBCN_H
#define LGE_FBCN_H

//#include "../mdss_mdp.h"

struct fbcn_interval_stats {
	u64 interval;
	u64 i_upper;
	u64 i_lower;
	int enable;
	int init;
	struct mutex fbcn_lock;
	struct notifier_block nb;
};

ssize_t fbcn_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t fbcn_en_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t fbcn_i_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t fbcn_i_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t fbcn_interval_show(struct device *dev, struct device_attribute *attr, char *buf);
#endif /* LGE_FBCN_H */
