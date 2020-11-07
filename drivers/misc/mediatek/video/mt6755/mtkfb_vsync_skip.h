/* Copyright (c) 2016, LG Electronics Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MTKFB_VSYNC_SKIP_H_
#define _MTKFB_VSYNC_SKIP_H_

#ifdef CONFIG_LGE_VSYNC_SKIP
struct vsync_skip_data {
	char enable_skip_vsync;
	unsigned int keep;
	unsigned int skip;
	unsigned int skip_ratio;
	unsigned int fps_cnt;
};

struct vsync_skip_data* vsync_skip_get_data(void);
ssize_t fps_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t fps_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t fps_ratio_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t fps_fcnt_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t show_blank_event_show(struct device *dev,
        struct device_attribute *attr, char *buf);

#endif
#endif
