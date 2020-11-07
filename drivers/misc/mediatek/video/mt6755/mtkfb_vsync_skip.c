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

#include <linux/types.h>
#include <linux/device.h>
#include "mtkfb_vsync_skip.h"
#include "primary_display.h"
//#include "display_recorder.h"

#ifdef CONFIG_LGE_VSYNC_SKIP
static unsigned int vsync_skip_param[61][2] = {
	/* keep, skip */
	{  0,  0},
	{  1, 59}, {  1, 29}, {  1, 19}, {  1, 14},
	{  1, 11}, {  1,  9}, {  1,  7}, {  2, 13}, {  1,  6},
	{  1,  5}, {  2,  9}, {  1,  4}, {  2,  7}, {  3, 10},
	{  1,  3}, {  3,  8}, {  2,  5}, {  3,  7}, {  4,  9},
	{  1,  2}, {  4,  7}, {  3,  5}, {  5,  8}, {  2,  3},
	{  5,  7}, {  6,  8}, {  4,  5}, {  5,  6}, {  6,  7},
	{  1,  1}, {  7,  6}, {  6,  5}, {  5,  4}, {  8,  6},
	{  7,  5}, {  3,  2}, {  8,  5}, {  5,  3}, {  7,  4},
	{  2,  1}, {  9,  4}, {  7,  3}, {  5,  2}, {  8,  3},
	{  3,  1}, { 10,  3}, {  7,  2}, {  4,  1}, {  9,  2},
	{  5,  1}, {  6,  1}, { 13,  2}, {  7,  1}, {  9,  1},
	{ 11,  1}, { 14,  1}, { 19,  1}, { 29,  1}, { 59,  1},
	{  0,  0},
};

struct vsync_skip_data vsync_skip;
struct vsync_skip_data* vsync_skip_get_data(void)
{
	return &vsync_skip;
}

static int vsync_skip_adjust_param(int fps) {

	if (fps <= 0 || fps >= 60) {
		vsync_skip.keep = 0;
		vsync_skip.skip = 0;
		return 0;
	}

	vsync_skip.keep = vsync_skip_param[fps][0];
	vsync_skip.skip = vsync_skip_param[fps][1];
	return 0;
}

ssize_t fps_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ulong fps;

	if (!count)
		return -EINVAL;

	fps = simple_strtoul(buf, NULL, 10);

	if (fps == 0 || fps >= 60) {
		vsync_skip.enable_skip_vsync = 0;
		vsync_skip.skip_ratio = 60;
		vsync_skip_adjust_param(vsync_skip.skip_ratio);
		primary_display_force_set_fps(vsync_skip.keep,
					      vsync_skip.skip);
		pr_info("Disable frame skip.\n");
	} else {
		vsync_skip.enable_skip_vsync = 1;
		vsync_skip.skip_ratio = fps;
		vsync_skip_adjust_param(vsync_skip.skip_ratio);
		primary_display_force_set_fps(vsync_skip.keep,
					      vsync_skip.skip);
		pr_info("Enable frame skip: Set to %lu fps.\n", fps);
	}
	return count;
}

ssize_t fps_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int r = 0;

	r = snprintf(buf, PAGE_SIZE,
		     "enable_skip_vsync=%d\n"
		     "keep=%d\n"
		     "skip=%d\n"
		     "fps_cnt=%d\n",
		     vsync_skip.enable_skip_vsync,
		     vsync_skip.keep,
		     vsync_skip.skip,
		     vsync_skip.fps_cnt);
	return r;
}

ssize_t fps_ratio_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int r = 0;
	r = snprintf(buf, PAGE_SIZE, "%d 60\n", vsync_skip.skip_ratio);
	return r;
}

ssize_t fps_fcnt_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int r = 0;
	static unsigned int fps_cnt_before = 0;

	if (vsync_skip.fps_cnt < 0)
		goto read_fail;

	r = snprintf(buf, PAGE_SIZE, "%d\n",
		     vsync_skip.fps_cnt - fps_cnt_before);

	if (vsync_skip.fps_cnt > UINT_MAX / 2) {
		vsync_skip.fps_cnt = 0;
		fps_cnt_before = 0;
	} else {
		fps_cnt_before = vsync_skip.fps_cnt;
	}

	return r;

read_fail:
	fps_cnt_before = 0;
	r = snprintf(buf,PAGE_SIZE, "0\n");
	return r;
}

ssize_t show_blank_event_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	int r = 0;
	r = snprintf(buf, PAGE_SIZE, "panel_power_on = %d\n",
		     primary_display_is_alive());
	return r;
}

#endif
