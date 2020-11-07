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
#ifndef __RT4832_H__
#define __RT4832_H__

void rt4832_dsv_ctrl(int enable);
void rt4832_dsv_toggle_ctrl(void);

#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
extern int old_bl_level;
int rt4832_chargepump_set_backlight_level(unsigned int level);
#endif

#endif
