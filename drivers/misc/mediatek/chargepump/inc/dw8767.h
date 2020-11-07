/*
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

#ifndef __DW8767_H__
#define __DW8767_H__

#if defined(BUILD_LK)
void chargepump_DSV_on(void);
void chargepump_DSV_off(void);
#endif
void dw8767_dsv_ctrl(int enable);

#endif
