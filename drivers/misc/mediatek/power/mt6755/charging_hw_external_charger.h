/*
* Copyright(C) 2015 LGE.
*
* Author : LGE
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

#ifndef __CHARGING_HW_EXTERNAL_CHARGER_H__
#define __CHARGING_HW_EXTERNAL_CHARGER_H__

#include <linux/power_supply.h>

/* register power-supply as main charger */
int chr_control_register(struct power_supply *psy);
int chr_control_register_slave(struct power_supply *psy);

#endif
