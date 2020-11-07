/*
 * Copyright (C) LG Electronics 2014
 *
 * Charger Controller interface
 *
 */
#ifndef __CHARGER_CONTROLLER_H__
#define __CHARGER_CONTROLLER_H__

#include <linux/power_supply.h>

#ifdef CONFIG_LGE_PM_VZW_REQ
typedef enum vzw_chg_state {
	VZW_NO_CHARGER,
	VZW_NORMAL_CHARGING,
	VZW_INCOMPATIBLE_CHARGING,
	VZW_UNDER_CURRENT_CHARGING,
	VZW_USB_DRIVER_UNINSTALLED,
	VZW_CHARGER_STATUS_MAX,
} chg_state;
#endif

int chgctrl_thermal_limit(int ma);
int notify_charger_controller(struct power_supply *psy);
void chgctrl_property_override(enum power_supply_property prop,
				       union power_supply_propval *val);

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
int chgctrl_select_charging_current(int *icl, int *fcc);
int chgctrl_select_cv(int *vfloat);
int chgctrl_charging_enabled(void);
int chgctrl_battery_charging_enabled(void);
void chgctrl_update_battery_health(void);
int chgctrl_hvdcp_enabled(void);
#endif

#endif

