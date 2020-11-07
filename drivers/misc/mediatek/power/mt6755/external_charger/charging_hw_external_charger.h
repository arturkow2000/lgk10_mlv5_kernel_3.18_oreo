#ifndef __CHARGING_HW_EXTERNAL_CHARGER_H__
#define __CHARGING_HW_EXTERNAL_CHARGER_H__

#include <linux/power_supply.h>

/* register power-supply as main charger */
int chr_control_register(struct power_supply *psy);
int chr_control_register_slave(struct power_supply *psy);

#endif
