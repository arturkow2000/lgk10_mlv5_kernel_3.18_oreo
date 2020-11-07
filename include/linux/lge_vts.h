#ifndef __LGE_VTS_H__
#define __LGE_VTS_H__

bool lge_vts_get_enabled(void);
void lge_vts_register(char *name, void (*enable)(bool en));

int lge_vts_get_battery_temp(void);

#endif