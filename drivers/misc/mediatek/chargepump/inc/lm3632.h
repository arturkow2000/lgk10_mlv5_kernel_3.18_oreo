#ifndef __LM3632_H__
#define __LM3632_H__

#if defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
int lm3632_chargepump_set_backlight_level(unsigned int level);
void lm3632_dsv_ctrl(int enable);
#else
void chargepump_DSV_on(void);
void chargepump_DSV_off(void);
#endif
#if defined(CONFIG_TOVIS_INCELL_TD4100_HD_LV7)|| defined(CONFIG_LGD_INCELL_LG4894_HD_LV5)
extern int lm3632_old_bl_level;
#endif
#endif