#ifndef LCD_POWER_H
#define LCD_POWER_H

/* This enum is for power control between display-touch driver*/
typedef enum {
	DEEP_SLEEP_ENTER = 0, /* For entering deep sleep sequence */
	DEEP_SLEEP_EXIT, /* For exiting deep sleep sequence */
	DSV_TOGGLE, /* For setting DSV Toggle mode */
	DSV_ALWAYS_ON, /* For setting DSV Always on mode */
}DISP_SET_POWER_MODE;

void primary_display_set_power_mode(unsigned int mode);

#endif /* LCD_POWER_H */
