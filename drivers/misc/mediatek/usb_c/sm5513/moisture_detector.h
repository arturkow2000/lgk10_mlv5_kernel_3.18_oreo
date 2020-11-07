
#ifndef __MOISTURE_DETECTOR_PRIVATE_H__
#define __MOISTURE_DETECTOR_PRIVATE_H__

#include <linux/i2c.h>
#include <soc/mediatek/lge/lge_boot_mode.h>

/* debug message level mapping */
//#define mois_dbg_msg      pr_err
#define mois_dbg_msg      pr_debug
#define mois_err_msg      pr_err

enum sbu_path {
    SBU_PATH_OPEN,		//OPEN
	SBU_PATH_UART,		//HOST1
	SBU_PATH_USBID,		//HOST2
};

extern int moisture_det_create(int irq);
extern void start_moisture_detection(int irq);
extern void moisture_en_set(bool enable);
extern bool moisture_en_get(void);
extern bool moisture_status_get(void);

enum moisture_state {
	ADC_STATE_DRY = 0,
	ADC_STATE_WDT, //Wet Detecting
	ADC_STATE_WFD, //Dry Waiting
	ADC_STATE_WET,
	ADC_STATE_GND, //Edge Ground cable
};

#endif
