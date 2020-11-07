
#ifndef __SM5513_H__
#define __SM5513_H__

#include <linux/platform_device.h>
#include <linux/regmap.h>

#define MFD_DEV_NAME "sm5513"

enum sm5513_cc_src_cmp_values {
    CC_SRC_CMP_Rd       = ((0x0 << 1) | (0x1 << 0)),
    CC_SRC_CMP_Ra       = ((0x0 << 1) | (0x0 << 0)),
    CC_SRC_CMP_Open     = ((0x1 << 1) | (0x1 << 0)),
};

enum sm5513_cc_snk_cmp_values {
    CC_SNK_CMP_Open_HV  = 0xF,
    CC_SNK_CMP_Open     = 0x0,
    CC_SNK_CMP_Default  = 0x1,
    CC_SNK_CMP_1_5A     = 0x3,
    CC_SNK_CMP_3_0A     = 0x7,
};

enum sm5513_channel_mux {
    MUX_CH_0  		= 0,		/* Channel Open */
    MUX_CH_1		= 1,		/* Channel for UART */
	MUX_CH_2		= 2,		/* Channel for FACTORY/WATER */
	MUX_CH_FAULT	= 3,		/* Channel Fault */
};

enum sm5513_path_control {
	PATH_EDGE		= 0,
	PATH_UART		= 1,
	PATH_SBU		= 2,
	PATH_OPEN		= 3,
	PATH_DEFAULT	= 4,
};

struct sm5513_platform_data {
    int irq;
	int irq_gpio;
    bool wakeup;
};

/* SM5513 Enable API */
extern int sm5513_enable_cc_detection(bool enable);
extern int sm5513_get_cc_src_cmp(u8 *cc1, u8 *cc2);
extern int sm5513_get_cc_snk_cmp(u8 *cc1, u8 *cc2);
#ifdef CONFIG_LGE_USB_TYPE_C
extern bool sm5513_is_factory_cable(void);
extern int sm5513_cc_information(void);
extern void sm5513_set_path_hw_default(void);

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
extern int cc_controller_get_cc_value(void);
#endif
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
extern int sm5513_get_input_suspend(void);
#endif
#endif
extern int sm5513_set_mux_channel(int channel);
extern u8 sm5513_get_mux_channel(void);
extern bool sm5513_switch_path(int path);
extern int sm5513_current_path(void);
extern bool sm5513_current_cc_status(void);
#ifdef CONFIG_LGE_MODIFY_MODEL
extern bool sm5513_get_modify_model(void);
#endif
#endif /* __SM5513_H__ */
