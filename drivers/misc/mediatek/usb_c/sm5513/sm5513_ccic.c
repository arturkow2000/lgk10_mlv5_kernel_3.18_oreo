
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/sm5513-private.h>
#include <linux/usb/class-dual-role.h>
#include <linux/power_supply.h>
#include <typec.h>
#include <soc/mediatek/lge/board_lge.h>
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
#include <mt-plat/battery_common.h>
#include <linux/wakelock.h>
#include "moisture_detector.h"
#endif

#include <mt-plat/upmu_common.h>
#include <mt-plat/mt_gpio.h>
#include <linux/delay.h>

#ifdef CONFIG_DUAL_ROLE_USB_INTF
#define DUAL_ROLE_SWAP_TIMEOUT	1500
#endif
#define GPIO_SEL_USBID_EDGE	11
#define CCIC_DEV_NAME   "sm5513-ccic"

struct sm5513_ccic_info {
    struct device *dev;
    struct i2c_client *i2c;

    struct mutex mutex_lock;
#ifdef CONFIG_DUAL_ROLE_USB_INTF
    struct dual_role_phy_instance *dual_role_phy;
#endif

    struct usbtypc *typec;
	struct power_supply cc_psy;
    bool connected_device;
    bool connected_host;

    int irq;

    /* CC STATUS */
    int cable_type;
    int cable_flip;
    int adv_curr;
    int attach_type;
	int vbus_in;
	int vbus_rd;

	#ifdef CONFIG_LGE_USB_TYPE_C
	/* CC Value */
	u8 cc1;
	u8 cc2;
	#endif

#ifdef CONFIG_DUAL_ROLE_USB_INTF
	int role_changing;
	struct delayed_work	role_swap_timer_work;
#endif
	int is_present;
	ktime_t time_stamp;
	struct wake_lock sm5513_moisture_wakelock;
	atomic_t is_wakelock;

    u8 rev_id;
	int path_default;
#ifdef CONFIG_LGE_USB_TYPE_C
	int typec_mode;
#endif
};
static struct sm5513_ccic_info *g_sm5513_ccic = NULL;

#ifdef CONFIG_LGE_MODIFY_MODEL
static bool g_modify_model = false;
/* false -> active water detection
 * true  -> deactive water detection
 */
#endif
static enum power_supply_property sm5513_cc_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TYPE,
#ifdef CONFIG_LGE_USB_TYPE_C
	POWER_SUPPLY_PROP_TYPEC_MODE,
#endif
};

static void sm5513_water_detection(struct sm5513_ccic_info *ccic);

static void sm5513_set_ccic_op_mode(struct i2c_client *i2c, u8 mode)
{
    sm5513_update_reg(i2c, SM5513_REG_CC_CNTL1, mode, 0xf);
}

static void sm5513_set_ccic_command(struct i2c_client *i2c, u8 cmd)
{
    sm5513_update_reg(i2c, SM5513_REG_CC_CNTL4, cmd, 0xf);
}

static void sm5513_print_regmap(struct i2c_client *i2c)
{
    u8 reg_start = SM5513_REG_INTMASK1;
    u8 reg_buf[14] = {0, };
    int i;

    sm5513_bulk_read(i2c, reg_start, 8, reg_buf);
    sm5513_bulk_read(i2c, reg_start+8, 6, &reg_buf[8]);
	#if	0	/* Remove i2c read */
    /* Extra debugging reg value */
    sm5513_bulk_read(i2c, 0x20, 2, reg_buf_ex);
    sm5513_bulk_read(i2c, 0x30, 5, &reg_buf_ex[2]);
    sm5513_bulk_read(i2c, 0xD0, 6, &reg_buf_ex[7]);
    sm5513_read_reg(i2c, 0xE0, &reg_buf_ex[13]);
    sm5513_bulk_read(i2c, 0xED, 2, &reg_buf_ex[14]);
	#endif

    sm5513_err_msg("%s: print-regmap = ", CCIC_DEV_NAME);
    for (i=0; i < 14; ++i) {
        sm5513_err_msg("0x%x:0x%x ", reg_start + i, reg_buf[i]);
    }
	#if 0	/* Remove i2c read */
    for (i=0; i < 16; ++i) {
        if (i < 2) {
            sm5513_dbg_msg("0x%x:0x%x ", 0x20 + i, reg_buf_ex[i]);
        } else if (i < 7) {
            sm5513_dbg_msg("0x%x:0x%x ", 0x30 + i - 2, reg_buf_ex[i]);
        } else if (i < 13) {
            sm5513_dbg_msg("0x%x:0x%x ", 0xD0 + i - 7, reg_buf_ex[i]);
        } else if (i < 14) {
            sm5513_dbg_msg("0x%x:0x%x ", 0xE0, reg_buf_ex[i]);
        } else {
            sm5513_dbg_msg("0x%x:0x%x ", 0xED + i - 14, reg_buf_ex[i]);
        }
    }
	#endif

    sm5513_err_msg("\n");
}

static void sm5513_update_cc_status(struct sm5513_ccic_info *ccic)
{
    u8 reg;

    sm5513_read_reg(ccic->i2c, SM5513_REG_CC_STATUS, &reg);

    ccic->cable_type = (reg >> 6) & 0x1;
    ccic->cable_flip = (reg >> 5) & 0x1;
    ccic->adv_curr = (reg >> 3) & 0x3;
    ccic->attach_type = (reg >> 0) & 0x7;

	sm5513_read_reg(ccic->i2c, SM5513_REG_CC_SRC_CMP, &reg);
	ccic->vbus_in = (reg >> 6) & 0x1;

    sm5513_err_msg("%s: cc_status=(PC=0x%x, FLIP=0x%x, CURRENT=0x%x, TYPE=0x%x, VBUS=0x%x)\n",
             CCIC_DEV_NAME, ccic->cable_type, ccic->cable_flip, ccic->adv_curr, ccic->attach_type, ccic->vbus_in);

	/* Change value for dual role property */
	if (ccic->adv_curr == ADV_CURR_3V_2)
		ccic->adv_curr	= ADV_CURR_3V_1;
}

/* CC_INFORMATION Status							*
 * If you want info, check probe3 value				*/
#ifdef CONFIG_LGE_USB_TYPE_C
int sm5513_cc_information(void)
{
	u8 reg;

	if (g_sm5513_ccic == NULL) {
		sm5513_err_msg("%s: cc_information is not ready\n", CCIC_DEV_NAME);
		return -1;
	}

	sm5513_read_reg(g_sm5513_ccic->i2c, SM5513_REG_PROBE3, &reg);
	reg	&= 0x1F;

	return reg;
}
#endif

/** 
 * Support MediaTek USB Type-C Switch interface 
 */ 
int register_typec_switch_callback(struct typec_switch_data *new_driver)
{
    int ret = -1;

    sm5513_err_msg("%s: register type-c switch name=%s, type=%d\n", CCIC_DEV_NAME, new_driver->name, new_driver->type);

    if (g_sm5513_ccic == NULL) {
        sm5513_err_msg("%s: do not probed device driver\n", CCIC_DEV_NAME);
        return 0;
    }

    sm5513_update_cc_status(g_sm5513_ccic);


	if (new_driver->type == DEVICE_TYPE) {
        g_sm5513_ccic->typec->device_driver = new_driver;
        g_sm5513_ccic->typec->device_driver->on = DISABLE;
        if (g_sm5513_ccic->attach_type == ATTACH_TYPE_SOURCE) {
			g_sm5513_ccic->typec->device_driver->enable(g_sm5513_ccic->typec->device_driver->priv_data);
            g_sm5513_ccic->connected_device = 1;
        }
        ret = 0;
	}

	if (new_driver->type == HOST_TYPE) {
		g_sm5513_ccic->typec->host_driver = new_driver;
		g_sm5513_ccic->typec->host_driver->on = DISABLE;
        if (g_sm5513_ccic->attach_type == ATTACH_TYPE_SINK) {
			g_sm5513_ccic->typec->host_driver->enable(g_sm5513_ccic->typec->host_driver->priv_data);
            g_sm5513_ccic->connected_host = 1;
        }
        ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(register_typec_switch_callback);

int unregister_typec_switch_callback(struct typec_switch_data *new_driver)
{
    sm5513_err_msg("%s: unregister type-c switch name=%s, type=%d\n", CCIC_DEV_NAME, new_driver->name, new_driver->type);

    if (g_sm5513_ccic == NULL) {
        sm5513_err_msg("%s: do not probed device driver\n", CCIC_DEV_NAME);
        return -1;
    }

	if ((new_driver->type == DEVICE_TYPE) && (g_sm5513_ccic->typec->device_driver == new_driver))
		g_sm5513_ccic->typec->device_driver = NULL;

	if ((new_driver->type == HOST_TYPE) && (g_sm5513_ccic->typec->host_driver == new_driver))
		g_sm5513_ccic->typec->host_driver = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(unregister_typec_switch_callback);

/*
 * Support Export functions for CC detection control
 */
bool g_irq_flag = true;
int sm5513_enable_cc_detection(bool enable)
{
    u8 reg_buf[3];

    sm5513_dbg_msg("%s: %s (enable=%d)\n", CCIC_DEV_NAME, __func__, enable);

    if (g_sm5513_ccic == NULL) {
        sm5513_err_msg("%s: do not probed device driver\n", CCIC_DEV_NAME);
        return -1;
    }

    if (enable) {
        sm5513_bulk_read(g_sm5513_ccic->i2c, SM5513_REG_INT1, 2, reg_buf);  /* Clear Interrupt REG */
		if(!g_irq_flag) {
			sm5513_dbg_msg("%s:[Mois] enable_irq!\n", __func__);
	        enable_irq(g_sm5513_ccic->irq);
			g_irq_flag = true;
		}
        sm5513_set_ccic_command(g_sm5513_ccic->i2c, CC_CMD_UNATT_SNK);      /* Enable CC detection */
    } else {
		if (g_irq_flag) {
			sm5513_dbg_msg("%s:[Mois] disable_irq!\n", __func__);
	        disable_irq_nosync(g_sm5513_ccic->irq);
			g_irq_flag = false;
		}
        sm5513_set_ccic_command(g_sm5513_ccic->i2c, CC_CMD_DISABLE);        /* Disable CC detection */
    }

    return 0;
}
EXPORT_SYMBOL_GPL(sm5513_enable_cc_detection);

#ifdef CONFIG_LGE_MODIFY_MODEL
bool sm5513_get_modify_model(void)
{
	return g_modify_model;
}
#endif
static int get_stable_reg_value(u8 addr, u8 *reg)
{
    u8 reg_now = 0, reg_old = 0;
    int i, cnt;

    cnt = 0;
    for (i=0; i < 30; ++i) {
        sm5513_read_reg(g_sm5513_ccic->i2c, addr, &reg_now);
        if (reg_now == reg_old) {
            cnt++;
            if (cnt > 3) {
                break;
            }
        } else {
            reg_old = reg_now;
            cnt = 0;
        }

        mdelay(5);
    }

    if (i >= 30) {
        sm5513_err_msg("%s:%s can't getting stable reg value (addr=0x%x)\n", CCIC_DEV_NAME, __func__, addr);
        return -EBUSY;
    }

    *reg = reg_now;

    return 0;
}

int sm5513_get_cc_src_cmp(u8 *cc1, u8 *cc2)
{
    int ret;
    u8 reg;

    ret = get_stable_reg_value(SM5513_REG_CC_SRC_CMP, &reg);
    if (ret < 0) {
        return ret;
    }

    *cc1 = (reg & (0x3 << 0)) >> 0;
    *cc2 = (reg & (0x3 << 2)) >> 2;

    /* sm5513_dbg_msg("%s:%s (cc1=%d, cc2=%d)\n", CCIC_DEV_NAME, __func__, *cc1, *cc2); */

    return 0;
}

int sm5513_get_cc_snk_cmp(u8 *cc1, u8 *cc2)
{
    int ret;
    u8 reg;

    ret = get_stable_reg_value(SM5513_REG_CC_SNK_CMP, &reg);
    if (ret < 0) {
        return ret;
    }

    *cc1 = (reg & (0xF << 0)) >> 0;
    *cc2 = (reg & (0xF << 4)) >> 4;

    /* sm5513_dbg_msg("%s:%s (cc1=%d, cc2=%d)\n", CCIC_DEV_NAME, __func__, *cc1, *cc2); */

    return 0;
}

static char* cc_snk_type_print(u8 type)
{
	switch(type)
	{
		case CC_SNK_CMP_Open_HV:
			return "Open_HV";
		case CC_SNK_CMP_Open:
			return "Rp_Open";
		case CC_SNK_CMP_Default:
			return "Rp_Default";
		case CC_SNK_CMP_1_5A:
			return "Rp_1.5A";
		case CC_SNK_CMP_3_0A:
			return "Rp_3.0A";
		default:
			sm5513_err_msg("%s:%s Unknown Snk CC\n", CCIC_DEV_NAME, __func__);
			return "Unknown CC\n";
	}
}

static char* cc_src_type_print(u8 type)
{
	switch(type)
	{
		case CC_SRC_CMP_Rd:
			return "Rd";
		case CC_SRC_CMP_Ra:
			return "Ra";
		case CC_SRC_CMP_Open:
			return "Open";
		default:
			sm5513_err_msg("%s:%s Unknown Src CC\n", CCIC_DEV_NAME, __func__);
			return "Unknown CC\n";
	}
}

#ifdef CONFIG_LGE_USB_TYPE_C
unsigned int change_dual_role_snk_cc(u8 cc)
{
	unsigned int ret;

	if (cc == CC_SNK_CMP_Default)
		ret	= DUAL_ROLE_PROP_CC_RP_DEFAULT;
	else if (cc == CC_SNK_CMP_1_5A)
		ret	= DUAL_ROLE_PROP_CC_RP_POWER1P5;
	else if (cc == CC_SNK_CMP_3_0A)
		ret	= DUAL_ROLE_PROP_CC_RP_POWER3P0;
	else if ((cc == CC_SNK_CMP_Open) || (cc == CC_SNK_CMP_Open_HV))
		ret	= DUAL_ROLE_PROP_CC_OPEN;
	else
		ret = DUAL_ROLE_PROP_CC_OPEN;

	return ret;
}

unsigned int change_dual_role_src_cc(u8 cc)
{
	unsigned int ret;

	if (cc == CC_SRC_CMP_Rd)
		ret	= DUAL_ROLE_PROP_CC_RD;
	else if (cc == CC_SRC_CMP_Ra)
		ret	= DUAL_ROLE_PROP_CC_RA;
	else if (cc == CC_SRC_CMP_Open)
		ret	= DUAL_ROLE_PROP_CC_OPEN;
	else
		ret = DUAL_ROLE_PROP_CC_OPEN;

	return ret;
}
#endif

void sm5513_get_cc_value(struct sm5513_ccic_info *ccic)
{
	u8 cc1, cc2;
	char* cc1_str	= "";
	char* cc2_str	= "";

	if (ccic->attach_type == ATTACH_TYPE_SOURCE) {
		sm5513_get_cc_snk_cmp(&cc1, &cc2);
		cc1_str		= cc_snk_type_print(cc1);
		cc2_str		= cc_snk_type_print(cc2);
		#ifdef CONFIG_LGE_USB_TYPE_C
		ccic->cc1	= change_dual_role_snk_cc(cc1);
		ccic->cc2	= change_dual_role_snk_cc(cc2);
		#endif
	} else if (ccic->attach_type == ATTACH_TYPE_SINK) {
		sm5513_get_cc_src_cmp(&cc1, &cc2);
		#ifdef CONFIG_LGE_USB_TYPE_C
		/* Check Powered cable */
		if ((cc1 == CC_SRC_CMP_Ra) && (ccic->cable_type != CABLE_TYPE_POWERED))
			cc1	= CC_SRC_CMP_Open;
		if ((cc2 == CC_SRC_CMP_Ra) && (ccic->cable_type != CABLE_TYPE_POWERED))
			cc2	= CC_SRC_CMP_Open;
		#endif
		cc1_str		= cc_src_type_print(cc1);
		cc2_str		= cc_src_type_print(cc2);
		#ifdef CONFIG_LGE_USB_TYPE_C
		ccic->cc1	= change_dual_role_src_cc(cc1);
		ccic->cc2	= change_dual_role_src_cc(cc2);
		#endif
	} else {
		sm5513_get_cc_src_cmp(&cc1, &cc2);
		#ifdef CONFIG_LGE_USB_TYPE_C
		/* Check Powered cable */
		if ((cc1 == CC_SRC_CMP_Ra) && (ccic->cable_type != CABLE_TYPE_POWERED))
			cc1 = CC_SRC_CMP_Open;
		if ((cc2 == CC_SRC_CMP_Ra) && (ccic->cable_type != CABLE_TYPE_POWERED))
			cc2 = CC_SRC_CMP_Open;
		#endif
		cc1_str		= cc_src_type_print(cc1);
		cc2_str		= cc_src_type_print(cc2);
		#ifdef CONFIG_LGE_USB_TYPE_C
		ccic->cc1	= change_dual_role_src_cc(cc1);
		ccic->cc2	= change_dual_role_src_cc(cc2);
		#endif
	}
	sm5513_err_msg("%s:%s (cc1=%s, cc2=%s)\n", CCIC_DEV_NAME, __func__, cc1_str, cc2_str);
}

#ifdef CONFIG_LGE_USB_TYPE_C
bool sm5513_is_factory_cable(void)
{
	int sbu_adc_voltage = 0;
	struct sm5513_ccic_info *ccic = g_sm5513_ccic;

	if (ccic->rev_id >= SM5513_REVISION_3) {
		if (ccic->attach_type == ATTACH_TYPE_UNORI_DEBUG) {
			sm5513_err_msg("%s:%s - Rd/Rd Cable\n", CCIC_DEV_NAME, __func__);
			return 1;
		}
	} else {
		sm5513_get_cc_value(ccic);
		if ((ccic->cc1 == DUAL_ROLE_PROP_CC_RD) && (ccic->cc2 == DUAL_ROLE_PROP_CC_RD)) {
			sm5513_dbg_msg("%s:%s Rd/Rd Cable\n", CCIC_DEV_NAME, __func__);
			return 1;
		}
	}

	sm5513_dbg_msg("%s:%s No Factory Cable\n", CCIC_DEV_NAME, __func__);

#ifdef CONFIG_LGE_MODIFY_MODEL
	if (g_modify_model) { // For alpha model (WD disable)
		// Nothing to do
	} else { // For normal model (WD enable)
		if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) { //ONLY check chargerlogo mode
			sm5513_switch_path(PATH_SBU);
			msleep(50);
			sbu_adc_voltage = PMIC_IMM_GetOneChannelValue(7, 5, 0);
			sm5513_switch_path(PATH_OPEN);
			msleep(50);
			if (sbu_adc_voltage >= 1200) {
				sm5513_err_msg("[Mois]%s insert WET cable! immediately go to WET state, sbu_V = %d\n",
						__func__, sbu_adc_voltage);
				sm5513_water_detection(ccic);
			}
		}
	}
#else
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) { //ONLY check chargerlogo mode
		sm5513_switch_path(PATH_SBU);
		msleep(50);
		sbu_adc_voltage = PMIC_IMM_GetOneChannelValue(7, 5, 0);
		sm5513_switch_path(PATH_OPEN);
		msleep(50);
		if (sbu_adc_voltage >= 1200) {
			sm5513_err_msg("[Mois]%s insert WET cable! immediately go to WET state, sbu_V = %d\n",
					__func__, sbu_adc_voltage);
			sm5513_water_detection(ccic);
		}
	}
#endif
	if (ccic->path_default == PATH_UART)
		return 0;

	return 0;
}

void sm5513_set_path_hw_default(void)
{
    if (g_sm5513_ccic == NULL) {
        sm5513_err_msg("%s: do not probed device driver\n", CCIC_DEV_NAME);
        return;
    }

	/* Release ENnOEPIN,ENSELPIN,ENnSINKPIN to Default */
	sm5513_update_reg(g_sm5513_ccic->i2c, SM5513_REG_SWCNTL, 0x03, 0x07);
}

#ifdef CONFIG_LGE_PM_EXTERNAL_CHARGER_SUPPORT
int cc_controller_get_cc_value(void)
{
    struct sm5513_ccic_info *ccic = g_sm5513_ccic;
    int cc_value = DUAL_ROLE_PROP_CC_OPEN;

    if ((ccic->cc1 | ccic->cc2) == DUAL_ROLE_PROP_CC_RP_DEFAULT)
        cc_value = DUAL_ROLE_PROP_CC_RP_DEFAULT;
    else if ((ccic->cc1 | ccic->cc2) == DUAL_ROLE_PROP_CC_RP_POWER1P5)
        cc_value = DUAL_ROLE_PROP_CC_RP_POWER1P5;
    else if ((ccic->cc1 | ccic->cc2) == DUAL_ROLE_PROP_CC_RP_POWER3P0)
        cc_value = DUAL_ROLE_PROP_CC_RP_POWER3P0;
    else if ((ccic->cc1 | ccic->cc2) == DUAL_ROLE_PROP_CC_OPEN)
        cc_value = DUAL_ROLE_PROP_CC_OPEN;
    else
        cc_value = DUAL_ROLE_PROP_CC_OPEN;

    return cc_value;
}
#endif
#endif

#ifdef CONFIG_LGE_USB_TYPE_C
static void sm5513_set_prop_to_usb(int mode)
{
    struct power_supply *usb_psy;
    union power_supply_propval val;

    usb_psy = power_supply_get_by_name("usb");
    if (!usb_psy || !usb_psy->set_property)
        return;

    val.intval = mode;
    usb_psy->set_property(usb_psy, POWER_SUPPLY_PROP_TYPEC_MODE, &val);
}

static void sm5513_update_typec_mode(struct sm5513_ccic_info *ccic)
{
    int typec_mode = ccic->typec_mode;

    switch (ccic->attach_type) {
    case ATTACH_TYPE_SOURCE:
        switch(ccic->adv_curr) {
        case ADV_CURR_1_5V:
            ccic->typec_mode = POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
            break;
        case ADV_CURR_3V_1:
            /* fall thourgh */
        case ADV_CURR_3V_2:
            ccic->typec_mode = POWER_SUPPLY_TYPEC_SOURCE_HIGH;
            break;
        case ADV_CURR_DEFAULT:
        default:
            ccic->typec_mode = POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
            break;
        }
        break;
    case ATTACH_TYPE_SINK:
        ccic->typec_mode = POWER_SUPPLY_TYPEC_SINK;
        break;
    case ATTACH_TYPE_AUDIO:
        /* fall thourgh */
    case ATTACH_TYPE_AUDIO_CHARGE:
        ccic->typec_mode = POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
        break;
    case ATTACH_TYPE_UNORI_DEBUG:
        ccic->typec_mode = POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
        break;
    case ATTACH_TYPE_ORI_DEBUG:
        ccic->typec_mode = POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE;
        break;
    case ATTACH_TYPE_NONE:
        ccic->typec_mode = POWER_SUPPLY_TYPEC_NONE;
        break;
    default:
        ccic->typec_mode = POWER_SUPPLY_TYPEC_NONE;
        break;
    }

    if (ccic->typec_mode != typec_mode)
        sm5513_set_prop_to_usb(ccic->typec_mode);
}
#endif

/**
 * Support DUAL_ROLE_CLASS interface 
 */
#ifdef CONFIG_DUAL_ROLE_USB_INTF
static enum dual_role_property sm5513_dual_role_props[] = {
	DUAL_ROLE_PROP_SUPPORTED_MODES,
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
	DUAL_ROLE_PROP_VCONN_SUPPLY,
#ifdef CONFIG_LGE_USB_TYPE_C
	DUAL_ROLE_PROP_CC1,
	DUAL_ROLE_PROP_CC2,
#endif
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
	DUAL_ROLE_PROP_MOISTURE_EN,
#endif
};

static int sm5513_dual_role_get_property(struct dual_role_phy_instance *dual_role,
                                         enum dual_role_property prop,
                                         unsigned int *val)
{
    struct sm5513_ccic_info *ccic = g_sm5513_ccic;

    sm5513_update_cc_status(ccic);

    switch (prop) {
    case DUAL_ROLE_PROP_SUPPORTED_MODES:
		/* In O-OS, DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP_AND_FAULT should not used */
        *val = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
        break;

    case DUAL_ROLE_PROP_MODE:
		#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
		if (moisture_status_get())	{
			*val = DUAL_ROLE_PROP_MODE_FAULT;
			break;
		}
		#endif
        switch (ccic->attach_type) {
        case ATTACH_TYPE_SOURCE:
            *val = DUAL_ROLE_PROP_MODE_UFP;
            break;
        case ATTACH_TYPE_SINK:
            *val = DUAL_ROLE_PROP_MODE_DFP;
			/* If VBUS + Rd, Mode must be UFP */
			if (ccic->vbus_rd)
				*val = DUAL_ROLE_PROP_MODE_UFP;
            break;
		case ATTACH_TYPE_UNORI_DEBUG:
			if (ccic->rev_id >= SM5513_REVISION_3)	{
				*val = DUAL_ROLE_PROP_MODE_UFP;
			} else {
				*val = DUAL_ROLE_PROP_MODE_NONE;
			}
			break;
        case ATTACH_TYPE_NONE:
        default:
            *val = DUAL_ROLE_PROP_MODE_NONE;
            break;
        }
        break;

    case DUAL_ROLE_PROP_PR:
		#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
		if (moisture_status_get())	{
			*val = DUAL_ROLE_PROP_PR_FAULT;
			break;
		}
		#endif
        switch (ccic->attach_type) {
        case ATTACH_TYPE_SOURCE:
            *val = DUAL_ROLE_PROP_PR_SNK;
            break;
        case ATTACH_TYPE_SINK:
            *val = DUAL_ROLE_PROP_PR_SRC;
			/* If VBUS + Rd, Mode must be SNK */
			if (ccic->vbus_rd)
				*val = DUAL_ROLE_PROP_PR_SNK;
            break;
		case ATTACH_TYPE_UNORI_DEBUG:
			if (ccic->rev_id >= SM5513_REVISION_3)	{
				*val = DUAL_ROLE_PROP_PR_SNK;
			} else {
				*val = DUAL_ROLE_PROP_PR_NONE;
			}
			break;
        case ATTACH_TYPE_NONE:
        default:
            *val = DUAL_ROLE_PROP_PR_NONE;
            break;
        }
        break;

    case DUAL_ROLE_PROP_DR:
		#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
		if (moisture_status_get())	{
			*val = DUAL_ROLE_PROP_DR_FAULT;
			break;
		}
		#endif
        switch (ccic->attach_type) {
        case ATTACH_TYPE_SOURCE:
            *val = DUAL_ROLE_PROP_DR_DEVICE;
            break;
        case ATTACH_TYPE_SINK:
            *val = DUAL_ROLE_PROP_DR_HOST;
			/* If VBUS + Rd, Mode must be DEVICE */
			if (ccic->vbus_rd)
				*val = DUAL_ROLE_PROP_DR_DEVICE;
            break;
		case ATTACH_TYPE_UNORI_DEBUG:
			if (ccic->rev_id >= SM5513_REVISION_3)	{
				*val = DUAL_ROLE_PROP_DR_DEVICE;
			} else {
				*val = DUAL_ROLE_PROP_DR_NONE;
			}
			break;
        case ATTACH_TYPE_NONE:
        default:
            *val = DUAL_ROLE_PROP_DR_NONE;
            break;
        }
		break;

    case DUAL_ROLE_PROP_VCONN_SUPPLY:
		*val = DUAL_ROLE_PROP_VCONN_SUPPLY_NO;
        break;

#ifdef CONFIG_LGE_USB_TYPE_C
	case DUAL_ROLE_PROP_CC1:
	case DUAL_ROLE_PROP_CC2:
		sm5513_get_cc_value(ccic);
		#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
		if (moisture_status_get())	{
			*val = DUAL_ROLE_PROP_CC_OPEN;
			break;
		}
		#endif
		switch (ccic->attach_type) {
		case ATTACH_TYPE_SOURCE:
		case ATTACH_TYPE_SINK:
			if (prop == DUAL_ROLE_PROP_CC1)
				*val	= ccic->cc1;
			else
				*val	= ccic->cc2;
			break;

		case ATTACH_TYPE_AUDIO:
		case ATTACH_TYPE_AUDIO_CHARGE:
			*val	= DUAL_ROLE_PROP_CC_RA;
			break;

		case ATTACH_TYPE_DEBUG:
		case ATTACH_TYPE_ORI_DEBUG:
			if (prop == DUAL_ROLE_PROP_CC1)
				*val	= ccic->cc1;
			else
				*val	= ccic->cc2;
            break;

		case ATTACH_TYPE_UNORI_DEBUG:
			*val	= DUAL_ROLE_PROP_CC_RD;
			break;

		case ATTACH_TYPE_NONE:
		default:
			*val	= DUAL_ROLE_PROP_CC_OPEN;
			break;
		}
		break;
#endif

#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
	case DUAL_ROLE_PROP_MOISTURE_EN:
		if (moisture_en_get())
			*val	= DUAL_ROLE_PROP_MOISTURE_EN_ENABLE;
		else
			*val	= DUAL_ROLE_PROP_MOISTURE_EN_DISABLE;
		break;
#endif

    default:
        sm5513_err_msg("%s:%s-invalid property index = %d\n", CCIC_DEV_NAME, __func__, prop);
        return -EINVAL;
    }

	sm5513_err_msg("%s:%s-prop=%d ret: %d\n", CCIC_DEV_NAME, __func__, prop, *val);

    return 0;
}

static int sm5513_dual_role_set_property(struct dual_role_phy_instance *dual_role,
                                         enum dual_role_property prop,
                                         const unsigned int *val)
{
    struct sm5513_ccic_info *ccic = g_sm5513_ccic;
    int ret = 0;

    sm5513_err_msg("%s:%s-prop=%d, val=%d\n", CCIC_DEV_NAME, __func__, prop, *val);

	switch (prop) {
	case DUAL_ROLE_PROP_MODE:
		/* Go to Host Mode */
		if (*val == DUAL_ROLE_PROP_MODE_DFP) {
            sm5513_err_msg("%s:Do Role Swap [SINK->SRC]\n", CCIC_DEV_NAME);
            ccic->role_changing = CC_OP_MODE_SRC_ONLY;
            sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_SRC_ONLY);
            sm5513_set_ccic_command(ccic->i2c, CC_CMD_UNATT_SRC);
			schedule_delayed_work(&ccic->role_swap_timer_work, DUAL_ROLE_SWAP_TIMEOUT * HZ / 1000);

		/* Go to Device Mode */
		} else if (*val == DUAL_ROLE_PROP_MODE_UFP)	{
            sm5513_err_msg("%s:Do Role Swap [SRC->SINK]\n", CCIC_DEV_NAME);
            ccic->role_changing = CC_OP_MODE_SNK_ONLY;
            sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_SNK_ONLY);
            sm5513_set_ccic_command(ccic->i2c, CC_CMD_UNATT_SNK);
			schedule_delayed_work(&ccic->role_swap_timer_work, DUAL_ROLE_SWAP_TIMEOUT * HZ / 1000);
		} else {
			sm5513_err_msg("%s:Unsupported Status\n", CCIC_DEV_NAME);
		}
		break;

	/* sm5513 should not suppot PD. If not used below, it must be remove */
	#if	0
    case DUAL_ROLE_PROP_PR:
        if (*val == DUAL_ROLE_PROP_PR_SRC && ccic->attach_type == ATTACH_TYPE_SOURCE) {
            sm5513_dbg_msg("%s:Do Power-role-swap [SINK->SRC]\n", CCIC_DEV_NAME);
            sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_TRY_SRC);
            sm5513_set_ccic_command(ccic->i2c, CC_CMD_UNATT_SRC);
        } else if (*val == DUAL_ROLE_PROP_PR_SNK && ccic->attach_type == ATTACH_TYPE_SINK) {
            sm5513_dbg_msg("%s:Do Power-role-swap [SRC->SINK]\n", CCIC_DEV_NAME);
            sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_TRY_SNK);
            sm5513_set_ccic_command(ccic->i2c, CC_CMD_UNATT_SNK);
        }
		break;

    case DUAL_ROLE_PROP_DR:
        if (*val == DUAL_ROLE_PROP_DR_HOST && ccic->attach_type == ATTACH_TYPE_SOURCE) {
            sm5513_dbg_msg("%s:Do Data-role-swap [Device->Host]\n", CCIC_DEV_NAME);
            sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_TRY_SRC);
            sm5513_set_ccic_command(ccic->i2c, CC_CMD_UNATT_SRC);
        } else if (*val == DUAL_ROLE_PROP_DR_DEVICE && ccic->attach_type == ATTACH_TYPE_SINK) {
            sm5513_dbg_msg("%s:Do Data-role-swap [Host->Device]\n", CCIC_DEV_NAME);
            sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_TRY_SNK);
            sm5513_set_ccic_command(ccic->i2c, CC_CMD_UNATT_SNK);
        }
		break;

    case DUAL_ROLE_PROP_VCONN_SUPPLY:
		if (*val == DUAL_ROLE_PROP_VCONN_SUPPLY_NO && ccic->vconn_enabled != 0) {
			sm5513_dbg_msg("%s: Do VConn [Enable->Disable]\n", CCIC_DEV_NAME);
		} else if (*val == DUAL_ROLE_PROP_VCONN_SUPPLY_YES && ccic->vconn_enabled == 0) {
			sm5513_dbg_msg("%s: Do VConn [Disable->Enable]\n", CCIC_DEV_NAME);
		}
		break;
	#endif

#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
	case DUAL_ROLE_PROP_MOISTURE_EN:
		if ((*val == DUAL_ROLE_PROP_MOISTURE_EN_ENABLE) && (moisture_en_get() != true)) {
			moisture_en_set(true);
			sm5513_err_msg("%s:[Mois] is changed to ENABLE \n", CCIC_DEV_NAME);
#ifdef CONFIG_DUAL_ROLE_USB_INTF
			dual_role_instance_changed(ccic->dual_role_phy);
#endif
		} else if ((*val == DUAL_ROLE_PROP_MOISTURE_EN_DISABLE) && (moisture_en_get() != false)) {
			moisture_en_set(false);
			sm5513_err_msg("%s:[Mois] is changed to DISABLE \n", CCIC_DEV_NAME);
#ifdef CONFIG_DUAL_ROLE_USB_INTF
			dual_role_instance_changed(ccic->dual_role_phy);
#endif
		}
		break;
#endif

	default:
    	ret = -EINVAL;
	}

    return ret;
}

static int sm5513_dual_role_prop_is_writeable(struct dual_role_phy_instance *dual_role,
											   enum dual_role_property prop)
{
    int ret = 0;

    sm5513_dbg_msg("%s:%s-prop=%d\n", CCIC_DEV_NAME, __func__, prop);

    switch (prop) {
	case DUAL_ROLE_PROP_MODE:
#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
	case DUAL_ROLE_PROP_MOISTURE_EN:
#endif
        ret = 1;
        break;

	/* sm5513 should not suppot PD. If not used below, it must be remove */
	#if	0
	case DUAL_ROLE_PROP_PR:
	case DUAL_ROLE_PROP_DR:
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
		ret	= 1;
		break;
	#endif

    default:
        break;
    }

    return ret;
}

static void do_role_swap_timer_work(struct work_struct *work)
{
	struct sm5513_ccic_info *ccic = container_of(work, struct sm5513_ccic_info, role_swap_timer_work.work);

	if (ccic->role_changing != CC_OP_MODE_DEFAULT) {
		sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_TRY_SNK);
		ccic->role_changing = CC_OP_MODE_DEFAULT;
		sm5513_err_msg("%s:%s- Role Swap Timeout\n", CCIC_DEV_NAME, __func__);
	}
}

static int register_dual_role_instance(struct sm5513_ccic_info *ccic)
{
    struct dual_role_phy_desc *drp_desc;

	drp_desc = devm_kzalloc(ccic->dev, sizeof(*drp_desc), GFP_KERNEL);
	if (!drp_desc) {
        sm5513_err_msg("%s:%s-fail to kzalloc for dual_role_desc\n", CCIC_DEV_NAME, __func__);
		return -ENOMEM;
    }
    drp_desc->name = "otg_default";
    drp_desc->num_properties = ARRAY_SIZE(sm5513_dual_role_props);
    drp_desc->properties = sm5513_dual_role_props;
    drp_desc->get_property = sm5513_dual_role_get_property;
    drp_desc->set_property = sm5513_dual_role_set_property;
    drp_desc->property_is_writeable = sm5513_dual_role_prop_is_writeable;
    drp_desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP,

    ccic->dual_role_phy = devm_dual_role_instance_register(ccic->dev, drp_desc);
    if (IS_ERR(ccic->dual_role_phy)) {
        sm5513_err_msg("%s: fail to register dual_role_instance for sm5513\n", CCIC_DEV_NAME);
        return -EINVAL;
    }
    ccic->dual_role_phy->drv_data = ccic;

	INIT_DELAYED_WORK(&ccic->role_swap_timer_work, do_role_swap_timer_work);

    return 0;
}

#endif

static int sm5513_cc_get_property(struct power_supply *psy, enum power_supply_property prop, union power_supply_propval *val)
{
	struct sm5513_ccic_info *ccic	= container_of(psy, struct sm5513_ccic_info, cc_psy);
	sm5513_update_cc_status(ccic);
	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval	= ccic->is_present;
		if (moisture_status_get())
			val->intval = BMT_status.charger_exist;
		break;
#ifdef CONFIG_LGE_USB_TYPE_C
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		val->intval = ccic->typec_mode;
		break;
#endif
	default:
		return -EINVAL;
	}

	sm5513_err_msg("%s:%s-prop=%d val: %d\n", CCIC_DEV_NAME, __func__, prop, val->intval);

	return 0;
}

static int sm5513_cc_set_property(struct power_supply *psy, enum power_supply_property prop, const union power_supply_propval *val)
{
	int ret = 0;
	struct sm5513_ccic_info *ccic = container_of(psy, struct sm5513_ccic_info, cc_psy);
	sm5513_update_cc_status(ccic);
	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (val->intval == ccic->is_present)
			break;
		ccic->is_present	= val->intval;
		break;
#ifdef CONFIG_LGE_USB_TYPE_C
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		if (val->intval == ccic->typec_mode)
			break;
		ccic->typec_mode = val->intval;
		break;
#endif
	default:
		return -EINVAL;
	}

	return ret;
}

static int sm5513_cc_property_is_writeable(struct power_supply *psy, enum power_supply_property prop)
{
	int rc = 0;
	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		rc = 1;
		break;

	default:
		rc = 0;
		break;
	}

	return rc;
}

static struct power_supply sm5513_psy = {
	.name = "usb_pd",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties	= sm5513_cc_properties,
	.num_properties = ARRAY_SIZE(sm5513_cc_properties),
	.get_property = sm5513_cc_get_property,
	.set_property = sm5513_cc_set_property,
	.property_is_writeable = sm5513_cc_property_is_writeable,
};

static __inline void sm5513_ccic_init_hw_configuration(struct sm5513_ccic_info *ccic)
{
    /* configure system intiialize setting */

#ifdef CONFIG_LGE_MODIFY_MODEL
	if (g_modify_model) { // For alpha model (WD disable)
		sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK1, 0xF4);		/* Enable SRC_ADV_CHG / ATTACH / DETACH */
		sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK2, 0xF4); 	/* Enable DPDM_OVP(SBU) / CC2_OVP / CC1_OVP */
	} else { // For normal model (WD enable)
		if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) { //ONLY check chargerlogo mode
			sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK1, 0x34);     /* Enable CC_ST_CHG / CC_ABNORMAL/ SRC_ADV_CHG / ATTACH / DETACH */
			sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK2, 0xF4); 	/* Enable DPDM_OVP(SBU) / CC2_OVP / CC1_OVP */
		} else {
			sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK1, 0x74);     /* Enable CC_ST_CHG / SRC_ADV_CHG / ATTACH / DETACH */
			sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK2, 0xFC); 	/* Enable CC2_OVP / CC1_OVP */
		}
	}
#else
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) { //ONLY check chargerlogo mode
		sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK1, 0x34);     /* Enable CC_ST_CHG / CC_ABNORMAL/ SRC_ADV_CHG / ATTACH / DETACH */
		sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK2, 0xF4); 	/* Enable DPDM_OVP(SBU) / CC2_OVP / CC1_OVP */
	} else {
		sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK1, 0x74);     /* Enable CC_ST_CHG / SRC_ADV_CHG / ATTACH / DETACH */
		sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK2, 0xFC); 	/* Enable CC2_OVP / CC1_OVP */
	}
#endif

	/* Set VBUS Ignore */
	if (ccic->rev_id >= SM5513_REVISION_3) {
		sm5513_update_reg(ccic->i2c, SM5513_REG_H_SPARE1, (0x1 << 7), (0x1 << 7));
	}

	sm5513_update_reg(ccic->i2c, SM5513_REG_CC_CNTL1, (0x01 << 4), (0x03 << 4));	/* Set Rp_1.5A for DRP:SRC */
    sm5513_write_reg(ccic->i2c, SM5513_REG_CC_CNTL2, 0x38);         /* enable SRC_DEB_OR_EN, SRC_DEB_EN, SRC_AUD_EN */
    sm5513_write_reg(ccic->i2c, SM5513_REG_CC_CNTL3, 0x30);         /* CC_Debounc 128ms, DRP Perod 90ms, Duty 30% */

    /* restore current optimize */
    sm5513_update_reg(ccic->i2c, 0x21, (0x0 << 6), (0x1 << 6));
    sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_TRY_SNK);

    sm5513_print_regmap(ccic->i2c);
}

static void sm5513_ccic_attach(struct sm5513_ccic_info *ccic)
{

    sm5513_err_msg("%s: %s - start\n", CCIC_DEV_NAME, __func__);

    sm5513_print_regmap(ccic->i2c);
    sm5513_update_cc_status(ccic);
	sm5513_update_reg(ccic->i2c, SM5513_REG_CC_CNTL1, (0x00 << 4), (0x03 << 4));	/* Set Rp_Default for UFP */

    switch (ccic->attach_type) {
    case ATTACH_TYPE_SOURCE:
        sm5513_err_msg("%s: Attached SOURCE - Device\n", CCIC_DEV_NAME);
        if (ccic->typec->device_driver && !ccic->connected_device) {
            ccic->typec->device_driver->enable(ccic->typec->device_driver->priv_data);
            ccic->connected_device = 1;
        }
		ccic->is_present	= 1;
        break;
    case ATTACH_TYPE_SINK:
        sm5513_err_msg("%s: Attached SINK - Host\n", CCIC_DEV_NAME);
		/* If VBUS+ Rd , it will load device driver */
		if (ccic->vbus_in)	{
			sm5513_err_msg("%s: VBUS + Rd !!! \n", CCIC_DEV_NAME);
			if (ccic->typec->device_driver && !ccic->connected_device) {
				ccic->typec->device_driver->enable(ccic->typec->device_driver->priv_data);
				ccic->connected_device = 1;
			}
			ccic->is_present	= 1;
			ccic->vbus_rd		= 1;
			break;
		}

        if (ccic->typec->host_driver && !ccic->connected_host) {
            ccic->typec->host_driver->enable(ccic->typec->host_driver->priv_data);
            ccic->connected_host = 1;
        }
        break;

    case ATTACH_TYPE_AUDIO:			/* Src. Audio Adapter Accessory - Ra + Ra */
        sm5513_err_msg("%s: Not Supported - Audio Adapter Accessory (attach_type=0x%x)\n", CCIC_DEV_NAME, ccic->attach_type);
		break;

    case ATTACH_TYPE_AUDIO_CHARGE:	/* Src. Audio Adapter Accessory Charge - Ra + Ra */
        sm5513_err_msg("%s: Not Supported - Audio Adapter Accessory Charge (attach_type=0x%x)\n", CCIC_DEV_NAME, ccic->attach_type);
		break;

    case ATTACH_TYPE_DEBUG:
        sm5513_err_msg("%s: now didn't support it (attach_type=0x%x)\n", CCIC_DEV_NAME, ccic->attach_type);
		break;

    case ATTACH_TYPE_UNORI_DEBUG:	/* Src. Debug Accessory Mode - Rd + Rd */
        sm5513_err_msg("%s: Debug Accessory Mode (attach_type=0x%x)\n", CCIC_DEV_NAME, ccic->attach_type);
		if (ccic->rev_id >= SM5513_REVISION_3)	{
			ccic->is_present		= 1;
		}
		break;

    case ATTACH_TYPE_ORI_DEBUG:		/* Src. Powered cable with sink - Rd + Ra */
        sm5513_err_msg("%s: Not Supported - Powered cable with sink (attach_type=0x%x)\n", CCIC_DEV_NAME, ccic->attach_type);
        break;

    case ATTACH_TYPE_NONE:
        sm5513_err_msg("%s: invalid attach type (attach_type=0x%x)\n", CCIC_DEV_NAME, ccic->attach_type);
        break;
    }

	/* Set Op-Mode for Role Swap */
	#ifdef CONFIG_DUAL_ROLE_USB_INTF
	if (ccic->role_changing != CC_OP_MODE_DEFAULT) {
		sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_TRY_SNK);
		cancel_delayed_work(&ccic->role_swap_timer_work);
		ccic->role_changing = CC_OP_MODE_DEFAULT;
		sm5513_err_msg("%s: %s - Set Role Swap Default\n", CCIC_DEV_NAME, __func__);
	}
	#endif

    sm5513_err_msg("%s: %s - end\n", CCIC_DEV_NAME, __func__);
}

static void ccic_do_disconnect(struct sm5513_ccic_info *ccic)
{
    if (ccic->typec->host_driver && ccic->connected_host) {
        ccic->typec->host_driver->disable(ccic->typec->host_driver->priv_data);
        ccic->connected_host = 0;
    }
    if (ccic->typec->device_driver && ccic->connected_device) {
        ccic->typec->device_driver->disable(ccic->typec->device_driver->priv_data);
        ccic->connected_device = 0;
    }
	ccic->is_present	= 0;
}

static void sm5513_ccic_detach(struct sm5513_ccic_info *ccic)
{
    sm5513_err_msg("%s: %s - start\n", CCIC_DEV_NAME, __func__);

    /* sm5513_print_regmap(ccic->i2c); */
    sm5513_update_cc_status(ccic);
	sm5513_update_reg(ccic->i2c, SM5513_REG_CC_CNTL1, (0x01 << 4), (0x03 << 4));	/* Set Rp_1.5A for DRP:SRC */

    ccic_do_disconnect(ccic);
	/* Pass : Op-Mode Set for Role Swap */
	/* If role_changing run, could not change op_mode */
	#ifdef CONFIG_DUAL_ROLE_USB_INTF
    if (ccic->role_changing == CC_OP_MODE_DEFAULT) {
        sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_TRY_SNK);
    }
	#else
	sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_TRY_SNK);
	#endif

	ccic->vbus_rd		= 0;
    sm5513_err_msg("%s: %s - end\n", CCIC_DEV_NAME, __func__);
}

static void sm5513_water_detection(struct sm5513_ccic_info *ccic)
{
    sm5513_err_msg("%s: %s - start\n", CCIC_DEV_NAME, __func__);

//  sm5513_print_regmap(ccic->i2c);
    sm5513_update_cc_status(ccic);

    /* Do Water detect process */
	#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
	start_moisture_detection(ccic->irq); /* temp disable for moisture test */
	#endif
    sm5513_err_msg("%s: %s - end\n", CCIC_DEV_NAME, __func__);
}

/* Remove other fault status 0x40 */
#if	0
static void sm5513_ccic_fault(struct sm5513_ccic_info *ccic)
{
    sm5513_err_msg("%s: %s - start\n", CCIC_DEV_NAME, __func__);

    sm5513_print_regmap(ccic->i2c);
    sm5513_update_cc_status(ccic);

    ccic_do_disconnect(ccic);

    sm5513_err_msg("%s: %s - end\n", CCIC_DEV_NAME, __func__);
}
#endif

#define CC_BURST_COUNT 20
#define CC_TIME_OUT_MS 2000
int g_water_cnt = 0;

static irqreturn_t ccic_irq_thread(int irq, void *data)
{
    struct sm5513_ccic_info *ccic = data;
    u8 int1, int2, status1, status2;
    bool water_det = 0;
	ktime_t end_time;
    u8 reg_buf[6] = {0, };
    int ret;


    mutex_lock(&ccic->mutex_lock);

    ret	= sm5513_bulk_read(ccic->i2c, SM5513_REG_INT1, 6, reg_buf);
	if (ret	< 0)	{
		sm5513_err_msg("%s: i2c error: %d\n", CCIC_DEV_NAME, ret);
		mutex_unlock(&ccic->mutex_lock);
		return IRQ_HANDLED;
	}
	int1	= reg_buf[0];
	int2	= reg_buf[1];
	status1	= reg_buf[4];
	status2	= reg_buf[5];
    sm5513_err_msg("%s: int1=0x%x, int2=0x%x, status1=0x%x, status2=0x%x\n", CCIC_DEV_NAME, int1, int2, status1, status2);

    if ((int1 & 0x1) && (status1 & 0x1) && (!water_det)) {
        sm5513_ccic_attach(ccic);
#ifdef CONFIG_LGE_USB_TYPE_C
        sm5513_update_typec_mode(ccic);
#endif
#ifdef CONFIG_DUAL_ROLE_USB_INTF
        dual_role_instance_changed(ccic->dual_role_phy);
#endif
    }

#ifdef CONFIG_LGE_USB_TYPE_C
    /* charge source changed */
    if ((int1 & SM5513_INT1_SRC_ADV_CHG) &&
            (status1 & SM5513_STATUS1_ATTACH)) {
        sm5513_update_cc_status(ccic);
        sm5513_update_typec_mode(ccic);
    }
#endif

    if ((int1 & 0x2) && (status1 & 0x2)) {
        sm5513_ccic_detach(ccic);
#ifdef CONFIG_LGE_USB_TYPE_C
        sm5513_update_typec_mode(ccic);
#endif
#ifdef CONFIG_DUAL_ROLE_USB_INTF
        dual_role_instance_changed(ccic->dual_role_phy);
#endif
    }

#ifdef CONFIG_LGE_MODIFY_MODEL
	if (!g_modify_model) {
	    if ((int1 & 0x80) && (ccic->rev_id > 0)) {
			wake_lock_timeout(&ccic->sm5513_moisture_wakelock, CC_TIME_OUT_MS + 50);
			sm5513_dbg_msg("[sm5513][Mois][Wake] %s: Set WAKELOCK_TIMEOUT\n", __func__);

			ccic->irq = irq;
			sm5513_err_msg("[sm5513][Mois] cc ...? g_water_cnt = %d\n", g_water_cnt);
			if (!g_water_cnt) {
				ccic->time_stamp = ktime_get();
				sm5513_dbg_msg("[Mois] Start ktime stamp\n");
			}
			g_water_cnt++;
			end_time = ktime_get();
			if (ktime_to_ms(ktime_sub(end_time, ccic->time_stamp)) <= CC_TIME_OUT_MS) {
				if (g_water_cnt > CC_BURST_COUNT) {
					sm5513_err_msg("[Mois] CC sm5513 Water detected! irq_n=%d\n", irq);
					atomic_set(&ccic->is_wakelock, true); /* set flag */
					sm5513_water_detection(ccic);
					water_det = 1;
					g_water_cnt = 0;
				}
			} else {
				water_det = 0;
				g_water_cnt = 0;
				wake_unlock(&ccic->sm5513_moisture_wakelock);
				sm5513_dbg_msg("[sm5513][Mois][Wake] %s: WAKE UNLOCK\n", __func__);
			}
		} // if (int1 & 0x80)
    } // g_modify_model
#endif
/* Remove other fault status 0x40 */
#if	0
    if ((int1 & 0x40) && (status1 & 0x40) && (ccic->rev_id > 0)) {
        sm5513_ccic_fault(ccic);
#ifdef CONFIG_DUAL_ROLE_USB_INTF
        dual_role_instance_changed(ccic->dual_role_phy);
#endif
    }
#endif

/* Remove other fault status */
#if	0
    if (int2 & status2) {
        sm5513_ccic_fault(ccic);
#ifdef CONFIG_DUAL_ROLE_USB_INTF
        dual_role_instance_changed(ccic->dual_role_phy);
#endif
    }
#endif

    mutex_unlock(&ccic->mutex_lock);

    return IRQ_HANDLED;
}

void sm5513_switch_init(struct sm5513_ccic_info *ccic)
{

	mt_set_gpio_mode(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_MODE_GPIO);
	mt_set_gpio_pull_enable(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_PULL_DOWN);
//	mt_set_gpio_pull_select(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_PULL_UP); //for edge
	mt_set_gpio_dir(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_DIR_OUT);

	/* It must be set by scenario */
	mt_set_gpio_out(GPIO_SEL_USBID_EDGE, GPIO_OUT_ZERO);
	/* mt_set_gpio_out(GPIO_SEL_USBID_EDGE, GPIO_OUT_ZERO); */ // for edge

	/* Set ENnOEPIN,ENSELPIN,ENnSINKPIN to 0 */
	sm5513_update_reg(ccic->i2c, SM5513_REG_SWCNTL, 0, 0x07);

	#ifdef CONFIG_LGE_USB_DEBUGGER
	/* Check printk.disable_uart by mtk api */
	if (mt_get_uartlog_status())	{
		ccic->path_default	= PATH_UART;
		sm5513_write_reg(ccic->i2c, SM5513_REG_INTMASK1, 0xF4);     /* Disable CC_ST_CHG, CC_ABNORML */
	} else {
		ccic->path_default	= PATH_OPEN;
	}
	#else
	ccic->path_default	= PATH_OPEN;
	#endif

}

int sm5513_current_path(void)
{
	return sm5513_get_mux_channel();
}

bool sm5513_current_cc_status(void)
{
	u8 val;
	sm5513_read_reg(g_sm5513_ccic->i2c, SM5513_REG_CC_CNTL4, &val);

	return !(val & 0x08);
}

static char *switch_path_type_print(u8 type)
{
	switch (type)
	{
		case PATH_EDGE:
			return "EDGE";
		case PATH_UART:
			return "UART";
		case PATH_SBU:
			return "SBU";
		case PATH_OPEN:
			return "OPEN";
		case PATH_DEFAULT:
			return "DEFAULT";
		default:
			return "Unknown path type";
	}
}

bool sm5513_switch_path(int path)
{
	bool check_path	= 0;
	u8 get_channel	= 0;
	char *path_str = "";
    struct sm5513_ccic_info *ccic = g_sm5513_ccic;

	switch (path)	{
	case PATH_EDGE:
		mt_set_gpio_out(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_OUT_ONE);
		sm5513_set_mux_channel(MUX_CH_0);
		break;

	case PATH_UART:
		mt_set_gpio_out(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_OUT_ZERO);
		sm5513_set_mux_channel(MUX_CH_1);
		break;

	case PATH_SBU:
		mt_set_gpio_out(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_OUT_ZERO);
		sm5513_set_mux_channel(MUX_CH_2);
		break;

	case PATH_OPEN:
		mt_set_gpio_out(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_OUT_ZERO);
		sm5513_set_mux_channel(MUX_CH_0);
		break;

	case PATH_DEFAULT:
		if (ccic->path_default == PATH_EDGE)	{
			mt_set_gpio_out(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_OUT_ONE);
			sm5513_set_mux_channel(MUX_CH_0);
			path	= PATH_EDGE;
		} else if (ccic->path_default == PATH_UART)	{
			mt_set_gpio_out(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_OUT_ZERO);
			sm5513_set_mux_channel(MUX_CH_1);
			path	= PATH_UART;
		} else if (ccic->path_default == PATH_OPEN) {
			mt_set_gpio_out(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_OUT_ZERO);
			sm5513_set_mux_channel(MUX_CH_0);
			path    = PATH_OPEN;
		} else {
			mt_set_gpio_out(GPIO_SEL_USBID_EDGE|0x80000000, GPIO_OUT_ZERO);
			sm5513_set_mux_channel(MUX_CH_2);
			path	= PATH_SBU;
		}
		break;
	}

	/* wait to swtich path - 15ms */
	mdelay(15);

	/* recheck path */
	if ((path == PATH_EDGE) && (!gpio_get_value(GPIO_SEL_USBID_EDGE))) {
		check_path	= 1;
	} else if (path >= PATH_UART) {
		if (gpio_get_value(GPIO_SEL_USBID_EDGE)) {
			check_path	= 1;
		} else {
			get_channel	= sm5513_get_mux_channel();
			if ((path == PATH_UART) && (get_channel == MUX_CH_1))
				check_path	= 0;
			else if ((path == PATH_SBU) && (get_channel == MUX_CH_2))
				check_path	= 0;
			else if ((path == PATH_OPEN) && (get_channel == MUX_CH_0))
				check_path	= 0;
			else
				check_path	= 1;
		}
	}

	if (check_path)	{
		sm5513_err_msg("%s: %s path %d switch failed !!\n", CCIC_DEV_NAME, __func__, path);
		return	1;
	}

	path_str = switch_path_type_print(path);

	sm5513_err_msg("%s: %s path %s(%d) switch !!\n",
			CCIC_DEV_NAME, __func__, path_str, path);

	/* If condition, edge_irq enable/disable */
	return 0;
}

#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION

#ifdef CONFIG_DUAL_ROLE_USB_INTF
void notify_moisture(void)
{
	struct sm5513_ccic_info *ccic = g_sm5513_ccic;

	dual_role_instance_changed(ccic->dual_role_phy);

}
#endif
void sm5513_i2c_sbu_path(int path)
{
	struct sm5513_ccic_info *ccic = g_sm5513_ccic;
	switch (path) {
	case SBU_PATH_UART:
		sm5513_write_reg(ccic->i2c, SM5513_REG_SWCNTL, 0x28);
		break;
	case SBU_PATH_USBID:
		sm5513_write_reg(ccic->i2c, SM5513_REG_SWCNTL, 0x10);
		break;
	case SBU_PATH_OPEN:
	default:
		sm5513_write_reg(ccic->i2c, SM5513_REG_SWCNTL, 0x00);
		break;
	}

}
void sm5513_i2c_sbu_path_check(void)
{
	struct sm5513_ccic_info *ccic = g_sm5513_ccic;
	u8 val;
	sm5513_read_reg(ccic->i2c, SM5513_REG_SWCNTL, &val);
	printk("[Mois] reg(0x0E), val(%x)\n", val);
}

void sm5513_i2c_cc_disable(void)
{
	struct sm5513_ccic_info *ccic = g_sm5513_ccic;
	sm5513_write_reg(ccic->i2c, SM5513_REG_CC_CNTL4, 0x88);
}
void sm5513_i2c_cc_enable(void)
{
	struct sm5513_ccic_info *ccic = g_sm5513_ccic;
	sm5513_write_reg(ccic->i2c, SM5513_REG_CC_CNTL4, 0x82);
}

void sm5513_write_status_reg(u8 reg, u8 value)
{
	struct sm5513_ccic_info *ccic = g_sm5513_ccic;
	sm5513_write_reg(ccic->i2c, reg, value);
}

u8 sm5513_read_status_reg(void)
{
	struct sm5513_ccic_info *ccic = g_sm5513_ccic;
	u8 status1;
	sm5513_read_reg(ccic->i2c, SM5513_REG_STATUS1, &status1);
	printk("[Mois] SM5513_REG_STATUS1, status1=0x%x\n", status1);
	return status1;
}

u8 sm5513_read_status_reg2(void)
{
	struct sm5513_ccic_info *ccic = g_sm5513_ccic;
	u8 status2;
	sm5513_read_reg(ccic->i2c, SM5513_REG_STATUS2, &status2);
	//printk("[Mois] SM5513_REG_STATUS1, status1=0x%x\n", status2);
	return status2;
}

int sm5513_get_input_suspend(void)
{
	if (moisture_status_get())
		return 1;
	else
		return 0;
}
void sm5513_set_wake_lock(bool flag, int time) {
	if (flag) { // Set wakelock
		wake_lock_timeout(&g_sm5513_ccic->sm5513_moisture_wakelock, msecs_to_jiffies(time));
		atomic_set(&g_sm5513_ccic->is_wakelock, true);
		printk("[Mois][Wake] Set wake lock timeout!\n");
	} else { // Set wake unlock
		if (atomic_read(&g_sm5513_ccic->is_wakelock)) {
			wake_unlock(&g_sm5513_ccic->sm5513_moisture_wakelock);
			atomic_set(&g_sm5513_ccic->is_wakelock, false);
			printk("[Mois][Wake] Set wake unlock!\n");
		} else {
			printk("[Mois][Wake] Already set wake unlock!\n");
		}

	}

}
bool sm5513_is_wakelock(void) {
	if (atomic_read(&g_sm5513_ccic->is_wakelock))
		return true;
	else
		return false;
}
#endif

static int sm5513_ccic_probe(struct platform_device *pdev)
{
	struct sm5513_dev *sm5513 = dev_get_drvdata(pdev->dev.parent);
    struct sm5513_ccic_info *ccic;
    int ret = 0;

	ccic = kzalloc(sizeof(struct sm5513_ccic_info), GFP_KERNEL);
	if (!ccic) {
        sm5513_err_msg("%s: fail to kalloc for sm5513_ccic\n", CCIC_DEV_NAME);
		return -ENOMEM;
    }
    ccic->typec = kzalloc(sizeof(struct usbtypc), GFP_KERNEL);
	if (!ccic->typec) {
        sm5513_err_msg("%s: fail to kalloc for usb_typec\n", CCIC_DEV_NAME);
		return -ENOMEM;
    }

	ccic->dev = &pdev->dev;
	ccic->i2c = sm5513->i2c;
#ifdef CONFIG_DUAL_ROLE_USB_INTF
    ccic->role_changing = CC_OP_MODE_DEFAULT;
#endif
    ccic->connected_device = 0;
    ccic->connected_host = 0;
    ccic->rev_id = (sm5513->device_id >> 4) & 0xf;
	ccic->is_present = 0;
	ccic->path_default	= PATH_OPEN; // DEFAULT OPEN
	ccic->vbus_rd	= 0;

#ifdef CONFIG_LGE_MODIFY_MODEL
	g_modify_model = lge_get_modify_model();
	sm5513_err_msg("[Mois] This model water detection state = %s\n",
			g_modify_model == true ? "DISABLE(ALPHA)" : "ENABLE(NORMAL)");
#endif
    sm5513_ccic_init_hw_configuration(ccic);

	sm5513_switch_init(ccic);

	mutex_init(&ccic->mutex_lock);

	g_sm5513_ccic = ccic;

	sm5513_switch_path(PATH_DEFAULT);

	ccic->irq = gpio_to_irq(sm5513->pdata->irq_gpio);
	sm5513_err_msg("%s: ccic_irq=%d\n", CCIC_DEV_NAME, ccic->irq);

	platform_set_drvdata(pdev, ccic);

#ifdef CONFIG_LGE_USB_MOISTURE_DETECTION
#ifdef CONFIG_LGE_MODIFY_MODEL
	if (!g_modify_model)
		moisture_det_create(ccic->irq);
#endif
#endif

	/* Set Dual Role */
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	ret = register_dual_role_instance(ccic);
	if (ret < 0) {
		goto err_dual_role;
	}
#endif
	g_sm5513_ccic->time_stamp = ktime_get();
	wake_lock_init(&g_sm5513_ccic->sm5513_moisture_wakelock, WAKE_LOCK_SUSPEND, "SM5513_MOIS.lock");

	/* Set Power_supply */
	ccic->cc_psy	= sm5513_psy;
	ret		= power_supply_register(&ccic->i2c->dev, &ccic->cc_psy);
	if (ret < 0) {
		sm5513_err_msg("%s: sm5513 power supply register failed. - %d\n", CCIC_DEV_NAME, ret);
		goto err_supply;
	}

	/* Enable Irq. */
    ret = request_threaded_irq(ccic->irq, NULL, ccic_irq_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT, "ccic-irq", ccic);
    if (ret) {
        sm5513_err_msg("%s: fail to request IRQ (ret=%d)\n", CCIC_DEV_NAME, ret);
        goto err_irq;
    }
	sm5513_enable_cc_detection(ENABLE);

    sm5513_err_msg("%s: Probe Done (rev_id=0x%x).\n", CCIC_DEV_NAME, ccic->rev_id);
    return 0;

err_irq:
err_supply:
	power_supply_unregister(&ccic->cc_psy);
err_dual_role:
#ifdef CONFIG_DUAL_ROLE_USB_INTF
    devm_dual_role_instance_unregister(&pdev->dev, ccic->dual_role_phy);
#endif
	mutex_destroy(&ccic->mutex_lock);
    kfree(ccic->typec);
    kfree(ccic);

    return ret;
}

static int sm5513_ccic_remove(struct platform_device *pdev)
{
	struct sm5513_ccic_info *ccic = platform_get_drvdata(pdev);

#ifdef CONFIG_DUAL_ROLE_USB_INTF
    devm_dual_role_instance_unregister(&pdev->dev, ccic->dual_role_phy);
#endif
    mutex_destroy(&ccic->mutex_lock);    
    kfree(ccic->typec);
    kfree(ccic);

    return 0;
}

static void sm5513_ccic_shutdown(struct platform_device *pdev)
{
	struct sm5513_ccic_info *ccic = platform_get_drvdata(pdev);

	/* set channel switch for factory */
	sm5513_switch_path(PATH_SBU);

	/* Release ENnOEPIN,ENSELPIN,ENnSINKPIN to Default */
	sm5513_update_reg(ccic->i2c, SM5513_REG_SWCNTL, 0x03, 0x07);

    /* reduce of current consumption on power off mode */
    sm5513_set_ccic_op_mode(ccic->i2c, CC_OP_MODE_SNK_ONLY);
    sm5513_update_reg(ccic->i2c, 0x21, (0x1 << 6), (0x1 << 6));

	wake_lock_destroy(&ccic->sm5513_moisture_wakelock);
	atomic_set(&ccic->is_wakelock, false);

	/* Release vbus ignore */
	if (ccic->rev_id >= SM5513_REVISION_3) {
		sm5513_update_reg(ccic->i2c, SM5513_REG_H_SPARE1, (0x0 << 7), (0x1 << 7));
	}

}

static struct platform_driver sm5513_ccic_driver = {
	.probe		= sm5513_ccic_probe,
	.remove		= sm5513_ccic_remove,
	.driver = {
		.name	= "sm5513-ccic",
		.owner	= THIS_MODULE,
	},
    .shutdown   = sm5513_ccic_shutdown,
};

static int __init sm5513_usb_init(void)
{
	return platform_driver_register(&sm5513_ccic_driver);
}
module_init(sm5513_usb_init);

static void __exit sm5513_usb_exit(void)
{
	platform_driver_unregister(&sm5513_ccic_driver);
}
module_exit(sm5513_usb_exit);


