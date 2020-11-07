#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <mt-plat/upmu_common.h>
#include <soc/mediatek/lge/lge_cable_id.h>
#include <soc/mediatek/lge/board_lge.h>

#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>

#ifdef CONFIG_LGE_USB_TYPE_C
#include <linux/mfd/sm5513.h>
#include <linux/usb/class-dual-role.h>
#endif

#define MOD "[LGE_CABLE_ID]"

#define CHANNEL_USB_ID 7
#if defined CONFIG_LGE_USB_EMBEDDED_BATTERY
#define MAX_USB_ID_COUNT 2
#define MAX_USB_VOLTAGE_COUNT 4
#define CABLE_VOLTAGE_DIFFERENCE 100
#define NORMAL_CABLE_VOLTAGE 1800
#else
#define MAX_USB_ID_COUNT 3
#endif

#ifdef CONFIG_MACH_MT6750_TH8
#define NORMAL_CABLE_VOLTAGE 1800
#endif

#define GPIO_23_USB_ID         (GPIO23 | 0x80000000)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)        (sizeof(x) / sizeof((x)[0]))
#endif

#define CABLE_NO_VALUE               0
#define CABLE_56K_VALUE             56
#define CABLE_130K_VALUE           130
#define CABLE_910K_VALUE           910


typedef struct  {
    int adc_min;
    int adc_max;
    usb_cable_type cable_type;
    int cable_value;
} usb_cable_adc_type;


#if defined(CONFIG_LGE_PM_USB_ID_R_100K)
/* LDO 1.8V, R1 100k */
__attribute__((weak)) usb_cable_adc_type lge_cable_id_table[] = {
/*                   adc_min      adc_max        cable_type          cable_value */
/* _56K     */  {   586,     706,      LT_CABLE_56K,   CABLE_56K_VALUE},
/* _130K    */  {   947,    1087,     LT_CABLE_130K,   CABLE_130K_VALUE},
/* _910K    */  {  1521,    1699,     LT_CABLE_910K,   CABLE_910K_VALUE},
/* _USER    */  {  1700,    1900,   USB_CABLE_400MA,   CABLE_NO_VALUE},
};
#elif defined(CONFIG_LGE_USB_TYPE_C)
/* In SM5513 typeC model, USB_ID is connected with SBU. CC controller	*
   could control this path. But it has ovp resistance(about 1000k).		*
   So USB ID table has voltage drop.									*/
/* LDO 1.8V, R1 200k, TypeC - EVB */
__attribute__((weak)) usb_cable_adc_type lge_cable_id_table_1_8v_200k[] = {
/*                  adc_min     adc_max     cable_type  */
/* _56K     */  {   342,      411,        LT_CABLE_56K,   CABLE_56K_VALUE},
/* _130K    */  {   604,      706,       LT_CABLE_130K,   CABLE_130K_VALUE},
/* _910K    */  {  1201,     1321,       LT_CABLE_910K,   CABLE_910K_VALUE},
/* _USER    */  {  1322,     1900,     USB_CABLE_400MA,   CABLE_NO_VALUE},
};
/* LDO 1.0V, R1 240k, TypeC - From RevA */
__attribute__((weak)) usb_cable_adc_type lge_cable_id_table_1_0v_240k[] = {
/*                  adc_min     adc_max     cable_type  */
/* _56K     */  {   120,      236,        LT_CABLE_56K,   CABLE_56K_VALUE},
/* _130K    */  {   237,      397,       LT_CABLE_130K,   CABLE_130K_VALUE},
/* _910K    */  {   562,      721,       LT_CABLE_910K,   CABLE_910K_VALUE},
/* _USER    */  {   722,     1900,     USB_CABLE_400MA,   CABLE_NO_VALUE},
};
usb_cable_adc_type *lge_cable_id_table = NULL;
#else
/* LDO 1.8V, R1 620k */
__attribute__((weak)) usb_cable_adc_type lge_cable_id_table[] = {
/*                  adc_min     adc_max     cable_type  */
/* _56K     */  {   85,    209,      LT_CABLE_56K,    CABLE_56K_VALUE},
/* _130K    */  {   228,    356,     LT_CABLE_130K,   CABLE_130K_VALUE},
/* _910K    */  {   972,   1172,     LT_CABLE_910K,   CABLE_910K_VALUE},
/* _USER    */  {  1600,   1900,   USB_CABLE_400MA,   CABLE_NO_VALUE},
};
#endif

static int g_cable_voltage = 0;
static int g_cable_value = CABLE_NO_VALUE;
static usb_cable_type g_cable_type = NO_INIT_CABLE;

static usb_cable_type lge_boot_cable = NO_INIT_CABLE;

static DEFINE_MUTEX(lge_cable_mutex);

/* CAUTION: These strings are come from LK. */
static char *cable_str[] = {
	" "," "," "," "," "," ",
	"LT_56K",
	"LT_130K",
	"400MA",
	"DTC_500MA",
	"Abnormal_400MA",
	"LT_910K",
	"NO_INIT",
};

static int __init board_cable_setup(char *cable_info)
{
	int	i;
	lge_boot_cable = NO_INIT_CABLE;
	for (i = LT_CABLE_56K; i <= NO_INIT_CABLE; i++)	{
		if(!strncmp(cable_info, cable_str[i], (strlen(cable_info) > strlen(cable_str[i])) ? strlen(cable_info) : strlen(cable_str[i]))) {
			lge_boot_cable	= (usb_cable_type) i;
			/* it is defined externally in <asm/system_info.h> */
			/* system_rev = lge_bd_rev; */
			break;
		}
	}

	printk(KERN_ERR "unified LK bootcmd lge.cable setup: %s\n", cable_str[lge_boot_cable]);

	return	1;
}
__setup("lge.cable=", board_cable_setup);

usb_cable_type lge_get_board_cable(void)
{
	return	lge_boot_cable;
}

bool lge_is_factory_cable_boot(void)
{
	switch (lge_boot_cable) {
		case LT_CABLE_56K :
		case LT_CABLE_130K :
		case LT_CABLE_910K :
			return true;
		default:
			return false;
	}
}

int lge_read_cable_voltage(int* voltage)
{
	int adc_voltage;

	#ifndef CONFIG_LGE_USB_TYPE_C
	mt_set_gpio_mode(GPIO_23_USB_ID, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_23_USB_ID, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_23_USB_ID, GPIO_PULL_DISABLE);
	#endif

	mdelay(50);

	adc_voltage = PMIC_IMM_GetOneChannelValue(CHANNEL_USB_ID, 5, 0);
#ifdef CONFIG_MACH_MT6750_TH8
	/* Added feature to bypass USBID ADC issue caused by USBID pin absent. */
	if (lge_get_board_revno() == HW_REV_A)
		adc_voltage = NORMAL_CABLE_VOLTAGE;
#endif
#ifdef CONFIG_MACH_MT6750_LV7V
	/* Added feature to bypass USBID ADC issue caused by USBID pin Pullup 1V. */
	if (lge_get_board_revno() == HW_REV_0) {
		adc_voltage = NORMAL_CABLE_VOLTAGE;
		pr_info("%s %s Rev_0 Force cable voltage : %d\n", MOD, __func__, adc_voltage);
	}
#endif
	if (adc_voltage < 0) {
		pr_err("%s Failed to read usb id\n", __func__);
		*voltage = 0;
		return -1;
	}
	pr_notice("%s usb id voltage: %d\n", MOD, adc_voltage);

	*voltage = adc_voltage;
	return 0;
}

int lge_get_cable_voltage(void)
{
	pr_info("%s %s cable voltage : %d\n", MOD, __func__, g_cable_voltage);
	return g_cable_voltage;
}

int lge_get_cable_value(void)
{
	pr_info("%s %s cable value : %d\n", MOD, __func__, g_cable_value);
	return g_cable_value;
}

usb_cable_type lge_get_cable_type(void)
{
	static int p_count;
	lge_read_cable_type();

	if ((g_cable_type != NO_INIT_CABLE) && (p_count++ >= 5)) {
		pr_info("%s %s USB Cable ID : %d\n", MOD, __func__, g_cable_type);
		p_count = 0;
	}

	return g_cable_type;
}

usb_cable_type lge_get_cable_type_selfd(void)
{
	static int p_count;
	lge_read_cable_type_selfd();

	if (p_count++ >= 5) {
		pr_info("%s %s USB Cable ID : %d\n", MOD, __func__, g_cable_type);
		p_count = 0;
	}

	return g_cable_type;
}

/* This function will be used before sm5513 chip revision */
#ifdef CONFIG_LGE_USB_TYPE_C
usb_cable_type memoried_lge_read_cable_type(void)
{
	/* pr_err("%s %s memoried type %d\n", MOD, __func__,g_cable_type); */
	return g_cable_type;
}
#endif

bool lge_is_factory_cable(void)
{
	/* Used memoried value because of switching */
	#ifdef CONFIG_LGE_USB_TYPE_C
	switch (memoried_lge_read_cable_type()) {
	case LT_CABLE_56K :
	case LT_CABLE_130K :
	case LT_CABLE_910K :
		return true;
	default:
		return false;
	}
	#else
	switch (lge_get_cable_type()) {
		case LT_CABLE_56K :
		case LT_CABLE_130K :
		case LT_CABLE_910K :
			return true;
		default:
			return false;
	}
	#endif
}

unsigned int lge_reset_cable_type(void)
{
	mutex_lock(&lge_cable_mutex);
	if (g_cable_type != NO_INIT_CABLE) {
		g_cable_voltage = 0;
		g_cable_value = CABLE_NO_VALUE;
		g_cable_type = NO_INIT_CABLE;
		pr_err("%s %s\n", MOD, __func__);
	}
	mutex_unlock(&lge_cable_mutex);
	return g_cable_type;
}

#ifdef CONFIG_LGE_USB_EMBEDDED_BATTERY
int lge_read_factory_cable_voltage(int* voltage)
{
	bool normal_case = false;
	int i = 0, cable_voltage = 0;
	int cable_voltage_data[2] = {0};

	do {
		if (i != 0) msleep(10);

		if (lge_read_cable_voltage(&cable_voltage) < 0)
			return -1;
		cable_voltage_data[0] = cable_voltage;

		msleep(20);

		if (lge_read_cable_voltage(&cable_voltage) < 0)
			return -1;

		cable_voltage_data[1] = cable_voltage;

		if(abs(cable_voltage_data[1] - cable_voltage_data[0]) < CABLE_VOLTAGE_DIFFERENCE) {
			normal_case = true;
			break;
		}
	} while (!normal_case && (++i < MAX_USB_VOLTAGE_COUNT));

	*voltage = cable_voltage;

	return 0;
}

int lge_read_check_cable_voltage(int* voltage)
{
	bool abnormal_cable = false;
	int i = 0, j = 0, cable_voltage = 0;
	int cable_voltage_data[MAX_USB_VOLTAGE_COUNT] = {0};

	do {
		if (i != 0) msleep(10);

		if (lge_read_cable_voltage(&cable_voltage) < 0)
			return -1;

		cable_voltage_data[i] = cable_voltage;

		for(j = 1; j < i + 1; j++) {
			/* Assume that the cable is normal when the differences are over 100 mV */
			if(abs(cable_voltage_data[i] - cable_voltage_data[i-j]) > CABLE_VOLTAGE_DIFFERENCE) {
				abnormal_cable = true;
				cable_voltage = NORMAL_CABLE_VOLTAGE;
				pr_err("%s %s Read abnormal cable_voltage\n", MOD, __func__);
				break;
			}
		}
	} while (!abnormal_cable && (++i < MAX_USB_VOLTAGE_COUNT));

	*voltage = cable_voltage;

	return 0;
}
#endif

usb_cable_type lge_read_cable_type(void)
{
	int count = 0, cable_enum = 0, ret = 0;
#if defined(CONFIG_LGE_PM_USB_ID_R_100K)
	int table_size = ARRAY_SIZE(lge_cable_id_table);
#elif defined(CONFIG_LGE_USB_TYPE_C)
	int table_size = ARRAY_SIZE(lge_cable_id_table_1_0v_240k);
	if (lge_get_board_revno() == HW_REV_0)
		lge_cable_id_table	= lge_cable_id_table_1_8v_200k;
	else
		lge_cable_id_table	= lge_cable_id_table_1_0v_240k;
#else
	int table_size = ARRAY_SIZE(lge_cable_id_table);
#endif

	mutex_lock(&lge_cable_mutex);

	if (g_cable_type != NO_INIT_CABLE) {
		mutex_unlock(&lge_cable_mutex);
		pr_debug("%s %s: already done\n", MOD, __func__);
		return g_cable_type;
	}

#ifdef CONFIG_USB_MTK_DUALMODE
#ifdef CONFIG_LGE_USB_TYPE_C
	if (sm5513_is_factory_cable()) {
		sm5513_switch_path(PATH_SBU);
	} else {
		mutex_unlock(&lge_cable_mutex);
		return lge_reset_cable_type();
	}
#else
	if (mtk_switch_id_adc_mode()) {
		mutex_unlock(&lge_cable_mutex);
		return lge_reset_cable_type();
	}
#endif
#endif

	do {
		if (count != 0) msleep(10);

#ifdef CONFIG_LGE_USB_EMBEDDED_BATTERY
		if (lge_is_factory_cable_boot())
			ret = lge_read_factory_cable_voltage(&g_cable_voltage);
		else
			ret = lge_read_check_cable_voltage(&g_cable_voltage);
#else
		ret = lge_read_cable_voltage(&g_cable_voltage);
#endif
		if (ret) {
#ifdef CONFIG_USB_MTK_DUALMODE
	#ifdef CONFIG_LGE_USB_TYPE_C
			sm5513_switch_path(PATH_DEFAULT);
	#else
			mtk_switch_id_dig_mode();
	#endif
#endif
			mutex_unlock(&lge_cable_mutex);
			return lge_reset_cable_type();
		}

		for (cable_enum = 0; cable_enum < table_size; cable_enum++) {
			if ((lge_cable_id_table[cable_enum].adc_min <= g_cable_voltage) &&
				(lge_cable_id_table[cable_enum].adc_max >= g_cable_voltage))
	            break;
	    }
	} while ((cable_enum == table_size) && (++count < MAX_USB_ID_COUNT));

#ifdef CONFIG_USB_MTK_DUALMODE
	#ifdef CONFIG_LGE_USB_TYPE_C
	sm5513_switch_path(PATH_DEFAULT);
	#else
	mtk_switch_id_dig_mode();
	#endif
#endif

	if (cable_enum == table_size) {
		g_cable_type = ABNORMAL_USB_CABLE_400MA;
		g_cable_value = CABLE_NO_VALUE;
	} else {
		g_cable_type = lge_cable_id_table[cable_enum].cable_type;
		g_cable_value = lge_cable_id_table[cable_enum].cable_value;
	}

	mutex_unlock(&lge_cable_mutex);

	pr_err("%s %s usb cable type: %s(%d): %d mV\n", MOD, __func__,
		cable_str[g_cable_type], g_cable_type, g_cable_voltage);

    return g_cable_type;
}

usb_cable_type lge_read_cable_type_selfd(void)
{
	int count = 0, cable_enum = 0, ret = 0;
#if defined(CONFIG_LGE_PM_USB_ID_R_100K)
	int table_size = ARRAY_SIZE(lge_cable_id_table);
#elif defined(CONFIG_LGE_USB_TYPE_C)
	int table_size = ARRAY_SIZE(lge_cable_id_table_1_0v_240k);
	if (lge_get_board_revno() == HW_REV_0)
		lge_cable_id_table	= lge_cable_id_table_1_8v_200k;
	else
		lge_cable_id_table	= lge_cable_id_table_1_0v_240k;
#else
	int table_size = ARRAY_SIZE(lge_cable_id_table);
#endif

	mutex_lock(&lge_cable_mutex);

#ifdef CONFIG_USB_MTK_DUALMODE
#ifdef CONFIG_LGE_USB_TYPE_C
	sm5513_switch_path(PATH_SBU);
#else
	if (mtk_switch_id_adc_mode()) {
		mutex_unlock(&lge_cable_mutex);
		return lge_reset_cable_type();
	}
#endif
#endif

	do {
		if (count != 0) msleep(10);

#ifdef CONFIG_LGE_USB_EMBEDDED_BATTERY
		if (lge_is_factory_cable_boot())
			ret = lge_read_factory_cable_voltage(&g_cable_voltage);
		else
			ret = lge_read_check_cable_voltage(&g_cable_voltage);
#else
		ret = lge_read_cable_voltage(&g_cable_voltage);
#endif
		if (ret) {
#ifdef CONFIG_USB_MTK_DUALMODE
	#ifdef CONFIG_LGE_USB_TYPE_C
			sm5513_switch_path(PATH_DEFAULT);
	#else
			mtk_switch_id_dig_mode();
	#endif
#endif
			mutex_unlock(&lge_cable_mutex);
			return lge_reset_cable_type();
		}

		for (cable_enum = 0; cable_enum < table_size; cable_enum++) {
			if ((lge_cable_id_table[cable_enum].adc_min <= g_cable_voltage) &&
				(lge_cable_id_table[cable_enum].adc_max >= g_cable_voltage))
	            break;
	    }
	} while ((cable_enum == table_size) && (++count < MAX_USB_ID_COUNT));

#ifdef CONFIG_USB_MTK_DUALMODE
	#ifdef CONFIG_LGE_USB_TYPE_C
	sm5513_switch_path(PATH_DEFAULT);
	#else
	mtk_switch_id_dig_mode();
	#endif
#endif

	if (cable_enum == table_size) {
		g_cable_type = ABNORMAL_USB_CABLE_400MA;
		g_cable_value = CABLE_NO_VALUE;
	} else {
		g_cable_type = lge_cable_id_table[cable_enum].cable_type;
		g_cable_value = lge_cable_id_table[cable_enum].cable_value;
	}

	mutex_unlock(&lge_cable_mutex);

	pr_err("%s %s usb cable type: %s(%d): %d mV\n", MOD, __func__,
		cable_str[g_cable_type], g_cable_type, g_cable_voltage);

    return g_cable_type;
}

#ifdef CONFIG_LGE_USB_TYPE_C
bool usb_c_get_power_3p0(void)
{
    int cc_value = 0;

    cc_value = cc_controller_get_cc_value();

    if (cc_value == DUAL_ROLE_PROP_CC_RP_DEFAULT)
        pr_info("%s : RP_DEFAULT\n", __func__);
    else if (cc_value == DUAL_ROLE_PROP_CC_RP_POWER1P5)
        pr_info("%s : RP_POWER1P5\n", __func__);
    else if (cc_value == DUAL_ROLE_PROP_CC_RP_POWER3P0)
        pr_info("%s : RP_POWER3P0\n", __func__);
    else if (cc_value == DUAL_ROLE_PROP_CC_OPEN)
        pr_info("%s : OPEN\n", __func__);
    else
        pr_info("%s : Nothing\n", __func__);

    if (cc_value == DUAL_ROLE_PROP_CC_RP_POWER3P0)
        return true;
    else
        return false;
}
#endif
