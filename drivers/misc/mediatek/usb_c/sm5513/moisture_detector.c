#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <mt_gpio.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/battery_common.h>
#include <linux/power_supply.h>
#include <linux/mfd/sm5513-private.h>
#include <typec.h>
#include "moisture_detector.h"

int cc_wet_state_table[6][2] = { // {Time(ms), Repeat}
								{1 * 1000, 1}, // 1s, 1 repeat
								{6 * 1000, 1}, // 6s, 1 repeat
								{60 * 1000, 10}, // 1 minute, 10 repeat => 10 mins
								{600 * 1000, 12}, // 10 mins, 12 repeat => 2 hours
								{1800 * 1000, 32767}, // 30 mins, inf repeat => infinity
								{0,0}}; // resolved

struct mois_detc{
	struct device		dev;
	struct delayed_work	check_moisture_state_work;
	struct delayed_work judge_moisture_irq_work;
	struct delayed_work wait_dry_work;
	struct delayed_work wait_dry_work_for_cc;
	struct mutex mutex_lock;
	int			adc_state;
	int			pre_adc_state;
	int			irq;
	int			judge_count;
	int			counter;
	int			edge_irq_num;
	int			cc_irq_num;
	bool		real_irq;
	spinlock_t	lock;
	int			cc_wet_state;
	bool		pr_moisture_en;
	bool		pr_moisture;
	bool		cc_enable_flag;
	atomic_t	is_judging;
};

static struct mois_detc *g_mois_detctor;

#ifdef CONFIG_DUAL_ROLE_USB_INTF
extern void notify_moisture(void);
#endif
extern void sm5513_i2c_sbu_path(int open);
extern void sm5513_i2c_sbu_path_check(void);
extern void sm5513_i2c_cc_disable(void);
extern void sm5513_i2c_cc_enable(void);
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int sm5513_enable_cc_detection(bool enable);
extern u8 sm5513_read_status_reg(void);
extern u8 sm5513_read_status_reg2(void);
extern void sm5513_write_status_reg(u8 reg, u8 value);
extern bool sm5513_is_factory_cable(void);
extern void sm5513_set_wake_lock(bool flag, int time);
extern bool sm5513_is_wakelock(void);
static struct kobject *debug_kobj;

#define EDGE_DTE_IRQ    9
#define EDG_MOISTURE_ADC_CHANNEL    7 //14
#define CHANNEL_USB_ID	7
#define SEL_USBID_EDGE	11

#define EDGE_PATH	1
#define SBU_PATH	0
static int get_usbid_edge_path(void)
{
	int ret = 0;

	ret = gpio_get_value(SEL_USBID_EDGE);

	return ret;
}

static int get_sbu_adc_value (int* sbu_voltage)
{
	int sbu_adc_voltage = 0;
	int ret = 0;

	mutex_lock(&g_mois_detctor->mutex_lock);

	ret = sm5513_current_path(); //current path

	sm5513_i2c_sbu_path(SBU_PATH_USBID);

	msleep(50);

	sbu_adc_voltage = PMIC_IMM_GetOneChannelValue(CHANNEL_USB_ID, 5, 0);
	if (sbu_adc_voltage < 0) {
		mois_err_msg("[Mois]%s Failed to read SBU_adc\n", __func__);
		return -1;
	}

//	if (ret != PATH_SBU) {
		sm5513_i2c_sbu_path(SBU_PATH_OPEN);
//	}
	*sbu_voltage = sbu_adc_voltage;

	mutex_unlock(&g_mois_detctor->mutex_lock);
	return 0;
}

#define EDGE_WET_THRESHOLD 830 //1300
#ifdef CONFIG_MACH_MT6750S_CV7A
#define SBU_WET_THRESHOLD 548	//452 // 630 // 800
#else
#define SBU_WET_THRESHOLD 452 // 630 // 800
#endif
#define SBU_SHORT_THRESHOLD 1200
#define SBU_GND_THRESHOLD 30 // ADD GND_THRESHOLD

static bool is_sbu_wet (int* sbu_voltage)
{
	int sbu_adc;

	get_sbu_adc_value(&sbu_adc);

	*sbu_voltage = sbu_adc;

	if (((sbu_adc < SBU_WET_THRESHOLD) && (sbu_adc > SBU_GND_THRESHOLD)) ||
			((lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) && (sbu_adc >= SBU_SHORT_THRESHOLD)))
		return true;
	else
		return false;
}

static bool is_sbu_dry (int* sbu_voltage)
{
	int sbu_adc;

	if (BMT_status.charger_type != CHARGER_UNKNOWN){ // inserted usb cable
		mois_err_msg("[Mois]%s USB(TA) cable INSERTED! always return false!\n", __func__);
		return false;
	}

	get_sbu_adc_value(&sbu_adc);
	*sbu_voltage = sbu_adc;

	if(((sbu_adc >= SBU_WET_THRESHOLD) && (sbu_adc <= SBU_SHORT_THRESHOLD)) || (sbu_adc <= SBU_GND_THRESHOLD))
		return true;
	else
		return false;
}

static bool is_cc_wet (void)
{
	if(sm5513_read_status_reg() & 0x80)
		return true;
	else
		return false;
}

static void moisture_irq_enable(void)
{
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) { //ONLY check chargerlogo mode
		sm5513_write_status_reg(SM5513_REG_INTMASK1, 0x34);     /* Enable CC_ST_CHG / CC_ABNORMAL/ SRC_ADV_CHG / ATTACH / DETACH */
	} else {
		sm5513_write_status_reg(SM5513_REG_INTMASK1, 0x74);     /* Enable CC_ST_CHG / SRC_ADV_CHG / ATTACH / DETACH */
	}
	mois_dbg_msg("[Mois]%s Enable Moisture IRQ!!!\n",__func__);
}

static void moisture_irq_disable(void)
{
	sm5513_write_status_reg(SM5513_REG_INTMASK1, SM5513_INTMSK1_CC_ST_CHG | SM5513_INTMSK1_CC_ABNORMAL);
	mois_dbg_msg("[Mois]%s Disable Moisture IRQ!\n",__func__);
}

void moisture_en_set(bool enable)
{
#ifdef CONFIG_LGE_MODIFY_MODEL
	if (sm5513_get_modify_model() == true)
		return;
#endif
	if (enable == true)	{
		g_mois_detctor->pr_moisture_en	= true;
	} else {
		g_mois_detctor->pr_moisture_en	= false;
	}
}

bool moisture_en_get(void)
{
#ifdef CONFIG_LGE_MODIFY_MODEL
	if (sm5513_get_modify_model() == true)
		return false;
#endif
	if (g_mois_detctor->pr_moisture_en)
		return true;
	else
		return false;
}

bool moisture_status_get(void)
{
#ifdef CONFIG_LGE_MODIFY_MODEL
	if (sm5513_get_modify_model() == true)
		return false;
#endif
	/* If moisture detection is not enabled */
	if (!g_mois_detctor->pr_moisture_en)
		return false;

	if (g_mois_detctor->pr_moisture)
		return true;
	else
		return false;
}

static int moisture_gpio_init(struct mois_detc *info)
{
	// GPIO setting
	mt_set_gpio_mode(EDGE_DTE_IRQ|0x80000000, GPIO_MODE_GPIO);
	mt_set_gpio_pull_enable(EDGE_DTE_IRQ|0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(EDGE_DTE_IRQ|0x80000000, GPIO_PULL_UP);
	mt_set_gpio_dir(EDGE_DTE_IRQ|0x80000000, GPIO_DIR_IN);

	return 0;
}

#define PERIOD_1SEC 1 * 1000 // 1sec
#define PERIOD_2SEC 2 * 1000 // 1sec
#define PERIOD_5SEC 5 * 1000 // 5sec
#define PERIOD_10SEC 10 * 1000 // 10sec
static void check_moisture_state_work(struct work_struct *w)
{
	struct mois_detc *mois_detctor =
		container_of(w, struct mois_detc, check_moisture_state_work.work);
	int sbu_voltage = 0;
	mois_dbg_msg("[Mois]%s pre_adc_state=%d adc_state=%d \n",
		__func__, g_mois_detctor->pre_adc_state, g_mois_detctor->adc_state);

	switch(g_mois_detctor->adc_state) {
	case ADC_STATE_DRY :
	case ADC_STATE_GND :
		if (!atomic_read(&g_mois_detctor->is_judging)) {
			if(!g_mois_detctor->cc_enable_flag) {
				sm5513_enable_cc_detection(ENABLE);
				g_mois_detctor->cc_enable_flag = true;
				mois_dbg_msg("[Mois]%s CC ENABLE!!!\n", __func__);
			}
		}

		/* Moisture status change */
		if (mois_detctor->pr_moisture)	{
			mois_detctor->pr_moisture	= false;
			mois_dbg_msg("[Mois]%s pr_moisture -> false!\n", __func__);
			if (sm5513_is_wakelock()) { // True (Before state is WET)
				sm5513_set_wake_lock(false, 0);
			}
			BMT_status.charging_enabled = KAL_TRUE;
//			sm5513_switch_path(PATH_SBU);
#ifdef CONFIG_DUAL_ROLE_USB_INTF
			notify_moisture();
#endif
			wake_up_bat3();		/* change for battery */
		}
		break;

	case ADC_STATE_WFD :
	case ADC_STATE_WET :
		if(g_mois_detctor->cc_enable_flag) {
			sm5513_enable_cc_detection(DISABLE);
			g_mois_detctor->cc_enable_flag = false;
			mois_dbg_msg("[Mois]%s CC DISABLE!!!\n", __func__);
		}

		/* Moisture status change */
		if (!mois_detctor->pr_moisture)	{
			mois_detctor->pr_moisture	= true;
			mois_dbg_msg("[Mois]%s pr_moisture -> true!\n", __func__);
			BMT_status.charging_enabled = KAL_FALSE;
			sm5513_switch_path(PATH_OPEN);
#ifdef CONFIG_DUAL_ROLE_USB_INTF
			notify_moisture();
#endif
			wake_up_bat3();		/* change for battery */
		}

		if (sm5513_is_wakelock()) { // True (Before state is WET)
			sm5513_set_wake_lock(true, 1050); // Reflash wakelock for 1050ms
		}
		schedule_delayed_work(&mois_detctor->check_moisture_state_work,
			msecs_to_jiffies(PERIOD_1SEC)); //1sec
		break;
	case ADC_STATE_WDT :
		//check the OVP or SBU >= 1.20V
		if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {// ONLY check chargerlogo mode
			get_sbu_adc_value(&sbu_voltage);
			if ((sbu_voltage >= SBU_SHORT_THRESHOLD) || (sm5513_read_status_reg2() & 0x2F)) {
				// OVP! (BSTOVP || DPDM_OVP || VCONN_OVP || CC1,2_OVP)
				mois_err_msg("[Mois]%s insert WET cable! immediately go to WET state\n",
						__func__);
				if(g_mois_detctor->cc_enable_flag) {
					sm5513_enable_cc_detection(DISABLE);
					g_mois_detctor->cc_enable_flag = false;
					mois_dbg_msg("[Mois]%s CC DISABLE!!! (WDT)\n", __func__);
				}
				g_mois_detctor->adc_state = ADC_STATE_WET;
				cancel_delayed_work(&mois_detctor->judge_moisture_irq_work);
				sm5513_set_wake_lock(true, 1050);
				schedule_delayed_work(&mois_detctor->check_moisture_state_work, msecs_to_jiffies(PERIOD_1SEC));
				schedule_delayed_work(&mois_detctor->wait_dry_work, 0);
			} else { // if not SHORT
				msleep(10);
				schedule_delayed_work(&mois_detctor->check_moisture_state_work,
						msecs_to_jiffies(PERIOD_1SEC)); // 1sec
			}
		}
		if (!atomic_read(&g_mois_detctor->is_judging)) {
			if(!g_mois_detctor->cc_enable_flag) {
				sm5513_enable_cc_detection(ENABLE);
				g_mois_detctor->cc_enable_flag = true;
				mois_dbg_msg("[Mois]%s CC ENABLE!!!\n", __func__);
			}
		}

		schedule_delayed_work(&mois_detctor->check_moisture_state_work,
				msecs_to_jiffies(PERIOD_1SEC)); // 1sec

		break;
	}
	g_mois_detctor->pre_adc_state = g_mois_detctor->adc_state;
}

#define DRY_COUNT 10
int g_loop_count = 0;

static void wait_dry_work(struct work_struct *w)
{
	struct mois_detc *mois_detctor = container_of(w, struct mois_detc,
		wait_dry_work.work);
	int sbu_voltage = 0;
	int dry_count = 0;
	int loop_count;

	mois_dbg_msg("[Mois]%s start \n", __func__);
	if (BMT_status.charger_type != CHARGER_UNKNOWN){ // inserted usb cable
		mois_err_msg("[Mois]%s USB(TA) cable INSERTED! do not execute wait_dry_work workqueue\n", __func__);
		msleep(1000);
		schedule_delayed_work(&mois_detctor->wait_dry_work, msecs_to_jiffies(PERIOD_10SEC));
		return ;
	}

	sm5513_switch_path(PATH_SBU);

	if (g_mois_detctor->cc_enable_flag) {
		sm5513_enable_cc_detection(DISABLE);
		g_mois_detctor->cc_enable_flag = false;
	}

	if (is_sbu_dry(&sbu_voltage)) {
		mois_err_msg("[Mois]%s Maybe almost Dry State.. sbu=%d\n",
				__func__, sbu_voltage);
		sm5513_switch_path(PATH_OPEN);
		for (loop_count = 0; loop_count < DRY_COUNT; loop_count++) {
			msleep(1000);
			sm5513_switch_path(PATH_SBU);
			if (is_sbu_dry(&sbu_voltage)) {
				dry_count = loop_count + 1;
				mois_err_msg("[Mois][SUB]%s Maybe almost Dry State.. sbu_sub(%d/%d)=%d\n",
				__func__, dry_count, DRY_COUNT, sbu_voltage);
			} else {
				mois_err_msg("[Mois][SUB]%s NOT YET DRY! sbu_sub(%d/%d) = %d\n",
						__func__, dry_count, DRY_COUNT, sbu_voltage);
				dry_count = 0;
				break;
			}
			sm5513_switch_path(PATH_OPEN);
		}

		if (dry_count < DRY_COUNT) {
			sm5513_switch_path(PATH_OPEN);
			mois_err_msg("[Mois]%s NOT YET DRY wait... sbu=%d\n",
					__func__, sbu_voltage);
			schedule_delayed_work(&mois_detctor->wait_dry_work, msecs_to_jiffies(PERIOD_10SEC));
		} else { // SBU dry complete... now check CC pin
			mois_err_msg("[Mois]%s SBU DRY complete. go to next stage\n",
					__func__);
			cancel_delayed_work(&mois_detctor->wait_dry_work_for_cc);
			mois_detctor->cc_wet_state = 0;
			msleep(10);
			g_loop_count = 0;
			schedule_delayed_work(&mois_detctor->wait_dry_work_for_cc, 0);
		}
	} else {
		sm5513_switch_path(PATH_OPEN);
		g_mois_detctor->adc_state = ADC_STATE_WET;
		mois_err_msg("[Mois]%s NOT YET DRY wait...... sbu=%d\n",
				__func__, sbu_voltage);
		cancel_delayed_work(&mois_detctor->wait_dry_work_for_cc);
		schedule_delayed_work(&mois_detctor->wait_dry_work, msecs_to_jiffies(PERIOD_10SEC));
	}
}

static void wait_dry_work_for_cc(struct work_struct *w)
{
	struct mois_detc *mois_detctor = container_of(w, struct mois_detc,
		wait_dry_work_for_cc.work);

	if(!g_mois_detctor->cc_enable_flag) {
		sm5513_enable_cc_detection(ENABLE);
		g_mois_detctor->cc_enable_flag = true;
	}
	msleep(10);

	if (!is_cc_wet()) { // if normal state
		g_mois_detctor->adc_state = ADC_STATE_DRY;
		g_mois_detctor->cc_wet_state = 0;
//		sm5513_switch_path(PATH_SBU);
		mois_err_msg("[Mois]%s DRY COMPLETE!!! CC status = %d\n",
			__func__, is_cc_wet());
		msleep(50);
		mois_err_msg("[Mois]%s moisture irq enable! (CC_WET_STATE_LV1)\n", __func__);
		moisture_irq_enable();
		return ;
	} else { // if detected water on cc pin
		if(!g_mois_detctor->cc_enable_flag) {
			sm5513_enable_cc_detection(DISABLE);
			g_mois_detctor->cc_enable_flag = false;
		}
		msleep(10);
		mois_err_msg("[Mois]%s cc wet level = %d, g_loop_count = %d/%d\n",
				__func__, g_mois_detctor->cc_wet_state, g_loop_count + 1, cc_wet_state_table[g_mois_detctor->cc_wet_state][1]);
		if (g_loop_count >= cc_wet_state_table[g_mois_detctor->cc_wet_state][1]) {
			g_loop_count = 0;

			mois_err_msg("[Mois]%s cc wet level has been promoted (%d -> %d)\n", __func__,
					g_mois_detctor->cc_wet_state, g_mois_detctor->cc_wet_state + 1);

			mois_detctor->cc_wet_state++;
			schedule_delayed_work(&mois_detctor->wait_dry_work_for_cc,
					msecs_to_jiffies(cc_wet_state_table[g_mois_detctor->cc_wet_state][0]));
		} else {
			g_loop_count++;
			schedule_delayed_work(&mois_detctor->wait_dry_work_for_cc,
					msecs_to_jiffies(cc_wet_state_table[g_mois_detctor->cc_wet_state][0]));
		}
	}
}

#define COUNT_SBU_ADC 10
#define DELAY_SBU_ADC 250
static void judge_moisture_irq_work(struct work_struct *w)
{
	int sbu_voltage = 0;
	int sbu_count=0;
	int loop_count;
	struct mois_detc *mois_detctor = container_of(w, struct mois_detc,
		judge_moisture_irq_work.work);

	msleep(150);

	atomic_set(&g_mois_detctor->is_judging, true); // judging flag set

	if (g_mois_detctor->cc_enable_flag) {
		sm5513_enable_cc_detection(DISABLE);
		g_mois_detctor->cc_enable_flag = false;
	}

	for (loop_count = 0; loop_count < COUNT_SBU_ADC; loop_count++) {
		mois_dbg_msg("[Mois]%s check sbu adc count %d\n",
				__func__, loop_count);
		sm5513_set_wake_lock(true, (DELAY_SBU_ADC + 100));
		msleep(DELAY_SBU_ADC);

		if (BMT_status.charger_type != CHARGER_UNKNOWN){ // inserted usb cable
			mois_err_msg("[Mois]%s USB(TA) cable INSERTED! during judge wet state. judge will be canceled!\n", __func__);
			loop_count = 0; // always not water state
			break; // exit for loop
		}

		sm5513_switch_path(PATH_SBU);
		if (is_sbu_wet(&sbu_voltage)) {
			sbu_count++;
		}
		sm5513_switch_path(PATH_OPEN);
		mois_err_msg("[Mois]%s check sbu adc %d\n",
				__func__, sbu_voltage);
	}

	if(sbu_count >= COUNT_SBU_ADC) {
		mois_err_msg("[Mois]%s It is WATER! change state\n", __func__);
		//1. state change - WET
		//2. Start wait drywork
		//3. finish this function
		g_mois_detctor->adc_state = ADC_STATE_WET;
		schedule_delayed_work(&mois_detctor->wait_dry_work, 0);
		atomic_set(&g_mois_detctor->is_judging, false); // judging flag unset
	} else {
		mois_err_msg("[Mois]%s It is NOT water! when check sbu adc\n",
			__func__);
		g_mois_detctor->adc_state = ADC_STATE_DRY;

		if (!g_mois_detctor->cc_enable_flag) {
			sm5513_enable_cc_detection(ENABLE);
			g_mois_detctor->cc_enable_flag = true;
		}

		atomic_set(&g_mois_detctor->is_judging, true); // judging flag unset
		mois_dbg_msg("[Mois]%s moisture irq enable! (SBU_COUNT >= COUNT_SBU_ADC)\n", __func__);
		moisture_irq_enable();
	}
}

void start_moisture_detection(int irq)
{
	struct mois_detc *mois_detctor = g_mois_detctor;

	mois_dbg_msg("[Mois] %s: start!\n", __func__);

	moisture_irq_disable();

	g_mois_detctor->adc_state = ADC_STATE_WDT;

	cancel_delayed_work(&mois_detctor->check_moisture_state_work);
	cancel_delayed_work(&mois_detctor->judge_moisture_irq_work);
	cancel_delayed_work(&mois_detctor->wait_dry_work);
	cancel_delayed_work(&mois_detctor->wait_dry_work_for_cc);

	schedule_delayed_work(&mois_detctor->check_moisture_state_work, 0);
	schedule_delayed_work(&mois_detctor->judge_moisture_irq_work, msecs_to_jiffies(PERIOD_1SEC));
}

static ssize_t show_selusb(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = get_usbid_edge_path();

	return sprintf(buf, "SEL_USBID_EDGE = %d\n", ret);
}

static ssize_t store_selusb(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) < 1)
		return size;

	pr_err("[Mois] Mositure Switch change to %d\n", value);
	msleep(10);
	switch (value) {
		case 1:
			pr_err("[Mois] Do not change to PATH_EDGE\n");
			break;
		case 0:
			sm5513_switch_path(PATH_SBU);
			break;
		default:
			break;
	}

	return size;
}

static ssize_t show_sbu_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int value = -1;

	ret = get_usbid_edge_path();

	get_sbu_adc_value(&value);

	return sprintf(buf, "edge_path = %d, sbu_value = %d\n",
			ret, value);
}

static ssize_t show_cc_enable_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int cc_status = 0;
	char *ret_char[4] = {"OPEN", "UART", "SBU", "FAULT"};
	char *cc_char[2] = {"DISABLE", "ENABLE"};

	ret = sm5513_current_path();
	cc_status = sm5513_current_cc_status();

	/* return value
	 * 1 : UART
	 * 2 : FACTORY/WATER (SBU)
	 * 3 : Fault
	 * 0 : Open
	 */

	return sprintf(buf, "real switch_path = %s, real_cc_status = %s, flag cc_status = %s\n",
			ret_char[ret], cc_char[cc_status] ,cc_char[g_mois_detctor->cc_enable_flag]);
}
static ssize_t store_cc_enable_test(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) < 1)
		return size;

	pr_err("[Mois] Temporary modify CC pin %d to %d\n", !value, !!value);
	msleep(10);

	value = !!value;

	switch (value) {
		case 1:
			if(!g_mois_detctor->cc_enable_flag) {
				sm5513_enable_cc_detection(ENABLE);
				g_mois_detctor->cc_enable_flag = true;
				mois_dbg_msg("[Mois]%s CC ENABLE using SYSFS\n", __func__);
			} else {
				mois_dbg_msg("[Mois] Already CC Enabled\n");
			}
			break;
		case 0:
			if(g_mois_detctor->cc_enable_flag) {
				sm5513_enable_cc_detection(DISABLE);
				g_mois_detctor->cc_enable_flag = false;
				mois_dbg_msg("[Mois]%s CC DISABLE using SYSFS\n", __func__);
			} else {
				mois_dbg_msg("[Mois] Already CC Disabled\n");
			}
			break;
		default:
			break;
	}

	return size;
}

static DEVICE_ATTR(sel_usb, S_IWUSR | S_IRUGO, show_selusb, store_selusb);
static DEVICE_ATTR(sbu_voltage, S_IRUSR | S_IRUGO, show_sbu_voltage, NULL);
static DEVICE_ATTR(cc_enable, S_IWUSR | S_IRUGO, show_cc_enable_test, store_cc_enable_test);
static struct attribute *moisture_attrs[] = {
	&dev_attr_sel_usb.attr,
	&dev_attr_sbu_voltage.attr,
	&dev_attr_cc_enable.attr,
	NULL
};

static const struct attribute_group moisture_attr_group = {
	.name	= "moisture",
	.attrs	= moisture_attrs,
};

int moisture_det_create(int cc_irq_num)
{
	int ret;
	struct mois_detc *mois_detctor;

	mois_dbg_msg("[Mois]%s start \n", __func__);

	mois_detctor = kzalloc(sizeof(*mois_detctor), GFP_KERNEL);
	if (!mois_detctor){
		mois_err_msg("[Mois]%s start error\n", __func__);
		return -ENOMEM;
	}

	mois_detctor->irq = gpio_to_irq(EDGE_DTE_IRQ);
	mois_detctor->edge_irq_num = -1;
	mois_detctor->cc_irq_num = cc_irq_num;
	mois_detctor->adc_state = ADC_STATE_DRY;
	mois_detctor->pre_adc_state = ADC_STATE_DRY;
	mois_detctor->judge_count = 0;
	mois_detctor->counter = 0;
	mois_detctor->cc_wet_state = 0;
	mois_detctor->pr_moisture_en = true;
	mois_detctor->pr_moisture = false;
	mois_detctor->cc_enable_flag = true;
	atomic_set(&mois_detctor->is_judging, false);
	g_mois_detctor = mois_detctor;
	moisture_gpio_init(mois_detctor);

	mutex_init(&mois_detctor->mutex_lock);

	mois_err_msg("[Mois]%s edge_irq=%d cc_irq=%d\n", __func__, mois_detctor->edge_irq_num, mois_detctor->cc_irq_num);

	INIT_DELAYED_WORK(&mois_detctor->check_moisture_state_work,
		check_moisture_state_work);
	INIT_DELAYED_WORK(&mois_detctor->judge_moisture_irq_work,
		judge_moisture_irq_work);
	INIT_DELAYED_WORK(&mois_detctor->wait_dry_work,
		wait_dry_work);
	INIT_DELAYED_WORK(&mois_detctor->wait_dry_work_for_cc,
		wait_dry_work_for_cc);

	debug_kobj = kobject_create_and_add("moisture_debug", NULL);
	if (!debug_kobj)
		return -ENOMEM;
	ret = sysfs_create_group(debug_kobj, &moisture_attr_group);
	if (ret) {
		kobject_put(debug_kobj);
		return ret;
	}

	return 0;
}

