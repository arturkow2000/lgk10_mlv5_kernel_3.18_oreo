#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>

#ifdef CONFIG_MTK_SMART_BATTERY
#include <mach/mt_battery_meter.h>
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/battery_meter.h>
#endif

#ifdef CONFIG_LGE_PM_USB_ID
#include <soc/mediatek/lge/lge_cable_id.h>
#endif

#ifdef CONFIG_MTK_SMART_BATTERY
extern PMU_ChargerStruct BMT_status;
extern void wake_up_bat(void);
#endif

static int g_AtCmdChargeMode = 0;		//for at%charge
static int g_AtCmdChargingModeOff = 0;

/* Internal APIs */
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

#define AUXADC_PCB_REV_CHANNEL	12
static int get_pcb_revision_voltage(void)
{
	int data[4] = {0, 0, 0, 0};
	int rawvalue = 0;
	int adc_voltage = 0;
	int ret;

	ret = IMM_GetOneChannelValue(AUXADC_PCB_REV_CHANNEL, data, &rawvalue);
	if (ret) {
		printk(KERN_INFO "[LGBM_AtCmd] Failed to read PCB_Revision\n");
		return -1;
	}

	adc_voltage = data[0] * 100 + data[1];
	printk(KERN_INFO "[LGBM_AtCmd] PCB_Revision ADC : %d\n", adc_voltage);

	return adc_voltage;
}

/* External APIs */
int get_AtCmdChargingModeOff(void)
{
	return g_AtCmdChargingModeOff;
}
EXPORT_SYMBOL(get_AtCmdChargingModeOff);

/* sysfs */
static ssize_t show_LGBM_AtCmdUsbidadc(struct device *dev, struct device_attribute *attr, char *buf)
{
	lge_get_cable_type_selfd();
	return sprintf(buf, "%d,%d", lge_get_cable_voltage(), lge_get_cable_value());
}
static DEVICE_ATTR(LGBM_AtCmdUsbidadc, 0444, show_LGBM_AtCmdUsbidadc, NULL);

static ssize_t show_LGBM_AtCmdCharge(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* return factory charge mode */
	return sprintf(buf, "%d\n", g_AtCmdChargeMode);
}
static ssize_t store_LGBM_AtCmdCharge(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	/* set factory charge mode */
	if(buf != NULL && size != 0) {
		if(buf[0] == '1')
			g_AtCmdChargeMode = 1;
		else
			g_AtCmdChargeMode = 0;
	}

#ifdef CONFIG_MTK_SMART_BATTERY
	wake_up_bat();
#endif

	return size;
}
static DEVICE_ATTR(LGBM_AtCmdCharge, 0664, show_LGBM_AtCmdCharge, store_LGBM_AtCmdCharge);

static ssize_t show_LGBM_AtCmdChcomp(struct device *dev, struct device_attribute *attr, char *buf)
{
	int isChargeComplete = 0;
	int voltage = battery_meter_get_battery_voltage(KAL_TRUE);

	if (voltage > RECHARGING_VOLTAGE) {
		isChargeComplete = 1;
	}

	return sprintf(buf, "%d\n", isChargeComplete);
}
static DEVICE_ATTR(LGBM_AtCmdChcomp, 0444, show_LGBM_AtCmdChcomp, NULL);

static ssize_t show_LGBM_AtCmdChargingModeOff(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_AtCmdChargingModeOff);
}
static ssize_t store_LGBM_AtCmdChargingModeOff(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	if (buf != NULL && size != 0) {
		if(buf[0] == '1')
			g_AtCmdChargingModeOff = 1;
	}

#ifdef CONFIG_MTK_SMART_BATTERY
	wake_up_bat();
#endif
	return size;
}
static DEVICE_ATTR(LGBM_AtCmdChargingModeOff, 0664, show_LGBM_AtCmdChargingModeOff, store_LGBM_AtCmdChargingModeOff);

static ssize_t show_LGBM_AtCmdBattExist(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", BMT_status.charger_exist);
}
static DEVICE_ATTR(LGBM_AtCmdBattExist, 0444, show_LGBM_AtCmdBattExist, NULL);

static ssize_t show_LGBM_AtCmdBatl(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* return battery voltage */

	int batVolt = 0;

	batVolt = battery_meter_get_battery_voltage(KAL_TRUE);

	return sprintf(buf, "%d\n", batVolt);
}
static DEVICE_ATTR(LGBM_AtCmdBatl, 0444, show_LGBM_AtCmdBatl, NULL);

static ssize_t show_LGBM_AtCmdBatmp(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* return battery temperature */

	int batTemp = 0;

	batTemp = battery_meter_get_battery_temperature();

	return sprintf(buf, "%d\n", batTemp);
}
static DEVICE_ATTR(LGBM_AtCmdBatmp, 0444, show_LGBM_AtCmdBatmp, NULL);

static ssize_t show_LGBM_AtCmdPcbrev(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* return pcb revision voltage */

	int pcb_rev_voltage;

	pcb_rev_voltage = get_pcb_revision_voltage();

	return sprintf(buf, "%d\n", pcb_rev_voltage);
}
static DEVICE_ATTR(LGBM_AtCmdPcbrev, 0444, show_LGBM_AtCmdPcbrev, NULL);

/* should be called in battery_probe() */
int LGBM_AtCmd_create_files(struct platform_device *dev)
{
	int ret_device_file = 0;

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdCharge);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdChcomp);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdUsbidadc);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdChargingModeOff);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBattExist);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBatl);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBatmp);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdPcbrev);

	return 0;
}
EXPORT_SYMBOL(LGBM_AtCmd_create_files);

int LGBM_AtCmd_init(struct platform_device *dev)
{
	return 0;
}
EXPORT_SYMBOL(LGBM_AtCmd_init);
