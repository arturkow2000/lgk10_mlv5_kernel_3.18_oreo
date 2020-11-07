/***************************************************************************
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *    File  :
 *    Author(s)   :
 *    Description :
 *
 ***************************************************************************/

/****************************************************************************
* Include files
****************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/switch.h>
#include <linux/firmware.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/usb.h>
#include <linux/power_supply.h>
#ifdef CONFIG_LGE_BOOT_MODE
#include <soc/mediatek/lge/lge_boot_mode.h>
#endif

#include <linux/input/epack.h>
#include "internal.h"

/****************************************************************************
* Local function Prototypes
****************************************************************************/
static int epack_irq_init(struct epack *ep);

/****************************************************************************
* Manifest constants / Defines
****************************************************************************/
const char *dbg_str_pwr_status[] = {
	"NA",
	"OK",
	"NG",
	"UNKNOWN",
	"BLOCKED",
};

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
* Type definitions
****************************************************************************/
#ifndef BATTERY_BOOL
#define BATTERY_BOOL
typedef enum {
	KAL_FALSE = 0,
	KAL_TRUE  = 1,
} kal_bool;
extern kal_bool g_bat_init_flag;
extern bool usb_is_init;
bool epack_is_init = false;
#endif

/****************************************************************************
* Variables
****************************************************************************/
static struct epack *g_epack = NULL;
//static int epack_is_attactch = -1;
//static int epack_power_is_ok = -1;
//static int epack_vcharger_is_in = -1;
//static int epack_need_checking = -1;

/****************************************************************************
* Extern functions prototypes
****************************************************************************/
extern kal_bool bat_is_charger_exist(void);
void wake_up_bat(void);
extern void update_mtk_xhci_status(void);
extern void mtk_set_otg_state(int state);
extern void max98925_spk_enable(int enable);
extern char *xhci_get_event_status(void);

/****************************************************************************
* Local functions
****************************************************************************/
static bool epack_usb_is_umsdev(struct usb_device *udev)
{
	static struct usb_device *connected_udev = NULL;
	struct usb_interface *intf;

	if (udev->actconfig) {
		intf = udev->actconfig->interface[0];

		if (intf->cur_altsetting->desc.bInterfaceClass ==
				USB_CLASS_MASS_STORAGE) {
			connected_udev = udev;
			return true;
		} else {
			return false;
		}
	}

	/* device plugout */
	if (connected_udev == udev) {
		connected_udev = NULL;
		return true;
	}

	return false;
}

static void epack_set_pwr_status(int status)
{
	struct epack *ep = g_epack;

	if (!ep || !ep->dev)
		return;

	if (ep->pwr_status != status) {
		ep->pwr_status = status;
		dev_info(ep->dev, "EPACK_POWER is %s\n", dbg_str_pwr_status[status]);
		if (epack_is_init && ep->pwr_status == EPACK_POWER_UNKNOWN)
			dev_info(ep->dev, "WARN : epack power is unknown\n");
	}
}

static void epack_dbg_gpio(const char *str, struct epack *ep)
{
	int epid = gpio_get_value(ep->gpio_epid);
	int vinava = gpio_get_value(ep->gpio_vinava);
	int vchgdet = gpio_get_value(ep->gpio_vchgdet);

	dev_info(ep->dev, "%s : GPIO : epid=%d, ava=%d, chg=%d\n", str, epid, vinava, vchgdet);
}

static int epack_check_pwr_status(int epid, int vinava, int vchgdet)
{
	int power_status = EPACK_POWER_UNKNOWN;

	if (!epid) {
		power_status = EPACK_POWER_NA;
	} else {
		if (epack_get_pwr_path() == EPACK_PWR_PATH_MAIN) {
			if (vchgdet) {
				if (vinava) {
					power_status = EPACK_POWER_OK;
				} else {
					power_status = EPACK_POWER_NG;
				}
			} else {
				power_status = EPACK_POWER_UNKNOWN;
			}
		} else {
			if (bat_is_charger_exist()) {
				power_status = EPACK_POWER_OK;
			} else {
				power_status = EPACK_POWER_NG;
			}
		}
	}

	return power_status;
}

static int charger_eoc_detected(void)
{
	struct power_supply *batt_psy = NULL;
	struct power_supply *cc_psy = NULL;
	union power_supply_propval val;
	int rc;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy)
		return 0;

	rc = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (rc)
		return 0;
	if (val.intval < 100)
		return 0;

	cc_psy = power_supply_get_by_name("charger_controller");
	if (!cc_psy)
		return 0;

	rc = cc_psy->get_property(cc_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &val);
	if (rc)
		return 0;
	if (val.intval != POWER_SUPPLY_CHARGE_TYPE_EOC)
		return 0;

	return 1;

}

/****************************************************************************
* Global functions
****************************************************************************/
int epack_set_pwr_path(int path)
{
	static char *mode[] = {
		"pwr_main",
		"pwr_epack",
	};
	struct epack *ep = g_epack;
	struct pinctrl_state *state;
	int rc;

	if (!ep || !ep->pin)
		return -ENODEV;

	if (ep->pwr_path == path)
		return 0;

	state = pinctrl_lookup_state(ep->pin, mode[path]);
	if (IS_ERR(state))
		return -EINVAL;

	rc = pinctrl_select_state(ep->pin, state);
	if (rc)
		return rc;

	ep->pwr_path = path;
	dev_info(ep->dev, "POWER_PATH is %s\n", path == EPACK_PWR_PATH_EPACK ? "EPACK" : "MAIN");

	return rc;
}
EXPORT_SYMBOL(epack_set_pwr_path);

int epack_get_pwr_path(void)
{
	struct epack *ep = g_epack;

	if (!ep)
		return EPACK_PWR_PATH_MAIN;

	return ep->pwr_path;
}
EXPORT_SYMBOL(epack_get_pwr_path);

int epack_set_usb_path(int path)
{
	static char *mode[] = {
		"usb_main",
		"usb_epack",
	};
	struct epack *ep = g_epack;
	struct pinctrl_state *state;
	int rc;

	if (!ep || !ep->pin)
		return -ENODEV;

	if (ep->usb_path == path)
		return 0;

	state = pinctrl_lookup_state(ep->pin, mode[path]);
	if (IS_ERR(state))
		return -EINVAL;

	rc = pinctrl_select_state(ep->pin, state);
	if (rc)
		return rc;

	ep->usb_path= path;
	dev_info(ep->dev, "USB_PATH is %s\n", path == EPACK_USB_PATH_MAIN ? "MAIN" : "EPACK");

	return rc;
}
EXPORT_SYMBOL(epack_set_usb_path);

int epack_get_usb_path(void)
{
	struct epack *ep = g_epack;

	if (!ep)
		return EPACK_USB_PATH_MAIN;

	return ep->usb_path;
}
EXPORT_SYMBOL(epack_get_usb_path);

int epack_is_ok(void)
{
	struct epack *ep = g_epack;

	if (!ep || !epack_is_init)
		return 0;

	if (ep->is_present && ep->pwr_status == EPACK_POWER_OK)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(epack_is_ok);

int epack_is_otg_working(void)
{
	struct epack *ep = g_epack;

	if (!ep)
		return 0;

	return ep->otg_device;
}
EXPORT_SYMBOL(epack_is_otg_working);

bool epack_is_umsdev_connected(void)
{
	struct epack *ep = g_epack;

	if (!ep)
		return false;

	dev_dbg(ep->dev, "ums dev is %s\n", ep->umsdev_cnt ? "connected" : "NOT connected");
	return ep->umsdev_cnt ? true : false;
}
EXPORT_SYMBOL(epack_is_umsdev_connected);

int audio_get_epack_state(void)
{
	struct epack *ep = g_epack;

	if (!ep)
		return 0;

	return ep->aud_state.state? 1:0;
}
EXPORT_SYMBOL(audio_get_epack_state);

void epack_call_update_handler(int charger)
{
	struct epack *ep = g_epack;

	if (!ep || ep->fw_status == EPACK_FW_UPGRADE)
		return;

	epack_dbg_gpio("ISR = PMIC", ep);

	if (!charger && gpio_get_value(ep->gpio_epid) && ep->aud_state.state) {
		dev_info(ep->dev, "turn off the amp immediately to avoid pop-noise\n");
		max98925_spk_enable(false);
		dev_info(ep->dev, "set audio to disable\n");
		switch_set_state(&ep->aud_state, EPACK_AUD_DISABLE);
	}

	ep->trigger_src |= EPACK_TRIGGER_PMIC;
	schedule_delayed_work(&ep->update, msecs_to_jiffies(200));

}
EXPORT_SYMBOL(epack_call_update_handler);


/****************************************************************************
* Event triggered handler functions
****************************************************************************/
static void epack_update_handler(struct work_struct *work)
{
	struct epack *ep = container_of(to_delayed_work(work), struct epack, update);
	int power_status = NUM_EPACK_POWER;
	int rc;

#if 1 /* branden.you_20170616 */ /* consider mutext lock */
	int trigger = ep->trigger_src;
	int epid = ep->epid_status;
	int vinava = ep->vinava_status;
	int vchgdet = ep->vchgdet_status;

	ep->trigger_src &= ~trigger;
#endif

	dev_info(ep->dev, "epack_update_handler() called by ( 0x%02X )\n", trigger);

	if (ep->fw_status == EPACK_FW_UPGRADE) {
		dev_info(ep->dev, "WARN : do nothing during firmware upgrade\n");
		return; /* prevent any action */
	}

	if (trigger & EPACK_TRIGGER_INIT) {
		if (epid && ep->pwr_status == EPACK_POWER_NG) {
			dev_info(ep->dev, "vbus might be turned off by cmd, so try to turn on\n");
			rc = epack_control_vbus(ep, 1);
			if (rc) {
				dev_err(ep->dev, "failed to turn on vbus so it's real problem. rc=%d\n", rc);
			}
		}
	}

	if (trigger & (EPACK_TRIGGER_INIT | EPACK_TRIGGER_EPID | EPACK_TRIGGER_VCHGDET)) {
		if (vchgdet) {
			if (epack_get_pwr_path() != EPACK_PWR_PATH_MAIN) {
				epack_set_pwr_path(EPACK_PWR_PATH_MAIN);
				//msleep(200);
			}
		} else {
			if (epid) {
				if (epack_get_pwr_path() != EPACK_PWR_PATH_EPACK) {
					epack_set_pwr_path(EPACK_PWR_PATH_EPACK);
					msleep(350);
				}
			} else {
				if (epack_get_pwr_path() != EPACK_PWR_PATH_MAIN) {
					epack_set_pwr_path(EPACK_PWR_PATH_MAIN);
					//msleep(200);
				}
			}
		}
	}

	if (trigger & (EPACK_TRIGGER_INIT | EPACK_TRIGGER_EPID)) {
		if (epid) {
			ep->need_checking = true;
			cancel_delayed_work(&ep->monitor);
			schedule_delayed_work(&ep->monitor, msecs_to_jiffies(5*1000));
		} else {
			ep->need_checking = false;
			cancel_delayed_work(&ep->monitor);
			epack_set_usb_path(EPACK_USB_PATH_MAIN);
		}
	}

	power_status = epack_check_pwr_status(epid, vinava, vchgdet);
	if (power_status == EPACK_POWER_UNKNOWN) {
		dev_info(ep->dev, "WARN : Power Status is UNKNOWN so retry to check after some delay\n");
		schedule_delayed_work(&ep->update, msecs_to_jiffies(100));
		return;
	}

	if (ep->pwr_status != power_status || trigger & EPACK_TRIGGER_INIT) {

		epack_set_pwr_status(power_status);

		#ifdef CONFIG_LGE_BOOT_MODE
		if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
			return; /* prevent any action */
		}
		#endif

		/* firmware related job */
		if (epack_is_ok() && ep->need_checking) {
			ep->need_checking = false;
			if (epack_need_fw_upgrade(ep)) {
				rc = epack_do_fw_upgrade(ep, NULL);
				if (!rc)
					return; /* prevent any action */
				else
					dev_info(ep->dev, "failed to upgrade firmware\n");
			} else {
				dev_info(ep->dev, "don't need to upgrade firmware\n");
			}
		}

		/* audio related job */
		if (epack_is_ok()) {
			if (!ep->aud_state.state) {
				dev_info(ep->dev, "set audio to enable\n");
				switch_set_state(&ep->aud_state, EPACK_AUD_ENABLE);
			}
		} else {
			if (ep->aud_state.state) {
				if (epid) {
					dev_info(ep->dev, "turn off the amp immediately to avoid pop-noise\n");
					max98925_spk_enable(false);
				}
				dev_info(ep->dev, "set audio to disable\n");
				switch_set_state(&ep->aud_state, EPACK_AUD_DISABLE);
			}
		}

		/* usb related job */
		update_mtk_xhci_status();

	}

}

static irqreturn_t epack_epid_isr(int irq, void *data)
{
	struct epack *ep = (struct epack *)data;
	int delay; /* ms */

	disable_irq_nosync(irq);

	epack_dbg_gpio("ISR = EPID", ep);

	ep->epid_status = 1 - ep->epid_status;

	if (ep->epid_status) {
		/* epack attached */
		dev_info(ep->dev, "EPACK is ATTACHED\n");
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
		ep->is_present = true;
		delay = 600; /* give some delay until epack turns on the vbus */
	} else {
		/* epack dettached */
		dev_info(ep->dev, "EPACK is DETACHED\n");
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
		ep->is_present = false;
		delay = 5;
	}

	ep->trigger_src |= EPACK_TRIGGER_EPID;
	cancel_delayed_work(&ep->update);
	schedule_delayed_work(&ep->update, msecs_to_jiffies(delay));

	enable_irq(irq);

	return IRQ_HANDLED;

}


static irqreturn_t epack_vinava_isr(int irq, void *data)
{
	struct epack *ep = (struct epack *)data;

	disable_irq_nosync(irq);

	epack_dbg_gpio("ISR = VINAVA", ep);

	ep->vinava_status = 1 - ep->vinava_status;

	if (ep->vinava_status) {
		/* vinava is HIGH */
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
	} else {
		/* vinava is LOW */
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
	}

	if (ep->fw_status != EPACK_FW_UPGRADE) {
		ep->trigger_src |= EPACK_TRIGGER_VINAVA;
		cancel_delayed_work(&ep->update);
		schedule_delayed_work(&ep->update, msecs_to_jiffies(50));
	}

	enable_irq(irq);

	return IRQ_HANDLED;

}

static irqreturn_t epack_vchgdet_isr(int irq, void *data)
{
	struct epack *ep = (struct epack *)data;

	disable_irq_nosync(irq);

	epack_dbg_gpio("ISR = VCHGDET", ep);

	ep->vchgdet_status = 1 - ep->vchgdet_status;

	if (ep->vchgdet_status) {
		/* vcharger is HIGH */
		dev_info(ep->dev, "VCHGDET is HIGH\n");
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
	} else {
		/* vcharger is LOW */
		dev_info(ep->dev, "VCHGDET is LOW\n");
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
	}

	ep->trigger_src |= EPACK_TRIGGER_VCHGDET;
	cancel_delayed_work(&ep->update);
	schedule_delayed_work(&ep->update, msecs_to_jiffies(50));

	enable_irq(irq);

	return IRQ_HANDLED;

}

static int epack_usb_callback(struct notifier_block *self,
		unsigned long action, void *priv)
{
	struct epack *ep = container_of(self, struct epack, usb_nb);
	struct usb_device *udev = (struct usb_device *)priv;

	switch (action) {
		case USB_DEVICE_ADD:
			ep->otgdev_cnt++;
			if (epack_usb_is_umsdev(udev)) {
				ep->umsdev_cnt++;
				dev_info(ep->dev, "ums dev is connected\n");
				wake_up_bat();
			}
			break;
		case USB_DEVICE_REMOVE:
			ep->otgdev_cnt--;
			if (epack_usb_is_umsdev(udev)) {
				ep->umsdev_cnt--;
				dev_info(ep->dev, "ums dev is dis-connected\n");
				wake_up_bat();
			}
			break;
		default:
			break;
	}

	if (action == USB_DEVICE_ADD && ep->otgdev_cnt == 3) {
		if (epack_get_usb_path() == EPACK_USB_PATH_EPACK) {
			ep->otg_device = 1;
			mtk_set_otg_state(3); /* OTG_EPACK */
		}
	}

	if (action == USB_DEVICE_REMOVE && ep->otgdev_cnt == 2 && ep->otg_device == 1) {
		ep->otg_device = 0;
		mtk_set_otg_state(2); /* OTG_EPACK_ONLY */
		update_mtk_xhci_status();
	}

	dev_dbg(ep->dev, "otgdev_cnt=%d, umsdev_cnt=%d\n", ep->otgdev_cnt, ep->umsdev_cnt);

	return NOTIFY_OK;
}

static void epack_monitor(struct work_struct *work)
{
	struct epack *ep = container_of(to_delayed_work(work), struct epack, monitor);
	u16 voltage, temp, chg_stat, fault, output;
	int delay_in_sec;
	int rc;
	//static int count = 4;

	#ifdef CONFIG_LGE_BOOT_MODE
	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		//count--;
		//dev_info(ep->dev, "MONITOR : BOOT_MODE : CHARGER_LOGO ( count = %d )\n", count);
		//if (count == 0) {
		if (charger_eoc_detected()) {
			dev_info(ep->dev, "EOC was detected so turn off vbus to meet CEC testing\n");
			rc = epack_control_vbus(ep, 0);
			if (rc) {
				dev_err(ep->dev, "failed to turn on vbus so it's real problem. rc=%d\n", rc);
			} else {
				return;
			}
		}
		delay_in_sec = 10; /* sec */
		goto done;
	}
	#endif

	if (!ep->is_present) {
		dev_info(ep->dev, "MONITOR : epack is not present so quit monitor\n");
		return;
	}

	if (!epack_is_ok()) {
		dev_info(ep->dev, "MONITOR : epack is not OK\n");
		delay_in_sec = 10; /* sec */
		goto done;
	}

	if (ep->fw_status != EPACK_FW_OK) {
		dev_info(ep->dev, "MONITOR : firmware is not OK\n");
		delay_in_sec = 10; /* sec */
		goto done;
	}

	delay_in_sec = 60; /* sec */
	rc = epack_read_dbg_info(ep, &voltage, &temp, &chg_stat, &fault, &output);
	if (rc)
		dev_err(ep->dev, "MONITOR : failed to read monitoring values\n");
	else
		dev_info(ep->dev, "MONITOR : out=%d, volt=%dmV, temp=%d, chg=0x%02X, fault=0x%02X\n",
			output, voltage, temp, chg_stat, fault);

done:
	schedule_delayed_work(&ep->monitor, msecs_to_jiffies(delay_in_sec*1000));
}

static void epack_delayed_init(struct work_struct *work)
{
	struct epack *ep = container_of(to_delayed_work(work), struct epack, init);
	static int init_done = false;
	int power_status = NUM_EPACK_POWER;
	int rc;

	if (g_bat_init_flag == 0) {
		dev_info(ep->dev, "battery is not initialized so wait\n");
		schedule_delayed_work(&ep->init, msecs_to_jiffies(1000));
		return;
	}

	if (!init_done) {
		ep->epid_status = gpio_get_value(ep->gpio_epid);
		ep->vinava_status = gpio_get_value(ep->gpio_vinava);
		ep->vchgdet_status = gpio_get_value(ep->gpio_vchgdet);

		if (ep->is_present != ep->epid_status) {
			ep->is_present = ep->epid_status;
			dev_info(ep->dev, "EPACK is %s\n", ep->is_present ? "PRESENT":"ABSENT");
		}

		if (ep->is_present) {
			if (ep->vchgdet_status) {
				epack_set_pwr_path(EPACK_PWR_PATH_MAIN);
			} else {
				epack_set_pwr_path(EPACK_PWR_PATH_EPACK);
			}
		} else {
			epack_set_pwr_path(EPACK_PWR_PATH_MAIN);
		}

		power_status = epack_check_pwr_status(ep->epid_status, ep->vinava_status, ep->vchgdet_status);
		if (power_status == EPACK_POWER_UNKNOWN) {
			dev_info(ep->dev, "WARN : Power Status is UNKNOWN so retry to check after some delay\n");
			schedule_delayed_work(&ep->init, msecs_to_jiffies(200));
			return;
		} else {
			epack_set_pwr_status(power_status);
			init_done = true;
		}
	}

	if (!usb_is_init && !mtk_is_otg_main()) {
		dev_info(ep->dev, "usb is not initialized so wait\n");
		schedule_delayed_work(&ep->init, msecs_to_jiffies(2000));
		return;
	}

	epack_is_init = true;

	rc = epack_irq_init(ep);
	if (rc) {
		dev_err(ep->dev, "failed to init irq. rc=%d\n", rc);
	}

	ep->trigger_src |= EPACK_TRIGGER_INIT;
	schedule_delayed_work(&ep->update, msecs_to_jiffies(5));

}

/****************************************************************************
* Device driver functions
****************************************************************************/
static ssize_t show_cmd_test(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc =0;

	rc += sprintf(buf+rc, "Usage : \n");
	rc += sprintf(buf+rc, "cat fw_version : FW version check \n");
	rc += sprintf(buf+rc, "echo 2 > cmd_test : Sync Check\n");
	rc += sprintf(buf+rc, "echo 3 > cmd_test : Slave Mode check \n");
	rc += sprintf(buf+rc, "echo 4 > cmd_test : Slave Device ID read \n");
	rc += sprintf(buf+rc, "echo 5 > cmd_test : Slave config read \n");
	rc += sprintf(buf+rc, "echo 6 > cmd_test : set LDROM_MODE\n");
	rc += sprintf(buf+rc, "echo 7 > cmd_test : set APROM_MODE\n");
	rc += sprintf(buf+rc, "echo 8 > cmd_test : update aprom\n");
	rc += sprintf(buf+rc, "echo 9 > cmd_test : HW version check\n");
	rc += sprintf(buf+rc, "echo 1 go > fw_version : Quick FW update \n");

	return rc;
}

static ssize_t store_cmd_test(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct epack *ep = dev_get_drvdata(dev);
	int temp = simple_strtoul(buf, NULL, 10);

	dev_info(ep->dev, "CMD : %d\n", temp);

	return count;
}

static ssize_t store_upgrade(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct epack *ep = dev_get_drvdata(dev);
	int rc = 0;
	char path[256] = "";

	if (count > 256)
		return count;

	rc = sscanf(buf, "%s", path);
	dev_info(ep->dev, "FW PATH : %s\n", path);
	rc = epack_do_fw_upgrade(ep, path);
	if (rc)
		dev_info(ep->dev, "failed to upgrade firmware\n");

	return count;
}

static ssize_t store_restore(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct epack *ep = dev_get_drvdata(dev);
	int rc = 0;

	rc = epack_do_fw_upgrade(ep, NULL);
	if (rc)
		dev_info(ep->dev, "failed to restore firmware\n");

	return count;
}

static ssize_t show_otg_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", xhci_get_event_status());
}

static DEVICE_ATTR(cmd_test, S_IRUGO | S_IWUSR | S_IWGRP,
		show_cmd_test, store_cmd_test);
static DEVICE_ATTR(fw_upgrade, S_IWUSR | S_IWGRP, NULL, store_upgrade);
static DEVICE_ATTR(fw_restore, S_IWUSR | S_IWGRP, NULL, store_restore);
static DEVICE_ATTR(otg_mode, S_IRUGO, show_otg_mode, NULL);

static struct device_attribute *epack_attrs[] = {
	&dev_attr_cmd_test,
	&dev_attr_fw_upgrade,
	&dev_attr_fw_restore,
	&dev_attr_otg_mode,
};

static int epack_sysfs_init(struct epack *ep)
{
	int rc = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(epack_attrs); i++) {
		rc = device_create_file(ep->dev, epack_attrs[i]);
		if (rc)
			dev_err(ep->dev, "failed to create attr %d\n", i);
	}

	return rc;
}

static int epack_debugfs_id_show(struct seq_file *m, void *start)
{
	struct epack *ep = m->private;

	if (ep->is_present)
		seq_printf(m, "Present");
	else
		seq_printf(m, "None");
	seq_printf(m, "\n");

	return 0;
}

static int epack_debugfs_id_open(struct inode *inode, struct file *file)
{
	struct epack *ep = inode->i_private;

	return single_open(file, epack_debugfs_id_show, ep);
}

static struct file_operations epack_debugfs_id_ops = {
	.owner		= THIS_MODULE,
	.open		= epack_debugfs_id_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int epack_debugfs_vbus_show(struct seq_file *m, void *start)
{
	struct epack *ep = m->private;

	if (ep->pwr_status == EPACK_POWER_OK)
		seq_printf(m, "Present");
	else
		seq_printf(m, "None");
	seq_printf(m, "\n");

	return 0;
}

static int epack_debugfs_vbus_open(struct inode *inode, struct file *file)
{
	struct epack *ep = inode->i_private;

	return single_open(file, epack_debugfs_vbus_show, ep);
}

static struct file_operations epack_debugfs_vbus_ops = {
	.owner		= THIS_MODULE,
	.open		= epack_debugfs_vbus_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int epack_debugfs_firmware_show(struct seq_file *m, void *start)
{
	struct epack *ep = m->private;

	if (ep->fw_ver > 0)
		seq_printf(m, "%02X", ep->fw_ver);
	else
		seq_printf(m, "N/A");
	seq_printf(m, "\n");

	return 0;
}

static int epack_debugfs_firmware_open(struct inode *inode, struct file *file)
{
	struct epack *ep = inode->i_private;

	return single_open(file, epack_debugfs_firmware_show, ep);
}

static struct file_operations epack_debugfs_firmware_ops = {
	.owner		= THIS_MODULE,
	.open		= epack_debugfs_firmware_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int epack_debugfs_revision_show(struct seq_file *m, void *start)
{
	struct epack *ep = m->private;

	if (ep->hw_ver > 0)
		seq_printf(m, "%02X", ep->hw_ver);
	else
		seq_printf(m, "N/A");
	seq_printf(m, "\n");

	return 0;
}

static int epack_debugfs_revision_open(struct inode *inode, struct file *file)
{
	struct epack *ep = inode->i_private;

	return single_open(file, epack_debugfs_revision_show, ep);
}

static struct file_operations epack_debugfs_revision_ops = {
	.owner		= THIS_MODULE,
	.open		= epack_debugfs_revision_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int epack_debugfs_otg_show(struct seq_file *m, void *start)
{
	struct epack *ep = m->private;

	if (ep->usb_path == EPACK_USB_PATH_MAIN) {
		seq_printf(m, "Disabled\n");
		return 0;
	}

	if (ep->otg_device)
		seq_printf(m, "Connected");
	else
		seq_printf(m, "Enabled");
	seq_printf(m, "\n");

	return 0;
}

static int epack_debugfs_otg_open(struct inode *inode, struct file *file)
{
	struct epack *ep = inode->i_private;

	return single_open(file, epack_debugfs_otg_show, ep);
}

static struct file_operations epack_debugfs_otg_ops = {
	.owner		= THIS_MODULE,
	.open		= epack_debugfs_otg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int epack_debugfs_state_show(struct seq_file *m, void *start)
{
	struct epack *ep = m->private;

	if (!ep->is_present) {
		seq_printf(m, "EPACK : not connected\n");
		return 0;
	}
	seq_printf(m, "EPACK : connected\n");

	if (ep->pwr_status != EPACK_POWER_OK) {
		seq_printf(m, "Power : None\n");
		return 0;
	}
	seq_printf(m, "Power : ");
	if (ep->pwr_path == EPACK_PWR_PATH_MAIN)
		seq_printf(m, "USB ");
	else
		seq_printf(m, "EPACK");
	seq_printf(m, "\n");

	if (ep->usb_path == EPACK_USB_PATH_MAIN)
		seq_printf(m, "USB Path : usb\n");
	else
		seq_printf(m, "USB Path : epack\n");

	if (ep->otgdev_cnt > 2)
		seq_printf(m, "OTG Device : %d %s connected\n",
			ep->otgdev_cnt-2,
			ep->otgdev_cnt > 3 ? "devices" : "device");
	else
		seq_printf(m, "OTG Device : not connected\n");

	if (ep->aud_state.state)
		seq_printf(m, "Audio : online\n");
	else
		seq_printf(m, "Audio : offline\n");

	return 0;
}

static int epack_debugfs_state_open(struct inode *inode, struct file *file)
{
	struct epack *ep = inode->i_private;

	return single_open(file, epack_debugfs_state_show, ep);
}

static struct file_operations epack_debugfs_state_ops = {
	.owner		= THIS_MODULE,
	.open		= epack_debugfs_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct epack_debugfs_attr_t {
	char *name;
	struct file_operations *ops;
} epack_debugfs_attrs[] = {
	/* for Hidden Menu */
	{"id", &epack_debugfs_id_ops},
	{"vbus", &epack_debugfs_vbus_ops},
	{"firmware", &epack_debugfs_firmware_ops},
	{"revision", &epack_debugfs_revision_ops},
	{"otg", &epack_debugfs_otg_ops},
	/* for Debugging */
	{"state", &epack_debugfs_state_ops},
};

static int epack_debugfs_init(struct epack *ep)
{
	struct dentry *ent;
	int i;

	ep->debugfs = debugfs_create_dir("epack", NULL);
	if (!ep->debugfs) {
		dev_err(ep->dev, "failed to create debugfs\n");
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(epack_debugfs_attrs); i++) {
		ent = debugfs_create_file(epack_debugfs_attrs[i].name,
				S_IFREG | S_IRUGO,
				ep->debugfs, ep, epack_debugfs_attrs[i].ops);
		if (!ent)
			dev_err(ep->dev, "failed to create %s debugfs\n",
					epack_debugfs_attrs[i].name);
	}

	return 0;
}

static int epack_gpio_init(struct epack *ep)
{
	struct pinctrl_state *state;
	int rc;

	rc = devm_gpio_request(ep->dev, ep->gpio_epid, "ep_det");
	if (rc) {
		dev_err(ep->dev, "failed to request ep_det gpio, rc=%d\n", rc);
		return rc;
	}

	rc = devm_gpio_request(ep->dev, ep->gpio_vinava, "ep_pwr_vinava");
	if (rc) {
		dev_err(ep->dev, "failed to request vinava gpio, rc=%d\n", rc);
		return rc;
	}

	rc = devm_gpio_request(ep->dev, ep->gpio_vchgdet, "ep_pwr_vchgdet");
	if (rc) {
		dev_err(ep->dev, "failed to request vchgdet gpio, rc=%d\n", rc);
		return rc;
	}

	state = pinctrl_lookup_state(ep->pin, "init");
	if (!IS_ERR(state))
		pinctrl_select_state(ep->pin, state);
	else
		dev_err(ep->dev, "failed to get pin ctrl - init\n");

	return rc;

}

static int epack_irq_init(struct epack *ep)
{
	int rc;

	dev_info(ep->dev, "done epack_irq_init()\n");

	rc = gpio_set_debounce(ep->gpio_epid, 80*1000); /* if set to 50ms, un-wanted interrupt shall be happened */
	if (rc)
		dev_err(ep->dev, "failed to set epid debounce, rc=%d\n", rc);

	rc = devm_request_irq(ep->dev, ep->irq_epid, epack_epid_isr,
			ep->epid_status ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH, "epid_irq", ep);
	if (rc) {
		dev_err(ep->dev, "failed to request ep_det irq, rc=%d\n", rc);
		return rc;
	}

	rc = gpio_set_debounce(ep->gpio_vinava, 70*1000); /* need to set it 20ms bigger than vchgdet */
	if (rc)
		dev_err(ep->dev, "failed to set vinava debounce, rc=%d\n", rc);

	rc = devm_request_irq(ep->dev, ep->irq_vinava, epack_vinava_isr,
			ep->vinava_status ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH, "vinava_irq", ep);
	if (rc) {
		dev_err(ep->dev, "failed to request vinava irq, rc=%d\n", rc);
		return rc;
	}

	rc = gpio_set_debounce(ep->gpio_vchgdet, 50*1000);
	if (rc)
		dev_err(ep->dev, "failed to set vchgdet debounce, rc=%d\n", rc);

	rc = devm_request_irq(ep->dev, ep->irq_vchgdet, epack_vchgdet_isr,
			ep->vchgdet_status ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH, "vchgdet_irq", ep);
	if (rc) {
		dev_err(ep->dev, "failed to request vchgdet irq, rc=%d\n", rc);
		return rc;
	}

	return rc;
}


static int epack_parse_dt(struct epack *ep)
{
	struct device_node *np = ep->dev->of_node;

	ep->gpio_epid = of_get_named_gpio_flags(np, "gpio", 0, NULL);
	if (!gpio_is_valid(ep->gpio_epid)) {
		dev_err(ep->dev, "failed to read det-gpio\n");
		return -EINVAL;
	}

	ep->gpio_vinava = of_get_named_gpio_flags(np, "gpio", 1, NULL);
	if (!gpio_is_valid(ep->gpio_vinava)) {
		dev_err(ep->dev, "failed to read vinava-gpio\n");
		return -EINVAL;
	}

	ep->gpio_vchgdet = of_get_named_gpio_flags(np, "gpio", 2, NULL);
	if (!gpio_is_valid(ep->gpio_vchgdet)) {
		dev_err(ep->dev, "failed to read vchgdet-gpio\n");
		return -EINVAL;
	}

	ep->pin = devm_pinctrl_get(ep->dev);
	if (IS_ERR(ep->pin)) {
		dev_err(ep->dev, "failed to get pinctrl\n");
		return PTR_ERR(ep->pin);
	}

	ep->irq_epid = irq_of_parse_and_map(np, 0);
	if (!ep->irq_epid)
		return -ENODEV;

	ep->irq_vinava = irq_of_parse_and_map(np, 1);
	if (!ep->irq_vinava)
		return -ENODEV;

	ep->irq_vchgdet = irq_of_parse_and_map(np, 2);
	if (!ep->irq_vchgdet)
		return -ENODEV;

	return 0;
}

static int epack_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct epack *ep;
	int rc;

	ep = devm_kzalloc(&client->dev, sizeof(*ep), GFP_KERNEL);
	if (!ep) {
		dev_err(&client->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	g_epack = ep;

	ep->client = client;
	ep->dev = &client->dev;
	i2c_set_clientdata(client, ep);

	dev_info(ep->dev, "enter epack_probe()\n");

	INIT_DELAYED_WORK(&ep->update, epack_update_handler);
	INIT_DELAYED_WORK(&ep->monitor, epack_monitor);
	INIT_DELAYED_WORK(&ep->init, epack_delayed_init);

	mutex_init(&ep->i2c_rw_lock);

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (EPACK_PACKET_SIZE > 8) {
		ep->i2c_buf = (u8 *)dma_alloc_coherent(NULL, EPACK_PACKET_SIZE,
					&ep->i2c_dma, GFP_KERNEL | GFP_DMA32);
		if (!ep->i2c_buf)
			dev_err(&client->dev, "failed to get dma buffer\n");
	}
#endif

	rc = epack_parse_dt(ep);
	if (rc) {
		dev_err(&client->dev, "failed to parse epack dt. rc=%d\n", rc);
		return rc;
	}

	rc = epack_sysfs_init(ep);
	if (rc) {
		dev_err(&client->dev, "failed to init sysfs. rc=%d\n", rc);
	}

	rc = epack_debugfs_init(ep);
	if (rc) {
		dev_err(&client->dev, "failed to init debugfs. rc=%d\n", rc);
	}

	rc = epack_fw_init(ep);
	if (rc) {
		dev_err(&client->dev, "failed to init epack fw. rc=%d\n", rc);
		return rc;
	}

	ep->aud_state.name = "epack";
	rc = switch_dev_register(&ep->aud_state);
	if (rc) {
		dev_err(&client->dev, "failed to init epack audio switch. rc=%d\n", rc);
		return rc;
	}

	rc = epack_gpio_init(ep);
	if (rc) {
		dev_err(&client->dev, "failed to init gpio. rc=%d\n", rc);
		return rc;
	}

	ep->usb_path = -1;
	ep->pwr_path = -1;

	ep->epid_status = gpio_get_value(ep->gpio_epid);
	ep->is_present = ep->epid_status;
	dev_info(ep->dev, "EPACK is %s\n", ep->is_present ? "PRESENT":"ABSENT");

	epack_set_usb_path(EPACK_USB_PATH_MAIN);

	ep->vchgdet_status = gpio_get_value(ep->gpio_vchgdet);

	if (ep->is_present) {
		if (ep->vchgdet_status) {
			epack_set_pwr_path(EPACK_PWR_PATH_MAIN);
		} else {
			epack_set_pwr_path(EPACK_PWR_PATH_EPACK);
		}
	} else {
		epack_set_pwr_path(EPACK_PWR_PATH_MAIN);
	}

	#ifdef CONFIG_LGE_BOOT_MODE
	if (lge_get_boot_mode() != LGE_BOOT_MODE_CHARGERLOGO) {
		ep->usb_nb.notifier_call = epack_usb_callback;
		usb_register_notify(&ep->usb_nb);
	}
	#else
	ep->usb_nb.notifier_call = epack_usb_callback;
	usb_register_notify(&ep->usb_nb);
	#endif

	schedule_delayed_work(&ep->init, 0);

	return rc;
}

static int epack_remove(struct i2c_client *client)
{
	struct epack *ep = i2c_get_clientdata(client);

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (ep->i2c_buf)
		dma_free_coherent(NULL, EPACK_PACKET_SIZE, ep->i2c_buf, ep->i2c_dma);
#endif

	return 0;
}

static const struct i2c_device_id epack_id_table[] = {
	{"epack", 0},
};

#ifdef CONFIG_OF
static struct of_device_id epack_match_table[] = {
	{ .compatible = "em-tech,epack",},
};
#endif

static struct i2c_driver epack_driver = {
	.driver = {
		.name = "epack",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = epack_match_table,
#endif
	},
	.probe = epack_probe,
	.remove = epack_remove,
	.id_table = epack_id_table,
};

static __init int epack_init(void)
{
	return i2c_add_driver(&epack_driver);
}
late_initcall(epack_init);

static __exit void epack_exit(void)
{
	i2c_del_driver(&epack_driver);
}
module_exit(epack_exit);

MODULE_DESCRIPTION("E-Pack Driver");
MODULE_LICENSE("GPL");

/* End Of File */
