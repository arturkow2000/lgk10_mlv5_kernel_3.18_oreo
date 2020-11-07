/*
 * earjack debugger trigger
 *
 * Copyright (C) 2012 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_MTK_PLATFORM
#include <linux/of.h>
#include <linux/of_irq.h>
#else
#include <soc/qcom/lge/board_lge.h>
#endif

struct earjack_debugger_device {
	int gpio;
#ifdef CONFIG_SND_SOC_ES9018
	int hph;
	int power;
#endif
	int irq;
	int (*set_uart_console)(int enable);
};

struct earjack_debugger_platform_data {
	int gpio_trigger;
#ifdef CONFIG_MTK_PLATFORM
	int irq;
	int debounce;
#endif
#ifdef CONFIG_SND_SOC_ES9018
	int hph_switch;
	int power_gpio;
#endif
};


#ifdef CONFIG_MTK_PLATFORM
int mtk_set_uart_console(int enable) {
	extern void mtk_uart_set_port_as_uart(void);
	extern void mtk_uart_set_port_as_gpio(void);

	extern int mt_need_uart_console;
	extern void mt_disable_uart(void);
	extern void mt_enable_uart(void);

	if (enable) {
		mtk_uart_set_port_as_uart();

		mt_need_uart_console = 1;
		mt_enable_uart();
	} else {
		mt_need_uart_console = 0;
		mt_disable_uart();

		mtk_uart_set_port_as_gpio();
	}

	return 0;
}
#endif

static int earjack_debugger_detected(void *dev)
{
	struct earjack_debugger_device *adev = dev;
	/* earjack debugger detecting by gpio 77 is changed
	 * from
	 *  G4: rev.A <= rev
	 *  Z2: rev.B <= rev
	 * as like
	 *  low  => uart enable
	 *  high => uart disable
	 */

	// Low if earjack debugger is in
	return !gpio_get_value(adev->gpio);
}

static irqreturn_t earjack_debugger_irq_handler(int irq, void *_dev)
{
	struct earjack_debugger_device *adev = _dev;
	int detect;

#ifdef CONFIG_MTK_PLATFORM
#else
	printk(KERN_INFO "Calling %s\n", __func__);
	msleep(400);
#endif

	detect = earjack_debugger_detected(adev);

	if (detect) {
#ifdef CONFIG_SND_SOC_ES9018
		gpio_set_value(adev->power, 1);
		gpio_set_value(adev->hph, 1);
#endif

#ifdef CONFIG_MTK_PLATFORM
		adev->set_uart_console(1);
#else
		adev->set_uart_console(
			lge_uart_console_should_enable_on_earjack_debugger());
#endif
		pr_info("%s() : in!!\n", __func__);
	} else {
		/* restore uart console status to default mode */
#ifdef CONFIG_SND_SOC_ES9018
		gpio_set_value(adev->power, 0);
		gpio_set_value(adev->hph, 0);
#endif

#ifdef CONFIG_MTK_PLATFORM
		adev->set_uart_console(0);
#else
		adev->set_uart_console(
				lge_uart_console_should_enable_on_default());
#endif
		pr_info("%s() : out!!\n", __func__);
	}

	return IRQ_HANDLED;
}



#ifdef CONFIG_MTK_PLATFORM
static void earjack_debugger_parse_dt(struct device *dev,
		struct earjack_debugger_platform_data *pdata)
{
	int debounce[] = { 0, 0 };
	struct device_node *np = dev->of_node;

	if (np) {
		of_property_read_u32_array(np, "debounce", debounce, 2);

		pdata->gpio_trigger = debounce[0];
		pdata->debounce = debounce[1];
		pdata->irq = irq_of_parse_and_map(np, 0);
	}

	pr_info("%s::irq_gpio=%d, irq=%d\n", __func__,
			pdata->gpio_trigger, pdata->irq);
}


static int earjack_debugger_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct earjack_debugger_device *adev;
	struct earjack_debugger_platform_data *pdata;
	struct pinctrl *pinctrl = NULL;
	struct pinctrl_state *gpio_cfg = NULL;
	int detect;

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct earjack_debugger_platform_data),
				GFP_KERNEL);
		if (pdata == NULL) {
			pr_err("%s: no pdata\n", __func__);
			return -ENOMEM;
		}
		pdev->dev.platform_data = pdata;
		earjack_debugger_parse_dt(&pdev->dev, pdata);
	} else {
		pdata = pdev->dev.platform_data;
	}

	if (!pdata) {
		pr_err("%s: no pdata\n", __func__);
		return -ENOMEM;
	}

	adev = kzalloc(sizeof(struct earjack_debugger_device), GFP_KERNEL);
	if (!adev) {
		pr_err("%s: no memory\n", __func__);
		return -ENOMEM;
	}

#ifdef CONFIG_SND_SOC_ES9018
	adev->hph = pdata->hph_switch;
	adev->power = pdata->power_gpio;
#endif

	adev->set_uart_console = mtk_set_uart_console;

	// set up GPIO
	adev->gpio = pdata->gpio_trigger;
	pinctrl = devm_pinctrl_get(&pdev->dev);
	gpio_cfg = pinctrl_lookup_state(pinctrl, "earjack_debugger_cfg");
	if (gpio_cfg) {
		pinctrl_select_state(pinctrl, gpio_cfg);
	} else {
		pr_err("%s: failed to get pinctrl state=earjack_debugger_cfg\n", __func__);
	}

	// set up IRQ
	adev->irq = pdata->irq;
	if (pdata->debounce > 0) {
		gpio_set_debounce(adev->gpio, pdata->debounce);
	}
	ret = request_irq(adev->irq, earjack_debugger_irq_handler, IRQF_TRIGGER_NONE,
					"earjack_debugger_trigger", adev);
	if (ret < 0) {
		pr_err("%s: failed to request irq, ret=%d\n", __func__, ret);
		goto err_request_irq;
	}

	platform_set_drvdata(pdev, adev);

	// set up uart
	detect = earjack_debugger_detected(adev);
	adev->set_uart_console(detect);

	pr_info("earjack debugger probed, detect=%d\n", detect);

	return ret;

err_request_irq:
	kfree(adev);

	return ret;
}
#else // QCT
static void earjack_debugger_parse_dt(struct device *dev,
		struct earjack_debugger_platform_data *pdata)
{
        struct device_node *np = dev->of_node;
        pdata->gpio_trigger = of_get_named_gpio_flags(np, "serial,irq-gpio",
			0, NULL);
#ifdef CONFIG_SND_SOC_ES9018
        pdata->hph_switch = of_get_named_gpio_flags(np, "serial,hph-sw", 0, NULL);
        pdata->power_gpio = of_get_named_gpio_flags(np, "serial,power-gpio", 0, NULL);
#endif
}

static int earjack_debugger_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct earjack_debugger_device *adev;
	struct earjack_debugger_platform_data *pdata;

	lge_uart_console_set_config(UART_CONSOLE_ENABLE_ON_EARJACK_DEBUGGER);

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct earjack_debugger_platform_data),
				GFP_KERNEL);
		if (pdata == NULL) {
			pr_err("%s: no pdata\n", __func__);
			return -ENOMEM;
		}
		pdev->dev.platform_data = pdata;
		earjack_debugger_parse_dt(&pdev->dev, pdata);
	} else {
		pdata = pdev->dev.platform_data;
	}

	if (!pdata) {
		pr_err("%s: no pdata\n", __func__);
		return -ENOMEM;
	}

	adev = kzalloc(sizeof(struct earjack_debugger_device), GFP_KERNEL);
	if (!adev) {
		pr_err("%s: no memory\n", __func__);
		return -ENOMEM;
	}

	adev->gpio = pdata->gpio_trigger;
#ifdef CONFIG_SND_SOC_ES9018
	adev->hph = pdata->hph_switch;
	adev->power = pdata->power_gpio;
#endif
	adev->irq = gpio_to_irq(pdata->gpio_trigger);
	adev->set_uart_console = msm_serial_set_uart_console;

	platform_set_drvdata(pdev, adev);

	ret = gpio_request_one(adev->gpio, GPIOF_IN,
			"gpio_earjack_debugger");
	if (ret < 0) {
		pr_err("%s: failed to request gpio %d\n", __func__,
				adev->gpio);
		goto err_gpio_request;
	}

	ret = request_threaded_irq(adev->irq, NULL,
			earjack_debugger_irq_handler,
			IRQF_TRIGGER_RISING |
				IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT,
			"earjack_debugger_trigger", adev);
	if (ret < 0) {
		pr_err("%s: failed to request irq\n", __func__);
		goto err_request_irq;
	}

	if (earjack_debugger_detected(adev)) {
		pr_debug("[UART CONSOLE][%s] %s uart console\n",
			__func__,
			lge_uart_console_should_enable_on_earjack_debugger() ?
				"enable" : "disable");
		adev->set_uart_console(
			lge_uart_console_should_enable_on_earjack_debugger());
	}

	pr_info("earjack debugger probed\n");

	return ret;

err_request_irq:
	gpio_free(adev->gpio);
err_gpio_request:
	kfree(adev);

	return ret;
}
#endif // CONFIG_MTK_PLATFORM

static int earjack_debugger_remove(struct platform_device *pdev)
{
	struct earjack_debugger_device *adev = platform_get_drvdata(pdev);

	free_irq(adev->irq, adev);
#ifdef CONFIG_MTK_PLATFORM
#else
	gpio_free(adev->gpio);
#endif
	kfree(adev);

	return 0;
}

static void earjack_debugger_shutdown(struct platform_device *pdev)
{
	struct earjack_debugger_device *adev = platform_get_drvdata(pdev);

	disable_irq(adev->irq);
}

static int earjack_debugger_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct earjack_debugger_device *adev = platform_get_drvdata(pdev);

	disable_irq(adev->irq);

	return 0;
}

static int earjack_debugger_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct earjack_debugger_device *adev = platform_get_drvdata(pdev);

	enable_irq(adev->irq);

	return 0;
}

static const struct dev_pm_ops earjack_debugger_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(earjack_debugger_suspend,
			earjack_debugger_resume)
};

#ifdef CONFIG_OF
static struct of_device_id earjack_debugger_match_table[] = {
	{ .compatible = "serial,earjack-debugger", },
	{ },
};
#endif

static struct platform_driver earjack_debugger_driver = {
	.probe = earjack_debugger_probe,
	.remove = earjack_debugger_remove,
	.shutdown = earjack_debugger_shutdown,
	.driver = {
		.name = "earjack-debugger",
		.pm = &earjack_debugger_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = earjack_debugger_match_table,
#endif
	},
};

static int __init earjack_debugger_init(void)
{
	return platform_driver_register(&earjack_debugger_driver);
}

static void __exit earjack_debugger_exit(void)
{
	platform_driver_unregister(&earjack_debugger_driver);
}

static int enable_uart(const char *val, struct kernel_param *kp)
{
#ifdef CONFIG_MTK_PLATFORM
	bool enable;
	if (!val) val = "1";

	strtobool(val, &enable);
	if (enable) {
		mtk_set_uart_console(1);
	} else {
		mtk_set_uart_console(0);
	}
#endif
	return 0;
}

int dummy_arg;
module_param_call(enable_uart, enable_uart, param_get_bool, &dummy_arg, S_IWUSR);

//module_init(earjack_debugger_init);
late_initcall(earjack_debugger_init);
module_exit(earjack_debugger_exit);
