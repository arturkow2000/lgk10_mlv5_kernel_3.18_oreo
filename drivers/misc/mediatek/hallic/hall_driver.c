#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/slab.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>

#include <linux/pinctrl/consumer.h>

/* If any MTK platform does not support both edge trigger, use this feature.
 * c.f MT6755M support both edge, do not need it.
 * // falling or rising, not both falling + rising
 * #define HALLIC_IRQ_SUPPORT_ONLY_ONE_EDGE
 * */

/* SMART COVER Support */
#ifdef CONFIG_SMART_COVER_DETECTION
typedef enum {
	SMARTCOVER_POUCH_OPENED		= 0,		/* HALL_INT High */
	SMARTCOVER_POUCH_CLOSED		= 1		/* HALL_INT Low */
} pouch_state_t;
#endif

#ifdef CONFIG_STYLUS_PEN_DETECTION
typedef enum {
	STYLUS_PEN_OUT  = 0,				/* STYLUS_DETECT High */
	STYLUS_PEN_IN   = 1				/* STYLUS_DETECT Low */
} pen_state_t;
#endif

#define STR_POUCH_STATE(s)	((s) == SMARTCOVER_POUCH_OPENED ? "open" : \
								"close")
#define STR_PEN_STATE(p)	((p) == STYLUS_PEN_OUT ? "out" : "in")


#define HALL_IC_DEV_NAME "hallic"
#define HALL_IC_COMPATIBLE "lge,hallic"

struct hallic_platform_data {
#ifdef CONFIG_SMART_COVER_DETECTION
	int pouch_irq_gpio;
	int pouch_irq;
	int pouch_debounce;
#endif
#ifdef CONFIG_STYLUS_PEN_DETECTION
	int pen_irq_gpio;
	int pen_irq;
	int pen_debounce;
#endif
};

struct hallic_cradle {
	spinlock_t lock;
	struct hallic_platform_data *pdata;

	struct wake_lock wake_lock;
	int wake_lock_timeout_ms;

#ifdef CONFIG_SMART_COVER_DETECTION
	struct switch_dev sdev;
	pouch_state_t pouch_state;
#endif

#ifdef CONFIG_STYLUS_PEN_DETECTION
	struct switch_dev pen_sdev;
	pen_state_t pen_state;
#endif
};

#ifdef CONFIG_SMART_COVER_DETECTION
static void hallic_get_pouch_state(int gpio, pouch_state_t *pouch_state);
static struct delayed_work pouch_work;
#endif
#ifdef CONFIG_STYLUS_PEN_DETECTION
static struct delayed_work pen_work;
#endif

static struct workqueue_struct *cradle_wq;
static struct hallic_cradle *cradle;
static struct hallic_platform_data hallic_platform_data;

#ifdef CONFIG_SMART_COVER_DETECTION
#ifdef CONFIG_LGE_TOUCH_HALL_IC_COVER
static int is_smart_cover_closed = 0;
int cradle_smart_cover_status(void)
{
	return is_smart_cover_closed;
}
#endif

int hallic_is_pouch_closed(void)
{
	pouch_state_t pouch_state;

	if (!cradle || !cradle->pdata) {
		pr_err("%s::error, cradle is null", __func__);
		return -1;
	}

	hallic_get_pouch_state(cradle->pdata->pouch_irq_gpio, &pouch_state);
	if (pouch_state == SMARTCOVER_POUCH_CLOSED)
		return 1;
	else
		return 0;
}

static void hallic_get_pouch_state(int gpio, pouch_state_t *pouch_state)
{
	unsigned long flags;

	if (pouch_state) {
		spin_lock_irqsave(&cradle->lock, flags);
		if (gpio_get_value(gpio)) {
			/* if the gpio is high, it means that pouch is opened.*/
			*pouch_state = SMARTCOVER_POUCH_OPENED;
		} else {
			/* if the gpio is low, it means that pouch is closed. */
			*pouch_state = SMARTCOVER_POUCH_CLOSED;
		}
		spin_unlock_irqrestore(&cradle->lock, flags);

		pr_info("%s::pouch state = %d(%s)\n", __func__,
				*pouch_state, STR_POUCH_STATE(*pouch_state));
	}
}
#endif

#ifdef CONFIG_STYLUS_PEN_DETECTION
static void hallic_get_pen_state(int gpio, pen_state_t *pen_state)
{
	unsigned long flags;

	if (pen_state) {
		spin_lock_irqsave(&cradle->lock, flags);
		if (gpio_get_value(gpio)) {
			/* if the gpio is high, it means that pen is out. */
			*pen_state = STYLUS_PEN_OUT;
		} else {
			/* if the gpio is low, it means that pen is in. */
			*pen_state = STYLUS_PEN_IN;
		}
		spin_unlock_irqrestore(&cradle->lock, flags);

		pr_info("%s::pen state = %d(%s)\n", __func__,
				*pen_state, STR_PEN_STATE(*pen_state));
	}
}
#endif

static void boot_cradle_det_func(void)
{
	if (!cradle) {
		pr_err("%s::error, cradle is null", __func__);
		return;
	}

#ifdef CONFIG_SMART_COVER_DETECTION
	if (cradle->pdata->pouch_irq_gpio >= 0) {
		hallic_get_pouch_state(cradle->pdata->pouch_irq_gpio,
						&cradle->pouch_state);

		wake_lock_timeout(&cradle->wake_lock,
			msecs_to_jiffies(cradle->wake_lock_timeout_ms));
		switch_set_state(&cradle->sdev, cradle->pouch_state);
	}
#endif

#ifdef CONFIG_STYLUS_PEN_DETECTION
	if (cradle->pdata->pen_irq_gpio >= 0) {
		hallic_get_pen_state(cradle->pdata->pen_irq_gpio,
						&cradle->pen_state);

		wake_lock_timeout(&cradle->wake_lock,
			msecs_to_jiffies(cradle->wake_lock_timeout_ms));
		switch_set_state(&cradle->pen_sdev, cradle->pen_state);
	}
#endif

#ifdef CONFIG_LGE_TOUCH_HALL_IC_COVER
	is_smart_cover_closed = cradle->pouch_state;
#endif
}

#ifdef CONFIG_SMART_COVER_DETECTION
static void hallic_pouch_work_func(struct work_struct *work)
{
	pouch_state_t pouch_state;
#ifdef HALLIC_IRQ_SUPPORT_ONLY_ONE_EDGE
	unsigned long flags;
#endif

	if (!cradle) {
		pr_err("%s::error, cradle is null", __func__);
		return;
	}

	hallic_get_pouch_state(cradle->pdata->pouch_irq_gpio, &pouch_state);
	if (cradle->pouch_state != pouch_state) {
		cradle->pouch_state = pouch_state;

#ifdef HALLIC_IRQ_SUPPORT_ONLY_ONE_EDGE
		spin_lock_irqsave(&cradle->lock, flags);
		if (pouch_state == SMARTCOVER_POUCH_OPENED) {
			/* if smartcover is open, set gpio to detect close. */
			irq_set_irq_type(cradle->pdata->pouch_irq,
						IRQF_TRIGGER_FALLING);
		} else {
			/* if smartcover is closed, set gpio to detect open. */
			irq_set_irq_type(cradle->pdata->pouch_irq,
						IRQF_TRIGGER_RISING);
		}
		spin_unlock_irqrestore(&cradle->lock, flags);
#endif

		wake_lock_timeout(&cradle->wake_lock,
			msecs_to_jiffies(cradle->wake_lock_timeout_ms));
		switch_set_state(&cradle->sdev, pouch_state);

		pr_info("%s::pouch state is changed to %d(%s)\n", __func__ ,
				pouch_state, STR_POUCH_STATE(pouch_state));
	} else {
		pr_info("%s::pouch state is not changed from %d(%s)\n",
			__func__ , pouch_state, STR_POUCH_STATE(pouch_state));
	}
}

static irqreturn_t hallic_pouch_irq_handler(int irq, void *handle)
{
#ifdef CONFIG_LGE_TOUCH_HALL_IC_COVER
	is_smart_cover_closed = !gpio_get_value(cradle->pdata->pouch_irq_gpio);
#endif
	cancel_delayed_work(&pouch_work);
	queue_delayed_work(cradle_wq, &pouch_work, msecs_to_jiffies(0));

	pr_debug("%s::pouch irq done !!!!\n", __func__);

	return IRQ_HANDLED;
}

static void smart_cover_gpio_init(struct device *dev)
{
	struct pinctrl *pinctrl = NULL;
	struct pinctrl_state *hall_int_cfg = NULL;
	unsigned long irq_flag;
	int ret;

	if (!cradle) {
		pr_err("%s::error, cradle is null\n", __func__);
		return;
	}

	/* set up GPIO */
	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl)) {
		pr_err("%s::error, fail to get pinctrl\n", __func__);
		return;
	}

	hall_int_cfg = pinctrl_lookup_state(pinctrl, "hall_int_cfg");
	if (IS_ERR(hall_int_cfg)) {
		pr_err("%s::error, fail to get hall_int_cfg\n", __func__);
		return;
	}

	pinctrl_select_state(pinctrl, hall_int_cfg);

	/* set up IRQ */
#ifdef HALLIC_IRQ_SUPPORT_ONLY_ONE_EDGE
	hallic_get_pouch_state(cradle->pdata->pouch_irq_gpio,
					&cradle->pouch_state);

	if (cradle->pouch_state == SMARTCOVER_POUCH_OPENED) {
		/* if smartcover is open, set gpio to detect close. */
		irq_flag = IRQF_TRIGGER_FALLING;
	} else {
		/* if smartcover is closed, set gpio to detect open. */
		irq_flag = IRQF_TRIGGER_RISING;
	}
#else
	irq_flag = IRQF_TRIGGER_NONE;
#endif

	/* initialize irq HALL_INT */
	if (cradle->pdata->pouch_debounce > 0) {
		gpio_set_debounce(cradle->pdata->pouch_irq_gpio,
				cradle->pdata->pouch_debounce);
	}
	ret = request_irq(cradle->pdata->pouch_irq, hallic_pouch_irq_handler,
					irq_flag, "hallic_pouch-eint", NULL);
	if (ret)
		pr_err("%s:error, fail to register irq handler, ret=%d",
							__func__, ret);

	if (enable_irq_wake(cradle->pdata->pouch_irq))
		pr_err("%s::error, fail to enable_irq_wake\n", __func__);
}

static ssize_t cradle_pouch_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;

	if (cradle) {
		len = snprintf(buf, PAGE_SIZE, "cradle state : %d -> %s\n",
							cradle->pouch_state,
					STR_POUCH_STATE(cradle->pouch_state));
	}

	return len;
}

static struct device_attribute cradle_pouch_attr = __ATTR(pouch, S_IRUGO,
					cradle_pouch_state_show, NULL);
#endif /* CONFIG_SMART_COVER_DETECTION */

#ifdef CONFIG_STYLUS_PEN_DETECTION
static void hallic_pen_work_func(struct work_struct *work)
{
	pen_state_t pen_state;
#ifdef HALLIC_IRQ_SUPPORT_ONLY_ONE_EDGE
	unsigned long flags;
#endif

	if (!cradle) {
		pr_err("%s::error, cradle is null", __func__);
		return;
	}

	hallic_get_pen_state(cradle->pdata->pen_irq_gpio, &pen_state);
	if (cradle->pen_state != pen_state) {
		cradle->pen_state = pen_state;

#ifdef HALLIC_IRQ_SUPPORT_ONLY_ONE_EDGE
		spin_lock_irqsave(&cradle->lock, flags);
		if (pen_state == STYLUS_PEN_OUT) {
			/* if pen is out, set gpio to detect in */
			irq_set_irq_type(cradle->pdata->pen_irq,
					IRQF_TRIGGER_FALLING);
		} else {
			/* if pen is in, set gpio to detect out */
			irq_set_irq_type(cradle->pdata->pen_irq,
					IRQF_TRIGGER_RISING);
		}
		spin_unlock_irqrestore(&cradle->lock, flags);
#endif

		wake_lock_timeout(&cradle->wake_lock,
			msecs_to_jiffies(cradle->wake_lock_timeout_ms));
		switch_set_state(&cradle->pen_sdev, pen_state);

		pr_info("%s::pen state is changed to %d(%s)\n", __func__ ,
				pen_state, STR_PEN_STATE(pen_state));
	} else {
		pr_info("%s::pen state is not changed from %d(%s)\n", __func__ ,
				pen_state, STR_PEN_STATE(pen_state));
	}
}

static irqreturn_t hallic_pen_irq_handler(int irq, void *handle)
{
	cancel_delayed_work(&pen_work);
	queue_delayed_work(cradle_wq, &pen_work, msecs_to_jiffies(200));

	pr_debug("%s::pen irq done!!!!\n", __func__);

	return IRQ_HANDLED;
}

static void stylus_pen_gpio_init(struct device *dev)
{
	struct pinctrl *pinctrl = NULL;
	struct pinctrl_state *stylus_detect_cfg = NULL;
	unsigned long irq_flag;
	int ret;

	if (!cradle) {
		pr_err("%s::error, cradle is null", __func__);
		return;
	}

	/* set up GPIO */
	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl)) {
		pr_err("%s::error, fail to get pinctrl\n", __func__);
		return;
	}

	stylus_detect_cfg = pinctrl_lookup_state(pinctrl, "stylus_detect_cfg");
	if (IS_ERR(stylus_detect_cfg)) {
		pr_err("%s::error, fail to get stylus_detect_cfg\n", __func__);
		return;
	}

	pinctrl_select_state(pinctrl, stylus_detect_cfg);

	/* set up IRQ */
#ifdef HALLIC_IRQ_SUPPORT_ONLY_ONE_EDGE
	hallic_get_pen_state(cradle->pdata->pen_irq_gpio, &cradle->pen_state);

	if (cradle->pen_state == STYLUS_PEN_OUT) {
		/* if pen is out, set gpio to detect in */
		irq_flag = IRQF_TRIGGER_FALLING;
	} else {
		/* if pen is in, set gpio to detect out */
		irq_flag = IRQF_TRIGGER_RISING;
	}
#else
		irq_flag = IRQF_TRIGGER_NONE;
#endif

	/* initialize irq STYLUS_DETECT */
	if (cradle->pdata->pen_debounce > 0) {
		gpio_set_debounce(cradle->pdata->pen_irq_gpio,
				cradle->pdata->pen_debounce);
	}
	ret = request_irq(cradle->pdata->pen_irq, hallic_pen_irq_handler,
					irq_flag, "hallic_pen-eint", NULL);
	if (ret) {
		pr_err("%s::error, fail to register irq handler, ret=%d\n",
							 __func__, ret);
	}

	if (enable_irq_wake(cradle->pdata->pen_irq))
		pr_err("%s::error, fail to enable_irq_wake\n", __func__);
}

static ssize_t cradle_pen_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;

	if (cradle) {
		len = snprintf(buf, PAGE_SIZE, "pen state : %d -> %s\n",
			cradle->pen_state, STR_PEN_STATE(cradle->pen_state));
	}

	return len;
}

static struct device_attribute cradle_pen_attr = __ATTR(pen, S_IRUGO,
					cradle_pen_state_show, NULL);
#endif /* CONFIG_STYLUS_PEN_DETECTION */

#ifdef CONFIG_SMART_COVER_DETECTION
static ssize_t cradle_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case 0:
		return sprintf(buf, "smartcover\n");
	case 2:
		return sprintf(buf, "CARKET\n");
	}
	return -EINVAL;
}
#endif

static void hallic_parse_dt(struct device *dev,
		struct hallic_platform_data *pdata)
{
	int debounce[4] = { 0, 0, 0, 0 };
	int debounce_cnt = 0;

	struct device_node *node = dev->of_node;

#ifdef CONFIG_SMART_COVER_DETECTION
	pdata->pouch_irq_gpio = -1;
	pdata->pouch_debounce = 0;
	pdata->pouch_irq = -1;
#endif

#ifdef CONFIG_STYLUS_PEN_DETECTION
	pdata->pen_irq_gpio = -1;
	pdata->pen_debounce = 0;
	pdata->pen_irq = -1;
#endif

	if (node) {
		debounce_cnt = of_property_count_u32_elems(node, "debounce");
		if (debounce_cnt >= 2 && debounce_cnt <= ARRAY_SIZE(debounce)) {
			of_property_read_u32_array(node, "debounce", debounce,
								debounce_cnt);

#ifdef CONFIG_SMART_COVER_DETECTION
			pdata->pouch_irq_gpio = debounce[0];
			pdata->pouch_debounce = debounce[1];

			/* fitst */
			pdata->pouch_irq = irq_of_parse_and_map(node, 0);

			pr_info("%s::pouch_irq_gpio=%d, pouch_irq=%d\n",
								__func__,
				pdata->pouch_irq_gpio, pdata->pouch_irq);
#endif

#ifdef CONFIG_STYLUS_PEN_DETECTION
			if (debounce_cnt >= 4) {
				pdata->pen_irq_gpio = debounce[2];
				pdata->pen_debounce = debounce[3];

				/* second */
				pdata->pen_irq = irq_of_parse_and_map(node, 1);

				pr_info("%s::pen_irq_gpio=%d, pen_irq=%d\n",
				 __func__, pdata->pen_irq_gpio, pdata->pen_irq);
			}
#endif
		}
	} else {
		pr_err("%s::not found for %s\n", __func__,
						HALL_IC_COMPATIBLE);
	}
}

static int hallic_cradle_probe(struct platform_device *pdev)
{
	int ret = -1;

	struct hallic_platform_data *pdata = &hallic_platform_data;

	if (pdev->dev.of_node) {
		pdev->dev.platform_data = pdata;
		hallic_parse_dt(&pdev->dev, pdata);
	} else {
		return -ENODEV;
	}

	cradle = kzalloc(sizeof(*cradle), GFP_KERNEL);
	if (!cradle)
		return -ENOMEM;

	cradle->pdata = pdata;
	spin_lock_init(&cradle->lock);
	wake_lock_init(&cradle->wake_lock, WAKE_LOCK_SUSPEND,
					"hall_ic_wakeups");
	cradle->wake_lock_timeout_ms = 3000;

#ifdef CONFIG_SMART_COVER_DETECTION
	if (pdata->pouch_irq_gpio >= 0) {
		cradle->sdev.name = "smartcover";
		cradle->sdev.print_name = cradle_print_name;

		ret = switch_dev_register(&cradle->sdev);
		if (ret < 0)
			goto err_switch_dev_register;

		INIT_DELAYED_WORK(&pouch_work, hallic_pouch_work_func);

		smart_cover_gpio_init(&pdev->dev);
	}
#endif

#ifdef CONFIG_STYLUS_PEN_DETECTION
	if (pdata->pen_irq_gpio >= 0) {
		cradle->pen_sdev.name = "pen_state";
		/* not use - it is not necessary */
		/* cradle->pen_sdev.print_name = cradle_print_name; */

		ret = switch_dev_register(&cradle->pen_sdev);
		if (ret < 0)
			goto err_switch_dev_register;

		INIT_DELAYED_WORK(&pen_work, hallic_pen_work_func);

		stylus_pen_gpio_init(&pdev->dev);
	}
#endif

	pr_info("%s::init cradle\n", __func__);

	boot_cradle_det_func();

#ifdef CONFIG_SMART_COVER_DETECTION
	if (pdata->pouch_irq_gpio >= 0) {
		ret = device_create_file(&pdev->dev, &cradle_pouch_attr);
		if (ret)
			goto err_request_irq;
	}
#endif

#ifdef CONFIG_STYLUS_PEN_DETECTION
	if (pdata->pen_irq_gpio >= 0) {
		ret = device_create_file(&pdev->dev, &cradle_pen_attr);
		if (ret)
			goto err_request_irq;
	}
#endif

	pr_info("%s::hall_ic, probe done\n", __func__);

	return 0;

#if defined(CONFIG_SMART_COVER_DETECTION) || defined(CONFIG_STYLUS_PEN_DETECTION)
err_request_irq:
err_switch_dev_register:
#endif
#ifdef CONFIG_SMART_COVER_DETECTION
	switch_dev_unregister(&cradle->sdev);
#endif
#ifdef CONFIG_STYLUS_PEN_DETECTION
	switch_dev_unregister(&cradle->pen_sdev);
#endif

	kfree(cradle);
	cradle = NULL;

	return ret;
}

static int hallic_cradle_remove(struct platform_device *pdev)
{
#ifdef CONFIG_SMART_COVER_DETECTION
	cancel_delayed_work_sync(&pouch_work);
	switch_dev_unregister(&cradle->sdev);
#endif

#ifdef CONFIG_STYLUS_PEN_DETECTION
	cancel_delayed_work_sync(&pen_work);
	switch_dev_unregister(&cradle->pen_sdev);
#endif

	platform_set_drvdata(pdev, NULL);

	kfree(cradle);
	cradle = NULL;

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hallic_of_match[] = {
	{.compatible = HALL_IC_COMPATIBLE, },
	{},
};
#endif

static struct platform_driver hallic_cradle_driver = {
	.probe  = hallic_cradle_probe,
	.remove = hallic_cradle_remove,
	.driver	= {
		.name	= HALL_IC_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = hallic_of_match,
#endif
	},
};

static int __init hallic_cradle_init(void)
{
	cradle_wq = create_singlethread_workqueue("cradle_wq");

	if (!cradle_wq) {
		pr_err("fail to create workqueue\n");
		return -ENOMEM;
	}

	return platform_driver_register(&hallic_cradle_driver);
}

static void __exit hallic_cradle_exit(void)
{
	if (cradle_wq) {
		destroy_workqueue(cradle_wq);
		cradle_wq = NULL;
	}

	platform_driver_unregister(&hallic_cradle_driver);
}

module_init(hallic_cradle_init);
module_exit(hallic_cradle_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("Generic HALL IC Driver for MTK platform");
MODULE_LICENSE("GPL");

