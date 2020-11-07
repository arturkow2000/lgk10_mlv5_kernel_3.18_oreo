#include "AudDrv_Gpio_Amp.h"
#include <sound/soc.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#if defined(CONFIG_MACH_MT6750S_CV5A)
#include <soc/mediatek/lge/board_lge.h>
#endif

#define AMP_EN_PRINTK(fmt, arg...) \
    do { \
        printk("[AMP]: %s() "fmt"\n", __func__,##arg); \
    }while (0)

struct audio_gpio_attr spk_sw_gpios[PINCTRL_SPK_NUM] = {
        [PINCTRL_SPK_SW_DEFAULT] = {"aud_spk_sw_default", NULL},
        [PINCTRL_SPK_SW_OFF] = {"aud_spk_sw_off", NULL},
        [PINCTRL_SPK_SW_ON] = {"aud_spk_sw_on", NULL},
};

#if defined(CONFIG_SND_EXT_AMP_USE_GPIO_EXPANDER)
#ifdef CONFIG_TI_TCA6418
extern int ioexp_gpio_set_value(uint8_t gpio, uint8_t value);
#endif
#ifdef CONFIG_GPIO_PCAL6416A
extern int nxp_gpio_set_value(uint8_t gpio, uint8_t value);
#endif

void Speaker_Switch_GPIO_Expander(int gpio, int enable)
{
#ifdef CONFIG_MACH_MT6750S_CV7A
        nxp_gpio_set_value(gpio, enable);

#elif defined(CONFIG_MACH_MT6750S_CV5A)
        if (lge_get_board_revno() == 0) {
                #if defined(CONFIG_TI_TCA6418)
                ioexp_gpio_set_value(gpio, enable);
                #endif
        } else {
                #if defined(CONFIG_GPIO_PCAL6416A)
                nxp_gpio_set_value(gpio, enable);
                #endif
        }
#else

#if defined(CONFIG_TI_TCA6418)
        ioexp_gpio_set_value(gpio, enable);
#elif defined(CONFIG_GPIO_PCAL6416A)
        nxp_gpio_set_value(gpio, enable);
#endif

#endif //CONFIG_MACH_MT6750S_CV5A
}
#endif

void Speaker_Switch_GPIO_On(struct ext_amp_data amp_data){
        AMP_EN_PRINTK("SPK SWT ON!!");
        if(amp_data.use_pinctrl) {
		  pinctrl_select_state(amp_data.pinctrl_spk_sw, spk_sw_gpios[PINCTRL_SPK_SW_ON].gpioctrl);
	  } else {
                if(amp_data.amp_gpio > -1) {
#if defined(CONFIG_SND_EXT_AMP_USE_GPIO_EXPANDER)
                        Speaker_Switch_GPIO_Expander(amp_data.amp_gpio, 1);
#else
                        gpio_set_value(amp_data.amp_gpio, 1);
#endif
                } else {
                        AMP_EN_PRINTK("Error : SPK Amp GPIO is %d\n", amp_data.amp_gpio);
                }
        }
}
EXPORT_SYMBOL(Speaker_Switch_GPIO_On);

void Speaker_Switch_GPIO_Off(struct ext_amp_data amp_data){
        AMP_EN_PRINTK("SPK SWT OFF!!");
	  if(amp_data.use_pinctrl) {
		  pinctrl_select_state(amp_data.pinctrl_spk_sw, spk_sw_gpios[PINCTRL_SPK_SW_OFF].gpioctrl);
	  } else {
                if(amp_data.amp_gpio > -1) {
#if defined(CONFIG_SND_EXT_AMP_USE_GPIO_EXPANDER)
                        Speaker_Switch_GPIO_Expander(amp_data.amp_gpio, 0);
#else
                        gpio_set_value(amp_data.amp_gpio, 0);
#endif
                } else {
                        AMP_EN_PRINTK("Error : SPK Amp GPIO is %d\n", amp_data.amp_gpio);
                }
        }
}
EXPORT_SYMBOL(Speaker_Switch_GPIO_Off);

void Speaker_Switch_GPIO_Default(struct ext_amp_data amp_data){
        AMP_EN_PRINTK("SPK SWT Default!!");
	  if(amp_data.use_pinctrl) {
		  pinctrl_select_state(amp_data.pinctrl_spk_sw, spk_sw_gpios[PINCTRL_SPK_SW_DEFAULT].gpioctrl);
	  } else {
                if(amp_data.amp_gpio > -1) {
#if defined(CONFIG_SND_EXT_AMP_USE_GPIO_EXPANDER)
                        Speaker_Switch_GPIO_Expander(amp_data.amp_gpio, 0);
#else
                        gpio_set_value(amp_data.amp_gpio, 0);
#endif
                } else {
                        AMP_EN_PRINTK("Error : SPK Amp GPIO is %d\n", amp_data.amp_gpio);
                }
        }
}
EXPORT_SYMBOL(Speaker_Switch_GPIO_Default);

void Speaker_Switch_GPIO_Init(struct ext_amp_data* amp_data, struct device* dev)
{
        int i;
        amp_data->pinctrl_spk_sw= devm_pinctrl_get(dev);

        AMP_EN_PRINTK("SPK SWT Init!!");

        if(of_property_read_u32_index(dev->of_node, "ti,gpio-control", 0, &amp_data->use_pinctrl)) {
                AMP_EN_PRINTK("ti,gpio-control is not exist.");
                amp_data->use_pinctrl = 0;
        }

        if(amp_data->use_pinctrl) {
                AMP_EN_PRINTK("SPK SWT Init!! - Use Pin Control");
                for(i = 0; i < ARRAY_SIZE(spk_sw_gpios); i++){
                        spk_sw_gpios[i].gpioctrl = pinctrl_lookup_state(amp_data->pinctrl_spk_sw, spk_sw_gpios[i].name);
                        if(IS_ERR(spk_sw_gpios[i].gpioctrl)){
                                spk_sw_gpios[i].gpioctrl = NULL;
                                pr_err("%s: pinctrl_lookup_state %s fail %ld\n",
                                                __func__, spk_sw_gpios[i].name,
                                                PTR_ERR(spk_sw_gpios[i].gpioctrl));
                        }
                }
        } else {
                AMP_EN_PRINTK("SPK SWT Init!! - Get GPIO");
#if defined(CONFIG_SND_EXT_AMP_USE_GPIO_EXPANDER)
                if(of_property_read_u32_index(dev->of_node, "ti,amp-gpio", 0, &amp_data->amp_gpio)) {
                        AMP_EN_PRINTK("Can not get amp-gpio.");
                        amp_data->amp_gpio = -1;
                } else {
                        AMP_EN_PRINTK("Amp GPIO is %d", amp_data->amp_gpio);
                }
#else
                amp_data->amp_gpio = of_get_named_gpio(dev->of_node, "ti,amp-gpio", 0);
                if (amp_data->amp_gpio < 0) {
                        AMP_EN_PRINTK("Looking up %s property in node %s failed %d\n", "ti,amp-gpio", dev->of_node->full_name, amp_data->amp_gpio);
                }
#endif
        }
};

EXPORT_SYMBOL(Speaker_Switch_GPIO_Init);



