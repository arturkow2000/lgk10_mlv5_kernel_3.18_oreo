#ifndef _AUDDRV_EXT_AMP_GPIO_H_
#define _AUDDRV_EXT_AMP_GPIO_H_

#include <linux/module.h>
#include <linux/device.h>
#include <sound/soc.h>

enum spk_sw_gpio_type {
        PINCTRL_SPK_SW_DEFAULT = 0,
        PINCTRL_SPK_SW_OFF,
        PINCTRL_SPK_SW_ON,
        PINCTRL_SPK_NUM
};

struct audio_gpio_attr {
        const char *name;
        struct pinctrl_state *gpioctrl;
};

struct ext_amp_data {
    int use_pinctrl;
    int amp_gpio;
    struct pinctrl *pinctrl_spk_sw;
};


void Speaker_Switch_GPIO_On(struct ext_amp_data amp_data);

void Speaker_Switch_GPIO_Off(struct ext_amp_data amp_data);

void Speaker_Switch_GPIO_Default(struct ext_amp_data amp_data);

void Speaker_Switch_GPIO_Init(struct ext_amp_data* amp_data, struct device* dev);

#if defined(CONFIG_SND_EXT_AMP_USE_GPIO_EXPANDER)
void Speaker_Switch_GPIO_Expander(int gpio, int enable);
#endif

#endif


