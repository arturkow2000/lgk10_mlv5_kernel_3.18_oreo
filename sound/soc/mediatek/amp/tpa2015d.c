/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2009
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/

#include <linux/module.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <sound/soc.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <sound/tlv.h>
#include <mach/gpio_const.h>
#include <mt_gpio.h>

#include "AudDrv_Ext_Amp.h"
#include "AudDrv_Gpio_Amp.h"

#define TPA2015D_AMP_NAME   "ti,tpa2015d"



/*****************************************************************************
*                          DEBUG INFO
*****************************************************************************/
#define TPA2015D_LOG_ON

#ifdef TPA2015D_LOG_ON
#define EAMP_PRINTK(fmt, arg...) \
    do { \
        printk("[EAMP]: %s() "fmt"\n", __func__,##arg); \
    }while (0)
#else
#define EAMP_PRINTK(fmt, arg...)
#endif
/*****************************************************************************
*				   GLOBAL VARIABLES
*****************************************************************************/
static char power_state;

//static struct pinctrl *pinctrl_spk_sw;

extern struct audio_gpio_attr spk_sw_gpios[PINCTRL_SPK_NUM];

static struct ext_amp_data tpa2015d_data;

static int tpa2015d_init(void)
{
#ifndef CONFIG_SND_EXT_AMP_USE_GPIO_EXPANDER
    Speaker_Switch_GPIO_Default(tpa2015d_data);
#endif
    return 0;
}

static int tpa2015d_amp_on_get(struct snd_kcontrol *kcontrol,
                                        struct snd_ctl_elem_value *ucontrol)
{
    EAMP_PRINTK("\n");

    ucontrol->value.integer.value[0] = power_state;

    return 0;
}

static int tpa2015d_amp_on_set(struct snd_kcontrol *kcontrol,
                                        struct snd_ctl_elem_value *ucontrol)
{
    EAMP_PRINTK("state = %ld\n ", ucontrol->value.integer.value[0]);

   if (ucontrol->value.integer.value[0]) {
        Speaker_Switch_GPIO_On(tpa2015d_data);
        power_state = ucontrol->value.integer.value[0];
    } else {
        power_state = ucontrol->value.integer.value[0];
        Speaker_Switch_GPIO_Off(tpa2015d_data);
    }

    return 0;
}

static const char *const tpa2015d_amp_ctrl[] = { "Off", "On" };
static const struct soc_enum tpa2015d_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tpa2015d_amp_ctrl), tpa2015d_amp_ctrl)
};

static const struct snd_kcontrol_new tpa2015d_snd_controls[] = {
    SOC_ENUM_EXT("Ext_Speaker_Amp_Switch", tpa2015d_enum[0],
                tpa2015d_amp_on_get, tpa2015d_amp_on_set),
};

int tpa2015d_add_controls(struct snd_soc_codec *codec)
{
    EAMP_PRINTK("%s\n", __func__);

    return snd_soc_add_codec_controls(codec, tpa2015d_snd_controls,
                                    ARRAY_SIZE(tpa2015d_snd_controls));
}

static int tpa2015d_dev_probe(struct platform_device *pdev)
{
    EAMP_PRINTK("%s\n", __func__);
    register_ext_amp_ctrl(tpa2015d_add_controls);

    Speaker_Switch_GPIO_Init(&tpa2015d_data, &(pdev->dev));

    tpa2015d_init();

    return 0;
}

static int tpa2015d_dev_remove(struct platform_device *pdev)
{
    EAMP_PRINTK("%s\n", __func__);

    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tpa2015d_of_ids[] = {
	{.compatible = TPA2015D_AMP_NAME,},
	{}
};
#endif

static struct platform_driver tpa2015d_driver = {
	.driver = {
		   .name = TPA2015D_AMP_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = tpa2015d_of_ids,
#endif
		   },
	.probe = tpa2015d_dev_probe,
	.remove = tpa2015d_dev_remove,
};

#ifndef CONFIG_OF
static struct platform_device *tpa2015d_dev;
#endif


static int __init tpa2015d_dev_init(void)
{
	EAMP_PRINTK("%s:\n", __func__);
#ifndef CONFIG_OF
	int ret = 0;

	tpa2015d_dev = platform_device_alloc(TPA2015D_AMP_NAME, -1);

	if (!tpa2015d_dev)
		return -ENOMEM;


	ret = platform_device_add(tpa2015d_dev);
	if (ret != 0) {
		platform_device_put(tpa2015d_dev);
		return ret;
	}
#endif


	return platform_driver_register(&tpa2015d_driver);
}

static void __exit tpa2015d_dev_exit(void)
{
	EAMP_PRINTK("%s:\n", __func__);

	platform_driver_unregister(&tpa2015d_driver);
}

subsys_initcall(tpa2015d_dev_init);
module_exit(tpa2015d_dev_exit);

/* Module information */
MODULE_DESCRIPTION("TI, TPA2015D driver");
MODULE_LICENSE("GPL v2");

