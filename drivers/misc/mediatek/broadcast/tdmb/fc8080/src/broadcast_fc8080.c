#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
//#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>         /* wake_lock, unlock */

#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/broadcast_fc8080.h"
#include "../inc/fci_types.h"
#include "../inc/bbm.h"

//#include <linux/pm_qos.h> // FEATURE_DMB_USE_PM_QOS

#include <linux/err.h>
#include <linux/of_gpio.h>

#include <linux/clk.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqchip/mt-eic.h>
#include "mt_gpio.h"
#include "mach/gpio_const.h"
#include "mt_spi.h"
#include "mt_clkbuf_ctl.h"

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

static struct mt_chip_conf mtk_spi_config = {
    .setuptime = 3,
    .holdtime = 3,
    .high_time = 3,
    .low_time = 3,
    .cs_idletime = 2,
    .ulthgh_thrsh = 0,
    .cpol = 0,
    .cpha = 0,
    .rx_mlsb = 1,
    .tx_mlsb = 1,
    .tx_endian = 0,
    .rx_endian = 0,
    .com_mod = DMA_TRANSFER,
    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 2,
};

//#include <linux/msm-bus.h> // FEATURE_DMB_USE_BUS_SCALE

/* external function */
extern int broadcast_fc8080_drv_if_isr(void);
extern void tunerbb_drv_fc8080_isr_control(fci_u8 onoff);

/* proto type declare */
static int broadcast_tdmb_fc8080_probe(struct spi_device *spi);
//static int broadcast_tdmb_fc8080_remove(struct spi_device *spi);
static int broadcast_tdmb_fc8080_suspend(struct spi_device *spi, pm_message_t mesg);
static int broadcast_tdmb_fc8080_resume(struct spi_device *spi);

/* SPI Data read using workqueue */
//#define FEATURE_DMB_USE_WORKQUEUE
//#define FEATURE_DMB_USE_BUS_SCALE
//#define FEATURE_DMB_USE_PM_QOS

#define LGE_FC8080_DRV_VER  "1.00.02"

/************************************************************************/
/* LINUX Driver Setting                                                 */
/************************************************************************/
static uint32 user_stop_flg = 0;
struct tdmb_fc8080_ctrl_blk
{
    boolean                                    TdmbPowerOnState;
    struct spi_device*                        spi_ptr;
#ifdef FEATURE_DMB_USE_WORKQUEUE
    struct work_struct                        spi_work;
    struct workqueue_struct*                spi_wq;
#endif
    struct mutex                            mutex;
    struct wake_lock                        wake_lock;    /* wake_lock,wake_unlock */
    boolean                                    spi_irq_status;
    spinlock_t                                spin_lock;
    struct clk                                *clk;
    struct platform_device                    *pdev;
#ifdef FEATURE_DMB_USE_BUS_SCALE
    struct msm_bus_scale_pdata                *bus_scale_pdata;
    fci_u32                                        bus_scale_client_id;
#endif
#ifdef FEATURE_DMB_USE_PM_QOS
    struct pm_qos_request    pm_req_list;
#endif
    struct device_node               *dmb_irq_node;
    struct device_node               *dmb_lna_node;
    struct device_node               *dmb_ant_switch_node;

    uint32                            dmb_irq;
    uint32                            dmb_ant;

    uint32                            dmb_xtal_freq;
    uint32                            dmb_use_xtal;

    uint32                            dmb_spi_bus_num;
#ifdef CONFIG_OF
        struct pinctrl *pinctrl_gpios;
        struct pinctrl_state *pins_default;
        struct pinctrl_state *pins_miso_spi, *pins_miso_pullhigh, *pins_miso_pulllow, *pins_miso_pulldisable;
        struct pinctrl_state *pins_mosi_spi, *pins_mosi_pullhigh, *pins_mosi_pulllow, *pins_mosi_pulldisable;
        struct pinctrl_state *pins_cs_spi,*pins_cs_pullhigh, *pins_cs_pulllow, *pins_cs_pulldisable;
        struct pinctrl_state *pins_clk_spi, *pins_clk_pullhigh, *pins_clk_pulllow, *pins_clk_pulldisable;
        struct pinctrl_state *pins_en_high, *pins_en_low;
        struct pinctrl_state *pins_irq_pulllow, *pins_irq_pulldisable;
        struct pinctrl_state *pins_lna_high, *pins_lna_low;
        struct pinctrl_state *pins_lna_en_high, *pins_lna_en_low;
        struct pinctrl_state *pins_ant_sw_enable, *pins_ant_sw_disable;

        struct pinctrl * pinctrl_gpios_for_lna;
        struct pinctrl * pinctrl_gpios_for_ant_switch;
#endif
};

static struct tdmb_fc8080_ctrl_blk fc8080_ctrl_info;

static Device_drv device_fc8080 = {
    &broadcast_fc8080_drv_if_power_on,
    &broadcast_fc8080_drv_if_power_off,
    &broadcast_fc8080_drv_if_init,
    &broadcast_fc8080_drv_if_stop,
    &broadcast_fc8080_drv_if_set_channel,
    &broadcast_fc8080_drv_if_detect_sync,
    &broadcast_fc8080_drv_if_get_sig_info,
    &broadcast_fc8080_drv_if_get_fic,
    &broadcast_fc8080_drv_if_get_msc,
    &broadcast_fc8080_drv_if_reset_ch,
    &broadcast_fc8080_drv_if_user_stop,
    &broadcast_fc8080_drv_if_select_antenna,
    &broadcast_fc8080_drv_if_set_nation,
    &broadcast_fc8080_drv_if_is_on
};

#if 0
void tdmb_fc8080_spi_write_read_test(void)
{
    uint16 i;
    uint32 wdata = 0;
    uint32 ldata = 0;
    uint32 data = 0;
    uint32 temp = 0;

#define TEST_CNT    5
//    tdmb_fc8080_power_on();

    printk("tdmb_fc8080_spi_write_read_test ++\n");
    for(i=0;i<TEST_CNT;i++)
    {
        bbm_com_write(NULL, 0xa4, i & 0xff);
        bbm_com_read(NULL, 0xa4, (fci_u8*)&data);
        printk("FC8080 byte test (0x%x,0x%x)\n", i & 0xff, data);
        if((i & 0xff) != data)
            printk("FC8080 byte test (0x%x,0x%x)\n", i & 0xff, data);
    }

    for(i=0;i<TEST_CNT;i++)
    {
        bbm_com_word_write(NULL, 0xa4, i & 0xffff);
        bbm_com_word_read(NULL, 0xa4, (fci_u16*)&wdata);
        printk("FC8080 word test (0x%x,0x%x)\n", i & 0xffff, wdata);
        if((i & 0xffff) != wdata)
            printk("FC8080 word test (0x%x,0x%x)\n", i & 0xffff, wdata);
    }

    for(i=0;i<TEST_CNT;i++)
    {
        bbm_com_long_write(NULL, 0xa4, i & 0xffffffff);
        bbm_com_long_read(NULL, 0xa4, (fci_u32*)&ldata);
        printk("FC8080 long test (0x%x,0x%x)\n", i & 0xffffffff, ldata);
        if((i & 0xffffffff) != ldata)
            printk("FC8080 long test (0x%x,0x%x)\n", i & 0xffffffff, ldata);
    }

    data = 0;

    for(i=0;i<TEST_CNT;i++)
    {
        temp = i&0xff;
        bbm_com_tuner_write(NULL, 0x58, 0x01, (fci_u8*)&temp, 0x01);
        bbm_com_tuner_read(NULL, 0x58, 0x01, (fci_u8*)&data, 0x01);
        printk("FC8080 tuner test (0x%x,0x%x)\n", i & 0xff, data);
        if((i & 0xff) != data)
            printk("FC8080 tuner test (0x%x,0x%x)\n", i & 0xff, data);
    }
   // tdmb_fc8080_power_off();
     printk("tdmb_fc8080_spi_write_read_test --\n");
}
#endif

static int tdmb_pinctrl_init_fc8080(void)
{
    int rc = ERROR;
#ifdef CONFIG_OF
    struct platform_device *pdev = NULL;

    pdev = of_find_device_by_node(fc8080_ctrl_info.dmb_irq_node);

    /* get pinctrl pointer */
    fc8080_ctrl_info.pinctrl_gpios = devm_pinctrl_get(&pdev->dev);

    if(IS_ERR_OR_NULL(fc8080_ctrl_info.pinctrl_gpios)) {
        printk("%s: Getting pinctrl handle failed\n", __func__);
        return -EINVAL;
    }

    /* it's normal that get "default" will failed */
    fc8080_ctrl_info.pins_default = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "default");
    if (IS_ERR(fc8080_ctrl_info.pins_default)) {
        printk("%s can't find tdmb pinctrl default\n", __func__);
        /* return -EINVAL; */
    }

    fc8080_ctrl_info.pins_miso_spi = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "miso_spi");
    if (IS_ERR(fc8080_ctrl_info.pins_miso_spi)) {
        printk("%s can't find tdmb pinctrl miso_spi\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_miso_pullhigh = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "miso_pullhigh");
    if (IS_ERR(fc8080_ctrl_info.pins_miso_pullhigh)) {
        printk("%s can't find tdmb pinctrl miso_pullhigh\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_miso_pulllow = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "miso_pulllow");
    if (IS_ERR(fc8080_ctrl_info.pins_miso_pulllow)) {
        printk("%s can't find tdmb pinctrl miso_pulllow\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_miso_pulldisable = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "miso_pulldisable");
    if (IS_ERR(fc8080_ctrl_info.pins_miso_pulldisable)) {
        printk("%s can't find tdmb pinctrl miso_pulldisable\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_mosi_spi = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "mosi_spi");
    if (IS_ERR(fc8080_ctrl_info.pins_mosi_spi)) {
        printk("%s can't find tdmb pinctrl pins_mosi_spi\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_mosi_pullhigh = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "mosi_pullhigh");
    if (IS_ERR(fc8080_ctrl_info.pins_mosi_pullhigh)) {
        printk("%s can't find tdmb pinctrl mosi_pullhigh\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_mosi_pulllow = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "mosi_pulllow");
    if (IS_ERR(fc8080_ctrl_info.pins_mosi_pulllow)) {
        printk("%s can't find tdmb pinctrl mosi_pulllow\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_mosi_pulldisable = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "mosi_pulldisable");
    if (IS_ERR(fc8080_ctrl_info.pins_mosi_pulldisable)) {
        printk("%s can't find tdmb pinctrl mosi_pulldisable\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_cs_spi = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "cs_spi");
    if (IS_ERR(fc8080_ctrl_info.pins_cs_spi)) {
        printk("%s can't find tdmb pinctrl pins_cs_spi\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_cs_pullhigh = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "cs_pullhigh");
    if (IS_ERR(fc8080_ctrl_info.pins_cs_pullhigh)) {
        printk("%s can't find tdmb pinctrl cs_pullhigh\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_cs_pulllow = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "cs_pulllow");
    if (IS_ERR(fc8080_ctrl_info.pins_cs_pulllow)) {
        printk("%s can't find tdmb pinctrl cs_pulllow\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_cs_pulldisable = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "cs_pulldisable");
    if (IS_ERR(fc8080_ctrl_info.pins_cs_pulldisable)) {
        printk("%s can't find tdmb pinctrl cs_pulldisable\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_clk_spi = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "clk_spi");
    if (IS_ERR(fc8080_ctrl_info.pins_clk_spi)) {
        printk("%s can't find tdmb pinctrl pins_clk_spi\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_clk_pullhigh = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "clk_pullhigh");
    if (IS_ERR(fc8080_ctrl_info.pins_clk_pullhigh)) {
        printk("%s can't find tdmb pinctrl clk_pullhigh\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_clk_pulllow = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "clk_pulllow");
    if (IS_ERR(fc8080_ctrl_info.pins_clk_pulllow)) {
        printk("%s can't find tdmb pinctrl clk_pulllow\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_clk_pulldisable = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "clk_pulldisable");
    if (IS_ERR(fc8080_ctrl_info.pins_clk_pulldisable)) {
        printk("%s can't find tdmb pinctrl clk_pulldisable\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_en_high = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "en_high");
    if (IS_ERR(fc8080_ctrl_info.pins_en_high)) {
        printk("%s can't find tdmb pinctrl en_high\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_en_low = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "en_low");
    if (IS_ERR(fc8080_ctrl_info.pins_en_low)) {
        printk("%s can't find tdmb pinctrl en_low\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_irq_pulllow = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "irq_low");
    if (IS_ERR(fc8080_ctrl_info.pins_irq_pulllow)) {
        printk("%s can't find tdmb pinctrl irq_low\n", __func__);
        return -EINVAL;
    }

    fc8080_ctrl_info.pins_irq_pulldisable = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios, "irq_disable");
    if (IS_ERR(fc8080_ctrl_info.pins_irq_pulldisable)) {
        printk("%s can't find tdmb pinctrl irq_low\n", __func__);
        return -EINVAL;
    }
    rc = OK;

    printk("[%s] get pinctrl success!\n", __func__);
#endif
    return rc;
}
static int tdmb_pinctrl_init_lna(void)
{
    int rc = ERROR;
#ifdef CONFIG_OF
    struct platform_device *pdev_lna = NULL;

    if(fc8080_ctrl_info.dmb_lna_node)
    {
        pdev_lna = of_find_device_by_node(fc8080_ctrl_info.dmb_lna_node);

        fc8080_ctrl_info.pinctrl_gpios_for_lna = devm_pinctrl_get(&pdev_lna->dev);
        if(IS_ERR_OR_NULL(fc8080_ctrl_info.pinctrl_gpios_for_lna))
        {
            printk("%s: Getting pinctrl handle for lna failed\n", __func__);
            fc8080_ctrl_info.pinctrl_gpios_for_lna = NULL;
        }
        rc = OK;
    }
    else
    {
        return ERROR;
    }

    if(rc == OK && fc8080_ctrl_info.pinctrl_gpios_for_lna)
    {
        fc8080_ctrl_info.pins_lna_high = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios_for_lna, "lna_high");
        if (IS_ERR(fc8080_ctrl_info.pins_lna_high)) {
            printk("%s can't find tdmb pinctrl lna_high(%p)\n", __func__, fc8080_ctrl_info.pins_lna_high);
            fc8080_ctrl_info.pins_lna_high = NULL;
        }

        fc8080_ctrl_info.pins_lna_low = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios_for_lna, "lna_low");
        if (IS_ERR(fc8080_ctrl_info.pins_lna_low)) {
            printk("%s can't find tdmb pinctrl lna_low(%p)\n", __func__, fc8080_ctrl_info.pins_lna_low);
            fc8080_ctrl_info.pins_lna_low = NULL;
        }

        fc8080_ctrl_info.pins_lna_en_high = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios_for_lna, "lna_en_high");
        if (IS_ERR(fc8080_ctrl_info.pins_lna_en_high)) {
            printk("%s can't find tdmb pinctrl lna_en_high(%p)\n", __func__, fc8080_ctrl_info.pins_lna_en_high);
            fc8080_ctrl_info.pins_lna_en_high = NULL;
        }

        fc8080_ctrl_info.pins_lna_en_low = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios_for_lna, "lna_en_low");
        if (IS_ERR(fc8080_ctrl_info.pins_lna_en_low)) {
            printk("%s can't find tdmb pinctrl lna_en_low(%p)\n", __func__, fc8080_ctrl_info.pins_lna_en_low);
            fc8080_ctrl_info.pins_lna_en_low = NULL;
        }
    }
#endif
    return rc;
}

static int tdmb_pinctrl_init_ant_switch(void)
{
    int rc = ERROR;
#ifdef CONFIG_OF
    struct platform_device *pdev_ant_switch = NULL;

    if(fc8080_ctrl_info.dmb_ant_switch_node)
    {
        pdev_ant_switch = of_find_device_by_node(fc8080_ctrl_info.dmb_ant_switch_node);

        fc8080_ctrl_info.pinctrl_gpios_for_ant_switch = devm_pinctrl_get(&pdev_ant_switch->dev);
        if(IS_ERR_OR_NULL(fc8080_ctrl_info.pinctrl_gpios_for_ant_switch))
        {
            printk("%s: Getting pinctrl handle for ant switch failed\n", __func__);
            fc8080_ctrl_info.pinctrl_gpios_for_ant_switch = NULL;
        }
        rc = OK;
    }
    else
    {
        return ERROR;
    }

    if(rc == OK && fc8080_ctrl_info.pinctrl_gpios_for_ant_switch)
    {
        fc8080_ctrl_info.pins_ant_sw_enable = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios_for_ant_switch, "ant_sw_enable");
        if (IS_ERR(fc8080_ctrl_info.pins_ant_sw_enable)) {
            printk("%s can't find tdmb pinctrl ant_sw_enable(%p)\n", __func__, fc8080_ctrl_info.pins_ant_sw_enable);
            fc8080_ctrl_info.pins_ant_sw_enable = NULL;
        }

        fc8080_ctrl_info.pins_ant_sw_disable = pinctrl_lookup_state(fc8080_ctrl_info.pinctrl_gpios_for_ant_switch, "ant_sw_disable");
        if (IS_ERR(fc8080_ctrl_info.pins_ant_sw_disable)) {
            printk("%s can't find tdmb pinctrl ant_sw_disable(%p)\n", __func__, fc8080_ctrl_info.pins_ant_sw_disable);
            fc8080_ctrl_info.pins_ant_sw_disable = NULL;
        }
    }
#endif
    return rc;
}
static int tdmb_pinctrl_init(void)
{
    int rc = ERROR;
    int status = ERROR;

    rc = tdmb_pinctrl_init_fc8080();
    if(rc == OK)
    {
        status = tdmb_pinctrl_init_lna();
        if(status == ERROR)
        {
            printk("[%s] LNA pinctrl is not initialized\n",__func__);
        }
        status = tdmb_pinctrl_init_ant_switch();
        if(status == ERROR)
        {
            printk("[%s] Ant Switch pinctrl is not initialized\n",__func__);
        }
    }
    else
    {
        return ERROR;
    }

    return rc;
}

void tunerbb_drv_load_hw_configure(void)
{
    fc8080_ctrl_info.dmb_irq_node = of_find_compatible_node(NULL, NULL, "mediatek,dtv");
    if (fc8080_ctrl_info.dmb_irq_node) {

        /*load interrupt configuration to register irq function*/
        fc8080_ctrl_info.dmb_irq = irq_of_parse_and_map(fc8080_ctrl_info.dmb_irq_node, 0);

        /*load gpio configuration */
        printk("[%s] dmb_irq = %d\n", __func__, fc8080_ctrl_info.dmb_irq);

        of_property_read_u32(fc8080_ctrl_info.dmb_irq_node, "spi-bus-num", &fc8080_ctrl_info.dmb_spi_bus_num);

        printk("[%s] dmb_spi_bus_num = %d\n", __func__, fc8080_ctrl_info.dmb_spi_bus_num);

        /*load hw configruation value*/
        of_property_read_u32(fc8080_ctrl_info.dmb_irq_node, "use-xtal", &fc8080_ctrl_info.dmb_use_xtal);
        of_property_read_u32(fc8080_ctrl_info.dmb_irq_node, "xtal-freq", &fc8080_ctrl_info.dmb_xtal_freq);

        printk("[%s] dmb_use_xtal = %d, dmb_xtal_freq=%d\n", __func__,
            fc8080_ctrl_info.dmb_use_xtal, fc8080_ctrl_info.dmb_xtal_freq);
    }

    fc8080_ctrl_info.dmb_lna_node = of_find_compatible_node(NULL, NULL, "lge,dtv_lna");
    if(fc8080_ctrl_info.dmb_lna_node)
    {
        if (!of_device_is_available(fc8080_ctrl_info.dmb_lna_node)) {
            printk("[%s] dmb_lna_node is not availabe\n",__func__);
            fc8080_ctrl_info.dmb_lna_node = NULL;
        }
    }

    fc8080_ctrl_info.dmb_ant_switch_node = of_find_compatible_node(NULL, NULL, "lge,dtv_ant_switch");
    if(fc8080_ctrl_info.dmb_ant_switch_node)
    {
        if (!of_device_is_available(fc8080_ctrl_info.dmb_ant_switch_node)) {
            printk("[%s] dmb_ant_switch_node is not availabe\n",__func__);
            fc8080_ctrl_info.dmb_ant_switch_node = NULL;
        }
    }
}

void tunerbb_drv_hw_setting(void)
{
    printk("[%s]!!!\n", __func__);
#ifdef CONFIG_OF
    // DMB EN
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_en_low);
    mdelay(50);

    // IRQ
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_irq_pulllow);
    mdelay(50);

    // LNA
    if(!IS_ERR_OR_NULL(fc8080_ctrl_info.pins_lna_en_low)) {
        pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_lna_en_low);
    }
    if(!IS_ERR_OR_NULL(fc8080_ctrl_info.pins_lna_low)) {
        pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_lna_low);
    }

    //SPI MISO
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_miso_pulllow);
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_miso_spi);

    //SPI MOSI
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_mosi_pulllow);
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_mosi_spi);

    //SPI CLK
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_clk_pulllow);
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_clk_spi);

    //SPI CS
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_cs_pulllow);
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_cs_spi);
#endif
}

void tunerbb_drv_hw_init(void)
{
    printk("[tdmb][MTK] tunerbb_drv_hw_init\n");
#ifdef CONFIG_OF
    // DMB EN
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_en_high);
    mdelay(30);

    // IRQ
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_irq_pulldisable);

    // LNA
    if(!IS_ERR_OR_NULL(fc8080_ctrl_info.pins_lna_en_high)) {
        pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_lna_en_high);
    }
    if(!IS_ERR_OR_NULL(fc8080_ctrl_info.pins_lna_low)) {
        pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_lna_low);
    }

    // ANT Switch
    if(!IS_ERR_OR_NULL(fc8080_ctrl_info.pins_ant_sw_enable)) {
        pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_ant_sw_enable);
    }

    // SPI MISO
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_miso_pulldisable);
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_miso_spi);

    // SPI MOSI
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_mosi_pulldisable);
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_mosi_spi);

    // SPI CLK
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_clk_pulldisable);
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_clk_spi);

    // SPI CS
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_cs_pulldisable);
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_cs_spi);
#endif
}


void tunerbb_drv_hw_deinit(void)
{
    printk("[tdmb][MTK] tunerbb_drv_hw_deinit\n");
#ifdef CONFIG_OF
    //tdmb_fc8080_interrupt_lock();
    // DMB EN
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_en_low);

    // IRQ
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_irq_pulllow);

    // LNA
    if(!IS_ERR_OR_NULL(fc8080_ctrl_info.pins_lna_low)) {
        pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_lna_low);
    }
    if(!IS_ERR_OR_NULL(fc8080_ctrl_info.pins_lna_en_low)) {
        pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_lna_en_low);
    }

    // ANT Switch
    if(!IS_ERR_OR_NULL(fc8080_ctrl_info.pins_ant_sw_disable)) {
        pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_ant_sw_disable);
    }

    // SPI MISO
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_miso_pulllow);

    // SPI MOSI
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_mosi_pulllow);

    // SPI CLK
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_clk_pulllow);

    // SPI CS
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_cs_pulllow);
#endif
}

struct spi_device *tdmb_fc8080_get_spi_device(void)
{
    return fc8080_ctrl_info.spi_ptr;
}


void tdmb_fc8080_set_userstop(int mode)
{
    user_stop_flg = mode;
    printk("tdmb_fc8080_set_userstop, user_stop_flg = %d \n", user_stop_flg);
}

int tdmb_fc8080_mdelay(int32 ms)
{
    int32    wait_loop =0;
    int32    wait_ms = ms;
    int        rc = 1;  /* 0 : false, 1 : true */

    if(ms > 100)
    {
        wait_loop = (ms /100);   /* 100, 200, 300 more only , Otherwise this must be modified e.g (ms + 40)/50 */
        wait_ms = 100;
    }

    do
    {
        mdelay(wait_ms);

        if(user_stop_flg == 1)
        {
            printk("~~~~~~~~ Ustop flag is set so return false ms =(%d)~~~~~~~\n", ms);
            rc = 0;
            break;
        }
    }while((--wait_loop) > 0);

    return rc;
}

void tdmb_fc8080_Must_mdelay(int32 ms)
{
    mdelay(ms);
}

int tdmb_fc8080_tdmb_is_on(void)
{
    return (int)fc8080_ctrl_info.TdmbPowerOnState;
}

unsigned int tdmb_fc8080_get_xtal_freq(void)
{
    return (unsigned int)fc8080_ctrl_info.dmb_xtal_freq;
}

/* EXPORT_SYMBOL() : when we use external symbol
which is not included in current module - over kernel 2.6 */
//EXPORT_SYMBOL(tdmb_fc8080_tdmb_is_on);
/*[BCAST002][S] 20140804 seongeun.jin - modify chip init check timing issue on BLT*/
#ifdef FEATURE_POWER_ON_RETRY
int tdmb_fc8080_power_on_retry(void)
{
    int res;
    int i;

    tdmb_fc8080_interrupt_lock();

    for(i = 0 ; i < 10; i++)
    {
        printk("[FC8080] tdmb_fc8080_power_on_retry :  %d\n", i);
        if(!fc8080_ctrl_info.dmb_use_xtal) {
            if(fc8080_ctrl_info.clk != NULL) {
                clk_disable_unprepare(fc8080_ctrl_info.clk);
                printk("[FC8080] retry clk_disable %d\n", i);
            }
            else
                printk("[FC8080] ERR fc8080_ctrl_info.clkdis is NULL\n");
        }

        tunerbb_drv_hw_init();

        if(!fc8080_ctrl_info.dmb_use_xtal) {
#if 0
            if(fc8080_ctrl_info.clk != NULL) {
                res = clk_prepare_enable(fc8080_ctrl_info.clk);
                if (res) {
                    printk("[FC8080] retry clk_prepare_enable fail %d\n", i);
                }
            }
            else
                printk("[FC8080] ERR fc8080_ctrl_info.clken is NULL\n");
#else
            printk("[FC8080] RF clock enable\n");
            clk_buf_ctrl(CLK_BUF_CONN, true);
#endif
        }
        mdelay(30);

        res = bbm_com_probe(NULL);

        if (!res)
            break;

    }

    tdmb_fc8080_interrupt_free();

    return res;
}
#endif
/*[BCAST002][E]*/
int tdmb_fc8080_power_on(void)
{
    int rc = FALSE;

    printk("tdmb_fc8080_power_on \n");
    if ( fc8080_ctrl_info.TdmbPowerOnState == FALSE )
    {
        wake_lock(&fc8080_ctrl_info.wake_lock);

        tunerbb_drv_hw_init();

#ifdef FEATURE_DMB_USE_BUS_SCALE
        msm_bus_scale_client_update_request(fc8080_ctrl_info.bus_scale_client_id, 1); /* expensive call, index:1 is the <84 512 3000 152000> entry */
#endif
        if(!fc8080_ctrl_info.dmb_use_xtal) {
#if 0
            if(fc8080_ctrl_info.clk != NULL) {
                rc = clk_prepare_enable(fc8080_ctrl_info.clk);
                if (rc) {
#ifdef CONFIG_OF
                    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_en_low);
#endif
                    mdelay(5);
                    dev_err(&fc8080_ctrl_info.spi_ptr->dev, "could not enable clock\n");
                    return rc;
                }
            }
#else
            printk("[FC8080] RF clock enable\n");
            clk_buf_ctrl(CLK_BUF_CONN, true);
#endif
        }
#ifdef FEATURE_DMB_USE_PM_QOS
        if(pm_qos_request_active(&fc8080_ctrl_info.pm_req_list)) {
            pm_qos_update_request(&fc8080_ctrl_info.pm_req_list, 20);
        }
#endif
//        gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_P-1), 0);
//        gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_N-1), 1);

        mdelay(30); /* Due to X-tal stablization Time */

        tdmb_fc8080_interrupt_free();

        fc8080_ctrl_info.TdmbPowerOnState = TRUE;

        printk("tdmb_fc8080_power_on OK\n");
    }
    else
    {
        printk("tdmb_fc8080_power_on the power already turn on \n");
    }

    printk("tdmb_fc8080_power_on completed \n");
    rc = TRUE;

    return rc;
}

int tdmb_fc8080_power_off(void)
{
    if ( fc8080_ctrl_info.TdmbPowerOnState == TRUE )
    {
        tdmb_fc8080_interrupt_lock();

        if(!fc8080_ctrl_info.dmb_use_xtal) {
#if 0
            if(fc8080_ctrl_info.clk != NULL) {
                clk_disable_unprepare(fc8080_ctrl_info.clk);
            }
#else
            printk("[FC8080] RF clock disable\n");
            clk_buf_ctrl(CLK_BUF_CONN, false);
#endif
        }

        fc8080_ctrl_info.TdmbPowerOnState = FALSE;

        tunerbb_drv_hw_deinit();

        wake_unlock(&fc8080_ctrl_info.wake_lock);
        mdelay(20);

//        gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_P-1), 1);    // for ESD TEST
//        gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_N-1), 0);

#ifdef FEATURE_DMB_USE_BUS_SCALE
        msm_bus_scale_client_update_request(fc8080_ctrl_info.bus_scale_client_id, 0); /* expensive call, index:0 is the <84 512 0 0> entry */
#endif
#ifdef FEATURE_DMB_USE_PM_QOS
        if(pm_qos_request_active(&fc8080_ctrl_info.pm_req_list)) {
            pm_qos_update_request(&fc8080_ctrl_info.pm_req_list, PM_QOS_DEFAULT_VALUE);
        }
#endif
    }
    else
    {
        printk("tdmb_fc8080_power_on the power already turn off \n");
    }

    printk("tdmb_fc8080_power_off completed \n");
    return TRUE;
}

int tdmb_fc8080_select_antenna(unsigned int sel)
{
    return FALSE;
}

void tdmb_fc8080_interrupt_lock(void)
{
    if (fc8080_ctrl_info.spi_ptr == NULL)
    {
        printk("tdmb_fc8080_interrupt_lock fail\n");
    }
    else
    {
        mt_eint_mask(fc8080_ctrl_info.dmb_irq);
    }
}

void tdmb_fc8080_interrupt_free(void)
{
    if (fc8080_ctrl_info.spi_ptr == NULL)
    {
        printk("tdmb_fc8080_interrupt_free fail\n");
    }
    else
    {
        mt_eint_unmask(fc8080_ctrl_info.dmb_irq);
    }
}

int tdmb_fc8080_spi_write_read(uint8* tx_data, int tx_length, uint8 *rx_data, int rx_length)
{
    int rc;

    struct spi_transfer    t = {
            .tx_buf        = tx_data,
            .rx_buf        = rx_data,
            .len        = tx_length+rx_length,
        };

    struct spi_message    m;

    if (fc8080_ctrl_info.spi_ptr == NULL)
    {
        printk("tdmb_fc8080_spi_write_read error txdata=0x%x, length=%d\n", (unsigned int)tx_data, tx_length+rx_length);
        return FALSE;
    }

    mutex_lock(&fc8080_ctrl_info.mutex);

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    rc = spi_sync(fc8080_ctrl_info.spi_ptr, &m);

    if ( rc < 0 )
    {
        printk("tdmb_fc8080_spi_read_burst result(%d), actual_len=%d\n",rc, m.actual_length);
    }

    mutex_unlock(&fc8080_ctrl_info.mutex);

    return TRUE;
}

#ifdef FEATURE_DMB_USE_WORKQUEUE
static irqreturn_t broadcast_tdmb_spi_isr(int irq, void *handle)
{
    struct tdmb_fc8080_ctrl_blk* fc8080_info_p;

    fc8080_info_p = (struct tdmb_fc8080_ctrl_blk *)handle;
    if ( fc8080_info_p && fc8080_info_p->TdmbPowerOnState )
    {
        unsigned long flag;
        if (fc8080_info_p->spi_irq_status)
        {
            printk("######### spi read function is so late skip #########\n");
            return IRQ_HANDLED;
        }
//        printk("***** broadcast_tdmb_spi_isr coming *******\n");
        spin_lock_irqsave(&fc8080_info_p->spin_lock, flag);
        queue_work(fc8080_info_p->spi_wq, &fc8080_info_p->spi_work);
        spin_unlock_irqrestore(&fc8080_info_p->spin_lock, flag);
    }
    else
    {
        printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
    }

    return IRQ_HANDLED;
}

static void broacast_tdmb_spi_work(struct work_struct *tdmb_work)
{
    struct tdmb_fc8080_ctrl_blk *pTdmbWorkData;

    pTdmbWorkData = container_of(tdmb_work, struct tdmb_fc8080_ctrl_blk, spi_work);
    if ( pTdmbWorkData )
    {
        tunerbb_drv_fc8080_isr_control(0);
        pTdmbWorkData->spi_irq_status = TRUE;
        broadcast_fc8080_drv_if_isr();
        pTdmbWorkData->spi_irq_status = FALSE;
        tunerbb_drv_fc8080_isr_control(1);
    }
    else
    {
        printk("~~~~~~~broadcast_tdmb_spi_work call but pTdmbworkData is NULL ~~~~~~~\n");
    }
}
#else

static irqreturn_t broadcast_tdmb_spi_event_handler(int irq, void *handle)
{
    struct tdmb_fc8080_ctrl_blk* fc8080_info_p;
    fc8080_info_p = (struct tdmb_fc8080_ctrl_blk *)handle;

    if ( fc8080_info_p && fc8080_info_p->TdmbPowerOnState )
    {
        if (fc8080_info_p->spi_irq_status)
        {
            printk("######### spi read function is so late skip ignore #########\n");
            return IRQ_HANDLED;
        }
        tunerbb_drv_fc8080_isr_control(0);
        fc8080_info_p->spi_irq_status = TRUE;
        broadcast_fc8080_drv_if_isr();
        fc8080_info_p->spi_irq_status = FALSE;
        tunerbb_drv_fc8080_isr_control(1);
    }
    else
    {
        printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
    }
    return IRQ_HANDLED;
}
#endif

static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias="dmb_spi",
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
       //.max_speed_hz = 24*1000*1000,//24MHz
        .controller_data  = &mtk_spi_config,
    },
};

static int broadcast_tdmb_fc8080_probe(struct spi_device *spi)
{
    int rc;

    if(spi == NULL)
    {
        printk("broadcast_fc8080_probe spi is NULL, so spi can not be set\n");
        return -1;
    }
    fc8080_ctrl_info.TdmbPowerOnState = FALSE;
    fc8080_ctrl_info.spi_ptr                 = spi;
    fc8080_ctrl_info.spi_ptr->mode             = SPI_MODE_0;
    fc8080_ctrl_info.spi_ptr->bits_per_word     = 8;
    fc8080_ctrl_info.spi_ptr->max_speed_hz     = (16000*1000);
    fc8080_ctrl_info.pdev = to_platform_device(&spi->dev);

#ifdef FEATURE_DMB_USE_BUS_SCALE
    fc8080_ctrl_info.bus_scale_pdata = msm_bus_cl_get_pdata(fc8080_ctrl_info.pdev);
    fc8080_ctrl_info.bus_scale_client_id = msm_bus_scale_register_client(fc8080_ctrl_info.bus_scale_pdata);
#endif

    // Once I have a spi_device structure I can do a transfer anytime

    rc = spi_setup(spi);
    printk("broadcast_tdmb_fc8080_probe spi_setup=%d\n", rc);
       if (rc){
            printk( "[fc8080] Spi setup error(%d)\n", rc);
            return rc;
        }
    bbm_com_hostif_select(NULL, 1);


    if(!fc8080_ctrl_info.dmb_use_xtal) {
#if 0
        fc8080_ctrl_info.clk = clk_get(&fc8080_ctrl_info.spi_ptr->dev, "tdmb_xo");
        if (IS_ERR(fc8080_ctrl_info.clk)) {
            rc = PTR_ERR(fc8080_ctrl_info.clk);
            dev_err(&fc8080_ctrl_info.spi_ptr->dev, "could not get clock\n");
            return rc;
        }

        /* We enable/disable the clock only to assure it works */
        rc = clk_prepare_enable(fc8080_ctrl_info.clk);
        if (rc) {
            dev_err(&fc8080_ctrl_info.spi_ptr->dev, "could not enable clock\n");
            return rc;
        }
        clk_disable_unprepare(fc8080_ctrl_info.clk);
#endif
    }

#ifdef FEATURE_DMB_USE_WORKQUEUE
    INIT_WORK(&fc8080_ctrl_info.spi_work, broacast_tdmb_spi_work);
    fc8080_ctrl_info.spi_wq = create_singlethread_workqueue("tdmb_spi_wq");
    if(fc8080_ctrl_info.spi_wq == NULL){
        printk("Failed to setup tdmb spi workqueue \n");
        return -ENOMEM;
    }
#endif
    rc = tdmb_pinctrl_init();

    if(rc) {
        printk("[%s] tdmb_pinctrl_init =%d\n",__func__, rc);
        return rc;
    }

    tunerbb_drv_hw_setting();

    if(fc8080_ctrl_info.dmb_irq <= 0) {
        printk("[tdmb] irq_of_parse_and_map IRQ can't parsing!.\n");
    } else {
        rc = request_threaded_irq(fc8080_ctrl_info.dmb_irq, NULL, broadcast_tdmb_spi_event_handler, 
            IRQF_ONESHOT | IRQF_DISABLED | IRQF_TRIGGER_FALLING, spi->dev.driver->name, &fc8080_ctrl_info);

        if (rc){
            printk("[tdmb] dmb request irq fail : %d\n", rc);
        }
    }

    tdmb_fc8080_interrupt_lock();

    mutex_init(&fc8080_ctrl_info.mutex);

    wake_lock_init(&fc8080_ctrl_info.wake_lock,  WAKE_LOCK_SUSPEND, dev_name(&spi->dev));

    spin_lock_init(&fc8080_ctrl_info.spin_lock);

#ifdef FEATURE_DMB_USE_PM_QOS
    pm_qos_add_request(&fc8080_ctrl_info.pm_req_list, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
#endif
    printk("broadcast_fc8080_probe End\n");

    return rc;
}
/*
static int broadcast_tdmb_fc8080_remove(struct spi_device *spi)
{
    printk("broadcast_tdmb_fc8080_remove \n");

#ifdef FEATURE_DMB_USE_WORKQUEUE
    if (fc8080_ctrl_info.spi_wq)
    {
        flush_workqueue(fc8080_ctrl_info.spi_wq);
        destroy_workqueue(fc8080_ctrl_info.spi_wq);
    }
#endif

#ifdef FEATURE_DMB_USE_BUS_SCALE
    msm_bus_scale_unregister_client(fc8080_ctrl_info.bus_scale_client_id);
#endif
#ifdef CONFIG_OF
    pinctrl_select_state(fc8080_ctrl_info.pinctrl_gpios, fc8080_ctrl_info.pins_irq_pulldisable);
#endif

    mutex_destroy(&fc8080_ctrl_info.mutex);

    wake_lock_destroy(&fc8080_ctrl_info.wake_lock);

#ifdef FEATURE_DMB_USE_PM_QOS
    pm_qos_remove_request(&fc8080_ctrl_info.pm_req_list);
#endif
    memset((unsigned char*)&fc8080_ctrl_info, 0x0, sizeof(struct tdmb_fc8080_ctrl_blk));
    return 0;
}
*/
static int broadcast_tdmb_fc8080_suspend(struct spi_device *spi, pm_message_t mesg)
{
    printk("broadcast_tdmb_fc8080_suspend \n");
    return 0;
}

static int broadcast_tdmb_fc8080_resume(struct spi_device *spi)
{
    printk("broadcast_tdmb_fc8080_resume \n");
    return 0;
}
#if 0
static int broadcast_tdmb_fc8080_check_chip_id(void)
{
    int rc = ERROR;

    rc = tdmb_fc8080_power_on();

    if(rc == TRUE && !bbm_com_probe(NULL)) //rc == TRUE : power on success,  OK : 0
        rc = OK;
    else
        rc = ERROR;

    tdmb_fc8080_power_off();

    return rc;
}
#endif

#ifdef CONFIG_OF
struct of_device_id tdmb_spi_table[] = {
    {
        .compatible = "broadcast,tdmb",
    },
    {}
};

MODULE_DEVICE_TABLE(of, tdmb_spi_table);
#endif

static struct spi_driver broadcast_tdmb_driver = {
    .driver = {
        .name = "dmb_spi",
#ifdef CONFIG_OF
        .of_match_table = tdmb_spi_table,
#endif
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
    },
    .probe = broadcast_tdmb_fc8080_probe,
    .remove= __exit_p(	),
    .suspend = broadcast_tdmb_fc8080_suspend,
    .resume  = broadcast_tdmb_fc8080_resume,
};

static int __init broadcast_tdmb_fc8080_drv_init(void)
{
    int rc;

    if(broadcast_tdmb_drv_check_module_init() != OK) {
        rc = ERROR;
        return rc;
    }

    tunerbb_drv_load_hw_configure();    
    spi_board_devs[0].bus_num = fc8080_ctrl_info.dmb_spi_bus_num;
    spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));

    rc = spi_register_driver(&broadcast_tdmb_driver);
    printk("[fc8080] spi_register_driver %d \n", rc);
#if 0
    if(broadcast_tdmb_fc8080_check_chip_id() != OK) {
        spi_unregister_driver(&broadcast_tdmb_driver);
        rc = ERROR;
        return rc;
    }
#endif

    rc = broadcast_tdmb_drv_start(&device_fc8080);
        if (rc < 0)
            printk("[fc8080] SPI driver register failed\n");

    printk("LGE_FC8080_DRV_VER : %s\n", LGE_FC8080_DRV_VER);
    return rc;
}

static void __exit broadcast_tdmb_fc8080_drv_exit(void)
{
    spi_unregister_driver(&broadcast_tdmb_driver);
}

module_init(broadcast_tdmb_fc8080_drv_init);
module_exit(broadcast_tdmb_fc8080_drv_exit);

/* optional part when we include driver code to build-on
it's just used when we make device driver to module(.ko)
so it doesn't work in build-on */
MODULE_DESCRIPTION("FC8080 tdmb device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("FCI");
