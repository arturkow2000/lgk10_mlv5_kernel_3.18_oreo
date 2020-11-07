#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/async.h>

#include "broadcast_dmb_typedef.h"
#include "broadcast_dmb_drv_ifdef.h"
#include "broadcast_fc8180.h"
#include "fci_types.h"
#include "fci_oal.h"
#include "bbm.h"
#include "fc8180_drv_api.h"
#include "fc8180_isr.h"
#include "fci_ringbuffer.h"

#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#if defined(CONFIG_ARCH_MT6755)
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/irqchip/mt-eic.h>
#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt6755/include/mach/gpio_const.h>
#include "../../../../spi/mediatek/mt6755/mt_spi.h"
#include "mt_clkbuf_ctl.h"

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
#endif

struct ISDBT_INIT_INFO_T *hInit;

//#define FEATURE_DTV_USE_PINCTRL

#define FC8180_NAME        "broadcast1"

#define RING_BUFFER_SIZE	(188 * 320)
static struct task_struct *isdbt_kthread;
struct fci_ringbuffer		RingBuffer;
static wait_queue_head_t isdbt_isr_wait;
u8 irq_error_cnt;
static u8 isdbt_isr_sig=0;
static u8 isdbt_isr_start=0;
u32 totalTS=0;
u32 totalErrTS=0;

u8 module_init_flag;

enum ISDBT_MODE{
	ISDBT_POWERON       = 0,
	ISDBT_POWEROFF	    = 1,
	ISDBT_DATAREAD		= 2
};
enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;

u8 static_ringbuffer[RING_BUFFER_SIZE];
static DEFINE_MUTEX(ringbuffer_lock);

//#define _NOT_USE_WAKE_LOCK_
struct broadcast_fc8180_ctrl_data {
	int			pwr_state;
	struct wake_lock	wake_lock;
	struct spi_device	*spi_dev;
	struct i2c_client	*pclient;
    struct platform_device                    *pdev;
    struct clk                                *clk;
    struct device_node *dtv_irq_node;

    uint32                            dtv_en;
    uint32                            dtv_irq;
    uint32                            dtv_irq_gpio;
    uint32                            dtv_ant_sw_gipo;

    uint32                            dtv_xtal_freq;
    uint32                            dtv_use_xtal;
    uint32                            dtv_use_ant_sw;
    uint32                            dtv_ant_sw_active_value;

    uint32                            dtv_spi_miso;
    uint32                            dtv_spi_mosi;
    uint32                            dtv_spi_clk;
    uint32                            dtv_spi_cs;

    uint32                            dtv_spi_bus_num;
};

static struct broadcast_fc8180_ctrl_data  IsdbCtrlInfo;

struct i2c_client *FCI_GET_I2C_DRIVER(void)
{
	return IsdbCtrlInfo.pclient;
}

struct spi_device *FCI_GET_SPI_DRIVER(void)
{
	return IsdbCtrlInfo.spi_dev;
}

static Device_drv device_fc8180 = {
    &broadcast_fc8180_drv_if_power_on,
    &broadcast_fc8180_drv_if_power_off,
    &broadcast_fc8180_drv_if_open,
    &broadcast_fc8180_drv_if_close,
    &broadcast_fc8180_drv_if_set_channel,
    &broadcast_fc8180_drv_if_resync,
    &broadcast_fc8180_drv_if_detect_sync,
    &broadcast_fc8180_drv_if_get_sig_info,
    &broadcast_fc8180_drv_if_get_ch_info,
    &broadcast_fc8180_drv_if_get_dmb_data,
    &broadcast_fc8180_drv_if_reset_ch,
    &broadcast_fc8180_drv_if_user_stop,
    &broadcast_fc8180_drv_if_select_antenna,
    &broadcast_fc8180_drv_if_isr,
    &broadcast_fc8180_drv_if_read_control,
    &broadcast_fc8180_drv_if_get_mode,
};

#ifdef FEATURE_DTV_USE_PINCTRL
static int oneseg_pinctrl_init(void)
{
    struct pinctrl *oneseg_pinctrl;
    struct pinctrl_state *gpio_state_suspend;

    printk("[dtv] oneseg_pinctrl_init\n");

    oneseg_pinctrl = devm_pinctrl_get(&IsdbCtrlInfo.pdev->dev);

    if(IS_ERR_OR_NULL(oneseg_pinctrl)) {
        pr_err("%s: Getting pinctrl handle failed\n", __func__);
        return -EINVAL;
    }
    gpio_state_suspend
     = pinctrl_lookup_state(oneseg_pinctrl, "gpio_oneseg_suspend");

     if(IS_ERR_OR_NULL(gpio_state_suspend)) {
         pr_err("%s: Failed to get the suspend state pinctrl handle\n", __func__);
         return -EINVAL;
    }

    if(pinctrl_select_state(oneseg_pinctrl, gpio_state_suspend)) {
        pr_err("%s: error on pinctrl_select_state for oneseg enable and irq pin\n", __func__);
        return -EINVAL;
    }
    else {
        printk("%s: success to set pinctrl_select_state for oneseg enable and irq pin\n", __func__);
    }

    return 0;
}
#endif
void tunerbb_drv_load_hw_configure(void)
{
    IsdbCtrlInfo.dtv_irq_node = of_find_compatible_node(NULL, NULL, "mediatek,dtv");
    if (IsdbCtrlInfo.dtv_irq_node) {

        /*load interrupt configuration to register irq function*/
        IsdbCtrlInfo.dtv_irq = irq_of_parse_and_map(IsdbCtrlInfo.dtv_irq_node, 0);

        /*load gpio configuration */
        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "irq-gpio", &IsdbCtrlInfo.dtv_irq_gpio);
        IsdbCtrlInfo.dtv_irq_gpio |= 0x80000000;

        printk("[%s] dtv_irq = %d, dtv_irq_gpio=%d\n", __func__, IsdbCtrlInfo.dtv_irq, IsdbCtrlInfo.dtv_irq_gpio & ~0x80000000);

        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "en-gpio", &IsdbCtrlInfo.dtv_en);
        IsdbCtrlInfo.dtv_en |= 0x80000000;

        printk("[%s] dtv_en = %d\n", __func__, IsdbCtrlInfo.dtv_en & ~0x80000000);

        //of_property_read_u32(fc8080_ctrl_info.dmb_irq_node, "ant-sw-gpio", &fc8080_ctrl_info.dmb_ant);
        //fc8080_ctrl_info.dmb_ant |= 0x80000000;

        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "spi-miso-gpio", &IsdbCtrlInfo.dtv_spi_miso);
        IsdbCtrlInfo.dtv_spi_miso |= 0x80000000;

        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "spi-mosi-gpio", &IsdbCtrlInfo.dtv_spi_mosi);
        IsdbCtrlInfo.dtv_spi_mosi |= 0x80000000;

        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "spi-clk-gpio", &IsdbCtrlInfo.dtv_spi_clk);
        IsdbCtrlInfo.dtv_spi_clk |= 0x80000000;

        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "spi-cs-gpio", &IsdbCtrlInfo.dtv_spi_cs);
        IsdbCtrlInfo.dtv_spi_cs |= 0x80000000;

        printk("[%s] dtv_spi_miso = %d, dtv_spi_mosi=%d\n", __func__, IsdbCtrlInfo.dtv_spi_miso & ~0x80000000, IsdbCtrlInfo.dtv_spi_mosi & ~0x80000000);
        printk("[%s] dtv_spi_clk = %d, dtv_spi_cs=%d\n", __func__, IsdbCtrlInfo.dtv_spi_clk & ~0x80000000, IsdbCtrlInfo.dtv_spi_cs & ~0x80000000);

        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "spi-bus-num", &IsdbCtrlInfo.dtv_spi_bus_num);

        printk("[%s] dtv_spi_bus_num = %d\n", __func__, IsdbCtrlInfo.dtv_spi_bus_num);

        /*load hw configruation value*/
        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "use-xtal", &IsdbCtrlInfo.dtv_use_xtal);
        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "use-ant-sw", &IsdbCtrlInfo.dtv_use_ant_sw);
        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "xtal-freq", &IsdbCtrlInfo.dtv_xtal_freq);
        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "ant-sw-gpio", &IsdbCtrlInfo.dtv_ant_sw_gipo);
        IsdbCtrlInfo.dtv_ant_sw_gipo |= 0x80000000;

        of_property_read_u32(IsdbCtrlInfo.dtv_irq_node, "ant-sw-active-value", &IsdbCtrlInfo.dtv_ant_sw_active_value);

        printk("[%s] dtv_use_xtal = %d, dtv_xtal_freq=%d\n", __func__, IsdbCtrlInfo.dtv_use_xtal, IsdbCtrlInfo.dtv_xtal_freq);
        printk("[%s] dtv_use_ant_sw=%d, dtv_ant_sw_gpio=%d\n", __func__, IsdbCtrlInfo.dtv_use_ant_sw, IsdbCtrlInfo.dtv_ant_sw_gipo & ~0x80000000);
    }
}


void tunerbb_drv_hw_setting(void)
{
    print_log(NULL, "[1seg][MTK] GPIO_ISDBT_PWR_EN Port request!!!\n");

    if(IsdbCtrlInfo.dtv_use_ant_sw) {
        mt_set_gpio_mode(IsdbCtrlInfo.dtv_ant_sw_gipo, GPIO_MODE_00);
        mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_ant_sw_gipo, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_ant_sw_gipo, GPIO_PULL_DOWN);
        mt_set_gpio_dir(IsdbCtrlInfo.dtv_ant_sw_gipo, GPIO_DIR_OUT);
        mt_set_gpio_out(IsdbCtrlInfo.dtv_ant_sw_gipo, IsdbCtrlInfo.dtv_ant_sw_active_value);
    }

    mt_set_gpio_mode(IsdbCtrlInfo.dtv_en, GPIO_MODE_00);
    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_en, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_en, GPIO_PULL_DOWN);
    ms_wait(50);
    mt_set_gpio_dir(IsdbCtrlInfo.dtv_en, GPIO_DIR_OUT);
    mt_set_gpio_out(IsdbCtrlInfo.dtv_en, GPIO_OUT_ZERO);

    print_log(NULL, "[1seg][MTK] ISDBT_IRQ_INT Port request!!!\n");
    mt_set_gpio_mode(IsdbCtrlInfo.dtv_irq_gpio, GPIO_MODE_00);
    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_irq_gpio, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_irq_gpio, GPIO_PULL_DOWN);
    mt_set_gpio_dir(IsdbCtrlInfo.dtv_irq_gpio, GPIO_DIR_IN);
    ms_wait(50);

    //SPI MISO
    mt_set_gpio_mode(IsdbCtrlInfo.dtv_spi_miso, GPIO_MODE_02);
    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_miso, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_spi_miso, GPIO_PULL_DOWN);

    //SPI MOSI
    mt_set_gpio_mode(IsdbCtrlInfo.dtv_spi_mosi, GPIO_MODE_02);
    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_mosi, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_spi_mosi, GPIO_PULL_DOWN);

    //SPI CLK
    mt_set_gpio_mode(IsdbCtrlInfo.dtv_spi_clk, GPIO_MODE_02);
    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_clk , GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_spi_clk , GPIO_PULL_DOWN);

    //SPI CS
    mt_set_gpio_mode(IsdbCtrlInfo.dtv_spi_cs, GPIO_MODE_02);
    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_cs, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_spi_cs, GPIO_PULL_DOWN);
    return;

}

void tunerbb_drv_hw_init(void)
{
    print_log(NULL, "[1seg][MTK] isdbt_hw_init\n");

    if(IsdbCtrlInfo.dtv_use_ant_sw) {
        mt_set_gpio_out(IsdbCtrlInfo.dtv_ant_sw_gipo, IsdbCtrlInfo.dtv_ant_sw_active_value);
    }

    mt_set_gpio_out(IsdbCtrlInfo.dtv_en, GPIO_OUT_ONE);
    ms_wait(30);

    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_irq_gpio, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_miso, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_mosi, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_clk, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_cs, GPIO_PULL_DISABLE);
    mt_set_gpio_mode(IsdbCtrlInfo.dtv_spi_cs, GPIO_MODE_02);
    mt_set_gpio_dir(IsdbCtrlInfo.dtv_spi_cs, GPIO_DIR_IN);

    return;
}

void tunerbb_drv_hw_deinit(void)
{
    print_log(0, "[1seg][MTK] isdbt_hw_deinit\n");

    fci_irq_disable();

    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_en, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_en, GPIO_PULL_DOWN);
    mt_set_gpio_dir(IsdbCtrlInfo.dtv_en, GPIO_DIR_OUT);
    mt_set_gpio_out(IsdbCtrlInfo.dtv_en, GPIO_OUT_ZERO);

    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_irq_gpio, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_irq_gpio, GPIO_PULL_DOWN);

    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_miso, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_spi_miso, GPIO_PULL_DOWN);

    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_mosi, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_spi_mosi, GPIO_PULL_DOWN);

    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_clk , GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_spi_clk , GPIO_PULL_DOWN);

    mt_set_gpio_mode(IsdbCtrlInfo.dtv_spi_cs, GPIO_MODE_00);
    mt_set_gpio_pull_enable(IsdbCtrlInfo.dtv_spi_cs, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(IsdbCtrlInfo.dtv_spi_cs, GPIO_PULL_DOWN);
    mt_set_gpio_dir(IsdbCtrlInfo.dtv_spi_cs, GPIO_DIR_OUT);
    mt_set_gpio_out(IsdbCtrlInfo.dtv_spi_cs, GPIO_OUT_ZERO);

    return;
}
#if 0
static int broadcast_fc8180_config_gpios(void)
{
    int rc = OK;
    int err_count = 0;

    IsdbCtrlInfo.dtv_en = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node,"fci_spi_fc8180,en-gpio",0);

    rc = gpio_request(IsdbCtrlInfo.dtv_en, "ISDBT_EN");
    if(rc < 0){
        printk("[dtv]broadcast ISDBT_EN gpio_request err\n");
        err_count++;
    }
    udelay(20);

    IsdbCtrlInfo.dtv_irq = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node,"fci_spi_fc8180,irq-gpio",0);

    rc = gpio_request(IsdbCtrlInfo.dtv_irq, "ISDBT_IRQ_INT");
    if(rc < 0){
        printk("[dtv]broadcast ISDBT_IRQ_INT gpio_request err\n");
        err_count++;
    }
    udelay(20);
    /* We set IRQ GPIO Output,0 for preven IRQ PIN from being current leakage */
    /* This IRQ Pin must be Input when DTV start or power on */

    gpio_direction_output(IsdbCtrlInfo.dtv_en, 0);
    gpio_direction_output(IsdbCtrlInfo.dtv_irq, 0);

    if(err_count > 0){
        printk("[dtv]config gpios err_count =(%d)", err_count);
    }
    return rc;
}
#endif
unsigned int isr_flag;

int fc8180_power_on(void)
{
	int i = 0;

#if 0
    int rc = 0;
#endif

	if (IsdbCtrlInfo.pwr_state != 1) {
#ifndef _NOT_USE_WAKE_LOCK_
		wake_lock(&IsdbCtrlInfo.wake_lock);
#endif
		while (driver_mode == ISDBT_DATAREAD) {
			ms_wait(100);
			print_log(NULL, "[FC8180] ISDBT_DATARREAD\
				mode i=(%d)\n", i);

			if (i++ > 5)
				break;
		}

#if defined(CONFIG_ARCH_MT6755)
        tunerbb_drv_hw_init();
#else
        gpio_set_value(IsdbCtrlInfo.dtv_en, 1);
        gpio_direction_input(IsdbCtrlInfo.dtv_irq);
#endif

        // For T1 time (>= 1msec)
        msleep(30);

        if(!IsdbCtrlInfo.dtv_use_xtal)
        {
#if 0
            if(IsdbCtrlInfo.clk != NULL) {
                rc = clk_prepare_enable(IsdbCtrlInfo.clk);
                if (rc) {
                    gpio_set_value(IsdbCtrlInfo.dtv_en, 0);
                    msleep(30);
                    dev_err(&IsdbCtrlInfo.spi_dev->dev, "could not enable clock\n");

                    return rc;
                }
            }
#else
            print_log(NULL, "[FC8180] RF clock enable\n");

            clk_buf_ctrl(CLK_BUF_CONN, true);
#endif
        }

        // For T4 time (>= 20msec)
        msleep(30);
        driver_mode = ISDBT_POWERON;
	} else {
        print_log(NULL, "[FC8180] ready on!! \n");
    }

	IsdbCtrlInfo.pwr_state = 1;

	return OK;
}

int fc8180_is_power_on()
{
	return (int)IsdbCtrlInfo.pwr_state;
}

int fc8180_stop(void)
{
    if (driver_mode == ISDBT_DATAREAD) {
            driver_mode = ISDBT_POWEROFF;
            ms_wait(200);
    }
    driver_mode = ISDBT_POWEROFF;

    return OK;
}

int fc8180_power_off(void)
{
	driver_mode = ISDBT_POWEROFF;

	if (IsdbCtrlInfo.pwr_state == 0)	{
		print_log(NULL, "[FC8180] power is immediately off\n");

		return OK;
	} else {
		print_log(NULL, "[FC8180] power_off\n");

        if(!IsdbCtrlInfo.dtv_use_xtal)
        {
#if 0
            if(IsdbCtrlInfo.clk != NULL) {
                clk_disable_unprepare(IsdbCtrlInfo.clk);
            }
#else
            print_log(NULL, "[FC8180] RF clock disable\n");

            clk_buf_ctrl(CLK_BUF_CONN, false);
#endif
        }

#if defined(CONFIG_ARCH_MT6755)
        tunerbb_drv_hw_deinit();
#else
	    gpio_direction_output(IsdbCtrlInfo.dtv_irq, 0);
        gpio_set_value(IsdbCtrlInfo.dtv_en, 0);
#endif
        msleep(10);
	}

#ifndef _NOT_USE_WAKE_LOCK_
	wake_unlock(&IsdbCtrlInfo.wake_lock);
#endif

	IsdbCtrlInfo.pwr_state = 0;

	return OK;
}

unsigned int fc8180_get_ts(void *buf, unsigned int size)
{
	s32 avail;
	ssize_t len, total_len = 0;

	if (fci_ringbuffer_empty(&RingBuffer))
		return 0;

	/*mutex_lock(&ringbuffer_lock);*/
	avail = fci_ringbuffer_avail(&RingBuffer);

	if (size >= avail)
		len = avail;
	else
		len = size - (size % 188);

	total_len = fci_ringbuffer_read_user(&RingBuffer, buf, len);
	/*mutex_unlock(&ringbuffer_lock);*/

	return total_len;
}

static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
    if ((driver_mode == ISDBT_POWEROFF) || (isdbt_isr_start)) {
        print_log(NULL, "fc8180 isdbt_irq : abnormal Interrupt\
            occurred fc8180 power off state.cnt : %d\n", irq_error_cnt);
        irq_error_cnt++;
        isdbt_isr_start = 0;
    } else {
        print_log(NULL, "fc8180 isdbt_irq : %d\n", isdbt_isr_sig);
        isdbt_isr_sig++;
        wake_up(&isdbt_isr_wait);
    }

#if defined(CONFIG_ARCH_MT6755)
    mt_eint_unmask(IsdbCtrlInfo.dtv_irq);
#endif

    return IRQ_HANDLED;
}

int data_callback(u32 hDevice, u8 *data, int len)
{
	int i;
    unsigned long ts_error_count = 0;
	totalTS += (len / 188);

	for (i = 0; i < len; i += 188) {
		if ((data[i+1] & 0x80) || data[i] != 0x47)
			ts_error_count++;
	}

    if (ts_error_count > 0) {
		totalErrTS += ts_error_count;
		print_log(NULL, "[FC8180] data_callback totalErrTS\
			: %d, len : %d\n", totalErrTS, len);
    }
	/*mutex_lock(&ringbuffer_lock);*/
	if (fci_ringbuffer_free(&RingBuffer) < len)
		FCI_RINGBUFFER_SKIP(&RingBuffer, len);

	fci_ringbuffer_write(&RingBuffer, data, len);
	/*wake_up_interruptible(&(RingBuffer.queue));*/

	/*mutex_unlock(&ringbuffer_lock);*/

	return 0;
}

static int isdbt_thread(void *hDevice)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	#ifdef MTK_FTRACE_TEST
	unsigned long previous_time = 0;
	unsigned int isdbt_ftrace_mode = 0;
	#endif

#if defined(CONFIG_ARCH_MT6755)
	struct sched_param param = {.sched_priority = 97};
	sched_setscheduler(current, SCHED_FIFO, &param);
#endif
	set_user_nice(current, -20);

	print_log(hInit, "isdbt_kthread enter\n");

	bbm_com_ts_callback_register((u32)hInit, data_callback);

	init_waitqueue_head(&isdbt_isr_wait);

	while (1) {
		wait_event_interruptible(isdbt_isr_wait
			, isdbt_isr_sig || kthread_should_stop());

		if (driver_mode == ISDBT_POWERON) {
                    driver_mode = ISDBT_DATAREAD;
                    print_log(hInit, "isdbt_kthread isr\n");
                    bbm_com_isr(NULL);
                    if (driver_mode == ISDBT_DATAREAD)
                        driver_mode = ISDBT_POWERON;
		}

		if (isdbt_isr_sig > 0) {
			#ifdef MTK_FTRACE_TEST
			isdbt_isr_sig--;
			if (isdbt_isr_sig > 0) {
				if (isdbt_ftrace_mode == 0) {
					if (isdbt_isr_sig > 1) {
						tracing_off();
						isdbt_ftrace_mode = 1;
					} else if (isdbt_isr_sig) {
						if ((previous_time) &&
							((unsigned long)jiffies
							- previous_time)
							< 300) {
							tracing_off();
							isdbt_ftrace_mode = 1;
						}
						previous_time
						= (unsigned long)jiffies;
					}
				}
				isdbt_isr_sig = 0;
			}
			#else
			isdbt_isr_sig--;
			if (isdbt_isr_sig > 0)
				isdbt_isr_sig = 0;
			#endif
		}

		if (kthread_should_stop())
			break;
	}

	bbm_com_ts_callback_deregister();

	print_log(NULL, "isdbt_kthread exit\n");

	return 0;
}

void fci_irq_disable(void)
{
    isdbt_isr_sig = 0;

#if defined(CONFIG_ARCH_MT6755)
    mt_eint_mask(IsdbCtrlInfo.dtv_irq);
#else
    print_log(NULL, "[FC8180] fci_irq_disable %d\n"
    	, IsdbCtrlInfo.spi_dev->irq);
 #endif
}

void fci_irq_enable(void)
{
    isdbt_isr_sig = 0;
    isdbt_isr_start = 1;

#if defined(CONFIG_ARCH_MT6755)
    mt_eint_unmask(IsdbCtrlInfo.dtv_irq);
#else
    print_log(NULL, "[FC8180] fci_irq_enable %d\n"
    , IsdbCtrlInfo.spi_dev->irq);
#endif
}

void broadcast_fci_ringbuffer_flush(void)
{
	fci_ringbuffer_flush(&RingBuffer);
}
/*
static int broadcast_spi_suspend(struct spi_device *spi
	, pm_message_t mesg)
{
    print_log(NULL, "Suspend\n");
    return 0;
}

static int broadcast_spi_resume(struct spi_device *spi)
{
    print_log(NULL, "Resume\n");
    return 0;
}
*/
static int broadcast_spi_remove(struct spi_device *spi)
{
    print_log(NULL, "broadcast_spi_remove \n");

#if defined(CONFIG_ARCH_MT6755)
    mt_set_gpio_mode(IsdbCtrlInfo.dtv_en, GPIO_MODE_00);
    mt_set_gpio_dir(IsdbCtrlInfo.dtv_en, GPIO_DIR_IN);
#else
    free_irq(IsdbCtrlInfo.dtv_irq, NULL);
#endif

    if (isdbt_kthread) {
        kthread_stop(isdbt_kthread);
        isdbt_kthread = NULL;
    }
    bbm_com_hostif_deselect(NULL);

    wake_lock_destroy(&IsdbCtrlInfo.wake_lock);

    return 0;
}

static int broadcast_spi_probe(struct spi_device *spi)
{
    int rc;
    //int ret;

    spi->mode 			= SPI_MODE_0;
    spi->bits_per_word	= 8;
    IsdbCtrlInfo.pdev = to_platform_device(&spi->dev);

    print_log(NULL, "[FC8180] broadcast_spi_probe start\n");

    rc = spi_setup(spi);
    if (rc) {
        print_log(NULL, "[FC8180] Spi setup error(%d)\n", rc);
        return rc;
    }

    IsdbCtrlInfo.spi_dev = spi;

    if(!IsdbCtrlInfo.dtv_use_xtal)
    {
#if 0
        IsdbCtrlInfo.clk = clk_get(&IsdbCtrlInfo.spi_dev->dev, "oneseg_xo");
        if (IS_ERR(IsdbCtrlInfo.clk)) {
            rc = PTR_ERR(IsdbCtrlInfo.clk);
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "could not get clock\n");

            return rc;
        }

        /* We enable/disable the clock only to assure it works */
        rc = clk_prepare_enable(IsdbCtrlInfo.clk);
        if (rc) {
            dev_err(&IsdbCtrlInfo.spi_dev->dev, "could not enable clock\n");

            return rc;
        }

        clk_disable_unprepare(IsdbCtrlInfo.clk);
#endif
    }

#ifndef _NOT_USE_WAKE_LOCK_
    wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
    dev_name(&spi->dev));
#endif
    fci_ringbuffer_init(&RingBuffer, &static_ringbuffer[0]
        , RING_BUFFER_SIZE);

    if (!isdbt_kthread) {
        print_log(NULL, "kthread run\n");
        isdbt_kthread = kthread_run(isdbt_thread, NULL, "isdbt_thread");
    }

#if defined(CONFIG_ARCH_MT6755)
    tunerbb_drv_hw_setting();

    if(IsdbCtrlInfo.dtv_irq < 0) {
       print_log(NULL, "[1seg] irq_of_parse_and_map IRQ can't parsing!.\n");
    } else {
        rc = request_irq(IsdbCtrlInfo.dtv_irq, isdbt_irq
            , IRQF_DISABLED | IRQF_TRIGGER_FALLING, "dtv", NULL);

        if (rc){
            print_log(NULL, "[1seg] dmb request irq fail : %d\n", rc);
        }
    }
#else
    /* Config GPIOs : IRQ PIN Output and 0 set at initial */
    broadcast_fc8180_config_gpios();

#ifdef FEATURE_DTV_USE_PINCTRL
    oneseg_pinctrl_init();
#endif

    rc = request_irq(spi->irq, isdbt_irq
        , IRQF_DISABLED | IRQF_TRIGGER_FALLING, FC8180_NAME, NULL);

    if (rc){
        print_log(NULL, "[1seg] dmb request irq fail : %d\n", rc);
    }
#endif

    rc = broadcast_dmb_drv_start(&device_fc8180);
    if (rc) {
        print_log(NULL, "[1seg] Failed to load Device (%d)\n", rc);
        rc = ERROR;
    }

    if (rc < 0)
        goto free_irq;

    fci_irq_disable(); /* Must disabled */

    print_log(NULL, "[FC8180] broadcast_spi_probe End.\n");

    return 0;

free_irq:
    broadcast_spi_remove(IsdbCtrlInfo.spi_dev);

    return rc;
}

#if defined(CONFIG_ARCH_MT6755)
struct spi_device_id spi_dtv_id_table = {"dtv_spi", 0};

static struct spi_driver broadcast_spi_driver = {
	.driver = {
		.name = "dtv_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = broadcast_spi_probe,
	.remove= __exit_p(broadcast_spi_remove),
	.id_table = &spi_dtv_id_table,
};

static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias="dtv_spi",
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
       //.max_speed_hz = 24*1000*1000,//24MHz
        .controller_data  = &mtk_spi_config,
    },
};
#else
//#ifdef CONFIG_OF //Open firmware must be defined for dts useage
static struct of_device_id fci_spi_fc8180_table[] = {
{
	.compatible = "fci,fc8180-spi",}, //Compatible node must match dts
	{ },
};

static struct spi_driver broadcast_spi_driver = {
	.driver = {
		.name = "fc8180-spi",
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = fci_spi_fc8180_table,
	},

	.probe = broadcast_spi_probe,
	.suspend = broadcast_spi_suspend,
	.resume	= broadcast_spi_resume,
	.remove	= broadcast_spi_remove,
};
#endif

static void async_broadcast_dmb_fc8180_drv_init(void *data, async_cookie_t cookie)
{
	int ret = 0;

	print_log(NULL, "[FC8180] call %s\n", __func__);
	if (module_init_flag)
		return;

    // Todo(add revision check rountine)

    if(broadcast_dmb_drv_check_module_init() != OK) {
        print_log(NULL, "[FC8180] broadcast_dmb_dr_check_module_init failed\n");
        return;
    }

#if defined(CONFIG_ARCH_MT6755)
    tunerbb_drv_load_hw_configure();    
    spi_board_devs[0].bus_num = IsdbCtrlInfo.dtv_spi_bus_num;
    spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
#endif

    ret = spi_register_driver(&broadcast_spi_driver);
    if (ret < 0)
        print_log(NULL, "[FC8180] SPI driver register failed\n");

    print_log(NULL, "[FC8180] end %s\n", __func__);
}

static int __init broadcast_dmb_fc8180_drv_init(void)
{
	print_log(NULL, "[FC8180] call %s\n", __func__);

	async_schedule(async_broadcast_dmb_fc8180_drv_init, NULL);

	return 0;
}

static void __exit broadcast_dmb_fc8180_drv_exit(void)
{
	spi_unregister_driver(&broadcast_spi_driver);
}

#if 0
static int broadcast_Isdb_i2c_probe(struct i2c_client *client
, const struct i2c_device_id *id)
{
	int rc = 0;

	int addr;

	print_log(NULL, "broadcast_Isdb_i2c_probe client:0x%X\n"
		, (unsigned int)client);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		print_log(NULL, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}
	/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
	addr = client->addr;
	pr_err("[1seg] i2c Slaveaddr [%x] \n", addr);

	IsdbCtrlInfo.pclient = client;
	/*i2c_set_clientdata(client, (void*)&IsdbCtrlInfo.pclient);*/

	tunerbb_drv_hw_setting();
#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
					dev_name(&client->dev));
#endif

	return rc;
}

static int broadcast_Isdb_i2c_remove(struct i2c_client *client)
{
	int rc = 0;

	print_log(NULL, "[%s]\n", __func__);
#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_destroy(&IsdbCtrlInfo.wake_lock);
#endif
	memset((unsigned char *)&IsdbCtrlInfo, 0x0
	, sizeof(struct broadcast_fc8180_ctrl_data));
	/*TcpalDeleteSemaphore(&fc8180DrvSem);*/
	return rc;
}

static int broadcast_Isdb_i2c_suspend(struct i2c_client *client
	, pm_message_t mesg)
{
	int rc = 0;
	print_log(NULL, "[%s]\n", __func__);
	return rc;
}

static int broadcast_Isdb_i2c_resume(struct i2c_client *client)
{
	int rc = 0;
	print_log(NULL, "[%s]\n", __func__);
	return rc;
}

static const struct i2c_device_id isdbt_fc8180_id[] = {
/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
	{"tcc3535_i2c",	0},
/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */
	{},
};

MODULE_DEVICE_TABLE(i2c, isdbt_fc8180_id);


/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
static struct of_device_id tcc3535_i2c_table[] = {
{ .compatible = "telechips,tcc3535-i2c",},
{ },
};
/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */

static struct i2c_driver broadcast_Isdb_driver = {
	.driver = {
		.name = "tcc3535_i2c",
		.owner = THIS_MODULE,
		.of_match_table = tcc3535_i2c_table,
	},
	.probe = broadcast_Isdb_i2c_probe,
	.remove	= __devexit_p(broadcast_Isdb_i2c_remove),
	.id_table = isdbt_fc8180_id,
	.suspend = broadcast_Isdb_i2c_suspend,
	.resume  = broadcast_Isdb_i2c_resume,
};

int __devinit broadcast_dmb_drv_init(void)
{
	int rc;
	print_log(NULL, "[%s]\n", __func__);
	rc = broadcast_dmb_drv_start();
	if (rc) {
		print_log(NULL, "failed to load\n");
		return rc;
	}
	print_log(NULL, "[%s add i2c driver]\n", __func__);
	rc = i2c_add_driver(&broadcast_Isdb_driver);
	print_log(NULL, "broadcast_add_driver rc = (%d)\n", rc);
	return rc;
}

static void __exit broadcast_dmb_drv_exit(void)
{
	i2c_del_driver(&broadcast_Isdb_driver);
}
#endif

module_init(broadcast_dmb_fc8180_drv_init);
module_exit(broadcast_dmb_fc8180_drv_exit);
MODULE_DESCRIPTION("broadcast_dmb_drv_init");
MODULE_LICENSE("GPL");
