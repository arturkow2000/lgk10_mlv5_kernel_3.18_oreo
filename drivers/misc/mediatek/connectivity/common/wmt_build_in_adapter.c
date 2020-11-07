/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
/*#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__*/
#include <linux/kernel.h>

#include "wmt_build_in_adapter.h"

/*device tree mode*/
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/irqreturn.h>
#include <linux/of_address.h>
#endif

#include <linux/interrupt.h>
// Debug RTC 32k clock
#include <linux/delay.h>
#include <mt_pmic_wrap.h>
#include <upmu_common.h>
#include <linux/regulator/consumer.h>
#include "mt_rtc_hw.h"
#include "mtk_rtc_hal.h"
#include "rtc.h"
#include "upmu_hw.h"
// Dump GPIO
#include "mt_gpio.h"
#include "mt_spm_sleep.h"

#define CONNADP_LOG_LOUD    4
#define CONNADP_LOG_DBG     3
#define CONNADP_LOG_INFO    2
#define CONNADP_LOG_WARN    1
#define CONNADP_LOG_ERR     0

/*******************************************************************************
 * Connsys adaptation layer logging utility
 ******************************************************************************/
static unsigned int gConnAdpDbgLvl = CONNADP_LOG_INFO;

#define CONNADP_LOUD_FUNC(fmt, arg...) \
do { \
	if (gConnAdpDbgLvl >= CONNADP_LOG_LOUD) \
		pr_info("[L]%s:"  fmt, __func__, ##arg); \
} while (0)
#define CONNADP_DBG_FUNC(fmt, arg...) \
do { \
	if (gConnAdpDbgLvl >= CONNADP_LOG_DBG) \
		pr_info("[D]%s:"  fmt, __func__, ##arg); \
} while (0)
#define CONNADP_INFO_FUNC(fmt, arg...)  \
do { \
	if (gConnAdpDbgLvl >= CONNADP_LOG_INFO) \
		pr_info("[I]%s:"  fmt, __func__, ##arg); \
} while (0)
#define CONNADP_WARN_FUNC(fmt, arg...) \
do { \
	if (gConnAdpDbgLvl >= CONNADP_LOG_WARN) \
		pr_info("[W]%s:"  fmt, __func__, ##arg); \
} while (0)
#define CONNADP_ERR_FUNC(fmt, arg...) \
do { \
	if (gConnAdpDbgLvl >= CONNADP_LOG_ERR) \
		pr_info("[E]%s(%d):"  fmt, __func__, __LINE__, ##arg); \
} while (0)


/*******************************************************************************
 * Bridging from platform -> wmt_drv.ko
 ******************************************************************************/
static struct wmt_platform_bridge bridge;

void wmt_export_platform_bridge_register(struct wmt_platform_bridge *cb)
{
	if (unlikely(!cb))
		return;
	bridge.thermal_query_cb = cb->thermal_query_cb;
#if MTK_WCN_CMB_FOR_SDIO_1V_AUTOK
	bridge.autok_cb = cb->autok_cb;
#endif
	CONNADP_INFO_FUNC("\n");
}
EXPORT_SYMBOL(wmt_export_platform_bridge_register);

void wmt_export_platform_bridge_unregister(void)
{
	memset(&bridge, 0, sizeof(struct wmt_platform_bridge));
	CONNADP_INFO_FUNC("\n");
}
EXPORT_SYMBOL(wmt_export_platform_bridge_unregister);

int mtk_wcn_cmb_stub_query_ctrl(void)
{
	CONNADP_DBG_FUNC("\n");
	if (unlikely(!bridge.thermal_query_cb)) {
		CONNADP_WARN_FUNC("Thermal query not registered\n");
		return -1;
	} else
		return bridge.thermal_query_cb();
}

#if MTK_WCN_CMB_FOR_SDIO_1V_AUTOK
int mtk_wcn_cmb_stub_1vautok_for_dvfs(void)
{
	CONNADP_DBG_FUNC("\n");
	if (unlikely(!bridge.autok_cb)) {
		CONNADP_WARN_FUNC("1v autok not registered\n");
		return -1;
	} else
		return bridge.autok_cb();
}
#endif


/*******************************************************************************
 * SDIO integration with platform MMC driver
 ******************************************************************************/
static void mtk_wcn_cmb_sdio_request_eirq(msdc_sdio_irq_handler_t irq_handler, void *data);
static void mtk_wcn_cmb_sdio_enable_eirq(void);
static void mtk_wcn_cmb_sdio_disable_eirq(void);
static void mtk_wcn_cmb_sdio_register_pm(pm_callback_t pm_cb, void *data);

struct sdio_ops mt_sdio_ops[4] = {
	{NULL, NULL, NULL, NULL},
	{NULL, NULL, NULL, NULL},
	{mtk_wcn_cmb_sdio_request_eirq, mtk_wcn_cmb_sdio_enable_eirq,
		mtk_wcn_cmb_sdio_disable_eirq, mtk_wcn_cmb_sdio_register_pm},
	{mtk_wcn_cmb_sdio_request_eirq, mtk_wcn_cmb_sdio_enable_eirq,
		mtk_wcn_cmb_sdio_disable_eirq, mtk_wcn_cmb_sdio_register_pm}
};

static atomic_t sdio_claim_irq_enable_flag;
static atomic_t irq_enable_flag;

static msdc_sdio_irq_handler_t mtk_wcn_cmb_sdio_eirq_handler;
static void *mtk_wcn_cmb_sdio_eirq_data;

unsigned int wifi_irq = 0xffffffff;
EXPORT_SYMBOL(wifi_irq);

pm_callback_t mtk_wcn_cmb_sdio_pm_cb;
EXPORT_SYMBOL(mtk_wcn_cmb_sdio_pm_cb);

void *mtk_wcn_cmb_sdio_pm_data;
EXPORT_SYMBOL(mtk_wcn_cmb_sdio_pm_data);

static int _mtk_wcn_sdio_irq_flag_set(int flag)
{
	if (flag != 0)
		atomic_set(&sdio_claim_irq_enable_flag, 1);
	else
		atomic_set(&sdio_claim_irq_enable_flag, 0);

	CONNADP_DBG_FUNC("sdio_claim_irq_enable_flag:%d\n", atomic_read(&sdio_claim_irq_enable_flag));

	return atomic_read(&sdio_claim_irq_enable_flag);
}

int wmt_export_mtk_wcn_sdio_irq_flag_set(int flag)
{
	return _mtk_wcn_sdio_irq_flag_set(flag);
}
EXPORT_SYMBOL(wmt_export_mtk_wcn_sdio_irq_flag_set);

static irqreturn_t mtk_wcn_cmb_sdio_eirq_handler_stub(int irq, void *data)
{
	if ((mtk_wcn_cmb_sdio_eirq_handler != NULL) && (atomic_read(&sdio_claim_irq_enable_flag) != 0))
		mtk_wcn_cmb_sdio_eirq_handler(mtk_wcn_cmb_sdio_eirq_data);
	return IRQ_HANDLED;
}

static void mtk_wcn_cmb_sdio_request_eirq(msdc_sdio_irq_handler_t irq_handler, void *data)
{
#ifdef CONFIG_OF
	struct device_node *node;
	int ret = -EINVAL;

	CONNADP_INFO_FUNC("enter\n");
	_mtk_wcn_sdio_irq_flag_set(0);
	atomic_set(&irq_enable_flag, 1);
	mtk_wcn_cmb_sdio_eirq_data = data;
	mtk_wcn_cmb_sdio_eirq_handler = irq_handler;

	node = (struct device_node *)of_find_compatible_node(NULL, NULL, "mediatek,connectivity-combo");
	if (node) {
		wifi_irq = irq_of_parse_and_map(node, 0);/* get wifi eint num */
		ret = request_irq(wifi_irq, mtk_wcn_cmb_sdio_eirq_handler_stub, IRQF_TRIGGER_LOW,
				"WIFI-eint", NULL);
		CONNADP_DBG_FUNC("WIFI EINT irq %d !!\n", wifi_irq);

		if (ret)
			CONNADP_WARN_FUNC("WIFI EINT IRQ LINE NOT AVAILABLE!!\n");
		else
			mtk_wcn_cmb_sdio_disable_eirq();/*not ,chip state is power off*/
	} else
		CONNADP_WARN_FUNC("can't find connectivity compatible node\n");

	CONNADP_INFO_FUNC("exit\n");
#else
	CONNADP_ERR_FUNC("not implemented\n");
#endif
}

static void mtk_wcn_cmb_sdio_register_pm(pm_callback_t pm_cb, void *data)
{
	CONNADP_DBG_FUNC("mtk_wcn_cmb_sdio_register_pm (0x%p, 0x%p)\n", pm_cb, data);
	/* register pm change callback */
	mtk_wcn_cmb_sdio_pm_cb = pm_cb;
	mtk_wcn_cmb_sdio_pm_data = data;
}

static void mtk_wcn_cmb_sdio_enable_eirq(void)
{
	if (atomic_read(&irq_enable_flag))
		CONNADP_DBG_FUNC("wifi eint has been enabled\n");
	else {
		atomic_set(&irq_enable_flag, 1);
		if (wifi_irq != 0xfffffff) {
			enable_irq(wifi_irq);
			CONNADP_DBG_FUNC(" enable WIFI EINT irq %d !!\n", wifi_irq);
		}
	}
}

static void mtk_wcn_cmb_sdio_disable_eirq(void)
{
	if (!atomic_read(&irq_enable_flag))
		CONNADP_DBG_FUNC("wifi eint has been disabled!\n");
	else {
		if (wifi_irq != 0xfffffff) {
			disable_irq_nosync(wifi_irq);
			CONNADP_DBG_FUNC("disable WIFI EINT irq %d !!\n", wifi_irq);
		}
		atomic_set(&irq_enable_flag, 0);
	}
}

void wmt_export_mtk_wcn_cmb_sdio_disable_eirq(void)
{
	mtk_wcn_cmb_sdio_disable_eirq();
}
EXPORT_SYMBOL(wmt_export_mtk_wcn_cmb_sdio_disable_eirq);

// RTC Debug 32k clock
static unsigned short RTC_Read(unsigned short addr)
{
	unsigned int rdata=0;
	pwrap_read((unsigned int)addr, &rdata);
	return (unsigned short)rdata;
}

static void RTC_Write(unsigned short addr, unsigned short data)
{
	pwrap_write((unsigned int)addr, (unsigned int)data);
}

bool rtc_check_cotsx(void)
{
	unsigned short test_out = 0;
//1.	TEST_CON0(0x208) = 0x11
//2.	DA_XMODE =  TEST_OUT(0x206)[0]
//3.	DA_DCXO32K_EN =  TEST_OUT(0x206)[1]
//jade CoTSX pin mux priority higher than frequency meter measurement

	RTC_Write(0x208, (RTC_Read(0x208)&~0x1F00) | 0x11<<8);
	test_out = RTC_Read(0x206);
//	print("[RTC] 0x%x, 0x%x, XMODE = %d, DCXO32K_EN = %d\n", RTC_Read(0x208), test_out, test_out & 0x1, (test_out >> 1) & 0x1);
	if((test_out & 0x3) == 0x3) {
//		print("[RTC] coTSX\n");
		return true;
	}
	return false;

}

unsigned short get_frequency_meter(unsigned short val, unsigned short measureSrc, unsigned short window_size)
{
	unsigned short ret = 0;
	// int i;
	// unsigned int begin = get_timer(0);
	unsigned int begin = 0;

	if(val!=0) {
		// unsigned short osc32con;
		// RTC_Write(RTC_BBPU, RTC_Read(RTC_BBPU) | RTC_BBPU_KEY | RTC_BBPU_RELOAD);
		// Write_trigger();
		// osc32con = RTC_Read(RTC_OSC32CON) & 0xFFE0;
		// rtc_xosc_write(osc32con | (val & 0x1f));
	}
	#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	CONNADP_ERR_FUNC("CONFIG_MTK_PMIC_CHIP_MT6353\n");
	#else
	CONNADP_ERR_FUNC("CONFIG_MTK_PMIC_CHIP_MT6351\n");
	#endif

	pmic_config_interface(RG_FQMTR_CLK_ON_SET, 1, RG_FQMTR_CLK_ON_MASK, RG_FQMTR_CLK_ON_SHIFT);
	#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_config_interface(FQMTR_CLK_CK_PDN_CLR, 1, FQMTR_CLK_CK_PDN_MASK, FQMTR_CLK_CK_PDN_SHIFT);
	pmic_config_interface(RG_FQMTR_32K_CLK_PDN_CLR, 1, RG_FQMTR_32K_CLK_PDN_MASK, RG_FQMTR_32K_CLK_PDN_SHIFT);
	#else
	pmic_config_interface(RG_FQMTR_32K_CLK_PDN_SET, 1, RG_FQMTR_32K_CLK_PDN_MASK, RG_FQMTR_32K_CLK_PDN_SHIFT);
	#endif

	RTC_Write(RG_FQMTR_RST, RTC_Read(RG_FQMTR_RST) | FQMTR_RST);			//FQMTR reset
	while( !(RTC_Read(RG_FQMTR_DATA)==0) && (FQMTR_BUSY&RTC_Read(RG_FQMTR_BUSY))==FQMTR_BUSY);
	RTC_Write(RG_FQMTR_RST, RTC_Read(RG_FQMTR_RST) & ~FQMTR_RST);			//FQMTR normal

	#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	if (!rtc_check_cotsx()) {
		pmic_config_interface(PMIC_TOP_CLKSQ_RTC_CLR_ADDR, 1, PMIC_RG_CLKSQ_RTC_EN_HW_MODE_MASK, PMIC_RG_CLKSQ_RTC_EN_HW_MODE_SHIFT);
		pmic_config_interface(PMIC_TOP_CLKSQ_RTC_SET_ADDR, 1, PMIC_RG_CLKSQ_RTC_EN_MASK, PMIC_RG_CLKSQ_RTC_EN_SHIFT);
	}
	pmic_config_interface(PMIC_FQMTR_DCXO26M_EN_ADDR, 1, PMIC_FQMTR_DCXO26M_EN_MASK, PMIC_FQMTR_DCXO26M_EN_SHIFT);
	#endif

	RTC_Write(RG_FQMTR_WINSET, window_size); //set freq. meter window value (0=1X32K(fix clock))
	#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	RTC_Write(RG_FQMTR_TCKSEL, FQMTR_DCXO26M_EN | FQMTR_EN | measureSrc); //enable freq. meter, set measure clock to 26Mhz
	#else
	RTC_Write(RG_FQMTR_TCKSEL, FQMTR_EN | measureSrc);
	#endif

	mdelay(1);
	while( (FQMTR_BUSY&RTC_Read(RG_FQMTR_BUSY))==FQMTR_BUSY )
	{
		if (begin > 1000)
		{
			CONNADP_ERR_FUNC("[RTC] get frequency time out\n");
			break;
		}
		begin++;
		mdelay(1);
	};		// FQMTR read until ready
	ret = RTC_Read(RG_FQMTR_DATA);				//read data should be closed to 26M/32k = 794

	#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	if (!rtc_check_cotsx()) {
		pmic_config_interface(PMIC_TOP_CLKSQ_RTC_SET_ADDR, 1, PMIC_RG_CLKSQ_RTC_EN_HW_MODE_MASK, PMIC_RG_CLKSQ_RTC_EN_HW_MODE_SHIFT);
		pmic_config_interface(PMIC_TOP_CLKSQ_RTC_CLR_ADDR, 1, PMIC_RG_CLKSQ_RTC_EN_MASK, PMIC_RG_CLKSQ_RTC_EN_SHIFT);
	}
	pmic_config_interface(PMIC_FQMTR_DCXO26M_EN_ADDR, 0, PMIC_FQMTR_DCXO26M_EN_MASK, PMIC_FQMTR_DCXO26M_EN_SHIFT);
	#endif

	RTC_Write(RG_FQMTR_TCKSEL, RTC_Read(RG_FQMTR_TCKSEL) & ~(FQMTR_EN)); //enable freq. meter, set measure clock to 26Mhz
	CONNADP_ERR_FUNC("[RTC] get_frequency_meter: input=0x%x, ouput=%d\n",val, ret);

	pmic_config_interface(RG_FQMTR_CLK_ON_CLR, 1, RG_FQMTR_CLK_ON_MASK, RG_FQMTR_CLK_ON_SHIFT);
	#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_config_interface(FQMTR_CLK_CK_PDN_SET, 1, FQMTR_CLK_CK_PDN_MASK, FQMTR_CLK_CK_PDN_SHIFT);
	pmic_config_interface(RG_FQMTR_32K_CLK_PDN_SET, 1, RG_FQMTR_32K_CLK_PDN_MASK, RG_FQMTR_32K_CLK_PDN_SHIFT);
	#else
	pmic_config_interface(RG_FQMTR_32K_CLK_PDN_CLR, 1, RG_FQMTR_32K_CLK_PDN_MASK, RG_FQMTR_32K_CLK_PDN_SHIFT);
	#endif

	return ret;
}

void rtc_measure_four_clock(unsigned short *result)
{
	unsigned short window_size;
	unsigned short regval;

	regval = RTC_Read(RG_FQMTR_CKSEL) & (~(RG_FQMTR_CKSEL_MASK << RG_FQMTR_CKSEL_SHIFT));
	RTC_Write(RG_FQMTR_CKSEL, regval | FQMTR_FIX_CLK_26M);					//select 26M as fixed clock
	window_size = 4;
	mdelay(1);
	result[0] = get_frequency_meter(0, FQMTR_FQM26M_CK, window_size); 		//select 26M as target clock

	regval = RTC_Read(RG_FQMTR_CKSEL) & (~(RG_FQMTR_CKSEL_MASK << RG_FQMTR_CKSEL_SHIFT));
	RTC_Write(RG_FQMTR_CKSEL, regval | FQMTR_FIX_CLK_XOSC_32K_DET);			//select XOSC_DET as fixed clock
	window_size = 4;
	mdelay(1);
	result[1] = get_frequency_meter(0, FQMTR_FQM26M_CK, window_size); 		//select 26M as target clock

	regval = RTC_Read(RG_FQMTR_CKSEL) & (~(RG_FQMTR_CKSEL_MASK << RG_FQMTR_CKSEL_SHIFT));
	RTC_Write(RG_FQMTR_CKSEL, regval | FQMTR_FIX_CLK_26M);					//select 26M as fixed clock
	window_size = 3970;  // (26M / 32K) * 5
	mdelay(1);
	result[2] = get_frequency_meter(0, FQMTR_XOSC32_CK, window_size);		//select xosc_32 as target clock
	result[2] = get_frequency_meter(0, FQMTR_DCXO_F32K_CK, window_size); 		//select DCXO_32 as target clock

	regval = RTC_Read(RG_FQMTR_CKSEL) & (~(RG_FQMTR_CKSEL_MASK << RG_FQMTR_CKSEL_SHIFT));
	RTC_Write(RG_FQMTR_CKSEL, regval | FQMTR_FIX_CLK_EOSC_32K);				//select EOSC_32 as fixed clock 
	window_size = 4;
	mdelay(1);
	result[3] = get_frequency_meter(0, FQMTR_FQM26M_CK, window_size);		//select 26M as target clock
}

void mtk_wcn_consys_hw_dump_pmic_status(void)
{
	unsigned char value;

	value = pmic_get_register_value(PMIC_DA_QI_VCN18_EN);//1 is enable
	CONNADP_ERR_FUNC("PMIC_DA_QI_VCN18_EN value = %d\n", value);

	value = pmic_get_register_value(PMIC_DA_QI_VCN18_STB);//1 is enable
	CONNADP_ERR_FUNC("PMIC_DA_QI_VCN18_STB value = %d\n", value);

	value = pmic_get_register_value(PMIC_DA_QI_VCN28_EN);//1 is enable
	CONNADP_ERR_FUNC("PMIC_DA_QI_VCN28_EN value = %d\n", value);

	value = pmic_get_register_value(PMIC_DA_QI_VCN28_STB);//1 is enable
	CONNADP_ERR_FUNC("PMIC_DA_QI_VCN28_STB value = %d\n", value);

	value = pmic_get_register_value(PMIC_RG_VCN28_VOSEL);//3 is 2.8V
	CONNADP_ERR_FUNC("PMIC_RG_VCN28_VOSEL value = %d\n", value);

	value = pmic_get_register_value(PMIC_DA_QI_VCN33_EN);//1 is enable
	CONNADP_ERR_FUNC("PMIC_DA_QI_VCN33_EN value = %d\n", value);

	value = pmic_get_register_value(PMIC_DA_QI_VCN33_STB);//1 is enable
	CONNADP_ERR_FUNC("PMIC_DA_QI_VCN33_STB value = %d\n", value);

	value = pmic_get_register_value(PMIC_RG_VCN33_VOSEL);//0 is 3.3V
	CONNADP_ERR_FUNC("PMIC_RG_VCN33_VOSEL value = %d\n", value);
}

int wmt_export_mtk_wcn_consys_check_hw_info(void)
{
	unsigned short result[4];
	CONNADP_ERR_FUNC("[RTC] mtk_wcn_consys_check_32k_clock\n");
	rtc_measure_four_clock(result);
	CONNADP_ERR_FUNC("[RTC] result: %d, %d, %d, %d\n", result[0], result[1], result[2], result[3]);
	// dump PMIC
	mtk_wcn_consys_hw_dump_pmic_status();
	// dump GPIO
	gpio_dump_regs();
	// Check connsys is SLEEP vis PCM_REG13_DATA & CONN_SRCCLKENA
	CONNADP_ERR_FUNC("CONN is SLEEP? %s\n", spm_is_conn_sleep() ? "yes" : "no");
	return 1;
}
EXPORT_SYMBOL(wmt_export_mtk_wcn_consys_check_hw_info);
