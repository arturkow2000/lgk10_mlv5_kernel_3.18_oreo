/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 */

#ifndef __MT_RTC_MT6353_H__
#define __MT_RTC_MT6353_H__

#define RTC_BBPU_2SEC_CK_SEL			(1U << 7)
#define RTC_BBPU_2SEC_EN			(1U << 8)
#define RTC_BBPU_2SEC_MODE_SHIFT		9
#define RTC_BBPU_2SEC_MODE_MSK			(3U << RTC_BBPU_2SEC_MODE_SHIFT)
#define RTC_BBPU_2SEC_STAT_CLR			(1U << 11)
#define RTC_BBPU_2SEC_STAT			(1U << 12)
#define RTC_BBPU_2SEC_LPD_OPT			(1U << 13)

#define RTC_FQMTR_LOW_BASE	(794 - 2)
#define RTC_FQMTR_HIGH_BASE (794 + 2)

//**	frequency meter related		*//
#define RG_FQMTR_PDN			MT6353_CLK_CKPDN_CON1_SET
#define RG_FQMTR_PON			MT6353_CLK_CKPDN_CON1_CLR
#define RG_FQMTR_EN			MT6353_FQMTR_CON0
#define RG_FQMTR_CKSEL			MT6353_CLK_CKSEL_CON0
#define RG_FQMTR_CKSEL_SET		MT6353_CLK_CKSEL_CON0_SET
#define RG_FQMTR_CKSEL_CLR		MT6353_CLK_CKSEL_CON0_CLR
#define RG_FQMTR_RST			MT6353_TOP_RST_CON0
#define RG_FQMTR_TCKSEL			MT6353_FQMTR_CON0
#define RG_FQMTR_WINSET			MT6353_FQMTR_CON1
#define RG_FQMTR_BUSY			MT6353_FQMTR_CON0
#define RG_FQMTR_DCXO26M_EN		MT6353_FQMTR_CON0
#define RG_FQMTR_DATA			MT6353_FQMTR_CON2
#define RG_FQMTR_26M_CLK_ON		MT6353_TOP_CLKSQ_SET

#define RG_FQMTR_32K_CLK_PDN		MT6353_CLK_CKPDN_CON1
#define RG_FQMTR_32K_CLK_PDN_SET	PMIC_CLK_CKPDN_CON1_SET_ADDR
#define RG_FQMTR_32K_CLK_PDN_CLR	PMIC_CLK_CKPDN_CON1_CLR_ADDR
#define RG_FQMTR_32K_CLK_PDN_MASK	PMIC_CLK_FQMTR_32K_CK_PDN_MASK
#define RG_FQMTR_32K_CLK_PDN_SHIFT	PMIC_CLK_FQMTR_32K_CK_PDN_SHIFT

#define FQMTR_CLK_CK_PDN_SET		PMIC_CLK_CKPDN_CON1_SET_ADDR
#define FQMTR_CLK_CK_PDN_CLR		PMIC_CLK_CKPDN_CON1_CLR_ADDR
#define FQMTR_CLK_CK_PDN_MASK		PMIC_CLK_FQMTR_CK_PDN_MASK
#define FQMTR_CLK_CK_PDN_SHIFT		PMIC_CLK_FQMTR_CK_PDN_SHIFT

#define RG_FQMTR_CLK_ON_SET 		PMIC_TOP_CLKSQ_SET_ADDR
#define RG_FQMTR_CLK_ON_CLR 		PMIC_TOP_CLKSQ_CLR_ADDR
#define RG_FQMTR_CLK_ON_MASK		PMIC_RG_CLKSQ_EN_FQR_MASK
#define RG_FQMTR_CLK_ON_SHIFT		PMIC_RG_CLKSQ_EN_FQR_SHIFT


#define RG_FQMTR_CKSEL_MASK		PMIC_CLK_FQMTR_CK_CKSEL_MASK
#define RG_FQMTR_CKSEL_SHIFT		PMIC_CLK_FQMTR_CK_CKSEL_SHIFT
#define RG_FQMTR_TCKSEL_MASK		PMIC_FQMTR_TCKSEL_MASK
#define RG_FQMTR_TCKSEL_SHIFT		PMIC_FQMTR_TCKSEL_SHIFT
#define RG_FQMTR_26M_EN_MASK		PMIC_DA_CLKSQ_EN_VA28_MASK
#define RG_FQMTR_26M_EN_SHIFT		PMIC_DA_CLKSQ_EN_VA28_SHIFT

#define FQMTR_CLK_PDN			(1U << PMIC_CLK_FQMTR_32K_CK_PDN_SHIFT)
#define FQMTR_FIX_CLK_26M		(0 << RG_FQMTR_CKSEL_SHIFT)
#define FQMTR_FIX_CLK_XOSC_32K_DET	(1 << RG_FQMTR_CKSEL_SHIFT)
#define FQMTR_FIX_CLK_EOSC_32K		(2 << RG_FQMTR_CKSEL_SHIFT)
#define FQMTR_FIX_CLK_RTC_32K		(3 << RG_FQMTR_CKSEL_SHIFT)
#define FQMTR_FIX_CLK_SMPS_CK		(4 << RG_FQMTR_CKSEL_SHIFT)
#define FQMTR_FIX_CLK_TCK_SEC		(5 << RG_FQMTR_CKSEL_SHIFT)
#define FQMTR_FIX_CLK_PMU_75K		(6 << RG_FQMTR_CKSEL_SHIFT)
#define FQMTR_PDN			(1U << PMIC_CLK_FQMTR_CK_PDN_SHIFT)
#define FQMTR_RST			(1U << PMIC_RG_FQMTR_RST_SHIFT)
#define FQMTR_EN			(1U << PMIC_FQMTR_EN_SHIFT)
#define FQMTR_BUSY			(1U << PMIC_FQMTR_BUSY_SHIFT)
#define FQMTR_DCXO26M_EN		(1U << PMIC_FQMTR_DCXO26M_EN_SHIFT)
#define FQMTR_XOSC32_CK			0
#define	FQMTR_DCXO_F32K_CK 		1
#define	FQMTR_EOSC32_CK		 	2
#define	FQMTR_XOSC32_CK_DETECTON 	3
#define	FQMTR_FQM26M_CK 		4
#define	FQMTR_FQM32K_CK 		5
#define	FQMTR_TEST_CK 			6
#define FQMTR_WINSET    		0x0000

#define RG_75K_32K_SEL			PMIC_CLK_75K_32K_SEL_ADDR
#define RTC_75K_TO_32K			(1U << PMIC_CLK_75K_32K_SEL_SHIFT)
#define RTC_HW_STATUS			MT6353_TOPSTATUS
#define RTC_HW_DET_BYPASS		(8 << PMIC_RTC_XTAL_DET_RSV_SHIFT)
#define	RTC_HW_XOSC_MODE		(~(2 << PMIC_RTC_XTAL_DET_RSV_SHIFT))
#define	RTC_HW_DCXO_MODE		(2 << PMIC_RTC_XTAL_DET_RSV_SHIFT)

#define RTC_CLKSQ_EN_HW_MODE 	(1U << PMIC_RG_CLKSQ_RTC_EN_HW_MODE_SHIFT)
#define RTC_CLKSQ_EN			(1U << PMIC_RG_CLKSQ_RTC_EN_SHIFT)

/* we map HW YEA 0 (2000) to 1968 not 1970 because 2000 is the leap year */
#define RTC_MIN_YEAR            1968
#define RTC_NUM_YEARS           128
//#define RTC_MAX_YEAR          (RTC_MIN_YEAR + RTC_NUM_YEARS - 1)

#endif /* __MT_RTC_MT6353_H__ */
