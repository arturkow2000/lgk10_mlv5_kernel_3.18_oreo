/*****************************************************************************
*
* Filename:
* ---------
*   bq24262.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   bq24262 header file
*
* Author:
* -------
*
****************************************************************************/



#define bq24262_CON0      0x00 /* Status/Control Register */
#define bq24262_CON1      0x01 /* Control Register */
#define bq24262_CON2      0x02 /* Control/Battery Voltage Register */
#define bq24262_CON3      0x03 /* Vender/Part/Revision Register */
#define bq24262_CON4      0x04 /* Battery Termination/Fast Charge Current Register */
#define bq24262_CON5      0x05 /* Vin-dpm Voltage/MINSYS Status Register */
#define bq24262_CON6      0x06 /* Safety Timer/NTC Monitor Register */
#define bq24262_REG_NUM 7

/**********************************************************
  *
  *   [MASK/SHIFT]
  *
  *********************************************************/
/* CON0 : Status Control Register */
#define CON0_TMR_RST   0x01
#define CON0_TMR_RST_SHIFT  7

#define CON0_BOOST_MODE       0x01
#define CON0_BOOST_MODE_SHIFT      6

#define CON0_CHRG_STAT   0x03
#define CON0_CHRG_STAT_SHIFT  4

#define CON0_EN_SHIPMODE   0x01
#define CON0_EN_SHIPMODE_SHIFT  3

#define CON0_CHG_FAULT   0x07
#define CON0_CHG_FAULT_SHIFT  0

/* CON1 : Control Register */
#define CON1_REG_RST     0x01
#define CON1_REG_RST_SHIFT    7

#define CON1_IINLIM     0x0F
#define CON1_IINLIM_SHIFT    4

#define CON1_EN_STAT        0x11
#define CON1_EN_STAT_SHIFT       3

#define CON1_EN_CHG_TERM        0x21
#define CON1_EN_CHG_TERM_SHIFT       2

#define CON1_CHG_CONFIG        0x41 /* Active Low */
#define CON1_CHG_CONFIG_SHIFT       1

#define CON1_HZ_MODE   0x81
#define CON1_HZ_MODE_SHIFT  0

/* CON2 : Control/Battery Voltage Register */
#define CON2_VBREG   0x3F
#define CON2_VBREG_SHIFT   2

#define CON2_MOD_FREQ    0x03
#define CON2_MOD_FREQ_SHIFT   0

/* CON3 : Vender/Part/Revision Register */
#define CON3_VENDER   0x07
#define CON3_VENDER_SHIFT  5

#define CON3_PART_NUMBER           0x03
#define CON3_PART_NUMBER_SHIFT          3

/* CON4 : Battery Termination/Fast Charge Current Register */
#define CON4_ICHG     0x1F
#define CON4_ICHG_SHIFT    3

#define CON4_ITERM     0x07
#define CON4_ITERM_SHIFT    0

/* CON5 : Vin-dpm Voltage/MINSYS Status Register */
#define CON5_MINSYS_STATUS      0x01
#define CON5_MINSYS_STATUS_SHIFT     7

#define CON5_DPM_STATUS     0x01
#define CON5_DPM_STATUS_SHIFT    6

#define CON5_LOW_CHG      0x01
#define CON5_LOW_CHG_SHIFT     5

#define CON5_DPDM_EN           0x01
#define CON5_DPDM_EN_SHIFT          4

#define CON5_CD_STATUS           0x01
#define CON5_CD_STATUS_SHIFT          3

#define CON5_VINDPM           0x07
#define CON5_VINDPM_SHIFT          0

/* CON6 : Safety Timer/NTC Monitor Register */
#define CON6_XTMR_EN         0x01
#define CON6_XTMR_EN_SHIFT        7

#define CON6_TMR_BIT           0x03
#define CON6_TMR_BIT_SHIFT          5

#define CON6_BOOST_ILIM     0x01
#define CON6_BOOST_ILIM_SHIFT    4

#define CON6_TS_EN     0x01
#define CON6_TS_EN_SHIFT    3

#define CON6_TS_FAULT     0x03
#define CON6_TS_FAULT_SHIFT    1

#define CON6_VINDPM_OFF     0x01
#define CON6_VINDPM_OFF_SHIFT    0

/**********************************************************
  *
  *   [Extern Function]
  *
  *********************************************************/

extern int bq24262_dump_register(void);

#ifdef CONFIG_LGE_PM_OTG_BOOST_MODE
/*Boost mode for OTG*/
extern int detect_otg_mode(int);
#endif
