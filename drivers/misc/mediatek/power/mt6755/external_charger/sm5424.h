/*
* sm5424.c -- 3.5A Input, Switch Mode Charger device driver
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/******************************************************************************
* Register addresses
******************************************************************************/
#define SM5424_INT1		    0x00
#define SM5424_INT2		    0x01
#define SM5424_INT3		    0x02
#define SM5424_INTMASK1		0x03
#define SM5424_INTMASK2		0x04
#define SM5424_INTMASK3		0x05
#define SM5424_STATUS1		0x06
#define SM5424_STATUS2		0x07
#define SM5424_STATUS3		0x08
#define SM5424_CNTL		0x09
#define SM5424_VBUSCNTL		0x0A
#define SM5424_CHGCNTL1		0x0B
#define SM5424_CHGCNTL2		0x0C
#define SM5424_CHGCNTL3		0x0D
#define SM5424_CHGCNTL4		0x0E
#define SM5424_CHGCNTL5		0x0F
#define SM5424_CHGCNTL6		0x10
#define SM5424_DEVICEID		0x37


#define SM5424_REG_NUM 0x12

/**********************************************************
*
*   [MASK/SHIFT]
*
*********************************************************/
//INT1
#define SM5424_INT1_VBUSPOK		0x01
#define SM5424_INT1_VBUSUVLO	0x02
#define SM5424_INT1_VBUSOVP		0x04
#define SM5424_INT1_VBUSLIMIT	0x08
#define SM5424_INT1_BATOVP		0x10
#define SM5424_INT1_AICL		0x20
#define SM5424_INT1_NOBAT		0x40
#define SM5424_INT1_MASK        0x7F
#define SM5424_INT1_SHIFT       0
//INT2
#define SM5424_INT2_CHGON		0x01
#define SM5424_INT2_Q4FULLON	0x02
#define SM5424_INT2_TOPOFF		0x04
#define SM5424_INT2_DONE		0x08
#define SM5424_INT2_BOOSTPOK	0x10
#define SM5424_INT2_MASK        0x1F
#define SM5424_INT2_SHIFT       0
//INT3
#define SM5424_INT3_THEMREG		0x01
#define SM5424_INT3_THEMSHDN	0x02
#define SM5424_INT3_OTGFAIL		0x04
#define SM5424_INT3_DISLIMIT	0x08
#define SM5424_INT3_PRETMROFF	0x10
#define SM5424_INT3_FASTTMROFF	0x20
#define SM5424_INT3_HOT			0x40
#define SM5424_INT3_COLD		0x80
#define SM5424_INT3_MASK        0xFF
#define SM5424_INT3_FAIL        0x07
#define SM5424_INT3_SHIFT       0
//INTMSK1
#define SM5424_INTMSK1_VBUSPOKM		0x01
#define SM5424_INTMSK1_VBUSUVLOM	0x02
#define SM5424_INTMSK1_VBUSOVPM		0x04
#define SM5424_INTMSK1_VBUSLIMITM	0x08
#define SM5424_INTMSK1_BATOVPM		0x10
#define SM5424_INTMSK1_AICLM		0x20
#define SM5424_INTMSK1_NOBATM		0x40
#define SM5424_INTMSK1_MASK        	0x7F
#define SM5424_INTMSK1_SHIFT       	0
//INTMSK2
#define SM5424_INTMSK2_CHGONM		0x01
#define SM5424_INTMSK2_Q4FULLONM	0x02
#define SM5424_INTMSK2_TOPOFFM		0x04
#define SM5424_INTMSK2_DONEM		0x08
#define SM5424_INTMSK2_BOOSTPOKM	0x10
#define SM5424_INTMSK2_MASK        	0x1F
#define SM5424_INTMSK2_SHIFT       	0
//INTMSK3
#define SM5424_INTMSK3_THEMREGM		0x01
#define SM5424_INTMSK3_THEMSHDNM	0x02
#define SM5424_INTMSK3_OTGFAILM		0x04
#define SM5424_INTMSK3_DISLIMITM	0x08
#define SM5424_INTMSK3_PRETMROFFM	0x10
#define SM5424_INTMSK3_FASTTMROFFM	0x20
#define SM5424_INTMSK3_HOTM			0x40
#define SM5424_INTMSK3_COLDM		0x80
#define SM5424_INTMSK3_MASK        	0xFF
#define SM5424_INTMSK3_SHIFT       	0
//STATUS1
#define SM5424_STATUS1_VBUSPOK		0x01
#define SM5424_STATUS1_VBUSUVLO		0x02
#define SM5424_STATUS1_VBUSOVP		0x04
#define SM5424_STATUS1_VBUSLIMIT	0x08
#define SM5424_STATUS1_BATOVP		0x10
#define SM5424_STATUS1_AICL			0x20
#define SM5424_STATUS1_NOBAT		0x40
#define SM5424_STATUS1_MASK        	0x7F
#define SM5424_STATUS1_SHIFT       	0
//STATUS2
#define SM5424_STATUS2_CHGON		0x01
#define SM5424_STATUS2_Q4FULLON		0x02
#define SM5424_STATUS2_TOPOFF		0x04
#define SM5424_STATUS2_TOPOFF_SHIFT	2
#define SM5424_STATUS2_DONE		0x08
#define SM5424_STATUS2_DONE_SHIFT	3
#define SM5424_STATUS2_BOOSTPOK		0x10
#define SM5424_STATUS2_MASK        	0x1F
#define SM5424_STATUS2_SHIFT       	0
//STATUS3
#define SM5424_STATUS3_THEMREG		0x01
#define SM5424_STATUS3_THEMSHDN		0x02
#define SM5424_STATUS3_OTGFAIL		0x04
#define SM5424_STATUS3_DISLIMIT		0x08
#define SM5424_STATUS3_PRETMROFF	0x10
#define SM5424_STATUS3_FASTTMROFF	0x20
#define SM5424_STATUS3_HOT			0x40
#define SM5424_STATUS3_COLD			0x80
#define SM5424_STATUS3_MASK        	0xFF
#define SM5424_STATUS3_SHIFT       	0
//CNTL
#define SM5424_CNTL_nENZCS_MASK       0x1
#define SM5424_CNTL_nENZCS_SHIFT      6
#define SM5424_CNTL_ENI2CRESET_MASK   0x1
#define SM5424_CNTL_ENI2CRESET_SHIFT  5
#define SM5424_CNTL_SUSPEND_MASK      0x1
#define SM5424_CNTL_SUSPEND_SHIFT     4
#define SM5424_CNTL_RESET_MASK        0x1
#define SM5424_CNTL_RESET_SHIFT       3
#define SM5424_CNTL_ENBOOST_MASK      0x1
#define SM5424_CNTL_ENBOOST_SHIFT     2
#define SM5424_CNTL_AUTOSTOP_MASK     0x1
#define SM5424_CNTL_AUTOSTOP_SHIFT    1
#define SM5424_CNTL_CHGEN_MASK        0x1
#define SM5424_CNTL_CHGEN_SHIFT       0
//VBUSCNTL
#define SM5424_VBUSCNTL_VBUSLIMIT_MASK      0x7F
#define SM5424_VBUSCNTL_VBUSLIMIT_SHIFT     0
//CHGCNTL1
#define SM5424_CHGCNTL1_AICLTH_MASK         0x3
#define SM5424_CHGCNTL1_AICLTH_SHIFT        6
#define SM5424_CHGCNTL1_AICLEN_MASK         0x1
#define SM5424_CHGCNTL1_AICLEN_SHIFT        5
#define SM5424_CHGCNTL1_DISLIMIT_MASK       0x3
#define SM5424_CHGCNTL1_DISLIMIT_SHIFT      3
#define SM5424_CHGCNTL1_PRECHG_MASK         0x7
#define SM5424_CHGCNTL1_PRECHG_SHIFT        0
//CHGCNTL2
#define SM5424_CHGCNTL2_RECHG_MASK         0x1
#define SM5424_CHGCNTL2_RECHG_SHIFT        6
#define SM5424_CHGCNTL2_FASTCHG_MASK        0x3F
#define SM5424_CHGCNTL2_FASTCHG_SHIFT       0
//CHGCNTL3
#define SM5424_CHGCNTL3_BATREG_MASK         0x3F
#define SM5424_CHGCNTL3_BATREG_SHIFT        0
//CHGCNTL4
#define SM5424_CHGCNTL4_Q2LIM_MASK       	0x3
#define SM5424_CHGCNTL4_Q2LIM_SHIFT      	6
#define SM5424_CHGCNTL4_FREQSEL_MASK       	0x3
#define SM5424_CHGCNTL4_FREQSEL_SHIFT      	4
#define SM5424_CHGCNTL4_TOPOFF_MASK         0xF
#define SM5424_CHGCNTL4_TOPOFF_SHIFT        0
//CHGCNTL5
#define SM5424_CHGCNTL5_TOPOFFTIMER_MASK    0x3
#define SM5424_CHGCNTL5_TOPOFFTIMER_SHIFT   6
#define SM5424_CHGCNTL5_FASTTIMER_MASK      0x3
#define SM5424_CHGCNTL5_FASTTIMER_SHIFT     4
#define SM5424_CHGCNTL5_OTGCURRENT_MASK     0x3
#define SM5424_CHGCNTL5_OTGCURRENT_SHIFT    2
#define SM5424_CHGCNTL5_BST_IQ3LIMIT_MASK   0x3
#define SM5424_CHGCNTL5_BST_IQ3LIMIT_SHIFT  0
//CHGCNTL6
#define SM5424_CHGCNTL6_VOTG_MASK           0x3
#define SM5424_CHGCNTL6_VOTG_SHIFT          0
//DEVICEID
#define SM5424_DEVICEID_REVISIONID_MASK     0xF
#define SM5424_DEVICEID_REVISIONID_SHIFT    0

// FAST Charge current
#define FASTCHG_350mA	0x0
#define FASTCHG_400mA	0x1
#define FASTCHG_450mA	0x2
#define FASTCHG_500mA	0x3
#define FASTCHG_550mA	0x4
#define FASTCHG_600mA	0x5
#define FASTCHG_650mA	0x6
#define FASTCHG_700mA	0x7
#define FASTCHG_750mA	0x8
#define FASTCHG_800mA	0x9
#define FASTCHG_850mA	0xA
#define FASTCHG_900mA	0xB
#define FASTCHG_950mA	0xC
#define FASTCHG_1000mA	0xD
#define FASTCHG_1050mA	0xE
#define FASTCHG_1100mA	0xF
#define FASTCHG_1150mA	0x10
#define FASTCHG_1200mA	0x11
#define FASTCHG_1250mA	0x12
#define FASTCHG_1300mA	0x13
#define FASTCHG_1350mA	0x14
#define FASTCHG_1400mA	0x15
#define FASTCHG_1450mA	0x16
#define FASTCHG_1500mA	0x17
#define FASTCHG_1550mA	0x18
#define FASTCHG_1600mA	0x19
#define FASTCHG_1650mA	0x1A
#define FASTCHG_1700mA	0x1B
#define FASTCHG_1750mA	0x1C
#define FASTCHG_1800mA	0x1D
#define FASTCHG_1850mA	0x1E
#define FASTCHG_1900mA	0x1F
#define FASTCHG_1950mA	0x20
#define FASTCHG_2000mA	0x21
#define FASTCHG_2050mA	0x22
#define FASTCHG_2100mA	0x23
#define FASTCHG_2150mA	0x24
#define FASTCHG_2200mA	0x25
#define FASTCHG_2250mA	0x26
#define FASTCHG_2300mA	0x27
#define FASTCHG_2350mA	0x28
#define FASTCHG_2400mA	0x29
#define FASTCHG_2450mA	0x2A
#define FASTCHG_2500mA	0x2B
#define FASTCHG_2550mA	0x2C
#define FASTCHG_2600mA	0x2D
#define FASTCHG_2650mA	0x2E
#define FASTCHG_2700mA	0x2F
#define FASTCHG_2750mA	0x30
#define FASTCHG_2800mA  0x31
#define FASTCHG_2850mA  0x32
#define FASTCHG_2900mA  0x33
#define FASTCHG_2950mA  0x34
#define FASTCHG_3000mA  0x35
#define FASTCHG_3050mA  0x36
#define FASTCHG_3100mA  0x37
#define FASTCHG_3150mA  0x38
#define FASTCHG_3200mA  0x39
#define FASTCHG_3250mA  0x3A
#define FASTCHG_3300mA  0x3B
#define FASTCHG_3350mA  0x3C
#define FASTCHG_3400mA  0x3D
#define FASTCHG_3450mA  0x3E
#define FASTCHG_3500mA  0x3F

// Input current Limit
#define VBUSLIMIT_100mA    0x0
#define VBUSLIMIT_375mA    0x1
#define VBUSLIMIT_400mA    0x2
#define VBUSLIMIT_425mA    0x3
#define VBUSLIMIT_450mA    0x4
#define VBUSLIMIT_475mA    0x5
#define VBUSLIMIT_500mA    0x6
#define VBUSLIMIT_525mA    0x7
#define VBUSLIMIT_550mA    0x8
#define VBUSLIMIT_575mA    0x9
#define VBUSLIMIT_600mA    0xA
#define VBUSLIMIT_625mA    0xB
#define VBUSLIMIT_650mA    0xC
#define VBUSLIMIT_675mA    0xD
#define VBUSLIMIT_700mA    0xE
#define VBUSLIMIT_725mA    0xF
#define VBUSLIMIT_750mA    0x10
#define VBUSLIMIT_775mA    0x11
#define VBUSLIMIT_800mA    0x12
#define VBUSLIMIT_825mA    0x13
#define VBUSLIMIT_850mA    0x14
#define VBUSLIMIT_875mA    0x15
#define VBUSLIMIT_900mA    0x16
#define VBUSLIMIT_925mA    0x17
#define VBUSLIMIT_950mA    0x18
#define VBUSLIMIT_975mA    0x19
#define VBUSLIMIT_1000mA   0x1A
#define VBUSLIMIT_1025mA   0x1B
#define VBUSLIMIT_1050mA   0x1C
#define VBUSLIMIT_1075mA   0x1D
#define VBUSLIMIT_1100mA   0x1E
#define VBUSLIMIT_1125mA   0x1F
#define VBUSLIMIT_1150mA   0x20
#define VBUSLIMIT_1175mA   0x21
#define VBUSLIMIT_1200mA   0x22
#define VBUSLIMIT_1225mA   0x23
#define VBUSLIMIT_1250mA   0x24
#define VBUSLIMIT_1275mA   0x25
#define VBUSLIMIT_1300mA   0x26
#define VBUSLIMIT_1325mA   0x27
#define VBUSLIMIT_1350mA   0x28
#define VBUSLIMIT_1375mA   0x29
#define VBUSLIMIT_1400mA   0x2A
#define VBUSLIMIT_1425mA   0x2B
#define VBUSLIMIT_1450mA   0x2C
#define VBUSLIMIT_1475mA   0x2D
#define VBUSLIMIT_1500mA   0x2E
#define VBUSLIMIT_1525mA   0x2F
#define VBUSLIMIT_1550mA   0x30
#define VBUSLIMIT_1575mA   0x31
#define VBUSLIMIT_1600mA   0x32
#define VBUSLIMIT_1625mA   0x33
#define VBUSLIMIT_1650mA   0x34
#define VBUSLIMIT_1675mA   0x35
#define VBUSLIMIT_1700mA   0x36
#define VBUSLIMIT_1725mA   0x37
#define VBUSLIMIT_1750mA   0x38
#define VBUSLIMIT_1775mA   0x39
#define VBUSLIMIT_1800mA   0x3A
#define VBUSLIMIT_1825mA   0x3B
#define VBUSLIMIT_1850mA   0x3C
#define VBUSLIMIT_1875mA   0x3D
#define VBUSLIMIT_1900mA   0x3E
#define VBUSLIMIT_1925mA   0x3F
#define VBUSLIMIT_1950mA   0x40
#define VBUSLIMIT_1975mA   0x41
#define VBUSLIMIT_2000mA   0x42
#define VBUSLIMIT_2025mA   0x43
#define VBUSLIMIT_2050mA   0x44
#define VBUSLIMIT_2075mA   0x45
#define VBUSLIMIT_2100mA   0x46
#define VBUSLIMIT_2125mA   0x47
#define VBUSLIMIT_2150mA   0x48
#define VBUSLIMIT_2175mA   0x49
#define VBUSLIMIT_2200mA   0x4A
#define	VBUSLIMIT_2225mA   0x4B
#define	VBUSLIMIT_2250mA   0x4C
#define	VBUSLIMIT_2275mA   0x4D
#define	VBUSLIMIT_2300mA   0x4E
#define	VBUSLIMIT_2325mA   0x4F
#define	VBUSLIMIT_2350mA   0x50
#define	VBUSLIMIT_2375mA   0x51
#define	VBUSLIMIT_2400mA   0x52
#define	VBUSLIMIT_2425mA   0x53
#define	VBUSLIMIT_2450mA   0x54
#define	VBUSLIMIT_2475mA   0x55
#define	VBUSLIMIT_2500mA   0x56
#define	VBUSLIMIT_2525mA   0x57
#define	VBUSLIMIT_2550mA   0x58
#define	VBUSLIMIT_2575mA   0x59
#define	VBUSLIMIT_2600mA   0x5A
#define	VBUSLIMIT_2625mA   0x5B
#define	VBUSLIMIT_2650mA   0x5C
#define	VBUSLIMIT_2675mA   0x5D
#define	VBUSLIMIT_2700mA   0x5E
#define	VBUSLIMIT_2725mA   0x5F
#define	VBUSLIMIT_2750mA   0x60
#define	VBUSLIMIT_2775mA   0x61
#define	VBUSLIMIT_2800mA   0x62
#define	VBUSLIMIT_2825mA   0x63
#define	VBUSLIMIT_2850mA   0x64
#define	VBUSLIMIT_2875mA   0x65
#define	VBUSLIMIT_2900mA   0x66
#define	VBUSLIMIT_2925mA   0x67
#define	VBUSLIMIT_2950mA   0x68
#define	VBUSLIMIT_2975mA   0x69
#define	VBUSLIMIT_3000mA   0x6A
#define	VBUSLIMIT_3025mA   0x6B
#define	VBUSLIMIT_3050mA   0x6C
#define	VBUSLIMIT_3075mA   0x6D
#define	VBUSLIMIT_3100mA   0x6E
#define	VBUSLIMIT_3125mA   0x6F
#define	VBUSLIMIT_3150mA   0x70
#define	VBUSLIMIT_3175mA   0x71
#define	VBUSLIMIT_3200mA   0x72
#define	VBUSLIMIT_3225mA   0x73
#define	VBUSLIMIT_3250mA   0x74
#define	VBUSLIMIT_3275mA   0x75
#define	VBUSLIMIT_3300mA   0x76
#define	VBUSLIMIT_3325mA   0x77
#define	VBUSLIMIT_3350mA   0x78
#define	VBUSLIMIT_3375mA   0x79
#define	VBUSLIMIT_3400mA   0x7A
#define	VBUSLIMIT_3425mA   0x7B
#define	VBUSLIMIT_3450mA   0x7C
#define	VBUSLIMIT_3475mA   0x7D
#define	VBUSLIMIT_3500mA   0x7E
#define	VBUSLIMIT_3525mA   0x7F

// AICL TH
#define AICL_THRESHOLD_4_5_V        0x0
#define AICL_THRESHOLD_4_6_V        0x1
#define AICL_THRESHOLD_4_7_V        0x2
#define AICL_THRESHOLD_4_8_V        0x3
#define AICL_THRESHOLD_MASK        	0x3
#define AICL_THRESHOLD_SHIFT		6

// PRECHG
#define PRECHG_350mA         0x0
#define PRECHG_400mA         0x1
#define PRECHG_450mA         0x2
#define PRECHG_500mA         0x3
#define PRECHG_550mA         0x4
#define PRECHG_600mA         0x5
#define PRECHG_650mA         0x6
#define PRECHG_700mA         0x7
#define PRECHG_MASK       	 0x3
#define PRECHG_SHIFT       	 0

// Battery Regulation Voltage
#define	BATREG_3990mV	0x0
#define	BATREG_4000mV	0x1
#define	BATREG_4010mV	0x2
#define	BATREG_4020mV	0x3
#define	BATREG_4030mV	0x4
#define	BATREG_4040mV	0x5
#define	BATREG_4050mV	0x6
#define	BATREG_4060mV	0x7
#define	BATREG_4070mV	0x8
#define	BATREG_4080mV	0x9
#define	BATREG_4090mV	0xA
#define	BATREG_4100mV	0xB
#define	BATREG_4110mV	0xC
#define	BATREG_4120mV	0xD
#define	BATREG_4130mV	0xE
#define	BATREG_4140mV	0xF
#define	BATREG_4150mV	0x10
#define	BATREG_4160mV	0x11
#define	BATREG_4170mV	0x12
#define	BATREG_4180mV	0x13
#define	BATREG_4190mV	0x14
#define	BATREG_4200mV	0x15
#define	BATREG_4210mV	0x16
#define	BATREG_4220mV	0x17
#define	BATREG_4230mV	0x18
#define	BATREG_4240mV	0x19
#define	BATREG_4250mV	0x1A
#define	BATREG_4260mV	0x1B
#define	BATREG_4270mV	0x1C
#define	BATREG_4280mV	0x1D
#define	BATREG_4290mV	0x1E
#define	BATREG_4300mV	0x1F
#define	BATREG_4310mV	0x20
#define	BATREG_4320mV	0x21
#define	BATREG_4330mV	0x22
#define	BATREG_4340mV	0x23
#define	BATREG_4350mV	0x24
#define	BATREG_4360mV	0x25
#define	BATREG_4370mV	0x26
#define	BATREG_4380mV	0x27
#define	BATREG_4390mV	0x28
#define	BATREG_4400mV	0x29
#define	BATREG_4410mV	0x2A
#define	BATREG_4420mV	0x2B
#define	BATREG_4430mV	0x2C
#define	BATREG_4440mV	0x2D
#define	BATREG_4450mV	0x2E
#define	BATREG_4460mV	0x2F
#define	BATREG_4470mV	0x30
#define	BATREG_4480mV	0x31
#define	BATREG_4490mV	0x32
#define	BATREG_4500mV	0x33
#define	BATREG_4510mV	0x34
#define	BATREG_4520mV	0x35
#define	BATREG_4530mV	0x36
#define	BATREG_4540mV	0x37
#define	BATREG_4550mV	0x38
#define	BATREG_4560mV	0x39
#define	BATREG_4570mV	0x3A
#define	BATREG_4580mV	0x3B
#define	BATREG_4590mV	0x3C
#define	BATREG_4600mV	0x3D
#define	BATREG_4610mV	0x3E
#define	BATREG_4620mV	0x3F
#define BATREG_MASK     0x3F
#define BATREG_SHIF     0

// TOPOFF current
#define TOPOFF_100mA     0x0
#define TOPOFF_125mA     0x1
#define TOPOFF_150mA     0x2
#define TOPOFF_175mA     0x3
#define TOPOFF_200mA     0x4
#define TOPOFF_225mA     0x5
#define TOPOFF_250mA     0x6
#define TOPOFF_275mA     0x7
#define TOPOFF_300mA     0x8
#define TOPOFF_325mA     0x9
#define TOPOFF_350mA     0xA
#define TOPOFF_375mA     0xB
#define TOPOFF_400mA     0xC
#define TOPOFF_425mA     0xD
#define TOPOFF_450mA     0xE
#define TOPOFF_475mA     0xF
#define TOPOFF_MASK      0xF
#define TOPOFF_SHIFT     0

// FASTTIMER
#define FASTTIMER_DISABLED      0x0
#define FASTTIMER_4_0_HOUR      0x1
#define FASTTIMER_6_0_HOUR      0x2
#define FASTTIMER_8_0_HOUR      0x3
#define FASTTIMER_MASK       	0x3
#define FASTTIMER_SHIFT       	4

// OTGCURRENT
#define OTGCURRENT_500mA      	0x0
#define OTGCURRENT_700mA      	0x1
#define OTGCURRENT_900mA      	0x2
#define OTGCURRENT_1500mA      	0x3
#define OTGCURRENT_MASK       	0x3
#define OTGCURRENT_SHIFT       	2

// BSTIQ3LIMIT
#define BSTIQ3LIMIT_2P0A      	0x0
#define BSTIQ3LIMIT_2P8A      	0x1
#define BSTIQ3LIMIT_3P5A      	0x2
#define BSTIQ3LIMIT_4P0A      	0x3
#define BSTIQ3LIMIT_MASK       	0x3
#define BSTIQ3LIMIT_SHIFT       0

// VOTG voltage
#define VOTG_5_0_V      0x0
#define VOTG_5_1_V      0x1
#define VOTG_5_2_V      0x2
#define VOTG_MASK    	0x3
#define VOTG_SHIFT    	0

// TOPOFFTIMER
#define TOPOFFTIMER_10MIN       0x0
#define TOPOFFTIMER_20MIN       0x1
#define TOPOFFTIMER_30MIN       0x2
#define TOPOFFTIMER_45MIN       0x3
#define TOPOFFTIMER_MASK     	0x3
#define TOPOFFTIMER_SHIFT     	6

// Enable charger
#define CHARGE_EN 		1
#define CHARGE_DIS 		0

// Enable OTG
#define ENBOOST_EN 		1
#define ENBOOST_DIS 	0

// Enable SUSPEND
#define SUSPEND_EN 		1
#define SUSPEND_DIS 	0

// Enable AICL
#define AICL_EN 		1
#define AICL_DIS 		0

// Enable RECHARGE
#define RECHG_100mV		1
#define RECHG_50mV		0

// AUTOSTOP
#define AUTOSTOP_EN		1
#define AUTOSTOP_DIS	0


/******************************************************************************/
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
extern int detect_vbus_ok(void);
#endif
