/*
 * Copyright (C) 2011 LG Electronics Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LGE_BATTERY_ID_H__
#define __LGE_BATTERY_ID_H__
enum cell_type {
#ifdef CONFIG_LGE_PM_BATTERY_ID_EMBEDDED_ADC
	LGC,
	TOCAD,
	ATL
#else
	LGC_LLL,
	TCD_AAC
#endif
};

enum {
	BATT_ID_MISSING         = -1,
	BATT_ID_UNKNOWN         = 0,
	/* ADC */
	BATT_ID_LGC             = 5,
	BATT_ID_TOCAD           = 75,
	BATT_ID_ATL             = 200,
	BATT_ID_BYD             = 225,
	BATT_ID_LISHEN          = 230,
	/* Authentication IC */
	BATT_ID_DS2704_N        = 17,
	BATT_ID_DS2704_L        = 32,
	BATT_ID_DS2704_C        = 48,
	BATT_ID_ISL6296_N       = 73,
	BATT_ID_ISL6296_L       = 94,
	BATT_ID_ISL6296_C       = 105,
	BATT_ID_ISL6296A_N      = 110,
	BATT_ID_ISL6296A_L      = 115,
	BATT_ID_ISL6296A_C      = 120,
	BATT_ID_RA4301_VC0      = 130,
	BATT_ID_RA4301_VC1      = 147,
	BATT_ID_RA4301_VC2      = 162,
	BATT_ID_SW3800_VC0      = 187,
	BATT_ID_SW3800_VC1      = 204,
	BATT_ID_SW3800_VC2      = 219,
};

#define BATT_NOT_PRESENT 200

bool lge_battery_check(void);

#endif  /* __LGE_BATTERY_ID_H__ */

