/***************************************************************************
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
 *    File	: 
 *    Author(s)   : 
 *    Description :
 *
 ***************************************************************************/
#ifndef __EPACK_H__
#define __EPACK_H__

/****************************************************************************
* Nested Include Files
****************************************************************************/

/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/

/**********************************************************
* Driver Capability
**********************************************************/

/****************************************************************************
* Type Definitions
****************************************************************************/
enum {
	EPACK_POWER_NA = 0,
	EPACK_POWER_OK,
	EPACK_POWER_NG,
	EPACK_POWER_UNKNOWN,
	EPACK_POWER_BLOCKED,
	NUM_EPACK_POWER,
};

enum {
	EPACK_USB_PATH_MAIN = 0,
	EPACK_USB_PATH_EPACK,
};

enum {
	EPACK_PWR_PATH_MAIN = 0,
	EPACK_PWR_PATH_EPACK,
};

enum {
	EPACK_FW_OK = 0,
	EPACK_FW_UPGRADE,
};

enum {
	EPACK_APROM_MODE =1,
	EPACK_LDROM_MODE,
};

/****************************************************************************
* Exported Variables
****************************************************************************/

/****************************************************************************
* Macros
****************************************************************************/
#if 1 /* branden.you_20170517 */
#else
#define LGTP_DEBUG 1

#define LGTP_TAG "[TOUCH]"

/* LGTP_MODULE : will be defined in each c-files */
#define TOUCH_ERR(fmt, args...)		pr_err(LGTP_TAG"[E]"LGTP_MODULE" %s() line=%d : "fmt, __func__, \
							__LINE__, ##args)
#define TOUCH_WARN(fmt, args...)	pr_err(LGTP_TAG"[W]"LGTP_MODULE" %s() line=%d : "fmt, __func__, \
							__LINE__, ##args)
#define TOUCH_LOG(fmt, args...)		pr_err(LGTP_TAG"[L]"LGTP_MODULE" " fmt, ##args)
#define TOUCH_DBG(fmt, args...)		pr_err(LGTP_TAG"[D]"LGTP_MODULE" " fmt, ##args)
#define TOUCH_FUNC(f)			pr_debug(LGTP_TAG"[F]"LGTP_MODULE" %s()\n", __func__)
#define TOUCH_FUNC_EXT(fmt, args...)	pr_err(LGTP_TAG"[F]"LGTP_MODULE" %s() : "fmt, __func__, ##args)

#define WRITE_BUFFER(_desc, _size, fmt, args...) (_size += snprintf(_desc + _size, sizeof(_desc) - _size, fmt, ##args))

#define WRITE_SYSBUF(_desc, _size, fmt, args...) (_size += snprintf(_desc + _size, PAGE_SIZE - _size, fmt, ##args))
#endif

/****************************************************************************
* Global Function Prototypes
****************************************************************************/
int epack_set_usb_path(int);
int epack_get_usb_path(void);
bool epack_is_umsdev_connected(void);
int epack_is_ok(void);
bool mtk_is_otg_main(void);
int mtk_otg_boost_is_on(void);

int epack_set_pwr_path(int path);
int epack_get_pwr_path(void);
int epack_is_otg_working(void);
int audio_get_epack_state(void);
void epack_call_update_handler(int charger);

#endif /* __EPACK_H__ */

/* End Of File */
