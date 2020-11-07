#define MODULE_TAG	"<atmf04>"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/unistd.h>
#include <linux/async.h>
#include <linux/in.h>
#include <linux/notifier.h>

#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/sort.h>
//#include <mt_gpio.h>
#include <mach/gpio_const.h>
#include <linux/of_irq.h>
#include <linux/firmware.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

#include <linux/pinctrl/consumer.h>
#include <linux/dma-mapping.h>
//#include <mach/board_lge.h>
#include <linux/irqchip/mt-eic.h>
#include "bs_log.h"

#include "atmf04.h"

#if defined (CONFIG_LGE_USE_CAP_SENSOR)
#define CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM
#define CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM
#define CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE
//#define CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY
#define CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS
#define CONFIG_LGE_CAP_SENSOR_REDUCE_BOOTING_TIME
#if !defined (CONFIG_SENSOR_ATMF04_2ND) && (defined (CONFIG_MACH_MSM8952_B3_ATT_US) || defined(CONFIG_MACH_MSM8952_B3_BELL_CA) || defined(CONFIG_MACH_MSM8952_B3_RGS_CA))
#define CONFIG_LGE_CAP_SENSOR_CURRENT_ISSUE // b3_att_us rev.B HW event - 2nd cap sensor makes sleep current increasing by always on.
#endif
#endif

#if defined CONFIG_SENSOR_ATMF04_2ND
#define CONFIG_LGE_CAP_SENSOR_BOOTING_TIME_IMPROVEMENT
#endif

#define ATMF04_DRV_NAME     "atmf04"

#define FAR_STATUS      1
#define NEAR_STATUS     0

#define CH1_FAR         2
#define CH2_FAR        (2<<2)
#define CH1_NEAR        1
#define CH2_NEAR       (1<<2)

/* I2C Suspend Check */
#define ATMF04_STATUS_RESUME        0
#define ATMF04_STATUS_SUSPEND       1
#define ATMF04_STATUS_QUEUE_WORK    2

/* Calibration Check */

#if defined(CONFIG_LGE_USE_CAP_SENSOR) // Device tuning value
#define ATMF04_CRCS_DUTY_LOW        300
#define ATMF04_CRCS_DUTY_HIGH       1650
#define ATMF04_CRCS_COUNT           1270
#define ATMF04_CSCR_RESULT          -1
#else // defined(CONFIG_LGE_USE_CAP_SENSOR)
#define ATMF04_CRCS_DUTY_LOW        375
#define ATMF04_CRCS_DUTY_HIGH       800
#define ATMF04_CRCS_COUNT           80
#define ATMF04_CSCR_RESULT          -2
#endif // defined(CONFIG_LGE_USE_CAP_SENSOR)

/* I2C Register */

#if defined(CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
#if defined(CONFIG_LGE_ATMF04_2CH)
#define I2C_ADDR_SSTVT_H            0x01
#define I2C_ADDR_SSTVT_L            0x02
#define I2C_ADDR_SSTVT_CH2_H        0x03
#define I2C_ADDR_SSTVT_CH2_L        0x04
#define I2C_ADDR_SAFE_DUTY_CHK      0x1C
#define I2C_ADDR_SYS_CTRL           0x1F
#define I2C_ADDR_SYS_STAT           0x20
#define I2C_ADDR_CR_DUTY_H          0x26
#define I2C_ADDR_CR_DUTY_L          0x27
#define I2C_ADDR_CS_DUTY_H          0x28
#define I2C_ADDR_CS_DUTY_L          0x29
#define I2C_ADDR_CS_DUTY_CH2_H      0x2A
#define I2C_ADDR_CS_DUTY_CH2_L      0x2B
#define I2C_ADDR_PER_H              0x22
#define I2C_ADDR_PER_L              0x23
#define I2C_ADDR_PER_CH2_H          0x24
#define I2C_ADDR_PER_CH2_L          0x25
#define I2C_ADDR_TCH_OUTPUT         0x21
#define I2C_ADDR_PGM_VER_MAIN       0x71
#define I2C_ADDR_PGM_VER_SUB        0x72
#else
#define I2C_ADDR_SSTVT_H            0x01
#define I2C_ADDR_SSTVT_L            0x02
#define I2C_ADDR_TCH_ONOFF_CNT      0x03
#define I2C_ADDR_DIG_FILTER         0x04
#define I2C_ADDR_TEMP_COEF_UP       0x06
#define I2C_ADDR_TEMP_COEF_DN       0x07
#define I2C_ADDR_SAFE_DUTY_CHK      0x0E
#define I2C_ADDR_SYS_CTRL           0x0F
#define I2C_ADDR_SYS_STAT           0x10
#define I2C_ADDR_CR_DUTY_H          0x11
#define I2C_ADDR_CR_DUTY_L          0x12
#define I2C_ADDR_CS_DUTY_H          0x13
#define I2C_ADDR_CS_DUTY_L          0x14
#define I2C_ADDR_PER_H              0x15
#define I2C_ADDR_PER_L              0x16
#define I2C_ADDR_TCH_OUTPUT         0x17
#define I2C_ADDR_PGM_VER_MAIN       0x18
#define I2C_ADDR_PGM_VER_SUB        0x19
#endif
#else
#define I2C_ADDR_SSTVT_H            0x01
#define I2C_ADDR_SSTVT_L            0x02
#define I2C_ADDR_TEMP_COEF_UP       0x03
#define I2C_ADDR_TEMP_COEF_DN       0x04
#define I2C_ADDR_DIG_FILTER         0x05
#define I2C_ADDR_TCH_ONOFF_CNT      0x06
#define I2C_ADDR_SAFE_DUTY_CHK      0x07
#define I2C_ADDR_SYS_CTRL           0x08
#define I2C_ADDR_SYS_STAT           0x09
#define I2C_ADDR_CR_DUTY_H          0x0A
#define I2C_ADDR_CR_DUTY_L          0x0B
#define I2C_ADDR_CS_DUTY_H          0x0C
#define I2C_ADDR_CS_DUTY_L          0x0D
#define I2C_ADDR_PER_H              0x0E
#define I2C_ADDR_PER_L              0x0F
#define I2C_ADDR_TCH_OUTPUT         0x10
#define I2C_ADDR_PGM_VER_MAIN       0x16
#define I2C_ADDR_PGM_VER_SUB        0x17
#endif

#if defined(CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
//Calibration Data Backup/Restore
#define I2C_ADDR_CMD_OPT 					0x7E
#define I2C_ADDR_COMMAND 					0x7F
#define I2C_ADDR_REQ_DATA					0x80
#define CMD_R_CD_DUTY						0x04		//Cal Data Duty Read
#define CMD_R_CD_REF						0x05		//Cal Data Ref Read
#define CMD_W_CD_DUTY						0x84		//Cal Data Duty Read
#define CMD_W_CD_REF						0x85		//Cal Data Ref Read
#define SZ_CALDATA_UNIT 					24
static int CalData[6][SZ_CALDATA_UNIT];
#endif

#define BIT_PERCENT_UNIT            8.192
#define MK_INT(X, Y)                (((int)X << 8)+(int)Y)

#define ENABLE_SENSOR_PINS          0
#define DISABLE_SENSOR_PINS         1

#define ON_SENSOR                   1	//Fixed HAL side
#define OFF_SENSOR                  2	//Fixed HAL side
#define PATH_CAPSENSOR_CAL  "/vendor/persist-lg/sensor/sar_controller_cal.dat"

static struct i2c_driver atmf04_driver;
static struct workqueue_struct *atmf04_workqueue;

static unsigned char fuse_data[SZ_PAGE_DATA];

#ifdef CONFIG_OF
enum sensor_dt_entry_status {
	DT_REQUIRED,
	DT_SUGGESTED,
	DT_OPTIONAL,
};

enum sensor_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
	DT_STRING,
};

struct sensor_dt_to_pdata_map {
	const char			*dt_name;
	void				*ptr_data;
	enum sensor_dt_entry_status status;
	enum sensor_dt_entry_type	type;
	int				default_val;
};
#endif
struct atmf04_hw local_cust;
struct atmf04_hw *hw =&local_cust;
	struct atmf04_hw {
		u32 irq_gpio;
		u32 chip_enable;
		u32 InputPinsNum;
		char* fw_name;
	};

static struct i2c_client
*atmf04_i2c_client; /* global i2c_client to support ioctl */

struct atmf04_platform_data {
	int (*init)(struct i2c_client *client);
	void (*exit)(struct i2c_client *client);
	struct device_node *irq_node;
	unsigned int irq_gpio;
	unsigned long chip_enable;
#if defined (CONFIG_LGE_CAP_SENSOR_CURRENT_ISSUE) || defined (CONFIG_LGE_CAP_SENSOR_BOOTING_TIME_IMPROVEMENT)
	unsigned long chip_enable2;
#endif
	int (*power_on)(struct i2c_client *client, bool on);
	u32 irq_gpio_flags;

	bool i2c_pull_up;

	struct regulator *vcc_ana;
	struct regulator *vcc_dig;
	struct regulator *vcc_i2c;

	u32 vdd_ana_supply_min;
	u32 vdd_ana_supply_max;
	u32 vdd_ana_load_ua;

	u32 input_pins_num; // not include ref sensor pin
	const char *fw_name;
};

struct atmf04_data {
	int (*get_nirq_low)(void);
	struct i2c_client *client;
	struct mutex update_lock;
	struct mutex enable_lock;
	struct delayed_work	dwork;		/* for PS interrupt */
	struct wake_lock ps_wlock;
	struct input_dev *input_dev_cap;
#ifdef CONFIG_OF
	struct atmf04_platform_data *platform_data;
	int irq;
#endif
	unsigned int enable;
	unsigned int sw_mode;
	atomic_t i2c_status;

	unsigned int cap_detection;
	int touch_out;
};

static bool on_sensor = false;
static bool check_allnear = false;
static bool cal_result = false; // debugging calibration paused

#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
static bool probe_end_flag = false;
#endif

static int get_bit(unsigned short x, int n);
static short get_abs(short x);
#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
static void check_firmware_ready(struct i2c_client *client);
static void check_init_touch_ready(struct i2c_client *client);
#endif
static void chg_mode(unsigned char flag, struct i2c_client *client)
{
	if(flag == ON) {
		i2c_smbus_write_byte_data(client, ADDR_EFLA_STS, 0x80);
		PINFO("change_mode : %d\n",i2c_smbus_read_byte_data(client, ADDR_EFLA_STS));
	}
	else {
		i2c_smbus_write_byte_data(client, ADDR_EFLA_STS, 0x00);
		PINFO("change_mode : %d\n",i2c_smbus_read_byte_data(client, ADDR_EFLA_STS));
	}
	mdelay(1);
}

#if defined(CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
static int Backup_CalData(struct i2c_client *client)
{
	int loop, dloop;
	int ret;

	for(loop = 0 ; loop < 2 ; loop++)
	{
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
		if (ret) {
			PINFO("atmf04: i2c_write_fail \n");
			return ret;
		}
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_R_CD_DUTY);
		if (ret) {
			PINFO("atmf04: i2c_write_fail \n");
			return ret;
		}

		mdelay(1); 		//1 ms Delay

		for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
			CalData[loop][dloop] = i2c_smbus_read_byte_data(client,I2C_ADDR_REQ_DATA + dloop);
	}

	for(loop = 0 ; loop < 3 ; loop++)
	{
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
		if (ret) {
			PINFO("atmf04: i2c_write_fail \n");
			return ret;
		}
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_R_CD_REF);
		if (ret) {
			PINFO("atmf04: i2c_write_fail \n");
			return ret;
		}

		mdelay(1); 		//1 ms Delay

		for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
			CalData[3+loop][dloop] = i2c_smbus_read_byte_data(client,I2C_ADDR_REQ_DATA + dloop);
	}
	if(CalData[0][0] == 0xFF || (CalData[0][0] == 0x00 && CalData[0][1] == 0x00))
	{
		PINFO("atmf04: Invalid cal data, Not back up this value.");
		return -1;
	}

	for(loop =0;loop<6;loop++)
	{
		for(dloop=0;dloop < SZ_CALDATA_UNIT ; dloop++)
			PINFO("atmf04: backup_caldata data[%d][%d] : %d \n",loop,dloop,CalData[loop][dloop]);
	}

	PINFO("atmf04 backup_cal success");
	return 0;
}

static int Write_CalData(struct i2c_client *client)
{
	int loop, dloop;
	int ret;

	for(loop = 0 ; loop < 3 ; loop++)
	{
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
		if (ret) {
			PINFO("atmf04: i2c_write_fail \n");
			return ret;
		}
		ret =i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_W_CD_DUTY);
		if (ret) {
			PINFO("atmf04: i2c_write_fail \n");
			return ret;
		}

		mdelay(1); 		//1 ms Delay

		for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
		{
			ret = i2c_smbus_write_byte_data(client,I2C_ADDR_REQ_DATA + dloop, CalData[loop][dloop]);
			if (ret) {
				PINFO("atmf04: i2c_write_fail \n");
				return ret;
			}

		}
	}

	for(loop = 0 ; loop < 3 ; loop++)
	{
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_CMD_OPT, loop);
		if (ret) {
			PINFO("atmf04: i2c_write_fail \n");
			return ret;
		}
		ret = i2c_smbus_write_byte_data(client,I2C_ADDR_COMMAND, CMD_W_CD_REF);
		if (ret) {
			PINFO("atmf04: i2c_write_fail \n");
			return ret;
		}

		mdelay(1); 		//1 ms Delay

		for(dloop = 0; dloop < SZ_CALDATA_UNIT ; dloop++)
		{
			ret = i2c_smbus_write_byte_data(client,I2C_ADDR_REQ_DATA + dloop, CalData[3+loop][dloop]);
			if (ret) {
				PINFO("atmf04: i2c_write_fail \n");
				return ret;
			}

		}
	}
	return 0;
}

static int RestoreProc_CalData(struct atmf04_data *data, struct i2c_client *client)
{
	int loop;
	int ret;
	//Power On
	gpio_set_value(data->platform_data->chip_enable, 0);
	mdelay(450);


	//Calibration data write
	ret = Write_CalData(client);
	if(ret)
		return ret;

	//Initial code write
	for(loop = 0 ; loop < CNT_INITCODE ; loop++) {
		ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
		if (ret) {
			PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
		}
		PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
	}

	check_firmware_ready(client);
	i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x80);
	check_firmware_ready(client);

	//Software Reset2
	i2c_smbus_write_byte_data(client,I2C_ADDR_SYS_CTRL, 0x02);
	PINFO("atmf04 restore_cal success");
	return 0;
}

#endif

#if defined(CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY)
static int Update_Sensitivity(struct atmf04_data *data, struct i2c_client *client)
{
	int ret = 0;
	char sstvt_h,sstvt_l;
#ifdef CONFIG_LGE_ATMF04_2CH
    char sstvt_ch2_h,sstvt_ch2_l;
#endif
	unsigned char loop;

	PINFO("Update_Sensitivity Check");

	sstvt_h = i2c_smbus_read_byte_data(client, I2C_ADDR_SSTVT_H);
	if (ret) {
		PINFO("i2c_read_fail[0x%x]",I2C_ADDR_SSTVT_H);
		goto i2c_fail;
	}

	sstvt_l = i2c_smbus_read_byte_data(client, I2C_ADDR_SSTVT_L);
	if (ret) {
		PINFO("i2c_read_fail[0x%x]",I2C_ADDR_SSTVT_L);
		goto i2c_fail;
	}

#ifdef CONFIG_LGE_ATMF04_2CH
    sstvt_ch2_h = i2c_smbus_read_byte_data(client, I2C_ADDR_SSTVT_CH2_H);
	if (ret) {
		PINFO("i2c_read_fail[0x%x]",I2C_ADDR_SSTVT_CH2_H);
		goto i2c_fail;
	}

	sstvt_ch2_l = i2c_smbus_read_byte_data(client, I2C_ADDR_SSTVT_CH2_L);
	if (ret) {
		PINFO("i2c_read_fail[0x%x]",I2C_ADDR_SSTVT_CH2_L);
		goto i2c_fail;
	}
#endif
	PINFO(" > sstvt_h : 0x%02x, sstvt_l : 0x%02x", sstvt_h, sstvt_l);

	if(sstvt_h != InitCodeVal[0] || sstvt_l != InitCodeVal[1]) {

		mdelay(500);
#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
		check_firmware_ready(client);
#endif

		PINFO("Update_Sensitivity Start");
		for(loop = 0 ; loop < CNT_INITCODE ; loop++) {
			ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
			if (ret) {
				PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
				return ret;
			}
			PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
		}

		//E-flash Data Save Command old version: 0x80, 2ch version : 0x0c
		//ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x80);
		ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x04);
		if (ret) {
			PINFO("i2c_write_fail[0x%x]",I2C_ADDR_SYS_CTRL);
			goto i2c_fail;
		}
		mdelay(50);		//50ms delay
		PINFO("Update_Sensitivity Finished");
	}

	return 0;

i2c_fail:
	return ret;
}
#endif

static unsigned char chk_done(unsigned int wait_cnt, struct i2c_client *client)
{
	unsigned int trycnt = 0;
	unsigned char rtn;

	do
	{
		if(++trycnt > wait_cnt) {
			PINFO("RTN_TIMEOUT");
			return RTN_TIMEOUT;
		}
		mdelay(1);
		rtn = i2c_smbus_read_byte_data(client, ADDR_EFLA_STS);
	}while((rtn & FLAG_DONE) != FLAG_DONE);

	return RTN_SUCC;
}

static unsigned char chk_done_erase(unsigned int wait_cnt, struct i2c_client *client)
{
	unsigned int trycnt = 0;
	unsigned char rtn;

	do
	{
		if(++trycnt > wait_cnt) return RTN_TIMEOUT;

		mdelay(5);
		rtn = i2c_smbus_read_byte_data(client, ADDR_EFLA_STS);
	}while((rtn & FLAG_DONE_ERASE) != FLAG_DONE_ERASE);

	return RTN_SUCC;
}

static unsigned char erase_eflash(struct i2c_client *client)
{
	i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_ERASE_ALL);

	if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT)
		return RTN_TIMEOUT; //timeout

	return RTN_SUCC;
}

static unsigned char write_eflash_page(unsigned char flag, unsigned char * page_addr, unsigned char * wdata, struct i2c_client *client)
{
	unsigned char paddr[2];

	if(flag != FLAG_FUSE)
	{
		paddr[0] = page_addr[1];
		paddr[1] = page_addr[0];
	}
	else	//Extra User Memory
	{
		paddr[0] = 0x00;
		paddr[1] = 0x00;
	}

	if(flag != FLAG_FUSE)
	{
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_L_WR);
	}
	else
	{
		//Erase
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_ERASE);
		if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;
		//Write
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_WR);
	}

	if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;

	return RTN_SUCC;
}

static unsigned char read_eflash_page(unsigned char flag, unsigned char * page_addr, unsigned char * rdata, struct i2c_client *client)
{
	unsigned char paddr[2];

	if(flag != FLAG_FUSE)
	{
		paddr[0] = page_addr[1];
		paddr[1] = page_addr[0];
	}
	else	//Extra User Memory
	{
		paddr[0] = 0x00;
		paddr[1] = 0x00;
	}

	if(flag != FLAG_FUSE)
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_RD);
	else
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_RD);

	if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;

	return RTN_SUCC;
}

static void onoff_sensor(struct atmf04_data *data, int onoff_mode)
{
	int nparse_mode;

	nparse_mode = onoff_mode;
	PINFO("onoff_sensor: nparse_mode [%d]",nparse_mode);

	if (nparse_mode == ENABLE_SENSOR_PINS) {
#if 1 // fixed bug to support cap sensor HAL
		input_report_abs(data->input_dev_cap, ABS_DISTANCE, 3);/* Initializing input event */
		input_sync(data->input_dev_cap);
#endif
		gpio_set_value(data->platform_data->chip_enable, 0);    /*chip_en pin - low : on, high : off*/
		mdelay(500);
		if (!on_sensor)
			enable_irq_wake(data->irq);
		on_sensor = true;
#if 0 // fixed bug to support cap sensor HAL
		if (gpio_get_value(data->platform_data->irq_gpio)) {
			input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* force FAR detection */
			input_sync(data->input_dev_cap);
		}
#endif
	}
	if (nparse_mode == DISABLE_SENSOR_PINS) {
		gpio_set_value(data->platform_data->chip_enable, 1);    /*chip_en pin - low : on, high : off*/
		if (on_sensor)
			disable_irq_wake(data->irq);
		on_sensor = false;
	}
}

static unsigned char load_firmware(struct atmf04_data *data, struct i2c_client *client, const char *name)
{
	const struct firmware *fw = NULL;
	unsigned char rtn;
	int ret, i, count = 0;
	int max_page;
	unsigned short main_version, sub_version;
	unsigned char page_addr[2];
	unsigned char fw_version, ic_fw_version, page_num;
	int version_addr;
#if defined (CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
	int restore = 0;
#endif
    unsigned char sys_status = 1;

	PINFO("Load Firmware Entered");

    gpio_set_value(data->platform_data->chip_enable, 1);
    mdelay(50);
	gpio_set_value(data->platform_data->chip_enable, 0);

	ret = request_firmware(&fw, name, &data->client->dev);
	if (ret) {
		PINFO("Unable to open bin [%s]  ret %d", name, ret);
        if (fw)
          release_firmware(fw);
		return 1;
	} else {
		PINFO("Open bin [%s] ret : %d ", name, ret);
	}

	max_page = (fw->size)/SZ_PAGE_DATA;
	version_addr = (fw->size)-SZ_PAGE_DATA;
	fw_version = MK_INT(fw->data[version_addr], fw->data[version_addr+1]);
	page_num = fw->data[version_addr+3];
	PINFO("###########fw version : %d.%d, fw_version : %d, page_num : %d###########", fw->data[version_addr], fw->data[version_addr+1], fw_version, page_num);

	mdelay(10);

	main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
	sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
	ic_fw_version = MK_INT(main_version, sub_version);
	PINFO("###########ic version : %d.%d, ic_fw_version : %d###########", main_version, sub_version, ic_fw_version);

    if( (fw->data[version_addr] != main_version) || (fw->data[version_addr+1] != sub_version)) {
        mdelay(500);
        check_firmware_ready(client);
        sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
		//mdelay(200);
#if defined(CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
		if ((ic_fw_version == 0) || ((sys_status & 0x06) == 0)) {
			restore = 0;
		}
		else {
			if (Backup_CalData(client) < 0){
				restore = 0;
			}
			else {
				restore = 1;
			}
		}
#endif
		/* IC Download Mode Change */
		chg_mode(ON, client);

		/* fuse data process */
		page_addr[0] = 0x00;
		page_addr[1] = 0x00;

		rtn = read_eflash_page(FLAG_FUSE, page_addr, fuse_data, client);
		if (rtn != RTN_SUCC) {
			PERR("read eflash page fail!");
			if (fw)
				release_firmware(fw);
			return rtn;		/* fuse read fail */
		}

		fuse_data[51] |= 0x80;

		rtn = write_eflash_page(FLAG_FUSE, page_addr, fuse_data, client);
		if (rtn != RTN_SUCC) {
			PERR("write eflash page fail!");
			if (fw)
				release_firmware(fw);
			return rtn;		/* fuse write fail */
		}

		/* firmware write process */
		rtn = erase_eflash(client);
		if(rtn != RTN_SUCC) {
			PINFO("earse fail\n");
			if (fw)
				release_firmware(fw);
			return rtn;		//earse fail
		}

		while(count < page_num) {
			//PINFO("%d\n",count);
			for(i=0; i < SZ_PAGE_DATA; i++) {
				i2c_smbus_write_byte_data(client, i, fw->data[i + (count*SZ_PAGE_DATA)]);
				//PINFO("%d : %x ",i + (count*SZ_PAGE_DATA),fw->data[i + (count*SZ_PAGE_DATA)]);
			}
			//PINFO("\n");
			page_addr[1] = (unsigned char)((count & 0xFF00) >> 8);
			page_addr[0] = (unsigned char)(count & 0x00FF);

			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
			i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, page_addr[1]);

			/*Eflash write command 0xFC -> 0x01 Write*/
			i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_L_WR);

			if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT)
			{
				if (fw)
					release_firmware(fw);
				return RTN_TIMEOUT;
			}

			count++;
		}
        chg_mode(OFF, client);
	    gpio_set_value(data->platform_data->chip_enable, 1);
	    mdelay(50);
	    gpio_set_value(data->platform_data->chip_enable, 0);
	}
	else {
		PINFO("Not update firmware. Firmware version is lower than ic.(Or same version)");
	}

#if defined(CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY)
	Update_Sensitivity(data, client);
#endif

	gpio_set_value(data->platform_data->chip_enable, 1);

#if defined(CONFIG_LGE_CAP_SENSOR_NEW_ALGORITHM)
	mdelay(10);
	if(restore)
	{
		ret = RestoreProc_CalData(data, client);
		if(ret)
			PINFO("restore fail ret: %d",ret);
	}
#endif
	PINFO("disable ok");

	release_firmware(fw);

	return 0;
}

static int write_initcode(struct i2c_client *client)
{
	struct atmf04_data *data = i2c_get_clientdata(client);
	unsigned char loop;
	int en_state;
	int ret = 0;

	PINFO("write_initcode entered[%d]",data->platform_data->irq_gpio);

	en_state = gpio_get_value(data->platform_data->chip_enable);

	if (en_state)
		gpio_set_value(data->platform_data->chip_enable, 0);
#ifdef CONFIG_MACH_MSM8916_E7IILTE_SPR_US
	mdelay(350);
#else
	mdelay(450);
#endif

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
	check_firmware_ready(client);
#endif

	for(loop = 0 ; loop < CNT_INITCODE ; loop++) {
		ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
		if (ret) {
			PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
			return ret;
		}
		PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
	}

#if defined(CONFIG_LGE_CAP_SENSOR_UPDATE_SENSITIVITY)
	for(loop = 0 ; loop < 2 ; loop++) {
		ret = i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop+8]); // write sensing percent value to autocal range
		if (ret) {
			PINFO("i2c_write_fail[0x%x]",InitCodeAddr[loop]);
			return ret;
		}
		PINFO("##FORCE SET[0x%x][0x%x]##", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
	}
#endif
	return 0;
}

static bool valid_multiple_input_pins(struct atmf04_data *data)
{
	if (data->platform_data->input_pins_num > 1)
		return true;

	return false;

}

static int write_calibration_data(struct atmf04_data *data, char *filename)
{
	int fd = 0;

	char file_result[2]; // debugging calibration paused

	PINFO("write_calibration_data Entered[%d]",data->platform_data->irq_gpio);
	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT, 0664);

	if (fd >= 0) {
#if 1 // debugging calibration paused
		if(cal_result) {
			file_result[0] = CAP_CAL_RESULT_PASS;
			file_result[1] = '\0';
			sys_write(fd, file_result, 1);
		} else {
			strncpy(file_result, CAP_CAL_RESULT_FAIL, strlen(CAP_CAL_RESULT_FAIL));
			file_result[1] = '\0';
			sys_write(fd, file_result, 1);
		}
		PINFO("%s: write [%s] to %s", __FUNCTION__, file_result, filename);
		sys_close(fd);
#else
		sys_write(fd,0,sizeof(int));
		sys_close(fd);
#endif
	} else {
		PINFO("%s: %s open failed [%d]......", __FUNCTION__, filename, fd);
	}

	//PINFO("sys open to save cal.dat");

	return 0;
}

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
static void check_firmware_ready(struct i2c_client *client)
{
	unsigned char sys_status = 1;
	int i;

	sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
	for(i = 0 ; i < 100 ; i++) {
		if (get_bit(sys_status, 0) == 1) {
			PINFO("%s: Firmware is busy now.....[%d] sys_status = [0x%x]", __FUNCTION__, i, sys_status);
			mdelay(10);
			sys_status = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
		} else {
			break;
		}
	}
	PINFO("%s: sys_status = [0x%x]", __FUNCTION__, sys_status);

	return;

}

static void check_init_touch_ready(struct i2c_client *client)
{
	unsigned char init_touch_md_check = 0;
	int i;

	init_touch_md_check = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
	for(i = 0 ; i < 50 ; i++) {
		if ((get_bit(init_touch_md_check, 2) == 0) && (get_bit(init_touch_md_check, 1) == 0)) {
			PINFO("%s: Firmware init touch is not yet ready.....[%d] sys_statue = [0x%x]", __FUNCTION__, i, init_touch_md_check);
			mdelay(10);
			init_touch_md_check = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
		} else {
			break;
		}
	}
	PINFO("%s: sys_status = [0x%x]", __FUNCTION__, init_touch_md_check);

	return;

}
#endif
static ssize_t atmf04_show_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmf04_data *data = i2c_get_clientdata(client);

	int loop;

	int ret = 0;
	client = data->client;


	for (loop = 0; loop < CNT_INITCODE; loop++) {
		PINFO("###### [0x%x][0x%x]###", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
	}
	return ret;
}

static ssize_t atmf04_show_regproxctrl0(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	PINFO("atmf04_show_regproxctrl0: %d\n",on_sensor);
	if(on_sensor==true)
		return sprintf(buf,"0x%02x\n",0x0C);
	return sprintf(buf,"0x%02x\n",0x00);
}

static ssize_t atmf04_store_reg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmf04_data *data = i2c_get_clientdata(client);
	unsigned char loop;
	client = data->client;

	for (loop = 0; loop < CNT_INITCODE; loop++) {
		i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
		PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], InitCodeVal[loop]);
	}
	i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);
	return count;
}

static ssize_t atmf04_show_proxstatus(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret ;
	struct atmf04_data *data = dev_get_drvdata(dev);
	ret = gpio_get_value(data->platform_data->irq_gpio);
	return sprintf(buf, "%d\n", ret);
}


static ssize_t atmf04_store_onoffsensor(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct atmf04_data *data = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if(val == ON_SENSOR) {
		onoff_sensor(data,ENABLE_SENSOR_PINS);
		PINFO("Store ON_SENSOR[%d]",data->platform_data->irq_gpio);
	}
	else if (val == OFF_SENSOR) {
		PINFO("Store OFF_SENSOR[%d]",data->platform_data->irq_gpio);
		onoff_sensor(data,DISABLE_SENSOR_PINS);
	}
	return count;
}

static ssize_t atmf04_show_docalibration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmf04_data *data = i2c_get_clientdata(client);
	unsigned char  safe_duty;
	client = data->client;

	/* check safe duty for validation of cal*/
	safe_duty = i2c_smbus_read_byte_data(client, I2C_ADDR_SAFE_DUTY_CHK);

	return sprintf(buf, "%d\n", get_bit(safe_duty, 7));
}

static ssize_t atmf04_store_docalibration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmf04_data *data = i2c_get_clientdata(client);
	int ret, init_state;
	unsigned char  safe_duty;
	client = data->client;

	PINFO("atmf04_store_docalibration[%d]",data->platform_data->irq_gpio);
	init_state = write_initcode(client);
	PINFO("write_initcode End[%d]",data->platform_data->irq_gpio);

#if 1 // debugging calibration paused
	if(init_state) {
		PINFO("%s: write_initcode result is failed.....\n", __FUNCTION__);
		cal_result = false;
		write_calibration_data(data, PATH_CAPSENSOR_CAL);
		return count;
	} else {
		PINFO("%s: write_initcode result is successful..... continue next step!!!\n", __FUNCTION__);
	}
#else
	if(init_state)
		return count;
#endif

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x0C);
	mutex_unlock(&data->update_lock);
	if(ret)
		PINFO("i2c_write_fail\n");

	mdelay(450);

	safe_duty = i2c_smbus_read_byte_data(client, I2C_ADDR_SAFE_DUTY_CHK);

	PINFO("[%u]safe_duty : %d", data->platform_data->irq_gpio, get_bit(safe_duty, 7));

#if 1 // debugging calibration paused
	if(ret || (get_bit(safe_duty, 7) != 1)) {
		// There is i2c failure or safe duty bit is not 1
		PINFO("%s: calibration result is failed.....\n", __FUNCTION__);
		cal_result = false;
	} else {
		PINFO("%s: calibration result is successful!!!!!\n", __FUNCTION__);
		cal_result = true;
	}
#endif

	write_calibration_data(data, PATH_CAPSENSOR_CAL);

	return count;
}

static ssize_t atmf04_store_regreset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmf04_data *data = i2c_get_clientdata(client);
	short tmp;
	short cs_per[2], cs_per_result;
	short cr_duty[2], cs_duty[2], cr_duty_val, cs_duty_val;
#ifdef CONFIG_LGE_ATMF04_2CH
	short cs_per_ch2[2], cs_per_result_ch2;
	short cs_duty_ch2[2], cs_duty_val_ch2;
#endif
	int ret;
    client = data->client;

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
	check_firmware_ready(client);
#endif

#if 1 // debugging calibration paused
	// Whether cal is pass or fail, make it cal_result true to check raw data/CS/CR/count in bypass mode of AAT
	cal_result = true;
	write_calibration_data(data, PATH_CAPSENSOR_CAL);
#endif

	ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x02);
	if(ret)
		PINFO("[%d]i2c_write_fail\n",data-> platform_data->irq_gpio);

	// CR Duty value
    cr_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H);
	cr_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L);
	cr_duty_val = MK_INT(cr_duty[1], cr_duty[0]);

	// Ch1 Per value
	cs_per[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_H);
	cs_per[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_L);
	tmp = MK_INT(cs_per[0], cs_per[1]);
	cs_per_result = tmp / 8;    // BIT_PERCENT_UNIT;

	// Ch1 CS Duty value
	cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_H);
	cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_L);
	cs_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

	PINFO("[%d] Result(ch1): %2d %6d %6d", data-> platform_data->irq_gpio, cs_per_result, cr_duty_val, cs_duty_val);

#ifdef CONFIG_LGE_ATMF04_2CH
	// Ch2 Per value
	cs_per_ch2[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_CH2_H);
	cs_per_ch2[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_CH2_L);
	tmp = MK_INT(cs_per_ch2[0], cs_per_ch2[1]);
	cs_per_result_ch2 = tmp / 8;    // BIT_PERCENT_UNIT;

	// Ch2 CS Duty value
	cs_duty_ch2[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_H);
	cs_duty_ch2[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_L);
	cs_duty_val_ch2 = MK_INT(cs_duty_ch2[1], cs_duty_ch2[0]);

	PINFO("[%d] Result(ch2): %2d %6d %6d", data-> platform_data->irq_gpio, cs_per_result_ch2, cr_duty_val, cs_duty_val_ch2);
#endif

	return count;
}

static int get_bit(unsigned short x, int n) {
	return (x & (1 << n)) >> n;
}

static short get_abs(short x) {
	return ((x >= 0) ? x : -x);
}

static ssize_t atmf04_show_regproxdata(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmf04_data *data = i2c_get_clientdata(client);
	short cs_per[2], cs_per_result;
	short cr_duty[2], cs_duty[2], cr_duty_val, cs_duty_val;
#ifdef CONFIG_LGE_ATMF04_2CH
	short cs_per_ch2[2], cs_per_result_ch2;
	short cs_duty_ch2[2], cs_duty_val_ch2;
	short cap_value_ch2;
#endif
	short tmp, cap_value;
	int nlength = 0;
	char buf_regproxdata[256] = "";
	char buf_line[64] = "";
	unsigned char init_touch_md;
#if defined(CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM)	 /*auto calibration FW*/
	int check_mode;
#endif
    client = data->client;
	memset(buf_line, 0, sizeof(buf_line));
	memset(buf_regproxdata, 0, sizeof(buf_regproxdata));

	init_touch_md = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);

	cs_per[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_H);
	cs_per[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_L);
	tmp = MK_INT(cs_per[0], cs_per[1]);
	cs_per_result = tmp / 8;    // BIT_PERCENT_UNIT;

	cr_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H);
	cr_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L);
	cr_duty_val = MK_INT(cr_duty[1], cr_duty[0]);

	cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_H);
	cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_L);
	cs_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

	cap_value = (int)cs_duty_val * (int)cs_per_result;

#ifdef CONFIG_LGE_ATMF04_2CH
	// Ch2 Per value
	cs_per_ch2[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_CH2_H);
	cs_per_ch2[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_CH2_L);
	tmp = MK_INT(cs_per_ch2[0], cs_per_ch2[1]);
	cs_per_result_ch2 = tmp / 8;    // BIT_PERCENT_UNIT;

	// Ch2 CS Duty value
	cs_duty_ch2[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_H);
	cs_duty_ch2[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_CH2_L);
	cs_duty_val_ch2 = MK_INT(cs_duty_ch2[1], cs_duty_ch2[0]);

	cap_value_ch2 = (int)cs_duty_val_ch2 * (int)cs_per_result_ch2;

#if defined(CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM)	/*auto calibration FW*/
	check_mode = get_bit(init_touch_md, 2);

	PINFO("Enable CH2 check_mode[%d]", check_mode);

	/* Normal Mode */
	if (check_mode) {
		PINFO("Enable CH2 check_mode[%d]", check_mode);
		sprintf(buf_line, "[R]%6d %6d %6d %6d %6d %6d %6d %6d\n",
		                    get_bit(init_touch_md,2), cs_per_result, cs_per_result_ch2, cr_duty_val, cs_duty_val, cs_duty_val_ch2, cap_value, cap_value_ch2);
	}
	/* Init Touch Mode */
	else {
#endif
    PINFO("Enable CH2 check_mode[%d]", check_mode);
		sprintf(buf_line, "[R]%6d %6d %6d %6d %6d %6d %6d %6d\n",
		                   get_bit(init_touch_md,1), cs_per_result, cs_per_result_ch2, cr_duty_val, cs_duty_val, cs_duty_val_ch2, cap_value, cap_value_ch2);
    }
#else
	// printk("H: %x L:%x H:%x L:%x\n",cr_duty[1] ,cr_duty[0], cs_duty[1], cs_duty[0]);
#if defined(CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM)	/*auto calibration FW*/
	check_mode = get_bit(init_touch_md, 2);

	if (check_mode)		/* Normal Mode */
		sprintf(buf_line, "[R]%6d %6d %6d %6d %6d\n", get_bit(init_touch_md,2), cs_per_result, cr_duty_val, cs_duty_val, cap_value);
	else		/* Init Touch Mode */
#endif
    	sprintf(buf_line, "[R]%6d %6d %6d %6d %6d\n", get_bit(init_touch_md,1), cs_per_result, cr_duty_val, cs_duty_val, cap_value);
#endif
	nlength = strlen(buf_regproxdata);
	strcpy(&buf_regproxdata[nlength], buf_line);

	return sprintf(buf, "%s", buf_regproxdata);
}

static ssize_t atmf04_store_checkallnear(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val == 0)
		check_allnear = false;
	else if (val == 1)
		check_allnear = true;

	printk("atmf04_store_checkallnear %d\n",check_allnear);
	return count;
}

static ssize_t atmf04_show_count_inputpins(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count_inputpins = 0;

	struct atmf04_data *data = dev_get_drvdata(dev);

	count_inputpins = data->platform_data->input_pins_num;
	if (count_inputpins > 1) {
		if (valid_multiple_input_pins(data) == false)
			count_inputpins = 1;
	}
	return sprintf(buf, "%d\n", count_inputpins);
}

static ssize_t atmf04_store_firmware(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	const char *fw_name = NULL;
	struct atmf04_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	client = data->client;

	fw_name = data->platform_data->fw_name;
	load_firmware(data, client, fw_name);
	return count;
}

static ssize_t atmf04_show_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned short main_version, sub_version;
	unsigned char ic_fw_version;
	struct i2c_client *client = to_i2c_client(dev);
	struct atmf04_data *data = dev_get_drvdata(dev);
	char buf_line[64] = "";
	int nlength = 0;
	char buf_regproxdata[256] = "";
	client = data->client;

	memset(buf_line, 0, sizeof(buf_line));
	onoff_sensor(data,ENABLE_SENSOR_PINS);

	mdelay(200);

	main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
	sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
	ic_fw_version = MK_INT(main_version, sub_version);
	PINFO("###########ic version : %d.%d, ic_fw_version : %d###########", main_version, sub_version, ic_fw_version);

	sprintf(buf_line, "%d.%d\n",main_version, sub_version);
	nlength = strlen(buf_regproxdata);
	strcpy(&buf_regproxdata[nlength], buf_line);

	return sprintf(buf,"%s", buf_regproxdata);
}

static ssize_t atmf04_show_check_far(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmf04_data *data = i2c_get_clientdata(client);
	unsigned char init_touch_md_check;
	short tmp, cs_per[2], cs_per_result, crcs_count;
	short cr_duty[2], cs_duty[2], cr_duty_val, cs_duty_val;
	int bit_mask = 1; //bit for reading of I2C_ADDR_SYS_STAT
	client = data->client;

	mutex_lock(&data->update_lock);

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
	check_init_touch_ready(client);
#endif

	cs_per[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_H);
	cs_per[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_L);
	tmp = MK_INT(cs_per[0], cs_per[1]);
	cs_per_result = tmp / 8;    // BIT_PERCENT_UNIT;

	cr_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H);
	cr_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L);
	cr_duty_val = MK_INT(cr_duty[1], cr_duty[0]);

	cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_H);
	cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_L);
	cs_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

	crcs_count = get_abs(cr_duty_val - cs_duty_val);

	init_touch_md_check = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
#if defined(CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM) /*auto calibration FW*/
	if(get_bit(init_touch_md_check, 2))
		bit_mask = 2;  /* Normal Mode */
	else
		bit_mask = 1;  /* Init Touch Mode */
#endif

	if(gpio_get_value(data->platform_data->irq_gpio) == 1) {
		if ((get_bit(init_touch_md_check, bit_mask) != 1) || (cs_duty_val < ATMF04_CRCS_DUTY_LOW)\
				||(cr_duty_val <ATMF04_CRCS_DUTY_LOW) || (cs_duty_val > ATMF04_CRCS_DUTY_HIGH)\
				|| (cr_duty_val > ATMF04_CRCS_DUTY_HIGH) || (cs_per_result < ATMF04_CSCR_RESULT) || (crcs_count > ATMF04_CRCS_COUNT)) {
			PINFO("[fail] 1.cal_check : %d, cr_value : %d, cs_value : %d, status : %d Count : %d",get_bit(init_touch_md_check, bit_mask), cr_duty_val, cs_duty_val, cs_per_result, crcs_count);
			mutex_unlock(&data->update_lock);
			return sprintf(buf,"%d",0);
		}
		else {
			PINFO("[pass] 2.cal_check : %d, cr_value : %d, cs_value : %d, status : %d Count : %d",get_bit(init_touch_md_check, bit_mask), cr_duty_val, cs_duty_val, cs_per_result, crcs_count);
			mutex_unlock(&data->update_lock);
			return sprintf(buf,"%d",1);
		}
	}
	else {
		PINFO("[fail] 3.cal_check : %d, cr_value : %d, cs_value : %d, status : %d Count : %d",get_bit(init_touch_md_check, bit_mask), cr_duty_val, cs_duty_val, cs_per_result, crcs_count);
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",0);
	}
}

static ssize_t atmf04_show_check_mid(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmf04_data *data = i2c_get_clientdata(client);
	unsigned char init_touch_md_check;
	short tmp, cs_per[2], cs_per_result, crcs_count;
	short cr_duty[2], cs_duty[2], cr_duty_val, cs_duty_val;
	int bit_mask = 1; //bit for reading of I2C_ADDR_SYS_STAT
	client = data->client;

	mutex_lock(&data->update_lock);

#if defined(CONFIG_LGE_CAP_SENSOR_CHECK_FIRMWARE_STATUS)
	check_init_touch_ready(client);
#endif

	cs_per[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_H);
	cs_per[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_L);
	tmp = MK_INT(cs_per[0], cs_per[1]);
	cs_per_result = tmp / 8;    // BIT_PERCENT_UNIT;

	cr_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H);
	cr_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L);
	cr_duty_val = MK_INT(cr_duty[1], cr_duty[0]);

	cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_H);
	cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_L);
	cs_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

	crcs_count = get_abs(cr_duty_val - cs_duty_val);

	init_touch_md_check = i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT);
#if defined(CONFIG_LGE_CAP_SENSOR_AUTO_CAL_ALGORITHM) /*auto calibration FW*/
	if(get_bit(init_touch_md_check, 2))
		bit_mask = 2;  /* Normal Mode */
	else
		bit_mask = 1;  /* Init Touch Mode */
#endif

	if ((get_bit(init_touch_md_check, bit_mask) != 1) || (cs_duty_val < ATMF04_CRCS_DUTY_LOW)\
			||(cr_duty_val <ATMF04_CRCS_DUTY_LOW) || (cs_duty_val > ATMF04_CRCS_DUTY_HIGH)\
			|| (cr_duty_val > ATMF04_CRCS_DUTY_HIGH) || /*(cs_per_result < ATMF04_CSCR_RESULT) || */(crcs_count > ATMF04_CRCS_COUNT)) {
		PINFO("[fail] 1.cal_check : %d, cr_value : %d, cs_value : %d, status : %d Count : %d",get_bit(init_touch_md_check, bit_mask), cr_duty_val, cs_duty_val, cs_per_result, crcs_count);
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",0);
	}
	else {
		PINFO("[pass] 2.cal_check : %d, cr_value : %d, cs_value : %d, status : %d Count : %d",get_bit(init_touch_md_check, bit_mask), cr_duty_val, cs_duty_val, cs_per_result, crcs_count);
		mutex_unlock(&data->update_lock);
		return sprintf(buf,"%d",1);
	}

}


static DEVICE_ATTR(onoff,        0664, NULL, atmf04_store_onoffsensor);
static DEVICE_ATTR(proxstatus,   0664, atmf04_show_proxstatus, NULL);
static DEVICE_ATTR(docalibration,0664, atmf04_show_docalibration, atmf04_store_docalibration);
static DEVICE_ATTR(reg_ctrl,     0664, atmf04_show_reg, atmf04_store_reg);
static DEVICE_ATTR(regproxdata,  0664, atmf04_show_regproxdata, NULL);
static DEVICE_ATTR(regreset,     0664, NULL, atmf04_store_regreset);
static DEVICE_ATTR(checkallnear, 0664, NULL, atmf04_store_checkallnear);
static DEVICE_ATTR(cntinputpins, 0664, atmf04_show_count_inputpins, NULL);
static DEVICE_ATTR(regproxctrl0, 0664, atmf04_show_regproxctrl0, NULL);
static DEVICE_ATTR(download,     0664, NULL, atmf04_store_firmware);
static DEVICE_ATTR(version,      0664, atmf04_show_version, NULL);
static DEVICE_ATTR(check_far,    0664, atmf04_show_check_far, NULL);
static DEVICE_ATTR(check_mid,    0664, atmf04_show_check_mid, NULL);


static struct attribute *atmf04_attributes[] = {
	&dev_attr_onoff.attr,
	&dev_attr_docalibration.attr,
	&dev_attr_proxstatus.attr,
	&dev_attr_reg_ctrl.attr,
	&dev_attr_regproxdata.attr,
	&dev_attr_regreset.attr,
	&dev_attr_checkallnear.attr,
	&dev_attr_cntinputpins.attr,
	&dev_attr_regproxctrl0.attr,
	&dev_attr_download.attr,
	&dev_attr_version.attr,
	&dev_attr_check_far.attr,
	&dev_attr_check_mid.attr,
	NULL,
};

static struct attribute_group atmf04_attr_group = {
	.attrs = atmf04_attributes,
};

static void atmf04_reschedule_work(struct atmf04_data *data,
		unsigned long delay)
{
	int ret;
	struct i2c_client *client = data->client;
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
		dev_err(&client->dev, "atmf04_reschedule_work : set wake lock timeout!\n");
		wake_lock_timeout(&data->ps_wlock, msecs_to_jiffies(1500));
		//cancel_delayed_work(&data->dwork);
		ret = queue_delayed_work(atmf04_workqueue, &data->dwork, delay);
		if (ret < 0) {
			PINFO("queue_work fail, ret = %d", ret);
		}
	} else {
		PINFO("cap sensor enable pin is already high... power off status");
	}
}

/* assume this is ISR */
static irqreturn_t atmf04_interrupt(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct atmf04_data *data = i2c_get_clientdata(client);
	int tmp = -1;
	tmp = atomic_read(&data->i2c_status);

	if (gpio_get_value(data->platform_data->chip_enable) == 0) { // if power on
		dev_err(&client->dev,"atmf04_interrupt\n");
		atmf04_reschedule_work(data, 0);
	}

	return IRQ_HANDLED;
}

static void atmf04_work_handler(struct work_struct *work)
{
	struct atmf04_data *data = container_of(work, struct atmf04_data, dwork.work);
	struct i2c_client *client = data->client;
	int irq_state;

#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
	if(probe_end_flag == true) {
#endif
	data->touch_out = i2c_smbus_read_byte_data(client, I2C_ADDR_TCH_OUTPUT);
	irq_state = gpio_get_value(data->platform_data->irq_gpio);
    PINFO("touch_out[%x] irq_state[%d]", data->touch_out, irq_state);

    /* When I2C fail and abnormal status*/
    if (data->touch_out < 0)
	{
		PINFO("I2C Error[%d]", data->touch_out);
        input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
        input_sync(data->input_dev_cap);

        PINFO("[%d]NEAR ",data->platform_data->irq_gpio);
	}
	else
	{
		if (gpio_get_value(data->platform_data->chip_enable) == 1) { // if power off
			PINFO("cap sensor is already power off : touch_out(0x%x)", data->touch_out);
			return;
		}
#ifdef CONFIG_LGE_ATMF04_2CH
        /* FAR */
	    if (irq_state == 0 && data->touch_out == (CH2_FAR|CH1_FAR))
	    {
	       data->cap_detection = 0;

		   input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* NEAR-to-FAR detection */
           input_sync(data->input_dev_cap);

           PINFO("[%d]FAR ",data->platform_data->irq_gpio);
	    }
        /* NEAR */
	    else
        {
           data->cap_detection = 1;

           input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
           input_sync(data->input_dev_cap);

           PINFO("[%d]NEAR ",data->platform_data->irq_gpio);
	    }
        PINFO("[%d] Work Handler done",data->platform_data->irq_gpio);
#else
        /* Occurred touch event */
        if(data->touch_out)
        {
			//	if ((data->touch_out == 1 ) && (data->cap_datection == 0)) {
			data->cap_detection = 1;

			input_report_abs(data->input_dev_cap, ABS_DISTANCE, NEAR_STATUS);/* FAR-to-NEAR detection */
			input_sync(data->input_dev_cap);

			PINFO("[%d]NEAR ",data->platform_data->irq_gpio);
	    }
        else
        {
			data->cap_detection = 0;

			input_report_abs(data->input_dev_cap, ABS_DISTANCE, FAR_STATUS);/* NEAR-to-FAR detection */
			input_sync(data->input_dev_cap);

			PINFO("[%d]FAR ",data->platform_data->irq_gpio);
		}
		PINFO("[%d] Work Handler done",data->platform_data->irq_gpio);
#endif
	}
#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
	}
	else{
		PINFO("probe_end_flag = False[%d]",probe_end_flag);
	}
#endif

}

int get_sensor_dts_func(const char *name, struct atmf04_hw *hw)
{
	int ret;
	u32 irq_gpio[] = {0};
	u32 chip_enable[] = {0};
	u32 InputPinsNum[] = {0};
	char* fw_name;

	struct device_node *node = NULL;

	if (name == NULL)
		return -1;

	node = of_find_compatible_node(NULL, NULL, name);
	if (node) {
		ret = of_property_read_u32_array(node, "Adsemicon,irq-gpio", irq_gpio, ARRAY_SIZE(irq_gpio));
		if (ret == 0)
			hw->irq_gpio = irq_gpio[0];
		ret = of_property_read_u32_array(node, "Adsemicon,chip_enable", chip_enable, ARRAY_SIZE(chip_enable));
		if (ret == 0)
			hw->chip_enable = chip_enable[0];

		ret = of_property_read_u32_array(node, "Adsemicon,InputPinsNum", InputPinsNum, ARRAY_SIZE(InputPinsNum));
		if (ret == 0)
			hw->InputPinsNum = InputPinsNum[0];

		ret = of_property_read_string(node, "Adsemicon,fw_name", (const char **)&fw_name);
		if (ret == 0)
			hw->fw_name = fw_name;

	} else {
		PERR("Device Tree: can not find node!. Go to use old cust info\n");
		return -1;
	}
	return 0;

}

static int atmf04_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct atmf04_data *data;
#ifdef CONFIG_OF
	struct atmf04_platform_data *platform_data;
#endif
	const char *name ="adsemicon,atmf04";
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		return -EIO;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct atmf04_data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}
	PINFO("ATMF04 probe, kzalloc complete\n");

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
				sizeof(struct atmf04_platform_data), GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		data->platform_data = platform_data;
		client->dev.platform_data = platform_data;
	} else {
		platform_data = client->dev.platform_data;
	}
#endif
	err = get_sensor_dts_func(name, hw);
	if (err < 0) {
		PERR("get dts info fail\n");
		goto exit;
	}
	data->platform_data->irq_gpio= hw->irq_gpio;
	data->platform_data->chip_enable = hw->chip_enable;
	data->platform_data->input_pins_num = hw->InputPinsNum;
	data->platform_data->fw_name = hw->fw_name;

	err = gpio_request(data->platform_data->chip_enable, "atmf04_chip_enable");
		if(err) {
			PINFO("chip_enable request fail\n");
			goto exit;
		}
	gpio_direction_output(data->platform_data->chip_enable, 0); // First, set chip_enable gpio direction to output.

	data->client = client;
	atmf04_i2c_client = client;
	i2c_set_clientdata(client, data);
	data->cap_detection = 0;

#ifdef CONFIG_OF
	/* h/w initialization */
	if (platform_data->init)
		err = platform_data->init(client);

	if (platform_data->power_on)
		err = platform_data->power_on(client, true);
#endif

	client->adapter->retries = 15;

	if (client->adapter->retries == 0)
		goto exit;

	atomic_set(&data->i2c_status, ATMF04_STATUS_RESUME);

	mutex_init(&data->update_lock);
	mutex_init(&data->enable_lock);

	wake_lock_init(&data->ps_wlock, WAKE_LOCK_SUSPEND, "capsensor_wakelock");
	INIT_DELAYED_WORK(&data->dwork, atmf04_work_handler);

	err = gpio_request(data->platform_data->irq_gpio, "atmf04_irq_gpio");
	if(err) {
		PINFO("irq_gpio request fail\n");
		goto exit_irq_init_failed;
	}
	gpio_direction_input(data->platform_data->irq_gpio); // First, set chip_enable gpio direction to output.

	data->irq = client->irq = gpio_to_irq(data->platform_data->irq_gpio);
	PINFO("irq_gpio #: %d\n", data->irq);

	err = request_irq(client->irq, atmf04_interrupt, IRQ_TYPE_EDGE_BOTH, ATMF04_DRV_NAME, (void *)client);
	if(err) {
		PINFO("chip_enable request fail\n");
		goto exit_irq_init_failed;
	}

	err = enable_irq_wake(data->irq);

	data->input_dev_cap = input_allocate_device();
	if (!data->input_dev_cap) {
		err = -ENOMEM;
		PINFO("Failed to allocate input device cap !\n");
		goto exit_free_irq;
	}

	set_bit(EV_ABS, data->input_dev_cap->evbit);

	input_set_abs_params(data->input_dev_cap, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_cap->name = "lge_sar_rf";
	data->input_dev_cap->dev.init_name = "lge_sar_rf";
	data->input_dev_cap->id.bustype = BUS_I2C;
	input_set_drvdata(data->input_dev_cap, data);

	err = input_register_device(data->input_dev_cap);
	if (err) {
		err = -ENOMEM;
		PINFO("Unable to register input device cap(%s)\n",
				data->input_dev_cap->name);
		goto exit_free_dev_cap;
	}

	if (data->platform_data->fw_name) {
		err = load_firmware(data, client, data->platform_data->fw_name);
		if (err) {
			PINFO("Failed to request firmware\n");
			//goto exit_free_irq;
			goto exit_irq_init_failed;
		}
	}
	err = sysfs_create_group(&data->input_dev_cap->dev.kobj, &atmf04_attr_group);
		if (err)
			PINFO("sysfs create fail!\n");

	/* default sensor off */
#if defined(CONFIG_LGE_CAP_SENSOR_IGNORE_INT_ON_PROBE)
	probe_end_flag = true;
#endif
	onoff_sensor(data, DISABLE_SENSOR_PINS);
	PINFO("interrupt is hooked\n");

	return 0;

exit_irq_init_failed:
	wake_lock_destroy(&data->ps_wlock);
	mutex_destroy(&data->update_lock);
	mutex_destroy(&data->enable_lock);
exit_free_dev_cap:
exit_free_irq:
	free_irq(data->irq, client);
exit:
	PINFO("Error");
	return err;
}

static int atmf04_remove(struct i2c_client *client)
{

	struct atmf04_data *data = i2c_get_clientdata(client);
	struct atmf04_platform_data *pdata = data->platform_data;


	disable_irq_wake(client->irq);

	free_irq(client->irq, client);

	if (pdata->power_on)
		pdata->power_on(client, false);

	if (pdata->exit)
		pdata->exit(client);

	mutex_destroy(&data->update_lock);
	mutex_destroy(&data->enable_lock);
	PINFO("ATMF04 remove\n");
	return 0;
}

static const struct i2c_device_id atmf04_id[] = {
	{ ATMF04_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, atmf04_id);

#ifdef CONFIG_OF
static struct of_device_id atmf04_match_table[] = {
	{ .compatible = "adsemicon,atmf04",},
	{ },
};
#else
#define atmf04_match_table NULL
#endif

static struct i2c_driver atmf04_driver = {
	.driver = {
		.name   = ATMF04_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = atmf04_match_table,
	},
	.probe  = atmf04_probe,
	.remove = atmf04_remove,
	.id_table = atmf04_id,
};

static void async_atmf04_init(void *data, async_cookie_t cookie)
{
	atmf04_workqueue = create_workqueue("capsensor");
	if (i2c_add_driver(&atmf04_driver)) {
		PINFO("failed at i2c_add_driver()");
		if(atmf04_workqueue)
			destroy_workqueue(atmf04_workqueue);

		atmf04_workqueue = NULL;
		return;
	}
}

static int __init atmf04_init(void)
{
	PINFO("ATMF04 init SAR Controller driver: release.\n");
	async_schedule(async_atmf04_init, NULL);

	return 0;
}

static void __exit atmf04_exit(void)
{
	PINFO("ATMF04 Proximity driver: release.\n");
	if (atmf04_workqueue)
		destroy_workqueue(atmf04_workqueue);

	atmf04_workqueue = NULL;

	i2c_del_driver(&atmf04_driver);
}

MODULE_DESCRIPTION("ATMF04 cap sensor driver");
MODULE_LICENSE("GPL");

module_init(atmf04_init);
module_exit(atmf04_exit);

