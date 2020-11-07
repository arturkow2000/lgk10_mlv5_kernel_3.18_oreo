/* akm09916.c - akm09916 compass driver
 *
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
/* #include <linux/earlysuspend.h> */

#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
/* #include <linux/proc_fs.h> */

/* #include <mach/mt_devs.h> */
/* #include <mach/mt_typedefs.h> */
/* #include <mach/mt_pm_ldo.h> */


#define POWER_NONE_MACRO -1

/* #include <cust_mag.h> */
#include "akm09916.h"
#include "mag.h"
#include <hwmsen_helper.h>

/*----------------------------------------------------------------------------*/
#define DEBUG 0
#define AKM09916_DEV_NAME         "akm09916"
#define DRIVER_VERSION          "1.0.1"
/*----------------------------------------------------------------------------*/
#define AKM09916_DEBUG		1
#define AKM09916_DEBUG_MSG	0
#define AKM09916_DEBUG_FUNC	0
#define AKM09916_DEBUG_DATA	1
#define MAX_FAILURE_COUNT	3
#define AKM09916_RETRY_COUNT	10
#define AKM09916_DEFAULT_DELAY	100

/* #define AKM_Pseudogyro                   // enable this if you need use 6D gyro */

#if AKM09916_DEBUG_MSG
#define AKMDBG(format, ...)	pr_err("AKM09916 " format "\n", ## __VA_ARGS__)
#else
#define AKMDBG(format, ...)
#endif

#if AKM09916_DEBUG_FUNC
#define AKMFUNC(func) pr_info("AKM09916 " func " is called\n")
#else
#define AKMFUNC(func)
#endif

static struct i2c_client *this_client;

/* Addresses to scan -- protected by sense_data_mutex */
static char sense_data[SENSOR_DATA_SIZE];
static struct mutex sense_data_mutex;
/* calibration msensor and orientation data */
static int sensor_data[CALIBRATION_DATA_SIZE];
static struct mutex sensor_data_mutex;
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static short akmd_delay = AKM09916_DEFAULT_DELAY;

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

static int factory_mode;
static int ecompass_status;

static int mEnabled;

/* static struct proc_dir_entry *akm09916_ecompass_status_proc = NULL; */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id akm09916_i2c_id[] = { {AKM09916_DEV_NAME, 0}, {} };

/* static struct i2c_board_info __initdata i2c_akm09916={ I2C_BOARD_INFO("akm09916", (AKM09916_I2C_ADDRESS>>1))}; */

struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

struct mag_hw *get_cust_mag(void)
{
	return &mag_cust;
}

/*the adapter id will be available in customization*/
/* static unsigned short akm09916_force[] = {0x00, AKM09916_I2C_ADDRESS, I2C_CLIENT_END, I2C_CLIENT_END}; */
/* static const unsigned short *const akm09916_forces[] = { akm09916_force, NULL }; */
/* static struct i2c_client_address_data akm09916_addr_data = { .forces = akm09916_forces,}; */
/*----------------------------------------------------------------------------*/
static int akm09916_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int akm09916_i2c_remove(struct i2c_client *client);
static int akm09916_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int akm09916_local_init(void);
static int akm09916_remove(void);


/*----------------------------------------------------------------------------*/
typedef enum {
	AMK_FUN_DEBUG = 0x01,
	AMK_DATA_DEBUG = 0X02,
	AMK_HWM_DEBUG = 0X04,
	AMK_CTR_DEBUG = 0X08,
	AMK_I2C_DEBUG = 0x10,
} AMK_TRC;


/*----------------------------------------------------------------------------*/
struct akm_i2c_data {
	struct i2c_client *client;
	struct mag_hw *hw;
	atomic_t layout;
	atomic_t trace;
	struct hwmsen_convert cvt;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id akm09916_of_match[] = {
	{.compatible = "mediatek,msensor",},
	{},
};

/* MODULE_DEVICE_TABLE(of, akm09916_of_match); */
#endif

static struct i2c_driver akm09916_i2c_driver = {
	.driver = {
/* .owner = THIS_MODULE, */
		   .name = AKM09916_DEV_NAME,
#ifdef CONFIG_OF
		   .of_match_table = akm09916_of_match,
#endif
		   },
	.probe = akm09916_i2c_probe,
	.remove = akm09916_i2c_remove,
	.detect = akm09916_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
/* .suspend    = akm09916_suspend, */
/* .resume     = akm09916_resume, */
#endif
	.id_table = akm09916_i2c_id,
/* .address_data = &akm09916_addr_data, */
};

/*----------------------------------------------------------------------------*/
#if 0
static struct platform_driver akm_sensor_driver = {
	.probe = akm_probe,
	.remove = akm_remove,
	.driver = {
		   .name = "msensor",
		   .owner = THIS_MODULE,
		   }
};
#endif

static struct mag_init_info akm09916_init_info = {
	.name = "akm09916",
	.init = akm09916_local_init,
	.uninit = akm09916_remove,
};


/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
/*----------------------------------------------------------------------------*/
static void akm09916_power(struct mag_hw *hw, unsigned int on)
{
	static unsigned int power_on;

	power_on = on;
}

static long AKI2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;

#if DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
	char addr = rxData[0];
#endif


	/* Caller should check parameter validity. */
	if ((rxData == NULL) || (length < 1))
		return -EINVAL;

	for (loop_i = 0; loop_i < AKM09916_RETRY_COUNT; loop_i++) {
		this_client->addr = this_client->addr & I2C_MASK_FLAG;
		this_client->addr = this_client->addr | I2C_WR_FLAG;
		if (i2c_master_send(this_client, (const char *)rxData, ((length << 0X08) | 0X01)))
			break;

		mdelay(10);
	}

	if (loop_i >= AKM09916_RETRY_COUNT) {
		pr_err("%s retry over %d\n", __func__, AKM09916_RETRY_COUNT);
		return -EIO;
	}
#if DEBUG
	if (atomic_read(&data->trace) & AMK_I2C_DEBUG) {
		pr_info("RxData: len=%02x, addr=%02x\n  data=", length, addr);
		for (i = 0; i < length; i++)
			pr_info(" %02x", rxData[i]);

		pr_info("\n");
	}
#endif
	return 0;
}

static long AKI2C_TxData(char *txData, int length)
{
	uint8_t loop_i;

#if DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
#endif

	/* Caller should check parameter validity. */
	if ((txData == NULL) || (length < 2))
		return -EINVAL;

	this_client->addr = this_client->addr & I2C_MASK_FLAG;
	for (loop_i = 0; loop_i < AKM09916_RETRY_COUNT; loop_i++) {
		if (i2c_master_send(this_client, (const char *)txData, length) > 0)
			break;
		mdelay(10);
	}

	if (loop_i >= AKM09916_RETRY_COUNT) {
		pr_err("%s retry over %d\n", __func__, AKM09916_RETRY_COUNT);
		return -EIO;
	}
#if DEBUG
	if (atomic_read(&data->trace) & AMK_I2C_DEBUG) {
		pr_info("TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for (i = 0; i < (length - 1); i++)
			pr_info(" %02x", txData[i + 1]);

		pr_info("\n");
	}
#endif
	return 0;
}

static long AKECS_SetMode_SngMeasure(void)
{
	char buffer[2];
	/* Set measure mode */
	buffer[0] = AK09916_REG_CNTL2;
	buffer[1] = AK09916_MODE_SNG_MEASURE;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static long AKECS_SetMode_SelfTest(void)
{
	char buffer[2];

	/* Set measure mode */
	buffer[0] = AK09916_REG_CNTL2;
	buffer[1] = AK09916_MODE_SELF_TEST;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode_PowerDown(void)
{
	char buffer[2];
	/* Set powerdown mode */
	buffer[0] = AK09916_REG_CNTL2;
	buffer[1] = AK09916_MODE_POWERDOWN;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static long AKECS_Reset(int hard)
{
	unsigned char buffer[2];
	long err = 0;

	if (hard != 0) {
		/* TODO change to board setting */
		/* gpio_set_value(akm->rstn, 0); */
		udelay(5);
		/* gpio_set_value(akm->rstn, 1); */
	} else {
		/* Set measure mode */
		buffer[0] = AK09916_REG_CNTL3;
		buffer[1] = AK09916_RESET_DATA;

		err = AKI2C_TxData(buffer, 2);
		if (err < 0)
			AKMDBG("%s: Can not set SRST bit.", __func__);
	 else
			AKMDBG("Soft reset is done.");

	}

	/* Device will be accessible 300 us after */
	udelay(300);		/* 100 */

	return err;
}

static long AKECS_SetMode(char mode)
{
	long ret;

	switch (mode & 0x1F) {


	case AK09916_MODE_SNG_MEASURE:
		ret = AKECS_SetMode_SngMeasure();
		break;

	case AK09916_MODE_SELF_TEST:
		ret = AKECS_SetMode_SelfTest();
		break;

	case AK09916_MODE_POWERDOWN:
		ret = AKECS_SetMode_PowerDown();
		break;

	default:
		AKMDBG("%s: Unknown mode(%d)", __func__, mode);
		return -EINVAL;
	}

	/* wait at least 100us after changing mode */
	udelay(100);

	return ret;
}

static int AKECS_CheckDevice(void)
{
	char buffer[2];
	int ret;

	AKMDBG(" AKM check device id");
	buffer[0] = AK09916_REG_WIA1;



	/* Read data */
	ret = AKI2C_RxData(buffer, AKM_SENSOR_INFO_SIZE);
	AKMDBG(" AKM check device id = %x", buffer[0]);
	AKMDBG("ret = %d", ret);
	if (ret < 0)
		return ret;

	/* Check read data */
	if ((buffer[0] != AK09916_WIA1_VALUE) || (buffer[1] != AK09916_WIA2_VALUE))
		return -ENXIO;

	return 0;
}

/* Daemon application save the data */
static void AKECS_SaveData(int *buf)
{
#if DEBUG
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
#endif

	mutex_lock(&sensor_data_mutex);
	memcpy(sensor_data, buf, sizeof(sensor_data));
	mutex_unlock(&sensor_data_mutex);
}

/* M-sensor daemon application have set the sng mode */
static long AKECS_GetData(char *rbuf, int size)
{
	char temp;
	int loop_i, ret;
#if DEBUG
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
#endif

	if (size < SENSOR_DATA_SIZE) {
		pr_err("buff size is too small %d!\n", size);
		return -1;
	}

	memset(rbuf, 0, SENSOR_DATA_SIZE);
	rbuf[0] = AK09916_REG_ST1;

	for (loop_i = 0; loop_i < AKM09916_RETRY_COUNT; loop_i++) {
		ret = AKI2C_RxData(rbuf, 1);
		if (ret) {
			pr_err("read ST1 resigster failed!\n");
			return -1;
		}

		if ((rbuf[0] & 0x01) == 0x01)
			break;

		mdelay(2);
		rbuf[0] = AK09916_REG_ST1;
	}

	if (loop_i >= AKM09916_RETRY_COUNT) {
		pr_err("Data read retry larger the max count!\n");
		if (0 == factory_mode)
			return -1;	/* if return we can not get data at factory mode */
	}

	temp = rbuf[0];
	rbuf[1] = AK09916_REG_HXL;
	ret = AKI2C_RxData(&rbuf[1], SENSOR_DATA_SIZE - 1);

	if (ret < 0) {
		pr_err("AKM8975 akm8975_work_func: I2C failed\n");
		return -1;
	}
	rbuf[0] = temp;
	mutex_lock(&sense_data_mutex);
	memcpy(sense_data, rbuf, sizeof(sense_data));
	mutex_unlock(&sense_data_mutex);

#if DEBUG
	if (atomic_read(&data->trace) & AMK_DATA_DEBUG) {
		AKMDBG("Get device data: %d, %d, %d, %d , %d, %d, %d, %d!\n",
		       sense_data[0], sense_data[1], sense_data[2], sense_data[3],
		       sense_data[4], sense_data[5], sense_data[6], sense_data[7]);
	}
#endif

	return 0;
}

/* Get Msensor Raw data */
static int AKECS_GetRawData(char *rbuf, int size)
{
	char strbuf[SENSOR_DATA_SIZE];
	s16 data[3];

	if ((atomic_read(&open_flag) == 0) || (factory_mode == 1)) {
		AKECS_SetMode_SngMeasure();
		mdelay(10);
	}

	AKECS_GetData(strbuf, SENSOR_DATA_SIZE);
	data[0] = (s16) (strbuf[1] | (strbuf[2] << 8));
	data[1] = (s16) (strbuf[3] | (strbuf[4] << 8));
	data[2] = (s16) (strbuf[5] | (strbuf[6] << 8));

	sprintf(rbuf, "%x %x %x", data[0], data[1], data[2]);

	return 0;

}



static int AKECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}

static int AKECS_GetCloseStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) <= 0));
	return atomic_read(&open_flag);
}




/*----------------------------------------------------------------------------*/
static int akm09916_ReadChipInfo(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= AKM09916_BUFSIZE - 1))
		return -1;

	if (!this_client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "akm09916 Chip");
	return 0;
}

/*----------------------------shipment test------------------------------------------------*/
/*!
 @return If @a testdata is in the range of between @a lolimit and @a hilimit,
 the return value is 1, otherwise -1.
 @param[in] testno   A pointer to a text string.
 @param[in] testname A pointer to a text string.
 @param[in] testdata A data to be tested.
 @param[in] lolimit  The maximum allowable value of @a testdata.
 @param[in] hilimit  The minimum allowable value of @a testdata.
 @param[in,out] pf_total
 */
int
TEST_DATA(const char testno[],
	  const char testname[],
	  const int testdata, const int lolimit, const int hilimit, int *pf_total)
{
	int pf;			/* Pass;1, Fail;-1 */

	if ((testno == NULL) && (strncmp(testname, "START", 5) == 0)) {
		/* Display header */
		AKMDBG("--------------------------------------------------------------------\n");
		AKMDBG(" Test No. Test Name    Fail    Test Data    [      Low         High]\n");
		AKMDBG("--------------------------------------------------------------------\n");

		pf = 1;
	} else if ((testno == NULL) && (strncmp(testname, "END", 3) == 0)) {
		/* Display result */
		AKMDBG("--------------------------------------------------------------------\n");
		if (*pf_total == 1)
			AKMDBG("Factory shipment test was passed.\n\n");
		else
			AKMDBG("Factory shipment test was failed.\n\n");

		pf = 1;
	} else {
		if ((lolimit <= testdata) && (testdata <= hilimit))
			pf = 1;	/* Pass */
		else
			pf = -1;	/* Fail */

		/* display result */
		AKMDBG(" %7s  %-10s      %c    %9d    [%9d    %9d]\n",
		       testno, testname, ((pf == 1) ? ('.') : ('F')), testdata, lolimit, hilimit);
	}

	/* Pass/Fail check */
	if (*pf_total != 0) {
		if ((*pf_total == 1) && (pf == 1))
			*pf_total = 1;	/* Pass */
		else
			*pf_total = -1;	/* Fail */
	}
	return pf;
}

/*!
 Execute "Onboard Function Test" (NOT includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
int FST_AK09916(void)
{
	int pf_total;		/* p/f flag for this subtest */
	char i2cData[16];
	int hdata[3];

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	pf_total = 1;

	/* *********************************************** */
	/* Step1 */
	/* *********************************************** */

	/* Reset device. */
	if (AKECS_Reset(0) < 0) {
		AKMDBG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}
	/* Read values from WIA. */
	i2cData[0] = AK09916_REG_WIA1;
	if (AKI2C_RxData(i2cData, 2) < 0) {
		AKMDBG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}
	/* TEST */
	TEST_DATA(TLIMIT_NO_RST_WIA1_09916, TLIMIT_TN_RST_WIA1_09916, (int)i2cData[0],
		  TLIMIT_LO_RST_WIA1_09916, TLIMIT_HI_RST_WIA1_09916, &pf_total);
	TEST_DATA(TLIMIT_NO_RST_WIA2_09916, TLIMIT_TN_RST_WIA2_09916, (int)i2cData[1],
		  TLIMIT_LO_RST_WIA2_09916, TLIMIT_HI_RST_WIA2_09916, &pf_total);

	/* Set to PowerDown mode */
	if (AKECS_SetMode(AK09916_MODE_POWERDOWN) < 0) {
		AKMDBG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}
	/* *********************************************** */
	/* Step2 */
	/* *********************************************** */

	/* Set to SNG measurement pattern (Set CNTL register) */
	if (AKECS_SetMode(AK09916_MODE_SNG_MEASURE) < 0) {
		AKMDBG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}
	/* Wait for DRDY pin changes to HIGH. */
	/* usleep(AKM_MEASURE_TIME_US); */
	/* Get measurement data from AK09916 */
	/* ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + TEMP + ST2 */
	/* = 1 + (1 + 1) + (1 + 1) + (1 + 1) + 1 + 1 = 9yte */
	/* if (AKD_GetMagneticData(i2cData) != AKD_SUCCESS) { */
	if (AKECS_GetData(i2cData, SENSOR_DATA_SIZE) < 0) {
		AKMDBG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}
	/* hdata[0] = (int)((((uint)(i2cData[2]))<<8)+(uint)(i2cData[1])); */
	/* hdata[1] = (int)((((uint)(i2cData[4]))<<8)+(uint)(i2cData[3])); */
	/* hdata[2] = (int)((((uint)(i2cData[6]))<<8)+(uint)(i2cData[5])); */

	hdata[0] = (s16) (i2cData[1] | (i2cData[2] << 8));
	hdata[1] = (s16) (i2cData[3] | (i2cData[4] << 8));
	hdata[2] = (s16) (i2cData[5] | (i2cData[6] << 8));

	/* TEST */
	/* i2cData[0] &= 0x7F; */
	TEST_DATA(TLIMIT_NO_SNG_ST1_09916, TLIMIT_TN_SNG_ST1_09916, (int)i2cData[0],
		  TLIMIT_LO_SNG_ST1_09916, TLIMIT_HI_SNG_ST1_09916, &pf_total);

	/* TEST */
	TEST_DATA(TLIMIT_NO_SNG_HX_09916, TLIMIT_TN_SNG_HX_09916, hdata[0], TLIMIT_LO_SNG_HX_09916,
		  TLIMIT_HI_SNG_HX_09916, &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_HY_09916, TLIMIT_TN_SNG_HY_09916, hdata[1], TLIMIT_LO_SNG_HY_09916,
		  TLIMIT_HI_SNG_HY_09916, &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_HZ_09916, TLIMIT_TN_SNG_HZ_09916, hdata[2], TLIMIT_LO_SNG_HZ_09916,
		  TLIMIT_HI_SNG_HZ_09916, &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_ST2_09916, TLIMIT_TN_SNG_ST2_09916, (int)i2cData[8],
		  TLIMIT_LO_SNG_ST2_09916, TLIMIT_HI_SNG_ST2_09916, &pf_total);

	/* Set to Self-test mode (Set CNTL register) */
	if (AKECS_SetMode(AK09916_MODE_SELF_TEST) < 0) {
		AKMDBG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}
	/* Wait for DRDY pin changes to HIGH. */
	/* usleep(AKM_MEASURE_TIME_US); */
	/* Get measurement data from AK09916 */
	/* ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + TEMP + ST2 */
	/* = 1 + (1 + 1) + (1 + 1) + (1 + 1) + 1 + 1 = 9byte */
	/* if (AKD_GetMagneticData(i2cData) != AKD_SUCCESS) { */
	if (AKECS_GetData(i2cData, SENSOR_DATA_SIZE) < 0) {
		AKMDBG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}
	/* TEST */
	i2cData[0] &= 0x7F;
	TEST_DATA(TLIMIT_NO_SLF_ST1_09916, TLIMIT_TN_SLF_ST1_09916, (int)i2cData[0],
		  TLIMIT_LO_SLF_ST1_09916, TLIMIT_HI_SLF_ST1_09916, &pf_total);

	/* hdata[0] = (int)((((uint)(i2cData[2]))<<8)+(uint)(i2cData[1])); */
	/* hdata[1] = (int)((((uint)(i2cData[4]))<<8)+(uint)(i2cData[3])); */
	/* hdata[2] = (int)((((uint)(i2cData[6]))<<8)+(uint)(i2cData[5])); */

	hdata[0] = (s16) (i2cData[1] | (i2cData[2] << 8));
	hdata[1] = (s16) (i2cData[3] | (i2cData[4] << 8));
	hdata[2] = (s16) (i2cData[5] | (i2cData[6] << 8));

	/* TEST */
	TEST_DATA(TLIMIT_NO_SLF_RVHX_09916,
		  TLIMIT_TN_SLF_RVHX_09916,
		  hdata[0], TLIMIT_LO_SLF_RVHX_09916, TLIMIT_HI_SLF_RVHX_09916, &pf_total);

	TEST_DATA(TLIMIT_NO_SLF_RVHY_09916,
		  TLIMIT_TN_SLF_RVHY_09916,
		  hdata[1], TLIMIT_LO_SLF_RVHY_09916, TLIMIT_HI_SLF_RVHY_09916, &pf_total);

	TEST_DATA(TLIMIT_NO_SLF_RVHZ_09916,
		  TLIMIT_TN_SLF_RVHZ_09916,
		  hdata[2], TLIMIT_LO_SLF_RVHZ_09916, TLIMIT_HI_SLF_RVHZ_09916, &pf_total);

	TEST_DATA(TLIMIT_NO_SLF_ST2_09916,
		  TLIMIT_TN_SLF_ST2_09916,
		  (int)i2cData[8], TLIMIT_LO_SLF_ST2_09916, TLIMIT_HI_SLF_ST2_09916, &pf_total);

	return pf_total;
}

/*!
 Execute "Onboard Function Test" (includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
int FctShipmntTestProcess_Body(void)
{
	int pf_total = 1;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	TEST_DATA(NULL, "START", 0, 0, 0, &pf_total);

	/* *********************************************** */
	/* Step 1 to 2 */
	/* *********************************************** */
	pf_total = FST_AK09916();

	/* *********************************************** */
	/* Judge Test Result */
	/* *********************************************** */
	TEST_DATA(NULL, "END", 0, 0, 0, &pf_total);

	return pf_total;
}

static ssize_t store_shipment_test(struct device_driver *ddri, const char *buf, size_t count)
{
	/* struct i2c_client *client = this_client; */
	/* struct akm_i2c_data *data = i2c_get_clientdata(client); */
	/* int layout = 0; */


	return count;
}

static ssize_t show_shipment_test(struct device_driver *ddri, char *buf)
{
	char result[10];
	int res = 0;

	res = FctShipmntTestProcess_Body();
	if (1 == res) {
		AKMDBG("shipment_test pass\n");
		strcpy(result, "y");
	} else if (-1 == res) {
		AKMDBG("shipment_test fail\n");
		strcpy(result, "n");
	} else {
		AKMDBG("shipment_test NaN\n");
		strcpy(result, "NaN");
	}

	return sprintf(buf, "%s\n", result);
}

static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[AKM09916_BUFSIZE];

	sprintf(strbuf, "akmd09916");
	return sprintf(buf, "%s", strbuf);
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[AKM09916_BUFSIZE];

	akm09916_ReadChipInfo(strbuf, AKM09916_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	char sensordata[SENSOR_DATA_SIZE];
	char strbuf[AKM09916_BUFSIZE];

	if (atomic_read(&open_flag) == 0) {
		AKECS_SetMode_SngMeasure();
		mdelay(10);
		AKECS_GetData(sensordata, SENSOR_DATA_SIZE);
	} else {
		mutex_lock(&sense_data_mutex);
		memcpy(sensordata, sense_data, sizeof(sensordata));
		mutex_unlock(&sense_data_mutex);
	}



	sprintf(strbuf, "%d %d %d %d %d %d %d %d %d\n", sensordata[0], sensordata[1], sensordata[2],
		sensordata[3], sensordata[4], sensordata[5], sensordata[6], sensordata[7],
		sensordata[8]);

	return sprintf(buf, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
	short tmp[3];
	char strbuf[AKM09916_BUFSIZE];

	tmp[0] = sensor_data[13] * CONVERT_O / CONVERT_O_DIV;
	tmp[1] = sensor_data[14] * CONVERT_O / CONVERT_O_DIV;
	tmp[2] = sensor_data[15] * CONVERT_O / CONVERT_O_DIV;
	sprintf(strbuf, "%d, %d, %d\n", tmp[0], tmp[1], tmp[2]);

	return sprintf(buf, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		       data->hw->direction, atomic_read(&data->layout), data->cvt.sign[0],
		       data->cvt.sign[1], data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1],
		       data->cvt.map[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if (1 == kstrtoint(buf, 10, &layout)) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt))
			pr_err("HWMSEN_GET_CONVERT function error!\r\n");
		else if (!hwmsen_get_convert(data->hw->direction, &data->cvt))
			pr_err("invalid layout: %d, restore to %d\n", layout,
			       data->hw->direction);
		else {
			pr_err("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else {
		pr_err("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
	ssize_t len = 0;

	if (data->hw) {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
				data->hw->i2c_num, data->hw->direction, data->hw->power_id,
				data->hw->power_vol);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct akm_i2c_data *obj = i2c_get_clientdata(this_client);

	if (NULL == obj) {
		pr_err("akm_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct akm_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;

	if (NULL == obj) {
		pr_err("akm_i2c_data is null!!\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		pr_err("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon, S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(shipmenttest, S_IRUGO | S_IWUSR, show_shipment_test, store_shipment_test);
static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(layout, S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace, S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *akm09916_attr_list[] = {
	&driver_attr_daemon,
	&driver_attr_shipmenttest,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_posturedata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
};

/*----------------------------------------------------------------------------*/
static int akm09916_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(akm09916_attr_list) / sizeof(akm09916_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, akm09916_attr_list[idx]);
		if (err) {
			pr_err("driver_create_file (%s) = %d\n",
			       akm09916_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int akm09916_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(akm09916_attr_list) / sizeof(akm09916_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, akm09916_attr_list[idx]);

	return err;
}


/*----------------------------------------------------------------------------*/
static int akm09916_open(struct inode *inode, struct file *file)
{
	struct akm_i2c_data *obj = i2c_get_clientdata(this_client);
	int ret = -1;

	if (atomic_read(&obj->trace) & AMK_CTR_DEBUG)
		AKMDBG("Open device node:akm09916\n");

	ret = nonseekable_open(inode, file);

	return ret;
}

/*----------------------------------------------------------------------------*/
static int akm09916_release(struct inode *inode, struct file *file)
{
	struct akm_i2c_data *obj = i2c_get_clientdata(this_client);

	atomic_dec(&dev_open_count);
	if (atomic_read(&obj->trace) & AMK_CTR_DEBUG)
		AKMDBG("Release device node:akm09916\n");

	return 0;
}


/*----------------------------------------------------------------------------*/
/* static int akm09916_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg) */
static long akm09916_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char sData[SENSOR_DATA_SIZE];	/* for GETDATA */
	char rwbuf[RWBUF_SIZE];	/* for READ/WRITE */
	char buff[AKM09916_BUFSIZE];	/* for chip information */
	char mode;		/* for SET_MODE */
	int value[26];		/* for SET_YPR */
	int64_t delay[3];	/* for GET_DELAY */
	int caldata[3];
	int status;		/* for OPEN/CLOSE_STATUS */
	long ret = -1;		/* Return value. */
	char layout;
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
	struct hwm_sensor_data *osensor_data;
	uint32_t enable;
	/* These two buffers are initialized at start up.
	   After that, the value is not changed */
	unsigned char sense_info[AKM_SENSOR_INFO_SIZE];
	unsigned char sense_conf[AKM_SENSOR_CONF_SIZE];

	/* pr_err("akm09916 cmd:0x%x\n", cmd); */
	switch (cmd) {
	case ECS_IOCTL_WRITE:
		/* AKMFUNC("ECS_IOCTL_WRITE"); */
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(rwbuf, argp, sizeof(rwbuf))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}

		if ((rwbuf[0] < 2) || (rwbuf[0] > (RWBUF_SIZE - 1))) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;

		break;
	case ECS_IOCTL_RESET:
		ret = AKECS_Reset(0);	/* sw: 0, hw: 1 */
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_READ:
		/* AKMFUNC("ECS_IOCTL_READ"); */
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}

		if (copy_from_user(rwbuf, argp, sizeof(rwbuf))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}

		if ((rwbuf[0] < 1) || (rwbuf[0] > (RWBUF_SIZE - 1))) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;

		if (copy_to_user(argp, rwbuf, rwbuf[0] + 1)) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_INFO:

		sense_info[0] = AK09916_REG_WIA1;

		ret = AKI2C_RxData(sense_info, AKM_SENSOR_INFO_SIZE);
		if (ret < 0)
			return ret;

		if (copy_to_user(argp, sense_info, AKM_SENSOR_INFO_SIZE)) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_CONF:
		sense_conf[0] = sense_conf[1] = sense_conf[2] = 128;
		if (copy_to_user(argp, sense_conf, AKM_SENSOR_CONF_SIZE)) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}

		break;

	case ECS_IOCTL_SET_MODE:
		/* AKMFUNC("ECS_IOCTL_SET_MODE"); */
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&mode, argp, sizeof(mode))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		ret = AKECS_SetMode(mode);	/* MATCH command from AKMD PART */
		if (ret < 0)
			return ret;

		break;

	case ECS_IOCTL_GETDATA:
		/* AKMFUNC("ECS_IOCTL_GETDATA"); */
		ret = AKECS_GetData(sData, SENSOR_DATA_SIZE);
		if (ret < 0)
			return ret;

		if (copy_to_user(argp, sData, sizeof(sData))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_SET_YPR_09916:
		/* AKMFUNC("ECS_IOCTL_SET_YPR"); */
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(value, argp, sizeof(value))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		AKECS_SaveData(value);
		break;

	case ECS_IOCTL_GET_OPEN_STATUS:
		/* AKMFUNC("IOCTL_GET_OPEN_STATUS"); */
		status = AKECS_GetOpenStatus();
		/* AKMDBG("AKECS_GetOpenStatus returned (%d)", status); */
		if (copy_to_user(argp, &status, sizeof(status))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_GET_CLOSE_STATUS:
		/* AKMFUNC("IOCTL_GET_CLOSE_STATUS"); */
		status = AKECS_GetCloseStatus();
		/* AKMDBG("AKECS_GetCloseStatus returned (%d)", status); */
		if (copy_to_user(argp, &status, sizeof(status))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_GET_OSENSOR_STATUS:
		/* AKMFUNC("ECS_IOCTL_GET_OSENSOR_STATUS"); */
		status = atomic_read(&o_flag);
		if (copy_to_user(argp, &status, sizeof(status))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_GET_DELAY_09916:
		/* AKMFUNC("IOCTL_GET_DELAY"); */
		delay[0] = (int)akmd_delay * 1000000;
		delay[1] = (int)akmd_delay * 1000000;
		delay[2] = (int)akmd_delay * 1000000;
		if (copy_to_user(argp, delay, sizeof(delay))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case ECS_IOCTL_GET_LAYOUT_09916:
		layout = atomic_read(&data->layout);
		pr_err("layout=%d\r\n", layout);
		if (copy_to_user(argp, &layout, sizeof(layout))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;

	case MSENSOR_IOCTL_READ_CHIPINFO:
		if (argp == NULL) {
			pr_err("IO parameter pointer is NULL!\r\n");
			break;
		}

		akm09916_ReadChipInfo(buff, AKM09916_BUFSIZE);
		if (copy_to_user(argp, buff, strlen(buff) + 1))
			return -EFAULT;

		break;

	case MSENSOR_IOCTL_READ_SENSORDATA:
		if (argp == NULL) {
			pr_err("IO parameter pointer is NULL!\r\n");
			break;
		}

		AKECS_GetRawData(buff, AKM09916_BUFSIZE);

		if (copy_to_user(argp, buff, strlen(buff) + 1))
			return -EFAULT;

		break;

	case MSENSOR_IOCTL_READ_CALIDATA:
		if (argp == NULL) {
			pr_err("IO parameter pointer is NULL!\r\n");
			break;
		}
		caldata[0] = sensor_data[5] * CONVERT_M;
		caldata[1] = sensor_data[6] * CONVERT_M;
		caldata[2] = sensor_data[7] * CONVERT_M;

		sprintf(buff, "%x %x %x", caldata[0], caldata[1], caldata[2]);

		if (copy_to_user(argp, buff, strlen(buff) + 1))
			return -EFAULT;

		break;

	case MSENSOR_IOCTL_SENSOR_ENABLE:

		if (argp == NULL) {
			pr_err("IO parameter pointer is NULL!\r\n");
			break;
		}
		if (copy_from_user(&enable, argp, sizeof(enable))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		pr_debug("MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\r\n", enable);
		factory_mode = 1;
		if (1 == enable) {
			atomic_set(&o_flag, 1);
			atomic_set(&open_flag, 1);
		} else {
			atomic_set(&o_flag, 0);
			if (atomic_read(&m_flag) == 0)
				atomic_set(&open_flag, 0);
		}
		wake_up(&open_wq);

		break;

	case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
		if (argp == NULL) {
			pr_err("IO parameter pointer is NULL!\r\n");
			break;
		}
		/* AKECS_GetRawData(buff, AKM09916_BUFSIZE); */
		osensor_data = (struct hwm_sensor_data *)buff;
		mutex_lock(&sensor_data_mutex);

		osensor_data->values[0] = sensor_data[13] * CONVERT_O;
		osensor_data->values[1] = sensor_data[14] * CONVERT_O;
		osensor_data->values[2] = sensor_data[15] * CONVERT_O;
		osensor_data->status = sensor_data[8];
		osensor_data->value_divide = CONVERT_O_DIV;

		mutex_unlock(&sensor_data_mutex);

		sprintf(buff, "%x %x %x %x %x", osensor_data->values[0], osensor_data->values[1],
			osensor_data->values[2], osensor_data->status, osensor_data->value_divide);
		if (copy_to_user(argp, buff, strlen(buff) + 1))
			return -EFAULT;

		break;

	default:
		pr_err("%s not supported = 0x%04x", __func__, cmd);
		return -ENOIOCTLCMD;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static long akm09916_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_ECS_IOCTL_WRITE:
		if (arg32 == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_WRITE, (unsigned long)arg32);
		if (ret) {
			AKMDBG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
			return ret;
		}

		break;
	case COMPAT_ECS_IOCTL_RESET:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_RESET, (unsigned long)arg32);
		if (ret) {
			AKMDBG("ECS_IOCTL_RESET unlocked_ioctl failed.");
			return ret;
		}
		break;
	case COMPAT_ECS_IOCTL_READ:
		if (arg32 == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_READ, (unsigned long)arg32);
		if (ret) {
			AKMDBG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_INFO:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_INFO, (unsigned long)(arg32));
		if (ret) {
			AKMDBG("ECS_IOCTL_GET_INFO unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_CONF:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_CONF, (unsigned long)(arg32));
		if (ret) {
			AKMDBG("ECS_IOCTL_GET_CONF unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_SET_MODE:
		if (arg32 == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_SET_MODE, (unsigned long)(arg32));
		if (ret) {
			AKMDBG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
			return ret;
		}
		break;

	case COMPAT_ECS_IOCTL_GETDATA:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GETDATA, (unsigned long)(arg32));
		if (ret) {
			AKMDBG("ECS_IOCTL_GETDATA unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_SET_YPR_09916:
		if (arg32 == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_SET_YPR_09916,
						 (unsigned long)(arg32));
		if (ret) {
			AKMDBG("ECS_IOCTL_SET_YPR_09916 unlocked_ioctl failed.");
			return ret;
		}
		break;

	case COMPAT_ECS_IOCTL_GET_OPEN_STATUS:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_OPEN_STATUS,
						 (unsigned long)(arg32));
		if (ret) {
			AKMDBG("ECS_IOCTL_GET_OPEN_STATUS unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_CLOSE_STATUS:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_CLOSE_STATUS,
						 (unsigned long)(arg32));
		if (ret) {
			AKMDBG("ECS_IOCTL_GET_CLOSE_STATUS unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_OSENSOR_STATUS:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_OSENSOR_STATUS,
						 (unsigned long)(arg32));
		if (ret) {
			AKMDBG("ECS_IOCTL_GET_OSENSOR_STATUS unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_DELAY_09916:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_DELAY_09916,
						 (unsigned long)(arg32));
		if (ret) {
			AKMDBG("ECS_IOCTL_GET_DELAY_09916 unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_ECS_IOCTL_GET_LAYOUT_09916:
		ret = file->f_op->unlocked_ioctl(file, ECS_IOCTL_GET_LAYOUT_09916,
						 (unsigned long)arg32);
		if (ret) {
			AKMDBG("ECS_IOCTL_GET_LAYOUT_09916 unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_MSENSOR_IOCTL_READ_CHIPINFO:
		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CHIPINFO,
						 (unsigned long)arg32);
		if (ret) {
			AKMDBG("MSENSOR_IOCTL_READ_CHIPINFO unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_MSENSOR_IOCTL_READ_SENSORDATA:
		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_SENSORDATA,
						 (unsigned long)arg32);
		if (ret) {
			AKMDBG("MSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE:
		if (arg32 == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SENSOR_ENABLE,
						 (unsigned long)(arg32));
		if (ret) {
			AKMDBG("MSENSOR_IOCTL_SENSOR_ENABLE unlocked_ioctl failed.");
			return ret;
		}

		break;

	case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
		if (arg32 == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}

		ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_FACTORY_SENSORDATA,
						 (unsigned long)(arg32));
		if (ret) {
			AKMDBG("MSENSOR_IOCTL_READ_FACTORY_SENSORDATA unlocked_ioctl failed.");
			return ret;
		}
		break;

	default:
		/* pr_err("%s not supported = 0x%04x", __FUNCTION__, cmd); */
		return -ENOIOCTLCMD;
	}
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static const struct file_operations akm09916_fops = {
	.owner = THIS_MODULE,
	.open = akm09916_open,
	.release = akm09916_release,
	/* .unlocked_ioctl = akm09916_ioctl, */
	.unlocked_ioctl = akm09916_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = akm09916_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice akm09916_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msensor",
	.fops = &akm09916_fops,
};

/*----------------------------------------------------------------------------*/
int akm09916_operate(void *self, uint32_t command, void *buff_in, int size_in,
		     void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *msensor_data;

#if DEBUG
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & AMK_FUN_DEBUG)
		AKMFUNC("akm09916_operate");
#endif
	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 10)
				value = 10;
			akmd_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		AKMDBG("Enable sensor!\n");
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {

			value = *(int *)buff_in;

			if (value == 1) {
				atomic_set(&m_flag, 1);
				atomic_set(&open_flag, 1);
			} else {
				atomic_set(&m_flag, 0);
				if (atomic_read(&o_flag) == 0) {
					/* atomic_set(&m_flag, 0);  */
					/* if gyro, rv,la ,gravity open , then  m flag open */
					atomic_set(&open_flag, 0);
				}
			}
			wake_up(&open_wq);

			/* TODO: turn device into standby or normal mode */
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			pr_err("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			msensor_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			msensor_data->values[0] = sensor_data[5] * CONVERT_M;
			msensor_data->values[1] = sensor_data[6] * CONVERT_M;
			msensor_data->values[2] = sensor_data[7] * CONVERT_M;
			msensor_data->status = sensor_data[8];
			msensor_data->value_divide = CONVERT_M_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & AMK_HWM_DEBUG) {
				AKMDBG("Hwm get m-sensor data: %d, %d, %d. divide %d, status %d!\n",
				       msensor_data->values[0], msensor_data->values[1],
				       msensor_data->values[2], msensor_data->value_divide,
				       msensor_data->status);
			}
#endif
		}
		break;
	default:
		pr_err("msensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int akm09916_orientation_operate(void *self, uint32_t command, void *buff_in, int size_in,
				 void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *osensor_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & AMK_FUN_DEBUG)
		AKMFUNC("akm09916_orientation_operate");
#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 10)
				value = 10;
			akmd_delay = value;
		}
		break;

	case SENSOR_ENABLE:

		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {

			value = *(int *)buff_in;
			/* pr_err("akm09916_orientation_operate SENSOR_ENABLE=%d  mEnabled=%d\n",value,mEnabled); */

			if (mEnabled <= 0) {
				if (value == 1) {
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
			} else if (mEnabled == 1) {
				if (!value) {
					atomic_set(&o_flag, 0);
					if (atomic_read(&m_flag) == 0)
						atomic_set(&open_flag, 0);
				}
			}

			if (value) {
				mEnabled++;
				if (mEnabled > 32767)
					mEnabled = 32767;
			} else {
				mEnabled--;
				if (mEnabled < 0)
					mEnabled = 0;
			}
			wake_up(&open_wq);
		}

		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			pr_err("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			osensor_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			osensor_data->values[0] = sensor_data[13] * CONVERT_O;
			osensor_data->values[1] = sensor_data[14] * CONVERT_O;
			osensor_data->values[2] = sensor_data[15] * CONVERT_O;
			osensor_data->status = sensor_data[8];
			osensor_data->value_divide = CONVERT_O_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & AMK_HWM_DEBUG) {
				AKMDBG("Hwm get o-sensor data: %d, %d, %d. divide %d, status %d!\n",
				       osensor_data->values[0], osensor_data->values[1],
				       osensor_data->values[2], osensor_data->value_divide,
				       osensor_data->status);
			}
#endif
		}
		break;
	default:
		pr_err("gsensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

#ifdef AKM_Pseudogyro
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
int akm09916_gyroscope_operate(void *self, uint32_t command, void *buff_in, int size_in,
			       void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *gyrosensor_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & AMK_FUN_DEBUG)
		AKMFUNC("akm09916_gyroscope_operate");
#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;

			akmd_delay = 10;	/* fix to 100Hz */
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {

			value = *(int *)buff_in;
			/* pr_err("akm09916_gyroscope_operate SENSOR_ENABLE=%d  mEnabled=%d\n",value,mEnabled); */
			if (mEnabled <= 0) {
				if (value == 1) {
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
			} else if (mEnabled == 1) {
				if (!value) {
					atomic_set(&o_flag, 0);
					if (atomic_read(&m_flag) == 0)
						atomic_set(&open_flag, 0);
				}
			}

			if (value) {
				mEnabled++;
				if (mEnabled > 32767)
					mEnabled = 32767;
			} else {
				mEnabled--;
				if (mEnabled < 0)
					mEnabled = 0;
			}

			wake_up(&open_wq);
		}

		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			pr_err("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			gyrosensor_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			gyrosensor_data->values[0] = sensor_data[9] * CONVERT_Q16;
			gyrosensor_data->values[1] = sensor_data[10] * CONVERT_Q16;
			gyrosensor_data->values[2] = sensor_data[11] * CONVERT_Q16;
			gyrosensor_data->status = sensor_data[12];
			gyrosensor_data->value_divide = CONVERT_Q16_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & AMK_HWM_DEBUG) {
				AKMDBG
				    ("Hwm get gyro-sensor data: %d, %d, %d. divide %d, status %d!\n",
				     gyrosensor_data->values[0], gyrosensor_data->values[1],
				     gyrosensor_data->values[2], gyrosensor_data->value_divide,
				     gyrosensor_data->status);
			}
#endif
		}
		break;
	default:
		pr_err("gyrosensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int akm09916_rotation_vector_operate(void *self, uint32_t command, void *buff_in, int size_in,
				     void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *RV_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & AMK_FUN_DEBUG)
		AKMFUNC("akm09916_rotation_vector_operate");
#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			akmd_delay = 10;	/* fix to 100Hz */
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {

			value = *(int *)buff_in;

			if (mEnabled <= 0) {
				if (value == 1) {
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
			} else if (mEnabled == 1) {
				if (!value) {
					atomic_set(&o_flag, 0);
					if (atomic_read(&m_flag) == 0)
						atomic_set(&open_flag, 0);
				}
			}

			if (value) {
				mEnabled++;
				if (mEnabled > 32767)
					mEnabled = 32767;
			} else {
				mEnabled--;
				if (mEnabled < 0)
					mEnabled = 0;
			}
			wake_up(&open_wq);
		}

		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			pr_err("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			RV_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			RV_data->values[0] = sensor_data[22] * CONVERT_Q16;
			RV_data->values[1] = sensor_data[23] * CONVERT_Q16;
			RV_data->values[2] = sensor_data[24] * CONVERT_Q16;
			RV_data->status = 0;	/* sensor_data[19];  fix w-> 0 w */
			RV_data->value_divide = CONVERT_Q16_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & AMK_HWM_DEBUG) {
				AKMDBG
				    ("Hwm get rv-sensor data: %d, %d, %d. divide %d, status %d!\n",
				     RV_data->values[0], RV_data->values[1], RV_data->values[2],
				     RV_data->value_divide, RV_data->status);
			}
#endif
		}
		break;
	default:
		pr_err("RV  operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}


/*----------------------------------------------------------------------------*/
int akm09916_gravity_operate(void *self, uint32_t command, void *buff_in, int size_in,
			     void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *gravity_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & AMK_FUN_DEBUG)
		AKMFUNC("akm09916_gravity_operate");
#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 10)
				value = 10;
			akmd_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {

			value = *(int *)buff_in;

			if (mEnabled <= 0) {
				if (value == 1) {
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
			} else if (mEnabled == 1) {
				if (!value) {
					atomic_set(&o_flag, 0);
					if (atomic_read(&m_flag) == 0)
						atomic_set(&open_flag, 0);
				}
			}

			if (value) {
				mEnabled++;
				if (mEnabled > 32767)
					mEnabled = 32767;
			} else {
				mEnabled--;
				if (mEnabled < 0)
					mEnabled = 0;
			}
			wake_up(&open_wq);
		}

		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			pr_err("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			gravity_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			gravity_data->values[0] = sensor_data[16] * CONVERT_Q16;
			gravity_data->values[1] = sensor_data[17] * CONVERT_Q16;
			gravity_data->values[2] = sensor_data[18] * CONVERT_Q16;
			gravity_data->status = sensor_data[4];
			gravity_data->value_divide = CONVERT_Q16_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & AMK_HWM_DEBUG) {
				AKMDBG
				    ("Hwm get gravity-sensor data: %d, %d, %d. divide %d, status %d!\n",
				     gravity_data->values[0], gravity_data->values[1],
				     gravity_data->values[2], gravity_data->value_divide,
				     gravity_data->status);
			}
#endif
		}
		break;
	default:
		pr_err("gravity operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}


/*----------------------------------------------------------------------------*/
int akm09916_linear_accelration_operate(void *self, uint32_t command, void *buff_in, int size_in,
					void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data *LA_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct akm_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if (atomic_read(&data->trace) & AMK_FUN_DEBUG)
		AKMFUNC("akm09916_linear_accelration_operate");

#endif

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 10)
				value = 10;

			akmd_delay = value;
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			pr_err("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {

			value = *(int *)buff_in;

			if (mEnabled <= 0) {
				if (value == 1) {
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
			} else if (mEnabled == 1) {
				if (!value) {
					atomic_set(&o_flag, 0);
					if (atomic_read(&m_flag) == 0)
						atomic_set(&open_flag, 0);
				}
			}

			if (value) {
				mEnabled++;
				if (mEnabled > 32767)
					mEnabled = 32767;
			} else {
				mEnabled--;
				if (mEnabled < 0)
					mEnabled = 0;
			}
			wake_up(&open_wq);
		}

		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			pr_err("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			LA_data = (struct hwm_sensor_data *)buff_out;
			mutex_lock(&sensor_data_mutex);

			LA_data->values[0] = sensor_data[19] * CONVERT_Q16;
			LA_data->values[1] = sensor_data[20] * CONVERT_Q16;
			LA_data->values[2] = sensor_data[21] * CONVERT_Q16;
			LA_data->status = sensor_data[4];
			LA_data->value_divide = CONVERT_Q16_DIV;

			mutex_unlock(&sensor_data_mutex);
#if DEBUG
			if (atomic_read(&data->trace) & AMK_HWM_DEBUG) {
				AKMDBG
				    ("Hwm get LA-sensor data: %d, %d, %d. divide %d, status %d!\n",
				     LA_data->values[0], LA_data->values[1], LA_data->values[2],
				     LA_data->value_divide, LA_data->status);
			}
#endif
		}
		break;
	default:
		pr_err("linear_accelration operate function no this parameter %d!\n",
		       command);
		err = -1;
		break;
	}

	return err;
}

#endif

#ifndef	CONFIG_HAS_EARLYSUSPEND
#if 0
/*----------------------------------------------------------------------------*/
static int akm09916_suspend(struct i2c_client *client, pm_message_t msg)
{
	/* int err; */
	struct akm_i2c_data *obj = i2c_get_clientdata(client);


	if (msg.event == PM_EVENT_SUSPEND)
		akm09916_power(obj->hw, 0);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int akm09916_resume(struct i2c_client *client)
{
	/* int err; */
	struct akm_i2c_data *obj = i2c_get_clientdata(client);


	akm09916_power(obj->hw, 1);


	return 0;
}

/*----------------------------------------------------------------------------*/
#endif
#else				/*CONFIG_HAS_EARLY_SUSPEND is defined */
/*----------------------------------------------------------------------------*/
static void akm09916_early_suspend(struct early_suspend *h)
{
	struct akm_i2c_data *obj = container_of(h, struct akm_i2c_data, early_drv);
	int err = 0;

	if (NULL == obj) {
		pr_err("null pointer!!\n");
		return;
	}

	err = AKECS_SetMode(AK09916_MODE_POWERDOWN);
	if (err < 0) {
		AKMDBG("%s:%d Error.\n", __func__, __LINE__);
		return;
	}

	akm09916_power(obj->hw, 0);
}

/*----------------------------------------------------------------------------*/
static void akm09916_late_resume(struct early_suspend *h)
{
	struct akm_i2c_data *obj = container_of(h, struct akm_i2c_data, early_drv);
	int err;


	if (NULL == obj) {
		pr_err("null pointer!!\n");
		return;
	}
	akm09916_power(obj->hw, 1);

	err = AKECS_SetMode(AK09916_MODE_SNG_MEASURE);
	if (err < 0) {
		AKMDBG("%s:%d Error.\n", __func__, __LINE__);
		return;
	}
}

/*----------------------------------------------------------------------------*/
#endif				/*CONFIG_HAS_EARLYSUSPEND */
/*----------------------------------------------------------------------------*/
static int akm09916_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, AKM09916_DEV_NAME);
	return 0;
}

static ssize_t ecompass_status_read_proc(struct file *file, char *buffer, size_t count,
					 loff_t *ppos)
{
	char *page = NULL;
	char *ptr = NULL;
	int len = 0;
	int err = -1;

	page = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!page) {
		kfree(page);
		return -ENOMEM;
	}
	ptr = page;

	ptr += sprintf(ptr, "%d\n", ecompass_status);

	len = ptr - page;
	if (*ppos >= len) {
		kfree(page);
		return 0;
	}
	err = copy_to_user(buffer, (char *)page, len);
	*ppos += len;
	if (err) {
		kfree(page);
		return err;
	}
	kfree(page);
	return len;
}

static const struct file_operations akm09916_ecompass_status_proc_fops = {
	.read = ecompass_status_read_proc
};


/*----------------------------------------------------------------------------*/
static int akm09916_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct akm_i2c_data *data;
	int err = 0;
	struct hwmsen_object sobj_m, sobj_o;
	/* struct hwmsen_object sobj_gyro, sobj_rv; */
	/* struct hwmsen_object sobj_gravity, sobj_la; */
	/* struct mag_control_path ctl = {0}; */
	/* struct mag_data_path mag_data = {0}; */

	data = kmalloc(sizeof(struct akm_i2c_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct akm_i2c_data));
	data->hw = get_cust_mag();

	atomic_set(&data->layout, hw->direction);
	atomic_set(&data->trace, 0);

	mutex_init(&sense_data_mutex);
	mutex_init(&sensor_data_mutex);

	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	this_client = new_client;

	pr_err(" AKM09916 akm09916_probe: before init prob\n");
	/* Check connection */
	err = AKECS_CheckDevice();
	if (err < 0) {
		pr_err("AKM09916 akm09916_probe: check device connect error\n");
		goto exit_init_failed;
	}


	/* Register sysfs attribute */
	err = akm09916_create_attr(&akm09916_init_info.platform_diver_addr->driver);
	if (err) {
		pr_err("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	err = misc_register(&akm09916_device);
	if (err) {
		pr_err("akm09916_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	sobj_m.self = data;
	sobj_m.polling = 1;
	sobj_m.sensor_operate = akm09916_operate;

	err = hwmsen_attach(ID_MAGNETIC, &sobj_m);
	if (err) {
		pr_err("attach fail = %d\n", err);
		goto exit_kfree;
	}

	sobj_o.self = data;
	sobj_o.polling = 1;
	sobj_o.sensor_operate = akm09916_orientation_operate;

	err = hwmsen_attach(ID_ORIENTATION, &sobj_o);
	if (err) {
		pr_err("attach fail = %d\n", err);
		goto exit_kfree;
	}
#ifdef AKM_Pseudogyro
	/* pseudo gyro sensor */
	sobj_gyro.self = data;
	sobj_gyro.polling = 1;
	sobj_gyro.sensor_operate = akm09916_gyroscope_operate;

	err = hwmsen_attach(ID_GYROSCOPE, &sobj_gyro);
	if (err) {
		pr_err("attach fail = %d\n", err);
		goto exit_kfree;
	}
	/* rotation vector sensor */

	sobj_rv.self = data;
	sobj_rv.polling = 1;
	sobj_rv.sensor_operate = akm09916_rotation_vector_operate;

	err = hwmsen_attach(ID_ROTATION_VECTOR, &sobj_rv);
	if (err) {
		pr_err("attach fail = %d\n", err);
		goto exit_kfree;
	}
	/* Gravity sensor */

	sobj_gravity.self = data;
	sobj_gravity.polling = 1;
	sobj_grvity.sensor_operate = akm09916_gravity_operate;

	err = hwmsen_attach(ID_GRAVITY, &sobj_gravity);
	if (err) {
		pr_err("attach fail = %d\n", err);
		goto exit_kfree;
	}
	/* LINEAR_ACCELERATION sensor */

	sobj_la.self = data;
	sobj_la.polling = 1;
	sobj_la.sensor_operate = akm09916_linear_accelration_operate;

	err = hwmsen_attach(ID_LINEAR_ACCELERATION, &sobj_la);
	if (err) {
		/* pr_err("attach fail = %d\n", err); */
		goto exit_kfree;
	}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_drv.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	    data->early_drv.suspend = akm09916_early_suspend,
	    data->early_drv.resume = akm09916_late_resume, register_early_suspend(&data->early_drv);
#endif

	pr_err("%s: OK\n", __func__);
	ecompass_status = 1;
	return 0;

exit_sysfs_create_group_failed:
exit_init_failed:
exit_misc_device_register_failed:
exit_kfree:
	kfree(data);
exit:
	pr_err("%s: err = %d\n", __func__, err);
	ecompass_status = 0;
	return err;
}

/*----------------------------------------------------------------------------*/
static int akm09916_i2c_remove(struct i2c_client *client)
{
	int err;

	err = akm09916_delete_attr(&akm09916_init_info.platform_diver_addr->driver);

	if (err)
		pr_err("akm09916_delete_attr fail: %d\n", err);

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	misc_deregister(&akm09916_device);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int akm09916_local_init(void)
{

	akm09916_power(hw, 1);

	atomic_set(&dev_open_count, 0);
	/* akm09916_force[0] = hw->i2c_num; */

	if (i2c_add_driver(&akm09916_i2c_driver)) {
		pr_err("add driver error\n");
		return -1;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int akm09916_remove(void)
{

	akm09916_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&akm09916_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init akm09916_init(void)
{
	const char *name = "mediatek,akm09916";

	hw = get_mag_dts_func(name, hw);
	if (!hw)
		MAG_ERR("get dts info!!!\n");

	mag_driver_add(&akm09916_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit akm09916_exit(void)
{
/* platform_driver_unregister(&akm_sensor_driver); */
}

/*----------------------------------------------------------------------------*/
module_init(akm09916_init);
module_exit(akm09916_exit);

MODULE_AUTHOR("viral wang");
MODULE_DESCRIPTION("AKM09916 compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
