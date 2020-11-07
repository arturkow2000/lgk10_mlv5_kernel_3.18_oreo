/* 
 * ICM406XX sensor driver
 * Copyright (C) 2018 Invensense, Inc.
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

#include <linux/ioctl.h>
#include "cust_gyro.h"
#include "gyroscope.h"
#include "../../accelerometer/icm406xx_a/icm406xx_register.h"
#include "../../accelerometer/icm406xx_a/icm406xx_share.h"
#include "icm406xx_gyro.h"

#define ICM406XX_GYRO_DEV_NAME "ICM406XX_GYRO"

struct icm406xx_gyro_i2c_data {
    struct i2c_client *client;
    struct gyro_hw *hw;
    struct hwmsen_convert cvt;
    /*misc*/
    atomic_t trace;
    atomic_t suspend;
    atomic_t selftest;	
    atomic_t is_enabled;
    /*data*/
    s16 cali_sw[ICM406XX_AXIS_NUM+1];
    s16 data[ICM406XX_AXIS_NUM+1];
};

static int icm406xx_gyro_init_flag =  -1;

static struct i2c_client *icm406xx_gyro_i2c_client;
static struct icm406xx_gyro_i2c_data *obj_i2c_data;

#ifdef ICM406XX_SELFTEST
static char selftestRes[8] = { 0 };
#define SELF_TEST_GYR_BW_IND        BIT_GYRO_UI_LNM_BW_10_IIR
#endif

/* +/-1000DPS as default */
static int g_icm406xx_gyro_sensitivity = ICM406XX_GYRO_DEFAULT_SENSITIVITY;

static struct gyro_hw gyro_cust;
static struct gyro_hw *hw = &gyro_cust;

static int icm406xx_gyro_local_init(struct platform_device *pdev);
static int icm406xx_gyro_remove(void);
static struct gyro_init_info icm406xx_gyro_init_info = {
    .name = ICM406XX_GYRO_DEV_NAME,
    .init = icm406xx_gyro_local_init,
    .uninit = icm406xx_gyro_remove,
};

static int icm406xx_gyro_SetFullScale(struct i2c_client *client, u8 gyro_fsr)
{
    u8 databuf[2] = {0};
    int res = 0;
        
    res = icm406xx_share_read_register(REG_GYRO_CONFIG0, databuf, 1);
    if (res < 0) {
        GYRO_ERR("read fsr register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    /* clear FSR bit */
    databuf[0] &= ~BIT_GYRO_FSR;
    databuf[0] |= gyro_fsr << SHIFT_GYRO_FS_SEL;
    g_icm406xx_gyro_sensitivity = (ICM406XX_GYRO_MAX_SENSITIVITY >> (3 - gyro_fsr)) + 1;   
    res = icm406xx_share_write_register(REG_GYRO_CONFIG0, databuf, 1);
    if (res < 0){
        GYRO_ERR("write fsr register err!\n");
        return ICM406XX_ERR_BUS;
    }
    return ICM406XX_SUCCESS;
}


static int icm406xx_gyro_SetFilter(struct i2c_client *client, u8 gyro_filter)
{
    u8 databuf[2] = {0};
    int res = 0;

    res = icm406xx_share_read_register(REG_GYRO_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
        GYRO_ERR("read filter register err!\n");
        return ICM406XX_ERR_BUS;
    }
    /* clear filter bit */
    databuf[0] &= ~BIT_GYRO_FILTER;
    databuf[0] |= gyro_filter;
    res = icm406xx_share_write_register(REG_GYRO_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
        GYRO_ERR("write filter register err!\n");
        return ICM406XX_ERR_BUS;
    }
    return ICM406XX_SUCCESS;    
}

static int icm406xx_gyro_ReadSensorDataDirect(struct i2c_client *client,
    s16 data[ICM406XX_AXIS_NUM])
{
    char databuf[6];
    int i;
    int res = 0;

    if (client == NULL) 
        return ICM406XX_ERR_INVALID_PARAM;
    res = icm406xx_share_read_register(REG_GYRO_DATA_X0_UI, databuf,
        ICM406XX_DATA_LEN);
    if (res < 0) {
        GYRO_ERR("read gyroscope data error\n");
        return ICM406XX_ERR_BUS ;
    }
    /* convert 8-bit to 16-bit */
    for (i = 0; i < ICM406XX_AXIS_NUM; i++)
        data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
    return ICM406XX_SUCCESS;
}

static int icm406xx_gyro_ReadSensorData(struct i2c_client *client,
    char *buf, int bufsize)
{
    char databuf[6];
    int  data[3];
    int  i = 0;
    int res = 0;
    struct icm406xx_gyro_i2c_data * obj = i2c_get_clientdata(client);

    if (client == NULL)
        return ICM406XX_ERR_INVALID_PARAM;
    res = icm406xx_share_read_register(REG_GYRO_DATA_X0_UI, databuf,
        ICM406XX_DATA_LEN);
    if (res < 0) {
        GYRO_ERR("read gyroscope data error\n");
        return ICM406XX_ERR_BUS;
    }
    /* convert 8-bit to 16-bit */
    for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
        obj->data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
        /* add calibration value */
        obj->data[i] += obj->cali_sw[i];
    }
    /* orientation translation lower (sensor) 
    --> upper (device) & unit translation */
    for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
        data[i] = obj->cvt.sign[i] * obj->data[obj->cvt.map[i]];
        data[i] = data[i] * ICM406XX_GYRO_MAX_SENSITIVITY  
            / g_icm406xx_gyro_sensitivity;
    }
    sprintf(buf, "%04x %04x %04x",
        data[ICM406XX_AXIS_X], 
        data[ICM406XX_AXIS_Y], 
        data[ICM406XX_AXIS_Z]);
    if (atomic_read(&obj->trace)) {
        GYRO_LOG("Gyroscope data - %04x %04x %04x\n",
            data[ICM406XX_AXIS_X],
            data[ICM406XX_AXIS_Y],
            data[ICM406XX_AXIS_Z]);
    }
    return ICM406XX_SUCCESS;
}

static int icm406xx_gyro_ReadRawData(struct i2c_client * client, char * buf)
{	
    int res = 0;
    s16 data[ICM406XX_AXIS_NUM] = { 0, 0, 0 };
	
    res = icm406xx_gyro_ReadSensorDataDirect(client, data);
    if (res < 0) {
        GYRO_ERR("read gyroscope raw data  error\n");
        return ICM406XX_ERR_BUS;
    }
    /* sensor raw data direct read from sensor register	
    no orientation translation, no unit translation */
    sprintf(buf, "%04x %04x %04x",
        data[ICM406XX_AXIS_X],
        data[ICM406XX_AXIS_Y],
        data[ICM406XX_AXIS_Z]);
    return ICM406XX_SUCCESS;
}

static int icm406xx_gyro_ResetCalibration(struct i2c_client *client)
{
    //struct icm406xx_gyro_i2c_data *obj = i2c_get_clientdata(client);
    struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;

    memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
    return ICM406XX_SUCCESS;
}

static int icm406xx_gyro_ReadCalibration(struct i2c_client *client,
    struct SENSOR_DATA *sensor_data)
{
    //struct icm406xx_gyro_i2c_data *obj = i2c_get_clientdata(client);
    struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;
    int cali[ICM406XX_AXIS_NUM];
    int i;

    for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
        /* remap to device orientation */
        cali[i] = obj->cvt.sign[i]*obj->cali_sw[obj->cvt.map[i]];
    }
    sensor_data->x = cali[ICM406XX_AXIS_X] * ICM406XX_GYRO_MAX_SENSITIVITY /
        g_icm406xx_gyro_sensitivity;
    sensor_data->y = cali[ICM406XX_AXIS_Y] * ICM406XX_GYRO_MAX_SENSITIVITY /
        g_icm406xx_gyro_sensitivity;
    sensor_data->z = cali[ICM406XX_AXIS_Z] * ICM406XX_GYRO_MAX_SENSITIVITY /
        g_icm406xx_gyro_sensitivity;
    if (atomic_read(&obj->trace)) {
        GYRO_LOG("Gyro ReadCalibration:[sensor_data:%5d %5d %5d]\n",
            sensor_data->x, sensor_data->y, sensor_data->z);
        GYRO_LOG("Gyro ReadCalibration:[cali_sw:%5d %5d %5d]\n",
            obj->cali_sw[ICM406XX_AXIS_X], 
            obj->cali_sw[ICM406XX_AXIS_Y], 
            obj->cali_sw[ICM406XX_AXIS_Z]);
    }
	return ICM406XX_SUCCESS;
}

static int icm406xx_gyro_WriteCalibration(struct i2c_client *client,
    struct SENSOR_DATA *sensor_data)
{
    //struct icm406xx_gyro_i2c_data *obj = i2c_get_clientdata(client);
    struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;
    int cali[ICM406XX_AXIS_NUM];
    int i;

    /* upper layer (HAL) unit translation into lower layer (sensor) unit */
    cali[ICM406XX_AXIS_X] = sensor_data->x * g_icm406xx_gyro_sensitivity /
        ICM406XX_GYRO_MAX_SENSITIVITY;
    cali[ICM406XX_AXIS_Y] = sensor_data->y * g_icm406xx_gyro_sensitivity /
        ICM406XX_GYRO_MAX_SENSITIVITY;
    cali[ICM406XX_AXIS_Z] = sensor_data->z * g_icm406xx_gyro_sensitivity /
        ICM406XX_GYRO_MAX_SENSITIVITY;
    /* orientation translation */
    for (i = 0; i < ICM406XX_AXIS_NUM; i++)
        obj->cali_sw[obj->cvt.map[i]] += obj->cvt.sign[i] * cali[i];
    if (atomic_read(&obj->trace)) {
        GYRO_LOG("Gyro writeCalibration:[sensor_data:%5d %5d %5d]\n",
            sensor_data->x, sensor_data->y, sensor_data->z);
        GYRO_LOG("Gyro writeCalibration:[cali_sw:%5d %5d %5d]\n",
            obj->cali_sw[ICM406XX_AXIS_X], 
            obj->cali_sw[ICM406XX_AXIS_Y], 
            obj->cali_sw[ICM406XX_AXIS_Z]);
    }
    return ICM406XX_SUCCESS;
}

#ifdef ICM406XX_SELFTEST
static int icm406xx_gyro_ResetOffsetReg(void)
{
    int res = 0;
    char databuf[2] = {0};

    databuf[0] = 0x00;
    res = icm406xx_share_write_register(REG_GOS_USER0, databuf, 1);
    if (res < 0) {
        GYRO_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    res = icm406xx_share_write_register(REG_GOS_USER1, databuf, 1);
    if (res < 0) {
        GYRO_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    res = icm406xx_share_write_register(REG_GOS_USER2, databuf, 1);
    if (res < 0) {
        GYRO_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    res = icm406xx_share_write_register(REG_GOS_USER3, databuf, 1);
    if (res < 0) {
        GYRO_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    res = icm406xx_share_read_register(REG_GOS_USER4, 
        databuf, 1);
    if (res) {
        GYRO_ERR("read offset fail: %d\n", res);
        return false;
    }
    databuf[0] &= 0xf0;
    res = icm406xx_share_write_register(REG_GOS_USER4, databuf, 1);
    if (res < 0) {
        GYRO_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    return ICM406XX_SUCCESS;
}

static int  icm406xx_gyro_InitSelfTest(struct i2c_client * client)
{
    int res = 0;

    /* softreset */
    res = icm406xx_share_ChipSoftReset();
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set power mode */
    icm406xx_share_set_sensor_power_mode(ICM406XX_SENSOR_TYPE_GYRO,
        BIT_GYRO_MODE_LNM);    
    /* reset offset*/
    res = icm406xx_gyro_ResetOffsetReg();
    if (res != ICM406XX_SUCCESS)
        return res;        
    /* setpowermode(true) --> exit sleep */
    res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_GYRO, true);
    if (res != ICM406XX_SUCCESS)
        return res;
	/* fsr : ICM406XX_GYRO_RANGE_250DPS */
    res = icm406xx_gyro_SetFullScale(client, ICM406XX_GYRO_RANGE_250DPS);	
    if (res != ICM406XX_SUCCESS)
        return res;
    /* filter : SELF_TEST_GYR_BW_IND */
    res = icm406xx_gyro_SetFilter(client, SELF_TEST_GYR_BW_IND);
    if (res != ICM406XX_SUCCESS)
        return res;		
    /* odr : 1000hz (1kHz) */
    res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_GYRO,
        1000000, true);			
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set enable sensor */
    res = icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_GYRO, true);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* delay for selftest */
    mdelay(SELF_TEST_READ_INTERVAL_MS);
    return res;
}

static int icm406xx_gyro_CalcAvgWithSamples(struct i2c_client *client,
    int avg[3], int count)
{
    int res = 0;
    int i, nAxis;
    s16 sensor_data[ICM406XX_AXIS_NUM];
    s32 sum[ICM406XX_AXIS_NUM] = {0,};

    for (i = 0; i < count; i++) {
        res = icm406xx_gyro_ReadSensorDataDirect(client, sensor_data);
        if (res) {
            GYRO_ERR("read data fail: %d\n", res);
            return ICM406XX_ERR_STATUS;
        }	
        for (nAxis = 0; nAxis < ICM406XX_AXIS_NUM; nAxis++)
            sum[nAxis] += sensor_data[nAxis];
        /* data register updated @1khz */
        mdelay(1);
    }
    for (nAxis = 0; nAxis < ICM406XX_AXIS_NUM; nAxis++)
        avg[nAxis] = (int)(sum[nAxis] / count) * SELF_TEST_PRECISION;
    return ICM406XX_SUCCESS;
}

static bool icm406xx_gyro_DoSelfTest(struct i2c_client *client)
{
    int res = 0;
    int i;
    int gyro_ST_on[ICM406XX_AXIS_NUM], gyro_ST_off[ICM406XX_AXIS_NUM];
    /* index of otp_lookup_tbl */
    u8  st_code[ICM406XX_AXIS_NUM];
    u16 st_otp[ICM406XX_AXIS_NUM];
    bool otp_value_has_zero = false;
    bool test_result = true;
    u8 databuf[2] = {0};
    int st_res;
    int retry;

    databuf[0] = BIT_BANK_SEL_1;
    res = icm406xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    if (res < 0) {
        GYRO_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    /* acquire OTP value from OTP lookup table */
    res = icm406xx_share_read_register(REG_XG_ST_DATA, 
        &(st_code[0]), 3);
    if (res) {
        GYRO_ERR("read data fail: %d\n", res);
        return false;
    }
    GYRO_LOG("st_code: %02x, %02x, %02x\n",
        st_code[0], st_code[1], st_code[2]);
    databuf[0] = BIT_BANK_SEL_0;
    res = icm406xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    if (res < 0) {
        GYRO_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    /* lookup OTP value with st_code */
    for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
        if (st_code[i] != 0)
            st_otp[i] = st_otp_lookup_tbl[ st_code[i] - 1];
        else {
            st_otp[i] = 0;
            otp_value_has_zero = true;
        }
    }
    /* read sensor data and calculate average values from it */
    for (retry = 0 ; retry < RETRY_CNT_SELF_TEST ; retry++ ) {
        /* read 200 samples with selftest off */
        res = icm406xx_gyro_CalcAvgWithSamples(client, gyro_ST_off,
            SELF_TEST_SAMPLE_NB);
        if (res) {
            GYRO_ERR("read data fail: %d\n", res);
            return false;
        }
        /* set selftest on */
        databuf[0] = (BIT_TEST_GX_EN | BIT_TEST_GY_EN | BIT_TEST_GZ_EN);
        res = icm406xx_share_write_register(REG_SELF_TEST_CONFIG, databuf, 1);
        if (res) {
            GYRO_ERR("enable st gyro fail: %d\n", res);
            return false;
        }
        /* wait 20ms for oscillations to stabilize */
        mdelay(20);
        /* Read 200 Samples with selftest on */
        res = icm406xx_gyro_CalcAvgWithSamples(client, gyro_ST_on,
            SELF_TEST_SAMPLE_NB);
        if (res) {
            GYRO_ERR("read data fail: %d\n", res);
            return false;
        }
        /* set selftest off */
        databuf[0] = 0x00;
        res = icm406xx_share_write_register(REG_SELF_TEST_CONFIG, databuf, 1);
        if (res) {
            GYRO_ERR("disable st gyro fail: %d\n", res);
            return false;
        }        
        /* wait 20ms for oscillations to stabilize */
        mdelay(20);
        /* compare calculated value with OTP value to judge success or fail */
        if (!otp_value_has_zero) {
            /* criteria a */
            for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
                st_res = gyro_ST_on[i] - gyro_ST_off[i];
                if (st_res <= st_otp[i] * SELF_TEST_GYR_SHIFT_DELTA) {
                    GYRO_LOG("error gyro[%d] : st_res = %d, st_otp = %d\n",
                        i, st_res, st_otp[i]);
                    test_result = false;
                }
            }
        } else {
            /* criteria b */
            for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
                st_res = abs(gyro_ST_on[i] - gyro_ST_off[i]);
                if (st_res < SELF_TEST_MIN_GYR) {
                    GYRO_LOG("error gyro[%d] : st_res = %d, min = %d\n",
                        i, st_res, SELF_TEST_MIN_GYR);
                    test_result = false;
                }
            }
        }
        if (test_result) {
            /* criteria c */
            for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
                if (abs(gyro_ST_off[i]) > SELF_TEST_MAX_GYR_OFFSET) {
                    GYRO_LOG("error gyro[%d] = %ld, max = %d\n",
                        i, abs(gyro_ST_off[i]), SELF_TEST_MAX_GYR_OFFSET);
                    test_result = false;
                }
            }
        }
        if (test_result)
            break;
    }
    return test_result;
}
#endif

static int icm406xx_gyro_init_client(struct i2c_client *client, bool enable)
{
    int res = 0;
    u8 databuf[2] = {0};

    /* xxit sleep mode */
    res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_GYRO, true);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set fsr +/-1000 dps as default */
    res = icm406xx_gyro_SetFullScale(client, ICM406XX_GYRO_RANGE_1000DPS);	
    if (res != ICM406XX_SUCCESS)
        return res;
#ifdef ICM406XX_ACCEL_LOW_POWER_MODE
    /* set power mode */
    icm406xx_share_set_sensor_power_mode(ICM406XX_SENSOR_TYPE_GYRO,
        BIT_GYRO_MODE_LPM);
    /* set filter BIT_ACCEL_UI_LPM_BW_2_FIR as default */
    res = icm406xx_gyro_SetFilter(client, BIT_GYRO_UI_LPM_AVG_1);
    if (res != ICM406XX_SUCCESS)
        return res;
#else
    /* set power mode */
    icm406xx_share_set_sensor_power_mode(ICM406XX_SENSOR_TYPE_GYRO,
        BIT_GYRO_MODE_LNM);
    /* set filter BIT_ACCEL_UI_LNM_BW_2_FIR as default */
    res = icm406xx_gyro_SetFilter(client, BIT_GYRO_UI_LNM_BW_2_FIR);
    if (res != ICM406XX_SUCCESS)
        return res;
#endif
    /* set filter BIT_GYRO_UI_LNM_BW_2_FIR as default */
    res = icm406xx_gyro_SetFilter(client, BIT_GYRO_UI_LNM_BW_2_FIR);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set 5ms(200hz) sample rate */
    res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_GYRO,
        5000000, false);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set gyro config1 as default */    
    databuf[0] = 0x1A;
    res = icm406xx_share_write_register(REG_GYRO_CONFIG1, databuf, 1);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* disable sensor - standby mode for gyroscope */
    res = icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_GYRO, enable);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set power mode - sleep or normal */
    res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_GYRO, enable);
    if (res != ICM406XX_SUCCESS)
        return res;
    GYRO_LOG("icm406xx_gyro_init_client OK!\n");
    return ICM406XX_SUCCESS;
}

struct gyro_hw *get_cust_gyro(void)
{
    return &gyro_cust;
}

static void icm406xx_gyro_power(struct gyro_hw *hw, unsigned int on)
{
    /* nothing to do here, because the power of sensor is always on */
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    char strbuf[ICM406XX_BUFSIZE];

    icm406xx_share_ReadChipInfo(strbuf, ICM406XX_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
    char strbuf[ICM406XX_BUFSIZE];
    struct i2c_client *client = icm406xx_gyro_i2c_client;

    if (NULL == client) {
        GYRO_ERR("i2c client is null!!\n");
        return 0;
    }
    icm406xx_gyro_ReadSensorData(client, strbuf, ICM406XX_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    ssize_t res = 0;
    struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GYRO_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
    return res;
}

static ssize_t store_trace_value(struct device_driver *ddri,
    const char *buf, size_t count)
{
    int trace;
    struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GYRO_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    if (0 == kstrtoint(buf, 16, &trace))
        atomic_set(&obj->trace, trace);
    else
        GYRO_ERR("invalid content: '%s', length = %zu\n", buf, count);
    return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GYRO_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    if (obj->hw)
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
            obj->hw->i2c_num,
            obj->hw->direction,
            obj->hw->power_id,
            obj->hw->power_vol);
    else
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    return len;
}

static ssize_t show_chip_orientation(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GYRO_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    GYRO_LOG("[%s] default direction: %d\n", __func__, obj->hw->direction);
    len = snprintf(buf, PAGE_SIZE, "default direction = %d\n",
        obj->hw->direction);
    return len;
}

static ssize_t store_chip_orientation(struct device_driver *ddri,
    const char *buf, size_t tCount)
{
    int nDirection = 0;
    int res = 0;
    struct icm406xx_gyro_i2c_data   *obj = obj_i2c_data;

    if (obj == NULL) {
        GYRO_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    res = kstrtoint(buf, 10, &nDirection);
    if (res != 0) {
        if (hwmsen_get_convert(nDirection, &obj->cvt))
            GYRO_ERR("ERR: fail to set direction\n");
    }
    GYRO_LOG("[%s] set direction: %d\n", __func__, nDirection);
    return tCount;
}

#ifdef ICM406XX_SELFTEST
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = icm406xx_gyro_i2c_client;

    if (NULL == client) {
        GYRO_ERR("i2c client is null!!\n");
        return 0;
    }
    return snprintf(buf, 8, "%s\n", selftestRes);
}

static ssize_t store_selftest_value(struct device_driver *ddri,
    const char *buf, size_t count)
{
    struct i2c_client *client = icm406xx_gyro_i2c_client;
    int num;
    int res = 0;

    /* check parameter values to run selftest */
    res = kstrtoint(buf, 10, &num);
    if (res != 0) {
        GYRO_ERR("parse number fail\n");
        return count;
    } else if (num == 0) {
        GYRO_ERR("invalid data count\n");
        return count;
    }
    /* run selftest */
    res = icm406xx_gyro_InitSelfTest(client);
    if (icm406xx_gyro_DoSelfTest(client) == true) {
        strcpy(selftestRes, "y");
        GYRO_LOG("GYRO SELFTEST : PASS\n");
    } else {
        strcpy(selftestRes, "n");
        GYRO_LOG("GYRO SELFTEST : FAIL\n");
    }
    /* selftest is considered to be called only in factory mode
    in general mode, the condition before selftest will not be recovered
    and sensor will not be in sleep mode */
    res = icm406xx_gyro_init_client(client, true);		
    return count;
}
#endif

static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(trace,  S_IWUSR | S_IRUGO, show_trace_value,
    store_trace_value);
static DRIVER_ATTR(status,  S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation,
    store_chip_orientation);
#ifdef ICM406XX_SELFTEST
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, show_selftest_value,
    store_selftest_value);
#endif

static struct driver_attribute *icm406xx_gyro_attr_list[] = {
    /* chip information - whoami */
    &driver_attr_chipinfo,
    /* dump sensor data */
    &driver_attr_sensordata,
    /* trace log */
    &driver_attr_trace,
    /* chip status */
    &driver_attr_status,
    /* chip orientation information */
    &driver_attr_orientation,
#ifdef ICM406XX_SELFTEST
    /* run selftest when store, report selftest result when show */
    &driver_attr_selftest,
#endif
};

static int icm406xx_gyro_create_attr(struct device_driver *driver)
{
    int idx;
    int res = 0;
    int num = (int)(sizeof(icm406xx_gyro_attr_list)/
        sizeof(icm406xx_gyro_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;
    for (idx = 0; idx < num; idx++) {
        res = driver_create_file(driver, icm406xx_gyro_attr_list[idx]);
        if (0 != res) {
            GYRO_ERR("driver_create_file (%s) = %d\n",
                icm406xx_gyro_attr_list[idx]->attr.name, res);
            break;
        }
    }
    return res;
}

static int icm406xx_gyro_delete_attr(struct device_driver *driver)
{
    int idx;
    int res = 0;
    int num = (int)(sizeof(icm406xx_gyro_attr_list)/
        sizeof(icm406xx_gyro_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;
    for (idx = 0; idx < num; idx++)
        driver_remove_file(driver, icm406xx_gyro_attr_list[idx]);
    return res;
}

static int icm406xx_gyro_open(struct inode *inode, struct file *file)
{
    file->private_data = icm406xx_gyro_i2c_client;

    if (file->private_data == NULL) {
        GYRO_ERR("null pointer!!\n");
        return -EINVAL;
    }	
    return nonseekable_open(inode, file);
}

static int icm406xx_gyro_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static long icm406xx_gyro_unlocked_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;
    char strbuf[ICM406XX_BUFSIZE] = {0};
    void __user *data;
    long res = 0;
    int copy_cnt = 0;
    struct SENSOR_DATA sensor_data;
    int smtRes = 0;

	if (obj == NULL)
		return -EFAULT;

    if (_IOC_DIR(cmd) & _IOC_READ)
        res = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        res = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (res) {
        GYRO_ERR("access error: %08X, (%2d, %2d)\n",
            cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }
    switch (cmd) {
    case GYROSCOPE_IOCTL_INIT:
        icm406xx_gyro_init_client(client, true);
        break;
    case GYROSCOPE_IOCTL_SMT_DATA:
        data = (void __user *) arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        GYRO_LOG("ioctl smtRes: %d!\n", smtRes);
        copy_cnt = copy_to_user(data, &smtRes,  sizeof(smtRes));
        if (copy_cnt) {
            res = -EFAULT;
            GYRO_ERR("copy gyro data to user failed!\n");
        }
        GYRO_LOG("copy gyro data to user OK: %d!\n", copy_cnt);
        break;
    case GYROSCOPE_IOCTL_READ_SENSORDATA:
        data = (void __user *) arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm406xx_gyro_ReadSensorData(client, strbuf, ICM406XX_BUFSIZE);
        if (copy_to_user(data, strbuf, sizeof(strbuf))) {
            res = -EFAULT;
            break;
        }
        break;
    case GYROSCOPE_IOCTL_SET_CALI:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        if (copy_from_user(&sensor_data, data, sizeof(sensor_data)))
            res = -EFAULT;
        else {
            GYRO_LOG("gyro set cali:[%5d %5d %5d]\n",
                sensor_data.x, sensor_data.y, sensor_data.z);
            res = icm406xx_gyro_WriteCalibration(client, &sensor_data);
        }
        break;
    case GYROSCOPE_IOCTL_GET_CALI:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        res = icm406xx_gyro_ReadCalibration(client, &sensor_data);
        if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
            res = -EFAULT;
            break;
        }
        break;
    case GYROSCOPE_IOCTL_CLR_CALI:
        res = icm406xx_gyro_ResetCalibration(client);
        break;
    case GYROSCOPE_IOCTL_READ_SENSORDATA_RAW: 
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm406xx_gyro_ReadRawData(client, strbuf);
        if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
            res = -EFAULT;
            break;
        }
        break;
    default:
        GYRO_ERR("unknown IOCTL: 0x%08x\n", cmd);
        res = -ENOIOCTLCMD;
    }
    return res;
}

#ifdef CONFIG_COMPAT
static long icm406xx_gyro_compat_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    long res = 0;
    void __user *arg32 = compat_ptr(arg);

    if(!file->f_op || !file->f_op->unlocked_ioctl){
        GYRO_ERR("compat_ion_ioctl file has no f_op\n");
        GYRO_ERR("or no f_op->unlocked_ioctl.\n");
        return -ENOTTY;
    }
    switch (cmd) {
    case COMPAT_GYROSCOPE_IOCTL_INIT:
    case COMPAT_GYROSCOPE_IOCTL_SMT_DATA:
    case COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA:
    case COMPAT_GYROSCOPE_IOCTL_SET_CALI:
    case COMPAT_GYROSCOPE_IOCTL_GET_CALI:
    case COMPAT_GYROSCOPE_IOCTL_CLR_CALI:
    case COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA_RAW:
    case COMPAT_GYROSCOPE_IOCTL_READ_TEMPERATURE:
    case COMPAT_GYROSCOPE_IOCTL_GET_POWER_STATUS:
        if (arg32 == NULL) {
            GYRO_ERR("invalid argument.");
            return -EINVAL;
        }
        res = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
        break;
    default:
        GYRO_ERR("%s not supported = 0x%04x\n", __func__, cmd);
        res = -ENOIOCTLCMD;
        break;
    }
    return res;
}
#endif

static const struct file_operations icm406xx_gyro_fops = {
    .open = icm406xx_gyro_open,
    .release = icm406xx_gyro_release,
    .unlocked_ioctl = icm406xx_gyro_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = icm406xx_gyro_compat_ioctl,
#endif
};

static struct miscdevice icm406xx_gyro_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gyroscope",
    .fops = &icm406xx_gyro_fops,
};

static int icm406xx_gyro_open_report_data(int open)
{
    /* nothing to do here for 406xx */
    return 0;
}

static int icm406xx_gyro_enable_nodata(int en)
{
    /* if use this type of enable , 
    gsensor only enabled but not report inputEvent to HAL */
    int res = 0;
    bool power = false;
    struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;

    if (1 == en) {
        power = true;
        res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_GYRO, power);
        res |= icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_GYRO, power);
    } else {
        power = false;
        res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_GYRO,
            0, false);
        res |= icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_GYRO, power);
        res |= icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_GYRO, power);
    }
    if (res != ICM406XX_SUCCESS) {
        GYRO_ERR("icm406xx_gyro_SetPowerMode fail!\n");
        return -1;
    }
    atomic_set(&obj->is_enabled, en);
    GYRO_LOG("icm406xx_gyro_enable_nodata OK!\n");
    return 0;
}

static int icm406xx_gyro_set_delay(u64 ns)
{
    /* power mode setting in case of set_delay
    is called before enable_nodata */
    if(icm406xx_share_get_sensor_power(ICM406XX_SENSOR_TYPE_GYRO) == false)
        icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_GYRO, true);
    icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_GYRO, ns, false);
    return 0;
}

static int icm406xx_gyro_get_data(int *x , int *y, int *z, int *status)
{
    char buff[ICM406XX_BUFSIZE];
    int res = 0;

    /* dps */
    icm406xx_gyro_ReadSensorData(obj_i2c_data->client, buff, ICM406XX_BUFSIZE);
    res = sscanf(buff, "%x %x %x", x, y, z);
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    return 0;
}

static const struct i2c_device_id icm406xx_gyro_i2c_id[] =
    {{ICM406XX_GYRO_DEV_NAME, 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id gyro_of_match[] = {
    {.compatible = "inven,icm406_gyro"},
    {},
};
#endif

static int icm406xx_gyro_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs/1000/1000;

	GYRO_LOG("set gyro delay = %d\n", value);
	return icm406xx_gyro_set_delay(samplingPeriodNs);
}

static int icm406xx_gyro_flush(void)
{
	return gyro_flush_report();
}

static int icm406xx_gyro_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct i2c_client *new_client;
    struct icm406xx_gyro_i2c_data *obj;
    struct gyro_control_path ctl = {0};
    struct gyro_data_path data = {0};
    int res = 0;

    obj = kzalloc(sizeof(*obj), GFP_KERNEL);
    if (!obj) {
        res = -ENOMEM;
        goto exit;
    }
	res = get_gyro_dts_func(client->dev.of_node, hw);
    if (res<0) {
        GYRO_ERR("get dts info fail\n");
		res = -EFAULT;
		goto exit;
    }
    memset(obj, 0, sizeof(struct icm406xx_gyro_i2c_data));
    /* hwmsen_get_convert() depend on the direction value */
    obj->hw = hw;

    res = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
    if (res) {
        GYRO_ERR("invalid direction: %d\n", obj->hw->direction);
        goto exit;
    }
    if (0 != obj->hw->addr) {
        client->addr = obj->hw->addr >> 1;
        GYRO_LOG("gyro_use_i2c_addr: %x\n", client->addr);
    }
    obj_i2c_data = obj;
	client->addr = *hw->i2c_addr;
    obj->client = client;
    new_client = obj->client;
    i2c_set_clientdata(new_client, obj);
    atomic_set(&obj->trace, 0);
    atomic_set(&obj->suspend, 0);
    atomic_set(&obj->is_enabled, 0);
    icm406xx_gyro_i2c_client = new_client;

    res = icm406xx_gyro_init_client(new_client, false);
    if (res)
        goto exit_init_failed;
    /* misc_register() for factory mode, engineer mode and so on */
    res = misc_register(&icm406xx_gyro_device);
    if (res) {
        GYRO_ERR("icm406xx_gyro_device misc register failed!\n");
        goto exit_misc_device_register_failed;
    }
    /* create platform_driver attribute */
    res = icm406xx_gyro_create_attr(
        &(icm406xx_gyro_init_info.platform_diver_addr->driver));
    if (res) {
        GYRO_ERR("icm406xx_g create attribute err = %d\n", res);
        goto exit_create_attr_failed;
    }
    /* fill the gyro_control_path */
    ctl.is_use_common_factory = false;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch = obj->hw->is_batch_supported;
    ctl.open_report_data = icm406xx_gyro_open_report_data;
    ctl.enable_nodata = icm406xx_gyro_enable_nodata;
    ctl.set_delay  = icm406xx_gyro_set_delay;
	ctl.batch = icm406xx_gyro_batch;
	ctl.flush = icm406xx_gyro_flush;
    /* register the gyro_control_path */
    res = gyro_register_control_path(&ctl);
    if (res) {
        GYRO_ERR("register gyro control path err\n");
        goto exit_kfree;
    }
    /* fill the gyro_data_path */
    data.get_data = icm406xx_gyro_get_data;
    data.vender_div = DEGREE_TO_RAD;
    /* register the gyro_data_path */
    res = gyro_register_data_path(&data);
    if (res) {
        GYRO_ERR("register gyro_data_path fail = %d\n", res);
        goto exit_kfree;
    }
    icm406xx_gyro_init_flag = 0;
    GYRO_LOG("%s: OK\n", __func__);
    return 0;

exit_create_attr_failed:
    misc_deregister(&icm406xx_gyro_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
exit:
    kfree(obj);
    obj = NULL;
    icm406xx_gyro_init_flag =  -1;
    GYRO_ERR("%s: err = %d\n", __func__, res);
    return res;
}

static int icm406xx_gyro_i2c_remove(struct i2c_client *client)
{
    int res = 0;

    res = icm406xx_gyro_delete_attr(
        &(icm406xx_gyro_init_info.platform_diver_addr->driver));
    if (res)
        GYRO_ERR("icm406xx_gyro_delete_attr fail: %d\n", res);
    res = misc_deregister(&icm406xx_gyro_device);
    if (res)
        GYRO_ERR("misc_deregister fail: %d\n", res);
    icm406xx_gyro_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}
/*
static int icm406xx_gyro_i2c_detect(struct i2c_client *client,
    struct i2c_board_info *info)
{
    strcpy(info->type, ICM406XX_GYRO_DEV_NAME);
    return 0;
}
*/
static int icm406xx_gyro_i2c_suspend(struct i2c_client *client,
    pm_message_t msg)
{
    int res = 0;
    //struct icm406xx_gyro_i2c_data *obj = i2c_get_clientdata(client);
    struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;

    if (msg.event == PM_EVENT_SUSPEND) {
        if (obj == NULL) {
            GYRO_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->suspend, 1);
        res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_GYRO, false);
        if (res < 0) {
            GYRO_ERR("write power control fail!\n");
            return res;
        }
    }
    icm406xx_gyro_power(obj->hw, 0);
    GYRO_LOG("icm406xx_gyro suspend ok\n");
    return res;
}

static int icm406xx_gyro_i2c_resume(struct i2c_client *client)
{
    //struct icm406xx_gyro_i2c_data *obj = i2c_get_clientdata(client);
    struct icm406xx_gyro_i2c_data *obj = obj_i2c_data;
    int res = 0;

    if (obj == NULL) {
        GYRO_ERR("null pointer!!\n");
        return -EINVAL;
    }
    icm406xx_gyro_power(obj->hw, 1);
    if(atomic_read(&obj->is_enabled) == 1) {
        res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_GYRO, true);
    }
    if (res) {
        GYRO_ERR("resume client fail!!\n");
        return res;
    }
    atomic_set(&obj->suspend, 0);
    GYRO_LOG("icm406xx_gyro resume ok\n");
    return 0;
}

static struct i2c_driver icm406xx_gyro_i2c_driver = {
    .driver = {
    .name = ICM406XX_GYRO_DEV_NAME,
#ifdef CONFIG_OF
    .of_match_table = gyro_of_match,
#endif
    },
    .probe = icm406xx_gyro_i2c_probe,
    .remove = icm406xx_gyro_i2c_remove,
/*    .detect = icm406xx_gyro_i2c_detect,*/
    .suspend = icm406xx_gyro_i2c_suspend,
    .resume = icm406xx_gyro_i2c_resume,
    .id_table = icm406xx_gyro_i2c_id,
};

static int icm406xx_gyro_remove(void)
{
    icm406xx_gyro_power(hw, 0);
    i2c_del_driver(&icm406xx_gyro_i2c_driver);
    return 0;
}

static int icm406xx_gyro_local_init(struct platform_device *pdev)
{
    icm406xx_gyro_power(hw, 1);
    if (i2c_add_driver(&icm406xx_gyro_i2c_driver)) {
        GYRO_ERR("add driver error\n");
        return -1;
    }
    if (-1 == icm406xx_gyro_init_flag)		
        return -1;
    return 0;
}

static int __init icm406xx_gyro_init(void)
{
/*
    const char *name = "invensense,icm406xx_gyro";

    hw = get_gyro_dts_func(name, hw);
    if (!hw)
        GYRO_ERR("get dts info fail\n");
*/
    gyro_driver_add(&icm406xx_gyro_init_info);
    return 0;
}

static void __exit icm406xx_gyro_exit(void)
{
    /* nothing to do here */
}

module_init(icm406xx_gyro_init);
module_exit(icm406xx_gyro_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("icm406xx gyroscope driver");
