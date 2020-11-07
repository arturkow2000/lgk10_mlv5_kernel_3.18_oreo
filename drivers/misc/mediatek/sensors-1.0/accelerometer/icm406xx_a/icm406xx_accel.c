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

#if defined(CONFIG_OF)
#include <linux/of.h>
#endif
#include <linux/ioctl.h>
#include <hwmsensor.h>
#include "cust_acc.h"
#include "accel.h"
#include "icm406xx_register.h"
#include "icm406xx_share.h"
#include "icm406xx_accel.h"

#define ICM406XX_ACCEL_DEV_NAME    "ICM406XX_ACCEL"

struct icm406xx_accel_i2c_data {
    struct i2c_client    *client;
    struct acc_hw    *hw;
    struct hwmsen_convert    cvt;
    /*misc*/
    atomic_t    trace;
    atomic_t    suspend;
    atomic_t    selftest;
    atomic_t    is_enabled;
    /*data*/
    s16    cali_sw[ICM406XX_AXIS_NUM+1];
    s16    data[ICM406XX_AXIS_NUM+1];	
    u8    offset[ICM406XX_AXIS_NUM+1];
};

static int icm406xx_accel_init_flag =  -1;
struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;
struct i2c_client *icm406xx_accel_i2c_client;
static struct icm406xx_accel_i2c_data *obj_i2c_data;
#ifdef ICM406XX_SELFTEST
static char selftestRes[8] = { 0 };
#define SELF_TEST_ACC_BW_IND        BIT_ACCEL_UI_LNM_BW_10_IIR
#endif

// +/-4G as default
static int g_icm406xx_accel_sensitivity = ICM406XX_ACCEL_DEFAULT_SENSITIVITY;
static int icm406xx_accel_local_init(void);
static int icm406xx_accel_remove(void);
static struct acc_init_info icm406xx_accel_init_info = {
    .name = ICM406XX_ACCEL_DEV_NAME,
    .init = icm406xx_accel_local_init,
    .uninit = icm406xx_accel_remove,
};

#define DEBUG_MAX_CHECK_COUNT    32
#define DEBUG_MAX_IOCTL_CMD_COUNT    168
#define DEBUG_MAX_LOG_STRING_LENGTH    512
#define DEBUG_IOCTL_CMD_DEFAULT    0xFF
#define DEBUG_PRINT_ELEMENT_COUNT    8
#define DEBUG_PRINT_CHAR_COUNT_PER_LINE    24

struct icm406xx_debug_data {
    int    nTestCount[DEBUG_MAX_CHECK_COUNT];
    int    nUsedTestCount;
    unsigned char    ucCmdLog[DEBUG_MAX_IOCTL_CMD_COUNT];
    int    nLogIndex;	
    unsigned char    ucPrevCmd;
    int    nDriverState;
    char    strDebugDump[DEBUG_MAX_LOG_STRING_LENGTH];	
    int    nStrIndex;
};

struct icm406xx_debug_data icm406xx_accel_debug;

static void initDebugData(void)
{
    memset(&icm406xx_accel_debug, 0x00, sizeof(icm406xx_accel_debug));
    icm406xx_accel_debug.ucPrevCmd = DEBUG_IOCTL_CMD_DEFAULT;
    memset(&(icm406xx_accel_debug.ucCmdLog[0]), 
        DEBUG_IOCTL_CMD_DEFAULT, DEBUG_MAX_IOCTL_CMD_COUNT);
}

static void addCmdLog(unsigned int cmd)
{
    unsigned char converted_cmd = DEBUG_IOCTL_CMD_DEFAULT;

    switch (cmd) {
        case GSENSOR_IOCTL_INIT:    converted_cmd = 0; break;
        case GSENSOR_IOCTL_READ_CHIPINFO:    converted_cmd = 1; break;
        case GSENSOR_IOCTL_READ_SENSORDATA:    converted_cmd = 2; break;
        case GSENSOR_IOCTL_READ_OFFSET:    converted_cmd = 3; break;
        case GSENSOR_IOCTL_READ_GAIN:    converted_cmd = 4; break;
        case GSENSOR_IOCTL_READ_RAW_DATA:    converted_cmd = 5; break;
        case GSENSOR_IOCTL_SET_CALI:    converted_cmd = 6; break;
        case GSENSOR_IOCTL_GET_CALI:    converted_cmd = 7; break;
        case GSENSOR_IOCTL_CLR_CALI:    converted_cmd = 8; break;
    }
    icm406xx_accel_debug.nDriverState = 0;
    if ( (icm406xx_accel_debug.nLogIndex < DEBUG_MAX_IOCTL_CMD_COUNT) &&
        (converted_cmd != DEBUG_IOCTL_CMD_DEFAULT) ) {
        icm406xx_accel_debug.ucCmdLog[icm406xx_accel_debug.nLogIndex++]
            = converted_cmd;
        icm406xx_accel_debug.ucPrevCmd = converted_cmd;
    }
}

static void printCmdLog(void)
{
    int i;
    int nRowCount = 0;

    nRowCount = (int)(icm406xx_accel_debug.nLogIndex / 8);
    for (i = 0; i < nRowCount + 1; i++) {
        sprintf(&icm406xx_accel_debug.strDebugDump[i * 24],
            "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
            icm406xx_accel_debug.ucCmdLog[i*8+0],
            icm406xx_accel_debug.ucCmdLog[i*8+1],
            icm406xx_accel_debug.ucCmdLog[i*8+2],
            icm406xx_accel_debug.ucCmdLog[i*8+3],	
            icm406xx_accel_debug.ucCmdLog[i*8+4],
            icm406xx_accel_debug.ucCmdLog[i*8+5],
            icm406xx_accel_debug.ucCmdLog[i*8+6],
            icm406xx_accel_debug.ucCmdLog[i*8+7]);
    }
    ACC_LOG("ACCEL_IOCTL_CMD_LOG : \n%s\t%d\n", 
        icm406xx_accel_debug.strDebugDump,icm406xx_accel_debug.nLogIndex);
}

static int icm406xx_accel_SetFullScale(struct i2c_client *client, u8 accel_fsr)
{
    u8 databuf[2] = {0};
    int res = 0;

    res = icm406xx_share_read_register(REG_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
        ACC_ERR("read fsr register err!\n");
        return ICM406XX_ERR_BUS;
    }
    /* clear FSR bit */
    databuf[0] &= ~BIT_ACCEL_FSR;
    databuf[0] |= accel_fsr << SHIFT_ACCEL_FS_SEL;
    g_icm406xx_accel_sensitivity = ICM406XX_ACCEL_MIN_SENSITIVITY << accel_fsr;
    res = icm406xx_share_write_register(REG_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
        ACC_ERR("write fsr register err!\n");
        return ICM406XX_ERR_BUS;
    }
    return ICM406XX_SUCCESS;
}

static int icm406xx_accel_SetFilter(struct i2c_client *client, u8 accel_filter)
{
    u8 databuf[2] = {0};
    int res = 0;

    res = icm406xx_share_read_register(REG_GYRO_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
        ACC_ERR("read filter register err!\n");
        return ICM406XX_ERR_BUS;
    }
    /*  clear filter bit */
    databuf[0] &= ~BIT_ACCEL_FILTER;
    databuf[0] |= accel_filter;
    res = icm406xx_share_write_register(REG_GYRO_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
		ACC_ERR("write filter register err!\n");
		return ICM406XX_ERR_BUS;
    }
    return ICM406XX_SUCCESS;    
}

static int icm406xx_accel_ReadSensorDataDirect(struct i2c_client *client,
    s16 data[ICM406XX_AXIS_NUM])
{
    char databuf[6];
    int i;
    int res = 0;

    if (client == NULL) 
        return ICM406XX_ERR_INVALID_PARAM;
    res = icm406xx_share_read_register(REG_ACCEL_DATA_X0_UI, databuf,
        ICM406XX_DATA_LEN);
    if (res < 0) {
        ACC_ERR("read accelerometer data error\n");
        return ICM406XX_ERR_BUS ;
    }
    /*  convert 8-bit to 16-bit */
    for (i = 0; i < ICM406XX_AXIS_NUM; i++)
        data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
    return ICM406XX_SUCCESS;
}

static int icm406xx_accel_ReadSensorData(struct i2c_client *client,
    char *buf, int bufsize)
{
    char databuf[6];
    int  data[3];
    int  i = 0;
    int res = 0;
//    struct icm406xx_accel_i2c_data * obj = i2c_get_clientdata(client);
	struct icm406xx_accel_i2c_data * obj = obj_i2c_data;

    if (client == NULL)
        return ICM406XX_ERR_INVALID_PARAM;
    res = icm406xx_share_read_register(REG_ACCEL_DATA_X0_UI, databuf,
        ICM406XX_DATA_LEN);
    if (res < 0) {
        ACC_ERR("read accelerometer data error\n");
        return ICM406XX_ERR_BUS;
    }
    /*  convert 8-bit to 16-bit */
    for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
        obj->data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
        /* add calibration value */
        obj->data[i] += obj->cali_sw[i];
    }
    /* orientation translation lower (sensor) 
    --> upper (device) & unit translation */
    for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
        data[i] = obj->cvt.sign[i] * obj->data[obj->cvt.map[i]];
        data[i] = data[i] * GRAVITY_EARTH_1000  / g_icm406xx_accel_sensitivity;
    }
    sprintf(buf, "%04x %04x %04x",
        data[ICM406XX_AXIS_X], 
        data[ICM406XX_AXIS_Y], 
        data[ICM406XX_AXIS_Z]);
    if (atomic_read(&obj->trace)) {
        ACC_LOG("Accelerometer data - %04x %04x %04x\n",
            data[ICM406XX_AXIS_X],
            data[ICM406XX_AXIS_Y],
            data[ICM406XX_AXIS_Z]);
    }
    return ICM406XX_SUCCESS;
}

static int icm406xx_accel_ReadOffsetData(struct i2c_client *client, char *buf)
{
//    struct icm406xx_accel_i2c_data * obj = i2c_get_clientdata(client);
	struct icm406xx_accel_i2c_data * obj = obj_i2c_data;


    if (client == NULL)
        return ICM406XX_ERR_INVALID_PARAM;
    /* offset value [0, 0, 0] */
    sprintf(buf, "%04x %04x %04x",
        obj->offset[ICM406XX_AXIS_X],
        obj->offset[ICM406XX_AXIS_Y],
        obj->offset[ICM406XX_AXIS_Z]);
    return ICM406XX_SUCCESS;
}

static int icm406xx_accel_ReadGain(struct i2c_client *client,
    struct GSENSOR_VECTOR3D *gsensor_gain)
{
    if (client == NULL)
        return ICM406XX_ERR_INVALID_PARAM;
    gsensor_gain->x = gsensor_gain->y = gsensor_gain->z = 
        g_icm406xx_accel_sensitivity;
    return ICM406XX_SUCCESS;
}

static int icm406xx_accel_ReadRawData(struct i2c_client * client, char * buf)
{	
    int res = 0;
    s16 data[ICM406XX_AXIS_NUM] = { 0, 0, 0 };
	
    res = icm406xx_accel_ReadSensorDataDirect(client, data);
    if (res < 0) {
        ACC_ERR("read accelerometer raw data  error\n");
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

static int icm406xx_accel_ResetCalibration(struct i2c_client *client)
{
    //struct icm406xx_accel_i2c_data *obj = i2c_get_clientdata(client);
    struct icm406xx_accel_i2c_data * obj = obj_i2c_data;

    memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
    return ICM406XX_SUCCESS;
}

static int icm406xx_accel_ReadCalibration(struct i2c_client *client,
    struct SENSOR_DATA *sensor_data)
{
    //struct icm406xx_accel_i2c_data *obj = i2c_get_clientdata(client);
    struct icm406xx_accel_i2c_data * obj = obj_i2c_data;
    int i;
    int res = 0;
    int dat[ICM406XX_AXIS_NUM];

    for (i = 0; i < ICM406XX_AXIS_NUM; i++)
        dat[i] = obj->cvt.sign[i] * obj->cali_sw[obj->cvt.map[i]];

    /* lower layer (sensor) unit translation into upper layer (HAL) unit */
    sensor_data->x = dat[ICM406XX_AXIS_X] * GRAVITY_EARTH_1000 / 
        g_icm406xx_accel_sensitivity;
    sensor_data->y = dat[ICM406XX_AXIS_Y] * GRAVITY_EARTH_1000 / 
        g_icm406xx_accel_sensitivity;
    sensor_data->z = dat[ICM406XX_AXIS_Z] * GRAVITY_EARTH_1000 / 
        g_icm406xx_accel_sensitivity;
    if (atomic_read(&obj->trace)) {
        ACC_LOG("Accel ReadCalibration:[sensor_data:%5d %5d %5d]\n",
            sensor_data->x, sensor_data->y, sensor_data->z);
        ACC_LOG("Accel ReadCalibration:[cali_sw:%5d %5d %5d]\n",
            obj->cali_sw[ICM406XX_AXIS_X], 
            obj->cali_sw[ICM406XX_AXIS_Y], 
            obj->cali_sw[ICM406XX_AXIS_Z]);
    }
    return res;
}

static int icm406xx_accel_WriteCalibration(struct i2c_client *client,
    struct SENSOR_DATA *sensor_data)
{
    //struct icm406xx_accel_i2c_data *obj = i2c_get_clientdata(client);
    struct icm406xx_accel_i2c_data * obj = obj_i2c_data;
    int cali[ICM406XX_AXIS_NUM];
    int res = 0;
    int i;

    /* upper layer (HAL) unit translation into lower layer (sensor) unit */
    cali[ICM406XX_AXIS_X] = sensor_data->x * 
        g_icm406xx_accel_sensitivity / GRAVITY_EARTH_1000;
    cali[ICM406XX_AXIS_Y] = sensor_data->y * 
        g_icm406xx_accel_sensitivity / GRAVITY_EARTH_1000;
    cali[ICM406XX_AXIS_Z] = sensor_data->z * 
        g_icm406xx_accel_sensitivity / GRAVITY_EARTH_1000;
    /* orientation translation */
    for (i = 0; i < ICM406XX_AXIS_NUM; i++)
        obj->cali_sw[obj->cvt.map[i]] += obj->cvt.sign[i] * cali[i];		
    if (atomic_read(&obj->trace)) {
        ACC_LOG("Accel writeCalibration:[sensor_data:%5d %5d %5d]\n",
            sensor_data->x, sensor_data->y, sensor_data->z);
        ACC_LOG("Accel writeCalibration:[cali_sw:%5d %5d %5d]\n",
            obj->cali_sw[ICM406XX_AXIS_X], 
            obj->cali_sw[ICM406XX_AXIS_Y], 
            obj->cali_sw[ICM406XX_AXIS_Z]);
    }
    return res;
}

#ifdef ICM406XX_SELFTEST
static int icm406xx_accel_ResetOffsetReg(void)
{
    int res = 0;
    char databuf[2] = {0};

    res = icm406xx_share_read_register(REG_GOS_USER4, 
        databuf, 1);
    if (res) {
        ACC_ERR("read offset fail: %d\n", res);
        return false;
    }
    databuf[0] &= 0x0f;
    res = icm406xx_share_write_register(REG_GOS_USER4, databuf, 1);
    if (res < 0) {
        ACC_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    databuf[0] = 0x00;
    res = icm406xx_share_write_register(REG_GOS_USER5, databuf, 1);
    if (res < 0) {
        ACC_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    res = icm406xx_share_write_register(REG_GOS_USER6, databuf, 1);
    if (res < 0) {
        ACC_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    res = icm406xx_share_write_register(REG_GOS_USER7, databuf, 1);
    if (res < 0) {
        ACC_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    res = icm406xx_share_write_register(REG_GOS_USER8, databuf, 1);
    if (res < 0) {
        ACC_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    return ICM406XX_SUCCESS;
}

static int  icm406xx_accel_InitSelfTest(struct i2c_client * client)
{
    int res = 0;

    /* soft reset */
    res = icm406xx_share_ChipSoftReset();
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set power mode */
    icm406xx_share_set_sensor_power_mode(ICM406XX_SENSOR_TYPE_ACC,
        BIT_ACCEL_MODE_LNM);
    /* reset offset*/
    res = icm406xx_accel_ResetOffsetReg();
    if (res != ICM406XX_SUCCESS)
        return res;        
    /* setpowermode(true) --> exit sleep */
    res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_ACC, true);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* fsr : ICM406XX_ACCEL_RANGE_2G */
    res = icm406xx_accel_SetFullScale(client, ICM406XX_ACCEL_RANGE_2G);	
    if (res != ICM406XX_SUCCESS)
        return res;
    /* filter : SELF_TEST_ACC_BW_IND */
    res = icm406xx_accel_SetFilter(client, SELF_TEST_ACC_BW_IND);
    if (res != ICM406XX_SUCCESS)
        return res;	
    /* odr : 1000hz (1kHz) */
    res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_ACC, 
        1000000, true);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set enable sensor */
    res = icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_ACC, true);
    if (res != ICM406XX_SUCCESS)
        return res;
    mdelay(SELF_TEST_READ_INTERVAL_MS);
    return res;
}

static int icm406xx_accel_CalcAvgWithSamples(struct i2c_client *client,
    int avg[3], int count)
{
    int res = 0;
    int i, nAxis;
    s16 sensor_data[ICM406XX_AXIS_NUM];
    s32 sum[ICM406XX_AXIS_NUM] = { 0, 0, 0 };

    for (i = 0; i < count; i++) {
        res = icm406xx_accel_ReadSensorDataDirect(client, sensor_data);
        if (res) {
            ACC_ERR("read data fail: %d\n", res);
            return ICM406XX_ERR_STATUS;
        }
        for (nAxis = 0; nAxis < ICM406XX_AXIS_NUM; nAxis++)
            sum[nAxis] += sensor_data[nAxis];
        /* data register updated @1khz */
        mdelay(1);
    }
    for (nAxis = 0; nAxis < ICM406XX_AXIS_NUM; nAxis++)
        avg[nAxis] = (int)(sum[nAxis] / count) * SELF_TEST_PRECISION;
    return res;
}

static bool icm406xx_accel_DoSelfTest(struct i2c_client *client)
{
    int res = 0;
    int i;
    int acc_ST_on[ICM406XX_AXIS_NUM], acc_ST_off[ICM406XX_AXIS_NUM];
    /* index of otp_lookup_tbl */
    u8  st_code[ICM406XX_AXIS_NUM];
    u16 st_otp[ICM406XX_AXIS_NUM];
    bool otp_value_has_zero = false;
    bool test_result = true;
    u8 databuf[2] = {0};
    int ratio, st_res;
    int retry;

    databuf[0] = BIT_BANK_SEL_2;
    res = icm406xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    if (res < 0) {
        ACC_ERR("write bank select register err!\n");
        return ICM406XX_ERR_BUS;
    }    
    /* acquire OTP value from OTP lookup table */
    res = icm406xx_share_read_register(REG_XA_ST_DATA, &(st_code[0]), 3);
    if (res) {
        ACC_ERR("Read selftest opt data failed: %d\n", res);
        return false;
    }
    ACC_LOG("st_code: %02x, %02x, %02x\n", st_code[0], st_code[1], st_code[2]);
    databuf[0] = BIT_BANK_SEL_0;
    res = icm406xx_share_write_register(REG_REG_BANK_SEL, databuf, 1);
    if (res < 0) {
        ACC_ERR("write bank select register err!\n");
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
        res = icm406xx_accel_CalcAvgWithSamples(client, acc_ST_off,
            SELF_TEST_SAMPLE_NB);
        if (res) {
            ACC_ERR("Read sample data failed: %d\n", res);
            return false;
        }
        /* set selftest on */
        databuf[0] = (BIT_TEST_AX_EN | BIT_TEST_AY_EN | BIT_TEST_AZ_EN);
        databuf[0] |= BIT_SELF_TEST_REGULATOR_EN;
        res = icm406xx_share_write_register(REG_SELF_TEST_CONFIG, databuf, 1);
        if (res) {
            ACC_ERR("enable st accel fail: %d\n", res);
            return false;
        }
        /* wait 20ms for oscillations to stabilize */
        mdelay(20);
        /* Read 200 Samples with selftest on */
        res = icm406xx_accel_CalcAvgWithSamples(client, acc_ST_on, 
            SELF_TEST_SAMPLE_NB);
        if (res) {
            ACC_ERR("Read data fail: %d\n", res);
            return false;
        }
        /* set selftest off */
        databuf[0] = 0x00;
        res = icm406xx_share_write_register(REG_SELF_TEST_CONFIG, databuf, 1);
        if (res) {
            ACC_ERR("disable st accel fail: %d\n", res);
            return false;
        }
        /* wait 20ms for oscillations to stabilize */
        mdelay(20);        
        /* compare calculated value with OTP value to judge success or fail */
        if (otp_value_has_zero == false) {
            /* criteria a */
            for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
                st_res = acc_ST_on[i] - acc_ST_off[i];
                ratio = abs(st_res / st_otp[i] - SELF_TEST_PRECISION);
                if (ratio >= SELF_TEST_ACC_SHIFT_DELTA) {
                    ACC_LOG("error accel[%d] : st_res = %d, st_otp = %d\n",
                        i, st_res, st_otp[i]);
                    test_result = false;
                }
            }
        } else {
            /* criteria b */
            for (i = 0; i < ICM406XX_AXIS_NUM; i++) {
                st_res = abs(acc_ST_on[i] - acc_ST_off[i]);
                if (st_res < SELF_TEST_MIN_ACC || st_res > SELF_TEST_MAX_ACC) {
                    ACC_LOG("error accel[%d] : st_res = %d, min = %d, max = %d\n",
                        i, st_res, SELF_TEST_MIN_ACC, SELF_TEST_MAX_ACC);
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

static int icm406xx_accel_init_client(struct i2c_client *client, bool enable)
{
    int res = 0;
    u8 databuf[2] = {0};

    /* exit sleep mode */
    res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_ACC, true);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set fsr ICM406XX_ACCEL_RANGE_4G as default */
    res = icm406xx_accel_SetFullScale(client, ICM406XX_ACCEL_RANGE_4G);
    if (res != ICM406XX_SUCCESS)
        return res;
#ifdef ICM406XX_ACCEL_LOW_POWER_MODE
    /* set power mode */
    icm406xx_share_set_sensor_power_mode(ICM406XX_SENSOR_TYPE_ACC,
        BIT_ACCEL_MODE_LPM);
    /* set filter BIT_ACCEL_UI_LPM_AVG_1 as default */
    res = icm406xx_accel_SetFilter(client, BIT_ACCEL_UI_LPM_AVG_1);
    if (res != ICM406XX_SUCCESS)
        return res;
#else
    /* set power mode */
    icm406xx_share_set_sensor_power_mode(ICM406XX_SENSOR_TYPE_ACC,
        BIT_ACCEL_MODE_LNM);
    /* set filter BIT_ACCEL_UI_LNM_BW_2_FIR as default */
    res = icm406xx_accel_SetFilter(client, BIT_ACCEL_UI_LNM_BW_2_FIR);
    if (res != ICM406XX_SUCCESS)
        return res;
#endif
    /* set 5ms(200hz) sample rate */
    res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_ACC,
        5000000, false);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set accel config1 as default */
    databuf[0] = 0x15;
    res = icm406xx_share_write_register(REG_ACCEL_CONFIG1, databuf, 1);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* disable sensor - standby mode for accelerometer */
    res = icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_ACC, enable);
    if (res != ICM406XX_SUCCESS)
        return res;
    /* set power mode - sleep or normal */
    res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_ACC, enable);
    if (res != ICM406XX_SUCCESS)
        return res;
    ACC_LOG("icm406xx_accel_init_client OK!\n");
    return ICM406XX_SUCCESS;
}

struct acc_hw *get_cust_acc(void)
{
    return &accel_cust;
}

static void icm406xx_accel_power(struct acc_hw *hw, unsigned int on)
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
    struct i2c_client *client = icm406xx_accel_i2c_client;
    char strbuf[ICM406XX_BUFSIZE];

    if (NULL == client) {
        ACC_ERR("i2c client is null!!\n");
        return 0;
    }
    icm406xx_accel_ReadSensorData(client, strbuf, ICM406XX_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    struct icm406xx_accel_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        ACC_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
    return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, 
    const char *buf, size_t count)
{
    struct icm406xx_accel_i2c_data *obj = obj_i2c_data;
    int trace;

    if (obj == NULL) {
        ACC_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    if (0 == kstrtoint(buf, 16, &trace))
        atomic_set(&obj->trace, trace);
    else
        ACC_ERR("invalid content: '%s', length = %zu\n", buf, count);
    return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct icm406xx_accel_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        ACC_ERR("i2c_data obj is null!!\n");
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
    struct icm406xx_accel_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        ACC_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    ACC_LOG("[%s] default direction: %d\n", __func__, obj->hw->direction);
    len = snprintf(buf, PAGE_SIZE, "default direction = %d\n", obj->hw->direction);
    return len;
}

static ssize_t store_chip_orientation(struct device_driver *ddri, 
    const char *buf, size_t tCount)
{
    int nDirection = 0;
    int res = 0;
    struct icm406xx_accel_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        ACC_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    res = kstrtoint(buf, 10, &nDirection);
    if (res != 0) {
        if (hwmsen_get_convert(nDirection, &obj->cvt))
            ACC_ERR("ERR: fail to set direction\n");
    }
    ACC_LOG("[%s] set direction: %d\n", __func__, nDirection);
    return tCount;
}

static ssize_t show_register_dump(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct icm406xx_accel_i2c_data *obj = obj_i2c_data;
    int i;
    u8 databuf[2] = {0};
    int res = 0;		

    if (obj == NULL) {
        ACC_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    len += snprintf(buf + len, PAGE_SIZE, "Register Dump\n");
    for(i = 0; i < 0x7F; i++) {
        res = icm406xx_share_read_register(i, databuf, 1);
        if (res < 0) {
            ACC_ERR("read register err!\n");
            return 0;
        }		
        len += snprintf(buf + len, PAGE_SIZE, "0x%02x: 0x%02x\n",
            i, databuf[0]);
    }
    return len;
}

#ifdef ICM406XX_SELFTEST
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = icm406xx_accel_i2c_client;

    if (NULL == client) {
        ACC_ERR("i2c client is null!!\n");
        return 0;
    }
    return snprintf(buf, 8, "%s\n", selftestRes);
}

static ssize_t store_selftest_value(struct device_driver *ddri, 
    const char *buf, size_t count)
{
    struct i2c_client *client = icm406xx_accel_i2c_client;
    int num;
    int res = 0;

    /* check parameter values to run selftest */
    res = kstrtoint(buf, 10, &num);
    if (res != 0) {
        ACC_ERR("parse number fail\n");
        return count;
    } else if (num == 0) {
        ACC_ERR("invalid data count\n");
        return count;
    }
    /* run selftest */
    res = icm406xx_accel_InitSelfTest(client);
    if (icm406xx_accel_DoSelfTest(client) == true) {
        strcpy(selftestRes, "y");
        ACC_LOG("ACCEL SELFTEST : PASS\n");
    } else {
        strcpy(selftestRes, "n");
        ACC_LOG("ACCEL SELFTEST : FAIL\n");
    }
    /* selftest is considered to be called only in factory mode
    in general mode, the condition before selftest will not be recovered
    and sensor will not be in sleep mode */
    res = icm406xx_accel_init_client(client, true);		
    return count;
}
#endif

static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, 
    store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, 
    store_chip_orientation);
static DRIVER_ATTR(regdump, S_IRUGO, show_register_dump, NULL);
#ifdef ICM406XX_SELFTEST
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, show_selftest_value, 
    store_selftest_value);
#endif

static struct driver_attribute *icm406xx_accel_attr_list[] = {
    /* chip information */
    &driver_attr_chipinfo,
    /* dump sensor data */
    &driver_attr_sensordata,
    /* trace log */
    &driver_attr_trace,
    /* chip status */
    &driver_attr_status,
    /* chip orientation information */
    &driver_attr_orientation,
    /* register dump */
    &driver_attr_regdump,
#ifdef ICM406XX_SELFTEST
    /* run selftest when store, report selftest result when show */
    &driver_attr_selftest
#endif
};

static int icm406xx_accel_create_attr(struct device_driver *driver)
{
    int idx;
    int res = 0;
    int num = 
        (int)(sizeof(icm406xx_accel_attr_list)/
        sizeof(icm406xx_accel_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;
    for (idx = 0; idx < num; idx++) {
        res = driver_create_file(driver, icm406xx_accel_attr_list[idx]);
        if (0 != res) {
            ACC_ERR("driver_create_file (%s) = %d\n", 
                icm406xx_accel_attr_list[idx]->attr.name, res);
            break;
        }
    }
    return res;
}

static int icm406xx_accel_delete_attr(struct device_driver *driver)
{
    int idx;
    int res = 0;
    int num = 
        (int)(sizeof(icm406xx_accel_attr_list)/
        sizeof(icm406xx_accel_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;
    for (idx = 0; idx < num; idx++)
        driver_remove_file(driver, icm406xx_accel_attr_list[idx]);
    return res;
}

static int icm406xx_accel_open(struct inode *inode, struct file *file)
{
    file->private_data = icm406xx_accel_i2c_client;
    if (file->private_data == NULL) {
        ACC_ERR("null pointer!!\n");
        return -EINVAL;
    }	
    return nonseekable_open(inode, file);
}

static int icm406xx_accel_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static long icm406xx_accel_unlocked_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client *)file->private_data;
    struct icm406xx_accel_i2c_data *obj = 
//        (struct icm406xx_accel_i2c_data *)i2c_get_clientdata(client);
		obj_i2c_data;

    char strbuf[ICM406XX_BUFSIZE] = {0};
    void __user *data;
    long res = 0;
    struct SENSOR_DATA sensor_data;
    static struct GSENSOR_VECTOR3D gsensor_gain;

    if (_IOC_DIR(cmd) & _IOC_READ)
        res = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        res = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (res) {
        ACC_ERR("access error: %08X, (%2d, %2d)\n", 
            cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }
    addCmdLog(cmd);
    printCmdLog();
    switch (cmd) {
    case GSENSOR_IOCTL_INIT:
        icm406xx_share_ChipSoftReset();
        icm406xx_accel_init_client(client, true);
        break;
    case GSENSOR_IOCTL_READ_CHIPINFO:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm406xx_share_ReadChipInfo(strbuf, ICM406XX_BUFSIZE);
        if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
            res = -EINVAL;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_SENSORDATA:
        data = (void __user *) arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm406xx_accel_ReadSensorData(client, strbuf, ICM406XX_BUFSIZE);
        if (copy_to_user(data, strbuf, sizeof(strbuf))) {
            res = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_OFFSET:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm406xx_accel_ReadOffsetData(client, strbuf);
        if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
            res = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_GAIN:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm406xx_accel_ReadGain(client, &gsensor_gain);
        if (copy_to_user(data, &gsensor_gain, 
            sizeof(struct GSENSOR_VECTOR3D))) {
            res = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_RAW_DATA:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm406xx_accel_ReadRawData(client, strbuf);
        if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
            res = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_SET_CALI:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
            res = -EFAULT;
            break;
        }
        if (atomic_read(&obj->suspend)) {
            ACC_ERR("Perform calibration in suspend state!!\n");
            res = -EINVAL;
        } else {
            ACC_LOG("accel set cali:[%5d %5d %5d]\n", 
                sensor_data.x, sensor_data.y, sensor_data.z);
            res = icm406xx_accel_WriteCalibration(client, &sensor_data);
        }
        break;
    case GSENSOR_IOCTL_GET_CALI:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        res = icm406xx_accel_ReadCalibration(client, &sensor_data);
        if (res)
            break;
        if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
            res = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_CLR_CALI:
        res = icm406xx_accel_ResetCalibration(client);
        break;
    default:
        ACC_ERR("unknown IOCTL: 0x%08x\n", cmd);
        res = -ENOIOCTLCMD;
    }
    return res;
}

#ifdef CONFIG_COMPAT
static long icm406xx_accel_compat_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    long res = 0;
    void __user *arg32 = compat_ptr(arg);

    if(!file->f_op || !file->f_op->unlocked_ioctl){
        ACC_ERR("compat_ion_ioctl file has no f_op\n");
        ACC_ERR("or no f_op->unlocked_ioctl.\n");
        return -ENOTTY;
    }
    switch (cmd) {
    case COMPAT_GSENSOR_IOCTL_INIT:
    case COMPAT_GSENSOR_IOCTL_READ_CHIPINFO: 
    case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
    case COMPAT_GSENSOR_IOCTL_READ_OFFSET:		
    case COMPAT_GSENSOR_IOCTL_READ_GAIN:		
    case COMPAT_GSENSOR_IOCTL_READ_RAW_DATA:
    case COMPAT_GSENSOR_IOCTL_SET_CALI:
    case COMPAT_GSENSOR_IOCTL_GET_CALI:
    case COMPAT_GSENSOR_IOCTL_CLR_CALI:
        if (arg32 == NULL) {
            ACC_ERR("invalid argument.");
            return -EINVAL;
        }
        res = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
        break;
    default:
        ACC_ERR("%s not supported = 0x%04x", __func__, cmd);
        return -ENOIOCTLCMD;
    }
    return res;
}
#endif

static const struct file_operations icm406xx_accel_fops = {
    .open = icm406xx_accel_open,
    .release = icm406xx_accel_release,
    .unlocked_ioctl = icm406xx_accel_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = icm406xx_accel_compat_ioctl,
#endif
};

static struct miscdevice icm406xx_accel_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gsensor",
    .fops = &icm406xx_accel_fops,
};

static int icm406xx_accel_open_report_data(int open)
{
    /* nothing to do here for 406xx */
    return 0;
}

static int icm406xx_accel_enable_nodata(int en)
{
    /* if use this type of enable , 
    gsensor only enabled but not report inputEvent to HAL */
    int res = 0;
    bool power = false;
    struct icm406xx_accel_i2c_data *obj = obj_i2c_data;

    if (1 == en) {
        power = true;
        res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_ACC, power);
        res |= icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_ACC, power);
    }
    if (0 == en) {
        power = false;
        res = icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_ACC, 0, false);
        res |= icm406xx_share_EnableSensor(ICM406XX_SENSOR_TYPE_ACC, power);
        res |= icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_ACC, power);
    }
    if (res != ICM406XX_SUCCESS) {
        ACC_LOG("fail!\n");
        return -1;
    }
    atomic_set(&obj->is_enabled, en);
    ACC_LOG("icm406xx_accel_enable_nodata OK!\n");
    return 0;
}

static int icm406xx_accel_set_delay(u64 ns)
{
    /* power mode setting in case of set_delay
    is called before enable_nodata */
    ACC_LOG("%s is called [ns:%lld]\n", __func__, ns);
    if(icm406xx_share_get_sensor_power(ICM406XX_SENSOR_TYPE_ACC) == false)	
        icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_ACC, true);
    
    icm406xx_share_SetSampleRate(ICM406XX_SENSOR_TYPE_ACC, ns, false);	
    return 0;
}

static int icm406xx_accel_get_data(int *x , int *y, int *z, int *status)
{
    char buff[ICM406XX_BUFSIZE];
    int res = 0;
	
    icm406xx_accel_ReadSensorData(obj_i2c_data->client, 
        buff, ICM406XX_BUFSIZE);
    res = sscanf(buff, "%x %x %x", x, y, z);
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    return 0;
}

static const struct i2c_device_id icm406xx_accel_i2c_id[] = 
    {{ICM406XX_ACCEL_DEV_NAME, 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
    {.compatible = "inven,icm406_acc"},
    {},
};
#endif

static int gsensor_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs/1000/1000;

	ACC_LOG("gsensor_batch period=(%d ms)\n", value);
	return icm406xx_accel_set_delay(samplingPeriodNs);
}
static int gsensor_flush(void)
{
	return acc_flush_report();
}

static int icm406xx_accel_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct i2c_client *new_client;
    struct icm406xx_accel_i2c_data *obj;
    struct acc_control_path ctl = {0};
    struct acc_data_path data = {0};
    int res = 0;

    ACC_LOG();

    obj = kzalloc(sizeof(*obj), GFP_KERNEL);
    if (!obj) {
        res = -ENOMEM;
        goto exit;
    }
	res = get_accel_dts_func(client->dev.of_node, hw);
    if (res<0){
        ACC_ERR("get dts info fail\n");
		res = -EFAULT;
		goto exit;
    }
    memset(obj, 0, sizeof(struct icm406xx_accel_i2c_data));
    /* hwmsen_get_convert() depend on the direction value */
    obj->hw = hw;
    res = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
    if (res) {
        ACC_ERR("invalid direction: %d\n", obj->hw->direction);
        goto exit;
    }
    obj_i2c_data = obj;
	client->addr = *hw->i2c_addr;
    obj->client = client;
    new_client = obj->client;
    i2c_set_clientdata(new_client, obj);
    atomic_set(&obj->trace, 0);
    atomic_set(&obj->suspend, 0);
    atomic_set(&obj->is_enabled, 0);
    icm406xx_accel_i2c_client = new_client;
    /* soft reset is called only in accelerometer init */
    /* do not call soft reset in gyro and step_count init */
#if 0	/*LGE_SENSOR_CHANGE I2C Fail ISSUE */
    res = icm406xx_share_ChipSoftReset();
    if (res != ICM406XX_SUCCESS)
        goto exit_init_failed;
#endif	/*LGE_SENSOR_CHANGE I2C Fail ISSUE */
    res = icm406xx_accel_init_client(new_client, false);
    if (res)
        goto exit_init_failed;
    /* misc_register() for factory mode, engineer mode and so on */
    res = misc_register(&icm406xx_accel_device);
    if (res) {
        ACC_ERR("icm406xx_accel_device misc register failed!\n");
        goto exit_misc_device_register_failed;
    }
    /* create platform_driver attribute */
    res = icm406xx_accel_create_attr(
        &(icm406xx_accel_init_info.platform_diver_addr->driver));
    if (res) {
        ACC_ERR("icm406xx_accel create attribute err = %d\n", res);
        goto exit_create_attr_failed;
    }
    /* fill the acc_control_path */
    ctl.is_use_common_factory = false;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch = obj->hw->is_batch_supported;
    ctl.open_report_data = icm406xx_accel_open_report_data;
    ctl.enable_nodata = icm406xx_accel_enable_nodata;
    ctl.set_delay  = icm406xx_accel_set_delay;
    ctl.batch = gsensor_batch;
    ctl.flush = gsensor_flush;
    /* register the acc_control_path */
    res = acc_register_control_path(&ctl);
    if (res) {
        ACC_ERR("register accel control path err\n");
        goto exit_kfree;
    }
    /* fill the acc_data_path */
    data.get_data = icm406xx_accel_get_data;	
    data.vender_div = 1000;
    /* register the acc_data_path */
    res = acc_register_data_path(&data);
    if (res) {
        ACC_ERR("register accel_data_path fail = %d\n", res);
        goto exit_kfree;
    }
    icm406xx_accel_init_flag = 0;
    ACC_LOG("%s: OK\n", __func__);
    return 0;

exit_create_attr_failed:
    misc_deregister(&icm406xx_accel_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
exit:
    kfree(obj);
    obj = NULL;
    icm406xx_accel_init_flag =  -1;
    ACC_ERR("%s: err = %d\n", __func__, res);
    return res;
}

static int icm406xx_accel_i2c_remove(struct i2c_client *client)
{
    int res = 0;

    res = icm406xx_accel_delete_attr(
        &(icm406xx_accel_init_info.platform_diver_addr->driver));
    if (res)
        ACC_ERR("icm406xx_accel_delete_attr fail: %d\n", res);
    res = misc_deregister(&icm406xx_accel_device);
    if (res)
        ACC_ERR("misc_deregister fail: %d\n", res);
#if 0
	res = hwmsen_detach(ID_ACCELEROMETER);
    if (res)
        ACC_ERR("hwmsen_detach fail: %d\n", res);
#endif
    icm406xx_accel_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}
/*
static int icm406xx_accel_i2c_detect(struct i2c_client *client,
    struct i2c_board_info *info)
{
    strcpy(info->type, ICM406XX_ACCEL_DEV_NAME);
    return 0;
}
*/
static int icm406xx_accel_i2c_suspend(struct i2c_client *client,
    pm_message_t msg)
{
    int res = 0;
//    struct icm406xx_accel_i2c_data *obj = i2c_get_clientdata(client);
	struct icm406xx_accel_i2c_data * obj = obj_i2c_data;


    if (msg.event == PM_EVENT_SUSPEND) {
        if (obj == NULL) {
            ACC_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->suspend, 1);
        res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_ACC, false);
        if (res < 0) {
            ACC_ERR("write power control fail!\n");
            return res;
        }
        icm406xx_accel_power(obj->hw, 0);
        ACC_LOG("icm406xx_accel suspend ok\n");
    }
    return res;
}

static int icm406xx_accel_i2c_resume(struct i2c_client *client)
{
//    struct icm406xx_accel_i2c_data *obj = i2c_get_clientdata(client);
	struct icm406xx_accel_i2c_data * obj = obj_i2c_data;

    int res = 0;

    if (obj == NULL) {
        ACC_ERR("null pointer!!\n");
        return -EINVAL;
    }
    icm406xx_accel_power(obj->hw, 1);
    if(atomic_read(&obj->is_enabled) == 1) {
        res = icm406xx_share_SetPowerMode(ICM406XX_SENSOR_TYPE_ACC, true);
    }
    if (res) {
        ACC_ERR("resume client fail!!\n");
        return res;
    }
    atomic_set(&obj->suspend, 0);
    ACC_LOG("icm406xx_accel resume ok\n");
    return 0;
}

static struct i2c_driver icm406xx_accel_i2c_driver = {
    .driver = {
    .name = ICM406XX_ACCEL_DEV_NAME,
#ifdef CONFIG_OF
    .of_match_table = accel_of_match,
#endif
    },
    .probe = icm406xx_accel_i2c_probe,
    .remove = icm406xx_accel_i2c_remove,
    /*.detect = icm406xx_accel_i2c_detect,*/
    .suspend = icm406xx_accel_i2c_suspend,
    .resume = icm406xx_accel_i2c_resume,
    .id_table = icm406xx_accel_i2c_id,
};

static int icm406xx_accel_remove(void)
{
    icm406xx_accel_power(hw, 0);
    i2c_del_driver(&icm406xx_accel_i2c_driver);
    return 0;
}

static int icm406xx_accel_local_init(void)
{
    icm406xx_accel_power(hw, 1);
    initDebugData();
    if (i2c_add_driver(&icm406xx_accel_i2c_driver)) {
        ACC_ERR("add driver error\n");
        return -1;
    }
    if (-1 == icm406xx_accel_init_flag)		
        return -1;
    return 0;
}

static int __init icm406xx_accel_init(void)
{
/*
    const char *name = "invensense,icm406xx_accel";

    hw = get_accel_dts_func(name, hw);
    if (!hw)
        ACC_ERR("get dts info fail\n");
*/
    acc_driver_add(&icm406xx_accel_init_info);
    return 0;
}

static void __exit icm406xx_accel_exit(void)
{
    /* nothing to do here */
}

module_init(icm406xx_accel_init);
module_exit(icm406xx_accel_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("icm406xx accelerometer driver");

