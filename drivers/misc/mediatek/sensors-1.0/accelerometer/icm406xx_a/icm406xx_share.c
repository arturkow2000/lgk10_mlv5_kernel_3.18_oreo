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

#include <hwmsensor.h>
#include "cust_acc.h"
#include "accel.h"
#include "icm406xx_register.h"
#include "icm406xx_share.h"
#include "icm406xx_share_interface.h"
#include <linux/math64.h> 

struct icm406xx_sensor_status_info {
    bool sensor_power;
    int sample_rate;
    u8  sensor_power_mode;
};
static struct icm406xx_sensor_status_info 
    icm406xx_all_sensor_status_info[ICM406XX_SENSOR_TYPE_MAX];

static void icm406xx_set_sensor_power_mode(int paramSensor,
    u8 sensor_power_mode)
{
    icm406xx_all_sensor_status_info[paramSensor].sensor_power_mode
        = sensor_power_mode;
}

static u8 icm406xx_get_sensor_power_mode(int paramSensor)
{
    return icm406xx_all_sensor_status_info[paramSensor].sensor_power_mode;
}

static bool icm406xx_get_sensor_power(int paramSensor)
{
    return icm406xx_all_sensor_status_info[paramSensor].sensor_power;
}

static bool icm406xx_any_accel_based_sensor_is_on(void)
{
/* return true if any other step counter sensors are enabled except itself
    step counter sensors are 
	   : ICM406XX_SENSOR_TYPE_SC, 
	   : ICM406XX_SENSOR_TYPE_SD, 
	   : ICM406XX_SENSOR_TYPE_SMD 
*/
    if (icm406xx_all_sensor_status_info
        [ICM406XX_SENSOR_TYPE_SC].sensor_power)
        return true;
    if (icm406xx_all_sensor_status_info
        [ICM406XX_SENSOR_TYPE_SD].sensor_power)
        return true;
    if (icm406xx_all_sensor_status_info
        [ICM406XX_SENSOR_TYPE_SMD].sensor_power)
        return true;
    return false;
}

static int icm406xx_ChipSoftReset(void)
{
    u8 databuf[10];
    int res = 0;
    int i;

    memset(databuf, 0, sizeof(u8) * 10);
    /* read */
    res = icm406xx_share_read_register(REG_CHIP_CONFIG_REG, databuf, 0x1);
    if (res < 0) {
        ACC_ERR("read power ctl register err!\n");
        return ICM406XX_ERR_BUS;
    }
    /* set device_reset bit to do soft reset */
    databuf[0] |= BIT_SOFT_RESET;
    res = icm406xx_share_write_register(REG_CHIP_CONFIG_REG, databuf, 0x1);
    if (res < 0) {
        ACC_ERR("write power ctl register err!\n");
        return ICM406XX_ERR_BUS;
    }
    mdelay(100);
    /* sensor status reset */
    for (i = 0; i < ICM406XX_SENSOR_TYPE_MAX; i++) {
        icm406xx_all_sensor_status_info[i].sensor_power = false;
        icm406xx_all_sensor_status_info[i].sample_rate = 0;
    }
    return ICM406XX_SUCCESS;
}

static int icm406xx_SetPowerMode(int sensor_type, bool enable)
{
    u8 databuf[2] = {0};
    int res = 0;
    int i;

    if(sensor_type >= ICM406XX_SENSOR_TYPE_MAX)
        return ICM406XX_ERR_INVALID_PARAM;
    icm406xx_all_sensor_status_info[sensor_type].sensor_power = enable;
    res = icm406xx_share_read_register(REG_TMST_CONFIG, databuf, 1);
    if (res < 0) {
        ACC_ERR("read tmst register err!\n");
        return ICM406XX_ERR_BUS;
    }
    if (enable == false) {
        /* check power status of all sensors */
        for(i = 0; i < ICM406XX_SENSOR_TYPE_MAX; i++)
            if(icm406xx_all_sensor_status_info[i].sensor_power == true)
                break;
        /* turn off RC oscillator when all sensors are disabled */
        if(i == ICM406XX_SENSOR_TYPE_MAX) {
            databuf[0] = BIT_EN_DREG_FIFO_D2A;	
        }
    } else {
        databuf[0] = BIT_EN_DREG_FIFO_D2A |
            BIT_TMST_TO_REGS_EN | 
            BIT_TMST_EN;  
    }
    res = icm406xx_share_write_register(REG_TMST_CONFIG, databuf, 1);
    if (res < 0) {
        ACC_ERR("write tmst register err!\n");
        return ICM406XX_ERR_BUS;
    }
    ACC_LOG("set power mode ok %d!\n", enable);
    return ICM406XX_SUCCESS;
}

static int icm406xx_EnableInterrupt(u8 int_type, bool enable)
{
    int res = 0;
    u8 databuf[2] = {0};

    res = icm406xx_share_read_register(REG_INT_SOURCE1, databuf, 1);
    if (res < 0) {
        ACC_ERR("read interrupt register err!\n");
        return ICM406XX_ERR_BUS;
    }
    switch (int_type) {
/*
    case ICM406XX_INT_TYPE_UI_AGC_RDY:
        break;
    case ICM406XX_INT_TYPE_FIFO_FULL:
        break;
    case ICM406XX_INT_TYPE_FIFO_THS:
        break;
    case ICM406XX_INT_TYPE_UI_DRDY:
        break;
    case ICM406XX_INT_TYPE_RESET_DONE:
        break;
    case ICM406XX_INT_TYPE_UI_FSYNC:
        break;
*/
    case ICM406XX_INT_TYPE_SMD:
        if(enable == true)
            databuf[0] |= BIT_INT_SMD_INT1_EN;
        else
            databuf[0] &= ~BIT_INT_SMD_INT1_EN;
        break;
    case ICM406XX_INT_TYPE_WOM:
        if(enable == true)
            databuf[0] |= BIT_INT_WOM_XYZ_INT1_EN;
        else
            databuf[0] &= ~BIT_INT_WOM_XYZ_INT1_EN;
        break;
    default:
        ACC_ERR("interrupt tyep %x is not supported", int_type);
        return -ICM406XX_ERR_INVALID_PARAM;
    }
    res = icm406xx_share_write_register(REG_INT_SOURCE1, databuf, 1);
    if (res < 0) {
        ACC_ERR("read interrupt register err!\n");
        return ICM406XX_ERR_BUS;
    }
    return ICM406XX_SUCCESS;
}

static int icm406xx_EnableSensor(int sensor_type, bool enable)
{
    u8 databuf[2] = {0};
    int res = 0;

    if(sensor_type >= ICM406XX_SENSOR_TYPE_MAX)
        return ICM406XX_ERR_INVALID_PARAM;
    icm406xx_all_sensor_status_info[sensor_type].sensor_power = enable;
    res = icm406xx_share_read_register(REG_PWR_MGMT_0, databuf, 1);
    if (res < 0) {
        ACC_ERR("read power mgmt register err!\n");
        return ICM406XX_ERR_BUS;
    }
    /* gyro based sensors */
    if (sensor_type == ICM406XX_SENSOR_TYPE_GYRO) {
        /* clear gyro enable */
        databuf[0] &= ~0x0C;
        if (enable == true) {
            if (icm406xx_all_sensor_status_info
                [ICM406XX_SENSOR_TYPE_GYRO].sample_rate > ICM406XX_LPM_MAX_RATE)
                databuf[0] |= BIT_GYRO_MODE_LNM;
            else
                databuf[0] |=
                    icm406xx_get_sensor_power_mode(ICM406XX_SENSOR_TYPE_GYRO);
        }
    }
    /* accel based sensors */
    if (sensor_type == ICM406XX_SENSOR_TYPE_ACC ||
        sensor_type == ICM406XX_SENSOR_TYPE_SC ||
        sensor_type == ICM406XX_SENSOR_TYPE_SD ||
        sensor_type == ICM406XX_SENSOR_TYPE_SMD
    ) {
        /* clear accel enable */
        databuf[0] &= ~0x03;
        if (enable == true || icm406xx_any_accel_based_sensor_is_on()) {
            if (icm406xx_all_sensor_status_info
                [ICM406XX_SENSOR_TYPE_ACC].sample_rate > ICM406XX_LPM_MAX_RATE)
                databuf[0] |= BIT_ACCEL_MODE_LNM;
            else
                databuf[0] |=
                    icm406xx_get_sensor_power_mode(ICM406XX_SENSOR_TYPE_ACC);
        }
    }
    res = icm406xx_share_write_register(REG_PWR_MGMT_0, databuf, 1);
    if (res < 0) {
        ACC_ERR("set power mgmt failed!\n");
        return ICM406XX_ERR_BUS;
    }
    if(enable == true)
        mdelay(1);
    return ICM406XX_SUCCESS;
}

static int icm406xx_ReadChipInfo(char *buf, int bufsize)
{
    u8 databuf[2] = {0};
    int res = 0;

    if ((NULL == buf) || (bufsize <= 30))
        return -1;
    res = icm406xx_share_read_register(REG_WHO_AM_I, databuf, 1);
    if (res < 0) {
        ACC_ERR("read who_am_i register err!\n");
        return ICM406XX_ERR_BUS;
    }
    switch (databuf[0]) {
    case WHO_AM_I_ICM406XX:
        sprintf(buf, "ICM406XX [0x%x]", databuf[0]);		
        break;
    default:
        sprintf(buf, "Unknown Sensor [0x%x]", databuf[0]);
        break;	
    }
    return ICM406XX_SUCCESS;
}

static int icm406xx_SetSampleRate(int sensor_type,
    u64 delay_ns, bool force_1khz)
{
    u8 databuf[2] = {0};
    int sample_rate = 0;
    int res = 0;
    int i, highest_sample_rate = 0;
    
    /* ns to us */
    sample_rate = (int)(div64_ul(delay_ns, 1000));
    /* us to ms */
    sample_rate = (int)(sample_rate / 1000);
    /* ms to hz */
    if(sample_rate != 0)
        sample_rate = (int)(1000 / sample_rate);
    /* sample rate: 5hz to 200hz; */
    /* when force_1khz is true, it means self test mode is running at 1khz */
    if ((sample_rate > 200) && (force_1khz == false))
        sample_rate = 200;
    else if (sample_rate < 25)
        sample_rate = 25;
    if(icm406xx_all_sensor_status_info[sensor_type].sample_rate == sample_rate)
        return ICM406XX_SUCCESS;
    icm406xx_all_sensor_status_info[sensor_type].sample_rate = sample_rate;
    if (sensor_type == ICM406XX_SENSOR_TYPE_GYRO) {
        res = icm406xx_share_read_register(REG_GYRO_CONFIG0, databuf, 1);
        if (res < 0) {
            ACC_ERR("read odr register err!\n");
            return ICM406XX_ERR_BUS;
        }
        /*
        b'0011: 8kHz 
        b'0100: 4kHz 
        b'0101: 2kHz 
        b'0110: 1kHz 
        b'0111: 200Hz
        b'1000: 100Hz 
        b'1001: 50Hz 
        b'1010: 25Hz 
        */
        databuf[0] &= ~BIT_GYRO_ODR;
        if (sample_rate > 200)
            databuf[0] |= 6;
        else if (sample_rate > 100)
            databuf[0] |= 7;
        else if (sample_rate > 50)
            databuf[0] |= 8;
        else if (sample_rate > 25)
            databuf[0] |= 9;
        else
            databuf[0] |= 10;
        res = icm406xx_share_write_register(REG_GYRO_CONFIG0, databuf, 1);
        if (res < 0) {
            ACC_ERR("write odr register err!\n");
            return ICM406XX_ERR_BUS;
        }
    }
    if (sensor_type == ICM406XX_SENSOR_TYPE_ACC ||
        sensor_type == ICM406XX_SENSOR_TYPE_SC ||
        sensor_type == ICM406XX_SENSOR_TYPE_SD ||
        sensor_type == ICM406XX_SENSOR_TYPE_SMD
    ) {
        /* check sample rate of enabled sensors */
        for(i = 0; i < ICM406XX_SENSOR_TYPE_MAX; i++) {
            if (i == ICM406XX_SENSOR_TYPE_GYRO)
                continue;
            if(icm406xx_all_sensor_status_info[i].sensor_power == true) {
                if(highest_sample_rate < 
                    icm406xx_all_sensor_status_info[i].sample_rate)
                    highest_sample_rate = 
                        icm406xx_all_sensor_status_info[i].sample_rate;
            }
        }
        res = icm406xx_share_read_register(REG_ACCEL_CONFIG0, databuf, 1);
        if (res < 0) {
            ACC_ERR("read odr register err!\n");
            return ICM406XX_ERR_BUS;
        }
        /*
        b'0011: 8kHz 
        b'0100: 4kHz 
        b'0101: 2kHz 
        b'0110: 1kHz 
        b'0111: 200Hz
        b'1000: 100Hz 
        b'1001: 50Hz 
        b'1010: 25Hz 
        */
        databuf[0] &= ~BIT_ACCEL_ODR;
        if (highest_sample_rate > 200)
            databuf[0] |= 6;
        else if (highest_sample_rate > 100)
            databuf[0] |= 7;
        else if (highest_sample_rate > 50)
            databuf[0] |= 8;
        else if (highest_sample_rate > 25)
            databuf[0] |= 9;
        else
            databuf[0] |= 10;
        res = icm406xx_share_write_register(REG_ACCEL_CONFIG0, databuf, 1);
        if (res < 0) {
            ACC_ERR("write odr register err!\n");
            return ICM406XX_ERR_BUS;
        }
    }
    return ICM406XX_SUCCESS;
}

/* wrappers */
void icm406xx_share_set_sensor_power_mode(int paramSensor, u8 paramPower)
{
    icm406xx_set_sensor_power_mode(paramSensor, paramPower);
}
EXPORT_SYMBOL(icm406xx_share_set_sensor_power_mode);

u8 icm406xx_share_get_sensor_power_mode(int paramSensor)
{
    return icm406xx_get_sensor_power_mode(paramSensor);
}
EXPORT_SYMBOL(icm406xx_share_get_sensor_power_mode);

bool icm406xx_share_get_sensor_power(int paramSensor)
{
    return icm406xx_get_sensor_power(paramSensor);
}
EXPORT_SYMBOL(icm406xx_share_get_sensor_power);

bool icm406xx_share_any_accel_based_sensor_is_on(void)
{	
    return icm406xx_any_accel_based_sensor_is_on();
}
EXPORT_SYMBOL(icm406xx_share_any_accel_based_sensor_is_on);

int icm406xx_share_ChipSoftReset(void)
{
    return icm406xx_ChipSoftReset();
}
EXPORT_SYMBOL(icm406xx_share_ChipSoftReset);

int icm406xx_share_SetPowerMode(int sensor_type, bool enable)
{
    return icm406xx_SetPowerMode(sensor_type, enable);
}
EXPORT_SYMBOL(icm406xx_share_SetPowerMode);

int icm406xx_share_EnableInterrupt(u8 int_type, bool enable)
{
    return icm406xx_EnableInterrupt(int_type, enable);
}
EXPORT_SYMBOL(icm406xx_share_EnableInterrupt);

int icm406xx_share_EnableSensor(int sensor_type, bool enable)
{
    return icm406xx_EnableSensor(sensor_type, enable);
}
EXPORT_SYMBOL(icm406xx_share_EnableSensor);

int icm406xx_share_ReadChipInfo(char *buf, int bufsize)
{
    return icm406xx_ReadChipInfo(buf, bufsize);
}
EXPORT_SYMBOL(icm406xx_share_ReadChipInfo);

int icm406xx_share_SetSampleRate(int sensor_type,
    u64 delay_ns, bool force_1khz)
{
    return icm406xx_SetSampleRate(sensor_type, delay_ns, force_1khz);
}
EXPORT_SYMBOL(icm406xx_share_SetSampleRate);
