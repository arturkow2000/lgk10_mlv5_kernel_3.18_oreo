/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/* s_add new sensor driver here */
/* export functions */

//Main Sensor
UINT32 HI1333_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc); // ADD rpp
UINT32 S5K3P8SX_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 HI1332_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 IMX258_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
//Sub Sensor
UINT32 HI553_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 HI846_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 HI556_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 S5K5E8YX_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);

/* ! Add Sensor Init function here */
/* ! Note: */
/* ! 1. Add by the resolution from ""large to small"", due to large sensor */
/* !    will be possible to be main sensor. */
/* !    This can avoid I2C error during searching sensor. */
/* ! 2. This file should be the same as mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp */
ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT kdSensorList[MAX_NUM_OF_SUPPORT_SENSOR + 1] = {

//Main Sensor
#if defined(HI1332_MIPI_RAW)
    {HI1332_SENSOR_ID, SENSOR_DRVNAME_HI1332_MIPI_RAW, HI1332_MIPI_RAW_SensorInit},
#endif
#if defined(HI1333_MIPI_RAW)
    {HI1333_SENSOR_ID, SENSOR_DRVNAME_HI1333_MIPI_RAW, HI1333_MIPI_RAW_SensorInit},
#endif
#if defined(S5K3P8SX_MIPI_RAW)
	{S5K3P8SX_SENSOR_ID, SENSOR_DRVNAME_S5K3P8SX_MIPI_RAW, S5K3P8SX_MIPI_RAW_SensorInit},
#endif
#if defined(IMX258_MIPI_RAW)
    {IMX258_SENSOR_ID, SENSOR_DRVNAME_IMX258_MIPI_RAW, IMX258_MIPI_RAW_SensorInit},
#endif

//Sub Sensor
#if defined(HI553_MIPI_RAW)
    {HI553_SENSOR_ID, SENSOR_DRVNAME_HI553_MIPI_RAW, HI553_MIPI_RAW_SensorInit},
#endif
#if defined(HI846_MIPI_RAW)
	{HI846_SENSOR_ID, SENSOR_DRVNAME_HI846_MIPI_RAW, HI846_MIPI_RAW_SensorInit},
#endif
#if defined(HI556_MIPI_RAW)
    {HI556_SENSOR_ID, SENSOR_DRVNAME_HI556_MIPI_RAW, HI556_MIPI_RAW_SensorInit},
#endif
#if defined(S5K5E8YX_MIPI_RAW)
    {S5K5E8YX_SENSOR_ID, SENSOR_DRVNAME_S5K5E8YX_MIPI_RAW, S5K5E8YX_MIPI_RAW_SensorInit},
#endif

/*  ADD sensor driver before this line */
	{0, {0}
	 , NULL}
	,			/* end of list */
};

/* e_add new sensor driver here */
