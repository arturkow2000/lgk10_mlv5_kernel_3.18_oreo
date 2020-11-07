#ifndef SENSOR_CAL_FILE_IO
#define SENSOR_CAL_FILE_IO

#define SENSOR_PROXIMITY_CAL_FILE "/persist-lg/sensor/proximity_cal_data.txt"
#define SENSOR_ACCEL_CAL_FILE     "/persist-lg/sensor/accel_cal_data.txt"
#define SENSOR_GYRO_CAL_FILE     "/persist-lg/sensor/gyro_cal_data.txt"
#define SENSOR_HISTORY_FILE 	"/persist-lg/sensor/sensor_history.txt"
#define DEFAULT_CAL_FILE_NAME   "/persist-lg/sensor/default_cal_data.txt"

int sensor_calibration_read(int sensor, int* cal);
int sensor_calibration_save(int sensor, int* cal);
int sensor_history_save(int sensor, const char* __event, const char* __msg);

#endif
