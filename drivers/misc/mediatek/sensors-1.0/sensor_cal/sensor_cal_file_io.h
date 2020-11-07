#ifndef SENSOR_CAL_FILE_IO
#define SENSOR_CAL_FILE_IO

#define SENSOR_PROXIMITY_CAL_FILE "/vendor/persist-lg/sensor/proximity_cal_data.txt"
#define SENSOR_ACCEL_CAL_FILE     "/vendor/persist-lg/sensor/accel_cal_data.txt"
#define SENSOR_GYRO_CAL_FILE     "/vendor/persist-lg/sensor/gyro_cal_data.txt"
#define SENSOR_HISTORY_FILE 	"/vendor/persist-lg/sensor/sensor_history.txt"
#define DEFAULT_CAL_FILE_NAME   "/vendor/persist-lg/sensor/default_cal_data.txt"

int sensor_calibration_read(int sensor, int* cal);
int sensor_calibration_save(int sensor, int* cal);
//int make_sensor_cal_data_file(int sensor, char* fname);


#endif
