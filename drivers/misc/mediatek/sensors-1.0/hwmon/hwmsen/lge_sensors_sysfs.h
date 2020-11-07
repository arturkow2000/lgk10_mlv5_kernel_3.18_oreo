/* Copyright (c) 2016, LG Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef __LGE_SENSORS_SYSFS_H__
#define __LGE_SENSORS_SYSFS_H__

#define ACCEL              "accelerometer"
#define GYRO               "gyroscope"
#define PROX               "proximity"

#define CALIBRATION        "run_calibration"
#define SELFTEST           "selftest"
#define CALIBRATION_DATA   "calibration_data"

#define SENSORS_SYSFS_ACCEL "/sys/bus/platform/drivers/gsensor"
#define SENSORS_SYSFS_GYRO  "/sys/bus/platform/drivers/gyroscope"
#define SENSORS_SYSFS_PROX  "/sys/bus/platform/drivers/als_ps"

struct sensors_sysfs_array {
	const char *group;
	const char *user_node;
	const char *kernel_node;
	struct proc_dir_entry* parent;
};

const char *sensors_mandatory_paths[][2] = {
#ifdef CONFIG_CUSTOM_KERNEL_ACCELEROMETER
	{ ACCEL,   CALIBRATION },
	{ ACCEL,   SELFTEST },
#endif
#ifdef CONFIG_CUSTOM_KERNEL_GYROSCOPE
	{ GYRO,    CALIBRATION },
	{ GYRO,    SELFTEST },
#endif
#ifdef CONFIG_CUSTOM_KERNEL_ALSPS
	{ PROX,    CALIBRATION },
	{ PROX,    CALIBRATION_DATA },
#endif
};

/* unused
const char *group_names[] = {
	ACCEL,
	GYRO,
	PROX,
};
*/

/* Set sysfs node for non-using DT */
const char *default_sensors_sysfs_path[][3] = {
#ifdef CONFIG_CUSTOM_KERNEL_ACCELEROMETER
	{ ACCEL,   CALIBRATION,      SENSORS_SYSFS_ACCEL"/run_fast_calibration" },
	{ ACCEL,   SELFTEST,         SENSORS_SYSFS_ACCEL"/"SELFTEST },
#endif
#ifdef CONFIG_CUSTOM_KERNEL_GYROSCOPE
	{ GYRO,    CALIBRATION,      SENSORS_SYSFS_GYRO"/run_fast_calibration" },
	{ GYRO,    SELFTEST,         SENSORS_SYSFS_GYRO"/"SELFTEST },
#endif
#ifdef CONFIG_CUSTOM_KERNEL_ALSPS
	{ PROX,    CALIBRATION,      SENSORS_SYSFS_PROX"/"CALIBRATION },
	{ PROX,    CALIBRATION_DATA, SENSORS_SYSFS_PROX"/"CALIBRATION_DATA },
#endif
};
#endif
