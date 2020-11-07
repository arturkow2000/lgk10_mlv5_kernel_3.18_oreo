/* 
 * ICM206XX sensor driver
 * Copyright (C) 2016 Invensense, Inc.
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

#ifndef ICM206XX_SC_H
#define ICM206XX_SC_H

#define ICM406XX_WOM_THRESHOLD_TUNE

/* 50Hz */
#define DEFAULT_SAMPLING_PERIOD_NS	(NSEC_PER_SEC / 50)

/* Wake-on-Motion/No-Motion */
#define ICM406XX_WOM_COUNT 200 // 200 samples
#define ICM406XX_WOM_COMPUTE(val_mg) ((256 * val_mg) / 1000)
#define ICM406XX_WOM_DEFAULT_THRESHOLD 25

/* INT configurations */
// Polarity: 0 -> Active Low, 1 -> Active High
#define INT_POLARITY    1
// Drive circuit: 0 -> Open Drain, 1 -> Push-Pull
#define INT_DRIVE_CIRCUIT    1
// Mode: 0 -> Pulse, 1 -> Latch
#define INT_MODE    0

#define SIG_ICM406XX    44
#define ICM406XX_IOCTL_GROUP    0x10
#define ICM406XX_WRITE_DAEMON_PID    _IO(ICM406XX_IOCTL_GROUP, 1)
#define ICM406XX_READ_SENSOR_DATA    _IO(ICM406XX_IOCTL_GROUP, 2)
#define ICM406XX_WRITE_SENSOR_DATA    _IO(ICM406XX_IOCTL_GROUP, 3)
#define ICM406XX_LOG    _IO(ICM406XX_IOCTL_GROUP, 15)

#define REQUEST_SIGNAL_PROCESS_DATA    0x01

#endif /* ICM206XX_SC_H */

