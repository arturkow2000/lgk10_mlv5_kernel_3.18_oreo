/*
 * Definitions for akm09911 compass chip.
 */
#ifndef AKM09916_H
#define AKM09916_H

#include <linux/ioctl.h>

#define AKM09916_I2C_NAME "akm09916"

#define AKM09916_I2C_ADDRESS	0x18
#define AKM09916_BUFSIZE		0x50

/*! \name AK09916 register address
\anchor AK09916_REG
Defines a register address of the AK09916.*/
/*! @{*/
/* Device specific constant values */
#define AK09916_REG_WIA1			0x00
#define AK09916_REG_WIA2			0x01
#define AK09916_REG_RSV1			0x02
#define AK09916_REG_RSV2			0x03
#define AK09916_REG_ST1				0x10
#define AK09916_REG_HXL				0x11
#define AK09916_REG_HXH				0x12
#define AK09916_REG_HYL				0x13
#define AK09916_REG_HYH				0x14
#define AK09916_REG_HZL				0x15
#define AK09916_REG_HZH				0x16
#define AK09916_REG_TMPS			0x17
#define AK09916_REG_ST2				0x18
#define AK09916_REG_CNTL1			0x30
#define AK09916_REG_CNTL2			0x31
#define AK09916_REG_CNTL3			0x32

/*! \name AK09916 operation mode
 \anchor AK09916_Mode
 Defines an operation mode of the AK09911.*/
#define AK09916_MODE_SNG_MEASURE	0x01
#define AK09916_MODE_SELF_TEST		0x10
#define AK09916_MODE_POWERDOWN		0x00
#define AK09916_RESET_DATA			0x01

#define AK09916_REGS_SIZE		13
#define AK09916_WIA1_VALUE		0x48
#define AK09916_WIA2_VALUE		0x09

/* To avoid device dependency, convert to general name */
#define AKM_I2C_NAME			"akm09916"
#define AKM_MISCDEV_NAME		"akm09916_dev"
#define AKM_SYSCLS_NAME			"compass"
#define AKM_SYSDEV_NAME			"akm09916"
#define AKM_REG_MODE			AK09916_REG_CNTL2
#define AKM_REG_RESET			AK09916_REG_CNTL3
#define AKM_REG_STATUS			AK09916_REG_ST1
#define AKM_MEASURE_TIME_US		10000
#define AKM_DRDY_IS_HIGH(x)		((x) & 0x01)
#define AKM_SENSOR_INFO_SIZE	2
#define AKM_SENSOR_CONF_SIZE	3
#define AKM_SENSOR_DATA_SIZE	9

#define AKM_YPR_DATA_SIZE		26
#define AKM_RWBUF_SIZE			16
#define AKM_REGS_SIZE			AK09916_REGS_SIZE
#define AKM_REGS_1ST_ADDR		AK09916_REG_WIA1

#define SENSOR_DATA_SIZE		9	/* Rx buffer size, i.e from ST1 to ST2 */
#define RWBUF_SIZE				16	/* Read/Write buffer size. */
/* YPR_DATA_SIZE = CALIBRATION_DATA_SIZE, copy rbuf[] data to sensor_data[] */
#define CALIBRATION_DATA_SIZE	AKM_YPR_DATA_SIZE

#define AKM_MODE_SNG_MEASURE	AK09916_MODE_SNG_MEASURE
#define AKM_MODE_SELF_TEST		AK09916_MODE_SELF_TEST
#define AKM_MODE_POWERDOWN		AK09916_MODE_POWERDOWN
#define AKM_RESET_DATA			AK09916_RESET_DATA

#define ACC_DATA_FLAG               0
#define MAG_DATA_FLAG               1
#define MAGUC_DATA_FLAG             2
#define GYR_DATA_FLAG               3
#define FUSION_DATA_FLAG            4
#define AKM_NUM_SENSORS             5

#define ACC_DATA_READY              (1 << (ACC_DATA_FLAG))
#define MAG_DATA_READY              (1 << (MAG_DATA_FLAG))
#define MAGUC_DATA_READY            (1 << (MAGUC_DATA_FLAG))
#define GYR_DATA_READY              (1 << (GYR_DATA_FLAG))
#define FUSION_DATA_READY           (1 << (FUSION_DATA_FLAG))
/*! @}*/



/* conversion of magnetic data (for AK09911) to uT units */
/* #define CONVERT_M                   (1.0f*0.06f) */
/* conversion of orientation data to degree units */
/* #define CONVERT_O                   (1.0f/64.0f) */

#define CONVERT_M			6
#define CONVERT_M_DIV		100	/* 6/100 = CONVERT_M */
#define CONVERT_O			1
#define CONVERT_O_DIV		64	/* 1/64 = CONVERT_O */

#define CONVERT_Q16			1
#define CONVERT_Q16_DIV		65536	/* 1/64 = CONVERT_O */



#define CSPEC_SPI_USE			0
#define DBG_LEVEL0   0x0001	/* Critical */
#define DBG_LEVEL1   0x0002	/* Notice */
#define DBG_LEVEL2   0x0003	/* Information */
#define DBG_LEVEL3   0x0004	/* Debug */
#define DBGFLAG      DBG_LEVEL2

/*
//sensors_io.h need modify@junger
#define AKMIO                   0xA1

* IOCTLs for AKM library *
#define ECS_IOCTL_READ              _IOWR(AKMIO, 0x01, char*)
#define ECS_IOCTL_WRITE             _IOW(AKMIO, 0x02, char*)
#define ECS_IOCTL_SET_MODE          _IOW(AKMIO, 0x03, short)
#define ECS_IOCTL_GETDATA           _IOR(AKMIO, 0x04, char[SENSOR_DATA_SIZE])
#define ECS_IOCTL_SET_YPR           _IOW(AKMIO, 0x05, int[YPR_DATA_SIZE])
#define ECS_IOCTL_GET_OPEN_STATUS   _IOR(AKMIO, 0x06, int)
#define ECS_IOCTL_GET_CLOSE_STATUS  _IOR(AKMIO, 0x07, int)
#define ECS_IOCTL_GET_DELAY         _IOR(AKMIO, 0x08, long long int[AKM_NUM_SENSORS])
#define ECS_IOCTL_GET_LAYOUT        _IOR(AKMIO, 0x09, char)
#define ECS_IOCTL_GET_OUTBIT        _IOR(AKMIO, 0x0B, char)
#define ECS_IOCTL_RESET             _IO(AKMIO, 0x0C)
#define ECS_IOCTL_GET_ACCEL         _IOR(AKMIO, 0x30, short[3])
*/

/* IOCTLs for Msensor misc. device library */
#define MSENSOR						   0x83
/* IOCTLs for AKM library */
#define ECS_IOCTL_WRITE                 _IOW(MSENSOR, 0x0b, char*)
#define ECS_IOCTL_READ                  _IOWR(MSENSOR, 0x0c, char*)
#define ECS_IOCTL_RESET	        _IO(MSENSOR, 0x0d)	/* NOT used in AK8975 */
#define ECS_IOCTL_SET_MODE              _IOW(MSENSOR, 0x0e, short)
#define ECS_IOCTL_GETDATA               _IOR(MSENSOR, 0x0f, char[SENSOR_DATA_SIZE])
#define ECS_IOCTL_SET_YPR               _IOW(MSENSOR, 0x10, short[12])
#define ECS_IOCTL_GET_OPEN_STATUS       _IOR(MSENSOR, 0x11, int)
#define ECS_IOCTL_GET_CLOSE_STATUS      _IOR(MSENSOR, 0x12, int)
#define ECS_IOCTL_GET_OSENSOR_STATUS	_IOR(MSENSOR, 0x13, int)
#define ECS_IOCTL_GET_DELAY             _IOR(MSENSOR, 0x14, short)
#define ECS_IOCTL_GET_PROJECT_NAME      _IOR(MSENSOR, 0x15, char[64])
#define ECS_IOCTL_GET_MATRIX            _IOR(MSENSOR, 0x16, short [4][3][3])
#define	ECS_IOCTL_GET_LAYOUT			_IOR(MSENSOR, 0x17, int[3])

#define ECS_IOCTL_GET_ACCEL		_IOR(MSENSOR, 0x24, short[3])

#define ECS_IOCTL_GET_INFO			_IOR(MSENSOR, 0x27, unsigned char[AKM_SENSOR_INFO_SIZE])
#define ECS_IOCTL_GET_CONF			_IOR(MSENSOR, 0x28, unsigned char[AKM_SENSOR_CONF_SIZE])
#define ECS_IOCTL_SET_YPR_09916               _IOW(MSENSOR, 0x29, int[26])
#define ECS_IOCTL_GET_DELAY_09916             _IOR(MSENSOR, 0x30, int64_t[3])
#define	ECS_IOCTL_GET_LAYOUT_09916			_IOR(MSENSOR, 0x31, char)

#ifndef DBGPRINT
#define DBGPRINT(level, format, ...) \
	((((level) != 0) && ((level) <= DBGFLAG))  \
	 ? (pr_debug((format), ##__VA_ARGS__)) \
	 : (void)0)

#endif

struct akm09916_platform_data {
	char layout;
	char outbit;
	int gpio_DRDY;
	int gpio_RSTN;
};

/*** Limit of factory shipment test *******************************************/
#define TLIMIT_TN_REVISION_09916				""
#define TLIMIT_NO_RST_WIA1_09916				"1-3"
#define TLIMIT_TN_RST_WIA1_09916				"RST_WIA1"
#define TLIMIT_LO_RST_WIA1_09916				0x48
#define TLIMIT_HI_RST_WIA1_09916				0x48
#define TLIMIT_NO_RST_WIA2_09916				"1-4"
#define TLIMIT_TN_RST_WIA2_09916				"RST_WIA2"
#define TLIMIT_LO_RST_WIA2_09916				0x09
#define TLIMIT_HI_RST_WIA2_09916				0x09

#define TLIMIT_NO_SNG_ST1_09916				"2-3"
#define TLIMIT_TN_SNG_ST1_09916				"SNG_ST1"
#define TLIMIT_LO_SNG_ST1_09916				1
#define TLIMIT_HI_SNG_ST1_09916				1

#define TLIMIT_NO_SNG_HX_09916				"2-4"
#define TLIMIT_TN_SNG_HX_09916				"SNG_HX"
#define TLIMIT_LO_SNG_HX_09916				-32752
#define TLIMIT_HI_SNG_HX_09916				32752

#define TLIMIT_NO_SNG_HY_09916				"2-6"
#define TLIMIT_TN_SNG_HY_09916				"SNG_HY"
#define TLIMIT_LO_SNG_HY_09916				-32752
#define TLIMIT_HI_SNG_HY_09916				32752

#define TLIMIT_NO_SNG_HZ_09916				"2-8"
#define TLIMIT_TN_SNG_HZ_09916				"SNG_HZ"
#define TLIMIT_LO_SNG_HZ_09916				-32752
#define TLIMIT_HI_SNG_HZ_09916				32752

#define TLIMIT_NO_SNG_ST2_09916				"2-10"
#define TLIMIT_TN_SNG_ST2_09916				"SNG_ST2"
#define TLIMIT_LO_SNG_ST2_09916				0
#define TLIMIT_HI_SNG_ST2_09916				112

#define TLIMIT_NO_SLF_ST1_09916				"2-13"
#define TLIMIT_TN_SLF_ST1_09916				"SLF_ST1"
#define TLIMIT_LO_SLF_ST1_09916				1
#define TLIMIT_HI_SLF_ST1_09916				1

#define TLIMIT_NO_SLF_RVHX_09916			"2-14"
#define TLIMIT_TN_SLF_RVHX_09916			"SLF_REVSHX"
#define TLIMIT_LO_SLF_RVHX_09916			-200
#define TLIMIT_HI_SLF_RVHX_09916			200

#define TLIMIT_NO_SLF_RVHY_09916			"2-16"
#define TLIMIT_TN_SLF_RVHY_09916			"SLF_REVSHY"
#define TLIMIT_LO_SLF_RVHY_09916			-200
#define TLIMIT_HI_SLF_RVHY_09916			200

#define TLIMIT_NO_SLF_RVHZ_09916			"2-18"
#define TLIMIT_TN_SLF_RVHZ_09916			"SLF_REVSHZ"
#define TLIMIT_LO_SLF_RVHZ_09916			-1000
#define TLIMIT_HI_SLF_RVHZ_09916			-200

#define TLIMIT_NO_SLF_ST2_09916				"2-20"
#define TLIMIT_TN_SLF_ST2_09916				"SLF_ST2"
#define TLIMIT_LO_SLF_ST2_09916				0
#define TLIMIT_HI_SLF_ST2_09916				112

#endif
