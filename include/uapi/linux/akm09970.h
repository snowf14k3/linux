/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_AKM09970_H
#define _UAPI_LINUX_AKM09970_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define AKM09970_DRV_NAME	"akm09970"
#define AKM09970_CLASS_NAME	"akm"

#define AKM_IOC_MAGIC		'M'
#define AKM_PRIVATE		109

#define AK09970_REG_WIA		0x00
#define AK09970_REG_ST_XYZ	0x17
#define AK09970_REG_CNTL1	0x20
#define AK09970_REG_CNTL2	0x21
#define AK09970_REG_RESET	0x30
#define AK09970_RESET_DATA	0x01
#define AK09970_WIA1_VALUE	0x48
#define AK09970_WIA2_VALUE	0xc0

#define AK09970_MODE_POWERDOWN		0x00
#define AK09970_MODE_CONTINUOUS_10HZ	0x08
#define AK09970_MODE_CONTINUOUS_20HZ	0x0a
#define AK09970_MODE_CONTINUOUS_50HZ	0x0c
#define AK09970_MODE_CONTINUOUS_100HZ	0x0e

#define AKM_SENSOR_INFO_SIZE	2
#define AKM_SENSOR_CONF_SIZE	3
#define AKM_SENSOR_DATA_SIZE	8

#define AK09970_MODE_POS	0
#define AK09970_MODE_MSK	0x0f
#define AK09970_MODE_REG	AK09970_REG_CNTL2
#define AK09970_SDR_MODE_POS	4
#define AK09970_SDR_MODE_MSK	0x10
#define AK09970_SDR_MODE_REG	AK09970_REG_CNTL2
#define AK09970_SMR_MODE_POS	5
#define AK09970_SMR_MODE_MSK	0x20
#define AK09970_SMR_MODE_REG	AK09970_REG_CNTL2

#define AKM_DRDY_IS_HIGH(x)	((x) & 0x01)
#define AKM_DOR_IS_HIGH(x)	((x) & 0x02)
#define AKM_ERRADC_IS_HIGH(x)	((x) & 0x01)
#define AKM_ERRXY_IS_HIGH(x)	((x) & 0x80)
#define AK09970_SENS_Q16	((__s32)72090)
#define AKM_DRDY_TIMEOUT_MS	100
#define AKM_DEFAULT_MEASURE_HZ	10
#define AKM09970_VDD_MIN_UV	1800000
#define AKM09970_VDD_MAX_UV	1800000
#define PWM_PERIOD_DEFAULT_NS	1000000

/*
 * Keep the downstream ioctl payload layout. The eight data bytes contain
 * the AK09970 status and raw XYZ registers; position calibration belongs
 * to the consumer of this interface.
 */
struct akm09970_platform_data {
	__u8 sensor_smr;
	__u8 sensor_mode;
	__u8 sensor_state;
	__u8 data[AKM_SENSOR_DATA_SIZE];
};

#define AKM_IOC_SET_ACTIVE \
	_IOW(AKM_IOC_MAGIC, AKM_PRIVATE + 1, struct akm09970_platform_data)
#define AKM_IOC_SET_MODE \
	_IOW(AKM_IOC_MAGIC, AKM_PRIVATE + 2, struct akm09970_platform_data)
#define AKM_IOC_GET_SENSEDATA \
	_IOR(AKM_IOC_MAGIC, AKM_PRIVATE + 4, struct akm09970_platform_data)
#define AKM_IOC_GET_SENSSMR \
	_IOR(AKM_IOC_MAGIC, AKM_PRIVATE + 5, struct akm09970_platform_data)

/* New samples use CLOCK_BOOTTIME; GET_SAMPLE_SNAPSHOT returns -ENODATA
 * until a valid sample is available. The sequence increases on each sample.
 */
struct akm09970_sample_snapshot {
	__u64 timestamp_ns;
	__u32 sequence;
	__u8 data[AKM_SENSOR_DATA_SIZE];
	__u32 reserved;
};

#define AKM_IOC_GET_SAMPLE_SNAPSHOT \
	_IOR(AKM_IOC_MAGIC, AKM_PRIVATE + 6, struct akm09970_sample_snapshot)

#endif /* _UAPI_LINUX_AKM09970_H */
