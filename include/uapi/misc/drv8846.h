/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_MISC_DRV8846_H
#define _UAPI_MISC_DRV8846_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define DRV8846_MISC_NAME "drv8846_dev"

#define DOWN 0
#define UP 1
#define DELTAMS 5

enum running_state {
	STILL = 0,
	SPEEDUP,
	FULLSTEAM,
	SLOWDOWN,
	UNIFORMSPEED,
};

struct op_parameter {
	__u32 dir;
	__u32 duration_ms;
	__u32 period_ns;
};

#define MOTOR_MAGIC 0xd3
/* Automatic run blocks until calibrated endpoint confirmation or failure. */
#define MOTOR_IOC_SET_AUTORUN		_IOW(MOTOR_MAGIC, 0x01, __u8)
#define MOTOR_IOC_SET_MANUALRUN		_IOW(MOTOR_MAGIC, 0x02, struct op_parameter)
#define MOTOR_IOC_STOP			_IO(MOTOR_MAGIC, 0x03)
#define MOTOR_IOC_GET_REMAIN_TIME	_IOR(MOTOR_MAGIC, 0x10, long)
#define MOTOR_IOC_GET_STATE		_IOR(MOTOR_MAGIC, 0x11, enum running_state)

/* Last powered cutoff; TIMED does not verify the camera position. */
#define DRV8846_STOP_UNKNOWN		0U
#define DRV8846_STOP_TIMED		1U
#define DRV8846_STOP_REQUESTED		2U
#define DRV8846_STOP_FAULT		3U
#define DRV8846_STOP_TIMEOUT		4U
#define DRV8846_STOP_ERROR		5U
#define DRV8846_STOP_ABORTED		6U

#define DRV8846_STOP_POSITION		7U
#define DRV8846_STOP_NO_POSITION		8U
#define DRV8846_STOP_HALL_ERROR		9U

#define DRV8846_POSITION_UNKNOWN	0U
#define DRV8846_POSITION_DOWN		1U
#define DRV8846_POSITION_UP		2U

/*
 * Privileged, explicit provisioning only. There are no built-in thresholds.
 * Userspace must verify calibration belongs to this exact assembled device
 * before setting DEVICE_VERIFIED. The kernel validates the format, bounds
 * and disjoint endpoint windows; it cannot authenticate their provenance.
 *
 * Each window contains measured signed raw XYZ limits in sensor register
 * order, before any Android layout transform, at SDR=0 and SMR=0. Index 0
 * is fully retracted, index 1 fully extended. Values must be measured on
 * this unit, including its repeatability/noise margin, not copied from
 * another unit. All axes must be bounded and endpoint boxes must not overlap.
 * Calibration is volatile: provision after each boot. No kernel file reads.
 */
#define DRV8846_CALIBRATION_VERSION	1U
#define DRV8846_CALIBRATION_DEVICE_VERIFIED	1U

struct drv8846_calibration {
	__u32 version;
	__u32 flags;
	__s32 minimum[2][3];
	__s32 maximum[2][3];
	__u32 reserved[4];
};

#define MOTOR_IOC_SET_CALIBRATION \
	_IOW(MOTOR_MAGIC, 0x20, struct drv8846_calibration)
#define MOTOR_IOC_CLEAR_CALIBRATION	_IO(MOTOR_MAGIC, 0x21)
/*
 * Both automatic and manual starts reject fault/timeout latches. Only this
 * privileged acknowledgment clears them, while idle with nFAULT deasserted.
 */
#define MOTOR_IOC_CLEAR_FAULTS		_IO(MOTOR_MAGIC, 0x23)
/* Requires two distinct, fresh samples; never reports a timed guess. */
#define MOTOR_IOC_GET_POSITION		_IOR(MOTOR_MAGIC, 0x22, __u32)

/* Emergency STOP retains camera exclusivity but cancels ready/retraction. */
#define DRV8846_CAMERA_OWNED		1U
#define DRV8846_CAMERA_CANCELLED		2U
#define MOTOR_IOC_GET_CAMERA_STATE	_IOR(MOTOR_MAGIC, 0x24, __u32)

#define MOTOR_IOC_GET_STOP_REASON	_IOR(MOTOR_MAGIC, 0x12, __u32)

#endif /* _UAPI_MISC_DRV8846_H */
