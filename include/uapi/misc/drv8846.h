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

#define MOTOR_IOC_GET_STOP_REASON	_IOR(MOTOR_MAGIC, 0x12, __u32)

#endif /* _UAPI_MISC_DRV8846_H */
