/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_DRV8846_H
#define _LINUX_DRV8846_H

#include <linux/err.h>
#include <linux/kconfig.h>

struct device;
struct drv8846;

/*
 * get/put hold the supplier's memory, device and module lifetime. All camera
 * calls may sleep. Pair one successful open with close or abort. close consumes
 * ownership even on failure; abort cuts power without attempting retraction.
 * Userspace STOP cancels ready/retraction, retaining exclusivity until close.
 * No API supplies default calibration or silently homes an unknown position.
 */
#if IS_REACHABLE(CONFIG_TI_DRV8846)
struct drv8846 *drv8846_get(struct device *consumer, const char *property);
void drv8846_put(struct drv8846 *motor);
int drv8846_camera_open(struct drv8846 *motor);
/* Verify current UP position without moving, before each stream start. */
int drv8846_camera_ready(struct drv8846 *motor);
int drv8846_camera_close(struct drv8846 *motor);
void drv8846_camera_abort(struct drv8846 *motor);
#else
static inline struct drv8846 *drv8846_get(struct device *consumer,
					const char *property)
{
	return ERR_PTR(-ENODEV);
}
static inline void drv8846_put(struct drv8846 *motor) { }
static inline int drv8846_camera_open(struct drv8846 *motor)
{
	return -ENODEV;
}
static inline int drv8846_camera_ready(struct drv8846 *motor)
{
	return -ENODEV;
}
static inline int drv8846_camera_close(struct drv8846 *motor)
{
	return -ENODEV;
}
static inline void drv8846_camera_abort(struct drv8846 *motor) { }
#endif
#endif /* _LINUX_DRV8846_H */
