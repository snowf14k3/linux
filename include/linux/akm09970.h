/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_AKM09970_H
#define _LINUX_AKM09970_H

#include <linux/err.h>
#include <linux/kconfig.h>
#include <uapi/linux/akm09970.h>

struct device;
struct akm09970;

#if IS_REACHABLE(CONFIG_AKM09970)
struct akm09970 *akm09970_get(struct device *consumer, const char *property);
void akm09970_put(struct akm09970 *sensor);
int akm09970_acquire(struct akm09970 *sensor);
void akm09970_release(struct akm09970 *sensor);
int akm09970_snapshot(struct akm09970 *sensor,
		     struct akm09970_sample_snapshot *snapshot);
#else
static inline struct akm09970 *akm09970_get(struct device *consumer,
					 const char *property)
{
	return ERR_PTR(-ENODEV);
}
static inline void akm09970_put(struct akm09970 *sensor) { }
static inline int akm09970_acquire(struct akm09970 *sensor)
{
	return -ENODEV;
}
static inline void akm09970_release(struct akm09970 *sensor) { }
static inline int akm09970_snapshot(struct akm09970 *sensor,
				   struct akm09970_sample_snapshot *snapshot)
{
	return -ENODEV;
}
#endif
#endif /* _LINUX_AKM09970_H */
