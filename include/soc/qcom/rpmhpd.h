/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SOC_QCOM_RPMHPD_H
#define __SOC_QCOM_RPMHPD_H

#include <linux/kconfig.h>

#if IS_REACHABLE(CONFIG_QCOM_RPMHPD)
int qcom_rpmhpd_wait_sync(unsigned long timeout_ms);
#else
static inline int qcom_rpmhpd_wait_sync(unsigned long timeout_ms)
{
	return 0;
}
#endif

#endif /* __SOC_QCOM_RPMHPD_H */
