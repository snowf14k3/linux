/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef QCOM_SMBX_H
#define QCOM_SMBX_H

#include <linux/power_supply.h>

/* The caller must validate PPS, battery temperature and pump faults first. */
int qcom_smbx_set_charge_pump(struct power_supply *main_psy, bool enable);

#endif /* QCOM_SMBX_H */
