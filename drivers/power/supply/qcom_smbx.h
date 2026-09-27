/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef QCOM_SMBX_H
#define QCOM_SMBX_H

#include <linux/power_supply.h>

/* The caller must validate PPS, battery temperature and pump faults first. */
int qcom_smbx_set_charge_pump(struct power_supply *main_psy, bool enable);
int qcom_smbx_set_charge_pump_input_limit(struct power_supply *main_psy,
						 int ua);
int qcom_smbx_get_charge_pump_fcc_max(struct power_supply *main_psy,
					     int *max_ua);
int qcom_smbx_set_charge_pump_fcc(struct power_supply *main_psy, int ua);

#endif /* QCOM_SMBX_H */
