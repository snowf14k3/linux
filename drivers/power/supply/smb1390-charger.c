// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm SMB1390 auxiliary charge pump
 *
 * Register layout and fault bits are derived from the downstream
 * smb1390-charger driver. The switcher starts disabled and is only enabled
 * after the main charger and USB input policy pass their checks.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "qcom_smbx.h"

#define SMB1390_CORE_STATUS1_REG		0x1006
#define SMB1390_WIN_OV_BIT		BIT(0)
#define SMB1390_WIN_UV_BIT		BIT(1)
#define SMB1390_ILIM_BIT			BIT(5)
#define SMB1390_TEMP_ALARM_BIT		BIT(6)
#define SMB1390_VPH_OV_SOFT_BIT		BIT(7)

#define SMB1390_CORE_STATUS2_REG		0x1007
#define SMB1390_SWITCHER_HOLD_OFF_BIT	BIT(0)
#define SMB1390_VPH_OV_HARD_BIT		BIT(1)
#define SMB1390_TSD_BIT			BIT(2)
#define SMB1390_IREV_BIT		BIT(3)
#define SMB1390_IOC_BIT			BIT(4)
#define SMB1390_VIN_UV_BIT		BIT(5)
#define SMB1390_VIN_OV_BIT		BIT(6)
#define SMB1390_EN_PIN_OUT2_BIT		BIT(7)

#define SMB1390_CORE_CONTROL1_REG	0x1020
#define SMB1390_CMD_EN_SWITCHER_BIT	BIT(0)

#define SMB1390_CORE_FTRIM_CTRL_REG	0x1031
#define SMB1390_CORE_FTRIM_ILIM_REG	0x1030
#define SMB1390_CORE_FTRIM_LVL_REG	0x1033
#define SMB1390_CORE_FTRIM_MISC_REG	0x1034
#define SMB1390_WIN_OV_LEVEL_MASK	GENMASK(3, 2)
#define SMB1390_WIN_OV_LEVEL_1V		BIT(3)
#define SMB1390_TRACKING_1P5X_BIT	BIT(0)
#define SMB1390_IREV_THRESHOLD_BIT	BIT(1)
#define SMB1390_ILIM_MASK		GENMASK(4, 0)
#define SMB1390_ILIM_MIN_UA		1000000
#define SMB1390_ILIM_MAX_UA		3200000
#define SMB1390_ILIM_BASE_UA		500000
#define SMB1390_ILIM_STEP_UA		100000

/* Values used by tcpm_psy_online_states in drivers/usb/typec/tcpm/tcpm.c. */
#define SMB1390_PPS_ONLINE		2
#define SMB1390_FIXED_ONLINE		1
/* Conservative software limits; PMIC hardware protection remains active. */
#define SMB1390_POLICY_MIN_BATT_DECIC	150
#define SMB1390_POLICY_MAX_BATT_DECIC	400
#define SMB1390_POLICY_MAX_SOC		85
#define SMB1390_POLICY_MAX_DIE_MC	70000
#define SMB1390_POLICY_MAX_CONNECTOR_MC	50000
#define SMB1390_POLICY_MAX_SKIN_MC	45000
#define SMB1390_POLICY_MIN_VBAT_UV	3400000
#define SMB1390_POLICY_MAX_VBAT_UV	4300000
#define SMB1390_POLICY_HEADROOM_UV	500000
#define SMB1390_POLICY_MIN_VBUS_UV	7000000
#define SMB1390_POLICY_MAX_VBUS_UV	10000000
#define SMB1390_POLICY_VBUS_TOLERANCE_UV	500000
#define SMB1390_POLICY_MAX_INPUT_UA	2800000
#define SMB1390_POLICY_MIN_INPUT_UA	1500000
#define SMB1390_POLICY_PROVISIONAL_ICL_UA	500000
#define SMB1390_POLICY_MAX_FCC_UA	5100000
#define SMB1390_POLICY_MID_FCC_UA	3000000
#define SMB1390_POLICY_EDGE_FCC_UA	1000000
#define SMB1390_POLICY_ACTIVE_MS	1000
#define SMB1390_POLICY_IDLE_MS	5000

#define SMB1390_INT_SUMMARY_REG		0x0550
#define SMB1390_CORE_INT_SET_TYPE_REG	0x1011
#define SMB1390_CORE_INT_POL_HIGH_REG	0x1012
#define SMB1390_CORE_INT_POL_LOW_REG	0x1013
#define SMB1390_CORE_INT_LATCHED_CLR_REG	0x1014
#define SMB1390_CORE_INT_EN_SET_REG	0x1015
#define SMB1390_CORE_INT_EN_CLR_REG	0x1016
#define SMB1390_CORE_INT_LATCHED_STS_REG	0x1018
#define SMB1390_CORE_SUMMARY_BIT		BIT(0)
#define SMB1390_FAULT_IRQ_MASK		(GENMASK(5, 1) | BIT(7))
#define SMB1390_ILIM_IRQ_BIT		BIT(6)
#define SMB1390_CORE_IRQ_MASK		(SMB1390_FAULT_IRQ_MASK | \
					 SMB1390_ILIM_IRQ_BIT)
#define SMB1390_FAULT_IRQ_BOTH_EDGES	GENMASK(4, 1)

struct smb1390 {
	struct device *dev;
	struct regmap *regmap;
	struct iio_channel *die_temp;
	struct iio_channel *connector_temp;
	struct iio_channel *skin_temp;
	struct power_supply *pd_psy;
	struct power_supply *main_psy;
	struct power_supply *battery_psy;
	struct power_supply *psy;
	int irq;
	bool irq_faulted;
	bool hw_configured;
	bool policy_enabled;
	bool policy_active;
	bool policy_faulted;
	bool pps_owned;
	u32 target_vbus_uv;
	u32 target_input_ua;
	u32 max_input_ua;
	u8 ilim_retries;
	struct mutex policy_lock;
	struct delayed_work policy_work;
};

struct smb1390_reg_setting {
	u16 reg;
	u8 value;
};

/* Downstream SMB1390 rev2/3 configuration for one auxiliary pump. */
static const struct smb1390_reg_setting smb1390_dual_settings[] = {
	/* The final 0x1031 value selects the downstream 85 C alert level. */
	{ SMB1390_CORE_FTRIM_CTRL_REG, 0x7a },
	{ 0x1032, 0x07 },
	{ 0x1035, 0x63 },
	{ 0x1036, 0x80 },
	{ 0x103a, 0x44 },
};

static const struct regmap_config smb1390_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x10ff,
};

static int smb1390_disable_switcher(struct smb1390 *chip)
{
	unsigned int control;
	int ret;

	ret = regmap_update_bits(chip->regmap, SMB1390_CORE_CONTROL1_REG,
				 SMB1390_CMD_EN_SWITCHER_BIT, 0);
	if (ret)
		return ret;

	ret = regmap_read(chip->regmap, SMB1390_CORE_CONTROL1_REG, &control);
	if (ret)
		return ret;

	return control & SMB1390_CMD_EN_SWITCHER_BIT ? -EIO : 0;
}

static int smb1390_write_verify(struct smb1390 *chip, unsigned int reg,
				unsigned int mask, unsigned int value)
{
	unsigned int readback;
	int ret;

	ret = regmap_update_bits(chip->regmap, reg, mask, value);
	if (ret)
		return ret;

	ret = regmap_read(chip->regmap, reg, &readback);
	if (ret)
		return ret;

	return (readback & mask) == (value & mask) ? 0 : -EIO;
}

static int smb1390_set_ilim(struct smb1390 *chip, unsigned int ua)
{
	if (ua < SMB1390_ILIM_MIN_UA || ua > SMB1390_ILIM_MAX_UA ||
	    (ua - SMB1390_ILIM_BASE_UA) % SMB1390_ILIM_STEP_UA)
		return -EINVAL;

	return smb1390_write_verify(chip, SMB1390_CORE_FTRIM_ILIM_REG,
				    SMB1390_ILIM_MASK,
				    (ua - SMB1390_ILIM_BASE_UA) /
				    SMB1390_ILIM_STEP_UA);
}

static int smb1390_configure_dual(struct smb1390 *chip)
{
	unsigned int control;
	size_t i;
	int ret;

	chip->hw_configured = false;
	ret = regmap_read(chip->regmap, SMB1390_CORE_CONTROL1_REG, &control);
	if (ret)
		return ret;
	if (control & SMB1390_CMD_EN_SWITCHER_BIT)
		return -EBUSY;

	ret = smb1390_write_verify(chip, SMB1390_CORE_FTRIM_LVL_REG,
				   SMB1390_WIN_OV_LEVEL_MASK,
				   SMB1390_WIN_OV_LEVEL_1V);
	if (ret)
		return ret;
	ret = smb1390_write_verify(chip, SMB1390_CORE_FTRIM_MISC_REG,
				   SMB1390_TRACKING_1P5X_BIT |
				   SMB1390_IREV_THRESHOLD_BIT, 0);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(smb1390_dual_settings); i++) {
		ret = smb1390_write_verify(chip, smb1390_dual_settings[i].reg,
					   0xff, smb1390_dual_settings[i].value);
		if (ret)
			return ret;
	}

	ret = smb1390_set_ilim(chip, SMB1390_ILIM_MIN_UA);
	if (ret)
		return ret;

	ret = regmap_read(chip->regmap, SMB1390_CORE_CONTROL1_REG, &control);
	if (ret)
		return ret;
	if (control & SMB1390_CMD_EN_SWITCHER_BIT)
		return -EIO;

	chip->hw_configured = true;
	return 0;
}

static int smb1390_switcher_online(struct smb1390 *chip, bool *online)
{
	unsigned int status2, control;
	int ret;

	ret = regmap_read(chip->regmap, SMB1390_CORE_STATUS2_REG, &status2);
	if (ret)
		return ret;

	ret = regmap_read(chip->regmap, SMB1390_CORE_CONTROL1_REG, &control);
	if (ret)
		return ret;

	*online = !!(control & SMB1390_CMD_EN_SWITCHER_BIT) &&
		  !!(status2 & SMB1390_EN_PIN_OUT2_BIT) &&
		  !(status2 & SMB1390_SWITCHER_HOLD_OFF_BIT);
	return 0;
}

static irqreturn_t smb1390_fault_irq(int irq, void *data)
{
	struct smb1390 *chip = data;
	unsigned int summary, latched;
	int ret;

	ret = regmap_read(chip->regmap, SMB1390_INT_SUMMARY_REG, &summary);
	if (ret)
		goto fail;
	if (!(summary & SMB1390_CORE_SUMMARY_BIT))
		return IRQ_NONE;

	ret = regmap_read(chip->regmap, SMB1390_CORE_INT_LATCHED_STS_REG,
			  &latched);
	if (ret)
		goto fail;

	ret = regmap_write(chip->regmap, SMB1390_CORE_INT_LATCHED_CLR_REG,
			   latched);
	if (ret)
		goto fail;
	if (!latched) {
		ret = -EIO;
		goto fail;
	}
	if (!(latched & SMB1390_FAULT_IRQ_MASK)) {
		power_supply_changed(chip->psy);
		if (READ_ONCE(chip->policy_enabled))
			mod_delayed_work(system_wq, &chip->policy_work, 0);
		return IRQ_HANDLED;
	}

	WRITE_ONCE(chip->irq_faulted, true);
	ret = smb1390_disable_switcher(chip);
	if (ret)
		dev_err_ratelimited(chip->dev,
				    "failed to stop switcher after fault: %d\n", ret);
	if (chip->main_psy) {
		ret = qcom_smbx_set_charge_pump(chip->main_psy, false);
		if (ret)
			dev_err_ratelimited(chip->dev,
					    "failed to force SMB_EN low: %d\n", ret);
	}
	ret = regmap_write(chip->regmap, SMB1390_CORE_INT_EN_CLR_REG, 0xff);
	if (ret)
		disable_irq_nosync(irq);
	power_supply_changed(chip->psy);
	if (READ_ONCE(chip->policy_enabled))
		mod_delayed_work(system_wq, &chip->policy_work, 0);
	return IRQ_HANDLED;

fail:
	WRITE_ONCE(chip->irq_faulted, true);
	if (smb1390_disable_switcher(chip))
		dev_err_ratelimited(chip->dev,
				    "failed to stop switcher after IRQ read error\n");
	if (chip->main_psy &&
	    qcom_smbx_set_charge_pump(chip->main_psy, false))
		dev_err_ratelimited(chip->dev, "failed to force SMB_EN low\n");
	regmap_write(chip->regmap, SMB1390_CORE_INT_EN_CLR_REG, 0xff);
	disable_irq_nosync(irq);
	dev_err_ratelimited(chip->dev, "IRQ handling failed: %d\n", ret);
	power_supply_changed(chip->psy);
	if (READ_ONCE(chip->policy_enabled))
		mod_delayed_work(system_wq, &chip->policy_work, 0);
	return IRQ_HANDLED;
}

static struct power_supply *smb1390_get_supply(struct device *dev,
					       const char *property)
{
	struct power_supply *psy;

	psy = devm_power_supply_get_by_reference(dev, property);
	if (!psy)
		return ERR_PTR(-EPROBE_DEFER);

	return psy;
}

static int smb1390_enable_fault_irq(struct smb1390 *chip)
{
	unsigned int enabled;
	int ret;

	/* Raphael maps only core peripheral 0x10; bit 0 is off-window. */
	ret = regmap_write(chip->regmap, SMB1390_CORE_INT_EN_CLR_REG, 0xff);
	if (ret)
		return ret;
	ret = regmap_write(chip->regmap, SMB1390_CORE_INT_LATCHED_CLR_REG,
			   0xff);
	if (ret)
		return ret;

	ret = regmap_update_bits(chip->regmap, SMB1390_CORE_INT_SET_TYPE_REG,
				 SMB1390_CORE_IRQ_MASK, SMB1390_CORE_IRQ_MASK);
	if (ret)
		return ret;
	ret = regmap_update_bits(chip->regmap, SMB1390_CORE_INT_POL_HIGH_REG,
				 SMB1390_CORE_IRQ_MASK, SMB1390_CORE_IRQ_MASK);
	if (ret)
		return ret;
	ret = regmap_update_bits(chip->regmap, SMB1390_CORE_INT_POL_LOW_REG,
				 SMB1390_CORE_IRQ_MASK,
				 SMB1390_FAULT_IRQ_BOTH_EDGES);
	if (ret)
		return ret;

	ret = regmap_write(chip->regmap, SMB1390_CORE_INT_EN_SET_REG,
			   SMB1390_CORE_IRQ_MASK);
	if (ret)
		return ret;
	ret = regmap_read(chip->regmap, SMB1390_CORE_INT_EN_SET_REG,
			  &enabled);
	if (ret)
		return ret;
	return (enabled & SMB1390_CORE_IRQ_MASK) ==
		SMB1390_CORE_IRQ_MASK ? 0 : -EIO;
}

static int smb1390_get_policy_supplies(struct smb1390 *chip,
				       struct device *dev)
{
	if (!device_property_present(dev, "usb-pd-supply")) {
		if (device_property_present(dev, "main-charger-supply") ||
		    device_property_present(dev, "battery-supply"))
			return -EINVAL;
		return 0;
	}

	if (!device_property_present(dev, "main-charger-supply") ||
	    !device_property_present(dev, "battery-supply"))
		return -EINVAL;

	chip->pd_psy = smb1390_get_supply(dev, "usb-pd-supply");
	if (IS_ERR(chip->pd_psy))
		return PTR_ERR(chip->pd_psy);

	chip->main_psy = smb1390_get_supply(dev, "main-charger-supply");
	if (IS_ERR(chip->main_psy))
		return PTR_ERR(chip->main_psy);

	chip->battery_psy = smb1390_get_supply(dev, "battery-supply");
	if (IS_ERR(chip->battery_psy))
		return PTR_ERR(chip->battery_psy);

	return 0;
}

static const enum power_supply_property smb1390_properties[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int smb1390_get_property(struct power_supply *psy,
				 enum power_supply_property prop,
				 union power_supply_propval *val)
{
	struct smb1390 *chip = power_supply_get_drvdata(psy);
	unsigned int status1, status2, control;
	bool online;
	int temp, ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_HEALTH:
		ret = regmap_read(chip->regmap, SMB1390_CORE_STATUS1_REG,
				  &status1);
		if (ret)
			return ret;

		ret = regmap_read(chip->regmap, SMB1390_CORE_STATUS2_REG,
				  &status2);
		if (ret)
			return ret;

		ret = regmap_read(chip->regmap, SMB1390_CORE_CONTROL1_REG,
				  &control);
		if (ret)
			return ret;

		if ((status1 & SMB1390_TEMP_ALARM_BIT) ||
		    (status2 & SMB1390_TSD_BIT))
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if ((status1 & (SMB1390_WIN_OV_BIT |
				     SMB1390_VPH_OV_SOFT_BIT)) ||
			 (status2 & (SMB1390_VPH_OV_HARD_BIT |
				     SMB1390_VIN_OV_BIT)))
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if ((control & SMB1390_CMD_EN_SWITCHER_BIT) &&
			 ((status1 & SMB1390_WIN_UV_BIT) ||
			  (status2 & SMB1390_VIN_UV_BIT)))
			val->intval = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
		else if (status2 & (SMB1390_IREV_BIT | SMB1390_IOC_BIT))
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else if (READ_ONCE(chip->irq_faulted))
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		/* An unexpected switcher enable is a fault. */
		else if ((control & SMB1390_CMD_EN_SWITCHER_BIT) &&
			 !READ_ONCE(chip->policy_active))
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;

		return 0;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = regmap_read(chip->regmap, SMB1390_CORE_FTRIM_ILIM_REG,
				  &control);
		if (ret)
			return ret;
		val->intval = SMB1390_ILIM_BASE_UA +
			      (control & SMB1390_ILIM_MASK) *
			      SMB1390_ILIM_STEP_UA;
		return 0;

	case POWER_SUPPLY_PROP_ONLINE:
		ret = smb1390_switcher_online(chip, &online);
		if (ret)
			return ret;

		val->intval = online;
		return 0;

	case POWER_SUPPLY_PROP_TEMP:
		if (READ_ONCE(chip->irq_faulted))
			return -EIO;

		ret = smb1390_switcher_online(chip, &online);
		if (ret)
			return ret;

		/* The ADC is not reliable while the charge pump is disabled. */
		if (!online)
			return -ENODATA;

		ret = iio_read_channel_processed(chip->die_temp, &temp);
		if (ret)
			return ret;

		/* ADC5 reports millidegrees; power_supply uses deci-Celsius. */
		val->intval = temp / 100;
		return 0;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "smb1390";
		return 0;

	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc smb1390_psy_desc = {
	.name = "smb1390",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = smb1390_properties,
	.num_properties = ARRAY_SIZE(smb1390_properties),
	.get_property = smb1390_get_property,
};

struct smb1390_policy_data {
	int vbat_uv;
	int battery_temp;
	int connector_temp_mc;
	int skin_temp_mc;
	int main_icl_ua;
};

static int smb1390_psy_get_int(struct power_supply *psy,
			       enum power_supply_property prop, int *value)
{
	union power_supply_propval result;
	int ret;

	ret = power_supply_get_property(psy, prop, &result);
	if (!ret)
		*value = result.intval;
	return ret;
}

static int smb1390_psy_set_int(struct power_supply *psy,
			       enum power_supply_property prop, int value)
{
	union power_supply_propval request = { .intval = value };

	return power_supply_set_property(psy, prop, &request);
}

static int smb1390_policy_precheck(struct smb1390 *chip,
				   struct smb1390_policy_data *data)
{
	int present, soc, health, online, type, ret;

	if (!chip->hw_configured || !chip->irq ||
	    READ_ONCE(chip->irq_faulted) || chip->policy_faulted)
		return -EIO;

	ret = smb1390_psy_get_int(chip->battery_psy,
				   POWER_SUPPLY_PROP_PRESENT, &present);
	if (ret || !present)
		return ret ? ret : -ENODATA;
	ret = smb1390_psy_get_int(chip->battery_psy,
				   POWER_SUPPLY_PROP_TEMP, &data->battery_temp);
	if (ret)
		return ret;
	if (data->battery_temp < SMB1390_POLICY_MIN_BATT_DECIC ||
	    data->battery_temp > SMB1390_POLICY_MAX_BATT_DECIC)
		return -ERANGE;
	ret = iio_read_channel_processed(chip->connector_temp,
					  &data->connector_temp_mc);
	if (ret || data->connector_temp_mc < -20000 ||
	    data->connector_temp_mc > SMB1390_POLICY_MAX_CONNECTOR_MC)
		return ret ? ret : -ERANGE;
	ret = iio_read_channel_processed(chip->skin_temp,
					  &data->skin_temp_mc);
	if (ret || data->skin_temp_mc < -20000 ||
	    data->skin_temp_mc > SMB1390_POLICY_MAX_SKIN_MC)
		return ret ? ret : -ERANGE;

	ret = smb1390_psy_get_int(chip->battery_psy,
				   POWER_SUPPLY_PROP_CAPACITY, &soc);
	if (ret)
		return ret;
	if (soc < 0 || soc >= SMB1390_POLICY_MAX_SOC)
		return -ERANGE;
	ret = smb1390_psy_get_int(chip->battery_psy,
				   POWER_SUPPLY_PROP_VOLTAGE_NOW, &data->vbat_uv);
	if (ret)
		return ret;
	if (data->vbat_uv < SMB1390_POLICY_MIN_VBAT_UV ||
	    data->vbat_uv > SMB1390_POLICY_MAX_VBAT_UV)
		return -ERANGE;

	ret = smb1390_psy_get_int(chip->main_psy,
				   POWER_SUPPLY_PROP_ONLINE, &online);
	if (ret || !online)
		return ret ? ret : -ENODEV;
	ret = smb1390_psy_get_int(chip->main_psy,
				   POWER_SUPPLY_PROP_HEALTH, &health);
	if (ret || health != POWER_SUPPLY_HEALTH_GOOD)
		return ret ? ret : -EIO;
	ret = smb1390_psy_get_int(chip->main_psy,
				   POWER_SUPPLY_PROP_CURRENT_MAX,
				   &data->main_icl_ua);
	if (ret)
		return ret;
	if (data->main_icl_ua < SMB1390_POLICY_PROVISIONAL_ICL_UA)
		return -EAGAIN;

	ret = smb1390_psy_get_int(chip->pd_psy,
				   POWER_SUPPLY_PROP_USB_TYPE, &type);
	if (ret)
		return ret;
	if (type != POWER_SUPPLY_USB_TYPE_PD_PPS &&
	    type != POWER_SUPPLY_USB_TYPE_PD_PPS_SPR_AVS)
		return -EOPNOTSUPP;
	ret = smb1390_psy_get_int(chip->psy, POWER_SUPPLY_PROP_HEALTH,
				   &health);
	if (ret || health != POWER_SUPPLY_HEALTH_GOOD)
		return ret ? ret : -EIO;

	return 0;
}

static int smb1390_policy_set_fcc(struct smb1390 *chip,
				   const struct smb1390_policy_data *data)
{
	int max_fcc, current_fcc, target_fcc, ret;

	ret = smb1390_psy_get_int(chip->main_psy,
				   POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				   &max_fcc);
	if (ret)
		return ret;
	if (max_fcc < SMB1390_POLICY_EDGE_FCC_UA)
		return -ERANGE;

	target_fcc = min(max_fcc, SMB1390_POLICY_MAX_FCC_UA);
	if (data->battery_temp < 200 || data->battery_temp > 350 ||
	    data->connector_temp_mc > 45000 ||
	    data->skin_temp_mc > 40000)
		target_fcc = min(target_fcc, SMB1390_POLICY_EDGE_FCC_UA);
	else if (data->battery_temp < 250 || data->battery_temp > 320 ||
		 data->connector_temp_mc > 40000 ||
		 data->skin_temp_mc > 35000)
		target_fcc = min(target_fcc, SMB1390_POLICY_MID_FCC_UA);
	target_fcc = rounddown(target_fcc, 50000);

	ret = smb1390_psy_get_int(chip->main_psy,
				   POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
				   &current_fcc);
	if (ret || current_fcc == target_fcc)
		return ret;
	return smb1390_psy_set_int(chip->main_psy,
				   POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
				   target_fcc);
}

static int smb1390_policy_stop(struct smb1390 *chip)
{
	int ret, first_error = 0, online, max_fcc;

	WRITE_ONCE(chip->policy_active, false);
	ret = smb1390_disable_switcher(chip);
	if (ret)
		first_error = ret;
	ret = qcom_smbx_set_charge_pump(chip->main_psy, false);
	if (ret && !first_error)
		first_error = ret;
	ret = smb1390_psy_set_int(chip->main_psy,
				  POWER_SUPPLY_PROP_CURRENT_MAX,
				  SMB1390_POLICY_PROVISIONAL_ICL_UA);
	if (ret && !first_error)
		first_error = ret;
	ret = smb1390_psy_get_int(chip->main_psy,
				   POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				   &max_fcc);
	if (!ret) {
		if (max_fcc <= 0)
			ret = -EINVAL;
		else
			ret = smb1390_psy_set_int(chip->main_psy,
					POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
					rounddown(min(max_fcc, 1500000),
						  50000));
	}
	if (ret && !first_error)
		first_error = ret;
	ret = smb1390_set_ilim(chip, SMB1390_ILIM_MIN_UA);
	if (ret && !first_error)
		first_error = ret;

	if (chip->pps_owned) {
		ret = smb1390_psy_set_int(chip->pd_psy,
					  POWER_SUPPLY_PROP_ONLINE,
					  SMB1390_FIXED_ONLINE);
		if (!ret) {
			chip->pps_owned = false;
		} else {
			int status = smb1390_psy_get_int(chip->pd_psy,
						POWER_SUPPLY_PROP_ONLINE,
						&online);

			if (!status && !online)
				chip->pps_owned = false;
			else if (!first_error)
				first_error = ret;
		}
	}

	chip->target_vbus_uv = 0;
	chip->target_input_ua = 0;
	if (first_error)
		chip->policy_faulted = true;
	power_supply_changed(chip->psy);
	return first_error;
}

static int smb1390_policy_start(struct smb1390 *chip,
				struct smb1390_policy_data *data)
{
	struct smb1390_policy_data fresh;
	int online, min_uv, max_uv, max_ua, main_uv, main_icl, die_temp, ret;
	u32 target_uv, target_ua, ilim_ua;
	bool switcher_on = false;
	int attempt;

	ret = smb1390_psy_get_int(chip->pd_psy, POWER_SUPPLY_PROP_ONLINE,
				   &online);
	if (ret || online != SMB1390_FIXED_ONLINE)
		return ret ? ret : -EBUSY;

	/* An attempted negotiation must be rolled back even after a timeout. */
	chip->pps_owned = true;
	ret = smb1390_psy_set_int(chip->pd_psy, POWER_SUPPLY_PROP_ONLINE,
				  SMB1390_PPS_ONLINE);
	if (ret)
		return ret;
	ret = smb1390_psy_get_int(chip->pd_psy, POWER_SUPPLY_PROP_ONLINE,
				   &online);
	if (ret || online != SMB1390_PPS_ONLINE)
		return ret ? ret : -EIO;
	if (READ_ONCE(chip->irq_faulted))
		return -EIO;

	ret = smb1390_psy_get_int(chip->pd_psy,
				   POWER_SUPPLY_PROP_VOLTAGE_MIN, &min_uv);
	if (ret)
		return ret;
	ret = smb1390_psy_get_int(chip->pd_psy,
				   POWER_SUPPLY_PROP_VOLTAGE_MAX, &max_uv);
	if (ret)
		return ret;
	ret = smb1390_psy_get_int(chip->pd_psy,
				   POWER_SUPPLY_PROP_CURRENT_MAX, &max_ua);
	if (ret)
		return ret;
	if (min_uv <= 0 || max_uv <= 0 ||
	    max_ua < SMB1390_POLICY_MIN_INPUT_UA)
		return -ERANGE;

	target_uv = data->vbat_uv * 2 + SMB1390_POLICY_HEADROOM_UV;
	if (target_uv < SMB1390_POLICY_MIN_VBUS_UV ||
	    target_uv > SMB1390_POLICY_MAX_VBUS_UV ||
	    target_uv < (u32)min_uv || target_uv > (u32)max_uv)
		return -ERANGE;
	target_ua = min_t(u32, max_ua, chip->max_input_ua);
	if (target_ua < SMB1390_POLICY_MIN_INPUT_UA ||
	    (u64)target_uv * target_ua < 10000000ULL * 1000000ULL)
		return -ERANGE;

	ret = smb1390_psy_set_int(chip->pd_psy,
				  POWER_SUPPLY_PROP_VOLTAGE_NOW, target_uv);
	if (ret || READ_ONCE(chip->irq_faulted))
		return ret ? ret : -EIO;
	ret = smb1390_psy_set_int(chip->pd_psy,
				  POWER_SUPPLY_PROP_CURRENT_NOW, target_ua);
	if (ret || READ_ONCE(chip->irq_faulted))
		return ret ? ret : -EIO;

	for (attempt = 0; attempt < 10; attempt++) {
		if (READ_ONCE(chip->irq_faulted))
			return -EIO;
		msleep(50);
		ret = smb1390_psy_get_int(chip->main_psy,
					   POWER_SUPPLY_PROP_VOLTAGE_NOW,
					   &main_uv);
		if (ret)
			return ret;
		if (abs(main_uv - (int)target_uv) <=
		    SMB1390_POLICY_VBUS_TOLERANCE_UV)
			break;
	}
	if (attempt == 10)
		return -ETIMEDOUT;
	ret = smb1390_psy_set_int(chip->main_psy,
				  POWER_SUPPLY_PROP_CURRENT_MAX, target_ua);
	if (ret)
		return ret;
	for (attempt = 0; attempt < 60; attempt++) {
		if (READ_ONCE(chip->irq_faulted))
			return -EIO;
		msleep(50);
		ret = smb1390_psy_get_int(chip->main_psy,
					   POWER_SUPPLY_PROP_CURRENT_MAX,
					   &main_icl);
		if (ret)
			return ret;
		if (main_icl >= (int)target_ua - 50000)
			break;
	}
	if (attempt == 60)
		return -ETIMEDOUT;
	ret = smb1390_psy_get_int(chip->pd_psy, POWER_SUPPLY_PROP_ONLINE,
				   &online);
	if (ret || online != SMB1390_PPS_ONLINE)
		return ret ? ret : -ENODEV;
	ret = smb1390_psy_get_int(chip->main_psy,
				   POWER_SUPPLY_PROP_VOLTAGE_NOW, &main_uv);
	if (ret || abs(main_uv - (int)target_uv) >
	    SMB1390_POLICY_VBUS_TOLERANCE_UV)
		return ret ? ret : -EIO;
	ret = smb1390_policy_precheck(chip, &fresh);
	if (ret)
		return ret;
	if (abs(fresh.vbat_uv - data->vbat_uv) > 150000)
		return -EAGAIN;

	ilim_ua = min(target_ua * 5 / 4, (u32)SMB1390_ILIM_MAX_UA);
	ilim_ua = rounddown(ilim_ua - SMB1390_ILIM_BASE_UA,
			     SMB1390_ILIM_STEP_UA) + SMB1390_ILIM_BASE_UA;
	ret = smb1390_set_ilim(chip, ilim_ua);
	if (ret)
		return ret;
	ret = qcom_smbx_set_charge_pump(chip->main_psy, true);
	if (ret || READ_ONCE(chip->irq_faulted))
		return ret ? ret : -EIO;
	ret = smb1390_write_verify(chip, SMB1390_CORE_CONTROL1_REG,
				   SMB1390_CMD_EN_SWITCHER_BIT,
				   SMB1390_CMD_EN_SWITCHER_BIT);
	if (ret)
		return ret;

	for (attempt = 0; attempt < 10; attempt++) {
		if (READ_ONCE(chip->irq_faulted))
			return -EIO;
		msleep(50);
		ret = smb1390_switcher_online(chip, &switcher_on);
		if (ret)
			return ret;
		if (switcher_on)
			break;
	}
	if (!switcher_on)
		return -ETIMEDOUT;

	ret = iio_read_channel_processed(chip->die_temp, &die_temp);
	if (ret || die_temp < -20000 ||
	    die_temp > SMB1390_POLICY_MAX_DIE_MC)
		return ret ? ret : -ERANGE;
	if (READ_ONCE(chip->irq_faulted))
		return -EIO;
	ret = smb1390_policy_set_fcc(chip, &fresh);
	if (ret || READ_ONCE(chip->irq_faulted))
		return ret ? ret : -EIO;

	chip->target_vbus_uv = target_uv;
	chip->target_input_ua = target_ua;
	WRITE_ONCE(chip->policy_active, true);
	power_supply_changed(chip->psy);
	return 0;
}

static int smb1390_policy_monitor(struct smb1390 *chip,
				  struct smb1390_policy_data *data)
{
	int online, pd_uv, pd_ua, main_uv, main_ua, die_temp, ret;
	unsigned int status1;
	bool switcher_on;
	int target_uv;

	ret = smb1390_psy_get_int(chip->pd_psy, POWER_SUPPLY_PROP_ONLINE,
				   &online);
	if (ret || online != SMB1390_PPS_ONLINE)
		return ret ? ret : -ENODEV;
	ret = smb1390_psy_get_int(chip->pd_psy,
				   POWER_SUPPLY_PROP_VOLTAGE_NOW, &pd_uv);
	if (ret)
		return ret;
	ret = smb1390_psy_get_int(chip->pd_psy,
				   POWER_SUPPLY_PROP_CURRENT_NOW, &pd_ua);
	if (ret)
		return ret;
	if (abs(pd_uv - (int)chip->target_vbus_uv) >
	    SMB1390_POLICY_VBUS_TOLERANCE_UV ||
	    pd_ua < (int)chip->target_input_ua ||
	    data->main_icl_ua < (int)chip->target_input_ua - 50000)
		return -EIO;

	ret = smb1390_psy_get_int(chip->main_psy,
				   POWER_SUPPLY_PROP_VOLTAGE_NOW, &main_uv);
	if (ret)
		return ret;
	ret = smb1390_psy_get_int(chip->main_psy,
				   POWER_SUPPLY_PROP_CURRENT_NOW, &main_ua);
	if (ret)
		return ret;
	if (abs(main_uv - (int)chip->target_vbus_uv) >
	    SMB1390_POLICY_VBUS_TOLERANCE_UV ||
	    main_ua < 0 || main_ua > (int)chip->target_input_ua + 500000)
		return -EIO;

	target_uv = data->vbat_uv * 2 + SMB1390_POLICY_HEADROOM_UV;
	if (abs(target_uv - (int)chip->target_vbus_uv) > 300000)
		return -EAGAIN;
	ret = smb1390_switcher_online(chip, &switcher_on);
	if (ret || !switcher_on)
		return ret ? ret : -EIO;
	ret = regmap_read(chip->regmap, SMB1390_CORE_STATUS1_REG, &status1);
	if (ret)
		return ret;
	if (status1 & SMB1390_ILIM_BIT) {
		chip->ilim_retries++;
		return chip->ilim_retries >= 3 ? -EIO : -EAGAIN;
	}
	ret = iio_read_channel_processed(chip->die_temp, &die_temp);
	if (ret || die_temp < -20000 ||
	    die_temp > SMB1390_POLICY_MAX_DIE_MC)
		return ret ? ret : -ERANGE;

	if (READ_ONCE(chip->irq_faulted))
		return -EIO;
	return smb1390_policy_set_fcc(chip, data);
}

static void smb1390_policy_work(struct work_struct *work)
{
	struct smb1390 *chip = container_of(to_delayed_work(work),
					     struct smb1390, policy_work);
	struct smb1390_policy_data data;
	unsigned int delay_ms = SMB1390_POLICY_IDLE_MS;
	unsigned int control;
	int ret, state;

	mutex_lock(&chip->policy_lock);
	if (!READ_ONCE(chip->policy_enabled))
		goto unlock;
	if (!smb1390_psy_get_int(chip->pd_psy, POWER_SUPPLY_PROP_ONLINE,
				 &state) && !state) {
		chip->ilim_retries = 0;
		if (!READ_ONCE(chip->irq_faulted))
			chip->policy_faulted = false;
	}

	ret = smb1390_policy_precheck(chip, &data);
	if (ret) {
		if (chip->policy_active || chip->pps_owned) {
			smb1390_policy_stop(chip);
			if (ret != -ENODEV && ret != -EOPNOTSUPP &&
			    ret != -EAGAIN)
				chip->policy_faulted = true;
		} else {
			int check = regmap_read(chip->regmap,
						SMB1390_CORE_CONTROL1_REG,
						&control);

			if (check || (control & SMB1390_CMD_EN_SWITCHER_BIT))
				smb1390_policy_stop(chip);
		}
		goto schedule;
	}

	if (chip->policy_active) {
		ret = smb1390_policy_monitor(chip, &data);
		if (ret) {
			dev_warn_ratelimited(chip->dev,
					     "PPS monitor stopped charge pump: %d\n",
					     ret);
			smb1390_policy_stop(chip);
			if (ret != -ENODEV && ret != -EOPNOTSUPP &&
			    ret != -EAGAIN)
				chip->policy_faulted = true;
		} else {
			delay_ms = SMB1390_POLICY_ACTIVE_MS;
		}
	} else if (!chip->policy_faulted) {
		ret = smb1390_policy_start(chip, &data);
		if (ret) {
			bool had_pps = chip->pps_owned;

			if (had_pps) {
				dev_warn_ratelimited(chip->dev,
					     "PPS start failed: %d\n", ret);
				smb1390_policy_stop(chip);
				chip->policy_faulted = true;
			}
		} else {
			delay_ms = SMB1390_POLICY_ACTIVE_MS;
		}
	}

schedule:
	if (READ_ONCE(chip->policy_enabled))
		mod_delayed_work(system_wq, &chip->policy_work,
				 msecs_to_jiffies(delay_ms));
unlock:
	mutex_unlock(&chip->policy_lock);
}

static int smb1390_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct power_supply *psy;
	struct smb1390 *chip;
	unsigned int status;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->dev = &client->dev;
	chip->policy_enabled = device_property_read_bool(chip->dev,
						"qcom,pps-charge-pump-policy");
	if (chip->policy_enabled) {
		ret = device_property_read_u32(chip->dev,
					       "qcom,max-input-current-microamp",
					       &chip->max_input_ua);
		if (ret)
			return dev_err_probe(chip->dev, ret,
					     "missing policy input current cap\n");
		if (chip->max_input_ua < SMB1390_POLICY_MIN_INPUT_UA ||
		    chip->max_input_ua > SMB1390_POLICY_MAX_INPUT_UA ||
		    chip->max_input_ua % 50000)
			return dev_err_probe(chip->dev, -EINVAL,
					     "invalid policy input current cap\n");
	}

	chip->regmap = devm_regmap_init_i2c(client, &smb1390_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(chip->regmap),
				     "failed to create register map\n");

	/* Always start with the switcher disabled before policy setup. */
	ret = smb1390_disable_switcher(chip);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to disable and verify charge pump switcher\n");
	ret = smb1390_configure_dual(chip);
	if (ret) {
		smb1390_disable_switcher(chip);
		return dev_err_probe(&client->dev, ret,
				     "failed to configure single auxiliary pump\n");
	}

	ret = regmap_read(chip->regmap, SMB1390_CORE_STATUS1_REG, &status);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read charge pump status\n");

	chip->die_temp = devm_iio_channel_get(&client->dev, "die-temp");
	if (IS_ERR(chip->die_temp))
		return dev_err_probe(&client->dev, PTR_ERR(chip->die_temp),
				     "failed to get die temperature channel\n");
	if (chip->policy_enabled) {
		chip->connector_temp = devm_iio_channel_get(chip->dev,
							 "connector-temp");
		if (IS_ERR(chip->connector_temp))
			return dev_err_probe(chip->dev,
					     PTR_ERR(chip->connector_temp),
					     "failed to get connector temperature\n");
		chip->skin_temp = devm_iio_channel_get(chip->dev, "skin-temp");
		if (IS_ERR(chip->skin_temp))
			return dev_err_probe(chip->dev,
					     PTR_ERR(chip->skin_temp),
					     "failed to get skin temperature\n");
	}

	ret = smb1390_get_policy_supplies(chip, &client->dev);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to resolve charging policy supplies\n");
	if (chip->policy_enabled &&
	    (!chip->pd_psy || !chip->main_psy || !chip->battery_psy))
		return dev_err_probe(chip->dev, -EINVAL,
				     "PPS policy needs all supply references\n");
	mutex_init(&chip->policy_lock);
	INIT_DELAYED_WORK(&chip->policy_work, smb1390_policy_work);
	if (chip->main_psy) {
		ret = qcom_smbx_set_charge_pump(chip->main_psy, false);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "failed to hold main SMB_EN low\n");
	}

	i2c_set_clientdata(client, chip);
	psy_cfg.drv_data = chip;
	psy_cfg.fwnode = dev_fwnode(&client->dev);

	psy = devm_power_supply_register(&client->dev, &smb1390_psy_desc,
					 &psy_cfg);
	if (IS_ERR(psy))
		return dev_err_probe(&client->dev, PTR_ERR(psy),
				     "failed to register charge pump power supply\n");
	chip->psy = psy;
	chip->irq = client->irq;
	if (chip->pd_psy && chip->irq < 0)
		return dev_err_probe(chip->dev, chip->irq,
				     "fault IRQ unavailable\n");
	if (chip->pd_psy && !chip->irq)
		return dev_err_probe(chip->dev, -EINVAL,
				     "policy supplies require a fault IRQ\n");

	if (chip->irq > 0) {
		ret = smb1390_enable_fault_irq(chip);
		if (ret) {
			regmap_write(chip->regmap,
				     SMB1390_CORE_INT_EN_CLR_REG, 0xff);
			return dev_err_probe(chip->dev, ret,
					     "failed to configure fault IRQ\n");
		}

		ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
					       smb1390_fault_irq,
					       IRQF_ONESHOT | IRQF_SHARED,
					       "smb1390-fault", chip);
		if (ret) {
			regmap_write(chip->regmap,
				     SMB1390_CORE_INT_EN_CLR_REG, 0xff);
			return dev_err_probe(chip->dev, ret,
					     "failed to request fault IRQ\n");
		}
	}
	if (chip->policy_enabled)
		mod_delayed_work(system_wq, &chip->policy_work, 0);

	return 0;
}

static void smb1390_shutdown(struct i2c_client *client)
{
	struct smb1390 *chip = i2c_get_clientdata(client);
	int ret;

	if (chip->policy_enabled) {
		WRITE_ONCE(chip->policy_enabled, false);
		mutex_lock(&chip->policy_lock);
		if (chip->policy_active || chip->pps_owned) {
			ret = smb1390_policy_stop(chip);
			if (ret)
				dev_err(chip->dev, "policy stop failed: %d\n", ret);
		}
		mutex_unlock(&chip->policy_lock);
		cancel_delayed_work_sync(&chip->policy_work);
	}

	if (chip->irq > 0) {
		regmap_write(chip->regmap, SMB1390_CORE_INT_EN_CLR_REG, 0xff);
		regmap_write(chip->regmap, SMB1390_CORE_INT_LATCHED_CLR_REG,
			     0xff);
	}

	ret = smb1390_disable_switcher(chip);
	if (ret)
		dev_err(&client->dev, "failed to disable and verify switcher: %d\n",
			ret);
	if (chip->main_psy) {
		ret = qcom_smbx_set_charge_pump(chip->main_psy, false);
		if (ret)
			dev_err(&client->dev, "failed to hold SMB_EN low: %d\n",
				ret);
	}
}

static void smb1390_remove(struct i2c_client *client)
{
	smb1390_shutdown(client);
}

static const struct of_device_id smb1390_of_match[] = {
	{ .compatible = "qcom,smb1390" },
	{ }
};
MODULE_DEVICE_TABLE(of, smb1390_of_match);

static const struct i2c_device_id smb1390_id[] = {
	{ "smb1390" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb1390_id);

static struct i2c_driver smb1390_driver = {
	.driver = {
		.name = "smb1390",
		.of_match_table = smb1390_of_match,
	},
	.probe = smb1390_probe,
	.remove = smb1390_remove,
	.shutdown = smb1390_shutdown,
	.id_table = smb1390_id,
};
module_i2c_driver(smb1390_driver);

MODULE_DESCRIPTION("Qualcomm SMB1390 charge pump diagnostics");
MODULE_LICENSE("GPL");
