// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm SMB1390 auxiliary charge pump
 *
 * Register layout and fault bits are derived from the downstream
 * smb1390-charger driver. This driver deliberately leaves the switcher
 * disabled until the main charger and USB input policy can coordinate it.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define SMB1390_CORE_STATUS1_REG		0x1006
#define SMB1390_TEMP_ALARM_BIT		BIT(6)
#define SMB1390_VPH_OV_SOFT_BIT		BIT(7)

#define SMB1390_CORE_STATUS2_REG		0x1007
#define SMB1390_VPH_OV_HARD_BIT		BIT(1)
#define SMB1390_TSD_BIT			BIT(2)
#define SMB1390_IREV_BIT		BIT(3)
#define SMB1390_IOC_BIT			BIT(4)
#define SMB1390_VIN_OV_BIT		BIT(6)

#define SMB1390_CORE_CONTROL1_REG	0x1020
#define SMB1390_CMD_EN_SWITCHER_BIT	BIT(0)

struct smb1390 {
	struct regmap *regmap;
	struct iio_channel *die_temp;
};

static const struct regmap_config smb1390_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x10ff,
};

static const enum power_supply_property smb1390_properties[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int smb1390_get_property(struct power_supply *psy,
				 enum power_supply_property prop,
				 union power_supply_propval *val)
{
	struct smb1390 *chip = power_supply_get_drvdata(psy);
	unsigned int status1, status2;
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

		if ((status1 & SMB1390_TEMP_ALARM_BIT) ||
		    (status2 & SMB1390_TSD_BIT))
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if ((status1 & SMB1390_VPH_OV_SOFT_BIT) ||
			 (status2 & (SMB1390_VPH_OV_HARD_BIT |
				     SMB1390_VIN_OV_BIT)))
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (status2 & (SMB1390_IREV_BIT | SMB1390_IOC_BIT))
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;

		return 0;

	case POWER_SUPPLY_PROP_TEMP:
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

	chip->regmap = devm_regmap_init_i2c(client, &smb1390_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(chip->regmap),
				     "failed to create register map\n");

	/* No mainline policy can safely request this pump yet. */
	ret = regmap_update_bits(chip->regmap, SMB1390_CORE_CONTROL1_REG,
				 SMB1390_CMD_EN_SWITCHER_BIT, 0);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to disable charge pump switcher\n");

	ret = regmap_read(chip->regmap, SMB1390_CORE_STATUS1_REG, &status);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read charge pump status\n");

	chip->die_temp = devm_iio_channel_get(&client->dev, "die-temp");
	if (IS_ERR(chip->die_temp))
		return dev_err_probe(&client->dev, PTR_ERR(chip->die_temp),
				     "failed to get die temperature channel\n");

	i2c_set_clientdata(client, chip);
	psy_cfg.drv_data = chip;
	psy_cfg.fwnode = dev_fwnode(&client->dev);

	psy = devm_power_supply_register(&client->dev, &smb1390_psy_desc,
					 &psy_cfg);
	if (IS_ERR(psy))
		return dev_err_probe(&client->dev, PTR_ERR(psy),
				     "failed to register charge pump power supply\n");

	return 0;
}

static void smb1390_shutdown(struct i2c_client *client)
{
	struct smb1390 *chip = i2c_get_clientdata(client);
	int ret;

	ret = regmap_update_bits(chip->regmap, SMB1390_CORE_CONTROL1_REG,
				 SMB1390_CMD_EN_SWITCHER_BIT, 0);
	if (ret)
		dev_err(&client->dev, "failed to disable switcher: %d\n", ret);
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
