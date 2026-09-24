// SPDX-License-Identifier: GPL-2.0-only
/*
 * Raw read-only EEPROM access for the four Xiaomi Raphael camera modules.
 * The CamX maps use a 16-bit offset and 8-bit data; the contents are opaque.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#define RAPHAEL_EEPROM_MCLK_HZ	19200000UL

struct raphael_camera_eeprom {
	struct i2c_client *client;
	struct regulator *vio;
	struct regulator *vaf;
	struct clk *mclk;
	struct mutex lock;
	u32 size;
};

static int raphael_camera_eeprom_read_byte(struct i2c_client *client,
					   unsigned int offset, u8 *value)
{
	u8 address[] = { offset >> 8, offset & 0xff };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.len = sizeof(address),
			.buf = address,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = value,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret == ARRAY_SIZE(msgs))
		return 0;

	return ret < 0 ? ret : -EIO;
}

static int raphael_camera_eeprom_read(void *context, unsigned int offset,
				      void *val, size_t bytes)
{
	struct raphael_camera_eeprom *eeprom = context;
	u8 *data = val;
	unsigned long rate;
	size_t i;
	int ret, off_ret;

	if (!bytes)
		return 0;
	if (offset >= eeprom->size || bytes > eeprom->size - offset)
		return -EINVAL;

	mutex_lock(&eeprom->lock);
	ret = regulator_enable(eeprom->vio);
	if (ret)
		goto out_unlock;

	if (eeprom->vaf) {
		ret = regulator_enable(eeprom->vaf);
		if (ret)
			goto disable_vio;
	}

	ret = clk_prepare_enable(eeprom->mclk);
	if (ret)
		goto disable_vaf;

	rate = clk_get_rate(eeprom->mclk);
	if (rate != RAPHAEL_EEPROM_MCLK_HZ) {
		dev_err(&eeprom->client->dev, "unexpected MCLK rate %lu Hz\n",
			rate);
		ret = -EINVAL;
		goto disable_mclk;
	}

	/* Do not assume a multi-byte sequential read protocol. */
	for (i = 0; i < bytes; i++) {
		ret = raphael_camera_eeprom_read_byte(eeprom->client,
						      offset + i, &data[i]);
		if (ret)
			break;
	}

disable_mclk:
	clk_disable_unprepare(eeprom->mclk);
disable_vaf:
	if (eeprom->vaf) {
		off_ret = regulator_disable(eeprom->vaf);
		if (!ret)
			ret = off_ret;
	}
disable_vio:
	off_ret = regulator_disable(eeprom->vio);
	if (!ret)
		ret = off_ret;
out_unlock:
	mutex_unlock(&eeprom->lock);

	return ret;
}

static int raphael_camera_eeprom_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct raphael_camera_eeprom *eeprom;
	struct nvmem_config config = { };
	struct nvmem_device *nvmem;
	u32 size;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter lacks I2C transfers\n");

	ret = device_property_read_u32(dev, "size", &size);
	if (ret)
		return dev_err_probe(dev, ret, "missing raw EEPROM size\n");
	if (size != 7158 && size != 8040 && size != 8192)
		return dev_err_probe(dev, -EINVAL, "invalid raw EEPROM size\n");

	eeprom = devm_kzalloc(dev, sizeof(*eeprom), GFP_KERNEL);
	if (!eeprom)
		return -ENOMEM;

	eeprom->client = client;
	eeprom->size = size;
	mutex_init(&eeprom->lock);

	eeprom->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(eeprom->vio))
		return dev_err_probe(dev, PTR_ERR(eeprom->vio),
				     "failed to get VIO supply\n");

	if (device_property_present(dev, "vaf-supply")) {
		eeprom->vaf = devm_regulator_get(dev, "vaf");
		if (IS_ERR(eeprom->vaf))
			return dev_err_probe(dev, PTR_ERR(eeprom->vaf),
					     "failed to get VAF supply\n");
	}

	eeprom->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(eeprom->mclk))
		return dev_err_probe(dev, PTR_ERR(eeprom->mclk),
				     "failed to get MCLK\n");

	config.dev = dev;
	config.name = dev_name(dev);
	config.id = NVMEM_DEVID_NONE;
	config.owner = THIS_MODULE;
	config.type = NVMEM_TYPE_EEPROM;
	config.read_only = true;
	config.root_only = true;
	config.reg_read = raphael_camera_eeprom_read;
	config.size = size;
	config.word_size = 1;
	config.stride = 1;
	config.priv = eeprom;

	nvmem = devm_nvmem_register(dev, &config);
	return PTR_ERR_OR_ZERO(nvmem);
}

static const struct of_device_id raphael_camera_eeprom_of_match[] = {
	{ .compatible = "xiaomi,raphael-camera-eeprom" },
	{ }
};
MODULE_DEVICE_TABLE(of, raphael_camera_eeprom_of_match);

static struct i2c_driver raphael_camera_eeprom_driver = {
	.probe = raphael_camera_eeprom_probe,
	.driver = {
		.name = "raphael-camera-eeprom",
		.of_match_table = raphael_camera_eeprom_of_match,
	},
};
module_i2c_driver(raphael_camera_eeprom_driver);

MODULE_DESCRIPTION("Raw read-only EEPROMs for Xiaomi Raphael camera modules");
MODULE_LICENSE("GPL");
