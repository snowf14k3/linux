// SPDX-License-Identifier: GPL-2.0-only
/*
 * Dongwoon DW9763 voice coil lens actuator
 *
 * Raphael's CamX module data supplies the power-on register sequence.
 * Position uses the same 10-bit DAC register pair as the DW9768 family.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define DW9763_REG_CONTROL	CCI_REG8(0x02)
#define DW9763_REG_POSITION	CCI_REG16(0x03)
#define DW9763_REG_AAC_MODE	CCI_REG8(0x06)
#define DW9763_REG_AAC_TIME	CCI_REG8(0x07)
#define DW9763_MAX_FOCUS	1023

enum dw9763_supply {
	DW9763_VIN,
	DW9763_VDD,
	DW9763_NUM_SUPPLIES,
};

struct dw9763 {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[DW9763_NUM_SUPPLIES];
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler controls;
	struct v4l2_ctrl *focus;
};

static struct dw9763 *to_dw9763(struct v4l2_subdev *sd)
{
	return container_of(sd, struct dw9763, sd);
}

static int dw9763_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct dw9763 *vcm = to_dw9763(sd);
	int ret;

	ret = regulator_bulk_enable(DW9763_NUM_SUPPLIES, vcm->supplies);
	if (ret)
		return ret;

	/* CamX waits 1 ms after enabling the actuator's 2.8 V rail. */
	usleep_range(1000, 1200);

	cci_write(vcm->regmap, DW9763_REG_CONTROL, 0x01, &ret);
	if (ret)
		goto disable_supplies;

	usleep_range(1000, 1200);

	cci_write(vcm->regmap, DW9763_REG_CONTROL, 0x00, &ret);
	cci_write(vcm->regmap, DW9763_REG_CONTROL, 0x02, &ret);
	cci_write(vcm->regmap, DW9763_REG_AAC_MODE, 0x60, &ret);
	cci_write(vcm->regmap, DW9763_REG_AAC_TIME, 0x02, &ret);
	cci_write(vcm->regmap, DW9763_REG_POSITION,
		  READ_ONCE(vcm->focus->val), &ret);
	if (!ret)
		return 0;

disable_supplies:
	regulator_bulk_disable(DW9763_NUM_SUPPLIES, vcm->supplies);
	return ret;
}

static int dw9763_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct dw9763 *vcm = to_dw9763(sd);
	int ret;

	/* CamX de-initialization writes zero to the control register. */
	ret = cci_write(vcm->regmap, DW9763_REG_CONTROL, 0x00, NULL);
	if (ret)
		dev_warn(dev, "failed to de-initialize actuator: %d\n", ret);

	return regulator_bulk_disable(DW9763_NUM_SUPPLIES, vcm->supplies);
}

static int dw9763_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct dw9763 *vcm = container_of(ctrl->handler, struct dw9763,
					 controls);
	int active;
	int ret;

	if (ctrl->id != V4L2_CID_FOCUS_ABSOLUTE)
		return -EINVAL;

	/* The control value is applied when runtime PM next powers the lens. */
	if (!IS_ENABLED(CONFIG_PM))
		return cci_write(vcm->regmap, DW9763_REG_POSITION,
				 ctrl->val, NULL);

	active = pm_runtime_get_if_active(vcm->dev);
	if (active < 0)
		return active;
	if (!active)
		return 0;

	ret = cci_write(vcm->regmap, DW9763_REG_POSITION, ctrl->val, NULL);
	pm_runtime_put(vcm->dev);
	return ret;
}

static const struct v4l2_ctrl_ops dw9763_ctrl_ops = {
	.s_ctrl = dw9763_set_ctrl,
};

static int dw9763_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int dw9763_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put_autosuspend(sd->dev);
	return 0;
}

static const struct v4l2_subdev_internal_ops dw9763_internal_ops = {
	.open = dw9763_open,
	.close = dw9763_close,
};

static const struct v4l2_subdev_ops dw9763_subdev_ops = {};

static int dw9763_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9763 *vcm;
	int ret;

	vcm = devm_kzalloc(dev, sizeof(*vcm), GFP_KERNEL);
	if (!vcm)
		return -ENOMEM;

	vcm->dev = dev;
	vcm->supplies[DW9763_VIN].supply = "vin";
	vcm->supplies[DW9763_VDD].supply = "vdd";

	ret = devm_regulator_bulk_get(dev, DW9763_NUM_SUPPLIES,
				      vcm->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	v4l2_i2c_subdev_init(&vcm->sd, client, &dw9763_subdev_ops);

	vcm->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(vcm->regmap))
		return dev_err_probe(dev, PTR_ERR(vcm->regmap),
				     "failed to initialize CCI regmap\n");

	v4l2_ctrl_handler_init(&vcm->controls, 1);
	vcm->focus = v4l2_ctrl_new_std(&vcm->controls, &dw9763_ctrl_ops,
				       V4L2_CID_FOCUS_ABSOLUTE, 0,
				       DW9763_MAX_FOCUS, 1, 0);
	if (vcm->controls.error) {
		ret = vcm->controls.error;
		goto free_controls;
	}

	vcm->sd.ctrl_handler = &vcm->controls;
	vcm->sd.internal_ops = &dw9763_internal_ops;
	vcm->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	vcm->sd.entity.function = MEDIA_ENT_F_LENS;

	ret = media_entity_pads_init(&vcm->sd.entity, 0, NULL);
	if (ret)
		goto free_controls;

	pm_runtime_set_suspended(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	if (!IS_ENABLED(CONFIG_PM)) {
		ret = dw9763_runtime_resume(dev);
		if (ret)
			goto disable_pm;
	}

	ret = v4l2_async_register_subdev(&vcm->sd);
	if (ret)
		goto power_off;

	return 0;

power_off:
	if (!IS_ENABLED(CONFIG_PM))
		dw9763_runtime_suspend(dev);
disable_pm:
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);
	media_entity_cleanup(&vcm->sd.entity);
free_controls:
	v4l2_ctrl_handler_free(&vcm->controls);
	return ret;
}

static void dw9763_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9763 *vcm = to_dw9763(sd);

	v4l2_async_unregister_subdev(sd);
	pm_runtime_disable(vcm->dev);
	if (!IS_ENABLED(CONFIG_PM) || !pm_runtime_status_suspended(vcm->dev))
		dw9763_runtime_suspend(vcm->dev);
	pm_runtime_dont_use_autosuspend(vcm->dev);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&vcm->controls);
}

static const struct of_device_id dw9763_of_match[] = {
	{ .compatible = "dongwoon,dw9763" },
	{}
};
MODULE_DEVICE_TABLE(of, dw9763_of_match);

static const struct dev_pm_ops dw9763_pm_ops = {
	SET_RUNTIME_PM_OPS(dw9763_runtime_suspend, dw9763_runtime_resume, NULL)
};

static struct i2c_driver dw9763_i2c_driver = {
	.driver = {
		.name = "dw9763",
		.pm = &dw9763_pm_ops,
		.of_match_table = dw9763_of_match,
	},
	.probe = dw9763_probe,
	.remove = dw9763_remove,
};
module_i2c_driver(dw9763_i2c_driver);

MODULE_DESCRIPTION("Dongwoon DW9763 camera lens actuator");
MODULE_LICENSE("GPL");
