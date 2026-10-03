// SPDX-License-Identifier: GPL-2.0-only
/*
 * Dongwoon DW9763 voice coil lens actuator
 *
 * Raphael's CamX module data supplies the power-on register sequence.
 * Position uses the same 10-bit DAC register pair as the DW9768 family.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/mutex.h>
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
	struct kref ref;
	atomic_t registered;
	struct mutex pm_lock;
	unsigned int open_users;
	bool removed;
	bool controls_initialized;
	bool entity_initialized;
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

	cci_write(vcm->regmap, DW9763_REG_CONTROL, 0x00, &ret);
	if (ret)
		goto disable_supplies;
	/* The module's delayUs belongs to the normal-mode write, not reset. */
	usleep_range(1000, 1200);

	cci_write(vcm->regmap, DW9763_REG_CONTROL, 0x02, &ret);
	cci_write(vcm->regmap, DW9763_REG_AAC_MODE, 0x60, &ret);
	cci_write(vcm->regmap, DW9763_REG_AAC_TIME, 0x02, &ret);
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

	if (READ_ONCE(vcm->removed))
		return -ENODEV;

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
	struct dw9763 *vcm = to_dw9763(sd);
	int ret;

	mutex_lock(&vcm->pm_lock);
	if (vcm->removed) {
		ret = -ENODEV;
		goto unlock;
	}
	ret = pm_runtime_resume_and_get(sd->dev);
	if (ret < 0)
		goto unlock;
	/*
	 * Restore only after PM marks the device active. A control cached
	 * during resume would otherwise miss the final position write.
	 */
	ret = v4l2_ctrl_handler_setup(&vcm->controls);
	if (ret)
		pm_runtime_put(sd->dev);
	else
		vcm->open_users++;
unlock:
	mutex_unlock(&vcm->pm_lock);
	return ret;
}

static int dw9763_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9763 *vcm = to_dw9763(sd);

	mutex_lock(&vcm->pm_lock);
	if (!vcm->removed) {
		vcm->open_users--;
		pm_runtime_put_autosuspend(sd->dev);
	}
	mutex_unlock(&vcm->pm_lock);
	return 0;
}

static void dw9763_free(struct kref *ref)
{
	struct dw9763 *vcm = container_of(ref, struct dw9763, ref);

	if (vcm->entity_initialized)
		media_entity_cleanup(&vcm->sd.entity);
	if (vcm->controls_initialized)
		v4l2_ctrl_handler_free(&vcm->controls);
	mutex_destroy(&vcm->pm_lock);
	kfree(vcm);
}

static int dw9763_registered(struct v4l2_subdev *sd)
{
	struct dw9763 *vcm = to_dw9763(sd);

	/* Do not overlap a new registration with an old open devnode. */
	if (atomic_cmpxchg(&vcm->registered, 0, 1))
		return -EBUSY;
	kref_get(&vcm->ref);
	return 0;
}

static void dw9763_release(struct v4l2_subdev *sd)
{
	struct dw9763 *vcm = to_dw9763(sd);

	/* Consume each registration reference only once. */
	if (atomic_xchg(&vcm->registered, 0))
		kref_put(&vcm->ref, dw9763_free);
}

static const struct v4l2_subdev_internal_ops dw9763_internal_ops = {
	.registered = dw9763_registered,
	.open = dw9763_open,
	.close = dw9763_close,
	.release = dw9763_release,
};

static const struct v4l2_subdev_ops dw9763_subdev_ops = {};

/* Retained file handles must stop using devm resources on probe failure too. */
static void dw9763_shutdown(struct dw9763 *vcm)
{
	mutex_lock(&vcm->pm_lock);
	v4l2_ctrl_lock(vcm->focus);
	WRITE_ONCE(vcm->removed, true);
	v4l2_ctrl_unlock(vcm->focus);
	mutex_unlock(&vcm->pm_lock);
	v4l2_async_unregister_subdev(&vcm->sd);
	pm_runtime_disable(vcm->dev);
	while (vcm->open_users) {
		pm_runtime_put_noidle(vcm->dev);
		vcm->open_users--;
	}
	if (!IS_ENABLED(CONFIG_PM) || !pm_runtime_status_suspended(vcm->dev))
		dw9763_runtime_suspend(vcm->dev);
	pm_runtime_dont_use_autosuspend(vcm->dev);
}

static int dw9763_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9763 *vcm;
	int ret;

	vcm = kzalloc_obj(*vcm);
	if (!vcm)
		return -ENOMEM;

	kref_init(&vcm->ref);
	mutex_init(&vcm->pm_lock);
	vcm->dev = dev;
	vcm->supplies[DW9763_VIN].supply = "vin";
	vcm->supplies[DW9763_VDD].supply = "vdd";

	ret = devm_regulator_bulk_get(dev, DW9763_NUM_SUPPLIES,
				      vcm->supplies);
	if (ret) {
		dev_err_probe(dev, ret, "failed to get supplies\n");
		goto put_vcm;
	}

	v4l2_i2c_subdev_init(&vcm->sd, client, &dw9763_subdev_ops);

	vcm->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(vcm->regmap)) {
		ret = dev_err_probe(dev, PTR_ERR(vcm->regmap),
				     "failed to initialize CCI regmap\n");
		goto put_vcm;
	}

	ret = v4l2_ctrl_handler_init(&vcm->controls, 1);
	if (ret)
		goto put_vcm;
	vcm->controls_initialized = true;
	vcm->focus = v4l2_ctrl_new_std(&vcm->controls, &dw9763_ctrl_ops,
				       V4L2_CID_FOCUS_ABSOLUTE, 0,
				       DW9763_MAX_FOCUS, 1, 0);
	if (vcm->controls.error) {
		ret = vcm->controls.error;
		goto put_vcm;
	}

	vcm->sd.ctrl_handler = &vcm->controls;
	vcm->sd.internal_ops = &dw9763_internal_ops;
	vcm->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	vcm->sd.entity.function = MEDIA_ENT_F_LENS;

	ret = media_entity_pads_init(&vcm->sd.entity, 0, NULL);
	if (ret)
		goto put_vcm;
	vcm->entity_initialized = true;

	pm_runtime_set_suspended(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	if (!IS_ENABLED(CONFIG_PM)) {
		ret = dw9763_runtime_resume(dev);
		if (ret)
			goto disable_pm;
		ret = v4l2_ctrl_handler_setup(&vcm->controls);
		if (ret)
			goto power_off;
	}

	ret = v4l2_async_register_subdev(&vcm->sd);
	if (ret) {
		dw9763_shutdown(vcm);
		goto put_vcm;
	}

	return 0;

power_off:
	if (!IS_ENABLED(CONFIG_PM))
		dw9763_runtime_suspend(dev);
disable_pm:
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);
put_vcm:
	kref_put(&vcm->ref, dw9763_free);
	return ret;
}

static void dw9763_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9763 *vcm = to_dw9763(sd);

	dw9763_shutdown(vcm);
	kref_put(&vcm->ref, dw9763_free);
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
