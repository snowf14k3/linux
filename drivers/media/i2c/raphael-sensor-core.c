// SPDX-License-Identifier: GPL-2.0-only
/*
 * V4L2 RAW10 image sensor support for Xiaomi Raphael camera modules.
 *
 * Register sequences and mode timing live in raphael-sensor-tables.c.
 * The camera elevator and Hall switch are independent devices; userspace
 * must extend the front camera before starting this sensor.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "raphael-sensor.h"

#define RAPHAEL_SENSOR_MCLK_RATE	19200000
#define RAPHAEL_SENSOR_LANES	4

struct raphael_sensor {
	struct device *dev;
	const struct raphael_sensor_variant *variant;
	struct regmap *regmap;
	struct clk *mclk;
	struct regulator *avdd;
	struct regulator *dvdd;
	struct regulator *dovdd;
	struct regulator *custom1;
	struct gpio_desc *reset_gpio;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct mutex mutex;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	const struct raphael_sensor_mode *mode;
	bool streaming;
};

static inline struct raphael_sensor *to_raphael_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct raphael_sensor, sd);
}

static int raphael_write_list(struct raphael_sensor *sensor,
			      const struct raphael_sensor_reg_list *list)
{
	unsigned int i;
	int ret;

	for (i = 0; i < list->num_regs; i++) {
		const struct raphael_sensor_reg *reg = &list->regs[i];

		ret = cci_write(sensor->regmap, reg->reg, reg->val, NULL);
		if (ret)
			return ret;
		if (reg->delay_us > 10)
			usleep_range(reg->delay_us, reg->delay_us + 100);
		else if (reg->delay_us)
			udelay(reg->delay_us);
	}

	return 0;
}

static int raphael_sensor_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct raphael_sensor *sensor = to_raphael_sensor(sd);
	bool custom1_on = false, dvdd_on = false, dovdd_on = false;
	bool mclk_on = false;
	int ret;

	ret = regulator_enable(sensor->avdd);
	if (ret)
		return ret;
	usleep_range(5000, 6000);

	if (sensor->custom1) {
		/* IMX586: VANA, CUSTOM1, VDIG, VIO, MCLK, RESET. */
		ret = regulator_enable(sensor->custom1);
		if (ret)
			goto rollback;
		custom1_on = true;
		ret = regulator_enable(sensor->dvdd);
		if (ret)
			goto rollback;
		dvdd_on = true;
		ret = regulator_enable(sensor->dovdd);
		if (ret)
			goto rollback;
		dovdd_on = true;
	} else {
		/* Other modules: VANA, VIO, VDIG, MCLK, RESET. */
		ret = regulator_enable(sensor->dovdd);
		if (ret)
			goto rollback;
		dovdd_on = true;
		ret = regulator_enable(sensor->dvdd);
		if (ret)
			goto rollback;
		dvdd_on = true;
	}

	ret = clk_prepare_enable(sensor->mclk);
	if (ret)
		goto rollback;
	mclk_on = true;

	/* Match the downstream physical GPIO high -> low reset sequence. */
	ret = gpiod_direction_output(sensor->reset_gpio, 1);
	if (ret)
		goto rollback;
	usleep_range(10000, 11000);
	ret = gpiod_direction_output(sensor->reset_gpio, 0);
	if (ret)
		goto rollback;
	usleep_range(2000, 3000);

	return 0;

rollback:
	if (mclk_on)
		clk_disable_unprepare(sensor->mclk);
	if (sensor->custom1) {
		if (dovdd_on)
			regulator_disable(sensor->dovdd);
		if (dvdd_on)
			regulator_disable(sensor->dvdd);
		if (custom1_on)
			regulator_disable(sensor->custom1);
	} else {
		if (dvdd_on)
			regulator_disable(sensor->dvdd);
		if (dovdd_on)
			regulator_disable(sensor->dovdd);
	}
	regulator_disable(sensor->avdd);
	return ret;
}

static int raphael_sensor_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct raphael_sensor *sensor = to_raphael_sensor(sd);
	int ret = 0, err;

	clk_disable_unprepare(sensor->mclk);
	if (sensor->custom1) {
		/* IMX586 CamX sequence: MCLK, VIO, VDIG, CUSTOM1, VANA. */
		usleep_range(2000, 3000);
		err = regulator_disable(sensor->dovdd);
		if (!ret)
			ret = err;
		err = regulator_disable(sensor->dvdd);
		if (!ret)
			ret = err;
		err = regulator_disable(sensor->custom1);
		if (!ret)
			ret = err;
	} else {
		/* Other modules: MCLK, VDIG, VIO, VANA. */
		usleep_range(1000, 2000);
		err = regulator_disable(sensor->dvdd);
		if (!ret)
			ret = err;
		usleep_range(1000, 2000);
		err = regulator_disable(sensor->dovdd);
		if (!ret)
			ret = err;
	}

	err = regulator_disable(sensor->avdd);
	if (!ret)
		ret = err;

	return ret;
}

static DEFINE_RUNTIME_DEV_PM_OPS(raphael_sensor_pm_ops,
				 raphael_sensor_power_off,
				 raphael_sensor_power_on, NULL);

static int raphael_sensor_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct raphael_sensor *sensor =
		container_of(ctrl->handler, struct raphael_sensor, ctrls);
	const struct raphael_sensor_variant *variant = sensor->variant;
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		u32 max = sensor->mode->height + ctrl->val -
			  variant->exposure_margin;

		__v4l2_ctrl_modify_range(sensor->exposure,
					 sensor->exposure->minimum, max, 1,
					 min_t(u32, sensor->exposure->default_value,
					       max));
	}

	ret = pm_runtime_get_if_in_use(sensor->dev);
	if (ret <= 0)
		return ret < 0 ? ret : 0;

	ret = raphael_write_list(sensor, &variant->group_hold_on);
	if (ret)
		goto put_pm;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = cci_write(sensor->regmap,
				CCI_REG16(variant->frame_length_register),
				sensor->mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		if (variant->exposure_data_bytes == 3)
			ret = cci_write(sensor->regmap,
					CCI_REG24(variant->exposure_register),
					ctrl->val << variant->exposure_shift, NULL);
		else
			ret = cci_write(sensor->regmap,
					CCI_REG16(variant->exposure_register),
					ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(sensor->regmap,
				CCI_REG16(variant->analogue_gain_register),
				ctrl->val, NULL);
		break;
	default:
		ret = 0;
		break;
	}

	{
		int release_ret = raphael_write_list(sensor,
						     &variant->group_hold_off);

		if (!ret)
			ret = release_ret;
	}

put_pm:
	pm_runtime_put(sensor->dev);
	return ret;
}

static const struct v4l2_ctrl_ops raphael_sensor_ctrl_ops = {
	.s_ctrl = raphael_sensor_set_ctrl,
};

static void raphael_sensor_update_controls(struct raphael_sensor *sensor)
{
	const struct raphael_sensor_mode *mode = sensor->mode;
	u32 hblank = mode->hts - mode->width;
	u32 vblank = mode->vts - mode->height;
	u32 exposure_max = mode->vts - sensor->variant->exposure_margin;

	__v4l2_ctrl_modify_range(sensor->hblank, hblank, hblank, 1, hblank);
	__v4l2_ctrl_modify_range(sensor->vblank, vblank,
				 sensor->variant->vts_max - mode->height,
				 1, vblank);
	__v4l2_ctrl_s_ctrl(sensor->vblank, vblank);
	__v4l2_ctrl_modify_range(sensor->exposure,
				 sensor->variant->exposure_min,
				 exposure_max, 1,
				 min_t(u32, exposure_max, mode->vts / 2));
	__v4l2_ctrl_s_ctrl(sensor->link_freq, mode->link_freq_index);
	__v4l2_ctrl_modify_range(sensor->pixel_rate, mode->pixel_rate,
				 mode->pixel_rate, 1, mode->pixel_rate);
}

static int raphael_sensor_init_controls(struct raphael_sensor *sensor)
{
	const struct raphael_sensor_variant *variant = sensor->variant;
	const struct raphael_sensor_mode *mode = sensor->mode;
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	u32 hblank = mode->hts - mode->width;
	u32 vblank = mode->vts - mode->height;
	u32 exposure_max = mode->vts - variant->exposure_margin;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 6);
	if (ret)
		return ret;
	hdl->lock = &sensor->mutex;

	sensor->link_freq = v4l2_ctrl_new_int_menu(
		hdl, NULL, V4L2_CID_LINK_FREQ,
		variant->num_link_frequencies - 1, 0,
		variant->link_frequencies);
	sensor->pixel_rate = v4l2_ctrl_new_std(hdl, NULL,
					       V4L2_CID_PIXEL_RATE,
					       mode->pixel_rate,
					       mode->pixel_rate, 1,
					       mode->pixel_rate);
	sensor->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK,
					  hblank, hblank, 1, hblank);
	sensor->vblank = v4l2_ctrl_new_std(hdl, &raphael_sensor_ctrl_ops,
					  V4L2_CID_VBLANK, vblank,
					  sensor->variant->vts_max -
					  mode->height, 1, vblank);
	sensor->exposure = v4l2_ctrl_new_std(hdl, &raphael_sensor_ctrl_ops,
					    V4L2_CID_EXPOSURE,
					    variant->exposure_min,
					    exposure_max, 1,
					    min_t(u32, exposure_max,
						  mode->vts / 2));
	v4l2_ctrl_new_std(hdl, &raphael_sensor_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN,
			  variant->gain_min, variant->gain_max,
			  1, variant->gain_default);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->sd.ctrl_handler = hdl;

	return 0;
}

static int raphael_sensor_enum_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct raphael_sensor *sensor = to_raphael_sensor(sd);

	if (code->index)
		return -EINVAL;
	code->code = sensor->variant->mbus_code;
	return 0;
}

static int raphael_sensor_enum_size(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	struct raphael_sensor *sensor = to_raphael_sensor(sd);
	unsigned int count = 0;
	unsigned int i, j;

	if (fse->code != sensor->variant->mbus_code)
		return -EINVAL;

	for (i = 0; i < sensor->variant->num_modes; i++) {
		const struct raphael_sensor_mode *mode =
			&sensor->variant->modes[i];

		for (j = 0; j < i; j++)
			if (sensor->variant->modes[j].width == mode->width &&
			    sensor->variant->modes[j].height == mode->height)
				break;
		if (j < i)
			continue;
		if (count++ != fse->index)
			continue;

		fse->min_width = fse->max_width = mode->width;
		fse->min_height = fse->max_height = mode->height;
		return 0;
	}

	return -EINVAL;
}

static int raphael_sensor_enum_interval(
	struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct raphael_sensor *sensor = to_raphael_sensor(sd);
	unsigned int count = 0;
	unsigned int i;

	if (fie->code != sensor->variant->mbus_code)
		return -EINVAL;

	for (i = 0; i < sensor->variant->num_modes; i++) {
		const struct raphael_sensor_mode *mode =
			&sensor->variant->modes[i];

		if (fie->width != mode->width || fie->height != mode->height)
			continue;
		if (count++ != fie->index)
			continue;

		fie->interval.numerator = mode->hts * mode->vts;
		fie->interval.denominator = mode->pixel_rate;
		return 0;
	}

	return -EINVAL;
}

static int raphael_sensor_set_fmt(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct raphael_sensor *sensor = to_raphael_sensor(sd);
	const struct raphael_sensor_mode *best = &sensor->variant->modes[0];
	const struct raphael_sensor_mode *mode;
	u64 best_distance = U64_MAX;
	unsigned int i;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && sensor->streaming)
		return -EBUSY;

	for (i = 0; i < sensor->variant->num_modes; i++) {
		u64 dx, dy, distance;

		mode = &sensor->variant->modes[i];
		dx = mode->width > fmt->format.width ?
			mode->width - fmt->format.width :
			fmt->format.width - mode->width;
		dy = mode->height > fmt->format.height ?
			mode->height - fmt->format.height :
			fmt->format.height - mode->height;
		distance = dx + dy;
		if (distance < best_distance ||
		    (distance == best_distance && mode == sensor->mode)) {
			best = mode;
			best_distance = distance;
		}
	}

	fmt->format.width = best->width;
	fmt->format.height = best->height;
	fmt->format.code = sensor->variant->mbus_code;
	fmt->format.field = V4L2_FIELD_NONE;
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sensor->mode = best;
		raphael_sensor_update_controls(sensor);
	}

	return 0;
}

static int raphael_sensor_get_interval(
	struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
	struct v4l2_subdev_frame_interval *fival)
{
	struct raphael_sensor *sensor = to_raphael_sensor(sd);

	if (fival->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;
	fival->interval.numerator = sensor->mode->hts *
		(sensor->mode->height + sensor->vblank->val);
	fival->interval.denominator = sensor->mode->pixel_rate;
	return 0;
}

static int raphael_sensor_set_interval(
	struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
	struct v4l2_subdev_frame_interval *fival)
{
	struct raphael_sensor *sensor = to_raphael_sensor(sd);
	const struct raphael_sensor_mode *best = NULL;
	u64 requested, difference = U64_MAX;
	unsigned int i;

	if (fival->which != V4L2_SUBDEV_FORMAT_ACTIVE ||
	    !fival->interval.numerator || !fival->interval.denominator ||
	    sensor->streaming)
		return -EINVAL;

	requested = div_u64((u64)fival->interval.denominator * 100,
			    fival->interval.numerator);
	for (i = 0; i < sensor->variant->num_modes; i++) {
		const struct raphael_sensor_mode *mode =
			&sensor->variant->modes[i];
		u64 delta;

		if (mode->width != sensor->mode->width ||
		    mode->height != sensor->mode->height)
			continue;
		delta = mode->fps_x100 > requested ?
			mode->fps_x100 - requested : requested - mode->fps_x100;
		if (delta < difference) {
			best = mode;
			difference = delta;
		}
	}

	if (!best)
		return -EINVAL;
	sensor->mode = best;
	raphael_sensor_update_controls(sensor);
	fival->interval.numerator = best->hts *
		(best->height + sensor->vblank->val);
	fival->interval.denominator = best->pixel_rate;
	return 0;
}

static int raphael_sensor_init_state(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state)
{
	struct raphael_sensor *sensor = to_raphael_sensor(sd);
	struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);

	fmt->width = sensor->mode->width;
	fmt->height = sensor->mode->height;
	fmt->code = sensor->variant->mbus_code;
	fmt->field = V4L2_FIELD_NONE;
	return 0;
}

static int raphael_sensor_start_streaming(struct raphael_sensor *sensor)
{
	const struct raphael_sensor_variant *variant = sensor->variant;
	int ret;

	ret = raphael_write_list(sensor, &variant->init);
	if (ret)
		return ret;
	ret = raphael_write_list(sensor, &sensor->mode->settings);
	if (ret)
		return ret;
	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret)
		return ret;
	return raphael_write_list(sensor, &variant->stream_on);
}

static int raphael_sensor_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct raphael_sensor *sensor = to_raphael_sensor(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	if (sensor->streaming == !!enable)
		goto unlock;

	if (enable) {
		ret = pm_runtime_resume_and_get(sensor->dev);
		if (ret < 0)
			goto unlock;
		ret = raphael_sensor_start_streaming(sensor);
		if (ret) {
			/* A multi-write start sequence may have enabled the sensor. */
			raphael_write_list(sensor, &sensor->variant->stream_off);
			pm_runtime_put(sensor->dev);
			goto unlock;
		}
		sensor->streaming = true;
	} else {
		ret = raphael_write_list(sensor, &sensor->variant->stream_off);
		sensor->streaming = false;
		pm_runtime_put(sensor->dev);
	}

unlock:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static const struct v4l2_subdev_core_ops raphael_sensor_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops raphael_sensor_video_ops = {
	.s_stream = raphael_sensor_set_stream,
};

static const struct v4l2_subdev_pad_ops raphael_sensor_pad_ops = {
	.enum_mbus_code = raphael_sensor_enum_code,
	.enum_frame_size = raphael_sensor_enum_size,
	.enum_frame_interval = raphael_sensor_enum_interval,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = raphael_sensor_set_fmt,
	.get_frame_interval = raphael_sensor_get_interval,
	.set_frame_interval = raphael_sensor_set_interval,
};

static const struct v4l2_subdev_ops raphael_sensor_subdev_ops = {
	.core = &raphael_sensor_core_ops,
	.video = &raphael_sensor_video_ops,
	.pad = &raphael_sensor_pad_ops,
};

static const struct media_entity_operations raphael_sensor_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops raphael_sensor_internal_ops = {
	.init_state = raphael_sensor_init_state,
};

static int raphael_sensor_check_endpoint(struct raphael_sensor *sensor)
{
	struct v4l2_fwnode_endpoint bus = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *endpoint;
	unsigned int i, j;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev),
						  NULL);
	if (!endpoint)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;

	if (bus.bus.mipi_csi2.num_data_lanes != RAPHAEL_SENSOR_LANES) {
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < sensor->variant->num_link_frequencies; i++) {
		for (j = 0; j < bus.nr_of_link_frequencies; j++)
			if (sensor->variant->link_frequencies[i] ==
			    bus.link_frequencies[j])
				break;
		if (j == bus.nr_of_link_frequencies) {
			ret = -EINVAL;
			goto out;
		}
	}

out:
	v4l2_fwnode_endpoint_free(&bus);
	return ret;
}

static int raphael_sensor_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct raphael_sensor *sensor;
	const struct raphael_sensor_variant *variant;
	u64 chip_id;
	int ret;

	variant = device_get_match_data(dev);
	if (!variant)
		return -ENODEV;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;
	sensor->variant = variant;
	sensor->mode = &variant->modes[0];
	mutex_init(&sensor->mutex);
	v4l2_i2c_subdev_init(&sensor->sd, client, &raphael_sensor_subdev_ops);
	sensor->sd.internal_ops = &raphael_sensor_internal_ops;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap)) {
		ret = PTR_ERR(sensor->regmap);
		goto destroy_mutex;
	}

	sensor->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sensor->mclk)) {
		ret = PTR_ERR(sensor->mclk);
		goto destroy_mutex;
	}
	if (clk_get_rate(sensor->mclk) != RAPHAEL_SENSOR_MCLK_RATE) {
		ret = -EINVAL;
		goto destroy_mutex;
	}

	sensor->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(sensor->avdd)) {
		ret = PTR_ERR(sensor->avdd);
		goto destroy_mutex;
	}
	sensor->dvdd = devm_regulator_get(dev, "dvdd");
	if (IS_ERR(sensor->dvdd)) {
		ret = PTR_ERR(sensor->dvdd);
		goto destroy_mutex;
	}
	sensor->dovdd = devm_regulator_get(dev, "dovdd");
	if (IS_ERR(sensor->dovdd)) {
		ret = PTR_ERR(sensor->dovdd);
		goto destroy_mutex;
	}
	if (variant->custom1_supply) {
		sensor->custom1 = devm_regulator_get(dev, "custom1");
		if (IS_ERR(sensor->custom1)) {
			ret = PTR_ERR(sensor->custom1);
			goto destroy_mutex;
		}
	}

	sensor->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sensor->reset_gpio)) {
		ret = PTR_ERR(sensor->reset_gpio);
		goto destroy_mutex;
	}

	ret = raphael_sensor_check_endpoint(sensor);
	if (ret)
		goto destroy_mutex;

	ret = raphael_sensor_power_on(dev);
	if (ret)
		goto destroy_mutex;

	ret = cci_read(sensor->regmap, CCI_REG16(variant->chip_id_register),
		       &chip_id, NULL);
	if (ret)
		goto power_off;
	if (chip_id != variant->chip_id) {
		dev_err(dev, "unexpected chip ID 0x%llx (expected 0x%x)\n",
			chip_id, variant->chip_id);
		ret = -ENODEV;
		goto power_off;
	}

	ret = raphael_sensor_init_controls(sensor);
	if (ret)
		goto power_off;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.entity.ops = &raphael_sensor_entity_ops;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto free_controls;

	sensor->sd.state_lock = &sensor->mutex;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto free_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto cleanup_subdev;
	pm_runtime_idle(dev);

	return 0;

cleanup_subdev:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&sensor->sd);
free_entity:
	media_entity_cleanup(&sensor->sd.entity);
free_controls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
power_off:
	raphael_sensor_power_off(dev);
destroy_mutex:
	mutex_destroy(&sensor->mutex);
	return ret;
}

static void raphael_sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct raphael_sensor *sensor = to_raphael_sensor(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev)) {
		raphael_sensor_power_off(sensor->dev);
		pm_runtime_set_suspended(sensor->dev);
	}
	mutex_destroy(&sensor->mutex);
}

static const struct of_device_id raphael_sensor_of_match[] = {
	{ .compatible = "sony,imx586", .data = &raphael_imx586 },
	{ .compatible = "xiaomi,raphael-ov8856", .data = &raphael_ov8856 },
	{ .compatible = "samsung,s5k3l6", .data = &raphael_s5k3l6 },
	{ .compatible = "samsung,s5k3t2", .data = &raphael_s5k3t2 },
	{ }
};
MODULE_DEVICE_TABLE(of, raphael_sensor_of_match);

static struct i2c_driver raphael_sensor_driver = {
	.driver = {
		.name = "raphael-sensors",
		.pm = &raphael_sensor_pm_ops,
		.of_match_table = raphael_sensor_of_match,
	},
	.probe = raphael_sensor_probe,
	.remove = raphael_sensor_remove,
};
module_i2c_driver(raphael_sensor_driver);

MODULE_DESCRIPTION("Xiaomi Raphael RAW camera sensors");
MODULE_LICENSE("GPL");
