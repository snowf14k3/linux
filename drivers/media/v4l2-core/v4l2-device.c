// SPDX-License-Identifier: GPL-2.0-or-later
/*
    V4L2 device support.

    Copyright (C) 2008  Hans Verkuil <hverkuil@kernel.org>

 */

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

int v4l2_device_register(struct device *dev, struct v4l2_device *v4l2_dev)
{
	if (v4l2_dev == NULL)
		return -EINVAL;

	INIT_LIST_HEAD(&v4l2_dev->subdevs);
	spin_lock_init(&v4l2_dev->lock);
	v4l2_prio_init(&v4l2_dev->prio);
	kref_init(&v4l2_dev->ref);
	get_device(dev);
	v4l2_dev->dev = dev;
	if (dev == NULL) {
		/* If dev == NULL, then name must be filled in by the caller */
		if (WARN_ON(!v4l2_dev->name[0]))
			return -EINVAL;
		return 0;
	}

	/* Set name to driver name + device name if it is empty. */
	if (!v4l2_dev->name[0])
		snprintf(v4l2_dev->name, sizeof(v4l2_dev->name), "%s %s",
			dev->driver->name, dev_name(dev));
	if (!dev_get_drvdata(dev))
		dev_set_drvdata(dev, v4l2_dev);
	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_device_register);

static void v4l2_device_release(struct kref *ref)
{
	struct v4l2_device *v4l2_dev =
		container_of(ref, struct v4l2_device, ref);

	if (v4l2_dev->release)
		v4l2_dev->release(v4l2_dev);
}

int v4l2_device_put(struct v4l2_device *v4l2_dev)
{
	return kref_put(&v4l2_dev->ref, v4l2_device_release);
}
EXPORT_SYMBOL_GPL(v4l2_device_put);

int v4l2_device_set_name(struct v4l2_device *v4l2_dev, const char *basename,
						atomic_t *instance)
{
	int num = atomic_inc_return(instance) - 1;
	int len = strlen(basename);

	if (basename[len - 1] >= '0' && basename[len - 1] <= '9')
		snprintf(v4l2_dev->name, sizeof(v4l2_dev->name),
				"%s-%d", basename, num);
	else
		snprintf(v4l2_dev->name, sizeof(v4l2_dev->name),
				"%s%d", basename, num);
	return num;
}
EXPORT_SYMBOL_GPL(v4l2_device_set_name);

void v4l2_device_disconnect(struct v4l2_device *v4l2_dev)
{
	if (v4l2_dev->dev == NULL)
		return;

	if (dev_get_drvdata(v4l2_dev->dev) == v4l2_dev)
		dev_set_drvdata(v4l2_dev->dev, NULL);
	put_device(v4l2_dev->dev);
	v4l2_dev->dev = NULL;
}
EXPORT_SYMBOL_GPL(v4l2_device_disconnect);

static void v4l2_subdev_release(struct v4l2_subdev *sd);

void v4l2_device_unregister(struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd;

	/* Just return if v4l2_dev is NULL or if it was already
	 * unregistered before. */
	if (v4l2_dev == NULL || !v4l2_dev->name[0])
		return;
	v4l2_device_disconnect(v4l2_dev);

	/*
	 * A callback can remove other subdevs, so do not cache the next one.
	 * Keep sd alive until the I2C/SPI teardown has finished using it.
	 */
	while (!list_empty(&v4l2_dev->subdevs)) {
		sd = list_first_entry(&v4l2_dev->subdevs, struct v4l2_subdev, list);
		kref_get(&sd->framework_ref);
		v4l2_device_unregister_subdev(sd);
		if (sd->flags & V4L2_SUBDEV_FL_IS_I2C)
			v4l2_i2c_subdev_unregister(sd);
		else if (sd->flags & V4L2_SUBDEV_FL_IS_SPI)
			v4l2_spi_subdev_unregister(sd);
		v4l2_subdev_release(sd);
	}
	/* Mark as unregistered, thus preventing duplicate unregistrations */
	v4l2_dev->name[0] = '\0';
}
EXPORT_SYMBOL_GPL(v4l2_device_unregister);

int __v4l2_device_register_subdev(struct v4l2_device *v4l2_dev,
				  struct v4l2_subdev *sd, struct module *module)
{
	int err;

	/* Check for valid input */
	if (!v4l2_dev || !sd || sd->v4l2_dev || !sd->name[0])
		return -EINVAL;
	/* Registration/list changes remain serialized by the bridge driver. */
	mutex_lock(&sd->devnode_lock);
	if (sd->devnode || sd->unregistering) {
		mutex_unlock(&sd->devnode_lock);
		return -EBUSY;
	}
	mutex_unlock(&sd->devnode_lock);
	kref_init(&sd->framework_ref);

	/*
	 * The reason to acquire the module here is to avoid unloading
	 * a module of sub-device which is registered to a media
	 * device. To make it possible to unload modules for media
	 * devices that also register sub-devices, do not
	 * try_module_get() such sub-device owners.
	 */
	sd->owner_v4l2_dev = v4l2_dev->dev && v4l2_dev->dev->driver &&
		module == v4l2_dev->dev->driver->owner;

	if (!sd->owner_v4l2_dev && !try_module_get(module))
		return -ENODEV;

	sd->v4l2_dev = v4l2_dev;
	/* This just returns 0 if either of the two args is NULL */
	err = v4l2_ctrl_add_handler(v4l2_dev->ctrl_handler, sd->ctrl_handler,
				    NULL, true);
	if (err)
		goto error_module;

#if defined(CONFIG_MEDIA_CONTROLLER)
	/* Register the entity. */
	if (v4l2_dev->mdev) {
		err = media_device_register_entity(v4l2_dev->mdev, &sd->entity);
		if (err < 0)
			goto error_module;
	}
#endif

	if (sd->internal_ops && sd->internal_ops->registered) {
		err = sd->internal_ops->registered(sd);
		if (err)
			goto error_unregister;
	}

	sd->owner = module;

	spin_lock(&v4l2_dev->lock);
	list_add_tail(&sd->list, &v4l2_dev->subdevs);
	spin_unlock(&v4l2_dev->lock);

	return 0;

error_unregister:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_device_unregister_entity(&sd->entity);
#endif
error_module:
	if (!sd->owner_v4l2_dev)
		module_put(module);
	sd->v4l2_dev = NULL;
	return err;
}
EXPORT_SYMBOL_GPL(__v4l2_device_register_subdev);

static void v4l2_subdev_release_final(struct kref *ref)
{
	struct v4l2_subdev *sd = container_of(ref, struct v4l2_subdev, framework_ref);
	struct module *owner = !sd->owner_v4l2_dev ? sd->owner : NULL;

	mutex_lock(&sd->devnode_lock);
	/* The release callback may free the memory containing sd. */
	sd->devnode = NULL;
	sd->unregistering = false;
	mutex_unlock(&sd->devnode_lock);
	if (sd->internal_ops && sd->internal_ops->release)
		sd->internal_ops->release(sd);
	module_put(owner);
}

static void v4l2_subdev_release(struct v4l2_subdev *sd)
{
	kref_put(&sd->framework_ref, v4l2_subdev_release_final);
}

static void v4l2_device_release_subdev_node(struct video_device *vdev)
{
	struct v4l2_subdev *sd = video_get_drvdata(vdev);

	/* A bridge teardown hold may outlive the video_device itself. */
	mutex_lock(&sd->devnode_lock);
	sd->devnode = NULL;
	mutex_unlock(&sd->devnode_lock);
	v4l2_subdev_release(sd);
	kfree(vdev);
}

int __v4l2_device_register_subdev_nodes(struct v4l2_device *v4l2_dev,
					bool read_only)
{
	struct video_device *vdev;
	struct v4l2_subdev *sd;
	int err;

	/* Register a device node for every subdev marked with the
	 * V4L2_SUBDEV_FL_HAS_DEVNODE flag.
	 */
	list_for_each_entry(sd, &v4l2_dev->subdevs, list) {
		if (!(sd->flags & V4L2_SUBDEV_FL_HAS_DEVNODE))
			continue;

		mutex_lock(&sd->devnode_lock);
		if (sd->unregistering || sd->v4l2_dev != v4l2_dev) {
			err = -ENODEV;
			goto unlock;
		}
		if (sd->devnode) {
			/* A failed batch must be unregistered before retrying. */
			if (!video_is_registered(sd->devnode)) {
				err = -EBUSY;
				goto unlock;
			}
			mutex_unlock(&sd->devnode_lock);
			continue;
		}

		vdev = kzalloc_obj(*vdev);
		if (!vdev) {
			err = -ENOMEM;
			goto unlock;
		}

		video_set_drvdata(vdev, sd);
		strscpy(vdev->name, sd->name, sizeof(vdev->name));
		vdev->dev_parent = sd->dev;
		vdev->v4l2_dev = v4l2_dev;
		vdev->fops = &v4l2_subdev_fops;
		vdev->release = v4l2_device_release_subdev_node;
		vdev->ctrl_handler = sd->ctrl_handler;
		if (read_only)
			set_bit(V4L2_FL_SUBDEV_RO_DEVNODE, &vdev->flags);
		sd->devnode = vdev;
		err = __video_register_device(vdev, VFL_TYPE_SUBDEV, -1, 1,
					      sd->owner);
		if (err < 0) {
			sd->devnode = NULL;
			kfree(vdev);
			goto unlock;
		}
		/*
		 * Keep the node alive until subdev unregistration. A partial
		 * batch failure may unregister the node first, but must not
		 * release the still-registered subdev or its module reference.
		 */
		get_device(&vdev->dev);
#if defined(CONFIG_MEDIA_CONTROLLER)
		sd->entity.info.dev.major = VIDEO_MAJOR;
		sd->entity.info.dev.minor = vdev->minor;

		/* Interface is created by __video_register_device() */
		if (vdev->v4l2_dev->mdev) {
			struct media_link *link;

			link = media_create_intf_link(&sd->entity,
						      &vdev->intf_devnode->intf,
						      MEDIA_LNK_FL_ENABLED |
						      MEDIA_LNK_FL_IMMUTABLE);
			if (!link) {
				err = -ENOMEM;
				goto unlock;
			}
		}
#endif
		mutex_unlock(&sd->devnode_lock);
	}
	return 0;

unlock:
	mutex_unlock(&sd->devnode_lock);
	list_for_each_entry(sd, &v4l2_dev->subdevs, list) {
		mutex_lock(&sd->devnode_lock);
		if (sd->devnode)
			video_unregister_device(sd->devnode);
		mutex_unlock(&sd->devnode_lock);
	}

	return err;
}
EXPORT_SYMBOL_GPL(__v4l2_device_register_subdev_nodes);

void v4l2_device_unregister_subdev(struct v4l2_subdev *sd)
{
	struct v4l2_device *v4l2_dev;
	struct video_device *vdev;

	if (!sd)
		return;

	mutex_lock(&sd->devnode_lock);
	v4l2_dev = sd->v4l2_dev;
	if (!v4l2_dev || sd->unregistering) {
		mutex_unlock(&sd->devnode_lock);
		return;
	}

	/*
	 * Quiesce open/ioctl before callbacks or media graph removal. The
	 * registration hold keeps vdev and sd alive across node withdrawal.
	 */
	sd->unregistering = true;
	vdev = sd->devnode;
	if (vdev)
		video_unregister_device(vdev);
	mutex_unlock(&sd->devnode_lock);

	spin_lock(&v4l2_dev->lock);
	list_del(&sd->list);
	spin_unlock(&v4l2_dev->lock);

	/* Callbacks may unregister other subdevs; never hold devnode_lock here. */
	if (sd->internal_ops && sd->internal_ops->unregistered)
		sd->internal_ops->unregistered(sd);

	mutex_lock(&sd->devnode_lock);
	sd->v4l2_dev = NULL;
	mutex_unlock(&sd->devnode_lock);

#if defined(CONFIG_MEDIA_CONTROLLER)
	if (v4l2_dev->mdev) {
		/*
		 * Both pads and links are removed in the right order by the
		 * media core. File operations can no longer enter this graph.
		 */
		media_device_unregister_entity(&sd->entity);
	}
#endif
	if (vdev)
		/* Final put may free sd and vdev. */
		put_device(&vdev->dev);
	else
		v4l2_subdev_release(sd);
}
EXPORT_SYMBOL_GPL(v4l2_device_unregister_subdev);
