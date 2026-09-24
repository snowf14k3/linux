// SPDX-License-Identifier: GPL-2.0-only
/*
 * AK09970 magnetic sensor used to observe the Raphael popup camera.
 *
 * Preserve the downstream /dev/akm09970 ioctl payload while using the
 * current GPIO, regulator and I2C interfaces. Position thresholds are
 * mechanism-specific and are interpreted by userspace from raw samples.
 */

#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <uapi/linux/akm09970.h>

struct akm09970 {
	struct i2c_client *client;
	struct miscdevice miscdev;
	struct regulator *vdd;
	struct gpio_desc *reset;
	struct kref ref;
	struct mutex lock;
	wait_queue_head_t waitq;
	atomic_t data_ready;
	atomic_t sample_error;
	struct delayed_work initial_sample_work;
	u8 chip_info[2];
	u8 sample[AKM_SENSOR_DATA_SIZE];
	u64 sample_timestamp_ns;
	u32 sample_sequence;
	u32 measure_hz;
	bool sample_valid;
	bool active;
	bool resume_active;
	bool suspended;
	bool removing;
};

static void akm09970_release_ref(struct kref *ref)
{
	struct akm09970 *sensor = container_of(ref, struct akm09970, ref);

	kfree(sensor);
}

static u8 akm09970_mode(u32 hz)
{
	if (hz >= 100)
		return AK09970_MODE_CONTINUOUS_100HZ;
	if (hz >= 50)
		return AK09970_MODE_CONTINUOUS_50HZ;
	if (hz >= 20)
		return AK09970_MODE_CONTINUOUS_20HZ;

	return AK09970_MODE_CONTINUOUS_10HZ;
}

static void akm09970_invalidate_sample_locked(struct akm09970 *sensor,
					       bool error)
{
	sensor->sample_valid = false;
	sensor->sample_timestamp_ns = 0;
	atomic_set(&sensor->data_ready, 0);
	atomic_set(&sensor->sample_error, error);
	if (error)
		wake_up_interruptible_poll(&sensor->waitq, EPOLLERR);
}

static int akm09970_power_on(struct akm09970 *sensor)
{
	int ret;

	ret = regulator_enable(sensor->vdd);
	if (ret)
		return ret;

	/* reset-gpios is active low: logical 1 asserts reset. */
	ret = gpiod_set_value_cansleep(sensor->reset, 1);
	if (ret)
		goto disable_supply;
	udelay(50);
	ret = gpiod_set_value_cansleep(sensor->reset, 0);
	if (ret)
		goto disable_supply;
	udelay(100);

	return 0;

disable_supply:
	regulator_disable(sensor->vdd);
	return ret;
}

static void akm09970_power_off(struct akm09970 *sensor)
{
	gpiod_set_value_cansleep(sensor->reset, 1);
	regulator_disable(sensor->vdd);
}

static int akm09970_activate_locked(struct akm09970 *sensor)
{
	int ret;

	if (sensor->active)
		return 0;

	ret = akm09970_power_on(sensor);
	if (ret)
		return ret;

	ret = i2c_smbus_write_byte_data(sensor->client, AK09970_REG_CNTL2,
					akm09970_mode(sensor->measure_hz));
	if (ret < 0) {
		akm09970_power_off(sensor);
		return ret;
	}

	sensor->active = true;
	akm09970_invalidate_sample_locked(sensor, false);
	mod_delayed_work(system_wq, &sensor->initial_sample_work,
			 msecs_to_jiffies(20));
	return 0;
}

static void akm09970_deactivate_locked(struct akm09970 *sensor)
{
	if (!sensor->active)
		return;

	i2c_smbus_write_byte_data(sensor->client, AK09970_REG_CNTL2,
				  AK09970_MODE_POWERDOWN);
	sensor->active = false;
	cancel_delayed_work(&sensor->initial_sample_work);
	akm09970_invalidate_sample_locked(sensor, false);
	akm09970_power_off(sensor);
}

static void akm09970_read_sample_locked(struct akm09970 *sensor)
{
	u8 sample[AKM_SENSOR_DATA_SIZE];
	int ret;

	if (!sensor->active || sensor->removing)
		return;

	ret = i2c_smbus_read_i2c_block_data(sensor->client,
					    AK09970_REG_ST_XYZ,
					    sizeof(sample), sample);
	if (ret != AKM_SENSOR_DATA_SIZE)
		goto invalid;

	if (AKM_ERRADC_IS_HIGH(sample[0]) ||
	    AKM_ERRXY_IS_HIGH(sample[1]) ||
	    !AKM_DRDY_IS_HIGH(sample[1]))
		goto invalid;

	memcpy(sensor->sample, sample, sizeof(sample));
	sensor->sample_timestamp_ns = ktime_get_boottime_ns();
	sensor->sample_sequence++;
	sensor->sample_valid = true;
	atomic_set(&sensor->sample_error, 0);
	atomic_set(&sensor->data_ready, 1);
	wake_up_interruptible_poll(&sensor->waitq, EPOLLIN | EPOLLRDNORM);
	return;

invalid:
	akm09970_invalidate_sample_locked(sensor, true);
}

static void akm09970_initial_sample_work(struct work_struct *work)
{
	struct akm09970 *sensor =
		container_of(to_delayed_work(work), struct akm09970,
			     initial_sample_work);

	mutex_lock(&sensor->lock);
	akm09970_read_sample_locked(sensor);
	mutex_unlock(&sensor->lock);
}

static irqreturn_t akm09970_irq_thread(int irq, void *data)
{
	struct akm09970 *sensor = data;

	mutex_lock(&sensor->lock);
	akm09970_read_sample_locked(sensor);
	mutex_unlock(&sensor->lock);

	return IRQ_HANDLED;
}

static int akm09970_open(struct inode *inode, struct file *file)
{
	struct akm09970 *sensor =
		container_of(file->private_data, struct akm09970, miscdev);
	int ret;

	if (!kref_get_unless_zero(&sensor->ref))
		return -ENODEV;

	mutex_lock(&sensor->lock);
	ret = sensor->removing ? -ENODEV : 0;
	mutex_unlock(&sensor->lock);
	if (ret) {
		kref_put(&sensor->ref, akm09970_release_ref);
		return ret;
	}

	file->private_data = sensor;
	return nonseekable_open(inode, file);
}

static int akm09970_release(struct inode *inode, struct file *file)
{
	struct akm09970 *sensor = file->private_data;

	kref_put(&sensor->ref, akm09970_release_ref);
	return 0;
}

static __poll_t akm09970_poll(struct file *file, poll_table *wait)
{
	struct akm09970 *sensor = file->private_data;

	poll_wait(file, &sensor->waitq, wait);
	if (READ_ONCE(sensor->removing))
		return EPOLLHUP;
	if (atomic_read(&sensor->sample_error))
		return EPOLLERR;
	if (atomic_xchg(&sensor->data_ready, 0))
		return EPOLLIN | EPOLLRDNORM;

	return 0;
}

static long akm09970_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct akm09970 *sensor = file->private_data;
	struct akm09970_platform_data payload;
	struct akm09970_sample_snapshot snapshot = { 0 };
	int ret = 0;

	if (READ_ONCE(sensor->removing))
		return -ENODEV;

	if (cmd == AKM_IOC_SET_ACTIVE || cmd == AKM_IOC_SET_MODE) {
		if (copy_from_user(&payload, (void __user *)arg,
				   sizeof(payload)))
			return -EFAULT;
	} else {
		memset(&payload, 0, sizeof(payload));
	}

	mutex_lock(&sensor->lock);
	if (sensor->removing) {
		ret = -ENODEV;
		goto out_unlock;
	}

	switch (cmd) {
	case AKM_IOC_SET_ACTIVE:
		if (sensor->suspended) {
			ret = -EBUSY;
			break;
		}
		if (payload.sensor_state)
			ret = akm09970_activate_locked(sensor);
		else
			akm09970_deactivate_locked(sensor);
		break;
	case AKM_IOC_SET_MODE:
		if (sensor->suspended) {
			ret = -EBUSY;
			break;
		}
		sensor->measure_hz = payload.sensor_mode;
		akm09970_invalidate_sample_locked(sensor, false);
		if (sensor->active)
			ret = i2c_smbus_write_byte_data(sensor->client,
					AK09970_REG_CNTL2,
					akm09970_mode(sensor->measure_hz));
		break;
	case AKM_IOC_GET_SENSEDATA:
		payload.sensor_state = sensor->active;
		payload.sensor_mode = sensor->measure_hz;
		memcpy(payload.data, sensor->sample, sizeof(payload.data));
		break;
	case AKM_IOC_GET_SAMPLE_SNAPSHOT:
		if (!sensor->active || !sensor->sample_valid) {
			ret = atomic_read(&sensor->sample_error) ?
				-EIO : -ENODATA;
			break;
		}
		snapshot.timestamp_ns = sensor->sample_timestamp_ns;
		snapshot.sequence = sensor->sample_sequence;
		memcpy(snapshot.data, sensor->sample, sizeof(snapshot.data));
		break;
	case AKM_IOC_GET_SENSSMR:
		payload.sensor_smr = 0;
		break;
	default:
		ret = -ENOTTY;
		break;
	}

out_unlock:
	mutex_unlock(&sensor->lock);
	if (ret)
		return ret;

	if (cmd == AKM_IOC_GET_SAMPLE_SNAPSHOT &&
	    copy_to_user((void __user *)arg, &snapshot, sizeof(snapshot)))
		return -EFAULT;

	if (cmd == AKM_IOC_GET_SENSEDATA || cmd == AKM_IOC_GET_SENSSMR)
		if (copy_to_user((void __user *)arg, &payload,
				 sizeof(payload)))
			return -EFAULT;

	return 0;
}

#ifdef CONFIG_COMPAT
static long akm09970_compat_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	return akm09970_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations akm09970_fops = {
	.owner = THIS_MODULE,
	.open = akm09970_open,
	.release = akm09970_release,
	.unlocked_ioctl = akm09970_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = akm09970_compat_ioctl,
#endif
	.poll = akm09970_poll,
	.llseek = no_llseek,
};

static int akm09970_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct akm09970 *sensor;
	u32 measure_hz = 100;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK |
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter lacks AK09970 SMBus operations\n");
	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "missing data-ready IRQ\n");

	sensor = kzalloc_obj(*sensor, GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->client = client;
	kref_init(&sensor->ref);
	mutex_init(&sensor->lock);
	init_waitqueue_head(&sensor->waitq);
	atomic_set(&sensor->data_ready, 0);
	atomic_set(&sensor->sample_error, 0);
	INIT_DELAYED_WORK(&sensor->initial_sample_work,
			  akm09970_initial_sample_work);
	device_property_read_u32(dev, "asahi-kasei,measure-freq-hz", &measure_hz);
	if (measure_hz != 10 && measure_hz != 20 &&
	    measure_hz != 50 && measure_hz != 100) {
		ret = dev_err_probe(dev, -EINVAL, "invalid measurement rate\n");
		goto err_free;
	}
	sensor->measure_hz = measure_hz;

	sensor->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(sensor->vdd)) {
		ret = dev_err_probe(dev, PTR_ERR(sensor->vdd),
				    "failed to get vdd\n");
		goto err_free;
	}
	sensor->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset)) {
		ret = dev_err_probe(dev, PTR_ERR(sensor->reset),
				    "failed to get reset GPIO\n");
		goto err_free;
	}

	ret = akm09970_power_on(sensor);
	if (ret)
		goto err_free;

	ret = i2c_smbus_read_i2c_block_data(client, AK09970_REG_WIA,
					    sizeof(sensor->chip_info),
					    sensor->chip_info);
	akm09970_power_off(sensor);
	if (ret < 0)
		goto err_free;
	if (ret != AKM_SENSOR_INFO_SIZE ||
	    sensor->chip_info[0] != AK09970_WIA1_VALUE ||
	    sensor->chip_info[1] != AK09970_WIA2_VALUE) {
		ret = dev_err_probe(dev, -ENODEV,
				    "unexpected AK09970 chip ID\n");
		goto err_free;
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					akm09970_irq_thread, IRQF_ONESHOT,
					dev_name(dev), sensor);
	if (ret) {
		ret = dev_err_probe(dev, ret, "failed to request IRQ\n");
		goto err_free;
	}

	sensor->miscdev.minor = MISC_DYNAMIC_MINOR;
	sensor->miscdev.name = AKM09970_DRV_NAME;
	sensor->miscdev.fops = &akm09970_fops;
	sensor->miscdev.parent = dev;
	ret = misc_register(&sensor->miscdev);
	if (ret)
		goto err_irq;

	i2c_set_clientdata(client, sensor);
	return 0;

err_irq:
	devm_free_irq(dev, client->irq, sensor);
err_free:
	kfree(sensor);
	return ret;
}

static void akm09970_remove(struct i2c_client *client)
{
	struct akm09970 *sensor = i2c_get_clientdata(client);

	mutex_lock(&sensor->lock);
	sensor->removing = true;
	mutex_unlock(&sensor->lock);
	misc_deregister(&sensor->miscdev);
	devm_free_irq(&client->dev, client->irq, sensor);
	cancel_delayed_work_sync(&sensor->initial_sample_work);
	mutex_lock(&sensor->lock);
	akm09970_deactivate_locked(sensor);
	mutex_unlock(&sensor->lock);
	wake_up_interruptible_poll(&sensor->waitq, EPOLLHUP);
	kref_put(&sensor->ref, akm09970_release_ref);
}

static int akm09970_suspend(struct device *dev)
{
	struct akm09970 *sensor = dev_get_drvdata(dev);

	mutex_lock(&sensor->lock);
	sensor->resume_active = sensor->active;
	sensor->suspended = true;
	akm09970_deactivate_locked(sensor);
	mutex_unlock(&sensor->lock);

	return 0;
}

static int akm09970_resume(struct device *dev)
{
	struct akm09970 *sensor = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&sensor->lock);
	sensor->suspended = false;
	if (sensor->resume_active)
		ret = akm09970_activate_locked(sensor);
	sensor->resume_active = false;
	mutex_unlock(&sensor->lock);

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(akm09970_pm_ops,
				akm09970_suspend, akm09970_resume);

static const struct of_device_id akm09970_of_match[] = {
	{ .compatible = "asahi-kasei,ak09970" },
	{ .compatible = "akm,akm09970" },
	{ }
};
MODULE_DEVICE_TABLE(of, akm09970_of_match);

static const struct i2c_device_id akm09970_id[] = {
	{ AKM09970_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, akm09970_id);

static struct i2c_driver akm09970_driver = {
	.probe = akm09970_probe,
	.remove = akm09970_remove,
	.id_table = akm09970_id,
	.driver = {
		.name = AKM09970_DRV_NAME,
		.pm = pm_sleep_ptr(&akm09970_pm_ops),
		.of_match_table = akm09970_of_match,
	},
};
module_i2c_driver(akm09970_driver);

MODULE_DESCRIPTION("AK09970 Raphael popup camera Hall sensor");
MODULE_LICENSE("GPL");
