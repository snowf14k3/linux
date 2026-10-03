// SPDX-License-Identifier: GPL-2.0-only
/*
 * Raphael popup camera stepper motor driven by a TI DRV8846.
 *
 * The GPIO and PWM wiring and three-stage timing come from the Xiaomi
 * downstream kernel. A hardware fault or watchdog expiry cuts the bridge
 * power in IRQ context; PWM shutdown is then completed in process context.
 */

#include <linux/akm09970.h>
#include <linux/capability.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/drv8846.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/limits.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/pwm.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <uapi/misc/drv8846.h>

#define DRV8846_MIN_PERIOD_NS	50000U
#define DRV8846_MAX_PERIOD_NS	1000000U
#define DRV8846_MAX_MOVE_MS	1500U
#define DRV8846_DOWN_EXTRA_MS	5U
#define DRV8846_WATCHDOG_GRACE_JIFFIES	2U
/* Three periods at the slowest supported Hall acquisition rate. */
#define DRV8846_HALL_READY_MS	300U
#define DRV8846_HALL_POLL_MS	10U

struct drv8846 {
	struct device *dev;
	struct akm09970 *hall;
	struct drv8846_calibration calibration;
	bool calibration_valid;
	bool camera_owned;
	bool camera_cancelled;
	struct miscdevice miscdev;
	struct kref ref;
	struct mutex lock;
	struct mutex command_lock;
	spinlock_t safety_lock;
	struct gpio_desc *mode0;
	struct gpio_desc *mode1;
	struct gpio_desc *dir;
	struct gpio_desc *sleep;
	struct gpio_desc *enable;
	struct gpio_desc *fault;
	struct pwm_device *pwm;
	struct delayed_work phase_work;
	struct work_struct stop_work;
	struct hrtimer watchdog;
	wait_queue_head_t waitq;
	struct fasync_struct *async;
	atomic_t move_done;
	enum running_state state;
	bool armed;
	u32 last_stop_reason;
	u32 watchdog_grace_ms;
	bool fault_latched;
	bool timeout_latched;
	bool cutoff_failed;
	bool suspended;
	bool removing;
	unsigned int pending_stops;
	u32 rampup_period_ns;
	u32 high_period_ns;
	u32 rampdown_period_ns;
	u32 rampup_duration_ms;
	u32 high_duration_ms;
	u32 rampdown_duration_ms;
	u32 step_mode;
	u32 direction;
	int fault_irq;
};

static struct platform_driver drv8846_driver;

static void drv8846_release_ref(struct kref *ref)
{
	struct drv8846 *motor = container_of(ref, struct drv8846, ref);

	kfree(motor);
}

/* sleep and enable are required to be non-sleeping TLMM GPIOs. */
static void drv8846_cut_power_locked(struct drv8846 *motor, u32 reason)
{
	int sleep_ret, enable_ret;

	if (motor->armed)
		motor->last_stop_reason = reason;
	motor->armed = false;
	sleep_ret = gpiod_set_value(motor->sleep, 0);
	enable_ret = gpiod_set_value(motor->enable, 0);
	if (sleep_ret || enable_ret) {
		motor->cutoff_failed = true;
		motor->last_stop_reason = DRV8846_STOP_ERROR;
	}
}

static void drv8846_cut_power(struct drv8846 *motor, u32 reason)
{
	unsigned long flags;

	spin_lock_irqsave(&motor->safety_lock, flags);
	if (reason == DRV8846_STOP_FAULT)
		motor->fault_latched = true;
	if (reason == DRV8846_STOP_TIMEOUT || reason == DRV8846_STOP_NO_POSITION)
		motor->timeout_latched = true;
	drv8846_cut_power_locked(motor, reason);
	spin_unlock_irqrestore(&motor->safety_lock, flags);
}

/*
 * Called before waiting for command_lock: a racing STOP, suspend or remove
 * must drop bridge power even when a PWM transaction is blocked.
 */
static bool drv8846_request_stop(struct drv8846 *motor, bool suspend,
				 bool remove)
{
	unsigned long flags;

	spin_lock_irqsave(&motor->safety_lock, flags);
	if (motor->removing) {
		spin_unlock_irqrestore(&motor->safety_lock, flags);
		return false;
	}
	if (suspend)
		motor->suspended = true;
	if (remove)
		motor->removing = true;
	if (!suspend && !remove)
		motor->pending_stops++;
	drv8846_cut_power_locked(motor, suspend || remove ?
				 DRV8846_STOP_ABORTED : DRV8846_STOP_REQUESTED);
	spin_unlock_irqrestore(&motor->safety_lock, flags);

	return true;
}

static void drv8846_finish_stop_request(struct drv8846 *motor)
{
	unsigned long flags;

	spin_lock_irqsave(&motor->safety_lock, flags);
	if (WARN_ON_ONCE(!motor->pending_stops)) {
		spin_unlock_irqrestore(&motor->safety_lock, flags);
		return;
	}
	motor->pending_stops--;
	spin_unlock_irqrestore(&motor->safety_lock, flags);
}

static bool drv8846_is_armed(struct drv8846 *motor)
{
	unsigned long flags;
	bool armed;

	spin_lock_irqsave(&motor->safety_lock, flags);
	armed = motor->armed;
	spin_unlock_irqrestore(&motor->safety_lock, flags);

	return armed;
}

static int drv8846_pwm_apply(struct drv8846 *motor, u32 period_ns, bool enable)
{
	struct pwm_state state;

	pwm_get_state(motor->pwm, &state);
	state.period = period_ns;
	state.duty_cycle = enable ? period_ns / 2 : 0;
	state.enabled = enable;

	return pwm_apply_might_sleep(motor->pwm, &state);
}

static void drv8846_stop_locked(struct drv8846 *motor, u32 reason)
{
	enum running_state old_state = motor->state;
	unsigned long flags;
	int ret;

	drv8846_cut_power(motor, reason);
	hrtimer_cancel(&motor->watchdog);
	if (READ_ONCE(motor->cutoff_failed))
		dev_err_ratelimited(motor->dev,
				    "motor power cutoff GPIO write failed\n");
	ret = drv8846_pwm_apply(motor, motor->rampup_period_ns, false);
	if (ret) {
		dev_err(motor->dev, "failed to disable motor PWM: %d\n", ret);
		if (old_state != STILL) {
			spin_lock_irqsave(&motor->safety_lock, flags);
			if (motor->last_stop_reason == reason)
				motor->last_stop_reason = DRV8846_STOP_ERROR;
			spin_unlock_irqrestore(&motor->safety_lock, flags);
		}
	}

	motor->state = STILL;
	if (old_state != STILL) {
		atomic_set(&motor->move_done, 1);
		wake_up_interruptible_poll(&motor->waitq, EPOLLIN | EPOLLRDNORM);
		kill_fasync(&motor->async, SIGIO, POLL_IN);
	}
}

static void drv8846_stop_work(struct work_struct *work)
{
	struct drv8846 *motor = container_of(work, struct drv8846, stop_work);

	mutex_lock(&motor->lock);
	drv8846_stop_locked(motor, DRV8846_STOP_ERROR);
	mutex_unlock(&motor->lock);
}

static enum hrtimer_restart drv8846_watchdog(struct hrtimer *timer)
{
	struct drv8846 *motor = container_of(timer, struct drv8846, watchdog);

	drv8846_cut_power(motor, DRV8846_STOP_TIMEOUT);
	schedule_work(&motor->stop_work);

	return HRTIMER_NORESTART;
}

static irqreturn_t drv8846_fault_irq(int irq, void *data)
{
	struct drv8846 *motor = data;

	drv8846_cut_power(motor, DRV8846_STOP_FAULT);
	schedule_work(&motor->stop_work);

	return IRQ_HANDLED;
}

static void drv8846_phase_work(struct work_struct *work)
{
	struct drv8846 *motor =
		container_of(to_delayed_work(work), struct drv8846, phase_work);
	u32 next_ms = 0;
	u32 period_ns = 0;
	int ret;

	mutex_lock(&motor->lock);
	if (!drv8846_is_armed(motor))
		goto out;

	switch (motor->state) {
	case SPEEDUP:
		motor->state = FULLSTEAM;
		period_ns = motor->high_period_ns;
		next_ms = motor->high_duration_ms;
		if (motor->direction == DOWN)
			next_ms += DRV8846_DOWN_EXTRA_MS;
		break;
	case FULLSTEAM:
		motor->state = SLOWDOWN;
		period_ns = motor->rampdown_period_ns;
		next_ms = motor->rampdown_duration_ms;
		break;
	case SLOWDOWN:
	case UNIFORMSPEED:
		drv8846_stop_locked(motor, DRV8846_STOP_TIMED);
		goto out;
	default:
		goto out;
	}

	ret = drv8846_pwm_apply(motor, period_ns, true);
	if (ret) {
		dev_err(motor->dev, "failed to change motor PWM: %d\n", ret);
		drv8846_stop_locked(motor, DRV8846_STOP_ERROR);
		goto out;
	}

	/* An IRQ can cut the bridge while the PWM provider is sleeping. */
	if (drv8846_is_armed(motor))
		mod_delayed_work(system_highpri_wq, &motor->phase_work,
				 msecs_to_jiffies(next_ms));
out:
	mutex_unlock(&motor->lock);
}

static int drv8846_start_locked(struct drv8846 *motor, u32 direction,
			 u32 period_ns, u32 duration_ms, bool automatic)
{
	unsigned long flags;
	u32 total_ms;
	u32 watchdog_ms;
	int ret;

	if (direction != UP && direction != DOWN)
		return -EINVAL;
	if (period_ns < DRV8846_MIN_PERIOD_NS ||
	    period_ns > DRV8846_MAX_PERIOD_NS)
		return -EINVAL;
	if (!duration_ms || duration_ms > DRV8846_MAX_MOVE_MS)
		return -EINVAL;

	if (READ_ONCE(motor->removing) || READ_ONCE(motor->suspended)) {
		ret = -ENODEV;
		goto out_command;
	}
	if (READ_ONCE(motor->pending_stops)) {
		ret = -ECANCELED;
		goto out_command;
	}
	if (READ_ONCE(motor->cutoff_failed) || READ_ONCE(motor->fault_latched)) {
		ret = -EIO;
		goto out_command;
	}
	if (READ_ONCE(motor->timeout_latched)) {
		ret = -ETIMEDOUT;
		goto out_command;
	}

	drv8846_cut_power(motor, DRV8846_STOP_ABORTED);
	hrtimer_cancel(&motor->watchdog);
	cancel_delayed_work_sync(&motor->phase_work);
	cancel_work_sync(&motor->stop_work);

	mutex_lock(&motor->lock);
	if (READ_ONCE(motor->removing) || READ_ONCE(motor->suspended)) {
		ret = -ENODEV;
		goto out_unlock;
	}
	if (READ_ONCE(motor->pending_stops)) {
		ret = -ECANCELED;
		goto out_unlock;
	}
	if (READ_ONCE(motor->cutoff_failed)) {
		ret = -EIO;
		goto out_unlock;
	}

	drv8846_stop_locked(motor, DRV8846_STOP_ABORTED);
	atomic_set(&motor->move_done, 0);

	ret = gpiod_get_value_cansleep(motor->fault);
	if (ret < 0)
		goto out_unlock;
	if (ret) {
		ret = -EIO;
		goto out_unlock;
	}

	ret = gpiod_set_value_cansleep(motor->mode0,
					!!(motor->step_mode & 1));
	if (ret)
		goto out_unlock;
	ret = gpiod_set_value_cansleep(motor->mode1,
					!!(motor->step_mode & 2));
	if (ret)
		goto out_unlock;
	/* Downstream wiring drives DIR low for UP and high for DOWN. */
	ret = gpiod_set_value_cansleep(motor->dir, direction == DOWN);
	if (ret)
		goto out_unlock;
	motor->direction = direction;
	if (automatic) {
		total_ms = motor->rampup_duration_ms +
			   motor->high_duration_ms +
			   motor->rampdown_duration_ms;
		if (direction == DOWN)
			total_ms += DRV8846_DOWN_EXTRA_MS;
	} else {
		total_ms = duration_ms;
	}

	ret = drv8846_pwm_apply(motor, period_ns, true);
	if (ret)
		goto out_stop;

	/* Allow phase work two ticks, without exceeding the hard move limit. */
	watchdog_ms = min_t(u32, total_ms +
		jiffies_to_msecs(DRV8846_WATCHDOG_GRACE_JIFFIES),
		DRV8846_MAX_MOVE_MS);
	motor->watchdog_grace_ms = watchdog_ms - total_ms;
	hrtimer_start(&motor->watchdog, ms_to_ktime(watchdog_ms),
		      HRTIMER_MODE_REL);
	spin_lock_irqsave(&motor->safety_lock, flags);
	if (motor->cutoff_failed) {
		ret = -EIO;
	} else if (motor->fault_latched) {
		ret = -EIO;
	} else if (motor->timeout_latched) {
		ret = -ETIMEDOUT;
	} else if (motor->removing || motor->suspended ||
		   motor->pending_stops) {
		ret = -ECANCELED;
	} else {
		ret = gpiod_set_value(motor->enable, 1);
		if (!ret)
			ret = gpiod_set_value(motor->sleep, 1);
		if (!ret)
			motor->armed = true;
	}
	spin_unlock_irqrestore(&motor->safety_lock, flags);
	if (ret)
		goto out_stop;

	motor->state = automatic ? SPEEDUP : UNIFORMSPEED;
	mod_delayed_work(system_highpri_wq, &motor->phase_work,
			 msecs_to_jiffies(automatic ?
					motor->rampup_duration_ms : duration_ms));
	mutex_unlock(&motor->lock);

	return 0;

out_stop:
	drv8846_cut_power(motor, DRV8846_STOP_ERROR);
	hrtimer_cancel(&motor->watchdog);
	drv8846_stop_locked(motor, DRV8846_STOP_ERROR);
out_unlock:
	mutex_unlock(&motor->lock);
out_command:
	return ret;
}

/* command_lock excludes configuration changes and competing movements. */
static void drv8846_quiesce(struct drv8846 *motor, u32 reason)
{
	drv8846_cut_power(motor, reason);
	hrtimer_cancel(&motor->watchdog);
	cancel_delayed_work_sync(&motor->phase_work);
	cancel_work_sync(&motor->stop_work);
	mutex_lock(&motor->lock);
	drv8846_stop_locked(motor, reason);
	mutex_unlock(&motor->lock);
}

static int drv8846_check_available(struct drv8846 *motor)
{
	if (READ_ONCE(motor->removing))
		return -ENODEV;
	if (READ_ONCE(motor->suspended))
		return -EBUSY;
	if (READ_ONCE(motor->pending_stops))
		return -ECANCELED;
	if (READ_ONCE(motor->cutoff_failed))
		return -EIO;
	return 0;
}

static int drv8846_check_safe(struct drv8846 *motor)
{
	int ret = drv8846_check_available(motor);

	if (ret)
		return ret;
	if (READ_ONCE(motor->fault_latched))
		return -EIO;
	if (READ_ONCE(motor->timeout_latched))
		return -ETIMEDOUT;
	ret = gpiod_get_value_cansleep(motor->fault);
	return ret < 0 ? ret : ret ? -EIO : 0;
}

static u32 drv8846_classify(struct drv8846 *motor,
			    const struct akm09970_sample_snapshot *sample)
{
	const struct drv8846_calibration *cal = &motor->calibration;
	bool match;
	int endpoint, axis, value;

	for (endpoint = 0; endpoint < 2; endpoint++) {
		match = true;
		for (axis = 0; axis < 3; axis++) {
			/* Downstream MAKE_S16 uses MSB then LSB. */
			value = (s16)get_unaligned_be16(&sample->data[2 + 2 * axis]);
			if (value < cal->minimum[endpoint][axis] ||
			    value > cal->maximum[endpoint][axis])
				match = false;
		}
		if (match)
			return endpoint == DOWN ? DRV8846_POSITION_DOWN :
						  DRV8846_POSITION_UP;
	}
	return DRV8846_POSITION_UNKNOWN;
}

/* Two distinct samples acquired after entry; cached samples never suffice. */
static int drv8846_position_locked(struct drv8846 *motor, u32 *position)
{
	struct akm09970_sample_snapshot sample;
	u64 since = ktime_get_boottime_ns();
	ktime_t deadline = ktime_add_ms(ktime_get(), DRV8846_HALL_READY_MS);
	u32 previous = DRV8846_POSITION_UNKNOWN, sequence = 0;
	bool have_sample = false;
	int ret;

	for (;;) {
		ret = drv8846_check_safe(motor);
		if (ret)
			return ret;
		ret = akm09970_snapshot(motor->hall, &sample);
		if (ret && ret != -ENODATA)
			return ret;
		if (!ret && sample.timestamp_ns >= since &&
		    (!have_sample || sample.sequence != sequence)) {
			*position = drv8846_classify(motor, &sample);
			if (have_sample && *position == previous)
				return *position == DRV8846_POSITION_UNKNOWN ?
					-ERANGE : 0;
			sequence = sample.sequence;
			previous = *position;
			have_sample = true;
		}
		if (ktime_compare(ktime_get(), deadline) >= 0)
			return -ETIMEDOUT;
		msleep(DRV8846_HALL_POLL_MS);
	}
}

static int drv8846_move_checked_locked(struct drv8846 *motor, u32 direction)
{
	struct akm09970_sample_snapshot sample;
	u32 target, position, sequence = 0;
	u32 reason = DRV8846_STOP_HALL_ERROR;
	unsigned int stable = 0;
	u64 since;
	ktime_t deadline;
	unsigned long flags;
	int ret;

	if (direction != UP && direction != DOWN)
		return -EINVAL;
	ret = drv8846_check_safe(motor);
	if (ret)
		return ret;
	if (!motor->calibration_valid)
		return -ENODATA;
	if (!motor->hall)
		return -ENODEV;
	if (drv8846_is_armed(motor))
		return -EBUSY;
	ret = akm09970_acquire(motor->hall);
	if (ret)
		return ret;
	ret = drv8846_position_locked(motor, &position);
	if (ret)
		goto release;
	target = direction == UP ? DRV8846_POSITION_UP : DRV8846_POSITION_DOWN;
	if (position == target)
		goto release;

	since = ktime_get_boottime_ns();
	deadline = ktime_add_ms(ktime_get(), DRV8846_MAX_MOVE_MS);
	ret = drv8846_start_locked(motor, direction, motor->rampup_period_ns,
				   motor->rampup_duration_ms, true);
	if (ret)
		goto release;
	for (;;) {
		if (ktime_compare(ktime_get(), deadline) >= 0) {
			ret = -ETIMEDOUT;
			reason = DRV8846_STOP_TIMEOUT;
			break;
		}
		ret = drv8846_check_safe(motor);
		if (ret) {
			reason = DRV8846_STOP_ABORTED;
			break;
		}
		if (!drv8846_is_armed(motor)) {
			ret = READ_ONCE(motor->fault_latched) ? -EIO : -ETIMEDOUT;
			reason = DRV8846_STOP_NO_POSITION;
			break;
		}
		ret = akm09970_snapshot(motor->hall, &sample);
		if (ret)
			break;
		if (sample.timestamp_ns >= since && sample.sequence != sequence) {
			sequence = sample.sequence;
			position = drv8846_classify(motor, &sample);
			stable = position == target ? stable + 1 : 0;
			if (stable >= 2) {
				reason = DRV8846_STOP_POSITION;
				break;
			}
		}
		msleep(DRV8846_HALL_POLL_MS);
	}
	if (reason == DRV8846_STOP_NO_POSITION) {
		spin_lock_irqsave(&motor->safety_lock, flags);
		if (motor->last_stop_reason == DRV8846_STOP_TIMED)
			motor->last_stop_reason = reason;
		spin_unlock_irqrestore(&motor->safety_lock, flags);
	}
	drv8846_quiesce(motor, reason);
	/* An IRQ/suspend may win immediately before the confirmed cutoff. */
	if (!ret && READ_ONCE(motor->last_stop_reason) != DRV8846_STOP_POSITION)
		ret = -EIO;
	if (!ret && READ_ONCE(motor->cutoff_failed))
		ret = -EIO;
	/* Confirm again with the bridge off; motion-time detection is not final. */
	if (!ret) {
		ret = drv8846_position_locked(motor, &position);
		if (!ret && position != target)
			ret = -ERANGE;
		if (ret) {
			spin_lock_irqsave(&motor->safety_lock, flags);
			if (ret == -ERANGE || ret == -ETIMEDOUT)
				motor->timeout_latched = true;
			if (motor->last_stop_reason == DRV8846_STOP_POSITION)
				motor->last_stop_reason = ret == -ERANGE ?
					DRV8846_STOP_NO_POSITION : DRV8846_STOP_HALL_ERROR;
			spin_unlock_irqrestore(&motor->safety_lock, flags);
		}
	}
release:
	if (!ret)
		ret = drv8846_check_safe(motor);
	akm09970_release(motor->hall);
	return ret;
}

static int drv8846_start(struct drv8846 *motor, u32 direction,
			u32 period_ns, u32 duration_ms, bool automatic)
{
	int ret;

	mutex_lock(&motor->command_lock);
	if (motor->camera_owned)
		ret = -EBUSY;
	else if (automatic)
		ret = drv8846_move_checked_locked(motor, direction);
	else
		ret = drv8846_start_locked(motor, direction, period_ns,
					   duration_ms, false);
	mutex_unlock(&motor->command_lock);
	return ret;
}

static bool drv8846_valid_calibration(const struct drv8846_calibration *cal)
{
	bool disjoint = false;
	int endpoint, axis;

	if (cal->version != DRV8846_CALIBRATION_VERSION ||
	    cal->flags != DRV8846_CALIBRATION_DEVICE_VERIFIED ||
	    memchr_inv(cal->reserved, 0, sizeof(cal->reserved)))
		return false;
	for (axis = 0; axis < 3; axis++) {
		for (endpoint = 0; endpoint < 2; endpoint++) {
			if (cal->minimum[endpoint][axis] < S16_MIN ||
			    cal->maximum[endpoint][axis] > S16_MAX ||
			    cal->minimum[endpoint][axis] > cal->maximum[endpoint][axis])
				return false;
		}
		if (cal->maximum[DOWN][axis] < cal->minimum[UP][axis] ||
		    cal->maximum[UP][axis] < cal->minimum[DOWN][axis])
			disjoint = true;
	}
	return disjoint;
}

struct drv8846 *drv8846_get(struct device *consumer, const char *property)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct drv8846 *motor;
	int ret = -EPROBE_DEFER;

	np = of_parse_phandle(consumer->of_node, property, 0);
	if (!np)
		return ERR_PTR(-ENODEV);
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	device_lock(&pdev->dev);
	motor = platform_get_drvdata(pdev);
	if (pdev->dev.driver != &drv8846_driver.driver || !motor ||
	    READ_ONCE(motor->removing))
		goto unlock;
	if (!try_module_get(THIS_MODULE))
		goto unlock;
	if (!device_link_add(consumer, &pdev->dev,
			     DL_FLAG_AUTOREMOVE_CONSUMER)) {
		module_put(THIS_MODULE);
		ret = -ENOMEM;
		goto unlock;
	}
	kref_get(&motor->ref);
	device_unlock(&pdev->dev);
	return motor;
unlock:
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(drv8846_get);

void drv8846_put(struct drv8846 *motor)
{
	put_device(motor->dev);
	kref_put(&motor->ref, drv8846_release_ref);
	module_put(THIS_MODULE);
}
EXPORT_SYMBOL_GPL(drv8846_put);

int drv8846_camera_open(struct drv8846 *motor)
{
	int ret;

	mutex_lock(&motor->command_lock);
	if (motor->camera_owned)
		ret = -EBUSY;
	else {
		ret = drv8846_move_checked_locked(motor, UP);
		if (!ret) {
			motor->camera_owned = true;
			motor->camera_cancelled = false;
		}
	}
	mutex_unlock(&motor->command_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(drv8846_camera_open);

int drv8846_camera_ready(struct drv8846 *motor)
{
	u32 position;
	int ret;

	mutex_lock(&motor->command_lock);
	ret = drv8846_check_safe(motor);
	if (ret)
		goto unlock;
	if (!motor->camera_owned) {
		ret = -EPIPE;
		goto unlock;
	}
	if (motor->camera_cancelled) {
		ret = -ECANCELED;
		goto unlock;
	}
	if (!motor->calibration_valid || !motor->hall) {
		ret = -ENODATA;
		goto unlock;
	}
	ret = akm09970_acquire(motor->hall);
	if (ret)
		goto unlock;
	ret = drv8846_position_locked(motor, &position);
	if (!ret && position != DRV8846_POSITION_UP)
		ret = -ERANGE;
	if (!ret)
		ret = drv8846_check_safe(motor);
	akm09970_release(motor->hall);
unlock:
	mutex_unlock(&motor->command_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(drv8846_camera_ready);

int drv8846_camera_close(struct drv8846 *motor)
{
	int ret = 0;

	mutex_lock(&motor->command_lock);
	if (motor->camera_owned) {
		if (motor->camera_cancelled)
			ret = -ECANCELED;
		else
			ret = drv8846_move_checked_locked(motor, DOWN);
	}
	motor->camera_owned = false;
	mutex_unlock(&motor->command_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(drv8846_camera_close);

void drv8846_camera_abort(struct drv8846 *motor)
{
	bool stop = drv8846_request_stop(motor, false, false);

	mutex_lock(&motor->command_lock);
	if (stop) {
		if (!READ_ONCE(motor->removing))
			drv8846_quiesce(motor, DRV8846_STOP_ABORTED);
		drv8846_finish_stop_request(motor);
	}
	motor->camera_owned = false;
	mutex_unlock(&motor->command_lock);
}
EXPORT_SYMBOL_GPL(drv8846_camera_abort);

static int drv8846_fasync(int fd, struct file *file, int on)
{
	struct drv8846 *motor = file->private_data;

	return fasync_helper(fd, file, on, &motor->async);
}

static __poll_t drv8846_poll(struct file *file, poll_table *wait)
{
	struct drv8846 *motor = file->private_data;

	poll_wait(file, &motor->waitq, wait);
	if (READ_ONCE(motor->removing))
		return EPOLLHUP;
	if (atomic_xchg(&motor->move_done, 0))
		return EPOLLIN | EPOLLRDNORM;

	return 0;
}

static int drv8846_open(struct inode *inode, struct file *file)
{
	struct drv8846 *motor =
		container_of(file->private_data, struct drv8846, miscdev);
	int ret;

	if (!kref_get_unless_zero(&motor->ref))
		return -ENODEV;

	mutex_lock(&motor->lock);
	ret = READ_ONCE(motor->removing) ? -ENODEV : 0;
	mutex_unlock(&motor->lock);
	if (ret) {
		kref_put(&motor->ref, drv8846_release_ref);
		return ret;
	}

	file->private_data = motor;
	return nonseekable_open(inode, file);
}

static int drv8846_release(struct inode *inode, struct file *file)
{
	struct drv8846 *motor = file->private_data;

	drv8846_fasync(-1, file, 0);
	kref_put(&motor->ref, drv8846_release_ref);

	return 0;
}

static long drv8846_remaining_ms(struct drv8846 *motor)
{
	long remain = 0;

	mutex_lock(&motor->lock);
	if (drv8846_is_armed(motor) && hrtimer_active(&motor->watchdog)) {
		remain = ktime_to_ms(hrtimer_get_remaining(&motor->watchdog));
		remain -= motor->watchdog_grace_ms;
		remain = max(remain, 0L);
	}
	mutex_unlock(&motor->lock);

	return remain;
}

static long drv8846_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct drv8846 *motor = file->private_data;
	struct op_parameter params;
	struct drv8846_calibration calibration;
	u32 position;
	enum running_state state;
	unsigned long flags;
	u32 reason;
	u8 direction;
	long remain = 0;
	int ret;

	if (READ_ONCE(motor->removing))
		return -ENODEV;

	switch (cmd) {
	case MOTOR_IOC_SET_CALIBRATION:
	case MOTOR_IOC_CLEAR_CALIBRATION:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		if (cmd == MOTOR_IOC_SET_CALIBRATION) {
			if (copy_from_user(&calibration, (void __user *)arg,
					   sizeof(calibration)))
				return -EFAULT;
			if (!drv8846_valid_calibration(&calibration))
				return -EINVAL;
		}
		mutex_lock(&motor->command_lock);
		ret = drv8846_check_available(motor);
		if (!ret && (motor->camera_owned || drv8846_is_armed(motor)))
			ret = -EBUSY;
		if (!ret) {
			motor->calibration_valid = cmd == MOTOR_IOC_SET_CALIBRATION;
			if (motor->calibration_valid)
				motor->calibration = calibration;
			else
				memset(&motor->calibration, 0, sizeof(motor->calibration));
		}
		mutex_unlock(&motor->command_lock);
		return ret;
	case MOTOR_IOC_CLEAR_FAULTS:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		mutex_lock(&motor->command_lock);
		ret = drv8846_check_available(motor);
		if (!ret && (motor->camera_owned || drv8846_is_armed(motor)))
			ret = -EBUSY;
		if (!ret) {
			spin_lock_irqsave(&motor->safety_lock, flags);
			ret = gpiod_get_value(motor->fault);
			if (!ret) {
				motor->fault_latched = false;
				motor->timeout_latched = false;
			} else if (ret > 0) {
				ret = -EIO;
			}
			spin_unlock_irqrestore(&motor->safety_lock, flags);
		}
		mutex_unlock(&motor->command_lock);
		return ret;
	case MOTOR_IOC_GET_POSITION:
		mutex_lock(&motor->command_lock);
		ret = drv8846_check_available(motor);
		if (!ret && (!motor->calibration_valid || !motor->hall))
			ret = -ENODATA;
		if (!ret && drv8846_is_armed(motor))
			ret = -EBUSY;
		if (!ret) {
			ret = akm09970_acquire(motor->hall);
			if (!ret) {
				ret = drv8846_position_locked(motor, &position);
				akm09970_release(motor->hall);
			}
		}
		mutex_unlock(&motor->command_lock);
		if (ret)
			return ret;
		return copy_to_user((void __user *)arg, &position,
				    sizeof(position)) ? -EFAULT : 0;
	case MOTOR_IOC_SET_AUTORUN:
		if (copy_from_user(&direction, (void __user *)arg,
				   sizeof(direction)))
			return -EFAULT;
		return drv8846_start(motor, direction,
				     motor->rampup_period_ns,
				     motor->rampup_duration_ms, true);
	case MOTOR_IOC_SET_MANUALRUN:
		if (copy_from_user(&params, (void __user *)arg,
				   sizeof(params)))
			return -EFAULT;
		return drv8846_start(motor, params.dir, params.period_ns,
				     params.duration_ms, false);
	case MOTOR_IOC_STOP:
		if (!drv8846_request_stop(motor, false, false))
			return -ENODEV;
		mutex_lock(&motor->command_lock);
		ret = READ_ONCE(motor->removing) ? -ENODEV : 0;
		if (!ret) {
			/* STOP cancels later motion, retaining camera exclusivity. */
			if (motor->camera_owned)
				motor->camera_cancelled = true;
			hrtimer_cancel(&motor->watchdog);
			cancel_delayed_work_sync(&motor->phase_work);
			cancel_work_sync(&motor->stop_work);
			mutex_lock(&motor->lock);
			drv8846_stop_locked(motor, DRV8846_STOP_REQUESTED);
			mutex_unlock(&motor->lock);
		}
		drv8846_finish_stop_request(motor);
		mutex_unlock(&motor->command_lock);
		return ret;
	case MOTOR_IOC_GET_REMAIN_TIME:
		remain = drv8846_remaining_ms(motor);
		if (copy_to_user((void __user *)arg, &remain, sizeof(remain)))
			return -EFAULT;
		return 0;
	case MOTOR_IOC_GET_STATE:
		mutex_lock(&motor->lock);
		state = drv8846_is_armed(motor) ? motor->state : STILL;
		mutex_unlock(&motor->lock);
		if (copy_to_user((void __user *)arg, &state, sizeof(state)))
			return -EFAULT;
		return 0;
	case MOTOR_IOC_GET_CAMERA_STATE:
		mutex_lock(&motor->command_lock);
		reason = motor->camera_owned ? DRV8846_CAMERA_OWNED : 0;
		if (motor->camera_owned && motor->camera_cancelled)
			reason |= DRV8846_CAMERA_CANCELLED;
		mutex_unlock(&motor->command_lock);
		return copy_to_user((void __user *)arg, &reason,
				    sizeof(reason)) ? -EFAULT : 0;
	case MOTOR_IOC_GET_STOP_REASON:
		spin_lock_irqsave(&motor->safety_lock, flags);
		reason = motor->last_stop_reason;
		spin_unlock_irqrestore(&motor->safety_lock, flags);
		if (copy_to_user((void __user *)arg, &reason, sizeof(reason)))
			return -EFAULT;
		return 0;
	default:
		return -ENOTTY;
	}
}

#ifdef CONFIG_COMPAT
static long drv8846_compat_ioctl(struct file *file, unsigned int cmd,
				 unsigned long arg)
{
	struct drv8846 *motor = file->private_data;
	compat_long_t remain = 0;

	if (_IOC_TYPE(cmd) == MOTOR_MAGIC && _IOC_NR(cmd) == 0x10 &&
	    _IOC_DIR(cmd) == _IOC_READ &&
	    _IOC_SIZE(cmd) == sizeof(compat_long_t)) {
		if (READ_ONCE(motor->removing))
			return -ENODEV;
		remain = drv8846_remaining_ms(motor);
		if (copy_to_user(compat_ptr(arg), &remain, sizeof(remain)))
			return -EFAULT;
		return 0;
	}

	return drv8846_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations drv8846_fops = {
	.owner = THIS_MODULE,
	.open = drv8846_open,
	.release = drv8846_release,
	.unlocked_ioctl = drv8846_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drv8846_compat_ioctl,
#endif
	.poll = drv8846_poll,
	.fasync = drv8846_fasync,
};

static int drv8846_read_config(struct device *dev, struct drv8846 *motor)
{
	struct device_node *np = dev->of_node;

	motor->rampup_period_ns = 625000;
	motor->high_period_ns = 52083;
	motor->rampdown_period_ns = 625000;
	motor->rampup_duration_ms = 50;
	motor->high_duration_ms = 720;
	motor->rampdown_duration_ms = 50;
	motor->step_mode = 2;

	of_property_read_u32(np, "ti,rampup-period-ns",
			     &motor->rampup_period_ns);
	of_property_read_u32(np, "ti,high-period-ns",
			     &motor->high_period_ns);
	of_property_read_u32(np, "ti,rampdown-period-ns",
			     &motor->rampdown_period_ns);
	of_property_read_u32(np, "ti,rampup-duration-ms",
			     &motor->rampup_duration_ms);
	of_property_read_u32(np, "ti,high-duration-ms",
			     &motor->high_duration_ms);
	of_property_read_u32(np, "ti,rampdown-duration-ms",
			     &motor->rampdown_duration_ms);
	of_property_read_u32(np, "ti,step-mode", &motor->step_mode);

	if (motor->rampup_period_ns < DRV8846_MIN_PERIOD_NS ||
	    motor->rampup_period_ns > DRV8846_MAX_PERIOD_NS ||
	    motor->high_period_ns < DRV8846_MIN_PERIOD_NS ||
	    motor->high_period_ns > DRV8846_MAX_PERIOD_NS ||
	    motor->rampdown_period_ns < DRV8846_MIN_PERIOD_NS ||
	    motor->rampdown_period_ns > DRV8846_MAX_PERIOD_NS ||
	    !motor->rampup_duration_ms || !motor->high_duration_ms ||
	    !motor->rampdown_duration_ms ||
	    motor->rampup_duration_ms > DRV8846_MAX_MOVE_MS ||
	    motor->high_duration_ms > DRV8846_MAX_MOVE_MS ||
	    motor->rampdown_duration_ms > DRV8846_MAX_MOVE_MS ||
	    motor->rampup_duration_ms + motor->high_duration_ms +
	    motor->rampdown_duration_ms + DRV8846_DOWN_EXTRA_MS >
	    DRV8846_MAX_MOVE_MS || motor->step_mode > 3)
		return dev_err_probe(dev, -EINVAL, "invalid motor timing\n");

	return 0;
}

static int drv8846_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drv8846 *motor;
	int irq, ret;

	motor = kzalloc_obj(*motor, GFP_KERNEL);
	if (!motor)
		return -ENOMEM;

	motor->dev = dev;
	motor->fault_irq = -1;
	motor->last_stop_reason = DRV8846_STOP_UNKNOWN;
	kref_init(&motor->ref);
	mutex_init(&motor->lock);
	mutex_init(&motor->command_lock);
	spin_lock_init(&motor->safety_lock);
	init_waitqueue_head(&motor->waitq);
	atomic_set(&motor->move_done, 0);
	INIT_DELAYED_WORK(&motor->phase_work, drv8846_phase_work);
	INIT_WORK(&motor->stop_work, drv8846_stop_work);
	hrtimer_setup(&motor->watchdog, drv8846_watchdog,
		      CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	if (of_find_property(dev->of_node, "ti,hall-sensor", NULL)) {
		motor->hall = akm09970_get(dev, "ti,hall-sensor");
		if (IS_ERR(motor->hall)) {
			ret = dev_err_probe(dev, PTR_ERR(motor->hall),
					    "failed to acquire Hall sensor\n");
			motor->hall = NULL;
			goto err_free;
		}
	}

	ret = drv8846_read_config(dev, motor);
	if (ret)
		goto err_free;

	motor->mode0 = devm_gpiod_get(dev, "mode0", GPIOD_OUT_LOW);
	if (IS_ERR(motor->mode0)) {
		ret = dev_err_probe(dev, PTR_ERR(motor->mode0),
				    "failed to get mode0 GPIO\n");
		goto err_free;
	}
	motor->mode1 = devm_gpiod_get(dev, "mode1", GPIOD_OUT_LOW);
	if (IS_ERR(motor->mode1)) {
		ret = dev_err_probe(dev, PTR_ERR(motor->mode1),
				    "failed to get mode1 GPIO\n");
		goto err_free;
	}
	motor->dir = devm_gpiod_get(dev, "dir", GPIOD_OUT_LOW);
	if (IS_ERR(motor->dir)) {
		ret = dev_err_probe(dev, PTR_ERR(motor->dir),
				    "failed to get direction GPIO\n");
		goto err_free;
	}
	motor->sleep = devm_gpiod_get(dev, "sleep", GPIOD_OUT_LOW);
	if (IS_ERR(motor->sleep)) {
		ret = dev_err_probe(dev, PTR_ERR(motor->sleep),
				    "failed to get sleep GPIO\n");
		goto err_free;
	}
	motor->enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(motor->enable)) {
		ret = dev_err_probe(dev, PTR_ERR(motor->enable),
				    "failed to get enable GPIO\n");
		goto err_free;
	}
	motor->fault = devm_gpiod_get(dev, "fault", GPIOD_IN);
	if (IS_ERR(motor->fault)) {
		ret = dev_err_probe(dev, PTR_ERR(motor->fault),
				    "failed to get fault GPIO\n");
		goto err_free;
	}

	if (gpiod_cansleep(motor->sleep) || gpiod_cansleep(motor->enable) ||
	    gpiod_cansleep(motor->fault)) {
		ret = dev_err_probe(dev, -EINVAL,
				    "motor safety GPIOs must be IRQ-safe\n");
		goto err_free;
	}

	motor->pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(motor->pwm)) {
		ret = dev_err_probe(dev, PTR_ERR(motor->pwm),
				    "failed to get motor PWM\n");
		goto err_free;
	}
	ret = drv8846_pwm_apply(motor, motor->rampup_period_ns, false);
	if (ret) {
		dev_err(dev, "failed to disable motor PWM: %d\n", ret);
		goto err_free;
	}

	irq = gpiod_to_irq(motor->fault);
	if (irq < 0) {
		ret = irq;
		goto err_free;
	}
	ret = devm_request_irq(dev, irq, drv8846_fault_irq,
			       IRQF_TRIGGER_FALLING, dev_name(dev), motor);
	if (ret) {
		dev_err(dev, "failed to request fault IRQ: %d\n", ret);
		goto err_free;
	}
	motor->fault_irq = irq;

	motor->miscdev.minor = MISC_DYNAMIC_MINOR;
	motor->miscdev.name = DRV8846_MISC_NAME;
	motor->miscdev.fops = &drv8846_fops;
	motor->miscdev.parent = dev;
	ret = misc_register(&motor->miscdev);
	if (ret)
		goto err_free;

	platform_set_drvdata(pdev, motor);
	return 0;

err_free:
	if (motor->fault_irq >= 0) {
		devm_free_irq(dev, motor->fault_irq, motor);
		cancel_work_sync(&motor->stop_work);
	}
	if (motor->hall)
		akm09970_put(motor->hall);
	kfree(motor);
	return ret;
}

static void drv8846_remove(struct platform_device *pdev)
{
	struct drv8846 *motor = platform_get_drvdata(pdev);

	drv8846_request_stop(motor, false, true);
	misc_deregister(&motor->miscdev);

	mutex_lock(&motor->command_lock);
	devm_free_irq(&pdev->dev, motor->fault_irq, motor);
	hrtimer_cancel(&motor->watchdog);
	cancel_delayed_work_sync(&motor->phase_work);
	cancel_work_sync(&motor->stop_work);
	mutex_lock(&motor->lock);
	drv8846_stop_locked(motor, DRV8846_STOP_ABORTED);
	mutex_unlock(&motor->lock);
	motor->camera_owned = false;
	if (motor->hall) {
		akm09970_put(motor->hall);
		motor->hall = NULL;
	}
	mutex_unlock(&motor->command_lock);

	wake_up_interruptible_poll(&motor->waitq, EPOLLHUP);
	kref_put(&motor->ref, drv8846_release_ref);
}

static void drv8846_shutdown(struct platform_device *pdev)
{
	struct drv8846 *motor = platform_get_drvdata(pdev);

	if (!drv8846_request_stop(motor, true, false))
		return;

	mutex_lock(&motor->command_lock);
	hrtimer_cancel(&motor->watchdog);
	cancel_delayed_work_sync(&motor->phase_work);
	cancel_work_sync(&motor->stop_work);
	mutex_lock(&motor->lock);
	drv8846_stop_locked(motor, DRV8846_STOP_ABORTED);
	mutex_unlock(&motor->lock);
	motor->camera_owned = false;
	mutex_unlock(&motor->command_lock);
}

static int drv8846_suspend(struct device *dev)
{
	struct drv8846 *motor = dev_get_drvdata(dev);

	if (!drv8846_request_stop(motor, true, false))
		return -ENODEV;

	mutex_lock(&motor->command_lock);
	hrtimer_cancel(&motor->watchdog);
	cancel_delayed_work_sync(&motor->phase_work);
	cancel_work_sync(&motor->stop_work);
	mutex_lock(&motor->lock);
	drv8846_stop_locked(motor, DRV8846_STOP_ABORTED);
	mutex_unlock(&motor->lock);
	motor->camera_owned = false;
	mutex_unlock(&motor->command_lock);

	return 0;
}

static int drv8846_resume(struct device *dev)
{
	struct drv8846 *motor = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = 0;

	mutex_lock(&motor->command_lock);
	spin_lock_irqsave(&motor->safety_lock, flags);
	if (motor->removing)
		ret = -ENODEV;
	else
		motor->suspended = false;
	spin_unlock_irqrestore(&motor->safety_lock, flags);
	mutex_unlock(&motor->command_lock);

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(drv8846_pm_ops,
				drv8846_suspend, drv8846_resume);

static const struct of_device_id drv8846_of_match[] = {
	{ .compatible = "ti,drv8846" },
	{ }
};
MODULE_DEVICE_TABLE(of, drv8846_of_match);

static struct platform_driver drv8846_driver = {
	.probe = drv8846_probe,
	.remove = drv8846_remove,
	.shutdown = drv8846_shutdown,
	.driver = {
		.name = "raphael-drv8846",
		.pm = pm_sleep_ptr(&drv8846_pm_ops),
		.of_match_table = drv8846_of_match,
	},
};
module_platform_driver(drv8846_driver);

MODULE_DESCRIPTION("Raphael DRV8846 popup camera motor");
MODULE_LICENSE("GPL");
