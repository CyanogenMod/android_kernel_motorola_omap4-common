/*
 * Copyright (C) 2009 Kionix, Inc.
 * Copyright (C) 2009-2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/kxtf9.h>

#define NAME				"kxtf9"

/* Maximum polled-device-reported g value */
#define G_MAX			8000

#define SHIFT_ADJ_2G		4
#define SHIFT_ADJ_4G		3
#define SHIFT_ADJ_8G		2

#define AXISDATA_REG		0x06

/* status registers (read only) */
#define INT_SRC_REG1		0x15
#define INT_SRC_REG2		0x16
#define STATUS_REG		0x18
#define INT_REL			0x1A

/* control registers (read/write) */
#define CTRL_REG1		0x1B
#define CTRL_REG2		0x1C
#define CTRL_REG3		0x1D
#define INT_CTRL_REG1		0x1E
#define INT_CTRL_REG2		0x1F
#define INT_CTRL_REG3		0x20
#define DATA_CTRL_REG		0x21

#define TDT_TIMER		0x2B
#define TDT_H_THRESH		0x2C
#define TDT_L_THRESH		0x2D
#define TDT_TAP_TIMER		0x2E
#define TDT_TOTAL_TIMER		0x2F
#define TDT_LATENCY_TIMER	0x30
#define TDT_WINDOW_TIMER	0x31

#define WUF_THRESH		0x5A

/* interrupt source register 1 bits */
#define TLE			(1 << 5)
#define TRI			(1 << 4)
#define TDO			(1 << 3)
#define TUP			(1 << 2)
#define TFD			(1 << 1)
#define TFU			(1 << 0)

/* interrupt source register 2 bits */
#define DRDY			(1 << 4)
#define TDTS1			(1 << 3)
#define TDTS0			(1 << 2)
#define WUFS			(1 << 1)
#define TPS			(1 << 0)

/* status register bits */
#define INT			(1 << 4)

/* control register 1 bits
 * PC1 must be set to 0 in order to properly
 * change the majority of registers */
#define PC1			(1 << 7)
#define RES			(1 << 6)
#define DRDYE			(1 << 5)
#define GSEL1			(1 << 4)
#define GSEL0			(1 << 3)
#define TDTE			(1 << 2)
#define WUFE			(1 << 1)
#define TPE			(1 << 0)

/* control register 3 bits */
#define SRST			(1 << 7)
#define OTPA			(1 << 6)
#define OTPB			(1 << 5)
#define DCST			(1 << 4)
#define OTDTA			(1 << 3)
#define OTDTB			(1 << 2)
#define OWUFA			(1 << 1)
#define OWUFB			(1 << 0)

/* interrupt control register 1 bits */
#define IEN			(1 << 5)
#define IEA			(1 << 4)
#define IEL			(1 << 3)
#define IEU			(1 << 2)

/* interrupt control register 3 bits */
#define TLEM			(1 << 5)
#define TRIM			(1 << 4)
#define TDOM			(1 << 3)
#define TUPM			(1 << 2)
#define TFDM			(1 << 1)
#define TFUM			(1 << 0)

/* data control register bits */
#define ODR12_5			0x00    /* Do not use per Kionix */
#define ODR25			0x01
#define ODR50			0x02
#define ODR100			0x03
#define ODR200			0x04
#define ODR400			0x05
#define ODR800			0x06

#define G_2G			0x00
#define G_4G			0x08
#define G_8G			0x10

#define FUZZ			32
#define FLAT			32
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
#define AUTO_INCREMENT		0x80

static struct {
	unsigned int cutoff;
	unsigned int mask;
} odr_table[] = {
	{
	3,    ODR800}, {
	5,    ODR400}, {
	10,   ODR200}, {
	20,   ODR100}, {
	40,   ODR50}, {
	100,  ODR25},
};

struct kxtf9_data {
	struct i2c_client *client;
	struct kxtf9_platform_data *pdata;

	struct mutex lock;

	struct delayed_work input_work;
	struct input_dev *input_dev;

	struct work_struct irq_work;
	struct workqueue_struct *work_queue;
	int irq;

	struct notifier_block pm_notifier;

	int hw_initialized;
	atomic_t enabled;
	int on_before_suspend;

	u8 shift_adj;
	u8 resume_state[14];

	struct regulator *regulator;
};

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct kxtf9_data *kxtf9_misc_data;


static int kxtf9_i2c_read(struct kxtf9_data *tf9, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = tf9->client->addr,
		 .flags = tf9->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = tf9->client->addr,
		 .flags = (tf9->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(tf9->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&tf9->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int kxtf9_i2c_write(struct kxtf9_data *tf9, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = tf9->client->addr,
		 .flags = tf9->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(tf9->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&tf9->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int kxtf9_hw_init(struct kxtf9_data *tf9)
{
	int err = -1;
	u8 buf[8];

	buf[0] = (CTRL_REG1);
	buf[1] = tf9->resume_state[0] & ~PC1;
	buf[2] = tf9->resume_state[1];
	buf[3] = tf9->resume_state[2];
	buf[4] = tf9->resume_state[3];
	buf[5] = tf9->resume_state[4];
	buf[6] = tf9->resume_state[5];
	buf[7] = tf9->resume_state[6];
	err = kxtf9_i2c_write(tf9, buf, 7);
	if (err < 0)
		return err;

	buf[0] = (TDT_TIMER);
	buf[1] = tf9->resume_state[7];
	buf[2] = tf9->resume_state[8];
	buf[3] = tf9->resume_state[9];
	buf[4] = tf9->resume_state[10];
	buf[5] = tf9->resume_state[11];
	buf[6] = tf9->resume_state[12];
	buf[7] = tf9->resume_state[13];
	err = kxtf9_i2c_write(tf9, buf, 7);
	if (err < 0)
		return err;

	tf9->hw_initialized = 1;

	return 0;
}

static void kxtf9_device_power_off(struct kxtf9_data *tf9)
{
	int err;
	u8 buf[2] = { CTRL_REG1, tf9->resume_state[0] & ~PC1 };

	if (tf9->pdata->tdt_always_on)
		return;

	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		dev_err(&tf9->client->dev, "soft power off failed\n");

	/*
	 * PC1 and RES must both be 0 to get proper standby with
	 * current savings. Setting RES to 0 sets the resolution to 8 bit
	 */
	buf[1] = tf9->resume_state[0] & ~(PC1 | RES);
	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		dev_err(&tf9->client->dev, "current save failed\n");

	if (tf9->regulator) {
		regulator_disable(tf9->regulator);
		tf9->hw_initialized = 0;
	}
}

static int kxtf9_device_power_on(struct kxtf9_data *tf9)
{
	int err;
	u8 buf[2] = { CTRL_REG1, tf9->resume_state[0] };

	if (tf9->pdata->tdt_always_on && tf9->hw_initialized)
		return 0;

	if (tf9->regulator) {
		err = regulator_enable(tf9->regulator);
		if (err < 0)
			return err;
	}

	if (!tf9->hw_initialized) {
		err = kxtf9_hw_init(tf9);
		if (err < 0) {
			kxtf9_device_power_off(tf9);
			return err;
		}
	}

	/* bring out of standby */
	return kxtf9_i2c_write(tf9, buf, 1);
}

static int kxtf9_get_interrupt_data(struct kxtf9_data *tf9, int *intr)
{
	int err = -1;
	u8 buf[6];

	buf[0] = (INT_SRC_REG1);
	err = kxtf9_i2c_read(tf9, buf, 6);
	if (err < 0)
		return err;

	intr[0] = buf[0];
	intr[1] = buf[1];
	intr[2] = buf[3];

	return err;
}

static void kxtf9_report_event(struct kxtf9_data *tf9, int *intr)
{
	if (intr[2] & INT) {
		if (intr[1] & DRDY)
			pr_info("new accel data available\n");
		if (intr[1] & TPS)
			pr_info("tilt position has changed\n");
		switch (intr[1] & (TDTS1 | TDTS0)) {
		case TDTS0:
			pr_info("single tap\n");
			break;
		case TDTS1:
			pr_info("double tap\n");
			break;
		}
		switch (intr[0]) {
		case TLE:
			pr_info("x negative reported\n");
			break;
		case TRI:
			pr_info("x positive reported\n");
			break;
		case TDO:
			pr_info("y negative reported\n");
			break;
		case TUP:
			pr_info("y positive reported\n");
			break;
		case TFD:
			pr_info("z negative reported\n");
			break;
		case TFU:
			pr_info("z positive reported\n");
			break;
		}
	}
}



static irqreturn_t kxtf9_isr(int irq, void *dev)
{
	struct kxtf9_data *tf9 = dev;

	queue_work(tf9->work_queue, &tf9->irq_work);

	return IRQ_HANDLED;
}

static void kxtf9_irq_work_func(struct work_struct *work)
{
	struct kxtf9_data *tf9 = container_of(work, struct kxtf9_data,
						irq_work);
	int intr[3] = { 0 };
	int err;

	mutex_lock(&tf9->lock);

	err = kxtf9_get_interrupt_data(tf9, intr);
	if (err < 0)
		dev_err(&tf9->client->dev, "get_acceleration_data failed\n");
	else
		kxtf9_report_event(tf9, intr);

	mutex_unlock(&tf9->lock);
}

int kxtf9_update_g_range(struct kxtf9_data *tf9, u8 new_g_range)
{
	int err;
	u8 shift;
	u8 range = tf9->resume_state[0] & ~(GSEL1 | GSEL0);
	u8 buf[2];

	switch (new_g_range) {
	case 2:
		range |= G_2G;
		shift = SHIFT_ADJ_2G;
		break;
	case 4:
		range |= G_4G;
		shift = SHIFT_ADJ_4G;
		break;
	case 8:
		range |= G_8G;
		shift = SHIFT_ADJ_8G;
		break;
	default:
		return -EINVAL;
	}

	if (atomic_read(&tf9->enabled)) {
		buf[0] = CTRL_REG1;
		buf[1] = tf9->resume_state[0] & ~PC1;
		err = kxtf9_i2c_write(tf9, buf, 1);
		if (err < 0)
			return err;

		buf[1] = range & ~PC1;
		err = kxtf9_i2c_write(tf9, buf, 1);
		if (err < 0)
			return err;

		buf[1] = range;
		err = kxtf9_i2c_write(tf9, buf, 1);
		if (err < 0)
			return err;
	}

	tf9->resume_state[0] = range;
	tf9->shift_adj = shift;

	return 0;
}

int kxtf9_update_odr(struct kxtf9_data *tf9, int poll_interval)
{
	int err = -1;
	int i;
	u8 config;
	u8 buf[2];

	/*
	 * Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching
	 */
	for (i = 0; i < ARRAY_SIZE(odr_table); i++) {
		config = odr_table[i].mask;
		if (poll_interval < odr_table[i].cutoff)
			break;
	}

	/*
	 * If device is currently enabled, we need to write new
	 *  configuration out to it
	 */
	if (atomic_read(&tf9->enabled)) {
		buf[0] = CTRL_REG1;
		buf[1] = tf9->resume_state[0] & ~PC1;
		err = kxtf9_i2c_write(tf9, buf, 1);
		if (err < 0)
			return err;

		buf[0] = DATA_CTRL_REG;
		buf[1] = config;
		err = kxtf9_i2c_write(tf9, buf, 1);
		if (err < 0)
			return err;

		buf[0] = CTRL_REG1;
		buf[1] = tf9->resume_state[0];
		err = kxtf9_i2c_write(tf9, buf, 1);
		if (err < 0)
			return err;
	}

	tf9->resume_state[6] = config;

	return 0;
}

static int kxtf9_get_acceleration_data(struct kxtf9_data *tf9, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];

	acc_data[0] = (AXISDATA_REG);
	err = kxtf9_i2c_read(tf9, acc_data, 6);
	if (err < 0)
		return err;

	xyz[0] = (int) (((acc_data[1]) << 8) | acc_data[0]);
	xyz[1] = (int) (((acc_data[3]) << 8) | acc_data[2]);
	xyz[2] = (int) (((acc_data[5]) << 8) | acc_data[4]);

	xyz[0] = (xyz[0] & 0x8000) ? (xyz[0] | 0xFFFF0000) : (xyz[0]);
	xyz[1] = (xyz[1] & 0x8000) ? (xyz[1] | 0xFFFF0000) : (xyz[1]);
	xyz[2] = (xyz[2] & 0x8000) ? (xyz[2] | 0xFFFF0000) : (xyz[2]);

	xyz[0] >>= tf9->shift_adj;
	xyz[1] >>= tf9->shift_adj;
	xyz[2] >>= tf9->shift_adj;

	return err;
}

static void kxtf9_report_values(struct kxtf9_data *tf9, int *xyz)
{
	input_report_abs(tf9->input_dev, ABS_X, xyz[0]);
	input_report_abs(tf9->input_dev, ABS_Y, xyz[1]);
	input_report_abs(tf9->input_dev, ABS_Z, xyz[2]);
	input_sync(tf9->input_dev);
}

static int kxtf9_enable(struct kxtf9_data *tf9)
{
	int err;

	if (!atomic_cmpxchg(&tf9->enabled, 0, 1)) {

		err = kxtf9_device_power_on(tf9);
		if (err < 0) {
			atomic_set(&tf9->enabled, 0);
			return err;
		}
		schedule_delayed_work(&tf9->input_work,
				      msecs_to_jiffies(tf9->
						       pdata->poll_interval));
	}

	return 0;
}

static int kxtf9_disable(struct kxtf9_data *tf9)
{
	if (atomic_cmpxchg(&tf9->enabled, 1, 0)) {
		cancel_delayed_work_sync(&tf9->input_work);
		kxtf9_device_power_off(tf9);
	}

	return 0;
}

static int kxtf9_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = kxtf9_misc_data;

	return 0;
}

static long kxtf9_misc_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int err;
	int interval;
	struct kxtf9_data *tf9 = file->private_data;

	switch (cmd) {
	case KXTF9_IOCTL_GET_DELAY:
		interval = tf9->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case KXTF9_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 200)
			return -EINVAL;

		if (interval > tf9->pdata->min_interval)
			tf9->pdata->poll_interval = interval;
		else
			tf9->pdata->poll_interval = tf9->pdata->min_interval;
		err = kxtf9_update_odr(tf9, tf9->pdata->poll_interval);
		if (err < 0)
			return err;

		break;

	case KXTF9_IOCTL_SET_ENABLE:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 1)
			return -EINVAL;

		if (interval)
			kxtf9_enable(tf9);
		else
			kxtf9_disable(tf9);

		break;

	case KXTF9_IOCTL_GET_ENABLE:
		interval = atomic_read(&tf9->enabled);
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EINVAL;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations kxtf9_misc_fops = {
	.owner = THIS_MODULE,
	.open = kxtf9_misc_open,
	.unlocked_ioctl = kxtf9_misc_ioctl,
};

static struct miscdevice kxtf9_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &kxtf9_misc_fops,
};

static int kxtf9_resume(struct kxtf9_data *tf9)
{
	if (tf9->on_before_suspend)
		return kxtf9_enable(tf9);
	return 0;
}

static int kxtf9_suspend(struct kxtf9_data *tf9)
{
	tf9->on_before_suspend = atomic_read(&tf9->enabled);
	return kxtf9_disable(tf9);
}

static int kxtf9_pm_event(struct notifier_block *this, unsigned long event,
				void *ptr)
{
	struct kxtf9_data *tf9 = container_of(this, struct kxtf9_data,
						pm_notifier);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		kxtf9_suspend(tf9);
		break;
	case PM_POST_SUSPEND:
		kxtf9_resume(tf9);
	}
	return NOTIFY_DONE;
}

static void kxtf9_input_work_func(struct work_struct *work)
{
	struct kxtf9_data *tf9 = container_of((struct delayed_work *)work,
						  struct kxtf9_data,
						  input_work);
	int xyz[3] = { 0 };
	int err;

	mutex_lock(&tf9->lock);

	err = kxtf9_get_acceleration_data(tf9, xyz);
	if (err < 0)
		dev_err(&tf9->client->dev, "get_acceleration_data failed\n");
	else
		kxtf9_report_values(tf9, xyz);

	schedule_delayed_work(&tf9->input_work,
			      msecs_to_jiffies(tf9->pdata->poll_interval));
	mutex_unlock(&tf9->lock);
}

#ifdef KXTF9_OPEN_ENABLE
int kxtf9_input_open(struct input_dev *input)
{
	struct kxtf9_data *tf9 = input_get_drvdata(input);

	return kxtf9_enable(tf9);
}

void kxtf9_input_close(struct input_dev *dev)
{
	struct kxtf9_data *tf9 = input_get_drvdata(dev);

	kxtf9_disable(tf9);
}
#endif

static int kxtf9_validate_pdata(struct kxtf9_data *tf9)
{
	/* Enforce minimum polling interval */
	if (tf9->pdata->poll_interval < tf9->pdata->min_interval) {
		dev_err(&tf9->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int kxtf9_input_init(struct kxtf9_data *tf9)
{
	int err;

	INIT_DELAYED_WORK(&tf9->input_work, kxtf9_input_work_func);

	tf9->input_dev = input_allocate_device();
	if (!tf9->input_dev) {
		err = -ENOMEM;
		dev_err(&tf9->client->dev, "input device allocate failed\n");
		goto err0;
	}

#ifdef KXTF9_OPEN_ENABLE
	tf9->input_dev->open = kxtf9_input_open;
	tf9->input_dev->close = kxtf9_input_close;
#endif

	input_set_drvdata(tf9->input_dev, tf9);

	set_bit(EV_ABS, tf9->input_dev->evbit);

	input_set_abs_params(tf9->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(tf9->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(tf9->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	tf9->input_dev->name = "accelerometer";

	err = input_register_device(tf9->input_dev);
	if (err) {
		dev_err(&tf9->client->dev,
			"unable to register input polled device %s\n",
			tf9->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(tf9->input_dev);
err0:
	return err;
}

static void kxtf9_input_cleanup(struct kxtf9_data *tf9)
{
	input_unregister_device(tf9->input_dev);
	input_free_device(tf9->input_dev);
}

static int kxtf9_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct kxtf9_data *tf9;
	int err = 0;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto err0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}

	tf9 = kzalloc(sizeof(*tf9), GFP_KERNEL);
	if (tf9 == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&tf9->lock);
	mutex_lock(&tf9->lock);
	tf9->client = client;

	tf9->pdata = kzalloc(sizeof(*tf9->pdata), GFP_KERNEL);
	if (tf9->pdata == NULL) {
		err = -ENOMEM;
		goto err1;
	}

	memcpy(tf9->pdata, client->dev.platform_data, sizeof(*tf9->pdata));

	err = kxtf9_validate_pdata(tf9);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	INIT_WORK(&tf9->irq_work, kxtf9_irq_work_func);
	tf9->work_queue = create_singlethread_workqueue("kxtf9_wq");

	if (!tf9->work_queue) {
		err = -ENOMEM;
		dev_err(&client->dev, "cannot create work queue: %d\n", err);
		goto err1_1;
	}

	if (tf9->pdata->tdt_always_on) {
		err = request_irq(tf9->pdata->irq, kxtf9_isr,
			IRQF_TRIGGER_FALLING, "kxtf9_irq", tf9);
		if (err < 0) {
			dev_err(&client->dev, "request irq failed: %d\n", err);
			goto err1_2;
		}
	}

	i2c_set_clientdata(client, tf9);

	tf9->regulator = regulator_get(&client->dev, tf9->pdata->regulator);
	if (IS_ERR_OR_NULL(tf9->regulator)) {
		dev_warn(&client->dev, "unable to get regulator\n");
		tf9->regulator = NULL;
	}

	memset(tf9->resume_state, 0, ARRAY_SIZE(tf9->resume_state));

	tf9->resume_state[0] = PC1 | RES
				| (tf9->pdata->tdt_always_on ? TDTE : 0);
	tf9->resume_state[1] = 0;
	tf9->resume_state[2] = tf9->pdata->tdt_odr;
	tf9->resume_state[3] = IEN;
	tf9->resume_state[4] = 0;
	tf9->resume_state[5] = tf9->pdata->tdt_directions;
	tf9->resume_state[6] = ODR25;
	tf9->resume_state[7] = tf9->pdata->tdt_timer_init;
	tf9->resume_state[8] = tf9->pdata->tdt_h_thresh_init;
	tf9->resume_state[9] = tf9->pdata->tdt_l_thresh_init;
	tf9->resume_state[10] = tf9->pdata->tdt_tap_timer_init;
	tf9->resume_state[11] = tf9->pdata->tdt_total_timer_init;
	tf9->resume_state[12] = tf9->pdata->tdt_latency_timer_init;
	tf9->resume_state[13] = tf9->pdata->tdt_window_timer_init;

	err = kxtf9_device_power_on(tf9);
	if (err < 0)
		goto err2;

	atomic_set(&tf9->enabled, 1);

	err = kxtf9_update_g_range(tf9, tf9->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto err3;
	}

	err = kxtf9_update_odr(tf9, tf9->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err3;
	}

	err = kxtf9_input_init(tf9);
	if (err < 0)
		goto err3;

	kxtf9_misc_data = tf9;

	err = misc_register(&kxtf9_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "tf9d_device register failed\n");
		goto err4;
	}

	tf9->pm_notifier.notifier_call = kxtf9_pm_event;
	err = register_pm_notifier(&tf9->pm_notifier);
	if (err < 0)
		goto err5;

	kxtf9_device_power_off(tf9);

	/* As default, do not report information */
	atomic_set(&tf9->enabled, 0);

	mutex_unlock(&tf9->lock);

	dev_info(&client->dev, "kxtf9 probed\n");

	return 0;

err5:
	misc_deregister(&kxtf9_misc_device);
err4:
	kxtf9_input_cleanup(tf9);
err3:
	kxtf9_device_power_off(tf9);
err2:
	if (tf9->pdata->tdt_always_on)
		free_irq(tf9->pdata->irq, tf9);
	if (tf9->regulator)
		regulator_put(tf9->regulator);
err1_2:
	destroy_workqueue(tf9->work_queue);
err1_1:
	mutex_unlock(&tf9->lock);
	mutex_destroy(&tf9->lock);
	kfree(tf9->pdata);
err1:
	kfree(tf9);
err0:
	return err;
}

static int __devexit kxtf9_remove(struct i2c_client *client)
{
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);

	unregister_pm_notifier(&tf9->pm_notifier);
	misc_deregister(&kxtf9_misc_device);
	kxtf9_input_cleanup(tf9);
	kxtf9_device_power_off(tf9);
	if (tf9->pdata->tdt_always_on)
		free_irq(tf9->pdata->irq, tf9);
	if (tf9->regulator)
		regulator_put(tf9->regulator);
	destroy_workqueue(tf9->work_queue);
	mutex_destroy(&tf9->lock);
	kfree(tf9->pdata);
	kfree(tf9);

	return 0;
}

static const struct i2c_device_id kxtf9_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, kxtf9_id);

static struct i2c_driver kxtf9_driver = {
	.driver = {
		   .name = NAME,
		   },
	.probe = kxtf9_probe,
	.remove = __devexit_p(kxtf9_remove),
	.id_table = kxtf9_id,
};

static int __init kxtf9_init(void)
{
	pr_info("KXTF9 accelerometer driver\n");
	return i2c_add_driver(&kxtf9_driver);
}

static void __exit kxtf9_exit(void)
{
	i2c_del_driver(&kxtf9_driver);
	return;
}

module_init(kxtf9_init);
module_exit(kxtf9_exit);

MODULE_DESCRIPTION("KXTF9 accelerometer driver");
MODULE_AUTHOR("Kionix");
MODULE_LICENSE("GPL");
