/*
 * Copyright (C) 2009-2010 Motorola, Inc.
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

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/suspend.h>
#include <linux/slab.h>

#include <linux/lis3dh.h>

#define NAME				"lis3dh"

/** Maximum polled-device-reported g value */
#define G_MAX			16000

#define SHIFT_ADJ_2G		4
#define SHIFT_ADJ_4G		3
#define SHIFT_ADJ_8G		2
#define SHIFT_ADJ_16G		1

#define AXISDATA_REG		0x28

/* ctrl 1: ODR3 ODR2 ODR1 ODR0 LPen Zen Yen Xen */
#define CTRL_REG1		0x20	/* power control reg */
#define CTRL_REG2		0x21	/* power control reg */

#define CTRL_REG3		0x22	/* power control reg */
/* ctrl 4: BDU BLE FS1 FS0 HR ST1 ST0 SIM */
#define CTRL_REG4		0x23	/* interrupt control reg */

#define PM_LOW          	0x08
#define PM_NORMAL       	0x00
#define ENABLE_ALL_AXES 	0x07

#define ODR_OFF		0x00
#define ODR1		0x10
#define ODR10		0x20
#define ODR25		0x30
#define ODR50		0x40
#define ODR100		0x50
#define ODR200		0x60
#define ODR400		0x70
#define ODR1250		0x90

#define G_2G		0x00
#define G_4G		0x10
#define G_8G		0x20
#define G_16G		0x30
#define HIGH_RES	0x08

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
	3,    PM_NORMAL | ODR1250}, {
	5,    PM_NORMAL |  ODR400}, {
	10,   PM_NORMAL |  ODR200}, {
	20,   PM_NORMAL |  ODR100}, {
	40,   PM_NORMAL |  ODR50}, {
	100,  PM_NORMAL |  ODR25}, {
	1000, PM_NORMAL |  ODR10}, {
	0,    PM_NORMAL |  ODR1},
};

struct lis3dh_data {
	struct i2c_client *client;
	struct lis3dh_platform_data *pdata;

	struct mutex lock;

	struct delayed_work input_work;
	struct input_dev *input_dev;

	int hw_initialized;
	atomic_t enabled;
	int on_before_suspend;

	u8 shift_adj;
	u8 resume_state[5];
};

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct lis3dh_data *lis3dh_misc_data;

static struct notifier_block lis3dh_pm_notifier;

static int lis3dh_i2c_read(struct lis3dh_data *lis, u8 * buf, int len)
{
	lis->client->flags = 0;
	int err = i2c_smbus_read_i2c_block_data(lis->client, buf[0], len, buf);
	if (err != len) {
		dev_err(&lis->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis3dh_i2c_write(struct lis3dh_data *lis, u8 * buf, int len)
{
	lis->client->flags = 0;
	int err = i2c_smbus_write_i2c_block_data(lis->client, buf[0], len, buf+1);
	if (err) {
		dev_err(&lis->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis3dh_hw_init(struct lis3dh_data *lis)
{
	int err = -1;
	u8 buf[6];

	buf[0] = (AUTO_INCREMENT | CTRL_REG1);
	buf[1] = lis->resume_state[0];
	buf[2] = lis->resume_state[1];
	buf[3] = lis->resume_state[2];
	buf[4] = lis->resume_state[3];
	buf[5] = lis->resume_state[4];
	err = lis3dh_i2c_write(lis, buf, 5);
	if (err < 0)
		return err;

	lis->hw_initialized = 1;

	return 0;
}

static void lis3dh_device_power_off(struct lis3dh_data *lis)
{
	int err;
	u8 buf[2] = { CTRL_REG1, ODR_OFF };

	err = lis3dh_i2c_write(lis, buf, 1);
	if (err < 0)
		dev_err(&lis->client->dev, "soft power off failed\n");

	if (lis->pdata->power_off) {
		lis->pdata->power_off();
		lis->hw_initialized = 0;
	}
}

static int lis3dh_device_power_on(struct lis3dh_data *lis)
{
	int err;

	if (lis->pdata->power_on) {
		err = lis->pdata->power_on();
		if (err < 0)
			return err;
	}

	if (!lis->hw_initialized) {
		err = lis3dh_hw_init(lis);
		if (err < 0) {
			lis3dh_device_power_off(lis);
			return err;
		}
	}

	return 0;
}

int lis3dh_update_g_range(struct lis3dh_data *lis, int new_g_range)
{
	int err;
	u8 shift;
	u8 buf[2];

	switch (new_g_range) {
	case 2:
		buf[1] = G_2G | HIGH_RES;
		shift = SHIFT_ADJ_2G;
		break;
	case 4:
		buf[1] = G_4G | HIGH_RES;
		shift = SHIFT_ADJ_4G;
		break;
	case 8:
		buf[1] = G_8G | HIGH_RES;
		shift = SHIFT_ADJ_8G;
		break;
	case 16:
		buf[1] = G_16G | HIGH_RES;
		shift = SHIFT_ADJ_16G;
		break;
	default:
		return -EINVAL;
	}

	if (atomic_read(&lis->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = CTRL_REG4;
		err = lis3dh_i2c_write(lis, buf, 1);
		if (err < 0)
			return err;
	}

	lis->resume_state[3] = buf[1];
	lis->shift_adj = shift;

	return 0;
}

int lis3dh_update_odr(struct lis3dh_data *lis, int poll_interval)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(odr_table); i++) {
		config[1] = odr_table[i].mask;
		if (poll_interval < odr_table[i].cutoff)
			break;
	}

	config[1] |= ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&lis->enabled)) {
		config[0] = CTRL_REG1;
		err = lis3dh_i2c_write(lis, config, 1);
		if (err < 0)
			return err;
	}

	lis->resume_state[0] = config[1];

	return 0;
}

static int lis3dh_get_acceleration_data(struct lis3dh_data *lis, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];

	acc_data[0] = (AUTO_INCREMENT | AXISDATA_REG);
	err = lis3dh_i2c_read(lis, acc_data, 6);
	if (err < 0)
		return err;

	xyz[0] = (int) (((acc_data[1]) << 8) | acc_data[0]);
	xyz[1] = (int) (((acc_data[3]) << 8) | acc_data[2]);
	xyz[2] = (int) (((acc_data[5]) << 8) | acc_data[4]);

	xyz[0] = (xyz[0] & 0x8000) ? (xyz[0] | 0xFFFF0000) : (xyz[0]);
	xyz[1] = (xyz[1] & 0x8000) ? (xyz[1] | 0xFFFF0000) : (xyz[1]);
	xyz[2] = (xyz[2] & 0x8000) ? (xyz[2] | 0xFFFF0000) : (xyz[2]);

	xyz[0] >>= lis->shift_adj;
	xyz[1] >>= lis->shift_adj;
	xyz[2] >>= lis->shift_adj;

	return err;
}

static void lis3dh_report_values(struct lis3dh_data *lis, int *xyz)
{
	input_report_abs(lis->input_dev, ABS_X, xyz[0]);
	input_report_abs(lis->input_dev, ABS_Y, xyz[1]);
	input_report_abs(lis->input_dev, ABS_Z, xyz[2]);
	input_sync(lis->input_dev);
}

static int lis3dh_enable(struct lis3dh_data *lis)
{
	int err;

	if (!atomic_cmpxchg(&lis->enabled, 0, 1)) {

		err = lis3dh_device_power_on(lis);
		if (err < 0) {
			atomic_set(&lis->enabled, 0);
			return err;
		}
		schedule_delayed_work(&lis->input_work,
				      msecs_to_jiffies(lis->
						       pdata->poll_interval));
	}

	return 0;
}

static int lis3dh_disable(struct lis3dh_data *lis)
{
	if (atomic_cmpxchg(&lis->enabled, 1, 0)) {
		cancel_delayed_work_sync(&lis->input_work);
		lis3dh_device_power_off(lis);
	}

	return 0;
}

static int lis3dh_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = lis3dh_misc_data;

	return 0;
}

static long lis3dh_misc_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int err;
	int interval;
	struct lis3dh_data *lis = file->private_data;

	switch (cmd) {
	case LIS3DH_IOCTL_GET_DELAY:
		interval = lis->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case LIS3DH_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 200)
			return -EINVAL;

		if (interval > lis->pdata->min_interval)
			lis->pdata->poll_interval = interval;
		else
			lis->pdata->poll_interval = lis->pdata->min_interval;
		err = lis3dh_update_odr(lis, lis->pdata->poll_interval);
		if (err < 0)
			return err;

		break;

	case LIS3DH_IOCTL_SET_ENABLE:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 1)
			return -EINVAL;

		if (interval)
			lis3dh_enable(lis);
		else
			lis3dh_disable(lis);

		break;

	case LIS3DH_IOCTL_GET_ENABLE:
		interval = atomic_read(&lis->enabled);
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EINVAL;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations lis3dh_misc_fops = {
	.owner = THIS_MODULE,
	.open = lis3dh_misc_open,
	.unlocked_ioctl = lis3dh_misc_ioctl,
};

static struct miscdevice lis3dh_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &lis3dh_misc_fops,
};

static void lis3dh_input_work_func(struct work_struct *work)
{
	struct lis3dh_data *lis = container_of((struct delayed_work *)work,
						  struct lis3dh_data,
						  input_work);
	int xyz[3] = { 0 };
	int err;

	mutex_lock(&lis->lock);
	err = lis3dh_get_acceleration_data(lis, xyz);
	if (err < 0)
		dev_err(&lis->client->dev, "get_acceleration_data failed\n");
	else
		lis3dh_report_values(lis, xyz);

	schedule_delayed_work(&lis->input_work,
			      msecs_to_jiffies(lis->pdata->poll_interval));
	mutex_unlock(&lis->lock);
}

#ifdef LIS3DH_OPEN_ENABLE
int lis3dh_input_open(struct input_dev *input)
{
	struct lis3dh_data *lis = input_get_drvdata(input);

	return lis3dh_enable(lis);
}

void lis3dh_input_close(struct input_dev *dev)
{
	struct lis3dh_data *lis = input_get_drvdata(dev);

	lis3dh_disable(lis);
}
#endif

static int lis3dh_validate_pdata(struct lis3dh_data *lis)
{
	/* Enforce minimum polling interval */
	if (lis->pdata->poll_interval < lis->pdata->min_interval) {
		dev_err(&lis->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis3dh_input_init(struct lis3dh_data *lis)
{
	int err;

	INIT_DELAYED_WORK(&lis->input_work, lis3dh_input_work_func);

	lis->input_dev = input_allocate_device();
	if (!lis->input_dev) {
		err = -ENOMEM;
		dev_err(&lis->client->dev, "input device allocate failed\n");
		goto err0;
	}

#ifdef LIS3DH_OPEN_ENABLE
	lis->input_dev->open = lis3dh_input_open;
	lis->input_dev->close = lis3dh_input_close;
#endif

	input_set_drvdata(lis->input_dev, lis);

	set_bit(EV_ABS, lis->input_dev->evbit);

	input_set_abs_params(lis->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(lis->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(lis->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	lis->input_dev->name = "accelerometer";

	err = input_register_device(lis->input_dev);
	if (err) {
		dev_err(&lis->client->dev,
			"unable to register input polled device %s\n",
			lis->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(lis->input_dev);
err0:
	return err;
}

static void lis3dh_input_cleanup(struct lis3dh_data *lis)
{
	input_unregister_device(lis->input_dev);
	input_free_device(lis->input_dev);
}

static int lis3dh_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lis3dh_data *lis;
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

	lis = kzalloc(sizeof(*lis), GFP_KERNEL);
	if (lis == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&lis->lock);
	mutex_lock(&lis->lock);
	lis->client = client;

	lis->pdata = kmalloc(sizeof(*lis->pdata), GFP_KERNEL);
	if (lis->pdata == NULL) {
		err = -ENOMEM;
		goto err1;
	}

	memcpy(lis->pdata, client->dev.platform_data, sizeof(*lis->pdata));

	err = lis3dh_validate_pdata(lis);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, lis);

	if (lis->pdata->init) {
		err = lis->pdata->init();
		if (err < 0)
			goto err1_1;
	}

	memset(lis->resume_state, 0, ARRAY_SIZE(lis->resume_state));

	lis->resume_state[0] = ODR_OFF | ENABLE_ALL_AXES;
	lis->resume_state[1] = 0;
	lis->resume_state[2] = 0;
	lis->resume_state[3] = 0;
	lis->resume_state[4] = 0;

	err = lis3dh_device_power_on(lis);
	if (err < 0)
		goto err2;

	atomic_set(&lis->enabled, 1);

	err = lis3dh_update_g_range(lis, lis->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto err2;
	}

	err = lis3dh_update_odr(lis, lis->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2;
	}

	err = lis3dh_input_init(lis);
	if (err < 0)
		goto err3;

	lis3dh_misc_data = lis;

	err = misc_register(&lis3dh_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "lisd_device register failed\n");
		goto err4;
	}

	lis3dh_device_power_off(lis);

	register_pm_notifier(&lis3dh_pm_notifier);

	/* As default, do not report information */
	atomic_set(&lis->enabled, 0);

	mutex_unlock(&lis->lock);

	dev_info(&client->dev, "lis3dh probed\n");

	return 0;

err4:
	lis3dh_input_cleanup(lis);
err3:
	lis3dh_device_power_off(lis);
err2:
	if (lis->pdata->exit)
		lis->pdata->exit();
err1_1:
	mutex_unlock(&lis->lock);
	kfree(lis->pdata);
err1:
	kfree(lis);
err0:
	return err;
}

static int __devexit lis3dh_remove(struct i2c_client *client)
{
	struct lis3dh_data *lis = i2c_get_clientdata(client);

	misc_deregister(&lis3dh_misc_device);
	lis3dh_input_cleanup(lis);
	lis3dh_device_power_off(lis);
	if (lis->pdata->exit)
		lis->pdata->exit();
	kfree(lis->pdata);
	kfree(lis);

	return 0;
}

static const struct i2c_device_id lis3dh_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lis3dh_id);

static int lis3dh_resume(void)
{
	if (lis3dh_misc_data->on_before_suspend)
		return lis3dh_enable(lis3dh_misc_data);
	return 0;
}

static int lis3dh_suspend(void)
{
	lis3dh_misc_data->on_before_suspend =
		atomic_read(&lis3dh_misc_data->enabled);
	return lis3dh_disable(lis3dh_misc_data);
}

static int lis3dh_pm_event(struct notifier_block *this, unsigned long event,
				void *ptr)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		lis3dh_suspend();
		break;
	case PM_POST_SUSPEND:
		lis3dh_resume();
	}
	return NOTIFY_DONE;
}

static struct notifier_block lis3dh_pm_notifier = {
	.notifier_call = lis3dh_pm_event,
};

static struct i2c_driver lis3dh_driver = {
	.driver = {
		   .name = NAME,
		   },
	.probe = lis3dh_probe,
	.remove = __devexit_p(lis3dh_remove),
	.id_table = lis3dh_id,
};

static int __init lis3dh_init(void)
{
	pr_info("LIS3DH accelerometer driver\n");
	return i2c_add_driver(&lis3dh_driver);
}

static void __exit lis3dh_exit(void)
{
	i2c_del_driver(&lis3dh_driver);
	return;
}

module_init(lis3dh_init);
module_exit(lis3dh_exit);

MODULE_DESCRIPTION("lis3dh accelerometer driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
