/*
 * Copyright (C) 2010-2011 Motorola, Inc.
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
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/input/bmp180.h>

#undef REGISTER_ACCESS_ENABLE

#undef BMP180_OPEN_ENABLE

/* Register definitions */
#define BMP180_TAKE_MEAS_REG		0xf4
#define BMP180_READ_MEAS_REG_U		0xf6
#define BMP180_READ_MEAS_REG_L		0xf7
#define BMP180_READ_MEAS_REG_XL		0xf8

/* Bytes defined by the spec to take measurements
Temperature measurement command */
#define BMP180_MEAS_TEMP		0x2e
/* Pressure measurement command */
#define BMP180_MEAS_PRESS		0x34
/* Measurement in progress flag */
#define BMP180_MEAS_BUSY		0x20

/* EEPROM registers each is a two byte value so there is
an upper byte and a lower byte */
#define BMP180_EEPROM_AC1_U	0xaa
#define BMP180_EEPROM_AC1_L	0xab
#define BMP180_EEPROM_AC2_U	0xac
#define BMP180_EEPROM_AC2_L	0xad
#define BMP180_EEPROM_AC3_U	0xae
#define BMP180_EEPROM_AC3_L	0xaf
#define BMP180_EEPROM_AC4_U	0xb0
#define BMP180_EEPROM_AC4_L	0xb1
#define BMP180_EEPROM_AC5_U	0xb2
#define BMP180_EEPROM_AC5_L	0xb3
#define BMP180_EEPROM_AC6_U	0xb4
#define BMP180_EEPROM_AC6_L	0xb5
#define BMP180_EEPROM_B1_U	0xb6
#define BMP180_EEPROM_B1_L	0xb7
#define BMP180_EEPROM_B2_U	0xb8
#define BMP180_EEPROM_B2_L	0xb9
#define BMP180_EEPROM_MB_U	0xba
#define BMP180_EEPROM_MB_L	0xbb
#define BMP180_EEPROM_MC_U	0xbc
#define BMP180_EEPROM_MC_L	0xbd
#define BMP180_EEPROM_MD_U	0xbe
#define BMP180_EEPROM_MD_L	0xbf

#ifdef REGISTER_ACCESS_ENABLE
struct bmp180_reg {
	const char *name;
	uint8_t reg;
} bmp180_regs[] = {
	{"MEASURE_REG", BMP180_TAKE_MEAS_REG},
	{"CNTRL_1", BMP180_READ_MEAS_REG_U},
	{"CNTRL_2", BMP180_READ_MEAS_REG_L},
	{"CNTRL_3", BMP180_READ_MEAS_REG_XL},
	{"EE_AC1_U", BMP180_EEPROM_AC1_U},
	{"EE_AC1_L", BMP180_EEPROM_AC1_L},
	{"EE_AC2_U", BMP180_EEPROM_AC2_U},
	{"EE_AC2_L", BMP180_EEPROM_AC2_L},
	{"EE_AC3_U", BMP180_EEPROM_AC3_U},
	{"EE_AC3_L", BMP180_EEPROM_AC3_L},
	{"EE_AC4_U", BMP180_EEPROM_AC4_U},
	{"EE_AC4_L", BMP180_EEPROM_AC4_L},
	{"EE_AC5_U", BMP180_EEPROM_AC5_U},
	{"EE_AC5_L", BMP180_EEPROM_AC5_L},
	{"EE_AC6_U", BMP180_EEPROM_AC6_U},
	{"EE_AC6_L", BMP180_EEPROM_AC6_L},
	{"EE_B1_U", BMP180_EEPROM_B1_U},
	{"EE_B1_L", BMP180_EEPROM_B1_L},
	{"EE_B2_U", BMP180_EEPROM_B2_U},
	{"EE_B2_L", BMP180_EEPROM_B2_L},
	{"EE_MB_U", BMP180_EEPROM_MB_U},
	{"EE_MB_L", BMP180_EEPROM_MB_L},
	{"EE_MC_U", BMP180_EEPROM_MC_U},
	{"EE_MC_L", BMP180_EEPROM_MC_L},
	{"EE_MD_U", BMP180_EEPROM_MD_U},
	{"EE_MD_L", BMP180_EEPROM_MD_L},
};
#endif /* REGISTER_ACCESS_ENABLE */

#define BMP180_DEBUG_NORMAL 1
#define BMP180_DEBUG_EXTRA 2

#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
#define AUTO_INCREMENT		0x80

#define MUTEX_RETRY_DELAY	5

struct bmp180_eeprom_data {
	s16 AC1, AC2, AC3;
	u16 AC4, AC5, AC6;
	s16 B1, B2;
	s16 MB, MC, MD;
};

struct bmp180_data {
	struct i2c_client *client;
	struct bmp180_platform_data *pdata;
	struct mutex lock;    /* Protect the driver state */

	struct workqueue_struct *barom_wq;
	struct delayed_work start_temperature_work;
	struct delayed_work start_pressure_work;
	struct delayed_work report_pressure_work;

	unsigned long temperature_delay;
	unsigned long pressure_delay;
	unsigned long polling_delay;

	struct input_dev *input_dev;
	struct notifier_block pm_notifier;

	u8 oversampling_rate;

	int uncalib_temperature;
	int uncalib_pressure;
	int calib_temperature;
	long calib_pressure;
	long b5;		/* Needed for pressure calculation */

	struct bmp180_eeprom_data bmp180_eeprom_vals;

	int enabled;
	int on_before_suspend;
	struct regulator *regulator;
	u8 resume_state[5];
};

/*
 * Because this is global this limits the driver to a single instance.
 */
static struct bmp180_data *bmp180_misc_data;

static uint32_t bmp180_debug;
module_param_named(baro_debug, bmp180_debug, uint, 0644);

static int bmp180_set_param_enable(const char *val, struct kernel_param *kp);
static int bmp180_get_param_enable(char *buffer, struct kernel_param *kp);

static int bmp180_enable_param;
module_param_call(enable, bmp180_set_param_enable, bmp180_get_param_enable,
	&bmp180_enable_param, 0644);
MODULE_PARM_DESC(enable, "Enable/disable the barometer.");

static int bmp180_set_param_poll_interval(const char *val,
	struct kernel_param *kp);
static int bmp180_get_param_poll_interval(char *buffer,
	struct kernel_param *kp);

static int bmp180_poll_interval_param;
module_param_call(poll_interval, bmp180_set_param_poll_interval,
	bmp180_get_param_poll_interval, &bmp180_poll_interval_param, 0644);
MODULE_PARM_DESC(poll_interval, "Set the barometer's sample rate.");

static int bmp180_set_param_accuracy(const char *val, struct kernel_param *kp);
static int bmp180_get_param_accuracy(char *buffer, struct kernel_param *kp);

static int bmp180_accuracy_param;
module_param_call(accuracy, bmp180_set_param_accuracy,
	bmp180_get_param_accuracy, &bmp180_accuracy_param, 0644);
MODULE_PARM_DESC(accuracy, "Set the barometer's accuracy.");

static int bmp180_i2c_read(struct bmp180_data *barom, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = barom->client->addr,
		 .flags = barom->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = barom->client->addr,
		 .flags = (barom->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(barom->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		pr_err("%s:read transfer error\n", __func__);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int bmp180_i2c_write(struct bmp180_data *barom, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = barom->client->addr,
		 .flags = barom->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(barom->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		pr_err("%s:write transfer error\n", __func__);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int bmp180_update_measurement_accuracy(struct bmp180_data *barom,
				       int accuracy)
{
	if ((accuracy >= 0) && (accuracy <= 3))
		barom->oversampling_rate = accuracy;

	return 0;
}

static int bmp180_enable(struct bmp180_data *barom)
{
	int err = 0;

	if (barom->enabled == 0) {
		if (barom->regulator) {
			err = regulator_enable(barom->regulator);
			if (err < 0)
				return err;
		}
		barom->enabled = 1;
		queue_delayed_work(barom->barom_wq,
			&barom->start_temperature_work, 0);

		pr_info("%s:Barometer enabled\n", __func__);
	}

	return 0;
}

static int bmp180_disable(struct bmp180_data *barom)
{
	if (barom->enabled == 1) {
		barom->enabled = 0;
		cancel_delayed_work_sync(&barom->start_temperature_work);
		cancel_delayed_work_sync(&barom->start_pressure_work);
		cancel_delayed_work_sync(&barom->report_pressure_work);
		flush_workqueue(barom->barom_wq);
		if (barom->regulator)
			regulator_disable(barom->regulator);
	}

	pr_info("%s:Barometer disabled\n", __func__);

	return 0;
}

static int bmp180_set_param_enable(const char *char_value,
	struct kernel_param *kp)
{
	unsigned long int_value;
	int  ret;

	if (!char_value)
		return -EINVAL;

	if (!bmp180_misc_data)
		return -EINVAL;

	ret = strict_strtoul(char_value, (unsigned int)0, &int_value);
	if (ret)
		return ret;

	*((int *)kp->arg) = int_value;

	mutex_lock(&bmp180_misc_data->lock);

	if (int_value)
		ret = bmp180_enable(bmp180_misc_data);
	else
		ret = bmp180_disable(bmp180_misc_data);

	mutex_unlock(&bmp180_misc_data->lock);

	return ret;
}

static int bmp180_get_param_enable(char *buffer, struct kernel_param *kp)
{
	int num_chars;

	if (!buffer)
		return -EINVAL;

	if (!bmp180_misc_data) {
		scnprintf(buffer, PAGE_SIZE, "0\n");
		return 1;
	}

	mutex_lock(&bmp180_misc_data->lock);

	num_chars = scnprintf(buffer, 2, "%d\n",
		bmp180_misc_data->enabled);

	mutex_unlock(&bmp180_misc_data->lock);

	return num_chars;
}

#ifdef BMP180_OPEN_ENABLE
int bmp180_input_open(struct input_dev *input)
{
	struct bmp180_data *barom = input_get_drvdata(input);
	int ret;

	mutex_lock(&barom->lock);

	ret = bmp180_enable(barom);

	mutex_unlock(&barom->lock);

	return ret;
}

void bmp180_input_close(struct input_dev *dev)
{
	struct bmp180_data *barom = input_get_drvdata(dev);

	mutex_lock(&barom->lock);

	bmp180_disable(barom);

	mutex_unlock(&barom->lock);
}
#endif /* BMP180_OPEN_ENABLE */

static int bmp180_set_param_poll_interval(const char *char_value,
	struct kernel_param *kp)
{
	unsigned long int_value;
	int  ret;

	if (!char_value)
		return -EINVAL;

	if (!bmp180_misc_data)
		return -EINVAL;

	ret = strict_strtoul(char_value, (unsigned int)0, &int_value);
	if (ret)
		return ret;

	*((int *)kp->arg) = int_value;

	mutex_lock(&bmp180_misc_data->lock);

	bmp180_misc_data->pdata->poll_interval =
		max((int)int_value, bmp180_misc_data->pdata->min_interval);

	bmp180_misc_data->polling_delay = bmp180_misc_data->pdata->poll_interval
		- bmp180_misc_data->temperature_delay
		- bmp180_misc_data->pressure_delay ;

	mutex_unlock(&bmp180_misc_data->lock);

	return ret;
}

static int bmp180_get_param_poll_interval(char *buffer,
	struct kernel_param *kp)
{
	int num_chars;

	if (!buffer)
		return -EINVAL;

	if (!bmp180_misc_data) {
		scnprintf(buffer, PAGE_SIZE, "0\n");
		return 1;
	}

	mutex_lock(&bmp180_misc_data->lock);

	num_chars = scnprintf(buffer, PAGE_SIZE, "%d\n",
		bmp180_misc_data->pdata->poll_interval);

	mutex_unlock(&bmp180_misc_data->lock);

	return num_chars;
}

static int bmp180_set_param_accuracy(const char *char_value,
	struct kernel_param *kp)
{
	unsigned long int_value;
	int  ret;

	if (!char_value)
		return -EINVAL;

	if (!bmp180_misc_data)
		return -EINVAL;

	ret = strict_strtoul(char_value, (unsigned int)0, &int_value);
	if (ret)
		return ret;

	*((int *)kp->arg) = int_value;

	mutex_lock(&bmp180_misc_data->lock);

	bmp180_update_measurement_accuracy(bmp180_misc_data, int_value);

	mutex_unlock(&bmp180_misc_data->lock);

	return ret;
	}

static int bmp180_get_param_accuracy(char *buffer, struct kernel_param *kp)
{
	int num_chars;

	if (!buffer)
		return -EINVAL;

	if (!bmp180_misc_data) {
		scnprintf(buffer, PAGE_SIZE, "0\n");
		return 1;
	}

	mutex_lock(&bmp180_misc_data->lock);

	num_chars = scnprintf(buffer, PAGE_SIZE, "%d\n",
		bmp180_misc_data->oversampling_rate);

	mutex_unlock(&bmp180_misc_data->lock);

	return num_chars;
}

#ifdef REGISTER_ACCESS_ENABLE
static ssize_t bmp180_registers_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client,
						 dev);
	struct bmp180_data *barom = i2c_get_clientdata(client);
	u8 barom_reg[2];
	unsigned i, n, reg_count;

	mutex_lock(&barom->lock);

	reg_count = sizeof(bmp180_regs) / sizeof(bmp180_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		barom_reg[0] = (AUTO_INCREMENT | bmp180_regs[i].reg);
		bmp180_i2c_read(barom, barom_reg, 1);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       bmp180_regs[i].name, barom_reg[0]);
	}

	mutex_unlock(&barom->lock);

	return n;
}

static ssize_t bmp180_registers_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client,
						 dev);
	struct bmp180_data *barom = i2c_get_clientdata(client);
	unsigned i, reg_count, value;
	int error;
	u8 barom_reg[2];
	char name[30];

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -EINVAL;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -EINVAL;
	}

	reg_count = sizeof(bmp180_regs) / sizeof(bmp180_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, bmp180_regs[i].name)) {
			mutex_lock(&barom->lock);

			barom_reg[0] = (AUTO_INCREMENT | bmp180_regs[i].reg);
			barom_reg[1] = value;
			error = bmp180_i2c_write(barom, barom_reg, 2);

			mutex_unlock(&barom->lock);

			if (error) {
				pr_err("%s:Failed to write register %s\n",
				       __func__, name);
				return -EINVAL;
			}
			return count;
		}
	}

	pr_err("%s:no such register %s\n", __func__, name);

	return -EINVAL;
}

static DEVICE_ATTR(registers, 0644, bmp180_registers_show,
		   bmp180_registers_store);
#endif /* REGISTER_ACCESS_ENABLE */

static int bmp180_get_temperature_data(struct bmp180_data *barom)
{
	int err = -1;
	u8 buf[2] = { BMP180_READ_MEAS_REG_U, 0 };
	int x1;
	unsigned int x2;

	err = bmp180_i2c_read(barom, buf, 2);
	if (err) {
		pr_err("%s:Cannot read pressure measurement\n", __func__);
		return err;
	}
	if (bmp180_debug & BMP180_DEBUG_EXTRA)
		pr_info("%s:Read Temp 0x%X 0x%X\n", __func__, buf[0], buf[1]);

	barom->uncalib_temperature = (buf[0] << 8) + buf[1];

	/* The math is derived from the data sheet. */
	x1 = ((barom->uncalib_temperature - barom->bmp180_eeprom_vals.AC6) *
	      barom->bmp180_eeprom_vals.AC5) >> 15;
	x2 = (barom->bmp180_eeprom_vals.MC << 11) /
	    (x1 + barom->bmp180_eeprom_vals.MD);
	barom->b5 = x1 + x2;
	barom->calib_temperature = (barom->b5 + 8) >> 4;
	if (bmp180_debug & BMP180_DEBUG_NORMAL)
		pr_info("%s:Calibrated Temp %d\n",
		__func__, barom->calib_temperature);

	return err;
}

static int bmp180_get_barometer_data(struct bmp180_data *barom)
{
	int err = -1;
	long x1, x2, x3, b3, b6;
	unsigned long b4, b7;
	long p;
	u8 buf[3] = { BMP180_READ_MEAS_REG_U, 0, 0 };

	err = bmp180_i2c_read(barom, buf, 3);
	if (err) {
		pr_err("%s:Cannot read pressure measurement\n", __func__);
		return err;
	}

	/* Raw data to uncalibrate pressure.  Conversion compliments of the
	data sheet */
	barom->uncalib_pressure = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) >>
		(8 - barom->oversampling_rate);
	if (bmp180_debug & BMP180_DEBUG_EXTRA)
		pr_info("%s:Uncalibrated pressure %d\n", __func__,
		       barom->uncalib_pressure);

	/* Complicated math compliments of the data sheet */
	b6 = (barom->b5 - 4000);
	x1 = (barom->bmp180_eeprom_vals.B2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (barom->bmp180_eeprom_vals.AC2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((long)barom->bmp180_eeprom_vals.AC1) * 4 +
		x3) << barom->oversampling_rate) + 2) >> 2;
	x1 = (barom->bmp180_eeprom_vals.AC3 * b6) >> 13;
	x2 = (barom->bmp180_eeprom_vals.B1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (barom->bmp180_eeprom_vals.AC4 *
	      (unsigned long)(x3 + 32768)) >> 15;
	b7 = ((unsigned long)barom->uncalib_pressure -
	      b3) * (50000 >> barom->oversampling_rate);
	if (b7 < 0x80000000)
		p = (b7 * 2) / b4;
	else
		p = (b7 / b4) * 2;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	barom->calib_pressure = p + ((x1 + x2 + 3791) >> 4);
	if (bmp180_debug & BMP180_DEBUG_NORMAL)
		pr_info("%s:Calibrated Pressure is %li\n",
		__func__, barom->calib_pressure);

	return err;
}

static void bmp180_start_temperature_work_func(struct work_struct *work)
{
	int err;
	u8 buf[2];

	if (mutex_trylock(&bmp180_misc_data->lock) == 0) {
		queue_delayed_work(bmp180_misc_data->barom_wq,
			&bmp180_misc_data->start_temperature_work,
			msecs_to_jiffies(MUTEX_RETRY_DELAY));
		return;
	}

	if (bmp180_misc_data->enabled == 0) {
		mutex_unlock(&bmp180_misc_data->lock);
		return;
	}

	buf[0] = (AUTO_INCREMENT | BMP180_TAKE_MEAS_REG);
	buf[1] = BMP180_MEAS_TEMP;

	err = bmp180_i2c_write(bmp180_misc_data, buf, 2);
	if (err) {
		pr_err("%s:Cannot start temp measurement\n", __func__);
		goto error;
	}

	queue_delayed_work(bmp180_misc_data->barom_wq,
		&bmp180_misc_data->start_pressure_work,
		msecs_to_jiffies(bmp180_misc_data->temperature_delay));

	mutex_unlock(&bmp180_misc_data->lock);

	return;

error:
	queue_delayed_work(bmp180_misc_data->barom_wq,
		&bmp180_misc_data->start_temperature_work,
		msecs_to_jiffies(bmp180_misc_data->polling_delay));

	mutex_unlock(&bmp180_misc_data->lock);

	return;
}

static void bmp180_start_pressure_work_func(struct work_struct *work)
{
	int err = 0;
	u8 buf[2];

	if (mutex_trylock(&bmp180_misc_data->lock) == 0) {
		queue_delayed_work(bmp180_misc_data->barom_wq,
			&bmp180_misc_data->start_pressure_work,
			msecs_to_jiffies(MUTEX_RETRY_DELAY));
		return;
	}

	if (bmp180_misc_data->enabled == 0) {
		mutex_unlock(&bmp180_misc_data->lock);
		return;
	}

	if (bmp180_debug & BMP180_DEBUG_NORMAL)
		pr_info("%s:Temp cycle\n", __func__);

	err = bmp180_get_temperature_data(bmp180_misc_data);
	if (err) {
		pr_err("%s:Cannot read temp measurement\n", __func__);
		goto error;
	}

	/* Setup for a pressure measurement */
	buf[0] = (AUTO_INCREMENT | BMP180_TAKE_MEAS_REG);
	buf[1] = BMP180_MEAS_PRESS |
		(bmp180_misc_data->oversampling_rate << 6);

	err = bmp180_i2c_write(bmp180_misc_data, buf, 2);
	if (err) {
		pr_err("%s:Cannot start pressure measurement\n",
			__func__);
		goto error;
	}

	queue_delayed_work(bmp180_misc_data->barom_wq,
		&bmp180_misc_data->report_pressure_work,
		msecs_to_jiffies(bmp180_misc_data->pressure_delay));

	mutex_unlock(&bmp180_misc_data->lock);

	return;

error:
	queue_delayed_work(bmp180_misc_data->barom_wq,
		&bmp180_misc_data->start_temperature_work,
		msecs_to_jiffies(bmp180_misc_data->polling_delay));

	mutex_unlock(&bmp180_misc_data->lock);

	return;
}

static void bmp180_report_pressure_work_func(struct work_struct *work)
{
	int err = 0;

	if (mutex_trylock(&bmp180_misc_data->lock) == 0) {
		queue_delayed_work(bmp180_misc_data->barom_wq,
			&bmp180_misc_data->report_pressure_work,
			msecs_to_jiffies(MUTEX_RETRY_DELAY));
		return;
	}

	if (bmp180_misc_data->enabled == 0) {
		mutex_unlock(&bmp180_misc_data->lock);
		return;
	}

	/* Get and report the pressure */
	if (bmp180_debug & BMP180_DEBUG_NORMAL)
		pr_info("%s:Pressure cycle\n", __func__);

	err = bmp180_get_barometer_data(bmp180_misc_data);
	if (err) {
		pr_err("%s:Pressure measurement failed\n", __func__);
		goto error;
	}

	input_report_abs(bmp180_misc_data->input_dev, ABS_PRESSURE,
			 bmp180_misc_data->calib_pressure);
	input_sync(bmp180_misc_data->input_dev);

error:
	queue_delayed_work(bmp180_misc_data->barom_wq,
		&bmp180_misc_data->start_temperature_work,
		msecs_to_jiffies(bmp180_misc_data->polling_delay));

	mutex_unlock(&bmp180_misc_data->lock);

	return;
}

static int bmp180_validate_pdata(struct bmp180_data *barom)
{
	barom->pdata->poll_interval = max(barom->pdata->poll_interval,
					  barom->pdata->min_interval);

	return 0;
}

static int bmp180_input_init(struct bmp180_data *barom)
{
	int err;

	barom->input_dev = input_allocate_device();
	if (!barom->input_dev) {
		err = -ENOMEM;
		pr_err("%s:input device allocate failed\n", __func__);
		goto err0;
	}

#ifdef BMP180_OPEN_ENABLE
	barom->input_dev->open = bmp180_input_open;
	barom->input_dev->close = bmp180_input_close;
#endif /* BMP180_OPEN_ENABLE */

	input_set_drvdata(barom->input_dev, barom);

	set_bit(EV_ABS, barom->input_dev->evbit);

	/* Need to define the correct min and max */
	input_set_abs_params(barom->input_dev, ABS_PRESSURE,
				barom->pdata->min_p, barom->pdata->max_p,
				barom->pdata->fuzz, barom->pdata->flat);

	barom->input_dev->name = "barometer";

	err = input_register_device(barom->input_dev);
	if (err) {
		pr_err("%s:unable to register input polled device %s\n",
			__func__, barom->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(barom->input_dev);
err0:
	return err;
}

static void bmp180_input_cleanup(struct bmp180_data *barom)
{
	input_unregister_device(barom->input_dev);
}

static int bmp180_read_store_eeprom_val(struct bmp180_data *barom)
{
	int err = 0;
	u8 buf[22];

	buf[0] = BMP180_EEPROM_AC1_U;
	err = bmp180_i2c_read(barom, buf, 22);
	if (err) {
		pr_err("%s:Cannot read EEPROM values\n", __func__);
		return err;
	}

	barom->bmp180_eeprom_vals.AC1 = (buf[0] << 8) | buf[1];
	barom->bmp180_eeprom_vals.AC2 = (buf[2] << 8) | buf[3];
	barom->bmp180_eeprom_vals.AC3 = (buf[4] << 8) | buf[5];
	barom->bmp180_eeprom_vals.AC4 = (buf[6] << 8) | buf[7];
	barom->bmp180_eeprom_vals.AC5 = (buf[8] << 8) | buf[9];
	barom->bmp180_eeprom_vals.AC6 = (buf[10] << 8) | buf[11];
	barom->bmp180_eeprom_vals.B1 = (buf[12] << 8) | buf[13];
	barom->bmp180_eeprom_vals.B2 = (buf[14] << 8) | buf[15];
	barom->bmp180_eeprom_vals.MB = (buf[16] << 8) | buf[17];
	barom->bmp180_eeprom_vals.MC = (buf[18] << 8) | buf[19];
	barom->bmp180_eeprom_vals.MD = (buf[20] << 8) | buf[21];

	return 0;
}

static int bmp180_suspend(struct bmp180_data *barom)
{
	int ret = 0;

	barom->on_before_suspend = barom->enabled;
	if (barom->enabled)
		ret = bmp180_disable(barom);

	if (bmp180_debug & BMP180_DEBUG_NORMAL)
		pr_info("%s:Barometer suspended\n", __func__);

	return ret;
}

static int bmp180_resume(struct bmp180_data *barom)
{
	int ret = 0;

	if (barom->on_before_suspend)
		ret = bmp180_enable(barom);

	if (bmp180_debug & BMP180_DEBUG_NORMAL)
		pr_info("%s:Barometer resumed\n", __func__);

	return ret;
}

static int bmp180_pm_event(struct notifier_block *this, unsigned long event,
	void *ptr)
{
	struct bmp180_data *barom = container_of(this,
		struct bmp180_data, pm_notifier);

	mutex_lock(&barom->lock);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		bmp180_suspend(barom);
		break;
	case PM_POST_SUSPEND:
		bmp180_resume(barom);
		break;
	}

	mutex_unlock(&barom->lock);

	return NOTIFY_DONE;
}

static int bmp180_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bmp180_data *barom;
	int err = -1;

	if (client->dev.platform_data == NULL) {
		pr_err("%s:platform data is NULL. exiting.\n", __func__);
		err = -ENODEV;
		goto err_pdata;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s:client not i2c capable\n", __func__);
		err = -ENODEV;
		goto err_i2c;
	}

	barom = kzalloc(sizeof(*barom), GFP_KERNEL);
	if (barom == NULL) {
		pr_err("%s:failed to allocate memory for module data\n",
		       __func__);
		err = -ENOMEM;
		goto err_kzalloc_barom;
	}
	bmp180_misc_data = barom;

	barom->pdata = kzalloc(sizeof(*barom->pdata), GFP_KERNEL);
	if (barom->pdata == NULL) {
		pr_err("%s:failed to allocate memory for module pdata\n",
		       __func__);
		err = -ENOMEM;
		goto err_kzalloc_pdata;
	}

	memcpy(barom->pdata, client->dev.platform_data, sizeof(*barom->pdata));

	err = bmp180_validate_pdata(barom);
	if (err < 0) {
		pr_err("%s:failed to validate platform data\n", __func__);
		goto err_validate_pdata;
	}

	mutex_init(&barom->lock);

	barom->client = client;
	barom->oversampling_rate = 0;
	barom->temperature_delay = barom->pdata->temperature_delay;
	barom->pressure_delay = barom->pdata->pressure_delay;
	barom->polling_delay = barom->pdata->poll_interval
		- barom->temperature_delay
		- barom->pressure_delay ;
	barom->b5 = 0;
	/* As default, do not report information */
	barom->enabled = 0;

	i2c_set_clientdata(client, barom);

	err = bmp180_read_store_eeprom_val(barom);
	if (err) {
		pr_err("%s: Reading the EEPROM failed\n", __func__);
		err = -ENODEV;
		goto err_read_eeprom;
	}

	barom->barom_wq = create_singlethread_workqueue("barometer_wq");
	if (!barom->barom_wq) {
		pr_err("%s: Cannot create work queue\n", __func__);
		err = -ENOMEM;
		goto err_create_wq;
	}

	INIT_DELAYED_WORK(&barom->start_temperature_work,
		bmp180_start_temperature_work_func);
	INIT_DELAYED_WORK(&barom->start_pressure_work,
		bmp180_start_pressure_work_func);
	INIT_DELAYED_WORK(&barom->report_pressure_work,
		bmp180_report_pressure_work_func);

	barom->regulator = regulator_get(&client->dev,
		barom->pdata->regulator_name);
	if (IS_ERR_OR_NULL(barom->regulator)) {
		pr_warn("%s:unable to get regulator\n", __func__);
		barom->regulator = NULL;
	}

	err = bmp180_input_init(barom);
	if (err < 0) {
		pr_err("%s:input init failed\n", __func__);
		goto err_input_init;
	}

	barom->pm_notifier.notifier_call = bmp180_pm_event;
	err = register_pm_notifier(&barom->pm_notifier);
	if (err < 0) {
		pr_err("%s:Register_pm_notifier failed: %d\n", __func__, err);
		goto err_register_pm_notifier;
	}

#ifdef REGISTER_ACCESS_ENABLE
	err = device_create_file(&client->dev, &dev_attr_registers);
	if (err < 0)
		pr_info("%s:File device creation failed: %d\n", __func__, err);
#endif /* REGISTER_ACCESS_ENABLE */

	pr_info("%s:Probe completed\n", __func__);

	return 0;

err_register_pm_notifier:
	bmp180_input_cleanup(barom);
err_input_init:
	if (barom->regulator)
		regulator_put(barom->regulator);
	destroy_workqueue(barom->barom_wq);
err_create_wq:
err_read_eeprom:
	mutex_destroy(&barom->lock);
err_validate_pdata:
	bmp180_misc_data = NULL;
	kfree(barom->pdata);
err_kzalloc_pdata:
	kfree(barom);
err_kzalloc_barom:
err_i2c:
err_pdata:
	return err;
}

static int __devexit bmp180_remove(struct i2c_client *client)
{
	/* TO DO: revisit ordering here once _probe order is finalized */
	struct bmp180_data *barom = i2c_get_clientdata(client);

	bmp180_disable(barom);
	unregister_pm_notifier(&barom->pm_notifier);
#ifdef REGISTER_ACCESS_ENABLE
	device_remove_file(&client->dev, &dev_attr_registers);
#endif /* REGISTER_ACCESS_ENABLE */
	bmp180_input_cleanup(barom);
	if (barom->regulator)
		regulator_put(barom->regulator);
	destroy_workqueue(barom->barom_wq);
	mutex_destroy(&barom->lock);
	bmp180_misc_data = NULL;
	kfree(barom->pdata);
	kfree(barom);

	return 0;
}

static const struct i2c_device_id bmp180_id[] = {
	{BMP180_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, bmp180_id);

static struct i2c_driver bmp180_driver = {
	.driver = {
		   .name = BMP180_NAME,
		   },
	.probe = bmp180_probe,
	.remove = __devexit_p(bmp180_remove),
	.id_table = bmp180_id,
};

static int __init bmp180_init(void)
{
	pr_info("BMP180 barometer driver\n");
	return i2c_add_driver(&bmp180_driver);
}

static void __exit bmp180_exit(void)
{
	i2c_del_driver(&bmp180_driver);
	return;
}

module_init(bmp180_init);
module_exit(bmp180_exit);

MODULE_DESCRIPTION("BMP180 barometer driver");
MODULE_AUTHOR("Motorola Mobility");
MODULE_LICENSE("GPL");
