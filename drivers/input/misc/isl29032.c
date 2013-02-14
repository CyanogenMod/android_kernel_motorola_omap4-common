/*
 * Copyright (C) 2011 Motorola Mobility, Inc.
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

#include <linux/isl29032.h>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define LD_ISL29032_ALLOWED_R_BYTES 1
#define LD_ISL29032_ALLOWED_W_BYTES 2
#define LD_ISL29032_MAX_RW_RETRIES 5
#define LD_ISL29032_I2C_RETRY_DELAY 10

#define ISL29032_CHIPID         0x00
#define ISL29032_CONFIGURE      0x01
#define ISL29032_INTERRUPT      0x02
#define ISL29032_PROX_LT        0x03
#define ISL29032_PROX_HT        0x04
#define ISL29032_ALSIR_TH1      0x05
#define ISL29032_ALSIR_TH2      0x06
#define ISL29032_ALSIR_TH3      0x07
#define ISL29032_PROX_DATA      0x08
#define ISL29032_ALSIR_DT1      0x09
#define ISL29032_ALSIR_DT2      0x0a
#define ISL29032_TEST1          0x0e
#define ISL29032_TEST2          0x0f

#define ISL29032_HIGH_LUX_RANGE 2000
#define ISL29032_LOW_LUX_RANGE  125

/* Thresholds for switching between lux ranges. */
#define ISL29032_LOW_TO_HIGH_COUNTS     800     /* ~24 lux */
#define ISL29032_HIGH_TO_LOW_COUNTS     32      /* ~16 lux */

#define ISL29032_ALS_FLAG_MASK          0x08
#define ISL29032_PROX_FLAG_MASK         0x80

#define ISL29032_CNF_ALS_RANGE_MASK     0x02
#define ISL29032_CNF_ALS_EN_MASK        0x04
#define ISL29032_CNF_PROX_EN_MASK       0x80
/* unit = millimeter */
#define PROXIMITY_NEAR  30              /* prox close threshold is 22-70mm */
#define PROXIMITY_FAR   1000            /* 1 meter */

struct isl29032_data {
	struct input_dev *dev;
	struct i2c_client *client;
	struct regulator *regulator;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct isl29032_platform_data *pdata;
	struct miscdevice miscdevice;
	struct notifier_block pm_notifier;
	struct mutex mutex; /* Used to prevent multiple access to the part */
	unsigned int suspended;
	unsigned int hw_initialized;
	unsigned int prox_enabled;
	unsigned int als_enabled;
	unsigned int prox_near;
	unsigned int last_prox_near;
	unsigned int lux_level;
	unsigned int prox_lt;
	unsigned int prox_ht;
};

static struct isl29032_data *isl29032_misc_data;

static struct isl29032_reg {
	const char *name;
	u8 reg;
} isl29032_regs[] = {
	{ "CHIP_ID",        ISL29032_CHIPID },
	{ "CONFIGURE",      ISL29032_CONFIGURE },
	{ "INTERRUPT",      ISL29032_INTERRUPT },
	{ "PROX_LT",        ISL29032_PROX_LT },
	{ "PROX_HT",        ISL29032_PROX_HT },
	{ "ALS_IR_TH1",     ISL29032_ALSIR_TH1 },
	{ "ALS_IR_TH2",     ISL29032_ALSIR_TH2 },
	{ "ALS_IR_TH3",     ISL29032_ALSIR_TH3 },
	{ "PROX_DATA",      ISL29032_PROX_DATA },
	{ "ALS_IR_DT1",     ISL29032_ALSIR_DT1 },
	{ "ALS_IR_DT2",     ISL29032_ALSIR_DT2 },
	{ "ENABLE",         ISL29032_TEST1 },
	{ "DISABLE",        ISL29032_TEST2 },
};

#define ISL29032_DBG_INPUT              0x00000001
#define ISL29032_DBG_POWER_ON_OFF       0x00000002
#define ISL29032_DBG_ENABLE_DISABLE     0x00000004
#define ISL29032_DBG_IOCTL              0x00000008
#define ISL29032_DBG_SUSPEND_RESUME     0x00000010
static u32 isl29032_debug;

module_param_named(als_debug, isl29032_debug, uint, 0644);

static int isl29032_read_reg(struct isl29032_data *isl,
			     u8 reg,
			     u8 *value)
{
	int error = 0;
	int i = 0;
	u8 dest_buffer;

	do {
		dest_buffer = reg;
		error = i2c_master_send(isl->client, &dest_buffer, 1);
		if (error == 1) {
			error = i2c_master_recv(isl->client,
				&dest_buffer, LD_ISL29032_ALLOWED_R_BYTES);
		}
		if (error != LD_ISL29032_ALLOWED_R_BYTES) {
			pr_err("%s: read[%i] failed: %d\n", __func__, i, error);
			msleep_interruptible(LD_ISL29032_I2C_RETRY_DELAY);
		}
	} while ((error != LD_ISL29032_ALLOWED_R_BYTES) &&
			((++i) < LD_ISL29032_MAX_RW_RETRIES));

	if (error == LD_ISL29032_ALLOWED_R_BYTES) {
		error = 0;
		if (value)
			*value = dest_buffer;
	}

	return error;
}

static int isl29032_write_reg(struct isl29032_data *isl,
			      u8 reg,
			      u8 value)
{
	u8 buf[LD_ISL29032_ALLOWED_W_BYTES] = { reg, value };
	int bytes;
	int i = 0;

	do {
		bytes = i2c_master_send(isl->client, buf,
			LD_ISL29032_ALLOWED_W_BYTES);

		if (bytes != LD_ISL29032_ALLOWED_W_BYTES) {
			pr_err("%s: write %d failed: %d\n", __func__, i, bytes);
			msleep_interruptible(LD_ISL29032_I2C_RETRY_DELAY);
		}
	} while ((bytes != (LD_ISL29032_ALLOWED_W_BYTES))
		&& ((++i) < LD_ISL29032_MAX_RW_RETRIES));

	if (bytes != LD_ISL29032_ALLOWED_W_BYTES) {
		pr_err("%s: i2c_master_send error\n", __func__);
		return -EIO;
	}

	return 0;
}

static int isl29032_set_bit(struct isl29032_data *isl,
				   u8 reg,
				   u8 bit_mask,
				   u8 value)
{
	int error;
	u8 reg_val;

	error = isl29032_read_reg(isl, reg, &reg_val);
	if (error != 0) {
		pr_err("%s:Unable to read register 0x%x: %d\n",
			__func__, reg, error);
		return -EFAULT;
	}

	if (value)
		reg_val |= (u8)bit_mask;
	else
		reg_val &= (u8)~bit_mask;

	error = isl29032_write_reg(isl, reg, reg_val);
	if (error != 0) {
		pr_err("%s:Unable to write register 0x%x: %d\n",
			__func__, reg, error);
		return -EFAULT;
	}

	return 0;
}

static int isl29032_set_als_enable(struct isl29032_data *isl,
				   unsigned int value)
{
	return isl29032_set_bit(isl, ISL29032_CONFIGURE,
		ISL29032_CNF_ALS_EN_MASK, value);
}

static int isl29032_set_als_range(struct isl29032_data *isl,
				  unsigned int value)
{
	return isl29032_set_bit(isl, ISL29032_CONFIGURE,
		ISL29032_CNF_ALS_RANGE_MASK, value);
}

static int isl29032_clear_als_flag(struct isl29032_data *isl)
{
	return isl29032_set_bit(isl, ISL29032_INTERRUPT,
		ISL29032_ALS_FLAG_MASK, 0);
}

static int isl29032_clear_prox_flag(struct isl29032_data *isl)
{
	return isl29032_set_bit(isl, ISL29032_INTERRUPT,
		ISL29032_PROX_FLAG_MASK, 0);
}

static int isl29032_clear_prox_and_als_flags(struct isl29032_data *isl)
{
	return isl29032_set_bit(isl, ISL29032_INTERRUPT,
		ISL29032_ALS_FLAG_MASK | ISL29032_PROX_FLAG_MASK, 0);
}

static int isl29032_set_prox_enable(struct isl29032_data *isl,
				    unsigned int value)
{
	int error;
	if (isl29032_debug & ISL29032_DBG_ENABLE_DISABLE)
		pr_info("%s: bit = %d prox_near = %d\n",
			__func__, value, isl->prox_near);

	error = isl29032_set_bit(isl, ISL29032_CONFIGURE,
		ISL29032_CNF_PROX_EN_MASK,
		value);

	if (!error && !value && isl->prox_near)
		isl->prox_near = 0;

	return error;
}

static int isl29032_init_registers(struct isl29032_data *isl)
{
	/* as per intersil recommendations */
	if (isl29032_write_reg(isl, ISL29032_CONFIGURE, 0) ||
		isl29032_write_reg(isl, ISL29032_TEST2, 0x29) ||
		isl29032_write_reg(isl, ISL29032_TEST1, 0) ||
		isl29032_write_reg(isl, ISL29032_TEST2, 0)) {

		pr_err("%s:Register initialization failed\n", __func__);
		return -EIO;
	}
	msleep(2);

	if (isl29032_write_reg(isl, ISL29032_CONFIGURE,
			isl->pdata->configure) ||
		isl29032_write_reg(isl, ISL29032_INTERRUPT,
			isl->pdata->interrupt_cntrl) ||
		isl29032_write_reg(isl, ISL29032_PROX_LT,
			isl->pdata->prox_lower_threshold) ||
		isl29032_write_reg(isl, ISL29032_PROX_HT,
			isl->pdata->prox_higher_threshold) ||
		isl29032_write_reg(isl, ISL29032_ALSIR_TH1, 0xFF) ||
		isl29032_write_reg(isl, ISL29032_ALSIR_TH2, 0xFF) ||
		isl29032_write_reg(isl, ISL29032_ALSIR_TH3, 0xFF)) {
		pr_err("%s:Register initialization failed\n", __func__);
		return -EIO;
	}
	isl->lux_level = ISL29032_HIGH_LUX_RANGE;
	return 0;
}

static int isl29032_read_adj_als(struct isl29032_data *isl,
				 unsigned int *raw_als_count)
{
	int lens_adj_lux = -1;
	int error;
	unsigned int als_read_data;
	u8 als_lower;
	u8 als_upper;

	error = isl29032_read_reg(isl, ISL29032_ALSIR_DT1, &als_lower);
	if (error != 0) {
		pr_err("%s:Unable to read ISL29032_ALSIR_DT1 register: %d\n",
			__func__, error);
		return error;
	}

	error = isl29032_read_reg(isl, ISL29032_ALSIR_DT2, &als_upper);
	if (error != 0) {
		pr_err("%s:Unable to read ISL29032_ALSIR_DT2 register: %d\n",
			__func__, error);
		return error;
	}

	als_read_data = (als_upper << 8) | als_lower;
	if (raw_als_count)
		*raw_als_count = als_read_data;

	if (isl29032_debug & ISL29032_DBG_INPUT)
		pr_info("%s: Data read from ALS 0x%X\n",
		__func__, als_read_data);

	lens_adj_lux = (isl->lux_level * als_read_data) /
		(isl->pdata->lens_percent_t * 41);

	return lens_adj_lux;
}

static int isl29032_set_als_thresholds(struct isl29032_data *isl,
				       unsigned int raw_als_count)
{
	unsigned int zone_size, als_low, als_high;
	unsigned int switch_range = 0;

	/* turn off ALS since we're going to be reconfiguring it */
	if (isl29032_set_als_enable(isl, 0))
		return -EFAULT;

	/* if we're in the highest low-lux range, switch to high lux
		or if in lowest high-lux range, switch to low lux*/
	if ((isl->lux_level == ISL29032_LOW_LUX_RANGE) &&
		(raw_als_count >= ISL29032_LOW_TO_HIGH_COUNTS)) {
		if (isl29032_debug & ISL29032_DBG_INPUT)
			pr_info("%s: Switching to high lux range\n", __func__);
		isl->lux_level	= ISL29032_HIGH_LUX_RANGE;
		switch_range	= 1;
		raw_als_count	= raw_als_count * ISL29032_LOW_LUX_RANGE
			/ ISL29032_HIGH_LUX_RANGE;

	} else if ((isl->lux_level == ISL29032_HIGH_LUX_RANGE) &&
		(raw_als_count <= ISL29032_HIGH_TO_LOW_COUNTS)) {
		if (isl29032_debug & ISL29032_DBG_INPUT)
			pr_info("%s: Switching to low lux range\n", __func__);
		isl->lux_level	= ISL29032_LOW_LUX_RANGE;
		switch_range	= 1;
		raw_als_count	= raw_als_count * ISL29032_HIGH_LUX_RANGE
			/ ISL29032_LOW_LUX_RANGE;
	}

	zone_size = 1;
	als_low = ((raw_als_count > zone_size) ? raw_als_count - zone_size : 0);
	if (raw_als_count <= 0xFFE) {
		als_high = raw_als_count + zone_size;
		als_high = ((als_high > 0xFFE) ? 0xFFE : als_high);
	} else {
		als_high = 0xFFF;
	}

	/* reconfigure if needed */
	if (switch_range) {
		isl29032_set_als_range(isl,
			(isl->lux_level == ISL29032_LOW_LUX_RANGE) ? 0 : 1);
	}

	if (isl29032_write_reg(isl, ISL29032_ALSIR_TH1, als_low & 0x0FF) ||
		isl29032_write_reg(isl, ISL29032_ALSIR_TH2,
			((als_low & 0xF00) >> 8) | ((als_high & 0x00F) << 4)) ||
		isl29032_write_reg(isl, ISL29032_ALSIR_TH3,
			(als_high & 0xFF0) >> 4)) {
		pr_err("%s: Couldn't set als thresholds\n", __func__);

		return -EFAULT;
	}

	if (isl29032_set_als_enable(isl, 1))
		return -EFAULT;

	return 0;
}

static int isl29032_report_prox(struct isl29032_data *isl, int force_report)
{
	int error = 0;
	u8 prox_data = 0;

	error = isl29032_read_reg(isl, ISL29032_PROX_DATA, &prox_data);
	if (error != 0) {
		pr_err("%s:Unable to read ISL29032_PROX_DATA register: %d\n",
			__func__, error);
		return error;
	}

	if (isl29032_debug & ISL29032_DBG_INPUT)
		pr_info("%s: Data read from PROX 0x%X, near=%d\n",
		__func__, prox_data, isl->prox_near);

	if (isl->prox_near && (prox_data < isl->prox_lt)) {
		isl->prox_near = 0;

		if (isl29032_write_reg(isl, ISL29032_PROX_LT, (u8)0) ||
			isl29032_write_reg(isl, ISL29032_PROX_HT,
			(u8)isl->prox_ht)) {
			pr_err("%s: Error writing to prox registers\n",
				__func__);
			return -EIO;
		}

		if (isl29032_debug & ISL29032_DBG_INPUT)
			pr_info("%s: Prox far detected\n", __func__);
	} else if (!isl->prox_near && (prox_data > isl->prox_ht)) {
		isl->prox_near = 1;

		if (isl29032_write_reg(isl, ISL29032_PROX_LT,
			(u8)isl->prox_lt) ||
			isl29032_write_reg(isl, ISL29032_PROX_HT, (u8)0xFF)) {
			pr_err("%s: Error writing to prox registers\n",
				__func__);
			return -EIO;
		}

		if (isl29032_debug & ISL29032_DBG_INPUT)
			pr_info("%s: Prox near detected\n", __func__);
	}

	/* Don't report the prox state if it hasn't changed. */
	if (force_report || (isl->prox_near != isl->last_prox_near)) {
		isl->last_prox_near = isl->prox_near;
		input_report_abs(isl->dev, ABS_DISTANCE,
			isl->prox_near ? PROXIMITY_NEAR : PROXIMITY_FAR);
		input_sync(isl->dev);
	}

	return !isl->prox_near;
}

static void isl29032_report_als(struct isl29032_data *isl)
{
	unsigned int raw_als_data = 0;
	int lux_val;

	lux_val = isl29032_read_adj_als(isl, &raw_als_data);
	isl29032_set_als_thresholds(isl, raw_als_data);

	if (lux_val >= 0) {
		input_event(isl->dev, EV_LED, LED_MISC,
			((lux_val > 1) ? lux_val : 2));
		input_sync(isl->dev);
	}
}

static unsigned int isl29032_get_avg_noise_floor(struct isl29032_data *isl)
{
	int err = -EINVAL;
	unsigned int i, sum = 0, avg = 0;
	u8 prox_data;
	unsigned int num_samples = isl->pdata->num_samples_for_noise_floor;

	/* turn off PROX_EN */
	isl29032_set_prox_enable(isl, 0);
	msleep(2);

	for (i = 0; i < num_samples; i++) {
		/* turn on PROX_EN */
		err = isl29032_set_prox_enable(isl, 1);
		if (err) {
			pr_err("%s: Error enabling proximity sensor: %d\n",
				__func__, err);
			break;
		}
		msleep(2); /* sleep for a bit before reading PROX_DATA */
		err = isl29032_read_reg(isl, ISL29032_PROX_DATA, &prox_data);
		if (err) {
			pr_err("%s: Error reading prox data: %d\n",
				__func__, err);
			break;
		}
		/* turn back off */
		err = isl29032_set_prox_enable(isl, 0);
		if (err) {
			pr_err("%s: Error disabling proximity sensor: %d\n",
				__func__, err);
			break;
		}
		msleep(2);
		sum += prox_data;
	}

	if (!err)
		avg = sum / num_samples;

	if (isl29032_debug & ISL29032_DBG_ENABLE_DISABLE)
		pr_info("%s: Noise floor is 0x%x ", __func__, avg);

	return avg;
}

static int isl29032_set_prox_thresholds(struct isl29032_data *isl,
					int avg_noise_floor)
{
	unsigned int offset;

	if ((avg_noise_floor >
		(isl->pdata->crosstalk_vs_covered_threshold)) ||
		(avg_noise_floor == 0)) {
		offset = isl->pdata->default_prox_noise_floor;
	} else {
		offset = avg_noise_floor;
	}
	isl->prox_lt = offset + isl->pdata->prox_lower_threshold;
	isl->prox_ht = offset + isl->pdata->prox_higher_threshold;

	/* check for overflow beyond 1 byte */
	if ((isl->prox_lt > 0xFF) || (isl->prox_ht > 0xFF)) {
		pr_err("%s: noise adjusted proximity thresholds are 0x%x "
			"and 0x%x, overflowing 8 bits, using defaults\n",
			__func__, isl->prox_lt, isl->prox_ht);
		isl->prox_lt = isl->pdata->prox_lower_threshold;
		isl->prox_ht = isl->pdata->prox_higher_threshold;
	}

	if (isl29032_write_reg(isl, ISL29032_PROX_LT, (u8)isl->prox_lt) ||
		isl29032_write_reg(isl, ISL29032_PROX_HT, (u8)isl->prox_ht)) {
		pr_err("%s: Error writing to prox threshold registers\n",
			__func__);
		return -EIO;
	}
	return 0;
}

static void isl29032_device_power_off(struct isl29032_data *isl)
{
	int error;

	if (isl29032_debug & ISL29032_DBG_POWER_ON_OFF)
		pr_info("%s: initialized=%d\n", __func__, isl->hw_initialized);

	if (isl->hw_initialized && isl->regulator) {
		disable_irq_nosync(isl->client->irq);
		error = regulator_disable(isl->regulator);
		if (error) {
			pr_err("%s: regulator_disable failed: %d\n",
				__func__, error);
			enable_irq(isl->client->irq);
			return;
		}
		isl->hw_initialized = 0;
	}
}

static int isl29032_device_power_on(struct isl29032_data *isl)
{
	int error;

	if (isl29032_debug & ISL29032_DBG_POWER_ON_OFF)
		pr_info("%s: initialized=%d\n", __func__, isl->hw_initialized);

	if (!isl->hw_initialized) {
		if (isl->regulator) {
			error = regulator_enable(isl->regulator);
			if (error) {
				pr_err("%s: regulator_enable failed: %d\n",
					__func__, error);
				return error;
			}
		}

		error = isl29032_init_registers(isl);
		if (error < 0) {
			pr_err("%s: init_registers failed: %d\n",
				__func__, error);
			if (isl->regulator)
				regulator_disable(isl->regulator);
			return error;
		}

		enable_irq(isl->client->irq);

		isl->hw_initialized = 1;
	}

	return 0;
}

static int isl29032_enable_prox(struct isl29032_data *isl)
{
	int error;
	int avg_noise_floor;

	if (isl->suspended) {
		if (isl29032_debug & ISL29032_DBG_ENABLE_DISABLE)
			pr_info("%s: Powering on\n", __func__);
		error = isl29032_device_power_on(isl);
		if (error)
			return error;
	}

	if (!isl->prox_enabled) {
		avg_noise_floor = isl29032_get_avg_noise_floor(isl);
		error = isl29032_set_prox_thresholds(isl, avg_noise_floor);
		if (error) {
			pr_err("%s: Error settin prox thresholds: %d\n",
				__func__, error);
			return error;
		}

		error = isl29032_set_prox_enable(isl, 1);
		if (error) {
			pr_err("%s: Error enabling prox: %d\n",
				__func__, error);
			return error;
		}

		isl29032_report_prox(isl, 1);
	}

	isl->prox_enabled = 1;

	return 0;
}

static int isl29032_disable_prox(struct isl29032_data *isl)
{
	int error;

	if (isl->prox_enabled) {
		error = isl29032_set_prox_enable(isl, 0);
		if (error) {
			pr_err("%s: Unable to turn off prox: %d\n",
				__func__, error);
			return error;
		}
		isl29032_clear_prox_flag(isl);
	}

	isl->prox_enabled = 0;

	if (isl->suspended && isl->hw_initialized) {
		if (isl29032_debug & ISL29032_DBG_ENABLE_DISABLE)
			pr_info("%s: Powering off\n", __func__);
		isl29032_device_power_off(isl);
	}

	return 0;
}

static int isl29032_enable_als(struct isl29032_data *isl)
{
	int error;

	if (!isl->als_enabled && !isl->suspended) {
		error = isl29032_set_als_enable(isl, 1);
		if (error) {
			pr_err("%s: Unable to turn on ALS: %d\n",
				__func__, error);
			return error;
		}

		msleep(100); /* Allow the light sensor to read the zone */

		isl29032_report_als(isl);
	}

	isl->als_enabled = 1;

	return 0;
}

static int isl29032_disable_als(struct isl29032_data *isl)
{
	int error;

	if (isl->als_enabled) {
		error = isl29032_set_als_enable(isl, 0);
		if (error) {
			pr_err("%s: Unable to turn off ALS: %d\n",
				__func__, error);
			return error;
		}
	}

	isl->als_enabled = 0;

	return 0;
}

static int isl29032_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = isl29032_misc_data;

	return 0;
}

static long isl29032_misc_ioctl_locked(struct isl29032_data *isl,
				       unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 enable;

	if (isl29032_debug & ISL29032_DBG_IOCTL)
		pr_info("%s\n", __func__);

	switch (cmd) {
	case ISL29032_IOCTL_SET_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			isl29032_enable_prox(isl);
		else
			isl29032_disable_prox(isl);

		break;

	case ISL29032_IOCTL_GET_ENABLE:
		enable = isl->prox_enabled;
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	case ISL29032_IOCTL_SET_LIGHT_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			isl29032_enable_als(isl);
		else
			isl29032_disable_als(isl);

		break;

	case ISL29032_IOCTL_GET_LIGHT_ENABLE:
		enable = isl->als_enabled;
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static long isl29032_misc_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct isl29032_data *isl = file->private_data;
	int error;

	if (isl29032_debug & ISL29032_DBG_IOCTL)
		pr_info("%s: cmd = 0x%08X\n", __func__, cmd);

	mutex_lock(&isl->mutex);
	error = isl29032_misc_ioctl_locked(isl, cmd, arg);
	mutex_unlock(&isl->mutex);

	return error;
}

static const struct file_operations isl29032_misc_fops = {
	.owner = THIS_MODULE,
	.open = isl29032_misc_open,
	.unlocked_ioctl = isl29032_misc_ioctl,
};

static ssize_t ld_isl29032_registers_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct isl29032_data *isl = i2c_get_clientdata(client);
	unsigned int i, n, reg_count;
	u8 value;

	reg_count = sizeof(isl29032_regs) / sizeof(isl29032_regs[0]);
	mutex_lock(&isl->mutex);
	for (i = 0, n = 0; i < reg_count; i++) {
		isl29032_read_reg(isl, isl29032_regs[i].reg, &value);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			"%-20s = 0x%02X\n",
			isl29032_regs[i].name,
			value);
	}
	mutex_unlock(&isl->mutex);

	return n;
}

static ssize_t ld_isl29032_registers_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct isl29032_data *isl = i2c_get_clientdata(client);
	unsigned int i, reg_count, value;
	int error;
	char name[30];

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -EMSGSIZE;
	}

	if (sscanf(buf, "%30s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -EINVAL;
	}

	reg_count = sizeof(isl29032_regs) / sizeof(isl29032_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, isl29032_regs[i].name)) {
			switch (isl29032_regs[i].reg) {
			case ISL29032_PROX_LT:
				isl->pdata->
					prox_lower_threshold = value;
			break;

			case ISL29032_PROX_HT:
				isl->pdata->
					prox_higher_threshold = value;
			break;

			case ISL29032_TEST1:
				mutex_lock(&isl->mutex);
				isl29032_enable_prox(isl);
				mutex_unlock(&isl->mutex);
				return count;
			break;

			case ISL29032_TEST2:
				mutex_lock(&isl->mutex);
				isl29032_disable_prox(isl);
				mutex_unlock(&isl->mutex);
				return count;
			break;
			}
			mutex_lock(&isl->mutex);
			error = isl29032_write_reg(isl,
				isl29032_regs[i].reg,
				value);
			mutex_unlock(&isl->mutex);
			if (error) {
				pr_err("%s:Failed to write register %s\n",
					__func__, name);
				return error;
			}
			return count;
		}
	}

	pr_err("%s:no such register %s\n", __func__, name);
	return -EINVAL;
}

static DEVICE_ATTR(registers, 0644, ld_isl29032_registers_show,
		   ld_isl29032_registers_store);

static irqreturn_t ld_isl29032_irq_handler(int irq, void *dev)
{
	struct isl29032_data *isl = dev;

	disable_irq_nosync(isl->client->irq);
	queue_work(isl->workqueue, &isl->work);
	enable_irq(isl->client->irq);

	return IRQ_HANDLED;
}

static void isl29032_work_func_locked(struct isl29032_data *isl)
{
	int error;
	int clear_prox = 1;
	u8 interrupt_reg;

	error = isl29032_read_reg(isl, ISL29032_INTERRUPT, &interrupt_reg);
	if (error != 0) {
		pr_err("%s: Unable to read interrupt register: %d\n",
			__func__, error);
		return;
	}

	if (isl->als_enabled && (interrupt_reg & ISL29032_ALS_FLAG_MASK))
		isl29032_report_als(isl);

	if (isl->prox_enabled && (interrupt_reg & ISL29032_PROX_FLAG_MASK))
		clear_prox = isl29032_report_prox(isl, 0);

	isl29032_clear_prox_and_als_flags(isl);
}

static void ld_isl29032_work_func(struct work_struct *work)
{
	struct isl29032_data *isl =
		container_of(work, struct isl29032_data, work);

	mutex_lock(&isl->mutex);
	if (isl->hw_initialized)
		isl29032_work_func_locked(isl);
	mutex_unlock(&isl->mutex);
}

static int isl29032_suspend(struct isl29032_data *isl)
{
	int error;

	if (isl->als_enabled) {
		error = isl29032_set_als_enable(isl, 0);
		isl29032_clear_als_flag(isl);
		if (error) {
			pr_err("%s: Unable to turn off ALS: %d\n",
				__func__, error);
			return error;
		}
		if (isl29032_debug & ISL29032_DBG_SUSPEND_RESUME)
			pr_info("%s: turned off ALS\n", __func__);
	}

	if (!isl->prox_enabled)
		isl29032_device_power_off(isl);

	isl->suspended = 1;

	return 0;
}

static int isl29032_resume(struct isl29032_data *isl)
{
	int error;

	isl29032_device_power_on(isl);

	if (isl->als_enabled) {
		error = isl29032_set_als_enable(isl, 1);
		if (error) {
			pr_err("%s: Unable to turn on ALS: %d\n",
				__func__, error);
			return error;
		}

		msleep(100); /* Allow the light sensor to read the zone */

		isl29032_report_als(isl);
	}

	isl->suspended = 0;

	return 0;
}

static int isl29032_pm_event(struct notifier_block *this, unsigned long event,
			     void *ptr)
{
	struct isl29032_data *isl = container_of(this,
		struct isl29032_data, pm_notifier);

	if (isl29032_debug & ISL29032_DBG_SUSPEND_RESUME)
		pr_info("%s: event = %lu\n", __func__, event);

	mutex_lock(&isl->mutex);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		isl29032_suspend(isl);
		break;
	case PM_POST_SUSPEND:
		isl29032_resume(isl);
		break;
	}

	mutex_unlock(&isl->mutex);

	return NOTIFY_DONE;
}

static int ld_isl29032_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct isl29032_platform_data *pdata = client->dev.platform_data;
	struct isl29032_data *isl;
	int error = 0;

	if (pdata == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;
	}

	client->irq = pdata->irq;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s:I2C_FUNC_I2C not supported\n", __func__);
		return -ENODEV;
	}

	isl = kzalloc(sizeof(struct isl29032_data), GFP_KERNEL);
	if (isl == NULL) {
		error = -ENOMEM;
		goto error_alloc_data_failed;
	}

	pdata->configure &= ~ISL29032_CNF_ALS_EN_MASK;
	pdata->configure &= ~ISL29032_CNF_PROX_EN_MASK;
	pdata->configure |= ISL29032_CNF_ALS_RANGE_MASK;

	isl->client = client;
	isl->pdata = pdata;

	if (isl->pdata->regulator_name[0] != '\0') {
		isl->regulator = regulator_get(NULL,
			isl->pdata->regulator_name);
		if (IS_ERR(isl->regulator)) {
			pr_err("%s: cannot acquire regulator [%s]\n",
				__func__, isl->pdata->regulator_name);
			error = PTR_ERR(isl->regulator);
			goto error_regulator_get_failed;
		}
	}

	isl->dev = input_allocate_device();
	if (!isl->dev) {
		error = -ENOMEM;
		pr_err("%s: input device allocate failed: %d\n", __func__,
			error);
		goto error_input_allocate_failed;
	}

	isl->dev->name = "light-prox";
	input_set_capability(isl->dev, EV_LED, LED_MISC);
	input_set_capability(isl->dev, EV_ABS, ABS_DISTANCE);

	isl29032_misc_data = isl;
	isl->miscdevice.minor = MISC_DYNAMIC_MINOR;
	isl->miscdevice.name = LD_ISL29032_NAME;
	isl->miscdevice.fops = &isl29032_misc_fops;
	error = misc_register(&isl->miscdevice);
	if (error < 0) {
		pr_err("%s: misc_register failed\n", __func__);
		goto error_misc_register_failed;
	}

	isl->pm_notifier.notifier_call = isl29032_pm_event;
	error = register_pm_notifier(&isl->pm_notifier);
	if (error < 0) {
		pr_err("%s: register_pm_notifier failed\n", __func__);
		goto error_register_pm_notifier_failed;
	}

	isl->hw_initialized = 0;
	isl->suspended = 0;

	isl->prox_enabled = 0;
	isl->als_enabled = 0;
	isl->lux_level = ISL29032_HIGH_LUX_RANGE;
	isl->prox_near = 0;
	isl->last_prox_near = 0;

	isl->workqueue = create_singlethread_workqueue("als_wq");
	if (!isl->workqueue) {
		pr_err("%s: Cannot create work queue\n", __func__);
		error = -ENOMEM;
		goto error_create_wq_failed;
	}

	INIT_WORK(&isl->work, ld_isl29032_work_func);

	error = request_irq(client->irq,
		ld_isl29032_irq_handler,
		(IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING),
		LD_ISL29032_NAME, isl);
	if (error != 0) {
		pr_err("%s: irq request failed: %d\n", __func__, error);
		error = -ENODEV;
		goto error_req_irq_failed;
	}
	enable_irq_wake(client->irq);
	disable_irq(client->irq);

	i2c_set_clientdata(client, isl);

	mutex_init(&isl->mutex);

	error = input_register_device(isl->dev);
	if (error) {
		pr_err("%s: input device register failed:%d\n", __func__,
			error);
		goto error_input_register_failed;
	}

	error = device_create_file(&client->dev, &dev_attr_registers);
	if (error < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, error);
		error = -ENODEV;
		goto error_create_registers_file_failed;
	}

	error = isl29032_device_power_on(isl);
	if (error < 0) {
		pr_err("%s:power_on failed: %d\n", __func__, error);
		goto error_power_on_failed;
	}

	return 0;

error_power_on_failed:
	device_remove_file(&client->dev, &dev_attr_registers);
error_create_registers_file_failed:
	input_unregister_device(isl->dev);
	isl->dev = NULL;
error_input_register_failed:
	mutex_destroy(&isl->mutex);
	i2c_set_clientdata(client, NULL);
	free_irq(isl->client->irq, isl);
error_req_irq_failed:
	destroy_workqueue(isl->workqueue);
error_create_wq_failed:
	unregister_pm_notifier(&isl->pm_notifier);
error_register_pm_notifier_failed:
	misc_deregister(&isl->miscdevice);
error_misc_register_failed:
	input_free_device(isl->dev);
error_input_allocate_failed:
	regulator_put(isl->regulator);
error_regulator_get_failed:
	kfree(isl);
error_alloc_data_failed:
	return error;
}

static int ld_isl29032_remove(struct i2c_client *client)
{
	struct isl29032_data *isl = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_registers);

	isl29032_device_power_off(isl);

	input_unregister_device(isl->dev);

	mutex_destroy(&isl->mutex);
	i2c_set_clientdata(client, NULL);
	free_irq(isl->client->irq, isl);

	destroy_workqueue(isl->workqueue);

	unregister_pm_notifier(&isl->pm_notifier);

	misc_deregister(&isl->miscdevice);

	regulator_put(isl->regulator);

	kfree(isl);

	return 0;
}

static const struct i2c_device_id isl29032_id[] = {
	{LD_ISL29032_NAME, 0},
	{}
};

static struct i2c_driver ld_isl29032_i2c_driver = {
	.probe		= ld_isl29032_probe,
	.remove		= ld_isl29032_remove,
	.id_table	= isl29032_id,
	.driver = {
		.name = LD_ISL29032_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ld_isl29032_init(void)
{
	return i2c_add_driver(&ld_isl29032_i2c_driver);
}

static void __exit ld_isl29032_exit(void)
{
	i2c_del_driver(&ld_isl29032_i2c_driver);
}

module_init(ld_isl29032_init);
module_exit(ld_isl29032_exit);

MODULE_DESCRIPTION("ALS and Proximity driver for ISL29032");
MODULE_AUTHOR("Motorola Mobility");
MODULE_LICENSE("GPL");
