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

#include <linux/ct405.h>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#define CT405_I2C_RETRIES	2
#define CT405_I2C_RETRY_DELAY	5

#define CT405_COMMAND_SELECT		0x80
#define CT405_COMMAND_AUTO_INCREMENT	0x20
#define CT405_COMMAND_SPECIAL_FUNCTION	0x60
#define CT405_COMMAND_PROX_INT_CLEAR	0x05
#define CT405_COMMAND_ALS_INT_CLEAR	0x06

#define CT405_ENABLE			0x00
#define CT405_ENABLE_PIEN		(1<<5)
#define CT405_ENABLE_AIEN		(1<<4)
#define CT405_ENABLE_WEN		(1<<3)
#define CT405_ENABLE_PEN		(1<<2)
#define CT405_ENABLE_AEN		(1<<1)
#define CT405_ENABLE_PON		(1<<0)

#define CT405_ATIME			0x01
#define CT405_ATIME_SATURATED		0xFF
#define CT405_ATIME_NOT_SATURATED	0xEE

#define CT405_PTIME			0x02

#define CT405_WTIME			0x03
#define CT405_WTIME_ALS_OFF		0xEE
#define CT405_WTIME_ALS_ON		0xFF
#define CT405_WTIME_SATURATED		0xFF

#define CT405_AILTL			0x04
#define CT405_AILTH			0x05
#define CT405_AIHTL			0x06
#define CT405_AIHTH			0x07
#define CT405_PILTL			0x08
#define CT405_PILTH			0x09
#define CT405_PIHTL			0x0A
#define CT405_PIHTH			0x0B

#define CT405_PERS			0x0C
#define CT405_PERS_PPERS		0xF0
#define CT405_PERS_APERS		0x0F
#define CT405_PERS_APERS_NOT_SATURATED	0x14
#define CT405_PERS_APERS_SATURATED	0x13

#define CT405_CONFIG			0x0D
#define CT405_CONFIG_AGL		(1<<2)

#define CT405_PPCOUNT			0x0E

#define CT405_CONTROL			0x0F
#define CT405_CONTROL_PDIODE_CH0	0x10
#define CT405_CONTROL_PDIODE_CH1	0x20
#define CT405_CONTROL_PGAIN_1X		0x00
#define CT405_CONTROL_PGAIN_2X		0x04
#define CT405_CONTROL_PGAIN_4X		0x08
#define CT405_CONTROL_PGAIN_8X		0x0C
#define CT405_CONTROL_AGAIN_1X		0x00
#define CT405_CONTROL_AGAIN_8X		0x01
#define CT405_CONTROL_AGAIN_16X		0x02
#define CT405_CONTROL_AGAIN_120X	0x03

#define CT405_REV_ID			0x11

#define CT405_ID			0x12

#define CT405_STATUS			0x13
#define CT405_STATUS_PINT		(1<<5)
#define CT405_STATUS_AINT		(1<<4)

#define CT405_C0DATA			0x14
#define CT405_C0DATAH			0x15
#define CT405_C1DATA			0x16
#define CT405_C1DATAH			0x17
#define CT405_PDATA			0x18
#define CT405_PDATAH			0x19

#define CT405_C0DATA_MAX		0xFFFF
#define CT405_PDATA_MAX			0x03FF
#define CT406_PDATA_MAX			0x07FF

#define CT405_PROXIMITY_NEAR		30	/* 30 mm */
#define CT405_PROXIMITY_FAR		1000	/* 1 meter */

#define CT405_ALS_LOW_TO_HIGH_THRESHOLD	200	/* 200 lux */
#define CT405_ALS_HIGH_TO_LOW_THRESHOLD	100	/* 100 lux */

#define CT40X_REV_ID_CT405 0x02
#define CT40X_REV_ID_CT406a 0x03
#define CT40X_REV_ID_CT406b 0x04

enum ct405_prox_mode {
	CT405_PROX_MODE_SATURATED,
	CT405_PROX_MODE_UNCOVERED,
	CT405_PROX_MODE_COVERED,
};

enum ct405_als_mode {
	CT405_ALS_MODE_SUNLIGHT,
	CT405_ALS_MODE_LOW_LUX,
	CT405_ALS_MODE_HIGH_LUX,
};

enum ct40x_hardware_type {
	CT405_HW_TYPE,
	CT406_HW_TYPE,
};

struct ct405_data {
	struct input_dev *dev;
	struct i2c_client *client;
	struct regulator *regulator;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct ct405_platform_data *pdata;
	struct miscdevice miscdevice;
	struct notifier_block pm_notifier;
	struct mutex mutex;
	struct wake_lock wl;
	/* state flags */
	unsigned int suspended;
	unsigned int regs_initialized;
	unsigned int oscillator_enabled;
	unsigned int prox_requested;
	unsigned int prox_enabled;
	enum ct405_prox_mode prox_mode;
	unsigned int als_requested;
	unsigned int als_enabled;
	enum ct405_als_mode als_mode;
	unsigned int wait_enabled;
	/* numeric values */
	unsigned int prox_noise_floor;
	unsigned int prox_low_threshold;
	unsigned int prox_high_threshold;
	unsigned int als_low_threshold;
	u16 prox_saturation_threshold;
	u16 prox_covered_offset;
	u16 prox_uncovered_offset;
	u16 prox_recalibrate_offset;
	u16 pdata_max;
	enum ct40x_hardware_type hw_type;
};

static struct ct405_data *ct405_misc_data;

static struct ct405_reg {
	const char *name;
	u8 reg;
} ct405_regs[] = {
	{ "ENABLE",	CT405_ENABLE },
	{ "ATIME",	CT405_ATIME },
	{ "PTIME",	CT405_PTIME },
	{ "WTIME",	CT405_WTIME },
	{ "AILTL",	CT405_AILTL },
	{ "AILTH",	CT405_AILTH },
	{ "AIHTL",	CT405_AIHTL },
	{ "AIHTH",	CT405_AIHTH },
	{ "PILTL",	CT405_PILTL },
	{ "PILTH",	CT405_PILTH },
	{ "PIHTL",	CT405_PIHTL },
	{ "PIHTH",	CT405_PIHTH },
	{ "PERS",	CT405_PERS },
	{ "CONFIG",	CT405_CONFIG },
	{ "PPCOUNT",	CT405_PPCOUNT },
	{ "CONTROL",	CT405_CONTROL },
	{ "ID",		CT405_ID },
	{ "STATUS",	CT405_STATUS },
	{ "C0DATA",	CT405_C0DATA },
	{ "C0DATAH",	CT405_C0DATAH },
	{ "C1DATA",	CT405_C1DATA },
	{ "C1DATAH",	CT405_C1DATAH },
	{ "PDATA",	CT405_PDATA },
	{ "PDATAH",	CT405_PDATAH },
};

#define CT405_DBG_INPUT			0x00000001
#define CT405_DBG_POWER_ON_OFF		0x00000002
#define CT405_DBG_ENABLE_DISABLE	0x00000004
#define CT405_DBG_IOCTL			0x00000008
#define CT405_DBG_SUSPEND_RESUME	0x00000010
static u32 ct405_debug = 0x00000000;
module_param_named(debug_mask, ct405_debug, uint, 0644);

static int ct405_i2c_read(struct ct405_data *ct, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = ct->client->addr,
			.flags = ct->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = ct->client->addr,
			.flags = (ct->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	buf[0] |= CT405_COMMAND_SELECT;
	do {
		err = i2c_transfer(ct->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(CT405_I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < CT405_I2C_RETRIES));

	if (err != 2) {
		pr_err("%s: read transfer error.\n", __func__);
		dev_err(&ct->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int ct405_i2c_write(struct ct405_data *ct, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = ct->client->addr,
			.flags = ct->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	buf[0] |= CT405_COMMAND_SELECT;
	do {
		err = i2c_transfer(ct->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(CT405_I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < CT405_I2C_RETRIES));

	if (err != 1) {
		pr_err("%s: write transfer error.\n", __func__);
		dev_err(&ct->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int ct405_write_enable(struct ct405_data *ct)
{
	int error = 0;
	u8 reg_data[2] = {0x00, 0x00};
	reg_data[0] = CT405_ENABLE;
	if (ct->oscillator_enabled || ct->als_enabled || ct->prox_enabled) {
		reg_data[1] |= CT405_ENABLE_PON;
		if (!ct->oscillator_enabled) {
			error = ct405_i2c_write(ct, reg_data, 1);
			if (error < 0)
				return error;
			msleep(3);
			ct->oscillator_enabled = 1;
		}
		if (ct->als_enabled)
			reg_data[1] |= CT405_ENABLE_AEN | CT405_ENABLE_AIEN;
		if (ct->prox_enabled)
			reg_data[1] |= CT405_ENABLE_PEN | CT405_ENABLE_PIEN;
		if (ct->wait_enabled)
			reg_data[1] |= CT405_ENABLE_WEN;
	}
	if (ct405_debug & CT405_DBG_ENABLE_DISABLE)
		pr_info("%s: writing ENABLE=0x%02x\n", __func__, reg_data[1]);

	return ct405_i2c_write(ct, reg_data, 1);
}

static int ct405_set_als_enable(struct ct405_data *ct,
				unsigned int enable)
{
	int error = 0;
	if (ct->als_enabled != enable) {
		ct->als_enabled = enable;
		if (ct->regs_initialized)
			error = ct405_write_enable(ct);
	}
	return error;
}

static int ct405_set_prox_enable(struct ct405_data *ct,
				 unsigned int enable)
{
	int error = 0;
	if (ct->prox_enabled != enable) {
		ct->prox_enabled = enable;
		if (ct->regs_initialized)
			error = ct405_write_enable(ct);
	}
	return error;
}

static int ct405_set_wait_enable(struct ct405_data *ct,
				 unsigned int enable)
{
	int error = 0;
	if (ct->wait_enabled != enable) {
		ct->wait_enabled = enable;
		if (ct->regs_initialized)
			error = ct405_write_enable(ct);
	}
	return error;
}

static int ct405_clear_als_flag(struct ct405_data *ct)
{
	u8 reg_data[1] = {0};
	reg_data[0] = CT405_COMMAND_SPECIAL_FUNCTION
		| CT405_COMMAND_ALS_INT_CLEAR;
	return ct405_i2c_write(ct, reg_data, 0);
}

static int ct405_clear_prox_flag(struct ct405_data *ct)
{
	u8 reg_data[1] = {0};
	reg_data[0] = CT405_COMMAND_SPECIAL_FUNCTION
		| CT405_COMMAND_PROX_INT_CLEAR;
	return ct405_i2c_write(ct, reg_data, 0);
}

static int ct405_init_registers(struct ct405_data *ct)
{
	int error = 0;
	u8 reg_data[3] = {0};

	/* write ALS integration time = ~49 ms */
	/* write prox integration time = ~3 ms */
	reg_data[0] = (CT405_ATIME | CT405_COMMAND_AUTO_INCREMENT);
	reg_data[1] = CT405_ATIME_NOT_SATURATED;
	if (ct->hw_type == CT405_HW_TYPE)
		reg_data[2] = 0xFF; /* 2.73 ms */
	else
		reg_data[2] = 0xFE; /* 5.46 ms */
	error = ct405_i2c_write(ct, reg_data, 2);
	if (error < 0)
		return error;

	/* write IR LED pulse count = 2 */
	/* write proximity diode = ch1, proximity gain = 1/2, ALS gain = 1 */
	reg_data[0] = (CT405_PPCOUNT | CT405_COMMAND_AUTO_INCREMENT);
	reg_data[1] = 2;
	if (ct->hw_type == CT405_HW_TYPE)
		reg_data[2] = CT405_CONTROL_PDIODE_CH1
			| CT405_CONTROL_PGAIN_2X | CT405_CONTROL_AGAIN_1X;
	else
		reg_data[2] = CT405_CONTROL_PDIODE_CH1
			| CT405_CONTROL_PGAIN_1X | CT405_CONTROL_AGAIN_1X;
	error = ct405_i2c_write(ct, reg_data, 2);
	if (error < 0)
		return error;

	return 0;
}

static void ct405_write_als_thresholds(struct ct405_data *ct)
{
	u8 reg_data[5] = {0};
	unsigned int ailt = ct->als_low_threshold;
	unsigned int aiht = CT405_C0DATA_MAX;
	int error;

	reg_data[0] = (CT405_AILTL | CT405_COMMAND_AUTO_INCREMENT);
	reg_data[1] = (ailt & 0xFF);
	reg_data[2] = ((ailt >> 8) & 0xFF);
	reg_data[3] = (aiht & 0xFF);
	reg_data[4] = ((aiht >> 8) & 0xFF);

	error = ct405_i2c_write(ct, reg_data, 4);
	if (error < 0)
		pr_err("%s: Error writing new ALS thresholds: %d\n",
			__func__, error);
}

static void ct405_als_mode_sunlight(struct ct405_data *ct)
{
	int error;
	u8 reg_data[2] = {0};

	/* set AGL to reduce ALS gain by 1/6 */
	reg_data[0] = CT405_CONFIG;
	reg_data[1] = CT405_CONFIG_AGL;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: error writing CONFIG: %d\n", __func__, error);
		return;
	}

	/* write ALS gain = 1 */
	reg_data[0] = CT405_CONTROL;
	if (ct->hw_type == CT405_HW_TYPE)
		reg_data[1] = CT405_CONTROL_PDIODE_CH1
			| CT405_CONTROL_PGAIN_2X | CT405_CONTROL_AGAIN_1X;
	else
		reg_data[1] = CT405_CONTROL_PDIODE_CH1
			| CT405_CONTROL_PGAIN_1X | CT405_CONTROL_AGAIN_1X;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: error writing ALS gain: %d\n", __func__, error);
		return;
	}

	/* write ALS integration time = ~3 ms */
	reg_data[0] = CT405_ATIME;
	reg_data[1] = CT405_ATIME_SATURATED;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: error writing ATIME: %d\n", __func__, error);
		return;
	}

	ct->als_mode = CT405_ALS_MODE_SUNLIGHT;
	ct->als_low_threshold = ct->prox_saturation_threshold;
	ct405_write_als_thresholds(ct);
}

static void ct405_als_mode_low_lux(struct ct405_data *ct)
{
	int error;
	u8 reg_data[2] = {0};

	/* clear AGL for regular ALS gain behavior */
	reg_data[0] = CT405_CONFIG;
	reg_data[1] = 0;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: error writing CONFIG: %d\n", __func__, error);
		return;
	}

	/* write ALS gain = 8 */
	reg_data[0] = CT405_CONTROL;
	if (ct->hw_type == CT405_HW_TYPE)
		reg_data[1] = CT405_CONTROL_PDIODE_CH1
			| CT405_CONTROL_PGAIN_2X | CT405_CONTROL_AGAIN_8X;
	else
		reg_data[1] = CT405_CONTROL_PDIODE_CH1
			| CT405_CONTROL_PGAIN_1X | CT405_CONTROL_AGAIN_8X;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: error writing ALS gain: %d\n", __func__, error);
		return;
	}

	/* write ALS integration time = ~49 ms */
	reg_data[0] = CT405_ATIME;
	reg_data[1] = CT405_ATIME_NOT_SATURATED;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: error writing ATIME: %d\n", __func__, error);
		return;
	}

	ct->als_mode = CT405_ALS_MODE_LOW_LUX;
	ct->als_low_threshold = CT405_C0DATA_MAX - 1;
	ct405_write_als_thresholds(ct);
}

static void ct405_als_mode_high_lux(struct ct405_data *ct)
{
	int error;
	u8 reg_data[2] = {0};

	/* clear AGL for regular ALS gain behavior */
	reg_data[0] = CT405_CONFIG;
	reg_data[1] = 0;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: error writing CONFIG: %d\n", __func__, error);
		return;
	}

	/* write ALS gain = 1 */
	reg_data[0] = CT405_CONTROL;
	if (ct->hw_type == CT405_HW_TYPE)
		reg_data[1] = CT405_CONTROL_PDIODE_CH1
			| CT405_CONTROL_PGAIN_2X | CT405_CONTROL_AGAIN_1X;
	else
		reg_data[1] = CT405_CONTROL_PDIODE_CH1
			| CT405_CONTROL_PGAIN_1X | CT405_CONTROL_AGAIN_1X;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: error writing ALS gain: %d\n", __func__, error);
		return;
	}

	/* write ALS integration time = ~49 ms */
	reg_data[0] = CT405_ATIME;
	reg_data[1] = CT405_ATIME_NOT_SATURATED;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: error writing ATIME: %d\n", __func__, error);
		return;
	}

	ct->als_mode = CT405_ALS_MODE_HIGH_LUX;
	ct->als_low_threshold = CT405_C0DATA_MAX - 1;
	ct405_write_als_thresholds(ct);
}

static void ct405_write_prox_thresholds(struct ct405_data *ct)
{
	u8 reg_data[5] = {0};
	unsigned int pilt = ct->prox_low_threshold;
	unsigned int piht = ct->prox_high_threshold;
	int error;

	reg_data[0] = (CT405_PILTL | CT405_COMMAND_AUTO_INCREMENT);
	reg_data[1] = (pilt & 0xFF);
	reg_data[2] = ((pilt >> 8) & 0xFF);
	reg_data[3] = (piht & 0xFF);
	reg_data[4] = ((piht >> 8) & 0xFF);

	error = ct405_i2c_write(ct, reg_data, 4);
	if (error < 0)
		pr_err("%s: Error writing new prox thresholds: %d\n",
			__func__, error);
}

static void ct405_prox_mode_saturated(struct ct405_data *ct)
{
	ct->prox_mode = CT405_PROX_MODE_SATURATED;
	pr_info("%s: Prox mode saturated\n", __func__);
}

static void ct405_prox_mode_uncovered(struct ct405_data *ct)
{
	unsigned int noise_floor = ct->prox_noise_floor;
	unsigned int pilt = noise_floor - ct->prox_recalibrate_offset;
	unsigned int piht = noise_floor + ct->prox_covered_offset;

	if (pilt > ct->pdata_max)
		pilt = 0;
	if (piht > ct->pdata_max)
		piht = ct->pdata_max;

	ct->prox_mode = CT405_PROX_MODE_UNCOVERED;
	ct->prox_low_threshold = pilt;
	ct->prox_high_threshold = piht;
	ct405_write_prox_thresholds(ct);
	pr_info("%s: Prox mode uncovered\n", __func__);
}

static void ct405_prox_mode_covered(struct ct405_data *ct)
{
	unsigned int noise_floor = ct->prox_noise_floor;
	unsigned int pilt = noise_floor + ct->prox_uncovered_offset;
	unsigned int piht = ct->pdata_max;

	if (pilt > ct->pdata_max)
		pilt = ct->pdata_max;

	ct->prox_mode = CT405_PROX_MODE_COVERED;
	ct->prox_low_threshold = pilt;
	ct->prox_high_threshold = piht;
	ct405_write_prox_thresholds(ct);
	pr_info("%s: Prox mode covered\n", __func__);
}

static void ct405_device_power_off(struct ct405_data *ct)
{
	int error;

	if (ct405_debug & CT405_DBG_POWER_ON_OFF)
		pr_info("%s: initialized=%d\n", __func__, ct->regs_initialized);

	if (ct->regs_initialized) {
		disable_irq_nosync(ct->client->irq);
		ct->oscillator_enabled = 0;
		error = ct405_write_enable(ct);
		if (error) {
			pr_err("%s: ct405_disable failed: %d\n",
				__func__, error);
		}
		ct->regs_initialized = 0;
	}

	if (ct->regulator) {
		error = regulator_disable(ct->regulator);
		if (error) {
			pr_err("%s: regulator_disable failed: %d\n",
				__func__, error);
		}
	}

}

static int ct405_device_power_on(struct ct405_data *ct)
{
	int error;

	if (ct405_debug & CT405_DBG_POWER_ON_OFF)
		pr_info("%s: initialized=%d\n", __func__, ct->regs_initialized);

	if (ct->regulator) {
		error = regulator_enable(ct->regulator);
		if (error) {
			pr_err("%s: regulator_enable failed: %d\n",
				__func__, error);
			return error;
		}
	}
	return 0;
}

static int ct405_device_init(struct ct405_data *ct)
{
	int error;

	if (!ct->regs_initialized) {
		error = ct405_init_registers(ct);
		if (error < 0) {
			pr_err("%s: init_registers failed: %d\n",
				__func__, error);
			if (ct->regulator)
				regulator_disable(ct->regulator);
			return error;
		}
		ct->oscillator_enabled = 0;
		ct->prox_enabled = 0;
		ct->prox_mode = CT405_PROX_MODE_SATURATED;
		ct->als_enabled = 0;
		ct->als_mode = CT405_ALS_MODE_LOW_LUX;

		enable_irq(ct->client->irq);

		ct->regs_initialized = 1;
	}

	return 0;
}

static void ct405_check_als_range(struct ct405_data *ct, unsigned int lux)
{
	if (ct->als_mode == CT405_ALS_MODE_LOW_LUX) {
		if (lux >= CT405_ALS_LOW_TO_HIGH_THRESHOLD)
			ct405_als_mode_high_lux(ct);
	} else if (ct->als_mode == CT405_ALS_MODE_HIGH_LUX) {
		if (lux < CT405_ALS_HIGH_TO_LOW_THRESHOLD)
			ct405_als_mode_low_lux(ct);
	}
}

static int ct405_enable_als(struct ct405_data *ct)
{
	int error;
	u8 reg_data[5] = {0};

	if (!ct->suspended && ct->als_mode != CT405_ALS_MODE_SUNLIGHT) {
		error = ct405_set_als_enable(ct, 0);
		if (error) {
			pr_err("%s: Unable to turn off ALS: %d\n",
				__func__, error);
			return error;
		}

		ct405_als_mode_low_lux(ct);

		/* write wait time = ALS on value */
		reg_data[0] = CT405_WTIME;
		reg_data[1] = CT405_WTIME_ALS_ON;
		error = ct405_i2c_write(ct, reg_data, 1);
		if (error < 0) {
			pr_err("%s: Error  %d\n", __func__, error);
			return error;
		}
		error = ct405_set_wait_enable(ct, 0);
		if (error) {
			pr_err("%s: Unable to set wait enable: %d\n",
				__func__, error);
			return error;
		}

		/* write ALS interrupt persistence */
		reg_data[0] = CT405_PERS;
		reg_data[1] = CT405_PERS_APERS_NOT_SATURATED;
		error = ct405_i2c_write(ct, reg_data, 1);
		if (error < 0) {
			pr_err("%s: Error  %d\n", __func__, error);
			return error;
		}

		ct405_clear_als_flag(ct);

		error = ct405_set_als_enable(ct, 1);
		if (error) {
			pr_err("%s: Unable to turn on ALS: %d\n",
				__func__, error);
			return error;
		}
	}

	return 0;
}

static int ct405_disable_als(struct ct405_data *ct)
{
	int error;
	u8 reg_data[2] = {0};

	if (ct->als_enabled && ct->als_mode != CT405_ALS_MODE_SUNLIGHT) {
		error = ct405_set_als_enable(ct, 0);
		if (error < 0) {
			pr_err("%s: Error  %d\n", __func__, error);
			return error;
		}


		/* write wait time = ALS off value */
		reg_data[0] = CT405_WTIME;
		reg_data[1] = CT405_WTIME_ALS_OFF;
		error = ct405_i2c_write(ct, reg_data, 1);
		if (error < 0) {
			pr_err("%s: Error  %d\n", __func__, error);
			return error;
		}
		error = ct405_set_wait_enable(ct, 1);
		if (error) {
			pr_err("%s: Unable to set wait enable: %d\n",
				__func__, error);
			return error;
		}

		ct405_clear_als_flag(ct);
	}

	return 0;
}

static void ct405_measure_noise_floor(struct ct405_data *ct)
{
	int error = -EINVAL;
	unsigned int num_samples = ct->pdata->prox_samples_for_noise_floor;
	unsigned int i, sum = 0, avg = 0;
	unsigned int max = ct->pdata_max - 1 - ct->prox_covered_offset;
	u8 reg_data[2] = {0};

	/* disable ALS temporarily */
	error = ct405_set_als_enable(ct, 0);
	if (error < 0)
		pr_err("%s: error disabling ALS: %d\n",
			__func__, error);

	/* clear AGL for regular ALS gain behavior */
	reg_data[0] = CT405_CONFIG;
	reg_data[1] = 0;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0)
		pr_err("%s: error writing CONFIG: %d\n",
			__func__, error);

	/* write wait time = ALS off value */
	reg_data[0] = CT405_WTIME;
	reg_data[1] = CT405_WTIME_ALS_OFF;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0)
		pr_err("%s: Error writing WTIME: %d\n",
				__func__, error);
	error = ct405_set_wait_enable(ct, 1);
	if (error) {
		pr_err("%s: Unable to set wait enable: %d\n",
			__func__, error);
	}

	for (i = 0; i < num_samples; i++) {
		/* enable prox sensor and wait */
		error = ct405_set_prox_enable(ct, 1);
		if (error) {
			pr_err("%s: Error enabling proximity sensor: %d\n",
				__func__, error);
			break;
		}
		msleep(5);

		reg_data[0] = (CT405_PDATA | CT405_COMMAND_AUTO_INCREMENT);
		error = ct405_i2c_read(ct, reg_data, 2);
		if (error) {
			pr_err("%s: Error reading prox data: %d\n",
				__func__, error);
			break;
		}
		sum += (reg_data[1] << 8) | reg_data[0];

		/* disable prox sensor */
		error = ct405_set_prox_enable(ct, 0);
		if (error) {
			pr_err("%s: Error disabling proximity sensor: %d\n",
				__func__, error);
			break;
		}
	}

	if (!error)
		avg = sum / num_samples;

	if (avg < max)
		ct->prox_noise_floor = avg;
	else
		ct->prox_noise_floor = max;

	if (ct405_debug & CT405_DBG_ENABLE_DISABLE)
		pr_info("%s: Noise floor is 0x%x\n", __func__,
			ct->prox_noise_floor);

	ct405_prox_mode_uncovered(ct);

	pr_info("%s: Prox mode startup\n", __func__);
	error = ct405_set_prox_enable(ct, 1);
	if (error)
		pr_err("%s: Error enabling proximity sensor: %d\n",
			__func__, error);

	/* re-enable ALS if necessary */
	ct405_als_mode_low_lux(ct);
	if (ct->als_requested && !ct->suspended)
		ct405_enable_als(ct);
}

static int ct405_check_saturation(struct ct405_data *ct)
{
	int error = 0;
	u8 reg_data[2] = {0};
	unsigned int c0data;

	/* disable prox */
	ct405_set_prox_enable(ct, 0);
	ct->prox_low_threshold = 0;
	ct->prox_high_threshold = ct->pdata_max;
	ct405_write_prox_thresholds(ct);

	ct405_als_mode_sunlight(ct);
	ct405_set_als_enable(ct, 1);

	/* write ALS interrupt persistence = saturated value */
	reg_data[0] = CT405_PERS;
	reg_data[1] = CT405_PERS_APERS_SATURATED;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0)
		return 0;

	/* write wait time = saturated value */
	reg_data[0] = CT405_WTIME;
	reg_data[1] = CT405_WTIME_SATURATED;
	error = ct405_i2c_write(ct, reg_data, 1);
	if (error < 0)
		return 0;
	error = ct405_set_wait_enable(ct, 1);
	if (error) {
		pr_err("%s: Unable to set wait enable: %d\n",
			__func__, error);
		return error;
	}

	msleep(5);

	/* read C0DATA */
	reg_data[0] = (CT405_C0DATA | CT405_COMMAND_AUTO_INCREMENT);
	error = ct405_i2c_read(ct, reg_data, 2);
	if (error < 0)
		return 0;
	c0data = (reg_data[1] << 8) | reg_data[0];

	return (c0data > ct->als_low_threshold);
}

static int ct405_enable_prox(struct ct405_data *ct)
{
	int error;

	if (ct->suspended) {
		if (ct405_debug & CT405_DBG_ENABLE_DISABLE)
			pr_info("%s: Powering on\n", __func__);
		error = ct405_device_power_on(ct);
		if (error)
			return error;
		error = ct405_device_init(ct);
		if (error)
			return error;
	}

	if (ct405_check_saturation(ct))
		ct405_prox_mode_saturated(ct);
	else
		ct405_measure_noise_floor(ct);

	return 0;
}

static int ct405_disable_prox(struct ct405_data *ct)
{
	int error;

	if (ct->prox_enabled) {
		error = ct405_set_prox_enable(ct, 0);
		if (error) {
			pr_err("%s: Error disabling proximity sensor: %d\n",
				__func__, error);
			return error;
		}
		ct405_clear_prox_flag(ct);
		ct405_prox_mode_saturated(ct);

		if (ct->als_requested) {
			if (ct->als_mode == CT405_ALS_MODE_SUNLIGHT) {
				ct405_als_mode_low_lux(ct);
				if (!ct->suspended)
					ct405_enable_als(ct);
			}
		}
	}

	if (ct->suspended && ct->regs_initialized) {
		if (ct405_debug & CT405_DBG_ENABLE_DISABLE)
			pr_info("%s: Powering off\n", __func__);
		ct405_device_power_off(ct);
	}

	return 0;
}


static void ct405_report_prox(struct ct405_data *ct)
{
	int error = 0;
	u8 reg_data[2] = {0};
	unsigned int pdata = 0;

	reg_data[0] = (CT405_PDATA | CT405_COMMAND_AUTO_INCREMENT);
	error = ct405_i2c_read(ct, reg_data, 2);
	if (error < 0) {
		wake_unlock(&ct->wl);
		return;
	}

	pdata = (reg_data[1] << 8) | reg_data[0];
	if (ct405_debug & CT405_DBG_INPUT)
		pr_info("%s: PDATA = %d\n", __func__, pdata);

	switch (ct->prox_mode) {
	case CT405_PROX_MODE_SATURATED:
		pr_warn("%s: prox interrupted in saturated mode!\n", __func__);
		wake_unlock(&ct->wl);
		break;
	case CT405_PROX_MODE_UNCOVERED:
		if (pdata < ct->prox_low_threshold)
			ct405_enable_prox(ct);
		if (pdata > ct->prox_high_threshold) {
			input_event(ct->dev, EV_MSC, MSC_RAW,
					CT405_PROXIMITY_NEAR);
			input_sync(ct->dev);
			ct405_prox_mode_covered(ct);
		}
		break;
	case CT405_PROX_MODE_COVERED:
		if (pdata < ct->prox_low_threshold) {
			input_event(ct->dev, EV_MSC, MSC_RAW,
					CT405_PROXIMITY_FAR);
			input_sync(ct->dev);
			ct405_prox_mode_uncovered(ct);
		}
		break;
	default:
		pr_err("%s: prox mode is %d!\n", __func__, ct->prox_mode);
		wake_unlock(&ct->wl);
	}

	ct405_clear_prox_flag(ct);
}

static void ct405_report_als(struct ct405_data *ct)
{
	int error;
	u8 reg_data[4] = {0};
	unsigned int c0data;
	unsigned int c1data;
	unsigned int ratio;
	unsigned int lux = 0;

	reg_data[0] = (CT405_C0DATA | CT405_COMMAND_AUTO_INCREMENT);
	error = ct405_i2c_read(ct, reg_data, 4);
	if (error < 0)
		return;

	c0data = (reg_data[1] << 8) | reg_data[0];
	c1data = (reg_data[3] << 8) | reg_data[2];
	if (ct405_debug & CT405_DBG_INPUT)
		pr_info("%s: C0DATA = %d, C1DATA = %d\n",
			 __func__, c0data, c1data);

	/* calculate lux using piecewise function from TAOS */
	if (c0data == 0)
		c0data = 1;
	ratio = 100 * c1data / c0data;
	switch (ct->als_mode) {
	case CT405_ALS_MODE_SUNLIGHT:
		if (c0data == 0x0400 || c1data == 0x0400)
			lux = 0xFFFF;
		else if (ratio <= 51)
			lux = (1041*c0data - 1963*c1data);
		else if (ratio <= 58)
			lux = (342*c0data - 587*c1data);
		else
			lux = 0;
		break;
	case CT405_ALS_MODE_LOW_LUX:
		if (c0data == 0x4800 || c1data == 0x4800)
			lux = CT405_ALS_LOW_TO_HIGH_THRESHOLD;
		else if (ratio <= 51)
			lux = (121*c0data - 227*c1data) / 100;
		else if (ratio <= 58)
			lux = (40*c0data - 68*c1data) / 100;
		else
			lux = 0;
		break;
	case CT405_ALS_MODE_HIGH_LUX:
		if (c0data == 0x4800 || c1data == 0x4800)
			lux = 0xFFFF;
		else if (ratio <= 51)
			lux = (964*c0data - 1818*c1data) / 100;
		else if (ratio <= 58)
			lux = (317*c0data - 544*c1data) / 100;
		else
			lux = 0;
		break;
	default:
		pr_err("%s: ALS mode is %d!\n", __func__, ct->als_mode);
	}

	/* input.c filters consecutive LED_MISC values <=1. */
	lux = (lux >= 2) ? lux : 2;

	input_event(ct->dev, EV_LED, LED_MISC, lux);
	input_sync(ct->dev);

	if (ct->als_mode != CT405_ALS_MODE_SUNLIGHT)
		ct405_check_als_range(ct, lux);

	ct405_clear_als_flag(ct);

	if (ct->als_mode == CT405_ALS_MODE_SUNLIGHT)
		ct405_enable_prox(ct); /* re-check saturation */
}

static int ct405_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = ct405_misc_data;

	return 0;
}

static long ct405_misc_ioctl_locked(struct ct405_data *ct,
				    unsigned int cmd,
				    unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 enable;

	if (ct405_debug & CT405_DBG_IOCTL)
		pr_info("%s\n", __func__);

	switch (cmd) {
	case CT405_IOCTL_SET_PROX_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		ct->prox_requested = (enable != 0);
		if (ct->prox_requested)
			ct405_enable_prox(ct);
		else
			ct405_disable_prox(ct);

		break;

	case CT405_IOCTL_GET_PROX_ENABLE:
		enable = ct->prox_requested;
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	case CT405_IOCTL_SET_LIGHT_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		ct->als_requested = (enable != 0);
		if (ct->als_requested)
			ct405_enable_als(ct);
		else
			ct405_disable_als(ct);

		break;

	case CT405_IOCTL_GET_LIGHT_ENABLE:
		enable = ct->als_requested;
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static long ct405_misc_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	struct ct405_data *ct = file->private_data;
	int error;

	if (ct405_debug & CT405_DBG_IOCTL)
		pr_info("%s: cmd = 0x%08X\n", __func__, cmd);

	mutex_lock(&ct->mutex);
	error = ct405_misc_ioctl_locked(ct, cmd, arg);
	mutex_unlock(&ct->mutex);

	return error;
}

static const struct file_operations ct405_misc_fops = {
	.owner = THIS_MODULE,
	.open = ct405_misc_open,
	.unlocked_ioctl = ct405_misc_ioctl,
};

static ssize_t ct405_registers_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ct405_data *ct = i2c_get_clientdata(client);
	int error = 0;
	unsigned int i, n, reg_count;
	u8 reg_data[1] = {0};

	reg_count = sizeof(ct405_regs) / sizeof(ct405_regs[0]);
	mutex_lock(&ct->mutex);
	for (i = 0, n = 0; i < reg_count; i++) {
		reg_data[0] = ct405_regs[i].reg;
		error = ct405_i2c_read(ct, reg_data, 1);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			"%-20s = 0x%02X\n",
			ct405_regs[i].name,
			reg_data[0]);
	}
	mutex_unlock(&ct->mutex);

	return n;
}

static ssize_t ct405_registers_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ct405_data *ct = i2c_get_clientdata(client);
	unsigned int i, reg_count;
	unsigned int value;
	u8 reg_data[2] = {0};
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

	reg_count = sizeof(ct405_regs) / sizeof(ct405_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, ct405_regs[i].name)) {
			mutex_lock(&ct->mutex);
			error = ct405_i2c_write(ct, reg_data, 1);
			mutex_unlock(&ct->mutex);
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

static DEVICE_ATTR(registers, 0644, ct405_registers_show,
		   ct405_registers_store);

static irqreturn_t ct405_irq_handler(int irq, void *dev)
{
	struct ct405_data *ct = dev;

	disable_irq_nosync(ct->client->irq);

	wake_lock_timeout(&ct->wl, HZ);

	queue_work(ct->workqueue, &ct->work);

	return IRQ_HANDLED;
}

static void ct405_work_func_locked(struct ct405_data *ct)
{
	int error;
	u8 reg_data[1] = {0};
	reg_data[0] = CT405_STATUS;
	error = ct405_i2c_read(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: Unable to read interrupt register: %d\n",
			__func__, error);
		wake_unlock(&ct->wl);
		return;
	}

	if (ct->prox_enabled && (reg_data[0] & CT405_STATUS_PINT))
		ct405_report_prox(ct);
	else
		wake_unlock(&ct->wl);

	if (ct->als_enabled && (reg_data[0] & CT405_STATUS_AINT))
		ct405_report_als(ct);

}

static void ct405_work_func(struct work_struct *work)
{
	struct ct405_data *ct =
		container_of(work, struct ct405_data, work);

	mutex_lock(&ct->mutex);
	if (ct->regs_initialized)
		ct405_work_func_locked(ct);
	mutex_unlock(&ct->mutex);

	enable_irq(ct->client->irq);
}

static int ct405_suspend(struct ct405_data *ct)
{
	ct405_disable_als(ct);

	if (!ct->prox_requested)
		ct405_device_power_off(ct);

	ct->suspended = 1;

	return 0;
}

static int ct405_resume(struct ct405_data *ct)
{
	ct405_device_power_on(ct);
	ct405_device_init(ct);

	ct->suspended = 0;

	if (ct->als_requested)
		ct405_enable_als(ct);

	return 0;
}

static int ct405_pm_event(struct notifier_block *this,
			  unsigned long event,
			  void *ptr)
{
	struct ct405_data *ct = container_of(this,
		struct ct405_data, pm_notifier);

	if (ct405_debug & CT405_DBG_SUSPEND_RESUME)
		pr_info("%s: event = %lu\n", __func__, event);

	mutex_lock(&ct->mutex);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		ct405_suspend(ct);
		break;
	case PM_POST_SUSPEND:
		ct405_resume(ct);
		break;
	}

	mutex_unlock(&ct->mutex);

	return NOTIFY_DONE;
}

static int ct405_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct ct405_platform_data *pdata = client->dev.platform_data;
	struct ct405_data *ct;
	u8 reg_data[1] = {0};
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

	ct = kzalloc(sizeof(struct ct405_data), GFP_KERNEL);
	if (ct == NULL) {
		error = -ENOMEM;
		goto error_alloc_data_failed;
	}

	ct->client = client;
	ct->pdata = pdata;

	if (ct->pdata->regulator_name[0] != '\0') {
		ct->regulator = regulator_get(NULL,
			ct->pdata->regulator_name);
		if (IS_ERR(ct->regulator)) {
			pr_err("%s: cannot acquire regulator [%s]\n",
				__func__, ct->pdata->regulator_name);
			error = PTR_ERR(ct->regulator);
			goto error_regulator_get_failed;
		}
	}

	ct->dev = input_allocate_device();
	if (!ct->dev) {
		error = -ENOMEM;
		pr_err("%s: input device allocate failed: %d\n", __func__,
			error);
		goto error_input_allocate_failed;
	}

	ct->dev->name = "light-prox";
	input_set_capability(ct->dev, EV_LED, LED_MISC);
	input_set_capability(ct->dev, EV_MSC, MSC_RAW);

	ct405_misc_data = ct;
	ct->miscdevice.minor = MISC_DYNAMIC_MINOR;
	ct->miscdevice.name = LD_CT405_NAME;
	ct->miscdevice.fops = &ct405_misc_fops;
	error = misc_register(&ct->miscdevice);
	if (error < 0) {
		pr_err("%s: misc_register failed\n", __func__);
		goto error_misc_register_failed;
	}

	ct->pm_notifier.notifier_call = ct405_pm_event;
	error = register_pm_notifier(&ct->pm_notifier);
	if (error < 0) {
		pr_err("%s: register_pm_notifier failed\n", __func__);
		goto error_register_pm_notifier_failed;
	}

	ct->regs_initialized = 0;
	ct->suspended = 0;

	ct->oscillator_enabled = 0;
	ct->prox_requested = 0;
	ct->prox_enabled = 0;
	ct->prox_mode = CT405_PROX_MODE_SATURATED;
	ct->als_requested = 0;
	ct->als_enabled = 0;
	ct->als_mode = CT405_ALS_MODE_LOW_LUX;

	ct->workqueue = create_singlethread_workqueue("als_wq");
	if (!ct->workqueue) {
		pr_err("%s: Cannot create work queue\n", __func__);
		error = -ENOMEM;
		goto error_create_wq_failed;
	}

	INIT_WORK(&ct->work, ct405_work_func);

	error = request_irq(client->irq, ct405_irq_handler,
		IRQF_TRIGGER_LOW, LD_CT405_NAME, ct);
	if (error != 0) {
		pr_err("%s: irq request failed: %d\n", __func__, error);
		error = -ENODEV;
		goto error_req_irq_failed;
	}
	enable_irq_wake(client->irq);
	disable_irq(client->irq);

	i2c_set_clientdata(client, ct);

	mutex_init(&ct->mutex);

	wake_lock_init(&ct->wl, WAKE_LOCK_SUSPEND, "ct405_wake");

	error = input_register_device(ct->dev);
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

	error = ct405_device_power_on(ct);
	if (error) {
		pr_err("%s:power_on failed: %d\n", __func__, error);
		goto error_power_on_failed;
	}

	reg_data[0] = CT405_REV_ID;
	error = ct405_i2c_read(ct, reg_data, 1);
	if (error < 0) {
		pr_err("%s: revision read failed: %d\n", __func__, error);
		goto error_revision_read_failed;
	}
	if (reg_data[0] == CT40X_REV_ID_CT405) {
		pr_info("%s: CT405 part detected\n", __func__);
		ct->prox_saturation_threshold
			= ct->pdata->ct405_prox_saturation_threshold;
		ct->prox_covered_offset = ct->pdata->ct405_prox_covered_offset;
		ct->prox_uncovered_offset
			= ct->pdata->ct405_prox_uncovered_offset;
		ct->prox_recalibrate_offset
			= ct->pdata->ct405_prox_recalibrate_offset;
		ct->pdata_max = CT405_PDATA_MAX;
		ct->hw_type = CT405_HW_TYPE;
	} else {
		pr_info("%s: CT406 part detected\n", __func__);
		ct->prox_saturation_threshold
			= ct->pdata->ct406_prox_saturation_threshold;
		ct->prox_covered_offset = ct->pdata->ct406_prox_covered_offset;
		ct->prox_uncovered_offset
			= ct->pdata->ct406_prox_uncovered_offset;
		ct->prox_recalibrate_offset
			= ct->pdata->ct406_prox_recalibrate_offset;
		ct->pdata_max = CT406_PDATA_MAX;
		ct->hw_type = CT406_HW_TYPE;
	}

	error = ct405_device_init(ct);
	if (error) {
		pr_err("%s:device init failed: %d\n", __func__, error);
		goto error_revision_read_failed;
	}

	return 0;

error_revision_read_failed:
	ct405_device_power_off(ct);
error_power_on_failed:
	device_remove_file(&client->dev, &dev_attr_registers);
error_create_registers_file_failed:
	input_unregister_device(ct->dev);
	ct->dev = NULL;
error_input_register_failed:
	mutex_destroy(&ct->mutex);
	i2c_set_clientdata(client, NULL);
	free_irq(ct->client->irq, ct);
error_req_irq_failed:
	destroy_workqueue(ct->workqueue);
error_create_wq_failed:
	unregister_pm_notifier(&ct->pm_notifier);
error_register_pm_notifier_failed:
	misc_deregister(&ct->miscdevice);
error_misc_register_failed:
	input_free_device(ct->dev);
error_input_allocate_failed:
	regulator_put(ct->regulator);
error_regulator_get_failed:
	kfree(ct);
error_alloc_data_failed:
	return error;
}

static int ct405_remove(struct i2c_client *client)
{
	struct ct405_data *ct = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_registers);

	ct405_device_power_off(ct);

	input_unregister_device(ct->dev);

	wake_lock_destroy(&ct->wl);
	mutex_destroy(&ct->mutex);
	i2c_set_clientdata(client, NULL);
	free_irq(ct->client->irq, ct);

	destroy_workqueue(ct->workqueue);

	unregister_pm_notifier(&ct->pm_notifier);

	misc_deregister(&ct->miscdevice);

	regulator_put(ct->regulator);

	kfree(ct);

	return 0;
}

static const struct i2c_device_id ct405_id[] = {
	{LD_CT405_NAME, 0},
	{}
};

static struct i2c_driver ct405_i2c_driver = {
	.probe		= ct405_probe,
	.remove		= ct405_remove,
	.id_table	= ct405_id,
	.driver = {
		.name = LD_CT405_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ct405_init(void)
{
	return i2c_add_driver(&ct405_i2c_driver);
}

static void __exit ct405_exit(void)
{
	i2c_del_driver(&ct405_i2c_driver);
}

module_init(ct405_init);
module_exit(ct405_exit);

MODULE_DESCRIPTION("ALS and Proximity driver for CT405");
MODULE_AUTHOR("Motorola Mobility");
MODULE_LICENSE("GPL");
