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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/gpio_mapping.h>
#include <linux/fs.h>
#include <linux/slab.h>

#include <linux/lm48901.h>

#define NAME "lm48901"

/*
W_R_L :  It means CH1_SEL : Woofer,  CH2_SEL : Right,    CH3_SEL : Left
L_W_R :  It means CH1_SEL :  Left,      CH2_SEL : Woofer, CH3_SEL : Right
*/
enum {
	W_R_L = 0xC6,
	R_W_L = 0xC9,
	W_L_R = 0xD2,
	L_W_R = 0xD8,
	R_L_W = 0xE1,
	L_R_W = 0xE4
};

/*
I2S_I2S_I2S :  It means OUT1_SEL : I2S,  OUT2_SEL : I2S, OUT3_SEL : I2S
DIS_I2S_I2S :  It means OUT1_SEL : Disabled,  OUT2_SEL : I2S, OUT3_SEL : I2S
OUT4_SEL is always Disabled.
*/
enum {
	DIS_DIS_DIS = 0x00,
	DSP_DIS_DIS = 0x01,
	I2S_DIS_DIS = 0x02,
	DIS_DSP_DIS = 0x04,
	DSP_DSP_DIS = 0x05,
	DIS_I2S_DIS = 0x08,
	I2S_I2S_DIS = 0x0A,
	DIS_DIS_DSP = 0x10,
	DSP_DIS_DSP = 0x11,
	DIS_DSP_DSP = 0x14,
	DSP_DSP_DSP = 0x15,
	DIS_DIS_I2S = 0x20,
	I2S_DIS_I2S = 0x22,
	DIS_I2S_I2S = 0x28,
	I2S_I2S_I2S = 0x2A,
};

#define MAX_BIT_REGISTER 32

#define LM48901_TAB_LEN  0x500

/** Register map */
#define LM48901_FILTER_CONTROL_REG                       0x500
#define LM48901_FILTER_COMP1_REG                         0x501
#define LM48901_FILTER_COMP2_REG                         0x502
#define LM48901_FILTER_DEBUG0_REG                        0x503
#define LM48901_FILTER_DEBUG1_REG                        0x504
#define LM48901_FILTER_STATUS_REG                        0x505
#define LM48901_FILTER_TAP_REG                           0x508
#define LM48901_ACCUML_DEBUG_REG                         0x509
#define LM48901_ACCUMH_DEBUG_REG                         0x50A
#define LM48901_DBG_SAT_REG                              0x50B
#define LM48901_STAT_PCNT1_REG                           0x50C
#define LM48901_STAT_PCNT2_REG                           0x50D
#define LM48901_STAT_ACNT1_REG                           0x50E
#define LM48901_STAT_ACNT2_REG                           0x50F

#define LM48901_DELAY_REG                                0x520
#define LM48901_ENABLE_CLOCKS_REG                        0x521
#define LM48901_DIGITAL_MIXER_REG                        0x522
#define LM48901_ANALOG_REG                               0x523
#define LM48901_I2S_PORT_REG                             0x524
#define LM48901_I2S_PORT2_REG                            0x525
#define LM48901_ADC_TRIM_COEFFICIENT_REG                 0x526
#define LM48901_READBACK_REG                             0x528

#define LM48901_SYS_CONFIG_REG                           0x530
#define LM48901_CL_REG0_REG                              0x531
#define LM48901_CL_REG1_REG                              0x532
#define LM48901_E2_OFFSET_REG                            0x533
#define LM48901_I2C_EnXt_REG                             0x534
#define LM48901_MBIST_STAT_REG                           0x538


/*
 * Register 500 - LM48901_FILTER_CONTROL_REG
 */
#define LM48901_ARRAY_TAP_M                         0x000000FF
#define LM48901_PRE_TAP_M                           0x00007F00
#define LM48901_CH_SEL_M                            0x00FF0000
#define LM48901_PRE_BYPASS_M                        0x10000000
#define LM48901_ARRAY_BYPASS_M                      0x20000000
#define LM48901_PRE_ENABLE_M                        0x40000000
#define LM48901_ARRAY_ENABLE_M                      0x80000000


/*
 * Register 501 - LM48901_FILTER_COMP1_REG
 */
#define LM48901_ARRAY_COMP_SELECT_M                 0x00FF0000

/*
 * Register 502 - LM48901_FILTER_COMP2_REG
 */
#define LM48901_G1_GAIN_0_M                         0x000000E0
#define LM48901_POST_GAIN_0_M                       0x00007000
#define LM48901_G1_GAIN_1_M                         0x00E00000
#define LM48901_POST_GAIN_1_M                       0x70000000

/*
 * Register 504 - LM48901_FILTER_DEBUG1_REG
 */
#define LM48901_DBG_ENABLE_M                        0x00000080


/*
 * Register 521 - LM48901_ENABLE_CLOCKS_REG
 */
#define LM48901_I2S_CLK_M                           0x00000800


/*
 * Register 522 - LM48901_DIGITAL_MIXER_REG
 */
#define LM48901_MUTE_M                              0x00000040
#define LM48901_I2S_LVL_M                           0x00003F00
#define LM48901_I2S_DSP_M                           0x00010000
#define LM48901_OUT_SEL_M                           0xFF000000

/*
 * Register 524 - LM48901_I2S_PORT_REG
 */
#define LM48901_STEREO_M                            0x00000001
#define LM48901_RX_ENABLE_M                         0x00000002

/*
 * Register 525 - LM48901_I2S_PORT2_REG
 */
#define LM48901_RX_WIDTH_M                          0x00000007

/*
 * Register 530 - LM48901_SYS_CONFIG_REG
 */
#define LM48901_CL_ENABLE_M                         0x00800000


#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5

struct lm48901_data {
	struct i2c_client *client;
	struct lm48901_platform_data *pdata;
	struct mutex lock; /* used for all functions */
};

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct lm48901_data *lm48901_misc_data;

static char get_idx_from_mask(unsigned int mask)
{
	char idx = 0;

	while (!(mask & 0x1)) {
		mask >>= 1;
		idx++;

		if (idx == MAX_BIT_REGISTER)
			return -1;
	}

	return idx;
}

static int lm48901_i2c_read(struct lm48901_data *lm48901,
				unsigned short reg, unsigned int *value)
{
	int err;
	int tries = 0;

	u8 addr[2] = { 0, 0 };
	u8 buf[4] = { 0, 0, 0, 0 };

	struct i2c_msg msgs[] = {
		{
		 .addr = lm48901->client->addr,
		 .flags = lm48901->client->flags & I2C_M_TEN,
		 .len = 2,
		 .buf = addr,
		 },
		{
		 .addr = lm48901->client->addr,
		 .flags = (lm48901->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = 4,
		 .buf = buf,
		 },
	};

	addr[0] = (reg >> 8);
	addr[1] = (reg & 0xff);

	do {
		err = i2c_transfer(lm48901->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&lm48901->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	*value = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

	return err;
}

static int lm48901_i2c_write(struct lm48901_data *lm48901,
				unsigned short reg, unsigned int value)
{
	int err;
	int tries = 0;
	u8 buf[6] = { 0, 0, 0, 0, 0, 0 };

	struct i2c_msg msgs[] = {
		{
		 .addr = lm48901->client->addr,
		 .flags = lm48901->client->flags & I2C_M_TEN,
		 .len = 6,
		 .buf = buf,
		 },
	};

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = value & 0xff;
	buf[3] = (value >> 8) & 0xff;
	buf[4] = (value >> 16) & 0xff;
	buf[5] = (value >> 24) & 0xff;

	do {
		err = i2c_transfer(lm48901->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&lm48901->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lm48901_reg_read(struct lm48901_data *lm48901,
			unsigned short reg,
			unsigned int *value)
{
	int retval = -EINVAL;

	mutex_lock(&lm48901->lock);

	retval = lm48901_i2c_read(lm48901, reg , value);

	mutex_unlock(&lm48901->lock);

	return retval;
}

static int lm48901_reg_write(struct lm48901_data *lm48901,
			unsigned short reg,
			unsigned int value,
		       unsigned int mask)
{
	int retval = -EINVAL;
	unsigned int old_value = 0;

	mutex_lock(&lm48901->lock);

	value &= mask;

	retval = lm48901_i2c_read(lm48901, reg , &old_value);

	pr_debug("Old value = 0x%08X\n", old_value);

	if (retval != 0)
		goto error;

	old_value &= ~mask;
	value |= old_value;

	pr_debug("New value = 0x%08X\n", value);

	retval = lm48901_i2c_write(lm48901, reg, value);

error:

	mutex_unlock(&lm48901->lock);

	return retval;
}

static int lm48901_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = lm48901_misc_data;

	pr_debug("%s:Enter\n", __func__);

	return 0;
}

static long lm48901_misc_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = 0;

	struct lm48901_data *lm48901 = file->private_data;

	switch (cmd) {
	case LM48901_IOCTL_AMP_DISABLE:
		pr_debug("Disabled a lm48901_amp_en\n");
		if (lm48901->pdata->amp_en_gpio != -1)
			gpio_set_value(lm48901->pdata->amp_en_gpio, 0);
		break;

	case LM48901_IOCTL_AMP_ENABLE:
		pr_debug("Enabled a lm48901_amp_en\n");
		if (lm48901->pdata->amp_en_gpio != -1)
			gpio_set_value(lm48901->pdata->amp_en_gpio, 1);
		break;

	case LM48901_IOCTL_INIT:
		{
		int i = 0;
		unsigned int value = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("mode in lm48901 = %d\n", value);

		lm48901->pdata->mode = value;

		gpio_set_value(lm48901->pdata->i2c_en_gpio, 1);

		/*I2C slave mode*/
		lm48901_reg_write(lm48901, LM48901_SYS_CONFIG_REG,
			0x00000000, LM48901_CL_ENABLE_M);

		/*I2S clk input to PLL*/
		lm48901_reg_write(lm48901, LM48901_ENABLE_CLOCKS_REG,
			LM48901_I2S_CLK_M, LM48901_I2S_CLK_M);

		/*To have two volume control*/
		lm48901_reg_write(lm48901, LM48901_FILTER_COMP1_REG,
			0x00040000, LM48901_ARRAY_COMP_SELECT_M);

		/*G1_GAIN_0 : 256*/
		lm48901_reg_write(lm48901, LM48901_FILTER_COMP2_REG,
			0x000000E0, LM48901_G1_GAIN_0_M);
		/*POST_GAIN_0 : 3 */
		lm48901_reg_write(lm48901, LM48901_FILTER_COMP2_REG,
			0x00005000, LM48901_POST_GAIN_0_M);

		/*G1_GAIN_1 : 256*/
		lm48901_reg_write(lm48901, LM48901_FILTER_COMP2_REG,
			0x00E00000, LM48901_G1_GAIN_1_M);
		/*POST_GAIN_1 : 2 */
		lm48901_reg_write(lm48901, LM48901_FILTER_COMP2_REG,
			0x30000000, LM48901_POST_GAIN_1_M);

		/*init to 0x0 on LM48901_DIGITAL_MIXER_REG*/
		lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
			0x00000000, 0xFFFFFFFF);
		/*-3dB I2S level*/
		lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
			0x00003100, LM48901_I2S_LVL_M);
		/*I2S Data Passed to DSP*/
		lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
			LM48901_I2S_DSP_M, LM48901_I2S_DSP_M);

		/*OUT_SEL*/
		if (value == LM48901_WOOFER) {
			/*OUT1,2,3 : DSP, OUT4 : Disabled*/
			lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
				0x15000000, LM48901_OUT_SEL_M);
		} else {
			/*OUT1,2 : DSP, OUT3,4 : Disabled*/
			lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
				0x05000000, LM48901_OUT_SEL_M);
		}

		/*Set if using the PreFilter or the Array Filter*/
		lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
			LM48901_PRE_BYPASS_M, LM48901_PRE_BYPASS_M);
		lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
			0x00000000, LM48901_ARRAY_BYPASS_M);
		lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
			0x00000000, LM48901_PRE_ENABLE_M);
		lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
			LM48901_ARRAY_ENABLE_M, LM48901_ARRAY_ENABLE_M);

		/*STEREO to 1*/
		lm48901_reg_write(lm48901, LM48901_I2S_PORT_REG,
			LM48901_STEREO_M, LM48901_STEREO_M);

		/*RX_ENABLE to 1*/
		lm48901_reg_write(lm48901, LM48901_I2S_PORT_REG,
			LM48901_RX_ENABLE_M, LM48901_RX_ENABLE_M);

		/*RX_WIDTH to 16 bits*/
		lm48901_reg_write(lm48901, LM48901_I2S_PORT2_REG,
			0x00000003, LM48901_RX_WIDTH_M);

		/*load coef*/
		lm48901_reg_write(lm48901, LM48901_FILTER_DEBUG1_REG,
			LM48901_DBG_ENABLE_M, LM48901_DBG_ENABLE_M);

		for (i = 0; i < LM48901_TAB_LEN; i++) {
			lm48901_reg_write(lm48901, i,
				lm48901_tab_2p1_3ch[i], 0xFFFFFFFF);
		}

		lm48901_reg_write(lm48901, LM48901_FILTER_DEBUG1_REG,
			~LM48901_DBG_ENABLE_M, LM48901_DBG_ENABLE_M);

		}
		break;

	case LM48901_IOCTL_SET_MUTE:
		{
		unsigned int value = 0;
		char idx = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("mute = %d\n", value);

		idx = get_idx_from_mask(LM48901_MUTE_M);

		value <<= idx;

		pr_debug("idx = %d , value = %d\n", idx, value);

		lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
			value, LM48901_MUTE_M);
		}
		break;

	case LM48901_IOCTL_SET_I2S_LVL:
		{
		unsigned int value = 0;
		char idx = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("i2s level = %d\n", value);

		idx = get_idx_from_mask(LM48901_I2S_LVL_M);

		value <<= idx;

		pr_debug("idx = %d , value = %d\n", idx, value);

		lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
			value, LM48901_I2S_LVL_M);
		}
		break;

	case LM48901_IOCTL_SET_ORIENTATION:
		{
		unsigned int value = 0;
		char idx = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("orientation  = %d\n", value);

		lm48901->pdata->orientation = value;

		switch (value) {
		case LM48901_ORIENTATION_A:
			value = L_R_W;
			break;

		case LM48901_ORIENTATION_B:
			value = R_L_W;
			break;

		case LM48901_ORIENTATION_C:
			value = L_W_R;
			break;

		case LM48901_ORIENTATION_D:
			value = R_W_L;
			break;

		default:
			value = L_R_W;
			break;
		}

		idx = get_idx_from_mask(LM48901_CH_SEL_M);

		value <<= idx;

		lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
			value, LM48901_CH_SEL_M);

		if (lm48901->pdata->mode == LM48901_STEREO) {
			switch (lm48901->pdata->orientation) {
			case LM48901_ORIENTATION_A:
				value = DSP_DSP_DIS;
				break;

			case LM48901_ORIENTATION_B:
				value = DSP_DSP_DIS;
				break;

			case LM48901_ORIENTATION_C:
				value = DSP_DIS_DSP;
				break;

			case LM48901_ORIENTATION_D:
				value = DSP_DIS_DSP;
				break;

			default:
				value = DSP_DSP_DIS;
				break;
			}

			idx = get_idx_from_mask(LM48901_OUT_SEL_M);

			value <<= idx;

			lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
				value, LM48901_OUT_SEL_M);
		}

		}
		break;

	case LM48901_IOCTL_SET_MODE:
		{
		unsigned int value = 0;
		char idx = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("mode = %d\n", value);

		lm48901->pdata->mode = value;

		if (value == LM48901_WOOFER) {
			value = DSP_DSP_DSP;
		} else {
			switch (lm48901->pdata->orientation) {
			case LM48901_ORIENTATION_A:
				value = DSP_DSP_DIS;
				break;

			case LM48901_ORIENTATION_B:
				value = DSP_DSP_DIS;
				break;

			case LM48901_ORIENTATION_C:
				value = DSP_DIS_DSP;
				break;

			case LM48901_ORIENTATION_D:
				value = DSP_DIS_DSP;
				break;

			default:
				value = DSP_DSP_DIS;
				break;
			}
		}

		idx = get_idx_from_mask(LM48901_OUT_SEL_M);

		value <<= idx;

		lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
			value, LM48901_OUT_SEL_M);
		}
		break;

	case LM48901_IOCTL_GET_REG:
		{
		struct lm48901_regacc reg_access = {0 , 0};

		if (copy_from_user(&reg_access,
			(struct lm48901_regacc *)argp, sizeof(reg_access)))
			return -EFAULT;

		if (lm48901_reg_read(lm48901, reg_access.reg,
				&reg_access.value))
			return -EFAULT;

		pr_debug("reg = 0x%X, value = 0x%08X\n",
			reg_access.reg, reg_access.value);

		if (copy_to_user(argp, &reg_access, sizeof(reg_access)))
			return -EFAULT;
		}
		break;

	case LM48901_IOCTL_SET_REG:
		{
		struct lm48901_regacc reg_access = {0 , 0};

		if (copy_from_user(&reg_access, argp, sizeof(reg_access)))
			return -EFAULT;

		pr_debug("reg = 0x%X, value = 0x%08X\n",
			reg_access.reg, reg_access.value);

		if (lm48901_reg_write(lm48901, reg_access.reg,
				reg_access.value, 0xFFFFFFFF))
			return -EFAULT;
		}
		break;

	case LM48901_IOCTL_I2C_DISABLE:
		pr_debug("Disabled a lm48901_i2c_en\n");
		if (lm48901->pdata->i2c_en_gpio != -1)
			gpio_set_value(lm48901->pdata->i2c_en_gpio, 0);
		break;

	case LM48901_IOCTL_I2C_ENABLE:
		pr_debug("Enabled a lm48901_i2c_en\n");
		if (lm48901->pdata->i2c_en_gpio != -1)
			gpio_set_value(lm48901->pdata->i2c_en_gpio, 1);
		break;

	case LM48901_IOCTL_SET_OUT_SEL:
		{
		unsigned int value = 0;
		char idx = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("out_sel  = 0x%X\n", value);

		idx = get_idx_from_mask(LM48901_OUT_SEL_M);

		value <<= idx;

		pr_debug("idx = %d , value = 0x%08X\n", idx, value);

		lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
			value, LM48901_OUT_SEL_M);
		}
		break;

	case LM48901_IOCTL_SET_CH_SEL:
		{
		unsigned int value = 0;
		char idx = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("ch_sel  = 0x%X\n", value);

		idx = get_idx_from_mask(LM48901_CH_SEL_M);

		value <<= idx;

		pr_debug("idx = %d , value = 0x%08X\n", idx, value);

		lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
			value, LM48901_CH_SEL_M);
		}
		break;

	case LM48901_IOCTL_LOAD_COEF:
		{
		unsigned int value = 0;
		const unsigned int *lm48901_coef = NULL;
		int i = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("load  = %d\n", value);

		switch (value) {
		case LM48901_STEREO:
			lm48901_coef = lm48901_tab_stereo_2ch;
			break;

		case LM48901_WOOFER:
			lm48901_coef = lm48901_tab_2p1_3ch;
			break;

		default:
			lm48901_coef = lm48901_tab_2p1_3ch;
			break;
		}

		if (lm48901_coef != NULL) {
			lm48901_reg_write(lm48901, LM48901_FILTER_DEBUG1_REG,
				LM48901_DBG_ENABLE_M, LM48901_DBG_ENABLE_M);

			for (i = 0; i < LM48901_TAB_LEN; i++) {
				lm48901_reg_write(lm48901, i,
					lm48901_coef[i], 0xFFFFFFFF);
			}

			lm48901_reg_write(lm48901, LM48901_FILTER_DEBUG1_REG,
				~LM48901_DBG_ENABLE_M, LM48901_DBG_ENABLE_M);
		}

		}
		break;

	case LM48901_IOCTL_SET_G1_GAIN_0:
		{
		unsigned int value = 0;
		char idx = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("G1_GAIN_0  level = %d\n", value);

		idx = get_idx_from_mask(LM48901_G1_GAIN_0_M);

		value <<= idx;

		pr_debug("idx = %d , value = %X\n", idx, value);

		lm48901_reg_write(lm48901, LM48901_FILTER_COMP2_REG,
			value, LM48901_G1_GAIN_0_M);
		}
		break;

	case LM48901_IOCTL_SET_POST_GAIN_0:
		{
		unsigned int value = 0;
		char idx = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("POST_GAIN_0  level = %d\n", value);

		idx = get_idx_from_mask(LM48901_POST_GAIN_0_M);

		value <<= idx;

		pr_debug("idx = %d , value = %X\n", idx, value);

		lm48901_reg_write(lm48901, LM48901_FILTER_COMP2_REG,
			value, LM48901_POST_GAIN_0_M);
		}
		break;

	case LM48901_IOCTL_SET_G1_GAIN_1:
		{
		unsigned int value = 0;
		char idx = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("G1_GAIN_1  level = %d\n", value);

		idx = get_idx_from_mask(LM48901_G1_GAIN_1_M);

		value <<= idx;

		pr_debug("idx = %d , value = %X\n", idx, value);

		lm48901_reg_write(lm48901, LM48901_FILTER_COMP2_REG,
			value, LM48901_G1_GAIN_1_M);
		}
		break;

	case LM48901_IOCTL_SET_POST_GAIN_1:
		{
		unsigned int value = 0;
		char idx = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("POST_GAIN_1  level = %d\n", value);

		idx = get_idx_from_mask(LM48901_POST_GAIN_1_M);

		value <<= idx;

		pr_debug("idx = %d , value = %X\n", idx, value);

		lm48901_reg_write(lm48901, LM48901_FILTER_COMP2_REG,
			value, LM48901_POST_GAIN_1_M);
		}
		break;

	case LM48901_IOCTL_USE_DSP:
		{
		unsigned int value = 0;

		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		pr_debug("use DSP  = %d\n", value);

		if (value) {
			lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
				LM48901_PRE_BYPASS_M, LM48901_PRE_BYPASS_M);
			lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
				0x00000000, LM48901_ARRAY_BYPASS_M);
			lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
				0x00000000, LM48901_PRE_ENABLE_M);
			lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
				LM48901_ARRAY_ENABLE_M, LM48901_ARRAY_ENABLE_M);

			/*I2S Data Passed to DSP*/
			lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
				LM48901_I2S_DSP_M, LM48901_I2S_DSP_M);
			/*out1,2,3:DSP, out4:disabled*/
			lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
				0x15000000, LM48901_OUT_SEL_M);
		} else {
			lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
				LM48901_PRE_BYPASS_M, LM48901_PRE_BYPASS_M);
			lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
				LM48901_ARRAY_BYPASS_M, LM48901_ARRAY_BYPASS_M);
			lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
				0x00000000, LM48901_PRE_ENABLE_M);
			lm48901_reg_write(lm48901, LM48901_FILTER_CONTROL_REG,
				0x00000000, LM48901_ARRAY_ENABLE_M);

			/*I2S Data Not Passed to DSP*/
			lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
				0x00000000, LM48901_I2S_DSP_M);
			/*out1,2,3:I2S, out4:disabled*/
			lm48901_reg_write(lm48901, LM48901_DIGITAL_MIXER_REG,
				0x2A000000, LM48901_OUT_SEL_M);
		}

		}
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static const struct file_operations lm48901_misc_fops = {
	.owner = THIS_MODULE,
	.open = lm48901_misc_open,
	.unlocked_ioctl = lm48901_misc_ioctl,
};

static struct miscdevice lm48901_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &lm48901_misc_fops,
};

static int __init lm48901_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lm48901_data *lm48901;
	int err = 0;

	pr_debug("%s:Enter\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}

	lm48901 = kzalloc(sizeof(*lm48901), GFP_KERNEL);
	if (lm48901 == NULL) {
		dev_err(&client->dev,
			"Failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&lm48901->lock);
	lm48901->client = client;

	i2c_set_clientdata(client, lm48901);

	lm48901->pdata = kzalloc(sizeof(*lm48901->pdata), GFP_KERNEL);
	if (lm48901->pdata == NULL)
		goto err1;

	memcpy(lm48901->pdata,
			client->dev.platform_data, sizeof(*lm48901->pdata));

	lm48901->pdata->orientation = LM48901_ORIENTATION_A;
	lm48901->pdata->mode = LM48901_WOOFER;

	lm48901_misc_data = lm48901;

	pr_debug("I2C addr for lm48901 = 0x%x\n",
			lm48901_misc_data->client->addr);
	pr_debug("lm48901_amp_en = %d\n",
			lm48901_misc_data->pdata->amp_en_gpio);
	pr_debug("lm48901_i2c_en = %d\n",
			lm48901_misc_data->pdata->i2c_en_gpio);

	err = misc_register(&lm48901_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "Failed to register a lm48901 device\n");
		goto err2;
	}

	pr_debug("lm48901 probed\n");

	return 0;

err2:
	kfree(lm48901->pdata);
err1:
	mutex_destroy(&lm48901->lock);
	kfree(lm48901);
err0:
	return err;
}

static int __devexit lm48901_remove(struct i2c_client *client)
{
	struct lm48901_data *lm48901 = i2c_get_clientdata(client);

	misc_deregister(&lm48901_misc_device);
	mutex_destroy(&lm48901->lock);
	kfree(lm48901->pdata);
	kfree(lm48901);

	return 0;
}

static const struct i2c_device_id lm48901_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lm48901_id);

static struct i2c_driver lm48901_driver = {
	.driver = {
			.name = NAME,
			.owner = THIS_MODULE,
	},
	.probe = lm48901_probe,
	.remove = __devexit_p(lm48901_remove),
	.id_table = lm48901_id,
};

static int __init lm48901_init(void)
{
	return i2c_add_driver(&lm48901_driver);
}

static void __exit lm48901_exit(void)
{
	i2c_del_driver(&lm48901_driver);
	return;
}

module_init(lm48901_init);
module_exit(lm48901_exit);

MODULE_DESCRIPTION("LM48901 driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Motorola Mobility");
