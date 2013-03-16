/*
 * Core Source for:
 * Cypress CY8C201XX driver.
 * For use with Cypress CY8C201XX CapSense(R) parts.
 * Supported parts include:
 * CY8C201A0x
 *
 * Copyright (C) 2009-2011 Cypress Semiconductor, Inc.
 * Copyright (C) 2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */

#include "cy8c201xx_core.h"

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input/touch_platform.h>
#include <linux/version.h>	/* Required for kernel version checking */
#include <linux/firmware.h>	/* This enables firmware class loader code */

#define CY_DELAY_DFLT			20 /* ms */
#define CY_DELAY_NORMAL			11 /* ms */
#define CY_DELAY_SAVING			120 /* ms */
#define CY_DELAY_RECONFIG		200 /* ms */
#define CY_DELAY_DEBOUNCE		35 /* ms */

#define CY_MAX_PRBUF_SIZE		PIPE_BUF
#define CY_OPEN_DRAIN_BIT		0x80
#define CY_DEEP_SLEEP_BIT		0x01
#define CY_RESET_BASELINE_BIT		0x80
#define CY_CAPSENSE_FILTER_REG		0x56
#define CY_DEVICE_INFO_REG		0x7A
#define CY_I2C_ADDR_DM			0x7C
#define CY_SLEEP_CTRL			0x7F
#define CY_SLEEP_ENABLE			0x80	/* sleep enable bit */
#define CY_RW_REGID_MAX			0xFF
#define CY_RW_REG_DATA_MAX		0xFF

enum cy201_sett_flags {
	CY_USE_SLEEP = 0x01,
	CY_USE_LOW_IRQ_PULSE = 0x02,
	CY_USE_LATCHED = 0x04,
	CY_USE_INTERNAL_PU = 0x08,	/* pullup vs. open drain */
	CY_USE_SLEEP_PIN = 0x10,	/* use Sleep Pin to control sleep */
	CY_USE_RESET_PIN = 0x20,	/* use XRES to "wake" on resume */
};

enum cy201_ic_grpnum {
	CY_IC_GRPNUM_RESERVED = 0,
	CY_IC_GRPNUM_BUTTON_PRESS,
	CY_IC_GRPNUM_PORT_CONFIG,
	CY_IC_GRPNUM_BUTTON_CONFIG,
	CY_IC_GRPNUM_SCAN_SEQUENCE,
	CY_IC_GRPNUM_TUNE_THRESHOLD,
	CY_IC_GRPNUM_SLIDER_CONFIG,
	CY_IC_GRPNUM_SLEEP_CTRL,
	CY_IC_GRPNUM_NUM	/* always last */
};

enum cy201_powerstate {
	CY_IDLE_STATE,		/* IC cannot be reached */
	CY_READY_STATE,		/* pre-operational; ready to go to ACTIVE */
	CY_ACTIVE_STATE,	/* app is running, IC is scanning */
	CY_LOW_PWR_STATE,	/* not currently used  */
	CY_SLEEP_STATE,		/* app is running, IC is idle */
	CY_BL_STATE,		/* bootloader is running */
	CY_LDR_STATE,		/* loader is running */
	CY_SYSINFO_STATE,	/* switching to sysinfo mode */
	CY_INVALID_STATE	/* always last in the list */
};

static char *cy201_powerstate_string[] = {
	/* Order must match enum cy201_powerstate above */
	"IDLE",
	"READY",
	"ACTIVE",
	"LOW_PWR",
	"SLEEP",
	"BOOTLOADER",
	"LOADER",
	"SYSINFO",
	"INVALID"
};

enum CY_CMD {
	CY_CMD_GET_FW_REV =	0x00,
	CY_CMD_SAVE_SETTINGS =	0x01,
	CY_CMD_RESTORE_FACT =	0x02,
	CY_CMD_WR_POR_DFLT =	0x03,
	CY_CMD_RD_POR_DFLT =	0x04,
	CY_CMD_RD_CONFIG =	0x05,
	CY_CMD_RECONFIG =	0x06,		/* soft reset */
	CY_CMD_SET_NORMAL =	0x07,
	CY_CMD_ENTER_SETUP =	0x08,
	CY_CMD_START_SCAN =	0x09,
	CY_CMD_STOP_SCAN =	0x0A,
	CY_CMD_SCAN_STATUS =	0X0B,
};

struct cy201_cmd {
	u8 reg;
	int len;
	u8 *dat;
	char *nam;
	int dly;
};

struct cy201_pdata {
	u32 nbr;
	u32 addr;
	struct cy201_cmd *p_cfg;
};

static const u8 unlock_i2c[] = {
	0x3C, 0xA5, 0x69
};
static const struct cy201_cmd cy201_unlock_i2c = {
	.reg = 0x79,
	.len = sizeof(unlock_i2c),
	.dat = (u8 *)unlock_i2c,
	.nam = "unlock_i2c",
	.dly = CY_DELAY_NORMAL,
};

static u8 i2c_set[] = {
	0
};
static const struct cy201_cmd cy201_set_i2c = {
	.reg = CY_I2C_ADDR_DM,
	.len = sizeof(i2c_set),
	.dat = (u8 *)i2c_set,
	.nam = "set_i2c",
	.dly = CY_DELAY_NORMAL,
};

static const u8 lock_i2c[] = {
	0x96, 0x5A, 0xC3
};
static const struct cy201_cmd cy201_lock_i2c = {
	.reg = 0x79,
	.len = sizeof(lock_i2c),
	.dat = (u8 *)lock_i2c,
	.nam = "lock_i2c",
	.dly = CY_DELAY_NORMAL,
};

static const u8 restore_factory_settings[] = {
	CY_CMD_RESTORE_FACT
};
static const struct cy201_cmd cy201_restore_factory_settings = {
	.reg = 0xA0,
	.len = sizeof(restore_factory_settings),
	.dat = (u8 *)restore_factory_settings,
	.nam = "restore_factory_settings",
	.dly = CY_DELAY_SAVING,
};

static const u8 enter_setup[] = {
	CY_CMD_ENTER_SETUP
};
static const struct cy201_cmd cy201_enter_setup = {
	.reg = 0xA0,
	.len = sizeof(enter_setup),
	.dat = (u8 *)enter_setup,
	.nam = "enter_setup",
	.dly = CY_DELAY_NORMAL,
};

/*
 * TODO:  Update structure and string names
 *
 * Please note that settings descriptions, their names,
 *   and other associated commands may be inconsistent
 *   with their actual function at this point.
 */

/* Clear output */
static u8 clr_out_ports[] = {
	0x00, 0x08
};
static struct cy201_cmd cy201_clr_out_ports = {
	.reg = 0x04,
	.len = sizeof(clr_out_ports),
	.dat = (u8 *)clr_out_ports,
	.nam = "clr_out_ports",
	.dly = CY_DELAY_NORMAL,
};

/* Port configs */
static struct cy201_cmd cy201_port_configs = {
	.reg = 0x06,
	.len = 0,
	.dat = NULL,
	.nam = "port_configs",
	.dly = CY_DELAY_NORMAL
};

/* Button config and debounce */
static struct cy201_cmd cy201_debounce_settings = {
	.reg = 0x4E,
	.len = 0,
	.dat = NULL,
	.nam = "debounce_settings",
	.dly = CY_DELAY_DEBOUNCE,
};

/* Scan positions */
static struct cy201_cmd cy201_scan_positions = {
	.reg = 0x57,
	.len = 0,
	.dat = NULL,
	.nam = "scan_positions",
	.dly = CY_DELAY_NORMAL,
};

/* Tuning thresholds */
static struct cy201_cmd cy201_tune_thresholds = {
	.reg = 0x61,
	.len = 0,
	.dat = NULL,
	.nam = "tune_thresholds",
	.dly = CY_DELAY_NORMAL,
};

/* Slider config */
static struct cy201_cmd cy201_slider_config = {
	.reg = 0x75,
	.len = 0,
	.dat = NULL,
	.nam = "slider_config",
	.dly = CY_DELAY_NORMAL,
};

/* Sleep control */
static struct cy201_cmd cy201_sleep_settings = {
	.reg = 0x7E,
	.len = 0,
	.dat = NULL,
	.nam = "sleep_settings",
	.dly = CY_DELAY_NORMAL,
};

static u8 rebaseline[] = {
	0x80
};
static struct cy201_cmd cy201_rebaseline = {
	.reg = CY_CAPSENSE_FILTER_REG,
	.len = sizeof(rebaseline),
	.dat = rebaseline,
	.nam = "rebaseline",
	.dly = CY_DELAY_NORMAL,
};

static const u8 save_settings[] = {
	CY_CMD_SAVE_SETTINGS
};
static const struct cy201_cmd cy201_save_settings = {
	.reg = 0xA0,
	.len = sizeof(save_settings),
	.dat = (u8 *)save_settings,
	.nam = "save_settings",
	.dly = CY_DELAY_SAVING,
};

static const u8 reconfig_to_POR[] = {
	CY_CMD_RECONFIG
};
static const struct cy201_cmd cy201_reconfig_to_POR = {
	.reg = 0xA0,
	.len = sizeof(reconfig_to_POR),
	.dat = (u8 *)reconfig_to_POR,
	.nam = "reconfig_to_POR",
	.dly = CY_DELAY_RECONFIG,
};

static const struct cy201_cmd *cy201_init_cmds[] = {
/*
 * comment out until structure format get fixed
 *
 *	&cy201_unlock_i2c,
 *	&cy201_set_i2c,
 *	&cy201_lock_i2c,
 */
	&cy201_restore_factory_settings,
	&cy201_reconfig_to_POR,
	&cy201_enter_setup,
	&cy201_port_configs,
	&cy201_debounce_settings,
	&cy201_slider_config,
	&cy201_scan_positions,
	&cy201_tune_thresholds,
	&cy201_sleep_settings,
	NULL,
};
static const struct cy201_cmd *cy201_save_cmds[] = {
	&cy201_save_settings,
	&cy201_reconfig_to_POR,
	NULL,
};

static const struct cy201_pdata cy201_pdata_tbl[] = {
	{
		.nbr = CY_IC_GRPNUM_PORT_CONFIG,
		.addr = 0x04,
	},
	{
		.nbr = CY_IC_GRPNUM_BUTTON_CONFIG,
		.addr = 0x4E,
		.p_cfg = &cy201_debounce_settings,
	},
	{
		.nbr = CY_IC_GRPNUM_SCAN_SEQUENCE,
		.addr = 0x57,
		.p_cfg = &cy201_scan_positions,
	},
	{
		.nbr = CY_IC_GRPNUM_TUNE_THRESHOLD,
		.addr = 0x61,
		.p_cfg = &cy201_tune_thresholds,
	},
	{
		.nbr = CY_IC_GRPNUM_SLIDER_CONFIG,
		.addr = 0x75,
		.p_cfg = &cy201_slider_config,
	},
	{
		.nbr = CY_IC_GRPNUM_SLEEP_CTRL,
		.addr = 0x7E,
		.p_cfg = &cy201_sleep_settings,
	}
};

struct cy201_input_ports {
	u8 input_port0;
	u8 input_port1;
	u8 status_port0;
	u8 status_port1;
};

struct cy201_output_ports {
	u8 output_port0;
	u8 output_port1;
};

#define CY_NUM_INPUT_PORT	2
#define CY_NUM_INPUT_PER_PORT	5
#define CY_NUM_BUTTON		(CY_NUM_INPUT_PER_PORT * CY_NUM_INPUT_PORT)

#ifdef CONFIG_TOUCHSCREEN_DEBUG
char *cy201_key_name[] = {
	"MENU",
	"HOME",
	"BACK",
	"SEARCH",
	"RESERVED1",
	"RESERVED2",
	"RESERVED3",
	"RESERVED4",
	"RESERVED5",
	"RESERVED6"
};
#define cy201_pr_key(ts, l, b) {\
	cy201_dbg(ts, l, "%s: %s\n", __func__, cy201_key_name[b]); \
}
#else
#define cy201_pr_key(ts, l, b)
#endif

struct cy201_button {
	int key;
	u16 bits;
	bool down;
};

struct cy201 {
	struct device *dev;
	int irq;
	struct input_dev *input;
	struct mutex mutex;	/* Used to prevent concurrent accesses */
	struct mutex startup_mutex; /* protect power on sequence */
	char phys[32];
	const struct bus_type *bus_type;
	const struct touch_platform_data *platform_data;
	struct cy201_input_ports input_ports;
	struct cy201_button button[CY_NUM_BUTTON];
	struct cy201_bus_ops *bus_ops;
	struct completion int_running;
	enum cy201_powerstate power_state;
	bool irq_enabled;
	uint16_t button_map;	/* Store previous state for polling */
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	int ic_grpnum;
	int ic_grpoffset;
#endif
	bool resuming;	/* Workaround I2C lockup issues */
};

static int cy201_hw_wake(struct cy201 *ts, int wake)
{
	if ((ts->platform_data->flags & CY_USE_SLEEP_PIN) &&
		(ts->platform_data->hw_recov))
		return ts->platform_data->hw_recov(wake);
	else
		return 0;
}

static int cy201_read_block_data(struct cy201 *ts, u8 command,
	u8 length, void *buf, int i2c_addr)
{
	int retval;
	int tries;

	if (!buf || !length)
		return -EIO;

	cy201_hw_wake(ts, 0);
	for (tries = 0, retval = -1;
		tries < CY_NUM_RETRY && (retval < 0);
		tries++) {
		retval = ts->bus_ops->read(ts->bus_ops, command,
			length, buf, i2c_addr);
		if (retval) {
			pr_warning("%s: try to read I2C bus (tries =%d)\n",
				__func__, tries);
			if ((retval == -ETIMEDOUT) && (ts->resuming)) {
				pr_err("%s: Timeout error during resume\n",
					__func__);
				goto cy201_read_block_data_fail;
			}
			msleep(CY_DELAY_DFLT);
		}
	}

cy201_read_block_data_fail:
	cy201_hw_wake(ts, 1);

	if (retval < 0) {
		pr_err("%s: I2C read block data fail (ret=%d)\n",
			__func__, retval);
	}
	return retval;
}

static int cy201_write_block_data(struct cy201 *ts, u8 command,
	u8 length, const void *buf, int i2c_addr)
{
	int retval;
	int tries;

	if (!buf || !length)
		return -EIO;

	cy201_hw_wake(ts, 0);
	for (tries = 0, retval = -1;
		tries < CY_NUM_RETRY && (retval < 0);
		tries++) {
		retval = ts->bus_ops->write(ts->bus_ops, command,
			length, buf, i2c_addr);
		if (retval) {
			pr_warning("%s: try to write I2C bus (tries =%d)\n",
				__func__, tries);
			if ((retval == -ETIMEDOUT) && (ts->resuming)) {
				pr_err("%s: Timeout error during resume\n",
					__func__);
				goto cy201_write_block_data_fail;
			}
			msleep(CY_DELAY_DFLT);
		}
	}

cy201_write_block_data_fail:
	cy201_hw_wake(ts, 1);

	if (retval < 0) {
		pr_err("%s: I2C write block data fail (ret=%d)\n",
			__func__, retval);
	}
	return retval;
}

static int cy201_wr_cmd(struct cy201 *ts,
	const  struct cy201_cmd *cmd, int i2c_addr)
{
	int ret;

	ret = cy201_write_block_data(ts, cmd->reg,
		cmd->len, cmd->dat, i2c_addr);
	if (cmd->dly)
		msleep(cmd->dly);
	return ret;
}

static int cy201_input_ports(struct cy201 *ts)
{
	return cy201_read_block_data(ts, 0,
		sizeof(struct cy201_input_ports),
		&ts->input_ports, i2c_set[0] & CY201_I2C_ADR_MASK);
}

static void cy201_pr_state(struct cy201 *ts)
{
	pr_info("%s: %s\n", __func__,
		ts->power_state < CY_INVALID_STATE ?
		cy201_powerstate_string[ts->power_state] :
		"INVALID");
}

static int cy201_hw_reset(struct cy201 *ts)
{
	int ret = 0;

	if (ts->platform_data->flags & CY_USE_RESET_PIN) {
		ret = ts->platform_data->hw_reset();
		/* Wait 100 ms for sensor getting ready */
		msleep(100);
	}
	return ret;
}

static int cy201_config(struct cy201 *ts, bool save)
{
	const struct cy201_cmd *cmd;
	int retval = 0;
	int i = 0;

	/*
	 * TODO: Revisit states here.  Active can print
	 *   multiple times during a normal init, and 
	 *   it may be more appropriate for the caller
	 *   to handle state changes where there is more context.
	 */
	do {
		if (save)
			cmd = cy201_save_cmds[i++];
		else
			cmd = cy201_init_cmds[i++];
		if (cmd) {
			cy201_dbg(ts, CY_DBG_LVL_1,
				"%s: Wr cmd reg=0x%02X len=%d nam=%s\n",
				__func__, cmd->reg, cmd->len, cmd->nam);
			retval = cy201_wr_cmd(ts, cmd,
				i2c_set[0] & CY201_I2C_ADR_MASK);
			if (retval < 0) {
				pr_err("%s: Failed write cmd=%s r=%d\n",
					__func__, cmd->nam, retval);
			}
		}
	} while (cmd);

	return retval;
}

static int cy201_hw_recover(struct cy201 *ts, int wake)
{
	u8 reg;
	int retval = 0;

	cy201_dbg(ts, CY_DBG_LVL_2,
		"%s: try recover (wake=%d)\n", __func__, wake);

	retval = cy201_read_block_data(ts, CY_SLEEP_CTRL,
			sizeof(reg), &reg,
			i2c_set[0] & CY201_I2C_ADR_MASK);
	if (retval < 0) {
		pr_err("%s: fail read at reg=%02X\n",
			__func__, CY_SLEEP_CTRL);
	} else {
		if (wake)
			reg &= ~(CY_SLEEP_ENABLE | CY_DEEP_SLEEP_BIT);
		else
			reg |= CY_SLEEP_ENABLE | CY_DEEP_SLEEP_BIT;
		retval = cy201_write_block_data(ts, CY_SLEEP_CTRL,
				sizeof(reg), &reg,
				i2c_set[0] & CY201_I2C_ADR_MASK);
		msleep(CY_DELAY_NORMAL);
		if (retval < 0) {
			pr_err("%s: fail write reg=%02X dat=%02X\n",
				__func__, CY_SLEEP_CTRL, reg);
		} else if (wake)
			ts->power_state = CY_ACTIVE_STATE;
		else
			ts->power_state = CY_SLEEP_STATE;
	}

	return retval;
};

static int cy201_hw_rebaseline(struct cy201 *ts)
{
	u8 reg;
	int retval = 0;

	cy201_dbg(ts, CY_DBG_LVL_2,
		"%s: try rebaseline \n", __func__);

	retval = cy201_read_block_data(ts, CY_CAPSENSE_FILTER_REG,
			sizeof(reg), &reg,
			i2c_set[0] & CY201_I2C_ADR_MASK);
	if (retval < 0) {
		pr_err("%s: fail read at reg=%02X\n",
			__func__, CY_CAPSENSE_FILTER_REG);
	} else {
		reg |= CY_RESET_BASELINE_BIT;
		retval = cy201_write_block_data(ts, CY_CAPSENSE_FILTER_REG,
				sizeof(reg), &reg,
				i2c_set[0] & CY201_I2C_ADR_MASK);
		msleep(CY_DELAY_NORMAL);
		if (retval < 0) {
			pr_err("%s: fail write reg=%02X dat=%02X\n",
				__func__, CY_CAPSENSE_FILTER_REG, reg);
		}
	}

	return retval;
};

/* Driver version */
static ssize_t cy201_drv_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cy201 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver: %s\nVersion: %s\nDate: %s\n",
		ts->input->name, CY201_DRIVER_VERSION, CY201_DRIVER_DATE);
}
static DEVICE_ATTR(drv_ver, S_IRUGO, cy201_drv_ver_show, NULL);


/* Driver status */
static ssize_t cy201_drv_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cy201 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver state is %s\n",
		cy201_powerstate_string[ts->power_state]);
}
static DEVICE_ATTR(drv_stat, S_IRUGO, cy201_drv_stat_show, NULL);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Group Number */
static ssize_t cy201_ic_grpnum_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cy201 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Group: %d\n", ts->ic_grpnum);
}
static ssize_t cy201_ic_grpnum_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cy201 *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int retval = 0;

	mutex_lock(&(ts->mutex));
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cy201_ic_grpnum_store_error_exit;
	}

	if (value > 0xFF)
		value = 0xFF;
	ts->ic_grpnum = value;


cy201_ic_grpnum_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->mutex));
	return retval;
}
static DEVICE_ATTR(ic_grpnum, S_IRUSR | S_IWUSR,
	cy201_ic_grpnum_show, cy201_ic_grpnum_store);

/* Group Offset */
static ssize_t cy201_ic_grpoffset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cy201 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Offset: %u\n", ts->ic_grpoffset);
}
static ssize_t cy201_ic_grpoffset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cy201 *ts = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	mutex_lock(&(ts->mutex));
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cy201_ic_grpoffset_store_error_exit;
	}

	if (value > 0xFF)
		value = 0xFF;
	ts->ic_grpoffset = value;

cy201_ic_grpoffset_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->mutex));
	return retval;
}
static DEVICE_ATTR(ic_grpoffset, S_IRUSR | S_IWUSR,
	cy201_ic_grpoffset_show, cy201_ic_grpoffset_store);

/* Group Data */
static ssize_t cy201_ic_grpdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cy201 *ts = dev_get_drvdata(dev);
	int i;
	int num_read = 0;
	int start_read = 0;
	u8 *ic_buf = kzalloc(ts->platform_data->
		sett[CY_IC_GRPNUM_BUTTON_PRESS]->size, GFP_KERNEL);
	u8 *grpdata;
	ssize_t ret;

	if (!ic_buf) {
		pr_err("%s: Failed to kzalloc ic_buf!\n", __func__);
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Failed to kzalloc ic_buf.\n");
		goto end1;
	}

	if (!(ts->ic_grpnum < CY_IC_GRPNUM_NUM)) {
		pr_err("%s: Group %d does not exist.\n",
			__func__, ts->ic_grpnum);
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Group %d does not exist.\n",
			ts->ic_grpnum);
		goto end;
	}

	if (ts->ic_grpoffset > ts->platform_data->sett[ts->ic_grpnum]->size) {
		pr_err("%s: Offset %u exceeds group size of %d\n",
			__func__, ts->ic_grpoffset, ts->ic_grpnum);
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Offset %u exceeds group size of %d\n",
			ts->ic_grpoffset, ts->ic_grpnum);
		goto end;
	}

	start_read = ts->ic_grpoffset;
	num_read = ts->platform_data->sett[ts->ic_grpnum]->size - start_read;
	if (num_read) {
		mutex_lock(&(ts->mutex));
		grpdata = (u8 *)ts->platform_data->sett[ts->ic_grpnum]->data;
		for (i = 0; i < num_read; i++)
			ic_buf[i] = grpdata[start_read+i];
		mutex_unlock(&(ts->mutex));
	}

	snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Group %d, Offset %u:\n", ts->ic_grpnum, ts->ic_grpoffset);
	for (i = 0; i < num_read; i++) {
		snprintf(buf, CY_MAX_PRBUF_SIZE,
			"%s0x%02X\n", buf, ic_buf[i]);
	}
	ret = snprintf(buf, CY_MAX_PRBUF_SIZE,
		"%s(%d bytes)\n", buf, num_read);
end:
	kfree(ic_buf);
end1:
	return ret;
}
static ssize_t cy201_ic_grpdata_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cy201 *ts = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;
	const char *pbuf = buf;
	u8 *ic_buf = kzalloc(ts->platform_data->
		sett[CY_IC_GRPNUM_BUTTON_PRESS]->size, GFP_KERNEL);
	int i = 0;
	int j;
	char scan_buf[5];
	u8 *grpdata;
	int grpsize = 0;

	if (!ic_buf) {
		pr_err("%s: Failed to kzalloc ic_buf!\n", __func__);
		goto cy201_ic_grpdata_store_error_exit1;
	}

	if (!(ts->ic_grpnum < CY_IC_GRPNUM_NUM)) {
		pr_err("%s: Group %d does not exist.\n",
			__func__, ts->ic_grpnum);
		goto cy201_ic_grpdata_store_error_exit;
	}

	if (ts->ic_grpoffset > ts->platform_data->sett[ts->ic_grpnum]->size) {
		pr_err("%s: Offset %u exceeds group size of %d\n",
			__func__, ts->ic_grpoffset, ts->ic_grpnum);
		goto cy201_ic_grpdata_store_error_exit;
	}

	i = 0;
	while (pbuf <= ((buf + size) - (sizeof(scan_buf)-1))) {
		while (((*pbuf == ' ') || (*pbuf == ',')) &&
			(pbuf < ((buf + size) - 4)))
			pbuf++;
		if (pbuf <= ((buf + size) - (sizeof(scan_buf)-1))) {
			memset(scan_buf, 0, sizeof(scan_buf));
			for (j = 0; j <  sizeof(scan_buf)-1; j++)
				scan_buf[j] = *pbuf++;
			retval = strict_strtoul(scan_buf, 16, &value);
			if (retval < 0) {
				pr_err("%s: Invalid data format. "
					"Use \"0xHH,...,0xHH\" instead.\n",
					__func__);
				goto cy201_ic_grpdata_store_error_exit;
			} else {
				ic_buf[i] = value;
				i++;
			}
		} else
			break;
	}

	mutex_lock(&(ts->mutex));
	grpsize = ts->platform_data->sett[ts->ic_grpnum]->size;
	cy201_dbg(ts, CY_DBG_LVL_2,
		"%s: write %d bytes at grpnum=%d grpoffset=%u "
		"grpsize=%d\n",
		__func__, i, ts->ic_grpnum, ts->ic_grpoffset, grpsize);
	i = i > grpsize - ts->ic_grpoffset ? grpsize - ts->ic_grpoffset : i;
	grpdata = (u8 *)ts->platform_data->sett[ts->ic_grpnum]->data;
	for (j = 0; j < i; j++)
		grpdata[j] = ic_buf[i];
	mutex_unlock(&(ts->mutex));

cy201_ic_grpdata_store_error_exit:
	kfree(ic_buf);
cy201_ic_grpdata_store_error_exit1:
	retval = size;
	return retval;
}
static DEVICE_ATTR(ic_grpdata, S_IRUSR | S_IWUSR,
	cy201_ic_grpdata_show, cy201_ic_grpdata_store);

struct cy201_device_info {
	u8 id;
	u8 status;
};

static ssize_t cy201_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cy201 *ts = dev_get_drvdata(dev);
	struct cy201_device_info device_info;
	int retval = 0;

	memset(&device_info, 0, sizeof(struct cy201_device_info));
	retval = cy201_read_block_data(ts, CY_DEVICE_INFO_REG,
			sizeof(struct cy201_device_info),
			&device_info, i2c_set[0] & CY201_I2C_ADR_MASK);

	if (retval) {
		pr_err("%s: Error reading device info r=%d\n",
			__func__, retval);
	}

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Device ID: 0x%02X  Device Status: 0x%02X\n",
		device_info.id, device_info.status);
}
static DEVICE_ATTR(ic_ver, S_IRUSR | S_IWUSR,
	cy201_ic_ver_show, NULL);
#endif


#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Disable Driver IRQ */
static ssize_t cy201_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	static const char *fmt_disabled = "Driver IRQ is disabled\n";
	static const char *fmt_enabled = "Driver IRQ is enabled\n";
	struct cy201 *ts = dev_get_drvdata(dev);

	if (ts->irq_enabled == false)
		return snprintf(buf, strlen(fmt_disabled), fmt_disabled);
	else
		return snprintf(buf, strlen(fmt_enabled), fmt_enabled);
}
static ssize_t cy201_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval = 0;
	struct cy201 *ts = dev_get_drvdata(dev);
	unsigned long value;

	mutex_lock(&(ts->mutex));

	if (size > 2) {
		pr_err("%s: Err, data too large\n", __func__);
		retval = -EOVERFLOW;
		goto cy201_drv_irq_store_error_exit;
	}

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value\n", __func__);
		goto cy201_drv_irq_store_error_exit;
	}

	if (ts->irq_enabled == false) {
		if (value == 1) {
			/* Enable IRQ */
			enable_irq(ts->irq);
			pr_info("%s: Driver IRQ now enabled\n", __func__);
			ts->irq_enabled = true;
		} else {
			pr_info("%s: Driver IRQ already disabled\n", __func__);
		}
	} else {
		if (value == 0) {
			/* Disable IRQ */
			disable_irq_nosync(ts->irq);
			pr_info("%s: Driver IRQ now disabled\n", __func__);
			ts->irq_enabled = false;
		} else {
			pr_info("%s: Driver IRQ already enabled\n", __func__);
		}
	}

	retval = size;

cy201_drv_irq_store_error_exit:
	mutex_unlock(&(ts->mutex));
	return retval;
}
static DEVICE_ATTR(drv_irq, S_IRUSR | S_IWUSR,
	cy201_drv_irq_show, cy201_drv_irq_store);
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Driver debugging */
static ssize_t cy201_drv_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cy201 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Debug Setting: %u\n", ts->bus_ops->tsdebug);
}
static ssize_t cy201_drv_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cy201 *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value;

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		pr_err("%s: Failed to convert value r=%d\n", __func__, retval);
		goto cy201_drv_debug_store_exit;
	}

	switch (value) {
	case CY_DBG_LVL_0:
	case CY_DBG_LVL_1:
	case CY_DBG_LVL_2:
	case CY_DBG_LVL_3:
		pr_info("%s: Debug setting=%d\n", __func__, (int)value);
		ts->bus_ops->tsdebug = value;
		break;
	default:
		pr_err("%s: Invalid debug setting=%d\n", __func__, (int)value);
		break;
	}

cy201_drv_debug_store_exit:
	return size;
}
static DEVICE_ATTR(drv_debug, S_IRUSR | S_IWUSR,
	cy201_drv_debug_show, cy201_drv_debug_store);
#endif

static void cy201_ldr_init(struct cy201 *ts)
{
	if (device_create_file(ts->dev, &dev_attr_drv_ver))
		pr_err("%s: Cannot create drv_ver\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_stat))
		pr_err("%s: Cannot create drv_stat\n", __func__);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (device_create_file(ts->dev, &dev_attr_ic_grpnum))
		printk(KERN_ERR "%s: Cannot create ic_grpnum\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpoffset))
		printk(KERN_ERR "%s: Cannot create ic_grpoffset\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpdata))
		printk(KERN_ERR "%s: Cannot create ic_grpdata\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_ver))
		printk(KERN_ERR "%s: Cannot create ic_ver\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_irq))
		printk(KERN_ERR "%s: Cannot create drv_irq\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_debug))
		printk(KERN_ERR "%s: Cannot create drv_debug\n", __func__);
#endif

	return;
}

static void cy201_ldr_free(struct cy201 *ts)
{
	device_remove_file(ts->dev, &dev_attr_drv_ver);
	device_remove_file(ts->dev, &dev_attr_drv_stat);
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	device_remove_file(ts->dev, &dev_attr_ic_grpnum);
	device_remove_file(ts->dev, &dev_attr_ic_grpoffset);
	device_remove_file(ts->dev, &dev_attr_ic_grpdata);

	device_remove_file(ts->dev, &dev_attr_ic_ver);
	device_remove_file(ts->dev, &dev_attr_drv_irq);
	device_remove_file(ts->dev, &dev_attr_drv_debug);

#endif
}

static int cy201_startup(struct cy201 *ts)
{
	int retval;
	int tries = 0;

cy201_startup_start:
	if (tries > 10) {
		pr_err("%s: startup fail max attempts %d", __func__, tries);
		goto cy201_startup_exit;
	} else if (tries != 0) {
		pr_err("%s: startup fail on attempt %d", __func__, tries);
	}

	tries++;

	retval = cy201_hw_reset(ts);
	if (retval < 0) {
		pr_err("%s: HW reset failure\n", __func__);
		ts->power_state = CY_INVALID_STATE;
	} else {
		ts->power_state = CY_ACTIVE_STATE;
		retval = cy201_hw_recover(ts, 1);
		if (retval) {
			pr_err("%s: Error recover chip (step 1) r=%d\n",
				__func__, retval);
			if (ts->resuming)
				goto cy201_startup_start;
		}
		/* Assume cy201_startup() is called by resume function only */
		retval = cy201_wr_cmd(ts, &cy201_enter_setup,
			ts->platform_data->addr[0]);
		if (retval < 0) {
			pr_err("%s: Error enter setup mode r=%d\n",
				__func__, retval);
			if (ts->resuming)
				goto cy201_startup_start;
		}
		retval = cy201_wr_cmd(ts, &cy201_reconfig_to_POR,
			ts->platform_data->addr[0]);
		if (retval < 0) {
			pr_err("%s: Error reconfig to POR r=%d\n",
				__func__, retval);
			if (ts->resuming)
				goto cy201_startup_start;
		}
		retval = cy201_hw_rebaseline(ts);
		if (retval) {
			pr_err("%s: Error rebaseline chip r=%d\n",
				__func__, retval);
			if (ts->resuming)
				goto cy201_startup_start;
		}
	}

cy201_startup_exit:
	cy201_pr_state(ts);
	return retval;
}

#define CY_BUTTON_DOWN		1
#define CY_BUTTON_UP		0

static int cy201_rd_buttons(struct cy201 *ts)
{
	int i;
	int irq_level = -1;
	bool button_down = false;
	bool use_pulse_low = false;
	int retval = 0;
	u16 button_status = 0;
	u16 button_mask = 0x0001;

	/* read button input ports */
	retval = cy201_input_ports(ts);
	if (retval < 0) {
		pr_err("%s: Error reading input ports r=%d\n",
			__func__, retval);
		goto cy201_rd_buttons_no_buttons;
	}

	/* get irq level */
	use_pulse_low =
		ts->platform_data->flags & CY_USE_LOW_IRQ_PULSE ? true : false;
	if (ts->platform_data->irq_stat)
		irq_level = ts->platform_data->irq_stat();
	if (!ts->platform_data->irq_stat || irq_level < 0) {
		pr_err("%s: Error getting irq level level=%d\n",
			__func__, irq_level);
		goto cy201_rd_buttons_no_buttons;
	}

	/* determine if a button is down */
	if (use_pulse_low) {
		if (irq_level)
			button_down = false;
		else
			button_down = true;
	} else {
		if (irq_level)
			button_down = true;
		else
			button_down = false;
	}

	cy201_dbg(ts, CY_DBG_LVL_1,
		"%s: Input-ports=%02X %02X status-ports=%02X %02X r=%d "
		"irq_level=%d button_down=%d use_pulse_low=%d\n",
		__func__,
		ts->input_ports.input_port0,
		ts->input_ports.input_port1,
		ts->input_ports.status_port0,
		ts->input_ports.status_port1,
		retval, irq_level, (int)button_down, (int)use_pulse_low);

	button_status = ts->input_ports.input_port0 |
		(ts->input_ports.input_port1 << 5);

	if (button_down) {	/* Press */
		for (i = 0; i < ARRAY_SIZE(ts->button); i++) {
			if (button_status & button_mask) {
				ts->button[i].down = true;
				input_report_key(ts->input,
					ts->button[i].key, CY_BUTTON_DOWN);
				/*
				 * Normally check if more keys
				 * could be reported against
				 * platform bit masks
				 * instead of breaking.
				 * There probably should be
				 * some debouncing here
				 * somewhere (latched data?).
				 */
				break;
			}
			button_mask = button_mask << 1;
		}
	} else {	/* Release */
		for (i = 0; i < ARRAY_SIZE(ts->button); i++) {
			if (ts->button[i].down) {
				ts->button[i].down = false;
				input_report_key(ts->input,
					ts->button[i].key, CY_BUTTON_UP);
			}
		}
	}
	input_sync(ts->input);

cy201_rd_buttons_no_buttons:
	return retval;
}

static irqreturn_t cy201_irq(int irq, void *handle)
{
	struct cy201 *ts = handle;

	switch (ts->power_state) {
	case CY_ACTIVE_STATE:
		/* process the button touches */
		cy201_rd_buttons(ts);
		break;
	default:
		break;
	}

	return IRQ_HANDLED;
}

static int cy201_diff(struct cy201 *ts)
{
	int i;
	bool diff = false;

	for (i = CY_IC_GRPNUM_PORT_CONFIG;
		i < CY_IC_GRPNUM_SLEEP_CTRL; i++) {
		/*
		 * TODO:
		 * add code to check for differences between the chip
		 * register setups and the platform data for the
		 * register setup
		 */
		diff = true; /* should be true to always write settings */
	}

	return (int)diff;
}

static int cy201_chip_addr_set(struct cy201 *ts)
{
	u8 reg;
	int retval;

	retval = cy201_hw_reset(ts);
	if (retval < 0) {
		pr_err("%s: HW reset failure\n", __func__);
		goto end;
	}

	/* try accessing the device at the expected programmed address */
	retval = cy201_read_block_data(ts, CY_DEVICE_INFO_REG,
		sizeof(reg), &reg, i2c_set[0] & CY201_I2C_ADR_MASK);
	if (retval < 0) {
		/*
		 * if the expected address doesn't work
		 * try address 0 for a blank part
		 */
		retval = cy201_read_block_data(ts, CY_DEVICE_INFO_REG,
			sizeof(reg), &reg, 0x00);
		if (retval < 0) {
			pr_err("%s: Cannot access device @ either "
				"addr 0x00 or 0x%02X\n",
				__func__, (int)i2c_set[0] & CY201_I2C_ADR_MASK);
			goto end;
		}

		if (0xA0 != reg) {
			pr_err("%s: ic not found\n", __func__);
			retval = -EIO;
			goto end;
		}
		pr_info("%s: try changing i2c addr @ 0x%02X\n", __func__,
			i2c_set[0] & CY201_I2C_ADR_MASK);
		retval = cy201_wr_cmd(ts, &cy201_unlock_i2c, 0x00);
		if (retval < 0) {
			pr_err("%s: Fail on write unlock i2c cmd r=%d\n",
				__func__, retval);
			goto end;
		}
		retval = cy201_wr_cmd(ts, &cy201_set_i2c, 0x00);
		if (retval < 0) {
			pr_err("%s: Fail on write set i2c cmd r=%d\n",
				__func__, retval);
			goto end;
			/* try and relock the i2c reg */
		}
		retval = cy201_wr_cmd(ts, &cy201_lock_i2c, 0x00);
		if (retval < 0) {
			pr_err("%s: Fail on write lock i2c cmd r=%d\n",
				__func__, retval);
			goto end;
		}
		retval = cy201_wr_cmd(ts, &cy201_save_settings,
			i2c_set[0] & CY201_I2C_ADR_MASK);
		if (retval < 0) {
			pr_err("%s: Fail on write saving i2c cmd r=%d\n",
				__func__, retval);
			goto end;
		}
		pr_info("%s: done\n", __func__);
	}
	/* TODO: This is the ready state */
end:
	return retval;
}

static int cy201_ld_pdata_sett(struct cy201 *ts)
{
	int ret = 0;
	int i;

	pr_info("%s: loading platform data\n", __func__);
	/* Load platform settings */
	for(i = 0; i < ARRAY_SIZE(cy201_pdata_tbl); i++) {
		if(ts->platform_data->sett[cy201_pdata_tbl[i].nbr] == NULL) {
			pr_err("%s: Warning - group %d settings not found.\n",
				__func__, (int)cy201_pdata_tbl[i].nbr);
			continue;
		}
		/*
		 * This is an undocumented "feature" that writing to
		 *   registers 0x04 and 0x05 while writing to other
		 *   registers in the same transaction (e.g., 0x06, 0x07)
		 *   causes other settings to become corrupted.
		 * This corruption triggers another undocumented
		 *   "feature" where the button IC will change
		 *   user settings it does not like or thinks is
		 *   appropriate, which "corrupts" settings in
		 *   later transactions.
		 * To work around this, we need to write these two
		 *   registers separately after doing all other writes.
		 *   This also keeps the IC from generating an interrupt
		 *   before we have written all of the correct settings.
		 */
		if (cy201_pdata_tbl[i].addr == 0x04) {
			/*
			 * Change the struct data to the platform pointer,
			 *   which will override the default values defined
			 *   above.  Size will remain sizeof(clr_out_ports),
			 *   so only the first bytes are actually read.
			 */
			cy201_clr_out_ports.dat = (u8 *)ts->platform_data->
				sett[cy201_pdata_tbl[i].nbr]->data;
			/* Point remaining settings to other data structure */
			cy201_port_configs.dat = (u8 *)ts->platform_data->
				sett[cy201_pdata_tbl[i].nbr]->data + 2;
			cy201_port_configs.len = ts->platform_data->
				sett[cy201_pdata_tbl[i].nbr]->size - 2;
			continue;
		}
		cy201_pdata_tbl[i].p_cfg->dat = (u8 *)ts->platform_data->
				sett[cy201_pdata_tbl[i].nbr]->data;
		cy201_pdata_tbl[i].p_cfg->len = ts->platform_data->
				sett[cy201_pdata_tbl[i].nbr]->size;
	}


	/* init button structs */
	for (i = 0; i < ARRAY_SIZE(ts->button); i++) {
		ts->button[i].key = ts->platform_data->frmwrk->abs[i];
		/* TODO: bits field may be used for additional key mask */
		ts->button[i].bits = 0;  /* At some point, sett[1]->data[?] */
		ts->button[i].down = false;
	}

	return ret;
}

static int cy201_power_on(struct cy201 *ts)
{
	int retval = 0;

	if (!ts) {
		retval = -ENOMEM;
		goto out;
	}

	/* Add /sys files */
	cy201_ldr_init(ts);
	ts->power_state = CY_IDLE_STATE;

	/* enable interrupts */
	if(request_threaded_irq(ts->irq, NULL, cy201_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		ts->input->name, ts)) {
		pr_err("%s: IRQ request failed\n", __func__);
		retval = -EIO;
		goto error;
	}
	/* enable_irq(ts->irq); is implied above */
	disable_irq(ts->irq);

	if (cy201_diff(ts)) {
		ts->power_state = CY_LDR_STATE;
		retval = cy201_config(ts, false);
		retval = cy201_config(ts, true);
		if (retval) {
			pr_err("%s: Error return from writing configs r=%d\n",
			__func__, retval);
			goto error;
		} else {
			cy201_dbg(ts, CY_DBG_LVL_3,
				"%s: Succeed to write configs\n", __func__);
		}
	}
	retval = cy201_wr_cmd(ts, &cy201_clr_out_ports,
		ts->platform_data->addr[0]);
	if (retval < 0) {
		pr_err("%s: Error clearing button ports r=%d\n",
			__func__, retval);
		goto error;
	}

	if (ts->platform_data->sett[CY_IC_GRPNUM_BUTTON_CONFIG]) {
		/*
		 * TODO:  Check and jump to the correct offset
		 *   forward rather than stepping back one.
		 *   This implementation is error prone if too many
		 *   or not enough settings are provided.
		 */
		cy201_rebaseline.dat[0] = cy201_rebaseline.dat[0] |
			ts->platform_data->sett[CY_IC_GRPNUM_BUTTON_CONFIG]
			->data[ts->platform_data->
			sett[CY_IC_GRPNUM_BUTTON_CONFIG]->size - 1];
	}
	cy201_dbg(ts, CY_DBG_LVL_2, "%s: rebaselining...\n", __func__);
	retval = cy201_wr_cmd(ts, &cy201_rebaseline,
			ts->platform_data->addr[0]);
	if (retval < 0) {
		pr_err("%s: Failed to re-baseline the IC\n", __func__);
		goto error;
	}

	ts->irq_enabled = true;
	ts->power_state = CY_ACTIVE_STATE;
	enable_irq(ts->irq);
	goto done;

error:
	ts->irq_enabled = false;
	ts->power_state = CY_INVALID_STATE;
done:
	cy201_pr_state(ts);
out:
	return retval;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
int cy201_resume(void *handle)
{
	struct cy201 *ts = handle;
	int retval = 0;

	mutex_lock(&ts->mutex);

	cy201_dbg(ts, CY_DBG_LVL_3, "%s: Resuming...", __func__);

	ts->resuming = true;

	if (ts->power_state == CY_LDR_STATE)
		goto cy201_resume_exit;

	if ((ts->platform_data->flags & CY_USE_SLEEP) &&
		(ts->power_state != CY_ACTIVE_STATE)) {
		/*
		 * TODO: add any "wake" requirements
		 */
		if (ts->platform_data->flags & CY_USE_SLEEP_PIN)
			retval = cy201_hw_recover(ts, 1);
		else
			retval = cy201_startup(ts);
		if (retval < 0)
			pr_err("%s: fail RESUME r=%d\n", __func__, retval);
	}
	cy201_dbg(ts, CY_DBG_LVL_3, "%s: Wake Up %s\n", __func__,
		(retval < 0) ? "FAIL" : "PASS");

cy201_resume_exit:
	ts->resuming = false;
	mutex_unlock(&ts->mutex);
	return retval;
}
EXPORT_SYMBOL_GPL(cy201_resume);

int cy201_suspend(void *handle)
{
	struct cy201 *ts = handle;
	int retval = 0;

	mutex_lock(&ts->mutex);

	cy201_dbg(ts, CY_DBG_LVL_3, "%s: Suspending...", __func__);

	if (ts->power_state == CY_SLEEP_STATE)
		goto cy201_suspend_exit;

	if (ts->power_state == CY_LDR_STATE) {
		retval = -EBUSY;
		goto cy201_suspend_exit;
	}

	if ((ts->platform_data->flags & CY_USE_SLEEP) &&
		(ts->power_state == CY_ACTIVE_STATE)) {
		/*
		 * TODO: add any "sleep" requirements
		 */
		retval = cy201_hw_recover(ts, 0);
		if (retval < 0)
			pr_err("%s: fail SUSPEND r=%d\n", __func__, retval);
	}
	cy201_pr_state(ts);

cy201_suspend_exit:
	mutex_unlock(&ts->mutex);
	return retval;
}
EXPORT_SYMBOL_GPL(cy201_suspend);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void cy201_early_suspend(struct early_suspend *h)
{
	struct cy201 *ts = container_of(h, struct cy201, early_suspend);
	int retval = 0;

	retval = cy201_suspend(ts);
	if (retval < 0) {
		pr_err("%s: Early suspend failed with error code %d\n",
			__func__, retval);
	}
}

void cy201_late_resume(struct early_suspend *h)
{
	struct cy201 *ts = container_of(h, struct cy201, early_suspend);
	int retval = 0;

	retval = cy201_resume(ts);
	if (retval < 0) {
		pr_err("%s: Late resume failed with error code %d\n",
			__func__, retval);
	}
}
#endif

void cy201_core_release(void *handle)
{
	struct cy201 *ts = handle;

	if (ts) {
#ifdef CONFIG_HAS_EARLY_SUSPEND
		unregister_early_suspend(&ts->early_suspend);
#endif
		free_irq(ts->irq, ts);
		cy201_ldr_free(ts);
		mutex_destroy(&ts->mutex);
		mutex_destroy(&ts->startup_mutex);
		free_irq(ts->irq, ts);
		input_unregister_device(ts->input);
		kfree(ts);
	}
}
EXPORT_SYMBOL_GPL(cy201_core_release);

static int cy201_open(struct input_dev *dev)
{
	return 0;
}

static void cy201_close(struct input_dev *dev)
{
	return;
}

void *cy201_core_init(struct cy201_bus_ops *bus_ops,
	struct device *dev, int irq, char *name)
{
	struct input_dev *input_device;
	struct cy201 *ts = kzalloc(sizeof(*ts), GFP_KERNEL);

	if (!ts) {
		pr_err("%s: Error, kzalloc\n", __func__);
		goto error_alloc_data;
	}

	if ((dev == NULL) || (bus_ops == NULL)) {
		pr_err("%s: Error, dev, or bus_ops null\n",
			__func__);
		kfree(ts);
		goto error_alloc_data;
	}

	mutex_init(&ts->mutex);
	mutex_init(&ts->startup_mutex);
	ts->dev = dev;
	ts->platform_data = dev->platform_data;
	ts->bus_ops = bus_ops;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->bus_ops->tsdebug = CY_DBG_LVL_0;
#endif
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->ic_grpnum = CY_IC_GRPNUM_BUTTON_PRESS;
	ts->ic_grpoffset = 0;
#endif
	init_completion(&ts->int_running);
	ts->resuming = false;

	ts->irq = irq;

	if (ts->irq <= 0) {
		pr_err("%s: Error, failed to allocate irq\n", __func__);
		goto error_init;
	}

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device) {
		pr_err("%s: Error, failed to allocate input device\n",
			__func__);
		goto error_input_allocate_device;
	}

	ts->input = input_device;
	input_device->name = name;
	snprintf(ts->phys, sizeof(ts->phys), "%s", dev_name(dev));
	input_device->phys = ts->phys;
	input_device->dev.parent = ts->dev;
	ts->bus_type = bus_ops->dev->bus;
	input_device->open = cy201_open;
	input_device->close = cy201_close;
	input_set_drvdata(input_device, ts);
	dev_set_drvdata(dev, ts);

	__set_bit(EV_KEY, input_device->evbit);

	bitmap_fill(input_device->keybit, KEY_MAX);
	i2c_set[0] = ts->platform_data->addr[0];
	if (!(ts->platform_data->flags & CY_USE_INTERNAL_PU))
		i2c_set[0] |= CY_OPEN_DRAIN_BIT;

	if (cy201_chip_addr_set(ts)) {
		pr_err("%s: Error, failed to set sensor address\n", __func__);
		goto error_sensor_access;
	}
	if (cy201_ld_pdata_sett(ts)) {
		pr_err("%s: Error, failed to set platform setting\n", __func__);
		goto error_sensor_access;
	}
	if (cy201_power_on(ts)) {
		pr_err("%s: Error, failed to power on sensor\n", __func__);
		goto error_sensor_access;
	}

	input_set_events_per_packet(input_device, 60);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = cy201_early_suspend;
	ts->early_suspend.resume = cy201_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	if (input_register_device(input_device)) {
		pr_err("%s: Error, failed to register input device\n",
			__func__);
		goto error_input_register_device;
	}

	goto no_error;

error_sensor_access:
error_input_register_device:
	input_free_device(input_device);
error_input_allocate_device:

error_init:
	mutex_destroy(&ts->mutex);
	mutex_destroy(&ts->startup_mutex);
	kfree(ts);
error_alloc_data:
	pr_err("%s: Failed Initialization\n", __func__);
	return NULL;
no_error:
	return ts;
}
EXPORT_SYMBOL_GPL(cy201_core_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress CapSense(R) I2C driver");
MODULE_AUTHOR("Cypress");

