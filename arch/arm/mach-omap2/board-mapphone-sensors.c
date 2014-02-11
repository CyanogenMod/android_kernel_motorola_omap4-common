/*
 * linux/arch/arm/mach-omap2/board-mapphone-sensors.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <linux/akm8975.h>
#include <linux/attiny48mu_cap_prox.h>
#include <linux/bu52014hfv.h>
#include <linux/clk.h>
#include <linux/ct405.h>
#include <linux/delay.h>
#include <linux/gpio_mapping.h>
#include <linux/i2c/adp8870.h>
#include <linux/i2c/lm3532.h>
#include <linux/input.h>
#include <linux/input/bmp180.h>
#include <linux/interrupt.h>
#include <linux/isl29030.h>
#include <linux/kxtf9.h>
#include <linux/l3g4200d.h>
#include <linux/leds.h>
#include <linux/leds-lp8550.h>
#include <linux/lis3dh.h>
#include <linux/max9635.h>
#include <linux/msp430.h>
#include <linux/regulator/consumer.h>
#include <linux/vib-gpio.h>
#include <linux/vib-pwm.h>

#include "control.h"
#include <plat/dmtimer.h>
#include <plat/gpio.h>
#include <plat/keypad.h>
#include <plat/mux.h>

#include "dt_path.h"
#include <linux/of.h>


#define REGULATOR_NAME_LENGTH 10

/*
 * Vibe
 */
static int vib_pwm_enable_regulator;
static int vib_pwm_timer_id;
static unsigned long load_reg;
static unsigned long cmp_reg;

static struct regulator *vibrator_regulator;


static int vib_init_regulator(void)
{
	struct regulator *reg;
	struct device_node *node;
	const void *prop = NULL;
	int min_volt = 3000000;
	int max_volt = 3000000;
	int len = 0;

	reg = regulator_get(NULL, "vvib");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	vibrator_regulator = reg;

	node = of_find_node_by_path("/System@0/SPI@0/PowerIC@0");
	if (node) {
		prop = of_get_property(node, "regulator_init", &len);
		if (prop && len) {
			min_volt = ((u32 *) prop)[1];
			max_volt = ((u32 *) prop)[2];
		} else {
			pr_err("regulator_init is not defined\n");
		}
		of_node_put(node);
	}

	regulator_set_voltage(vibrator_regulator, min_volt, max_volt);
	return 0;
}

/*
 * GPIO vibrator
 */

static int vib_gpio_init(void)
{
	return vib_init_regulator();
}

static void vib_gpio_exit(void)
{
	regulator_put(vibrator_regulator);
}

static int vib_gpio_power_on(void)
{
	return regulator_enable(vibrator_regulator);
}

static int vib_gpio_power_off(void)
{
	if (vibrator_regulator)
		return regulator_disable(vibrator_regulator);
	return 0;
}

static struct vib_gpio_platform_data vib_gpio_data = {
	.gpio = 181,
	.max_timeout = 15000,
	.active_low = 0,
	.initial_vibrate = 0,
	.init = vib_gpio_init,
	.exit = vib_gpio_exit,
	.power_on = vib_gpio_power_on,
	.power_off = vib_gpio_power_off,
};

static struct platform_device vib_gpio = {
	.name           = "vib-gpio",
	.id             = -1,
	.dev            = {
		.platform_data  = &vib_gpio_data,
	},
};


static void __init gpio_vibrator_init(void)
{
	int vibrator_gpio = get_gpio_by_name("vib_control_en");
	if (vibrator_gpio < 0) {
		pr_info("Cannot retrieve vibrator enable GPIO from dev tree\n");
		return;
	}

	vib_gpio_data.gpio = vibrator_gpio;
	if (gpio_request(vib_gpio_data.gpio, "vib control enable")) {
		pr_err("Vibrator enable GPIO request failed\n");
		return;
	}

	gpio_direction_output(vib_gpio_data.gpio, 0);
}


/*
 * PWM Vibrator
 */

static struct omap_dm_timer *vib_pwm_timer;

static int vib_pwm_init(void);
static void vib_pwm_exit(void);
static void vib_pwm_power_on(void);
static void vib_pwm_power_off(void);
static int vib_pwm_gpio = 9;
static int vib_pwm_duty = 2857;
static int vib_pwm_period = 5714;

static struct vib_pwm_platform_data vib_pwm_data = {
	.initial_vibrate = 500,
	.init = vib_pwm_init,
	.exit = vib_pwm_exit,
	.power_on = vib_pwm_power_on,
	.power_off = vib_pwm_power_off,
	.device_name = "vibrator",
};

static struct platform_device vib_pwm = {
	.name = VIB_PWM_NAME,
	.id = -1,
	.dev = {
		.platform_data = &vib_pwm_data,
	},
};

static void pwm_enable(int value)
{
	if (vib_pwm_enable_regulator) { /* Enable via regulator */
		if (value == 0) {
			if (vibrator_regulator)
				regulator_disable(vibrator_regulator);
		} else if (value == 1) {
			regulator_set_voltage(vibrator_regulator, 3000000,
								3000000);
			regulator_enable(vibrator_regulator);
		}
	} else { /* Enable via GPIO */
		gpio_set_value(vib_pwm_gpio, value);
	}
}

static int vib_pwm_init(void)
{
	uint32_t timer_rate = 0;
	int ret = 0;

	vib_pwm_timer = omap_dm_timer_request_specific(vib_pwm_timer_id);
	if (vib_pwm_timer == NULL)
		return -ENODEV;

	timer_rate = clk_get_rate(omap_dm_timer_get_fclk(vib_pwm_timer));
	load_reg = timer_rate * vib_pwm_period / 1000000;
	cmp_reg = timer_rate * (vib_pwm_period - vib_pwm_duty)
			/ 1000000;

	omap_dm_timer_set_source(vib_pwm_timer, OMAP_TIMER_SRC_32_KHZ);

	if (vib_pwm_enable_regulator)
		vib_init_regulator();
	else {

		int vibrator_gpio = get_gpio_by_name("lin_vib_amp_en");
		if (vibrator_gpio < 0) {
			pr_info("Error re vib enable GPIO from devtree\n");
			return -ENODEV;
		} else {
			vib_pwm_gpio = vibrator_gpio;
			ret = gpio_request(vib_pwm_gpio,
				"lin vib amp enable");
			if (ret) {
				pr_err("Vibrator GPIO request error %d\n", ret);
				return ret;
			}
			gpio_direction_output(vib_pwm_gpio, 1);
		}
	}


	return 0;
}

static void vib_pwm_exit(void)
{
	omap_dm_timer_stop(vib_pwm_timer);
	omap_dm_timer_disable(vib_pwm_timer);
	pwm_enable(0);
}

static void vib_pwm_power_on(void)
{
	pwm_enable(1);

	omap_dm_timer_enable(vib_pwm_timer);
	omap_dm_timer_set_int_enable(vib_pwm_timer, 0);
	omap_dm_timer_set_load(vib_pwm_timer, 1, -load_reg);
	omap_dm_timer_set_match(vib_pwm_timer, 1, -cmp_reg);
	omap_dm_timer_set_pwm(vib_pwm_timer, 0, 1,
		       OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_write_counter(vib_pwm_timer, -2);
	omap_dm_timer_start(vib_pwm_timer);
}

static void vib_pwm_power_off(void)
{
	omap_dm_timer_stop(vib_pwm_timer);
	omap_dm_timer_disable(vib_pwm_timer);
	pwm_enable(0);
}


static void vibrator_init(void)
{
	struct device_node *node;
	const void *prop = NULL;
	int len = 0;

	node = of_find_node_by_path("/System@0/PWMVibrator@0");
	if (node != NULL) {
		pr_info("Vibrator enabled as PWM\n");
		vib_pwm_enable_regulator = 0;

		prop = of_get_property(node, "period", &len);
		if (prop && len)
			vib_pwm_period = *((int *)prop);

		prop = of_get_property(node, "duty", &len);
		if (prop && len)
			vib_pwm_duty = *((int *)prop);

		prop = of_get_property(node, "timer_id", &len);
		if (prop && len)
			vib_pwm_timer_id = *((int *)prop);

		prop = of_get_property(node, "enable_reg", &len);
		if (prop && len)
			vib_pwm_enable_regulator = *((int *)prop);

		pr_info("period %d duty %d timer_id %d enable_reg %d\n",
			vib_pwm_period, vib_pwm_duty,
			vib_pwm_timer_id, vib_pwm_enable_regulator);

		platform_device_register(&vib_pwm);
		of_node_put(node);
	}

	node = of_find_node_by_path("/System@0/GpioVibrator@0");
	if (node != NULL) {
		pr_info("Vibrator enabled as GPIO\n");
		gpio_vibrator_init();
		platform_device_register(&vib_gpio);
	}
}


/*
 * LIS3DH
 */

static struct regulator *lis3dh_regulator;
static char lis3dh_reg_name[REGULATOR_NAME_LENGTH] = "";

static int lis3dh_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, lis3dh_reg_name);
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	lis3dh_regulator = reg;
	return 0;
}

static void lis3dh_exit(void)
{
	regulator_put(lis3dh_regulator);
}

static int lis3dh_power_on(void)
{
	return regulator_enable(lis3dh_regulator);
}

static int lis3dh_power_off(void)
{
	if (lis3dh_regulator)
		return regulator_disable(lis3dh_regulator);
	return 0;
}

struct lis3dh_platform_data mp_lis3dh_pdata = {
	.init = lis3dh_initialization,
	.exit = lis3dh_exit,
	.power_on = lis3dh_power_on,
	.power_off = lis3dh_power_off,

	.min_interval = 1,
	.poll_interval = 200,
	.g_range = 8,
};

#define LIS3DH_DEV_NAME         "lis3dh"
#define LIS3DH_I2C_DEFAULT_ADDR 0x19
#define LIS3DH_I2C_DEFAULT_BUS  4


static int __init lis3dh_init(struct device_node *node)
{
	const void *prop;
	int len = 0;
	static struct i2c_board_info i2c_info = {0};
	u8 i2c_bus;
	int err;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "poll_interval", &len);
	if (prop && len)
		mp_lis3dh_pdata.poll_interval = *(int *)prop;
	prop = of_get_property(node, "min_interval", &len);
	if (prop && len)
		mp_lis3dh_pdata.min_interval = *(int *)prop;
	prop = of_get_property(node, "g_range", &len);
	if (prop && len)
		mp_lis3dh_pdata.g_range = *(int *)prop;
	prop = of_get_property(node, "regulator", &len);
	if (prop && len) {
		strncpy(lis3dh_reg_name,
			(char *)prop, REGULATOR_NAME_LENGTH - 1);
		lis3dh_reg_name[REGULATOR_NAME_LENGTH - 1] = '\0';
	}

	strncpy(i2c_info.type, LIS3DH_DEV_NAME, I2C_NAME_SIZE);
	i2c_info.platform_data = (void *) &mp_lis3dh_pdata;
	prop = of_get_property(node, DT_PROP_I2C_ADDR, &len);
	if (prop && len)
		i2c_info.addr = *(u32 *)prop;
	else
		i2c_info.addr = LIS3DH_I2C_DEFAULT_ADDR;

	prop = of_get_property(node, DT_PROP_I2C_BUS, &len);
	if (prop && len)
		i2c_bus = *(u32 *)prop;
	else
		i2c_bus = LIS3DH_I2C_DEFAULT_BUS;

	err = i2c_register_board_info(i2c_bus, &i2c_info, 1);
	if (err) {
		pr_err("i2c_register_board_info failed for lis3dh: %d\n", err);
		return -ENODEV;
	}

	return 0;
}



/*
 * KXTF9
 */

struct kxtf9_platform_data mp_kxtf9_pdata = {
	.min_interval	= 2,
	.poll_interval	= 200,
	.g_range = 8,
	.irq = 0,

	.regulator = "",

	.tdt_always_on		= 0,
	.tdt_odr		= 0xC,
	.tdt_directions		= 0x2,

	.tdt_timer_init		= 0x78,
	.tdt_h_thresh_init	= 0xB6,
	.tdt_l_thresh_init	= 0x1A,
	.tdt_tap_timer_init	= 0xA2,
	.tdt_total_timer_init	= 0x24,
	.tdt_latency_timer_init	= 0x28,
	.tdt_window_timer_init	= 0xA0,
};


static int __init kxtf9_init(struct device_node *node)
{
	const void *prop;
	int gpio = -1, len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "regulator", &len);
	if (prop && len) {
		strncpy(mp_kxtf9_pdata.regulator, (char *)prop,
			sizeof(mp_kxtf9_pdata.regulator) - 1);
		mp_kxtf9_pdata.regulator[
			sizeof(mp_kxtf9_pdata.regulator) - 1] = '\0';
	}

	gpio = get_gpio_by_name("kxtf9_int");
	if (gpio < 0)
		return -ENODEV;

	gpio_request(gpio, "kxtf9 accelerometer int");
	gpio_direction_input(gpio);

	mp_kxtf9_pdata.irq = gpio_to_irq(gpio);

	return 0;
}


/*
 * Capacitive Proximity Sensor
 */

struct attiny48mu_cap_prox_platform_data mp_attiny48mu_cap_prox_pdata = {
	.irq				= 0,
	.poll_interval                  = 10000,
	.min_poll_interval              = 200,
	.key1_ref_drift_thres_l         = 5,
	.key3_ref_drift_thres_l         = 5,
	.key1_ref_drift_thres_h         = 30,
	.key3_ref_drift_thres_h         = 30,
	.ref_drift_diff_thres           = 9,
	.key1_ref_drift_thres_l_sleep         = 5,
	.key3_ref_drift_thres_l_sleep         = 5,
	.key1_ref_drift_thres_h_sleep         = 30,
	.key3_ref_drift_thres_h_sleep         = 30,
	.ref_drift_diff_thres_sleep           = 9,
	.key1_save_drift_thres          = 125,
	.key3_save_drift_thres          = 165,
	.save_drift_diff_thres          = 90,
	.key1_failsafe_thres            = 250,
	.key3_failsafe_thres            = 150,
	.key2_signal_thres              = 1700,
	.key4_signal_thres              = 2200,
	.plat_cap_prox_cfg = {
		.lp_mode                = 0x03,
		.lp_mode_active         = 0x03,
		.address_ptr            = 0x10,
		.reset                  = 0x20,
		.key_enable_mask        = 0x35,
		.data_integration       = 0x40,
		.neg_drift_rate         = 0x50,
		.pos_drift_rate         = 0x60,
		.force_detect           = 0x70,
		.calibrate              = 0x80,
		.thres_key1             = 0x92,
		.ref_backup             = 0xaa,
		.thres_key2             = 0xb2,
		.reserved12             = 0xc0,
		.drift_hold_time        = 0xd0,
		.reserved14             = 0xe0,
		.reserved15             = 0xf0,
	},
};


static int __init attiny48mu_cap_prox_init(struct device_node *node)
{
	int gpio = -1;
	const void *prop;
	int len = 0;

	prop = of_get_property(node, "poll_interval", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.poll_interval = *(u16 *)prop;

	prop = of_get_property(node, "key1_ref_drift_thres_l", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.key1_ref_drift_thres_l
			= *(u8 *)prop;

	prop = of_get_property(node, "key3_ref_drift_thres_l", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.key3_ref_drift_thres_l
			= *(u8 *)prop;

	prop = of_get_property(node, "key1_ref_drift_thres_h", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.key1_ref_drift_thres_h
			= *(u8 *)prop;

	prop = of_get_property(node, "key3_ref_drift_thres_h", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.key3_ref_drift_thres_h
			= *(u8 *)prop;

	prop = of_get_property(node, "ref_drift_diff_thres", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.ref_drift_diff_thres
			= *(u8 *)prop;

	prop = of_get_property(node, "key1_ref_drift_thres_l_sleep", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.key1_ref_drift_thres_l_sleep
			= *(u8 *)prop;

	prop = of_get_property(node, "key3_ref_drift_thres_l_sleep", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.key3_ref_drift_thres_l_sleep
			= *(u8 *)prop;

	prop = of_get_property(node, "key1_ref_drift_thres_h_sleep", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.key1_ref_drift_thres_h_sleep
			= *(u8 *)prop;

	prop = of_get_property(node, "key3_ref_drift_thres_h_sleep", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.key3_ref_drift_thres_h_sleep
			= *(u8 *)prop;

	prop = of_get_property(node, "ref_drift_diff_thres_sleep", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.ref_drift_diff_thres_sleep
			= *(u8 *)prop;

	prop = of_get_property(node, "thres_key1", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.plat_cap_prox_cfg.thres_key1
			= *(u8 *)prop;

	prop = of_get_property(node, "thres_key2", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.plat_cap_prox_cfg.thres_key2
			= *(u8 *)prop;

	prop = of_get_property(node, "lp_mode", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.plat_cap_prox_cfg.lp_mode
			= *(u8 *)prop;

	prop = of_get_property(node, "lp_mode_active", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.plat_cap_prox_cfg.lp_mode_active
			= *(u8 *)prop;

	prop = of_get_property(node, "data_integration", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.plat_cap_prox_cfg.data_integration
			= *(u8 *)prop;

	prop = of_get_property(node, "neg_drift_rate", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.plat_cap_prox_cfg.neg_drift_rate
			= *(u8 *)prop;

	prop = of_get_property(node, "pos_drift_rate", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.plat_cap_prox_cfg.pos_drift_rate
			= *(u8 *)prop;

	prop = of_get_property(node, "drift_hold_time", &len);
	if (prop && len)
		mp_attiny48mu_cap_prox_pdata.plat_cap_prox_cfg.drift_hold_time
			= *(u8 *)prop;

	gpio = get_gpio_by_name("cap_prox_int");
	if (gpio < 0)
		return -ENODEV;

	gpio_request(gpio, "cap_prox_irq");
	gpio_direction_input(gpio);

	mp_attiny48mu_cap_prox_pdata.irq = gpio_to_irq(gpio);

	return 0;
}


/*
 *MSP430
 */

static struct regulator *mapphone_msp430_regulator;

static int mapphone_msp430_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_msp430_regulator = reg;
	return 0;
}

static void mapphone_msp430_exit(void)
{
	regulator_put(mapphone_msp430_regulator);
}

static int mapphone_msp430_power_on(void)
{
	return regulator_enable(mapphone_msp430_regulator);
}

static int mapphone_msp430_power_off(void)
{
	if (mapphone_msp430_regulator)
		return regulator_disable(mapphone_msp430_regulator);
	return 0;
}

struct msp430_platform_data mp_msp430_data = {
	.init = mapphone_msp430_initialization,
	.exit = mapphone_msp430_exit,
	.power_on = mapphone_msp430_power_on,
	.power_off = mapphone_msp430_power_off,
	.gpio_reset = -1,
	.gpio_test = -1,
	.gpio_int = -1,
	.test_pin_active_value = 1,
};


static void __init mapphone_msp430_init(void)
{
	int err = 0;
	int gpio = -1;
	u32 reg = 0;
	gpio = get_gpio_by_name("msp430_reset");
	if (gpio < 0)
		return;
	mp_msp430_data.gpio_reset = gpio;

	gpio = get_gpio_by_name("msp430_test");
	if (gpio < 0)
		return;
	mp_msp430_data.gpio_test = gpio;

	gpio = get_gpio_by_name("msp430_int");
	if (gpio < 0)
		return;
	mp_msp430_data.gpio_int = gpio;

	/* configure MSP430 gpios */
	err = gpio_request(mp_msp430_data.gpio_reset, "msp430 reset");
	if (err) {
		pr_err("MSP430 getting reset gpio failed with %d\n", err);
		return;
	}

	gpio_direction_output(mp_msp430_data.gpio_reset, 0);
	err = gpio_request(mp_msp430_data.gpio_test, "msp430 test");
	if (err) {
		pr_err("MSP430 getting test gpio failed with %d\n", err);
		gpio_free(mp_msp430_data.gpio_reset);
		return;
	}

	gpio_direction_output(mp_msp430_data.gpio_test, 0);

	/* setting up pbias on SIM, needed for this GPIO */
	reg = omap4_ctrl_pad_readl(
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE);
	reg |= (OMAP4_USBC1_ICUSB_PWRDNZ_MASK |
		OMAP4_USIM_PBIASLITE_PWRDNZ_MASK |
		OMAP4_MMC1_PWRDNZ_MASK);
	reg &= ~(OMAP4_USIM_PBIASLITE_VMODE_MASK);
	omap4_ctrl_pad_writel(reg,
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE);

	err = gpio_request(mp_msp430_data.gpio_int, "msp430 interrupt");
	if (err) {
		pr_err("MSP430 getting reset gpio failed with %d\n", err);
		gpio_free(mp_msp430_data.gpio_reset);
		gpio_free(mp_msp430_data.gpio_test);
		return;
	}
	gpio_direction_input(mp_msp430_data.gpio_int);
}


struct platform_device msp430_platform_device = {
	.name = "msp430",
	.id = -1,
	.dev = {
		.platform_data = &mp_msp430_data,
	},
};

/*
 * AKM8975
 */

static struct regulator *akm8975_regulator;
static char akm8975_reg_name[REGULATOR_NAME_LENGTH] = "";

static int akm8975_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, akm8975_reg_name);
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	akm8975_regulator = reg;
	return 0;
}

static void akm8975_exit(void)
{
	if (akm8975_regulator)
		regulator_put(akm8975_regulator);
}

static int akm8975_power_on(void)
{
	if (akm8975_regulator)
		return regulator_enable(akm8975_regulator);
	return 0;
}

static int akm8975_power_off(void)
{
	if (akm8975_regulator)
		return regulator_disable(akm8975_regulator);
	return 0;
}

struct akm8975_platform_data mp_akm8975_pdata = {
	.init = akm8975_initialization,
	.exit = akm8975_exit,
	.power_on = akm8975_power_on,
	.power_off = akm8975_power_off,
	.layout = 1,
};

static int __init akm8975_init(struct device_node *node)
{
	const void *prop;
	int gpio = -1, len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "regulator", &len);
	if (prop && len) {
		strncpy(akm8975_reg_name,
			(char *)prop, REGULATOR_NAME_LENGTH - 1);
		akm8975_reg_name[REGULATOR_NAME_LENGTH - 1] = '\0';
	}

	prop = of_get_property(node, "layout", &len);
	if (prop && len)
		mp_akm8975_pdata.layout = *(char *)prop;

	gpio = get_gpio_by_name("akm8975_int");
	if (gpio < 0)
		return -ENODEV;

	gpio_request(gpio, "akm8975 irq");
	gpio_direction_input(gpio);

	return 0;
}

/*
 * ISL29030
 */

struct isl29030_platform_data mp_isl29030_pdata = {
	.configure = 0x62,
	.interrupt_cntrl = 0x20,
	.prox_lower_threshold = 0x1e,
	.prox_higher_threshold = 0x32,
	.crosstalk_vs_covered_threshold = 0x30,
	.default_prox_noise_floor = 0x30,
	.num_samples_for_noise_floor = 0x05,
	.lens_percent_t = 20,
	.regulator_name = "",
};

static int __init isl29030_init(struct device_node *node)
{
	const void *prop;
	int gpio = -1, len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "configure", &len);
	if (prop && len)
		mp_isl29030_pdata.configure = *(u8 *)prop;

	prop = of_get_property(node, "interrupt_cntrl", &len);
	if (prop && len)
		mp_isl29030_pdata.interrupt_cntrl = *(u8 *)prop;

	prop = of_get_property(node, "prox_lower_threshold", &len);
	if (prop && len)
		mp_isl29030_pdata.prox_lower_threshold = *(u8 *)prop;

	prop = of_get_property(node, "prox_higher_threshold", &len);
	if (prop && len)
		mp_isl29030_pdata.prox_higher_threshold = *(u8 *)prop;

	prop = of_get_property(node, "crosstalk_vs_covered_threshold", &len);
	if (prop && len)
		mp_isl29030_pdata.crosstalk_vs_covered_threshold = *(u8 *)prop;

	prop = of_get_property(node, "default_prox_noise_floor", &len);
	if (prop && len)
		mp_isl29030_pdata.default_prox_noise_floor = *(u8 *)prop;

	prop = of_get_property(node, "num_samples_for_noise_floor", &len);
	if (prop && len)
		mp_isl29030_pdata.num_samples_for_noise_floor = *(u8 *)prop;

	prop = of_get_property(node, "lens_percent_t", &len);
	if (prop && len)
		mp_isl29030_pdata.lens_percent_t = *(u8 *)prop;

	prop = of_get_property(node, "regulator", &len);
	if (prop && len) {
		strncpy(mp_isl29030_pdata.regulator_name,
			(char *)prop,
			sizeof(mp_isl29030_pdata.regulator_name) - 1);
		mp_isl29030_pdata.regulator_name[
			sizeof(mp_isl29030_pdata.regulator_name) - 1] = '\0';
	}

	gpio = get_gpio_by_name("isl29030_int");
	if (gpio < 0)
		return -ENODEV;

	mp_isl29030_pdata.irq = gpio_to_irq(gpio);
	gpio_request(gpio, "isl29030 proximity int");
	gpio_direction_input(gpio);

	return 0;
}


/*
 * MAX9635
 */

struct max9635_platform_data mp_max9635_pdata = {
	.configure = 0x80,
	.threshold_timer = 0x04,
	.def_low_threshold = 0xFE,
	.def_high_threshold = 0xFF,
	.lens_coeff = 0x20,
};


static int __init max9635_init(struct device_node *node)
{
	const void *prop;
	int gpio = -1, len = 0;

	prop = of_get_property(node, "lens_coeff", &len);
	if (prop && len)
		mp_max9635_pdata.lens_coeff = *(u8 *)prop;

	gpio = get_gpio_by_name("max9635_int");
	if (gpio < 0)
		return -ENODEV;

	gpio_request(gpio, "max9635 als int");
	gpio_direction_input(gpio);

	return 0;
}


/*
 * CT405
 */

struct ct405_platform_data mp_ct405_pdata = {
	.regulator_name = "",
	.prox_samples_for_noise_floor = 0x05,

	.ct405_prox_saturation_threshold = 0x0208,
	.ct405_prox_covered_offset = 0x008c,
	.ct405_prox_uncovered_offset = 0x0046,
	.ct405_prox_recalibrate_offset = 0x0046,

	.ct406_prox_saturation_threshold = 0x0208,
	.ct406_prox_covered_offset = 0x008c,
	.ct406_prox_uncovered_offset = 0x0046,
	.ct406_prox_recalibrate_offset = 0x0046,
};

static int __init ct405_init(struct device_node *node)
{
	const void *prop;
	int gpio = -1, len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "prox_samples_for_noise_floor", &len);
	if (prop && len)
		mp_ct405_pdata.prox_samples_for_noise_floor = *(u8 *)prop;

	prop = of_get_property(node, "ct405_prox_saturation_threshold", &len);
	if (prop && len)
		mp_ct405_pdata.ct405_prox_saturation_threshold = *(u16 *)prop;

	prop = of_get_property(node, "ct405_prox_covered_offset", &len);
	if (prop && len)
		mp_ct405_pdata.ct405_prox_covered_offset = *(u16 *)prop;

	prop = of_get_property(node, "ct405_prox_uncovered_offset", &len);
	if (prop && len)
		mp_ct405_pdata.ct405_prox_uncovered_offset = *(u16 *)prop;

	prop = of_get_property(node, "ct405_prox_recalibrate_offset", &len);
	if (prop && len)
		mp_ct405_pdata.ct405_prox_recalibrate_offset = *(u16 *)prop;

	prop = of_get_property(node, "ct406_prox_saturation_threshold", &len);
	if (prop && len)
		mp_ct405_pdata.ct406_prox_saturation_threshold = *(u16 *)prop;

	prop = of_get_property(node, "ct406_prox_covered_offset", &len);
	if (prop && len)
		mp_ct405_pdata.ct406_prox_covered_offset = *(u16 *)prop;

	prop = of_get_property(node, "ct406_prox_uncovered_offset", &len);
	if (prop && len)
		mp_ct405_pdata.ct406_prox_uncovered_offset = *(u16 *)prop;

	prop = of_get_property(node, "ct406_prox_recalibrate_offset", &len);
	if (prop && len)
		mp_ct405_pdata.ct406_prox_recalibrate_offset = *(u16 *)prop;

	prop = of_get_property(node, "regulator", &len);
	if (prop && len) {
		strncpy(mp_ct405_pdata.regulator_name, (char *)prop,
			sizeof(mp_ct405_pdata.regulator_name) - 1);
		mp_ct405_pdata.regulator_name[
			sizeof(mp_ct405_pdata.regulator_name) - 1] = '\0';
	}

	gpio = get_gpio_by_name("ct405_int");
	if (gpio < 0)
		return -ENODEV;

	mp_ct405_pdata.irq = gpio_to_irq(gpio);
	gpio_request(gpio, "ct405 proximity int");
	gpio_direction_input(gpio);

	return 0;
}

/*
 * BU52014HFV
 */

static struct bu52014hfv_platform_data mp_bu52014hfv_pdata = {
	.docked_north_gpio = 0,
	.docked_south_gpio = 0,
	.kickstand_gpio = 0,
	.north_is_desk = 1,
	.sensor_function = 0,
};

static int __init bu52014hfv_init(struct device_node *node)
{
	const void *prop;
	int bu52014hfv_north_gpio = -1;
	int bu52014hfv_south_gpio = -1;
	int bu52014hfv_kickstand_gpio = -1;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "north_is_desk", NULL);
	if (prop)
		mp_bu52014hfv_pdata.north_is_desk = *(u8 *)prop;

	prop = of_get_property(node, "sensor_function", NULL);
	if (prop)
		mp_bu52014hfv_pdata.sensor_function = *(u8 *)prop;

	if (mp_bu52014hfv_pdata.sensor_function == BU52014HFV_DOCK_SENSOR) {
		bu52014hfv_north_gpio = get_gpio_by_name("bu52014hfv_n_gpio");
		if (bu52014hfv_north_gpio < 0)
			return -ENODEV;

		mp_bu52014hfv_pdata.docked_north_gpio = bu52014hfv_north_gpio;

		bu52014hfv_south_gpio = get_gpio_by_name("bu52014hfv_s_gpio");
		if (bu52014hfv_south_gpio < 0)
			return -ENODEV;

		mp_bu52014hfv_pdata.docked_south_gpio = bu52014hfv_south_gpio;

		gpio_request(mp_bu52014hfv_pdata.docked_north_gpio,
			"bu52014hfv dock north");
		gpio_direction_input(mp_bu52014hfv_pdata.docked_north_gpio);

		gpio_request(mp_bu52014hfv_pdata.docked_south_gpio,
			"bu52014hfv dock south");
		gpio_direction_input(mp_bu52014hfv_pdata.docked_south_gpio);
	} else if (mp_bu52014hfv_pdata.sensor_function
			== BU52014HFV_KICKSTAND_SENSOR) {
		bu52014hfv_kickstand_gpio
			= get_gpio_by_name("bu52014hfv_k_gpio");
		if (bu52014hfv_kickstand_gpio < 0)
			return -ENODEV;

		mp_bu52014hfv_pdata.kickstand_gpio = bu52014hfv_kickstand_gpio;

		gpio_request(mp_bu52014hfv_pdata.kickstand_gpio,
			"bu52014hfv kickstand");
		gpio_direction_input(mp_bu52014hfv_pdata.kickstand_gpio);
	} else
		return -EINVAL;

	return 0;
}

static struct platform_device mp_bu52014hfv = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &mp_bu52014hfv_pdata,
	},
};

/*
 * ADP8870
 */

static int __init adp8870_init(void)
{
	int adp8870_reset_gpio = get_gpio_by_name("adp8870_reset");

	if (adp8870_reset_gpio < 0)
		return -ENODEV;

	gpio_request(adp8870_reset_gpio, "adp8870 reset");
	gpio_direction_output(adp8870_reset_gpio, 1);

	return 0;
}

static struct led_info adp8870_leds[] = {
	{
		.name = "adp8870-led7",
		.default_trigger = "none",
		.flags = ADP8870_LED_D7 | ADP8870_LED_DIS_BLINK,
	},
};

struct adp8870_backlight_platform_data mp_adp8870_pdata = {
	.bl_led_assign = ADP8870_BL_D1 | ADP8870_BL_D2 | ADP8870_BL_D3 |
		ADP8870_BL_D4 | ADP8870_BL_D5 | ADP8870_BL_D6,
	.pwm_assign = 0,

	.bl_fade_in = ADP8870_FADE_T_DIS,
	.bl_fade_out = ADP8870_FADE_T_DIS,
	.bl_fade_law = ADP8870_FADE_LAW_LINEAR,

	.en_ambl_sens = 0,
	.abml_filt = ADP8870_BL_AMBL_FILT_320ms,

	.l1_daylight_max = ADP8870_BL_CUR_mA(20),
	.l1_daylight_dim = ADP8870_BL_CUR_mA(0),
	.l2_bright_max = ADP8870_BL_CUR_mA(14),
	.l2_bright_dim = ADP8870_BL_CUR_mA(0),
	.l3_office_max = ADP8870_BL_CUR_mA(6),
	.l3_office_dim = ADP8870_BL_CUR_mA(0),
	.l4_indoor_max = ADP8870_BL_CUR_mA(3),
	.l4_indor_dim = ADP8870_BL_CUR_mA(0),
	.l5_dark_max = ADP8870_BL_CUR_mA(2),
	.l5_dark_dim = ADP8870_BL_CUR_mA(0),

	.l2_trip = ADP8870_L2_COMP_CURR_uA(710),
	.l2_hyst = ADP8870_L2_COMP_CURR_uA(73),
	.l3_trip = ADP8870_L3_COMP_CURR_uA(389),
	.l3_hyst = ADP8870_L3_COMP_CURR_uA(54),
	.l4_trip = ADP8870_L4_COMP_CURR_uA(167),
	.l4_hyst = ADP8870_L4_COMP_CURR_uA(16),
	.l5_trip = ADP8870_L5_COMP_CURR_uA(43),
	.l5_hyst = ADP8870_L5_COMP_CURR_uA(11),

	.leds = adp8870_leds,
	.num_leds = ARRAY_SIZE(adp8870_leds),
	.led_fade_law = ADP8870_FADE_LAW_LINEAR,
	.led_fade_in = ADP8870_FADE_T_DIS,
	.led_fade_out = ADP8870_FADE_T_DIS,
	.led_on_time = ADP8870_LED_ONT_200ms,
};

/*
 *      LM3532
 */

struct lm3532_backlight_platform_data mp_lm3532_pdata = {
	.gpio = -1,

	.led1_controller = LM3532_CNTRL_A,
	.led2_controller = LM3532_CNTRL_B,
	.led3_controller = LM3532_CNTRL_C,

	.ctrl_a_name = "lm3532_bl",
	.ctrl_b_name = "lm3532_led1",
	.ctrl_c_name = "lm3532_led2",

	.susd_ramp = 0xC0,
	.runtime_ramp = 0xC0,

	.als1_res = 0xE0,
	.als2_res = 0xE0,
	.als_dwn_delay = 0x00,
	.als_cfgr = 0x2C,

	.en_ambl_sens = 0,

	.init_delay_ms = 250,

	.ctrl_a_usage = LM3532_BACKLIGHT_DEVICE,
	.ctrl_a_pwm = 0xC2,
	.ctrl_a_fsc = 0xFF,
	.ctrl_a_l4_daylight = 0xFF,
	.ctrl_a_l3_bright = 0xCC,
	.ctrl_a_l2_office = 0x99,
	.ctrl_a_l1_indoor = 0x66,
	.ctrl_a_l0_dark = 0x33,

	.ctrl_b_usage = LM3532_UNUSED_DEVICE,
	.ctrl_b_pwm = 0x82,
	.ctrl_b_fsc = 0xFF,
	.ctrl_b_l4_daylight = 0xFF,
	.ctrl_b_l3_bright = 0xCC,
	.ctrl_b_l2_office = 0x99,
	.ctrl_b_l1_indoor = 0x66,
	.ctrl_b_l0_dark = 0x33,

	.ctrl_c_usage = LM3532_UNUSED_DEVICE,
	.ctrl_c_pwm = 0x82,
	.ctrl_c_fsc = 0xFF,
	.ctrl_c_l4_daylight = 0xFF,
	.ctrl_c_l3_bright = 0xCC,
	.ctrl_c_l2_office = 0x99,
	.ctrl_c_l1_indoor = 0x66,
	.ctrl_c_l0_dark = 0x33,

	.l4_high = 0xCC,
	.l4_low = 0xCC,
	.l3_high = 0x99,
	.l3_low = 0x99,
	.l2_high = 0x66,
	.l2_low = 0x66,
	.l1_high = 0x33,
	.l1_low = 0x33,

	.boot_brightness = LM3532_MAX_BRIGHTNESS,
};

static int __init lm3532_init(struct device_node *node)
{
	int gpio = -1;
	const void *prop;
	int len = 0;

	gpio = get_gpio_by_name("lm3532_reset");
	if (gpio < 0)
		return -ENODEV;

	gpio_request(gpio, "lm3532 reset");
	gpio_direction_output(gpio, 1);
	mp_lm3532_pdata.gpio = gpio;

	prop = of_get_property(node, "led1_controller", &len);
	if (prop && len)
		mp_lm3532_pdata.led1_controller = *(u8 *)prop;
	prop = of_get_property(node, "led2_controller", &len);
	if (prop && len)
		mp_lm3532_pdata.led2_controller = *(u8 *)prop;
	prop = of_get_property(node, "led3_controller", &len);
	if (prop && len)
		mp_lm3532_pdata.led3_controller = *(u8 *)prop;
	prop = of_get_property(node, "ctrl_a_usage", &len);
	if (prop && len)
		mp_lm3532_pdata.ctrl_a_usage = *(u8 *)prop;
	prop = of_get_property(node, "ctrl_b_usage", &len);
	if (prop && len)
		mp_lm3532_pdata.ctrl_b_usage = *(u8 *)prop;
	prop = of_get_property(node, "ctrl_c_usage", &len);
	if (prop && len)
		mp_lm3532_pdata.ctrl_c_usage = *(u8 *)prop;
	prop = of_get_property(node, "ctrl_a_name", &len);
	if (prop && len) {
		strncpy(mp_lm3532_pdata.ctrl_a_name,
			(char *)prop,
			sizeof(mp_lm3532_pdata.ctrl_a_name) - 1);
		mp_lm3532_pdata.ctrl_a_name[
			sizeof(mp_lm3532_pdata.ctrl_a_name) - 1] = '\0';
	}
	prop = of_get_property(node, "ctrl_b_name", &len);
	if (prop && len) {
		strncpy(mp_lm3532_pdata.ctrl_b_name,
			(char *)prop,
			sizeof(mp_lm3532_pdata.ctrl_b_name) - 1);
		mp_lm3532_pdata.ctrl_b_name[
			sizeof(mp_lm3532_pdata.ctrl_b_name) - 1] = '\0';
	}
	prop = of_get_property(node, "ctrl_c_name", &len);
	if (prop && len) {
		strncpy(mp_lm3532_pdata.ctrl_c_name,
			(char *)prop,
			sizeof(mp_lm3532_pdata.ctrl_c_name) - 1);
		mp_lm3532_pdata.ctrl_c_name[
			sizeof(mp_lm3532_pdata.ctrl_c_name) - 1] = '\0';
	}
	prop = of_get_property(node, "ctrl_a_pwm", &len);
	if (prop && len)
		mp_lm3532_pdata.ctrl_a_pwm = *(u8 *)prop;
	prop = of_get_property(node, "ctrl_b_pwm", &len);
	if (prop && len)
		mp_lm3532_pdata.ctrl_b_pwm = *(u8 *)prop;
	prop = of_get_property(node, "ctrl_c_pwm", &len);
	if (prop && len)
		mp_lm3532_pdata.ctrl_c_pwm = *(u8 *)prop;

	prop = of_get_property(node, "ctrl_a_fsc", &len);
	if (prop && len)
		mp_lm3532_pdata.ctrl_a_fsc = *(u8 *)prop;
	prop = of_get_property(node, "ctrl_b_fsc", &len);
	if (prop && len)
		mp_lm3532_pdata.ctrl_b_fsc = *(u8 *)prop;
	prop = of_get_property(node, "ctrl_c_fsc", &len);
	if (prop && len)
		mp_lm3532_pdata.ctrl_c_fsc = *(u8 *)prop;

	prop = of_get_property(node, "boot_brightness", &len);
	if (prop && len)
		mp_lm3532_pdata.boot_brightness = *(u8 *)prop;

	return 0;
}

/*
 *      LP8550
 */

struct lp8550_eeprom_data mp_lp8550_eeprom_data[] = {
	/* Set the backlight current to 19mA each step is .12mA */
	{0xa1},
	/* Boost freq 312khz, PWM controled w/constant current,
	thermal deration disabled, brightness slope 500mS */
	{0x67},
	/* Adaptive mode for light loads, advanced slope enabled, 50% mode
	Adaptive mode enabled, Boost is enabled, Boost Imax is 2.5A */
	{0xbf},
	/* UVLO is disabled, phase shift PWM enabled, PWM Freq 19232 */
	{0x3f},
	/* LED current resistor disabled, LED Fault = 3.3V */
	{0x08},
	/* Vsync is enabled, Dither enabled, Boost voltage 20V */
	{0xea},
	/* PLL 13-bit counter */
	{0x64},
	/* 1-bit hysteresis w/10 bit resolution, PWM output freq is set with
	PWM_FREQ EEPROM bits */
	{0x2a},
};

struct lp8550_platform_data mp_lp8550_pdata = {
	.power_up_brightness = 0x80,
	.dev_ctrl_config = 0x04,
	.brightness_control = 0x80,
	.dev_id = 0xfc,
	.direct_ctrl = 0x01,
	.eeprom_table = mp_lp8550_eeprom_data,
	.eeprom_tbl_sz = ARRAY_SIZE(mp_lp8550_eeprom_data),
	.scaling_factor = 690,
};

static int __init lp8550_init(struct device_node *node)
{
	const void *prop;
	int len = 0;
	int gpio = get_gpio_by_name("lp8550_en");
	if (gpio < 0)
		return -ENODEV;

	gpio_request(gpio, "lp8550 en");
	gpio_direction_output(gpio, 1);

	prop = of_get_property(node, "power_up_brightness", &len);
	if (prop && len)
		mp_lp8550_pdata.power_up_brightness = *(u8 *)prop;
	prop = of_get_property(node, "dev_ctrl_config", &len);
	if (prop && len)
		mp_lp8550_pdata.dev_ctrl_config = *(u8 *)prop;
	prop = of_get_property(node, "scaling_factor", &len);
	if (prop && len)
		mp_lp8550_pdata.scaling_factor = *(u16 *)prop;

	return 0;
}

/*
 *      L3G4200D
 */

struct l3g4200d_platform_data mp_l3g4200d_pdata = {
	.regulator_name = "",
	.poll_interval = 10,
	.ctrl_reg1 = 0x1F,
	.ctrl_reg2 = 0x00,
	.ctrl_reg3 = 0x08,
	.ctrl_reg4 = 0xA0,
	.ctrl_reg5 = 0x00,
	.reference = 0x00,
	.fifo_ctrl_reg = 0x00,
	.int1_cfg = 0x00,
	.int1_tsh_xh = 0x00,
	.int1_tsh_xl = 0x00,
	.int1_tsh_yh = 0x00,
	.int1_tsh_yl = 0x00,
	.int1_tsh_zh = 0x00,
	.int1_tsh_zl = 0x00,
	.int1_duration = 0x00,
};

static int __init l3g4200d_init(struct device_node *node)
{
	int gpio = -1;
	const void *prop;
	int len = 0;

	if (node == NULL)
		return -ENODEV;

	gpio = get_gpio_by_name("l3g4200d_int");
	if (gpio < 0)
		return -ENODEV;

	gpio_request(gpio, "l3g4200d gyroscope int");
	gpio_direction_input(gpio);

	mp_l3g4200d_pdata.gpio_drdy = gpio;
	mp_l3g4200d_pdata.irq = OMAP_GPIO_IRQ(gpio);

	prop = of_get_property(node, "regulator", &len);
	if (prop && len) {
		strncpy(mp_l3g4200d_pdata.regulator_name,
			(char *)prop,
			sizeof(mp_l3g4200d_pdata.regulator_name) - 1);
		mp_l3g4200d_pdata.regulator_name[
			sizeof(mp_l3g4200d_pdata.regulator_name) - 1] = '\0';
	}

	return 0;
}

struct bmp180_platform_data mp_bmp180_pdata = {
	.poll_interval = 500,
	.temperature_delay = 6,
	.pressure_delay = 27,
	.min_interval = 100,
	.max_p = 150000,
	.min_p = 0,
	.fuzz = 0,
	.flat = 0,
	.regulator_name = "",
};

static int __init bmp180_init(struct device_node *node)
{
	const void *prop;
	int len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "regulator", &len);
	if (prop && len) {
		strncpy(mp_bmp180_pdata.regulator_name,
			(char *)prop,
			sizeof(mp_bmp180_pdata.regulator_name) - 1);
		mp_bmp180_pdata.regulator_name[
			sizeof(mp_bmp180_pdata.regulator_name) - 1] = '\0';
	}
	return 0;
}

/*
 * TCMD
 */

/* TODO: make more dynamic */
static ssize_t isl29030_sysfs_show(struct class *dev,
	struct class_attribute *attr, char *buf)
{
	int data;
	char *str = buf;
	ssize_t count;
	int gpio = get_gpio_by_name("isl29030_int");

	if (gpio < 0)
		return -ENODEV;

	data = gpio_get_value(gpio);
	str += sprintf(str, "%d\n", data);
	count = (ssize_t) (str - buf);
	return count;
}

static CLASS_ATTR(isl29030, S_IRUGO, isl29030_sysfs_show, NULL);

static ssize_t kxtf9_sysfs_show(struct class *dev,
	struct class_attribute *attr, char *buf)
{
	int data;
	char *str = buf;
	ssize_t count;
	int gpio = get_gpio_by_name("kxtf9_int");

	if (gpio < 0)
		return -ENODEV;

	data = gpio_get_value(gpio);
	str += sprintf(str, "%d\n", data);
	count = (ssize_t) (str - buf);
	return count;
}

static CLASS_ATTR(kxtf9, S_IRUGO, kxtf9_sysfs_show, NULL);

void tcmd_testing(void)
{
	struct class *kxtf9_class, *isl29030_class;

	kxtf9_class = class_create(THIS_MODULE, "kxtf9");
	if (IS_ERR(kxtf9_class))
		pr_err("kxtf9 can't register class\n");
	else if (class_create_file(kxtf9_class, &class_attr_kxtf9)) {
		pr_err("kxtf9: can't create sysfs\n");
		class_destroy(kxtf9_class);
	}

	isl29030_class = class_create(THIS_MODULE, "isl29030");
	if (IS_ERR(isl29030_class))
		pr_err("isl29030 can't register class\n");
	else if (class_create_file(isl29030_class, &class_attr_isl29030) != 0) {
		pr_err("isl29030: can't create sysfs\n");
		class_destroy(isl29030_class);
	}
}

static void __init accelerometer_init(void)
{
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_name(NULL, "Accelerometer");
	if (node) {
		prop = of_get_property(node, "type", &len);
		if (prop && len) {
			switch (*(int *)prop) {
			case 0x00180000:
				pr_info("Kionix,KXTF9 found!\n");
				if (kxtf9_init(node))
					pr_err("kxtf9 error\n");
				/* move i2c driver register here */
				break;
			case 0x00140001:
				pr_info("STMicro,LIS3DH found!\n");
				if (lis3dh_init(node))
					pr_err("lis3dh error\n");
				/* move i2c driver register here */
				break;
			}
		}
		of_node_put(node);
	}
}

static void __init ir_init(void)
{
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_name(NULL, "Proximity");
	if (node) {
		prop = of_get_property(node, "type", &len);
		if (prop && len) {
			switch (*(int *)prop) {
			case 0x001D0000:
				pr_info("Intersil,ISL29030 found!\n");
				if (isl29030_init(node))
					pr_err("isl29030 error\n");
				/* move i2c driver register here */
				break;
			case 0x00250000:
				pr_info("TAOS,CT405 found!\n");
				if (ct405_init(node))
					pr_err("ct405 error\n");
				/* move i2c driver register here */
				break;
			}
		}
		of_node_put(node);
	}

	node = of_find_node_by_name(NULL, "ALS");
	if (node) {
		prop = of_get_property(node, "type", &len);
		if (prop && len) {
			switch (*(int *)prop) {
			case 0x00080002:
				pr_info("Maxim-ic,MAX9635 found\n");
				if (max9635_init(node))
					pr_err("max96355 error\n");
				/* move i2c driver register here */
				break;
			}
		}
		of_node_put(node);
	}
}

static void __init backlight_init(void)
{
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_name(NULL, "LCDBacklight");
	if (node) {
		prop = of_get_property(node, "type", &len);
		if (prop && len) {
			switch (*(int *)prop) {
			case 0x000B0004:
				pr_info("National,LM3532 found!\n");
				if (lm3532_init(node))
					pr_err("lm3532 error\n");
				/* move i2c driver register here */
				break;
			case 0x000B0005:
				pr_info("National,LP8550 found!\n");
				if (lp8550_init(node))
					pr_err("lp8550 error\n");
				/* move i2c driver register here */
				break;
			case 0x00200000:
				pr_info("AnalogDevices,ADP8870 found!\n");
				if (adp8870_init())
					pr_err("adp8870 error\n");
				/* move i2c driver register here */
				break;
			}
		}

		of_node_put(node);
	}
}

static void __init magnetometer_init(void)
{
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_name(NULL, "Magnetometer");
	if (node) {
		prop = of_get_property(node, "type", &len);
		if (prop && len) {
			switch (*(int *)prop) {
			case 0x00150001:
				pr_info("AsahiKasei,AKM8975 found!\n");
				if (akm8975_init(node))
					pr_err("akm8975 error\n");
				/* move i2c driver register here */
				break;
			}
		}
		of_node_put(node);
	}
}

static void __init hall_effect_init(void)
{
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_name(NULL, "HallEffect");
	if (node) {
		prop = of_get_property(node, "type", &len);
		if (prop && len) {
			switch (*(int *)prop) {
			case 0x00170000:
				pr_info("Rohm,BU52014HFV found!\n");
				if (!bu52014hfv_init(node))
					platform_device_register(
						&mp_bu52014hfv);
				else
					pr_err("bu52014hfv error\n");
				break;
			}
		}
		of_node_put(node);
	}
}

static void __init gyroscope_init(void)
{
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_name(NULL, "Gyroscope");
	if (node) {
		prop = of_get_property(node, "type", &len);
		if (prop && len) {
			switch (*(int *)prop) {
			case 0x00140002:
				pr_info("ST Micro L3G4200D found\n");
				if (l3g4200d_init(node))
					pr_err("L3G4200D init failed\n");
				/* move i2c driver register here */
				break;
			}
		}
		of_node_put(node);
	}
}

static void __init barometer_init(void)
{
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_name(NULL, "Barometer");
	if (node) {
		prop = of_get_property(node, "type", &len);
		if (prop && len) {
			switch (*(int *)prop) {
			case 0x00230000:
				pr_info("Bosch BMP180 found\n");
				if (bmp180_init(node))
					pr_err("BMP180 init failed\n");
				/* move i2c driver register here */
				break;
			}
		}
		of_node_put(node);
	}
}

static void __init cap_proximity_init(void)
{
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_name(NULL, "Cap_Proximity");
	if (node) {
		prop = of_get_property(node, "type", &len);
		if (prop && len) {
			switch (*(int *)prop) {
			case 0x00260000:
				pr_info("Atmel ATTINY48MU Cap Prox found\n");
				if (attiny48mu_cap_prox_init(node))
					pr_err("ATTINY48MU Cap Prox "
						"init failed\n");
				/* move i2c driver register here */
				break;
			}
		}
		of_node_put(node);
	}
}

/*
 * Sensors
 */

void __init mapphone_sensors_init(void)
{
	mapphone_msp430_init();
	accelerometer_init();
	magnetometer_init();
	ir_init();
	backlight_init();
	hall_effect_init();
	gyroscope_init();
	barometer_init();
	cap_proximity_init();
	vibrator_init();
	tcmd_testing();
}
