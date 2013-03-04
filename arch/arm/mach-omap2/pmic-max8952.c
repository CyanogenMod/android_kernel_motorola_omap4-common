/*
 * MAX-specific functions for the OPP code
 *
 * Copyright (C) 2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include "voltage.h"
#include "pm.h"

#define RAMP_REG_ADDR    0x06
#define RAMP_REG_VALUE   0x10
#define SLAVE_ADDR       0x60

/**
 * omap_max8952_vsel_to_vdc - convert MAX8952 VSEL value to microvolts DC
 * @vsel: MAX8952 VSEL value to convert
 *
 * Returns the microvolts DC that the MAX8952 Regulator should generate when
 * programmed with @vsel.
 */
unsigned long omap_max8952_vsel_to_uv(unsigned char vsel)
{
	if (vsel > 0x3F)
		vsel = 0x3F;
	return (((vsel * 100) + 7700)) * 100;
}

/**
 * omap_max8952_uv_to_vsel - convert microvolts DC to MAX8952 VSEL value
 * @uv: microvolts DC to convert
 *
 * Returns the VSEL value necessary for the MAX8952 Regulator to
 * generate an output voltage equal to or greater than @uv microvolts DC.
 */
unsigned char omap_max8952_uv_to_vsel(unsigned long uv)
{
	if (uv < 770000)
		uv = 770000;
	else if (uv > 1400000)
		uv = 1400000;
	return DIV_ROUND_UP(uv - 770000, 10000);
}

u8 omap_max8952_reconfigure_regs(struct voltagedomain *voltdm,
				unsigned char i2c_addr)
{
	omap_vc_bypass_send_i2c_msg(voltdm, i2c_addr,
				RAMP_REG_ADDR, RAMP_REG_VALUE);
	return 0;
}

static struct omap_voltdm_pmic omap443x_max8952_mpu = {
	.slew_rate		= 16000,
	.step_size		= 10000,
	.on_volt		= 1375000,
	.onlp_volt		= 1375000,
	.ret_volt		= 830000,
	.off_volt		= 0,
	.volt_setup_time	= 0,
	.switch_on_time		= 1000,
	.vp_erroroffset		= OMAP4_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin		= OMAP4_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax		= OMAP4_VP_VSTEPMAX_VSTEPMAX,
	.vp_vddmin		= 900000,
	.vp_vddmax		= 1400000,
	.vp_timeout_us		= 0x200,
	.i2c_slave_addr		= 0x60,
	.volt_reg_addr		= 0x03,
	.cmd_reg_addr		= 0x03,
	.i2c_high_speed		= false,
	.i2c_scll_low		= 0x60,
	.i2c_scll_high		= 0x26,
	.i2c_hscll_low		= 0x0B,
	.i2c_hscll_high		= 0x00,
	.reconfigure_switcher	= omap_max8952_reconfigure_regs,
	.vsel_to_uv		= omap_max8952_vsel_to_uv,
	.uv_to_vsel		= omap_max8952_uv_to_vsel,
};

static __initdata struct omap_pmic_map omap_max_map[] = {
	{
		.name = "mpu",
		.omap_chip = OMAP_CHIP_INIT(CHIP_IS_OMAP443X),
		.pmic_data = &omap443x_max8952_mpu,
	},
	/* Terminator */
	{ .name = NULL, .pmic_data = NULL},
};



/*  */
static __initdata struct omap_pmic_description max_pmic_desc = {
	.pmic_lp_tshut = 70,	/* T-OFF 1ns rounded */
	.pmic_lp_tstart = 140,	/* T-start */
};

int __init omap_max8952_init(void)
{
	return  omap_pmic_register_data(omap_max_map, &max_pmic_desc);
}
