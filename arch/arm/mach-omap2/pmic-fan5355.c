/*
 * FAN5355-specific functions for the OPP code
 *
 * Copyright (C) 2011 Motorola, Inc.
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
#include <linux/io.h>
#include <linux/kernel.h>
#include "voltage.h"
#include "pm.h"

#define CONTROL1_REG_ADDR  0x02
#define CONTROL1_REG_VALUE 0x96
#define CONTROL2_REG_ADDR  0x03
#define CONTROL2_REG_VALUE 0x67

/**
 * omap_fan5355_vsel_to_vdc - convert FAN535503 VSEL value to microvolts DC
 * @vsel: FAN535503 VSEL value to convert
 *
 * Returns the microvolts DC that the FAN535503 Regulator should generate when
 * programmed with @vsel.
 */
unsigned long omap_fan535503_vsel_to_uv(unsigned char vsel)
{
	/* Extract bits[5:0] */
	vsel &= 0x3F;

	return (((vsel * 125) + 7500)) * 100;
}

/**
 * omap_fan535508_vsel_to_vdc - convert FAN535508 VSEL value to microvolts DC
 * @vsel: FAN535508 VSEL value to convert
 *
 * Returns the microvolts DC that the FAN535508 Regulator should generate when
 * programmed with @vsel.
 */
unsigned long omap_fan535508_vsel_to_uv(unsigned char vsel)
{
	/* Extract bits[5:0] */
	vsel &= 0x3F;

	if (vsel > 0x37)
		vsel = 0x37;
	return (((vsel * 125) + 7500)) * 100;
}


/**
 * omap_fan535503_uv_to_vsel - convert microvolts DC to FAN535503 VSEL value
 * @uv: microvolts DC to convert
 *
 * Returns the VSEL value necessary for the MAX8952 Regulator to
 * generate an output voltage equal to or greater than @uv microvolts DC.
 */
unsigned char omap_fan535503_uv_to_vsel(unsigned long uv)
{
	unsigned char vsel;
	if (uv < 750000)
		uv = 750000;
	else if (uv > 1537500)
		uv = 1537500;

	vsel = DIV_ROUND_UP(uv - 750000, 12500);
	return vsel | 0xC0;
}

/**
 * omap_fan535508_uv_to_vsel - convert microvolts DC to FAN535508 VSEL value
 * @uv: microvolts DC to convert
 *
 * Returns the VSEL value necessary for the MAX8952 Regulator to
 * generate an output voltage equal to or greater than @uv microvolts DC.
 */
unsigned char omap_fan535508_uv_to_vsel(unsigned long uv)
{
	unsigned char vsel;
	if (uv < 750000)
		uv = 750000;
	else if (uv > 1437500)
		uv = 1437500;

	vsel = DIV_ROUND_UP(uv - 750000, 12500);
	return vsel | 0xC0;
}

u8 omap_fan5355_reconfigure_regs(struct voltagedomain *voltdm,
					unsigned char i2c_addr)
{
	omap_vc_bypass_send_i2c_msg(voltdm, i2c_addr, CONTROL1_REG_ADDR,
				CONTROL1_REG_VALUE);

	omap_vc_bypass_send_i2c_msg(voltdm, i2c_addr, CONTROL2_REG_ADDR,
				CONTROL2_REG_VALUE);
	return 0;
}


/* fan5335-core */
static struct omap_voltdm_pmic omap4_fan_core = {
	.slew_rate		= 4000,
	.step_size		= 12500,
	.on_volt		= 1300000,
	.onlp_volt		= 1300000,
	.ret_volt		= 837500,
	.off_volt		= 60000,
	.volt_setup_time	= 0,
	.switch_on_time		= 1000,
	.vp_erroroffset		= OMAP4_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin		= OMAP4_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax		= OMAP4_VP_VSTEPMAX_VSTEPMAX,
	.vp_vddmin		= 850000,
	.vp_vddmax		= 1350000,
	.vp_timeout_us		= 0x200,
	.i2c_slave_addr		= 0x4A,
	.i2c_high_speed		= false,
	.i2c_scll_low		= 0x60,
	.i2c_scll_high		= 0x26,
	.i2c_hscll_low		= 0x0B,
	.i2c_hscll_high		= 0x00,
	.volt_reg_addr		= 0x01,
	.cmd_reg_addr		= 0x01,
	.reconfigure_switcher	= omap_fan5355_reconfigure_regs,
	.vsel_to_uv		= omap_fan535508_vsel_to_uv,
	.uv_to_vsel		= omap_fan535508_uv_to_vsel,
};

/* fan5335 iva */
static struct omap_voltdm_pmic omap4_fan_iva = {
	.slew_rate		= 4000,
	.step_size		= 12500,
	.on_volt		= 1125000,
	.onlp_volt		= 1125000,
	.ret_volt		= 837500,
	.off_volt		= 600000,
	.volt_setup_time	= 0,
	.switch_on_time		= 549,
	.vp_erroroffset		= OMAP4_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin		= OMAP4_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax		= OMAP4_VP_VSTEPMAX_VSTEPMAX,
	.vp_vddmin		= 850000,
	.vp_vddmax		= 1350000,
	.vp_timeout_us		= 0x200,
	.i2c_slave_addr		= 0x48,
	.volt_reg_addr		= 0x01,
	.cmd_reg_addr		= 0x01,
	.i2c_high_speed		= false,
	.i2c_scll_low		= 0x60,
	.i2c_scll_high		= 0x26,
	.i2c_hscll_low		= 0x0B,
	.i2c_hscll_high		= 0x00,
	.reconfigure_switcher	= omap_fan5355_reconfigure_regs,
	.vsel_to_uv		= omap_fan535503_vsel_to_uv,
	.uv_to_vsel		= omap_fan535503_uv_to_vsel,
};


static __initdata struct omap_pmic_map fan_map[] = {
	{
		.name = "core",
		.omap_chip = OMAP_CHIP_INIT(CHIP_IS_OMAP443X),
		.pmic_data = &omap4_fan_core,
	},
	{
		.name = "iva",
		.omap_chip = OMAP_CHIP_INIT(CHIP_IS_OMAP443X),
		.pmic_data = &omap4_fan_iva,
	},
	/* Terminator */
	{ .name = NULL, .pmic_data = NULL},
};

static __initdata struct omap_pmic_description fan_desc = {
	.pmic_lp_tshut = 500,	/* T-OFF */
	.pmic_lp_tstart = 500,	/* T-ON */
};

int __init omap_fan5355_init(void)
{
	return omap_pmic_register_data(fan_map, &fan_desc);
}

