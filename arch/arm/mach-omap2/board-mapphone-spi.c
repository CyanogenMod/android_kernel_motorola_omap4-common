/*
 * arch/arm/mach-omap2/board-mapphone-spi.c
 *
 * Copyright (C) 2009-2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio_mapping.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>
#include <plat/mcspi.h>
#include <plat/gpio.h>
#include <plat/omap34xx.h>
#include <plat/omap44xx.h>
#include "board-mapphone.h"
/* XXX #include <linux/mfd/tps65912.h> */
#include <linux/gpio_mapping.h>

#include "dt_path.h"
#include <linux/of.h>
#include <mach/ctrl_module_pad_core_44xx.h>
#include <mach/ctrl_module_pad_wkup_44xx.h>

#include <linux/spi/mdm6600_spi_tty.h>
#include <asm/bootinfo.h>

struct cpcap_spi_init_data mapphone_cpcap_spi_init[CPCAP_REG_SIZE + 1] = {
	{CPCAP_REG_ASSIGN1,   0x0101},
	{CPCAP_REG_ASSIGN2,   0x0000},
	{CPCAP_REG_ASSIGN3,   0x0000},
	{CPCAP_REG_ASSIGN4,   0x0000},
	{CPCAP_REG_ASSIGN5,   0x0000},
	{CPCAP_REG_ASSIGN6,   0x0000},
	{CPCAP_REG_UCC1,      0x0000},
	{CPCAP_REG_PC1,       0x010A},
	{CPCAP_REG_PC2,       0x0150},
	{CPCAP_REG_PGC,       0x0000},
	{CPCAP_REG_SI2CC1,    0x0281},
	{CPCAP_REG_Si2CC2,    0x00C4},
	{CPCAP_REG_SDVSPLL,   0xDB74},
	{CPCAP_REG_S1C1,      0x6832},
	{CPCAP_REG_S1C2,      0x3232},
	{CPCAP_REG_S2C1,      0x082C},
	{CPCAP_REG_S2C2,      0x2C2C},
	{CPCAP_REG_S3C,       0x0541},
	{CPCAP_REG_S4C1,      0x082C},
	{CPCAP_REG_S4C2,      0x2C2C},
	{CPCAP_REG_S6C,       0x0000},
	{CPCAP_REG_SDVSPLL,   0xDB04},
	{CPCAP_REG_VWLAN2C,   0x0000},
	{CPCAP_REG_VUSBINT1C, 0x0029},
	{CPCAP_REG_VUSBINT2C, 0x0029},
	{CPCAP_REG_VAUDIOC,   0x0060},
	{CPCAP_REG_CCCC2,     0x002B},
	{CPCAP_REG_ADCC1,     0x9000},
	{CPCAP_REG_ADCC2,     0x4136},
	{CPCAP_REG_USBC1,     0x1201},
	{CPCAP_REG_USBC3,     0x7DFB},
	{CPCAP_REG_OWDC,      0x0003},
	{CPCAP_REG_GPIO0,     0x3004},
	{CPCAP_REG_GPIO1,     0x3004},
	{CPCAP_REG_GPIO2,     0x3204},
	{CPCAP_REG_GPIO3,     0x3008},
	{CPCAP_REG_GPIO4,     0x3204},
	{CPCAP_REG_GPIO5,     0x3008},
	{CPCAP_REG_GPIO6,     0x3004},
	{CPCAP_REG_KLC,       0x0000},
	{CPCAP_REG_MDLC,      0x0000},
	{CPCAP_REG_UNUSED,    0x0000},
};

unsigned short cpcap_regulator_mode_values[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW5]      = 0x0020 | CPCAP_REG_OFF_MODE_SEC,
	[CPCAP_VCAM]     = 0x0003,
	[CPCAP_VCSI]     = 0x0043,
	[CPCAP_VDAC]     = 0x0003,
	[CPCAP_VDIG]     = 0x0082,
	[CPCAP_VFUSE]    = 0x0080,
	[CPCAP_VHVIO]    = 0x0012,
	[CPCAP_VSDIO]    = 0x0082,
	[CPCAP_VPLL]     = 0x0002,
	[CPCAP_VRF1]     = 0x0004,
	[CPCAP_VRF2]     = 0x0000,
	[CPCAP_VRFREF]   = 0x0000,
	[CPCAP_VWLAN1]   = 0x0000,
	[CPCAP_VWLAN2]   = 0x020C,
	[CPCAP_VSIM]     = 0x0003,
	[CPCAP_VSIMCARD] = 0x1E00,
	[CPCAP_VVIB]     = 0x0001,
	[CPCAP_VUSB]     = 0x000C,
	[CPCAP_VAUDIO]   = 0x0004,
};

unsigned short cpcap_regulator_off_mode_values[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW5]      = 0x0000,
	[CPCAP_VCAM]     = 0x0000,
	[CPCAP_VCSI]     = 0x0041,
	[CPCAP_VDAC]     = 0x0000,
	[CPCAP_VDIG]     = 0x0000,
	[CPCAP_VFUSE]    = 0x0000,
	[CPCAP_VHVIO]    = 0x0000,
	[CPCAP_VSDIO]    = 0x0000,
	[CPCAP_VPLL]     = 0x0000,
	[CPCAP_VRF1]     = 0x0000,
	[CPCAP_VRF2]     = 0x0000,
	[CPCAP_VRFREF]   = 0x0000,
	[CPCAP_VWLAN1]   = 0x0000,
	[CPCAP_VWLAN2]   = 0x0000,
	[CPCAP_VSIM]     = 0x0000,
	[CPCAP_VSIMCARD] = 0x0000,
	[CPCAP_VVIB]     = 0x0000,
	[CPCAP_VUSB]     = 0x0000,
	[CPCAP_VAUDIO]   = 0x0000,
};

#define OMAP_CTRL_BASE	OMAP443X_CTRL_BASE
#define USBA0_OTG_DP	0x196
#define USBA0_OTG_DM	0x198
#define SIM_CLK		0x042
#define GPMC_NBE1	0x088
#define GPIO_MODE	0x622

/* gpio_wk7 is mux mode 3 of OMAP4 pin FREF_CLK4_REQ */
#define CPCAP_GPIO 7

#define REGULATOR_CONSUMER(name, device) { .supply = name, .dev = device, }

enum usb_otg_muxmode {
	OMAP_USB_OTG,
	OMAP_UART_2 = 0x2,
	OMAP_SAFE_MODE = 0x7,
};

static char omap_check_hpd_status(void);
static void omap_force_factory_kill(int enabled);
static void omap_force_charge_complete(int closed);
static void omap_force_charge_terminate(int closed);
static void omap_force_inductive_path(int closed);
static void omap_force_cable_path(int closed);
static int omap_check_inductive_path(void);
static int omap_check_cable_path(void);
static void omap_usb_otg_muxmode(enum cpcap_usb_otg_muxmode muxmode);
static void omap_usb_switch_muxmode(enum cpcap_usb_switch_muxmode muxmode);
static void omap_enable_spdif_audio(int status);
static void omap_enable_usbhs_extpwr(int val);
static int  omap_usbhs_extpwr_init(struct cpcap_usbhs_ext_pwr *pdata);
static void omap_usbhs_extpwr_exit(struct cpcap_usbhs_ext_pwr *pdata);

struct regulator_consumer_supply cpcap_sw5_consumers[] = {
	REGULATOR_CONSUMER("sw5", NULL /* lighting_driver */),
};

struct regulator_consumer_supply cpcap_vcam_consumers[] = {
	REGULATOR_CONSUMER("vcam", NULL /* cpcap_cam_device */),
};

extern struct platform_device mapphone_dss_device;

struct regulator_consumer_supply cpcap_vhvio_consumers[] = {
	REGULATOR_CONSUMER("vhvio", NULL /* lighting_driver */),
#if 0
	REGULATOR_CONSUMER("vhvio", NULL /* lighting_driver */),
	REGULATOR_CONSUMER("vhvio", NULL /* magnetometer */),
	REGULATOR_CONSUMER("vhvio", NULL /* light sensor */),
	REGULATOR_CONSUMER("vhvio", NULL /* accelerometer */),
	REGULATOR_CONSUMER("vhvio", NULL /* display */),
#endif
};

struct regulator_consumer_supply cpcap_vsdio_consumers[] = {
	REGULATOR_CONSUMER("vsdio", NULL),
};

struct regulator_consumer_supply cpcap_vcsi_consumers[] = {
	REGULATOR_CONSUMER("vdds_dsi", NULL /* &mapphone_dss_device.dev */),
};

struct regulator_consumer_supply cpcap_vwlan1_consumers[] = {
	REGULATOR_CONSUMER("vwlan1", NULL /* cpcap_cam_device */),
};

struct regulator_consumer_supply cpcap_vwlan2_consumers[] = {
	REGULATOR_CONSUMER("vwlan2", NULL /* sd slot */),
};

struct regulator_consumer_supply cpcap_vsim_consumers[] = {
	REGULATOR_CONSUMER("vsim", NULL),
};

struct regulator_consumer_supply cpcap_vsimcard_consumers[] = {
	REGULATOR_CONSUMER("vsimcard", NULL),
};

struct regulator_consumer_supply cpcap_vvib_consumers[] = {
	REGULATOR_CONSUMER("vvib", NULL /* vibrator */),
};

struct regulator_consumer_supply cpcap_vusb_consumers[] = {
	REGULATOR_CONSUMER("vusb", NULL /* accy det */),
};

struct regulator_consumer_supply cpcap_vaudio_consumers[] = {
	REGULATOR_CONSUMER("vaudio", NULL /* mic opamp */),
};

struct regulator_consumer_supply cpcap_vfuse_consumers[] = {
	REGULATOR_CONSUMER("vfuse", NULL),
};

struct regulator_consumer_supply cpcap_vrf1_consumers[] = {
	REGULATOR_CONSUMER("vrf1", NULL),
};

struct regulator_consumer_supply cpcap_vdac_consumers[] = {
	REGULATOR_CONSUMER("vdac", NULL),
};

static struct regulator_init_data cpcap_regulator[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW5] = {
		.constraints = {
			.min_uV			= 5050000,
			.max_uV			= 5050000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.boot_on 		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw5_consumers),
		.consumer_supplies	= cpcap_sw5_consumers,
	},
	[CPCAP_VCAM] = {
		.constraints = {
			.min_uV			= 2900000,
			.max_uV			= 2900000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,

		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vcam_consumers),
		.consumer_supplies	= cpcap_vcam_consumers,
	},
	[CPCAP_VCSI] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.boot_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vcsi_consumers),
		.consumer_supplies	= cpcap_vcsi_consumers,
	},
	[CPCAP_VDAC] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vdac_consumers),
		.consumer_supplies	= cpcap_vdac_consumers,
	},
	[CPCAP_VDIG] = {
		.constraints = {
			.min_uV			= 1875000,
			.max_uV			= 1875000,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
			.apply_uV		= 1,
		},
	},
	[CPCAP_VFUSE] = {
		.constraints = {
			.min_uV			= 1500000,
			.max_uV			= 3150000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
		},
		.num_consumer_supplies  = ARRAY_SIZE(cpcap_vfuse_consumers),
		.consumer_supplies      = cpcap_vfuse_consumers,
	},
	[CPCAP_VHVIO] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.always_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vhvio_consumers),
		.consumer_supplies	= cpcap_vhvio_consumers,
	},
	[CPCAP_VSDIO] = {
		.constraints = {
			.min_uV			= 2900000,
			.max_uV			= 2900000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
			.apply_uV		= 1,
			.always_on		= 1,
			/* P0B's DDR eMMC will fail to re-initialize after
			 * certain power off/on iteration on Vcc
			 * Remove this if we have a fix after we
			 * get more from h/w
			 */
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vsdio_consumers),
		.consumer_supplies	= cpcap_vsdio_consumers,
	},
	[CPCAP_VPLL] = {
		.constraints = {
			.min_uV			= 1200000,
			.max_uV			= 1800000,
			.valid_ops_mask		= 0,
			.apply_uV		= 1,
		},
	},
	[CPCAP_VRF1] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vrf1_consumers),
		.consumer_supplies	= cpcap_vrf1_consumers,
	},
	[CPCAP_VRF2] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VRFREF] = {
		.constraints = {
			.min_uV			= 2500000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VWLAN1] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1900000,
			.valid_ops_mask		= 0,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vwlan1_consumers),
		.consumer_supplies	= cpcap_vwlan1_consumers,
	},
	[CPCAP_VWLAN2] = {
		.constraints = {
			.min_uV			= 3000000,
			.max_uV			= 3000000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vwlan2_consumers),
		.consumer_supplies	= cpcap_vwlan2_consumers,
	},
	[CPCAP_VSIM] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 2900000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vsim_consumers),
		.consumer_supplies	= cpcap_vsim_consumers,
	},
	[CPCAP_VSIMCARD] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 2900000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vsimcard_consumers),
		.consumer_supplies	= cpcap_vsimcard_consumers,
	},
	[CPCAP_VVIB] = {
		.constraints = {
			.min_uV			= 1300000,
			.max_uV			= 3000000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vvib_consumers),
		.consumer_supplies	= cpcap_vvib_consumers,
	},
	[CPCAP_VUSB] = {
		.constraints = {
			.min_uV			= 3300000,
			.max_uV			= 3300000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vusb_consumers),
		.consumer_supplies	= cpcap_vusb_consumers,
	},
	[CPCAP_VAUDIO] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_modes_mask	= (REGULATOR_MODE_NORMAL |
						   REGULATOR_MODE_STANDBY),
			.valid_ops_mask		= REGULATOR_CHANGE_MODE,
			.always_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vaudio_consumers),
		.consumer_supplies	= cpcap_vaudio_consumers,
	},
};

struct regulator_init_data mapphone_tps65912_regulators[] = {
	{
		.constraints = {
			.name			= "VDCDC1",
			.min_uV			= 700000,
			.max_uV			= 1487500,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
			.always_on		= true,
		},
	},
	{
		.constraints = {
			.name			= "VDCDC2",
			.min_uV			= 500000,
			.max_uV			= 3800000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
			.always_on		= true,
		},
	},
	{
		.constraints = {
			.name			= "VDCDC3",
			.min_uV			= 700000,
			.max_uV			= 1487500,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
			.always_on		= true,
		},
	},
	{
		.constraints = {
			.name			= "VDCDC4",
			.min_uV			= 700000,
			.max_uV			= 1487500,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
			.always_on	= true,
		},
	},
	{
		.constraints = {
			.name			= "LDO1",
			.min_uV			= 800000,
			.max_uV			= 3300000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name			= "LDO2",
			.min_uV			= 800000,
			.max_uV			= 3300000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name			= "LDO3",
			.min_uV			= 800000,
			.max_uV			= 3300000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name			= "LDO4",
			.min_uV			= 800000,
			.max_uV			= 3300000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name			= "LDO5",
			.min_uV			= 800000,
			.max_uV			= 3300000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name			= "LDO6",
			.min_uV			= 800000,
			.max_uV			= 3300000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name			= "LDO7",
			.min_uV			= 800000,
			.max_uV			= 3300000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name			= "LDO8",
			.min_uV			= 800000,
			.max_uV			= 3300000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name			= "LDO9",
			.min_uV			= 800000,
			.max_uV			= 3300000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
			.always_on		= true,
		},
	},
	{
		.constraints = {
			.name			= "LDO10",
			.min_uV			= 800000,
			.max_uV			= 3300000,
			.apply_uV		= true,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL
						| REGULATOR_MODE_STANDBY,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_STATUS,
		},
	},
};

static struct cpcap_adc_ato mapphone_cpcap_adc_ato = {
	.ato_in = 0x0480,
	.atox_in = 0,
	.adc_ps_factor_in = 0x0200,
	.atox_ps_factor_in = 0,
	.ato_out = 0,
	.atox_out = 0,
	.adc_ps_factor_out = 0,
	.atox_ps_factor_out = 0,
};

static void ac_changed(struct power_supply *ac,
			struct cpcap_batt_ac_data *ac_state)
{
#ifdef CONFIG_OMAP_PM_SRF
	static char requested;
	int ret = 0;

	if (!ac || !ac_state)
		return;

	if (ac_state->online) {
		/* To reduce OMAP Vdd1 DC/DC converter output voltage dips as
		 * much as possible, limit Vdd1 to OPP3-OPP5 when the phone is
		 * connected to a charger. */
		if (!requested)
			ret = resource_request("vdd1_opp", ac->dev, VDD1_OPP3);

		if (!ret)
			requested = 1;
	} else if (requested) {
		ret = resource_release("vdd1_opp", ac->dev);

		if (!ret)
			requested = 0;
	}
#endif
}

static void batt_changed(struct power_supply *batt,
			 struct cpcap_batt_data *batt_state)
{
#ifdef CONFIG_OMAP_PM_SRF
	static char requested;
	int ret = 0;

	if (!batt || !batt_state)
		return;

	if (batt_state->batt_temp < 0) {
		/* To reduce OMAP Vdd1 DC/DC converter output voltage dips as
		 * much as possible, limit Vdd1 to OPP3-OPP5 when the
		 * temperature is below 0 degrees C. */
		if (!requested)
			ret = resource_request("vdd1_opp", batt->dev, VDD1_OPP3);

		if (!ret)
			requested = 1;
	} else if (requested) {
		ret = resource_release("vdd1_opp", batt->dev);

		if (!ret)
			requested = 0;
	}
#endif
}

static struct hdmi_hpd_data mapphone_hdmi_hpd = {
	.hpd_gpio = 63,
	.check_hpd_status = omap_check_hpd_status,
};

static struct cpcap_factory_kill maphone_factory_kill = {
	.supported = 0,
	.gpio_ctrl = 9,
	.force_factory_kill = omap_force_factory_kill,
};

static struct cpcap_ind_chrg maphone_ind_chrg = {
	.supported = 0,
	.gpio_swst0 = 84,
	.gpio_swst1 = 85,
	.gpio_reven0 = 86,
	.gpio_reven1 = 87,
	.gpio_chrgterm = 121,
	.gpio_chrgcmpl = 118,
	.force_charge_complete = omap_force_charge_complete,
	.force_charge_terminate = omap_force_charge_terminate,
	.force_inductive_path = omap_force_inductive_path,
	.force_cable_path = omap_force_cable_path,
	.check_inductive_path = omap_check_inductive_path,
	.check_cable_path = omap_check_cable_path,
};

static struct cpcap_usb_mux mapphone_mux_switch = {
	.supported = 0,
	.bp_bypass_supported = 0,
	.gpio_mux_sel1 = 60,
	.gpio_mux_sel2 = 0,
	.configure_otg_muxmode =
		omap_usb_otg_muxmode,
	.configure_switch_muxmode =
		omap_usb_switch_muxmode,
};

static int feature_mdm6600_spi;

static struct cpcap_spdif_audio mapphone_spdif_audio = {
	.supported = 0,
	.gpio_spdif = 26,
	.enable_spdif_audio = omap_enable_spdif_audio,
};

static struct cpcap_usbhs_ext_pwr mapphone_usbhs_ext_pwr = {
	.supported = 0,
	.gpio_usbhs_ext_pwr = 84,
	.usbhs_ext_pwr_en = omap_enable_usbhs_extpwr,
	.gpio_oc = -1, /* invalid gpio value */
	.init = omap_usbhs_extpwr_init,
	.exit = omap_usbhs_extpwr_exit,
};

static void omap_force_factory_kill(int enabled)
{
	if (maphone_factory_kill.supported)
		gpio_set_value(maphone_factory_kill.gpio_ctrl, enabled);
}

static void omap_force_charge_complete(int closed)
{
	if (maphone_ind_chrg.supported)
		gpio_set_value(maphone_ind_chrg.gpio_chrgcmpl, closed);
}

static void omap_force_charge_terminate(int closed)
{
	if (maphone_ind_chrg.supported)
		gpio_set_value(maphone_ind_chrg.gpio_chrgterm, closed);
}

static void omap_force_inductive_path(int closed)
{
	if (maphone_ind_chrg.supported)
		gpio_set_value(maphone_ind_chrg.gpio_reven1, closed);
}

static void omap_force_cable_path(int closed)
{
	if (maphone_ind_chrg.supported)
		gpio_set_value(maphone_ind_chrg.gpio_reven0, closed);
}

static int omap_check_inductive_path(void)
{
	int val = 1; /* active low */
	if (maphone_ind_chrg.supported)
		val = gpio_get_value(maphone_ind_chrg.gpio_swst1);
	return val;
}

static int omap_check_cable_path(void)
{
	int val = 1; /* active low */
	if (maphone_ind_chrg.supported)
		val = gpio_get_value(maphone_ind_chrg.gpio_swst0);
	return val;
}

static struct cpcap_leds mapphone_cpcap_leds = {
	.display_led = {
		.display_reg = CPCAP_REG_MDLC,
		.display_mask = 0xFFFF,
		.display_off = 0xFFFA,
		.display_init = 0xB019,
		.poll_intvl = 3000,
	},
	.button_led = {
		.button_reg = CPCAP_REG_BLUEC,
		.button_mask = 0x03FF,
		.button_on = 0x00F5,
		.button_off = 0x00F4,
	},
	.kpad_led = {
		.kpad_reg = CPCAP_REG_KLC,
		.kpad_mask = 0x7FFF,
		.kpad_on = 0x5FF5,
		.kpad_off = 0x5FF0,
	},
	/* To find LUX value from ALS data,
	   below variables are used.
	    * lux_max - LUX maximum value
	    * lux_minimum - LUX minimum value
	    * als_max - Maximum ALS data
	    * als_min - Minimum ALS data */
	.als_data = {
		.lux_max = 5000,
		.lux_min = 100,
		.als_max = 590,
		.als_min = 9,
	},
};

static void write_omap_mux_register(u16 offset, u8 mode, u8 input_en)
{
	u16 tmp_val, reg_val;
	u32 reg = OMAP_CTRL_BASE + offset;

	reg_val = mode | (input_en << 8);
	tmp_val = omap_readw(reg) & ~(0x0007 | (1 << 8));
	reg_val = reg_val | tmp_val;
	if (!input_en)
		reg_val = reg_val & ~(0x4000);
	omap_writew(reg_val, reg);
}

static void omap_usb_otg_muxmode(enum cpcap_usb_otg_muxmode muxmode)
{
	u16 mux_regs_save;
	u16 gpio_mask = 0x2000;

	if (mapphone_mux_switch.supported) {
		mux_regs_save = omap_readw(OMAP_CTRL_BASE + GPIO_MODE);

		switch (muxmode) {
		case USB_OTG:
			omap_writew((mux_regs_save  & ~(gpio_mask)),
				OMAP_CTRL_BASE + GPIO_MODE);
			write_omap_mux_register(USBA0_OTG_DP, OMAP_USB_OTG, 1);
			write_omap_mux_register(USBA0_OTG_DM, OMAP_USB_OTG, 1);
			break;
		case UART_2:
			omap_writew((mux_regs_save | gpio_mask),
				OMAP_CTRL_BASE + GPIO_MODE);
			write_omap_mux_register(USBA0_OTG_DP, OMAP_UART_2, 1);
			write_omap_mux_register(USBA0_OTG_DM, OMAP_UART_2, 1);
			break;
		case SAFE_MODE:
			omap_writew((mux_regs_save  & ~(gpio_mask)),
				OMAP_CTRL_BASE + GPIO_MODE);
			write_omap_mux_register(USBA0_OTG_DP,
				OMAP_SAFE_MODE, 0);
			write_omap_mux_register(USBA0_OTG_DM,
				OMAP_SAFE_MODE, 0);
			break;
		default:
			break;
		}
	}
}

static void omap_usb_switch_muxmode(enum cpcap_usb_switch_muxmode muxmode)
{
	if (mapphone_mux_switch.supported) {
		switch (muxmode) {
		case CPCAP_DM_DP:
			if (mapphone_mux_switch.bp_bypass_supported)
				gpio_set_value(
					mapphone_mux_switch.gpio_mux_sel1, 0);
			gpio_set_value(mapphone_mux_switch.gpio_mux_sel2, 0);
			break;
		case MDM_RX_TX:
			if (mapphone_mux_switch.bp_bypass_supported)
				gpio_set_value(
					mapphone_mux_switch.gpio_mux_sel1, 1);
			gpio_set_value(mapphone_mux_switch.gpio_mux_sel2, 0);
			break;
		case OTG_DM_DP:
			gpio_set_value(mapphone_mux_switch.gpio_mux_sel2, 1);
			if (mapphone_mux_switch.bp_bypass_supported)
				gpio_set_value(
					mapphone_mux_switch.gpio_mux_sel1, 1);
			break;
		default:
			break;
		}
	}
}

static void omap_enable_spdif_audio(int status)
{
	if (mapphone_spdif_audio.supported)
		gpio_set_value(mapphone_spdif_audio.gpio_spdif, status);

	printk(KERN_INFO "%s: %s spdif audio!\n", __func__,
		(status ? "Enabled" : "Disabled"));
}

static void omap_enable_usbhs_extpwr(int val)
{
	if (mapphone_usbhs_ext_pwr.supported)
		gpio_set_value(mapphone_usbhs_ext_pwr.gpio_usbhs_ext_pwr, val);
}

static irqreturn_t usbhs_extpwr_oc_irq(int irq, void *priv)
{
	/* disable host power */
	((struct cpcap_usbhs_ext_pwr *)priv)->usbhs_ext_pwr_en(0);

	printk(KERN_INFO "USB HOST POWER Over-current detected\n");

	return IRQ_HANDLED;
}

static int omap_usbhs_extpwr_init(struct cpcap_usbhs_ext_pwr *p)
{
	int ret = -EPERM;
	if (gpio_is_valid(p->gpio_oc)) {
		int irq_oc = -1;

		irq_oc = gpio_to_irq(p->gpio_oc);
		if (irq_oc < 0)
			goto bail_gpio;

		ret = request_irq(irq_oc,
				  usbhs_extpwr_oc_irq,
				  /* active low */
				  IRQF_TRIGGER_FALLING | IRQF_DISABLED,
				  "usbhs_extpwr_oc",
				  &mapphone_usbhs_ext_pwr);
		if (ret)
			goto bail_gpio;
	}

	return 0;

 bail_gpio:
	printk(KERN_ERR "USB HOST POWER : "
	       "Fail to initialize with given GPIO value! (%d)\n", ret);
	return -EPERM;
}

static void omap_usbhs_extpwr_exit(struct cpcap_usbhs_ext_pwr *p)
{
	free_irq(gpio_to_irq(p->gpio_oc),
		 &mapphone_usbhs_ext_pwr);
}

static char omap_check_hpd_status(void)
{
	char status = 0;
	status = gpio_get_value(mapphone_hdmi_hpd.hpd_gpio);
	printk(KERN_ERR "%s: LID %s !\n", __func__,
		(status ? "Opened" : "Closed"));
	return status;
}

int mapphone_cpcap_irq_pending(int irq);

static struct cpcap_platform_data mapphone_cpcap_data = {
	.init = mapphone_cpcap_spi_init,
	.regulator_mode_values = cpcap_regulator_mode_values,
	.regulator_off_mode_values = cpcap_regulator_off_mode_values,
	.regulator_init = cpcap_regulator,
	.adc_ato = &mapphone_cpcap_adc_ato,
	.leds = &mapphone_cpcap_leds,
	.ac_changed = NULL,
	.batt_changed = batt_changed,
	.usb_changed = NULL,
	.is_umts = 0,
	.usb_drv_ctrl_via_ulpi = STATUS_SUPPORTED,
	.irq_pending = mapphone_cpcap_irq_pending,
	.ind_chrg = &maphone_ind_chrg,
	.usb_mux = &mapphone_mux_switch,
	.spdif_audio = &mapphone_spdif_audio,
	.ste_disable_low_freq_limit_hs_charge_pump = false,
	.usbhs_ext_pwr = &mapphone_usbhs_ext_pwr,
	.factory_kill = &maphone_factory_kill,
	.hpd_status = &mapphone_hdmi_hpd,
};

static struct mdm6600_spi_platform_data mapphone_mdm6600_data = {
	.gpio_mrdy = 115,
	.gpio_srdy = 114,
};

static struct spi_board_info mapphone_spi_board_info[] __initdata = {
	{
		.modalias = "cpcap",
		.bus_num = 1,
		.chip_select = 0,
		.max_speed_hz = 3000000,
		.platform_data = &mapphone_cpcap_data,
		.mode = SPI_CS_HIGH,
	},
	/* Add All other SPI Entries before this */
	{  /* additional record for BP IPC SPI */
		.modalias = "mdm6600_spi",
		.bus_num = 2,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz = 18000000,
		.platform_data = &mapphone_mdm6600_data,
		.irq = 0,
	},
};

#if 0 /* XXX */
static struct tps65912_platform_data mapphone_tps65912_data = {
	.tps65912_pmic_init_data = mapphone_tps65912_regulators,
	.write_only = false,
	.is_dcdc1_avs = true,
	.is_dcdc2_avs = false,
	.is_dcdc3_avs = true,
	.is_dcdc4_avs = true,
};

static struct spi_board_info mapphone_spi_board_info_tps65912 __initdata = {
	/* TPS65912 pmic configuration */
	.modalias       = "tps65912",
	.bus_num        = 1,
	.chip_select    = 1,
	.max_speed_hz   = 3000000,
	.platform_data  = &mapphone_tps65912_data,
	.mode = SPI_CS_HIGH,
};
#endif


struct omap_spi_init_entry {
	u32 reg;
	u32 data;
} __attribute__ ((__packed__));

struct omap_rgt_mode_entry {
	u32 id;
	u16 data;
} __attribute__ ((__packed__));

struct omap_rgt_init_entry {
	u32 id;
	u32 min_uV;
	u32 max_uV;
	u32 valid_ops_mask;
	u8 always_on;
	u8 boot_on;
	u8 apply_uV;
} __attribute__ ((__packed__));

struct omap_ind_chrg_entry {
	u8 supported;
	u8 gpio_swst0;
	u8 gpio_swst1;
	u8 gpio_reven0;
	u8 gpio_reven1;
	u8 gpio_chrgterm;
	u8 gpio_chrgcmpl;
} __attribute__ ((__packed__));

struct omap_whisper_entry {
	u8 supported;
	u8 bp_bypass_supported;
	u8 gpio_mux_sel1;
	u8 gpio_mux_sel2;
} __attribute__ ((__packed__));

struct omap_spdif_audio_entry {
	u8 supported;
	u8 gpio_spdif;
} __attribute__ ((__packed__));

struct omap_usbhs_ext_pwr_entry {
	u8 supported;
	u8 gpio_usbhs_ext_pwr;
} __attribute__ ((__packed__));

static void spdif_audio_init(void *p_data)
{
	struct omap_spdif_audio_entry *p = p_data;

	mapphone_spdif_audio.supported = p->supported;
	mapphone_spdif_audio.gpio_spdif = p->gpio_spdif;
	pr_debug("SPDIF Supported? [%d]!\n", p->supported);
	pr_debug("SPDIF spdif_audio [%d]!\n", p->gpio_spdif);
}

static void usbhs_ext_pwr_init(void *p_data)
{
	struct omap_usbhs_ext_pwr_entry *p = p_data;

	mapphone_usbhs_ext_pwr.supported = p->supported;
	mapphone_usbhs_ext_pwr.gpio_usbhs_ext_pwr = p->gpio_usbhs_ext_pwr;
	pr_debug("Ext Power for USBHS Supported? [%d]!\n", p->supported);
	pr_debug("Ext Power for USBHS EN gpio [%d]!\n", p->gpio_usbhs_ext_pwr);
}

static void whisper_init(void *p_data)
{
	struct omap_whisper_entry *p = p_data;

	mapphone_mux_switch.supported = p->supported;
	mapphone_mux_switch.bp_bypass_supported = p->bp_bypass_supported;
	mapphone_mux_switch.gpio_mux_sel1 = p->gpio_mux_sel1;
	mapphone_mux_switch.gpio_mux_sel2 = p->gpio_mux_sel2;
	printk(KERN_INFO "CPCAP: WHISPER Supported? [%d]!\n", p->supported);
	printk(KERN_INFO "CPCAP: WHISPER BP BYPASS Supported? [%d]!\n",
		p->bp_bypass_supported);
	printk(KERN_INFO "CPCAP: WHISPER mux_sel1 [%d]!\n", p->gpio_mux_sel1);
	printk(KERN_INFO "CPCAP: WHISPER mux_sel2 [%d]!\n", p->gpio_mux_sel2);
}

static void ind_chrg_init(void *p_data)
{
	struct omap_ind_chrg_entry *p = p_data;

	maphone_ind_chrg.supported = p->supported;
	maphone_ind_chrg.gpio_swst0 = p->gpio_swst0;
	maphone_ind_chrg.gpio_swst1 = p->gpio_swst1;
	maphone_ind_chrg.gpio_reven0 = p->gpio_reven0;
	maphone_ind_chrg.gpio_reven1 = p->gpio_reven1;
	maphone_ind_chrg.gpio_chrgterm = p->gpio_chrgterm;
	maphone_ind_chrg.gpio_chrgcmpl = p->gpio_chrgcmpl;
	printk(KERN_INFO "CPCAP: INDUCTIVE Supported? [%d]!\n", p->supported);
	printk(KERN_INFO "CPCAP: INDUCTIVE swst0 [%d]!\n", p->gpio_swst0);
	printk(KERN_INFO "CPCAP: INDUCTIVE swst1 [%d]!\n", p->gpio_swst1);
	printk(KERN_INFO "CPCAP: INDUCTIVE reven0 [%d]!\n", p->gpio_reven0);
	printk(KERN_INFO "CPCAP: INDUCTIVE reven1 [%d]!\n", p->gpio_reven1);
	printk(KERN_INFO "CPCAP: INDUCTIVE chrgterm [%d]!\n", p->gpio_chrgterm);
	printk(KERN_INFO "CPCAP: INDUCTIVE chrgcmpl [%d]!\n", p->gpio_chrgcmpl);
}

static void regulator_init(void *p_data)
{
	struct omap_rgt_init_entry *p = p_data;
	struct regulator_init_data *p_devs = cpcap_regulator;

	if (p->id < CPCAP_NUM_REGULATORS) {
		p_devs[p->id].constraints.min_uV = p->min_uV;
		p_devs[p->id].constraints.max_uV = p->max_uV;
		p_devs[p->id].constraints.valid_ops_mask = p->valid_ops_mask;
		p_devs[p->id].constraints.always_on = p->always_on;
		p_devs[p->id].constraints.boot_on = p->boot_on;
		p_devs[p->id].constraints.apply_uV = p->apply_uV;
		printk(KERN_INFO "CPCAP: Overwrite regulator init [%d]!\n",
				p->id);
	} else {
		printk(KERN_ERR "CPCAP: Too big cpcap regulator count!\n");
	}
}

static void regulator_mode_init(void *p_data)
{
	struct omap_rgt_mode_entry *p = p_data;
	unsigned short *p_devs = cpcap_regulator_mode_values;

	if (p->id < CPCAP_NUM_REGULATORS) {
		p_devs[p->id] = p->data;
		printk(KERN_INFO "CPCAP: Overwrite regulator mode [%d]!\n",
				p->id);
	} else {
		printk(KERN_ERR "CPCAP: Too big cpcap regulator count!\n");
	}
}

static void regulator_off_mode_init(void *p_data)
{
	struct omap_rgt_mode_entry *p = p_data;
	unsigned short *p_devs = cpcap_regulator_off_mode_values;

	if (p->id < CPCAP_NUM_REGULATORS) {
		p_devs[p->id] = p->data;
		printk(KERN_INFO "CPCAP: Overwrite regulator off mode [%d]!\n",
				p->id);
	} else {
		printk(KERN_ERR "CPCAP: Too big cpcap regulator count!\n");
	}
}

static void configure_pbias(void)
{
	u32 reg;

	reg = omap_readl(OMAP4_CTRL_MODULE_PAD_WKUP|
		OMAP4_CTRL_MODULE_PAD_WKUP_CONTROL_USIMIO);
	omap_writel((reg|OMAP4_USIM_PWRDNZ_MASK)&
		~(OMAP4_PAD_USIM_CLK_LOW_MASK|OMAP4_PAD_USIM_RST_LOW_MASK),
		OMAP4_CTRL_MODULE_PAD_WKUP|
		OMAP4_CTRL_MODULE_PAD_WKUP_CONTROL_USIMIO);

	reg = omap_readl(OMAP4_CTRL_MODULE_PAD_CORE|
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE);
	omap_writel((reg|OMAP4_USIM_PBIASLITE_PWRDNZ_MASK)&
		~(OMAP4_USIM_PBIASLITE_HIZ_MODE_MASK|
		OMAP4_USIM_PBIASLITE_VMODE_MASK)
		, OMAP4_CTRL_MODULE_PAD_CORE|
		OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE);
}

static void cpcap_spi_init(void *p_data)
{
	struct omap_spi_init_entry *p = p_data;
	struct cpcap_spi_init_data *p_devs = mapphone_cpcap_spi_init;
	int i = 0;

	for (i = 0; i < CPCAP_REG_SIZE + 1; i++) {
		if (p_devs[i].reg == CPCAP_REG_UNUSED) {
			p_devs[i].reg = p->reg;
			p_devs[i].data = p->data;

			if (i != CPCAP_REG_SIZE)
				p_devs[i + 1].reg = CPCAP_REG_UNUSED;

			printk(KERN_INFO "CPCAP: Add new reg [%d] setting!\n",
					p->reg);
			return;
		}

		if (p_devs[i].reg == p->reg) {
			p_devs[i].data = p->data;

			printk(KERN_INFO "CPCAP: Overwrite reg [%d] setting!\n",
					p->reg);
			return;
		}

		if (i == CPCAP_REG_SIZE)
			printk(KERN_ERR "CPCAP: Too big cpcap reg count!\n");
	}
}

static void __init cpcap_of_init(void)
{
	int size, unit_size, i, count;
	struct device_node *node;
	const void *prop;
	struct device_node *bp_node;
	struct device_node *hpd_node;
	const void *bp_prop;
	const void *hpd_prop;
	char *cpcap_bp_model = "CDMA";

	bp_node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (bp_node) {
		bp_prop = of_get_property(bp_node, DT_PROP_CHOSEN_BP, NULL);
		if (bp_prop)
			cpcap_bp_model = (char *)bp_prop;

		of_node_put(bp_node);
	}

	if (strcmp(cpcap_bp_model, "UMTS") >= 0)
		mapphone_cpcap_data.is_umts = 1;

	node = of_find_node_by_path(DT_PATH_CPCAP);
	if (node == NULL) {
		printk(KERN_ERR
				"Unable to read node %s from device tree!\n",
				DT_PATH_CPCAP);
		return;
	}

	prop = of_get_property(node, DT_PROP_CPCAP_BUSNUM, NULL);
	if (prop) {
		mapphone_spi_board_info[0].bus_num = *(u16 *)prop;

		printk(KERN_INFO "CPCAP: overwriting bus_num with %d\n", \
			mapphone_spi_board_info[0].bus_num);
	} else
		printk(KERN_INFO "CPCAP: using default bus_num %d\n", \
			mapphone_spi_board_info[0].bus_num);

	prop = of_get_property(node, DT_PROP_CPCAP_WIRELESSCHRG, &size);
	if (prop)
		ind_chrg_init((struct cpcap_ind_chrg *)prop);
	else {
		printk(KERN_ERR "Read property %s error!\n",
				DT_PROP_CPCAP_WIRELESSCHRG);
	}

	prop = of_get_property(node, DT_PROP_CPCAP_WHISPER, &size);
	if (prop)
		whisper_init((struct cpcap_usb_mux *)prop);
	else {
		printk(KERN_ERR "Read property %s error!\n",
				DT_PROP_CPCAP_WHISPER);
	}

	prop = of_get_property(node, DT_PROP_CPCAP_SPDIFAUDIO, &size);
	if (prop)
		spdif_audio_init((struct cpcap_spdif_audio *)prop);
	else {
		printk(KERN_ERR "Read property %s error!\n",
				DT_PROP_CPCAP_SPDIFAUDIO);
	}

	prop = of_get_property(node,
		DT_PROP_CPCAP_STE_MODE_HS_WORKAROUND, &size);
	if (prop)
		mapphone_cpcap_data.ste_disable_low_freq_limit_hs_charge_pump
				= *(bool *)(prop);
	else
		mapphone_cpcap_data.ste_disable_low_freq_limit_hs_charge_pump
				= false;

	prop = of_get_property(node, DT_PROP_CPCAP_USBHS_EXTPWR, &size);
	if (prop)
		usbhs_ext_pwr_init((struct cpcap_usbhs_ext_pwr *)prop);
	else {
		printk(KERN_ERR "Read property %s error!\n",
				DT_PROP_CPCAP_USBHS_EXTPWR);
	}

	hpd_node = of_find_node_by_path(DT_PATH_DISPLAY2);
	if (hpd_node) {
		hpd_prop = of_get_property(hpd_node, "gpio_hpd", NULL);
		if (hpd_prop)
			mapphone_hdmi_hpd.hpd_gpio = *(u32 *)hpd_prop;
		of_node_put(hpd_node);
	}

	unit_size = sizeof(struct omap_spi_init_entry);
	prop = of_get_property(node, DT_PROP_CPCAP_SPIINIT, &size);
	if (prop) {
		if (!(size % unit_size)) {
			count = size / unit_size;
			printk(KERN_INFO "cpcap init size = %d\n", count);

			for (i = 0; i < count; i++)
				cpcap_spi_init((struct omap_spi_init_entry *)
					       prop + i);
		} else {
			printk(KERN_ERR "Read property %s error!\n",
					DT_PROP_CPCAP_SPIINIT);
			of_node_put(node);
			return;
		}
	}

	unit_size = sizeof(struct omap_rgt_init_entry);
	prop = of_get_property(node, DT_PROP_CPCAP_RGTINIT, &size);
	if (prop) {
		if (!(size % unit_size)) {
			count = size / unit_size;
			printk(KERN_INFO "cpcap init size = %d\n", count);

			for (i = 0; i < count; i++)
				regulator_init((struct omap_rgt_init_entry *)
					       prop + i);
		} else {
			printk(KERN_ERR "Read property %s error!\n",
					DT_PROP_CPCAP_RGTINIT);
			of_node_put(node);
			return;
		}
	}

	unit_size = sizeof(struct omap_rgt_mode_entry);
	prop = of_get_property(node, DT_PROP_CPCAP_RGTMODE, &size);
	if (prop) {
		if (!(size % unit_size)) {
			count = size / unit_size;
			printk(KERN_INFO "cpcap init size = %d\n", count);

			for (i = 0; i < count; i++)
				regulator_mode_init(
					(struct omap_rgt_mode_entry *)prop + i);
		} else {
			printk(KERN_ERR "Read property %s error!\n",
					DT_PROP_CPCAP_RGTMODE);
			of_node_put(node);
			return;
		}
	}

	unit_size = sizeof(struct omap_rgt_mode_entry);
	prop = of_get_property(node, DT_PROP_CPCAP_RGTOFFMODE, &size);
	if (prop) {
		if (!(size % unit_size)) {
			count = size / unit_size;
			printk(KERN_INFO "cpcap init size = %d\n", count);

			for (i = 0; i < count; i++)
				regulator_off_mode_init(
					(struct omap_rgt_mode_entry *)prop + i);
		} else {
			printk(KERN_ERR "Read property %s error!\n",
					DT_PROP_CPCAP_RGTOFFMODE);
			of_node_put(node);
			return;
		}
	}

	of_node_put(node);

	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (node) {
		prop = of_get_property(node, "feature_mdm6600_spi", &size);
		if (prop && size &&
				(bi_powerup_reason() != PU_REASON_CHARGER)) {
			feature_mdm6600_spi = *(u8 *)prop;
			pr_debug("%s MDM6600 SPI in the Platform\n",
				feature_mdm6600_spi ? "Enabling" : "Disabling");
		}
		of_node_put(node);
	}
	return;
}

int mapphone_cpcap_irq_pending(int irq)
{
	return gpio_get_value(irq_to_gpio(irq));
}

static void pmu_of_init(void)
{
	struct device_node *node;
	const void *prop;
	int size;

	node = of_find_node_by_path(DT_PATH_PMU);
	if (node) {
		prop = of_get_property(node, "type", &size);
		if (prop && size) {
			switch (*(int *)prop) {
			case 0x00030011: /* TPS65912 */
				printk(KERN_INFO "TPS65912: Found!\n");
				prop = of_get_property(node, "write_only", NULL);
				if (prop && *(u8 *)prop) {
					printk(KERN_INFO "TPS65912: write only\n");
					/* XXX mapphone_tps65912_data.write_only = true; */
				}
				/* XXX spi_register_board_info(&mapphone_spi_board_info_tps65912, 1); */

				break;

			default:
				break;
			}
		}

	}

	of_node_put(node);
}

void __init mapphone_spi_init(char *boot_mode)
{
	int irq;
	int ret;
	int i;

	cpcap_of_init();

	configure_pbias();

	for (i = 0; i < CPCAP_REG_SIZE; i++) {
		if (mapphone_cpcap_spi_init[i].reg == CPCAP_REG_UNUSED)
			break;
	}
	mapphone_cpcap_data.init_len = i;

	ret = gpio_request(CPCAP_GPIO, "cpcap-irq");
	if (ret)
		return;
	ret = gpio_direction_input(CPCAP_GPIO);
	if (ret)
		goto free_gpio_cpcap_exit;

	if (maphone_ind_chrg.supported) {
		ret = gpio_request(maphone_ind_chrg.gpio_chrgcmpl,
			"cpcap-ind-chrgcmpl");
		if (ret)
			goto free_gpio_cpcap_exit;
		ret = gpio_direction_output(maphone_ind_chrg.gpio_chrgcmpl, 0);
		if (ret)
			goto free_gpio_chrgcmpl_exit;

		ret = gpio_request(maphone_ind_chrg.gpio_chrgterm,
			"cpcap-ind-chrgterm");
		if (ret)
			goto free_gpio_chrgcmpl_exit;
		ret = gpio_direction_output(maphone_ind_chrg.gpio_chrgterm, 0);
		if (ret)
			goto free_gpio_chrgterm_exit;

		ret = gpio_request(maphone_ind_chrg.gpio_swst0,
			"cpcap-ind-swst0");
		if (ret)
			goto free_gpio_chrgterm_exit;
		ret = gpio_direction_input(maphone_ind_chrg.gpio_swst0);
		if (ret)
			goto free_gpio_swst0_exit;

		ret = gpio_request(maphone_ind_chrg.gpio_swst1,
			"cpcap-ind-swst1");
		if (ret)
			goto free_gpio_swst0_exit;
		ret = gpio_direction_input(maphone_ind_chrg.gpio_swst1);
		if (ret)
			goto free_gpio_swst1_exit;

		ret = gpio_request(maphone_ind_chrg.gpio_reven0,
			"cpcap-ind-reven0");
		if (ret)
			goto free_gpio_swst1_exit;
		ret = gpio_direction_output(maphone_ind_chrg.gpio_reven0, 0);
		if (ret)
			goto free_gpio_reven0_exit;

		ret = gpio_request(maphone_ind_chrg.gpio_reven1,
			"cpcap-ind-reven1");
		if (ret)
			goto free_gpio_reven0_exit;
		ret = gpio_direction_output(maphone_ind_chrg.gpio_reven1, 0);
		if (ret)
			goto free_gpio_reven1_exit;
	}

	if (mapphone_mux_switch.supported) {
		if (mapphone_mux_switch.bp_bypass_supported) {
			ret = gpio_request(mapphone_mux_switch.gpio_mux_sel1,
				"ap_usb_otg_en1");
			if (ret)
				goto free_gpio_reven1_exit;
			ret = gpio_direction_output(
				mapphone_mux_switch.gpio_mux_sel1, 0);
			if (ret)
				goto free_gpio_mux_sel1;
		}

		ret = gpio_request(mapphone_mux_switch.gpio_mux_sel2,
			"ap_usb_otg_en2");
		if (ret)
				goto free_gpio_mux_sel1;

		ret = gpio_direction_output(
			mapphone_mux_switch.gpio_mux_sel2, 0);
		if (ret)
			goto free_gpio_mux_sel2;
	}

	if (mapphone_spdif_audio.supported) {
		ret = gpio_request(mapphone_spdif_audio.gpio_spdif,
			"spdif_en");
		if (ret)
			goto free_gpio_mux_sel2;
		ret = gpio_direction_output(mapphone_spdif_audio.gpio_spdif, 0);
		if (ret)
			goto free_gpio_spdif_exit;
	}

	if (mapphone_usbhs_ext_pwr.supported) {
		int gpio;
		ret = gpio_request(mapphone_usbhs_ext_pwr.gpio_usbhs_ext_pwr,
			"usbhs_ext_pwr_en");
		if (ret)
			goto free_gpio_spdif_exit;
		ret = gpio_direction_output(
			mapphone_usbhs_ext_pwr.gpio_usbhs_ext_pwr, 0);
		if (ret)
			goto free_gpio_usbhs_extpwr_exit;

		/* Setup over-current detection */
		gpio = get_gpio_by_name("usbhs_extpwr_oc");

		/*
		 * OC detection may not be supported at some product
		 *   even if ext power is supported.
		 * We can tell this by reading gpio value which label is
		 *   'usbhs_extpwr_oc'.
		 * If there is no valid value for the label,
		 *   we assume that OC is not supported.
		 *   (But it's not error-case.)
		 */
		if (gpio_is_valid(gpio)) {
			if (gpio_request(gpio, "usbhs_extpwr_oc"))
				goto free_gpio_usbhs_extpwr_exit;
			if (gpio_direction_input(gpio)) {
				gpio_free(gpio);
				goto free_gpio_usbhs_extpwr_exit;
			}
			mapphone_usbhs_ext_pwr.gpio_oc = gpio;
		}
	}

	ret = get_gpio_by_name("fact_kill_override");
	if (ret >= 0) {
		maphone_factory_kill.supported = 1;
		maphone_factory_kill.gpio_ctrl = ret;
	}

	if (!strncmp(boot_mode, "ap-bp-bypass",
					BOOT_MODE_MAX_LEN)) {
		omap_usb_switch_muxmode(MDM_RX_TX);
		mapphone_mux_switch.supported = 0;
		mapphone_mux_switch.bp_bypass_supported = 0;
	}

	goto finish_mapphone_spi_init;

free_gpio_usbhs_extpwr_exit:
	if (mapphone_usbhs_ext_pwr.supported)
		gpio_free(mapphone_usbhs_ext_pwr.gpio_usbhs_ext_pwr);
free_gpio_spdif_exit:
	if (mapphone_spdif_audio.supported)
		gpio_free(mapphone_spdif_audio.gpio_spdif);
free_gpio_mux_sel2:
	if (mapphone_mux_switch.supported)
		gpio_free(mapphone_mux_switch.gpio_mux_sel2);
free_gpio_mux_sel1:
	if (mapphone_mux_switch.bp_bypass_supported)
		gpio_free(mapphone_mux_switch.gpio_mux_sel1);
free_gpio_reven1_exit:
	if (maphone_ind_chrg.supported)
		gpio_free(maphone_ind_chrg.gpio_reven1);
	else
		goto free_gpio_cpcap_exit;
free_gpio_reven0_exit:
	gpio_free(maphone_ind_chrg.gpio_reven0);
free_gpio_swst1_exit:
	gpio_free(maphone_ind_chrg.gpio_swst1);
free_gpio_swst0_exit:
	gpio_free(maphone_ind_chrg.gpio_swst0);
free_gpio_chrgterm_exit:
	gpio_free(maphone_ind_chrg.gpio_chrgterm);
free_gpio_chrgcmpl_exit:
	gpio_free(maphone_ind_chrg.gpio_chrgcmpl);
free_gpio_cpcap_exit:
	gpio_free(CPCAP_GPIO);
	printk(KERN_ERR "Request cpcap gpio failed error!\n");
	return;

finish_mapphone_spi_init:
	irq = gpio_to_irq(CPCAP_GPIO);

	mapphone_spi_board_info[0].irq = irq;

	if (feature_mdm6600_spi)
		spi_register_board_info(mapphone_spi_board_info,
					ARRAY_SIZE(mapphone_spi_board_info));
	else
		spi_register_board_info(mapphone_spi_board_info,
					ARRAY_SIZE(mapphone_spi_board_info)-1);

	pmu_of_init();

	/* regulator_has_full_constraints(); */
}
