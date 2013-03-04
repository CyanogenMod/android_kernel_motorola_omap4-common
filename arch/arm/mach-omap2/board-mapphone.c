/*
 * Board support file for OMAP4-based Motorola Android Platform devices.
 *
 * Copyright 2011 Motorola Mobility, Inc.
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/usb/otg.h>
#include <linux/hwspinlock.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/omapfb.h>
#include <linux/reboot.h>
#include <linux/wl12xx.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>
#include <linux/of_fdt.h>
#include <linux/of.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/dmm.h>

#include <asm/mach-types.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/omap-serial.h>
#include <plat/remoteproc.h>
#include <video/omapdss.h>
#include <video/omap-panel-nokia-dsi.h>
#include <plat/omap-pm.h>
#include <plat/hdq.h>
#include <plat/cpu.h>

#include <mach/system.h>
#include <linux/gpio_mapping.h>

#include <../drivers/w1/w1_family.h> /* for W1_EEPROM_DS2502 */

#include <linux/wakelock.h>
#include "dt_path.h"
#include "board-mapphone.h"
#include "board-mapphone-sensors.h"
#include "board-mapphone-padconf.h"
#include "omap4_ion.h"
#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "common-board-devices.h"
#include "pm.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "omap_ram_console.h"
/* for TI WiLink devices */
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <plat/omap-serial.h>
#include <plat/omap_hsi.h>
#include <linux/wl12xx.h>

#ifdef CONFIG_SND_LM48901_AMP
#include <linux/lm48901.h>
#endif

#include <plat/board-mapphone-emu_uart.h>

#define WILINK_UART_DEV_NAME "/dev/ttyO3"

#define MAPPHONE_POWER_OFF_GPIO         176

#define TPS62361_GPIO   7

char *bp_model = "CDMA";
static unsigned long mapphone_wifi_pmena_gpio;
static unsigned long mapphone_wifi_irq_gpio;

static char boot_mode[BOOT_MODE_MAX_LEN+1];

int __init board_boot_mode_init(char *s)
{
	strncpy(boot_mode, s, BOOT_MODE_MAX_LEN);
	boot_mode[BOOT_MODE_MAX_LEN] = '\0';
	pr_debug("boot_mode=%s\n", boot_mode);
	return 1;
}
__setup("androidboot.mode=", board_boot_mode_init);

/* Flat dev tree address */
#define ATAG_FLAT_DEV_TREE_ADDRESS 0xf100040A
struct tag_flat_dev_tree_address {
	u32 address;
	u32 size;
};

static u32 fdt_start_address;
static u32 fdt_size;

/* process flat device tree for hardware configuration */
static int __init parse_tag_flat_dev_tree_address(const struct tag *tag)
{
	struct tag_flat_dev_tree_address *fdt_addr =
		(struct tag_flat_dev_tree_address *)&tag->u;

	if (fdt_addr->size) {
		fdt_start_address = (u32)phys_to_virt(fdt_addr->address);
		fdt_size = fdt_addr->size;
	}

	/*have_of = 1;*/
	printk(KERN_INFO
		"flat_dev_tree_address=0x%08x, flat_dev_tree_size == 0x%08X\n",
		fdt_addr->address,
		fdt_addr->size);

	return 0;
}

__tagtable(ATAG_FLAT_DEV_TREE_ADDRESS, parse_tag_flat_dev_tree_address);

/* TODO: handle suspend/resume here.
 * Upon every suspend, make sure the wilink chip is capable enough to wake-up the
 * OMAP host.
 */
static int plat_wlink_kim_suspend(struct platform_device *pdev, pm_message_t
		state)
{
	return 0;
}

static int plat_wlink_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static bool uart_req;
static struct wake_lock st_wk_lock;
/* Call the uart disable of serial driver */
static int plat_uart_disable(void)
{
	int port_id = 0;
	int err = 0;
	if (uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_disable(port_id);
		if (!err)
			uart_req = false;
	}
	wake_unlock(&st_wk_lock);
	return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
	int port_id = 0;
	int err = 0;
	if (!uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_enable(port_id);
		if (!err)
			uart_req = true;
	}
	wake_lock(&st_wk_lock);
	return err;
}

static struct wl12xx_platform_data omap4_mapphone_wlan_data __initdata = {
	.irq = -1, /* OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),*/
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
};

int wifi_set_power(struct device *dev, int slot, int power_on, int vdd)
{
	static int power_state;
	pr_debug("Powering %s wifi", (power_on ? "on" : "off"));
	if (power_on == power_state)
		return 0;
	power_state = power_on;
	if (power_on) {
		gpio_set_value(mapphone_wifi_pmena_gpio, 1);
		mdelay(15);
		gpio_set_value(mapphone_wifi_pmena_gpio, 0);
		mdelay(1);
		gpio_set_value(mapphone_wifi_pmena_gpio, 1);
		mdelay(70);
	} else
		gpio_set_value(mapphone_wifi_pmena_gpio, 0);
	return 0;
}
static void omap4_mapphone_wifi_init(void)
{
	int ret;
	mapphone_wifi_pmena_gpio = get_gpio_by_name("wlan_pmena");
	mapphone_wifi_irq_gpio = get_gpio_by_name("wlan_irqena");
	ret = gpio_request(mapphone_wifi_pmena_gpio, "wifi_pmena");
	if (ret < 0)
		goto out;
	gpio_direction_output(mapphone_wifi_pmena_gpio, 0);
	omap4_mapphone_wlan_data.irq = OMAP_GPIO_IRQ(mapphone_wifi_irq_gpio);
	if (wl12xx_set_platform_data(&omap4_mapphone_wlan_data))
		pr_err("Error setting wl12xx data\n");
out:
	return;
}
/* wl128x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 174,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3686400,
	.suspend = plat_wlink_kim_suspend,
	.resume = plat_wlink_kim_resume,
	.chip_asleep = plat_uart_disable,
	.chip_awake  = plat_uart_enable,
	.chip_enable = plat_uart_enable,
	.chip_disable = plat_uart_disable,
};

static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device *mapphone_devices[] __initdata = {
	&wl128x_device,
	&btwilink_device,
};

static struct omap_board_config_kernel mapphone_config[] __initdata = {
};

static void __init mapphone_bp_model_init(void)
{
	struct device_node *bp_node;
	const void *bp_prop;

	bp_node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (bp_node) {
		bp_prop = of_get_property(bp_node, DT_PROP_CHOSEN_BP, NULL);
		if (bp_prop)
			bp_model = (char *)bp_prop;

		of_node_put(bp_node);
	}
}

static void __init mapphone_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif

	if (fdt_start_address) {
		struct device_node *machine_node;
		const void *machine_prop;
		const void *cpu_tier_prop;
		void *mem;

		mem = __alloc_bootmem(fdt_size, __alignof__(int), 0);
		BUG_ON(!mem);
		memcpy(mem, (const void *)fdt_start_address, fdt_size);
		initial_boot_params = (struct boot_param_header *)mem;
		pr_info("Unflattening device tree: 0x%08x\n", (u32)mem);
		unflatten_device_tree();

		machine_node = of_find_node_by_path(DT_PATH_MACHINE);
		if (machine_node) {
			machine_prop = of_get_property(machine_node,
				DT_PROP_MACHINE_TYPE, NULL);
			if (machine_prop)
				set_machine_name((const char *)machine_prop);

			cpu_tier_prop = of_get_property(machine_node,
				DT_PROP_CPU_TIER, NULL);
			if (cpu_tier_prop)
				set_cpu_tier((const char *)cpu_tier_prop);

			of_node_put(machine_node);
		}

	}
}

static void omap4_audio_conf(void)
{
}

static struct i2c_board_info __initdata
	mapphone_i2c_bus1_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus2_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus3_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus4_board_info[I2C_BUS_MAX_DEVICES];

#ifdef CONFIG_SND_LM48901_AMP
struct lm48901_platform_data lm48901_pdata = {
	.amp_en_gpio = -1,
	.i2c_en_gpio = -1,
};
#endif

static struct i2c_board_info __initdata
	mapphone_i2c_1_boardinfo[] = {
		{
			I2C_BOARD_INFO("tmp105", 0x48),
		},
#ifdef CONFIG_LEDS_LP8550
		{
			I2C_BOARD_INFO(LD_LP8550_NAME, 0x2c),
			.platform_data = &mp_lp8550_pdata,
		},
#endif
#ifdef CONFIG_BACKLIGHT_LM3532
		{
			I2C_BOARD_INFO("lm3532", 0x38),
			.platform_data = &mp_lm3532_pdata,
		},
#endif
#ifdef CONFIG_SND_LM48901_AMP
		{
			I2C_BOARD_INFO("lm48901", 0x30),
			.platform_data = &lm48901_pdata,
		},
#endif
		{
			I2C_BOARD_INFO("lvds_panel", 0x50),
			.platform_data = NULL,
		},
};

static struct i2c_board_info __initdata
	mapphone_i2c_2_boardinfo[] = {
		{
			I2C_BOARD_INFO("invalid_touch_panel", 0x02),
			.platform_data = NULL,
			.irq = OMAP_GPIO_IRQ(183),  /* Legacy val */
		},
		{
			I2C_BOARD_INFO("invalid_touch_btn", 0x02),
			.platform_data = NULL,
		},
		{
			I2C_BOARD_INFO("isl29030_als_ir", 0x44),
			.platform_data = &mp_isl29030_pdata,
		},
		{
			I2C_BOARD_INFO(LD_CT405_NAME, 0x39),
			.platform_data = &mp_ct405_pdata,
		},
#ifdef CONFIG_INPUT_MAX9635
		{
			I2C_BOARD_INFO("max9635", 0x4b),
			.platform_data = &mp_max9635_pdata,
			.irq = OMAP_GPIO_IRQ(177),
		},
#endif
#ifdef CONFIG_SENSORS_ATTINY48MU_CAP_PROX
		{
			I2C_BOARD_INFO("attiny48mu_cap_prox", 0x12),
			.platform_data = &mp_attiny48mu_cap_prox_pdata,
		},
#endif

};

static struct i2c_board_info __initdata mapphone_i2c_3_boardinfo[] = {
};

static struct i2c_board_info __initdata mapphone_i2c_4_boardinfo[] = {
	{
		I2C_BOARD_INFO("kxtf9", 0x0F),
		.platform_data = &mp_kxtf9_pdata,
	},
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.platform_data = &mp_akm8975_pdata,
		.irq = OMAP_GPIO_IRQ(175),
	},
	{
		I2C_BOARD_INFO("l3g4200d", 0x68),
		.platform_data = &mp_l3g4200d_pdata,
	},
	{
		I2C_BOARD_INFO("bmp180", 0x77),
		.platform_data = &mp_bmp180_pdata,
		.irq = OMAP_GPIO_IRQ(178),
	},
	{
		I2C_BOARD_INFO("msp430", 0x48),
		.platform_data = &mp_msp430_data,
	},
};

static void __init mapphone_pmic_mux_init(void)
{

	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata mapphone_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata mapphone_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata mapphone_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata mapphone_i2c_4_bus_pdata;

static int __init mapphone_tmp105_init(struct i2c_board_info *info)
{
	int gpio;

	if (!info)
		return -EINVAL;

	/* preinit the pcb temperature sensor */
	gpio = get_gpio_by_name("tmp105_int");
	if (gpio >= 0) {
		gpio_request(gpio, "tmp105_intr");
		gpio_direction_input(gpio);
		info->irq = gpio_to_irq(gpio);
	}

	return 0;
}

static struct i2c_board_info *get_board_info
(
	char *dev_name,
	int bus_num,
	struct i2c_board_info *board_info_table,
	int size
)
{
	int i;
	char *entry_name;

	if (dev_name != NULL && board_info_table) {
		/* search for the name in the table */
		for (i = 0; i < size; i++) {
				entry_name = board_info_table[i].type;
				if (strncmp(entry_name, dev_name,\
					strlen(entry_name)) == 0)
					return &board_info_table[i];
			}
	}

	return NULL;
}

static int initialize_i2c_bus_info
(
	int bus_num,
	struct i2c_board_info *board_info,
	int info_size,
	struct i2c_board_info *master_board_info,
	int master_info_size
)
{
	int dev_cnt = 0;
	struct device_node *bus_node;
	const void *feat_prop;
	char *device_names;
	char dev_name[I2C_MAX_DEV_NAME_LEN];
	int device_name_len, i, j;
	struct i2c_board_info *master_entry;
	char prop_name[I2C_BUS_PROP_NAME_LEN];

	j = 0;

	bus_node = of_find_node_by_path(DT_PATH_I2C);
	if (bus_node == NULL || board_info == NULL)
		return dev_cnt;

	snprintf(prop_name, I2C_BUS_PROP_NAME_LEN,
		"bus%1ddevices", bus_num);

	feat_prop = of_get_property(bus_node,
			prop_name, NULL);
	if (NULL != feat_prop) {
		device_names = (char *)feat_prop;
		printk(KERN_INFO
			"I2C-%d devices: %s\n", bus_num, device_names);
		device_name_len = strlen(device_names);

		memset(dev_name, 0x0, I2C_MAX_DEV_NAME_LEN);

		for (i = 0; i < device_name_len; i++) {

			if (device_names[i] != '\0' &&
				device_names[i] != ',')
				dev_name[j++] = device_names[i];
			/* parse for ',' in string */
			if (device_names[i] == ',' ||
				(i == device_name_len-1)) {

				if (dev_cnt < info_size) {
					master_entry =
						get_board_info(dev_name,
							bus_num,
							master_board_info,
							master_info_size);
					if (master_entry != NULL) {
						memcpy(
							&board_info[dev_cnt++],
							master_entry,
							sizeof(
							struct i2c_board_info));
						printk(KERN_INFO
							"%s -> I2C bus-%d\n",
							master_entry->type,
							bus_num);

					}
					j = 0;
					memset(
							dev_name,
							0x0,
							I2C_MAX_DEV_NAME_LEN);
				}
			}
		}
	}
	return dev_cnt;
}

static int __init omap4_i2c_init(void)
{
	int i2c_bus_devices = 0;

	omap_i2c_hwspinlock_init(1, 0, &mapphone_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &mapphone_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &mapphone_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &mapphone_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &mapphone_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &mapphone_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &mapphone_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &mapphone_i2c_4_bus_pdata);

	/* Populate I2C bus 1 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			1, mapphone_i2c_bus1_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_1_boardinfo,
			ARRAY_SIZE(mapphone_i2c_1_boardinfo));
	omap_register_i2c_bus(1, 400,
			mapphone_i2c_bus1_board_info, i2c_bus_devices);

	/* Populate I2C bus 2 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			2, mapphone_i2c_bus2_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_2_boardinfo,
			ARRAY_SIZE(mapphone_i2c_2_boardinfo));
	omap_register_i2c_bus(2, 400,
			mapphone_i2c_bus2_board_info, i2c_bus_devices);

	/* Populate I2C bus 3 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			3, mapphone_i2c_bus3_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_3_boardinfo,
			ARRAY_SIZE(mapphone_i2c_3_boardinfo));
	omap_register_i2c_bus(3, 400,
			mapphone_i2c_bus3_board_info, i2c_bus_devices);

	/* Populate I2C bus 4 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			4, mapphone_i2c_bus4_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_4_boardinfo,
			ARRAY_SIZE(mapphone_i2c_4_boardinfo));
	omap_register_i2c_bus(4, 400,
			mapphone_i2c_bus4_board_info, i2c_bus_devices);

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	return 0;
}

static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

#else
#define board_mux	NULL
#define board_wkup_mux NULL
#endif

/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices = {
	.cs0_device = &lpddr2_elpida_2G_S4_dev,
};

static unsigned int __init get_ap_ddr_size(void)
{
	/*If no ddr size defined, default value is 512 MB*/
	unsigned int sz = 512;

	struct device_node *sz_node;
	const void *sz_prop;

	sz_node = of_find_node_by_path(DT_PATH_CHOSEN);

	if (sz_node != NULL) {
		sz_prop = of_get_property(sz_node, \
			DT_PROP_CHOSEN_AP_DDR_SIZE, NULL);
		if (sz_prop)
			sz = *(unsigned int *)sz_prop;
		of_node_put(sz_node);
	} else
		printk(KERN_ERR"%s can not find Chosen@0 node\n", __func__);

	printk(KERN_INFO"%s: AP DDR size = %d MB\n", __func__, sz);
	return sz;
}


static void __init mapphone_voltage_init(void)
{
	struct device_node *node;
	const uint32_t *val;

	/* Read CTRL_FUSE_OPP_VDD_MPU_3 to check whether it is 1.2G supported */
	if (omap_readl(0x4A00224C) & 0xFFFFFF)
		omap4_features |= OMAP4_HAS_MPU_1_2GHZ;

	node = of_find_node_by_path("/System@0/PwrMgmt@0");
	if (node) {
		val = of_get_property(node, "fan5355_reg", NULL);
		if (val != NULL) {
			if (*val) {
				pr_info("Using fan5355 switchers\n");
				omap_fan5355_init();
			}
		} else {
			pr_info("Using cpcap switchers\n");
			/* cpcap is the default power supply for core and iva */
			omap_cpcap_init();
		}
		/* enable_suspend_off is true only when enable_off_mode is
		 * defined and its value is set to true in devtree
		 */
		enable_suspend_off = false;
		val = of_get_property(node, "enable_off_mode", NULL);
		if (val != NULL) {
			if (*val)
				enable_suspend_off = true;
		}

		/* Some HW needs to be limited to 1.0Ghz. */
		val = of_get_property(node, "limit_1_0_ghz", NULL);
		if (val != NULL) {
			if (*val)
				omap4_features &= ~OMAP4_HAS_MPU_1_2GHZ;
		}


		of_node_put(node);
	}

	omap_max8952_init();
}


/* RST_TIME1>4ms will trigger CPCAP to trigger a system cold reset */
static void mapphone_pm_set_rstime1(void)
{
	s16 prcm_mod_base = 0;
	s16 prcm_mod_offset = 0;

	if (cpu_is_omap44xx()) {
		prcm_mod_base = OMAP4430_PRM_DEVICE_INST;
		prcm_mod_offset = OMAP4_PRM_RSTTIME_OFFSET;
	} else
		WARN_ON(1);

	/* Configure RST_TIME1 to 6ms */
	omap4_prm_rmw_inst_reg_bits(OMAP4430_RSTTIME1_MASK,
			0xc8<<OMAP4430_RSTTIME1_SHIFT,
			prcm_mod_base,
			prcm_mod_offset);
}

static void mapphone_pm_power_off(void);
static int set_cold_reset(struct notifier_block *this,
				unsigned long event, void *ptr);

static struct timer_list mapphone_shutdown_timer;

struct shutdown_timer_arg {
	unsigned long event;
	char *command;
};

static void mapphone_shutdown_timeout_handler(unsigned long data)
{
	struct shutdown_timer_arg *arg = (struct shutdown_timer_arg *)data;

	printk(KERN_ERR"mapphone shutdown/reboot timeout!\n");

	if (!arg || SYS_RESTART != arg->event) {
		printk(KERN_ERR"%s: Force to power down\n", __func__);
		mapphone_pm_power_off();
	} else {
		printk(KERN_ERR"%s: Force to cold reset\n", __func__);
		set_cold_reset(NULL, arg->event, (void *)arg->command);
		machine_restart(arg->command);
	}
}

static int mapphone_shutdown_timer_kickoff(struct notifier_block *this,
						unsigned long event, void *ptr)
{
	struct shutdown_timer_arg *arg =
		(struct shutdown_timer_arg *)mapphone_shutdown_timer.data;

	if (unlikely(!arg))
		return 0;

	arg->event = event;
	arg->command = (char *)ptr;

	/*give 15s for shutdown or reboot */
	mod_timer(&mapphone_shutdown_timer, jiffies + 15*HZ);
	printk(KERN_DEBUG "mapphone shutdown/reboot timer has kicked off\n");
	return 0;
}

static struct notifier_block mapphone_reboot_timer_notifier = {
	.notifier_call = mapphone_shutdown_timer_kickoff,
	.priority = INT_MAX,
};

static void mapphone_shutdown_timer_init(void)
{
	struct shutdown_timer_arg *shutdown_arg;

	shutdown_arg = kmalloc(sizeof(struct shutdown_timer_arg), GFP_KERNEL);
	if (unlikely(!shutdown_arg)) {
		printk(KERN_INFO "Failed to allocate memory for "
			"shutdown timer argument\n");
		return;
	}

	shutdown_arg->event = SYS_POWER_OFF;
	shutdown_arg->command = NULL;

	init_timer(&mapphone_shutdown_timer);
	mapphone_shutdown_timer.data = (unsigned long)shutdown_arg;
	mapphone_shutdown_timer.function = mapphone_shutdown_timeout_handler;

	register_reboot_notifier(&mapphone_reboot_timer_notifier);
}

static int set_cold_reset(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	/* set cold reset rst time1 */
	u32 result = omap4_prm_read_inst_reg(OMAP4430_PRM_DEVICE_INST,
						OMAP4_PRM_RSTTIME_OFFSET);

	printk(KERN_ERR"Read RST TIME=%x\n", result);

	mapphone_pm_set_rstime1();
	result = omap4_prm_read_inst_reg(OMAP4430_PRM_DEVICE_INST,
						OMAP4_PRM_RSTTIME_OFFSET);
	printk(KERN_ERR"Reset time is configured!=%x\n", result);

	return NOTIFY_DONE;
}

static struct notifier_block mapphone_panic_notifier = {
	.notifier_call = set_cold_reset,
	.priority = INT_MAX,
};

static struct notifier_block mapphone_cold_reboot_notifier = {
	.notifier_call = set_cold_reset,
	.priority = INT_MAX,
};

static void mapphone_rst_init(void)
{
	mapphone_shutdown_timer_init();

#ifdef CONFIG_MOT_ENG_PHONE_RESET
	atomic_notifier_chain_register(&panic_notifier_list,
					&mapphone_panic_notifier);
	register_reboot_notifier(&mapphone_cold_reboot_notifier);
#else
	set_cold_reset(NULL, 0, NULL);
#endif
}

static int external_amp_gpio = -1;
static struct platform_device mapphone_external_amp_device = {
	.name	= "mapphone_external_amp",
	.id	= -1,
	.num_resources = 0,
	.dev.platform_data = &external_amp_gpio,
};

static inline void mapphone_external_amp_init(void)
{
	external_amp_gpio = get_gpio_by_name("external_amp_en");

	if (external_amp_gpio < 0) {
		pr_debug("Cannot find external amp gpio by name\n");
	} else {
		int ret = 0;
		pr_debug("external_amp_gpio = %d\n", external_amp_gpio);
		ret = gpio_request(external_amp_gpio, "external amp");

		if (ret == 0)
			ret = gpio_direction_output(external_amp_gpio, 0);
		else
			pr_err("Failed to request external amp gpio\n");
		platform_device_register(&mapphone_external_amp_device);
	}
}

#ifdef CONFIG_SND_LM48901_AMP
static inline void mapphone_lm48901_init(void)
{
	int ret = 0;

	/* lm48901_amp_en */
	lm48901_pdata.amp_en_gpio = get_gpio_by_name("lm48901_amp_en");

	if (lm48901_pdata.amp_en_gpio < 0) {
		pr_debug("Cannot find a lm48901_amp_en gpio by name\n");
		return;
	}

	pr_debug("lm48901_amp_en gpio = %d\n", lm48901_pdata.amp_en_gpio);
	ret = gpio_request(lm48901_pdata.amp_en_gpio, "lm48901_amp_en");
	if (ret != 0) {
		pr_err("Failed to request the lm48901_amp_en gpio\n");
		return;
	}
	gpio_direction_output(lm48901_pdata.amp_en_gpio, 0);

	/* lm48901_i2c_en */
	lm48901_pdata.i2c_en_gpio = get_gpio_by_name("lm48901_i2c_en");

	if (lm48901_pdata.i2c_en_gpio < 0) {
		pr_debug("Cannot find a lm48901_i2c_en gpio by name\n");
		return;
	}

	pr_debug("lm48901_i2c_en gpio = %d\n", lm48901_pdata.i2c_en_gpio);
	ret = gpio_request(lm48901_pdata.i2c_en_gpio, "lm48901_i2c_en");
	if (ret != 0) {
		pr_err("Failed to request the lm48901_i2c_en  gpio\n");
		return;
	}
	gpio_direction_output(lm48901_pdata.i2c_en_gpio, 0);
}
#endif

static int power_off_gpio = -1;

static void mapphone_pm_power_off(void)
{
	printk(KERN_INFO "mapphone_pm_power_off start...\n");
	local_irq_disable();

	gpio_direction_output(power_off_gpio, 0);

	do {} while (1);

	local_irq_enable();
}

static void mapphone_pm_reset(void)
{
	arch_reset('h', NULL);
}

static int cpcap_charger_connected_probe(struct platform_device *pdev)
{
	pm_power_off = mapphone_pm_reset;
	return 0;
}

static int cpcap_charger_connected_remove(struct platform_device *pdev)
{
	pm_power_off = mapphone_pm_power_off;
	return 0;
}

static struct platform_driver cpcap_charger_connected_driver = {
	.probe          = cpcap_charger_connected_probe,
	.remove         = cpcap_charger_connected_remove,
	.driver         = {
		.name   = "cpcap_charger_connected",
		.owner  = THIS_MODULE,
	},
};

static void __init mapphone_power_off_init(void)
{
	power_off_gpio = get_gpio_by_name("power_off");
	if (power_off_gpio < 0) {
		printk(KERN_INFO "Can't get power_off gpio from devtree\n");
		power_off_gpio = MAPPHONE_POWER_OFF_GPIO;
	}

	gpio_request(power_off_gpio, "mapphone power off");
	gpio_direction_output(power_off_gpio, 1);

	pm_power_off = mapphone_pm_power_off;

	platform_driver_register(&cpcap_charger_connected_driver);
}

static struct omap2_hdq_platform_config mapphone_hdq_data = {
	.mode = OMAP_SDQ_MODE,
	.id = W1_EEPROM_DS2502,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode                   = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode                   = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode                   = MUSB_PERIPHERAL,
#endif
	.power                  = 100,
};

static void __init mapphone_musb_init(void)
{
	struct device_node *node;
	const void *prop;
	int size;
	int use_utmi = 0;
	u16 power = 0;
	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);

	if (node) {
		prop = of_get_property(node, "feature_musb_utmi", &size);
		if (prop && size) {
			use_utmi = *(u8 *)prop;
			pr_debug("Using %s as the MUSB Mode \n",
				use_utmi ? "UTMI" : "ULPI");

		} else
			pr_debug("USB Defaulting to ULPI \n");
		of_node_put(node);
	}

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node) {
		prop = of_get_property(node,
				DT_PROP_CHOSEN_MUSBHS_EXTPOWER, &size);
		if (prop && size) {
			power = *(u16 *)prop;
			pr_debug("Current supplied by ext power: %d\n", power);
		}
		of_node_put(node);
	}

	if (use_utmi)
		musb_board_data.interface_type = MUSB_INTERFACE_UTMI;

	if (power > 100 && power <= 500 )
		musb_board_data.power = power;

	usb_musb_init(&musb_board_data);
}

#ifdef CONFIG_OMAP_HSI
static void __init mapphone_hsi_init(void)
{
	struct device_node *node;
	const void *prop;
	int size;
	int feature_hsi = 0;

	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (node) {
		prop = of_get_property(node, "feature_hsi", &size);
		if (prop && size) {
			feature_hsi = *(u8 *)prop;
			pr_debug("%s HSI in the Platform.\n",
				 feature_hsi ? "Enabling" : "Disabling");
		}
		of_node_put(node);
	}
	if (feature_hsi)
		omap_hsi_allow_registration();
}
#endif

#ifdef CONFIG_SPI_QMI
static struct platform_device qmi_over_spi = {
	.name   = "qmi-over-spi",
	.id     = 0,
};

static void __init mapphone_qmi_over_spi_init(void)
{
	struct device_node *node;
	const void *prop;
	int size;
	int feature_qmi_over_spi = 0;

	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (node) {
		prop = of_get_property(node, "feature_qmi_over_spi", &size);
		if (prop && size) {
			feature_qmi_over_spi = *(u8 *)prop;
			pr_err("%s QMI over SPI\n",
				feature_qmi_over_spi ?
				"Enabling" : "Disabling");
		}
		of_node_put(node);
	}

	printk(KERN_INFO "feature_qmi_over_spi=%d\n", feature_qmi_over_spi);
	if (feature_qmi_over_spi)
		platform_device_register(&qmi_over_spi);
}
#endif

#define ZEROV_CTRL_RGLTR_NONE 0
#define ZEROV_CTRL_RGLTR_VCSI 1

static struct zerov_ctrl_platform_data {
	u32    rgltr_id;
	int    rgltr_enable_count;
	struct regulator *rgltr;
} zerov_ctrl_pdata;

static int zerov_ctrl_suspend(struct platform_device *pdev,
						pm_message_t state)
{
	struct zerov_ctrl_platform_data *pdata = pdev->dev.platform_data;
	int status = 0;


	/* If OFF mode is disabled, or volt_off mode is disabled, then
	 * don't turn off the zerov control regulator.
	 */
	if (!off_mode_enabled)
		return 0;

	if (pdata->rgltr_enable_count == 1) {
		status = regulator_disable(pdata->rgltr);
		if (status) {
			pr_err("%s: regulator disable failed: "
				"status=%d\n", __func__, status);
		} else
			pdata->rgltr_enable_count--;
	} else
		pr_err("%s: rgltr_enable_count(=%d) mismatch\n",
			__func__, pdata->rgltr_enable_count);

	return 0;
}

static int zerov_ctrl_resume(struct platform_device *pdev)
{
	struct zerov_ctrl_platform_data *pdata = pdev->dev.platform_data;
	int status = 0;


	/* If OFF mode is disabled, or volt_off mode is disabled, then
	 * zerov control regulator is not disabled in suspend. So, should
	 * not enable it here.
	 */
	if (!off_mode_enabled)
		return 0;


	if (pdata->rgltr_enable_count == 0) {
		status = regulator_enable(pdata->rgltr);
		BUG_ON(status != 0);
		pdata->rgltr_enable_count++;
	} else
		pr_err("%s: rgltr_enable_count(=%d) mismatch\n",
			__func__, pdata->rgltr_enable_count);
	return 0;
}

static int __init zerov_ctrl_probe(struct platform_device *pdev)
{
	struct regulator *zerov_regulator = NULL;
	int status = 0;
	struct zerov_ctrl_platform_data *pdata = pdev->dev.platform_data;


	if (!pdata || pdata->rgltr_id != ZEROV_CTRL_RGLTR_VCSI) {
		pr_warning("%s: Invalid regulator\n", __func__);
		return -EINVAL;
	}

	zerov_regulator = regulator_get(NULL, "vdds_dsi");

	BUG_ON(IS_ERR(zerov_regulator));

	status = regulator_enable(zerov_regulator);
	BUG_ON(status != 0);

	pdata->rgltr_enable_count++;
	pdata->rgltr = zerov_regulator;

	return 0;
}

static struct platform_driver zerov_ctrl_driver = {
	.suspend        = zerov_ctrl_suspend,
	.resume         = zerov_ctrl_resume,
	.driver		= {
		.name	= "zerov_ctrl",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device zerov_ctrl_device = {
	.name = "zerov_ctrl",
	.id = -1,
	.dev = {
		.platform_data = &zerov_ctrl_pdata,
	},
};

static int __init mapphone_zerov_ctrl_init(void)
{
	struct device_node *node;
	const void *prop;
	int status = 0;


	node = of_find_node_by_path("/System@0/PwrMgmt@0");
	if (node) {
		prop = of_get_property(node, "zerov_ctrl_rgltr", NULL);

		if (!prop || (*(int *)prop != ZEROV_CTRL_RGLTR_VCSI)) {
			pr_warning("%s: Invalid or no ctrl regulator\n",
					__func__);
			return 0;
		}

		if (prop && (*(int *)prop == ZEROV_CTRL_RGLTR_VCSI)) {

			zerov_ctrl_pdata.rgltr_id = *(int *)prop;

			status = platform_device_register(&zerov_ctrl_device);

			if (status) {
				pr_info("zerov_ctrl_device register failed\n");
				BUG();
			}

			status = platform_driver_probe(&zerov_ctrl_driver,
							zerov_ctrl_probe);

			if (status) {
				pr_info("zerov_ctrl_driver register failed\n");
				BUG();
			}
		}
	}
	return 0;
}

/* mapphone_zerov_ctrl_init and probe MUST happen after the regulator init
 * (which happens in subsys_initcall) and before CPUIdle is registered.
 * CPUIdle is registered in omap4_pm_init() which is defined as late_initcall.
 * So, mapphone_zerov_ctrl_init() can be called either in module_init or
 * device_initcall_sync.
 */
device_initcall_sync(mapphone_zerov_ctrl_init);

static void __init mapphone_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, NULL, package);

	mapphone_rst_init();

	if (get_ap_ddr_size() > 512)
		emif_devices.cs1_device = &lpddr2_elpida_2G_S4_dev;

	omap_emif_setup_device_details(&emif_devices, &emif_devices);

	omap_board_config = mapphone_config;
	omap_board_config_size = ARRAY_SIZE(mapphone_config);

	omap_init_board_version(0);
	mapphone_bp_model_init();
	omap4_audio_conf();
	mapphone_pmic_mux_init();
	mapphone_gpio_mapping_init();
	/* touch_init() must run before i2c_init() */
	mapphone_touch_panel_init(&mapphone_i2c_2_boardinfo[0]);
	mapphone_touch_btn_init(&mapphone_i2c_2_boardinfo[1]);

	/* hardcode tmp105 to bus-1, master-board-info-0 */
	mapphone_tmp105_init(&mapphone_i2c_1_boardinfo[0]);

	/*the fucntion must run after the mapphone_touch_panel_init*/
	omap4_create_board_props();
	omap4_i2c_init();
	mapphone_voltage_init();
	mapphone_omap44xx_padconf_init();

#ifdef CONFIG_EMU_UART_DEBUG
	/* emu-uart function will override devtree iomux setting */
	activate_emu_uart();
//#else
//	deactivate_emu_uart();
#endif

	omap4_register_ion();
	platform_add_devices(mapphone_devices, ARRAY_SIZE(mapphone_devices));
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
	mapphone_mdm_ctrl_init();
	mapphone_spi_init(boot_mode);
	mapphone_cpcap_client_init();
	mapphone_serial_init();
	mapphone_panel_init();
	mapphone_vibrator_init();
	mapphone_sensors_init();
	omap4_mapphone_wifi_init();
	omap_hdq1w_init(&mapphone_hdq_data);

	mapphone_usbhost_init();
	mapphone_musb_init();
#ifdef CONFIG_OMAP_HSI
	mapphone_hsi_init();
#endif
	mapphone_power_off_init();
	mapphone_gadget_init(boot_mode);

	mapphone_external_amp_init();
#ifdef CONFIG_SND_LM48901_AMP
	mapphone_lm48901_init();
#endif
	/* XXX usb_musb_init(&musb_board_data); */

#ifdef CONFIG_SPI_QMI
	mapphone_qmi_over_spi_init();
#endif

	omap_dmm_init();

	mapphone_hsmmc_init();

	if (cpu_is_omap446x()) {
		/* Vsel0 = gpio, vsel1 = gnd */
		status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
					OMAP_PIN_OFF_OUTPUT_HIGH, -1);
		if (status)
			pr_err("TPS62361 initialization failed: %d\n", status);
	}

	omap_enable_smartreflex_on_init();
	if (enable_suspend_off)
		omap_pm_enable_off_mode();

}

static void __init mapphone_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}
static void __init mapphone_reserve(void)
{
	omap_ram_console_init(OMAP4_RAMCONSOLE_START,
			OMAP4_RAMCONSOLE_SIZE);

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	pr_debug("%s: SMC size=%dMB, addr=0x%x\n",
		__func__, (PHYS_ADDR_SMC_SIZE >> 20), PHYS_ADDR_SMC_MEM);

	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	pr_debug("%s: DUCATI Memory size=%dMB, addr=0x%x\n",
		__func__, (PHYS_ADDR_DUCATI_SIZE >> 20), PHYS_ADDR_DUCATI_MEM);

	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE);

#ifdef CONFIG_ION_OMAP
	omap_ion_init();
#endif

	omap_reserve();
}

MACHINE_START(MAPPHONE, "mapphone_")
	/* Maintainer: Motorola Mobility, Inc. */
	.boot_params	= 0x80000100,
	.reserve	= mapphone_reserve,
	.map_io		= mapphone_map_io,
	.init_early	= mapphone_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= mapphone_init,
	.timer		= &omap_timer,
MACHINE_END
