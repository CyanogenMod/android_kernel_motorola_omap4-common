/*
 * arch/arm/mach-omap2/board-mapphone.h
 *
 * Copyright 2011 Motorola Mobility, Inc.
 *
 * Based on arch/arm/mach-omap2/board-blaze.h
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MACH_OMAP_BOARD_MAPPHONE_H
#define _MACH_OMAP_BOARD_MAPPHONE_H

#include <linux/i2c.h>

extern char *bp_model;

void __init set_machine_name(const char *name);
void __init set_cpu_tier(const char *tier);

void omap4_create_board_props(void);
void omap_ion_init(void);
void omap4_register_ion(void);
extern void __init mapphone_serial_init(void);
extern void __init mapphone_sensors_init(void);
void __init mapphone_cpcap_client_init(void);
void __init mapphone_spi_init(char *boot_mode);
int __init mapphone_hsmmc_init(void);
void __init mapphone_gpio_mapping_init(void);
void __init mapphone_touch_panel_init(struct i2c_board_info *i2c_info);
void __init mapphone_touch_btn_init(struct i2c_board_info *i2c_info);
void __init mapphone_panel_init(void);
extern void __init mapphone_vibrator_init(void);
extern void __init mapphone_usb_init(void);
extern void __init mapphone_gadget_init(char *boot_mode);
extern void __init mapphone_usbhost_init(void);
extern int __init mapphone_mdm_ctrl_init(void);
extern struct attribute_group *mapphone_touch_vkey_prop_attr_group;

struct omap_ion_platform_data;
void mapphone_android_display_setup(struct omap_ion_platform_data *ion);

#define BOOT_MODE_MAX_LEN 30

#define I2C_BUS_MAX_DEVICES 5
#define I2C_MAX_DEV_NAME_LEN 16
#define I2C_BUS_PROP_NAME_LEN 12
extern void mcbsp3_i2s1_pin_mux_switch(unsigned short incall);

#ifdef CONFIG_OMAP_HSI
extern void __init omap_hsi_dev_init(void);
#endif

#endif
