/*
 * arch/arm/mach-omap2/board-mapphone-keypad.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2010 Motorola, Inc.
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

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>

#include <plat/gpio.h>
#include <linux/gpio_mapping.h>
#include "board-mapphone.h"
#include <plat/omap4-keypad.h>

#include "dt_path.h"
#include <linux/of.h>

#ifdef CONFIG_KEYBOARD_ADP5588
#include <linux/i2c/adp5588.h>
#include <linux/delay.h>
#endif

#ifdef CONFIG_MOT_KEYBOARD_ADP5588
#include <linux/adp5588_keypad.h>
#endif

#define OMAP4_KBDOCP_BASE               0x4A31C000
#define GPIO_NOT_INITIALIZED       -1

static int omap_keymap[] = {
	KEY(0, 0, KEY_9),
	KEY(1, 0, KEY_R),
	KEY(2, 0, KEY_SEND),
	KEY(3, 0, KEY_VOLUMEDOWN),
	KEY(4, 0, KEY_F4),
	KEY(5, 0, KEY_VOLUMEUP),
	KEY(6, 0, KEY_SEARCH),
	KEY(7, 0, KEY_D),

	KEY(0, 1, KEY_7),
	KEY(1, 1, KEY_M),
	KEY(2, 1, KEY_L),
	KEY(3, 1, KEY_K),
	KEY(4, 1, KEY_N),
	KEY(5, 1, KEY_C),
	KEY(6, 1, KEY_Z),
	KEY(7, 1, KEY_RIGHTSHIFT),

	KEY(0, 2, KEY_1),
	KEY(1, 2, KEY_Y),
	KEY(2, 2, KEY_I),
	KEY(3, 2, KEY_SLASH),
	KEY(4, 2, KEY_LEFTALT),
	KEY(5, 2, KEY_DOT),
	KEY(6, 2, KEY_G),
	KEY(7, 2, KEY_E),

	/*KEY(0, 3, KEY_),*/
	KEY(1, 3, KEY_6),
	KEY(2, 3, KEY_3),
	KEY(3, 3, KEY_DOWN),
	KEY(4, 3, KEY_UP),
	KEY(5, 3, KEY_LEFT),
	KEY(6, 3, KEY_RIGHT),
	KEY(7, 3, KEY_REPLY),

	KEY(0, 4, KEY_5),
	KEY(1, 4, KEY_J),
	KEY(2, 4, KEY_B),
	KEY(3, 4, KEY_TAB),
	KEY(4, 4, KEY_T),
	KEY(5, 4, KEY_GRAVE),
	KEY(6, 4, KEY_MENU),
	KEY(7, 4, KEY_X),

	KEY(0, 5, KEY_8),
	KEY(1, 5, KEY_SPACE),
	KEY(2, 5, KEY_LEFTALT),
	KEY(3, 5, KEY_COMMA),
	KEY(4, 5, KEY_SLASH),
	KEY(5, 5, KEY_EMAIL),
	KEY(6, 5, KEY_BACKSPACE),
	KEY(7, 5, KEY_A),

	KEY(0, 6, KEY_2),
	KEY(1, 6, KEY_0),
	KEY(2, 6, KEY_F),
	KEY(3, 6, KEY_LEFTSHIFT),
	KEY(4, 6, KEY_ENTER),
	KEY(5, 6, KEY_O),
	KEY(6, 6, KEY_H),
	KEY(7, 6, KEY_Q),

	KEY(0, 7, KEY_4),
	KEY(1, 7, KEY_V),
	KEY(2, 7, KEY_S),
	KEY(3, 7, KEY_P),
	KEY(4, 7, KEY_EMAIL),
	KEY(5, 7, KEY_MUTE),
	KEY(6, 7, KEY_U),
	KEY(7, 7, KEY_W),
};

static struct matrix_keymap_data omap_keymap_data = {
	.keymap			= omap_keymap,
	.keymap_size		= ARRAY_SIZE(omap_keymap),
};

static struct omap4_keypad_platform_data omap_kp_data = {
	.keymap_data	= &omap_keymap_data,
	.rows		= 8,
	.cols		= 8,
	/* XXX .delay		= 4, */
	.rep		= 0,
};

static struct platform_device omap_kp_device = {
	.name		= "omap4-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &omap_kp_data,
	},
};

/* Do not break the EV_SW When enable omap4 keyboard driver. */
static int virtual_matrix_func(struct gpio_event_input_devs *input_devs,
	struct gpio_event_info *info, void **data, int func)
{
	unsigned short keyentry = KEY_Q;
	unsigned short keycode = keyentry & MATRIX_KEY_MASK;
	unsigned short dev = keyentry >> MATRIX_CODE_BITS;

	if (func != GPIO_EVENT_FUNC_INIT)
		return 0;

	input_set_capability(input_devs->dev[dev], EV_KEY, keycode);
	return 0;
}

static struct gpio_event_direct_entry mapphone_switch_map[] = {
	{GPIO_NOT_INITIALIZED,		SW_LID}
};

static struct gpio_event_direct_entry mapphone_gpiokey_map[] = {
	{GPIO_NOT_INITIALIZED,  KEY_VOLUMEDOWN}
};

static struct gpio_event_matrix_info mapphone_keypad_matrix_info = {
	.info.func = virtual_matrix_func,
	.settle_time.tv64 = 40 * NSEC_PER_USEC,
	.poll_time.tv64 = 20 * NSEC_PER_MSEC,
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_REMOVE_PHANTOM_KEYS |
		 GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_input_info mapphone_switch_input_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_SW,
	.keymap = mapphone_switch_map,
	.keymap_size = ARRAY_SIZE(mapphone_switch_map)
};

static struct gpio_event_input_info mapphone_gpiokey_input_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = mapphone_gpiokey_map,
	.keymap_size = ARRAY_SIZE(mapphone_gpiokey_map)
};

static struct gpio_event_info *mapphone_switch_info[3];

static struct gpio_event_platform_data mapphone_switch_data = {
	.name = "mapphone-switch",
	.info = NULL,
	.info_count = 0,
};

static struct gpio_event_platform_data mapphone_gpiokey_data = {
	.name = "mapphone-gpiokey",
	.info = NULL,
	.info_count = 0,
};

static struct platform_device mapphone_switch_device = {
	.name = GPIO_EVENT_DEV_NAME,
	//.name = "mapphone-switch",
	.id = 0,
	.dev		= {
		.platform_data	= &mapphone_switch_data,
	},
};

static struct platform_device mapphone_gpiokey_device = {
	.name = GPIO_EVENT_DEV_NAME,
	//.name = "mapphone-gpiokey",
	.id = 0,
	.dev	    = {
		.platform_data  = &mapphone_gpiokey_data,
	},
};


static int mapphone_reset_keys_up[] = {
	BTN_MOUSE,		/* XXX */
	0
};

static int mapphone_reset_keys_down[] = {
	KEY_VOLUMEDOWN,
	KEY_END,
	0
};

static struct keyreset_platform_data mapphone_reset_keys_pdata = {
	.crash_key = KEY_VOLUMEUP,
	.keys_up = mapphone_reset_keys_up,
	.keys_down = mapphone_reset_keys_down,
};

struct platform_device mapphone_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &mapphone_reset_keys_pdata,
};

static int __init mapphone_dt_kp_init(void)
{
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_KEYPAD);
	if (node) {
		prop = of_get_property(node, DT_PROP_KEYPAD_ROWS, NULL);
		if (prop)
			omap_kp_data.rows = *(u8 *)prop;

		prop = of_get_property(node, DT_PROP_KEYPAD_COLS, NULL);
		if (prop)
			omap_kp_data.cols = *(u8 *)prop;

		prop = of_get_property(node, DT_PROP_KEYPAD_MAPS_OMAP4, NULL);
		if (prop)
			omap_keymap_data.keymap = (unsigned int *)prop;

		omap_keymap_data.keymap_size =
				omap_kp_data.rows * omap_kp_data.cols;

		of_node_put(node);
	}

	return node ? 0 : -ENODEV;
}

static int __init mapphone_init_keypad(void)
{
	int status;
	int is_slider_found = 0;
	int is_gpiokey_found = 0;

	int slider_gpio, volume_gpio, i;
	int array_size = 0;

	/*
	 * For those h/w don't have slider, there will be no slider_data
	 * in devtree. All other h/w should provide slider_data.
	 * There is no default gpio for it
	 */
	slider_gpio = get_gpio_by_name("slider_data");

	if (slider_gpio > 0)
		for (i = 0; i < ARRAY_SIZE(mapphone_switch_map); i++)
			if (SW_LID == mapphone_switch_map[i].code) {
				mapphone_switch_map[i].gpio = slider_gpio;
				is_slider_found = 1;
				break;
			}
	volume_gpio = get_gpio_by_name("volume_gpio");
	if (volume_gpio >= 0)
		for (i = 0; i < ARRAY_SIZE(mapphone_gpiokey_map); i++)
			if (KEY_VOLUMEDOWN == mapphone_gpiokey_map[i].code) {
				mapphone_gpiokey_map[i].gpio = volume_gpio;
				is_gpiokey_found = 1;
				break;
			}
	if (is_slider_found) {
		mapphone_switch_info[0] = &mapphone_keypad_matrix_info.info;
		mapphone_switch_info[1] = &mapphone_switch_input_info.info;
		array_size = 2;

		if (is_gpiokey_found)
			mapphone_switch_info[array_size++] =
					&mapphone_gpiokey_input_info.info;

		mapphone_switch_data.info = mapphone_switch_info;
		mapphone_switch_data.info_count = array_size;
	}

	if (mapphone_dt_kp_init() != 0)
		printk(KERN_INFO "Keypad: using non-dt configuration\n");

	//printk("registering reset_key_device\n");
	platform_device_register(&mapphone_reset_keys_device);

	if (is_slider_found) {
		//printk("registering switch_device\n");
		platform_device_register(&mapphone_switch_device);
	is_gpiokey_found = 0; //Don't register gpiokey device. it only breaks things.
		if (is_gpiokey_found)
			{
			printk("registering gpiokey_device\n");
			platform_device_register(&mapphone_gpiokey_device);
			}
	}
	status = omap4_keyboard_init(&omap_kp_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);

	return status;
}

device_initcall(mapphone_init_keypad);
