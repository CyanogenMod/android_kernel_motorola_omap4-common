/*
 * Copyright (C) ST-Ericsson AB 2011
 * Author:	Erwan Bracq / Erwan.Bracq@stericsson.com
 * License terms: GNU General Public License (GPL), version 2.
 */

struct memwrite_data	{
	u32 address;
	u32 bitmask;
	u32 value;
};

/* CUSTOM section. All fields bellow need to be updated
 * to match the targeted configuration
 */

#define DRV_VERSION		"0.1_MOT_EDISON"

static struct memwrite_data platform_reset_array[] = {
	/* {register, mask, value},
	 * {register, mask, value}...
	 */
};

static struct memwrite_data gpioserv_en_safe_mode_array[] = {
	{0x4A100078, 0x0000010F, 0x00000007}
};

static struct memwrite_data gpioserv_dis_safe_mode_array[] = {
	{0x4A100078, 0x0000010F, 0x00000003}
};

static struct memwrite_data gpiout_en_safe_mode_array[] = {
	{0x4A10007C, 0x0000011F, 0x0000000F}
};

static struct memwrite_data gpiout_dis_safe_mode_array[] = {
	{0x4A10007C, 0x0000010F, 0x00000003}
};

static struct memwrite_data ios_en_safe_mode_array[] = {
	{0x4A100138, 0x011F0000, 0x010B0000},
	{0x4A100140, 0x0000011F, 0x0000010B}
};

static struct memwrite_data ios_dis_safe_mode_array[] = {
	{0x4A100138, 0x011F0000, 0x01010000},
	{0x4A100140, 0x0000011F, 0x00000001}
};

static struct memwrite_data tx_en_safe_mode_array[] = {
	{0x4A100140, 0x00000007, 0x00000007}
};

static struct memwrite_data tx_dis_safe_mode_array[] = {
	{0x4A100140, 0x00000007, 0x00000001}
};
