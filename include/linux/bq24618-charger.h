/*
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

#ifndef _LINUX_BQ24618_CHARGER_H__
#define _LINUX_BQ24618_CHARGER_H__

#define BQ24618_NAME "bq24618"

#ifdef __KERNEL__

struct bq24618_platform_data {
	u8 bq24618_enable;
	u8 iset1;
	u8 iset2;
	u8 power_good;
	u8 enable_gpio;
	u8 iset1_gpio;
	u8 iset2_gpio;
	u8 pg_gpio;
	u8 led_gpio;

} __packed;

#endif	/* __KERNEL__ */

enum {
	BQ24618_IOCTL_NUM_INIT,
	BQ24618_IOCTL_NUM_ENABLE,
	BQ24618_IOCTL_NUM_SET_CURRENT,
	BQ24618_IOCTL_NUM_LED_ENABLE,
	BQ24618_IOCTL_NUM_VSET,
	BQ24618_IOCTL_NUM_GET_POWERGOOD
};

enum {
	BQ24618_SET_CURRENT_0MA,
	BQ24618_SET_CURRENT_500MA,
	BQ24618_SET_CURRENT_750MA,
	BQ24618_SET_CURRENT_1600MA,
	BQ24618_SET_CURRENT_3000MA,
	BQ24618_SET_CURRENT_DEFAULT = BQ24618_SET_CURRENT_3000MA,
	BQ24618_SET_CURRENT_MAX,
};

#define BQ24618_IOCTL_INIT \
	_IOW(0, BQ24618_IOCTL_NUM_INIT, unsigned char)
#define BQ24618_IOCTL_ENABLE \
	_IOW(0, BQ24618_IOCTL_NUM_ENABLE, unsigned char)
#define BQ24618_IOCTL_SET_CURRENT \
	_IOW(0, BQ24618_IOCTL_NUM_SET_CURRENT, unsigned char)
#define BQ24618_IOCTL_LED_ENABLE \
	_IOW(0, BQ24618_IOCTL_NUM_LED_ENABLE, unsigned char)
#define BQ24618_IOCTL_VSET \
	_IOW(0, BQ24618_IOCTL_NUM_VSET, unsigned char)
#define BQ24618_IOCTL_GET_POWERGOOD \
	_IOR(0, BQ24618_IOCTL_NUM_GET_POWERGOOD, unsigned char)

#endif	/* _LINUX_BQ24618_CHARGER_H__ */
