/*
 * Copyright (C) 2009 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * Copyright (C) 2010 Motorola, Inc.
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

#ifndef __KXTF9_H__
#define __KXTF9_H__

#include <linux/ioctl.h>

#define KXTF9_IOCTL_BASE		77
/* The following define the IOCTL command values via the ioctl macros */
#define KXTF9_IOCTL_SET_DELAY		_IOW(KXTF9_IOCTL_BASE, 0, int)
#define KXTF9_IOCTL_GET_DELAY		_IOR(KXTF9_IOCTL_BASE, 1, int)
#define KXTF9_IOCTL_SET_ENABLE		_IOW(KXTF9_IOCTL_BASE, 2, int)
#define KXTF9_IOCTL_GET_ENABLE		_IOR(KXTF9_IOCTL_BASE, 3, int)

#ifdef __KERNEL__

struct kxtf9_platform_data {
	int poll_interval;
	int min_interval;
	int g_range;
	int irq;

	u8 regulator[10];

	u8 tdt_always_on;
	u8 tdt_odr;
	u8 tdt_directions;

	u8 tdt_timer_init;
	u8 tdt_h_thresh_init;
	u8 tdt_l_thresh_init;
	u8 tdt_tap_timer_init;
	u8 tdt_total_timer_init;
	u8 tdt_latency_timer_init;
	u8 tdt_window_timer_init;

};

#endif /* __KERNEL__ */

#endif  /* __KXTF9_H__ */
