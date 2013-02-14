/*
 * Copyright (C) 2009 Motorola, Inc.
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

#ifndef _LINUX_ISL29032_H__
#define _LINUX_ISL29032_H__

#include <linux/ioctl.h>

#ifdef __KERNEL__

#include <linux/types.h>

#define LD_ISL29032_NAME "isl29032"

#define ISL29032_REGULATOR_NAME_LENGTH	10

struct isl29032_platform_data {
	u8	configure;
	u8	interrupt_cntrl;
	u8	prox_lower_threshold;
	u8	prox_higher_threshold;
	u8	crosstalk_vs_covered_threshold;
	u8	default_prox_noise_floor;
	u8	num_samples_for_noise_floor;
	u32	lens_percent_t;
	u16	irq;
	u8	regulator_name[ISL29032_REGULATOR_NAME_LENGTH];
} __packed;

#endif	/* __KERNEL__ */

#define ISL29032_IOCTL_BASE		0xA3
#define ISL29032_IOCTL_GET_ENABLE	_IOR(ISL29032_IOCTL_BASE, 0x00, char)
#define ISL29032_IOCTL_SET_ENABLE	_IOW(ISL29032_IOCTL_BASE, 0x01, char)
#define ISL29032_IOCTL_GET_LIGHT_ENABLE	_IOR(ISL29032_IOCTL_BASE, 0x02, char)
#define ISL29032_IOCTL_SET_LIGHT_ENABLE	_IOW(ISL29032_IOCTL_BASE, 0x03, char)

#endif	/* _LINUX_ISL29032_H__ */
