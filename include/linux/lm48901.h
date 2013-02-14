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

#ifndef __LM48901_H__
#define __LM48901_H__

#include <linux/ioctl.h>

extern const unsigned int lm48901_tab_2p1_3ch[];
extern const unsigned int lm48901_tab_stereo_2ch[];

struct lm48901_platform_data {
	int amp_en_gpio;
	int i2c_en_gpio;
	unsigned char orientation;
	unsigned char mode;
} __attribute__ ((packed)) ;

struct lm48901_regacc {
	unsigned short reg;
	unsigned int value;
} __attribute__ ((packed)) ;

/*
Orientation Type
*/
enum {
	LM48901_ORIENTATION_A,
	LM48901_ORIENTATION_B,
	LM48901_ORIENTATION_C,
	LM48901_ORIENTATION_D,
};

/*
Output Type
*/
enum {
	LM48901_STEREO,
	LM48901_WOOFER,
};

/*
Unmute/Mute Type
*/
enum {
	LM48901_UNMUTE,
	LM48901_MUTE,
};

#define LM48901_IOCTL_BASE 'L'

#define LM48901_IOCTL_AMP_DISABLE	\
	_IO(LM48901_IOCTL_BASE, 0x01)

#define LM48901_IOCTL_AMP_ENABLE \
	_IO(LM48901_IOCTL_BASE, 0x02)

#define LM48901_IOCTL_INIT \
	_IOW(LM48901_IOCTL_BASE, 0x03, unsigned char)

#define LM48901_IOCTL_SET_MUTE \
	_IOW(LM48901_IOCTL_BASE, 0x04, bool)

#define LM48901_IOCTL_SET_I2S_LVL \
	_IOW(LM48901_IOCTL_BASE, 0x05, unsigned char)

#define LM48901_IOCTL_SET_ORIENTATION \
	_IOW(LM48901_IOCTL_BASE, 0x06, unsigned char)

#define LM48901_IOCTL_SET_MODE \
	_IOW(LM48901_IOCTL_BASE, 0x07, unsigned char)

/*
APIs below are just for testing.
*/
#define LM48901_IOCTL_GET_REG	\
	_IOR(LM48901_IOCTL_BASE, 0x10, unsigned short)

#define LM48901_IOCTL_SET_REG \
	_IOW(LM48901_IOCTL_BASE, 0x11, struct lm48901_regacc)

#define LM48901_IOCTL_I2C_DISABLE \
	_IO(LM48901_IOCTL_BASE, 0x12)

#define LM48901_IOCTL_I2C_ENABLE \
	_IO(LM48901_IOCTL_BASE, 0x13)

#define LM48901_IOCTL_SET_OUT_SEL \
	_IOW(LM48901_IOCTL_BASE, 0x14, unsigned char)

#define LM48901_IOCTL_SET_CH_SEL \
	_IOW(LM48901_IOCTL_BASE, 0x15, unsigned char)

#define LM48901_IOCTL_LOAD_COEF \
	_IOW(LM48901_IOCTL_BASE, 0x16, unsigned char)

#define LM48901_IOCTL_SET_G1_GAIN_0 \
	_IOW(LM48901_IOCTL_BASE, 0x17, unsigned char)

#define LM48901_IOCTL_SET_POST_GAIN_0 \
	_IOW(LM48901_IOCTL_BASE, 0x18, unsigned char)

#define LM48901_IOCTL_SET_G1_GAIN_1 \
	_IOW(LM48901_IOCTL_BASE, 0x19, unsigned char)

#define LM48901_IOCTL_SET_POST_GAIN_1 \
	_IOW(LM48901_IOCTL_BASE, 0x1A, unsigned char)

#define LM48901_IOCTL_USE_DSP \
	_IOW(LM48901_IOCTL_BASE, 0x1B, unsigned char)

#endif  /* __LM48901_H__ */
