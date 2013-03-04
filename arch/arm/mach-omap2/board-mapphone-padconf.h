/*
 * board-mapphone-padconf.h
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __BOARD_MAPPHONE_PADCONF_H
#define __BOARD_MAPPHONE_PADCONF_H

struct padconf_core_entry {
	u16 offset;
	u8 mode;
	u8 pull_type;
	u8 input_en;
	u8 offmode_en;
	u8 offout_type;
	u8 offpull_type;
	u8 offwkup_en;
} __attribute__ ((__packed__));

struct padconf_wkup_entry {
	u16 offset;
	u8 mode;
	u8 pull_type;
	u8 input_en;
	u8 offwkup_en;
} __attribute__ ((__packed__));

#define OMAP44XX_CORE_PADCONF_SETTING(mode, input_en, pull, offmode, offout, \
			offpull, offwkup) \
		(((u16) mode) | ((u16) input_en) << 8 | ((u16) pull) << 3 | \
		(((u16) (offmode) << 9) | ((u16) (offout) << 10) | \
		((u16) (offpull) << 12) | ((u16) (offwkup) << 14)))

#define OMAP44XX_WKUP_PADCONF_SETTING(mode, input_en, pull, offwkup) \
		(((u16) mode) | ((u16) input_en) << 8 | ((u16) pull) << 3 | \
		((u16) (offwkup) << 14))

#define OMAP44XX_CORE_PADCONF_OFFSET_BEGIN	0x0040
#define OMAP44XX_CORE_PADCONF_OFFSET_END	0x01D4

#define OMAP44XX_WKUP_PADCONF_BASE		0x4A31E000
#define OMAP44XX_WKUP_PADCONF_OFFSET_BEGIN	0x0040
#define OMAP44XX_WKUP_PADCONF_OFFSET_END	0x0074

extern void __init mapphone_omap44xx_padconf_init(void);

#endif
