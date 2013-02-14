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

#ifndef _MOTSOC1_H
#define _MOTSOC1_H

#include <linux/ioctl.h>  /* For IOCTL macros */

/** The following define the IOCTL command values via the ioctl macros */
#define MOTSOC1_IOCTL_BASE 199
#define MOTSOC1_IOCTL_GET_FIRMWARE_VERSIONS _IOR(MOTSOC1_IOCTL_BASE, 0, int)
#define MOTSOC1_IOCTL_FLASH_WRITER_MODE _IOW(MOTSOC1_IOCTL_BASE, 1, int)
#define MOTSOC1_IOCTL_SET_PINS _IOW(MOTSOC1_IOCTL_BASE, 2, int)
#define MOTSOC1_IOCTL_CHIPERASE _IOW(MOTSOC1_IOCTL_BASE, 3, int)
#define MOTSOC1_IOCTL_BLOCKERASE _IOW(MOTSOC1_IOCTL_BASE, 4, int)
#define MOTSOC1_IOCTL_SET_ROM_ADDRESS _IOW(MOTSOC1_IOCTL_BASE, 5, int)
#define MOTSOC1_IOCTL_SET_FLASH_BYTE _IOW(MOTSOC1_IOCTL_BASE, 6, int)
#define MOTSOC1_IOCTL_SEND_PROGRAMMING_FIRMWARE _IOW(MOTSOC1_IOCTL_BASE, 7, int)
#define MOTSOC1_IOCTL_RESET _IOW(MOTSOC1_IOCTL_BASE, 8, int)
#define MOTSOC1_IOCTL_PARAMETER_SETTING_MODE _IOW(MOTSOC1_IOCTL_BASE, 9, int)
#define MOTSOC1_IOCTL_TEST_READ	_IOW(MOTSOC1_IOCTL_BASE, 10, int)
#define MOTSOC1_IOCTL_TEST_WRITE _IOW(MOTSOC1_IOCTL_BASE, 11, int)
#define MOTSOC1_IOCTL_TEST_WRITE_READ _IOW(MOTSOC1_IOCTL_BASE, 12, int)
#define MOTSOC1_IOCTL_SET_DEBUG _IOW(MOTSOC1_IOCTL_BASE, 13, int)
#define MOTSOC1_IOCTL_GET_MODE _IOW(MOTSOC1_IOCTL_BASE, 14, int)
#define MOTSOC1_IOCTL_POWER_OFF_MODE _IOW(MOTSOC1_IOCTL_BASE, 15, int)
#define MOTSOC1_IOCTL_CHECK_SUM_ENTIRE_MEM _IOW(MOTSOC1_IOCTL_BASE, 16, int)
#define MOTSOC1_IOCTL_CHECK_SUM_LAST_32KB _IOW(MOTSOC1_IOCTL_BASE, 17, int)
#define MOTSOC1_IOCTL_SET_UPGRADE_STATUS _IOW(MOTSOC1_IOCTL_BASE, 18, int)
#define MOTSOC1_IOCTL_GET_UPGRADE_STATUS _IOR(MOTSOC1_IOCTL_BASE, 19, int)

#define MOTSOC1_MAX_PACKET_LENGTH (1024*8)
#define MOTSOC1_BLOCKERASE_LENGTH (1024*64)
#define MOTSOC1_BLOCKWRITE_LENGTH (1024*64)
#define MOTSOC1_ROM_ADD_START 0x10000000
#define MOTSOC1_ROM_PROGRAM_AREA_START MOTSOC1_FLASH_ROM_ADD_START
#define MOTSOC1_ROM_MEMORY_MAP_AREA_START 0x10170000
#define MOTSOC1_ROM_FSHD_START 0x101F0000
#define MOTSOC1_ROM_SIZE 0x01F8000
#define MOTSOC1_ROM_FLASH_ALL_SIZE 0x01F8000
#define MOTSOC1_ROM_FLASH_FIRMWARE_SIZE \
	(MOTSOC1_ROM_MEMORY_MAP_AREA_START - MOTSOC1_ROM_ADD_START)
#define MOTSOC1_ROM_FLASH_PARAMETER_SIZE \
	(MOTSOC1_ROM_FSHD_START - MOTSOC1_ROM_MEMORY_MAP_AREA_START)

enum motsoc1_mode {
	UNINITIALIZED,
	POWER_OFF_MODE,
	STANDBY_MODE,
	FLASH_WRITER_MODE,
	SYSTEM_INIT_MODE,
	PARAMETER_SETTING_MODE,
	MONITOR_MODE,
	CAPTURE_MODE,
	INVALID_MODE,
};

enum ispupgrade_status{
	NO_NEED_TO_FLASH,
	FLASHING,
	FLASHING_COMPLETE
};

#ifdef __KERNEL__
struct motsoc1_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	void (*mclk_on)(void);
	void (*mclk_off)(void);
	int gpio_reset;
};
#endif /* __KERNEL__ */

#endif /* _MOTSOC1_H */
