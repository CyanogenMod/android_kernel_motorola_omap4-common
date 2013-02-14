/*
 * Header file for:
 * Cypress CY8C201XX driver.
 * For use with Cypress CY8C201XX CapSense(R) parts.
 * Supported parts include:
 * CY8C201A0x
 *
 * Copyright (C) 2009-2011 Cypress Semiconductor, Inc.
 * Copyright (C) 2010-2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */

#ifndef __CY201_CORE_H__
#define __CY201_CORE_H__

#include <linux/kernel.h>
#include <linux/err.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define CY201_I2C_NAME                 "cy8c201xx"
#define CY201_DRIVER_VERSION           "Rev1-05M5"
#define CY201_DRIVER_DATE              "2011-08-31"
#define CY201_I2C_ADR_MASK		0x7F

#define CY_NUM_RETRY                26 /* max retries for rd/wr ops */

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* use the following defines for dynamic debug printing */
/*
 * Level 0: Default Level
 * All debug (cy201_dbg) prints turned off
 */
#define CY_DBG_LVL_0		0
/*
 * Level 1:  Used to verify driver and IC are working
 *    Input from IC, output to event queue
 */
#define CY_DBG_LVL_1		1
/*
 * Level 2:  Used to further verify/debug the IC
 *    Output to IC
 */
#define CY_DBG_LVL_2		2
/*
 * Level 3:  Used to further verify/debug the driver
 *    Driver internals
 */
#define CY_DBG_LVL_3		3
#define CY_DBG_LVL_MAX		CY_DBG_LVL_3

#define cy201_dbg(ts, l, f, a...) {\
	if ((ts->bus_ops->tsdebug) > ((l) - 1)) \
		pr_info(f, ## a);\
}
#else
#define cy201_dbg(ts, l, f, a...) {}
#endif

struct cy201_bus_ops {
	s32 (*write)(void *handle, u8 addr, u8 length,
		const void *values, int i2c_addr);
	s32 (*read)(void *handle, u8 addr, u8 length,
		void *values, int i2c_addr);
	struct device *dev;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	u8 tsdebug;
#endif
};

void *cy201_core_init(struct cy201_bus_ops *bus_ops,
	struct device *dev, int irq, char *name);
void cy201_core_release(void *handle);

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
int cy201_resume(void *handle);
int cy201_suspend(void *handle);
#endif

#endif /* __CY201_CORE_H__ */
