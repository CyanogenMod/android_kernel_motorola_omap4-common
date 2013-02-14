/*
 * arch/arm/plat-omap/include/mach/hdq.h
 *
 * Copyright (C) 2009 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#ifndef _OMAP2_HDQ_H
#define _OMAP2_HDQ_H

#define OMAP_SDQ_MODE 1
#define OMAP_HDQ_MODE 0

extern struct platform_device omap_hdq_device;

struct omap2_hdq_platform_config {
	unsigned short	mode;
	int id;
};

void omap_hdq1w_init(struct omap2_hdq_platform_config *pdata);

/**
 * struct omap_hdq_platform_data - OMAP HDQ controller platform data
 */
struct omap_hdq_platform_data {
	unsigned short	mode;
	int id;
	struct omap_i2c_dev_attr *dev_attr;
	void (*set_mpu_wkup_lat)(struct device *dev, int set);
	int (*device_enable) (struct platform_device *pdev);
	int (*device_shutdown) (struct platform_device *pdev);
	int (*device_idle) (struct platform_device *pdev);
};

/* Prototypes for OMAP platform I2C core initialization code */

struct omap_hdq_platform_data * __init omap_hdq_get_pdata(void);

#endif
