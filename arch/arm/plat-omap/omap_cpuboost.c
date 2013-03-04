/*
 * arch/arm/plat-omap/omap_cpuboost.c
 *
 * Copyright (C) 2011 Motorola Mobility Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <plat/common.h>
#include "../mach-omap2/dvfs.h"

#define OMAP_CPUBOOST_TIME_MAX	10000		/* 10sec */
#define OMAP_CPUBOOST_FREQ_MIN	300000000	/* 300 MHz */
#define OMAP_CPUBOOST_FREQ_MAX	1000000000	/* 1 GHz */

struct cpuboost_work_struct {
	struct delayed_work work;
};

struct cpuboost_main_struct {
	struct cpuboost_work_struct *work;
};

static struct mutex lock;
static struct device *device;
static struct cpuboost_main_struct *cbs;
static unsigned long cpuboost_time;

static int cpuboost_time_set(const char *val, struct kernel_param *kp)
{
	struct device *mpu_dev;
	int ret = param_set_ulong(val, kp);
	mpu_dev = omap2_get_mpuss_device();

	mutex_lock(&lock);

	cpuboost_time = (cpuboost_time > OMAP_CPUBOOST_TIME_MAX) ?
				 OMAP_CPUBOOST_TIME_MAX : cpuboost_time;

	cancel_delayed_work(&(cbs->work->work));
	printk(KERN_INFO "cpuboost_time_set = %ld\n", cpuboost_time);
	if (cpuboost_time > 0) {
		omap_device_scale(device, mpu_dev, OMAP_CPUBOOST_FREQ_MAX);
		schedule_delayed_work(&(cbs->work->work),
					msecs_to_jiffies(cpuboost_time));
	}

	if ((ret) || (cpuboost_time == 0)) {
		omap_device_scale(device, mpu_dev, OMAP_CPUBOOST_FREQ_MIN);
		printk(KERN_INFO "cpuboost_time cleared\n");
	}
	mutex_unlock(&lock);

	return ret;
}

module_param_call(cpuboost_time, cpuboost_time_set, param_get_uint,
					&cpuboost_time, 0644);

static void cpuboost_process_work(struct work_struct *work)
{
	struct device *mpu_dev = omap2_get_mpuss_device();

	mutex_lock(&lock);
	omap_device_scale(device, mpu_dev, OMAP_CPUBOOST_FREQ_MIN);
	printk(KERN_INFO "cpuboost_time cleared\n");
	cpuboost_time = 0;
	mutex_unlock(&lock);
}

static int __init cpuboost_init_module(void)
{
	mutex_init(&lock);
	cbs = kzalloc(sizeof(struct cpuboost_main_struct), GFP_KERNEL);
	device = kzalloc(sizeof(struct device), GFP_KERNEL);
	dev_set_name(device, "omap_cpuboost");

	cbs->work = kzalloc(sizeof(struct cpuboost_work_struct), GFP_KERNEL);
	INIT_DELAYED_WORK(&(cbs->work->work), cpuboost_process_work);

	return 0;
}

module_init(cpuboost_init_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Motorola Mobility");
MODULE_DESCRIPTION("OMAP CPUBOOST");
