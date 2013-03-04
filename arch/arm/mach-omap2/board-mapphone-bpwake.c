/*
 * linux/arch/arm/mach-omap2/board-mapphone-bpwake.c
 *
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/gpio_mapping.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/prom.h>
#include "board-mapphone.h"

#ifdef CONFIG_PM
extern void omap_uart_block_sleep(int num);
#endif

static struct wake_lock baseband_wakeup_wakelock;

#define MAPPHONE_AP_UART 0

static irqreturn_t mapphone_bpwake_irqhandler(int irq, void *unused)
{
#ifdef CONFIG_PM
	omap_uart_block_sleep(MAPPHONE_AP_UART);
#endif
	/*
	 * uart_block_sleep keeps uart clock active for 500 ms,
	 * prevent suspend for 1 sec to be safe
	 */
	wake_lock_timeout(&baseband_wakeup_wakelock, HZ);
	return IRQ_HANDLED;
}

static int mapphone_bpwake_probe(struct platform_device *pdev)
{
	int rc;

	int apwake_trigger_gpio = *(int *)(pdev->dev.platform_data);

	gpio_request(apwake_trigger_gpio, "BP -> AP IPC trigger");
	gpio_direction_input(apwake_trigger_gpio);

	wake_lock_init(&baseband_wakeup_wakelock, WAKE_LOCK_SUSPEND, "bpwake");

	rc = request_irq(gpio_to_irq(apwake_trigger_gpio),
			 mapphone_bpwake_irqhandler,
			 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			 "Remote Wakeup", NULL);
	if (rc) {
		wake_lock_destroy(&baseband_wakeup_wakelock);
		printk(KERN_ERR
		       "Failed requesting APWAKE_TRIGGER irq (%d)\n", rc);
		return rc;
	}
	enable_irq_wake(gpio_to_irq(apwake_trigger_gpio));
	return 0;
}

static int mapphone_bpwake_remove(struct platform_device *pdev)
{
	int apwake_trigger_gpio = *(int *)(pdev->dev.platform_data);

	wake_lock_destroy(&baseband_wakeup_wakelock);

	free_irq(gpio_to_irq(apwake_trigger_gpio), NULL);
	gpio_free(apwake_trigger_gpio);
	return 0;
}

static int mapphone_bpwake_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	return 0;
}

static int mapphone_bpwake_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mapphone_bpwake_driver = {
	.probe		= mapphone_bpwake_probe,
	.remove		= mapphone_bpwake_remove,
	.suspend	= mapphone_bpwake_suspend,
	.resume		= mapphone_bpwake_resume,
	.driver		= {
		.name		= "mapphone_bpwake",
		.owner		= THIS_MODULE,
	},
};

static int __init mapphone_bpwake_init(void)
{
	return platform_driver_register(&mapphone_bpwake_driver);
}

static void __exit mapphone_bpwake_exit(void)
{
	platform_driver_unregister(&mapphone_bpwake_driver);
}

module_init(mapphone_bpwake_init);
module_exit(mapphone_bpwake_exit);

MODULE_DESCRIPTION("Mapphone BP Wake Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Motorola Mobility");
