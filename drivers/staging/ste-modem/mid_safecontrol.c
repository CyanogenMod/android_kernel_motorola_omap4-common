/*
 * Copyright (C) ST-Ericsson AB 2011
 * Author:	Erwan Bracq / Erwan.Bracq@stericsson.com
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include "mid_safecontrol_tab.h"

#define	DRV_NAME		"mid_safecontrol"
#define PFX			DRV_NAME ": "

/* The following section should match MID opcode */
#define EN_GPIOSERV_SAFE_MODE	0x1
#define DIS_GPIOSERV_SAFE_MODE	0x2
#define EN_GPIOUT_SAFE_MODE	0x3
#define DIS_GPIOUT_SAFE_MODE	0x4
#define EN_IOS_SAFE_MODE	0x5
#define DIS_IOS_SAFE_MODE	0x6
#define PLATFORM_REBOOT		0x7
#define EN_TX_SAFE_MODE		0xB
#define DIS_TX_SAFE_MODE	0xC

struct mid_safecontrol_struct {
	struct platform_device	pdev;
};

static struct mid_safecontrol_struct mid_safecontrol;

static struct class mid_safecontrol_class = {
	.name = "mid_safecontrol"
};

void multi_memwrite(struct memwrite_data *entries, size_t length)
{
	int index = 0;
	unsigned long regval = 0;
	while (index != length) {
		regval = readl(&entries[index].address);
	regval = (regval & ~entries[index].bitmask) |
			(entries[index].value & entries[index].bitmask);
		writel(regval, &entries[index].address);
		index++;
	}
}

static ssize_t execute_opcode(struct class *class,
				struct class_attribute *attr,
				const char *buf, size_t count)
{
	long opcode = 0;
	if (strict_strtol(buf, 10, &opcode) == -EINVAL) {
		printk(KERN_WARNING PFX "Received invalid opcode.");
		return PAGE_SIZE;
	}
	printk(KERN_WARNING PFX "Got opcode: %ld", opcode);
	switch (opcode) {
	case EN_GPIOSERV_SAFE_MODE:
		printk(KERN_WARNING PFX
			"Execute GPIO SERVICE enable safe mode.\n");
		multi_memwrite(gpioserv_en_safe_mode_array,
			ARRAY_SIZE(gpioserv_en_safe_mode_array));
		break;
	case DIS_GPIOSERV_SAFE_MODE:
		printk(KERN_WARNING PFX
			"Execute GPIO SERVICE disable safe mode (normal mode).\n");
		multi_memwrite(gpioserv_dis_safe_mode_array,
			ARRAY_SIZE(gpioserv_dis_safe_mode_array));
		break;
	case EN_GPIOUT_SAFE_MODE:
		printk(KERN_WARNING PFX
			"Execute GPIO OUT enable safe mode.\n");
		multi_memwrite(gpiout_en_safe_mode_array,
			ARRAY_SIZE(gpiout_en_safe_mode_array));
		break;
	case DIS_GPIOUT_SAFE_MODE:
		printk(KERN_WARNING PFX
			"Execute GPIO OUT disable safe mode (normal mode)).\n");
		multi_memwrite(gpiout_dis_safe_mode_array,
			ARRAY_SIZE(gpiout_dis_safe_mode_array));
		break;
	case EN_IOS_SAFE_MODE:
		printk(KERN_WARNING PFX
			"Execute IOs OUT enable safe mode.\n");
		multi_memwrite(ios_en_safe_mode_array,
			ARRAY_SIZE(ios_en_safe_mode_array));
		break;
	case DIS_IOS_SAFE_MODE:
		printk(KERN_WARNING PFX
			"Execute IOs OUT disable safe mode (normal mode).\n");
		multi_memwrite(ios_dis_safe_mode_array,
			ARRAY_SIZE(ios_dis_safe_mode_array));
		break;
	case PLATFORM_REBOOT:
		printk(KERN_WARNING PFX "Execute platform RESET.\n");
			multi_memwrite(platform_reset_array,
		ARRAY_SIZE(platform_reset_array));
		break;
	case EN_TX_SAFE_MODE:
		printk(KERN_WARNING PFX "Execute TX enable safe mode.\n");
		multi_memwrite(tx_en_safe_mode_array, ARRAY_SIZE(tx_en_safe_mode_array));
		break;
	case DIS_TX_SAFE_MODE:
		printk(KERN_WARNING PFX "Execute TX disable safe mode (normal mode).\n");
		multi_memwrite(tx_dis_safe_mode_array, ARRAY_SIZE(tx_dis_safe_mode_array));
		break;

	default:
		break;
	}
	return PAGE_SIZE;
}

CLASS_ATTR(exec_opcode, 0200, NULL, execute_opcode);

static int __devexit mid_safecontrol_remove(struct platform_device *pdev)
{
	class_remove_file(&mid_safecontrol_class, &class_attr_exec_opcode);
	class_unregister(&mid_safecontrol_class);
	return 0;
}

static int __devinit mid_safecontrol_probe(struct platform_device *pdev)
{
	int res = 0;

	/* User space entries. */
	/* Register class for SYSFS. */
	res = class_register(&mid_safecontrol_class);
	if (unlikely(res)) {
		printk(KERN_WARNING PFX
		       "err: %d, can't create sysfs node.\n", res);
		goto err_class_register_failed;
	}

	/* Create SYSFS node - exec_opcode. */
	res = class_create_file(&mid_safecontrol_class,
				&class_attr_exec_opcode);
	if (unlikely(res)) {
		printk(KERN_WARNING PFX
		       "err: %d, can't create sysfs node.\n", res);
		goto err_sysfs_create_attr_failed;
	}

	return res;

err_sysfs_create_attr_failed:
	class_unregister(&mid_safecontrol_class);
err_class_register_failed:

	return res;
}

static struct platform_driver mid_safecontrol_driver = {
	.probe		= mid_safecontrol_probe,
	.remove		= __devexit_p(mid_safecontrol_remove),
	.driver		= {
				.owner = THIS_MODULE,
				.name = DRV_NAME,
			}
};

static int __init mid_safecontrol_init(void)
{
	int err = 0;

	err = platform_driver_register(&mid_safecontrol_driver);
	if (err)
		return err;

	mid_safecontrol.pdev.name = DRV_NAME;
	err = platform_device_register(&mid_safecontrol.pdev);

	if (err) {
		printk(KERN_ERR PFX "failed to register platform device\n");
		goto unreg_platform_driver;
	}

	return 0;

unreg_platform_driver:
	platform_driver_unregister(&mid_safecontrol_driver);
	return err;
}

static void __exit mid_safecontrol_exit(void)
{
	platform_device_unregister(&mid_safecontrol.pdev);
	platform_driver_unregister(&mid_safecontrol_driver);
}

MODULE_AUTHOR("Erwan Bracq <erwan.bracq@stericsson.com>");
MODULE_DESCRIPTION("MID safe mode helper driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(mid_safecontrol_init);
module_exit(mid_safecontrol_exit);
