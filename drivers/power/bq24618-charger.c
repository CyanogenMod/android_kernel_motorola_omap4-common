/*
 * Copyright (C) 2007-2011 Motorola, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/stddef.h>
#include <linux/slab.h>

#include <linux/bq24618-charger.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>

#define BQ24618_CPCAP_CHRG_LED_12V	CPCAP_REG_GPIO1
#define BQ24618_CPCAP_CHRG_ENABLE	CPCAP_REG_GPIO2
#define BQ24618_CPCAP_CHRG_VOL_SET	CPCAP_REG_GPIO3
#define BQ24618_CPCAP_CHRG_ISET_1	CPCAP_REG_GPIO4
#define BQ24618_CPCAP_CHRG_ISET_2	CPCAP_REG_GPIO5
#define BQ24618_CPCAP_CHRG_DET_PG	CPCAP_REG_GPIO6

#define BQ24618_GPIOx_MACROML		CPCAP_BIT_GPIO0MACROML
#define BQ24618_GPIOx_MACROMH		CPCAP_BIT_GPIO0MACROMH
#define BQ24618_GPIOx_MACROINITL	CPCAP_BIT_GPIO0MACROINITL
#define BQ24618_GPIOx_MACROINITH	CPCAP_BIT_GPIO0MACROINITH

#define BQ24618_GPIOxMUX0		CPCAP_BIT_GPIO0MUX0
#define BQ24618_GPIOxMUX1		CPCAP_BIT_GPIO0MUX1
#define BQ24618_GPIOxMUX2		CPCAP_BIT_GPIO5MUX2

#define BQ24618_GPIOxDRV		CPCAP_BIT_GPIO0DRV
#define BQ24618_GPIOxDIR		CPCAP_BIT_GPIO0DIR
#define BQ24618_GPIOxPUEN		CPCAP_BIT_GPIO0PUEN
#define BQ24618_GPIOxVLEV		CPCAP_BIT_GPIO0VLEV
#define BQ24618_GPIOxOT			CPCAP_BIT_GPIO0OT
#define BQ24618_GPIOxS			CPCAP_BIT_GPIO0S

static int bq24618_open(struct inode *inode, struct file *file);
static long bq24618_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg);

const struct file_operations bq24618_fops = {
	.owner	= THIS_MODULE,
	.open	= bq24618_open,
	.unlocked_ioctl = bq24618_ioctl,
};

static struct miscdevice bq24618_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= BQ24618_NAME,
	.fops	= &bq24618_fops,
};

/* cpcap related */
struct bq24618_cpcap_data {
	struct cpcap_device *cpcap_dev;
};

static struct bq24618_cpcap_data *cpcap_info;
static int bq24618_enabled = 1;

static DEFINE_MUTEX(bq24618_lock);

/* *****************************************************

	Low level interface routine.

* *****************************************************/

static void bq24618_chg_init(struct bq24618_cpcap_data *info)
{
	unsigned short mask = BQ24618_GPIOxDRV |
			      BQ24618_GPIOxDIR |
			      BQ24618_GPIOxPUEN |
			      BQ24618_GPIOxOT |
			      BQ24618_GPIOxMUX0 |
			      BQ24618_GPIOxMUX1 |
			      BQ24618_GPIOxVLEV |
			      BQ24618_GPIOx_MACROML |
			      BQ24618_GPIOx_MACROMH |
			      BQ24618_GPIOx_MACROINITL |
			      BQ24618_GPIOx_MACROINITH;

	cpcap_regacc_write(info->cpcap_dev, BQ24618_CPCAP_CHRG_ENABLE,
		BQ24618_GPIOxDIR |
		BQ24618_GPIOx_MACROML |
		BQ24618_GPIOx_MACROMH |
		BQ24618_GPIOxVLEV, mask);

	if (bq24618_enabled) {
		bq24618_enabled = 0;
		pr_info("BQ24618 disabled\n");
	}

	cpcap_regacc_write(info->cpcap_dev, BQ24618_CPCAP_CHRG_LED_12V,
		BQ24618_GPIOxDIR |
		BQ24618_GPIOx_MACROML |
		BQ24618_GPIOx_MACROMH, mask);

	cpcap_regacc_write(info->cpcap_dev, BQ24618_CPCAP_CHRG_VOL_SET,
		BQ24618_GPIOxDIR |
		BQ24618_GPIOx_MACROML |
		BQ24618_GPIOx_MACROMH, mask);

	cpcap_regacc_write(info->cpcap_dev, BQ24618_CPCAP_CHRG_ISET_1,
		BQ24618_GPIOxDIR |
		BQ24618_GPIOx_MACROML |
		BQ24618_GPIOx_MACROMH, mask);

	cpcap_regacc_write(info->cpcap_dev, BQ24618_CPCAP_CHRG_ISET_2,
		BQ24618_GPIOxDIR |
		BQ24618_GPIOx_MACROML |
		BQ24618_GPIOx_MACROMH,
		mask | BQ24618_GPIOxMUX2);

	cpcap_regacc_write(info->cpcap_dev, BQ24618_CPCAP_CHRG_DET_PG,
		0,
		mask | BQ24618_GPIOxMUX2);

}

static int bq24618_open(struct inode *inode, struct file *file)
{
	pr_info("BQ24618 driver opened\n");
	file->private_data = cpcap_info;
	return 0;
}

static long bq24618_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	unsigned char rwbuf;
	unsigned short iset1 = 0;
	unsigned short iset2 = 0;
	unsigned short read;
	struct bq24618_cpcap_data *bq24618_cpcap = file->private_data;
	long ret = 0;

	mutex_lock(&bq24618_lock);

	/* get information from user */
	switch (cmd) {

	case BQ24618_IOCTL_INIT:
		bq24618_chg_init(bq24618_cpcap);
		break;

	case BQ24618_IOCTL_ENABLE:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
			ret = -EFAULT;
			goto out;
		}

		if (bq24618_enabled && !rwbuf) {
			bq24618_enabled = 0;
			pr_info("BQ24618 disabled\n");
		} else if (!bq24618_enabled && rwbuf) {
			bq24618_enabled = 1;
			pr_info("BQ24618 enabled\n");
		}
		/* if passed variable is 0, we will set CHRG_EN pin as low
		 * (disable) otherwise set as HIGH(enable) */
		if (rwbuf)
			cpcap_regacc_write(bq24618_cpcap->cpcap_dev,
				BQ24618_CPCAP_CHRG_ENABLE,
				BQ24618_GPIOxDRV,
				BQ24618_GPIOxDRV);
		else
			cpcap_regacc_write(bq24618_cpcap->cpcap_dev,
				BQ24618_CPCAP_CHRG_ENABLE,
				0, BQ24618_GPIOxDRV);
		break;

	case BQ24618_IOCTL_SET_CURRENT:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
			ret = -EFAULT;
			goto out;
		}
		/* it passes the current setting type, 1(500),2(750),3(1500)
		 * 4(3000)*/
		switch (rwbuf) {
		case BQ24618_SET_CURRENT_500MA:
			pr_info("BQ24618 500mA set\n");
			iset1 = BQ24618_GPIOxDRV;
			iset2 = BQ24618_GPIOxDRV;
			break;

		case BQ24618_SET_CURRENT_750MA:
			pr_info("BQ24618 750mA set\n");
			iset1 = 0;
			iset2 = BQ24618_GPIOxDRV;
			break;

		case BQ24618_SET_CURRENT_1600MA:
			pr_info("BQ24618 1600mA set\n");
			iset1 = BQ24618_GPIOxDRV;
			iset2 = 0;
			break;

		case BQ24618_SET_CURRENT_3000MA:
			pr_info("BQ24618 3000mA set\n");
			iset1 = 0;
			iset2 = 0;
			break;

		default:
			pr_err("BQ24618 need current setting. Set default.\n");
			break;
		}
		cpcap_regacc_write(bq24618_cpcap->cpcap_dev,
			BQ24618_CPCAP_CHRG_ISET_1,
			iset1, BQ24618_GPIOxDRV);
		cpcap_regacc_write(bq24618_cpcap->cpcap_dev,
			BQ24618_CPCAP_CHRG_ISET_2,
			iset2, BQ24618_GPIOxDRV);
		break;

	case BQ24618_IOCTL_LED_ENABLE:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
			ret = -EFAULT;
			goto out;
		}
		pr_info("BQ24618 LED %s\n", ((rwbuf) ? "on" : "off"));
		/* if passed variable is 0, we will set CHRG_LED pin as low
		 * (turn off) otherwise set as HIGH(turn on) */
		if (rwbuf)
			cpcap_regacc_write(bq24618_cpcap->cpcap_dev,
				BQ24618_CPCAP_CHRG_LED_12V,
				BQ24618_GPIOxDRV,
				BQ24618_GPIOxDRV);
		else
			cpcap_regacc_write(bq24618_cpcap->cpcap_dev,
				BQ24618_CPCAP_CHRG_LED_12V,
				0,
				BQ24618_GPIOxDRV);
		break;

	case BQ24618_IOCTL_VSET:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
			ret = -EFAULT;
			goto out;
		}
		pr_info("BQ24618 %s\n", ((rwbuf) ? "4.2V set" : "3.8V set"));
		/* if passed variable is 0, we will set VSET pin as low(3.8V)
		 * otherwise set as HIGH(4.2V) */
		if (rwbuf)
			cpcap_regacc_write(bq24618_cpcap->cpcap_dev,
				BQ24618_CPCAP_CHRG_VOL_SET,
				BQ24618_GPIOxDRV,
				BQ24618_GPIOxDRV);
		else
			cpcap_regacc_write(bq24618_cpcap->cpcap_dev,
				BQ24618_CPCAP_CHRG_VOL_SET,
				0,
				BQ24618_GPIOxDRV);
		break;

	case BQ24618_IOCTL_GET_POWERGOOD:
		cpcap_regacc_read(bq24618_cpcap->cpcap_dev,
			BQ24618_CPCAP_CHRG_DET_PG,
			&read);
		rwbuf = (unsigned char)(read & BQ24618_GPIOxS);
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf))) {
			ret = -EFAULT;
			goto out;
		}
		break;

	default:
		break;
	}

out:
	mutex_unlock(&bq24618_lock);
	return ret;
}

static int cpcap_bq24618_probe(struct platform_device *pdev)
{
	struct bq24618_cpcap_data *info;

	pr_info("%s:BQ24618 Probe enter\n", __func__);
	if (pdev == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;
	}
	info = kzalloc(sizeof(struct bq24618_cpcap_data), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->cpcap_dev = pdev->dev.platform_data;
	platform_set_drvdata(pdev, info);

	cpcap_info = info;

	if (misc_register(&bq24618_device) < 0) {
		kfree(info);
		pr_info("%s:Can't register /dev/bq24618\n", __func__);
		return -ENODEV;
	}

	bq24618_chg_init(info);

	pr_info("%s:BQ24618 probe exit\n", __func__);

	return 0;
}

static int cpcap_bq24618_remove(struct platform_device *pdev)
{
	struct cpcap_device *info = platform_get_drvdata(pdev);

	kfree(info);
	misc_deregister(&bq24618_device);

	return 0;
}


struct platform_driver cpcap_bq24618_driver = {
	.probe = cpcap_bq24618_probe,
	.remove = cpcap_bq24618_remove,
	.driver = {
		.name = "bq24618",
		.owner = THIS_MODULE,
		},
};

/*
 *  BQ24618 ChargerIC init
 */
static int __init cpcap_bq24618_init(void)
{
	return cpcap_driver_register(&cpcap_bq24618_driver);
}

static void __exit cpcap_bq24618_exit(void)
{
	platform_driver_unregister(&cpcap_bq24618_driver);
	pr_debug("BQ24618 ChargerIC driver: exit\n");
}

module_init(cpcap_bq24618_init);
module_exit(cpcap_bq24618_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("BQ24618 Charger IC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
