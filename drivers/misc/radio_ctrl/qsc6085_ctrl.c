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
 * 02111-1307  USA
 */
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/radio_ctrl/radio_class.h>
#include <linux/radio_ctrl/qsc6085_ctrl.h>

#define GPIO_MAX_NAME 30

enum qsc6085_status {
	QSC6085_STATUS_NORMAL,
	QSC6085_STATUS_FLASH,
	QSC6085_STATUS_RESETTING,
	QSC6085_STATUS_OFF,
	QSC6085_STATUS_UNDEFINED,
};

static const char * const qsc6085_status_str[] = {
	[QSC6085_STATUS_NORMAL]		= RADIO_STATUS_NORMAL_NAME,
	[QSC6085_STATUS_FLASH]		= RADIO_STATUS_FLASH_NAME,
	[QSC6085_STATUS_RESETTING]	= RADIO_STATUS_RESETTING_NAME,
	[QSC6085_STATUS_OFF]		= RADIO_STATUS_OFF_NAME,
	[QSC6085_STATUS_UNDEFINED]	= RADIO_STATUS_UNDEFINED_NAME,
};

struct qsc6085_info {
	unsigned int power_gpio;
	char power_name[GPIO_MAX_NAME];

	unsigned int ps_hold_gpio;
	char ps_hold_name[GPIO_MAX_NAME];

	unsigned int reset_gpio;
	char reset_name[GPIO_MAX_NAME];

	unsigned int flash_gpio;
	char flash_name[GPIO_MAX_NAME];

	bool boot_flash;
	enum qsc6085_status status;

	struct radio_dev rdev;
};

static ssize_t qsc6085_status_show(struct radio_dev *rdev, char *buff)
{
	struct qsc6085_info *info =
		container_of(rdev, struct qsc6085_info, rdev);

	pr_debug("%s: qsc6085_status = %d\n", __func__, info->status);
	if (info->status > QSC6085_STATUS_UNDEFINED)
		info->status = QSC6085_STATUS_UNDEFINED;

	return snprintf(buff, RADIO_STATUS_MAX_LENGTH, "%s\n",
		qsc6085_status_str[info->status]);
}

static ssize_t qsc6085_do_powerdown(struct qsc6085_info *info)
{
	int i, err = -1;

	pr_info("%s: powering down\n", __func__);

	/* Check to see if the modem is already powered down */
	if (!gpio_get_value(info->reset_gpio)) {
		pr_info("%s: modem already powered down.\n", __func__);
		info->status = QSC6085_STATUS_OFF;
		return 0;
	}

	/* Disable ability to notify clients of bp reset activity */
	disable_irq(gpio_to_irq(info->reset_gpio));

	pr_info("%s: initiate modem power down.\n", __func__);
	/* Press modem Power Button */
	gpio_direction_output(info->power_gpio, 1);
	mdelay(100);
	gpio_direction_output(info->power_gpio, 0);
	/* Wait up to 5 seconds for the modem to properly power down */
	for (i = 0; i < 10; i++) {
		if (!gpio_get_value(info->reset_gpio)) {
			pr_info("%s: modem powered down.\n", __func__);
			err = 0;
			break;
		}
		mdelay(500);
	}

	if (err) {
		/* Pull power from the modem */
		pr_info("%s: modem power down failed. Pull power.\n", __func__);
		gpio_direction_output(info->ps_hold_gpio, 1);
		mdelay(5);
		gpio_direction_output(info->ps_hold_gpio, 0);
	}

	info->status = QSC6085_STATUS_OFF;
	enable_irq(gpio_to_irq(info->reset_gpio));
	return err;
}

static ssize_t qsc6085_do_powerup(struct qsc6085_info *info)
{
	int i, err = -1;
	pr_debug("%s: enter\n", __func__);

	if (gpio_get_value(info->reset_gpio)) {
		pr_info("%s: modem already powered up.\n", __func__);
		return 0;
	}

	/* power on in normal or flash mode */
	if (info->boot_flash)
		gpio_direction_output(info->flash_gpio, 1);
	else
		gpio_direction_output(info->flash_gpio, 0);


	pr_info("%s: powering up\n", __func__);

	/* Press modem Power Button */
	gpio_direction_output(info->power_gpio, 1);
	mdelay(100);
	gpio_direction_output(info->power_gpio, 0);
	/* Wait up to 5 seconds for the modem to power up */
	for (i = 0; i < 10; i++) {
		if (gpio_get_value(info->reset_gpio)) {
			pr_info("%s: modem powered up.\n", __func__);
			err = 0;
			break;
		}
		mdelay(500);
	}

	if (!err) {
		if (info->boot_flash) {
			pr_debug("%s: started wrigley in flash mode\n",
				__func__);
			info->status = QSC6085_STATUS_FLASH;
		} else {
			pr_debug("%s: started wrigley in normal mode\n",
					__func__);
			info->status = QSC6085_STATUS_NORMAL;
		}
	} else {
		pr_err("%s: failed to start wrigley\n", __func__);
		info->status = QSC6085_STATUS_UNDEFINED;
	}

	return err;
}

static ssize_t qsc6085_set_flash_mode(struct qsc6085_info *info,
						bool enable)
{
	pr_debug("%s: set boot state to %d\n", __func__, enable);
	info->boot_flash = enable;
	return 0;
}

static ssize_t qsc6085_command(struct radio_dev *rdev, char *cmd)
{
	struct qsc6085_info *info =
		container_of(rdev, struct qsc6085_info, rdev);

	pr_info("%s: user command = %s\n", __func__, cmd);

	if (strcmp(cmd, "shutdown") == 0)
		return qsc6085_do_powerdown(info);
	else if (strcmp(cmd, "powerup") == 0)
		return qsc6085_do_powerup(info);
	else if (strcmp(cmd, "bootmode_normal") == 0)
		return qsc6085_set_flash_mode(info, 0);
	else if (strcmp(cmd, "bootmode_flash") == 0)
		return qsc6085_set_flash_mode(info, 1);

	pr_err("%s: command %s not supported\n", __func__, cmd);
	return -EINVAL;
}

static irqreturn_t qsc6085_reset_fn(int irq, void *data)
{
	struct qsc6085_info *info = (struct qsc6085_info *) data;
	pr_debug("%s:reset irq (%d) fired\n", __func__, irq);
	if (info->rdev.dev)
		kobject_uevent(&info->rdev.dev->kobj, KOBJ_CHANGE);
	return IRQ_HANDLED;
}

static irqreturn_t qsc6085_reset_isr(int irq, void *data)
{
	struct qsc6085_info *info = (struct qsc6085_info *) data;
	pr_debug("%s:  reset irq (%d) fired\n", __func__, irq);
	info->status = QSC6085_STATUS_RESETTING;
	return IRQ_WAKE_THREAD;
}

static int __devinit qsc6085_probe(struct platform_device *pdev)
{
	struct qsc6085_ctrl_platform_data *pdata = pdev->dev.platform_data;
	struct qsc6085_info *info;
	int reset_irq, err = 0;

	dev_info(&pdev->dev, "qsc6085_probe");
	pr_debug("%s: %s - %s\n", __func__, dev_name(&pdev->dev), pdata->name);

	info = kzalloc(sizeof(struct qsc6085_info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto err_exit;
	}

	platform_set_drvdata(pdev, info);

	/* setup radio_class device */
	info->rdev.name = pdata->name;
	info->rdev.status = qsc6085_status_show;
	info->rdev.command = qsc6085_command;

	/* ps_hold */
	pr_debug("%s: setup qsc6085_ps_hold\n", __func__);
	info->ps_hold_gpio = pdata->gpio_pshold;
	snprintf(info->ps_hold_name, GPIO_MAX_NAME, "%s-%s",
		info->rdev.name, "ps_hold");
	err = gpio_request(info->ps_hold_gpio, info->ps_hold_name);
	if (err) {
		pr_err("%s: err_ps_hold\n", __func__);
		goto err_pshold;
	}
	gpio_export(info->ps_hold_gpio, false);

	/* reset */
	pr_debug("%s: setup qsc6085_reset\n", __func__);
	info->reset_gpio = pdata->gpio_reset_out;
	snprintf(info->reset_name, GPIO_MAX_NAME, "%s-%s",
		info->rdev.name, "reset");
	err = gpio_request(info->reset_gpio, info->reset_name);
	if (err) {
		pr_err("%s: err requesting reset gpio\n", __func__);
		goto err_reset;
	}
	gpio_direction_input(info->reset_gpio);
	reset_irq = gpio_to_irq(info->reset_gpio);
	err = request_threaded_irq(reset_irq, qsc6085_reset_isr,
		qsc6085_reset_fn, IRQ_TYPE_EDGE_FALLING, info->reset_name,
		info);
	if (err) {
		pr_err("%s: request irq (%d) %s failed\n",
			__func__, reset_irq, info->reset_name);
		gpio_free(info->reset_gpio);
		goto err_reset;
	}
	gpio_export(info->reset_gpio, false);

	/* force_flash */
	pr_debug("%s: setup qsc6085_force_flash\n", __func__);
	info->flash_gpio = pdata->gpio_flash_enable;
	snprintf(info->flash_name, GPIO_MAX_NAME, "%s-%s",
		info->rdev.name, "flash");
	err = gpio_request(info->flash_gpio, info->flash_name);
	if (err) {
		pr_err("%s: error requesting flash gpio\n", __func__);
		goto err_flash;
	}
	gpio_export(info->flash_gpio, false);

	/* power_enable */
	pr_debug("%s: setup qsc6085_power_en\n", __func__);
	info->power_gpio = pdata->gpio_power;
	snprintf(info->flash_name, GPIO_MAX_NAME, "%s-%s",
		info->rdev.name, "power_enable");
	err = gpio_request(info->power_gpio, info->power_name);
	if (err) {
		pr_err("%s: error requesting power gpio\n", __func__);
		goto err_power;
	}
	gpio_export(info->power_gpio, false);

	/* try to determine the boot up mode of the device */
	info->boot_flash = !!gpio_get_value(info->flash_gpio);
	if (gpio_get_value(info->reset_gpio)) {
		if (info->boot_flash)
			info->status = QSC6085_STATUS_FLASH;
		else
			info->status = QSC6085_STATUS_NORMAL;
	} else
		info->status = QSC6085_STATUS_OFF;

	pr_debug("%s: initial status = %s\n", __func__,
		qsc6085_status_str[info->status]);

	err = radio_dev_register(&info->rdev);
	if (err) {
		pr_err("%s: failed to register radio device\n", __func__);
		goto err_dev_register;
	}

	return 0;

err_dev_register:
	gpio_free(info->power_gpio);
err_power:
	gpio_free(info->flash_gpio);
err_flash:
	free_irq(reset_irq, info);
	gpio_free(info->reset_gpio);
err_reset:
	gpio_free(info->ps_hold_gpio);
err_pshold:
	platform_set_drvdata(pdev, NULL);
	kfree(info);
err_exit:
	return err;
}

static void __devexit qsc6085_shutdown(struct platform_device *pdev)
{
	struct qsc6085_info *info = platform_get_drvdata(pdev);
	pr_debug("%s: %s - %s\n", __func__,
		dev_name(&pdev->dev), info->rdev.name);
	(void) qsc6085_do_powerdown(info);
}

static int __devexit qsc6085_remove(struct platform_device *pdev)
{
	struct qsc6085_info *info = platform_get_drvdata(pdev);

	pr_debug("%s: %s - %s\n", __func__,
		dev_name(&pdev->dev), info->rdev.name);

	radio_dev_unregister(&info->rdev);

	/* flash */
	gpio_free(info->flash_gpio);

	/* reset */
	free_irq(gpio_to_irq(info->reset_gpio), info);
	gpio_free(info->reset_gpio);

	/* disable */
	gpio_free(info->ps_hold_gpio);

	/* power */
	gpio_free(info->power_gpio);

	platform_set_drvdata(pdev, NULL);
	kfree(info);

	return 0;
}

static struct platform_driver qsc6085_driver = {
	.probe = qsc6085_probe,
	.remove = __devexit_p(qsc6085_remove),
	.shutdown = __devexit_p(qsc6085_shutdown),
	.driver = {
		.name = QSC6085_CTRL_MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init qsc6085_init(void)
{

	pr_debug("%s: initializing %s\n", __func__, qsc6085_driver.driver.name);
	return platform_driver_register(&qsc6085_driver);
}

static void __exit qsc6085_exit(void)
{
	pr_debug("%s: exiting %s\n", __func__, qsc6085_driver.driver.name);
	return platform_driver_unregister(&qsc6085_driver);
}

module_init(qsc6085_init);
module_exit(qsc6085_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("QSC6085 Modem Control");
MODULE_LICENSE("GPL");

