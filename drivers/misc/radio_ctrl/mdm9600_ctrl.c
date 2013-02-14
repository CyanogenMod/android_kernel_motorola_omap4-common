/*
     Copyright (C) 2010 Motorola, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
     02111-1307  USA
*/
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/kobject.h>
#include <linux/radio_ctrl/mdm9600_ctrl.h>
#include <linux/radio_ctrl/radio_class.h>

#define LOOP_DELAY_TIME_MS          100

#define AP_STATUS_BP_PANIC_ACK		0x00
#define AP_STATUS_USB_BYPASS		0x01
#define AP_STATUS_NO_BYPASS		0x02
#define AP_STATUS_BP_SHUTDOWN_REQ	0x03

#define BP_STATUS_PANIC			0x00
#define BP_STATUS_PANIC_BUSY_WAIT	0x01
#define BP_STATUS_QC_DLOAD		0x02
#define BP_STATUS_RAM_DOWNLOADER	0x03
#define BP_STATUS_PHONE_CODE_AWAKE	0x04
#define BP_STATUS_PHONE_CODE_ASLEEP	0x05
#define BP_STATUS_SHUTDOWN_ACK		0x06
#define BP_STATUS_UNDEFINED		0x07

static const char *mdmctrl = "mdm9600_ctrl";

static const char *bp_status[8] = {
	[BP_STATUS_PANIC]		= MDM9600_STATUS_PANIC_NAME,
	[BP_STATUS_PANIC_BUSY_WAIT]	= MDM9600_STATUS_PANIC_BUSY_WAIT_NAME,
	[BP_STATUS_QC_DLOAD]		= MDM9600_STATUS_QC_DLOAD_NAME,
	[BP_STATUS_RAM_DOWNLOADER]	= MDM9600_STATUS_RAM_DOWNLOADER_NAME,
	[BP_STATUS_PHONE_CODE_AWAKE]	= MDM9600_STATUS_PHONE_CODE_AWAKE_NAME,
	[BP_STATUS_PHONE_CODE_ASLEEP]	= MDM9600_STATUS_PHONE_CODE_ASLEEP_NAME,
	[BP_STATUS_SHUTDOWN_ACK]	= MDM9600_STATUS_SHUTDOWN_ACK_NAME,
	[BP_STATUS_UNDEFINED]		= MDM9600_STATUS_UNDEFINED_NAME,
};

static const char *bp_power_state[2] = {
	"off",
	"on",
};

#define BP_STATUS_MAX_LENGTH	32
#define BP_COMMAND_MAX_LENGTH       32

/* structure to keep track of gpio, irq, and irq enabled info */
struct gpio_info {
	int irq;
	struct work_struct work;
};

struct mdm_ctrl_info {
	struct mdm9600_ctrl_platform_data *pdata;
	struct gpio_info gpios[MDM9600_CTRL_NUM_GPIOS];
};

static struct mdm_ctrl_info mdm_ctrl;

static DEFINE_MUTEX(mdm_ctrl_info_lock);

struct workqueue_struct *mdm9600_wq;

static struct radio_dev radio_cdev;

static unsigned int bp_status_idx = BP_STATUS_UNDEFINED;
static unsigned int bp_power_idx;

static unsigned int bp_bootmode = RADIO_BOOTMODE_NORMAL;

static void __devexit mdm_ctrl_shutdown(struct platform_device *pdev);
static void mdm_ctrl_powerup(void);
static int mdm_process_reboot(struct notifier_block *this,
				unsigned long event, void *ptr);

static struct notifier_block mdm9600_reboot_notifier = {
	.notifier_call = mdm_process_reboot,
	.priority = 1,
};

static const char *bp_status_string(unsigned int stat)
{
	if (stat < ARRAY_SIZE(bp_status))
		return bp_status[stat];
	else
		return "status out of range";
}

static const char *bp_power_state_string(unsigned int stat)
{
	if (stat < ARRAY_SIZE(bp_power_state))
		return bp_power_state[stat];
	else
		return "status out of range";
}

static ssize_t mdm_status_show(struct radio_dev *dev, char *buff)
{
	ssize_t status = 0;
	status = snprintf(buff, BP_STATUS_MAX_LENGTH, "%s\n",
			  bp_status_string(bp_status_idx));

	return status;
}

static ssize_t mdm_power_show(struct radio_dev *rdev, char *buff)
{
	ssize_t status = 0;
	status = snprintf(buff, BP_STATUS_MAX_LENGTH, "%s\n",
			  bp_power_state_string(bp_power_idx));

	return status;
}

static ssize_t mdm_user_command(struct radio_dev *rdev, char *post_strip)
{

	pr_info("%s: user command = %s\n", mdmctrl, post_strip);

	if (strcmp(post_strip, "shutdown") == 0)
		mdm_ctrl_shutdown(NULL);
	else if (strcmp(post_strip, "powerup") == 0)
		mdm_ctrl_powerup();
	else if (strcmp(post_strip, "bootmode_normal") == 0)
		bp_bootmode = RADIO_BOOTMODE_NORMAL;
	else if (strcmp(post_strip, "bootmode_flash") == 0)
		bp_bootmode = RADIO_BOOTMODE_FLASH;
	else
		return -EINVAL;

	return 0;
}

static unsigned int mdm_gpio_get_value(struct mdm9600_ctrl_gpio gpio)
{
	return gpio_get_value(gpio.number);
}

static void mdm_gpio_set_value(struct mdm9600_ctrl_gpio gpio,
	unsigned int value)
{
	gpio_direction_output(gpio.number, value);
}

static void mdm_gpio_free(struct mdm9600_ctrl_gpio *gpio)
{
	if (gpio->allocated)
		gpio_free(gpio->number);
	gpio->allocated = 0;
}

static int mdm_gpio_setup(struct mdm9600_ctrl_gpio *gpio)
{
	if (gpio_request(gpio->number, gpio->name))  {
		pr_err("failed to aquire gpio %s\n", gpio->name);
		return -1;
	}
	gpio->allocated = 1;
	gpio_export(gpio->number, false);
	if (gpio->direction == MDM9600_GPIO_DIRECTION_IN)
		gpio_direction_input(gpio->number);
	else if (gpio->direction == MDM9600_GPIO_DIRECTION_OUT &&
		 gpio->default_value <= 1)
		gpio_direction_output(gpio->number, gpio->default_value);

	return 0;
}

static unsigned int get_bp_status(void)
{
	unsigned int status = BP_STATUS_UNDEFINED;
	unsigned int bp_status[3] = {0};

	mutex_lock(&mdm_ctrl_info_lock);
	if (mdm_ctrl.pdata) {
		bp_status[0] = mdm_gpio_get_value(
			mdm_ctrl.pdata->gpios[MDM9600_CTRL_GPIO_STATUS_B0]);
		bp_status[1] = mdm_gpio_get_value(
			mdm_ctrl.pdata->gpios[MDM9600_CTRL_GPIO_STATUS_B1]);
		bp_status[2] = mdm_gpio_get_value(
			mdm_ctrl.pdata->gpios[MDM9600_CTRL_GPIO_STATUS_B2]);
	}
	mutex_unlock(&mdm_ctrl_info_lock);

	status = ((bp_status[2] & 0x1) << 2) |
		 ((bp_status[1] & 0x1) << 1) |
		  (bp_status[0] & 0x1);

	return status;
}

static unsigned int get_bp_power_status(void)
{
	unsigned int status = 0;

	mutex_lock(&mdm_ctrl_info_lock);
	if (mdm_ctrl.pdata) {
		status = mdm_gpio_get_value(
			mdm_ctrl.pdata->gpios[MDM9600_CTRL_GPIO_BP_RESOUT]);
	}

	mutex_unlock(&mdm_ctrl_info_lock);

	return status & 0x1;
}

static unsigned int get_ap_status(void)
{
	unsigned int status;
	unsigned int ap_status[2] =  {0};

	mutex_lock(&mdm_ctrl_info_lock);
	if (mdm_ctrl.pdata) {
		ap_status[0] = mdm_gpio_get_value(
			mdm_ctrl.pdata->gpios[MDM9600_CTRL_GPIO_STATUS_A0]);
		ap_status[1] = mdm_gpio_get_value(
			mdm_ctrl.pdata->gpios[MDM9600_CTRL_GPIO_STATUS_A1]);
	}
	mutex_unlock(&mdm_ctrl_info_lock);

	status = ((ap_status[1] & 0x1) << 1) |
		  (ap_status[0] & 0x1);

	return status;
}

static void set_ap_status(unsigned int status)
{
	mutex_lock(&mdm_ctrl_info_lock);
	if (mdm_ctrl.pdata) {
		mdm_gpio_set_value(
			mdm_ctrl.pdata->gpios[MDM9600_CTRL_GPIO_STATUS_A0],
			(status & 0x1));
		mdm_gpio_set_value(
			mdm_ctrl.pdata->gpios[MDM9600_CTRL_GPIO_STATUS_A1],
			(status >> 1) & 0x1);
	}
	mutex_unlock(&mdm_ctrl_info_lock);
}

static void set_bp_pwron(int on)
{
	mutex_lock(&mdm_ctrl_info_lock);
	if ((mdm_ctrl.pdata) && ((on == 1) || (on == 0))) {
		mdm_gpio_set_value(
			mdm_ctrl.pdata->gpios[MDM9600_CTRL_GPIO_BP_PWRON],
			on);
	}
	mutex_unlock(&mdm_ctrl_info_lock);
}

static void set_ap_reset_bp(int on)
{
	mutex_lock(&mdm_ctrl_info_lock);
	if ((mdm_ctrl.pdata) && ((on == 1) || (on == 0))) {
		mdm_gpio_set_value(
			mdm_ctrl.pdata->gpios[MDM9600_CTRL_GPIO_AP_RESET_BP],
			on);
	}
	mutex_unlock(&mdm_ctrl_info_lock);
}

static void update_bp_status(void)
{
	int bp_status_prev_idx = bp_status_idx;
	int bp_power_prev_idx = bp_power_idx;

	bp_status_idx = get_bp_status();
	bp_power_idx = get_bp_power_status();

	if (bp_power_idx == bp_power_prev_idx)
		pr_debug("%s: modem status: %s -> %s [power %s]\n", mdmctrl,
			bp_status_string(bp_status_prev_idx),
			bp_status_string(bp_status_idx),
			bp_power_state_string(bp_power_idx));
	else
		pr_info("%s: modem status: %s -> %s [power %s]\n", mdmctrl,
			bp_status_string(bp_status_prev_idx),
			bp_status_string(bp_status_idx),
			bp_power_state_string(bp_power_idx));

	kobject_uevent(&radio_cdev.dev->kobj, KOBJ_CHANGE);
}

static void mdm_ctrl_set_bootmode(int mode)
{
	mutex_lock(&mdm_ctrl_info_lock);
	if (mdm_ctrl.pdata && ((mode == 0) || (mode == 1))) {
		gpio_request(mdm_ctrl.pdata->cmd_gpios.cmd1,
				"BP Command 1");
		gpio_direction_output(mdm_ctrl.pdata->cmd_gpios.cmd1,
					mode);
		gpio_request(mdm_ctrl.pdata->cmd_gpios.cmd2,
				"BP Command 2");
		gpio_direction_output(mdm_ctrl.pdata->cmd_gpios.cmd2,
					mode);
	}
	mutex_unlock(&mdm_ctrl_info_lock);
}

static void mdm_ctrl_clr_bootmode(void)
{
	mutex_lock(&mdm_ctrl_info_lock);
	if (mdm_ctrl.pdata) {
		gpio_free(mdm_ctrl.pdata->cmd_gpios.cmd1);
		gpio_free(mdm_ctrl.pdata->cmd_gpios.cmd2);
	}
	mutex_unlock(&mdm_ctrl_info_lock);
}

static void mdm_ctrl_powerup(void)
{
	unsigned int bp_status, i;

	if (get_bp_power_status() == 1) {
		bp_status = get_bp_status();
		pr_info("%s: modem already powered up status %s [0x%x].\n",
			mdmctrl, bp_status_string(bp_status), bp_status);
		return;
	}

	mdm_ctrl_set_bootmode(bp_bootmode);

	set_ap_status(AP_STATUS_NO_BYPASS);
	pr_debug("%s: ap_status set to %d\n", mdmctrl, get_ap_status());
	msleep(100);
	set_ap_reset_bp(0);
	msleep(100);
	/* Toggle the power, delaying to allow modem to respond */
	set_bp_pwron(1);
	msleep(100);
	set_bp_pwron(0);

	/* verify power up by sampling reset */
	for (i = 0; i < 10; i++) {
		bp_status = get_bp_power_status();
		pr_debug("%s: reset value = %d\n", __func__, bp_status);
		if (bp_status) {
			pr_info("%s: powered Up mdm9600\n", __func__);
			break;
		}
		msleep(400);
	}

	if (!bp_status)
		pr_err("%s: FAILED to start mdm9600\n", __func__);
	else {
		/* If the modem powered up, we need to give the modem
		 * bootloader some time to read the command pins before
		 * we stop driving them
		 */
		/* HACK: workaround MDM issue where it cannot handle
		 * QMI client interface allocations early during its
		 * powerup.  Block RIL here for 15 seconds.
		 */
		if (bp_bootmode == RADIO_BOOTMODE_NORMAL)
			msleep(30000);
		else
			msleep(5000);
	}

	mdm_ctrl_clr_bootmode();

	/* now let user handles bp status change through uevent */
}

static void irq_worker(struct work_struct *work)
{
	struct gpio_info *gpio = container_of(work, struct gpio_info, work);
	update_bp_status();
	enable_irq(gpio->irq);
}

static irqreturn_t irq_handler(int irq, void *data)
{
	struct gpio_info *gpio = (struct gpio_info *) data;

	disable_irq_nosync(irq);
	queue_work(mdm9600_wq, &gpio->work);

	return IRQ_HANDLED;
}

static int mdm_gpio_setup_internal(struct mdm9600_ctrl_platform_data *pdata)
{
	int i;
	int rv = 0;
	struct gpio_info *gpio_data = NULL;

	mutex_lock(&mdm_ctrl_info_lock);
	memset(&mdm_ctrl, 0, sizeof(mdm_ctrl));

	mdm_ctrl.pdata = pdata;

	for (i = 0; i < MDM9600_CTRL_NUM_GPIOS; i++) {
		gpio_data = &mdm_ctrl.gpios[i];
		if (pdata->gpios[i].direction == MDM9600_GPIO_DIRECTION_IN) {
			INIT_WORK(&gpio_data->work, irq_worker);
			gpio_data->irq = gpio_to_irq(pdata->gpios[i].number);
			rv = request_irq(gpio_data->irq, irq_handler,
				IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING,
					pdata->gpios[i].name, gpio_data);
			if (rv < 0) {
				pr_err(
				"%s: Cannot request IRQ (%d) from kernel!\n",
				mdmctrl, gpio_data->irq);
			} else {
				enable_irq_wake(gpio_data->irq);
			}
		}
	}

	mutex_unlock(&mdm_ctrl_info_lock);
	return rv;
}

static void mdm_gpio_cleanup_internal(void)
{
	int i;
	struct gpio_info *gpio_data = NULL;

	mutex_lock(&mdm_ctrl_info_lock);

	for (i = 0; i < MDM9600_CTRL_NUM_GPIOS; i++) {
		gpio_data = &mdm_ctrl.gpios[i];

		if (gpio_data->irq) {
			disable_irq_wake(gpio_data->irq);
			free_irq(gpio_data->irq, gpio_data);
		}
	}
	memset(&mdm_ctrl, 0, sizeof(mdm_ctrl));
	mutex_unlock(&mdm_ctrl_info_lock);
}

static struct radio_dev radio_cdev = {
	.power_status = mdm_power_show,
	.status = mdm_status_show,
	.command = mdm_user_command,
};

static int __devinit mdm9600_ctrl_probe(struct platform_device *pdev)
{
	int i;
	struct mdm9600_ctrl_platform_data *pdata = pdev->dev.platform_data;
	radio_cdev.name = pdata->name;

	dev_info(&pdev->dev, "mdm_ctrl_probe\n");
	pr_debug("%s:%s radio_cdev = %p\n", __func__, pdata->name, &radio_cdev);

	for (i = 0; i < MDM9600_CTRL_NUM_GPIOS; i++) {
		if (mdm_gpio_setup(&pdata->gpios[i])) {
			dev_err(&pdev->dev, "failed to aquire gpio %d\n",
				pdata->gpios[i].number);
			goto probe_cleanup;
		}
	}

	mdm9600_wq = create_singlethread_workqueue("mdm9600_ctrl_wq");
	if (!mdm9600_wq) {
		dev_err(&pdev->dev, "Cannot create work queue.\n");
		goto probe_cleanup;
	}

	if (mdm_gpio_setup_internal(pdata) < 0) {
		dev_err(&pdev->dev, "Failed to setup bp  status irq\n");
		goto err_setup;
	}

	if (radio_dev_register(&radio_cdev)) {
		pr_err("%s: failed to register mdm_ctr device\n", __func__);
		goto err_setup;
	}

	gpio_request(pdata->cmd_gpios.cmd1, "BP Command 1");
	gpio_request(pdata->cmd_gpios.cmd2, "BP Command 2");
	mdm_ctrl_clr_bootmode();

	update_bp_status();
	register_reboot_notifier(&mdm9600_reboot_notifier);

	return 0;

err_setup:
	mdm_gpio_cleanup_internal();
	destroy_workqueue(mdm9600_wq);

probe_cleanup:
	for (i = 0; i < MDM9600_CTRL_NUM_GPIOS; i++)
		mdm_gpio_free(&pdata->gpios[i]);

	return -1;
}

static int __devexit mdm_ctrl_remove(struct platform_device *pdev)
{
	int i;
	struct mdm9600_ctrl_platform_data *pdata = pdev->dev.platform_data;

	dev_info(&pdev->dev, "cleanup\n");

	radio_dev_unregister(&radio_cdev);

	mdm_gpio_cleanup_internal();

	if (mdm9600_wq)
		destroy_workqueue(mdm9600_wq);

	for (i = 0; i < MDM9600_CTRL_NUM_GPIOS; i++)
		mdm_gpio_free(&pdata->gpios[i]);

	return 0;
}

static unsigned int __devexit bp_shutdown_wait(unsigned int delay_sec)
{
	unsigned int i, loop_count;
	unsigned int bp_pd_ack = 0;
	unsigned int pd_failure = 1;

	loop_count = (delay_sec * 1000) / LOOP_DELAY_TIME_MS;

	for (i = 0; i < loop_count; i++) {
		if (!bp_pd_ack && get_bp_status() == BP_STATUS_SHUTDOWN_ACK) {
			pr_info("%s: Modem ack'd shutdown req.\n", mdmctrl);
			bp_pd_ack = 1;
			set_bp_pwron(0);
		}

		if (get_bp_power_status() == 0) {
			pr_info("%s: Modem powered off.\n", mdmctrl);
			pd_failure = 0;
			break;
		}
		msleep(LOOP_DELAY_TIME_MS);
	}
	set_bp_pwron(0);
	return pd_failure;
}

static void __devexit mdm_ctrl_shutdown(struct platform_device *pdev)
{
	unsigned int pd_failure;
	unsigned int bp_status;

	if (get_bp_power_status() == 0) {
		pr_info("%s: modem already powered down.\n", mdmctrl);
		return;
	}

	bp_status = get_bp_status();
	pr_info("%s: Shutting down initial status %s [0x%x]\n",
		mdmctrl, bp_status_string(bp_status), bp_status);

	set_ap_status(AP_STATUS_BP_SHUTDOWN_REQ);

	/* Allow modem to process status */
	msleep(100);
	pr_debug("%s: ap_status set to %d\n", mdmctrl, get_ap_status());

	/* Assert PWRON to tell modem to shutdown and leave pin asserted */
	/* until acknowledged or wait times out */
	set_bp_pwron(1);
	msleep(100);

	/* This should be enough to power down the modem */
	/* if this doesn't work, reset the modem and try */
	/* one more time, ultimately the modem will be   */
	/* hard powered off */
	pd_failure = bp_shutdown_wait(5);
	if (pd_failure) {
		pr_info("%s: Resetting unresponsive modem.\n", mdmctrl);
		set_ap_reset_bp(1);
		pd_failure = bp_shutdown_wait(7);
	}

	if (pd_failure)
		pr_err("%s: Modem failed to power down.\n", mdmctrl);
}

static int mdm_process_reboot(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	pr_info("%s\n", __func__);
	mdm_ctrl_shutdown(NULL);
	return NOTIFY_DONE;
}

static struct platform_driver mdm9x00_ctrl_driver = {
	.probe = mdm9600_ctrl_probe,
	.remove = __devexit_p(mdm_ctrl_remove),
	.shutdown = __devexit_p(mdm_ctrl_shutdown),
	.driver = {
		.name = MDM9600_CTRL_MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init mdm9600_ctrl_init(void)
{
	pr_debug("%s: initializing %s\n",
		__func__, mdm9x00_ctrl_driver.driver.name);
	return platform_driver_register(&mdm9x00_ctrl_driver);
}

static void __exit mdm9600_ctrl_exit(void)
{
	pr_debug("%s: exiting %s\n", __func__, mdm9x00_ctrl_driver.driver.name);
	platform_driver_unregister(&mdm9x00_ctrl_driver);
}

module_init(mdm9600_ctrl_init);
module_exit(mdm9600_ctrl_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("MDM9X00 Control Driver");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");
