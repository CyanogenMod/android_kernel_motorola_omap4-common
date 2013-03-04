/*
 * arch/arm/mach-omap2/board-mapphone-modem.c
 *
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio_mapping.h>
#include <mach/ctrl_module_pad_core_44xx.h>
#include <mach/ctrl_module_pad_wkup_44xx.h>
#include <linux/idr.h>
#include <linux/radio_ctrl/mdm6600_ctrl.h>
#include <linux/radio_ctrl/mdm9600_ctrl.h>
#include <linux/radio_ctrl/wrigley_ctrl.h>
#include <linux/radio_ctrl/qsc6085_ctrl.h>
#include <linux/slab.h>
#include <linux/kdev_t.h>
#include <linux/err.h>
#include <asm/bootinfo.h>

#include <linux/of_fdt.h>
#include <linux/of.h>
#include "dt_path.h"

#define MAPPHONE_BP_QSC6085	0x001E0000
#define MAPPHONE_BP_MDM6600	0x001E0001
#define MAPPHONE_BP_MDM9600	0x001E0002
#define MAPPHONE_BP_STE_M570	0x00240000
#define MAPPHONE_BP_W3GLTE	0x0003000F

#define MAX_MODEMS		2
#define DEFAULT_MODEM_COUNT	1

#define FLAG_TRIG_FALL	5	/* trigger on falling edge */
#define FLAG_TRIG_RISE	6	/* trigger on rising edge */
#define FLAG_ACTIVE_LOW	7	/* sysfs value has active low */
#define GPIO_TRIGGER_MASK	(BIT(FLAG_TRIG_FALL) | BIT(FLAG_TRIG_RISE))
#define PDESC_ID_SHIFT	16	/* add new flags before this one */
#define GPIO_FLAGS_MASK		((1 << PDESC_ID_SHIFT) - 1)

static int mapphone_bpwake_gpio = -1;
static struct platform_device mapphone_bpwake_device = {
	.name		= "mapphone_bpwake",
	.id		 = -1,
	.num_resources	= 0,
	.dev.platform_data = &mapphone_bpwake_gpio,
};

struct ste_gpio {
	u32 pin_nr;
	char device_name[20];
	u8 value;
	u8 direction;
	unsigned long flags;
	u8 device_found;
};

static const struct {
	const char *name;
	unsigned long flags;
} trigger_types[] = {
	{ "none",    0 },
	{ "falling", BIT(FLAG_TRIG_FALL) },
	{ "rising",  BIT(FLAG_TRIG_RISE) },
	{ "both",    BIT(FLAG_TRIG_FALL) | BIT(FLAG_TRIG_RISE) },
};

static struct class ste_gpio_class = {
	.name =		"ste",
	.owner =	THIS_MODULE,
};

struct poll_desc {
	struct work_struct	work;
	struct sysfs_dirent	*value_sd;
};

static struct idr pdesc_idr;

static DEFINE_MUTEX(ste_lock);

static struct ste_gpio ste_gpio[] = {
	{
		.pin_nr = 8,
		.device_name = "ipc_i_bp_resout2",
		.value = 0,
		.direction = 1,
		.flags = 0,
	},
	{
		.pin_nr = 49,
		.device_name = "ipc_o_ap_reset_bp",
		.value = 0,
		.direction = 0,
		.flags = 0,
	},
	{
		.pin_nr = 52,
		.device_name = "ipc_o_bp_service",
		.value = 0,
		.direction = 0,
		.flags = 0,
	},
	{
		.pin_nr = 54,
		.device_name = "ipc_o_bp_onswa",
		.value = 0,
		.direction = 0,
		.flags = 0,
	},
	{
		.pin_nr = 95,
		.device_name = "ipc_i_bp_pwrrstin",
		.value = 0,
		.direction = 1,
		.flags = 0,
	},
	{
		.pin_nr = 190,
		.device_name = "ipc_o_bp_sar_voice",
		.value = 0,
		.direction = 0,
		.flags = 0,
	},
	{
		.pin_nr = 139,
		.device_name = "ipc_o_bp_sar_data",
		.value = 0,
		.direction = 0,
		.flags = 0,
	},
	{
		.pin_nr = 102,
		.device_name = "ipc_o_bp_onswc",
		.value = 0,
		.direction = 0,
		.flags = 0,
	},
};

struct mapphone_modem {
	int n_modems;
	int type[MAX_MODEMS];
};

static struct mapphone_modem mapphone_modem;

static struct mdm6600_ctrl_platform_data mdm6600_ctrl_platform_data = {
	.name = "mdm6600",
	.gpios[MDM6600_CTRL_GPIO_AP_STATUS_0] = {
		0, MDM6600_GPIO_DIRECTION_OUT, MDM6600_GPIO_INVALID, 0,
		"mdm_ap_status0"},
	.gpios[MDM6600_CTRL_GPIO_AP_STATUS_1] = {
		0, MDM6600_GPIO_DIRECTION_OUT, MDM6600_GPIO_INVALID, 0,
		"mdm_ap_status1"},
	.gpios[MDM6600_CTRL_GPIO_AP_STATUS_2] = {
		0, MDM6600_GPIO_DIRECTION_OUT, MDM6600_GPIO_INVALID, 0,
		"mdm_ap_status2"},
	.gpios[MDM6600_CTRL_GPIO_BP_STATUS_0] = {
		0, MDM6600_GPIO_DIRECTION_IN, 0, 0, "mdm_bp_status0"},
	.gpios[MDM6600_CTRL_GPIO_BP_STATUS_1] = {
		0, MDM6600_GPIO_DIRECTION_IN, 0, 0, "mdm_bp_status1"},
	.gpios[MDM6600_CTRL_GPIO_BP_STATUS_2] = {
		0, MDM6600_GPIO_DIRECTION_IN, 0, 0, "mdm_bp_status2"},
	.gpios[MDM6600_CTRL_GPIO_BP_RESOUT]   = {
		0, MDM6600_GPIO_DIRECTION_IN, 0, 0, "mdm_bp_resout"},
	.gpios[MDM6600_CTRL_GPIO_BP_RESIN]    = {
		0, MDM6600_GPIO_DIRECTION_OUT, 0, 0, "mdm_bp_resin"},
	.gpios[MDM6600_CTRL_GPIO_BP_PWRON]    = {
		0, MDM6600_GPIO_DIRECTION_OUT, 0, 0, "mdm_bp_pwr_on"}
};

static struct platform_device mdm6600_ctrl_platform_device = {
	.name = MDM6600_CTRL_MODULE_NAME,
	.id = -1,
	.dev = {
		.platform_data = &mdm6600_ctrl_platform_data,
	},
};

static struct mdm9600_ctrl_platform_data mdm9600_ctrl_platform_data = {
	.name = "mdm9600",
	.gpios[MDM9600_CTRL_GPIO_STATUS_A0] = {
		0, MDM9600_GPIO_DIRECTION_OUT, MDM9600_GPIO_INVALID, 0,
		"mdm_ap_status0"},
	.gpios[MDM9600_CTRL_GPIO_STATUS_A1] = {
		0, MDM9600_GPIO_DIRECTION_OUT, MDM9600_GPIO_INVALID, 0,
		"mdm_ap_status1"},
	.gpios[MDM9600_CTRL_GPIO_STATUS_B0] = {
		0, MDM9600_GPIO_DIRECTION_IN, 0, 0, "mdm_bp_status0"},
	.gpios[MDM9600_CTRL_GPIO_STATUS_B1] = {
		0, MDM9600_GPIO_DIRECTION_IN, 0, 0, "mdm_bp_status1"},
	.gpios[MDM9600_CTRL_GPIO_STATUS_B2] = {
		0, MDM9600_GPIO_DIRECTION_IN, 0, 0, "mdm_bp_status2"},
	.gpios[MDM9600_CTRL_GPIO_BP_RESOUT]   = {
		0, MDM9600_GPIO_DIRECTION_IN, 0, 0, "mdm_bp_resout"},
	.gpios[MDM9600_CTRL_GPIO_BP_PWRON]    = {
		0, MDM9600_GPIO_DIRECTION_OUT, 0, 0, "mdm_bp_pwr_on"},
	.gpios[MDM9600_CTRL_GPIO_AP_RESET_BP]    = {
		0, MDM9600_GPIO_DIRECTION_OUT, 0, 0, "mdm_ap_reset_bp"}
};

static struct platform_device mdm9600_ctrl_platform_device = {
	.name = MDM9600_CTRL_MODULE_NAME,
	.id = -1,
	.dev = {
		.platform_data = &mdm9600_ctrl_platform_data,
	},
};

static struct wrigley_ctrl_platform_data w3g_lte_ctrl_platform_data = {
	.name = "w3g_lte",
};

static struct platform_device w3g_lte_ctrl_platform_device = {
	.name = WRIGLEY_CTRL_MODULE_NAME,
	.id = -1,
	.dev = {
		.platform_data = &w3g_lte_ctrl_platform_data,
	},
};

static struct qsc6085_ctrl_platform_data qsc6085_ctrl_platform_data = {
	.name = "qsc6085",
};

static struct platform_device qsc6085_ctrl_platform_device = {
	.name = QSC6085_CTRL_MODULE_NAME,
	.id = -1,
	.dev = {
		.platform_data = &qsc6085_ctrl_platform_data,
	},
};

static int mapphone_bp_get_count(void)
{
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return DEFAULT_MODEM_COUNT;
	}
	prop = of_get_property(node, DT_PROP_CHOSEN_BP_COUNT, NULL);
	if (prop) {
		pr_err("Update Modem Count = %d\n", *(char *)prop);
		return (int)(*(char *)prop);
	}
	of_node_put(node);

	return DEFAULT_MODEM_COUNT;
}


static int mapphone_bp_get_type(u8 modem)
{
	int ret = 0;
	struct device_node *node;
	const void *prop;
	int size;
	char dt_path_modem[sizeof(DT_PATH_MODEM) + 4];

	sprintf(dt_path_modem, "%s%d", DT_PATH_MODEM, modem);
	node = of_find_node_by_path(dt_path_modem);
	if (node) {
		prop = of_get_property(node, \
			DT_PROP_MODEM_TYPE, &size);
		if (prop && size)
			ret = *(u32 *)prop;
		of_node_put(node);
	}
	return ret;
}

static int mapphone_w3glte_mdm_ctrl_init(void)
{
	u32 reg;
	int gpio;

	reg = omap_readl(OMAP4_CTRL_MODULE_PAD_WKUP|
		OMAP4_CTRL_MODULE_PAD_WKUP_CONTROL_USIMIO);
	omap_writel((reg|OMAP4_USIM_PWRDNZ_MASK)&
		~(OMAP4_PAD_USIM_CLK_LOW_MASK|OMAP4_PAD_USIM_RST_LOW_MASK),
		OMAP4_CTRL_MODULE_PAD_WKUP|
		OMAP4_CTRL_MODULE_PAD_WKUP_CONTROL_USIMIO);

	reg = omap_readl(OMAP4_CTRL_MODULE_PAD_CORE|
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE);
	omap_writel((reg|OMAP4_USIM_PBIASLITE_PWRDNZ_MASK)&
		~(OMAP4_USIM_PBIASLITE_HIZ_MODE_MASK|
		OMAP4_USIM_PBIASLITE_SUPPLY_HI_OUT_MASK|
		OMAP4_USIM_PBIASLITE_VMODE_ERROR_MASK|
		OMAP4_USIM_PBIASLITE_VMODE_MASK)
		, OMAP4_CTRL_MODULE_PAD_CORE|
		OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_PBIASLITE);

	gpio = get_gpio_by_name("lte_w_disable_b");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get lte_w_disable_b "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	w3g_lte_ctrl_platform_data.gpio_disable = gpio;

	gpio = get_gpio_by_name("lte_force_flash");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get lte_force_flash "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	w3g_lte_ctrl_platform_data.gpio_force_flash = gpio;

	gpio = get_gpio_by_name("lte_power_en");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get lte_power_en "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	w3g_lte_ctrl_platform_data.gpio_power_enable = gpio;

	gpio = get_gpio_by_name("lte_reset_mcu");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get lte_reset_mcu "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	w3g_lte_ctrl_platform_data.gpio_reset = gpio;

	platform_device_register(&w3g_lte_ctrl_platform_device);
	return 0;

}

static int mapphone_mdm6600_mdm_ctrl_init(void)
{
	int gpio;

	gpio = get_gpio_by_name("ipc_o_bp_pwron");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_bp_pwron "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.gpios[MDM6600_CTRL_GPIO_BP_PWRON].number =
		gpio;

	gpio = get_gpio_by_name("ipc_i_bp_resout");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_i_bp_resout "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.gpios[MDM6600_CTRL_GPIO_BP_RESOUT].number =
		gpio;

	gpio = get_gpio_by_name("ipc_i_bp_ready");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_i_bp_ready "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.gpios[MDM6600_CTRL_GPIO_BP_STATUS_0].number =
		gpio;

	gpio = get_gpio_by_name("ipc_i_bp_ready2");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_i_bp_ready2 "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.gpios[MDM6600_CTRL_GPIO_BP_STATUS_1].number =
		 gpio;

	gpio = get_gpio_by_name("ipc_i_bp_ready3");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_i_bp_ready3 "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.gpios[MDM6600_CTRL_GPIO_BP_STATUS_2].number =
		 gpio;

	gpio = get_gpio_by_name("ipc_o_ap_ready");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_ap_ready "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.gpios[MDM6600_CTRL_GPIO_AP_STATUS_0].number =
		 gpio;

	gpio = get_gpio_by_name("ipc_o_ap_ready2");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_ap_ready2 "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.gpios[MDM6600_CTRL_GPIO_AP_STATUS_1].number =
		 gpio;

	gpio = get_gpio_by_name("ipc_o_ap_ready3");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_ap_ready3 "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.gpios[MDM6600_CTRL_GPIO_AP_STATUS_2].number =
		gpio;

	gpio = get_gpio_by_name("ipc_o_ap_reset_bp");
	if (gpio < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_o_ap_reset_bp "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.gpios[MDM6600_CTRL_GPIO_BP_RESIN].number =
		gpio;

	gpio = get_gpio_by_name("ipc_bpwake_trigger");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_bpwake_trigger "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.cmd_gpios.cmd1 = gpio;

	gpio = get_gpio_by_name("ipc_apwake_trigger");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_apwake_trigger "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm6600_ctrl_platform_data.cmd_gpios.cmd2 = gpio;
	platform_device_register(&mdm6600_ctrl_platform_device);
	return 0;
}

static int mapphone_mdm9600_mdm_ctrl_init(void)
{
	int gpio;

	gpio = get_gpio_by_name("ipc_o_bp_pwron");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_bp_pwron "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm9600_ctrl_platform_data.
		gpios[MDM9600_CTRL_GPIO_BP_PWRON].number = gpio;

	gpio = get_gpio_by_name("ipc_i_bp_resout");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_i_bp_resout "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm9600_ctrl_platform_data.
		gpios[MDM9600_CTRL_GPIO_BP_RESOUT].number = gpio;

	gpio = get_gpio_by_name("ipc_i_bp_ready");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_i_bp_ready "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm9600_ctrl_platform_data.
		gpios[MDM9600_CTRL_GPIO_STATUS_B0].number = gpio;

	gpio = get_gpio_by_name("ipc_i_bp_ready2");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_i_bp_ready2 "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm9600_ctrl_platform_data.
		gpios[MDM9600_CTRL_GPIO_STATUS_B1].number = gpio;

	gpio = get_gpio_by_name("ipc_i_bp_ready3");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_i_bp_ready3 "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm9600_ctrl_platform_data.
		gpios[MDM9600_CTRL_GPIO_STATUS_B2].number = gpio;

	gpio = get_gpio_by_name("ipc_o_ap_ready");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_ap_ready "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm9600_ctrl_platform_data.
		gpios[MDM9600_CTRL_GPIO_STATUS_A0].number = gpio;

	gpio = get_gpio_by_name("ipc_o_ap_ready2");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_ap_ready2 "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm9600_ctrl_platform_data.
		gpios[MDM9600_CTRL_GPIO_STATUS_A1].number = gpio;

	gpio = get_gpio_by_name("ipc_o_ap_reset_bp");
	if (gpio < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_o_ap_reset_bp "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm9600_ctrl_platform_data.
		gpios[MDM9600_CTRL_GPIO_AP_RESET_BP].number = gpio;

	/* Setup ap_status2 and bp_wake_host as the command gpios */
	gpio = get_gpio_by_name("ipc_o_ap_ready3");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_ap_ready3 "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm9600_ctrl_platform_data.cmd_gpios.cmd1 = gpio;

	gpio = get_gpio_by_name("ipc_i_bp_wake_host");
	if (gpio < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_i_bp_wake_host "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	mdm9600_ctrl_platform_data.cmd_gpios.cmd2 = gpio;

	platform_device_register(&mdm9600_ctrl_platform_device);
	return 0;
}

static int mapphone_qsc6085_mdm_ctrl_init(void)
{
	int gpio;

	gpio = get_gpio_by_name("ipc_o_bp_flash_en");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_bp_flash_en "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	qsc6085_ctrl_platform_data.gpio_flash_enable = gpio;

	gpio = get_gpio_by_name("ipc_o_bp_pshold");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_bp_pshold "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	qsc6085_ctrl_platform_data.gpio_pshold = gpio;

	gpio = get_gpio_by_name("ipc_o_bp_pwron");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_o_bp_pwron "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	qsc6085_ctrl_platform_data.gpio_power = gpio;

	gpio = get_gpio_by_name("ipc_i_bp_resout");
	if (gpio < 0) {
		printk(KERN_ERR "%s: can't get ipc_i_bp_resout "
				"from device_tree\n", __func__);
		return -EINVAL;
	}
	qsc6085_ctrl_platform_data.gpio_reset_out = gpio;

	platform_device_register(&qsc6085_ctrl_platform_device);
	return 0;
}

static ssize_t gpio_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ste_gpio *ste_gpio = dev_get_drvdata(dev);
	ssize_t status;

	mutex_lock(&ste_lock);
	ste_gpio->value = !!gpio_get_value_cansleep(ste_gpio->pin_nr);
	status = sprintf(buf, "%d\n", ste_gpio->value);
	mutex_unlock(&ste_lock);
	return status;
}

static ssize_t gpio_value_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct ste_gpio *ste_gpio = dev_get_drvdata(dev);
	ssize_t status;
	long value;

	mutex_lock(&ste_lock);
	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
		ste_gpio->value = !!value;
		gpio_set_value_cansleep(ste_gpio->pin_nr, ste_gpio->value);
		status = size;
	}
	mutex_unlock(&ste_lock);
	return status;
}

static const DEVICE_ATTR(value, 0644, gpio_value_show, gpio_value_store);

static irqreturn_t gpio_sysfs_irq(int irq, void *priv)
{
	struct work_struct *work = priv;

	schedule_work(work);
	return IRQ_HANDLED;
}

static void gpio_notify_sysfs(struct work_struct *work)
{
	struct poll_desc *pdesc;

	pdesc = container_of(work, struct poll_desc, work);
	sysfs_notify_dirent(pdesc->value_sd);
}

static int gpio_setup_irq(struct device *dev, unsigned long gpio_flags)
{
	struct ste_gpio *ste_gpio = dev_get_drvdata(dev);
	struct poll_desc *pdesc;
	unsigned long irq_flags;
	int ret, irq, id;

	if ((ste_gpio->flags & GPIO_TRIGGER_MASK) == gpio_flags)
		return 0;

	irq = gpio_to_irq(ste_gpio->pin_nr);
	if (irq < 0)
		return -EIO;

	id = ste_gpio->flags >> PDESC_ID_SHIFT;
	pdesc = idr_find(&pdesc_idr, id);
	if (pdesc) {
		free_irq(irq, &pdesc->work);
		cancel_work_sync(&pdesc->work);
	}

	ste_gpio->flags &= ~GPIO_TRIGGER_MASK;

	if (!gpio_flags) {
		ret = 0;
		goto free_sd;
	}

	irq_flags = IRQF_SHARED;
	if (test_bit(FLAG_TRIG_FALL, &gpio_flags))
		irq_flags |= test_bit(FLAG_ACTIVE_LOW, &ste_gpio->flags) ?
			IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
	if (test_bit(FLAG_TRIG_RISE, &gpio_flags))
		irq_flags |= test_bit(FLAG_ACTIVE_LOW, &ste_gpio->flags) ?
			IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

	if (!pdesc) {
		pdesc = kmalloc(sizeof(*pdesc), GFP_KERNEL);
		if (!pdesc) {
			ret = -ENOMEM;
			goto err_out;
		}

		do {
			ret = -ENOMEM;
			if (idr_pre_get(&pdesc_idr, GFP_KERNEL))
				ret = idr_get_new_above(&pdesc_idr,
						pdesc, 1, &id);
		} while (ret == -EAGAIN);

		if (ret)
			goto free_mem;

		ste_gpio->flags &= GPIO_FLAGS_MASK;
		ste_gpio->flags |= (unsigned long)id << PDESC_ID_SHIFT;

		if (ste_gpio->flags >> PDESC_ID_SHIFT != id) {
			ret = -ERANGE;
			goto free_id;
		}

		pdesc->value_sd = sysfs_get_dirent(dev->kobj.sd, NULL, "value");
		if (!pdesc->value_sd) {
			ret = -ENODEV;
			goto free_id;
		}
		INIT_WORK(&pdesc->work, gpio_notify_sysfs);
	}

	ret = request_irq(irq, gpio_sysfs_irq, irq_flags,
			"ste", &pdesc->work);
	if (ret)
		goto free_sd;

	ste_gpio->flags |= gpio_flags;
	return 0;

free_sd:
	if (pdesc)
		sysfs_put(pdesc->value_sd);
free_id:
	idr_remove(&pdesc_idr, id);
	ste_gpio->flags &= GPIO_FLAGS_MASK;
free_mem:
	kfree(pdesc);
err_out:
	return ret;
}

static ssize_t gpio_edge_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ste_gpio *ste_gpio = dev_get_drvdata(dev);
	ssize_t status = 0;
	int i;

	mutex_lock(&ste_lock);

	for (i = 0; i < ARRAY_SIZE(trigger_types); i++)
		if ((ste_gpio->flags & GPIO_TRIGGER_MASK)
				== trigger_types[i].flags) {
			status = sprintf(buf, "%s\n",
					 trigger_types[i].name);
			break;
		}

	mutex_unlock(&ste_lock);
	return status;
}

static ssize_t gpio_edge_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(trigger_types); i++)
		if (sysfs_streq(trigger_types[i].name, buf))
			goto found;
	return -EINVAL;

found:
	mutex_lock(&ste_lock);

	status = gpio_setup_irq(dev, trigger_types[i].flags);
	if (!status)
		status = size;

	mutex_unlock(&ste_lock);

	return status;
}

static DEVICE_ATTR(edge, 0644, gpio_edge_show, gpio_edge_store);

static int __init mapphone_ste_m570_init_gpio(void)
{
	int i, status, pin_nr;
	struct device *dev;

	for (i = 0; i < ARRAY_SIZE(ste_gpio); i++) {
		pin_nr = get_gpio_by_name(ste_gpio[i].device_name);
		if (pin_nr < 0) {
			printk(KERN_DEBUG "%s: can't get %s from device_tree\n",
					__func__, ste_gpio[i].device_name);
			ste_gpio[i].device_found = 0;
		} else {
			ste_gpio[i].pin_nr = pin_nr;
			ste_gpio[i].device_found = 1;
		}
	}

	idr_init(&pdesc_idr);
	status = class_register(&ste_gpio_class);

	for (i = 0; i < ARRAY_SIZE(ste_gpio); i++) {
		/* Skip GPIOs not found in the device_tree */
		if (ste_gpio[i].device_found == 0)
			continue;
		status = gpio_request(ste_gpio[i].pin_nr, "ste");
		if (status < 0) {
			printk(KERN_ERR "STE: gpio%d request failed!\n",
					ste_gpio[i].pin_nr);
			return status;
		}

		dev = device_create(&ste_gpio_class, NULL, MKDEV(0, 0),
					(void *) &ste_gpio[i], "gpio%d",
					ste_gpio[i].pin_nr);
		if (IS_ERR(dev)) {
			printk(KERN_ERR "STE: device gpio%d could "
				"not be created!\n", ste_gpio[i].pin_nr);
			return IS_ERR(dev);
		}

		status = device_create_file(dev, &dev_attr_value);
		if (status) {
			printk(KERN_ERR "STE: attribute gpio%d could "
				"not be created!\n", ste_gpio[i].pin_nr);
			return status;
		}

		status = device_create_file(dev, &dev_attr_edge);
		if (status) {
			printk(KERN_ERR "STE: attribute gpio%d could "
				"not be created!\n", ste_gpio[i].pin_nr);
			return status;
		}

		gpio_set_value_cansleep(ste_gpio[i].pin_nr, ste_gpio[i].value);
		if (ste_gpio[i].direction)
			gpio_direction_input(ste_gpio[i].pin_nr);
		else
			gpio_direction_output(ste_gpio[i].pin_nr,
							ste_gpio[i].value);
	}

	return status;
}

static int mapphone_ste_m570_mdm_ctrl_init(void)
{
	mapphone_ste_m570_init_gpio();
	return 0;
}

static inline void mapphone_bpwake_init(void)
{
	mapphone_bpwake_gpio = get_gpio_by_name("ipc_apwake_trigger");
	if (mapphone_bpwake_gpio >= 0) {
		mdm6600_ctrl_platform_data.mapphone_bpwake_device =
			&mapphone_bpwake_device;
		platform_device_register(&mapphone_bpwake_device);
	}
}

int __init mapphone_mdm_ctrl_init(void)
{
	int i, ret = 0;

	if (bi_powerup_reason() == PU_REASON_CHARGER) {
		pr_info("Charge Only Mode : No MDMCTRL");
		return ret;
	}

	mapphone_bpwake_init();

	mapphone_modem.n_modems = mapphone_bp_get_count();
	if (!mapphone_modem.n_modems ||
		mapphone_modem.n_modems > MAX_MODEMS) {

		pr_err("Invalid BPs #%d Found, Is this a Phone ??",
			mapphone_modem.n_modems);
		return -ENODEV;
	}

	for (i = 0; i < mapphone_modem.n_modems && ret == 0; ++i) {
		mapphone_modem.type[i] = mapphone_bp_get_type(i);
		switch (mapphone_modem.type[i]) {
		case MAPPHONE_BP_MDM6600:
			pr_debug("Found Modem MAPPHONE_BP_MDM6600");
			ret = mapphone_mdm6600_mdm_ctrl_init();
			break;
		case MAPPHONE_BP_MDM9600:
			pr_debug("Found Modem MAPPHONE_BP_MDM9600");
			ret = mapphone_mdm9600_mdm_ctrl_init();
			break;
		case MAPPHONE_BP_QSC6085:
			pr_debug("Found Modem MAPPHONE_BP_QSC6085");
			ret = mapphone_qsc6085_mdm_ctrl_init();
			break;
		case MAPPHONE_BP_STE_M570:
			pr_debug("Found Modem MAPPHONE_BP_STE_M570");
			ret = mapphone_ste_m570_mdm_ctrl_init();
			break;
		case MAPPHONE_BP_W3GLTE:
			pr_debug("Found Modem MAPPHONE_BP_W3GLTE");
			ret = mapphone_w3glte_mdm_ctrl_init();
			break;
		default:
			ret = -ENODEV;
			pr_err("Bad BP %d[ID: 0x%x], Is this a Phone ??", i,
				mapphone_modem.type[i]);
			break;
		}
	}
	return ret;
}

