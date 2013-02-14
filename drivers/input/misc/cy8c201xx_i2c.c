/*
 * Source for:
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

#include "cy8c201xx_core.h"

#include <linux/i2c.h>
#include <linux/slab.h>

#define CY_I2C_DATA_SIZE  128

struct cy201_i2c {
	struct cy201_bus_ops ops;
	struct i2c_client *client;
	void *cy201_client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
};

static s32 cy201_i2c_read_block_data(void *handle, u8 addr,
	u8 length, void *values, int i2c_addr)
{
	struct cy201_i2c *ts = container_of(handle, struct cy201_i2c, ops);
	int retval = 0;

	ts->client->addr = i2c_addr;
	retval = i2c_master_send(ts->client, &addr, 1);
	if (retval < 0)
		return retval;
	else if (retval != 1)
		return -EIO;

	retval = i2c_master_recv(ts->client, values, length);

	return (retval < 0) ? retval : retval != length ? -EIO : 0;
}

static s32 cy201_i2c_write_block_data(void *handle, u8 addr,
	u8 length, const void *values, int i2c_addr)
{
	struct cy201_i2c *ts = container_of(handle, struct cy201_i2c, ops);
	int retval;

	ts->wr_buf[0] = addr;
	memcpy(&ts->wr_buf[1], values, length);

	ts->client->addr = i2c_addr;
	retval = i2c_master_send(ts->client, ts->wr_buf, length+1);

	return (retval < 0) ? retval : retval != length+1 ? -EIO : 0;
}

static int __devinit cy201_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct cy201_i2c *ts;
	int retval;

	pr_info("%s: Starting %s probe...\n", __func__, CY201_I2C_NAME);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: fail check I2C functionality\n", __func__);
		return -EIO;
	}

	/* allocate and clear memory */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		pr_err("%s: Error, kzalloc.\n", __func__);
		return -ENOMEM;
	}

	/* register driver_data */
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->ops.write = cy201_i2c_write_block_data;
	ts->ops.read = cy201_i2c_read_block_data;
	ts->ops.dev = &client->dev;
	ts->ops.dev->bus = &i2c_bus_type;
	ts->cy201_client = cy201_core_init(&ts->ops, &client->dev,
			client->irq, client->name);

	if (IS_ERR(ts->cy201_client)) {
		retval = PTR_ERR(ts->cy201_client);
		kfree(ts);
		pr_err("%s: Registration fail ret=%d\n", __func__, retval);
		return retval;
	}

	pr_info("%s: Registration for %s complete\n", __func__, CY201_I2C_NAME);

	return 0;
}

/* registered in driver struct */
static int __devexit cy201_i2c_remove(struct i2c_client *client)
{
	struct cy201_i2c *ts;

	pr_info("%s: exit\n", __func__);
	ts = i2c_get_clientdata(client);
	cy201_core_release(ts->cy201_client);
	kfree(ts);
	return 0;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int cy201_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	struct cy201_i2c *ts = i2c_get_clientdata(client);

	return cy201_suspend(ts);
}

static int cy201_i2c_resume(struct i2c_client *client)
{
	struct cy201_i2c *ts = i2c_get_clientdata(client);

	return cy201_resume(ts);
}
#endif

static const struct i2c_device_id cy201_i2c_id[] = {
	{ CY201_I2C_NAME, 0 },  { }
};

static struct i2c_driver cy201_i2c_driver = {
	.driver = {
		.name = CY201_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = cy201_i2c_probe,
	.remove = __devexit_p(cy201_i2c_remove),
	.id_table = cy201_i2c_id,
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = cy201_i2c_suspend,
	.resume = cy201_i2c_resume,
#endif
};

static int __init cy201_i2c_init(void)
{
	return i2c_add_driver(&cy201_i2c_driver);
}

static void __exit cy201_i2c_exit(void)
{
	return i2c_del_driver(&cy201_i2c_driver);
}

module_init(cy201_i2c_init);
module_exit(cy201_i2c_exit);

MODULE_ALIAS("i2c:cy201");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress CapSense(R) I2C driver");
MODULE_AUTHOR("Cypress");
MODULE_DEVICE_TABLE(i2c, cy201_i2c_id);
