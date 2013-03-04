/* drivers/input/keyreset.c
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/input.h>
#include <linux/keyreset.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>


#define KEYRESET_TIMER_DELAY 3500   /* ms */
#define HWRESET_TIMER_DELAY 6000    /* ms */

struct keyreset_state {
	struct input_handler input_handler;
	int crash_key;
	unsigned long keybit[BITS_TO_LONGS(KEY_CNT)];
	unsigned long upbit[BITS_TO_LONGS(KEY_CNT)];
	unsigned long key[BITS_TO_LONGS(KEY_CNT)];
	spinlock_t lock;
	int key_down_target;
	int key_down;
	int key_up;
	struct timer_list keyreset_timer;
	struct timer_list hwreset_timer;
	int restart_disabled;
};

int restart_requested;
int crash_requested;

static struct work_struct keyreset;
static struct work_struct hwreset_set;
static struct work_struct hwreset_clear;
static struct workqueue_struct *_keyreset_wq;

static void deferred_restart(struct work_struct *dummy)
{
	cpcap_set_bit(CPCAP_REG_VAL1,
		CPCAP_BIT_PANIC, CPCAP_BIT_PANIC);
	restart_requested = 2;
	sys_sync();
	restart_requested = 3;
	BUG();
}

static void keyreset_timer(unsigned long data)
{
	printk(KERN_EMERG "current process is %d:%s, prio is %d.\n",
		current->pid, current->comm, current->prio);
	dump_stack();
	pr_info("keyboard reset\n");
	queue_work(_keyreset_wq, &keyreset);
	restart_requested = 1;
}

static void cpcap_set_hwreset(struct work_struct *dummy)
{
	cpcap_set_bit(CPCAP_REG_VAL1,
		CPCAP_BIT_PANIC, CPCAP_BIT_PANIC);
}

static void cpcap_clear_hwreset(struct work_struct *dummy)
{
	cpcap_set_bit(CPCAP_REG_VAL1, 0, CPCAP_BIT_PANIC);
}

static void hwreset_timer(unsigned long data)
{
	queue_work(_keyreset_wq, &hwreset_set);
}

static void keyreset_event(struct input_handle *handle, unsigned int type,
			   unsigned int code, int value)
{
	unsigned long flags;
	struct keyreset_state *state = handle->private;

	if (type != EV_KEY)
		return;

	if (code >= KEY_MAX)
		return;

	if (code == state->crash_key) {
		crash_requested = value;
		if (!crash_requested && timer_pending(&state->keyreset_timer))
			del_timer(&state->keyreset_timer);
	}

	if (!test_bit(code, state->keybit))
		return;

	spin_lock_irqsave(&state->lock, flags);
	if (!test_bit(code, state->key) == !value)
		goto done;
	__change_bit(code, state->key);
	if (test_bit(code, state->upbit)) {
		if (value) {
			state->restart_disabled = 1;
			state->key_up++;
		} else
			state->key_up--;
	} else {
		if (value)
			state->key_down++;
		else
			state->key_down--;
	}
	if (state->key_down == 0 && state->key_up == 0)
		state->restart_disabled = 0;
	pr_debug("reset key changed %d %d new state %d-%d-%d\n", code, value,
		 state->key_down, state->key_up, state->restart_disabled);

	if (value && !state->restart_disabled &&
	    state->key_down == state->key_down_target && crash_requested) {
		state->restart_disabled = 1;
		if (restart_requested)
			panic("keyboard reset failed, %d", restart_requested);
		mod_timer_pinned(&state->keyreset_timer, jiffies +
			msecs_to_jiffies(KEYRESET_TIMER_DELAY));
	}

	if (value && state->key_down == state->key_down_target) {
		mod_timer_pinned(&state->hwreset_timer, jiffies +
			msecs_to_jiffies(HWRESET_TIMER_DELAY));
	}

	if (!value && ((state->key_down_target - state->key_down) == 1)) {
		if (timer_pending(&state->keyreset_timer))
			del_timer(&state->keyreset_timer);
		if (timer_pending(&state->hwreset_timer))
			del_timer(&state->hwreset_timer);
		if (!timer_pending(&state->hwreset_timer))
			queue_work(_keyreset_wq, &hwreset_clear);
	}
done:
	spin_unlock_irqrestore(&state->lock, flags);
}

static int keyreset_connect(struct input_handler *handler,
					  struct input_dev *dev,
					  const struct input_device_id *id)
{
	int i;
	int ret;
	struct input_handle *handle;
	struct keyreset_state *state =
		container_of(handler, struct keyreset_state, input_handler);

	for (i = 0; i < KEY_MAX; i++) {
		if (test_bit(i, state->keybit) && test_bit(i, dev->keybit))
			break;
	}
	if (i == KEY_MAX)
		return -ENODEV;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "keyreset";
	handle->private = state;

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	pr_info("using input dev %s for key reset\n", dev->name);

	return 0;

err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void keyreset_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id keyreset_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};
MODULE_DEVICE_TABLE(input, keyreset_ids);

static int keyreset_probe(struct platform_device *pdev)
{
	int ret;
	int key, *keyp;
	struct keyreset_state *state;
	struct keyreset_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;
	state->crash_key = pdata->crash_key;

	spin_lock_init(&state->lock);
	keyp = pdata->keys_down;
	while ((key = *keyp++)) {
		if (key >= KEY_MAX)
			continue;
		state->key_down_target++;
		__set_bit(key, state->keybit);
	}
	if (pdata->keys_up) {
		keyp = pdata->keys_up;
		while ((key = *keyp++)) {
			if (key >= KEY_MAX)
				continue;
			__set_bit(key, state->keybit);
			__set_bit(key, state->upbit);
		}
	}
	setup_timer(&state->keyreset_timer, keyreset_timer,
		    (unsigned long)state);
	setup_timer(&state->hwreset_timer, hwreset_timer,
		    (unsigned long)state);
	state->input_handler.event = keyreset_event;
	state->input_handler.connect = keyreset_connect;
	state->input_handler.disconnect = keyreset_disconnect;
	state->input_handler.name = KEYRESET_NAME;
	state->input_handler.id_table = keyreset_ids;
	ret = input_register_handler(&state->input_handler);
	if (ret) {
		kfree(state);
		return ret;
	}
	platform_set_drvdata(pdev, state);
	return 0;
}

int keyreset_remove(struct platform_device *pdev)
{
	struct keyreset_state *state = platform_get_drvdata(pdev);
	input_unregister_handler(&state->input_handler);
	kfree(state);
	return 0;
}


struct platform_driver keyreset_driver = {
	.driver.name = KEYRESET_NAME,
	.probe = keyreset_probe,
	.remove = keyreset_remove,
};

static int __init keyreset_init(void)
{
	_keyreset_wq = create_singlethread_workqueue("keyreset");
	if (!_keyreset_wq)
		return 1;
	INIT_WORK(&keyreset, deferred_restart);
	INIT_WORK(&hwreset_set, cpcap_set_hwreset);
	INIT_WORK(&hwreset_clear, cpcap_clear_hwreset);
	return platform_driver_register(&keyreset_driver);
}

static void __exit keyreset_exit(void)
{
	flush_workqueue(_keyreset_wq);
	destroy_workqueue(_keyreset_wq);
	return platform_driver_unregister(&keyreset_driver);
}

module_init(keyreset_init);
module_exit(keyreset_exit);
