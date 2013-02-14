/*
 * Copyright (C) 2010-2012 Motorola Mobility, Inc.
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

/* Driver for Atmel maXTouch touchscreens that uses touch_platform.h */
#include "atmxt.h"
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/firmware.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_DEBUG
#include <linux/time.h>
#endif

static int atmxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int atmxt_remove(struct i2c_client *client);
static int atmxt_suspend(struct i2c_client *client, pm_message_t message);
static int atmxt_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void atmxt_early_suspend(struct early_suspend *handler);
static void atmxt_late_resume(struct early_suspend *handler);
#endif
static int __devinit atmxt_init(void);
static void __devexit atmxt_exit(void);
static void atmxt_free(struct atmxt_driver_data *dd);
static void atmxt_free_ic_data(struct atmxt_driver_data *dd);
static void atmxt_set_drv_state(struct atmxt_driver_data *dd,
		enum atmxt_driver_state state);
static int atmxt_get_drv_state(struct atmxt_driver_data *dd);
static void atmxt_set_ic_state(struct atmxt_driver_data *dd,
		enum atmxt_ic_state state);
static int atmxt_get_ic_state(struct atmxt_driver_data *dd);
static int atmxt_verify_pdata(struct atmxt_driver_data *dd);
static int atmxt_register_inputs(struct atmxt_driver_data *dd);
static int atmxt_request_irq(struct atmxt_driver_data *dd);
static int atmxt_restart_ic(struct atmxt_driver_data *dd,
		struct touch_firmware *fw, bool force_fw_upgrade);
static irqreturn_t atmxt_isr(int irq, void *handle);
static int atmxt_get_info_header(struct atmxt_driver_data *dd);
static int atmxt_get_object_table(struct atmxt_driver_data *dd);
static int atmxt_process_object_table(struct atmxt_driver_data *dd);
static int atmxt_identify_panel(struct atmxt_driver_data *dd,
		uint8_t *entry, struct touch_settings *pid_data,
		uint8_t *count, uint8_t *id);
static struct atmxt_obj *atmxt_create_object(int *err, uint8_t *entry);
static int atmxt_copy_platform_data(struct atmxt_obj *obj,
		struct touch_settings *tsett, uint8_t num_panels, uint8_t id);
static int atmxt_create_id_table(struct atmxt_driver_data *dd, uint32_t ids);
static int atmxt_check_settings(struct atmxt_driver_data *dd, bool *reset);
static int atmxt_send_settings(struct atmxt_driver_data *dd, bool save_nvm);
static int atmxt_recalibrate_ic(struct atmxt_driver_data *dd);
static int atmxt_start_ic_calibration_fix(struct atmxt_driver_data *dd);
static int atmxt_verify_ic_calibration_fix(struct atmxt_driver_data *dd);
static int atmxt_stop_ic_calibration_fix(struct atmxt_driver_data *dd);
static int atmxt_i2c_write(struct atmxt_driver_data *dd,
		uint8_t addr_lo, uint8_t addr_hi,
		const uint8_t *buf, int size);
static int atmxt_i2c_read(struct atmxt_driver_data *dd, uint8_t *buf, int size);
static void atmxt_check_useful_addr(struct atmxt_driver_data *dd,
		uint8_t *entry);
static int atmxt_set_useful_data(struct atmxt_driver_data *dd);
static void atmxt_compute_checksum(struct atmxt_driver_data *dd);
static void atmxt_compute_partial_checksum(uint8_t *byte1, uint8_t *byte2,
		uint8_t *low, uint8_t *mid, uint8_t *high);
static void atmxt_active_handler(struct atmxt_driver_data *dd);
static struct atmxt_obj *atmxt_get_obj(
		struct atmxt_driver_data *dd, uint8_t num);
static uint8_t *atmxt_get_entry(struct atmxt_driver_data *dd, uint8_t num);
static int atmxt_process_message(struct atmxt_driver_data *dd,
		uint8_t *msg, uint8_t size);
static void atmxt_report_touches(struct atmxt_driver_data *dd);
static void atmxt_release_touches(struct atmxt_driver_data *dd);
static int atmxt_message_handler6(struct atmxt_driver_data *dd,
		uint8_t *msg, uint8_t size);
static int atmxt_message_handler9(struct atmxt_driver_data *dd,
		uint8_t *msg, uint8_t size);
static int atmxt_message_handler42(struct atmxt_driver_data *dd,
		uint8_t *msg, uint8_t size);
static int atmxt_resume_restart(struct atmxt_driver_data *dd);
static int atmxt_force_bootloader(struct atmxt_driver_data *dd);
static bool atmxt_check_firmware_upgrade(struct atmxt_driver_data *dd,
		struct touch_firmware *fw);
static int atmxt_validate_firmware(const uint8_t *img, uint32_t size);
static int atmxt_flash_firmware(struct atmxt_driver_data *dd,
		const uint8_t *img, uint32_t size);
static char *atmxt_msg2str(const uint8_t *msg, uint8_t size);
static bool atmxt_wait4irq(struct atmxt_driver_data *dd);
static int atmxt_create_debug_files(struct atmxt_driver_data *dd);
static void atmxt_remove_debug_files(struct atmxt_driver_data *dd);

static const struct i2c_device_id atmxt_id[] = {
	/* This name must match the i2c_board_info name */
	{ ATMXT_I2C_NAME, 0 }, { }
};

MODULE_DEVICE_TABLE(i2c, atmxt_id);

static struct i2c_driver atmxt_driver = {
	.driver = {
		.name = ATMXT_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = atmxt_probe,
	.remove = __devexit_p(atmxt_remove),
	.id_table = atmxt_id,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = atmxt_suspend,
	.resume = atmxt_resume,
#endif
};

static int atmxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct atmxt_driver_data *dd = NULL;
	int err = 0;
	bool debugfail = false;
	bool icfail = false;

	printk(KERN_INFO "%s: Driver: %s, Version: %s, Date: %s\n", __func__,
		ATMXT_I2C_NAME, ATMXT_DRIVER_VERSION, ATMXT_DRIVER_DATE);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: I2C_FUNC_I2C failure.\n", __func__);
		err = -ENODEV;
		goto atmxt_probe_fail;
	}

	if (client == NULL) {
		printk(KERN_ERR "%s: I2C client structure is missing.\n",
			__func__);
		err = -EINVAL;
		goto atmxt_probe_fail;
	}

	dd = kzalloc(sizeof(struct atmxt_driver_data), GFP_KERNEL);
	if (dd == NULL) {
		printk(KERN_ERR "%s: Unable to create driver data.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_probe_fail;
	}

	dd->drv_stat = ATMXT_DRV_PROBE;
	dd->ic_stat = ATMXT_IC_UNKNOWN;
	dd->status = 0x0000;
	dd->client = client;
	dd->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, dd);
	dd->in_dev = NULL;

	dd->mutex = kzalloc(sizeof(struct mutex), GFP_KERNEL);
	if (dd->mutex == NULL) {
		printk(KERN_ERR "%s: Unable to create mutex lock.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_probe_fail;
	}
	mutex_init(dd->mutex);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	dd->dbg = kzalloc(sizeof(struct atmxt_debug), GFP_KERNEL);
	if (dd->dbg == NULL) {
		printk(KERN_ERR "%s: Unable to create driver debug data.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_probe_fail;
	}
	dd->dbg->dbg_lvl = ATMXT_DBG0;
#endif

	err = atmxt_verify_pdata(dd);
	if (err < 0)
		goto atmxt_probe_fail;

	dd->settings = dd->pdata->flags;

	err = atmxt_register_inputs(dd);
	if (err < 0)
		goto atmxt_probe_fail;

#ifdef CONFIG_HAS_EARLYSUSPEND
	dd->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	dd->es.suspend = atmxt_early_suspend;
	dd->es.resume = atmxt_late_resume;
	register_early_suspend(&dd->es);
#endif

	err = atmxt_request_irq(dd);
	if (err < 0)
		goto atmxt_probe_fail;

	mutex_lock(dd->mutex);

	err = atmxt_create_debug_files(dd);
	if (err < 0) {
		printk(KERN_ERR
			"%s: Probe had error %d when creating debug files.\n",
			__func__, err);
		debugfail = true;
	}

	err = atmxt_restart_ic(dd, dd->pdata->fw, false);
	if (err < 0) {
		printk(KERN_ERR
			"%s: Probe had error %d when restarting IC.\n",
				__func__, err);
		icfail = true;
	}

	if (!icfail) {
		atmxt_set_drv_state(dd, ATMXT_DRV_ACTIVE);
		dd->status = dd->status | (1 << ATMXT_IRQ_ENABLED_FLAG);
		enable_irq(dd->client->irq);
	} else {
		atmxt_set_drv_state(dd, ATMXT_DRV_IDLE);
	}

	mutex_unlock(dd->mutex);

	goto atmxt_probe_pass;

atmxt_probe_fail:
	atmxt_free(dd);
	printk(KERN_ERR "%s: Probe failed with error code %d.\n",
		__func__, err);
	return err;

atmxt_probe_pass:
	if (debugfail || icfail) {
		printk(KERN_INFO "%s: Probe completed with errors.\n",
			__func__);
	} else {
		printk(KERN_INFO "%s: Probe successful.\n", __func__);
	}
	return 0;
}

static int atmxt_remove(struct i2c_client *client)
{
	struct atmxt_driver_data *dd;

	dd = i2c_get_clientdata(client);
	if (dd != NULL) {
		free_irq(dd->client->irq, dd);
		atmxt_remove_debug_files(dd);
		atmxt_free(dd);
	}

	i2c_set_clientdata(client, NULL);

	return 0;
}

static int atmxt_suspend(struct i2c_client *client, pm_message_t message)
{
	int err = 0;
	struct atmxt_driver_data *dd;
	int drv_state;
	int ic_state;
	uint8_t sleep_cmd[2] = {0x00, 0x00};

	dd = i2c_get_clientdata(client);
	if (dd == NULL) {
		printk(KERN_ERR "%s: Driver data is missing.\n", __func__);
		err = -ENODATA;
		goto atmxt_suspend_no_dd_fail;
	}

	mutex_lock(dd->mutex);

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Suspending...\n", __func__);

	drv_state = atmxt_get_drv_state(dd);
	ic_state = atmxt_get_ic_state(dd);
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	printk(KERN_INFO "%s: last event info - time:%lu.%lu "
		"count:%lu, active:%d, status:0x%02X\n",
		__func__,
		(long unsigned int)dd->dbg->evt_sec,
		(long unsigned int)dd->dbg->evt_ms,
		(long unsigned int)dd->dbg->evt_count,
		dd->dbg->evt_act, dd->status);
#endif

	switch (drv_state) {
	case ATMXT_DRV_ACTIVE:
	case ATMXT_DRV_IDLE:
		switch (ic_state) {
		case ATMXT_IC_ACTIVE:
			atmxt_dbg(dd, ATMXT_DBG3,
				"%s: Putting touch IC to sleep...\n", __func__);
			dd->status = dd->status &
				~(1 << ATMXT_FIXING_CALIBRATION);
			err = atmxt_i2c_write(dd,
				dd->addr->pwr[0], dd->addr->pwr[1],
				&(sleep_cmd[0]), 2);
			if (err < 0) {
				printk(KERN_ERR
					"%s: %s %s %d.\n", __func__,
					"Failed to put touch IC to sleep",
					"with error code", err);
				goto atmxt_suspend_fail;
			} else {
				atmxt_set_ic_state(dd, ATMXT_IC_SLEEP);
			}
			break;
		default:
			printk(KERN_ERR "%s: Driver %s, IC %s suspend.\n",
				__func__, atmxt_driver_state_string[drv_state],
				atmxt_ic_state_string[ic_state]);
		}
		break;

	case ATMXT_DRV_ERROR:
		break;

	default:
		printk(KERN_ERR "%s: Driver state \"%s\" suspend.\n",
			__func__, atmxt_driver_state_string[drv_state]);
	}

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Suspend complete.\n", __func__);

atmxt_suspend_fail:
	mutex_unlock(dd->mutex);

atmxt_suspend_no_dd_fail:
	return err;
}

static int atmxt_resume(struct i2c_client *client)
{
	int err = 0;
	struct atmxt_driver_data *dd;
	int drv_state;
	int ic_state;

	dd = i2c_get_clientdata(client);
	if (dd == NULL) {
		printk(KERN_ERR "%s: Driver data is missing.\n", __func__);
		err = -ENODATA;
		goto atmxt_resume_no_dd_fail;
	}

	mutex_lock(dd->mutex);

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Resuming...\n", __func__);

	drv_state = atmxt_get_drv_state(dd);
	ic_state = atmxt_get_ic_state(dd);

	switch (drv_state) {
	case ATMXT_DRV_ACTIVE:
	case ATMXT_DRV_IDLE:
		switch (ic_state) {
		case ATMXT_IC_ACTIVE:
			printk(KERN_ERR "%s: Driver %s, IC %s resume.\n",
				__func__, atmxt_driver_state_string[drv_state],
				atmxt_ic_state_string[ic_state]);
			break;
		case ATMXT_IC_SLEEP:
			atmxt_dbg(dd, ATMXT_DBG3,
				"%s: Waking touch IC...\n", __func__);
			err = atmxt_i2c_write(dd,
				dd->addr->pwr[0], dd->addr->pwr[1],
				&(dd->data->pwr[0]), 2);
			if (err < 0) {
				printk(KERN_ERR
					"%s: Failed to wake touch IC %s %d.\n",
					__func__, "with error code", err);
				err = atmxt_resume_restart(dd);
				if (err < 0) {
					printk(KERN_ERR
						"%s: %s %s %d.\n",
						__func__,
						"Failed restart after resume",
						"with error code", err);
				}
				goto atmxt_resume_fail;
			} else {
				atmxt_set_ic_state(dd, ATMXT_IC_ACTIVE);
			}
			err = atmxt_start_ic_calibration_fix(dd);
			if (err < 0) {
				printk(KERN_ERR "%s: %s %s %d.\n", __func__,
					"Failed to start calibration fix",
					"with error code", err);
				goto atmxt_resume_fail;
			}
			err = atmxt_recalibrate_ic(dd);
			if (err < 0) {
				printk(KERN_ERR
					"%s: Recalibration failed %s %d.\n",
					__func__, "with error code", err);
				goto atmxt_resume_fail;
			}
			atmxt_release_touches(dd);
			break;
		default:
			printk(KERN_ERR "%s: Driver %s, IC %s resume--%s...\n",
				__func__, atmxt_driver_state_string[drv_state],
				atmxt_ic_state_string[ic_state], "recovering");
			err = atmxt_resume_restart(dd);
			if (err < 0) {
				printk(KERN_ERR "%s: Recovery failed %s %d.\n",
					__func__, "with error code", err);
				goto atmxt_resume_fail;
			}
		}
		break;

	case ATMXT_DRV_ERROR:
		break;

	default:
		printk(KERN_ERR "%s: Driver %s, IC %s resume--%s...\n",
			__func__, atmxt_driver_state_string[drv_state],
			atmxt_ic_state_string[ic_state], "recovering");
		err = atmxt_resume_restart(dd);
		if (err < 0) {
			printk(KERN_ERR "%s: Recovery failed %s %d.\n",
				__func__, "with error code", err);
			goto atmxt_resume_fail;
		}
	}

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Resume complete.\n", __func__);

atmxt_resume_fail:
	mutex_unlock(dd->mutex);

atmxt_resume_no_dd_fail:
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void atmxt_early_suspend(struct early_suspend *handler)
{
	int err = 0;
	struct atmxt_driver_data *dd;

	dd = container_of(handler, struct atmxt_driver_data, es);

	err = atmxt_suspend(dd->client, PMSG_SUSPEND);
	if (err < 0) {
		printk(KERN_ERR "%s: Suspend failed with error code %d",
			__func__, err);
	}

	return;
}
static void atmxt_late_resume(struct early_suspend *handler)
{
	int err = 0;
	struct atmxt_driver_data *dd;

	dd = container_of(handler, struct atmxt_driver_data, es);

	err = atmxt_resume(dd->client);
	if (err < 0) {
		printk(KERN_ERR "%s: Resume failed with error code %d",
			__func__, err);
	}

	return;
}
#endif

static int __devinit atmxt_init(void)
{
	return i2c_add_driver(&atmxt_driver);
}

static void __devexit atmxt_exit(void)
{
	i2c_del_driver(&atmxt_driver);
	return;
}

module_init(atmxt_init);
module_exit(atmxt_exit);

static void atmxt_free(struct atmxt_driver_data *dd)
{
	if (dd != NULL) {
		dd->pdata = NULL;
		dd->client = NULL;
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&dd->es);
#endif

		if (dd->mutex != NULL) {
			mutex_destroy(dd->mutex);
			kfree(dd->mutex);
			dd->mutex = NULL;
		}

		if (dd->in_dev != NULL) {
			input_unregister_device(dd->in_dev);
			dd->in_dev = NULL;
		}

		atmxt_free_ic_data(dd);

		if (dd->rdat != NULL) {
			kfree(dd->rdat);
			dd->rdat = NULL;
		}

		if (dd->dbg != NULL) {
			kfree(dd->dbg);
			dd->dbg = NULL;
		}

		kfree(dd);
		dd = NULL;
	}

	return;
}

static void atmxt_free_ic_data(struct atmxt_driver_data *dd)
{
	struct atmxt_obj *cur_obj;
	struct atmxt_obj *next_obj;
	struct atmxt_obj *cur_inst;
	struct atmxt_obj *next_inst;

	if (dd->info_blk != NULL) {
		kfree(dd->info_blk->data);
		dd->info_blk->data = NULL;
		kfree(dd->info_blk->msg_id);
		dd->info_blk->msg_id = NULL;
		kfree(dd->info_blk);
		dd->info_blk = NULL;
	}

	cur_obj = dd->objs;
	while (cur_obj != NULL) {
		next_obj = cur_obj->next_obj;
		cur_inst = cur_obj->next_inst;
		while (cur_inst != NULL) {
			next_inst = cur_inst->next_inst;
			kfree(cur_inst->data);
			kfree(cur_inst);
			cur_inst = next_inst;
		}
		kfree(cur_obj->data);
		kfree(cur_obj);
		cur_obj = next_obj;
	}
	dd->objs = NULL;

	if (dd->addr != NULL) {
		kfree(dd->addr);
		dd->addr = NULL;
	}

	if (dd->data != NULL) {
		kfree(dd->data);
		dd->data = NULL;
	}

	return;
}

static void atmxt_set_drv_state(struct atmxt_driver_data *dd,
		enum atmxt_driver_state state)
{
	printk(KERN_INFO "%s: Driver state %s -> %s\n", __func__,
		atmxt_driver_state_string[dd->drv_stat],
		atmxt_driver_state_string[state]);
	dd->drv_stat = state;
	return;
}

static int atmxt_get_drv_state(struct atmxt_driver_data *dd)
{
	return dd->drv_stat;
}

static void atmxt_set_ic_state(struct atmxt_driver_data *dd,
		enum atmxt_ic_state state)
{
	printk(KERN_INFO "%s: IC state %s -> %s\n", __func__,
		atmxt_ic_state_string[dd->ic_stat],
		atmxt_ic_state_string[state]);
	dd->ic_stat = state;
	return;
}

static int atmxt_get_ic_state(struct atmxt_driver_data *dd)
{
	return dd->ic_stat;
}

static int atmxt_verify_pdata(struct atmxt_driver_data *dd)
{
	int err = 0;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Verifying platform data...\n", __func__);

	if (dd->pdata == NULL) {
		printk(KERN_ERR "%s: Platform data is missing.\n", __func__);
		err = -ENODATA;
		goto atmxt_verify_pdata_fail;
	}

	if (dd->pdata->frmwrk == NULL) {
		printk(KERN_ERR "%s: Platform framework data is missing.\n",
			__func__);
		err = -ENODATA;
		goto atmxt_verify_pdata_fail;
	} else if (dd->pdata->frmwrk->abs == NULL) {
		printk(KERN_ERR "%s: Platform abs data is missing.\n",
			__func__);
		err = -ENODATA;
		goto atmxt_verify_pdata_fail;
	} else if (dd->pdata->frmwrk->size == 0 ||
				dd->pdata->frmwrk->size % 5 != 0) {
		printk(KERN_ERR "%s: Platform abs data format is invalid.\n",
			__func__);
		err = -EINVAL;
		goto atmxt_verify_pdata_fail;
	}

	if (dd->pdata->hw_reset == NULL) {
		printk(KERN_ERR "%s: Hardware reset function is missing.\n",
			__func__);
		err = -ENOSYS;
		goto atmxt_verify_pdata_fail;
	}

	if (dd->pdata->hw_recov == NULL) {
		printk(KERN_ERR "%s: Hardware recovery function is missing.\n",
			__func__);
		err = -ENOSYS;
		goto atmxt_verify_pdata_fail;
	}

	if (dd->pdata->irq_stat == NULL) {
		printk(KERN_ERR "%s: IRQ status function is missing.\n",
			__func__);
		err = -ENOSYS;
		goto atmxt_verify_pdata_fail;
	}

	goto atmxt_verify_pdata_pass;

atmxt_verify_pdata_fail:
	atmxt_set_drv_state(dd, ATMXT_DRV_ERROR);

atmxt_verify_pdata_pass:
	return err;
}

static int atmxt_register_inputs(struct atmxt_driver_data *dd)
{
	int err = 0;
	int i = 0;
	int iter = 0;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Registering inputs...\n", __func__);

	dd->rdat = kzalloc(sizeof(struct atmxt_report_data), GFP_KERNEL);
	if (dd->rdat == NULL) {
		printk(KERN_ERR "%s: Unable to create report data.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_register_inputs_allocation_fail;
	}

	dd->in_dev = input_allocate_device();
	if (dd->in_dev == NULL) {
		printk(KERN_ERR "%s: Failed to allocate input device.\n",
			__func__);
		err = -ENODEV;
		goto atmxt_register_inputs_allocation_fail;
	}

	dd->in_dev->name = ATMXT_I2C_NAME;
	input_set_drvdata(dd->in_dev, dd);

	set_bit(EV_ABS, dd->in_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, dd->in_dev->propbit);
	for (i = 0; i < dd->pdata->frmwrk->size; i += 5) {
		if (dd->pdata->frmwrk->abs[i+0] != ATMXT_ABS_RESERVED) {
			input_set_abs_params(dd->in_dev,
				dd->pdata->frmwrk->abs[i+0],
				dd->pdata->frmwrk->abs[i+1],
				dd->pdata->frmwrk->abs[i+2],
				dd->pdata->frmwrk->abs[i+3],
				dd->pdata->frmwrk->abs[i+4]);
		}

		if (iter < ARRAY_SIZE(dd->rdat->axis)) {
			dd->rdat->axis[iter] = dd->pdata->frmwrk->abs[i+0];
			iter++;
		}
	}

	for (i = iter; i < ARRAY_SIZE(dd->rdat->axis); i++)
		dd->rdat->axis[i] = ATMXT_ABS_RESERVED;

	if (dd->pdata->frmwrk->enable_vkeys)
		input_set_capability(dd->in_dev, EV_KEY, KEY_PROG1);

	input_set_events_per_packet(dd->in_dev,
		ATMXT_MAX_TOUCHES * (ARRAY_SIZE(dd->rdat->axis) + 1));

	err = input_register_device(dd->in_dev);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to register input device.\n",
			__func__);
		err = -ENODEV;
		goto atmxt_register_inputs_registration_fail;
	}

	goto atmxt_register_inputs_pass;

atmxt_register_inputs_registration_fail:
	input_free_device(dd->in_dev);
	dd->in_dev = NULL;

atmxt_register_inputs_allocation_fail:
	atmxt_set_drv_state(dd, ATMXT_DRV_ERROR);

atmxt_register_inputs_pass:
	return err;
}

static int atmxt_request_irq(struct atmxt_driver_data *dd)
{
	int err = 0;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Requesting IRQ...\n", __func__);

	err = dd->pdata->irq_stat();
	if (err < 0) {
		printk(KERN_ERR "%s: Cannot test IRQ line level.\n", __func__);
		goto atmxt_request_irq_fail;
	} else if (err == 0) {
		printk(KERN_ERR
			"%s: Line already active; cannot request IRQ.\n",
			__func__);
		err = -EIO;
		goto atmxt_request_irq_fail;
	}

	err = request_threaded_irq(dd->client->irq, NULL, atmxt_isr,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, ATMXT_I2C_NAME, dd);
	if (err < 0) {
		printk(KERN_ERR "%s: IRQ request failed.\n", __func__);
		goto atmxt_request_irq_fail;
	}

	disable_irq_nosync(dd->client->irq);

	goto atmxt_request_irq_pass;

atmxt_request_irq_fail:
	atmxt_set_drv_state(dd, ATMXT_DRV_ERROR);

atmxt_request_irq_pass:
	return err;
}

static int atmxt_restart_ic(struct atmxt_driver_data *dd,
		struct touch_firmware *fw, bool force_fw_upgrade)
{
	int err = 0;
	bool irq_low = false;
	uint32_t size = 0;
	bool upgrade_fw = false;
	bool need_reset = false;
	int cur_drv_state = 0;
	bool upgrade_complete = false;

	if (force_fw_upgrade) {
		upgrade_fw = true;
		goto atmxt_restart_upgradefw_check;
	}

atmxt_restart_ic_start:
	atmxt_dbg(dd, ATMXT_DBG3, "%s: Restarting IC...\n", __func__);

	atmxt_free_ic_data(dd);
	atmxt_release_touches(dd);
	irq_low = false;
	if (atmxt_get_ic_state(dd) != ATMXT_IC_UNKNOWN)
		atmxt_set_ic_state(dd, ATMXT_IC_UNKNOWN);

	if (!upgrade_fw && !upgrade_complete) {
		atmxt_dbg(dd, ATMXT_DBG2,
			"%s: Resetting touch IC...\n", __func__);
		err = dd->pdata->hw_reset();
		if (err < 0) {
			printk(KERN_ERR
				"%s: Unable to bring touch IC out of reset.\n",
				__func__);
			goto atmxt_restart_ic_fail;
		}
	}

	irq_low = atmxt_wait4irq(dd);
	if (!irq_low && !upgrade_fw) {
		printk(KERN_ERR "%s: Timeout waiting for interrupt.\n",
			__func__);
		err = -ETIME;
		goto atmxt_restart_ic_fail;
	}

	dd->info_blk = kzalloc(sizeof(struct atmxt_info_block), GFP_KERNEL);
	if (dd->info_blk == NULL) {
		printk(KERN_ERR "%s: Unable to create info block data.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_restart_ic_fail;
	}

	if (upgrade_fw) {
		if (!irq_low) {
			atmxt_dbg(dd, ATMXT_DBG3,
				"%s: Ignored interrupt timeout.\n", __func__);
		}

		atmxt_dbg(dd, ATMXT_DBG3,
			"%s: Trying bootloader...\n", __func__);
		upgrade_fw = false;
		goto atmxt_restart_ic_upgradefw_start;
	}

	err = atmxt_get_info_header(dd);
	if (err < 0) {
		atmxt_dbg(dd, ATMXT_DBG3,
			"%s: Error reaching IC in normal mode.  %s\n",
			__func__, "Trying bootloader...");
atmxt_restart_ic_upgradefw_start:
		dd->client->addr = dd->pdata->addr[1];
		err = atmxt_i2c_read(dd, &(dd->info_blk->header[0]), 3);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to find touch IC.\n",
				__func__);
			dd->client->addr = dd->pdata->addr[0];
			atmxt_set_ic_state(dd, ATMXT_IC_UNAVAILABLE);
			goto atmxt_restart_ic_fail;
		}

		atmxt_set_ic_state(dd, ATMXT_IC_BOOTLOADER);
		if (dd->info_blk->header[0] & 0x20) {
			printk(KERN_INFO "%s: %s: 0x%02X, %s: 0x%02X\n",
				__func__,
				"Bootloader ID", dd->info_blk->header[1],
				"Bootloader Version", dd->info_blk->header[2]);
		} else {
			printk(KERN_INFO "%s: Bootloader ID: 0x%02X\n",
				__func__, dd->info_blk->header[0] & 0x1F);
		}

		if (!(dd->info_blk->header[0] & 0x80)) {
			if (upgrade_complete) {
				printk(KERN_ERR "%s: Firmware CRC failure.\n",
					__func__);
				atmxt_set_ic_state(dd, ATMXT_IC_ERR_BADAPP);
			} else {
				printk(KERN_INFO
					"%s: Firmware CRC failure; %s.\n",
					__func__, "going to try reflashing IC");
			}
		}

		if (upgrade_complete) {
			printk(KERN_ERR "%s: %s--%s.\n", __func__,
				"Still in bootloader mode after reflash",
				"check firmware image");
			dd->client->addr = dd->pdata->addr[0];
			err = -EINVAL;
			goto atmxt_restart_ic_fail;
		}

		if (fw == NULL) {
			printk(KERN_ERR "%s: %s. %s.\n",
				__func__, "No platform firmware image present",
				"Unable to reflash touch IC");
			atmxt_set_ic_state(dd, ATMXT_IC_ERR_BADAPP);
			dd->client->addr = dd->pdata->addr[0];
			err = -ENOENT;
			goto atmxt_restart_ic_fail;
		}

		err = atmxt_validate_firmware(fw->img, fw->size);
		if (err < 0) {
			printk(KERN_ERR
				"%s: Platform firmware image is invalid.\n",
				__func__);
			dd->client->addr = dd->pdata->addr[0];
			atmxt_set_ic_state(dd, ATMXT_IC_ERR_BADAPP);
			goto atmxt_restart_ic_fail;
		}

		cur_drv_state = atmxt_get_drv_state(dd);
		atmxt_set_drv_state(dd, ATMXT_DRV_REFLASH);
		err = atmxt_flash_firmware(dd, fw->img, fw->size);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to update IC firmware.\n",
				__func__);
			dd->client->addr = dd->pdata->addr[0];
			atmxt_set_ic_state(dd, ATMXT_IC_ERR_BADAPP);
			atmxt_set_drv_state(dd, cur_drv_state);
			goto atmxt_restart_ic_fail;
		}

		atmxt_set_drv_state(dd, cur_drv_state);
		dd->client->addr = dd->pdata->addr[0];
		atmxt_dbg(dd, ATMXT_DBG3,
			"%s: Reflash completed.  Re-starting cycle...\n",
			__func__);
		upgrade_complete = true;
		goto atmxt_restart_ic_start;
	}

	atmxt_set_ic_state(dd, ATMXT_IC_PRESENT);
	printk(KERN_INFO "%s: Family ID: 0x%02X, Variant ID: 0x%02X, " \
		"Version: 0x%02X, Build: 0x%02X, Matrix: %ux%u, Objects: %u\n",
		__func__, dd->info_blk->header[0], dd->info_blk->header[1],
		dd->info_blk->header[2], dd->info_blk->header[3],
		dd->info_blk->header[4], dd->info_blk->header[5],
		dd->info_blk->header[6]);

	if (atmxt_get_drv_state(dd) == ATMXT_DRV_PROBE) {
		upgrade_fw = atmxt_check_firmware_upgrade(dd, fw);
		if (upgrade_fw) {
			err = atmxt_validate_firmware(fw->img, fw->size);
			if (err < 0) {
				printk(KERN_ERR "%s: %s--%s.\n", __func__,
					"Firmware upgrade image is invalid",
					"not upgrading");
				upgrade_fw = false;
			}

			if (upgrade_complete) {
				printk(KERN_ERR "%s: %s %s %s.\n", __func__,
					"Platform firmware version",
					"does not match platform firmware",
					"after upgrade");
				upgrade_fw = false;
			}
		}
	}

	size = dd->info_blk->header[6] * 6;
	if (size > 255) {
		printk(KERN_ERR "%s: Too many objects present.\n", __func__);
		err = -EOVERFLOW;
		goto atmxt_restart_ic_fail;
	}
	dd->info_blk->size = size;

	dd->info_blk->data = kzalloc(sizeof(uint8_t) * size, GFP_KERNEL);
	if (dd->info_blk->data == NULL) {
		printk(KERN_ERR "%s: Unable to create table data.\n", __func__);
		err = -ENOMEM;
		goto atmxt_restart_ic_fail;
	}

	err = atmxt_get_object_table(dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Error getting object table.\n", __func__);
		if (upgrade_fw)
			goto atmxt_restart_upgradefw_check;
		else
			goto atmxt_restart_ic_fail;
	}

	dd->addr = kzalloc(sizeof(struct atmxt_addr), GFP_KERNEL);
	if (dd->addr == NULL) {
		printk(KERN_ERR "%s: Unable to create address book.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_restart_ic_fail;
	}

	dd->data = kzalloc(sizeof(struct atmxt_data), GFP_KERNEL);
	if (dd->data == NULL) {
		printk(KERN_ERR "%s: Unable to create data book.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_restart_ic_fail;
	}

	err = atmxt_process_object_table(dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Processing info block failed.\n",
			__func__);
		goto atmxt_restart_ic_fail;
	}

atmxt_restart_upgradefw_check:
	if (upgrade_fw) {
		printk(KERN_INFO "%s: Resetting IC to upgrade firmware...\n",
			__func__);
		err = atmxt_force_bootloader(dd);
		if (err < 0) {
			printk(KERN_ERR "%s: Unable to force flash mode.\n",
				__func__);
			goto atmxt_restart_ic_fail;
		}
		goto atmxt_restart_ic_start;
	}

	err = atmxt_set_useful_data(dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to set useful data.\n", __func__);
		goto atmxt_restart_ic_fail;
	}

	atmxt_compute_checksum(dd);

	err = atmxt_check_settings(dd, &need_reset);
	if (err < 0) {
		printk(KERN_ERR "%s: Unable to check/update IC %s.\n",
			__func__, "with platform settings");
		goto atmxt_restart_ic_fail;
	} else if (need_reset) {
		upgrade_fw = false;
		upgrade_complete = false;
		goto atmxt_restart_ic_start;
	}

	atmxt_set_ic_state(dd, ATMXT_IC_ACTIVE);

	err = atmxt_start_ic_calibration_fix(dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to start IC calibration fix.\n",
			__func__);
		goto atmxt_restart_ic_fail;
	}

atmxt_restart_ic_fail:
	return err;
}

static irqreturn_t atmxt_isr(int irq, void *handle)
{
	struct atmxt_driver_data *dd = handle;
	int drv_state;
	int ic_state;

	mutex_lock(dd->mutex);

	drv_state = atmxt_get_drv_state(dd);
	ic_state = atmxt_get_ic_state(dd);

	atmxt_dbg(dd, ATMXT_DBG3,
		"%s: IRQ Received -- Driver: %s, IC: %s\n", __func__,
		atmxt_driver_state_string[drv_state],
		atmxt_ic_state_string[ic_state]);

	switch (drv_state) {
	case ATMXT_DRV_ACTIVE:
		switch (ic_state) {
		case ATMXT_IC_SLEEP:
			atmxt_dbg(dd, ATMXT_DBG3,
				"%s: Servicing IRQ during sleep...\n",
				__func__);
		case ATMXT_IC_ACTIVE:
			atmxt_active_handler(dd);
			break;
		default:
			printk(KERN_ERR "%s: Driver %s, IC %s IRQ received.\n",
				__func__, atmxt_driver_state_string[drv_state],
				atmxt_ic_state_string[ic_state]);
			if (dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG)) {
				disable_irq_nosync(dd->client->irq);
				dd->status = dd->status &
					~(1 << ATMXT_IRQ_ENABLED_FLAG);
				atmxt_set_drv_state(dd, ATMXT_DRV_IDLE);
			}
			break;
		}
		break;

	default:
		printk(KERN_ERR "%s: Driver state \"%s\" IRQ received.\n",
			__func__, atmxt_driver_state_string[drv_state]);
		if (dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG)) {
			disable_irq_nosync(dd->client->irq);
			dd->status = dd->status &
				~(1 << ATMXT_IRQ_ENABLED_FLAG);
			atmxt_set_drv_state(dd, ATMXT_DRV_IDLE);
		}
		break;
	}

	atmxt_dbg(dd, ATMXT_DBG3, "%s: IRQ Serviced.\n", __func__);
	mutex_unlock(dd->mutex);

	return IRQ_HANDLED;
}

static int atmxt_get_info_header(struct atmxt_driver_data *dd)
{
	int err = 0;

	err = atmxt_i2c_write(dd, 0x00, 0x00, NULL, 0);
	if (err < 0)
		goto atmxt_get_info_header_fail;

	err = atmxt_i2c_read(dd, &(dd->info_blk->header[0]), 7);
	if (err < 0)
		goto atmxt_get_info_header_fail;

atmxt_get_info_header_fail:
	return err;
}

static int atmxt_get_object_table(struct atmxt_driver_data *dd)
{
	int err = 0;
	int i = 0;                      /* Fix Atmel's data order */
	int top = 0;                    /* Fix Atmel's data order */
	int cur = 0;                    /* Fix Atmel's data order */
	uint8_t lo_addr = 255;          /* Fix Atmel's data order */
	uint8_t hi_addr = 255;          /* Fix Atmel's data order */
	uint8_t temp[6];                /* Fix Atmel's data order */

	err = atmxt_i2c_write(dd, 0x07, 0x00, NULL, 0);
	if (err < 0) {
		printk(KERN_ERR
			"%s: Unable to set address pointer to object table.\n",
			__func__);
		goto atmxt_get_object_table_fail;
	}

	err = atmxt_i2c_read(dd, dd->info_blk->data, dd->info_blk->size);
	if (err < 0) {
		printk(KERN_ERR "%s: Object table read failed.\n", __func__);
		goto atmxt_get_object_table_fail;
	}

	/* Fix Atmel's data order */
	while (top < dd->info_blk->size) {
		for (i = top; i < dd->info_blk->size; i += 6) {
			if (dd->info_blk->data[i+2] < hi_addr) {
				lo_addr = dd->info_blk->data[i+1];
				hi_addr = dd->info_blk->data[i+2];
				cur = i;
			} else if ((dd->info_blk->data[i+2] == hi_addr) &&
				(dd->info_blk->data[i+1] < lo_addr)) {
					lo_addr = dd->info_blk->data[i+1];
					hi_addr = dd->info_blk->data[i+2];
					cur = i;
			}
		}

		memcpy(&(temp[0]), &(dd->info_blk->data[top]), 6);
		memmove(&(dd->info_blk->data[top]),
			&(dd->info_blk->data[cur]), 6);
		memcpy(&(dd->info_blk->data[cur]), &(temp[0]), 6);

		lo_addr = 255;
		hi_addr = 255;
		top = top + 6;
		cur = top;
	}

atmxt_get_object_table_fail:
	return err;
}

static int atmxt_process_object_table(struct atmxt_driver_data *dd)
{
	int err = 0;
	int i = 0;
	int j = 0;
	struct atmxt_obj *first = NULL;
	struct atmxt_obj *obj = NULL;
	struct atmxt_obj *prev = NULL;
	struct atmxt_obj *inst = NULL;
	struct atmxt_obj *prev_inst = NULL;
	uint32_t ids = 0;
	uint32_t inst_addr = 0;
	int nvm_start = 0;
	uint8_t panel_count = 0x01;
	uint8_t panel_id = 0x01;

	for (i = 0; i < dd->info_blk->size; i += 6) {
		if (dd->info_blk->data[i+0] != 38) {
			ids += dd->info_blk->data[i+5] *
				(dd->info_blk->data[i+4] + 1);
			atmxt_check_useful_addr(dd, &(dd->info_blk->data[i]));
			continue;
		} else {
			nvm_start = i;
			break;
		}
	}

	err = atmxt_identify_panel(dd, &(dd->info_blk->data[nvm_start]),
		dd->pdata->sett[38], &panel_count, &panel_id);
	if (err < 0) {
		printk(KERN_ERR "%s: Panel identification failed.\n",
			__func__);
		goto atmxt_process_object_table_fail;
	}

	for (i = nvm_start + 6; i < dd->info_blk->size; i += 6) {
		obj = atmxt_create_object(&err, &(dd->info_blk->data[i]));
		if (err < 0) {
			printk(KERN_ERR "%s: Unable to create entry %hu.\n",
				__func__, dd->info_blk->data[i]);
			goto atmxt_process_object_table_fail;
		}

		if (first == NULL)
			first = obj;

		if (prev != NULL)
			prev->next_obj = obj;
		prev = obj;

		prev_inst = obj;
		for (j = 1; j <= dd->info_blk->data[i+4]; j++) {
			inst = atmxt_create_object(&err,
				&(dd->info_blk->data[i]));
			if (err < 0) {
				printk(KERN_ERR
					"%s: %s %u %s %u.\n",
					__func__, "Unable to create instance",
					j, "of object", dd->info_blk->data[i]);
				goto atmxt_process_object_table_fail;
			}

			prev_inst->next_inst = inst;
			prev_inst = inst;

			inst_addr = (inst->addr[1] << 8) | inst->addr[0];
			inst_addr = inst_addr + (inst->size * j);
			if (inst_addr > 0xFFFF) {
				printk(KERN_ERR "%s: %s %u of object %u %s.\n",
					__func__, "The address for instance", j,
					obj->num, "exceeds addressable space");
				err = -EOVERFLOW;
				goto atmxt_process_object_table_fail;
			}

			inst->addr[0] = (uint8_t) inst_addr;
			inst->addr[1] = (uint8_t) (inst_addr >> 8);
		}

		ids += dd->info_blk->data[i+5] * (dd->info_blk->data[i+4] + 1);
		dd->info_blk->obj_size +=
			(dd->info_blk->data[i+3] + 1) *
			(dd->info_blk->data[i+4] + 1);

		err = atmxt_copy_platform_data(obj, dd->pdata->sett[obj->num],
			panel_count, panel_id);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to copy platform data.\n",
				__func__);
			goto atmxt_process_object_table_fail;
		}

		atmxt_check_useful_addr(dd, &(dd->info_blk->data[i]));
	}

	err = atmxt_create_id_table(dd, ids);
	if (err < 0) {
		printk(KERN_ERR "%s: Unable to create ID table.\n", __func__);
		goto atmxt_process_object_table_fail;
	}

atmxt_process_object_table_fail:
	dd->objs = first;
	return err;
}

static int atmxt_identify_panel(struct atmxt_driver_data *dd,
		uint8_t *entry, struct touch_settings *pid_data,
		uint8_t *count, uint8_t *id)
{
	int err = 0;
	int i = 0;
	int j = 0;
	uint8_t *buf = NULL;
	uint8_t rec_bytes = 0x00;

	if (pid_data == NULL)
		goto atmxt_identify_panel_fail;

	if (pid_data->tag == 0) {
		printk(KERN_ERR "%s: No panel tag data found--%s.\n",
			__func__, "assuming only one panel");
		*count = 0x01;
	} else {
		*count = pid_data->tag;
	}

	if (pid_data->size % *count != 0) {
		printk(KERN_ERR
			"%s: Panel identifaction misaligned.\n",
			__func__);
		err = -EINVAL;
		goto atmxt_identify_panel_fail;
	}

	if (entry[4] != 0) {
		printk(KERN_ERR "%s: %s--%s...\n", __func__,
			"Multiple instances of object 38 are not supported",
			"using only the first instance");
	}

	buf = kzalloc(sizeof(uint8_t) * (entry[3] + 1), GFP_KERNEL);
	if (buf == NULL) {
		printk(KERN_ERR "%s: Unable to allocate buf.\n", __func__);
		err = -ENOMEM;
		goto atmxt_identify_panel_fail;
	}

	err = atmxt_i2c_write(dd, entry[1], entry[2], NULL, 0);
	if (err < 0) {
		printk(KERN_ERR "%s: Unable to set address pointer.\n",
			__func__);
		goto atmxt_identify_panel_fail;
	}

	err = atmxt_i2c_read(dd, buf, (entry[3] + 1));
	if (err < 0) {
		printk(KERN_ERR "%s: Unable to read panel data.\n", __func__);
		goto atmxt_identify_panel_fail;
	}

	rec_bytes = pid_data->size / *count;
	for (i = 0; i < *count; i++) {
		for (j = 0; j < rec_bytes; j++) {
			if (buf[j] != pid_data->data[i * rec_bytes + j])
				break;
		}

		if (j == rec_bytes) {
			*id = i + 1;
			break;
		}
	}

	if (i == *count) {
		printk(KERN_ERR "%s: Panel did not match ID data--%s.\n",
			__func__, "defaulting to first panel data");
		*id = 0x01;
	}

atmxt_identify_panel_fail:
	kfree(buf);
	return err;
}

static struct atmxt_obj *atmxt_create_object(int *err, uint8_t *entry)
{
	struct atmxt_obj *obj = NULL;

	*err = 0;

	if (entry[3] == 255) {
		printk(KERN_ERR "%s: Object %hu is too large.\n",
			__func__, entry[0]);
		*err = -EOVERFLOW;
		goto atmxt_create_object_fail;
	}

	obj = kzalloc(sizeof(struct atmxt_obj), GFP_KERNEL);
	if (obj == NULL) {
		printk(KERN_ERR "%s: Unable to allocate object memory.\n",
			__func__);
		*err = -ENOMEM;
		goto atmxt_create_object_fail;
	}

	obj->num = entry[0];
	obj->addr[0] = entry[1];
	obj->addr[1] = entry[2];
	obj->size = entry[3] + 1;

	obj->data = kzalloc(sizeof(uint8_t) * obj->size, GFP_KERNEL);
	if (obj->data == NULL) {
		printk(KERN_ERR
			"%s: Unable to create data for object %u.\n",
			__func__, obj->num);
		*err = -EOVERFLOW;
		kfree(obj);
		obj = NULL;
		goto atmxt_create_object_fail;
	}

atmxt_create_object_fail:
	return obj;
}

static int atmxt_copy_platform_data(struct atmxt_obj *obj,
		struct touch_settings *tsett, uint8_t num_panels, uint8_t id)
{
	int err = 0;
	int i = 0;
	int iter = 0;
	int size = 0;
	const uint8_t *data = NULL;
	int data_size = 0;
	uint8_t inst_count = 0x00;

	if (tsett == NULL)
		goto atmxt_copy_platform_data_fail;

	if (tsett->tag % num_panels != 0) {
		printk(KERN_ERR "%s: Panel data unevenly packed.\n", __func__);
		err = -EINVAL;
		goto atmxt_copy_platform_data_fail;
	}

	if (tsett->tag == 0) {
		inst_count = 1;
		data_size = tsett->size;
		data = &(tsett->data[0]);
	} else {
		inst_count = tsett->tag / num_panels;

		if (tsett->size % (inst_count * num_panels) != 0) {
			printk(KERN_ERR "%s: Settings data unevenly packed.\n",
				__func__);
			err = -EINVAL;
			goto atmxt_copy_platform_data_fail;
		}

		data_size = tsett->size / (inst_count * num_panels);
		data = &(tsett->data[(id - 1) * data_size * inst_count]);
	}

	if (data_size > obj->size)
		size = obj->size;
	else
		size = data_size;

	for (i = 0; i < inst_count; i++) {
		if (obj != NULL) {
			memcpy(obj->data, &(data[iter]), size);
			iter += data_size;
			obj = obj->next_inst;
		} else {
			break;
		}
	}

atmxt_copy_platform_data_fail:
	return err;
}

static int atmxt_create_id_table(struct atmxt_driver_data *dd, uint32_t ids)
{
	int err = 0;
	int i = 0;
	int j = 0;
	int k = 0;
	int iter = 0;

	ids++;
	if (ids > 255) {
		printk(KERN_ERR "%s: Too many report IDs used.\n", __func__);
		err = -EOVERFLOW;
		goto atmxt_create_id_table_fail;
	}

	dd->info_blk->msg_id = kzalloc(sizeof(uint8_t) * ids, GFP_KERNEL);
	if (dd->info_blk->msg_id == NULL) {
		printk(KERN_ERR "%s: Unable to create ID table.\n", __func__);
		err = -ENOMEM;
		goto atmxt_create_id_table_fail;
	}

	dd->info_blk->id_size = ids;
	dd->info_blk->msg_id[iter] = 0;
	iter++;
	for (i = 0; i < dd->info_blk->size; i += 6) {
		for (j = 0; j <= dd->info_blk->data[i+4]; j++) {
			for (k = 0; k < dd->info_blk->data[i+5]; k++) {
				dd->info_blk->msg_id[iter] =
					dd->info_blk->data[i+0];
				iter++;
			}
		}
	}

atmxt_create_id_table_fail:
	return err;
}

static int atmxt_check_settings(struct atmxt_driver_data *dd, bool *reset)
{
	int err = 0;
	uint8_t *msg_buf = NULL;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Checking IC settings...\n", __func__);

	if (dd->data->max_msg_size < 5) {
		printk(KERN_ERR "%s: Message size is too small.\n", __func__);
		err = -EINVAL;
		goto atmxt_check_settings_fail;
	}

	msg_buf = kzalloc(sizeof(uint8_t) * dd->data->max_msg_size, GFP_KERNEL);
	if (msg_buf == NULL) {
		printk(KERN_ERR
			"%s: Unable to allocate memory for message buffer.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_check_settings_fail;
	}

	err = atmxt_i2c_write(dd, dd->addr->msg[0], dd->addr->msg[1], NULL, 0);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to set message buffer pointer.\n",
			__func__);
		goto atmxt_check_settings_fail;
	}

	err = atmxt_i2c_read(dd, msg_buf, dd->data->max_msg_size);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to read message.\n", __func__);
		goto atmxt_check_settings_fail;
	}

	atmxt_dbg(dd, ATMXT_DBG3,
		"%s: %s 0x%02X%02X%02X, %s 0x%02X%02X%02X.\n", __func__,
		"Driver checksum is", dd->info_blk->obj_chksum[0],
		dd->info_blk->obj_chksum[1], dd->info_blk->obj_chksum[2],
		"IC checksum is", msg_buf[2], msg_buf[3], msg_buf[4]);

	if ((msg_buf[2] == dd->info_blk->obj_chksum[0]) &&
		(msg_buf[3] == dd->info_blk->obj_chksum[1]) &&
		(msg_buf[4] == dd->info_blk->obj_chksum[2])) {
		err = atmxt_process_message(dd,
			msg_buf, dd->data->max_msg_size);
		if (err < 0) {
			printk(KERN_ERR "%s: Error processing first message.\n",
				__func__);
			goto atmxt_check_settings_fail;
		}
		*reset = false;
	} else if (*reset) {
		printk(KERN_ERR "%s: %s.\n", __func__,
			"Previous attempt to write platform settings failed");
		err = -EINVAL;
		goto atmxt_check_settings_fail;
	} else {
		printk(KERN_INFO "%s: Updating IC settings...\n", __func__);
		err = atmxt_send_settings(dd, true);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to update IC settings.\n",
				__func__);
			goto atmxt_check_settings_fail;
		}

		msleep(500);
		*reset = true;
	}

atmxt_check_settings_fail:
	kfree(msg_buf);
	return err;
}

static int atmxt_send_settings(struct atmxt_driver_data *dd, bool save_nvm)
{
	int err = 0;
	uint8_t *buf = NULL;
	int iter = 0;
	struct atmxt_obj *obj = NULL;
	struct atmxt_obj *inst = NULL;
	uint8_t nvm_cmd = 0x55;

	if (dd->objs == NULL) {
		printk(KERN_ERR "%s: Unable able to send settings--%s.\n",
			__func__, "no objects exist");
		err = -ENODATA;
		goto atmxt_send_settings_fail;
	}

	buf = kzalloc(sizeof(uint8_t) * dd->info_blk->obj_size, GFP_KERNEL);
	if (buf == NULL) {
		printk(KERN_ERR "%s: Unable to allocate settings buffer.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_send_settings_fail;
	}

	obj = dd->objs;
	while (obj != NULL && iter < dd->info_blk->obj_size) {
		if ((iter + obj->size) <= dd->info_blk->obj_size) {
			memcpy(&(buf[iter]), obj->data, obj->size);
			iter = iter + obj->size;
		} else {
			printk(KERN_ERR
				"%s: Attempted overflow with object %u.\n",
				__func__, obj->num);
			err = -EOVERFLOW;
			goto atmxt_send_settings_fail;
		}

		inst = obj->next_inst;
		while (inst != NULL && iter < dd->info_blk->obj_size) {
			if ((iter + inst->size) <= dd->info_blk->obj_size) {
				memcpy(&(buf[iter]), inst->data, inst->size);
				iter = iter + obj->size;
			} else {
				printk(KERN_ERR
					"%s: Inst overflow for object %u.\n",
					__func__, inst->num);
				err = -EOVERFLOW;
				goto atmxt_send_settings_fail;
			}
			inst = inst->next_inst;
		}

		obj = obj->next_obj;
	}

	err = atmxt_i2c_write(dd,
		dd->objs->addr[0], dd->objs->addr[1], buf, iter);
	if (err < 0) {
		printk(KERN_ERR "%s: Error writing settings to IC.\n",
			__func__);
		goto atmxt_send_settings_fail;
	}

	if (save_nvm) {
		err = atmxt_i2c_write(dd, dd->addr->nvm[0], dd->addr->nvm[1],
			&nvm_cmd, sizeof(uint8_t));
		if (err < 0) {
			printk(KERN_ERR "%s: Error backing up to NVM.\n",
				__func__);
			goto atmxt_send_settings_fail;
		}
	}

atmxt_send_settings_fail:
	kfree(buf);

	return err;
}

static int atmxt_recalibrate_ic(struct atmxt_driver_data *dd)
{
	int err = 0;
	uint8_t cmd = 0x01;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Asking touch IC to recalibrate...\n",
		__func__);

	err = atmxt_i2c_write(dd, dd->addr->cal[0], dd->addr->cal[1], &cmd, 1);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to send calibrate to touch IC.\n",
			__func__);
		goto atmxt_recalibrate_ic_fail;
	}

atmxt_recalibrate_ic_fail:
	return err;
}

static int atmxt_start_ic_calibration_fix(struct atmxt_driver_data *dd)
{
	int err = 0;
	uint8_t sett[6] = {0x05, 0x00, 0x00, 0x00, 0x01, 0x80};

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Starting IC calibration fix...\n",
		__func__);

	if (dd->info_blk->header[0] == 0xA0) {
		atmxt_dbg(dd, ATMXT_DBG3, "%s: IC has invalid family ID.\n",
			__func__);
		goto atmxt_start_ic_calibration_fix_fail;
	}

	sett[1] = dd->data->acq[1];
	err = atmxt_i2c_write(dd, dd->addr->acq[0], dd->addr->acq[1],
		&(sett[0]), 6);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to update acquisition settings.\n",
			__func__);
		goto atmxt_start_ic_calibration_fix_fail;
	}

	dd->data->timer = 0;
	dd->status = dd->status & ~(1 << ATMXT_RECEIVED_CALIBRATION);
	dd->status = dd->status | (1 << ATMXT_FIXING_CALIBRATION);

atmxt_start_ic_calibration_fix_fail:
	return err;
}

static int atmxt_verify_ic_calibration_fix(struct atmxt_driver_data *dd)
{
	int err = 0;
	unsigned long toc = 0;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Verifying IC calibration fix...\n",
		__func__);

	toc = jiffies;
	if (dd->status & (1 << ATMXT_RECEIVED_CALIBRATION)) {
		dd->data->timer = 0;
		dd->status = dd->status & ~(1 << ATMXT_RECEIVED_CALIBRATION);
	} else if (dd->status & (1 << ATMXT_REPORT_TOUCHES)) {
		if (dd->data->timer == 0)
			dd->data->timer = toc;

		if (((toc - dd->data->timer) * 1000 / HZ) >= 2500) {
			err = atmxt_stop_ic_calibration_fix(dd);
			if (err < 0) {
				printk(KERN_ERR "%s: %s %s.\n", __func__,
					"Failed to stop",
					"fixing IC calibration");
				goto atmxt_verify_ic_calibration_fix_fail;
			}
		}
	}

atmxt_verify_ic_calibration_fix_fail:
	return err;
}

static int atmxt_stop_ic_calibration_fix(struct atmxt_driver_data *dd)
{
	int err = 0;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Stopping IC calibration fix...\n",
		__func__);

	err = atmxt_i2c_write(dd, dd->addr->acq[0], dd->addr->acq[1],
		&(dd->data->acq[0]), 6);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to update acquisition settings.\n",
			__func__);
		goto atmxt_stop_ic_calibration_fix_fail;
	}

	dd->status = dd->status & ~(1 << ATMXT_FIXING_CALIBRATION);

atmxt_stop_ic_calibration_fix_fail:
	return err;
}

static int atmxt_i2c_write(struct atmxt_driver_data *dd,
		uint8_t addr_lo, uint8_t addr_hi,
		const uint8_t *buf, int size)
{
	int err = 0;
	uint8_t *data_out;
	int size_out;
	int i = 0;
	char *str = NULL;

	size_out = size + 2;
	data_out = kzalloc(sizeof(uint8_t) * size_out, GFP_KERNEL);
	if (data_out == NULL) {
		printk(KERN_ERR "%s: Unable to allocate write memory.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_i2c_write_exit;
	}

	data_out[0] = addr_lo;
	data_out[1] = addr_hi;
	if (buf != NULL && size > 0)
		memcpy(&(data_out[2]), buf, size);

	for (i = 1; i <= ATMXT_I2C_ATTEMPTS; i++) {
		err = i2c_master_send(dd->client, data_out, size_out);
		if (err < 0) {
			printk(KERN_ERR
				"%s: %s %d, failed with error code %d.\n",
				__func__, "On I2C write attempt", i, err);
		} else if (err < size_out) {
			printk(KERN_ERR
				"%s: %s %d, wrote %d bytes instead of %d.\n",
				__func__, "On I2C write attempt", i, err,
				size_out);
			err = -EBADE;
		} else {
			break;
		}

		udelay(ATMXT_I2C_WAIT_TIME);
	}

	if (err < 0)
		printk(KERN_ERR "%s: I2C write failed.\n", __func__);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	str = atmxt_msg2str(data_out, size_out);
#endif
	atmxt_dbg(dd, ATMXT_DBG2, "%s: %s\n", __func__, str);
	kfree(str);

atmxt_i2c_write_exit:
	kfree(data_out);

	return err;
}

static int atmxt_i2c_read(struct atmxt_driver_data *dd, uint8_t *buf, int size)
{
	int err = 0;
	int i = 0;
	char *str = NULL;

	for (i = 1; i <= ATMXT_I2C_ATTEMPTS; i++) {
		err = i2c_master_recv(dd->client, buf, size);
		if (err < 0) {
			printk(KERN_ERR
				"%s: %s %d, failed with error code %d.\n",
				__func__, "On I2C read attempt", i, err);
		} else if (err < size) {
			printk(KERN_ERR
				"%s: %s %d, received %d bytes instead of %d.\n",
				__func__, "On I2C read attempt", i, err, size);
			err = -EBADE;
		} else {
			break;
		}

		udelay(ATMXT_I2C_WAIT_TIME);
	}

	if (err < 0)
		printk(KERN_ERR "%s: I2C read failed.\n", __func__);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	str = atmxt_msg2str(buf, size);
#endif
	atmxt_dbg(dd, ATMXT_DBG1, "%s: %s\n", __func__, str);
	kfree(str);

	return err;
}

static void atmxt_check_useful_addr(struct atmxt_driver_data *dd,
		uint8_t *entry)
{
	switch (entry[0]) {
	case 5:
		dd->addr->msg[0] = entry[1];
		dd->addr->msg[1] = entry[2];
		break;

	case 6:
		dd->addr->rst[0] = entry[1];
		dd->addr->rst[1] = entry[2];

		dd->addr->nvm[0] = entry[1] + 1;
		dd->addr->nvm[1] = entry[2];
		if (dd->addr->nvm[0] < entry[1])
			dd->addr->nvm[1]++;

		dd->addr->cal[0] = entry[1] + 2;
		dd->addr->cal[1] = entry[2];
		if (dd->addr->cal[0] < entry[1])
			dd->addr->cal[1]++;

		dd->addr->dbg_cmd[0] = entry[1] + 5;
		dd->addr->dbg_cmd[1] = entry[2];
		if (dd->addr->dbg_cmd[0] < entry[1])
			dd->addr->dbg_cmd[1]++;

		break;

	case 7:
		dd->addr->pwr[0] = entry[1];
		dd->addr->pwr[1] = entry[2];
		break;

	case 8:
		dd->addr->acq[0] = entry[1] + 4;
		dd->addr->acq[1] = entry[2];
		if (dd->addr->acq[0] < entry[1])
			dd->addr->acq[1]++;
		break;

	case 18:
		dd->addr->chg_cmd[0] = entry[1] + 1;
		dd->addr->chg_cmd[1] = entry[2];
		if (dd->addr->chg_cmd[0] < entry[1])
			dd->addr->chg_cmd[1]++;
		break;

	case 37:
		dd->addr->dbg[0] = entry[1];
		dd->addr->dbg[1] = entry[2];
		break;

	default:
		break;
	}

	return;
}

static int atmxt_set_useful_data(struct atmxt_driver_data *dd)
{
	int err = 0;
	int i = 0;
	struct atmxt_obj *obj = NULL;
	uint8_t *entry = NULL;

	entry = atmxt_get_entry(dd, 5);
	if (entry == NULL) {
		printk(KERN_ERR "%s: Message processor is missing.\n",
			__func__);
		err = -ENODATA;
		goto atmxt_set_useful_data_fail;
	}

	dd->data->max_msg_size = entry[3];

	obj = atmxt_get_obj(dd, 7);
	if (obj == NULL) {
		printk(KERN_ERR "%s: Power object is missing.\n", __func__);
		err = -ENOENT;
		goto atmxt_set_useful_data_fail;
	} else if (obj->size < 2) {
		printk(KERN_ERR "%s: Power object size is too small.\n",
			__func__);
		err = -ENODATA;
		goto atmxt_set_useful_data_fail;
	}

	dd->data->pwr[0] = obj->data[0];
	dd->data->pwr[1] = obj->data[1];

	obj = atmxt_get_obj(dd, 8);
	if (obj == NULL) {
		printk(KERN_ERR "%s: Acquisition object is missing.\n",
			__func__);
		err = -ENOENT;
		goto atmxt_set_useful_data_fail;
	} else if (obj->size < 10) {
		printk(KERN_ERR
			"%s: Acquisition object size is too small.\n",
			__func__);
		err = -ENODATA;
		goto atmxt_set_useful_data_fail;
	}

	dd->data->acq[0] = obj->data[4];
	dd->data->acq[1] = obj->data[5];
	dd->data->acq[2] = obj->data[6];
	dd->data->acq[3] = obj->data[7];
	dd->data->acq[4] = obj->data[8];
	dd->data->acq[5] = obj->data[9];

	dd->data->res[0] = false;
	dd->data->res[1] = false;

	obj = atmxt_get_obj(dd, 9);
	if (obj == NULL) {
		printk(KERN_ERR "%s: Touch object is missing.\n", __func__);
		err = -ENOENT;
		goto atmxt_set_useful_data_fail;
	} else if (obj->size < 22) {
		atmxt_dbg(dd, ATMXT_DBG3,
			"%s: Only 10-bit resolution is available.\n", __func__);
	} else {
		if (obj->data[19] >= 0x04)
			dd->data->res[0] = true;

		if (obj->data[21] >= 0x04)
			dd->data->res[1] = true;
	}

	if (obj->size < 5) {
		printk(KERN_ERR "%s: Touch object size is too small.\n",
			__func__);
		err = -ENODATA;
		goto atmxt_set_useful_data_fail;
	}

	dd->data->xysize[0] = obj->data[3];
	dd->data->xysize[1] = obj->data[4];

	switch (dd->info_blk->header[0]) {
	case 0xA0:
		atmxt_dbg(dd, ATMXT_DBG3, "%s: IC has invalid family ID.\n",
			__func__);
		dd->data->max_x = 0;
		break;
	case 0x82:
		dd->data->max_x = 24;
		if (dd->data->xysize[0] > 24) {
			printk(KERN_ERR "%s: X size of %hu exceeds 24.\n",
				__func__, dd->data->xysize[0]);
			dd->data->xysize[0] = 24;
		}
		if (dd->data->xysize[1] > 14) {
			printk(KERN_ERR "%s: Y size of %hu exceeds 14.\n",
				__func__, dd->data->xysize[1]);
			dd->data->xysize[1] = 14;
		}
		break;
	default:
		dd->data->max_x = 20;
		if (dd->data->xysize[0] > 20) {
			printk(KERN_ERR "%s: X size of %hu exceeds 20.\n",
				__func__, dd->data->xysize[0]);
			dd->data->xysize[0] = 20;
		}
		if (dd->data->xysize[1] > 14) {
			printk(KERN_ERR "%s: Y size of %hu exceeds 14.\n",
				__func__, dd->data->xysize[1]);
			dd->data->xysize[1] = 14;
		}
		break;
	}

	for (i = 1; i < dd->info_blk->id_size; i++) {
		if (dd->info_blk->msg_id[i] == 9) {
			dd->data->touch_id_offset = i;
			break;
		}
	}

	if (dd->data->touch_id_offset == 0) {
		printk(KERN_ERR "%s: Touch object has reporting error.\n",
			__func__);
		err = -ENODATA;
		goto atmxt_set_useful_data_fail;
	}

atmxt_set_useful_data_fail:
	return err;
}

static void atmxt_compute_checksum(struct atmxt_driver_data *dd)
{
	uint8_t low = 0;
	uint8_t mid = 0;
	uint8_t high = 0;
	struct atmxt_obj *obj;
	struct atmxt_obj *inst;
	uint8_t byte1 = 0;
	uint8_t byte2 = 0;
	bool need_next_byte = false;
	uint8_t iter = 0;

	obj = dd->objs;

	while (obj != NULL) {
		if (need_next_byte) {
			byte2 = obj->data[iter];
			iter++;
		} else {
			byte1 = obj->data[iter];
			iter++;
			if (iter >= obj->size) {
				need_next_byte = true;
				inst = obj->next_inst;
				iter = 0;

				while (inst != NULL) {
					if (need_next_byte) {
						byte2 = inst->data[iter];
						iter++;
					} else {
						byte1 = inst->data[iter];
						iter++;
						if (iter >= inst->size) {
							need_next_byte = true;
							inst = obj->next_inst;
							iter = 0;
							continue;
						} else {
						    byte2 = inst->data[iter];
						    iter++;
						}
					}

					atmxt_compute_partial_checksum(
						&byte1, &byte2,
						&low, &mid, &high);
					need_next_byte = false;
					if (iter >= inst->size) {
						inst = inst->next_inst;
						iter = 0;
					}
				}

				if (need_next_byte || (iter == 0)) {
					obj = obj->next_obj;
					iter = 0;
					continue;
				}
			} else {
				byte2 = obj->data[iter];
				iter++;
			}
		}

		atmxt_compute_partial_checksum(&byte1, &byte2,
			&low, &mid, &high);
		need_next_byte = false;
		if (iter >= obj->size) {
			obj = obj->next_obj;
			iter = 0;
		}
	}

	if (need_next_byte) {
		byte2 = 0;
		atmxt_compute_partial_checksum(&byte1, &byte2,
			&low, &mid, &high);
	}

	dd->info_blk->obj_chksum[0] = low;
	dd->info_blk->obj_chksum[1] = mid;
	dd->info_blk->obj_chksum[2] = high;

	return;
}

static void atmxt_compute_partial_checksum(uint8_t *byte1, uint8_t *byte2,
		uint8_t *low, uint8_t *mid, uint8_t *high)
{
	bool xor_result = false;

	if (*high & 0x80)
		xor_result = true;

	*high = *high << 1;
	if (*mid & 0x80)
		(*high)++;

	*mid = *mid << 1;
	if (*low & 0x80)
		(*mid)++;

	*low = *low << 1;

	*low = *low ^ *byte1;
	*mid = *mid ^ *byte2;

	if (xor_result) {
		*low = *low ^ 0x1B;
		*high = *high ^ 0x80;
	}

	return;
}

static void atmxt_active_handler(struct atmxt_driver_data *dd)
{
	int err = 0;
	int i = 0;
	uint8_t *msg_buf = NULL;
	int size = 0;
	char *contents = NULL;
	bool msg_fail = false;
	int last_err = 0;
	int msg_size = 0;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Starting active handler...\n", __func__);

	msg_size = dd->data->max_msg_size;
	size = (dd->rdat->active_touches + 1) * msg_size;
	msg_buf = kzalloc(sizeof(uint8_t) * size, GFP_KERNEL);
	if (msg_buf == NULL) {
		printk(KERN_ERR
			"%s: Unable to allocate memory for message buffer.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_active_handler_fail;
	}

	err = atmxt_i2c_write(dd, dd->addr->msg[0], dd->addr->msg[1], NULL, 0);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to set message buffer pointer.\n",
			__func__);
		goto atmxt_active_handler_fail;
	}

	err = atmxt_i2c_read(dd, msg_buf, size);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to read messages.\n", __func__);
		goto atmxt_active_handler_fail;
	}

	if (msg_buf[0] == 0xFF) {
		contents = atmxt_msg2str(msg_buf, size);
		printk(KERN_ERR "%s: Received invalid data:  %s.\n",
			__func__, contents);
		err = -EINVAL;
		goto atmxt_active_handler_fail;
	}

	for (i = 0; i < size; i += msg_size) {
		if (msg_buf[i] == 0xFF) {
			atmxt_dbg(dd, ATMXT_DBG3, "%s: Finished processing.\n",
				__func__);
			break;
		}

		atmxt_dbg(dd, ATMXT_DBG3,
			"%s: Processing message %d...\n", __func__,
			(i + 1) / msg_size);
		err = atmxt_process_message(dd, &(msg_buf[i]), msg_size);
		if (err < 0) {
			printk(KERN_ERR
				"%s: Processing message %d failed %s %d.\n",
				__func__, (i / msg_size) + 1,
				"with error code", err);
			msg_fail = true;
			last_err = err;
		}
	}

	if (dd->status & (1 << ATMXT_FIXING_CALIBRATION)) {
		err = atmxt_verify_ic_calibration_fix(dd);
		if (err < 0) {
			printk(KERN_ERR "%s: Unable to verify IC calibration.\n",
				__func__);
			goto atmxt_active_handler_fail;
		}
	}

	if (dd->status & (1 << ATMXT_REPORT_TOUCHES)) {
		atmxt_report_touches(dd);
		dd->status = dd->status & ~(1 << ATMXT_REPORT_TOUCHES);
	}

	if (msg_fail) {
		err = last_err;
		goto atmxt_active_handler_fail;
	}

	goto atmxt_active_handler_pass;

atmxt_active_handler_fail:
	printk(KERN_ERR "%s: Touch active handler failed with error code %d.\n",
		__func__, err);

atmxt_active_handler_pass:
	kfree(msg_buf);
	kfree(contents);
	return;
}

static struct atmxt_obj *atmxt_get_obj(
		struct atmxt_driver_data *dd, uint8_t num)
{
	struct atmxt_obj *obj;

	obj = dd->objs;

	while (obj != NULL) {
		if (obj->num == num)
			break;
		else
			obj = obj->next_obj;
	}

	return obj;
}

static uint8_t *atmxt_get_entry(struct atmxt_driver_data *dd, uint8_t num)
{
	uint8_t *entry = NULL;
	int i = 0;

	for (i = 0; i < dd->info_blk->size; i += 6) {
		if (dd->info_blk->data[i+0] == num) {
			entry = &(dd->info_blk->data[i]);
			goto atmxt_get_entry_exit;
		}
	}

atmxt_get_entry_exit:
	return entry;
}

static int atmxt_process_message(struct atmxt_driver_data *dd,
		uint8_t *msg, uint8_t size)
{
	int err = 0;
	char *contents = NULL;

	if (msg[0] <= (dd->info_blk->id_size-1)) {
		switch (dd->info_blk->msg_id[msg[0]]) {
		case 6:
			err = atmxt_message_handler6(dd, msg, size);
			break;
		case 9:
			err = atmxt_message_handler9(dd, msg, size);
			break;
		case 42:
			err = atmxt_message_handler42(dd, msg, size);
		default:
			contents = atmxt_msg2str(msg, size);
			printk(KERN_ERR "%s: Object %u sent this:  %s.\n",
				__func__, dd->info_blk->msg_id[msg[0]],
				contents);
			break;
		}
	} else {
		contents = atmxt_msg2str(msg, size);
		printk(KERN_ERR "%s: Received unknown message:  %s.\n",
			__func__, contents);
	}

	if (err < 0)
		printk(KERN_ERR "%s: Message processing failed.\n", __func__);

	kfree(contents);
	return err;
}

static void atmxt_report_touches(struct atmxt_driver_data *dd)
{
	int i = 0;
	int j = 0;
	int rval = 0;
	int id = 0;
	int x = 0;
	int y = 0;
	int p = 0;
	int w = 0;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	struct timespec uptime;
#endif

	dd->rdat->active_touches = 0;

	for (i = 0; i < ATMXT_MAX_TOUCHES; i++) {
		if (!(dd->rdat->tchdat[i].active))
			continue;

		id =  dd->rdat->tchdat[i].id;
		x = dd->rdat->tchdat[i].x;
		y = dd->rdat->tchdat[i].y;
		p = dd->rdat->tchdat[i].p;
		w = dd->rdat->tchdat[i].w;

		dd->rdat->active_touches++;

		atmxt_dbg(dd, ATMXT_DBG1, "%s: ID=%d, X=%d, Y=%d, P=%d, W=%d\n",
			__func__, id, x, y, p, w);

		for (j = 0; j < ARRAY_SIZE(dd->rdat->axis); j++) {
			switch (j) {
			case 0:
				rval = x;
				break;
			case 1:
				rval = y;
				break;
			case 2:
				rval = p;
				break;
			case 3:
				rval = w;
				break;
			case 4:
				rval = id;
				break;
			}
			if (dd->rdat->axis[j] != ATMXT_ABS_RESERVED) {
				input_report_abs(dd->in_dev,
					dd->rdat->axis[j], rval);
			}
		}
		input_mt_sync(dd->in_dev);
#ifdef CONFIG_TOUCHSCREEN_DEBUG
		do_posix_clock_monotonic_gettime(&uptime);
		monotonic_to_bootbased(&uptime);
		dd->dbg->evt_sec = uptime.tv_sec;
		dd->dbg->evt_ms = uptime.tv_nsec/(NSEC_PER_SEC/100);
		dd->dbg->evt_count++;
		if (dd->rdat->tchdat[i].active)
			dd->dbg->evt_act = dd->rdat->active_touches;
#endif
	}

	if (dd->rdat->active_touches == 0)
		input_mt_sync(dd->in_dev);

	input_sync(dd->in_dev);

	return;
}

static void atmxt_release_touches(struct atmxt_driver_data *dd)
{
	int i = 0;

	atmxt_dbg(dd, ATMXT_DBG1, "%s: Releasing all touches...\n", __func__);

	for (i = 0; i < ATMXT_MAX_TOUCHES; i++)
		dd->rdat->tchdat[i].active = false;

	atmxt_report_touches(dd);

	return;
}

static int atmxt_message_handler6(struct atmxt_driver_data *dd,
		uint8_t *msg, uint8_t size)
{
	int err = 0;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Handling message type 6...\n", __func__);

	if (size < 5) {
		printk(KERN_ERR "%s: Message size is too small.\n", __func__);
		err = -EINVAL;
		goto atmxt_message_handler6_fail;
	}

	if (msg[1] & 0x80) {
		printk(KERN_INFO "%s: Touch IC reset complete.\n", __func__);
		dd->data->last_stat = 0x00;
	}

	if ((msg[1] & 0x40) && !(dd->data->last_stat & 0x40)) {
		printk(KERN_ERR "%s: Acquisition cycle overflow.\n", __func__);
	} else if (!(msg[1] & 0x40) && (dd->data->last_stat & 0x40)) {
		printk(KERN_INFO "%s: Acquisition cycle now normal.\n",
			__func__);
	}

	if ((msg[1] & 0x20) && !(dd->data->last_stat & 0x20)) {
		printk(KERN_ERR "%s: Signal error in IC acquisition.\n",
			__func__);
	} else if (!(msg[1] & 0x20) && (dd->data->last_stat & 0x20)) {
		printk(KERN_INFO "%s: IC acquisition signal now in range.\n",
			__func__);
	}

	if ((msg[1] & 0x10) && !(dd->data->last_stat & 0x10)) {
		printk(KERN_INFO "%s: Touch IC is calibrating.\n", __func__);
		dd->status = dd->status | (1 << ATMXT_RECEIVED_CALIBRATION);
	} else if (!(msg[1] & 0x10) && (dd->data->last_stat & 0x10)) {
		printk(KERN_INFO "%s: Touch IC calibration complete.\n",
			__func__);
		dd->status = dd->status | (1 << ATMXT_RECEIVED_CALIBRATION);
	}

	if (msg[1] & 0x08) {
		printk(KERN_ERR "%s: Hardware configuration error--%s.\n",
			__func__, "check platform settings");
		dd->data->last_stat = dd->data->last_stat & 0xF7;
	} else if (!(msg[1] & 0x08) && (dd->data->last_stat & 0x08)) {
		printk(KERN_INFO
			"%s: Hardware configuration error corrected.\n",
			__func__);
	}

	if (msg[1] & 0x04) {
		printk(KERN_ERR "%s: IC reports I2C communication error.\n",
			__func__);
	}

	if (msg[1] == dd->data->last_stat) {
		printk(KERN_INFO "%s: Received checksum 0x%02X%02X%02X.\n",
			__func__, msg[2], msg[3], msg[4]);
	}

	dd->data->last_stat = msg[1];

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (dd->status & (1 << ATMXT_IGNORE_CHECKSUM))
		goto atmxt_message_handler6_fail;
#endif

	if ((msg[2] != dd->info_blk->obj_chksum[0]) ||
		(msg[3] != dd->info_blk->obj_chksum[1]) ||
		(msg[4] != dd->info_blk->obj_chksum[2])) {
		if (!(dd->status & (1 << ATMXT_CHECKSUM_FAILED))) {
			printk(KERN_ERR "%s: IC settings checksum fail.  %s\n",
				__func__, "Restarting the IC...");
			err = atmxt_resume_restart(dd);
			if (err < 0) {
				printk(KERN_ERR
					"%s: Failed to restart the touch IC.\n",
					__func__);
				goto atmxt_message_handler6_fail;
			}

			dd->status = dd->status | (1 << ATMXT_CHECKSUM_FAILED);
		} else {
			printk(KERN_ERR "%s: IC settings checksum fail.  %s\n",
				__func__, "Sending settings (no backup)...");
			err = atmxt_send_settings(dd, false);
			if (err < 0) {
				printk(KERN_ERR
					"%s: Failed to update IC settings.\n",
					__func__);
				goto atmxt_message_handler6_fail;
			}
		}
	}

atmxt_message_handler6_fail:
	return err;
}

static int atmxt_message_handler9(struct atmxt_driver_data *dd,
		uint8_t *msg, uint8_t size)
{
	int err = 0;
	uint8_t tchidx = 0;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Handling message type 9...\n", __func__);

	if (size < 7) {
		printk(KERN_ERR "%s: Message size is too small.\n", __func__);
		err = -EINVAL;
		goto atmxt_message_handler9_fail;
	}

	tchidx = msg[0] - dd->data->touch_id_offset;
	if (tchidx >= ARRAY_SIZE(dd->rdat->tchdat)) {
		printk(KERN_ERR "%s: Touch %hu is unsupported.\n",
			__func__, tchidx);
		err = -EOVERFLOW;
		goto atmxt_message_handler9_fail;
	}

	dd->status = dd->status | (1 << ATMXT_REPORT_TOUCHES);

	dd->rdat->tchdat[tchidx].id = tchidx;

	dd->rdat->tchdat[tchidx].x = (msg[2] << 4) | ((msg[4] & 0xF0) >> 4);
	if (!(dd->data->res[0]))
		dd->rdat->tchdat[tchidx].x = dd->rdat->tchdat[tchidx].x >> 2;

	dd->rdat->tchdat[tchidx].y = (msg[3] << 4) | (msg[4] & 0x0F);
	if (!(dd->data->res[1]))
		dd->rdat->tchdat[tchidx].y = dd->rdat->tchdat[tchidx].y >> 2;

	dd->rdat->tchdat[tchidx].p = msg[6];
	dd->rdat->tchdat[tchidx].w = msg[5];

	if (((msg[1] & 0x40) && (msg[1] & 0x20)) ||
		((msg[1] & 0x40) && (msg[1] & 0x02)) ||
		((msg[1] & 0x20) && (msg[1] & 0x02))) {
		printk(KERN_ERR "%s: System too slow %s 0x%02X.\n",
			__func__, "to see all touch events for report id",
			msg[0]);
	}

	if (msg[1] & 0x22) {
		atmxt_dbg(dd, ATMXT_DBG1, "%s: Touch ID %hu released.\n",
			__func__, tchidx);
		dd->rdat->tchdat[tchidx].active = false;
	} else {
		dd->rdat->tchdat[tchidx].active = true;
	}

	if (msg[1] & 0x02) {
		printk(KERN_INFO "%s: Touch ID %hu suppressed.\n",
			__func__, tchidx);
	}

atmxt_message_handler9_fail:
	return err;
}

static int atmxt_message_handler42(struct atmxt_driver_data *dd,
		uint8_t *msg, uint8_t size)
{
	int err = 0;

	atmxt_dbg(dd, ATMXT_DBG3,
		"%s: Handling message type 42...\n", __func__);

	if (size < 2) {
		printk(KERN_ERR "%s: Message size is too small.\n", __func__);
		err = -EINVAL;
		goto atmxt_message_handler42_fail;
	}

	if (msg[1] & 0x01) {
		printk(KERN_ERR "%s: Touch suppression is active.\n",
			__func__);
	} else {
		printk(KERN_INFO "%s: Touch suppression is disabled.\n",
			__func__);
	}

atmxt_message_handler42_fail:
	return err;
}

static int atmxt_resume_restart(struct atmxt_driver_data *dd)
{
	int err = 0;

	atmxt_dbg(dd, ATMXT_DBG3, "%s: Resume restarting IC...\n", __func__);

	if (dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG)) {
		disable_irq_nosync(dd->client->irq);
		dd->status = dd->status & ~(1 << ATMXT_IRQ_ENABLED_FLAG);
	}

	err = atmxt_restart_ic(dd, dd->pdata->fw, false);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to restart the touch IC.\n",
			__func__);
		atmxt_set_drv_state(dd, ATMXT_DRV_IDLE);
		goto atmxt_resume_restart_fail;
	}

	if (!(dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG))) {
		if (atmxt_get_drv_state(dd) != ATMXT_DRV_ACTIVE)
			atmxt_set_drv_state(dd, ATMXT_DRV_ACTIVE);
		dd->status = dd->status | (1 << ATMXT_IRQ_ENABLED_FLAG);
		enable_irq(dd->client->irq);
	}

atmxt_resume_restart_fail:
	return err;
}

static int atmxt_force_bootloader(struct atmxt_driver_data *dd)
{
	int err = 0;
	int i = 0;
	uint8_t cmd = 0x00;
	bool chg_used = false;

	if (dd->addr == NULL) {
		atmxt_dbg(dd, ATMXT_DBG3, "%s: No address data available.\n",
			__func__);
		goto atmxt_force_bootloader_use_recov;
	}

	if ((dd->addr->chg_cmd[0] == 0) && (dd->addr->chg_cmd[1] == 0)) {
		atmxt_dbg(dd, ATMXT_DBG3,
			"%s: No interrupt force available--%s.\n",
			__func__, "will pool instead");
		goto atmxt_force_bootloader_check_reset;
	}

	cmd = 0x02;
	err = atmxt_i2c_write(dd,
		dd->addr->chg_cmd[0], dd->addr->chg_cmd[1], &cmd, 1);
	if (err < 0) {
		printk(KERN_ERR "%s: Unable to force interrupt low--%s.\n",
			__func__, "will poll instead");
	} else {
		chg_used = true;
	}

atmxt_force_bootloader_check_reset:
	if ((dd->addr->rst[0] == 0) && (dd->addr->rst[1] == 0)) {
		atmxt_dbg(dd, ATMXT_DBG3,
			"%s: No soft reset available--%s.\n", __func__,
			"will try hardware recovery instead");
		goto atmxt_force_bootloader_use_recov;
	}

	cmd = 0xA5;
	err = atmxt_i2c_write(dd, dd->addr->rst[0], dd->addr->rst[1], &cmd, 1);
	if (err < 0) {
		printk(KERN_ERR "%s: Unable to send flash reset command.\n",
			__func__);
		goto atmxt_force_bootloader_use_recov;
	}

	if (!chg_used) {
		for (i = 0; i < 128; i++) {
			if (dd->pdata->irq_stat() == 1)
				break;
			else
				udelay(2000);
		}

		if (i == 128) {
			printk(KERN_ERR "%s: %s.\n", __func__,
				"Waiting for flash reset timed out");
			err = -ETIME;
			goto atmxt_force_bootloader_exit;
		}
	}

	goto atmxt_force_bootloader_exit;

atmxt_force_bootloader_use_recov:
	atmxt_dbg(dd, ATMXT_DBG2, "%s: Using hardware recovery...\n", __func__);
	err = dd->pdata->hw_recov(0);
	if (err < 0) {
		printk(KERN_ERR "%s: Forced hardware recovery failed--%s.\n",
			__func__, "unable to reflash IC");
		goto atmxt_force_bootloader_exit;
	}

atmxt_force_bootloader_exit:
	return err;
}

static bool atmxt_check_firmware_upgrade(struct atmxt_driver_data *dd,
		struct touch_firmware *fw)
{
	bool upgrade_fw = false;

	if (fw == NULL) {
		printk(KERN_ERR
			"%s: Platform firmware structure not present.  %s.\n",
			__func__, "Unable to check for firmware upgrade.");
		goto atmxt_check_firmware_upgrade_exit;
	}

	if ((fw->vsize < 4) || (fw->ver == NULL)) {
		printk(KERN_ERR
			"%s: Platform firmware version info is invalid.  %s\n",
			__func__, "Unable to check for firmware upgrade.");
		goto atmxt_check_firmware_upgrade_exit;
	}

	if ((fw->ver[0] != dd->info_blk->header[0]) &&
		(fw->ver[1] != dd->info_blk->header[1])) {
		printk(KERN_ERR
			"%s: Platform firmware does not match touch IC.  %s\n",
			__func__, "Unable to check for firmware upgrade.");
		goto atmxt_check_firmware_upgrade_exit;
	}

	if (fw->ver[2] > dd->info_blk->header[2]) {
		upgrade_fw = true;
	} else {
		if ((fw->ver[2] == dd->info_blk->header[2]) &&
			(fw->ver[3] > dd->info_blk->header[3])) {
			upgrade_fw = true;
		} else {
			upgrade_fw = false;
		}
	}

atmxt_check_firmware_upgrade_exit:
	return upgrade_fw;
}

static int atmxt_validate_firmware(const uint8_t *img, uint32_t size)
{
	int err = 0;
	uint32_t iter = 0;
	int length = 0;

	if (img == NULL || size == 0) {
		printk(KERN_ERR "%s: No firmware image found.\n", __func__);
		err = -ENODATA;
		goto atmxt_validate_firmware_fail;
	}

	while (iter < (size - 1)) {
		length = (img[iter+0] << 8) | img[iter+1];
		if ((iter + length + 2) > size) {
			printk(KERN_ERR
				"%s: Overflow in firmware image %s %u.\n",
				__func__, "on iter", iter);
			err = -EOVERFLOW;
			goto atmxt_validate_firmware_fail;
		}

		iter = iter + length + 2;
	}

	if (iter != size) {
		printk(KERN_ERR "%s: Firmware image misaligned.\n", __func__);
		err = -ENOEXEC;
		goto atmxt_validate_firmware_fail;
	}

atmxt_validate_firmware_fail:
	return err;
}

static int atmxt_flash_firmware(struct atmxt_driver_data *dd,
		const uint8_t *img, uint32_t size)
{
	int err = 0;
	uint32_t iter = 0;
	uint8_t status;
	bool irq_low = false;
	bool frame_crc_failed = false;

	printk(KERN_INFO "%s: Reflashing touch IC...\n", __func__);

	err = atmxt_i2c_write(dd, 0xDC, 0xAA, NULL, 0);
	if (err < 0) {
		printk(KERN_ERR "%s: Unable to send unlock command.\n",
			__func__);
		goto atmxt_flash_firmware_fail;
	}

	while (iter < (size - 1)) {
		irq_low = atmxt_wait4irq(dd);
		if (!irq_low) {
			printk(KERN_ERR
				"%s: Timeout waiting %s for iter %u.\n",
				__func__, "for frame interrupt", iter);
			err = -ETIME;
			goto atmxt_flash_firmware_fail;
		}

		err = atmxt_i2c_read(dd, &status, 1);
		if (err < 0) {
			printk(KERN_ERR
				"%s: Error reading frame byte for iter %u.\n",
				__func__, iter);
			goto atmxt_flash_firmware_fail;
		}

		err = atmxt_i2c_write(dd, img[iter+0], img[iter+1],
			&(img[iter+2]), (img[iter+0] << 8) | img[iter+1]);
		if (err < 0) {
			printk(KERN_ERR "%s: Error sending frame iter %u.\n",
				__func__, iter);
			goto atmxt_flash_firmware_fail;
		}

		irq_low = atmxt_wait4irq(dd);
		if (!irq_low) {
			printk(KERN_ERR
				"%s: Timeout waiting %s for iter %u.\n",
				__func__, "for check interrupt", iter);
			err = -ETIME;
			goto atmxt_flash_firmware_fail;
		}

		err = atmxt_i2c_read(dd, &status, 1);
		if (err < 0) {
			printk(KERN_ERR
				"%s: Error reading check byte for iter %u.\n",
				__func__, iter);
			goto atmxt_flash_firmware_fail;
		}

		if (status != 0x02) {
			printk(KERN_ERR "%s: %s 0x%02X %s %u.\n", __func__,
				"Unexpected status", status,
				"received for iter", iter);
			err = -EPROTO;
			goto atmxt_flash_firmware_fail;
		}

		irq_low = atmxt_wait4irq(dd);
		if (!irq_low) {
			printk(KERN_ERR
				"%s: Timeout waiting %s for iter %u.\n",
				__func__, "for result interrupt", iter);
			err = -ETIME;
			goto atmxt_flash_firmware_fail;
		}

		err = atmxt_i2c_read(dd, &status, 1);
		if (err < 0) {
			printk(KERN_ERR
				"%s: Error reading result byte for iter %u.\n",
				__func__, iter);
			goto atmxt_flash_firmware_fail;
		}

		if (status == 0x04) {
			iter = iter + ((img[iter+0] << 8) | img[iter+1]) + 2;
			frame_crc_failed = false;
		} else if (!frame_crc_failed) {
			printk(KERN_ERR "%s: %s %u--%s.\n",
				__func__, "Frame CRC failed for iter",
				iter, "will try to re-send");
			frame_crc_failed = true;
		} else {
			printk(KERN_ERR "%s: %s %u--%s.\n",
				__func__, "Frame CRC failed for iter",
				iter, "check firmware image");
			err = -ECOMM;
			goto atmxt_flash_firmware_fail;
		}
	}

atmxt_flash_firmware_fail:
	return err;
}

static char *atmxt_msg2str(const uint8_t *msg, uint8_t size)
{
	char *str = NULL;
	int i = 0;
	int err = 0;

	str = kzalloc(sizeof(char) * (size * 5), GFP_KERNEL);
	if (str == NULL) {
		printk(KERN_ERR "%s: Failed to allocate message string.\n",
			__func__);
		goto atmxt_msg2str_exit;
	}

	for (i = 0; i < size; i++) {
		err = sprintf(str, "%s0x%02X ", str, msg[i]);
		if (err < 0) {
			printk(KERN_ERR "%s: Error in sprintf on pass %d",
				__func__, i);
			goto atmxt_msg2str_exit;
		}
	}

	str[err-1] = '\0';

atmxt_msg2str_exit:
	return str;
}

static bool atmxt_wait4irq(struct atmxt_driver_data *dd)
{
	bool irq_low = false;
	int i = 0;

	for (i = 0; i < 500; i++) {
		if (dd->pdata->irq_stat() != 0) {
			msleep(20);
		} else {
			irq_low = true;
			break;
		}
	}

	return irq_low;
}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
static ssize_t atmxt_debug_drv_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	return sprintf(buf, "Current Debug Level: %hu\n", dd->dbg->dbg_lvl);
}
static ssize_t atmxt_debug_drv_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);
	unsigned long value = 0;

	mutex_lock(dd->mutex);

	err = strict_strtoul(buf, 10, &value);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to convert value.\n", __func__);
		goto atmxt_debug_drv_debug_store_exit;
	}

	if (value > 255) {
		printk(KERN_ERR "%s: Invalid debug level %lu--setting to %u.\n",
			__func__, value, ATMXT_DBG3);
		dd->dbg->dbg_lvl = ATMXT_DBG3;
	} else {
		dd->dbg->dbg_lvl = value;
		printk(KERN_INFO "%s: Debug level is now %hu.\n",
			__func__, dd->dbg->dbg_lvl);
	}

	err = size;

atmxt_debug_drv_debug_store_exit:
	mutex_unlock(dd->mutex);

	return err;
}
static DEVICE_ATTR(drv_debug, S_IRUSR | S_IWUSR,
	atmxt_debug_drv_debug_show, atmxt_debug_drv_debug_store);

static ssize_t atmxt_debug_drv_flags_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	return sprintf(buf, "Current Driver Flags: 0x%04X\n", dd->settings);
}
static ssize_t atmxt_debug_drv_flags_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);
	unsigned long value = 0;

	mutex_lock(dd->mutex);

	err = strict_strtoul(buf, 16, &value);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to convert value.\n", __func__);
		goto atmxt_debug_drv_flags_store_exit;
	}

	if (value > 65535) {
		printk(KERN_ERR "%s: Invalid flag settings 0x%08lX passed.\n",
			__func__, value);
		err = -EOVERFLOW;
		goto atmxt_debug_drv_flags_store_exit;
	} else {
		dd->settings = value;
		atmxt_dbg(dd, ATMXT_DBG3,
			"%s: Driver flags now set to 0x%04X.\n",
			__func__, dd->settings);
	}

	err = size;

atmxt_debug_drv_flags_store_exit:
	mutex_unlock(dd->mutex);

	return err;
}
static DEVICE_ATTR(drv_flags, S_IRUSR | S_IWUSR,
	atmxt_debug_drv_flags_show, atmxt_debug_drv_flags_store);
#endif

static ssize_t atmxt_debug_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	if (dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG))
		return sprintf(buf, "Driver interrupt is ENABLED.\n");
	else
		return sprintf(buf, "Driver interrupt is DISABLED.\n");
}
static ssize_t atmxt_debug_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);
	unsigned long value = 0;

	mutex_lock(dd->mutex);

	err = strict_strtoul(buf, 10, &value);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to convert value.\n", __func__);
		goto atmxt_debug_drv_irq_store_exit;
	}

	if ((atmxt_get_drv_state(dd) != ATMXT_DRV_ACTIVE) &&
		(atmxt_get_drv_state(dd) != ATMXT_DRV_IDLE)) {
		printk(KERN_ERR "%s: %s %s or %s states.\n",
			__func__, "Interrupt can be changed only in",
			atmxt_driver_state_string[ATMXT_DRV_ACTIVE],
			atmxt_driver_state_string[ATMXT_DRV_IDLE]);
		err = -EACCES;
		goto atmxt_debug_drv_irq_store_exit;
	}

	switch (value) {
	case 0:
		if (dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG)) {
			disable_irq_nosync(dd->client->irq);
			atmxt_set_drv_state(dd, ATMXT_DRV_IDLE);
			dd->status =
				dd->status & ~(1 << ATMXT_IRQ_ENABLED_FLAG);
		}
		break;

	case 1:
		if (!(dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG))) {
			dd->status =
				dd->status | (1 << ATMXT_IRQ_ENABLED_FLAG);
			enable_irq(dd->client->irq);
			atmxt_set_drv_state(dd, ATMXT_DRV_ACTIVE);
		}
		break;

	default:
		printk(KERN_ERR "%s: Invalid value passed.\n", __func__);
		err = -EINVAL;
		goto atmxt_debug_drv_irq_store_exit;
	}

	err = size;

atmxt_debug_drv_irq_store_exit:
	mutex_unlock(dd->mutex);

	return err;
}
static DEVICE_ATTR(drv_irq, S_IRUSR | S_IWUSR,
	atmxt_debug_drv_irq_show, atmxt_debug_drv_irq_store);

static ssize_t atmxt_debug_driver_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	return sprintf(buf, "Driver state is %s.\nIC state is %s.\n",
		atmxt_driver_state_string[atmxt_get_drv_state(dd)],
		atmxt_ic_state_string[atmxt_get_ic_state(dd)]);
}
static DEVICE_ATTR(drv_stat, S_IRUGO, atmxt_debug_driver_stat_show, NULL);

static ssize_t atmxt_debug_driver_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Driver: %s\nVersion: %s\nDate: %s\n",
		ATMXT_I2C_NAME, ATMXT_DRIVER_VERSION, ATMXT_DRIVER_DATE);
}
static DEVICE_ATTR(drv_ver, S_IRUGO, atmxt_debug_driver_ver_show, NULL);

static ssize_t atmxt_debug_hw_irqstat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	err = dd->pdata->irq_stat();
	if (err < 0) {
		printk(KERN_ERR
			"%s: Function irq_stat() failed with error code %d.\n",
			__func__, err);
		err = sprintf(buf,
			"Function irq_stat() failed with error code %d.\n",
			err);
		goto atmxt_debug_hw_irqstat_show_exit;
	}

	switch (err) {
	case 0:
		err = sprintf(buf, "Interrupt line is LOW.\n");
		break;
	case 1:
		err = sprintf(buf, "Interrupt line is HIGH.\n");
		break;
	default:
		err = sprintf(buf, "Function irq_stat() returned %d.\n", err);
		break;
	}

atmxt_debug_hw_irqstat_show_exit:
	return err;
}
static DEVICE_ATTR(hw_irqstat, S_IRUSR, atmxt_debug_hw_irqstat_show, NULL);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
static ssize_t atmxt_debug_hw_recov_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);
	unsigned long value = 0;

	mutex_lock(dd->mutex);

	err = strict_strtoul(buf, 10, &value);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to convert value.\n", __func__);
		goto atmxt_debug_hw_recov_store_fail;
	}

	err = dd->pdata->hw_recov(value);
	if (err < 0) {
		printk(KERN_ERR
			"%s: Hardware recovery failed with error code %d.\n",
			__func__, err);
		goto atmxt_debug_hw_recov_store_fail;
	}

	err = size;

atmxt_debug_hw_recov_store_fail:
	mutex_unlock(dd->mutex);
	return err;
}
static DEVICE_ATTR(hw_recov, S_IWUSR, NULL, atmxt_debug_hw_recov_store);

static ssize_t atmxt_debug_hw_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	mutex_lock(dd->mutex);

	if (dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG)) {
		disable_irq_nosync(dd->client->irq);
		dd->status = dd->status & ~(1 << ATMXT_IRQ_ENABLED_FLAG);
	}

	err = atmxt_restart_ic(dd, dd->pdata->fw, false);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to %s with error code %d.\n",
			__func__, "re-initialize the touch IC", err);
		atmxt_set_drv_state(dd, ATMXT_DRV_IDLE);
		goto atmxt_debug_hw_reset_store_fail;
	}

	if (((atmxt_get_drv_state(dd) == ATMXT_DRV_ACTIVE)) &&
		(!(dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG)))) {
		dd->status = dd->status | (1 << ATMXT_IRQ_ENABLED_FLAG);
		enable_irq(dd->client->irq);
	}

	err = size;

atmxt_debug_hw_reset_store_fail:
	mutex_unlock(dd->mutex);
	return err;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, atmxt_debug_hw_reset_store);

static ssize_t atmxt_debug_ic_group_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);
	int i = 0;
	uint8_t addr_lo = 0x00;
	uint8_t addr_hi = 0x00;
	uint8_t *entry = NULL;
	int size = 0;
	uint8_t *data_in = NULL;

	mutex_lock(dd->mutex);

	if ((dd->info_blk == NULL) || (dd->info_blk->data == NULL)) {
		printk(KERN_ERR "%s: No touch IC data is available.\n",
			__func__);
		err = sprintf(buf, "No touch IC data is available.\n");
		goto atmxt_debug_ic_group_data_show_exit;
	}

	entry = atmxt_get_entry(dd, dd->dbg->grp_num);
	if (entry == NULL) {
		printk(KERN_ERR "%s: Group %hu does not exist.\n",
			__func__, dd->dbg->grp_num);
		err = sprintf(buf, "Group %hu does not exist.\n",
			dd->dbg->grp_num);
			goto atmxt_debug_ic_group_data_show_exit;
	}

	if (dd->dbg->grp_off > entry[3]) {
		printk(KERN_ERR "%s: Offset %hu exceeds group size of %u.\n",
			__func__, dd->dbg->grp_off, entry[3]+1);
		err = sprintf(buf, "Offset %hu exceeds group size of %u.\n",
			dd->dbg->grp_off, entry[3]+1);
		goto atmxt_debug_ic_group_data_show_exit;
	}

	addr_lo = entry[1] + dd->dbg->grp_off;
	addr_hi = entry[2];
	if (addr_lo < entry[1])
		addr_hi++;

	err = atmxt_i2c_write(dd, addr_lo, addr_hi, NULL, 0);
	if (err < 0) {
		printk(KERN_ERR
			"%s: Failed to set group pointer with error code %d.\n",
			__func__, err);
		err = sprintf(buf,
			"Failed to set group pointer with error code %d.\n",
			err);
		goto atmxt_debug_ic_group_data_show_exit;
	}

	size = entry[3] - dd->dbg->grp_off + 1;
	data_in = kzalloc(sizeof(uint8_t) * size, GFP_KERNEL);
	if (data_in == NULL) {
		printk(KERN_ERR "%s: Unable to allocate memory buffer.\n",
			__func__);
		err = sprintf(buf, "Unable to allocate memory buffer.\n");
		goto atmxt_debug_ic_group_data_show_exit;
	}

	err = atmxt_i2c_read(dd, data_in, size);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to read group data.\n", __func__);
		err = sprintf(buf, "Failed to read group data.\n");
		goto atmxt_debug_ic_group_data_show_exit;
	}

	err = sprintf(buf, "Group %hu, Offset %hu:\n",
		dd->dbg->grp_num, dd->dbg->grp_off);
	if (err < 0) {
		printk(KERN_ERR "%s: Error in header sprintf.\n", __func__);
		goto atmxt_debug_ic_group_data_show_exit;
	}

	for (i = 0; i < size; i++) {
		err = sprintf(buf, "%s0x%02hX\n", buf, data_in[i]);
		if (err < 0) {
			printk(KERN_ERR "%s: Error in sprintf loop %d.\n",
				__func__, i);
			goto atmxt_debug_ic_group_data_show_exit;
		}
	}

	err = sprintf(buf, "%s(%u bytes)\n", buf, size);
	if (err < 0) {
		printk(KERN_ERR "%s: Error in byte count sprintf.\n", __func__);
		goto atmxt_debug_ic_group_data_show_exit;
	}

atmxt_debug_ic_group_data_show_exit:
	kfree(data_in);
	mutex_unlock(dd->mutex);

	return (ssize_t) err;
}
static ssize_t atmxt_debug_ic_group_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);
	int i = 0;
	uint8_t addr_lo = 0x00;
	uint8_t addr_hi = 0x00;
	uint8_t *entry = NULL;
	int data_size = 0;
	uint8_t *data_out = NULL;
	unsigned long value = 0;
	uint8_t *conv_buf = NULL;

	mutex_lock(dd->mutex);

	if ((dd->info_blk == NULL) || (dd->info_blk->data == NULL)) {
		printk(KERN_ERR "%s: No touch IC data is available.\n",
			__func__);
		err = -ENODATA;
		goto atmxt_debug_ic_group_data_store_exit;
	}

	entry = atmxt_get_entry(dd, dd->dbg->grp_num);
	if (entry == NULL) {
		printk(KERN_ERR "%s: Group %hu does not exist.\n",
			__func__, dd->dbg->grp_num);
		err = -ENOENT;
		goto atmxt_debug_ic_group_data_store_exit;
	}

	if ((dd->dbg->grp_off) > entry[3]) {
		printk(KERN_ERR "%s: Offset %hu exceeds data size.\n",
			__func__, dd->dbg->grp_off);
		err = -EOVERFLOW;
		goto atmxt_debug_ic_group_data_store_exit;
	}

	if ((size % 5 != 0) || (size == 0)) {
		printk(KERN_ERR "%s: Invalid data format.  %s\n",
			 __func__, "Use \"0xHH,0xHH,...,0xHH\" instead.");
		err = -EINVAL;
		goto atmxt_debug_ic_group_data_store_exit;
	}

	data_out = kzalloc(sizeof(uint8_t) * (size / 5), GFP_KERNEL);
	if (data_out == NULL) {
		printk(KERN_ERR "%s: Unable to allocate output buffer.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_debug_ic_group_data_store_exit;
	}

	conv_buf = kzalloc(sizeof(uint8_t) * 5, GFP_KERNEL);
	if (conv_buf == NULL) {
		printk(KERN_ERR "%s: Unable to allocate conversion buffer.\n",
			__func__);
		err = -ENOMEM;
		goto atmxt_debug_ic_group_data_store_exit;
	}

	for (i = 0; i < size; i += 5) {
		memcpy(conv_buf, &(buf[i]), 4);
		err = strict_strtoul(conv_buf, 16, &value);
		if (err < 0) {
			printk(KERN_ERR "%s: Argument conversion failed.\n",
				__func__);
			goto atmxt_debug_ic_group_data_store_exit;
		} else if (value > 255) {
			printk(KERN_ERR "%s: Value 0x%lX is too large.\n",
				__func__, value);
			err = -EOVERFLOW;
			goto atmxt_debug_ic_group_data_store_exit;
		}

		data_out[data_size] = value;
		data_size++;
	}

	if ((dd->dbg->grp_off + data_size) > (entry[3] + 1)) {
		printk(KERN_ERR "%s: Trying to write %d bytes at offset %hu, "
			"which exceeds group size of %hu.\n", __func__,
			 data_size, dd->dbg->grp_off, entry[3]+1);
		err = -EOVERFLOW;
		goto atmxt_debug_ic_group_data_store_exit;
	}

	addr_lo = entry[1] + dd->dbg->grp_off;
	addr_hi = entry[2];
	if (addr_lo < entry[1])
		addr_hi++;

	err = atmxt_i2c_write(dd, addr_lo, addr_hi, data_out, data_size);
	if (err < 0) {
		printk(KERN_ERR
			"%s: Failed to write data with error code %d.\n",
			__func__, err);
		goto atmxt_debug_ic_group_data_store_exit;
	}

	if (!(dd->status & (1 << ATMXT_IGNORE_CHECKSUM))) {
		printk(KERN_INFO
			"%s: Disabled settings checksum verification %s.\n",
			__func__, "until next boot");
	}
	dd->status = dd->status | (1 << ATMXT_IGNORE_CHECKSUM);

	err = size;

atmxt_debug_ic_group_data_store_exit:
	kfree(data_out);
	kfree(conv_buf);
	mutex_unlock(dd->mutex);

	return (ssize_t) err;
}
static DEVICE_ATTR(ic_grpdata, S_IRUSR | S_IWUSR,
	atmxt_debug_ic_group_data_show, atmxt_debug_ic_group_data_store);

static ssize_t atmxt_debug_ic_group_number_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	return sprintf(buf, "Current Group: %hu\n", dd->dbg->grp_num);
}
static ssize_t atmxt_debug_ic_group_number_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);
	unsigned long value = 0;

	mutex_lock(dd->mutex);

	err = strict_strtoul(buf, 10, &value);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to convert value.\n", __func__);
		goto atmxt_debug_ic_group_number_store_exit;
	}

	if (value > 255) {
		printk(KERN_ERR "%s: Invalid group number %lu--%s.\n",
			__func__, value, "setting to 255");
		dd->dbg->grp_num = 255;
	} else {
		dd->dbg->grp_num = value;
	}

	err = size;

atmxt_debug_ic_group_number_store_exit:
	mutex_unlock(dd->mutex);

	return err;
}
static DEVICE_ATTR(ic_grpnum, S_IRUSR | S_IWUSR,
	atmxt_debug_ic_group_number_show, atmxt_debug_ic_group_number_store);

static ssize_t atmxt_debug_ic_group_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	return sprintf(buf, "Current Offset: %hu\n", dd->dbg->grp_off);
}
static ssize_t atmxt_debug_ic_group_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);
	unsigned long value = 0;

	mutex_lock(dd->mutex);

	err = strict_strtoul(buf, 10, &value);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to convert value.\n", __func__);
		goto atmxt_debug_ic_group_offset_store_exit;
	}

	if (value > 255) {
		printk(KERN_ERR "%s: Invalid offset %lu--setting to 255.\n",
			__func__, value);
		dd->dbg->grp_off = 255;
	} else {
		dd->dbg->grp_off = value;
	}

	err = size;

atmxt_debug_ic_group_offset_store_exit:
	mutex_unlock(dd->mutex);

	return err;
}
static DEVICE_ATTR(ic_grpoffset, S_IRUSR | S_IWUSR,
	atmxt_debug_ic_group_offset_show, atmxt_debug_ic_group_offset_store);

static void atmxt_debug_ic_reflash_callback(const struct firmware *fw,
		void *context)
{
	int err = 0;
	struct atmxt_driver_data *dd = context;
	struct touch_firmware touch_fw;

	mutex_lock(dd->mutex);

	dd->status = dd->status & ~(1 << ATMXT_WAITING_FOR_FW_FLAG);

	if (fw == NULL) {
		printk(KERN_ERR "%s: No firmware received.\n", __func__);
		goto atmxt_debug_ic_reflash_callback_fail;
	}

	if (fw->data[0] < 4) {
		printk(KERN_ERR "%s: Invalid firmware header.\n", __func__);
		err = -EINVAL;
		goto atmxt_debug_ic_reflash_callback_fail;
	} else if ((fw->data[0] + 1) >= fw->size) {
		printk(KERN_ERR "%s: Received malformed firmware.\n",
			__func__);
		err = -EOVERFLOW;
		goto atmxt_debug_ic_reflash_callback_fail;
	} else {
		atmxt_dbg(dd, ATMXT_DBG3, "%s: Received firmware for "\
			"Family ID: 0x%02X, Variant ID: 0x%02X, "\
			"Version: 0x%02X, Build: 0x%02X\n", __func__,
			fw->data[1], fw->data[2], fw->data[3], fw->data[4]);
	}

	touch_fw.img = &(fw->data[fw->data[0] + 1]);
	touch_fw.size = (uint32_t) (fw->size - (fw->data[0] + 1));

	err = atmxt_validate_firmware(touch_fw.img, touch_fw.size);
	if (err < 0) {
		printk(KERN_ERR
			"%s: Firmware image is invalid with error code %d.\n",
			__func__, err);
		goto atmxt_debug_ic_reflash_callback_fail;
	}

	if (dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG)) {
		disable_irq_nosync(dd->client->irq);
		dd->status = dd->status & ~(1 << ATMXT_IRQ_ENABLED_FLAG);
	}

	err = atmxt_restart_ic(dd, &touch_fw, true);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to %s with error code %d.\n",
			__func__, "force flash and initialize the touch IC",
			err);
		atmxt_set_drv_state(dd, ATMXT_DRV_IDLE);
		goto atmxt_debug_ic_reflash_callback_fail;
	}

	if (((atmxt_get_drv_state(dd) == ATMXT_DRV_ACTIVE)) &&
		(!(dd->status & (1 << ATMXT_IRQ_ENABLED_FLAG)))) {
		dd->status = dd->status | (1 << ATMXT_IRQ_ENABLED_FLAG);
		enable_irq(dd->client->irq);
	}

atmxt_debug_ic_reflash_callback_fail:
	mutex_unlock(dd->mutex);

	return;
}
static ssize_t atmxt_debug_ic_reflash_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	if (dd->status & (1 << ATMXT_WAITING_FOR_FW_FLAG))
		return sprintf(buf, "Driver is waiting for firmware load.\n");
	else
		return sprintf(buf, "No firmware loading in progress.\n");
}
static ssize_t atmxt_debug_ic_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	mutex_lock(dd->mutex);

	if (dd->status & (1 << ATMXT_WAITING_FOR_FW_FLAG)) {
		printk(KERN_ERR "%s: Driver is already waiting for firmware.\n",
			__func__);
		err = -EALREADY;
		goto atmxt_debug_ic_reflash_store_fail;
	}

	printk(KERN_INFO "%s: Enabling firmware class loader...\n", __func__);

	err = request_firmware_nowait(THIS_MODULE,
		FW_ACTION_NOHOTPLUG, "", &(dd->client->dev),
		GFP_KERNEL, dd, atmxt_debug_ic_reflash_callback);
	if (err < 0) {
		printk(KERN_ERR
			"%s: Firmware request failed with error code %d.\n",
			__func__, err);
		goto atmxt_debug_ic_reflash_store_fail;
	}

	dd->status = dd->status | (1 << ATMXT_WAITING_FOR_FW_FLAG);
	err = size;

atmxt_debug_ic_reflash_store_fail:
	mutex_unlock(dd->mutex);

	return err;
}
static DEVICE_ATTR(ic_reflash, S_IRUSR | S_IWUSR,
	atmxt_debug_ic_reflash_show, atmxt_debug_ic_reflash_store);
#endif

static ssize_t atmxt_debug_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atmxt_driver_data *dd = dev_get_drvdata(dev);

	if (dd->info_blk == NULL) {
		return sprintf(buf,
			"No touch IC version information is available.\n");
	} else {
		return sprintf(buf, "%s0x%02X\n%s0x%02X\n%s0x%02X\n%s0x%02X\n",
			"Family ID: ", dd->info_blk->header[0],
			"Variant ID: ", dd->info_blk->header[1],
			"Version: ", dd->info_blk->header[2],
			"Build: ", dd->info_blk->header[3]);
	}
}
static DEVICE_ATTR(ic_ver, S_IRUGO, atmxt_debug_ic_ver_show, NULL);

static int atmxt_create_debug_files(struct atmxt_driver_data *dd)
{
	int err = 0;
	int check = 0;

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	check = device_create_file(&(dd->client->dev), &dev_attr_drv_debug);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create drv_debug.\n", __func__);
		err = check;
	}

	check = device_create_file(&(dd->client->dev), &dev_attr_drv_flags);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create drv_flags.\n", __func__);
		err = check;
	}
#endif

	check = device_create_file(&(dd->client->dev), &dev_attr_drv_irq);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create drv_irq.\n", __func__);
		err = check;
	}

	check = device_create_file(&(dd->client->dev), &dev_attr_drv_stat);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create drv_stat.\n", __func__);
		err = check;
	}

	check = device_create_file(&(dd->client->dev), &dev_attr_drv_ver);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create drv_ver.\n", __func__);
		err = check;
	}

	check = device_create_file(&(dd->client->dev), &dev_attr_hw_irqstat);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create hw_irqstat.\n", __func__);
		err = check;
	}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	check = device_create_file(&(dd->client->dev), &dev_attr_hw_recov);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create hw_recov.\n", __func__);
		err = check;
	}

	check = device_create_file(&(dd->client->dev), &dev_attr_hw_reset);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create hw_reset.\n", __func__);
		err = check;
	}

	check = device_create_file(&(dd->client->dev), &dev_attr_ic_grpdata);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create ic_grpdata.\n", __func__);
		err = check;
	}

	check = device_create_file(&(dd->client->dev), &dev_attr_ic_grpnum);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create ic_grpnum.\n", __func__);
		err = check;
	}

	check = device_create_file(&(dd->client->dev), &dev_attr_ic_grpoffset);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create ic_grpoffset.\n",
			__func__);
		err = check;
	}

	check = device_create_file(&(dd->client->dev), &dev_attr_ic_reflash);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create ic_reflash.\n", __func__);
		err = check;
	}
#endif

	check = device_create_file(&(dd->client->dev), &dev_attr_ic_ver);
	if (check < 0) {
		printk(KERN_ERR "%s: Failed to create ic_ver.\n", __func__);
		err = check;
	}

	return err;
}

static void atmxt_remove_debug_files(struct atmxt_driver_data *dd)
{
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	device_remove_file(&(dd->client->dev), &dev_attr_drv_debug);
	device_remove_file(&(dd->client->dev), &dev_attr_drv_flags);
	device_remove_file(&(dd->client->dev), &dev_attr_drv_irq);
#endif
	device_remove_file(&(dd->client->dev), &dev_attr_drv_stat);
	device_remove_file(&(dd->client->dev), &dev_attr_drv_ver);
	device_remove_file(&(dd->client->dev), &dev_attr_hw_irqstat);
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	device_remove_file(&(dd->client->dev), &dev_attr_hw_recov);
	device_remove_file(&(dd->client->dev), &dev_attr_hw_reset);
	device_remove_file(&(dd->client->dev), &dev_attr_ic_grpdata);
	device_remove_file(&(dd->client->dev), &dev_attr_ic_grpnum);
	device_remove_file(&(dd->client->dev), &dev_attr_ic_grpoffset);
	device_remove_file(&(dd->client->dev), &dev_attr_ic_reflash);
#endif
	device_remove_file(&(dd->client->dev), &dev_attr_ic_ver);
	return;
}
