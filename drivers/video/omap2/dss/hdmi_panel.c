/*
 * hdmi_panel.c
 *
 * HDMI library support functions for TI OMAP4 processors.
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors:	Mythri P k <mythripk@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <video/omapdss.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include "dss.h"

#include <video/hdmi_ti_4xxx_ip.h>

#define MAX_EDID_READ_ATTEMPTS 5

#ifdef CONFIG_SND_SOC_OMAP_HDMI_CODEC
extern void omap_hdmi_audio_set_plug_state(int plugged);
#endif

static struct {
	struct mutex hdmi_lock;
	struct switch_dev hpd_switch;
	struct switch_dev hdmi_audio_switch;
} hdmi;

static ssize_t hdmi_deepcolor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r;
	r = omapdss_hdmi_get_deepcolor();
	return snprintf(buf, PAGE_SIZE, "%d\n", r);
}

static ssize_t hdmi_deepcolor_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long deep_color;
	int r = kstrtoul(buf, 0, &deep_color);
	if (r || deep_color > 2)
		return -EINVAL;
	omapdss_hdmi_set_deepcolor(deep_color);
	return size;
}

static DEVICE_ATTR(deepcolor, S_IRUGO | S_IWUSR, hdmi_deepcolor_show,
							hdmi_deepcolor_store);

static int hdmi_panel_probe(struct omap_dss_device *dssdev)
{
	DSSDBG("ENTER hdmi_panel_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT |
			OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;

	/* sysfs entry to provide user space control to set deepcolor mode */
	if (device_create_file(&dssdev->dev, &dev_attr_deepcolor))
		DSSERR("failed to create sysfs file\n");

	return 0;
}

static void hdmi_panel_remove(struct omap_dss_device *dssdev)
{
	device_remove_file(&dssdev->dev, &dev_attr_deepcolor);
}

static int hdmi_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;
	DSSDBG("ENTER hdmi_panel_enable\n");

	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
			/* this can happen in boot during
			 * race for hdmi to auto enable display
			 * vs omapfb trying to enable it
			 */
			r = 0;
			pr_info("%s: returning 0 because already enabled\n",
				__func__);
		}
		goto err;
	}

	if (hdmi_get_current_hpd() == 0) {
		pr_warn("%s: returning 0 but not enabling because hpd is 0\n",
			__func__);
		goto err;
	}

	/* omapfb will always try to enable the driver on boot, but
	 * we may not have read edid yet and set a mode */
	if (dssdev->panel.timings.pixel_clock == 0) {
		DSSWARN("%s: returning 0 but not enabling because"
			" pixel_clock is 0\n", __func__);
		goto err;
	}
	DSSINFO("%s: pixel_clock = %d, res = %d,%d\n",
		__func__, dssdev->panel.timings.pixel_clock,
		dssdev->panel.timings.x_res, dssdev->panel.timings.y_res);

	r = omapdss_hdmi_display_enable(dssdev, true);
	if (r) {
		DSSERR("failed to power on\n");
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
err:
	mutex_unlock(&hdmi.hdmi_lock);

	return r;
}

static void hdmi_panel_disable(struct omap_dss_device *dssdev)
{
	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		msleep(100);
		dssdev->manager->blank(dssdev->manager, true);
		omapdss_hdmi_display_disable(dssdev);
	} else
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	mutex_unlock(&hdmi.hdmi_lock);
}

static int hdmi_panel_suspend(struct omap_dss_device *dssdev)
{
	int r = 0;

	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	hdmi_panel_hpd_handler();

	omapdss_hdmi_display_disable(dssdev);
err:
	mutex_unlock(&hdmi.hdmi_lock);

	return r;
}

static int hdmi_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
err:
	mutex_unlock(&hdmi.hdmi_lock);

	hdmi_panel_hpd_handler();

	return r;
}

enum {
	HPD_STATE_RESET,
	HPD_STATE_CHECK_PLUG_STATE,
	HPD_STATE_CHECK_EDID,
	HPD_STATE_DONE_ENABLED,
	HPD_STATE_DONE_DISABLED,
};

static DEFINE_SPINLOCK(hpd_state_lock);
static struct hpd_worker_data {
	struct delayed_work dwork;
	int state;
	int edid_reads;
} hpd_work;
static struct workqueue_struct *my_workq;

static int hdmi_hpd_set_state(int target_state, int resched_time) {
	unsigned long flags;
	int ret = 0;
	spin_lock_irqsave(&hpd_state_lock, flags);

	/* If the state machine is attempting to advance to anything past
	 * CHECK_PLUG_STATE, but we are currently in the RESET state, then we
	 * must have been reset by an HPD IRQ while conducting some long running
	 * operation in the worker callback.  There should already be another
	 * worker callback in flight, so we should do nothing and just get out
	 * of the pending callback's way.
	 */
	if ((hpd_work.state == HPD_STATE_RESET) &&
		(target_state >= HPD_STATE_CHECK_EDID)) {
		ret = -1;
		goto done;
	}

	if (target_state == HPD_STATE_RESET)
		hdmi_set_edid_state(false);

	__cancel_delayed_work(&hpd_work.dwork);
	hpd_work.state = target_state;
	if (resched_time >= 0)
		queue_delayed_work(my_workq,
				&hpd_work.dwork,
				msecs_to_jiffies(resched_time));

done:
	spin_unlock_irqrestore(&hpd_state_lock, flags);
	return ret;
}

static int hdmi_hpd_get_state(void) {
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&hpd_state_lock, flags);
	ret = hpd_work.state;
	spin_unlock_irqrestore(&hpd_state_lock, flags);

	return ret;
}

static void hdmi_hotplug_detect_worker(struct work_struct *work)
{
	struct hpd_worker_data *d = container_of(work, typeof(*d), dwork.work);
	struct omap_dss_device *dssdev = NULL;
	int state;

	int match(struct omap_dss_device *dssdev, void *arg)
	{
		return sysfs_streq(dssdev->name , "hdmi");
	}
	dssdev = omap_dss_find_device(NULL, match);

	if (dssdev == NULL)
		return;

	mutex_lock(&hdmi.hdmi_lock);

	state = hdmi_hpd_get_state();
	pr_info("in hpd work state %d, dssdev state %d, hpd state %d\n",
			state, dssdev->state, hdmi_get_current_hpd());

	switch (state) {
	case HPD_STATE_RESET:
		/* Were we just reset?  If so, shut everything down, then
		 * schedule a check of the plug state in the near future.
		 */
#ifdef CONFIG_SND_SOC_OMAP_HDMI_CODEC
		omap_hdmi_audio_set_plug_state(0);
#endif
		switch_set_state(&hdmi.hdmi_audio_switch, 0);
		switch_set_state(&hdmi.hpd_switch, 0);
		memset(&dssdev->panel.audspecs, 0,
				sizeof(dssdev->panel.audspecs));
		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
			mutex_unlock(&hdmi.hdmi_lock);
			dssdev->driver->disable(dssdev);
			mutex_lock(&hdmi.hdmi_lock);
		} else {
			/*
			 * We could have enabled during edid
			 * read phase so call disable directly
			 * bypassing the notifications.  if
			 * we're already disabled, this will
			 * be a noop.
			 */
			omapdss_hdmi_display_disable(dssdev);
		}

		hdmi_hpd_set_state(HPD_STATE_CHECK_PLUG_STATE, 60);
		break;

	case HPD_STATE_CHECK_PLUG_STATE:
		if (hdmi_get_current_hpd()) {
			/* Looks like there is something plugged in.
			 * Enable the DSS driver and get ready to read the
			 * sink's EDID information.
			 */
			hpd_work.edid_reads = 0;

			/*
			 * Just turn on power to hdmi so we can
			 * read edid, but don't turn on the
			 * display yet.
			 */
			omapdss_hdmi_display_enable(dssdev, false);

			hdmi_hpd_set_state(HPD_STATE_CHECK_EDID, 60);
		} else {
			/* nothing plugged in, so we are finished.  Go
			 * to the DONE_DISABLED state and stay
			 * there until the next HPD event. */
			goto end_disabled;
		}
		break;

	case HPD_STATE_CHECK_EDID:
		if (hdmi_get_current_hpd() == 0) {
			/* hpd dropped - stop EDID read */
			pr_info("hpd == 0, aborting EDID read\n");
			goto end_disabled;
		}

		if (!hdmi_read_edid(&dssdev->panel.timings)) {
			/* Failed to read EDID.  If we still have retry attempts
			 * left, schedule another attempt.  Otherwise give up
			 * and just go to the disabled state.
			 */
			hpd_work.edid_reads++;
			if (hpd_work.edid_reads >= MAX_EDID_READ_ATTEMPTS) {
				pr_info("Failed to read EDID after %d times."
				        " Giving up.\n", hpd_work.edid_reads);
				goto end_disabled;
			} else {
				hdmi_hpd_set_state(HPD_STATE_CHECK_EDID, 60);
			}
			break;
		}

		/* get monspecs from edid */
		hdmi_get_monspecs(&dssdev->panel.monspecs);

		/* get audio support from edid */
		hdmi_get_audspecs(&dssdev->panel.audspecs);

		pr_info("panel size %d by %d\n",
				dssdev->panel.monspecs.max_x,
				dssdev->panel.monspecs.max_y);
		dssdev->panel.width_in_um =
				dssdev->panel.monspecs.max_x * 10000;
		dssdev->panel.height_in_um =
				dssdev->panel.monspecs.max_y * 10000;

		/* Finally, if we have not already been reset by another hot
		 * plug event, set the switch which informs the user mode code
		 * that we are plugged and ready to have our video mode
		 * configured.
		 */
		if (!hdmi_hpd_set_state(HPD_STATE_DONE_ENABLED, -1)) {
			mutex_unlock(&hdmi.hdmi_lock);
			/* set and enable an initial video mode
			 * because some monitors don't like it if
			 * we delay this too long and who knows when
			 * usermode will do this (especially in factory
			 * setting).
			 */
			omapdss_hdmi_display_set_initial_mode(dssdev);
			switch_set_state(&hdmi.hpd_switch, 1);
			return;
		}

		break;

	default:
		/* We should not have scheduled work in any other states.  Log a
		 * warning and ignore the work. */
		pr_warn("HPD worker scheduled unexpected state %d",
				hpd_work.state);
		break;
	}

	mutex_unlock(&hdmi.hdmi_lock);
	return;

end_disabled:
	omapdss_hdmi_display_disable(dssdev);
	hdmi_hpd_set_state(HPD_STATE_DONE_DISABLED, -1);
	mutex_unlock(&hdmi.hdmi_lock);
}

void hdmi_panel_hpd_handler(void)
{
	hdmi_hpd_set_state(HPD_STATE_RESET, 40);
}

static void hdmi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	mutex_lock(&hdmi.hdmi_lock);

	/* we don't have timings on boot if no hdmi
	 * connected, but surface flinger crashes if
	 * we don't return something non-zero.
	 */
	if (dssdev->panel.timings.x_res == 0) {
		DSSINFO("no timings yet, returning some safe defaults\n");
		*timings = (struct omap_video_timings)
			{640, 480, 31746, 128, 24, 29, 9, 40, 2};
	} else
		*timings = dssdev->panel.timings;

	mutex_unlock(&hdmi.hdmi_lock);
}

static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	DSSDBG("hdmi_set_timings\n");

	mutex_lock(&hdmi.hdmi_lock);

	dssdev->panel.timings = *timings;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		omapdss_hdmi_display_set_timing(dssdev);

	mutex_unlock(&hdmi.hdmi_lock);
}

static int hdmi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	int r = 0;

	DSSDBG("hdmi_check_timings\n");

	mutex_lock(&hdmi.hdmi_lock);

	r = omapdss_hdmi_display_check_timing(dssdev, timings);
	if (r) {
		DSSERR("Timing cannot be applied\n");
		goto err;
	}
err:
	mutex_unlock(&hdmi.hdmi_lock);
	return r;
}

static int hdmi_get_modedb(struct omap_dss_device *dssdev,
			   struct fb_videomode *modedb, int modedb_len)
{
	struct fb_monspecs *specs = &dssdev->panel.monspecs;
	if (specs->modedb_len < modedb_len)
		modedb_len = specs->modedb_len;
	memcpy(modedb, specs->modedb, sizeof(*modedb) * modedb_len);
	return modedb_len;
}
static void hdmi_get_resolution(struct omap_dss_device *dssdev,
			       u16 *xres, u16 *yres)
{
	/*
	 * Initialize the timings to 640 * 480
	 * This is only for framebuffer update not for TV timing setting
	 * Setting TV timing will be done only on enable
	 */
	if (dssdev->panel.timings.x_res == 0) {
		*xres = 640;
		*yres = 480;
	} else {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	}
}

static void hdmi_get_fb_resolution(struct omap_dss_device *dssdev,
			       u16 *xres, u16 *yres)
{
	if (dssdev->panel.fb_width_in_pixels == 0) {
		*xres = 640;
		*yres = 480;
	} else {
		*xres = dssdev->panel.fb_width_in_pixels;
		*yres = dssdev->panel.fb_height_in_pixels;
	}
}



static int hdmi_set_mode(struct omap_dss_device *dssdev,
			 struct fb_videomode *vm)
{
	int ret;

	/* Reject any set mode request if edid hasn't been
	 * read (or has been lost).  Bouncing hpd could cause
	 * this call to happen when the hpd state machine
	 * is still running handling a new hdp->1 case, and
	 * the disable of the driver that setting the mode
	 * requires could cause the edid read to fail.
	 */
	if (hdmi_hpd_get_state() != HPD_STATE_DONE_ENABLED)
		return -ENODEV;

	ret = omapdss_hdmi_display_set_mode(dssdev, vm);

	mutex_lock(&hdmi.hdmi_lock);

	/* if we succeeded in setting our display mode, and our display is
	 * active, and our edid reports that we have support for any audio modes
	 * at all, then signal that fact to the user mode level of code.
	 *
	 * TODO: There is still a lot about audio support which could be made
	 * better.  For example, the audio driver does not bother to ask which
	 * audio modes are supported when deciding to enable or not.  In fact,
	 * it only checks to see if the selected video mode is an "hdmi video
	 * mode" or not.  Its possible for an HDMI panel to accept a VESA/DVI
	 * video mode and still support audio.  Likewise, just because a panel
	 * is using HDMI does not mean that the panel must support audio.
	 * Finally, just because the panel supports a particular audio mode does
	 * not mean that the mode can be driven at all times.  Some high bitrate
	 * audio modes can only be supported if the chosen pixel clock is high
	 * enough.  All of these factors should be taken into account by the
	 * HDMI audio driver when it decides which audio modes are allowed and
	 * which ones are not.  Currently none of this is taken into account.
	 */
	if (!ret &&
	   (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) &&
	   (dssdev->panel.audspecs.basic_audio_support ||
	    dssdev->panel.audspecs.valid_mode_cnt)) {
#ifdef CONFIG_SND_SOC_OMAP_HDMI_CODEC
		omap_hdmi_audio_set_plug_state(1);
#endif
		switch_set_state(&hdmi.hdmi_audio_switch, 1);
	}

	mutex_unlock(&hdmi.hdmi_lock);

	return ret;
}

static struct omap_dss_driver hdmi_driver = {
	.probe		= hdmi_panel_probe,
	.remove		= hdmi_panel_remove,
	.enable		= hdmi_panel_enable,
	.disable	= hdmi_panel_disable,
	.suspend	= hdmi_panel_suspend,
	.resume		= hdmi_panel_resume,
	.get_timings	= hdmi_get_timings,
	.set_timings	= hdmi_set_timings,
	.check_timings	= hdmi_check_timings,
	.get_resolution = hdmi_get_resolution,
	.get_fb_resolution = hdmi_get_fb_resolution,
	.get_modedb	= hdmi_get_modedb,
	.set_mode	= hdmi_set_mode,
	.driver			= {
		.name   = "hdmi_panel",
		.owner  = THIS_MODULE,
	},
};

int hdmi_panel_init(void)
{
	mutex_init(&hdmi.hdmi_lock);

	hdmi.hpd_switch.name = "hdmi";
	switch_dev_register(&hdmi.hpd_switch);

	hdmi.hdmi_audio_switch.name = "hdmi_audio";
	switch_dev_register(&hdmi.hdmi_audio_switch);

	my_workq = create_singlethread_workqueue("hdmi_hotplug");
	INIT_DELAYED_WORK(&hpd_work.dwork, hdmi_hotplug_detect_worker);
	omap_dss_register_driver(&hdmi_driver);

	/*
	 * Defer checking plugged in hpd state until
	 * hdcp firmware has been loaded.
	 */
	return 0;
}

void hdmi_panel_exit(void)
{
	destroy_workqueue(my_workq);
	omap_dss_unregister_driver(&hdmi_driver);

	switch_dev_unregister(&hdmi.hpd_switch);
	switch_dev_unregister(&hdmi.hdmi_audio_switch);
}
