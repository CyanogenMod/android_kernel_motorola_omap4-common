/*
 * drivers/video/omap2/misc/dssmgr.c
 *
 * Copyright (C) 2010 Motorola Inc.
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

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pagemap.h>
#include <linux/platform_device.h>
#include <linux/slab.h> /* For kzalloc/kfree*/

#include <video/omapdss.h>
#include <video/hdmi_ti_4xxx_ip.h>

#include <linux/omap-dssmgr.h>
#include "dssmgr-oly.h"

#include "../dss/dss.h"

#define DEVICE_NAME  "omap-dssmgr"

/*#define DM_DEBUG*/
#ifdef DM_DEBUG
#define DMDBG(format, ...) \
	printk(KERN_INFO "DSSMGR: " format, ## __VA_ARGS__)
#else
#define DMDBG(format, ...)
#endif

#define EDID_BLOCK_SIZE      (DSSMGR_EDID_BLOCK_LEN)
#define EDID_MAX_BLOCKS      (4)
#define EDID_MAX_SIZE        (EDID_MAX_BLOCKS * EDID_BLOCK_SIZE)
#define EDID_HEADER_SIZE     (8)
#define EDID_NUM_EXT_BLKS    (0x7E)

static const u8 header[EDID_HEADER_SIZE] = { 0x0,  0xff, 0xff, 0xff
					   , 0xff, 0xff, 0xff, 0x0};

struct dssmgr_device {
	struct mutex   mtx; /* Lock for all device accesses */

	int major;
	struct class  *cls;
	struct device *dev;

	u32 client_next_id;
	int client_cnt;

	int num_dpys;
	struct omap_dss_device      *dpys[DSSMGR_MAX_DISPLAYS];
	int num_mgrs;
	struct omap_overlay_manager *mgrs[DSSMGR_MAX_MANAGERS];
	int num_olys;
	struct omap_overlay         *olys[DSSMGR_MAX_OVERLAYS];

	/* Overlay manipulation specific data */
	struct dssmgr_oly_data       oly_data;
};

static struct dssmgr_device *g_dev;

/*=== Local Functions ==================================================*/

static int dssmgr_enable_dpy(struct dssmgr_cmd *cmd)
{
	struct omap_dss_device *dpy;
	int rc = 0;
	int idx, en;

	idx = cmd->comp_id1 - 1;
	en  = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_dpys) {
		printk(KERN_ERR "DSSMGR: Invalid Display ID\n");
		rc = -EINVAL;
		goto failed;
	}
	if (en != 0 && en != 1) {
		printk(KERN_ERR "DSSMGR: Invalid Display Enable\n");
		rc = -EINVAL;
		goto failed;
	}

	dpy = g_dev->dpys[idx];
	if (en) {
		rc = dpy->driver->enable(dpy);
		DMDBG("Enabled display (%s/%d)\n", dpy->name, rc);
	} else {
		dpy->driver->disable(dpy);
		DMDBG("Disabled display (%s)\n", dpy->name);
	}

failed:
	return rc;
}

static int dssmgr_attach_oly_2_mgr(struct dssmgr_cmd *cmd)
{
	struct omap_overlay         *oly;
	struct omap_overlay_manager *mgr, *old;
#ifdef USE_DSSMGR_OLY_FEATURES
	struct dssmgr_oly_data      *od;
#endif
	int rc = 0;
	int rc2 = 0;
	int o_idx, m_idx;
	int rm_force = 0;

	o_idx = cmd->comp_id1 - 1;
	m_idx = cmd->comp_id2 - 1;

	if (o_idx < 0 || o_idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	if (m_idx < 0 || m_idx >= g_dev->num_mgrs) {
		rc = -EINVAL;
		goto failed;
	}

	oly = g_dev->olys[o_idx];
	mgr = g_dev->mgrs[m_idx];
	if (oly->manager != mgr) {
#ifdef USE_DSSMGR_OLY_FEATURES
		od = &g_dev->oly_data;
		if (!dssmgr_oly_get_force_disable(od, o_idx)) {
			dssmgr_oly_set_force_disable(od, o_idx, 1);
			rm_force = 1;
		}
#endif

		old = NULL;
		if (oly->manager) {
			old = oly->manager;
			oly->unset_manager(oly);
			old->apply(old);
		}

		rc = oly->set_manager(oly, mgr);
		if (rc != 0) {
			printk(KERN_ERR "DSSMGR: Set MGR Failure\n");
			if (old != NULL) {
				oly->set_manager(oly, old);
				old->apply(old);
			}
		}

		if (rm_force) {
#ifdef USE_DSSMGR_OLY_FEATURES
			/* Does the mgr apply */
			rc2 = dssmgr_oly_set_force_disable(od, o_idx, 0);
#endif
		} else if (rc == 0) {
			/* the set_manager succeeded, so apply */
			rc2 = mgr->apply(mgr);
		}

		if (rc2 != 0 && rc == 0)
			rc = rc2;

		DMDBG("Oly(%s)->Mgr(%s)/%d\n", oly->name, mgr->name, rc);
	}

failed:
	return rc;
}

static int dssmgr_attach_mgr_2_dpy(struct dssmgr_cmd *cmd)
{
	struct omap_overlay_manager *mgr;
	struct omap_dss_device      *dpy;
	int rc = 0;
	int m_idx, d_idx;

	m_idx = cmd->comp_id1 - 1;
	d_idx = cmd->comp_id2 - 1;

	if (m_idx < 0 || m_idx >= g_dev->num_mgrs) {
		rc = -EINVAL;
		goto failed;
	}
	if (d_idx < 0 || d_idx >= g_dev->num_dpys) {
		rc = -EINVAL;
		goto failed;
	}

	mgr = g_dev->mgrs[m_idx];
	dpy = g_dev->dpys[d_idx];
	if (mgr->device != dpy) {
		if (mgr->device)
			mgr->unset_device(mgr);

		rc = mgr->set_device(mgr, dpy);
		if (rc != 0)
			printk(KERN_ERR "DSSMGR: Set DPY Failure\n");
		else
			rc = mgr->apply(mgr);

		DMDBG("Mgr(%s)->Dpy(%s)/%d\n", mgr->name, dpy->name, rc);
	}

failed:
	return rc;
}

static int getResCode(struct dssmgr_resolution *res, int *code)
{
	int rc = -1;
	int c = -1;

	int w = res->width;
	int h = res->height;
	int i = ((res->flags & DSSMGR_RES_FLAG_INTERLACED) != 0);
	int r60hz = (res->rate == 60);
	int r50hz = (res->rate == 50);

	switch (h) {
	case 480:
		if (w == 640 && !i && r60hz)
			c = 1;
		else if (w == 720 && i && r60hz)
			c = 6;
		else if (w == 720 && !i && r60hz)
			c = 2;
		break;
	case 576:
		if (w == 720 && i && r50hz)
			c = 21;
		else if (w == 720 && !i && r50hz)
			c = 17;
		break;
	case 720:
		if (w == 1280 && !i && r60hz)
			c = 4;
		else if (w == 1280 && !i && r50hz)
			c = 19;
		break;
	case 1080:
		if (w == 1920 && i && r60hz)
			c = 5;
		else if (w == 1920 && !i && r60hz)
			c = 16;
		else if (w == 1920 && i && r50hz)
			c = 20;
		else if (w == 1920 && !i && r50hz)
			c = 31;
		break;
	}

	if (c != -1) {
		*code = c;
		rc = 0;
	}

	return rc;
}

static void dssmgr_convert_timings(struct dssmgr_resolution *res,
					struct fb_videomode *fb_timings)
{
	memset(fb_timings, 0, sizeof(*fb_timings));
	fb_timings->xres         = res->width;
	fb_timings->yres         = res->height;
	fb_timings->pixclock     = KHZ2PICOS(res->timing.clk);
	fb_timings->right_margin = res->timing.hfp;
	fb_timings->left_margin  = res->timing.hbp;
	fb_timings->hsync_len    = res->timing.hsw;
	fb_timings->lower_margin = res->timing.vfp;
	fb_timings->upper_margin = res->timing.vbp;
	fb_timings->vsync_len    = res->timing.vsw;
	fb_timings->sync         = (res->timing.sp & 0x01) ?
						FB_SYNC_HOR_HIGH_ACT : 0;
	fb_timings->sync        |= (res->timing.sp & 0x02) ?
						FB_SYNC_VERT_HIGH_ACT : 0;
	fb_timings->vmode        = FB_VMODE_NONINTERLACED;
	fb_timings->flag         = FB_MODE_IS_VESA;
}

static int dssmgr_set_resolution(struct dssmgr_cmd *cmd)
{
	struct omap_dss_device    *dpy;
	struct fb_videomode        timings;
	int rc = 0;
	int idx;
	int code;
	int reenable = 0;
	int audio_en = 0;

	idx = cmd->comp_id1 - 1;

	if (idx < 0 || idx >= g_dev->num_dpys) {
		rc = -EINVAL;
		goto failed;
	}

	DMDBG("Resolution: %dx%d @ %d (%04x)\n",
		cmd->resolution.width, cmd->resolution.height,
		cmd->resolution.rate, cmd->resolution.flags);

	dpy = g_dev->dpys[idx];

	if (dpy->state == OMAP_DSS_DISPLAY_ACTIVE ||
	    dpy->state == OMAP_DSS_DISPLAY_TRANSITION) {
		reenable = 1;
		dpy->driver->disable(dpy);
	}

	if (cmd->resolution.flags & DSSMGR_RES_FLAG_MIN_DS)
		dpy->phy.hdmi.ds_percent = 50;
	else
		dpy->phy.hdmi.ds_percent = 100;

	if (cmd->resolution.flags & DSSMGR_RES_FLAG_DVI_AUDIO)
		audio_en = 1;

	if (!(cmd->resolution.flags & (DSSMGR_RES_FLAG_TIMING |
					DSSMGR_RES_FLAG_HDMI))) {
		DMDBG("Not HDMI and no timings\n");
		rc = -EINVAL;
		goto failed;
	} else if (cmd->resolution.flags & DSSMGR_RES_FLAG_HDMI) {
		/* Set via EDID timing mode/code */
		if (dpy->driver->set_hdmi_mode == NULL) {
			rc = -EINVAL;
			goto failed;
		}

		rc = getResCode(&cmd->resolution, &code);
		if (rc != 0) {
			rc = -EINVAL;
			goto failed;
		}

		rc = dpy->driver->set_hdmi_mode(dpy, code);
	} else {
		/* Set via direct timing data */
		if (dpy->driver->set_mode == NULL) {
			rc = -EINVAL;
			goto failed;
		}

		dssmgr_convert_timings(&cmd->resolution, &timings);

		/* For now, flag as DSSMGR set so that the HDMI driver
		 * will not force enable the output as it does for
		 * the TI interface case.
		 */
		timings.flag |= FB_MODE_FLAG_DSSMGR;

		/* FIXME: For now add the audio flag to the sp value, which
		 *        can be done since only the lower two bits of sp
		 *        are used.
		 */
		if (audio_en)
			timings.flag |= FB_MODE_FLAG_DVI_AUDIO;

		dpy->driver->set_mode(dpy, &timings);
	}

#ifdef USE_DSSMGR_OLY_FEATURES
	dmmgr_oly_reconfig_for_dpy(&g_dev->oly_data, dpy);
#endif

failed:

	if (reenable)
		rc = dpy->driver->enable(dpy);

	return rc;
}

static int dssmgr_enable_hpd(struct dssmgr_cmd *cmd)
{
	struct omap_dss_device *dpy;
	int rc = 0;
	int idx, en;

	idx = cmd->comp_id1 - 1;
	en  = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_dpys) {
		rc = -EINVAL;
		goto failed;
	}
	if (en != 0 && en != 1) {
		rc = -EINVAL;
		goto failed;
	}

	dpy = g_dev->dpys[idx];

	if (dpy->driver == NULL || dpy->driver->set_hpd == NULL) {
		rc = -EINVAL;
		goto failed;
	}

	rc = dpy->driver->set_hpd(dpy, en);

	DMDBG("Set HPD Enable on display to %d (%s/%d)\n", en, dpy->name, rc);

failed:
	return rc;
}

#ifdef USE_DSSMGR_OLY_FEATURES
static int dssmgr_reset_oly(struct dssmgr_cmd *cmd)
{
	int rc = 0;
	int idx;
	int dis;

	idx = cmd->comp_id1 - 1;
	dis = cmd->val1;

	DMDBG("dssmgr_reset_oly/%d\n", idx);

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}

	if (dis != 0 && dis != 1) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_reset_oly(&g_dev->oly_data, idx, dis);

failed:
	return rc;
}

static int dssmgr_force_disable_oly(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, dis;

	idx = cmd->comp_id1 - 1;
	dis = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	if (dis != 0 && dis != 1) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_disable(&g_dev->oly_data, idx, dis);

failed:
	return rc;
}

static int dssmgr_force_rotation_oly(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, rot;

	idx = cmd->comp_id1 - 1;
	rot = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	switch (rot) {
	case DSSMGR_ROTATION_IGNORE:
	case DSSMGR_ROTATION_0:
	case DSSMGR_ROTATION_90:
	case DSSMGR_ROTATION_180:
	case DSSMGR_ROTATION_270:
		break;
	default:
		rc = -EINVAL;
		goto failed;
		break;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_rotation(&g_dev->oly_data, idx,
					(enum dssmgr_rotation) rot);

failed:
	return rc;
}

static int dssmgr_force_scale_oly(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, scale, percent;

	idx     = cmd->comp_id1 - 1;
	scale   = cmd->val1;
	percent = cmd->val2;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	switch (scale) {
	case DSSMGR_SCALE_IGNORE:
	case DSSMGR_SCALE_FILL_SCREEN:
	case DSSMGR_SCALE_FIT_TO_SCREEN:
	case DSSMGR_SCALE_FILL_PERCENT_OF_SCREEN:
	case DSSMGR_SCALE_FIT_PERCENT_OF_SCREEN:
	case DSSMGR_SCALE_FIT_TO_MIRROR:
	case DSSMGR_SCALE_CENTER:
		break;
	default:
		rc = -EINVAL;
		goto failed;
		break;
	}
	if ((scale == DSSMGR_SCALE_FILL_PERCENT_OF_SCREEN
	    || scale == DSSMGR_SCALE_FIT_PERCENT_OF_SCREEN
	    ) && (percent < 40 || percent > 100)) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_scale(&g_dev->oly_data, idx,
					(enum dssmgr_scale) scale, percent);

failed:
	return rc;
}

static int dssmgr_force_mirror_oly(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int s_idx, d_idx;

	s_idx = cmd->comp_id1 - 1;
	d_idx = cmd->comp_id2 - 1;

	if (s_idx < 0 || s_idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	/* The -1 case is to disable the forced mirroring */
	if (d_idx < -1 || d_idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}

	if (d_idx == -1)
		d_idx = DSSMGR_OLY_INVALID_IDX;

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_mirror(&g_dev->oly_data, s_idx, d_idx);

failed:
	return rc;
}

static int dssmgr_force_enable_oly_nf(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, en;

	idx = cmd->comp_id1 - 1;
	en = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	if (en != 0 && en != 1) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_enable_nf(&g_dev->oly_data, idx, en);

failed:
	return rc;
}

static int dssmgr_force_exclusive_oly(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, en;

	idx = cmd->comp_id1 - 1;
	en = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}
	if (en != 0 && en != 1) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_exclusive(&g_dev->oly_data, idx, en);

failed:
	return rc;
}

static int dssmgr_force_oly_disp_size(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int idx, w, h;

	idx = cmd->comp_id1 - 1;
	w = cmd->val1;
	h = cmd->val2;

	if (idx < 0 || idx >= g_dev->num_olys) {
		rc = -EINVAL;
		goto failed;
	}

	dssmgr_oly_set_client_id(&g_dev->oly_data, client_id);
	rc = dssmgr_oly_set_force_disp_size(&g_dev->oly_data, idx, w, h);

failed:
	return rc;
}

static int dssmgr_set_prefer_mirror(struct dssmgr_cmd *cmd)
{
	int rc = 0;
	int val;

	val = cmd->val1;

	rc = dssmgr_oly_set_prefer_mirror(&g_dev->oly_data, val);

	return rc;
}

static int dssmgr_force_mirror_crop_side(struct dssmgr_cmd *cmd, u32 client_id)
{
	int rc = 0;
	int side, amount;
	side   = cmd->val1;
	amount = cmd->val2;

	switch (side) {
	case DSSMGR_MIRROR_CROP_TOP:
	case DSSMGR_MIRROR_CROP_BOTTOM:
	case DSSMGR_MIRROR_CROP_RIGHT:
	case DSSMGR_MIRROR_CROP_LEFT:
		break;
	default:
		rc = -EINVAL;
		goto failed;
		break;
	}

	rc = dssmgr_oly_set_force_mirror_crop_side(&g_dev->oly_data
						   , side, amount);

failed:
	return rc;
}
#endif /* USE_DSSMGR_OLY_FEATURES */

static int dssmgr_manual_pwrctrl_dpy(struct dssmgr_cmd *cmd)
{
	struct omap_dss_device *dpy;
	int rc = 0;
	int idx, state;

	idx   = cmd->comp_id1 - 1;
	state = cmd->val1;

	if (idx < 0 || idx >= g_dev->num_dpys) {
		printk(KERN_ERR "DSSMGR: Invalid Display ID\n");
		rc = -EINVAL;
		goto failed;
	}
	if (state != 0 && state != 1 && state != 2) {
		printk(KERN_ERR "DSSMGR: Invalid Display PwrCtrl Enable\n");
		rc = -EINVAL;
		goto failed;
	}

	dpy = g_dev->dpys[idx];

	/* For now only allow HDMI devices to be manually controlled */
	if (dpy->type != OMAP_DISPLAY_TYPE_HDMI) {
		printk(KERN_ERR "DSSMGR: Can't set Non-HDMI Display\n");
		goto failed;
	} else if (dpy->manual_power_control != state) {
		dpy->manual_power_control = state;
		DMDBG("Manual Display PwrCtrl (%s/%d)\n", dpy->name, state);

		/* Ignore the disable case since the client can get the driver
		 * into the correct state before disabling.
		 */
		if (state == OMAP_DSS_MPC_SUSPEND &&
		    dpy->state != OMAP_DSS_DISPLAY_SUSPENDED) {
			if (dpy->driver->suspend)
				rc = dpy->driver->suspend(dpy);
			DMDBG("Suspend display (%s/%d)\n", dpy->name, rc);
		} else if (state == OMAP_DSS_MPC_RESUME &&
		    dpy->state == OMAP_DSS_DISPLAY_SUSPENDED) {
			if (dpy->driver->resume)
				rc = dpy->driver->resume(dpy);
			DMDBG("Resume display (%s/%d)\n", dpy->name, rc);
		}
	}

failed:
	return rc;
}

static int dssmgr_s_cmd(struct dssmgr_cmd *client_cmd, u32 client_id)
{
	struct dssmgr_cmd cmd;
	int rc = 0;
	int idx;
	struct omap_dss_device *dpy;

	/* Serialize these commands */
	mutex_lock(&g_dev->mtx);

	rc = copy_from_user(&cmd, client_cmd, sizeof(cmd));
	if (rc != 0) {
		printk(KERN_ERR "DSSMGR: S_CMD copy from user failed\n");
		goto failed;
	}

	idx = cmd.comp_id1 - 1;
	dpy = g_dev->dpys[idx];

	dsi_runtime_get();

	switch (cmd.cmd) {
	case DSSMGR_CMD_ENABLE_DPY:
		rc = dssmgr_enable_dpy(&cmd);
		break;
	case DSSMGR_CMD_ATTACH_OLY2MGR:
		rc = dssmgr_attach_oly_2_mgr(&cmd);
		break;
	case DSSMGR_CMD_ATTACH_MGR2DPY:
		rc = dssmgr_attach_mgr_2_dpy(&cmd);
		break;
	case DSSMGR_CMD_SET_RESOLUTION:
		rc = dssmgr_set_resolution(&cmd);
		break;
	case DSSMGR_CMD_ENABLE_HPD:
		rc = dssmgr_enable_hpd(&cmd);
		break;
	case DSSMGR_CMD_MANUAL_PWRCTRL_DPY:
		rc = dssmgr_manual_pwrctrl_dpy(&cmd);
		break;
#ifdef USE_DSSMGR_OLY_FEATURES
	case DSSMGR_CMD_RESET_OLY:
		rc = dssmgr_reset_oly(&cmd);
		break;
	case DSSMGR_CMD_FORCE_DISABLE_OLY:
		rc = dssmgr_force_disable_oly(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_ROTATION_OLY:
		rc = dssmgr_force_rotation_oly(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_SCALE_OLY:
		rc = dssmgr_force_scale_oly(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_MIRROR_OLY:
		rc = dssmgr_force_mirror_oly(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_ENABLE_OLY_NEXT_FRAME:
		rc = dssmgr_force_enable_oly_nf(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_EXCLUSIVE_OLY:
		rc = dssmgr_force_exclusive_oly(&cmd, client_id);
		break;
	case DSSMGR_CMD_FORCE_OLY_DISP_SIZE:
		rc = dssmgr_force_oly_disp_size(&cmd, client_id);
		break;
	case DSSMGR_CMD_PREFER_MIRROR:
		rc = dssmgr_set_prefer_mirror(&cmd);
		break;
	case DSSMGR_CMD_FORCE_MIRROR_CROP_SIDE:
		rc = dssmgr_force_mirror_crop_side(&cmd, client_id);
		break;
	case DSSMGR_CMD_PAUSE_MUTE_OLY:
		rc = -EINVAL; /* Not Supported */
		break;
#else
	case DSSMGR_CMD_RESET_OLY:
	case DSSMGR_CMD_FORCE_DISABLE_OLY:
	case DSSMGR_CMD_FORCE_ROTATION_OLY:
	case DSSMGR_CMD_FORCE_SCALE_OLY:
	case DSSMGR_CMD_FORCE_MIRROR_OLY:
	case DSSMGR_CMD_FORCE_ENABLE_OLY_NEXT_FRAME:
	case DSSMGR_CMD_FORCE_EXCLUSIVE_OLY:
	case DSSMGR_CMD_FORCE_OLY_DISP_SIZE:
	case DSSMGR_CMD_PREFER_MIRROR:
	case DSSMGR_CMD_FORCE_MIRROR_CROP_SIDE:
	case DSSMGR_CMD_PAUSE_MUTE_OLY:
		printk(KERN_ERR "DSSMGR: Overlay Features Not Supported\n");
		rc = -EINVAL; /* Not Supported */
		break;
#endif
	default:
		printk(KERN_ERR "DSSMGR: Invalid CMD (%x)\n", cmd.cmd);
		rc = -EINVAL;
		break;
	}

	dsi_runtime_put();

failed:
	mutex_unlock(&g_dev->mtx);

	return rc;
}

static int dssmgr_querycfg(struct dssmgr_cfg *client_cfg)
{
	struct dssmgr_cfg            cfg;
	struct omap_dss_device      *dssdev;
	struct dssmgr_display       *dpy;
	struct omap_overlay_manager *dssmgr;
	struct dssmgr_manager       *mgr;
	struct omap_overlay         *dssoly;
	struct dssmgr_overlay       *oly;
	int i, j;

	memset(&cfg, 0, sizeof(struct dssmgr_cfg));

	for (i = 0; i < g_dev->num_dpys; i++) {
		dssdev = g_dev->dpys[i];
		dpy    = &cfg.displays[i];
		dpy->id = i + 1;
		strncpy(dpy->name, dssdev->name, DSSMGR_MAX_NAME_SIZE);
		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
			dpy->flags |= DSSMGR_DPY_FLAG_ACTIVE;
		if (dssdev->driver->get_edid != NULL) {
			dpy->flags |= DSSMGR_DPY_FLAG_EDID;
			if (dssdev->platform_enable_hpd != NULL)
				dpy->flags |= DSSMGR_DPY_FLAG_HOTPLUG;
		}

		dpy->resolution.width  = dssdev->panel.timings.x_res;
		dpy->resolution.height = dssdev->panel.timings.y_res;
		dpy->resolution.rate   = 0;
		dpy->resolution.flags  = 0;
	}

	for (i = 0; i < g_dev->num_mgrs; i++) {
		dssmgr = g_dev->mgrs[i];
		mgr    = &cfg.managers[i];
		mgr->id = i + 1;
		strncpy(mgr->name, dssmgr->name, DSSMGR_MAX_NAME_SIZE);
		for (j = 0; j < g_dev->num_dpys; j++) {
			if (dssmgr->device == g_dev->dpys[j]) {
				mgr->disp_id = j + 1;
				break;
			}
		}
	}

	for (i = 0; i < g_dev->num_olys; i++) {
		dssoly = g_dev->olys[i];
		oly    = &cfg.overlays[i];
		oly->id = i + 1;
		strncpy(oly->name, dssoly->name, DSSMGR_MAX_NAME_SIZE);
		for (j = 0; j < g_dev->num_mgrs; j++) {
			if (dssoly->manager == g_dev->mgrs[j]) {
				oly->mgr_id = j + 1;
				break;
			}
		}
		oly->active = dssoly->info.enabled;

#ifdef USE_DSSMGR_OLY_FEATURES
		/* Client Enabled Flag */
		oly->rsvd[0] = (u8) g_dev->oly_data.info[i].enabled;
		/* Force Disable Flag */
		oly->rsvd[1] = (u8) g_dev->oly_data.flds[i].force_disable;
		/* Force Rotation */
		oly->rsvd[2] = (u8) g_dev->oly_data.flds[i].force_rotation;
		/* Force Scale */
		oly->rsvd[3] = (u8) g_dev->oly_data.flds[i].force_scale;
		/* Force Mirror Destination Index */
		oly->rsvd[4] = g_dev->oly_data.flds[i].force_mirror_dst_idx;
		/* Force Enable on Next Client Frame */
		oly->rsvd[5] = (u8) g_dev->oly_data.flds[i].force_enable_nf;
		/* Force Exclusive Overlay */
		oly->rsvd[6] = (u8) g_dev->oly_data.flds[i].force_exclusive;
		/* Client Frame Count */
		oly->fcnt    = (u8) g_dev->oly_data.flds[i].fcnt;
#endif
	}

	return copy_to_user(client_cfg, &cfg, sizeof(cfg));
}

static int dssmgr_g_edid(struct dssmgr_edid *client_edid)
{
	static u8 edid[EDID_MAX_SIZE];

	struct omap_dss_device *dssdev;
	struct dssmgr_edid      c_edid;
	int rc = 0;
	int idx;
	int blk;
	int max;
	int i;

	rc = copy_from_user(&c_edid, client_edid, sizeof(c_edid));
	if (rc != 0) {
		printk(KERN_ERR "DSSMGR: G_EDID copy from user failed\n");
		goto failed;
	}

	idx = c_edid.dpy_id - 1;
	blk = c_edid.block;

	if (idx < 0 || idx >= g_dev->num_dpys) {
		printk(KERN_ERR "DSSMGR: G_EDID invalid display idx\n");
		rc = -EINVAL;
		goto failed;
	}

	dssdev = g_dev->dpys[idx];

	if (dssdev->driver->get_edid == NULL)	{
		printk(KERN_ERR "DSSMGR: G_EDID non-EDID display\n");
		rc = -EINVAL;
		goto failed;
	}

	if (blk < 0 && blk >= EDID_MAX_BLOCKS) {
		printk(KERN_ERR "DSSMGR: G_EDID invalid EDID block\n");
		rc = -EINVAL;
		goto failed;
	}

	/* Due to an issue in the TI driver, if you try to read the EDID
	 * a second time the data shifts, thus we cache all the EDID data
	 * when the first block is requested and read from the cache
	 * when the further blocks are read.
	 * The TI bug for this issue is CSR OMAPS00228236.
	 */
	if (blk == 0) {
		if (dssdev->driver->get_edid(dssdev, edid,
						EDID_MAX_SIZE) != 0) {
			printk(KERN_WARNING "HDMI failed to read EDID\n");
			rc = -EIO;
			goto failed;
		}
	}

	for (i = 0; i < EDID_HEADER_SIZE; i++) {
		if (edid[i] == header[i])
			continue;
		break;
	}

	if (i != EDID_HEADER_SIZE) {
		printk(KERN_WARNING "HDMI EDID Invalid (%d)\n", i);
		rc = -EIO;
		goto failed;
	}

	max = edid[EDID_NUM_EXT_BLKS];
	if (max > (EDID_MAX_BLOCKS - 1))
		max = (EDID_MAX_BLOCKS - 1);

	if (blk > max) {
		printk(KERN_ERR "Invalid EDID Block Requested\n");
		rc = -EINVAL;
	} else {
		memcpy(c_edid.data, &edid[blk * EDID_BLOCK_SIZE],
							EDID_BLOCK_SIZE);
	}

	if (rc == 0)
		rc = copy_to_user(client_edid, &c_edid, sizeof(c_edid));

failed:
	return rc;
}

/*=== Driver Interface Functions =======================================*/

static int dssmgr_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	if (g_dev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&g_dev->mtx);

	file->private_data = (void *) g_dev->client_next_id++;
	g_dev->client_cnt++;

	DMDBG("dssmgr_open/%d/%d\n", g_dev->client_cnt, g_dev->client_next_id);

	mutex_unlock(&g_dev->mtx);
	return rc;
}

static int dssmgr_release(struct inode *inode, struct file *file)
{
	int rc = 0;
	u32 client_id;
	int i;

	if (g_dev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	client_id = (u32) file->private_data;

	mutex_lock(&g_dev->mtx);

	DMDBG("dssmgr_release/%d/%d\n", g_dev->client_cnt, client_id);

	if (g_dev->client_cnt > 0)
		g_dev->client_cnt--;

#ifdef USE_DSSMGR_OLY_FEATURES
	if (g_dev->client_cnt == 0)
		dssmgr_oly_reset(&g_dev->oly_data);
	else
		dssmgr_oly_reset_client(&g_dev->oly_data, client_id);
#endif

	/* Since only this component will place a display into manual power
	 * control mode, clear the flag here to ensure power control is
	 * handled normally
	 */
	for (i = 0; i < g_dev->num_dpys; i++) {
		g_dev->dpys[i]->manual_power_control = OMAP_DSS_MPC_DISABLED;

		if (g_dev->dpys[i]->driver->set_hpd != NULL)
			g_dev->dpys[i]->driver->set_hpd(g_dev->dpys[i], false);
	}

	mutex_unlock(&g_dev->mtx);

	return rc;
}

static long dssmgr_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	long rc = 0;
	u32 client_id;

	if (unlikely(_IOC_TYPE(cmd) != DSSMGR_IOCTL_MAGIC)) {
		printk(KERN_ERR "DSSMGR: Bad command value (%d)\n", cmd);
		return -EINVAL;
	}

	switch (cmd) {
	case DSSMGR_QUERYCFG:
		rc = dssmgr_querycfg((struct dssmgr_cfg *) arg);
		break;
	case DSSMGR_S_CMD:
		client_id = (u32) f->private_data;
		DMDBG("dssmgr_ioctl/cmd/%d/%d\n", client_id,
					((struct dssmgr_cmd *) arg)->cmd);
		rc = dssmgr_s_cmd((struct dssmgr_cmd *) arg, client_id);
		break;
	case DSSMGR_G_EDID:
		rc = dssmgr_g_edid((struct dssmgr_edid *) arg);
		break;
	default:
		printk(KERN_ERR "DSSMGR: Invalid ioctl (%x)\n", cmd);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static const struct file_operations dssmgr_fops = {
	.owner		= THIS_MODULE,
	.open		= dssmgr_open,
	.release	= dssmgr_release,
	.unlocked_ioctl	= dssmgr_ioctl,
};

static struct miscdevice dssmgr_misc_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE_NAME,
	.fops	= &dssmgr_fops,
};

static int __init dssmgr_probe(struct platform_device *pdev)
{
	struct omap_dss_device *dssdev;
	int rc = 0;
	int i, n;

	DMDBG("dssmgr_probe\n");

	g_dev = (struct dssmgr_device *)
			kzalloc(sizeof(struct dssmgr_device), GFP_KERNEL);
	if (g_dev == NULL)
		return -ENOMEM;

	memset(g_dev, 0, sizeof(g_dev));

	mutex_init(&g_dev->mtx);

	g_dev->client_next_id = 1;
	g_dev->client_cnt     = 0;

	rc = misc_register(&dssmgr_misc_device);
	if (rc) {
		printk(KERN_ERR "dssmgr: misc register failed (%d)\n", rc);
		goto failed_dev;
	}

	g_dev->num_dpys = 0;
	dssdev = NULL;
	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);
		if (g_dev->num_dpys < DSSMGR_MAX_DISPLAYS)
			g_dev->dpys[g_dev->num_dpys++] = dssdev;
	}

	if (g_dev->num_dpys <= 0) {
		dev_err(&pdev->dev, "No displays\n");
		rc = -EINVAL;
		goto failed_dpy;
	}

	n = omap_dss_get_num_overlay_managers();
	g_dev->num_mgrs = (n < DSSMGR_MAX_MANAGERS) ? n : DSSMGR_MAX_MANAGERS;
	for (i = 0; i < g_dev->num_mgrs; i++)
		g_dev->mgrs[i] = omap_dss_get_overlay_manager(i);

	n = omap_dss_get_num_overlays();
	g_dev->num_olys = (n < DSSMGR_MAX_OVERLAYS) ? n : DSSMGR_MAX_OVERLAYS;
	for (i = 0; i < g_dev->num_olys; i++)
		g_dev->olys[i] = omap_dss_get_overlay(i);

#ifdef USE_DSSMGR_OLY_FEATURES
	rc = dssmgr_oly_init(&g_dev->oly_data);
	if (rc != 0) {
		printk(KERN_ERR "dssmgr-oly init failed\n");
		goto failed_dpy;
	}
#endif

	g_dev->dev = device_create(g_dev->cls, g_dev->dev,
				MKDEV(g_dev->major, 0), NULL, DEVICE_NAME);

	return 0;

failed_dpy:
	misc_deregister(&dssmgr_misc_device);
failed_dev:
	kfree(g_dev);
	g_dev = NULL;
	return rc;
}

static int dssmgr_remove(struct platform_device *pdev)
{
	DMDBG("dssmgr_remove\n");

	if (g_dev) {
#ifdef USE_DSSMGR_OLY_FEATURES
		dssmgr_oly_remove(&g_dev->oly_data);
#endif
		misc_deregister(&dssmgr_misc_device);
		kfree(g_dev);
		g_dev = NULL;
	}

	return 0;
}

static struct platform_driver dssmgr_driver = {
	.remove		= dssmgr_remove,
	.driver		= {
		.name   = DEVICE_NAME,
	},
};

static int __init dssmgr_init(void)
{
	int rc;

	DMDBG("dssmgr_init\n");

	rc = platform_driver_probe(&dssmgr_driver, dssmgr_probe);
	if (rc != 0) {
		printk(KERN_ERR "failed dssmgr register/probe %d\n", rc);
		return -ENODEV;
	}

	return 0;
}

static void __exit dssmgr_exit(void)
{
	DMDBG("dssmgr_exit\n");

	platform_driver_unregister(&dssmgr_driver);
}

device_initcall_sync(dssmgr_init);
module_exit(dssmgr_exit);

MODULE_DESCRIPTION("DSS2 Manager");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

