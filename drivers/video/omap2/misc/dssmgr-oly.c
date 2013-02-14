/*
 * drivers/video/omap2/misc/dssmgr-oly.c
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
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <video/omapdss.h>
#include <plat/dma.h>
#include <mach/tiler.h>

#include <linux/omap-dssmgr.h>
#include "dssmgr-oly.h"

/*#define REMOVED_FOR_ICS_BRINGUP*/
/*#define DMO_DEBUG*/
#ifdef DMO_DEBUG
#define DMODBG(format, ...) \
	printk(KERN_INFO "DSSMGR: " format, ## __VA_ARGS__)
#else
#define DMODBG(format, ...)
#endif

#define DMA_TX_TIMEOUT   (1000)

struct dssmgr_oly_data *g_data;

static int dssmgr_oly_override_and_set(struct dssmgr_oly_data *data,
				int idx, struct omap_overlay_info *info);
static int dssmgr_oly_set_info(struct dssmgr_oly_data *data,
				struct omap_overlay *oly,
				struct omap_overlay_info *info);

/*=== Local Functions ==================================================*/

static int dssmgr_oly_set_and_apply(struct dssmgr_oly_data *data, uint8_t idx)
{
	struct omap_overlay_info  info;
	struct omap_dss_device   *dev;
	int rc = 0;

	if (data->mirror.dst_idx != idx)
		memcpy(&info, &data->info[idx], sizeof(info));
	else
		data->flds[idx].get_func(data->olys[idx], &info);

	rc = dssmgr_oly_override_and_set(data, idx, &info);
	if (rc != 0) {
		printk(KERN_ERR "DSSMGR-OLY: Set Info Failure\n");
	} else {
		rc = data->olys[idx]->manager->apply(
					data->olys[idx]->manager);
		if (rc != 0) {
			printk(KERN_ERR "DSSMGR-OLY: Apply Failure\n");
		} else {
			dev = data->olys[idx]->manager->device;
			/* Temporarily check if the driver is in manual mode
			 * because there is an issue in the tablet dss driver
			 * where it hangs if update() is called while in
			 * automatic mode.
			 * TODO: This check will be removed once
			 * that issue is resolved.
			 */
			if (dev && dev->driver && dev->driver->update &&
			    dev->driver->get_update_mode &&
			    (dev->driver->get_update_mode(dev) ==
			    OMAP_DSS_UPDATE_MANUAL)) {
				rc = dev->driver->update(dev, 0, 0,
						dev->panel.timings.x_res,
						dev->panel.timings.y_res);
			}
		}
	}
	return rc;
}

static int dssmgr_oly_disable_oly(struct dssmgr_oly_data *data, uint8_t idx)
{
	struct omap_overlay      *oly;
	struct omap_overlay_info  info;
	struct omap_dss_device   *dev;
	int rc = 0;

	oly = data->olys[idx];
	data->flds[idx].get_func(oly, &info);
	if (info.enabled != false) {
		info.enabled = false;
		data->flds[idx].set_func(oly, &info);
		oly->manager->apply(oly->manager);
		dev = oly->manager->device;
		/* Temporarily check if the driver is in manual mode
		 * because there is an issue in the tablet dss driver
		 * where it hangs if update() is called while in
		 * automatic mode.
		 * TODO: This check will be removed once
		 * that issue is resolved.
		 */
		if (dev && dev->driver && dev->driver->update &&
		    dev->driver->get_update_mode &&
		    (dev->driver->get_update_mode(dev) ==
		    OMAP_DSS_UPDATE_MANUAL)) {
			dev->driver->update(dev, 0, 0,
					dev->panel.timings.x_res,
					dev->panel.timings.y_res);
		}
	}

	return rc;
}

static void dssmgr_oly_get_dpy_size(struct dssmgr_oly_data *data,
						uint8_t idx, int *w, int *h)
{
	if (data->olys[idx]->manager->device) {
		*w = data->olys[idx]->manager->device->panel.timings.x_res;
		*h = data->olys[idx]->manager->device->panel.timings.y_res;
	}
}

static void dssmgr_oly_get_norm_input(struct omap_overlay_info *info,
						int *in_w, int *in_h)
{
	int rot = info->rotation;

	if (rot == OMAP_DSS_ROT_90 || rot == OMAP_DSS_ROT_270) {
		*in_w  = info->height;
		*in_h  = info->width;
	} else {
		*in_w  = info->width;
		*in_h  = info->height;
	}
}

static void dssmgr_oly_rot_input(int rot, int *in_w, int *in_h)
{
	int t;

	if (rot == OMAP_DSS_ROT_90 || rot == OMAP_DSS_ROT_270) {
		t      = *in_w;
		*in_w  = *in_h;
		*in_h  = t;
	}
}

static void dssmgr_oly_non_square_pixel_chk(int src_h, int dst_w, int dst_h,
								int *src_w)
{
	int w;
	int d;

	/* Check for aspect ratio changes based on the source and destination
	 * size information.  If the aspect ratio is changing, change the input
	 * size to reflect the aspect ratio change.
	 *
	 * An aspect ratio change is detected by calculating the source width
	 * based on the other info, as in:
	 *
	 *   w     dst_w
	 * ----- = -----  or  w = (src_h * dst_w) / dst_h
	 * src_h   dst_h
	struct omap_overlay      *oly;
	 */
	w = (int) ((((uint32_t) src_h) * ((uint32_t) dst_w)) / dst_h);

	/* There are some cases where the difference is small and we likely
	 * do *not* want to change, specifically thinking about some camera
	 * preview use cases, so add some wiggle room
	 */
	d = w - *src_w;
	if (d < -8 || d > 8)
		*src_w = w;
}

static void dssmgr_oly_scale(int scale, int percent,
			     int dpy_w, int dpy_h, int in_w, int in_h,
			     int mir_x, int mir_y, int mir_w, int mir_h,
			     int *out_x, int *out_y, int *out_w, int *out_h)
{
	int w, h;
	int tw, th;
	int ratio;
	const int shift = 10; /* Fixed point shift factor */
	const int ratio_limit = (4 << shift); /* 4X scaling limit */

	if (scale == DSSMGR_SCALE_FILL_SCREEN) {
		*out_x = 0;
		*out_y = 0;
		*out_w = dpy_w;
		*out_h = dpy_h;
	} else if (scale == DSSMGR_SCALE_FILL_PERCENT_OF_SCREEN) {
		w = (dpy_w * percent)/100;
		h = (dpy_h * percent)/100;
		*out_x = (dpy_w - w) / 2;
		*out_y = (dpy_h - h) / 2;
		*out_w = w;
		*out_h = h;
	} else if (scale == DSSMGR_SCALE_FIT_TO_SCREEN) {
		tw = (dpy_w << shift) / in_w;
		th = (dpy_h << shift) / in_h;
		ratio = (tw < th) ? tw : th;
		if (ratio > ratio_limit)
			ratio = ratio_limit;
		w = (in_w * ratio) >> shift;
		h = (in_h * ratio) >> shift;
		w = (w + 1) & ~1;
		h = (h + 1) & ~1;

		*out_x = (dpy_w - w) / 2;
		*out_y = (dpy_h - h) / 2;
		*out_w = w;
		*out_h = h;
	} else if (scale == DSSMGR_SCALE_FIT_PERCENT_OF_SCREEN) {
		tw = (((dpy_w * percent)/100) << shift) / in_w;
		th = (((dpy_h * percent)/100) << shift) / in_h;
		ratio = (tw < th) ? tw : th;
		if (ratio > ratio_limit)
			ratio = ratio_limit;
		w = (in_w * ratio) >> shift;
		h = (in_h * ratio) >> shift;
		w = (w + 1) & ~1;
		h = (h + 1) & ~1;

		*out_x = (dpy_w - w) / 2;
		*out_y = (dpy_h - h) / 2;
		*out_w = w;
		*out_h = h;
	} else if (scale == DSSMGR_SCALE_FIT_TO_MIRROR) {
		tw = (mir_w << shift) / in_w;
		th = (mir_h << shift) / in_h;
		ratio = (tw < th) ? tw : th;
		if (ratio > ratio_limit)
			ratio = ratio_limit;
		w = (in_w * ratio) >> shift;
		h = (in_h * ratio) >> shift;
		w = (w + 1) & ~1;
		h = (h + 1) & ~1;

		*out_x = ((mir_w - w) / 2) + mir_x;
		*out_y = ((mir_h - h) / 2) + mir_y;
		*out_w = w;
		*out_h = h;
	} else if (scale == DSSMGR_SCALE_CENTER) {
		if (in_w <= dpy_w && in_h <= dpy_h) {
			*out_x = ((dpy_w - in_w) / 2);
			*out_y = ((dpy_h - in_h) / 2);
			*out_w = in_w;
			*out_h = in_h;
		} else {
			*out_x = 0;
			*out_y = 0;
			*out_w = dpy_w;
			*out_h = dpy_h;
		}
	}
}

static void dssmgr_oly_override(struct dssmgr_oly_data *data, uint8_t idx,
				struct omap_overlay_info *info)
{
	struct dssmgr_oly_flds   *flds;
	struct omap_overlay      *oly;
	struct dssmgr_oly_mirror *mirror;
	int in_w, in_h;
	int out_x, out_y, out_w, out_h;
	int dpy_w = 0, dpy_h = 0;
	int rot;
	int m_idx;
	int scale;
	int adj_w, adj_h;

	flds = &data->flds[idx];

	/* Always enable the mirrored UI frame.  It can sometimes be false
	 * here, mostly due to force disables.
	 */
	if ((data->mirror.dst_idx == idx) && (data->mirror.updating == true))
		info->enabled = true;

	if (data->client_frame && flds->force_enable_nf)
		info->enabled = true;

	if (flds->force_disable)
		info->enabled = false;

	/* Gather some info */
	dssmgr_oly_get_dpy_size(data, idx, &dpy_w, &dpy_h);

	in_w  = info->width;
	in_h  = info->height;
	rot   = info->rotation;
	out_x = info->pos_x;
	out_y = info->pos_y;
	out_w = info->out_width;
	out_h = info->out_height;

	mirror = &data->mirror;
	scale = flds->force_scale;

	/* Only for FIT_TO_MIRROR overlays (just the video/cam oly),
	 * if mirroring is disabled use a different scaling type.
	 */
	if (scale == DSSMGR_SCALE_FIT_TO_MIRROR &&
	    mirror->src_idx == DSSMGR_OLY_INVALID_IDX)
		scale = DSSMGR_SCALE_FIT_TO_SCREEN;

	if (info->rotation_type == OMAP_DSS_ROT_TILER) {
		dssmgr_oly_get_norm_input(info, &in_w, &in_h);

		if (flds->force_rotation != DSSMGR_ROTATION_IGNORE) {
			switch (flds->force_rotation) {
			case DSSMGR_ROTATION_90:
				rot = OMAP_DSS_ROT_90;
				break;
			case DSSMGR_ROTATION_180:
				rot = OMAP_DSS_ROT_180;
				break;
			case DSSMGR_ROTATION_270:
				rot = OMAP_DSS_ROT_270;
				break;
			case DSSMGR_ROTATION_0:
			default:
				rot = OMAP_DSS_ROT_0;
				break;
			}
		}
		dssmgr_oly_rot_input(rot, &in_w, &in_h);
	}

	/* Adjust input window to account for forced cropping
	 * i.e. status bar removal
	 */
	if ((data->mirror.updating == true)
		&& (data->mirror.force_crop_amount != 0)) {
		u32 crop_amount = data->mirror.force_crop_amount;

		switch (data->mirror.force_crop_side) {
		case DSSMGR_MIRROR_CROP_BOTTOM:
			/* Account for rotation */
			if ((rot == OMAP_DSS_ROT_90)
			    || rot == OMAP_DSS_ROT_270) {
				in_w = mirror->h;
				in_h = mirror->w - crop_amount;
			} else {
				in_w = mirror->w;
				in_h = mirror->h - crop_amount;
			}
			break;
		case DSSMGR_MIRROR_CROP_TOP:
		case DSSMGR_MIRROR_CROP_RIGHT:
		case DSSMGR_MIRROR_CROP_LEFT:
			/* Not Supported */
			break;
		}
	 }

	adj_w = in_w;
	adj_h = in_h;
	if (scale == DSSMGR_SCALE_FIT_TO_MIRROR) {
		int ow, oh;

		/* Only for FIT_TO_MIRROR overlays (just the video/cam oly),
		 * check and adjust for non-square pixels.
		 */

		/* Adjust the output w/h to correspond to any input change */
		ow = out_w;
		oh = out_h;
		if (in_w != info->width) {
			ow = out_h;
			oh = out_w;
		}

		dssmgr_oly_non_square_pixel_chk(adj_h, ow, oh, &adj_w);
	}

	dssmgr_oly_scale(flds->force_scale, flds->force_scale_percent,
			 dpy_w, dpy_h, adj_w, adj_h,
			 mirror->dst_x, mirror->dst_y,
			 mirror->dst_w, mirror->dst_h,
			 &out_x, &out_y, &out_w, &out_h);

	info->width        = in_w;
	info->height       = in_h;
	info->rotation     = rot;
	info->pos_x        = out_x;
	info->pos_y        = out_y;
	info->out_width    = out_w;
	info->out_height   = out_h;

	if (info->rotation_type == OMAP_DSS_ROT_TILER)
		info->screen_width = info->width;

	/* Check if the mirror exclusive overlay is active, if so, disable the
	 * mirror overlay
	 */
	if (data->mirror.src_idx != DSSMGR_OLY_INVALID_IDX &&
			idx == data->mirror.excl_idx && info->enabled) {
		m_idx = data->mirror.dst_idx;
		oly = data->olys[m_idx];
		if (oly->info.enabled != false)
			dssmgr_oly_disable_oly(data, m_idx);
	}
}

/* Similar to "dssmgr_oly_set_info" except the info is not from the client */
static int dssmgr_oly_override_and_set(struct dssmgr_oly_data *data, int idx,
					struct omap_overlay_info *info)
{
	/* Override as needed */
	dssmgr_oly_override(data, idx, info);

	/* Set the info to the DSS */
	return data->flds[idx].set_func(data->olys[idx], info);
}

static int dssmgr_oly_set_info(struct dssmgr_oly_data *data,
				struct omap_overlay *oly,
				struct omap_overlay_info *info)
{
	struct omap_overlay_info local_info;
	int rc = 0;
	int i;
	uint8_t idx = DSSMGR_OLY_INVALID_IDX;

	for (i = 0; i < DSSMGR_MAX_OVERLAYS; i++) {
		if (data->olys[i] == oly) {
			idx = i;
			break;
		}
	}

	if (idx == DSSMGR_OLY_INVALID_IDX) {
		rc = -EINVAL;
		goto failed;
	}

	/* Skip to the end if mirroring */
	if ((data->client_frame) && (i == data->mirror.dst_idx) &&
	    (data->prefer_mirror == 1))
		goto failed;

	data->flds[idx].fcnt++;

	/* Save as most recent client provide info */
	memcpy(&data->info[idx], info, sizeof(*info));

	/* Don't use the client provided pointer since we may change the data
	 * and that may be reflected in the client kept data structure
	 */
	memcpy(&local_info, info, sizeof(*info));

	/* Catch the case where some client tries to update the mirror overlay
	 * while the mirroring is active.  In this case, just force an update
	 * of the overlay settings on the next mirrored frame.
	 */
	if (idx == data->mirror.dst_idx
	   && data->mirror.updating == false
	   && data->client_frame) {
		printk(KERN_WARNING "DSSMGR-OLY: Force mirror oly update\n");
		data->mirror.force_update = true;
	}

	rc = dssmgr_oly_override_and_set(data, idx, &local_info);

failed:
	return rc;
}

/* This is the substitute "set_overlay_info" function.
 * All calls to the "set_overlay_info" function of any overlay will
 * come here, so the overlay info can be adjusted as requested by
 * the client
 */
static int dssmgr_oly_set_info_lock(struct omap_overlay *oly,
				struct omap_overlay_info *info)
{
	int rc = 0;

	mutex_lock(&g_data->mtx);

	g_data->client_frame = 1;

	rc = dssmgr_oly_set_info(g_data, oly, info);

	g_data->client_frame = 0;

	mutex_unlock(&g_data->mtx);

	return rc;
}

/* This is the substitute "get_overlay_info" function.
 * All calls to the "get_overlay_info" function of any overlay will
 * come here, so the correct overlay info can be returned
 */
static void dssmgr_oly_get_info_lock(struct omap_overlay *oly,
				struct omap_overlay_info *info)
{
	int i;

	mutex_lock(&g_data->mtx);

	for (i = 0; i < DSSMGR_MAX_OVERLAYS; i++) {
		if (g_data->olys[i] == oly) {
			/* Return most recently set info */
			memcpy(info, &g_data->info[i], sizeof(*info));
			break;
		}
	}

	mutex_unlock(&g_data->mtx);
}

void dssmgr_oly_reset_oly_idx(struct dssmgr_oly_data *data, uint8_t idx)
{
	data->flds[idx].force_disable        = 0;
	data->flds[idx].force_rotation       = DSSMGR_ROTATION_IGNORE;
	data->flds[idx].force_scale          = DSSMGR_SCALE_IGNORE;
	data->flds[idx].force_mirror_dst_idx = DSSMGR_OLY_INVALID_IDX;
	data->flds[idx].force_enable_nf      = 0;
	data->flds[idx].force_exclusive      = 0;
	data->flds[idx].force_disp_size      = 0;
#ifdef REMOVED_FOR_ICS_BRINGUP
	omap_vout_override_disp_size(idx, 0, 0);
#endif
	data->flds[idx].fcnt                 = 0;
	if (data->mirror.excl_idx == idx)
		data->mirror.excl_idx = DSSMGR_OLY_INVALID_IDX;
}

/*=== Interface Functions ==============================================*/

int dssmgr_oly_init(struct dssmgr_oly_data *data)
{
	struct omap_overlay *oly;
	int rc = 0;
	int num;
	int i;

	BUG_ON(data == NULL);

	if (g_data != NULL) {
		printk(KERN_ERR "dssmgr-oly already init'd\n");
		rc = -EINVAL;
		goto failed;
	}

	/* Store for use in the set/get_info_lock functions */
	g_data = data;

	mutex_init(&data->mtx);

	/* Hook into the set/get_overlay_info call paths */
	num = omap_dss_get_num_overlays();
	num = (num < DSSMGR_MAX_OVERLAYS) ? num : DSSMGR_MAX_OVERLAYS;
	for (i = 0; i < num; i++) {
		oly = omap_dss_get_overlay(i);
		data->olys[i] = oly;
		memset(&data->flds[i], 0, sizeof(data->flds[i]));
		data->flds[i].set_func = oly->set_overlay_info;
		data->flds[i].get_func = oly->get_overlay_info;
		oly->set_overlay_info = dssmgr_oly_set_info_lock;
		oly->get_overlay_info = dssmgr_oly_get_info_lock;
		data->flds[i].get_func(oly, &data->info[i]);
	}
	data->num = num;

	/* Reset the oly fields */
	for (i = 0; i < data->num; i++)
		dssmgr_oly_reset_oly_idx(data, i);

	/* Init the mirror indexes */
	data->mirror.src_idx  = DSSMGR_OLY_INVALID_IDX;
	data->mirror.dst_idx  = DSSMGR_OLY_INVALID_IDX;
	data->mirror.excl_idx = DSSMGR_OLY_INVALID_IDX;

	/* Reset mirror overlay preference */
	data->prefer_mirror = 0;

	/* Reset mirror force crop amount */
	data->mirror.force_crop_amount = 0;

	return 0;

failed:
	return rc;
}

void dssmgr_oly_remove(struct dssmgr_oly_data *data)
{
	struct omap_overlay *oly;
	int i;

	BUG_ON(data == NULL);

	if (g_data) {
		omap_free_dma(data->mirror.dma_ch);

		for (i = 0; i < data->num; i++) {
			oly = data->olys[i];
			oly->set_overlay_info = data->flds[i].set_func;
			oly->get_overlay_info = data->flds[i].get_func;
			oly->set_overlay_info(oly, &data->info[i]);
		}

		g_data = NULL;
	}
}

void dssmgr_oly_set_client_id(struct dssmgr_oly_data *data, u32 id)
{
	BUG_ON(data == NULL);

	DMODBG("dssmgr_oly_set_client_id/%d\n", id);

	mutex_lock(&data->mtx);

	data->client_id = id;

	mutex_unlock(&data->mtx);
}

void dssmgr_oly_reset(struct dssmgr_oly_data *data)
{
	int i;

	BUG_ON(data == NULL);

	data->prefer_mirror = 0;

	data->mirror.force_crop_amount = 0;

	for (i = 0; i < data->num; i++)
		dssmgr_oly_reset_oly(data, i, 0);
}

void dssmgr_oly_reset_oly(struct dssmgr_oly_data *data, uint8_t idx, int dis)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	DMODBG("dssmgr_oly_reset_oly/%d/%d\n", idx, dis);

	mutex_lock(&data->mtx);

	dssmgr_oly_reset_oly_idx(data, idx);
	data->flds[idx].force_disable = dis; /* Force disable if requested */
	dssmgr_oly_set_and_apply(data, idx);

	mutex_unlock(&data->mtx);
}

void dssmgr_oly_reset_client(struct dssmgr_oly_data *data, u32 id)
{
	struct dssmgr_oly_flds *flds;
	int i;
	int dirty;

	BUG_ON(data == NULL);

	DMODBG("dssmgr_oly_reset_client/%d\n", id);

	for (i = 0; i < data->num; i++) {
		dirty = 0;
		flds = &data->flds[i];

		if (flds->disable_client_id == id) {
			flds->force_disable = 0;
			dirty = 1;
		}
		if (flds->rotation_client_id == id) {
			flds->force_rotation = DSSMGR_ROTATION_IGNORE;
			dirty = 1;
		}
		if (flds->scale_client_id == id) {
			flds->force_scale = DSSMGR_SCALE_IGNORE;
			dirty = 1;
		}
		if (flds->mirror_client_id == id) {
			flds->force_mirror_dst_idx = DSSMGR_OLY_INVALID_IDX;
			dirty = 1;
		}
		if (flds->enable_nf_client_id == id) {
			flds->force_enable_nf = 0;
			dirty = 1;
		}
		if (flds->exclusive_client_id == id) {
			flds->force_exclusive = 0;
			if (data->mirror.excl_idx == i)
				data->mirror.excl_idx = DSSMGR_OLY_INVALID_IDX;
			dirty = 1;
		}
		if (flds->disp_size_client_id == id) {
			flds->force_disp_size = 0;
#ifdef REMOVED_FOR_ICS_BRINGUP
			omap_vout_override_disp_size(i, 0, 0);
#endif
		}

		if (dirty)
			dssmgr_oly_set_and_apply(data, i);
	}
}

int dssmgr_oly_get_force_disable(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[idx].force_disable;
}

int dssmgr_oly_set_force_disable(struct dssmgr_oly_data *data, uint8_t idx,
							uint8_t dis)
{
	int rc = 0;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);
	BUG_ON(dis > 1);

	/* Verify a change is occuring */
	if (data->flds[idx].force_disable != dis) {
		mutex_lock(&data->mtx);

		data->flds[idx].force_disable = dis;
		data->flds[idx].disable_client_id = data->client_id;
		rc = dssmgr_oly_set_and_apply(data, idx);

		mutex_unlock(&data->mtx);
	}

	return rc;
}

int dssmgr_oly_get_force_rotation(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[idx].force_rotation;
}

int dssmgr_oly_set_force_rotation(struct dssmgr_oly_data *data, uint8_t idx,
						enum dssmgr_rotation rot)
{
	int rc = 0;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	/* Verify a change is occuring */
	if (data->flds[idx].force_rotation != rot) {
		mutex_lock(&data->mtx);

		data->flds[idx].force_rotation = rot;
		data->flds[idx].rotation_client_id = data->client_id;
		rc = dssmgr_oly_set_and_apply(data, idx);

		mutex_unlock(&data->mtx);
	}

	return rc;
}

int dssmgr_oly_get_force_scale(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[idx].force_scale;
}

int dssmgr_oly_set_force_scale(struct dssmgr_oly_data *data, uint8_t idx,
				enum dssmgr_scale scale, int percent)
{
	int rc = 0;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	/* Verify a change is occuring */
	if (data->flds[idx].force_scale != scale
	   || data->flds[idx].force_scale_percent != (u32) percent) {
		mutex_lock(&data->mtx);

		data->flds[idx].force_scale = scale;
		data->flds[idx].force_scale_percent = (u32) percent;
		data->flds[idx].scale_client_id = data->client_id;
		rc = dssmgr_oly_set_and_apply(data, idx);

		mutex_unlock(&data->mtx);
	}

	return rc;
}

int dssmgr_oly_get_force_enable_nf(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[idx].force_enable_nf;
}

/* Force enable the overlay when the client pushes the next frame */
int dssmgr_oly_set_force_enable_nf(struct dssmgr_oly_data *data, uint8_t idx
						, uint8_t en)
{
	int rc = 0;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);
	BUG_ON(en > 1);

	/* Verify a change is occuring */
	if (data->flds[idx].force_enable_nf != en) {
		mutex_lock(&data->mtx);

		data->flds[idx].force_enable_nf = en;
		data->flds[idx].enable_nf_client_id = data->client_id;

		/* No need to set & apply when enabled since its the next
		 * pushed frame that we will tigger off of
		 */
		if (!en)
			rc = dssmgr_oly_set_and_apply(data, idx);

		mutex_unlock(&data->mtx);
	}

	return rc;
}

int dssmgr_oly_get_force_exclusive(struct dssmgr_oly_data *data, uint8_t idx)
{
	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	return data->flds[idx].force_exclusive;
}

int dssmgr_oly_set_force_exclusive(struct dssmgr_oly_data *data, uint8_t idx
						, uint8_t en)
{
	int rc = 0;
	int m_idx;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);
	BUG_ON(en > 1);

	/* Verify a change is occuring */
	if (data->mirror.excl_idx != DSSMGR_OLY_INVALID_IDX &&
					data->mirror.excl_idx != idx) {
		rc = -EBUSY;
	} else if (data->flds[idx].force_exclusive != en) {
		mutex_lock(&data->mtx);

		data->flds[idx].force_exclusive = en;
		data->flds[idx].enable_nf_client_id = data->client_id;

		if (en) {
			data->mirror.excl_idx = idx;

			/* Disable the mirrored overlay */
			m_idx = data->mirror.dst_idx;
			if (m_idx != DSSMGR_OLY_INVALID_IDX)
				dssmgr_oly_disable_oly(data, m_idx);
		} else {
			data->mirror.excl_idx = DSSMGR_OLY_INVALID_IDX;
			/* The next mirror frame will re-enable the mirror */
		}

		mutex_unlock(&data->mtx);
	}

	return rc;
}

int dssmgr_oly_set_force_disp_size(struct dssmgr_oly_data *data, uint8_t idx
						, uint16_t w, uint16_t h)
{
	int rc = 0;

	BUG_ON(data == NULL);
	BUG_ON(idx >= DSSMGR_MAX_OVERLAYS);

	mutex_lock(&data->mtx);

	data->flds[idx].force_disp_size = (w != 0 && h != 0) ? 1 : 0;
	data->flds[idx].disp_size_client_id = data->client_id;
#ifdef REMOVED_FOR_ICS_BRINGUP
	omap_vout_override_disp_size(idx, w, h);
#endif

	mutex_unlock(&data->mtx);

	return rc;
}

int dssmgr_oly_set_prefer_mirror(struct dssmgr_oly_data *data, uint16_t val)
{
	int rc = 0;

	BUG_ON(data == NULL);

	mutex_lock(&data->mtx);

	data->prefer_mirror = val;

	mutex_unlock(&data->mtx);

	return rc;
}

int dssmgr_oly_set_force_mirror_crop_side(struct dssmgr_oly_data *data
					  , enum dssmgr_mirror_crop_side side
					  , u32 amount)
{
	int rc = 0;

	BUG_ON(data == NULL);

	mutex_lock(&data->mtx);

	data->mirror.force_crop_side = side;
	data->mirror.force_crop_amount = (u32) amount;

	mutex_unlock(&data->mtx);

	return rc;
}

void dmmgr_oly_reconfig_for_dpy(struct dssmgr_oly_data *data
						, struct omap_dss_device *dpy)
{
	struct omap_overlay *oly;
	int i;

	BUG_ON(data == NULL);

	for (i = 0; i < data->num; i++) {
		oly = data->olys[i];
		if (oly->manager == NULL || oly->manager->device == NULL)
			continue;

		if (oly->manager->device == dpy)
			dssmgr_oly_set_and_apply(data, i);
	}
}

