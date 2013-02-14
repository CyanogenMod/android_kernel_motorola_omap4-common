/*
 * drivers/video/omap2/misc/dssmgr-oly.h
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
 *
 * -----------------------------------------------------------------------
 *
 * This interface is intended to handle DSS overlay manipulation
 */

#ifndef __DSSMGR_OLY_H__
#define __DSSMGR_OLY_H__

#include <linux/mutex.h>
#include <video/omapdss.h>

#include <linux/omap-dssmgr.h>

/* From the overlay driver */
extern void omap_vout_override_disp_size(enum omap_plane ovl, int w, int h);

#define DSSMGR_OLY_INVALID_IDX  0xFF

#define DSSMGR_OLY_PM_OFF       0
#define DSSMGR_OLY_PM_IDLE      1
#define DSSMGR_OLY_PM_PAUSE     2
#define DSSMGR_OLY_PM_MUTE      3

#define DSSMGR_OLY_PM_BUF_W     1920 /*720*/
#define DSSMGR_OLY_PM_BUF_H     1088 /*480*/

typedef int (*ovl_set_info)(struct omap_overlay *ovl,
				struct omap_overlay_info *info);
typedef	void (*ovl_get_info)(struct omap_overlay *ovl,
				struct omap_overlay_info *info);

struct dssmgr_oly_mirror {
	int               src_idx;
	int               dst_idx;
	int               w;
	int               h;
	int               mode;
	int               bpp;
	int               buf_idx;
	unsigned long     buf[2];
	unsigned long     buf_alloc[2];

	int               dst_x;
	int               dst_y;
	int               dst_w;
	int               dst_h;

	/* The overlay mirroring can't run with */
	int               excl_idx;

	u32               dma_en;
	u32               dma_fn;
	u32               dma_src_fi;
	u32               dma_dst_fi;

	int               dma_id;
	int               dma_ch;
	bool              dma_complete;
	wait_queue_head_t dma_wait;

	bool              updating;
	bool              force_update;

	enum dssmgr_mirror_crop_side force_crop_side;
	u32               force_crop_amount;
};

struct dssmgr_oly_flds {
	ovl_set_info set_func;
	ovl_get_info get_func;

	int                  force_disable;
	u32                  disable_client_id;
	enum dssmgr_rotation force_rotation;
	u32                  rotation_client_id;
	enum dssmgr_scale    force_scale;
	u32                  force_scale_percent;
	u32                  scale_client_id;
	uint8_t              force_mirror_dst_idx;
	u32                  mirror_client_id;
	int                  force_enable_nf;
	u32                  enable_nf_client_id;
	int                  force_exclusive;
	u32                  exclusive_client_id;
	int                  force_disp_size;
	u32                  disp_size_client_id;

	uint32_t             fcnt;
};

struct dssmgr_oly_data {
	struct mutex mtx; /* Lock for all device accesses */

	u32 client_id;

	int client_frame;

	int num;
	/* DSS Overlay structure */
	struct omap_overlay      *olys[DSSMGR_MAX_OVERLAYS];
	/* Last client set info */
	struct omap_overlay_info  info[DSSMGR_MAX_OVERLAYS];
	/* Extra fields per overlay */
	struct dssmgr_oly_flds    flds[DSSMGR_MAX_OVERLAYS];
	/* Mirroring fields
	 * - Only one mirror active at any time
	 */
	struct dssmgr_oly_mirror  mirror;
	/* Indicates whether or not mirrored overlay is preferred
	 * over the user overlay
	 */
	int prefer_mirror;
};

/* Intended to be called at driver init and removal times */
int  dssmgr_oly_init(struct dssmgr_oly_data *data);
void dssmgr_oly_remove(struct dssmgr_oly_data *data);

/* Set client ID for use in tracking override settings */
void dssmgr_oly_set_client_id(struct dssmgr_oly_data *data, u32 id);

/* Resets all override settings */
void dssmgr_oly_reset(struct dssmgr_oly_data *data);
/* Resets all override settings for a specific overlay */
void dssmgr_oly_reset_oly(struct dssmgr_oly_data *data, uint8_t idx, int dis);
/* Resets all override settings for a specific overlay */
void dssmgr_oly_reset_client(struct dssmgr_oly_data *data, u32 id);

int  dssmgr_oly_get_force_disable(struct dssmgr_oly_data *data, uint8_t idx);
int  dssmgr_oly_set_force_disable(struct dssmgr_oly_data *data, uint8_t idx
						, uint8_t dis);
int  dssmgr_oly_get_force_rotation(struct dssmgr_oly_data *data, uint8_t idx);
int  dssmgr_oly_set_force_rotation(struct dssmgr_oly_data *data, uint8_t idx
						, enum dssmgr_rotation rot);
int  dssmgr_oly_get_force_scale(struct dssmgr_oly_data *data, uint8_t idx);
int  dssmgr_oly_set_force_scale(struct dssmgr_oly_data *data, uint8_t idx
				, enum dssmgr_scale scale, int percent);
/* Returns the destination oly if mirroring, else -1 */
int  dssmgr_oly_get_force_mirror(struct dssmgr_oly_data *data, uint8_t s_idx);
int  dssmgr_oly_set_force_mirror(struct dssmgr_oly_data *data, uint8_t s_idx
						, uint8_t d_idx);
/* Force enable the overlay when the client pushes the next frame */
int  dssmgr_oly_get_force_enable_nf(struct dssmgr_oly_data *data, uint8_t idx);
int  dssmgr_oly_set_force_enable_nf(struct dssmgr_oly_data *data, uint8_t idx
						, uint8_t en);
/* Force the overlay to be exclusive, blocking mirroring when oly is enabled */
int  dssmgr_oly_get_force_exclusive(struct dssmgr_oly_data *data, uint8_t idx);
int  dssmgr_oly_set_force_exclusive(struct dssmgr_oly_data *data, uint8_t idx
						, uint8_t en);
/* Force the overlay driver to see a specified display size */
int  dssmgr_oly_set_force_disp_size(struct dssmgr_oly_data *data, uint8_t idx
						, uint16_t w, uint16_t h);
/* Prefer mirror updates over client updates */
int  dssmgr_oly_set_prefer_mirror(struct dssmgr_oly_data *data, uint16_t val);
/* Force the overlay driver to crop a specified amount from the specified size
 * This is used for removing the status bar from the mirrored UI
 */
int  dssmgr_oly_set_force_mirror_crop_side(struct dssmgr_oly_data *data
					   , enum dssmgr_mirror_crop_side side
					   , u32 amount);


/* Force re-config of overlays on passed in display */
void dmmgr_oly_reconfig_for_dpy(struct dssmgr_oly_data *data
						, struct omap_dss_device *dpy);

#endif /* __DSSMGR_OLY_H__ */

