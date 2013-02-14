/*
 * Copyright (C) 2010 Motorola, Inc.
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

#include <linux/ioctl.h>

#ifndef __OMAP_DSSMGR_H__
#define __OMAP_DSSMGR_H__

#define DSSMGR_MAX_NAME_SIZE	   (32)

#define DSSMGR_MAX_DISPLAYS	   (5)
#define DSSMGR_MAX_MANAGERS	   (5)
#define DSSMGR_MAX_OVERLAYS	   (5)

#define DSSMGR_EDID_BLOCK_LEN      (128)

#define DSSMGR_MAX_RESOLUTIONS     (30)

/* Defines for the "flags" field of "dssmgr_display" */
#define DSSMGR_DPY_FLAG_ACTIVE     (1 << 0)
#define DSSMGR_DPY_FLAG_HOTPLUG    (1 << 1)
#define DSSMGR_DPY_FLAG_EDID       (1 << 2)

/* Defines for the "flags" field of "dssmgr_resolution" */
#define DSSMGR_RES_FLAG_HDMI       (1 << 0)
#define DSSMGR_RES_FLAG_INTERLACED (1 << 1)
#define DSSMGR_RES_FLAG_TIMING     (1 << 2) /* non-HDMI timing data included */
#define DSSMGR_RES_FLAG_MIN_DS     (1 << 3) /* min drive strength */
#define DSSMGR_RES_FLAG_DVI_AUDIO  (1 << 4) /* dvi resolution audio enable */

struct dssmgr_timing {
	uint32_t clk;
	uint16_t hfp;
	uint16_t hsw;
	uint16_t hbp;
	uint16_t vfp;
	uint16_t vsw;
	uint16_t vbp;
	uint8_t  sp;    /* H = bit 0, V = bit 1 */
	uint8_t  audio; /* Deprecated - do not use */
};

struct dssmgr_resolution {
	uint16_t width;  /* Pixels */
	uint16_t height; /* Pixels */
	uint16_t rate;   /* Hz, 0 if unknown*/
	uint16_t flags;
	struct dssmgr_timing timing; /* non-HDMI only */
};

struct dssmgr_display {
	/* Identifier, non-zero if valid */
	int id;
	/* Name */
	char name[DSSMGR_MAX_NAME_SIZE + 1];
	/* Flags denoting if it supports multiple resolutions */
	uint32_t flags;
	/* Active resolution info */
	struct dssmgr_resolution resolution;
};

struct dssmgr_manager {
	/* Identifier, non-zero if valid */
	int id;
	/* Name */
	char name[DSSMGR_MAX_NAME_SIZE + 1];
	/* Attached display id */
	int disp_id;
};

struct dssmgr_overlay {
	/* Identifier, non-zero if valid */
	int id;
	/* Name */
	char name[DSSMGR_MAX_NAME_SIZE + 1];
	/* Attached manager id */
	int mgr_id;
	/* Flag denoting if it is enabled */
	int active;
	/* Frame count */
	uint32_t fcnt;
	/* Reserved */
	uint8_t rsvd[20];
};

struct dssmgr_cfg {
	struct dssmgr_display displays[DSSMGR_MAX_DISPLAYS];
	struct dssmgr_manager managers[DSSMGR_MAX_MANAGERS];
	struct dssmgr_overlay overlays[DSSMGR_MAX_OVERLAYS];
};

enum dssmgr_rotation {
	/* Use overlay client setting */
	DSSMGR_ROTATION_IGNORE,
	DSSMGR_ROTATION_0,
	DSSMGR_ROTATION_90,
	DSSMGR_ROTATION_180,
	DSSMGR_ROTATION_270,
};

enum dssmgr_scale {
	/* Use overlay client setting */
	DSSMGR_SCALE_IGNORE,
	/* Fill the screen, ignoring aspect ratio */
	DSSMGR_SCALE_FILL_SCREEN,
	/* Fill the screen, but maintain aspect ratio */
	DSSMGR_SCALE_FIT_TO_SCREEN,
	/* Fill a percent of the screen, ignoring aspect ratio
	 * The scaling percent, 40 to 100, is passed as the <extra> value */
	DSSMGR_SCALE_FILL_PERCENT_OF_SCREEN,
	/* Fill a percent of the screen, but maintain aspect ratio
	 * The scaling percent, 40 to 100, is passed as the <extra> value */
	DSSMGR_SCALE_FIT_PERCENT_OF_SCREEN,
	/* Fit to the mirrored overlay rectangle, but maintain aspect ratio */
	DSSMGR_SCALE_FIT_TO_MIRROR,
	/* Center the unscaled frame in the screen */
	DSSMGR_SCALE_CENTER,
};

enum dssmgr_mirror_crop_side {
	DSSMGR_MIRROR_CROP_TOP,
	DSSMGR_MIRROR_CROP_BOTTOM,
	DSSMGR_MIRROR_CROP_RIGHT,
	DSSMGR_MIRROR_CROP_LEFT,
};

enum dssmgr_cmd_type {
	/* ENABLE/DISABLE a display (comp_id1 & val1 = [1/0] state) */
	DSSMGR_CMD_ENABLE_DPY = 1,
	/* ATTACH overlay (comp_id1) to manager (comp_id2) */
	DSSMGR_CMD_ATTACH_OLY2MGR,
	/* ATTACH manager (comp_id1) to display (comp_id2) */
	DSSMGR_CMD_ATTACH_MGR2DPY,
	/* SET RESOLUTION of a display (comp_id1 & resolution) */
	DSSMGR_CMD_SET_RESOLUTION,
	/* ENABLE/DISABLE hot-plug detect on a display
	 * (comp_id1 & val1 = [1/0] state) */
	DSSMGR_CMD_ENABLE_HPD,
	/* RESET an overlay force commands (comp_id1)
	 * if val1 == 1 the overlay will be left force disabled */
	DSSMGR_CMD_RESET_OLY,
	/* FORCE DISABLE an overlay (comp_id1 & val1 = [1/0] state) */
	DSSMGR_CMD_FORCE_DISABLE_OLY,
	/* FORCE an overlay to a given rotation
	 * (comp_id1 & val1 = <dssmgr_rotation>) */
	DSSMGR_CMD_FORCE_ROTATION_OLY,
	/* FORCE an overlay scaling mode
	 * (comp_id1, val1 = <dssmgr_scale>, val2 = <extra>) */
	DSSMGR_CMD_FORCE_SCALE_OLY,
	/* FORCE an overlay mirroring mode
	 * MIRROR overlay (comp_id1) to overlay (comp_id2)
	 * The destination overlay can be 0 to disable mirroring */
	DSSMGR_CMD_FORCE_MIRROR_OLY,
	/* FORCE ENABLE an overlay's next pushed frame
	 * (comp_id1 & val1 = [1/0] state) */
	DSSMGR_CMD_FORCE_ENABLE_OLY_NEXT_FRAME,
	/* FORCE EXCLUSIVE mode on the overlay (blocks Mirroring)
	 * (comp_id1 & val1 = [1/0] state) */
	DSSMGR_CMD_FORCE_EXCLUSIVE_OLY,
	/* FORCE a display size to the overlay driver
	 * (val1 = width & val2 = height, disabled if either are 0) */
	DSSMGR_CMD_FORCE_OLY_DISP_SIZE,
	/* Prefer mirror overlay over user overlay if they conflict
	 * (val1 = [1/0] enable/disable) */
	DSSMGR_CMD_PREFER_MIRROR,
	/* FORCE crop a number of pixels from the specified side of
	* the UI that will be mirrored over HDMI
	* val1 = side of source to crop (enum - dssmgr_mirror_crop_side)
	* val2 = amount crop (# pixels from the source - i.e. LCD) */
	DSSMGR_CMD_FORCE_MIRROR_CROP_SIDE,
	/* Manual Power Control (SUSPEND/RESUME) of a display
	 * (comp_id1 & val1 = [0/1/2] disable/suspend/resume) */
	DSSMGR_CMD_MANUAL_PWRCTRL_DPY,
	/* Pause or Mute an overlay data stream
	 * (comp_id1 & val1 = [0-Off/1-Idle/2-Pause/3-Mute])
	 * The only valid state transition from Off is to Idle
	 * Cannot transition from Pause to Mute or vise-versa
	 * This feature can not be used on the GFX overlay
	 * This feature only works for NV12 frames */
	DSSMGR_CMD_PAUSE_MUTE_OLY,
};

struct dssmgr_cmd {
	/* The command to perform */
	enum dssmgr_cmd_type cmd;
	/* The component(s) involved */
	int comp_id1;
	int comp_id2;
	/* Generic values */
	uint32_t val1;
	uint32_t val2;
	/* Resolution */
	struct dssmgr_resolution resolution;
};

struct dssmgr_edid {
	/* The ID of the display to retrieve the EDID block from */
	int dpy_id;
	/* The block to retrieve, 0 indexed */
	int block;
	/* Returned data */
	uint8_t data[DSSMGR_EDID_BLOCK_LEN];
};

#define DSSMGR_IOCTL_MAGIC	'g'
#define DSSMGR_IOCTL_BASE	0x20
#define DSSMGR_QUERYCFG		_IOWR(DSSMGR_IOCTL_MAGIC, \
				    DSSMGR_IOCTL_BASE+0, struct dssmgr_cfg)
#define DSSMGR_S_CMD		_IOW(DSSMGR_IOCTL_MAGIC, \
				    DSSMGR_IOCTL_BASE+1, struct dssmgr_cmd)
#define DSSMGR_G_EDID		_IOW(DSSMGR_IOCTL_MAGIC, \
				    DSSMGR_IOCTL_BASE+2, struct dssmgr_edid)

#endif /* __OMAP_DSSMGR_H__ */

