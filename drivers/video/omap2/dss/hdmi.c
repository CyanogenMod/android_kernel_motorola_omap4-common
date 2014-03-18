/*
 * hdmi.c
 *
 * HDMI interface DSS driver setting for TI's OMAP4 family of processor.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Yong Zhi
 *	Mythri pk <mythripk@ti.com>
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

#define DSS_SUBSYS_NAME "HDMI"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <video/omapdss.h>
#include <video/hdmi_ti_4xxx_ip.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/omapfb.h>

#include "dss.h"
#include "dss_features.h"

#define HDMI_WP			0x0
#define HDMI_CORE_SYS		0x400
#define HDMI_CORE_AV		0x900
#define HDMI_PLLCTRL		0x200
#define HDMI_PHY		0x300

/* HDMI EDID Length move this */
#define HDMI_EDID_MAX_LENGTH			512
#define EDID_TIMING_DESCRIPTOR_SIZE		0x12
#define EDID_DESCRIPTOR_BLOCK0_ADDRESS		0x36
#define EDID_DESCRIPTOR_BLOCK1_ADDRESS		0x80
#define EDID_SIZE_BLOCK0_TIMING_DESCRIPTOR	4
#define EDID_SIZE_BLOCK1_TIMING_DESCRIPTOR	4

#define OMAP_HDMI_TIMINGS_NB			34

static struct {
	struct mutex lock;
	struct omap_display_platform_data *pdata;
	struct platform_device *pdev;
	struct omap_dss_device *dssdev;
	struct hdmi_ip_data hdmi_data;
	u8 edid[HDMI_EDID_MAX_LENGTH];
	u8 edid_set;

	bool custom_set;
	enum hdmi_deep_color_mode deep_color;
	struct hdmi_config cfg;
	struct regulator *hdmi_reg;

	int hdmi_irq;
	struct clk *sys_clk;
	struct clk *hdmi_clk;

	int runtime_count;
	int enabled;
	int display_on;
	bool set_mode;
	bool wp_reset_done;

	struct fb_videomode initial_vmode;

	void (*hdmi_start_frame_cb)(void);
	void (*hdmi_irq_cb)(int);
	bool (*hdmi_power_on_cb)(void);
} hdmi;

static const u8 edid_header[8] = {0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0};

int set_dispc_clk(void)
{
	struct clk *clk;
	int r;

	if (cpu_is_omap44xx()) {
		clk = clk_get(NULL, "dpll_per_m5x2_ck");
		if (IS_ERR(clk)) {
			DSSERR("Failed to get dpll_per_m5x2_ck\n");
			r = PTR_ERR(clk);
			return r;
		}
		r = clk_set_rate(clk, 192000000);
		if (r)
			return r;
	}
	return 0;
}
static int hdmi_runtime_get(void)
{
	int r;

	DSSDBG("hdmi_runtime_get\n");

	if (hdmi.runtime_count++ == 0) {
		r = dss_runtime_get();
		if (r)
			goto err_get_dss;

		r = dispc_runtime_get();
		if (r)
			goto err_get_dispc;

		clk_enable(hdmi.sys_clk);
		clk_enable(hdmi.hdmi_clk);

		r = pm_runtime_get_sync(&hdmi.pdev->dev);
		WARN_ON(r);
		if (r < 0)
			goto err_runtime_get;
	}

	return 0;

err_runtime_get:
	clk_disable(hdmi.sys_clk);
	clk_disable(hdmi.hdmi_clk);
	dispc_runtime_put();
err_get_dispc:
	dss_runtime_put();
err_get_dss:
	return r;
}

static void hdmi_runtime_put(void)
{
	int r;

	DSSDBG("hdmi_runtime_put\n");

	if (--hdmi.runtime_count == 0) {
		r = pm_runtime_put_sync(&hdmi.pdev->dev);
		WARN_ON(r);

		clk_disable(hdmi.sys_clk);
		clk_disable(hdmi.hdmi_clk);

		dispc_runtime_put();
		dss_runtime_put();
	}
}

int hdmi_init_display(struct omap_dss_device *dssdev)
{
	DSSDBG("init_display\n");

	return 0;
}

static int relaxed_fb_mode_is_equal(const struct fb_videomode *mode1,
				    const struct fb_videomode *mode2)
{
	u32 ratio1 = mode1->flag & (FB_FLAG_RATIO_4_3 | FB_FLAG_RATIO_16_9);
	u32 ratio2 = mode2->flag & (FB_FLAG_RATIO_4_3 | FB_FLAG_RATIO_16_9);

	return (mode1->xres         == mode2->xres &&
		mode1->yres         == mode2->yres &&
		mode1->pixclock     <= mode2->pixclock * 201 / 200 &&
		mode1->pixclock     >= mode2->pixclock * 200 / 201 &&
		mode1->hsync_len + mode1->left_margin + mode1->right_margin ==
		mode2->hsync_len + mode2->left_margin + mode2->right_margin &&
		mode1->vsync_len + mode1->upper_margin + mode1->lower_margin ==
		mode2->vsync_len + mode2->upper_margin + mode2->lower_margin &&
		(!ratio1 || !ratio2 || ratio1 == ratio2) &&
		(mode1->vmode & FB_VMODE_INTERLACED) ==
		(mode2->vmode & FB_VMODE_INTERLACED));
}

static int hdmi_set_timings(struct fb_videomode *vm, bool check_only)
{
	int i = 0;
	DSSDBG("hdmi_get_code\n");

	pr_debug("%s: xres = %d, yres = %d, pixclock = %d (%lu KHz)n",
		__func__, vm->xres, vm->yres, vm->pixclock,
		 PICOS2KHZ(vm->pixclock));
	pr_debug("hsync_len = %d, left_margin = %d, right_margin = %d\n",
		vm->hsync_len, vm->left_margin, vm->right_margin);
	pr_debug("vsync_len = %d, upper_margin = %d, lower_margin = %d\n",
		vm->vsync_len, vm->upper_margin, vm->lower_margin);
	pr_debug("flag = 0x%x, vmode = 0x%x, check_only = %d\n",
		 vm->flag, vm->vmode, check_only);

	if (!vm->xres || !vm->yres || !vm->pixclock)
		goto fail;

	for (i = 0; i < CEA_MODEDB_SIZE; i++) {
		if (relaxed_fb_mode_is_equal(cea_modes + i, vm)) {
			/* save old flag field */
			u32 flag = vm->flag;
			*vm = cea_modes[i];
			vm->flag = flag;
			pr_debug("%s: found match cea_mode %d\n",
				__func__, i);
			if (check_only)
				return 1;
			hdmi.cfg.cm.cea_code = i;
			hdmi.cfg.timings = cea_modes[hdmi.cfg.cm.cea_code];
			pr_info("%s: set timing to cea mode %d, %dx%d@%dHz\n",
				__func__, i, hdmi.cfg.timings.xres,
				hdmi.cfg.timings.yres,
				hdmi.cfg.timings.refresh);
			goto done;
		}
	}

	for (i = 0; i < VESA_MODEDB_SIZE; i++) {
		if (relaxed_fb_mode_is_equal(vesa_modes + i, vm)) {
			/* save old flag field */
			u32 flag = vm->flag;
			*vm = vesa_modes[i];
			vm->flag = flag;
			pr_debug("%s: found match vesa_mode %d\n",
				__func__, i);
			if (check_only)
				return 1;
			hdmi.cfg.cm.cea_code = 0;
			hdmi.cfg.timings = vesa_modes[i];
			pr_info("%s: set timing to vesa mode %d, %dx%d@%dHz\n",
				__func__, i, hdmi.cfg.timings.xres,
				hdmi.cfg.timings.yres,
				hdmi.cfg.timings.refresh);
			goto done;
		}
	}

fail:
	if (check_only)
		return 0;
	hdmi.cfg.cm.cea_code = 1;
	hdmi.cfg.timings = cea_modes[hdmi.cfg.cm.cea_code];
	pr_info("%s: set default timing to cea mode %d, %dx%d@%dHz\n",
		__func__, hdmi.cfg.cm.cea_code, hdmi.cfg.timings.xres,
		hdmi.cfg.timings.yres, hdmi.cfg.timings.refresh);

	i = -1;
done:

	DSSDBG("%s-%d\n", hdmi.cfg.cm.cea_code ? "CEA" : "VESA",
	       hdmi.cfg.cm.cea_code ? hdmi.cfg.cm.cea_code : i);
	return i >= 0;
}

void hdmi_get_monspecs(struct fb_monspecs *specs)
{
	int i;
	char *edid = (char *) hdmi.edid;
	int preferred_vmode = -1;
	int default_code;
	struct fb_videomode default_vmode;

	memset(specs, 0x0, sizeof(*specs));
	if (!hdmi.edid_set)
		return;

	fb_edid_to_monspecs(edid, specs);
	if (specs->modedb == NULL)
		return;

	for (i = 1; i <= edid[0x7e] && i * 128 < HDMI_EDID_MAX_LENGTH; i++) {
		if (edid[i * 128] == 0x2)
			fb_edid_add_monspecs(edid + i * 128, specs);
	}

	if (specs->misc & FB_MISC_HDMI) {
		DSSINFO("HDMI sink\n");
		hdmi.cfg.cm.hdmi_mode = HDMI_HDMI;
	} else {
		DSSINFO("DVI sink\n");
		hdmi.cfg.cm.hdmi_mode = HDMI_DVI;
	}

	memset(&hdmi.initial_vmode, 0, sizeof(hdmi.initial_vmode));
	default_code = hdmi.dssdev->panel.hdmi_default_cea_code;
	if (default_code > 0 && default_code < CEA_MODEDB_SIZE) {
		default_vmode = cea_modes[default_code];
		DSSINFO("Preferred cea_mode[%d], %dx%d@%d\n",
			default_code, default_vmode.xres,
			default_vmode.yres, default_vmode.refresh);
	} else
		default_code = 0;

	/* mark resolutions we support */
	for (i = 0; i < specs->modedb_len; i++) {
		u32 max_pclk;

		pr_debug("Checking modedb[%d]: %dx%d@%d\n",
			 i, specs->modedb[i].xres,
			 specs->modedb[i].yres,
			 specs->modedb[i].refresh);

		max_pclk = hdmi.dssdev->clocks.hdmi.max_pixclk_khz;
		if (!hdmi_set_timings(&specs->modedb[i], true))
			continue;

		if (max_pclk &&
		    max_pclk < PICOS2KHZ(specs->modedb[i].pixclock))
			continue;

		if (specs->modedb[i].flag & FB_FLAG_PIXEL_REPEAT)
			continue;

		/* if we made it here the mode is actually possible */
		specs->modedb[i].flag |= FB_FLAG_HW_CAPABLE;

		/* pick out the first mode that it claims to be preferred
		 * and mark as such */
		if ((preferred_vmode < 0) &&
			specs->misc & FB_MISC_1ST_DETAIL &&
			specs->modedb[i].flag & FB_MODE_IS_DETAILED) {
			pr_info("First DTD marked preferred video mode: "
				" %dx%d@%d\n",
				specs->modedb[i].xres,
				specs->modedb[i].yres,
				specs->modedb[i].refresh);
			specs->modedb[i].flag |= FB_FLAG_PREFERRED;
			preferred_vmode = i;
		}

		/*
		 * If this is a native resolution (came from a SVD),
		 * and it's higher resolution than the preferred
		 * resolution from the first DTD, mark it preferred
		 * instead.
		 */
		if ((specs->modedb[i].flag & FB_FLAG_NATIVE) &&
		    ((preferred_vmode < 0) ||
		     ((specs->modedb[i].xres * specs->modedb[i].yres) >
		      (specs->modedb[preferred_vmode].xres *
		       specs->modedb[preferred_vmode].yres)))) {
			if (preferred_vmode >= 0) {
				/* clear previous */
				specs->modedb[preferred_vmode].flag &=
					~FB_FLAG_PREFERRED;
				pr_info("Replacing previous preferred "
					"vmode with better one\n");
			}
			pr_info("Marking native video mode as preferred:"
				" %dx%d@%d\n",
				specs->modedb[i].xres,
				specs->modedb[i].yres,
				specs->modedb[i].refresh);
			specs->modedb[i].flag |= FB_FLAG_PREFERRED;
			preferred_vmode = i;
		}
	}

	/*
	 * Because the preferred mode might change a few times
	 * in above pass (e.g. if the SVD native is better
	 * than a DTD first entry), we select the initial
	 * vmode in a second pass through the list.
	 */
	if (default_code) {
		for (i = 0; i < specs->modedb_len; i++) {
			/* check if we can use this mode as the initial one */
			if ((specs->modedb[i].flag & FB_FLAG_HW_CAPABLE) &&
			    relaxed_fb_mode_is_equal(&default_vmode,
						     &specs->modedb[i])) {
				pr_info("initial_vmode set to cea_mode[%d]\n",
					default_code);
				hdmi.initial_vmode = specs->modedb[i];
				break;
			}
		}
	}

	/*
	 * If the platform preferred mode isn't set or isn't supported
	 * by the sink, choose the sink's preferred mode if there
	 * is one.  If there's no sink preferred mode either, then
	 * choose the first supported mode.
	 */
	if (!hdmi.initial_vmode.pixclock) {
		if (preferred_vmode >= 0) {
			hdmi.initial_vmode = specs->modedb[preferred_vmode];
			pr_info("initial_vmode set to sink"
				" preferred %dx%d@%d\n",
				hdmi.initial_vmode.xres,
				hdmi.initial_vmode.yres,
				hdmi.initial_vmode.refresh);
		} else {
			/* take the first supported vmode as the initial one */
			for (i = 0; i < specs->modedb_len; i++) {
				if (specs->modedb[i].flag & FB_FLAG_HW_CAPABLE) {
					pr_info("initial_vmode set to cea_mode[%d]\n",
						default_code);
					hdmi.initial_vmode = specs->modedb[i];
					break;
				}
			}
		}
	}

	if (hdmi.initial_vmode.pixclock)
		pr_info("%s: initial_vmode set to %dx%d@%dHz\n",
			__func__, hdmi.initial_vmode.xres,
			hdmi.initial_vmode.yres, hdmi.initial_vmode.refresh);
	else
		pr_info("%s: No usable video mode found!\n", __func__);
}

#define EDID_BLK_SIZE 0x80
static void hdmi_parse_cea_audio_blocks(struct omap_hdmi_audio_modes *specs,
					const u8* edid)
{
	u8 parse_offset;
	u8 data_block_end;

	/* At this point, we know that we are parsing an EDID CEA Extension
	 * block.  Byte index 2 of the header should mark where the CEA data
	 * blocks finish and where the VESA detailed timing descriptors start.
	 * We are looking for CEA Audio Data blocks, so don't bother to parse
	 * past the end of the data block region.
	 */
	parse_offset = 4;
	data_block_end = edid[2];
	if (data_block_end > EDID_BLK_SIZE)
		data_block_end = EDID_BLK_SIZE;

	while (parse_offset < data_block_end) {
		/* Read the tag and length of the data block. */
		u8 tag = (edid[parse_offset] >> 5) & 0x7;
		u8 len = (edid[parse_offset] & 0x1F) + 1;
		const u8* desc;
		int desc_cnt;

		/* Sanity check length */
		if ((parse_offset + len) > data_block_end) {
			pr_warn("Malformed data block detected in EDID CEA "
				"Extension block at offset %d in the extension "
			        "block.  Block length (%d) exceeds end of "
				"extension section (%d)",
				parse_offset, len, data_block_end);
			break;
		}

		/* 0x1 is the tag for an audio data block */
		if (tag != 0x01) {
			parse_offset += len;
			continue;
		}

		desc = edid + parse_offset + 1;
		desc_cnt = (len - 1) / 3;

		while (desc_cnt && (specs->valid_mode_cnt <
					OMAP_MAX_HDMI_AUDIO_MODES)) {
			struct cea861_short_audio_descriptor* d;
			d = specs->audio_modes + specs->valid_mode_cnt;

			/* audio code are bits[3, 6] of byte index 0 */
			d->code = (desc[0] >> 3) & 0xF;

			/* max channels - 1 are bits [0, 2] of byte index 0.  A
			 * raw value of 0 indicates unknown. */
			d->max_channels = desc[0] & 0x7;
			if (d->max_channels)
				d->max_channels++;

			/* sample rate bitfield is made of bits[0, 6] of
			 * byte index 1. */
			d->sample_rates = desc[1] & 0x7F;

			/* if the audio code is in the range [2, 8], then byte
			 * index 2 indicated the max bit rate of the compressed
			 * audio in kbps divided by 8 */
			if ((d->code >= CEA861_AUDIO_CODE_AC3) &&
			    (d->code <= CEA861_AUDIO_CODE_ATRAC))
			    d->max_bitrate = ((u32)desc[2]) * 8;

			/* Finally, stash the raw value of byte index 2 in the
			 * structure.  It can contain format specific
			 * information for audio codes > 8 */
			d->extra_data = desc[2];

			/* advance to the next descriptor */
			specs->valid_mode_cnt++;
			desc += 3;
			desc_cnt--;
		}

		parse_offset += len;
	}
}

void hdmi_get_audspecs(struct omap_hdmi_audio_modes *specs)
{
	u32 blk_offset;

	if (!specs)
		return;

	memset(specs, 0, sizeof(*specs));

	if (!hdmi.edid_set)
		return;

	/* Search our EDID structure for valid CEA Extensions blocks.  For each
	 * CEA Extension block, search for and parse Audio Data Blocks which
	 * will contain CEA Short Audio Descriptors.
	 *
	 * EDID block 0 should always be a VESA block, so we should start
	 * looking for CEA Extension blocks at block 1.
	 */
	for ( blk_offset =  EDID_BLK_SIZE;
	     (blk_offset +  EDID_BLK_SIZE) <= HDMI_EDID_MAX_LENGTH;
	      blk_offset += EDID_BLK_SIZE) {
		const u8* edid = ((u8*)hdmi.edid) + blk_offset;

		/* Is this a CEA Extension?  If so, its tag will be 0x02. */
		if (edid[0] != 0x02)
			continue;

		/* We only understand CEA Extension block versions 1-3.  If this
		 * is a version 2 or version 3 extension block, it will have a
		 * flags field which (among other things) will indicate whether
		 * or not basic audio support is present.
		 */
		switch (edid[1]) {
			/* V1, no flags data. */
			case 0x01:
				break;

			/* V[23], check basic audio flag. */
			case 0x02:
			case 0x03:
				/* bit 6 of the flags field indicates basic
				 * audio support. */
				if (edid[3] & 0x40)
					specs->basic_audio_support = 1;
				pr_debug("edid[0x83] = 0x%x, "
					 "basic_audio_support = %d\n",
					edid[3], specs->basic_audio_support);
				break;

			/* unknown version, skip it. */
			default:
				continue;
		}

		hdmi_parse_cea_audio_blocks(specs, edid);
	}
}

void hdmi_set_edid_state(bool val)
{
	hdmi.edid_set = val;
	if (val)
		pr_info("hdmi: EDID info read\n");
	else {
		pr_info("hdmi: EDID and custom_set cleared\n");
		/* clear custom set if we lost edid */
		hdmi.custom_set = 0;
	}
}
EXPORT_SYMBOL(hdmi_set_edid_state);

u8 *hdmi_read_edid(struct omap_video_timings *dp)
{
	int ret = 0, i;

	if (hdmi.edid_set)
		return hdmi.edid;

	memset(hdmi.edid, 0, HDMI_EDID_MAX_LENGTH);

	ret = read_ti_4xxx_edid(&hdmi.hdmi_data, hdmi.edid,
						HDMI_EDID_MAX_LENGTH);

	for (i = 0; i < HDMI_EDID_MAX_LENGTH; i += 16)
		pr_info("edid[%03x] = %02x %02x %02x %02x %02x %02x %02x %02x "
			 "%02x %02x %02x %02x %02x %02x %02x %02x\n", i,
			hdmi.edid[i], hdmi.edid[i + 1], hdmi.edid[i + 2],
			hdmi.edid[i + 3], hdmi.edid[i + 4], hdmi.edid[i + 5],
			hdmi.edid[i + 6], hdmi.edid[i + 7], hdmi.edid[i + 8],
			hdmi.edid[i + 9], hdmi.edid[i + 10], hdmi.edid[i + 11],
			hdmi.edid[i + 12], hdmi.edid[i + 13], hdmi.edid[i + 14],
			hdmi.edid[i + 15]);

	if (ret) {
		DSSWARN("failed to read E-EDID\n");
		return NULL;
	}

	if (memcmp(hdmi.edid, edid_header, sizeof(edid_header)))
		return NULL;

	hdmi_set_edid_state(true);

	return hdmi.edid;
}

static void hdmi_compute_pll(struct omap_dss_device *dssdev, int phy,
		struct hdmi_pll_info *pi)
{
	unsigned long clkin, refclk;
	u32 mf;

	clkin = clk_get_rate(hdmi.sys_clk) / 10000;
	/*
	 * Input clock is predivided by N + 1
	 * out put of which is reference clk
	 */
	pi->regn = dssdev->clocks.hdmi.regn;
	refclk = clkin / (pi->regn + 1);

	/*
	 * multiplier is pixel_clk/ref_clk
	 * Multiplying by 100 to avoid fractional part removal
	 */
	pi->regm = (phy * 100 / (refclk)) / 100;
	pi->regm2 = dssdev->clocks.hdmi.regm2;

	/*
	 * fractional multiplier is remainder of the difference between
	 * multiplier and actual phy(required pixel clock thus should be
	 * multiplied by 2^18(262144) divided by the reference clock
	 */
	mf = (phy - pi->regm * refclk) * 262144;
	pi->regmf = mf / (refclk);

	/*
	 * Dcofreq should be set to 1 if required pixel clock
	 * is greater than 1000MHz
	 */
	pi->dcofreq = phy > 1000 * 100;
	pi->regsd = ((pi->regm * clkin / 10) / ((pi->regn + 1) * 250) + 5) / 10;

	DSSDBG("M = %d Mf = %d\n", pi->regm, pi->regmf);
	DSSDBG("range = %d sd = %d\n", pi->dcofreq, pi->regsd);
}

static void hdmi_load_hdcp_keys(struct omap_dss_device *dssdev)
{
	int aksv;
	DSSDBG("hdmi_load_hdcp_keys\n");
	/* load the keys and reset the wrapper to populate the AKSV registers*/
	if (hdmi.hdmi_power_on_cb) {
		aksv = hdmi_ti_4xx_check_aksv_data(&hdmi.hdmi_data);
		DSSINFO("%s: aksv = %d\n", __func__, aksv);
		if ((aksv == HDMI_AKSV_ZERO) && hdmi.hdmi_power_on_cb()) {
			hdmi_ti_4xxx_set_wait_soft_reset(&hdmi.hdmi_data);

			/*
			 * The aksv keys aren't always readable right away,
			 * even after we waited for the soft reset to
			 * clear.  TRM says AKSV keys aren't transferred
			 * until 2ms after reset.
			 */
			usleep_range(2000, 4000);

			aksv = hdmi_ti_4xx_check_aksv_data(&hdmi.hdmi_data);
			hdmi.wp_reset_done = (aksv == HDMI_AKSV_VALID) ?
				true : false;
			DSSINFO("HDMI_WRAPPER RESET DONE wp_reset_done = %d,"
				" aksv = %d\n",	hdmi.wp_reset_done, aksv);
		} else if (aksv == HDMI_AKSV_VALID)
			hdmi.wp_reset_done = true;
		else if (aksv == HDMI_AKSV_ERROR)
			hdmi.wp_reset_done = false;

		if (!hdmi.wp_reset_done)
			DSSERR("*** INVALID AKSV: "
			       "Do not perform HDCP AUTHENTICATION\n");
		else
			DSSINFO("AKSV has been loaded\n");

	}

}

static int hdmi_display_on(struct omap_dss_device *dssdev)
{
	int r;
	struct hdmi_pll_info pll_data;
	struct omap_video_timings *p;
	unsigned long phy;

	DSSINFO("Entering %s\n", __func__);

	if (!hdmi.custom_set) {
		DSSERR("%s: custom_set is false, returning\n", __func__);
		return -ENODEV;
	}

	if (hdmi.display_on) {
		DSSWARN("%s: hdmi display already on\n", __func__);
		return 0;
	}

	/* Load the HDCP keys if not already loaded */
	hdmi_load_hdcp_keys(dssdev);

	/* To be safe, make sure video is off before changing settings */
	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 0);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 0);

	p = &dssdev->panel.timings;

	phy = p->pixel_clock;

	switch (hdmi.deep_color) {
	case HDMI_DEEP_COLOR_30BIT:
		phy = (p->pixel_clock * 125) / 100 ;
		hdmi.cfg.deep_color = HDMI_DEEP_COLOR_30BIT;
		break;
	case HDMI_DEEP_COLOR_36BIT:
		if (p->pixel_clock == 148500) {
			printk(KERN_ERR "36 bit deep color not supported");
			return -EINVAL;
		}

		phy = (p->pixel_clock * 150) / 100;
		hdmi.cfg.deep_color = HDMI_DEEP_COLOR_36BIT;
		break;
	case HDMI_DEEP_COLOR_24BIT:
	default:
		phy = p->pixel_clock;
		hdmi.cfg.deep_color = HDMI_DEEP_COLOR_24BIT;
		break;
	}

	hdmi_compute_pll(dssdev, phy, &pll_data);

	/* config the PLL and PHY hdmi_set_pll_pwrfirst */
	r = hdmi_ti_4xxx_pll_program(&hdmi.hdmi_data, &pll_data);
	if (r) {
		DSSERR("Failed to lock PLL\n");
		return -EIO;
	}

	r = hdmi_ti_4xxx_phy_init(&hdmi.hdmi_data, phy);
	if (r) {
		DSSERR("Failed to start PHY\n");
		return -EIO;
	}

	hdmi_ti_4xxx_basic_configure(&hdmi.hdmi_data, &hdmi.cfg);

	/* Make selection of HDMI in DSS */
	dss_select_hdmi_venc_clk_source(DSS_HDMI_M_PCLK);

	set_dispc_clk();

	/* Select the dispc clock source as PRCM clock, to ensure that it is not
	 * DSI PLL source as the clock selected by DSI PLL might not be
	 * sufficient for the resolution selected / that can be changed
	 * dynamically by user. This can be moved to single location , say
	 * Boardfile.
	 */
	dss_select_dispc_clk_source(dssdev->clocks.dispc.dispc_fclk_src);

	/* bypass TV gamma table */
	dispc_enable_gamma_table(0);

	/* tv size */
	dispc_set_digit_size(dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 1);

	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 1);

	/* Start hdcp negotiation if needed */
	if (hdmi.hdmi_start_frame_cb && hdmi.wp_reset_done)
		(*hdmi.hdmi_start_frame_cb)();

	hdmi.display_on = true;

	return 0;
}

static void hdmi_power_off(struct omap_dss_device *dssdev)
{
	if (hdmi.hdmi_irq_cb)
		hdmi.hdmi_irq_cb(HDMI_HPD_LOW);

	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 0);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 0);
	hdmi_ti_4xxx_phy_off(&hdmi.hdmi_data, hdmi.set_mode);
	hdmi_ti_4xxx_set_pll_pwr(&hdmi.hdmi_data, HDMI_PLLPWRCMD_ALLOFF);
	hdmi_runtime_put();
	hdmi.deep_color = HDMI_DEEP_COLOR_24BIT;
}

int omapdss_hdmi_get_pixel_clock(void)
{
	return PICOS2KHZ(hdmi.cfg.timings.pixclock);
}

int omapdss_hdmi_get_mode(void)
{
	return hdmi.cfg.cm.hdmi_mode;
}

int omapdss_hdmi_register_hdcp_callbacks(void (*hdmi_start_frame_cb)(void),
					 void (*hdmi_irq_cb)(int status),
					 bool (*hdmi_power_on_cb)(void))
{
	hdmi.hdmi_start_frame_cb = hdmi_start_frame_cb;
	hdmi.hdmi_irq_cb = hdmi_irq_cb;
	hdmi.hdmi_power_on_cb = hdmi_power_on_cb;

	return hdmi_ti_4xxx_wp_get_video_state(&hdmi.hdmi_data);
}
EXPORT_SYMBOL(omapdss_hdmi_register_hdcp_callbacks);

void omapdss_hdmi_set_deepcolor(int val)
{
	hdmi.deep_color = val;
}

int omapdss_hdmi_get_deepcolor(void)
{
	return hdmi.deep_color;
}

int hdmi_get_current_hpd()
{
	return gpio_get_value(hdmi.dssdev->hpd_gpio);
}

static irqreturn_t hpd_irq_handler(int irq, void *ptr)
{
	int hpd = hdmi_get_current_hpd();
	pr_info("hpd %d\n", hpd);

	hdmi_panel_hpd_handler();

	return IRQ_HANDLED;
}

static irqreturn_t hdmi_irq_handler(int irq, void *arg)
{
	int r = 0;

	r = hdmi_ti_4xxx_irq_handler(&hdmi.hdmi_data);

	DSSDBG("Received HDMI IRQ = %08x\n", r);

	if (hdmi.hdmi_irq_cb)
		hdmi.hdmi_irq_cb(r);

	return IRQ_HANDLED;
}

int omapdss_hdmi_display_check_timing(struct omap_dss_device *dssdev,
					struct omap_video_timings *timings)
{
	struct fb_videomode t;

	omapfb_dss2fb_timings(timings, &t);

	/* also check interlaced timings */
	if (!hdmi_set_timings(&t, true)) {
		t.yres *= 2;
		t.vmode |= FB_VMODE_INTERLACED;
	}
	if (!hdmi_set_timings(&t, true))
		return -EINVAL;
	return 0;
}

int omapdss_hdmi_display_set_mode(struct omap_dss_device *dssdev,
				  struct fb_videomode *vm)
{
	int r1, r2;
	DSSINFO("Enter omapdss_hdmi_display_set_mode\n");
	/* turn the hdmi off and on to get new timings to use */
	hdmi.set_mode = true;
	dssdev->driver->disable(dssdev);
	hdmi.set_mode = false;
	r1 = hdmi_set_timings(vm, false) ? 0 : -EINVAL;
	/* convert hdmi.cfg.timings to dssdev->panel.timings */
	if (!r1)
		omapfb_fb2dss_timings(&hdmi.cfg.timings, &dssdev->panel.timings);
	hdmi.custom_set = 1;
	r2 = dssdev->driver->enable(dssdev);
	return r1 ? : r2;
}

int omapdss_hdmi_display_set_initial_mode(struct omap_dss_device *dssdev)
{
	int r1, r2;

	DSSINFO("Enter omapdss_hdmi_display_set_initial_mode\n");
	if (!hdmi.initial_vmode.pixclock) {
		DSSWARN("No valid initial_vmode set\n");
		return -EINVAL;
	}
	r1 = hdmi_set_timings(&hdmi.initial_vmode, false) ? 0 : -EINVAL;
	/* convert hdmi.cfg.timings to dssdev->panel.timings */
	if (!r1)
		omapfb_fb2dss_timings(&hdmi.cfg.timings, &dssdev->panel.timings);
	hdmi.custom_set = 1;
	r2 = dssdev->driver->enable(dssdev);
	return r1 ? : r2;
}

void omapdss_hdmi_display_set_timing(struct omap_dss_device *dssdev)
{
	struct fb_videomode t;

	omapfb_dss2fb_timings(&dssdev->panel.timings, &t);
	/* also check interlaced timings */
	if (!hdmi_set_timings(&t, true)) {
		t.yres *= 2;
		t.vmode |= FB_VMODE_INTERLACED;
	}

	omapdss_hdmi_display_set_mode(dssdev, &t);
}

int omapdss_hdmi_display_enable(struct omap_dss_device *dssdev, bool display_on)
{
	int r = 0;

	DSSINFO("ENTER %s, display_on = %d\n", __func__, display_on);

	mutex_lock(&hdmi.lock);

	if (hdmi.enabled) {
		DSSINFO("%s: already enabled\n", __func__);
		goto enabled;
	}

	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err0;
	}

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r) {
			DSSERR("failed to enable GPIO's\n");
			goto err1;
		}
	}

	hdmi.hdmi_reg = regulator_get(NULL, "hdmi_5V_en");
	if (IS_ERR_OR_NULL(hdmi.hdmi_reg)) {
		DSSERR("Failed to get hdmi_vref regulator\n");
		r = PTR_ERR(hdmi.hdmi_reg) ? : -ENODEV;
		goto err2;
	}

	r = regulator_enable(hdmi.hdmi_reg);
	if (r) {
		DSSERR("failed to enable hdmi_vref regulator\n");
		goto err3;
	}

	r = hdmi_runtime_get();
	if (r) {
		DSSERR("failed to enable hdmi_vref regulator\n");
		goto err4;
	}

enabled:
	if (display_on) {
		r = hdmi_display_on(dssdev);
		if (r) {
			DSSERR("failed to turn display on\n");
			goto err5;
		}
	}

	hdmi.enabled = true;

	mutex_unlock(&hdmi.lock);
	return 0;

err5:
	hdmi_runtime_put();
err4:
	regulator_disable(hdmi.hdmi_reg);
err3:
	regulator_put(hdmi.hdmi_reg);
err2:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
err1:
	omap_dss_stop_device(dssdev);
err0:
	mutex_unlock(&hdmi.lock);
	return r;
}

void omapdss_hdmi_display_disable(struct omap_dss_device *dssdev)
{
	DSSINFO("Enter hdmi_display_disable\n");
	mutex_lock(&hdmi.lock);

	if (!hdmi.enabled)
		goto done;

	hdmi.enabled = false;
	hdmi.wp_reset_done = false;
	hdmi.display_on = false;

	hdmi_power_off(dssdev);

	regulator_disable(hdmi.hdmi_reg);

	regulator_put(hdmi.hdmi_reg);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omap_dss_stop_device(dssdev);
done:
	mutex_unlock(&hdmi.lock);
}

static int hdmi_get_clocks(struct platform_device *pdev)
{
	struct clk *clk;

	clk = clk_get(&pdev->dev, "sys_clk");
	if (IS_ERR(clk)) {
		DSSERR("can't get sys_clk\n");
		return PTR_ERR(clk);
	}

	hdmi.sys_clk = clk;

	clk = clk_get(&pdev->dev, "hdmi_clk");
	if (IS_ERR(clk)) {
		DSSERR("can't get hdmi_clk\n");
		clk_put(hdmi.sys_clk);
		return PTR_ERR(clk);
	}

	hdmi.hdmi_clk = clk;

	return 0;
}

static void hdmi_put_clocks(void)
{
	if (hdmi.sys_clk)
		clk_put(hdmi.sys_clk);
	if (hdmi.hdmi_clk)
		clk_put(hdmi.hdmi_clk);
}

/* HDMI HW IP initialisation */
static int omapdss_hdmihw_probe(struct platform_device *pdev)
{
	struct resource *hdmi_mem;
	struct omap_dss_board_info *board_data;
	int r;

	hdmi.pdata = pdev->dev.platform_data;
	hdmi.pdev = pdev;

	mutex_init(&hdmi.lock);

	/* save reference to HDMI device */
	board_data = hdmi.pdata->board_data;
	for (r = 0; r < board_data->num_devices; r++) {
		if (board_data->devices[r]->type == OMAP_DISPLAY_TYPE_HDMI)
			hdmi.dssdev = board_data->devices[r];
	}
	if (!hdmi.dssdev) {
		DSSERR("can't get HDMI device\n");
		return -EINVAL;
	}

	hdmi_mem = platform_get_resource(hdmi.pdev, IORESOURCE_MEM, 0);
	if (!hdmi_mem) {
		DSSERR("can't get IORESOURCE_MEM HDMI\n");
		return -EINVAL;
	}

	/* Base address taken from platform */
	hdmi.hdmi_data.base_wp = ioremap(hdmi_mem->start,
						resource_size(hdmi_mem));
	if (!hdmi.hdmi_data.base_wp) {
		DSSERR("can't ioremap WP\n");
		return -ENOMEM;
	}

	r = hdmi_get_clocks(pdev);
	if (r) {
		iounmap(hdmi.hdmi_data.base_wp);
		return r;
	}

	pm_runtime_enable(&pdev->dev);

	r = request_irq(gpio_to_irq(hdmi.dssdev->hpd_gpio), hpd_irq_handler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"hpd", NULL);
	if (r < 0) {
		pr_err("hdmi: request_irq %d failed\n",
			gpio_to_irq(hdmi.dssdev->hpd_gpio));
		return -EINVAL;
	}

	hdmi.hdmi_irq = platform_get_irq(pdev, 0);

	r = request_irq(hdmi.hdmi_irq, hdmi_irq_handler, 0, "OMAP HDMI", NULL);
	if (r < 0) {
		pr_err("hdmi: request_irq %s failed\n",
			pdev->name);
		return -EINVAL;
	}

	hdmi.hdmi_data.hdmi_core_sys_offset = HDMI_CORE_SYS;
	hdmi.hdmi_data.hdmi_core_av_offset = HDMI_CORE_AV;
	hdmi.hdmi_data.hdmi_pll_offset = HDMI_PLLCTRL;
	hdmi.hdmi_data.hdmi_phy_offset = HDMI_PHY;
	hdmi.wp_reset_done = false;

//lucize - moto hack
	/* Init the drive strength percent to full power */
	hdmi.dssdev->phy.hdmi.phy = 100;
//
	hdmi_panel_init();

	return 0;
}

static int omapdss_hdmihw_remove(struct platform_device *pdev)
{
	hdmi_panel_exit();

	if (hdmi.dssdev)
		free_irq(gpio_to_irq(hdmi.dssdev->hpd_gpio), hpd_irq_handler);
	hdmi.dssdev = NULL;

	pm_runtime_disable(&pdev->dev);

	hdmi_put_clocks();

	iounmap(hdmi.hdmi_data.base_wp);

	return 0;
}

static struct platform_driver omapdss_hdmihw_driver = {
	.probe          = omapdss_hdmihw_probe,
	.remove         = omapdss_hdmihw_remove,
	.driver         = {
		.name   = "omapdss_hdmi",
		.owner  = THIS_MODULE,
	},
};

int hdmi_init_platform_driver(void)
{
	return platform_driver_register(&omapdss_hdmihw_driver);
}

void hdmi_uninit_platform_driver(void)
{
	return platform_driver_unregister(&omapdss_hdmihw_driver);
}

void hdmi_dump_regs(struct seq_file *s)
{
	if (hdmi_runtime_get())
		return;

	hdmi_ti_4xxx_dump_regs(&hdmi.hdmi_data, s);

	hdmi_runtime_put();
}
