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

#include "../dss/dss.h"
#include "../dss/dss_features.h"

/*#define HDTV_DEBUG*/
#ifdef HDTV_DEBUG
#define HDTVDBG(format, ...) \
	printk(KERN_INFO "hdtv: " format, ## __VA_ARGS__)
#else
#define HDTVDBG(format, ...)
#endif

#ifdef CONFIG_OMAP_PM
#include <linux/pm_qos_params.h>
static struct pm_qos_request_list pm_qos_handle;
#endif

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
	int code;
	int mode;
	u8 edid[HDMI_EDID_MAX_LENGTH];
	u8 edid_set;
	bool can_do_hdmi;

	bool custom_set;
	enum hdmi_deep_color_mode deep_color;
	struct hdmi_config cfg;

	int hdmi_irq;
	struct clk *sys_clk;
	struct clk *hdmi_clk;

	int runtime_count;
	bool edid_only;
	bool enabled;
	bool set_mode;
	bool wp_reset_done;
	bool hpd_en;

	void (*hdmi_start_frame_cb)(void);
	void (*hdmi_irq_cb)(int);
	bool (*hdmi_power_on_cb)(void);
} hdmi;

static const u8 edid_header[8] = {0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0};

static int hdmi_runtime_get(void)
{
	int r;

	HDTVDBG("hdmi_runtime_get\n");

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

	HDTVDBG("hdmi_runtime_put\n");

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
	HDTVDBG("init_display\n");

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

	HDTVDBG("hdmi_set_timings\n");

	if (!vm->xres || !vm->yres || !vm->pixclock)
		goto fail;

	for (i = 0; i < CEA_MODEDB_SIZE; i++) {
		if (relaxed_fb_mode_is_equal(cea_modes + i, vm)) {
			*vm = cea_modes[i];
			if (check_only)
				return 1;
			hdmi.cfg.cm.code = i;
			hdmi.cfg.cm.mode = HDMI_HDMI;
			hdmi.cfg.timings = cea_modes[hdmi.cfg.cm.code];
			goto done;
		}
	}

	for (i = 0; i < VESA_MODEDB_SIZE; i++) {
		if (relaxed_fb_mode_is_equal(vesa_modes + i, vm)) {
			*vm = vesa_modes[i];
			if (check_only)
				return 1;
			hdmi.cfg.cm.code = i;
			hdmi.cfg.cm.mode = HDMI_DVI;
			hdmi.cfg.timings = vesa_modes[hdmi.cfg.cm.code];
			goto done;
		}
	}

fail:
	if (check_only)
		return 0;

	hdmi.cfg.cm.code = 0;
	hdmi.cfg.cm.mode = HDMI_DVI;
	hdmi.cfg.timings = *vm;

	i = 0;
done:

	HDTVDBG("%s-%d\n", hdmi.cfg.cm.mode ? "CEA" : "VESA", hdmi.cfg.cm.code);
	return i >= 0;
}

void hdmi_get_monspecs(struct fb_monspecs *specs)
{
	int i, j;
	char *edid = (char *) hdmi.edid;

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

	hdmi.can_do_hdmi = specs->misc & FB_MISC_HDMI;

	/* filter out resolutions we don't support */
	for (i = j = 0; i < specs->modedb_len; i++) {
		u32 max_pclk = hdmi.dssdev->clocks.hdmi.max_pixclk_khz;
		if (!hdmi_set_timings(&specs->modedb[i], true))
			continue;

		if (max_pclk &&
		    max_pclk < PICOS2KHZ(specs->modedb[i].pixclock))
			continue;

		if (specs->modedb[i].flag & FB_FLAG_PIXEL_REPEAT)
			continue;

		specs->modedb[j++] = specs->modedb[i];
	}
	specs->modedb_len = j;
}

u8 *hdmi_read_edid(struct omap_video_timings *dp)
{
	int ret = 0, i;

	if (hdmi.edid_set)
		return hdmi.edid;

	/* Prevent a potential panic */
	if (hdmi.runtime_count <= 0) {
		pr_err("hdtv: Reading the EDID with the clks off\n");
		return NULL;
	}

	memset(hdmi.edid, 0, HDMI_EDID_MAX_LENGTH);

	ret = read_ti_4xxx_edid(&hdmi.hdmi_data, hdmi.edid,
						HDMI_EDID_MAX_LENGTH);

	for (i = 0; i < HDMI_EDID_MAX_LENGTH; i += 16)
		HDTVDBG("edid[%03x] = %02x %02x %02x %02x %02x %02x %02x %02x"
			" %02x %02x %02x %02x %02x %02x %02x %02x\n", i,
			hdmi.edid[i], hdmi.edid[i + 1], hdmi.edid[i + 2],
			hdmi.edid[i + 3], hdmi.edid[i + 4], hdmi.edid[i + 5],
			hdmi.edid[i + 6], hdmi.edid[i + 7], hdmi.edid[i + 8],
			hdmi.edid[i + 9], hdmi.edid[i + 10], hdmi.edid[i + 11],
			hdmi.edid[i + 12], hdmi.edid[i + 13],
			hdmi.edid[i + 14], hdmi.edid[i + 15]);

	if (ret) {
		printk(KERN_WARNING "hdtv: failed to read E-EDID\n");
		return NULL;
	}

	if (memcmp(hdmi.edid, edid_header, sizeof(edid_header)))
		return NULL;

	hdmi.edid_set = true;
	return hdmi.edid;
}

static void hdmi_compute_pll(struct omap_dss_device *dssdev, int phy,
		struct hdmi_pll_info *pi)
{
	u32 clkinKHz;
	u32 phy_x_nplus1;
	u32 t;

	clkinKHz = clk_get_rate(hdmi.sys_clk) / 1000;

	/* Input clock is predivided by N + 1
	 * out put of which is reference clk
	 */
	pi->regn = dssdev->clocks.hdmi.regn;
	pi->regm2 = dssdev->clocks.hdmi.regm2;

	/* Cached multiple for math below */
	phy_x_nplus1 = phy * (pi->regn + 1);
	/* m = (((phy) * (n+1)) / clkin)
	 * The m register does a /10, so we need a value 10 times larger here.
	 * In order to maintain the most significant digits, the clkin value
	 * is divide by 10 here which results in a value 10 times larger.
	 */
	pi->regm = ((10 * phy_x_nplus1) / clkinKHz);
	/* mf (fractional part) = (remainder from regm) * (2^18) / clkin
	 *
	 * Unfortunately, there is a potential of overflowing a u32 if we mult
	 * by (2^18).  So, instead, the remainder will mult'd by (2^8) prior to
	 * the div by clkin and finally mult'd by (2^10) to ensure the right
	 * magnitude is maintained.  This method still results in a offset
	 * from the desired value, so to get very precise, the remainder from
	 * this division is mult'd by (2^10), divided, and that result is
	 * added to create the final very precise value.
	 *
	 * mf  = (((remainder of regm) << 8) / clkin) << 10
	 * mf += ((remainder of above) << 10) / clkin
	 */
	t = ((10 * phy_x_nplus1) % clkinKHz);
	pi->regmf = ((t << 8) / clkinKHz) << 10;
	t = ((t << 8) % clkinKHz);
	pi->regmf += (t << 10) / clkinKHz;

	/*
	 * Dcofreq should be set to 1 if required pixel clock
	 * is greater than 1000MHz
	 */
	pi->dcofreq = phy > (1000 * 100);
	pi->regsd = ((pi->regm * (clkinKHz/10) / 10) /
					((pi->regn + 1) * 250) + 5) / 10;

	HDTVDBG("M = %d Mf = %d\n", pi->regm, pi->regmf);
	HDTVDBG("range = %d sd = %d\n", pi->dcofreq, pi->regsd);
}

static void hdmi_load_hdcp_keys(struct omap_dss_device *dssdev)
{
	int aksv;

	DSSDBG("hdmi_load_hdcp_keys\n");
	/* load the keys and reset the wrapper to populate the AKSV registers*/
	if (hdmi.hdmi_power_on_cb) {
		aksv = hdmi_ti_4xx_check_aksv_data(&hdmi.hdmi_data);
		if ((aksv == HDMI_AKSV_ZERO) &&
		    hdmi.custom_set &&
		    hdmi.hdmi_power_on_cb()) {
			hdmi_ti_4xxx_set_wait_soft_reset(&hdmi.hdmi_data);
			/* HDCP keys are available in AKSV registers 2ms after
			 * the RESET# rising edge, hence delay before reading
			 * the registers*/
			mdelay(10);
			aksv = hdmi_ti_4xx_check_aksv_data(&hdmi.hdmi_data);
			hdmi.wp_reset_done = (aksv == HDMI_AKSV_VALID) ?
								true : false;
			DSSINFO("HDMI_WRAPPER RESET DONE\n");
		} else if (aksv == HDMI_AKSV_VALID)
			hdmi.wp_reset_done = true;
		else if (aksv == HDMI_AKSV_ERROR)
			hdmi.wp_reset_done = false;

		if (!hdmi.wp_reset_done)
			DSSERR("*** INVALID AKSV: %d "
				"Do not perform HDCP AUTHENTICATION\n", aksv);
	}

}
/* Set / Release c-state constraints */
static void hdmi_set_l3_cstr(struct omap_dss_device *dssdev, bool enable)
{
#ifdef CONFIG_OMAP_PM
	HDTVDBG("%s c-state constraint for HDMI\n",
		enable ? "Set" : "Release");

	if (enable)
		pm_qos_add_request(&pm_qos_handle, PM_QOS_CPU_DMA_LATENCY, 100);
	else
		pm_qos_remove_request(&pm_qos_handle);
#else
	HDTVDBG("C-STATE Constraints require COMFIG_OMAP_PM to be set\n");
#endif
}

static int hdmi_power_on(struct omap_dss_device *dssdev)
{
	int r;
	struct hdmi_pll_info pll_data;
	struct omap_video_timings *p;
	unsigned long phy;

	/* If we had already enabled the clks for EDID only mode
	 * we don't need to enable them here
	 */
	if (!hdmi.edid_only) {
		r = dss_clken_restore_ctx();
		if (r)
			return r;

		r = hdmi_runtime_get();
		if (r) {
			dss_clkdis_save_ctx(true);
			return r;
		}
	}

	hdmi_set_l3_cstr(dssdev, true);

	/* Load the HDCP keys if not already loaded*/
	hdmi_load_hdcp_keys(dssdev);

	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 0);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 0);

	p = &dssdev->panel.timings;

	if (!hdmi.custom_set) {
		struct fb_videomode vesa_vga = vesa_modes[4];
		hdmi_set_timings(&vesa_vga, false);
	}

	omapfb_fb2dss_timings(&hdmi.cfg.timings, &dssdev->panel.timings);

	HDTVDBG("hdmi_power_on/x %d/y %d;\n", p->x_res, p->y_res);

	phy = p->pixel_clock;

	switch (hdmi.deep_color) {
	case HDMI_DEEP_COLOR_30BIT:
		phy = (p->pixel_clock * 125) / 100 ;
		hdmi.cfg.deep_color = HDMI_DEEP_COLOR_30BIT;
		break;
	case HDMI_DEEP_COLOR_36BIT:
		if (p->pixel_clock == 148500) {
			printk(KERN_ERR "hdtv: 36 bit color not supported");
			goto err;
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
		HDTVDBG("Failed to lock PLL\n");
		goto err;
	}

	r = hdmi_ti_4xxx_phy_init(&hdmi.hdmi_data,
						dssdev->phy.hdmi.ds_percent);
	if (r) {
		HDTVDBG("Failed to start PHY\n");
		goto err;
	}

	hdmi.cfg.cm.mode = hdmi.can_do_hdmi ? hdmi.mode : HDMI_DVI;
	hdmi.cfg.cm.code = hdmi.code;
	hdmi_ti_4xxx_basic_configure(&hdmi.hdmi_data, &hdmi.cfg);

	/* Make selection of HDMI in DSS */
	dss_select_hdmi_venc_clk_source(DSS_HDMI_M_PCLK);

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

	if (hdmi.hdmi_start_frame_cb &&
	    hdmi.custom_set &&
	    hdmi.wp_reset_done)
		(*hdmi.hdmi_start_frame_cb)();

	return 0;
err:
	hdmi_set_l3_cstr(dssdev, false);
	hdmi_runtime_put();
	dss_clkdis_save_ctx(true);
	return -EIO;
}

static int hdmi_power_edid_only(struct omap_dss_device *dssdev)
{
	struct hdmi_pll_info       pll_data;
	struct omap_video_timings *p;
	struct fb_videomode        vesa_vga;
	int r;
	unsigned long phy;

	r = dss_clken_restore_ctx();
	if (r)
		return r;
	r = hdmi_runtime_get();
	if (r) {
		dss_clkdis_save_ctx(true);
		return r;
	}

	/* Load the HDCP keys if not already loaded*/
	hdmi_load_hdcp_keys(dssdev);

	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 0);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 0);

	p = &dssdev->panel.timings;

	HDTVDBG("hdmi_power_edid_only\n");

	vesa_vga = vesa_modes[4];
	hdmi_set_timings(&vesa_vga, false);

	omapfb_fb2dss_timings(&hdmi.cfg.timings, &dssdev->panel.timings);

	phy = p->pixel_clock;
	hdmi.cfg.deep_color = HDMI_DEEP_COLOR_24BIT;

	hdmi_compute_pll(dssdev, phy, &pll_data);

	/* config the PLL and PHY hdmi_set_pll_pwrfirst */
	r = hdmi_ti_4xxx_pll_program(&hdmi.hdmi_data, &pll_data);
	if (r) {
		HDTVDBG("Failed to lock PLL\n");
		goto err;
	}

	r = hdmi_ti_4xxx_phy_init(&hdmi.hdmi_data,
						dssdev->phy.hdmi.ds_percent);
	if (r) {
		HDTVDBG("Failed to start PHY\n");
		goto err;
	}

	hdmi.cfg.cm.mode = hdmi.can_do_hdmi ? hdmi.mode : HDMI_DVI;
	hdmi.cfg.cm.code = hdmi.code;
	hdmi_ti_4xxx_basic_configure(&hdmi.hdmi_data, &hdmi.cfg);

	return 0;
err:
	hdmi_runtime_put();
	dss_clkdis_save_ctx(true);
	return -EIO;
}

static void hdmi_power_off(struct omap_dss_device *dssdev)
{
	if (hdmi.hdmi_irq_cb)
		hdmi.hdmi_irq_cb(HDMI_HPD_LOW);

	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 0);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 0);
	hdmi_ti_4xxx_phy_off(&hdmi.hdmi_data, hdmi.set_mode);
	hdmi_ti_4xxx_set_pll_pwr(&hdmi.hdmi_data, HDMI_PLLPWRCMD_ALLOFF);
	if (!hdmi.edid_only)
		hdmi_set_l3_cstr(dssdev, false);
	hdmi_runtime_put();
	dss_clkdis_save_ctx(true);
	hdmi.deep_color = HDMI_DEEP_COLOR_24BIT;
}

int omapdss_hdmi_get_pixel_clock(void)
{
	return PICOS2KHZ(hdmi.cfg.timings.pixclock);
}

int omapdss_hdmi_get_mode(void)
{
	return hdmi.mode;
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
	int hpd;

	if (hdmi.hpd_en) {
		hpd = hdmi_get_current_hpd();

		HDTVDBG("hpd %d\n", hpd);

		hdmi_panel_hpd_handler(hpd);
	}

	return IRQ_HANDLED;
}

static irqreturn_t hdmi_irq_handler(int irq, void *arg)
{
	int r = 0;

	r = hdmi_ti_4xxx_irq_handler(&hdmi.hdmi_data);

	HDTVDBG("Received HDMI IRQ = %08x\n", r);

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
	int rc = 0;
	int audio_en;
	int dssmgr;

	HDTVDBG("omapdss_hdmi_display_set_mode\n");

	/* Have to grab these first since "hdmi_set_timings" may change vm */
	audio_en = (vm->flag & FB_MODE_FLAG_DVI_AUDIO) ? 1 : 0;
	dssmgr   = (vm->flag & FB_MODE_FLAG_DSSMGR)    ? 1 : 0;

	rc = hdmi_set_timings(vm, false) ? 0 : -EINVAL;
	hdmi.custom_set = 1;
	hdmi.mode = hdmi.cfg.cm.mode;
	hdmi.code = hdmi.cfg.cm.code;
	/* Force audio for DVI resolutions if requested */
	if (audio_en && hdmi.mode == HDMI_DVI) {
		hdmi.mode = HDMI_HDMI;
		hdmi.code = 0;
	}

	/* The DSSMGR flag chk here is just a hack to allow this driver
	 * to function with or without the DSSMGR, mostly for bringup
	 * purposes, but it doesn't hurt to keep around
	 */
	if (rc == 0 && (!dssmgr || hdmi.enabled)) {
		/* turn the hdmi off and on to get new timings to use */
		hdmi.set_mode = true;
		dssdev->driver->disable(dssdev);
		hdmi.set_mode = false;
		rc = dssdev->driver->enable(dssdev);
	}

	return rc;
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

int omapdss_hdmi_display_enable(struct omap_dss_device *dssdev, int edid_only)
{
	int r = 0;

	HDTVDBG("hdmi_display_enable\n");

	mutex_lock(&hdmi.lock);

	if (edid_only && (hdmi.edid_only || hdmi.enabled))
		goto err0;
	if (!edid_only && hdmi.enabled)
		goto err0;

	r = omap_dss_start_device(dssdev);
	if (r) {
		printk(KERN_ERR "hdtv: failed to start device\n");
		goto err0;
	}

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r) {
			printk(KERN_ERR "hdtv: failed to enable GPIO's\n");
			goto err1;
		}
	}

	if (edid_only)
		r = hdmi_power_edid_only(dssdev);
	else
		r = hdmi_power_on(dssdev);
	if (r) {
		printk(KERN_ERR "hdtv: failed to power on device\n");
		goto err2;
	}

	if (edid_only) {
		hdmi.edid_only = true;
	} else {
		hdmi.enabled = true;
		hdmi.edid_only = false;
	}

	mutex_unlock(&hdmi.lock);
	return 0;

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
	HDTVDBG("hdmi_display_disable\n");

	mutex_lock(&hdmi.lock);

	if (!hdmi.enabled && !hdmi.edid_only)
		goto done;

	/* Need to power off before clearing the flags */
	hdmi_power_off(dssdev);
	hdmi.enabled = false;
	hdmi.edid_only = false;
	hdmi.wp_reset_done = false;

	if (dssdev->sync_lost_error == 0)
		if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
			/* clear EDID and mode on disable only */
			hdmi.edid_set = false;
			hdmi.custom_set = 0;
			HDTVDBG("clearing EDID info\n");
		}

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omap_dss_stop_device(dssdev);
done:
	mutex_unlock(&hdmi.lock);
}

int mapphone_omapdss_hdmi_get_edid(struct omap_dss_device *dssdev, u8 *edid, int len)
{
	int rc = -1;
	int size = HDMI_EDID_MAX_LENGTH;

	if (edid == NULL) {
		printk(KERN_ERR "hdtv: Invalid edid pointer\n");
	} else {
		if (!hdmi.edid_set)
			hdmi_read_edid(NULL);

		if (!hdmi.edid_set) {
			printk(KERN_ERR "hdtv: No EDID found\n");
		} else {
			if (len < HDMI_EDID_MAX_LENGTH)
				size = len;
			memcpy(edid, hdmi.edid, size);
			rc = 0;
		}
	}

	return rc;
}

int omapdss_set_hdmi_mode(struct omap_dss_device *dssdev, int code)
{
	int rc = 0;

	HDTVDBG("omapdss_set_hdmi_mode (%d/%d)\n", code, hdmi.enabled);

	if (code <= 0 && code >= CEA_MODEDB_SIZE) {
		printk(KERN_ERR "hdtv: Invalid code\n");
		rc = -EIO;
	} else {
		hdmi.custom_set = 1;
		hdmi.mode = HDMI_HDMI;
		hdmi.code = code;
		hdmi.cfg.timings = cea_modes[code];
	}

	if (rc == 0 && hdmi.enabled) {
		printk(KERN_WARNING "hdtv: omapdss_set_hdmi_mode force\n");
		/* turn the hdmi off and on to get new timings to use */
		hdmi.set_mode = true;
		dssdev->driver->disable(dssdev);
		hdmi.set_mode = false;
		hdmi.custom_set = 1;
		rc = dssdev->driver->enable(dssdev);
	}

	return rc;
}

int omapdss_set_hdmi_hpd(struct omap_dss_device *dssdev, bool enable)
{
	int rc = 0;

	HDTVDBG("omapdss_set_hdmi_hpd (%d)\n", enable);

	if (enable == hdmi.hpd_en)
		return 0;

	hdmi.hpd_en = enable;

	if (enable) {
		if (dssdev->platform_enable_hpd)
			rc = dssdev->platform_enable_hpd(dssdev);

		if (!rc && hdmi_get_current_hpd())
			hdmi_panel_hpd_handler(1);
	} else {
		if (dssdev->platform_disable_hpd)
			dssdev->platform_disable_hpd(dssdev);

		hdmi_panel_hpd_handler(0);
	}

	return rc;
}

void omapdss_set_hdmi_test(int test)
{
	if (hdmi.dssdev->set_backlight)
		hdmi.dssdev->set_backlight(hdmi.dssdev, test);
}

static int hdmi_get_clocks(struct platform_device *pdev)
{
	struct clk *clk;

	clk = clk_get(&pdev->dev, "sys_clk");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "hdtv: can't get sys_clk\n");
		return PTR_ERR(clk);
	}

	hdmi.sys_clk = clk;

	clk = clk_get(&pdev->dev, "hdmi_clk");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "hdtv: can't get hdmi_clk\n");
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
		printk(KERN_ERR "hdtv: can't get HDMI device\n");
		return -EINVAL;
	}

	hdmi_mem = platform_get_resource(hdmi.pdev, IORESOURCE_MEM, 0);
	if (!hdmi_mem) {
		printk(KERN_ERR "hdtv: can't get IORESOURCE_MEM HDMI\n");
		return -EINVAL;
	}

	/* Base address taken from platform */
	hdmi.hdmi_data.base_wp = ioremap(hdmi_mem->start,
						resource_size(hdmi_mem));
	if (!hdmi.hdmi_data.base_wp) {
		printk(KERN_ERR "hdtv: can't ioremap WP\n");
		return -ENOMEM;
	}

	r = hdmi_get_clocks(pdev);
	if (r) {
		iounmap(hdmi.hdmi_data.base_wp);
		return r;
	}

	pm_runtime_enable(&pdev->dev);

	r = request_irq(gpio_to_irq(hdmi.dssdev->hpd_gpio), hpd_irq_handler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING,
			"hpd", NULL);
	if (r < 0) {
		printk(KERN_ERR "hdtv: request_irq %d failed\n",
			gpio_to_irq(hdmi.dssdev->hpd_gpio));
		return -EINVAL;
	}

	hdmi.hdmi_irq = platform_get_irq(pdev, 0);

	r = request_irq(hdmi.hdmi_irq, hdmi_irq_handler, 0, "OMAP HDMI", NULL);
	if (r < 0) {
		printk(KERN_ERR "hdtv: request_irq %s failed\n",
			pdev->name);
		return -EINVAL;
	}

	hdmi.hdmi_data.hdmi_core_sys_offset = HDMI_CORE_SYS;
	hdmi.hdmi_data.hdmi_core_av_offset = HDMI_CORE_AV;
	hdmi.hdmi_data.hdmi_pll_offset = HDMI_PLLCTRL;
	hdmi.hdmi_data.hdmi_phy_offset = HDMI_PHY;
	hdmi.wp_reset_done = false;

	/* Init the drive strength percent to full power */
	hdmi.dssdev->phy.hdmi.ds_percent = 100;

	hdmi_panel_init();

	if (hdmi_get_current_hpd())
		hdmi_panel_hpd_handler(1);

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
