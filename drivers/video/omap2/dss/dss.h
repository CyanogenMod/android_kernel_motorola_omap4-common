/*
 * linux/drivers/video/omap2/dss/dss.h
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
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

#ifndef __OMAP2_DSS_H
#define __OMAP2_DSS_H

#ifdef CONFIG_OMAP2_DSS_DEBUG_SUPPORT
#define DEBUG
#endif

#ifdef DEBUG
extern unsigned int dss_debug;
#ifdef DSS_SUBSYS_NAME
#define DSSDBG(format, ...) \
	if (dss_debug) \
		printk(KERN_DEBUG "omapdss " DSS_SUBSYS_NAME ": " format, \
		## __VA_ARGS__)
#else
#define DSSDBG(format, ...) \
	if (dss_debug) \
		printk(KERN_DEBUG "omapdss: " format, ## __VA_ARGS__)
#endif

#ifdef DSS_SUBSYS_NAME
#define DSSDBGF(format, ...) \
	if (dss_debug) \
		printk(KERN_DEBUG "omapdss " DSS_SUBSYS_NAME \
				": %s(" format ")\n", \
				__func__, \
				## __VA_ARGS__)
#else
#define DSSDBGF(format, ...) \
	if (dss_debug) \
		printk(KERN_DEBUG "omapdss: " \
				": %s(" format ")\n", \
				__func__, \
				## __VA_ARGS__)
#endif

#else /* DEBUG */
#define DSSDBG(format, ...)
#define DSSDBGF(format, ...)
#endif


#ifdef DSS_SUBSYS_NAME
#define DSSERR(format, ...) \
	printk(KERN_ERR "omapdss " DSS_SUBSYS_NAME " error: " format, \
	## __VA_ARGS__)
#else
#define DSSERR(format, ...) \
	printk(KERN_ERR "omapdss error: " format, ## __VA_ARGS__)
#endif

#ifdef DSS_SUBSYS_NAME
#define DSSINFO(format, ...) \
	printk(KERN_INFO "omapdss " DSS_SUBSYS_NAME ": " format, \
	## __VA_ARGS__)
#else
#define DSSINFO(format, ...) \
	printk(KERN_INFO "omapdss: " format, ## __VA_ARGS__)
#endif

#ifdef DSS_SUBSYS_NAME
#define DSSWARN(format, ...) \
	printk(KERN_WARNING "omapdss " DSS_SUBSYS_NAME ": " format, \
	## __VA_ARGS__)
#else
#define DSSWARN(format, ...) \
	printk(KERN_WARNING "omapdss: " format, ## __VA_ARGS__)
#endif

/* OMAP TRM gives bitfields as start:end, where start is the higher bit
   number. For example 7:0 */
#define FLD_MASK(start, end)	(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

enum omap_burst_size {
	OMAP_DSS_BURST_4x32 = 0,
	OMAP_DSS_BURST_8x32 = 1,
	OMAP_DSS_BURST_16x32 = 2,
};

enum omap_parallel_interface_mode {
	OMAP_DSS_PARALLELMODE_BYPASS,		/* MIPI DPI */
	OMAP_DSS_PARALLELMODE_RFBI,		/* MIPI DBI */
	OMAP_DSS_PARALLELMODE_DSI,
};

enum dss_hdmi_venc_clk_source_select {
	DSS_VENC_TV_CLK = 0,
	DSS_HDMI_M_PCLK = 1,
};

struct dss_clock_info {
	/* rates that we get with dividers below */
	unsigned long fck;

	/* dividers */
	u16 fck_div;
};

struct dispc_clock_info {
	/* rates that we get with dividers below */
	unsigned long lck;
	unsigned long pck;

	/* dividers */
	u16 lck_div;
	u16 pck_div;
};

struct dsi_clock_info {
	/* rates that we get with dividers below */
	unsigned long fint;
	unsigned long clkin4ddr;
	unsigned long clkin;
	unsigned long dsi_pll_hsdiv_dispc_clk;	/* OMAP3: DSI1_PLL_CLK
						 * OMAP4: PLLx_CLK1 */
	unsigned long dsi_pll_hsdiv_dsi_clk;	/* OMAP3: DSI2_PLL_CLK
						 * OMAP4: PLLx_CLK2 */
	unsigned long lp_clk;

	/* dividers */
	u16 regn;
	u16 regm;
	u16 regm_dispc;	/* OMAP3: REGM3
			 * OMAP4: REGM4 */
	u16 regm_dsi;	/* OMAP3: REGM4
			 * OMAP4: REGM5 */
	u16 lp_clk_div;

	u8 highfreq;
	bool use_sys_clk;
};

struct dispc_config {
	u32 sizex, sizey;
	u32 burstsize;
	u32 pixelinc;
	u32 rowinc;
	u32 bursttype;
	u32 antiflicker;
	u32 doublestride;
	u32 ba;
	u32 bacbcr;
	u32 format;
	u32 rotation;
	u32 gfx_top_buffer;
	u32 gfx_bottom_buffer;
	u32 vid1_top_buffer;
	u32 vid1_bottom_buffer;
	u32 vid2_top_buffer;
	u32 vid2_bottom_buffer;
	u32 vid3_top_buffer;
	u32 vid3_bottom_buffer;
	u32 wb_top_buffer;
	u32 wb_bottom_buffer;
};

/*TODO: Move this structure to manager.c*/
struct writeback_cache_data {
	/* If true, cache changed, but not written to shadow registers. Set
	 * in apply(), cleared when registers written.
	 */
	bool dirty;
	/* If true, shadow registers contain changed values not yet in real
	 * registers. Set when writing to shadow registers, cleared at
	 * VSYNC/EVSYNC
	 */
	bool shadow_dirty;
	bool enabled;
	u32 paddr;
	u32 p_uv_addr; /* relevant for NV12 format only */
	u16 out_width;
	u16 out_height;
	u16 width;
	u16 height;
	u32	fifo_low;
	u32	fifo_high;
	enum omap_color_mode			color_mode;
	enum omap_color_mode			input_color_mode;
	enum omap_writeback_capturemode	capturemode;
	enum omap_writeback_source		source;
	enum omap_burst_size			burst_size;
	enum omap_writeback_mode		mode;
	u8					rotation;
	enum omap_dss_rotation_type		rotation_type;
};

struct seq_file;
struct platform_device;

/* core */
struct bus_type *dss_get_bus(void);
struct regulator *dss_get_vdds_dsi(void);
struct regulator *dss_get_vdds_sdi(void);
void omap_dss_request_high_bandwidth(struct device *dss_dev);
void omap_dss_reset_high_bandwidth(struct device *dss_dev);
void omap_dss_overlay_ensure_bw(void);

/* display */
int dss_suspend_all_devices(void);
int dss_resume_all_devices(void);
void dss_disable_all_devices(void);

void dss_init_device(struct platform_device *pdev,
		struct omap_dss_device *dssdev);
void dss_uninit_device(struct platform_device *pdev,
		struct omap_dss_device *dssdev);
bool dss_use_replication(struct omap_dss_device *dssdev,
		enum omap_color_mode mode);
void default_get_overlay_fifo_thresholds(enum omap_plane plane,
		u32 fifo_size, enum omap_burst_size *burst_size,
		u32 *fifo_low, u32 *fifo_high);
int dss_clkdis_save_ctx(bool do_clk_disable);
int dss_clken_restore_ctx(void);

/* manager */
int dss_init_overlay_managers(struct platform_device *pdev);
void dss_uninit_overlay_managers(struct platform_device *pdev);
int dss_mgr_wait_for_go_ovl(struct omap_overlay *ovl);
int dss_setup_partial_planes(struct omap_dss_device *dssdev,
				u16 *x, u16 *y, u16 *w, u16 *h,
				bool enlarge_update_area);
void dss_start_update(struct omap_dss_device *dssdev);

/* overlay */
void dss_init_overlays(struct platform_device *pdev);
void dss_uninit_overlays(struct platform_device *pdev);
int dss_check_overlay(struct omap_overlay *ovl, struct omap_dss_device *dssdev);
void dss_overlay_setup_dispc_manager(struct omap_overlay_manager *mgr);
#ifdef L4_EXAMPLE
void dss_overlay_setup_l4_manager(struct omap_overlay_manager *mgr);
#endif
void dss_recheck_connections(struct omap_dss_device *dssdev, bool force);
/* Write back */
void dss_init_writeback(struct platform_device *pdev);
void dss_uninit_writeback(struct platform_device *pdev);
bool omap_dss_check_wb(struct writeback_cache_data *wb, int overlayId,
			int managerId);

/* DSS */
int dss_init_platform_driver(void);
void dss_uninit_platform_driver(void);

int dss_runtime_get(void);
void dss_runtime_put(void);

void dss_save_context(void);
void dss_restore_context(void);

void dss_select_hdmi_venc_clk_source(enum dss_hdmi_venc_clk_source_select);
const char *dss_get_generic_clk_source_name(enum omap_dss_clk_source clk_src);
void dss_dump_clocks(struct seq_file *s);

void dss_dump_regs(struct seq_file *s);
#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_OMAP2_DSS_DEBUG_SUPPORT)
void dss_debug_dump_clocks(struct seq_file *s);
#endif

void dss_sdi_init(u8 datapairs);
int dss_sdi_enable(void);
void dss_sdi_disable(void);

void dss_select_dispc_clk_source(enum omap_dss_clk_source clk_src);
void dss_select_dsi_clk_source(int dsi_module,
		enum omap_dss_clk_source clk_src);
void dss_select_lcd_clk_source(enum omap_channel channel,
		enum omap_dss_clk_source clk_src);
enum omap_dss_clk_source dss_get_dispc_clk_source(void);
enum omap_dss_clk_source dss_get_dsi_clk_source(int dsi_module);
enum omap_dss_clk_source dss_get_lcd_clk_source(enum omap_channel channel);

void dss_set_venc_output(enum omap_dss_venc_type type);
void dss_set_dac_pwrdn_bgz(bool enable);

unsigned long dss_get_dpll4_rate(void);
int dss_calc_clock_rates(struct dss_clock_info *cinfo);
int dss_set_clock_div(struct dss_clock_info *cinfo);
int dss_get_clock_div(struct dss_clock_info *cinfo);
int dss_calc_clock_div(bool is_tft, unsigned long req_pck,
		struct dss_clock_info *dss_cinfo,
		struct dispc_clock_info *dispc_cinfo);

/* SDI */
#ifdef CONFIG_OMAP2_DSS_SDI
int sdi_init(void);
void sdi_exit(void);
int sdi_init_display(struct omap_dss_device *display);
#else
static inline int sdi_init(void)
{
	return 0;
}
static inline void sdi_exit(void)
{
}
#endif

/* DSI */
#ifdef CONFIG_OMAP2_DSS_DSI

struct dentry;
struct file_operations;

int dsi_init_platform_driver(void);
void dsi_uninit_platform_driver(void);

int dsi_runtime_get(void);
void dsi_runtime_put(void);

void dsi_dump_clocks(struct seq_file *s);
void dsi_create_debugfs_files_irq(struct dentry *debugfs_dir,
		const struct file_operations *debug_fops);
void dsi_create_debugfs_files_reg(struct dentry *debugfs_dir,
		const struct file_operations *debug_fops);

int dsi_init_display(struct omap_dss_device *display);
void dsi_irq_handler(void);
unsigned long dsi_get_pll_hsdiv_dispc_rate(struct platform_device *dsidev);
int dsi_pll_set_clock_div(struct platform_device *dsidev,
		struct dsi_clock_info *cinfo);
int dsi_pll_calc_clock_div_pck(struct platform_device *dsidev, bool is_tft,
		unsigned long req_pck, struct dsi_clock_info *cinfo,
		struct dispc_clock_info *dispc_cinfo);
int dsi_pll_init(struct platform_device *dsidev, bool enable_hsclk,
		bool enable_hsdiv);
void dsi_pll_uninit(struct platform_device *dsidev, bool disconnect_lanes);
void dsi_get_overlay_fifo_thresholds(enum omap_plane plane,
		u32 fifo_size, enum omap_burst_size *burst_size,
		u32 *fifo_low, u32 *fifo_high);
void dsi_wait_pll_hsdiv_dispc_active(struct platform_device *dsidev);
void dsi_wait_pll_hsdiv_dsi_active(struct platform_device *dsidev);
struct platform_device *dsi_get_dsidev_from_id(int module);
#else
static inline int dsi_init_platform_driver(void)
{
	return 0;
}
static inline void dsi_uninit_platform_driver(void)
{
}
static inline int dsi_runtime_get(struct platform_device *dsidev)
{
	return 0;
}
static inline void dsi_runtime_put(struct platform_device *dsidev)
{
}
static inline unsigned long dsi_get_pll_hsdiv_dispc_rate(struct platform_device *dsidev)
{
	WARN("%s: DSI not compiled in, returning rate as 0\n", __func__);
	return 0;
}
static inline int dsi_pll_set_clock_div(struct platform_device *dsidev,
		struct dsi_clock_info *cinfo)
{
	WARN("%s: DSI not compiled in\n", __func__);
	return -ENODEV;
}
static inline int dsi_pll_calc_clock_div_pck(struct platform_device *dsidev,
		bool is_tft, unsigned long req_pck,
		struct dsi_clock_info *dsi_cinfo,
		struct dispc_clock_info *dispc_cinfo)
{
	WARN("%s: DSI not compiled in\n", __func__);
	return -ENODEV;
}
static inline int dsi_pll_init(struct platform_device *dsidev,
		bool enable_hsclk, bool enable_hsdiv)
{
	WARN("%s: DSI not compiled in\n", __func__);
	return -ENODEV;
}
static inline void dsi_pll_uninit(struct platform_device *dsidev,
		bool disconnect_lanes)
{
}
static inline void dsi_wait_pll_hsdiv_dispc_active(struct platform_device *dsidev)
{
}
static inline void dsi_wait_pll_hsdiv_dsi_active(struct platform_device *dsidev)
{
}
static inline struct platform_device *dsi_get_dsidev_from_id(int module)
{
	WARN("%s: DSI not compiled in, returning platform device as NULL\n",
			__func__);
	return NULL;
}
#endif

/* DPI */
#ifdef CONFIG_OMAP2_DSS_DPI
int dpi_init(void);
void dpi_exit(void);
int dpi_init_display(struct omap_dss_device *dssdev);
#else
static inline int dpi_init(void)
{
	return 0;
}
static inline void dpi_exit(void)
{
}
#endif

/* DISPC */
int dispc_init_platform_driver(void);
void dispc_uninit_platform_driver(void);
void dispc_dump_clocks(struct seq_file *s);
void dispc_dump_irqs(struct seq_file *s);
void dispc_dump_regs(struct seq_file *s);
void dispc_irq_handler(void);
void dispc_fake_vsync_irq(void);

int dispc_runtime_get(void);
void dispc_runtime_put(void);

void dispc_save_context(void);
void dispc_restore_context(void);

void dispc_enable_sidle(void);
void dispc_disable_sidle(void);

void dispc_lcd_enable_signal_polarity(bool act_high);
void dispc_lcd_enable_signal(bool enable);
void dispc_pck_free_enable(bool enable);
void dispc_enable_fifohandcheck(enum omap_channel channel, bool enable);

void dispc_set_lcd_size(enum omap_channel channel, u16 width, u16 height);
void dispc_set_digit_size(u16 width, u16 height);
u32 dispc_get_plane_fifo_size(enum omap_plane plane);
void dispc_setup_plane_fifo(enum omap_plane plane, u32 low, u32 high);
void dispc_enable_fifomerge(bool enable);
void dispc_set_burst_size(enum omap_plane plane,
		enum omap_burst_size burst_size);
void dispc_set_zorder(enum omap_plane plane,
			enum omap_overlay_zorder zorder);
void dispc_enable_zorder(enum omap_plane plane, bool enable);
void dispc_enable_cpr(enum omap_channel channel, bool enable);
void dispc_set_cpr_coef(enum omap_channel channel,
		struct omap_dss_cpr_coefs *coefs);
void _dispc_setup_color_conv_coef(enum omap_plane plane,
	const struct omap_dss_cconv_coefs *ct);

void dispc_set_plane_ba0(enum omap_plane plane, u32 paddr);
void dispc_set_plane_ba1(enum omap_plane plane, u32 paddr);
void dispc_set_plane_pos(enum omap_plane plane, u16 x, u16 y);
void dispc_set_plane_size(enum omap_plane plane, u16 width, u16 height);
void dispc_set_channel_out(enum omap_plane plane,
		enum omap_channel channel_out);
void dispc_set_wb_channel_out(enum omap_plane plane);

void dispc_enable_gamma_table(bool enable);
int dispc_setup_plane(enum omap_plane plane,
		      u32 paddr, u16 screen_width,
		      u16 pos_x, u16 pos_y,
		      u16 width, u16 height,
		      u16 out_width, u16 out_height,
		      enum omap_color_mode color_mode,
		      bool ilace,
		      int x_decim, int y_decim, bool five_taps,
		      enum omap_dss_rotation_type rotation_type,
		      u8 rotation, bool mirror,
		      u8 global_alpha, u8 pre_mult_alpha,
		      enum omap_channel channel,
		      u32 puv_addr, bool source_of_wb);
int dispc_scaling_decision(u16 width, u16 height,
		u16 out_width, u16 out_height,
		enum omap_plane plane,
		enum omap_color_mode color_mode,
		enum omap_channel channel, u8 rotation,
		enum omap_dss_rotation_type type,
		u16 min_x_decim, u16 max_x_decim,
		u16 min_y_decim, u16 max_y_decim,
		u16 *x_decim, u16 *y_decim, bool *three_tap);

bool dispc_go_busy(enum omap_channel channel);
void dispc_go(enum omap_channel channel);
void dispc_enable_channel(enum omap_channel channel,
		enum omap_display_type type, bool enable);
bool dispc_is_channel_enabled(enum omap_channel channel);
int dispc_enable_plane(enum omap_plane plane, bool enable);
void dispc_enable_replication(enum omap_plane plane, bool enable);

void dispc_set_parallel_interface_mode(enum omap_channel channel,
		enum omap_parallel_interface_mode mode);
void dispc_set_tft_data_lines(enum omap_channel channel, u8 data_lines);
void dispc_set_lcd_display_type(enum omap_channel channel,
		enum omap_lcd_display_type type);
void dispc_set_loadmode(enum omap_dss_load_mode mode);

void dispc_set_default_color(enum omap_channel channel, u32 color);
u32 dispc_get_default_color(enum omap_channel channel);
void dispc_set_trans_key(enum omap_channel ch,
		enum omap_dss_trans_key_type type,
		u32 trans_key);
void dispc_get_trans_key(enum omap_channel ch,
		enum omap_dss_trans_key_type *type,
		u32 *trans_key);
void dispc_enable_trans_key(enum omap_channel ch, bool enable);
void dispc_enable_alpha_blending(enum omap_channel ch, bool enable);
bool dispc_trans_key_enabled(enum omap_channel ch);
bool dispc_alpha_blending_enabled(enum omap_channel ch);

bool dispc_lcd_timings_ok(struct omap_video_timings *timings);
void dispc_set_lcd_timings(enum omap_channel channel,
		struct omap_video_timings *timings);
unsigned long dispc_fclk_rate(void);
unsigned long dispc_lclk_rate(enum omap_channel channel);
unsigned long dispc_pclk_rate(enum omap_channel channel);
void dispc_set_pol_freq(enum omap_channel channel,
		enum omap_panel_config config, u8 acbi, u8 acb);
void dispc_find_clk_divs(bool is_tft, unsigned long req_pck, unsigned long fck,
		struct dispc_clock_info *cinfo);
int dispc_calc_clock_rates(unsigned long dispc_fclk_rate,
		struct dispc_clock_info *cinfo);
int dispc_set_clock_div(enum omap_channel channel,
		struct dispc_clock_info *cinfo);
int dispc_get_clock_div(enum omap_channel channel,
		struct dispc_clock_info *cinfo);
u32 sa_calc_wrap(struct dispc_config *dispc_reg_config, u32 channel_no);
int dispc_setup_wb(struct writeback_cache_data *wb);
void dispc_setup_wb_source(enum omap_writeback_source source);
void dispc_go_wb(void);

/* VENC */
#ifdef CONFIG_OMAP2_DSS_VENC
int venc_init_platform_driver(void);
void venc_uninit_platform_driver(void);
void venc_dump_regs(struct seq_file *s);
int venc_init_display(struct omap_dss_device *display);
#else
static inline int venc_init_platform_driver(void)
{
	return 0;
}
static inline void venc_uninit_platform_driver(void)
{
}
#endif

/* HDMI */
#ifdef CONFIG_OMAP4_DSS_HDMI
int hdmi_init_platform_driver(void);
void hdmi_uninit_platform_driver(void);
int hdmi_init_display(struct omap_dss_device *dssdev);
#else
static inline int hdmi_init_display(struct omap_dss_device *dssdev)
{
	return 0;
}
static inline int hdmi_init_platform_driver(void)
{
	return 0;
}
static inline void hdmi_uninit_platform_driver(void)
{
}
#endif
int omapdss_hdmi_display_enable(struct omap_dss_device *dssdev);
void omapdss_hdmi_display_disable(struct omap_dss_device *dssdev);
void omapdss_hdmi_display_set_timing(struct omap_dss_device *dssdev);
int omapdss_hdmi_display_check_timing(struct omap_dss_device *dssdev,
					struct omap_video_timings *timings);
int omapdss_hdmi_display_set_mode(struct omap_dss_device *dssdev,
					struct fb_videomode *mode);
int omapdss_hdmi_display_set_initial_mode(struct omap_dss_device *dssdev);

void omapdss_hdmi_restart(void);
int hdmi_panel_hpd_handler(int hpd);
int omapdss_hdmi_get_pixel_clock(void);
int omapdss_hdmi_get_mode(void);
int omapdss_hdmi_get_deepcolor(void);
ssize_t omapdss_hdmi_get_edid(char *edid);
void omapdss_hdmi_set_deepcolor(int val);
int omapdss_hdmi_get_s3d_mode(void);
void omapdss_hdmi_set_s3d_mode(int val);
void omapdss_hdmi_enable_s3d(bool enable);
int omapdss_hdmi_get_s3d_enable(void);

int hdmi_get_current_hpd(void);
void hdmi_get_monspecs(struct fb_monspecs *specs);
void hdmi_inform_hpd_to_cec(int status);
void hdmi_inform_power_on_to_cec(int status);
u8 *hdmi_read_edid(struct omap_video_timings *);
void hdmi_set_edid_state(bool val);

int hdmi_panel_init(void);
void hdmi_panel_exit(void);
void hdmi_dump_regs(struct seq_file *s);
int omapdss_hdmi_register_hdcp_callbacks(void (*hdmi_start_frame_cb)(void),
					 void (*hdmi_irq_cb)(int status),
					 bool (*hdmi_power_on_cb)(void));
int omapdss_hdmi_register_cec_callbacks(void (*hdmi_cec_enable_cb)(int status),
					 void (*hdmi_cec_irq_cb)(void),
					 void (*hdmi_cec_hpd)(int phy_addr,
					 int status));
int omapdss_hdmi_unregister_cec_callbacks(void);

int omap_dss_ovl_set_info(struct omap_overlay *ovl,
		struct omap_overlay_info *info);

/*#ifdef CONFIG_PANEL_MAPPHONE_OMAP4_HDTV
#define FB_MODE_FLAG_DVI_AUDIO   (1 << 30)
#define FB_MODE_FLAG_DSSMGR      (1 << 29)
int omapdss_hdmi_display_enable(struct omap_dss_device *dssdev, int edid_only);
int mapphone_omapdss_hdmi_get_edid(struct omap_dss_device *dssdev, u8 *edid, int len);
int omapdss_set_hdmi_mode(struct omap_dss_device *dssdev, int code);
int omapdss_set_hdmi_hpd(struct omap_dss_device *dssdev, bool enable);
void omapdss_set_hdmi_test(int test);
#endif
*/
/* RFBI */
#ifdef CONFIG_OMAP2_DSS_RFBI
int rfbi_init_platform_driver(void);
void rfbi_uninit_platform_driver(void);
void rfbi_dump_regs(struct seq_file *s);
int rfbi_init_display(struct omap_dss_device *display);
#else
static inline int rfbi_init_platform_driver(void)
{
	return 0;
}
static inline void rfbi_uninit_platform_driver(void)
{
}
#endif


#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
static inline void dss_collect_irq_stats(u32 irqstatus, unsigned *irq_arr)
{
	int b;
	for (b = 0; b < 32; ++b) {
		if (irqstatus & (1 << b))
			irq_arr[b]++;
	}
}
#endif

#endif
