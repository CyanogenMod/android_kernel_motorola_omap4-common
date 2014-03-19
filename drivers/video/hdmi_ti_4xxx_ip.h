/*
 * hdmi_ti_4xxx_ip.h
 *
 * HDMI header definition for TI81xx, TI38xx, TI OMAP4 etc processors.
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _HDMI_TI_4xxx_H_
#define _HDMI_TI_4xxx_H_

#include <linux/string.h>
#include <video/omapdss.h>
#include <video/hdmi_ti_4xxx_ip.h>

struct hdmi_reg { u16 idx; };

#define HDMI_REG(idx)			((const struct hdmi_reg) { idx })

/* HDMI Wrapper */
#define HDMI_WP_REG(idx)			HDMI_REG(idx)

#define HDMI_WP_REVISION			HDMI_WP_REG(0x0)
#define HDMI_WP_SYSCONFIG			HDMI_WP_REG(0x10)
#define HDMI_WP_IRQSTATUS_RAW			HDMI_WP_REG(0x24)
#define HDMI_WP_IRQSTATUS			HDMI_WP_REG(0x28)
#define HDMI_WP_PWR_CTRL			HDMI_WP_REG(0x40)
#define HDMI_WP_IRQENABLE_SET			HDMI_WP_REG(0x2C)
#define HDMI_WP_VIDEO_CFG			HDMI_WP_REG(0x50)
#define HDMI_WP_VIDEO_SIZE			HDMI_WP_REG(0x60)
#define HDMI_WP_VIDEO_TIMING_H			HDMI_WP_REG(0x68)
#define HDMI_WP_VIDEO_TIMING_V			HDMI_WP_REG(0x6C)
#define HDMI_WP_WP_CLK				HDMI_WP_REG(0x70)
#define HDMI_WP_AUDIO_CFG			HDMI_WP_REG(0x80)
#define HDMI_WP_AUDIO_CFG2			HDMI_WP_REG(0x84)
#define HDMI_WP_AUDIO_CTRL			HDMI_WP_REG(0x88)
#define HDMI_WP_AUDIO_DATA			HDMI_WP_REG(0x8C)
#define HDMI_WP_IRQSTATUS_CORE			0x1
#define HDMI_WP_IRQENABLE_CORE			0x1
#define HDMI_WP_AUDIO_FIFO_UNDERFLOW		(0x1 << 8)

/* HDMI IP Core System */
#define HDMI_CORE_SYS_REG(idx)			HDMI_REG(idx)

#define HDMI_CORE_SYS_VND_IDL			HDMI_CORE_SYS_REG(0x0)
#define HDMI_CORE_SYS_DEV_IDL			HDMI_CORE_SYS_REG(0x8)
#define HDMI_CORE_SYS_DEV_IDH			HDMI_CORE_SYS_REG(0xC)
#define HDMI_CORE_SYS_DEV_REV			HDMI_CORE_SYS_REG(0x10)
#define HDMI_CORE_SYS_SRST			HDMI_CORE_SYS_REG(0x14)
#define HDMI_CORE_CTRL1			HDMI_CORE_SYS_REG(0x20)
#define HDMI_CORE_SYS_SYS_STAT			HDMI_CORE_SYS_REG(0x24)
#define HDMI_CORE_SYS_VID_ACEN			HDMI_CORE_SYS_REG(0x124)
#define HDMI_CORE_SYS_VID_MODE			HDMI_CORE_SYS_REG(0x128)
#define HDMI_CORE_SYS_INTR_STATE		HDMI_CORE_SYS_REG(0x1C0)
#define HDMI_CORE_SYS_INTR1			HDMI_CORE_SYS_REG(0x1C4)
#define HDMI_CORE_SYS_INTR2			HDMI_CORE_SYS_REG(0x1C8)
#define HDMI_CORE_SYS_INTR3			HDMI_CORE_SYS_REG(0x1CC)
#define HDMI_CORE_SYS_INTR4			HDMI_CORE_SYS_REG(0x1D0)
#define HDMI_CORE_SYS_UMASK1			HDMI_CORE_SYS_REG(0x1D4)
#define HDMI_CORE_SYS_UMASK4			HDMI_CORE_SYS_REG(0x1E0)

#define HDMI_CORE_SYS_TMDS_CTRL		HDMI_CORE_SYS_REG(0x208)
#define HDMI_CORE_SYS_DE_DLY			HDMI_CORE_SYS_REG(0xC8)
#define HDMI_CORE_SYS_DE_CTRL			HDMI_CORE_SYS_REG(0xCC)
#define HDMI_CORE_SYS_DE_TOP			HDMI_CORE_SYS_REG(0xD0)
#define HDMI_CORE_SYS_DE_CNTL			HDMI_CORE_SYS_REG(0xD8)
#define HDMI_CORE_SYS_DE_CNTH			HDMI_CORE_SYS_REG(0xDC)
#define HDMI_CORE_SYS_DE_LINL			HDMI_CORE_SYS_REG(0xE0)
#define HDMI_CORE_SYS_DE_LINH_1		HDMI_CORE_SYS_REG(0xE4)
#define HDMI_CORE_AKSV(n)			HDMI_CORE_SYS_REG(n*4 + 0x74)
#define HDMI_CORE_CTRL1_VEN_FOLLOWVSYNC	0x1
#define HDMI_CORE_CTRL1_HEN_FOLLOWHSYNC	0x1
#define HDMI_CORE_CTRL1_BSEL_24BITBUS		0x1
#define HDMI_CORE_CTRL1_EDGE_RISINGEDGE	0x1
#define HDMI_CORE_CTRL1_POWER_DOWN     0x1
#define HDMI_CORE_SYSTEM_INTR2__BCAP	0x80
#define HDMI_CORE_SYSTEM_INTR3__RI_ERR	0xF0
#define HDMI_CORE_SYSTEM_INTR4_CEC	0x08

/* HDMI DDC E-DID */
#define HDMI_CORE_DDC_CMD			HDMI_CORE_SYS_REG(0x3CC)
#define HDMI_CORE_DDC_STATUS			HDMI_CORE_SYS_REG(0x3C8)
#define HDMI_CORE_DDC_ADDR			HDMI_CORE_SYS_REG(0x3B4)
#define HDMI_CORE_DDC_OFFSET			HDMI_CORE_SYS_REG(0x3BC)
#define HDMI_CORE_DDC_COUNT1			HDMI_CORE_SYS_REG(0x3C0)
#define HDMI_CORE_DDC_COUNT2			HDMI_CORE_SYS_REG(0x3C4)
#define HDMI_CORE_DDC_DATA			HDMI_CORE_SYS_REG(0x3D0)
#define HDMI_CORE_DDC_SEGM			HDMI_CORE_SYS_REG(0x3B8)

/* HDMI IP Core Audio Video */
#define HDMI_CORE_AV_REG(idx)			HDMI_REG(idx)

#define HDMI_CORE_AV_HDMI_CTRL			HDMI_CORE_AV_REG(0xBC)
#define HDMI_CORE_AV_DPD			HDMI_CORE_AV_REG(0xF4)
#define HDMI_CORE_AV_PB_CTRL1			HDMI_CORE_AV_REG(0xF8)
#define HDMI_CORE_AV_PB_CTRL2			HDMI_CORE_AV_REG(0xFC)
#define HDMI_CORE_AV_AVI_TYPE			HDMI_CORE_AV_REG(0x100)
#define HDMI_CORE_AV_AVI_VERS			HDMI_CORE_AV_REG(0x104)
#define HDMI_CORE_AV_AVI_LEN			HDMI_CORE_AV_REG(0x108)
#define HDMI_CORE_AV_AVI_CHSUM			HDMI_CORE_AV_REG(0x10C)
#define HDMI_CORE_AV_AVI_DBYTE(n)		HDMI_CORE_AV_REG(n * 4 + 0x110)
#define HDMI_CORE_AV_AVI_DBYTE_NELEMS		HDMI_CORE_AV_REG(15)
#define HDMI_CORE_AV_SPD_DBYTE			HDMI_CORE_AV_REG(0x190)
#define HDMI_CORE_AV_SPD_DBYTE_NELEMS		HDMI_CORE_AV_REG(27)
#define HDMI_CORE_AV_AUD_DBYTE(n)		HDMI_CORE_AV_REG(n * 4 + 0x210)
#define HDMI_CORE_AV_AUD_DBYTE_NELEMS		HDMI_CORE_AV_REG(10)
#define HDMI_CORE_AV_MPEG_DBYTE		HDMI_CORE_AV_REG(0x290)
#define HDMI_CORE_AV_MPEG_DBYTE_NELEMS		HDMI_CORE_AV_REG(27)
#define HDMI_CORE_AV_GEN_DBYTE(n)		HDMI_CORE_AV_REG(n * 4 + 0x300)
#define HDMI_CORE_AV_GEN_DBYTE_NELEMS		HDMI_CORE_AV_REG(31)
#define HDMI_CORE_AV_GEN2_DBYTE		HDMI_CORE_AV_REG(0x380)
#define HDMI_CORE_AV_GEN2_DBYTE_NELEMS		HDMI_CORE_AV_REG(31)
#define HDMI_CORE_AV_ACR_CTRL			HDMI_CORE_AV_REG(0x4)
#define HDMI_CORE_AV_FREQ_SVAL			HDMI_CORE_AV_REG(0x8)
#define HDMI_CORE_AV_N_SVAL1			HDMI_CORE_AV_REG(0xC)
#define HDMI_CORE_AV_N_SVAL2			HDMI_CORE_AV_REG(0x10)
#define HDMI_CORE_AV_N_SVAL3			HDMI_CORE_AV_REG(0x14)
#define HDMI_CORE_AV_CTS_SVAL1			HDMI_CORE_AV_REG(0x18)
#define HDMI_CORE_AV_CTS_SVAL2			HDMI_CORE_AV_REG(0x1C)
#define HDMI_CORE_AV_CTS_SVAL3			HDMI_CORE_AV_REG(0x20)
#define HDMI_CORE_AV_CTS_HVAL1			HDMI_CORE_AV_REG(0x24)
#define HDMI_CORE_AV_CTS_HVAL2			HDMI_CORE_AV_REG(0x28)
#define HDMI_CORE_AV_CTS_HVAL3			HDMI_CORE_AV_REG(0x2C)
#define HDMI_CORE_AV_AUD_MODE			HDMI_CORE_AV_REG(0x50)
#define HDMI_CORE_AV_SPDIF_CTRL		HDMI_CORE_AV_REG(0x54)
#define HDMI_CORE_AV_HW_SPDIF_FS		HDMI_CORE_AV_REG(0x60)
#define HDMI_CORE_AV_SWAP_I2S			HDMI_CORE_AV_REG(0x64)
#define HDMI_CORE_AV_SPDIF_ERTH		HDMI_CORE_AV_REG(0x6C)
#define HDMI_CORE_AV_I2S_IN_MAP		HDMI_CORE_AV_REG(0x70)
#define HDMI_CORE_AV_I2S_IN_CTRL		HDMI_CORE_AV_REG(0x74)
#define HDMI_CORE_AV_I2S_CHST0			HDMI_CORE_AV_REG(0x78)
#define HDMI_CORE_AV_I2S_CHST1			HDMI_CORE_AV_REG(0x7C)
#define HDMI_CORE_AV_I2S_CHST2			HDMI_CORE_AV_REG(0x80)
#define HDMI_CORE_AV_I2S_CHST4			HDMI_CORE_AV_REG(0x84)
#define HDMI_CORE_AV_I2S_CHST5			HDMI_CORE_AV_REG(0x88)
#define HDMI_CORE_AV_ASRC			HDMI_CORE_AV_REG(0x8C)
#define HDMI_CORE_AV_I2S_IN_LEN		HDMI_CORE_AV_REG(0x90)
#define HDMI_CORE_AV_HDMI_CTRL			HDMI_CORE_AV_REG(0xBC)
#define HDMI_CORE_AV_AUDO_TXSTAT		HDMI_CORE_AV_REG(0xC0)
#define HDMI_CORE_AV_AUD_PAR_BUSCLK_1		HDMI_CORE_AV_REG(0xCC)
#define HDMI_CORE_AV_AUD_PAR_BUSCLK_2		HDMI_CORE_AV_REG(0xD0)
#define HDMI_CORE_AV_AUD_PAR_BUSCLK_3		HDMI_CORE_AV_REG(0xD4)
#define HDMI_CORE_AV_TEST_TXCTRL		HDMI_CORE_AV_REG(0xF0)
#define HDMI_CORE_AV_DPD			HDMI_CORE_AV_REG(0xF4)
#define HDMI_CORE_AV_PB_CTRL1			HDMI_CORE_AV_REG(0xF8)
#define HDMI_CORE_AV_PB_CTRL2			HDMI_CORE_AV_REG(0xFC)
#define HDMI_CORE_AV_AVI_TYPE			HDMI_CORE_AV_REG(0x100)
#define HDMI_CORE_AV_AVI_VERS			HDMI_CORE_AV_REG(0x104)
#define HDMI_CORE_AV_AVI_LEN			HDMI_CORE_AV_REG(0x108)
#define HDMI_CORE_AV_AVI_CHSUM			HDMI_CORE_AV_REG(0x10C)
#define HDMI_CORE_AV_SPD_TYPE			HDMI_CORE_AV_REG(0x180)
#define HDMI_CORE_AV_SPD_VERS			HDMI_CORE_AV_REG(0x184)
#define HDMI_CORE_AV_SPD_LEN			HDMI_CORE_AV_REG(0x188)
#define HDMI_CORE_AV_SPD_CHSUM			HDMI_CORE_AV_REG(0x18C)
#define HDMI_CORE_AV_AUDIO_TYPE		HDMI_CORE_AV_REG(0x200)
#define HDMI_CORE_AV_AUDIO_VERS		HDMI_CORE_AV_REG(0x204)
#define HDMI_CORE_AV_AUDIO_LEN			HDMI_CORE_AV_REG(0x208)
#define HDMI_CORE_AV_AUDIO_CHSUM		HDMI_CORE_AV_REG(0x20C)
#define HDMI_CORE_AV_MPEG_TYPE			HDMI_CORE_AV_REG(0x280)
#define HDMI_CORE_AV_MPEG_VERS			HDMI_CORE_AV_REG(0x284)
#define HDMI_CORE_AV_MPEG_LEN			HDMI_CORE_AV_REG(0x288)
#define HDMI_CORE_AV_MPEG_CHSUM		HDMI_CORE_AV_REG(0x28C)
#define HDMI_CORE_AV_CP_BYTE1			HDMI_CORE_AV_REG(0x37C)
#define HDMI_CORE_AV_CEC_ADDR_ID		HDMI_CORE_AV_REG(0x3FC)
#define HDMI_CORE_AV_SPD_DBYTE_ELSIZE		0x4
#define HDMI_CORE_AV_GEN2_DBYTE_ELSIZE		0x4
#define HDMI_CORE_AV_MPEG_DBYTE_ELSIZE		0x4
#define HDMI_CORE_AV_GEN_DBYTE_ELSIZE		0x4

/* PLL */
#define HDMI_PLL_REG(idx)			HDMI_REG(idx)

#define PLLCTRL_PLL_CONTROL			HDMI_PLL_REG(0x0)
#define PLLCTRL_PLL_STATUS			HDMI_PLL_REG(0x4)
#define PLLCTRL_PLL_GO				HDMI_PLL_REG(0x8)
#define PLLCTRL_CFG1				HDMI_PLL_REG(0xC)
#define PLLCTRL_CFG2				HDMI_PLL_REG(0x10)
#define PLLCTRL_CFG3				HDMI_PLL_REG(0x14)
#define PLLCTRL_CFG4				HDMI_PLL_REG(0x20)

/* HDMI PHY */
#define HDMI_PHY_REG(idx)			HDMI_REG(idx)

#define HDMI_TXPHY_TX_CTRL			HDMI_PHY_REG(0x0)
#define HDMI_TXPHY_DIGITAL_CTRL		HDMI_PHY_REG(0x4)
#define HDMI_TXPHY_POWER_CTRL			HDMI_PHY_REG(0x8)
#define HDMI_TXPHY_PAD_CFG_CTRL		HDMI_PHY_REG(0xC)


/**************************************
* HDMI CEC registers *
***************************************/
#define HDMI_CEC_REG(idx)			HDMI_REG(idx)

#define HDMI_CEC_DEV_ID                         HDMI_CEC_REG(0x0)
#define HDMI_CEC_SPEC                           HDMI_CEC_REG(0x4)
#define HDMI_CEC_TX_INIT                        HDMI_CEC_REG(0x20)
#define HDMI_CEC_TX_DEST                        HDMI_CEC_REG(0x24)
#define HDMI_CEC_TRANSMIT_DATA                  HDMI_CEC_REG(0x7c)
#define HDMI_CEC_SETUP                          HDMI_CEC_REG(0x38)
#define HDMI_CEC_TX_COMMAND                     HDMI_CEC_REG(0x3c)
#define HDMI_CEC_DBG_3                          HDMI_CEC_REG(0x1c)
#define HDMI_CEC_INT_STATUS_1                   HDMI_CEC_REG(0x9c)
#define HDMI_CEC_INT_STATUS_0                   HDMI_CEC_REG(0x98)
#define HDMI_CEC_INT_ENABLE_0                   HDMI_CEC_REG(0x90)
#define HDMI_CEC_RX_CMD_HEADER                  HDMI_CEC_REG(0xb8)
#define HDMI_CEC_RX_COUNT                       HDMI_CEC_REG(0xB4)
#define HDMI_CEC_RX_CONTROL                     HDMI_CEC_REG(0xB0)
#define HDMI_CEC_RX_COMMAND                     HDMI_CEC_REG(0xbc)
#define HDMI_CEC_RX_OPERAND                     (0xc0)
#define HDMI_CEC_TX_OPERAND                     (0x40)
#define HDMI_CEC_CA_7_0                         (0x88)
#define HDMI_CEC_CA_15_8                        (0x8c)

#define HDMI_CEC_TX_FIFO_INT_MASK		(0x64)
#define HDMI_CEC_RETRANSMIT_CNT_INT_MASK	(0x2)

#define FLD_MASK(start, end)	(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))
#define REG_FLD_MOD(base, idx, val, start, end) \
	hdmi_write_reg(base, idx, FLD_MOD(hdmi_read_reg(base, idx),\
							val, start, end))
#define REG_GET(base, idx, start, end) \
	FLD_GET(hdmi_read_reg(base, idx), start, end)

#define WR_REG_32(base, offset, val)	__raw_writel(val, base + offset)
#define RD_REG_32(base, offset)		__raw_readl(base + offset)

enum hdmi_phy_pwr {
	HDMI_PHYPWRCMD_OFF = 0,
	HDMI_PHYPWRCMD_LDOON = 1,
	HDMI_PHYPWRCMD_TXON = 2
};

enum hdmi_clk_refsel {
	HDMI_REFSEL_PCLK = 0,
	HDMI_REFSEL_REF1 = 1,
	HDMI_REFSEL_REF2 = 2,
	HDMI_REFSEL_SYSCLK = 3
};

enum hdmi_core_inputbus_width {
	HDMI_INPUT_8BIT = 0,
	HDMI_INPUT_10BIT = 1,
	HDMI_INPUT_12BIT = 2
};

enum hdmi_core_dither_trunc {
	HDMI_OUTPUTTRUNCATION_8BIT = 0,
	HDMI_OUTPUTTRUNCATION_10BIT = 1,
	HDMI_OUTPUTTRUNCATION_12BIT = 2,
	HDMI_OUTPUTDITHER_8BIT = 3,
	HDMI_OUTPUTDITHER_10BIT = 4,
	HDMI_OUTPUTDITHER_12BIT = 5
};

enum hdmi_core_deepcolor_ed {
	HDMI_DEEPCOLORPACKECTDISABLE = 0,
	HDMI_DEEPCOLORPACKECTENABLE = 1
};

enum hdmi_core_packet_mode {
	HDMI_PACKETMODERESERVEDVALUE = 0,
	HDMI_PACKETMODE24BITPERPIXEL = 4,
	HDMI_PACKETMODE30BITPERPIXEL = 5,
	HDMI_PACKETMODE36BITPERPIXEL = 6,
	HDMI_PACKETMODE48BITPERPIXEL = 7
};

enum hdmi_core_tclkselclkmult {
	HDMI_FPLL05IDCK = 0,
	HDMI_FPLL10IDCK = 1,
	HDMI_FPLL20IDCK = 2,
	HDMI_FPLL40IDCK = 3
};

enum hdmi_core_packet_ctrl {
	HDMI_PACKETENABLE = 1,
	HDMI_PACKETDISABLE = 0,
	HDMI_PACKETREPEATON = 1,
	HDMI_PACKETREPEATOFF = 0
};

enum hdmi_packing_mode {
	HDMI_PACK_10b_RGB_YUV444 = 0,
	HDMI_PACK_24b_RGB_YUV444_YUV422 = 1,
	HDMI_PACK_20b_YUV422 = 2,
	HDMI_PACK_ALREADYPACKED = 7
};


struct hdmi_core_video_config {
	enum hdmi_core_inputbus_width	ip_bus_width;
	enum hdmi_core_dither_trunc	op_dither_truc;
	enum hdmi_core_deepcolor_ed	deep_color_pkt;
	enum hdmi_core_packet_mode	pkt_mode;
	enum hdmi_core_hdmi_dvi		hdmi_dvi;
	enum hdmi_core_tclkselclkmult	tclk_sel_clkmult;
};

/*
 * Refer to section 8.2 in HDMI 1.3 specification for
 * details about infoframe databytes
 */
struct hdmi_core_infoframe_avi {
	u8	db1_format;
		/* Y0, Y1 rgb,yCbCr */
	u8	db1_active_info;
		/* A0  Active information Present */
	u8	db1_bar_info_dv;
		/* B0, B1 Bar info data valid */
	u8	db1_scan_info;
		/* S0, S1 scan information */
	u8	db2_colorimetry;
		/* C0, C1 colorimetry */
	u8	db2_aspect_ratio;
		/* M0, M1 Aspect ratio (4:3, 16:9) */
	u8	db2_active_fmt_ar;
		/* R0...R3 Active format aspect ratio */
	u8	db3_itc;
		/* ITC IT content. */
	u8	db3_ec;
		/* EC0, EC1, EC2 Extended colorimetry */
	u8	db3_q_range;
		/* Q1, Q0 Quantization range */
	u8	db3_nup_scaling;
		/* SC1, SC0 Non-uniform picture scaling */
	u8	db4_videocode;
		/* VIC0..6 Video format identification */
	u8	db5_pixel_repeat;
		/* PR0..PR3 Pixel repetition factor */
	u16	db6_7_line_eoftop;
		/* Line number end of top bar */
	u16	db8_9_line_sofbottom;
		/* Line number start of bottom bar */
	u16	db10_11_pixel_eofleft;
		/* Pixel number end of left bar */
	u16	db12_13_pixel_sofright;
		/* Pixel number start of right bar */
};

struct hdmi_core_packet_enable_repeat {
	u32	audio_pkt;
	u32	audio_pkt_repeat;
	u32	avi_infoframe;
	u32	avi_infoframe_repeat;
	u32	gen_cntrl_pkt;
	u32	gen_cntrl_pkt_repeat;
	u32	generic_pkt;
	u32	generic_pkt_repeat;
};

struct hdmi_video_format {
	enum hdmi_packing_mode	packing_mode;
	u32			y_res;	/* Line per panel */
	u32			x_res;	/* pixel per line */
};

struct hdmi_video_interface {
	int	vsp;	/* Vsync polarity */
	int	hsp;	/* Hsync polarity */
	int	interlacing;
	int	tm;	/* Timing mode */
};

int hdmi_lib_start_acr_wa(void);
void hdmi_lib_stop_acr_wa(void);
#endif
