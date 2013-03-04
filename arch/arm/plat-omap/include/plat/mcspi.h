#ifndef _OMAP2_MCSPI_H
#define _OMAP2_MCSPI_H

#define OMAP2_MCSPI_MASTER		0
#define OMAP2_MCSPI_SLAVE		1

#define OMAP2_MCSPI_REV 0
#define OMAP3_MCSPI_REV 1
#define OMAP4_MCSPI_REV 2

#define OMAP4_MCSPI_REG_OFFSET 0x100

struct omap2_mcspi_platform_config {
	unsigned short	num_cs;
	unsigned short	mode;
	unsigned short	dma_mode;
	unsigned short	force_cs_mode;
	unsigned short	fifo_depth;
	unsigned int	regs_offset;
	unsigned int	*regs_data;
};

struct omap2_mcspi_dev_attr {
	unsigned short num_chipselect;
};

struct omap2_mcspi_device_config {
	unsigned turbo_mode:1;

	/* Do we want one channel enabled at the same time? */
	unsigned single_channel:1;
	/* Swap data lines */
	unsigned swap_datalines;
};

enum {
	OMAP2_MCSPI_REVISION = 0,
	OMAP2_MCSPI_SYSCONFIG,
	OMAP2_MCSPI_SYSSTATUS,
	OMAP2_MCSPI_IRQSTATUS,
	OMAP2_MCSPI_IRQENABLE,
	OMAP2_MCSPI_WAKEUPENABLE,
	OMAP2_MCSPI_SYST,
	OMAP2_MCSPI_MODULCTRL,
	OMAP2_MCSPI_XFERLEVEL,
	OMAP2_MCSPI_CHCONF0,
	OMAP2_MCSPI_CHSTAT0,
	OMAP2_MCSPI_CHCTRL0,
	OMAP2_MCSPI_TX0,
	OMAP2_MCSPI_RX0,
	OMAP2_MCSPI_HL_REV,
	OMAP2_MCSPI_HL_HWINFO,
	OMAP2_MCSPI_HL_SYSCONFIG,
};

static const u16 omap2_reg_map[] = {
	[OMAP2_MCSPI_REVISION]		= 0x00,
	[OMAP2_MCSPI_SYSCONFIG]		= 0x10,
	[OMAP2_MCSPI_SYSSTATUS]		= 0x14,
	[OMAP2_MCSPI_IRQSTATUS]		= 0x18,
	[OMAP2_MCSPI_IRQENABLE]		= 0x1c,
	[OMAP2_MCSPI_WAKEUPENABLE]	= 0x20,
	[OMAP2_MCSPI_SYST]		= 0x24,
	[OMAP2_MCSPI_MODULCTRL]		= 0x28,
	[OMAP2_MCSPI_XFERLEVEL]		= 0x7c,
	[OMAP2_MCSPI_CHCONF0]		= 0x2c,
	[OMAP2_MCSPI_CHSTAT0]		= 0x30,
	[OMAP2_MCSPI_CHCTRL0]		= 0x34,
	[OMAP2_MCSPI_TX0]		= 0x38,
	[OMAP2_MCSPI_RX0]		= 0x3c,
};

static const u16 omap4_reg_map[] = {
	[OMAP2_MCSPI_REVISION]		= 0x100,
	[OMAP2_MCSPI_SYSCONFIG]		= 0x110,
	[OMAP2_MCSPI_SYSSTATUS]		= 0x114,
	[OMAP2_MCSPI_IRQSTATUS]		= 0x118,
	[OMAP2_MCSPI_IRQENABLE]		= 0x01c,
	[OMAP2_MCSPI_WAKEUPENABLE]	= 0x120,
	[OMAP2_MCSPI_SYST]		= 0x124,
	[OMAP2_MCSPI_MODULCTRL]		= 0x128,
	[OMAP2_MCSPI_XFERLEVEL]		= 0x17c,
	[OMAP2_MCSPI_CHCONF0]		= 0x12c,
	[OMAP2_MCSPI_CHSTAT0]		= 0x130,
	[OMAP2_MCSPI_CHCTRL0]		= 0x134,
	[OMAP2_MCSPI_TX0]		= 0x138,
	[OMAP2_MCSPI_RX0]		= 0x13c,
	[OMAP2_MCSPI_HL_REV]		= 0x000,
	[OMAP2_MCSPI_HL_HWINFO]		= 0x004,
	[OMAP2_MCSPI_HL_SYSCONFIG]	= 0x010,
};


#endif
