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

#endif
