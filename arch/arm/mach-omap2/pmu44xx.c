/*
 * arch/arm/mach-omap2/pmu44xx.c
 *
 * Copyright (C) 2011 Motorola Mobility, Inc.
 *
 * Author: Hongmei Li <a21834@motorola.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <plat/io.h>

#define OMAP4_CTI0_BASE (L4_EMU_44XX_BASE + 0x148000)
#define OMAP4_CTI1_BASE (L4_EMU_44XX_BASE + 0x149000)
#define OMAP4_CTI_CTRL_OFFSET 0x000
#define OMAP4_CTI_INTACK_OFFSET 0x010
#define OMAP4_CTI_LOCK_OFFSET 0xFB0
#define OMAP4_CTI_TRIGIN1_OFFSET 0x024
#define OMAP4_CTI_TRIGOUT6_OFFSET 0x0B8

#define OMAP4_CM_L3INSTR_L3_3_CLKCTRL 0x4A008E20
#define OMAP4_CM_L3INSTR_L3_INSTR_CLKCTRL 0x4A008E28
#define OMAP4_CM_EMU_CLKSTCTRL 0x4a307a00

static unsigned long CTI[2] = {OMAP4_CTI0_BASE, OMAP4_CTI1_BASE};

void pmu_ack_cti_int(void)
{
	unsigned int cpu = smp_processor_id();
	__raw_writel(1 << 6,  OMAP2_EMU_IO_ADDRESS(CTI[cpu]
				+ OMAP4_CTI_INTACK_OFFSET));
}

void pmu_l3clk_init(void)
{
	omap_writel(1, OMAP4_CM_L3INSTR_L3_3_CLKCTRL);
	omap_writel(1, OMAP4_CM_L3INSTR_L3_INSTR_CLKCTRL);
	omap_writel(2, OMAP4_CM_EMU_CLKSTCTRL);
	while ((omap_readl(OMAP4_CM_EMU_CLKSTCTRL) & 0x300) != 0x300)
		;
}

void pmu_cti_init(int cpu)
{
	unsigned int lock_pattern = 0xC5ACCE55;

	/* unlock the module so application writes can go through */
	__raw_writel(lock_pattern, OMAP2_EMU_IO_ADDRESS(CTI[cpu]
					+ OMAP4_CTI_LOCK_OFFSET));
	/* Enable TRIGIN[1] to go to channel 2 */
	__raw_writel(1 << (2 + cpu),  OMAP2_EMU_IO_ADDRESS(CTI[cpu]
					+ OMAP4_CTI_TRIGIN1_OFFSET));
	/* Enable channel 2 to go to TRIGOUT[6] */
	__raw_writel(1 << (2 + cpu),  OMAP2_EMU_IO_ADDRESS(CTI[cpu]
					+ OMAP4_CTI_TRIGOUT6_OFFSET));
	/* Enable CTI0 module */
	__raw_writel(0x1, OMAP2_EMU_IO_ADDRESS(CTI[cpu]
				+ OMAP4_CTI_CTRL_OFFSET));
	/* barrier */
	__raw_readl(OMAP2_EMU_IO_ADDRESS(CTI[cpu] + OMAP4_CTI_CTRL_OFFSET));
}
