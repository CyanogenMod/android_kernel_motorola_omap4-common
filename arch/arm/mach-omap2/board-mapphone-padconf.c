/*
 * board-mapphone-padconf.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>

#include <plat/hardware.h>
#include "control.h"
#include "dt_path.h"
#include "board-mapphone-padconf.h"
#include "mux.h"

#define IOMUX_MODE_MASK                0x00000007

void __init mapphone_omap44xx_padconf_init(void)
{
	struct device_node *node;
	const struct padconf_core_entry *core;
	const struct padconf_wkup_entry *wkup;
	int i, size;

	node = of_find_node_by_path(DT_PATH_MUX);
	if (node == NULL) {
		printk(KERN_ERR "Unable to read node %s from device tree!\n",
		       DT_PATH_MUX);
		return;
	}

	core = (const void *)of_get_property(node, DT_PROP_MUX_PAD, &size);
	if ((!core) || ((size % sizeof(struct padconf_core_entry)) != 0)) {
		printk(KERN_ERR "Read property %s error!\n", DT_PROP_MUX_PAD);
	} else {
		for (i = 0; i < size / sizeof(struct padconf_core_entry); i++) {
			if ((core->offset < OMAP44XX_CORE_PADCONF_OFFSET_BEGIN)
					|| (core->offset > OMAP44XX_CORE_PADCONF_OFFSET_END)) {
				printk(KERN_ERR "Wakeup padconf setting out of "
						"range: 0x%04X", core->offset);
				continue;
			}

			omap_writew(OMAP44XX_CORE_PADCONF_SETTING(core->mode,
						core->input_en,
						core->pull_type,
						core->offmode_en,
						core->offout_type,
						core->offpull_type,
						core->offwkup_en),
					OMAP443X_CTRL_BASE + core->offset);
			core++;
		}
	}

	wkup = (const void *)of_get_property(node, DT_PROP_MUX_PADWKUPS, &size);
	if ((!wkup) || ((size % sizeof(struct padconf_wkup_entry)) != 0)) {
		printk(KERN_ERR "Read property %s error!\n",
				DT_PROP_MUX_PADWKUPS);
	} else {
		for (i = 0; i < size / sizeof(struct padconf_wkup_entry); i++) {
			if ((wkup->offset < OMAP44XX_WKUP_PADCONF_OFFSET_BEGIN)
					|| (wkup->offset > OMAP44XX_WKUP_PADCONF_OFFSET_END)) {
				printk(KERN_ERR "Wakeup padconf setting out of "
						"range: 0x%04X", wkup->offset);
				continue;
			}

			omap_writew(OMAP44XX_WKUP_PADCONF_SETTING(wkup->mode,
						wkup->input_en,
						wkup->pull_type,
						wkup->offwkup_en),
					OMAP44XX_WKUP_PADCONF_BASE
					+ wkup->offset);
			wkup++;
		}
	}

	of_node_put(node);

	return;
}

static void padconf_mux_write(unsigned short offset, unsigned short mode)
{
	unsigned short align_offset = offset & ~0x3;
	unsigned int value;
	unsigned short reg_high, reg_low;
	unsigned short mask = 0xFFFF;

	value = omap4_ctrl_pad_readl(align_offset);
	reg_high = value >> 16;
	reg_low = value & mask;
	if (offset & 0x2) {
		reg_high &= ~IOMUX_MODE_MASK;
		reg_high |=  mode;
	} else {
		reg_low &= ~IOMUX_MODE_MASK;
		reg_low |=  mode;
	}
	value = (reg_high << 16) | reg_low;
	omap4_ctrl_pad_writel(value, align_offset);
}

void mcbsp3_i2s1_pin_mux_switch(unsigned short incall)
{
	unsigned short abe_pdm_ul_data_offset =
		OMAP4_CTRL_MODULE_PAD_ABE_PDM_UL_DATA_OFFSET;    /* 0x106 */
	unsigned short abe_pdm_dl_data_offset =
		OMAP4_CTRL_MODULE_PAD_ABE_PDM_DL_DATA_OFFSET;    /* 0x108 */
	unsigned short usbb1_ulpitll_dat4_offset =
		OMAP4_CTRL_MODULE_PAD_USBB1_ULPITLL_DAT4_OFFSET;  /* 0xD2 */
	unsigned short usbb1_ulpitll_dat5_offset =
		OMAP4_CTRL_MODULE_PAD_USBB1_ULPITLL_DAT5_OFFSET;  /* 0xD4 */

	if (incall) {
		padconf_mux_write(abe_pdm_ul_data_offset, OMAP_MUX_MODE7);
		padconf_mux_write(abe_pdm_dl_data_offset, OMAP_MUX_MODE7);
		padconf_mux_write(usbb1_ulpitll_dat4_offset, OMAP_MUX_MODE2);
		padconf_mux_write(usbb1_ulpitll_dat5_offset, OMAP_MUX_MODE2);
	} else {
		padconf_mux_write(abe_pdm_ul_data_offset, OMAP_MUX_MODE1);
		padconf_mux_write(abe_pdm_dl_data_offset, OMAP_MUX_MODE1);
		padconf_mux_write(usbb1_ulpitll_dat4_offset, OMAP_MUX_MODE7);
		padconf_mux_write(usbb1_ulpitll_dat5_offset, OMAP_MUX_MODE7);
	}
}
