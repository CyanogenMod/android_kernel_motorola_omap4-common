/*
 * Toshiba TC358765 bridge chip panel support
 *
 * Copyright (C) Texas Instruments  Corporation
 * Author: Jerry Alexander <x0135174@ti.com>
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <video/omapdss.h>
#include <video/omap-panel-mapphone-dsi.h>
#include "../dss/dss.h"

#include "panel-mapphone-d2l.h"

#define DRIVER_NAME "d2l_i2c_drv"
#define DEVICE_NAME "d2l_i2c"

static u8 dsi_vc_cmd;
static u8 dsi_vc_video;

struct d2l_i2c {
	struct i2c_client *client;
	struct mutex xfer_lock;
} *sd1;

/*************************************
**** Interface utility functions *****
*************************************/
static int d2l_read_block(int reg, u8 *data, int len)
{
	unsigned char wb[2];
	struct i2c_msg msg[2];
	int r;
	mutex_lock(&sd1->xfer_lock);
	wb[0] = (reg & 0xff00) >> 8;
	wb[1] = reg & 0xff;
	msg[0].addr = sd1->client->addr;
	msg[0].len = 2;
	msg[0].flags = 0;
	msg[0].buf = wb;
	msg[1].addr = sd1->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	r = i2c_transfer(sd1->client->adapter, msg, 2);
	mutex_unlock(&sd1->xfer_lock);

	if (r == 2)
		return len;

	return r;
}

static int d2l_i2c_read(int reg)
{
	int r;
	u8 data[4];
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;

	r = d2l_read_block(reg, data, 4);
	return ((int)data[3] << 24) | ((int)(data[2]) << 16) |
	    ((int)(data[1]) << 8) | ((int)(data[0]));
}

int d2l_write_register(struct omap_dss_device *dssdev, u16 reg, u32 value)
{

	int ret;
	DSI_long_write_packet dsi_buf;

	dsi_buf.data_type = 0x29;	/* generic long write */
	dsi_buf.ECC = 0;	/* no error correcting code */
	dsi_buf.WC = 6;		/* pay load byte count */

	/* Configure register and value */
	dsi_buf.Data_buf[0] = (reg >> 0) & 0xFF;
	dsi_buf.Data_buf[1] = (reg >> 8) & 0xFF;
	dsi_buf.Data_buf[2] = (value >> 0) & 0xFF;
	dsi_buf.Data_buf[3] = (value >> 8) & 0xFF;
	dsi_buf.Data_buf[4] = (value >> 16) & 0xFF;
	dsi_buf.Data_buf[5] = (value >> 24) & 0xFF;

	/* Send long packet to DSI */
	ret = dsi_vc_gen_write_nosync(dssdev, dsi_vc_cmd,
				(u8 *) dsi_buf.Data_buf, 6);
	udelay(10);

	return ret;
}

/****************************
********* DEBUG *************
****************************/
void d2l_dump_regs(void)
{
#define DUMPREG(r) printk(KERN_DEBUG "%-35s : %08x\n", #r, d2l_i2c_read(r))

	printk(KERN_DEBUG "I2C Toshiba Registers\n");

	printk(KERN_DEBUG "DSI D-PHY Layer Registers\n");
	DUMPREG(D0W_DPHYCONTTX);
	DUMPREG(CLW_DPHYCONTRX);
	DUMPREG(D0W_DPHYCONTRX);
	DUMPREG(D1W_DPHYCONTRX);
	DUMPREG(D2W_DPHYCONTRX);
	DUMPREG(D3W_DPHYCONTRX);
	DUMPREG(COM_DPHYCONTRX);
	DUMPREG(CLW_CNTRL);
	DUMPREG(D0W_CNTRL);
	DUMPREG(D1W_CNTRL);
	DUMPREG(D2W_CNTRL);
	DUMPREG(D3W_CNTRL);
	DUMPREG(DFTMODE_CNTRL);

	printk(KERN_DEBUG "DSI PPI Layer Registers\n");
	DUMPREG(PPI_STARTPPI);
	DUMPREG(PPI_BUSYPPI);
	DUMPREG(PPI_LINEINITCNT);
	DUMPREG(PPI_LPTXTIMECNT);
	DUMPREG(PPI_LANEENABLE);
	DUMPREG(PPI_TX_RX_TA);
	DUMPREG(PPI_CLS_ATMR);
	DUMPREG(PPI_D0S_ATMR);
	DUMPREG(PPI_D1S_ATMR);
	DUMPREG(PPI_D2S_ATMR);
	DUMPREG(PPI_D3S_ATMR);
	DUMPREG(PPI_D0S_CLRSIPOCOUNT);
	DUMPREG(PPI_D1S_CLRSIPOCOUNT);
	DUMPREG(PPI_D2S_CLRSIPOCOUNT);
	DUMPREG(PPI_D3S_CLRSIPOCOUNT);
	DUMPREG(CLS_PRE);
	DUMPREG(D0S_PRE);
	DUMPREG(D1S_PRE);
	DUMPREG(D2S_PRE);
	DUMPREG(D3S_PRE);
	DUMPREG(CLS_PREP);
	DUMPREG(D0S_PREP);
	DUMPREG(D1S_PREP);
	DUMPREG(D2S_PREP);
	DUMPREG(D3S_PREP);
	DUMPREG(CLS_ZERO);
	DUMPREG(D0S_ZERO);
	DUMPREG(D1S_ZERO);
	DUMPREG(D2S_ZERO);
	DUMPREG(D3S_ZERO);
	DUMPREG(PPI_CLRFLG);
	DUMPREG(PPI_CLRSIPO);
	DUMPREG(PPI_HSTimeout);
	DUMPREG(PPI_HSTimeoutEnable);

	printk(KERN_DEBUG "DSI Protocol Layer Registers\n");
	DUMPREG(DSI_BUSYDSI);
	DUMPREG(DSI_LANEENABLE);
	DUMPREG(DSI_LANESTATUS0);
	DUMPREG(DSI_LANESTATUS1);
	DUMPREG(DSI_INTSTATUS);
	DUMPREG(DSI_INTMASK);
	DUMPREG(DSI_INTCLR);
	DUMPREG(DSI_LPTXTO);;

	printk(KERN_DEBUG "DSI General Registers\n");
	DUMPREG(DSIERRCNT);

	printk(KERN_DEBUG "DSI Application Layer Registers\n");
	DUMPREG(APLCTRL);
	DUMPREG(RDPKTLN);

	printk(KERN_DEBUG "Video Path Registers\n");
	DUMPREG(VPCTRL);
	DUMPREG(HTIM1);
	DUMPREG(HTIM2);
	DUMPREG(VTIM1);
	DUMPREG(VTIM2);
	DUMPREG(VFUEN);

	printk(KERN_DEBUG "LVDS Registers\n");
	DUMPREG(LVMX0003);
	DUMPREG(LVMX0407);
	DUMPREG(LVMX0811);
	DUMPREG(LVMX1215);
	DUMPREG(LVMX1619);
	DUMPREG(LVMX2023);
	DUMPREG(LVMX2427);
	DUMPREG(LVCFG);
	DUMPREG(LVPHY0);
	DUMPREG(LVPHY1);

	printk(KERN_DEBUG "System Registers\n");
	DUMPREG(SYSSTAT);
	DUMPREG(SYSRST);

	printk(KERN_DEBUG "GPIO Registers\n");
	DUMPREG(GPIOC);
	DUMPREG(GPIOO);
	DUMPREG(GPIOI);

	printk(KERN_DEBUG "Chip Revision Registers\n");
	DUMPREG(IDREG);

	printk(KERN_DEBUG "Debug Registers\n");
	DUMPREG(DEBUG00);
	DUMPREG(DEBUG01);
#undef DUMPREG
}
EXPORT_SYMBOL(d2l_dump_regs);

static ssize_t d2l_regs_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	d2l_dump_regs();
	return 0;
}

static DEVICE_ATTR(d2l_regs, S_IRUGO, d2l_regs_show, NULL);

static struct attribute *d2l_attrs[] = {
	&dev_attr_d2l_regs.attr,
	NULL,
};

static struct attribute_group d2l_attr_group = {
	.attrs = d2l_attrs,
};

static int d2l_set_hs_mode_timing(struct omap_dss_device *dssdev)
{
	u32 tlpx, ths_prepare, ths_zero;
	u32 d2l_clk, byte_d2l_clk;
	u32 ppi_lptxtimecnt;
	u32 d0_3s_clrsipcnt;
	u32 ppi_rx_rx_ta_val;

	if (dssdev->clocks.dsi.regn == 0)
		goto error;

	d2l_clk = (2 * (dssdev->clocks.dsi.regm) /
			(dssdev->clocks.dsi.regn) * 26000000 / 1) / 4;
	byte_d2l_clk = d2l_clk / 1000 / 1000 / 4;

	if (d2l_clk == 0)
		goto error;

	/* nsec */
	tlpx = (dssdev->phy.dsi.hs_timing.tlpx_half
			* 1000 * 1000 / (d2l_clk / 1000)) * 2;
	ths_prepare = dssdev->phy.dsi.hs_timing.ths_prepare
			* 1000 * 1000 / (d2l_clk / 1000);
	ths_zero =
		(dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero
		- dssdev->phy.dsi.hs_timing.ths_prepare)
		* 1000 * 1000 / (d2l_clk / 1000);

	if (byte_d2l_clk > 1000)
		goto error;

	ppi_lptxtimecnt = tlpx / (1000 / byte_d2l_clk);
	d0_3s_clrsipcnt = (ths_prepare + ths_zero / 2) /
			(1000 / byte_d2l_clk) - 2;

	/* TXTAGOCNT is used to configure the TTA-GET timing as specified
	by the MIPI DPHY specification on Global Operation Timing Parameters.
	TXTAGOCNT[26:16]
	This register field should be set to be = (5 * PPI_LPTXTIMECNT . 3) / 4.
	TXTASURECNT [10:0]
	TXTASURECNT is used to configure the TTA-SURE timing as specified
	by the MIPI DPHY specification on Global Operation Timing Parameters
	This register field should be set to be = 3 * PPI_LPTXTIMECNT /2 */
	ppi_rx_rx_ta_val =
		(DIV_ROUND_UP(5 * ppi_lptxtimecnt - 3, 4) << 16)
		| DIV_ROUND_UP(3 * ppi_lptxtimecnt, 2);

	/* register setting if host wishes to perform DSI read transaction */
	d2l_write_register(dssdev, PPI_TX_RX_TA, ppi_rx_rx_ta_val);
	/* SYSLPTX Timing Generation Counter */
	d2l_write_register(dssdev, PPI_LPTXTIMECNT, ppi_lptxtimecnt);
	/* D*S_CLRSIPOCOUNT = [(THS-SETTLE+THS-ZERO) / HS_byte_clock_period] */
	d2l_write_register(dssdev, PPI_D0S_CLRSIPOCOUNT, d0_3s_clrsipcnt);
	d2l_write_register(dssdev, PPI_D1S_CLRSIPOCOUNT, d0_3s_clrsipcnt);
	d2l_write_register(dssdev, PPI_D2S_CLRSIPOCOUNT, d0_3s_clrsipcnt);
	d2l_write_register(dssdev, PPI_D3S_CLRSIPOCOUNT, d0_3s_clrsipcnt);

	return 0;
error:
	return -EINVAL;
}

static inline struct mapphone_dsi_panel_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct mapphone_dsi_panel_data *) dssdev->data;
}

int mapphone_panel_d2l_on(struct omap_dss_device *dssdev)
{
	int ret;
	u32 r;
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if ((dsi_vc_cmd == 0) && (dsi_vc_video == 0)) {
		if (dssdev->driver->get_dsi_vc_chnls)
			dssdev->driver->get_dsi_vc_chnls(dssdev, &dsi_vc_cmd,
						&dsi_vc_video);
		else {
			dsi_vc_cmd = 1;
			dsi_vc_video = 0;
		}
	}

	ret = d2l_set_hs_mode_timing(dssdev);
	if (ret)
		goto error;

	/* SpeedLaneSel: HS4L */
	d2l_write_register(dssdev, DSI_LANEENABLE, 0x0000001F);
	d2l_write_register(dssdev, PPI_LANEENABLE, 0x0000001F);

	d2l_write_register(dssdev, PPI_STARTPPI, 0x00000001);
	d2l_write_register(dssdev, DSI_STARTDSI, 0x00000001);

	/* configure D2L on-chip PLL */
	d2l_write_register(dssdev, LVPHY1, 0x00000000);
	/* set frequency range allowed and clock/data lanes */
	d2l_write_register(dssdev, LVPHY0, 0x00044006);

	/* configure D2L chip LCD Controller configuration registers */
	d2l_write_register(dssdev, VPCTRL, 0x00f00120);

	/* When VTGEN/EVTMODE are both on, EVTMODE has no weight.
	 * Only when VTGEN is OFF, EVTMODE matters.
	 * For VTGEN On, HTIM1,HTIM2,VTIM1,VTIM2 are required.
	 * For VTGEN Off, HTIM1, VTIM1 are required.
	 */
	r = FLD_VAL(panel_data->lvds_panel_timing.hbpr, 24, 16) |
		FLD_VAL(panel_data->lvds_panel_timing.hpw, 8, 0);
	d2l_write_register(dssdev, HTIM1, r);

	r = FLD_VAL(panel_data->lvds_panel_timing.hfpr, 24, 16) |
		FLD_VAL(dssdev->panel.timings.x_res, 10, 0);
	d2l_write_register(dssdev, HTIM2, r);

	r = FLD_VAL(panel_data->lvds_panel_timing.vbpr, 23, 16) |
		FLD_VAL(panel_data->lvds_panel_timing.vspr, 7, 0);
	d2l_write_register(dssdev, VTIM1, r);

	r = FLD_VAL(panel_data->lvds_panel_timing.vfpr, 23, 16) |
		FLD_VAL(dssdev->panel.timings.y_res, 10, 0);
	d2l_write_register(dssdev, VTIM2, r);

	d2l_write_register(dssdev, LVCFG, 0x00000001);

	/* Issue a soft reset to LCD Controller for a clean start */
	d2l_write_register(dssdev, SYSRST, 0x00000004);
	d2l_write_register(dssdev, VFUEN, 0x00000001);
	d2l_write_register(dssdev, DSI_INTCLR, 0xffffffff);

	return 0;
error:
	return -EINVAL;
}

void mapphone_panel_d2l_off(struct omap_dss_device *dssdev)
{
	if ((dsi_vc_cmd == 0) && (dsi_vc_video == 0)) {
		if (dssdev->driver->get_dsi_vc_chnls)
			dssdev->driver->get_dsi_vc_chnls(dssdev, &dsi_vc_cmd,
						&dsi_vc_video);
		else {
			dsi_vc_cmd = 1;
			dsi_vc_video = 0;
		}
	}

	omapdss_dsi_vc_enable_hs(dssdev, dsi_vc_cmd, false);
	dsi_videomode_panel_preinit(dssdev);
	/* LVEN = 0 */
	d2l_write_register(dssdev, LVCFG, 0x00000000);
	/* LV_RST = 1 and LV_BP = 1 */
	d2l_write_register(dssdev, LVPHY0, 0x00444106);
	msleep(5);
}

static int __devinit d2l_i2c_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	sd1 = kzalloc(sizeof(struct d2l_i2c), GFP_KERNEL);
	if (sd1 == NULL)
		return -ENOMEM;

	/* store i2c_client pointer on private data structure */
	sd1->client = client;

	/* store private data structure pointer on i2c_client structure */
	i2c_set_clientdata(client, sd1);

	/* init mutex */
	mutex_init(&sd1->xfer_lock);

	return 0;
}

/* driver remove function */
static int __devexit d2l_i2c_remove(struct i2c_client *client)
{
	struct d2l_i2c *sd1 = i2c_get_clientdata(client);

	/* remove client data */
	i2c_set_clientdata(client, NULL);

	/* free private data memory */
	kfree(sd1);

	return 0;
}

static const struct i2c_device_id d2l_i2c_idtable[] = {
	{DEVICE_NAME, 0},
	{},
};

static struct i2c_driver d2l_i2c_driver = {
	.probe = d2l_i2c_probe,
	.remove = __exit_p(d2l_i2c_remove),
	.id_table = d2l_i2c_idtable,
	.driver = {
		   .name = DRIVER_NAME},
};

static int __init mapphone_panel_d2l_init(void)
{
	return i2c_add_driver(&d2l_i2c_driver);
}

static void __exit mapphone_panel_d2l_exit(void)
{
	i2c_del_driver(&d2l_i2c_driver);
}

module_init(mapphone_panel_d2l_init);
module_exit(mapphone_panel_d2l_exit);

MODULE_AUTHOR("Jerry Alexander <x0135174@ti.com>");
MODULE_DESCRIPTION("D2l Driver");
MODULE_LICENSE("GPL");
