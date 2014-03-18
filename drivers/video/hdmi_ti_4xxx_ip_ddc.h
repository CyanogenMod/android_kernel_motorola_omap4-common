/*
 * hdmi_ti_4xxx_ip_ddc.h
 *
 * HDMI interface DSS driver setting for TI's OMAP4 family of processor.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Dandawate Saket
 *	Dandawate Saket <dsaket@ti.com>
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
#include <linux/ioctl.h>
#include <linux/types.h>

#define DDC_RX_SLV 0x74

#define MASTER_BASE		0xEC
#define MDDC_MANUAL_ADDR	0xEC
#define MDDC_SLAVE_ADDR		0xED
#define MDDC_SEGMENT_ADDR	0xEE
#define MDDC_OFFSET_ADDR	0xEF
#define MDDC_DIN_CNT_LSB_ADDR	0xF0
#define MDDC_DIN_CNT_MSB_ADDR	0xF1
#define MDDC_STATUS_ADDR	0xF2
#define MDDC_COMMAND_ADDR	0xF3
#define MDDC_FIFO_ADDR		0xF4
#define MDDC_FIFO_CNT_ADDR	0xF5

#define BIT_MDDC_ST_IN_PROGR	0x10
#define BIT_MDDC_ST_I2C_LOW	0x40
#define BIT_MDDC_ST_NO_ACK	0x20

/* DDC Command[3:0]:
 *
 * 1111 - Abort transaction
 * 1001 - Clear FIFO
 * 1010 - Clock SCL
 * 0000 - Current address read with no ACK on last byte
 * 0001 - Current address read with ACK on last byte
 * 0010 - Sequential read with no ACK on last byte
 * 0011 - Sequential read with ACK on last byte
 * 0100 - Enhanced DDC read with no ACK on last byte
 * 0101 - Enhanced DDC read with ACK on last byte
 * 0110 - Sequential write ignoring ACK on last byte
 * 0111 - Sequential write requiring ACK on last byte
 */

#define MASTER_CMD_ABORT        0x0f
#define MASTER_CMD_CLEAR_FIFO   0x09
#define MASTER_CMD_CLOCK        0x0a
#define MASTER_CMD_CUR_RD       0x00
#define MASTER_CMD_SEQ_RD       0x02
#define MASTER_CMD_ENH_RD       0x04
#define MASTER_CMD_SEQ_WR       0x06

#define MASTER_FIFO_WR_USE      0x01
#define MASTER_FIFO_RD_USE      0x02
#define MASTER_FIFO_EMPTY       0x04
#define MASTER_FIFO_FULL        0x08
#define MASTER_DDC_BUSY         0x10
#define MASTER_DDC_NOACK        0x20
#define MASTER_DDC_STUCK        0x40
#define MASTER_DDC_RSVD         0x80

/* OMAP 4 HDMI TRM: */
#define HDMI_IP_CORE_SYSTEM__DDC_MAN		0x3B0
#define HDMI_IP_CORE_SYSTEM__DDC_ADDR		0x3B4
#define HDMI_IP_CORE_SYSTEM__DDC_SEGM		0x3B8
#define HDMI_IP_CORE_SYSTEM__DDC_OFFSET		0x3BC
#define HDMI_IP_CORE_SYSTEM__DDC_COUNT1		0x3C0
#define HDMI_IP_CORE_SYSTEM__DDC_COUNT2		0x3C4
#define HDMI_IP_CORE_SYSTEM__DDC_STATUS		0x3C8
#define HDMI_IP_CORE_SYSTEM__DDC_CMD		0x3CC
#define HDMI_IP_CORE_SYSTEM__DDC_DATA		0x3D0
#define HDMI_IP_CORE_SYSTEM__DDC_FIFOCNT	0x3D4

#define IIC_OK 0
#define _IIC_CAPTURED  1
#define _IIC_NOACK     2
#define _MDDC_CAPTURED 3
#define _MDDC_NOACK    4
#define _MDDC_FIFO_FULL  5

typedef struct {
	u8 slaveAddr;
	u8 offset; /* "offset = DDC_SEGM register" */
	u8 regAddr;
	u8 nbytes_lsb;
	u8 nbytes_msb;
	u8 dummy;
	u8 cmd;
	u8 *pdata;
	u8 data[6];
} mddc_type;

enum ddc_operation {
	DDC_READ,
	DDC_WRITE
};

enum ri_suspend_resume {
	AUTO_RI_SUSPEND,
	AUTO_RI_RESUME
};

/***********************/
/* DDC addresses  */
/***********************/

#define DDC_BKSV_ADDR		0x00
#define DDC_Ri_ADDR		0x08
#define DDC_AKSV_ADDR		0x10
#define DDC_AN_ADDR		0x18
#define DDC_V_ADDR		0x20
#define DDC_BCAPS_ADDR		0x40
#define DDC_BSTATUS_ADDR	0x41
#define DDC_KSV_FIFO_ADDR	0x43

#define DDC_BKSV_LEN		5
#define DDC_Ri_LEN		2
#define DDC_AKSV_LEN		5
#define DDC_AN_LEN		8
#define DDC_V_LEN		20
#define DDC_BCAPS_LEN		1
#define DDC_BSTATUS_LEN		2

#define DDC_BIT_REPEATER	6

#define DDC_BSTATUS0_MAX_DEVS	0x80
#define DDC_BSTATUS0_DEV_COUNT	0x7F
#define DDC_BSTATUS1_MAX_CASC	0x08

#ifdef DEBUG
#define DDC_DBG			/* Log DDC data */
#undef POWER_TRANSITION_DBG	/* Add wait loops to allow testing DSS power
				 * transition during debug*/
#endif

/* DDC access timeout in ms */
#define DDC_TIMEOUT	500
#define DDC_STOP_FRAME_BLOCKING_TIMEOUT (2*DDC_TIMEOUT)

#define MAX_DDC_ERR	5

/* Status / error codes */
#define DDC_OK			0
#define DDC_ERROR		1

#define HDMI_IP_CORE_SYSTEM 0x400
/* HDMI WP base address */
/*----------------------*/
#define HDMI_WP			0x58006000

#define WR_REG_32(base, offset, val)	__raw_writel(val, base + offset)
#define RD_REG_32(base, offset)		__raw_readl(base + offset)

#define RD_FIELD_32(base, offset, start, end) \
	((RD_REG_32(base, offset) & FLD_MASK(start, end)) >> (end))


#undef DBG

#ifdef HDCP_DEBUG
#define DBG(format, ...) \
		printk(KERN_DEBUG "HDCP: " format "\n", ## __VA_ARGS__)
#else
#define DBG(format, ...)
#endif

struct ddc {
	void __iomem *hdmi_wp_base_addr;
	struct mutex lock;
	spinlock_t spinlock;
	int pending_disable;
};
extern struct ddc ddc;

/* DDC */
int ddc_read(u16 no_bytes, u8 addr, u8 *pdata);
int ddc_write(u16 no_bytes, u8 addr, u8 *pdata);
void ddc_abort(void);
int ddc_start_transfer(mddc_type *mddc_cmd, u8 operation);
