/*
 * hdmi_ti_4xxx_ip_ddc.c
 *
 * DDC interface DSS driver setting for TI's OMAP4 family of processor.
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
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include "hdmi_ti_4xxx_ip_ddc.h"
#include "hdmi_ti_4xxx_ip.h"
#include "omap2/dss/dss.h"

struct ddc ddc;

/*-----------------------------------------------------------------------------
 * Function: ddc_start_transfer
 *-----------------------------------------------------------------------------
 */
int ddc_start_transfer(mddc_type *mddc_cmd, u8 operation)
{
	u8 *cmd = (u8 *)mddc_cmd;
	struct timeval t0, t1, t2;
	u32 time_elapsed_ms = 0;
	u32 i, size;
	unsigned long flags;

	mutex_lock(&ddc.lock);

#ifdef _9032_AUTO_RI_
	if (hdcp_suspend_resume_auto_ri(AUTO_RI_SUSPEND)) {
		mutex_unlock(&ddc.lock);
		return -DDC_ERROR;
	}
#endif

	/*
	 * Serialize DDC_CMD and DDC_ADDR register accesses, which
	 * are shared by ddc_abort(), with a spinlock.  ddc_abort()
	 * is called at interrupt time so cannot grab a mutex.  The
	 * other hdmi wp registers will be guarded using the mutex.
	 */
	spin_lock_irqsave(&ddc.spinlock, flags);

	/* Abort Master DDC operation and Clear FIFO pointer */
	WR_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__DDC_CMD, MASTER_CMD_CLEAR_FIFO);

	/* Read to flush */
	RD_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__DDC_CMD);

	/* Sending DDC header, it'll clear DDC Status register too */
	for (i = 0; i < 7; i++) {
		WR_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			  HDMI_IP_CORE_SYSTEM__DDC_ADDR + i * sizeof(uint32_t),
			  cmd[i]);

		/* Read to flush */
		RD_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			  HDMI_IP_CORE_SYSTEM__DDC_ADDR +
			  i * sizeof(uint32_t));
	}

	spin_unlock_irqrestore(&ddc.spinlock, flags);

	do_gettimeofday(&t1);
	t0 = t1;

	i = 0;
	size = mddc_cmd->nbytes_lsb + ((u32)mddc_cmd->nbytes_msb << 8);

	while ((i < size) && (ddc.pending_disable == 0)) {
		if (operation == DDC_WRITE) {
			/* Write data to DDC FIFO as long as it is NOT full */
			if (RD_FIELD_32(ddc.hdmi_wp_base_addr +
				       HDMI_IP_CORE_SYSTEM,
				       HDMI_IP_CORE_SYSTEM__DDC_STATUS, 3, 3)
									== 0) {
				WR_REG_32(ddc.hdmi_wp_base_addr +
					  HDMI_IP_CORE_SYSTEM,
					  HDMI_IP_CORE_SYSTEM__DDC_DATA,
					  mddc_cmd->pdata[i++]);
				do_gettimeofday(&t1);
			}
		} else if (operation == DDC_READ) {
			/* Read from DDC FIFO as long as it is NOT empty */
			if (RD_FIELD_32(ddc.hdmi_wp_base_addr +
				       HDMI_IP_CORE_SYSTEM,
				       HDMI_IP_CORE_SYSTEM__DDC_STATUS, 2, 2)
									== 0) {
				mddc_cmd->pdata[i++] =
					RD_REG_32(ddc.hdmi_wp_base_addr +
					  HDMI_IP_CORE_SYSTEM,
					  HDMI_IP_CORE_SYSTEM__DDC_DATA);
				do_gettimeofday(&t1);
			}
		}

		do_gettimeofday(&t2);
		time_elapsed_ms = (t2.tv_sec - t1.tv_sec) * 1000 +
				  (t2.tv_usec - t1.tv_usec) / 1000;

		if (time_elapsed_ms > DDC_TIMEOUT) {
			DBG("DDC timeout - no data during %d ms - "
			    "status=%02x %u",
					DDC_TIMEOUT,
					RD_REG_32(ddc.hdmi_wp_base_addr +
					  HDMI_IP_CORE_SYSTEM,
					  HDMI_IP_CORE_SYSTEM__DDC_STATUS),
					jiffies_to_msecs(jiffies));
			goto ddc_error;
		}
	}

	if (ddc.pending_disable)
		goto ddc_terminated;

	/* Wait for the FIFO to be empty (end of transfer) */
	while ((RD_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			   HDMI_IP_CORE_SYSTEM__DDC_STATUS) != 0x4) &&
		(ddc.pending_disable == 0)) {
		do_gettimeofday(&t2);
		time_elapsed_ms = (t2.tv_sec - t1.tv_sec) * 1000 +
				  (t2.tv_usec - t1.tv_usec) / 1000;

		if (time_elapsed_ms > DDC_TIMEOUT) {
			DBG("DDC timeout - FIFO not getting empty - "
			    "status=%02x",
				RD_REG_32(ddc.hdmi_wp_base_addr +
					  HDMI_IP_CORE_SYSTEM,
					  HDMI_IP_CORE_SYSTEM__DDC_STATUS));
			goto ddc_error;
		}
	}

	if (ddc.pending_disable)
		goto ddc_terminated;

	DBG("DDC transfer: bytes: %d time_us: %lu status: %x",
		i,
		(t2.tv_sec - t0.tv_sec) * 1000000 + (t2.tv_usec - t0.tv_usec),
		RD_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			  HDMI_IP_CORE_SYSTEM__DDC_STATUS));

#ifdef DDC_DBG
	{
		int k;
		for (k = 0; k < i; k++)
			printk(KERN_DEBUG "%02x ", mddc_cmd->pdata[k]);
		printk(KERN_DEBUG "\n");
	}
#endif

#ifdef _9032_AUTO_RI_
	/* Re-enable Auto Ri */
	if (hdcp_suspend_resume_auto_ri(AUTO_RI_RESUME)) {
		mutex_unlock(&ddc.lock);
		return -DDC_ERROR;
	}
#endif

	mutex_unlock(&ddc.lock);
	return DDC_OK;

ddc_error:
	ddc_abort();
	mutex_unlock(&ddc.lock);
	return -DDC_ERROR;

ddc_terminated:
	DBG("DDC transfer terminated - status=%02x",
		RD_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			  HDMI_IP_CORE_SYSTEM__DDC_STATUS));

	mutex_unlock(&ddc.lock);
	return DDC_OK;
}

/*-----------------------------------------------------------------------------
 * Function: hdmi_ddc_operation
 *-----------------------------------------------------------------------------
 */
static int ddc_operation(u16 no_bytes, u8 addr, u8 *pdata,
			      enum ddc_operation operation)
{
	mddc_type mddc;

	mddc.slaveAddr	= DDC_RX_SLV;
	mddc.offset	= 0;
	mddc.regAddr	= addr;
	mddc.nbytes_lsb	= no_bytes & 0xFF;
	mddc.nbytes_msb	= (no_bytes & 0x300) >> 8;
	mddc.dummy	= 0;
	mddc.pdata	= pdata;

	if (operation == DDC_READ)
		mddc.cmd = MASTER_CMD_SEQ_RD;
	else
		mddc.cmd = MASTER_CMD_SEQ_WR;

	DBG("DDC %s: offset=%02x len=%d %u", operation == DDC_READ ?
					     "READ" : "WRITE",
					     addr, no_bytes,
					     jiffies_to_msecs(jiffies));

	return ddc_start_transfer(&mddc, operation);
}

/*-----------------------------------------------------------------------------
 * Function: ddc_read
 *-----------------------------------------------------------------------------
 */
int ddc_read(u16 no_bytes, u8 addr, u8 *pdata)
{
	return ddc_operation(no_bytes, addr, pdata, DDC_READ);
}

/*-----------------------------------------------------------------------------
 * Function: ddc_write
 *-----------------------------------------------------------------------------
 */
int ddc_write(u16 no_bytes, u8 addr, u8 *pdata)
{
	return ddc_operation(no_bytes, addr, pdata, DDC_WRITE);
}

/*-----------------------------------------------------------------------------
 * Function: ddc_abort
 *-----------------------------------------------------------------------------
 */
void ddc_abort(void)
{
	unsigned long flags;

	/* In case of I2C_NO_ACK error, do not abort DDC to avoid
	 * DDC lockup
	 */
	if (RD_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		      HDMI_IP_CORE_SYSTEM__DDC_STATUS) & 0x20) {
		return;
	}

	spin_lock_irqsave(&ddc.spinlock, flags);

	/* Abort Master DDC operation and Clear FIFO pointer */
	WR_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__DDC_CMD, MASTER_CMD_ABORT);

	/* Read to flush */
	RD_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__DDC_CMD);

	WR_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__DDC_CMD, MASTER_CMD_CLEAR_FIFO);

	/* Read to flush */
	RD_REG_32(ddc.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__DDC_CMD);

	spin_unlock_irqrestore(&ddc.spinlock, flags);
}

static int __init ddc_init(void)
{
	DBG("ddc_init() %u", jiffies_to_msecs(jiffies));

	/* Map HDMI WP address */
	ddc.hdmi_wp_base_addr = ioremap(HDMI_WP, 0x1000);

	if (!ddc.hdmi_wp_base_addr) {
		printk(KERN_ERR "DDC: HDMI WP IOremap error\n");
		return -EFAULT;
	}

	spin_lock_init(&ddc.spinlock);
	mutex_init(&ddc.lock);

	return 0;

}
static void __exit ddc_exit(void)
{
	DBG("ddc_exit() %u", jiffies_to_msecs(jiffies));

	mutex_lock(&ddc.lock);

	/* Unmap HDMI WP */
	iounmap(ddc.hdmi_wp_base_addr);

	mutex_unlock(&ddc.lock);
	mutex_destroy(&ddc.lock);
}

module_init(ddc_init);
module_exit(ddc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OMAP DDC kernel module");
MODULE_AUTHOR("Dandawate Saket");

