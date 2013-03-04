/*
 * omap-hdcp.h
 *
 * HDCP interface DSS driver setting for TI's OMAP4 family of processor.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Fabrice Olivero
 *	Fabrice Olivero <f-olivero@ti.com>
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

#ifndef _OMAP_HDCP_H_
#define _OMAP_HDCP_H_


/********************************/
/* Structures related to ioctl  */
/********************************/

/* HDCP key size in 32-bit words */
#define DESHDCP_KEY_SIZE 160

/* HDCP ioctl */
#include <linux/ioctl.h>
#include <linux/types.h>

struct hdcp_encrypt_control {
	uint32_t in_key[DESHDCP_KEY_SIZE];
	uint32_t *out_key;
};

struct hdcp_enable_control {
	uint32_t key[DESHDCP_KEY_SIZE];
	int nb_retry;
};

#define MAX_SHA_DATA_SIZE	645
#define MAX_SHA_VPRIME_SIZE	20

struct hdcp_sha_in {
	uint8_t data[MAX_SHA_DATA_SIZE];
	uint32_t byte_counter;
	uint8_t vprime[MAX_SHA_VPRIME_SIZE];
};

struct hdcp_wait_control {
	uint32_t event;
	struct hdcp_sha_in *data;
};

/* HDCP ioctl */
#define HDCP_IOCTL_MAGIC 'h'
#define HDCP_ENABLE	  _IOW(HDCP_IOCTL_MAGIC, 0, \
				struct hdcp_enable_control)
#define HDCP_DISABLE	  _IO(HDCP_IOCTL_MAGIC, 1)
#define HDCP_ENCRYPT_KEY  _IOWR(HDCP_IOCTL_MAGIC, 2, \
				struct hdcp_encrypt_control)
#define HDCP_QUERY_STATUS _IOWR(HDCP_IOCTL_MAGIC, 3, uint32_t)
#define HDCP_WAIT_EVENT _IOWR(HDCP_IOCTL_MAGIC, 4, \
				struct hdcp_wait_control)
#define HDCP_DONE	_IOW(HDCP_IOCTL_MAGIC, 5, uint32_t)

/* HDCP state */
#define HDCP_STATE_DISABLED		0
#define HDCP_STATE_INIT			1
#define HDCP_STATE_AUTH_1ST_STEP	2
#define HDCP_STATE_AUTH_2ND_STEP	3
#define HDCP_STATE_AUTH_3RD_STEP	4
#define HDCP_STATE_AUTH_FAIL_RESTARTING	5
#define HDCP_STATE_AUTH_FAILURE		6

/* HDCP events */
#define HDCP_EVENT_STEP1	(1 << 0x0)
#define HDCP_EVENT_STEP2	(1 << 0x1)
#define HDCP_EVENT_EXIT		(1 << 0x2)

/* HDCP user space status */
#define HDCP_US_NO_ERR		(0 << 8)
#define HDCP_US_FAILURE		(1 << 8)

#endif /* _OMAP_HDCP_H_ */
