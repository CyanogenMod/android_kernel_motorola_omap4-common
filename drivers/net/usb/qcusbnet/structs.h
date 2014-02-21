/* structs.h - shared structures
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#ifndef QCUSBNET_STRUCTS_H
#define QCUSBNET_STRUCTS_H

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/kobject.h>
#include <linux/kthread.h>

#include <linux/usb/usbnet.h>

#include <linux/fdtable.h>

#define DBG(fmt, arg...)						\
do {									\
	if (qcusbnet_debug == 1)					\
		printk(KERN_INFO "QCUSBNet2k::%s " fmt, __func__, ##arg); \
} while (0)


#define VDBG(fmt, arg...)						\
do {									\
	if (qcusbnet_vdebug == 1)					\
		printk(KERN_INFO "QCUSBNet2k::%s " fmt, __func__, ##arg); \
} while (0)

#define ERR(fmt, arg...)\
	printk(KERN_ERR "QCUSBNet2k::%s " fmt, __func__, ##arg)

struct qcusbnet;

struct urbreq {
	struct list_head node;
	struct urb *urb;
};

#define DEFAULT_READ_URB_LENGTH 0x1000

struct qmidev {
	dev_t devnum;
	struct class *devclass;
	struct cdev *cdev;
	struct urb *readurb;
	struct urbsetup *readsetup;
	void *readbuf;
	struct urb *inturb;
	void *intbuf;
	struct list_head clients;
	/* clients lock */
	spinlock_t clients_lock;
	atomic_t qmitid;
};

enum {
	DOWN_NO_NDIS_CONNECTION = 0,
	DOWN_CDC_CONNECTION_SPEED = 1,
	DOWN_DRIVER_SUSPENDED = 2,
	DOWN_NET_IFACE_STOPPED = 3,
};

struct qcusbnet {
	struct list_head node;
	struct kref refcount;
	struct usbnet *usbnet;
	struct usb_interface *iface;
	struct work_struct work;
	struct workqueue_struct *wq;
	int (*open)(struct net_device *);
	int (*stop)(struct net_device *);
	netdev_tx_t (*start_xmit)
		(struct sk_buff *skb, struct net_device *dev);
	unsigned long down;
	bool registered;
	bool valid;
	bool dying;
	bool stopped;
	bool dlgt_carrier_state;
	struct mutex mutex; /* qcusbnet status mutex */

	struct qmidev qmi;
	char meid[14];
};

#endif /* !QCUSBNET_STRUCTS_H */
