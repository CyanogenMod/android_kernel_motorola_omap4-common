/* qcusbnet.c - gobi network device
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.

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

#include <linux/workqueue.h>
#include <linux/ctype.h>

#include "structs.h"
#include "qmidevice.h"
#include "qmi.h"
#include "qcusbnet.h"

#include "../../../usb/core/usb.h"
#include <linux/ctype.h>
#include <linux/if.h>
#include <linux/if_arp.h>

#define DRIVER_VERSION "1.0.110+google"
#define DRIVER_AUTHOR "Qualcomm Innovation Center"
#define DRIVER_DESC "QCUSBNet2k"

#define QMI_AUTOSUSPEND_DELAY 500

static LIST_HEAD(qcusbnet_list);
static DEFINE_MUTEX(qcusbnet_lock);

int qcusbnet_debug;
int qcusbnet_vdebug;
static struct class *devclass;

static __be16 qcnet_prot_type_trans(struct usbnet *usbnet, struct sk_buff *skb,
							struct net_device *dev);

static void free_dev(struct kref *ref)
{
	struct qcusbnet *dev = container_of(ref, struct qcusbnet, refcount);
	list_del(&dev->node);
	kfree(dev);
}

void qcusbnet_put(struct qcusbnet *dev)
{
	mutex_lock(&qcusbnet_lock);
	kref_put(&dev->refcount, free_dev);
	mutex_unlock(&qcusbnet_lock);
}

struct qcusbnet *qcusbnet_get(struct qcusbnet *key)
{
	/* Given a putative qcusbnet struct, return either the struct itself
	 * (with a ref taken) if the struct is still visible, or NULL if it's
	 * not. This prevents object-visibility races where someone is looking
	 * up an object as the last ref gets dropped; dropping the last ref and
	 * removing the object from the list are atomic with respect to getting
	 * a new ref. */
	struct qcusbnet *entry;
	mutex_lock(&qcusbnet_lock);
	list_for_each_entry(entry, &qcusbnet_list, node) {
		if (entry == key) {
			kref_get(&entry->refcount);
			mutex_unlock(&qcusbnet_lock);
			return entry;
		}
	}
	mutex_unlock(&qcusbnet_lock);
	return NULL;
}

struct qcusbnet *cdev_to_qcusbnet(struct cdev *cdev)
{
	struct qcusbnet *entry;
	mutex_lock(&qcusbnet_lock);
	list_for_each_entry(entry, &qcusbnet_list, node) {
		if (entry->qmi.cdev == cdev) {
			mutex_unlock(&qcusbnet_lock);
			return entry;
		}
	}
	mutex_unlock(&qcusbnet_lock);
	return NULL;
}

int qc_suspend(struct usb_interface *iface, pm_message_t event)
{
	struct usbnet *usbnet;
	struct qcusbnet *dev;

	if (!iface)
		return -ENOMEM;

	usbnet = usb_get_intfdata(iface);

	if (!usbnet || !usbnet->net) {
		ERR("failed to get netdevice\n");
		return -ENXIO;
	}

	dev = (struct qcusbnet *)usbnet->data[0];
	if (!dev) {
		ERR("failed to get QMIDevice\n");
		return -ENXIO;
	}

	if (!dev->registered) {
		ERR("QMI device not registered");
		return -EBUSY;
	}

	if (dev->wq) {
		destroy_workqueue(dev->wq);
		dev->wq = NULL;
	}


	if (event.event & PM_EVENT_SUSPEND &&
	    !(iface->dev.power.power_state.event & PM_EVENT_SUSPEND)) {
		if (usbnet_suspend(iface, event)) {
			DBG("usbnet suspend failed");
			return -EBUSY;
		}
		qc_stopread(dev);
		iface->dev.power.power_state.event = event.event;
		DBG("device suspended to power level %d\n", event.event);
	} else {
		DBG("Already Suspended ? %d %d\n", event.event,
		    iface->dev.power.power_state.event);
	}

	return 0;
}

static int qc_resume(struct usb_interface *iface)
{
	struct usbnet *usbnet;
	struct qcusbnet *dev;
	int ret = 0;
	usbnet = usb_get_intfdata(iface);
	dev = (struct qcusbnet *)usbnet->data[0];

	if (!dev || !usbnet->net) {
		ERR("failed to get net / QMIDevice\n");
		goto resume_fail;
	}

	if (iface->dev.power.power_state.event & PM_EVENT_SUSPEND) {
		DBG("resuming device from %d\n",
		    iface->dev.power.power_state.event);
		if (dev->registered)
			ret = qc_startread(dev);

		if (ret || !dev->registered) {
			ERR("qc_startread error %d %d\n",
			    ret, dev->registered);
			goto resume_fail;
		}
		iface->dev.power.power_state.event = PM_EVENT_ON;
		usbnet_resume(iface);
	} else {
		DBG("nothing to resume\n");
	}
	return 0;

resume_fail:
	ERR("resume failed, disconnect. state %d\n", usbnet->udev->state);
	if (usbnet->udev->state != USB_STATE_NOTATTACHED)
		usb_remove_device(usbnet->udev);
	return 0;

}

static int qc_reset_resume(struct usb_interface *iface)
{
	return qc_resume(iface);
}

static int qcnet_bind(struct usbnet *usbnet, struct usb_interface *iface)
{
	int numends;
	int i;
	struct usb_host_endpoint *endpoint = NULL;
	struct usb_host_endpoint *in = NULL;
	struct usb_host_endpoint *out = NULL;

	if (iface->num_altsetting != 1) {
		ERR("invalid num_altsetting %u\n", iface->num_altsetting);
		return -EINVAL;
	}

	numends = iface->cur_altsetting->desc.bNumEndpoints;
	for (i = 0; i < numends; i++) {
		endpoint = iface->cur_altsetting->endpoint + i;
		if (!endpoint) {
			ERR("invalid endpoint %u\n", i);
			return -EINVAL;
		}

		if (usb_endpoint_is_bulk_in(&endpoint->desc))
			in = endpoint;
		else if (usb_endpoint_is_bulk_out(&endpoint->desc))
			out = endpoint;
	}

	if (!in || !out) {
		ERR("invalid bulk endpoints\n");
		return -EINVAL;
	}

	if (usb_set_interface(usbnet->udev,
		iface->cur_altsetting->desc.bInterfaceNumber, 0))	{
		ERR("unable to set interface\n");
		return -EINVAL;
	}

	usbnet->in = usb_rcvbulkpipe(usbnet->udev,
			in->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	usbnet->out = usb_sndbulkpipe(usbnet->udev,
			out->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);

	DBG("in %x, out %x\n",
	    in->desc.bEndpointAddress,
	    out->desc.bEndpointAddress);

	strcpy(usbnet->net->name, "qmi%d");
	if (usbnet->net->type != ARPHRD_RAWIP)
		random_ether_addr(&usbnet->net->dev_addr[0]);

	return 0;
}

#define XOOM_DOWNLINK_MTU 1500
static int xoom_qcnet_bind(struct usbnet *usbnet, struct usb_interface *iface)
{
	int status = qcnet_bind(usbnet, iface);

	if (!status)
		usbnet->rx_urb_size = XOOM_DOWNLINK_MTU +
					usbnet->net->hard_header_len;

	return status;
}

static void qcnet_unbind(struct usbnet *usbnet, struct usb_interface *iface)
{
	struct qcusbnet *dev = (struct qcusbnet *)usbnet->data[0];
	if (dev->wq) {
		destroy_workqueue(dev->wq);
		dev->wq = NULL;
	}
	qc_deregister(dev);
	kfree(usbnet->net->netdev_ops);
	usbnet->net->netdev_ops = NULL;
	/* drop the list's ref */
	qcusbnet_put(dev);
}

static int qcnet_manage_power(struct usbnet *usbnet, int on)
{
	return 0;
}

static int qcnet_startxmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct qcusbnet *dev;
	struct usbnet *usbnet = netdev_priv(netdev);

	VDBG("\n");

	if (!usbnet || !usbnet->net) {
		ERR("failed to get usbnet device\n");
		return NETDEV_TX_BUSY;
	}

	dev = (struct qcusbnet *)usbnet->data[0];
	if (!dev) {
		ERR("failed to get QMIDevice\n");
		return NETDEV_TX_BUSY;
	}

	if (dev->start_xmit)
		dev->start_xmit(skb, netdev);
	else
		ERR("no USBNetStartXmit defined\n");

	return NETDEV_TX_OK;
}

static int qcnet_open(struct net_device *netdev)
{
	int status = 0;
	struct qcusbnet *dev;
	struct usbnet *usbnet = netdev_priv(netdev);

	if (!usbnet) {
		ERR("failed to get usbnet device\n");
		return -ENXIO;
	}

	dev = (struct qcusbnet *)usbnet->data[0];
	if (!dev) {
		ERR("failed to get QMIDevice\n");
		return -ENXIO;
	}

	DBG("\n");

	qc_cleardown(dev, DOWN_NET_IFACE_STOPPED);
	if (dev->open)
		status = dev->open(netdev);
	else
		ERR("no USBNetOpen defined\n");

	return status;
}

int qcnet_stop(struct net_device *netdev)
{
	struct qcusbnet *dev;
	struct usbnet *usbnet = netdev_priv(netdev);

	if (!usbnet || !usbnet->net) {
		ERR("failed to get netdevice\n");
		return -ENXIO;
	}

	dev = (struct qcusbnet *)usbnet->data[0];
	if (!dev) {
		ERR("failed to get QMIDevice\n");
		return -ENXIO;
	}

	qc_setdown(dev, DOWN_NET_IFACE_STOPPED);

	if (dev->stop != NULL)
		return dev->stop(netdev);
	return 0;
}

static const struct driver_info qc_netinfo = {
	.description   = "QCUSBNet Ethernet Device",
	.flags         = FLAG_ETHER,
	.bind          = qcnet_bind,
	.unbind        = qcnet_unbind,
	.data          = 0,
	.manage_power  = qcnet_manage_power,
};

static const struct driver_info xoom_qc_netinfo = {
	.description   = "Xoom QCUSBNet Ethernet Device",
	.flags         = FLAG_ETHER|FLAG_SEND_ZLP,
	.bind          = xoom_qcnet_bind,
	.unbind        = qcnet_unbind,
	.data          = 0,
	.manage_power  = qcnet_manage_power,
};

static const struct driver_info mdm9600_qc_netinfo = {
	.description   = "MDM9600 QCUSBNet IP Device",
	.flags         = FLAG_ETHER|FLAG_SEND_ZLP,
	.bind          = xoom_qcnet_bind,
	.unbind        = qcnet_unbind,
	.data          = 0,
	.manage_power  = qcnet_manage_power,
	.prot_type_trans = qcnet_prot_type_trans,
};

#define MKVIDPID(v, p)					\
{							\
	USB_DEVICE(v, p),				\
	.driver_info = (unsigned long)&qc_netinfo,	\
}

static const struct usb_device_id qc_vidpids[] = {
	MKVIDPID(0x05c6, 0x9215),	/* Acer Gobi 2000 */
	MKVIDPID(0x05c6, 0x9265),	/* Asus Gobi 2000 */
	MKVIDPID(0x16d8, 0x8002),	/* CMOTech Gobi 2000 */
	MKVIDPID(0x413c, 0x8186),	/* Dell Gobi 2000 */
	MKVIDPID(0x1410, 0xa010),	/* Entourage Gobi 2000 */
	MKVIDPID(0x1410, 0xa011),	/* Entourage Gobi 2000 */
	MKVIDPID(0x1410, 0xa012),	/* Entourage Gobi 2000 */
	MKVIDPID(0x1410, 0xa013),	/* Entourage Gobi 2000 */
	MKVIDPID(0x03f0, 0x251d),	/* HP Gobi 2000 */
	MKVIDPID(0x05c6, 0x9205),	/* Lenovo Gobi 2000 */
	MKVIDPID(0x05c6, 0x920b),	/* Generic Gobi 2000 */
	MKVIDPID(0x04da, 0x250f),	/* Panasonic Gobi 2000 */
	MKVIDPID(0x05c6, 0x9245),	/* Samsung Gobi 2000 */
	MKVIDPID(0x1199, 0x9001),	/* Sierra Wireless Gobi 2000 */
	MKVIDPID(0x1199, 0x9002),	/* Sierra Wireless Gobi 2000 */
	MKVIDPID(0x1199, 0x9003),	/* Sierra Wireless Gobi 2000 */
	MKVIDPID(0x1199, 0x9004),	/* Sierra Wireless Gobi 2000 */
	MKVIDPID(0x1199, 0x9005),	/* Sierra Wireless Gobi 2000 */
	MKVIDPID(0x1199, 0x9006),	/* Sierra Wireless Gobi 2000 */
	MKVIDPID(0x1199, 0x9007),	/* Sierra Wireless Gobi 2000 */
	MKVIDPID(0x1199, 0x9008),	/* Sierra Wireless Gobi 2000 */
	MKVIDPID(0x1199, 0x9009),	/* Sierra Wireless Gobi 2000 */
	MKVIDPID(0x1199, 0x900a),	/* Sierra Wireless Gobi 2000 */
	MKVIDPID(0x05c6, 0x9225),	/* Sony Gobi 2000 */
	MKVIDPID(0x05c6, 0x9235),	/* Top Global Gobi 2000 */
	MKVIDPID(0x05c6, 0x9275),	/* iRex Technologies Gobi 2000 */

	MKVIDPID(0x05c6, 0x920d),	/* Qualcomm Gobi 3000 */
	MKVIDPID(0x1410, 0xa021),	/* Novatel Gobi 3000 */

	{ /* Motorola Xoom */
	USB_DEVICE_AND_INTERFACE_INFO(0x22B8, 0x2A70, 0xff, 0xfb, 0xff),
	.driver_info = (unsigned long)&xoom_qc_netinfo
	},
	{ /* MDM9600 */
	USB_DEVICE_AND_INTERFACE_INFO(0x22B8, 0x2E0A, 0xff, 0xfb, 0xff),
	.driver_info = (unsigned long)&mdm9600_qc_netinfo
	},
	{ }
};

MODULE_DEVICE_TABLE(usb, qc_vidpids);

static void qcnet_register(struct work_struct *w)
{
	struct qcusbnet *dev = container_of(w, struct qcusbnet, work);
	qc_register(dev);
}

int qcnet_probe(struct usb_interface *iface,
		const struct usb_device_id *vidpids)
{
	int status;
	struct usbnet *usbnet;
	struct qcusbnet *dev;
	struct net_device_ops *netdevops;

	status = usbnet_probe(iface, vidpids);
	if (status < 0) {
		ERR("usbnet_probe failed %d\n", status);
		return status;
	}

	iface->needs_remote_wakeup = 1;
	usbnet = usb_get_intfdata(iface);

	if (!usbnet || !usbnet->net) {
		ERR("failed to get netdevice\n");
		return -ENXIO;
	}

	dev = kzalloc(sizeof(struct qcusbnet), GFP_KERNEL);
	if (!dev) {
		ERR("failed to allocate device buffers\n");
		return -ENOMEM;
	}

	usbnet->data[0] = (unsigned long)dev;

	dev->usbnet = usbnet;

	dev->wq = create_singlethread_workqueue("qcregister_work");
	if (!dev->wq) {
		ERR("failed to create workqueue\n");
		kzfree(dev);
		return -ENOMEM;
	}

	netdevops = kmalloc(sizeof(struct net_device_ops), GFP_KERNEL);
	if (!netdevops) {
		ERR("failed to allocate net device ops\n");
		destroy_workqueue(dev->wq);
		kzfree(dev);
		return -ENOMEM;
	}
	memcpy(netdevops, usbnet->net->netdev_ops,
				sizeof(struct net_device_ops));

	dev->open = netdevops->ndo_open;
	netdevops->ndo_open = qcnet_open;
	dev->stop = netdevops->ndo_stop;
	netdevops->ndo_stop = qcnet_stop;
	dev->start_xmit = netdevops->ndo_start_xmit;
	netdevops->ndo_start_xmit = qcnet_startxmit;

	if (vidpids->driver_info == (unsigned long)&mdm9600_qc_netinfo) {
		DBG("Switch to RAW IP mode for 9k modems\n");
		netdevops->ndo_set_mac_address = 0;
		netdevops->ndo_validate_addr   = 0;

		usbnet->net->header_ops        = 0;
		usbnet->net->type              = ARPHRD_RAWIP;
		usbnet->net->hard_header_len   = 0;
		usbnet->net->addr_len          = 0;

		usbnet->net->flags &= ~(IFF_BROADCAST | IFF_MULTICAST);
	}

	usbnet->net->netdev_ops = netdevops;
	usbnet->udev->dev.power.autosuspend_delay = QMI_AUTOSUSPEND_DELAY;
	memset(&(dev->usbnet->net->stats), 0, sizeof(struct net_device_stats));

	dev->iface = iface;
	memset(&(dev->meid), '0', 14);

	mutex_init(&dev->mutex);
	dev->valid = false;
	dev->registered = false;
	dev->dying = false;
	memset(&dev->qmi, 0, sizeof(dev->qmi));

	dev->qmi.devclass = devclass;

	kref_init(&dev->refcount);
	INIT_LIST_HEAD(&dev->node);
	INIT_LIST_HEAD(&dev->qmi.clients);
	spin_lock_init(&dev->qmi.clients_lock);

	dev->down = 0;
	qc_setdown(dev, DOWN_NO_NDIS_CONNECTION);
	qc_setdown(dev, DOWN_NET_IFACE_STOPPED);

	INIT_WORK(&dev->work, qcnet_register);
	queue_work(dev->wq, &dev->work);

	mutex_lock(&qcusbnet_lock);
	/* Give our initial ref to the list */
	list_add(&dev->node, &qcusbnet_list);
	mutex_unlock(&qcusbnet_lock);

	return status;
}
EXPORT_SYMBOL_GPL(qcnet_probe);

static __be16 qcnet_prot_type_trans(
		struct usbnet *usbnet,
		struct sk_buff *skb,
		struct net_device *dev)
{
	__be16 protocol = 0;

	if (usbnet->driver_info != &mdm9600_qc_netinfo) {
		/* Looks like a case of dangling pointer
		 * set protocol to default value (ethernet)
		 */
		return eth_type_trans(skb, dev);
	}

	skb->dev = dev;
	/* Determine L3 protocol */
	switch (skb->data[0] & 0xf0) {
	case 0x40:
		protocol = htons(ETH_P_IP);
		break;
	case 0x60:
		protocol = htons(ETH_P_IPV6);
		break;
	default:
		ERR("L3 protocol decode error: 0x%02x",
			skb->data[0] & 0xf0);
		/* skb will be dropped in upper layer for unknown protocol */
	}

	return protocol;
}

static struct usb_driver qcusbnet = {
	.name		= "QCUSBNet2k",
	.id_table	= qc_vidpids,
	.probe		= qcnet_probe,
	.disconnect	= usbnet_disconnect,
	.suspend	= qc_suspend,
	.resume		= qc_resume,
	.reset_resume	= qc_reset_resume,
	.supports_autosuspend = true,
};

static int __init modinit(void)
{
	devclass = class_create(THIS_MODULE, "QCQMI");
	if (IS_ERR(devclass)) {
		ERR("error at class_create %ld\n", PTR_ERR(devclass));
		return -ENOMEM;
	}
	printk(KERN_INFO "%s: %s\n", DRIVER_DESC, DRIVER_VERSION);
	return usb_register(&qcusbnet);
}
module_init(modinit);

static void __exit modexit(void)
{
	usb_deregister(&qcusbnet);
	class_destroy(devclass);
}
module_exit(modexit);

MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("Dual BSD/GPL");

module_param(qcusbnet_debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(qcusbnet_debug, "Debugging enabled or not");

module_param(qcusbnet_vdebug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(qcusbnet_vdebug, "Verbose debugging enabled or not");
