/* Network device over SPI support for Qualcomm MDM6xxxx serial
*/

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>

#include "qmi_net.h"
#include "ts27010_mux.h"

#define SPINET_NUM 4
#define TX_TIMEOUT (5*HZ)

static struct class *devclass;
struct net_device *spinet_devs[SPINET_NUM];
int qcspinet_debug;
module_param(qcspinet_debug, int, S_IRUGO | S_IWUSR);

#define SPINET_DBG(x...)  pr_debug(x)

static void tx_done(void *data)
{
	struct net_device *net = (struct net_device *)data;
	SPINET_DBG("%s\n", __func__);
	netif_wake_queue(net);
}

struct qcspinet *cdev_to_qcspinet(struct cdev *cdev)
{
	int i;
	struct qcspinet *entry;

	for (i = 0; i < SPINET_NUM; i++) {
		entry = netdev_priv(spinet_devs[i]);
		if (entry->qmi.cdev == cdev)
			return entry;
	}
	return NULL;
}

static void *spi_net_alloc_skb(unsigned char **buf, int len, void *data)
{
	struct sk_buff *skb;
	struct net_device *net = (struct net_device *)data;

	SPINET_DBG(KERN_ERR "%s: alloc skb for %d bytes\n", __func__, len);
	skb = dev_alloc_skb(len + 2);
	if (!skb) {
		/*printk_ratelimited
			("%s: on mem, packet dropped\n", __func__);*/
		net->stats.rx_dropped++;
		return NULL;
	}
	skb_reserve(skb, 2);
	*buf = skb_put(skb, len);

	return skb;
}

static int spi_net_rx(int line, void *obj, int len, void *data)
{
	int ret = 0;
	struct sk_buff *skb;
	struct net_device *net = (struct net_device *)data;

	SPINET_DBG(KERN_ERR "%s: got data %d bytes\n", __func__, len);

	skb = (struct sk_buff *)obj;
	skb->dev = net;
	skb->protocol = eth_type_trans(skb, net);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	net->stats.rx_packets++;
	net->stats.rx_bytes += len;
	netif_rx_ni(skb);
	ret = len;

	return ret;
}

static int spi_net_open(struct net_device *dev)
{
	int ret;
	struct qcspinet *priv;

	SPINET_DBG("%s\n", __func__);

	priv = netdev_priv(dev);
	/* open mux channel */
	ret = qcspi_mux_fake_open(priv->mux_cid,
		spi_net_rx, spi_net_alloc_skb, (void *)dev);
	if (ret) {
		pr_err("Failed to open mux %d!\n", priv->mux_cid);
		return ret;
	}
	qcspi_ldisc_add_write_done_notify(tx_done, dev);
	netif_start_queue(dev);
	return 0;
}

static int spi_net_release(struct net_device *dev)
{
	int ret;
	struct qcspinet *priv;

	SPINET_DBG("%s\n", __func__);

	priv = netdev_priv(dev);
	netif_stop_queue(dev);
	ret = qcspi_mux_fake_close(priv->mux_cid);
	return ret;
}

static int spi_net_tx(struct sk_buff *skb, struct net_device *dev)
{
	int ret;
	struct qcspinet *priv;

	SPINET_DBG("%s: len=%d\n", __func__, skb->len);
	priv = netdev_priv(dev);
	/* todo: handle fraged skb data transfer */
	ret = qcspi_mux_write(priv->mux_cid, skb->data, skb->len);
	if (ret != skb->len) {
		pr_err("%s: write %d bytes to mux channel %d failed!\n",
				__func__, skb->len, priv->mux_cid);
		netif_stop_queue(dev);
		dev->stats.tx_dropped++;
	} else {
		dev->stats.tx_packets++;
		dev->stats.tx_bytes += skb->len;
	}
	dev_kfree_skb(skb);
	return 0;
}

static void spi_net_tx_timeout(struct net_device *dev)
{
	SPINET_DBG("%s\n", __func__);
}

static int spi_net_config(struct net_device *dev, struct ifmap *map)
{
	SPINET_DBG("%s\n", __func__);
	return 0;
}

static int spi_net_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	SPINET_DBG("%s\n", __func__);
	return 0;
}

static const struct net_device_ops spi_netdev_ops = {
	.ndo_open = spi_net_open,
	.ndo_stop = spi_net_release,
	.ndo_start_xmit = spi_net_tx,
	.ndo_tx_timeout = spi_net_tx_timeout,
	.ndo_set_config = spi_net_config,
	.ndo_do_ioctl = spi_net_ioctl,
	.ndo_change_mtu	= eth_change_mtu,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

static void spi_net_create(struct net_device *dev, int n)
{
	struct qcspinet *spi_net;

	SPINET_DBG("%s\n", __func__);
	dev->netdev_ops      = &spi_netdev_ops;
	dev->watchdog_timeo = TX_TIMEOUT;
	spi_net = netdev_priv(dev);
	sprintf(dev->name, "qmi%d", n);
	sprintf(dev->dev_addr, "sspiw%d", n);
	/* for eth_validate_addr, see is_multicast_ether_addr() */
	dev->dev_addr[0] = 0x0;
	memset(spi_net, 0, sizeof(struct qcspinet));
	spi_net->order = n;
	spi_net->net = dev;
	/* hardcode here, later moved to board file?
	   dlci[0] is reserverd, dlci[1-4] for QMI net[1-4]
	   dlci[0x11-0x14] for QMI devices
	*/
	spi_net->mux_cid = n + 1;
	spi_net->qmi.mux_cid = 0x10 + n + 1;
}

static struct net_device *spi_net_register(int n)
{
	int status;
	struct qcspinet	*dev;
	struct net_device *net;

	SPINET_DBG("%s\n", __func__);
	net = alloc_etherdev(sizeof(*dev));
	if (!net) {
		pr_err("Cannot alloc etherdev!\n");
		return NULL;
	}
	spi_net_create(net, n);

	status = register_netdev(net);
	if (status) {
		pr_err("error to register net device\n");
		free_netdev(net);
		return NULL;
	}

	dev = netdev_priv(net);
	dev->qmi.devclass = devclass;
	qcspi_qmi_register(dev);
	return net;
}

static void spi_net_destroy(struct net_device *net)
{
	struct qcspinet	*dev;

	SPINET_DBG("%s\n", __func__);
	dev = netdev_priv(net);
	unregister_netdev(net);
	free_netdev(net);

	qcspi_qmi_deregister(dev);
}

static __init int spi_net_init(void)
{
	int i, err;

	SPINET_DBG("%s\n", __func__);

	err = qcspi_mux_init();
	if (err != 0) {
		pr_err("failed to init QMI mux driver\n");
		return err;
	}

	qcspinet_debug = 0;
	devclass = class_create(THIS_MODULE, "QCSPI_QMI");
	if (IS_ERR(devclass)) {
		pr_err("error at class_create %ld\n", PTR_ERR(devclass));
		return -ENOMEM;
	}

	for (i = 0; i < SPINET_NUM; i++) {
		spinet_devs[i] = spi_net_register(i);
		if (spinet_devs[i] == NULL)
			break;
	}

	if (i < SPINET_NUM) {
		for (; i > 0; i--)
			spi_net_destroy(spinet_devs[i]);
		return 1;
	}

	return 0;
}

static void spi_net_deregister(void)
{
	int i;

	SPINET_DBG("%s\n", __func__);
	for (i = 0; i < SPINET_NUM; i++)
		spi_net_destroy(spinet_devs[i]);
	class_destroy(devclass);
	qcspi_mux_exit();
}

static int qmi_over_spi_probe(struct platform_device *pdev)
{
	SPINET_DBG("qmi_over_spi_probe\n");
	return spi_net_init();
}

static int qmi_over_spi_remove(struct platform_device *pdev)
{
	SPINET_DBG("qmi_over_spi_remove\n");
	spi_net_deregister();
	return 0;
}

static struct platform_driver qmi_over_spi_driver = {
	.probe		= qmi_over_spi_probe,
	.remove		= qmi_over_spi_remove,
	.driver		= {
		.name		= "qmi-over-spi",
		.owner		= THIS_MODULE,
	},
};

static int __init qmi_over_spi_init(void)
{
	SPINET_DBG("qmi_over_spi_init\n");
	return platform_driver_register(&qmi_over_spi_driver);
}

static void __exit qmi_over_spi_exit(void)
{
	SPINET_DBG("qmi_over_spi_exit\n");
	platform_driver_unregister(&qmi_over_spi_driver);
}

module_init(qmi_over_spi_init);
module_exit(qmi_over_spi_exit);
