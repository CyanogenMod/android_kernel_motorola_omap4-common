#include <linux/types.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/netdevice.h>

#include "ts27010_mux.h"
#include "qmi_core.h"
#include "qmi_client.h"
#include "qmi_dev.h"
#include "qmi_net.h"

static int qcspinet2k_fwdelay;

#define IOCTL_QMI_GET_SERVICE_FILE	(0x8BE0 + 1)
#define IOCTL_QMI_GET_DEVICE_VIDPID	(0x8BE0 + 2)
#define IOCTL_QMI_GET_DEVICE_MEID	(0x8BE0 + 3)
#define IOCTL_QMI_CLOSE			(0x8BE0 + 4)

#define QMI_READY_WAIT_TIME 2000

#define QMIDEV_DBG(x...)  pr_debug(x)

static int devqmi_recv(int line, void *obj, int size, void *ctx)
{
	int result;
	u16 cid;
	struct qmidev *dev;
	u16 tid;
	unsigned char *data;

	QMIDEV_DBG("%s: Read %d bytes\n", __func__, size);
	data = (unsigned char *)obj;
	dev = ctx;

	if (qcspinet_debug)
		print_hex_dump(KERN_INFO, "QCSPINet2k: ", DUMP_PREFIX_OFFSET,
		       16, 1, data, size, true);

	/* todo: handle multiple qmux packets at one time */
	result = s_qmux_parse(&cid, data, size);
	if (result < 0) {
		pr_err("Read error parsing QMUX %d\n", result);
		kfree(data);
		return size;
	}

	if (size < result + 3) {
		pr_err("Data buffer too small to parse\n");
		kfree(data);
		return size;
	}

	if (cid == QMICTL)
		tid = *(u8 *)(data + result + 1);
	else
		tid = *(u16 *)(data + result + 1);

	QMIDEV_DBG("%s: cid=%d, tid=%d\n", __func__, cid, tid);
	qmi_clients_recv_data(&dev->clients, cid, tid, data, size);

	kfree(data);

	return size;
}

static bool qmi_ready(struct qmidev *dev, u16 timeout)
{
	int result;
	void *wbuf = NULL;
	size_t wbufsize;
	void *rbuf;
	u16 rbufsize;
	u16 now;
	struct qmi_client *ctl;
	u8 tid;
	DECLARE_COMPLETION_ONSTACK(done);

	ctl = qmi_client_by_cid(&dev->clients, 0);
	if (!ctl)
		return false;

	for (now = 0; now < timeout; now += QMI_READY_WAIT_TIME) {
		INIT_COMPLETION(done);
		tid = atomic_add_return(1, &ctl->tid);
		kfree(wbuf);
		wbuf = s_qmictl_new_ready(tid, &wbufsize);
		if (!wbuf)
			return false;

		result = qmi_read_async(ctl, tid, qmi_read_complete, &done);
		if (result) {
			kfree(wbuf);
			return false;
		}

		qmi_write_sync(dev, wbuf, wbufsize, QMICTL);

		result = wait_for_completion_timeout(
			&done, msecs_to_jiffies(QMI_READY_WAIT_TIME));
		if (result != 0) {
			if (qmi_client_del_read(ctl, tid, &rbuf, &rbufsize)) {
				kfree(rbuf);
				break;
			}
		} else {
			qmi_client_notify(ctl, tid);
		}
	}

	kfree(wbuf);

	if (now >= timeout)
		return false;

	pr_info("QMI Ready after %u milliseconds\n", now);

	/* 3580 and newer doesn't need a delay; older needs 5000ms */
	if (qcspinet2k_fwdelay)
		msleep(qcspinet2k_fwdelay * 1000);

	return true;
}

static bool qmidms_getmeid(struct qmidev *dev)
{
	int result;
	void *wbuf;
	size_t wbufsize;
	void *rbuf;
	u16 rbufsize;
	u16 cid;
	struct qmi_client *client;

	client = qmi_client_create(dev, QMIDMS);
	if (!client)
		return false;
	cid = client->cid;

	wbuf = s_qmidms_new_getmeid(1, &wbufsize);
	if (!wbuf)
		return false;

	result = qmi_write_sync(dev, wbuf, wbufsize, cid);
	kfree(wbuf);

	if (result < 0)
		return false;

	result = qmi_read_sync(client, &rbuf, 1);
	if (result < 0)
		return false;
	rbufsize = result;

	result = s_qmidms_meid_resp(rbuf, rbufsize, &dev->meid[0], 14);
	kfree(rbuf);
	qmi_client_release(dev, cid);

	if (result < 0) {
		pr_err("bad get MEID resp\n");
		memset(&dev->meid[0], '0', 14);
	} else {
		if (qcspinet_debug)
			print_hex_dump(KERN_ERR, "Get MEID resp: ",
				DUMP_PREFIX_OFFSET,
			       16, 1, dev->meid, 14, true);
	}

	return true;
}

static int devqmi_open(struct inode *inode, struct file *file)
{
	int ret;
	struct qmi_handler *handle;
	struct qcspinet *dev;

	QMIDEV_DBG("%s\n", __func__);
	dev = cdev_to_qcspinet(inode->i_cdev);
	if (!dev)
		return -ENXIO;

	ret = qcspi_mux_fake_open(dev->qmi.mux_cid,
			devqmi_recv, NULL, &dev->qmi);
	if (ret) {
		pr_err("Failed to open mux 0!\n");
		return ret;
	}

	if (!qmi_ready(&dev->qmi, QMI_READY_WAIT_TIME*2)) {
		pr_err("Device unresponsive to QMI\n");
		qcspi_mux_fake_close(dev->qmi.mux_cid);
		return -ETIMEDOUT;
	}

	ret = qmidms_getmeid(&dev->qmi);
	if (ret == false) {
		pr_err("QMI: Cannot get meid\n");
		qcspi_mux_fake_close(dev->qmi.mux_cid);
		return -ENXIO;
	}


	handle = kmalloc(sizeof(*handle), GFP_KERNEL);
	if (handle == NULL) {
		qcspi_mux_fake_close(dev->qmi.mux_cid);
		pr_err("QMI: Cannot alloc memory for QMI handler\n");
		return -ENOMEM;
	}
	handle->cid = (u16)-1;
	handle->dev = &dev->qmi;
	file->private_data = handle;

	return 0;
}

static long devqmi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int result = 0;
	struct qmi_client *client = NULL;
	struct qmi_handler *handle = (struct qmi_handler *)file->private_data;

	QMIDEV_DBG("%s\n", __func__);

	switch (cmd) {
	case IOCTL_QMI_GET_SERVICE_FILE:

		QMIDEV_DBG("Setting up QMI for service %lu\n", arg);
		if (!(u8)arg) {
			pr_err("QMI: Cannot use QMICTL from userspace\n");
			return -EINVAL;
		}

		if (handle->cid != (u16)-1) {
			pr_err("QMI: Close connection before opening\n");
			return -EBADR;
		}

		client = qmi_client_create(handle->dev, (u8)arg);
		if (client == NULL)
			return -EBUSY;

		handle->cid = client->cid;
		return 0;

	case IOCTL_QMI_CLOSE:
		QMIDEV_DBG("Tearing down QMI for service %lu", arg);
		if (handle->cid != (u16)-1) {
			qmi_client_release(handle->dev, handle->cid);
			handle->cid = (u16)-1;
		}
		return 0;

	case IOCTL_QMI_GET_DEVICE_VIDPID:
		return -ENXIO;

	case IOCTL_QMI_GET_DEVICE_MEID:
		if (!arg) {
			pr_err("QMI: Bad MEID buffer\n");
			return -EINVAL;
		}

		result = copy_to_user((unsigned int *)arg,
				&client->dev->meid[0], 14);
		if (result)
			pr_err("QMI: Copy MEID to userspace failure\n");

		return result;

	default:
		return -EBADRQC;
	}
}

static int devqmi_release(struct inode *inode, struct file *file)
{
	int ret;
	struct qmi_handler *handle = (struct qmi_handler *)file->private_data;

	QMIDEV_DBG("%s\n", __func__);
	if (handle->cid != (u16)-1)
		qmi_client_release(handle->dev, handle->cid);
	ret = qcspi_mux_fake_close(handle->dev->mux_cid);
	kfree(handle);
	return ret;
}

static ssize_t devqmi_read(struct file *file, char __user *buf, size_t size,
			   loff_t *pos)
{
	int result;
	void *data = NULL;
	void *smalldata;
	struct qmi_client *client;
	struct qmi_handler *handle = (struct qmi_handler *)file->private_data;

	QMIDEV_DBG("%s\n", __func__);

	if (handle->cid == (u16)-1) {
		pr_err("Client ID must be set by ioctl before reading\n");
		return -EBADR;
	}

	client = qmi_client_by_cid(&(handle->dev->clients), handle->cid);
	result = qmi_read_sync(client, &data, 0);
	if (result <= 0)
		return result;

	result -= QMUX_HEADER_SIZE;
	smalldata = data + QMUX_HEADER_SIZE;

	if (result > size) {
		pr_err("Read data is too large!\n");
		kfree(data);
		return -EOVERFLOW;
	}

	if (copy_to_user(buf, smalldata, result)) {
		pr_err("Error copying read data to user\n");
		result = -EFAULT;
	}

	kfree(data);
	return result;
}

static ssize_t devqmi_write(struct file *file, const char __user * buf,
			    size_t size, loff_t *pos)
{
	int status;
	void *wbuf;
	struct qmi_client *client;
	struct qmi_handler *handle = (struct qmi_handler *)file->private_data;

	QMIDEV_DBG("%s\n", __func__);

	if (handle->cid == (u16)-1) {
		pr_err("Client ID must be set by ioctl before reading\n");
		return -EBADR;
	}

	client = qmi_client_by_cid(&(handle->dev->clients), handle->cid);
	wbuf = kmalloc(size + QMUX_HEADER_SIZE, GFP_KERNEL);
	if (!wbuf)
		return -ENOMEM;
	status = copy_from_user(wbuf + QMUX_HEADER_SIZE, buf, size);
	if (status) {
		pr_err("Unable to copy data from userspace %d\n", status);
		kfree(wbuf);
		return status;
	}

	status = qmi_write_sync(client->dev, wbuf,
			size + QMUX_HEADER_SIZE, client->cid);

	kfree(wbuf);
	if (status == size + QMUX_HEADER_SIZE)
		return size;

	return status;
}

static unsigned int devqmi_poll(struct file *file, poll_table *wait)
{
	struct qmi_client *client;
	struct qmi_handler *handle = (struct qmi_handler *)file->private_data;
	unsigned int mask = 0;

	if (handle->cid == (u16)-1) {
		pr_err("Client ID must be set by ioctl before polling\n");
		return -EBADR;
	}

	client = qmi_client_by_cid(&(handle->dev->clients), handle->cid);
	poll_wait(file, &client->read_wait, wait);

	if (!list_empty(&client->reads))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static const struct file_operations devqmi_fops = {
	.owner = THIS_MODULE,
	.read  = devqmi_read,
	.write = devqmi_write,
	.unlocked_ioctl = devqmi_ioctl,
	.open  = devqmi_open,
	.release = devqmi_release,
	.poll  = devqmi_poll,
};

static int qcspi_qmi_create(struct qmidev *qmi, int minor)
{
	int result = -ENXIO;
	dev_t devno;

	QMIDEV_DBG("%s: minor=%d, qmi=%p\n", __func__, minor, qmi);

	result = alloc_chrdev_region(&devno, 0, 1, "qcqmi");
	if (result < 0)
		goto fail_qmi;

	qmi->cdev = cdev_alloc();
	qmi->cdev->owner = THIS_MODULE;
	qmi->cdev->ops = &devqmi_fops;

	result = cdev_add(qmi->cdev, devno, 1);
	if (result) {
		pr_err("error adding cdev\n");
		goto fail_cdev;
	}
	QMIDEV_DBG("creating qcqmi%d\n", minor);
	device_create(qmi->devclass, NULL, devno, NULL, "qcqmi%d", minor);
	qmi->devnum = devno;
	return 0;

fail_cdev:
	unregister_chrdev_region(devno, 1);
fail_qmi:
	return result;
}

int qcspi_qmi_register(struct qcspinet *dev)
{
	int result;
	unsigned long qmiidx = 0;
	char *name;
	struct qmi_client *client;

	name = strstr(dev->net->name, "qmi");
	if (!name) {
		pr_err("Bad net name: %s\n", dev->net->name);
		result = -ENXIO;
		goto fail_name;
	}
	name += strlen("qmi");
	result = strict_strtoul(name, 10, &qmiidx);
	if (result != 0) {
		pr_err("Bad minor number\n");
		result = -ENXIO;
		goto fail_name;
	}

	INIT_LIST_HEAD(&dev->qmi.clients.header);
	spin_lock_init(&dev->qmi.clients.lock);

	client = qmi_client_create(&dev->qmi, QMICTL);
	if (!client) {
		pr_err("%s:failed to create CTL client\n", __func__);
		return -ENOMEM;
	}

	result = qcspi_qmi_create(&dev->qmi, qmiidx);
fail_name:
	return result;
}

void qcspi_qmi_deregister(struct qcspinet *dev)
{
	QMIDEV_DBG("%s\n", __func__);
	if (!IS_ERR(dev->qmi.devclass))
		device_destroy(dev->qmi.devclass, dev->qmi.devnum);

	cdev_del(dev->qmi.cdev);
	unregister_chrdev_region(dev->qmi.devnum, 1);
}

module_param(qcspinet2k_fwdelay, int, S_IRUGO | S_IWUSR);
