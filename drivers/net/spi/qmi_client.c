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
#include <linux/list.h>

#include "ts27010_mux.h"
#include "qmi_core.h"
#include "qmi_client.h"
#include "qmi_dev.h"
#include "qmi_net.h"

static struct qmi_client *qmi_client_alloc(void)
{
	struct qmi_client *client;

	client = kmalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		pr_err("%s: no mem\n", __func__);
		return NULL;
	}
	client->cid = (u16)-1;
	client->dev = NULL;
	INIT_LIST_HEAD(&client->node);
	INIT_LIST_HEAD(&client->reads);
	INIT_LIST_HEAD(&client->notifies);
	atomic_set(&client->tid, 0);
	spin_lock_init(&client->lock);
	init_waitqueue_head(&client->read_wait);
	return client;
}

static void qmi_client_free(struct qmi_client *client)
{
	kfree(client);
}

/* Add data into qmi_client->reads list, the data is a full qmux
 * format buffer.
 */
bool qmi_client_add_read(struct qmi_client *client, u16 tid,
		void *data, u16 size)
{
	unsigned long flags;
	struct readreq *req;

	pr_debug("%s: client=%p, tid=%d, data size=%d\n",
			__func__, client, tid, size);
	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req) {
		pr_err("QMI: Cannot alloc memory for read request\n");
		return false;
	}

	req->data = kmalloc(size, GFP_KERNEL);
	if (!req->data) {
		pr_err("QMI: Cannot alloc memory for read buffer\n");
		kfree(req);
		return false;
	}

	memcpy(req->data, data, size);
	req->size = size;
	req->tid = tid;

	spin_lock_irqsave(&client->lock, flags);
	list_add_tail(&req->node, &client->reads);
	spin_unlock_irqrestore(&client->lock, flags);

	return true;
}

/* check client->reads list if got tid transaction data.
 * If tid = 0  means unconditional remove first data from queue,
 * this is used to remove all the reads when cleanup.
 */
bool qmi_client_del_read(struct qmi_client *client, u16 tid,
				void **data, u16 *size)
{
	unsigned long flags;
	bool found = false;
	struct readreq *tmp;

	spin_lock_irqsave(&client->lock, flags);
	list_for_each_entry(tmp, &client->reads, node) {
		if (!tid || tid == tmp->tid) {
			list_del(&tmp->node);
			found = true;
			break;
		}
	}
	spin_unlock_irqrestore(&client->lock, flags);

	if (found) {
		*data = tmp->data;
		*size = tmp->size;
		kfree(tmp);
	}
	return found;
}

/* Add a notify to client->notifies list. A notify is used when
 * desired transaction data receied, our notify function will be
 * called, most time it will be a semaphore increase.
 */
bool qmi_client_add_notify(struct qmi_client *client, u16 tid,
			     void (*hook)(u16 cid, void *),
			     void *data)
{
	unsigned long flags;
	struct notifyreq *notify;

	pr_debug("%s: client=%p, tid=%d\n", __func__, client, tid);
	notify = kmalloc(sizeof(*notify), GFP_KERNEL);
	if (!notify) {
		pr_err("QMI: Cannot alloc memory for notify\n");
		return false;
	}

	notify->func = hook;
	notify->data = data;
	notify->tid = tid;

	spin_lock_irqsave(&client->lock, flags);
	list_add_tail(&notify->node, &client->notifies);
	spin_unlock_irqrestore(&client->lock, flags);

	return true;
}

/* Remove a notify identified by comparing context data.
 */
bool qmi_client_del_notify_by_data(struct qmi_client *client, void *data)
{
	unsigned long flags;
	bool found = false;
	struct notifyreq *notify;

	spin_lock_irqsave(&client->lock, flags);
	list_for_each_entry(notify, &client->notifies, node) {
		if (notify->data == data) {
			found = true;
			list_del(&notify->node);
			break;
		}
	}
	spin_unlock_irqrestore(&client->lock, flags);

	if (found)
		kfree(notify);

	return found;
}

/* Remove a notify identified by tid.
 */
bool qmi_client_del_notify(struct qmi_client *client, u16 tid)
{
	unsigned long flags;
	bool found = false;
	struct notifyreq *notify;

	spin_lock_irqsave(&client->lock, flags);
	list_for_each_entry(notify, &client->notifies, node) {
		if (notify->tid == tid) {
			found = true;
			list_del(&notify->node);
			break;
		}
	}
	spin_unlock_irqrestore(&client->lock, flags);

	if (found)
		kfree(notify);

	return found;
}

/* Call notify callback identified by tid, then remove it from list.
 * tid=0 means unconditional remove first notify from queue, used
 * to get all notify in loop when cleaning up.
 */
bool qmi_client_notify(struct qmi_client *client, u16 tid)
{
	unsigned long flags;
	bool found = false;
	struct notifyreq *notify;

	pr_debug("%s: tid=%d\n", __func__, tid);
	spin_lock_irqsave(&client->lock, flags);
	list_for_each_entry(notify, &client->notifies, node) {
		pr_debug("%s: notify=%p\n", __func__, notify);
		if (!tid || !notify->tid || tid == notify->tid) {
			found = true;
			list_del(&notify->node);
			break;
		}
	}
	spin_unlock_irqrestore(&client->lock, flags);

	if (found && notify->func) {
		notify->func(client->cid, notify->data);
		kfree(notify);
	}

	return found;
}

/* Async read transaction tid data. If data already in reads list, call
 * notify callback directly, otherwise a notify will be added in list.
 */
int qmi_read_async(struct qmi_client *client, u16 tid,
		      void (*hook)(u16 cid, void *),
		      void *data)
{
	unsigned long flags;
	bool found = false;
	struct readreq *readreq;

	pr_debug("%s:client=%p\n", __func__, client);
	spin_lock_irqsave(&client->lock, flags);
	list_for_each_entry(readreq, &client->reads, node) {
		if (!tid || tid == readreq->tid) {
			found = true;
			break;
		}
	}
	spin_unlock_irqrestore(&client->lock, flags);

	if (found)
		hook(client->cid, data);
	else
		qmi_client_add_notify(client, tid, hook, data);

	return 0;
}

void qmi_read_complete(u16 cid, void *data)
{
	pr_debug("%s: cid=0x%04X\n", __func__, cid);
	complete(data);
}

/* Sync read transaction tid data, if data is in reads list,
 * data will be returned, otherwise a notify will be added,
 * then wait for a while for result.
 */
int qmi_read_sync(struct qmi_client *client, void **buf, u16 tid)
{
	int result;
	void *data;
	u16 size;
	DECLARE_COMPLETION_ONSTACK(done);

	while (!qmi_client_del_read(client, tid, &data, &size)) {
		INIT_COMPLETION(done);
		if (!qmi_client_add_notify(
			client, tid, qmi_read_complete, &done)) {
			pr_err("unable to register for notification\n");
			return -EFAULT;
		}

		result = wait_for_completion_timeout(
			&done, msecs_to_jiffies(5000));
		if (result == 0) {
			pr_err("Timeout to read sync\n");
			qmi_client_del_notify(client, tid);
			return -EINTR;
		}
	}

	*buf = data;
	return size;
}

/* write buf to dev, buf is qmux formated. */
int qmi_write_sync(struct qmidev *dev, char *buf, int size, u16 cid)
{
	int result;

	result = s_qmux_fill(cid, buf, size);
	if (result < 0)
		return result;

	if (qcspinet_debug)
		print_hex_dump(KERN_INFO,  "QCSPINET: ", DUMP_PREFIX_OFFSET,
		       16, 1, buf, size, true);

	result = qcspi_mux_write(dev->mux_cid, buf, size);
	if (result < 0)	{
		pr_err("write data to mux %d error %d\n", dev->mux_cid, result);
		return result;
	}

	return result;
}

/* add a client to clients list */
static bool qmi_client_add(struct qmi_clients *clients,
		struct qmi_client *client)
{
	unsigned long flags;

	spin_lock_irqsave(&clients->lock, flags);
	list_add_tail(&client->node, &clients->header);
	spin_unlock_irqrestore(&clients->lock, flags);

	return true;
}

static void qmi_client_dump(struct qmi_clients *clients)
{
	unsigned long flags;
	struct qmi_client *client;

	pr_debug("%s: clients=%p\n", __func__, clients);
	spin_lock_irqsave(&clients->lock, flags);
	list_for_each_entry(client, &clients->header, node) {
		pr_debug("client=%p, cid=%d\n", client, client->cid);
	}
	spin_unlock_irqrestore(&clients->lock, flags);
}

/* Reteive a client by cid from clients */
struct qmi_client *qmi_client_by_cid(struct qmi_clients *clients, u16 cid)
{
	unsigned long flags;
	bool found = false;
	struct qmi_client *client;

	spin_lock_irqsave(&clients->lock, flags);
	list_for_each_entry(client, &clients->header, node) {
		if (client->cid == cid) {
			found = true;
			break;
		}
	}
	spin_unlock_irqrestore(&clients->lock, flags);

	if (found)
		return client;
	else
		return NULL;
}

/* Remove a client by cid from clients */
static bool qmi_client_del(struct qmi_clients *clients, u16 cid)
{
	unsigned long flags;
	bool found = false;
	struct qmi_client *client = NULL;

	spin_lock_irqsave(&clients->lock, flags);
	list_for_each_entry(client, &clients->header, node) {
		if (client->cid == cid) {
			found = true;
			list_del(&client->node);
			break;
		}
	}
	spin_unlock_irqrestore(&clients->lock, flags);

	return found;
}

/* Create a QMI client by sending request to modem.
 */
struct qmi_client *qmi_client_create(struct qmidev *dev, u8 type)
{
	u16 cid;
	struct qmi_client *client;
	struct qmi_client *ctl;
	int result;
	void *wbuf;
	size_t wbufsize;
	void *rbuf;
	u16 rbufsize;
	u8 tid;

	pr_debug("%s:qmi=%p, type=%d\n", __func__, dev, type);
	client = qmi_client_alloc();
	if (!client)
		goto FAILED_NO_MEM;

	pr_debug("%s: client=%p\n", __func__, client);
	if (type) {
		ctl = qmi_client_by_cid(&dev->clients, 0);
		tid = atomic_add_return(1, &ctl->tid);
		wbuf = s_qmictl_new_getcid(tid, type, &wbufsize);
		if (!wbuf) {
			pr_err("QMI: cannot alloc memory for getcid request\n");
			goto FAILED_ALLOC_CLIENT;
		}

		result = qmi_write_sync(dev, wbuf, wbufsize, QMICTL);
		kfree(wbuf);
		if (result < 0) {
			pr_err("QMI: write failed %d\n", result);
			goto FAILED_ALLOC_CLIENT;
		}

		result = qmi_read_sync(ctl, &rbuf, tid);
		if (result < 0) {
			pr_err("QMI: read failed %d\n", result);
			goto FAILED_ALLOC_CLIENT;
		}
		rbufsize = result;

		result = s_qmictl_alloccid_resp(rbuf, rbufsize, &cid);
		kfree(rbuf);
		if (result < 0) {
			pr_err("bad alloccid response %d\n", result);
			goto FAILED_ALLOC_CLIENT;
		}
	} else {
		cid = 0;
	}

	pr_debug("client created: qmi dev=%p, cid=%d\n", dev, cid);
	if (qmi_client_by_cid(&dev->clients, cid)) {
		pr_err("Client memory already exists\n");
		kfree(client);
		return NULL;
	}

	client->cid = cid;
	client->dev = dev;
	qmi_client_add(&dev->clients, client);
	return client;

FAILED_ALLOC_CLIENT:
	qmi_client_free(client);
FAILED_NO_MEM:
	return NULL;
}

void qmi_client_release(struct qmidev *dev, u16 cid)
{
	int result;
	struct qmi_client *client;
	struct qmi_client *ctl;
	void *data;
	u16 size;
	void *wbuf;
	size_t wbufsize;
	void *rbuf;
	u16 rbufsize;
	u8 tid;

	pr_debug("releasing 0x%04X\n", cid);

	if (cid != QMICTL) {
		ctl = qmi_client_by_cid(&dev->clients, 0);
		tid = atomic_add_return(1, &ctl->tid);
		wbuf = s_qmictl_new_releasecid(tid, cid, &wbufsize);
		if (!wbuf) {
			pr_err("memory error\n");
			goto del_client;
		}

		result = qmi_write_sync(dev, wbuf, wbufsize, QMICTL);
		kfree(wbuf);
		if (result < 0) {
			pr_err("bad write status %d\n", result);
			goto del_client;
		}

		result = qmi_read_sync(ctl, &rbuf, tid);
		if (result < 0) {
			pr_err("bad read status %d\n", result);
			goto del_client;
		}

		rbufsize = result;
		result = s_qmictl_freecid_resp(rbuf, rbufsize);
		kfree(rbuf);
		if (result < 0) {
			pr_err("error %d parsing response\n", result);
			goto del_client;
		}
	}

del_client:
	client = qmi_client_by_cid(&dev->clients, cid);
	if (!client)
		return;

	qmi_client_del(&dev->clients, cid);

	while (qmi_client_notify(client, 0))
		;

	if (client->cid)
		wake_up(&client->read_wait);

	while (qmi_client_del_read(client, 0, &data, &size))
		kfree(data);

	qmi_client_free(client);
}

/* Dispatch received data to QMI clients, data is qmux formated */
void qmi_clients_recv_data(struct qmi_clients *clients, u16 cid,
		u16 tid, void *data, u16 size)
{
	unsigned long flags;
	struct qmi_client *client;
	bool is_broadcast = false;


	pr_debug("%s: cid=%d, tid=%d, data size=%d\n", __func__,
			cid, tid, size);
	spin_lock_irqsave(&clients->lock, flags);
	list_for_each_entry(client, &clients->header, node) {
		pr_debug("%s: client=%p, client->cid=%d\n",
				__func__, client, client->cid);
		/* if got a broadcast msg, means qmicid == 0xFF, send to
		 * all the same service type clients
		 * */
		is_broadcast = ((client->cid | 0xFF00) == cid) ? true : false;
		if (client->cid == cid || is_broadcast) {
			spin_unlock_irqrestore(&clients->lock, flags);
			qmi_client_add_read(client, tid, data, size);
			if (client->cid)
				wake_up_interruptible(&client->read_wait);
			qmi_client_notify(client, tid);
			spin_lock_irqsave(&clients->lock, flags);
			if (is_broadcast == false)
				break;
		}
	}
	spin_unlock_irqrestore(&clients->lock, flags);
}

