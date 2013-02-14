#include <linux/module.h>
#include <linux/types.h>
#include <linux/tty.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "ts27010_mux.h"
#include "ts27010_ringbuf.h"
#include "qmi_net.h"

#define SPILDISC_DBG(x...)  pr_debug(x)

struct qcspi_ldisc_data {
	struct ts27010_ringbuf		*rbuf;
	struct work_struct		recv_work;
	spinlock_t			recv_lock; /* */
	spinlock_t			send_lock; /* */
	struct workqueue_struct		*mux_parse_wq;
	wait_queue_head_t		mux_parse_wait;
	bool				rbuf_full;
};

struct qcspi_ldisc_write_done_notifies write_done_notifies;

static void qcspi_ldisc_write_done_notify_init(void)
{
	spin_lock_init(&write_done_notifies.lock);
	INIT_LIST_HEAD(&write_done_notifies.header);
}

struct notify_item *qcspi_ldisc_add_write_done_notify(notifier func, void *data)
{
	unsigned long flags;
	struct notify_item *item;

	SPILDISC_DBG("%s\n", __func__);
	item = kmalloc(sizeof(struct notify_item), GFP_KERNEL);
	if (item == NULL) {
		pr_err("%s: no memory\n", __func__);
		return NULL;
	}
	INIT_LIST_HEAD(&item->node);
	item->notify = func;
	item->data = data;
	spin_lock_irqsave(&write_done_notifies.lock, flags);
	list_add_tail(&item->node, &write_done_notifies.header);
	spin_unlock_irqrestore(&write_done_notifies.lock, flags);
	return item;
}

static void qcspi_ldisc_write_done_notify(void)
{
	unsigned long flags;
	struct notify_item *item;

	SPILDISC_DBG("%s\n", __func__);
	spin_lock_irqsave(&write_done_notifies.lock, flags);
	list_for_each_entry(item, &write_done_notifies.header, node) {
		if (item->notify) {
			spin_unlock_irqrestore(
				&write_done_notifies.lock, flags);
			item->notify(item->data);
			spin_lock_irqsave(&write_done_notifies.lock, flags);
		}
	}
	spin_unlock_irqrestore(&write_done_notifies.lock, flags);
}

static void qcspi_ldisc_recv_worker(struct work_struct *work)
{
	struct qcspi_ldisc_data *ts =
		container_of(work, struct qcspi_ldisc_data, recv_work);

	qcspi_mux_recv(ts->rbuf);
	ts->rbuf_full = false;
	wake_up_interruptible(&ts->mux_parse_wait);
}

int qcspi_ldisc_send(struct tty_struct *tty, u8 *data, int len)
{
	unsigned long flags;
	int ret;
	struct qcspi_ldisc_data *ts = tty->disc_data;

	spin_lock_irqsave(&ts->send_lock, flags);
	if (tty->driver->ops->write_room(tty) < len) {
		ret = 0;
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
		pr_err("QCSPI LDISC: no room to write\n");
	} else {
		ret = tty->driver->ops->write(tty, data, len);
		if (ret < len) {
			set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
			pr_err("QCSPI LDISC: write overflow\n");
		}
	}
	spin_unlock_irqrestore(&ts->send_lock, flags);
	return ret;
}

static int qcspi_ldisc_open(struct tty_struct *tty)
{
	struct qcspi_ldisc_data *ts;
	int err;

	SPILDISC_DBG("%s\n", __func__);
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		err = -ENOMEM;
		goto err0;
	}

	ts->rbuf = ts27010_ringbuf_alloc(LDISC_BUFFER_SIZE);
	if (ts->rbuf == NULL) {
		err = -ENOMEM;
		goto err1;
	}

	spin_lock_init(&ts->send_lock);
	spin_lock_init(&ts->recv_lock);
	init_waitqueue_head(&ts->mux_parse_wait);
	tty->disc_data = ts;
	qcspi_mux_tty = tty;
	INIT_WORK(&ts->recv_work, qcspi_ldisc_recv_worker);
	ts->mux_parse_wq = create_singlethread_workqueue("mux_parse_wq");
	if (!ts->mux_parse_wq) {
		pr_err("QCSPI LDISC: failed to create workqueue\n");
		err = -ENOMEM;
		goto err1;
	}
	SPILDISC_DBG("%s done\n", __func__);
	return 0;

err1:
	kfree(ts);
err0:
	return err;
}

static void qcspi_ldisc_close(struct tty_struct *tty)
{
	struct qcspi_ldisc_data *ts = tty->disc_data;

	SPILDISC_DBG("%s\n", __func__);
	flush_workqueue(ts->mux_parse_wq);
	destroy_workqueue(ts->mux_parse_wq);
	qcspi_mux_tty = NULL;
	ts27010_ringbuf_free(ts->rbuf);
	kfree(ts);
}

static int qcspi_ldisc_hangup(struct tty_struct *tty)
{
	qcspi_ldisc_close(tty);
	return 0;
}

static ssize_t qcspi_ldisc_read(struct tty_struct *tty, struct file *file,
				   unsigned char __user *buf, size_t count)
{
	return -EIO;
}

static ssize_t qcspi_ldisc_write(struct tty_struct *tty, struct file *file,
				   const unsigned char *buf, size_t count)
{
	return -EIO;
}

static int qcspi_ldisc_ioctl(struct tty_struct *tty, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int err;

	switch (cmd) {
	default:
		err = tty_mode_ioctl(tty, file, cmd, arg);
	}

	return err;
}

static unsigned int qcspi_ldisc_poll(struct tty_struct *tty,
				       struct file *file,
				       poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &tty->read_wait, wait);
	poll_wait(file, &tty->write_wait, wait);
	if (tty_hung_up_p(file))
		mask |= POLLHUP;
	if (!tty_is_writelocked(tty) && tty_write_room(tty) > 0)
		mask |= POLLOUT | POLLWRNORM;

	return mask;
}

static void qcspi_ldisc_receive(struct tty_struct *tty,
				  const unsigned char *data,
				  char *cflags, int count)
{
	struct qcspi_ldisc_data *ts = tty->disc_data;
	int n;
	unsigned long flags;

	SPILDISC_DBG(KERN_ERR "%s:count=%d\n", __func__, count);
	WARN_ON(count == 0);

	while (1) {
		spin_lock_irqsave(&ts->recv_lock, flags);
		if (ts27010_ringbuf_room(ts->rbuf) < count) {
			ts->rbuf_full = true;
			spin_unlock_irqrestore(&ts->recv_lock, flags);
			pr_err("%s: no room, wait for next retry.\n",
					__func__);
			wait_event_interruptible_timeout(ts->mux_parse_wait,
					ts->rbuf_full != true,
					msecs_to_jiffies(10));
			continue;
		}

		n = ts27010_ringbuf_write(ts->rbuf, data, count);
		spin_unlock_irqrestore(&ts->recv_lock, flags);

		if (n < count)
			pr_err("%s: buffer overrun. dropping data.\n",
					__func__);
		queue_work(ts->mux_parse_wq, &ts->recv_work);
		break;
	}
}

static void qcspi_ldisc_wakeup(struct tty_struct *tty)
{
	SPILDISC_DBG("%s\n", __func__);
	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	qcspi_ldisc_write_done_notify();
}

static struct tty_ldisc_ops qcspi_ldisc = {
	.owner  = THIS_MODULE,
	.magic	= TTY_LDISC_MAGIC,
	.name	= "n_qmi",
	.open	= qcspi_ldisc_open,
	.close	= qcspi_ldisc_close,
	.hangup	= qcspi_ldisc_hangup,
	.read	= qcspi_ldisc_read,
	.write	= qcspi_ldisc_write,
	.ioctl	= qcspi_ldisc_ioctl,
	.poll	= qcspi_ldisc_poll,
	.receive_buf = qcspi_ldisc_receive,
	.write_wakeup = qcspi_ldisc_wakeup,
};

int qcspi_ldisc_init(void)
{
	int err;

	qcspi_ldisc_write_done_notify_init();
	err = tty_register_ldisc(N_QMI, &qcspi_ldisc);
	if (err < 0)
		pr_err("qcspi: unable to register line discipline\n");

	return err;
}

void qcspi_ldisc_remove(void)
{
	tty_unregister_ldisc(N_QMI);
}


