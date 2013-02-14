/*
 * Copyright (C) 2007-2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "ts27010.h"
#include "ts27010_mux.h"
#include "ts27010_ringbuf.h"

#ifndef USB_QUEUE_RECV
/*/ #define USB_QUEUE_RECV /*/
#endif

struct ts27010_tty_channel_data {
	atomic_t ref_count;
	struct tty_struct *tty;
#ifdef USB_QUEUE_RECV
	struct mutex lock; /* channel r/w lock */
#endif
#ifdef PROC_DEBUG_MUX_STAT
	int sent_size;
	int recv_size;
	int back_size;
#endif
};

struct ts27010_tty_data {
	struct ts27010_tty_channel_data		chan[TS0710_MAX_MUX];
};

struct tty_struct *ts27010_usb_tty_table[TS0710_MAX_MUX];

static struct tty_driver *driver;

#define TS0710MUX_MAJOR 236
#define TS0710MUX_MINOR_START 0

#ifdef PROC_DEBUG_MUX_STAT
static struct ts27010_tty_data *s_td;
static int s_nWriteTotal;
static int s_nWriteProt;
static int s_nWriteCount;
static int s_nReadTotal;
static int s_nRecvCount;
static int s_nBackCount;

void ts27010_tty_usb_dump_io_clear(void)
{
	int i;
	for (i = 0; i < TS0710_MAX_MUX; i++) {
		s_td->chan[i].sent_size = 0;
		s_td->chan[i].recv_size = 0;
		s_td->chan[i].back_size = 0;
	}
	s_nWriteTotal = 0;
	s_nWriteProt = 0;
	s_nWriteCount = 0;
	s_nReadTotal = 0;
	s_nRecvCount = 0;
	s_nBackCount = 0;
}

void ts27010_tty_usb_dump_io(void)
{
	int i;
	for (i = 0; i < TS0710_MAX_MUX; i++) {
		mux_print(MSG_ERROR,
			"(%d) refcount: %d, sent: %d, recv: %d, back: %d\n",
			i, atomic_read(&s_td->chan[i].ref_count),
			s_td->chan[i].sent_size, s_td->chan[i].recv_size,
			s_td->chan[i].back_size);
	}
	mux_print(MSG_ERROR, "Total writen size: %d, prot: %d, "
		"total read size: %d\n",
		s_nWriteTotal, s_nWriteProt, s_nReadTotal);
	mux_print(MSG_ERROR, "Total writen count: %d, recved: %d, back: %d\n",
		s_nWriteCount, s_nRecvCount, s_nBackCount);
}
#endif /* PROC_DEBUG_MUX_STAT */

static int tty_push(struct tty_struct *tty, u8 *buf, int len)
{
	int sent = 0;
	FUNC_ENTER();
	while (sent < len) {
		int count = tty_insert_flip_string(
			tty, buf + sent, len - sent);
		if (count != (len - sent)) {
			mux_print(MSG_WARNING,
				"written size %d is not wanted %d\n",
				count, len - sent);
		}
		sent += count;
		/* tty_flip_buffer_push(tty); */
	}
	tty_flip_buffer_push(tty);
	FUNC_EXIT();
	return sent;
}

#ifdef USB_QUEUE_RECV
#ifdef USE_THREAD
typedef int (*THREAD_T)(void *data);
#else
typedef void (*WORKER_T)(struct work_struct *work);
#endif

struct mux_data_list_t {
	struct list_head list;
	struct ts27010_tty_data *td;
	int line;
	int size;
	u8 *body;
};

struct mux_recver_t {
	spinlock_t m_locker; /* recver list r/w lock */
	struct list_head m_datalist;
	const char *m_name;

#ifdef USE_THREAD
	/*
	long m_pid;
	struct completion m_exited;
	*/
	struct task_struct *m_tsk;
	struct semaphore m_sem;
#else
	struct delayed_work m_recv_work;
#endif
};
static struct mux_recver_t *s_mux_usb_recver;

static int mux_user_recv(struct mux_recver_t *recver)
{
	struct mux_data_list_t *entry;
	struct list_head *ptr;
	struct tty_struct *tty;
	unsigned long flags;

	FUNC_ENTER();

	spin_lock_irqsave(&recver->m_locker, flags);
	while (!(list_empty(&recver->m_datalist))) {
		ptr = recver->m_datalist.next;
		entry = list_entry(ptr, struct mux_data_list_t, list);
		spin_unlock_irqrestore(&recver->m_locker, flags);

		tty = entry->td->chan[entry->line].tty;
		WARN_ON(tty != ts27010_usb_tty_table[entry->line]);

		if (tty) {
			int sent = 0;
			mutex_lock(&entry->td->chan[entry->line].lock);
			while (sent < entry->size) {
				int count;
				mutex_lock(&tty->termios_mutex);
				if (test_bit(TTY_THROTTLED, &tty->flags)
					|| tty->receive_room < entry->size) {
					mutex_unlock(&tty->termios_mutex);
					break;
				}
				mutex_unlock(&tty->termios_mutex);
				count = tty_insert_flip_string(
					tty, entry->body + sent,
					entry->size - sent);
				if (count != (len - sent)) {
					mux_print(MSG_WARNING,
						"written size %d "
						"is not wanted %d\n",
						count, entry->size - sent);
				}
				sent += count;
				tty_flip_buffer_push(tty);
			}
			mutex_unlock(&entry->td->chan[entry->line].lock);
			if (sent < entry->size) {
				mux_print(MSG_INFO,
					"!!!/dev/usb%d yet no room recv, "
					"recv_room: %d, len: %d\n",
					entry->line + TS27010MUX_NAME_BASE,
					tty->receive_room, entry->size);
#ifdef QUEUE_SELF
				queue_delayed_work(g_mux_usb_queue,
					&recver->m_recv_work,
					msecs_to_jiffies(20));
#else
				schedule_delayed_work(&recver->m_recv_work,
					msecs_to_jiffies(20));
#endif
				return 0;
			}
			mux_print(MSG_DEBUG,
				"%d data sent to user space by %s\n",
				entry->size, recver->m_name);
		} else {
			mux_print(MSG_WARNING,
				"/dev/usb%d no tty waiting for data, "
				"discard data %d\n",
				entry->line + TS27010MUX_NAME_BASE,
				entry->size);
		}

		spin_lock_irqsave(&recver->m_locker, flags);
#ifdef PROC_DEBUG_MUX_STAT
		entry->td->chan[entry->line].back_size += entry->size;
		s_nBackCount++;
#endif
		list_del(ptr);
		kfree(entry->body);
		kfree(entry);
	}
	spin_unlock_irqrestore(&recver->m_locker, flags);

	FUNC_EXIT();

	return 0;
}

#ifdef USE_THREAD
static int mux_usb_recv_thread(void *data)
{
	struct mux_recver_t *recver = (struct mux_recver_t *)data;

	DAEMONIZE(recver->m_name);

	while (down_interruptible(&recver->m_sem) == 0)
		mux_user_recv(recver);

	mux_print(MSG_INFO, "thread %s exit\n", recver->m_name);
	/*
	complete_and_exit(&recver->m_exited, 0);
	*/
}
#else /* USE_THREAD */
static unsigned long s_mux_queuerecv_flags;
#define QUEUERECV_RUNNING 0

static void mux_usb_recv_worker(struct work_struct *work)
{
	struct mux_recver_t *recver =
		container_of(work, struct mux_recver_t, m_recv_work.work);

	FUNC_ENTER();

	if (test_and_set_bit(QUEUERECV_RUNNING, &s_mux_queuerecv_flags)) {
		mux_print(MSG_WARNING, "a queuerecv_work is running\n");
#ifdef QUEUE_SELF
		queue_delayed_work(g_mux_usb_queue, &recver->m_recv_work,
			msecs_to_jiffies(20));
#else
		schedule_delayed_work(
			&recver->m_recv_work, msecs_to_jiffies(20));
#endif
		return;
	}
	mux_user_recv(recver);
	clear_bit(QUEUERECV_RUNNING, &s_mux_queuerecv_flags);
	FUNC_EXIT();
}
#endif /* USE_THREAD */

static struct mux_recver_t *mux_alloc_recver(
	void *mux_recv_func, const char *name)
{
	struct mux_recver_t *recver;
	FUNC_ENTER();

	recver = (struct mux_recver_t *)kmalloc(sizeof(*recver), GFP_KERNEL);
	if (recver == NULL) {
		mux_print(MSG_ERROR, "alloc mux_recver_t failed\n");
		goto out;
	}

	INIT_LIST_HEAD(&recver->m_datalist);
	recver->m_locker = __SPIN_LOCK_UNLOCKED(recver->m_locker);
	recver->m_name = name;

#ifdef USE_THREAD
	/* init thread */
	sema_init(&recver->m_sem, 0);
	/*
	init_completion(&recver->m_exited);
	*/
	recver->m_tsk = kthread_create(
		(THREAD_T)mux_recv_func, recver, name);
	if (IS_ERR(recver->m_tsk))
		goto out;

	wake_up_process(recver->m_tsk);
#else
	INIT_DELAYED_WORK(&recver->m_recv_work, (WORKER_T)mux_recv_func);
#endif /* USE_THREAD */
	FUNC_EXIT();

	return recver;
out:
	kfree(recver);
	return NULL;
}

static void mux_recver_free(struct mux_recver_t *recver)
{
	struct mux_data_list_t *entry;
	struct list_head *ptr;
	unsigned long flags;

	FUNC_ENTER();

	if (!recver)
		return;
#ifdef USE_THREAD
	if (recver->m_tsk) {
		/*
		KILL_PROC(recver->m_pid, SIGTERM);
		wait_for_completion(&recver->m_exited);
		*/
		kthread_stop(recver->m_tsk);
	}
#endif
	spin_lock_irqsave(&recver->m_locker, flags);
	while (!(list_empty(&recver->m_datalist))) {
		ptr = recver->m_datalist.next;
		entry = list_entry(ptr, struct mux_data_list_t, list);
		list_del(ptr);
		kfree(entry->body);
		kfree(entry);
	}
	spin_unlock_irqrestore(&recver->m_locker, flags);

	kfree(recver);
	recver = NULL;
	FUNC_EXIT();
}

static int ts27010_usb_tty_queue_recv_data(struct mux_recver_t *recver,
	struct ts27010_tty_data *td, int line,
	struct ts27010_ringbuf *rbuf, int data_idx, int len)
{
	int ret = -ENOMEM;
	int count;
	struct mux_data_list_t *mux_list;
	unsigned long flags;

	FUNC_ENTER();

	if (!recver) {
		mux_print(MSG_ERROR, "No mux_usb_queue_recver available\n");
		goto EXIT;
	}

	if (len == 0) {
		mux_print(MSG_WARNING, "Don't queue empty data\n");
		goto EXIT;
	}

	mux_list = (struct mux_data_list_t *)kmalloc(
		sizeof(*mux_list), GFP_KERNEL);
	if (!mux_list) {
		mux_print(MSG_ERROR, "can not malloc (%d) memory!\n",
			sizeof(struct mux_data_list_t));
		goto EXIT;
	}

	mux_list->body = (u8 *)kmalloc(len, GFP_KERNEL);
	if (!(mux_list->body)) {
		kfree(mux_list);
		mux_print(MSG_ERROR, "can not malloc (%d) memory!\n", len);
		goto EXIT;
	}
	mux_list->size = len;
	mux_list->td = td;
	mux_list->line = line;
	count = ts27010_ringbuf_read(rbuf, data_idx, mux_list->body, len);
	if (count != len)
		mux_print(MSG_WARNING, "read size %d is not %d\n", count, len);
#ifdef DUMP_FRAME
	if (g_mux_usb_dump_user_data)
		mux_usb_hexdump(MSG_DEBUG, "dump tty_read",
			__func__, __LINE__, mux_list->body, len);
	else
		mux_usb_hexdump(MSG_MSGDUMP, "dump tty_read",
			__func__, __LINE__, mux_list->body, len);
#endif

	spin_lock_irqsave(&recver->m_locker, flags);
	list_add_tail(&mux_list->list, &recver->m_datalist);
#ifdef PROC_DEBUG_MUX_STAT
	td->chan[line].recv_size += len;
	s_nRecvCount++;
	s_nReadTotal += len;
#endif
	spin_unlock_irqrestore(&recver->m_locker, flags);
	mux_print(MSG_DEBUG, "%d into queue buffer by %s\n",
		len, recver->m_name);

#ifdef USE_THREAD
	up(&recver->m_sem);
#else
#ifdef QUEUE_SELF
	queue_delayed_work(g_mux_usb_queue, &recver->m_recv_work, 0);
#else
	schedule_delayed_work(&recver->m_recv_work, 0);
#endif
#endif /* USE_THREAD */
	FUNC_EXIT();
	return len;

EXIT:
	return ret;
}
#endif /* USB_QUEUE_RECV */

/*
int ts27010_tty_usb_send(int line, u8 *data, int len)
{
	struct ts27010_tty_data *td = driver->driver_state;
	struct tty_struct *tty;

	FUNC_ENTER();
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}

	tty = td->chan[line].tty;

	if (!tty) {
		mux_print(MSG_ERROR, "mux%d no open.  discarding %d bytes\n",
			line, len);
		return 0;
	}

	BUG_ON(tty_insert_flip_string(tty, data, len) != len);
	tty_flip_buffer_push(tty);

	FUNC_EXIT();
	return len;
}
*/

int ts27010_tty_usb_send_rbuf(int line, struct ts27010_ringbuf *rbuf,
			  int data_idx, int len)
{
	struct ts27010_tty_data *td = driver->driver_state;
	struct tty_struct *tty;
	u8 *buf;

	FUNC_ENTER();
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}

	tty = td->chan[line].tty;
	WARN_ON(tty != ts27010_usb_tty_table[line]);
	if (!tty) {
		mux_print(MSG_ERROR,
			"mux%d not be opened, discarding %d bytes data\n",
			line, len);
		return 0;
	}
#ifndef USB_QUEUE_RECV
	buf = (u8 *)kmalloc(len, GFP_KERNEL);
	if (buf) {
		int count = ts27010_ringbuf_read(rbuf, data_idx, buf, len);
		if (count != len)
			mux_print(MSG_WARNING,
				"read size %d is not %d\n", count, len);
#ifdef DUMP_FRAME
		if (g_mux_usb_dump_user_data)
			mux_usb_hexdump(MSG_DEBUG, "dump tty_read",
				__func__, __LINE__, buf, len);
		else
			mux_usb_hexdump(MSG_MSGDUMP, "dump tty_read",
				__func__, __LINE__, buf, len);
#endif

#ifdef PROC_DEBUG_MUX_STAT
		td->chan[line].recv_size += len;
		s_nRecvCount++;
		s_nReadTotal += len;
#endif
		tty_push(tty, buf, len);

#ifdef PROC_DEBUG_MUX_STAT
		td->chan[line].back_size += len;
		s_nBackCount++;
#endif
		kfree(buf);
	} else
		mux_print(MSG_ERROR, "can't get memory for tty_read\n");
#else /* USB_QUEUE_RECV */
	ts27010_usb_tty_queue_recv_data(
		s_mux_usb_recver, td, line, rbuf, data_idx, len);
#endif /* USB_QUEUE_RECV */

	FUNC_EXIT();
	return len;
}

static int ts27010_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct ts27010_tty_data *td = tty->driver->driver_state;
	int err;
	int line = tty->index;
	FUNC_ENTER();

	if (!ts27010_mux_usb_active()) {
		mux_print(MSG_ERROR,
			"tty open when line discipline not active.\n");
		return -ENODEV;
	}

	mux_print(MSG_INFO, "tty index /dev/usb/%d will be opened by %s:%d\n",
		line + TS27010MUX_NAME_BASE, current->comm, current->pid);
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}

	err = ts27010_mux_usb_line_open(line);
	if (err < 0)
		return err;

	atomic_inc(&td->chan[line].ref_count);
	if (atomic_read(&td->chan[line].ref_count) > 1) {
		mux_print(MSG_WARNING,
			"tty device: /dev/usb%d opened more than 1 times\n",
			line + TS27010MUX_NAME_BASE);
		WARN_ON(tty != ts27010_usb_tty_table[line]);
	} else {
		td->chan[line].tty = tty;
		ts27010_usb_tty_table[line] = tty;
		mux_print(MSG_INFO, "tty /dev/usb%d opened successfully\n",
			line + TS27010MUX_NAME_BASE);
		if (atomic_read(&td->chan[line].ref_count) < 1) {
			mux_print(MSG_WARNING,
				"tty device: /dev/usb%d wrong refcount: %d\n",
				line + TS27010MUX_NAME_BASE,
				atomic_read(&td->chan[line].ref_count));
			atomic_set(&td->chan[line].ref_count, 1);
		}
	}

	FUNC_EXIT();
	return 0;
}

static void ts27010_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct ts27010_tty_data *td = tty->driver->driver_state;
	int line = tty->index;
	FUNC_ENTER();

	mux_print(MSG_INFO, "tty device /dev/usb%d will be closed by %s:%d\n",
		line + TS27010MUX_NAME_BASE, current->comm, current->pid);
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return ;
	}

	if (atomic_read(&td->chan[line].ref_count) == 0) {
		mux_print(MSG_ERROR,
			"close error: tty device /dev/usb%d wasn't opened.\n",
			line);
		return;
	} else if (atomic_read(&td->chan[line].ref_count) < 0) {
		mux_print(MSG_WARNING,
			"tty device: /dev/usb%d wrong refcount: %d\n",
			line + TS27010MUX_NAME_BASE,
			atomic_read(&td->chan[line].ref_count));
		atomic_set(&td->chan[line].ref_count, 0);
	} else if (atomic_read(&td->chan[line].ref_count) > 1) {
		mux_print(MSG_WARNING,
			"tty device: /dev/usb%d isn't closed "
			"because of non-zero refcount\n",
			line + TS27010MUX_NAME_BASE);
		atomic_dec(&td->chan[line].ref_count);
	} else {
		ts27010_mux_usb_line_close(tty->index);

		td->chan[line].tty = NULL;
		ts27010_usb_tty_table[line] = NULL;
		atomic_set(&td->chan[line].ref_count, 0);

		/*
		 * the old code did:
		 *   wake_up_interruptible(&tty->read_wait);
		 *   wake_up_interruptible(&tty->write_wait);
		 *   tty->packet = 0;
		 *
		 * I belive this is unecessary
		 */
	}
	mux_print(MSG_INFO, "tty /dev/usb%d closed successfully\n",
		line + TS27010MUX_NAME_BASE);
	FUNC_EXIT();
}

static int ts27010_tty_write(struct tty_struct *tty,
			     const unsigned char *buf, int count)
{
	int ret;
	int line = tty->index;
	FUNC_ENTER();
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}
	mux_print(MSG_DEBUG, "write /dev/usb%d %d by %s:%d\n",
		line + TS27010MUX_NAME_BASE, count, current->comm, current->pid);
#ifdef DUMP_FRAME
	if (g_mux_usb_dump_user_data)
		mux_usb_hexdump(MSG_DEBUG, "dump tty_write",
			__func__, __LINE__, buf, count);
	else
		mux_usb_hexdump(MSG_MSGDUMP, "dump tty_write",
			__func__, __LINE__, buf, count);
#endif

	ret = ts27010_mux_usb_line_write(tty->index, buf, count);
#ifdef PROC_DEBUG_MUX_STAT
	if (ret == count) {
		((struct ts27010_tty_data *)tty->driver->driver_state)
			->chan[tty->index].sent_size += count;
		s_nWriteTotal += count;
		s_nWriteProt += count + (count < 128 ? 6 : 7);
		s_nWriteCount++;
	}
#endif
	FUNC_EXIT();
	return ret;
}


static int ts27010_tty_write_room(struct tty_struct *tty)
{
	int ret;
	int line = tty->index;
	FUNC_ENTER();
	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return -ENODEV;
	}
	ret = ts27010_mux_usb_line_write_room(tty->index);
	FUNC_EXIT();
	return ret;
}

static void ts27010_tty_flush_buffer(struct tty_struct *tty)
{
	int line = tty->index;
	FUNC_ENTER();

	if ((line < 0) || (line >= TS0710_MAX_MUX)) {
		mux_print(MSG_ERROR, "tty index out of range: %d.\n", line);
		return ;
	}

	ts27010_mux_usb_line_flush_buffer(line);
	FUNC_EXIT();
}

static int ts27010_tty_chars_in_buffer(struct tty_struct *tty)
{
	int ret;
	FUNC_ENTER();
	ret = ts27010_mux_usb_line_chars_in_buffer(tty->index);
	FUNC_EXIT();
	return ret;
}

static void ts27010_tty_throttle(struct tty_struct *tty)
{
	FUNC_ENTER();
	ts27010_mux_usb_line_throttle(tty->index);
	FUNC_EXIT();
}

static void ts27010_tty_unthrottle(struct tty_struct *tty)
{
	FUNC_ENTER();
	ts27010_mux_usb_line_unthrottle(tty->index);
	FUNC_EXIT();
}

static int ts27010_tty_ioctl(struct tty_struct *tty, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int ret;
	int line = tty->index;
	FUNC_ENTER();

	if ((line < 0) || (line >= TS0710_MAX_MUX))
		return -ENODEV;

	ret = ts27010_mux_usb_line_ioctl(cmd, arg, line);
	FUNC_EXIT();
	return ret;
}

int ts27010_tty_usb_open(void)
{
	FUNC_ENTER();

#ifdef USB_QUEUE_RECV
#ifdef USE_THREAD
	s_mux_usb_recver = mux_alloc_recver(
		mux_usb_recv_thread, "mux_usb_queue_recv");
#else
	s_mux_usb_recver = mux_alloc_recver(
		mux_usb_recv_worker, "mux_usb_queue_recv");
#endif /* USE_THREAD */
	if (!s_mux_usb_recver)
		return -ENOMEM;
#endif /* USB_QUEUE_RECV */
	FUNC_EXIT();
	return 0;
}

void ts27010_tty_usb_close(void)
{
	FUNC_ENTER();

#ifdef USB_QUEUE_RECV
	if (s_mux_usb_recver) {
		mux_recver_free(s_mux_usb_recver);
		s_mux_usb_recver = NULL;
	}
#endif
	FUNC_EXIT();
}

static const struct tty_operations ts27010_tty_ops = {
	.open = ts27010_tty_open,
	.close = ts27010_tty_close,
	.write = ts27010_tty_write,
	.write_room = ts27010_tty_write_room,
	.flush_buffer = ts27010_tty_flush_buffer,
	.chars_in_buffer = ts27010_tty_chars_in_buffer,
	.throttle = ts27010_tty_throttle,
	.unthrottle = ts27010_tty_unthrottle,
	.ioctl = ts27010_tty_ioctl,
};

int ts27010_tty_usb_init(void)
{
	struct ts27010_tty_data *td;
	int err;
	int i;

	FUNC_ENTER();

#ifdef USB_QUEUE_RECV
	s_mux_usb_recver = NULL;
#ifndef USE_THREAD
	s_mux_queuerecv_flags = 0;
#endif
#endif /* USB_QUEUE_RECV */

#ifdef PROC_DEBUG_MUX_STAT
	s_td = NULL;
	s_nWriteTotal = 0;
	s_nWriteProt = 0;
	s_nWriteCount = 0;
	s_nReadTotal = 0;
	s_nRecvCount = 0;
	s_nBackCount = 0;
#endif

	driver = alloc_tty_driver(TS0710_MAX_MUX);
	if (driver == NULL) {
		err = -ENOMEM;
		goto err0;
	}

	td = (struct ts27010_tty_data *)kzalloc(sizeof(*td), GFP_KERNEL);
	if (td == NULL) {
		mux_print(MSG_ERROR, "alloc ts27010_tty_data failed\n");
		err = -ENOMEM;
		goto err1;
	}

	for (i = 0; i < TS0710_MAX_MUX; i++) {
		atomic_set(&td->chan[i].ref_count, 0);
		td->chan[i].tty = NULL;
		ts27010_usb_tty_table[i] = NULL;
#ifdef USB_QUEUE_RECV
		mutex_init(&td->chan[i].lock);
#endif
#ifdef PROC_DEBUG_MUX_STAT
		td->chan[i].sent_size = 0;
		td->chan[i].recv_size = 0;
		td->chan[i].back_size = 0;
#endif
	}

	driver->driver_state = td;
#ifdef PROC_DEBUG_MUX_STAT
	s_td = td;
#endif

	driver->driver_name = "td_ts0710mux_usb";
	driver->name = "usb";
	driver->name_base = TS27010MUX_NAME_BASE;
	driver->major = TS0710MUX_MAJOR;
	driver->minor_start = TS0710MUX_MINOR_START;
	driver->type = TTY_DRIVER_TYPE_SERIAL;
	driver->subtype = SERIAL_TYPE_NORMAL;
	driver->init_termios = tty_std_termios;
	driver->init_termios.c_iflag = 0;
	driver->init_termios.c_oflag = 0;
	driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	driver->init_termios.c_lflag = 0;
	driver->other = NULL;
	driver->owner = THIS_MODULE;

	driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW
/*
		| TTY_DRIVER_DYNAMIC_DEV
*/
		;

	tty_set_operations(driver, &ts27010_tty_ops);
	err = tty_register_driver(driver);
	if (err) {
		mux_print(MSG_ERROR, "can't register tty driver: %d\n", err);
		err = -EINVAL;
		goto err2;
	}

	/*
	for (i = 0; i < TS0710_MAX_MUX; i++)
		tty_register_device(driver, i, NULL);
	*/

	FUNC_EXIT();
	return 0;

err2:
	kfree(td);
err1:
	put_tty_driver(driver);
err0:
	return err;
}

void ts27010_tty_usb_remove(void)
{
	struct ts27010_tty_data *td = driver->driver_state;
	FUNC_ENTER();

	tty_unregister_driver(driver);
	kfree(td);
	put_tty_driver(driver);
	FUNC_EXIT();
}

