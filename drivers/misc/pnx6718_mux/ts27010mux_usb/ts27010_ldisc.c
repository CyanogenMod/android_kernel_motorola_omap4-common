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
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "ts27010.h"
#include "ts27010_mux.h"
#include "ts27010_ringbuf.h"
#include "ts27010_misc.h"

struct ts27010_ldisc_data {
	struct ts27010_ringbuf		*rbuf;
	struct delayed_work		recv_work;
	spinlock_t			recv_lock; /* ldisc read lock */
	struct mutex			send_lock; /* ldisc write lock */
	atomic_t 			ref_count;
};

#ifdef PROC_DEBUG_MUX_STAT
int g_nStatDrvIO;
static int s_nDrvRecved;
static int s_nUSBRecved;
static int s_nDrvSent;

void ts27010_ldisc_usb_drv_stat_clear(void)
{
	s_nDrvSent = 0;
	s_nDrvRecved = 0;
	s_nUSBRecved = 0;
}

void ts27010_ldisc_usb_drv_stat(void)
{
	mux_print(MSG_ERROR, "Drv Sent: %d, Drv Recved: %d, UBS Recved: %d\n",
		s_nDrvSent, s_nDrvRecved, s_nUSBRecved);
}
#endif

#define RECV_RUNNING 0
#define MUX_USB_RESERVE_BUFFERSIZE (5 * DEF_TS0710_MTU)

static unsigned long s_mux_recv_flags;
static int s_bFC_USB;
static int s_bHold;
static struct ts27010_ldisc_data *s_ld;

static void ts27010_ldisc_recv_worker(struct work_struct *work)
{
	int left;
	unsigned long flags;
	FUNC_ENTER();

	if (test_and_set_bit(RECV_RUNNING, &s_mux_recv_flags)) {
		mux_print(MSG_WARNING, "another recv_work is running\n");
#ifdef QUEUE_SELF
		queue_delayed_work(g_mux_usb_queue, &s_ld->recv_work,
			msecs_to_jiffies(20));
#else
		schedule_delayed_work(&s_ld->recv_work, msecs_to_jiffies(20));
#endif
		return;
	}
	/* TODO: should have a *mux to pass around */
	ts27010_mux_usb_recv(s_ld->rbuf);

	spin_lock_irqsave(&s_ld->recv_lock, flags);
	left = ts27010_ringbuf_room(s_ld->rbuf);
	spin_unlock_irqrestore(&s_ld->recv_lock, flags);
	if (s_bFC_USB && left > MUX_USB_RESERVE_BUFFERSIZE) {
		mux_print(MSG_ERROR, "MUX stop flow control USB\n");
		s_bFC_USB = 0;
		if (ts27010mux_usb_tty && ts27010mux_usb_tty->driver
			&& ts27010mux_usb_tty->driver->ops
			&& ts27010mux_usb_tty->driver->ops->unthrottle)
			ts27010mux_usb_tty->driver->ops->unthrottle(
				ts27010mux_usb_tty);
	}
	clear_bit(RECV_RUNNING, &s_mux_recv_flags);

	FUNC_EXIT();
}

int ts27010_ldisc_usb_send(struct tty_struct *tty, u8 *data, int len)
{
	int sent = 0;
	int n;
	FUNC_ENTER();

	if (tty == NULL || tty->disc_data == NULL) {
		mux_print(MSG_ERROR,
			"try to send mux data while ttyUSB0 is closed.\n");
		return -ENODEV;
	}
	WARN_ON(tty->disc_data != s_ld);
	WARN_ON(tty != ts27010mux_usb_tty);

	if (tty->driver->ops->write_room
		&& tty->driver->ops->write_room(tty) < len) {
		mux_print(MSG_ERROR,
			"******** write overflow ******** \n");
		mux_print(MSG_ERROR, "no room for writing, "
			"request %d, but left %d\n", len,
			tty->driver->ops->write_room(tty));
		return -EAGAIN;
	}

	while (sent < len && ts27010mux_usb_tty) {
		n = tty->driver->ops->write(tty, data + sent, len - sent);
		if (n <= 0) {
			mux_print(MSG_ERROR, "write usb failed: %d-%d-%d\n",
				n, sent, len);
			mux_usb_hexdump(MSG_ERROR, "send frame failed",
				__func__, __LINE__, data, len);
			break;
		} else if (n < len) {
			mux_print(MSG_WARNING, "partially write %d\n", n);
		}
		sent += n;
	}
	if (!ts27010mux_usb_tty) {
		mux_print(MSG_ERROR, "try to send mux data "
			"while ttyUSB0 is closed.\n");
		mux_print(MSG_ERROR, "sent = %d\n", sent);
		mux_usb_hexdump(MSG_ERROR, "send frame failed",
			__func__, __LINE__, data, len);
		/* TODO: should trigger a panic or reset BP */
	}
#ifdef PROC_DEBUG_MUX_STAT
	if (g_nStatDrvIO)
		s_nDrvSent += sent;
#endif

	mux_print(MSG_DEBUG, "tx %d to BP\n", sent);
	FUNC_EXIT();
	return sent;
}

/*
 * Called when a tty is put into tx27010mux line discipline. Called in process
 * context.
 */
static int ts27010_ldisc_open(struct tty_struct *tty)
{
	int err = 0;
	FUNC_ENTER();

	if (ts27010mux_usb_tty) {
		atomic_inc(&s_ld->ref_count);
		mux_print(MSG_WARNING, "ldisc re-open by process %s(%d)\n",
			current->comm, current->pid);
		return 0;
	}
	if (tty && tty->driver) {
		mux_print(MSG_INFO, "(%s:%d) set driver_name: %s, "
			"driver: %s to mux usb ldisc\n",
			current->comm, current->pid,
			tty->driver->driver_name, tty->driver->name);
	} else {
		mux_print(MSG_ERROR, "no usb tty devices %x:%x\n",
			(unsigned int)tty,
			(unsigned int)(tty ? tty->driver : NULL));
		return -ENODEV;
	}
	/* disable MUX just for debugging USB driver */
/*/	return 0; /*/

#ifndef MUX_USB_UT
	if (s_bHold) {
		ts27010_mux_usb_mux_resume();
		s_bHold = 0;
	} else {
		err = ts27010_mux_usb_mux_open();
		if (err)
			goto err1;
	}
#endif
	tty->disc_data = s_ld;
	atomic_inc(&s_ld->ref_count);

	/* TODO: goes away with clean tty interface */
	ts27010mux_usb_tty = tty;

	FUNC_EXIT();
	return 0;

err1:
	tty->disc_data = NULL;
	return err;
}

/*
 * Called when the tty is put into another line discipline
 * or it hangs up.  We have to wait for any cpu currently
 * executing in any of the other ts27010_tty_* routines to
 * finish before we can call tsmux27010_unregister_channel and free
 * the tsmux27010 struct.  This routine must be called from
 * process context, not interrupt or softirq context.
 */
static void ts27010_ldisc_close(struct tty_struct *tty)
{
	FUNC_ENTER();

	atomic_dec(&s_ld->ref_count);
	if (atomic_read(&s_ld->ref_count) > 0) {
		mux_print(MSG_WARNING,
			"ldisc close non-zero refcount: process %s(%d)\n",
			current->comm, current->pid);
		return;
	}
	if (!ts27010mux_usb_tty) {
		atomic_set(&s_ld->ref_count, 0);
		mux_print(MSG_WARNING, "ldisc re-close by process %s(%d)\n",
			current->comm, current->pid);
		return;
	}
	if (tty && tty->driver) {
		mux_print(MSG_INFO, "driver_name: %s, "
			"driver: %s close mux usb ldisc\n",
			tty->driver->driver_name, tty->driver->name);
	}

	mutex_lock(&s_ld->send_lock);
	/* TODO: goes away with clean tty interface */
	ts27010mux_usb_tty = NULL;
	atomic_set(&s_ld->ref_count, 0);
#ifndef MUX_USB_UT
	if (s_bHold) {
		ts27010_mux_usb_mux_hold();
		mux_print(MSG_WARNING, "process %s(%d) hold MUX line disc\n",
			current->comm, current->pid);
	} else {
		ts27010_mux_usb_mux_close();
		mux_print(MSG_INFO, "process %s(%d) close MUX line discipline\n",
			current->comm, current->pid);
	}
#endif
	tty->disc_data = NULL;
	mutex_unlock(&s_ld->send_lock);
	FUNC_EXIT();
}

/*
 * Called on tty hangup in process context.
 *
 * Wait for I/O to driver to complete and unregister ts27010mux channel.
 * This is already done by the close routine, so just call that.
 */
static int ts27010_ldisc_hangup(struct tty_struct *tty)
{
	FUNC_ENTER();
	/* /dev/ttyUBS0 disconnected */
	s_bHold = 1;
	ts27010_ldisc_close(tty);
	FUNC_EXIT();
	return 0;
}

/*
 * Read does nothing - no data is ever available this way.
 */
static ssize_t ts27010_ldisc_read(struct tty_struct *tty, struct file *file,
				   unsigned char __user *buf, size_t count)
{
	FUNC_ENTER();
	return -EAGAIN;
}

/*
 * Write on the tty does nothing.
 */
static ssize_t ts27010_ldisc_write(struct tty_struct *tty, struct file *file,
				   const unsigned char *buf, size_t count)
{
	FUNC_ENTER();
	return -EAGAIN;
}

/*
 * Called in process context only. May be re-entered by multiple
 * ioctl calling threads.
 */
static int ts27010_ldisc_ioctl(struct tty_struct *tty, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int err;
	FUNC_ENTER();

	switch (cmd) {
	default:
		/* Try the various mode ioctls */
		err = tty_mode_ioctl(tty, file, cmd, arg);
	}

	FUNC_EXIT();
	return err;
}

/* No kernel lock - fine */
static unsigned int ts27010_ldisc_poll(struct tty_struct *tty,
				       struct file *file,
				       poll_table *wait)
{
	FUNC_ENTER();
	return 0;
}

/*
 * This can now be called from hard interrupt level as well
 * as soft interrupt level or mainline.  Because of this,
 * we copy the data and schedule work so that we can assure
 * the mux receive code is called in processes context.
 */
#if !defined(USB_LOOP) && !defined(MUX_USB_UT)
static
#endif
void ts27010_ldisc_usb_receive(struct tty_struct *tty,
				  const unsigned char *data,
				  char *cflags, int count)
{
	int n;
	int left;
	unsigned long flags;
	FUNC_ENTER();

	WARN_ON(count == 0);
	if (count == 0) {
		mux_print(MSG_ERROR, "receive 0 data\n");
		return;
	}
	WARN_ON(s_ld == NULL);
	if (s_ld == NULL) {
		mux_print(MSG_ERROR, "no ldisc data allocated\n");
		return;
	}
	WARN_ON(s_ld->rbuf == NULL);
	if (s_ld->rbuf == NULL) {
		mux_print(MSG_ERROR, "no ring buffer allocated\n");
		return;
	}
	WARN_ON(tty != ts27010mux_usb_tty);

#ifdef PROC_DEBUG_MUX_STAT
	if (g_nStatDrvIO)
		s_nUSBRecved += count;
#endif
	/* save data */
	mux_print(MSG_DEBUG, "rx %d from BP\n", count);
	spin_lock_irqsave(&s_ld->recv_lock, flags);
	n = ts27010_ringbuf_write(s_ld->rbuf, data, count);
#ifdef PROC_DEBUG_MUX_STAT
	if (g_nStatDrvIO)
		s_nDrvRecved += n;
#endif
	left = ts27010_ringbuf_room(s_ld->rbuf);
	spin_unlock_irqrestore(&s_ld->recv_lock, flags);
	if (left <= MUX_USB_RESERVE_BUFFERSIZE) {
		mux_print(MSG_ERROR, "MUX start flow control USB\n");
		s_bFC_USB = 1;
		/* use ts27010mux_usb_tty? */
		if (tty && tty->driver && tty->driver->ops
			&& tty->driver->ops->throttle)
			tty->driver->ops->throttle(tty);
	}

	if (n < count) {
		mux_print(MSG_ERROR, "*** buffer overrun. "
			"receive %d, save %d, missing %d!\n",
			count, n, count - n);
		mux_usb_hexdump(MSG_ERROR, "dump buf",
			__func__, __LINE__, data, count);
		/* TODO: should cache the unsaved data? */
	}

#ifdef MUX_USB_LOGGER
	ts27010_mux_usb_logger_logdata(
		g_mux_usb_logger, (u8 *)data, count, 0);
#endif
#ifdef DUMP_FRAME
	if (g_mux_usb_dump_frame)
		mux_usb_hexdump(MSG_ERROR, "dump recv data",
			__func__, __LINE__, data, count);
	else
		mux_usb_hexdump(MSG_MSGDUMP, "dump recv data",
			__func__, __LINE__, data, count);
#endif

	/* handle and dispatch data */
#ifndef MUX_USB_UT
#ifdef QUEUE_SELF
	queue_delayed_work(g_mux_usb_queue, &s_ld->recv_work, 0);
#else
	schedule_delayed_work(&s_ld->recv_work, 0);
#endif
#endif
	FUNC_EXIT();
}

static void ts27010_ldisc_wakeup(struct tty_struct *tty)
{
	FUNC_ENTER();
	mux_print(MSG_INFO, "Enter into ts27010mux_tty_wakeup.\n");
	FUNC_EXIT();
}

static struct tty_ldisc_ops ts27010_ldisc = {
	.owner  = THIS_MODULE,
	.magic	= TTY_LDISC_MAGIC,
	.name	= "n_td_ts27010_usb",
	.open	= ts27010_ldisc_open,
	.close	= ts27010_ldisc_close,
	.hangup	= ts27010_ldisc_hangup,
	.read	= ts27010_ldisc_read,
	.write	= ts27010_ldisc_write,
	.ioctl	= ts27010_ldisc_ioctl,
	.poll	= ts27010_ldisc_poll,
	.receive_buf = ts27010_ldisc_usb_receive,
	.write_wakeup = ts27010_ldisc_wakeup,
};

int ts27010_ldisc_usb_init(void)
{
	int err;
	FUNC_ENTER();

#ifdef PROC_DEBUG_MUX_STAT
	g_nStatDrvIO = 0;
	s_nDrvRecved = 0;
	s_nUSBRecved = 0;
	s_nDrvSent = 0;
#endif

	s_mux_recv_flags = 0;
	s_bFC_USB = 0;
	s_bHold = 0;
	s_ld = NULL;

	s_ld = (struct ts27010_ldisc_data *)kzalloc(sizeof(*s_ld), GFP_KERNEL);
	if (s_ld == NULL) {
		mux_print(MSG_ERROR, "alloc ts27010_ldisc_data failed\n");
		err = -ENOMEM;
		goto err0;
	}

	s_ld->rbuf = ts27010_ringbuf_alloc(LDISC_BUFFER_SIZE);
	if (s_ld->rbuf == NULL) {
		err = ENOMEM;
		goto err1;
	}

	mutex_init(&s_ld->send_lock);
	s_ld->recv_lock = __SPIN_LOCK_UNLOCKED(s_ld->recv_lock);
	INIT_DELAYED_WORK(&s_ld->recv_work, ts27010_ldisc_recv_worker);
	atomic_set(&s_ld->ref_count, 0);

	err = tty_register_ldisc(N_TD_TS2710_USB, &ts27010_ldisc);
	if (err < 0)
		mux_print(MSG_ERROR,
			"ts27010: unable to register line discipline\n");

	FUNC_EXIT();
	return err;

err1:
	kfree(s_ld);
	s_ld = NULL;
err0:
	return err;
}

void ts27010_ldisc_usb_remove(void)
{
	FUNC_ENTER();
	tty_unregister_ldisc(N_TD_TS2710_USB);
	ts27010_ringbuf_free(s_ld->rbuf);
	kfree(s_ld);
	FUNC_EXIT();
}
