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
#include <linux/delay.h>

#include <linux/kernel.h>
#include <linux/proc_fs.h>

#include <linux/serial.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/bitops.h>

#include <asm/system.h>

#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#endif

#include "ts27010.h"
#include "ts27010_mux.h"
#include "ts27010_td.h"
#include "ts27010_misc.h"

#ifdef CONFIG_WAKELOCK
/*  Add android power sleep lock */
#ifdef TS27010_UART_RETRAN
static struct wake_lock s_mux_resend_lock;
#endif
#endif

#ifdef TS27010_UART_RETRAN
struct sequence_number_t *g_ap_send_sn;
static struct sequence_number_t *s_ap_received_sn;
static struct ts27010_slide_window_t *s_slide_window;
static struct ts27010_timer_para_t *s_timer_para;
#endif /* TS27010_UART_RETRAN */

#ifdef MUX_USB_LOGGER
struct ts27010_mux_logger *g_mux_usb_logger;
#endif

#ifdef PROC_DEBUG_MUX
static struct proc_dir_entry *s_proc_mux;
#ifdef PROC_DEBUG_MUX_STAT
static struct proc_dir_entry *s_proc_stat_mux;
#endif
#endif

#ifdef USB_LOOP
static int ts27010_usb_control_send_loop(struct ts0710_con *ts0710,
	u8 *data, int len)
{
	struct short_frame *pkt = (struct short_frame *)(data + ADDRESS_OFFSET);

	switch (CLR_PF(data[CONTROL_OFFSET])) {
	case SABM:
	case DISC: {
			u8 frame[TS0710_FRAME_SIZE(0)] = {
				0xf9, 0x03, 0x73, 0x01, 0x35, 0xf9,
			};
			frame[1] = data[ADDRESS_OFFSET]; /* DLCI */
			/* simulate a USB IPC rx interrupt */
			ts27010_ldisc_usb_receive(
				ts27010mux_usb_tty, frame, NULL,
				TS0710_FRAME_SIZE(0));
		}
		break;
	case UIH: {/* echo PN */
			u8 type;
			type = pkt->data[0];
			switch (type >> 2) {
			case PN:
				pkt->h.addr.cr = 0;
				((struct mcc_short_frame *)pkt->data)->h.type.cr
					= 0;
				BUG_ON(TS0710_MCC_FRAME_SIZE(
					sizeof(struct pn_msg_data)) != len);
				ts27010_ldisc_usb_receive(
					ts27010mux_usb_tty, data, NULL, len);
				break;
			default:
				break;
			}
		}
		break;
	default:
		break;
	}
	return len;
}

/* for any UIH frame, the loop always returns "\r\nOK\r\n" */
static int ts27010_usb_uih_send_loop(struct ts0710_con *ts0710,
	u8 *data, int len)
{
	ts27010_ldisc_usb_receive(ts27010mux_usb_tty, data, NULL, len);
	return len;
}
#endif

#ifndef QUEUE_SEND
/* #define QUEUE_SEND */
#endif

#ifdef QUEUE_SEND
struct mux_data_list_t {
	struct list_head list;
	int size;
	u8 *body;
};

struct mux_sender_t {
	spinlock_t m_locker; /* sender list r/w lock */
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
	struct delayed_work m_send_work;
#endif
};

#ifdef USE_THREAD
typedef int (*THREAD_T)(void *data);
#else
typedef void (*WORKER_T)(struct work_struct *work);
#endif

#ifdef PROT_THREAD_SEND
static struct mux_sender_t *s_mux_protocol_sender;
#endif
static struct mux_sender_t *s_mux_uih_sender;

static int mux_usb_send_frame(struct mux_sender_t *sender)
{
	int sent;
	struct mux_data_list_t *entry;
	struct list_head *ptr;
	unsigned long flags = 0;

	spin_lock_irqsave(&sender->m_locker, flags);
	while (!(list_empty(&sender->m_datalist))) {
		ptr = sender->m_datalist.next;
		entry = list_entry(ptr, struct mux_data_list_t, list);
		spin_unlock_irqrestore(&sender->m_locker, flags);
		sent = ts27010_ldisc_usb_send(
			ts27010mux_usb_tty, entry->body, entry->size);
		if (sent != entry->size) {
			mux_print(MSG_WARNING, "%d of %d data sent to usb\n",
				sent, entry->size);
		}

		mux_print(MSG_DEBUG, "%d data sent to usb by %s\n",
			sent, sender->m_name);

		spin_lock_irqsave(&sender->m_locker, flags);
		list_del(ptr);
		kfree(entry->body);
		kfree(entry);
	}
	spin_unlock_irqrestore(&sender->m_locker, flags);

	return 0;
}

#ifdef USE_THREAD
static int mux_usb_send_thread(void *data)
{
	struct mux_sender_t *sender = (struct mux_sender_t *)data;

	DAEMONIZE(sender->m_name);

	while (down_interruptible(&sender->m_sem) == 0)
		mux_usb_send_frame(sender);

	mux_print(MSG_INFO, "thread %s exit\n", sender->m_name);
	/*
	complete_and_exit(&sender->m_exited, 0);
	*/
}
#else
static unsigned long s_mux_send_flags;
#define SEND_RUNNING 0

static void mux_usb_send_worker(struct work_struct *work)
{
	struct mux_sender_t *sender =
		container_of(work, struct mux_sender_t, m_send_work.work);

	if (test_and_set_bit(SEND_RUNNING, &s_mux_send_flags)) {
		mux_print(MSG_WARNING, "a send_work is running\n");
#ifdef QUEUE_SELF
		queue_delayed_work(g_mux_usb_queue, &sender->m_send_work,
			msecs_to_jiffies(20));
#else
		schedule_delayed_work(&sender->m_send_work,
			msecs_to_jiffies(20));
#endif
		return;
	}
	mux_usb_send_frame(sender);
	clear_bit(SEND_RUNNING, &s_mux_send_flags);
}
#endif

static struct mux_sender_t *mux_alloc_sender(
	void *mux_sender_func, const char *name)
{
	struct mux_sender_t *sender = NULL;

	sender = (struct mux_sender_t *)kzalloc(sizeof(*sender), GFP_KERNEL);
	if (sender == NULL) {
		mux_print(MSG_ERROR, "alloc mux_sender_t failed\n");
		goto out;
	}

	INIT_LIST_HEAD(&sender->m_datalist);
	sender->m_locker = __SPIN_LOCK_UNLOCKED(sender->m_locker);
	sender->m_name = name;

#ifdef USE_THREAD
	/* init thread */
	sema_init(&sender->m_sem, 0);
	/*
	init_completion(&sender->m_exited);
	*/
	sender->m_tsk = kthread_create(
		(THREAD_T)mux_sender_func, sender, name);
	if (IS_ERR(sender->m_tsk))
		goto out;

	wake_up_process(sender->m_tsk);
#else
	INIT_DELAYED_WORK(&sender->m_send_work, (WORKER_T)mux_sender_func);
#endif

	return sender;
out:
	kfree(sender);
	return NULL;
}

static void mux_sender_free(struct mux_sender_t *sender)
{
	struct mux_data_list_t *entry;
	struct list_head *ptr;
	unsigned long flags = 0;

	if (!sender)
		return;
#ifdef USE_THREAD
	if (sender->m_tsk) {
		/*
		KILL_PROC(sender->m_pid, SIGTERM);
		wait_for_completion(&sender->m_exited);
		*/
		kthread_stop(sender->m_tsk);
	}
#endif

	spin_lock_irqsave(&sender->m_locker, flags);
	while (!(list_empty(&sender->m_datalist))) {
		ptr = sender->m_datalist.next;
		entry = list_entry(ptr, struct mux_data_list_t, list);
		list_del(ptr);
		kfree(entry->body);
		kfree(entry);
	}
	spin_unlock_irqrestore(&sender->m_locker, flags);

	kfree(sender);
	sender = NULL;
}

static int mux_usb_queue_data(struct mux_sender_t *sender,
	u8 *buf, int len)
{
	int ret = -ENOMEM;
	struct mux_data_list_t *mux_list;
	unsigned long flags = 0;

	if (len == 0) {
		mux_print(MSG_WARNING, "Don't queue empty data\n");
		return -EIO;
	}

	mux_list = (struct mux_data_list_t *)kzalloc(
		sizeof(*mux_list), GFP_ATOMIC);
	if (!mux_list) {
		mux_print(MSG_ERROR, "can not malloc (%d) memory!\n",
			sizeof(struct mux_data_list_t));
		return ret;
	}

	mux_list->body = (u8 *)kmalloc(len, GFP_ATOMIC);
	if (!(mux_list->body)) {
		kfree(mux_list);
		mux_print(MSG_ERROR, "can not malloc (%d) memory!\n", len);
		return ret;
	}
	mux_list->size = len;
	memcpy(mux_list->body, buf, len);


	spin_lock_irqsave(&sender->m_locker, flags);
	list_add_tail(&mux_list->list, &sender->m_datalist);
	spin_unlock_irqrestore(&sender->m_locker, flags);
	mux_print(MSG_DEBUG, "%d into queue buffer by %s\n",
		len, sender->m_name);

#ifdef USE_THREAD
	up(&sender->m_sem);
#else
#ifdef QUEUE_SELF
	queue_delayed_work(g_mux_usb_queue, &sender->m_send_work, 0);
#else
	schedule_delayed_work(&sender->m_send_work, 0);
#endif
#endif

	return len;
}
#endif

static int ts27010_usb_driver_send(
	u8 *data, u32 len, struct wake_lock *mux_resend_lock)
{
	int ret;
	FUNC_ENTER();
	/*
	 * since ppp_async_push will hold a spin lock before call tty->wirte,
	 * we have to use mux_usb_queue_data
	 */
#ifdef QUEUE_SEND
	ret = mux_usb_queue_data(s_mux_uih_sender, data, len);
#else
	ret = ts27010_ldisc_usb_send(ts27010mux_usb_tty, data, len);
#endif
	if (ret < 0) {
		mux_print(MSG_ERROR, "pkt write error %d\n", ret);
		ret = -EIO;
	} else if (ret != len) {
		mux_print(MSG_ERROR, "short write %d < %d\n", ret, len);
		ret = -EIO;
	}
	FUNC_EXIT();
	return ret;
}

#ifdef TS27010_UART_RETRAN
static void transfer_timeout(unsigned long para)
{
	FUNC_ENTER();
	mux_print(MSG_INFO,
		"transfer_timeout, para= %ld, slide_window.head=%d, tail=%d, "
		"jiffies: %lu\n",
		para, ts27010_slidewindow_head(s_slide_window),
		ts27010_slidewindow_tail(s_slide_window), jiffies);

	if (para >= SLIDE_WINDOWS_SIZE_AP) {
		mux_print(MSG_ERROR, "illegal para: %ld", para);
		return;
	}
	ts27010_inc_timer_para(s_timer_para, para);
	ts27010_sched_retran(s_timer_para);

	FUNC_EXIT();
}

static void ts27010_usb_retran_worker(struct work_struct *work)
{
	int ret;
	int j;
	u8 para[SLIDE_WINDOWS_SIZE_AP];
	struct ts27010_retran_info_t *retran_info;
	FUNC_ENTER();

	memset(para, 0, sizeof(u8) * SLIDE_WINDOWS_SIZE_AP);
	/* get timeout indexs and clean counter */
	ts27010_get_timer_para(s_timer_para, para);

	ts27010_slidewindow_lock(s_slide_window);

	for (j = 0; j < SLIDE_WINDOWS_SIZE_AP; j++) {
		if (para[j]
			&& ts27010_slidewindow_is_idx_in(s_slide_window, j)) {
			/* need retransfer j */
			retran_info = ts27010_slidewindow_peek(
				s_slide_window, j);
			mux_print(MSG_INFO,
				"retransfering... index=%d, sn = %x, "
				"length=%d\n",
				j, ts27010_get_retran_sn(retran_info),
				ts27010_get_retran_len(retran_info));
			ret = ts27010_usb_driver_send(
				ts27010_get_retran_data(retran_info),
				ts27010_get_retran_len(retran_info),
				&s_mux_resend_lock);
			ts27010_mod_retran_timer(retran_info);

			if (!ret) {
				/* this frame retransfered */
				mux_print(MSG_INFO,
					"frame %x retransfered successful, "
					"length=%d\n",
					ts27010_get_retran_sn(retran_info),
					ts27010_get_retran_len(retran_info));
#ifdef CONFIG_WAKELOCK
				/*
				 * Re-transfer happened,
				 * so block system 1s for next re-send.
				 */
				wake_lock_timeout(&s_mux_resend_lock, HZ);
#endif
				if (ts27010_inc_retran_count(retran_info)
					== MAX_RETRAN_TIMES) {
					/* retran 10 times, trigger panic */
					mux_print(MSG_ERROR,
						"retrans frame %x(index %d) "
						"failed more than 10 times!\n",
						ts27010_get_retran_sn(
							retran_info), j);
					mux_usb_hexdump(MSG_ERROR,
						"dump frame",
						__func__, __LINE__,
						ts27010_get_retran_data(
							retran_info),
						ts27010_get_retran_len(
							retran_info));
					/* TODO: trigger panic or reset BP? */
				}
			} else {
				mux_print(MSG_ERROR,
					"retran interrupted because of "
					"ipc driver error\n");
				goto EXIT;
			}
		} else if (para[j]) {
			/*
			 * since j is not in slide window,
			 * should stop this timer
			 */
			mux_print(MSG_INFO,
				"timer index %d is out of slide window "
				"head: %d, tail %d\n",
				j, ts27010_slidewindow_head(s_slide_window),
				ts27010_slidewindow_tail(s_slide_window));
			ts27010_stop_retran_timer(
				ts27010_slidewindow_peek(s_slide_window, j));
			ts27010_clear_timer_para(s_timer_para, j);
		}
	}
EXIT:
	ts27010_slidewindow_unlock(s_slide_window);
	FUNC_EXIT();
}
#endif

/* only send UIH from APPs */
int ts27010_usb_uih_send(struct ts0710_con *ts0710, u8 *data, u32 len)
{
	int ret = -EIO;
	u8 dlci;
#ifdef TS27010_UART_RETRAN
	u8 index;
#endif
	FUNC_ENTER();

	mux_print(MSG_MSGDUMP, "will send UIH frame %d bytes\n", len);
#ifdef DUMP_FRAME
	if (g_mux_usb_dump_frame)
		mux_usb_hexdump(MSG_ERROR, "send UIH frame",
			__func__, __LINE__, data, len);
	else
		mux_usb_hexdump(MSG_MSGDUMP, "send UIH frame",
			__func__, __LINE__, data, len);
#endif

#ifdef USB_LOOP
	return ts27010_usb_uih_send_loop(ts0710, data, len);
#endif

#ifdef MUX_USB_LOGGER
	ts27010_mux_usb_logger_logdata(g_mux_usb_logger, data, len, 1);
#endif

#ifndef MUX_USB_UT
	/* seems bellow state checking is unneccessary */
	dlci = *(data + ADDRESS_OFFSET) >> 2;
	if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		mux_print(MSG_WARNING, "Flow stopped on DLCI: %d\n", dlci);
		return -EBUSY;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		mux_print(MSG_WARNING, "DLCI %d not connected\n", dlci);
		return -ENODEV;
	}
#endif

#ifdef TS27010_UART_RETRAN
	index = ts27010_slidewindo_store(s_slide_window, data, len);
	if (0xFF == index) {/* slide window is full, or get signal */
		/* trigger a panic or reset BP? */
		mux_print(MSG_ERROR, "slide window is full, block sending... "
			"should I report a panic?\n");
		return -EBUSY;
	}
	ret = ts27010_usb_driver_send(data, len, &s_mux_resend_lock);
	ts27010_slidewindow_lock(s_slide_window);
	ts27010_mod_retran_timer(
		ts27010_slidewindow_peek(s_slide_window, index));
	ts27010_slidewindow_unlock(s_slide_window);
	ts27010_clear_timer_para(s_timer_para, index);
#else
	ret = ts27010_usb_driver_send(data, len, NULL);
#endif

	FUNC_EXIT();
	return ret;
}

/*
* This function only be used for sending MUX control frame.
*/
int ts27010_usb_control_send(struct ts0710_con *ts0710, u8 *buf, u32 len)
{
	int ret = -EIO;
	FUNC_ENTER();

#ifdef DUMP_FRAME
	if (g_mux_usb_dump_frame)
		mux_usb_hexdump(MSG_ERROR, "send control frame",
			__func__, __LINE__, buf, len);
	else
		mux_usb_hexdump(MSG_MSGDUMP, "send control frame",
			__func__, __LINE__, buf, len);
#endif

#ifdef USB_LOOP
	return ts27010_usb_control_send_loop(ts0710, buf, len);
#endif

#ifdef MUX_USB_LOGGER
	ts27010_mux_usb_logger_logdata(g_mux_usb_logger, buf, len, 1);
#endif
	ret = ts27010_ldisc_usb_send(ts27010mux_usb_tty, buf, len);
	if (ret != len) {
		mux_print(MSG_ERROR, "ldisc send error %d(%d)\n", ret, len);
		ret = -EIO;
	}

	FUNC_EXIT();
	return ret;
}

#ifdef TS27010_UART_RETRAN
int ts27010_usb_process_ack(struct ts0710_con *ts0710, u8 sn)
{
	u8 error = sn >> 7;
	u8 number = sn & INDIFFERENT_SN;
	u8 index = 0xFF; /* index of the slide window */
	FUNC_ENTER();

	if (number == INDIFFERENT_SN) {
		/*
		 * the ACK has nothing to do with slide window,
		 * needn't to process, just ignore
		 */
		mux_print(MSG_MSGDUMP, "Invalid ACK frame, SN: %x\n", sn);
		return 0;
	}
	ts27010_slidewindow_lock(s_slide_window);

	/* sn will be removed the error flag in ts27010_slidewindow_is_sn_in */
	index = ts27010_slidewindow_is_sn_in(s_slide_window, sn);
	if (index == 0xFF) {
		/*
		 * unexpected sequence number, error, just ignore
		 * what if an ACK error while the sn is out of the window?
		 * impossible? yes
		 */
		mux_print(MSG_WARNING,
			"sn is not in slide window: tail=%d, head=%d, SN=%x\n",
			ts27010_slidewindow_tail(s_slide_window),
			ts27010_slidewindow_head(s_slide_window), sn);
	} else if (index == 0xFE) {
		if (error) {
			/*
			 * sn is not in slide window,
			 * but it is the next frame
			 * which we will send, so clear slide window
			 */
			ts27010_slidewindow_clear(s_slide_window);
			ts27010_slidewindow_wakeup(s_slide_window);
		} else {
			/*
			 * ACK OK, remove frames before this sn
			 * from sldie window
			 */
			mux_print(MSG_ERROR,
				"Encounter an unexpected error, "
				"BP sn count chaos? sn: %x, ap_send_sn: %x\n",
				sn, ts27010_sequence_number_get(g_ap_send_sn));
		}
	} else {/* correct sequence number */
		if (error) {
			/*
			 * ACK error, remove frames before this sn
			 * from sldie window
			 */
			u8 sn_head = ts27010_get_retran_sn(
				ts27010_slidewindow_peek(s_slide_window,
				ts27010_slidewindow_head(s_slide_window) - 1));
			u8 sn_tail = ts27010_get_retran_sn(
				ts27010_slidewindow_peek(s_slide_window,
				ts27010_slidewindow_tail(s_slide_window)));
			u8 sn_index = ts27010_get_retran_sn(
				ts27010_slidewindow_peek(s_slide_window,
				index));

			mux_print(MSG_WARNING,
				"received ACK ERROR, SN= %x, "
				"head sn = %d, tail sn = %d, "
				"index sn = %d,\n",
				sn, sn_head, sn_tail, sn_index);
			ts27010_clear_retran_counter(
				s_slide_window, s_timer_para, index, 0);
			ts27010_slidewindow_set_tail(s_slide_window, index);
		} else {
			/*
			 * ACK OK, remove frame of this sn and frames
			 * before this sn from slide window
			 */
			mux_print(MSG_DEBUG,
				"received ACK OK, SN= %x, "
				"head = %d, tail = %d, index = %d,\n",
				sn, ts27010_slidewindow_head(s_slide_window),
				ts27010_slidewindow_tail(s_slide_window),
				index);

			ts27010_clear_retran_counter(
				s_slide_window, s_timer_para, index, 1);
			ts27010_slidewindow_set_tail(
				s_slide_window,
				(index + 1) % SLIDE_WINDOWS_SIZE_AP);
		}
		ts27010_slidewindow_wakeup(s_slide_window);
	}
	ts27010_slidewindow_unlock(s_slide_window);

	FUNC_EXIT();
	return 0;
}

static int send_ack(struct ts0710_con *ts0710, u8 seq_num, u8 error)
{
	u8 buf[sizeof(struct short_frame) + FCS_SIZE + FLAG_SIZE];
	struct short_frame *ack = NULL;
	int ret;
	FUNC_ENTER();

	ack = (struct short_frame *)(buf + ADDRESS_OFFSET);
	ack->h.addr.ea = 1;
	/* MCC_CMD? */
	ack->h.addr.cr = ((ts0710->initiator) & 0x1);
	ack->h.addr.dlci = 0;/* only on control dlci */
	ack->h.control = ACK;
	ack->h.sn = seq_num | (error << 7);
	ack->h.length.ea = 1;
	ack->h.length.len = 0;

	buf[0] = TS0710_BASIC_FLAG;
	ack->data[0] = ts0710_usb_crc_data(
		buf + ADDRESS_OFFSET, sizeof(struct short_frame));
	ack->data[1] = TS0710_BASIC_FLAG;

	mux_print(MSG_MSGDUMP, "Send ACK, sn=%x\n", ack->h.sn);
	ret = ts27010_usb_control_send(ts0710, buf, TS0710_FRAME_SIZE(0));

	FUNC_EXIT();
	return ret;
}

int ts27010_usb_check_sequence_number(struct ts0710_con *ts0710, u8 sn)
{
	u8 ap_received_sn;
	int ret;
	FUNC_ENTER();

	ts27010_sequence_number_lock(s_ap_received_sn);
	ap_received_sn = ts27010_sequence_number_get(s_ap_received_sn);
	if (ap_received_sn == sn) {
		mux_print(MSG_DEBUG,
			"Got an expected frame, feedback ACK %x\n",
			ap_received_sn);
		send_ack(ts0710, ap_received_sn, ACK_OK);
		ts27010_sequence_number_inc(s_ap_received_sn);
		ret = 1;
	} else {
		mux_print(MSG_WARNING,
			"received SN %x is not expected SN: %x, "
			"discard data!\n", sn, ap_received_sn);
		mux_print(MSG_WARNING, "mux send bad-ack to bp\n");

		send_ack(ts0710, ap_received_sn, ACK_ERROR);
		ret = 0;
	}
	ts27010_sequence_number_unlock(s_ap_received_sn);
	FUNC_EXIT();
	return ret;
}
#endif

int ts27010_usb_td_open(void)
{
	FUNC_ENTER();
#ifdef PROC_DEBUG_MUX
	s_proc_mux = ts27010_usb_proc_alloc();
	if (!s_proc_mux)
		goto err;
#ifdef PROC_DEBUG_MUX_STAT
	s_proc_stat_mux = ts27010_usb_proc_stat_alloc();
	if (!s_proc_stat_mux)
		goto err;
#endif
#endif
#ifdef TS27010_UART_RETRAN
	s_timer_para = ts27010_alloc_timer_para(ts27010_usb_retran_worker);
	if (!s_timer_para)
		goto err;
	s_slide_window = ts27010_slidewindow_alloc(transfer_timeout);
	if (!s_slide_window)
		goto err;
	s_ap_received_sn = ts27010_sequence_number_alloc();
	if (!s_ap_received_sn)
		goto err;
	g_ap_send_sn = ts27010_sequence_number_alloc();
	if (!g_ap_send_sn)
		goto err;
#endif

#ifdef PROT_THREAD_SEND
#ifdef USE_THREAD
	s_mux_protocol_sender = mux_alloc_sender(
		mux_usb_send_thread, "mux_prot_sender");
#else
	s_mux_protocol_sender = mux_alloc_sender(
		mux_usb_send_worker, "mux_prot_sender");
#endif
	if (!s_mux_protocol_sender)
		goto err;
#endif

#ifdef QUEUE_SEND
#ifdef USE_THREAD
	s_mux_uih_sender = mux_alloc_sender(
		mux_usb_send_thread, "mux_uih_sender");
#else
	s_mux_send_flags = 0;
	s_mux_uih_sender = mux_alloc_sender(
		mux_usb_send_worker, "mux_uih_sender");
#endif
	if (!s_mux_uih_sender)
		goto err;
#endif

	FUNC_EXIT();
	return 0;

err:
#ifdef PROT_THREAD_SEND
	if (s_mux_protocol_sender)
		mux_sender_free(s_mux_protocol_sender);
#endif
#ifdef TS27010_UART_RETRAN
	if (g_ap_send_sn)
		ts27010_sequence_number_free(g_ap_send_sn);
	if (s_ap_received_sn)
		ts27010_sequence_number_free(s_ap_received_sn);
	if (s_slide_window)
		ts27010_slidewindow_free(s_slide_window);
	if (s_timer_para)
		ts27010_timer_para_free(s_timer_para);
#endif
#ifdef PROC_DEBUG_MUX
#ifdef PROC_DEBUG_MUX_STAT
	if (s_proc_stat_mux)
		ts27010_usb_proc_free(s_proc_stat_mux, "muxusb_stat");
#endif
	if (s_proc_mux)
		ts27010_usb_proc_free(s_proc_mux, "muxusb");
#endif
	return -ENOMEM;
}

int ts27010_usb_td_init(void)
{
	FUNC_ENTER();

#ifdef CONFIG_WAKELOCK
	/* Init android sleep lock. */
#ifdef TS27010_UART_RETRAN
	wake_lock_init(&s_mux_resend_lock, WAKE_LOCK_SUSPEND, "mux_resend");
#endif
#endif

#ifdef MUX_USB_LOGGER
	g_mux_usb_logger = ts27010_alloc_mux_usb_logger();
	if (!g_mux_usb_logger)
		return -ENOMEM;
#endif

	FUNC_EXIT();
	return 0;
}

void ts27010_usb_td_close(void)
{
#ifdef TS27010_UART_RETRAN
	int j;
#endif
	FUNC_ENTER();
#ifdef TS27010_UART_RETRAN
	if (s_slide_window) {
		for (j = 0; j < SLIDE_WINDOWS_SIZE_AP; j++) {
			ts27010_stop_retran_timer(
				ts27010_slidewindow_peek(s_slide_window, j));
		}
		ts27010_timer_para_free(s_timer_para);
		ts27010_slidewindow_free(s_slide_window);
		ts27010_sequence_number_free(s_ap_received_sn);
		ts27010_sequence_number_free(g_ap_send_sn);
	}
	s_timer_para = NULL;
	s_slide_window = NULL;
	s_ap_received_sn = NULL;
	g_ap_send_sn = NULL;
#endif

#ifdef QUEUE_SEND
#ifdef PROT_THREAD_SEND
	if (s_mux_protocol_sender) {
		mux_sender_free(s_mux_protocol_sender);
		s_mux_protocol_sender = NULL;
	}
#endif
	if (s_mux_uih_sender) {
		mux_sender_free(s_mux_uih_sender);
		s_mux_uih_sender = NULL;
	}
#endif

#ifdef PROC_DEBUG_MUX
#ifdef PROC_DEBUG_MUX_STAT
	if (s_proc_stat_mux)
		ts27010_usb_proc_free(s_proc_stat_mux, "muxusb_stat");
#endif
	if (s_proc_mux)
		ts27010_usb_proc_free(s_proc_mux, "muxusb");
#endif
	FUNC_EXIT();
}

void ts27010_usb_td_remove(void)
{
	FUNC_ENTER();
#ifdef CONFIG_WAKELOCK
	/* remove android sleep lock. */
#ifdef TS27010_UART_RETRAN
	wake_lock_destroy(&s_mux_resend_lock);
#endif
#endif

#ifdef MUX_USB_LOGGER
	ts27010_free_mux_usb_logger(g_mux_usb_logger);
#endif
	FUNC_EXIT();
}
