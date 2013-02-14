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

struct ts27010_ringbuf;
/*  Add android power sleep lock */
#ifdef CONFIG_WAKELOCK
/* this lock should be used during receive_buffer */
static struct wake_lock s_mux_suspend_lock;
#ifdef TS27010_UART_RETRAN
/* this lock should be used during re-transmission */
static struct wake_lock s_mux_resend_lock;
#endif
#endif

#ifdef TS27010_UART_RETRAN
struct sequence_number_t *g_ap_send_sn;
static struct sequence_number_t *s_ap_received_sn;
static struct ts27010_slide_window_t *s_slide_window;
static struct ts27010_timer_para_t *s_timer_para;
#endif /* TS27010_UART_RETRAN */

#ifdef MUX_UART_LOGGER
struct ts27010_mux_logger *g_mux_uart_logger;
#endif

#ifdef PROC_DEBUG_MUX
static struct proc_dir_entry *s_proc_mux;
#ifdef PROC_DEBUG_MUX_STAT
static struct proc_dir_entry *s_proc_stat_mux;
#endif
#endif

#ifdef UART_LOOP
static int ts27010_uart_control_send_loop(struct ts0710_con *ts0710,
	u8 *data, int len)
{
	struct short_frame *pkt = (struct short_frame *)(data + ADDRESS_OFFSET);

	switch (CLR_PF(data[CONTROL_OFFSET])) {
	case SABM:
	case DISC: {/* send UA */
#ifdef TS27010_UART_RETRAN
			u8 frame[TS0710_FRAME_SIZE(0)] = {
				0xf9, 0x03, 0x73, 0x00, 0x01, 0x35, 0xf9,
			};
			ts27010_sequence_number_lock(s_ap_received_sn);
			frame[3] = ts27010_sequence_number_get(
				s_ap_received_sn); /* SN */
			ts27010_sequence_number_unlock(s_ap_received_sn);
			frame[5] = ts0710_uart_crc_data(
				frame + ADDRESS_OFFSET, 4); /* FCS */
#else
			u8 frame[TS0710_FRAME_SIZE(0)] = {
				0xf9, 0x03, 0x73, 0x01, 0x35, 0xf9,
			};
			frame[1] = data[ADDRESS_OFFSET]; /* DLCI */
#endif
			/* simulate a UART IPC rx interrupt */
			ts27010_ldisc_uart_receive(ts27010mux_uart_tty,
				frame, NULL, TS0710_FRAME_SIZE(0));
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
#ifdef TS27010_UART_RETRAN
				ts27010_sequence_number_lock(s_ap_received_sn);
				pkt->h.sn = ts27010_sequence_number_get(
					s_ap_received_sn);/* SN */
				ts27010_sequence_number_unlock(
					s_ap_received_sn);
				data[len - 2] = ts0710_uart_crc_data(
					data + ADDRESS_OFFSET,
					len - FCS_SIZE - FLAG_SIZE);/* FCS */
#endif
				BUG_ON(TS0710_MCC_FRAME_SIZE(
					sizeof(struct pn_msg_data)) != len);
				ts27010_ldisc_uart_receive(
					ts27010mux_uart_tty, data, NULL, len);
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
static int ts27010_uart_uih_send_loop(struct ts0710_con *ts0710,
	u8 *data, int len)
{
#ifdef TS27010_UART_RETRAN
	u8 frame[TS0710_FRAME_SIZE(6)] = {
		0xf9, 0x05, 0xef, 0x05, 0x0d, 0x0d, 0x0a,
		0x4f, 0x4b, 0x0d, 0x0a, 0xe0, 0xf9,
	};
	frame[ADDRESS_OFFSET] = data[ADDRESS_OFFSET];/* DLCI */
	ts27010_sequence_number_lock(s_ap_received_sn);
	frame[SEQUENCE_OFFSET] = ts27010_sequence_number_get(
		s_ap_received_sn);/* SN */
	ts27010_sequence_number_unlock(s_ap_received_sn);
	frame[TS0710_FRAME_SIZE(6) - 2] = ts0710_uart_crc_data(
		frame + ADDRESS_OFFSET,
		TS0710_FRAME_SIZE(6) - FCS_SIZE - FLAG_SIZE);/* FCS */
	ts27010_ldisc_uart_receive(ts27010mux_uart_tty, frame, NULL,
		TS0710_FRAME_SIZE(6));
	ts27010_ldisc_uart_receive(ts27010mux_uart_tty, data, NULL, len);
#else
	ts27010_ldisc_uart_receive(ts27010mux_uart_tty, data, NULL, len);
#endif
	return len;
}
#endif

static int ts27010_uart_driver_send(
	u8 *data, u32 len, struct wake_lock *mux_resend_lock)
{
	int ret;
	FUNC_ENTER();

	ret = ts27010_ldisc_uart_send(ts27010mux_uart_tty, data, len);
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
		"transfer_timeout, para= %ld, slide_window.head=%d, "
		"tail=%d, jiffies: %lu\n",
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

static void ts27010_uart_retran_worker(struct work_struct *work)
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
		if (para[j] && ts27010_slidewindow_is_idx_in(
			s_slide_window, j)) {
			/* need retransfer j */
			retran_info = ts27010_slidewindow_peek(
				s_slide_window, j);
			mux_print(MSG_INFO,
				"retransfering... index=%d, "
				"sn = %x, length=%d\n",
				j, ts27010_get_retran_sn(retran_info),
				ts27010_get_retran_len(retran_info));
			ret = ts27010_uart_driver_send(
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
					mux_uart_hexdump(MSG_ERROR,
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
int ts27010_uart_uih_send(struct ts0710_con *ts0710, u8 *data, u32 len)
{
	int ret = -EIO;
	u8 dlci;
#ifdef TS27010_UART_RETRAN
	u8 index;
#endif
	FUNC_ENTER();

	mux_print(MSG_MSGDUMP, "will send UIH frame %d bytes\n", len);
#ifdef DUMP_FRAME
	if (g_mux_uart_dump_frame)
		mux_uart_hexdump(MSG_ERROR, "send UIH frame",
			__func__, __LINE__, data, len);
	else
		mux_uart_hexdump(MSG_MSGDUMP, "send UIH frame",
			__func__, __LINE__, data, len);
#endif

#ifdef UART_LOOP
	return ts27010_uart_uih_send_loop(ts0710, data, len);
#endif

#ifdef MUX_UART_LOGGER
	ts27010_mux_uart_logger_logdata(g_mux_uart_logger, data, len, 1);
#endif

	/* seems bellow state checking is unneccessary */
	dlci = *(data + ADDRESS_OFFSET) >> 2;
	if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		mux_print(MSG_WARNING, "Flow stopped on DLCI: %d\n", dlci);
		return -EBUSY;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		mux_print(MSG_WARNING, "DLCI %d not connected\n", dlci);
		return -ENODEV;
	}

#ifdef TS27010_UART_RETRAN
	index = ts27010_slidewindo_store(s_slide_window, data, len);
	if (0xFF == index) {/* slide window is full, or get signal */
		/* TODO: trigger a panic or reset BP? */
		mux_print(MSG_ERROR, "slide window is full, block sending... "
			"should I report a panic?\n");
		return -EBUSY;
	}
	ret = ts27010_uart_driver_send(data, len, &s_mux_resend_lock);
	ts27010_slidewindow_lock(s_slide_window);
	ts27010_mod_retran_timer(
		ts27010_slidewindow_peek(s_slide_window, index));
	ts27010_slidewindow_unlock(s_slide_window);
	ts27010_clear_timer_para(s_timer_para, index);
#else
	ret = ts27010_uart_driver_send(data, len, NULL);
#endif

	FUNC_EXIT();
	return ret;
}

/*
* This function only be used for sending MUX control frame.
*/
int ts27010_uart_control_send(struct ts0710_con *ts0710, u8 *buf, u32 len)
{
	int ret = -EIO;
	FUNC_ENTER();

#ifdef DUMP_FRAME
	if (g_mux_uart_dump_frame)
		mux_uart_hexdump(MSG_ERROR, "send control frame",
			__func__, __LINE__, buf, len);
	else
		mux_uart_hexdump(MSG_MSGDUMP, "send control frame",
			__func__, __LINE__, buf, len);
#endif

#ifdef UART_LOOP
	return ts27010_uart_control_send_loop(ts0710, buf, len);
#endif

#ifdef MUX_UART_LOGGER
	ts27010_mux_uart_logger_logdata(g_mux_uart_logger, buf, len, 1);
#endif
	ret = ts27010_ldisc_uart_send(ts27010mux_uart_tty, buf, len);
	if (ret != len) {
		mux_print(MSG_ERROR, "ldisc send error %d(%d)\n", ret, len);
		ret = -EIO;
	}

	FUNC_EXIT();
	return ret;
}

#ifdef TS27010_UART_RETRAN
int ts27010_uart_process_ack(struct ts0710_con *ts0710, u8 sn)
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

	/*
	 * sn will be removed the error flag
	 * in ts27010_slidewindow_is_sn_in
	 */
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
	} else {
		/* correct sequence number */
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
				ts27010_slidewindow_peek(
					s_slide_window, index));

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
			ts27010_slidewindow_set_tail(s_slide_window,
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
	ack->h.addr.dlci = 0;/* ACK only on control dlci */
	ack->h.control = ACK;
	ack->h.sn = seq_num | (error << 7);
	ack->h.length.ea = 1;
	ack->h.length.len = 0;

	buf[0] = TS0710_BASIC_FLAG;
	ack->data[0] = ts0710_uart_crc_data(
		buf + ADDRESS_OFFSET, sizeof(struct short_frame));
	ack->data[1] = TS0710_BASIC_FLAG;

	mux_print(MSG_MSGDUMP, "Send ACK, sn=%x\n", ack->h.sn);

	ret = ts27010_uart_control_send(ts0710, buf, TS0710_FRAME_SIZE(0));

	FUNC_EXIT();
	return ret;
}

int ts27010_check_sequence_number(struct ts0710_con *ts0710, u8 sn)
{
	u8 ap_received_sn;
	int ret;
	FUNC_ENTER();

	ts27010_sequence_number_lock(s_ap_received_sn);
	ap_received_sn = ts27010_sequence_number_get(s_ap_received_sn);
	if (ap_received_sn == sn) {
		mux_print(MSG_DEBUG, "Got an expected frame, "
			"feedback ACK %x\n", ap_received_sn);
		send_ack(ts0710, ap_received_sn, ACK_OK);
		ts27010_sequence_number_inc(s_ap_received_sn);
		ret = 1;
	} else {
		mux_print(MSG_WARNING,
			"received SN %x is not expected SN: %x, "
			"discard data!\n",
			sn, ap_received_sn);
		mux_print(MSG_WARNING, "mux send bad-ack to bp\n");

		send_ack(ts0710, ap_received_sn, ACK_ERROR);
		ret = 0;
	}
	ts27010_sequence_number_unlock(s_ap_received_sn);
	FUNC_EXIT();
	return ret;
}
#endif

int ts27010_uart_td_open(void)
{
	FUNC_ENTER();
#ifdef PROC_DEBUG_MUX
	s_proc_mux = ts27010_uart_proc_alloc();
	if (!s_proc_mux)
		goto err;
#ifdef PROC_DEBUG_MUX_STAT
	s_proc_stat_mux = ts27010_uart_proc_stat_alloc();
	if (!s_proc_stat_mux)
		goto err;
#endif
#endif
#ifdef TS27010_UART_RETRAN
	s_timer_para = ts27010_alloc_timer_para(ts27010_uart_retran_worker);
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
	return 0;

err:
#ifdef TS27010_UART_RETRAN
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
		ts27010_uart_proc_free(s_proc_stat_mux, "muxuart_stat");
#endif
	if (s_proc_mux)
		ts27010_uart_proc_free(s_proc_mux, "muxuart");
#endif
	return -ENOMEM;
}

int ts27010_uart_td_init(void)
{
	FUNC_ENTER();

#ifdef CONFIG_WAKELOCK
	/* Init android sleep lock. */
	wake_lock_init(&s_mux_suspend_lock, WAKE_LOCK_SUSPEND, "mux_dispatch");
#ifdef TS27010_UART_RETRAN
	wake_lock_init(&s_mux_resend_lock, WAKE_LOCK_SUSPEND, "mux_resend");
#endif
#endif

#ifdef MUX_UART_LOGGER
	g_mux_uart_logger = ts27010_alloc_mux_logger();
	if (!g_mux_uart_logger)
		return -ENOMEM;
#endif

	FUNC_EXIT();
	return 0;
}

void ts27010_uart_td_close(void)
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
#ifdef PROC_DEBUG_MUX
#ifdef PROC_DEBUG_MUX_STAT
	if (s_proc_stat_mux)
		ts27010_uart_proc_free(s_proc_stat_mux, "muxuart_stat");
#endif
	if (s_proc_mux)
		ts27010_uart_proc_free(s_proc_mux, "muxuart");
#endif
	FUNC_EXIT();
}

void ts27010_uart_td_remove(void)
{
	FUNC_ENTER();
#ifdef CONFIG_WAKELOCK
	/* remove android sleep lock. */
#ifdef TS27010_UART_RETRAN
	wake_lock_destroy(&s_mux_resend_lock);
#endif
	wake_lock_destroy(&s_mux_suspend_lock);
#endif

#ifdef MUX_UART_LOGGER
	ts27010_free_mux_logger(g_mux_uart_logger);
#endif
	FUNC_EXIT();
}
