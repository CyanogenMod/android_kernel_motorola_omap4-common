/*
 * File: ts27010_mux.c
 *
 * Portions derived from rfcomm.c, original header as follows:
 *
 * Copyright (C) 2000, 2001  Axis Communications AB
 * Copyright (C) 2002, 2004, 2009 Motorola, Inc.7
 *
 * Author: Mats Friden <mats.friden@axis.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Exceptionally, Axis Communications AB grants discretionary and
 * conditional permissions for additional use of the text contained
 * in the company's release of the AXIS OpenBT Stack under the
 * provisions set forth hereunder.
 *
 * Provided that, if you use the AXIS OpenBT Stack with other files,
 * that do not implement functionality as specified in the Bluetooth
 * System specification, to produce an executable, this does not by
 * itself cause the resulting executable to be covered by the GNU
 * General Public License. Your use of that executable is in no way
 * restricted on account of using the AXIS OpenBT Stack code with it.
 *
 * This exception does not however invalidate any other reasons why
 * the executable file might be covered by the provisions of the GNU
 * General Public License.
 *
 */
#define DEBUG

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
#include <linux/fs.h>
#include <linux/tty.h>

#include <asm/system.h>

#include "ts27010.h"
#include "ts27010_mux.h"
#include "ts27010_ringbuf.h"
#include "ts27010_misc.h"
#include "ts27010_td.h"

#define CRC_VALID 0xcf

#define TS0710MUX_IO_DLCI_FC_ON 0x54F2
#define TS0710MUX_IO_DLCI_FC_OFF 0x54F3
#define TS0710MUX_IO_FC_ON 0x54F4
#define TS0710MUX_IO_FC_OFF 0x54F5

#define TS0710MUX_SERIAL_BUF_SIZE 2048

#define TEST_PATTERN_SIZE 4

enum recv_state {
	RECV_STATE_IDLE,
	RECV_STATE_ADDR,
	RECV_STATE_CONTROL,
#ifdef TS27010_UART_RETRAN
	RECV_STATE_SN,
#endif
	RECV_STATE_LEN,
	RECV_STATE_LEN2,
	RECV_STATE_DATA,
	RECV_STATE_END,
};

/* this tty is /dev/ttyHS3, a real UART */
struct tty_struct *ts27010mux_uart_tty;
struct workqueue_struct *g_mux_uart_queue;


/* there are 14 MUX UART channel and 13 MUX UART TTY device */
/* Mapping from TTY NO to DLCI */
static const u8 tty2dlci[TS0710_MAX_MUX] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
};

/* Mapping from DLC to TTY NO */
static const u8 dlci2tty[TS0710_MAX_CHN] = {
	0xFF, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
};

static unsigned long s_mux_kill_flags;
static u8 crctable[256];
static struct ts0710_con ts0710_connection;

extern struct tty_struct *ts27010_uart_tty_table[TS0710_MAX_MUX + 1];

static int ts0710_valid_dlci(u8 dlci)
{
	int ret;
	FUNC_ENTER();
	if ((dlci < TS0710_MAX_CHN) && (dlci >= 0))
		ret = 1;
	else
		ret = 0;
	FUNC_EXIT();
	return ret;
}

static int ts0710_valid_control(u8 control)
{
	switch (CLR_PF(control)) {
	case SABM:
	case UA:
	case DM:
	case DISC:
	case UIH:
#ifdef TS27010_UART_RETRAN
	case ACK:
#endif
		return 1;

	default:
		return 0;
	}
}

#ifdef TS27010_UART_RETRAN
static int ts0710_valid_sn(u8 sn)
{
	/* ACK correct: sn should be in 0x00 ~ 0x3F */
	/* ACK error: sn should be in 0x80 ~ 0xBF */
	if (sn & MAX_TRANS_SN)
		return 0;
	else
		return 1;
}
#endif

static void ts0710_crc_create_table(u8 table[])
{
	int i, j;

	u8 data;
	u8 code_word = 0xe0;
	u8 sr = 0;
	FUNC_ENTER();

	for (j = 0; j < 256; j++) {
		data = (u8) j;

		for (i = 0; i < 8; i++) {
			if ((data & 0x1) ^ (sr & 0x1)) {
				sr >>= 1;
				sr ^= code_word;
			} else {
				sr >>= 1;
			}

			data >>= 1;
			sr &= 0xff;
		}

		table[j] = sr;
		sr = 0;
	}
	FUNC_EXIT();
}

static u8 ts0710_crc_start(void)
{
	return 0xff;
}

static u8 ts0710_crc_calc(u8 fcs, u8 c)
{
	return crctable[fcs ^ c];
}

static u8 ts0710_crc_end(u8 fcs)
{
	return 0xff - fcs;
}

#ifdef TS27010_UART_RETRAN
static int ts0710_crc_check(u8 fcs)
{
	return fcs == CRC_VALID;
}
#endif

u8 ts0710_uart_crc_data(u8 *data, int length)
{
	u8 fcs = ts0710_crc_start();
	u8 ret;
	FUNC_ENTER();

	while (length--)
		fcs = ts0710_crc_calc(fcs, *data++);

	ret = ts0710_crc_end(fcs);
	FUNC_EXIT();
	return ret;
}

static void ts0710_pkt_set_header(u8 *data, int len, int addr_ea,
					 int addr_cr, int addr_dlci,
					 int control)
{
	struct short_frame *pkt
		= (struct short_frame *)(data + ADDRESS_OFFSET);
	FUNC_ENTER();

	pkt->h.addr.ea = addr_ea;
	pkt->h.addr.cr = addr_cr;
	pkt->h.addr.dlci = addr_dlci;
	pkt->h.control = control;

#ifdef TS27010_UART_RETRAN
	if (addr_dlci == CTRL_CHAN || control != UIH)
		pkt->h.sn = INDIFFERENT_SN;
#endif

	if ((len) > SHORT_PAYLOAD_SIZE) {
		struct long_frame *long_pkt
			= (struct long_frame *)(data + ADDRESS_OFFSET);
		long_pkt->h.length.ea = 0;
		long_pkt->h.length.l_len = len & 0x7F;
		long_pkt->h.length.h_len = (len >> 7) & 0xFF;
	} else {
		pkt->h.length.ea = 1;
		pkt->h.length.len = len;
	}
	FUNC_EXIT();
}

static void *ts0710_pkt_data(u8 *data)
{
	struct short_frame *pkt
		= (struct short_frame *)(data + ADDRESS_OFFSET);
	void *ret;
	FUNC_ENTER();

	if (pkt->h.length.ea == 1)
		ret = pkt->data;
	else
		ret = pkt->data+1;
	FUNC_EXIT();
	return ret;
}

static int ts0710_pkt_send(struct ts0710_con *ts0710, u8 *data)
{
	struct short_frame *pkt
		= (struct short_frame *)(data + ADDRESS_OFFSET);
	u8 *d;
	int len;
	int header_len;
	int res;
	FUNC_ENTER();

	if (pkt->h.length.ea == 1) {
		len = pkt->h.length.len;
		d = pkt->data;
		header_len = sizeof(*pkt);
	} else {
		struct long_frame *long_pkt =
			(struct long_frame *)(data + ADDRESS_OFFSET);
		len = (long_pkt->h.length.h_len << 7) |
			long_pkt->h.length.l_len;
		d = long_pkt->data;
		header_len = sizeof(*long_pkt);
	}

	/* set BASIC_FLAGS and FCS */
	data[0] = TS0710_BASIC_FLAG;
#ifdef TS27010_UART_RETRAN
	if (CLR_PF(pkt->h.control) == UIH) {
		d[len] = ts0710_uart_crc_data(data + ADDRESS_OFFSET,
			TS0710_FRAME_SIZE(len) - FCS_SIZE - FLAG_SIZE);
	} else {
#endif
		d[len] = ts0710_uart_crc_data(
			data + ADDRESS_OFFSET, header_len);
#ifdef TS27010_UART_RETRAN
	}
#endif
	d[len + 1] = TS0710_BASIC_FLAG;

	if (!ts27010mux_uart_tty) {
		mux_print(MSG_WARNING, "ldisc closed. discarding %d bytes\n",
			   TS0710_FRAME_SIZE(len));
		return TS0710_FRAME_SIZE(len);
	}

	if (CLR_PF(data[CONTROL_OFFSET]) != UIH
		|| (data[ADDRESS_OFFSET] >> 2 == 0)) {
		/*
		 * to delay frame sending in order that
		 * MUX has time to handle recved frame
		 * to avaoid BP retran 10 times failure panic,
		 * should change timeout in PN negotiation
		 */
		res = ts27010_uart_control_send(
			ts0710, data, TS0710_FRAME_SIZE(len));
	} else {
		res = ts27010_uart_uih_send(
			ts0710, data, TS0710_FRAME_SIZE(len));
	}
	if (res < 0) {
		mux_print(MSG_ERROR, "pkt write error %d\n", res);
		return res;
	} else if (res != TS0710_FRAME_SIZE(len)) {
		mux_print(MSG_ERROR, "short write %d < %d\n", res,
		       TS0710_FRAME_SIZE(len));
		return -EIO;
	} else {
		mux_print(MSG_DEBUG, "send %d successfully\n", res);
		res = 0;
	}

	FUNC_EXIT();
	return res;
}

static void ts0710_reset_dlci_data(struct dlci_struct *d)
{
	FUNC_ENTER();

	d->state = DISCONNECTED;
	d->flow_control = 0;
	d->clients = 0;
	d->mtu = DEF_TS0710_MTU;
	d->initiator = 0;

	FUNC_EXIT();
}

static void ts0710_reset_dlci(struct dlci_struct *d)
{
	FUNC_ENTER();

	ts0710_reset_dlci_data(d);
	init_waitqueue_head(&d->open_wait);
	init_waitqueue_head(&d->close_wait);
	init_waitqueue_head(&d->mux_write_wait);
	mutex_init(&d->lock);

	FUNC_EXIT();
}

static void ts0710_reset_channels(struct ts0710_con *ts0710)
{
	int j;
	FUNC_ENTER();

	for (j = 0; j < TS0710_MAX_CHN; j++)
		ts0710_reset_dlci(&ts0710->dlci[j]);
	FUNC_EXIT();
}

static void ts0710_init(struct ts0710_con *ts0710)
{
	int j;

	FUNC_ENTER();

	ts0710_crc_create_table(crctable);
	ts0710_reset_channels(ts0710);
	init_waitqueue_head(&ts0710->test_wait);
	ts0710->test_errs = 0;
	ts0710->be_testing = 0;

	for (j = 0; j < TS0710_MAX_MUX; j++)
		mutex_init(&ts0710->chan[j].write_lock);

	FUNC_EXIT();
}

static void ts0710_upon_disconnect(struct ts0710_con *ts0710)
{
	int j;
	FUNC_ENTER();

	for (j = 0; j < TS0710_MAX_CHN; j++) {
		ts0710->dlci[j].state = DISCONNECTED;
		wake_up_interruptible(&ts0710->dlci[j].open_wait);
		wake_up_interruptible(&ts0710->dlci[j].close_wait);
	}
	ts0710_reset_channels(ts0710);
	FUNC_EXIT();
}

static int ts27010_send_cmd(struct ts0710_con *ts0710, u8 dlci, u8 cmd)
{
	u8 frame[TS0710_FRAME_SIZE(0)];
	int ret;
	FUNC_ENTER();

	ts0710_pkt_set_header(frame, 0, 1,
		ts0710->dlci[dlci].initiator & 0x01,
		/* MCC_RSP, */dlci, SET_PF(cmd));
	ret = ts0710_pkt_send(ts0710, frame);
	FUNC_EXIT();
	return ret;
}


static int ts27010_send_ua(struct ts0710_con *ts0710, u8 dlci)
{
	int ret;
	FUNC_ENTER();
	mux_print(MSG_INFO, "sending UA packet to DLCI %d\n", dlci);
	ret = ts27010_send_cmd(ts0710, dlci, UA);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_dm(struct ts0710_con *ts0710, u8 dlci)
{
	int ret;
	FUNC_ENTER();
	mux_print(MSG_INFO, "sending DM packet to DLCI %d\n", dlci);
	ret = ts27010_send_cmd(ts0710, dlci, DM);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_sabm(struct ts0710_con *ts0710, u8 dlci)
{
	int ret;
	FUNC_ENTER();
	mux_print(MSG_INFO, "sending SABM packet to DLCI %d\n", dlci);
	ret = ts27010_send_cmd(ts0710, dlci, SABM);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_disc(struct ts0710_con *ts0710, u8 dlci)
{
	int ret;
	FUNC_ENTER();
	mux_print(MSG_INFO, "sending DISC packet to DLCI %d\n", dlci);
	ret = ts27010_send_cmd(ts0710, dlci, DISC);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_uih(struct ts0710_con *ts0710, u8 dlci,
			    u8 *frame, const u8 *data, int len)
{
	int ret;

	FUNC_ENTER();
	mux_print(MSG_DEBUG, "sending %d bytes UIH to DLCI %d\n",
		 len, dlci);
	ts0710_pkt_set_header(frame,
		len,
		1, /* EA */
		MCC_CMD, /* CR */
		dlci, /* channel */
		CLR_PF(UIH)); /* control */

	memcpy(ts0710_pkt_data(frame), data, len);
	ret = ts0710_pkt_send(ts0710, frame);

	FUNC_EXIT();
	return ret;
}

static void ts27010_mcc_set_header(u8 *frame, int len, int cr, int cmd)
{
	struct mcc_short_frame *mcc_pkt;
	FUNC_ENTER();

	ts0710_pkt_set_header(frame, sizeof(struct mcc_short_frame) + len,
			      1, MCC_CMD, CTRL_CHAN, CLR_PF(UIH));

	mcc_pkt = ts0710_pkt_data(frame);
	mcc_pkt->h.type.ea = EA;
	mcc_pkt->h.type.cr = cr;
	mcc_pkt->h.type.type = cmd;
	mcc_pkt->h.length.ea = EA;
	mcc_pkt->h.length.len = len;
	FUNC_EXIT();
}

static void *ts27010_mcc_data(u8 *frame)
{
	return ((struct mcc_short_frame *)ts0710_pkt_data(frame))->value;
}

static int ts27010_send_fcon(struct ts0710_con *ts0710, int cr)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(0)];
	int ret;
	FUNC_ENTER();

	mux_print(MSG_INFO, "sending FCON MCC\n");
	ts27010_mcc_set_header(frame, 0, cr, FCON);

	ret = ts0710_pkt_send(ts0710, frame);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_fcoff(struct ts0710_con *ts0710, int cr)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(0)];
	int ret;
	FUNC_ENTER();

	mux_print(MSG_INFO, "sending FCOFF MCC\n");
	ts27010_mcc_set_header(frame, 0, cr, FCOFF);

	ret = ts0710_pkt_send(ts0710, frame);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_pn(struct ts0710_con *ts0710, u8 prior, int frame_size,
			   u8 credit_flow, u8 credits, u8 dlci, u8 cr)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(sizeof(struct pn_msg_data))];
	struct pn_msg_data *pn;
	int ret;
	FUNC_ENTER();

	mux_print(MSG_INFO, "sending PN MCC\n");
	ts27010_mcc_set_header(frame, sizeof(struct pn_msg_data), cr, PN);

	pn = ts27010_mcc_data(frame);
	pn->res1 = 0;
	pn->res2 = 0;
	pn->dlci = dlci;
	pn->frame_type = 0;
	pn->credit_flow = credit_flow;
	pn->prior = prior;

#ifdef TS27010_UART_RETRAN
	pn->ack_timer = RETRAN_TIMEOUT;
	pn->max_nbrof_retrans = MAX_RETRAN_TIMES;
#else
	pn->ack_timer = 0;
	pn->max_nbrof_retrans = 0;
#endif
	pn->frame_sizel = frame_size & 0xff;
	pn->frame_sizeh = frame_size >> 8;
	pn->credits = credits;

	ret = ts0710_pkt_send(ts0710, frame);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_nsc(struct ts0710_con *ts0710, u8 type, int cr)
{
	int ret;
	FUNC_ENTER();
#if 1
	mux_print(MSG_INFO, "Received Non supported command response\n");
	ret = 0;
#else
	u8 frame[TS0710_MCC_FRAME_SIZE(sizeof(struct mcc_type))];
	struct mcc_type *t;

	mux_print(MSG_DEBUG, "sending NSC MCC\n");
	ts27010_mcc_set_header(frame, sizeof(struct mcc_type), cr, NSC);

	t = ts27010_mcc_data(frame);
	t->ea = 1;
	t->cr = mcc_is_cmd(type);
	t->type = type >> 2;

	ret = ts0710_pkt_send(ts0710, frame);
#endif
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_test_msg(struct ts0710_con *ts0710, u8 *d_buf,
	int size, u8 cr)
{
	int ret;
	u8 frame[TS0710_MCC_FRAME_SIZE(size)];
	u8 *t_data;
	FUNC_ENTER();

	mux_print(MSG_DEBUG, "sending TEST MCC\n");
	ts27010_mcc_set_header(frame, size, cr, TEST);
#ifdef TS27010_UART_RETRAN
	((struct short_frame *)frame)->h.sn = INDIFFERENT_SN;
#endif

	t_data = ts27010_mcc_data(frame);

	memcpy(t_data, d_buf, size);

	ret = ts0710_pkt_send(ts0710, frame);
	if (ret == TS0710_MCC_FRAME_SIZE(size))
		ret = 0;
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_msc(struct ts0710_con *ts0710,
			    u8 value, int cr, u8 dlci)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(sizeof(struct msc_msg_data))];
	struct msc_msg_data *msc;
	int ret;
	FUNC_ENTER();

	mux_print(MSG_INFO, "sending MSC MCC\n");
	ts27010_mcc_set_header(frame, sizeof(struct msc_msg_data), cr, MSC);

	msc = ts27010_mcc_data(frame);

	msc->dlci.ea = 1;
	msc->dlci.cr = 1;
	msc->dlci.dlci = dlci;

	msc->v24_sigs = value;

	ret = ts0710_pkt_send(ts0710, frame);
	FUNC_EXIT();
	return ret;
}

static void ts27010_handle_test(struct ts0710_con *ts0710, u8 type,
			       struct ts27010_ringbuf *rbuf,
			       int data_idx, int len)
{
	FUNC_ENTER();

	mux_print(MSG_INFO, "test command received\n");
	if (mcc_is_cmd(type)) {
		u8 *data;
		int i;

		data = (u8 *)kmalloc(len, GFP_KERNEL);
		if (!data) {
			mux_print(MSG_WARNING,
				"not enough memory for test data: %d\n", len);
			return;
		}
		for (i = 0; i < len; i++)
			data[i] = ts27010_ringbuf_peek(rbuf, data_idx + i);

		ts27010_send_test_msg(ts0710, data, len, MCC_RSP);
		kfree(data);
	} else {
		if (ts0710->be_testing) {
			int i;
			u8 data[TEST_PATTERN_SIZE];
			if (len != TEST_PATTERN_SIZE) {
				mux_print(MSG_ERROR,
					"reveived test on length:%d != %d\n",
					len, TEST_PATTERN_SIZE);
				ts0710->test_errs = TEST_PATTERN_SIZE;
				return;
			}

			for (i = 0; i < TEST_PATTERN_SIZE; i++)
				data[i] = ts27010_ringbuf_peek(
					rbuf, data_idx + i);

			ts0710->test_errs = 0;
			for (i = 0; i < TEST_PATTERN_SIZE; i++) {
				if (data[i] != (i & 0xFF))
					ts0710->test_errs++;
			}
			ts0710->be_testing = 0;	/* Clear the flag */
			wake_up_interruptible(&ts0710->test_wait);
		} else {
			mux_print(MSG_ERROR, "Err: shouldn't or late "
				"to get test cmd response\n");
		}
	}
	FUNC_EXIT();
}

static void ts27010_handle_msc(struct ts0710_con *ts0710, u8 type,
			       struct ts27010_ringbuf *rbuf,
			       int data_idx, int len)
{
	u8 dlci;
	u8 v24_sigs;
	struct dlci_struct *d;
	FUNC_ENTER();

	dlci = ts27010_ringbuf_peek(rbuf, data_idx) >> 2;
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return;
	}
	v24_sigs = ts27010_ringbuf_peek(rbuf, data_idx + 1);

	d = &ts0710->dlci[dlci];
	if ((d->state != CONNECTED)
	    && (d->state != FLOW_STOPPED)) {
		mux_print(MSG_ERROR, "DLCI %d is not connected: %d\n",
			dlci, d->state);
		ts27010_send_dm(ts0710, dlci);
		return;
	}

	if (mcc_is_cmd(type)) {
		mux_print(MSG_INFO, "received modem status command\n");
		if (v24_sigs & FC) {
			if (d->state == CONNECTED) {
				mux_print(MSG_WARNING,
					"flow off on dlci %d\n", dlci);
				d->state = FLOW_STOPPED;
			} else {
				mux_print(MSG_WARNING,
					"flow off on dlci %d but state: %d\n",
					dlci, d->state);
			}
		} else {
			if (d->state == FLOW_STOPPED) {
				d->state = CONNECTED;
				mux_print(MSG_WARNING,
					"flow on on dlci %d\n", dlci);
				wake_up_interruptible(&d->mux_write_wait);
			} else {
				mux_print(MSG_WARNING,
					"flow on on dlci %d but state: %d\n",
					dlci, d->state);
			}
		}
		ts27010_send_msc(ts0710, v24_sigs, MCC_RSP, dlci);
	} else {
		mux_print(MSG_INFO, "received modem status response\n");

		if (v24_sigs & FC)
			mux_print(MSG_INFO, "flow off accepted\n");
		else
			mux_print(MSG_INFO, "flow on accepted\n");
	}
	FUNC_EXIT();
}

static void ts27010_handle_pn(struct ts0710_con *ts0710, u8 type,
			      struct ts27010_ringbuf *rbuf,
			      int data_idx, int len)
{
	u8 dlci;
	u16 frame_size;
	struct pn_msg_data pn;
	int i;
	FUNC_ENTER();

	if (len != 8) {
		mux_print(MSG_ERROR, "reveived pn on length:%d != 8\n", len);
		return;
	}

	for (i = 0; i < 8; i++)
		((u8 *)&pn)[i] = ts27010_ringbuf_peek(rbuf, data_idx + i);

	dlci = pn.dlci;
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return;
	}
	frame_size = pn.frame_sizel | (pn.frame_sizeh << 8);

	if (mcc_is_cmd(type)) {
		mux_print(MSG_INFO,
			"received PN command with frame size %d\n",
			frame_size);

		/* TODO: this looks like it will only ever shrink mtu */
		frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
		ts27010_send_pn(ts0710, pn.prior, frame_size,
				0, 0, dlci, MCC_RSP);
		ts0710->dlci[dlci].mtu = frame_size;

		mux_print(MSG_INFO, "mtu set to %d on dlci%d\n",
			 frame_size, dlci);
	} else {
		mux_print(MSG_INFO,
			"received PN response with frame size %d\n",
			frame_size);

		frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
		ts0710->dlci[dlci].mtu = frame_size;

		mux_print(MSG_INFO, "mtu set to %d on dlci%d\n",
			 frame_size, dlci);

		if (ts0710->dlci[dlci].state == NEGOTIATING) {
			mutex_lock(&ts0710->dlci[dlci].lock);
			ts0710->dlci[dlci].state = CONNECTING;
			mutex_unlock(&ts0710->dlci[dlci].lock);
			wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		}
	}
	FUNC_EXIT();
}

static void ts27010_handle_mcc(struct ts0710_con *ts0710, u8 control,
			       struct ts27010_ringbuf *rbuf,
			       int data_idx, int len)
{
	u8 type;
	u8 mcc_len;
	u8 j;
	FUNC_ENTER();

	type = ts27010_ringbuf_peek(rbuf, data_idx++);
	len--;
	mcc_len = ts27010_ringbuf_peek(rbuf, data_idx++);
	len--;
	mcc_len >>= 1;/* remove EA bit */

	if (mcc_len != len) {
		mux_print(MSG_WARNING, "handle_mcc: mcc_len:%d != len:%d\n",
			   mcc_len, len);
	}

	switch (type >> 2) {
	case TEST:
		ts27010_handle_test(ts0710, type, rbuf, data_idx, len);
		break;

	case FCON:
		mux_print(MSG_WARNING,
			"received all channels flow control on\n");
		if ((type & 0x2) >> 1 == MCC_CMD) {
			for (j = 0; j < TS0710_MAX_CHN; j++)
				ts0710->dlci[j].state = CONNECTED;
			ts27010_send_fcon(ts0710, MCC_RSP);
		}
		break;

	case FCOFF:
		mux_print(MSG_WARNING,
			"received all channels flow control off\n");
		if ((type & 0x2) >> 1 == MCC_CMD) {
			for (j = 0; j < TS0710_MAX_CHN; j++)
				ts0710->dlci[j].state = FLOW_STOPPED;
			ts27010_send_fcoff(ts0710, MCC_RSP);
		}
		break;

	case MSC:
		ts27010_handle_msc(ts0710, type, rbuf, data_idx, len);
		break;

	case PN:
		ts27010_handle_pn(ts0710, type, rbuf, data_idx, len);
		break;

	case NSC:
		mux_print(MSG_WARNING,
			"received non supported cmd response\n");
		break;

	default:
		mux_print(MSG_WARNING, "received a non supported command\n");
		ts27010_send_nsc(ts0710, type, MCC_RSP);
		break;
	}
	FUNC_EXIT();
}

static void ts27010_flow_on(u8 dlci, struct ts0710_con *ts0710)
{
	int i;
	u8 tty_idx;
	struct tty_struct *tty;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	FUNC_ENTER();

	tty_idx = dlci2tty[dlci];
	WARN_ON(tty_idx >= TS0710_MAX_MUX || tty_idx < 0);

	tty = ts27010_uart_tty_table[tty_idx];
	if (!tty) {
		mux_print(MSG_ERROR,
			"DLCI %d no tty used to set throttle\n", dlci);
		return;
	}
	if (test_bit(TTY_THROTTLED, &tty->flags)) {
		mux_print(MSG_WARNING,
			"DLCI %d tty throttle set, can't unthrottle\n",
			dlci);
		return;
	}

	if ((ts0710->dlci[0].state != CONNECTED)
		&& (ts0710->dlci[0].state != FLOW_STOPPED)) {
		mux_print(MSG_WARNING, "BP is not connected: %d\n",
			ts0710->dlci[0].state);
		return;
	} else if ((d->state != CONNECTED) && (d->state != FLOW_STOPPED)) {
		mux_print(MSG_WARNING, "DLCI %d is not connected: %d\n",
			dlci, d->state);
		return;
	}

	if (!(d->flow_control)) {
		mux_print(MSG_WARNING,
			"DLCI %d is not in AP flow_control\n", dlci);
		return;
	}

	for (i = 0; i < 3; i++) {
		int ret;
		ret = ts27010_send_msc(ts0710,
			EA | RTC | RTR | DV, MCC_CMD, dlci);
		if (ret < 0) {
			mux_print(MSG_WARNING,
				"send flow on failed on dlci %d: %d\n",
				dlci, ret);
			continue;
		} else {
			mux_print(MSG_INFO,
				"send flow on on dlci %d to BP successfully\n",
				dlci);
			/* should sleep to wait for BP wake up? */
			d->flow_control = 0;
			break;
		}
	}
	FUNC_EXIT();
}

static void ts27010_flow_off(struct tty_struct *tty, u8 dlci,
			    struct ts0710_con *ts0710)
{
	int i;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	FUNC_ENTER();

	if (!tty) {
		mux_print(MSG_ERROR,
			"DLCI %d no tty used to set throttle\n", dlci);
		return;
	}
	if (test_bit(TTY_THROTTLED, &tty->flags) == 0) {
		mux_print(MSG_WARNING,
			"DLCI %d no tty throttle set\n", dlci);
		return;
	}

	if ((ts0710->dlci[0].state != CONNECTED)
		&& (ts0710->dlci[0].state != FLOW_STOPPED)) {
		mux_print(MSG_WARNING, "BP is not connected: %d\n",
			ts0710->dlci[0].state);
		return;
	} else if ((d->state != CONNECTED) && (d->state != FLOW_STOPPED)) {
		mux_print(MSG_WARNING, "DLCI %d is not connected: %d\n",
			dlci, d->state);
		return;
	}

	if (d->flow_control) {
		mux_print(MSG_WARNING,
			"DLCI %d has been in AP flow_control\n", dlci);
		return;
	}

	for (i = 0; i < 3; i++) {
		int ret;
		ret = ts27010_send_msc(ts0710,
			EA | FC | RTC | RTR | DV, MCC_CMD, dlci);
		if (ret < 0) {
			mux_print(MSG_WARNING,
				"send flow off failed on dlci %d: %d\n",
				dlci, ret);
			continue;
		} else {
			mux_print(MSG_INFO,
				"send flow off on dlci %d successfully\n",
				dlci);
			/* should sleep to wait for BP wake up? */
			d->flow_control = 1;
			break;
		}
	}
	FUNC_EXIT();
}

static void ts27010_handle_sabm(
	struct ts0710_con *ts0710, u8 control, int dlci)
{
	FUNC_ENTER();
	mux_print(MSG_INFO, "SABM received on dlci %d\n", dlci);

	if (ts0710_valid_dlci(dlci)) {
		struct dlci_struct *d = &ts0710->dlci[dlci];
		ts27010_send_ua(ts0710, dlci);

		d->state = CONNECTED;
		wake_up_interruptible(&d->open_wait);
	} else {
		mux_print(MSG_WARNING, "invalid dlci %d.\n", dlci);
		/* ts27010_send_dm(ts0710, dlci); */
	}
	FUNC_EXIT();
}

static void ts27010_handle_ua(struct ts0710_con *ts0710, u8 control, int dlci)
{
	FUNC_ENTER();
	mux_print(MSG_INFO, "UA packet received on dlci %d\n", dlci);

	if (ts0710_valid_dlci(dlci)) {
		struct dlci_struct *d = &ts0710->dlci[dlci];

		if (d->state == CONNECTING) {
			mutex_lock(&d->lock);
			d->state = CONNECTED;
			mutex_unlock(&d->lock);
			wake_up_interruptible(&d->open_wait);
		} else if (d->state == DISCONNECTING) {
			if (dlci == 0) {
				ts0710_upon_disconnect(ts0710);
			} else {
				mutex_lock(&d->lock);
				d->state = DISCONNECTED;
				mutex_unlock(&d->lock);
				wake_up_interruptible(&d->open_wait);
				wake_up_interruptible(&d->close_wait);
				/*
				ts0710_reset_dlci_data(d);
				*/
			}
			mux_print(MSG_INFO, "dlci %d disconnected\n", dlci);
		} else {
			if (dlci != 0 && test_bit(dlci, &s_mux_kill_flags)) {
				mutex_lock(&d->lock);
				d->state = DISCONNECTING;
				mutex_unlock(&d->lock);
				clear_bit(dlci, &s_mux_kill_flags);
				ts27010_send_disc(ts0710, dlci);
			}
			mux_print(MSG_WARNING,
				"invalid UA packet on dlci: %x state: %d\n",
				dlci, d->state);
		}
	} else {
		mux_print(MSG_WARNING, "invalid dlci %d\n", dlci);
	}
	FUNC_EXIT();
}

static void ts27010_handle_dm(struct ts0710_con *ts0710, u8 control, int dlci)
{
	int oldstate;
	FUNC_ENTER();

	mux_print(MSG_INFO, "DM packet received on dlci %d\n", dlci);

	if (dlci == 0) {
		oldstate = ts0710->dlci[0].state;
		ts0710_upon_disconnect(ts0710);
		mutex_lock(&ts0710->dlci[0].lock);
		if (oldstate == CONNECTING)
			ts0710->dlci[0].state = REJECTED;
		mutex_unlock(&ts0710->dlci[0].lock);
	} else if (ts0710_valid_dlci(dlci)) {
		struct dlci_struct *d = &ts0710->dlci[dlci];

		mutex_lock(&d->lock);
		if (d->state == CONNECTING)
			d->state = REJECTED;
		else
			d->state = DISCONNECTED;
		mutex_unlock(&d->lock);

		wake_up_interruptible(&d->open_wait);
		wake_up_interruptible(&d->close_wait);
		ts0710_reset_dlci_data(d);
	} else {
		mux_print(MSG_WARNING, "invalid dlci %d\n", dlci);
	}
	FUNC_EXIT();
}

static void ts27010_handle_disc(
	struct ts0710_con *ts0710, u8 control, int dlci)
{
	FUNC_ENTER();
	mux_print(MSG_INFO, "DISC packet received on dlci %d\n", dlci);

	if (!dlci) {
		ts27010_send_ua(ts0710, dlci);
		mux_print(MSG_INFO, "sending back UA\n");

		ts0710_upon_disconnect(ts0710);
	} else if (ts0710_valid_dlci(dlci)) {
		struct dlci_struct *d = &ts0710->dlci[dlci];
		ts27010_send_ua(ts0710, dlci);
		mux_print(MSG_INFO, "sending back UA\n");

		d->state = DISCONNECTED;
		wake_up_interruptible(&d->open_wait);
		wake_up_interruptible(&d->close_wait);
		/*
		ts0710_reset_dlci_data(d);
		*/
	} else {
		mux_print(MSG_WARNING, "invalid dlci %d\n", dlci);
	}
	FUNC_EXIT();
}

static void ts27010_handle_uih(struct ts0710_con *ts0710, u8 control, int dlci,
	struct ts27010_ringbuf *rbuf, int data_idx, int len)
{
	int tty_idx;
	FUNC_ENTER();

	if ((dlci >= TS0710_MAX_CHN)) {
		mux_print(MSG_ERROR, "invalid dlci %d\n", dlci);
		/*
		 * since dlci is invalid,
		 * so we can't send DM to an invalid dlci
		 */
		/* ts27010_send_dm(ts0710, dlci); */
		return;
	}

	if (GET_PF(control)) {
		mux_print(MSG_WARNING,
			"dlci %d: uih packet with P/F set, discarding %d\n",
				dlci, len);
		return;
	}

	if ((ts0710->dlci[dlci].state != CONNECTED)
		&& (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		mux_print(MSG_WARNING,
			"uih: dlci %d not connected(%d), discarding %d.\n",
			dlci, ts0710->dlci[dlci].state, len);
		ts27010_send_dm(ts0710, dlci);
		return;
	}

	if (dlci == 0) {
		mux_print(MSG_INFO, "handle mcc on DLCI 0\n");
		ts27010_handle_mcc(ts0710, control, rbuf, data_idx, len);
		return;
	}
	mux_print(MSG_DEBUG,
		"receive 0x%x bytes UIH from DLCI %d\n", len, dlci);

	if (len > ts0710->dlci[dlci].mtu) {
		mux_print(MSG_WARNING, "dlci %d: uih_len:%d "
			   "is bigger than mtu:%d, discarding.\n",
			    dlci, len, ts0710->dlci[dlci].mtu);
		/* TODO: should shrink the data length but not discard it? */
		len = ts0710->dlci[dlci].mtu;
	}
	if (len == 0) {
		mux_print(MSG_WARNING, "dlci %d: uih_len is 0.\n", dlci);
		return;
	}
	tty_idx = dlci2tty[dlci];
	mux_print(MSG_DEBUG, "receive data on DLCI %d, /dev/mux%d\n",
		 dlci, tty_idx);

	ts27010_tty_uart_send_rbuf(tty_idx, rbuf, data_idx, len);
	FUNC_EXIT();
}

static void ts27010_handle_frame(struct ts0710_con *ts0710,
	struct ts27010_ringbuf *rbuf, u8 addr,
	u8 control, int data_idx, int len)
{
	int dlci;
	FUNC_ENTER();

	dlci = ts0710_dlci(addr);
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return;
	}
	switch (CLR_PF(control)) {
	case SABM:
		ts27010_handle_sabm(ts0710, control, dlci);
		break;

	case UA:
		ts27010_handle_ua(ts0710, control, dlci);
		break;

	case DM:
		ts27010_handle_dm(ts0710, control, dlci);
		break;

	case DISC:
		ts27010_handle_disc(ts0710, control, dlci);
		break;

	case UIH:
		ts27010_handle_uih(ts0710, control, dlci, rbuf, data_idx, len);
		break;

	default:
		mux_print(MSG_ERROR, "illegal packet: 0x%02x\n", control);
		break;
	}
	FUNC_EXIT();
}

#ifdef TS27010_UART_RETRAN
static void ts27010_handle_retran_frame(struct ts0710_con *ts0710,
	struct ts27010_ringbuf *rbuf, u8 addr, u8 sn,
	u8 control, int data_idx, int len)
{
	FUNC_ENTER();

	if (CLR_PF(control) == ACK) {
		ts27010_uart_process_ack(ts0710, sn);
	} else {
		if (sn != INDIFFERENT_SN) {
			if (!ts27010_check_sequence_number(ts0710, sn))
				return;
		}
		ts27010_handle_frame(
			ts0710, rbuf, addr, control, data_idx, len);
	}
	FUNC_EXIT();
}
#endif

int ts27010_mux_uart_active(void)
{
	int ret;
	FUNC_ENTER();
	ret = ts27010mux_uart_tty != NULL;
	FUNC_EXIT();
	return ret;
}

/* call with dlci locked held */
static int ts27010_wait_for_close(struct ts0710_con *ts0710, int dlci)
{
	int ret;
	struct dlci_struct *d = &ts0710->dlci[dlci];

	mutex_unlock(&d->lock);
	ret = wait_event_interruptible_timeout(d->close_wait,
		d->state != DISCONNECTING, TS0710MUX_TIME_OUT);

	mutex_lock(&d->lock);
	if (ret == 0) {/* timeout */
		mux_print(MSG_INFO,
			"DLCI %d Wait for disconnecting timeout!\n", dlci);
		return -ETIMEDOUT;
	}

	if (ret == -ERESTARTSYS) {/* got signal */
		mux_print(MSG_ERROR,
			"DLCI %d Send DISC got signal! but I send 2 again\n",
			dlci);
		msleep(20);
		ts27010_send_disc(ts0710, dlci);
		msleep(30);
		ts27010_send_disc(ts0710, dlci);
		return -EAGAIN;
	}

	if (d->state != DISCONNECTED) {
		mux_print(MSG_ERROR,
			"DLCI %d wait for disconnected "
			"got invalid state: %d!\n",
			dlci, d->state);
		return -ENODEV;
	}

	mux_print(MSG_INFO, "DLCI %d disconnected!\n", dlci);
	return 0;
}

static int ts27010_close_channel(u8 dlci)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	int try;
	int retval = 0;
	FUNC_ENTER();

	mux_print(MSG_INFO, "closing dlci %d\n", dlci);

	mutex_lock(&d->lock);
	if (d->state == DISCONNECTED || d->state == REJECTED) {
		mux_print(MSG_WARNING, "DLCI %d has been closed!\n", dlci);
		if (d->clients > 0)
			d->clients--;
		retval = 0;
		goto EXIT;
	} else if (d->state == DISCONNECTING) {
		/* Reentry */
		mux_print(MSG_WARNING,
			"DLCI %d is being disconnected!\n", dlci);

		try = 8;
		while (try--) {
			retval = ts27010_wait_for_close(ts0710, dlci);
			if (retval == -ETIMEDOUT)
				continue;
			else
				break;
		}
		if (!retval && d->clients > 0)
			d->clients--;
		goto EXIT;
	}

	if ((d->clients > 1)
		&& (d->state == CONNECTED || d->state == FLOW_STOPPED)) {
		mux_print(MSG_WARNING,
			"DLCI %d not closed clients: %d!\n",
			dlci, d->clients);
		d->clients--;
		retval = 0;
		goto EXIT;
	}

	WARN_ON(d->clients != 1);
	/* do closing dlci: CONNECTED->DISCONNECTING->DISCONNECTED */
	d->state = DISCONNECTING;
	try = 10;
	while (try--) {
		retval = ts27010_send_disc(ts0710, dlci);
		if (retval) {
			mux_print(MSG_ERROR, "DLCI %d send DISC failed\n",
				dlci);
			msleep_interruptible(20);
			continue;
		}

		retval = ts27010_wait_for_close(ts0710, dlci);
		if (retval == -ETIMEDOUT)
			continue;
		else if (!retval)
			mux_print(MSG_INFO, "DLCI %d closed\n", dlci);
		break;
	}

	if (try < 0 && d->state != DISCONNECTED)
		retval = -EIO;

	/* TODO: force close, unclear if this is the right thing to do */
	if (d->state != DISCONNECTED) {
		if (dlci == 0) {/* Control Channel, disconnect all channels */
			ts0710_upon_disconnect(ts0710);
		} else {/* Other Channel */
			ts0710_reset_dlci_data(d);
			wake_up_interruptible(&d->close_wait);
		}
	}
	d->state = DISCONNECTED;
	d->clients = 0;
	if (dlci != 0) {
		mutex_lock(&ts0710->dlci[0].lock);
		ts0710->dlci[0].clients--;
		mutex_unlock(&ts0710->dlci[0].lock);
		mux_print(MSG_DEBUG, "Dec control ref %d\n",
			ts0710->dlci[0].clients);
	}

EXIT:
	mutex_unlock(&d->lock);
	FUNC_EXIT();

	return retval;
}

void ts27010_mux_uart_line_close(int line)
{
	int dlci;
	int closeCTRL = 1;
	int j;
	FUNC_ENTER();

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return;
	}
	ts27010_close_channel(dlci);

	for (j = 1; j < TS0710_MAX_CHN; j++) {
		struct dlci_struct *d = &ts0710_connection.dlci[j];
		if (d->state != DISCONNECTED) {
			closeCTRL = 0;
			break;
		}
	}
	if (closeCTRL) {
		mux_print(MSG_INFO, "All mux devices closed, "
			"will close control channel.\n");
		ts27010_close_channel(0);
	}
	FUNC_EXIT();
}

/* call with dlci locked held */
static int ts27010_wait_for_open(struct ts0710_con *ts0710, int dlci)
{
	int ret;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	FUNC_ENTER();

	mutex_unlock(&d->lock);
	ret = wait_event_interruptible_timeout(d->open_wait,
		d->state != CONNECTING, TS0710MUX_TIME_OUT);
	/*
	 * It is possible that d->state could have changed back to
	 * to connecting between being woken up and aquiring the lock.
	 * The side effect is that this open() will return turn -ENODEV.
	 */
	mutex_lock(&d->lock);
	if (ret == 0) {/* timeout */
		mux_print(MSG_INFO,
			"DLCI %d Wait for connecting timeout!\n", dlci);
		return -ETIMEDOUT;
	}

	if (ret == -ERESTARTSYS) {/* got signal */
		mux_print(MSG_ERROR,
			"DLCI %d Wait for connecting got signal!\n", dlci);
		set_bit(dlci, &s_mux_kill_flags);
		return -EAGAIN;
	}

	if (d->state == REJECTED) {/* got DM, rejected by BP */
		mux_print(MSG_ERROR,
			"DLCI %d Wait for connecting got rejected!\n", dlci);
		return -EREJECTED;
	}

	if (d->state != CONNECTED && d->state != FLOW_STOPPED) {
		/* other not connected */
		mux_print(MSG_ERROR,
			"DLCI %d Wait for connecting got invalid state: %d!\n",
			dlci, d->state);
		return -ENODEV;
	}
	mux_print(MSG_INFO, "DLCI %d connected!\n", dlci);
	FUNC_EXIT();

	return 0;
}

static int ts27010_wait_for_negotiated(struct ts0710_con *ts0710, int dlci)
{
	int ret;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	FUNC_ENTER();

	mutex_unlock(&d->lock);
	ret = wait_event_interruptible_timeout(d->open_wait,
		d->state != NEGOTIATING, TS0710MUX_TIME_OUT);
	/*
	 * It is possible that d->state could have changed back to
	 * to connecting between being woken up and aquiring the lock.
	 * The side effect is that this open() will return turn -ENODEV.
	 */

	mutex_lock(&d->lock);
	if (ret == 0) {/* timeout */
		mux_print(MSG_INFO,
			"DLCI %d Wait for negotiated timeout!\n", dlci);
		return -ETIMEDOUT;
	}

	if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
		mux_print(MSG_INFO,
			"DLCI %d connected!\n", dlci);
		return 0;
	}

	if (d->state == CONNECTING) {
		mux_print(MSG_INFO,
			"DLCI %d negotiated!\n", dlci);
		FUNC_EXIT();
		return 0;
	}

	if (ret == -ERESTARTSYS) {/* got signal */
		mux_print(MSG_ERROR,
			"DLCI %d Wait for negotiated got signal!\n", dlci);
		set_bit(dlci, &s_mux_kill_flags);
		return -EAGAIN;
	}

	if (d->state == REJECTED) {
		mux_print(MSG_ERROR,
			"DLCI %d wait for negotiated got rejected!\n", dlci);
		return -EREJECTED;
	} else {
		mux_print(MSG_ERROR,
			"DLCI %d wait for negotiated got invalid state: %d!\n",
			dlci, d->state);
		return -ENODEV;
	}
}

static int ts27010_open_channel(u8 dlci)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	struct dlci_struct *d = NULL;
	int try;
	int retval = -ENODEV;
	FUNC_ENTER();

	mux_print(MSG_DEBUG, "will open dlci %d\n", dlci);

	d = &ts0710->dlci[dlci];

	mutex_lock(&d->lock);
	if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
		mux_print(MSG_WARNING,
			"DLCI %d has been opened clients: %d!\n",
			dlci, d->clients);
		WARN_ON(d->clients < 1);
		d->clients++;
		retval = 0;
		goto EXIT;
	}
	if (d->state == NEGOTIATING) {
		try = 8;
		while (try--) {
			retval = ts27010_wait_for_negotiated(
				ts0710, dlci);
			if (retval == -ETIMEDOUT)
				continue;
			else
				break;
		}
		if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
			d->clients++;
			retval = 0;
			goto EXIT;
		}
	}
	if (d->state == CONNECTING) {
		try = 8;
		while (try--) {
			retval = ts27010_wait_for_open(ts0710, dlci);
			if (retval == -ETIMEDOUT)
				continue;
			else
				break;
		}
		if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
			d->clients++;
			retval = 0;
		}
		goto EXIT;
	}
	if (d->clients > 0) {/* state may be DISCONNECTING */
		mux_print(MSG_ERROR, "DLCI %d invalid state: %d, clients: %d!\n",
			dlci, d->state, d->clients);
		retval = -EREJECTED;
		goto EXIT;
	}
	if ((d->state != DISCONNECTED) && (d->state != REJECTED)) {
		/* dlci state is chois */
		mux_print(MSG_ERROR,
			"DLCI %d state is invalid: %d!\n", dlci, d->state);
		retval = -ENODEV;
		goto EXIT;
	}

	WARN_ON(d->clients != 0);
	/*
	 * First open DLCI, so send PN and SABM
	 * state: DISCONNECTED -> NEGOTIATING -> CONNECTING -> CONNECTED
	 *				      -> DISCONNECTED
	 *						    -> DISCONNECTED
	 */
	d->state = NEGOTIATING;
	d->initiator = 1;
	try = 10;
	while (try--) {
		retval = ts27010_send_pn(ts0710, 7, d->mtu, 0, 0, dlci, 1);
		if (retval) {
			mux_print(MSG_ERROR,
				"send pn to open channel 0x%x failed: %d\n",
				dlci, retval);
			msleep_interruptible(20);
			continue;
		}
		mux_print(MSG_INFO, "wait for 0x%x negotiated\n", dlci);

		retval = ts27010_wait_for_negotiated(ts0710, dlci);
		if (retval == -ETIMEDOUT)
			continue;
		break;
	}

	if (d->state == CONNECTING) {
		try = 10;
		while (try--) {
			retval = ts27010_send_sabm(ts0710, dlci);
			if (retval) {
				mux_print(MSG_ERROR,
					"send sabm to open channel 0x%x "
					"failed: %d\n",
					dlci, retval);
				msleep_interruptible(20);
				continue;
			}
			mux_print(MSG_INFO, "wait for 0x%x opened\n", dlci);

			retval = ts27010_wait_for_open(ts0710, dlci);
			if (retval == -ETIMEDOUT)
				continue;
			break;
		}
	}

	if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
		mux_print(MSG_INFO, "DLCI %d connected!\n", dlci);
		d->clients++;
	} else {
		mux_print(MSG_ERROR, "open DLCI %d failed: %d\n",
			dlci, d->state);
		d->state = REJECTED;/* DISCONNECTED; */
		retval = -ENODEV;
		/* TODO: should call reset_bp here? */
	}

	/* other ttys might be waiting on this dlci */
	wake_up_interruptible(&d->open_wait);

EXIT:
	mutex_unlock(&d->lock);
	FUNC_EXIT();
	return retval;
}

static int ts27010_open_ctrl_channel(int dlci)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	int try;
	int retval = -ENODEV;
	FUNC_ENTER();

	mutex_lock(&d->lock);
	if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
		/* opened */
		mux_print(MSG_INFO,
			"DLCI %d has been opened clients: %d!\n",
			dlci, d->clients);
		d->clients++;
		retval = 0;
		goto EXIT;
	} else if (d->state == CONNECTING) {
		/*
		 * Reentry, SABM has been sent
		 * and release the lock waiting for UA by someone
		 */
		try = 8;
		while (try--) {
			retval = ts27010_wait_for_open(ts0710, dlci);
			if (retval == -ETIMEDOUT)
				continue;
			else
				break;
		}
		if (!retval) /* open successfully */
			d->clients++;
		goto EXIT;
	} else if (d->clients > 0) {
		/* clients > 0 but state is DISCONNECTING */
		mux_print(MSG_ERROR,
			"DLCI %d state invalid: %d, clients: %d!\n",
			dlci, d->state, d->clients);
		retval = -EREJECTED;
		/*
		d->clients = 0;
		*/
		goto EXIT;
	}

	if ((d->state != DISCONNECTED) && (d->state != REJECTED)) {
		/* clients 0 but state unmatched, seems impossible here */
		mux_print(MSG_ERROR,
			"DLCI %d state invalid: %d!\n", dlci, d->state);
		retval = -ENODEV;
		goto EXIT;
	}

	/*
	 * First open DLCI 0, so send SABM
	 * state: DISCONNECTED -> CONNECTING -> CONNECTED
	 *				     -> DISCONNECTED
	 */
	WARN_ON(d->clients != 0);
	ts0710->initiator = 1;
	d->initiator = 1;
	d->state = CONNECTING;
	try = 10;
	while (try--) {
		retval = ts27010_send_sabm(ts0710, dlci);
		if (retval) {
			mux_print(MSG_ERROR,
				"send sabm to open control 0 failed: %d\n",
				retval);
			msleep_interruptible(20);
			continue;
		}
		mux_print(MSG_DEBUG, "wait for DLCI %d opened\n", dlci);
		retval = ts27010_wait_for_open(ts0710, dlci);
		if (retval == -ETIMEDOUT)
			continue;
		else if (!retval) {
			mux_print(MSG_INFO, "DLCI %d connected!\n", dlci);
			d->clients++;
			break;
		} else
			break;
	}

	if (try < 0 && d->state != CONNECTED && d->state != FLOW_STOPPED) {
		mux_print(MSG_ERROR, "open DLCI 0 failed: %d\n",
			d->state);
		retval = -ENODEV;
		/* TODO: should call reset_bp here? */
	}

	if (d->state == CONNECTING)
		d->state = DISCONNECTED;

	wake_up_interruptible(&d->open_wait);

EXIT:
	mutex_unlock(&d->lock);
	FUNC_EXIT();

	return retval;
}

int ts27010_mux_uart_line_open(int line)
{
	int dlci;
	int retval;
	FUNC_ENTER();

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return -EINVAL;
	}

	/* make sure channel 0 is opened */
	retval = ts27010_open_ctrl_channel(0);
	if (retval != 0) {
		mux_print(MSG_ERROR, "Can't open control DLCI 0!\n");
		return retval;
	}

	retval = ts27010_open_channel(dlci);
	FUNC_EXIT();
	return retval;
}

static int do_tty_write_wakeup(
	struct ts0710_con *ts0710, u8 dlci, struct tty_struct *tty)
{
	FUNC_ENTER();
	if (!tty) {
		mux_print(MSG_ERROR, "no tty device DLCI %d\n", dlci);
		return -ENODEV;
	}

	if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		mux_print(MSG_ERROR, "Flow stopped on DLCI %d\n", dlci);
		return -ENODEV;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		mux_print(MSG_ERROR, "DLCI %d not connected\n", dlci);
		return -ENODEV;
	}
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP))
		&& tty->ldisc->ops->write_wakeup) {
		(tty->ldisc->ops->write_wakeup)(tty);
	}
	wake_up_interruptible(&tty->write_wait);

#ifdef SERIAL_HAVE_POLL_WAIT
	wake_up_interruptible(&tty->poll_wait);
#endif

	FUNC_EXIT();
	return 0;
}

int ts27010_mux_uart_line_write(int line, const unsigned char *buf, int count)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	int err = -EINVAL;
	int dlci;
	int mtu;
	int sent = 0;

	FUNC_ENTER();

	if (count <= 0)
		return 0;

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return -EIO;
	}

	mtu = ts0710->dlci[dlci].mtu;
	if (mtu > TS0710MUX_SEND_BUF_SIZE) {
		mux_print(MSG_ERROR, "buffer overflow %d/%d\n",
			mtu, TS0710MUX_SEND_BUF_SIZE);
		BUG_ON(0);
	}

	while (sent < count) {
		if (ts0710->dlci[0].state == FLOW_STOPPED) {
			mux_print(MSG_INFO, "Flow stopped on all channels, "
				"including /dev/mux%d\n",
				line);
			wait_event_interruptible(
				ts0710->dlci[0].mux_write_wait,
				ts0710->dlci[0].state != FLOW_STOPPED);
			if (signal_pending(current)) {
				mux_print(MSG_WARNING, "/dev/mux%d(DLCI:%d) "
					"Wait for writing got signal!\n",
					line, 0);
				return -EAGAIN;
			} else if (ts0710->dlci[0].state != CONNECTED) {
				mux_print(MSG_WARNING,
					"write on DLCI %d "
					"while not connected\n", 0);
				return -EDISCONNECTED;
			}
		}
		if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
			mux_print(MSG_INFO,
				"Flow stoppedon /dev/mux%d\n",
				line);
			wait_event_interruptible(
				ts0710->dlci[dlci].mux_write_wait,
				ts0710->dlci[dlci].state != FLOW_STOPPED);
			if (signal_pending(current)) {
				mux_print(MSG_WARNING, "/dev/mux%d(DLCI:%d) "
					"Wait for writing got signal!\n",
					line, dlci);
				return -EAGAIN;
			} else if (ts0710->dlci[dlci].state != CONNECTED) {
				mux_print(MSG_WARNING,
					"write on DLCI %d "
					"while not connected\n", dlci);
				return -EDISCONNECTED;
			}
		}
		if (ts0710->dlci[dlci].state == CONNECTED) {
			int n = ((count - sent) > mtu) ? mtu : (count - sent);
			mux_print(MSG_DEBUG, "preparing to write %d bytes "
				 "to /dev/mux%d\n",
				n, line);
			mutex_lock(&ts0710->chan[line].write_lock);

			err = ts27010_send_uih(
				ts0710, dlci, ts0710->chan[line].buf,
				buf + sent, n);
			if (err < 0)
				goto ERR;

			sent += n;

			mutex_unlock(&ts0710->chan[line].write_lock);
		} else {
			mux_print(MSG_WARNING,
				"write on DLCI %d while not connected\n",
				dlci);
			return -EDISCONNECTED;
		}
	}
	do_tty_write_wakeup(ts0710, tty2dlci[line],
		ts27010_uart_tty_table[line]);
	mux_print(MSG_DEBUG,
		"write %d bytes to DLCI %d successfully\n", count, dlci);

	FUNC_EXIT();
	return count;

ERR:
	mutex_unlock(&ts0710->chan[line].write_lock);

	return err;
}

#define TS0710MUX_MAX_CHARS_IN_BUF 65535

int ts27010_mux_uart_line_chars_in_buffer(int line)
{
#if 1
	struct ts0710_con *ts0710 = &ts0710_connection;
	int retval;
	u8 dlci;

	retval = TS0710MUX_MAX_CHARS_IN_BUF;

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		goto out;
	}
	if (ts0710->dlci[0].state == FLOW_STOPPED) {
		mux_print(MSG_WARNING, "Flow stopped on all channels,"
			"returning MAX chars in buffer\n");
		goto out;
	} else if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		mux_print(MSG_WARNING,
			"Flow stopped, returning MAX chars in buffer\n");
		goto out;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		mux_print(MSG_ERROR, "DLCI %d not connected\n", dlci);
		goto out;
	}

	retval = 0;

out:
	return retval;
#else
	struct ts0710_con *ts0710 = &ts0710_connection;
	int ret;
	FUNC_ENTER();

	if (mutex_is_locked(&ts0710->chan[line].write_lock))
		ret = TS0710MUX_SERIAL_BUF_SIZE;
	else
		ret = 0;
	FUNC_EXIT();
	return ret;
#endif
}

int ts27010_mux_uart_line_write_room(int line)
{
#if 1
	struct ts0710_con *ts0710 = &ts0710_connection;
	int retval;
	u8 dlci;
	FUNC_ENTER();

	retval = 0;

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		goto out;
	}
	if (ts0710->dlci[0].state == FLOW_STOPPED) {
		mux_print(MSG_WARNING,
			"Flow stopped on all channels, returning ZERO\n");
		goto out;
	} else if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		mux_print(MSG_WARNING, "Flow stopped, returning ZERO\n");
		goto out;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		mux_print(MSG_ERROR, "DLCI %d not connected\n", dlci);
		goto out;
	}

	retval = ts0710->dlci[dlci].mtu;

out:
	FUNC_EXIT();
	return retval;
#else
	struct ts0710_con *ts0710 = &ts0710_connection;
	int ret;
	FUNC_ENTER();

	if (mutex_is_locked(&ts0710->chan[line].write_lock))
		ret = 0;
	else
		ret = TS0710MUX_SERIAL_BUF_SIZE;
	FUNC_EXIT();
	return ret;
#endif
}

void ts27010_mux_uart_line_throttle(int line)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	u8 dlci;

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return;
	}

	ts27010_flow_off(ts27010_uart_tty_table[line], dlci, ts0710);
}

void ts27010_mux_uart_line_unthrottle(int line)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	u8 dlci;

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return;
	}

	ts27010_flow_on(dlci, ts0710);
}

static int ts27010_send_test_cmd(struct ts0710_con *ts0710)
{
	u8 *d_buf = NULL; /* Data buffer */
	int retval = -EFAULT;
	int j;

	if (ts0710->be_testing) {
		/* Reentry */
		retval = wait_event_interruptible_timeout(ts0710->test_wait,
			ts0710->be_testing != 1, TS0710MUX_TIME_OUT * 3);
		if (retval == 0) {
			mux_print(MSG_ERROR,
				"Wait for Test_cmd response timeout!\n");
			retval = -EFAULT;
		}
		if (retval == -ERESTARTSYS) {
			mux_print(MSG_ERROR,
				"Wait for Test_cmd response got signal!\n");
			retval = -EAGAIN;
		}
		if (ts0710->be_testing == 0) {
			if (ts0710->test_errs == 0)
				retval = 0;
			else
				retval = -EFAULT;
		}
	} else {
		ts0710->be_testing = 1;	/* Set the flag */
		d_buf = (u8 *)kmalloc(TEST_PATTERN_SIZE, GFP_KERNEL);
		if (!d_buf) {
			ts0710->test_errs = TEST_PATTERN_SIZE;
			retval = -ENOMEM;
			goto EXIT;
		}

		for (j = 0; j < TEST_PATTERN_SIZE; j++)
			d_buf[j] = j & 0xFF;

		retval = ts27010_send_test_msg(ts0710, d_buf,
			TEST_PATTERN_SIZE, MCC_CMD);
		if (retval) {
			mux_print(MSG_ERROR,
				"Send Test_cmd error: %d!\n", retval);
			ts0710->test_errs = TEST_PATTERN_SIZE;
			retval = -EIO;
			goto EXIT;
		}
		retval = wait_event_interruptible_timeout(ts0710->test_wait,
			ts0710->be_testing != 1, TS0710MUX_TIME_OUT * 2);
		if (retval == 0) {
			mux_print(MSG_ERROR,
				"Wait for Test_cmd response timeout!\n");
			ts0710->test_errs = TEST_PATTERN_SIZE;
			retval = -EFAULT;
			goto EXIT;
		}
		if (retval == -ERESTARTSYS) {
			mux_print(MSG_ERROR,
				"Send Test_cmd response got signal!\n");
			ts0710->test_errs = TEST_PATTERN_SIZE;
			retval = -EAGAIN;
			goto EXIT;
		}
		if (ts0710->be_testing == 0) {
			if (ts0710->test_errs == 0) {
#ifdef PROC_DEBUG_MUX_STAT
				g_nStatUARTDrvIO = 1;
#endif
				retval = 0;
			} else {
				retval = -EFAULT;
			}
		}
EXIT:
		ts0710->be_testing = 0;	/* Clear the flag */
		wake_up_interruptible(&ts0710->test_wait);

		kfree(d_buf);
	}
	return retval;
}

int ts27010_mux_uart_line_ioctl(unsigned int cmd, unsigned long arg, int line)
{
	int dlci;
	int ret;
	struct ts0710_con *ts0710 = &ts0710_connection;
	FUNC_ENTER();

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return -EINVAL;
	}

	switch (cmd) {
	case TS0710MUX_IO_MSC_HANGUP:
		ret = ts27010_send_msc(ts0710, EA | RTR | DV, MCC_CMD, dlci);
		if (ret < 0) {
			mux_print(MSG_ERROR,
				"ts27010_send_msc: %d, msc_hangup[%d]=0\n",
				ret, dlci);
			return -EAGAIN;
		} else {
			mux_print(MSG_INFO,
				"TS0710MUX_IO_MSC_HANGUP on channel %d\n",
				dlci);
			return 0;
		}
		return 0;

	case TS0710MUX_IO_TEST_CMD:
		ts27010_send_test_cmd(ts0710);
		return 0;

	default:
		break;
	}
	FUNC_EXIT();

	return -ENOIOCTLCMD;
}

void ts27010_mux_uart_line_flush_buffer(int line)
{
	FUNC_ENTER();

	do_tty_write_wakeup(&ts0710_connection, tty2dlci[line],
		ts27010_uart_tty_table[line]);

	FUNC_EXIT();
}

int ts27010_mux_uart_mux_open(void)
{
	int err;
	FUNC_ENTER();

#ifndef MUX_UART_UT
	err = ts27010_tty_uart_open();
	if (err)
		goto err0;
	err = ts27010_uart_td_open();
	if (err)
		goto err1;
#endif

	s_mux_kill_flags = 0;

	FUNC_EXIT();
	return 0;

err1:
	ts27010_tty_uart_close();
err0:
	return err;
}

/* Assumption: all mux devices have been closed before bellow called */
/* need send DISC to disconnect all unclosed channels? */
void ts27010_mux_uart_mux_close(void)
{
	int j;
	FUNC_ENTER();

	s_mux_kill_flags = 0;
	for (j = 0; j < TS0710_MAX_CHN; j++) {
		struct dlci_struct *d = &ts0710_connection.dlci[j];
		if (j != 0 && (d->state != DISCONNECTED || d->clients != 0)) {
			mux_print(MSG_ERROR,
				"DLCI %d not be closed, state: %d\n",
				j, d->state);
			/* TODO:
			 * if rild crashed, UART line discipline will be closed
			 * but mux devices not be closed
			 * and BP channels still connected,
			 * reconnect may encounter error
			ts27010_close_channel(j);
			BUG_ON(d->state != DISCONNECTED);
			BUG_ON(d->clients != 0);
			 */
		}
		ts0710_reset_dlci_data(d);
		/*/
		wake_up_interruptible(&d->open_wait);
		wake_up_interruptible(&d->close_wait);
		/*/
	}
#ifndef MUX_UART_UT
	ts27010_tty_uart_close();
	ts27010_uart_td_close();
#endif

	FUNC_EXIT();
}

#ifdef PROC_DEBUG_MUX_STAT
static int s_nMuxConsumed;
static int s_nMuxProtRecved;
static int s_nMuxDataRecved;
static int s_nReadFromeBuf;
static int s_nDiscard;

void ts27010_mux_uart_mux_stat_clear(void)
{
	s_nMuxConsumed = 0;
	s_nMuxProtRecved = 0;
	s_nMuxDataRecved = 0;
	s_nReadFromeBuf = 0;
	s_nDiscard = 0;
}

void ts27010_mux_uart_mux_stat(void)
{
	mux_print(MSG_ERROR, "Mux consumed: %d, protocol recved: %d, "
		"data recved: %d\n",
		s_nMuxConsumed, s_nMuxProtRecved, s_nMuxDataRecved);
	mux_print(MSG_ERROR, "Mux read from buffer: %d, discard: %d\n",
		s_nReadFromeBuf, s_nDiscard);
}
#endif

static void ts27010_mux_dump_ringbuf(
	struct ts27010_ringbuf *rbuf, int start_flag)
{
	mux_print(MSG_ERROR, "ringbuf size: %d, start_flag: %d\n",
		ts27010_ringbuf_level(rbuf), start_flag);
	if (rbuf->head > rbuf->tail) {
		mux_uart_hexdump(MSG_ERROR, "dump ringbuf",
			__func__, __LINE__,
			rbuf->buf + rbuf->tail,
			rbuf->head - rbuf->tail);
	} else {
		mux_uart_hexdump(MSG_ERROR, "dump ringbuf",
			__func__, __LINE__,
			rbuf->buf + rbuf->tail,
			rbuf->len - rbuf->tail);
		mux_uart_hexdump(MSG_ERROR, "dump ringbuf",
			__func__, __LINE__, rbuf->buf,
			rbuf->head);
	}
}

void ts27010_mux_uart_recv(struct ts27010_ringbuf *rbuf)
{
	int i;
	u8 c;
	int state = RECV_STATE_IDLE;
	int count; /* data length in ringbuffer */
	/* data length which should be removed from ringbuffer */
	int consume_idx = -1;
	int data_idx = 0; /* UIH data start position */
	int start_flag = 0; /* MUX frame start position */

	u8 addr = 0;
	u8 control = 0;
#ifdef TS27010_UART_RETRAN
	u8 sn = INDIFFERENT_SN;
	int crc_error;
#endif
	int len = 0;
	u8 fcs = 0;

	FUNC_ENTER();

	/* get data size in buffer */
	count = ts27010_ringbuf_level(rbuf);
#ifdef PROC_DEBUG_MUX_STAT
	if (g_nStatUARTDrvIO)
		s_nReadFromeBuf += count;
#endif

	mux_print(MSG_MSGDUMP, "ringbuf level: %d\n", count);
	for (i = 0; i < count; i++) {
		if (state != RECV_STATE_DATA)
			c = ts27010_ringbuf_peek(rbuf, i);

		switch (state) {
		case RECV_STATE_IDLE:
			if (c == TS0710_BASIC_FLAG) {
				/* begin a MUX frame header */
				mux_print(MSG_MSGDUMP,
					"state transit IDLE->ADDR\n");
				fcs = ts0710_crc_start();
				state = RECV_STATE_ADDR;
				/* save MUX frame start position */
				start_flag = i;
			} else {/* discard non-MUX data */
				consume_idx = i;
				mux_print(MSG_MSGDUMP,
					"discard non-MUX data: %02x\n", c);
#ifdef PROC_DEBUG_MUX_STAT
				if (g_nStatUARTDrvIO) {
					mux_print(MSG_ERROR,
						"discard non-MUX data: %02x\n",
						c);
					s_nDiscard++;
				}
#endif
			}
			break;

		case RECV_STATE_ADDR:
			if (!ts0710_valid_dlci(c >> 2)) {
				/*
				 * discard the fisrt F9
				 */
				mux_print(MSG_WARNING,
					"RX wrong data %02x, Drop msg.\n", c);
				ts27010_mux_dump_ringbuf(rbuf, start_flag);
				consume_idx = start_flag;
				i = start_flag;/* find next F9, i will inc */
				state = RECV_STATE_IDLE;
#ifdef PROC_DEBUG_MUX_STAT
				if (g_nStatUARTDrvIO)
					s_nDiscard++;
#endif
			} else {
				fcs = ts0710_crc_calc(fcs, c);
				addr = c;
				state = RECV_STATE_CONTROL;
				mux_print(MSG_MSGDUMP,
					"state transit: %02x ADDR->CTRL\n",
					addr);
			}
			break;

		case RECV_STATE_CONTROL:
			if (!ts0710_valid_control(c)) {
				/*
				 * discard the fisrt F9
				 */
				mux_print(MSG_WARNING,
					"RX wrong data %02x, Drop msg.\n", c);
				ts27010_mux_dump_ringbuf(rbuf, start_flag);
				consume_idx = start_flag;
				i = start_flag;/* find next F9, i will inc */
				state = RECV_STATE_IDLE;
#ifdef PROC_DEBUG_MUX_STAT
				if (g_nStatUARTDrvIO)
					s_nDiscard++;
#endif
			} else {
				fcs = ts0710_crc_calc(fcs, c);
				control = c;
				mux_print(MSG_MSGDUMP,
					"state transit: %02x CTRL->SN\n",
					control);
#ifdef TS27010_UART_RETRAN
				state = RECV_STATE_SN;
			}
			break;

		case RECV_STATE_SN:
			if (!ts0710_valid_sn(c)) {
				/*
				 * discard the fisrt F9
				 */
				mux_print(MSG_WARNING,
					"RX wrong data %02x, Drop msg.\n", c);
				ts27010_mux_dump_ringbuf(rbuf, start_flag);
				consume_idx = start_flag;
				i = start_flag; /* find next F9, i will inc */
				state = RECV_STATE_IDLE;
#ifdef PROC_DEBUG_MUX_STAT
				if (g_nStatUARTDrvIO)
					s_nDiscard++;
#endif
			} else {
				fcs = ts0710_crc_calc(fcs, c);
				sn = c;
				mux_print(MSG_MSGDUMP,
					"state transit: %02x SN->LEN\n", sn);
#endif /* TS27010_UART_RETRAN */
				state = RECV_STATE_LEN;
			}
			break;

		case RECV_STATE_LEN:
			fcs = ts0710_crc_calc(fcs, c);
			len = c >> 1;
			if (c & 0x1) {
				data_idx = i + 1;
				mux_print(MSG_MSGDUMP,
					"state transit: %02x LEN->DATA\n",
					len);
				state = RECV_STATE_DATA;
			} else {
				mux_print(MSG_MSGDUMP,
					"state transit: %02x LEN->LEN2\n",
					len);
				state = RECV_STATE_LEN2;
			}
			break;

		case RECV_STATE_LEN2:
			fcs = ts0710_crc_calc(fcs, c);
			len |= c << 7;
			data_idx = i + 1;
			/* FCS: (len + data_idx) */
			/* End F9: (len + data_idx + 1) */
			if (len > DEF_TS0710_MTU) {
				/*
				 * huge frame, discard the fisrt F9
				 */
				mux_print(MSG_ERROR, "wrong length: %d-%d-%d, "
					"out of rbuf size or MTU, drop msg.\n",
					len + data_idx, len, count);
				ts27010_mux_dump_ringbuf(rbuf, start_flag);
				consume_idx = start_flag;
				i = start_flag;/* find next F9, i will inc */
				state = RECV_STATE_IDLE;
#ifdef PROC_DEBUG_MUX_STAT
				if (g_nStatUARTDrvIO)
					s_nDiscard++;
#endif
				break;
			} else if ((len + data_idx + 1) >= count) {
				mux_print(MSG_DEBUG,
					"unintegrated frame: %d-%d-%d, "
					"wait ...\n",
					len, len + data_idx + 1, count);
			}
			mux_print(MSG_MSGDUMP,
				"state transit: %04x LEN2->DATA\n", len);
			state = RECV_STATE_DATA;
			break;

		case RECV_STATE_DATA:
#ifdef TS27010_UART_RETRAN
			/* for uart, UIH -> UI frame, so need check data FCS */
			c = ts27010_ringbuf_peek(rbuf, i);
			fcs = ts0710_crc_calc(fcs, c);
			if (i == data_idx + len) {
				/* FCS byte */
				mux_print(MSG_MSGDUMP,
					"recved FCS: %x, calced FCS: %x\n",
					c, fcs);
				state = RECV_STATE_END;
			}
#else /* TS27010_UART_RETRAN */
			if (i == data_idx + len) {
				/* FCS byte */
				c = ts27010_ringbuf_peek(rbuf, i);
				fcs = ts0710_crc_calc(fcs, c);
				mux_print(MSG_DEBUG,
					"recved FCS: %x, calced FCS: %x\n",
					c, fcs);
				state = RECV_STATE_END;
			}
#endif /* TS27010_UART_RETRAN */
			else if (i > data_idx + len) {
				mux_print(MSG_ERROR, "overflow: %d, %d, %d\n",
					i, data_idx, len);
				ts27010_mux_dump_ringbuf(rbuf, start_flag);
			}
			break;

		case RECV_STATE_END:
#ifdef TS27010_UART_RETRAN
			crc_error = ts0710_crc_check(fcs);
			if (c == TS0710_BASIC_FLAG && crc_error) {
				/* OK, we got a whole MUX frame */
				consume_idx = i;
#ifndef MUX_UART_UT
				ts27010_handle_retran_frame(
					&ts0710_connection, rbuf, addr,
					sn, control, data_idx, len);
#else
#endif

#ifdef PROC_DEBUG_MUX_STAT
				if (g_nStatUARTDrvIO) {
					s_nMuxProtRecved
						+= TS0710_FRAME_SIZE(len);
					s_nMuxDataRecved += len + 7;
				}
#endif
#else /* TS27010_UART_RETRAN */
			/* don't do CRC check according to BP request */
			if (c == TS0710_BASIC_FLAG) {
				/* && ts0710_crc_check(fcs)) { */
				/* OK, we got a whole MUX frame */
				consume_idx = i;
#ifndef MUX_USB_UT
				ts27010_handle_frame(
					&ts0710_connection, rbuf,
					addr, control, data_idx, len);
#else
#endif

#ifdef PROC_DEBUG_MUX_STAT
				if (g_nStatUARTDrvIO) {
					s_nMuxProtRecved
						+= TS0710_FRAME_SIZE(len);
					s_nMuxDataRecved += len + 7;
				}
#endif
#endif /* TS27010_UART_RETRAN */
			} else {
				/*
				 * if a frame is splitted to two frames by BP,
				 * not read the partial frame
				 * until full frame arrived.
				 */
				mux_print(MSG_WARNING, "lost synchronization, "
					"discard a frame flag\n");
				ts27010_mux_dump_ringbuf(rbuf, start_flag);
				consume_idx = start_flag;
				i = start_flag;/* find next F9, i will inc */
			}
			state = RECV_STATE_IDLE;
			break;
		default:
			mux_print(MSG_ERROR, "Unknown MUX state: %d\n", state);
#ifdef PROC_DEBUG_MUX_STAT
			if (g_nStatUARTDrvIO)
				s_nDiscard++;
#endif
			break;
		}
	}
#ifdef PROC_DEBUG_MUX_STAT
	if (g_nStatUARTDrvIO)
		s_nMuxConsumed += consume_idx + 1;
#endif
	ts27010_ringbuf_consume(rbuf, consume_idx + 1);

	FUNC_EXIT();
}

static int __init mux_init(void)
{
	int err;

	FUNC_ENTER();

	/*/ MSG_MSGDUMP; MSG_INFO; MSG_DEBUG; /*/
	g_mux_uart_print_level = MSG_INFO;
	g_mux_uart_dump_frame = 0;
	g_mux_uart_dump_user_data = 0;

#ifdef PROC_DEBUG_MUX_STAT
	ts27010_mux_uart_mux_stat_clear();
#endif

	/* init channels state and some locks */
	ts0710_init(&ts0710_connection);

#ifdef MUX_UART_UT
	test_ts27010_uart();
	return 0;
#endif

	g_mux_uart_queue = create_workqueue("mux_uart_queue");
	if (!g_mux_uart_queue) {
		mux_print(MSG_ERROR, "Create mux work queue failed!\n");
		return -ENOMEM;
	}

	/* register a tty line discipline */
	err = ts27010_ldisc_uart_init();
	if (err != 0) {
		mux_print(MSG_ERROR, "error %d registering line disc.\n", err);
		return err;
	}

	/* register mux tty devices */
	err = ts27010_tty_uart_init();
	if (err != 0) {
		mux_print(MSG_ERROR, "error %d registering tty.\n", err);
		goto err0;
	}

	/* init td related data */
	err = ts27010_uart_td_init();
	if (err != 0) {
		mux_print(MSG_ERROR, "error %d td init.\n", err);
		goto err1;
	}

	mux_print(MSG_INFO, "mux over UART registered\n");

	FUNC_EXIT();
	return 0;

err1:
	ts27010_tty_uart_remove();
err0:
	ts27010_ldisc_uart_remove();
	return err;
}

static void __exit mux_exit(void)
{
	FUNC_ENTER();

	ts27010_uart_td_remove();
	ts27010_tty_uart_remove();
	ts27010_ldisc_uart_remove();
	destroy_workqueue(g_mux_uart_queue);

	mux_print(MSG_INFO, "mux over UART unregistered\n");
	FUNC_EXIT();
}


module_init(mux_init);
module_exit(mux_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TS 07.10 Multiplexer for TD-SCDMA modem over UART");
