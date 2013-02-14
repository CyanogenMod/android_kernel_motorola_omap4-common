#include <linux/module.h>
#include <linux/types.h>
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

#include "ts27010_mux.h"
#include "ts27010_ringbuf.h"

#define TS0710MUX_TIME_OUT 2500
#define TS0710MUX_CLOSE_TIME_OUT 90
#define CRC_VALID 0xcf
#define TS0710MUX_SEND_BUF_SIZE (TS0710_FRAME_SIZE(DEF_TS0710_MTU))
#define TS0710MUX_SERIAL_BUF_SIZE 2048

#ifdef SUPPORT_MUX_CMD_DATA_TAG
#define CMDTAG 0x55
#define DATATAG 0xAA
#endif

enum recv_state {
	RECV_STATE_IDLE,
	RECV_STATE_ADDR,
	RECV_STATE_CONTROL,
	RECV_STATE_LEN,
	RECV_STATE_LEN2,
	RECV_STATE_DATA,
	RECV_STATE_END,
};

/* Bit number in flags of mux_send_struct */
struct tty_struct *qcspi_mux_tty;
static u8 crctable[256];
static struct ts0710_con ts0710_connection;

#define DBG_DATA	(1<<0)
#define DBG_CMD		(1<<1)
#define DBG_VERBOSE	(1<<2)

#ifdef DEBUG
static int debug;

module_param_named(debug_level, debug, int, S_IRUGO | S_IWUSR);

#define ts_debug(level, format, arg...)		\
	if (debug & level)			\
		pr_debug(format , ## arg);

#define TS0710_DBG_BUF_SIZE	2048
static unsigned char dbg_buf[TS0710_DBG_BUF_SIZE];

static void ts27010_debughex(int level, const char *header,
			     const u8 *buf, int len)
{
	int i;
	int c;

	if (len <= 0)
		return;

	c = 0;
	for (i = 0; (i < len) && (c < (TS0710_DBG_BUF_SIZE - 3)); i++) {
		sprintf(&dbg_buf[c], "%02x ", buf[i]);
		c += 3;
	}
	dbg_buf[c] = 0;

	ts_debug(level, "%s%s\n", header, dbg_buf);
}

static void ts27010_debugrbufhex(int level, const char *header,
				 struct ts27010_ringbuf *rbuf,
				 int idx, int len)
{
	int i;
	int c;

	if (len <= 0)
		return;

	c = 0;
	for (i = 0; (i < len) && (c < (TS0710_DBG_BUF_SIZE - 3)); i++) {
		sprintf(&dbg_buf[c], "%02x ",
			ts27010_ringbuf_peek(rbuf, idx+i));
		c += 3;
	}
	dbg_buf[c] = 0;

	ts_debug(level, "%s%s\n", header, dbg_buf);
}

static void ts27010_debugrbuf(int level, const char *header,
			      struct ts27010_ringbuf *rbuf,
			      int idx, int len)
{
	int i;

	len = min(TS0710_DBG_BUF_SIZE - 1, len);

	for (i = 0; i < len; i++)
		dbg_buf[i] = ts27010_ringbuf_peek(rbuf, idx+i);

	if (dbg_buf[i-1] == '\n')
		dbg_buf[i-1] = '\0';
	else
		dbg_buf[i] = '\0';

	ts_debug(level, "%s%s\n", header, dbg_buf);
}

static void ts27010_debugstr(int level, const char *header,
			     const char *buf, int len)
{
	if (len <= 0)
		return;

	len = min(TS0710_DBG_BUF_SIZE - 1, len);

	memcpy(dbg_buf, buf, len);

	if (dbg_buf[len-1] == '\n')
		dbg_buf[len-1] = '\0';
	else
		dbg_buf[len] = '\0';

	ts_debug(level, "%s%s\n", header, dbg_buf);
}

#else /* DEBUG */
static void ts27010_debughex(int level, const char *header,
			     const u8 *buf, int len) { }
static void ts27010_debugrbufhex(int level, const char *header,
				 struct ts27010_ringbuf *rbuf,
				 int idx, int len) { }
static void ts27010_debugrbuf(int level, const char *header,
			      struct ts27010_ringbuf *rbuf,
			      int idx, int len) { }
static void ts27010_debugstr(int level, const char *header,
			     const char *buf, int len) { }
#define ts_debug(level, format, arg...)	do {	\
	} while (0)

#endif /* DEBUG */


static int ts0710_valid_dlci(u8 dlci)
{
	if ((dlci < QCSPIMUX_MAX_CHN) && (dlci > 0))
		return 1;
	else
		return 0;
}

static void ts0710_crc_create_table(u8 table[])
{
	int i, j;

	u8 data;
	u8 code_word = 0xe0;
	u8 sr = 0;

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

static int ts0710_crc_check(u8 fcs)
{
	return fcs == CRC_VALID;
}

static u8 ts0710_crc_data(u8 *data, int length)
{
	u8 fcs = ts0710_crc_start();

	while (length--)
		fcs = ts0710_crc_calc(fcs, *data++);

	return ts0710_crc_end(fcs);
}

static void ts0710_pkt_set_header(u8 *data, int len, int addr_ea,
					 int addr_cr, int addr_dlci,
					 int control)
{
	struct short_frame *pkt = (struct short_frame *)(data + 1);

	pkt->h.addr.ea = addr_ea;
	pkt->h.addr.cr = addr_cr;
	pkt->h.addr.d = addr_dlci & 0x1;
	pkt->h.addr.server_chn = addr_dlci >> 1;
	pkt->h.control = control;

	if ((len) > SHORT_PAYLOAD_SIZE) {
		struct long_frame *long_pkt = (struct long_frame *)(data + 1);
		long_pkt->h.length.ea = 0;
		long_pkt->h.length.l_len = len & 0x7F;
		long_pkt->h.length.h_len = (len >> 7) & 0xFF;
	} else {
		pkt->h.length.ea = 1;
		pkt->h.length.len = len;
	}
}

static void *ts0710_pkt_data(u8 *data)
{
	struct short_frame *pkt = (struct short_frame *)(data + 1);
	if (pkt->h.length.ea == 1)
		return pkt->data;
	else
		return pkt->data+1;
}

static int ts0710_pkt_send(struct ts0710_con *ts0710, u8 *data)
{
	struct short_frame *pkt = (struct short_frame *)(data + 1);
	u8 *d;
	int len;
	int header_len;
	int res;

	if (pkt->h.length.ea == 1) {
		len = pkt->h.length.len;
		d = pkt->data;
		header_len = sizeof(*pkt);
	} else {
		struct long_frame *long_pkt = (struct long_frame *)(data + 1);
		len = (long_pkt->h.length.h_len << 7) |
			long_pkt->h.length.l_len;
		d = pkt->data+1;
		header_len = sizeof(*long_pkt);
	}

	data[0] = TS0710_BASIC_FLAG;
	d[len] = ts0710_crc_data(data+1, header_len);
	d[len+1] = TS0710_BASIC_FLAG;

	ts27010_debughex(DBG_VERBOSE, "ts27010: > ",
			 data, TS0710_FRAME_SIZE(len));

	if (!qcspi_mux_tty) {
		pr_warning("ts27010: ldisc closed.  discarding %d bytes\n",
			   TS0710_FRAME_SIZE(len));
		return TS0710_FRAME_SIZE(len);
	}

	res = qcspi_ldisc_send(qcspi_mux_tty, data,
				 TS0710_FRAME_SIZE(len));

	if (res < 0) {
		pr_err("ts27010: pkt write error %d\n", res);
		return res;
	} else if (res != TS0710_FRAME_SIZE(len)) {
		pr_err("ts27010: short write %d < %d\n", res,
		       TS0710_FRAME_SIZE(len));
		return -EIO;
	}

	return res;
}

/* TODO: look at this */
static void ts0710_reset_dlci(u8 j)
{
	if (j >= QCSPIMUX_MAX_CHN)
		return;

	ts0710_connection.dlci[j].state = DISCONNECTED;
	ts0710_connection.dlci[j].flow_control = 0;
	ts0710_connection.dlci[j].mtu = DEF_TS0710_MTU;
	init_waitqueue_head(&ts0710_connection.dlci[j].open_wait);
	init_waitqueue_head(&ts0710_connection.dlci[j].close_wait);
}

/* TODO: look at this */
static void ts0710_reset_con(void)
{
	int j;

	ts0710_connection.mtu = DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE;
	for (j = 0; j < QCSPIMUX_MAX_CHN; j++)
		ts0710_reset_dlci(j);
}

static void ts0710_init(void)
{
	ts0710_crc_create_table(crctable);
	ts0710_reset_con();
}

static void ts0710_upon_disconnect(void)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	int j;

	for (j = 0; j < QCSPIMUX_MAX_CHN; j++) {
		ts0710->dlci[j].state = DISCONNECTED;
		wake_up_interruptible(&ts0710->dlci[j].open_wait);
		wake_up_interruptible(&ts0710->dlci[j].close_wait);
	}
	ts0710_reset_con();
}

static int ts27010_send_cmd(struct ts0710_con *ts0710, u8 dlci, u8 cmd)
{
	u8 frame[TS0710_FRAME_SIZE(0)];
	ts0710_pkt_set_header(frame, 0, 1, MCC_RSP, dlci, SET_PF(cmd));
	return ts0710_pkt_send(ts0710, frame);
}

static int ts27010_send_ua(struct ts0710_con *ts0710, u8 dlci)
{
	ts_debug(DBG_CMD, "ts27010: sending UA packet to DLCI %d\n", dlci);
	return ts27010_send_cmd(ts0710, dlci, UA);
}

static int ts27010_send_dm(struct ts0710_con *ts0710, u8 dlci)
{
	ts_debug(DBG_CMD, "ts27010: sending DM packet to DLCI %d\n", dlci);
	return ts27010_send_cmd(ts0710, dlci, DM);
}

static int ts27010_send_sabm(struct ts0710_con *ts0710, u8 dlci)
{
	ts_debug(DBG_CMD, "ts27010: sending SABM packet to DLCI %d\n", dlci);
	return ts27010_send_cmd(ts0710, dlci, SABM);
}

static int ts27010_send_disc(struct ts0710_con *ts0710, u8 dlci)
{
	ts_debug(DBG_CMD, "ts27010: sending DISC packet to DLCI %d\n", dlci);
	return ts27010_send_cmd(ts0710, dlci, DISC);
}

#ifdef SUPPORT_MUX_CMD_DATA_TAG
static int ts27010_send_uih(struct ts0710_con *ts0710, u8 dlci,
			    u8 *frame, u8 tag, const u8 *data, int len)
{
	ts_debug(DBG_CMD,
		 "ts27010: sending %d length UIH packet to DLCI %d\n",
		 len, dlci);
	ts0710_pkt_set_header(frame, len+1, 1, MCC_CMD, dlci, CLR_PF(UIH));
	*(u8 *)ts0710_pkt_data(frame) = tag;
	memcpy(ts0710_pkt_data(frame)+1, data, len);
	return ts0710_pkt_send(ts0710, frame);
}
#else
static int ts27010_send_uih(struct ts0710_con *ts0710, u8 dlci,
			    u8 *frame, const u8 *data, int len)
{
	ts_debug(DBG_CMD,
		 "ts27010: sending %d length UIH packet to DLCI %d\n",
		 len, dlci);
	ts0710_pkt_set_header(frame, len, 1, MCC_CMD, dlci, CLR_PF(UIH));
	memcpy(ts0710_pkt_data(frame), data, len);
	return ts0710_pkt_send(ts0710, frame);
}
#endif

static void ts27010_mcc_set_header(u8 *frame, int len, int cr, int cmd)
{
	struct mcc_short_frame *mcc_pkt;
	ts0710_pkt_set_header(frame, sizeof(struct mcc_short_frame) + len,
			      1, MCC_CMD, CTRL_CHAN, CLR_PF(UIH));

	mcc_pkt = ts0710_pkt_data(frame);
	mcc_pkt->h.type.ea = EA;
	mcc_pkt->h.type.cr = cr;
	mcc_pkt->h.type.type = cmd;
	mcc_pkt->h.length.ea = EA;
	mcc_pkt->h.length.len = len;
}

static void *ts27010_mcc_data(u8 *frame)
{
	return ((struct mcc_short_frame *)ts0710_pkt_data(frame))->value;
}

static int ts27010_send_pn(struct ts0710_con *ts0710, u8 prior, int frame_size,
			   u8 credit_flow, u8 credits, u8 dlci, u8 cr)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(sizeof(struct pn_msg_data))];
	struct pn_msg_data *pn;

	ts_debug(DBG_CMD, "ts27010: sending PN MCC\n");
	ts27010_mcc_set_header(frame, sizeof(struct pn_msg_data), cr, PN);

	pn = ts27010_mcc_data(frame);
	pn->res1 = 0;
	pn->res2 = 0;
	pn->dlci = dlci;
	pn->frame_type = 0;
	pn->credit_flow = credit_flow;
	pn->prior = prior;
	pn->ack_timer = 0;
	pn->frame_sizel = frame_size & 0xff;
	pn->frame_sizeh = frame_size >> 8;
	pn->credits = credits;
	pn->max_nbrof_retrans = 0;

	return ts0710_pkt_send(ts0710, frame);
}

static int ts27010_send_nsc(struct ts0710_con *ts0710, u8 type, int cr)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(sizeof(struct mcc_type))];
	struct mcc_type *t;

	ts_debug(DBG_CMD, "ts27010: sending NSC MCC\n");
	ts27010_mcc_set_header(frame, sizeof(struct mcc_type), cr, NSC);

	t = ts27010_mcc_data(frame);
	t->ea = 1;
	t->cr = mcc_is_cmd(type);
	t->type = type >> 2;

	return ts0710_pkt_send(ts0710, frame);
}

static int ts27010_send_msc(struct ts0710_con *ts0710,
			    u8 value, int cr, u8 dlci)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(sizeof(struct msc_msg_data))];
	struct msc_msg_data *msc;

	ts_debug(DBG_CMD, "ts27010: sending MSC MCC\n");
	ts27010_mcc_set_header(frame, sizeof(struct msc_msg_data), cr, MSC);

	msc = ts27010_mcc_data(frame);

	msc->dlci.ea = 1;
	msc->dlci.cr = 1;
	msc->dlci.d = dlci & 1;
	msc->dlci.server_chn = (dlci >> 1) & 0x1f;

	msc->v24_sigs = value;

	return ts0710_pkt_send(ts0710, frame);
}

static void ts27010_handle_msc(struct ts0710_con *ts0710, u8 type,
			       struct ts27010_ringbuf *rbuf,
			       int data_idx, int len)
{
	u8 dlci;
	u8 v24_sigs;

	dlci = ts27010_ringbuf_peek(rbuf, data_idx) >> 2;
	v24_sigs = ts27010_ringbuf_peek(rbuf, data_idx + 1);

	if ((ts0710->dlci[dlci].state != CONNECTED)
	    && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		ts27010_send_dm(ts0710, dlci);
		return;
	}

	if (mcc_is_cmd(type)) {
		ts_debug(DBG_VERBOSE,
			 "ts27010: received modem status command\n");
		if (v24_sigs & FC) {
			if (ts0710->dlci[dlci].state == CONNECTED) {
				ts_debug(DBG_CMD,
					 "ts27010: flow off on dlci%d\n", dlci);
				ts0710->dlci[dlci].state = FLOW_STOPPED;
			}
		} else {
			if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
				ts0710->dlci[dlci].state = CONNECTED;
				ts_debug(DBG_CMD,
					 "ts27010: flow on on dlci%d\n", dlci);
				/* TODO: flow control not supported */
			}
		}
		ts27010_send_msc(ts0710, v24_sigs, MCC_RSP, dlci);
	} else {
		ts_debug(DBG_VERBOSE,
			 "ts27010: received modem status response\n");

		if (v24_sigs & FC)
			ts_debug(DBG_CMD, "ts27010: flow stop accepted\n");
	}
}

static void ts27010_handle_pn(struct ts0710_con *ts0710, u8 type,
			      struct ts27010_ringbuf *rbuf,
			      int data_idx, int len)
{
	u8 dlci;
	u16 frame_size;
	struct pn_msg_data pn;
	int i;

	if (len != 8) {
		pr_err("ts27010: reveived pn on length:%d != 8\n", len);
		return;
	}

	for (i = 0; i < 8; i++)
		((u8 *)&pn)[i] = ts27010_ringbuf_peek(rbuf, data_idx + i);

	dlci = pn.dlci;
	frame_size = pn.frame_sizel | (pn.frame_sizeh << 8);

	if (mcc_is_cmd(type)) {
		ts_debug(DBG_CMD,
			 "ts27010: received PN command with frame size %d\n",
			frame_size);

		/* TODO: this looks like it will only ever shrink mtu */
		frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
		ts27010_send_pn(ts0710, pn.prior, frame_size,
				0, 0, dlci, MCC_RSP);
		ts0710->dlci[dlci].mtu = frame_size;

		ts_debug(DBG_VERBOSE,
			 "ts27010: mtu set to %d on dlci%d\n",
			 frame_size, dlci);
	} else {
		ts_debug(DBG_CMD,
			 "ts27010: received PN response with frame size %d\n",
			frame_size);

		frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
		ts0710->dlci[dlci].mtu = frame_size;

		ts_debug(DBG_VERBOSE, "ts27010: mtu set to %d on dlci%d\n",
			 frame_size, dlci);

		if (ts0710->dlci[dlci].state == NEGOTIATING) {
			ts0710->dlci[dlci].state = CONNECTING;
			wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		}
	}
}

static void ts27010_handle_mcc(struct ts0710_con *ts0710, u8 control,
			       struct ts27010_ringbuf *rbuf,
			       int data_idx, int len)
{
	u8 type;
	u8 mcc_len;

	type = ts27010_ringbuf_peek(rbuf, data_idx++);
	len--;
	mcc_len = ts27010_ringbuf_peek(rbuf, data_idx++);
	len--;

	if (mcc_len != len) {
		pr_warning("ts27010: handle_mcc: mcc_len:%d != len:%d\n",
			   mcc_len, len);
	}

	switch (type >> 2) {
	case TEST:
		pr_warning("ts27010: test command unimplemented\n");
		break;

	case FCON:
		ts_debug(DBG_CMD,
			 "ts27010: received all channels flow control on\n");
		pr_warning("ts27010: flow control unimplemented\n");
		break;

	case FCOFF:
		ts_debug(DBG_CMD,
			 "ts27010: received all channels flow control off\n");
		pr_warning("ts27010: flow control unimplemented\n");
		break;

	case MSC:
		ts27010_handle_msc(ts0710, type, rbuf, data_idx, len);
		break;

	case PN:
		ts27010_handle_pn(ts0710, type, rbuf, data_idx, len);
		break;

	case NSC:
		pr_warning("ts27010: received non supported cmd response\n");
		break;

	default:
		pr_warning("ts27010: received a non supported command\n");
		ts27010_send_nsc(ts0710, type, MCC_RSP);
		break;
	}
}

static void ts27010_handle_sabm(struct ts0710_con *ts0710, u8 control, int dlci)
{
	ts_debug(DBG_CMD, "ts27010: SABM received on dlci %d\n", dlci);

	if (ts0710_valid_dlci(dlci)) {
		ts27010_send_ua(ts0710, dlci);

		ts0710->dlci[dlci].state = CONNECTED;
		wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
	} else {
		pr_warning("ts27010: invalid dlci %d. sending DM\n", dlci);
		ts27010_send_dm(ts0710, dlci);
	}
}

static void ts27010_handle_ua(struct ts0710_con *ts0710, u8 control, int dlci)
{
	ts_debug(DBG_CMD, "ts27010: UA packet received on dlci %d\n", dlci);

	if (ts0710_valid_dlci(dlci)) {
		if (ts0710->dlci[dlci].state == CONNECTING) {
			ts0710->dlci[dlci].state = CONNECTED;
			wake_up_interruptible(&ts0710->dlci[dlci].
					      open_wait);
		} else if (ts0710->dlci[dlci].state == DISCONNECTING) {
			if (dlci == 0) {
				ts0710_upon_disconnect();
			} else {
				ts0710->dlci[dlci].state = DISCONNECTED;
				wake_up_interruptible(&ts0710->dlci[dlci].
						      open_wait);
				wake_up_interruptible(&ts0710->dlci[dlci].
						      close_wait);
				ts0710_reset_dlci(dlci);
			}
		} else {
			pr_warning("ts27010: invalid UA packet\n");
		}
	} else {
		pr_warning("ts27010: invalid dlci %d\n", dlci);
	}
}

static void ts27010_handle_dm(struct ts0710_con *ts0710, u8 control, int dlci)
{
	int oldstate;

	ts_debug(DBG_CMD, "ts27010: DM packet received on dlci %d\n", dlci);

	if (dlci == 0) {
		oldstate = ts0710->dlci[0].state;
		ts0710_upon_disconnect();
		if (oldstate == CONNECTING)
			ts0710->dlci[0].state = REJECTED;
	} else if (ts0710_valid_dlci(dlci)) {
		if (ts0710->dlci[dlci].state == CONNECTING)
			ts0710->dlci[dlci].state = REJECTED;
		else
			ts0710->dlci[dlci].state = DISCONNECTED;

		wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
		ts0710_reset_dlci(dlci);
	} else {
		pr_warning("ts27010: invalid dlci %d\n", dlci);
	}
}

static void ts27010_handle_disc(struct ts0710_con *ts0710, u8 control, int dlci)
{
	ts_debug(DBG_CMD, "ts27010: DISC packet received on dlci %d\n", dlci);

	if (!dlci) {
		ts27010_send_ua(ts0710, dlci);
		ts_debug(DBG_CMD, "ts27010: sending back UA\n");

		ts0710_upon_disconnect();
	} else if (ts0710_valid_dlci(dlci)) {
		ts27010_send_ua(ts0710, dlci);
		ts_debug(DBG_CMD, "ts27010: sending back UA\n");

		ts0710->dlci[dlci].state = DISCONNECTED;
		wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
		ts0710_reset_dlci(dlci);
	} else {
		pr_warning("ts27010: invalid dlci %d\n", dlci);
	}
}

int ts27010_handle_channel_data(struct ts0710_con *ts0710, int line,
		struct ts27010_ringbuf *rbuf, int data_idx, int len)
{
	int i = 0;
	unsigned char *buf;
	void *obj;
	struct chan_struct *c = &ts0710->chan[line];

	if (!c->func) {
		pr_err("qcspi_mux: mux %d not opened, discard data!\n", line);
		return len;
	}

	if (!c->alloc_obj) {
		obj = kmalloc(len, GFP_KERNEL);
		buf = obj;
	} else {
		obj = c->alloc_obj(&buf, len, c->data);
	}
	if (!obj) {
		pr_err("qcspi_mux: No mem! Mux %d Data discarded!\n", line);
		return len;
	}

	while (len--) {
		char c = ts27010_ringbuf_peek(rbuf, data_idx++);
		buf[i++] = c;
	}
	/* call mux channel callback to distribut data,
	client cb will free buf */
	return c->func(line, obj, i, c->data);
}

static void ts27010_handle_uih(struct ts0710_con *ts0710, u8 control, int dlci,
			       struct ts27010_ringbuf *rbuf,
			       int data_idx, int len)
{
#ifdef SUPPORT_MUX_CMD_DATA_TAG
	int tag;
#endif
	int tty_idx;

	if ((dlci >= QCSPIMUX_MAX_CHN)) {
		pr_warning("invalid dlci %d\n", dlci);
		ts27010_send_dm(ts0710, dlci);
		return;
	}

	if (GET_PF(control)) {
		pr_warning("ts27010: uih packet with P/F set, discarding.\n");
		return;
	}

	if ((ts0710->dlci[dlci].state != CONNECTED)
	    && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		pr_warning("ts27010: uih: dlci %d not connected, discarding.\n",
			dlci);
		ts27010_send_dm(ts0710, dlci);
		return;
	}

	if (dlci == 0) {
		pr_info("ts27010: mcc on channel 0\n");
		ts27010_handle_mcc(ts0710, control, rbuf, data_idx, len);
		return;
	}

	ts_debug(DBG_CMD, "ts27010: uih on channel %d\n", dlci);

	if (len > ts0710->dlci[dlci].mtu) {
		pr_warning("ts27010: dlci%d: uih_len:%d "
			   "is bigger than mtu:%d, discarding.\n",
			    dlci, len, ts0710->dlci[dlci].mtu);
		return;
	}

#ifdef SUPPORT_MUX_CMD_DATA_TAG
	tag = ts27010_ringbuf_peek(rbuf, data_idx);
	len--;
	data_idx++;

	if (len == 0)
		return;

	switch (tag) {
	case CMDTAG:
		tty_idx = dlci;
		ts_debug(DBG_VERBOSE,
			 "ts27010: CMDTAG on DLCI %d, /dev/mux%d\n",
			 dlci, tty_idx);
		ts27010_debugrbuf(DBG_DATA, "ts27010: <C ",
				  rbuf, data_idx, len);
		break;

	case DATATAG:
	default:
		tty_idx = dlci;
		ts_debug(DBG_VERBOSE,
			 "ts27010: NON-CMDTAG on DLCI %d, /dev/mux%d\n",
			 dlci, tty_idx);
		ts27010_debugrbufhex(DBG_DATA, "ts27010: <D ",
				     rbuf, data_idx, len);
		break;
	}
#else
	if (len == 0)
		return;

		tty_idx = dlci;
		ts_debug(DBG_VERBOSE,
			 "ts27010: data on DLCI %d, /dev/mux%d\n",
			 dlci, tty_idx);
		ts27010_debugrbufhex(DBG_DATA, "ts27010: < ",
				  rbuf, data_idx, len);
#endif
	/* distribut data to mux clients */
	ts27010_handle_channel_data(ts0710, dlci, rbuf, data_idx, len);
}


static void ts27010_handle_frame(struct ts27010_ringbuf *rbuf, u8 addr,
				 u8 control, int data_idx, int len)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	int dlci;

	dlci = ts0710_dlci(addr);
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
		ts_debug(DBG_VERBOSE, "ts27010: illegal packet\n");
		break;
	}
}

int qcspi_mux_fake_close(int line)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	struct dlci_struct *d = &ts0710->dlci[line];
	struct chan_struct *c = &ts0710->chan[line];

	mutex_lock(&d->lock);
	d->state = DISCONNECTED;
	c->func = 0;
	mutex_unlock(&d->lock);

	return 0;
}
int qcspi_mux_close(int line)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	struct dlci_struct *d = &ts0710->dlci[line];
	struct chan_struct *c = &ts0710->chan[line];
	int try;
	int retval;

	ts_debug(DBG_CMD, "%s\n", __func__);
	ts_debug(DBG_CMD, "ts27010: closing dlci %d\n", line);

	mutex_lock(&d->lock);
	if (d->clients > 1) {
		d->clients--;
		mutex_unlock(&d->lock);
		return 0;
	}
	if (d->state == DISCONNECTED ||
	    d->state == REJECTED ||
	    d->state == DISCONNECTING) {
		mutex_unlock(&d->lock);
		return 0;
	}

	d->state = DISCONNECTING;
	try = 1;
	while (try--) {
		ts27010_send_disc(ts0710, line);
		mutex_unlock(&d->lock);
		retval = wait_event_interruptible_timeout(d->close_wait,
							  d->state !=
							  DISCONNECTING,
						TS0710MUX_CLOSE_TIME_OUT);
		mutex_lock(&d->lock);

		if (retval == 0)
			continue;

		if (retval == -ERESTARTSYS) {
			retval = -EAGAIN;
			break;
		}

		if (d->state != DISCONNECTED) {
			retval = -EIO;
			break;
		}

		c->func = 0;
		retval = 0;
		break;
	}

	if (try < 0)
		retval = -EIO;

	if (d->state != DISCONNECTED) {
		if (line == 0) {
			ts0710_upon_disconnect();
		} else {
			d->state = DISCONNECTED;
			wake_up_interruptible(&d->close_wait);
			ts0710_reset_dlci(line);
		}
	}

	mutex_unlock(&d->lock);

	return retval;
}

/* call with dlci locked held */
int qcspi_mux_wait_for_open(struct ts0710_con *ts0710, int dlci)
{
	int try = 8;
	int ret;
	struct dlci_struct *d = &ts0710->dlci[dlci];

	while (try--) {
		mutex_unlock(&d->lock);
		ret = wait_event_interruptible_timeout(d->open_wait,
						       d->state != CONNECTING,
						       TS0710MUX_TIME_OUT);
		/*
		 * It is possible that d->state could have changed back to
		 * to connecting between being woken up and aquiring the lock.
		 * The side effect is that this open() will return turn -ENODEV.
		 */

		mutex_lock(&d->lock);
		if (ret == 0)
			continue;

		if (ret == -ERESTARTSYS)
			return -EAGAIN;

		if (d->state == REJECTED)
			return -EREJECTED;

		if (d->state != CONNECTED)
			return -ENODEV;

		return 0;
	}

	return -ENODEV;
}

int qcspi_mux_fake_open(int line, qcspi_mux_cb func,
	qcspi_mux_alloc_obj alloc_obj, void *data)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	struct dlci_struct *d = &ts0710->dlci[line];
	struct chan_struct *c = &ts0710->chan[line];

	mutex_lock(&d->lock);
	d->state = CONNECTED;
	c->func = func;
	c->alloc_obj = alloc_obj;
	c->data = data;
	mutex_unlock(&d->lock);

	return 0;
}

int qcspi_mux_open(int line, qcspi_mux_cb func, void *data)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	struct dlci_struct *d = &ts0710->dlci[line];
	struct chan_struct *c = &ts0710->chan[line];
	int try;
	int retval;

	ts_debug(DBG_CMD, "%s\n", __func__);
	mutex_lock(&d->lock);
	if (d->clients > 0) {
		if (d->state == CONNECTED) {
			d->clients++;
			mutex_unlock(&d->lock);
			return 0;
		}

		if (d->state != CONNECTING) {
			mutex_unlock(&d->lock);
			return -EREJECTED;
		}
		retval = qcspi_mux_wait_for_open(ts0710, line);
		if (d->state == CONNECTED)
			d->clients++;
		mutex_unlock(&d->lock);
		return retval;
	}
	if (d->state != DISCONNECTED && d->state != REJECTED) {
		pr_err("ts27010: DLCI%d: state invalid on open\n", line);
		mutex_unlock(&d->lock);
		return -ENODEV;
	}

	/* we are the first to try to open the dlci */
	d->state = CONNECTING;
	try = line == 0 ? 10 : 3;
	while (try--) {
		ts27010_send_sabm(ts0710, line);

		mutex_unlock(&d->lock);
		retval = wait_event_interruptible_timeout(d->open_wait,
							  d->state !=
							  CONNECTING,
							  TS0710MUX_TIME_OUT);
		mutex_lock(&d->lock);

		if (retval == 0)
			continue;

		if (retval == -ERESTARTSYS) {
			retval = -EAGAIN;
			break;
		}

		if (d->state == REJECTED) {
			retval = -EREJECTED;
			break;
		}

		if (d->state != CONNECTED) {
			retval = -ENODEV;
			break;
		}

		c->func = func;
		c->data = data;
		retval = 0;
		break;
	}

	if (try < 0)
		retval = -ENODEV;

	if (d->state == CONNECTING)
		d->state = DISCONNECTED;

	/* other ttys might be waiting on this dlci */
	wake_up_interruptible(&d->open_wait);

	mutex_unlock(&d->lock);
	return retval;
}

int qcspi_mux_active(void)
{
	return qcspi_mux_tty != NULL;
}

/* todo: need any lock? */
int qcspi_mux_write(int line, const unsigned char *buf, int count)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	int dlci;
	int err;
	int c;
	int ret;
#ifdef SUPPORT_MUX_CMD_DATA_TAG
	u8 tag;
#endif

	ts_debug(DBG_CMD, "%s: len=%d\n", __func__, count);
	dlci = line;
	if (ts0710->dlci[dlci].state == CONNECTED) {

		c = min(count, (ts0710->dlci[dlci].mtu - 1));
		if (c <= 0) {
			err = 0;
			goto err;
		}

		ts_debug(DBG_VERBOSE, "ts27010: preparing to send %d bytes "
			 "from /dev/mux%d\n", c, line);

#ifdef SUPPORT_MUX_CMD_DATA_TAG
		ts27010_debugstr(DBG_DATA, "ts27010: >C ", buf, c);
		ts_debug(DBG_VERBOSE, "CMDTAG\n");
		tag = CMDTAG;
		ret = ts27010_send_uih(ts0710, dlci, ts0710->chan[line].buf,
				 tag, buf, c);
#else
		ts27010_debugstr(DBG_DATA, "ts27010: > ", buf, c);
		ret = ts27010_send_uih(ts0710, dlci, ts0710->chan[line].buf,
				 buf, c);
#endif
		if (ret <= 0)
			return ret;
		else
			return c;
	} else {
		pr_err("ts27010: write on DLCI %d while not connected\n",
			   dlci);
		return -EDISCONNECTED;
	}

err:
	return err;
}

void qcspi_mux_recv(struct ts27010_ringbuf *rbuf)
{
	int count;
	int i;
	u8 c;
	int state = RECV_STATE_IDLE;
	int consume_idx = -1;
	int data_idx = 0;
	u8 addr = 0;
	u8 control = 0;
	int len = 0;
	u8 fcs = 0;

	ts_debug(DBG_CMD, "%s\n", __func__);
	count = ts27010_ringbuf_level(rbuf);

	for (i = 0; i < count; i++) {
		c = ts27010_ringbuf_peek(rbuf, i);

		switch (state) {
		case RECV_STATE_IDLE:
			if (c == TS0710_BASIC_FLAG) {
				fcs = ts0710_crc_start();
				state = RECV_STATE_ADDR;
			} else {
				consume_idx = i;
			}
			break;

		case RECV_STATE_ADDR:
			if (c != TS0710_BASIC_FLAG) {
				fcs = ts0710_crc_calc(fcs, c);
				addr = c;
				state = RECV_STATE_CONTROL;
			} else {
				pr_warning(
					"ts27010: RX wrong data. Drop msg.\n");
				consume_idx = i;
			}
			break;

		case RECV_STATE_CONTROL:
			fcs = ts0710_crc_calc(fcs, c);
			control = c;
			state = RECV_STATE_LEN;
			break;

		case RECV_STATE_LEN:
			fcs = ts0710_crc_calc(fcs, c);
			len = c>>1;
			if (c & 0x1) {
				data_idx = i+1;
				state = RECV_STATE_DATA;
			} else {
				state = RECV_STATE_LEN2;
			}
			break;

		case RECV_STATE_LEN2:
			fcs = ts0710_crc_calc(fcs, c);
			len |= c<<7;
			data_idx = i+1;
			if (len + data_idx >= LDISC_BUFFER_SIZE) {
				pr_warning(
					"ts27010: wrong length, Drop msg.\n");
				state = RECV_STATE_IDLE;
				consume_idx = i;
				break;
			}
			state = RECV_STATE_DATA;
			break;

		case RECV_STATE_DATA:
			if (i < data_idx+len)
				/* i point to fcs bytes in next loop*/
				i = data_idx+len - 1;

			if (i == data_idx+len) {
				/* FCS byte */
				fcs = ts0710_crc_calc(fcs, c);
				state = RECV_STATE_END;
			}
			break;

		case RECV_STATE_END:
			if (c == TS0710_BASIC_FLAG && ts0710_crc_check(fcs)) {
				ts27010_handle_frame(rbuf, addr, control,
						      data_idx, len);
			} else {
				pr_warning("ts27010: lost synchronization\n");
			}
			consume_idx = i;
			state = RECV_STATE_IDLE;
			break;
		}
	}

	ts27010_ringbuf_consume(rbuf, consume_idx+1);

}

int qcspi_mux_init(void)
{
	int err;
	int i;

	ts_debug(DBG_CMD, "%s\n", __func__);

#ifdef DEBUG
	debug = 0;
#endif

	ts0710_init();

	for (i = 0; i < QCSPIMUX_MAX_CHN; i++) {
		mutex_init(&ts0710_connection.dlci[i].lock);
		ts0710_connection.chan[i].func = 0x0;
		ts0710_connection.chan[i].buf =
			kmalloc(TS0710MUX_SEND_BUF_SIZE, GFP_KERNEL);
		if (ts0710_connection.chan[i].buf == NULL) {
			err = -ENOMEM;
			goto err0;
		}
	}

	err = qcspi_ldisc_init();
	if (err != 0) {
		pr_err("ts27010mux: error %d registering line disc.\n", err);
		goto err0;
	}

	pr_info("ts27010 mux registered\n");

	return 0;

err0:
	for (i = 0; i < QCSPIMUX_MAX_CHN; i++)
		kfree(ts0710_connection.chan[i].buf);

	return err;
}

void qcspi_mux_exit(void)
{
	int j;

	for (j = 0; j < QCSPIMUX_MAX_CHN; j++)
		kfree(&ts0710_connection.chan[j].buf);

	qcspi_ldisc_remove();
}

