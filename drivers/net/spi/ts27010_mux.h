#ifndef QCSPIMUX_H
#define QCSPIMUX_H

#define QCSPIMUX_MAX_CHN 21

#define SET_PF(ctr) ((ctr) | (1 << 4))
#define CLR_PF(ctr) ((ctr) & 0xef)
#define GET_PF(ctr) (((ctr) >> 4) & 0x1)

#define SHORT_PAYLOAD_SIZE 127

#define EA 1
#define FCS_SIZE 1
#define FLAG_SIZE 2

#define TS0710_MAX_HDR_SIZE 5
#define DEF_TS0710_MTU 1600

#define TS0710_BASIC_FLAG 0xF9

/* the control field */
#define SABM 0x2f
#define SABM_SIZE 4
#define UA 0x63
#define UA_SIZE 4
#define DM 0x0f
#define DISC 0x43
#define UIH 0xef

/* the type field in a multiplexer command packet */
#define TEST 0x8
#define FCON 0x28
#define FCOFF 0x18
#define MSC 0x38
#define RPN 0x24
#define RLS 0x14
#define PN 0x20
#define NSC 0x4

/* V.24 modem control signals */
#define FC 0x2
#define RTC 0x4
#define RTR 0x8
#define IC 0x40
#define DV 0x80

#define CTRL_CHAN 0		/* The control channel is defined as DLCI 0 */
#define MCC_CR 0x2
#define MCC_CMD 1		/* Multiplexer command cr */
#define MCC_RSP 0		/* Multiplexer response cr */

static inline int mcc_is_cmd(u8 type)
{
	return type & MCC_CR;
}

static inline int mcc_is_rsp(u8 type)
{
	return !(type & MCC_CR);
}

#ifdef __LITTLE_ENDIAN_BITFIELD
struct address_field {
	u8 ea:1;
	u8 cr:1;
	u8 d:1;
	u8 server_chn:5;
} __packed;

static inline int ts0710_dlci(u8 addr)
{
	return (addr >> 2) & 0x3f;
}

struct short_length {
	u8 ea:1;
	u8 len:7;
} __packed;

struct long_length {
	u8 ea:1;
	u8 l_len:7;
	u8 h_len;
} __packed;

struct short_frame_head {
	struct address_field addr;
	u8 control;
	struct short_length length;
} __packed;

struct short_frame {
	struct short_frame_head h;
	u8 data[0];
} __packed;

struct long_frame_head {
	struct address_field addr;
	u8 control;
	struct long_length length;
	u8 data[0];
} __packed;

struct long_frame {
	struct long_frame_head h;
	u8 data[0];
} __packed;

/* Typedefinitions for structures used for the multiplexer commands */
struct mcc_type {
	u8 ea:1;
	u8 cr:1;
	u8 type:6;
} __packed;

struct mcc_short_frame_head {
	struct mcc_type type;
	struct short_length length;
	u8 value[0];
} __packed;

struct mcc_short_frame {
	struct mcc_short_frame_head h;
	u8 value[0];
} __packed;

struct mcc_long_frame_head {
	struct mcc_type type;
	struct long_length length;
	u8 value[0];
} __packed;

struct mcc_long_frame {
	struct mcc_long_frame_head h;
	u8 value[0];
} __packed;

/* MSC-command */
struct v24_sigs {
	u8 ea:1;
	u8 fc:1;
	u8 rtc:1;
	u8 rtr:1;
	u8 reserved:2;
	u8 ic:1;
	u8 dv:1;
} __packed;

struct brk_sigs {
	u8 ea:1;
	u8 b1:1;
	u8 b2:1;
	u8 b3:1;
	u8 len:4;
} __packed;

struct msc_msg_data {
	struct address_field dlci;
	u8 v24_sigs;
} __packed;

struct pn_msg_data {
	u8 dlci:6;
	u8 res1:2;

	u8 frame_type:4;
	u8 credit_flow:4;

	u8 prior:6;
	u8 res2:2;

	u8 ack_timer;
	u8 frame_sizel;
	u8 frame_sizeh;
	u8 max_nbrof_retrans;
	u8 credits;
} __packed;

#else
#error Only littel-endianess supported now!
#endif

#define TS0710_FRAME_SIZE(len)						\
	((len) > SHORT_PAYLOAD_SIZE ?					\
	 (len) + FLAG_SIZE + sizeof(struct long_frame) + FCS_SIZE :	\
	 (len) + FLAG_SIZE + sizeof(struct short_frame) + FCS_SIZE)

#define TS0710_MCC_FRAME_SIZE(len) \
	TS0710_FRAME_SIZE((len) + sizeof(struct mcc_short_frame))

enum {
	REJECTED = 0,
	DISCONNECTED,
	CONNECTING,
	NEGOTIATING,
	CONNECTED,
	DISCONNECTING,
	FLOW_STOPPED
};

enum qcspimux_events {
	CONNECT_IND,
	CONNECT_CFM,
	DISCONN_CFM
};

struct dlci_struct {
	u8 state;
	u8 flow_control;
	u16 mtu;
	int clients;
	struct mutex lock; /* */
	wait_queue_head_t open_wait;
	wait_queue_head_t close_wait;
};

typedef int (*qcspi_mux_cb) (int line, void *obj,
	int count, void *data);
typedef void *(*qcspi_mux_alloc_obj) (unsigned char **buf,
	int count, void *data);

struct chan_struct {
	qcspi_mux_cb func;
	qcspi_mux_alloc_obj alloc_obj;
	void *data;
	u8		*buf;
};

struct ts0710_con {
	u16 mtu;
	struct dlci_struct	dlci[QCSPIMUX_MAX_CHN];
	struct chan_struct	chan[QCSPIMUX_MAX_CHN];
};

#define LDISC_BUFFER_SIZE (35*1024 - sizeof(struct ts27010_ringbuf))
#define EDISCONNECTED 900	/* link is disconnected */
#define EREJECTED 901		/* link connection request is rejected */

/* real TTY device we will sit on and transfer data over,
   set by ioctl(fd,TIOCSETD,&ldisc);
*/
extern struct tty_struct *qcspi_mux_tty;
struct ts27010_ringbuf;

int qcspi_mux_active(void);
int qcspi_mux_open(int line, qcspi_mux_cb func, void *data);
int qcspi_mux_fake_open(int line, qcspi_mux_cb func,
			qcspi_mux_alloc_obj alloc_obj, void *data);
int qcspi_mux_close(int line);
int qcspi_mux_fake_close(int line);
int qcspi_mux_write(int line, const unsigned char *buf, int count);
void qcspi_mux_recv(struct ts27010_ringbuf *rbuf);

typedef void (*notifier)(void *data);
struct notify_item {
	struct list_head node;
	notifier notify;
	void *data;
};

struct qcspi_ldisc_write_done_notifies {
	spinlock_t lock; /* protect list when access */
	struct list_head header;
};

struct notify_item *
qcspi_ldisc_add_write_done_notify(notifier func, void *data);

int qcspi_ldisc_init(void);
void qcspi_ldisc_remove(void);
int qcspi_ldisc_send(struct tty_struct *tty, u8 *data, int len);
int qcspi_mux_init(void);
void qcspi_mux_exit(void);
#endif

