/*
 * File: ts27010.h
 *
 * Portions derived from rfcomm.c, original header as follows:
 *
 * Copyright (C) 2000, 2001  Axis Communications AB
 * Copyright (C) 2002, 2004, 2009 Motorola
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
#ifndef __TS27010_H__
#define __TS27010_H__

/* test workground, no need BP involved */
/* #define USB_LOOP */

/* mux unit test */
/* #define MUX_USB_UT */

/* uart retransmission */
/* #define TS27010_UART_RETRAN */

/* mux binary frames log system */
/* #define MUX_USB_LOGGER */

/* use thread to send/recv uih */
/* #define USE_THREAD */

/* dump send/recv frames */
#define DUMP_FRAME

#ifndef PROC_DEBUG_MUX
#define PROC_DEBUG_MUX
#ifndef PROC_DEBUG_MUX_STAT
#define PROC_DEBUG_MUX_STAT
#endif
#endif

#ifndef DEBUG
#define DEBUG
#endif

#define QUEUE_SELF

/* mux device number */
#define TS0710_MAX_CHN 12
#define TS0710_MAX_MUX 11

/*  wait event timeout, in jiffies  */
#define TS0710MUX_TIME_OUT 250

#define SET_PF(ctr) ((ctr) | (1 << 4))
#define CLR_PF(ctr) ((ctr) & 0xef)
#define GET_PF(ctr) (((ctr) >> 4) & 0x1)

#define SHORT_PAYLOAD_SIZE 127

#define EA 1
#define FCS_SIZE 1
#define FLAG_SIZE 2

#define ADDRESS_OFFSET 1
#define CONTROL_OFFSET 2

#ifdef TS27010_UART_RETRAN
#define SEQUENCE_OFFSET 3
#define LENGTH_OFFSET 4
#define DEF_TS0710_MTU 512
#else
#define LENGTH_OFFSET 3
#define DEF_TS0710_MTU 1800
/* #define DEF_TS0710_MTU 1024 */
#endif

#define CTRL_CHAN 0

#define TS0710_FRAME_SIZE(len)						\
	((len) > SHORT_PAYLOAD_SIZE ?					\
	 (len) + FLAG_SIZE + sizeof(struct long_frame) + FCS_SIZE :	\
	 (len) + FLAG_SIZE + sizeof(struct short_frame) + FCS_SIZE)

#define TS0710_MCC_FRAME_SIZE(len) \
	TS0710_FRAME_SIZE((len) + sizeof(struct mcc_short_frame))

/* Round to 4 bytes */
#define TS0710MUX_SEND_BUF_SIZE \
	((TS0710_FRAME_SIZE(DEF_TS0710_MTU) + 3) \
	& 0xFFFC)

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
	u8 dlci:6;
} __attribute__ ((packed));

static inline int ts0710_dlci(u8 addr)
{
	return (addr >> 2) & 0x3f;
}

struct short_length {
	u8 ea:1;
	u8 len:7;
} __attribute__ ((packed));

struct long_length {
	u8 ea:1;
	u8 l_len:7;
	u8 h_len;
} __attribute__ ((packed));

struct short_frame_head {
	struct address_field addr;
	u8 control;
#ifdef TS27010_UART_RETRAN
	u8 sn;
#endif
	struct short_length length;
} __attribute__ ((packed));

struct short_frame {
	struct short_frame_head h;
	u8 data[0];
} __attribute__ ((packed));

struct long_frame_head {
	struct address_field addr;
	u8 control;
#ifdef TS27010_UART_RETRAN
	u8 sn;
#endif
	struct long_length length;
	u8 data[0];
} __attribute__ ((packed));

struct long_frame {
	struct long_frame_head h;
	u8 data[0];
} __attribute__ ((packed));

/* Typedefinitions for structures used for the multiplexer commands */
struct mcc_type {
	u8 ea:1;
	u8 cr:1;
	u8 type:6;
} __attribute__ ((packed));

struct mcc_short_frame_head {
	struct mcc_type type;
	struct short_length length;
	u8 value[0];
} __attribute__ ((packed));

struct mcc_short_frame {
	struct mcc_short_frame_head h;
	u8 value[0];
} __attribute__ ((packed));

struct mcc_long_frame_head {
	struct mcc_type type;
	struct long_length length;
	u8 value[0];
} __attribute__ ((packed));

struct mcc_long_frame {
	struct mcc_long_frame_head h;
	u8 value[0];
} __attribute__ ((packed));

/* MSC-command */
struct v24_sigs {
	u8 ea:1;
	u8 fc:1;
	u8 rtc:1;
	u8 rtr:1;
	u8 reserved:2;
	u8 ic:1;
	u8 dv:1;
} __attribute__ ((packed));

struct brk_sigs {
	u8 ea:1;
	u8 b1:1;
	u8 b2:1;
	u8 b3:1;
	u8 len:4;
} __attribute__ ((packed));

struct msc_msg_data {
	struct address_field dlci;
	u8 v24_sigs;
} __attribute__ ((packed));

struct msc_msg {
	struct short_frame_head s_head;
	struct mcc_short_frame_head mcc_s_head;
	struct address_field dlci;
	u8 v24_sigs;
	u8 fcs;
} __attribute__ ((packed));

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
} __attribute__ ((packed));

/* NSC-command */
/*/
struct nsc_msg {
	struct short_frame_head s_head;
	struct mcc_short_frame_head mcc_s_head;
	struct mcc_type command_type;
	u8 fcs;
} __attribute__ ((packed));
/*/

#else
#error Only littel-endianess supported now!
#endif

enum {
	DISCONNECTED = 0,
	DISCONNECTING,
	NEGOTIATING,
	CONNECTING,
	CONNECTED,
	FLOW_STOPPED,
	HOLD_CONNECTED,
	HOLD_FLOW_STOPPED,
	REJECTED
};

/*/
enum ts0710_events {
	CONNECT_IND,
	CONNECT_CFM,
	DISCONN_CFM
};
/*/

struct dlci_struct {
	int clients;
	struct mutex lock; /* dlci lock */
	wait_queue_head_t open_wait;
	wait_queue_head_t close_wait;
	/*
	 * if BP is in FC, all app writing operations
	 * will be blocked on this queue
	 */
	wait_queue_head_t mux_write_wait;
	/*
	wait_queue_head_t mux_hold_wait;
	*/

	u8 state;
	u8 flow_control;
	u8 initiator;
	u8 dummy;
	u16 mtu;
	u16 dummy2;
};

struct chan_struct {
	struct mutex	write_lock; /* dlci write lock */
	u8		buf[TS0710MUX_SEND_BUF_SIZE];
};

struct ts0710_con {
/*/	u16 mtu; /*/
	struct chan_struct	chan[TS0710_MAX_MUX];
	struct dlci_struct	dlci[TS0710_MAX_CHN];

	wait_queue_head_t test_wait;
	u32 test_errs;
	u8 initiator;
	u8 be_testing;
	u16 dummy;
};

enum { MSG_MSGDUMP, MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR, MSG_NONE };

#ifdef DEBUG
extern int g_mux_usb_print_level;
extern void sched_show_task(struct task_struct *p);

#define kassert(cond) do { \
	if (!(cond)) { \
		printk(KERN_WARNING "MUX_UART_assert: %s failed at %s:%d\n", \
			#cond, __FILE__, __LINE__); \
		sched_show_task(current); \
	} \
} while (0)

static const char strLog[5] = {
	'M',
	'D',
	'I',
	'W',
	'E',
};
#define mux_print(level, fmt, args...) do { \
	if (level >= g_mux_usb_print_level) \
		printk(KERN_WARNING "MUX_USB<%c>[%s:%d]: " fmt, \
			strLog[level], __func__, __LINE__, ##args); \
} while (0)
#define FUNC_ENTER() do { \
	mux_print(MSG_MSGDUMP, "enter\n");\
} while (0)
#define FUNC_EXIT() do { \
	mux_print(MSG_MSGDUMP, "exit\n");\
} while (0)
void mux_usb_hexdump(int level, const char *title,
			const char *function, u32 line,
			const u8 *buf, u32 len);
#else /* DEBUG */
#define mux_print(level, fmt, args...) do { } while (0)
#define mux_usb_hexdump(lev, t, f, line, buf, len) do { } while (0)
#define kassert(cond) do { } while (0)
#define FUNC_ENTER() do { } while (0)
#define FUNC_EXIT() do { } while (0)
#endif /* DEBUG */

#endif /* __TS27010_H__ */
