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

/*
* This header file should be included by both MUX and other applications
* which access MUX device files. It gives the additional macro definitions
* shared between MUX and applications.
*/
#ifndef __TS27010_MUX_H__
#define __TS27010_MUX_H__

/*/ 8k: receive buffer size /*/
#define LDISC_BUFFER_SIZE (8 * 1024 - sizeof(struct ts27010_ringbuf))

/* TODO: should use the IOCTLNUM macros */
/* Special ioctl() upon a MUX device file for hanging up a call */
#define TS0710MUX_IO_MSC_HANGUP 0x54F0

/* Special ioctl() upon a MUX device file for MUX loopback test */
#define TS0710MUX_IO_TEST_CMD 0x54F1

/* TODO: get rid of these */
/* Special Error code might be return from write() to a MUX device file */
#define EDISCONNECTED 900	/* link is disconnected */

/* Special Error code might be return from open() to a MUX device file  */
#define EREJECTED 901		/* link connection request is rejected */

/* TODO: goes away with clean tty interface */
extern struct tty_struct *ts27010mux_uart_tty;

extern struct workqueue_struct *g_mux_uart_queue;
extern int g_mux_uart_dump_frame;
extern int g_mux_uart_dump_user_data;

#ifdef MUX_UART_LOGGER
extern struct ts27010_mux_logger *g_mux_uart_logger;
#endif

#ifdef PROC_DEBUG_MUX
#ifdef PROC_DEBUG_MUX_STAT
void ts27010_tty_uart_dump_io(void);
void ts27010_ldisc_uart_drv_stat(void);
void ts27010_mux_uart_mux_stat(void);
void ts27010_tty_uart_dump_io_clear(void);
void ts27010_ldisc_uart_drv_stat_clear(void);
void ts27010_mux_uart_mux_stat_clear(void);

extern int g_nStatUARTDrvIO;
#endif
#endif

#ifdef TS27010_UART_RETRAN
extern struct sequence_number_t *g_ap_send_sn;
#endif

#ifdef UART_LOOP
void ts27010_ldisc_uart_receive(struct tty_struct *tty,
	const unsigned char *data, char *cflags, int count);
#endif

u8 ts0710_uart_crc_data(u8 *data, int length);

struct ts27010_ringbuf;

/* in ts27010_mux.c */
int ts27010_mux_uart_active(void);
int ts27010_mux_uart_line_open(int line);
int ts27010_mux_uart_line_write(
	int line, const unsigned char *buf, int count);
int ts27010_mux_uart_line_chars_in_buffer(int line);
int ts27010_mux_uart_line_write_room(int line);

void ts27010_mux_uart_line_throttle(int line);
void ts27010_mux_uart_line_unthrottle(int line);
int ts27010_mux_uart_line_ioctl(
	unsigned int cmd, unsigned long arg, int line);
void ts27010_mux_uart_line_flush_buffer(int line);
int ts27010_mux_uart_mux_open(void);
void ts27010_mux_uart_mux_close(void);

void ts27010_mux_uart_recv(
	/* spinlock_t* recv_lock, */struct ts27010_ringbuf *rbuf);
void ts27010_mux_uart_line_close(int line);

/* in ts27010_ldisc.c */
int ts27010_ldisc_uart_init(void);
int ts27010_ldisc_uart_send(struct tty_struct *tty, u8 *data, int len);
void ts27010_ldisc_uart_remove(void);

/* in ts27010_tty.c */
int ts27010_tty_uart_init(void);
int ts27010_tty_uart_send(int line, u8 *data, int len);
int ts27010_tty_uart_send_rbuf(int line, struct ts27010_ringbuf *rbuf,
			  int data_idx, int len);
void ts27010_tty_uart_remove(void);
int ts27010_tty_uart_open(void);
void ts27010_tty_uart_close(void);

#endif /* __TS27010_MUX_H__ */
