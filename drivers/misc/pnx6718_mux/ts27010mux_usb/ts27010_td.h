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

#ifndef __TS27010_TD_H__
#define __TS27010_TD_H__

#ifdef TS27010_UART_RETRAN
#define ACK 0x4F
#define ACK_OK 0
#define ACK_ERROR 1
#define INDIFFERENT_SN 0x7F
#define SHORT_CRC_CHECK 4
#define LONG_CRC_CHECK 5
#endif

#ifndef UNUSED_PARAM
#define UNUSED_PARAM(v) (void)(v)
#endif

struct ts0710_con;
struct ts27010_ringbuf;

#ifdef TS27010_UART_RETRAN
int ts27010_usb_check_sequence_number(struct ts0710_con *ts0710, u8 sn);
/*
typedef void (*handle_uih) (struct ts0710_con *ts0710, u8 control, int dlci,
				struct ts27010_ringbuf *rbuf,
				int data_idx, int len);

void ts27010_handle_retran_uih(
	struct ts0710_con *ts0710, u8 sn, u8 control, int dlci,
	struct ts27010_ringbuf *rbuf,
	int data_idx, int len, handle_uih handle);
*/
int ts27010_usb_process_ack(struct ts0710_con *ts0710, u8 sn);
#endif

int ts27010_usb_control_send(struct ts0710_con *ts0710, u8 *buf, u32 len);
int ts27010_usb_uih_send(struct ts0710_con *ts0710, u8 *data, u32 len);
int ts27010_usb_td_open(void);
void ts27010_usb_td_close(void);
int ts27010_usb_td_init(void);
void ts27010_usb_td_remove(void);

#endif /* __TS27010_TD_H__ */
