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

#ifndef __TS27010_MISC_H__
#define __TS27010_MISC_H__

#ifdef PROC_DEBUG_MUX
struct proc_dir_entry *ts27010_usb_proc_alloc(void);
#ifdef PROC_DEBUG_MUX_STAT
struct proc_dir_entry *ts27010_usb_proc_stat_alloc(void);
#endif
inline void ts27010_usb_proc_free(
	struct proc_dir_entry *seq_no, const char *name);
#endif

#ifdef TS27010_UART_RETRAN

#define SLIDE_WINDOWS_SIZE_AP 6
#define MAX_TRANS_SN 0x40
#define RETRAN_TIMEOUT 100
#define MAX_RETRAN_TIMES 10

/***************************** sequence number ********************************/
struct sequence_number_t;
struct sequence_number_t *ts27010_sequence_number_alloc(void);
inline void ts27010_sequence_number_free(struct sequence_number_t *seq_no);
inline int ts27010_sequence_number_lock(struct sequence_number_t *seq_no);
inline void ts27010_sequence_number_unlock(struct sequence_number_t *seq_no);
inline u8 ts27010_sequence_number_get(struct sequence_number_t *seq_no);
inline void ts27010_sequence_number_inc(struct sequence_number_t *seq_no);

/***************************** timer param ********************************/
struct ts27010_timer_para_t;
struct ts27010_timer_para_t *ts27010_alloc_timer_para(
	void (*p)(struct work_struct *work));
inline void ts27010_timer_para_free(struct ts27010_timer_para_t *timer_para);
inline void ts27010_inc_timer_para(struct ts27010_timer_para_t *timer_para,
	unsigned long para);
inline void ts27010_clear_timer_para(struct ts27010_timer_para_t *timer_para,
	unsigned long para);
inline void ts27010_get_timer_para(struct ts27010_timer_para_t *timer_para,
	u8 *para);
inline void ts27010_sched_retran(struct ts27010_timer_para_t *timer_para);
/***************************** retran info ********************************/
struct ts27010_retran_info_t;

inline void ts27010_mod_retran_timer(
	struct ts27010_retran_info_t *retran_info);
inline void ts27010_stop_retran_timer(
	struct ts27010_retran_info_t *retran_info);
inline u8 *ts27010_get_retran_data(struct ts27010_retran_info_t *retran_info);
inline u16 ts27010_get_retran_len(struct ts27010_retran_info_t *retran_info);
inline u8 ts27010_get_retran_count(struct ts27010_retran_info_t *retran_info);
inline u8 ts27010_inc_retran_count(struct ts27010_retran_info_t *retran_info);
inline void ts27010_clean_retran_count(
	struct ts27010_retran_info_t *retran_info);
inline u8 ts27010_get_retran_sn(struct ts27010_retran_info_t *retran_info);

/***************************** slide window ********************************/

struct ts27010_slide_window_t;

struct ts27010_slide_window_t *ts27010_slidewindow_alloc(
		void (*timer_func)(unsigned long para));
inline void ts27010_slidewindow_free(struct ts27010_slide_window_t *sldwin);

u8 ts27010_slidewindo_store(struct ts27010_slide_window_t *slide_window,
					u8 *data, u32 len);
u8 ts27010_slidewindow_is_sn_in(
	struct ts27010_slide_window_t *slide_window, u8 sn);
void ts27010_slidewindow_clear(struct ts27010_slide_window_t *slide_window);
void ts27010_clear_retran_counter(struct ts27010_slide_window_t *slide_window,
	struct ts27010_timer_para_t *timer_para, u8 index, int include);

/* calculate window filled size*/
inline u8 ts27010_slidewindow_level(struct ts27010_slide_window_t *sldwin);
/* calculate window free size */
inline u8 ts27010_slidewindow_room(struct ts27010_slide_window_t *sldwin);
inline u8 ts27010_slidewindow_head(struct ts27010_slide_window_t *sldwin);
/*
inline u8 ts27010_slidewindow_inc_head(
	struct ts27010_slide_window_t *sldwin, u8 val);
*/
inline u8 ts27010_slidewindow_tail(struct ts27010_slide_window_t *sldwin);
inline u8 ts27010_slidewindow_set_tail(
	struct ts27010_slide_window_t *sldwin, u8 val);
inline int ts27010_slidewindow_lock(struct ts27010_slide_window_t *sldwin);
inline void ts27010_slidewindow_unlock(struct ts27010_slide_window_t *sldwin);
inline void ts27010_slidewindow_wakeup(struct ts27010_slide_window_t *sldwin);

/* Is the index in the slide window? */
/* return value: 0-no, 1-yes */
inline u8 ts27010_slidewindow_is_idx_in(
	struct ts27010_slide_window_t *slide_window, u8 index);
/* get ith datum, datum still in window */
inline struct ts27010_retran_info_t *ts27010_slidewindow_peek(
	struct ts27010_slide_window_t *sldwin, u8 i);

/* free count space */
inline int ts27010_slidewindow_consume(struct ts27010_slide_window_t *sldwin,
					  u8 count);

#endif /* TS27010_UART_RETRAN */

/***************************** mux logger ********************************/
#ifdef MUX_USB_LOGGER
struct ts27010_mux_logger;
struct ts27010_mux_logger *ts27010_alloc_mux_usb_logger(void);
int ts27010_mux_usb_logger_logdata(
	struct ts27010_mux_logger *, u8 *data, int len, int type);
void ts27010_free_mux_usb_logger(struct ts27010_mux_logger *mux_logger);
#endif

#endif /* __TS27010_MISC_H__ */
