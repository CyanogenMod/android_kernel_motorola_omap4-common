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
#include <linux/time.h>

#include <asm/system.h>
#include <linux/uaccess.h>

#include "ts27010.h"
#include "ts27010_mux.h"
#include "ts27010_td.h"
#include "ts27010_misc.h"

#ifdef DEBUG
/***************************** mux log ********************************/
int g_mux_usb_print_level;
int g_mux_usb_dump_frame;
int g_mux_usb_dump_user_data;

static u8 s_hexdump_buf[TS0710MUX_SEND_BUF_SIZE * 3 + 2 + 64];

/*/
void mux_usb_hexdump(int level, const char *title,
	const char *function, u32 line, const u8 *buf, u32 len)
{
	u32 i, j, size = TS0710MUX_SEND_BUF_SIZE * 3;
	if (level < g_mux_usb_print_level)
		return;

	printk(KERN_WARNING "MUX_USB[%s:%d]: ", function, line);
	printk(KERN_WARNING "%s - hexdump(len=%lu): ",
		title, (unsigned long) len);
	if (buf == NULL) {
		printk(KERN_WARNING " [NULL]");
	} else {
		for (i = 0, j = 0; i < len && j < size; i++, j += 3)
			//printk(KERN_WARNING " %02x ", buf[i]);
			sprintf(s_hexdump_buf + j, " %02x", buf[i]);
		s_hexdump_buf[j++] = '\n';
		s_hexdump_buf[j] = '\0';
		printk(KERN_WARNING "%s", s_hexdump_buf);
	}
}
/*/

void mux_usb_hexdump(int level, const char *title, const char *function,
	u32 line, const u8 *buf, u32 len)
{
	u32 i, j, size = TS0710MUX_SEND_BUF_SIZE * 3;
	if (level < g_mux_usb_print_level)
		return;
	printk(KERN_WARNING "MUX_USB[%s:%d]: ", function, line);

	if (buf == NULL) {
		printk(KERN_WARNING " [NULL]");
	} else {
		snprintf(s_hexdump_buf, 63,
			"MUX_USB: hexdump[%s-%d]: ", title, len);
		s_hexdump_buf[63] = 0;
		j = strlen(s_hexdump_buf);

		for (i = 0; i < len && j < size; i++, j += 3)
			sprintf(s_hexdump_buf + j, " %02x", buf[i]);
		s_hexdump_buf[j++] = '\n';
		s_hexdump_buf[j] = '\0';
		printk(KERN_WARNING "%s", s_hexdump_buf);
	}
}

#ifdef PROC_DEBUG_MUX
#ifdef PROC_DEBUG_MUX_STAT
static int mux_proc_stat_read(char *page, char **start, off_t off,
	int count, int *eof, void *data)
{
	int len = 0;

	if (off > 0) {
		*eof = 1;
		return 0;
	}

	ts27010_tty_usb_dump_io();
	ts27010_ldisc_usb_drv_stat();
	ts27010_mux_usb_mux_stat();
/*/	modem_usb_dump_stat(); /*/

	return len;
}

static ssize_t mux_proc_stat_write(struct file *flip, const char __user *buff,
	unsigned long len, void *data)
{
#define PROC_BUF_LEN 16
	char k_buf[PROC_BUF_LEN];
	char *startp;
	char *endp;
	unsigned long para;
	int count = PROC_BUF_LEN < len ? PROC_BUF_LEN : len;
	int ret = 0;

	memset(k_buf, 0, sizeof(k_buf));
	if (copy_from_user(k_buf, buff, count)) {
		ret = -EFAULT;
		goto err;
	} else {
		mux_print(MSG_INFO, "k_buf: %x, %s", (u32)k_buf, k_buf);
		startp = k_buf;
		para = simple_strtoul(startp, &endp, 10);
		if (endp == startp) {
			ret = -EINVAL;
			goto err;
		}
		g_nStatDrvIO = (int)para;
		mux_print(MSG_INFO, "stat mux=%d", g_nStatDrvIO);
		if (g_nStatDrvIO) {
			ts27010_tty_usb_dump_io_clear();
			ts27010_ldisc_usb_drv_stat_clear();
			ts27010_mux_usb_mux_stat_clear();
/*/			modem_usb_dump_stat_clear(); /*/
		}
		ret = count;
	}
err:
	return ret;
}
#endif

static int mux_proc_read(char *page, char **start, off_t off,
	int count, int *eof, void *data)
{
	int len = 0;

	if (off > 0) {
		*eof = 1;
		return 0;
	}

	len += sprintf(page + len,
		"print_level: %d, dump_frame: %d, dump_user_data: %d\n",
		g_mux_usb_print_level, g_mux_usb_dump_frame,
		g_mux_usb_dump_user_data);

	return len;
}

static ssize_t mux_proc_write(struct file *flip, const char __user *buff,
	unsigned long len, void *data)
{
#define PROC_BUF_LEN 16
	char k_buf[PROC_BUF_LEN];
	char *startp;
	char *endp;
	unsigned long para;
	int count = PROC_BUF_LEN < len ? PROC_BUF_LEN : len;
	int ret = 0;

	memset(k_buf, 0, sizeof(k_buf));
	if (copy_from_user(k_buf, buff, count)) {
		ret = -EFAULT;
		goto err;
	} else {
		mux_print(MSG_INFO, "k_buf: %x, %s", (u32)k_buf, k_buf);
		startp = k_buf;
		para = simple_strtoul(startp, &endp, 10);
		if (endp == startp) {
			ret = -EINVAL;
			goto err;
		}
		if (para <= MSG_NONE) {
			g_mux_usb_print_level = (int)para;
			mux_print(MSG_INFO, "print_level=%d",
				g_mux_usb_print_level);
		} else {
			mux_print(MSG_INFO, "invalid print_level: %d "
				"and the log level should be 0~%d",
				(int)para, MSG_NONE);
		}
		startp = endp + 1;
		para = simple_strtoul(startp, &endp, 10);
		if (endp == startp) {
			ret = count;
			goto err;
		}
		g_mux_usb_dump_frame = para ? 1 : 0;
		mux_print(MSG_INFO, "dump_frame=%d", g_mux_usb_dump_frame);
		startp = endp + 1;
		para = simple_strtoul(startp, &endp, 10);
		if (endp == startp) {
			ret = count;
			goto err;
		}
		g_mux_usb_dump_user_data = para ? 1 : 0;
		mux_print(MSG_INFO, "dump_user_data=%d",
			g_mux_usb_dump_user_data);

		ret = count;
	}
err:
	return ret;
}

typedef int (*PROC_READ)(
	char *page, char **start, off_t off,
	int count, int *eof, void *data);
typedef ssize_t (*PROC_WRITE)(
	struct file *flip, const char __user *buff,
	unsigned long len, void *data);

static struct proc_dir_entry *ts27010_create_proc(const char *name,
	PROC_READ read, PROC_WRITE write)
{
	struct proc_dir_entry *proc_mux =
		create_proc_entry(name, S_IRUSR | S_IWUSR, NULL);
	if (proc_mux == NULL) {
		printk(KERN_WARNING "create muxusb proc file failed!\n");
	} else {
		proc_mux->read_proc = read;
		proc_mux->write_proc = write;
#ifdef TEST_BH
		proc_mux->owner = THIS_MODULE;
#endif
	}
	return proc_mux;
}

struct proc_dir_entry *ts27010_usb_proc_alloc(void)
{
	return ts27010_create_proc("muxusb", mux_proc_read, mux_proc_write);
}

#ifdef PROC_DEBUG_MUX_STAT
struct proc_dir_entry *ts27010_usb_proc_stat_alloc(void)
{
	return ts27010_create_proc("muxusb_stat",
		mux_proc_stat_read, mux_proc_stat_write);
}
#endif

inline void ts27010_usb_proc_free(struct proc_dir_entry *proc_entry,
	const char *name)
{
	remove_proc_entry(name, NULL);
	proc_entry = NULL;
}
#endif /* PROC_DEBUG_MUX */
#endif /* DEBUG */


#ifdef TS27010_UART_RETRAN
/***************************** sequence number ********************************/
struct sequence_number_t {
	struct mutex m_lock; /* sequence number r/w lock */
	u8 m_sn;
};

struct sequence_number_t *ts27010_sequence_number_alloc(void)
{
	struct sequence_number_t *seq_no = NULL;
	FUNC_ENTER();

	seq_no = kzalloc(sizeof(*seq_no), GFP_KERNEL);
	if (seq_no == NULL) {
		mux_print(MSG_WARNING, "request memory failed\n");
	} else {
		seq_no->m_sn = 0;
		mutex_init(&seq_no->m_lock);
	}
	FUNC_EXIT();
	return seq_no;
}

inline void ts27010_sequence_number_free(struct sequence_number_t *seq_no)
{
	kfree(seq_no);
	seq_no = NULL;
}

inline int ts27010_sequence_number_lock(struct sequence_number_t *seq_no)
{
	FUNC_ENTER();
	mutex_lock(&seq_no->m_lock);
	FUNC_EXIT();
	return 0;
}

inline void ts27010_sequence_number_unlock(struct sequence_number_t *seq_no)
{
	FUNC_ENTER();
	mutex_unlock(&seq_no->m_lock);
	FUNC_EXIT();
}

inline u8 ts27010_sequence_number_get(struct sequence_number_t *seq_no)
{
	return seq_no->m_sn;
}

inline void ts27010_sequence_number_inc(struct sequence_number_t *seq_no)
{
/*/	seq_no->m_sn = (seq_no->m_sn + 1) % MAX_TRANS_SN; /*/
	seq_no->m_sn = (seq_no->m_sn + 1) & (MAX_TRANS_SN - 1);
}

/***************************** timer param ********************************/
struct ts27010_timer_para_t {
	spinlock_t m_locker; /* timer modification lock */
	struct work_struct m_send_work;
	u8 m_cur_index[SLIDE_WINDOWS_SIZE_AP];
};

struct ts27010_timer_para_t *ts27010_alloc_timer_para(
	void (*p)(struct work_struct *work))
{
	struct ts27010_timer_para_t *timer_para = NULL;
	FUNC_ENTER();

	timer_para = kzalloc(sizeof(*timer_para), GFP_KERNEL);
	if (timer_para == NULL) {
		mux_print(MSG_WARNING, "request memory failed\n");
	} else {
		timer_para->m_locker =
			__SPIN_LOCK_UNLOCKED(timer_para->m_locker);
		memset(timer_para->m_cur_index, 0,
			sizeof(timer_para->m_cur_index));

		INIT_WORK(&timer_para->m_send_work, p);
	}
	FUNC_EXIT();
	return timer_para;
}

inline void ts27010_timer_para_free(struct ts27010_timer_para_t *timer_para)
{
	kfree(timer_para);
	timer_para = NULL;
}

inline void ts27010_inc_timer_para(struct ts27010_timer_para_t *timer_para,
	unsigned long para)
{
	spin_lock_bh(&timer_para->m_locker);
	++timer_para->m_cur_index[para];
	spin_unlock_bh(&timer_para->m_locker);
}

inline void ts27010_clear_timer_para(struct ts27010_timer_para_t *timer_para,
	unsigned long para)
{
	spin_lock_bh(&timer_para->m_locker);
	timer_para->m_cur_index[para] = 0;
	spin_unlock_bh(&timer_para->m_locker);
}

inline void ts27010_get_timer_para(struct ts27010_timer_para_t *timer_para,
	u8 *para)
{
	spin_lock_bh(&timer_para->m_locker);
	memcpy(para, timer_para->m_cur_index,
		sizeof(u8) * SLIDE_WINDOWS_SIZE_AP);
	memset(timer_para->m_cur_index, 0, sizeof(u8) * SLIDE_WINDOWS_SIZE_AP);
	spin_unlock_bh(&timer_para->m_locker);
}

inline void ts27010_sched_retran(struct ts27010_timer_para_t *timer_para)
{
#ifdef QUEUE_SELF
	queue_work(g_mux_usb_queue, &timer_para->m_send_work);
#else
	schedule_work(&timer_para->m_send_work);
#endif
}

/***************************** retran info ********************************/
struct ts27010_retran_info_t {
	struct timer_list m_tl; /* retran timer */
	u8 m_data[TS0710MUX_SEND_BUF_SIZE]; /* frame data */
	u16 m_length; /* frame length */
	/* if one frame retraned more than 10 times, trigger panic */
	u8 m_counter;
	u8 m_sn;
};

inline void ts27010_mod_retran_timer(
	struct ts27010_retran_info_t *retran_info)
{
	mod_timer(&retran_info->m_tl, (jiffies + RETRAN_TIMEOUT));
}

inline void ts27010_stop_retran_timer(
	struct ts27010_retran_info_t *retran_info)
{
	del_timer_sync(&retran_info->m_tl);
}

inline u8 *ts27010_get_retran_data(struct ts27010_retran_info_t *retran_info)
{
	return retran_info->m_data;
}

inline u16 ts27010_get_retran_len(struct ts27010_retran_info_t *retran_info)
{
	return retran_info->m_length;
}

inline u8 ts27010_get_retran_count(struct ts27010_retran_info_t *retran_info)
{
	return retran_info->m_counter;
}

inline u8 ts27010_inc_retran_count(struct ts27010_retran_info_t *retran_info)
{
	return ++retran_info->m_counter;
}

inline void ts27010_clean_retran_count(
	struct ts27010_retran_info_t *retran_info)
{
	del_timer_sync(&retran_info->m_tl);
	memset(retran_info->m_data, 0, sizeof(u8) * TS0710MUX_SEND_BUF_SIZE);
	retran_info->m_length = 0;
	retran_info->m_counter = 0;
	retran_info->m_sn = 0xFF;
}

inline u8 ts27010_get_retran_sn(struct ts27010_retran_info_t *retran_info)
{
	return retran_info->m_sn;
}

/***************************** slide window ********************************/
struct ts27010_slide_window_t {
	struct mutex m_lock; /* slide window r/w lock */
	/* if slide window full, sleep on this wait queue */
	wait_queue_head_t m_retran_wait;
	struct ts27010_retran_info_t m_retran_info[SLIDE_WINDOWS_SIZE_AP];
	u8 m_head, m_tail;
};

/* calculate buffer filled size*/
inline u8 ts27010_slidewindow_level(struct ts27010_slide_window_t *sldwin)
{
	int level = sldwin->m_head - sldwin->m_tail;

	if (level < 0)
		level = SLIDE_WINDOWS_SIZE_AP + level;

	return level;
}

/* calculate buffer free size */
inline u8 ts27010_slidewindow_room(struct ts27010_slide_window_t *sldwin)
{
	return SLIDE_WINDOWS_SIZE_AP - ts27010_slidewindow_level(sldwin) - 1;
}

inline u8 ts27010_slidewindow_head(struct ts27010_slide_window_t *sldwin)
{
	return sldwin->m_head;
}

/*
inline u8 ts27010_slidewindow_inc_head(
	struct ts27010_slide_window_t *sldwin, u8 val)
{
	sldwin->m_head = val;
	return val;
}
*/

inline u8 ts27010_slidewindow_tail(struct ts27010_slide_window_t *sldwin)
{
	return sldwin->m_tail;
}

inline u8 ts27010_slidewindow_set_tail(
	struct ts27010_slide_window_t *sldwin, u8 val)
{
	if (sldwin->m_tail < slidwin->m_head) {
		WARN_ON(val > slidwin->m_head || val <= slidwind->m_tail);
	} else if (sldwin->m_tail > slidwin->m_head) {
		WARN_ON(val > slidwin->m_head && val <= slidwind->m_tail);
	} else {
		WARN_ON(val != slidwin->m_tail);
	}
	sldwin->m_tail = val % SLIDE_WINDOWS_SIZE_AP;
	return val;
}

inline int ts27010_slidewindow_lock(struct ts27010_slide_window_t *sldwin)
{
	mutex_lock(&sldwin->m_lock);
	return 0;
}

inline void ts27010_slidewindow_unlock(struct ts27010_slide_window_t *sldwin)
{
	mutex_unlock(&sldwin->m_lock);
}

inline void ts27010_slidewindow_wakeup(struct ts27010_slide_window_t *sldwin)
{
	wake_up_interruptible(&sldwin->m_retran_wait);
}

/* Is the index in the slide window? */
/* return value: 0-no, 1-yes */
inline u8 ts27010_slidewindow_is_idx_in(
	struct ts27010_slide_window_t *slide_window, u8 index)
{
	FUNC_ENTER();
	if (slide_window->m_head > slide_window->m_tail) {
		if ((index >= slide_window->m_tail)
			&& (index < slide_window->m_head)) {
			FUNC_EXIT();
			return 1;
		} else {
			mux_print(MSG_DEBUG,
				"index not in window, "
				"index=%d, head=%d, tail=%d",
				index,
				slide_window->m_head, slide_window->m_tail);
			return 0;
		}
	} else if (slide_window->m_head < slide_window->m_tail) {
		if ((index >= slide_window->m_head)
			&& (index < slide_window->m_tail)) {
			mux_print(MSG_DEBUG,
				"index not in_window, "
				"index=%d, head=%d, tail=%d",
				index,
				slide_window->m_head, slide_window->m_tail);
			return 0;
		} else {
			FUNC_EXIT();
			return 1;
		}
	} else {
		/* head == tail, no frame in the window */
		mux_print(MSG_DEBUG,
			"index not in window, index = %d, head = tail = %d\n",
			index, slide_window->m_head);
		return 0;
	}
}

/* get ith datum, datum still in rbuf */
inline struct ts27010_retran_info_t *ts27010_slidewindow_peek(
	struct ts27010_slide_window_t *sldwin, u8 i)
{
/*/
	struct ts27010_retran_info_t *retran_info = NULL;

	if (ts27010_slidewindow_is_idx_in(sldwin, i)
		retran_info = &sldwin->m_retran_info[(sldwin->m_tail + i)
			% SLIDE_WINDOWS_SIZE_AP];
	else
		retran_info = NULL;
	retran_info = &sldwin->m_retran_info[i % SLIDE_WINDOWS_SIZE_AP];
	return retran_info;
/*/
	return sldwin->m_retran_info + (i % SLIDE_WINDOWS_SIZE_AP);
}

/* free count space */
inline int ts27010_slidewindow_consume(struct ts27010_slide_window_t *sldwin,
					  u8 count)
{
	count = min(count, ts27010_slidewindow_level(sldwin));

	sldwin->m_tail = (sldwin->m_tail + count) % SLIDE_WINDOWS_SIZE_AP;

	return count;
}

static void reset_slide_window(struct ts27010_slide_window_t *slide_window)
{
	int i;
	struct ts27010_retran_info_t *retran_info;
	FUNC_ENTER();

	slide_window->m_head = 0;
	slide_window->m_tail = 0;
	for (i = 0; i < SLIDE_WINDOWS_SIZE_AP; i++) {
		retran_info = &slide_window->m_retran_info[i];
		memset(retran_info->m_data, 0,
			sizeof(u8) * TS0710MUX_SEND_BUF_SIZE);
		retran_info->m_length = 0;
		retran_info->m_counter = 0;
		retran_info->m_sn = 0xFF;
	}
	FUNC_EXIT();
}

static void init_slide_window_timer(
	struct ts27010_slide_window_t *slide_window,
	void (*timer_func)(unsigned long para))
{
	int i;
	struct ts27010_retran_info_t *retran_info;
	FUNC_ENTER();

	for (i = 0; i < SLIDE_WINDOWS_SIZE_AP; i++) {
		retran_info = &slide_window->m_retran_info[i];

		init_timer(&retran_info->m_tl);
		retran_info->m_tl.function = timer_func;
		retran_info->m_tl.data = (unsigned long)i;
	}
	FUNC_EXIT();
}

static void init_slide_window(struct ts27010_slide_window_t *slide_window,
		void (*timer_func)(unsigned long para))
{
	FUNC_ENTER();
	mutex_init(&slide_window->m_lock);
	init_waitqueue_head(&slide_window->m_retran_wait);

	reset_slide_window(slide_window);
	init_slide_window_timer(slide_window, timer_func);
}

struct ts27010_slide_window_t *ts27010_slidewindow_alloc(
		void (*timer_func)(unsigned long para))
{
	struct ts27010_slide_window_t *slide_window;
	FUNC_ENTER();

	slide_window = kzalloc(sizeof(*slide_window), GFP_KERNEL);
	if (slide_window == NULL) {
		mux_print(MSG_WARNING, "request memory failed\n");
		slide_window = NULL;
	} else
		init_slide_window(slide_window, timer_func);

	return slide_window;
}

inline void ts27010_slidewindow_free(struct ts27010_slide_window_t *sldwin)
{
	kfree(sldwin);
	sldwin = NULL;
}


/* before send a frame, put the frame into slide window for retransmission */
/* return value: 0xFF, blocked; 0~5, window index */
u8 ts27010_slidewindo_store(struct ts27010_slide_window_t *slide_window,
			u8 *data, u32 len)
{
	u8 index;
	u32 data_len = 0;
	struct short_frame *short_pkt;
	struct ts27010_retran_info_t *retran_info;
	u8 dlci = *(data + ADDRESS_OFFSET) >> 2;
	FUNC_ENTER();

	index = 0xFF;
	ts27010_slidewindow_lock(slide_window);
	if (len < TS0710_FRAME_SIZE(0)) {
		mux_print(MSG_ERROR, "too short frame length=%d\n", (int)len);
		goto EXIT;
	}
	if (ts27010_slidewindow_room(slide_window) == 0) {
		int retry = 10;
		mux_print(MSG_INFO, "slide window is full, head=%d, tail=%d\n",
			slide_window->m_head, slide_window->m_tail);
		while (retry--) {/* put task into retran wait queue */
			int ret;
			ts27010_slidewindow_unlock(slide_window);
			ret = wait_event_interruptible_timeout(
				slide_window->m_retran_wait,
				((slide_window->m_head + 1)
					% SLIDE_WINDOWS_SIZE_AP)
				!= slide_window->m_tail, TS0710MUX_TIME_OUT);
			ts27010_slidewindow_lock(slide_window);
			if (ret == 0) {/* timeout */
				mux_print(MSG_WARNING,
					"wait for slidewindow timeout\n");
				continue;
			} else if (ret == -ERESTARTSYS) {/* got signal */
				mux_print(MSG_WARNING, "DLCI: %d Wait for "
					"store_into_slide_window got signal!\n",
					dlci);
				goto EXIT;
			}
			break;
		}
		if (retry < 0) {
			mux_print(MSG_ERROR, "request slidewindow failed\n");
			goto EXIT;
		}
	}

	/* insert sn and calculate CRC */
	short_pkt = (struct short_frame *)(data + ADDRESS_OFFSET);
	ts27010_sequence_number_lock(g_ap_send_sn);
	short_pkt->h.sn = ts27010_sequence_number_get(g_ap_send_sn);
	ts27010_sequence_number_inc(g_ap_send_sn);
	ts27010_sequence_number_unlock(g_ap_send_sn);

	if ((short_pkt->h.length.ea) == 0) {
		struct long_frame *long_pkt =
			(struct long_frame *)(data + ADDRESS_OFFSET);
		data_len = (long_pkt->h.length.h_len << 7)
			| long_pkt->h.length.l_len;
		long_pkt->data[data_len] = ts0710_usb_crc_data((u8 *)long_pkt,
			(LONG_CRC_CHECK + data_len));
	} else {
		data_len = short_pkt->h.length.len;
		short_pkt->data[data_len] = ts0710_usb_crc_data((u8 *)short_pkt,
			(SHORT_CRC_CHECK + data_len));
	}

	/* store frame to the window */
	/* should sort them as sn ascendingly? */
	retran_info = &slide_window->m_retran_info[slide_window->m_head];
	memcpy(retran_info->m_data, data, len);
	retran_info->m_length = len;
	retran_info->m_sn = short_pkt->h.sn;
	/* update head */
	index = slide_window->m_head;
	slide_window->m_head = (slide_window->m_head + 1)
		% SLIDE_WINDOWS_SIZE_AP;
	mux_print(MSG_DEBUG, "frame %x on dlci %x stored into slide window,\n"
		"window head= %d, tail=%d\n",
		retran_info->m_sn, dlci,
		slide_window->m_head, slide_window->m_tail);

EXIT:
	ts27010_slidewindow_unlock(slide_window);
	FUNC_EXIT();

	return index;
}

/* Is the sn in the slide window? */
/* sn: received sequence number, including OK/Error info */
/* return value:
	0xFF - not in window, error sn
	0xFE - not in window, the next sn need to send,
		should clean all slide window
	0~4 -in window, return window index
*/
u8 ts27010_slidewindow_is_sn_in(
	struct ts27010_slide_window_t *slide_window, u8 sn)
{
	u8 number = sn & 0x7F; /* remove error flag */
	struct ts27010_retran_info_t *retran_info;
	u8 index;
	FUNC_ENTER();

	index = slide_window->m_tail;
	/*
	 * before this function called,
	 * the slide_window->m_lock has been locked,
	 * so it's safe
	 */
	if (slide_window->m_tail == slide_window->m_head) {
		mux_print(MSG_WARNING, "Empty slide window\n");
		return 0xFF;
	}
	if (index >= SLIDE_WINDOWS_SIZE_AP)
		index %= SLIDE_WINDOWS_SIZE_AP;

	while (index != slide_window->m_head) {
		retran_info = ts27010_slidewindow_peek(slide_window, index);

		if (ts27010_get_retran_sn(retran_info) == number) {
			mux_print(MSG_DEBUG, "Got a frame(sn %x) "
				"in slide window(index %d)\n",
				ts27010_get_retran_sn(retran_info), index);
			return index;
		}

		index = (index + 1) % SLIDE_WINDOWS_SIZE_AP;
	}
	/* here, index == head, get the last frame */
	retran_info = ts27010_slidewindow_peek(slide_window, index - 1);
	if (((number - ts27010_get_retran_sn(retran_info)) == 1)
		|| ((ts27010_get_retran_sn(retran_info) - number)
			== (MAX_TRANS_SN - 1))) {
		mux_print(MSG_INFO, "BP need next frame %x(current %x) "
			"which not in slide window\n",
			number, ts27010_get_retran_sn(retran_info));
		return 0xFE;
	}

	FUNC_EXIT();
	return 0xFF;
}

void ts27010_slidewindow_clear(struct ts27010_slide_window_t *slide_window)
{
	u8 j;
	FUNC_ENTER();

	for (j = slide_window->m_tail; j != slide_window->m_head; ) {
		ts27010_clean_retran_count(
			ts27010_slidewindow_peek(slide_window, j));
		j = (j + 1) % SLIDE_WINDOWS_SIZE_AP;
	}
	slide_window->m_tail = 0;
	slide_window->m_head = 0;
	FUNC_EXIT();
}

void ts27010_clear_retran_counter(struct ts27010_slide_window_t *slide_window,
	struct ts27010_timer_para_t *timer_para, u8 index, int include)
{
	u8 j;
	FUNC_ENTER();

	for (j = slide_window->m_tail; j != index; ) {
		ts27010_clean_retran_count(
			ts27010_slidewindow_peek(slide_window, j));
		/* need lock it? */
		timer_para->m_cur_index[j] = 0;
		j = (j + 1) % SLIDE_WINDOWS_SIZE_AP;
	}
	if (include) {
		ts27010_clean_retran_count(
			ts27010_slidewindow_peek(slide_window, index));
		/* need lock it? */
		timer_para->m_cur_index[index] = 0;
	}
	FUNC_EXIT();
}
#endif /* TS27010_UART_RETRAN */


