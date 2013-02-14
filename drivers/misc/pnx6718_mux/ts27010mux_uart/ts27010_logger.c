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
#include <linux/kthread.h>

#include <asm/system.h>
/* #include <asm/uaccess.h> */
#include <linux/uaccess.h>

#include "ts27010.h"
#include "ts27010_mux.h"
#include "ts27010_td.h"
#include "ts27010_misc.h"

/***************************** mux logger ********************************/
#ifdef MUX_UART_LOGGER

#define MUX_LOG_FILE "/data/mux_uart.log"
#define MUX_OLD_LOG_FILE "/data/mux_uart_old.log"

struct mux_logger_list_t {
	struct list_head list;
	int size;
	u8 *body;
};

struct ts27010_mux_logger {
	spinlock_t m_locker; /* logger list write lock */
	/*
	long m_archiver_pid;
	struct completion m_archiver_exited;
	*/
	struct task_struct *m_archiver_tsk;
	struct list_head m_datalist;
	struct semaphore m_sem;
	struct file *m_fp;
};

/* type: FA means from BP, FB means to BP */
static struct mux_logger_list_t *ts27010_mux_uart_logger_savedata(
	u8 *buf, int len, int type)
{
	struct mux_logger_list_t *mux_list = NULL;
	u8 offset = 0;

	mux_list = (struct mux_logger_list_t *)kzalloc(
		sizeof(struct mux_logger_list_t), GFP_ATOMIC);
	if (!mux_list) {
		mux_print(MSG_ERROR, "can not malloc (%d) memory!\n",
			sizeof(struct mux_logger_list_t));
		return NULL;
	}

	if (buf[0] == TS0710_BASIC_FLAG)
		offset = 1;
	else
		offset = 0;
	mux_list->body = (u8 *)kmalloc(len + offset, GFP_ATOMIC);
	if (!(mux_list->body)) {
		kfree(mux_list);
		mux_print(MSG_ERROR, "can not malloc (%d) memory!\n", len);
		return NULL;
	}
	if (buf[0] == TS0710_BASIC_FLAG)
		mux_list->body[0] = type + 0xFA;
	mux_list->size = len + offset;
	memcpy(mux_list->body + offset, buf, len);

	return mux_list;
}

int ts27010_mux_uart_logger_logdata(struct ts27010_mux_logger *mux_logger,
	u8 *buf, int len, int type)
{
	int ret = -ENOMEM;
	struct mux_logger_list_t *mux_list;

	if (len <= 0) {
		mux_print(MSG_WARNING, "Don't log empty data\n");
		return 0;
	}
	if (!mux_logger)
		return ret;

	mux_list = ts27010_mux_uart_logger_savedata(buf, len, type);
	if (mux_list) {
		unsigned long flags = 0;

		spin_lock_irqsave(&mux_logger->m_locker, flags);
		list_add_tail(&mux_list->list, &mux_logger->m_datalist);
		spin_unlock_irqrestore(&mux_logger->m_locker, flags);
		up(&mux_logger->m_sem);
		mux_print(MSG_DEBUG, "%d type %d into log buffer\n", type, len);
		ret = 0;
	}
	return ret;
}

static void mux_backup_oldlogfile(
	struct file *fp, char *curfile, char *old_file)
{
	struct file *ofp;

	ofp = filp_open(old_file,
		O_WRONLY | O_LARGEFILE | O_TRUNC | O_SYNC | O_CREAT,
		0644);
	if (!IS_ERR(ofp)) {
		char *buf;
		int size;
		struct kstat stat;
		int error;

		mux_print(MSG_INFO, "open mux backup log file successfully\n");

		error = vfs_stat(curfile, &stat);
		if (error) {
			mux_print(MSG_WARNING, "get file stat error,");
			filp_close(ofp, NULL);
			return;
		}
		size = stat.size;
		buf = (u8 *)kmalloc(size, GFP_KERNEL);
		if (!buf) {
			mux_print(MSG_ERROR, "alloc buffer failed\n");
			filp_close(ofp, NULL);
			return;
		}
		fp->f_op->read(fp, buf, size, &fp->f_pos);
		ofp->f_op->write(ofp, buf, size, &ofp->f_pos);
		kfree(buf);
		filp_close(ofp, NULL);
	}
}

static struct file *mux_logger_open_file(char *filename)
{
	struct file *fp;

	fp = filp_open(filename, O_RDONLY, 0);
	if (!IS_ERR(fp)) {
		/* backup old log file */
		mux_print(MSG_WARNING, "log file exists, will lost!\n");
		mux_backup_oldlogfile(fp, filename, MUX_OLD_LOG_FILE);
		filp_close(fp, NULL);
	}
	fp = filp_open(filename,
		O_WRONLY | O_LARGEFILE | O_TRUNC | O_SYNC | O_CREAT, 0644);
	if (!IS_ERR(fp))
		mux_print(MSG_INFO, "open mux log file successfully\n");
	else
		fp = NULL;

	return fp;
}

static struct file *mux_logger_getfs(struct ts27010_mux_logger *mux_logger)
{
	if (!mux_logger->m_fp)
		mux_logger->m_fp = mux_logger_open_file(MUX_LOG_FILE);
	return mux_logger->m_fp;
}

static void mux_logger_archive_data(
	struct ts27010_mux_logger *mux_logger, struct file *fp)
{
	int sent, count = 0;
	mm_segment_t oldfs;
	struct mux_logger_list_t *entry;
	struct list_head *ptr;
	unsigned long flags = 0;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	spin_lock_irqsave(&mux_logger->m_locker, flags);
	while (!(list_empty(&mux_logger->m_datalist))) {
		ptr = mux_logger->m_datalist.next;
		entry = list_entry(ptr, struct mux_logger_list_t, list);
		spin_unlock_irqrestore(&mux_logger->m_locker, flags);
/*/
	struct mux_logger_list_t *next;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	spin_lock_irqsave(&mux_logger->m_locker, flags);
	list_for_each_entry_safe(entry, next, &mux_logger->m_datalist, list) {
		spin_unlock_irqrestore(&mux_logger->m_locker, flags);
/*/
		sent = fp->f_op->write(fp, entry->body,
			entry->size, &fp->f_pos);
		mux_print(MSG_DEBUG, "%d data log archived\n", sent);
		count += sent;

		spin_lock_irqsave(&mux_logger->m_locker, flags);
		list_del(ptr);
		kfree(entry->body);
		kfree(entry);
	}
	spin_unlock_irqrestore(&mux_logger->m_locker, flags);
	set_fs(oldfs);
	mux_print(MSG_DEBUG, "total %d data log archived\n", count);
}

static int mux_logger_archive_thread(void *data)
{
	struct ts27010_mux_logger *mux_logger =
		(struct ts27010_mux_logger *)data;
	struct file *fp;

	if (!mux_logger)
		return -1;

	DAEMONIZE("mux_uart_logger");

	while (down_interruptible(&mux_logger->m_sem) == 0) {
		/* sleep since fs will be ready after phone power up 1 minutes*/
		fp = mux_logger_getfs(mux_logger);
		if (!fp)
			continue;
		mux_logger_archive_data(mux_logger, fp);
	}
	fp = mux_logger_getfs(mux_logger);
	if (fp) {
		mux_logger_archive_data(mux_logger, fp);
		filp_close(fp, NULL);
		fp = NULL;
	}

	mux_print(MSG_INFO, "thread exit\n");
	/*
	complete_and_exit(&mux_logger->m_archiver_exited, 0);
	*/

	return 0;
}

struct ts27010_mux_logger *ts27010_alloc_mux_logger(void)
{
	struct ts27010_mux_logger *mux_logger = NULL;

	mux_logger = (struct ts27010_mux_logger *)kzalloc(
		sizeof(struct ts27010_mux_logger), GFP_KERNEL);
	if (mux_logger == NULL) {
		mux_print(MSG_ERROR, "alloc ts27010_mux_logger failed\n");
		goto out;
	}

	INIT_LIST_HEAD(&mux_logger->m_datalist);
	mux_logger->m_locker = __SPIN_LOCK_UNLOCKED(mux_logger->m_locker);
	mux_logger->m_fp = NULL;

	/* init thread */
	/*
	init_completion(&mux_logger->m_archiver_exited);
	*/
	sema_init(&mux_logger->m_sem, 0);
	mux_logger->m_archiver_tsk = kthread_create(
		&mux_logger_archive_thread, mux_logger, "muxuart_logger");
	if (IS_ERR(mux_logger->m_archiver_tsk)) {
		mux_print(MSG_ERROR, "create log thread failed\n");
		goto out;
	}
	wake_up_process(mux_logger->m_archiver_tsk);

	return mux_logger;
out:
	kfree(mux_logger);
	return NULL;
}

void ts27010_free_mux_logger(struct ts27010_mux_logger *mux_logger)
{
	struct mux_logger_list_t *entry;
	struct list_head *ptr;
	unsigned long flags = 0;

	if (!mux_logger)
		return;
	if (mux_logger->m_archiver_tsk) {
		/*
		KILL_PROC(mux_logger->m_archiver_pid, SIGTERM);
		wait_for_completion(&mux_logger->m_archiver_exited);
		*/
		kthread_stop(mux_logger->m_archiver_tsk);
	}

	spin_lock_irqsave(&mux_logger->m_locker, flags);
	while (!(list_empty(&mux_logger->m_datalist))) {
		ptr = mux_logger->m_datalist.next;
		entry = list_entry(ptr, struct mux_logger_list_t, list);
		list_del(ptr);
		kfree(entry);
		kfree(entry->body);
	}
	spin_unlock_irqrestore(&mux_logger->m_locker, flags);

	kfree(mux_logger);
	mux_logger = NULL;
}
#endif

