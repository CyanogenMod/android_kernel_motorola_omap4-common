/*
 * drivers/media/video/media_cache_mgmt.c
 *
 * Copyright (C) 2011 Motorola Mobility Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/writeback.h>
#include <linux/gfp.h>
#include <linux/slab.h>

#include <../fs/internal.h> /* for inode_sb_list_lock */

struct media_cache_mgmt_work_struct {
	struct work_struct work;
};

struct media_cache_mgmt_main_struct {
	struct media_cache_mgmt_work_struct *work;
	struct workqueue_struct *workqueue;
};

static struct mutex lock;
static struct media_cache_mgmt_main_struct *mcm;
static unsigned int media_cache_mgmt;

static int media_cache_mgmt_set(const char *val, struct kernel_param *kp);

module_param_call(media_cache_mgmt, media_cache_mgmt_set, param_get_int,
					&media_cache_mgmt, 0644);


/*
 * Implement the manual drop-all-pagecache functions
 */

static void drop_pagecache_sb(struct super_block *sb, void *unused)
{
	struct inode *inode, *toput_inode = NULL;

	spin_lock(&inode_sb_list_lock);
	list_for_each_entry(inode, &sb->s_inodes, i_sb_list) {
		spin_lock(&inode->i_lock);
		if ((inode->i_state & (I_FREEING|I_WILL_FREE|I_NEW)) ||
				(inode->i_mapping->nrpages == 0)) {
			spin_unlock(&inode->i_lock);
			continue;
		}
		__iget(inode);
		spin_unlock(&inode->i_lock);
		spin_unlock(&inode_sb_list_lock);
		invalidate_mapping_pages(inode->i_mapping, 0, -1);
		iput(toput_inode);
		toput_inode = inode;
		spin_lock(&inode_sb_list_lock);
	}
	spin_unlock(&inode_sb_list_lock);
	iput(toput_inode);
}

static void drop_slab(void)
{
	int nr_objects;
	struct shrink_control shrink = {
		.gfp_mask = GFP_KERNEL,
	};

	do {
		nr_objects = shrink_slab(&shrink, 1000, 1000);
	} while (nr_objects > 10);
}

static int media_cache_mgmt_set(const char *val, struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);

	flush_workqueue(mcm->workqueue);
	if (media_cache_mgmt) {
		mutex_lock(&lock);
		queue_work(mcm->workqueue, &(mcm->work->work));
		mutex_unlock(&lock);
	}

	return ret;
}

static void media_cache_mgmt_process_work(struct work_struct *work)
{
	mutex_lock(&lock);
	if (media_cache_mgmt & 1)
		iterate_supers(drop_pagecache_sb, NULL);
	if (media_cache_mgmt & 2)
		drop_slab();
	mutex_unlock(&lock);
}

static int __init media_cache_mgmt_init_module(void)
{
	mutex_init(&lock);
	mcm = kzalloc(sizeof(struct media_cache_mgmt_main_struct), GFP_KERNEL);

	mcm->workqueue = create_singlethread_workqueue("media_cache_mgmt");
	mcm->work = kzalloc(sizeof(struct media_cache_mgmt_work_struct),
				GFP_KERNEL);
	INIT_WORK(&(mcm->work->work), media_cache_mgmt_process_work);

	return 0;
}

static void __exit media_cache_mgmt_exit_module(void)
{
	if (mcm->workqueue) {
		flush_workqueue(mcm->workqueue);
		destroy_workqueue(mcm->workqueue);
	}
	kfree(mcm->work);
	kfree(mcm);

	mutex_destroy(&lock);
}

module_init(media_cache_mgmt_init_module);
module_exit(media_cache_mgmt_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Motorola Mobility");
MODULE_DESCRIPTION("Media Cache Mgmt");

