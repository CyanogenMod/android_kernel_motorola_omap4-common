/*
 * Simple IO scheduler plus
 * Based on Noop, Deadline and V(R) IO schedulers.
 *
 * Copyright (C) 2012 Miguel Boton <mboton@gmail.com>
 *	     (C) 2013 Boy Petersen <boypetersen@gmail.com>
 *
 *
 * This algorithm does not do any kind of sorting, as it is aimed for
 * aleatory access devices, but it does some basic merging. We try to
 * keep minimum overhead to achieve low latency.
 *
 * Asynchronous and synchronous requests are not treated separately, but
 * we relay on deadlines to ensure fairness.
 *
 */
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/version.h>

enum { ASYNC, SYNC };

/* Tunables */
static const int sync_read_expire = (HZ / 16) * 5;	/* max time before a sync read is submitted. */
static const int sync_write_expire = (HZ / 8) * 23;	/* max time before a sync write is submitted. */

static const int async_read_expire = HZ * 4;	/* ditto for async, these limits are SOFT! */
static const int async_write_expire = HZ * 16;	/* ditto for async, these limits are SOFT! */

static const int writes_starved = 2;		/* max times reads can starve a write */
static const int async_reads_starved = 4;	/* max times sync reads can starve async reads */
static const int fifo_batch     = 1;		/* # of sequential requests treated as one
						   by the above parameters. For throughput. */

/* Elevator data */
struct sioplus_data {
	/* Request queues */
	struct list_head fifo_list[2][2];

	/* Attributes */
	unsigned int batched;
	unsigned int starved;
	unsigned int reads_starved;

	/* Settings */
	int fifo_expire[2][2];
	int fifo_batch;
	int writes_starved;
	int async_reads_starved;
};

static void
sioplus_merged_requests(struct request_queue *q, struct request *rq,
		    struct request *next)
{
	/*
	 * If next expires before rq, assign its expire time to rq
	 * and move into next position (next will be deleted) in fifo.
	 */
	if (!list_empty(&rq->queuelist) && !list_empty(&next->queuelist)) {
		if (time_before(rq_fifo_time(next), rq_fifo_time(rq))) {
			list_move(&rq->queuelist, &next->queuelist);
			rq_set_fifo_time(rq, rq_fifo_time(next));
		}
	}

	/* Delete next request */
	rq_fifo_clear(next);
}

static void
sioplus_add_request(struct request_queue *q, struct request *rq)
{
	struct sioplus_data *sd = q->elevator->elevator_data;
	const int sync = rq_is_sync(rq);
	const int data_dir = rq_data_dir(rq);

	/*
	 * Add request to the proper fifo list and set its
	 * expire time.
	 */
	rq_set_fifo_time(rq, jiffies + sd->fifo_expire[sync][data_dir]);
	list_add_tail(&rq->queuelist, &sd->fifo_list[sync][data_dir]);
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,38)
static int
sioplus_queue_empty(struct request_queue *q)
{
	struct sioplus_data *sd = q->elevator->elevator_data;

	/* Check if fifo lists are empty */
	return list_empty(&sd->fifo_list[SYNC][READ]) && list_empty(&sd->fifo_list[SYNC][WRITE]) &&
	       list_empty(&sd->fifo_list[ASYNC][READ]) && list_empty(&sd->fifo_list[ASYNC][WRITE]);
}
#endif

static struct request *
sioplus_expired_request(struct sioplus_data *sd, int sync, int data_dir)
{
	struct list_head *list = &sd->fifo_list[sync][data_dir];
	struct request *rq;

	if (list_empty(list))
		return NULL;

	/* Retrieve request */
	rq = rq_entry_fifo(list->next);

	/* Request has expired */
	if (time_after(jiffies, rq_fifo_time(rq)))
		return rq;

	return NULL;
}

static struct request *
sioplus_choose_expired_request(struct sioplus_data *sd)
{
	struct request *rq;

	/*
	 * Check expired requests.
	 * Asynchronous requests have priority over synchronous.
	 * Write requests have priority over read.
	 */
	rq = sioplus_expired_request(sd, ASYNC, WRITE);
	if (rq)
		return rq;
	rq = sioplus_expired_request(sd, ASYNC, READ);
	if (rq) {
		sd->reads_starved = 0;
		return rq;
	}

	rq = sioplus_expired_request(sd, SYNC, WRITE);
	if (rq)
		return rq;
	rq = sioplus_expired_request(sd, SYNC, READ);
	if (rq)
		return rq;


	return NULL;
}

static struct request *
sioplus_choose_request(struct sioplus_data *sd, int data_dir)
{
	struct list_head *sync = sd->fifo_list[SYNC];
	struct list_head *async = sd->fifo_list[ASYNC];

	/*
	 * Retrieve request from available fifo list.
	 * Synchronous requests have priority over asynchronous.
	 * Read requests have priority over write.
	 */
	if (!list_empty(&sync[data_dir])) {
		if (sd->reads_starved <= sd->async_reads_starved || data_dir == WRITE) {
			if (data_dir == READ && !list_empty(&async[data_dir])) {
				sd->reads_starved++;
			}
			return rq_entry_fifo(sync[data_dir].next);
		}
	}
	if (!list_empty(&async[data_dir])) {
		if (data_dir == READ) {
			sd->reads_starved = 0;
		}
		return rq_entry_fifo(async[data_dir].next);
	}

	if (!list_empty(&sync[!data_dir]))
		return rq_entry_fifo(sync[!data_dir].next);
	if (!list_empty(&async[!data_dir]))
		return rq_entry_fifo(async[!data_dir].next);

	return NULL;
}

static inline void
sioplus_dispatch_request(struct sioplus_data *sd, struct request *rq)
{
	/*
	 * Remove the request from the fifo list
	 * and dispatch it.
	 */
	rq_fifo_clear(rq);
	elv_dispatch_add_tail(rq->q, rq);

	sd->batched++;

	if (rq_data_dir(rq)) {
		sd->starved = 0;
	} else {
		if (!list_empty(&sd->fifo_list[SYNC][WRITE]) || 
				!list_empty(&sd->fifo_list[ASYNC][WRITE]))
			sd->starved++;
	}
}

static int
sioplus_dispatch_requests(struct request_queue *q, int force)
{
	struct sioplus_data *sd = q->elevator->elevator_data;
	struct request *rq = NULL;
	int data_dir = READ;

	/*
	 * Retrieve any expired request after a batch of
	 * sequential requests.
	 */
	if (sd->batched > sd->fifo_batch) {
		sd->batched = 0;
		rq = sioplus_choose_expired_request(sd);
	}

	/* Retrieve request */
	if (!rq) {
		if (sd->starved > sd->writes_starved)
			data_dir = WRITE;

		rq = sioplus_choose_request(sd, data_dir);
		if (!rq)
			return 0;
	}

	/* Dispatch request */
	sioplus_dispatch_request(sd, rq);

	return 1;
}

static struct request *
sioplus_former_request(struct request_queue *q, struct request *rq)
{
	struct sioplus_data *sd = q->elevator->elevator_data;
	const int sync = rq_is_sync(rq);
	const int data_dir = rq_data_dir(rq);

	if (rq->queuelist.prev == &sd->fifo_list[sync][data_dir])
		return NULL;

	/* Return former request */
	return list_entry(rq->queuelist.prev, struct request, queuelist);
}

static struct request *
sioplus_latter_request(struct request_queue *q, struct request *rq)
{
	struct sioplus_data *sd = q->elevator->elevator_data;
	const int sync = rq_is_sync(rq);
	const int data_dir = rq_data_dir(rq);

	if (rq->queuelist.next == &sd->fifo_list[sync][data_dir])
		return NULL;

	/* Return latter request */
	return list_entry(rq->queuelist.next, struct request, queuelist);
}

static void *
sioplus_init_queue(struct request_queue *q)
{
	struct sioplus_data *sd;

	/* Allocate structure */
	sd = kmalloc_node(sizeof(*sd), GFP_KERNEL, q->node);
	if (!sd)
		return NULL;

	/* Initialize fifo lists */
	INIT_LIST_HEAD(&sd->fifo_list[SYNC][READ]);
	INIT_LIST_HEAD(&sd->fifo_list[SYNC][WRITE]);
	INIT_LIST_HEAD(&sd->fifo_list[ASYNC][READ]);
	INIT_LIST_HEAD(&sd->fifo_list[ASYNC][WRITE]);

	/* Initialize data */
	sd->batched = 0;
	sd->starved = 0;
	sd->reads_starved = 0;
	sd->fifo_expire[SYNC][READ] = sync_read_expire;
	sd->fifo_expire[SYNC][WRITE] = sync_write_expire;
	sd->fifo_expire[ASYNC][READ] = async_read_expire;
	sd->fifo_expire[ASYNC][WRITE] = async_write_expire;
	sd->fifo_batch = fifo_batch;
	sd->writes_starved = writes_starved;
	sd->async_reads_starved = async_reads_starved;

	return sd;
}

static void
sioplus_exit_queue(struct elevator_queue *e)
{
	struct sioplus_data *sd = e->elevator_data;

	BUG_ON(!list_empty(&sd->fifo_list[SYNC][READ]));
	BUG_ON(!list_empty(&sd->fifo_list[SYNC][WRITE]));
	BUG_ON(!list_empty(&sd->fifo_list[ASYNC][READ]));
	BUG_ON(!list_empty(&sd->fifo_list[ASYNC][WRITE]));

	/* Free structure */
	kfree(sd);
}

/*
 * sysfs code
 */

static ssize_t
sioplus_var_show(int var, char *page)
{
	return sprintf(page, "%d\n", var);
}

static ssize_t
sioplus_var_store(int *var, const char *page, size_t count)
{
	char *p = (char *) page;

	*var = simple_strtol(p, &p, 10);
	return count;
}

#define SHOW_FUNCTION(__FUNC, __VAR, __CONV)				\
static ssize_t __FUNC(struct elevator_queue *e, char *page)		\
{									\
	struct sioplus_data *sd = e->elevator_data;			\
	int __data = __VAR;						\
	if (__CONV)							\
		__data = jiffies_to_msecs(__data);			\
	return sioplus_var_show(__data, (page));			\
}
SHOW_FUNCTION(sioplus_sync_read_expire_show, sd->fifo_expire[SYNC][READ], 1);
SHOW_FUNCTION(sioplus_sync_write_expire_show, sd->fifo_expire[SYNC][WRITE], 1);
SHOW_FUNCTION(sioplus_async_read_expire_show, sd->fifo_expire[ASYNC][READ], 1);
SHOW_FUNCTION(sioplus_async_write_expire_show, sd->fifo_expire[ASYNC][WRITE], 1);
SHOW_FUNCTION(sioplus_fifo_batch_show, sd->fifo_batch, 0);
SHOW_FUNCTION(sioplus_writes_starved_show, sd->writes_starved, 0);
SHOW_FUNCTION(sioplus_async_reads_starved_show, sd->async_reads_starved, 0);
#undef SHOW_FUNCTION

#define STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, __CONV)			\
static ssize_t __FUNC(struct elevator_queue *e, const char *page, size_t count)	\
{									\
	struct sioplus_data *sd = e->elevator_data;			\
	int __data;							\
	int ret = sioplus_var_store(&__data, (page), count);		\
	if (__data < (MIN))						\
		__data = (MIN);						\
	else if (__data > (MAX))					\
		__data = (MAX);						\
	if (__CONV)							\
		*(__PTR) = msecs_to_jiffies(__data);			\
	else								\
		*(__PTR) = __data;					\
	return ret;							\
}
STORE_FUNCTION(sioplus_sync_read_expire_store, &sd->fifo_expire[SYNC][READ], 0, INT_MAX, 1);
STORE_FUNCTION(sioplus_sync_write_expire_store, &sd->fifo_expire[SYNC][WRITE], 0, INT_MAX, 1);
STORE_FUNCTION(sioplus_async_read_expire_store, &sd->fifo_expire[ASYNC][READ], 0, INT_MAX, 1);
STORE_FUNCTION(sioplus_async_write_expire_store, &sd->fifo_expire[ASYNC][WRITE], 0, INT_MAX, 1);
STORE_FUNCTION(sioplus_fifo_batch_store, &sd->fifo_batch, 0, INT_MAX, 0);
STORE_FUNCTION(sioplus_writes_starved_store, &sd->writes_starved, 0, INT_MAX, 0);
STORE_FUNCTION(sioplus_async_reads_starved_store, &sd->async_reads_starved, 0, INT_MAX, 0);
#undef STORE_FUNCTION

#define DD_ATTR(name) \
	__ATTR(name, S_IRUGO|S_IWUSR, sioplus_##name##_show, \
				      sioplus_##name##_store)

static struct elv_fs_entry sioplus_attrs[] = {
	DD_ATTR(sync_read_expire),
	DD_ATTR(sync_write_expire),
	DD_ATTR(async_read_expire),
	DD_ATTR(async_write_expire),
	DD_ATTR(fifo_batch),
	DD_ATTR(writes_starved),
	DD_ATTR(async_reads_starved),
	__ATTR_NULL
};

static struct elevator_type iosched_sioplus = {
	.ops = {
		.elevator_merge_req_fn		= sioplus_merged_requests,
		.elevator_dispatch_fn		= sioplus_dispatch_requests,
		.elevator_add_req_fn		= sioplus_add_request,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,38)
		.elevator_queue_empty_fn	= sio_queue_empty,
#endif
		.elevator_former_req_fn		= sioplus_former_request,
		.elevator_latter_req_fn		= sioplus_latter_request,
		.elevator_init_fn		= sioplus_init_queue,
		.elevator_exit_fn		= sioplus_exit_queue,
	},

	.elevator_attrs = sioplus_attrs,
	.elevator_name = "sioplus",
	.elevator_owner = THIS_MODULE,
};

static int __init sioplus_init(void)
{
	/* Register elevator */
	elv_register(&iosched_sioplus); 

	return 0;
}

static void __exit sioplus_exit(void)
{
	/* Unregister elevator */
	elv_unregister(&iosched_sioplus);
}

module_init(sioplus_init);
module_exit(sioplus_exit);

MODULE_AUTHOR("Miguel Boton");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Simple IO scheduler plus");

