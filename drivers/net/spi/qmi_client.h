#ifndef QMI_CLIENT_H
#define QMI_CLIENT_H

struct readreq {
	struct list_head node;
	void *data;
	u16 tid;
	u16 size;
};

struct notifyreq {
	struct list_head node;
	void (*func)(u16, void *);
	u16  tid;
	void *data;
};

/*
 * QMI client can issue a read request by either sync or async,
 * read request will be met only when received data with the same
 * CID and TID.
 *
 * For async read, client will ad it into notifies list and then
 * return, later if matched data received, notify will be called.
 *
 * For sync read, it will act as async read but wait to 2HZ and
 * check if data received, if data received notify will be called
 * to interrupt wait, otherwise will fail.
 *
 * Please note that a QMI device can be opened multiple times and
 * alloc multiple clients. we also do not maintain TID, it is created
 * by uplayer.
 */
struct qmidev;

struct qmi_client {
	/* list node */
	struct list_head node;
	/* client ID */
	u16 cid;
	/* increase by each message */
	atomic_t tid;
	/* sync read request list */
	struct list_head reads;
	/* async read request list */
	struct list_head notifies;
	/* for prectect client lists access,
	 * use different for reads and notifies?
	 */
	spinlock_t lock;
	/* wait queue on read this client */
	wait_queue_head_t read_wait;
	/* client belongs to this dev */
	struct qmidev *dev;
};

struct qmi_clients {
	/* client list header */
	struct list_head header;
	/* for protect clients list */
	spinlock_t lock;
};

struct qmi_handler {
	int cid;
	struct qmidev *dev;
};

void qmi_read_complete(u16 cid, void *data);
struct qmi_client *qmi_client_by_cid(struct qmi_clients *clients, u16 cid);
bool qmi_client_add_read(struct qmi_client *client,
		u16 tid, void *data, u16 size);
bool qmi_client_del_read(struct qmi_client *client,
		u16 tid, void **data, u16 *size);
bool qmi_client_add_notify(struct qmi_client *client,
		u16 tid, void (*hook)(u16 cid, void *), void *data);
bool qmi_client_notify(struct qmi_client *client, u16 tid);

int qmi_read_async(struct qmi_client *client, u16 tid,
		      void (*hook)(u16 cid, void *),
		      void *data);
int qmi_read_sync(struct qmi_client *client, void **buf, u16 tid);
struct qmi_client *qmi_client_create(struct qmidev *dev, u8 type);
void qmi_client_release(struct qmidev *dev, u16 cid);

void qmi_clients_recv_data(struct qmi_clients *client,
		u16 cid, u16 tid, void *data, u16 size);
int qmi_write_sync(struct qmidev *dev, char *buf, int size, u16 cid);

#endif
