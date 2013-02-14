#ifndef QMI_CORE_H
#define QMI_CORE_H

#define QMICTL 0
#define QMIWDS 1
#define QMIDMS 2

struct qmux {
	u8 tf;  /* always 1 */
	u16 len;
	u8 ctrl;
	u8 service;
	u8 qmicid;
} __attribute__((__packed__));

struct getcid_req {
	struct qmux header;
	u8 req;
	u8 tid;
	u16 msgid;
	u16 tlvsize;
	u8 service;
	u16 size;
	u8 qmisvc;
} __attribute__((__packed__));

struct releasecid_req {
	struct qmux header;
	u8 req;
	u8 tid;
	u16 msgid;
	u16 tlvsize;
	u8 rlscid;
	u16 size;
	u16 cid;
} __attribute__((__packed__));

struct ready_req {
	struct qmux header;
	u8 req;
	u8 tid;
	u16 msgid;
	u16 tlvsize;
} __attribute__((__packed__));

struct seteventreport_req {
	struct qmux header;
	u8 req;
	u16 tid;
	u16 msgid;
	u16 tlvsize;
	u8 reportchanrate;
	u16 size;
	u8 period;
	u32 mask;
} __attribute__((__packed__));

struct getpkgsrvcstatus_req {
	struct qmux header;
	u8 req;
	u16 tid;
	u16 msgid;
	u16 tlvsize;
} __attribute__((__packed__));

struct getmeid_req {
	struct qmux header;
	u8 req;
	u16 tid;
	u16 msgid;
	u16 tlvsize;
} __attribute__((__packed__));

struct qmiwds_stats {
	u32 txok;
	u32 rxok;
	u32 txerr;
	u32 rxerr;
	u32 txofl;
	u32 rxofl;
	u64 txbytesok;
	u64 rxbytesok;
	bool linkstate;
	bool reconfigure;
};

#define QMUX_HEADER_SIZE (sizeof(struct qmux))

void *s_qmictl_new_getcid(u8 tid, u8 svctype, size_t *size);
void *s_qmictl_new_releasecid(u8 tid, u16 cid, size_t *size);
void *s_qmictl_new_ready(u8 tid, size_t *size);
void *s_qmiwds_new_seteventreport(u8 tid, size_t *size);
void *s_qmiwds_new_getpkgsrvcstatus(u8 tid, size_t *size);
void *s_qmidms_new_getmeid(u8 tid, size_t *size);
int s_qmux_parse(u16 *cid, void *buf, size_t size);
int s_qmi_msgisvalid(void *msg, u16 size);
int s_qmi_msgid(void *msg, u16 size);
int s_qmictl_alloccid_resp(void *buf, u16 size, u16 *cid);
int s_qmictl_freecid_resp(void *buf, u16 size);
int s_qmiwds_event_resp(void *buf, u16 size, struct qmiwds_stats *stats);
int s_qmidms_meid_resp(void *buf,  u16 size, char *meid, int meidsize);
int s_qmux_fill(u16 cid, void *buf, size_t size);

#endif
