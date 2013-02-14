#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "qmi_core.h"

void *s_qmictl_new_getcid(u8 tid, u8 svctype, size_t *size)
{
	struct getcid_req *req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return NULL;
	req->req = 0x00;
	req->tid = tid;
	req->msgid = 0x0022;
	req->tlvsize = 0x0004;
	req->service = 0x01;
	req->size = 0x0001;
	req->qmisvc = svctype;
	*size = sizeof(*req);
	return req;
}

void *s_qmictl_new_releasecid(u8 tid, u16 cid, size_t *size)
{
	struct releasecid_req *req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return NULL;
	req->req = 0x00;
	req->tid = tid;
	req->msgid = 0x0023;
	req->tlvsize = 0x05;
	req->rlscid = 0x01;
	req->size = 0x0002;
	req->cid = cid;
	*size = sizeof(*req);
	return req;
}

void *s_qmictl_new_ready(u8 tid, size_t *size)
{
	struct ready_req *req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return NULL;
	req->req = 0x00;
	req->tid = tid;
	req->msgid = 0x21;
	req->tlvsize = 0;
	*size = sizeof(*req);
	return req;
}

void *s_qmiwds_new_seteventreport(u8 tid, size_t *size)
{
	struct seteventreport_req *req = kmalloc(sizeof(*req), GFP_KERNEL);
	req->req = 0x00;
	req->tid = tid;
	req->msgid = 0x0001;
	req->tlvsize = 0x0008;
	req->reportchanrate = 0x11;
	req->size = 0x0005;
	req->period = 0x01;
	req->mask = 0x000000ff;
	*size = sizeof(*req);
	return req;
}

void *s_qmiwds_new_getpkgsrvcstatus(u8 tid, size_t *size)
{
	struct getpkgsrvcstatus_req *req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return NULL;
	req->req = 0x00;
	req->tid = tid;
	req->msgid = 0x22;
	req->tlvsize = 0x0000;
	*size = sizeof(*req);
	return req;
}

void *s_qmidms_new_getmeid(u8 tid, size_t *size)
{
	struct getmeid_req *req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return NULL;
	req->req = 0x00;
	req->tid = tid;
	req->msgid = 0x25;
	req->tlvsize = 0x0000;
	*size = sizeof(*req);
	return req;
}

int s_qmux_parse(u16 *cid, void *buf, size_t size)
{
	struct qmux *qmux = buf;

	if (!buf || size < 12)
		return -ENOMEM;

	if (qmux->tf != 1 || qmux->len != size - 1 || qmux->ctrl != 0x80)
		return -EINVAL;

	*cid = (qmux->qmicid << 8) + qmux->service;
	return sizeof(*qmux);
}

u16 tlv_get(void *msg, u16 msgsize, u8 type, void *buf, u16 bufsize)
{
	u16 pos;
	u16 msize = 0;

	if (!msg || !buf)
		return -ENOMEM;

	for (pos = 4;  pos + 3 < msgsize; pos += msize + 3) {
		msize = *(u16 *)(msg + pos + 1);
		if (*(u8 *)(msg + pos) == type) {
			if (bufsize < msize)
				return -ENOMEM;

			memcpy(buf, msg + pos + 3, msize);
			return msize;
		}
	}

	return -ENOMSG;
}

int s_qmi_msgisvalid(void *msg, u16 size)
{
	char tlv[4];

	if (tlv_get(msg, size, 2, &tlv[0], 4) == 4) {
		if (*(u16 *)&tlv[0] != 0)
			return *(u16 *)&tlv[2];
		else
			return 0;
	}
	return -ENOMSG;
}

int s_qmi_msgid(void *msg, u16 size)
{
	return size < 2 ? -ENODATA : *(u16 *)msg;
}

int s_qmictl_alloccid_resp(void *buf, u16 size, u16 *cid)
{
	int result;
	u8 offset = sizeof(struct qmux) + 2;

	if (!buf || size < offset)
		return -ENOMEM;

	buf = buf + offset;
	size -= offset;

	result = s_qmi_msgid(buf, size);
	if (result != 0x22)
		return -EFAULT;

	result = s_qmi_msgisvalid(buf, size);
	if (result != 0)
		return -EFAULT;

	result = tlv_get(buf, size, 0x01, cid, 2);
	if (result != 2)
		return -EFAULT;

	return 0;
}

int s_qmictl_freecid_resp(void *buf, u16 size)
{
	int result;
	u8 offset = sizeof(struct qmux) + 2;

	if (!buf || size < offset)
		return -ENOMEM;

	buf = buf + offset;
	size -= offset;

	result = s_qmi_msgid(buf, size);
	if (result != 0x23)
		return -EFAULT;

	result = s_qmi_msgisvalid(buf, size);
	if (result != 0)
		return -EFAULT;

	return 0;
}

int s_qmiwds_event_resp(void *buf, u16 size, struct qmiwds_stats *stats)
{
	int result;
	u8 status[2];

	u8 offset = sizeof(struct qmux) + 3;

	if (!buf || size < offset || !stats)
		return -ENOMEM;

	buf = buf + offset;
	size -= offset;

	result = s_qmi_msgid(buf, size);
	if (result == 0x01) {
		tlv_get(buf, size, 0x10, &stats->txok, 4);
		tlv_get(buf, size, 0x11, &stats->rxok, 4);
		tlv_get(buf, size, 0x12, &stats->txerr, 4);
		tlv_get(buf, size, 0x13, &stats->rxerr, 4);
		tlv_get(buf, size, 0x14, &stats->txofl, 4);
		tlv_get(buf, size, 0x15, &stats->rxofl, 4);
		tlv_get(buf, size, 0x19, &stats->txbytesok, 8);
		tlv_get(buf, size, 0x1A, &stats->rxbytesok, 8);
	} else if (result == 0x22) {
		result = tlv_get(buf, size, 0x01, &status[0], 2);
		if (result >= 1)
			stats->linkstate = status[0] == 0x02;
		if (result == 2)
			stats->reconfigure = status[1] == 0x01;

		if (result < 0)
			return result;
	} else {
		return -EFAULT;
	}

	return 0;
}

int s_qmidms_meid_resp(void *buf,  u16 size, char *meid, int meidsize)
{
	int result;

	u8 offset = sizeof(struct qmux) + 3;

	if (!buf || size < offset || meidsize < 14)
		return -ENOMEM;

	buf = buf + offset;
	size -= offset;

	result = s_qmi_msgid(buf, size);
	if (result != 0x25)
		return -EFAULT;

	result = s_qmi_msgisvalid(buf, size);
	if (result)
		return -EFAULT;

	result = tlv_get(buf, size, 0x12, meid, 14);
	if (result != 14)
		return -EFAULT;

	return 0;
}

int s_qmux_fill(u16 cid, void *buf, size_t size)
{
	struct qmux *qmux = buf;

	if (!buf || size < sizeof(*qmux))
		return -ENOMEM;

	qmux->tf = 1;
	qmux->len = size - 1;
	qmux->ctrl = 0x00;
	qmux->service = cid & 0xff;
	qmux->qmicid = cid >> 8;
	return 0;
}

