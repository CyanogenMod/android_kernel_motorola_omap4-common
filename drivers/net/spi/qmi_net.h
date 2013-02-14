#ifndef QMI_NET_H
#define QMI_NET_H

#include "qmi_dev.h"

struct qcspinet {
	int order;
	int mux_cid;
	struct qmidev qmi;
	struct net_device *net;
	struct mutex mutex; /* */
};

extern int qcspinet_debug;
#endif
