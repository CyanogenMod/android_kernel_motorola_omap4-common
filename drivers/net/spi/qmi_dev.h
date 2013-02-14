#ifndef QCSPI_QMI_H
#define QCSPI_QMI_H

#include "qmi_client.h"

struct qcspinet;

struct qmidev {
	dev_t devnum;
	int mux_cid;
	struct class *devclass;
	struct cdev *cdev;
	struct urbsetup *readsetup;
	struct qmi_clients clients;
	char meid[14];
};

int qcspi_qmi_register(struct qcspinet *dev);
void qcspi_qmi_deregister(struct qcspinet *dev);
struct qcspinet *cdev_to_qcspinet(struct cdev *cdev);

#endif

