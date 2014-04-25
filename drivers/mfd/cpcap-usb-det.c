/*
 * Copyright (C) 2007 - 2011 Motorola, Inc.
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
 *
 * Brief overview about the switches used
 * ======================================
 *
 * wsdev (Whisper switch)
 *	DOCKED      : PPD(UART) dock is detected.
 *	UNDOCKED    : SMART dock is detected.
 *                    No dock is connected.
 *
 * dsdev (Standard Dock switch)
 *      NO_DOCK	    : No dock is connected.
 *	DESK_DOCK   : DESK (PPD) dock is detected.
 *	CAR_DOCK    : CAR (PPD) dock is detected.
 *	LE_DOCK     : Low end dock is detected.
 *	HE_DOCK     : High end dock is detected.
 *
 * edsdev (Motorola Dock switch)
 *      NO_DOCK	    : No dock is connected.
 *	DESK_DOCK   : DESK (PPD) dock is detected.
 *	CAR_DOCK    : CAR (PPD) dock is detected.
 *	LE_DOCK     : Low end dock is detected.
 *	HE_DOCK     : High end dock is detected.
 *	MOBILE_DOCK : LAP/MOBILE(SMART) dock is detected.
 *
 * asdev (Audio switch)
 *	NO_DEVICE   : Audio cable not present.
 *	EMU_OUT     : Audio cable detected on a PPD dock.
 *	SPDIF_AUDIO_OUT : Audio cable detected on a SMART dock.
 *
 * sdsdev (Smart Dock switch) - Used only by Dock Status App
 *	DOCKED      : SMART dock is detected.
 *	UNDOCKED    : No dock is connected.
 *
 * csdev (charge capability switch) - Used to indicate the result of
 *                                        whisper authentication.
 *      CHARGE_NONE : Not available to check charger capability
 *      CHARGE_LOW  : Authentication failed for whisper devices.
 *      CHARGE_HIGH : Authentication passed for whisper devices.
 * noauthdev (No authentication switch) - Used to indicate if
 *                                        authentication is needed
 *      AUTH_REQUIRED     : Need to authenticate
 *      AUTH_NOT_REQUIRED : Do not need to authenticate, already done
 */

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include <linux/regulator/consumer.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>

#define CPCAP_SENSE4_LS		8
#define CPCAP_BIT_DP_S_LS	(CPCAP_BIT_DP_S << CPCAP_SENSE4_LS)
#define CPCAP_BIT_DM_S_LS	(CPCAP_BIT_DM_S << CPCAP_SENSE4_LS)
#define GPIO_BIT_SWST1          0x0020

#define SENSE_USB           (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_FACTORY       (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_FACTORY_MASK   (~(CPCAP_BIT_DP_S_LS | \
			      CPCAP_BIT_CHRGCURR1_S))

#define SENSE_CHARGER_FLOAT (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S       | \
			     CPCAP_BIT_DM_S_LS     | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_CHARGER       (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S       | \
			     CPCAP_BIT_DM_S_LS     | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_WHISPER_SPD    (CPCAP_BIT_CHRGCURR1_S | \
			      CPCAP_BIT_SESSVLD_S)

#define SENSE_CHARGER_IND   (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_DP_S_LS     | \
			     GPIO_BIT_SWST1)

#define SENSE_CHARGER_MASK  (CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_WHISPER_PPD   (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_WHISPER_PPD_MASK   (CPCAP_BIT_ID_FLOAT_S | \
				  CPCAP_BIT_SESSVLD_S)

#define SENSE_WHISPER_CABLE (CPCAP_BIT_CHRGCURR1_S)

#define SENSE_WHISPER_SMART (CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_WHISPER_LD2   (CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SE1_S       | \
			     CPCAP_BIT_DM_S_LS     | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_USB_ADAPTER      (CPCAP_BIT_ID_GROUND_S)
#define SENSE_USB_ADAPTER_MASK      (~(CPCAP_BIT_CHRGCURR1_S | \
				CPCAP_BIT_SE1_S | \
				CPCAP_BIT_DP_S_LS))

#define UNDETECT_TRIES		5
#define ADC_AUDIO_THRES     0x12C
#define ADC_ID_FLOAT_THRES  0x3D4
#define DELAY_200MS         200
#define DELAY_5MS           5

#define CPCAP_USB_DET_PRINT_STATUS (1U << 0)
#define CPCAP_USB_DET_PRINT_TRANSITION (1U << 1)
#ifdef CONFIG_CPCAP_USB_DET_LOGGING
static int cpcap_usb_det_debug_mask = 3;
#else
static int cpcap_usb_det_debug_mask;
#endif

static DEFINE_MUTEX(vusb_reg_lock);

module_param_named(cpcap_usb_det_debug_mask, cpcap_usb_det_debug_mask, int,
		   S_IRUGO | S_IWUSR | S_IWGRP);

#define pr_cpcap_usb_det(debug_level_mask, args...) \
	do { \
		if (cpcap_usb_det_debug_mask & \
		    CPCAP_USB_DET_PRINT_##debug_level_mask) { \
			pr_info(args); \
		} \
	} while (0)

enum cpcap_det_state {
	CONFIG,
	SAMPLE_1,
	SAMPLE_2,
	IDENTIFY,
	IDENTIFY_WHISPER_SEMU,
	IDENTIFY_WHISPER_SMART,
	USB,
	FACTORY,
	CHARGER,
	WHISPER_PPD,
	WHISPER_SMART,
	CHARGER_INDUCTIVE,
	USB_ADAPTER,
	WHISPER_PPD_UNKNOWN,
	WHISPER_SMART_LD2,
	WHISPER_SMART_AUTH_FAILED,
	NONE,
};

static const char *irq_names[23] = {"NA", "NA", "NA", "NA", "NA",
	"NA", "NA", "NA", "NA", "NA", "NA",
	"CPCAP_IRQ_VBUSOV",
	"CPCAP_IRQ_RVRS_CHRG",
	"CPCAP_IRQ_CHRG_DET",
	"CPCAP_IRQ_IDFLOAT",
	"CPCAP_IRQ_IDGND",
	"CPCAP_IRQ_SE1",
	"CPCAP_IRQ_SESSEND",
	"CPCAP_IRQ_SESSVLD",
	"CPCAP_IRQ_VBUSVLD",
	"CPCAP_IRQ_CHRG_CURR1",
	"CPCAP_IRQ_CHRG_CURR2",
	"CPCAP_IRQ_RVRS_MODE"};

enum {
	NO_DOCK,
	DESK_DOCK,
	CAR_DOCK,
	LE_DOCK,
	HE_DOCK,
	MOBILE_DOCK,
	WHISPER_CHARGER,
};

enum {
	NO_DEVICE,
	EMU_OUT,
	SPDIF_AUDIO_OUT,
};

enum {
	AUTH_NOT_STARTED,
	AUTH_IN_PROGRESS,
	AUTH_FAILED,
	AUTH_PASSED,
};

enum {
	UNDOCKED,
	DOCKED,
};

enum {
	AUTH_REQUIRED,
	AUTH_NOT_REQUIRED
};

static const char *accy_names[8] = {
	"USB",
	"FACTORY",
	"CHARGER",
	"WHISPER PPD",
	"WHISPER SMART",
	"USB DEVICE",
	"NONE",
	"UNKNOWN",
};

struct cpcap_usb_det_data {
	struct cpcap_device *cpcap;
	struct delayed_work work;
	struct workqueue_struct *wq;
	unsigned short sense;
	unsigned short prev_sense;
	enum cpcap_det_state state;
	enum cpcap_accy usb_accy;
	struct platform_device *usb_dev;
	struct platform_device *usb_connected_dev;
	struct platform_device *charger_connected_dev;
	struct regulator *regulator_vusb;
	struct regulator *regulator_vhpd;
	struct wake_lock wake_lock;
	unsigned char is_vusb_enabled;
	unsigned char is_vhpd_enabled;
	unsigned char undetect_cnt;
	char bpass_mod;
	struct cpcap_batt_ac_data ac;
	struct switch_dev wsdev; /* Whisper switch */
	struct switch_dev dsdev; /* Standard Dock switch */
	struct switch_dev asdev; /* Audio switch */
	struct switch_dev edsdev; /* Motorola Dock switch */
	struct switch_dev sdsdev; /* Smart Dock Switch */
	struct switch_dev csdev;  /*charge capablity switch */
	struct switch_dev noauthdev; /* If authentication is needed */
	char dock_id[CPCAP_WHISPER_ID_SIZE];
	char dock_prop[CPCAP_WHISPER_PROP_SIZE];
	unsigned char audio;
	short whisper_auth;
	struct cpcap_charger_request charger_data;
	enum cpcap_irqs irq;
	enum cpcap_support_status usb_drv_ctrl_via_ulpi;
	struct cpcap_usb_mux *usb_mux;
	unsigned char power_up;
};

static const char *accy_devices[] = {
	"cpcap_usb_charger",
	"cpcap_factory",
	"cpcap_charger",
	"cpcap_whisper_ppd",
	"cpcap_whisper_smart",
};

#ifdef CONFIG_USB_TESTING_POWER
static int testing_power_enable = -1;
module_param(testing_power_enable, int, 0644);
MODULE_PARM_DESC(testing_power_enable, "Enable factory cable power "
	"supply function for testing");
#endif

static void vusb_enable(struct cpcap_usb_det_data *data)
{
	mutex_lock(&vusb_reg_lock);
	if (!data->is_vusb_enabled) {
		wake_lock(&data->wake_lock);
		pr_cpcap_usb_det(STATUS, "usb wakelock taken\n");
		regulator_enable(data->regulator_vusb);
		data->is_vusb_enabled = 1;
	}
	mutex_unlock(&vusb_reg_lock);
}

static void vusb_disable(struct cpcap_usb_det_data *data)
{
	mutex_lock(&vusb_reg_lock);
	if (data->is_vusb_enabled) {
		wake_unlock(&data->wake_lock);
		pr_cpcap_usb_det(STATUS, "usb wakelock released\n");
		regulator_disable(data->regulator_vusb);
		data->is_vusb_enabled = 0;
	}
	mutex_unlock(&vusb_reg_lock);
}

static void vhpd_enable(struct cpcap_usb_det_data *data)
{
	if (!data->is_vhpd_enabled) {
		regulator_enable(data->regulator_vhpd);
		pr_cpcap_usb_det(STATUS, "hpd gpio enabled\n");
		data->is_vhpd_enabled = 1;
	}
}

static void vhpd_disable(struct cpcap_usb_det_data *data)
{
	if (data->is_vhpd_enabled) {
		regulator_disable(data->regulator_vhpd);
		pr_cpcap_usb_det(STATUS, "hpd gpio disabled\n");
		data->is_vhpd_enabled = 0;
	}
}

static ssize_t dock_print_name(struct switch_dev *switch_dev, char *buf)
{
	switch (switch_get_state(switch_dev)) {
	case NO_DOCK:
		return sprintf(buf, "None\n");
	case DESK_DOCK:
		return sprintf(buf, "DESK\n");
	case CAR_DOCK:
		return sprintf(buf, "CAR\n");
	case LE_DOCK:
		return sprintf(buf, "LE\n");
	case HE_DOCK:
		return sprintf(buf, "HE\n");
	case MOBILE_DOCK:
		return sprintf(buf, "MOBILE\n");
	}

	return -EINVAL;
}
static ssize_t dock_id_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct switch_dev *dsdev = dev_get_drvdata(dev);
	struct cpcap_usb_det_data *data =
		container_of(dsdev, struct cpcap_usb_det_data, dsdev);

	return snprintf(buf, PAGE_SIZE, "%s\n", data->dock_id);
}
static DEVICE_ATTR(dock_addr, S_IRUGO, dock_id_show, NULL);

static ssize_t dock_prop_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct switch_dev *dsdev = dev_get_drvdata(dev);
	struct cpcap_usb_det_data *data =
		container_of(dsdev, struct cpcap_usb_det_data, dsdev);

	return snprintf(buf, PAGE_SIZE, "%s\n", data->dock_prop);
}
static DEVICE_ATTR(dock_prop, S_IRUGO, dock_prop_show, NULL);

static ssize_t charger_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct switch_dev *csdev = dev_get_drvdata(dev);
	struct cpcap_usb_det_data *data =
		container_of(csdev, struct cpcap_usb_det_data, csdev);

	return snprintf(buf, PAGE_SIZE,
		"BATT_VLTG=%4dmV\nCHRG_CAP=%dmA\nWHISPER=%1d\n",
		data->charger_data.batt_vltg,
		data->charger_data.charger_capacity,
		data->charger_data.whisper_dock);
}
static DEVICE_ATTR(charger_data, S_IRUGO, charger_data_show, NULL);

static ssize_t emu_audio_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case EMU_OUT:
		return sprintf(buf, "Stereo out\n");
	case SPDIF_AUDIO_OUT:
		return sprintf(buf, "SPDIF audio out\n");
	}

	return -EINVAL;
}

void whisper_toggle_audio_switch_for_spdif(struct cpcap_device *cpcap,
					   int state)
{
	struct cpcap_usb_det_data *data = cpcap->accydata;
	struct cpcap_platform_data *pdata;

	if (!data)
		return;

	pdata = cpcap->spi->dev.platform_data;

	pdata->spdif_audio->enable_spdif_audio(state);
	state = ((state > 0) ? SPDIF_AUDIO_OUT : NO_DEVICE);
	switch_set_state(&data->asdev, state);

	pr_info("%s: Audio cable %s present\n", __func__,
		(state ? "is" : "not"));
}

static int get_sense(struct cpcap_usb_det_data *data)
{
	int retval = -EFAULT;
	unsigned short value;
	struct cpcap_device *cpcap;
	struct cpcap_platform_data *pdata;

	if (!data)
		return -EFAULT;
	cpcap = data->cpcap;
	pdata = cpcap->spi->dev.platform_data;

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS1, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT1,
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_FLOAT_I |
				      CPCAP_BIT_ID_GROUND_I),
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_FLOAT_I |
				      CPCAP_BIT_ID_GROUND_I));
	if (retval)
		return retval;

	data->sense = value & (CPCAP_BIT_ID_FLOAT_S |
			       CPCAP_BIT_ID_GROUND_S);

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS2, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT2,
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_VBUSVLD_I |
				     CPCAP_BIT_SESSVLD_I |
				     CPCAP_BIT_SE1_I),
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_VBUSVLD_I |
				     CPCAP_BIT_SESSVLD_I |
				     CPCAP_BIT_SE1_I));
	if (retval)
		return retval;

	data->sense |= value & (CPCAP_BIT_CHRGCURR1_S |
				CPCAP_BIT_SESSVLD_S |
				CPCAP_BIT_SE1_S);

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS4, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT4,
				     (CPCAP_BIT_DP_I |
				      CPCAP_BIT_DM_I),
				     (CPCAP_BIT_DP_I |
				      CPCAP_BIT_DM_I));
	if (retval)
		return retval;

	data->sense |= (value & (CPCAP_BIT_DP_S |
			       CPCAP_BIT_DM_S)) << CPCAP_SENSE4_LS;

	if (pdata->ind_chrg->check_inductive_path != NULL)
		if (pdata->ind_chrg->check_inductive_path() == 0)
			data->sense |= GPIO_BIT_SWST1;

	pr_cpcap_usb_det(STATUS, "CPCAP USB DET Sense = 0x%4X", data->sense);

	return 0;
}

static int configure_hardware(struct cpcap_usb_det_data *data,
			      enum cpcap_accy accy)
{
	int retval = -EFAULT;
	struct cpcap_device *cpcap;
	struct cpcap_platform_data *pdata;
	int count;

	cpcap = data->cpcap;
	pdata = cpcap->spi->dev.platform_data;

	if (data->usb_drv_ctrl_via_ulpi == STATUS_SUPPORTED)
		/* Take control of pull up from ULPI. */
		retval  = cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
				     CPCAP_BIT_PU_SPI,
				     CPCAP_BIT_PU_SPI);
	retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
				    CPCAP_BIT_DP150KPU,
				    (CPCAP_BIT_DP150KPU | CPCAP_BIT_DP1K5PU |
				     CPCAP_BIT_DM1K5PU | CPCAP_BIT_DPPD |
				     CPCAP_BIT_DMPD));

	pr_cpcap_usb_det(STATUS,  "configure_hardware: accy=%s\n",
		accy_names[accy]);

	switch (accy) {
	case CPCAP_ACCY_USB:
	case CPCAP_ACCY_FACTORY:
		pdata->usb_mux->configure_switch_muxmode(OTG_DM_DP);
		pdata->usb_mux->configure_otg_muxmode(USB_OTG);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBXCVREN,
					     CPCAP_BIT_USBXCVREN);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, 0,
					     CPCAP_BIT_VBUSSTBY_EN);

		/* Give USB driver control of pull up via ULPI. */
		if (data->usb_drv_ctrl_via_ulpi == STATUS_SUPPORTED)
			retval |= cpcap_regacc_write(data->cpcap,
						     CPCAP_REG_USBC3,
						     0,
						     CPCAP_BIT_PU_SPI |
						     CPCAP_BIT_DMPD_SPI |
						     CPCAP_BIT_DPPD_SPI |
						     CPCAP_BIT_SUSPEND_SPI |
						     CPCAP_BIT_ULPI_SPI_SEL);

		if ((data->cpcap->vendor == CPCAP_VENDOR_ST) &&
			(data->cpcap->revision == CPCAP_REVISION_2_0))
				vusb_enable(data);

		break;

	case CPCAP_ACCY_CHARGER:
		pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
		pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
		if (pdata->ind_chrg->force_cable_path != NULL)
			pdata->ind_chrg->force_cable_path(0);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     0, CPCAP_BIT_RVRSMODE);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_MODE);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, 0,
					     CPCAP_BIT_VBUSSTBY_EN);
		break;

	case CPCAP_ACCY_WHISPER_PPD:
		pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
		pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
		if (pdata->ind_chrg->force_cable_path != NULL)
			pdata->ind_chrg->force_cable_path(1);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     CPCAP_BIT_RVRSMODE,
					     CPCAP_BIT_RVRSMODE);
		for (count = 0; count < 50; count++) {
			cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
			cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_MODE);
			retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     CPCAP_BIT_RVRSMODE,
					     CPCAP_BIT_RVRSMODE);
		}
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     CPCAP_BIT_VBUSSTBY_EN,
					     CPCAP_BIT_VBUSSTBY_EN);

		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_MODE);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_IDGND);
		break;

	case CPCAP_ACCY_WHISPER_SMART:
	case CPCAP_ACCY_USB_DEVICE:
		pdata->usb_mux->configure_switch_muxmode(OTG_DM_DP);
		pdata->usb_mux->configure_otg_muxmode(USB_OTG);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBXCVREN,
					     CPCAP_BIT_USBXCVREN);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, 0,
					     CPCAP_BIT_VBUSSTBY_EN);
		/* Give USB driver control of pull up via ULPI. */
		if (data->usb_drv_ctrl_via_ulpi == STATUS_SUPPORTED) {
			retval |= cpcap_regacc_write(data->cpcap,
						     CPCAP_REG_USBC3,
						     0,
						     CPCAP_BIT_PU_SPI |
						     CPCAP_BIT_DMPD_SPI |
						     CPCAP_BIT_DPPD_SPI |
						     CPCAP_BIT_SUSPEND_SPI |
						     CPCAP_BIT_ULPI_SPI_SEL);
			retval |= cpcap_regacc_write(data->cpcap,
						     CPCAP_REG_USBC2,
						     CPCAP_BIT_USBXCVREN,
						     CPCAP_BIT_USBXCVREN);
		}
		break;

	case CPCAP_ACCY_UNKNOWN:
		pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
		pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
		if (pdata->ind_chrg->force_cable_path != NULL)
			pdata->ind_chrg->force_cable_path(0);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD |
					     CPCAP_BIT_ID100KPU);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM, 0,
					     CPCAP_BIT_RVRSMODE);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_MODE);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2, 0,
					     CPCAP_BIT_EMUMODE2 |
					     CPCAP_BIT_EMUMODE1 |
					     CPCAP_BIT_EMUMODE0);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     CPCAP_BIT_VBUSSTBY_EN,
					     CPCAP_BIT_VBUSSTBY_EN);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC, 0,
					     CPCAP_BIT_VBUS_SWITCH);
		data->whisper_auth = AUTH_NOT_STARTED;
		break;

	case CPCAP_ACCY_NONE:
	default:
		pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
		pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
		if (pdata->ind_chrg->force_cable_path != NULL)
			pdata->ind_chrg->force_cable_path(0);
		if (pdata->ind_chrg->force_inductive_path != NULL)
			pdata->ind_chrg->force_inductive_path(0);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM, 0,
					     CPCAP_BIT_RVRSMODE);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
		cpcap_irq_clear(data->cpcap, CPCAP_IRQ_RVRS_MODE);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC, 0,
					     CPCAP_BIT_VBUS_SWITCH);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBSUSPEND,
					     CPCAP_BIT_USBXCVREN |
					     CPCAP_BIT_USBSUSPEND);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     CPCAP_BIT_VBUSSTBY_EN,
					     CPCAP_BIT_VBUSSTBY_EN);
		if (data->usb_drv_ctrl_via_ulpi == STATUS_SUPPORTED)
			retval |= cpcap_regacc_write(data->cpcap,
						     CPCAP_REG_USBC3,
						     CPCAP_BIT_DMPD_SPI |
						     CPCAP_BIT_DPPD_SPI |
						     CPCAP_BIT_SUSPEND_SPI |
						     CPCAP_BIT_ULPI_SPI_SEL,
						     CPCAP_BIT_DMPD_SPI |
						     CPCAP_BIT_DPPD_SPI |
						     CPCAP_BIT_SUSPEND_SPI |
						     CPCAP_BIT_ULPI_SPI_SEL);
		data->whisper_auth = AUTH_NOT_STARTED;
		break;
	}

	if (retval != 0)
		retval = -EFAULT;

	return retval;
}

static unsigned char vbus_valid_adc_check(struct cpcap_usb_det_data *data)
{
	struct cpcap_adc_request req;
	int ret;

	req.format = CPCAP_ADC_FORMAT_CONVERTED;
	req.timing = CPCAP_ADC_TIMING_IMM;
	req.type = CPCAP_ADC_TYPE_BANK_0;

	ret = cpcap_adc_sync_read(data->cpcap, &req);
	if (ret) {
		dev_err(&data->cpcap->spi->dev,
		 "%s: ADC Read failed\n", __func__);
		return false;
	}
	return ((req.result[CPCAP_ADC_CHG_ISENSE] < 50) &&
		(req.result[CPCAP_ADC_VBUS] <
		(req.result[CPCAP_ADC_BATTP]+100))) ? false : true;
}

static bool id_adc_check(struct cpcap_usb_det_data *data, int *adc_result,
			 int delay)
{
	bool is_success = false;
	int ret;
	struct cpcap_adc_request req;
	unsigned short value;

	cpcap_regacc_read(data->cpcap, CPCAP_REG_USBC1, &value);
	value &= CPCAP_BIT_ID100KPU;

	cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, CPCAP_BIT_IDPUCNTRL,
			   (CPCAP_BIT_ID100KPU | CPCAP_BIT_IDPUCNTRL));

	msleep(delay);

	req.format = CPCAP_ADC_FORMAT_RAW;
	req.timing = CPCAP_ADC_TIMING_IMM;
	req.type = CPCAP_ADC_TYPE_BANK_0;

	ret = cpcap_adc_sync_read(data->cpcap, &req);
	if (ret) {
		dev_err(&data->cpcap->spi->dev,
			 "%s: ID ADC read failed\n", __func__);
		*adc_result = -1;
	} else {
		is_success = true;
		*adc_result = req.result[CPCAP_ADC_USB_ID];
		dev_info(&data->cpcap->spi->dev,
			"%s: ID ADC value = %d\n", __func__, *adc_result);
	}

	cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, value,
			   (CPCAP_BIT_ID100KPU | CPCAP_BIT_IDPUCNTRL));

	return is_success;
}

static void whisper_audio_check(struct cpcap_usb_det_data *data)
{
	int adc_value;

	if (!switch_get_state(&data->dsdev))
		return;

	if (id_adc_check(data, &adc_value, DELAY_200MS) == true) {
		data->audio = (adc_value > ADC_AUDIO_THRES)
			? EMU_OUT : NO_DEVICE;
		if (data->audio) {
			cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					   CPCAP_BIT_ID100KPU,
					   CPCAP_BIT_ID100KPU);
			cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
				(CPCAP_BIT_EMUMODE2 | CPCAP_BIT_EMUMODE0),
				(CPCAP_BIT_EMUMODE2 | CPCAP_BIT_EMUMODE1 |
				CPCAP_BIT_EMUMODE0));
		}

		switch_set_state(&data->asdev, data->audio);

		dev_info(&data->cpcap->spi->dev,
			 "%s: Audio cable %s present\n", __func__,
			 (data->audio ? "is" : "not"));
	}

	cpcap_irq_clear(data->cpcap, CPCAP_IRQ_IDGND);
}

static bool whisper_id_float_check(struct cpcap_usb_det_data *data)
{
	bool is_float = false;
	int adc_value;

	if (id_adc_check(data, &adc_value, DELAY_5MS) == true)
		is_float = (adc_value > ADC_ID_FLOAT_THRES) ? true : false;

	return is_float;
}

static void notify_accy(struct cpcap_usb_det_data *data, enum cpcap_accy accy)
{
	dev_info(&data->cpcap->spi->dev, "notify_accy: accy=%s\n",
		accy_names[accy]);

	if ((data->usb_accy != CPCAP_ACCY_NONE) && (data->usb_dev != NULL)) {
		if (!((data->usb_accy == CPCAP_ACCY_USB_DEVICE) &&
		    (accy == CPCAP_ACCY_CHARGER))) {
			platform_device_del(data->usb_dev);
			data->usb_dev = NULL;
		}
	}

	configure_hardware(data, accy);
	data->usb_accy = accy;

	if (accy != CPCAP_ACCY_NONE) {
		if (!data->usb_dev) {
			if (accy == CPCAP_ACCY_USB_DEVICE) {
				if (data->ac.model == CPCAP_BATT_AC_SMARTDOCK)
					data->usb_dev = platform_device_alloc(
					    accy_devices[CPCAP_ACCY_CHARGER],
					    -1);
			}
			else
				data->usb_dev = platform_device_alloc(
					accy_devices[accy], -1);
			if (data->usb_dev) {
				data->usb_dev->dev.platform_data = data->cpcap;
				platform_set_drvdata(data->usb_dev, &data->ac);
				platform_device_add(data->usb_dev);
			}
		}
	}

	if ((accy == CPCAP_ACCY_USB) || (accy == CPCAP_ACCY_FACTORY) ||
	    (accy == CPCAP_ACCY_USB_DEVICE)) {
		if (!data->usb_connected_dev) {
			data->usb_connected_dev =
			    platform_device_alloc("cpcap_usb_connected", -1);
			if (data->usb_connected_dev) {
				platform_device_add_data(
					data->usb_connected_dev,
					&data->usb_accy,
					sizeof(data->usb_accy));
				platform_device_add(data->usb_connected_dev);
			}
		}
	} else if (data->usb_connected_dev) {
		platform_device_del(data->usb_connected_dev);
		data->usb_connected_dev = NULL;
	}

	if ((accy == CPCAP_ACCY_CHARGER) ||
	    ((accy == CPCAP_ACCY_USB_DEVICE) &&
	     (data->ac.model == CPCAP_BATT_AC_SMARTDOCK))) {
		if (!data->charger_connected_dev) {
			data->charger_connected_dev =
			    platform_device_alloc("cpcap_charger_connected",
						  -1);
			if (data->charger_connected_dev)
				platform_device_add(
					data->charger_connected_dev);
		}
	} else if (data->charger_connected_dev) {
		platform_device_del(data->charger_connected_dev);
		data->charger_connected_dev = NULL;
	}

	if (accy == CPCAP_ACCY_NONE)
		vusb_disable(data);
}

static void notify_whisper_switch(struct cpcap_usb_det_data *data,
				enum cpcap_accy accy)
{
	struct cpcap_device *cpcap;
	struct cpcap_platform_data *pdata;

	cpcap = data->cpcap;
	pdata = cpcap->spi->dev.platform_data;

	pr_cpcap_usb_det(STATUS, "notify_whisper_switch: accy=%s\n",
		accy_names[accy]);

	if ((accy == CPCAP_ACCY_CHARGER) ||
		(accy == CPCAP_ACCY_WHISPER_PPD)) {
		switch_set_state(&data->wsdev, DOCKED);
		if (data->whisper_auth == AUTH_PASSED) {
			if (accy == CPCAP_ACCY_CHARGER)
				switch_set_state(&data->csdev, CHARGE_HIGH);
			else
				switch_set_state(&data->csdev, CHARGE_NONE);
		} else if (data->whisper_auth == AUTH_FAILED) {
			if (accy == CPCAP_ACCY_CHARGER)
				switch_set_state(&data->csdev, CHARGE_LOW);
			else
				switch_set_state(&data->csdev, CHARGE_NONE);
		}
	/* LD2 open->close */
	} else if ((accy == CPCAP_ACCY_WHISPER_SMART) &&
		(data->state == WHISPER_SMART_LD2)) {
		if (data->whisper_auth != AUTH_PASSED) {
			pr_cpcap_usb_det(STATUS, "n_w_s: Auth Not PASSED\n");
			switch_set_state(&data->noauthdev, AUTH_REQUIRED);
			switch_set_state(&data->wsdev, UNDOCKED);
			switch_set_state(&data->dsdev, NO_DOCK);
			switch_set_state(&data->edsdev, NO_DOCK);
			switch_set_state(&data->sdsdev, DOCKED);
			switch_set_state(&data->asdev, NO_DEVICE);
			switch_set_state(&data->csdev, CHARGE_NONE);
		} else {
			pr_cpcap_usb_det(STATUS, "n_w_s: Auth PASSED\n");
			switch_set_state(&data->noauthdev, AUTH_NOT_REQUIRED);
			switch_set_state(&data->dsdev, NO_DOCK);
			switch_set_state(&data->asdev, NO_DEVICE);
			switch_set_state(&data->csdev, CHARGE_NONE);
			vhpd_enable(data);
			msleep(100);
			if (pdata->hpd_status->check_hpd_status) {
				if (pdata->hpd_status->check_hpd_status()) {
					switch_set_state(&data->wsdev,
							 UNDOCKED);
					switch_set_state(&data->sdsdev, DOCKED);
					switch_set_state(&data->edsdev,
							 MOBILE_DOCK);
				} else {
					switch_set_state(&data->sdsdev,
							 UNDOCKED);
					switch_set_state(&data->wsdev, DOCKED);
					switch_set_state(&data->edsdev,
							 NO_DOCK);
				}
			}
			vhpd_disable(data);
		}
	} else {
		switch_set_state(&data->wsdev, UNDOCKED);
		switch_set_state(&data->dsdev, NO_DOCK);
		switch_set_state(&data->edsdev, NO_DOCK);
		switch_set_state(&data->asdev, NO_DEVICE);
		switch_set_state(&data->csdev, CHARGE_NONE);
		if (accy == CPCAP_ACCY_WHISPER_SMART)
			switch_set_state(&data->sdsdev, DOCKED);
		else {
			switch_set_state(&data->noauthdev, AUTH_REQUIRED);
			if ((accy == CPCAP_ACCY_NONE) &&
			    (data->state == WHISPER_SMART_AUTH_FAILED))
				switch_set_state(&data->sdsdev, DOCKED);
			else
				switch_set_state(&data->sdsdev, UNDOCKED);
			memset(data->dock_id, 0, CPCAP_WHISPER_ID_SIZE);
			memset(data->dock_prop, 0, CPCAP_WHISPER_PROP_SIZE);
		}
	}
}

static void detection_work(struct work_struct *work)
{
	struct cpcap_usb_det_data *data =
		container_of(work, struct cpcap_usb_det_data, work.work);
	unsigned char isVBusValid = 0;
	struct cpcap_device *cpcap;
	struct cpcap_platform_data *pdata;
	unsigned short value;
	int retval = -EFAULT;

	cpcap = data->cpcap;
	pdata = cpcap->spi->dev.platform_data;

	switch (data->state) {
	case CONFIG:
		pr_cpcap_usb_det(STATUS, "detection_work: CONFIG\n");
		vusb_enable(data);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDFLOAT);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DPI);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DMI);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SESSVLD);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_RVRS_MODE);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_RVRS_CHRG);

		configure_hardware(data, CPCAP_ACCY_UNKNOWN);
		switch_set_state(&data->noauthdev, AUTH_REQUIRED);
		data->undetect_cnt = 0;
		data->state = SAMPLE_1;
		queue_delayed_work(data->wq, &data->work,
					msecs_to_jiffies(11));
		break;

	case SAMPLE_1:
		pr_cpcap_usb_det(STATUS, "detection_work: SAMPLE1\n");
		get_sense(data);
		data->state = SAMPLE_2;
		queue_delayed_work(data->wq, &data->work,
					msecs_to_jiffies(10));
		break;

	case SAMPLE_2:
		pr_cpcap_usb_det(STATUS, "detection_work: SAMPLE2\n");
		data->prev_sense = data->sense;
		get_sense(data);

		if (data->prev_sense != data->sense) {
			/* Stay in this state */
			data->state = SAMPLE_2;
			queue_delayed_work(data->wq, &data->work,
					      msecs_to_jiffies(10));
		} else if (!(data->sense & CPCAP_BIT_SE1_S) &&
			   (data->sense & CPCAP_BIT_ID_FLOAT_S) &&
			   !(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			   !(data->sense & CPCAP_BIT_SESSVLD_S)) {
			data->state = IDENTIFY;
			queue_delayed_work(data->wq, &data->work,
					      msecs_to_jiffies(10));
		} else {
			data->state = IDENTIFY;
			queue_delayed_work(data->wq, &data->work, 0);
		}
		break;

	case IDENTIFY:
		get_sense(data);
		data->state = CONFIG;

		if (data->sense == SENSE_USB) {
			pr_cpcap_usb_det(STATUS,
				"detection_work: USB Identified\n");
			notify_accy(data, CPCAP_ACCY_USB);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

			/* Special handling of USB cable undetect. */
			data->state = USB;
		} else if ((data->sense & SENSE_FACTORY_MASK) ==
			   SENSE_FACTORY) {
			pr_cpcap_usb_det(STATUS,
				"detection_work: FACTORY Identified\n");

			if (!(data->power_up) &&
			    (pdata->factory_kill->force_factory_kill))
				pdata->factory_kill->force_factory_kill(1);

#ifdef CONFIG_USB_TESTING_POWER
			if (testing_power_enable > 0) {
				notify_accy(data, CPCAP_ACCY_NONE);
				cpcap_irq_unmask(data->cpcap,
					CPCAP_IRQ_CHRG_DET);
				cpcap_irq_unmask(data->cpcap,
					CPCAP_IRQ_CHRG_CURR1);
				cpcap_irq_unmask(data->cpcap,
				CPCAP_IRQ_VBUSVLD);
				break;
			}
#endif
			notify_accy(data, CPCAP_ACCY_FACTORY);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			if (data->power_up)
				cpcap_irq_unmask(data->cpcap,
						 CPCAP_IRQ_IDGND);
			/* Special handling of factory cable undetect. */
			data->state = FACTORY;
		} else if ((data->sense == SENSE_CHARGER_FLOAT) ||
			   (data->sense == SENSE_CHARGER) ||
			   (data->sense == SENSE_WHISPER_SPD) ||
			   (data->sense == SENSE_WHISPER_PPD) ||
			   (data->sense == SENSE_WHISPER_CABLE)) {
				pr_cpcap_usb_det(STATUS,
					"detection_work: IW Identified\n");
				data->state = IDENTIFY_WHISPER_SEMU;
				queue_delayed_work(data->wq, &data->work, 0);
		} else if ((data->sense == SENSE_WHISPER_SMART) ||
			   (data->sense == SENSE_WHISPER_LD2)) {
				pr_cpcap_usb_det(STATUS,
					"detection_work: IW Identified\n");
				data->state = IDENTIFY_WHISPER_SMART;
				queue_delayed_work(data->wq, &data->work, 0);
		} else if (data->sense == SENSE_CHARGER_IND) {
			pr_cpcap_usb_det(STATUS,
				"detection_work: INDUCTIVE CHRG Identified\n");
			data->ac.model = CPCAP_BATT_AC_IND;
			notify_accy(data, CPCAP_ACCY_CHARGER);
			data->state = CHARGER_INDUCTIVE;
			if (pdata->ind_chrg->force_inductive_path != NULL)
				pdata->ind_chrg->force_inductive_path(1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DPI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DMI);
		} else if (pdata->usbhs_ext_pwr->supported &&
			((data->sense & SENSE_USB_ADAPTER_MASK) ==
			SENSE_USB_ADAPTER) && !vbus_valid_adc_check(data)) {
			data->state = USB_ADAPTER;
			queue_delayed_work(data->wq, &data->work,
						msecs_to_jiffies(200));
		} else if ((vbus_valid_adc_check(data)) &&
				(data->usb_accy == CPCAP_ACCY_NONE)) {
			pr_cpcap_usb_det(STATUS,
				"detection_work: PARTIAL INSERTION\n");
			data->state = CONFIG;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DPI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DMI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		} else {
			pr_cpcap_usb_det(STATUS,
				"detection_work: No Accessory Connected\n");
			data->state = NONE;
			queue_delayed_work(data->wq, &data->work, 0);
		}
		break;

	case USB:
		pr_cpcap_usb_det(STATUS, "detection_work: USB\n");
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		if ((data->sense & CPCAP_BIT_SE1_S) ||
			(data->sense & CPCAP_BIT_ID_GROUND_S) ||
			(!isVBusValid)) {
				data->state = CONFIG;
				queue_delayed_work(data->wq, &data->work, 0);
		} else {
			data->state = USB;
			data->undetect_cnt = 0;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case FACTORY:
		pr_cpcap_usb_det(STATUS, "detection_work: FACTORY\n");
		get_sense(data);
		msleep(100);
		isVBusValid = vbus_valid_adc_check(data);

		/* The removal of a factory cable can only be detected if a
		 * charger is attached.
		 */
		if (data->sense & CPCAP_BIT_SE1_S) {
#ifdef CONFIG_TTA_CHARGER
			enable_tta();
#endif
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			if ((isVBusValid) && (!(data->power_up)) &&
			    (pdata->factory_kill->force_factory_kill)) {
				pdata->factory_kill->force_factory_kill(1);
			} else if ((!isVBusValid) && (data->power_up)) {
				data->power_up = 0;
			}
			data->state = FACTORY;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case IDENTIFY_WHISPER_SEMU:
		data->state = CONFIG;

		if ((data->sense == SENSE_CHARGER_FLOAT) ||
		    (data->sense == SENSE_CHARGER) ||
		    (data->sense == SENSE_WHISPER_SPD) ||
		    ((data->sense & SENSE_CHARGER_MASK) ==
		     SENSE_CHARGER_MASK)) {
			pr_cpcap_usb_det(STATUS,
				"detection_work: CHARGER Identified\n");
			data->ac.model = CPCAP_BATT_AC_CABLE;
			notify_accy(data, CPCAP_ACCY_CHARGER);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			data->state = CHARGER;
			notify_whisper_switch(data, CPCAP_ACCY_CHARGER);
		} else if ((data->sense == SENSE_WHISPER_PPD) ||
			   !(data->sense & SENSE_WHISPER_PPD_MASK) ||
			   (data->sense == SENSE_WHISPER_CABLE)) {
			if (whisper_id_float_check(data)) {
				pr_cpcap_usb_det(STATUS,
					"detection_work: Not really "
					"PPD device\n");
				data->state = NONE;
				queue_delayed_work(data->wq, &data->work, 0);
			} else {
				pr_cpcap_usb_det(STATUS,
					"detection_work: PPD Identified\n");
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
				notify_accy(data, CPCAP_ACCY_WHISPER_PPD);
				cpcap_irq_unmask(data->cpcap,
						CPCAP_IRQ_IDFLOAT);
				cpcap_irq_unmask(data->cpcap,
						CPCAP_IRQ_IDGND);
				cpcap_irq_unmask(data->cpcap,
						CPCAP_IRQ_RVRS_MODE);
				cpcap_irq_unmask(data->cpcap,
						CPCAP_IRQ_RVRS_CHRG);
				data->state = WHISPER_PPD;
				notify_whisper_switch(data,
						CPCAP_ACCY_WHISPER_PPD);
			}
		} else {
			queue_delayed_work(data->wq, &data->work, 0);
		}
		break;

	case IDENTIFY_WHISPER_SMART:
		pr_cpcap_usb_det(STATUS,
			"detection_work: SMART Identified\n");
		data->ac.model = CPCAP_BATT_AC_SMARTDOCK;
		notify_accy(data, CPCAP_ACCY_USB_DEVICE);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
		if (data->sense == SENSE_WHISPER_SMART) {
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			data->state = WHISPER_SMART;
		} else
			data->state = WHISPER_SMART_LD2;
		notify_whisper_switch(data, CPCAP_ACCY_WHISPER_SMART);
		break;

	case CHARGER:
		pr_cpcap_usb_det(STATUS, "detection_work: CHARGER\n");
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		if (!(data->sense & CPCAP_BIT_SESSVLD_S) &&
		    !(data->sense & CPCAP_BIT_ID_FLOAT_S) &&
		    (data->whisper_auth == AUTH_PASSED)) {
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			data->state = IDENTIFY_WHISPER_SEMU;
			queue_delayed_work(data->wq, &data->work, 0);
		} else if ((!(data->sense & CPCAP_BIT_SESSVLD_S)) ||
			(!isVBusValid)) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			if ((data->irq == CPCAP_IRQ_IDGND) &&
			    (data->whisper_auth == AUTH_PASSED)) {
				whisper_audio_check(data);
			}
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
		}
		break;

	case WHISPER_PPD:
		pr_cpcap_usb_det(STATUS, "detection_work: PPD\n");
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		if ((data->irq == CPCAP_IRQ_RVRS_MODE) ||
			(data->irq == CPCAP_IRQ_RVRS_CHRG)) {
			if ((data->whisper_auth == AUTH_NOT_STARTED) ||
				(data->whisper_auth == AUTH_FAILED)) {
				pr_cpcap_usb_det(STATUS,
					"detection_work: Set None\n");
				notify_accy(data, CPCAP_ACCY_NONE);
				notify_whisper_switch(data, CPCAP_ACCY_NONE);
			} else if (data->whisper_auth == AUTH_IN_PROGRESS) {
				pr_cpcap_usb_det(STATUS,
					"detection_work: Delay\n");
				msleep(5000);
				if (data->whisper_auth != AUTH_PASSED) {
					notify_accy(data, CPCAP_ACCY_NONE);
					notify_whisper_switch(data,
						CPCAP_ACCY_NONE);
				}
			}
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_RVRS_MODE);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
			if (data->whisper_auth == AUTH_PASSED) {
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDFLOAT);
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
				data->state = IDENTIFY_WHISPER_SEMU;
				queue_delayed_work(data->wq, &data->work, 0);
			} else {
				cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
				cpcap_irq_unmask(data->cpcap,
					CPCAP_IRQ_CHRG_DET);
				cpcap_irq_unmask(data->cpcap,
					CPCAP_IRQ_IDFLOAT);
				data->state = WHISPER_PPD_UNKNOWN;
			}
		} else if (data->sense & CPCAP_BIT_ID_FLOAT_S) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else if ((isVBusValid) &&
			   (data->sense & CPCAP_BIT_SESSVLD_S)) {
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
			data->state = IDENTIFY_WHISPER_SEMU;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			if (data->whisper_auth == AUTH_FAILED &&
			    data->usb_accy == CPCAP_ACCY_WHISPER_PPD) {
				notify_accy(data, CPCAP_ACCY_NONE);
			}

			if ((data->irq == CPCAP_IRQ_IDGND) &&
			    (data->whisper_auth == AUTH_PASSED)) {
				whisper_audio_check(data);
			}
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_RVRS_MODE);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
		}
		break;

	case WHISPER_SMART:
		pr_cpcap_usb_det(STATUS, "detection_work: SMART\n");
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		if (data->whisper_auth == AUTH_FAILED &&
		    data->usb_accy == CPCAP_ACCY_USB_DEVICE &&
		    isVBusValid) {
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			notify_accy(data, CPCAP_ACCY_NONE);
			data->state = WHISPER_SMART_AUTH_FAILED;
			notify_whisper_switch(data, CPCAP_ACCY_NONE);
			notify_accy(data, CPCAP_ACCY_CHARGER);
			break;
		}

		if ((!(data->sense & CPCAP_BIT_SESSVLD_S)) ||
			(!isVBusValid)) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			data->state = WHISPER_SMART;

			/* HD Dock:ID Pin should be released for SPDIF.
			   Need to mask the IDGND Interrupt so that
			   driver shouldn't wake-up everytime audio
			   data is transferred over SPDIF
			   MOBILE_Dock: Audio data is not transferred
			   over SPDIF. Inturn we use the ID_FLOAT to
			   run the detection code during LID transitions */
			switch (switch_get_state(&data->edsdev)) {
			case MOBILE_DOCK:
				cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
				cpcap_irq_unmask(data->cpcap,
						 CPCAP_IRQ_IDFLOAT);
				break;
			case HE_DOCK:
			default:
				if (!(data->sense & CPCAP_BIT_ID_GROUND_S) &&
				    isVBusValid) {
					cpcap_irq_mask(data->cpcap,
						       CPCAP_IRQ_IDGND);
					cpcap_irq_mask(data->cpcap,
						       CPCAP_IRQ_IDFLOAT);
				}
				break;
			}
		}
		break;

	case USB_ADAPTER:
		get_sense(data);
		if ((data->sense & SENSE_USB_ADAPTER_MASK) ==
			SENSE_USB_ADAPTER) {
			notify_accy(data, CPCAP_ACCY_USB_DEVICE);
			pdata->usbhs_ext_pwr->usbhs_ext_pwr_en(1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		} else {
			data->state = CONFIG;
			pdata->usbhs_ext_pwr->usbhs_ext_pwr_en(0);
			if ((data->sense & SENSE_USB_ADAPTER_MASK) !=
				(CPCAP_BIT_ID_FLOAT_S | CPCAP_BIT_SESSVLD_S))
				pr_cpcap_usb_det(STATUS,
				"Not supported adapter or device\n");
			queue_delayed_work(data->wq, &data->work, 0);
		}
		break;

	case WHISPER_SMART_LD2:
		pr_cpcap_usb_det(STATUS, "detection_work: SMART_LD2\n");
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		if ((switch_get_state(&data->edsdev) != MOBILE_DOCK) &&
		    (switch_get_state(&data->sdsdev) == DOCKED) &&
		    (isVBusValid) && (data->whisper_auth == AUTH_PASSED)) {
			pr_cpcap_usb_det(STATUS,
				"SMART_LD2: Its a SMART dock with SE1\n");
			data->state = WHISPER_SMART;
			queue_delayed_work(data->wq, &data->work, 0);
			break;
		}

		if (data->whisper_auth == AUTH_FAILED &&
		    data->usb_accy == CPCAP_ACCY_USB_DEVICE &&
		    isVBusValid) {
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			notify_accy(data, CPCAP_ACCY_NONE);
			data->state = WHISPER_SMART_AUTH_FAILED;
			notify_whisper_switch(data, CPCAP_ACCY_NONE);
			notify_accy(data, CPCAP_ACCY_CHARGER);
			break;
		}

		if ((!(data->sense & CPCAP_BIT_SESSVLD_S)) ||
			(!isVBusValid)) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			data->state = WHISPER_SMART_LD2;
			if (!(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			    data->irq == CPCAP_IRQ_IDGND &&
			    data->whisper_auth == AUTH_PASSED) {
				notify_whisper_switch(data,
					CPCAP_ACCY_WHISPER_SMART);
			}
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case CHARGER_INDUCTIVE:
		pr_cpcap_usb_det(STATUS, "detection_work: INDUCTIVE\n");
		get_sense(data);
		if ((data->sense & CPCAP_BIT_ID_GROUND_S) ||
		    !(data->sense & CPCAP_BIT_ID_FLOAT_S) ||
		    !(data->sense & CPCAP_BIT_DP_S_LS) ||
		    (data->sense & CPCAP_BIT_SE1_S)) {
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DPI);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DMI);
			if (pdata->ind_chrg->force_charge_complete != NULL)
				pdata->ind_chrg->force_charge_complete(1);
			data->state = CONFIG;
		} else if (!(data->sense & CPCAP_BIT_SESSVLD_S)) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DPI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DMI);
			data->state = CHARGER_INDUCTIVE;
		}

		break;

	case WHISPER_PPD_UNKNOWN:
		pr_cpcap_usb_det(STATUS, "detection_work: PPD_UNKNOWN\n");
		get_sense(data);
		retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS1, &value);
		if (retval)
			value = 0;

		if ((data->irq == CPCAP_IRQ_IDGND) ||
		    (data->irq == CPCAP_IRQ_IDFLOAT) ||
		    (value & CPCAP_BIT_CHRG_DET_S)) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}

		break;

	case WHISPER_SMART_AUTH_FAILED:
		pr_cpcap_usb_det(STATUS, "detection_work: SMART_AUTH_FAILED\n");
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		if ((!(data->sense & CPCAP_BIT_SESSVLD_S)) ||
			(!isVBusValid)) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			data->state = WHISPER_SMART_AUTH_FAILED;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case NONE:
		pr_cpcap_usb_det(STATUS, "detection_work: NONE\n");
		data->state = CONFIG;
		data->ac.model = CPCAP_BATT_AC_NONE;
		notify_accy(data, CPCAP_ACCY_NONE);
		notify_whisper_switch(data, CPCAP_ACCY_NONE);
		cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
		cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);

		/* When a charger is unpowered by unplugging from the
		 * wall, VBUS voltage will drop below CHRG_DET (3.5V)
		 * until the ICHRG bits are cleared.  Once ICHRG is
		 * cleared, VBUS will rise above CHRG_DET, but below
		 * VBUSVLD (4.4V) briefly as it decays.  If the charger
		 * is re-powered while VBUS is within this window, the
		 * VBUSVLD interrupt is needed to trigger charger
		 * detection.
		 *
		 * VBUSVLD must be masked before going into suspend.
		 * See cpcap_usb_det_suspend() for details.
		 */
		cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);
		cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
		cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		if (data->power_up)
			data->power_up = 0;
		break;

	default:
		/* This shouldn't happen.  Need to reset state machine. */
		vusb_disable(data);
		data->state = CONFIG;
		queue_delayed_work(data->wq, &data->work, 0);
		break;
	}
}

static void int_handler(enum cpcap_irqs int_event, void *data)
{
	struct cpcap_usb_det_data *usb_det_data = data;
	usb_det_data->irq = int_event;
	if ((int_event >= 0 && int_event <= 22))
		pr_cpcap_usb_det(STATUS,
			"cpcap_usb_det: irq=%s\n", irq_names[int_event]);
	else
		pr_cpcap_usb_det(STATUS,
			"cpcap_usb_det: irq=%d\n", int_event);
	queue_delayed_work(usb_det_data->wq, &(usb_det_data->work), 0);
}

int cpcap_accy_whisper(struct cpcap_device *cpcap,
		       struct cpcap_whisper_request *req)
{
	struct cpcap_usb_det_data *data = cpcap->accydata;
	struct cpcap_platform_data *pdata;

	int retval = -EAGAIN;
	int dock = NO_DOCK;
	unsigned short value = 0;

	if (!data)
		return -ENODEV;

	pdata = cpcap->spi->dev.platform_data;

	pr_cpcap_usb_det(STATUS, "%s: ioctl cmd = 0x%04x\n",
		__func__, req->cmd);

	pr_cpcap_usb_det(STATUS, "%s: data->state = %d\n",
		__func__, data->state);

	if ((data->state == CHARGER) ||
		(data->state == WHISPER_PPD)) {
		if (req->cmd & CPCAP_WHISPER_ENABLE_UART) {
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
			data->whisper_auth = AUTH_IN_PROGRESS;
			pdata->usb_mux->configure_switch_muxmode(OTG_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(UART_2);
		}

		retval = cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
			value, CPCAP_BIT_EMUMODE2 | CPCAP_BIT_EMUMODE1 |
			CPCAP_BIT_EMUMODE0);

		if (req->cmd & CPCAP_WHISPER_MODE_PU) {
			data->whisper_auth = AUTH_PASSED;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);
		} else if (!(req->cmd & CPCAP_WHISPER_ENABLE_UART)) {
			data->whisper_auth = AUTH_FAILED;
		}

		/* Report dock type to system. */
		dock = (req->cmd & CPCAP_WHISPER_ACCY_MASK) >>
			CPCAP_WHISPER_ACCY_SHFT;
		if (dock != WHISPER_CHARGER) {
			switch_set_state(&data->dsdev, dock);
			switch_set_state(&data->edsdev, dock);
		}
		pr_cpcap_usb_det(STATUS, "%s: Whisper_auth =%d\n", __func__,
			(data->whisper_auth));

		if (!(req->cmd & CPCAP_WHISPER_ENABLE_UART) ==
			STATUS_SUPPORTED) {
			pdata->usb_mux->configure_switch_muxmode(CPCAP_DM_DP);
			pdata->usb_mux->configure_otg_muxmode(SAFE_MODE);
			if (dock && (strlen(req->dock_id) <
				     CPCAP_WHISPER_ID_SIZE))
				strncpy(data->dock_id, req->dock_id,
					CPCAP_WHISPER_ID_SIZE);
			if (dock && (strlen(req->dock_prop) <
				     CPCAP_WHISPER_PROP_SIZE))
				strncpy(data->dock_prop, req->dock_prop,
					CPCAP_WHISPER_PROP_SIZE);
			whisper_audio_check(data);
		}
		if (data->state != WHISPER_PPD) {
			if (req->cmd & CPCAP_WHISPER_MODE_PU)
				switch_set_state(&data->csdev, CHARGE_HIGH);
			else if ((!(req->cmd & CPCAP_WHISPER_MODE_PU)) &&
				 (!(req->cmd & CPCAP_WHISPER_ENABLE_UART)))
				switch_set_state(&data->csdev, CHARGE_LOW);
		}
	} else if ( (data->state == WHISPER_SMART) ||
		(data->state == WHISPER_SMART_LD2)) {
		/* Report dock type to system. */
		dock = (req->cmd & CPCAP_WHISPER_ACCY_MASK) >>
			CPCAP_WHISPER_ACCY_SHFT;
		if (dock && (strlen(req->dock_id) < CPCAP_WHISPER_ID_SIZE))
			strncpy(data->dock_id, req->dock_id,
				CPCAP_WHISPER_ID_SIZE);
		if (dock && (strlen(req->dock_prop) < CPCAP_WHISPER_PROP_SIZE))
			strncpy(data->dock_prop, req->dock_prop,
				CPCAP_WHISPER_PROP_SIZE);
		switch (dock) {
		case HE_DOCK:
			switch_set_state(&data->dsdev, HE_DOCK);
			switch_set_state(&data->csdev, CHARGE_HIGH);
			break;
		case MOBILE_DOCK:
		default:
			switch_set_state(&data->dsdev, NO_DOCK);
			break;
		}
		switch_set_state(&data->edsdev, dock);

		if (req->cmd & CPCAP_WHISPER_MODE_PU)
			data->whisper_auth = AUTH_PASSED;
		else {
			data->whisper_auth = AUTH_FAILED;
			switch_set_state(&data->csdev, CHARGE_LOW);
		}
		cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		retval = 0;
	}

	return retval;
}

int cpcap_accy_charger(struct cpcap_device *cpcap,
		       struct cpcap_charger_request *req)
{
	struct cpcap_usb_det_data *data = cpcap->accydata;
	struct device *dev = &cpcap->spi->dev;
	int retval = -EAGAIN;
	char vltg_buf[30];
	char capacity_buf[30];
	char whisper_buf[30];

	char *envp[4];
	int env_offset = 0;

	pr_cpcap_usb_det(STATUS, "%s:Sending charge capability uevent\n",
		__func__);

	/* Put battery voltage into uevent string */
	sprintf(vltg_buf, "BATT_VLTG=%4dmV", req->batt_vltg);
	envp[env_offset++] = vltg_buf;

	/* Put charger capacity into uevent string */
	sprintf(capacity_buf, "CHRG_CAP=%dmA", req->charger_capacity);
	envp[env_offset++] = capacity_buf;

	/* Put whisper status into uevent string */
	sprintf(whisper_buf, "WHISPER=%1d", req->whisper_dock);
	envp[env_offset++] = whisper_buf;

	envp[env_offset] = NULL;
	retval = kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);

	/* Store current charger data */
	data->charger_data.batt_vltg = req->batt_vltg;
	data->charger_data.charger_capacity = req->charger_capacity;
	data->charger_data.whisper_dock = req->whisper_dock;

	return retval;
}
static int __init cpcap_usb_det_probe(struct platform_device *pdev)
{
	int retval;
	struct cpcap_usb_det_data *data;
	struct cpcap_platform_data *pdata;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cpcap = pdev->dev.platform_data;
	data->state = CONFIG;
	/* This flag is needed to enable/disable the
	 * Factory Kill switch depending on whether phone
	 * was powered up with a FACTORY cable or not.
	 */
	data->power_up = 1;
	data->wq = create_singlethread_workqueue("cpcap_accy");
	platform_set_drvdata(pdev, data);
	INIT_DELAYED_WORK(&data->work, detection_work);
	data->usb_accy = CPCAP_ACCY_NONE;
	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "usb");
	data->undetect_cnt = 0;
	data->bpass_mod = 'a';
	data->ac.model = CPCAP_BATT_AC_NONE;

	data->wsdev.name = "whisper";
	switch_dev_register(&data->wsdev);

	data->dsdev.name = "dock";
	data->dsdev.print_name = dock_print_name;
	switch_dev_register(&data->dsdev);

	data->asdev.name = "semu_audio";
	data->asdev.print_name = emu_audio_print_name;
	switch_dev_register(&data->asdev);

	data->edsdev.name = "extdock";
	data->edsdev.print_name = dock_print_name;
	switch_dev_register(&data->edsdev);

	data->sdsdev.name = "smartdock";
	switch_dev_register(&data->sdsdev);

	data->csdev.name = "charge_capability";
	switch_dev_register(&data->csdev);

	data->noauthdev.name = "noauth";
	switch_dev_register(&data->noauthdev);

	retval = device_create_file(data->dsdev.dev, &dev_attr_dock_addr);
	if (retval < 0) {
		dev_err(&pdev->dev, "Failed to create dock_addr file\n");
		goto free_mem;
	}
	retval = device_create_file(data->dsdev.dev, &dev_attr_dock_prop);
	if (retval < 0) {
		dev_err(&pdev->dev, "Failed to create dock_prop file\n");
		goto free_dock_addr;
	}
	retval = device_create_file(data->csdev.dev, &dev_attr_charger_data);
	if (retval < 0) {
		dev_err(&pdev->dev, "Failed to create charger_data file\n");
		goto free_dock_prop;
	}

	data->whisper_auth = AUTH_NOT_STARTED;
	data->irq = CPCAP_IRQ__START;

	pdata = data->cpcap->spi->dev.platform_data;
	data->usb_drv_ctrl_via_ulpi = pdata->usb_drv_ctrl_via_ulpi;

	data->regulator_vusb = regulator_get(NULL, "vusb");
	if (IS_ERR(data->regulator_vusb)) {
		dev_err(&pdev->dev, "Could not get regulator for cpcap_usb\n");
		retval = PTR_ERR(data->regulator_vusb);
		goto free_charger_data;
	}
	regulator_set_voltage(data->regulator_vusb, 3300000, 3300000);

	data->regulator_vhpd = regulator_get(NULL, "hdmi_5V_en");
	if (IS_ERR(data->regulator_vhpd)) {
		dev_err(&pdev->dev, "Could not get regulator for hdmi_hpd\n");
		retval = PTR_ERR(data->regulator_vhpd);
		goto free_regulator_vusb;
	}

	retval = cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_DET,
				    int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_CURR1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_SE1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_IDGND,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_VBUSVLD,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_IDFLOAT,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_DPI,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_DMI,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_SESSVLD,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_RVRS_MODE,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_RVRS_CHRG,
				     int_handler, data);

	if (data->usb_drv_ctrl_via_ulpi == STATUS_SUPPORTED)
		/* Now that HW initialization is done,
		   give USB control via ULPI. */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     0, CPCAP_BIT_ULPI_SPI_SEL);

	if (retval != 0) {
		dev_err(&pdev->dev, "Initialization Error\n");
		retval = -ENODEV;
		goto free_irqs;
	}

	if (pdata->usbhs_ext_pwr->init) {
		if (pdata->usbhs_ext_pwr->init(pdata->usbhs_ext_pwr))
			goto free_irqs;
	}
	data->cpcap->accydata = data;
	dev_set_drvdata(&pdev->dev, data);

	dev_info(&pdev->dev, "CPCAP USB detection device probed\n");

	/* Perform initial detection */
	queue_delayed_work(data->wq, &data->work, 0);

	return 0;

free_irqs:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_VBUSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDFLOAT);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DPI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DMI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SESSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_RVRS_MODE);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_RVRS_CHRG);
	regulator_put(data->regulator_vhpd);

free_regulator_vusb:
	regulator_put(data->regulator_vusb);
free_charger_data:
	device_remove_file(data->csdev.dev, &dev_attr_charger_data);
free_dock_prop:
	device_remove_file(data->dsdev.dev, &dev_attr_dock_prop);
free_dock_addr:
	device_remove_file(data->dsdev.dev, &dev_attr_dock_addr);

free_mem:
	switch_dev_unregister(&data->wsdev);
	switch_dev_unregister(&data->dsdev);
	switch_dev_unregister(&data->asdev);
	switch_dev_unregister(&data->edsdev);
	switch_dev_unregister(&data->sdsdev);
	switch_dev_unregister(&data->csdev);
	switch_dev_unregister(&data->noauthdev);

	wake_lock_destroy(&data->wake_lock);
	kfree(data);

	return retval;
}

static int __exit cpcap_usb_det_remove(struct platform_device *pdev)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);
	struct cpcap_platform_data *pdata = data->cpcap->spi->dev.platform_data;

	if (pdata->usbhs_ext_pwr->exit)
		pdata->usbhs_ext_pwr->exit(pdata->usbhs_ext_pwr);

	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_VBUSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDFLOAT);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DPI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DMI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SESSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_RVRS_MODE);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_RVRS_CHRG);


	configure_hardware(data, CPCAP_ACCY_NONE);
	cancel_delayed_work_sync(&data->work);
	destroy_workqueue(data->wq);

	if ((data->usb_accy != CPCAP_ACCY_NONE) && (data->usb_dev != NULL))
		platform_device_del(data->usb_dev);

	switch_dev_unregister(&data->wsdev);
	switch_dev_unregister(&data->dsdev);
	switch_dev_unregister(&data->asdev);
	switch_dev_unregister(&data->edsdev);
	switch_dev_unregister(&data->sdsdev);
	switch_dev_unregister(&data->csdev);

	vusb_disable(data);
	regulator_put(data->regulator_vusb);
	regulator_put(data->regulator_vhpd);

	wake_lock_destroy(&data->wake_lock);

	kfree(data);
	return 0;
}

#ifdef CONFIG_PM
static int cpcap_usb_det_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);

	/* VBUSVLD cannot be unmasked when entering suspend. If left
	 * unmasked, a false interrupt will be received, keeping the
	 * device out of suspend. The interrupt does not need to be
	 * unmasked when resuming from suspend since the use case
	 * for having the interrupt unmasked is over.
	 */
	cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);

	return 0;
}
#else
#define cpcap_usb_det_suspend NULL
#endif

static struct platform_driver cpcap_usb_det_driver = {
	.probe		= cpcap_usb_det_probe,
	.remove		= __exit_p(cpcap_usb_det_remove),
	.suspend	= cpcap_usb_det_suspend,
	.driver		= {
		.name	= "cpcap_usb_det",
		.owner	= THIS_MODULE,
	},
};

static int __init cpcap_usb_det_init(void)
{
	return cpcap_driver_register(&cpcap_usb_det_driver);
}
/* The CPCAP USB detection driver must be started later to give the MUSB
 * driver time to complete its initialization. */
late_initcall(cpcap_usb_det_init);

static void __exit cpcap_usb_det_exit(void)
{
	platform_driver_unregister(&cpcap_usb_det_driver);
}
module_exit(cpcap_usb_det_exit);

MODULE_ALIAS("platform:cpcap_usb_det");
MODULE_DESCRIPTION("CPCAP USB detection driver");
MODULE_LICENSE("GPL");
