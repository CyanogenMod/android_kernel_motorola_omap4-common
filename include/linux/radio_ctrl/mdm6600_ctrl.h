/*
     Copyright (C) 2010 Motorola, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
     02111-1307  USA
*/
#ifndef __LINUX_RADIO_MDM6600_H__
#define __LINUX_RADIO_MDM6600_H__

#define MDM6600_STATUS_PANIC_NAME		"panic"
#define MDM6600_STATUS_PANIC_BUSY_WAIT_NAME	"panic busy wait"
#define MDM6600_STATUS_QC_DLOAD_NAME		"qc dload"
#define MDM6600_STATUS_RAM_DOWNLOADER_NAME	"ram downloader"
#define MDM6600_STATUS_PHONE_CODE_AWAKE_NAME	"awake"
#define MDM6600_STATUS_PHONE_CODE_ASLEEP_NAME	"asleep"
#define MDM6600_STATUS_SHUTDOWN_ACK_NAME	"shutdown ack"
#define MDM6600_STATUS_UNDEFINED_NAME		"undefined"


#ifdef __KERNEL__

#define MDM6600_GPIO_INVALID -1
#define MDM6600_CTRL_MODULE_NAME "mdm6600_ctrl"

extern bool mdm6600_ctrl_bp_is_shutdown;
static inline bool mdm6600_ctrl_is_bp_up(void)
{
	return !mdm6600_ctrl_bp_is_shutdown;
}

enum {
	MDM6600_CTRL_GPIO_AP_STATUS_0,
	MDM6600_CTRL_GPIO_AP_STATUS_1,
	MDM6600_CTRL_GPIO_AP_STATUS_2,
	MDM6600_CTRL_GPIO_BP_STATUS_0,
	MDM6600_CTRL_GPIO_BP_STATUS_1,
	MDM6600_CTRL_GPIO_BP_STATUS_2,
	MDM6600_CTRL_GPIO_BP_RESOUT,
	MDM6600_CTRL_GPIO_BP_RESIN,
	MDM6600_CTRL_GPIO_BP_PWRON,
	MDM6600_CTRL_NUM_GPIOS,
};

enum {
	MDM6600_GPIO_DIRECTION_IN,
	MDM6600_GPIO_DIRECTION_OUT,
};

struct mdm6600_ctrl_gpio {
	unsigned int number;
	unsigned int direction;
	unsigned int default_value;
	unsigned int allocated;
	char *name;
};

struct mdm6600_command_gpios {
	unsigned int cmd1;
	unsigned int cmd2;
};

struct mdm6600_ctrl_platform_data {
	char *name;
	int  bootmode;
	struct mdm6600_ctrl_gpio gpios[MDM6600_CTRL_NUM_GPIOS];
	struct mdm6600_command_gpios cmd_gpios;
	struct platform_device *mapphone_bpwake_device;
};

#endif /* __KERNEL__ */

#endif /* __LINUX_RADIO_MDM6600_H__ */
