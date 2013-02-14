/*
     Copyright (C) 2011 Motorola, Inc.

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
#ifndef __LINUX_RADIO_MDM9600_H__
#define __LINUX_RADIO_MDM9600_H__

#define MDM9600_STATUS_PANIC_NAME		"panic"
#define MDM9600_STATUS_PANIC_BUSY_WAIT_NAME	"panic busy wait"
#define MDM9600_STATUS_QC_DLOAD_NAME		"qc dload"
#define MDM9600_STATUS_RAM_DOWNLOADER_NAME	"ram downloader"
#define MDM9600_STATUS_PHONE_CODE_AWAKE_NAME	"awake"
#define MDM9600_STATUS_PHONE_CODE_ASLEEP_NAME	"asleep"
#define MDM9600_STATUS_SHUTDOWN_ACK_NAME	"shutdown ack"
#define MDM9600_STATUS_UNDEFINED_NAME		"undefined"


#ifdef __KERNEL__

#define MDM9600_GPIO_INVALID -1
#define MDM9600_CTRL_MODULE_NAME "mdm9600_ctrl"

enum {
	MDM9600_CTRL_GPIO_STATUS_A0,
	MDM9600_CTRL_GPIO_STATUS_A1,
	MDM9600_CTRL_GPIO_STATUS_B0,
	MDM9600_CTRL_GPIO_STATUS_B1,
	MDM9600_CTRL_GPIO_STATUS_B2,
	MDM9600_CTRL_GPIO_BP_RESOUT,
	MDM9600_CTRL_GPIO_BP_PWRON,
	MDM9600_CTRL_GPIO_AP_RESET_BP,
	MDM9600_CTRL_NUM_GPIOS,
};

enum {
	MDM9600_GPIO_DIRECTION_IN,
	MDM9600_GPIO_DIRECTION_OUT,
};

struct mdm9600_ctrl_gpio {
	unsigned int number;
	unsigned int direction;
	unsigned int default_value;
	unsigned int allocated;
	char *name;
};

struct mdm9600_command_gpios {
	unsigned int cmd1;
	unsigned int cmd2;
};

struct mdm9600_ctrl_platform_data {
	char *name;
	struct mdm9600_ctrl_gpio gpios[MDM9600_CTRL_NUM_GPIOS];
	struct mdm9600_command_gpios cmd_gpios;
};

#endif /* __KERNEL__ */

#endif /* __LINUX_RADIO_MDM9600_H__ */
