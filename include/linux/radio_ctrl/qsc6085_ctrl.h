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
#ifndef __LINUX_QSC6085_CTRL_H__
#define __LINUX_QSC6085_CTRL_H__

#ifdef __KERNEL__
#define QSC6085_CTRL_MODULE_NAME "qsc6085_ctrl"

struct qsc6085_ctrl_platform_data {
	char *name;
	unsigned int gpio_power;
	unsigned int gpio_pshold;
	unsigned int gpio_reset_out;
	unsigned int gpio_flash_enable;
};

#endif
#endif /* __LINUX_MDM_CTRL_H__ */
