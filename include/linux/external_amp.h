/*
 * Copyright (C) 2011 Motorola, Inc.
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
 */

#ifndef _LINUX_EXTERNAL_AMP_H__
#define _LINUX_EXTERNAL_AMP_H__

#include <linux/ioctl.h>

#define EXTERNAL_AMP_IOCTL_BASE 'E'
#define EXTERNAL_AMP_IOCTL_DISABLE _IO(EXTERNAL_AMP_IOCTL_BASE, 0x01)
#define EXTERNAL_AMP_IOCTL_ENABLE	 _IO(EXTERNAL_AMP_IOCTL_BASE, 0x02)

#endif	/* _LINUX_EXTERNAL_AMP_H__ */
