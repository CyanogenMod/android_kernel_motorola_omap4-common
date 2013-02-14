/*
 * Copyright (C) 2010-2011 Motorola, Inc.
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

#ifndef __BMP180_H__
#define __BMP180_H__

#define BMP180_NAME "bmp180"

#ifdef __KERNEL__
struct bmp180_platform_data {
	int poll_interval;
	int temperature_delay;
	int pressure_delay;
	int min_interval;
	int max_p;
	int min_p;
	int fuzz;
	int flat;
	char regulator_name[10];
};
#endif /* __KERNEL__ */

#endif  /* __BMP180_H__ */

