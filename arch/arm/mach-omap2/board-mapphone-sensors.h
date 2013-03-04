/*
 * Platform data for mapphone sensors.
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __BOARD_MAPPHONE_SENSORS_H
#define __BOARD_MAPPHONE_SENSORS_H

#include <linux/attiny48mu_cap_prox.h>
#include <linux/ct405.h>
#include <linux/i2c/adp8870.h>
#include <linux/i2c/lm3532.h>
#include <linux/leds-lp8550.h>
#include <linux/input/bmp180.h>
#include <linux/isl29030.h>
#include <linux/isl29032.h>
#include <linux/kxtf9.h>
#include <linux/l3g4200d.h>
#include <linux/lis3dh.h>
#include <linux/max9635.h>

extern struct adp8870_backlight_platform_data mp_adp8870_pdata;
extern struct akm8975_platform_data mp_akm8975_pdata;
extern struct attiny48mu_cap_prox_platform_data mp_attiny48mu_cap_prox_pdata;
extern struct bmp180_platform_data mp_bmp180_pdata;
extern struct ct405_platform_data mp_ct405_pdata;
extern struct isl29030_platform_data mp_isl29030_pdata;
extern struct isl29032_platform_data mp_isl29032_pdata;
extern struct kxtf9_platform_data mp_kxtf9_pdata;
extern struct lm3532_backlight_platform_data mp_lm3532_pdata;
extern struct lp8550_platform_data mp_lp8550_pdata;
extern struct lis3dh_platform_data mp_lis3dh_pdata;
extern struct l3g4200d_platform_data mp_l3g4200d_pdata;
extern struct max9635_platform_data mp_max9635_pdata;
extern struct msp430_platform_data mp_msp430_data;

#endif /* __BOARD_MAPPHONE_SENSORS_H */
