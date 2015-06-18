/*
 * OMAP4 OPP table definitions.
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *	Kevin Hilman
 *	Thara Gopinath
 * Copyright (C) 2010-2011 Nokia Corporation.
 *      Eduardo Valentin
 *      Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/opp.h>

#include <plat/cpu.h>
#include <plat/common.h>

#include "control.h"
#include "omap_opp_data.h"
#include "pm.h"

/*
 * Structures containing OMAP4430 voltage supported and various
 * voltage dependent data for each VDD.

 *	Stock CM12 Reference voltage
#define OMAP4430_VDD_MPU_OPP50_UV		1025000		300Mhz
#define OMAP4430_VDD_MPU_OPP100_UV		1200000		600Mhz
#define OMAP4430_VDD_MPU_OPPTURBO_UV		1325000		800Mhz
#define OMAP4430_VDD_MPU_OPPNITRO_UV		1388000		1.0Ghz
#define OMAP4430_VDD_MPU_OPPNITROB_UV		1398000		1.2Ghz

*/


#define OMAP4430_VDD_MPU_OPP50_UV		1025000		/* 300Mhz */
#define OMAP4430_VDD_MPU_OPP50A_UV		1076000		/* 400Mhz */
#define OMAP4430_VDD_MPU_OPP50B_UV		1123000		/* 500Mhz */
#define OMAP4430_VDD_MPU_OPP100_UV		1200000		/* 600Mhz */
#define OMAP4430_VDD_MPU_OPP100A_UV		1275000		/* 700Mhz */
#define OMAP4430_VDD_MPU_OPPTURBO_UV		1325000		/* 800Mhz */
#define OMAP4430_VDD_MPU_OPPTURBOA_UV		1355000		/* 900Mhz */
#define OMAP4430_VDD_MPU_OPPNITRO_UV		1388000		/* 1.0Ghz */
#define OMAP4430_VDD_MPU_OPPNITROA_UV		1390000		/* 1.1Ghz */
#define OMAP4430_VDD_MPU_OPPNITROB_UV		1398000		/* 1.2Ghz */
#define OMAP4430_VDD_MPU_OPPNITROC_UV		1402000		/* 1.3Ghz */
#define OMAP4430_VDD_MPU_OPPNITROD_UV		1415000		/* 1.4Ghz */
#define OMAP4430_VDD_MPU_OPPNITROD_ULV		1409000		/* 1.4Ghz */
#define OMAP4430_VDD_MPU_OPPNITROD_UV_HV	1439000		/* 1.4Ghz HV */

struct omap_volt_data omap443x_vdd_mpu_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP, 0),
#if !defined(CONFIG_USE_FREQ_REDUC_MOD) && defined(CONFIG_USE_MULTI_FREQ)
	/* Add 400mhz and 500Mhz */
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP50A_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP50B_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP, 0),
#endif
#if !defined(CONFIG_USE_FREQ_REDUC_MOD) && !defined(CONFIG_USE_MULTI_FREQ)
	/* Add 600Mhz and 800Mhz */
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP, 0),
#endif
#if !defined(CONFIG_USE_FREQ_REDUC_MOD) && defined(CONFIG_USE_MULTI_FREQ)
	/* Add 600, 700 ,800 and 900Mhz */
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP100A_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBOA_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP, 0),
#endif
	/* Add 1.0Ghz */
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITRO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0xfa, 0x27, OMAP_ABB_FAST_OPP, 0),
#ifdef CONFIG_USE_MULTI_FREQ
	/* Add 1.1Ghz */
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROA_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP, 0),
#endif
	/* Add 1.2Ghz */
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROB_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP, 0),
#ifdef CONFIG_USE_13_CLOCK
	/* Add 1.3Ghz */
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROC_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP, 0),
#endif
#ifdef CONFIG_USE_14_CLOCK
	/* Add 1.4Ghz */
#ifdef CONFIG_USE_HIGH_VOLTAGE
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROD_UV_HV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP, 0),
#else
#ifdef CONFIG_USE_ULV_VOLTAGE
        VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROD_ULV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP, 0),
#else
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITROD_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP, 0),
#endif
	#endif
#endif
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0),
};

#define OMAP4430_VDD_IVA_OPP50_UV		950000
#define OMAP4430_VDD_IVA_OPP100_UV		1114000
#define OMAP4430_VDD_IVA_OPPTURBO_UV		1291000

struct omap_volt_data omap443x_vdd_iva_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0),
};

#define OMAP4430_VDD_CORE_OPP50_UV		962000
#ifndef CONFIG_USE_GPU_OC
#define OMAP4430_VDD_CORE_OPP100_UV		1127000
#else
#define OMAP4430_VDD_CORE_OPP100_UV		1250000
#endif
struct omap_volt_data omap443x_vdd_core_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_CORE_OPP50, 0xf4, 0x0c, OMAP_ABB_NONE, 0),
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0xf9, 0x16, OMAP_ABB_NONE, 0),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0),
};

/* Dependency of domains are as follows for OMAP4430 (OPP based):
 *
 *	MPU	IVA	CORE
 *	50	50	50+
 *	50	100+	100
 *	100+	50	100
 *	100+	100+	100
 */

/* OMAP 4430 MPU Core VDD dependency table */
static struct omap_vdd_dep_volt omap443x_vdd_mpu_core_dep_data[] = {
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP50_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
#if !defined(CONFIG_USE_FREQ_REDUC_MOD) && defined(CONFIG_USE_MULTI_FREQ)
	/* Add 400mhz and 500Mhz */
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP50A_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP50B_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
#endif
#if !defined(CONFIG_USE_FREQ_REDUC_MOD) && !defined(CONFIG_USE_MULTI_FREQ)
	/* Add 600Mhz and 800Mhz */
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP100_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPTURBO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
#endif
#if !defined(CONFIG_USE_FREQ_REDUC_MOD) && defined(CONFIG_USE_MULTI_FREQ)
	/* Add 600, 700 ,800 and 900Mhz */
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP100_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP100A_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPTURBO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPTURBOA_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
#endif
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITRO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
#ifdef CONFIG_USE_MULTI_FREQ
	/* Add 1.1Ghz */
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITROA_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
#endif
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITROB_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
#ifdef CONFIG_USE_13_CLOCK
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITROC_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
#endif
#ifdef CONFIG_USE_14_CLOCK
#ifdef CONFIG_USE_HIGH_VOLTAGE
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITROD_UV_HV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
#else
#ifdef CONFIG_USE_ULV_VOLTAGE
        {.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITROD_ULV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
#else
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITROD_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
#endif
#endif
#endif
};

struct omap_vdd_dep_info omap443x_vddmpu_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap443x_vdd_mpu_core_dep_data,
		.nr_dep_entries = ARRAY_SIZE(omap443x_vdd_mpu_core_dep_data),
	},
	{.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

/* OMAP 4430 MPU IVA VDD dependency table */
static struct omap_vdd_dep_volt omap443x_vdd_iva_core_dep_data[] = {
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPP50_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPP100_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPPTURBO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
};

struct omap_vdd_dep_info omap443x_vddiva_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap443x_vdd_iva_core_dep_data,
		.nr_dep_entries = ARRAY_SIZE(omap443x_vdd_iva_core_dep_data),
	},
	{.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

static struct omap_opp_def __initdata omap443x_opp_def_list[] = {
	/* MPU OPP1 - OPP50 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 300000000, OMAP4430_VDD_MPU_OPP50_UV),
#if !defined(CONFIG_USE_FREQ_REDUC_MOD) && defined(CONFIG_USE_MULTI_FREQ)
	/* Add 400mhz and 500Mhz */
	/* MPU OPP1 - OPP50 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 400000000, OMAP4430_VDD_MPU_OPP50A_UV),
	/* MPU OPP1 - OPP50 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 500000000, OMAP4430_VDD_MPU_OPP50B_UV),
#endif
#if !defined(CONFIG_USE_FREQ_REDUC_MOD) && !defined(CONFIG_USE_MULTI_FREQ)
	/* Add 600Mhz and 800Mhz */
	/* MPU OPP2 - OPP100 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 600000000, OMAP4430_VDD_MPU_OPP100_UV),
	/* MPU OPP3 - OPP-Turbo */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 800000000, OMAP4430_VDD_MPU_OPPTURBO_UV),
#endif
#if !defined(CONFIG_USE_FREQ_REDUC_MOD) && defined(CONFIG_USE_MULTI_FREQ)
	/* Add 600, 700 ,800 and 900Mhz */
	/* MPU OPP2 - OPP100 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 600000000, OMAP4430_VDD_MPU_OPP100_UV),
	/* MPU OPP2 - OPP100 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 700000000, OMAP4430_VDD_MPU_OPP100A_UV),
	/* MPU OPP3 - OPP-Turbo */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 800000000, OMAP4430_VDD_MPU_OPPTURBO_UV),
	/* MPU OPP3 - OPP-Turbo */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 900000000, OMAP4430_VDD_MPU_OPPTURBOA_UV),
#endif
	/* MPU OPP4 - OPP-NT */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 1000000000, OMAP4430_VDD_MPU_OPPNITRO_UV),
#ifdef CONFIG_USE_MULTI_FREQ
	/* Add 1.1Ghz */
	/* MPU OPP5 - OPP-SB */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 1100000000, OMAP4430_VDD_MPU_OPPNITROA_UV),
#endif
	/* MPU OPP5 - OPP-SB */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", false, 1200000000, OMAP4430_VDD_MPU_OPPNITROB_UV),
#ifdef CONFIG_USE_13_CLOCK
	/* MPU OPP4 - OPP-NTSB1 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", false, 1300000000, OMAP4430_VDD_MPU_OPPNITROC_UV),
#endif
#ifdef CONFIG_USE_14_CLOCK
	/* MPU OPP4 - OPP-NTSB2 */
#ifdef CONFIG_USE_HIGH_VOLTAGE
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", false, 1400000000, OMAP4430_VDD_MPU_OPPNITROD_UV_HV),
#else
#ifdef CONFIG_USE_ULV_VOLTAGE
        OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", false, 1400000000, OMAP4430_VDD_MPU_OPPNITROD_ULV),
#else
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", false, 1400000000, OMAP4430_VDD_MPU_OPPNITROD_UV),
#endif
#endif
#endif
	/* L3 OPP1 - OPP50 */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 100000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* L3 OPP2 - OPP100, OPP-Turbo, OPP-SB */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 200000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* IVA OPP1 - OPP50 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 133000000, OMAP4430_VDD_IVA_OPP50_UV),
	/* IVA OPP2 - OPP100 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 266100000, OMAP4430_VDD_IVA_OPP100_UV),
	/* IVA OPP3 - OPP-Turbo */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 332000000, OMAP4430_VDD_IVA_OPPTURBO_UV),
	/* SGX OPP1 - OPP50 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 153600000, OMAP4430_VDD_CORE_OPP50_UV),
#ifndef CONFIG_USE_GPU_OC
	/* SGX OPP2 - OPP100 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 307200000, OMAP4430_VDD_CORE_OPP100_UV),
#else
	/* SGX OPP2 - OPP100A */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 384000000, OMAP4430_VDD_CORE_OPP100_UV),
#endif
	/* FDIF OPP1 - OPP25 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 32000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* FDIF OPP2 - OPP50 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 64000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* FDIF OPP3 - OPP100 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 128000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* DSP OPP1 - OPP50 */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 232750000, OMAP4430_VDD_IVA_OPP50_UV),
	/* DSP OPP2 - OPP100 */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 465500000, OMAP4430_VDD_IVA_OPP100_UV),
	/* DSP OPP3 - OPPTB */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", false, 496000000, OMAP4430_VDD_IVA_OPPTURBO_UV),
	/* HSI OPP1 - OPP50 */
	OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 96000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* HSI OPP2 - OPP100 */
	OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 192000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* ABE OPP1 - OPP50 */
	OPP_INITIALIZER("aess", "abe_clk", "iva", true, 98304000, OMAP4430_VDD_IVA_OPP50_UV),
	/* ABE OPP2 - OPP100 */
	OPP_INITIALIZER("aess", "abe_clk", "iva", true, 196608000, OMAP4430_VDD_IVA_OPP100_UV),
};

#define OMAP4460_VDD_MPU_OPP50_UV		1025000
#define OMAP4460_VDD_MPU_OPP100_UV		1203000
#define OMAP4460_VDD_MPU_OPPTURBO_UV		1317000
#define OMAP4460_VDD_MPU_OPPNITRO_UV		1380000
#define OMAP4460_VDD_MPU_OPPNITROSB_UV		1390000

struct omap_volt_data omap446x_vdd_mpu_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPTURBO_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xfa, 0x23, OMAP_ABB_FAST_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPNITRO_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0xfa, 0x27, OMAP_ABB_FAST_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPNITROSB_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP, 0),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0),
};

#define OMAP4460_VDD_IVA_OPP50_UV		 950000
#define OMAP4460_VDD_IVA_OPP100_UV		1140000
#define OMAP4460_VDD_IVA_OPPTURBO_UV		1291000
#define OMAP4460_VDD_IVA_OPPNITRO_UV		1375000

struct omap_volt_data omap446x_vdd_iva_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPPTURBO_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP, 0),
	VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPPNITRO_UV, 10000, OMAP44XX_CONTROL_FUSE_IVA_OPPNITRO, 0xfa, 0x23, OMAP_ABB_FAST_OPP, 0),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0),
};

#define OMAP4460_VDD_CORE_OPP50_UV		 962000
#define OMAP4460_VDD_CORE_OPP100_UV		1127000
#define OMAP4460_VDD_CORE_OPP100_OV_UV		1250000

struct omap_volt_data omap446x_vdd_core_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP50, 0xf4, 0x0c, OMAP_ABB_NONE, 0),
	VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP100_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0xf9, 0x16, OMAP_ABB_NONE, 0),
	VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP100_OV_UV, 10000, OMAP44XX_CONTROL_FUSE_CORE_OPP100OV, 0xf9, 0x16, OMAP_ABB_NONE, 0),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0, 0),
};

/* OMAP 4460 MPU Core VDD dependency table */
static struct omap_vdd_dep_volt omap446x_vdd_mpu_core_dep_data[] = {
	{.main_vdd_volt = OMAP4460_VDD_MPU_OPP50_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4460_VDD_MPU_OPP100_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4460_VDD_MPU_OPPTURBO_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4460_VDD_MPU_OPPNITRO_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4460_VDD_MPU_OPPNITROSB_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
};

struct omap_vdd_dep_info omap446x_vddmpu_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap446x_vdd_mpu_core_dep_data,
		.nr_dep_entries = ARRAY_SIZE(omap446x_vdd_mpu_core_dep_data),
	},
	{.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

/* OMAP 4460 MPU IVA VDD dependency table */
static struct omap_vdd_dep_volt omap446x_vdd_iva_core_dep_data[] = {
	{.main_vdd_volt = OMAP4460_VDD_IVA_OPP50_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4460_VDD_IVA_OPP100_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4460_VDD_IVA_OPPTURBO_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
};

struct omap_vdd_dep_info omap446x_vddiva_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap446x_vdd_iva_core_dep_data,
		.nr_dep_entries = ARRAY_SIZE(omap446x_vdd_iva_core_dep_data),
	},
	{.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

static struct omap_opp_def __initdata omap446x_opp_def_list[] = {
	/* MPU OPP1 - OPP50 */
	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", true, 350000000, OMAP4460_VDD_MPU_OPP50_UV),
	/* MPU OPP2 - OPP100 */
	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", true, 700000000, OMAP4460_VDD_MPU_OPP100_UV),
	/* MPU OPP3 - OPP-Turbo */
	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", true, 920000000, OMAP4460_VDD_MPU_OPPTURBO_UV),
	/* MPU OPP4 - OPP-Nitro */
	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", false, 1200000000, OMAP4460_VDD_MPU_OPPNITRO_UV),
	/* MPU OPP4 - OPP-Nitro SpeedBin */
	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", false, 1500000000, OMAP4460_VDD_MPU_OPPNITROSB_UV),
	/* L3 OPP1 - OPP50 */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 100000000, OMAP4460_VDD_CORE_OPP50_UV),
	/* L3 OPP2 - OPP100 */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 200000000, OMAP4460_VDD_CORE_OPP100_UV),
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 200000000, OMAP4460_VDD_CORE_OPP100_OV_UV),
	/* IVA OPP1 - OPP50 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 133000000, OMAP4460_VDD_IVA_OPP50_UV),
	/* IVA OPP2 - OPP100 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 266100000, OMAP4460_VDD_IVA_OPP100_UV),
	/*
	 * IVA OPP3 - OPP-Turbo + Disabled as the reference schematics
	 * recommends Phoenix VCORE2 which can supply only 600mA - so the ones
	 * above this OPP frequency, even though OMAP is capable, should be
	 * enabled by board file which is sure of the chip power capability
	 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 332000000, OMAP4460_VDD_IVA_OPPTURBO_UV),
	/* IVA OPP4 - OPP-Nitro */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 430000000, OMAP4460_VDD_IVA_OPPNITRO_UV),
	/* IVA OPP5 - OPP-Nitro SpeedBin*/
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 500000000, OMAP4460_VDD_IVA_OPPNITRO_UV),

	/* SGX OPP1 - OPP50 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 153600000, OMAP4460_VDD_CORE_OPP50_UV),
	/* SGX OPP2 - OPP100 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 307200000, OMAP4460_VDD_CORE_OPP100_UV),
	/* SGX OPP3 - OPPOV */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 384000000, OMAP4460_VDD_CORE_OPP100_OV_UV),
	/* FDIF OPP1 - OPP25 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 32000000, OMAP4460_VDD_CORE_OPP50_UV),
	/* FDIF OPP2 - OPP50 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 64000000, OMAP4460_VDD_CORE_OPP50_UV),
	/* FDIF OPP3 - OPP100 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 128000000, OMAP4460_VDD_CORE_OPP100_UV),
	/* DSP OPP1 - OPP50 */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 232750000, OMAP4460_VDD_IVA_OPP50_UV),
	/* DSP OPP2 - OPP100 */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 465500000, OMAP4460_VDD_IVA_OPP100_UV),
	/* DSP OPP3 - OPPTB */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", false, 496000000, OMAP4460_VDD_IVA_OPPTURBO_UV),
	/* HSI OPP1 - OPP50 */
	OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 96000000, OMAP4460_VDD_CORE_OPP50_UV),
	/* HSI OPP2 - OPP100 */
	OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 192000000, OMAP4460_VDD_CORE_OPP100_UV),
	/* ABE OPP1 - OPP50 */
	OPP_INITIALIZER("aess", "abe_clk", "iva", true, 98304000, OMAP4460_VDD_IVA_OPP50_UV),
	/* ABE OPP2 - OPP100 */
	OPP_INITIALIZER("aess", "abe_clk", "iva", true, 196608000, OMAP4460_VDD_IVA_OPP100_UV),
};

/**
 * omap4_mpu_opp_enable() - helper to enable the OPP
 * @freq:	frequency to enable
 */
static void __init omap4_mpu_opp_enable(unsigned long freq)
{
	struct device *mpu_dev;
	int r;

	mpu_dev = omap2_get_mpuss_device();
	if (!mpu_dev) {
		pr_err("%s: no mpu_dev, did not enable f=%ld\n", __func__,
			freq);
		return;
	}

	r = opp_enable(mpu_dev, freq);
	if (r < 0)
		dev_err(mpu_dev, "%s: opp_enable failed(%d) f=%ld\n", __func__,
			r, freq);
}

#ifdef CONFIG_USE_GPU_OC
/**
 * omap4_gpu_opp_enable() - helper to enable the GPU OPP
 * @freq:	frequency to enable
*/
static void __init omap4_gpu_opp_enable(unsigned long freq)
{
	struct device *gpu_dev;
	int r;

	gpu_dev = omap4_get_gpu_device();
	if (!gpu_dev) {
		pr_err("%s: no gpu_dev, did not enable f=%ld\n", __func__,
			freq);
		return;
	}

	r = opp_enable(gpu_dev, freq);
	if (r < 0)
		dev_err(gpu_dev, "%s: opp_enable failed(%d) f=%ld\n", __func__,
			r, freq);
}
#endif

/**
 * omap4_opp_init() - initialize omap4 opp table
 */
int __init omap4_opp_init(void)
{
	int r = -ENODEV;
	int trimmed = 1;

	if (!cpu_is_omap44xx())
		return r;
	if (cpu_is_omap443x())
		r = omap_init_opp_table(omap443x_opp_def_list,
			ARRAY_SIZE(omap443x_opp_def_list));
	else if (cpu_is_omap446x()) {
		r = omap_init_opp_table(omap446x_opp_def_list,
			ARRAY_SIZE(omap446x_opp_def_list));
		trimmed = omap_readl(0x4a002268) & ((1 << 18) | (1 << 19));
		/* if device is untrimmed override DPLL TRIM register */
		if (!trimmed)
			omap_writel(0x29, 0x4a002330);
	}

	if (!r) {
		pr_info("omap4_opp_init r is not set.");
		if (omap4_has_mpu_1_2ghz()) {
			omap4_mpu_opp_enable(1200000000);

#ifdef CONFIG_USE_13_CLOCK
			omap4_mpu_opp_enable(1300000000);
#endif
#ifdef CONFIG_USE_14_CLOCK
			omap4_mpu_opp_enable(1400000000);
#endif
		}
		if (!trimmed)
			pr_info("This is DPLL un-trimmed SOM. OPP is limited at 1.2 GHz\n");
		if (omap4_has_mpu_1_5ghz() && trimmed)
			omap4_mpu_opp_enable(1500000000);
	}

#ifdef CONFIG_USE_GPU_OC
	/* Confirm GPU OC OPP Enabled */
	omap4_gpu_opp_enable(384000000);
#endif	
	return r;
}
device_initcall(omap4_opp_init);
