/*
 * Copyright (c) 2017 Advanced Micro Devices, Inc. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE
 */

#ifndef _GPU_IOV_MODULE__MONITOR_TONGA_H
#define _GPU_IOV_MODULE__MONITOR_TONGA_H

#include "gim_monitor.h"

#define PPSMC_MSG_SCLKDPM_GETENABLEDMASK      (0x162)
#define PPSMC_MSG_API_GETSCLKFREQUENCY        (0x200)

/* Thermal */
enum thermal_sensor_type {
	/* TCEN0 sensors */
	GNB_TCEN_0,
	GNB_TCEN_1,
	GNB_TCEN_2,
	GNB_TCEN_3,
	GNB_TCEN_4,
	GNB_TCEN_5,
	GNB_TCEN_6,
	GNB_TCEN_7,
	GNB_TCON_GLOBAL,

	/* TCEN1 sensors */
	GNB_TCEN1_0,
	GNB_TCEN1_1,
	GNB_TCEN1_2,
	GNB_TCEN1_3,
	GNB_TCEN1_4,
	GNB_TCEN1_5,
	GNB_TCEN1_6,
	GNB_TCEN1_7,

	/* TCEN2 sensors */
	GNB_TCEN2_0,
	GNB_TCEN2_1,
	GNB_TCEN2_2,
	GNB_TCEN2_3,
	GNB_TCEN2_4,
	GNB_TCEN2_5,
	GNB_TCEN2_6,
	GNB_TCEN2_7,

	GNB_TCEN_0_MAX,
	GNB_TCEN_1_MAX,
	GNB_TCEN_2_MAX,

	TCEN_PWR_DN_STATUS,

	/* discrete GPUs */
	TSS_CURRENT,    /* TSS reading based on selection */
	TSS_0,
	TSS_1,
	TSS_2,
	TSS_3,
	TSS_4,

	/* internal thermal temp replacing the TSS ones from SI */
	ASIC_MAX,
	CTF,
	SW_CTF_LIMIT,
	HW_CTF_LIMIT,

	/* external thermal sensor */
	LM63,    /* typical discrete GPU external thermal controller */

	/* estimated temperature readings (added for Trinity, Kaveri, Kabini) */
	EST_CPU_0,
	EST_CPU_1,
	EST_CPU_2,
	EST_CPU_3,
	EST_CPU_4,
	EST_CPU_5,
	EST_GPU,

	/* Tahiti */
	TMON_0_INT,     /* internal thermal sensor in TMON itself */
	TMON_1_INT,     /* internal thermal sensor in TMON itself */
	TMON_2_INT,     /* internal thermal sensor in TMON itself */

	TMON_0_RDIL0,
	TMON_0_RDIL1,
	TMON_0_RDIL2,
	TMON_0_RDIL3,
	TMON_0_RDIL4,
	TMON_0_RDIL5,
	TMON_0_RDIL6,
	TMON_0_RDIL7,

	TMON_0_RDIL8,
	TMON_0_RDIL9,
	TMON_0_RDIL10,
	TMON_0_RDIL11,

	/* Hawaii */
	TMON_0_RDIL12,
	TMON_0_RDIL13,
	TMON_0_RDIL14,
	TMON_0_RDIL15,


	TMON_0_RDIR0,
	TMON_0_RDIR1,
	TMON_0_RDIR2,
	TMON_0_RDIR3,
	TMON_0_RDIR4,
	TMON_0_RDIR5,
	TMON_0_RDIR6,
	TMON_0_RDIR7,

	TMON_0_RDIR8,
	TMON_0_RDIR9,
	TMON_0_RDIR10,
	TMON_0_RDIR11,

	/* Hawaii */
	TMON_0_RDIR12,
	TMON_0_RDIR13,
	TMON_0_RDIR14,
	TMON_0_RDIR15,

	TMON_1_RDIL0,
	TMON_1_RDIL1,
	TMON_1_RDIL2,
	TMON_1_RDIL3,
	TMON_1_RDIL4,
	TMON_1_RDIL5,
	TMON_1_RDIL6,
	TMON_1_RDIL7,

	/* Hawaii */
	TMON_1_RDIL8,
	TMON_1_RDIL9,
	TMON_1_RDIL10,
	TMON_1_RDIL11,
	TMON_1_RDIL12,
	TMON_1_RDIL13,
	TMON_1_RDIL14,
	TMON_1_RDIL15,

	TMON_1_RDIR0,
	TMON_1_RDIR1,
	TMON_1_RDIR2,
	TMON_1_RDIR3,
	TMON_1_RDIR4,
	TMON_1_RDIR5,
	TMON_1_RDIR6,
	TMON_1_RDIR7,

	/* Hawaii */
	TMON_1_RDIR8,
	TMON_1_RDIR9,
	TMON_1_RDIR10,
	TMON_1_RDIR11,
	TMON_1_RDIR12,
	TMON_1_RDIR13,
	TMON_1_RDIR14,
	TMON_1_RDIR15,

	/* fiji */
	TMON_2_RDIL0,
	TMON_2_RDIL1,
	TMON_2_RDIL2,
	TMON_2_RDIL3,
	TMON_2_RDIL4,
	TMON_2_RDIL5,
	TMON_2_RDIL6,
	TMON_2_RDIL7,

	TMON_2_RDIL8,
	TMON_2_RDIL9,
	TMON_2_RDIL10,
	TMON_2_RDIL11,
	TMON_2_RDIL12,
	TMON_2_RDIL13,
	TMON_2_RDIL14,
	TMON_2_RDIL15,

	TMON_2_RDIR0,
	TMON_2_RDIR1,
	TMON_2_RDIR2,
	TMON_2_RDIR3,
	TMON_2_RDIR4,
	TMON_2_RDIR5,
	TMON_2_RDIR6,
	TMON_2_RDIR7,

	TMON_2_RDIR8,
	TMON_2_RDIR9,
	TMON_2_RDIR10,
	TMON_2_RDIR11,
	TMON_2_RDIR12,
	TMON_2_RDIR13,
	TMON_2_RDIR14,
	TMON_2_RDIR15,

	/* TMON thermal gradient */
	TMON_GRADIENT,  /* M2E */
	TMON_GRADIENT_M2M,

	/* DTE temperature */
	DTE,

	/* RDI reading corresponding to CTF */
	CTF_RDI,

	/* memory chip temperature */
	HBM_STACK0,
	HBM_STACK1,
	HBM_STACK2,
	HBM_STACK3,

	/* unknown/invalid */
	TSS_UNKNOWN
};


union cg_mult_thermal_ctrl {
	struct {
#if defined(LITTLEENDIAN_CPU)
		unsigned int                       ts_filter : 4;
		unsigned int                          unused : 5;
		unsigned int               thermal_range_rst : 1;
		unsigned int                                 : 10;
		unsigned int                        temp_sel : 8;
		unsigned int                 thm_ready_clear : 1;
		unsigned int                                 : 3;
#elif defined(BIGENDIAN_CPU)
		unsigned int                                 : 3;
		unsigned int                 thm_ready_clear : 1;
		unsigned int                        temp_sel : 8;
		unsigned int                                 : 10;
		unsigned int               thermal_range_rst : 1;
		unsigned int                          unused : 5;
		unsigned int                       ts_filter : 4;
#endif
	} bitfields, bits;
	unsigned int u32all;
	signed int	i32all;
	float	f32all;
};

union thm_tmon0_rdil0_data {
	struct {
#if		defined(LITTLEENDIAN_CPU)
		unsigned int                               z : 11;
		unsigned int                           valid : 1;
		unsigned int                            temp : 12;
		unsigned int                                 : 8;
#elif	defined(BIGENDIAN_CPU)
		unsigned int                                 : 8;
		unsigned int                            temp : 12;
		unsigned int                           valid : 1;
		unsigned int                               z : 11;
#endif
	} bitfields, bits;
	unsigned int	u32all;
	signed int	i32all;
	float	f32all;
};

#define AMDGIM_POWER_DELAY_MS               100

#define ixS0_VID_SMIO_CNTL                         0xC06001E4
#define ixPWR_SVI2_STATUS                          0xC0200294

union pwr_svi2_status {
	struct {
#if		defined(LITTLEENDIAN_CPU)
		unsigned int                      plane1_vid : 8;
		unsigned int                      plane2_vid : 8;
		unsigned int                 vid_change_busy : 1;
		unsigned int                       svi2_busy : 1;
		unsigned int                  delay_cnt_busy : 1;
		unsigned int                                 : 13;
#elif defined(BIGENDIAN_CPU)
		unsigned int                                 : 13;
		unsigned int                  delay_cnt_busy : 1;
		unsigned int                       svi2_busy : 1;
		unsigned int                 vid_change_busy : 1;
		unsigned int                      plane2_vid : 8;
		unsigned int                      plane1_vid : 8;
#endif
	} bitfields, bits;
	unsigned int	u32all;
	signed int	i32all;
	float	f32all;
};

/* SMC */
#define SMC_WAIT_TICK_MS        10
#define SMC_WAIT_TIMEOUT_MS     (1 * 1000)

union smc_msg_arg_1 {
	struct {
#if             defined(LITTLEENDIAN_CPU)
		unsigned int                     smc_msg_arg : 32;
#elif defined(BIGENDIAN_CPU)
		unsigned int                     smc_msg_arg : 32;
#endif
	} bitfields, bits;
	unsigned int    u32all;
	signed int      i32all;
	float   f32all;
};

union smc_message_1 {
	struct {
#if             defined(LITTLEENDIAN_CPU)
		unsigned int                         smc_msg : 16;
		unsigned int                                 : 16;
#elif defined(BIGENDIAN_CPU)
		unsigned int                                 : 16;
		unsigned int                         smc_msg : 16;
#endif
	} bitfields, bits;
	unsigned int    u32all;
	signed int      i32all;
	float   f32all;
};

union smc_resp_1 {
	struct {
#if             defined(LITTLEENDIAN_CPU)
		unsigned int                        smc_resp : 16;
		unsigned int                                 : 16;
#elif defined(BIGENDIAN_CPU)
		unsigned int                                 : 16;
		unsigned int                        smc_resp : 16;
#endif
	} bitfields, bits;
	unsigned int    u32all;
	signed int      i32all;
	float   f32all;
};

#endif
