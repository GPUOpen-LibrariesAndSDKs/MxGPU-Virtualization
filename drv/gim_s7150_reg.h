/*
 * Copyright (c) 2014-2017 Advanced Micro Devices, Inc. All rights reserved.
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

#ifndef _GPU_IOV_MODULE__S7150_REG_H
#define _GPU_IOV_MODULE__S7150_REG_H

#include "gmc/gmc_8_1_d.h"
#include "gmc/gmc_8_1_sh_mask.h"

#include "bif/bif_5_0_d.h"
#include "bif/bif_5_0_sh_mask.h"

#include "oss/oss_3_0_d.h"
#include "oss/oss_3_0_sh_mask.h"

#include "gca/gfx_8_0_d.h"
#include "gca/gfx_8_0_sh_mask.h"

#include "smu/smu_7_1_2_d.h"
#include "smu/smu_7_1_2_sh_mask.h"

#define ixCG_INTERRUPT_STATUS                      0xC0200048

#define CG_INTERRUPT_STATUS__DISP_TIMER2_TRIGGER_MASK_MASK 0x00000400L
#define CG_INTERRUPT_STATUS__DISP_TIMER_TRIGGER_MASK_MASK  0x00000800L

#define CG_INTERRUPT_STATUS__DISP_TIMER2_TRIGGER_MASK__SHIFT 10
#define CG_INTERRUPT_STATUS__DISP_TIMER_TRIGGER_MASK__SHIFT  11

#define SRBM_STATUS__DRM_RQ_PENDING_MASK                   0x00000001L
#define SRBM_STATUS__DRM_BUSY_MASK                         0x00040000L

#define ixSWRST_EP_CONTROL_0                       0x140010C

#define ixSMC_PC_A                                 0x80000358
#define ixSMC_PC_F                                 0x8000035C
#define ixSMC_PC_D                                 0x80000360
#define ixSMC_PC_X                                 0x80000364
#define ixSMC_PC_M                                 0x80000368
#define ixSMC_PC_W                                 0x8000036C
#define ixSMC_PC_C                                 0x80000370

#define mmSMU_ACTIVE_FCN_ID                             0x01C2

#define mmCP_DMA_PIO_CONTROL				0xC063
#define mmCP_DMA_PIO_SRC_ADDR				0xC064
#define mmCP_DMA_PIO_SRC_ADDR_HI			0xC065
#define mmCP_DMA_PIO_DST_ADDR                           0xC066
#define mmCP_DMA_PIO_DST_ADDR_HI                        0xC067
#define mmCP_DMA_PIO_COMMAND				0xC0E8

#define mmCC_BIF_BX_STRAP0 0x14e1

#endif /* __S7150_REG_H__ */

