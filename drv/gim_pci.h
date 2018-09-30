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

#ifndef _GPU_IOV_MODULE__PCI_H
#define _GPU_IOV_MODULE__PCI_H

#include "gim_kcl_type.h"

/* PCI_EXT_CAP_ID_GPUIOV is equal to PCI_EXT_CAP_ID_VNDR.*/
#define PCI_GPUIOV_MAX_VFS 16
struct pci_gpu_iov {
	int pos;

	kcl_type_u8 version;

    /* VSEC definition. */
	kcl_type_u16 vsec_id;
	kcl_type_u8  vsec_rev;
	kcl_type_u16 vsec_len;

	kcl_type_u8 func_id;
	kcl_type_u8 next_func_id;

	kcl_type_u8  context_size;
	/* 0: local frame buffer, 1: system memory. */
	kcl_type_u8  context_loc;
	kcl_type_u32 context_offset;

	kcl_type_u16 total_fb_available;
	kcl_type_u16 total_fb_consumed;

	kcl_type_u16 vf_fb_size[PCI_GPUIOV_MAX_VFS];
	kcl_type_u16 vf_fb_offset[PCI_GPUIOV_MAX_VFS];
};



/* GPU Virtualization */
#define PCI_EXT_CAP_ID__GPUIOV 0x0b

#define PCI_GPUIOV_CAP              0x02 /* 16bits */
#define PCI_GPUIOV_CAP__VER(x) ((x)&0xf)
#define PCI_GPUIOV_NEXT_CAP__OFFSET(x) ((x)>>4)

#define PCI_GPUIOV_VSEC             0x04 /* 32bits*/
#define PCI_GPUIOV_VSEC__ID__GPU_IOV 0x02
#define PCI_GPUIOV_VSEC__ID(x)  ((x) & 0xffff)
#define PCI_GPUIOV_VSEC__REV(x) (((x) >> 16) & 0x0f)
#define PCI_GPUIOV_VSEC__LENGTH(x) ((x)>>20)


#define PCI_GPUIOV_CMD_CONTROL      0x0c /* 8bits */
#define PCI_GPUIOV_FCN_ID           0x0d /* 8bits */
#define PCI_GPUIOV_NXT_FCN_ID       0x0e /* 8bits */

#define PCI_GPUIOV_CMD_STATUS       0x10 /* 8bits */

#define PCI_GPUIOV_RESET_CONTROL    0x14 /* 8bits */


#define PCI_GPUIOV_RESET_NOTIFICATION 0x18 /* 32bits */
#define PCI_GPUIOV_VM_INIT_STATUS     0x1c /* 32bits */

#define PCI_GPUIOV_CNTXT            0x20 /* 32bits */
#define PCI_GPUIOV_CNTXT__SIZE(x) ((x) & 0x7f)
#define PCI_GPUIOV_CNTXT__LOC(x) (((x) >> 7) & 0x01)
#define PCI_GPUIOV_CNTXT__OFFSET(x) ((x) >> 10)


#define PCI_GPUIOV_CNTXT__LOC_IN_FB  0
#define PCI_GPUIOV_CNTXT__LOC_IN_SYS 1

#define PCI_GPUIOV_CNTXT__SIZE__PUT(x) ((x) & 0x7f)
#define PCI_GPUIOV_CNTXT__LOC__PUT(x) (((x) & 0x01) << 7)
#define PCI_GPUIOV_CNTXT__OFFSET__PUT(x) (((x) & 0x3fffff) << 10)

#define PCI_GPUIOV_TOTAL_FB_AVAILABLE 0x24 /* 16bits */
#define PCI_GPUIOV_TOTAL_FB_CONSUMED  0x26 /* 16bits */

#define PCI_GPUIOV_AUTO_SCH_OFFSET    0x2a /* 8bits */
#define PCI_GPUIOV_DISP_OFFSET        0x2b /* 8bits */


#define PCI_GPUIOV_VF0_FB_SIZE      0x2c /* 16bits */
#define PCI_GPUIOV_VF0_FB_OFFSET    0x2e /* 16bits */

#define PCI_GPUIOV_VF1_FB_SIZE      0x30 /* 16bits */
#define PCI_GPUIOV_VF1_FB_OFFSET    0x32 /* 16bits */

#define PCI_GPUIOV_VF2_FB_SIZE      0x34 /* 16bits */
#define PCI_GPUIOV_VF2_FB_OFFSET    0x36 /* 16bits */

#define PCI_GPUIOV_VF3_FB_SIZE      0x38 /* 16bits */
#define PCI_GPUIOV_VF3_FB_OFFSET    0x3a /* 16bits */

#define PCI_GPUIOV_VF4_FB_SIZE      0x3c /* 16bits */
#define PCI_GPUIOV_VF4_FB_OFFSET    0x3e /* 16bits */

#define PCI_GPUIOV_VF5_FB_SIZE      0x40 /* 16bits */
#define PCI_GPUIOV_VF5_FB_OFFSET    0x42 /* 16bits */

#define PCI_GPUIOV_VF6_FB_SIZE      0x44 /* 16bits */
#define PCI_GPUIOV_VF6_FB_OFFSET    0x46 /* 16bits */

#define PCI_GPUIOV_VF7_FB_SIZE      0x48 /* 16bits */
#define PCI_GPUIOV_VF7_FB_OFFSET    0x4a /* 16bits */

#define PCI_GPUIOV_VF8_FB_SIZE      0x4c /* 16bits */
#define PCI_GPUIOV_VF8_FB_OFFSET    0x4e /* 16bits */

#define PCI_GPUIOV_VF9_FB_SIZE      0x50 /* 16bits */
#define PCI_GPUIOV_VF9_FB_OFFSET    0x52 /* 16bits */

#define PCI_GPUIOV_VF10_FB_SIZE     0x54 /* 16bits */
#define PCI_GPUIOV_VF10_FB_OFFSET   0x56 /* 16bits */

#define PCI_GPUIOV_VF11_FB_SIZE     0x58 /* 16bits */
#define PCI_GPUIOV_VF11_FB_OFFSET   0x5a /* 16bits */

#define PCI_GPUIOV_VF12_FB_SIZE     0x5c /* 16bits */
#define PCI_GPUIOV_VF12_FB_OFFSET   0x5e /* 16bits */

#define PCI_GPUIOV_VF13_FB_SIZE     0x60 /* 16bits */
#define PCI_GPUIOV_VF13_FB_OFFSET   0x62 /* 16bits */

#define PCI_GPUIOV_VF14_FB_SIZE     0x64 /* 16bits */
#define PCI_GPUIOV_VF14_FB_OFFSET   0x66 /* 16bits */

#define PCI_GPUIOV_VF15_FB_SIZE     0x68 /* 16bits */
#define PCI_GPUIOV_VF15_FB_OFFSET   0x6c /* 16bits */

#define PCI_SRIOV_CAP_ARI_PRESERVED 0x2

struct pci_device_id;
uint32_t get_bdf(struct pci_dev *pdev);
int enumerate_vfs(struct pci_dev *dev, int count, struct pci_dev *vf_devices[]);
int enumerate_all_pfs(const struct pci_device_id *device_list, int count,
			struct pci_dev *pf_devices[]);
int get_func_id(struct pci_dev *pf_dev, struct pci_dev *vf_dev);

int sriov_is_ari_enabled(struct pci_dev *pf_dev);
int sriov_enable_ari(struct pci_dev *pf_dev);
void enable_device(struct pci_dev *dev);

#endif

