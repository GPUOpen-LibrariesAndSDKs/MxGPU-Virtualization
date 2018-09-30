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

#ifndef _GPU_IOV_MODULE__FLR_H

#define _GPU_IOV_MODULE__FLR_H

/* For VF, save and restore the first 1024 Bytes pci cfg space
 * For PF, save and restore the first 1280 Bytes pci cfg space
 */
#define VF_FLR_PCI_CONFIG_SIZE  1024
#define PF_FLR_PCI_CONFIG_SIZE  1280
#define PCI_CONFIG_SIZE_TO_SAVE 4096

struct flr_state {
	unsigned int    pcie_strap_misc;
	unsigned int    swrst_ep_control_0;
	unsigned int    config_memsize;
	unsigned char   pci_cfg[VF_FLR_PCI_CONFIG_SIZE];
};

int gim_save_cpc_state(struct adapter *adapt, unsigned int *cp_cpc_ic);
int gim_restore_cpc_state(struct adapter *adapt, unsigned int *cp_cpc_ic);
int gim_save_vddgfx_state(struct adapter *adapt, struct function *vf);
int gim_vf_flr(struct adapter *adapt, struct function *vf);
int gim_function_level_reset(struct adapter *adapt, struct function *vf);

#endif
