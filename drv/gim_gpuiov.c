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

#include <linux/pci.h>
#include <linux/pci_regs.h>

#include "gim_pci.h"
#include "gim_adapter.h"
#include "gim_gpuiov.h"

#include "gim_debug.h"
#include "gim_interface.h"

int pci_gpu_iov_init(struct pci_dev *dev, struct pci_gpu_iov *gpuiov)
{
	int off;
	int pos;
	int is_found = 0;
	kcl_type_u32 data;

	gpuiov->pos = 0;

	if (!pci_is_pcie(dev))
		return -ENODEV;

	/* Look for the GPU-IOV capability structure. */
	/* It will be under the Vendor specific code */
	pos = 0;
	while ((pos = pci_find_next_ext_capability(dev,
						pos,
						PCI_EXT_CAP_ID_VNDR)))	{
		kcl_type_u32 vsec = 0;

		off = pos + PCI_GPUIOV_VSEC;
		pci_read_config_dword(dev, off, &vsec);
		gpuiov->vsec_id = PCI_GPUIOV_VSEC__ID(vsec);

		if (gpuiov->vsec_id == PCI_GPUIOV_VSEC__ID__GPU_IOV) {
			gpuiov->vsec_rev = PCI_GPUIOV_VSEC__REV(vsec);
			gpuiov->vsec_len   = PCI_GPUIOV_VSEC__LENGTH(vsec);
			is_found = 1;
			break;
		}
	}

	if (!is_found) {
		gim_err("AMD GIM pci_gpu_iov_init: no GPU_IOV caps found.\n");
		return -ENODEV;
	}

	gpuiov->pos = pos;

	off = gpuiov->pos + PCI_GPUIOV_FCN_ID;
	pci_read_config_byte(dev, off, &gpuiov->func_id);

	off = gpuiov->pos + PCI_GPUIOV_NXT_FCN_ID;
	pci_read_config_byte(dev, off, &gpuiov->next_func_id);

	off = gpuiov->pos + PCI_GPUIOV_CNTXT;
	pci_read_config_dword(dev, off, &data);
	gpuiov->context_size   = PCI_GPUIOV_CNTXT__SIZE(data);
	gpuiov->context_loc    = PCI_GPUIOV_CNTXT__LOC(data);
	gpuiov->context_offset = PCI_GPUIOV_CNTXT__OFFSET(data);

	off = gpuiov->pos + PCI_GPUIOV_TOTAL_FB_AVAILABLE;
	pci_read_config_word(dev, off, &gpuiov->total_fb_available);

	gim_info("total_fb_available = %d\n ", gpuiov->total_fb_available);
	gim_info("AMD GIM pci_gpu_iov_init pos = %x\n", gpuiov->pos);
	gim_info("AMD GIM pci_gpu_iov_init total_fb_available = %x\n",
		gpuiov->total_fb_available);

	return 0;
}

int set_gpuiov_context(struct pci_dev *dev,
			struct pci_gpu_iov *gpuiov,
			kcl_type_u8 context_size, kcl_type_u8 context_loc,
			kcl_type_u32 context_offset)
{
	int off;
	kcl_type_u32 data;

	if (gpuiov->pos == 0)
		return -1;

	data = TO_256KBYTES(MBYTES_TO_BYTES(context_offset))/context_size;
	data = PCI_GPUIOV_CNTXT__SIZE__PUT(context_size)
		| PCI_GPUIOV_CNTXT__LOC__PUT(context_loc)
		| PCI_GPUIOV_CNTXT__OFFSET__PUT(data);
	off = gpuiov->pos + PCI_GPUIOV_CNTXT;
	pci_write_config_dword(dev, off, data);

	gpuiov->context_size   = context_size;
	gpuiov->context_loc    = context_loc;
	gpuiov->context_offset = context_offset;

	return 0;
}

int set_gpuiov_function_id(struct pci_dev *dev,
			struct pci_gpu_iov *gpuiov,
			kcl_type_u8 function_id)
{
	int off;

	if (gpuiov->pos == 0)
		return -1;

	off = gpuiov->pos + PCI_GPUIOV_FCN_ID;
	pci_write_config_byte(dev, off, function_id);

	gpuiov->func_id = function_id;

	return 0;
}

int set_gpuiov_next_function_id(struct pci_dev *dev,
				struct pci_gpu_iov *gpuiov,
				kcl_type_u8 next_func_id)
{
	int off;

	if (gpuiov->pos == 0)
		return -1;

	off = gpuiov->pos + PCI_GPUIOV_NXT_FCN_ID;
	pci_write_config_byte(dev, off, next_func_id);

	gpuiov->next_func_id = next_func_id;

	return 0;
}

int set_gpuiov_total_fb_consumed(struct pci_dev *dev,
				struct pci_gpu_iov *gpuiov,
				kcl_type_u16 total_fb_consumed)
{
	int off;

	if (gpuiov->pos == 0)
		return -1;

	off = gpuiov->pos + PCI_GPUIOV_TOTAL_FB_CONSUMED;
	pci_write_config_word(dev, off, total_fb_consumed);
	gpuiov->total_fb_consumed = total_fb_consumed;

	return 0;
}

int set_gpuiov_vf_frame_buffer(struct pci_dev *dev,
				struct pci_gpu_iov *gpuiov,
				int vf, kcl_type_u16 size, kcl_type_u16 offset)
{
	int off;
	kcl_type_u32 data;

	data = (((size)>>4) - 1) | ((offset >> 4) << 16);
	off = gpuiov->pos + PCI_GPUIOV_VF0_FB_SIZE + vf * sizeof(kcl_type_u32);
	pci_write_config_dword(dev, off, data);

	gpuiov->vf_fb_size[vf] = size;
	gpuiov->vf_fb_offset[vf] = offset;

	return 0;
}

int set_gpuiov_command(struct pci_dev *dev, struct pci_gpu_iov *gpuiov,
			int command, int func_id, int next_func_id)
{
	int off;
	kcl_type_u32 data;

	data = command | CMD_EXECUTE | (func_id << 8) | (next_func_id << 16);
	off = gpuiov->pos + PCI_GPUIOV_CMD_CONTROL;
	pci_write_config_dword(dev, off, data);

	return 0;
}

int get_vf_hw_fb_settings(struct pci_dev *pf_dev,
			  struct pci_gpu_iov *gpuiov,
			  int vf_id, uint32_t *fb_start,
			  uint32_t *fb_size)
{
	uint16_t size;
	uint16_t start;
	int off;

	off = gpuiov->pos + PCI_GPUIOV_VF0_FB_SIZE + (vf_id * 4);
	pci_read_config_word(pf_dev, off, &size);

	off = gpuiov->pos + PCI_GPUIOV_VF0_FB_OFFSET + (vf_id * 4);
	pci_read_config_word(pf_dev, off, &start);

	*fb_start = start * 16;  /* Register is in units of 16MB */
	*fb_size  = size * 16;   /* Register is in units of 16MB */
	*fb_size += 16;   /* Register is size - 16MB */

	return 0;
}
