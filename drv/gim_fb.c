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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>


#include "gim_fb.h"
#include "gim_adapter.h"
#include "gim_command.h"
#include "gim_config.h"
#include "gim_gpuiov.h"
#include "gim_debug.h"
#include "gim_dma.h"


int init_context_save_area(struct adapter *adapt, kcl_type_u64 base,
						kcl_type_u64 size)
{
	gim_info("AMD GIM init_context_save_area: base =%llx size=%llx.\n",
			base, size);

	set_gpuiov_context(adapt->pf.pci_dev, &adapt->gpuiov, size,
						PCI_GPUIOV_CNTXT__LOC_IN_FB,
						base);
	return 0;
}

int init_pf_fb(struct adapter *adapt, kcl_type_u64 size)
{
	/* PF occupies the frame buffer always starting from 0. */
	adapt->partition[PF_INDEX].slot.base = 0;
	adapt->partition[PF_INDEX].slot.size = size;

	adapt->pf.fb_partition = &adapt->partition[PF_INDEX];

	/* Programming total_fb_consumed in GPUIOV. */
	gim_info("total framebuffer available = %x\n",
			adapt->gpuiov.total_fb_available);
	gim_info("pf framebuffer = %llx\n", size);
	gim_info("total framebuffer consumed = %llx\n",
			adapt->gpuiov.total_fb_available - size);
	set_gpuiov_total_fb_consumed(adapt->pf.pci_dev, &adapt->gpuiov,
				     adapt->gpuiov.total_fb_available - size);
	return 0;
}

void init_vf_fb(struct adapter *adapt, struct function *func)
{
	/* Convert from GPUIOV values to byte values */
	unsigned long long fb_offset = func->fb_partition->slot.base << 20;
	unsigned long long fb_size = func->fb_partition->slot.size << 20;

	gim_info("init_vf_fb(fb_offset = 0x%08llx, size=0x%08llx\n", fb_offset,
			 fb_size);
	dma_clear(adapt, fb_offset, fb_size);
}

int init_fb_dynamic(struct adapter *adapt, kcl_type_u64 base,
					kcl_type_u64 total_size)
{
	struct slot_list_node *slot;
	struct partition *partition;
	kcl_type_u32 i, j;

	slot = vmalloc(sizeof(struct slot_list_node));
	slot->slot.base = base;
	slot->slot.size = total_size;
	slot->next = NULL;

	adapt->max_fb_slot = total_size;
	adapt->empty_slot_list = slot;
	mutex_init(&adapt->fb_dynamic_alloc_mutex);

	partition = &adapt->partition[VF0_INDEX];
	for (i = 0 ; i < adapt->total_vfs ; ++i) {
		partition->slot.base = 0;
		partition->slot.size = 0;

		for (j = 0; j < adapt->total_vfs; j++) {
			if (adapt->vfs[j].func_id == i)
				adapt->vfs[j].fb_partition = partition;
		}
		partition++;
	}
	return 0;
}

int get_fb_size_static(struct adapter *adapter, u32 *num_vf, u64 base,
	u64 total_size, u64 min_size)
{
	u32 new_count;
	u32 vf_fb_size;

	if (get_vf_fb_option() == VF_FB__DEFAULT) {
		vf_fb_size = rounddown(total_size / *num_vf, FB__SIZE_IN_16M);
		if (vf_fb_size < min_size) {
			// Not enough memory to support all vfs.
			*num_vf = total_size / min_size;
			vf_fb_size = rounddown(min_size, FB__SIZE_IN_16M);
		}
	} else {
		vf_fb_size	= get_vf_fb_option();
		gim_info("VF FB size specified as %dMB, min_size = %lld\n",
			vf_fb_size, min_size);
		if (vf_fb_size < min_size)
			vf_fb_size = min_size;
		new_count = total_size / vf_fb_size;
		if (new_count < *num_vf)
			*num_vf = new_count;
		vf_fb_size = rounddown(vf_fb_size, FB__SIZE_IN_16M);
	}

	return vf_fb_size;
}

int init_fb_static(struct adapter *adapt, kcl_type_u32 num_vf,
				kcl_type_u64 base, kcl_type_u64 total_size,
					kcl_type_u64 min_size)
{
	kcl_type_u32 i, j;
	struct partition *partition;

	gim_info("AMD GIM init_fb_static: num_vf = %d, base= %lld, total_size=%lld, min_size=%lld\n",
			 num_vf, base, total_size, min_size);
	if (num_vf > 0) {
		kcl_type_u32 vf_fb_size;
		kcl_type_u32 vf_avg_fb_size;

		vf_avg_fb_size = rounddown(total_size / num_vf,
				FB__SIZE_IN_16M);
		if (get_vf_fb_option() == VF_FB__DEFAULT) {
			vf_fb_size  = vf_avg_fb_size;
		} else {
			vf_fb_size  = get_vf_fb_option();
			gim_info("VF FB size specified as %dMB, min_size = %lld\n",
					 vf_fb_size, min_size);
			if (vf_fb_size < min_size)
				vf_fb_size = min_size;

			if (vf_fb_size > vf_avg_fb_size)
				vf_fb_size = vf_avg_fb_size;
		}

		gim_info("AMD GIM init_fb_static: vf_fb_size = %d, base= %lld\n",
				 vf_fb_size, base);

		adapt->max_fb_slot = vf_fb_size;

		partition = &adapt->partition[VF0_INDEX];
		for (i = 0 ; i < num_vf ; ++i) {
			partition->slot.base = base;
			partition->slot.size = vf_fb_size;

			gim_info("AMD GIM init_fb_static: partition %d base =%lld,size= %lld\n",
				 i, partition->slot.base,
				 partition->slot.size);

			base += vf_fb_size;

			set_gpuiov_vf_frame_buffer(adapt->pf.pci_dev,
						   &adapt->gpuiov, i,
						   partition->slot.size,
						   partition->slot.base);

			for (j = 0; j < adapt->total_vfs; j++) {
				if (adapt->vfs[j].func_id == i)
					adapt->vfs[j].fb_partition = partition;
			}
			partition++;
		}
	}
	return 0;
}

int init_frame_buffer_partition(struct adapter *adapt)
{
	kcl_type_u64 max_fb_size;
	kcl_type_u64 max_pf_fb_size;
	kcl_type_u64 mini_pf_fb_size;
	kcl_type_u64 mini_vf_fb_size;
	kcl_type_u64 csa_base;
	kcl_type_u64 vf_fb_base;
	kcl_type_u64 pf_fb_size;
	kcl_type_u64 vf_fb_size;
	struct resource *res;

	res = &adapt->pf.pci_dev->resource[BAR__FRAME_BUFFER];
	mini_pf_fb_size = TO_MBYTES(res->end - res->start + 1);
	/*
	 * If large bar is enabled, PF does not work as vga device.
	 * Set minimal pf fb to be 16M bytes.
	 */
	if (mini_pf_fb_size >= adapt->gpuiov.total_fb_available)
		mini_pf_fb_size = 16;

	gim_info("PCI defined PF FB size = %lld MB\n", mini_pf_fb_size);

	res = &adapt->vfs[0].pci_dev->resource[BAR__FRAME_BUFFER];
	mini_vf_fb_size = TO_MBYTES(res->end - res->start + 1);
	gim_info("PCI defined VF FB size = %lld MB\n", mini_vf_fb_size);
	max_fb_size = rounddown(adapt->gpuiov.total_fb_available -
		roundup(FB__RESERVED_CSA_IN_1M, FB__SIZE_IN_16M),
						FB__SIZE_IN_16M);
	gim_info("Total FB Available = %d MB, CSA = %d MB\n",
		adapt->gpuiov.total_fb_available, FB__RESERVED_CSA_IN_1M);
	gim_info("Max Remaining FB Size = %lld\n", max_fb_size);
	/* Validate the PF frame buffer size */
	pf_fb_size = (kcl_type_u64)get_pf_fb_option();
	if (pf_fb_size <= mini_pf_fb_size)
		pf_fb_size = mini_pf_fb_size;

	max_pf_fb_size = max_fb_size
			 - adapt->enabled_vfs
			 * roundup(mini_vf_fb_size, FB__SIZE_IN_16M);
	if (pf_fb_size > max_pf_fb_size)
		pf_fb_size = max_pf_fb_size;

	gim_info("PF FB size after checking limits from config file = %lldMB\n",
			pf_fb_size);
	pf_fb_size = rounddown(pf_fb_size, FB__SIZE_IN_16M);
	gim_info("PF rounded down to nearest 16MB boundary = %lld\n",
		 pf_fb_size);

	/* Init the PF frame buffer. */
	init_pf_fb(adapt, pf_fb_size);

	/* CSA is reserved at top of the frame buffer. */
	csa_base = pf_fb_size;
	gim_info("CSA starts at offset %lldMB\n", csa_base);
	init_context_save_area(adapt, pf_fb_size, FB__CSA_SIZE_IN_256K);

	/* VFs FB follows PF FB, starting from vf_fb_size */
	vf_fb_base = roundup(csa_base + FB__RESERVED_CSA_IN_1M,
			     FB__SIZE_IN_16M);
	gim_info("VF FB base = %lldMB (%lld + %d)\n", vf_fb_base, csa_base,
		 roundup(FB__RESERVED_CSA_IN_1M, FB__SIZE_IN_16M));

	vf_fb_size = rounddown(max_fb_size - pf_fb_size, FB__SIZE_IN_16M);
	gim_info("VF FB Size = %lldMB (%lld - %lld)\n", vf_fb_size, max_fb_size,
			 pf_fb_size);

	/* save it for monitoring use */
	adapt->vf_fb_base = vf_fb_base;
	adapt->vf_fb_size = vf_fb_size;

	if (get_fb_partition_option() == FB_PARTITION__STATIC) {
		init_fb_static(adapt, adapt->enabled_vfs, vf_fb_base,
				vf_fb_size, mini_vf_fb_size);
		adapt->available_vfs = adapt->enabled_vfs;
	} else if (get_fb_partition_option() == FB_PARTITION__DYNAMIC) {
		init_fb_dynamic(adapt, vf_fb_base, vf_fb_size);
	} else {
		return -1;
	}

	return 0;
}

