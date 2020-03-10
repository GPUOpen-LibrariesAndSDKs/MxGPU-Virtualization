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
#include <linux/delay.h>
#include <linux/version.h>

#include "gim_adapter.h"
#include "gim_interface.h"
#include "gim_pci.h"
#include "gim_config.h"
#include "gim_timer.h"
#include "gim_debug.h"

uint32_t set_opt(uint32_t index, uint32_t value)
{
	return set_option(index, value);
}

uint32_t get_opt(uint32_t index)
{
	uint32_t res = 0;

	switch (index) {
	case GIM_OPTION__FB_PARTITION:
		res = get_fb_partition_option();
		break;

	case GIM_OPTION__SCHEDULER:
		res = get_scheduler_option();
		break;

	case GIM_OPTION__VF_NUMBER:
		res = get_vf_number_option();
		break;

	case GIM_OPTION__PF_FB_SIZE:
		res = get_pf_fb_option();
		break;

	default:
		res = -1;
		break;
	}
	return res;
}

static uint32_t idx_to_bdf(int adapter_idx)
{
	if (adapter_idx < get_adapter_count())
		return get_bdf(get_adapters()[adapter_idx].pf.pci_dev);
	return 0;
}

/*
 * Works for both PF or VF BDF's
 *
 */
struct function *bdf_to_function(uint32_t bdf)
{
	int i;
	struct adapter *adapt = get_adapters();

	if (adapt == NULL)
		return NULL;

	for (i = 0 ; i < get_adapter_count() ; ++i) {
		int     j;

		adapt = get_adapters() + i;

		if (adapt->pf.bdf == bdf)
			return &adapt->pf;

		for (j = 0; j < adapt->total_vfs; ++j) {
			if (adapt->vfs[j].bdf == bdf)
				return &adapt->vfs[j];
		}
	}
	return NULL;
}

/*
 * Works for both PF or VF
 */
struct flr_state *get_flr_state_ptr(uint32_t bdf)
{
	struct function *fn = bdf_to_function(bdf);

	if (fn != NULL)
		return &fn->flr_state;

	gim_info("%02x:%02x.%0x is not a valid BDF", ((bdf)>>8)&0xff,
					((bdf)>>3)&0x1f, (bdf)&0x7);
	return NULL;
}


/*
 * Works for both PF and VF
 */
struct adapter *bdf_to_adapter(uint32_t bdf)
{
	int i;
	struct adapter *adapt = get_adapters();

	for (i = 0 ; i < get_adapter_count() ; ++i) {
		int     j;

		adapt = get_adapters() + i;

		if (adapt->pf.bdf == bdf)
			return adapt;

		for (j = 0; j < adapt->total_vfs; ++j) {
			if (adapt->vfs[j].bdf == bdf)
				return adapt;
		}
	}
	gim_err("WARNING: bdf 0x%x cannot be found on any adapt\n", bdf);
	return NULL;
}

uint32_t get_pf_bdf(uint32_t bdf)
{
	struct adapter *adapt = bdf_to_adapter(bdf);

	if (adapt != NULL)
		return adapt->pf.bdf;
	return 0;
}

uint32_t get_pdev_bdf(struct pci_dev *pdev)
{
	return ((pdev->bus->number << 8) | pdev->devfn);
}

struct pci_dev *bdf_to_dev(uint32_t bdf)
{
	int i;

	for (i = 0 ; i < get_adapter_count() ; ++i) {
		int     j;
		struct adapter *adapt = get_adapters() + i;

		if (adapt->pf.bdf == bdf)
			return adapt->pf.pci_dev;

		for (j = 0; j < adapt->total_vfs; ++j) {
			if (adapt->vfs[j].bdf == bdf)
				return adapt->vfs[j].pci_dev;
		}
	}
	return NULL;
}

static struct pci_dev *bdf_to_pdev(uint32_t bdf)
{
	struct adapter *adapt = bdf_to_adapter(bdf);

	if (adapt != NULL)
		return adapt->pf.pci_dev;
	return NULL;
}
/*
 * Works for both PF or VF
 */
struct pci_dev *bdf_to_pcidev(int bdf)
{
	int i;

	for (i = 0 ; i < get_adapter_count() ; ++i) {
		int     j;
		struct adapter *adapt = get_adapters() + i;

		if (adapt->pf.bdf == bdf)
			return adapt->pf.pci_dev;

		for (j = 0; j < adapt->total_vfs; ++j) {
			if (adapt->vfs[j].bdf == bdf)
				return adapt->vfs[j].pci_dev;
		}
	}
	return NULL;
}



struct function *vfid_to_function(struct adapter *adapt, uint32_t vf_id)
{
	int i;

	for (i = 0 ; i < adapt->enabled_vfs; ++i) {
		if (adapt->vfs[i].func_id == vf_id)
			return &adapt->vfs[i];
	}
	return NULL;
}

struct adapter *vf_to_adapter(uint32_t vf_bdf)
{
	int i;

	for (i = 0; i < get_adapter_count(); ++i) {
		int j;
		struct adapter *adapt = get_adapters() + i;

		for (j = 0; j < adapt->enabled_vfs; ++j) {
			if (adapt->vfs[j].bdf == vf_bdf)
				return adapt;
		}
	}
	return NULL;
}


struct function *vf_to_function(uint32_t vf_bdf)
{
	int i;

	for (i = 0 ; i < get_adapter_count() ; ++i) {
		int j;
		struct adapter *adapt = get_adapters() + i;

		for (j = 0 ; j < adapt->total_vfs ; ++j) {
			if (adapt->vfs[j].bdf == vf_bdf &&
				adapt->vfs[j].func_id < adapt->enabled_vfs)
				return &adapt->vfs[j];
		}
	}
	return NULL;
}

static struct function *domid_to_function(uint32_t domid)
{
	int i;

	for (i = 0 ; i < get_adapter_count() ; ++i) {
		int j;
		struct adapter *adapt = get_adapters() + i;

		for (j = 0 ; j < adapt->total_vfs ; ++j) {
			if (adapt->vfs[j].is_available == 0 &&
				adapt->vfs[j].dom_id == domid)
				return &adapt->vfs[j];
		}
	}
	return NULL;
}

uint32_t get_pf_count(void)
{
	return get_adapter_count();
}

uint32_t enumerate_pfs(uint32_t count, uint32_t *pf_bdfs)
{
	uint32_t i;

	for (i = 0 ; i < count && i < get_adapter_count() ; ++i)
		pf_bdfs[i] = idx_to_bdf(i);
	return i;
}

uint32_t  get_total_vfs(uint32_t pf_bdf)
{
	struct pci_dev *pdev = bdf_to_pdev(pf_bdf);

	return pci_sriov_get_totalvfs(pdev);
}

uint32_t enable_sriov(uint32_t pf_bdf, uint32_t vfs_count)
{
	struct pci_dev *pdev = bdf_to_pdev(pf_bdf);

	if (vfs_count == 0 || vfs_count > get_total_vfs(pf_bdf))
		vfs_count = get_total_vfs(pf_bdf);

	gim_info("Enable SRIOV\n");
	gim_info("Enable SRIOV vfs count = %d\n", vfs_count);
	if (pdev != NULL) {
		int res = pci_enable_sriov(pdev, vfs_count);

		if (res == 0) {
			struct adapter *adapt = bdf_to_adapter(pf_bdf);

			init_vfs(adapt, vfs_count);
			return GIM_OK;

		} else {
			gim_err("Fail to enable sriov, status = %x\n", res);
			return GIM_ERROR;
		}
	} else {
		return GIM_ERROR;
	}
}

uint32_t disable_sriov(uint32_t pf_bdf)
{
	struct pci_dev *pdev = bdf_to_pdev(pf_bdf);

	gim_info("Disable SRIOV\n");

	if (pdev != NULL)
		pci_disable_sriov(pdev);
	return GIM_OK;
}


/* VF management */
uint32_t get_available_vf_count(uint32_t pf_bdf)
{
	struct adapter *adapt = bdf_to_adapter(pf_bdf);

	if (adapt != NULL)
		return adapt->available_vfs;
	return 0;
}

uint32_t enumerate_available_vfs(uint32_t pf_bdf, uint32_t count,
						uint32_t *vf_bdfs)
{
	struct adapter *adapt = bdf_to_adapter(pf_bdf);

	if (adapt != NULL) {
		uint32_t i     = 0;
		uint32_t avail = 0;

		for (i = 0; i < adapt->enabled_vfs && avail < count; ++i) {
			if (adapt->vfs[i].is_available)
				vf_bdfs[avail++] = adapt->vfs[i].bdf;
		}
		return avail;
	}
	return 0;
}

uint32_t enumerate_all_vf_bdfs(uint32_t pf_bdf, uint32_t count,
						uint32_t *vf_bdfs)
{
	struct adapter *adapt = bdf_to_adapter(pf_bdf);

	if (adapt != NULL) {
		uint32_t i     = 0;
		uint32_t avail = 0;

		for (i = 0; i < adapt->enabled_vfs && avail < count; ++i)
			vf_bdfs[avail++] = adapt->vfs[i].bdf;
		return avail;
	}
	return 0;
}

uint32_t enumerate_all_vfs(uint32_t pf_bdf, uint32_t count,
					struct vf_info *vf_info)
{
	struct adapter *adapt = bdf_to_adapter(pf_bdf);

	if (adapt != NULL) {
		uint32_t i = 0;

		for (i = 0; i < adapt->total_vfs && i < count; ++i) {
			if (adapt->vfs[i].func_id < adapt->enabled_vfs) {
				vf_info[i].bdf = adapt->vfs[i].bdf;
				vf_info[i].sched_level =
					adapt->vfs[i].sched_level;
				vf_info[i].fb_size =
					adapt->vfs[i].fb_partition->slot.size;
				vf_info[i].dom_id = adapt->vfs[i].dom_id;
				vf_info[i].state = adapt->vfs[i].is_available;
				get_hw_fb_setting(adapt,
					adapt->vfs[i].func_id,
					&(vf_info[i].hw_fb_start),
					&(vf_info[i].hw_fb_size));
				vf_info[i].func_id = adapt->vfs[i].func_id;
			}
		}
		return i;
	}
	return 0;
}

uint32_t get_max_fb_slot(uint32_t pf_bdf)
{
	struct adapter *adapt = bdf_to_adapter(pf_bdf);

	return adapt->max_fb_slot;
}

uint32_t fb_alloc(uint32_t pf_bdf, uint32_t fbSize)
{
	return true;
}

uint32_t fb_free(uint32_t pf_bdf, uint32_t fbOffset)
{
	return true;
}

uint32_t pause_sched(uint32_t pf_bdf)
{
	struct adapter *adapt = bdf_to_adapter(pf_bdf);

	if (adapt == NULL)
		return false;

	delete_timer(&adapt->sched_timer);
	adapt->scheduler_running = false;
	return true;
}

uint32_t resume_sched(uint32_t pf_bdf)
{
	struct adapter *adapt = bdf_to_adapter(pf_bdf);

	if (adapt == NULL)
		return false;

	restart_timer(&adapt->sched_timer);
	adapt->scheduler_running = true;
	return true;
}

uint32_t start_scheduler(uint32_t pf_bdf)
{
	int new_quota;
	struct adapter *adapt = bdf_to_adapter(pf_bdf);

	if (adapt == NULL)
		return false;

	new_quota = adapt->quota;
	if (adapt->curr_running_func == NULL)
		return true;

	switch (adapt->curr_running_func->func->sched_level) {
	case 0:
		new_quota *= 4;
		break;
	case 1:
		new_quota *= 6;
		break;
	case 2:
		new_quota *= 8;
		break;
	default:
		break;

	}

	start_timer(&adapt->sched_timer, new_quota);

	adapt->scheduler_running = true;
	return true;
}

void enable_preemption(uint32_t pf_bdf, int enable)
{
	struct adapter *adapt = bdf_to_adapter(pf_bdf);

	if (adapt == NULL)
		return;


	if (!adapt->scheduler_running) {
		adapt->enable_preeption = enable;
		gim_info("%s mid-buffer preemption in GIM\n",
				enable ? "enable":"disable");

		return;
	}

	gim_err("%s mid-buffer preemption in GIM failed;"
		"please stop gpu scheduler first\n",
		enable ? "enable":"disable");

}


uint32_t send_signal(int vm_id, int signal_no)
{
	gim_err("WARNING: Function not supported!\n");
	return 0;
}

void switch_to_self(uint32_t pf_bdf, int enable)
{
	struct adapter *adapt = bdf_to_adapter(pf_bdf);

	if (adapt == NULL)
		return;

	 if (!adapt->scheduler_running) {
		adapt->switch_to_itself = enable;
		gim_info("%s world switch to the same function\n",
				enable ? "enable":"disable");
		return;
	 }

	gim_err("%s world switch to the same function failed, please stop gpu scheduler first\n",
		enable ? "enable":"disable");

}

uint32_t do_world_switch(uint32_t pf_bdf)
{
	struct adapter *adapt;

	adapt = bdf_to_adapter(pf_bdf);
	world_switch(adapt);
	return true;
}

uint32_t trgger_single_switch(uint32_t pf_bdf)
{
	struct adapter *adapt =  bdf_to_adapter(pf_bdf);

	if (adapt == NULL)
		return false;

	if (adapt->scheduler_running)
		gim_info("world switch scheduler is running now, please stop the scheduler first\n");
	else
		trigger_world_switch(adapt, true);

	return true;
}


uint32_t alloc_vf(uint32_t bdf, int dom_id, int qemu_pid, int fb_size,
							int sched_level)
{
	struct function *func = NULL;

	func = vf_to_function(bdf);


	gim_warn("This function assumes that gfx_pci points to a specific VF\n");
	gim_warn("needs to be re-written to be able to specify PF or VF.If PF then get next available VF\n");
	/*
	 * Note that we can't get an arbitrary VF from a PF yet
	 *because of issues in QEMU that need to be resolved first
	 * Currently we have to specify pci=<VF>.
	 *Once it can be obtained from  gfx_pci instead then this will work
	 */
	gim_info("alloc_vf: Bdf:%x,dom_id:%d, qemu_pid:%d,fb_size:%d, sched_level:%d\n",
		bdf, dom_id, qemu_pid, fb_size, sched_level);
	if (func) {
		if (func->is_available == 1)
			alloc_new_vf(func, dom_id, qemu_pid,
					fb_size, sched_level);
		else {
			gim_warn("bdf %03x is already in use, can't allocate it, force reuse for now\n",
					bdf);
			alloc_new_vf(func, dom_id, qemu_pid,
					fb_size, sched_level);
		}

		return func->bdf;
	}
	return 0;
}

uint32_t alloc_vf_from_bdf(uint32_t vf_bdf, int dom_id,
			int qemu_pid, int fb_size, int sched_level)
{
	int ret = 1;
	struct function *func = vf_to_function(vf_bdf);

	alloc_new_vf(func, dom_id, qemu_pid, fb_size, sched_level);
	return ret;
}

/*
 * Map a Host physical address to a virtual address
 */
void map_mmr(uint32_t vf_bdf, kcl_type_u64 phys_addr, kcl_type_u64 length)
{
#if 0
	gim_info("map_mmr bdf = %x, addre = %x, length = %x\n",
		vf_bdf, phys_addr, length);
	Function *func = vf_to_function(vf_bdf);

	func->mmio_base = ioremap(phys_addr, length);
	gim_info("map_mmr ptr_base = %x\n", func->mmio_ptr);
#endif

}

uint32_t free_vf_from_bdf(uint32_t vf_bdf)
{
	int ret = 1;
	struct function *func = vf_to_function(vf_bdf);

	free_vf(func);
	return ret;
}

uint32_t free_vf_from_domid(uint32_t domid)
{
	int ret = 1;
	struct function *func = domid_to_function(domid);

	if (func)
		free_vf(func);

	return ret;
}

int is_pf(int bdf)
{
	int i;

	for (i = 0 ; i < get_adapter_count() ; ++i) {
		struct adapter *adapt = get_adapters() + i;

		if (adapt->pf.bdf == bdf)
			return 1;
	}
	return 0;
}
