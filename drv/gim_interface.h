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

#ifndef _GPU_IOV_MODULE__GIM_INTERFACE_H
#define _GPU_IOV_MODULE__GIM_INTERFACE_H

#include "gim_adapter.h"
#include "gim_command.h"

/* Options */
#define GIM_OPTION__FB_PARTITION          0
#define GIM_OPTION__FB_PARTITION__FIX_SIZE      0
#define GIM_OPTION__FB_PARTITION__VARIED_SIZE   1

#define GIM_OPTION__SCHEDULER   1
#define GIM_OPTION__SCHEDULER__ROUND_ROBIN              0
#define GIM_OPTION__SCHEDULER__BORROWED_VIRTUAL_TIME    1

#define GIM_OPTION__VF_NUMBER   2
#define GIM_OPTION__VF_NUMBER__DEFAULT  0   /* Maximum VFs are enabled. */

#define GIM_OPTION__PF_FB_SIZE  3

/* Min size of FB is allocated to PF. */
#define GIM_OPTION__PF_FB_SIZE__DEFAULT 0

/* Max size of FB is allocated to PF. */
#define GIM_OPTION__PF_FB_SIZE__MAX	65536

uint32_t set_opt(uint32_t index, uint32_t value);
uint32_t get_opt(uint32_t index);

/* Multi-adapter management */
uint32_t get_pf_count(void);
uint32_t enumerate_pfs(uint32_t count, uint32_t *pf_bdfs);
uint32_t enumerate_all_vf_bdfs(uint32_t pf_bdf,
			uint32_t count, uint32_t *vf_bdfs);

/* sriov management */
uint32_t get_total_vfs(uint32_t pf_bdf);

uint32_t enable_sriov(uint32_t pf_bdf, uint32_t vfs_count);
uint32_t disable_sriov(uint32_t pf_bdf);

/* VF management */
uint32_t get_available_vf_count(uint32_t pf_bdf);
uint32_t enumerate_available_vfs(uint32_t pf_bdf,
			uint32_t count, uint32_t *vf_bdfs);
uint32_t vf_alloc(uint32_t vf_bdf);
uint32_t vf_free(uint32_t vf_bdf);
uint32_t set_pid(uint32_t vf_bdf, uint32_t pid);
uint32_t set_fb(uint32_t vf_bdf, uint32_t fb_offset);
uint32_t set_sched_level(uint32_t vf_bdf, uint32_t sched_level);
uint32_t set_acpi_event(uint32_t vf_bdf, uint32_t acpi_event);
uint32_t enumerate_all_vfs(uint32_t pf_bdf, uint32_t count,
				struct vf_info *vf_info);
/* FB management */
uint32_t get_max_fb_slot(uint32_t pf_bdf);
uint32_t fb_alloc(uint32_t pf_bdf, uint32_t fb_size);
uint32_t fb_free(uint32_t pf_bdf, uint32_t fb_offset);

/* Scheduler */
uint32_t pause_sched(uint32_t pf_bdf);
uint32_t resume_sched(uint32_t pf_bdf);
uint32_t start_scheduler(uint32_t pf_bdf);
uint32_t get_active_functions(uint32_t pf_bdf);
uint32_t init(uint32_t vf_bdf);
uint32_t idle(uint32_t vf_bdf);
uint32_t save(uint32_t vf_bdf);
uint32_t load(uint32_t vf_bdf);
uint32_t run(uint32_t vf_bdf);
uint32_t context_switch(uint32_t vf_bdf, uint32_t vf_bdf_next);

void switch_to_self(uint32_t pf_bdf, int enable);
void enable_preemption(uint32_t pf_bdf, int enable);

uint32_t send_signal(int vm_id, int signal_no);
uint32_t alloc_vf(uint32_t pf_bdf, int dom_id,  int qemu_pid,
		int fb_size, int sched_level);
uint32_t alloc_vf_from_bdf(uint32_t vf_bdf, int dom_id,
		int qemu_pid, int fb_size, int sched_level);
uint32_t free_vf_from_bdf(uint32_t vf_bdf);

uint32_t free_vf_from_domid(uint32_t domid);

uint32_t trgger_single_switch(uint32_t pf_bdf);

uint32_t do_world_switch(uint32_t pf_bdf);
void map_mmr(uint32_t vf_bdf, kcl_type_u64 phys_addr, kcl_type_u64 length);
struct pci_dev *bdf_to_dev(uint32_t bdf);
uint32_t get_pdev_bdf(struct pci_dev *pdev);
struct flr_state *get_flr_state_ptr(uint32_t bdf);
int is_pf(int bdf);
struct adapter *bdf_to_adapter(uint32_t bdf);
void remove_from_run_list(struct function *func);
void switch_to_vf(struct adapter *a, struct function *f);
struct function_list_node *get_function_node(struct function *f);
#endif

