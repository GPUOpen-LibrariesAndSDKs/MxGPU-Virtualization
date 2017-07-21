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

#include <linux/string.h>
#include <linux/module.h>

#include "gim_interface.h"
#include "gim_command.h"
#include "gim_unwrapper.h"
#include "gim_flr.h"
#include "gim_debug.h"

static char output_buf[SYS_BUFFER_SIZE];

uint32_t call_interface_functions(const void *buf, size_t count)
{
	struct command *command = (struct command *)buf;
	struct output  *output  = (struct output *)output_buf;

	output->size   = sizeof(struct output);
	output->result = GIM_ERROR;

	if (count >= sizeof(struct command)) {
		if (command->version != GIM_HEADER_VER) {
			output->result = GIM_INVALIDATE_COMMAND_SIZE;
			return GIM_INVALIDATE_VERSION;
		}

	}

	switch (command->command) {
	case GIM__MA__GET_OPTION: {
		struct get_option_input  *in =
				(struct get_option_input *)command;
		struct get_option_output *out =
				(struct get_option_output *)output_buf;

		if (in->size != sizeof(struct get_option_input)) {
			out->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}
		out->size   = sizeof(struct get_option_output);
		out->value  = get_opt(in->index);
		out->result = (out->value >= 0) ?
			GIM_OK : GIM_INVALIDATE_OPTION;
		break;
	}

	case GIM__MA__SET_OPTION: {
		struct set_option_input  *in  =
				(struct set_option_input *)command;

		if (in->size != sizeof(struct set_option_input)) {
			output->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}
		output->result = (set_opt(in->index, in->value) == 0)
					? GIM_OK : GIM_INVALIDATE_OPTION;
		break;
	}

	case GIM__MA__GET_PF_COUNT: {
		struct get_pf_count_output *out =
			(struct get_pf_count_output *)output_buf;

		if (command->size != sizeof(struct command)) {
			out->result = GIM_INVALIDATE_COMMAND_SIZE;
			return GIM_INVALIDATE_COMMAND_SIZE;
		}

		out->size    = sizeof(struct get_pf_count_output);
		out->result  = GIM_OK;
		out->pf_count = get_pf_count();
		break;
	}

	case GIM__MA__ENUMERATE_PFS: {
		struct enumerate_pfs_output *out =
			(struct enumerate_pfs_output *)output_buf;

		if (command->size != sizeof(struct command)) {
			out->result = GIM_INVALIDATE_COMMAND_SIZE;
			return GIM_INVALIDATE_COMMAND_SIZE;
		}

		out->pf_count = enumerate_pfs((SYS_BUFFER_SIZE - output->size) /
						sizeof(uint32_t), out->pf_bdfs);
		out->result  = GIM_OK;
		if (out->pf_count > 0)
			out->size = sizeof(struct enumerate_pfs_output) +
					(out->pf_count - 1) * sizeof(uint32_t);
		else
			out->size =
			sizeof(struct enumerate_pfs_output) - sizeof(uint32_t);
		break;
	}

	case GIM__SRIOV__GET_TOTAL_VF_COUNT: {
		struct get_total_vfs_input  *in  =
				(struct get_total_vfs_input *)command;
		struct get_total_vfs_output *out = (
				struct get_total_vfs_output *)output_buf;

		if (in->size != sizeof(struct get_total_vfs_input)) {
			out->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}
		out->vf_count = get_total_vfs(in->pf_bdf);
		out->result  = GIM_OK;
		out->size    = sizeof(struct get_total_vfs_output);
		break;
	}

	case GIM__SRIOV__ENABLE: {
		struct enable_sriov_input  *in  =
				(struct enable_sriov_input *)command;
		struct output *out = (struct output *)output_buf;

		if (in->size != sizeof(struct enable_sriov_input)) {
			out->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}

		out->result = enable_sriov(in->pf_bdf, in->vfs_count);
		out->size   = sizeof(struct output);
		break;
	}

	case GIM__SRIOV__DISABLE: {
		struct disable_sriov_input *in  =
				(struct disable_sriov_input *)command;
		struct output *out = (struct output *)output_buf;

		if (in->size != sizeof(struct disable_sriov_input)) {
			out->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}

		out->result = disable_sriov(in->pf_bdf);
		out->size   = sizeof(struct output);
		break;
	}

	case GIM__VF__GET_AVAILABLE_VF_COUNT: {
		struct get_available_vf_count_input  *in  =
			(struct get_available_vf_count_input  *)command;
		struct get_available_vf_count_output *out =
			(struct get_available_vf_count_output *)output_buf;

		if (in->size != sizeof(struct get_available_vf_count_input)) {
			out->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}
		out->vf_count = get_available_vf_count(in->pf_bdf);
		out->result  = GIM_OK;
		out->size    = sizeof(struct get_available_vf_count_output);
		break;
	}

	case GIM__VF__ENUMERATE_AVAILABLE_VFS: {
		struct enumerate_available_vfs_input  *in  =
			    (struct enumerate_available_vfs_input *)command;
		struct enumerate_available_vfs_output *out =
			    (struct enumerate_available_vfs_output *)output_buf;

		if (in->size != sizeof(struct enumerate_available_vfs_input)) {
			output->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}

		out->vf_count = enumerate_available_vfs(in->pf_bdf,
			(SYS_BUFFER_SIZE - output->size) / sizeof(uint32_t),
			out->vf_bdfs);
		out->result  = GIM_OK;
		if (out->vf_count > 0)
			out->size =
				sizeof(struct enumerate_available_vfs_output)
				+ (out->vf_count - 1) * sizeof(uint32_t);
		else
			out->size =
				sizeof(struct enumerate_available_vfs_output)
				- sizeof(uint32_t);

		break;
	}

	case GIM__VF__ENUMERATE_VFS: {
		struct enumerate_vfs_input  *in  =
				(struct enumerate_vfs_input *)command;
		struct enumerate_vfs_output *out =
				(struct enumerate_vfs_output *)output_buf;

		if (in->size != sizeof(struct enumerate_vfs_input)) {
			output->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}
		out->vf_count = enumerate_all_vfs(in->pf_bdf,
					(SYS_BUFFER_SIZE - output->size) /
					sizeof(struct vf_info), out->vf_info);
		out->result  = GIM_OK;
		if (out->vf_count > 0)
			out->size = sizeof(struct enumerate_vfs_output) +
				(out->vf_count - 1) * sizeof(struct vf_info);
		else
			out->size = sizeof(struct enumerate_vfs_output) -
						sizeof(struct vf_info);
		break;
	}

	/* used by QEMU */
	case GIM__VF__ALLOC: {
		struct vf_alloc_input  *in  = (struct vf_alloc_input *)command;
		struct vf_alloc_output *out =
					(struct vf_alloc_output *)output_buf;

		if (in->size != sizeof(struct vf_alloc_input)) {
			output->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}

		out->vf_bdf = alloc_vf(in->pf_bdf, in->domid,
					in->pid, in->fb_size, in->sched_level);
		if (out->vf_bdf)
			out->result = GIM_OK;
		else
			out->result = GIM_ERROR; /* no available VF */
		break;
	}

	case GIM__VF__ALLOC_WITH_VBDF: {
		struct vf_alloc_with_vbdf_input  *in  =
				(struct vf_alloc_with_vbdf_input *)command;
		 struct vf_alloc_with_vbdf_output *out =
				(struct vf_alloc_with_vbdf_output *)output_buf;

		if (in->size != sizeof(struct vf_alloc_with_vbdf_input)) {
			output->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}

		alloc_vf_from_bdf(in->vf_bdf, in->domid,
				in->pid, in->fb_size, in->sched_level);
		out->result  = GIM_OK;
		break;
	}

	case GIM__VF__SET_MMR_INFO: {
		struct vf_mmr_info *in = (struct vf_mmr_info *)command;

		if (in->size != sizeof(struct vf_mmr_info)) {
		output->result = GIM_INVALIDATE_INPUT_SIZE;
		return GIM_INVALIDATE_INPUT_SIZE;
		}
		map_mmr(in->vf_bdf, in->physcal_add, in->mmr_size);
		break;
	}

	case GIM__VF__FREE: {
		struct vf_free_input  *in  = (struct vf_free_input *)command;

		if (in->size != sizeof(struct vf_free_input)) {
			output->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}

		free_vf_from_bdf(in->vf_bdf);
		break;
	}

	case GIM__DOMAIN__SHUTDOWN: {
		struct domain_shutdown_input  *in  =
				(struct domain_shutdown_input *)command;

		if (in->size != sizeof(struct domain_shutdown_input)) {
			output->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}

		free_vf_from_domid(in->domid);
		break;
	}

	case GIM__VF__SET_PID:
		break;

	case GIM__VF__SET_FRAME_BUFFER:
		break;

	case GIM__VF__SET_SCHEDULE_LEVEL:
		break;

	case GIM__VF__SET_ACPI_EVENT:
		break;

	case GIM__FB__GET_MAX_SLOT: {
		struct get_max_fb_slot_input  *in  =
				(struct get_max_fb_slot_input *)command;
		struct get_max_fb_slot_output *out =
				(struct get_max_fb_slot_output *)output_buf;

		if (in->size != sizeof(struct get_max_fb_slot_input)) {
			output->result = GIM_INVALIDATE_INPUT_SIZE;
			return GIM_INVALIDATE_INPUT_SIZE;
		}

		out->max_fb_slot_size = get_max_fb_slot(in->pf_bdf);
		out->result  = GIM_OK;
		break;
	}

	case GIM__FB__ALLOC:
		break;

	case GIM__FB__FREE:
		break;

	case GIM__SCHEDULER__PAUSE: {
		struct pause_scheduler_input *in =
			(struct pause_scheduler_input *)command;

		pause_sched(in->pf_bdf);
		break;
	}

	case GIM__SCHEDULER__RESUME: {
		struct resume_scheduler_input *in  =
			(struct resume_scheduler_input *)command;

		start_scheduler(in->pf_bdf);
		break;
	}

	case GIM__SCHEDULER__GET_ACTIVE_FUNCTIONS:
		break;

	case GIM__SCHEDULER__INIT:
		break;

	case GIM__SCHEDULER__IDLE:
		break;

	case GIM__SCHEDULER__SAVE:
		break;

	case GIM__SCHEDULER__LOAD:
		break;

	case GIM__SCHEDULER__RUN:
		break;

	case GIM__SCHEDULER__CONTEXT_SWITCH: {
		struct context_switch_input  *in =
				(struct context_switch_input *)command;

		do_world_switch(in->pf_bdf);
		break;
	}

	case GIM__SCHEDULER__GET_CUR_VF:
		gim_info("GIM__SCHEDULER__GET_CUR_VF - not yet supported\n");
		break;

	case GIM__DEBUG__MANUAL_SWITCH: {
		struct manual_switch_input *in =
				(struct manual_switch_input *)command;

		trgger_single_switch(in->pf_bdf);
		break;
	}

	case GIM__DEBUG__ENABLE_PREEMPTION: {
		struct enable_preemption_input  *in =
				(struct enable_preemption_input *)command;

		enable_preemption(in->pf_bdf, in->enable);
		break;
	}

	case GIM__DEBUG__SEND_SIGNAL: {
		struct send_signal_input  *in =
				(struct send_signal_input *)command;

		send_signal(in->vm_id, in->signal_id);
		break;
	}

	case GIM__RESET__VF_FLR: {
		uint32_t bdf = command->data[0];

		gim_info("GIM__RESET__VF_FLR for BDF 0x%04x - not yet supported\n",
					bdf);
		break;
	}

	case GIM__RESET__PF_FLR:
		/* uint32_t bdf = command->data[0]; */
		gim_info("GIM__RESET__PF_FLR - not yet supported\n");
		break;

	case GIM__RESET__PF_FLR_SOFT:
		/* uint32_t bdf = command->data[0]; */
		gim_info("GIM__RESET__PF_FLR_SOFT - not yet supported\n");
		break;

	case GIM__RESET__ENABLE_BUS_MASTER:
		gim_info("GIM__RESET__ENABLE_BUS_MASTER - not yet supported\n");
		break;

	case GIM__RESET__DISABLE_BUS_MASTER:
		gim_info("GIM__RESET__DISABLE_BUS_MASTER - not yet supported\n");
		break;

	case GIM__RESET__IS_ASIC_HUNG:
		gim_info("GIM__RESET__IS_ASIC_HUNG - not yet supported\n");
		break;

	case GIM__RESET__NOTIFY_RESET: {
		uint32_t bdf = command->data[0];

		gim_info("GIM__RESET__NOTIFY_RESET for BDF 0x%04x - not yet supported\n",
				bdf);
		break;
	}

	}
	return true;
}


uint32_t respond_interface_functions(char *buf)
{
	struct output *output = (struct output *)output_buf;

	memcpy(buf, output,
	((output->size < SYS_BUFFER_SIZE) ? output->size : SYS_BUFFER_SIZE));
	return output->size;
}

