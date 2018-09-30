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

#ifndef _GPU_IOV_MODULE__COMMAND_H
#define _GPU_IOV_MODULE__COMMAND_H

/****************************************************************************
* GIM command version
*****************************************************************************/

#define GIM_HEADER_MAJOR_VER                 0x0001
#define GIM_HEADER_MINOR_VER                 0x0000
#define GIM_HEADER_VER ((GIM_HEADER_MAJOR_VER << 16) | GIM_HEADER_MINOR_VER)

/****************************************************************************
* GIM Return Codes
*****************************************************************************/

#define GIM_OK                                  0x00000000
#define GIM_ERROR                               0x00000001
#define GIM_NOT_SUPPORTED                       0x00000002
#define GIM_ERR_INVALID_SIZE                    0x00000003
#define GIM_NOT_FOUND                           0x00000004
#define GIM_INVALIDATE_COMMAND_SIZE             0x00000005
#define GIM_INVALIDATE_HEADER_SIZE              0x00000006
#define GIM_INVALIDATE_INPUT_SIZE               0x00000007
#define GIM_INVALIDATE_VERSION                  0x00000008
#define GIM_INVALIDATE_VFS_COUNT                0x00000009
#define GIM_INVALIDATE_OPTION                   0x0000000a


/****************************************************************************
* GIM command
*****************************************************************************/
#define GIM__MA__GET_OPTION                     0x00
#define GIM__MA__SET_OPTION                     0x01
#define GIM__MA__GET_PF_COUNT                   0x02
#define GIM__MA__ENUMERATE_PFS                  0x03

#define GIM__SRIOV__GET_TOTAL_VF_COUNT          0x10
#define GIM__SRIOV__ENABLE                      0x11
#define GIM__SRIOV__DISABLE                     0x12
#define GIM__DOMAIN__SHUTDOWN                   0x18

#define GIM__VF__ALLOC_WITH_VBDF                0x19
#define GIM__VF__GET_AVAILABLE_VF_COUNT         0x20
#define GIM__VF__ENUMERATE_AVAILABLE_VFS        0x21
#define GIM__VF__ALLOC                          0x22
#define GIM__VF__FREE                           0x23
#define GIM__VF__SET_PID                        0x24
#define GIM__VF__SET_FRAME_BUFFER               0x25
#define GIM__VF__SET_SCHEDULE_LEVEL             0x26
#define GIM__VF__SET_ACPI_EVENT                 0x27
#define GIM__VF__ENUMERATE_VFS                  0x28
#define GIM__VF__SET_MMR_INFO                   0x29

#define GIM__FB__GET_MAX_SLOT                   0x30
#define GIM__FB__ALLOC                          0x31
#define GIM__FB__FREE                           0x32

#define GIM__SCHEDULER__PAUSE                   0x40
#define GIM__SCHEDULER__RESUME                  0x41
#define GIM__SCHEDULER__GET_ACTIVE_FUNCTIONS    0x42
#define GIM__SCHEDULER__INIT                    0x43
#define GIM__SCHEDULER__IDLE                    0x44
#define GIM__SCHEDULER__SAVE                    0x45
#define GIM__SCHEDULER__LOAD                    0x46
#define GIM__SCHEDULER__RUN                     0x47
#define GIM__SCHEDULER__CONTEXT_SWITCH          0x48
#define GIM__SCHEDULER__GET_CUR_VF              0x49


#define GIM__DEBUG__MANUAL_SWITCH           0x60
#define GIM__DEBUG__ENABLE_PREEMPTION       0x61
#define GIM__DEBUG__SEND_SIGNAL             0x62

/*
 * Control VF/PF reset states
 */
#define GIM__RESET__VF_FLR                  0x73
#define GIM__RESET__PF_FLR                  0x74
#define GIM__RESET__PF_FLR_SOFT             0x75
#define GIM__RESET__ENABLE_BUS_MASTER       0x76
#define GIM__RESET__DISABLE_BUS_MASTER      0x77
#define GIM__RESET__IS_ASIC_HUNG            0x78
#define GIM__RESET__NOTIFY_RESET            0x79

#pragma pack(push, 1)

#define InputHeader     \
	uint32_t size;      \
	uint32_t version;   \
	uint32_t command

#define OutputHeader	\
	uint32_t size;      \
	uint32_t result

/* General structure for command.*/
struct command {
	InputHeader;
	uint32_t data[1];
};

/*General structure for output.*/
struct output {
	OutputHeader;
	uint32_t data[1];
};

/*GIM__MA__GET_OPTION*/
struct get_option_input {
	InputHeader;
	uint32_t index;
};

/*Output*/
struct get_option_output {
	OutputHeader;
	uint32_t value;
};

/* GIM__MA__SET_OPTION */
/* Input */
struct set_option_input {
	InputHeader;
	uint32_t index;
	uint32_t value;
};


/* GIM__MA__GET_PF_COUNT */
/* Input: none */
/* Output */
struct get_pf_count_output {
	OutputHeader;
	uint32_t pf_count;
};

/* GIM__MA__ENUMERATE_PFS */
/* Input: standard */
/* Output */
struct enumerate_pfs_output {
	OutputHeader;
	uint32_t pf_count;
	uint32_t pf_bdfs[1];
};

/* GIM__SRIOV__GET_TOTAL_VF_COUNT */
/* Input */
struct get_total_vfs_input {
	InputHeader;
	uint32_t pf_bdf;
};

/* Output */
struct get_total_vfs_output {
	OutputHeader;
	uint32_t vf_count;
};

/* GIM__SRIOV__ENABLE */
/* Input */
struct enable_sriov_input {
	InputHeader;
	uint32_t pf_bdf;
	uint32_t vfs_count;
};


/* GIM__SRIOV__DISABLE */
/* Input */
struct disable_sriov_input {
	InputHeader;
	uint32_t pf_bdf;
};

/* GIM__VF__GET_AVAILABLE_VF_COUNT */
/* Input */
struct get_available_vf_count_input {
	InputHeader;
	uint32_t pf_bdf;
};

/* Output */
struct get_available_vf_count_output {
	OutputHeader;
	uint32_t vf_count;
};

/* GIM__VF__ENUMERATE_AVAILABLE_VFS */
/* Input */
struct enumerate_available_vfs_input {
	InputHeader;
	uint32_t pf_bdf;
};

/* Output */
struct enumerate_available_vfs_output {
	OutputHeader;
	uint32_t vf_count;
	uint32_t vf_bdfs[1];
};

/* GIM__VF__ENUMERATE_VFS */
/* Input */
struct enumerate_vfs_input {
	InputHeader;
	uint32_t pf_bdf;
};

/* Output */
struct vf_info {
	uint32_t bdf;
	uint32_t state       : 1;
	uint32_t sched_level : 2;
	uint32_t reserved    : 29;
	uint32_t fb_size;        /* Frame buffer size in MB */
	int dom_id;
	int func_id;
	uint32_t hw_fb_size;
	uint32_t hw_fb_start;
};

struct enumerate_vfs_output {
	OutputHeader;
	uint32_t vf_count;
	struct vf_info vf_info[1];
};

/* GIM__VF__ALLOC */
/* Input */
struct vf_alloc_input {
	InputHeader;
	uint32_t pf_bdf;
	int domid;
	int pid;
	int fb_size;
	int sched_level;
};

/* Output */
struct vf_alloc_output {
	OutputHeader;
	uint32_t vf_bdf;
};


/* GIM__VF__ALLOC_WITH_VFID */
/* Input */
struct vf_alloc_with_vbdf_input {
	InputHeader;
	uint32_t pf_bdf;
	uint32_t vf_bdf;
	int domid;
	int pid;
	int fb_size;
	int sched_level;
};

/* Output */
struct vf_alloc_with_vbdf_output {
	OutputHeader;
};

/* For testing */
/* Input */
struct vf_alloc_input_test {
	InputHeader;
	uint32_t vf_bdf; /* alloc from vf_bdf */
	int domid;
	int pid;
};

/* Output */
struct vf_alloc_output_test {
	OutputHeader;
};

struct vf_mmr_info {
	InputHeader;
	uint32_t vf_bdf; /* alloc from vf_bdf */
	unsigned long long    physcal_add;
	unsigned long long     mmr_size;
};

/* GIM__VF__FREE */
/* Input */
struct vf_free_input {
	InputHeader;
	uint32_t vf_bdf;
};


/* GIM__DOMAIN__SHUTDOWN */
/* Input */
struct domain_shutdown_input {
	InputHeader;
	uint32_t domid;
};

/* GIM__VF__SET_PID */
/* Input */
struct set_pid_input {
	InputHeader;
	uint32_t vf_bdf;
	uint32_t pid;
};

/* GIM__VF__SET_FRAME_BUFFER */
/* Input */
struct set_fb_input {
	InputHeader;
	uint32_t vf_bdf;
	uint32_t fb_offset;
};

/* GIM__VF__SET_SCHEDULE_LEVEL */
/* Input */
struct set_schedule_level_input {
	InputHeader;
	uint32_t vf_bdf;
	uint32_t sched_level;
};

/* GIM__VF__SET_ACPI_EVENT */
/* Input */
struct set_acpi_event_input {
	InputHeader;
	uint32_t vf_bdf;
	uint32_t acpi_event;
};

/* GIM__FB__GET_MAX_SLOT */
/* Input */
struct get_max_fb_slot_input {
	InputHeader;
	 uint32_t pf_bdf;
};

/* Output */
struct get_max_fb_slot_output {
	OutputHeader;
	uint32_t max_fb_slot_size;
};

/* GIM__FB__ALLOC */
/* Input */
struct fb_alloc_input {
	InputHeader;
	uint32_t pf_bdf;
	uint32_t fb_size_required;
};

/* Output */
struct fb_alloc_output {
	OutputHeader;
	uint32_t fb_offset;
};

/* GIM__FB__FREE */
/* Input */
struct fb_free_input {
	InputHeader;
	uint32_t pf_bdf;
	uint32_t fb_offset;
};

/* GIM__SCHEDULER__PAUSE */
/* Input */
struct pause_scheduler_input {
	InputHeader;
	uint32_t pf_bdf;
};

/* GIM__SCHEDULER__RESUME */
/* Input */
struct resume_scheduler_input {
	InputHeader;
	uint32_t pf_bdf;
};

/* GIM__SCHEDULER__GET_ACTIVE_FUNCTIONS */
/* Input */
struct get_active_functions_input {
	InputHeader;
	uint32_t pf_bdf;
};

/* Output */
struct get_active_functions_output {
	OutputHeader;
	uint32_t active_funcs;
};

/* GIM__SCHEDULER__INIT */
/* Input */
struct init_input {
	InputHeader;
	uint32_t vf_bdf;
};

/* GIM__SCHEDULER__IDLE */
/* Input */
struct idle_input {
	InputHeader;
	uint32_t vf_bdf;
};

/* GIM__SCHEDULER__SAVE */
/* Input: */
struct save_input {
	InputHeader;
	uint32_t vf_bdf;
};

/* GIM__SCHEDULER__LOAD */
/* Input */
struct load_input {
	InputHeader;
	uint32_t vf_bdf;
};

/* GIM__SCHEDULER__RUN */
/* Input */
struct run_input {
	InputHeader;
	uint32_t vf_bdf;
};

/* GIM__SCHEDULER__CONTEXT_SWITCH */
/* Input */
struct context_switch_input {
	InputHeader;
	uint32_t pf_bdf;
};

/* GIM__DEBUG__MANUAL_SWITCH */
/* Input */
struct manual_switch_input {
	InputHeader;
	uint32_t pf_bdf;
};

/* GIM__DEBUG__ENABLE_PREEMPTION */
/* Input */
struct enable_preemption_input {
	InputHeader;
	uint32_t pf_bdf;
	uint32_t enable;
};

/* GIM__DEBUG__SEND_SIGNAL */
/* Input */
struct send_signal_input {
	InputHeader;
	uint32_t vm_id;
	uint32_t signal_id;
};

#pragma pack(pop)

/****************************************************************************
* VF's signal number (real-time signals are in the range of 33 to 64
*****************************************************************************/
#define SIG_VF_EXCLUSIVE_MMIO 59
#define SIG_VF_TRAP_MMIO 60
#define SIG_VF_STOP_MMR 61
#define SIG_VF_ACTIVE 62
#define SIG_VF_INACTIVE 63


/****************************************************************************
* GIM config file
*****************************************************************************/
#define GIM_CONFIG_PATH "/etc/gim_config"

/****************************************************************************
* Frame Buffer Partition Options
*****************************************************************************/
#define FB_OPTION__KEY "fb_option"
#define FB_PARTITION__START   0
#define FB_PARTITION__STATIC  0
#define FB_PARTITION__DYNAMIC 1
#define FB_PARTITION__DEFAULT FB_PARTITION__STATIC
#define FB_PARTITION__MAX     FB_PARTITION__DYNAMIC

/****************************************************************************
* Scheduler Options
*****************************************************************************/
#define SCHEDULE_OPTION__KEY "sched_option"
#define SCHEDULER__START               0
#define SCHEDULER__ROUND_ROBIN_SOLID   0
#define SCHEDULER__PREDICT_PERF        1
#define SCHEDULER__ROUND_ROBIN_LIQUID  2
#define SCHEDULER__DEFAULT             SCHEDULER__ROUND_ROBIN_SOLID
#define SCHEDULER__MAX                 SCHEDULER__ROUND_ROBIN_LIQUID

/****************************************************************************
* VF number Options
*****************************************************************************/
#define VF_NUMBER__KEY "vf_num"
#define VF_NUMBER__START               0
#define VF_NUMBER__DEFAULT             0
#define VF_NUMBER__MAX                 16

/****************************************************************************
* PF frame buffer size(in MegaByess) Options
*****************************************************************************/
#define PF_FB__KEY "pf_fb"
#define PF_FB__START                    0
#define PF_FB__DEFAULT                  PF_FB__START
#define PF_FB__MAX                      65536

/****************************************************************************
* VF frame buffer size(in MegaByess) Options
*****************************************************************************/
#define VF_FB__KEY "vf_fb"
#define VF_FB__START                    0
#define VF_FB__DEFAULT                  VF_FB__START
#define VF_FB__MAX                      65536

/****************************************************************************
* Scheduler time quanta interval in milliseconds Options
* Set MAX to 2 seconds.  A more realistic MAX would be
* in the 20msec-30msec range.
* Allow higher for debugging and testing.
*****************************************************************************/
#define SCHED_INTERVAL__KEY "sched_interval"
#define SCHED_INTERVAL__START		0
#define SCHED_INTERVAL__DEFAULT		SCHED_INTERVAL__START
#define SCHED_INTERVAL__MAX		2000

/****************************************************************************
* Scheduler time quanta interval in microseconds Options
* Set MAX to 999 microseconds.  Especially for performance tuning
*****************************************************************************/
#define SCHED_INTERVAL_US__KEY "sched_interval_us"
#define SCHED_INTERVAL_US__START		0
#define SCHED_INTERVAL_US__DEFAULT		SCHED_INTERVAL_US__START
#define SCHED_INTERVAL_US__MAX		999

/****************************************************************************
* Some miscellaneous debug parameters
*****************************************************************************/

#define FB_CLEAR__KEY "fb_clear"
#define FB_CLEAR__START			0
#define FB_CLEAR__DEFAULT		0
#define FB_CLEAR__MAX			1

/****************************************************************************
* Filename of /sysfs
*****************************************************************************/
#define FILE_SRIOV "/sys/bus/pci/drivers/gim/sriov"

/****************************************************************************
* The buffer size for /sysfs is one page
*****************************************************************************/
#define SYS_BUFFER_SIZE 0x1000

#endif


