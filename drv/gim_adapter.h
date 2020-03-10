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

#ifndef _GPU_IOV_MODULE__ADAPTER_H
#define _GPU_IOV_MODULE__ADAPTER_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/version.h>
#include "gim_fb.h"
#include "gim_pci.h"
#include "gim_flr.h"
#include "gim_pci_config.h"

/* Macro for debug */
#define SPIN_LOCK_DEBUG

#ifdef SPIN_LOCK_DEBUG
void __amd_spin_lock(spinlock_t *lock, const char *function, int line);

#define amd_spin_lock(x)     \
		__amd_spin_lock(x, __func__, __LINE__)
#else
#define amd_spin_lock(x) spin_lock(x)
#endif

#ifdef CONFIG_MMIO_QEMU_SECURITY
/*
 *  * 0x5320/40 - MAILBOX registers
 *  * 0x540C    - BIF_IOV_FUNC_IDENTIFIER
 *  * 0x5480    - HDP_MEM_COHERENCY_FLUSH_CTRL
 *  * Syntax requires /4 for a single register specifying a range of 4 bytes
 *    offset
 **/
/* access range indicates accessible mmio range when mmio is trapped*/
#define ACCESS_RANGE  "0x14c8/0x28,0x1503/0x4,0x1520/0x4,0x150a/0x4"
/* unaccess range indicates unaccessible mmio range when mmio is untrapped*/
#define UNACCESS_RANGE  \
	"0x1480/0x28,0x1610/0x4,0x14a1/0x30,0x14b8/0x4,0x14e5/0x4,0xf800/0x1000"

enum GIM_MMIO_STATUS {
	GIM_MMIO_BLOCK = 0,
	GIM_MMIO_UNBLOCK,
};

#define WAIT_COMPLETE_TIMEOUT 30 /* ms */
#endif

/* ARI mode*/
#define PF_BUS        1
#define PF_BUS_PLUS_1 2

#define ARI_MODE_MASK 0x00600000
#define ARI_MODE_0    0x00000000
#define ARI_MODE_1    0x00200000
#define ARI_MODE_2    0x00400000
#define ARI_MODE_3    0x00600000

union physical_address {
	struct {
		uint32_t	low_part;
		int32_t		high_part;
	} u;

	int64_t		quad_part;
};


#define FLR_REASON_CLEAR          0
#define FLR_REASON_EXT_TRIGGER    1
#define FLR_REASON_FAILED_INIT    2
#define FLR_REASON_FAILED_IDLE    3
#define FLR_REASON_FAILED_SAVE    4
#define FLR_REASON_FAILED_LOAD    5
#define FLR_REASON_FAILED_RUN     6
#define FLR_REASON_FAILED_GFX_IDLE 7
#define FLR_REASON_EXCLUSIVE_TIMEOUT 8


#define MAX_MSG_LEN  64

/* Kernel >= 5.0 removed some old timespec methods */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0)
#define TIMESPECTYPE timespec
#define TIMEVALTYPE  timeval
#define TIMESPEC_ADD(x,y)   timespec_add((x),(y))
#define TIMESPEC_SUB(x,y)   timespec_sub((x),(y))
#define GETNSTIMEOFDAY(x)   getnstimeofday(x)
#else
#define TIMESPECTYPE timespec64
#define TIMEVALTYPE  timespec64
#define TIMESPEC_ADD(x,y)   timespec64_add((x),(y))
#define TIMESPEC_SUB(x,y)   timespec64_sub((x),(y))
#define GETNSTIMEOFDAY(x)   ktime_get_real_ts64(x)
#define do_gettimeofday(x)  ktime_get_real_ts64(x)
#endif

struct op_time_log {
	struct TIMEVALTYPE init_start;
	struct TIMEVALTYPE init_end;
	struct TIMEVALTYPE finish_start;
	struct TIMEVALTYPE finish_end;
	int reset_count;
	struct TIMEVALTYPE reset_time;
	/* we need accurate time here*/
	struct TIMESPECTYPE active_last_tick;
	struct TIMESPECTYPE active_time;
};

enum amdgim_option_mode {
	AMDGIM_OPTION_MODE_ZZZ,  /* zero zero zero */
	AMDGIM_OPTION_MODE_OSG,  /* offset size gfx */
	AMDGIM_OPTION_LEN,
};

struct amdgim_user_option {
	bool valid;
	enum amdgim_option_mode mode;
	unsigned int fb_offset;
	unsigned int fb_size;
	unsigned int gfx_time_divider;
	unsigned int vce_time_level;
	unsigned int uvd_time_level;
	unsigned int gfx_time_quota; /* add this to save time in scheduling */
};

struct vf_load_status {
	uint64_t cp_wptr_poll_pf_offset;
	uint64_t sdma0_wptr_poll_pf_offset;
	uint64_t sdma1_wptr_poll_pf_offset;
	uint64_t cpc_wptr_poll_pf_offset[48];
	uint64_t cpc_rptr_report_pf_offset[48];
	uint32_t last_cpc_wptr_completed[48];
	uint32_t cpc_wptr_poll_count;
	uint32_t cpc_rptr_poll_count;
	uint32_t last_cp_wptr_completed;
	uint32_t last_cp_rptr_completed;
	uint32_t last_sdma0_wptr_completed;
	uint32_t last_sdma1_wptr_completed;
	uint32_t last_grbm_status;
	uint32_t last_srbm_status2;
	uint32_t count_switch_total;
	uint32_t count_switch_skipped;
	bool	need_work;
	uint32_t cpc_index[48];
};

struct function {
	struct adapter *adapt;
	uint32_t            is_available;
	uint32_t            is_scheduled;
	uint32_t            scheduled_num;

	uint32_t            bdf;
	struct pci_dev *pci_dev;
	struct partition *fb_partition;
	int               func_id;
	int               dom_id;
	int               qemu_pid;
	int               sched_level;

	uint32_t  *mmr_base;
	uint32_t       mmr_size;
	uint32_t    *reg_sav_res_data;
	uint32_t    *reg_sav_res_offset;
	uint32_t	sav_res_list_size;

	void *doorbell;
	void *fb_va;
	union physical_address fb_pa;
	struct flr_state flr_state;
	int             needs_flr;
	int             in_flr;
	int             flr_reason_code;
	uint32_t        msg_data[4];
	char            text_msg[MAX_MSG_LEN+1];
	uint32_t        msg_len;
	uint32_t        rptr;
	uint32_t        wptr;

	struct vf_load_status status;
#ifdef CONFIG_MMIO_QEMU_SECURITY
	/* determine whether to trap mmio for security */
	struct completion unblock_complete;
	struct completion block_complete;
	enum GIM_MMIO_STATUS mmio_status;
#endif

	/* Operation Log for GPU Monitoring */
	struct op_time_log time_log;
	/* User option of this function */
	struct amdgim_user_option user_option;
};


enum { MAX_VIRTUAL_FUNCTIONS = 16 };
enum { MAX_PHYSICAL_FUNCTIONS = 1 };
enum { TOTAL_FUNCTIONS = MAX_VIRTUAL_FUNCTIONS + MAX_PHYSICAL_FUNCTIONS };

enum { PF_INDEX   = 0,
	   VF0_INDEX  = 1,
	   VF1_INDEX  = VF0_INDEX  + 1,
	   VF2_INDEX  = VF1_INDEX  + 1,
	   VF3_INDEX  = VF2_INDEX  + 1,
	   VF4_INDEX  = VF3_INDEX  + 1,
	   VF5_INDEX  = VF4_INDEX  + 1,
	   VF6_INDEX  = VF5_INDEX  + 1,
	   VF7_INDEX  = VF6_INDEX  + 1,
	   VF8_INDEX  = VF7_INDEX  + 1,
	   VF9_INDEX  = VF8_INDEX  + 1,
	   VF10_INDEX = VF9_INDEX  + 1,
	   VF11_INDEX = VF10_INDEX + 1,
	   VF12_INDEX = VF11_INDEX + 1,
	   VF13_INDEX = VF12_INDEX + 1,
	   VF14_INDEX = VF13_INDEX + 1,
	   VF15_INDEX = VF14_INDEX + 1,
	   VF_MAX     = VF15_INDEX,
	 };

enum { TIMER_SLICE_IN_MS = 1 };

enum { BAR__FRAME_BUFFER = 0,
	   BAR__MMIO         = 2,
	   BAR__DOORBELL     = 4,
	   BAR__IOPORT       = 5};

enum { FB__RESERVED_CSA_IN_1M = 8 };
enum { FB__SIZE_IN_16M     = 16 };
enum { FB__CSA_SIZE_IN_256K = 1  };

#define TO_MBYTES(x) ((x)>>20)
#define TO_256KBYTES(x) ((x)>>18)
#define MBYTES_TO_BYTES(x) (((kcl_type_u64)x)<<20)


struct gpu_iov {
	struct partition   fb_partion[MAX_VIRTUAL_FUNCTIONS];
};

struct function_list_node {
	struct function *func;
	struct function_list_node *next;
	struct function_list_node *pre;
	int    inuse;
};

struct slot_list_node {
	struct slot slot;
	struct slot_list_node *next;
};

/* IH */
enum idh_event {
	IDH_CLR_MSG_BUF = 0,
	IDH_READY_TO_ACCESS_GPU = 1,
	IDH_FLR_NOTIFICATION,
	IDH_FLR_NOTIFICATION_COMPLETION,

	IDH_REQ_GPU_INIT_ACCESS = 1,
	IDH_REL_GPU_INIT_ACCESS,
	IDH_REQ_GPU_FINI_ACCESS,
	IDH_REL_GPU_FINI_ACCESS,
	IDH_REQ_GPU_RESET_ACCESS,

	/* This access is invalid so would be aborted. */
	IDH_REQ_GPU_INVALID_ACCESS,
	IDH_TEXT_MESSAGE = 255
};

struct iv_ring_entry {
	/* DWORD[0] */
	unsigned int source_id               : 8;
	unsigned int                        : 8;
	unsigned int time_stamp_48_high       : 16;
	/* DWORD[1] */
	unsigned int source_data             : 28;
	unsigned int                        : 4;
	/* DWORD[2] */
	unsigned int ring_id                 : 8;
	unsigned int vm_id                   : 8;
	unsigned int pas_id                  : 16;
	/* DWORD[3] */
	unsigned int time_stamp_48_low        : 32;
};

struct iv_ring_memory {
	unsigned int    page_cnt;
	unsigned long   *page_list;
};

struct interrupt_handler {
	/* Physical address of IV ring */
	union physical_address                ivr_ma;
	/* pointer to IV ring */
	struct iv_ring_entry                 *ivr_va;
	/* Ring buffer size in Bytes */
	unsigned int                    ivr_size_in_bytes;
	unsigned int                    ivr_alloc_size_in_bytes;
	unsigned int                    ivr_num_entries;
	unsigned int                    ivr_rptr;
	unsigned int                    ivr_wptr;
	unsigned int                    ivr_rptr_reg;
	unsigned int                    ivr_wptr_reg;
	union physical_address                ivr_wptr_wa;
	volatile unsigned int           *ivr_wptr_wb;
	/* The memory block for iv ring */
	struct iv_ring_memory                iv_ring;
	/* ih door bell pointer */
	volatile unsigned int           *rptr_doorbell;
	unsigned int                    rptr_doorbell_offset;
	unsigned int                    ring_size_log2;
	unsigned char                   is_overflow;
	/* 1: use mm reg to update rptr
	 * 0: use doorbell to update rptr
	 */
	unsigned char                   is_legacy_rptr;
};

/* Type definition for Interrupt Handling Routine */
typedef int (*IhRoutine_t)(void *pContext);

struct work_task {
	unsigned int func_id;
	enum idh_event event;
	/* 0 = task is Idle
	 * 1 = task is waiting to be serviced
	 */
	int waiting_to_execute;
};

struct adapter {
	int      adapt_id;
	struct function pf;
	struct function vfs[MAX_VIRTUAL_FUNCTIONS];
	/* Need these statically defined as we can't vmfree
	 * them from interrupts routine.
	 */
	struct function_list_node fn_list_nodes[MAX_VIRTUAL_FUNCTIONS];

	/* total number of enabled vfs; for static frame buffer schema.this
	 * could be less than total_vfs
	 */
	uint32_t     enabled_vfs;

	/* current available vfs. Max = enabled_vfs */
	uint32_t     available_vfs;
	uint32_t     total_vfs;

	struct pci_dev *p2p_bridge_dev;

	spinlock_t mailbox_lock;

	/* save fb_base and fb_size for gpu monitoring use */
	unsigned long long vf_fb_base;
	unsigned long long vf_fb_size;

	/* SR IOV */
	unsigned long vf_bar[6];

	/* GPU IOV */
	uint32_t      active_func;
	/* Total frame buffer available, in units of 1MB. */
	uint32_t      fb_available;
	/*unit 1MB */
	uint32_t      fb_consumed;
	/* 256kb-aligned start address of context save area. */
	uint32_t      context_offset;
	/* (readonly)256kb-aligned, FB required per VF. */
	uint32_t      context_size;

	struct    pci_gpu_iov gpuiov;

	/* frame buffer patition. */
	struct partition    partition[TOTAL_FUNCTIONS];
	struct mutex fb_dynamic_alloc_mutex;
	struct slot_list_node *empty_slot_list;
	uint32_t max_fb_slot;

	struct hrtimer sched_timer; /* Schedule timer. */
	int       quota;            /* time quota. */
	spinlock_t lock;
	struct mutex signal_mutex;
	struct work_struct sched_work;

	/* running functions on a adapter */
	struct function_list_node *running_func_list;
	/* current running function */
	struct function_list_node *curr_running_func;
	struct mutex curr_running_func_mutex;
	spinlock_t ih_lock;

	bool enable_preeption;
	bool single_switch;
	bool switch_to_itself;
	bool scheduler_running;
	unsigned int sched_opt;

	unsigned char *pvbios_image;
	unsigned int  vbios_image_size;
	/* ROM information used by ATOM parser*/
	void                    *rom_info;

	/* IH block */
	struct interrupt_handler            *ih;
	struct work_struct normal_work;
	struct work_struct abnormal_work; /* handle abnormal cases */


	struct work_task irq_tasks[MAX_VIRTUAL_FUNCTIONS];
	int vf_req_gpu_access;
	/* the time when a VF enters full access mode*/
	struct TIMESPECTYPE start_time;
	/* timeout check timer*/
	struct hrtimer timeout_timer;
	/* record the last VF id that owns gpu*/
	int last_gpu_owner;
	/* We are in a pf flr and scheduling needs to be handled
	 * a little differently
	 */
	int is_pf_flr;
	int adp_id;
#ifdef CONFIG_GIM_HEARTBEAT_TIMER
	int world_switch_count;
#endif

	unsigned int vddgfx_state_saved[MAX_VIRTUAL_FUNCTIONS+1];
	unsigned int rlcv_scratch[16][256];
	/*memory for save/restore pf pci cfg when pci hot reset*/
	void *pf_flr_pci_cfg;

	unsigned int sclk_dpm_cap; /* gpu monitor parameter*/
};

struct gim_ucode_info {
	/* Ucode table size */
	unsigned int    ucode_size;
	/* Ucode version string */
	char            *ucode_ver;
	/* Ucode feature version */
	unsigned int    ucode_feature_ver;
	/* Pointer to ucode table */
	unsigned int    *ucode_table;
	/* The offset from start of table */
	unsigned int    jump_table_offset;
	/* The size of jump table */
	unsigned int    jump_table_size;
};

enum { MAX_ADAPTERS_IN_SYSTEM = 8 };

struct named_reg_list {
	uint32_t   idx_index;/* Indirect index register */

	/* If non-zero then this is an indirect register access */
	uint32_t   idx_data;
	uint32_t   offset; /* register offset */
	char      *name;   /* register test name */
};

uint32_t get_adapter_count(void);
struct adapter *get_adapters(void);
struct adapter *get_default_adapter(void);
uint32_t set_new_adapter(struct pci_dev *pdev);
void release_all_adapters(void);
void check_rlc_regs(uint32_t bdf, char *comment);
void run_sdma(uint32_t bdf);
void check_sdma(uint32_t bdf);
struct function_list_node *is_on_run_list(struct function *fn);
uint32_t get_mailbox_index(struct function *func);
uint32_t init_vfs(struct adapter *adapt, uint32_t vfsCount);
int init_frame_buffer_partition(struct adapter *adapt);
int wait_for_gfx_idle(struct function *func);
int init_vf(struct function *func);
int load_vf(struct function *func);
int enable_vf(struct function *func);
int idle_vf(struct function *func);
int save_vf(struct function *func);
int run_vf(struct function *func);
int load_vf(struct function *func);
int load_rlcv_state(struct function *vf);
int save_rlcv_state(struct function *vf);
int stop_current_vf(struct adapter *adapt);
int alloc_new_vf(struct function *func, int dom_id,
		int qemu_pid, int fb_size,
		int sched_level);
void world_switch(struct adapter *adapt);
int switch_vfs(struct function *func, struct function *next_func);
struct function *get_available_VF(struct adapter *adapt);
int free_vf(struct function *func);
int signal_qemu(int pid, int signal_id);
int pause_all_guest(struct adapter *adapt);
int unpause_all_guest(struct adapter *adapt);
int trigger_world_switch(struct adapter *adapt, bool single_switch);
struct function *get_vf(struct adapter *adapt, uint32_t vf_id);

int get_hw_fb_setting(struct adapter *adapt, int vf_num,
			uint32_t *fb_start, uint32_t *fb_size);
int gim_post_vbios(struct adapter *adapt, int post_type);

/* IH */
int adapter_interrupt_register(struct adapter *adapt);
void adapter_interrupt_unregister(struct adapter *adapt);
/* static irqreturn_t adapt_interrupt_handler(int irq, void *dev_id); */
void work_handler(struct work_struct *work);
int ih_execute(IhRoutine_t ih_routine, void *ih_context);
int flr_save_state(uint32_t bdf, struct flr_state *state);
int flr_restore_state(uint32_t bdf, struct flr_state *state);
int init_register_init_state(struct adapter *adapt);
int load_register_init_state(struct adapter *adapt);
int reset_rlcv_state_machine(struct adapter *adapt);
struct function *vfid_to_function(struct adapter *adapt, uint32_t vf_id);
struct function *bdf_to_function(uint32_t bdf);
void send_mailbox_reset_notify_to_vf(uint32_t pf_bdf, int vf_num);
struct function_list_node *alloc_fn_list_node(struct adapter *adapt);
void free_fn_list_node(struct adapter *adapt, struct function_list_node *fln);
struct function_list_node *add_func_to_run_list(struct function *func);
void remove_from_run_list(struct function *func);
void load_vbios(struct function *func);
void set_timer_interrupts(struct adapter *adapt, int enable);
void dump_gpu_status(struct function *func);

int gim_sched_reset(struct adapter *adapt,
			struct function *current_running_vf,
			struct function *next_running_vf,
			int command_status);

int gim_sched_reset_vf(struct adapter *adapt,
			struct function *current_running_vf,
			struct function *next_running_vf,
			int command_status);

int gim_sched_reset_gpu(struct adapter *adapt);
struct TIMESPECTYPE time_elapsed(struct TIMESPECTYPE *ts_start);
void pause_scheduler(struct adapter *adapt);
void resume_scheduler(struct adapter *adapt);
int get_scheduler_time_interval(struct adapter *adapt, struct function *func);
void check_smu(uint32_t bdf, char *comment);
void dump_runlist(struct adapter *adapt);
void *map_vf_fb(struct pci_dev *pdev);
int get_scheduled_func(struct adapter *adapt);
void mark_func_scheduled(struct function *func);
void mark_func_not_scheduled(struct function *func);
void idle_all_adapters(void);

int gim_init_vf_load_status(struct function *func);
int idle_pf(struct adapter *adapt);
int switch_to_pf(struct adapter *adapt);
int clear_vf_fb(struct adapter *adapt, struct function *func);

extern char gim_driver_name[];
extern struct aer_item device_list[];

#endif
