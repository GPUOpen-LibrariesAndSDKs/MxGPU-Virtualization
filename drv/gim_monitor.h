/*
 * Copyright (c) 2017 Advanced Micro Devices, Inc. All rights reserved.
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

#ifndef _GPU_IOV_MODULE__MONITOR_H
#define _GPU_IOV_MODULE__MONITOR_H

#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include "gim_adapter.h"
#include "gim_atombios.h"
#include "gim_atombios_com.h"

#define LITTLEENDIAN_CPU

#define AMDGIM_CDEV_NAME                    "amdgim_monitor"
#define AMDGIM_CLASS_NAME                   "amdgim_monitor_class"
#define AMDGIM_DEVICE_FILE_NAME             "amdgim"

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
#define AMDGIM_MAJOR                        1002
#else
#define AMDGIM_MAJOR			    0
#endif

#define AMDGIM_MINOR_START                  0
#define AMDGIM_MINOR_LEN                    1

#define AMDGIM_STRLEN_VERYSHORT             4
#define AMDGIM_STRLEN_SHORT                 16
#define AMDGIM_STRLEN_NORMAL                32
#define AMDGIM_STRLEN_LONG                  64
#define AMDGIM_STRLEN_VERYLONG              256

#define AMDGIM_BUFFER_SIZE                  4096

#define AMDGIM_GPU_ERROR_MSG_SIZE           AMDGIM_STRLEN_VERYLONG


#define AMDGIM_STR_YES                      "Yes"
#define AMDGIM_STR_NO                       "No"
#define AMDGIM_STR_NONE                     "None"
#define AMDGIM_STR_ACTIVE                   "Active"
#define AMDGIM_STR_AVAILABLE                "Available"
#define AMDGIM_STR_NA                       "N/A"
#define AMDGIM_STR_OFF                      "OFF"

#define AMDGIM_ERROR_STR_INVALID_CMD        "Error: invalid command."
#define AMDGIM_ERROR_STR_UNEXPECTED         "Error: unexpected error."

#define AMDGIM_VF_STATE_ACTIVE              3
#define AMDGIM_VF_STATE_FULLACCESS          2
#define AMDGIM_VF_STATE_AVAILABLE           1

#define AMDGIM_IS_SMC_ACTIVE(intr)          ((intr) > 0x20100)
#define AMDGIM_TONGA_MAX_ACTIVITY           256

#define AMDGIM_DATAEXCHANGE_SIZE_KB         (4)
#define AMDGIM_MECFW_SIZE_KB                (512)
#define AMDGIM_VBIOS_SIZE_KB				(64)

#if 1
/* currently we still use top down fw reservation */
#define AMDGIM_RESERVE_FB_SIZE_KB \
	(AMDGIM_DATAEXCHANGE_SIZE_KB + AMDGIM_MECFW_SIZE_KB)
#else
#define AMDGIM_RESERVE_FB_SIZE_KB \
	(AMDGIM_VBIOS_SIZE_KB + \
	AMDGIM_DATAEXCHANGE_SIZE_KB + \
	AMDGIM_MECFW_SIZE_KB)
#endif

#define AMDGIM_RESERVE_FB_OFFSET_KB			(0)

#define AMDGIM_GET_PCI_DOMAIN(x)            (((x) & 0xffff0000) >> 16)
#define AMDGIM_GET_PCI_BUS(x)               (((x) & 0x0000FF00) >> 8)
#define AMDGIM_GET_PCI_DEVICE(x)            (((x) & 0x000000F8) >> 3)
#define AMDGIM_GET_PCI_FUNCTION(x)          (((x) & 0x00000007) >> 0)


#define AMDGIM_FUNC_ENTRY(f)     (f)
#define AMDGIM_TOSTR_TEMP(buf, tmp) \
	sprintf(buf, "%0.2fC", tmp)
#define AMDGIM_TOSTR_WATT(buf, tmp) \
	sprintf(buf, "%0.2fW", tmp)
#define AMDGIM_TOSTR_PERCENT(buf, tmp) \
	sprintf(buf, "%0.2f\%", tmp)
#define AMDGIM_TOSTR_PCI_SPEED(buf, tmp) \
	sprintf(buf, "%dGT/s", tmp)
#define AMDGIM_TOSTR_PCI_WIDTH(buf, tmp) \
	sprintf(buf, "x%d", tmp)
#define AMDGIM_TOSTR_PCI_WIDTH(buf, tmp) \
	sprintf(buf, "x%d", tmp)
#define AMDGIM_TOSTR_PCI_VOLT(buf, tmp) \
	sprintf(buf, "%0.4fV", tmp)

enum{
	AMDGIM_ASIC_TYPE_TONGA = 0,

	AMDGIM_ASIC_TYPE_UNKNOWN,
	AMDGIM_ASIC_TYPE_LEN
};

enum{
	AMDGIM_ASIC_PCI_DEVICE_ID = 0,
	AMDGIM_ASIC_PCI_VENDOR_ID,
	AMDGIM_ASIC_PCI_REV_ID,
	AMDGIM_ASIC_PCI_LEN
};


enum{
	AMDGIM_ASICINFO_NAME = 0,
	AMDGIM_ASICINFO_CU,
	AMDGIM_ASICINFO_GFX,
	AMDGIM_ASICINFO_VF_PREFIX,
	AMDGIM_ASICINFO_VIDEO_ENCODER,
	AMDGIM_ASICINFO_DPM_CAP,
	AMDGIM_ASICINFO_POWER_CAP,
	AMDGIM_ASICINFO_MAX_VF_NUM,
	AMDGIM_ASICINFO_UNCORR_ECC,
	AMDGIM_ASICINFO_MAX_GFX_CLK,
	AMDGIM_ASICINFO_LEN
};

enum{
	AMDGIM_COMMAND_GPUINFO = 0,
	AMDGIM_COMMAND_GPUVS,
	AMDGIM_COMMAND_GPUBIOS,
	AMDGIM_COMMAND_GPUVF_PF,
	AMDGIM_COMMAND_GPUVF_VF,
	AMDGIM_COMMAND_CLRVFFB,
	AMDGIM_COMMAND_GETVF,
	AMDGIM_COMMAND_RELVF,
	AMDGIM_COMMAND_FLR,
	AMDGIM_COMMAND_HOTLINK_RESET,
	AMDGIM_COMMAND_LEN
};

enum{
	AMDGIM_GPUVF_TOTAL = 0,
	AMDGIM_GPUVF_PF,
	AMDGIM_GPUVF_VF,
	AMDGIM_GPUVF_LEN
};

/* monitor error code */
enum AMDGIM_ERROR_MONITOR {
	AMDGIM_ERROR_MONITOR_SUCCESS = 0,
	AMDGIM_ERROR_MONITOR_UNKNOWN,
	AMDGIM_ERROR_MONITOR_INVALID_PARAM,
	AMDGIM_ERROR_MONITOR_CLRVFFB_VF_UNSET,
	AMDGIM_ERROR_MONITOR_CLRVFFB_VF_INUSE,

	AMDGIM_ERROR_MONITOR_CLRVFFB_VF_FULL_ACCESS,
	AMDGIM_ERROR_MONITOR_GETVF_INVALID_FB_OFFSET,
	AMDGIM_ERROR_MONITOR_GETVF_INVALID_FB_SIZE,
	AMDGIM_ERROR_MONITOR_GETVF_INVALID_GFX_PART,
	AMDGIM_ERROR_MONITOR_GETVF_NO_VF_AVAILABLE,

	AMDGIM_ERROR_MONITOR_GETVF_OSG_VF_INUSE,
	AMDGIM_ERROR_MONITOR_GETVF_OSG_IN_ZZZ,
	AMDGIM_ERROR_MONITOR_GETVF_GFX_PART_NOT_ENOUGH,
	AMDGIM_ERROR_MONITOR_GETVF_CANT_ALLOC_OFFSET_AND_SIZE,
	AMDGIM_ERROR_MONITOR_GETVF_ZZZ_IN_OSG,

	AMDGIM_ERROR_MONITOR_RELVF_VF_NOT_FOUND,
	AMDGIM_ERROR_MONITOR_RELVF_VF_INUSE,
	AMDGIM_ERROR_MONITOR_RELVF_VF_UNSET,
	AMDGIM_ERROR_MONITOR_GET_ROM_HEADER_FAIL,
	AMDGIM_ERROR_MONITOR_SKIP_ROM_OFFSET_FAIL,

	AMDGIM_ERROR_MONITOR_GET_MASTER_TABLE_FAIL,
	AMDGIM_ERROR_MONITOR_VFS_SCHEDULE_SWITCH_FAILED,
	AMDGIM_ERROR_MONITOR_NO_ENOUGH_QUOTA,
	AMDGIM_ERROR_MONITOR_GPUVS_SCLK_INVALID_DIVIDER,

	AMDGIM_ERROR_MONITOR_TEST,
	AMDGIM_ERROR_MONITOR_MAX
};

typedef int amdgim_command_func_t(char *param, void *obj, void *result);
typedef int amdgim_command_to_str_func_t(void *obj, char *str);
struct amdgim_command_op {
	amdgim_command_func_t        *func;
};

struct amdgim_command_to_str {
	amdgim_command_to_str_func_t *func;
};

/* DATA EXCHANGE */
#pragma pack(push)
/* pack(4) because gim need to edit VF framebuffer by MMRegisters.
 * This method write 4 byte once. So pack(4) would make the code simpler.
 */
#pragma pack(4)
/* GIM supports feature of Error log collecting */
#define AMDGIM_FEATURE_ERROR_LOG_COLLECT     0x1
struct amdgim_pf2vf_info_header {
	unsigned int size;/* the total structure size in byte. */
	unsigned int version;/* version of this structure, written by the GIM */
};
struct  amdgim_pf2vf_info_v1 {
	/* header contains size and version */
	struct amdgim_pf2vf_info_header header;
	/* max_width * max_height */
	unsigned int uvd_enc_max_pixels_count;
	/* 16x16 pixels/sec, codec independent */
	unsigned int uvd_enc_max_bandwidth;
	/* max_width * max_height */
	unsigned int vce_enc_max_pixels_count;
	/* 16x16 pixels/sec, codec independent */
	unsigned int vce_enc_max_bandwidth;
	/* MEC FW position in kb from the start of VF visible frame buffer */
	unsigned int mecfw_kboffset;
	/* The features flags of the GIM driver supports. */
	unsigned int feature_flags;
	/* use private key from mailbox 2 to create chueksum */
	unsigned int checksum;
};

struct amdgim_vf2pf_info_header {
	/* the total structure size in byte. */
	unsigned int size;
	/* version of this structure, written by the guest,
	 * gim read it to decide how to analyze the structure
	 */
	unsigned int version;
};

struct amdgim_vf2pf_info_v1 {
	/* header contains size and version */
	struct amdgim_vf2pf_info_header header;
	/* driver version */
	char driver_version[64];
	/* driver certification, 1=WHQL, 0=None */
	unsigned int driver_cert;
	/* guest OS type and version: need a define */
	unsigned int os_info;
	/* in the unit of 1M */
	unsigned int fb_usage;
	/* guest gfx engine usage percentage */
	unsigned int gfx_usage;
	/* guest gfx engine health percentage */
	unsigned int gfx_health;
	/* guest compute engine usage percentage */
	unsigned int compute_usage;
	/* guest compute engine health percentage */
	unsigned int compute_health;
	/* guest vce engine usage percentage. 0xffff means N/A. */
	unsigned int vce_enc_usage;
	/* guest vce engine health percentage. 0xffff means N/A. */
	unsigned int vce_enc_health;
	/* guest uvd engine usage percentage. 0xffff means N/A. */
	unsigned int uvd_enc_usage;
	/* guest uvd engine usage percentage. 0xffff means N/A. */
	unsigned int uvd_enc_health;
	unsigned int checksum;
};
#pragma pack(pop)

struct amdgim_mcil_data_info {
	unsigned long size;
	unsigned long datasize;
	void *data;
	unsigned long offset;
	unsigned long ret_size;
	unsigned long padding;
};

union amdgim_dbdf {
	struct{
		unsigned int func   : 3;
		unsigned int dev    : 5;
		unsigned int bus    : 8;
		unsigned int domain : 16;
	} fields;
	unsigned int u32all;
};

struct amdgim_getvf_option {
	unsigned int fb_offset;
	unsigned int fb_size;
	unsigned int gfx_divider;
};


struct amdgim_gpuinfo {
	char name[AMDGIM_STRLEN_NORMAL];
	unsigned int dbdf;
	int dpm_cap;
	int power_cap;
	int fbsize;
	bool uncorr_ecc;
	int max_vf_num;
	char gfx_engine[AMDGIM_STRLEN_NORMAL];
	int max_gfx_clk;
	int cu_number;
	char video_encoder[AMDGIM_STRLEN_LONG];
	/* pcie_link_speed = pcie_speed / 2 */
	int pcie_speed;
	int pcie_width;
};

struct amdgim_gpuinfos {
	int gpu_count;
	struct amdgim_gpuinfo gpuinfo[MAX_ADAPTERS_IN_SYSTEM];
};

struct amdgim_gpuvs {
	char name[AMDGIM_STRLEN_NORMAL];
	unsigned int dbdf;
	unsigned int power_usage;
	unsigned int cur_volt;
	unsigned int temperature;
	unsigned int cur_dpm;
	unsigned int memory_usage;
	unsigned int available_vf;
	unsigned int gfx_clk;
	unsigned int gfx_usage;
	unsigned int vce_usage;
	unsigned int uvd_usage;
	unsigned int correctable_error;
	unsigned int uncorrectable_error;
};

struct amdgim_gpuvs_group {
	int gpu_count;
	struct amdgim_gpuvs gpuvs[MAX_ADAPTERS_IN_SYSTEM];
};

struct amdgim_vbios_info {
	char name[AMDGIM_STRLEN_NORMAL];
	char dbdf[AMDGIM_STRLEN_NORMAL];
	char vbios_pn[AMDGIM_STRLEN_LONG];
	unsigned int version;
	char date[AMDGIM_STRLEN_NORMAL];
	char serial[AMDGIM_STRLEN_VERYLONG];
	unsigned short dev_id;
	unsigned int rev_id;
	unsigned int sub_dev_id;
	unsigned int sub_ved_id;
};

struct amdgim_vbios_infos {
	int gpu_count;
	struct amdgim_vbios_info vbiosinfos[MAX_ADAPTERS_IN_SYSTEM];
};

struct amdgim_vfinfo {
	int vf_id;
	char vf_name[AMDGIM_STRLEN_NORMAL];
	unsigned int dbdf;
	int vf_state;
	int fb_size;
	int gfx_engine_part;
};

struct amdgim_gpu_vfinfos {
	/* type=AMDGIM_GPUVF_PF */
	int type;
	unsigned int gpu_id;
	char name[AMDGIM_STRLEN_NORMAL];
	unsigned int dbdf;
	int vf_count;
	struct amdgim_vfinfo vf_info[MAX_VIRTUAL_FUNCTIONS];
};

struct amdgim_total_infos {
	/* type=AMDGIM_GPUVF_TOTAL */
	int type;
	int gpu_count;
	struct amdgim_gpu_vfinfos gpu_vfinfos[MAX_ADAPTERS_IN_SYSTEM];
};

struct amdgim_vf_detail {
	/* type=AMDGIM_GPUVF_VF */
	int type;
	unsigned int gpu_dbdf;
	int gpu_active_vf;
	unsigned int vf_dbdf;
	char vf_name[AMDGIM_STRLEN_NORMAL];
	char vf_type[AMDGIM_STRLEN_NORMAL];
	bool vf_state;
	char guest_driver_version[AMDGIM_STRLEN_LONG];
	unsigned int guest_driver_cert;
	unsigned long long vf_running_section;
	unsigned long long vf_active_section;
	struct op_time_log time_log;
	int fbsize;
	int vce_bandwidth;
	int hevc_bandwidth;
};

struct amdgim_vf_bdf_list {
	int count;
	char bdf[MAX_VIRTUAL_FUNCTIONS][AMDGIM_STRLEN_SHORT];
};

/* OPERATIONS */
int amdgim_do_op(char *param, void *object, int index, char *result);

struct amdgim_asic_operation {
	int (*amdgim_get_gpu_power)(struct adapter *, int *);
	int (*amdgim_get_gfx_activity)(struct adapter *, int *);
	int (*amdgim_get_asic_temperature)(struct adapter*, int *);
	int (*amdgim_get_vddc)(struct adapter*, int*);
	int (*amdgim_get_dpm_status)(struct adapter*, int*);
	int (*amdgim_get_dpm_cap)(struct adapter*, int*);
	int (*amdgim_get_mem_activity)(struct adapter*, int*);
	int (*amdgim_get_uvd_activity)(struct adapter*, int*);
	int (*amdgim_get_vce_activity)(struct adapter*, int*);
	int (*amdgim_get_gecc)(struct adapter*, bool*);
	int (*amdgim_get_sclk)(struct adapter*, int*);
	int (*amdgim_get_bios_serial)(struct adapter*, char *);
	int (*amdgim_get_ecc_info)(struct adapter*, int*, int*);
	void (*amdgim_tostrall)(struct adapter*, struct amdgim_gpuvs*, char*);
};

struct amdgim_asic_operation *amdgim_tonga_get_asic_op(void);
int amdgim_get_vf_candidate(struct adapter *p_adapter);

#endif
