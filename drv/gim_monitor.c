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

#include <linux/string.h>
#include <linux/rtc.h>

#include "gim_adapter.h"
#include "gim_monitor.h"
#include "gim_monitor_ioctl.h"
#include "gim_os_service.h"
#include "gim_debug.h"
#include "gim_reset.h"
#include "gim_kcl_pci.h"
#include "gim_atombios.h"
#include "gim_interface.h"
#include "gim_timer.h"
#include "gim_gpuiov.h"
#include "oss/oss_3_0_d.h"

#define AMDGIM_CLEAR_SIZE_IN_MB             128
#define AMDGIM_CLEAR_SIZE_TICK_BYTE         1024
#define AMDGIM_MB_TO_BYTE(x)                ((x) << 20)
#define AMDGIM_MB_TO_MC_SIZE(x)             ((x) << 12)
#define AMDGIM_MC_SIZE_TO_MB(x)             ((x) >> 12)

#define BIOS_ATOM_PREFIX                    "ATOMBIOS"
#define ATOM_VBIOS_PART_NUMBER_OFFSET       0x80
#define VBIOS_DATE_OFFSET                   0x50
#define BIOS_STRING_LENGTH                  43

#define AMDGIM_HOUR_IN_SEC                  3600
#define AMDGIM_MINUTE_IN_SEC                60

#define TIME_FMRT "%04d-%02d-%02d %02d:%02d:%02d\n"

static
unsigned short asic_type_table[AMDGIM_ASIC_TYPE_LEN][AMDGIM_ASIC_PCI_LEN] = {
	[AMDGIM_ASIC_TYPE_UNKNOWN] = {
		[AMDGIM_ASIC_PCI_VENDOR_ID] = 0,
		[AMDGIM_ASIC_PCI_DEVICE_ID] = 0,
		[AMDGIM_ASIC_PCI_REV_ID] = 0,
	},
	[AMDGIM_ASIC_TYPE_TONGA] = {
		[AMDGIM_ASIC_PCI_VENDOR_ID] = 0x1002,
		[AMDGIM_ASIC_PCI_DEVICE_ID] = 0x6929,
		[AMDGIM_ASIC_PCI_REV_ID] = 0,
	},
};

static char *asic_info_table[AMDGIM_ASIC_TYPE_LEN][AMDGIM_ASICINFO_LEN] = {
	/* name,cu,gfx,vf_perfix */
	[AMDGIM_ASIC_TYPE_UNKNOWN] = {
		[AMDGIM_ASICINFO_NAME] = AMDGIM_STR_NA,
		[AMDGIM_ASICINFO_CU] = AMDGIM_STR_NA,
		[AMDGIM_ASICINFO_GFX] = AMDGIM_STR_NA,
		[AMDGIM_ASICINFO_VF_PREFIX] = AMDGIM_STR_NA,
		[AMDGIM_ASICINFO_VIDEO_ENCODER] = AMDGIM_STR_NA,
		[AMDGIM_ASICINFO_DPM_CAP] = AMDGIM_STR_NA,
		[AMDGIM_ASICINFO_POWER_CAP] = AMDGIM_STR_NA,
		[AMDGIM_ASICINFO_MAX_VF_NUM] = AMDGIM_STR_NA,
		[AMDGIM_ASICINFO_UNCORR_ECC] = AMDGIM_STR_NA,
		[AMDGIM_ASICINFO_MAX_GFX_CLK] = AMDGIM_STR_NA,
	},
	[AMDGIM_ASIC_TYPE_TONGA] = {
		[AMDGIM_ASICINFO_NAME] = "S7150",
		[AMDGIM_ASICINFO_CU] = "32",
		[AMDGIM_ASICINFO_GFX] = "GFX8",
		[AMDGIM_ASICINFO_VF_PREFIX] = "MxGPU_V1_",
		[AMDGIM_ASICINFO_VIDEO_ENCODER] = AMDGIM_STR_NONE,
		[AMDGIM_ASICINFO_DPM_CAP] = "8",
		[AMDGIM_ASICINFO_POWER_CAP] = "109",
		[AMDGIM_ASICINFO_MAX_VF_NUM] = "16",
		[AMDGIM_ASICINFO_UNCORR_ECC] = AMDGIM_STR_NO,
		[AMDGIM_ASICINFO_MAX_GFX_CLK] = "1000",
	},
};

static int command_results_len[AMDGIM_COMMAND_LEN] = {
	[AMDGIM_COMMAND_GPUINFO] = sizeof(struct amdgim_gpuinfos),
	[AMDGIM_COMMAND_GPUVS] = sizeof(struct amdgim_gpuvs_group),
	[AMDGIM_COMMAND_GPUBIOS] = sizeof(struct amdgim_vbios_infos),
	[AMDGIM_COMMAND_GPUVF_PF] = sizeof(struct amdgim_total_infos),
	[AMDGIM_COMMAND_GPUVF_VF] = sizeof(struct amdgim_vf_detail),
	[AMDGIM_COMMAND_CLRVFFB] = AMDGIM_STRLEN_VERYLONG,
	[AMDGIM_COMMAND_GETVF] = AMDGIM_STRLEN_VERYLONG,
	[AMDGIM_COMMAND_RELVF] = AMDGIM_STRLEN_VERYLONG,
	[AMDGIM_COMMAND_FLR] = AMDGIM_STRLEN_VERYLONG,
	[AMDGIM_COMMAND_HOTLINK_RESET] = AMDGIM_STRLEN_VERYLONG,
};

static const int base_time_slice[MAX_VIRTUAL_FUNCTIONS+1] = {
	[0] = 500,	/* 1 VF enabled  */
	[1] = 14,  /* 2 VF enabled, slice = 2*7 */
	[2] = 21,  /* 3 VF enabled, slice = 3*7  */
	[3] = 28,  /* 4 VF enabled, slice = 4*7  */
	[4] = 35,  /* 5 VF enabled, slice = 5*7  */
	[5] = 42,  /* 6 VF enabled, slice = 6*7  */
	[6] = 49,  /* 7 VF enabled, slice = 7*7  */
	[7] = 56,  /* 8 VF enabled, slice = 8*7  */
	[8] = 63,  /* 9 VF enabled, slice = 9*7  */
	[9] = 70,  /* 10 VF enabled, slice = 10*7  */
	[10] = 77,	/* 11 VF enabled, slice = 11*7	*/
	[11] = 84,	/* 12 VF enabled, slice = 12*7	*/
	[12] = 78,	/* 13 VF enabled, slice = 13*6	*/
	[13] = 84,	/* 14 VF enabled, slice = 14*6	*/
	[14] = 75,	/* 15 VF enabled, slice = 15*5	*/
	[15] = 80,	/* 16 VF enabled, slice = 16*5	*/
};

static const int base_time_slice_stress[MAX_VIRTUAL_FUNCTIONS+1] = {
	[0] = 1,  /* 1 VF enabled  */
	[1] = 2,  /* 2 VF enabled, slice = 2*1 */
	[2] = 3,  /* 3 VF enabled, slice = 3*1	*/
	[3] = 4,  /* 4 VF enabled, slice = 4*1	*/
	[4] = 5,  /* 5 VF enabled, slice = 5*1	*/
	[5] = 6,  /* 6 VF enabled, slice = 6*1	*/
	[6] = 7,  /* 7 VF enabled, slice = 7*1	*/
	[7] = 8,  /* 8 VF enabled, slice = 8*1	*/
	[8] = 9,  /* 9 VF enabled, slice = 9*1	*/
	[9] = 10,  /* 10 VF enabled, slice = 10*1  */
	[10] = 11,	/* 11 VF enabled, slice = 11*1	*/
	[11] = 12,	/* 12 VF enabled, slice = 12*1	*/
	[12] = 13,	/* 13 VF enabled, slice = 13*1	*/
	[13] = 14,	/* 14 VF enabled, slice = 14*1	*/
	[14] = 15,	/* 15 VF enabled, slice = 15*1	*/
	[15] = 16,	/* 16 VF enabled, slice = 16*1	*/
};

static const char *amdgim_error_monitor[AMDGIM_ERROR_MONITOR_MAX] = {
	/* AMDGIM_ERROR_MONITOR_SUCCESS */
	"Operation Succeeded",
	/* AMDGIM_ERROR_MONITOR_UNKNOWN */
	"Unknown Error",
	/* AMDGIM_ERROR_MONITOR_INVALID_PARAM */
	"Invalid Parameters",
	/* AMDGIM_ERROR_MONITOR_CLRVFFB_VF_UNSET */
	"The VF is not setup yet. No FB clear action can be taken",
	/* AMDGIM_ERROR_MONITOR_CLRVFFB_VF_INUSE */
	"The VF is in use. FB can't be cleared",

	/* AMDGIM_ERROR_MONITOR_CLRVFFB_VF_FULL_ACCESS */
	"The VF is in the FULL ACCESS",
	/* AMDGIM_ERROR_MONITOR_GETVF_INVALID_FB_OFFSET */
	"Invalid frame buffer offset, not in range",
	/* AMDGIM_ERROR_MONITOR_GETVF_INVALID_FB_SIZE */
	"Invalid frame buffer size, should be larger than 256M and aligned "
	"in 16M",
	/* AMDGIM_ERROR_MONITOR_GETVF_INVALID_GFX_PART */
	"Invalid GFX partition, should be power of 2 and in the range from 0 "
	"to 16",
	/* AMDGIM_ERROR_MONITOR_GETVF_NO_VF_AVAILABLE */
	"Can't find an availible VF",

	/* AMDGIM_ERROR_MONITOR_GETVF_OSG_VF_INUSE */
	"Detected Active VFs. The GPU partition should be managed with three "
	"zero inputs.",
	/* AMDGIM_ERROR_MONITOR_GETVF_OSG_IN_ZZZ */
	"The GPU partition should be managed with three zero inputs.",
	/* AMDGIM_ERROR_MONITOR_GETVF_GFX_PART_NOT_ENOUGH */
	"The specified GFX_partition cannot be satisfied",
	/* AMDGIM_ERROR_MONITOR_GETVF_CANT_ALLOC_OFFSET_AND_SIZE */
	"Can not allocate a frame buffer with this offset and size",
	/* AMDGIM_ERROR_MONITOR_GETVF_ZZZ_IN_OSG */
	"The GPU partition should be managed with three non-zero inputs.",

	/* AMDGIM_ERROR_MONITOR_RELVF_VF_NOT_FOUND */
	"Failed to release VF. The specified VF cannot be found.",
	/* AMDGIM_ERROR_MONITOR_RELVF_VF_INUSE */
	"Failed to release VF. The VF is still in use.",
	/* AMDGIM_ERROR_MONITOR_RELVF_VF_UNSET */
	"Failed to release VF. The VF has not been set.",
	/* AMDGIM_ERROR_MONITOR_GET_ROM_HEADER_FAIL */
	"Failed to get rom header.",
	/* AMDGIM_ERROR_MONITOR_SKIP_ROM_OFFSET_FAIL */
	"Failed skip rom offset.",

	/* AMDGIM_ERROR_MONITOR_GET_MASTER_TABLE_FAIL */
	"Failed to get master table.",
	/* AMDGIM_ERROR_MONITOR_VFS_SCHEDULE_SWITCH_FAILED */
	"Schedule VF%u to VF%u failed, failure reason is %u, try to reset.",
	/* AMDGIM_ERROR_MONITOR_NO_ENOUGH_QUOTA */
	"Not enough quota left.",
	/* AMDGIM_ERROR_MONITOR_GPUVS_SCLK_INVALID_DIVIDER */
	"Invalid divider of GPUVS sclk.",

	/* AMDGIM_ERROR_MONITOR_TEST */
	"This is error log collect test for MONITOR component "
	"(test count %llu)."
};

char *amdgim_skip_space(char *str)
{
	char char_set[] = " \n\r\10";

	while (str != NULL && strspn(str, char_set) && *str != '\0')
		str++;

	return str;
}

static int amdgim_get_asic_index(struct adapter *adapt)
{
	int i;
	unsigned short did, vid;

	for (i = 0; i < AMDGIM_ASIC_TYPE_LEN; ++i) {
		vid = asic_type_table[i][AMDGIM_ASIC_PCI_VENDOR_ID];
		did = asic_type_table[i][AMDGIM_ASIC_PCI_DEVICE_ID];
		if ((vid == adapt->pf.pci_dev->vendor) &&
			(did == adapt->pf.pci_dev->device))
			return i;
	}

	return 0;
}

static struct amdgim_asic_operation *amdgim_get_asic_op(unsigned int index)
{
	switch (index) {
	case AMDGIM_ASIC_TYPE_TONGA:
		return amdgim_tonga_get_asic_op();
	default:
		gim_err("Unknown ASIC");
		break;
	}
	return NULL;
}

char *amdgim_dbdf_str2int(int *dbdf, char *str)
{
	int dom, bus, dev, fun;
	char *numset = "1234567890abcdefABCDEF";

	if (str != NULL && strspn(str, numset)) {
		str = get_int(&dom, str);
		dom &= 0xffff;
	} else {
		return NULL;
	}

	if (str != NULL && *str == ':')
		str++;
	else
		return NULL;

	if (str != NULL && strspn(str, numset)) {
		str = get_int(&bus, str);
		bus &= 0xff;
	} else {
		return NULL;
	}

	if (str != NULL && *str == ':')
		str++;
	else
		return NULL;

	if (str != NULL && strspn(str, numset)) {
		str = get_int(&dev, str);
		dev &= 0x1f;
	} else {
		return NULL;
	}

	if (str != NULL && *str == '.')
		str++;
	else
		return NULL;

	if (str != NULL && strspn(str, numset)) {
		str = get_int(&fun, str);
		fun &= 0x7;
	} else {
		return NULL;
	}

	*dbdf = (dom << 16) | (bus << 8) | (dev << 3) | fun;

	return str;
}

int amdgim_parse_getvf(char *param, struct amdgim_getvf_option *option)
{
	int ret;

	ret = sscanf(param, "%u %u %u", &option->fb_offset, &option->fb_size,
		&option->gfx_divider);

	if (ret != 3)
		return AMDGIM_ERROR_MONITOR_INVALID_PARAM;

	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

int amdgim_parse_relvf(char *param, union amdgim_dbdf *dbdf)
{
	param = amdgim_dbdf_str2int(&dbdf->u32all, param);
	if (param == NULL)
		return AMDGIM_ERROR_MONITOR_INVALID_PARAM;

	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

static int amdgim_read_rom_image(struct adapter *p_adapter,
			struct amdgim_mcil_data_info *rominfo)
{

	unsigned char *p_rom;
	unsigned char *p_dest;
	unsigned long i;

	p_dest = (unsigned char *)(rominfo->data);
	p_rom  = (unsigned char *)(p_adapter->pvbios_image);
	p_rom += rominfo->offset;

	for (i = 0; i < rominfo->datasize; i++)
		p_dest[i] = p_rom[i];

	rominfo->ret_size = rominfo->datasize;
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

static unsigned int amdgim_get_vbios_rom_header(struct adapter *p_adapter,
				struct atom_rom_header *rom_hdr)
{
	unsigned long vbios_version;
	unsigned short offset_to_vbios_table;
	struct amdgim_mcil_data_info rominfo;

	vbios_version = 0;

	rominfo.size = sizeof(struct amdgim_mcil_data_info);
	rominfo.data = &offset_to_vbios_table;
	rominfo.offset = OFFSET_TO_POINTER_TO_ATOM_ROM_HEADER;
	rominfo.datasize = sizeof(unsigned short);
	if (amdgim_read_rom_image(p_adapter, &rominfo)
		 == AMDGIM_ERROR_MONITOR_SUCCESS) {
		rominfo.size     = sizeof(struct amdgim_mcil_data_info);
		rominfo.data     = rom_hdr;
		rominfo.offset   = offset_to_vbios_table;
		rominfo.datasize = sizeof(struct atom_rom_header);

		return amdgim_read_rom_image(p_adapter, &rominfo);
	}

	return AMDGIM_ERROR_MONITOR_UNKNOWN;
}

static unsigned int amdgim_get_vbios_master_table(struct adapter *p_adapter,
			struct atom_master_data_table *data_tlb)
{
	struct amdgim_mcil_data_info rominfo;
	struct atom_rom_header rom_hdr;

	if (amdgim_get_vbios_rom_header(p_adapter, &rom_hdr)
		 == AMDGIM_ERROR_MONITOR_SUCCESS) {
		if (rom_hdr.master_data_table_offset != 0) {
			rominfo.size     = sizeof(struct amdgim_mcil_data_info);
			rominfo.data     = data_tlb;
			rominfo.offset   = rom_hdr.master_data_table_offset;
			rominfo.datasize =
					sizeof(struct atom_master_data_table);

			return amdgim_read_rom_image(p_adapter, &rominfo);
		}
	} else{
		gim_err("Failed to get master table");
	}
	return AMDGIM_ERROR_MONITOR_UNKNOWN;
}

static unsigned int amdgim_get_vbios_firmware_info(struct adapter *p_adapter,
				struct atom_firmware_info *atom_fw_info)
{
	struct amdgim_mcil_data_info rominfo;
	struct atom_master_data_table data_tlb;

	if (amdgim_get_vbios_master_table(p_adapter, &data_tlb)
		 == AMDGIM_ERROR_MONITOR_SUCCESS) {
		if (data_tlb.tables_list.firmware_info != 0) {
			rominfo.size = sizeof(struct amdgim_mcil_data_info);
			rominfo.data = atom_fw_info;
			rominfo.offset = data_tlb.tables_list.firmware_info;
			rominfo.datasize = sizeof(struct atom_firmware_info);

			return amdgim_read_rom_image(p_adapter, &rominfo);
		}
	}

	return AMDGIM_ERROR_MONITOR_UNKNOWN;
}

static unsigned int amdgim_get_vbios_ver_subinfo(struct adapter *p_adapter,
					struct amdgim_vbios_info *vbiosinfo)
{
	unsigned long vbios_version;
	struct atom_rom_header rom_hdr;
	struct atom_firmware_info atom_fw_info;

	vbios_version = 0;

	if (amdgim_get_vbios_rom_header(p_adapter, &rom_hdr)
		== AMDGIM_ERROR_MONITOR_SUCCESS) {
		vbiosinfo->sub_ved_id = rom_hdr.sub_sys_vendor_id;
		vbiosinfo->sub_dev_id = rom_hdr.subsystem_id;
	}

	if (amdgim_get_vbios_firmware_info(p_adapter, &atom_fw_info)
		== AMDGIM_ERROR_MONITOR_SUCCESS)
		vbiosinfo->version = atom_fw_info.firmware_revision;

	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

static unsigned char *amdgim_find_str_in_rom(struct adapter *p_adapter,
				char *str, int start, int end)
{
	unsigned long str_off;
	unsigned char *p_rom;
	unsigned short str_len;

	str_off = 0;
	str_len = strnlen(str, AMDGIM_STRLEN_LONG);
	p_rom = p_adapter->pvbios_image;

	for (; start <= end; ++start) {
		for (str_off = 0; str_off < str_len; ++str_off) {
			if (str[str_off] != *(p_rom + start + str_off))
				break;
		}

		if (str_off == str_len || str[str_off] == 0)
			return p_rom + start;
	}
	return NULL;
}

static int amdgim_get_vbios_pn(struct adapter *p_adapter, unsigned char *pn_str)
{
	unsigned char *p_rom;
	unsigned short off_to_vbios_str;
	unsigned char *vbios_str;
	int count;

	off_to_vbios_str = 0;
	p_rom = p_adapter->pvbios_image;

	if (*(p_rom + OFFSET_TO_GET_ATOMBIOS_NUMBER_OF_STRINGS) != 0) {
		off_to_vbios_str = *(unsigned short *)(p_rom
				+ OFFSET_TO_GET_ATOMBIOS_STRING_START);
		vbios_str = (unsigned char *)(p_rom + off_to_vbios_str);
	} else{
		vbios_str = p_rom + ATOM_VBIOS_PART_NUMBER_OFFSET;
	}

	if (*vbios_str == 0) {
		vbios_str = amdgim_find_str_in_rom(p_adapter,
					BIOS_ATOM_PREFIX, 3, 1024);

		if (vbios_str == NULL)
			vbios_str += sizeof(BIOS_ATOM_PREFIX) - 1;
	}
	if (NULL != vbios_str && 0 == *vbios_str)
		vbios_str++;

	if (vbios_str != NULL) {

		count = 0;
		while ((count < BIOS_STRING_LENGTH)
			 && vbios_str[count] >= ' '
			 && vbios_str[count] <= 'z') {
			pn_str[count] = vbios_str[count];
			count++;
		}
		pn_str[count] = 0;
	}
	return 0;
}

static int amdgim_get_vbios_date(struct adapter *p_adapter, char *date_str)
{
	unsigned char *p_rom;
	unsigned char *date_in_rom;

	p_rom = p_adapter->pvbios_image;

	date_in_rom = p_rom + VBIOS_DATE_OFFSET;

	date_str[0] = '2';
	date_str[1] = '0';
	date_str[2] = date_in_rom[6];
	date_str[3] = date_in_rom[7];
	date_str[4] = '/';
	date_str[5] = date_in_rom[0];
	date_str[6] = date_in_rom[1];
	date_str[7] = '/';
	date_str[8] = date_in_rom[3];
	date_str[9] = date_in_rom[4];
	date_str[10] = ' ';
	date_str[11] = date_in_rom[9];
	date_str[12] = date_in_rom[10];
	date_str[13] = date_in_rom[11];
	date_str[14] = date_in_rom[12];
	date_str[15] = date_in_rom[13];
	date_str[16] = '\0';

	return 0;

}

/* Do a self-switch immediately to check if the running VF is alive,
 * not recommended to run when not in manual mode or if more than
 * 1 VF is running
 */
static int amdgim_gpu_check_idle(struct adapter *p_adapter)
{
	struct function_list_node *next_node;
	struct function *p_current_func;
	struct function *p_next_func;
	int retVal = 0;

	mutex_lock(&p_adapter->curr_running_func_mutex);
	next_node = p_adapter->curr_running_func->next;

	p_current_func = p_adapter->curr_running_func->func;
	p_next_func =  next_node->func;

	retVal = switch_vfs(p_current_func, p_next_func);
	if (retVal != GIM_OK) {
		gim_err("Schedule VF%u to VF%u failed",
				p_current_func->func_id,
				p_next_func->func_id);
		gim_err("failure reason is %u, try to reset",
				p_current_func->flr_reason_code
				+ p_next_func->flr_reason_code);

		gim_sched_reset(p_adapter, p_current_func, p_next_func,
				p_current_func->flr_reason_code
				+ p_next_func->flr_reason_code);
		if (p_current_func->needs_flr) {
			p_adapter->curr_running_func = next_node;
			p_current_func->needs_flr = 0;
			remove_from_run_list(p_current_func);
		} else {
			p_next_func->needs_flr = 0;
			remove_from_run_list(p_next_func);
		}
	} else {
		p_adapter->curr_running_func = next_node;
		mutex_unlock(&p_adapter->curr_running_func_mutex);
		/* so it is really living, skip the clean.*/
		gim_info("There is still one VF running");
		return AMDGIM_ERROR_MONITOR_UNKNOWN;
	}
	/* there is no one living, go ahead*/
	mutex_unlock(&p_adapter->curr_running_func_mutex);
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

int amdgim_get_vf_candidate(struct adapter *p_adapter)
{
	unsigned int vf_candidate;
	unsigned int max_try = 0;
	struct function_list_node *node;
	struct function_list_node *ptr;

	vf_candidate = 0;

	mutex_lock(&p_adapter->curr_running_func_mutex);
	if (p_adapter->curr_running_func != NULL) {
		node = p_adapter->curr_running_func;
		ptr = node;
		do {
			vf_candidate |= 1 << ptr->func->func_id;
			ptr = ptr->next;
			max_try++;
		} while (ptr != node && max_try < p_adapter->enabled_vfs);
	}
	mutex_unlock(&p_adapter->curr_running_func_mutex);

	if (max_try >= p_adapter->enabled_vfs)
		gim_warn("max_try = %d, enabled_vfs =%d\n",
			max_try, p_adapter->enabled_vfs);

	/* if there is just one vf running,
	 * check whether is was shutdown
	 */
	if (((vf_candidate - 1) & vf_candidate) == 0
		&& vf_candidate != 0
		&& p_adapter->switch_to_itself == false){
		if (amdgim_gpu_check_idle(p_adapter)
			== AMDGIM_ERROR_MONITOR_SUCCESS)
			vf_candidate = 0;
	}

	return vf_candidate;
}

static void amdgim_put_info(void *obj, char *str)
{

	strncpy(str, (char *)obj, AMDGIM_BUFFER_SIZE-1);
	strcat(str, "\n");
}


/* True speed = speed /2 */
static int amdgim_get_pcie_link(struct adapter *adapt,
				int *speed, int *width)
{
	int pos;
	int cap1, status;
	int max_speed, max_width, cur_speed, cur_width;

	pos = pci_find_capability(adapt->pf.pci_dev, PCI_CAP_ID_EXP);
	pci_read_config_dword(adapt->pf.pci_dev, pos + PCI_EXP_LNKCAP, &cap1);
	pci_read_config_dword(adapt->pf.pci_dev, pos + PCI_EXP_LNKSTA, &status);

	max_speed = cap1 & PCI_EXP_LNKCAP_SLS;
	cur_speed = status & PCI_EXP_LNKSTA_CLS;

	max_width = (cap1 & PCI_EXP_LNKCAP_MLW) >> 4;
	cur_width = (status & PCI_EXP_LNKSTA_NLW) >> PCI_EXP_LNKSTA_NLW_SHIFT;
	switch (max_speed) {
	case 1:
		*speed = 5;
		break;
	case 2:
		*speed = 10;
		break;
	case 3:
		*speed = 16;
		break;
	default:
		*speed = 0;
		break;
	}

	*width = max_width;
	return 0;
}

int amdgim_get_total_time_slice(struct adapter *p_adapter)
{
	return base_time_slice[p_adapter->enabled_vfs - 1];
}

int amdgim_get_quota_ms(struct adapter *p_adapter, int divider)
{
	unsigned int deserve_quota;
	unsigned int vf_candidate;
	int quota_left;
	unsigned int vf_funcid = 0;

	deserve_quota = base_time_slice[p_adapter->enabled_vfs - 1] / divider;
	vf_candidate = amdgim_get_vf_candidate(p_adapter);
	quota_left = base_time_slice[p_adapter->enabled_vfs - 1];

	for (vf_funcid = 0; vf_funcid < p_adapter->enabled_vfs; ++vf_funcid) {
		/* if this vf had been set manually, ignore the status and
		 * regard it as running.
		 */
		if (true == p_adapter->vfs[vf_funcid].user_option.valid)
			quota_left -=
			p_adapter->vfs[vf_funcid].user_option.gfx_time_quota;
		/* if has default vf running */
		vf_candidate >>= 1;
	}

	if (quota_left < deserve_quota) {
		gim_err("No enough quota left.\n");
		return 0;
	} else {
		return deserve_quota;
	}

}

static int amdgim_op_gpuinfo(char *param, void *obj, void *result)
{
	int i;
	int ret;
	int asic;
	struct amdgim_asic_operation *asic_op;
	struct amdgim_gpuinfos *p_gpuinfos;
	struct adapter *p_adapter;
	struct adapter *adapters = get_adapters();
	int adapter_count = get_adapter_count();

	p_gpuinfos = (struct amdgim_gpuinfos *)result;
	for (i = 0; i < adapter_count; ++i) {
		p_adapter = adapters + i;
		asic = amdgim_get_asic_index(p_adapter);
		asic_op = amdgim_get_asic_op(asic);
		/* ASIC name */
		strncpy(p_gpuinfos->gpuinfo[i].name,
				asic_info_table[asic][AMDGIM_ASICINFO_NAME],
				AMDGIM_STRLEN_NORMAL);

		/* bdf */
		p_gpuinfos->gpuinfo[i].dbdf = p_adapter->pf.bdf
			| pci_domain_nr(p_adapter->pf.pci_dev->bus) << 16;

		ret = kstrtoint(asic_info_table[asic][AMDGIM_ASICINFO_DPM_CAP],
				10, &(p_gpuinfos->gpuinfo[i].dpm_cap));
		if (ret != 0)
			gim_err("failed to get AMDGIM_ASICINFO_DPM_CAP");

		ret = kstrtoint(
			asic_info_table[asic][AMDGIM_ASICINFO_POWER_CAP],
			10,
			&(p_gpuinfos->gpuinfo[i].power_cap));
		if (ret != 0)
			gim_err("failed to get AMDGIM_ASICINFO_POWER_CAP");
		ret = kstrtoint(asic_info_table[asic][AMDGIM_ASICINFO_CU],
				10, &(p_gpuinfos->gpuinfo[i].cu_number));
		if (ret != 0)
			gim_err("failed to get AMDGIM_ASICINFO_CU");

		ret = kstrtoint(
			asic_info_table[asic][AMDGIM_ASICINFO_MAX_GFX_CLK],
			10,
			&(p_gpuinfos->gpuinfo[i].max_gfx_clk));
		if (ret != 0)
			gim_err("failed to get AMDGIM_ASICINFO_MAX_GFX_CLK");

		p_gpuinfos->gpuinfo[i].max_vf_num = p_adapter->total_vfs;
		p_gpuinfos->gpuinfo[i].fbsize =
					p_adapter->gpuiov.total_fb_available;

		/* GFXENGINE */
		strncpy(p_gpuinfos->gpuinfo[i].gfx_engine,
				asic_info_table[asic][AMDGIM_ASICINFO_GFX],
				AMDGIM_STRLEN_NORMAL);

		/* video_encoder */
		strncpy(p_gpuinfos->gpuinfo[i].video_encoder,
			asic_info_table[asic][AMDGIM_ASICINFO_VIDEO_ENCODER],
			AMDGIM_STRLEN_NORMAL);

		amdgim_get_pcie_link(p_adapter,
				&p_gpuinfos->gpuinfo[i].pcie_speed,
				&p_gpuinfos->gpuinfo[i].pcie_width);
	}
	p_gpuinfos->gpu_count = i;
	return 0;
}

static int amdgim_op_gpuvs(char *param, void *obj, void *result)
{
	int i;
	int asic;
	int vf_candidate;
	struct adapter *p_adapter;
	struct amdgim_gpuvs_group *p_gpuvs_g;
	struct amdgim_asic_operation *asic_op;
	struct adapter *adapters = get_adapters();
	int adapter_count = get_adapter_count();

	p_gpuvs_g = (struct amdgim_gpuvs_group *)result;
	for (i = 0; i < adapter_count; ++i) {
		p_adapter = adapters + i;
		asic = amdgim_get_asic_index(p_adapter);
		asic_op = amdgim_get_asic_op(asic);
		if (asic_op == NULL)
			continue;

		strncpy(p_gpuvs_g->gpuvs[i].name,
			asic_info_table[asic][AMDGIM_ASICINFO_NAME],
			AMDGIM_STRLEN_NORMAL);

		p_gpuvs_g->gpuvs[i].dbdf = p_adapter->pf.bdf
			 | (pci_domain_nr(p_adapter->pf.pci_dev->bus) << 16);

		if (asic_op->amdgim_get_gpu_power != NULL)
			asic_op->amdgim_get_gpu_power(p_adapter,
					&p_gpuvs_g->gpuvs[i].power_usage);

		if (asic_op->amdgim_get_vddc != NULL)
			asic_op->amdgim_get_vddc(p_adapter,
					&p_gpuvs_g->gpuvs[i].cur_volt);

		if (asic_op->amdgim_get_asic_temperature != NULL)
			asic_op->amdgim_get_asic_temperature(p_adapter,
					&p_gpuvs_g->gpuvs[i].temperature);

		if (asic_op->amdgim_get_dpm_status != NULL)
			asic_op->amdgim_get_dpm_status(p_adapter,
					&p_gpuvs_g->gpuvs[i].cur_dpm);

		if (asic_op->amdgim_get_mem_activity != NULL)
			asic_op->amdgim_get_mem_activity(p_adapter,
					&p_gpuvs_g->gpuvs[i].memory_usage);

		if (asic_op->amdgim_get_gfx_activity != NULL)
			asic_op->amdgim_get_gfx_activity(p_adapter,
					&p_gpuvs_g->gpuvs[i].gfx_usage);

		if (asic_op->amdgim_get_vce_activity != NULL)
			asic_op->amdgim_get_vce_activity(p_adapter,
					&p_gpuvs_g->gpuvs[i].vce_usage);

		if (asic_op->amdgim_get_uvd_activity != NULL)
			asic_op->amdgim_get_uvd_activity(p_adapter,
					&p_gpuvs_g->gpuvs[i].uvd_usage);

		if (asic_op->amdgim_get_sclk != NULL)
			asic_op->amdgim_get_sclk(p_adapter,
					&p_gpuvs_g->gpuvs[i].gfx_clk);

		if (asic_op->amdgim_get_ecc_info != NULL)
			asic_op->amdgim_get_ecc_info(p_adapter,
				&p_gpuvs_g->gpuvs[i].correctable_error,
				&p_gpuvs_g->gpuvs[i].uncorrectable_error);

		vf_candidate = amdgim_get_vf_candidate(p_adapter);

		p_gpuvs_g->gpuvs[i].available_vf = p_adapter->enabled_vfs;

		/*how many 1 in the binary*/
		while (vf_candidate) {
			vf_candidate &= vf_candidate - 1;
			p_gpuvs_g->gpuvs[i].available_vf--;
		}

		p_gpuvs_g->gpu_count++;
	}

	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

static int amdgim_op_gpubios(char *param, void *obj, void *result)
{
	int i;
	int asic;
	struct adapter *adapt;
	struct amdgim_vbios_infos *infos;
	struct amdgim_asic_operation *asic_op;

	int adapter_count = get_adapter_count();
	struct adapter *adapts = get_adapters();

	infos = (struct amdgim_vbios_infos *)result;
	for (i = 0; i < adapter_count; ++i) {
		adapt = adapts + i;
		asic = amdgim_get_asic_index(adapt);
		asic_op = amdgim_get_asic_op(asic);

		asic_op->amdgim_get_bios_serial(adapt,
				infos->vbiosinfos[i].serial);

		amdgim_get_vbios_ver_subinfo(adapt, &infos->vbiosinfos[i]);
		amdgim_get_vbios_pn(adapt, infos->vbiosinfos[i].vbios_pn);
		amdgim_get_vbios_date(adapt, infos->vbiosinfos[i].date);

		pci_read_config_byte(adapt->pf.pci_dev,
				PCI_REVISION_ID,
				(unsigned char *)&infos->vbiosinfos[i].rev_id);

	}
	infos->gpu_count = i;

	return 0;
}

static int amdgim_op_gpuvf_pf(char *param, void *obj, void *result)
{
	int asic;
	unsigned int j;
	unsigned int vf_candidate;

	struct function *p_func;
	struct partition  *part;
	struct adapter *p_adapter;
	struct amdgim_vfinfo *vfinfo;
	struct amdgim_gpu_vfinfos *gpuinfo;

	p_adapter = ((struct function *)obj)->adapt;
	gpuinfo = (struct amdgim_gpu_vfinfos *)result;

	gpuinfo->type = AMDGIM_GPUVF_PF;

	asic = amdgim_get_asic_index(p_adapter);
	gpuinfo->gpu_id = 0;
	sprintf(gpuinfo->name, "%s",
		asic_info_table[asic][AMDGIM_ASICINFO_NAME]);
	gpuinfo->dbdf = p_adapter->pf.bdf
		| (pci_domain_nr(p_adapter->pf.pci_dev->bus) << 16);
	gpuinfo->vf_count = p_adapter->enabled_vfs;
	vf_candidate = amdgim_get_vf_candidate(p_adapter);
	for (j = 0; j < gpuinfo->vf_count; ++j) {
		p_func = &p_adapter->vfs[j];
		vfinfo = &gpuinfo->vf_info[j];
		vfinfo->vf_id = p_func->func_id;
		sprintf(vfinfo->vf_name, "%s%d",
			asic_info_table[asic][AMDGIM_ASICINFO_VF_PREFIX],
			p_adapter->enabled_vfs);

		vfinfo->dbdf = p_func->bdf
			| (pci_domain_nr(p_func->pci_dev->bus) << 16);
		if ((vf_candidate >> p_func->func_id) & 1)
			vfinfo->vf_state = true;
		else
			vfinfo->vf_state = false;

		part = &p_func->adapt->partition[VF0_INDEX + p_func->func_id];
		vfinfo->fb_size = part->slot.size;

		if (false == p_adapter->pf.user_option.valid) {
			if (p_adapter->sched_opt == SCHEDULER__PREDICT_PERF)
				vfinfo->gfx_engine_part = 100 /
				get_scheduled_func(p_adapter);
			else
				vfinfo->gfx_engine_part = 100 /
				p_adapter->enabled_vfs;
		} else {
			if (false == p_func->user_option.valid)
				vfinfo->gfx_engine_part = 0;
			else
				vfinfo->gfx_engine_part = 100 /
				p_func->user_option.gfx_time_divider;
		}
	}

	return 0;
}

static int amdgim_op_gpuvf_vf(char *param, void *obj, void *result)
{
	int asic;
	unsigned int temp;
	unsigned int vf_candidate;

	struct TIMEVALTYPE cur_time;
	struct function *p_func;
	struct partition *part;
	struct adapter *p_adapter;
	struct timespec kernel_time;
	struct amdgim_vf_detail *vfdetail;

	p_func = (struct function *)obj;
	p_adapter = p_func->adapt;
	vfdetail = (struct amdgim_vf_detail *)result;

	asic = amdgim_get_asic_index(p_adapter);

	vfdetail->type = AMDGIM_GPUVF_VF;
	vfdetail->gpu_dbdf = p_adapter->pf.bdf
		| (pci_domain_nr(p_adapter->pf.pci_dev->bus) << 16);
	vfdetail->vf_dbdf = p_func->bdf
		| (pci_domain_nr(p_func->pci_dev->bus) << 16);
	vf_candidate = amdgim_get_vf_candidate(p_adapter);

	if ((vf_candidate >> p_func->func_id) & 1)
		vfdetail->vf_state = true;
	else
		vfdetail->vf_state = false;

	vfdetail->gpu_active_vf = 0;
	temp = vf_candidate;
	while (temp != 0) {
		vfdetail->gpu_active_vf++;
		temp = (temp - 1) & temp;
	}

	sprintf(vfdetail->vf_name, "%s%d",
		asic_info_table[asic][AMDGIM_ASICINFO_VF_PREFIX],
		p_adapter->enabled_vfs);

	sprintf(vfdetail->vf_type, "%s",
		asic_info_table[asic][AMDGIM_ASICINFO_NAME]);

	vfdetail->guest_driver_cert = 0;
	strncpy(vfdetail->guest_driver_version,
			AMDGIM_STR_NA, AMDGIM_STRLEN_LONG - 1);

	vfdetail->time_log = p_func->time_log;
	do_gettimeofday(&cur_time);

	if (vfdetail->vf_state)
		vfdetail->vf_running_section = cur_time.tv_sec
				 - vfdetail->time_log.init_start.tv_sec;
	else
		vfdetail->vf_running_section = 0;

	if (vfdetail->vf_state) {
		vfdetail->vf_active_section =
				 p_func->time_log.active_time.tv_sec;
		/* if this is the only one running and self switch is off,
		 * add its running time
		 */
		if (vfdetail->gpu_active_vf == 1
			&& p_adapter->switch_to_itself == false) {
			getnstimeofday(&kernel_time);
			vfdetail->vf_active_section += kernel_time.tv_sec
				 - p_func->time_log.active_last_tick.tv_sec;
		}
	} else{
		vfdetail->vf_active_section = 0;
	}

	part =  &p_adapter->partition[VF0_INDEX + p_func->func_id];
	vfdetail->fbsize = part->slot.size;
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

int amdgim_do_clrvffb(struct function *p_func, unsigned char pattern)
{
	struct adapter *p_adapter = p_func->adapt;
	unsigned long long tmp_mmr_base = 0;
	unsigned long long vf_base_addr = 0;
	unsigned long long vf_size = 0;
	unsigned long long size_to_clear = 0;
	unsigned long long vf_size_left = 0;
	unsigned int nonsurface_mc = 0;
	unsigned int vf_candidate;
	unsigned int clear_tick;
	unsigned int ret = AMDGIM_ERROR_MONITOR_SUCCESS;
	mutex_lock(&get_monitor_driver()->core_func_mutex);
	if (true == p_adapter->pf.user_option.valid &&
		false == p_func->user_option.valid){
		/* if the adapter is manually set but the vf is not,
		 * refuse the clear request
		 */
		ret =  AMDGIM_ERROR_MONITOR_CLRVFFB_VF_UNSET;
		goto failed;
	}

	vf_candidate = amdgim_get_vf_candidate(p_adapter);

	if ((vf_candidate >> p_func->func_id) & 1) {
		gim_err("The VF is in use. FB can't be cleared");
		ret =  AMDGIM_ERROR_MONITOR_CLRVFFB_VF_INUSE;
		goto failed;
	}

	/* fb base addr and size */
	vf_base_addr = p_func->fb_partition->slot.base;
	vf_size = p_func->fb_partition->slot.size;

	gim_info("Start to cleaning VF %d", p_func->func_id);
	nonsurface_mc = read_reg32(p_adapter->pf.mmr_base,
					mmHDP_NONSURFACE_BASE);
	write_reg32(p_adapter->pf.mmr_base, mmHDP_NONSURFACE_BASE,
			nonsurface_mc + AMDGIM_MB_TO_MC_SIZE(vf_base_addr));

	for (vf_size_left = vf_size;
		vf_size_left > 0;
		vf_size_left -= size_to_clear) {
		if (vf_size_left > AMDGIM_CLEAR_SIZE_IN_MB)
			size_to_clear = AMDGIM_CLEAR_SIZE_IN_MB;
		else
			size_to_clear = vf_size_left;

		tmp_mmr_base = vf_base_addr + vf_size - vf_size_left;
		write_reg32(p_adapter->pf.mmr_base,
			mmHDP_NONSURFACE_BASE,
			nonsurface_mc + AMDGIM_MB_TO_MC_SIZE(tmp_mmr_base));

		for (clear_tick = 0;
			clear_tick < AMDGIM_MB_TO_BYTE(size_to_clear);
			clear_tick += AMDGIM_CLEAR_SIZE_TICK_BYTE)
			memset(p_adapter->pf.fb_va + clear_tick,
				pattern, AMDGIM_CLEAR_SIZE_TICK_BYTE);

		gim_info("cleared offset=%llx, size=%lld",
			vf_base_addr + vf_size - vf_size_left,
			size_to_clear);
	}

	write_reg32(p_adapter->pf.mmr_base,
			mmHDP_NONSURFACE_BASE,
			nonsurface_mc);

	gim_info("Clean up frame buffer offset=%llx,size=%lldMB",
		vf_base_addr, vf_size);

failed:
	mutex_unlock(&get_monitor_driver()->core_func_mutex);
	return ret;
}

static int amdgim_op_clrvffb(char *param, void *obj, void *result)
{
	int ret;
	struct function *p_func = (struct function *)obj;
	unsigned int pattern;
	unsigned int error = AMDGIM_ERROR_MONITOR_SUCCESS;
	/*parse param*/
	if (strnlen(param, AMDGIM_STRLEN_NORMAL) > AMDGIM_STRLEN_VERYSHORT) {
		error = AMDGIM_ERROR_MONITOR_INVALID_PARAM;
		goto out;
	} else{
		ret = kstrtouint(param, 10, &pattern);
		if (ret != 0) {
			gim_err("failed to get pattern for clrvffb");
			error = AMDGIM_ERROR_MONITOR_INVALID_PARAM;
			goto out;
		}
	}
	error = amdgim_do_clrvffb(p_func, (unsigned char)pattern);

out:
	if (error != AMDGIM_ERROR_MONITOR_SUCCESS)
		sprintf(result,
			"Failed to do clrvffb with error: %s\n",
			amdgim_error_monitor[error]);
	else
		sprintf(result, "OK\n");

	return error;
}

int amdgim_do_getvf(struct adapter *p_adapter,
	struct amdgim_getvf_option option, struct function **ret_func)
{
	int vf_candidate;
	int i;
	struct function *vf = NULL;
	int occupied_base[MAX_VIRTUAL_FUNCTIONS] = {0};
	int occupied_top[MAX_VIRTUAL_FUNCTIONS] = {0};
	int free_base;
	int free_top;
	int fb_size;
	int total_fb_top;
	struct partition *partition;
	int ret = AMDGIM_ERROR_MONITOR_SUCCESS;
	struct amdgim_user_option user_option;

	user_option.fb_offset = option.fb_offset;
	user_option.fb_size = option.fb_size;
	user_option.gfx_time_divider = option.gfx_divider;

	*ret_func = NULL;
	mutex_lock(&get_monitor_driver()->core_func_mutex);

	/* offset is from 16M to max */
	if (user_option.fb_offset < p_adapter->vf_fb_base &&
		user_option.fb_offset != 0) {
		ret = AMDGIM_ERROR_MONITOR_GETVF_INVALID_FB_OFFSET;
		goto failed;
	}
	/* power of 2, value<=16 */
	if (!(((user_option.gfx_time_divider-1) & user_option.gfx_time_divider)
		== 0 && user_option.gfx_time_divider <= 16)){
		ret = AMDGIM_ERROR_MONITOR_GETVF_INVALID_GFX_PART;
		goto failed;
	}

	if (user_option.fb_size != 0 && (user_option.fb_size < 256 ||
		user_option.fb_size != user_option.fb_size / 16 * 16)) {
		ret = AMDGIM_ERROR_MONITOR_GETVF_INVALID_FB_SIZE;
		goto failed;
	}

	vf_candidate = amdgim_get_vf_candidate(p_adapter);

	for (i = 0; i < p_adapter->enabled_vfs; ++i) {
		/* find a vf is not living and not configured */
		if (!((vf_candidate >> i) & 1) && false ==
			p_adapter->vfs[i].user_option.valid) {
			vf = &p_adapter->vfs[i];
			break;
		}

	}

	if (vf == NULL) {
		ret = AMDGIM_ERROR_MONITOR_GETVF_NO_VF_AVAILABLE;
		goto failed;
	}

	/* all are not 0 */
	if (user_option.fb_size * user_option.gfx_time_divider *
		user_option.fb_offset > 0 && vf != NULL) {
		if (false == p_adapter->pf.user_option.valid) {
			if (vf_candidate != 0) {
				ret = AMDGIM_ERROR_MONITOR_GETVF_OSG_VF_INUSE;
				goto failed;
			}
		} else {
			if (p_adapter->pf.user_option.mode ==
				AMDGIM_OPTION_MODE_ZZZ) {
				ret = AMDGIM_ERROR_MONITOR_GETVF_OSG_IN_ZZZ;
				goto failed;
			}
		}
		/* try if there is quota left and calculate the quota */
		user_option.gfx_time_quota = amdgim_get_quota_ms(p_adapter,
			user_option.gfx_time_divider);
		if (user_option.gfx_time_quota == 0) {
			ret = AMDGIM_ERROR_MONITOR_GETVF_GFX_PART_NOT_ENOUGH;
			goto failed;
		}

		/* calculate the fb_size */
		fb_size = user_option.fb_size;
		/* try to find if there is enough fb
		 * scan all vfs
		 */
		for (i = 0; i < p_adapter->enabled_vfs; ++i) {
			if ((vf_candidate >> i & 1) || true ==
				p_adapter->vfs[i].user_option.valid) {
				occupied_base[i] =
				p_adapter->vfs[i].fb_partition->slot.base;
				occupied_top[i] =
				p_adapter->vfs[i].fb_partition->slot.base +
				p_adapter->vfs[i].fb_partition->slot.size;
			}
		}

		free_base = user_option.fb_offset;
		free_top = free_base + fb_size;
		for (i = 0; i < p_adapter->enabled_vfs; ++i) {
			if (occupied_base[i] != 0) {
				/* find an occupied vf
				 * if the mem in the range
				 */
				if ((free_base <= occupied_base[i] &&
					occupied_base[i] < free_top) ||
					(free_base < occupied_top[i]
					&& occupied_top[i] <= free_top)){
					gim_info("The partition hit the"
					" occupied memory of VF %d,"
					" base=%x,size=%d", i,
					occupied_base[i], occupied_top[i]);
					free_base = occupied_top[i];
					free_top = free_base + fb_size;
					/* the offset is specified, so if the
					 * fb is occupied, it is already failed
					 */
					break;
				}
			}
		}

		total_fb_top = p_adapter->vf_fb_base + p_adapter->vf_fb_size;

		if (free_top <= total_fb_top && free_base ==
			user_option.fb_offset) {
			partition = vf->fb_partition;
			partition->slot.base = free_base;
			partition->slot.size = fb_size;
			set_gpuiov_vf_frame_buffer(p_adapter->pf.pci_dev,
				&p_adapter->gpuiov, vf->func_id,
				partition->slot.size,
				partition->slot.base);
			gim_info("Set VF %d, BDF=%02x:%02x.%x, fb_base=%llx,"
				" fb_size=%lld, quota=%dms",
				vf->func_id, AMDGIM_GET_PCI_BUS(vf->bdf),
				AMDGIM_GET_PCI_DEVICE(vf->bdf),
				AMDGIM_GET_PCI_FUNCTION(vf->bdf),
				partition->slot.base,
				partition->slot.size,
				user_option.gfx_time_quota);
			user_option.valid = true;
			user_option.mode = AMDGIM_OPTION_MODE_OSG;
			vf->user_option = user_option;
			p_adapter->pf.user_option.valid = true;
			p_adapter->pf.user_option.mode =
				AMDGIM_OPTION_MODE_OSG;
			*ret_func = vf;

		} else {
			ret =
			AMDGIM_ERROR_MONITOR_GETVF_CANT_ALLOC_OFFSET_AND_SIZE;
			goto failed;
		}

	} else if ((user_option.fb_size * user_option.gfx_time_divider *
		user_option.fb_offset) == 0 && (user_option.fb_size +
		user_option.gfx_time_divider + user_option.fb_offset) > 0) {
		ret = AMDGIM_ERROR_MONITOR_INVALID_PARAM;
		goto failed;
	} else if (0 == (user_option.fb_size + user_option.gfx_time_divider +
		user_option.fb_offset)) {
		if (true == p_adapter->pf.user_option.valid) {
			if (p_adapter->pf.user_option.mode !=
				AMDGIM_OPTION_MODE_ZZZ){
				ret = AMDGIM_ERROR_MONITOR_GETVF_ZZZ_IN_OSG;
				goto failed;
			}
		}
		p_adapter->pf.user_option.mode = AMDGIM_OPTION_MODE_ZZZ;
		p_adapter->pf.user_option.valid = true;
		vf->user_option.valid = true;
		*ret_func = vf;
	} /* if can't find available vf */
failed:
	mutex_unlock(&get_monitor_driver()->core_func_mutex);
	return ret;
}

int amdgim_op_getvf(char *param, void *obj, void *result)
{
	struct adapter *p_adapter = ((struct function *)obj)->adapt;
	struct amdgim_getvf_option getvf_option;
	struct function *ret_func;
	unsigned int error = AMDGIM_ERROR_MONITOR_SUCCESS;

	error = amdgim_parse_getvf(param, &getvf_option);
	if (error != AMDGIM_ERROR_MONITOR_SUCCESS)
		goto out;

	error = amdgim_do_getvf(p_adapter, getvf_option, &ret_func);

out:
	if (error != AMDGIM_ERROR_MONITOR_SUCCESS) {
		sprintf(result,
			"Failed to do getvf with error: %s\n",
			amdgim_error_monitor[error]);
	} else {
		if (ret_func != NULL) {
			sprintf(result, "%04x:%02x:%02x.%x\n",
				pci_domain_nr(p_adapter->pf.pci_dev->bus),
				AMDGIM_GET_PCI_BUS(ret_func->bdf),
				AMDGIM_GET_PCI_DEVICE(ret_func->bdf),
				AMDGIM_GET_PCI_FUNCTION(ret_func->bdf));
		} else {
			sprintf(result, "Can't find available VF\n");
		}
	}

	return error;
}

int amdgim_do_relvf(struct adapter *p_adapter, union amdgim_dbdf dbdf)
{
	struct function *p_func;
	int i;
	int vf_candidate;
	unsigned long long fb_base;
	unsigned long long fb_size;
	struct resource *res;
	unsigned long long mini_fb_size;
	struct partition *partition;
	unsigned int vf_num;
	unsigned int ret = AMDGIM_ERROR_MONITOR_SUCCESS;

	mutex_lock(&get_monitor_driver()->core_func_mutex);
	p_func = bdf_to_function(dbdf.u32all & 0xffff);
	if (p_func == NULL || p_func->adapt != p_adapter) {
		ret = AMDGIM_ERROR_MONITOR_RELVF_VF_NOT_FOUND;
		goto failed;
	}

	vf_candidate = amdgim_get_vf_candidate(p_adapter);
	if ((vf_candidate >> p_func->func_id) & 1) {
		ret = AMDGIM_ERROR_MONITOR_RELVF_VF_INUSE;
		goto failed;
	}

	if (false == p_func->user_option.valid) {
		ret = AMDGIM_ERROR_MONITOR_RELVF_VF_UNSET;
		goto failed;
	}

	if (p_func->user_option.mode == AMDGIM_OPTION_MODE_OSG) {

		res = &p_adapter->vfs[0].pci_dev->resource[BAR__FRAME_BUFFER];
		mini_fb_size = TO_MBYTES(res->end - res->start + 1);

		vf_num = p_adapter->enabled_vfs;

		fb_size = get_fb_size_static(p_adapter, &vf_num,
			p_adapter->vf_fb_base, p_adapter->vf_fb_size,
			mini_fb_size);

		fb_base = p_adapter->vf_fb_base + fb_size * p_func->func_id;

		partition = p_func->fb_partition;
		partition->slot.base = fb_base;
		partition->slot.size = fb_size;

		set_gpuiov_vf_frame_buffer(p_adapter->pf.pci_dev,
			&p_adapter->gpuiov, p_func->func_id,
			partition->slot.size, partition->slot.base);
		p_func->user_option.gfx_time_quota =
			amdgim_get_total_time_slice(p_adapter) /
			p_adapter->enabled_vfs;
	}

	p_func->user_option.valid = false;
	/* if all VFs are released, release the whole GPU */
	p_adapter->pf.user_option.valid = false;
	for (i = 0; i < p_adapter->enabled_vfs; ++i) {
		if (true == p_adapter->vfs[i].user_option.valid) {
			p_adapter->pf.user_option.valid = true;
			break;
		}
	}
failed:
	mutex_unlock(&get_monitor_driver()->core_func_mutex);
	return ret;

}

int amdgim_op_relvf(char *param, void *obj, void *result)
{
	struct adapter *p_adapter = ((struct function *)obj)->adapt;
	union amdgim_dbdf dbdf;
	int error = AMDGIM_ERROR_MONITOR_SUCCESS;

	error = amdgim_parse_relvf(param, &dbdf);
	if (error != AMDGIM_ERROR_MONITOR_SUCCESS)
		goto out;

	error = amdgim_do_relvf(p_adapter, dbdf);

out:
	if (error != AMDGIM_ERROR_MONITOR_SUCCESS)
		sprintf(result, "Failed to do relvf with error: %s\n",
		amdgim_error_monitor[error]);
	else
		sprintf(result, "OK\n");

	return error;
}

int amdgim_op_flr(char *param, void *obj, void *result)
{
	struct adapter *p_adapter = ((struct function *)obj)->adapt;
	struct function *p_func = (struct function *)obj;
	unsigned int vf_candidate;
	int ret;

	vf_candidate = amdgim_get_vf_candidate(p_adapter);

	if (!((vf_candidate >> p_func->func_id) & 1)) {
		sprintf(result, "OK\n");
		return AMDGIM_ERROR_MONITOR_SUCCESS;
	}

	ret = gim_sched_reset_vf(p_adapter, p_func, NULL,
		FLR_REASON_FAILED_IDLE);

	if (ret != 0) {
		sprintf(result, "Failed to reset the VF\n");
		return AMDGIM_ERROR_MONITOR_UNKNOWN;
	}

	sprintf(result, "OK\n");
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

int amdgim_op_hotlink_reset(char *param, void *obj, void *result)
{
	struct adapter *p_adapter = ((struct function *)obj)->adapt;
	int ret;

	ret = gim_sched_reset_gpu(p_adapter);

	if (ret != 0) {
		sprintf(result, "Failed to do hotlink reset\n");
		return AMDGIM_ERROR_MONITOR_UNKNOWN;
	}

	sprintf(result, "OK\n");
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}


/* OP TO STR */
static int amdgim_op2str_gpuinfo(void *obj, char *str)
{
	int i;
	char buf[AMDGIM_STRLEN_VERYLONG];
	struct amdgim_gpuinfos *p_gpuinfos;

	*str = 0;

	p_gpuinfos = (struct amdgim_gpuinfos *)obj;
	for (i = 0; i < p_gpuinfos->gpu_count; ++i) {
		sprintf(buf, "GPU #%d\n", i);
		strcat(str, buf);

		sprintf(buf, "\tName:%s\n", p_gpuinfos->gpuinfo[i].name);
		strcat(str, buf);

		sprintf(buf, "\tBusId:%04x:%02x:%02x.%x\n",
			AMDGIM_GET_PCI_DOMAIN(p_gpuinfos->gpuinfo[i].dbdf),
			AMDGIM_GET_PCI_BUS(p_gpuinfos->gpuinfo[i].dbdf),
			AMDGIM_GET_PCI_DEVICE(p_gpuinfos->gpuinfo[i].dbdf),
			AMDGIM_GET_PCI_FUNCTION(p_gpuinfos->gpuinfo[i].dbdf));
		strcat(str, buf);

		sprintf(buf, "\tDPM Cap:%d\n", p_gpuinfos->gpuinfo[i].dpm_cap);
		strcat(str, buf);

		sprintf(buf, "\tPower Cap:%d W\n",
			p_gpuinfos->gpuinfo[i].power_cap);
		strcat(str, buf);

		sprintf(buf, "\tFrame Buffer Size:%d M\n",
			p_gpuinfos->gpuinfo[i].fbsize);
		strcat(str, buf);

		if (p_gpuinfos->gpuinfo[i].uncorr_ecc)
			sprintf(buf, "\tUncorr ECC:%s\n", AMDGIM_STR_YES);
		else
			sprintf(buf, "\tUncorr ECC:%s\n", AMDGIM_STR_NO);
		strcat(str, buf);

		sprintf(buf, "\tMax VF#:%d\n",
			p_gpuinfos->gpuinfo[i].max_vf_num);
		strcat(str, buf);

		sprintf(buf, "\tGFX Engine:%s\n",
			p_gpuinfos->gpuinfo[i].gfx_engine);
		strcat(str, buf);

		sprintf(buf, "\tGFX MAX Clock:%d MHz\n",
			p_gpuinfos->gpuinfo[i].max_gfx_clk);
		strcat(str, buf);

		sprintf(buf, "\tVideo Encoder:%s\n",
			p_gpuinfos->gpuinfo[i].video_encoder);
		strcat(str, buf);

		sprintf(buf, "\tPCIE Link Speed:%d GT/s\n",
			p_gpuinfos->gpuinfo[i].pcie_speed >> 1);
		strcat(str, buf);

		sprintf(buf, "\tPCIE Link Width:x%d\n",
			p_gpuinfos->gpuinfo[i].pcie_width);
		strcat(str, buf);
	}
	return 0;
}

static int amdgim_op2str_gpubios(void *obj, char *str)
{
	struct amdgim_vbios_infos *info;
	unsigned int version;
	int i;
	char buf[AMDGIM_STRLEN_VERYLONG];
	*str = 0;

	info = (struct amdgim_vbios_infos *)obj;
	for (i = 0; i < info->gpu_count; ++i) {
		sprintf(buf, "GPU #%d\n", i);
		strcat(str, buf);

		sprintf(buf, "\tSerial ID:%s\n",
			info->vbiosinfos[i].serial);
		strcat(str, buf);

		sprintf(buf, "\tVBIOS Pn:%s\n",
			info->vbiosinfos[i].vbios_pn);
		strcat(str, buf);

		sprintf(buf, "\tVBIOS Build Date:%s\n",
			info->vbiosinfos[i].date);
		strcat(str, buf);

		version = info->vbiosinfos[i].version;
		sprintf(buf, "\tVBIOS Version:%d.%d.%d.%d\n",
				(version >> 24) & 0x000000ff,
				(version >> 16) & 0x000000ff,
				(version >>  8) & 0x000000ff,
				(version) & 0x000000ff);
		strcat(str, buf);

		sprintf(buf, "\tASIC Device ID:%x\n",
			info->vbiosinfos[i].dev_id);
		strcat(str, buf);

		sprintf(buf, "\tASIC Revision ID:%x\n",
			info->vbiosinfos[i].rev_id);
		strcat(str, buf);

		sprintf(buf, "\tSubSystem Vendor ID:%x\n",
			info->vbiosinfos[i].sub_ved_id);
		strcat(str, buf);

		sprintf(buf, "\tSubSystem Device ID:%x\n",
			info->vbiosinfos[i].sub_dev_id);
		strcat(str, buf);
	}
	return 0;
}

static int amdgim_op2str_gpuvs(void *obj, char *str)
{
	int asic;
	unsigned int i = 0;

	struct amdgim_gpuvs_group *p_gpuvs_g;
	struct amdgim_gpuvs *p_gpuvs;
	struct adapter *p_adapter_list;
	struct amdgim_asic_operation *asic_op;
	char buf[AMDGIM_STRLEN_VERYLONG];

	p_gpuvs_g = (struct amdgim_gpuvs_group *)obj;
	p_adapter_list = get_adapters();
	for (i = 0; i < p_gpuvs_g->gpu_count; ++i) {
		p_gpuvs = &p_gpuvs_g->gpuvs[i];
		sprintf(buf, "GPU #%d\n", i);
		strcat(str, buf);
		asic = amdgim_get_asic_index(&p_adapter_list[i]);
		asic_op = amdgim_get_asic_op(asic);

		if (asic_op != NULL
			&& asic_op->amdgim_tostrall != NULL)
			asic_op->amdgim_tostrall(&p_adapter_list[i],
						p_gpuvs, str);
		else
			gim_err("Failed to convert gpuvs info");
	}
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

static int amdgim_op2str_gpuvf_pf(void *obj, char *str)
{
	struct amdgim_gpu_vfinfos *gpuinfo;
	struct amdgim_vfinfo *vfinfo;
	unsigned int i;
	char buf[AMDGIM_STRLEN_VERYLONG];

	gpuinfo = (struct amdgim_gpu_vfinfos *)obj;
	for (i = 0; i < gpuinfo->vf_count; ++i) {
		vfinfo = &gpuinfo->vf_info[i];
		sprintf(buf, "VF #%d\n", vfinfo->vf_id);
		strcat(str, buf);
		sprintf(buf, "\tType:%s\n", gpuinfo->name);
		strcat(str, buf);
		sprintf(buf, "\tBusId:%04x:%02x:%02x.%x\n",
			AMDGIM_GET_PCI_DOMAIN(vfinfo->dbdf),
			AMDGIM_GET_PCI_BUS(vfinfo->dbdf),
			AMDGIM_GET_PCI_DEVICE(vfinfo->dbdf),
			AMDGIM_GET_PCI_FUNCTION(vfinfo->dbdf));

		strcat(str, buf);
		sprintf(buf, "\tName:%s\n", vfinfo->vf_name);
		strcat(str, buf);

		if (vfinfo->vf_state)
			sprintf(buf, "\tVF State:%s\n", AMDGIM_STR_ACTIVE);
		else
			sprintf(buf, "\tVF State:%s\n", AMDGIM_STR_AVAILABLE);
		strcat(str, buf);

		sprintf(buf, "\tVF Size:%d M\n", vfinfo->fb_size);
		strcat(str, buf);

		sprintf(buf, "\tGFX engine partition:%d%%\n",
			vfinfo->gfx_engine_part);
		strcat(str, buf);
	}
	return 0;
}

static int amdgim_op2str_gpuvf_vf(void *obj, char *str)
{
	struct amdgim_vf_detail *vfdetail = (struct amdgim_vf_detail *)obj;
	struct rtc_time tm;
	unsigned int hour;
	unsigned int minute;
	unsigned long long sec;
	char buf[AMDGIM_STRLEN_VERYLONG];

	/* vf detail info */
	sprintf(buf, "GPU:%04x:%02x:%02x.%x\n",
			AMDGIM_GET_PCI_DOMAIN(vfdetail->gpu_dbdf),
			AMDGIM_GET_PCI_BUS(vfdetail->gpu_dbdf),
			AMDGIM_GET_PCI_DEVICE(vfdetail->gpu_dbdf),
			AMDGIM_GET_PCI_FUNCTION(vfdetail->gpu_dbdf));
	strcat(str, buf);

	sprintf(buf, "\tActive vGPUs:%d\n", vfdetail->gpu_active_vf);
	strcat(str, buf);

	sprintf(buf, "\tvGPU ID:%04x:%02x:%02x.%x\n",
			AMDGIM_GET_PCI_DOMAIN(vfdetail->vf_dbdf),
			AMDGIM_GET_PCI_BUS(vfdetail->vf_dbdf),
			AMDGIM_GET_PCI_DEVICE(vfdetail->vf_dbdf),
			AMDGIM_GET_PCI_FUNCTION(vfdetail->vf_dbdf));
	strcat(str, buf);

	sprintf(buf, "\t\tvGPU Name:%s\n", vfdetail->vf_name);
	strcat(str, buf);

	sprintf(buf, "\t\tvGPU Type:%s\n", vfdetail->vf_type);
	strcat(str, buf);

	sprintf(buf, "\t\tGuest Driver Version:%s\n",
		 vfdetail->guest_driver_version);
	strcat(str, buf);

	sprintf(buf, "\t\tGuest Driver Certification:%s\n",
		(vfdetail->guest_driver_cert == 1)?"WHQL":"None");
	strcat(str, buf);


	if (vfdetail->vf_state)
		sprintf(buf, "\t\tvGPU State:%s\n", AMDGIM_STR_ACTIVE);
	else
		sprintf(buf, "\t\tvGPU State:%s\n", AMDGIM_STR_AVAILABLE);
	strcat(str, buf);

	/* get time */
	sec = vfdetail->vf_running_section;
	hour = sec / AMDGIM_HOUR_IN_SEC;
	sec = sec % AMDGIM_HOUR_IN_SEC;
	minute = sec / AMDGIM_MINUTE_IN_SEC;
	sec = sec % AMDGIM_MINUTE_IN_SEC;

	sprintf(buf, "\t\tVF current running section:%d:%d:%lld\n",
		hour, minute, sec);
	strcat(str, buf);

	/* get active time */
	sec = vfdetail->vf_active_section;
	hour = sec / AMDGIM_HOUR_IN_SEC;
	sec = sec % AMDGIM_HOUR_IN_SEC;
	minute = sec / AMDGIM_MINUTE_IN_SEC;
	sec = sec % AMDGIM_MINUTE_IN_SEC;

	sprintf(buf, "\t\tVF active section:%d:%d:%lld\n", hour, minute, sec);
	strcat(str, buf);

	/* init start */
	if (vfdetail->time_log.init_start.tv_sec > 0) {
		rtc_time_to_tm(vfdetail->time_log.init_start.tv_sec
				- sys_tz.tz_minuteswest * 60, &tm);
		sprintf(buf, "\t\tVF Last Init Start:" TIME_FMRT,
				tm.tm_year + 1900,
				tm.tm_mon + 1,
				tm.tm_mday,
				tm.tm_hour,
				tm.tm_min,
				tm.tm_sec);
	} else{
		sprintf(buf, "\t\tVF Last Init Start:%s\n", AMDGIM_STR_NA);
	}
	strcat(str, buf);

	/* init end */
	if (vfdetail->time_log.init_end.tv_sec > 0) {
		rtc_time_to_tm(vfdetail->time_log.init_end.tv_sec
				- sys_tz.tz_minuteswest * 60, &tm);
		sprintf(buf, "\t\tVF Last Init Finish:" TIME_FMRT,
				tm.tm_year + 1900,
				tm.tm_mon + 1,
				tm.tm_mday,
				tm.tm_hour,
				tm.tm_min,
				tm.tm_sec);
	} else{
		sprintf(buf, "\t\tVF Last Init Finish:%s\n", AMDGIM_STR_NA);
	}
	strcat(str, buf);

	/* finish start */
	if (vfdetail->time_log.finish_start.tv_sec > 0) {
		rtc_time_to_tm(vfdetail->time_log.finish_start.tv_sec
				- sys_tz.tz_minuteswest * 60, &tm);
		sprintf(buf, "\t\tVF Last Shutdown Start:" TIME_FMRT,
				tm.tm_year + 1900,
				tm.tm_mon + 1,
				tm.tm_mday,
				tm.tm_hour,
				tm.tm_min,
				tm.tm_sec);
	} else{
		sprintf(buf, "\t\tVF Last Shutdown Start:%s\n", AMDGIM_STR_NA);
	}
	strcat(str, buf);
	/* finish end */
	if (vfdetail->time_log.finish_end.tv_sec > 0) {
		rtc_time_to_tm(vfdetail->time_log.finish_end.tv_sec
				- sys_tz.tz_minuteswest * 60, &tm);
		sprintf(buf,
			"\t\tVF Last Shutdown Finish:" TIME_FMRT,
			tm.tm_year + 1900,
			tm.tm_mon + 1,
			tm.tm_mday,
			tm.tm_hour,
			tm.tm_min,
			tm.tm_sec);
	} else{
		sprintf(buf, "\t\tVF Last Shutdown Finish:%s\n", AMDGIM_STR_NA);
	}
	strcat(str, buf);

	/* reset time */
	if (vfdetail->time_log.reset_time.tv_sec > 0) {
		rtc_time_to_tm(vfdetail->time_log.reset_time.tv_sec
				 - sys_tz.tz_minuteswest * 60,
				 &tm);
		sprintf(buf,
			"\t\tVF Last Reset At:" TIME_FMRT,
			tm.tm_year + 1900,
			tm.tm_mon + 1,
			tm.tm_mday,
			tm.tm_hour,
			tm.tm_min,
			tm.tm_sec);
	} else{
		sprintf(buf, "\t\tVF Last Reset At:%s\n", AMDGIM_STR_NA);
	}
	strcat(str, buf);

	sprintf(buf, "\t\tVF Reset Time:%d\n", vfdetail->time_log.reset_count);
	strcat(str, buf);

	sprintf(buf, "\t\tVF FB Size:%d M\n", vfdetail->fbsize);
	strcat(str, buf);
	return 0;

}

static int amdgim_op2str_clrvffb(void *obj, char *str)
{
	amdgim_put_info(obj, str);
	return 0;
}

static int amdgim_op2str_getvf(void *obj, char *str)
{
	amdgim_put_info(obj, str);
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

static int amdgim_op2str_relvf(void *obj, char *str)
{
	amdgim_put_info(obj, str);
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

static int amdgim_op2str_flr(void *obj, char *str)
{
	amdgim_put_info(obj, str);
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

static int amdgim_op2str_hotlink_reset(void *obj, char *str)
{
	amdgim_put_info(obj, str);
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

static struct amdgim_command_op command_ops[AMDGIM_COMMAND_LEN] = {
	[AMDGIM_COMMAND_GPUINFO] = {AMDGIM_FUNC_ENTRY(amdgim_op_gpuinfo)},
	[AMDGIM_COMMAND_GPUVS] = {AMDGIM_FUNC_ENTRY(amdgim_op_gpuvs)},
	[AMDGIM_COMMAND_GPUBIOS] = {AMDGIM_FUNC_ENTRY(amdgim_op_gpubios)},
	[AMDGIM_COMMAND_GPUVF_PF] = {AMDGIM_FUNC_ENTRY(amdgim_op_gpuvf_pf)},
	[AMDGIM_COMMAND_GPUVF_VF] = {AMDGIM_FUNC_ENTRY(amdgim_op_gpuvf_vf)},
	[AMDGIM_COMMAND_CLRVFFB] = {AMDGIM_FUNC_ENTRY(amdgim_op_clrvffb)},
	[AMDGIM_COMMAND_GETVF] = {AMDGIM_FUNC_ENTRY(amdgim_op_getvf)},
	[AMDGIM_COMMAND_RELVF] = {AMDGIM_FUNC_ENTRY(amdgim_op_relvf)},
	[AMDGIM_COMMAND_FLR] = {AMDGIM_FUNC_ENTRY(amdgim_op_flr)},
	[AMDGIM_COMMAND_HOTLINK_RESET] = {
		AMDGIM_FUNC_ENTRY(amdgim_op_hotlink_reset)},
};

static struct amdgim_command_to_str command_ops_tostr[AMDGIM_COMMAND_LEN] = {
	[AMDGIM_COMMAND_GPUINFO] = {AMDGIM_FUNC_ENTRY(amdgim_op2str_gpuinfo)},
	[AMDGIM_COMMAND_GPUVS] = {AMDGIM_FUNC_ENTRY(amdgim_op2str_gpuvs)},
	[AMDGIM_COMMAND_GPUBIOS] = {AMDGIM_FUNC_ENTRY(amdgim_op2str_gpubios)},
	[AMDGIM_COMMAND_GPUVF_PF] = {AMDGIM_FUNC_ENTRY(amdgim_op2str_gpuvf_pf)},
	[AMDGIM_COMMAND_GPUVF_VF] = {AMDGIM_FUNC_ENTRY(amdgim_op2str_gpuvf_vf)},
	[AMDGIM_COMMAND_CLRVFFB] = {AMDGIM_FUNC_ENTRY(amdgim_op2str_clrvffb)},
	[AMDGIM_COMMAND_GETVF] = {AMDGIM_FUNC_ENTRY(amdgim_op2str_getvf)},
	[AMDGIM_COMMAND_RELVF] = {AMDGIM_FUNC_ENTRY(amdgim_op2str_relvf)},
	[AMDGIM_COMMAND_FLR] = {AMDGIM_FUNC_ENTRY(amdgim_op2str_flr)},
	[AMDGIM_COMMAND_HOTLINK_RESET] = {
		AMDGIM_FUNC_ENTRY(amdgim_op2str_hotlink_reset)},
};

int amdgim_do_op(char *param, void *function, int index, char *result)
{
	void *data;

	data = kzalloc(command_results_len[index], GFP_KERNEL);
	if (data == NULL)
		goto failed_exit;

	if (command_ops[index].func(amdgim_skip_space(param), function, data)) {
		amdgim_put_info(data, result);
		goto direct_exit;
	}

	if (command_ops_tostr[index].func(data, result))
		goto direct_exit;

direct_exit:
	kfree(data);

	return AMDGIM_ERROR_MONITOR_SUCCESS;
failed_exit:
	return AMDGIM_ERROR_MONITOR_UNKNOWN;

}

