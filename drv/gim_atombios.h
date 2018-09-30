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

#ifndef _GPU_IOV_MODULE__GIM_ATOMBIOS_H
#define _GPU_IOV_MODULE__GIM_ATOMBIOS_H

#include "gim_adapter.h"
#include "gim_atombios_com.h"

#define ATOM_DATATABLE                  0
#define ATOM_COMMANDTABLE               1
#define ATOM_MASTER_ROM_HEADERTABLE     2

#define ATOM_ASIC_POSTED                0
#define ATOM_ASIC_NEED_POST		1

#define ONE_K                          0x00000400L
#define HALF_K                         0x00000200L

#define VBIOS_POST_INIT_ASIC      0
#define VBIOS_POST_LOAD_UCODE     1
#define VBIOS_POST_FULL_POST      2
#define VBIOS_POST_REPOST         3
#define VBIOS_POST_UNKNOWN	  4

#define FORCE_VBIOS_POST	10
#define POST_VBIOS_IF_NEEDED	11

#define UPDATE_FIRMWARE_SMU	0x01
#define UPDATE_FIRMWARE_RLCV	0x02
#define UPDATE_FIRMWARE_TOC	0x04
#define UPDATE_FIRMWARE_SMC_PATCH	0x8

#define PPSMC_MSG_THERMAL_CNTL_ENABLE       (0x10a)

int atom_init_parser(struct adapter *adapt);
int atom_post_vbios(struct adapter *adapt, int post_type);
int atom_chk_asic_status(struct adapter *adapt);
int patch_firmware(struct adapter *adapt);

struct cail_parser_table_context {
	uint32_t size;
	uint32_t table_index;           /* indicate index of Command table */
	void     *para_ptr;              /* Parameter space pointer */
	uint32_t parser_result;         /* Save the result from Parser */
};

struct cail_adapter_config_info {
	uint32_t rom_length;		/* Size of ATI ROM in bytes */
	uint32_t rom_header_offset;	/* Offset to ROM header */

	void *rom_base_addr;	/* Mapped logical base addr of ROM */
	void *io_base_addr;    /* IO access address */

};

int atom_dpm_state_cntl(struct adapter *adapt,
			struct pwr_mgt_param *param);

void enable_thermal_control(struct adapter *adapt);

#define GIM_VBIOS_SIGNATURE             "761295520"
#define GIM_VBIOS_SIGNATURE_OFFSET      0x30
#define GIM_VBIOS_SIGNATURE_SIZE        sizeof(GIM_VBIOS_SIGNATURE)
#define GIM_VBIOS_SIGNATURE_END         (GIM_VBIOS_SIGNATURE_OFFSET	\
					+ GIM_VBIOS_SIGNATURE_SIZE)
#define GIM_IS_VALID_VBIOS(p) \
		((p)[0] == 0x55 && (p)[1] == 0xAA)

#define GIM_VBIOS_LENGTH(p) \
		((p)[2] << 9)

#endif
