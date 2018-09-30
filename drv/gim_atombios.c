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

#include <linux/vmalloc.h>
#include "gim_atom.h"
#include "gim_debug.h"
#include "gim_atombios.h"
#include "gim_adapter.h"
#include "gim_atombios_com.h"
#include "gim_os_service.h"
#include "gim_s7150_reg.h"
#include "gim_vbios_patch.h"

void *atom_get_tab_pointer(struct adapter *adapt,
			uint32_t table_type,
			uint32_t table_idx,
			uint32_t table_rev)
{
	struct cail_adapter_config_info *adapter_info;
	void *table = NULL;
	struct atom_rom_header             *atom_rom_hdr;

	adapter_info = (struct cail_adapter_config_info *)(adapt->rom_info);
	atom_rom_hdr = (struct atom_rom_header *)
			((uint64_t)adapter_info->rom_base_addr
			+ adapter_info->rom_header_offset);

	table = (void *)atom_rom_hdr;

	if (table_type != ATOM_MASTER_ROM_HEADERTABLE) {
		struct atom_master_data_table      *data_table;
		struct atom_master_command_table   *cmd_table;
		uint16_t                      *us_table_pointer;

		if (table_type == ATOM_DATATABLE) {
			data_table = (struct atom_master_data_table *)
				   ((uint64_t)adapter_info->rom_base_addr
				   + atom_rom_hdr->master_data_table_offset);

			us_table_pointer =
				(uint16_t *)&(data_table->tables_list);
		} else {
			/* asking for command table */
			cmd_table = (struct atom_master_command_table *)
				((uint64_t)adapter_info->rom_base_addr
				+ atom_rom_hdr->master_command_table_offset);
			us_table_pointer = (uint16_t *)
					&(cmd_table->list_of_command_tables);
		}

		/* get the pointer we want */
		if (us_table_pointer[table_idx])
			/* if the offset is not ZERO */
			table = (void *)((uint64_t)adapter_info->rom_base_addr
					+ us_table_pointer[table_idx]);
		else
			table = NULL;
	}

	return table;
}

int atom_query_table_rev(struct adapter *adapt,
	struct cail_parser_table_context *context)
{
	struct atom_common_rom_command_table_header  *table_hdr;

	if ((context == NULL)
	    || (context->size < sizeof(struct cail_parser_table_context)))
		return -1;

	table_hdr = (struct atom_common_rom_command_table_header *)
			atom_get_tab_pointer(adapt,
					     ATOM_COMMANDTABLE,
					     context->table_index,
					     0);
	if (table_hdr) {
		if (table_hdr->common_header.table_format_revision
		    > PARSER_MAJOR_REVISION)
			return -1;

		context->parser_result =
		      (table_hdr->common_header.table_content_revision << 16)
		      | (table_hdr->common_header.table_format_revision);
		return 0;
	} else
		return -1;
}

int atom_exec_bios_table(struct adapter *adapt,
			  struct cail_parser_table_context *context)
{

	struct device_data parser_ctx;
	enum cd_status parse_result;

	if ((context == NULL)
	    || (context->size < sizeof(struct cail_parser_table_context)))
		return -1;

	/* We need to check if parser is capable to execute. */
	if (atom_query_table_rev(adapt, context) != 0) {
		gim_err("atom_query_table_rev: failed");
		return -1;
	}

	 /* Parameter space */
	parser_ctx.parameter_space = context->para_ptr;
	parser_ctx.dev            = adapt;
	parser_ctx.bios_image     = (uint8_t *)(
					(struct cail_adapter_config_info *)
					adapt->rom_info)->rom_base_addr;
	parser_ctx.format          = TABLE_FORMAT_BIOS;

	parse_result = parse_table((struct device_data *)&parser_ctx,
				   (uint8_t)context->table_index);

	if (parse_result != 0) {
		gim_err("Parse table failed! error %u", parse_result);
		return -1;
	}

	return 0;
}

int atom_init_fan_cntl(struct adapter *adapt)
{
	struct cail_parser_table_context  parser_ctx;
	struct write_byte_hw_i2c_data_parm   speed_fan_cnt_ps;
	struct atom_common_table_header         *table_hdr;
	int                                    ret = 0;
	uint16_t                            tbl_idx;

	tbl_idx     = get_index_into_master_table(command,
					       write_byte_to_hw_assisted_i2c);

	table_hdr = (struct atom_common_table_header *)
			atom_get_tab_pointer(adapt,
					     ATOM_COMMANDTABLE,
					     tbl_idx,
					     0);
	/* there is not write_byte_to_hw_assisted_i2c tabe. */
	if (table_hdr == NULL)
		return 1;

	if ((table_hdr->table_format_revision == 0)
	    || ((table_hdr->table_format_revision == 1)
	    && (table_hdr->table_content_revision < 2))) {
		memset((char *)&speed_fan_cnt_ps,
			0,
			sizeof(struct write_byte_hw_i2c_data_parm));

		parser_ctx.para_ptr = (void *)&speed_fan_cnt_ps;
		parser_ctx.size = sizeof(struct cail_parser_table_context);
		parser_ctx.table_index =
			get_index_into_master_table(command, speed_fan_control);

		ret = atom_exec_bios_table(adapt, &parser_ctx);
	}

	return ret;
}

int atom_post_vbios(struct adapter *adapt, int post_type)
{
	uint16_t	tbl_idx;
	struct atom_firmware_info	*fw_info;
	int ret = 0;


	if (((struct cail_adapter_config_info *)adapt->rom_info)->rom_base_addr
	    == NULL)
		return -1;

	tbl_idx = get_index_into_master_table(data, firmware_info); /* 4 */
	fw_info = (struct atom_firmware_info *)atom_get_tab_pointer(adapt,
							     ATOM_DATATABLE,
							     tbl_idx,
							     0);

	gim_info("ATOM_PostVBIOS: firmware_info passed\n");

	if (fw_info) {
		struct asic_init_ps_alloc_v1_2	asic_init_ps;
		struct cail_parser_table_context	parser_ctx;
		struct atom_common_table_header	*table_hdr;
		struct asic_init_parm_v1_2	*init_clk;

		parser_ctx.table_index = get_index_into_master_table(command,
								     asic_init);
		table_hdr  = (struct atom_common_table_header *)
				atom_get_tab_pointer(adapt,
						    ATOM_COMMANDTABLE,
						    parser_ctx.table_index,
						    0);

		memset((char *)&asic_init_ps,
			0,
			sizeof(struct asic_init_ps_alloc_v1_2));

		init_clk = &asic_init_ps.asic_init_clocks;

		init_clk->sclk_clock.clk_freq_in_10khz =
						fw_info->default_engine_clock;
		init_clk->mem_clock.clk_freq_in_10khz =
						fw_info->default_mem_clock;
		if (post_type == VBIOS_POST_REPOST) {
			gim_info("Do a re-post");
			/* Load uCode and do Asic Init */
			init_clk->sclk_clock.clk_flag = 0x40;
		}

		if (post_type == VBIOS_POST_FULL_POST) {
			gim_info("Do a full post");
			/* Load uCode and do Asic Init */
			init_clk->sclk_clock.clk_flag = 0x40;
		}

		if (post_type == VBIOS_POST_LOAD_UCODE) {
			gim_info("just load uCode");
			/* Load uCode and Skip asic init */
			init_clk->sclk_clock.clk_flag = 0x42;
		}

		if ((table_hdr->table_format_revision == 1)
		    && (table_hdr->table_content_revision >= 2))
			init_clk->mem_clock.clk_flag = 0;

		parser_ctx.para_ptr = (void *)&asic_init_ps;
		parser_ctx.size = sizeof(struct cail_parser_table_context);
		gim_info("asic_init before, engine clock = %x;"
			 " memory clock =%x\n",
			 fw_info->default_engine_clock,
			 fw_info->default_mem_clock);

		ret = atom_exec_bios_table(adapt, &parser_ctx);
		gim_info("asic_init after\n");
	} else {
		gim_err("get Firmware info failed\n");
		ret = -1;
	}

	if (ret == 0) {
		gim_info("atom_init_fan_cntl before\n");
		ret = atom_init_fan_cntl(adapt);
		gim_info("atom_init_fan_cntl after\n");
		if (ret == 1) {
			gim_err("BIOS table not found!\n");
			ret = 0;
		}
	}

	return ret;
}

int atom_read_rom_img_off(struct cail_adapter_config_info *adapter_info,
			void *dest, uint32_t rom_off, uint32_t length)
{
	uint32_t rom_size = adapter_info->rom_length;
	int ret = 0;

	if (rom_size != 0) {
		if (rom_off > rom_size)
			return -1;
		else if (length + rom_off > rom_size)
			length = rom_size - rom_off;
	}

	memcpy(dest,
		adapter_info->rom_base_addr + rom_off,
		length);

	return ret;
}

int atom_init_parser(struct adapter *adapt)
{
	int ret = 0;
	unsigned short rom_header_off;
	unsigned char rom_length;
	struct cail_adapter_config_info *adapter_info;

	adapter_info = (struct cail_adapter_config_info *)(adapt->rom_info);

	ret = atom_read_rom_img_off(adapter_info,
				      &rom_header_off,
				      OFFSET_TO_POINTER_TO_ATOM_ROM_HEADER,
				      sizeof(unsigned short));
	if (ret != 0) {
		gim_err("atom_read_rom_img_off: ROM HEADER fails\n");
		return ret;
	}
	adapter_info->rom_header_offset = (uint32_t)rom_header_off;

	ret = atom_read_rom_img_off(adapter_info,
				      &rom_length,
				      OFFSET_TO_ATOM_ROM_IMAGESIZE,
				      sizeof(unsigned char));
	if (ret != 0) {
		gim_err("atom_read_rom_img_off: ROM IMAGESIZE fails\n");
		return ret;
	}
	adapter_info->rom_length = rom_length * HALF_K;

	return ret;
}

int atom_chk_asic_status(struct adapter *adapt)
{
	uint32_t dw_reset, dw_rcl;

	dw_reset = pf_read_register(adapt, mmBIOS_SCRATCH_7);
	gim_info("ATOM_CheckAsicStatus - BIOS_SCRATCH_7 = 0x%08x\n",
		 dw_reset);
	dw_reset &= ATOM_S7_ASIC_INIT_COMPLETE_MASK;
	gim_info("Isolate ATOM_S7_ASIC_INIT_COMPLETE_MASK bit(s) = 0x%08x\n",
		 dw_reset);

	dw_rcl = pf_read_register(adapt, mmRLC_CNTL);
	gim_info("RLC_CNTL = 0x%08x\n", dw_rcl);
	dw_rcl &= RLC_CNTL__RLC_ENABLE_F32_MASK;
	gim_info("Isolate RLC_CNTL__RLC_ENABLE_F32_MASK = 0x%08x\n", dw_rcl);

	if (dw_reset) {
		gim_info("ATOM_ASIC_POSTED");
		return ATOM_ASIC_POSTED;
	}

	gim_info("ATOM_ASIC_NEED_POST\n");

	return ATOM_ASIC_NEED_POST;
}

unsigned int get_smu_version(struct adapter *adapt)
{
	unsigned int ret = 0;
	uint16_t tbl_index;
	struct atom_gpu_virt_info_v2_1 *current_info;

	tbl_index = get_index_into_master_table(data, gpu_virt);
	current_info = (struct atom_gpu_virt_info_v2_1 *)
		atom_get_tab_pointer(adapt, ATOM_DATATABLE, tbl_index, 0);
	pf_write_register(adapt, mmSMC_IND_INDEX_6, ixROM_INDEX);
	pf_write_register(adapt, mmSMC_IND_DATA_6,
			  current_info->smc_ucode_rom_start_addr + 0x14);
	pf_write_register(adapt, mmSMC_IND_INDEX_6, ixROM_DATA);
	ret = pf_read_register(adapt, mmSMC_IND_DATA_6);
	return ret;
}

/*
 * Read the RLCV version via the SMC
 */
unsigned int get_rlcv_version(struct adapter *adapt)
{
	unsigned int ret = 0;
	uint16_t tbl_index;
	struct atom_gpu_virt_info_v2_1 *current_info;

	tbl_index = get_index_into_master_table(data, gpu_virt);
	current_info = (struct atom_gpu_virt_info_v2_1 *)
		atom_get_tab_pointer(adapt, ATOM_DATATABLE, tbl_index, 0);
	pf_write_register(adapt, mmSMC_IND_INDEX_6, ixROM_INDEX);
	pf_write_register(adapt, mmSMC_IND_DATA_6,
			  current_info->rlcv_ucode_rom_start_addr);
	pf_write_register(adapt, mmSMC_IND_INDEX_6, ixROM_DATA);
	ret = pf_read_register(adapt, mmSMC_IND_DATA_6);
	return ret;
}

/*
 * Copy a single firmware from vbios image (in system memory) to
 * reserved VBios location. Assume that the caller has already set up
 * the aperture to point to the reserved area
 */
static void copy_firmware_from_rom_to_reserved(struct adapter *adapt,
		uint32_t addr, uint32_t size, void *firmware_buffer)
{
	if (size) {
		memcpy((void *)((unsigned char *)(adapt->pf.fb_va) + addr),
			firmware_buffer, size);
	}
}

static void atom_read_fw(struct adapter *adapt,
			unsigned int firmware_off,
			unsigned int length, void *pfirmware_buffer)
{
	unsigned int *vbios = NULL;
	unsigned int length_in_dword;
	unsigned int i;

	length_in_dword = length + sizeof(unsigned int) - 1;
	length_in_dword = length_in_dword/sizeof(unsigned int);

	vbios = (unsigned int *)pfirmware_buffer;
	write_reg32((void *)adapt->pf.mmr_base, mmSMC_IND_INDEX_6, ixROM_INDEX);
	write_reg32((void *)adapt->pf.mmr_base, mmSMC_IND_DATA_6, firmware_off);
	write_reg32((void *)adapt->pf.mmr_base, mmSMC_IND_INDEX_6, ixROM_DATA);

	for (i = 0; i < length_in_dword; i++)
		vbios[i] = read_reg32((void *)adapt->pf.mmr_base,
					mmSMC_IND_DATA_6);
}

/*
 * Copy all of the firmware as specified in the GPUVirtualization table
 * into the FB reserved area. Assume that the caller has already set up
 * the aperture to point to the reserved area
 */
static void load_default_firmware(struct adapter *adapt)
{
	uint16_t tbl_index;
	struct atom_gpu_virt_info_v2_1 *info;
	void *buffer = NULL;
	unsigned long len = 0;
	bool bl = false;

	tbl_index = get_index_into_master_table(data, gpu_virt);
	info = (struct atom_gpu_virt_info_v2_1 *)
		atom_get_tab_pointer(adapt, ATOM_DATATABLE, tbl_index, 0);

	bl = info->smc_ucode_length > info->smc_patch_table_length;
	len = bl ? info->smc_ucode_length : info->smc_patch_table_length;

	bl = len > info->rlcv_ucode_length;
	len = bl ? len : info->rlcv_ucode_length;

	bl = len > info->toc_ucode_length;
	len = bl ? len : info->toc_ucode_length;

	/* 4 bytes aglined */
	len   = len + sizeof(unsigned int) - 1;
	buffer = vmalloc(len);
	if (buffer == NULL) {
		gim_err("allocate buffer memory failed");
		return;
	}

	atom_read_fw(adapt, info->smc_ucode_rom_start_addr,
			info->smc_ucode_length, buffer);
	copy_firmware_from_rom_to_reserved(adapt,
			info->smc_ucode_rom_start_addr,
			info->smc_ucode_length, buffer);

	atom_read_fw(adapt, info->rlcv_ucode_rom_start_addr,
			info->rlcv_ucode_length, buffer);
	copy_firmware_from_rom_to_reserved(adapt,
			info->rlcv_ucode_rom_start_addr,
			info->rlcv_ucode_length, buffer);

	atom_read_fw(adapt, info->toc_ucode_start_addr,
			info->toc_ucode_length, buffer);
	copy_firmware_from_rom_to_reserved(adapt, info->toc_ucode_start_addr,
			info->toc_ucode_length, buffer);

	atom_read_fw(adapt, info->smc_patch_table_start_addr,
			info->smc_patch_table_length, buffer);
	copy_firmware_from_rom_to_reserved(adapt,
			info->smc_patch_table_start_addr,
			info->smc_patch_table_length, buffer);
	vfree(buffer);
}

/*
 * Check the versions of the firmware in the patch file against the
 * version of the firmware in the option rom If the patch file is
 * newer then set a bit for the firware that needs to be updated.
 */
static unsigned int firmware_requires_update(struct adapter *adapt)
{
	uint16_t	tbl_index;
	struct atom_gpu_virt_info_v2_1 *patch_info;
	struct atom_common_table_header       *patch_table_header;
	unsigned int	rom_ver;
	unsigned int	patch_ver;
	unsigned char  *ucode;
	unsigned int bits = 0;

	tbl_index =
		get_index_into_master_table(data, gpu_virt);
	patch_table_header =
		(struct atom_common_table_header *)sriov_vbios_patch_buf;
	patch_info = (struct atom_gpu_virt_info_v2_1 *)
		(sriov_vbios_patch_buf + patch_table_header->stru_size);
	if (patch_info->smc_ucode_rom_start_addr) {
		rom_ver = get_smu_version(adapt);
		ucode = sriov_vbios_patch_buf +
			patch_info->smc_ucode_rom_start_addr;
		patch_ver = *(uint32_t *)(ucode + 0x14);
		gim_info("SMU option ROM version 0x%x", rom_ver);
		gim_info("versus patch version 0x%x\n", patch_ver);
		if (patch_ver > rom_ver)
			bits |= UPDATE_FIRMWARE_SMU;
	}
	if (patch_info->rlcv_ucode_rom_start_addr) {
		rom_ver = get_rlcv_version(adapt);
		ucode = sriov_vbios_patch_buf +
			patch_info->rlcv_ucode_rom_start_addr;
		patch_ver = *(uint32_t *)(ucode);
		gim_info("RLCV option ROM version %d versus patch version %d\n",
			 rom_ver, patch_ver);
		if (patch_ver > rom_ver)
			bits |= UPDATE_FIRMWARE_RLCV;
	}
	if (patch_info->toc_ucode_start_addr) {
		gim_info("TOC found, update it\n");
		bits |= UPDATE_FIRMWARE_TOC;
	}
	return bits;
}

/*
 * During VBios post the VBios image is copied to the VBios reserved area.
 * This function will copy the new smu firmware into the VBios copy in the
 *  VBios reserved area.
 * After the copy is done the caller needs to do a "load_firmware" call.
 *
 * The sriov_vbios_patch_buf has the following format
 * - SMC_InitTable command table
 * - GPUVirtualizationInfoTable data table
 * - uCode1 (smu)
 * - uCode2 (rlcv)
 * - ...
 */
int patch_firmware(struct adapter *adapt)
{
	unsigned int	hdp_nonsurface_base;
	unsigned int	total_fb_size;
	unsigned int	firmware_map;
	unsigned int	reg_val;
	uint16_t	tbl_index;
	void		*cmd_table;
	struct atom_common_table_header       *patch_table_header;
	struct atom_gpu_virt_info_v2_1 *current_info;
	struct atom_gpu_virt_info_v2_1 *patch_info;

	if (((struct cail_adapter_config_info *)adapt->rom_info)->rom_base_addr
	    == NULL) {
		gim_warn("Rom Image is missing\n");
		return -1;
	}
	tbl_index = get_index_into_master_table(data, gpu_virt);
	current_info = (struct atom_gpu_virt_info_v2_1 *)
		atom_get_tab_pointer(adapt, ATOM_DATATABLE, tbl_index, 0);
	patch_table_header = (struct atom_common_table_header *)
				sriov_vbios_patch_buf;
	patch_info = (struct atom_gpu_virt_info_v2_1 *)
		(sriov_vbios_patch_buf + patch_table_header->stru_size);

	/*
	 * if any of the patch firmware version is bigger
	 * than option rom version, then the firmware and
	 * table will be patched
	 */
	firmware_map = firmware_requires_update(adapt);

	/* disable HDP write combine
	 * HDP write combine can't work well with host side write combine.
	 */
	reg_val = pf_read_register(adapt, mmHDP_HOST_PATH_CNTL);
	/* disable WRITE_COMBINE_EN and WRITE_COMBINE_64B_EN */
	reg_val &= ~(3 << 21);
	pf_write_register(adapt, mmHDP_HOST_PATH_CNTL, reg_val);

	if (firmware_map) {
		gim_info("Update smc_init table\n");

		/* Replace the smc_init table in system memory
		 * with the first data structure in the patch file.
		 */
		tbl_index = get_index_into_master_table(command, smc_init);
		cmd_table = atom_get_tab_pointer(adapt,
				ATOM_COMMANDTABLE, tbl_index, 0);
		memcpy(cmd_table, sriov_vbios_patch_buf,
			patch_table_header->stru_size);

		/* Find the location where VBios has copied the VBios image.
		 * The VBios image should be at the top of PF visible memory
		 */
		total_fb_size = pf_read_register(adapt, mmCONFIG_MEMSIZE);

		/* Adjust the HDP window to point to the beginning of
		 * the VBios reserved location.
		 */
		hdp_nonsurface_base = pf_read_register(adapt,
						       mmHDP_NONSURFACE_BASE);
		pf_write_register(adapt, mmHDP_NONSURFACE_BASE,
				  hdp_nonsurface_base + (total_fb_size << 12));

		/*
		 * The PF FB VA offset 0 points to the VBios image offset 0
		 * at the top of PF FB current_info points to the GPU info
		 * structure in PF FB
		 */

		load_default_firmware(adapt);

		if (firmware_map & UPDATE_FIRMWARE_SMU) {
			gim_info("Update smu firmware\n");
			memcpy((void *)((unsigned char *)(adapt->pf.fb_va)
				+ current_info->smc_ucode_rom_start_addr),
				sriov_vbios_patch_buf
				+ patch_info->smc_ucode_rom_start_addr,
				patch_info->smc_ucode_length);
		}

		if (firmware_map & UPDATE_FIRMWARE_RLCV) {
			gim_info("Update RLCV firmware\n");
			memcpy((void *)((unsigned char *)(adapt->pf.fb_va)
				+ current_info->rlcv_ucode_rom_start_addr),
			       sriov_vbios_patch_buf
			       + patch_info->rlcv_ucode_rom_start_addr,
			       patch_info->rlcv_ucode_length);
		}

		if (firmware_map & UPDATE_FIRMWARE_TOC) {
			gim_info("Update TOC\n");
			memcpy((void *)((unsigned char *)(adapt->pf.fb_va)
				       + current_info->toc_ucode_start_addr),
			       sriov_vbios_patch_buf
			       + patch_info->toc_ucode_start_addr,
			       patch_info->toc_ucode_length);
		}

		/* Restore the HDP window */
		pf_write_register(adapt, mmHDP_NONSURFACE_BASE,
				  hdp_nonsurface_base);
	} else {
		gim_info("Firmware is up to date.  Don't need to patch it\n");
	}

	return 0;
}

int atom_dpm_state_cntl(struct adapter *adapt,
			struct pwr_mgt_param *param)
{
	struct cail_parser_table_context  context;
	struct atom_common_table_header *table_hdr;
	int ret = 0;
	uint16_t tbl_index;

	tbl_index = get_index_into_master_table(command,
						enable_asic_static_pwr_mgt);

	table_hdr = atom_get_tab_pointer(adapt,
					ATOM_COMMANDTABLE, tbl_index, 0);

	if (table_hdr == NULL) {
		gim_info("Table header not found");
		return -1;
	}

	if ((table_hdr->table_format_revision >= 2) &&
			(table_hdr->table_content_revision < 2)) {
		context.para_ptr = (void *)param;
		context.table_index = tbl_index,
			context.size = sizeof(struct cail_parser_table_context);

		ret = atom_exec_bios_table(adapt, &context);
	} else {
		gim_info("Table revision not match");
		return -1;
	}

	return ret;
}

void enable_thermal_control(struct adapter *adapt)
{
	struct pwr_mgt_param  pwr_paras = {
		PPSMC_MSG_THERMAL_CNTL_ENABLE,
		0
	};

	if (atom_dpm_state_cntl(adapt, &pwr_paras))
		gim_info("Thermal Control:Execute table failed");
	else
		gim_info("Thermal Control Enable");
}

