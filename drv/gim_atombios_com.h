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

#ifndef _GPU_IOV_MODULE__ATOMBIOS_H
#define _GPU_IOV_MODULE__ATOMBIOS_H

#pragma pack(push, 1)     /* BIOS data must use byte aligment */

/* Define offset to location of ROM header. */
#define OFFSET_TO_ROM_IMAGESIZE                     0x00000002L
#define OFFSET_TO_ATOM_ROM_IMAGESIZE                0X00000002L
#define OFFSET_TO_POINTER_TO_ATOM_ROM_HEADER        0x00000048L

/* ENABLE_ASIC_STATIC_PWR_MGT_PARAMETERS_V2_1 */
struct pwr_mgt_param {
	unsigned long smc_msg_id;
	unsigned long smc_msg_arg;
};

enum atombios_image_offset {
	OFFSET_TO_ATOM_ROM_HEADER_POINTER          = 0x00000048,
	OFFSET_TO_ATOM_ROM_IMAGE_SIZE              = 0x00000002,
	OFFSET_TO_ATOMBIOS_ASIC_BUS_MEM_TYPE       = 0x94,
	/*including the terminator 0x0!*/
	MAXSIZE_OF_ATOMBIOS_ASIC_BUS_MEM_TYPE      = 20,
	OFFSET_TO_GET_ATOMBIOS_NUMBER_OF_STRINGS   = 0x2f,
	OFFSET_TO_GET_ATOMBIOS_STRING_START        = 0x6e,
};

/*
 *Common header for all tables (Data table, Command table).
 *Every table pointed  struct atom_master_data_table has this common header.
 *And the pointer actually points to this header.
 */
struct atom_common_table_header {
	uint16_t stru_size;
	/* Change it when the Parser is not backward compatible */
	unsigned char table_format_revision;
	/* Change it only when the table needs to change but the firmware */
	unsigned char table_content_revision;
	/* Image can't be updated, while Driver needs to carry the new table!*/
};

/*Structure stores the ROM header. */
struct atom_rom_header {
	struct atom_common_table_header header;
	/* Signature to distinguish between Atombios and non-atombios, */
	unsigned char firmware_signature[4];
	/* atombios should init it as "ATOM", don't change the position */
	uint16_t bios_runtime_seg_add;
	uint16_t protect_mode_info_offset;
	uint16_t config_filename_offset;
	uint16_t crc_block_offset;
	uint16_t bios_bootup_message_offset;
	uint16_t int10_offset;
	uint16_t pci_bus_dev_init_code;
	uint16_t io_base_add;
	uint16_t sub_sys_vendor_id;
	uint16_t subsystem_id;
	uint16_t pci_info_offset;
	/*Offest for SW to get all command table offsets,don't change the pos*/
	uint16_t master_command_table_offset;
	/*Offest for SW to get all data table offsets,don't change the pos*/
	uint16_t master_data_table_offset;
	unsigned char extend_func_code;
	unsigned char reserved;
};

/*
 *Command Table Portion
 *Structures used in Command.mtb
 */
struct atom_master_list_of_command_tables {
	/* function table,used by various SW components,latest version 1.1 */
	uint16_t asic_init;
	/* atomic table,Used by Bios when enabling HW ICON */
	uint16_t get_display_surf_size;
	/* atomic table,used by various SW components,called from asic_init */
	uint16_t asic_reg_init;
	/* atomic table,used only by Bios */
	uint16_t vram_block_vender_detection;
	/* Only used by Bios */
	uint16_t digx_encoder_control;
	/* atomic table,used by SW components,called from asic_init */
	uint16_t mem_controller_init;
	/* function table,used by SW components,latest version 2.1 */
	uint16_t enable_crtc_mem_req;
	/* atomic table,used by SW compts,call from set_mem_clock if needed */
	uint16_t mem_param_adjust;
	/* function table,directly used by SW components,latest version 1.2 */
	uint16_t dvo_encoder_control;
	/* atomic table,only used by Bios */
	uint16_t gpio_pin_control;
	/* function table,directly used by SW components,latest version 1.1 */
	uint16_t set_engine_clock;
	/* function table,directly used by SW components,latest version 1.1 */
	uint16_t set_mem_clock;
	/* function table,directly used by SW components,latest version 1.2 */
	uint16_t set_pixel_clock;
	/* atomic table,indirectly used by SW compts,called from asic_init */
	uint16_t enable_disp_power_gating;
	/* atomic table,indirectly used by SW comps,called from set_mem_clock*/
	uint16_t reset_mem_dll;
	/* atomic table,indirectly used by SW compts,call from set_mem_clock */
	uint16_t reset_mem_Dev;
	/* atomic table,used only by Bios */
	uint16_t mem_pll_init;
	/* atomic table,used by various SW componentes. */
	uint16_t adjust_display_pll;
	/* atomic table,indirectly used by SW compts,call from set_mem_clock */
	uint16_t adjust_mem_controller;
	/* atomic table,only used by Bios */
	uint16_t enable_asic_static_pwr_mgt;
	/* atomic table,only used by Bios */
	uint16_t set_uniphy_instance;
	/* atomic table,directly used by SW compts,latest version 1.2 */
	uint16_t dac_load_detection;
	/* atomic table,directly used by SW components,latest version 1.3 */
	uint16_t lvtma_encoder_control;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t hw_misc_operation;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t dac1_encoder_control;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t dac2_encoder_control;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t dvo_output_control;
	/* atomic table,Obsolete from Ry6xx, use DAC2 Output instead */
	uint16_t cv1_output_control;
	/* Only used by Bios */
	uint16_t get_cond_golden_setting;
	/* function table,directly used by  SW components,latest version 1.1 */
	uint16_t smc_init;
	/* only used by BIOS */
	uint16_t patch_mc_setting;
	/* only used by BIOS */
	uint16_t mc_seq_control;
	/* atomic table,Obsolete from Ry6xx,only used by BIOS for GFX harvest*/
	uint16_t gfx_harvesting;
	/* atomic table,used only by Bios */
	uint16_t enable_scaler;
	/* atomic table,directly used by SW compts,latest version 1.1 */
	uint16_t blank_crtc;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t enable_crtc;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t get_pixel_clock;
	/* function table,directly used by SW components,latest version 1.1 */
	uint16_t enable_vga_render;
	/* atomic table,only used by Bios */
	uint16_t get_sclk_over_mclk_ratio;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t set_crtc_timing;
	/* atomic table,used by various SW components,latest version 1.1 */
	uint16_t set_crtc_over_scan;
	/* atomic table, used only by Bios */
	uint16_t set_crtc_replication;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t select_crtc_source;
	/* atomic table,used only by Bios */
	uint16_t enable_graph_surf;
	/* atomic table,used only by Bios */
	uint16_t update_crtc_double_buffer_reg;
	/* atomic table,only used by Bios */
	uint16_t lut_auto_fill;
	/* atomic table,only used by Bios */
	uint16_t enable_hw_icon_cursor;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t get_mem_clock;
	/* atomic table,directly used by SW compts,latest version 1.1 */
	uint16_t get_engine_clock;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t set_crtc_dtdt_timing;
	/* atomic table,directly used by SW components,latest version 2.1 */
	uint16_t external_encoder_control;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t lvtma_output_control;
	/* atomic table,used only by Bios */
	uint16_t vram_block_detection_by_strap;
	/* atomic table,only used by Bios */
	uint16_t gfx_dma_control;
	/* function table,only used by Bios */
	uint16_t process_i2c_channel_trans;
	/* function table,indirectly used by various SW components */
	uint16_t write_byte_to_hw_assisted_i2c;
	/* atomic table,indirectly used by various SW components */
	uint16_t read_hw_assisted_i2c_status;
	/* function table,indirectly used by SW compts,call from asic_init*/
	uint16_t speed_fan_control;
	/* atomic table,directly used by SW compts,latest version 1.1 */
	uint16_t power_connector_detection;
	/* atomic table,indirectly used by SW compts,call from set_mem_clock*/
	uint16_t mc_sync;
	/* atomic table,used by SW compts,call from SetMemory/EngineClock */
	uint16_t compute_mem_engine_pll;
	/* atomic table,used by SW compts,call from SetMem/set_engine_clock*/
	uint16_t mem_refresh_conversion;
	/* atomic table,used only by Bios */
	uint16_t vram_get_current_info_block;
	/* atomic table,indirect used by SW compts,called from set_mem_clock */
	uint16_t dynamic_mem_settings;
	/* atomic table,used only by Bios */
	uint16_t mem_training;
	/* atomic table,directly used by SW compts,latest version 1.2 */
	uint16_t enable_spread_spectrum_on_ppll;
	/* atomic table,directly used by SW compts,latest version 1.1 */
	uint16_t tmdsa_output_control;
	/* func table,direct/indirect used by SW compts,latest version 1.1*/
	uint16_t set_voltage;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t dac1_output_control;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t read_efuse_value;
	/*use by Bios,obsolete soon.Switch to use "ReadEDIDFromHWAssistedI2C"*/
	uint16_t compute_mem_clock_param;
	/* atomic table,indirectly used by SW compts,called from asic_init */
	uint16_t clock_source;
	/* atomic table,indirectly used by SW compts,call from set_mem_clock*/
	uint16_t mem_dev_init;
	/* atomic table,used by SW compts,call from EnableVGARender*/
	uint16_t get_disp_obj_info;
	/* atomic table,directly used by SW compts,latest version 1.1 */
	uint16_t dig1_encoder_control;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t dig2_encoder_control;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t dig1_transmitter_control;
	/* atomic table,directly used by SW components,latest version 1.1 */
	uint16_t dig2_transmitter_control;
	/* function table,only used by Bios */
	uint16_t process_aus_channel_trans;
	/* function table,only used by Bios */
	uint16_t dp_encoder_service;
	/* function table,only used by Bios since SI */
	uint16_t get_voltage_info;
};

struct atom_master_command_table {
	struct atom_common_table_header header;
	struct atom_master_list_of_command_tables list_of_command_tables;
};

/* Structures used in every command table */
struct atom_table_attr {
	/* [7:0]=Size of workspace in Bytes (in multiple of a dword), */
	uint16_t ws_size_in_bytes:8;
	/* [14:8]=Size of parameter space in Bytes (multiple of a dword)*/
	uint16_t ps_size_in_bytes:7;
	/* [15]=Table updated by utility flag */
	uint16_t update_by_utility:1;
};

/*
 *Common header for all command tables.
 *Every table pointed by struct atom_master_command_table
 *has this common header.
 *And the pointer actually points to this header.
 */
struct atom_common_rom_command_table_header {
	struct atom_common_table_header common_header;
	struct atom_table_attr table_attr;
};

/* Structures used by adjust_mem_controllerTable*/
struct compute_mem_engine_pll_parm {
	/* When return, it's re-cal clk based on Fb_div Post_Div and ref_div*/
	uint32_t clock;
	unsigned char action;	/* 0:reserved 1:Memory 2:Engine */
	unsigned char reserved;	/* may expand to return larger Fbdiv later */
	unsigned char fb_div;	/* return value */
	unsigned char post_div;	/* return value */
};


/*
 *Applicable to DRAM self refresh exit only. when set,
 *it means it will go to program DRAM self refresh exit path
 */
#define b3DRAM_SELF_REFRESH_EXIT	0x20

/* Structures used by set_engine_clockTable */
struct set_engine_clock_parm {
	uint32_t target_engine_clock;    /* In 10Khz unit */
};

struct set_engine_clock_ps_alloc {
	uint32_t target_engine_clock;    /* In 10Khz unit */
	struct compute_mem_engine_pll_parm sreserved;
};

/* Structures used by asic_init.ctb */
struct asic_init_parm {
	uint32_t default_engine_clock;    /* In 10Khz unit */
	uint32_t default_mem_clock;    /* In 10Khz unit */
};

struct asic_init_ps_alloc {
	struct asic_init_parm asic_init_clocks;
	/* Caller doesn't need to init this structure */
	struct set_engine_clock_ps_alloc sreserved;
};

struct asic_init_clock_parm {
	uint32_t clk_freq_in_10khz:24;
	uint32_t clk_flag:8;
};

struct asic_init_parm_v1_2 {
	struct asic_init_clock_parm sclk_clock;    /* In 10Khz unit */
	struct asic_init_clock_parm mem_clock;    /* In 10Khz unit */
};

struct asic_init_ps_alloc_v1_2 {
	struct asic_init_parm_v1_2 asic_init_clocks;
	uint32_t reserved[8];
};

/* Following Structures and constant may be obsolete */
struct write_byte_hw_i2c_data_parm {
	uint16_t prescale;    /* Ratio between Engine clock and I2C clock */
	uint16_t byte_offset;    /* Write to which byte */
	/* Upper portion of usByteOffset is Format of data */
	/* 1bytePS+offsetPS */
	/* 2bytesPS+offsetPS */
	/* blockID+offsetPS */
	/* blockID+offsetID */
	/* blockID+counterID+offsetID */
	unsigned char data;     /* PS data1 */
	/* Status byte 1=success, 2=failure, Also is used as PS data2 */
	unsigned char status;
	unsigned char slave_addr; /* Write to which slave */
	unsigned char line_number;    /* Write from which HW assisted line */
};


/* Structure used in Data.mtb */
struct atom_master_list_of_data_tables {
	/* Offest for utility to get parser info,Don't change this pos! */
	uint16_t utility_pipe_line;
	/*
	 * Only used by MM Lib,latest version 1.1, not configurable from BIOS,
	 * ineed to include the table to build Bios
	 */
	uint16_t multimedia_cap_info;
	/*
	 * Only used by MM Lib,latest version 2.1, not configurable from BIOS,
	 * need to include the table to build Bios
	 */
	uint16_t multimedia_config_info;
	/* Only used by Bios */
	uint16_t standard_vesa_timing;
	/* Shared by various SW components,latest version 1.4 */
	uint16_t firmware_info;
	/* Only used by BIOS */
	uint16_t palette_data;
	/* Shared by SW components,latest version 1.3, called LVDS_Info */
	uint16_t lcd_info;
	/* Internal used by VBIOS only version 3.1 */
	uint16_t dig_transmitter_info;
	/* Shared by various SW components,latest version 1.1 */
	uint16_t analog_tv_info;
	/* Will be obsolete from R600 */
	uint16_t support_dev_info;
	/* Shared by SW components,latest version 1.2 will be used from R600 */
	uint16_t gpio_i2c_info;
	/* Shared by SW components,latest version 1.3 will be used from R600 */
	uint16_t vram_usage_by_firmware;
	/* Shared by various SW components,latest version 1.1 */
	uint16_t gpio_pin_lut;
	/* Only used by Bios */
	uint16_t vesa_to_internal_mode_lut;
	/* Shared by SW components,latest version 2.1 will be used from R600 */
	uint16_t component_video_info;
	/* Shared by SW components,latest version 2.1,new design from R600 */
	uint16_t power_play_info;
	/* Will be obsolete from R600 */
	uint16_t gpu_virt;
	/* Only used by Bios */
	uint16_t save_restore_info;
	/*
	 * Shared by SW components,latest version 1.2, used to call SS_Info,
	 * change to new name because of int ASIC SS info
	 */
	uint16_t ppll_ss_info;
	/* Defined and used by external SW, should be obsolete soon */
	uint16_t oem_info;
	/* Will be obsolete from R600 */
	uint16_t xtmds_info;
	/*
	 * Shared by various SW components,latest version 1.1,
	 * only enabled when ext SS chip is used
	 */
	uint16_t mclkss_info;
	/* Shared by various SW components,latest version 1.1 */
	uint16_t object_header;
	/* Only used by Bios,this table position can't change at all!! */
	uint16_t indirect_io_access;
	/* Only used by command table */
	uint16_t mc_init_param;
	/* Will be obsolete from R600 */
	uint16_t asic_vddc_info;
	/* New tabel name from R600, used to be called "ASIC_MVDDC_Info" */
	uint16_t asic_internalss_info;
	/* Only used by command table */
	uint16_t tv_video_mode;
	/* Only used by command table, latest version 1.3 */
	uint16_t vram_info;
	/*
	 * Used for VBIOS and Diag utility for memory training purpose
	 * since R600. the new table rev start from 2.1
	 */
	uint16_t mem_training_info;
	/* Shared by various SW components */
	uint16_t integrated_sys_info;
	/*
	 * New table name from R600,
	 * used to be called "ASIC_VDDCI_Info" for pre-R600
	 */
	uint16_t asic_profiling_info;
	/* Shared by various SW components, latest version 1.1 */
	uint16_t voltage_obj_info;
	/* Shared by various SW components, latest versoin 1.1 */
	uint16_t power_source_info;
	uint16_t service_info;
};

struct atom_master_data_table {
	struct atom_common_table_header header;
	struct atom_master_list_of_data_tables tables_list;
};

/* Structures used in firmware_infoTable */
struct atom_firmware_cap {
	uint16_t firmware_posted:1;
	uint16_t dual_crtc_support:1;
	uint16_t extended_desktop_support:1;
	uint16_t mem_clockss_support:1;
	uint16_t engine_clockss_support:1;
	uint16_t gpu_controls_bl:1;
	uint16_t wmi_support:1;
	uint16_t pp_mode_assigned:1;
	uint16_t hyper_mem_support:1;
	uint16_t hyper_mem_size:4;
	uint16_t post_without_mode_set:1;
	uint16_t scl2_redefined:1;
	uint16_t reserved:1;
};

union atom_firmware_cap_accs {
	struct atom_firmware_cap sbf_access;
	uint16_t sus_access;
};

struct atom_firmware_info {
	struct atom_common_table_header header;
	uint32_t firmware_revision;
	uint32_t default_engine_clock;    /* In 10Khz unit */
	uint32_t default_mem_clock;    /* In 10Khz unit */
	uint32_t driver_target_engine_clock;    /* In 10Khz unit */
	uint32_t driver_target_mem_clock;    /* In 10Khz unit */
	uint32_t max_engine_clock_pll_output;    /* In 10Khz unit */
	uint32_t max_mem_clock_pll_output;    /* In 10Khz unit */
	uint32_t max_pixel_clock_pll_output;    /* In 10Khz unit */
	uint32_t asic_max_engine_clock;    /* In 10Khz unit */
	uint32_t asic_max_mem_clock;    /* In 10Khz unit */
	unsigned char asic_max_temp;
	unsigned char padding[3];    /* Don't use them */
	uint32_t reserved_for_bios[3];    /* Don't use them */
	uint16_t min_engine_clock_pll_input;    /* In 10Khz unit */
	uint16_t max_engine_clock_pll_input;    /* In 10Khz unit */
	uint16_t min_engine_clock_pll_output;    /* In 10Khz unit */
	uint16_t min_mem_clock_pll_input;    /* In 10Khz unit */
	uint16_t max_mem_clock_pll_input;    /* In 10Khz unit */
	uint16_t min_mem_clock_pll_output;    /* In 10Khz unit */
	uint16_t max_pixel_clock;    /* In 10Khz unit, Max.  Pclk */
	uint16_t min_pixel_clock_pll_input;    /* In 10Khz unit */
	uint16_t max_pixel_clock_pll_input;    /* In 10Khz unit */
	/* In 10Khz unit, the definitions above can't change!!! */
	uint16_t min_pixel_clock_pll_output;
	union    atom_firmware_cap_accs firmware_cap;
	uint16_t reference_clock;  /* In 10Khz unit */
	/* RTS PM4 starting location in ROM in 1Kb unit */
	uint16_t pm_rts_location;
	unsigned char pm_rts_stream_size;    /* RTS PM4 packets in Kb unit */
	unsigned char design_id;    /* Indicate what is the board design */
	unsigned char mem_module_id;    /* Indicate what is the board design */
};

#define ATOM_MAX_FIRMWARE_VRAM_USAGE_INFO         1

struct atom_firmware_vram_reserve_info {
	uint32_t start_addr_used_by_firmware;
	uint16_t firmware_use_kb;
	uint16_t reserved;
};

struct atom_vram_usage_by_firmware {
	struct atom_common_table_header header;

	struct atom_firmware_vram_reserve_info
		firmware_vram_reserve_info[ATOM_MAX_FIRMWARE_VRAM_USAGE_INFO];
};

#define get_index_into_master_table(master, name) \
	(((char *)(&((struct atom_master_list_of_##master##_tables *)0)->name) \
	-(char *)0)/sizeof(uint16_t))


/*Structure used in gpu_virt_table*/
struct atom_gpu_virt_info_v2_1 {
	struct atom_common_table_header   header;
	uint32_t mc_ucode_rom_start_addr;
	uint32_t mc_ucode_length;
	uint32_t smc_ucode_rom_start_addr;
	uint32_t smc_ucode_length;
	uint32_t rlcv_ucode_rom_start_addr;
	uint32_t rlcv_ucode_length;
	uint32_t toc_ucode_start_addr;
	uint32_t toc_ucode_length;
	uint32_t smc_patch_table_start_addr;
	uint32_t smc_patch_table_length;
	uint32_t sys_flag;
};

#define ATOM_S7_ASIC_INIT_COMPLETE_MASK     0x00000200

#pragma pack(pop)

#endif /* _ATOMBIOS_H */
