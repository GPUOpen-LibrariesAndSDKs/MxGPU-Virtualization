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

#ifndef _GPU_IOV_MODULE__GIM_ATOM_H
#define _GPU_IOV_MODULE__GIM_ATOM_H

#include <linux/types.h>

#pragma pack(push, 1)

#define PARSER_MAJOR_REVISION	6
#define PARSER_MINOR_REVISION	0

#define INDIRECTIO_ID		1
#define INDIRECTIO_END_OF_ID	9
#define INDIRECT_IO_READ	0
#define INDIRECT_IO_WRITE	0x80
#define INDIRECT_IO_MM		0
#define INDIRECT_IO_PLL		1
#define INDIRECT_IO_MC		2

#define PARSER_STRINGS		0
#define PARSER_DEC              1
#define PARSER_HEX              2

#define DB_CURRENT_COMMAND_TABLE	0xFF

#define TABLE_FORMAT_BIOS	0x0
#define TABLE_FORMAT_EASF	0x00000001
#define TABLE_ENABLE_DEBUGGER   0x00000002

#define EASF_TABLE_INDEX_MASK	0xfc
#define EASF_TABLE_ATTR_MASK    0x03

#define CD_ERROR(a)	(((int32_t) (a)) > CD_COMPLETED)
#define CD_ERROR_OR_COMPLETED(a)	(((int32_t) (a)) > CD_SUCCESS)

#define WS_QUOTIENT_C		64
#define WS_REMINDER_C		(WS_QUOTIENT_C+1)
#define WS_DATAPTR_C            (WS_REMINDER_C+1)
#define WS_SHIFT_C              (WS_DATAPTR_C+1)
#define WS_OR_MASK_C            (WS_SHIFT_C+1)
#define WS_AND_MASK_C           (WS_OR_MASK_C+1)
#define WS_FB_WINDOW_C          (WS_AND_MASK_C+1)
#define WS_ATTRIBUTES_C         (WS_FB_WINDOW_C+1)
#define WS_REGPTR_C             (WS_ATTRIBUTES_C+1)

#define SOURCE_ONLY_CMD_TYPE	0/* 0xFE */
#define SOURCE_DESTINATION_CMD_TYPE	1/* 0xFD */
#define DESTINATION_ONLY_CMD_TYPE	2/* 0xFC */

#define ACCESS_TYPE_BYTE	0/* 0xF9 */
#define ACCESS_TYPE_WORD        1/* 0xF8 */
#define ACCESS_TYPE_DWORD       2/* 0xF7 */
#define SWITCH_TYPE_ACCESS      3/* 0xF6 */

#define CD_CONTINUE             0/* 0xFB */
#define CD_STOP                 1/* 0xFA */

#define RELATIVE_TO_TABLE(x)    (x + (unsigned char *) \
				(data->working_table_data->table_head))

#define IS_END_OF_TABLE(cmd) ((cmd) == EOT_OPCODE)

enum opcode {
	Reserved_00= 0,
	/* MOVE_ group */
	MOVE_REG_OPCODE,            /* 1    = 0x01 */
	FirstValidCommand = MOVE_REG_OPCODE,
	MOVE_PS_OPCODE,                /* 2    = 0x02 */
	MOVE_WS_OPCODE,                /* 3    = 0x03 */
	MOVE_FB_OPCODE,                /* 4    = 0x04 */
	MOVE_PLL_OPCODE,            /* 5    = 0x05 */
	MOVE_MC_OPCODE,                /* 6    = 0x06 */
	/* Logic group */
	AND_REG_OPCODE,                /* 7    = 0x07 */
	AND_PS_OPCODE,                /* 8    = 0x08 */
	AND_WS_OPCODE,                /* 9    = 0x09 */
	AND_FB_OPCODE,                /* 10    = 0x0A */
	AND_PLL_OPCODE,                /* 11    = 0x0B */
	AND_MC_OPCODE,                /* 12    = 0x0C */
	OR_REG_OPCODE,                /* 13    = 0x0D */
	OR_PS_OPCODE,                /* 14    = 0x0E */
	OR_WS_OPCODE,                /* 15    = 0x0F */
	OR_FB_OPCODE,                /* 16    = 0x10 */
	OR_PLL_OPCODE,                /* 17    = 0x11 */
	OR_MC_OPCODE,                /* 18    = 0x12 */
	SHIFT_LEFT_REG_OPCODE,        /* 19    = 0x13 */
	SHIFT_LEFT_PS_OPCODE,        /* 20    = 0x14 */
	SHIFT_LEFT_WS_OPCODE,        /* 21    = 0x15 */
	SHIFT_LEFT_FB_OPCODE,        /* 22    = 0x16 */
	SHIFT_LEFT_PLL_OPCODE,        /* 23    = 0x17 */
	SHIFT_LEFT_MC_OPCODE,        /* 24    = 0x18 */
	SHIFT_RIGHT_REG_OPCODE,        /* 25    = 0x19 */
	SHIFT_RIGHT_PS_OPCODE,        /* 26    = 0x1A */
	SHIFT_RIGHT_WS_OPCODE,        /* 27    = 0x1B */
	SHIFT_RIGHT_FB_OPCODE,        /* 28    = 0x1C */
	SHIFT_RIGHT_PLL_OPCODE,        /* 29    = 0x1D */
	SHIFT_RIGHT_MC_OPCODE,        /* 30    = 0x1E */
	/* Arithmetic group */
	MUL_REG_OPCODE,                /* 31    = 0x1F */
	MUL_PS_OPCODE,                /* 32    = 0x20 */
	MUL_WS_OPCODE,                /* 33    = 0x21 */
	MUL_FB_OPCODE,                /* 34    = 0x22 */
	MUL_PLL_OPCODE,                /* 35    = 0x23 */
	MUL_MC_OPCODE,                /* 36    = 0x24 */
	DIV_REG_OPCODE,                /* 37    = 0x25 */
	DIV_PS_OPCODE,                /* 38    = 0x26 */
	DIV_WS_OPCODE,                /* 39    = 0x27 */
	DIV_FB_OPCODE,                /* 40    = 0x28 */
	DIV_PLL_OPCODE,                /* 41    = 0x29 */
	DIV_MC_OPCODE,                /* 42    = 0x2A */
	ADD_REG_OPCODE,                /* 43    = 0x2B */
	ADD_PS_OPCODE,                /* 44    = 0x2C */
	ADD_WS_OPCODE,                /* 45    = 0x2D */
	ADD_FB_OPCODE,                /* 46    = 0x2E */
	ADD_PLL_OPCODE,                /* 47    = 0x2F */
	ADD_MC_OPCODE,                /* 48    = 0x30 */
	SUB_REG_OPCODE,                /* 49    = 0x31 */
	SUB_PS_OPCODE,                /* 50    = 0x32 */
	SUB_WS_OPCODE,                /* 51    = 0x33 */
	SUB_FB_OPCODE,                /* 52    = 0x34 */
	SUB_PLL_OPCODE,                /* 53    = 0x35 */
	SUB_MC_OPCODE,                /* 54    = 0x36 */
	/* Control grouop */
	SET_ATI_PORT_OPCODE,        /* 55    = 0x37 */
	SET_PCI_PORT_OPCODE,        /* 56    = 0x38 */
	SET_SYS_IO_PORT_OPCODE,        /* 57    = 0x39 */
	SET_REG_BLOCK_OPCODE,        /* 58    = 0x3A */
	SET_FB_BASE_OPCODE,            /* 59    = 0x3B */
	COMPARE_REG_OPCODE,            /* 60    = 0x3C */
	COMPARE_PS_OPCODE,            /* 61    = 0x3D */
	COMPARE_WS_OPCODE,            /* 62    = 0x3E */
	COMPARE_FB_OPCODE,            /* 63    = 0x3F */
	COMPARE_PLL_OPCODE,            /* 64    = 0x40 */
	COMPARE_MC_OPCODE,            /* 65    = 0x41 */
	SWITCH_OPCODE,                /* 66    = 0x42 */
	JUMP__OPCODE,                /* 67    = 0x43 */
	JUMP_EQUAL_OPCODE,            /* 68    = 0x44 */
	JUMP_BELOW_OPCODE,            /* 69    = 0x45 */
	JUMP_ABOVE_OPCODE,            /* 70    = 0x46 */
	JUMP_BELOW_OR_EQUAL_OPCODE,    /* 71    = 0x47 */
	JUMP_ABOVE_OR_EQUAL_OPCODE,    /* 72    = 0x48 */
	JUMP_NOT_EQUAL_OPCODE,        /* 73    = 0x49 */
	TEST_REG_OPCODE,            /* 74    = 0x4A */
	TEST_PS_OPCODE,                /* 75    = 0x4B */
	TEST_WS_OPCODE,                /* 76    = 0x4C */
	TEST_FB_OPCODE,                /* 77    = 0x4D */
	TEST_PLL_OPCODE,            /* 78    = 0x4E */
	TEST_MC_OPCODE,                /* 79    = 0x4F */
	DELAY_MILLISEC_OPCODE,        /* 80    = 0x50 */
	DELAY_MICROSEC_OPCODE,        /* 81    = 0x51 */
	CALL_TABLE_OPCODE,            /* 82    = 0x52 */
	REPEAT_OPCODE,                /* 83    = 0x53 */
	/* Miscellaneous    group */
	CLEAR_REG_OPCODE,            /* 84    = 0x54 */
	CLEAR_PS_OPCODE,            /* 85    = 0x55 */
	CLEAR_WS_OPCODE,            /* 86    = 0x56 */
	CLEAR_FB_OPCODE,            /* 87    = 0x57 */
	CLEAR_PLL_OPCODE,            /* 88    = 0x58 */
	CLEAR_MC_OPCODE,            /* 89    = 0x59 */
	NOP_OPCODE,                    /* 90    = 0x5A */
	EOT_OPCODE,                    /* 91    = 0x5B */
	MASK_REG_OPCODE,            /* 92    = 0x5C */
	MASK_PS_OPCODE,                /* 93    = 0x5D */
	MASK_WS_OPCODE,                /* 94    = 0x5E */
	MASK_FB_OPCODE,                /* 95    = 0x5F */
	MASK_PLL_OPCODE,            /* 96    = 0x60 */
	MASK_MC_OPCODE,                /* 97    = 0x61 */
	/* BIOS dedicated group */
	POST_CARD_OPCODE,            /* 98    = 0x62 */
	BEEP_OPCODE,                /* 99    = 0x63 */
	SAVE_REG_OPCODE,            /* 100 = 0x64 */
	RESTORE_REG_OPCODE,            /* 101    = 0x65 */
	SET_DATA_BLOCK_OPCODE,            /* 102     = 0x66 */

	XOR_REG_OPCODE,                /* 103    = 0x67 */
	XOR_PS_OPCODE,                /* 104    = 0x68 */
	XOR_WS_OPCODE,                /* 105    = 0x69 */
	XOR_FB_OPCODE,                /* 106    = 0x6a */
	XOR_PLL_OPCODE,                /* 107    = 0x6b */
	XOR_MC_OPCODE,                /* 108    = 0x6c */

	SHL_REG_OPCODE,                /* 109    = 0x6d */
	SHL_PS_OPCODE,                /* 110    = 0x6e */
	SHL_WS_OPCODE,                /* 111    = 0x6f */
	SHL_FB_OPCODE,                /* 112    = 0x70 */
	SHL_PLL_OPCODE,                /* 113    = 0x71 */
	SHL_MC_OPCODE,                /* 114    = 0x72 */

	SHR_REG_OPCODE,                /* 115    = 0x73 */
	SHR_PS_OPCODE,                /* 116    = 0x74 */
	SHR_WS_OPCODE,                /* 117    = 0x75 */
	SHR_FB_OPCODE,                /* 118    = 0x76 */
	SHR_PLL_OPCODE,                /* 119    = 0x77 */
	SHR_MC_OPCODE,                /* 120    = 0x78 */

	DEBUG_OPCODE,                   /* 121    = 0x79 */
	CTB_DS_OPCODE,                  /* 122    = 0x7A */

	LastValidCommand = CTB_DS_OPCODE,
	/* Extension specificaTOR */
	Extension    = 0x80,    /* 128 = 0x80 Next byte is an OPCODE as well */
	Reserved_FF = 255       /* 255 = 0xFF */
};


enum cd_status {
	CD_SUCCESS,
	CD_CALL_TABLE,
	CD_COMPLETED = 0x10,
	CD_GENERAL_ERROR = 0x80,
	CD_INVALID_OPCODE,
	CD_NOT_IMPLEMENTED,
	CD_EXEC_TABLE_NOT_FOUND,
	CD_EXEC_PARAMETER_ERROR,
	CD_EXEC_PARSER_ERROR,
	CD_INVALID_DESTINATION_TYPE,
	CD_UNEXPECTED_BEHAVIOR,
	CD_INVALID_SWITCH_OPERAND_SIZE
};

enum compare_flags {
	below,
	equal,
	above,
	not_equal,
	overflow,
	no_condition
};

enum mem_resource {
	stack_resource,
	frame_buffer_resource,
	bios_image_resource
};

enum ports {
	ati_regs_port,
	pci_port,
	system_io_port
};

enum operand_type {
	type_register,
	type_param_space,
	type_work_space,
	type_frame_buffer,
	type_indirect,
	type_direct,
	type_pll,
	type_mc
};

enum destination_operand_type {
	dest_register,
	dest_param_space,
	dest_work_space,
	dest_frame_buffer,
	dest_pll,
	dest_mc
};

enum source_operand_type {
	source_register,
	source_param_space,
	source_work_space,
	source_frame_buffer,
	source_indirect,
	source_direct,
	source_pll,
	source_mc
};

enum alignment_type {
	alignment_dword,
	alignment_lower_word,
	alignment_middle_word,
	alignment_upper_word,
	alignment_byte0,
	alignment_byte1,
	alignment_byte2,
	alignment_byte3
};

struct parameters_type {
	uint8_t destination;
	uint8_t source;
};

/*
 *The following structures don't used to
 *allocate any type of objects(variables).
 *they are serve the only purpose: Get proper access to data(commands),
 *found in the tables
 */
struct pa_byte_byte {
	uint8_t pa_destination;
	uint8_t pa_source;
	uint8_t pa_padding[8];
};

struct pa_byte_word {
	uint8_t pa_destination;
	uint16_t pa_source;
	uint8_t pa_padding[7];
};

struct pa_byte_dword {
	uint8_t pa_destination;
	uint32_t pa_source;
	uint8_t pa_padding[5];
};

struct pa_word_byte {
	uint16_t pa_destination;
	uint8_t pa_source;
	uint8_t pa_padding[7];
};

struct pa_word_word {
	uint16_t pa_destination;
	uint16_t pa_source;
	uint8_t pa_padding[6];
};

struct pa_word_dword {
	uint16_t pa_destination;
	uint32_t pa_source;
	uint8_t pa_padding[4];
};

struct pa_word_xx {
	uint16_t pa_destination;
	uint8_t pa_padding[8];
};

struct pa_byte_xx {
	uint8_t pa_destination;
	uint8_t pa_padding[9];
};

/*The following 6 definitions used for Mask operation*/
struct pa_byte_byte_byte {
	uint8_t pa_destination;
	uint8_t pa_and_mask_byte;
	uint8_t pa_or_mask_byte;
	uint8_t pa_padding[7];
};

struct pa_byte_word_word {
	uint8_t pa_destination;
	uint16_t pa_and_mask_word;
	uint16_t pa_or_mask_word;
	uint8_t pa_padding[5];
};

struct pa_byte_dword_dword {
	uint8_t pa_destination;
	uint32_t pa_and_mask_dword;
	uint32_t pa_or_mask_dword;
	uint8_t pa_padding;
};

struct pa_word_byte_byte {
	uint16_t pa_destination;
	uint8_t pa_and_mask_byte;
	uint8_t pa_or_mask_byte;
	uint8_t pa_padding[6];
};

struct pa_word_word_word {
	uint16_t pa_destination;
	uint16_t pa_and_mask_word;
	uint16_t pa_or_mask_word;
	uint8_t pa_padding[4];
};

struct pa_word_dword_dword {
	uint16_t pa_destination;
	uint32_t pa_and_mask_dword;
	uint32_t pa_or_mask_dword;
};

union parameter_access {
	struct pa_byte_xx byte_xx;
	struct pa_byte_byte byte_byte;
	struct pa_byte_word byte_word;
	struct pa_byte_dword byte_dword;
	struct pa_word_byte word_byte;
	struct pa_word_word word_word;
	struct pa_word_dword word_dword;
	struct pa_word_xx Word_xx;
	/*The following 6 definitions used for Mask operation*/
	struct pa_byte_byte_byte byte_byte_and_byte_or;
	struct pa_byte_word_word byte_word_and_word_or;
	struct pa_byte_dword_dword byte_dword_and_dword_or;
	struct pa_word_byte_byte word_byte_and_byte_or;
	struct pa_word_word_word word_word_and_word_or;
	struct pa_word_dword_dword word_dword_and_dword_or;
};

struct command_attribute {
	uint8_t source:3;
	uint8_t source_alignment:3;
	uint8_t destination_alignment:2;
};

struct source_destination_alignment {
	uint8_t dest_alignment;
	uint8_t src_alignment;
};

struct multiplication_result {
	uint32_t low_32bit;
	uint32_t high_32bit;
};

struct division_result {
	uint32_t quotient32;
	uint32_t reminder32;
};

union division_multiplication_result {
	struct multiplication_result multiplication;
	struct division_result division;
};

struct command_header {
	uint8_t opcode;
	struct command_attribute attribute;
};

union word_byte_union {
	uint16_t windex;
	uint8_t bindex[2];
};

struct cmd_header {
	uint8_t opcode;
	struct command_attribute attribute;
	union word_byte_union destination;
};

struct generic_attribute_command {
	struct command_header header;
	union parameter_access parameters;
};

struct command_type_1 {
	uint8_t opcode;
	union parameter_access parameters;
};

struct command_type_opcode_offset16 {
	uint8_t opcode;
	uint16_t cd_offset16;
};

struct command_type_opcode_value_byte {
	uint8_t opcode;
	uint8_t value;
};

union command_specific_union {
	uint8_t continue_switch;
	uint8_t control_operand_source_position;
	uint8_t index_in_master_table;
};

struct compressed_command {
	uint8_t opcode;
	uint16_t offset16;
	uint8_t difference[4];
};

struct cd_generic_byte {
	uint16_t command_type:3;
	uint16_t current_parameter_size:3;
	uint16_t command_access_type:3;
	uint16_t current_port:2;
	uint16_t ps_size_by_calling_table:5;
};

struct device_data {
	uint32_t *parameter_space;
	void *dev;
	uint8_t *bios_image;
	uint32_t format;
};


struct working_table_data {
	uint8_t *table_head;
	uint8_t *ip;    /* Commands pointer */
	uint32_t *work_space;
	struct working_table_data *prev_working_table_data;
};

struct parser_temp_data {
	struct device_data *device_data;
	struct working_table_data *working_table_data;
	uint32_t source_data32;
	uint32_t dest_data32;
	union division_multiplication_result multiplication_or_division;
	uint32_t index;
	uint32_t current_fb_window;
	uint32_t indirect_data;
	uint16_t current_reg_block;
	uint16_t current_data_block;
	uint16_t attributes_data;
	uint8_t *indirect_io_table;
	uint8_t *indirect_io_table_pointer;
	struct generic_attribute_command *cmd;    /* CurrentCommand; */
	struct source_destination_alignment cd_mask;
	struct parameters_type parameters_type;
	struct cd_generic_byte multi_purpose;
	uint8_t compare_flags;
	union command_specific_union command_specific;
	enum cd_status status;
	uint8_t shift_to_mask_converter;
	uint8_t current_port_id;
	uint8_t compressed_cmd[8];
	uint8_t compressed_cmd_idx;
};

typedef void (*commands_decoder)(struct parser_temp_data *data);
typedef void (*write_io_function)(struct parser_temp_data *data);
typedef uint32_t (*read_io_function)(struct parser_temp_data *data);
typedef uint32_t (*cd_get_parameters)(struct parser_temp_data *data);

struct command_properties {
	void (*function)(struct parser_temp_data *data);
	uint8_t destination;
	uint8_t headersize;
};

struct indirect_io_parser_commands {
	void (*func)(struct parser_temp_data *data);
	uint8_t csize;
};

#pragma pack(pop)

void put_data_register(struct parser_temp_data *data);
void put_data_ps(struct parser_temp_data *data);
void put_data_ws(struct parser_temp_data *data);
void put_data_fb(struct parser_temp_data *data);
void put_data_pll(struct parser_temp_data *data);
void put_data_mc(struct parser_temp_data *data);

uint32_t get_parameters_direct32(struct parser_temp_data *data);
uint32_t get_parameters_direct16(struct parser_temp_data *data);
uint32_t get_parameters_direct8(struct parser_temp_data *data);

uint32_t get_parameters_register(struct parser_temp_data *data);
uint32_t get_parameters_ps(struct parser_temp_data *data);
uint32_t get_parameters_ws(struct parser_temp_data *data);
uint32_t get_parameters_fb(struct parser_temp_data *data);
uint32_t get_parameters_pll(struct parser_temp_data *data);
uint32_t get_parameters_mc(struct parser_temp_data *data);

void skip_parameters16(struct parser_temp_data *data);
void skip_parameters8(struct parser_temp_data *data);

uint32_t get_parameters_indirect(struct parser_temp_data *data);
uint32_t get_parameters_direct(struct parser_temp_data *data);

uint16_t *get_data_master_table_pointer(struct device_data *device_data);
uint16_t *get_command_master_table_pointer(struct device_data *device_data);
uint8_t get_true_index_in_master_table(struct parser_temp_data *data,
					uint8_t index_master_table);

void indirect_io_command(struct parser_temp_data *data);
void indirect_io_command_move(struct parser_temp_data *data, uint32_t temp);
void indirect_io_command_move_index(struct parser_temp_data *data);
void indirect_io_command_move_attr(struct parser_temp_data *data);
void indirect_io_command_move_data(struct parser_temp_data *data);
void indirect_io_command_set(struct parser_temp_data *data);
void indirect_io_command_clear(struct parser_temp_data *data);
void atom_write_fb32(struct parser_temp_data *working_table_data);
uint32_t atom_read_fb32(struct parser_temp_data *working_table_data);
void atom_delay_ms(struct parser_temp_data *working_table_data);
void atom_delay_us(struct parser_temp_data *working_table_data);
uint32_t atom_read_reg32(struct parser_temp_data *working_table_data);
void atom_write_reg32(struct parser_temp_data *working_table_data);
void atom_read_ind_reg32(struct parser_temp_data *working_table_data);
void atom_write_ind_reg32(struct parser_temp_data *working_table_data);

enum cd_status parse_table(struct device_data *device_data,
				uint8_t index_master_table);

#endif
