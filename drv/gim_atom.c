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

#include "gim_atom.h"
#include "gim_adapter.h"
#include "gim_debug.h"
#include "gim_os_service.h"
#include "gim_atombios_com.h"

/*
 * waive the coding style (80 characters limitation) for this file.
 */

read_io_function get_para_direct_array[8] = {
	get_parameters_direct32,
	get_parameters_direct16,
	get_parameters_direct16,
	get_parameters_direct16,
	get_parameters_direct8,
	get_parameters_direct8,
	get_parameters_direct8,
	get_parameters_direct8
};

commands_decoder put_data_func[6] = {
	put_data_register,
	put_data_ps,
	put_data_ws,
	put_data_fb,
	put_data_pll,
	put_data_mc
};

cd_get_parameters get_destination[6] = {
	get_parameters_register,
	get_parameters_ps,
	get_parameters_ws,
	get_parameters_fb,
	get_parameters_pll,
	get_parameters_mc
};

commands_decoder skip_destination[6] = {
	skip_parameters16,
	skip_parameters8,
	skip_parameters8,
	skip_parameters8,
	skip_parameters8,
	skip_parameters8
};

cd_get_parameters get_src[8] = {
	get_parameters_register,
	get_parameters_ps,
	get_parameters_ws,
	get_parameters_fb,
	get_parameters_indirect,
	get_parameters_direct,
	get_parameters_pll,
	get_parameters_mc
};

struct indirect_io_parser_commands indirect_io_parser_cmd[10] = {
	{indirect_io_command, 1},
	{indirect_io_command, 2},
	{atom_read_ind_reg32, 3},
	{atom_write_ind_reg32, 3},
	{indirect_io_command_clear, 3},
	{indirect_io_command_set, 3},
	{indirect_io_command_move_index, 4},
	{indirect_io_command_move_attr, 4},
	{indirect_io_command_move_data, 4},
	{indirect_io_command, 3}
};

uint32_t align_mask[8] = {
	0xFFFFFFFF,
	0xFFFF,
	0xFFFF,
	0xFFFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF
};

uint8_t source_alignment_shift[8] = { 0, 0, 8, 16, 0, 8, 16, 24 };
uint8_t destination_alignment_shift[4] = { 0, 8, 16, 24 };

void atom_write_fb32(struct parser_temp_data *working_table_data)
{
}

uint32_t atom_read_fb32(struct parser_temp_data *working_table_data)
{
	return 0;
}

void atom_delay_ms(struct parser_temp_data *working_table_data)
{
	delay_in_micro_seconds(working_table_data->source_data32*1000);
}

void atom_delay_us(struct parser_temp_data *working_table_data)
{
	delay_in_micro_seconds(working_table_data->source_data32);
}

uint32_t atom_read_reg32(struct parser_temp_data *working_table_data)
{
	struct adapter *dev;
	unsigned int offset;

	dev = working_table_data->device_data->dev;
	offset = (uint16_t) working_table_data->index;

	if ((offset << 2) >= dev->pf.mmr_size)
		return read_reg32_idx((void *)dev->pf.mmr_base, offset);

	return read_reg32((void *)dev->pf.mmr_base, offset);
}

void atom_write_reg32(struct parser_temp_data *working_table_data)
{
	void  *base;
	struct adapter *p_adapter;
	unsigned int offset;

	p_adapter = working_table_data->device_data->dev;
	base = (void *)p_adapter->pf.mmr_base;
	offset = (uint16_t) working_table_data->index;
	if ((offset << 2) >= p_adapter->pf.mmr_size)
		write_reg32_idx(base, offset, working_table_data->dest_data32);
	else
		write_reg32(base, offset, working_table_data->dest_data32);
}

void atom_read_ind_reg32(struct parser_temp_data *working_table_data)
{
	void  *base;
	struct adapter *p_adapter;
	unsigned int offset;

	p_adapter = working_table_data->device_data->dev;
	base = (void *)p_adapter->pf.mmr_base;
	offset = *(uint16_t *)(working_table_data->indirect_io_table_pointer + 1);

	if ((offset << 2) >= p_adapter->pf.mmr_size)
		working_table_data->indirect_data = read_reg32_idx(base, offset);
	else
		working_table_data->indirect_data = read_reg32(base, offset);
}

void atom_write_ind_reg32(struct parser_temp_data *working_table_data)
{
	uint8_t *ptr;
	struct adapter *p_adapter;
	unsigned int offset;

	p_adapter = working_table_data->device_data->dev;
	ptr = working_table_data->indirect_io_table_pointer;
	offset = *(uint16_t *)(ptr + 1);

	if ((offset << 2) >= p_adapter->pf.mmr_size)
		write_reg32_idx((void *)p_adapter->pf.mmr_base,
				offset,
				working_table_data->indirect_data);
	else
		write_reg32((void *)p_adapter->pf.mmr_base,
			   offset,
			   working_table_data->indirect_data);
}

void indirect_io_command(struct parser_temp_data *data)
{
}

void indirect_io_command_move_index(struct parser_temp_data *data)
{
	uint8_t a_off;
	uint8_t b_off;
	uint8_t c_off;

	a_off = data->indirect_io_table_pointer[1];
	b_off = data->indirect_io_table_pointer[2];
	c_off = data->indirect_io_table_pointer[3];

	data->indirect_data &= ~((0xFFFFFFFF >> (32 - a_off)) << c_off);
	data->indirect_data |= (((data->index >> b_off)
				& (0xFFFFFFFF >> (32 - a_off))) << c_off);
}

void indirect_io_command_move_attr(struct parser_temp_data *data)
{
	uint8_t a_off;
	uint8_t b_off;
	uint8_t c_off;

	a_off = data->indirect_io_table_pointer[1];
	b_off = data->indirect_io_table_pointer[2];
	c_off = data->indirect_io_table_pointer[3];

	data->indirect_data &= ~((0xFFFFFFFF >> (32 - a_off)) << c_off);
	data->indirect_data |= (((data->attributes_data >> b_off)
				& (0xFFFFFFFF >> (32 - a_off))) << c_off);
}

void indirect_io_command_move_data(struct parser_temp_data *data)
{
	uint8_t a_off;
	uint8_t b_off;
	uint8_t c_off;

	a_off = data->indirect_io_table_pointer[1];
	b_off = data->indirect_io_table_pointer[2];
	c_off = data->indirect_io_table_pointer[3];

	data->indirect_data &= ~((0xFFFFFFFF >> (32 - a_off)) << c_off);
	data->indirect_data |= (((data->dest_data32 >> b_off) &
		  (0xFFFFFFFF >> (32 - a_off))) << c_off);
}

void indirect_io_command_set(struct parser_temp_data *data)
{
	uint8_t a_off;
	uint8_t b_off;

	a_off = data->indirect_io_table_pointer[1];
	b_off = data->indirect_io_table_pointer[2];

	data->indirect_data |= ((0xFFFFFFFF >> (32 - a_off)) << b_off);
}

void indirect_io_command_clear(struct parser_temp_data *data)
{
	uint8_t a_off;
	uint8_t b_off;

	a_off = data->indirect_io_table_pointer[1];
	b_off = data->indirect_io_table_pointer[2];

	data->indirect_data &= ~((0xFFFFFFFF >> (32 - a_off)) << b_off);
}

uint32_t indirect_input_output(struct parser_temp_data *data)
{
	uint8_t size;
	uint8_t idx;
	uint16_t off;

	while (*data->indirect_io_table_pointer) {
		if (data->indirect_io_table_pointer[1] == data->indirect_data
		     && data->indirect_io_table_pointer[0] == INDIRECTIO_ID) {
			idx = *data->indirect_io_table_pointer;
			size = indirect_io_parser_cmd[idx].csize;
			data->indirect_io_table_pointer += size;

			while (*data->indirect_io_table_pointer
				!= INDIRECTIO_END_OF_ID) {
				idx = *data->indirect_io_table_pointer;
				indirect_io_parser_cmd[idx].func(data);
				size = indirect_io_parser_cmd[idx].csize;
				data->indirect_io_table_pointer += size;
			}

			off = *(uint16_t *)(data->indirect_io_table_pointer + 1);
			data->indirect_io_table_pointer -= off;
			data->indirect_io_table_pointer++;
			return data->indirect_data;
		}

		idx = *data->indirect_io_table_pointer;
		size = indirect_io_parser_cmd[idx].csize;
		data->indirect_io_table_pointer += size;
	}

	return 0;
}

void put_data_register(struct parser_temp_data *data)
{
	struct generic_attribute_command *cmd;

	cmd = (struct generic_attribute_command *) data->compressed_cmd;

	data->index = (uint32_t)cmd->parameters.Word_xx.pa_destination;
	data->index += data->current_reg_block;
	if (data->multi_purpose.current_port == ati_regs_port) {
		if (data->current_port_id == INDIRECT_IO_MM) {
			if (data->index == 0)
				data->dest_data32 <<= 2;
			atom_write_reg32(data);
		} else {
			data->indirect_data =
				data->current_port_id + INDIRECT_IO_WRITE;
			indirect_input_output(data);
		}
	}
}

void put_data_ps(struct parser_temp_data *data)
{
	struct generic_attribute_command *cmd;

	cmd = (struct generic_attribute_command *) data->compressed_cmd;

	*(data->device_data->parameter_space
	 + cmd->parameters.byte_xx.pa_destination) = data->dest_data32;
}

void put_data_ws(struct parser_temp_data *data)
{
	struct generic_attribute_command *cmd;
	uint8_t dest;

	cmd = (struct generic_attribute_command *) data->compressed_cmd;
	dest = cmd->parameters.byte_xx.pa_destination;

	if (dest < WS_QUOTIENT_C)
		*(data->working_table_data->work_space + dest) =
							data->dest_data32;
	else if (dest == WS_REMINDER_C)
		data->multiplication_or_division.division.reminder32 =
							data->dest_data32;
	else if (dest == WS_QUOTIENT_C)
		data->multiplication_or_division.division.quotient32 =
							data->dest_data32;
	else if (dest == WS_DATAPTR_C)
		data->current_data_block = (uint16_t) data->dest_data32;
	else if (dest == WS_SHIFT_C)
		data->shift_to_mask_converter = (uint8_t) data->dest_data32;
	else if (dest == WS_FB_WINDOW_C)
		data->current_fb_window = data->dest_data32;
	else if (dest == WS_ATTRIBUTES_C)
		data->attributes_data = (uint16_t) data->dest_data32;
	else if (dest == WS_REGPTR_C)
		data->current_reg_block = (uint16_t) data->dest_data32;

}

void get_byte_dest_idx(struct parser_temp_data *data)
{
	struct generic_attribute_command *cmd;

	cmd = (struct generic_attribute_command *)data->compressed_cmd;
	data->index = (uint32_t)cmd->parameters.byte_xx.pa_destination;
}

void get_byte_src_idx(struct parser_temp_data *data)
{
	uint8_t idx;

	if (data->cmd->header.opcode > 0x80) {
		idx = data->compressed_cmd_idx;
		data->index = (uint32_t)data->compressed_cmd[idx];
		data->compressed_cmd_idx++;
	} else {
		data->index = (uint32_t)*data->working_table_data->ip;
		data->working_table_data->ip++;
	}
}

void get_word_src_idx(struct parser_temp_data *data)
{
	uint8_t idx;

	if (data->cmd->header.opcode > 0x80) {
		idx = data->compressed_cmd_idx;
		data->index = (uint32_t)
				*((uint16_t *) &data->compressed_cmd[idx]);
		data->compressed_cmd_idx += 2;
	} else {
		data->index = (uint32_t)
				*(uint16_t *)data->working_table_data->ip;
		data->working_table_data->ip += 2;
	}
}

void get_dword_src_idx(struct parser_temp_data *data)
{
	uint8_t idx;

	if (data->cmd->header.opcode > 0x80) {
		idx = data->compressed_cmd_idx;
		data->index = *((uint32_t *) &data->compressed_cmd[idx]);
		data->compressed_cmd_idx += 4;
	} else {
		data->index = *(uint32_t *)data->working_table_data->ip;
		data->working_table_data->ip += 4;
	}
}

void put_data_fb(struct parser_temp_data *data)
{
	get_byte_dest_idx(data);
	/* Make an Index from address first, then add to the Index */
	data->index += (data->current_fb_window >> 2);
	atom_write_fb32(data);
}

void put_data_pll(struct parser_temp_data *data)
{
	get_byte_dest_idx(data);
}

void put_data_mc(struct parser_temp_data *data)
{
	get_byte_dest_idx(data);
}

void skip_parameters8(struct parser_temp_data *data)
{
	if (data->cmd->header.opcode > 0x80)
		data->compressed_cmd_idx++;
	else
		data->working_table_data->ip++;
}

void skip_parameters16(struct parser_temp_data *data)
{
	if (data->cmd->header.opcode > 0x80)
		data->compressed_cmd_idx += 2;
	else
		data->working_table_data->ip += 2;
}

uint32_t get_parameters_register(struct parser_temp_data *data)
{
	get_word_src_idx(data);
	data->index += data->current_reg_block;
	if (data->current_port_id == INDIRECT_IO_MM)
		return atom_read_reg32(data);

	data->indirect_data = data->current_port_id + INDIRECT_IO_READ;
	return indirect_input_output(data);
}

uint32_t get_parameters_ps(struct parser_temp_data *data)
{
	get_byte_src_idx(data);
	return *(data->device_data->parameter_space + data->index);
}

uint32_t get_parameters_ws(struct parser_temp_data *data)
{
	union division_multiplication_result *div;

	get_byte_src_idx(data);
	if (data->index < WS_QUOTIENT_C)
		return *(data->working_table_data->work_space + data->index);

	switch (data->index) {
	case WS_REMINDER_C:
		div = &data->multiplication_or_division;
		return div->division.reminder32;
	case WS_QUOTIENT_C:
		div = &data->multiplication_or_division;
		return div->division.quotient32;
	case WS_DATAPTR_C:
		return (uint32_t)data->current_data_block;
	case WS_OR_MASK_C:
		return ((uint32_t) 1) << data->shift_to_mask_converter;
	case WS_AND_MASK_C:
		return ~(((uint32_t) 1) << data->shift_to_mask_converter);
	case WS_FB_WINDOW_C:
		return data->current_fb_window;
	case WS_ATTRIBUTES_C:
		return data->attributes_data;
	case WS_REGPTR_C:
		return (uint32_t) data->current_reg_block;
	}

	return 0;

}

uint32_t get_parameters_pll(struct parser_temp_data *data)
{
	get_byte_src_idx(data);
	return 1;
}

uint32_t get_parameters_mc(struct parser_temp_data *data)
{
	get_byte_src_idx(data);
	return 1;
}

uint32_t get_parameters_fb(struct parser_temp_data *data)
{
	get_byte_src_idx(data);
	data->index += (data->current_fb_window >> 2);
	return atom_read_fb32(data);
}

#define RELATIVE_TO_BIOS_IMAGE(x) \
		((uint64_t)x \
		+ (uint64_t)(((struct device_data *)data->device_data)->bios_image))

uint32_t get_parameters_indirect(struct parser_temp_data *data)
{
	get_word_src_idx(data);
	return *(uint32_t *) (RELATIVE_TO_BIOS_IMAGE(data->index)
			      + data->current_data_block);
}

uint32_t get_parameters_direct8(struct parser_temp_data *data)
{
	data->cd_mask.src_alignment = alignment_byte0;
	get_byte_src_idx(data);
	return data->index;
}

uint32_t get_parameters_direct16(struct parser_temp_data *data)
{
	data->cd_mask.src_alignment = alignment_lower_word;
	get_word_src_idx(data);
	return data->index;
}

uint32_t get_parameters_direct32(struct parser_temp_data *data)
{
	data->cd_mask.src_alignment = alignment_dword;
	get_dword_src_idx(data);
	return data->index;
}

uint32_t get_parameters_direct(struct parser_temp_data *data)
{
	uint8_t src_align;
	struct generic_attribute_command *cmd;

	cmd = (struct generic_attribute_command *)data->compressed_cmd;
	src_align = cmd->header.attribute.source_alignment;

	return get_para_direct_array[src_align](data);
}

void common_src_data_trans(struct parser_temp_data *data)
{
	uint8_t src_align;
	uint8_t dest_align;

	src_align = data->cd_mask.src_alignment;
	dest_align = data->cd_mask.dest_alignment;

	data->source_data32 >>= source_alignment_shift[src_align];
	data->source_data32 &= align_mask[src_align];
	data->source_data32 <<= destination_alignment_shift[dest_align];
	data->index = data->dest_data32;
}

void common_dest_data_trans(struct parser_temp_data *data)
{
	uint8_t src_align;
	uint8_t dest_align;

	src_align = data->cd_mask.src_alignment;
	dest_align = data->cd_mask.dest_alignment;

	data->index &= ~(align_mask[src_align]
			<< destination_alignment_shift[dest_align]);
	data->dest_data32 >>= destination_alignment_shift[dest_align];
	data->dest_data32 &= align_mask[src_align];
	data->dest_data32 <<= destination_alignment_shift[dest_align];
	data->dest_data32 |= data->index;
}

void common_opt_data_trans(struct parser_temp_data *data)
{
	uint8_t src_align;
	uint8_t dest_align;

	src_align = data->cd_mask.src_alignment;
	dest_align = data->cd_mask.dest_alignment;

	data->source_data32 >>= source_alignment_shift[src_align];
	data->source_data32 &= align_mask[src_align];
	data->dest_data32 >>= destination_alignment_shift[dest_align];
	data->dest_data32 &= align_mask[src_align];
}

void proc_move(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;
	uint8_t src_align;
	uint8_t dest_align;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;
	src_align = data->cd_mask.src_alignment;
	dest_align = data->cd_mask.dest_alignment;

	if (src_align != alignment_dword)
		data->dest_data32 = get_destination[dest](data);
	else
		skip_destination[dest] (data);

	data->source_data32 = get_src[source] (data);

	if (src_align != alignment_dword) {
		data->dest_data32 &= ~(align_mask[src_align]
				     << destination_alignment_shift[dest_align]);
		common_src_data_trans(data);
		data->dest_data32 |= data->source_data32;
	} else {
		data->dest_data32 = data->source_data32;
	}

	put_data_func[dest](data);
}

void proc_mask(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;
	uint8_t src_align;
	uint8_t dest_align;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;
	src_align = data->cd_mask.src_alignment;
	dest_align = data->cd_mask.dest_alignment;

	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_parameters_direct(data);
	data->index = get_src[source](data);
	data->source_data32 <<= destination_alignment_shift[dest_align];
	data->source_data32 |= ~(align_mask[src_align]
				<< destination_alignment_shift[dest_align]);
	data->dest_data32 &= data->source_data32;

	data->index >>= source_alignment_shift[src_align];
	data->index &= align_mask[src_align];
	data->index <<= destination_alignment_shift[dest_align];
	data->dest_data32 |= data->index;
	put_data_func[dest](data);
}

void proc_and(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;
	uint8_t src_align;
	uint8_t dest_align;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;
	src_align = data->cd_mask.src_alignment;
	dest_align = data->cd_mask.dest_alignment;

	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_src[source](data);
	data->source_data32 >>= source_alignment_shift[src_align];
	data->source_data32 <<= destination_alignment_shift[dest_align];
	data->source_data32 |= ~(align_mask[src_align]
				<< destination_alignment_shift[dest_align]);
	data->dest_data32 &= data->source_data32;
	put_data_func[dest](data);
}

void proc_or(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;

	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_src[source](data);
	common_src_data_trans(data);
	data->dest_data32 |= data->source_data32;
	put_data_func[dest] (data);
}

void proc_xor(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;

	data->dest_data32 = get_destination[dest] (data);
	data->source_data32 = get_src[source] (data);
	common_src_data_trans(data);
	data->dest_data32 ^= data->source_data32;
	put_data_func[dest](data);
}

void proc_shl(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;

	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_src[source](data);
	common_src_data_trans(data);
	data->dest_data32 <<= data->source_data32;
	common_dest_data_trans(data);
	put_data_func[dest](data);
}

void proc_shr(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;

	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_src[source](data);
	common_src_data_trans(data);
	data->dest_data32 >>= data->source_data32;
	common_dest_data_trans(data);
	put_data_func[dest](data);
}

void proc_add(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;

	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_src[source](data);
	common_src_data_trans(data);
	data->dest_data32 += data->source_data32;
	common_dest_data_trans(data);
	put_data_func[dest](data);
}

void proc_sub(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;

	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_src[source](data);
	common_src_data_trans(data);
	data->dest_data32 -= data->source_data32;
	common_dest_data_trans(data);
	put_data_func[dest](data);
}

void proc_mul(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;

	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_src[source](data);
	common_opt_data_trans(data);
	data->multiplication_or_division.multiplication.low_32bit =
				data->dest_data32 * data->source_data32;
}

void proc_div(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;

	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_src[source](data);

	common_opt_data_trans(data);
	data->multiplication_or_division.division.quotient32 =
				data->dest_data32 / data->source_data32;
	data->multiplication_or_division.division.reminder32 =
				data->dest_data32 % data->source_data32;
}

void proc_compare(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;
	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_src[source](data);
	common_opt_data_trans(data);

	/* Here we just set flags based on evaluation */
	if (data->dest_data32 == data->source_data32)
		data->compare_flags = equal;
	else if (data->dest_data32 < data->source_data32)
		data->compare_flags = below;
	else
		data->compare_flags = above;

}

void proc_clear(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t alignment;

	dest = data->parameters_type.destination;
	alignment = data->cd_mask.src_alignment;

	data->dest_data32 = get_destination[dest](data);
	data->dest_data32 &= ~(align_mask[alignment]
			       << source_alignment_shift[alignment]);
	put_data_func[dest](data);

}

void proc_shift(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t alignment;
	uint32_t mask;

	alignment = data->cd_mask.src_alignment;
	mask = align_mask[alignment]	<< source_alignment_shift[alignment];

	dest = data->parameters_type.destination;
	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_parameters_direct8(data);

	/* save original value of the destination */
	data->index = data->dest_data32 & ~mask;
	data->dest_data32 &= mask;

	if (((struct generic_attribute_command *)
		data->compressed_cmd)->header.opcode < SHIFT_RIGHT_REG_OPCODE)
		data->dest_data32 <<= data->source_data32;
	else
		data->dest_data32 >>= data->source_data32;

	/* Clear any bits shifted out of masked area... */
	data->dest_data32 &= mask;
	/* ... and restore the area outside of masked with original values */
	data->dest_data32 |= data->index;

	/* write data back */
	put_data_func[data->parameters_type.destination](data);
}

void proc_test(struct parser_temp_data *data)
{
	uint8_t dest;
	uint8_t source;

	dest = data->parameters_type.destination;
	source = data->parameters_type.source;

	data->dest_data32 = get_destination[dest](data);
	data->source_data32 = get_src[source](data);
	common_opt_data_trans(data);
	if (data->dest_data32 & data->source_data32)
		data->compare_flags = (uint8_t)not_equal;
	else
		data->compare_flags = (uint8_t)equal;
}

void proc_set_fb_base(struct parser_temp_data *data)
{
	uint8_t alignment;
	uint8_t source;

	source = data->parameters_type.source;
	alignment = data->cd_mask.src_alignment;

	data->source_data32 = get_src[source](data);
	data->source_data32 >>= source_alignment_shift[alignment];
	data->source_data32 &= align_mask[alignment];
	data->current_fb_window = data->source_data32;
}

void proc_switch(struct parser_temp_data *data)
{
	uint8_t alignment;
	uint8_t source;

	source = data->parameters_type.source;
	alignment = data->cd_mask.src_alignment;
	data->source_data32 = get_src[source](data);
	data->source_data32 >>=
			source_alignment_shift[alignment];
	data->source_data32 &= align_mask[alignment];
	while (*(uint16_t *) data->working_table_data->ip
		!= (((uint16_t) NOP_OPCODE << 8) + NOP_OPCODE)) {
		if (*data->working_table_data->ip == 'c') {
			data->working_table_data->ip++;
			data->dest_data32 = get_parameters_direct(data);
			data->index = get_parameters_direct16(data);
			if (data->source_data32 == data->dest_data32) {
				data->working_table_data->ip =
					RELATIVE_TO_TABLE(data->index);
				return;
			}
		}
	}
	data->working_table_data->ip += sizeof(uint16_t);
}

void cmd_set_data_block(struct parser_temp_data *data)
{
	struct command_type_1 *cmd;
	uint8_t value;
	uint16_t *pMasterdataTable;

	cmd = (struct command_type_1 *)data->working_table_data->ip;

	value = cmd->parameters.byte_xx.pa_destination;
	if (value == 0)
		data->current_data_block = 0;
	else {
		if (value == DB_CURRENT_COMMAND_TABLE) {
			data->current_data_block =
					(uint16_t)
					(data->working_table_data->table_head
					- data->device_data->bios_image);
		} else {
			pMasterdataTable =
				get_data_master_table_pointer(data->device_data);
			data->current_data_block =
				(uint16_t)
				((uint16_t *)pMasterdataTable)[value];
		}
	}
	data->working_table_data->ip += sizeof(struct command_type_opcode_value_byte);
}

void cmd_set_ati_port(struct parser_temp_data *data)
{
	struct command_type_1 *cmd;

	cmd = (struct command_type_1 *)data->working_table_data->ip;

	data->multi_purpose.current_port = ati_regs_port;
	data->current_port_id = (uint8_t) cmd->parameters.Word_xx.pa_destination;
	data->working_table_data->ip += sizeof(struct command_type_opcode_offset16);
}

void cmd_set_reg_block(struct parser_temp_data *data)
{
	struct command_type_1 *cmd;

	cmd = (struct command_type_1 *)data->working_table_data->ip;

	data->current_reg_block = cmd->parameters.Word_xx.pa_destination;
	data->working_table_data->ip += sizeof(struct command_type_opcode_offset16);
}

/* Atavism!!! Review!!! */
void cmd_set_x_port(struct parser_temp_data *data)
{
	data->multi_purpose.current_port = data->parameters_type.destination;
	data->working_table_data->ip += sizeof(uint8_t);

}

void cmd_delay_millisec(struct parser_temp_data *data)
{
	struct command_type_1 *cmd;

	cmd = (struct command_type_1 *)data->working_table_data->ip;
	data->source_data32 = cmd->parameters.byte_xx.pa_destination;
	atom_delay_ms(data);
	data->working_table_data->ip += sizeof(struct command_type_opcode_value_byte);
}

void cmd_delay_microsec(struct parser_temp_data *data)
{
	struct command_type_1 *cmd;

	cmd = (struct command_type_1 *)data->working_table_data->ip;
	data->source_data32 = cmd->parameters.byte_xx.pa_destination;
	atom_delay_us(data);
	data->working_table_data->ip += sizeof(struct command_type_opcode_value_byte);
}

void proc_post_char(struct parser_temp_data *data)
{
}

void proc_debug(struct parser_temp_data *data)
{
}

void proc_ds(struct parser_temp_data *data)
{
	struct command_type_1 *cmd;

	cmd = (struct command_type_1 *)data->working_table_data->ip;
	data->working_table_data->ip += cmd->parameters.Word_xx.pa_destination
					+ sizeof(struct command_type_opcode_offset16);
}

void cmd_call_table(struct parser_temp_data *data)
{
	uint8_t val;
	uint16_t *master_table_off;
	struct atom_common_rom_command_table_header *tab_hdr;

	data->working_table_data->ip += sizeof(struct command_type_opcode_value_byte);
	master_table_off = get_command_master_table_pointer(data->device_data);

	val = ((struct command_type_opcode_value_byte *)data->cmd)->value;
	/* if the offset is not ZERO */
	if (((uint16_t *) master_table_off)[val] != 0) {
		data->command_specific.index_in_master_table =
				get_true_index_in_master_table(data, val);

		tab_hdr = (struct atom_common_rom_command_table_header *)
			   data->working_table_data->table_head;
		data->multi_purpose.ps_size_by_calling_table =
				(tab_hdr->table_attr.ps_size_in_bytes >> 2);

		data->device_data->parameter_space +=
			data->multi_purpose.ps_size_by_calling_table;
		data->status = CD_CALL_TABLE;
		data->cmd = (struct generic_attribute_command *)master_table_off;
	}
}

void cmd_nop(struct parser_temp_data *data)
{

}

static void not_implemented(struct parser_temp_data *data)
{
	data->status = CD_NOT_IMPLEMENTED;
}

void proc_jump(struct parser_temp_data *data)
{
	struct command_type_opcode_offset16 *offset;

	offset = (struct command_type_opcode_offset16 *)data->working_table_data->ip;

	if ((data->parameters_type.destination == no_condition)
	     || (data->parameters_type.destination == data->compare_flags)) {
		data->working_table_data->ip =
				RELATIVE_TO_TABLE(offset->cd_offset16);
	} else {
		data->working_table_data->ip +=
				sizeof(struct command_type_opcode_offset16);
	}
}

void proc_jumpe(struct parser_temp_data *data)
{
	struct command_type_opcode_offset16 *offset;

	offset = (struct command_type_opcode_offset16 *)data->working_table_data->ip;

	if ((data->compare_flags == equal)
	    || (data->compare_flags == data->parameters_type.destination)) {
		data->working_table_data->ip =
				RELATIVE_TO_TABLE(offset->cd_offset16);
	} else {
		data->working_table_data->ip +=
				sizeof(struct command_type_opcode_offset16);
	}
}

void proc_jumpne(struct parser_temp_data *data)
{
	struct command_type_opcode_offset16 *offset;

	offset = (struct command_type_opcode_offset16 *)data->working_table_data->ip;

	if (data->compare_flags != equal) {
		data->working_table_data->ip =
				RELATIVE_TO_TABLE(offset->cd_offset16);
	} else {
		data->working_table_data->ip +=
				sizeof(struct command_type_opcode_offset16);
	}
}

struct command_properties call_tab[] = {
	{NULL, 0, 0},
	{proc_move, dest_register, sizeof(struct command_header)},
	{proc_move, dest_param_space, sizeof(struct command_header)},
	{proc_move, dest_work_space, sizeof(struct command_header)},
	{proc_move, dest_frame_buffer, sizeof(struct command_header)},
	{proc_move, dest_pll, sizeof(struct command_header)},
	{proc_move, dest_mc, sizeof(struct command_header)},
	{proc_and, dest_register, sizeof(struct command_header)},
	{proc_and, dest_param_space, sizeof(struct command_header)},
	{proc_and, dest_work_space, sizeof(struct command_header)},
	{proc_and, dest_frame_buffer, sizeof(struct command_header)},
	{proc_and, dest_pll, sizeof(struct command_header)},
	{proc_and, dest_mc, sizeof(struct command_header)},
	{proc_or, dest_register, sizeof(struct command_header)},
	{proc_or, dest_param_space, sizeof(struct command_header)},
	{proc_or, dest_work_space, sizeof(struct command_header)},
	{proc_or, dest_frame_buffer, sizeof(struct command_header)},
	{proc_or, dest_pll, sizeof(struct command_header)},
	{proc_or, dest_mc, sizeof(struct command_header)},
	{proc_shift, dest_register, sizeof(struct command_header)},
	{proc_shift, dest_param_space, sizeof(struct command_header)},
	{proc_shift, dest_work_space, sizeof(struct command_header)},
	{proc_shift, dest_frame_buffer, sizeof(struct command_header)},
	{proc_shift, dest_pll, sizeof(struct command_header)},
	{proc_shift, dest_mc, sizeof(struct command_header)},
	{proc_shift, dest_register, sizeof(struct command_header)},
	{proc_shift, dest_param_space, sizeof(struct command_header)},
	{proc_shift, dest_work_space, sizeof(struct command_header)},
	{proc_shift, dest_frame_buffer, sizeof(struct command_header)},
	{proc_shift, dest_pll, sizeof(struct command_header)},
	{proc_shift, dest_mc, sizeof(struct command_header)},
	{proc_mul, dest_register, sizeof(struct command_header)},
	{proc_mul, dest_param_space, sizeof(struct command_header)},
	{proc_mul, dest_work_space, sizeof(struct command_header)},
	{proc_mul, dest_frame_buffer, sizeof(struct command_header)},
	{proc_mul, dest_pll, sizeof(struct command_header)},
	{proc_mul, dest_mc, sizeof(struct command_header)},
	{proc_div, dest_register, sizeof(struct command_header)},
	{proc_div, dest_param_space, sizeof(struct command_header)},
	{proc_div, dest_work_space, sizeof(struct command_header)},
	{proc_div, dest_frame_buffer, sizeof(struct command_header)},
	{proc_div, dest_pll, sizeof(struct command_header)},
	{proc_div, dest_mc, sizeof(struct command_header)},
	{proc_add, dest_register, sizeof(struct command_header)},
	{proc_add, dest_param_space, sizeof(struct command_header)},
	{proc_add, dest_work_space, sizeof(struct command_header)},
	{proc_add, dest_frame_buffer, sizeof(struct command_header)},
	{proc_add, dest_pll, sizeof(struct command_header)},
	{proc_add, dest_mc, sizeof(struct command_header)},
	{proc_sub, dest_register, sizeof(struct command_header)},
	{proc_sub, dest_param_space, sizeof(struct command_header)},
	{proc_sub, dest_work_space, sizeof(struct command_header)},
	{proc_sub, dest_frame_buffer, sizeof(struct command_header)},
	{proc_sub, dest_pll, sizeof(struct command_header)},
	{proc_sub, dest_mc, sizeof(struct command_header)},
	{cmd_set_ati_port, ati_regs_port, 0},
	{cmd_set_x_port, pci_port, 0},
	{cmd_set_x_port, system_io_port, 0},
	{cmd_set_reg_block, 0, 0},
	{proc_set_fb_base, 0, sizeof(struct command_header)},
	{proc_compare, dest_register, sizeof(struct command_header)},
	{proc_compare, dest_param_space, sizeof(struct command_header)},
	{proc_compare, dest_work_space, sizeof(struct command_header)},
	{proc_compare, dest_frame_buffer, sizeof(struct command_header)},
	{proc_compare, dest_pll, sizeof(struct command_header)},
	{proc_compare, dest_mc, sizeof(struct command_header)},
	{proc_switch, 0, sizeof(struct command_header)},
	{proc_jump, no_condition, 0},
	{proc_jump, equal, 0},
	{proc_jump, below, 0},
	{proc_jump, above, 0},
	{proc_jumpe, below, 0},
	{proc_jumpe, above, 0},
	{proc_jumpne, 0, 0},
	{proc_test, dest_register, sizeof(struct command_header)},
	{proc_test, dest_param_space, sizeof(struct command_header)},
	{proc_test, dest_work_space, sizeof(struct command_header)},
	{proc_test, dest_frame_buffer, sizeof(struct command_header)},
	{proc_test, dest_pll, sizeof(struct command_header)},
	{proc_test, dest_mc, sizeof(struct command_header)},
	{cmd_delay_millisec, 0, 0},
	{cmd_delay_microsec, 0, 0},
	{cmd_call_table, 0, 0},
	{not_implemented, 0, 0},
	{proc_clear, dest_register, sizeof(struct command_header)},
	{proc_clear, dest_param_space, sizeof(struct command_header)},
	{proc_clear, dest_work_space, sizeof(struct command_header)},
	{proc_clear, dest_frame_buffer, sizeof(struct command_header)},
	{proc_clear, dest_pll, sizeof(struct command_header)},
	{proc_clear, dest_mc, sizeof(struct command_header)},
	{cmd_nop, 0, sizeof(uint8_t)},
	{cmd_nop, 0, sizeof(uint8_t)},
	{proc_mask, dest_register, sizeof(struct command_header)},
	{proc_mask, dest_param_space, sizeof(struct command_header)},
	{proc_mask, dest_work_space, sizeof(struct command_header)},
	{proc_mask, dest_frame_buffer, sizeof(struct command_header)},
	{proc_mask, dest_pll, sizeof(struct command_header)},
	{proc_mask, dest_mc, sizeof(struct command_header)},
	{proc_post_char, 0, 0},
	{not_implemented, 0, 0},
	{not_implemented, 0, 0},
	{not_implemented, 0, 0},
	{cmd_set_data_block, 0, 0},
	{proc_xor, dest_register, sizeof(struct command_header)},
	{proc_xor, dest_param_space, sizeof(struct command_header)},
	{proc_xor, dest_work_space, sizeof(struct command_header)},
	{proc_xor, dest_frame_buffer, sizeof(struct command_header)},
	{proc_xor, dest_pll, sizeof(struct command_header)},
	{proc_xor, dest_mc, sizeof(struct command_header)},
	{proc_shl, dest_register, sizeof(struct command_header)},
	{proc_shl, dest_param_space, sizeof(struct command_header)},
	{proc_shl, dest_work_space, sizeof(struct command_header)},
	{proc_shl, dest_frame_buffer, sizeof(struct command_header)},
	{proc_shl, dest_pll, sizeof(struct command_header)},
	{proc_shl, dest_mc, sizeof(struct command_header)},
	{proc_shr, dest_register, sizeof(struct command_header)},
	{proc_shr, dest_param_space, sizeof(struct command_header)},
	{proc_shr, dest_work_space, sizeof(struct command_header)},
	{proc_shr, dest_frame_buffer, sizeof(struct command_header)},
	{proc_shr, dest_pll, sizeof(struct command_header)},
	{proc_shr, dest_mc, sizeof(struct command_header)},
	{proc_debug, 0, 0},
	{proc_ds, 0, 0},
};

uint8_t proc_cmd_properties(struct parser_temp_data *data)
{
	uint8_t index;
	uint8_t opcode;
	uint8_t *tmp_cmd;
	struct command_header *cmd_hdr;

	opcode = ((struct command_header *)data->working_table_data->ip)->opcode;
	if (opcode > 0x80) {
		tmp_cmd = (uint8_t *) (data->device_data->bios_image
			    + ((struct compressed_command *)data->cmd)->offset16);

		data->compressed_cmd_idx = 0;
		if (((struct compressed_command *) data->cmd)->offset16 & 0x8000) {
			data->compressed_cmd[0] = *tmp_cmd++;
			data->compressed_cmd_idx++;
			opcode <<= 1;
			opcode |= 1;
		} else {
			tmp_cmd += 0x8000;
		}

		data->working_table_data->ip += 3;
		while (opcode != 0x80) {
			index = data->compressed_cmd_idx;
			if (opcode & 0x80)
				data->compressed_cmd[index] = *tmp_cmd;
			else
				data->compressed_cmd[index] =
						*data->working_table_data->ip++;

			data->compressed_cmd_idx++;
			tmp_cmd++;
			opcode <<= 1;
		}

		opcode = data->compressed_cmd[0];
		data->compressed_cmd_idx = call_tab[opcode].headersize;

		cmd_hdr = (struct command_header *)data->compressed_cmd;
		data->parameters_type.source = cmd_hdr->attribute.source;
		data->cd_mask.src_alignment =
				cmd_hdr->attribute.source_alignment;

		data->cd_mask.dest_alignment =
				cmd_hdr->attribute.destination_alignment;

	} else {
		for (data->compressed_cmd_idx = 0;
		     data->compressed_cmd_idx < 8;
		     data->compressed_cmd_idx++) {
			index = data->compressed_cmd_idx;
			data->compressed_cmd[index] =
				data->working_table_data->ip[index];
		}

		data->working_table_data->ip += call_tab[opcode].headersize;
		data->parameters_type.source =
			data->cmd->header.attribute.source;
		data->cd_mask.src_alignment =
			data->cmd->header.attribute.source_alignment;
		data->cd_mask.dest_alignment =
			data->cmd->header.attribute.destination_alignment;
	}

	data->parameters_type.destination = call_tab[opcode].destination;
	return opcode;
}

uint16_t *get_command_master_table_pointer(struct device_data *device_data)
{
	uint16_t *master_table_off = NULL;

	master_table_off = (uint16_t *)(*(uint16_t *)(device_data->bios_image
			     + OFFSET_TO_POINTER_TO_ATOM_ROM_HEADER)
			     + device_data->bios_image);

	master_table_off = (uint16_t *)((uint32_t)((struct atom_rom_header *)
			    master_table_off)->master_command_table_offset
			    + device_data->bios_image);

	master_table_off = (uint16_t *)&(((struct atom_master_command_table *)
			    master_table_off)->list_of_command_tables);

	return master_table_off;
}

uint16_t *get_data_master_table_pointer(struct device_data *device_data)
{
	uint16_t *master_table_off;

	master_table_off = (uint16_t *)(*(uint16_t *)(device_data->bios_image
			    + OFFSET_TO_POINTER_TO_ATOM_ROM_HEADER)
			    + device_data->bios_image);

	master_table_off = (uint16_t *)((uint32_t)((struct atom_rom_header *)
			    master_table_off)->master_data_table_offset
			    + device_data->bios_image);

	master_table_off = (uint16_t *)&(((struct atom_master_data_table *)
			    master_table_off)->tables_list);

	return master_table_off;
}

uint8_t get_true_index_in_master_table(struct parser_temp_data *data,
				  uint8_t index_master_table)
{
	return index_master_table;
}

enum cd_status parse_table(struct device_data *device_data, uint8_t index_master_table)
{
	struct parser_temp_data data;
	struct working_table_data *prev_working_table_data;

	data.device_data = (struct device_data *) device_data;
	data.cmd = (struct generic_attribute_command *)get_data_master_table_pointer(device_data);
	data.indirect_io_table_pointer = (uint8_t *)((uint32_t) ((struct atom_master_list_of_data_tables *)data.cmd)->indirect_io_access + device_data->bios_image);
	data.indirect_io_table_pointer += sizeof(struct atom_common_table_header);

	data.cmd = (struct generic_attribute_command *)get_command_master_table_pointer(device_data);
	index_master_table = get_true_index_in_master_table((struct parser_temp_data *)&data, index_master_table);
	/* if the offset is not ZERO */
	if (((uint16_t *) data.cmd)[index_master_table] != 0) {
		data.command_specific.index_in_master_table = index_master_table;
		data.multi_purpose.current_port = ati_regs_port;
		data.current_port_id = INDIRECT_IO_MM;
		data.current_reg_block = 0;
		data.current_fb_window = 0;
		prev_working_table_data = NULL;
		data.status = CD_CALL_TABLE;

		do {
			if (data.status == CD_CALL_TABLE) {
				index_master_table = data.command_specific.index_in_master_table;
				/* if the offset is not ZERO */
				if (((uint16_t *) data.cmd)[index_master_table] != 0) {
					data.working_table_data = (struct working_table_data *) kmalloc(((struct atom_common_rom_command_table_header *)
								(((uint16_t *)data.cmd)[index_master_table] + device_data->bios_image))->table_attr.ws_size_in_bytes +
							sizeof(struct working_table_data), GFP_KERNEL);
					if (data.working_table_data != NULL) {
						data.working_table_data->work_space = (uint32_t *)((uint8_t *)data.working_table_data + sizeof(struct working_table_data));
						data.working_table_data->table_head = (uint8_t *)(((uint16_t *)data.cmd)[index_master_table] + device_data->bios_image);
						data.working_table_data->ip = ((uint8_t *)data.working_table_data->table_head) + sizeof(struct atom_common_rom_command_table_header);
						data.working_table_data->prev_working_table_data = prev_working_table_data;
						prev_working_table_data = data.working_table_data;
						data.status = CD_SUCCESS;
					} else {
						data.status = CD_UNEXPECTED_BEHAVIOR;
					}
				} else {
					data.status = CD_EXEC_TABLE_NOT_FOUND;
				}
			}
			if (!CD_ERROR(data.status)) {
				data.status = CD_SUCCESS;
				while (!CD_ERROR_OR_COMPLETED(data.status)) {
					data.cmd = (struct generic_attribute_command *)data.working_table_data->ip;

					if (IS_END_OF_TABLE(((struct command_header *)data.working_table_data->ip)->opcode)) {
						data.status = CD_COMPLETED;
						prev_working_table_data = data.working_table_data->prev_working_table_data;

						kfree(data.working_table_data);
						data.working_table_data = prev_working_table_data;
						if (prev_working_table_data != NULL) {
							data.device_data->parameter_space -=
								(((struct atom_common_rom_command_table_header *)data.working_table_data->table_head)->table_attr.ps_size_in_bytes >> 2);
						}
					} else {
						index_master_table = proc_cmd_properties((struct parser_temp_data *) &data);
						(*call_tab[index_master_table].function)((struct parser_temp_data *)&data);
						delay_in_micro_seconds(10);
					}
				}
			} else {
				break;
			}
		} while (prev_working_table_data != NULL);

		if (data.status == CD_COMPLETED)
			return CD_SUCCESS;

		return data.status;
	} else
		return CD_SUCCESS;
}
