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

#ifndef _GPU_IOV_MODULE__OS_SERVICE_H
#define _GPU_IOV_MODULE__OS_SERVICE_H

#include <linux/pci.h>

#define REG_FIELD_SHIFT(reg, field) reg##__##field##__SHIFT
#define REG_FIELD_MASK(reg, field) reg##__##field##_MASK

#define REG_SET_FIELD(orig_val, reg, field, field_val)			\
	(((orig_val) & ~REG_FIELD_MASK(reg, field)) |			\
	(REG_FIELD_MASK(reg, field) &					\
	((field_val) << REG_FIELD_SHIFT(reg, field))))

#define REG_GET_FIELD(value, reg, field)				\
	(((value) & REG_FIELD_MASK(reg,					\
	field)) >> REG_FIELD_SHIFT(reg, field))

void  map_mmio(struct function *func, struct pci_dev *pdev);
uint32_t read_register(struct function *func, uint32_t index);
void write_register(struct function *func, uint32_t index, uint32_t value);
uint32_t pf_read_register(struct adapter *adapt, uint32_t index);
void pf_write_register(struct adapter *adapt, uint32_t index, uint32_t value);
void delay_in_micro_seconds(uint32_t micro_seconds);
void write_reg32_idx(uint32_t *mmr, uint32_t reg, uint32_t val);
void write_reg32(uint32_t *mmr, uint32_t reg, uint32_t val);
uint32_t read_reg32_idx(uint32_t *mmr, uint32_t reg);
uint32_t read_reg32(uint32_t *mmr, uint32_t reg);

#endif

