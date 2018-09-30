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

#include <linux/string.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/signal.h>
#include <linux/interrupt.h>

#include "gim_debug.h"
#include "gim_adapter.h"
#include "gim_os_service.h"

void  map_mmio(struct function *func, struct pci_dev *pdev)
{
	void *p_mmr_base = NULL;
	int i = 0;
	unsigned int flag = 0;

	/* Find the MMIO BAR.
	 *The MMIO has the attributes of MEMORY and non-prefetch.
	 */

	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		flag = pci_resource_flags(pdev, i);
		if ((flag & IORESOURCE_MEM) && !(flag & IORESOURCE_PREFETCH))
			break;
	}

	if (i == DEVICE_COUNT_RESOURCE) {
		gim_err("adapter has no suitable MMIO region\n");
		return;
	}

	p_mmr_base =  ioremap_nocache(pci_resource_start(pdev, i),
					pci_resource_len(pdev, i));

	if (p_mmr_base == NULL) {
		gim_err("can't iomap for BAR %d\n",  i);
		return;
	}

	func->mmr_base = p_mmr_base;
	func->mmr_size = pci_resource_len(pdev, i);

}



uint32_t  read_register(struct function *func, uint32_t index)
{
	return read_reg32(func->mmr_base, index);
}



void  write_register(struct function *func, uint32_t index, uint32_t value)
{
	return write_reg32(func->mmr_base, index, value);
}


uint32_t  pf_read_register(struct adapter *adapt, uint32_t index)
{
	return read_register(&adapt->pf, index);
}

void pf_write_register(struct adapter *adapt, uint32_t index, uint32_t value)
{
	return write_register(&adapt->pf, index, value);
}



void  delay_in_micro_seconds(uint32_t micro_seconds)
{
	udelay(micro_seconds);
}

/*
 * read_reg32() - Read the MMIO register
 * @mmr: start virtual address for MMIO register
 * @reg: register offset
 * return	register val: success
 */
uint32_t read_reg32(uint32_t *mmr, uint32_t reg)
{
	uint32_t val;

	val = ioread32((unsigned char *)mmr + (reg << 2));
	return val;
}

/*
 * read_reg32_idx() - Read the MMIO register via MM index/data
 * @mmr: start virtual address for MMIO register
 * @reg: register offset
 * return	register val: success
 */
uint32_t read_reg32_idx(uint32_t *mmr, uint32_t reg)
{

	unsigned int val;

	iowrite32(reg << 2, mmr);
	val = ioread32((unsigned char *)mmr + 4);
	return val;
}


/*
 * write_reg32() - Write the MMIO register
 * @mmr: start virtual address for MMIO register
 * @reg: register offset
 * @val: value to be write to register
 */
void write_reg32(uint32_t *mmr, uint32_t reg, uint32_t val)
{
	iowrite32(val, (unsigned char *)mmr + (reg << 2));
}

/*
 * write_reg32_idx() - Write the MMIO register via MM index/data
 * @mmr: start virtual address for MMIO register
 * @reg: register offset
 * @val: value to be write to register
 */

void write_reg32_idx(uint32_t *mmr, uint32_t reg, uint32_t val)
{
	iowrite32(reg << 2, mmr);
	iowrite32(val, (unsigned char *)mmr + 4);
}

