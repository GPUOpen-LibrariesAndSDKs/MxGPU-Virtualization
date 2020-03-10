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

#include "gim_irqmgr.h"
#include "gim_debug.h"
#include "gim_kcl_os.h"
#include "gim_timer.h"
#include "gim_command.h"
#include "gim_os_service.h"
#include "gim_s7150_reg.h"

#include <linux/string.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/signal.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>

const struct irq_source irq_sources[IRQ_SOURCE_MAX_NUMBER] = {
	[IRQ_SOURCE_MAILBOX_MSGBUF_VALID] = {
		/* irqSrc */
		IRQ_SOURCEX_BIF_MAILBOX_MSGBUF_VALID,
		/* sourceID */
		IH_IV_SRCID_BIF_VF_PF_MSGBUF_VALID,
		/* extendedID */
		IV_EXTID_NONE,
		/* mask_reg */
		mmMAILBOX_INT_CNTL,
		/* maskBit */
		MAILBOX_INT_CNTL__VALID_INT_EN_MASK,
		/* unMaskBit */
		MAILBOX_INT_CNTL__VALID_INT_EN_MASK,
		/* mask_reg_ack_bits */
		0,
		/* ackReg */
		0,
		/* ackBit */
		0,
		/* ackRegAckBits */
		0,
		/* enable_func */
		NULL,
		/* ack_func*/
		NULL,
	},
	[IRQ_SOURCE_MAILBOX_MSGBUF_ACK] = {
		IRQ_SOURCEX_BIF_MAILBOX_MSGBUF_ACK,
		IH_IV_SRCID_BIF_PF_VF_MSGBUF_ACK,
		IV_EXTID_NONE,
		mmMAILBOX_INT_CNTL,
		MAILBOX_INT_CNTL__ACK_INT_EN_MASK,
		MAILBOX_INT_CNTL__ACK_INT_EN_MASK,
		0,
		0,
		0,
		0,
		NULL,
		NULL,
	},
};

/*
 * alloc_iv_ring() - Allocate system memory for IV ring buffer
 * @adapt: pointer to amdgpuv_device
 * return	0: success
 * 		-1: failure
 */
static int alloc_iv_ring(struct adapter *adapt)
{
	unsigned int page_cnt = 0, cnt = 0;
	unsigned int size = 0;
	unsigned long *list = NULL;
	void *page = NULL;
	unsigned int idx = 0;
	struct interrupt_handler *ih = adapt->ih;

	ih->ivr_num_entries = (IVRING_SIZE/IVRING_ENTRY_SIZE);
	gim_info("ih->ivr_num_entries = %d\n", ih->ivr_num_entries);

	ih->ivr_size_in_bytes = ih->ivr_num_entries * IVRING_ENTRY_SIZE;
	gim_info("ih->ivr_size_in_bytes = %d\n", ih->ivr_size_in_bytes);

	/* Add an extra 4 Bytes for the IV Ring WPTR WB location */
	ih->ivr_alloc_size_in_bytes = ih->ivr_size_in_bytes
					+ sizeof(unsigned int);
	gim_info("ih->ivr_alloc_size_in_bytes = %d\n",
		ih->ivr_alloc_size_in_bytes);
	page_cnt = PAGE_CNT_UP(ih->ivr_alloc_size_in_bytes);
	gim_info("iv ring page_cnt = %d\n", page_cnt);

	ih->iv_ring.page_cnt = page_cnt;
	/* Add temp variable size to make the expression more readable*/
	size = page_cnt * sizeof(unsigned long);
	ih->iv_ring.page_list =
		(unsigned long *)kcl_mem_small_buffer_alloc(size);

	if (ih->iv_ring.page_list == NULL) {
		gim_info("no enough memory for iv_ring page_list\n");
		return -1;
	}
	memset(ih->iv_ring.page_list, 0, page_cnt*sizeof(unsigned long));

	for (idx = 0; idx < page_cnt; idx++) {
		page = kcl_mem_alloc_page();
		if (page == NULL) {
			gim_info("Failed to alloc page for iv ring\n");
			return -1;
		}
		ih->iv_ring.page_list[idx] = (unsigned long)page;

		/* get page */
		kcl_get_page(page);
		/* set page reserve */
		kcl_reserve_page(page);
	}

	cnt = ih->iv_ring.page_cnt;
	list = ih->iv_ring.page_list;
	ih->ivr_va = (struct iv_ring_entry *)kcl_mem_map_page_list(list, cnt);
	gim_info("ih->ivr_va = %p\n", ih->ivr_va);

	ih->ivr_ma.quad_part = pci_map_page(adapt->pf.pci_dev,
						(struct page *)list[0], 0,
						PAGE_SIZE,
						PCI_DMA_BIDIRECTIONAL);
	gim_info("ih->ivr_ma.quad_part = 0x%llx\n", ih->ivr_ma.quad_part);

	ih->ivr_wptr_wb = (unsigned int *)((unsigned int *)ih->ivr_va
						+ (ih->ivr_size_in_bytes/4));
	gim_info("ih->ivr_wptr_wb = %p\n", ih->ivr_wptr_wb);

	ih->ivr_wptr_wa.quad_part = pci_map_page(adapt->pf.pci_dev,
						    (struct page *)list[1], 0,
						    PAGE_SIZE,
						    PCI_DMA_BIDIRECTIONAL);
	gim_info("ih->ivr_wptr_wa.quad_part = 0x%llx\n",
		ih->ivr_wptr_wa.quad_part);

	/* Zero memory for iv ring and extra wb location */
	kcl_memset(ih->ivr_va, 0, ih->ivr_alloc_size_in_bytes);
	ih->is_legacy_rptr = 0;
	gim_info("update rptr via doorbell\n");

	return 0;
}

void ih_iv_ring_hw_init(struct adapter *adapt)
{
	kcl_type_u32 interrupt_cntl, interrupt_cntl2;
	kcl_type_u32 ih_cntl, ih_rb_cntl, ih_rb_base, ih_debug;
	union physical_address  rb_base_ma;
	struct interrupt_handler *ih = adapt->ih;


	/* Program IH_RB_BASE */
	ih_rb_base = read_register(&adapt->pf, mmIH_RB_BASE);
	rb_base_ma.quad_part = (ih->ivr_ma.quad_part /
				(unsigned long long)IVRING_ALIGNMENT);
	ih_rb_base = REG_SET_FIELD(ih_rb_base, IH_RB_BASE, ADDR,
					rb_base_ma.u.low_part);
	write_register(&adapt->pf, mmIH_RB_BASE, ih_rb_base);

	gim_info("the physical address of ring buffer: 0x%x",
		rb_base_ma.u.low_part);

	/* Setup IH dummy reads */
	interrupt_cntl = read_register(&adapt->pf, mmINTERRUPT_CNTL);
	interrupt_cntl2 = read_register(&adapt->pf, mmINTERRUPT_CNTL2);

	/* Use the physical address of the iv ring base as the IH
	 * dummy read address
	 */
	interrupt_cntl2 = REG_SET_FIELD(interrupt_cntl2, INTERRUPT_CNTL2,
					IH_DUMMY_RD_ADDR,
					rb_base_ma.u.low_part);
	write_register(&adapt->pf, mmINTERRUPT_CNTL2, interrupt_cntl2);

	interrupt_cntl = REG_SET_FIELD(interrupt_cntl, INTERRUPT_CNTL,
					IH_DUMMY_RD_OVERRIDE, 0);
	interrupt_cntl = REG_SET_FIELD(interrupt_cntl, INTERRUPT_CNTL,
					IH_REQ_NONSNOOP_EN, 0);
	write_register(&adapt->pf, mmINTERRUPT_CNTL, interrupt_cntl);

	/* Program IH_RB_CNTL */
	ih_rb_cntl = read_register(&adapt->pf, mmIH_RB_CNTL);
	ih_rb_cntl = REG_SET_FIELD(ih_rb_cntl, IH_RB_CNTL, RB_SIZE,
					ih->ring_size_log2);
	ih_rb_cntl = REG_SET_FIELD(ih_rb_cntl, IH_RB_CNTL, WPTR_OVERFLOW_CLEAR,
					1);
	ih_rb_cntl = REG_SET_FIELD(ih_rb_cntl, IH_RB_CNTL,
					WPTR_WRITEBACK_ENABLE, 1);
	ih_rb_cntl = REG_SET_FIELD(ih_rb_cntl, IH_RB_CNTL, RB_GPU_TS_ENABLE, 1);
	ih_rb_cntl = REG_SET_FIELD(ih_rb_cntl, IH_RB_CNTL, RPTR_REARM, 1);
	write_register(&adapt->pf, mmIH_RB_CNTL, ih_rb_cntl);

	/* Program IH_CNTL */
	ih_cntl = read_register(&adapt->pf, mmIH_CNTL);

	ih_cntl = REG_SET_FIELD(ih_cntl, IH_CNTL, MC_WRREQ_CREDIT, 0x10);
	ih_cntl = REG_SET_FIELD(ih_cntl, IH_CNTL, MC_WR_CLEAN_CNT, 0x10);
	write_register(&adapt->pf, mmIH_CNTL, ih_cntl);

	/* Program IH_DEBUG */
	ih_debug = read_register(&adapt->pf, mmIH_DEBUG);
	ih_debug = REG_SET_FIELD(ih_debug, IH_DEBUG, WPTR_OVERFLOW_ENABLE, 1);
	write_register(&adapt->pf, mmIH_DEBUG, ih_debug);

	ih_iv_ring_setup_wptr(adapt);
	ih_iv_ring_setup_rptr(adapt);
}

/*
 * free_iv_ring() - Free system memory allocated for IV ring buffer
 * @adapt: pointer to amdgpuv_device
 * return	0: success
 * 		-1: failure
 */
static int free_iv_ring(struct adapter *adapt)
{
	unsigned int idx = 0;
	void *page = NULL;
	struct interrupt_handler *ih = adapt->ih;

	if (ih->ivr_va) {
		gim_info("unmap the iv ring\n");
		kcl_mem_unmap((void *)(ih->ivr_va));
	}

	for (idx = 0; idx < ih->iv_ring.page_cnt; idx++) {
		page = (void *)ih->iv_ring.page_list[idx];
		kcl_unreserve_page(page);
		kcl_put_page(page);
		kcl_mem_free_page(page);
	}
	return 0;
}

/*
 * ih_iv_ring_init() - Init the interrupt ring
 * @adapt: pointer to amdgpuv_device
 * return	0: success
 * 		-1: failure
 */

int ih_iv_ring_init(struct adapter *adapt)
{
	struct interrupt_handler *ih = adapt->ih;

	unsigned int size_of_log2 = 0;
	unsigned int size_of_dwords = 0;
	unsigned int i = 0;

	/* Disable the interrupt before intializing iv ring */
	ih_iv_ring_disable(adapt);
	if (alloc_iv_ring(adapt) != 0)
		return -1;

	/* Translate the RB size in Bytes to be log2 based DWORDs*/
	size_of_dwords = (ih->ivr_size_in_bytes / sizeof(unsigned int));
	for (i = 0; i < 32; i++) {
		if (size_of_dwords == (1 << i)) {
			size_of_log2 = i;
			break;
		}
	}

	ih->ring_size_log2 = size_of_log2;
	ih->rptr_doorbell = (unsigned int *)adapt->pf.doorbell + DOORBELL_IH;
	ih->rptr_doorbell_offset = DOORBELL_IH;
	gim_info("ih->rptr_doorbell = %p", ih->rptr_doorbell);
	gim_info("ih->rptr_doorbell_offset = 0x%x", ih->rptr_doorbell_offset);

	ih_iv_ring_hw_init(adapt);

	/* Enable the iv ring */
	ih_iv_ring_enable(adapt);

	gim_info("init iv ring successfully");
	return 0;
}

/*
 * ih_iv_ring_fini() - Uninit the interrupt ring
 * @adapt: pointer to amdgpuv_device
 * return	0: success
 * 		-1: failure
 */

int ih_iv_ring_fini(struct adapter *adapt)
{
	struct interrupt_handler *ih = adapt->ih;

	if (free_iv_ring(adapt) != 0)
		return -1;

	if (ih != NULL) {

		if (ih->iv_ring.page_list) {
			kcl_mem_small_buffer_free(ih->iv_ring.page_list);
			ih->iv_ring.page_list = NULL;
		}

	} else {
		gim_info("No IH ring to free\n");
	}

	return 0;
}

/*
 * ih_iv_ring_enable() - Enable the interrupt ring
 * @adapt: pointer to amdgpuv_device
 * return	0: success
 * 		-1: failure
 */

void ih_iv_ring_enable(struct adapter *adapt)
{
	kcl_type_u32 ih_rb_cntl;
	struct interrupt_handler *ih = adapt->ih;

	if (ih->is_legacy_rptr) {
		write_register(&adapt->pf, mmIH_RB_RPTR, 0);
		write_register(&adapt->pf, mmIH_RB_WPTR, 0);
	}

	if (ih->ivr_wptr_wb != NULL) {
		ih->ivr_wptr_reg = *(ih->ivr_wptr_wb);
		gim_info("ih->ivr_wptr_reg = 0x%x\n", ih->ivr_wptr_reg);
		ih->ivr_wptr = ih->ivr_wptr_reg / sizeof(struct iv_ring_entry);
		gim_info("ih->ivr_wptr = %d\n", ih->ivr_wptr);
		ih->ivr_rptr_reg = ih->ivr_wptr_reg;
		gim_info("ih->ivr_rptr_reg = 0x%x\n", ih->ivr_rptr_reg);
		ih->ivr_rptr = ih->ivr_rptr_reg / sizeof(struct iv_ring_entry);
		gim_info("ih->ivr_rptr = %d\n", ih->ivr_rptr);
		*(ih->rptr_doorbell) = ih->ivr_rptr_reg;
		gim_info("*(ih->rptr_doorbell) = 0x%x\n", *(ih->rptr_doorbell));
	}

	ih_rb_cntl = read_register(&adapt->pf, mmIH_RB_CNTL);
	ih_rb_cntl = REG_SET_FIELD(ih_rb_cntl, IH_RB_CNTL, RB_ENABLE, 1);
	ih_rb_cntl = REG_SET_FIELD(ih_rb_cntl, IH_RB_CNTL, ENABLE_INTR, 1);
	write_register(&adapt->pf, mmIH_RB_CNTL, ih_rb_cntl);
}

/*
 * ih_iv_ring_disable() - Disable the interrupt ring
 * @adapt: pointer to amdgpuv_device
 */
void ih_iv_ring_disable(struct adapter *adapt)
{
	kcl_type_u32 ih_rb_cntl;
	struct interrupt_handler *ih = adapt->ih;

	ih_rb_cntl = read_register(&adapt->pf, mmIH_RB_CNTL);
	ih_rb_cntl = REG_SET_FIELD(ih_rb_cntl, IH_RB_CNTL, RB_ENABLE, 0);
	ih_rb_cntl = REG_SET_FIELD(ih_rb_cntl, IH_RB_CNTL, ENABLE_INTR, 0);
	write_register(&adapt->pf, mmIH_RB_CNTL, ih_rb_cntl);

	write_register(&adapt->pf, mmIH_RB_RPTR, 0);
	write_register(&adapt->pf, mmIH_RB_WPTR, 0);
	gim_info("disable iv ring successfully\n");

	if (ih->ivr_wptr_wb != NULL) {
		ih->ivr_wptr_reg = *(ih->ivr_wptr_wb);
		gim_info("ih->ivr_wptr_reg = 0x%x\n", ih->ivr_wptr_reg);
		ih->ivr_wptr = ih->ivr_wptr_reg / sizeof(struct iv_ring_entry);
		gim_info("ih->ivr_wptr = %d\n", ih->ivr_wptr);
		ih->ivr_rptr_reg = ih->ivr_wptr_reg;
		gim_info("ih->ivr_rptr_reg = 0x%x\n", ih->ivr_rptr_reg);
		ih->ivr_rptr = ih->ivr_rptr_reg / sizeof(struct iv_ring_entry);
		gim_info("ih->ivr_rptr = %d\n", ih->ivr_rptr);
		*(ih->rptr_doorbell) = ih->ivr_rptr_reg;
		gim_info("*(ih->rptr_doorbell) = 0x%x\n", *(ih->rptr_doorbell));
	}
}

/*
 * ih_iv_ring_setup_wptr() - Set the iv ring wptr
 * @adapt: pointer to amdgpuv_device
 */
void ih_iv_ring_setup_wptr(struct adapter *adapt)
{
	struct interrupt_handler *ih = adapt->ih;

	write_register(&adapt->pf, mmIH_RB_WPTR_ADDR_LO,
			   (unsigned int)ih->ivr_wptr_wa.u.low_part);
	write_register(&adapt->pf, mmIH_RB_WPTR_ADDR_HI,
			   (unsigned int)ih->ivr_wptr_wa.u.high_part);

	if (ih->ivr_wptr_wb != NULL)
		*ih->ivr_wptr_wb = 0;

	write_register(&adapt->pf, mmIH_RB_WPTR, 0);
}

/*
 * ih_iv_ring_setup_rptr() - Set the iv ring rptr
 * @adapt: pointer to amdgpuv_device
 */
void ih_iv_ring_setup_rptr(struct adapter *adapt)
{
	kcl_type_u32 ih_doorbell_rptr, bif_doorbell_aper_en;
	struct interrupt_handler *ih = adapt->ih;

	ih_doorbell_rptr = read_register(&adapt->pf, mmIH_DOORBELL_RPTR);
	ih_doorbell_rptr = REG_SET_FIELD(ih_doorbell_rptr, IH_DOORBELL_RPTR,
					 OFFSET, ih->rptr_doorbell_offset);

	if (ih->is_legacy_rptr)
		ih_doorbell_rptr = REG_SET_FIELD(ih_doorbell_rptr,
							IH_DOORBELL_RPTR,
							ENABLE, 0);
	else
		ih_doorbell_rptr = REG_SET_FIELD(ih_doorbell_rptr,
							IH_DOORBELL_RPTR,
							ENABLE, 1);
	write_register(&adapt->pf, mmIH_DOORBELL_RPTR, ih_doorbell_rptr);

	if (!(ih->is_legacy_rptr)) {
		/* Enable BIF doorbell aperture */
		bif_doorbell_aper_en = read_register(&adapt->pf,
						     mmBIF_DOORBELL_APER_EN);
		bif_doorbell_aper_en = REG_SET_FIELD(bif_doorbell_aper_en,
						     BIF_DOORBELL_APER_EN,
						     BIF_DOORBELL_APER_EN, 1);
		write_register(&adapt->pf, mmBIF_DOORBELL_APER_EN,
				bif_doorbell_aper_en);
		gim_info("write mmBIF_DOORBELL_APER_EN: 0x%x\n",
				bif_doorbell_aper_en);
	}

	if (ih->rptr_doorbell != NULL)
		*ih->rptr_doorbell = 0;

	write_register(&adapt->pf, mmIH_RB_RPTR, 0);
}

/*
 * ih_iv_ring_get_pointers() - Reads the iv ring read and write pointers from 
 * the hardware
 * @adapt: pointer to amdgpuv_device
 * @rptr: iv ring read pointer
 * @wptr: iv ring write pointer
 * @over_flow: iv ring overflow flag
 * return	0: success
 * 		-1: failure
 */

unsigned char ih_iv_ring_get_pointers(struct adapter *adapt,
					unsigned int *rptr,
					unsigned int *wptr,
					unsigned char *over_flow)
{
	kcl_type_u32    ih_rb_wptr;
	kcl_type_u32    ih_rb_rptr;
	struct interrupt_handler *ih = adapt->ih;
	struct iv_ring_entry iv_ring_entry;
	int r_idx;

	gim_info("ih_iv_ring_get_pointers\n");
	gim_info("ih->ivr_wptr_wb = 0x%p\n", ih->ivr_wptr_wb);
	gim_info("write offset: *(ih->ivr_wptr_wb) = 0x%x\n",
		*(ih->ivr_wptr_wb));
	gim_info("read idx: ih->ivr_rptr = 0x%x\n", ih->ivr_rptr);
	r_idx = ih->ivr_rptr;
	gim_info("Rx at entry %d in the ring\n", r_idx);
	iv_ring_entry = ih->ivr_va[r_idx];
	gim_info("iv_ring_entry.source_id = %d\n", iv_ring_entry.source_id);
	gim_info("iv_ring_entry.source_data = %d\n", iv_ring_entry.source_data);

	ih_rb_wptr = read_register(&adapt->pf, mmIH_RB_WPTR);
	ih->ivr_wptr_reg = ih_rb_wptr;
	*over_flow = (unsigned char)(REG_GET_FIELD(ih_rb_wptr, IH_RB_WPTR,
					RB_OVERFLOW));
	*wptr = (ih_rb_wptr/sizeof(struct iv_ring_entry));

	if ((*over_flow) == 0) {
		if (ih->is_legacy_rptr) {
			ih_rb_rptr = read_register(&adapt->pf, mmIH_RB_RPTR);
			ih->ivr_rptr_reg = ih_rb_rptr;
			*rptr = (ih_rb_rptr/sizeof(struct iv_ring_entry));
		} else {
			*rptr = ih->ivr_rptr;
			ih->ivr_rptr_reg =
			(*rptr) * sizeof(struct iv_ring_entry);
		}
	} else {
		gim_info("iv ring overflow happened\n");
		*rptr = (*wptr + ih->ivr_num_entries
			- IVRING_OVERFLOW_NUM_IRQ_TO_KEEP)
			% (ih->ivr_num_entries);
		ih->ivr_rptr_reg = (*rptr) * sizeof(struct iv_ring_entry);
	}

	return 0;
}

/*
 * ih_iv_ring_update_rptr() - Updates the iv ring rptr
 * @adapt: pointer to amdgpuv_device
 * @rpter: hw's read pointers index by unsigned int
 */
void ih_iv_ring_update_rptr(struct adapter *adapt, unsigned int rptr)
{
	kcl_type_u32    ih_rb_cntl = 0, ih_rb_rptr = 0;
	struct interrupt_handler *ih = adapt->ih;

	if (ih->is_overflow) {
		ih->is_overflow = 0;
		ih_rb_cntl = read_register(&adapt->pf, mmIH_RB_CNTL);
		ih_rb_cntl = REG_SET_FIELD(ih_rb_cntl, IH_RB_CNTL,
						WPTR_OVERFLOW_CLEAR, 1);
		write_register(&adapt->pf, mmIH_RB_CNTL, ih_rb_cntl);
	}

	if (ih->is_legacy_rptr) {
		ih_rb_rptr = rptr * sizeof(struct iv_ring_entry);
		gim_info("update the new rptr: ih_rb_rptr = 0x%x\n",
				ih_rb_rptr);
		write_register(&adapt->pf, mmIH_RB_RPTR, ih_rb_rptr);
	} else {
		ih->ivr_rptr_reg = rptr * sizeof(struct iv_ring_entry);
		gim_info("update the new rptr: ih->ivr_rptr_reg = 0x%x\n",
				ih->ivr_rptr_reg);
		*(ih->rptr_doorbell) = ih->ivr_rptr_reg;
	}

	if (ih->is_legacy_rptr)
		gim_info("update rptr via mmio reg: 0x%x\n", ih_rb_rptr);
	else
		gim_info("update rptr via doorbell: 0x%x\n",
				ih->ivr_rptr_reg);

	gim_info("current wptr: 0x%x\n", (unsigned int)(*(ih->ivr_wptr_wb)));
}

/*
 * ih_irq_source_enable() - Enable IRQ Sources
 * @adapt: pointer to amdgpuv_device
 * return	0: success
 * 		-1: failure
 */
int ih_irq_source_enable(struct adapter *adapt)
{
	unsigned int irq_source_count = IRQ_SOURCE_MAX_NUMBER;
	unsigned int idx = 0;
	unsigned int mask_reg = 0;
	unsigned int reg_data = 0;
	const struct irq_source *irq_source = NULL;

	for (idx = 0; idx < irq_source_count; idx++) {
		irq_source = &irq_sources[idx];
		if (irq_source == NULL) {
			gim_info("bad irq source table\n");
			return -1;
		}

		if (irq_source->mask_reg != 0) {
			mask_reg = irq_source->mask_reg;
			reg_data = read_register(&adapt->pf, mask_reg);
			gim_info("IH: read 0x%08x from mask_reg 0x%04x",
				reg_data, mask_reg);
			/* Mask out all ack bits in the mask register */
			reg_data &= ~irq_source->mask_reg_ack_bits;
			reg_data &= ~irq_source->unmask_bit;
			reg_data |= irq_source->mask_bit;
			gim_info("IH: write 0x%08x to mask_reg 0x%04x",
				reg_data, mask_reg);
			write_register(&adapt->pf, mask_reg, reg_data);
			gim_info("irq sourceID 0x%x get enabled\n",
					irq_source->source_id);
		}
	}
	return 0;
}

/*
 * ih_irq_source_disable() - Disable IRQ Sources
 * @adapt: pointer to amdgpuv_device
 * return	0: success
 * 		-1: failure
 */

int ih_irq_source_disable(struct adapter *adapt)
{
	unsigned int irq_source_count = IRQ_SOURCE_MAX_NUMBER;
	unsigned int idx = 0;
	unsigned int mask_reg = 0;
	unsigned int reg_data = 0;
	const struct irq_source *irq_source = NULL;

	for (idx = 0; idx < irq_source_count; idx++) {
		irq_source = &irq_sources[idx];
		if (irq_source == NULL) {
			gim_info("bad irq source table\n");
			return -1;
		}

		if (irq_source->mask_reg != 0) {
			mask_reg = irq_source->mask_reg;
			reg_data = read_register(&adapt->pf, mask_reg);
			reg_data &= ~irq_source->mask_reg_ack_bits;
			reg_data &= ~irq_source->unmask_bit;
			write_register(&adapt->pf, mask_reg, reg_data);
		}

		gim_info("disabled irq sourceID 0x%x\n", irq_source->source_id);
	}

	return 0;
}

/*
 * idh_queue() - Insert a new idh entry into idh queue
 * @task : the irq task needs to add in the queue
 * @event: amdgpuv idh event
 * @source_data: ih source data
 * return	0: success
 * 		VMK error code: failure
 */
static int idh_queue(struct work_task *task, enum idh_event event,
			unsigned int source_data)
{
	if (task->event == event && task->func_id == source_data &&
		task->waiting_to_execute == 1)	{
		gim_info("Event not added. abnormal event, same event twice\n");
		return 0; /* abnormal case, got twice same msg */
	}
	task->event = event;
	task->func_id = source_data;
	/* Mark the task as waiting for the scheduler to pick it up */
	task->waiting_to_execute = 1;
	gim_info("new idh: task->event = %d, task->func_id = %d\n",
		task->event, task->func_id);

	return 1;
}
static void handle_text_message(struct adapter *adapt, int func_id)
{
	int i;
	struct function *func = NULL;
	char *buf = NULL;

	for (i = 0; i < adapt->enabled_vfs; i++) {
		if (adapt->vfs[i].func_id == func_id) {
			func = &adapt->vfs[i];
			break;
		}
	}
	if (func == NULL) {
		gim_info("  func is NULL!\n");
		return;
	}
	buf = (char *)(&(func->msg_data[2]));
	for (i = 0; i < 8; ++i)	{
		func->text_msg[func->msg_len] = *buf;

		if (*buf == '\0' || ++(func->msg_len) >= MAX_MSG_LEN-1)	{
			/* In case exceeded MAX_LEN */
			func->text_msg[func->msg_len] = '\0';
			gim_info("VF%d driver message: %s\n", func_id,
				 func->text_msg);
			func->msg_len = 0;
			break;
		}
		++buf;
	}
}

/*
 * ih_irq_process() - Handle interrput event
 * @context: pointer to amdgpu_device
 * return	0: sucess
 * 		-1: failure
 */

int ih_irq_process(void *context)
{
	struct adapter *adapt = (struct adapter *)context;
	struct interrupt_handler *ih = adapt->ih;
	unsigned int irq_count = 0;
	unsigned int i = 0;
	struct iv_ring_entry iv_ring_entry;
	unsigned int event_data = 0, source_data = 0;
	struct work_task *task;
	int normal = 1;
	bool need_sched_work = false;
	unsigned int saved_mailboxindex;


	gim_info("AMD ISR is being invoked\n");

	if (ih_iv_ring_get_pointers(adapt, &ih->ivr_rptr, &ih->ivr_wptr,
					&ih->is_overflow) != 0) {
		gim_info("Failed to get the rptr and wptr\n");
		return -1;
	}

	if ((ih->ivr_rptr == ih->ivr_wptr) && (!ih->is_overflow)) {
		/* Update the rptr even if it's not amd gpu interrupt */
		ih_iv_ring_update_rptr(adapt, ih->ivr_rptr);
		gim_info("rearm MSI\n");
		return -1;
	}

	if (ih->ivr_wptr > ih->ivr_rptr)
		irq_count = ih->ivr_wptr - ih->ivr_rptr;
	else if (ih->ivr_rptr > ih->ivr_wptr)
		irq_count = (ih->ivr_num_entries - ih->ivr_rptr)
				+ ih->ivr_wptr;

	gim_info("received %d irqs in one ISR\n", irq_count);

	if (ih->ivr_va == NULL) {
		gim_info("Invalid iv ring\n");
		ih->ivr_rptr = ih->ivr_wptr;
	} else {
		/* Save the mailbox index */
		saved_mailboxindex =  get_mailbox_index(&adapt->pf);

		for (i = 0; i < irq_count; i++) {
			if (ih->ivr_rptr >= ih->ivr_num_entries) {
				gim_info("Invalid rptr and NumEntires\n");
				ih->ivr_rptr = ih->ivr_wptr;
				break;
			}

			iv_ring_entry = ih->ivr_va[ih->ivr_rptr++];
			/* Check for read pointer wraparound */
			ih->ivr_rptr %= ih->ivr_num_entries;
			source_data = iv_ring_entry.source_data;

			switch (iv_ring_entry.source_id) {
			case IH_IV_SRCID_BIF_VF_PF_MSGBUF_VALID:
				gim_info("VF_PF_MSGBUF_VALID received(Received msg from VF)");

				/* Make sure we are talking with the correct
				 * VF
				 */
				mailbox_update_index(&adapt->pf, source_data);

				/* update IDH queue
				 * msg buffer
				 */
				event_data = mailbox_msg_rcv(adapt,
								source_data);

				/* simply ack, do other things in work queue.
				 * Make sure to read the data before ACK'ing
				 */
				mailbox_ack_receipt(&adapt->pf);
				/* VF ID */
				task = &adapt->irq_tasks[source_data];

				gim_info("GPU access flag = 0x%x\n",
					 adapt->vf_req_gpu_access);
				if (event_data == IDH_TEXT_MESSAGE)
					handle_text_message(adapt,
							source_data);
				else if (event_data !=
					IDH_REQ_GPU_INVALID_ACCESS) {
					normal = idh_queue(task, event_data,
							source_data);
					need_sched_work = true;
				}
				break;

			case IH_IV_SRCID_BIF_PF_VF_MSGBUF_ACK:
				gim_info("PF_VF_MSGBUF_ACK received"
					"(VF has ACK'd the msg)\n");

				/* clear msg buffer to VF */
				mailbox_update_index(&adapt->pf,
							source_data);

				mailbox_clear_msg_valid(&adapt->pf);
				break;
			default:
				gim_info("unsupported irq source\n");
				gim_info("iv_ring_entry.source_id = 0x%x\n",
					iv_ring_entry.source_id);
				break;
			}
		}

		/* Restore the mailbox index */
		mailbox_update_index(&adapt->pf, saved_mailboxindex);


		/* schedule work in normal case */
		if (normal && need_sched_work)
			kcl_schedule_work(&adapt->normal_work);
		else {
			/* to do*/
			/* handle abnormal cases */
		}
	}

	ih_iv_ring_update_rptr(adapt, ih->ivr_rptr);

	gim_info("AMD ISR is complete\n");

	return 0;
}

int mailbox_update_index(struct function *func, unsigned int index)
{
	kcl_type_u32  mailbox_index;

	mailbox_index = read_register(func, mmMAILBOX_INDEX);
	mailbox_index = REG_SET_FIELD(mailbox_index, MAILBOX_INDEX,
				  MAILBOX_INDEX, index);
	write_register(func, mmMAILBOX_INDEX, mailbox_index);
	gim_info("write mmMAILBOX_INDEX: 0x%x\n", mailbox_index);

	return 0;
}

uint32_t get_mailbox_index(struct function *func)
{
	uint32_t index;

	index = read_register(func, mmMAILBOX_INDEX);

	return index;
}

/*
 * mailbox_msg_rcv() - Read msg from rcv register
 * @adapt: pointer to amdgpu_device
 * @func_id: function ID
 * return	val: read from rcv reg on success
 */

int mailbox_msg_rcv(struct adapter *adapt, int func_id)
{
	struct function *func = NULL;
	int i;
	int ret;

	for (i = 0; i < adapt->enabled_vfs; i++) {
		if (adapt->vfs[i].func_id == func_id) {
			func = &adapt->vfs[i];
			break;
		}
	}
	if (func == NULL) {
		gim_info("  func is NULL!\n");
		return 0;
	}

	func->msg_data[0] = read_register(&adapt->pf,
						mmMAILBOX_MSGBUF_RCV_DW0);
	func->msg_data[1] = read_register(&adapt->pf,
						mmMAILBOX_MSGBUF_RCV_DW1);
	func->msg_data[2] = read_register(&adapt->pf,
						mmMAILBOX_MSGBUF_RCV_DW2);
	func->msg_data[3] = read_register(&adapt->pf,
						mmMAILBOX_MSGBUF_RCV_DW3);
	gim_info("read mmMAILBOX_MSGBUF_RCV_DW0: 0x%x\n", func->msg_data[0]);
	gim_info("read mmMAILBOX_MSGBUF_RCV_DW1: %d", func->msg_data[1]);
	gim_info("read mmMAILBOX_MSGBUF_RCV_DW2: 0x%x\n", func->msg_data[2]);
	gim_info("read mmMAILBOX_MSGBUF_RCV_DW3: 0x%x\n", func->msg_data[3]);
	gim_dbg("func_option_valid=%d pf_option_valid=%d",
		func->user_option.valid, adapt->pf.user_option.valid);

	ret = func->msg_data[0];
	/* If the vf is not user configured but there is a VF configured,
	 * abort the access
	 */
	if (false == func->user_option.valid &&
		true == adapt->pf.user_option.valid &&
		ret == IDH_REQ_GPU_INIT_ACCESS) {
		gim_info("Unconfigured VF is not permitted to launch while "
			"there is a manually configured VF");
		ret = IDH_REQ_GPU_INVALID_ACCESS;
	} else if ((ret < IDH_REQ_GPU_INIT_ACCESS ||
		ret > IDH_REQ_GPU_RESET_ACCESS)
		&& adapt->enabled_vfs > 1) {
		/* limit the mailbox msg types */
		ret = IDH_REQ_GPU_INVALID_ACCESS;
	}

	return ret;
}
/*
 * mailbox_msg_trn() - Transmit msg to VFs
 * @func: poniter to function
 * @msg_date: message data sent to VF
 * return	0: success
 */
int mailbox_msg_trn(struct function *func, unsigned int msg_data)
{
	kcl_type_u32 mailbox_control, mailbox_msgbuf_trn_dw0, mailbox_index;

	mailbox_index = read_register(func, mmMAILBOX_INDEX);

	/* before write msg to msg buffer we need ensure TRN_MSG_VALID is 0 */
	mailbox_control = read_register(func, mmMAILBOX_CONTROL);
	mailbox_control = REG_SET_FIELD(mailbox_control,
					MAILBOX_CONTROL,
					TRN_MSG_VALID, 0);
	write_register(func, mmMAILBOX_CONTROL, mailbox_control);

	/* Write the new msg to msg buffer */
	mailbox_msgbuf_trn_dw0 = read_register(func, mmMAILBOX_MSGBUF_TRN_DW0);
	mailbox_msgbuf_trn_dw0 = REG_SET_FIELD(mailbox_msgbuf_trn_dw0,
						MAILBOX_MSGBUF_TRN_DW0,
						MSGBUF_DATA, msg_data);
	write_register(func, mmMAILBOX_MSGBUF_TRN_DW0, mailbox_msgbuf_trn_dw0);

	gim_info("write mmMAILBOX_MSGBUF_TRN_DW0: 0x%x to MAILBOX_INDEX 0x%x\n",
		 mailbox_msgbuf_trn_dw0,
		 REG_GET_FIELD(mailbox_index, MAILBOX_INDEX, MAILBOX_INDEX));

	if (msg_data != IDH_CLR_MSG_BUF)	{
	/* Inidcate the msg has been written to trn
	 * Actually this will trigger an interrupt to GFX driver (VF) if it get
	 * enabled GFX drvier can either read the msg in isr or just polling the
	 * rcv msg buffer
	 * After that, GFX driver will trigger an Ack interrupt to PF driver
	 * again PF driver should clear the trn valid fields in mailbox control
	 */

		mailbox_control = read_register(func, mmMAILBOX_CONTROL);
		mailbox_control = REG_SET_FIELD(mailbox_control,
						MAILBOX_CONTROL,
						TRN_MSG_VALID, 1);
		write_register(func, mmMAILBOX_CONTROL, mailbox_control);
		gim_info("write mmMAILBOX_CONTROL: 0x%x\n", mailbox_control);
	} else {
		gim_info("Clearing the Tx buffer - Don't set the valid data bitin mmMAILBOX_CONTROL");
	}

	return 0;
}

int mailbox_notify_flr(struct adapter *adapt, unsigned int completion)
{
	kcl_type_u32 mailbox_control, mailbox_msgbuf_trn_dw0;

	/* Write IDH_FLR_NOTIFICATION to msg buffer to indicate that flr
	 * is triggering
	 */
	mailbox_msgbuf_trn_dw0 = pf_read_register(adapt,
						  mmMAILBOX_MSGBUF_TRN_DW0);
	mailbox_msgbuf_trn_dw0 = REG_SET_FIELD(mailbox_msgbuf_trn_dw0,
						MAILBOX_MSGBUF_TRN_DW0,
						MSGBUF_DATA,
						(completion ?
						 IDH_FLR_NOTIFICATION_COMPLETION
						 : IDH_FLR_NOTIFICATION));
	pf_write_register(adapt, mmMAILBOX_MSGBUF_TRN_DW0,
				mailbox_msgbuf_trn_dw0);
	gim_info("write mmMAILBOX_MSGBUF_TRN_DW0: 0x%x\n",
		 mailbox_msgbuf_trn_dw0);

	if (!completion) {
		mailbox_control = pf_read_register(adapt, mmMAILBOX_CONTROL);
		mailbox_control = REG_SET_FIELD(mailbox_control,
						MAILBOX_CONTROL,
						TRN_MSG_VALID, 1);
		pf_write_register(adapt, mmMAILBOX_CONTROL, mailbox_control);
		gim_info("write mmMAILBOX_CONTROL: 0x%x\n", mailbox_control);
	}
	return 0;
}

int mailbox_ack_receipt(struct function *func)
{
	kcl_type_u32 mailbox_control, mailbox_index;

	mailbox_index = read_register(func, mmMAILBOX_INDEX);

	/* Ack the receipt of the message
	 * VF driver should clear the trn valid fields in mailbox control
	 * Then hw will clear the trn ack fields
	 */
	mailbox_control = read_register(func, mmMAILBOX_CONTROL);
	mailbox_control = REG_SET_FIELD(mailbox_control, MAILBOX_CONTROL,
					RCV_MSG_ACK, 1);
	write_register(func, mmMAILBOX_CONTROL, mailbox_control);

	gim_info("write mmMAILBOX_CONTROL: 0x%x to MAILBOX_INDEX 0x%x\n",
		 mailbox_control, REG_GET_FIELD(mailbox_index, MAILBOX_INDEX,
		 MAILBOX_INDEX));

	return 0;
}

int mailbox_clear_msg_valid(struct function *func)
{
	kcl_type_u32 mailbox_control;

	/* clear valid field */
	mailbox_control = read_register(func, mmMAILBOX_CONTROL);
	mailbox_control = REG_SET_FIELD(mailbox_control, MAILBOX_CONTROL,
					TRN_MSG_VALID, 0);
	write_register(func, mmMAILBOX_CONTROL, mailbox_control);
	gim_info("write mmMAILBOX_CONTROL: 0x%x\n", mailbox_control);

	return 0;
}

static void loop_once_for_all_active_VFs(struct adapter *adapt)
{
	struct function_list_node *next_node;
	struct function_list_node *curr_node = adapt->curr_running_func;
	int ret, new_quota;

	if (curr_node == NULL || curr_node == curr_node->next)
		return;

	next_node = curr_node->next;

	do {
		gim_info("switch to VF %d\n", next_node->func->func_id);

		ret = switch_vfs(curr_node->func, next_node->func);
		if (ret != GIM_OK) {
			gim_err("World Switch Failed!\n");
			gim_sched_reset(adapt, curr_node->func,
					next_node->func,
					curr_node->func->flr_reason_code
					+ next_node->func->flr_reason_code);
			if (curr_node->func->needs_flr) {
				curr_node->func->needs_flr = 0;
				if (adapt->sched_opt
					== SCHEDULER__PREDICT_PERF)
					mark_func_not_scheduled(
							curr_node->func);
			} else {
				next_node->func->needs_flr = 0;
				if (adapt->sched_opt
					== SCHEDULER__PREDICT_PERF)
					mark_func_not_scheduled(
							next_node->func);
				continue;
			}
		}

		curr_node = next_node;
		next_node = next_node->next;

		new_quota = get_scheduler_time_interval(adapt,
							curr_node->func);

		kcl_thread_sleep(new_quota);
	} while (curr_node != adapt->curr_running_func);
}

#ifdef CONFIG_MMIO_QEMU_SECURITY
int gim_sysfs_notify(struct function *func,
		     enum GIM_MMIO_STATUS stat)
{
	int ret = 0;

	if (stat == GIM_MMIO_BLOCK ||
	    stat == GIM_MMIO_UNBLOCK) {
		func->mmio_status = stat;
		sysfs_notify(&func->pci_dev->dev.kobj,
			     NULL, "listen_mmio");
	} else {
		ret = -1;
		gim_err("unsupported action for mmio\n");
	}

	return ret;
}

int  switch_mmio_status(struct function *func,
			enum GIM_MMIO_STATUS stat)
{
#ifdef QEMU_MMIO_SECURITY
	struct completion *compl = NULL;

	if (func == NULL || !(type == GIM_MMIO_BLOCK ||
			      type == GIM_MMIO_UNBLOCK)) {
		gim_err("error switch status\n");
		return ret;
	}

	gim_info("start to unblock mmio\n");

	if (stat == GIM_MMIO_BLOCK)
		compl = &func->block_complete;
	else
		compl = &func->unblock_complete;

	init_completion(compl);

        return gim_sysfs_notify(func, stat);
#else
	return 0;
#endif
}

int wait_for_mmio_complete(struct function *func,
			   enum GIM_MMIO_STATUS type)
{
#ifdef QEMU_MMIO_SECURITY
	long ret = -1;
	char str[16];
	unsigned long jiffies = msecs_to_jiffies(WAIT_COMPLETE_TIMEOUT);
	struct completion *compl = NULL;

	if (func == NULL || !(type == GIM_MMIO_BLOCK ||
			      type == GIM_MMIO_UNBLOCK)) {
		gim_err("error wait type\n");
		return ret;
	}

	memset(str, 0, sizeof(str));
	if (type == GIM_MMIO_BLOCK) {
		compl = &func->block_complete;
		strcpy(str, "block");
	} else {
		compl = &func->unblock_complete;
		strcpy(str, "unblock");
	}
	ret = wait_for_completion_interruptible_timeout(compl, jiffies);

        if (ret == -ERESTARTSYS) {
                gim_err("%s MMIO be interrupted\n", str);
                return -1;
        }
        else if (ret == 0) {
                gim_err("%s MMIO timedout\n", str);
                return -1;
        } else if (ret < 0) {
		gim_err("%s MMIO error: %ld\n", str, ret);
		return -1;
	} else {
		gim_info("%s take time: %u ms\n", str,
			jiffies_to_msecs(ret));
		return 0;
	}
#else
	return 0;
#endif
}
#endif

/*
 * handle_req_gpu_init_access() - Handle the REQ_GPU_INIT_ACCESS event on
 *				  specific vf
 * @adapt: pointer to amdgpuv pci device
 * @func_id: the vf function ID
 * @is_reset: whether the function needs to reset
 * return	0: success
 * 		-1: failure
 */

int handle_req_gpu_init_access(struct adapter *adapt, int func_id, int is_reset)
{
	struct function *function;
	int ret = 0;
	struct function_list_node *new_node;
	struct function_list_node *curr;

	gim_info("handle req_gpu_init_access event for VF%d, reset flag = %d\n",
		 func_id, is_reset);

	function = vfid_to_function(adapt, func_id);
	if (function == NULL)
		gim_warn("Can't find function for VF%d\n", func_id);

	alloc_new_vf(function, -2, 0, 0, 1);

	if (function) {
		mutex_lock(&adapt->curr_running_func_mutex);

		/* Stop the scheduler */
		/* assume timer is running at this point in normal cases */
		pause_scheduler(adapt);

		curr = adapt->curr_running_func;
		if (stop_current_vf(adapt)) {
			gim_err("Failed to stop current VF\n");
			ret = -1;
			goto out;
		}

		if (is_reset && !function->in_flr) {
			/* Try VF reset on it*/
			gim_info("VF Reset requirment from VF driver on VF%d\n",
				 function->func_id);
			if (gim_vf_flr(adapt, function) != 0)	{
				/* VF FLR couldn't recover the failure,
				 * try to use bigger hammer
				 */
				gim_info("VF Reset failed, using PF reset......\n");
				if (gim_sched_reset_gpu(adapt) != 0) {
					gim_err("PF reset failed!\n");
					ret = -1;
					goto out;
				}
				gim_info("VF reset successfully!\n");
			}
		}

#ifdef CONFIG_MMIO_QEMU_SECURITY
		/* need unblock mmio before init and run new vf*/
		switch_mmio_status(function, GIM_MMIO_UNBLOCK);
		wait_for_mmio_complete(function, GIM_MMIO_UNBLOCK);
#endif
		gim_info("init and run new VF\n");
		/* init and enable new vf */

		run_sdma(function->adapt->pf.bdf);
		run_sdma(function->bdf);

		load_register_init_state(adapt);
		new_node = is_on_run_list(function);

		/* mark function to be scheduled */
		if (new_node) {
			if (adapt->sched_opt == SCHEDULER__PREDICT_PERF)
				mark_func_scheduled(function);
			else {
				gim_err("Node is on the run_list.");
				/* Also frees the Node */
				remove_from_run_list(function);
			}
		}

		/* add to running list */
		new_node = add_func_to_run_list(function);
		if (new_node == NULL) {
			gim_err("VF%d failed to handle REQ_GPU_INIT_ACCESS\n",
				func_id);
			mutex_unlock(&adapt->curr_running_func_mutex);
			return -1;
		}

		/* record the init start time for monitoring */
		do_gettimeofday(&function->time_log.init_start);

		/* update current running vf */
		adapt->curr_running_func = new_node;

		init_vf(function);

		if (!is_reset)
			load_vbios(function);

		run_sdma(function->adapt->pf.bdf);
		run_sdma(function->bdf);
		run_vf(function);

		/* save vddgfx after IDH_REL_GPU_INIT_ACCESS */
		if (!adapt->vddgfx_state_saved[func_id]) {
			gim_save_vddgfx_state(adapt, function);
			gim_info("amdgpuv_save_vddgfx_state: VF %d vddgfx state is saved\n",
				 func_id);
			adapt->vddgfx_state_saved[func_id] = 1;
		}
out:
		mutex_unlock(&adapt->curr_running_func_mutex);
	} else {
		gim_warn("Invalid function requesting GPU access");
	}
	/* start timer for full access timeout check */
	start_timer(&adapt->timeout_timer,
			TIMEOUT_CHECK_INTERVAL);

	return ret;
}

/*
 * handle_rel_gpu_init_access() - Handle the REL_GPU_INIT_ACCESS event on
 *				  specific vf
 * @adapt: pointer to amdgpuv pci device
 * @func_id: the vf function ID
 * return	0: success
 * 		-1: failure
 */
int handle_rel_gpu_init_access(struct adapter *adapt, int func_id)
{
	struct function *function = NULL;
	int        i;
	int	   ret;

	/* stop timeout timer */
	delete_timer(&adapt->timeout_timer);

	/* get the function context */
	for (i = 0; i < adapt->enabled_vfs; i++) {
		if (adapt->vfs[i].func_id == func_id) {
			function = &adapt->vfs[i];
			break;
		}
	}
	if (function == NULL) {
		gim_warn("func %d not found", func_id);
		return (-1);
	}

#ifdef CONFIG_MMIO_QEMU_SECURITY
	switch_mmio_status(function, GIM_MMIO_BLOCK);
	wait_for_mmio_complete(function, GIM_MMIO_BLOCK);
#endif

	/* record init rel time for monitoring */
	do_gettimeofday(&function->time_log.init_end);

	if (function->in_flr) {
		gim_info("restore FLR VF %d to available\n", function->func_id);
		function->in_flr = 0;
	}

	/* restart scheduler */
	loop_once_for_all_active_VFs(adapt);

	gim_info("VF%d is indicated as the current running vf\n",
		 adapt->curr_running_func->func->func_id);
	ret = gim_init_vf_load_status(function);
	if (ret) {
		gim_err("VF[%u] not support liquid mode\n", function->func_id);
		remove_from_run_list(function);
		if (adapt->curr_running_func == NULL)
			switch_to_pf(adapt);
		else
			switch_vfs(adapt->curr_running_func->func,
				   &adapt->pf);
		init_vf_fb(adapt, function);
		if (adapt->curr_running_func == NULL)
			idle_pf(adapt);
		else
			switch_vfs(&adapt->pf,
				   adapt->curr_running_func->func);
	}
	resume_scheduler(adapt);
	return 0;
}

/*
 * handle_req_gpu_fini_access() - Handle the REQ_GPU_FINI_ACCESS event on 
 *				  specific vf
 * @adapt: pointer to amdgpuv pci device
 * @func_id: the vf function ID
 * return	0: success
 * 		-1: failure
 */
int handle_req_gpu_fini_access(struct adapter *adapt, int func_id)
{
	/* get the function context */
	struct function *func = NULL;
	int i;
	struct function_list_node *new_node;

	for (i = 0; i < adapt->enabled_vfs; i++) {
		if (adapt->vfs[i].func_id == func_id) {
			func = &adapt->vfs[i];
			break;
		}
	}

	if (func) {
		mutex_lock(&adapt->curr_running_func_mutex);

		/* Stop the scheduler
		 * assume timer is running at this point in normal cases
		 */
		pause_scheduler(adapt);
		gim_info("Scheduler has been stopped\n");

		new_node = adapt->curr_running_func;

		if (adapt->curr_running_func->func == func) {

			/* to do*/
			/* do nothing if it's the current running function */
		} else {
			while (new_node->func != func)
				new_node = new_node->next;

			/* Run the new vf
			 * stop current vf
			 */
			gim_info("Stop current VF\n");
			stop_current_vf(adapt);

			/* init and enable new vf */
			gim_info("Run new VF\n");

			load_vf(func);

			run_vf(func);

			/* update current running vf */
			adapt->curr_running_func = new_node;
		}

		mutex_unlock(&adapt->curr_running_func_mutex);
#ifdef CONFIG_MMIO_QEMU_SECURITY
		switch_mmio_status(func, GIM_MMIO_UNBLOCK);
		wait_for_mmio_complete(func, GIM_MMIO_UNBLOCK);
#endif

		/* record fini req time for monitoring */
		do_gettimeofday(&func->time_log.finish_start);
	}

	/* start timer for full access timeout check*/
	start_timer(&adapt->timeout_timer,
			TIMEOUT_CHECK_INTERVAL);
	return 0;
}

/*
 * handle_rel_gpu_fini_access() - Handle the REL_GPU_FINI_ACCESS event on
 *				  specific vf
 * @adapt: pointer to amdgpuv pci device
 * @func_id: the vf function ID
 * return	0: success
 * 		-1: failure
 */
int handle_rel_gpu_fini_access(struct adapter *adapt, int func_id)
{
	struct function *func = NULL;

	/* we don't need the spinlock here because scheduer is not started yet
	 * in normal cases.
	 *  release VF
	 *  this is done in free_vf
	 * simply start scheduler here
	 * start scheduler
	 */
	gim_info("Release fini access for VF%d\n", func_id);
	/* stop timeout timer */
	delete_timer(&adapt->timeout_timer);

	func = vfid_to_function(adapt, func_id);

	/* unhalt SDMA in case idle VF fail*/
	run_sdma(adapt->pf.bdf);
	run_sdma(func->bdf);

	if (adapt->sched_opt == SCHEDULER__PREDICT_PERF)
		/* mark function to be not scheduled */
		mark_func_not_scheduled(func);
	else
		remove_from_run_list(func);

	free_vf(func);

#ifdef CONFIG_MMIO_QEMU_SECURITY
	switch_mmio_status(func, GIM_MMIO_BLOCK);
	wait_for_mmio_complete(func, GIM_MMIO_BLOCK);
#endif

	/* record  fini req time for monitoring */
	do_gettimeofday(&func->time_log.finish_end);

	mutex_lock(&adapt->curr_running_func_mutex);
	loop_once_for_all_active_VFs(adapt);
	mutex_unlock(&adapt->curr_running_func_mutex);

	resume_scheduler(adapt);
	return 0;
}

int handle_fullaccess_timeout(struct adapter *adapt)
{
	struct function *func = NULL;
	struct function *next_func = NULL;
	struct function_list_node *node;
	struct TIMESPECTYPE time_diff;
	unsigned long time_us;
	int func_id = 0;

	if (adapt->vf_req_gpu_access == 0) {
		gim_info("GPU is not owned by any VF, stop full access handle\n");
		return 0;
	}

	func_id = adapt->vf_req_gpu_access & 0xF;
	func = vfid_to_function(adapt, func_id);

	gim_dbg("begin to check full access timeout for VF%d\n", func_id);

	if (func == NULL) {
		gim_info("failed to get vf with matched func_id");
		return 0;
	}

	if(adapt->start_time.tv_nsec == 0 && adapt->start_time.tv_sec == 0) {
		gim_info("The start time has been cleared!\n");
		return 0;
	}
	time_diff = time_elapsed(&adapt->start_time);
	time_us = time_diff.tv_sec * 1000000 + time_diff.tv_nsec / 1000;

	if (time_us < FULLACCESS_TIMEOUT) {
		gim_dbg("Need to restart the timer\n");
		return 1;
	} else {
		gim_info("VF%d full access timeout\n", func_id);
		node = adapt->curr_running_func;
		next_func = node->next->func;
		if (gim_sched_reset(adapt, func, next_func,
					FLR_REASON_EXCLUSIVE_TIMEOUT)) {
			gim_err("fullaccess timeout, reset failed!\n");
			return 0;
		}

		func->in_flr = 0;

		if (adapt->sched_opt == SCHEDULER__PREDICT_PERF)
			mark_func_not_scheduled(func);
		else
			remove_from_run_list(func);

		clear_vf_fb(adapt, func);
		adapt->vf_req_gpu_access = 0;
		if (adapt->curr_running_func != NULL)
			resume_scheduler(adapt);
		gim_info("No need to start the timer\n");
		return 0;
	}
}

/*
 * This is executed in the context of deferred work from the kernel work queue
 */
void signal_scheduler(void *pcontext)
{
	struct adapter *adapt = (struct adapter *)pcontext;
	struct work_task *req_gpu_task = NULL; /* MSG 1,3 */
	struct work_task *rel_gpu_task = NULL; /* MSG 2,4 */
	struct function *func;
	enum idh_event idh_event;

	int i;
	uint32_t reg;
	int skip = 0;

	gim_info("Invoked the task scheduler thread. Process IRQ activity\n");

	for (i = 0; i < MAX_VIRTUAL_FUNCTIONS; i++) {
		if (adapt->irq_tasks[i].waiting_to_execute == 0)
			continue;

		if (adapt->irq_tasks[i].event == IDH_REQ_GPU_INIT_ACCESS ||
		   adapt->irq_tasks[i].event == IDH_REQ_GPU_FINI_ACCESS ||
		   adapt->irq_tasks[i].event == IDH_REQ_GPU_RESET_ACCESS) {
			if (!req_gpu_task) {
				req_gpu_task = &adapt->irq_tasks[i];
				gim_info("Got a REQ_GPU_ACCESS task for VFindex %d\n",
					 i);
				gim_info("req_gpu_task --> Event = %d;"
					 "FuncID = %d\n",
					 req_gpu_task->event,
					 req_gpu_task->func_id);
			} else {
				gim_info("Multiple REQ_GPU_ACCESS requests are queued up");
				gim_info("req_gpu_task --> Event = %d;"
					 "FunID = %d", req_gpu_task->event,
					 req_gpu_task->func_id);
			}
		} else if (adapt->irq_tasks[i].event ==
				IDH_REL_GPU_INIT_ACCESS ||
				adapt->irq_tasks[i].event ==
				IDH_REL_GPU_FINI_ACCESS) {
			if (!rel_gpu_task) {
				rel_gpu_task = &adapt->irq_tasks[i];
				gim_info("Got a release GPU task\n");
				gim_info("rel_gpu_task --> Event = %d;"
					 "FunID = %d\n", rel_gpu_task->event,
					 rel_gpu_task->func_id);
				gim_info("Got a REL_GPU_ACCESS task for VFindex %d\n",
					 i);
			}
		}
	}

	if (req_gpu_task && rel_gpu_task)
		gim_info("Have both a REQ and REL task request\n");

	/* handle rel task first */
	if (rel_gpu_task) {
		idh_event = rel_gpu_task->event;
		/* Ensure that the owner of the GPU_ACCESS is the one that is
		 * releasing it
		 * Set the 0x80 bit to indicate that it is owned, lower bits are
		 * VF ID
		 */
		if (adapt->vf_req_gpu_access == 0x80 + rel_gpu_task->func_id) {
			switch (idh_event) {
			case IDH_REL_GPU_INIT_ACCESS:
				/* clear the timeout timestamp */
				adapt->start_time.tv_sec = 0;
				adapt->start_time.tv_nsec = 0;
				
				gim_info("IDH_REL_GPU_INIT_ACCESS\n");
				reg = pf_read_register(adapt, mmGRBM_STATUS);
				gim_info("    GRBM_STATUS = 0x%08x\n", reg);
				/* send SIG_VF_TRAP_MMIO to QEMU */
				handle_rel_gpu_init_access(adapt,
						rel_gpu_task->func_id);
				/* Task is finished */
				rel_gpu_task->waiting_to_execute = 0;

				break;
			case IDH_REL_GPU_FINI_ACCESS:
				/* clear the timeout timestamp */
				adapt->start_time.tv_sec = 0;
				adapt->start_time.tv_nsec = 0;
				
				gim_info("IDH_REL_GPU_FINI_ACCESS\n");
				/* send SIG_VF_TRAP_MMIO to QEMU */
				handle_rel_gpu_fini_access(adapt,
						rel_gpu_task->func_id);

				rel_gpu_task->waiting_to_execute = 0;

				break;
			default:
				gim_info("invalid idh event\n");
				break;
			}

			/* release global flag */
			adapt->vf_req_gpu_access = 0;
		} else {
			/* this is an abnormal request, dump it */
			rel_gpu_task->waiting_to_execute = 0;
			gim_err("Serious error! Release GPU when Idon'thave it\n");
		}

	}

	if (req_gpu_task) {
		if (rel_gpu_task != NULL)
			gim_info("Processing a Request_GPU after finishinga Release_GPU\n");
		idh_event = req_gpu_task->event;
		if (!adapt->vf_req_gpu_access) {
			switch (idh_event) {
			case IDH_REQ_GPU_INIT_ACCESS:
				gim_info("IDH_REQ_GPU_INIT_ACCESS\n");
				/* set the timestamp for full access
				 * timeout check
				 */
				GETNSTIMEOFDAY(&adapt->start_time);
				/* send SIG_VF_EXCLUSIVE_MMIO to QEMU */
				handle_req_gpu_init_access(adapt,
				req_gpu_task->func_id, NO_RESET);

				/* set executed in case this task will not be
				 * picked up next time
				 */
				req_gpu_task->waiting_to_execute = 0;
				mailbox_update_index(&adapt->pf,
							req_gpu_task->func_id);
				mailbox_msg_trn(&adapt->pf,
						IDH_READY_TO_ACCESS_GPU);

				break;

			case IDH_REQ_GPU_RESET_ACCESS:
				gim_info("IDH_REQ_GPU_RESET_ACCESS\n");
				/* set the timestamp for full access timeout
				 * check
				 */
				GETNSTIMEOFDAY(&adapt->start_time);
				handle_req_gpu_init_access(adapt,
				req_gpu_task->func_id, RESET_REQUEST);

				req_gpu_task->waiting_to_execute = 0;
				mailbox_update_index(&adapt->pf,
							req_gpu_task->func_id);
				mailbox_msg_trn(&adapt->pf,
						IDH_READY_TO_ACCESS_GPU);

				break;
			case IDH_REQ_GPU_FINI_ACCESS:
				gim_info("IDH_REQ_GPU_FINI_ACCESS\n");
				/*check if the request function is on run list*/
				func = vfid_to_function(adapt,
						req_gpu_task->func_id);
				if ((!func->is_scheduled
					&& adapt->sched_opt
					 == SCHEDULER__PREDICT_PERF)
					|| (!is_on_run_list(func)
					&& adapt->sched_opt
					 == SCHEDULER__ROUND_ROBIN_SOLID)
					|| (!is_on_run_list(func)
					&& adapt->sched_opt
					 == SCHEDULER__ROUND_ROBIN_LIQUID)) {
					req_gpu_task->waiting_to_execute = 0;
					skip = 1;
					break;
				}
				/* set the timestamp for full access timeout
				 * check
				 */
				GETNSTIMEOFDAY(&adapt->start_time);
				/* send SIG_VF_EXCLUSIVE_MMIO to QEMU */
				handle_req_gpu_fini_access(adapt,
						req_gpu_task->func_id);
				/* set executed in case this task will not be
				 * picked up next time
				 */
				req_gpu_task->waiting_to_execute = 0;
				mailbox_update_index(&adapt->pf,
						req_gpu_task->func_id);
				mailbox_msg_trn(&adapt->pf,
						IDH_READY_TO_ACCESS_GPU);

				break;
			default:
				gim_info("invalid idh event\n");
				break;
			}
			/* set global flag
			 * interrupt event 1&2 or 3&4 should not be seperated.
			 *
			 * Mark flag as 'in use' and identify owner if "skip"
			 * is not set
			 */
			if (!skip)
				adapt->vf_req_gpu_access = 0x80
							+ req_gpu_task->func_id;
		} else {
			gim_info("VF%d REQ GPU but VF%d already owns GPU\n",
				 req_gpu_task->func_id,
				 adapt->vf_req_gpu_access & 0xF);
		}

	}
}
