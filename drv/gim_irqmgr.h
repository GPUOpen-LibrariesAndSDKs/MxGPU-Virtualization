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

#ifndef _GPU_IOV_MODULE__IRQMGR_H
#define _GPU_IOV_MODULE__IRQMGR_H

#include "gim_adapter.h"

/* irq source */
#define IRQ_SOURCEX_BIF_MAILBOX_MSGBUF_VALID    0xFF00018b
#define IRQ_SOURCEX_BIF_MAILBOX_MSGBUF_ACK      0xFF00018c

/* src_id */
#define IH_IV_SRCID_BIF_PF_VF_MSGBUF_VALID      0x00000087  /* 135 */
#define IH_IV_SRCID_BIF_PF_VF_MSGBUF_ACK        0x00000088  /* 136 */
#define IH_IV_SRCID_BIF_VF_PF_MSGBUF_VALID      0x00000089  /* 137 */
#define IH_IV_SRCID_BIF_VF_PF_MSGBUF_ACK        0x0000008a  /* 138 */

/* ext_id */
#define IV_EXTID_NONE                           0x00000000

#define IVRING_SIZE                             (1 << 12)
#define IVRING_ENTRY_SIZE                       16
#define IVRING_ALIGNMENT                        0x00000100

/* ih doorbell offset */
#define DOORBELL_IH                                 0x1E8

#define IVRING_OVERFLOW_NUM_IRQ_TO_KEEP             32

#define NO_RESET          0
#define RESET_REQUEST     1

/* the interval(us) to check if VF full access timeout */
#define TIMEOUT_CHECK_INTERVAL  100 * 1000
/* timeout(us) */
#define FULLACCESS_TIMEOUT		(5000 * 1000)


enum irq_src_idx {
	IRQ_SOURCE_MAILBOX_MSGBUF_VALID = 0,
	IRQ_SOURCE_MAILBOX_MSGBUF_ACK,
	IRQ_SOURCE_MAX_NUMBER
};

struct irq_source {
	unsigned int irq_src;
	/* IV Interrupt source ID */
	unsigned int source_id;
	/* IV Interrupt extended ID */
	unsigned int extended_id;
	/* Mask register */
	unsigned int mask_reg;
	/* Mask bit to enable/disable the interrupt */
	unsigned int mask_bit;
	/* Some sources use more than 1 bit for the mask
	 * Use this to clear out all the bits
	 */
	unsigned int unmask_bit;
	/* Bitmap of all ack bits in the mask register */
	unsigned int mask_reg_ack_bits;
	/* Ack register */
	unsigned int ack_reg;
	/* Ack bit to acknowledget the interrupt */
	unsigned int ack_bit;
	/* Bitmap of all ack bits in the ack register*/
	unsigned int ack_reg_ack_bits;
	/* This function could be NULL */
	int (*enable_func)(struct adapter *adapt,
		enum irq_src_idx  irq_src, unsigned int state);
	/* This function could be NULL */
	int (*ack_func)(struct adapter *adapt, enum irq_src_idx irq_src);
};

int ih_iv_ring_init(struct adapter *adapt);
void ih_iv_ring_hw_init(struct adapter *adapt);
int ih_iv_ring_fini(struct adapter *adapt);
void ih_iv_ring_enable(struct adapter *adapt);
void ih_iv_ring_disable(struct adapter *adapt);
void ih_iv_ring_setup_wptr(struct adapter *adapt);
void ih_iv_ring_setup_rptr(struct adapter *adapt);
unsigned char ih_iv_ring_get_pointers(struct adapter *adapt,
					     unsigned int *rptr,
					     unsigned int *wptr,
					     unsigned char *over_flow);
void ih_iv_ring_update_rptr(struct adapter *adapt,
				   unsigned int rptr);
int ih_irq_source_enable(struct adapter *adapt);
int ih_irq_source_disable(struct adapter *adapt);
int ih_irq_process(void *context);

/* MAILBOX */
int mailbox_update_index(struct function *func, unsigned int index);
int mailbox_msg_rcv(struct adapter *adapt, int func_id);
int mailbox_msg_trn(struct function *func, unsigned int msg_data);
int mailbox_ack_receipt(struct function *func);
int mailbox_clear_msg_valid(struct function *func);
int mailbox_notify_flr(struct adapter *adapt, unsigned int completion);

int handle_req_gpu_init_access(struct adapter *adapt,
				int func_id, int is_reset);
int handle_rel_gpu_init_access(struct adapter *adapt, int func_id);
int handle_req_gpu_fini_access(struct adapter *adapt, int func_id);
int handle_rel_gpu_fini_access(struct adapter *adapt, int func_id);
int handle_fullaccess_timeout(struct adapter *adapt);
void signal_scheduler(void *pcontext);
#endif /* __IRQMGR_H__ */

