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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>


#include "gim_dma.h"
#include "gim_adapter.h"
#include "gim_command.h"
#include "gim_config.h"
#include "gim_gpuiov.h"
#include "gim_debug.h"
#include "gim_os_service.h"

#include "gim_s7150_reg.h"

/* Quick define for 5msec in units of nsec to be used with timespec. */
#define FIVE_MSEC__IN_NSEC 5000000

void *map_vf_fb(struct pci_dev *pdev);

/*
 * The DMA FIFO can hold two DMA requests.
 * Check and wait until there is at least one free spot available
 */
static int wait_dma_ready(struct adapter *adapt)
{
	struct TIMESPECTYPE start_time;
	unsigned long dma_cntl = 0;
	int rc = 0;

	GETNSTIMEOFDAY(&start_time);
	do {
		dma_cntl = pf_read_register(adapt, mmCP_DMA_CNTL);
		/* DMA has 2 PIO commands in the FIFO
		 * (top 2 bits are command count)
		 */
	} while ((dma_cntl & 0x80000000)
		 && (time_elapsed(&start_time).tv_nsec
		     < FIVE_MSEC__IN_NSEC)); /* Max 5 msec wait */

	if (dma_cntl & 0x80000000) {
		gim_info("DMA failed to make room for another cmd after 5 msec\n");
		gim_info(" dma_cntl = 0x%08lx\n", dma_cntl);
		rc = -1;
	}
	return rc;
}

/*
 * Assume: The CP DMA has the bandwidth of around 10 GBytes per second.
 * The time consumed is around 100 us for 1M bytes. Wait 5ms for safty.
 */
static int wait_dma_complete(struct adapter *adapt)
{
	struct TIMESPECTYPE start_time;
	unsigned int cp_stat = 0;
	int rc = 0;

	GETNSTIMEOFDAY(&start_time);
	do {
		cp_stat = pf_read_register(adapt, mmCP_STAT);
	} while ((cp_stat & 0x80400000)/* CP_BUSY or DMA_BUSY */
		/* Max 5 msec wait */
		 && (time_elapsed(&start_time).tv_nsec < FIVE_MSEC__IN_NSEC));

	if (cp_stat & 0x80400000) {
		gim_info("Timed out waiting for dma to finish.\n");
		gim_info(" cp_stat = 0x%08x\n", cp_stat);
		rc = -2;
	}
	return rc;
}

/*
 * Fill a region of VRAM with a pattern.
 */
static int dma_fill(struct adapter *adapt, unsigned long long offset,
		    unsigned long long size)
{
	int rc = 0;
	unsigned int pattern = 0xdeadbeef;

	/* Make sure that there is room for a new DMA command */
	rc = wait_dma_ready(adapt);
	if (rc) {
		gim_info("dma_fill() failed while waiting for dma_ready()\n");
		return rc;
	}

	/* src = data, dst = das, dst_mtype = 3(uncacheable) */
	pf_write_register(adapt, mmCP_DMA_PIO_CONTROL, 0x40c00c00);

	/* src: data(0) */
	pf_write_register(adapt, mmCP_DMA_PIO_SRC_ADDR, pattern);
	pf_write_register(adapt, mmCP_DMA_PIO_SRC_ADDR_HI, 0);

	/* dst =  offset */
	pf_write_register(adapt, mmCP_DMA_PIO_DST_ADDR,
			  (unsigned long) (offset & 0xffffffff));
	pf_write_register(adapt, mmCP_DMA_PIO_DST_ADDR_HI,
			  (unsigned long) ((offset >> 32) & 0xffffffff));

	/* command: RAW_WAIT SAIC: 1(not inc) DAS: 0(memory) SAS: 1(register)
	 * DIS_WC: 0
	 */
	pf_write_register(adapt,
			  mmCP_DMA_PIO_COMMAND,
			  0x54000000 | (((unsigned long) size) & 0x1fffff));

	return rc;
}

/*
 *dma_copy() - Copy a bunch of date from VRAM
 *@adapt: pointer to amdgpu_device
 *@src: source of the data
 *@dest: destination of the copy
 *@size: size of the data to be copied
 * return	0: success
 *		-1: failure
 */
int dma_copy(struct adapter *adapt, unsigned long long src,
	     unsigned long long dest, unsigned long long size)
{
	int rc = 0;

	rc = wait_dma_ready(adapt);
	if (rc) {
		gim_info("dma_copy failed with rc=%d\n", rc);
		return rc;
	}

	/* src = sas, src_mtype = 3, dst = das, dst_mtype = 3(uncacheable) */
	pf_write_register(adapt, mmCP_DMA_PIO_CONTROL, 0x00c00c00);

	/* src: data(0) */
	pf_write_register(adapt, mmCP_DMA_PIO_SRC_ADDR,
			  (unsigned long) (src & 0xffffffff));
	pf_write_register(adapt, mmCP_DMA_PIO_SRC_ADDR_HI,
			  (unsigned long) ((src >> 32) & 0xffffffff));

	/* dst =  offset */
	pf_write_register(adapt, mmCP_DMA_PIO_DST_ADDR,
			  (unsigned long) (dest & 0xffffffff));
	pf_write_register(adapt, mmCP_DMA_PIO_DST_ADDR_HI,
			  (unsigned long) ((dest >> 32) & 0xffffffff));

	/* command: RAW_WAIT DAS: 0(memory) SAS: 0(memory) DIS_WC: 0 */
	pf_write_register(adapt, mmCP_DMA_PIO_COMMAND,
			  0x40000000 | (unsigned long) size);

	return rc;
}
/*
 * Clear a region of FB memory using the following algorithm...
 *
 * Start by filling a 64 byte block at the start of the FB range with
 * a pattern (0x0 by default) and set the destination pointer to the end
 * of the block.
 *
 * Copy from the start of the block to the destination pointer which will
 * double the block size that has been cleared. Set the destination pointer
 * to the end of the block. On each iteration through the loop copy from the
 * start of the FB range to the destination while doubling the block size.
 *
 * After the block size is 1MB then do dma copies of 1MB at a time until
 * the limit is reached.
 */
int dma_clear(struct adapter *adapt, unsigned long long offset,
		unsigned long long size)
{
	unsigned long long	src_addr;
	unsigned long long	dest_addr;
	unsigned long long	block_size = 64;
	unsigned long long	copied_size = 0;
	int			rc = 0;

	/*
	 * Get the MCAddr of the start of the PF Framebuffer
	 * The VF Framebuffer will start at "PF_MCAddr + offset" in the
	 * PF address space.
	 */
	unsigned long pf_mc_addr = pf_read_register(adapt,
						    mmMC_VM_FB_LOCATION);

	/*
	 * src_addr will always point to the beginning offset of the region
	 * to clear (the 'seed' block) dest_addr will move through the region
	 *  as blocks are copied
	 */
	dest_addr = ((unsigned long long) (pf_mc_addr & 0xffff) << 24) + offset;
	src_addr = dest_addr;

	gim_info("Clear VRAM from 0x%08llx to 0x%08llx\n",
		 offset, offset + size - 1);

	/* Establish first 64 byte patterned block ('seed' block) */
	rc = dma_fill(adapt, dest_addr, block_size);
	if (rc)
		gim_warn("dma_fill() failed with rc = %d\n", rc);

	/* Advance the destination pointer by the filled region size. */
	dest_addr += block_size;
	/* We have already copied the first block
	 * by filling it with a pattern
	 */
	copied_size = block_size;

	/* If dma_fill failed it won't go into 'while' loop */
	while (rc == 0 && copied_size < size) {
		/* Make sure we don't go past end of region */
		if (copied_size + block_size > size)
			block_size = size - copied_size;

		rc = dma_copy(adapt, src_addr, dest_addr, block_size);

		dest_addr += block_size;
		copied_size += block_size;

		/* Block size is still less than 1MB */
		if (block_size < 0x100000)
			/* double the block size for the next iteration */
			block_size <<= 1;
	}
	wait_dma_complete(adapt);
	gim_info("dma_clear() - FB has been cleared by dma_copy.\n");

	return rc;
}
