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
#include <linux/vmalloc.h>

#include "gim_kcl_os.h"
#include "gim_debug.h"

void kcl_memset(void *s, int c, kcl_type_u64 count)
{
	memset(s, c, count);
}

void *kcl_mem_small_buffer_alloc(kcl_type_u32 size)
{
	return kmalloc(size, GFP_KERNEL);
}

void kcl_mem_small_buffer_free(void *p)
{
	kfree(p);
}

void *kcl_mem_alloc_page()
{
	return (void *)alloc_page(GFP_KERNEL | __GFP_HIGHMEM);
}

void kcl_mem_free_page(void *p)
{
	__free_page(p);
}

void *kcl_mem_alloc_page_size(unsigned int page_count)
{
	return kmalloc(page_count * PAGE_SIZE, GFP_KERNEL);
}

void *kcl_mem_map_page_list(unsigned long *pagelist, unsigned int count)
{
	void *vaddr = (void *)vmap((struct page **)pagelist,
					count, VM_MAP, PAGE_KERNEL);
	return vaddr;
}

unsigned long long kcl_map_page(struct pci_dev *pci_dev, unsigned long p)
{
	return pci_map_page(pci_dev, (struct page *)p, 0,
					PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
}

void kcl_mem_unmap(void *addr)
{
	if (addr)
		vunmap(addr);
}

unsigned long long kcl_get_page_ma(unsigned long p)
{
	return (unsigned long long)(page_to_pfn((struct page *)p)<<PAGE_SHIFT);
}

unsigned long long kcl_virt_to_pa(unsigned int *p)
{
	return (unsigned long long)(virt_to_phys((void *)p));
}

void kcl_reserve_page(void *p)
{
	SetPageReserved((struct page *)p);
}

void kcl_unreserve_page(void *p)
{
	ClearPageReserved((struct page *)p);
}

void kcl_get_page(void *p)
{
	get_page(p);
}

void kcl_put_page(void *p)
{
	put_page(p);
}

/*
 * kcl_schedule_work() - put work task in global workqueue
 * @work: job to be done
 *
 * return	0: work was already on the kernel-global workqueue
 * 		non-zero: otherwise.
 *
 * This puts a job in the kernel-global workqueue if it was not already
 * queued and leaves it in the same position on the kernel-global
 * workqueue otherwise.
 */
void kcl_schedule_work(struct work_struct *work)
{
	schedule_work(work);
}

signed long kcl_thread_sleep(int usecs)
{
	uint32_t ms = usecs / 1000;
	uint32_t us = usecs % 1000;

	gim_info("wait %d.%03dms\n", ms, us);
	mdelay(ms);
	udelay(us);
	return 0;
}
