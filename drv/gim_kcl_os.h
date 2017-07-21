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

#ifndef _GPU_IOV_MODULE__KCL_OS_H
#define _GPU_IOV_MODULE__KCL_OS_H

#include <linux/sched.h>

#include "gim_kcl_type.h"

#define PAGE_CNT_UP(x)  (((x) + (PAGE_SIZE - 1)) / PAGE_SIZE)

void kcl_memset(void *s, int c, kcl_type_u64 count);
void *kcl_mem_small_buffer_alloc(kcl_type_u32 size);
void kcl_mem_small_buffer_free(void *p);
void *kcl_mem_alloc_page(void);
void kcl_mem_free_page(void *p);
void *kcl_mem_alloc_page_size(unsigned int page_count);
void *kcl_mem_map_page_list(unsigned long *pagelist, unsigned int count);
unsigned long long kcl_map_page(struct pci_dev *pci_dev, unsigned long p);
void kcl_mem_unmap(void *addr);
unsigned long long kcl_get_page_ma(unsigned long p);
unsigned long long kcl_virt_to_pa(unsigned int *p);
void kcl_reserve_page(void *p);
void kcl_unreserve_page(void *p);
void kcl_get_page(void *p);
void kcl_put_page(void *p);
void kcl_schedule_work(struct work_struct *work);
signed long kcl_thread_sleep(int msecs);

#endif
