/*
 * Copyright 2017~2018 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef HW_CONTEXT_H_INCLUDED
#define HW_CONTEXT_H_INCLUDED
#include "gru.h"
#include <stdbool.h>

extern int detect_device(void);
extern void show_gpubios(void);
extern void show_gpuinfo(void);
extern void show_gpuvs(void);
extern int open_gpu(char *bus_id);
extern int close_gpu(void);
extern void gpu_status(void);
extern void list_vf(void);
extern void get_vf(char *fb_start, char *fb_size, char *gfx_partition);
extern int release_vf(char *bus_id);
extern int open_vf(char *bus_id);
extern int close_vf(void);
extern char *get_current_gpu(void);
extern char *get_current_vf(void);
extern bool is_gpu_opened(void);
extern bool is_vf_opened(void);
#endif
