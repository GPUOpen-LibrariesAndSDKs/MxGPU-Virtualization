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

#ifndef _GPU_IOV_MODULE__DEBUG_H

#define _GPU_IOV_MODULE__DEBUG_H

#include "gim_adapter.h"

#define GIM_LOG_LEVEL_NONE  0
#define GIM_LOG_LEVEL_ERR  1
#define GIM_LOG_LEVEL_WARN  2
#define GIM_LOG_LEVEL_INFO  3
#define GIM_LOG_LEVEL_DEBUG  4

#define NOTIFY_METHOD_MAILBOX 1
#define NOTIFY_METHOD_MSI     2
#define NOTIFY_METHOD_ALLBITS 3


/* written by driver - the combined size */
#define  REGISTER_RESTORE_LIST_SIZE             0x5f
/* currently need 0x28 locations */
#define  REGISTER_LIST_FORMAT_START             0x60

#define  GFX3D_STATIC_START                  0x60
#define  GFX3D_STATIC_LENGTH                 0x61
#define  GFX3D_DYN_START                     0x62
#define  GFX3D_DYN_LENGTH                    0x63
#define  CMP_STATIC_START                    0x64
#define  CMP_STATIC_LENGTH                   0x65
#define  CMP_DYN_START                       0x66
#define  CMP_DYN_LENGTH                      0x67
#define  CMN_STATIC_START                    0x68
#define  CMN_STATIC_LENGTH                   0x69
#define  CMN_DYN_START                       0x6a
#define  CMN_DYN_LENGTH                      0x6b
#define  VDDGFX_START                        0x6c
#define  VDDGFX_LENGTH                       0x6d
/* current length is 37, save 6 more locations (2 more indirect)
 * 0x60 to 0x8a -> 43 locations
 */
/* currently 9 entries */
#define  REGISTER_LIST_FORMAT_SEPARATE_START     0x8b
#define  VIRT_START                          0x8b
#define  VIRT_LENGTH                         0x8c
/* current length is 9, save 15 more locations (4 more indirect)
 * 0x8b - 0x9a
 */
/* this list is constant */
#define  STARTING_OFFSET_START                0x9b
/* contents are relative to RegisterListFormatStart */
#define  GFX3D_INDIRECT_START                0x9b
#define  CMP_INDIRECT_START                  0x9c
#define  CMN_INDIRECT_START                  0x9d
#define  VDDGFX_INDIRECT_START               0x9e
/* contents are relative to start of virtual list */
#define  VIRT_INDIRECT_START                 0x9f

int gim_get_log_level(void);

void gim_set_log_level(int level);

int gim_log_msg(const char *fmt, ...);

#define gim_info(fmt, s...)	\
	do { if (gim_get_log_level() >= GIM_LOG_LEVEL_INFO) \
		printk(KERN_INFO "gim info:(%s:%d) " fmt, __func__, \
			__LINE__, ##s); } while (0)

#define gim_warn(fmt, s...)	\
	do { if (gim_get_log_level() >= GIM_LOG_LEVEL_WARN) \
		printk(KERN_WARNING "gim warning:(%s:%d) " fmt, __func__, \
			__LINE__, ##s); } while (0)

#define gim_err(fmt, s...)	\
	do { if (gim_get_log_level() >= GIM_LOG_LEVEL_ERR) \
		printk(KERN_ERR "gim error:(%s:%d) " fmt, __func__, \
			__LINE__, ##s); } while (0)

#define gim_dbg(fmt, s...)	\
	do { if (gim_get_log_level() >= GIM_LOG_LEVEL_DEBUG) \
		printk(KERN_INFO "gim debug:(%s:%d) " fmt, __func__, \
			__LINE__, ##s); } while (0)


void gim_init_debug_interface(void);

void gim_exit_debug_interface(void);

void gim_save_register(struct function *func);
bool gim_check_register(struct function *func);
void gim_dump_register(struct function *func);

char *get_int(int *ret, char *in);

#endif
