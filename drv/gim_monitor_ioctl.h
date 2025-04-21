/*
 * Copyright (c) 2017 Advanced Micro Devices, Inc. All rights reserved.
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

#ifndef _GPU_IOV_MODULE__MONITOR_IOCTL_H
#define _GPU_IOV_MODULE__MONITOR_IOCTL_H

#include <linux/cdev.h>
#include "gim_adapter.h"
#include "gim_monitor.h"

#define AMDGIM_SYSFS_ENABLED
#define AMDGIM_DEVFS_ENABLED

#define GIM_IOCTL_NR(n)                _IOC_NR(n)

struct amdgim_mutex_lock {
	bool locked;
	struct mutex lock_mutex;
	struct TIMEVALTYPE timeout_start;
	spinlock_t atom_lock;
};

struct amdgim_devfs_dev {
	struct cdev cdev;
	int major;
	unsigned char command[AMDGIM_STRLEN_LONG];
	unsigned char result[AMDGIM_BUFFER_SIZE * MAX_ADAPTERS_IN_SYSTEM];
	unsigned int read_offset;
	unsigned int result_len;
};

enum amdgim_ioctl_index {
	AMDGIM_IOCTL_SET_COMMAND,
	AMDGIM_IOCTL_GET_RESULT,
	AMDGIM_IOCTL_GET_GPU_INFO,
	AMDGIM_IOCTL_GET_GPU_VOLATILE_STATE,
	AMDGIM_IOCTL_GET_GPU_VBIOS_INFO,
	AMDGIM_IOCTL_GET_GPU_VF_INFO,
	AMDGIM_IOCTL_GET_VF_DETAIL_INFO,
	AMDGIM_IOCTL_INDEX_LEN
};

typedef int amdgim_ioctl_func_t(struct file *filp,
				unsigned int cmd, unsigned long arg);

struct amdgim_ioctl {
	amdgim_ioctl_func_t         *func;
};

#define IOCTL_FUNC_ENTRY(f)   ((amdgim_ioctl_func_t *)f)

#ifdef AMDGIM_DEVFS_ENABLED
int amdgim_ioctl_set_command(struct file *filp, unsigned int cmd,
				unsigned long arg);
int amdgim_ioctl_get_result(struct file *filp, unsigned int cmd,
				unsigned long arg);
int amdgim_ioctl_get_gpu_info(struct file *filp, unsigned int cmd,
				unsigned long arg);
int amdgim_ioctl_get_gpu_volatile_state(struct file *filp, unsigned int cmd,
				unsigned long arg);
int amdgim_ioctl_get_gpu_vbios_info(struct file *filp, unsigned int cmd,
				unsigned long arg);
int amdgim_ioctl_get_gpu_vf_info(struct file *filp, unsigned int cmd,
				unsigned long arg);
int amdgim_ioctl_get_vf_detail_info(struct file *filp, unsigned int cmd,
				unsigned long arg);
int amdgim_ioctl_get_error_entry(struct file *filp, unsigned int cmd,
				unsigned long arg);

int amdgim_create_cdev(void);
int amdgim_destroy_cdev(void);
#endif

union amdgim_attr {
	struct device_attribute dev_attr;
	struct driver_attribute drv_attr;
};

struct amdgim_sysfs_dev {
	union {
		struct device *pdev;
		struct device_driver *pdrv;
	} node;
	union amdgim_attr attr;
	char *rbuf;
	char *wbuf;
	unsigned int rlen;
	struct function *owner;
};

struct amdgim_sysfs_group {

	struct amdgim_sysfs_dev drv_gpuinfo;
	struct amdgim_sysfs_dev drv_gpuvs;
	struct amdgim_sysfs_dev drv_gpubios;

	struct amdgim_sysfs_dev
		vf_gpuvf[MAX_ADAPTERS_IN_SYSTEM][MAX_VIRTUAL_FUNCTIONS];
	struct amdgim_sysfs_dev
		vf_clrvffb[MAX_ADAPTERS_IN_SYSTEM][MAX_VIRTUAL_FUNCTIONS];
	struct amdgim_sysfs_dev
		vf_flr[MAX_ADAPTERS_IN_SYSTEM][MAX_VIRTUAL_FUNCTIONS];

	struct amdgim_sysfs_dev pf_gpuvf[MAX_ADAPTERS_IN_SYSTEM];
	struct amdgim_sysfs_dev pf_getvf[MAX_ADAPTERS_IN_SYSTEM];
	struct amdgim_sysfs_dev pf_relvf[MAX_ADAPTERS_IN_SYSTEM];
	struct amdgim_sysfs_dev pf_hotlink_reset[MAX_ADAPTERS_IN_SYSTEM];
	struct amdgim_mutex_lock sysfs_mutex;
};

#ifdef AMDGIM_SYSFS_ENABLED
int amdgim_create_sysfs_dev(struct device_driver *pdrv);
int amdgim_destroy_sysfs_dev(void);
#endif

struct amdgim_monitor_driver {
	struct amdgim_devfs_dev devfs_dev;
	struct amdgim_sysfs_group sysfs_group;
	struct mutex core_func_mutex;
};

struct amdgim_monitor_driver *get_monitor_driver(void);
int amdgim_create_devices(struct device_driver *pdrv);
void amdgim_destroy_devices(void);

#endif
