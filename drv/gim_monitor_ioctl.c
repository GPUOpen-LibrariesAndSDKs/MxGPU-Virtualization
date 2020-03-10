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

#include <linux/fs.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/mod_devicetable.h>
#include <linux/ioctl.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include "gim_monitor.h"
#include "gim_monitor_ioctl.h"
#include "gim_debug.h"
#include "gim_timer.h"

static struct amdgim_monitor_driver m_drv;

char *amdgim_commands[AMDGIM_COMMAND_LEN] = {
	[AMDGIM_COMMAND_GPUINFO] = "gpuinfo",
	[AMDGIM_COMMAND_GPUVS] = "gpuvs",
	[AMDGIM_COMMAND_GPUBIOS] = "gpubios",
	[AMDGIM_COMMAND_GPUVF_PF] = "gpuvf",
	[AMDGIM_COMMAND_GPUVF_VF] = "gpuvf",
	[AMDGIM_COMMAND_CLRVFFB] = "clrvffb",
	[AMDGIM_COMMAND_GETVF] = "getvf",
	[AMDGIM_COMMAND_RELVF] = "relvf",
	[AMDGIM_COMMAND_FLR] = "flr",
	[AMDGIM_COMMAND_HOTLINK_RESET] = "hotlink_reset",
};


#define AMDGIM_LOCK_SLEEP_TICK_MS           100
#define AMDGIM_LOCK_TIMEOUT_S               3

void amdgim_mutex_get_lock(struct amdgim_mutex_lock *mutexlock)
{
	struct TIMEVALTYPE curr_t;

	do {
		spin_lock(&mutexlock->atom_lock);
		do_gettimeofday(&curr_t);
		/* if last_time.tv_sec ==0, it would never be timed out */
		if (mutexlock->locked == true &&
			mutexlock->timeout_start.tv_sec > 0 &&
			(curr_t.tv_sec - mutexlock->timeout_start.tv_sec)
			> AMDGIM_LOCK_TIMEOUT_S){
			/* timeout force release the lock */
			mutexlock->timeout_start.tv_sec = 0;
			mutexlock->locked = false;
			mutex_unlock(&mutexlock->lock_mutex);
		}
		/* if the lock had been released */
		if (mutexlock->locked == false) {
			if (mutex_trylock(&mutexlock->lock_mutex)) {
				mutexlock->locked = true;
				mutexlock->timeout_start.tv_sec = 0;
				spin_unlock(&mutexlock->atom_lock);
				break;
			}

			gim_info("try lock failed");
		}
		spin_unlock(&mutexlock->atom_lock);
		msleep(AMDGIM_LOCK_SLEEP_TICK_MS);
	} while (1);
}

/* just setting the time won't unlock the mutex. The mutex will be unlocked
 * only if release is called or the timeout is hit.
 */
void amdgim_mutex_start_timeout(struct amdgim_mutex_lock *mutexlock)
{
	spin_lock(&mutexlock->atom_lock);
	do_gettimeofday(&mutexlock->timeout_start);
	spin_unlock(&mutexlock->atom_lock);
}

void amdgim_mutex_release_lock(struct amdgim_mutex_lock *mutexlock)
{
	spin_lock(&mutexlock->atom_lock);
	if (mutexlock->locked == true) {
		if (mutexlock->timeout_start.tv_sec > 0) {
			mutexlock->timeout_start.tv_sec = 0;
			gim_info("timeout lock released");
		}
		mutexlock->locked = false;
		mutex_unlock(&mutexlock->lock_mutex);
	}
	spin_unlock(&mutexlock->atom_lock);
}




#ifdef AMDGIM_DEVFS_ENABLED

static struct class *amdgim_class;
static struct amdgim_devfs_dev *amdgim_pdev;

static struct amdgim_ioctl amdgim_ioctl_tbl[AMDGIM_IOCTL_INDEX_LEN] = {
	{IOCTL_FUNC_ENTRY(amdgim_ioctl_set_command)},
	{IOCTL_FUNC_ENTRY(amdgim_ioctl_get_result)},
	{IOCTL_FUNC_ENTRY(amdgim_ioctl_get_gpu_info)},
	{IOCTL_FUNC_ENTRY(amdgim_ioctl_get_gpu_volatile_state)},
	{IOCTL_FUNC_ENTRY(amdgim_ioctl_get_gpu_vbios_info)},
	{IOCTL_FUNC_ENTRY(amdgim_ioctl_get_gpu_vf_info)},
	{IOCTL_FUNC_ENTRY(amdgim_ioctl_get_vf_detail_info)},
};


/* IOCTL COMMAND FUNCTIONS */
int amdgim_ioctl_set_command(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	gim_info("ioctl set command");
	return -EINVAL;
}

int amdgim_ioctl_get_result(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	gim_info("ioctl get result");
	return -EINVAL;
}

int amdgim_ioctl_get_gpu_info(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	gim_info("ioctl get gpu info");
	return -EINVAL;
}

int amdgim_ioctl_get_gpu_volatile_state(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	gim_info("ioctl get gpu volatile state");
	return -EINVAL;
}

int amdgim_ioctl_get_gpu_vbios_info(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	gim_info("ioctl get gpu vbios info");

	return -EINVAL;
}

int amdgim_ioctl_get_gpu_vf_info(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	gim_info("ioctl get gpu vf info");

	return -EINVAL;
}

int amdgim_ioctl_get_vf_detail_info(struct file *filpi,
			unsigned int cmd, unsigned long arg)
{
	gim_info("ioctl get vf detail info");

	return -EINVAL;
}

/* SYSTEM INTERFACES */
static int amdgim_open(struct inode *inode, struct file *filp)
{
	gim_info("amdgim monitor Opened.");
	return 0;
}

static int amdgim_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t amdgim_read(struct file *filp, char __user *buf,
				size_t count, loff_t *ppos)
{
	gim_info("amdgim read data.");
	return 0;
}

static ssize_t amdgim_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *ppos)
{
	gim_info("amdgim write command");
	return 0;
}

static loff_t amdgim_llseek(struct file *filp, loff_t offset, int orig)
{
	gim_info("amdgim llseek, not implemented");
	return 0;
}

static long amdgim_ioctls(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	struct amdgim_ioctl *ioctl = NULL;
	unsigned int nr = GIM_IOCTL_NR(cmd);
	amdgim_ioctl_func_t *func = NULL;
	int ret = -EINVAL;

	ioctl = &amdgim_ioctl_tbl[nr];
	func = ioctl->func;

	if (func == NULL) {
		gim_info("no ioctl function");
		return -EINVAL;
	}

	ret = (func)(filp, cmd, arg);
	if (ret) {
		gim_info("ioctl failed, nr = 0x%x", nr);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations amdgim_fops = {
	.owner = THIS_MODULE,
	.open = amdgim_open,
	.release = amdgim_release,
	.llseek = amdgim_llseek,
	.read = amdgim_read,
	.write = amdgim_write,
	.unlocked_ioctl = amdgim_ioctls
};



int amdgim_create_cdev(void)
{
	int ret;
	dev_t devno;

	amdgim_pdev = NULL;

	amdgim_pdev = kzalloc(sizeof(struct amdgim_devfs_dev), GFP_KERNEL);
	if (!amdgim_pdev) {
		ret = -ENOMEM;
		goto fail_malloc;
	}

	amdgim_pdev->major = AMDGIM_MAJOR;
	devno = MKDEV(amdgim_pdev->major, AMDGIM_MINOR_START);

	if (amdgim_pdev->major) {
		ret = register_chrdev_region(devno,
					AMDGIM_MINOR_LEN, AMDGIM_CDEV_NAME);
	} else{
		ret = alloc_chrdev_region(&devno, AMDGIM_MINOR_START,
					AMDGIM_MINOR_LEN, AMDGIM_CDEV_NAME);
		amdgim_pdev->major = MAJOR(devno);
	}

	if (ret < 0) {
		gim_err("Can't allocate a devno");
		goto fail_get_devno;
	}

	cdev_init(&amdgim_pdev->cdev, &amdgim_fops);
	amdgim_pdev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&amdgim_pdev->cdev, devno, 1);

	if (ret) {
		gim_err("Error %d while adding amdgim", ret);
		goto fail_cdevadd;
	}

	amdgim_class = class_create(THIS_MODULE, AMDGIM_CLASS_NAME);
	device_create(amdgim_class, NULL, MKDEV(amdgim_pdev->major, 0),
			NULL, AMDGIM_DEVICE_FILE_NAME);

	return 0;

fail_cdevadd:
	unregister_chrdev_region(amdgim_pdev->major, 1);
fail_get_devno:
	kfree(amdgim_pdev);
	amdgim_pdev = NULL;
fail_malloc:
	return ret;
}

int amdgim_destroy_cdev(void)
{
	if (amdgim_pdev != NULL) {
		cdev_del(&(amdgim_pdev->cdev));
		unregister_chrdev_region(MKDEV(amdgim_pdev->major, 0),
					AMDGIM_MINOR_LEN);
		device_destroy(amdgim_class, MKDEV(amdgim_pdev->major, 0));
		class_destroy(amdgim_class);
		kfree(amdgim_pdev);
		amdgim_pdev = NULL;
	}
	return 0;
}

#endif

/**************************SYSFS******************************/

#ifdef AMDGIM_SYSFS_ENABLED

static int amdgim_sysfs_run_command(struct amdgim_sysfs_dev *pdev, int index)
{
	pdev->rbuf[0] = 0;
	if (amdgim_do_op(pdev->wbuf, (void *)pdev->owner, index, pdev->rbuf))
		return -EINVAL;

	pdev->rlen = strlen(pdev->rbuf);

	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

static int amdgim_sysfs_read_result(struct amdgim_sysfs_dev *pdev, char *buf,
				loff_t pos, size_t count)
{
	unsigned int datalen = 0;

	datalen = ((pdev->rlen - pos) > count) ? count : (pdev->rlen - pos);
	if (datalen > 0)
		strncpy(buf, pdev->rbuf + pos, datalen);

	return datalen;
}

static ssize_t amdgim_sysfs_drv_gpuinfo_show(struct device_driver *drv,
					char *buf)
{
	int ret = 0;
	struct amdgim_sysfs_dev *pdev;

	pdev = &(get_monitor_driver()->sysfs_group.drv_gpuvs);

	amdgim_mutex_get_lock(&m_drv.sysfs_group.sysfs_mutex);

	amdgim_sysfs_run_command(pdev, AMDGIM_COMMAND_GPUINFO);
	ret = amdgim_sysfs_read_result(pdev, buf, 0, AMDGIM_BUFFER_SIZE);
	amdgim_mutex_release_lock(&m_drv.sysfs_group.sysfs_mutex);
	return ret;
}

static ssize_t amdgim_sysfs_drv_gpuvs_show(struct device_driver *drv, char *buf)
{
	int ret = 0;
	struct amdgim_sysfs_dev *pdev;

	pdev = &(get_monitor_driver()->sysfs_group.drv_gpuvs);

	amdgim_mutex_get_lock(&m_drv.sysfs_group.sysfs_mutex);

	amdgim_sysfs_run_command(pdev, AMDGIM_COMMAND_GPUVS);
	ret = amdgim_sysfs_read_result(pdev, buf, 0, AMDGIM_BUFFER_SIZE);
	amdgim_mutex_release_lock(&m_drv.sysfs_group.sysfs_mutex);
	return ret;
}

static ssize_t amdgim_sysfs_drv_gpubios_show(struct device_driver *drv,
						char *buf)
{
	int ret = 0;
	struct amdgim_sysfs_dev *pdev;

	pdev = &(get_monitor_driver()->sysfs_group.drv_gpubios);

	amdgim_mutex_get_lock(&m_drv.sysfs_group.sysfs_mutex);

	amdgim_sysfs_run_command(pdev, AMDGIM_COMMAND_GPUBIOS);
	ret = amdgim_sysfs_read_result(pdev, buf, 0, AMDGIM_BUFFER_SIZE);
	amdgim_mutex_release_lock(&m_drv.sysfs_group.sysfs_mutex);
	return ret;
}

static ssize_t amdgim_sysfs_pf_gpuvf_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int ret = 0;
	struct amdgim_sysfs_dev *pdev;

	pdev = container_of((union amdgim_attr *)attr,
			struct amdgim_sysfs_dev, attr);
	amdgim_mutex_get_lock(&m_drv.sysfs_group.sysfs_mutex);

	amdgim_sysfs_run_command(pdev, AMDGIM_COMMAND_GPUVF_PF);
	ret = amdgim_sysfs_read_result(pdev, buf, 0, AMDGIM_BUFFER_SIZE);
	amdgim_mutex_release_lock(&m_drv.sysfs_group.sysfs_mutex);
	return ret;
}

static ssize_t amdgim_sysfs_vf_gpuvf_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret = 0;
	struct amdgim_sysfs_dev *pdev;

	pdev = container_of((union amdgim_attr *)attr,
			struct amdgim_sysfs_dev, attr);

	amdgim_mutex_get_lock(&m_drv.sysfs_group.sysfs_mutex);

	amdgim_sysfs_run_command(pdev, AMDGIM_COMMAND_GPUVF_VF);
	ret = amdgim_sysfs_read_result(pdev, buf, 0, AMDGIM_BUFFER_SIZE);
	amdgim_mutex_release_lock(&m_drv.sysfs_group.sysfs_mutex);
	return ret;
}

static ssize_t amdgim_sysfs_vf_clrvffb_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct amdgim_sysfs_dev *pdev = container_of((union amdgim_attr *)attr,
		struct amdgim_sysfs_dev, attr);
	return amdgim_sysfs_read_result(pdev, buf, 0, AMDGIM_BUFFER_SIZE);
}

static ssize_t amdgim_sysfs_vf_clrvffb_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct amdgim_sysfs_dev *pdev = container_of((union amdgim_attr *)attr,
		struct amdgim_sysfs_dev, attr);
	/* only allow one clrvffb running at the same time*/
	amdgim_mutex_get_lock(&m_drv.sysfs_group.sysfs_mutex);
	strncpy(pdev->wbuf, buf, AMDGIM_STRLEN_NORMAL);
	amdgim_sysfs_run_command(pdev, AMDGIM_COMMAND_CLRVFFB);
	amdgim_mutex_release_lock(&m_drv.sysfs_group.sysfs_mutex);
	return count;
}

static ssize_t amdgim_sysfs_vf_flr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	struct amdgim_sysfs_dev *pdev = container_of((union amdgim_attr *)attr,
		struct amdgim_sysfs_dev, attr);
	ret = amdgim_sysfs_read_result(pdev, buf, 0, AMDGIM_BUFFER_SIZE);
	amdgim_mutex_release_lock(&m_drv.sysfs_group.sysfs_mutex);
	return ret;
}

static ssize_t amdgim_sysfs_vf_flr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct amdgim_sysfs_dev *pdev = container_of((union amdgim_attr *)attr,
		struct amdgim_sysfs_dev, attr);
	amdgim_mutex_get_lock(&m_drv.sysfs_group.sysfs_mutex);
	strncpy(pdev->wbuf, buf, AMDGIM_STRLEN_NORMAL);
	amdgim_sysfs_run_command(pdev, AMDGIM_COMMAND_FLR);
	amdgim_mutex_start_timeout(&m_drv.sysfs_group.sysfs_mutex);
	return count;
}

static ssize_t amdgim_sysfs_pf_hotlink_reset_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct amdgim_sysfs_dev *pdev = container_of((union amdgim_attr *)attr,
		struct amdgim_sysfs_dev, attr);
	ret = amdgim_sysfs_read_result(pdev, buf, 0, AMDGIM_BUFFER_SIZE);
	amdgim_mutex_release_lock(&m_drv.sysfs_group.sysfs_mutex);
	return ret;
}

static ssize_t amdgim_sysfs_pf_hotlink_reset_store(
	struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	struct amdgim_sysfs_dev *pdev = container_of((union amdgim_attr *)attr,
		struct amdgim_sysfs_dev, attr);
	amdgim_mutex_get_lock(&m_drv.sysfs_group.sysfs_mutex);
	strncpy(pdev->wbuf, buf, AMDGIM_STRLEN_NORMAL);
	amdgim_sysfs_run_command(pdev, AMDGIM_COMMAND_HOTLINK_RESET);
	amdgim_mutex_start_timeout(&m_drv.sysfs_group.sysfs_mutex);
	return count;
}

static ssize_t amdgim_sysfs_pf_getvf_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	struct amdgim_sysfs_dev *pdev = container_of((union amdgim_attr *)attr,
		struct amdgim_sysfs_dev, attr);
	ret = amdgim_sysfs_read_result(pdev, buf, 0, AMDGIM_BUFFER_SIZE);
	amdgim_mutex_release_lock(&m_drv.sysfs_group.sysfs_mutex);
	return ret;
}

static ssize_t amdgim_sysfs_pf_getvf_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct amdgim_sysfs_dev *pdev = container_of((union amdgim_attr *)attr,
		struct amdgim_sysfs_dev, attr);
	amdgim_mutex_get_lock(&m_drv.sysfs_group.sysfs_mutex);
	strncpy(pdev->wbuf, buf, AMDGIM_STRLEN_NORMAL);
	amdgim_sysfs_run_command(pdev, AMDGIM_COMMAND_GETVF);
	amdgim_mutex_start_timeout(&m_drv.sysfs_group.sysfs_mutex);
	return count;
}

static ssize_t amdgim_sysfs_pf_relvf_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	struct amdgim_sysfs_dev *pdev = container_of((union amdgim_attr *)attr,
		struct amdgim_sysfs_dev, attr);
	ret = amdgim_sysfs_read_result(pdev, buf, 0, AMDGIM_BUFFER_SIZE);
	amdgim_mutex_release_lock(&m_drv.sysfs_group.sysfs_mutex);
	return ret;
}

static ssize_t amdgim_sysfs_pf_relvf_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct amdgim_sysfs_dev *pdev = container_of((union amdgim_attr *)attr,
		struct amdgim_sysfs_dev, attr);
	amdgim_mutex_get_lock(&m_drv.sysfs_group.sysfs_mutex);
	strncpy(pdev->wbuf, buf, AMDGIM_STRLEN_NORMAL);
	amdgim_sysfs_run_command(pdev, AMDGIM_COMMAND_RELVF);
	amdgim_mutex_start_timeout(&m_drv.sysfs_group.sysfs_mutex);
	return count;
}

int amdgim_init_sysfs_file(struct amdgim_sysfs_dev *sysfs_dev,
		unsigned int cmd, unsigned int mode,
		void *show, void *store,
		unsigned int rbsize, unsigned int wbsize,
		void *node, void *owner)
{
	int ret = 0;

	sysfs_dev->rbuf = (rbsize != 0) ? vmalloc(rbsize) : NULL;
	sysfs_dev->wbuf = (wbsize != 0) ? vmalloc(wbsize) : NULL;
	sysfs_dev->owner = owner;

	switch (cmd) {
	case AMDGIM_COMMAND_GPUINFO:
	case AMDGIM_COMMAND_GPUVS:
	case AMDGIM_COMMAND_GPUBIOS:
		sysfs_dev->attr.drv_attr.attr.name = amdgim_commands[cmd];
		sysfs_dev->attr.drv_attr.attr.mode = mode;
		sysfs_dev->attr.drv_attr.show = show;
		sysfs_dev->attr.drv_attr.store = store;
		sysfs_dev->node.pdrv = node;
		ret = driver_create_file(sysfs_dev->node.pdrv,
					&sysfs_dev->attr.drv_attr);
		if (ret != 0)
			sysfs_dev->node.pdrv = NULL;
		break;
	case AMDGIM_COMMAND_GPUVF_PF:
	case AMDGIM_COMMAND_GPUVF_VF:
	case AMDGIM_COMMAND_CLRVFFB:
	case AMDGIM_COMMAND_GETVF:
	case AMDGIM_COMMAND_RELVF:
	case AMDGIM_COMMAND_FLR:
	case AMDGIM_COMMAND_HOTLINK_RESET:
		sysfs_dev->attr.dev_attr.attr.name = amdgim_commands[cmd];
		sysfs_dev->attr.dev_attr.attr.mode = mode;
		sysfs_dev->attr.dev_attr.show = show;
		sysfs_dev->attr.dev_attr.store = store;
		sysfs_dev->node.pdev = node;
		ret = device_create_file(sysfs_dev->node.pdev,
					&sysfs_dev->attr.dev_attr);
		if (ret != 0)
			sysfs_dev->node.pdev = NULL;
		break;
	default:
		break;
	}
	return ret;
}

void amdgim_deinit_sysfs_file(struct amdgim_sysfs_dev *sysfs_dev,
		unsigned int cmd)
{
	if (sysfs_dev->rbuf != NULL)
		vfree(sysfs_dev->rbuf);
	if (sysfs_dev->wbuf != NULL)
		vfree(sysfs_dev->wbuf);

	switch (cmd) {
	case AMDGIM_COMMAND_GPUINFO:
	case AMDGIM_COMMAND_GPUVS:
	case AMDGIM_COMMAND_GPUBIOS:
		if (sysfs_dev->node.pdrv != NULL) {
			driver_remove_file(sysfs_dev->node.pdrv,
				&sysfs_dev->attr.drv_attr);
		}
		break;
	case AMDGIM_COMMAND_GPUVF_PF:
	case AMDGIM_COMMAND_GPUVF_VF:
	case AMDGIM_COMMAND_CLRVFFB:
	case AMDGIM_COMMAND_GETVF:
	case AMDGIM_COMMAND_RELVF:
	case AMDGIM_COMMAND_FLR:
	case AMDGIM_COMMAND_HOTLINK_RESET:
		if (sysfs_dev->node.pdev != NULL) {
			device_remove_file(sysfs_dev->node.pdev,
				&sysfs_dev->attr.dev_attr);
		}
		break;
	default:
		break;
	}
}

int amdgim_create_sysfs_dev(struct device_driver *pdrv)
{
	struct adapter *adapters = get_adapters();
	struct adapter *p_adapter;
	struct function *p_func;
	int adapter_count = get_adapter_count();
	int i;
	int j;
	struct amdgim_sysfs_group *sysfs_dev = &m_drv.sysfs_group;

	memset(sysfs_dev, 0, sizeof(struct amdgim_sysfs_group));

	/* create gpuinfo */
	amdgim_init_sysfs_file(&sysfs_dev->drv_gpuinfo,
			AMDGIM_COMMAND_GPUINFO,
			0444, amdgim_sysfs_drv_gpuinfo_show, NULL,
			AMDGIM_BUFFER_SIZE, 0,
			pdrv, NULL);
	/* create gpuvs */
	amdgim_init_sysfs_file(&sysfs_dev->drv_gpuvs,
			AMDGIM_COMMAND_GPUVS,
			0444, amdgim_sysfs_drv_gpuvs_show, NULL,
			AMDGIM_BUFFER_SIZE, 0,
			pdrv, NULL);
	/* create gpubios */
	amdgim_init_sysfs_file(&sysfs_dev->drv_gpubios,
			AMDGIM_COMMAND_GPUBIOS,
			0444, amdgim_sysfs_drv_gpubios_show, NULL,
			AMDGIM_BUFFER_SIZE, 0,
			pdrv, NULL);

	for (i = 0; i < adapter_count; ++i) {
		p_adapter = adapters + i;
		/* create getvf */
		amdgim_init_sysfs_file(&sysfs_dev->pf_getvf[i],
			AMDGIM_COMMAND_GETVF,
			0644,
			amdgim_sysfs_pf_getvf_show,
			amdgim_sysfs_pf_getvf_store,
			AMDGIM_GPU_ERROR_MSG_SIZE,
			AMDGIM_STRLEN_LONG,
			&p_adapter->pf.pci_dev->dev,
			&p_adapter->pf);
		/* create relvf */
		amdgim_init_sysfs_file(&sysfs_dev->pf_relvf[i],
			AMDGIM_COMMAND_RELVF,
			0644,
			amdgim_sysfs_pf_relvf_show,
			amdgim_sysfs_pf_relvf_store,
			AMDGIM_GPU_ERROR_MSG_SIZE,
			AMDGIM_STRLEN_LONG,
			&p_adapter->pf.pci_dev->dev,
			&p_adapter->pf);
		/* create gpuvf on pf */
		amdgim_init_sysfs_file(&sysfs_dev->pf_gpuvf[i],
			AMDGIM_COMMAND_GPUVF_PF,
			0444,
			amdgim_sysfs_pf_gpuvf_show,
			NULL,
			AMDGIM_BUFFER_SIZE,
			AMDGIM_STRLEN_LONG,
			&p_adapter->pf.pci_dev->dev,
			&p_adapter->pf);
		/* create hotlink_reset */
		amdgim_init_sysfs_file(&sysfs_dev->pf_hotlink_reset[i],
			AMDGIM_COMMAND_HOTLINK_RESET,
			0644,
			amdgim_sysfs_pf_hotlink_reset_show,
			amdgim_sysfs_pf_hotlink_reset_store,
			AMDGIM_GPU_ERROR_MSG_SIZE,
			AMDGIM_STRLEN_LONG,
			&p_adapter->pf.pci_dev->dev,
			&p_adapter->pf);

		for (j = 0; j < p_adapter->enabled_vfs; ++j) {
			p_func = &p_adapter->vfs[j];
			/* create gpuvf on vf */
			amdgim_init_sysfs_file(&sysfs_dev->vf_gpuvf[i][j],
				AMDGIM_COMMAND_GPUVF_VF,
				0444,
				amdgim_sysfs_vf_gpuvf_show,
				NULL,
				AMDGIM_BUFFER_SIZE,
				0,
				&p_func->pci_dev->dev, p_func);

			/* create clrvffb */
			amdgim_init_sysfs_file(&sysfs_dev->vf_clrvffb[i][j],
				AMDGIM_COMMAND_CLRVFFB,
				0644,
				amdgim_sysfs_vf_clrvffb_show,
				amdgim_sysfs_vf_clrvffb_store,
				AMDGIM_GPU_ERROR_MSG_SIZE,
				AMDGIM_STRLEN_LONG,
				&p_func->pci_dev->dev, p_func);
			/* create flr */
			amdgim_init_sysfs_file(&sysfs_dev->vf_flr[i][j],
				AMDGIM_COMMAND_FLR,
				0644,
				amdgim_sysfs_vf_flr_show,
				amdgim_sysfs_vf_flr_store,
				AMDGIM_GPU_ERROR_MSG_SIZE,
				AMDGIM_STRLEN_LONG,
				&p_func->pci_dev->dev, p_func);
		}
	}
	mutex_init(&sysfs_dev->sysfs_mutex.lock_mutex);

	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

int amdgim_destroy_sysfs_dev(void)
{
	struct adapter *adapters = get_adapters();
	struct adapter *p_adapter;
	struct function *p_func;
	int adapter_count = get_adapter_count();
	int i;
	int j;
	struct amdgim_sysfs_group *sysfs_dev = &m_drv.sysfs_group;

	for (i = 0; i < adapter_count; ++i) {
		p_adapter = adapters + i;

		for (j = 0; j < p_adapter->enabled_vfs; ++j) {
			p_func = &p_adapter->vfs[j];

			amdgim_deinit_sysfs_file(&sysfs_dev->vf_gpuvf[i][j],
				AMDGIM_COMMAND_GPUVF_VF);

			amdgim_deinit_sysfs_file(&sysfs_dev->vf_clrvffb[i][j],
				AMDGIM_COMMAND_CLRVFFB);

			amdgim_deinit_sysfs_file(&sysfs_dev->vf_flr[i][j],
				AMDGIM_COMMAND_FLR);
		}

		amdgim_deinit_sysfs_file(&sysfs_dev->pf_getvf[i],
			AMDGIM_COMMAND_GETVF);

		amdgim_deinit_sysfs_file(&sysfs_dev->pf_relvf[i],
			AMDGIM_COMMAND_RELVF);

		amdgim_deinit_sysfs_file(&sysfs_dev->pf_gpuvf[i],
			AMDGIM_COMMAND_GPUVF_PF);

		amdgim_deinit_sysfs_file(&sysfs_dev->pf_hotlink_reset[i],
			AMDGIM_COMMAND_HOTLINK_RESET);
	}

	amdgim_deinit_sysfs_file(&sysfs_dev->drv_gpuinfo,
			AMDGIM_COMMAND_GPUINFO);

	amdgim_deinit_sysfs_file(&sysfs_dev->drv_gpuvs,
			AMDGIM_COMMAND_GPUVS);

	amdgim_deinit_sysfs_file(&sysfs_dev->drv_gpubios,
			AMDGIM_COMMAND_GPUBIOS);

	mutex_destroy(&sysfs_dev->sysfs_mutex.lock_mutex);

	return AMDGIM_ERROR_MONITOR_SUCCESS;
}

#endif

int amdgim_create_devices(struct device_driver *pdrv)
{
	int ret;

	mutex_init(&m_drv.core_func_mutex);

	if (pdrv != NULL) {
		ret = amdgim_create_sysfs_dev(pdrv);
		if (ret)
			goto failed;
	} else{
		ret = AMDGIM_ERROR_MONITOR_UNKNOWN;
		goto failed;
	}

	ret = amdgim_create_cdev();

failed:
	return ret;
}

void amdgim_destroy_devices(void)
{
	amdgim_destroy_sysfs_dev();
	amdgim_destroy_cdev();
	mutex_destroy(&m_drv.core_func_mutex);
}

struct amdgim_monitor_driver *get_monitor_driver()
{
	return &m_drv;
}

