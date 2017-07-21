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

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/version.h>

struct file *file_open(const char *path, int flags, int rights)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int err = 0;

	oldfs = get_fs();
	set_fs(get_ds());
	filp = filp_open(path, flags, rights);

	set_fs(oldfs);
	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}
	return filp;
}

void file_close(struct file *file)
{
	filp_close(file, NULL);
}

unsigned long long file_size(struct file *file)
{
	mm_segment_t oldfs;
	struct kstat ks;

	oldfs = get_fs();
	set_fs(get_ds());

#if !defined(XEN_DUNDEE) && (KERNEL_VERSION(3, 9, 0) > LINUX_VERSION_CODE)
	/* 3.4.9 */
	vfs_getattr(file->f_vfsmnt, file->f_dentry, &ks);
#else
	/* 3.14.0 + */
	vfs_getattr(&file->f_path, &ks);
#endif
	set_fs(oldfs);

	return ks.size;
}

int file_truncate(struct file *file, unsigned long long size)
{
	int ret = 0;
	struct iattr newattrs;
	mm_segment_t oldfs;

	newattrs.ia_valid = ATTR_SIZE | ATTR_FILE;
	newattrs.ia_file = file;
	newattrs.ia_size = size;

	oldfs = get_fs();
	set_fs(get_ds());

#if KERNEL_VERSION(3, 13, 0) > LINUX_VERSION_CODE
	/* 3.4.9 */
	ret = notify_change(file->f_path.dentry, &newattrs);
#else
	/* to do */
	/* 3.14.0 +
	 * ret = notify_change(file->f_path.dentry, &newattrs, NULL);
	 */
#endif

	set_fs(oldfs);
	return ret;
}

int file_read(struct file *file, unsigned long long offset, unsigned char *data,
		unsigned int size)
{
	mm_segment_t oldfs;
	int ret;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_read(file, data, size, &offset);

	set_fs(oldfs);

	return ret;
}

int file_write(struct file *file, unsigned long long offset,
		unsigned char *data, unsigned int size)
{
	mm_segment_t oldfs;
	int ret;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_write(file, data, size, &offset);

	set_fs(oldfs);

	return ret;
}

int file_sync(struct file *file)
{
	vfs_fsync(file, 0);
	return 0;
}


