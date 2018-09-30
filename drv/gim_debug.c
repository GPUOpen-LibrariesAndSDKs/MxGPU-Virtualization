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

#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include "gim_debug.h"
#include "gim_interface.h"
#include "gim_os_service.h"
#include "gim_s7150_reg.h"

#define DEVICE_NAME "sriov_dev"
#define BUF_SIZE    1024

static int g_notify_method = NOTIFY_METHOD_MAILBOX;
static int g_gim_log_level = 3;

static int dev_open(struct inode *inode, struct file *fp);
static int dev_close(struct inode *inode, struct file *fp);
static ssize_t dev_read(struct file *fp, char *buf,
			size_t count, loff_t *f_pos);
static ssize_t dev_write(struct file *fp, const char *buf,
		  size_t count, loff_t *f_pos);

static int Major;
static char dev_cmd_buffer[BUF_SIZE+1];
static char dev_result_buffer[BUF_SIZE+1];
static int bytes_in_buffer;
static int read_ptr;

static int cmd_alloc(char *parms);
static int cmd_free(char *parms);
static int cmd_stop(char *parms);
static int cmd_start(char *parms);
static int cmd_switch(char *parms);
static int cmd_enable_preemption(char *parms);
static int cmd_signal(char *parms);
static int cmd_self_switch(char *parms);
static int cmd_log_level(char *parms);
static int cmd_dump_reg(char *parms);
static int cmd_notify_method(char *parms);

struct cmd_entry {
	char *cmd_name;
	int     (*cmd_fn)(char *parms);
	char *syntax;
};

static const struct file_operations dev_ops = {
	.read = dev_read,
	.write = dev_write,
	.open = dev_open,
	.release = dev_close
};

static struct cmd_entry cmd_table[] = {

	{"alloc_vf", cmd_alloc, "alloc vf-b.d.f fb_size(0x) - alloc vf\n"},
	{"free_vf", cmd_free, "free vf-b.d.f - free vf\n"},
	{"stop_sched", cmd_stop, "stop_sched pf stop GIM scheduler\n"},
	{"start_sched", cmd_start, "start_sched pf - start GIM scheduler\n"},
	{"switch", cmd_switch, "start_sched pf - start GIM scheduler\n"},
	{"preemption", cmd_enable_preemption, "enable mid-buff preemption\n"},
	{"signal", cmd_signal, "send signal to vm\n"},
	{"self_switch", cmd_self_switch, "world switch to the same func\n"},
	{"log_level", cmd_log_level, "set the gim log level\n"},
	{"dump_reg", cmd_dump_reg, "dump the saved register for vf\n"},
	{"notify", cmd_notify_method, "Set notification method: \t"
				      "1=MAILBOX, 2=MSI\n"},
	{NULL, NULL, NULL}
};

static int dev_open(struct inode *inode, struct file *fp)
{
	read_ptr = 0;
	return 0;
}

static int dev_close(struct inode *inode, struct file *fp)
{
	return 0;
}

static ssize_t dev_read(struct file *fp, char *buf, size_t count, loff_t *f_pos)
{
	int copy_result;
	/*min (bytes_in_buffer-read_ptr, count)*/
	int bytes_to_return;

	bytes_to_return = count;
	if ((bytes_in_buffer-read_ptr) < count)
		bytes_to_return = bytes_in_buffer-read_ptr;

	if (bytes_to_return < 1) {
		read_ptr = 0;
		return 0;
	}

	/* Transfer from kernel to user space */
	copy_result = copy_to_user(buf,
				   dev_result_buffer+read_ptr,
				   bytes_to_return);

	read_ptr += bytes_to_return;
	*f_pos = read_ptr;

	return bytes_to_return;
}

void clear(void)
{
	read_ptr = 0;
	bytes_in_buffer = 0;
	dev_result_buffer[0] = '\0';
}

#define MAX_TOKEN 50
static ssize_t dev_write(struct file *fp, const char *buf,
			 size_t count, loff_t *f_pos)
{
	int copy_result;
	int start;
	int cmd_idx = 0;

	char    *buf_ptr;
	char    *cmd;

	if (count > BUF_SIZE)
		return -1;

	copy_result = copy_from_user(dev_cmd_buffer, buf, count);
	if (copy_result)
		return -1;

	/* Ensure that the string is NULL terminated */
	dev_cmd_buffer[count] = '\0';

	/* Find start of first non-white space */
	start = strspn(dev_cmd_buffer, "\n");

	buf_ptr = &dev_cmd_buffer[start];
	cmd = strsep(&buf_ptr, "\n");

	clear();

	while (cmd_table[cmd_idx].cmd_name != NULL) {
		if (!strcmp(cmd_table[cmd_idx].cmd_name, cmd)) {
			cmd_table[cmd_idx].cmd_fn(buf_ptr);
			break;
		}
		++cmd_idx;
	}

	return count;
}

/* Assume NULL terminated strings */
/* Assume input is always in hex */
char *get_int(int *ret, char *in)
{
	char num_set[] = "1234567890ABCDEFabcdef";

	*ret = 0;
	/* Skip non-numeric characters */
	while (!strspn(in, num_set) && *in != '\0')
		++in;

	if (*in == '\0')
		return NULL;

	/* Convert ascii to int */
	while (strspn(in, num_set)) {
		*ret *= 16;
		if (*in >= '0' && *in <= '9')
			*ret += (*in - '0');
		else if (*in >= 'A' && *in <= 'F')
			*ret += (*in - 'A' + 10);
		else
			*ret += (*in - 'a' + 10);
		++in;
	}

	return in;
}

char *cmd_get_bdf(int *bdf, char *in)
{
	char *ptr;
	int b, d, f;

	ptr = in;
	ptr = get_int(&b, ptr);
	if (ptr == NULL)
		return NULL;

	ptr = get_int(&d, ptr);
	if (ptr == NULL)
		return NULL;

	ptr = get_int(&f, ptr);
	if (ptr == NULL)
		return NULL;

	*bdf = (b << 8) | (d << 3) | f;
	return ptr;
}

static int cmd_alloc(char *parms)
{
	int fb_size = 0;
	int vf_bdf = 0;
	char *ptr = parms;

	ptr = cmd_get_bdf(&vf_bdf, ptr);
	ptr = get_int(&fb_size, ptr);

	gim_info("alloc a new vf, vf_bdf = 0x%04x, fb_size = %d\n",
		 vf_bdf, fb_size);

	/* invalid domid and pid for testing */
	alloc_vf_from_bdf(vf_bdf, -1, -1, fb_size, -1);
	return 0;
}

static int cmd_free(char *parms)
{
	int vfBDF = 0;
	char *ptr = parms;

	ptr = cmd_get_bdf(&vfBDF, ptr);
	gim_info("free vf 0x%04x\n", vfBDF);


	/* invalid domid and pid for testing */
	free_vf_from_bdf(vfBDF);
	return 0;
}

static int cmd_stop(char *parms)
{
	int BDF = 0;
	char *ptr = parms;

	ptr = cmd_get_bdf(&BDF, ptr);
	gim_info("stop gpu world switch scheduler\n");

	pause_sched(BDF);

	return 0;
}


static int cmd_start(char *parms)
{
	int BDF = 0;

	gim_info("start gpu world switch scheduler\n");

	start_scheduler(BDF);
	return 0;
}


static int cmd_enable_preemption(char *parms)
{
	int BDF = 0;
	int enable = 0;
	char *ptr = parms;

	ptr = get_int(&enable, ptr);
	enable_preemption(BDF, enable);
	return 0;
}

static int cmd_signal(char *parms)
{
	int vm_id = 0;
	int active = 0;
	char *ptr = parms;


	ptr = get_int(&vm_id, ptr);
	ptr = get_int(&active, ptr);
	gim_info("send %s signal to vm %x\n",
		 active ? "active":"inactive",
		 vm_id);

	send_signal(vm_id, active ? SIG_VF_ACTIVE:SIG_VF_STOP_MMR);
	return 0;
}

static int cmd_switch(char *parms)
{
	int BDF = 0;

	gim_info("trigger a single world switch to gim\n");

	trgger_single_switch(BDF);
	return 0;
}


static int cmd_self_switch(char *parms)
{
	int BDF = 0;
	int switch_to_itself = 0;
	char *ptr = parms;


	ptr = get_int(&switch_to_itself, ptr);

	gim_info("change the world switch method to :%s\n",
		  switch_to_itself ?
		  "switch to the same function every time"
		  : "switch to the next function");

	switch_to_self(BDF, switch_to_itself);
	return 0;
}


static int cmd_log_level(char *parms)
{
	int log_level = 0;
	char *ptr = parms;


	ptr = get_int(&log_level, ptr);

	gim_set_log_level(log_level);
	return 0;
}


static int cmd_notify_method(char *parms)
{
	int method = 0;
	char *ptr = parms;

	ptr = get_int(&method, ptr);
	if (method == 2) {
		g_notify_method = NOTIFY_METHOD_MSI;
		gim_info("Reset Notification Method set to \"MSI\"\n");
	} else {
		g_notify_method = NOTIFY_METHOD_MAILBOX;
		gim_info("Reset Notification Method set to \"MAILBOX\"\n");
	}

	return 0;
}

static int cmd_dump_reg(char *parms)
{
	int vf_id = 0;
	char *ptr = parms;
	struct adapter *adapt = get_default_adapter();
	struct function *pfun = get_vf(adapt, vf_id);


	ptr = get_int(&vf_id, ptr);
	gim_info("cmd_dump_reg for vf id %x\n", vf_id);

	pfun = get_vf(adapt, vf_id);
	gim_info("cmd_dump_reg for pfun= %p\n", pfun);
	if (pfun != NULL)
		gim_dump_register(pfun);

	return 0;
}

void gim_init_debug_interface(void)
{
	Major = register_chrdev(0, DEVICE_NAME, &dev_ops);

	if (Major < 0)
		gim_info("Registering the character device failed with %d\n",
			  Major);

	memset(dev_cmd_buffer, 0, BUF_SIZE);
}

void gim_exit_debug_interface(void)
{
	unregister_chrdev(Major, DEVICE_NAME);
}


void save_register(struct function *func, int start_offset, int length_offset)
{
	int start, length;
	int i = 0;
	int reg_offset = 0;
	unsigned int srm_cntl = 0;

	/* put this list in a recognizable location */
	write_register(func, mmRLC_GPM_SCRATCH_ADDR, start_offset);

	start =  read_register(func, mmRLC_GPM_SCRATCH_DATA);
	gim_info("save_register start =  %d\n", start);

	write_register(func, mmRLC_GPM_SCRATCH_ADDR, length_offset);
	length =  read_register(func, mmRLC_GPM_SCRATCH_DATA);
	gim_info("save_register length = %d\n", length);

	/* enable auto increment in case it's disabled */
	srm_cntl = read_register(func, mmRLC_SRM_CNTL);
	srm_cntl |= RLC_SRM_CNTL__AUTO_INCR_ADDR_MASK;
	write_register(func, mmRLC_SRM_CNTL, srm_cntl);

	write_register(func, mmRLC_SRM_ARAM_ADDR, start);

	for (i = 0; i < length / 2; ++i) {
		reg_offset = read_register(func, mmRLC_SRM_ARAM_DATA);
		reg_offset &= 0x3ffff; /* bits 0:17 are the reg offset */
		func->reg_sav_res_offset[start/2+i] = reg_offset;
		func->reg_sav_res_data[start/2+i] =
					read_register(func, reg_offset);
		reg_offset = read_register(func, mmRLC_SRM_ARAM_DATA);
	}
}


bool check_register(struct function *func, int start_offset, int length_offset)
{
	int start, length;
	int i = 0;
	bool equal = true;
	int reg_offset = 0;
	int reg_data = 0;
	unsigned int srm_cntl = 0;

	/* put this list in a recognizable location */
	write_register(func, mmRLC_GPM_SCRATCH_ADDR, start_offset);
	start =  read_register(func, mmRLC_GPM_SCRATCH_DATA);
	write_register(func, mmRLC_GPM_SCRATCH_ADDR, length_offset);
	length =  read_register(func, mmRLC_GPM_SCRATCH_DATA);

	/* enable auto increment in case it's disabled */
	srm_cntl = read_register(func, mmRLC_SRM_CNTL);
	srm_cntl |= RLC_SRM_CNTL__AUTO_INCR_ADDR_MASK;
	write_register(func, mmRLC_SRM_CNTL, srm_cntl);

	write_register(func, mmRLC_SRM_ARAM_ADDR, start);

	for (i = 0; i < length / 2; ++i) {
		reg_offset = read_register(func, mmRLC_SRM_ARAM_DATA);
		reg_offset &= 0x3ffff; /* bits 0:17 are the reg offset */
		reg_data = read_register(func, reg_offset);
		if (func->reg_sav_res_data[start/2+i] != reg_data) {
			equal = false;
			gim_err("register save restore error at func_id = %x,\t"
				"reg offset = 0x%x, saved data =0x%x,\t"
				"data in register = 0x%x\n",
				func->func_id, reg_offset,
				func->reg_sav_res_data[start/2+i],
				reg_data);
		}
		reg_offset = read_register(func, mmRLC_SRM_ARAM_DATA);
	}
	return equal;
}


void gim_save_register(struct function *func)
{
	/* gfx3d static */
	save_register(func, GFX3D_STATIC_START, GFX3D_STATIC_LENGTH);
	gim_info("save_register for gfx3d static\n");

	/* gfx3d dynamic */
	save_register(func, GFX3D_DYN_START, GFX3D_DYN_LENGTH);
	gim_info("save_register for gfx3d dynamic\n");

	/* cmp static */
	save_register(func, CMP_STATIC_START, CMP_STATIC_LENGTH);
	gim_info("save_register for  cmp static\n");

	/* cmp dynamic */
	save_register(func, CMP_DYN_START, CMP_DYN_LENGTH);
	gim_info("save_register for cmp dynamic\n");

	/* cmn static */
	save_register(func, CMN_STATIC_START, CMN_STATIC_LENGTH);
	gim_info("save_register for cmn static\n");

	/* cmn dynamic */
	save_register(func, CMN_DYN_START, CMN_DYN_LENGTH);
	gim_info("save_register for cmn dynamic\n");

	/* vddgfx */
	save_register(func, VDDGFX_START, VDDGFX_LENGTH);
	gim_info("save_register for vddgfx\n");

	/* gfx3d indirect */
	/* cmp indirect */
	/* cmn indirect */
	/* vddgfx indirect */
}

bool gim_check_register(struct function *func)
{
	bool equal = true;
	/* gfx3d static */
	equal = check_register(func, GFX3D_STATIC_START, GFX3D_STATIC_LENGTH);

	/* gfx3d dynamic */
	equal = equal && check_register(func, GFX3D_DYN_START,
					GFX3D_DYN_LENGTH);

	/* cmp static */
	equal = equal && check_register(func, CMP_STATIC_START,
					CMP_STATIC_LENGTH);

	/* cmp dynamic */
	equal = equal && check_register(func, CMP_DYN_START, CMP_DYN_LENGTH);

	/* cmn static */
	equal = equal && check_register(func, CMN_STATIC_START,
					CMN_STATIC_LENGTH);

	/* cmn dynamic */
	equal = equal && check_register(func, CMN_DYN_START, CMN_DYN_LENGTH);

	/* vddgfx */
	equal = equal && check_register(func, VDDGFX_START, VDDGFX_LENGTH);

	/* gfx3d indirect */
	/* cmp indirect */
	/* cmn indirect */
	/* vddgfx indirect */
	return equal;
}

void gim_dump_register(struct function *func)
{
	int i = 0;

	gim_info("dump reg: size = %d\n", func->sav_res_list_size);
	for (i = 0; i < func->sav_res_list_size; i++)
	gim_info("dump reg: offset = %x, data = %x\n",
		 func->reg_sav_res_offset[i],
		 func->reg_sav_res_data[i]);
}

int gim_get_log_level(void)
{
	return g_gim_log_level;
}

void gim_set_log_level(int level)
{
	g_gim_log_level = level;
}

int gim_log_msg(const char *fmt, ...)
{
	va_list args;
	int ret;

	va_start(args, fmt);
	ret = printk(fmt, args);
	va_end(args);

	return ret;
}
