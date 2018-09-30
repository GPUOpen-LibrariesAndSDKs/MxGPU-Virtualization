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

#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/pci.h>
#include <linux/mod_devicetable.h>
#include <linux/fs.h>
#include <linux/version.h>
#include "gim_command.h"
#include "gim_debug.h"

/* Options defined in configure file */
struct config_option {
	const char *name;
	int         value;
	const int   min;
	const int   max;
};

enum option_index {
	CONFIG_OPTION__START = 0,
	CONFIG_OPTION__FB = CONFIG_OPTION__START,
	CONFIG_OPTION__SCHEDULER,
	CONFIG_OPTION__VF_NUMBER,
	CONFIG_OPTION__PF_FB_SIZE,
	CONFIG_OPTION__VF_FB_SIZE,
	CONFIG_OPTION__SCHED_INTERVAL,
	CONFIG_OPTION__SCHED_INTERVAL_US,
	CONFIG_OPTION__FB_CLEAR,
	CONFIG_OPTION__MAX
};

struct config_option config_options[] = {

	{FB_OPTION__KEY, FB_PARTITION__DEFAULT,
	 FB_PARTITION__START, FB_PARTITION__MAX},

	{SCHEDULE_OPTION__KEY, SCHEDULER__DEFAULT,
	 SCHEDULER__START, SCHEDULER__MAX},

	{VF_NUMBER__KEY, VF_NUMBER__DEFAULT,
	 VF_NUMBER__START, VF_NUMBER__MAX},

	{PF_FB__KEY, PF_FB__DEFAULT,
	 PF_FB__START, PF_FB__MAX},

	{VF_FB__KEY, VF_FB__DEFAULT,
	 VF_FB__START, VF_FB__MAX},

	{SCHED_INTERVAL__KEY, SCHED_INTERVAL__DEFAULT,
	 SCHED_INTERVAL__START, SCHED_INTERVAL__MAX},

	{SCHED_INTERVAL_US__KEY, SCHED_INTERVAL_US__DEFAULT,
	 SCHED_INTERVAL_US__START, SCHED_INTERVAL_US__MAX},

	{FB_CLEAR__KEY, FB_CLEAR__DEFAULT, FB_CLEAR__START, FB_CLEAR__MAX},
};

#define MAX_OPTION (sizeof(config_options)/sizeof(struct config_option))
#define MAX_CONFIG_FILE_LENGTH 1024

/* Options input from command line */
#define COMMAND_LINE_OPTION_DEFAULT -1
static uint32_t fb_option    = COMMAND_LINE_OPTION_DEFAULT;
module_param(fb_option, uint, S_IRUGO);
MODULE_PARM_DESC(fb_option, "Frame Buffer Partition.0:static partition; 1: dynamic partition");

static uint32_t sched_option = COMMAND_LINE_OPTION_DEFAULT;
module_param(sched_option, uint, S_IRUGO);
MODULE_PARM_DESC(sched_option, "GPU scheduler. 0: round robin solid; 1: predictable perf; 2: round robin liquid");

static unsigned int vf_num = COMMAND_LINE_OPTION_DEFAULT;
module_param(vf_num, uint, S_IRUGO);
MODULE_PARM_DESC(vf_num, "number of enabled virtual functions. 0: default number of VF in pci sriov config space; N: enable N VFs");

static unsigned int pf_fb = COMMAND_LINE_OPTION_DEFAULT;
module_param(pf_fb, uint, S_IRUGO);
MODULE_PARM_DESC(pf_fb, "Frame Buffer Size in MegaBytes for PF");

static unsigned int vf_fb = COMMAND_LINE_OPTION_DEFAULT;
module_param(vf_fb, uint, S_IRUGO);
MODULE_PARM_DESC(vf_fb, "Frame Buffer Size in MegaBytes for VF");


static unsigned int sched_interval = COMMAND_LINE_OPTION_DEFAULT;
module_param(sched_interval, uint, S_IRUGO);
MODULE_PARM_DESC(sched_interval, "Scheduling time quanta in milliseconds. 0: default quanta(6ms)");

static unsigned int sched_interval_us = COMMAND_LINE_OPTION_DEFAULT;
module_param(sched_interval_us, uint, S_IRUGO);
MODULE_PARM_DESC(sched_interval_us, "Scheduling time quanta in microseconds "
		 "Delta scheduling time add on sched_interval "
		 "0: default quanta(0us) range from 0-999");

static unsigned int fb_clear = COMMAND_LINE_OPTION_DEFAULT;
module_param(fb_clear, uint, S_IRUGO);
MODULE_PARM_DESC(fb_clear, "Clear the VRAM for VF. 0:Skipping; 1: clear FB of VF when VF is free or down");

static int search_config_key(char *key)
{
	int index;
	int ret = -1;

	for (index = 0 ; index < MAX_OPTION; ++index) {
		if (!strcmp(config_options[index].name, key)) {
			ret = index;
			break;
		}
	}
	return ret;
}

static int validate_option(int index, int value)
{
	int ret = -1;

	if (index >= CONFIG_OPTION__START && index <  CONFIG_OPTION__MAX) {
		if (value >= config_options[index].min
		    && value <= config_options[index].max)
			ret = 0;
	}

	return ret;
}

static int get_config_option(int index)
{
	if (index >= CONFIG_OPTION__START
	    && index <  CONFIG_OPTION__MAX)
		return config_options[index].value;

	return -1;
}

static void set_config_option(int index, int value)
{
	if (index >= CONFIG_OPTION__START
	    && index <  CONFIG_OPTION__MAX)
		config_options[index].value = value;
}

static int parse_config_file(char *data, unsigned long long size)
{
	const char delimiters[] = " =\n\t";
	char *token;
	char *running;
	int  empty;
	int  index;

	running = data;
	do {
		/* Get the key. */
		/* Get an non-empty token(including NULL token). */
		do {
			empty = 0;
			token = strsep(&running, delimiters);
			if (token)
				empty = (strlen(token) == 0);
		} while (empty);


		/* expecting a key. */
		if (token != NULL) {
			index = search_config_key(token);
			if (index < 0) {
				/* unknown token found, Error in the
				 * config file, exit.
				 */
				gim_err("AMD GIM unknown token: %s\n", token);
				break;
			}

			/* Get the value. */
			/* Get an non-empty token(including NULL token). */
			do {
				empty = 0;
				token = strsep(&running, delimiters);

				if (token)
					empty = (strlen(token) == 0);
			} while (empty);

			/* expecting a value. */
			if (token != NULL) {
				int value;
				int ret;

				ret = kstrtoint(token, 10, &value);
				if (ret) {
					gim_err("Failed to transfer str: %s\n",
						token);
				}

				if (validate_option(index, value) == 0)
					config_options[index].value = value;

				gim_info("AMD GIM %s = %d\n",
					 config_options[index].name,
					 config_options[index].value);
			}
		}
	} while (token != NULL);

	return 0;
}

static int read_config_file(void)
{
	int ret;
	unsigned long long size;
	struct file *config;
	char *content;
	int rc;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	loff_t loff_t = 0;
#endif

	config = filp_open(GIM_CONFIG_PATH, O_RDONLY, 0);
	if (IS_ERR(config)) {
		rc = PTR_ERR(config);
		gim_warn("can't open %s because of error: %d\n",
			GIM_CONFIG_PATH, rc);
		return -1;
	}

	size = i_size_read(file_inode(config)) + 1;
	content = kzalloc(size, GFP_KERNEL);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
	ret = kernel_read(config, 0, content, size);
#else
	ret = kernel_read(config, content, size, &loff_t);
#endif
	parse_config_file(content, size);
	kfree(content);
	filp_close(config, NULL);
	return 0;
}

static int write_config_file(char *content)
{
	int ret = 0;
	struct file *config;
	int rc;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	loff_t loff_t = 0;
#endif
	config = filp_open(GIM_CONFIG_PATH, O_CREAT|O_RDWR, 0);
	if (IS_ERR(config)) {
		rc = PTR_ERR(config);
		gim_warn("can't open %s because of error: %d\n",
			GIM_CONFIG_PATH, rc);
		return -1;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
	ret = kernel_write(config, content, strlen(content), 0);
#else
	ret = kernel_write(config, content, strlen(content), &loff_t);
#endif
	vfs_fsync(config, 0);
	filp_close(config, NULL);

	return 0;
}

int save_config(void)
{
	char *buf;
	int  index;
	int  cursor = 0;

	buf = vmalloc(MAX_CONFIG_FILE_LENGTH);
	memset(buf, 0, MAX_CONFIG_FILE_LENGTH);
	for (index = 0; index < MAX_OPTION; ++index) {
		cursor += snprintf(buf + cursor,
				   MAX_CONFIG_FILE_LENGTH - cursor,
				   "%s=%d\n",
				   config_options[index].name,
				   config_options[index].value);

		if (cursor >= MAX_CONFIG_FILE_LENGTH)
			break;
	}

	write_config_file(buf);
	vfree(buf);
	return 0;
}

int init_config(void)
{
	read_config_file();

	/* options of command line override options in config file. */
	if (validate_option(CONFIG_OPTION__FB, fb_option) == 0)
		set_config_option(CONFIG_OPTION__FB, fb_option);

	if (validate_option(CONFIG_OPTION__SCHEDULER, sched_option) == 0)
		set_config_option(CONFIG_OPTION__SCHEDULER, sched_option);

	if (validate_option(CONFIG_OPTION__VF_NUMBER, vf_num) == 0)
		set_config_option(CONFIG_OPTION__VF_NUMBER, vf_num);

	if (validate_option(CONFIG_OPTION__PF_FB_SIZE, pf_fb) == 0)
		set_config_option(CONFIG_OPTION__PF_FB_SIZE, pf_fb);

	if (validate_option(CONFIG_OPTION__VF_FB_SIZE, vf_fb) == 0)
		set_config_option(CONFIG_OPTION__VF_FB_SIZE, vf_fb);

	if (validate_option(CONFIG_OPTION__SCHED_INTERVAL,
			    sched_interval) == 0)
		set_config_option(CONFIG_OPTION__SCHED_INTERVAL,
				  sched_interval);
	if (validate_option(CONFIG_OPTION__SCHED_INTERVAL_US,
			    sched_interval_us) == 0)
		set_config_option(CONFIG_OPTION__SCHED_INTERVAL_US,
				  sched_interval_us);

	if (validate_option(CONFIG_OPTION__FB_CLEAR, fb_clear) == 0)
		set_config_option(CONFIG_OPTION__FB_CLEAR, fb_clear);

	gim_info("INIT CONFIG\n");

	/* save options to config file. */
	save_config();

	return 0;
}

uint32_t get_fb_partition_option(void)
{
	return get_config_option(CONFIG_OPTION__FB);
}

uint32_t get_scheduler_option(void)
{
	return get_config_option(CONFIG_OPTION__SCHEDULER);
}

uint32_t get_vf_number_option(void)
{
	return get_config_option(CONFIG_OPTION__VF_NUMBER);
}

uint32_t get_pf_fb_option(void)
{
	return get_config_option(CONFIG_OPTION__PF_FB_SIZE);
}

uint32_t get_vf_fb_option(void)
{
	return get_config_option(CONFIG_OPTION__VF_FB_SIZE);
}

uint32_t get_sched_interval_option(void)
{
	return get_config_option(CONFIG_OPTION__SCHED_INTERVAL);
}

uint32_t get_sched_interval_us_option(void)
{
	return get_config_option(CONFIG_OPTION__SCHED_INTERVAL_US);
}

uint32_t get_fb_clear_option(void)
{
	return get_config_option(CONFIG_OPTION__FB_CLEAR);
}

uint32_t set_option(int index, int value)
{
	uint32_t res = -1;

	if (validate_option(index, value) == 0) {
		set_config_option(index, value);
		save_config();
		res = 0;
	}
	return res;
}

