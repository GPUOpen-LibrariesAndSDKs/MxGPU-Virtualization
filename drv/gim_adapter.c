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
#include <linux/utsname.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/mod_devicetable.h>
#include <linux/fs.h>
#include <linux/sched.h>

#include "gim_file.h"
#include "gim_adapter.h"
#include "gim_os_service.h"
#include "gim_atombios.h"
#include "gim_command.h"
#include "gim_timer.h"
#include "gim_interface.h"
#include "gim_config.h"
#include "gim_gpuiov.h"
#include "gim_debug.h"
#include "gim_kcl_os.h"
#include "gim_kcl_pci.h"
#include "gim_irqmgr.h"
#include "gim_reset.h"
#include "gim_s7150_reg.h"

int in_triger_world_switch;
int in_world_switch;

static struct adapter adapts[MAX_ADAPTERS_IN_SYSTEM];
static uint32_t num_adapters;
static struct adapter *g_adapter;
/*
 * Return either the global scheduler timer interval or the per-function timer
 * interval based on the following algorithm...
 *
 * If the "sched_interval=" parameter in the gim_config file is non-zero then
 * this becomes the global scheduler timer interval for all functions on all
 * adapters (in msec).
 *
 * If the "sched_interval=" parameter in the gim_config file == 0 or is not set
 * then return the per-function time intervals.
 *
 * If the per-function time interval cannot be determined (ie due to disconnect
 * from XL config file) then return a default interval of 7msec.
 */
int get_scheduler_time_interval(struct adapter *adapt, struct function *func)
{
	int quota;

	quota = adapt->quota;

	/* Use individual values from XL config file */
	if (quota == 0) {
		switch (func->sched_level) {
		case 0:
			quota = 4;
			break;

		case 1:
			quota = 6;
			break;

		case 2:
			quota = 8;
			break;

		/* Config is missing or default in gim_config */
		default:
			quota = 7;
			break;
		}
	}
	return quota;
}

void pause_scheduler(struct adapter *adapt)
{
	delete_timer(&adapt->sched_timer);
	adapt->schedler_running = false;

	if (work_pending(&adapt->sched_work)) {
		cancel_work_sync(&adapt->sched_work);
		gim_info("The scheduler has a world switch ");
		gim_info("pending while trying to stop the scheduler\n");
		gim_info("This is not a concern as the schedler\n");
		gim_info("work task will be cancelled\n");
	}

	gim_info("Stop the Scheduler\n");
}

/*
 * Resume the scheduler timer.
 * This timer is used to trigger world switching between VFs
 */
void resume_scheduler(struct adapter *adapt)
{
	int quota;

	if (adapt->curr_running_func == NULL) {
		gim_info("No functions on the runlist.");
		gim_info("Don't need to restart the scheduler\n");
		return;
	}

	if (adapt->curr_running_func
	    == adapt->curr_running_func->next) {
		quota = 500;
		gim_dbg("VF%d is the only function on the runlist.",
			adapt->curr_running_func->func->func_id);
		gim_dbg("Set time interval to %dmsec\n",
			quota);
	} else {
		quota = get_scheduler_time_interval(adapt,
				adapt->curr_running_func->func);
	}

	if (!adapt->schedler_running)
		gim_warn("Restart the Scheduler for %d msec\n", quota);

	start_timer(&adapt->sched_timer, quota);
	adapt->schedler_running = true;
}

bool gim_read_rom_from_file(struct adapter *adapt,
			    unsigned char *rom_image,
			    unsigned int size)
{
	char name[256];
	struct file *fd;
	int result = true;
	int total_bytes;
	struct pci_dev *pci_dev = adapt->pf.pci_dev;

	snprintf(name, 255, "%s/%04x:%02x:%02x.%1u/rom",
		"/sys/bus/pci/devices",
		0, pci_dev->bus->number,
		PCI_SLOT(pci_dev->devfn),
		PCI_FUNC(pci_dev->devfn));

	fd = file_open(name, O_RDWR, 0);
	if (fd == NULL)
		return false;

	/* This is a quirky thing on Linux.  Even though the ROM and the file
	 * for the ROM in sysfs are read-only, the string "1" must be written to
	 * the file to enable the ROM.  After the data has been read, "0" must
	 * be written to the file to disable the ROM.
	 */
	file_write(fd, 0, "1", 1);

	for (total_bytes = 0 ; total_bytes < size ; ) {
		const int bytes = file_read(fd, total_bytes,
					(char *) rom_image + total_bytes,
					size - total_bytes);

		if (bytes == -1) {
			result = false;
			break;
		} else if (bytes == 0)
			break;

		total_bytes += bytes;
	}

	file_write(fd, 0, "0", 1);

	file_close(fd);
	return result;
}


/*
 * gim_read_rom_from_reg() - Read the VBIOS from INDEX/DATA registers,
 * not supported by all ASICs
 * @adapt: pointer to the amdgpuv_device
 * @rom_image: the start virtual address of the rom image
 * @length: the size of rom to be read
 */
void gim_read_rom_from_reg(struct adapter *adapt,
				unsigned char *rom_image,
				unsigned int length)
{
	unsigned int *vbios_rom = NULL;
	unsigned int length_in_dword;
	unsigned int i;

	/* BIOS must be read a DWORD at a time */
	vbios_rom = (unsigned int *)rom_image;
	length_in_dword = length/sizeof(unsigned int);

	gim_info("Reading VBios from ROM");

	/* setup the ROM_INDEX to 0 for initialization */
	write_reg32((void *)adapt->pf.mmr_base, mmSMC_IND_INDEX_6,
			ixROM_INDEX);
	write_reg32((void *)adapt->pf.mmr_base, mmSMC_IND_DATA_6, 0);

	/* set SMC index to data for continous read in the loop */
	write_reg32((void *)adapt->pf.mmr_base, mmSMC_IND_INDEX_6,
			ixROM_DATA);

	for (i = 0; i < length_in_dword; i++)
		vbios_rom[i] = read_reg32((void *)adapt->pf.mmr_base,
					mmSMC_IND_DATA_6);
}

/*
 * gim_vbios_posted() -check if vbios has been posted
 * @adapter: pointer to the amdgpuv_device
 * return true: vbios has been posted
 *		false: vbios has not been posted
 */

bool gim_vbios_posted(struct adapter *adapt)
{
	uint32_t reg;

	reg = read_reg32(adapt->pf.mmr_base, mmCONFIG_MEMSIZE);
	if (reg)
		return true;

	return false;
}

/*
 * gim_read_vbios() - Reade the video BIOS from the ROM
 * @adapt: pointer to amdgpuv_device
 * return	0: success
 *		-1: failure
 */

int gim_read_vbios(struct adapter *adapt)
{
	unsigned char header[GIM_VBIOS_SIGNATURE_END+1] = {0};
	unsigned int length = 0;
	int i = 0;
	int sum = 0;
	unsigned short idx = 0;
	unsigned char *vbios = NULL;

	adapt->vbios_image_size = 0;
	adapt->pvbios_image = NULL;

	/* Validate VBIOS signature */
	gim_read_rom_from_reg(adapt, &header[0], sizeof(header));

	header[GIM_VBIOS_SIGNATURE_END] = 0;
	if (!GIM_IS_VALID_VBIOS(header)) {
		gim_err("Invalid AMD VBIOS\n");
		return -1;
	}

	gim_info("VBIOS starts:  0x%0x, 0x%0x\n", header[0], header[1]);

	length = GIM_VBIOS_LENGTH(header);
	gim_info("VBios size is 0x%x\n", length);

	vbios = vmalloc(length);
	gim_info("vbios allocated at %p\n", vbios);
	if (vbios == NULL)
		return -1;

	gim_read_rom_from_reg(adapt, (unsigned char *)vbios, length);
	idx = vbios[0x18] + ((unsigned short)(vbios[0x19]) << 8);
	gim_info("BIOS Version Major 0x%X Minor 0x%X\n",
		(unsigned short)(vbios[idx + 0x13]),
		(unsigned short)(vbios[idx + 0x12]));

	for (i = 0; i < length; i++)
		sum += vbios[i];

	if (sum & 0xff) {
		gim_err("Invalid video BIOS image, check sum error");
		vfree(vbios);
		return -1;
	}

	adapt->vbios_image_size = length;
	adapt->pvbios_image = vbios;
	gim_info("Valid video BIOS image, ");
	gim_info("size = 0x%x, check sum is 0x%x\n",
		 length, sum);
	return 0;
}


/*
 * gim_post_vbios() - Post the video BIOS
 * @adapt: pointer to amdgpuv_device
 * @force_post: whether force post is needed
 * return	0: success
 *		-1: failure
 */
int gim_post_vbios(struct adapter *adapt, int force_post)
{
	int ret = 0;
	struct cail_adapter_config_info *info;

	info = kzalloc(sizeof(struct cail_adapter_config_info), GFP_KERNEL);
	if (info == NULL)
		return -1;

	adapt->rom_info = (void *)info;

	info->io_base_addr = (void *)adapt->pf.mmr_base;
	info->rom_base_addr = adapt->pvbios_image;
	if (atom_init_parser(adapt) != 0) {
		gim_err("Init Parser failed!\n");
		ret = -1;
		goto out;
	}

	gim_info("Init Parser passed!, continue\n");
	ret = atom_chk_asic_status(adapt);
	if (ret || force_post == FORCE_VBIOS_POST) {
		gim_info("Asic needs a VBios post\n");
		ret = atom_post_vbios(adapt, VBIOS_POST_INIT_ASIC);
		if (ret != 0) {
			gim_err("Post VBIOS failed! - failed to init asic\n");
			goto out;
		}
		gim_info("Post INIT_ASIC successfully!\n");
	} else {
		gim_info("Asic has already been initialized, don't need to INIT\n");
	}
	patch_firmware(adapt);

	gim_info("Asic needs firmware loaded\n");
	ret = atom_post_vbios(adapt, VBIOS_POST_LOAD_UCODE);
	if (ret != 0) {
		gim_err("Post VBIOS failed! - failed to load firmware\n");
		goto out;
	}
	gim_info("Post LOAD_FW successfully!\n");
	gim_info("Post VBIOS successfully!\n");

out:
	kfree(info);
	adapt->rom_info = NULL;
	return ret;
}

void *map_doorbell(struct pci_dev *pdev)
{
	void *p_doorbell_base = NULL;
	unsigned int flag = 0;
	int i = 0;

	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		flag = pci_resource_flags(pdev, i);
		if ((flag & IORESOURCE_MEM) && (flag & IORESOURCE_PREFETCH)
			 && pci_resource_len(pdev, i) <= 0x800000
			 && pci_resource_len(pdev, i) > 0x40000)
			break;
	}

	if (i == DEVICE_COUNT_RESOURCE) {
		gim_err("adapter has no suitable doorbell region\n");
		return NULL;
	}

	p_doorbell_base = ioremap_nocache(pci_resource_start(pdev, i),
					pci_resource_len(pdev, i));

	if (p_doorbell_base == NULL)
		gim_err("can't iomap for BAR %d\n",  i);

	return p_doorbell_base;
}

void *map_fb(struct pci_dev *pdev)
{
	void *p_fb_base = NULL;
	int i = 0;

	p_fb_base = ioremap_nocache(pci_resource_start(pdev, i),
					pci_resource_len(pdev, i));

	if (p_fb_base == NULL)
		gim_err("can't iomap for BAR %d\n",  i);

	return p_fb_base;

}
void *map_vf_fb(struct pci_dev *pdev)
{
	void *p_fb_base = NULL;
	int i = 0;

	gim_info("Map region 0x%llx for length %lld\n",
		pci_resource_start(pdev, i),
		pci_resource_len(pdev, i));
	p_fb_base = ioremap_nocache(pci_resource_start(pdev, i),
				pci_resource_len(pdev, i));

	if (p_fb_base == NULL)
		gim_err("can't iomap for BAR %d\n",  i);

	return p_fb_base;

}

static int linux_supports_subordinate_bus(struct adapter *adapt)
{
	struct new_utsname *buf;
	uint x = 0, y = 0, z = 0;
	int ret;

	buf = utsname();

	ret = sscanf(buf->release, "%u.%u.%u", &x, &y, &z);
	if (ret != 3)
		gim_warn("Faied to get x,y,z\n");
	/*
	 * Linux release numbers are usually of the form x.y.z but occasionally
	 * they are also x.y.z.a or simply m.n
	 * 'x' is typically a single digit.
	 * 'y' can be one or two numerical digits
	 * 'z' can be 3 digits (or more?)
	 *
	 * Support for subordinate bus was introduced in release 3.13.4
	 */
	gim_info("Linux release \"%s\"\n", buf->release);

	if (x > 3)
		return 1;  /* 4.x.x or greater */

	if (x < 3)
		return 0;  /* 2.x.x or less */

	/* Must be 3.x.x */
	if (y > 13)
		return 1;  /* 3.14.x or greater */

	if (y < 13)
		return 0;  /* 3.12.x or less */

	/* Must be 3.13.x */
	if (z >= 4)
		return 1;  /* 3.13.4 or greater */

	return 0;
}

static int program_ari_mode(struct adapter *adapt, int mode)
{
	uint32_t	bif_strap8;

	pf_write_register(adapt, mmPCIE_INDEX, 0x1500008);
	bif_strap8 = pf_read_register(adapt, mmPCIE_DATA);

	gim_info("Read bif_strap8 = 0x%08x\n", bif_strap8);

	bif_strap8 &= ~ARI_MODE_MASK;

	if (mode == PF_BUS) {
		gim_info("program_ari_mode - Set ARI_Mode = PF_BUS\n");
		bif_strap8 |= ARI_MODE_0;
	} else if (mode == PF_BUS_PLUS_1) {
		gim_info("program_ari_mode - Set ARI_Mode = PF+1_BUS\n");
		bif_strap8 |= ARI_MODE_1;
	} else {
		gim_info("program_ari_mode - Invalid ARI mode\n");
		return -1;
	}

	gim_info("Write bif_strap8 = 0x%08x\n", bif_strap8);
	pf_write_register(adapt, mmPCIE_DATA, bif_strap8);
	return 0;
}
/*
 * Determine the best ARI mode and set the adapt accordingly
 *
 * ARI mode is determined by the following criteria;
 *
 * - If ARI is enabled by platform, use all VF's on PF bus
 * - If Linux does not support suborinate bus (PF+1 bus) then use all VF's
 *   on PF bus
 * - Use VF on PF+1 bus only if ARI is not enabled and Linux can support it.
 *     Linux 3.13.4 introduces proper support for VFs on PF+1 bus. Versions
 *     of Linux prior to 3.13.4 will crash in the kernel when enabling sriov.
 */
static void set_ari_mode(struct adapter *adapt)
{
	if (!sriov_is_ari_enabled(adapt->pf.pci_dev)) {
		/* If ARI not enabled, try to enable it */
		if (!sriov_enable_ari(adapt->pf.pci_dev)) {
		/* Couldn't enable ARI, check if Linux supports PF_BUS+1 */
			if (linux_supports_subordinate_bus(adapt)) {
				program_ari_mode(adapt, PF_BUS_PLUS_1);
				return;
			}
		/* ARI not enabled and Linux does not support subordinate bus
		 * fall thru and use PF bus
		 */
		}
	}
	program_ari_mode(adapt, PF_BUS);
}

static struct pci_dev *search_p2p_bridge_dev(struct pci_dev *dev)
{
	struct pci_bus *bus;
	struct pci_dev *bridge;

	bus = dev->bus;

	while (bus) {
		bridge = bus->self;
		if (bridge) {
			if ((bridge->class >> 8) == PCI_CLASS_BRIDGE_PCI)
				return bridge;
		}
		bus = bus->parent;
	}

	return NULL;
}

int mask_unrecoverable_errors(struct pci_dev *dev)
{
	/* Search PCI_EXT_CAP_ID_AER of Advanced Error reporting */
	int pos = 0;

	pos = kcl_pci_find_ext_capability(dev, PCI_EXT_CAP_ID_AER, -1);
	if (pos) {
		/* mask all errors */
		kcl_pci_write_config_dword(dev,
					pos + PCI_ERR_UNCOR_MASK,
					0xffffffff);
		return 0;
	}

	return -1;
}

void clear_unrecoverable_errors(struct pci_dev *dev)
{
	struct pci_bus *bus;
	struct pci_dev *bridge;

	bus = dev->bus;

	while (bus) {
		bridge = bus->self;
		if (bridge) {
			if (mask_unrecoverable_errors(bridge) != 0)
				gim_warn("mask AER on bridge failed!\n");
		}
		bus = bus->parent;
	}

	if (mask_unrecoverable_errors(dev) != 0)
		gim_warn("mask AER on PF failed!\n");
}

void sched_work_handler(struct work_struct *work)
{
	struct adapter *adapt = container_of(work, struct adapter, sched_work);

	triger_world_switch(adapt, false);
}

uint32_t set_new_adapter(struct pci_dev *pdev)
{
	unsigned int interrupt_status;
	unsigned int size;
	unsigned int idx;

	/*
	 * num_adapters - is a global variable that is incremented each time
	 * that set_new_adapter() is called.  It maintains a count of the number
	 * of adapts that GIM is handling.
	 *
	 * adapts - also a global variable.  It is an array of struct adapter
	 * structures
	 *
	 */
	if (num_adapters < MAX_ADAPTERS_IN_SYSTEM) {
		struct adapter *curr = adapts + num_adapters;
		struct function *pf = &(curr->pf);

		gim_info("curr allocated at %p\n", curr);

		if (pdev->sriov == NULL) {
			gim_err("asic does not support SRIOV\n");
			return GIM_ERROR;
		}

		gim_info("SRIOV is supported\n");

		memset(curr, 0, sizeof(struct adapter));

		curr->p2p_bridge_dev = search_p2p_bridge_dev(pdev);
		if (!curr->p2p_bridge_dev)
			gim_err("can't find PCI bridge device!\n");
		else {
			gim_info("found PCI bridge device\n");
			gim_info("found: %02x:%x.%x\n",
				curr->p2p_bridge_dev->bus->number,
				(curr->p2p_bridge_dev->devfn>>3),
				(curr->p2p_bridge_dev->devfn & 0x7));
		}

		/* init spin lock and mutex */
		spin_lock_init(&curr->lock);
		spin_lock_init(&curr->ih_lock);
		spin_lock_init(&curr->mailbox_lock);

		mutex_init(&curr->signal_mutex);
		mutex_init(&curr->curr_running_func_mutex);

		/* map resources */
		curr->pf.pci_dev = pdev;
		curr->pf.bdf    = get_bdf(pdev);
		curr->pf.adapt = curr;

		map_mmio(pf, pdev);
		gim_info("mmio_base = %p", curr->pf.mmr_base);
		curr->pf.doorbell = map_doorbell(pdev);
		gim_info("doorbell = %p", curr->pf.doorbell);
		curr->pf.fb_va = map_fb(pdev);
		gim_info("pf.fb_va = %p", curr->pf.fb_va);
		curr->pf.fb_pa.quad_part = pci_resource_start(pdev, 0);

		/* enable IO/Memory/bus master */
		enable_device(curr->pf.pci_dev);

#if 0
		/* enable multi-vectors MSI */
		enable_mv_msi(curr);
#endif

		/* This is workaround currently
		 * At INIT_GPU, the RLC-V fw will check the scratch ram
		 * location 0x20(FB_CLEAR_DMA_SIZE) to decide:
		 *      FB_CLEAR_DMA_SIZE[31..16] count of DMA,
		 *      FB_CLEAR_DMA_SIZE[15..0 ] size_in_bytes, and
		 *      FB_CLEAR_DMA_SIZE[31..16] * FB_CLEAR_DMA_SIZE[15..0]
		 *        bytes of frame buffer is cleared by RLCV.
		 * if FB_CLEAR_DMA_SIZE[31..0] is zero, whole buffer will
		 * be cleared.
		 */

		/* Since RLCV is using wrong alorithm and consumes extra time,
		 * the clear frame of INIT_GPU is disalbed by setting:
		 *     FB_CLEAR_DMA_SIZE[31..16] = 1
		 *     FB_CLEAR_DMA_SIZE[15..0 ] = 0
		 * amdgpuv_clearFrameBuffer() takes its position instead until
		 * RLCV fix its problem
		 */
		pf_write_register(curr, mmRLC_GPU_IOV_SCRATCH_ADDR, 0x20);
		pf_write_register(curr, mmRLC_GPU_IOV_SCRATCH_DATA, 0x00010000);

		set_ari_mode(curr);

		if (gim_read_vbios(curr) != 0) {
			gim_err("fail to read vbios");
			return GIM_ERROR;
		}

		/* Note: this workaround is only temporary until
		 * vbios fix bug
		 */
		if (!gim_vbios_posted(curr)) {
			if (gim_post_vbios(curr, POST_VBIOS_IF_NEEDED) != 0) {
				gim_err("fail to post vbios");
				return GIM_ERROR;
			}
			gim_info("gim_post_vbios done");
		}

		/* TODO:
		 * be aware of mulit adapter case
		 */
		g_adapter = curr;

		/* This is workaround currently
		 * At INIT_GPU, the RLC-V fw will check the scratch ram
		 * location 0x20 (FB_CLEAR_DMA_SIZE) to decide:
		 *      FB_CLEAR_DMA_SIZE[31..16] count of DMA,
		 *      FB_CLEAR_DMA_SIZE[15..0 ] size_in_bytes, and
		 * FB_CLEAR_DMA_SIZE[31..16] * FB_CLEAR_DMA_SIZE[15..0]
		 * bytes of frame buffer is cleared by RLCV.
		 * If FB_CLEAR_DMA_SIZE[31..0] is zero, whole buffer will
		 * be cleared
		 */

		/* Since RLCV is using wrong alorithm and consumes extra time,
		 * the clear frame of INIT_GPU is disalbed by setting:
		 *     FB_CLEAR_DMA_SIZE[31..16] = 1
		 *     FB_CLEAR_DMA_SIZE[15..0 ] = 0
		 * amdgpuv_clearFrameBuffer() takes its position instead until
		 * RLCV fix its problem
		 */
		write_register(pf, mmRLC_GPU_IOV_SCRATCH_ADDR, 0x20);
		write_register(pf, mmRLC_GPU_IOV_SCRATCH_DATA, 0x00010000);


		num_adapters++;
		curr->adp_id = num_adapters;
		curr->adapt_id = num_adapters;
		curr->quota = get_sched_interval_option();
		if (curr->quota)
			gim_info("Scheduler Time interval set to %d msec\n",
				curr->quota);
		else {
			gim_info("Scheduler Time interval is per-vf from XL");
			gim_info("config file\n");
		}

		curr->enable_preeption = true;
		if (enable_sriov(curr->pf.bdf, get_vf_number_option())
		    != GIM_OK) {
			gim_err("Failed to properly enable SRIOV\n");
			return GIM_ERROR;
		}

		pci_gpu_iov_init(curr->pf.pci_dev, &curr->gpuiov);

		init_frame_buffer_partition(curr);

		curr->curr_running_func = NULL;
		curr->runnig_func_list = NULL;
		curr->pf_flr_pci_cfg = vmalloc(PF_FLR_PCI_CONFIG_SIZE);
		if (!curr->pf_flr_pci_cfg) {
			gim_err("fail to allocate memory for pf_flr_pci_cfg\n");
			return GIM_ERROR;
		}
		/* workaround the display timer issue,
		 * remove this if VBIOS added this logic.
		 */
		write_register(pf, mmSMU_IND_INDEX_0, ixCG_INTERRUPT_STATUS);
		interrupt_status = read_register(pf, mmSMU_IND_DATA_0);

		interrupt_status &=
			~(CG_INTERRUPT_STATUS__DISP_TIMER_TRIGGER_MASK_MASK
			| CG_INTERRUPT_STATUS__DISP_TIMER2_TRIGGER_MASK_MASK);

		write_register(pf, mmSMU_IND_INDEX_0, ixCG_INTERRUPT_STATUS);
		write_register(pf, mmSMU_IND_DATA_0, interrupt_status);

		write_register(pf, mmSMU_IND_INDEX_0, 0);

		/* Set the dma mask to be 40 bits address range */
		pci_set_dma_mask(curr->pf.pci_dev, 0xffffffffffull);

		/* interrupt */
		size = sizeof(struct interrupt_handler);
		curr->ih = kcl_mem_small_buffer_alloc(size);
		memset(curr->ih, 0, sizeof(struct interrupt_handler));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 1)
		pci_set_power_state(pdev, PCI_D0);
#endif

		/* enable MSI */
		gim_info("enable MSI");
		if (pci_enable_msi(curr->pf.pci_dev)) {
			gim_err("fail to enable MSI");
			return GIM_ERROR;
		}

		/* init IH ring */
		if (ih_iv_ring_init(curr) != 0) {
			gim_err("fail to init iv ring");
			return GIM_ERROR;
		}

		/* init work */
		gim_info("init work");
		INIT_WORK(&curr->normal_work, work_handler);

		INIT_WORK(&curr->sched_work, sched_work_handler);

		/* register interrupt */
		gim_info("register interrupt");
		if (adapter_interrupt_register(curr) != 0) {
			gim_err("fail to register interrupt");
			return GIM_ERROR;
		}

		/* enable irq source */
		if (ih_irq_source_enable(curr) != 0) {
			gim_err("fail to enable interrupt source");
			return GIM_ERROR;
		}

		/* set flag for irq_task list */
		curr->vf_req_gpu_access = 0;

		/*
		 * workaround
		 * rlcv might fail to enter safe mode if clock gating is
		 * enabled disable clock gating as a temp solution,
		 * 0x0020003c is the golden setting from kmd driver.
		 */
		write_register(pf, mmRLC_CGCG_CGLS_CTRL, 0x0020003c);

		/* init and save the PF state for later use */
		init_register_init_state(curr);

		gim_clear_all_errors(curr);

		clear_unrecoverable_errors(curr->pf.pci_dev);

		curr->sched_opt = get_scheduler_option();
		if (curr->sched_opt == SCHEDULER__PREDICT_PERF) {
			/* PF is a special case and it must be scheduled */
			curr->pf.is_scheduled = 1;

			/* Add all vfs into runlist
			 * But mark all vfs can be not scheduled
			 */
			for (idx = 0; idx < curr->total_vfs; ++idx) {
				add_func_to_run_list(&curr->vfs[idx]);
				curr->vfs[idx].is_scheduled = 0;
			}
		}

		/* Initialize the scheduler timer */
		set_timer(&curr->sched_timer);
		/* Initialize the timeout check timer*/
		set_timeout_timer(&curr->timeout_timer);

		/* Timer not running yet */
		curr->schedler_running = false;
		resume_scheduler(curr);
		curr->switch_to_itself = true;

		return GIM_OK;
	}
	return GIM_ERROR;
}

uint32_t init_vfs(struct adapter *adapt, uint32_t vfs_count)
{
	uint32_t count;
	uint32_t i;
	struct pci_dev *vf_devices[16];

	/* max vfs number could be used in gim.
	 *  min(vf_num_in_gim_config, vf_num_in_sriov)
	 */
	adapt->total_vfs       = vfs_count;
	adapt->enabled_vfs     = vfs_count;
	adapt->available_vfs   = adapt->enabled_vfs;
	count = enumerate_vfs(adapt->pf.pci_dev, vfs_count, vf_devices);
	if (count == 0) {
		gim_err("Failed to enumerate any VFs\n");
		return GIM_ERROR;
	}

	for (i = 0 ; i < count ; ++i) {
		adapt->vfs[i].adapt     = adapt;
		adapt->vfs[i].pci_dev      = vf_devices[i];
		adapt->vfs[i].bdf         = get_bdf(vf_devices[i]);
		adapt->vfs[i].is_available = 1;

		if (adapt->sched_opt == SCHEDULER__PREDICT_PERF)
			adapt->vfs[i].is_scheduled = 0;

		adapt->vfs[i].func_id = get_func_id(adapt->pf.pci_dev,
							vf_devices[i]);
		map_mmio(&(adapt->vfs[i]), adapt->vfs[i].pci_dev);

		adapt->vfs[i].dom_id = -1;
		adapt->vfs[i].fb_partition = NULL;
		adapt->vfs[i].sched_level = -1;
		adapt->fn_list_nodes[i].inuse = 0;
		pci_disable_error_reporting(adapt->vfs[i].pci_dev);
	}
	return GIM_OK;
}


void unmap_mmr_base(struct adapter *p_adapt)
{
	int i;

	iounmap(p_adapt->pf.mmr_base);
	for (i = 0 ; i < p_adapt->total_vfs  ; ++i) {
		if (p_adapt->vfs[i].mmr_base != NULL)
			iounmap(p_adapt->vfs[i].mmr_base);
	}
}

void pause_all_schedulers(void)
{
	int i;
	struct adapter *curr;

	gim_info("Pause all schedulers\n");
	for (i = 0 ; i < num_adapters ; ++i) {
		curr = adapts + i;

		delete_timer(&curr->sched_timer);
	}
}

void release_all_adapters(void)
{
	uint32_t i;
	struct adapter *curr;

	for (i = 0 ; i < num_adapters ; ++i) {
		curr = adapts + i;

		vfree(curr->pvbios_image);
		ih_irq_source_disable(curr);

		ih_iv_ring_fini(curr);
		if (curr->ih != NULL) {
			kcl_mem_small_buffer_free(curr->ih);
			curr->ih = NULL;
		}

		if (curr->pf_flr_pci_cfg != NULL) {
			vfree(curr->pf_flr_pci_cfg);
			curr->pf_flr_pci_cfg = NULL;
		}

		adapter_interrupt_unregister(curr);
		pci_disable_msi(curr->pf.pci_dev);
		unmap_mmr_base(curr);
		iounmap(curr->pf.fb_va);
		iounmap(curr->pf.doorbell);
		delete_timer(&curr->sched_timer);

		/* Disable SRIOV. */
		disable_sriov(curr->pf.bdf);
	}
}

uint32_t get_adapter_count(void)
{
	return num_adapters;
}

struct adapter *get_adapters(void)
{
	return adapts;
}

struct adapter *get_default_adapter(void)
{
	return g_adapter;
}

/* This routine must be called with fb_dynamic_alloc_mutex held */
static struct slot_list_node *find_suitable_slot(struct adapter *adapt,
						int vf_fb_size)
{
	struct slot_list_node *slot;

	slot = adapt->empty_slot_list;
	while (slot->slot.size < vf_fb_size)
		slot = slot->next;

	return slot;
}

/* This routine must be called with fb_dynamic_alloc_mutex held */
static void sort_slot_list(struct adapter *adapt,
			struct slot_list_node *modified_slot)
{
	struct slot_list_node *p;

	gim_info("--adapt->empty_slot_list = 0x%llx\n",
			(kcl_type_u64)adapt->empty_slot_list);
	gim_info("--modified_slot = 0x%llx\n",
			(kcl_type_u64)modified_slot);
	gim_info("--modified_slot->slot.base = %lld\n",
			modified_slot->slot.base);
	gim_info("--modified_slot->slot.size = %lld\n",
			modified_slot->slot.size);
	gim_info("--modified_slot->next = 0x%llx\n",
			(kcl_type_u64)modified_slot->next);

	/* remove */
	if (adapt->empty_slot_list) {
		if (adapt->empty_slot_list == modified_slot)
			adapt->empty_slot_list = modified_slot->next;
		else {
			p = adapt->empty_slot_list;
			while (p->next && p->next != modified_slot)
				p = p->next;

			if (p->next)/* p->next == modified_slot */
				p->next = modified_slot->next;
			else
				gim_err("slot is not in the list");
		}
	}

	/* insert to proper position */
	gim_info(" --adapt->empty_slot_list = 0x%llx\n",
			(kcl_type_u64)adapt->empty_slot_list);
	if (adapt->empty_slot_list == NULL) {
		modified_slot->next = adapt->empty_slot_list;
		adapt->empty_slot_list = modified_slot;
	} else {
		if (adapt->empty_slot_list->slot.size
		    >= modified_slot->slot.size) {
			modified_slot->next = adapt->empty_slot_list;
			adapt->empty_slot_list = modified_slot;
		} else {
			p = adapt->empty_slot_list;

			while (p->next &&
				p->next->slot.size < modified_slot->slot.size)
				p = p->next;

			modified_slot->next = p->next;
			p->next = modified_slot;
		}
	}
}

/* This routine must be called with fb_dynamic_alloc_mutex held */
static void delete_slot(struct adapter *adapt, struct slot_list_node *slot)
{
	struct slot_list_node *p;

	if (adapt->empty_slot_list) {
		if (adapt->empty_slot_list == slot) {
			adapt->empty_slot_list = slot->next;
		} else {
			p = adapt->empty_slot_list;
			while (p->next && p->next != slot)
				p = p->next;

			if (p->next)
				p->next = slot->next;
			else
				gim_err("error: slot is not in the list ");
		}
		vfree(slot);
	}
}

/* This routine must be called with fb_dynamic_alloc_mutex held */
static void add_and_merge_slot(struct adapter *adapt,
				struct slot_list_node *slot)
{
	struct slot_list_node *p;
	struct slot_list_node *tmp;

	gim_info("-- add and merge slot!\n");

	p = adapt->empty_slot_list;
	/* insert to head of the list */
	adapt->empty_slot_list = slot;
	slot->next = p;

	/* try to merge */
	while (p) {
		if (slot->slot.base + slot->slot.size == p->slot.base) {
			slot->slot.size = slot->slot.size + p->slot.size;
			/* remove p from list */
			tmp = p;
			p = p->next;
			delete_slot(adapt, tmp);
		} else if (p->slot.base + p->slot.size ==  slot->slot.base) {
			slot->slot.base = p->slot.base;
			slot->slot.size = slot->slot.size + p->slot.size;
			/* remove p from list */
			tmp = p;
			p = p->next;
			delete_slot(adapt, tmp);
		} else
			p = p->next;
	}

	sort_slot_list(adapt, slot);
}



static bool ring_is_empty(struct function *func)
{
	uint32_t reg1, reg2;

	reg1 = read_register(func, mmCP_RB_WPTR);
	reg2 = read_register(func, mmCP_RB_RPTR);
	if (reg1 != reg2) {
		gim_warn("CP_RB_WPTR (0x%08x) != CP_RB_RPTR (0x%08x)\n",
			reg1, reg2);
		return false;
	}

	reg1 = read_register(func, mmCP_RB1_WPTR);
	reg2 = read_register(func, mmCP_RB1_RPTR);
	if (reg1 != reg2) {
		gim_warn("CP_RB1_WPTR (0x%08x) != CP_RB1_RPTR (0x%08x)\n",
			reg1, reg2);
		return false;
	}

	reg1 = read_register(func, mmCP_RB2_WPTR);
	reg2 = read_register(func, mmCP_RB2_RPTR);
	if (reg1 != reg2) {
		gim_warn("CP_RB2_WPTR (0x%08x) != CP_RB2_RPTR (0x%08x)\n",
			reg1, reg2);
		return false;
	}

	reg1 = read_register(func, mmSDMA0_GFX_RB_WPTR);
	reg2 = read_register(func, mmSDMA0_GFX_RB_RPTR);
	if (reg1 != reg2) {
		gim_warn("SDMA0_GFX_RB_WPTR (0x%08x) != ", reg1);
		gim_warn("SDMA0_GFX_RB_RPTR (0x%08x)\n", reg2);
		return false;
	}

	reg1 = read_register(func, mmSDMA1_GFX_RB_WPTR);
	reg2 = read_register(func, mmSDMA1_GFX_RB_RPTR);
	if (reg1 != reg2) {
		gim_warn("SDMA1_GFX_RB_WPTR (0x%08x) != ", reg1);
		gim_warn("SDMA1_GFX_RB_RPTR (0x%08x)\n", reg2);
		return false;
	}

	return true;
}


static bool gfx_is_idle(struct function *func)
{
	uint32_t data;

	data =  read_register(func, mmGRBM_STATUS);
	if (data & (GRBM_STATUS__TA_BUSY_MASK	 | GRBM_STATUS__GDS_BUSY_MASK |
		GRBM_STATUS__VGT_BUSY_MASK | GRBM_STATUS__IA_BUSY_NO_DMA_MASK |
		GRBM_STATUS__IA_BUSY_MASK  | GRBM_STATUS__SX_BUSY_MASK  |
		GRBM_STATUS__SPI_BUSY_MASK | GRBM_STATUS__BCI_BUSY_MASK |
		GRBM_STATUS__SC_BUSY_MASK  | GRBM_STATUS__PA_BUSY_MASK  |
		GRBM_STATUS__DB_BUSY_MASK  | GRBM_STATUS__CB_BUSY_MASK))
		return false;

	if (data & (GRBM_STATUS__CP_BUSY_MASK |
			GRBM_STATUS__CP_COHERENCY_BUSY_MASK))
		return false;

	data =  read_register(func, mmGRBM_STATUS2);

	if (data & GRBM_STATUS2__RLC_BUSY_MASK)
		return false;

	if (data & (GRBM_STATUS2__CPF_BUSY_MASK |
			GRBM_STATUS2__CPC_BUSY_MASK |
			GRBM_STATUS2__CPG_BUSY_MASK))
		return false;

	data =  read_register(func, mmSRBM_STATUS2);
	if (data & SRBM_STATUS2__SDMA_BUSY_MASK)
		return false;

	if (data & SRBM_STATUS2__SDMA1_BUSY_MASK)
		return false;

	if (data & SRBM_STATUS2__XDMA_BUSY_MASK)
		return false;

	data =  read_register(func, mmSDMA0_STATUS_REG);

	if (!(data & SDMA0_STATUS_REG__IDLE_MASK))
		return false;


	data =  read_register(func, mmSDMA1_STATUS_REG);

	if (!(data & SDMA1_STATUS_REG__IDLE_MASK))
		return false;

	data =  read_register(func, mmSRBM_STATUS);

	if (data & (SRBM_STATUS__DRM_RQ_PENDING_MASK |
			SRBM_STATUS__DRM_BUSY_MASK))
		return false;

	/* TODO, we need to have a way to check if RLC busy.
	 * Confirm new RLC design.
	 */

	if (data & SRBM_STATUS__SEM_BUSY_MASK)
		return false;

	if (data & SRBM_STATUS__GRBM_RQ_PENDING_MASK)
		return false;

	if (data & SRBM_STATUS__VMC_BUSY_MASK)
		return false;

	gim_dbg("Gfx is idle now\n");
	return true;

}

void check_me_cntl(struct adapter *adapt, char *comment)
{
	uint32_t  data;

	data = pf_read_register(adapt, mmCP_ME_CNTL);
	if (data) {
		gim_info("CP_ME_CNTL = 0x%08x %s\n", data, comment);
		if (data & CP_ME_CNTL__ME_HALT_MASK)
			gim_err("  ME HALTED!\n");

		if (data & CP_ME_CNTL__PFP_HALT_MASK)
			gim_err("  PFP HALTED!\n");

		if (data & CP_ME_CNTL__CE_HALT_MASK)
			gim_err("  CE HALTED!\n");
	} else
		gim_info("ME/PFP/CE running %s\n", comment);
}

void check_base_addrs(struct function *func, char *comment)
{
	uint32_t data_l;
	uint32_t data_h;

	write_register(func, mmSRBM_GFX_CNTL, 9);
	data_l = read_register(func, mmCP_MQD_BASE_ADDR);
	data_h = read_register(func, mmCP_MQD_BASE_ADDR_HI);
	write_register(func, mmSRBM_GFX_CNTL, 0);
	gim_info("CP_MQD_BASE_ADDR = 0x%0x:%08x\n", data_h, data_l);
}

void dump_gpu_status(struct function *func)
{
	uint32_t data;
	uint32_t rlc;
	uint32_t rptr, wptr;

	gim_err("**** dump gpu status begin for struct adapter %d:%02d.%02d\n",
		((func->adapt->pf.bdf)>>8)&0xff,
		((func->adapt->pf.bdf)>>3)&0x1f,
		(func->adapt->pf.bdf)&0x7);
	pci_check_for_error(func->adapt->pf.pci_dev);
	check_base_addrs(func, " ");
	rptr = read_register(func, mmCP_RB_RPTR);
	wptr = read_register(func, mmCP_RB_WPTR);

	if (rptr != wptr) {
		gim_err(" CP Ring buffer is not empty,");
		gim_err(" RPTR = 0x%08x, WPTR = 0x%08x\n", rptr, wptr);
		gim_err("    When IDLE_GPU was sent RPTR = 0x%08x,\t"
			"WPTR = 0x%08x\n", func->rptr, func->wptr);
	}

	if (!ring_is_empty(func))
		gim_err("At least one ring is active\n");

	if (read_register(func, mmCP_RB1_WPTR)
		!= read_register(func, mmCP_RB1_RPTR))
		gim_err(" CP1 Ring buffer is not empty\n");


	if (read_register(func, mmCP_RB2_WPTR)
		!= read_register(func, mmCP_RB2_RPTR))
		gim_err(" CP2 Ring buffer is not empty\n");


	if (read_register(func, mmSDMA0_GFX_RB_WPTR)
		!= read_register(func, mmSDMA0_GFX_RB_RPTR))
		gim_err(" SDMA0 Ring buffer is not empty\n");


	if (read_register(func, mmSDMA1_GFX_RB_WPTR)
		!= read_register(func, mmSDMA1_GFX_RB_RPTR))
		gim_err(" SDMA1 Ring buffer is not empty\n");


	data =  read_register(func, mmGRBM_STATUS);
	gim_err(" mmGRBM_STATUS = 0x%x\n", data);

	data =  read_register(func, mmGRBM_STATUS2);
	gim_err(" mmGRBM_STATUS2 = 0x%x\n", data);

	data =  read_register(func, mmSRBM_STATUS);
	gim_err(" mmSRBM_STATUS = 0x%x\n", data);

	data =  read_register(func, mmSRBM_STATUS2);
	gim_err(" mmSRBM_STATUS2 = 0x%x\n", data);

	data =  read_register(func, mmSDMA0_STATUS_REG);
	gim_err(" mmSDMA0_STATUS_REG = 0x%x\n", data);

	data =  read_register(func, mmSDMA1_STATUS_REG);
	gim_err(" mmSDMA1_STATUS_REG = 0x%x\n", data);


	data =  read_register(func, mmGRBM_STATUS);
	if (data & (GRBM_STATUS__TA_BUSY_MASK	 | GRBM_STATUS__GDS_BUSY_MASK |
		GRBM_STATUS__VGT_BUSY_MASK | GRBM_STATUS__IA_BUSY_NO_DMA_MASK |
		GRBM_STATUS__IA_BUSY_MASK  | GRBM_STATUS__SX_BUSY_MASK	 |
		GRBM_STATUS__SPI_BUSY_MASK | GRBM_STATUS__BCI_BUSY_MASK	 |
		GRBM_STATUS__SC_BUSY_MASK  | GRBM_STATUS__PA_BUSY_MASK	 |
		GRBM_STATUS__DB_BUSY_MASK  | GRBM_STATUS__CB_BUSY_MASK))
		gim_err("GFX busy\n");

	if (data & (GRBM_STATUS__CP_BUSY_MASK |
			GRBM_STATUS__CP_COHERENCY_BUSY_MASK))
		gim_err("CP busy\n");

	data =  read_register(func, mmGRBM_STATUS2);

	if (data & GRBM_STATUS2__RLC_BUSY_MASK) {
		gim_err("RLC busy\n");
		rlc = read_register(func, mmRLC_STAT);
		if (rlc) {
			gim_err("RLC_STAT = 0x%08x\n", rlc);
			if (rlc & RLC_STAT__RLC_BUSY_MASK) {
				gim_err("    RLC busy processing a context");
				gim_err("switch\n");
			}

			if (rlc & RLC_STAT__RLC_GPM_BUSY_MASK) {
				gim_err("    RLC Graphics Power Management");
				gim_err("unit is busy\n");
			}

			if (rlc & RLC_STAT__RLC_SPM_BUSY_MASK) {
				gim_err("    RLC Streaming Performance");
				gim_err("Monitor block is busy\n");
			}
		}

		rlc = read_register(func, mmRLC_GPM_STAT);
		if (rlc & 1) {
			gim_err("RLC_GPM_STAT = 0x%08x", rlc);
			gim_err(" - RLC GPM module is busy\n");
		}
	}

	if (data & (GRBM_STATUS2__CPF_BUSY_MASK |
			GRBM_STATUS2__CPC_BUSY_MASK |
			GRBM_STATUS2__CPG_BUSY_MASK))
		gim_err("CP busy\n");



	data =  read_register(func, mmSRBM_STATUS2);

	if (data & SRBM_STATUS2__SDMA_BUSY_MASK)
		gim_err("SDMA busy\n");

	if (data & SRBM_STATUS2__SDMA1_BUSY_MASK)
		gim_err("SDMA1 busy\n");

	if (data & SRBM_STATUS2__XDMA_BUSY_MASK)
		gim_err("XDMA busy\n");

	data =  read_register(func, mmSDMA0_STATUS_REG);

	if (!(data & SDMA0_STATUS_REG__IDLE_MASK))
		gim_err("SDMA busy\n");

	data =  read_register(func, mmSDMA1_STATUS_REG);

	if (!(data & SDMA1_STATUS_REG__IDLE_MASK))
		gim_err("SDMA1 busy\n");

	data =  read_register(func, mmSRBM_STATUS);

	if (data & (SRBM_STATUS__DRM_RQ_PENDING_MASK |
		SRBM_STATUS__DRM_BUSY_MASK))
		gim_err("DRM busy\n");

	/* TODO, we need to have a way to check if RLC busy.
	 * Confirm new RLC design.
	 */

	if (data & SRBM_STATUS__SEM_BUSY_MASK)
		gim_err("SEM busy\n");

	if (data & SRBM_STATUS__GRBM_RQ_PENDING_MASK)
		gim_err("GRBM busy\n");

	if (data & SRBM_STATUS__VMC_BUSY_MASK)
		gim_err("VMC busy\n");

	data =  read_register(func, mmCP_CPF_STATUS);
	if (data & 0x80000000) {
		gim_err("CP_CPF_STATUS = 0x%08x\n", data);
		if (data & CP_CPF_STATUS__POST_WPTR_GFX_BUSY_MASK) {
			gim_err("    The write pointer has been updated and");
			gim_err("the initiated work is still being processed");
			gim_err("by the GFX pipe\n");
		}

		if (data & CP_CPF_STATUS__HQD_BUSY_MASK) {
			gim_err("   The HQD is busy for any of the following");
			gim_err("reasons: sending a message, fetching data, or");
			gim_err("reorder queues not empty\n");
		}

		if (data & CP_CPF_STATUS__CPF_CMP_BUSY_MASK)
			gim_err("    The Compute part of CPF is Busy.\n");
	}

	check_me_cntl(func->adapt, "GPU dump");
	data =  read_register(func, mmCP_CPF_BUSY_STAT);
	if (data) {
		gim_err("CP_CPF_BUSY_STAT = 0x%08x\n", data);
		if (data & CP_CPF_BUSY_STAT__HQD_WAIT_SEMAPHORE_BUSY_MASK)
			gim_err("    The HQD has a pending Wait semaphore\n");
	}

	gim_err("**** dump gpu status end\n");
}

int wait_for_gfx_idle(struct function *func)
{
	uint32_t time_out = 500*1000; /*500 ms */
	uint32_t time_passed = 0;

	if (func->adapt->enable_preeption)
		return GIM_OK;

	gim_dbg("try to wait for GPU idle\n");

	while (time_passed < time_out) {
		if (gfx_is_idle(func))
			return GIM_OK;
		udelay(100);
		time_passed += 100;
	}
	gim_warn("gfx is not idle after %d usec", time_passed);
	dump_gpu_status(func);
	return GIM_ERROR;
}


static bool is_cmd_complete(struct function *func)
{
	kcl_type_u8 command;
	kcl_type_u8 status;
	struct adapter *adapt = func->adapt;
	uint32_t  pci_status;


	pci_read_config_dword(adapt->pf.pci_dev, 4, &pci_status);
	gim_dbg("Cmd/Status @ 4 = 0x%08x\n", pci_status);

	pci_read_config_byte(adapt->pf.pci_dev,
		adapt->gpuiov.pos + PCI_GPUIOV_CMD_CONTROL, &command);
	pci_read_config_byte(adapt->pf.pci_dev,
		adapt->gpuiov.pos + PCI_GPUIOV_CMD_STATUS, &status);

	pci_read_config_dword(adapt->pf.pci_dev, 4, &pci_status);
	gim_dbg("Cmd/Status @ 4 = 0x%08x GPUIOV Cmd = 0x%0x, \t"
		"Status = 0x%0x\n", pci_status, command, status);

	return ((!(command & CMD_EXECUTE)) && status == 0);
}

#define GPU_STATUS_SIZE 13
static const unsigned int gpu_hang_check_list[GPU_STATUS_SIZE] = {
	mmCP_RB_RPTR,
	mmCP_RB1_RPTR,
	mmCP_RB2_RPTR,
	mmCP_IB1_BUFSZ,
	mmCP_IB2_BUFSZ,

	mmSDMA0_GFX_RB_RPTR,
	mmSDMA1_GFX_RB_RPTR,
	mmSDMA0_GFX_IB_OFFSET,
	mmSDMA1_GFX_IB_OFFSET,

	mmSQ_LB_DATA_ALU_CYCLES,
	mmSQ_LB_DATA_ALU_STALLS,
	mmSQ_LB_DATA_TEX_CYCLES,
	mmSQ_LB_DATA_TEX_STALLS
};

/*
 * Use CP/SDMA RB/IB' RPTR and CP performanace counter to determine if the GPU
 * is really hung or not.
 *
 * mmSQ_LB_DATA_ALU_CYCLES:  when the CU(SQ) perform an algorithm operation,
 *                           this counter increases.
 * mmSQ_LB_DATA_ALU_STALLS:  this counter tell us how many CU(SQ)  instructions
 *                           are in a waiting state, pending to be executed.
 * mmSQ_LB_DATA_TEX_CYCLES:  when the CU(SQ)  perform an memory access
 *                           operation, this counter increases.
 * mmSQ_LB_DATA_TEX_STALLS:  this counter tell us how many CU(SQ)  instructions
 *                           are pending for memory access operation(may be
 *                           waiting for the MC ACK)
 *
 * if the CP/SDMA  RB/IB's RPTR and the above 4 counter does not moves on for
 * 100 ms, we can solid believe that the GFX IP is really hung.
 */
static int is_gpu_hang(struct adapter *adapt)
{

	int i = 0;
	int reg_index = 0;
	unsigned int gpu_hang_check_list_data[GPU_STATUS_SIZE];
	unsigned char status = 255;
	unsigned char command = 255;
	int ret = 1;
	struct timespec diff;
	struct timespec start;
	struct pci_dev *pdev = adapt->pf.pci_dev;
	struct function *pf = &adapt->pf;
	int gpuiov_pos = adapt->gpuiov.pos;

	unsigned int  data = SQ_LB_CTR_CTRL__START_MASK |
				SQ_LB_CTR_CTRL__CLEAR_MASK;

	getnstimeofday(&start);

	/* Start/clear perf counters. */
	write_register(pf, mmSQ_LB_CTR_CTRL, data);

	for (i = 0; i < GPU_STATUS_SIZE; i++)
		gpu_hang_check_list_data[i] =
				read_register(pf, gpu_hang_check_list[i]);

	/* wait up to 10 ms */
	for (i = 0; i < 100; i++) {
		msleep(10);

		pci_read_config_byte(pdev,
				gpuiov_pos + PCI_GPUIOV_CMD_CONTROL,
				&command);

		pci_read_config_byte(pdev,
				gpuiov_pos + PCI_GPUIOV_CMD_STATUS,
				&status);

		if ((!(command & CMD_EXECUTE)) && (status == 0)) {
			ret = 0;
			break;
		}

		data = SQ_LB_CTR_CTRL__START_MASK|SQ_LB_CTR_CTRL__LOAD_MASK;

		write_register(pf, mmSQ_LB_CTR_CTRL, data);

		for (reg_index = 0; reg_index < GPU_STATUS_SIZE; reg_index++) {
			if (gpu_hang_check_list_data[reg_index] !=
			read_register(pf, gpu_hang_check_list[reg_index])) {
				/* rptr or CP performance counter moves on,
				 * it is not hung
				 */
				ret = 0;
				break;
			}
		}

		if (ret == 0)
			break;

		diff = time_elapsed(&start);
		if (diff.tv_nsec > 100*1000000)
			/* it has waited for 100 ms. */
			break;
	}

	write_register(pf, mmSQ_LB_CTR_CTRL, 0);

	return ret;
}

static int wait_cmd_complete(struct function *func)
{
	struct timespec start_time;
	struct timespec delta_time;

	uint32_t time_out = 1; /* 1 seconds */
	kcl_type_u8 command;
	kcl_type_u8 gpu_status;
	uint32_t data;
	uint32_t status;
	struct adapter *adapt = func->adapt;

	getnstimeofday(&start_time);

	pci_read_config_dword(adapt->pf.pci_dev, 4, &status);
	gim_dbg("Cmd/Status @ 4 = 0x%08x\n", status);
	data = read_register(func, mmRLC_GPM_STAT);
	gim_dbg("RLC_GPM_STAT = 0x%08x before waiting\n", data);
	pci_read_config_byte(adapt->pf.pci_dev,
			adapt->gpuiov.pos + PCI_GPUIOV_CMD_CONTROL,
			&command);
	if ((command & 7) == 7)
		time_out = 3;  /* Allow 3 second timeout for GPU_INIT */

	delta_time = time_elapsed(&start_time);
	while (delta_time.tv_sec < time_out) {
		if (is_cmd_complete(func)) {
			gim_dbg("Command 0x%02x took %ld.%09ld to complete",
				command, delta_time.tv_sec, delta_time.tv_nsec);
			command &= 7;
			return GIM_OK;
		}

		delta_time = time_elapsed(&start_time);
		if (delta_time.tv_nsec > 100*1000000) {
			if (is_gpu_hang(adapt))
				break;
		}

		udelay(10);
	}

	gim_err(" wait_cmd_complete -- time out after %ld.%09ld sec\n",
		delta_time.tv_sec, delta_time.tv_nsec);
	pci_read_config_byte(adapt->pf.pci_dev,
			adapt->gpuiov.pos + PCI_GPUIOV_CMD_CONTROL,
			&command);
	pci_read_config_byte(adapt->pf.pci_dev,
			adapt->gpuiov.pos + PCI_GPUIOV_CMD_STATUS,
			&gpu_status);
	gim_err("  Cmd = 0x%0x, Status = 0x%0x\n", command, gpu_status);

	dump_gpu_status(func);
	return GIM_ERROR;
}

int init_vf(struct function *func)
{
	struct adapter *adapt = func->adapt;
	int  is_pf = (&adapt->pf == func);
	uint32_t data;
	int ret;
	uint32_t status;

	if (!func->is_scheduled
		 && (adapt->sched_opt == SCHEDULER__PREDICT_PERF)) {
		gim_info("drop init_vf, since VF%d is not scheduled\n",
			func->func_id);
		return 0;
	}

	pci_read_config_dword(adapt->pf.pci_dev, 4, &status);
	gim_dbg("Cmd/Status @ 4 = 0x%08x\n", status);
	data = pf_read_register(adapt, mmRLC_CP_SCHEDULERS);
	gim_dbg("RLC_CP_SCHEDULERS = 0x%08x after INIT_GPU\n", data);

	gim_dbg("submit INIT_GPU command to ADP %d, %s %x\n",
		func->adapt->adp_id, is_pf ? "PF" : "VF", func->func_id);
	set_gpuiov_command(adapt->pf.pci_dev,
		&adapt->gpuiov,
		INIT_GPU, is_pf ? 0 : VF_ID(func->func_id), 0x0);
	ret =  wait_cmd_complete(func);

	data = pf_read_register(adapt, mmRLC_CP_SCHEDULERS);
	gim_dbg("RLC_CP_SCHEDULERS = 0x%08x after INIT_GPU\n", data);
	return ret;
}

int run_vf(struct function *func)
{
	struct adapter *adapt = func->adapt;
	int  is_pf = (&adapt->pf == func);
	int  ret;
	uint32_t status;

	if (!func->is_scheduled
		&& (adapt->sched_opt == SCHEDULER__PREDICT_PERF)) {
		gim_info("drop run_vf, since VF%d is not scheduled\n",
			func->func_id);
		return 0;
	}

	pci_read_config_dword(adapt->pf.pci_dev, 4, &status);
	gim_dbg("Cmd/Status @ 4 = 0x%08x\n", status);

	gim_dbg("submit RUN_GPU command to ADP %d, %s %x\n",
		func->adapt->adp_id, is_pf ? "PF" : "VF", func->func_id);
	set_gpuiov_command(adapt->pf.pci_dev,
			&adapt->gpuiov,
			RUN_GPU, is_pf ? 0 : VF_ID(func->func_id), 0x0);
	ret = wait_cmd_complete(func);

	return ret;
}

int idle_vf(struct function *func)
{
	struct adapter *adapt = func->adapt;
	int  is_pf = (&adapt->pf == func);
	uint32_t status;

	if (!func->is_scheduled
		&& (adapt->sched_opt == SCHEDULER__PREDICT_PERF)) {
		gim_info("drop idle_vf, since VF%d is not scheduled\n",
			func->func_id);
		return 0;
	}

	pci_read_config_dword(adapt->pf.pci_dev, 4, &status);
	gim_dbg("Cmd/Status @ 4 = 0x%08x\n", status);

	func->rptr = read_register(func, mmCP_RB_RPTR);
	func->wptr = read_register(func, mmCP_RB_WPTR);
	gim_dbg("submit IDLE_GPU command to ADP %d, %s %x,\t"
		"RPTR = 0x%08x, WPTR = 0x%08x\n",
		func->adapt->adp_id,
		is_pf ? "PF" : "VF", func->func_id,
		func->rptr,
		func->wptr);

	set_gpuiov_command(adapt->pf.pci_dev,
			&adapt->gpuiov,
			IDLE_GPU,
			is_pf ? 0 : VF_ID(func->func_id), 0x0);
	return wait_cmd_complete(func);
}

int save_vf(struct function *func)
{
	struct adapter *adapt = func->adapt;
	int  is_pf = (&adapt->pf == func);
	int ret;
	uint32_t status;

	if (!func->is_scheduled
		&& (adapt->sched_opt == SCHEDULER__PREDICT_PERF)) {
		gim_info("drop save_vf, since VF%d is not scheduled\n",
			func->func_id);
		return 0;
	}

	pci_read_config_dword(adapt->pf.pci_dev, 4, &status);
	gim_dbg("Cmd/Status @ 4 = 0x%08x\n", status);

	gim_dbg("submit SAVE_GPU_STATE command to ADP %d, %s %x\n",
		func->adapt->adp_id, is_pf ? "PF" : "VF", func->func_id);
	set_gpuiov_command(adapt->pf.pci_dev, &adapt->gpuiov,
		SAVE_GPU_STATE, is_pf ? 0 : VF_ID(func->func_id), 0x0);
	ret =  wait_cmd_complete(func);

	return ret;
}

int load_vf(struct function *func)
{
	struct adapter *adapt = func->adapt;
	int  is_pf = (&adapt->pf == func);
	int  ret;
	uint32_t status;

	if (!func->is_scheduled
		&& (adapt->sched_opt == SCHEDULER__PREDICT_PERF)) {
		gim_info("drop load_vf, since VF%d is not scheduled\n",
			func->func_id);
		return 0;
	}

	pci_read_config_dword(adapt->pf.pci_dev, 4, &status);
	gim_dbg("Cmd/Status @ 4 = 0x%08x\n", status);

	gim_dbg("submit LOAD_GPU_STATE command to ADP %d, %s %x\n",
		func->adapt->adp_id, is_pf ? "PF" : "VF", func->func_id);
	set_gpuiov_command(adapt->pf.pci_dev,
			&adapt->gpuiov,
			LOAD_GPU_STATE,
			is_pf ? 0 : VF_ID(func->func_id),
			0x0);
	ret = wait_cmd_complete(func);

	return ret;
}


/*
 * Switch to PF using the register init state that is saved in the PF CSA.
 *
 */
int switch_to_pf(struct adapter *adapt)
{
	struct function *fn;

	if (adapt == NULL) {
		gim_err("Invalid struct adapter pointer\n");
		return -1;
	}

	fn = &adapt->pf;

	/* Assumes that the PF CSA has already been initialized
	 * by init_register_init_state()
	 */
	if (load_vf(fn)) {
		gim_err("Failed to LOAD PF\n");
		return -1;
	}

	if (run_vf(fn)) {
		gim_err("Failed to RUN PF\n");
		return -1;
	}

	return 0;
}

/*
 * It is assumed that the PF is currently running
 * (can add safety check if needed for debug)
 * Idle the PF and leave the GPU in an Idle/Saved state.
 */
int idle_pf(struct adapter *adapt)
{
	struct function *fn;

	if (adapt == NULL) {
		gim_err("Invalid struct adapter pointer\n");
		return -1;
	}

	fn = &adapt->pf;

	if (idle_vf(fn)) {
		gim_err("Failed to IDLE PF\n");
		return -1;
	}

	if (save_vf(fn)) {
		gim_err("Failed to SAVE PF\n");
		return -1;
	}

	return 0;
}

int load_rlcv_state(struct function *vf)
{
	struct adapter *adapt = vf->adapt;
	int  ret, is_pf = (&adapt->pf == vf);

	if (!vf->is_scheduled
		&& (adapt->sched_opt == SCHEDULER__PREDICT_PERF)) {
		gim_err("Prevent to load_rlcv_state. VF%d is not scheduled\n",
			vf->func_id);
		return -1;
	}

	set_gpuiov_command(adapt->pf.pci_dev,
			&adapt->gpuiov,
			LOAD_RLCV_STATE,
			is_pf ? 0 : VF_ID(vf->func_id),
			0x0);
	ret = wait_cmd_complete(vf);

	if (ret == GIM_OK)
		gim_info("LOAD_RLCV_STATE done on %s%d",
			is_pf?"PF":"VF", vf->func_id);
	else
		gim_info("LOAD_RLCV_STATE failed on %s%d",
			is_pf?"PF":"VF", vf->func_id);

	return ret;
}

int save_rlcv_state(struct function *vf)
{
	struct adapter *adapt = vf->adapt;
	int  ret, is_pf = (&adapt->pf == vf);

	if (!vf->is_scheduled
		&& (adapt->sched_opt == SCHEDULER__PREDICT_PERF)) {
		gim_err("Prevent to save_rlcv_state. VF%d is not scheduled\n",
			vf->func_id);
		return -1;
	}

	set_gpuiov_command(adapt->pf.pci_dev,
			&adapt->gpuiov,
			SAVE_RLCV_STATE,
			is_pf ? 0 : VF_ID(vf->func_id),
			0x0);
	ret = wait_cmd_complete(vf);

	if (ret == GIM_OK)
		gim_info("SAVE_RLCV_STATE done on %s%d", is_pf?"PF":"VF",
			vf->func_id);
	else
		gim_info("SAVE_RLCV_STATE failed on %s%d", is_pf?"PF":"VF",
			vf->func_id);

	return ret;
}

int reset_rlcv_state_machine(struct adapter *adapt)
{
	struct function *vf = adapt->curr_running_func->func;

	if (adapt->sched_opt == SCHEDULER__PREDICT_PERF)
		vf->is_scheduled = 1; /* PF always can be scheduled*/

	if (init_vf(vf) != 0) {
		gim_info("init vf, Failed to restore RLCV state machine\n");
		return -1;
	}

	if (run_vf(vf) != 0) {
		gim_info("run vf, Failed to restore RLCV state machine\n");
		return -1;
	}

	if (idle_vf(vf) != 0) {
		gim_info("idle vf, Failed to restore RLCV state machine\n");
		return -1;
	}

	if (save_vf(vf) != 0) {
		gim_info("save vf, Failed to restore RLCV state machine\n");
		return -1;
	}

	return 0;
}

int world_switch_divided(struct function *func, struct function *next_func)
{
	int ret;
	int valid;

	ret = GIM_OK;
	func->flr_reason_code = FLR_REASON_CLEAR;
	next_func->flr_reason_code = FLR_REASON_CLEAR;

	valid = pci_validate_devices(device_list);
	if (!valid) {
		gim_err("Device disappeared before world switch\n");
		return GIM_ERROR;
	}

	if ((func->is_scheduled
		&& (func->adapt->sched_opt == SCHEDULER__PREDICT_PERF))
		|| (func->adapt->sched_opt == SCHEDULER__ROUND_ROBIN)) {
		ret = idle_vf(func);

		if (ret != GIM_OK) {
			func->flr_reason_code = FLR_REASON_FAILED_IDLE;
			func->needs_flr = 1;
			return ret;
		}

		ret = save_vf(func);

		if (ret != GIM_OK) {
			func->flr_reason_code = FLR_REASON_FAILED_SAVE;
			func->needs_flr = 1;
			return ret;
		}
	}

	if ((next_func->is_scheduled
		&& (next_func->adapt->sched_opt == SCHEDULER__PREDICT_PERF))
		|| (next_func->adapt->sched_opt == SCHEDULER__ROUND_ROBIN)) {
		ret = load_vf(next_func);

		if (ret != GIM_OK) {
			gim_err("LOAD_VF failed\n");
			next_func->flr_reason_code = FLR_REASON_FAILED_LOAD;
			next_func->needs_flr = 1;
			return ret;
		}

		ret = run_vf(next_func);

		if (ret != GIM_OK) {
			next_func->flr_reason_code = FLR_REASON_FAILED_RUN;
			next_func->needs_flr = 1;
		}
		next_func->scheduled_num++;
	}

	valid = pci_validate_devices(device_list);
	if (!valid) {
		gim_err("Device disappeared after world switch\n");
		return GIM_ERROR;
	}

	return ret;
}


int switch_vfs(struct function *func, struct function *next_func)
{
	if (world_switch_divided(func, next_func) != GIM_OK)
		return GIM_ERROR;

	return GIM_OK;
}

int stop_current_vf(struct adapter *adapt)
{
	struct function_list_node *curr;

	curr = adapt->curr_running_func;

	if (curr == NULL) {
		gim_warn("runlist is empty, ignore stop\n");
		return GIM_OK;
	}

	if ((curr->func->is_scheduled
		 && (adapt->sched_opt == SCHEDULER__PREDICT_PERF))
		|| (adapt->sched_opt == SCHEDULER__ROUND_ROBIN)) {
		if (idle_vf(curr->func)) {
			if (gim_sched_reset(adapt, curr->func, NULL,
					FLR_REASON_FAILED_IDLE) != 0) {
				gim_err("VF%d FLR failed!\n",
					curr->func->func_id);
				return -1;
			}
		}

		if (save_vf(curr->func)) {
			if (gim_sched_reset(adapt, curr->func, NULL,
					FLR_REASON_FAILED_SAVE) != 0) {
				gim_err("VF%d FLR failed!\n",
					curr->func->func_id);
				return -1;
			}
		}

	} else {
		gim_warn("runlist is not schedued, ignore stop\n");
	}

	return GIM_OK;
}

int alloc_new_vf(struct function *func, int dom_id, int qemu_pid,
		int fb_size, int sched_level)
{
	struct adapter *adapt;
	struct partition *partition;
	kcl_type_u64 vf_fb_size;
	kcl_type_u64 mini_vf_fb_size;
	struct resource *res;
	struct slot_list_node *slot = NULL;
	kcl_type_u32 sav_res_list_size = 0;

	gim_info("alloc_new_vf with pid %d", qemu_pid);
	/* check if func is already been assigned or not */
	if (func->is_available == 0)
		gim_warn("Fn is already in use but ignore it temporarily\n");

	adapt = func->adapt;
	load_vbios(func);

	write_register(&adapt->pf,
			mmRLC_GPM_SCRATCH_ADDR,
			REGISTER_RESTORE_LIST_SIZE);


	sav_res_list_size = read_register(&adapt->pf,
						mmRLC_GPM_SCRATCH_DATA);

	gim_info(" sav_res_list_size = %d\n", sav_res_list_size);

	if (func->reg_sav_res_data) {
		vfree(func->reg_sav_res_data);
		func->reg_sav_res_data = NULL;
	}

	if (func->reg_sav_res_offset) {
		vfree(func->reg_sav_res_offset);
		func->reg_sav_res_offset = NULL;
	}

	func->reg_sav_res_data =
		vmalloc(sizeof(uint32_t) * sav_res_list_size);
	func->reg_sav_res_offset =
		vmalloc(sizeof(uint32_t) * sav_res_list_size);
	func->sav_res_list_size = sav_res_list_size;

	/* allocate fb partition
	 * ignor fb_size if it is FIX schema
	 */
	if (get_fb_partition_option() == FB_PARTITION__DYNAMIC)	{
		/* frame buffer size must meet the requirement */
		res = &adapt->vfs[0].pci_dev->resource[BAR__FRAME_BUFFER];
		mini_vf_fb_size = TO_MBYTES(res->end - res->start + 1);
		gim_info("FB.start = 0x%08llx, FB.size = 0x%llx\n",
			res->start, res->end);
		mutex_lock(&adapt->fb_dynamic_alloc_mutex);
		if (fb_size && fb_size > mini_vf_fb_size
			&& fb_size <= adapt->max_fb_slot)
			vf_fb_size = fb_size;
		else
			/* TODO: GIM should return error */
			vf_fb_size = mini_vf_fb_size;

		vf_fb_size = rounddown(vf_fb_size, FB__SIZE_IN_16M);

		gim_info(" vf_fb_size = %lld\n", vf_fb_size);

		slot = find_suitable_slot(adapt, vf_fb_size);
		if (!slot) {
			gim_err("Could not allocate frame buffer for VF!\n");
			mutex_unlock(&adapt->fb_dynamic_alloc_mutex);
			return GIM_ERROR;
		}

		gim_info("slot->slot.size = %lld\n", slot->slot.size);

		partition = &adapt->partition[func->func_id+1];
		partition->slot.base = slot->slot.base;
		partition->slot.size = vf_fb_size;
		gim_info("AMD GIM alloc_new_vf: func id %d base = %llx \t"
			"size= %llx\n", func->func_id, partition->slot.base,
			partition->slot.size);

		set_gpuiov_vf_frame_buffer(adapt->pf.pci_dev,
					&adapt->gpuiov,
					func->func_id,
					partition->slot.size,
					partition->slot.base);

		/* update empty slot list */
		slot->slot.base += vf_fb_size;
		slot->slot.size -= vf_fb_size; /* size could be 0 here */

		if (slot->slot.size == 0)
			delete_slot(adapt, slot);
		else
			sort_slot_list(adapt, slot);

		/* update max_fb_slot */
		slot = adapt->empty_slot_list;
		if (slot) {
			/* find the last one */
			while (slot->next)
				slot = slot->next;
			adapt->max_fb_slot = slot->slot.size;
		} else
			adapt->max_fb_slot = 0;
		mutex_unlock(&adapt->fb_dynamic_alloc_mutex);
	}

	/* set sched level */
	if (sched_level >= 0 && sched_level <= 3) {
		func->sched_level = sched_level;
	} else {
		/* set to normal: 6ms */
		func->sched_level = 1;
	}

	gim_info("Defer adding VF to runlist until Mailbox work queue");
	/* VF is started in work qeueu
	 * this is moved to work qeueu
	 */

	func->is_available = 0;
	func->dom_id = dom_id;
	func->qemu_pid = qemu_pid;
	func->msg_len = 0;
	func->scheduled_num = 1;

	load_vbios(func);

	adapt->available_vfs--;
	gim_info("end of alloc_new_vf");
	return GIM_OK;
}

struct function_list_node *add_func_to_run_list(struct function *func)
{
	struct adapter *adapt;
	struct function_list_node *new_node;
	struct function_list_node *curr_node;

	if (func == NULL || func->adapt == NULL)
		return NULL;

	adapt = func->adapt;
	/* increase the module reference count*/
	if(adapt->sched_opt != SCHEDULER__PREDICT_PERF)
		try_module_get(THIS_MODULE);

	new_node = is_on_run_list(func);

	if (new_node == NULL) {
		gim_info("Add VF%d to the runlist\n", func->func_id);

		new_node = alloc_fn_list_node(adapt);
		if (new_node) {
			new_node->func = func;

			/* add to running list */
			if (adapt->curr_running_func == NULL) {
				/* Runlist is empty */
				new_node->next = new_node;
				new_node->pre = new_node;
			} else {
				/* Runlist is not empty.
				 * Add this vf into the runlist linked list.
				 */
				curr_node = adapt->curr_running_func;

				new_node->next = curr_node->next;
				curr_node->next = new_node;
				new_node->next->pre = new_node;
				new_node->pre = curr_node;
			}

			/* update current running vf to the new node */
			adapt->curr_running_func = new_node;
		} else {
			gim_err("Failed to allocate a new node\n");
			if(adapt->sched_opt != SCHEDULER__PREDICT_PERF)
				module_put(THIS_MODULE);
		}
	} else {
		gim_warn("VF%d is already on the run_list.\n", func->func_id);
		if(adapt->sched_opt != SCHEDULER__PREDICT_PERF)
			module_put(THIS_MODULE);
	}

	return new_node;
}

/*
 * Remove the specified function from the runlist
 *
 * The run_list is a doubly linked list of Nodes.  Each node contains a Function
 * The Functions are part of the adapt structure but the nodes are dynamically
 * allocated and freed as they are needed.
 * (As an optimization the node could be created as an array of 16 like the
 * functions but that requires changes to init_vf as well).
 *
 * Don't grab a spn lock in this function as it is assumed that the caller
 * already has the lock since it is working on the run_list.
 *
 */
void remove_from_run_list(struct function *func)
{
	struct function_list_node *node;
	struct function_list_node *start_node;
	struct adapter *adapt = func->adapt;

	gim_info("Remove struct function 0x%p from runlist\n", func);
	if (func == NULL) {
		gim_err("NULL function specified");
		return;
	}
	gim_info("  removing VF%d\n", func->func_id);

	node = start_node = adapt->curr_running_func;

	if (node == NULL) {
		gim_warn("run_list is null;"
			"Can't remove VF%d from the run_list",
			func->func_id);
		return;
	}

	/*Is it first function on the list? */
	if (node->func == func) {
		/* Only 1 on list */
		if (node->next == node) {
			gim_info("VF%d is the only function on running list\n",
				func->func_id);

			stop_current_vf(adapt);
			adapt->curr_running_func = NULL;
		} else {
			gim_info("There are 2 or more functions on the list");
			gim_info("set new head node (VF%d) = 0x%p\n",
				node->next->func->func_id, node->next);
			gim_info("new func = 0x%p\n", node->next->func);
			switch_vfs(node->func, node->next->func);
			adapt->curr_running_func = node->next;
		}
		/* Remove myself from the list by removing myself from
		 * between the previous and next nodes.
		 */
		node->pre->next = node->next;
		node->next->pre = node->pre;
		gim_info(" free node 0x%p\n", node);
		free_fn_list_node(adapt, node);
		/* decrease the module reference count*/
		if(adapt->sched_opt != SCHEDULER__PREDICT_PERF)
			module_put(THIS_MODULE);
	} else {
		/* Its not the first fucntion on the list, therefore there are
		 * more than 1 nodes on the list. Need to scan the list to find
		 * the function
		 */
		gim_info("  Not the first function on the run_list");
		node = start_node->next;

		while (node->func != func && node != start_node)
			node = node->next;

		/* Entire list has been scanned, check if we found it. */
		if (node->func == func) {
			/* Check the one we stopped at */
			gim_info("Found the node for VF%d, now remove node",
				func->func_id);
			node->pre->next = node->next;
			node->next->pre = node->pre;
			gim_info("  Freeing node %p\n", node);
			free_fn_list_node(adapt, node);
			/* decrease the module reference count*/
			if(adapt->sched_opt != SCHEDULER__PREDICT_PERF)
				module_put(THIS_MODULE);
		} else {
			gim_info("VF%d not found on the running list",
				func->func_id);
		}
	}
}

void mark_func_scheduled(struct function *func)
{
	/* increase the module reference count*/
	try_module_get(THIS_MODULE);
	if (func->is_scheduled)
		gim_warn("Mark VF%d is scheduled again\n", func->func_id);
	else
		gim_info("Mark VF%d is scheduled\n", func->func_id);
	func->is_scheduled = 1;
}

void mark_func_not_scheduled(struct function *func)
{
	if (!func->is_scheduled)
		gim_warn("Mark VF%d is not scheduled again\n", func->func_id);
	else
		gim_info("Mark VF%d is not scheduled\n", func->func_id);
	func->is_scheduled = 0;
	/* decrease the module reference count*/
	module_put(THIS_MODULE);
}

/*
 * clear_vf_fb() - Clear the VRAM for a virtual function
 * @adapt: point to amdgpu_device
 * @func: VF need to rmove
 *
 * This function is a wrapper for init_vf_fb().  It ensures
 * that all of the conditions are in place. so that the
 * init_vf_fb() function can be called.
 *
 * Prerequsites:
 * - VF to reset has been removed from the run list.
 * - current function is at load/run
 * - If there are no functions on the runlist then the VF to clear
 *   must be at load/run state.
 * - If there are other functions on the runlist then "func" can be
 *   at either load/run or idle/save state.
 *
 * Requirements to call init_vf_fb()
 * - scheduler must be stopped
 * - Switch to PF to do the actual clear.
 * - Switch away from PF after the clear has been completed.
 *
 */
static int clear_vf_fb(struct adapter *adapt, struct function *func)
{
	int ret = 0;

	gim_info("clear_vf_fb() - Clear FB for VF%d\n", func->func_id);


	/* Switch from curr function to PF.  The current function is at
	 * the head of the runlist. If the runlist is empty then "func" was
	 * the last running function and is still at load/run state
	 */
	if (adapt->curr_running_func != NULL) {
		gim_info("Switch from curr VF%d to PF\n",
			adapt->curr_running_func->func->func_id);
		ret = switch_vfs(adapt->curr_running_func->func,
				&adapt->pf);
	} else {
		gim_info("Switch from last VF%d to PF\n", func->func_id);
		ret = switch_vfs(func, &adapt->pf);
	}

	/* Clear the VRAM memory */
	if (get_fb_clear_option() == 1)
		init_vf_fb(adapt, func);
	else
		gim_info("Skipping clear of VF%d FB\n", func->func_id);

	/* If there is a function on the runlist then switch back to it.
	 * If the runlist is empty then there is nothing to switch back to.
	 * I can't switch back to the function that just had its VRAM cleared
	 * as this would cause a GPU hang because I just blew away its memory
	 * contents.
	 */
	if (adapt->curr_running_func != NULL) {
		gim_info("Switch PF to current VF%d at head of run list\n",
			adapt->curr_running_func->func->func_id);
		ret = switch_vfs(&adapt->pf,
			adapt->curr_running_func->func);
	} else {
		/* There is nothing on the runlist to switch to
		 * I can't switch back to the freed function after
		 * I have blown away VRAM
		 */
		gim_info("Run list is empty. PF is still current function\n");
		gim_info("PF can't switch back to VF%d after clearing FB\n",
			func->func_id);
		idle_pf(adapt);
		gim_info("PF has been idled\n");
	}
	return ret;
}


int free_vf(struct function *func)
{
	struct slot_list_node *slot;
	struct adapter *adapt = func->adapt;

	gim_info("free VF%d", func->func_id);
	clear_vf_fb(adapt, func);

	if (get_fb_partition_option() == FB_PARTITION__DYNAMIC) {
		slot = vmalloc(sizeof(struct slot_list_node));
		slot->slot.base = func->fb_partition->slot.base;
		slot->slot.size = func->fb_partition->slot.size;
		slot->next = NULL;

		mutex_lock(&adapt->fb_dynamic_alloc_mutex);
		add_and_merge_slot(adapt, slot);

		slot = adapt->empty_slot_list;
		while (slot->next)
			slot = slot->next; /* find the last one */
		adapt->max_fb_slot = slot->slot.size;
		mutex_unlock(&adapt->fb_dynamic_alloc_mutex);
	}

	func->is_available = 1;
	adapt->available_vfs++;
	return GIM_OK;
}


void world_switch(struct adapter *adapt)
{
	int retVal = 0;

	struct function_list_node *next_node;
	struct function *p_current_func;
	struct function *p_next_func;
	struct function *func_to_clear;
	struct function_list_node *curr;

	gim_dbg("Do world switch - ");
	if (gim_get_log_level() > 3) {
		dump_runlist(adapt);
		gim_info("\n");
	}

	++in_world_switch;
	mutex_lock(&adapt->curr_running_func_mutex);

	if (adapt->curr_running_func != NULL) {
		if (!adapt->schedler_running) {
			gim_info("World switch has been called but scheduler;"
				" is not running\n");
			gim_info(" - This is most likely due to a new VF;"
				" requesting Exclusive access mode\n");
		}

		/* lock to protect the current running function
		 * more than one VFs or self switch
		 * scheduler is not stopped
		 */
		if ((adapt->curr_running_func
		     != adapt->curr_running_func->next
		     ||	adapt->switch_to_itself)
		     && adapt->schedler_running) {
			next_node = adapt->curr_running_func->next;

			curr = adapt->curr_running_func;
			p_current_func = curr->func;
			p_next_func = curr->next->func;

			/*
			 * gim_sched_reset
			 * If only 1 func on run_list then current at load/run.
			 * If multiple func on run_list then the one that wasn't
			 * reset is at load/run (either curr or next).
			 * The reset one is at idle/save
			 *
			 */
			retVal = switch_vfs(p_current_func, p_next_func);
			if (retVal != GIM_OK) {
				gim_err("Schedule VF%d to VF%d failed;"
					"Failure reason is %d, try to reset\n",
					p_current_func->func_id,
					p_next_func->func_id,
					p_current_func->flr_reason_code +
					p_next_func->flr_reason_code);

				gim_sched_reset(adapt, p_current_func,
					p_next_func,
					p_current_func->flr_reason_code
					+ p_next_func->flr_reason_code);

				if (p_current_func->needs_flr) {
					p_current_func->needs_flr = 0;
					/* move "next" to the head of
					 * the runlist
					*/
					if ((adapt->sched_opt
						 == SCHEDULER__PREDICT_PERF))
						mark_func_not_scheduled(
							p_current_func);
					else
						remove_from_run_list(
							p_current_func);
					func_to_clear = p_current_func;
				} else {
					p_next_func->needs_flr = 0;
					/* This leaves "curr" at the head of
					 * the runlist
					 */
					if ((adapt->sched_opt
						 == SCHEDULER__PREDICT_PERF))
						mark_func_not_scheduled(
								p_next_func);
					else
						remove_from_run_list(
								p_next_func);
					func_to_clear = p_next_func;
				}
				/*
				 * The function that was left at load/run is
				 * now at the head of the runlist The function
				 * that is to have its VRAM cleared has been
				 * removed from the runlist.
				 */
				clear_vf_fb(adapt, func_to_clear);
			} else
				adapt->curr_running_func = next_node;

			if (adapt->curr_running_func != NULL)
				resume_scheduler(adapt);
		}
	}

	/* since timer may have sync issues with allocVF/freeVF,
	 * move unlock to here
	 */
	mutex_unlock(&adapt->curr_running_func_mutex);
	--in_world_switch;
}


struct function *get_available_VF(struct adapter *adapt)
{
	struct function *func = NULL;

	if (adapt != NULL) {
		uint32_t i = 0;

		for (i = 0; i < adapt->total_vfs; ++i) {
			if (adapt->vfs[i].is_available
			&& adapt->vfs[i].func_id < adapt->enabled_vfs) {
				func = &adapt->vfs[i];
				break;
			}
		}
	}
	return func;
}

struct function *get_vf(struct adapter *adapt, uint32_t vf_id)
{
	struct function *func = NULL;

	if (adapt != NULL) {
		uint32_t i = 0;

		for (i = 0; i < adapt->total_vfs; ++i) {
			if (adapt->vfs[i].func_id == vf_id) {
				func = &adapt->vfs[i];
				break;
			}
		}
	}
	return func;
}

/*
 * This function starts the ball rolling with a world switch.
 * In secure mode this function will send a signal to QEMU which will allow QEMU
 * to finish any MMIO accesses that are in-flight.  Once the activity is flushed
 * QEMU will call back to GIM to do the actual world switch. QEMU is responsible
 * for ensuring that an MMIO access goes through to the hardware only when the
 * corresponding VF for this VM is in valid context.  If it is not in context
 * then QEMU will hold the MMIO access until the VF does become into context.
 * This is the most secure mode as QEMU (via pt-graphics.c) will trap any valid
 * (or invalid, ie viral) register accesses.
 *
 * There is also an unsecure model where the VM must be trusted not to make any
 * MMIO accesses either valid or invalid. There is no protection against a VM
 * making a direct MMIO access to another context. To enable unsecure mode the
 * "world_switch()" and "return()" statements at the beginning of this function
 * should be uncommented.
 *
 */
int triger_world_switch(struct adapter *adapt, bool single_switch)
{
	struct function_list_node *curr;

	curr = adapt->curr_running_func;

	gim_dbg("Trigger world switch - ");
	if (gim_get_log_level() > 3) {
		dump_runlist(adapt);
		gim_info("\n");
	}

	++in_triger_world_switch;
	if (curr != NULL
		&& (curr != curr->next || adapt->switch_to_itself)) {
		adapt->single_switch = single_switch;

		gim_dbg("signal QEMU for vf%d to release",
				curr->func->func_id);
		world_switch(adapt);
	}

	--in_triger_world_switch;
	return GIM_OK;
}

int get_hw_fb_setting(struct adapter *adapt, int vf_num,
			uint32_t *fb_start, uint32_t *fb_size)
{
	return get_vf_hw_fb_settings(adapt->pf.pci_dev,
			&adapt->gpuiov,
			vf_num,
			fb_start,
			fb_size);
}

/* interrupt */
int ih_execute(IhRoutine_t ih_routine,
		void *ih_context)
{
	int ret = 0;
	struct adapter *adapt = (struct adapter *)ih_context;

	amd_spin_lock(&adapt->ih_lock);
	ret = ((IhRoutine_t)ih_routine)(ih_context);
	spin_unlock(&adapt->ih_lock);

	return ret;
}

void work_handler(struct work_struct *work)
{
	struct adapter *adapt = container_of(work, struct adapter, normal_work);

	signal_scheduler((void *)adapt);
}

/* interrupt handler */
static irqreturn_t adapter_interrupt_handler(int irq, void *dev_id)
{
	/* Execute the real isr with amdgpuv_device */
	ih_execute((IhRoutine_t)ih_irq_process, dev_id);

	/* TODO: check return value of ih_execute */
	return IRQ_HANDLED;
}

int adapter_interrupt_register(struct adapter *adapt)
{
	/* IRQF_DISABLED was depreted, IRQF_NOBALANCING caused PF can't
	 * recieved interrupt on 3.17.1.
	 */
	if (request_irq(adapt->pf.pci_dev->irq, adapter_interrupt_handler,
		   0, gim_driver_name, (void *)adapt)) {
		gim_err("Cannot register interrupt handler");
		return GIM_ERROR;
	}
	return GIM_OK;
}

void adapter_interrupt_unregister(struct adapter *adapt)
{
	free_irq(adapt->pf.pci_dev->irq, (void *)adapt);
}

struct named_reg_list smc_reg[] = {
	{0, 0, mmGRBM_STATUS, "GRBM_STATUS"},
	{0, 0, mmCP_RB_RPTR,  "CP_RB_RPTR"},
	{0, 0, mmCP_RB_WPTR,  "CP_RB_WPTR"},
	{0, 0, mmCP_RB_OFFSET, "CP_RB_OFFSET"},
	{0, 0, mmCP_MEC_CNTL, "CP_MEC_CNTL"},
	{0, 0, mmCP_ME_CNTL,  "CP_ME_CNTL"},
	{0, 0, mmCP_CPC_IC_BASE_LO, "CP_CPC_IC_BASE_LO"},
	{0, 0, mmCP_CPC_IC_BASE_HI, "CP_CPC_IC_BASE_HI"},
	{0, 0, mmSRBM_STATUS,  "SRBM_STATUS"},
	{0, 0, mmSRBM_STATUS2, "SRBM_STATUS2"},
	{0, 0, mmSDMA0_STATUS_REG, "SDMA0_STATUS_REG"},
	{0, 0, mmSDMA1_STATUS_REG, "SDMA1_STATUS_REG"},
	{0, 0, mmSMC_MESSAGE_0,     "SMC_MESSAGE_0"},
	{0, 0, mmSMC_RESP_0,        "SMC_RESP_0"},
	{mmSMU_IND_INDEX_3, mmSMU_IND_DATA_3, 0x20014, "SMU version (0x20014)"},
	{mmSMU_IND_INDEX_3, mmSMU_IND_DATA_3, ixSMC_PC_A, "SMC_PC_A"},
	{mmSMU_IND_INDEX_3, mmSMU_IND_DATA_3, ixSMC_PC_C, "SMC_PC_C"},
	{mmSMU_IND_INDEX_3, mmSMU_IND_DATA_3, ixSMC_PC_F, "SMC_PC_F"},
	{mmSMU_IND_INDEX_3, mmSMU_IND_DATA_3, ixSMC_PC_D, "SMC_PC_D"},
	{mmSMU_IND_INDEX_3, mmSMU_IND_DATA_3, ixSMC_PC_X, "SMC_PC_X"},
	{mmSMU_IND_INDEX_3, mmSMU_IND_DATA_3, ixSMC_PC_M, "SMC_PC_M"},
	{mmSMU_IND_INDEX_3, mmSMU_IND_DATA_3, ixSMC_PC_W, "SMC_PC_W"},
	{0, 0, mmIH_ACTIVE_FCN_ID,  "IH_ACTIVE_FCN_ID"},
	{0, 0, mmMC_SHARED_ACTIVE_FCN_ID,  "MC_SHARED_ACTIVE_FCN_ID"},
	{0, 0, mmSMU_ACTIVE_FCN_ID, "SMU_ACTIVE_FCN_ID"},
	{0, 0, mmSEM_ACTIVE_FCN_ID, "SEM_ACTIVE_FCN_ID"},
	{0, 0, mmSDMA0_ACTIVE_FCN_ID, "SDMA0_ACTIVE_FCN_ID"},
	{0, 0, mmSDMA1_ACTIVE_FCN_ID, "SDMA1_ACTIVE_FCN_ID"},
	{0, 0, mmRLC_GPU_IOV_ACTIVE_FCN_ID, "RLC_GPU_IOV_ACTIVE_FCN_ID"},
	{0, 0, mmCP_DMA_PIO_DST_ADDR, "CP_DMA_PIO_DST_ADDR"},
	{0, 0, mmCP_DMA_PIO_DST_ADDR_HI, "CP_DMA_PIO_DST_ADDR_HI"},
	{0, 0, mmMC_VM_SYSTEM_APERTURE_HIGH_ADDR,
					"MC_VM_SYSTEM_APERTURE_HIGH_ADDR"},
	{0, 0, mmMC_VM_SYSTEM_APERTURE_LOW_ADDR,
					"MC_VM_SYSTEM_APERTURE_LOW_ADDR"},
	{0, 0, mmMC_VM_FB_LOCATION, "MC_VM_FB_LOCATION"},
	{0, 0, mmMC_VM_FB_OFFSET, "MC_VM_FB_OFFSET"},
	{0, 0, mmMC_VM_FB_SIZE_OFFSET_VF0, "MC_VM_FB_SIZE_OFFSET_VF0"},
	{0, 0, mmMC_VM_FB_SIZE_OFFSET_VF1, "MC_VM_FB_SIZE_OFFSET_VF1"},
	{0, 0, mmRLC_GPU_IOV_UCODE_ADDR, "RLC_GPU_IOV_UCODE_ADDR"},
	{0, 0, mmRLC_GPU_IOV_UCODE_DATA, "RLC_GPU_IOV_UCODE_DATA"},
	{0, 0, mmRLC_GPM_THREAD_RESET,  "RLC_GPM_THREAD_RESET "},
	{0, 0, mmRLC_CNTL,              "RLC_CNTL             "},
	{0, 0, mmRLC_GPM_THREAD_ENABLE, "RLC_GPM_THREAD_ENABLE"},
	{0, 0, mmRLC_GPU_IOV_F32_CNTL,  "RLC_GPU_IOV_F32_CNTL "},
	{0, 0, mmRLC_GPM_STAT,          "RLC_GPM_STAT         "},
	{0, 0, mmRLC_GPU_IOV_VF_ENABLE, "RLC_GPU_IOV_VF_ENABLE"},
	{0, 0, mmRLC_GPU_IOV_CFG_REG1,  "RLC_GPU_IOV_CFG_REG1 "},
	{0, 0, mmRLC_GPU_IOV_CFG_REG2,  "RLC_GPU_IOV_CFG_REG2 "},
	{0, 0, mmRLC_GPU_IOV_CFG_REG6,  "RLC_GPU_IOV_CFG_REG6 "},
	{0, 0, mmRLC_GPU_IOV_ACTIVE_FCN_ID, "RLC_GPU_IOV_ACTIVE_FCN_ID "},
	{0, 0, mmRLC_GPM_VMID_THREAD2, "RLC_GPM_VMID_THREAD2"},
	{0, 0, mmRLC_GPU_IOV_SDMA0_STATUS, "RLC_GPU_IOV_SDMA0_STATUS"},
	{0, 0, mmRLC_GPU_IOV_SDMA1_STATUS, "RLC_GPU_IOV_SDMA1_STATUS"},
	{0, 0, mmRLC_GPU_IOV_SMU_RESPONSE, "RLC_GPU_IOV_SMU_RESPONSE"},
	{0, 0, mmRLC_GPU_IOV_VIRT_RESET_REQ, "RLC_GPU_IOV_VIRT_RESET_REQ"},
	{0, 0, mmRLC_GPU_IOV_RLC_RESPONSE, "RLC_GPU_IOV_RLC_RESPONSE"},
	{0, 0, 0, NULL},
};

void check_smu_int(uint32_t bdf, char *comment)
{
	/* Works for both PF and VF */
	struct function *fn = bdf_to_function(bdf);

	uint32_t  data;
	int i;
	int        this_is_pf = is_pf(bdf);
	int        vfid = 0xff;

	if (!this_is_pf) {
		gim_info("This is not a PF\n");
		if (fn)
			vfid = fn->func_id;
		else
			gim_info("NULL struct function!!!\n");
	}

	gim_info("Check SMU %s\n", comment);
	gim_info("------------\n");
	if (fn == NULL) {
		gim_info("Invalid BDF\n");
		return;
	}

	/* Display the registers listed in the smc_reg table */
	for (i = 0; smc_reg[i].name != NULL && smc_reg[i].offset != 0; ++i) {
		if (smc_reg[i].idx_data != 0) {
			write_register(fn, smc_reg[i].idx_index,
					smc_reg[i].offset);
			data = read_register(fn, smc_reg[i].idx_data);
		} else {
			data = read_register(fn, smc_reg[i].offset);
		}
		if (this_is_pf)
			gim_info("PF - %s = 0x%08x", smc_reg[i].name, data);
		else
			gim_info("VF%d - %s = 0x%08x",
				vfid, smc_reg[i].name, data);
	}

	gim_info("Read the mmCP_DMA_PIO_DST_ADDR to see if it is moving");
	for (i = 0; i < 5; ++i) {
		data = read_register(fn, mmCP_DMA_PIO_DST_ADDR);
		gim_info("CP_DMA_PIO_DST_ADDR = 0x%0x\n", data);
		mdelay(5);
	}


	/* Read the SMC_PC_C (program counter) several tims to
	 * see if it is moving
	 */
	write_register(fn,  mmSMU_IND_INDEX_3, ixSMC_PC_C);
	for (i = 0; i < 5; ++i) {
		data = read_register(fn, mmSMU_IND_DATA_3);
		gim_info("SMC_PC_C = 0x%0x\n", data);
		mdelay(5);
	}
}

void check_smu(uint32_t bdf, char *comment)
{
	check_smu_int (bdf, comment);
}

void dump_ucode(struct adapter *adapt, int addr_register,
		int data_register, char *reg_name)
{
	int i;
	uint32_t  addr[8];
	uint32_t  data[8];

	gim_info("%s uCode\n", reg_name);
	return;
	pf_write_register(adapt, addr_register, 0);
	for (i = 0; i < 8; i++) {
		data[i] = pf_read_register(adapt,  data_register);
		addr[i] = pf_read_register(adapt,  addr_register);
	}
	gim_info("  %04x: %08x %08x %08x %08x\n",
		addr[0], data[0], data[1], data[2], data[3]);
	gim_info("  %04x: %08x %08x %08x %08x\n",
		addr[4], data[4], data[5], data[6], data[7]);
}

void vbios_check(struct adapter *adapt, uint32_t bdf, char *comment)
{
	uint32_t data;
	uint32_t rlc, smu;

	gim_info("State %s\n", comment);

	rlc = pf_read_register(adapt, mmRLC_GPU_IOV_ACTIVE_FCN_ID);
	smu = pf_read_register(adapt, mmSMU_ACTIVE_FCN_ID);

	gim_info("RLC_GPU_IOV_ACTIVE_FCN_ID = %08x,\t"
		"SMU_ACTIVE_FCN_ID = %08x\n", rlc, smu);

	pf_write_register(adapt, mmSMU_IND_INDEX_3, ixSMC_PC_C);
	data = pf_read_register(adapt, mmSMU_IND_DATA_3);

	gim_info("SMC_PC_C = 0x%08x - ", data);
	if (data >= 0x20000)
		gim_info("SMU running from SRAM\n");
	else
		gim_info("SMU running boot-loader\n");

	/* Check RLC */
	data = pf_read_register(adapt, mmRLC_CNTL);
	gim_info("RLC_CNTL = 0x%08x - RLC ", data);
	if (data & 1)
		gim_info("enabled\n");
	else
		gim_info("disabled\n");

	data = pf_read_register(adapt, mmRLC_GPM_THREAD_ENABLE);
	gim_info("RLC_GPM_THREAD_ENABLE = 0x%08x - RLC Threads ", data);
	if ((data & 3) != 3)
		gim_info("not running\n");
	else
		gim_info("running\n");

	data = pf_read_register(adapt, mmRLC_GPU_IOV_F32_CNTL);
	gim_info("RLC_GPU_IOV_F32_CNTL = 0x%08x - RLC F32 ", data);
	if (data & 1)
		gim_info("running\n");
	else
		gim_info("not running\n");


	/* Check SDMA */
	data = pf_read_register(adapt, mmSDMA0_F32_CNTL);
	if (data & 1)
		gim_info("WARNING: SDMA0 is HALTED\n");
	else
		gim_info("SDMA0 is running\n");

	data = pf_read_register(adapt, mmSDMA1_F32_CNTL);
	if (data & 1)
		gim_info("WARNING: SDMA1 is HALTED\n");
	else
		gim_info("SDMA1 is running\n");
}

struct named_reg_list ih_reg_list[] = {
	{0, 0, mmIH_RB_BASE,	"IH_RB_BASE"},
	{0, 0, mmIH_RB_CNTL,	"IH_RB_CNTL"},
	{0, 0, mmIH_CNTL,	        "IH_CNTL"},
	{0, 0, mmIH_DEBUG,	        "IH_DEBUG"},
	{0, 0, mmINTERRUPT_CNTL,	"INTERRUPT_CNTL"},
	{0, 0, mmINTERRUPT_CNTL2,	"INTERRUPT_CNTL2"},
	{0, 0, mmIH_RB_RPTR,	"IH_RB_RPTR"},
	{0, 0, mmIH_RB_WPTR,	"IH_RB_WPTR"},
	{0, 0, mmIH_RB_WPTR_ADDR_LO,	"IH_RB_WPTR_ADDR_LO"},
	{0, 0, mmIH_RB_WPTR_ADDR_HI,	"IH_RB_WPTR_ADDR_HI"},
	{0, 0, mmIH_DOORBELL_RPTR,	"IH_DOORBELL_RPTR"},
	{0, 0, mmIH_ACTIVE_FCN_ID,    "IH_ACTIVE_FCN_ID"},
	{0, 0, mmBIF_DOORBELL_APER_EN,   "BIF_DOORBELL_APER_EN"},
	{0, 0, mmMAILBOX_INDEX,          "MAILBOX_INDEX"},
	{0, 0, mmMAILBOX_MSGBUF_TRN_DW0, "MAILBOX_MSGBUF_TRN_DW0"},
	{0, 0, mmMAILBOX_CONTROL,	       "MAILBOX_CONTROL"},
	{0, 0, mmIH_ACTIVE_FCN_ID, "IH_ACTIVE_FCN_ID"},
	{0, 0, 0, NULL}
};

void read_ih_regs(uint32_t bdf, char *comment)
{
	struct function *fn = bdf_to_function(bdf);
	uint32_t data;
	int i;

	if (comment != NULL)
		gim_info("IH related registers %s for BDF = 0x%0x",
			comment, bdf);

	if (fn == NULL) {
		gim_info("Bad Function pointer");
		return;
	}

	for (i = 0;
		ih_reg_list[i].name != NULL && ih_reg_list[i].offset != 0;
		++i) {
		data = read_register(fn, ih_reg_list[i].offset);
		gim_info("%s = 0x%08x", ih_reg_list[i].name, data);
	}
}

struct named_reg_list pf_reg_list[] = {
	{0, 0, mmIH_RB_BASE,          "IH_RB_BASE       "},
	{0, 0, mmINTERRUPT_CNTL,      "INTERRUPT_CNTL   "},
	{0, 0, mmINTERRUPT_CNTL2,     "INTERRUPT_CNTL2  "},
	{0, 0, mmMAILBOX_INT_CNTL,    "MAILBOX_INT_CNTL "},
	{0, 0, mmIH_CNTL,             "IH_CNTL          "},
	{0, 0, mmIH_DEBUG,            "IH_DEBUG         "},
	{0, 0, mmIH_RB_RPTR,          "IH_RB_RPTR       "},
	{0, 0, mmIH_RB_WPTR,          "IH_RB_WPTR       "},
	{0, 0, mmIH_RB_WPTR_ADDR_LO,  "IH_RB_WPTR_ADDR_LO"},
	{0, 0, mmIH_RB_WPTR_ADDR_HI,  "IH_RB_WPTR_ADDR_HI"},
	{0, 0, mmIH_DOORBELL_RPTR,    "IH_DOORBELL_RPTR"},
	{0, 0, mmBIF_DOORBELL_APER_EN, "BIF_DOORBELL_APER_EN"},
	{0, 0, 0, NULL}
};

struct named_reg_list rlcv_reg_list[] = {
	{0, 0, mmRLC_GPU_IOV_CFG_REG2, "RLC_GPU_IOV_CFG_REG2"},
	{0, 0, mmRLC_GPU_IOV_SDMA0_STATUS, "RLC_GPU_IOV_SDMA0_STATUS"},
	{0, 0, mmRLC_GPU_IOV_SDMA1_STATUS, "RLC_GPU_IOV_SDMA1_STATUS"},
	{0, 0, mmSDMA0_VM_CNTL, "SDMA0_VM_CNTL"},
	{0, 0, mmSDMA1_VM_CNTL, "SDMA1_VM_CNTL"},
	{0, 0, mmRLC_GPU_IOV_ACTIVE_FCN_ID, "RLC_GPU_IOV_ACTIVE_FCN_ID"},
	{0, 0, mmSRBM_CNTL, "SRBM_CNTL"},
	{0, 0, mmCP_ME_CNTL, "CP_ME_CNTL"},
	{0, 0, mmCP_MEC_CNTL, "CP_MEC_CNTL"},
	{0, 0, mmSDMA0_F32_CNTL, "SDMA0_F32_CNTL"},
	{0, 0, mmSDMA1_F32_CNTL, "SDMA1_F32_CNTL"},
	{0, 0, 0, NULL}
};



void save_mmio_regs(uint32_t bdf, uint32_t *buf, struct named_reg_list *list)
{
	struct function *func = bdf_to_function(bdf);
	int      i;

	for (i = 0; list[i].name != NULL; ++i) {
		*buf = read_register(func, list[i].offset);
		gim_info("Save 0x%08x from %s register", *buf, list[i].name);
		++buf;
	}
}
void restore_mmio_regs(uint32_t bdf, uint32_t *buf, struct named_reg_list *list)
{
	struct function *func = bdf_to_function(bdf);
	int      i;

	for (i = 0; list[i].name != NULL; ++i) {
		gim_info("Restore 0x%08x to %s register", *buf, list[i].name);
		write_register(func, list[i].offset, *buf);
		++buf;
	}
}

void compare_mmio_regs(uint32_t *buf1, uint32_t *buf2,
			struct named_reg_list *list)
{
	int      i;

	for (i = 0; list[i].name != NULL; ++i) {
		gim_info("%s \t0x%08x   0x%08x", list[i].name, *buf1, *buf2);

		++buf1;
		++buf2;
	}
}

void check_pf_regs(uint32_t bdf, char *comment)
{
	struct function *fn = bdf_to_function(bdf);
	uint32_t data;
	int i;


	if (!is_pf(bdf))
		return;

	gim_info("PF MMIO registers for bdf 0x%03x", bdf);
	if (comment != NULL)
		gim_info("    %s", comment);


	if (fn == NULL) {
		gim_info("Bad Function pointer");
		return;
	}

	for (i = 0; pf_reg_list[i].name != NULL; ++i) {
		data = read_register(fn, pf_reg_list[i].offset);
		gim_info("%s = 0x%08x", pf_reg_list[i].name, data);
	}
}

void check_rlc_regs(uint32_t bdf, char *comment)
{
	struct function *fn = bdf_to_function(bdf);
	uint32_t data;
	int i;


	if (!is_pf(bdf)) {
		gim_info("0x%0x is not the PF bdf\n", bdf);
		return;
	}

	gim_info("RLC_V registers for bdf 0x%03x", bdf);
	if (comment != NULL)
		gim_info("    %s", comment);

	if (fn == NULL) {
		gim_info("Bad Function pointer");
		return;
	}

	for (i = 0;
	     rlcv_reg_list[i].name != NULL && rlcv_reg_list[i].offset != 0;
	     ++i) {
		data = read_register(fn, rlcv_reg_list[i].offset);
		gim_info("%s = 0x%08x", rlcv_reg_list[i].name, data);
	}
}

void dump_runlist(struct adapter *adapt)
{
	struct function_list_node *start, *node;

	gim_info("curr->");

	if (adapt != NULL) {
		if (adapt->curr_running_func != NULL) {
			start = adapt->curr_running_func;
			node = start->next;
			gim_info("VF%d, scheduled_num = %d\n",
				start->func->func_id,
				start->func->scheduled_num);
			/* while (node->next != start) */
			while (node != start) {
				gim_info("VF%d, scheduled_num = %d\n",
					node->func->func_id,
					node->func->scheduled_num);
				node = node->next;
			}
		} else
			gim_info("NULL");
	} else
		gim_info("NULL");
}

/*
 * Initialize the register 'init_state'.
 *
 * Use the PF Save area in the CSA as a storage location for the register
 * init_state. The ini_state is a known register state that can be used to
 * set the asic registers prior to starting a new VF.
 *
 * There is a bug (?) where the register state is not cleared on a
 * shutdown/init sequence, so the next time that a VF is restarted some of
 * the registers contain garbage that prevents the driver from starting
 * properly.
 */
int init_register_init_state(struct adapter *adapt)
{
	struct function *fn;

	if (adapt == NULL) {
		gim_err("Invalid struct adapter pointer");
		return (-1);
	}

	fn = &adapt->pf;

	if (init_vf(fn)) {
		gim_err("Failed to INIT PF for initial register 'init-state'");
		return (-1);
	}

	if (run_vf(fn)) {
		gim_err("Failed to RUN PF for initial register 'init-state'");
		return (-1);
	}

	if (idle_vf(fn)) {
		gim_err("Failed to IDLE PF for initial register 'init-state'");
		return (-1);
	}

	if (save_vf(fn)) {
		gim_err("Failed to SAVE PF for initial register 'init-state'");
		return (-1);
	}
	return 0;
}

/*
 * Prior to submitting a GPU_INIT for a VF this function should be called
 * to set the register state to a known state.
 * Any failure from this fucntion should trigger a PF FLR.
 *
 * Alternatively it might be possible to call this function on shutdown of a VF
 * in order to improve startup performance.
 */

int load_register_init_state(struct adapter *adapt)
{
	struct function *fn;

	if (adapt == NULL) {
		gim_err("Invalid struct adapter pointer");
		return (-1);
	}

	fn = &adapt->pf;

	if (load_vf(fn)) {
		gim_err("Failed to LOAD register 'init-state'");
		return (-1);
	}

	if (run_vf(fn)) {
		gim_err("Failed to RUN register 'init-state'");
		return (-1);
	}

	if (idle_vf(fn)) {
		gim_err("Failed to IDLE register 'init-state'");
		return (-1);
	}

	if (save_vf(fn)) {
		gim_err("Failed to SAVE register 'init-state'");
		return (-1);
	}

	return 0;
}

void run_sdma(uint32_t pf_bdf)
{
	uint32_t  data;
	uint32_t  data1;
	uint32_t  wptr, rptr;
	struct function *fn = bdf_to_function(pf_bdf);


	gim_info("Check SDMA for bdf 0x%x\n", pf_bdf);
	data = read_register(fn, mmSDMA0_F32_CNTL);
	if (data & 1) {
		gim_info("SDMA0 is HALTED, unhalt it\n");
		data = 0;
		write_register(fn, mmSDMA0_F32_CNTL, data);
	} else {
		gim_info("SDMA0 is already running;"
			"It doesn't need to be unhalted\n");
	}

	data = read_register(fn, mmSDMA1_F32_CNTL);
	if (data & 1) {
		gim_info("SDMA1 is HALTED, unhalt it\n");
		data = 0;
		write_register(fn, mmSDMA1_F32_CNTL, data);
	} else {
		gim_info("SDMA1 is already running;"
			"It doesn't need to be unhalted\n");
	}

	data = read_register(fn, mmSDMA0_F32_CNTL);
	data1 = read_register(fn, mmSDMA1_F32_CNTL);
	gim_info("SDMA0_F32_CNTL = 0x%04x, SDMA1_F32_CNTL = 0x%04x\n",
		data, data1);

	wptr = read_register(fn, mmSDMA0_GFX_RB_WPTR);
	rptr = read_register(fn, mmSDMA0_GFX_RB_RPTR);
	gim_info("SDMA0 WPTR/RPTR = 0x%04x / 0x%04x\n", wptr, rptr);
}


void check_sdma(uint32_t pf_bdf)
{
	uint32_t  data;
	uint32_t  wptr, rptr;
	struct function *fn = bdf_to_function(pf_bdf);


	gim_info("Check SDMA for bdf 0x%x\n", pf_bdf);
	data = read_register(fn, mmSDMA0_F32_CNTL);
	if (data & 1)
		gim_info("SDMA0 is HALTED\n");
	else
		gim_info("SDMA0 is running\n");

	data = read_register(fn, mmSDMA1_F32_CNTL);
	if (data & 1)
		gim_info("SDMA1 is HALTED\n");
	else
		gim_info("SDMA1 is running\n");

	data = read_register(fn, mmSDMA0_F32_CNTL);
	gim_info("SDMA0_F32_CNTL = 0x%04x, ", data);
	data = read_register(fn, mmSDMA1_F32_CNTL);
	gim_info("SDMA1_F32_CNTL = 0x%04x\n", data);

	wptr = read_register(fn, mmSDMA0_GFX_RB_WPTR);
	rptr = read_register(fn, mmSDMA0_GFX_RB_RPTR);
	gim_info("SDMA0 WPTR/RPTR = 0x%04x / 0x%04x\n", wptr, rptr);

	wptr = read_register(fn, mmSDMA1_GFX_RB_WPTR);
	rptr = read_register(fn, mmSDMA1_GFX_RB_RPTR);
	gim_info("SDMA1 WPTR/RPTR = 0x%04x / 0x%04x\n", wptr, rptr);
}

struct function_list_node *is_on_run_list(struct function *fn)
{
	struct function_list_node *node, *start_node;
	struct adapter *adapt;


	if (fn == NULL) {
		gim_err("NULL function specified");
		return NULL;
	}
	adapt = fn->adapt;

	if (adapt == NULL) {
		gim_err("function not defined yet");
		return NULL;
	}

	node = start_node = adapt->curr_running_func;

	/* Nothing on run_list yet. */
	if (node == NULL)
		return NULL;

	/* First Node on the list matches */
	if (node->func == fn)
		return node;

	node = node->next;

	while (node != start_node) {
		if (node->func == fn) {
			gim_info("Found it!");
			return node;
		}
		node = node->next;
	}

	/* Not found on list */
	return 0;
}

struct function_list_node *get_function_node(struct function *func)
{
	struct function_list_node *node;
	struct function_list_node *start_node;
	struct adapter *adapt = func->adapt;

	mutex_lock(&adapt->curr_running_func_mutex);

	node = start_node = adapt->curr_running_func;

	while (node->func != func && node != start_node)
		node = node->next;

	mutex_unlock(&adapt->curr_running_func_mutex);

	/* Check the one we stopped at */
	if (node->func == func)
		return node;

	return NULL;
}

struct function_list_node *alloc_fn_list_node(struct adapter *adapt)
{
	int i;

	for (i = 0; i < MAX_VIRTUAL_FUNCTIONS; ++i) {
		if (adapt->fn_list_nodes[i].inuse == 0) {
			adapt->fn_list_nodes[i].inuse = 1;
			adapt->fn_list_nodes[i].func = NULL;
			adapt->fn_list_nodes[i].next = NULL;
			adapt->fn_list_nodes[i].pre  = NULL;

			gim_info("New Function List Node allocated at %p\t"
				"index %i\n", &adapt->fn_list_nodes[i], i);
			return &adapt->fn_list_nodes[i];
		}
	}
	gim_err("Failed to allocate a Function List Node\n");
	return NULL;
}

void free_fn_list_node(struct adapter *adapt, struct function_list_node *fln)
{
	int i;

	for (i = 0; i < MAX_VIRTUAL_FUNCTIONS; ++i) {
		if (&adapt->fn_list_nodes[i] == fln) {
			gim_info("Free Function_List_Node %p\n", fln);
			adapt->fn_list_nodes[i].inuse = 0;
		return;
		}
	}
	gim_err("Could not find Function List Node %p to free it\n", fln);
}

int check_cp_status(struct adapter *adapt, char *comment, struct function *func)
{
	uint32_t  data;

	data = pf_read_register(adapt, mmGRBM_STATUS);
	gim_dbg("GRBM_STATUS %s = 0x%08x\n", comment, data);

	if (data != 0x0003028)
		dump_gpu_status(func);
#if 0
	write_register(func, mmSRBM_GFX_CNTL, 9);
	data = read_register(func, mmCP_MQD_BASE_ADDR);
	gim_info("CP_MQD_BASE_ADDR = 0x%08x\n", data);
	data = read_register(func, mmCP_MQD_BASE_ADDR_HI);
	gim_info("CP_MQD_BASE_ADDR_HI = 0x%08x\n", data);
	write_register(func, mmSRBM_GFX_CNTL, 0);
#endif

	return GIM_OK;
}
char *decode_ram_usage(uint32_t index)
{
	switch (index) {
	case 0x30:
		return "  - KIQ:CP_MQD_BASE_ADDR VF0";

	case 0x31:
		return "  - KIQ:CP_MQD_BASE_ADDR_HI VF0";

	case 0x32:
		return "  - KIQ:CP_MQD_CONTROL VF0";

	case 0x33:
		return "  - KIQ:CP_MQD_BASE_ADDR VF1";

	case 0x34:
		return "  - KIQ:CP_MQD_BASE_ADDR_HI VF1";

	case 0x35:
		return "  - KIQ:CP_MQD_CONTROL VF1";

	case 0x64:
		return "  - HIQ:CP_MQD_BASE_ADDR VF0";

	case 0x65:
		return "  - HIQ:CP_MQD_BASE_ADDR_HI VF0";

	case 0x66:
		return "  - HIQ:CP_MQD_CONTROL VF0";

	case 0x67:
		return "  - HIQ:CP_MQD_BASE_ADDR VF1";

	case 0x68:
		return "  - HIQ:CP_MQD_BASE_ADDR_HI VF1";

	case 0x69:
		return "  - HIQ:CP_MQD_CONTROL VF1";

	default:
		return " ";
	}
}

void dump_scratch_ram(struct adapter *adapt,
			struct function *func,
			char *comment)
{
	int i = 0;
	uint32_t    data;
	uint32_t    reg_data;
	int  error_found = 0;
	int  is_pf = (&adapt->pf == func);


	if (gim_get_log_level() < 4)
		return;

	data = pf_read_register(adapt, mmRLC_CP_SCHEDULERS);
	if (is_pf)
		gim_info("RLC_CP_SCHEDULERS = 0x%0x %s for PF\n",
			data, comment);
	else
		gim_info("RLC_CP_SCHEDULERS = 0x%0x %s for VF%d\n",
			data, comment, func->func_id);

	gim_info("RLCV Scratch RAM\n");
	for (i = 0; i < 0x100; ++i) {
		pf_write_register(adapt, mmRLC_GPU_IOV_SCRATCH_ADDR, i);
		data = pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_ADDR);
		reg_data = pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_DATA);
		if (data != i) {
			if (!error_found)
				gim_warn("Cannot set Scratch RAM address;"
					 " tried to set %d but is %d\n",
					 i, data);
			error_found = 1;
		}

		if ((data >= 0x30 && data < 0x36) ||
			(data >= 0x64 && data < 0x6a) ||
			(data >= 0x97 && data < 0x9b) ||
			(data == 0x12))
			gim_info("RLC_V[0x%02x] = 0x%08x %s\n",
				data, reg_data, decode_ram_usage(data));
	}
}

struct timespec time_elapsed(struct timespec *ts_start)
{
	struct timespec ts_end;
	struct timespec ts_diff;

	getnstimeofday(&ts_end);
	ts_diff.tv_sec = ts_end.tv_sec - ts_start->tv_sec;
	if (ts_start->tv_nsec > ts_end.tv_nsec) {
		--ts_diff.tv_sec;
		ts_diff.tv_nsec = ts_end.tv_nsec +
				  (1000000000-ts_start->tv_nsec);
	} else {
		ts_diff.tv_nsec = ts_end.tv_nsec - ts_start->tv_nsec;
	}

	return ts_diff;
}

/* If lock available return 1, if can't lock return 0 */
int amd_try_spinlock(spinlock_t *lock, uint32_t usec_timeout)
{
	struct timespec start_time;
	struct timespec elapsed_time;

	/* check the lock */
	if (spin_trylock(lock))
		return 1;

	/* Lock is busy */
	gim_warn("Lock is busy\n");
	/* Get the current time */
	getnstimeofday(&start_time);
	/* loop checking lock and time until timeout */
	elapsed_time.tv_sec = 0;
	elapsed_time.tv_nsec = 0;
	while (!spin_trylock(lock)) {
		elapsed_time = time_elapsed(&start_time);
		if (elapsed_time.tv_nsec > (usec_timeout * 1000))
			return 0;
	}
	gim_info("Lock was busy but freed after %ld.%9ld sec\n",
			elapsed_time.tv_sec, elapsed_time.tv_nsec);
	return 1;
}

void __amd_spin_lock(spinlock_t *lock, const char *function, int line)
{
	int timeout = 10000;  /* Timeout in usec */

	if (!amd_try_spinlock(lock, timeout)) {
		gim_err("Spin lock is busy at %s:%d;"
			"it didn't free after timeout Still waiting!\n",
			function, line);
		spin_lock(lock);
	}
}

void load_vbios(struct function *func)
{
	struct resource *res;
	void *fb;

	res = &func->pci_dev->resource[BAR__FRAME_BUFFER];
	gim_info("FB.start = 0x%llx, FB.size = 0x%llx\n",
		res->start, res->end);
	if (func->adapt->pvbios_image == NULL) {
		gim_err("VBios image is missing\n");
		return;
	}

	gim_info("VBios -> %X %X %X %X %X %X %X %X\n",
		 func->adapt->pvbios_image[0],
		 func->adapt->pvbios_image[1],
		 func->adapt->pvbios_image[2],
		 func->adapt->pvbios_image[3],
		 func->adapt->pvbios_image[4],
		 func->adapt->pvbios_image[5],
		 func->adapt->pvbios_image[6],
		 func->adapt->pvbios_image[7]);

	fb = map_vf_fb(func->pci_dev);
	gim_info("FB VA = %p\n", fb);
	if (fb != NULL) {
		gim_info("Copy VBios from %p to %p for length 0x%0x\n",
			func->adapt->pvbios_image, fb,
			func->adapt->vbios_image_size);
		memcpy(fb, func->adapt->pvbios_image,
			func->adapt->vbios_image_size);
		gim_info("UnMap FB VA %p\n", fb);
		vunmap(fb);
	}
}

void set_timer_interrupts(struct adapter *adapt, int enable)
{
	kcl_type_u32 cg_intr_status;

	pf_write_register(adapt, mmSMC_IND_INDEX_0, ixCG_INTERRUPT_STATUS);
	cg_intr_status = pf_read_register(adapt, mmSMC_IND_DATA_0);

	if (enable) {
		/* Un-mask(enable) the timer interrupts */
		cg_intr_status = REG_SET_FIELD(cg_intr_status,
						CG_INTERRUPT_STATUS,
						DISP_TIMER_TRIGGER_MASK, 0);
		cg_intr_status = REG_SET_FIELD(cg_intr_status,
						CG_INTERRUPT_STATUS,
						DISP_TIMER2_TRIGGER_MASK, 0);
	} else {
		/* Mask(disable) the timer interrupts */
		cg_intr_status = REG_SET_FIELD(cg_intr_status,
						CG_INTERRUPT_STATUS,
						DISP_TIMER_TRIGGER_MASK, 1);
		cg_intr_status = REG_SET_FIELD(cg_intr_status,
						CG_INTERRUPT_STATUS,
						DISP_TIMER2_TRIGGER_MASK, 1);
	}

	pf_write_register(adapt, mmSMC_IND_DATA_0, cg_intr_status);
	pf_write_register(adapt, mmSMC_IND_INDEX_0, 0);
}

static int gim_notify_reset(struct adapter *adapt)
{
	struct function_list_node *curr;

	/* Notify reset to active VFs */
	curr = adapt->curr_running_func;
	do {
		curr->func->in_flr = 1;

		gim_info("Notify reset to VF%d\n", curr->func->func_id);

		amd_spin_lock(&adapt->mailbox_lock);
		mailbox_update_index(&adapt->pf, curr->func->func_id);
		mailbox_notify_flr(adapt, 0);
		spin_unlock(&adapt->mailbox_lock);

		curr = curr->next;

	} while (curr != adapt->curr_running_func);

	kcl_thread_sleep(10);
	return 0;
}

static void gim_notify_reset_completion(struct adapter *adapt)
{
	struct function_list_node *curr;

	/* Notify reset completion to active VFs */
	curr = adapt->curr_running_func;

	do {
		if (curr->func->in_flr) {
			gim_info("notify reset completion to VF%d\n",
				curr->func->func_id);
			amd_spin_lock(&adapt->mailbox_lock);
			mailbox_update_index(&adapt->pf,
					curr->func->func_id);
			mailbox_notify_flr(adapt, 1);
			spin_unlock(&adapt->mailbox_lock);
		}
		curr = curr->next;
	} while (curr != adapt->curr_running_func);
}

static int gim_notify_reset_per_vf(struct adapter *adapt, struct function *vf)
{
	gim_info("Notify reset to VF%d\n", vf->func_id);

	amd_spin_lock(&adapt->mailbox_lock);
	mailbox_update_index(&adapt->pf, vf->func_id);
	mailbox_notify_flr(adapt, 0);
	spin_unlock(&adapt->mailbox_lock);

	kcl_thread_sleep(10);
	return 0;
}

static void gim_notify_reset_completion_per_vf(struct adapter *adapt,
						struct function *vf)
{
	if (vf->in_flr) {
		gim_info("notify reset completion to VF%d\n", vf->func_id);
		amd_spin_lock(&adapt->mailbox_lock);
		mailbox_update_index(&adapt->pf, vf->func_id);
		mailbox_notify_flr(adapt, 1);
		spin_unlock(&adapt->mailbox_lock);
	}
}

int gim_sched_reset_vf(struct adapter *adapt,
			struct function *current_running_vf,
			struct function *next_running_vf,
			int command_status)
{
	struct function *vf_to_reset;
	struct function *vf_to_recover;

	if (command_status == FLR_REASON_FAILED_IDLE ||
	   command_status == FLR_REASON_FAILED_SAVE ||
	   command_status == FLR_REASON_EXCLUSIVE_TIMEOUT) {
		vf_to_reset = current_running_vf;
		vf_to_recover = next_running_vf;
	} else {
		vf_to_reset = next_running_vf;
		vf_to_recover = current_running_vf;
	}

	vf_to_reset->in_flr = 1;

	gim_notify_reset_per_vf(adapt, vf_to_reset);

	gim_info("begin to do VF FLR for VF%x\n", vf_to_reset->func_id);

	/* Trigger vf flr */
	if (gim_vf_flr(adapt, vf_to_reset))
		return -1;

	if (idle_vf(vf_to_reset))
		return -1;

	if (save_vf(vf_to_reset))
		return -1;

	if (vf_to_recover != NULL) {
		if (load_vf(vf_to_recover))
			return -1;

		if (run_vf(vf_to_recover))
			return -1;

		gim_info("GPU Reset VF%d, load/run next VF%d\n",
			vf_to_reset->func_id, vf_to_recover->func_id);
	} else
		gim_info("GPU Reset VF%d. No next VF, left in idle/save state;"
			"This state not expected.\n",
			vf_to_reset->func_id);

	gim_notify_reset_completion_per_vf(adapt, vf_to_reset);

	gim_info("end of VF FLR for VF%x\n", vf_to_reset->func_id);

	return 0;

}

int gim_sched_reset_gpu(struct adapter *adapt)
{
	int ret = 0;
	struct function_list_node *curr_func;

	gim_info("Pause scheduler\n");
	delete_timer(&adapt->sched_timer);
	adapt->schedler_running = false;

	/* Noitfy flr to active VFs */
	gim_info("Notify reset to active VFs\n");
	gim_notify_reset(adapt);

	/* PF FLR */
	gim_info("PF FLR to recover\n");
	/* Trigger pci hot reset */
	ret = gim_pci_hot_reset(adapt);

	/* Noitfy flr completion to active VFs */
	gim_info("Notify reset completion to active VFs\n");
	gim_notify_reset_completion(adapt);

	curr_func = adapt->curr_running_func;
	while (curr_func != NULL) {
		if (adapt->sched_opt == SCHEDULER__PREDICT_PERF)
			mark_func_not_scheduled(curr_func->func);
		else
			remove_from_run_list(curr_func->func);
	}

	return ret;
}

int gim_sched_reset(struct adapter *adapt,
					struct function *current_running_vf,
					struct function *next_running_vf,
					int command_status)
{
	if (gim_sched_reset_vf(adapt,
				current_running_vf,
				next_running_vf,
				command_status))
		return gim_sched_reset_gpu(adapt);

	return 0;
}
