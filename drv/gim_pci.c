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

#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/mod_devicetable.h>

#include "gim_pci.h"
#include "gim_debug.h"

uint32_t get_bdf(struct pci_dev *pdev)
{
	return ((pdev->bus->number << 8) | pdev->devfn);
}

static unsigned short get_vf_deviceid(struct pci_dev *dev)
{
	int pos;
	unsigned short device = 0;

	pos = pci_find_ext_capability(dev, PCI_EXT_CAP_ID_SRIOV);
	pci_read_config_word(dev, pos + PCI_SRIOV_VF_DID, &device);

	return device;
}

static unsigned short get_vf_offset(struct pci_dev *dev)
{
	int pos;
	unsigned short vf_offset = 0;

	pos = pci_find_ext_capability(dev, PCI_EXT_CAP_ID_SRIOV);
	pci_read_config_word(dev, pos + PCI_SRIOV_VF_OFFSET, &vf_offset);
	return vf_offset;
}

static unsigned short get_vf_stride(struct pci_dev *dev)
{
	int pos;
	unsigned short vf_stride = 0;

	pos = pci_find_ext_capability(dev, PCI_EXT_CAP_ID_SRIOV);
	pci_read_config_word(dev, pos + PCI_SRIOV_VF_STRIDE, &vf_stride);
	return vf_stride;
}

static unsigned short get_num_vfs(struct pci_dev *dev)
{
	int pos;
	unsigned short num_vfs = 0;

	pos = pci_find_ext_capability(dev, PCI_EXT_CAP_ID_SRIOV);
	pci_read_config_word(dev, pos + PCI_SRIOV_NUM_VF, &num_vfs);
	return num_vfs;
}

static int is_attached_vf(struct pci_dev *pf_dev, struct pci_dev *vf_dev)
{
	unsigned short  vf_stride = get_vf_stride(pf_dev);
	unsigned short  num_vfs   = get_num_vfs(pf_dev);
	uint32_t        vf_start  = get_bdf(pf_dev) + get_vf_offset(pf_dev);
	uint32_t        vf_bdf    = get_bdf(vf_dev);
	int             is_found  = 0;
	unsigned short  i;

	for (i = 0; i < num_vfs; ++i) {
		if (vf_bdf == vf_start)
			is_found = 1;

		vf_start += vf_stride;
	}
	return is_found;
}

int get_func_id(struct pci_dev *pf_dev, struct pci_dev *vf_dev)
{
	int func_id = -1;
	unsigned short  vf_stride = get_vf_stride(pf_dev);
	unsigned short  num_vfs   = get_num_vfs(pf_dev);
	uint32_t        vf_start  = get_bdf(pf_dev) + get_vf_offset(pf_dev);
	uint32_t        vf_bdf    = get_bdf(vf_dev);
	unsigned short  i;

	for (i = 0; i < num_vfs; ++i) {
		if (vf_bdf == vf_start)
			func_id = i;

		vf_start += vf_stride;
	}
	return func_id;
}

int enumerate_vfs(struct pci_dev *dev, int count, struct pci_dev *vf_devices[])
{
	struct pci_dev *vfdev    = NULL;
	int             vf_count  = 0;
	unsigned short  device_id = get_vf_deviceid(dev);

	while ((vfdev = pci_get_device(dev->vendor, device_id, vfdev)) &&
			vf_count < count) {
		if (is_attached_vf(dev, vfdev)) {
			vf_devices[vf_count++] = vfdev;
			gim_info("vf found: %02x:%x.%x\n", vfdev->bus->number,
				(vfdev->devfn>>3), (vfdev->devfn & 0x7));
		}
	}
	return vf_count;
}

int enumerate_all_pfs(const struct pci_device_id *device_list, int count,
			struct pci_dev *pf_devices[])
{
	struct pci_dev *pfdev = NULL;
	int pf_count = 0;
	int list_count = 0;

	while (device_list[list_count].vendor != 0 &&
		   device_list[list_count].device != 0) {
		while ((pfdev = pci_get_device(device_list[list_count].vendor,
						device_list[list_count].device,
						pfdev))) {
			if (pf_count < count) {
				pf_devices[pf_count++] = pfdev;
			} else {
				gim_info("More than %d phyiscal devicesfound\n!",
					count);
			}
		}
		/* Search for next device id */
		list_count++;
		pfdev = NULL;
	}
	return pf_count;
}

int sriov_is_ari_enabled(struct pci_dev *dev)
{
	uint16_t capabilities;
	uint16_t control;
	int pos;

	pos = pci_find_ext_capability(dev, PCI_EXT_CAP_ID_SRIOV);
	pci_read_config_word(dev, pos + PCI_SRIOV_CAP, &capabilities);

	gim_info("PCI_SRIOV_CAP = 0x%08x\n", capabilities);

	if (!(capabilities & PCI_SRIOV_CAP_ARI_PRESERVED)) {
		gim_info("PCI_SRIOV_CAP_ARI_PRESERVED is not set.\n");
		gim_info("--> Assume ARI is not enabled\n");
		return 0;
	}

	pci_read_config_word(dev, pos + PCI_SRIOV_CTRL, &control);

	gim_info("PCI_SRIOV_CTRL = 0x%08x\n", control);

	if (control & PCI_SRIOV_CTRL_ARI) {
		gim_info("PCI_SRIOV_CTRL_ARI is set --> ARI is supported\n");
		return 1;
	}

	gim_info("PCI_SRIOV_CTRL_ARI is not set --> ARI is not supported\n");

	return 0;
}

int sriov_enable_ari(struct pci_dev *pf_dev)
{
	/* We don't currently support enabling the ARI if it is not already
	 * enable
	 * Enabling ARI involves determining the path from the root port,
	 * through any switches and finally to the end-point.
	 *
	 * Along the path we need to check if ARI capability is supported and
	 * if it is we need to turn on the ARI control bit.
	 * If anything in the path fails then ARI cannot be enabled.
	 */
	return 0;
}

void enable_device(struct pci_dev *dev)
{
	unsigned short cmd = 0;

	pci_read_config_word(dev, PCI_COMMAND, &cmd);
	cmd |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
	pci_write_config_word(dev, PCI_COMMAND, cmd);
}
