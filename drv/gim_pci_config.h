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

#ifndef _GPU_IOV_MODULE__PCI_CONFIG_H
#define _GPU_IOV_MODULE__PCI_CONFIG_H


#include <linux/pci.h>

#define PCI_EXT_CONFIG_OFFSET 256
#define PCI_EXT_CONFIG_SIZE   4096
#define UNCORR_ERR_STATUS 0x154
#define PCI_EXP 0x58
#define PCI_AER 0x150    /* Advanced Error Reporting */
#define PCIE_UNCORR_ERR_STATUS 0x154
#define PCIE_UNCORR_ERR_MASK   0x158
#define PCIE_CORR_ERR_STATUS   0x160
#define PCIE_CORR_ERR_MASK     0x164


/*
 *Note that CAP_SRIOV is defined in the Tonga register spec as 0x330
 *but this is not guaranteed across asic generations.
 *While this will work with Tonga, it may not work for future asics.
 *If future asics place the CAP_SRIOV at a different offset
 *then the table will have to be dynamically adjusted for the new offset.
 *This value is used as an offset for the statically defined pci tables only.
 *Anywhere else in the code that needs to know the SRIOV offset is done
 *by scanning through the pci config space for the appropriate section header.
 */
#define CAP_SRIOV  0x330

struct pci_def {
	int offset;
	int size;
	char *name;
};

struct aer_item {
	int aer_offset;
	struct pci_dev *dev;
};

void dump_pci_config(uint8_t *pci_buf, char *comment);
void compare_pci_config(uint8_t *pci_buf, uint8_t *pci_buf2, char *comment);
void pci_config_save(struct pci_dev *dev, uint8_t *buf, int count);
void pci_config_restore(struct pci_dev *dev, uint8_t *buf, int count);
void pci_cache_bridges(struct pci_device_id *dev_list,
			int max_size, struct aer_item *aer_list);
void pci_disable_bridge_error_reporting(struct aer_item *aer_list);
void pci_cache_pfs(struct pci_dev *pf_devices[], int pf_count,
			int max_bridges, struct aer_item *device_list);
int  pci_validate_devices(struct aer_item *device_list);
int  pci_check_for_error(struct pci_dev *dev);
void pci_disable_error_reporting(struct pci_dev *dev);
int  check_for_error(struct pci_dev *dev, char *comment);
#endif
