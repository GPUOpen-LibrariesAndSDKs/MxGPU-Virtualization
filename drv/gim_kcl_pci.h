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

#ifndef _GPU_IOV_MODULE__KCL_PCI_H
#define _GPU_IOV_MODULE__KCL_PCI_H

#include "gim_kcl_type.h"

int kcl_pci_read_config_byte(kcl_pci_dev_handle dev,
				kcl_type_u16 where,
				kcl_type_u8 * val_ptr);

int kcl_pci_read_config_word(kcl_pci_dev_handle dev,
				kcl_type_u16 where,
				kcl_type_u16 *val_ptr);

int kcl_pci_read_config_dword(kcl_pci_dev_handle dev,
				kcl_type_u16 where,
				kcl_type_u32 *val_ptr);

int kcl_pci_write_config_byte(kcl_pci_dev_handle dev,
				kcl_type_u16 where,
				kcl_type_u8 val);

int kcl_pci_write_config_word(kcl_pci_dev_handle dev,
				kcl_type_u16 where,
				kcl_type_u16 val);

int kcl_pci_write_config_dword(kcl_pci_dev_handle dev,
				kcl_type_u16 where,
				kcl_type_u32 val);

void kcl_pci_enable_bus_master(kcl_pci_dev_handle dev);

void kcl_pci_disable_bus_master(kcl_pci_dev_handle dev);

int kcl_pci_find_ext_capability(kcl_pci_dev_handle dev,
				int cap, int pos);

int kcl_pci_find_capability(kcl_pci_dev_handle dev,
				int cap);

/* Extended Capabilities (PCI-X 2.0 and Express) */
#define GIM_PCI_EXT_CAP_ID(header)          (header & 0x0000ffff)
#define GIM_PCI_EXT_CAP_VER(header)         ((header >> 16) & 0xf)
#define GIM_PCI_EXT_CAP_NEXT(header)        ((header >> 20) & 0xffc)
#define GIM_PCI_CFG_SPACE_SIZE              256
#define GIM_PCI_CFG_SPACE_EXP_SIZE          4096
#define GIM_MIN_BYTES_PER_CAP               8

/* SR-IOV capabiltiy */
#define PCI_EXT_CAP_ID__SRIOV               0x10
#define PCI_SRIOV_CTRL                      0x08/* SR-IOV Control */
#define  PCI_SRIOV_CTRL_VFE                 0x01/* VF Enable */
#define  PCI_SRIOV_CTRL_MSE                 0x08/* VF Memory Space Enable */

#define PCI_CAP_ID_EXP         0x10    /* PCI Express */

#define PCI_STATUS             0x06    /* 16 bits */
#define PCI_REVISION_ID        0x08    /* Revision ID */
#define PCI_STATUS_CAP_LIST    0x10    /* Support Capability List */
#define PCI_CAPABILITY_LIST    0x34    /* Offset of first cap list entry */


#endif /* __KCL_PCI_H__ */

