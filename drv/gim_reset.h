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

#ifndef _GPU_IOV_MODULE__RESET_H
#define _GPU_IOV_MODULE__RESET_H

#define PCI_EXP 0x58
/* #define PCI_EXP_DEVSTA 0x0a */

/* Bridge Control Register */
#define PCI_BRIDGE_CNTL_REG                             0x3E
#define PCI_BRIDGE_CNTL_REG__SECONDARY_BUS_RESET        0x00000040

/* PCIE Link Cap Register */
#define PCIE_LINK_CAP_OFFSET                      0x0C
#define PCIE_LINK_CAP__LINK_SPEED                 0x0000000F
#define PCIE_LINK_CAP__LINK_WIDTH_MASK              0x000003f0L
#define PCIE_LINK_CAP__LINK_WIDTH__SHIFT          0x00000004
#define PCIE_LINK_CAP__CLOCK_POWER_MANAGEMENT_MASK 0x00040000
#define PCIE_LINK_CAP__LINK_ACTIVE_REPORT_CAPABLE  0x00100000

/* PCIE Link Status Register */
#define PCIE_LINK_STATUS_OFFSET                   0x12
#define PCIE_LINK_STATUS__LINK_TRAINING           0x00000800
#define PCIE_LINK_STATUS__LINK_ACTIVE           0x00002000


/* PCIE Link Cap 2 Register */
#define PCIE_LINK_CAP2_OFFSET                     0x2C
#define PCIE_LINK_CAP2__SUPPORTED_LINK_SPEEDS_VECTOR_MASK       0x000000FE
#define PCIE_LINK_CAP2__SUPPORTED_LINK_SPEEDS_VECTOR__SHIFT     0x00000001
#define PCIE_LINK_CAP2__SUPPORTED_LINK_SPEEDS_VECTOR__2_5       0x00000001
#define PCIE_LINK_CAP2__SUPPORTED_LINK_SPEEDS_VECTOR__5_0       0x00000002
#define PCIE_LINK_CAP2__SUPPORTED_LINK_SPEEDS_VECTOR__8_0       0x00000004


#define PCI_VENDOR_ID                       0x00
#define PCI_DEVICE_ID                       0x02

/* 500 ms */
#define PCIE_TRAINING_TIMEOUT_LIMIT (500*1000)

/* Extended Capabilities (PCI-X 2.0 and Express) */
#define PCI_EXT_CAP_ID(header) (header & 0x0000ffff)
#define PCI_EXT_CAP_VER(header) ((header >> 16) & 0xf)
#define PCI_EXT_CAP_NEXT(header) ((header >> 20) & 0xffc)

/* Advanced Error Reporting */
#define PCI_EXT_CAP_ID_AER 1
#define PCI_ERR_UNCOR_STATUS 4
#define PCI_ERR_CORR_STATUS 0x10
#define PCI_ERR_UNCOR_MASK 8/* Uncorrectable Error Mask */

#define PCI_CFG_SPACE_SIZE 256
#define PCI_CFG_SPACE_EXP_SIZE 4096

#define CAP_SRIOV  0x330
#define PCI_SRIOV_CAP 0x04

#define SOFT_REGISTERS_TABLE_28 0x3fc54
#define FW_STATE_543            0x3d440
#define UCODE_COUNT             13
#define TABLE_SIZE              0x20
#define FW_STATE_684            0x3d674
#define FW_STATE_685            0x3d678

int gim_pci_hot_reset(struct adapter *adapt);
void gim_clear_all_errors(struct adapter *adapt);
int validate_link_status(struct adapter *adapt);

#endif /*__RESET_H__*/


