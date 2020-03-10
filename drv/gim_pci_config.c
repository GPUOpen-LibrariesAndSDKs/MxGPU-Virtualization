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
#include <linux/delay.h>

#include "gim_pci_config.h"
#include "gim_debug.h"
#include "gim_pci.h"
#include "gim_flr.h"

/* DEBUG - used to save a temporary copy for comparing. */
uint8_t debug_config_space[4096];

/*
 * Config space restore table
 *
 * This table contains the config space offsets (with names)
 * in the order that they need to be restored.
 * Do not alter this table unless you absolutely know what you are doing
 * as changing the order or moving values can cause unexpected results
 * or system hangs.
 */

static struct pci_def restore_tbl[] = {
	{0, 2, "Vendor_Id"},
	{2, 2, "Device_Id"},
	{6, 2, "Status"},
	{8, 1, "Revision Id"},
	{9, 1, "Prog I/f"},
	{0x0a, 1, "Sub Class"},
	{0x0b, 1, "Base Class"},
	{0x0c, 1, ""},
	{0x0d, 1, ""},
	{0x0e, 1, ""},
	{0x0f, 1, ""},
	{0x10, 4, "BAR0"},
	{0x14, 4, "BAR1"},
	{0x18, 4, "BAR2"},
	{0x1c, 4, "BAR3"},
	{0x20, 4, "BAR4"},
	{0x24, 4, "BAR5"},
	{0x2c, 2, "Subsystem vendor Id"},
	{0x2e, 2, "Subsystem Id"},
	{0x30, 4, "ROM BAR"},
	{0x3c, 1, "Int Line"},
	{0x3d, 1, "Int Pin"},
	{0x3e, 1, "Min GNT"},
	{0x3f, 1, "Max Latency"},
	{0x40, 4, "Unused"},
	{0x44, 4, "Unused"},
	{0x48, 4, "Vendor Cap List"},
	{0x4c, 4, "Adapter Id Master-write"},
	{0x54, 4, "PMI Status Control"},

	/* 0x58 = PCIe Capabilities */
	{0x5c, 4, "PCI-DevCap"},
	{0x60, 2, "PCI-DevCntl"},
	{0x62, 2, "PCI-DevStatus"},
	{0x64, 4, "PCI-LinkCap"},
	{0x68, 2, "PCI-LinkCntl"},
	{0x6a, 2, "PCI-LinkStatus"},

	/* 0xA0 = MSI Capabilities */
	{0xa0, 2, "MSI_CAP_LIST"},
	{0xa2, 2, "MSI Msg Cntl"},
	{0xa4, 4, "MSI Msg Addr Lo"},
	{0xa8, 4, "MSI Msg Addr Hi"},
	{0xac, 4, "MSI Msg Data/MSI Mask"},
	{0xb0, 4, "MSI Mask/Pending"},
	{0xb4, 4, "MSI Pending_64"},


	{0x154, 4, "PCIe_UNCORR_ERR_STATUS"},
	{0x168, 4, "PCIe_ADV_ERR_CAP_CNTL"},
	{0x16c, 4, "PCIe_HDR_LOG0"},
	{0x170, 4, "PCIe_HDR_LOG1"},
	{0x174, 4, "PCIe_HDR_LOG2"},
	{0x178, 4, "PCIe_HDR_LOG3"},

	/* 0x200 = Resizeable BAR Enhanced Capability */
	{0x208, 2, "PCIe_BAR0_Cntl"},
	{0x210, 2, "PCIe_BAR1_Cntl"},
	{0x218, 2, "PCIe_BAR2_Cntl"},
	{0x220, 2, "PCIe_BAR3_Cntl"},
	{0x228, 2, "PCIe_BAR4_Cntl"},
	{0x230, 2, "PCIe_BAR5_Cntl"},
	/* Need to restore the BAR
	 *after writing the Resizeable Bar Enhanced Capability
	 */
	{0x10, 4, "BAR0"},
	{0x14, 4, "BAR1"},
	{0x18, 4, "BAR2"},
	{0x1c, 4, "BAR3"},
	{0x20, 4, "BAR4"},
	{0x24, 4, "BAR5"},

	/* 0x2b8 = unused? */
	{0x2b8, 4, "?"},

	/* 0x330 = SRIOV */
	{CAP_SRIOV+PCI_SRIOV_CAP, 4, "SR-IOV Capabilities"},
	{CAP_SRIOV+0x0a, 2, "SR-IOV Status"},
	{CAP_SRIOV+0x0c, 2, "SR-IOV Initial VFs"},
	{CAP_SRIOV+0x0e, 2, "SR-IOV Total VFs"},
	{CAP_SRIOV+0x10, 2, "SR-IOV Num VFs"},
	{CAP_SRIOV+0x12, 2, "SR-IOV Func Dep Link"},
	{CAP_SRIOV+0x14, 2, "SR-IOV First Offset"},
	{CAP_SRIOV+0x16, 2, "SR-IOV Stride"},
	{CAP_SRIOV+0x1a, 2, "SR-IOV VF Device ID"},
	{CAP_SRIOV+0x1c, 4, "SR-IOV Sup PageSize"},
	{CAP_SRIOV+0x20, 4, "SR-IOV Sys PageSize"},
	{CAP_SRIOV+0x24, 4, "SR-IOV VF BAR0"},
	{CAP_SRIOV+0x2c, 4, "SR-IOV VF BAR2"},
	/*  Note that restoring this BAR causes system hang */
	{CAP_SRIOV+0x38, 4, "SR-IOV VF BAR5"},

	/* Make sure VF enable bit is set AFTER setting up SRIOV capabilities.*/
	{CAP_SRIOV+0x08, 2, "SR-IOV Control"},

	/* 0x400 = GPU-IOV */
	{0x400+0x20, 4, "GPU-IOV Context"},
	{0x400+0x24, 2, "GPU-IOV FB Avail"},
	{0x400+0x26, 2, "GPU-IOV FB Consumed"},
	/* Note that Offset doesn't respond to word boundary cycles.
	*Writes must be dword
	*otherwise FBx offset is not restored properly
	*/
	{0x400+0x2c, 4, "GPU-IOV FB0 Size/Offset"},
	{0x400+0x30, 4, "GPU-IOV FB1 Size/Offset"},
	{0x400+0x34, 4, "GPU-IOV FB2 Size/Offset"},
	{0x400+0x38, 4, "GPU-IOV FB3 Size/Offset"},
	{0x400+0x3c, 4, "GPU-IOV FB4 Size/Offset"},
	{0x400+0x40, 4, "GPU-IOV FB5 Size/Offset"},
	{0x400+PCI_GPUIOV_VF6_FB_SIZE, 4, "GPU-IOV FB6 Size/Offset"},
	{0x400+PCI_GPUIOV_VF7_FB_SIZE, 4, "GPU-IOV FB7 Size/Offset"},
	{0x400+PCI_GPUIOV_VF8_FB_SIZE, 4, "GPU-IOV FB8 Size/Offset"},
	{0x400+PCI_GPUIOV_VF9_FB_SIZE, 4, "GPU-IOV FB9 Size/Offset"},
	{0x400+PCI_GPUIOV_VF10_FB_SIZE, 4, "GPU-IOV FB10 Size/Offset"},
	{0x400+PCI_GPUIOV_VF11_FB_SIZE, 4, "GPU-IOV FB11 Size/Offset"},
	{0x400+PCI_GPUIOV_VF12_FB_SIZE, 4, "GPU-IOV FB12 Size/Offset"},
	{0x400+PCI_GPUIOV_VF13_FB_SIZE, 4, "GPU-IOV FB13 Size/Offset"},
	{0x400+PCI_GPUIOV_VF14_FB_SIZE, 4, "GPU-IOV FB14 Size/Offset"},
	{0x400+PCI_GPUIOV_VF15_FB_SIZE, 4, "GPU-IOV FB15 Size/Offset"},
	{4, 2, "Command"},
	{0, 0, NULL},
};


/*
 * pci_table is used only for DEBUG to identify
 *the pci confnig space by register name
 */

struct pci_def pci_table[] = {
	{0, 2, "Vendor_Id"},
	{2, 2, "Device_Id"},
	{4, 2, "Command"},
	{6, 2, "Status"},
	{8, 1, "Revision Id"},
	{9, 1, "Prog I/f"},
	{0x0a, 1, "Sub Class"},
	{0x0b, 1, "Base Class"},
	{0x0c, 1, "Cache Line"},
	{0x0d, 1, "Latency"},
	{0x0e, 1, "Header"},
	{0x0f, 1, "BIST (Built In Self Test)"},
	{0x10, 4, "BAR0"},
	{0x14, 4, "BAR1"},
	{0x18, 4, "BAR2"},
	{0x1c, 4, "BAR3"},
	{0x20, 4, "BAR4"},
	{0x24, 4, "BAR5"},
	{0x2c, 2, "Subsystem vendor Id"},
	{0x2e, 2, "Subsystem Id"},
	{0x30, 4, "ROM BAR"},
	{0x3c, 1, "Int Line"},
	{0x3d, 1, "Int Pin"},
	{0x3e, 1, "Min GNT"},
	{0x3f, 1, "Max Latency"},
	{0x40, 4, "Unused"},
	{0x44, 4, "Unused"},
	{0x48, 4, "Vendor Cap List"},
	{0x4c, 4, "Adapter Id Master-write"},
	{0x54, 4, "PMI Status Control"},

	/* 0x58 = PCIe Capabilities */
	{0x5c, 4, "PCI-DevCap"},
	{0x60, 2, "PCI-DevCntl"},
	{0x62, 2, "PCI-DevStatus"},
	{0x64, 4, "PCI-LinkCap"},
	{0x68, 2, "PCI-LinkCntl"},
	{0x6a, 2, "PCI-LinkStatus"},

	/* 0xA0 = MSI Capabilities */
	{0xa0, 2, "MSI_CAP_LIST"},
	{0xa2, 2, "MSI Msg Cntl"},
	{0xa4, 4, "MSI Msg Addr Lo"},
	{0xa8, 4, "MSI Msg Addr Hi"},
	{0xac, 4, "MSI Msg Data/MSI Mask"},
	{0xb0, 4, "MSI Mask/Pending"},
	{0xb4, 4, "MSI Pending_64"},


	{0x154, 4, "PCIe_UNCORR_ERR_STATUS"},
	{0x168, 4, "PCIe_ADV_ERR_CAP_CNTL"},
	{0x16c, 4, "PCIe_HDR_LOG0"},
	{0x170, 4, "PCIe_HDR_LOG1"},
	{0x174, 4, "PCIe_HDR_LOG2"},
	{0x178, 4, "PCIe_HDR_LOG3"},

	/* 0x200 = Resizeable BAR Enhanced Capability */
	{0x208, 2, "PCIe_BAR0_Cntl"},
	{0x210, 2, "PCIe_BAR1_Cntl"},
	{0x218, 2, "PCIe_BAR2_Cntl"},
	{0x220, 2, "PCIe_BAR3_Cntl"},
	{0x228, 2, "PCIe_BAR4_Cntl"},
	{0x230, 2, "PCIe_BAR5_Cntl"},

	/* 0x2b8 = unused? */
	{0x2b8, 4, "?"},

	/* 0x330 = SRIOV */
	{CAP_SRIOV+PCI_SRIOV_CAP, 4, "SR-IOV Capabilities"},
	{CAP_SRIOV+0x08, 2, "SR-IOV Control"},
	{CAP_SRIOV+0x0a, 2, "SR-IOV Status"},
	{CAP_SRIOV+0x0c, 2, "SR-IOV Initial VFs"},
	{CAP_SRIOV+0x0e, 2, "SR-IOV Total VFs"},
	{CAP_SRIOV+0x10, 2, "SR-IOV Num VFs"},
	{CAP_SRIOV+0x12, 2, "SR-IOV Func Dep Link"},
	{CAP_SRIOV+0x14, 2, "SR-IOV First Offset"},
	{CAP_SRIOV+0x16, 2, "SR-IOV Stride"},
	{CAP_SRIOV+0x1a, 2, "SR-IOV VF Device ID"},
	{0x34c, 4, "SR-IOV Support Page Size"},
	{0x350, 4, "SR-IOV System Page Size"},
	{0x354, 4, "SR-IOV VF BAR0"},
	{0x358, 4, "SR-IOV VF BAR1"},
	{0x35c, 4, "SR-IOV VF BAR2"},
	{0x360, 4, "SR-IOV VF BAR3"},
	{0x364, 4, "SR-IOV VF BAR4"},
	{0x368, 4, "SR-IOV VF BAR5"},

	/* 0x400 = GPU-IOV */
	{0x400+0x0c, 1, "GPU-IOV Command"},
	{0x400+0x0d, 1, "GPU-IOV FCN_id"},
	{0x400+0x0e, 1, "GPU-IOV Next FCN_id"},
	{0x400+0x10, 1, "GPU-IOV Status"},
	{0x400+0x1c, 4, "GPU-IOV VM_INIT Status"},
	{0x400+PCI_GPUIOV_CNTXT, 4, "GPU-IOV Context"},
	{0x400+PCI_GPUIOV_TOTAL_FB_AVAILABLE, 2, "GPU-IOV FB Avail "},
	{0x400+PCI_GPUIOV_TOTAL_FB_CONSUMED, 2, "GPU-IOV FB Consumed"},
	{0x400+PCI_GPUIOV_VF0_FB_SIZE, 2, "GPU-IOV FB0 Size"},
	{0x400+PCI_GPUIOV_VF0_FB_OFFSET, 2, "GPU-IOV FB0 Offset "},
	{0x400+PCI_GPUIOV_VF1_FB_SIZE, 2, "GPU-IOV FB1 Size"},
	{0x400+PCI_GPUIOV_VF1_FB_OFFSET, 2, "GPU-IOV FB1 Offset "},
	{0x400+PCI_GPUIOV_VF2_FB_SIZE, 2, "GPU-IOV FB2 Size"},
	{0x400+PCI_GPUIOV_VF2_FB_OFFSET, 2, "GPU-IOV FB2 Offset "},
	{0x400+PCI_GPUIOV_VF3_FB_SIZE, 2, "GPU-IOV FB3 Size"},
	{0x400+PCI_GPUIOV_VF3_FB_OFFSET, 2, "GPU-IOV FB3 Offset "},
	{0x400+PCI_GPUIOV_VF4_FB_SIZE, 2, "GPU-IOV FB4 Size"},
	{0x400+PCI_GPUIOV_VF4_FB_OFFSET, 2, "GPU-IOV FB4 Offset "},
	{0x400+PCI_GPUIOV_VF5_FB_SIZE, 2, "GPU-IOV FB5 Size"},
	{0x400+PCI_GPUIOV_VF5_FB_OFFSET, 2, "GPU-IOV FB5 Offset "},
	{0x400+PCI_GPUIOV_VF6_FB_SIZE, 2, "GPU-IOV FB6 Size"},
	{0x400+PCI_GPUIOV_VF6_FB_OFFSET, 2, "GPU-IOV FB6 Offset "},
	{0x400+PCI_GPUIOV_VF7_FB_SIZE, 2, "GPU-IOV FB7 Size"},
	{0x400+PCI_GPUIOV_VF7_FB_OFFSET, 2, "GPU-IOV FB7 Offset "},
	{0x400+PCI_GPUIOV_VF8_FB_SIZE, 2, "GPU-IOV FB8 Size"},
	{0x400+PCI_GPUIOV_VF8_FB_OFFSET, 2, "GPU-IOV FB8 Offset "},
	{0x400+PCI_GPUIOV_VF9_FB_SIZE, 2, "GPU-IOV FB9 Size"},
	{0x400+PCI_GPUIOV_VF9_FB_OFFSET, 2, "GPU-IOV FB9 Offset "},
	{0x400+PCI_GPUIOV_VF10_FB_SIZE, 2, "GPU-IOV FB10 Size"},
	{0x400+PCI_GPUIOV_VF10_FB_OFFSET, 2, "GPU-IOV FB10 Offset "},
	{0x400+PCI_GPUIOV_VF11_FB_SIZE, 2, "GPU-IOV FB11 Size"},
	{0x400+PCI_GPUIOV_VF11_FB_OFFSET, 2, "GPU-IOV FB11 Offset "},
	{0x400+PCI_GPUIOV_VF12_FB_SIZE, 2, "GPU-IOV FB12 Size"},
	{0x400+PCI_GPUIOV_VF12_FB_OFFSET, 2, "GPU-IOV FB12 Offset "},
	{0x400+PCI_GPUIOV_VF13_FB_SIZE, 2, "GPU-IOV FB13 Size"},
	{0x400+PCI_GPUIOV_VF13_FB_OFFSET, 2, "GPU-IOV FB13 Offset "},
	{0x400+PCI_GPUIOV_VF14_FB_SIZE, 2, "GPU-IOV FB14 Size"},
	{0x400+PCI_GPUIOV_VF14_FB_OFFSET, 2, "GPU-IOV FB14 Offset "},
	{0x400+PCI_GPUIOV_VF15_FB_SIZE, 2, "GPU-IOV FB15 Size"},
	{0x400+PCI_GPUIOV_VF15_FB_OFFSET, 2, "GPU-IOV FB15 Offset "},
	{PCI_CONFIG_SIZE_TO_SAVE, 4, "End-of-Config"},
	{0, 0, NULL},
};

static int get_config_byte(struct pci_dev *dev, int pos)
{
	uint32_t  val;

	pci_read_config_dword(dev, pos & ~0x3, &val);

	val = ((val >> (8 * (pos & 0x3))) & 0x00FF);
	return val;
}

static int get_config_word(struct pci_dev *dev, int pos)
{
	uint32_t  val;

	pci_read_config_dword(dev, pos & ~0x3, &val);

	if ((pos & 3) == 0x3)
		gim_info("get_config_word() - Invalid request with pos = %d\n",
				pos);

	val = ((val >> (8 * (pos & 0x3))) & 0xFFFF);
	return val;
}
/*
 * Search the capability structure and return either the position of the
 * capability or 0 if not found
 */

static int get_capability(struct pci_dev *dev, int cap, int pos)
{
	int         next_pos;
	uint32_t    header;
	int         pos_specified = false;


	if (pos)
		pos_specified = true;

	/* Supports a capability list? */
	if (get_config_word(dev, PCI_STATUS) & PCI_STATUS_CAP_LIST) {

		if (!pos_specified) {
			/* Cap is word aligned */
			pos = get_config_byte(dev, PCI_CAPABILITY_LIST) & ~3;
		}
		while (pos && (pos < 255)) {
			if (get_config_byte(dev,
				pos + PCI_CAP_LIST_ID) == cap)	{
				/* This is the one we are looking for */
				return pos;

			}
			next_pos = get_config_byte(dev,
					pos + PCI_CAP_LIST_NEXT) & ~3;
			pos = next_pos;
		}
		/*
		 * Capability not found in Config space,
		 *check extended config space 256 -> 4096
		 */
		if (!pos_specified)
			pos = PCI_EXT_CONFIG_OFFSET;

		while (pos && (pos < PCI_EXT_CONFIG_SIZE)) {
			pci_read_config_dword(dev, pos, &header);
			if (PCI_EXT_CAP_ID(header) == cap)
				return pos;

			pos = PCI_EXT_CAP_NEXT(header);
		}
	}
	return 0;
}

int get_sriov_offset(struct pci_dev *dev)
{
	int pos = PCI_EXT_CONFIG_OFFSET;

	pos = get_capability(dev, PCI_EXT_CAP_ID_SRIOV, pos);
	gim_info("SRIOV Capabiltiy found at offset 0x%03x\n", pos);
	return pos;
}

void dump_pci_config(uint8_t *pci_buf, char *comment)
{
	uint32_t  val;
	int i;
	struct pci_def *tbl;

	tbl = pci_table;

	gim_info("PCIe Config Space - %s\n", comment);
	for (i = 0; tbl[i].name != NULL; ++i) {

		switch (tbl[i].size) {
		case 1:
			val = (uint32_t) (((uint8_t *)pci_buf)[tbl[i].offset]);
			gim_info("[0x%03x] - 0x%02x %s\n",
					tbl[i].offset, val, tbl[i].name);
			break;
		case 2:
			val =
			(uint32_t) (((uint16_t *)pci_buf)[tbl[i].offset/2]);
			gim_info("[0x%03x] - 0x%04x %s\n",
					tbl[i].offset, val, tbl[i].name);
			break;
		case 4:
			val = (((uint32_t *)pci_buf)[tbl[i].offset/4]);
			gim_info("[0x%03x] - 0x%08x %s\n",
					tbl[i].offset, val, tbl[i].name);
			break;
		}
	}
}

void compare_pci_config(uint8_t *buf1, uint8_t *buf2, char *comment)
{
	uint32_t  val1;
	uint32_t  val2;
	int       next_table_offset;
	int       next_expected_offset;
	int i;


	if (buf1 == NULL || buf2 == NULL) {
		gim_info("compare_pci_config - NULL input buffer\n");
		return;
	}

	gim_info("Compare PCIe Config Space - %s\n", comment);
	gim_info("Offset      Before        After\n");
	gim_info("------      ------        -----\n");
	for (i = 0; pci_table[i].name != NULL; ++i) {

		switch (pci_table[i].size) {
		case 1:
			val1 =
			(uint32_t)(((uint8_t *)buf1)[pci_table[i].offset]);
			val2 =
			(uint32_t)(((uint8_t *)buf2)[pci_table[i].offset]);
			if (val1 != val2)
				gim_info("[0x%03x] - 0x%02x 0x%02x %s\n",
					pci_table[i].offset,
					val1, val2, pci_table[i].name);
			break;

		case 2:
			val1 =
			(uint32_t) (((uint16_t *)buf1)[pci_table[i].offset/2]);
			val2 =
			(uint32_t) (((uint16_t *)buf2)[pci_table[i].offset/2]);
			if (val1 != val2)
				gim_info("[0x%03x] - 0x%04x 0x%04x %s\n",
					pci_table[i].offset,
					val1, val2, pci_table[i].name);
			break;

		case 4:
			val1 = (((uint32_t *)buf1)[pci_table[i].offset/4]);
			val2 = (((uint32_t *)buf2)[pci_table[i].offset/4]);
			if (val1 != val2)
				gim_info("[0x%03x] - 0x%08x 0x%08x %s\n",
					pci_table[i].offset, val1,
					val2, pci_table[i].name);
			break;
		}
		/* Fill in entries until the next offset */
		if (pci_table[i+1].name != NULL) {
			next_expected_offset = pci_table[i].offset
						+ pci_table[i].size;
			next_table_offset = pci_table[i+1].offset;
			while (next_expected_offset < next_table_offset) {
				val1 = (uint32_t)
				(((uint8_t *)buf1)[next_expected_offset]);
				val2 = (uint32_t)
				(((uint8_t *)buf2)[next_expected_offset]);
				next_expected_offset += 1;
				if (val1 != val2) {
					gim_info("[0x%03x] - 0x%02x 0x%02x\n",
					next_expected_offset, val1, val2);
				}
			}
		}
	}
}

void pci_config_save(struct pci_dev *dev, uint8_t *buf, int count)
{
	int i;
	uint32_t *pcfg;
	int pos = get_sriov_offset(dev);

	if (pos == 0)
		gim_warn("Can't find SR-IOV cap structure in config space\n");

	pcfg = (uint32_t *) (buf);

	for (i = 0; i < count; i += 4) {
		pci_read_config_dword(dev, i, pcfg);
		
		/* When we restore pci config space,
		*  we don't want to restore VF_ENABLE bit in SR-IOV structure.
		*/

		/* Prevent restore by not saving it in the first place. */
		if ((pos != 0) && (i == pos+PCI_SRIOV_CTRL)) {
			gim_info("pci_config_save: Found PCI_SRIOV_CTRL = 0x%08x\n",
					*pcfg);
			*pcfg &= ~PCI_SRIOV_CTRL_VFE;
			gim_info("pci_config_save: Save it as 0x%08x\n", *pcfg);
		}
		++pcfg;
	}
}

/*
 * restore_regrestore_reg() - Restore a single entry (index) from the restore_tbl[]
 * @dev: pointer to the pci device 
 * @data: pointer to the raw data in the form of an image of
 * @restore_table: the pcie config space restore_table -
 *		   a table of entries describing the data
 * 		   that is to be restored such as
 * 		   offset (into config space), size (byte, word or dword)
 * 		   and comment (name of field).
 * @index: the entry needs to restore
 *
 * We can't blindly copy the image back to config space,
 *there are some fules to follow regarding the order.
 *
 */
static void restore_reg(struct pci_dev *dev, uint8_t *data,
		struct pci_def *restore_table, int index)
{
	uint8_t    p8, i8;
	uint16_t   p16, i16;
	uint32_t   p32, i32;
	struct pci_def *entry;
	uint32_t   error;

	pci_read_config_dword(dev, UNCORR_ERR_STATUS, &error);
	if (error != 0x0) {
		gim_err("UNCORR_ERR_STATUS = 0x%08x before restoring index %d, offset 0x%02x\n",
			error, index, restore_table[index].offset);
		/* Clear all error bits */
		pci_write_config_dword(dev, UNCORR_ERR_STATUS, error);
		pci_read_config_dword(dev, UNCORR_ERR_STATUS, &error);
	}

	entry = &restore_table[index];
	switch (entry->size) {
	case 1:
		p8 = (((uint8_t *) data)[entry->offset]);
		pci_read_config_byte(dev, entry->offset, &i8);
		if (i8 != p8) {
			gim_info("Restoring 0x%02x to offset 0x%03x (%s)\n",
				p8, entry->offset, entry->name);
			pci_write_config_byte(dev, entry->offset, p8);
		}
		break;

	case 2:
		p16 = (((uint16_t *) data)[entry->offset/2]);
		pci_read_config_word(dev, entry->offset, &i16);
		if (i16 != p16) {
			gim_info("Restoring 0x%04x to offset 0x%03x (%s)\n",
				p16, entry->offset, entry->name);
			pci_write_config_word(dev, entry->offset, p16);
		}
		break;

	case 4:
		p32 = (((uint32_t *) data)[entry->offset/4]);
		pci_read_config_dword(dev, entry->offset, &i32);
		if (i32 != p32) {
			gim_info("Restoring 0x%08x to offset 0x%03x (%s)\n",
				p32, entry->offset, entry->name);
			pci_write_config_dword(dev, entry->offset, p32);
		}
		break;
	}
	pci_read_config_dword(dev, UNCORR_ERR_STATUS, &error);
	if (error != 0x0) {
		/* Clear all error bits */
		pci_write_config_dword(dev, UNCORR_ERR_STATUS, error);
		pci_read_config_dword(dev, UNCORR_ERR_STATUS, &error);
	}
}

void clear_all_errors(struct pci_dev *dev)
{
	uint16_t     data_16;
	uint32_t     data_32;

	/* Clear any pending statis in PCI_STATUS (offset 0x06) */
	pci_read_config_word(dev, PCI_STATUS, &data_16);
	data_16 &= 0xF900;  /* Isolate resettable error bits */
	if (data_16)
		pci_write_config_word(dev, PCI_STATUS, data_16);

	/* Clear DevStatus (offset 0x62) */
	pci_read_config_word(dev, PCI_EXP+PCI_EXP_DEVSTA, &data_16);
	data_16 &= 0x000F;  /* Isolate resettable error bits */
	if (data_16)
		pci_write_config_word(dev, PCI_EXP+PCI_EXP_DEVSTA, data_16);

	/* Clear uncorrectable error status */
	pci_read_config_dword(dev, PCI_AER+PCI_ERR_UNCOR_STATUS, &data_32);
	if (data_32) {
		pci_write_config_word(dev,
			PCI_AER+PCI_ERR_UNCOR_STATUS, data_32);
	}


}

void pci_config_restore(struct pci_dev *dev, uint8_t *buf, int count)
{
	int   i;
	uint32_t    *p_dword;
	uint32_t    data;
	uint32_t    cur_val;

	p_dword = (uint32_t *) (buf);

	check_for_error(dev, "R1");
	pci_config_save(dev, debug_config_space, 4096);
	compare_pci_config(buf, debug_config_space,
		"Before restoring PCIe config Space");


	pci_read_config_dword(dev, 0x10, &data);
	gim_info("BAR0 = 0x%08x before restore\n", data);

	pci_read_config_dword(dev, 4, &data);
	gim_info("Status/Command = 0x%08x before restore\n", data);

	check_for_error(dev, "R2");
	gim_info("Start Restore...\n");
	p_dword = (uint32_t *) (buf);

	/*
	 * Clear error registers
	 */
	clear_all_errors(dev);
	check_for_error(dev, "R3");

	/* Works for PF (Merge code later) */
	if (dev->is_physfn) {
		gim_info("Restore PF config space");

		/* Disable MEM_ACCESS so that the BARs can be updated */
		gim_info("Disable MEM_ACCESS so that the BARs can be updated\n");
		pci_read_config_dword(dev, 4, &data);
		data = 0x0404;
		pci_write_config_word(dev, 4, data);

		for (i = 0; restore_tbl[i].name != NULL; ++i)
			restore_reg(dev, buf, restore_tbl, i);

	} else {
		gim_info("Restore VF config space");
		
		/* Don't try to restore SRIOV or GPUIOV spaces */
		count = 330;
		for (i = 0x0; i < count; i += 4) {
			p_dword = &(((uint32_t *) buf)[i/4]);
			data = *p_dword;

			if (i == 0x04) {
				gim_info("Skip restoring offset 0x%03x, value would have been 0x%08x\n",
						i, *p_dword);
			} else {
				pci_read_config_dword(dev, i, &cur_val);
				if (cur_val != *p_dword) {
					gim_info("Config [0x%03x] = 0x%08x",
								i, cur_val);
					gim_info(" - restore 0x%08x\n",
								*p_dword);
					pci_write_config_dword(dev,
								i, *p_dword);
				}
			}

			pci_read_config_dword(dev, i, &data);
			++p_dword;
		}
		/* Restore Command [MEM_ACCESS_EN] */
		gim_info("Write 0x%04x to offset 0x%03x for VF to restore MEM_ACCESS_EN\n",
				0x0407, 4);
		pci_write_config_word(dev, 4, 0x0407);
	}

	gim_info("Restore complete\n");

	pci_read_config_dword(dev, 4, &data);
	gim_info("Status/Command = 0x%08x after restore\n", data);

	pci_read_config_dword(dev, 0x10, &data);
	gim_info("BAR0 = 0x%08x after restore\n", data);

}

int check_for_error(struct pci_dev *dev, char *comment)
{
	uint32_t    data;

	pci_read_config_dword(dev, 0x154, &data);
	if (data != 0) {
		gim_info("Uncorrectable error found 0x%08x, tag = \"%s\"\n",
				data, comment);
		/* Clear all error bits */
		pci_write_config_dword(dev, 0x154, data);
		pci_read_config_dword(dev, 0x154, &data);
		if (data != 0x0)
			gim_info("Can't clear the error\n");
		return 1;
	}
	return 0;
}

void pci_disable_error_reporting(struct pci_dev *dev)
{
	uint32_t corr, uncorr;

	gim_info("Disable error reporting for device: %02x:%x.%x\n",
			dev->bus->number,
			(dev->devfn>>3),
			(dev->devfn & 0x7));
	/* pci_write_config_dword (dev, PCIE_CORR_ERR_MASK, 0x071c0); */
	pci_read_config_dword(dev, PCIE_CORR_ERR_MASK, &corr);
	pci_read_config_dword(dev, PCIE_UNCORR_ERR_MASK, &uncorr);
	gim_info("Mask before -> corr = 0x%08x, uncorr = 0x%08x\n",
		corr, uncorr);
	pci_write_config_dword(dev, PCIE_CORR_ERR_MASK, 0xffffffff);
	pci_write_config_dword(dev, PCIE_UNCORR_ERR_MASK, 0x0ffffffff);

	/* Clear any errors previously reported. */
	pci_write_config_dword(dev, PCIE_CORR_ERR_STATUS, 0x0);
	pci_write_config_dword(dev, PCIE_UNCORR_ERR_STATUS, 0x0);

	pci_read_config_dword(dev, PCIE_CORR_ERR_MASK, &corr);
	pci_read_config_dword(dev, PCIE_UNCORR_ERR_MASK, &uncorr);
	gim_info("Mask after -> corr = 0x%08x, uncorr = 0x%08x\n",
		corr, uncorr);

}

int pci_check_for_error(struct pci_dev *dev)
{
	uint32_t    data;

	pci_read_config_dword(dev, 0x154, &data);
	if (data != 0) {
		gim_info("Uncorrectable error found 0x%08x\n", data);
		/* Clear all error bits */
		pci_write_config_dword(dev, 0x154, data);
		pci_read_config_dword(dev, 0x154, &data);
		if (data != 0x0)
			gim_info("Can't clear the error\n");
		return 1;
	}
	return 0;
}

/*
 * Pass in the bridge_list pre-populated with vendor and device ids
 * Fill in the aer_list of all devices that match on the list
 */
void pci_cache_bridges(struct pci_device_id *bridge, int max_size,
			struct aer_item *aer_list)
{
	int i = 0;
	int count = 0;
	struct pci_dev *dev;

	while (bridge[i].vendor != 0 && bridge[i].device != 0) {
		dev = NULL;
		gim_info("Look for BRIDGE vendor 0x%04x, device 0x%04x\n",
			bridge[i].vendor, bridge[i].device);
		while ((dev = pci_get_device(bridge[i].vendor,
			bridge[i].device, dev))) {
			gim_info("bridge device found: %02x:%x.%x\n",
					dev->bus->number,
					(dev->devfn>>3),
					(dev->devfn & 0x7));
			aer_list[count].aer_offset = bridge[i].driver_data;
			aer_list[count++].dev = dev;
		}
		++i;
	}
}

void pci_cache_pfs(struct pci_dev *pf_devices[], int pf_count, int max_bridges,
			struct aer_item *device_list)
{
	int i = 0;

	while (device_list[i].aer_offset != 0)
		++i;

	--i;
	while (pf_count) {
		--pf_count;
		device_list[++i].dev = pf_devices[pf_count];
		device_list[i].aer_offset = 0x150;
	}
	gim_info("Cache contains %d devices\n", i+1);
}

void pci_disable_bridge_error_reporting(struct aer_item *aer_list)
{
	int i = 0;
	struct pci_dev *dev;
	uint32_t corr, uncorr;

	while (aer_list[i].aer_offset != 0) {
		dev = aer_list[i].dev;
		gim_info("Write 0xffffffff to offset %x in config space of device %02x:%x.%02x\n",
			aer_list[i].aer_offset, dev->bus->number,
			(dev->devfn>>3), (dev->devfn & 0x7));

		pci_read_config_dword(dev, aer_list[i].aer_offset+0x14, &corr);
		pci_read_config_dword(dev, aer_list[i].aer_offset+0x08,
						&uncorr);
		gim_info("Mask before -> corr = 0x%08x, uncorr = 0x%08x\n",
				corr, uncorr);
		pci_write_config_dword(dev, aer_list[i].aer_offset+0x08,
				0xffffffff);
		pci_write_config_dword(dev, aer_list[i].aer_offset+0x14,
				0xffffffff);
		pci_read_config_dword(dev, aer_list[i].aer_offset+0x14, &corr);
		pci_read_config_dword(dev, aer_list[i].aer_offset+0x08,
				&uncorr);
		gim_info("Mask after ->  corr = 0x%08x, uncorr = 0x%08x\n",
				corr, uncorr);
		/* Clear the status */
		pci_write_config_dword(dev, aer_list[i].aer_offset+0x04, 0);
		pci_write_config_dword(dev, aer_list[i].aer_offset+0x10, 0);
		++i;
	}
}

/*
 * pci_validate_devices() - vaidate pci devices
 * return	1: all devices are valid
 * 		0: one device has disappeared
 */
int pci_validate_devices(struct aer_item *device_list)
{
	int i = 0;
	uint16_t vendor;
	struct pci_dev *dev;

	while (device_list[i].aer_offset != 0) {
		dev = device_list[i].dev;
		pci_read_config_word(dev, 0, &vendor);
		gim_dbg("Device %02x:%x.%x has vendor id = 0x%0x\n",
			dev->bus->number, (dev->devfn>>3),
			(dev->devfn & 0x7), vendor);
		if (vendor == 0 || vendor == 0xffff) {
			gim_err("Device %02x:%x.%x has disappeared with vendor id = 0x%0x\n",
			dev->bus->number, (dev->devfn>>3),
			(dev->devfn & 0x7), vendor);
			return 0;
		}
		++i;
	}
	return 1;
}
