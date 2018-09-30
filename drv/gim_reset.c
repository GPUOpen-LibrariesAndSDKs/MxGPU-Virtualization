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

#include "gim_adapter.h"
#include "gim_kcl_pci.h"
#include "gim_kcl_os.h"
#include "gim_irqmgr.h"
#include "gim_reset.h"
#include "gim_debug.h"
#include "gim_s7150_reg.h"
#include "gim_os_service.h"
#include "gim_atombios.h"

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
/* Need to restore the BAR after writing the Resizeable Bar Enhanced
 * Capability
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
	{CAP_SRIOV+0x28, 4, "SR-IOV VF BAR1"},
	{CAP_SRIOV+0x2c, 4, "SR-IOV VF BAR2"},
	{CAP_SRIOV+0x30, 4, "SR-IOV VF BAR3"},
	{CAP_SRIOV+0x34, 4, "SR-IOV VF BAR4"},
	/* Note that restoring this BAR causes system hang */
	{CAP_SRIOV+0x38, 4, "SR-IOV VF BAR5"},
	{CAP_SRIOV+0x3c, 4, "SR-IOV VF Migration State Array Offset"},
	/* Make sure VF enable bit is set AFTER setting up SRIOV capabilities.
	 */
	{CAP_SRIOV+0x08, 2, "SR-IOV Control"},

	/* 0x400 = GPU-IOV */
	{0x400+0x20, 4, "GPU-IOV Context"},
	{0x400+0x24, 2, "GPU-IOV FB Avail"},
	{0x400+0x26, 2, "GPU-IOV FB Consumed"},
	/* Note that Offset doesn't respond to word boundary cycles.
	 * Writes must be dword, otherwise FBx offset is not restored properly
	 */
	{0x400+PCI_GPUIOV_VF0_FB_SIZE, 4, "GPU-IOV FB0 Size/Offset"},
	{0x400+PCI_GPUIOV_VF1_FB_SIZE, 4, "GPU-IOV FB1 Size/Offset"},
	{0x400+PCI_GPUIOV_VF2_FB_SIZE, 4, "GPU-IOV FB2 Size/Offset"},
	{0x400+PCI_GPUIOV_VF3_FB_SIZE, 4, "GPU-IOV FB3 Size/Offset"},
	{0x400+PCI_GPUIOV_VF4_FB_SIZE, 4, "GPU-IOV FB4 Size/Offset"},
	{0x400+PCI_GPUIOV_VF5_FB_SIZE, 4, "GPU-IOV FB5 Size/Offset"},
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

static int gim_save_vf_cfg(struct adapter *adapt, struct function *vf)
{
	unsigned int idx = 0;
	unsigned int *pci_cfg = NULL;

	/* Save the vf pci cfg space */
	pci_cfg = (unsigned int *)(&vf->flr_state.pci_cfg);
	for (idx = 0; idx < VF_FLR_PCI_CONFIG_SIZE; idx += 4) {
		kcl_pci_read_config_dword(vf->pci_dev, idx, pci_cfg);
		pci_cfg++;
	}
	return 0;
}

static int gim_restore_vf_cfg(struct adapter *adapt, struct function *vf)
{
	unsigned int idx = 0;
	unsigned int *pci_cfg = NULL;
	unsigned int old_val = 0;

	/* Restore the vf pci cfg space */
	pci_cfg = (unsigned int *)(&vf->flr_state.pci_cfg);
	for (idx = 0; idx < VF_FLR_PCI_CONFIG_SIZE; idx += 4) {
		if (idx != 0x04) {
			kcl_pci_read_config_dword(vf->pci_dev, idx, &old_val);
			if (old_val != *pci_cfg) {
				kcl_pci_write_config_dword(vf->pci_dev, idx,
							*pci_cfg);
				gim_info("restore VF%d config[%x] = %x\n",
					vf->func_id, idx, *pci_cfg);
			}
		}
		pci_cfg++;
	}

	kcl_pci_write_config_dword(vf->pci_dev, 4,
			*((unsigned int *)(&vf->flr_state.pci_cfg + 4)));
	return 0;
}

static int gim_save_all_vfs_cfg(struct adapter *adapt)
{
	int vf_index = 0;

	for (vf_index = 0; vf_index < adapt->enabled_vfs; ++vf_index)
		gim_save_vf_cfg(adapt, adapt->vfs + vf_index);

	return 0;
}

static int gim_restore_all_vfs_cfg(struct adapter *adapt)
{
	int vf_index = 0;

	for (vf_index = 0; vf_index < adapt->enabled_vfs; ++vf_index) {
		gim_restore_vf_cfg(adapt, adapt->vfs + vf_index);

		if (!adapt->vfs[vf_index].is_available) {
			/* This variable is added to make the calling to
			 * function kcl_pci_enable_bus_master more clear
			 * to read
			 */
			struct pci_dev *pci_dev;

			pci_dev = adapt->vfs[vf_index].pci_dev;
			gim_info("Enable bus master on VF %d\n",
				adapt->vfs[vf_index].func_id);
			kcl_pci_enable_bus_master(pci_dev);
		}
	}
	return 0;
}

static int gim_save_pf_cfg(struct adapter *adapt, unsigned int *pci_cfg)
{
	int idx;

	for (idx = 0; idx < PF_FLR_PCI_CONFIG_SIZE; idx += 4) {
		kcl_pci_read_config_dword(adapt->pf.pci_dev, idx, pci_cfg);

		if (idx == 0)
			gim_info("Device and vender Id %x\n", *pci_cfg);

		if (idx == adapt->gpuiov.pos + PCI_SRIOV_CTRL)
			*pci_cfg &= ~(PCI_SRIOV_CTRL_VFE | PCI_SRIOV_CTRL_MSE);

		pci_cfg++;
	}
	return 0;
}

static void restore_reg(struct adapter *adapt,
		uint8_t *data, struct pci_def *restore_table, int index)
{
	uint8_t  p8, i8;
	uint16_t p16, i16;
	uint32_t p32, i32;
	struct pci_def  *entry;

	entry = &restore_table[index];
	switch (entry->size) {
	case 1:
		p8 = (((uint8_t *) data)[entry->offset]);
		kcl_pci_read_config_byte(adapt->pf.pci_dev, entry->offset, &i8);
		if (i8 != p8) {
			gim_info("Restoring 0x%02x to offset 0x%03x (%s)\n", p8,
				entry->offset, entry->name);
			kcl_pci_write_config_byte(adapt->pf.pci_dev,
						entry->offset, p8);
		}
		break;

	case 2:
		p16 = (((uint16_t *) data)[entry->offset/2]);
		kcl_pci_read_config_word(adapt->pf.pci_dev,
					entry->offset, &i16);
		if (i16 != p16) {
			gim_info("Restoring 0x%04x to offset 0x%03x (%s)\n",
				p16, entry->offset, entry->name);
			kcl_pci_write_config_word(adapt->pf.pci_dev,
						entry->offset, p16);
		}
		break;

	case 4:
		p32 = (((uint32_t *) data)[entry->offset/4]);
		kcl_pci_read_config_dword(adapt->pf.pci_dev, entry->offset,
					&i32);
		if (i32 != p32) {
			gim_info("Restoring 0x%08x to offset 0x%03x (%s)\n",
				p32, entry->offset, entry->name);
			kcl_pci_write_config_dword(adapt->pf.pci_dev,
						entry->offset, p32);
		}
		break;
	}
}

void gim_restore_pf_sriov_cfg(struct adapter *adapt, unsigned int *pci_cfg)
{
	int idx;

	for (idx = 0; restore_tbl[idx].name != NULL; ++idx) {
		if (restore_tbl[idx].offset < CAP_SRIOV)
			continue;

	restore_reg(adapt, (uint8_t *)pci_cfg, restore_tbl, idx);
	}
}

int gim_restore_pf_cfg(struct adapter *adapt, unsigned int *pci_cfg)
{
	int idx;
	/* Disable MEM_ACCESS so that the BARs can be updated
	 * PCI_COMMMAND_MASTER + PCI_COMMAND_INTX_DISABLE
	 */
	kcl_pci_write_config_word(adapt->pf.pci_dev, 0x4, 0x0404);
	for (idx = 0; restore_tbl[idx].name != NULL; ++idx) {
	/* Do not restore SRIOV before VBIOS post*/
		if (restore_tbl[idx].offset >= CAP_SRIOV)
			break;

		restore_reg(adapt, (uint8_t *)pci_cfg, restore_tbl, idx);
	}

	kcl_pci_write_config_word(adapt->pf.pci_dev, 0x4, 0x0007);

	/* Restore mmCONFIG_MEMSIZE*/
	pf_write_register(adapt, mmCONFIG_MEMSIZE,
			(unsigned int)*((unsigned short *)pci_cfg
			+ (adapt->gpuiov.pos
			+ PCI_GPUIOV_TOTAL_FB_AVAILABLE) / 2));
	return 0;
}

void gim_clear_all_errors(struct adapter *adapt)
{
	unsigned short data_16;
	unsigned int   data_32;
	int            pos_pcie_cap = 0;
	int            pos_aer = -1;

	/* Clear any pending status in PCI_STATUS (offset 0x06) */
	kcl_pci_read_config_word(adapt->pf.pci_dev, PCI_STATUS, &data_16);
	data_16 &= 0xF900;  /* Isolate resettable error bits */
	if (data_16) {
		gim_info("PCI status = 0x%x\n", data_16);
		kcl_pci_write_config_word(adapt->pf.pci_dev, PCI_STATUS,
					data_16);
	}

	pos_pcie_cap = kcl_pci_find_capability(adapt->pf.pci_dev,
						PCI_CAP_ID_EXP);
	gim_info("PCIE cap pos %x\n", pos_pcie_cap);


	pos_aer = kcl_pci_find_ext_capability(adapt->pf.pci_dev,
						PCI_EXT_CAP_ID_AER, pos_aer);
	gim_info("AER ext cap pos %x\n", pos_aer);

	/* Clear DevStatus (offset 0x62) */
	kcl_pci_read_config_word(adapt->pf.pci_dev, pos_pcie_cap+PCI_EXP_DEVSTA,
				&data_16);
	data_16 &= 0x000F;  /* Isolate resettable error bits */
	if (data_16) {
		gim_info("DevStatus = 0x%x\n", data_16);
		kcl_pci_write_config_word(adapt->pf.pci_dev,
					pos_pcie_cap+PCI_EXP_DEVSTA, data_16);
	}

	/* Clear uncorrectable error status (offset 0x154) */
	kcl_pci_read_config_dword(adapt->pf.pci_dev,
				pos_aer+PCI_ERR_UNCOR_STATUS, &data_32);
	if (data_32) {
		gim_info("PCIE unrecoverable error = 0x%x\n", data_32);
		kcl_pci_write_config_dword(adapt->pf.pci_dev,
					pos_aer+PCI_ERR_UNCOR_STATUS, data_32);
	}

	/* Clear the correctable error status(offset 0x160) */
	kcl_pci_read_config_dword(adapt->pf.pci_dev,
			pos_aer+PCI_ERR_CORR_STATUS, &data_32);
	if (data_32) {
		gim_info("PCIE unrecoverable error = 0x%x\n", data_32);
		kcl_pci_write_config_dword(adapt->pf.pci_dev,
					pos_aer+PCI_ERR_CORR_STATUS, data_32);
	}
}

int validate_link_status(struct adapter *adapt)
{
	/* Search PCI_CAP_ID_EXP */
	int pos = 0;
	unsigned int data = 0;
	unsigned int data1 = 0;
	unsigned int timeout = 0;
	unsigned short vender_id = 0;
	unsigned short pci_command = 0;

	pos = pci_find_capability(adapt->p2p_bridge_dev, PCI_CAP_ID_EXP);

	/* PCI_CAP_ID_EXP found? */
	if (pos) {
		gim_info("PCIE cap %x\n", pos);

		kcl_pci_read_config_dword(adapt->p2p_bridge_dev,
					pos + PCIE_LINK_CAP_OFFSET, &data);
		kcl_pci_read_config_dword(adapt->p2p_bridge_dev,
					pos + PCIE_LINK_CAP2_OFFSET, &data1);

		gim_info("PCIE_LINK_CAP_OFFSET %x\n", data);
		gim_info("PCIE_LINK_CAP2_OFFSET %x\n", data1);

		if (((data & PCIE_LINK_CAP__LINK_SPEED) == 3) &&
		   (((data1 & PCIE_LINK_CAP2__SUPPORTED_LINK_SPEEDS_VECTOR_MASK)
		   >> PCIE_LINK_CAP2__SUPPORTED_LINK_SPEEDS_VECTOR__SHIFT) &
		   PCIE_LINK_CAP2__SUPPORTED_LINK_SPEEDS_VECTOR__8_0)) {
			/* 8.0 GT/s supported */
			if (data &
				/* cut the redundant words */
				PCIE_LINK_CAP__LINK_ACTIVE_REPORT_CAPABLE) {
				timeout = PCIE_TRAINING_TIMEOUT_LIMIT;
				while (timeout)	{
					/* pci_dev and offPos are added to make
					 * the calling to function
					 * kcl_pci_read_config_dword easier to
					 * read
					 */
					struct pci_dev *pci_dev;
					int offPos;

					pci_dev = adapt->p2p_bridge_dev;
					offPos = pos + PCIE_LINK_STATUS_OFFSET;
					kcl_pci_read_config_dword(pci_dev,
							offPos, &data);
					if (data &
					    PCIE_LINK_STATUS__LINK_ACTIVE)
						break;

					/* Spin wait 20 us for each
					 *check to avoid tight loop
					 */
					udelay(20);
					timeout -= 20;

				}

				if (timeout == 0) {
					gim_info("Failed to get PCIe Link active,link status %x\n",
						data);
					return -1;
				}

				timeout = 0;
				gim_info("Validate link status done, linkSpeed 3, PCIe link active capable\n");

			} else {
				timeout = 100 * 1000; /* Delay 100ms */
				gim_info("PCIe link active capability not found, wait 100ms\n");
			}
		} else {
			timeout = PCIE_TRAINING_TIMEOUT_LIMIT;
			while (timeout) {
				kcl_pci_read_config_dword(adapt->p2p_bridge_dev,
							pos +
							PCIE_LINK_STATUS_OFFSET,
							&data);
				if ((data & PCIE_LINK_STATUS__LINK_TRAINING)
					== 0)
					break;
				/* Spin wait 20 us for each check to
				 * avoid tight loop
				 */
				udelay(20);
				timeout -= 20;

			}

			if (timeout == 0) {
				gim_info("Failed at link training , linkstatus %x\n",
					data);
				return -1;
			}
			gim_info("link training done\n");

			/* Delay extra 70 ms */
			timeout = 70 * 1000;
		}

		if (timeout)
			kcl_thread_sleep(timeout);

		/* Confirm GPU is up */
		timeout = PCIE_TRAINING_TIMEOUT_LIMIT;
		while (timeout) {
			kcl_pci_read_config_word(adapt->pf.pci_dev,
						PCI_VENDOR_ID, &vender_id);


			if (vender_id == PCI_VENDOR_ID_ATI)
				break;
			/* Spin wait 20 us for each check to avoid tight loop */
			udelay(20);
			timeout -= 20;

		}

		if (timeout == 0) {
			gim_info("GPU failed to appear\n");
			return -1;
		}
		gim_info("GPU is up\n");

	} else {
		gim_info("PCIE cap isn't found, pooling until GFX deviceappears\n");
		timeout = PCIE_TRAINING_TIMEOUT_LIMIT;
		while (timeout) {
			kcl_pci_read_config_word(adapt->pf.pci_dev, PCI_COMMAND,
						&pci_command);
			if (pci_command == 0)
				break;

			/* Spin wait 20 us for each check to avoid tight loop */
			udelay(20);
			timeout -= 20;

		}

		if (timeout == 0) {
			gim_info("GPU failed to appear\n");
			return -1;
		}
		gim_info("GPU is up\n");

	}
	return 0;
}

static void gim_pci_reset_bus(struct pci_dev *dev)
{
	kcl_type_u16 ctrl;

	kcl_pci_read_config_word(dev, PCI_BRIDGE_CONTROL, &ctrl);
	gim_info("pci ctrl: 0x%x\n", ctrl);

	ctrl |= PCI_BRIDGE_CTL_BUS_RESET;
	kcl_pci_write_config_word(dev, PCI_BRIDGE_CONTROL, ctrl);

	/* when sleep period is less than 20ms, it is not accurate to
	 * use msleep
	 */
	mdelay(2);

	ctrl &= ~PCI_BRIDGE_CTL_BUS_RESET;
	kcl_pci_write_config_word(dev, PCI_BRIDGE_CONTROL, ctrl);

	ssleep(1);
}


static int trigger_hot_reset(struct adapter *adapt)
{
	if (adapt->p2p_bridge_dev == NULL) {
		gim_info("Cannot trigger hot reset: PCI bridge is unavailable\n");
		return -1;
	}

	gim_info("start pci reset bus\n");
	gim_pci_reset_bus(adapt->p2p_bridge_dev);

	/* Validate link status */
	return validate_link_status(adapt);
}
static void clear_firmware_state(struct adapter *adapt)
{
	unsigned int val = 0;
	int idx = 0;

	/* Clean load status */
	pf_write_register(adapt, mmSMC_IND_INDEX_0, SOFT_REGISTERS_TABLE_28);
	val = pf_read_register(adapt, mmSMC_IND_DATA_0);
	gim_info("SOFT_REGISTERS_TABLE_28 %x\n", val);
	pf_write_register(adapt, mmSMC_IND_DATA_0, 0);

	/* Clean version number */
	for (idx = 0 ; idx < UCODE_COUNT ; ++idx) {
		pf_write_register(adapt,
					mmSMC_IND_INDEX_0,
					FW_STATE_543 + idx * TABLE_SIZE);

		val = pf_read_register(adapt, mmSMC_IND_DATA_0);
		gim_info("ucode %x version %x\n", idx, val);
		pf_write_register(adapt, mmSMC_IND_DATA_0, 0);
	}

	/* Clean drvFBAddr */
	pf_write_register(adapt,
			   mmSMC_IND_INDEX_0,
			   FW_STATE_684);
	val = pf_read_register(adapt, mmSMC_IND_DATA_0);
	gim_info("drvFbAddrHighValid %x\n", val);
	pf_write_register(adapt, mmSMC_IND_DATA_0, 0);

	pf_write_register(adapt,
			   mmSMC_IND_INDEX_0,
			   FW_STATE_685);
	val = pf_read_register(adapt, mmSMC_IND_DATA_0);
	gim_info("drvFbAddrLowValid %x\n", val);
	pf_write_register(adapt, mmSMC_IND_DATA_0, 0);

	pf_write_register(adapt, mmSMC_IND_INDEX_0, 0);
}

static int gim_enable_sriov(struct adapter *adapt)
{
	int result = 0;
	unsigned short sriov_control = 0;
	unsigned int timeout = 100 * 1000;

	kcl_pci_read_config_word(adapt->pf.pci_dev,
				adapt->gpuiov.pos + PCI_SRIOV_CTRL,
				&sriov_control);
	gim_info("sriov control = %x\n", sriov_control);

	sriov_control |= (PCI_SRIOV_CTRL_VFE | PCI_SRIOV_CTRL_MSE);
	kcl_pci_write_config_word(adapt->pf.pci_dev,
				adapt->gpuiov.pos + PCI_SRIOV_CTRL,
				sriov_control);

	kcl_thread_sleep(timeout);
	return result;
}

static void gim_set_pf_init_status(struct adapter *adapt)
{
	kcl_type_u32 init_status = 0;

	init_status = REG_SET_FIELD(init_status, VM_INIT_STATUS,
					VM_INIT_STATUS, 1);
	pf_write_register(adapt, mmVM_INIT_STATUS, init_status);
}

/*
 * gim_pci_hot_reset() - pci hot reset
 * @adapt: pointer to a vmkernel dev
 * return	0: success
 * 		1: failure
 */
int gim_pci_hot_reset(struct adapter *adapt)
{
	int ret = 0;
	unsigned char *pcicfg = NULL;
	unsigned int bif_bx_strap0 = 0;

	gim_info("PCI hot reset start...\n");

	/* Disable interrupt ring */
	ih_iv_ring_disable(adapt);
	gim_info("disable interrupt done\n");

	/* Disable timer interrupts */
	set_timer_interrupts(adapt, 0);
	gim_info("disable timer interrupts done\n");

	gim_save_all_vfs_cfg(adapt);
	gim_info("save all vfs config space done\n");
	if (!adapt->pf_flr_pci_cfg) {
		gim_err("no memory to save pf pci cfg, can't do hot reset\n");
		return -1;
	}
	pcicfg = adapt->pf_flr_pci_cfg;
	gim_save_pf_cfg(adapt, (unsigned int *)pcicfg);
	gim_info("save the pf config space done\n");
	gim_clear_all_errors(adapt);

	/* Save bif_bx strap0*/
	bif_bx_strap0 = pf_read_register(adapt, mmCC_BIF_BX_STRAP0);
	gim_info("CC_BIF_BX_STRAP0: %x\n", bif_bx_strap0);

	ret = trigger_hot_reset(adapt);
	gim_clear_all_errors(adapt);
	gim_info("Hot reset %s\n", (ret == 0) ? "done" : "failed");
	if (ret == -1)
		return -1;

	gim_restore_pf_cfg(adapt, (unsigned int *)pcicfg);
	gim_info("restore PF cfg space done\n");

	/* Restore bif_bx strap0*/
	pf_write_register(adapt, mmCC_BIF_BX_STRAP0, bif_bx_strap0);

	/* patch: clear firmware state */
	clear_firmware_state(adapt);

	gim_post_vbios(adapt, FORCE_VBIOS_POST);
	gim_clear_all_errors(adapt);
	gim_info("post VBIOS done\n");

	/* Enable SRIOV to after VBIOS */
	gim_restore_pf_sriov_cfg(adapt, (unsigned int *)pcicfg);
	gim_enable_sriov(adapt);
	gim_clear_all_errors(adapt);
	gim_info("enable SRIOV done\n");

	/* Restore all vfs config space */
	gim_restore_all_vfs_cfg(adapt);
	gim_info("restore vfs cfg space done\n");

	/* Set PF init status */
	gim_set_pf_init_status(adapt);
	gim_info("set PF init status done\n");

	/* Enable timer interrupts */
	set_timer_interrupts(adapt, 1);
	gim_info("enable timer interrupts done\n");

	/* Enable interrupt ring */
	ih_iv_ring_hw_init(adapt);
	ih_iv_ring_enable(adapt);
	ih_irq_source_enable(adapt);
	gim_info("enable interrupt ring done\n");

	/* Reset RLCV state machine */
	ret = reset_rlcv_state_machine(adapt);
	gim_info("Restore the rlcv state machine done\n");
	gim_info("PF FLR done...\n");

	return ret;
}

