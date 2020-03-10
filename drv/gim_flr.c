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

#include "gim_debug.h"
#include "gim_flr.h"
#include "gim_adapter.h"
#include "gim_pci_config.h"
#include "gim_interface.h"
#include "gim_kcl_pci.h"
#include "gim_kcl_os.h"
#include "gim_os_service.h"
#include "gim_s7150_reg.h"

static void save_msi_regs(struct pci_dev *dev, unsigned char *p)
{
	int msi_loc, i;
	kcl_type_u8 data_8;

	msi_loc = pci_find_capability(dev, PCI_CAP_ID_MSI);
	if (msi_loc) {
		for (i = 0; i <= 0xf; i++) {
			pci_read_config_byte(dev, msi_loc + i, &data_8);
			*(p + msi_loc + i) = data_8;
		}
	} else
		gim_info("can't find msi cap!\n");
}

static void restore_msi_regs(struct pci_dev *dev, unsigned char *p)
{
	int msi_loc, i;
	kcl_type_u8 data_8;

	msi_loc = pci_find_capability(dev, PCI_CAP_ID_MSI);
	if (msi_loc) {
		for (i = 0; i <= 0xf; i++) {
			data_8 = *(p + msi_loc + i);
			pci_write_config_byte(dev, msi_loc + i, data_8);
		}
	} else
		gim_info("can't find msi cap!\n");
}

static void save_scratch_memory(struct adapter *adapt, unsigned int *dest,
				unsigned int addrReg, unsigned int dataReg,
				unsigned int  offset, unsigned int length)
{
	unsigned int index = offset;

	for (index = offset; index < offset + length; index++) {
		pf_write_register(adapt, addrReg, index);
		*dest = pf_read_register(adapt, dataReg);
		dest++;
	}
}

static void restore_scratch_memory(struct adapter *adapt, unsigned int *src,
				unsigned int addrReg, unsigned int dataReg,
				unsigned int offset, unsigned int length)
{
	unsigned int index = offset;

	for (index = offset; index < offset + length; index++) {
		pf_write_register(adapt, addrReg, index);
		pf_write_register(adapt, dataReg, src[index]);
	}
}

int gim_save_cpc_state(struct adapter *adapt, unsigned int *cp_cpc_ic)
{
	cp_cpc_ic[0] = pf_read_register(adapt, mmCP_CPC_IC_BASE_LO);
	cp_cpc_ic[1] = pf_read_register(adapt, mmCP_CPC_IC_BASE_HI);
	cp_cpc_ic[2] = pf_read_register(adapt, mmCP_CPC_IC_BASE_CNTL);

	return 0;
}

int gim_restore_cpc_state(struct adapter *adapt, unsigned int *cp_cpc_ic)
{
	pf_write_register(adapt, mmCP_CPC_IC_BASE_LO,  cp_cpc_ic[0]);
	pf_write_register(adapt, mmCP_CPC_IC_BASE_HI,  cp_cpc_ic[1]);
	pf_write_register(adapt, mmCP_CPC_IC_BASE_CNTL,  cp_cpc_ic[2]);

	gim_info("CPC state is restored to addr: 0x%x%x ctrl: 0x%x\n",
			 cp_cpc_ic[1], cp_cpc_ic[0], cp_cpc_ic[2]);

	return 0;
}

int gim_save_vddgfx_state(struct adapter *adapt, struct function *vf)
{
	save_rlcv_state(vf);
	gim_info("RLCV responced SAVE_RLCV_STATE\n");

	/* re-locate the scrach addr */
	while (pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_ADDR) != 0)
		pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_DATA);

	gim_info("mmRLC_GPU_IOV_SCRATCH_ADDR = %x\n",
			 pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_ADDR));
	save_scratch_memory(adapt,
				adapt->rlcv_scratch[vf->func_id],
				mmRLC_GPU_IOV_SCRATCH_ADDR,
				mmRLC_GPU_IOV_SCRATCH_DATA,
				0,
				256);
	gim_info("RLCV scratch saved\n");

	return 0;
}

static int gim_restore_vddgfx_state(struct adapter *adapt, struct function *vf)
{
	kcl_type_u32 rlc_iov_cntl, rlc_cntl;

	pf_write_register(adapt, mmRLC_GPM_THREAD_ENABLE, 0xf);

	/* Re-enable rlcv first */
	rlc_iov_cntl = pf_read_register(adapt, mmRLC_GPU_IOV_F32_CNTL);
	rlc_iov_cntl = REG_SET_FIELD(rlc_iov_cntl,
					RLC_GPU_IOV_F32_CNTL,
					ENABLE,
					1);
	pf_write_register(adapt, mmRLC_GPU_IOV_F32_CNTL, rlc_iov_cntl);
	gim_info("RLC_V enabled\n");

	/* Re-enable RLC */
	rlc_cntl = pf_read_register(adapt, mmRLC_CNTL);
	rlc_cntl = REG_SET_FIELD(rlc_cntl, RLC_CNTL, RLC_ENABLE_F32, 1);
	pf_write_register(adapt, mmRLC_CNTL, rlc_cntl);
	gim_info("RLC_G enabled\n");

	while (pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_ADDR) != 0x0)
		pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_DATA);

	gim_info("mmRLC_GPU_IOV_SCRATCH_ADDR = %x\n",
		pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_ADDR));
	restore_scratch_memory(adapt,
				adapt->rlcv_scratch[vf->func_id],
				mmRLC_GPU_IOV_SCRATCH_ADDR,
				mmRLC_GPU_IOV_SCRATCH_DATA,
				0,
				0x29);

	while (pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_ADDR) != 0xb0)
		pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_DATA);

	gim_info("mmRLC_GPU_IOV_SCRATCH_ADDR = %x\n",
		pf_read_register(adapt, mmRLC_GPU_IOV_SCRATCH_ADDR));
	restore_scratch_memory(adapt,
				adapt->rlcv_scratch[vf->func_id],
				mmRLC_GPU_IOV_SCRATCH_ADDR,
				mmRLC_GPU_IOV_SCRATCH_DATA,
				0xb0,
				0x1f);

	load_rlcv_state(vf);
	gim_info("RLCV response LOAD_RLCV_STATE\n");

	return 0;
}

static int gim_enable_flr(struct adapter *adapt)
{
	unsigned int reg_val = 0;

	/* Program ixPCI_STRAP_MISC::STRAP_FLR_EN = 1 */
	pf_write_register(adapt, mmPCIE_INDEX, ixPCIE_STRAP_MISC);
	reg_val = pf_read_register(adapt, mmPCIE_DATA);
	gim_info("Read PCIE_STRAP_MISC = 0x%x\n", reg_val);
	reg_val |= PCIE_STRAP_MISC__STRAP_FLR_EN_MASK;
	pf_write_register(adapt, mmPCIE_DATA, reg_val);
	gim_info("Write PCIE_STRAP_MISC = 0x%x\n", reg_val);

	/* Program ixSWRST_EP_CONTROL_0::EP_FLR0_REST_EN = 1 */
	pf_write_register(adapt, mmPCIE_INDEX, ixSWRST_EP_CONTROL_0);
	reg_val = pf_read_register(adapt, mmPCIE_DATA);
	gim_info("Read SWRST_EP_CONTROL_0 = 0x0%x\n", reg_val);
	reg_val |= SWRST_EP_COMMAND_0__EP_FLR0_RESET_MASK;
	reg_val |= SWRST_EP_CONTROL_0__EP_FLR_DISABLE_CFG_RST_MASK;
	pf_write_register(adapt, mmPCIE_DATA, reg_val);
	gim_info("Write SWRST_EP_CONTROL_0 = 0x%x\n", reg_val);

	return 0;
}

static int gim_save_flr_state(struct adapter *adapt, struct function *vf)
{
	unsigned int idx = 0;
	unsigned int *pci_cfg = NULL;

	/* Save the vf pci cfg space */
	pci_cfg = (unsigned int *)(&vf->flr_state.pci_cfg);
	for (idx = 0; idx < VF_FLR_PCI_CONFIG_SIZE; idx += 4) {
		kcl_pci_read_config_dword(vf->pci_dev, idx, pci_cfg);
		pci_cfg++;
	}

	save_msi_regs(vf->pci_dev, (unsigned char *)&vf->flr_state.pci_cfg);

	/* Save PCIE_STRAP_MISC */
	pf_write_register(adapt, mmPCIE_INDEX, ixPCIE_STRAP_MISC);
	vf->flr_state.pcie_strap_misc = pf_read_register(adapt, mmPCIE_DATA);
	gim_info("Read PCIE_STRAP_MISC = 0x%x\n",
		vf->flr_state.pcie_strap_misc);

	/* Save SWRST_EP_CONTROL_0 */
	pf_write_register(adapt, mmPCIE_INDEX, ixSWRST_EP_CONTROL_0);
	vf->flr_state.swrst_ep_control_0 = pf_read_register(adapt,
							mmPCIE_DATA);
	gim_info("Read SWRST_EP_CONTROL_0 = 0x%x\n",
		vf->flr_state.swrst_ep_control_0);

	return 0;
}

static int gim_restore_flr_state(struct adapter *adapt, struct function *vf)
{
	unsigned int idx = 0;
	unsigned int *pci_cfg = NULL;

	/* Restore the vf pci cfg space */
	pci_cfg = (unsigned int *)(&vf->flr_state.pci_cfg);
	for (idx = 0; idx < VF_FLR_PCI_CONFIG_SIZE; idx += 4) {
		kcl_pci_write_config_dword(vf->pci_dev, idx, *pci_cfg);
		pci_cfg++;
	}

	restore_msi_regs(vf->pci_dev, (unsigned char *)&vf->flr_state.pci_cfg);

	/* Restore PCIE_STRAP_MISC */
	pf_write_register(adapt, mmPCIE_INDEX, ixPCIE_STRAP_MISC);
	pf_write_register(adapt, mmPCIE_DATA, vf->flr_state.pcie_strap_misc);
	gim_info("Write PCIE_STRAP_MISC = 0x%x\n",
		vf->flr_state.pcie_strap_misc);

	/* Restore SWRST_EP_CONTROL_0 */
	pf_write_register(adapt, mmPCIE_INDEX, ixSWRST_EP_CONTROL_0);
	pf_write_register(adapt, mmPCIE_DATA,
			vf->flr_state.swrst_ep_control_0);
	gim_info("Write SWRST_EP_CONTROL_0 = 0x%x\n",
		vf->flr_state.swrst_ep_control_0);

	return 0;
}

int gim_function_level_reset(struct adapter *adapt, struct function *vf)
{
	int pos = 0;
	kcl_type_u16 val = 0;
	/* Wait at least 100ms for vf_flr complete*/


	/* Enable vf flr */
	gim_enable_flr(adapt);

	pos = kcl_pci_find_capability(vf->pci_dev, PCI_CAP_ID_EXP);
	if (pos) {
		kcl_pci_disable_bus_master(vf->pci_dev);
		kcl_pci_read_config_word(vf->pci_dev,
					pos + PCI_EXP_DEVCTL,
					&val);
		gim_info("Read 0x%x @pos = 0x%x\n",
			val, (pos + PCI_EXP_DEVCTL));

		val |= PCI_EXP_DEVCTL_BCR_FLR;
		kcl_pci_write_config_word(vf->pci_dev,
					pos + PCI_EXP_DEVCTL,
					val);
		gim_info("Write 0x%x @pos = 0x%x\n",
			val, (pos + PCI_EXP_DEVCTL));

		kcl_thread_sleep(100 * 1000);
		kcl_pci_enable_bus_master(vf->pci_dev);
	} else {
		gim_info("No PCIE CAPs found\n");
		return -1;
	}
	return 0;
}

int gim_vf_flr(struct adapter *adapt, struct function *vf)
{
	unsigned int sdma0_gfx_doorbell;
	unsigned int sdma1_gfx_doorbell;
	unsigned int rlc_cgcg_cgls_ctrl;
	unsigned int cp_cpc_ic[4];

	rlc_cgcg_cgls_ctrl = pf_read_register(adapt, mmRLC_CGCG_CGLS_CTRL);

	sdma0_gfx_doorbell = pf_read_register(adapt, mmSDMA0_GFX_DOORBELL);
	sdma0_gfx_doorbell &= ~SDMA0_GFX_DOORBELL__CAPTURED_MASK;
	sdma1_gfx_doorbell = pf_read_register(adapt, mmSDMA1_GFX_DOORBELL);
	sdma1_gfx_doorbell &= ~SDMA1_GFX_DOORBELL__CAPTURED_MASK;

	gim_save_cpc_state(adapt, cp_cpc_ic);

	/* save FLR state */
	gim_save_flr_state(adapt, vf);

	do_gettimeofday(&vf->time_log.reset_time);
	vf->time_log.reset_count++;

	/* do the FLR */
	if (gim_function_level_reset(adapt, vf) != 0) {
		gim_info("FLR fails\n");
		return -1;
	}

	/* restore FLR state */
	gim_restore_flr_state(adapt, vf);

	gim_restore_vddgfx_state(adapt, vf);

	gim_restore_cpc_state(adapt, cp_cpc_ic);

	pf_write_register(adapt, mmRLC_CGCG_CGLS_CTRL, rlc_cgcg_cgls_ctrl);
	pf_write_register(adapt, mmSDMA0_GFX_DOORBELL, sdma0_gfx_doorbell);
	pf_write_register(adapt, mmSDMA1_GFX_DOORBELL, sdma1_gfx_doorbell);

	return 0;
}

