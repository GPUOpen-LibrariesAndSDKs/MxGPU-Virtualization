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

#ifndef _GPU_IOV_MODULE__GPUIOV_H
#define _GPU_IOV_MODULE__GPUIOV_H

#define PF_ID(x) (x)
#define VF_ID(x) (0x80 | x)
#define CMD_EXECUTE         0x10
#define CMD_EXECUTE_INTR_EN 0x20
#define PCI_CFG_SPACE_SIZE      256
#define PCI_CFG_SPACE_EXP_SIZE  4096

enum gpu_iov_command {
	IDLE_GPU                = 0x01,
	SAVE_GPU_STATE          = 0x02,
	LOAD_GPU_STATE          = 0x03,
	RUN_GPU                 = 0x04,
	CONTEXT_SWITCH          = 0x05,
	ENABLE_AUTO_HW_SWITCH   = 0x06,
	INIT_GPU                = 0x07,
	SAVE_RLCV_STATE         = 0x08,
	LOAD_RLCV_STATE         = 0x09
};

enum gpu_iov_command_status {
	COMMAND_DONE = 0x0,
	IDLING_GPU = 0x11,
	SAVING_GPU_STATE = 0x12,
	LOADING_GPU_STATE  = 0x13,
	ENABLING_GPU  = 0x14,
	INITING_GPU = 0x15,
	SAVING_RLCV_STATE = 0x16,
	LOADING_RLCV_STATE  = 0x17,
	GFX_HANG = 0x0e,
	SDMA_HANG = 0x0f
};

int pci_gpu_iov_init(struct pci_dev *dev, struct pci_gpu_iov *gpuiov);
int set_gpuiov_context(struct pci_dev *dev, struct pci_gpu_iov *gpuiov,
			kcl_type_u8 context_size, kcl_type_u8 context_loc,
			kcl_type_u32 context_offset);
int set_gpuiov_function_id(struct pci_dev *dev, struct pci_gpu_iov *gpuiov,
			kcl_type_u8 function_id);
int set_gpuiov_next_function_id(struct pci_dev *dev,
			struct pci_gpu_iov *gpuiov, kcl_type_u8 next_func_id);
int set_gpuiov_total_fb_consumed(struct pci_dev *dev,
			struct pci_gpu_iov *gpuiov,
			kcl_type_u16 total_fb_consumed);
int set_gpuiov_vf_frame_buffer(struct pci_dev *dev, struct pci_gpu_iov *gpuiov,
			int vf, kcl_type_u16 size, kcl_type_u16 offset);
int set_gpuiov_command(struct pci_dev *dev, struct pci_gpu_iov *gpuiov,
			int command, int func_id, int next_func_id);
int get_vf_hw_fb_settings(struct pci_dev *pf_dev, struct pci_gpu_iov *gpuiov,
			int vf_id, uint32_t *fb_start, uint32_t *fb_size);

#endif

