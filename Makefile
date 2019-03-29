#
# Copyright (c) 2014-2017 Advanced Micro Devices, Inc. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE

GIM_VERSION=1.1.4

LINUXINCLUDE := $(LINUXINCLUDE) -I$(src)/drv/asic_reg
KBUILD_CFLAGS := $(KBUILD_CFLAGS) -D DRV_VERSION=$(GIM_VERSION)

#KBUILD_CFLAGS += -DCONFIG_GIM_HEARTBEAT_TIMER=y
#KBUILD_CFLAGS += -DCONFIG_MMIO_QEMU_SECURITY=y

obj-m += gim.o
gim-objs := drv/gim_drv.o drv/gim_interface.o drv/gim_adapter.o drv/gim_pci.o drv/gim_unwrapper.o drv/gim_gpuiov.o drv/gim_config.o drv/gim_timer.o drv/gim_fb.o drv/gim_debug.o drv/gim_flr.o drv/gim_atom.o drv/gim_atombios.o drv/gim_os_service.o drv/gim_irqmgr.o drv/gim_kcl_os.o drv/gim_pci_config.o drv/gim_kcl_pci.o drv/gim_reset.o drv/gim_dma.o drv/gim_monitor.o drv/gim_monitor_ioctl.o drv/gim_monitor_tonga.o

PWD := $(shell pwd)

ifeq ($(KERNELRELEASE),)
KERNELDIR ?= /lib/modules/$(shell uname -r)/build

$(info $(KBUILD_CFLAGS))
all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
endif

