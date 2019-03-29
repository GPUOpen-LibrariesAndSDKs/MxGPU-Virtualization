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

obj-m := gim.o
gim-objs := gim_drv.o gim_interface.o gim_adapter.o gim_pci.o gim_unwrapper.o gim_gpuiov.o gim_config.o gim_timer.o gim_fb.o gim_debug.o gim_flr.o gim_atom.o gim_atombios.o gim_os_service.o gim_irqmgr.o gim_kcl_os.o gim_pci_config.o gim_kcl_pci.o gim_reset.o gim_dma.o gim_monitor.o gim_monitor_ioctl.o gim_monitor_tonga.o

LINUXINCLUDE := $(LINUXINCLUDE) -I$(src)/asic_reg

#GIM configuration
#ccflags-y += -DCONFIG_GIM_HEARTBEAT_TIMER
#ccflags-y += -D DRV_VERSION=1.0
#ccflags-y += -DCONFIG_MMIO_QEMU_SECURITY

PWD := $(shell pwd)

ifeq ($(KERNELRELEASE),)
KERNELDIR ?= /lib/modules/$(shell uname -r)/build

$(info $(KBUILD_CFLAGS))
all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install
endif
