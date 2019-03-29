# GIM

```javascript
Copyright (c) 2014-2019 Advanced Micro Devices, Inc. All rights reserved.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE
```
## What is GIM?
 [GIM](https://github.com/GPUOpen-LibrariesAndSDKs/MxGPU-Virtualization) 
 (GPU-IOV Module) is a Linux kernel module for AMD SR-IOV based HW
 Virtualization (MxGPU) product. It can support KVM, open source Xen and
 any other Linux kernel based hypervisors with necessary kernel compatibility
 modification. GIM is reponsible for: 
 * GPU IOV initialization
 * Virtual function configuration and enablement
 * GPU scheduling for world switch
 * Hang detection and virtual function level reset (FLR)
 * PF/VF hand shake and other GPU utilities.

 Currently, only AMD S7150 series GPUs are supported.

## DOCUMENTATION:
 All documents are listed in SRC_ROOT/docs
 
## SOFTWARE REQUIREMENTS:
 * The tested host OS for GIM is Ubuntu16.04.2. All other
 hypervisor SW(KVM, XEN, QEMU, LIBVIRT) versions are aligned with default
 version of OS.

 GIM supports KVM in Ubuntu 16.04, and supports XEN in CentOS 7.3.

 Host OS     | Kernel            | KVM/Xen | QEMU  | libvirt
 ------------|-------------------|---------|-------|--------------
 Ubuntu 16.04.2 server | 4.4.0-75-generic  | KVM 4.4 | 2.5.0 | 1.3.1  |


 * The tested guest OS

 Guest OS  |  Distributions                                 
 ----------|-------------------------------------------------
 Linux     |  Ubuntu 16.04 LTS 64bit; CentOS 7.3 64 bit       
 Windows   |  Win7 64bit; Win10 TH2 64bit                   
  

 * Some legacy Linux kernels have issues with enabling PCI SR-IOV.
   It is suggested to use Ubuntu 4.4.0-75-generic kernel and apply the patch for 
   IOV module. The patch file can be found under SRC_ROOT/patch.

## HOW TO BUILD & INSTALL:
 All driver source codes are under SRC_ROOT/drv.
 1. Typing command "make" in terminal under SRC_ROOT/drv can generate gim.ko.
 2. Typing command "make install" in terminal under SRC_ROOT/drv can install
   gim.ko to /lib/modules/$(KERNELRELEASE)/GIM/.
 3. Generally, Just run helper SRC_ROOT/gim.sh in a command terminal also can
   completed build and installation. And gim.sh -help can display usages.
 4. Blacklist amdgpu and amdkfd and reboot the server. Such as, under Ubuntu, 
   by adding the following line to the end of file /etc/modprobe.d/blacklist.conf
   
   ```
   blacklist amdgpu
   blacklist amdkfd
   ```
## HOW TO LOAD:
 1. Typing command "modprobe gim" in terminal can load gim driver
 2. Usually, Typing command "lsmod | grep gim " and "lspci | grep AMD" in
   terminal can help to check if gim driver is loaded

## HOW TO CONFIGURE BUILD:
  Read this section carefully. New configuration options could be added in
  each release, and unexpected problems can occur if the configuration files are
  not set up as expected.

  * Alternative configuration commands are
 
        CONFIG_GIM_HEARTBEAT_TIMER
                Heartbeat timer provides a useful way to display statistics
                of GIM periodically for debug purpose.
                Currently, it only counts the number of scheduler timer ticks
                and world switches, if the configuration is defined.

 * Edit configuraiton in Makefile
 
        GIM is an external kernel driver module. The driver configuration is
        different from upstream kernel driver. Editing "KBUILD_CFLAGS" in
        the Makefile is preferred to pass the configuration to GIM.

## HISTORY:
 - 1.0 (2017/07/20)
        The original release support AMD S7150 series.

 - 2.0 (2018/9/30)
        Fix some issues for AMD S7150 series.

 - 3.0 (2019/3/28)
	Adding support for mixed guest OS.
	Fix some issues for AMD S7150 series.
