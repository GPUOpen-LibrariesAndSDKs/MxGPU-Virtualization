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

#include "gim_kcl_pci.h"
#include "gim_debug.h"
#include "gim_interface.h"

int kcl_pci_read_config_byte(
	kcl_pci_dev_handle dev, kcl_type_u16 where, kcl_type_u8 *val_ptr)
{
	return pci_read_config_byte((struct pci_dev *)dev, where, val_ptr);
}

int kcl_pci_read_config_word(
	kcl_pci_dev_handle dev, kcl_type_u16 where, kcl_type_u16 *val_ptr)
{
	return pci_read_config_word((struct pci_dev *)dev, where, val_ptr);
}

int kcl_pci_read_config_dword(
	kcl_pci_dev_handle dev, kcl_type_u16 where, kcl_type_u32 *val_ptr)
{
	return pci_read_config_dword((struct pci_dev *)dev, where, val_ptr);
}

int kcl_pci_write_config_byte(
	kcl_pci_dev_handle dev, kcl_type_u16 where, kcl_type_u8 val)
{
	return pci_write_config_byte((struct pci_dev *)dev, where, val);
}

int kcl_pci_write_config_word(
	kcl_pci_dev_handle dev, kcl_type_u16 where, kcl_type_u16 val)
{
	return pci_write_config_word((struct pci_dev *)dev, where, val);
}

int kcl_pci_write_config_dword(
	kcl_pci_dev_handle dev, kcl_type_u16 where, kcl_type_u32 val)
{
	return pci_write_config_dword((struct pci_dev *)dev, where, val);
}

void kcl_pci_enable_bus_master(kcl_pci_dev_handle dev)
{
	unsigned short cmd = 0;

	kcl_pci_read_config_word(dev, PCI_COMMAND, &cmd);
	cmd |= PCI_COMMAND_MASTER;
	kcl_pci_write_config_word(dev, PCI_COMMAND, cmd);
}

void kcl_pci_disable_bus_master(kcl_pci_dev_handle dev)
{
	unsigned short cmd = 0;

	kcl_pci_read_config_word(dev, PCI_COMMAND, &cmd);
	cmd &= (~PCI_COMMAND_MASTER);
	kcl_pci_write_config_word(dev, PCI_COMMAND, cmd);
}

int kcl_pci_find_capability(kcl_pci_dev_handle dev,
							int cap)
{
	return pci_find_capability((struct pci_dev *)dev, cap);
}

int kcl_pci_find_ext_capability(kcl_pci_dev_handle dev,
						int cap, int pos)
{
	return pci_find_ext_capability(dev, cap);
}

