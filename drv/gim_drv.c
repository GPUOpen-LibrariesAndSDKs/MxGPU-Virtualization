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

#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/mod_devicetable.h>
#include <linux/version.h>
#include "gim_adapter.h"
#include "gim_unwrapper.h"
#include "gim_pci.h"
#include "gim_interface.h"
#include "gim_config.h"
#include "gim_timer.h"
#include "gim_debug.h"
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include "gim_pci_config.h"
#include "gim_monitor_ioctl.h"

#ifndef DRV_VERSION
#define DRV_VERSION 0.0
#endif

#define _STR(x) #x
#define STR(x) _STR(x)

char gim_driver_name[] = "gim";
char gim_driver_version[] = STR(DRV_VERSION);
static const char gim_driver_string[] =
		"GPU IOV MODULE";
static const char gim_copyright[] =
		"Copyright (c) 2014-2017 Advanced Micro Devices, Inc. All rights reserved.";


/* TBD: Tonga Device ID */
static const struct pci_device_id gim_pci_tbl[] = {
	{ PCI_VDEVICE(ATI, 0x6929), 0 },    /* Tonga S7150 'Cloudy Quark' */
	/* required last entry */
	{0, }
};



#define MAX_BRIDGES	10

struct aer_item device_list[MAX_BRIDGES];

static ssize_t sriov_store(struct device_driver *drv, const char *buf,
		size_t count)
{
	call_interface_functions(buf, count);
	return count;
}

static ssize_t sriov_show(struct device_driver *drv, char *buf)
{
	return respond_interface_functions(buf);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
static DRIVER_ATTR(sriov, (S_IWUSR|S_IRUSR), sriov_show, sriov_store);
#else
static DRIVER_ATTR_RW(sriov);
#endif


static int gim_probe(struct pci_dev *pdev,
		const struct pci_device_id *ent)
{
	set_new_adapter(pdev);
	gim_info("AMD GIM probe: pf_count = %d\n", get_adapter_count());
	return 0;
}

static void gim_remove(struct pci_dev *pdev)
{
	gim_info("AMD GIM remvoe\n");
}

static void gim_shutdown(struct pci_dev *pdev)
{
	gim_info("AMD GIM shutdown\n");
}

static pci_ers_result_t gim_io_error_detected(struct pci_dev *pdev,
		pci_channel_state_t state)
{
	gim_info("AMD GIM io error detected\n");

	/* Request a slot slot reset. */
	return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t gim_io_slot_reset(struct pci_dev *pdev)
{
	gim_info("AMD GIM io slot reset\n");
	return PCI_ERS_RESULT_RECOVERED;
}

static void gim_io_resume(struct pci_dev *pdev)
{
	gim_info("AMD GIM io resume\n");

}

static struct pci_error_handlers gim_err_handler = {
	.error_detected = gim_io_error_detected,
	.slot_reset = gim_io_slot_reset,
	.resume = gim_io_resume,
};

static struct pci_driver gim_driver = {
	.name     = gim_driver_name,
	.id_table = NULL,
	.probe    = gim_probe,
	.remove   = gim_remove,
	.shutdown = gim_shutdown,
	.err_handler = &gim_err_handler
};

static int gim_init(void)
{
	int ret = 0;
	int pf_count;


	struct pci_dev *pfDevices[MAX_ADAPTERS_IN_SYSTEM];

	gim_info("Start AMD open source GIM initialization\n");

	gim_info("%s - version %s\n",
			gim_driver_string, gim_driver_version);

	gim_info("%s\n", gim_copyright);
	init_config();

	pf_count = enumerate_all_pfs(gim_pci_tbl,
		MAX_ADAPTERS_IN_SYSTEM, pfDevices);
	if (pf_count > 0) {
		int count;

		for (count = 0 ; count < pf_count ; count++)
			gim_probe(pfDevices[count], NULL);

	}

	ret = pci_register_driver(&gim_driver);

	ret = driver_create_file(&gim_driver.driver,
			&driver_attr_sriov);


	gim_init_debug_interface();

	amdgim_create_devices(&gim_driver.driver);

#ifdef CONFIG_GIM_HEARTBEAT_TIMER
	init_heartbeat_timer();
	start_heartbeat_timer(10);  /* 10 second delay */
#endif

	return ret;
}

static void gim_exit(void)
{
	gim_info("Exit AMD open source GIM!\n");

	idle_all_adapters();
	amdgim_destroy_devices();

#ifdef CONFIG_GIM_HEARTBEAT_TIMER
	delete_heartbeat_timer();
#endif
	release_all_adapters();

	driver_remove_file(&gim_driver.driver, &driver_attr_sriov);
	pci_unregister_driver(&gim_driver);


	gim_exit_debug_interface();
}

	module_init(gim_init)
module_exit(gim_exit)

MODULE_VERSION(STR(DRV_VERSION));

MODULE_AUTHOR("Advanced Micro Devices, Inc.");
MODULE_DESCRIPTION("GPU IOV MODULE");
MODULE_LICENSE("GPL and additional rights");
