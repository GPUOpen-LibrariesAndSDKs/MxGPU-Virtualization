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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>

#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/signal.h>

#include <linux/version.h>

#include "gim_adapter.h"
#include "gim_timer.h"
#include "gim_debug.h"
#include "gim_command.h"
#include "gim_kcl_os.h"
#include "gim_interface.h"
#include "gim_irqmgr.h"

enum hrtimer_restart my_timer_callback(struct hrtimer *timer)
{
	struct adapter *adapt = container_of(timer,
			struct adapter, sched_timer);

	kcl_schedule_work(&adapt->sched_work);
#ifdef CONFIG_GIM_HEARTBEAT_TIMER
	adapt->world_switch_count++;
#endif

	return HRTIMER_NORESTART;
}

enum hrtimer_restart timeout_timer_callback(struct hrtimer *timer)
{
	struct adapter *adapt = container_of(timer, struct adapter,
						timeout_timer);

	if(handle_fullaccess_timeout(adapt))
		start_timer(&adapt->timeout_timer,
					TIMEOUT_CHECK_INTERVAL);

	return HRTIMER_NORESTART;
}

int set_timeout_timer(struct hrtimer *timer)
{
	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer->function = &timeout_timer_callback;
	return 0;
}

int set_timer(struct hrtimer *timer)
{
	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer->function = &my_timer_callback;
	return 0;
}

void delete_timer(struct hrtimer *timer)
{
	int ret;

	ret = hrtimer_cancel(timer);
}

int start_timer(struct hrtimer *timer, int delay_in_us)
{
	ktime_t ktime = ktime_set(0, US_TO_NS(delay_in_us));
	hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
	return 0;
}

int restart_timer(struct hrtimer *timer)
{
	hrtimer_restart(timer);
	return 0;
}

#ifdef CONFIG_GIM_HEARTBEAT_TIMER
/*
 * Heartbeat timer provides a useful interval to display debug statistics
 * that can be accumulated in a debug version of the GIM driver.
 *
 * Currently counts the number of scheduler timer ticks and world switches.
 *
 * This is purely debug code and is controlled by the CONFIG_GIM_HEARTBEAT_TIMER define
 */

static struct hrtimer heartbeat_timer;

int start_heartbeat_timer(int delay_in_sec)
{
	ktime_t ktime = ktime_set(delay_in_sec, 0);

	hrtimer_start(&heartbeat_timer, ktime, HRTIMER_MODE_REL);
	return 0;
}

void delete_heartbeat_timer(void)
{
	delete_timer(&heartbeat_timer);
}

enum hrtimer_restart my_watchdog_callback(struct hrtimer *timer)
{
	int i;
	int num_adapters = get_adapter_count();
	struct adapter *adapt = get_adapters();
	int ws_count = 0;

	/* Check if anything running */

	for (i = 0; i < num_adapters; ++i) {
		if (adapt != NULL)
			ws_count += adapt->world_switch_count;
	}

	if (ws_count) {

		for (i = 0; i < num_adapters; ++i) {
			if (adapt != NULL) {
				gim_info("GIM Heartbeat - Adp[%d] WS = %d\n",
					i, adapt->world_switch_count);
				dump_runlist(adapt);
				adapt->world_switch_count = 0;
			} else {
				gim_info("GIM Heartbeat - Adp[%d] = NULL\n", i);
			}
			++adapt;
		}
	}
	start_heartbeat_timer(10);
	return HRTIMER_NORESTART;
}

int init_heartbeat_timer(void)
{
	hrtimer_init(&heartbeat_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	heartbeat_timer.function = &my_watchdog_callback;
	return 0;
}

#endif
