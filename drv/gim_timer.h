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

#ifndef _GPU_IOV_MODULE__TIMER_H

#define _GPU_IOV_MODULE__TIMER_H

#define MS_TO_NS(x)     (x * 1000000L)
#define US_TO_NS(x)     (x * 1000L)

int set_timeout_timer(struct hrtimer *timer);
int set_timer(struct hrtimer *timer);
void delete_timer(struct hrtimer *timer);
int start_timer(struct hrtimer *timer, int delay_in_ms);
int restart_timer(struct hrtimer *timer);

#ifdef CONFIG_GIM_HEARTBEAT_TIMER
extern int in_trigger_world_switch;
extern int in_world_switch;

int start_heartbeat_timer(int delay_in_sec);
void delete_heartbeat_timer(void);
int init_heartbeat_timer(void);
#endif
extern uint32_t complete_time[8];


#define TRACE_SCHED_BUG

#endif
