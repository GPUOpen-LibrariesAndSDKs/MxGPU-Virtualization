/*
 * Copyright 2017~2018 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "gru.h"
#include "info.h"
#include "hw_context.h"

int cmd_gru_help(char *param)
{
    message_entry_t *entry;

    for (entry = gru_message_box; entry->command; entry++){
        if ((param == NULL) || (0 == strcmp(entry->command, param))) {
            print_info(entry->message);
            return RET_OK;
        }
    }
    GRU_LOG("Invalid command!");
    return RET_ERROR;
}

int cmd_gru_list(char *param)
{
    show_gpuinfo();
    return RET_OK;
}

int cmd_gru_status(char *param)
{
    show_gpuvs();
    return RET_OK;
}

int cmd_gru_bios(char *param)
{
    show_gpubios();
    return RET_OK;
}

int cmd_gru_open(char *param)
{
    if (param == NULL) {
        GRU_LOG("Invalid parameters!");
        return RET_ERROR;
    }

    char bus_id[13];

    sscanf(param, "%s", &bus_id);

    if (strlen(bus_id) != 12) {
        GRU_LOG("Invalid GPU bus ID %s", bus_id);
        return RET_ERROR;
    }

    return open_gpu(bus_id);
}

int cmd_gru_quit(char *param)
{
    return RET_EXIT;
}

int cmd_gpu_help(char *param)
{
    message_entry_t *entry;

    for (entry = gpu_message_box; entry->command; entry++){
        if ((param == NULL) || (0 == strcmp(entry->command, param))) {
            print_info(entry->message);
            return RET_OK;
        }
    }
    GRU_LOG("Invalid command!");
    return RET_ERROR;
}

int cmd_gpu_list(char *param)
{
    list_vf();

    return RET_OK;
}

int cmd_gpu_get(char *param)
{
    if (param == NULL) {
        GRU_LOG("Invalid parameters!");
        return RET_ERROR;
    }

    char fb_start[15];
    char fb_size[15];
    char gfx_partition[15];

    sscanf(param, "%s %s %s", &fb_start, &fb_size, &gfx_partition);
    get_vf(fb_start, fb_size, gfx_partition);

    return RET_OK;
}

int cmd_gpu_release(char *param)
{
    if (param == NULL) {
        GRU_LOG("Invalid parameters!");
        return RET_ERROR;
    }

    char bus_id[13];

    sscanf(param, "%s", &bus_id);

    if (strlen(bus_id) != 12) {
        GRU_LOG("Invalid VF bus ID %s", bus_id);
        return RET_ERROR;
    }

    release_vf(bus_id);

    return RET_OK;
}


int cmd_gpu_status(char *param)
{
    gpu_status();

    return RET_OK;
}

int cmd_gpu_reset(char *param)
{
    char cmd[100];

    sprintf(cmd,"echo 1 >/sys/bus/pci/devices/%s/hotlink_reset", get_current_gpu());
    system(cmd);
    sprintf(cmd,"cat /sys/bus/pci/devices/%s/hotlink_reset", get_current_gpu());
    system(cmd);

    return RET_OK;
}

int cmd_vf_help(char *param)
{
    message_entry_t *entry;

    for (entry = vf_message_box; entry->command; entry++){
        if ((param == NULL) || (0 == strcmp(entry->command, param))) {
            print_info(entry->message);
            return RET_OK;
        }
    }
    GRU_LOG("Invalid command!");
    return RET_ERROR;
}

int cmd_vf_reset(char *param)
{
    char cmd[100];

    sprintf(cmd,"echo 1 >/sys/bus/pci/devices/%s/flr", get_current_vf());
    system(cmd);
    sprintf(cmd,"cat /sys/bus/pci/devices/%s/flr", get_current_vf());
    system(cmd);

    return RET_OK;
}

int cmd_vf_status(char *param)
{
    char cmd[50];
    sprintf(cmd,"cat /sys/bus/pci/devices/%s/gpuvf", get_current_vf());
    system(cmd);

    return RET_OK;
}

int cmd_vf_clear(char *param)
{
    if (param == NULL) {
        GRU_LOG("Invalid parameters!");
        return RET_ERROR;
    }

    char pattern[10];

    sscanf(param, "%s", &pattern);
    char cmd[100];
    sprintf(cmd,"echo %s > /sys/bus/pci/devices/%s/clrvffb", pattern, get_current_vf());
    system(cmd);
    sprintf(cmd,"cat /sys/bus/pci/devices/%s/clrvffb", get_current_vf());
    system(cmd);

    return RET_OK;
}

int cmd_gpu_exit(char *param)
{
    return close_gpu();
}

int cmd_vf_exit(char *param)
{
    return close_vf();
}

int cmd_gpu_open(char *param)
{
    if (param == NULL) {
        GRU_LOG("Invalid parameters!");
        return RET_ERROR;
    }

    char bus_id[13];

    sscanf(param, "%s", &bus_id);

    if (strlen(bus_id) != 12) {
        GRU_LOG("Invalid VF bus ID %s", bus_id);
        return RET_ERROR;
    }

    return open_vf(bus_id);
}
