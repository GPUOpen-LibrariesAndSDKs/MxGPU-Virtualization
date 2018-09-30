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

#include "info.h"
#include "gru.h"

static char *gru_help_message[] = {
    "Commands:",
    " help       - show this help file",
    " list       - list all GPUs",
    " status     - show all GPU status",
    " bios       - show all GPU bios",
    " open       - open a GPU",
    " quit       - release resources and quit\n",
    " 'help <cmd>' shows details of each command",
    "EOF",
};

static char *gpu_help_message[] = {
    "Commands:",
    " help       - show this help file",
    " list       - list VFs under the GPU",
    " get        - get a VF with specific setting",
    " release    - release VF",
    " status     - show all VF status",
    " reset      - perform hotlink reset to GPU",
    " open       - open a VF",
    " exit       - return to upper layer command\n",
    " 'help <cmd>' shows details of each command",
    "EOF",
};

static char *vf_help_message[] = {
    "Commands:",
    " help       - show this help file",
    " status     - show VF status",
    " clear      - clear VF FB",
    " reset      - perform function level reset to VF",
    " exit       - return to upper layer command\n",
    " 'help <cmd>' shows details of each command",
    "EOF",
};

static char *gru_open_message[] = {
    "open a GPU",
    " open <BusId>",
    " BusId  - the GPU bus id",
    "           use 'gpu info' to see all the GPUs",
    "EOF",
};

static char *gru_list_message[] = {
    "List all GPUs",
    " list",
    "EOF",
};

static char *gru_status_message[] = {
    "Show all GPU status",
    " status",
    "EOF",
};

static char *gru_bios_message[] = {
    "Show all GPU bios",
    " bios",
    "EOF",
};

static char *gpu_open_message[] = {
    "open a VF",
    " open <BusId>",
    " BusId  - the VF bus id",
    "           use 'list' to list all the VFs under the GPU",
    "EOF",
};

static char *gpu_list_message[] = {
    "List all VFs under the GPU:",
    " list",
    "EOF",
};

static char *gpu_get_message[] = {
    "Get a VF with specific setting:",
    " get <fb_start> <fb_size> <gfx_partition>",
    " fb_start  - the offset from the beginning of FB.",
    "             the unit is MB, and the minimum offset alignment is 16MB.",
    " fb_size   - the fb size for this VF.",
    "             the unit is MB, the minimum size is 256MB, and the minimum size alignment is 16MB.",
    " gfx_partition - the GFX engine partitioning for this VF.",
    "                 the valid value is 1, 2, 4, 8, 16. ",
    "                 it specify the VF time slice to be the GPU GFX engine’s time or 1/2, 1/4, 1/8, 1/16 of a time slice.",
    "EOF",
};

static char *gpu_status_message[] = {
    "Show all VF status:",
    " status",
    "EOF",
};

static char *vf_status_message[] = {
    "Show the VF status",
    " status",
    "EOF",
};

static char *vf_clear_message[] = {
    "Clear the VF FB with the pattern",
    " clear <pattern>",
    " pattern   - value to write to FB",
    "EOF",
};

static char *gpu_release_message[] = {
    "Release a VF. The VF which was returned from 'get'",
    " release <BusId>",
    " BusId   - VF bus ID",
    "EOF",
};

static char *gpu_reset_message[] = {
    "Perform hotlink reset to GPU",
    " reset",
    "EOF",
};

static char *vf_reset_message[] = {
    "Perform function level reset to VF",
    " reset",
    "EOF",
};

message_entry_t gru_message_box[] = {
    {"help", gru_help_message},
    {"list", gru_list_message},
    {"status", gru_status_message},
    {"bios", gru_bios_message},
    {"open", gru_open_message},
    {NULL, NULL},
};

message_entry_t gpu_message_box[] = {
    {"help", gpu_help_message},
    {"list", gpu_list_message},
    {"get", gpu_get_message},
    {"release", gpu_release_message},
    {"status", gpu_status_message},
    {"reset", gpu_reset_message},
    {"open", gpu_open_message},
    {NULL, NULL},
};

message_entry_t vf_message_box[] = {
    {"help", vf_help_message},
    {"status", vf_status_message},
    {"clear", vf_clear_message},
    {"reset", vf_reset_message},
    {NULL, NULL},
};

void print_info(char **info)
{
    if (info == NULL)
        return;

    while (strcmp(*info, "EOF")) {
        GRU_LOG("%s", *info);
        info++;
    }
}
