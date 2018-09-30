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

#ifndef COMMAND_H_INCLUDED
#define COMMAND_H_INCLUDED

/* cmd structure */
typedef struct cmd_entry {
    char   *command;
    int    (*pAction) (char *tail);
} cmd_entry_t;

extern int cmd_gru_help(char *param);
extern int cmd_gru_list(char *param);
extern int cmd_gru_status(char *param);
extern int cmd_gru_bios(char *param);
extern int cmd_gru_open(char *param);
extern int cmd_gru_quit();

extern int cmd_gpu_help(char *param);
extern int cmd_gpu_list(char *param);
extern int cmd_gpu_get(char *param);
extern int cmd_gpu_release(char *param);
extern int cmd_gpu_status(char *param);
extern int cmd_gpu_reset(char *param);
extern int cmd_gpu_open(char *param);
extern int cmd_gpu_exit(char *param);

extern int cmd_vf_help(char *param);
extern int cmd_vf_status(char *param);
extern int cmd_vf_clear(char *param);
extern int cmd_vf_reset(char *param);
extern int cmd_vf_exit(char *param);

#endif
