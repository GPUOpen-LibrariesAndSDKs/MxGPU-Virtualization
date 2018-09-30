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
#include "command.h"
#include "hw_context.h"

cmd_entry_t gru_command_table[] = {
    {"help", cmd_gru_help},
    {"list", cmd_gru_list},
    {"status", cmd_gru_status},
    {"bios", cmd_gru_bios},
    {"open", cmd_gru_open},
    {"quit", cmd_gru_quit},
    {NULL, NULL}
};

cmd_entry_t gpu_command_table[] = {
    {"help", cmd_gpu_help},
    {"list", cmd_gpu_list},
    {"get", cmd_gpu_get},
    {"release", cmd_gpu_release},
    {"status", cmd_gpu_status},
    {"reset", cmd_gpu_reset},
    {"open", cmd_gpu_open},
    {"exit", cmd_gpu_exit},
    {NULL, NULL}
};

cmd_entry_t vf_command_table[] = {
    {"help", cmd_vf_help},
    {"status", cmd_vf_status},
    {"clear", cmd_vf_clear},
    {"reset", cmd_vf_reset},
    {"exit", cmd_vf_exit},
    {NULL, NULL}
};

static char * dupstr (char* s) {
    char *r;

    r = (char*) xmalloc ((strlen (s) + 1));
    strcpy (r, s);
    return (r);
}

static char* command_generator(const char* text, int state)
{
    static int list_index, len;
    cmd_entry_t *command_table;
    char *name;

    if (!state) {
        list_index = 0;
        len = strlen (text);
    }

    if (is_gpu_opened()) {
        if (is_vf_opened()) {
            command_table = vf_command_table;
        } else {
            command_table = gpu_command_table;
        }
    } else {
        command_table = gru_command_table;
    }

    while (name = command_table[list_index].command) {
        list_index++;

        if (strncmp (name, text, len) == 0)
            return (dupstr(name));
    }

    /* If no names matched, then return NULL. */
    return ((char *)NULL);

}

static char** command_completion(const char *text, int start, int end)
{
    char **matches;

    matches = (char **)NULL;

    if (start == 0)
        matches = rl_completion_matches ((char*)text, &command_generator);

    return (matches);

}

static int *parse_command(char *cmd, cmd_entry_t *cmdtable)
{
    cmd_entry_t *current;
    char *action, *tail, *parms, *comment;

    if ((cmd == NULL)) {
        return RET_ERROR;
    }

    action = tail = parms = comment = NULL;

    if ((comment = strchr (cmd, '#')) != NULL) {
        comment[0] = '\n';
        comment[1] = '\0';
    }

    if ((action = strtok (cmd, " \t\r\n")) == NULL) {
        return RET_ERROR;
    }

    if ((tail = strtok (NULL, "\r\n")) != NULL) {
        parms = tail + strspn (tail, " \t");
    }

    for (current = cmdtable; current->command; current++) {
        if (strcasecmp (action, current->command) == 0) {
            return (current->pAction (parms));
        }
    }

    return RET_ERROR;
}

static int process_input(void)
{
    int ret;
    char *input;
    rl_attempted_completion_function = command_completion;
    char prompt[30];
    char *token;

    do {
        if (is_gpu_opened()) {
            char cur_gpu[13];
            char *gpu_bf;

            strcpy(cur_gpu, get_current_gpu());
            token = strtok(cur_gpu, ":");
            gpu_bf = strtok(NULL, "\n");
            if (is_vf_opened()) {
                char cur_vf[13];
                char *vf_bf;
                strcpy(cur_vf, get_current_vf());
                token = strtok(cur_vf, ":");
                vf_bf = strtok(NULL, "\n");
                sprintf(prompt, "GRU>GPU:%s>VF:%s> ", gpu_bf, vf_bf);
                input = readline(prompt);
                rl_bind_key('\t',rl_complete);

                if (!input) {
                    return RET_ERROR;
                }
                add_history(input);
                ret = parse_command(input, vf_command_table);
                if (ret == RET_ERROR) {
                    GRU_LOG("Error: Invalid command");
                }
            } else {
                sprintf(prompt, "GRU>GPU:%s> ", gpu_bf);
                input = readline(prompt);
                rl_bind_key('\t',rl_complete);

                if (!input) {
                    return RET_ERROR;
                }
                add_history(input);
                ret = parse_command(input, gpu_command_table);
                if (ret == RET_ERROR) {
                    GRU_LOG("Error: Invalid command");
                }
            }
        } else {
            input = readline("GRU> ");
            rl_bind_key('\t',rl_complete);

            if (!input) {
                return RET_ERROR;
            }
            add_history(input);
            ret = parse_command(input, gru_command_table);
            if (ret == RET_ERROR) {
                GRU_LOG("Error: Invalid command");
            }
        }
    } while (ret != RET_EXIT);

    return ret;
}

static void handle_signal(void)
{
    exit(0);
}

int main()
{
    int ret;
    using_history();
    if (signal(SIGINT, handle_signal) == SIG_ERR) {
        GRU_LOG("Failed to register interrupts with kernel");
    }

    printf("\nGRU\n");
    printf("Copyright (C) 2017~2018  Advanced Micro Devices, Inc.\n\n");
    printf("Type 'help' for help. Optional launch parameter is index of card to use.\n");

    ret = detect_device();
    if (ret != RET_OK)
    {
        GRU_LOG("Detect device failed\n");
        return RET_ERROR;
    }

    while (ret != RET_EXIT) {
        ret = process_input();
    }

    exit(0);
}
