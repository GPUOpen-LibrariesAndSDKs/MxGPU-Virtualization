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

#include "hw_context.h"
#include "gru.h"

char g_device_name[10];
char g_cur_gpu[13];
char g_cur_vf[13];
bool g_gpu_opened = false;
bool g_vf_opened = false;

int detect_device(void)
{
    FILE *fp;
    char buf[100];
    char *line;
    char * token;

    fp = popen("cat /sys/bus/pci/drivers/gim/gpuinfo", "r");
    if (fp != NULL) {
        while (1) {
            // read "GPU #"
            line = fgets(buf, sizeof buf, fp);
            if (line == NULL) {
                GRU_LOG("Can't detect a device\n");
                return RET_ERROR;
            }

            if (strstr(line, "Name:") != NULL) {
                token = strtok(line, ":");
                token = strtok(NULL, "\n");
                snprintf(g_device_name, sizeof(g_device_name), "%s", token);
                pclose(fp);
                return RET_OK;
            }
        }
    }

    GRU_LOG("Open /sys/bus/pci/drivers/gim/gpuinfo failed\n");
    return RET_ERROR;
}

void show_gpubios(void)
{
    FILE *fp;
    char *line;
    char buf[100];
    char format_l1[] = "| %3d | %-18.18s | %16.16s | %11.11s | %13.13s   |\n";
    char format_l2[] = "|     | %-18.18s | %16.16s | %11.11s | %13.13s   |\n";
    int index = 0;
    int i;
    char *token;
    char *key[] = {
        "Serial ID",
        "VBIOS Pn",
        "VBIOS Build Date",
        "VBIOS Version",
        "ASIC Device ID",
        "ASIC Revision ID",
        "SubSystem Vendor ID",
        "SubSystem Device ID",
    };
    enum gpubios_key {
	GPUBIOS_Serial_ID,
	GPUBIOS_VBIOS_Pn,
	GPUBIOS_VBIOS_Build_Date,
	GPUBIOS_VBIOS_Version,
	GPUBIOS_ASIC_Device_ID,
	GPUBIOS_ASIC_Revision_ID,
	GPUBIOS_SubSystem_Vendor_ID,
	GPUBIOS_SubSystem_Device_ID,
	GPUBIOS_KEY_MAX
    };
    char value[GPUBIOS_KEY_MAX][20];

    /* gpubios table
    +-----+--------------------+------------------+-------------+-----------------+
    | GPU |     Serial ID      | VBIOS Build Date | ASIC Dev ID | Sub Vendor ID   |
    |     |     VBIOS Pn       |       Version    | ASIC Rev ID | Sub Dev ID      |
    +=====+====================+==================+=============+=================+
    |   0 | 0213f1094b1e29a4   | 2017/10/06 15:37 | 6860        | 1002            |
    |     | 113-D0513300-d104  | 16.1.1.0         | 0           | c35             |
    +-----+--------------------+------------------+-------------+-----------------+
    */

    fp = popen("cat /sys/bus/pci/drivers/gim/gpubios", "r");
    if (fp != NULL) {
        // print header
        printf("\n+-----+--------------------+------------------+-------------+-----------------+\n");
        printf("| GPU |     Serial ID      | VBIOS Build Date | ASIC Dev ID | Sub Vendor ID   |\n");
        printf("|     |     VBIOS Pn       | VBIOS Version    | ASIC Rev ID | Sub Dev ID      |\n");
        printf("+=====+====================+==================+=============+=================+\n");
        while (1) {
            // read "GPU #"
            line = fgets(buf, sizeof buf, fp);
            if (line == NULL || strstr(line, "GPU #") == NULL) {
                break;
            }

            // read Key's value
            for (i = 0; i < GPUBIOS_KEY_MAX; i++) {
                line = fgets(buf, sizeof buf, fp);

                if (line == NULL || strstr(line, key[i]) == NULL) {
                    printf("get key: %s failed\n", key[i]);
                    printf("key read: %s\n", line);
                    break;
                }
                token = strtok(line, ":");
                token = strtok(NULL, "\n");
                if (strlen(token) > 18) {
                    snprintf(value[i], sizeof(value[i]), "%s", token + strlen(token) - 18);
                } else {
                    snprintf(value[i], sizeof(value[i]), "%s", token);
                }
            }

            printf(format_l1, index, value[GPUBIOS_Serial_ID], value[GPUBIOS_VBIOS_Build_Date],
                   value[GPUBIOS_ASIC_Device_ID], value[GPUBIOS_SubSystem_Vendor_ID]);
            printf(format_l2, value[GPUBIOS_VBIOS_Pn], value[GPUBIOS_VBIOS_Version],
                   value[GPUBIOS_ASIC_Revision_ID], value[GPUBIOS_SubSystem_Device_ID]);
            printf("+-----+--------------------+------------------+-------------+-----------------+\n");

            index++;
        }
        pclose(fp);
    }

    return;
}

void show_gpuinfo(void)
{
    FILE *fp;
    char *line;
    char buf[100];
    char format_1[] = "| %3d | %-12.12s | %-7.7s | %-10.10s | %-6.6s | %-10.10s | %-9.9s |\n";
    char format_2[] = "|     | %-12.12s | %-7.7s | %-10.10s | %-6.6s | %-10.10s | %-9.9s |\n";
    int index = 0;
    int i;
    char *token;
    char *key[] = {
        "Name",
        "BusId",
        "DPM Cap",
        "Power Cap",
        "Frame Buffer Size",
        "Uncorr ECC",
        "Max VF#",
        "GFX Engine",
        "GFX MAX Clock",
        "Video Encoder",
        "PCIE Link Speed",
        "PCIE Link Width",
    };
    enum gpuinfo_key {
        GPUINFO_Name,
        GPUINFO_BusId,
        GPUINFO_DPM_Cap,
        GPUINFO_Power_Cap,
        GPUINFO_Frame_Buffer_Size,
        GPUINFO_Uncorr_ECC,
        GPUINFO_Max_VF,
        GPUINFO_GFX_Engine,
        GPUINFO_GFX_MAX_Clock,
        GPUINFO_Video_Encoder,
        GPUINFO_PCIE_Link_Speed,
        GPUINFO_PCIE_Link_Width,
        GPUINFO_KEY_MAX
    };
    char value[GPUINFO_KEY_MAX][15];

    /* gpuinfo table
    +-----+--------------+---------+------------+--------+------------+-----------+
    | GPU | Name         | DPM Cap | FB Size    | Max VF | GFX Engine | PL Speed  |
    |     | BusId        | PWR Cap | Encoder    | ECC    | MAX Clock  | PL Width  |
    +=====+==============+=========+============+========+============+===========+
    |   0 | Vega10       | 8       | 16368 M    | 4      | GFX9       | 8 GT/s    |
    |     | 0000:84:00.0 | 185 W   | H.264 HEVC | Yes    | 1400 MHz   | x16       |
    +-----+--------------+---------+------------+--------+------------+-----------+
    */

    fp = popen("cat /sys/bus/pci/drivers/gim/gpuinfo", "r");
    if (fp != NULL) {
        // print header
        printf("\n+-----+--------------+---------+------------+--------+------------+-----------+\n");
        printf("| GPU | Name         | DPM Cap | FB Size    | Max VF | GFX Engine | PL Speed  |\n");
        printf("|     | BusId        | PWR Cap | Encoder    | ECC    | MAX Clock  | PL Width  |\n");
        printf("+=====+==============+=========+============+========+============+===========+\n");
        while (1) {
            // read "GPU #"
            line = fgets(buf, sizeof buf, fp);
            if (line == NULL || strstr(line, "GPU #") == NULL) {
                break;
            }

            // read Key's value
            for (i = 0; i < GPUINFO_KEY_MAX; i++) {
                line = fgets(buf, sizeof buf, fp);

                if (line == NULL || strstr(line, key[i]) == NULL) {
                    printf("get key: %s failed\n", key[i]);
                    printf("key read: %s\n", line);
                    break;
                }
                token = strtok(line, ":");
                token = strtok(NULL, "\n");
                snprintf(value[i], sizeof(value[i]), "%s", token);
            }

            if (i == GPUINFO_KEY_MAX) {
                printf(format_1, index, value[GPUINFO_Name], value[GPUINFO_DPM_Cap], value[GPUINFO_Frame_Buffer_Size],
                       value[GPUINFO_Max_VF], value[GPUINFO_GFX_Engine], value[GPUINFO_PCIE_Link_Speed]);
                printf(format_2, value[GPUINFO_BusId], value[GPUINFO_Power_Cap], value[GPUINFO_Video_Encoder],
                        value[GPUINFO_Uncorr_ECC], value[GPUINFO_GFX_MAX_Clock], value[GPUINFO_PCIE_Link_Width]);
                printf("+-----+--------------+---------+------------+--------+------------+-----------+\n");
            } else {
                break;
            }

            index++;
        }
        pclose(fp);
    }

    return;
}

void show_gpuvs(void)
{
    FILE *fp;
    char *line;
    char buf[100];
    char vega10_format_1[] = "| %3d | %-12.12s | %-8.8s | %-10.10s | %-9.9s | %-9.9s | %-6.6s |\n";
    char vega10_format_2[] = "|     | %-12.12s | %-8.8s | %-10.10s | %-9.9s | %-9.9s | %-6.6s |\n";
    char s7150_format_1[] = "| %3d | %-12.12s | %-8.8s | %-10.10s | %-9.9s | %-18.18s |\n";
    char s7150_format_2[] = "|     | %-12.12s | %-8.8s | %-10.10s | %-9.9s | %-18.18s |\n";
    int index = 0;
    int i;
    char *token;
    char *vega10_key[] = {
        "Name",
        "BusId",
        "Current Volt",
        "Temperature",
        "GFX Engine Clock",
        "Memory Usage",
        "GFX Usage",
        "VCE Usage",
        "UVD Usage",
        "Available VF",
        "Correctable Error",
        "UnCorrectable Error",
    };
    enum vega10_gpuvs_key {
        GPUVS_VEGA10_Name,
        GPUVS_VEGA10_BusId,
        GPUVS_VEGA10_Current_Volt,
        GPUVS_VEGA10_Temperature,
        GPUVS_VEGA10_GFX_Engine_Clock,
        GPUVS_VEGA10_Memory_Usage,
        GPUVS_VEGA10_GFX_Usage,
        GPUVS_VEGA10_VCE_Usage,
        GPUVS_VEGA10_UVD_Usage,
        GPUVS_VEGA10_Available_VF,
        GPUVS_VEGA10_Correctable_Error,
        GPUVS_VEGA10_UnCorrectable_Error,
        GPUVS_VEGA10_KEY_MAX
    };
    char vega10_value[GPUVS_VEGA10_KEY_MAX][15];

    char *s7150_key[] = {
        "Name",
        "BusId",
        "Power Usage",
        "Current Volt",
        "Temperature",
        "Current DPM Level",
        "GFX Engine Clock",
        "Memory Usage",
        "GFX Usage",
        "Available VF",
    };
    enum s7150_gpuvs_key {
        GPUVS_S7150_Name,
        GPUVS_S7150_BusId,
        GPUVS_S7150_Power_Usage,
        GPUVS_S7150_Current_Volt,
        GPUVS_S7150_Temperature,
        GPUVS_S7150_Current_DPM_Level,
        GPUVS_S7150_GFX_Engine_Clock,
        GPUVS_S7150_Memory_Usage,
        GPUVS_S7150_GFX_Usage,
        GPUVS_S7150_Available_VF,
        GPUVS_S7150_KEY_MAX
    };
    char s7150_value[GPUVS_S7150_KEY_MAX][15];

    fp = popen("cat /sys/bus/pci/drivers/gim/gpuvs", "r");
    if (fp != NULL) {
        if (strstr(g_device_name, "Vega10") != NULL) {

            /* vega10 gpuvs table
            +-----+--------------+----------------------+--------+------------+-----------+
            | GPU | Name         | Cur Volt | GFX EngClk | Mem Usage | VCE Usage | CorErr |
            |     | BusId        | Temp     | Avail VF   | GFX Usage | UVD Usage | UnCErr |
            +=====+==============+==========+============+===========+===========+========+
            |   0 | Vega10       | 0.8000 V | 850.00 MHz | 0.00 %    | 0.00 %    | 0      |
            |     | 0000:84:00.0 | 33.00 C  | 4          | 0.00 %    | 0.00 %    | 0      |
            +-----+--------------+----------------------+--------+------------+-----------+
            */

            // print header
            printf("\n+-----+--------------+----------+------------+-----------+-----------+--------+\n");
            printf("| GPU | Name         | Cur Volt | GFX EngClk | Mem Usage | VCE Usage | CorErr |\n");
            printf("|     | BusId        | Temp     | Avail VF   | GFX Usage | UVD Usage | UnCErr |\n");
            printf("+=====+==============+==========+============+===========+===========+========+\n");
            while (1) {
                // read "GPU #"
                line = fgets(buf, sizeof buf, fp);
                if (line == NULL || strstr(line, "GPU #") == NULL) {
                    break;
                }

                // read Key's value
                for (i = 0; i < GPUVS_VEGA10_KEY_MAX; i++) {
                    line = fgets(buf, sizeof buf, fp);

                    if (line == NULL || strstr(line, vega10_key[i]) == NULL) {
                        printf("get key: %s failed\n", vega10_key[i]);
                        printf("key read: %s\n", line);
                        break;
                    }
                    token = strtok(line, ":");
                    token = strtok(NULL, "\n");
                    snprintf(vega10_value[i], sizeof(vega10_value[i]), "%s", token);
                }

                if (i == GPUVS_VEGA10_KEY_MAX) {
                    printf(vega10_format_1, index, vega10_value[GPUVS_VEGA10_Name], vega10_value[GPUVS_VEGA10_Current_Volt], vega10_value[GPUVS_VEGA10_GFX_Engine_Clock],
                            vega10_value[GPUVS_VEGA10_Memory_Usage], vega10_value[GPUVS_VEGA10_VCE_Usage], vega10_value[GPUVS_VEGA10_Correctable_Error]);
                    printf(vega10_format_2, vega10_value[GPUVS_VEGA10_BusId], vega10_value[GPUVS_VEGA10_Temperature], vega10_value[GPUVS_VEGA10_Available_VF],
                            vega10_value[GPUVS_VEGA10_GFX_Usage], vega10_value[GPUVS_VEGA10_UVD_Usage], vega10_value[GPUVS_VEGA10_UnCorrectable_Error]);
                    printf("+-----+--------------+----------+------------+-----------+-----------+--------+\n");
                } else {
                    break;
                }

                index++;
            }
        } else if (strstr(g_device_name, "S7150") != NULL) {

            /* s7150 gpuvs table
            +-----+--------------+----------+------------+-----------+--------------------+
            | GPU | Name         | Cur Volt | GFX EngClk | Mem Usage | Current DPM Level  |
            |     | BusId        | Temp     | Avail VF   | GFX Usage | Power Usage        |
            +=====+==============+==========+============+===========+====================+
            |   0 | S7150        | 0.8000 V | 300.00 MHz | 0.00 %    | 0                  |
            |     | 0000:82:00.0 | 57.50 C  | 4          | 0.00 %    | 34.56 W            |
            +-----+--------------+----------+------------+-----------+--------------------+
            */

            // print header
            printf("\n+-----+--------------+----------+------------+-----------+--------------------+\n");
            printf("| GPU | Name         | Cur Volt | GFX EngClk | Mem Usage | Current DPM Level  |\n");
            printf("|     | BusId        | Temp     | Avail VF   | GFX Usage | Power Usage        |\n");
            printf("+=====+==============+==========+============+===========+====================+\n");
            while (1) {
                // read "GPU #"
                line = fgets(buf, sizeof buf, fp);
                if (line == NULL || strstr(line, "GPU #") == NULL) {
                    break;
                }

                // read Key's value
                for (i = 0; i < GPUVS_S7150_KEY_MAX; i++) {
                    line = fgets(buf, sizeof buf, fp);

                    if (line == NULL || strstr(line, s7150_key[i]) == NULL) {
                        printf("get key: %s failed\n", s7150_key[i]);
                        printf("key read: %s\n", line);
                        break;
                    }
                    token = strtok(line, ":");
                    token = strtok(NULL, "\n");
                    snprintf(s7150_value[i], sizeof(s7150_value[i]), "%s", token);
                }

                if (i == GPUVS_S7150_KEY_MAX) {
                    printf(s7150_format_1, index, s7150_value[GPUVS_S7150_Name], s7150_value[GPUVS_S7150_Current_Volt],
                           s7150_value[GPUVS_S7150_GFX_Engine_Clock], s7150_value[GPUVS_S7150_Memory_Usage],
                           s7150_value[GPUVS_S7150_Current_DPM_Level]);
                    printf(s7150_format_2, s7150_value[GPUVS_S7150_BusId], s7150_value[GPUVS_S7150_Temperature],
                           s7150_value[GPUVS_S7150_Available_VF], s7150_value[GPUVS_S7150_GFX_Usage],
                           s7150_value[GPUVS_S7150_Power_Usage]);
                    printf("+-----+--------------+----------+------------+-----------+--------------------+\n");
                } else {
                    break;
                }

                index++;
            }
        }
        pclose(fp);
    } else {
        printf("Not supported device: %s\n", g_device_name);
    }

    return;
}

int open_gpu(char *bus_id)
{
    strcpy(g_cur_gpu, bus_id);
    g_gpu_opened = true;

    return RET_OK;
}

int close_gpu(void)
{
    g_gpu_opened = false;

    return RET_OK;
}

void gpu_status(void) {
    FILE *fp_gpu;
    FILE *fp_vf;
    char *line;
    char buf[100];
    char format[] = "| %2d | %-11.11s | %-12.12s | %-12.12s | %-12.12s | %-11.11s |\n";
    int index = 0;
    int i;
    char *token;
    char *key[] = {
        "Active vGPUs",
        "vGPU ID",
        "vGPU Name",
        "vGPU Type",
        "Guest Driver Version",
        "Guest Driver Certification",
        "vGPU State",
        "VF current running section",
        "VF active section",
        "VF Last Init Start",
        "VF Last Init Finish",
        "VF Last Shutdown Start",
        "VF Last Shutdown Finish",
        "VF Last Reset At",
        "VF Reset Time",
        "VF FB Size",
    };
    enum gpuvf_key {
        VF_Active_vGPUs,
        VF_vGPU_ID,
        VF_vGPU_Name,
        VF_vGPU_Type,
        VF_Guest_Driver_Version,
        VF_Guest_Driver_Certification,
        VF_vGPU_State,
        VF_current_running_section,
        VF_active_section,
        VF_Last_Init_Start,
        VF_Last_Init_Finish,
        VF_Last_Shutdown_Start,
        VF_Last_Shutdown_Finish,
        VF_Last_Reset_At,
        VF_Reset_Time,
        VF_FB_Size,
        VF_KEY_MAX

    };
    char value[VF_KEY_MAX][15];
    char cmd[100];
    char bus_id[15];

    // print header
    printf("\n+----+-------------+--------------+--------------+--------------+-------------+\n");
    printf("| VF | Type        | BusId        | Active Time  | Running Time | Reset Times |\n");
    printf("+====+=============+==============+==============+==============+=============+\n");

    sprintf(cmd,"cat /sys/bus/pci/devices/%s/gpuvf", g_cur_gpu);
    fp_gpu = popen(cmd, "r");
    if (fp_gpu != NULL) {
        // get VF BusId
        while (1) {
            line = fgets(buf, sizeof buf, fp_gpu);

            if (line == NULL) {
                pclose(fp_gpu);
                return;
            }

            if (strcasestr(line, "BusId") != NULL) {
                token = strtok(line, ":");
                token = strtok(NULL, "\n");
                snprintf(bus_id, sizeof(value[i]), "%s", token);

                sprintf(cmd,"cat /sys/bus/pci/devices/%s/gpuvf", bus_id);
                fp_vf = popen(cmd, "r");
                if (fp_vf != NULL) {
                    while (1) {
                        line = fgets(buf, sizeof buf, fp_vf);
                        if (line == NULL || strstr(line, "GPU:") == NULL) {
                            break;
                        }

                        // read Key's value
                        for (i = 0; i < VF_KEY_MAX; i++) {
                            line = fgets(buf, sizeof buf, fp_vf);

                            if (line == NULL || strcasestr(line, key[i]) == NULL) {
                                printf("get key: %s failed\n", key[i]);
                                printf("key read: %s\n", line);
                                break;
                            }
                            token = strtok(line, ":");
                            token = strtok(NULL, "\n");
                            snprintf(value[i], sizeof(value[i]), "%s", token);
                        }

                        if (i == VF_KEY_MAX) {
                            printf(format, index, value[VF_vGPU_Type], value[VF_vGPU_ID], value[VF_active_section],
                                   value[VF_current_running_section], value[VF_Reset_Time]);
                            printf("+----+-------------+--------------+--------------+--------------+-------------+\n");
                        } else {
                            break;
                        }

                        index++;
                    }
                    pclose(fp_vf);
                }
            }
        }


        pclose(fp_gpu);
    }

    return;
}

void list_vf(void) {
    FILE *fp;
    char *line;
    char buf[100];
    char format_1[] = "| %2d | %-6.6s | %-12.12s | %-10.10s | %-9.9s | %-7.7s | %-11.11s |\n";
    char format_2[] = "|    | %-6.6s | %-12.12s | %-10.10s | %-9.9s | %-7.7s | %-11.11s |\n";
    int index = 0;
    int i;
    char *token;
    char *key[] = {
        "Type",
        "BusId",
        "Name",
        "VF State",
        "VF Size",
        "GFX Engine Partition",
    };
    enum gpuvf_key {
        GPUVF_Type,
        GPUVF_BusId,
        GPUVF_Name,
        GPUVF_VF_State,
        GPUVF_VF_Size,
        GPUVF_GFX_Engine_Partition,
        GPUVF_KEY_MAX

    };
    char value[GPUVF_KEY_MAX][15];
    char cmd[100];

    /* gpuvs table
    +----+--------+--------------+------------+-----------+---------+-------------+
    | VF | Type   | BusId        | Name       | VF State  | VF Size | GFX EngPart |
    +====+========+==============+============+===========+=========+=============+
    |  0 | Vega10 | 0000:84:02.0 | MxGPU_V2_4 | Available | 4080 M  | 25.00 %     |
    +----+--------+--------------+------------+-----------+---------+-------------+
    */

    sprintf(cmd,"cat /sys/bus/pci/devices/%s/gpuvf", g_cur_gpu);
    fp = popen(cmd, "r");
    if (fp != NULL) {
        // print header
        printf("\n+----+--------+--------------+------------+-----------+---------+-------------+\n");
        printf("| VF | Type   | BusId        | Name       | VF State  | VF Size | GFX EngPart |\n");
        printf("+====+========+==============+============+===========+=========+=============+\n");
        while (1) {
            // read "VF #"
            line = fgets(buf, sizeof buf, fp);
            if (line == NULL || strstr(line, "VF #") == NULL) {
                break;
            }

            // read Key's value
            for (i = 0; i < GPUVF_KEY_MAX; i++) {
                line = fgets(buf, sizeof buf, fp);

                if (line == NULL || strcasestr(line, key[i]) == NULL) {
                    printf("get key: %s failed\n", key[i]);
                    printf("key read: %s\n", line);
                    break;
                }
                token = strtok(line, ":");
                token = strtok(NULL, "\n");
                snprintf(value[i], sizeof(value[i]), "%s", token);
            }

            if (i == GPUVF_KEY_MAX) {
                printf(format_1, index, value[GPUVF_Type], value[GPUVF_BusId], value[GPUVF_Name],
                       value[GPUVF_VF_State], value[GPUVF_VF_Size], value[GPUVF_GFX_Engine_Partition]);
                printf("+----+--------+--------------+------------+-----------+---------+-------------+\n");
            } else {
                break;
            }

            index++;
        }
        pclose(fp);
    }

    return;
}

void get_vf(char *fb_start, char *fb_size, char *gfx_partition)
{
    char cmd[100];

    sprintf(cmd,"echo %s %s %s >/sys/bus/pci/devices/%s/getvf", fb_start, fb_size, gfx_partition, g_cur_gpu);
    system(cmd);
    sprintf(cmd,"cat /sys/bus/pci/devices/%s/getvf", g_cur_gpu);
    system(cmd);
}

int release_vf(char *bus_id)
{
    char cmd[100];

    sprintf(cmd,"echo '%s' >/sys/bus/pci/devices/%s/relvf", bus_id, g_cur_gpu);
    system(cmd);
    sprintf(cmd,"cat /sys/bus/pci/devices/%s/relvf", g_cur_gpu);
    system(cmd);
}

int open_vf(char *bus_id)
{
    strcpy(g_cur_vf, bus_id);
    g_vf_opened = true;

    return RET_OK;
}

int close_vf(void)
{
    g_vf_opened = false;

    return RET_OK;
}

char *get_current_gpu(void)
{
    return g_cur_gpu;
}

char *get_current_vf(void)
{
    return g_cur_vf;
}

bool is_gpu_opened(void)
{
    return g_gpu_opened;
}

bool is_vf_opened(void)
{
    return g_vf_opened;
}
