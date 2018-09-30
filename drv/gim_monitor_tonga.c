/*
 * Copyright (c) 2017 Advanced Micro Devices, Inc. All rights reserved.
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

#include <linux/delay.h>
#include "gim_monitor.h"
#include "gim_monitor_tonga.h"
#include "gim_debug.h"
#include "gim_os_service.h"
#include "gim_atombios.h"
#include "gim_s7150_reg.h"
#include "gim_atombios.h"
#include "gim_monitor.h"
#include "gim_command.h"

int read_smc_ind_reg32(uint32_t *mmr, uint32_t reg)
{
	write_reg32(mmr, mmSMC_IND_INDEX_6, reg);
	return read_reg32(mmr, mmSMC_IND_DATA_6);
}

int send_smc_msg(uint32_t *mmr, uint32_t cmd,
		uint32_t *input, uint32_t *output)
{
	union smc_message_1 msg1;
	union smc_msg_arg_1 arg1;
	union smc_resp_1 resp1;
	uint32_t time_left = SMC_WAIT_TIMEOUT_MS;

	if (input != NULL) {
		arg1.u32all = read_reg32(mmr, mmSMC_MSG_ARG_0);
		arg1.bitfields.smc_msg_arg = *input;
		write_reg32(mmr, mmSMC_MSG_ARG_0, arg1.u32all);
	}

	for (time_left = SMC_WAIT_TIMEOUT_MS;
		time_left > 0;
		time_left -= SMC_WAIT_TICK_MS) {
		resp1.u32all = read_reg32(mmr, mmSMC_RESP_0);
		if (resp1.bitfields.smc_resp)
			break;

		mdelay(SMC_WAIT_TICK_MS);
	}

	msg1.u32all = read_reg32(mmr, mmSMC_MESSAGE_0);
	msg1.bitfields.smc_msg = cmd;
	write_reg32(mmr, mmSMC_MESSAGE_0, msg1.u32all);

	for (time_left = SMC_WAIT_TIMEOUT_MS;
		time_left > 0;
		time_left -= SMC_WAIT_TICK_MS) {
		resp1.u32all = read_reg32(mmr, mmSMC_RESP_0);
		if (resp1.bitfields.smc_resp)
			break;

		mdelay(SMC_WAIT_TICK_MS);
	}

	if (!(time_left > 0)) {
		gim_err("FAILED TO WAIT SMC RESPONSE");
		return -1;
	}

	if (resp1.bitfields.smc_resp != 1) {
		if (resp1.bitfields.smc_resp == 0xFF) {
			gim_err("SMC response not OK");
			return -1;
		}
		if (resp1.bitfields.smc_resp == 0xFE) {
			gim_err("SMC Unknown message 0x%x", cmd);
			return -1;
		}
	}

	if (output != NULL)
		*output = read_reg32(mmr, mmSMC_MSG_ARG_0);

	return 0;
}

static int get_thermal_sensor_type_from_temp_sel(struct adapter *adapt,
						 enum thermal_sensor_type *ts)
{
	union cg_mult_thermal_ctrl reg_cg_mult_therm_ctrl;

	reg_cg_mult_therm_ctrl.i32all =	read_smc_ind_reg32(
					(void *)adapt->pf.mmr_base,
					 ixCG_MULT_THERMAL_CTRL);

	switch (reg_cg_mult_therm_ctrl.bits.temp_sel) {
	case 0:
		*ts = TMON_0_RDIL0;
		break;
	case 1:
		*ts = TMON_0_RDIL1;
		break;
	case 2:
		*ts = TMON_0_RDIL2;
		break;
	case 3:
		*ts = TMON_0_RDIL3;
		break;


	case 16:
		*ts = TMON_0_RDIR0;
		break;
	case 17:
		*ts = TMON_0_RDIR1;
		break;
	case 18:
		*ts = TMON_0_RDIR2;
		break;
	case 19:
		*ts = TMON_0_RDIR3;
		break;
	case 20:
		*ts = TMON_0_RDIR4;
		break;
	case 21:
		*ts = TMON_0_RDIR5;
		break;
	case 22:
		*ts = TMON_0_RDIR6;
		break;
	case 23:
		*ts = TMON_0_RDIR7;
		break;
	case 24:
		*ts = TMON_0_RDIR8;
		break;
	case 25:
		*ts = TMON_0_RDIR9;
		break;
	case 26:
		*ts = TMON_0_RDIR10;
		break;
	case 27:
		*ts = TMON_0_RDIR11;
		break;
	case 28:
		*ts = TMON_0_RDIR12;
		break;
	case 29:
		*ts = TMON_0_RDIR13;
		break;
	case 30:
		*ts = TMON_0_RDIR14;
		break;
	case 31:
		*ts = TMON_0_RDIR15;
		break;

	case 32:
		*ts = TMON_1_RDIL0;
		break;
	case 33:
		*ts = TMON_1_RDIL1;
		break;
	case 34:
		*ts = TMON_1_RDIL2;
		break;
	case 35:
		*ts = TMON_1_RDIL3;
		break;

	case 48:
		*ts = TMON_1_RDIR0;
		break;
	case 49:
		*ts = TMON_1_RDIR1;
		break;
	case 50:
		*ts = TMON_1_RDIR2;
		break;
	case 51:
		*ts = TMON_1_RDIR3;
		break;
	case 52:
		*ts = TMON_1_RDIR4;
		break;
	case 53:
		*ts = TMON_1_RDIR5;
		break;
	case 54:
		*ts = TMON_1_RDIR6;
		break;
	case 55:
		*ts = TMON_1_RDIR7;
		break;
	case 56:
		*ts = TMON_1_RDIR8;
		break;
	case 57:
		*ts = TMON_1_RDIR9;
		break;
	case 58:
		*ts = TMON_1_RDIR10;
		break;
	case 59:
		*ts = TMON_1_RDIR11;
		break;
	case 60:
		*ts = TMON_1_RDIR12;
		break;
	case 61:
		*ts = TMON_1_RDIR13;
		break;
	case 62:
		*ts = TMON_1_RDIR14;
		break;
	case 63:
		*ts = TMON_1_RDIR15;
		break;

	default:
		return -1;
	}

	if (reg_cg_mult_therm_ctrl.bits.temp_sel < 32)
		return 0;
	else if (reg_cg_mult_therm_ctrl.bits.temp_sel < 64)
		return 0;
	else
		return -1;

}

/* temperature = val / 4 */
static int amdgim_tonga_get_asic_temperature(struct adapter *adapt, int *tval)
{
	enum thermal_sensor_type id = 0;
	uint32_t offset = 0;
	union thm_tmon0_rdil0_data reg_rdi;
	int ival;
	/* Go the CTF_RDI path in the AGT inherited from policy of bonaire */
	if (get_thermal_sensor_type_from_temp_sel(adapt, &id)) {
		gim_err("Failed to recognize CTF_RDI");
		return -1;
	}

	switch (id) {
	case TMON_0_RDIL0:
		offset = 0xC0300100;
		break;
	case TMON_0_RDIL1:
		offset = 0xC0300104;
		break;
	case TMON_0_RDIL2:
		offset = 0xC0300108;
		break;
	case TMON_0_RDIL3:
		offset = 0xC030010C;
		break;

	case TMON_0_RDIR0:
		offset = 0xC0300140;
		break;
	case TMON_0_RDIR1:
		offset = 0xC0300144;
		break;
	case TMON_0_RDIR2:
		offset = 0xC0300148;
		break;
	case TMON_0_RDIR3:
		offset = 0xC030014C;
		break;
	case TMON_0_RDIR4:
		offset = 0xC0300150;
		break;
	case TMON_0_RDIR5:
		offset = 0xC0300154;
		break;
	case TMON_0_RDIR6:
		offset = 0xC0300158;
		break;
	case TMON_0_RDIR7:
		offset = 0xC030015C;
		break;
	case TMON_0_RDIR8:
		offset = 0xC0300160;
		break;
	case TMON_0_RDIR9:
		offset = 0xC0300164;
		break;
	case TMON_0_RDIR10:
		offset = 0xC0300168;
		break;
	case TMON_0_RDIR11:
		offset = 0xC030016C;
		break;
	case TMON_0_RDIR12:
		offset = 0xC0300170;
		break;
	case TMON_0_RDIR13:
		offset = 0xC0300174;
		break;
	case TMON_0_RDIR14:
		offset = 0xC0300178;
		break;
	case TMON_0_RDIR15:
		offset = 0xC030017C;
		break;


	case TMON_1_RDIL0:
		offset = 0xC0300180;
		break;
	case TMON_1_RDIL1:
		offset = 0xC0300184;
		break;
	case TMON_1_RDIL2:
		offset = 0xC0300188;
		break;
	case TMON_1_RDIL3:
		offset = 0xC030018C;
		break;


	case TMON_1_RDIR0:
		offset = 0xC03001C0;
		break;
	case TMON_1_RDIR1:
		offset = 0xC03001C4;
		break;
	case TMON_1_RDIR2:
		offset = 0xC03001C8;
		break;
	case TMON_1_RDIR3:
		offset = 0xC03001CC;
		break;
	case TMON_1_RDIR4:
		offset = 0xC03001D0;
		break;
	case TMON_1_RDIR5:
		offset = 0xC03001D4;
		break;
	case TMON_1_RDIR6:
		offset = 0xC03001D8;
		break;
	case TMON_1_RDIR7:
		offset = 0xC03001DC;
		break;
	case TMON_1_RDIR8:
		offset = 0xC03001E0;
		break;
	case TMON_1_RDIR9:
		offset = 0xC03001E4;
		break;
	case TMON_1_RDIR10:
		offset = 0xC03001E8;
		break;
	case TMON_1_RDIR11:
		offset = 0xC03001EC;
		break;
	case TMON_1_RDIR12:
		offset = 0xC03001F0;
		break;
	case TMON_1_RDIR13:
		offset = 0xC03001F4;
		break;
	case TMON_1_RDIR14:
		offset = 0xC03001F8;
		break;
	case TMON_1_RDIR15:
		offset = 0xC03001FC;
		break;

	case TMON_0_INT:
		offset = 0xC0300300;
		break;
	case TMON_1_INT:
		offset = 0xC0300304;
		break;

	default:
		gim_err("Failed to recognize sensor ID = %d", id);
		 /* if id is not identified, do not proceed
		  * furthre. Invalid offset may cause other
		  * side effects
		  */
		return -1;
	}



	reg_rdi.u32all = read_smc_ind_reg32(adapt->pf.mmr_base, offset);

	ival = reg_rdi.bits.temp;

	/* TEMP is in format signbit + 9.2 in two's compliment */
	if (reg_rdi.bits.temp & 0x800)
		ival |= 0xFFFFF000; /*negative - extended the sign */

	*tval = ival;

	return 0;

}

/* gpu power = val / 256 */
static int amdgim_tonga_get_gpu_power(struct adapter *adapt, int *val)
{
	int ret;
	int error = 0;

	mutex_lock(&adapt->curr_running_func_mutex);

	if (adapt->curr_running_func != NULL) {
		ret = switch_vfs(adapt->curr_running_func->func, &adapt->pf);
		if (ret != GIM_OK) {
			gim_err("Failed to switch to PF.");
			error = -1;
			goto exit;
		}
	}

	if (send_smc_msg(adapt->pf.mmr_base, 0x170, NULL, NULL)) {
		gim_err("Failed to start SMU simulator.");
		error = -1;
		goto exit;
	}

	if (!send_smc_msg(adapt->pf.mmr_base, 0x171, NULL, NULL)) {
		if (!send_smc_msg(adapt->pf.mmr_base, 0x171, NULL, NULL)) {
			delay_in_micro_seconds(AMDGIM_POWER_DELAY_MS);
			ret = read_smc_ind_reg32((void *) adapt->pf.mmr_base,
						ixSMU_PM_STATUS_95);

			if ((ret > 0) && (ret < 1000 * 256)) {
				*val = ret;
				error = 0;
				goto exit;
			}
			gim_err("POWER RESULT NOT IN RANGE, %d", ret);
			error = -1;
			goto exit;
		}
	}

	gim_err("SMC RESPONSE ERROR");
exit:
	if (adapt->curr_running_func != NULL) {
		ret = switch_vfs(&adapt->pf, adapt->curr_running_func->func);
		if (ret != GIM_OK) {
			gim_err("Failed to switch back to VF%d.",
				adapt->curr_running_func->func->func_id);
			error = -1;
		}
	}

	mutex_unlock(&adapt->curr_running_func_mutex);
	return error;
}

/* activity = val/256 */
static int amdgim_tonga_get_gfx_activity(struct adapter *adapt, int *val)
{
	int smc_intr;
	int error = 0;
	int ret;

	mutex_lock(&adapt->curr_running_func_mutex);

	if (adapt->curr_running_func != NULL) {
		ret = switch_vfs(adapt->curr_running_func->func, &adapt->pf);
		if (ret != GIM_OK) {
			gim_err("Failed to switch to PF.");
			error = -1;
			goto exit;
		}
	}

	smc_intr = read_smc_ind_reg32(adapt->pf.mmr_base, ixSMC_PC_C);
	if (!AMDGIM_IS_SMC_ACTIVE(smc_intr)) {
		*val = 0;
		error = -1;
		goto exit;
	}

	*val = read_smc_ind_reg32(adapt->pf.mmr_base,
				ixSOFT_REGISTERS_TABLE_16);
exit:
	if (adapt->curr_running_func != NULL) {
		ret = switch_vfs(&adapt->pf, adapt->curr_running_func->func);
		if (ret != GIM_OK) {
			gim_err("Failed to switch back to VF%d.",
				adapt->curr_running_func->func->func_id);
			error = -1;
		}
	}

	mutex_unlock(&adapt->curr_running_func_mutex);
	return error;
}

/* activity = val/10000 */
static int amdgim_tonga_get_mem_activity(struct adapter *adapt, int *val)
{
	struct function *p_func;
	int vf_candidate = 0;
	int i;
	int usedmem = 0;

	vf_candidate = amdgim_get_vf_candidate(adapt);
	for (i = 0; i < adapt->enabled_vfs; ++i) {
		p_func = &adapt->vfs[i];
		if ((vf_candidate >> p_func->func_id) & 1)
			usedmem += p_func->fb_partition->slot.size;
	}

	*val = usedmem * 10000 / adapt->vf_fb_size;

	return 0;
}

/* vddc = (240 - val)/160 */
static int amdgim_tonga_get_vddc(struct adapter *adapt, int *val)
{
	union pwr_svi2_status status;
	int timeout = 1000;
	int time_count = 0;

	status.u32all = read_smc_ind_reg32(adapt->pf.mmr_base,
					ixPWR_SVI2_STATUS);

	while (1) {
		if (status.bits.svi2_busy == 0)
			break;

		status.u32all = read_smc_ind_reg32(adapt->pf.mmr_base,
						ixPWR_SVI2_STATUS);

		time_count++;

		if (time_count >= timeout) {
			gim_err("SVI2 BUSY");
			return -1;
		}
	}

	*val = status.bitfields.plane2_vid;
	return 0;
}

static int amdgim_tonga_get_gecc(struct adapter *adapt, bool *val)
{
	int regval;

	regval = read_reg32(adapt->pf.mmr_base, mmMC_ARB_GECC2) & 1;
	*val = (regval == 0) ? false:true;
	return 0;
}

static int amdgim_tonga_get_dpm_cap(struct adapter *adapt, int *cap)
{
	struct pwr_mgt_param pwr_paras = {
		PPSMC_MSG_SCLKDPM_GETENABLEDMASK,
		0};

	if (atom_dpm_state_cntl(adapt, &pwr_paras)) {
		gim_err("SCLK DPM:Execute table failed");
		*cap = 0;
		return -1;
	}

	if (pwr_paras.smc_msg_arg != 0) {
		*cap = pwr_paras.smc_msg_arg;
	} else{
		gim_info("SCLK DPM ARG = 0x%lx,ID=0x%lx, SCLK DPM disabled",
			pwr_paras.smc_msg_arg, pwr_paras.smc_msg_id);
		*cap = 0;
	}
	return 0;
}

static int amdgim_tonga_get_dpm_status(struct adapter *adapt, int *status)
{
	int target_index;
	int error = 0;
	int ret;

	mutex_lock(&adapt->curr_running_func_mutex);

	if (adapt->curr_running_func != NULL) {
		ret = switch_vfs(adapt->curr_running_func->func, &adapt->pf);
		if (ret != GIM_OK) {
			gim_err("Failed to switch to PF.");
			error = -1;
			goto exit;
		}
	}

	target_index = read_smc_ind_reg32(adapt->pf.mmr_base,
					ixTARGET_AND_CURRENT_PROFILE_INDEX);
	*status = (target_index
		& TARGET_AND_CURRENT_PROFILE_INDEX__CURR_SCLK_INDEX_MASK)
		 >> TARGET_AND_CURRENT_PROFILE_INDEX__CURR_SCLK_INDEX__SHIFT;

exit:
	if (adapt->curr_running_func != NULL) {
		ret = switch_vfs(&adapt->pf, adapt->curr_running_func->func);
		if (ret != GIM_OK) {
			gim_err("Failed to switch back to VF%d.",
				adapt->curr_running_func->func->func_id);
			error = -1;
		}
	}

	mutex_unlock(&adapt->curr_running_func_mutex);
	return error;

}

/* sclk = clk / 100 MHz */
static int amdgim_tonga_get_sclk(struct adapter *adapt, int *clk)
{
	int ret;
	int error = 0;

	mutex_lock(&adapt->curr_running_func_mutex);

	if (adapt->curr_running_func != NULL) {
		ret = switch_vfs(adapt->curr_running_func->func, &adapt->pf);
		if (ret != GIM_OK) {
			gim_err("Failed to switch to PF.");
			error = -1;
			goto exit;
		}
	}

	ret = send_smc_msg(adapt->pf.mmr_base,
			PPSMC_MSG_API_GETSCLKFREQUENCY, NULL, clk);
	if (ret != 0) {
		gim_err("Failed to get sclk");
		error = -1;
	}

exit:
	if (adapt->curr_running_func != NULL) {
		ret = switch_vfs(&adapt->pf, adapt->curr_running_func->func);
		if (ret != GIM_OK) {
			gim_err("Failed to switch back to VF%d.",
				adapt->curr_running_func->func->func_id);
			error = -1;
		}
	}

	mutex_unlock(&adapt->curr_running_func_mutex);
	return error;
}

int amdgim_tonga_get_bios_serial(struct adapter *adapt, char *serial)
{
	unsigned long long sid;
	unsigned long long sidext;
	unsigned long long data;

	/* sid bit[368,417] */
	sid = (read_smc_ind_reg32(adapt->pf.mmr_base,
				(ixSMU_EFUSE_0 + 44)) & 0xffff0000) >> 16;
	sid |= (read_smc_ind_reg32(adapt->pf.mmr_base,
				(ixSMU_EFUSE_0 + 48)) & 0x0000ffff) << 16;
	data = (read_smc_ind_reg32(adapt->pf.mmr_base,
				(ixSMU_EFUSE_0 + 48)) & 0xffff0000) >> 16;
	data |= (read_smc_ind_reg32(adapt->pf.mmr_base,
				(ixSMU_EFUSE_0 + 52)) & 0x0000ffff) << 16;
	sid = sid | (data << 32);
	sid &= 0x0003ffffffffffff;

	/* sid bit[506,548] */
	sidext = (read_smc_ind_reg32(adapt->pf.mmr_base,
				(ixSMU_EFUSE_0 + 60)) & 0xf3000000) >> 26;
	sidext |= (read_smc_ind_reg32(adapt->pf.mmr_base,
				(ixSMU_EFUSE_0 + 64)) & 0x0cffffff) << 6;
	data = (read_smc_ind_reg32(adapt->pf.mmr_base,
				(ixSMU_EFUSE_0 + 64)) & 0xf3000000) >> 26;
	data |= (read_smc_ind_reg32(adapt->pf.mmr_base,
				(ixSMU_EFUSE_0 + 68)) & 0x0cffffff) << 6;
	sidext = sidext | (data << 32);
	sidext &= 0x000003ffffffffff;
	sprintf(serial, "%016llx%016llx", sidext, sid);
	return 0;
}

static int amdgim_fpn_div(char *str, long long val1,
			long long val2, int accuracy)
{
	long long decimal;
	long long fraction;
	long long mult = 1;
	long long val;
	int i;
	char fmt[AMDGIM_STRLEN_NORMAL];

	for (i = 0; i < accuracy; ++i)
		mult *= 10;

	val = val1 * mult / val2;
	decimal = val / mult;
	fraction = val % mult;

	sprintf(fmt, "%%lld.%%0%dlld", accuracy);

	sprintf(str, fmt, decimal, fraction);
	return AMDGIM_ERROR_MONITOR_SUCCESS;
}


void amdgim_tonga_tostrall(struct adapter *p_adapter,
			struct amdgim_gpuvs *p_gpuvs, char *strbuf)
{
	char buf[AMDGIM_STRLEN_VERYLONG];
	char numbuf[AMDGIM_STRLEN_NORMAL];

	sprintf(buf, "\tName:%s\n", p_gpuvs->name);
	strcat(strbuf, buf);

	sprintf(buf, "\tBusId:%04x:%02x:%02x.%x\n",
		AMDGIM_GET_PCI_DOMAIN(p_gpuvs->dbdf),
		AMDGIM_GET_PCI_BUS(p_gpuvs->dbdf),
		AMDGIM_GET_PCI_DEVICE(p_gpuvs->dbdf),
		AMDGIM_GET_PCI_FUNCTION(p_gpuvs->dbdf));
	strcat(strbuf, buf);

	amdgim_fpn_div(numbuf, p_gpuvs->power_usage, 256, 2);
	sprintf(buf, "\tPower Usage:%s W\n", numbuf);
	strcat(strbuf, buf);

	amdgim_fpn_div(numbuf, p_gpuvs->cur_volt, 160, 4);
	sprintf(buf, "\tCurrent Volt:%s V\n", numbuf);
	strcat(strbuf, buf);

	amdgim_fpn_div(numbuf, p_gpuvs->temperature, 4, 2);
	sprintf(buf, "\tTemperature:%s C\n", numbuf);
	strcat(strbuf, buf);

	sprintf(buf, "\tCurrent DPM Level:%d\n",
		__builtin_ffs(p_gpuvs->cur_dpm));
	strcat(strbuf, buf);

	amdgim_fpn_div(numbuf, p_gpuvs->gfx_clk, 100, 2);
	sprintf(buf, "\tGFX Engine Clock:%s MHz\n", numbuf);
	strcat(strbuf, buf);

	amdgim_fpn_div(numbuf, p_gpuvs->memory_usage, 100, 2);
	sprintf(buf, "\tMemory Usage:%s %%\n", numbuf);
	strcat(strbuf, buf);

	amdgim_fpn_div(numbuf, p_gpuvs->gfx_usage, 256, 2);
	sprintf(buf, "\tGFX Usage:%s %%\n", numbuf);
	strcat(strbuf, buf);

	sprintf(buf, "\tAvailable VF:%d\n", p_gpuvs->available_vf);
	strcat(strbuf, buf);
}

static struct amdgim_asic_operation tonga_asic_op = {
	.amdgim_get_gpu_power = amdgim_tonga_get_gpu_power,
	.amdgim_get_gfx_activity = amdgim_tonga_get_gfx_activity,
	.amdgim_get_asic_temperature = amdgim_tonga_get_asic_temperature,
	.amdgim_get_vddc = amdgim_tonga_get_vddc,
	.amdgim_get_dpm_status = amdgim_tonga_get_dpm_status,
	.amdgim_get_dpm_cap = amdgim_tonga_get_dpm_cap,
	.amdgim_get_mem_activity = amdgim_tonga_get_mem_activity,
	.amdgim_get_gecc = amdgim_tonga_get_gecc,
	.amdgim_get_sclk = amdgim_tonga_get_sclk,
	.amdgim_get_bios_serial = amdgim_tonga_get_bios_serial,
	.amdgim_tostrall = amdgim_tonga_tostrall,
};

struct amdgim_asic_operation *amdgim_tonga_get_asic_op(void)
{
	return &tonga_asic_op;
}

