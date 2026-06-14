/*
 * Copyright 2022 Advanced Micro Devices, Inc.
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
 * Authors: AMD
 *
 */

/* FILE POLICY AND INTENDED USAGE:
 * This file implements basic dp phy functionality such as enable/disable phy
 * output and set lane/drive settings. This file is responsible for maintaining
 * and update software state representing current phy status such as current
 * link settings.
 */

#include <linux/dmi.h>

#include "amdgpu.h"
#include "link_dp_phy.h"
#include "link_dpcd.h"
#include "link_dp_training.h"
#include "link_dp_capability.h"
#include "clk_mgr.h"
#include "resource.h"
#include "link_enc_cfg.h"
#include "atomfirmware.h"
#include "grph_object_id.h"
#define DC_LOGGER \
	link->ctx->logger

#define IMAC5K_REBOOT_ROOT_OBJECT_ID 0x3114
#define IMAC5K_REBOOT_ROOT_DDC_HW_INST 3
#define IMAC5K_REBOOT_SLAVE_OBJECT_ID 0x3113
#define IMAC5K_REBOOT_SLAVE_DDC_HW_INST 2

static const char *imac5k_reboot_link_role(const struct dc_link *link)
{
	unsigned int object_id;

	if (!amdgpu_imac5k_reboot_in_progress ||
	    !dmi_match(DMI_SYS_VENDOR, "Apple Inc.") ||
	    !dmi_match(DMI_PRODUCT_NAME, "iMac19,1") || !link)
		return NULL;

	object_id = dal_graphics_object_id_to_uint(link->link_id);
	if (link->connector_signal == SIGNAL_TYPE_EDP &&
	    object_id == IMAC5K_REBOOT_ROOT_OBJECT_ID &&
	    link->ddc_hw_inst == IMAC5K_REBOOT_ROOT_DDC_HW_INST &&
	    link->link_enc &&
	    link->link_enc->transmitter == TRANSMITTER_UNIPHY_C)
		return "root";

	if (link->connector_signal == SIGNAL_TYPE_DISPLAY_PORT &&
	    object_id == IMAC5K_REBOOT_SLAVE_OBJECT_ID &&
	    link->ddc_hw_inst == IMAC5K_REBOOT_SLAVE_DDC_HW_INST &&
	    link->link_enc &&
	    link->link_enc->transmitter == TRANSMITTER_UNIPHY_D)
		return "slave";

	return NULL;
}

void dpcd_write_rx_power_ctrl(struct dc_link *link, bool on)
{
	const char *role = imac5k_reboot_link_role(link);
	uint8_t state;

	state = on ? DP_POWER_STATE_D0 : DP_POWER_STATE_D3;

	if (link->sync_lt_in_progress) {
		if (role)
			pr_emerg("IMAC5K-REBOOT: rx-power role=%s link=%u request=%02x skipped=sync-lt\n",
				 role, link->link_index, state);
		return;
	}

	if (role) {
		uint8_t before = 0;
		uint8_t after = 0;
		enum dc_status before_status;
		enum dc_status write_status;
		enum dc_status after_status;

		before_status = core_link_read_dpcd(link, DP_SET_POWER, &before,
						    sizeof(before));
		write_status = core_link_write_dpcd(link, DP_SET_POWER, &state,
						    sizeof(state));
		after_status = core_link_read_dpcd(link, DP_SET_POWER, &after,
						   sizeof(after));
		pr_emerg("IMAC5K-REBOOT: rx-power role=%s link=%u request=%02x before_status=%d before=%02x write_status=%d after_status=%d after=%02x\n",
			 role, link->link_index, state, before_status, before,
			 write_status, after_status, after);
		return;
	}

	core_link_write_dpcd(link, DP_SET_POWER, &state,
						 sizeof(state));

}

void dp_enable_link_phy(
	struct dc_link *link,
	const struct link_resource *link_res,
	enum signal_type signal,
	enum clock_source_id clock_source,
	const struct dc_link_settings *link_settings)
{
	link->cur_link_settings = *link_settings;
	link->dc->hwss.enable_dp_link_output(link, link_res, signal,
			clock_source, link_settings);
	dpcd_write_rx_power_ctrl(link, true);
}

void dp_disable_link_phy(struct dc_link *link,
		const struct link_resource *link_res,
		enum signal_type signal)
{
	struct dc  *dc = link->ctx->dc;
	const char *role = imac5k_reboot_link_role(link);
	bool send_d3 = !link->wa_flags.dp_keep_receiver_powered &&
		       !link->skip_implict_edp_power_control &&
		       link->type != dc_connection_none;

	if (role)
		pr_emerg("IMAC5K-REBOOT: phy-disable entry role=%s link=%u signal=%d send_d3=%d keep_rx=%d implicit_edp_skip=%d type=%d rate=%d lanes=%d\n",
			 role, link->link_index, signal, send_d3,
			 link->wa_flags.dp_keep_receiver_powered,
			 link->skip_implict_edp_power_control, link->type,
			 link->cur_link_settings.link_rate,
			 link->cur_link_settings.lane_count);

	if (send_d3)
		dpcd_write_rx_power_ctrl(link, false);

	dc->hwss.disable_link_output(link, link_res, signal);
	if (role)
		pr_emerg("IMAC5K-REBOOT: phy-disable output-disabled role=%s link=%u signal=%d\n",
			 role, link->link_index, signal);
	/* Clear current link setting.*/
	memset(&link->cur_link_settings, 0,
			sizeof(link->cur_link_settings));

	if (dc->clk_mgr->funcs->notify_link_rate_change)
		dc->clk_mgr->funcs->notify_link_rate_change(dc->clk_mgr, link);

	if (role)
		pr_emerg("IMAC5K-REBOOT: phy-disable exit role=%s link=%u\n",
			 role, link->link_index);
}

static inline bool is_immediate_downstream(struct dc_link *link, uint32_t offset)
{
	return (dp_parse_lttpr_repeater_count(link->dpcd_caps.lttpr_caps.phy_repeater_cnt) ==
			offset);
}

void dp_set_hw_lane_settings(
	struct dc_link *link,
	const struct link_resource *link_res,
	const struct link_training_settings *link_settings,
	uint32_t offset)
{
	const struct link_hwss *link_hwss = get_link_hwss(link, link_res);

	// Don't return here if using FIXED_VS link HWSS and encoding is 128b/132b
	if ((link_settings->lttpr_mode == LTTPR_MODE_NON_TRANSPARENT) &&
			!is_immediate_downstream(link, offset) &&
			(!((link->chip_caps & AMD_EXT_DISPLAY_PATH_CAPS__EXT_CHIP_MASK) == AMD_EXT_DISPLAY_PATH_CAPS__DP_FIXED_VS_EN) ||
			link_dp_get_encoding_format(&link_settings->link_settings) == DP_8b_10b_ENCODING))
		return;

	if (link_hwss->ext.set_dp_lane_settings)
		link_hwss->ext.set_dp_lane_settings(link, link_res,
				&link_settings->link_settings,
				link_settings->hw_lane_settings);

	memmove(link->cur_lane_setting,
			link_settings->hw_lane_settings,
			sizeof(link->cur_lane_setting));
}

void dp_set_drive_settings(
	struct dc_link *link,
	const struct link_resource *link_res,
	struct link_training_settings *lt_settings)
{
	/* program ASIC PHY settings*/
	dp_set_hw_lane_settings(link, link_res, lt_settings, DPRX);

	dp_hw_to_dpcd_lane_settings(lt_settings,
			lt_settings->hw_lane_settings,
			lt_settings->dpcd_lane_settings);

	/* Notify DP sink the PHY settings from source */
	dpcd_set_lane_settings(link, lt_settings, DPRX);
}

enum dc_status dp_set_fec_ready(struct dc_link *link, const struct link_resource *link_res, bool ready)
{
	/* FEC has to be "set ready" before the link training.
	 * The policy is to always train with FEC
	 * if the sink supports it and leave it enabled on link.
	 * If FEC is not supported, disable it.
	 */
	struct link_encoder *link_enc = link_res->dio_link_enc;
	enum dc_status status = DC_OK;
	uint8_t fec_config = 0;

	if (!link->dc->config.unify_link_enc_assignment)
		link_enc = link_enc_cfg_get_link_enc(link);
	ASSERT(link_enc);
	if (link_enc->funcs->fec_set_ready == NULL)
		return DC_NOT_SUPPORTED;

	if (ready && dp_should_enable_fec(link)) {
		fec_config = 1;

		status = core_link_write_dpcd(link, DP_FEC_CONFIGURATION,
				&fec_config, sizeof(fec_config));

		if (status == DC_OK) {
			link_enc->funcs->fec_set_ready(link_enc, true);
			link->fec_state = dc_link_fec_ready;
		}
	} else {
		if (link->fec_state == dc_link_fec_ready) {
			fec_config = 0;
			if (link->type != dc_connection_none)
				core_link_write_dpcd(link, DP_FEC_CONFIGURATION,
					&fec_config, sizeof(fec_config));

			link_enc->funcs->fec_set_ready(link_enc, false);
			link->fec_state = dc_link_fec_not_ready;
		}
	}

	return status;
}

void dp_set_fec_enable(struct dc_link *link, const struct link_resource *link_res, bool enable)
{
	struct link_encoder *link_enc = link_res->dio_link_enc;

	if (!link->dc->config.unify_link_enc_assignment)
		link_enc = link_enc_cfg_get_link_enc(link);

	if (link_enc == NULL || link_enc->funcs == NULL || link_enc->funcs->fec_set_enable == NULL)
		return;

	if (enable && dp_should_enable_fec(link)) {
		if (link->fec_state == dc_link_fec_ready) {
			/* According to DP spec, FEC enable sequence can first
			 * be transmitted anytime after 1000 LL codes have
			 * been transmitted on the link after link training
			 * completion. Using 1 lane RBR should have the maximum
			 * time for transmitting 1000 LL codes which is 6.173 us.
			 * So use 7 microseconds delay instead.
			 */
			udelay(7);
			link_enc->funcs->fec_set_enable(link_enc, true);
			link->fec_state = dc_link_fec_enabled;
		}
	} else {
		if (link->fec_state == dc_link_fec_enabled) {
			link_enc->funcs->fec_set_enable(link_enc, false);
			link->fec_state = dc_link_fec_ready;
		}
	}
}
