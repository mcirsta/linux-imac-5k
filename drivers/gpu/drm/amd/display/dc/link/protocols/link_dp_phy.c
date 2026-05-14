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

#define IMAC5K_WIN_SECONDARY_OBJECT_ID 0x3113
#define IMAC5K_WIN_SECONDARY_DDC_HW_INST 2
#define IMAC5K_WIN_PRIMARY_OBJECT_ID 0x3114
#define IMAC5K_WIN_PRIMARY_DDC_HW_INST 3

static bool link_is_imac5k_secondary_power_route(const struct dc_link *link)
{
	if (!link || link->connector_signal != SIGNAL_TYPE_DISPLAY_PORT)
		return false;

	if (dal_graphics_object_id_to_uint(link->link_id) !=
	    IMAC5K_WIN_SECONDARY_OBJECT_ID)
		return false;

	if (link->ddc_hw_inst != IMAC5K_WIN_SECONDARY_DDC_HW_INST)
		return false;

	if (!link->link_enc ||
	    link->link_enc->transmitter != TRANSMITTER_UNIPHY_D)
		return false;

	return true;
}

static bool link_is_imac5k_primary_power_route(const struct dc_link *link)
{
	if (!link || link->connector_signal != SIGNAL_TYPE_EDP)
		return false;

	if (dal_graphics_object_id_to_uint(link->link_id) !=
	    IMAC5K_WIN_PRIMARY_OBJECT_ID)
		return false;

	if (link->ddc_hw_inst != IMAC5K_WIN_PRIMARY_DDC_HW_INST)
		return false;

	if (!link->link_enc ||
	    link->link_enc->transmitter != TRANSMITTER_UNIPHY_C)
		return false;

	return true;
}

static const char *link_imac5k_role(const struct dc_link *link)
{
	if (link_is_imac5k_primary_power_route(link))
		return "primary";
	if (link_is_imac5k_secondary_power_route(link))
		return "secondary";
	return NULL;
}

static struct dc_link *link_imac5k_find_peer(const struct dc_link *self)
{
	const struct dc *dc;
	bool self_is_primary;
	bool self_is_secondary;
	unsigned int i;

	if (!self || !self->dc)
		return NULL;

	self_is_primary = link_is_imac5k_primary_power_route(self);
	self_is_secondary = link_is_imac5k_secondary_power_route(self);
	if (!self_is_primary && !self_is_secondary)
		return NULL;

	dc = self->dc;
	for (i = 0; i < dc->link_count; i++) {
		struct dc_link *other = dc->links[i];

		if (!other || other == self)
			continue;
		if (self_is_primary &&
		    link_is_imac5k_secondary_power_route(other))
			return other;
		if (self_is_secondary &&
		    link_is_imac5k_primary_power_route(other))
			return other;
	}
	return NULL;
}

static const char *link_imac5k_symclk_state_name(enum symclk_state state)
{
	switch (state) {
	case SYMCLK_OFF_TX_OFF: return "off/off";
	case SYMCLK_ON_TX_ON:   return "on/on";
	case SYMCLK_ON_TX_OFF:  return "on/off";
	default:                return "unknown";
	}
}

void dp_imac5k_log_phy_event(struct dc_link *link, const char *func,
			     const char *checkpoint)
{
	const char *role = link_imac5k_role(link);
	bool hw_hpd;

	if (!role)
		return;

	hw_hpd = link->dc && link->dc->link_srv ?
		dc_link_get_hpd_state(link) : false;

	DC_LOG_WARNING("IMAC5K: phy_trace func=%s checkpoint=%s role=%s link=%u signal=%d tx=%d ddc_hw=%u rate=%d lanes=%d link_active=%u link_state_valid=%u hpd_cached=%u hpd_hw=%u symclk_state=%s preserve_trained=%u skip_d3=%u handoff_allowed=%u handoff_consumed=%u stream_state=%d evidence=0x%x dpcd_proof=%u proof202=0x%02x proof203=0x%02x\n",
		       func ? func : "<none>",
		       checkpoint ? checkpoint : "<none>",
		       role, link->link_index, link->connector_signal,
		       link->link_enc ? link->link_enc->transmitter : -1,
		       link->ddc_hw_inst,
		       link->cur_link_settings.link_rate,
		       link->cur_link_settings.lane_count,
		       link->link_status.link_active, link->link_state_valid,
		       link->hpd_status, hw_hpd,
		       link_imac5k_symclk_state_name(link->phy_state.symclk_state),
		       link->imac5k_trained_link_preserved,
		       link->imac5k_skip_d3_after_trained,
		       link->imac5k_cached_link_handoff_allowed ? 1 : 0,
		       link->imac5k_cached_link_handoff_consumed ? 1 : 0,
		       link->imac5k_stream_enable_state,
		       link->imac5k_cached_link_aux_evidence,
		       link->imac5k_stream_state_dpcd_valid,
		       link->imac5k_stream_state_dpcd_202,
		       link->imac5k_stream_state_dpcd_203);
}

void dp_imac5k_probe_peer_aux(struct dc_link *acting_link,
			      const char *func, const char *checkpoint)
{
	struct dc_link *peer = link_imac5k_find_peer(acting_link);
	/* DC_LOGGER macro expects a variable named "link" in scope. */
	struct dc_link *link = acting_link;
	const char *self_role;
	const char *peer_role;
	u8 dpcd_rev = 0xff;
	enum dc_status status;
	bool peer_hw_hpd;

	if (!peer)
		return;

	self_role = link_imac5k_role(acting_link);
	peer_role = link_imac5k_role(peer);
	peer_hw_hpd = peer->dc && peer->dc->link_srv ?
		dc_link_get_hpd_state(peer) : false;

	status = core_link_read_dpcd(peer, DP_DPCD_REV, &dpcd_rev,
				     sizeof(dpcd_rev));

	DC_LOG_WARNING("IMAC5K: aux_probe func=%s checkpoint=%s self_role=%s self_link=%u peer_role=%s peer_link=%u dpcd_rev=0x%02x status=%d peer_hpd_cached=%u peer_hpd_hw=%u peer_link_active=%u peer_link_state_valid=%u peer_rate=%d peer_lanes=%d peer_symclk_state=%s peer_preserve_trained=%u peer_skip_d3=%u peer_handoff_allowed=%u peer_handoff_consumed=%u peer_stream_state=%d peer_evidence=0x%x peer_dpcd_proof=%u peer_proof202=0x%02x peer_proof203=0x%02x\n",
		       func ? func : "<none>",
		       checkpoint ? checkpoint : "<none>",
		       self_role ? self_role : "n/a",
		       acting_link->link_index,
		       peer_role ? peer_role : "n/a",
		       peer->link_index,
		       dpcd_rev, status,
		       peer->hpd_status, peer_hw_hpd,
		       peer->link_status.link_active,
		       peer->link_state_valid,
		       peer->cur_link_settings.link_rate,
		       peer->cur_link_settings.lane_count,
		       link_imac5k_symclk_state_name(peer->phy_state.symclk_state),
		       peer->imac5k_trained_link_preserved,
		       peer->imac5k_skip_d3_after_trained,
		       peer->imac5k_cached_link_handoff_allowed ? 1 : 0,
		       peer->imac5k_cached_link_handoff_consumed ? 1 : 0,
		       peer->imac5k_stream_enable_state,
		       peer->imac5k_cached_link_aux_evidence,
		       peer->imac5k_stream_state_dpcd_valid,
		       peer->imac5k_stream_state_dpcd_202,
		       peer->imac5k_stream_state_dpcd_203);
}

static bool link_should_preserve_imac5k_secondary_source_output(
	const struct dc_link *link)
{
	return link_is_imac5k_secondary_power_route(link) &&
	       link->imac5k_trained_link_preserved &&
	       link->imac5k_skip_d3_after_trained &&
	       !link->imac5k_cached_link_handoff_consumed;
}

/*
 * Public wrapper so hw-sequencer paths that bypass dp_disable_link_phy()
 * (e.g. dce110 power_down_encoders()) can honour the same preservation
 * decision for the trained iMac 5K secondary 0x3113 route.
 */
bool dp_imac5k_link_preserves_secondary_output(const struct dc_link *link)
{
	return link_should_preserve_imac5k_secondary_source_output(link);
}

/*
 * True if any link in this dc currently needs the iMac 5K secondary 0x3113
 * route preserved. Used by dc-wide teardown steps (clock sources, etc.) that
 * cannot make a per-link decision.
 */
bool dp_imac5k_any_link_preserves_secondary_output(const struct dc *dc)
{
	unsigned int i;

	if (!dc)
		return false;

	for (i = 0; i < dc->link_count; i++) {
		if (link_should_preserve_imac5k_secondary_source_output(
				dc->links[i]))
			return true;
	}
	return false;
}

void dpcd_write_rx_power_ctrl(struct dc_link *link, bool on)
{
	uint8_t state;
	enum dc_status status;

	state = on ? DP_POWER_STATE_D0 : DP_POWER_STATE_D3;

	if (link->sync_lt_in_progress) {
		if (link_is_imac5k_secondary_power_route(link))
			DC_LOG_WARNING("IMAC5K: secondary 0x3113 DP_SET_POWER skipped link=%u requested=0x%02x reason=sync-lt-in-progress cur_rate=%d cur_lanes=%d\n",
				       link->link_index, state,
				       link->cur_link_settings.link_rate,
				       link->cur_link_settings.lane_count);
		return;
	}

	if (!on && link_is_imac5k_secondary_power_route(link) &&
	    link->imac5k_skip_d3_after_trained &&
	    link->imac5k_trained_link_preserved) {
		DC_LOG_WARNING("IMAC5K: secondary 0x3113 DP_SET_POWER skipped link=%u requested=0x%02x reason=trained-link-preserve cur_rate=%d cur_lanes=%d proof_rate=%d proof_lanes=%d proof202=0x%02x proof203=0x%02x dpcd005=0x%02x dpcd080=0x%02x dpcd600=0x%02x branch500_508=%02x %02x %02x %02x %02x %02x %02x %02x %02x evidence=0x%x stream_state=%d handoff_allowed=%u handoff_consumed=%u link_active=%u link_state_valid=%u\n",
			       link->link_index, state,
			       link->cur_link_settings.link_rate,
			       link->cur_link_settings.lane_count,
			       link->imac5k_stream_state_dpcd_settings.link_rate,
			       link->imac5k_stream_state_dpcd_settings.lane_count,
			       link->imac5k_stream_state_dpcd_202,
			       link->imac5k_stream_state_dpcd_203,
			       link->imac5k_trained_link_dpcd_005,
			       link->imac5k_trained_link_dpcd_080,
			       link->imac5k_trained_link_dpcd_600,
			       link->imac5k_trained_link_branch_500_508[0],
			       link->imac5k_trained_link_branch_500_508[1],
			       link->imac5k_trained_link_branch_500_508[2],
			       link->imac5k_trained_link_branch_500_508[3],
			       link->imac5k_trained_link_branch_500_508[4],
			       link->imac5k_trained_link_branch_500_508[5],
			       link->imac5k_trained_link_branch_500_508[6],
			       link->imac5k_trained_link_branch_500_508[7],
			       link->imac5k_trained_link_branch_500_508[8],
			       link->imac5k_cached_link_aux_evidence,
			       link->imac5k_stream_enable_state,
			       link->imac5k_cached_link_handoff_allowed ? 1 : 0,
			       link->imac5k_cached_link_handoff_consumed ? 1 : 0,
			       link->link_status.link_active,
			       link->link_state_valid);
		return;
	}

	if (link_imac5k_role(link)) {
		dp_imac5k_log_phy_event(link, "dpcd_write_rx_power_ctrl",
					"pre-write");
		dp_imac5k_probe_peer_aux(link, "dpcd_write_rx_power_ctrl",
					 "pre-write");
	}

	status = core_link_write_dpcd(link, DP_SET_POWER, &state,
				     sizeof(state));
	if (link_is_imac5k_secondary_power_route(link))
		DC_LOG_WARNING("IMAC5K: secondary 0x3113 DP_SET_POWER write link=%u on=%u value=0x%02x status=%d cur_rate=%d cur_lanes=%d proof_preserved=%u skip_d3=%u link_active=%u link_state_valid=%u\n",
			       link->link_index, on, state, status,
			       link->cur_link_settings.link_rate,
			       link->cur_link_settings.lane_count,
			       link->imac5k_trained_link_preserved,
			       link->imac5k_skip_d3_after_trained,
			       link->link_status.link_active,
			       link->link_state_valid);

	if (link_imac5k_role(link)) {
		dp_imac5k_log_phy_event(link, "dpcd_write_rx_power_ctrl",
					"post-write");
		dp_imac5k_probe_peer_aux(link, "dpcd_write_rx_power_ctrl",
					 "post-write");
	}
}

void dp_enable_link_phy(
	struct dc_link *link,
	const struct link_resource *link_res,
	enum signal_type signal,
	enum clock_source_id clock_source,
	const struct dc_link_settings *link_settings)
{
	dp_imac5k_log_phy_event(link, "dp_enable_link_phy", "entry");
	dp_imac5k_probe_peer_aux(link, "dp_enable_link_phy", "entry");

	link->cur_link_settings = *link_settings;
	link->dc->hwss.enable_dp_link_output(link, link_res, signal,
			clock_source, link_settings);

	dp_imac5k_log_phy_event(link, "dp_enable_link_phy",
				"after-enable-dp-link-output");
	dp_imac5k_probe_peer_aux(link, "dp_enable_link_phy",
				 "after-enable-dp-link-output");

	dpcd_write_rx_power_ctrl(link, true);

	dp_imac5k_log_phy_event(link, "dp_enable_link_phy", "exit");
	dp_imac5k_probe_peer_aux(link, "dp_enable_link_phy", "exit");
}

void dp_disable_link_phy(struct dc_link *link,
		const struct link_resource *link_res,
		enum signal_type signal)
{
	struct dc  *dc = link->ctx->dc;

	dp_imac5k_log_phy_event(link, "dp_disable_link_phy", "entry");
	dp_imac5k_probe_peer_aux(link, "dp_disable_link_phy", "entry");

	if (!link->wa_flags.dp_keep_receiver_powered &&
			!link->skip_implict_edp_power_control &&
			link->type != dc_connection_none)
		dpcd_write_rx_power_ctrl(link, false);

	if (link_should_preserve_imac5k_secondary_source_output(link)) {
		DC_LOG_WARNING("IMAC5K: secondary 0x3113 source-output disable skipped link=%u reason=trained-link-preserve cur_rate=%d cur_lanes=%d proof_rate=%d proof_lanes=%d proof202=0x%02x proof203=0x%02x link_active=%u link_state_valid=%u stream_state=%d handoff_allowed=%u handoff_consumed=%u\n",
			       link->link_index,
			       link->cur_link_settings.link_rate,
			       link->cur_link_settings.lane_count,
			       link->imac5k_stream_state_dpcd_settings.link_rate,
			       link->imac5k_stream_state_dpcd_settings.lane_count,
			       link->imac5k_stream_state_dpcd_202,
			       link->imac5k_stream_state_dpcd_203,
			       link->link_status.link_active,
			       link->link_state_valid,
			       link->imac5k_stream_enable_state,
			       link->imac5k_cached_link_handoff_allowed ? 1 : 0,
			       link->imac5k_cached_link_handoff_consumed ? 1 : 0);
		dp_imac5k_log_phy_event(link, "dp_disable_link_phy",
					"exit-preserved");
		dp_imac5k_probe_peer_aux(link, "dp_disable_link_phy",
					 "exit-preserved");
		return;
	}

	dp_imac5k_log_phy_event(link, "dp_disable_link_phy",
				"before-hwss-disable");
	dp_imac5k_probe_peer_aux(link, "dp_disable_link_phy",
				 "before-hwss-disable");

	dc->hwss.disable_link_output(link, link_res, signal);
	/* Clear current link setting.*/
	memset(&link->cur_link_settings, 0,
			sizeof(link->cur_link_settings));

	dp_imac5k_log_phy_event(link, "dp_disable_link_phy",
				"after-hwss-disable");
	dp_imac5k_probe_peer_aux(link, "dp_disable_link_phy",
				 "after-hwss-disable");

	if (dc->clk_mgr->funcs->notify_link_rate_change)
		dc->clk_mgr->funcs->notify_link_rate_change(dc->clk_mgr, link);

	dp_imac5k_log_phy_event(link, "dp_disable_link_phy", "exit");
	dp_imac5k_probe_peer_aux(link, "dp_disable_link_phy", "exit");
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

