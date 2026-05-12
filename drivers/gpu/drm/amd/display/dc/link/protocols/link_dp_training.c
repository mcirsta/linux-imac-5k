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
 * This file implements all generic dp link training helper functions and top
 * level generic training sequence. All variations of dp link training sequence
 * should be called inside the top level training functions in this file to
 * ensure the integrity of our overall training procedure across different types
 * of link encoding and back end hardware.
 */
#include <linux/string.h>

#include "link_dp_training.h"
#include "link_dp_training_8b_10b.h"
#include "link_dp_training_128b_132b.h"
#include "link_dp_training_auxless.h"
#include "link_dp_training_dpia.h"
#include "link_dp_training_fixed_vs_pe_retimer.h"
#include "link_dpcd.h"
#include "link/accessories/link_dp_trace.h"
#include "link_dp_phy.h"
#include "link_dp_capability.h"
#include "link_edp_panel_control.h"
#include "link/link_detection.h"
#include "link/link_validation.h"
#include "atomfirmware.h"
#include "link_enc_cfg.h"
#include "grph_object_id.h"
#include "resource.h"
#include "dm_helpers.h"

#define DC_LOGGER \
	link->ctx->logger

#define POST_LT_ADJ_REQ_LIMIT 6
#define POST_LT_ADJ_REQ_TIMEOUT 200
#define LINK_TRAINING_RETRY_DELAY 50 /* ms */
#define IMAC5K_WIN_SECONDARY_OBJECT_ID 0x3113
#define IMAC5K_WIN_SECONDARY_DDC_HW_INST 2
#define IMAC5K_CACHED_LINK_EVIDENCE_10A (1U << 0)
#define IMAC5K_CACHED_LINK_EVIDENCE_SOURCE_DPCD (1U << 1)
#define IMAC5K_WINDOWS_ORDER_EXPECTED_EVIDENCE \
	(IMAC5K_CACHED_LINK_EVIDENCE_10A | \
	 IMAC5K_CACHED_LINK_EVIDENCE_SOURCE_DPCD)
#ifndef DP_SOURCE_TABLE_REVISION
#define DP_SOURCE_TABLE_REVISION 0x310
#endif
#define IMAC5K_DPCD_PANEL_LATCH 0x4F1

void dp_log_training_result(
	struct dc_link *link,
	const struct link_training_settings *lt_settings,
	enum link_training_result status)
{
	char *link_rate = "Unknown";
	char *lt_result = "Unknown";
	char *lt_spread = "Disabled";

	switch (lt_settings->link_settings.link_rate) {
	case LINK_RATE_LOW:
		link_rate = "RBR";
		break;
	case LINK_RATE_RATE_2:
		link_rate = "R2";
		break;
	case LINK_RATE_RATE_3:
		link_rate = "R3";
		break;
	case LINK_RATE_HIGH:
		link_rate = "HBR";
		break;
	case LINK_RATE_RBR2:
		link_rate = "RBR2";
		break;
	case LINK_RATE_RATE_6:
		link_rate = "R6";
		break;
	case LINK_RATE_HIGH2:
		link_rate = "HBR2";
		break;
	case LINK_RATE_RATE_8:
		link_rate = "R8";
		break;
	case LINK_RATE_HIGH3:
		link_rate = "HBR3";
		break;
	case LINK_RATE_UHBR10:
		link_rate = "UHBR10";
		break;
	case LINK_RATE_UHBR13_5:
		link_rate = "UHBR13.5";
		break;
	case LINK_RATE_UHBR20:
		link_rate = "UHBR20";
		break;
	default:
		break;
	}

	switch (status) {
	case LINK_TRAINING_SUCCESS:
		lt_result = "pass";
		break;
	case LINK_TRAINING_CR_FAIL_LANE0:
		lt_result = "CR failed lane0";
		break;
	case LINK_TRAINING_CR_FAIL_LANE1:
		lt_result = "CR failed lane1";
		break;
	case LINK_TRAINING_CR_FAIL_LANE23:
		lt_result = "CR failed lane23";
		break;
	case LINK_TRAINING_EQ_FAIL_CR:
		lt_result = "CR failed in EQ";
		break;
	case LINK_TRAINING_EQ_FAIL_CR_PARTIAL:
		lt_result = "CR failed in EQ partially";
		break;
	case LINK_TRAINING_EQ_FAIL_EQ:
		lt_result = "EQ failed";
		break;
	case LINK_TRAINING_LQA_FAIL:
		lt_result = "LQA failed";
		break;
	case LINK_TRAINING_LINK_LOSS:
		lt_result = "Link loss";
		break;
	case DP_128b_132b_LT_FAILED:
		lt_result = "LT_FAILED received";
		break;
	case DP_128b_132b_MAX_LOOP_COUNT_REACHED:
		lt_result = "max loop count reached";
		break;
	case DP_128b_132b_CHANNEL_EQ_DONE_TIMEOUT:
		lt_result = "channel EQ timeout";
		break;
	case DP_128b_132b_CDS_DONE_TIMEOUT:
		lt_result = "CDS timeout";
		break;
	default:
		break;
	}

	switch (lt_settings->link_settings.link_spread) {
	case LINK_SPREAD_DISABLED:
		lt_spread = "Disabled";
		break;
	case LINK_SPREAD_05_DOWNSPREAD_30KHZ:
		lt_spread = "0.5% 30KHz";
		break;
	case LINK_SPREAD_05_DOWNSPREAD_33KHZ:
		lt_spread = "0.5% 33KHz";
		break;
	default:
		break;
	}

	/* Connectivity log: link training */

	/* TODO - DP2.0 Log: add connectivity log for FFE PRESET */

	CONN_MSG_LT(link, "%sx%d %s VS=%d, PE=%d, DS=%s",
				link_rate,
				lt_settings->link_settings.lane_count,
				lt_result,
				lt_settings->hw_lane_settings[0].VOLTAGE_SWING,
				lt_settings->hw_lane_settings[0].PRE_EMPHASIS,
				lt_spread);
}

uint8_t dp_initialize_scrambling_data_symbols(
	struct dc_link *link,
	enum dc_dp_training_pattern pattern)
{
	uint8_t disable_scrabled_data_symbols = 0;

	switch (pattern) {
	case DP_TRAINING_PATTERN_SEQUENCE_1:
	case DP_TRAINING_PATTERN_SEQUENCE_2:
	case DP_TRAINING_PATTERN_SEQUENCE_3:
		disable_scrabled_data_symbols = 1;
		break;
	case DP_TRAINING_PATTERN_SEQUENCE_4:
	case DP_128b_132b_TPS1:
	case DP_128b_132b_TPS2:
		disable_scrabled_data_symbols = 0;
		break;
	default:
		ASSERT(0);
		DC_LOG_HW_LINK_TRAINING("%s: Invalid HW Training pattern: %d\n",
			__func__, pattern);
		break;
	}
	return disable_scrabled_data_symbols;
}

enum dpcd_training_patterns
	dp_training_pattern_to_dpcd_training_pattern(
	struct dc_link *link,
	enum dc_dp_training_pattern pattern)
{
	enum dpcd_training_patterns dpcd_tr_pattern =
	DPCD_TRAINING_PATTERN_VIDEOIDLE;

	switch (pattern) {
	case DP_TRAINING_PATTERN_SEQUENCE_1:
		DC_LOG_HW_LINK_TRAINING("%s: Using DP training pattern TPS1\n", __func__);
		dpcd_tr_pattern = DPCD_TRAINING_PATTERN_1;
		break;
	case DP_TRAINING_PATTERN_SEQUENCE_2:
		DC_LOG_HW_LINK_TRAINING("%s: Using DP training pattern TPS2\n", __func__);
		dpcd_tr_pattern = DPCD_TRAINING_PATTERN_2;
		break;
	case DP_TRAINING_PATTERN_SEQUENCE_3:
		DC_LOG_HW_LINK_TRAINING("%s: Using DP training pattern TPS3\n", __func__);
		dpcd_tr_pattern = DPCD_TRAINING_PATTERN_3;
		break;
	case DP_TRAINING_PATTERN_SEQUENCE_4:
		DC_LOG_HW_LINK_TRAINING("%s: Using DP training pattern TPS4\n", __func__);
		dpcd_tr_pattern = DPCD_TRAINING_PATTERN_4;
		break;
	case DP_128b_132b_TPS1:
		DC_LOG_HW_LINK_TRAINING("%s: Using DP 128b/132b training pattern TPS1\n", __func__);
		dpcd_tr_pattern = DPCD_128b_132b_TPS1;
		break;
	case DP_128b_132b_TPS2:
		DC_LOG_HW_LINK_TRAINING("%s: Using DP 128b/132b training pattern TPS2\n", __func__);
		dpcd_tr_pattern = DPCD_128b_132b_TPS2;
		break;
	case DP_128b_132b_TPS2_CDS:
		DC_LOG_HW_LINK_TRAINING("%s: Using DP 128b/132b training pattern TPS2 CDS\n",
					__func__);
		dpcd_tr_pattern = DPCD_128b_132b_TPS2_CDS;
		break;
	case DP_TRAINING_PATTERN_VIDEOIDLE:
		DC_LOG_HW_LINK_TRAINING("%s: Using DP training pattern videoidle\n", __func__);
		dpcd_tr_pattern = DPCD_TRAINING_PATTERN_VIDEOIDLE;
		break;
	default:
		ASSERT(0);
		DC_LOG_HW_LINK_TRAINING("%s: Invalid HW Training pattern: %d\n",
			__func__, pattern);
		break;
	}

	return dpcd_tr_pattern;
}

uint8_t dp_get_nibble_at_index(const uint8_t *buf,
	uint32_t index)
{
	uint8_t nibble;
	nibble = buf[index / 2];

	if (index % 2)
		nibble >>= 4;
	else
		nibble &= 0x0F;

	return nibble;
}

void dp_wait_for_training_aux_rd_interval(
	struct dc_link *link,
	uint32_t wait_in_micro_secs)
{
	usleep_range_state(wait_in_micro_secs, wait_in_micro_secs, TASK_UNINTERRUPTIBLE);

	DC_LOG_HW_LINK_TRAINING("%s:\n wait = %d\n",
		__func__,
		wait_in_micro_secs);
}

/* maximum pre emphasis level allowed for each voltage swing level*/
static const enum dc_pre_emphasis voltage_swing_to_pre_emphasis[] = {
		PRE_EMPHASIS_LEVEL3,
		PRE_EMPHASIS_LEVEL2,
		PRE_EMPHASIS_LEVEL1,
		PRE_EMPHASIS_DISABLED };

static enum dc_pre_emphasis get_max_pre_emphasis_for_voltage_swing(
	enum dc_voltage_swing voltage)
{
	enum dc_pre_emphasis pre_emphasis;
	pre_emphasis = PRE_EMPHASIS_MAX_LEVEL;

	if (voltage <= VOLTAGE_SWING_MAX_LEVEL)
		pre_emphasis = voltage_swing_to_pre_emphasis[voltage];

	return pre_emphasis;

}

static void maximize_lane_settings(const struct link_training_settings *lt_settings,
		struct dc_lane_settings lane_settings[LANE_COUNT_DP_MAX])
{
	uint32_t lane;
	struct dc_lane_settings max_requested;

	max_requested.VOLTAGE_SWING = lane_settings[0].VOLTAGE_SWING;
	max_requested.PRE_EMPHASIS = lane_settings[0].PRE_EMPHASIS;
	max_requested.FFE_PRESET = lane_settings[0].FFE_PRESET;

	/* Determine what the maximum of the requested settings are*/
	for (lane = 1; lane < lt_settings->link_settings.lane_count; lane++) {
		if (lane_settings[lane].VOLTAGE_SWING > max_requested.VOLTAGE_SWING)
			max_requested.VOLTAGE_SWING = lane_settings[lane].VOLTAGE_SWING;

		if (lane_settings[lane].PRE_EMPHASIS > max_requested.PRE_EMPHASIS)
			max_requested.PRE_EMPHASIS = lane_settings[lane].PRE_EMPHASIS;
		if (lane_settings[lane].FFE_PRESET.settings.level >
				max_requested.FFE_PRESET.settings.level)
			max_requested.FFE_PRESET.settings.level =
					lane_settings[lane].FFE_PRESET.settings.level;
	}

	/* make sure the requested settings are
	 * not higher than maximum settings*/
	if (max_requested.VOLTAGE_SWING > VOLTAGE_SWING_MAX_LEVEL)
		max_requested.VOLTAGE_SWING = VOLTAGE_SWING_MAX_LEVEL;

	if (max_requested.PRE_EMPHASIS > PRE_EMPHASIS_MAX_LEVEL)
		max_requested.PRE_EMPHASIS = PRE_EMPHASIS_MAX_LEVEL;

	/* Note, we are not checking
	 * if max_requested.FFE_PRESET.settings.level >  DP_FFE_PRESET_MAX_LEVEL,
	 * since FFE_PRESET.settings.level is 4 bits and DP_FFE_PRESET_MAX_LEVEL equals 15,
	 * so FFE_PRESET.settings.level will never be greater than 15.
	 */

	/* make sure the pre-emphasis matches the voltage swing*/
	if (max_requested.PRE_EMPHASIS >
		get_max_pre_emphasis_for_voltage_swing(
			max_requested.VOLTAGE_SWING))
		max_requested.PRE_EMPHASIS =
		get_max_pre_emphasis_for_voltage_swing(
			max_requested.VOLTAGE_SWING);

	for (lane = 0; lane < LANE_COUNT_DP_MAX; lane++) {
		lane_settings[lane].VOLTAGE_SWING = max_requested.VOLTAGE_SWING;
		lane_settings[lane].PRE_EMPHASIS = max_requested.PRE_EMPHASIS;
		lane_settings[lane].FFE_PRESET = max_requested.FFE_PRESET;
	}
}

void dp_hw_to_dpcd_lane_settings(
		const struct link_training_settings *lt_settings,
		const struct dc_lane_settings hw_lane_settings[LANE_COUNT_DP_MAX],
		union dpcd_training_lane dpcd_lane_settings[LANE_COUNT_DP_MAX])
{
	uint8_t lane = 0;

	for (lane = 0; lane < LANE_COUNT_DP_MAX; lane++) {
		if (link_dp_get_encoding_format(&lt_settings->link_settings) ==
				DP_8b_10b_ENCODING) {
			dpcd_lane_settings[lane].bits.VOLTAGE_SWING_SET =
					(uint8_t)(hw_lane_settings[lane].VOLTAGE_SWING);
			dpcd_lane_settings[lane].bits.PRE_EMPHASIS_SET =
					(uint8_t)(hw_lane_settings[lane].PRE_EMPHASIS);
			dpcd_lane_settings[lane].bits.MAX_SWING_REACHED =
					(hw_lane_settings[lane].VOLTAGE_SWING ==
							VOLTAGE_SWING_MAX_LEVEL ? 1 : 0);
			dpcd_lane_settings[lane].bits.MAX_PRE_EMPHASIS_REACHED =
					(hw_lane_settings[lane].PRE_EMPHASIS ==
							PRE_EMPHASIS_MAX_LEVEL ? 1 : 0);
		} else if (link_dp_get_encoding_format(&lt_settings->link_settings) ==
				DP_128b_132b_ENCODING) {
			dpcd_lane_settings[lane].tx_ffe.PRESET_VALUE =
					hw_lane_settings[lane].FFE_PRESET.settings.level;
		}
	}
}

uint8_t get_dpcd_link_rate(const struct dc_link_settings *link_settings)
{
	uint8_t link_rate = 0;
	enum dp_link_encoding encoding = link_dp_get_encoding_format(link_settings);

	if (encoding == DP_128b_132b_ENCODING)
		switch (link_settings->link_rate) {
		case LINK_RATE_UHBR10:
			link_rate = 0x1;
			break;
		case LINK_RATE_UHBR20:
			link_rate = 0x2;
			break;
		case LINK_RATE_UHBR13_5:
			link_rate = 0x4;
			break;
		default:
			link_rate = 0;
			break;
		}
	else if (encoding == DP_8b_10b_ENCODING)
		link_rate = (uint8_t) link_settings->link_rate;
	else
		link_rate = 0;

	return link_rate;
}

static bool link_is_imac5k_secondary_windows_route(const struct dc_link *link)
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

static bool link_is_imac5k_secondary_windows_candidate(const struct dc_link *link)
{
	uint32_t raw_obj;

	if (!link)
		return false;

	raw_obj = dal_graphics_object_id_to_uint(link->link_id);

	return raw_obj == IMAC5K_WIN_SECONDARY_OBJECT_ID ||
	       link->ddc_hw_inst == IMAC5K_WIN_SECONDARY_DDC_HW_INST ||
	       (link->link_enc &&
		link->link_enc->transmitter == TRANSMITTER_UNIPHY_D);
}

static bool imac5k_secondary_should_use_windows_link_settings_order(
	const struct dc_link *link,
	const struct link_training_settings *lt_settings)
{
	if (!link_is_imac5k_secondary_windows_route(link) || !lt_settings)
		return false;

	if (link_dp_get_encoding_format(&lt_settings->link_settings) !=
	    DP_8b_10b_ENCODING)
		return false;

	if (lt_settings->link_settings.use_link_rate_set)
		return false;

	return true;
}

static const char *imac5k_secondary_windows_order_skip_reason(
	const struct dc_link *link,
	const struct link_training_settings *lt_settings)
{
	uint32_t raw_obj;

	if (!link)
		return "no-link";

	if (!lt_settings)
		return "no-lt-settings";

	raw_obj = dal_graphics_object_id_to_uint(link->link_id);
	if (link->connector_signal != SIGNAL_TYPE_DISPLAY_PORT)
		return "not-displayport";

	if (raw_obj != IMAC5K_WIN_SECONDARY_OBJECT_ID)
		return "not-0x3113";

	if (link->ddc_hw_inst != IMAC5K_WIN_SECONDARY_DDC_HW_INST)
		return "not-ddc3";

	if (!link->link_enc)
		return "no-link-encoder";

	if (link->link_enc->transmitter != TRANSMITTER_UNIPHY_D)
		return "not-uniphy-d";

	if (link_dp_get_encoding_format(&lt_settings->link_settings) !=
	    DP_8b_10b_ENCODING)
		return "not-8b10b";

	if (lt_settings->link_settings.use_link_rate_set)
		return "uses-link-rate-set";

	return "unknown";
}

static void imac5k_secondary_log_windows_order_skip(
	const struct dc_link *link,
	const struct link_training_settings *lt_settings,
	const union lane_count_set *lane_count_set,
	const union down_spread_ctrl *downspread)
{
	uint32_t raw_obj = link ? dal_graphics_object_id_to_uint(link->link_id) : 0;
	uint32_t evidence = link ? link->imac5k_cached_link_aux_evidence : 0;
	uint32_t missing_expected_evidence =
		IMAC5K_WINDOWS_ORDER_EXPECTED_EVIDENCE &
		~evidence;
	enum dp_link_encoding encoding = lt_settings ?
		link_dp_get_encoding_format(&lt_settings->link_settings) :
		DP_UNKNOWN_ENCODING;

	DC_LOG_WARNING("IMAC5K: secondary 0x3113 Windows-order skipped before generic link-settings reason=%s link=%u raw_obj=0x%x signal=%d ddc_hw=%u tx=%d route_ok=%u encoding=%d use_lrs=%u req_rate=%d req_lanes=%d spread=%d enhanced=%d pattern_for_eq=%d dpcd_rev=0x%02x evidence=0x%x missing_expected_evidence=0x%x stream_state=%d dpcd_proof=%u handoff_allowed=%u handoff_consumed=%u cur_rate=%d cur_lanes=%d dpcd100=0x%02x dpcd101=0x%02x dpcd107=0x%02x\n",
		       imac5k_secondary_windows_order_skip_reason(link,
								  lt_settings),
		       link ? link->link_index : 0, raw_obj,
		       link ? link->connector_signal : SIGNAL_TYPE_NONE,
		       link ? link->ddc_hw_inst : 0,
		       (link && link->link_enc) ?
			       link->link_enc->transmitter : -1,
		       link_is_imac5k_secondary_windows_route(link) ? 1 : 0,
		       encoding,
		       lt_settings ?
			       lt_settings->link_settings.use_link_rate_set : 0,
		       lt_settings ? lt_settings->link_settings.link_rate : 0,
		       lt_settings ? lt_settings->link_settings.lane_count : 0,
		       lt_settings ? lt_settings->link_settings.link_spread : 0,
		       lt_settings ? lt_settings->enhanced_framing : 0,
		       lt_settings ? lt_settings->pattern_for_eq : 0,
		       link ? link->dpcd_caps.dpcd_rev.raw : 0,
		       evidence, missing_expected_evidence,
		       link ? link->imac5k_stream_enable_state : 0,
		       link ? link->imac5k_stream_state_dpcd_valid : 0,
		       (link && link->imac5k_cached_link_handoff_allowed) ? 1 : 0,
		       (link && link->imac5k_cached_link_handoff_consumed) ? 1 : 0,
		       link ? link->cur_link_settings.link_rate : 0,
		       link ? link->cur_link_settings.lane_count : 0,
		       lt_settings ? get_dpcd_link_rate(
				       &lt_settings->link_settings) : 0,
		       lane_count_set ? lane_count_set->raw : 0,
		       downspread ? downspread->raw : 0);
}

static void imac5k_secondary_log_windows_order_preconditions(
	struct dc_link *link)
{
	uint8_t dpcd_10a = 0;
	uint8_t source_table[3] = { 0 };
	uint8_t dpcd_4f1 = 0;
	uint8_t dpcd_205 = 0;
	uint8_t dpcd_100_101[2] = { 0 };
	uint8_t dpcd_202_203[2] = { 0 };
	enum dc_status status_10a;
	enum dc_status status_310;
	enum dc_status status_4f1;
	enum dc_status status_205;
	enum dc_status status_100_101;
	enum dc_status status_202_203;
	bool hpd_hw = link->dc && link->dc->link_srv &&
		link->dc->link_srv->get_hpd_state ?
		link->dc->link_srv->get_hpd_state(link) : false;

	status_10a = core_link_read_dpcd(link, DP_EDP_CONFIGURATION_SET,
					&dpcd_10a, sizeof(dpcd_10a));
	status_310 = core_link_read_dpcd(link, DP_SOURCE_TABLE_REVISION,
					source_table, sizeof(source_table));
	status_4f1 = core_link_read_dpcd(link, IMAC5K_DPCD_PANEL_LATCH,
					&dpcd_4f1, sizeof(dpcd_4f1));
	status_205 = core_link_read_dpcd(link, DP_SINK_STATUS,
					&dpcd_205, sizeof(dpcd_205));
	status_100_101 = core_link_read_dpcd(link, DP_LINK_BW_SET,
					    dpcd_100_101,
					    sizeof(dpcd_100_101));
	status_202_203 = core_link_read_dpcd(link,
					    DP_LANE0_1_STATUS,
					    dpcd_202_203,
					    sizeof(dpcd_202_203));

	DC_LOG_WARNING("IMAC5K: secondary 0x3113 Windows-order precondition snapshot link=%u evidence=0x%x missing_expected_evidence=0x%x stream_state=%d handoff_allowed=%u handoff_consumed=%u 0x10A=0x%02x status10a=%d assr=%u 0x310=%02x %02x %02x status310=%d source_like=%u 0x4F1=0x%02x status4f1=%d 0x205=0x%02x status205=%d 0x100=0x%02x 0x101=0x%02x status100_101=%d 0x202=0x%02x 0x203=0x%02x status202_203=%d link_active=%u link_state_valid=%u hpd=%u/%u cur_rate=%d cur_lanes=%d\n",
		       link->link_index, link->imac5k_cached_link_aux_evidence,
		       IMAC5K_WINDOWS_ORDER_EXPECTED_EVIDENCE &
			       ~link->imac5k_cached_link_aux_evidence,
		       link->imac5k_stream_enable_state,
		       link->imac5k_cached_link_handoff_allowed ? 1 : 0,
		       link->imac5k_cached_link_handoff_consumed ? 1 : 0,
		       dpcd_10a, status_10a,
		       !!(dpcd_10a & DP_ALTERNATE_SCRAMBLER_RESET_ENABLE),
		       source_table[0], source_table[1], source_table[2],
		       status_310,
		       status_310 == DC_OK &&
			       source_table[1] == 0x1d &&
			       source_table[2] == 0x03,
		       dpcd_4f1, status_4f1, dpcd_205, status_205,
		       dpcd_100_101[0], dpcd_100_101[1], status_100_101,
		       dpcd_202_203[0], dpcd_202_203[1], status_202_203,
		       link->link_status.link_active, link->link_state_valid,
		       link->hpd_status, hpd_hw,
		       link->cur_link_settings.link_rate,
		       link->cur_link_settings.lane_count);
}

static void imac5k_secondary_log_training_dpcd_window(
	struct dc_link *link,
	const struct link_training_settings *lt_settings,
	enum dc_dp_training_pattern pattern,
	uint32_t offset,
	uint32_t dpcd_base_lt_offset,
	uint8_t dpcd_pattern_raw,
	enum dc_status write_status,
	const char *stage)
{
	uint8_t dpcd_10a = 0;
	uint8_t source_table[3] = { 0 };
	uint8_t dpcd_102_103[2] = { 0 };
	uint8_t dpcd_202_203[2] = { 0 };
	uint8_t dpcd_600 = 0;
	enum dc_status status_10a;
	enum dc_status status_310;
	enum dc_status status_102_103;
	enum dc_status status_202_203;
	enum dc_status status_600;
	enum dp_link_encoding encoding = lt_settings ?
		link_dp_get_encoding_format(&lt_settings->link_settings) :
		DP_UNKNOWN_ENCODING;

	if (!link_is_imac5k_secondary_windows_route(link))
		return;

	status_10a = core_link_read_dpcd(link, DP_EDP_CONFIGURATION_SET,
					&dpcd_10a, sizeof(dpcd_10a));
	status_310 = core_link_read_dpcd(link, DP_SOURCE_TABLE_REVISION,
					source_table, sizeof(source_table));
	status_102_103 = core_link_read_dpcd(link, DP_TRAINING_PATTERN_SET,
					    dpcd_102_103,
					    sizeof(dpcd_102_103));
	status_202_203 = core_link_read_dpcd(link, DP_LANE0_1_STATUS,
					    dpcd_202_203,
					    sizeof(dpcd_202_203));
	status_600 = core_link_read_dpcd(link, DP_SET_POWER, &dpcd_600,
					sizeof(dpcd_600));

	DC_LOG_WARNING("IMAC5K: secondary 0x3113 training-dpcd %s link=%u offset=%u base=0x%x pattern=%d raw_pattern=0x%02x write_status=%d encoding=%d req_rate=%d req_lanes=%d cur_rate=%d cur_lanes=%d panel_mode=%d 0x10A=0x%02x status10a=%d 0x310=%02x %02x %02x status310=%d 0x102=0x%02x 0x103=0x%02x status102_103=%d 0x202=0x%02x 0x203=0x%02x status202_203=%d 0x600=0x%02x status600=%d evidence=0x%x stream_state=%d\n",
		       stage ? stage : "<none>", link->link_index, offset,
		       dpcd_base_lt_offset, pattern, dpcd_pattern_raw,
		       write_status, encoding,
		       lt_settings ? lt_settings->link_settings.link_rate : 0,
		       lt_settings ? lt_settings->link_settings.lane_count : 0,
		       link->cur_link_settings.link_rate,
		       link->cur_link_settings.lane_count,
		       dp_get_panel_mode(link), dpcd_10a, status_10a,
		       source_table[0], source_table[1], source_table[2],
		       status_310, dpcd_102_103[0], dpcd_102_103[1],
		       status_102_103, dpcd_202_203[0], dpcd_202_203[1],
		       status_202_203, dpcd_600, status_600,
		       link->imac5k_cached_link_aux_evidence,
		       link->imac5k_stream_enable_state);
}

static enum dc_status imac5k_secondary_dpcd_set_link_settings_windows_order(
	struct dc_link *link,
	const struct link_training_settings *lt_settings,
	const union lane_count_set *lane_count_set,
	const union down_spread_ctrl *downspread)
{
	uint8_t rate_lane[2];
	uint8_t rate;
	enum dc_status status;
	uint32_t raw_obj;

	rate = get_dpcd_link_rate(&lt_settings->link_settings);
	if (!rate) {
		DC_LOG_ERROR("IMAC5K: secondary 0x3113 Windows-order link-settings skipped: invalid DPCD rate for link %u req_rate=%d lanes=%d\n",
			     link->link_index,
			     lt_settings->link_settings.link_rate,
			     lt_settings->link_settings.lane_count);
		return DC_ERROR_UNEXPECTED;
	}

	raw_obj = dal_graphics_object_id_to_uint(link->link_id);
	rate_lane[0] = rate;
	rate_lane[1] = lane_count_set->raw;

	DC_LOG_WARNING("IMAC5K: secondary 0x3113 Windows-order link-settings begin link=%u raw_obj=0x%x ddc_hw=%u tx=%d stream_state=%d evidence=0x%x missing_expected_evidence=0x%x forced_exact_route=1 handoff_allowed=%u handoff_consumed=%u dpcd_proof=%u cur_rate=%d cur_lanes=%d req_rate=%d req_lanes=%d dpcd100=0x%02x dpcd101=0x%02x dpcd107=0x%02x\n",
		       link->link_index, raw_obj, link->ddc_hw_inst,
		       link->link_enc ? link->link_enc->transmitter : -1,
		       link->imac5k_stream_enable_state,
		       link->imac5k_cached_link_aux_evidence,
		       IMAC5K_WINDOWS_ORDER_EXPECTED_EVIDENCE &
			       ~link->imac5k_cached_link_aux_evidence,
		       link->imac5k_cached_link_handoff_allowed ? 1 : 0,
		       link->imac5k_cached_link_handoff_consumed ? 1 : 0,
		       link->imac5k_stream_state_dpcd_valid ? 1 : 0,
		       link->cur_link_settings.link_rate,
		       link->cur_link_settings.lane_count,
		       lt_settings->link_settings.link_rate,
		       lt_settings->link_settings.lane_count,
		       rate_lane[0], rate_lane[1], downspread->raw);

	imac5k_secondary_log_windows_order_preconditions(link);

	status = core_link_write_dpcd(link, DP_LINK_BW_SET, rate_lane,
				     sizeof(rate_lane));
	if (status != DC_OK) {
		DC_LOG_ERROR("IMAC5K: secondary 0x3113 Windows-order write 0x100/0x101 failed link=%u status=%d rate=0x%02x lane=0x%02x\n",
			     link->link_index, status, rate_lane[0],
			     rate_lane[1]);
		return status;
	}

	status = core_link_write_dpcd(link, DP_DOWNSPREAD_CTRL,
				     &downspread->raw,
				     sizeof(downspread->raw));
	if (status != DC_OK) {
		DC_LOG_ERROR("IMAC5K: secondary 0x3113 Windows-order write 0x107 failed link=%u status=%d spread=0x%02x\n",
			     link->link_index, status, downspread->raw);
		return status;
	}

	DC_LOG_WARNING("IMAC5K: secondary 0x3113 Windows-order link-settings complete link=%u wrote 0x100/0x101 together then 0x107; normal lane-status training follows\n",
		       link->link_index);
	DC_LOG_HW_LINK_TRAINING("%s\n %x rate = %x\n %x lane = %x framing = %x\n %x spread = %x\n",
		__func__,
		DP_LINK_BW_SET,
		lt_settings->link_settings.link_rate,
		DP_LANE_COUNT_SET,
		lt_settings->link_settings.lane_count,
		lt_settings->enhanced_framing,
		DP_DOWNSPREAD_CTRL,
		lt_settings->link_settings.link_spread);

	return DC_OK;
}

/* Only used for channel equalization */
uint32_t dp_translate_training_aux_read_interval(uint32_t dpcd_aux_read_interval)
{
	unsigned int aux_rd_interval_us = 400;

	switch (dpcd_aux_read_interval) {
	case 0x01:
		aux_rd_interval_us = 4000;
		break;
	case 0x02:
		aux_rd_interval_us = 8000;
		break;
	case 0x03:
		aux_rd_interval_us = 12000;
		break;
	case 0x04:
		aux_rd_interval_us = 16000;
		break;
	case 0x05:
		aux_rd_interval_us = 32000;
		break;
	case 0x06:
		aux_rd_interval_us = 64000;
		break;
	default:
		break;
	}

	return aux_rd_interval_us;
}

enum link_training_result dp_get_cr_failure(enum dc_lane_count ln_count,
					union lane_status *dpcd_lane_status)
{
	enum link_training_result result = LINK_TRAINING_SUCCESS;

	if (ln_count >= LANE_COUNT_ONE && !dpcd_lane_status[0].bits.CR_DONE_0)
		result = LINK_TRAINING_CR_FAIL_LANE0;
	else if (ln_count >= LANE_COUNT_TWO && !dpcd_lane_status[1].bits.CR_DONE_0)
		result = LINK_TRAINING_CR_FAIL_LANE1;
	else if (ln_count >= LANE_COUNT_FOUR && !dpcd_lane_status[2].bits.CR_DONE_0)
		result = LINK_TRAINING_CR_FAIL_LANE23;
	else if (ln_count >= LANE_COUNT_FOUR && !dpcd_lane_status[3].bits.CR_DONE_0)
		result = LINK_TRAINING_CR_FAIL_LANE23;
	return result;
}

bool is_repeater(const struct link_training_settings *lt_settings, uint32_t offset)
{
	return (lt_settings->lttpr_mode == LTTPR_MODE_NON_TRANSPARENT) && (offset != 0);
}

bool dp_is_max_vs_reached(
	const struct link_training_settings *lt_settings)
{
	uint32_t lane;
	for (lane = 0; lane <
		(uint32_t)(lt_settings->link_settings.lane_count);
		lane++) {
		if (lt_settings->dpcd_lane_settings[lane].bits.VOLTAGE_SWING_SET
			== VOLTAGE_SWING_MAX_LEVEL)
			return true;
	}
	return false;

}

bool dp_is_cr_done(enum dc_lane_count ln_count,
	union lane_status *dpcd_lane_status)
{
	bool done = true;
	uint32_t lane;
	/*LANEx_CR_DONE bits All 1's?*/
	for (lane = 0; lane < (uint32_t)(ln_count); lane++) {
		if (!dpcd_lane_status[lane].bits.CR_DONE_0)
			done = false;
	}
	return done;

}

bool dp_is_ch_eq_done(enum dc_lane_count ln_count,
		union lane_status *dpcd_lane_status)
{
	bool done = true;
	uint32_t lane;
	for (lane = 0; lane < (uint32_t)(ln_count); lane++)
		if (!dpcd_lane_status[lane].bits.CHANNEL_EQ_DONE_0)
			done = false;
	return done;
}

bool dp_is_symbol_locked(enum dc_lane_count ln_count,
		union lane_status *dpcd_lane_status)
{
	bool locked = true;
	uint32_t lane;
	for (lane = 0; lane < (uint32_t)(ln_count); lane++)
		if (!dpcd_lane_status[lane].bits.SYMBOL_LOCKED_0)
			locked = false;
	return locked;
}

bool dp_is_interlane_aligned(union lane_align_status_updated align_status)
{
	return align_status.bits.INTERLANE_ALIGN_DONE == 1;
}

bool dp_check_interlane_aligned(union lane_align_status_updated align_status,
		struct dc_link *link,
		uint8_t retries)
{
	/* Take into consideration corner case for DP 1.4a LL Compliance CTS as USB4
	 * has to share encoders unlike DP and USBC
	 */
	return (dp_is_interlane_aligned(align_status) ||
			(link->skip_fallback_on_link_loss && retries));
}

uint32_t dp_get_eq_aux_rd_interval(
		const struct dc_link *link,
		const struct link_training_settings *lt_settings,
		uint32_t offset,
		uint8_t retries)
{
	if (link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA) {
		if (offset == 0 && retries == 1 && lt_settings->lttpr_mode == LTTPR_MODE_NON_TRANSPARENT)
			return max(lt_settings->eq_pattern_time, (uint32_t) DPIA_CLK_SYNC_DELAY);
		else
			return dpia_get_eq_aux_rd_interval(link, lt_settings, offset);
	} else if (is_repeater(lt_settings, offset))
		return dp_translate_training_aux_read_interval(
				link->dpcd_caps.lttpr_caps.aux_rd_interval[offset - 1]);
	else
		return lt_settings->eq_pattern_time;
}

bool dp_check_dpcd_reqeust_status(const struct dc_link *link,
		enum dc_status status)
{
	return (status != DC_OK && link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA);
}

enum link_training_result dp_check_link_loss_status(
	struct dc_link *link,
	const struct link_training_settings *link_training_setting)
{
	enum link_training_result status = LINK_TRAINING_SUCCESS;
	union lane_status lane_status;
	union lane_align_status_updated dpcd_lane_status_updated;
	uint8_t dpcd_buf[6] = {0};
	uint32_t lane;

	core_link_read_dpcd(
			link,
			DP_SINK_COUNT,
			(uint8_t *)(dpcd_buf),
			sizeof(dpcd_buf));

	/*parse lane status*/
	for (lane = 0; lane < link->cur_link_settings.lane_count; lane++) {
		/*
		 * check lanes status
		 */
		lane_status.raw = dp_get_nibble_at_index(&dpcd_buf[2], lane);
		dpcd_lane_status_updated.raw = dpcd_buf[4];

		if (!lane_status.bits.CHANNEL_EQ_DONE_0 ||
			!lane_status.bits.CR_DONE_0 ||
			!lane_status.bits.SYMBOL_LOCKED_0 ||
			!dp_is_interlane_aligned(dpcd_lane_status_updated)) {
			/* if one of the channel equalization, clock
			 * recovery or symbol lock is dropped
			 * consider it as (link has been
			 * dropped) dp sink status has changed
			 */
			status = LINK_TRAINING_LINK_LOSS;
			break;
		}
	}

	return status;
}

enum dc_status dp_get_lane_status_and_lane_adjust(
	struct dc_link *link,
	const struct link_training_settings *link_training_setting,
	union lane_status ln_status[LANE_COUNT_DP_MAX],
	union lane_align_status_updated *ln_align,
	union lane_adjust ln_adjust[LANE_COUNT_DP_MAX],
	uint32_t offset)
{
	unsigned int lane01_status_address = DP_LANE0_1_STATUS;
	uint8_t lane_adjust_offset = 4;
	unsigned int lane01_adjust_address;
	uint8_t dpcd_buf[6] = {0};
	uint32_t lane;
	enum dc_status status;

	if (is_repeater(link_training_setting, offset)) {
		lane01_status_address =
				DP_LANE0_1_STATUS_PHY_REPEATER1 +
				((DP_REPEATER_CONFIGURATION_AND_STATUS_SIZE) * (offset - 1));
		lane_adjust_offset = 3;
	}

	status = core_link_read_dpcd(
		link,
		lane01_status_address,
		(uint8_t *)(dpcd_buf),
		sizeof(dpcd_buf));

	if (status != DC_OK) {
		DC_LOG_HW_LINK_TRAINING("%s:\n Failed to read from address 0x%X,"
			" keep current lane status and lane adjust unchanged",
			__func__,
			lane01_status_address);
		return status;
	}

	if (link_is_imac5k_secondary_windows_route(link))
		DC_LOG_WARNING("IMAC5K: secondary 0x3113 lane-status-read link=%u offset=%u addr=0x%x status=%d raw202=0x%02x raw203=0x%02x align=0x%02x adjust0_1=0x%02x adjust2_3=0x%02x req_rate=%d req_lanes=%d cur_rate=%d cur_lanes=%d stream_state=%d\n",
			       link->link_index, offset, lane01_status_address,
			       status, dpcd_buf[0], dpcd_buf[1],
			       dpcd_buf[2], dpcd_buf[lane_adjust_offset],
			       dpcd_buf[lane_adjust_offset + 1],
			       link_training_setting ?
				       link_training_setting->link_settings.link_rate : 0,
			       link_training_setting ?
				       link_training_setting->link_settings.lane_count : 0,
			       link->cur_link_settings.link_rate,
			       link->cur_link_settings.lane_count,
			       link->imac5k_stream_enable_state);

	for (lane = 0; lane <
		(uint32_t)(link_training_setting->link_settings.lane_count);
		lane++) {

		ln_status[lane].raw =
			dp_get_nibble_at_index(&dpcd_buf[0], lane);
		ln_adjust[lane].raw =
			dp_get_nibble_at_index(&dpcd_buf[lane_adjust_offset], lane);
	}

	ln_align->raw = dpcd_buf[2];

	if (link_is_imac5k_secondary_windows_route(link) &&
	    link_training_setting &&
	    dp_is_cr_done(link_training_setting->link_settings.lane_count,
			  ln_status) &&
	    dp_is_ch_eq_done(link_training_setting->link_settings.lane_count,
			     ln_status) &&
	    dp_is_symbol_locked(link_training_setting->link_settings.lane_count,
				ln_status) &&
	    dp_is_interlane_aligned(*ln_align)) {
		union lane_count_set lane_count_set = {0};
		uint8_t dpcd_005 = 0;
		uint8_t dpcd_080 = 0;
		uint8_t dpcd_600 = 0;
		uint8_t branch_500_508[9] = {0};
		enum dc_status status_005;
		enum dc_status status_080;
		enum dc_status status_500;
		enum dc_status status_600;
		bool first_preserve =
			!link->imac5k_trained_link_preserved;

		lane_count_set.bits.LANE_COUNT_SET =
			link_training_setting->link_settings.lane_count;
		lane_count_set.bits.ENHANCED_FRAMING =
			link_training_setting->enhanced_framing;
		lane_count_set.bits.POST_LT_ADJ_REQ_GRANTED = 0;

		link->imac5k_trained_link_preserved = true;
		link->imac5k_skip_d3_after_trained = true;
		link->imac5k_stream_state_dpcd_valid = true;
		link->imac5k_stream_state_dpcd_settings =
			link_training_setting->link_settings;
		link->imac5k_stream_state_dpcd_003 =
			link_training_setting->link_settings.link_spread ==
			LINK_SPREAD_05_DOWNSPREAD_30KHZ ?
			DP_MAX_DOWNSPREAD_0_5 : 0;
		link->imac5k_stream_state_dpcd_100 =
			get_dpcd_link_rate(
				&link_training_setting->link_settings);
		link->imac5k_stream_state_dpcd_101 = lane_count_set.raw;
		link->imac5k_stream_state_dpcd_202 = dpcd_buf[0];
		link->imac5k_stream_state_dpcd_203 = dpcd_buf[1];

		status_005 = core_link_read_dpcd(link,
						 DP_DOWNSTREAMPORT_PRESENT,
						 &dpcd_005,
						 sizeof(dpcd_005));
		status_080 = core_link_read_dpcd(link, DP_DOWNSTREAM_PORT_0,
						 &dpcd_080,
						 sizeof(dpcd_080));
		status_500 = core_link_read_dpcd(link, DP_BRANCH_OUI,
						 branch_500_508,
						 sizeof(branch_500_508));
		status_600 = core_link_read_dpcd(link, DP_SET_POWER,
						 &dpcd_600,
						 sizeof(dpcd_600));

		if (status_005 == DC_OK)
			link->imac5k_trained_link_dpcd_005 = dpcd_005;
		if (status_080 == DC_OK)
			link->imac5k_trained_link_dpcd_080 = dpcd_080;
		if (status_600 == DC_OK)
			link->imac5k_trained_link_dpcd_600 = dpcd_600;
		if (status_500 == DC_OK)
			memcpy(link->imac5k_trained_link_branch_500_508,
			       branch_500_508,
			       sizeof(link->imac5k_trained_link_branch_500_508));

		DC_LOG_WARNING("IMAC5K: secondary 0x3113 trained-link-preserve %s link=%u raw202=0x%02x raw203=0x%02x align=0x%02x req_rate=%d req_lanes=%d dpcd100=0x%02x dpcd101=0x%02x dpcd003=0x%02x dpcd005=0x%02x status005=%d dpcd080=0x%02x status080=%d branch500_508=%02x %02x %02x %02x %02x %02x %02x %02x %02x status500=%d dpcd600=0x%02x status600=%d skip_d3=1 stream_state=%d evidence=0x%x\n",
			       first_preserve ? "armed" : "refresh",
			       link->link_index, dpcd_buf[0], dpcd_buf[1],
			       dpcd_buf[2],
			       link_training_setting->link_settings.link_rate,
			       link_training_setting->link_settings.lane_count,
			       link->imac5k_stream_state_dpcd_100,
			       link->imac5k_stream_state_dpcd_101,
			       link->imac5k_stream_state_dpcd_003,
			       dpcd_005, status_005, dpcd_080, status_080,
			       branch_500_508[0], branch_500_508[1],
			       branch_500_508[2], branch_500_508[3],
			       branch_500_508[4], branch_500_508[5],
			       branch_500_508[6], branch_500_508[7],
			       branch_500_508[8], status_500, dpcd_600,
			       status_600, link->imac5k_stream_enable_state,
			       link->imac5k_cached_link_aux_evidence);
	}

	if (is_repeater(link_training_setting, offset)) {
		DC_LOG_HW_LINK_TRAINING("%s:\n LTTPR Repeater ID: %d\n"
				" 0x%X Lane01Status = %x\n 0x%X Lane23Status = %x\n ",
			__func__,
			offset,
			lane01_status_address, dpcd_buf[0],
			lane01_status_address + 1, dpcd_buf[1]);

		lane01_adjust_address = DP_ADJUST_REQUEST_LANE0_1_PHY_REPEATER1 +
				((DP_REPEATER_CONFIGURATION_AND_STATUS_SIZE) * (offset - 1));

		DC_LOG_HW_LINK_TRAINING("%s:\n LTTPR Repeater ID: %d\n"
				" 0x%X Lane01AdjustRequest = %x\n 0x%X Lane23AdjustRequest = %x\n",
					__func__,
					offset,
					lane01_adjust_address,
					dpcd_buf[lane_adjust_offset],
					lane01_adjust_address + 1,
					dpcd_buf[lane_adjust_offset + 1]);
	} else {
		DC_LOG_HW_LINK_TRAINING("%s:\n 0x%X Lane01Status = %x\n 0x%X Lane23Status = %x\n ",
			__func__,
			lane01_status_address, dpcd_buf[0],
			lane01_status_address + 1, dpcd_buf[1]);

		lane01_adjust_address = DP_ADJUST_REQUEST_LANE0_1;

		DC_LOG_HW_LINK_TRAINING("%s:\n 0x%X Lane01AdjustRequest = %x\n 0x%X Lane23AdjustRequest = %x\n",
			__func__,
			lane01_adjust_address,
			dpcd_buf[lane_adjust_offset],
			lane01_adjust_address + 1,
			dpcd_buf[lane_adjust_offset + 1]);
	}

	return status;
}

static void override_lane_settings(const struct link_training_settings *lt_settings,
		struct dc_lane_settings lane_settings[LANE_COUNT_DP_MAX])
{
	uint32_t lane;

	if (lt_settings->voltage_swing == NULL &&
			lt_settings->pre_emphasis == NULL &&
			lt_settings->ffe_preset == NULL &&
			lt_settings->post_cursor2 == NULL)

		return;

	for (lane = 0; lane < LANE_COUNT_DP_MAX; lane++) {
		if (lt_settings->voltage_swing)
			lane_settings[lane].VOLTAGE_SWING = *lt_settings->voltage_swing;
		if (lt_settings->pre_emphasis)
			lane_settings[lane].PRE_EMPHASIS = *lt_settings->pre_emphasis;
		if (lt_settings->post_cursor2)
			lane_settings[lane].POST_CURSOR2 = *lt_settings->post_cursor2;
		if (lt_settings->ffe_preset)
			lane_settings[lane].FFE_PRESET = *lt_settings->ffe_preset;
	}
}

void dp_get_lttpr_mode_override(struct dc_link *link, enum lttpr_mode *override)
{
	if (!dp_is_lttpr_present(link))
		return;

	if (link->dc->debug.lttpr_mode_override == LTTPR_MODE_TRANSPARENT) {
		*override = LTTPR_MODE_TRANSPARENT;
	} else if (link->dc->debug.lttpr_mode_override == LTTPR_MODE_NON_TRANSPARENT) {
		*override = LTTPR_MODE_NON_TRANSPARENT;
	} else if (link->dc->debug.lttpr_mode_override == LTTPR_MODE_NON_LTTPR) {
		*override = LTTPR_MODE_NON_LTTPR;
	}
	DC_LOG_DC("lttpr_mode_override chose LTTPR_MODE = %d\n", (uint8_t)(*override));
}

void override_training_settings(
		struct dc_link *link,
		const struct dc_link_training_overrides *overrides,
		struct link_training_settings *lt_settings)
{
	uint32_t lane;

	/* Override link spread */
	if (!link->dp_ss_off && overrides->downspread != NULL)
		lt_settings->link_settings.link_spread = *overrides->downspread ?
				LINK_SPREAD_05_DOWNSPREAD_30KHZ
				: LINK_SPREAD_DISABLED;

	/* Override lane settings */
	if (overrides->voltage_swing != NULL)
		lt_settings->voltage_swing = overrides->voltage_swing;
	if (overrides->pre_emphasis != NULL)
		lt_settings->pre_emphasis = overrides->pre_emphasis;
	if (overrides->post_cursor2 != NULL)
		lt_settings->post_cursor2 = overrides->post_cursor2;
	if (link->wa_flags.force_dp_ffe_preset && !dp_is_lttpr_present(link))
		lt_settings->ffe_preset = &link->forced_dp_ffe_preset;
	if (overrides->ffe_preset != NULL)
		lt_settings->ffe_preset = overrides->ffe_preset;
	/* Override HW lane settings with BIOS forced values if present */
	if ((link->chip_caps & AMD_EXT_DISPLAY_PATH_CAPS__DP_FIXED_VS_EN) &&
			lt_settings->lttpr_mode == LTTPR_MODE_TRANSPARENT) {
		lt_settings->voltage_swing = &link->bios_forced_drive_settings.VOLTAGE_SWING;
		lt_settings->pre_emphasis = &link->bios_forced_drive_settings.PRE_EMPHASIS;
		lt_settings->always_match_dpcd_with_hw_lane_settings = false;
	}
	for (lane = 0; lane < LANE_COUNT_DP_MAX; lane++) {
		lt_settings->hw_lane_settings[lane].VOLTAGE_SWING =
			lt_settings->voltage_swing != NULL ?
			*lt_settings->voltage_swing :
			VOLTAGE_SWING_LEVEL0;
		lt_settings->hw_lane_settings[lane].PRE_EMPHASIS =
			lt_settings->pre_emphasis != NULL ?
			*lt_settings->pre_emphasis
			: PRE_EMPHASIS_DISABLED;
		lt_settings->hw_lane_settings[lane].POST_CURSOR2 =
			lt_settings->post_cursor2 != NULL ?
			*lt_settings->post_cursor2
			: POST_CURSOR2_DISABLED;
	}

	if (lt_settings->always_match_dpcd_with_hw_lane_settings)
		dp_hw_to_dpcd_lane_settings(lt_settings,
				lt_settings->hw_lane_settings, lt_settings->dpcd_lane_settings);

	/* Override training timings */
	if (overrides->cr_pattern_time != NULL)
		lt_settings->cr_pattern_time = *overrides->cr_pattern_time;
	if (overrides->eq_pattern_time != NULL)
		lt_settings->eq_pattern_time = *overrides->eq_pattern_time;
	if (overrides->pattern_for_cr != NULL)
		lt_settings->pattern_for_cr = *overrides->pattern_for_cr;
	if (overrides->pattern_for_eq != NULL)
		lt_settings->pattern_for_eq = *overrides->pattern_for_eq;
	if (overrides->enhanced_framing != NULL)
		lt_settings->enhanced_framing = *overrides->enhanced_framing;
	if (link->preferred_training_settings.fec_enable != NULL)
		lt_settings->should_set_fec_ready = *link->preferred_training_settings.fec_enable;

	/* Check DP tunnel LTTPR mode debug option. */
	if (link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA && link->dc->debug.dpia_debug.bits.force_non_lttpr)
		lt_settings->lttpr_mode = LTTPR_MODE_NON_LTTPR;

	dp_get_lttpr_mode_override(link, &lt_settings->lttpr_mode);
}

enum dc_dp_training_pattern decide_cr_training_pattern(
		const struct dc_link_settings *link_settings)
{
	switch (link_dp_get_encoding_format(link_settings)) {
	case DP_8b_10b_ENCODING:
	default:
		return DP_TRAINING_PATTERN_SEQUENCE_1;
	case DP_128b_132b_ENCODING:
		return DP_128b_132b_TPS1;
	}
}

enum dc_dp_training_pattern decide_eq_training_pattern(struct dc_link *link,
		const struct link_resource *link_res,
		const struct dc_link_settings *link_settings)
{
	struct link_encoder *link_enc = link_res->dio_link_enc;
	struct encoder_feature_support *enc_caps;
	struct dpcd_caps *rx_caps = &link->dpcd_caps;
	enum dc_dp_training_pattern pattern = DP_TRAINING_PATTERN_SEQUENCE_2;

	switch (link_dp_get_encoding_format(link_settings)) {
	case DP_8b_10b_ENCODING:
		if (!link->dc->config.unify_link_enc_assignment)
			link_enc = link_enc_cfg_get_link_enc(link);

		if (!link_enc)
			break;

		enc_caps = &link_enc->features;
		if (enc_caps->flags.bits.IS_TPS4_CAPABLE &&
				rx_caps->max_down_spread.bits.TPS4_SUPPORTED)
			pattern = DP_TRAINING_PATTERN_SEQUENCE_4;
		else if (enc_caps->flags.bits.IS_TPS3_CAPABLE &&
				rx_caps->max_ln_count.bits.TPS3_SUPPORTED)
			pattern = DP_TRAINING_PATTERN_SEQUENCE_3;
		else
			pattern = DP_TRAINING_PATTERN_SEQUENCE_2;
		break;
	case DP_128b_132b_ENCODING:
		pattern = DP_128b_132b_TPS2;
		break;
	default:
		pattern = DP_TRAINING_PATTERN_SEQUENCE_2;
		break;
	}
	return pattern;
}

enum lttpr_mode dp_decide_lttpr_mode(struct dc_link *link,
		struct dc_link_settings *link_setting)
{
	enum dp_link_encoding encoding = link_dp_get_encoding_format(link_setting);

	if (encoding == DP_8b_10b_ENCODING)
		return dp_decide_8b_10b_lttpr_mode(link);
	else if (encoding == DP_128b_132b_ENCODING)
		return dp_decide_128b_132b_lttpr_mode(link);

	ASSERT(0);
	return LTTPR_MODE_NON_LTTPR;
}

void dp_decide_lane_settings(
		const struct link_training_settings *lt_settings,
		const union lane_adjust ln_adjust[LANE_COUNT_DP_MAX],
		struct dc_lane_settings hw_lane_settings[LANE_COUNT_DP_MAX],
		union dpcd_training_lane *dpcd_lane_settings)
{
	uint32_t lane;

	for (lane = 0; lane < LANE_COUNT_DP_MAX; lane++) {
		if (link_dp_get_encoding_format(&lt_settings->link_settings) ==
				DP_8b_10b_ENCODING) {
			hw_lane_settings[lane].VOLTAGE_SWING =
					(enum dc_voltage_swing)(ln_adjust[lane].bits.
							VOLTAGE_SWING_LANE);
			hw_lane_settings[lane].PRE_EMPHASIS =
					(enum dc_pre_emphasis)(ln_adjust[lane].bits.
							PRE_EMPHASIS_LANE);
		} else if (link_dp_get_encoding_format(&lt_settings->link_settings) ==
				DP_128b_132b_ENCODING) {
			hw_lane_settings[lane].FFE_PRESET.raw =
					ln_adjust[lane].tx_ffe.PRESET_VALUE;
		}
	}
	dp_hw_to_dpcd_lane_settings(lt_settings, hw_lane_settings, dpcd_lane_settings);

	if (lt_settings->disallow_per_lane_settings) {
		/* we find the maximum of the requested settings across all lanes*/
		/* and set this maximum for all lanes*/
		maximize_lane_settings(lt_settings, hw_lane_settings);
		override_lane_settings(lt_settings, hw_lane_settings);

		if (lt_settings->always_match_dpcd_with_hw_lane_settings)
			dp_hw_to_dpcd_lane_settings(lt_settings, hw_lane_settings, dpcd_lane_settings);
	}

}

void dp_decide_training_settings(
		struct dc_link *link,
		const struct link_resource *link_res,
		const struct dc_link_settings *link_settings,
		struct link_training_settings *lt_settings)
{
	if (link_dp_get_encoding_format(link_settings) == DP_8b_10b_ENCODING)
		decide_8b_10b_training_settings(link, link_res, link_settings, lt_settings);
	else if (link_dp_get_encoding_format(link_settings) == DP_128b_132b_ENCODING)
		decide_128b_132b_training_settings(link, link_res, link_settings, lt_settings);
}


enum dc_status configure_lttpr_mode_transparent(struct dc_link *link)
{
	uint8_t repeater_mode = DP_PHY_REPEATER_MODE_TRANSPARENT;

	DC_LOG_HW_LINK_TRAINING("%s\n Set LTTPR to Transparent Mode\n", __func__);
	return core_link_write_dpcd(link,
			DP_PHY_REPEATER_MODE,
			(uint8_t *)&repeater_mode,
			sizeof(repeater_mode));
}

static enum dc_status configure_lttpr_mode_non_transparent(
		struct dc_link *link,
		const struct link_training_settings *lt_settings)
{
	/* aux timeout is already set to extended */
	/* RESET/SET lttpr mode to enable non transparent mode */
	uint8_t repeater_cnt;
	uint32_t aux_interval_address;
	uint8_t repeater_id;
	enum dc_status result = DC_ERROR_UNEXPECTED;
	uint8_t repeater_mode = DP_PHY_REPEATER_MODE_TRANSPARENT;
	const struct dc *dc = link->dc;

	enum dp_link_encoding encoding = dc->link_srv->dp_get_encoding_format(&lt_settings->link_settings);

	if (encoding == DP_8b_10b_ENCODING) {
		DC_LOG_HW_LINK_TRAINING("%s\n Set LTTPR to Transparent Mode\n", __func__);
		result = core_link_write_dpcd(link,
				DP_PHY_REPEATER_MODE,
				(uint8_t *)&repeater_mode,
				sizeof(repeater_mode));

	}

	if (result == DC_OK) {
		link->dpcd_caps.lttpr_caps.mode = repeater_mode;
	}

	if (lt_settings->lttpr_mode == LTTPR_MODE_NON_TRANSPARENT) {

		DC_LOG_HW_LINK_TRAINING("%s\n Set LTTPR to Non Transparent Mode\n", __func__);

		repeater_mode = DP_PHY_REPEATER_MODE_NON_TRANSPARENT;
		result = core_link_write_dpcd(link,
				DP_PHY_REPEATER_MODE,
				(uint8_t *)&repeater_mode,
				sizeof(repeater_mode));

		if (result == DC_OK) {
			link->dpcd_caps.lttpr_caps.mode = repeater_mode;
		}

		if (encoding == DP_8b_10b_ENCODING) {
			repeater_cnt = dp_parse_lttpr_repeater_count(link->dpcd_caps.lttpr_caps.phy_repeater_cnt);

			/* Driver does not need to train the first hop. Skip DPCD read and clear
			 * AUX_RD_INTERVAL for DPTX-to-DPIA hop.
			 */
			if (link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA && repeater_cnt > 0 && repeater_cnt < MAX_REPEATER_CNT)
				link->dpcd_caps.lttpr_caps.aux_rd_interval[--repeater_cnt] = 0;

			for (repeater_id = repeater_cnt; repeater_id > 0 && repeater_id < MAX_REPEATER_CNT; repeater_id--) {
				aux_interval_address = DP_TRAINING_AUX_RD_INTERVAL_PHY_REPEATER1 +
						((DP_REPEATER_CONFIGURATION_AND_STATUS_SIZE) * (repeater_id - 1));
				core_link_read_dpcd(
						link,
						aux_interval_address,
						(uint8_t *)&link->dpcd_caps.lttpr_caps.aux_rd_interval[repeater_id - 1],
						sizeof(link->dpcd_caps.lttpr_caps.aux_rd_interval[repeater_id - 1]));
				link->dpcd_caps.lttpr_caps.aux_rd_interval[repeater_id - 1] &= 0x7F;
			}
		}
	}

	return result;
}

enum dc_status dpcd_configure_lttpr_mode(struct dc_link *link, struct link_training_settings *lt_settings)
{
	enum dc_status status = DC_OK;

	if (lt_settings->lttpr_mode == LTTPR_MODE_TRANSPARENT)
		status = configure_lttpr_mode_transparent(link);

	else if (lt_settings->lttpr_mode == LTTPR_MODE_NON_TRANSPARENT)
		status = configure_lttpr_mode_non_transparent(link, lt_settings);

	return status;
}

void repeater_training_done(struct dc_link *link, uint32_t offset)
{
	union dpcd_training_pattern dpcd_pattern = {0};

	const uint32_t dpcd_base_lt_offset =
			DP_TRAINING_PATTERN_SET_PHY_REPEATER1 +
				((DP_REPEATER_CONFIGURATION_AND_STATUS_SIZE) * (offset - 1));
	/* Set training not in progress*/
	dpcd_pattern.v1_4.TRAINING_PATTERN_SET = DPCD_TRAINING_PATTERN_VIDEOIDLE;

	core_link_write_dpcd(
		link,
		dpcd_base_lt_offset,
		&dpcd_pattern.raw,
		1);

	DC_LOG_HW_LINK_TRAINING("%s\n LTTPR Id: %d 0x%X pattern = %x\n",
		__func__,
		offset,
		dpcd_base_lt_offset,
		dpcd_pattern.v1_4.TRAINING_PATTERN_SET);
}

static enum link_training_result dpcd_exit_training_mode(struct dc_link *link, enum dp_link_encoding encoding)
{
	enum dc_status status;
	uint8_t sink_status = 0;
	uint32_t i;
	uint8_t lttpr_count = dp_parse_lttpr_repeater_count(link->dpcd_caps.lttpr_caps.phy_repeater_cnt);
	uint32_t intra_hop_disable_time_ms = (lttpr_count > 0 ? lttpr_count * 300 : 10);

	// Each hop could theoretically take over 256ms (max 128b/132b AUX RD INTERVAL)
	// To be safe, allow 300ms per LTTPR and 10ms for no LTTPR case

	/* clear training pattern set */
	status = dpcd_set_training_pattern(link, DP_TRAINING_PATTERN_VIDEOIDLE);

	if (dp_check_dpcd_reqeust_status(link, status))
		return LINK_TRAINING_ABORT;

	if (encoding == DP_128b_132b_ENCODING) {
		/* poll for intra-hop disable */
		for (i = 0; i < intra_hop_disable_time_ms; i++) {
			if ((core_link_read_dpcd(link, DP_SINK_STATUS, &sink_status, 1) == DC_OK) &&
					(sink_status & DP_INTRA_HOP_AUX_REPLY_INDICATION) == 0)
				break;
			fsleep(1000);
		}
	}

	return LINK_TRAINING_SUCCESS;
}

enum dc_status dpcd_configure_channel_coding(struct dc_link *link,
		struct link_training_settings *lt_settings)
{
	enum dp_link_encoding encoding =
			link_dp_get_encoding_format(
					&lt_settings->link_settings);
	enum dc_status status;

	status = core_link_write_dpcd(
			link,
			DP_MAIN_LINK_CHANNEL_CODING_SET,
			(uint8_t *) &encoding,
			1);
	DC_LOG_HW_LINK_TRAINING("%s:\n 0x%X MAIN_LINK_CHANNEL_CODING_SET = %x\n",
					__func__,
					DP_MAIN_LINK_CHANNEL_CODING_SET,
					encoding);

	return status;
}

enum dc_status dpcd_set_training_pattern(
	struct dc_link *link,
	enum dc_dp_training_pattern training_pattern)
{
	enum dc_status status;
	union dpcd_training_pattern dpcd_pattern = {0};

	dpcd_pattern.v1_4.TRAINING_PATTERN_SET =
			dp_training_pattern_to_dpcd_training_pattern(
					link, training_pattern);

	status = core_link_write_dpcd(
		link,
		DP_TRAINING_PATTERN_SET,
		&dpcd_pattern.raw,
		1);

	DC_LOG_HW_LINK_TRAINING("%s\n %x pattern = %x\n",
		__func__,
		DP_TRAINING_PATTERN_SET,
		dpcd_pattern.v1_4.TRAINING_PATTERN_SET);
	imac5k_secondary_log_training_dpcd_window(link, NULL,
						 training_pattern, DPRX,
						 DP_TRAINING_PATTERN_SET,
						 dpcd_pattern.raw, status,
						 "single-pattern-write");

	return status;
}

enum dc_status dpcd_set_link_settings(
	struct dc_link *link,
	const struct link_training_settings *lt_settings)
{
	uint8_t rate;
	enum dc_status status;

	union down_spread_ctrl downspread = {0};
	union lane_count_set lane_count_set = {0};

	downspread.raw = (uint8_t)
	(lt_settings->link_settings.link_spread);

	lane_count_set.bits.LANE_COUNT_SET =
	lt_settings->link_settings.lane_count;

	lane_count_set.bits.ENHANCED_FRAMING = lt_settings->enhanced_framing;
	lane_count_set.bits.POST_LT_ADJ_REQ_GRANTED = 0;


	if (link->ep_type == DISPLAY_ENDPOINT_PHY &&
			lt_settings->pattern_for_eq < DP_TRAINING_PATTERN_SEQUENCE_4) {
		lane_count_set.bits.POST_LT_ADJ_REQ_GRANTED =
				link->dpcd_caps.max_ln_count.bits.POST_LT_ADJ_REQ_SUPPORTED;
	}

	if (imac5k_secondary_should_use_windows_link_settings_order(
		    link, lt_settings))
		return imac5k_secondary_dpcd_set_link_settings_windows_order(
			link, lt_settings, &lane_count_set, &downspread);

	if (link_is_imac5k_secondary_windows_candidate(link))
		imac5k_secondary_log_windows_order_skip(link, lt_settings,
							&lane_count_set,
							&downspread);

	status = core_link_write_dpcd(link, DP_DOWNSPREAD_CTRL,
		&downspread.raw, sizeof(downspread));
	if (status != DC_OK)
		DC_LOG_ERROR("%s:%d: core_link_write_dpcd (DP_DOWNSPREAD_CTRL) failed\n", __func__, __LINE__);

	status = core_link_write_dpcd(link, DP_LANE_COUNT_SET,
		&lane_count_set.raw, 1);
	if (status != DC_OK)
		DC_LOG_ERROR("%s:%d: core_link_write_dpcd (DP_LANE_COUNT_SET) failed\n", __func__, __LINE__);

	if (link->dpcd_caps.dpcd_rev.raw >= DPCD_REV_13 &&
			lt_settings->link_settings.use_link_rate_set == true) {
		rate = 0;
		/* WA for some MUX chips that will power down with eDP and lose supported
		 * link rate set for eDP 1.4. Source reads DPCD 0x010 again to ensure
		 * MUX chip gets link rate set back before link training.
		 */
		if (link->connector_signal == SIGNAL_TYPE_EDP) {
			uint8_t supported_link_rates[16] = {0};

			core_link_read_dpcd(link, DP_SUPPORTED_LINK_RATES,
					supported_link_rates, sizeof(supported_link_rates));
		}
		status = core_link_write_dpcd(link, DP_LINK_BW_SET, &rate, 1);
		if (status != DC_OK)
			DC_LOG_ERROR("%s:%d: core_link_write_dpcd (DP_LINK_BW_SET) failed\n", __func__, __LINE__);

		status = core_link_write_dpcd(link, DP_LINK_RATE_SET,
				&lt_settings->link_settings.link_rate_set, 1);
		if (status != DC_OK)
			DC_LOG_ERROR("%s:%d: core_link_write_dpcd (DP_LINK_RATE_SET) failed\n", __func__, __LINE__);
	} else {
		rate = get_dpcd_link_rate(&lt_settings->link_settings);

		status = core_link_write_dpcd(link, DP_LINK_BW_SET, &rate, 1);
		if (status != DC_OK)
			DC_LOG_ERROR("%s:%d: core_link_write_dpcd (DP_LINK_BW_SET) failed\n", __func__, __LINE__);
	}

	if (rate) {
		DC_LOG_HW_LINK_TRAINING("%s\n %x rate = %x\n %x lane = %x framing = %x\n %x spread = %x\n",
			__func__,
			DP_LINK_BW_SET,
			lt_settings->link_settings.link_rate,
			DP_LANE_COUNT_SET,
			lt_settings->link_settings.lane_count,
			lt_settings->enhanced_framing,
			DP_DOWNSPREAD_CTRL,
			lt_settings->link_settings.link_spread);
	} else {
		DC_LOG_HW_LINK_TRAINING("%s\n %x rate set = %x\n %x lane = %x framing = %x\n %x spread = %x\n",
			__func__,
			DP_LINK_RATE_SET,
			lt_settings->link_settings.link_rate_set,
			DP_LANE_COUNT_SET,
			lt_settings->link_settings.lane_count,
			lt_settings->enhanced_framing,
			DP_DOWNSPREAD_CTRL,
			lt_settings->link_settings.link_spread);
	}

	return status;
}

enum dc_status dpcd_set_lane_settings(
	struct dc_link *link,
	const struct link_training_settings *link_training_setting,
	uint32_t offset)
{
	unsigned int lane0_set_address;
	enum dc_status status;
	lane0_set_address = DP_TRAINING_LANE0_SET;

	if (is_repeater(link_training_setting, offset))
		lane0_set_address = DP_TRAINING_LANE0_SET_PHY_REPEATER1 +
		((DP_REPEATER_CONFIGURATION_AND_STATUS_SIZE) * (offset - 1));

	status = core_link_write_dpcd(link,
		lane0_set_address,
		(uint8_t *)(link_training_setting->dpcd_lane_settings),
		link_training_setting->link_settings.lane_count);

	if (is_repeater(link_training_setting, offset)) {
		DC_LOG_HW_LINK_TRAINING("%s\n LTTPR Repeater ID: %d\n"
				" 0x%X VS set = %x  PE set = %x max VS Reached = %x  max PE Reached = %x\n",
			__func__,
			offset,
			lane0_set_address,
			link_training_setting->dpcd_lane_settings[0].bits.VOLTAGE_SWING_SET,
			link_training_setting->dpcd_lane_settings[0].bits.PRE_EMPHASIS_SET,
			link_training_setting->dpcd_lane_settings[0].bits.MAX_SWING_REACHED,
			link_training_setting->dpcd_lane_settings[0].bits.MAX_PRE_EMPHASIS_REACHED);

	} else {
		DC_LOG_HW_LINK_TRAINING("%s\n 0x%X VS set = %x  PE set = %x max VS Reached = %x  max PE Reached = %x\n",
			__func__,
			lane0_set_address,
			link_training_setting->dpcd_lane_settings[0].bits.VOLTAGE_SWING_SET,
			link_training_setting->dpcd_lane_settings[0].bits.PRE_EMPHASIS_SET,
			link_training_setting->dpcd_lane_settings[0].bits.MAX_SWING_REACHED,
			link_training_setting->dpcd_lane_settings[0].bits.MAX_PRE_EMPHASIS_REACHED);
	}

	return status;
}

void dpcd_set_lt_pattern_and_lane_settings(
	struct dc_link *link,
	const struct link_training_settings *lt_settings,
	enum dc_dp_training_pattern pattern,
	uint32_t offset)
{
	uint32_t dpcd_base_lt_offset;
	uint8_t dpcd_lt_buffer[5] = {0};
	union dpcd_training_pattern dpcd_pattern = {0};
	uint32_t size_in_bytes;
	enum dc_status status = DC_OK;
	enum dc_status lane_status = DC_OK;
	bool edp_workaround = false; /* TODO link_prop.INTERNAL */
	dpcd_base_lt_offset = DP_TRAINING_PATTERN_SET;

	if (is_repeater(lt_settings, offset))
		dpcd_base_lt_offset = DP_TRAINING_PATTERN_SET_PHY_REPEATER1 +
			((DP_REPEATER_CONFIGURATION_AND_STATUS_SIZE) * (offset - 1));

	/*****************************************************************
	* DpcdAddress_TrainingPatternSet
	*****************************************************************/
	dpcd_pattern.v1_4.TRAINING_PATTERN_SET =
		dp_training_pattern_to_dpcd_training_pattern(link, pattern);

	dpcd_pattern.v1_4.SCRAMBLING_DISABLE =
		dp_initialize_scrambling_data_symbols(link, pattern);

	dpcd_lt_buffer[DP_TRAINING_PATTERN_SET - DP_TRAINING_PATTERN_SET]
		= dpcd_pattern.raw;

	if (link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA)
		dpia_set_tps_notification(
			link,
			lt_settings,
			dpcd_pattern.v1_4.TRAINING_PATTERN_SET,
			offset);

	if (is_repeater(lt_settings, offset)) {
		DC_LOG_HW_LINK_TRAINING("%s\n LTTPR Repeater ID: %d\n 0x%X pattern = %x\n",
			__func__,
			offset,
			dpcd_base_lt_offset,
			dpcd_pattern.v1_4.TRAINING_PATTERN_SET);
	} else {
		DC_LOG_HW_LINK_TRAINING("%s\n 0x%X pattern = %x\n",
			__func__,
			dpcd_base_lt_offset,
			dpcd_pattern.v1_4.TRAINING_PATTERN_SET);
	}

	/* concatenate everything into one buffer*/
	size_in_bytes = lt_settings->link_settings.lane_count *
			sizeof(lt_settings->dpcd_lane_settings[0]);

	 // 0x00103 - 0x00102
	memmove(
		&dpcd_lt_buffer[DP_TRAINING_LANE0_SET - DP_TRAINING_PATTERN_SET],
		lt_settings->dpcd_lane_settings,
		size_in_bytes);

	if (is_repeater(lt_settings, offset)) {
		if (link_dp_get_encoding_format(&lt_settings->link_settings) ==
				DP_128b_132b_ENCODING)
			DC_LOG_HW_LINK_TRAINING("%s:\n LTTPR Repeater ID: %d\n"
					" 0x%X TX_FFE_PRESET_VALUE = %x\n",
					__func__,
					offset,
					dpcd_base_lt_offset,
					lt_settings->dpcd_lane_settings[0].tx_ffe.PRESET_VALUE);
		else if (link_dp_get_encoding_format(&lt_settings->link_settings) ==
				DP_8b_10b_ENCODING)
		DC_LOG_HW_LINK_TRAINING("%s:\n LTTPR Repeater ID: %d\n"
				" 0x%X VS set = %x PE set = %x max VS Reached = %x  max PE Reached = %x\n",
			__func__,
			offset,
			dpcd_base_lt_offset,
			lt_settings->dpcd_lane_settings[0].bits.VOLTAGE_SWING_SET,
			lt_settings->dpcd_lane_settings[0].bits.PRE_EMPHASIS_SET,
			lt_settings->dpcd_lane_settings[0].bits.MAX_SWING_REACHED,
			lt_settings->dpcd_lane_settings[0].bits.MAX_PRE_EMPHASIS_REACHED);
	} else {
		if (link_dp_get_encoding_format(&lt_settings->link_settings) ==
				DP_128b_132b_ENCODING)
			DC_LOG_HW_LINK_TRAINING("%s:\n 0x%X TX_FFE_PRESET_VALUE = %x\n",
					__func__,
					dpcd_base_lt_offset,
					lt_settings->dpcd_lane_settings[0].tx_ffe.PRESET_VALUE);
		else if (link_dp_get_encoding_format(&lt_settings->link_settings) ==
				DP_8b_10b_ENCODING)
			DC_LOG_HW_LINK_TRAINING("%s:\n 0x%X VS set = %x  PE set = %x max VS Reached = %x  max PE Reached = %x\n",
					__func__,
					dpcd_base_lt_offset,
					lt_settings->dpcd_lane_settings[0].bits.VOLTAGE_SWING_SET,
					lt_settings->dpcd_lane_settings[0].bits.PRE_EMPHASIS_SET,
					lt_settings->dpcd_lane_settings[0].bits.MAX_SWING_REACHED,
					lt_settings->dpcd_lane_settings[0].bits.MAX_PRE_EMPHASIS_REACHED);
	}
	if (edp_workaround) {
		/* for eDP write in 2 parts because the 5-byte burst is
		* causing issues on some eDP panels (EPR#366724)
		*/
		status = core_link_write_dpcd(
			link,
			DP_TRAINING_PATTERN_SET,
			&dpcd_pattern.raw,
			sizeof(dpcd_pattern.raw));

		lane_status = core_link_write_dpcd(
			link,
			DP_TRAINING_LANE0_SET,
			(uint8_t *)(lt_settings->dpcd_lane_settings),
			size_in_bytes);
		if (status == DC_OK)
			status = lane_status;

	} else if (link_dp_get_encoding_format(&lt_settings->link_settings) ==
			DP_128b_132b_ENCODING) {
		status = core_link_write_dpcd(
				link,
				dpcd_base_lt_offset,
				dpcd_lt_buffer,
				sizeof(dpcd_lt_buffer));
	} else {
		/* write it all in (1 + number-of-lanes)-byte burst*/
		status = core_link_write_dpcd(
				link,
				dpcd_base_lt_offset,
				dpcd_lt_buffer,
				size_in_bytes + sizeof(dpcd_pattern.raw));
	}

	imac5k_secondary_log_training_dpcd_window(link, lt_settings, pattern,
						 offset, dpcd_base_lt_offset,
						 dpcd_pattern.raw, status,
						 "pattern-lane-burst");
}

void start_clock_recovery_pattern_early(struct dc_link *link,
		const struct link_resource *link_res,
		struct link_training_settings *lt_settings,
		uint32_t offset)
{
	DC_LOG_HW_LINK_TRAINING("%s\n GPU sends TPS1. Wait 400us.\n",
			__func__);
	dp_set_hw_training_pattern(link, link_res, lt_settings->pattern_for_cr, offset);
	dp_set_hw_lane_settings(link, link_res, lt_settings, offset);
	udelay(400);
}

void dp_set_hw_test_pattern(
	struct dc_link *link,
	const struct link_resource *link_res,
	enum dp_test_pattern test_pattern,
	uint8_t *custom_pattern,
	uint32_t custom_pattern_size)
{
	const struct link_hwss *link_hwss = get_link_hwss(link, link_res);
	struct encoder_set_dp_phy_pattern_param pattern_param = {0};

	pattern_param.dp_phy_pattern = test_pattern;
	pattern_param.custom_pattern = custom_pattern;
	pattern_param.custom_pattern_size = custom_pattern_size;
	pattern_param.dp_panel_mode = dp_get_panel_mode(link);

	if (link_is_imac5k_secondary_windows_route(link))
		DC_LOG_WARNING("IMAC5K: secondary 0x3113 hw-test-pattern link=%u test_pattern=%d panel_mode=%d cur_rate=%d cur_lanes=%d stream_state=%d\n",
			       link->link_index, test_pattern,
			       pattern_param.dp_panel_mode,
			       link->cur_link_settings.link_rate,
			       link->cur_link_settings.lane_count,
			       link->imac5k_stream_enable_state);

	if (link_hwss->ext.set_dp_link_test_pattern)
		link_hwss->ext.set_dp_link_test_pattern(link, link_res, &pattern_param);
}

bool dp_set_hw_training_pattern(
	struct dc_link *link,
	const struct link_resource *link_res,
	enum dc_dp_training_pattern pattern,
	uint32_t offset)
{
	enum dp_test_pattern test_pattern = DP_TEST_PATTERN_UNSUPPORTED;

	switch (pattern) {
	case DP_TRAINING_PATTERN_SEQUENCE_1:
		test_pattern = DP_TEST_PATTERN_TRAINING_PATTERN1;
		break;
	case DP_TRAINING_PATTERN_SEQUENCE_2:
		test_pattern = DP_TEST_PATTERN_TRAINING_PATTERN2;
		break;
	case DP_TRAINING_PATTERN_SEQUENCE_3:
		test_pattern = DP_TEST_PATTERN_TRAINING_PATTERN3;
		break;
	case DP_TRAINING_PATTERN_SEQUENCE_4:
		test_pattern = DP_TEST_PATTERN_TRAINING_PATTERN4;
		break;
	case DP_128b_132b_TPS1:
		test_pattern = DP_TEST_PATTERN_128b_132b_TPS1_TRAINING_MODE;
		break;
	case DP_128b_132b_TPS2:
		test_pattern = DP_TEST_PATTERN_128b_132b_TPS2_TRAINING_MODE;
		break;
	default:
		break;
	}

	dp_set_hw_test_pattern(link, link_res, test_pattern, NULL, 0);

	return true;
}

static bool perform_post_lt_adj_req_sequence(
		struct dc_link *link,
		const struct link_resource *link_res,
		struct link_training_settings *lt_settings)
{
	enum dc_lane_count lane_count =
	lt_settings->link_settings.lane_count;

	uint32_t adj_req_count;
	uint32_t adj_req_timer;
	bool req_drv_setting_changed;
	uint32_t lane;
	union lane_status dpcd_lane_status[LANE_COUNT_DP_MAX] = {0};
	union lane_align_status_updated dpcd_lane_status_updated = {0};
	union lane_adjust dpcd_lane_adjust[LANE_COUNT_DP_MAX] = {0};

	req_drv_setting_changed = false;
	for (adj_req_count = 0; adj_req_count < POST_LT_ADJ_REQ_LIMIT;
	adj_req_count++) {

		req_drv_setting_changed = false;

		for (adj_req_timer = 0;
			adj_req_timer < POST_LT_ADJ_REQ_TIMEOUT;
			adj_req_timer++) {

			dp_get_lane_status_and_lane_adjust(
				link,
				lt_settings,
				dpcd_lane_status,
				&dpcd_lane_status_updated,
				dpcd_lane_adjust,
				DPRX);

			if (dpcd_lane_status_updated.bits.
					POST_LT_ADJ_REQ_IN_PROGRESS == 0)
				return true;

			if (!dp_is_cr_done(lane_count, dpcd_lane_status))
				return false;

			if (!dp_is_ch_eq_done(lane_count, dpcd_lane_status) ||
					!dp_is_symbol_locked(lane_count, dpcd_lane_status) ||
					!dp_is_interlane_aligned(dpcd_lane_status_updated))
				return false;

			for (lane = 0; lane < (uint32_t)(lane_count); lane++) {

				if (lt_settings->
				dpcd_lane_settings[lane].bits.VOLTAGE_SWING_SET !=
				dpcd_lane_adjust[lane].bits.VOLTAGE_SWING_LANE ||
				lt_settings->dpcd_lane_settings[lane].bits.PRE_EMPHASIS_SET !=
				dpcd_lane_adjust[lane].bits.PRE_EMPHASIS_LANE) {

					req_drv_setting_changed = true;
					break;
				}
			}

			if (req_drv_setting_changed) {
				dp_decide_lane_settings(lt_settings, dpcd_lane_adjust,
						lt_settings->hw_lane_settings, lt_settings->dpcd_lane_settings);

				dp_set_drive_settings(link,
						link_res,
						lt_settings);
				break;
			}

			msleep(1);
		}

		if (!req_drv_setting_changed) {
			DC_LOG_WARNING("%s: Post Link Training Adjust Request Timed out\n",
				__func__);

			ASSERT(0);
			return true;
		}
	}
	DC_LOG_WARNING("%s: Post Link Training Adjust Request limit reached\n",
		__func__);

	ASSERT(0);
	return true;

}

static enum link_training_result dp_transition_to_video_idle(
	struct dc_link *link,
	const struct link_resource *link_res,
	struct link_training_settings *lt_settings,
	enum link_training_result status)
{
	union lane_count_set lane_count_set = {0};

	/* 4. mainlink output idle pattern*/
	dp_set_hw_test_pattern(link, link_res, DP_TEST_PATTERN_VIDEO_MODE, NULL, 0);

	/*
	 * 5. post training adjust if required
	 * If the upstream DPTX and downstream DPRX both support TPS4,
	 * TPS4 must be used instead of POST_LT_ADJ_REQ.
	 */
	if (link->dpcd_caps.max_ln_count.bits.POST_LT_ADJ_REQ_SUPPORTED != 1 ||
			lt_settings->pattern_for_eq >= DP_TRAINING_PATTERN_SEQUENCE_4) {
		/* delay 5ms after Main Link output idle pattern and then check
		 * DPCD 0202h.
		 */
		if (link->connector_signal != SIGNAL_TYPE_EDP && status == LINK_TRAINING_SUCCESS) {
			msleep(5);
			if (!link->skip_fallback_on_link_loss)
				status = dp_check_link_loss_status(link, lt_settings);
		}
		return status;
	}

	if (status == LINK_TRAINING_SUCCESS &&
		perform_post_lt_adj_req_sequence(link, link_res, lt_settings) == false)
		status = LINK_TRAINING_LQA_FAIL;

	lane_count_set.bits.LANE_COUNT_SET = lt_settings->link_settings.lane_count;
	lane_count_set.bits.ENHANCED_FRAMING = lt_settings->enhanced_framing;
	lane_count_set.bits.POST_LT_ADJ_REQ_GRANTED = 0;

	core_link_write_dpcd(
		link,
		DP_LANE_COUNT_SET,
		&lane_count_set.raw,
		sizeof(lane_count_set));

	return status;
}

enum link_training_result dp_perform_link_training(
	struct dc_link *link,
	const struct link_resource *link_res,
	const struct dc_link_settings *link_settings,
	bool skip_video_pattern)
{
	enum link_training_result status = LINK_TRAINING_SUCCESS;
	struct link_training_settings lt_settings = {0};
	enum dp_link_encoding encoding =
			link_dp_get_encoding_format(link_settings);

	/* decide training settings */
	dp_decide_training_settings(
			link,
			link_res,
			link_settings,
			&lt_settings);

	override_training_settings(
			link,
			&link->preferred_training_settings,
			&lt_settings);

	/* reset previous training states */
	dpcd_exit_training_mode(link, encoding);

	/* configure link prior to entering training mode */
	dpcd_configure_lttpr_mode(link, &lt_settings);
	if (link_dp_get_encoding_format(link_settings) == DP_8b_10b_ENCODING)
		dp_set_fec_ready(link, link_res, lt_settings.should_set_fec_ready);
	dpcd_configure_channel_coding(link, &lt_settings);

	/* enter training mode:
	 * Per DP specs starting from here, DPTX device shall not issue
	 * Non-LT AUX transactions inside training mode.
	 */
	if (((link->chip_caps & AMD_EXT_DISPLAY_PATH_CAPS__EXT_CHIP_MASK) == AMD_EXT_DISPLAY_PATH_CAPS__DP_FIXED_VS_EN) && encoding == DP_8b_10b_ENCODING)
		status = dp_perform_fixed_vs_pe_training_sequence(link, link_res, &lt_settings);
	else if (encoding == DP_8b_10b_ENCODING)
		status = dp_perform_8b_10b_link_training(link, link_res, &lt_settings);
	else if (encoding == DP_128b_132b_ENCODING)
		status = dp_perform_128b_132b_link_training(link, link_res, &lt_settings);
	else
		ASSERT(0);

	/* exit training mode */
	if ((dpcd_exit_training_mode(link, encoding) != LINK_TRAINING_SUCCESS || status == LINK_TRAINING_ABORT) &&
			link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA)
		dpia_training_abort(link, &lt_settings, 0);

	/* switch to video idle */
	if ((status == LINK_TRAINING_SUCCESS) || !skip_video_pattern)
		status = dp_transition_to_video_idle(link,
				link_res,
				&lt_settings,
				status);

	/* dump debug data */
	dp_log_training_result(link, &lt_settings, status);
	if (status != LINK_TRAINING_SUCCESS)
		link->ctx->dc->debug_data.ltFailCount++;
	return status;
}

bool perform_link_training_with_retries(
	const struct dc_link_settings *link_setting,
	bool skip_video_pattern,
	int attempts,
	struct pipe_ctx *pipe_ctx,
	enum signal_type signal,
	bool do_fallback)
{
	int j;
	uint8_t delay_between_attempts = LINK_TRAINING_RETRY_DELAY;
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	enum dp_panel_mode panel_mode = dp_get_panel_mode(link);
	enum link_training_result status = LINK_TRAINING_CR_FAIL_LANE0;
	struct dc_link_settings cur_link_settings = *link_setting;
	struct dc_link_settings max_link_settings = *link_setting;
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	int fail_count = 0;
	bool is_link_bw_low = false; /* link bandwidth < stream bandwidth */
	bool is_link_bw_min = /* RBR x 1 */
		(cur_link_settings.link_rate <= LINK_RATE_LOW) &&
		(cur_link_settings.lane_count <= LANE_COUNT_ONE);

	dp_trace_commit_lt_init(link);


	if (link_dp_get_encoding_format(&cur_link_settings) == DP_8b_10b_ENCODING)
		/* We need to do this before the link training to ensure the idle
		 * pattern in SST mode will be sent right after the link training
		 */
		link_hwss->setup_stream_encoder(pipe_ctx);

	dp_trace_set_lt_start_timestamp(link, false);
	j = 0;
	while (j < attempts && fail_count < (attempts * 10)) {

		DC_LOG_HW_LINK_TRAINING("%s: Beginning link(%d) training attempt %u of %d @ rate(%d) x lane(%d) @ spread = %x\n",
					__func__, link->link_index, (unsigned int)j + 1, attempts,
				       cur_link_settings.link_rate, cur_link_settings.lane_count,
				       cur_link_settings.link_spread);

		dp_enable_link_phy(
			link,
			&pipe_ctx->link_res,
			signal,
			pipe_ctx->clock_source->id,
			&cur_link_settings);

		if (stream->sink_patches.dppowerup_delay > 0) {
			int delay_dp_power_up_in_ms = stream->sink_patches.dppowerup_delay;

			msleep(delay_dp_power_up_in_ms);
		}

		edp_set_panel_assr(link, pipe_ctx, &panel_mode, true);

		dp_set_panel_mode(link, panel_mode);

		if (link->aux_access_disabled) {
			dp_perform_link_training_skip_aux(link, &pipe_ctx->link_res, &cur_link_settings);
			return true;
		} else {
			if (!link->dc->config.consolidated_dpia_dp_lt && link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA) {
				status = dpia_perform_link_training(
						link,
						&pipe_ctx->link_res,
						&cur_link_settings,
						skip_video_pattern);

				/* Transmit idle pattern once training successful. */
				if (status == LINK_TRAINING_SUCCESS && !is_link_bw_low) {
					dp_set_hw_test_pattern(link, &pipe_ctx->link_res, DP_TEST_PATTERN_VIDEO_MODE, NULL, 0);
					// Update verified link settings to current one
					// Because DPIA LT might fallback to lower link setting.
					if (stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST) {
						link->verified_link_cap.link_rate = link->cur_link_settings.link_rate;
						link->verified_link_cap.lane_count = link->cur_link_settings.lane_count;
						dm_helpers_dp_mst_update_branch_bandwidth(link->ctx, link);
					}
				}
			} else {
				status = dp_perform_link_training(
						link,
						&pipe_ctx->link_res,
						&cur_link_settings,
						skip_video_pattern);
			}

			dp_trace_lt_total_count_increment(link, false);
			dp_trace_lt_result_update(link, status, false);
			dp_trace_set_lt_end_timestamp(link, false);
			if (status == LINK_TRAINING_SUCCESS && !is_link_bw_low) {
				// Update verified link settings to current one
				// Because DPIA LT might fallback to lower link setting.
				if (link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA &&
						stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST) {
					link->verified_link_cap.link_rate = link->cur_link_settings.link_rate;
					link->verified_link_cap.lane_count = link->cur_link_settings.lane_count;
					dm_helpers_dp_mst_update_branch_bandwidth(link->ctx, link);
				}
				return true;
			}
		}

		fail_count++;
		dp_trace_lt_fail_count_update(link, fail_count, false);
		if (link->ep_type == DISPLAY_ENDPOINT_PHY) {
			/* latest link training still fail or link training is aborted
			 * skip delay and keep PHY on
			 */
			if (j == (attempts - 1) || (status == LINK_TRAINING_ABORT))
				break;
		}

		if (link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA &&
				stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST &&
				!link->dc->config.enable_dpia_pre_training) {
			if (j == (attempts - 1))
				do_fallback = true;
			else
				do_fallback = false;
		}

		if (j == (attempts - 1)) {
			DC_LOG_WARNING(
				"%s: Link(%d) training attempt %u of %d failed @ rate(%d) x lane(%d) @ spread = %x : fail reason:(%d)\n",
				__func__, link->link_index, (unsigned int)j + 1, attempts,
				cur_link_settings.link_rate, cur_link_settings.lane_count,
				cur_link_settings.link_spread, status);
		} else {
			DC_LOG_HW_LINK_TRAINING(
				"%s: Link(%d) training attempt %u of %d failed @ rate(%d) x lane(%d) @ spread = %x : fail reason:(%d)\n",
				__func__, link->link_index, (unsigned int)j + 1, attempts,
				cur_link_settings.link_rate, cur_link_settings.lane_count,
				cur_link_settings.link_spread, status);
		}

		dp_disable_link_phy(link, &pipe_ctx->link_res, signal);

		/* Abort link training if failure due to sink being unplugged. */
		if (status == LINK_TRAINING_ABORT) {
			enum dc_connection_type type = dc_connection_none;

			if (link_detect_connection_type(link, &type) && type == dc_connection_none) {
				DC_LOG_HW_LINK_TRAINING("%s: Aborting training because sink unplugged\n", __func__);
				break;
			}
		}

		/* Try to train again at original settings if:
		 * - not falling back between training attempts;
		 * - aborted previous attempt due to reasons other than sink unplug;
		 * - successfully trained but at a link rate lower than that required by stream;
		 * - reached minimum link bandwidth.
		 */
		if (!do_fallback || (status == LINK_TRAINING_ABORT) ||
				(status == LINK_TRAINING_SUCCESS && is_link_bw_low) ||
				is_link_bw_min) {
			j++;
			cur_link_settings = *link_setting;
			delay_between_attempts += LINK_TRAINING_RETRY_DELAY;
			is_link_bw_low = false;
			is_link_bw_min = (cur_link_settings.link_rate <= LINK_RATE_LOW) &&
				(cur_link_settings.lane_count <= LANE_COUNT_ONE);

		} else if (do_fallback) { /* Try training at lower link bandwidth if doing fallback. */
			uint32_t req_bw;
			uint32_t link_bw;
			enum dc_link_encoding_format link_encoding = DC_LINK_ENCODING_UNSPECIFIED;

			decide_fallback_link_setting(link, &max_link_settings,
					&cur_link_settings, status);

			if (link_dp_get_encoding_format(&cur_link_settings) == DP_8b_10b_ENCODING)
				link_encoding = DC_LINK_ENCODING_DP_8b_10b;
			else if (link_dp_get_encoding_format(&cur_link_settings) == DP_128b_132b_ENCODING)
				link_encoding = DC_LINK_ENCODING_DP_128b_132b;

			/* Flag if reduced link bandwidth no longer meets stream requirements or fallen back to
			 * minimum link bandwidth.
			 */
			req_bw = dc_bandwidth_in_kbps_from_timing(&stream->timing, link_encoding);
			link_bw = dp_link_bandwidth_kbps(link, &cur_link_settings);
			is_link_bw_low = (req_bw > link_bw);
			is_link_bw_min = ((cur_link_settings.link_rate <= LINK_RATE_LOW) &&
				(cur_link_settings.lane_count <= LANE_COUNT_ONE));

			if (is_link_bw_low)
				DC_LOG_WARNING(
					"%s: Link(%d) bandwidth too low after fallback req_bw(%d) > link_bw(%d)\n",
					__func__, link->link_index, req_bw, link_bw);
		}

		msleep(delay_between_attempts);
	}

	return false;
}

