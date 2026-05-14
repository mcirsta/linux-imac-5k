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

#ifndef __DC_LINK_DP_PHY_H__
#define __DC_LINK_DP_PHY_H__

#include "link_service.h"
void dp_enable_link_phy(
	struct dc_link *link,
	const struct link_resource *link_res,
	enum signal_type signal,
	enum clock_source_id clock_source,
	const struct dc_link_settings *link_settings);

void dp_disable_link_phy(struct dc_link *link,
		const struct link_resource *link_res,
		enum signal_type signal);

void dp_set_hw_lane_settings(
		struct dc_link *link,
		const struct link_resource *link_res,
		const struct link_training_settings *link_settings,
		uint32_t offset);

void dp_set_drive_settings(
	struct dc_link *link,
	const struct link_resource *link_res,
	struct link_training_settings *lt_settings);

enum dc_status dp_set_fec_ready(struct dc_link *link,
		const struct link_resource *link_res, bool ready);

void dp_set_fec_enable(struct dc_link *link,
		const struct link_resource *link_res, bool enable);

void dpcd_write_rx_power_ctrl(struct dc_link *link, bool on);

/*
 * iMac 5K diagnostic helpers. These are no-ops on non-iMac routes and only
 * emit logs when the calling link (or its peer tile) matches the Windows
 * primary 0x3114 / DDC4 / UNIPHY_C or secondary 0x3113 / DDC3 / UNIPHY_D
 * route identity. They are intended for tracing when the secondary AUX path
 * dies between commit_streams_pretrain and the lower stream-enable.
 */
void dp_imac5k_log_phy_event(struct dc_link *link, const char *func,
			     const char *checkpoint);
void dp_imac5k_probe_peer_aux(struct dc_link *acting_link, const char *func,
			      const char *checkpoint);

/*
 * Returns true when the trained iMac 5K secondary 0x3113 route must be
 * preserved (transmitter / link state left intact). Used by hw-sequencer
 * teardown paths that do not route through dp_disable_link_phy().
 */
bool dp_imac5k_link_preserves_secondary_output(const struct dc_link *link);

#endif /* __DC_LINK_DP_PHY_H__ */
