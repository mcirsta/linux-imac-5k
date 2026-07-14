/*
 * Copyright 2023 Advanced Micro Devices, Inc.
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
 * This file owns the programming sequence of stream's dpms state associated
 * with the link and link's enable/disable sequences as result of the stream's
 * dpms state change.
 *
 * TODO - The reason link owns stream's dpms programming sequence is
 * because dpms programming sequence is highly dependent on underlying signal
 * specific link protocols. This unfortunately causes link to own a portion of
 * stream state programming sequence. This creates a gray area where the
 * boundary between link and stream is not clearly defined.
 */

#include "dm_services.h"
#include "link_dpms.h"
#include "apple5k.h"
#include "link_hwss.h"
#include "link_validation.h"
#include "accessories/link_dp_trace.h"
#include "protocols/link_dpcd.h"
#include "protocols/link_ddc.h"
#include "protocols/link_hpd.h"
#include "protocols/link_dp_phy.h"
#include "protocols/link_dp_capability.h"
#include "protocols/link_dp_training.h"
#include "protocols/link_edp_panel_control.h"
#include "protocols/link_dp_panel_replay.h"
#include "protocols/link_dp_dpia_bw.h"

#include "dm_helpers.h"
#include "link_enc_cfg.h"
#include "resource.h"
#include "dsc.h"
#include "dccg.h"
#include "clk_mgr.h"
#include "atomfirmware.h"
#include "vpg.h"
#include "grph_object_id.h"
#include <linux/dmi.h>
#include <linux/kexec.h>
#include <linux/ktime.h>
#include <linux/reboot.h>

#define DC_LOGGER \
	dc_logger
#define DC_LOGGER_INIT(logger) \
	struct dal_logger *dc_logger = logger

#define LINK_INFO(...) \
	DC_LOG_HW_HOTPLUG(  \
		__VA_ARGS__)

#define RETIMER_REDRIVER_INFO(...) \
	DC_LOG_RETIMER_REDRIVER(  \
		__VA_ARGS__)

#define MAX_MTP_SLOT_COUNT 64
#define LINK_TRAINING_ATTEMPTS 4
#define PEAK_FACTOR_X1000 1006
#define APPLE_5K_DPCD_PANEL_LATCH 0x4F1

struct apple5k_panel_mode_status {
	uint8_t r41c;
	uint8_t r41f;
	uint8_t r423;
	uint8_t r425;
	uint8_t r42f;
	uint8_t r4f1;
	enum dc_status s41c;
	enum dc_status s41f;
	enum dc_status s423;
	enum dc_status s425;
	enum dc_status s42f;
	enum dc_status s4f1;
};

static struct dc_link *apple5k_root_for_link(struct dc_link *link)
{
	return link_apple5k_root_for_link(link);
}

static enum apple5k_tx_state apple5k_get_state(struct dc_link *root)
{
	enum apple5k_tx_state state = APPLE5K_TX_IDLE;

	if (!root)
		return state;
	mutex_lock(&root->apple5k.lock);
	state = root->apple5k.state;
	mutex_unlock(&root->apple5k.lock);
	return state;
}

static bool apple5k_read_panel_mode_status(struct dc_link *root,
					   struct apple5k_panel_mode_status *st)
{
	if (!st || !dc_link_has_tiled_root_panel_patch(root))
		return false;

	memset(st, 0, sizeof(*st));
	st->s41c = core_link_read_dpcd(root, 0x41C, &st->r41c, sizeof(st->r41c));
	st->s425 = core_link_read_dpcd(root, 0x425, &st->r425, sizeof(st->r425));
	st->s4f1 = core_link_read_dpcd(root, APPLE_5K_DPCD_PANEL_LATCH,
				       &st->r4f1, sizeof(st->r4f1));
	return true;
}

static void apple5k_read_panel_diagnostics(
		struct dc_link *root, struct apple5k_panel_mode_status *st)
{
	if (!st || !dc_link_has_tiled_root_panel_patch(root))
		return;

	st->s41f = core_link_read_dpcd(root, 0x41F, &st->r41f,
				       sizeof(st->r41f));
	st->s423 = core_link_read_dpcd(root, 0x423, &st->r423,
				       sizeof(st->r423));
	st->s42f = core_link_read_dpcd(root, 0x42F, &st->r42f,
				       sizeof(st->r42f));
}

static bool apple5k_panel_status_is_native_good(
		const struct apple5k_panel_mode_status *st)
{
	return st && st->s41c == DC_OK && st->r41c == 0x15 &&
	       st->s425 == DC_OK && st->r425 == 0x00 &&
	       st->s4f1 == DC_OK && st->r4f1 == 0x01;
}

static bool apple5k_panel_status_is_base(
		const struct apple5k_panel_mode_status *st)
{
	return st && st->s41c == DC_OK && st->r41c == 0x05 &&
	       st->s425 == DC_OK && st->r425 == 0x02 &&
	       st->s4f1 == DC_OK && st->r4f1 == 0x00;
}

static bool apple5k_panel_status_is_mixed(
		const struct apple5k_panel_mode_status *st)
{
	if (!st || st->s41c != DC_OK || st->s425 != DC_OK ||
	    st->s4f1 != DC_OK)
		return false;

	return (st->r4f1 == 1 && st->r425 == 0x02) ||
	       (st->r41c == 0x15 && st->r425 == 0x02);
}

static bool apple5k_panel_status_needs_base_clear(
		const struct apple5k_panel_mode_status *st)
{
	return st && !apple5k_panel_status_is_base(st) &&
	       ((st->s4f1 == DC_OK && st->r4f1 != 0) ||
		(st->s41c == DC_OK && st->r41c == 0x15));
}

/*
 * Explicit full diagnostic snapshot.  Successful hot paths use the minimal
 * 0x41C/0x425/0x4F1 verdict tuple and call this only at coarse transaction
 * boundaries; failures retain the complete reverse-engineering register set.
 */
static void apple5k_log_panel_mode_status(
		struct dc_link *link, struct dc_link *root, const char *stage,
		const struct apple5k_panel_mode_status *st)
{
	DC_LOGGER_INIT(root->ctx->logger);

	DC_LOG_INFO("APPLE5K-MODE stage=%s link[%u] root[%u] 0x41C=0x%02x(s=%d) 0x41F=0x%02x(s=%d) 0x423=0x%02x(s=%d,b2=%u) 0x425=0x%02x(s=%d) 0x42F=0x%02x(s=%d) 0x4F1=0x%02x(s=%d) -> %s\n",
		    stage ? stage : "?", link->link_index, root->link_index,
		    st->r41c, st->s41c, st->r41f, st->s41f, st->r423,
		    st->s423,
		    (st->s423 == DC_OK) ? ((st->r423 >> 2) & 1) : 0,
		    st->r425, st->s425, st->r42f, st->s42f, st->r4f1,
		    st->s4f1,
		    (st->s425 == DC_OK) ?
			    ((st->r425 & 0x02) ? "COMPAT" : "NATIVE") :
			    "0x425-NACK");
}

static void link_apple_5k_log_panel_mode(struct dc_link *link,
					 const char *stage)
{
	struct dc_link *root;
	struct apple5k_panel_mode_status st;

	if (!link)
		return;
	root = apple5k_root_for_link(link);
	if (!root || !link_apple5k_log_enabled(root->dc, APPLE5K_LOG_PANEL))
		return;

	if (!apple5k_read_panel_mode_status(root, &st))
		return;
	apple5k_read_panel_diagnostics(root, &st);
	apple5k_log_panel_mode_status(link, root, stage, &st);
}

static enum dc_status apple5k_write_root_latch(struct dc_link *root,
					       uint8_t value,
					       const char *stage)
{
	return link_apple5k_write_latch(root, value, stage);
}

/*
 * Firmware ready-gate (GopWaitForConnectorMaskReady). Both GOPs poll the HPD
 * line-state register DC_GPIO_HPD_Y until the two tile HPD pads assert:
 * root tile = HPD2 (0x100), sibling tile = HPD1 (0x1); bit layout is
 * identical on DCE 11.2 and DCE 12. The absolute dword address is the
 * firmware BAR byte offset >> 2 on both ASICs — the SOC15 split on Vega
 * (DCE seg2 0x34C0 + mmDC_GPIO_HPD_Y 0x210D = 0x55CD) only changes how the
 * named macros are organized, not the bus address dm_read_reg() takes.
 */
#define APPLE5K_READY_GATE_ROOT_HPD	0x100	/* DC_GPIO_HPD2_Y */
#define APPLE5K_READY_GATE_SIBLING_HPD	0x001	/* DC_GPIO_HPD1_Y */

static bool apple5k_pipe_is_blanked(struct pipe_ctx *pipe_ctx);

static bool apple5k_read_ready_gate(struct dc_link *root,
				    uint32_t *address_out,
				    uint32_t *value_out)
{
	uint32_t address;

	if (!root)
		return false;
	switch (root->ctx->dce_version) {
	case DCE_VERSION_11_2:
		address = 0x1223C >> 2;
		break;
	case DCE_VERSION_12_0:
		address = 0x15734 >> 2;
		break;
	default:
		return false;
	}
	if (address_out)
		*address_out = address;
	if (value_out)
		*value_out = dm_read_reg(root->ctx, address);
	return true;
}

static void apple5k_log_ready_gate(struct dc_link *root, const char *stage)
{
	uint32_t address;
	uint32_t value;

	if (!root || !link_apple5k_log_enabled(root->dc, APPLE5K_LOG_POWER))
		return;

	if (!apple5k_read_ready_gate(root, &address, &value))
		return;

	DC_LOGGER_INIT(root->ctx->logger);
	DC_LOG_INFO("APPLE5K-TXN ready-gate stage=%s addr=0x%05x DC_GPIO_HPD_Y=0x%08x root_hpd2=%u sibling_hpd1=%u\n",
		    stage ? stage : "?", address, value,
		    (value & APPLE5K_READY_GATE_ROOT_HPD) ? 1 : 0,
		    (value & APPLE5K_READY_GATE_SIBLING_HPD) ? 1 : 0);
}

/*
 * Depth-1 boot cold-down. On the no-iGPU iMacPro1,1 the firmware GOP drives
 * the tiled panel through boot, so amdgpu binds to a HOT display engine —
 * links trained and the panel session owned by firmware — while the iGPU
 * iMacs (15,1–20,1) hand over a cold engine, the only base the coordinated
 * native enable is proven on. Sanitize the Apple panel session over AUX
 * before the stock boot blank/power-down tears the engine down, so the first
 * 5K enable starts from the same cold base as an iGPU machine.
 */
bool link_apple_5k_wants_cold_boot(const struct dc *dc)
{
	unsigned int i;

	if (!dc || !dc->apple5k_policy.enabled ||
	    dc->apple5k_policy.boot_mode != APPLE5K_BOOT_COLD ||
	    dc->ctx->dce_version != DCE_VERSION_12_0 ||
	    !dmi_match(DMI_PRODUCT_NAME, "iMacPro1,1"))
		return false;

	for (i = 0; i < dc->link_count; i++)
		if (dc_link_has_tiled_root_panel_patch(dc->links[i]) &&
		    dc->links[i]->tiled_peer &&
		    dc_link_has_tiled_slave_panel_patch(
			    dc->links[i]->tiled_peer) &&
		    dc->links[i]->tiled_peer->tiled_peer == dc->links[i] &&
		    dc->links[i]->local_sink &&
		    dc->links[i]->tiled_peer->local_sink &&
		    dc->links[i]->link_enc &&
		    dc->links[i]->tiled_peer->link_enc &&
		    dc->links[i]->ddc && dc->links[i]->tiled_peer->ddc)
			return true;

	return false;
}

void link_apple_5k_boot_cold_down(struct dc *dc)
{
	struct dc_link *root = NULL;
	struct dc_link *slave = NULL;
	struct apple5k_panel_mode_status st;
	uint8_t power_state = DP_POWER_STATE_D3;
	enum dc_status status;
	bool found_root_pipe = false;
	bool found_slave_pipe = false;
	bool pair_quiet = true;
	unsigned int i;

	if (!link_apple_5k_wants_cold_boot(dc))
		return;

	for (i = 0; i < dc->link_count; i++) {
		if (dc_link_has_tiled_root_panel_patch(dc->links[i])) {
			root = dc->links[i];
			break;
		}
	}
	if (!root)
		return;
	slave = root->tiled_peer;

	DC_LOGGER_INIT(root->ctx->logger);
	DC_LOG_INFO("APPLE5K-COLD boot cold-down root_link[%u] slave_link[%d]\n",
		    root->link_index, slave ? (int)slave->link_index : -1);

	apple5k_log_ready_gate(root, "cold-boot:pre");
	link_apple_5k_log_panel_mode(root, "cold-boot:pre");
	mutex_lock(&root->apple5k.lock);
	root->apple5k.state = APPLE5K_TX_QUIESCING;
	root->apple5k.latch_owner = APPLE5K_LATCH_TEARDOWN;
	root->apple5k.block_rearm = true;
	mutex_unlock(&root->apple5k.lock);

	/*
	 * Quiesce any inherited pair pipes before touching panel state.  The
	 * normal accelerated-mode path will perform the complete source teardown.
	 */
	for (i = 0; dc->current_state && i < MAX_PIPES; i++) {
		struct pipe_ctx *pipe = &dc->current_state->res_ctx.pipe_ctx[i];

		if (pipe->stream && !pipe->top_pipe &&
		    link_apple5k_root_for_link(pipe->stream->link) == root) {
			if (pipe->stream->link == root)
				found_root_pipe = true;
			else if (pipe->stream->link == slave)
				found_slave_pipe = true;
			dc->hwss.blank_stream(pipe);
			if (!apple5k_pipe_is_blanked(pipe))
				pair_quiet = false;
		}
	}
	if (root->link_enc && root->link_enc->funcs->is_dig_enabled &&
	    root->link_enc->funcs->is_dig_enabled(root->link_enc) &&
	    !found_root_pipe)
		pair_quiet = false;
	if (slave->link_enc && slave->link_enc->funcs->is_dig_enabled &&
	    slave->link_enc->funcs->is_dig_enabled(slave->link_enc) &&
	    !found_slave_pipe)
		pair_quiet = false;
	if (!pair_quiet) {
		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_BLOCKED;
		root->apple5k.block_rearm = true;
		mutex_unlock(&root->apple5k.lock);
		return;
	}

	/* Put the slave in D3 before a root clear can hide its AUX route. */
	if (slave && slave->local_sink) {
		status = core_link_write_dpcd(slave, DP_SET_POWER,
					      &power_state, sizeof(power_state));
		DC_LOG_INFO("APPLE5K-COLD slave sink D3 slave_link[%u] status=%d\n",
			    slave->link_index, status);
	}

	status = apple5k_write_root_latch(root, 0, "cold-boot:disarm");
	msleep(10);
	if (status == DC_OK &&
	    (!apple5k_read_panel_mode_status(root, &st) ||
	     !apple5k_panel_status_is_base(&st)))
		status = DC_ERROR_UNEXPECTED;
	mutex_lock(&root->apple5k.lock);
	root->apple5k.state = status == DC_OK ? APPLE5K_TX_READY :
						       APPLE5K_TX_BLOCKED;
	root->apple5k.block_rearm = status != DC_OK;
	if (status == DC_OK)
		root->apple5k.latch_owner = APPLE5K_LATCH_NONE;
	mutex_unlock(&root->apple5k.lock);

	link_apple_5k_log_panel_mode(root, "cold-boot:done");
	apple5k_log_ready_gate(root, "cold-boot:done");
}

static enum dc_status apple5k_neutralize_pair(struct dc_link *root,
					      struct dc_link *slave,
					      struct pipe_ctx *root_pipe,
					      struct pipe_ctx *slave_pipe,
					      const char *stage)
{
	struct apple5k_panel_mode_status st;
	uint8_t power_state = DP_POWER_STATE_D3;
	enum dc_status power_status = DC_ERROR_UNEXPECTED;
	enum dc_status disarm_status;
	enum apple5k_tx_state next_state;
	bool pair_blank = true;
	bool base = false;

	if (!root || !slave ||
	    !dc_link_has_tiled_root_panel_patch(root) ||
	    !dc_link_has_tiled_slave_panel_patch(slave))
		return DC_ERROR_UNEXPECTED;

	DC_LOGGER_INIT(root->ctx->logger);
	mutex_lock(&root->apple5k.lock);
	root->apple5k.state = APPLE5K_TX_ABORTING;
	root->apple5k.latch_owner = APPLE5K_LATCH_TEARDOWN;
	root->apple5k.block_rearm = true;
	mutex_unlock(&root->apple5k.lock);
	root->link_state_valid = false;
	slave->link_state_valid = false;
	slave->apple5k_source_table_ok = false;

	if (root_pipe)
		root->dc->hwss.blank_stream(root_pipe);
	if (slave_pipe)
		root->dc->hwss.blank_stream(slave_pipe);
	if ((root_pipe && !apple5k_pipe_is_blanked(root_pipe)) ||
	    (slave_pipe && !apple5k_pipe_is_blanked(slave_pipe)) ||
	    (!!root_pipe != !!slave_pipe))
		pair_blank = false;

	/* Never clear the pair latch while either source can still scan out. */
	if (!pair_blank) {
		DC_LOG_ERROR("APPLE5K-TXN neutralize blocked stage=%s root_link[%u] reason=pair-not-blank\n",
			     stage ? stage : "?", root->link_index);
		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_BLOCKED;
		mutex_unlock(&root->apple5k.lock);
		return DC_ERROR_UNEXPECTED;
	}

	/* A root clear can hide the sibling AUX route, so request D3 first. */
	if (slave->local_sink)
		power_status = core_link_write_dpcd(slave, DP_SET_POWER,
						  &power_state,
						  sizeof(power_state));

	disarm_status = apple5k_write_root_latch(root, 0, stage);
	msleep(10);
	if (disarm_status == DC_OK &&
	    apple5k_read_panel_mode_status(root, &st) &&
	    apple5k_panel_status_is_base(&st))
		base = true;

	next_state = base ? APPLE5K_TX_READY : APPLE5K_TX_BLOCKED;
	mutex_lock(&root->apple5k.lock);
	root->apple5k.state = next_state;
	root->apple5k.block_rearm = !base;
	if (base)
		root->apple5k.latch_owner = APPLE5K_LATCH_NONE;
	mutex_unlock(&root->apple5k.lock);

	DC_LOG_INFO("APPLE5K-TXN neutralize stage=%s root_link[%u] slave_link[%u] blank=%d slave_d3_s=%d disarm_s=%d base=%d next=%s\n",
		    stage ? stage : "?", root->link_index, slave->link_index,
		    pair_blank, power_status, disarm_status, base,
		    link_apple5k_state_name(next_state));
	if (!base)
		link_apple_5k_log_panel_mode(root, "native-txn:abort-blocked");

	return base ? DC_OK : DC_ERROR_UNEXPECTED;
}

#define APPLE5K_ROOT_AUX_RETRIES		5
#define APPLE5K_ROOT_AUX_RETRY_MS	10

static bool apple5k_prepare_root_aux(struct dc_link *root)
{
	uint8_t dpcd_rev = 0;
	enum dc_status status = DC_ERROR_UNEXPECTED;
	bool panel_on;
	int attempt;
	int attempts = 0;

	if (!root || root->connector_signal != SIGNAL_TYPE_EDP ||
	    !root->panel_cntl || !root->panel_cntl->funcs ||
	    !root->panel_cntl->funcs->is_panel_powered_on ||
	    !root->dc->hwss.edp_power_control ||
	    !root->dc->hwss.edp_wait_for_hpd_ready)
		return false;

	DC_LOGGER_INIT(root->ctx->logger);
	if (!root->dc->config.edp_no_power_sequencing)
		root->dc->hwss.edp_power_control(root, true);
	root->dc->hwss.edp_wait_for_hpd_ready(root, true);
	panel_on = root->panel_cntl->funcs->is_panel_powered_on(
			root->panel_cntl);

	for (attempt = 1; panel_on && attempt <= APPLE5K_ROOT_AUX_RETRIES;
	     attempt++) {
		attempts = attempt;
		dpcd_rev = 0;
		status = core_link_read_dpcd(root, DP_DPCD_REV, &dpcd_rev,
					     sizeof(dpcd_rev));
		if (status == DC_OK && dpcd_rev != 0)
			break;
		if (attempt < APPLE5K_ROOT_AUX_RETRIES)
			msleep(APPLE5K_ROOT_AUX_RETRY_MS);
	}

	DC_LOG_INFO("APPLE5K-TXN root-ready root_link[%u] panel_on=%d aux_s=%d dpcd_rev=0x%02x attempts=%d\n",
		    root->link_index, panel_on, status, dpcd_rev,
		    attempts);

	return panel_on && status == DC_OK && dpcd_rev != 0;
}

static bool apple5k_begin_native_enable(struct dc_link *root,
					struct dc_link *slave,
					struct pipe_ctx *root_pipe,
					struct pipe_ctx *slave_pipe)
{
	struct apple5k_panel_mode_status st;
	uint8_t power_state = DP_POWER_STATE_D0;
	uint8_t dpcd_rev = 0;
	uint32_t ready_gate = 0;
	enum dc_status power_status = DC_ERROR_UNEXPECTED;
	enum dc_status rev_status = DC_ERROR_UNEXPECTED;
	enum dc_status status;
	enum apple5k_tx_state prior_state;
	enum apple5k_latch_owner prior_owner;
	bool transactional;
	bool ready;
	bool have_status;
	int elapsed_ms;

	if (!dc_link_has_tiled_root_panel_patch(root) ||
	    !dc_link_has_tiled_slave_panel_patch(slave) ||
	    root->tiled_peer != slave || slave->tiled_peer != root ||
	    !root->local_sink || !slave->local_sink ||
	    !root->link_enc || !slave->link_enc ||
	    !root->ddc || !slave->ddc ||
	    !root_pipe || !slave_pipe ||
	    !root_pipe->stream || !slave_pipe->stream ||
	    root_pipe->stream->link != root ||
	    slave_pipe->stream->link != slave ||
	    !apple5k_pipe_is_blanked(root_pipe) ||
	    !apple5k_pipe_is_blanked(slave_pipe))
		return false;
	transactional = root->dc->apple5k_policy.pair_mode ==
						APPLE5K_PAIR_TRANSACTIONAL;
	ready = !transactional;

	DC_LOGGER_INIT(root->ctx->logger);
	mutex_lock(&root->apple5k.lock);
	if (root->apple5k.state == APPLE5K_TX_BLOCKED ||
	    root->apple5k.block_rearm) {
		mutex_unlock(&root->apple5k.lock);
		return false;
	}
	/*
	 * Enter ENABLING before powering the root so the eDP power-off guard
	 * holds VDD through the readiness probe and latch arm.  No panel-private
	 * write is allowed until the root AUX probe below succeeds.
	 */
	prior_state = root->apple5k.state;
	prior_owner = root->apple5k.latch_owner;
	root->apple5k.generation++;
	root->apple5k.state = APPLE5K_TX_ENABLING;
	root->apple5k.latch_owner = APPLE5K_LATCH_ENABLE;
	mutex_unlock(&root->apple5k.lock);
	if (!apple5k_prepare_root_aux(root)) {
		DC_LOG_ERROR("APPLE5K-TXN refusing enable without root VDD/AUX root_link[%u]\n",
			     root->link_index);
		mutex_lock(&root->apple5k.lock);
		if (root->apple5k.state == APPLE5K_TX_ENABLING &&
		    root->apple5k.latch_owner == APPLE5K_LATCH_ENABLE) {
			root->apple5k.state = prior_state;
			root->apple5k.latch_owner = prior_owner;
		}
		mutex_unlock(&root->apple5k.lock);
		return false;
	}
	mutex_lock(&root->apple5k.lock);
	if (root->apple5k.state != APPLE5K_TX_ENABLING ||
	    root->apple5k.latch_owner != APPLE5K_LATCH_ENABLE ||
	    root->apple5k.block_rearm) {
		mutex_unlock(&root->apple5k.lock);
		return false;
	}
	mutex_unlock(&root->apple5k.lock);

	apple5k_log_ready_gate(root, "native-txn:pre");
	have_status = apple5k_read_panel_mode_status(root, &st);
	if (have_status &&
	    link_apple5k_log_enabled(root->dc, APPLE5K_LOG_PANEL)) {
		apple5k_read_panel_diagnostics(root, &st);
		apple5k_log_panel_mode_status(root, root, "native-txn:pre",
						&st);
	}
	if (!have_status || st.s41c != DC_OK || st.s425 != DC_OK ||
	    st.s4f1 != DC_OK ||
	    (apple5k_panel_status_is_mixed(&st) &&
	     prior_state != APPLE5K_TX_DISCOVERING)) {
		DC_LOG_INFO("APPLE5K-TXN refusing enable from unknown/mixed state root_link[%u] 41c=0x%02x(s=%d) 425=0x%02x(s=%d) 4f1=0x%02x(s=%d)\n",
			    root->link_index, st.r41c, st.s41c, st.r425,
			    st.s425, st.r4f1, st.s4f1);
		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_BLOCKED;
		root->apple5k.block_rearm = true;
		mutex_unlock(&root->apple5k.lock);
		return false;
	}

	if (apple5k_panel_status_needs_base_clear(&st)) {
		status = apple5k_write_root_latch(root, 0,
						  "native-txn:clear-stale");
		msleep(10);
		have_status = apple5k_read_panel_mode_status(root, &st);
		if (have_status &&
		    link_apple5k_log_enabled(root->dc, APPLE5K_LOG_PANEL)) {
			apple5k_read_panel_diagnostics(root, &st);
			apple5k_log_panel_mode_status(root, root,
						"native-txn:after-clear", &st);
		}
		if (status != DC_OK || !have_status ||
		    !apple5k_panel_status_is_base(&st)) {
			DC_LOG_INFO("APPLE5K-TXN clear-stale failed root_link[%u] status=%d\n",
				    root->link_index, status);
			mutex_lock(&root->apple5k.lock);
			root->apple5k.state = APPLE5K_TX_BLOCKED;
			root->apple5k.block_rearm = true;
			mutex_unlock(&root->apple5k.lock);
			return false;
		}
	} else if (!apple5k_panel_status_is_base(&st)) {
		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_BLOCKED;
		root->apple5k.block_rearm = true;
		mutex_unlock(&root->apple5k.lock);
		return false;
	}

	status = apple5k_write_root_latch(root, 1, "native-txn:arm");
	if (status != DC_OK) {
		DC_LOG_INFO("APPLE5K-TXN arm failed root_link[%u] slave_link[%u] status=%d\n",
			    root->link_index, slave->link_index, status);
		apple5k_neutralize_pair(root, slave, root_pipe, slave_pipe,
					    "native-txn:arm-fail-disarm");
		return false;
	}

	msleep(10);
	for (elapsed_ms = 0; !ready && elapsed_ms <= 210; elapsed_ms += 5) {
		if (apple5k_read_ready_gate(root, NULL, &ready_gate) &&
		    (ready_gate & (APPLE5K_READY_GATE_ROOT_HPD |
				   APPLE5K_READY_GATE_SIBLING_HPD)) ==
				  (APPLE5K_READY_GATE_ROOT_HPD |
				   APPLE5K_READY_GATE_SIBLING_HPD)) {
			power_status = core_link_write_dpcd(slave, DP_SET_POWER,
							    &power_state,
							    sizeof(power_state));
			dpcd_rev = 0;
			rev_status = core_link_read_dpcd(slave, DP_DPCD_REV,
							 &dpcd_rev,
							 sizeof(dpcd_rev));
			if (power_status == DC_OK && rev_status == DC_OK &&
			    dpcd_rev != 0) {
				ready = true;
				break;
			}
		}
		if (elapsed_ms < 210)
			msleep(5);
	}
	if (!ready) {
		DC_LOG_ERROR("APPLE5K-TXN ready/AUX timeout root_link[%u] slave_link[%u] gate=0x%08x power_s=%d rev_s=%d rev=0x%02x\n",
			     root->link_index, slave->link_index, ready_gate,
			     power_status, rev_status, dpcd_rev);
		apple5k_neutralize_pair(root, slave, root_pipe, slave_pipe,
					    "native-txn:ready-fail-disarm");
		return false;
	}
	if (transactional &&
	    link_apple5k_log_enabled(root->dc, APPLE5K_LOG_LINK))
		DC_LOG_INFO("APPLE5K-TXN ready/AUX root_link[%u] slave_link[%u] elapsed_ms=%d gate=0x%08x dpcd_rev=0x%02x\n",
			    root->link_index, slave->link_index, elapsed_ms,
			    ready_gate, dpcd_rev);
	return true;
}

struct apple5k_link_status {
	uint8_t dpcd[4];	/* 0x202..0x205 */
	enum dc_status status;
	unsigned int lane_count;
	bool has_sink_status;
	bool trained;
};

/* Read the contiguous lane/alignment block in one AUX transaction. */
static bool apple5k_read_link_status(struct dc_link *link, bool include_sink,
				      struct apple5k_link_status *st)
{
	unsigned int size;

	if (!st)
		return false;
	size = include_sink ? sizeof(st->dpcd) : 3;
	memset(st, 0, sizeof(*st));
	if (!link || !link->local_sink)
		return false;

	st->lane_count = link->cur_link_settings.lane_count;
	st->has_sink_status = include_sink;
	if (st->lane_count < 1 || st->lane_count > 4)
		return false;
	st->status = core_link_read_dpcd(link, DP_LANE0_1_STATUS,
					 st->dpcd, size);
	if (st->status != DC_OK || !(st->dpcd[2] & 0x01) ||
	    (st->dpcd[0] & 0x07) != 0x07 ||
	    (st->lane_count >= 2 &&
	     ((st->dpcd[0] >> 4) & 0x07) != 0x07) ||
	    (st->lane_count >= 3 && (st->dpcd[1] & 0x07) != 0x07) ||
	    (st->lane_count >= 4 &&
	     ((st->dpcd[1] >> 4) & 0x07) != 0x07))
		return false;

	st->trained = true;
	return true;
}

static void apple5k_log_link_status(
		struct dc_link *link, const char *stage,
		const struct apple5k_link_status *st)
{
	if (!link || !st ||
	    !link_apple5k_log_enabled(link->dc, APPLE5K_LOG_LINK))
		return;

	DC_LOGGER_INIT(link->ctx->logger);
	DC_LOG_INFO("APPLE5K-TXN link-status stage=%s link[%u] lanes=%u status=%d 0x202=0x%02x 0x203=0x%02x 0x204=0x%02x sink_valid=%d 0x205=0x%02x trained=%d\n",
		    stage ? stage : "?", link->link_index, st->lane_count,
		    st->status, st->dpcd[0], st->dpcd[1], st->dpcd[2],
		    st->has_sink_status, st->dpcd[3], st->trained);
}

#define APPLE5K_NATIVE_POLL_INTERVAL_MS	5
#define APPLE5K_NATIVE_POLL_TIMEOUT_MS	100

static void apple5k_log_sink_link_status(struct dc_link *link);

static void apple5k_log_native_poll_change(
		struct dc_link *root, const struct apple5k_panel_mode_status *st,
		struct apple5k_panel_mode_status *prev, bool *have_prev,
		unsigned int poll, int elapsed_ms)
{
	if (!link_apple5k_log_enabled(root->dc, APPLE5K_LOG_PANEL) ||
	    (*have_prev && st->r41c == prev->r41c &&
	     st->r425 == prev->r425 && st->r4f1 == prev->r4f1))
		return;

	DC_LOGGER_INIT(root->ctx->logger);
	DC_LOG_INFO("APPLE5K-TXN native-poll #%u t=%dms 0x41C=0x%02x 0x425=0x%02x 0x4F1=0x%02x -> %s\n",
		    poll, elapsed_ms, st->r41c, st->r425, st->r4f1,
		    (st->s425 == DC_OK) ?
			    ((st->r425 & 0x02) ? "COMPAT" : "NATIVE") :
			    "0x425-NACK");
	*prev = *st;
	*have_prev = true;
}

static bool apple5k_finish_native_enable(struct dc_link *root,
					 struct dc_link *slave,
					 struct pipe_ctx *root_pipe,
					 struct pipe_ctx *slave_pipe,
					 bool transaction_started)
{
	struct apple5k_panel_mode_status st;
	struct apple5k_panel_mode_status prev;
	struct apple5k_link_status root_link_status;
	struct apple5k_link_status slave_link_status;
	bool have_prev = false;
	bool native_tuple_seen = false;
	bool native_good = false;
	bool root_trained = false;
	bool slave_trained = false;
	uint8_t slave_sink_status = 0;
	enum dc_status slave_sink_read = DC_ERROR_UNEXPECTED;
	ktime_t start;
	ktime_t native_at = 0;
	unsigned int polls = 0;
	int elapsed_ms = 0;

	if (!dc_link_has_tiled_root_panel_patch(root))
		return false;

	DC_LOGGER_INIT(root->ctx->logger);
	memset(&prev, 0, sizeof(prev));
	memset(&root_link_status, 0, sizeof(root_link_status));
	memset(&slave_link_status, 0, sizeof(slave_link_status));

	/*
	 * Source-side snapshot: was each tile's OTG (CRTC master-enable)
	 * actually running while we asked the panel to latch native? A panel
	 * stuck in compat with both timing generators up points at a missing
	 * source-signal step rather than a DPCD-sequencing problem.
	 */
	if (link_apple5k_log_enabled(root->dc, APPLE5K_LOG_TIMING) &&
	    root_pipe && root_pipe->stream_res.tg &&
	    root_pipe->stream_res.tg->funcs->is_tg_enabled &&
	    slave_pipe && slave_pipe->stream_res.tg &&
	    slave_pipe->stream_res.tg->funcs->is_tg_enabled) {
		struct timing_generator *rtg = root_pipe->stream_res.tg;
		struct timing_generator *stg = slave_pipe->stream_res.tg;

		DC_LOG_INFO("APPLE5K-TXN otg-state root_tg=%u en=%d slave_tg=%u en=%d\n",
			    rtg->inst, rtg->funcs->is_tg_enabled(rtg),
			    stg->inst, stg->funcs->is_tg_enabled(stg));
	}

	apple5k_log_ready_gate(root, "native-txn:verify");

	/*
	 * Poll the native verdict instead of sampling once: the firmware may
	 * reflect native in 0x425/0x41C later than the arm settle. Log every
	 * change of the (0x41C,0x425,0x4F1) tuple plus the time to first
	 * native, so a slow convergence is distinguishable from "never".
	 */
	start = ktime_get();
	for (;;) {
		elapsed_ms = (int)ktime_ms_delta(ktime_get(), start);
		polls++;
		if (!native_tuple_seen &&
		    apple5k_read_panel_mode_status(root, &st)) {
			apple5k_log_native_poll_change(root, &st, &prev,
						 &have_prev, polls, elapsed_ms);
			native_tuple_seen = apple5k_panel_status_is_native_good(&st);
		}
		if (native_tuple_seen) {
			if (!slave->apple5k_source_table_ok)
				break;
			root_trained = apple5k_read_link_status(
					root, false, &root_link_status);
			slave_trained = apple5k_read_link_status(
					slave, true, &slave_link_status);
			slave_sink_status = slave_link_status.dpcd[3];
			slave_sink_read = slave_link_status.status;
			if (slave_sink_read == DC_OK &&
			    (slave_sink_status & 0x01) &&
			    root_trained && slave_trained &&
			    apple5k_read_panel_mode_status(root, &st)) {
				apple5k_log_native_poll_change(root, &st, &prev,
							 &have_prev, polls,
							 elapsed_ms);
				if (apple5k_panel_status_is_native_good(&st)) {
					native_good = true;
					native_at = ktime_get();
					break;
				}
				native_tuple_seen = false;
			}
		}
		if (elapsed_ms >= APPLE5K_NATIVE_POLL_TIMEOUT_MS)
			break;
		msleep(APPLE5K_NATIVE_POLL_INTERVAL_MS);
	}
	apple5k_log_link_status(root, "native-txn:final",
				   &root_link_status);
	apple5k_log_link_status(slave, "native-txn:final",
				   &slave_link_status);

	DC_LOG_INFO("APPLE5K-TXN native-poll done root_link[%u] polls=%u native_good=%d source_table_ok=%d root_trained=%d slave_trained=%d slave_sink=0x%02x(s=%d) time_to_native_ms=%d total_ms=%d\n",
		    root->link_index, polls, native_good,
		    slave->apple5k_source_table_ok,
		    root_trained, slave_trained,
		    slave_sink_status, slave_sink_read,
		    native_good ? (int)ktime_ms_delta(native_at, start) : -1,
		    (int)ktime_ms_delta(ktime_get(), start));

	if (transaction_started && native_good) {
		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_NATIVE;
		root->apple5k.latch_owner = APPLE5K_LATCH_NONE;
		root->apple5k.block_rearm = false;
		mutex_unlock(&root->apple5k.lock);
		DC_LOG_INFO("APPLE5K-TXN verified native root_link[%u] slave_link[%u]\n",
			    root->link_index, slave ? slave->link_index : 0xffffffff);
		return true;
	}

	DC_LOG_INFO("APPLE5K-TXN native verify failed root_link[%u] slave_link[%u] started=%d native_good=%d; pair-quiescing\n",
		    root->link_index, slave ? slave->link_index : 0xffffffff,
		    transaction_started, native_good);
	link_apple_5k_log_panel_mode(root, "native-txn:verify-fail");
	apple5k_log_sink_link_status(root);
	apple5k_log_sink_link_status(slave);
	apple5k_neutralize_pair(root, slave, root_pipe, slave_pipe,
				    "native-txn:verify-fail-disarm");
	return false;
}

static void dp_write_tiled_stream_enable_latch(struct dc_link *link)
{
	enum dc_status status;
	struct dc_link *root = apple5k_root_for_link(link);
	bool enable_owned = true;

	if (!dc_link_needs_tiled_stream_enable_latch(link) || !link->local_sink)
		return;
	if (root && root->dc->apple5k_policy.pair_mode ==
					APPLE5K_PAIR_TRANSACTIONAL) {
		mutex_lock(&root->apple5k.lock);
		enable_owned = root->apple5k.state == APPLE5K_TX_ENABLING &&
			root->apple5k.latch_owner == APPLE5K_LATCH_ENABLE;
		mutex_unlock(&root->apple5k.lock);
	}
	if (!root || !enable_owned) {
		DC_LOGGER_INIT(link->ctx->logger);
		DC_LOG_INFO("APPLE5K: stream-enable latch skipped link[%u] root_link[%d] reason=no-native-transaction\n",
			    link->link_index, root ? (int)root->link_index : -1);
		return;
	}

	DC_LOGGER_INIT(link->ctx->logger);

	status = link_apple5k_write_latch(link, 1, "stream-enable-latch");
	DC_LOG_INFO("APPLE5K: stream-enable latch link[%u] status=%d sink=%p\n",
		    link->link_index, status, link->local_sink);
}

/*
 * Failure-only sink snapshot.  Keep these extra configuration/private reads
 * out of successful stream enable and native verification.
 */
static void apple5k_log_sink_link_status(struct dc_link *link)
{
	uint8_t lane_status[4] = { 0 };	/* 0x202 0x203 0x204 0x205 */
	uint8_t edp_cfg = 0;		/* 0x10A ASSR/panel-mode */
	uint8_t lane_count = 0;		/* 0x101 lane count set */
	uint8_t downspread = 0;		/* 0x107 MSA-ignore */
	uint8_t latch[3] = { 0 };	/* 0x4F0 0x4F1 0x4F2 Apple-private */

	if (!link || !link->local_sink ||
	    (!dc_link_has_tiled_root_panel_patch(link) &&
	     !dc_link_has_tiled_slave_panel_patch(link)) ||
	    !link_apple5k_log_enabled(link->dc, APPLE5K_LOG_LINK))
		return;

	DC_LOGGER_INIT(link->ctx->logger);

	core_link_read_dpcd(link, DP_LANE0_1_STATUS, lane_status, sizeof(lane_status));
	core_link_read_dpcd(link, DP_EDP_CONFIGURATION_SET, &edp_cfg, sizeof(edp_cfg));
	core_link_read_dpcd(link, DP_LANE_COUNT_SET, &lane_count, sizeof(lane_count));
	core_link_read_dpcd(link, DP_DOWNSPREAD_CTRL, &downspread, sizeof(downspread));
	core_link_read_dpcd(link, APPLE_5K_DPCD_PANEL_LATCH - 1, latch, sizeof(latch));

	DC_LOG_INFO("APPLE5K: sink-status link[%u] signal=%d lane01=0x%02x lane23=0x%02x align=0x%02x sink=0x%02x assr_0x10a=0x%02x lanes_0x101=%u msa_0x107=0x%02x latch_4f0_4f2=%02x %02x %02x\n",
		    link->link_index, link->connector_signal,
		    lane_status[0], lane_status[1], lane_status[2], lane_status[3],
		    edp_cfg, lane_count & 0x1f, downspread,
		    latch[0], latch[1], latch[2]);
}

void link_blank_all_dp_displays(struct dc *dc)
{
	unsigned int i;
	uint8_t dpcd_power_state = '\0';
	enum dc_status status = DC_ERROR_UNEXPECTED;

	for (i = 0; i < dc->link_count; i++) {
		if ((dc->links[i]->connector_signal != SIGNAL_TYPE_DISPLAY_PORT) ||
			(dc->links[i]->priv == NULL) || (dc->links[i]->local_sink == NULL))
			continue;

		/* DP 2.0 spec requires that we read LTTPR caps first */
		dp_retrieve_lttpr_cap(dc->links[i]);
		/* if any of the displays are lit up turn them off */
		status = core_link_read_dpcd(dc->links[i], DP_SET_POWER,
							&dpcd_power_state, sizeof(dpcd_power_state));

		if (status == DC_OK && dpcd_power_state == DP_POWER_STATE_D0)
			link_blank_dp_stream(dc->links[i], true);
	}

}

void link_blank_all_edp_displays(struct dc *dc)
{
	unsigned int i;
	uint8_t dpcd_power_state = '\0';
	enum dc_status status = DC_ERROR_UNEXPECTED;

	for (i = 0; i < dc->link_count; i++) {
		if ((dc->links[i]->connector_signal != SIGNAL_TYPE_EDP) ||
			(!dc->links[i]->edp_sink_present))
			continue;

		/* if any of the displays are lit up turn them off */
		status = core_link_read_dpcd(dc->links[i], DP_SET_POWER,
							&dpcd_power_state, sizeof(dpcd_power_state));

		if (status == DC_OK && dpcd_power_state == DP_POWER_STATE_D0)
			link_blank_dp_stream(dc->links[i], true);
	}
}

void link_blank_dp_stream(struct dc_link *link, bool hw_init)
{
	unsigned int j;
	struct dc  *dc = link->ctx->dc;
	enum signal_type signal = link->connector_signal;

	if ((signal == SIGNAL_TYPE_EDP) ||
		(signal == SIGNAL_TYPE_DISPLAY_PORT)) {
		if (link->ep_type == DISPLAY_ENDPOINT_PHY &&
			link->link_enc->funcs->get_dig_frontend &&
			link->link_enc->funcs->is_dig_enabled(link->link_enc)) {
			int fe = link->link_enc->funcs->get_dig_frontend(link->link_enc);

			if (fe != ENGINE_ID_UNKNOWN)
				for (j = 0; j < dc->res_pool->stream_enc_count; j++) {
					if (fe == dc->res_pool->stream_enc[j]->id) {
						dc->res_pool->stream_enc[j]->funcs->dp_blank(link,
									dc->res_pool->stream_enc[j]);
						break;
					}
				}
		}

		if (((!dc->is_switch_in_progress_dest) && ((!link->wa_flags.dp_keep_receiver_powered) || hw_init)) &&
			(link->type != dc_connection_none))
			dpcd_write_rx_power_ctrl(link, false);
	}
}

void link_set_all_streams_dpms_off_for_link(struct dc_link *link)
{
	struct pipe_ctx *pipes[MAX_PIPES];
	struct dc_stream_state *streams[MAX_PIPES];
	struct dc_state *state = link->dc->current_state;
	uint8_t count;
	int i;
	struct dc_stream_update stream_update;
	bool dpms_off = true;
	struct link_resource link_res = {0};

	memset(&stream_update, 0, sizeof(stream_update));
	stream_update.dpms_off = &dpms_off;

	link_get_master_pipes_with_dpms_on(link, state, &count, pipes);

	/* The subsequent call to dc_commit_updates_for_stream for a full update
	 * will release the current state and swap to a new state. Releasing the
	 * current state results in the stream pointers in the pipe_ctx structs
	 * to be zero'd. Hence, cache all streams prior to dc_commit_updates_for_stream.
	 */
	for (i = 0; i < count; i++)
		streams[i] = pipes[i]->stream;

	for (i = 0; i < count; i++) {
		stream_update.stream = streams[i];
		dc_commit_updates_for_stream(link->ctx->dc, NULL, 0,
				streams[i], &stream_update,
				state);
	}

	/* link can be also enabled by vbios. In this case it is not recorded
	 * in pipe_ctx. Disable link phy here to make sure it is completely off
	 */
	dp_disable_link_phy(link, &link_res, link->connector_signal);
}

void link_resume(struct dc_link *link)
{
	if (link->connector_signal != SIGNAL_TYPE_VIRTUAL)
		program_hpd_filter(link);
}

/* This function returns true if the pipe is used to feed video signal directly
 * to the link.
 */
static bool is_master_pipe_for_link(const struct dc_link *link,
		const struct pipe_ctx *pipe)
{
	return resource_is_pipe_type(pipe, OTG_MASTER) &&
			pipe->stream->link == link;
}

/*
 * This function finds all master pipes feeding to a given link with dpms set to
 * on in given dc state.
 */
void link_get_master_pipes_with_dpms_on(const struct dc_link *link,
		struct dc_state *state,
		uint8_t *count,
		struct pipe_ctx *pipes[MAX_PIPES])
{
	int i;
	struct pipe_ctx *pipe = NULL;

	*count = 0;
	for (i = 0; i < MAX_PIPES; i++) {
		pipe = &state->res_ctx.pipe_ctx[i];

		if (is_master_pipe_for_link(link, pipe) &&
				pipe->stream->dpms_off == false) {
			pipes[(*count)++] = pipe;
		}
	}
}

static bool apple5k_is_tile(struct dc_link *link)
{
	return apple5k_root_for_link(link) != NULL;
}

static bool apple5k_pipe_is_tile(struct pipe_ctx *pipe_ctx)
{
	return pipe_ctx && pipe_ctx->stream &&
	       apple5k_is_tile(pipe_ctx->stream->link);
}

static bool apple5k_pipe_is_blanked(struct pipe_ctx *pipe_ctx)
{
	if (!pipe_ctx || !pipe_ctx->stream_res.tg)
		return false;

	if (pipe_ctx->stream_res.opp &&
	    pipe_ctx->stream_res.opp->funcs->dpg_is_blanked)
		return pipe_ctx->stream_res.opp->funcs->dpg_is_blanked(
				pipe_ctx->stream_res.opp);

	if (pipe_ctx->stream_res.tg->funcs->is_blanked)
		return pipe_ctx->stream_res.tg->funcs->is_blanked(
				pipe_ctx->stream_res.tg);

	return false;
}

enum apple5k_target_class {
	APPLE5K_TARGET_NONE = 0,
	APPLE5K_TARGET_ROOT_ONLY,
	APPLE5K_TARGET_SLAVE_ONLY,
	APPLE5K_TARGET_PAIR,
	APPLE5K_TARGET_INVALID,
};

struct apple5k_target_snapshot {
	enum apple5k_target_class class;
	struct dc_stream_state *root_stream;
	struct dc_stream_state *slave_stream;
	struct pipe_ctx *root_pipe;
	struct pipe_ctx *slave_pipe;
	unsigned int root_stream_count;
	unsigned int slave_stream_count;
	unsigned int root_pipe_count;
	unsigned int slave_pipe_count;
};

static const char *apple5k_target_class_name(enum apple5k_target_class class)
{
	switch (class) {
	case APPLE5K_TARGET_NONE: return "NONE";
	case APPLE5K_TARGET_ROOT_ONLY: return "ROOT_ONLY";
	case APPLE5K_TARGET_SLAVE_ONLY: return "SLAVE_ONLY";
	case APPLE5K_TARGET_PAIR: return "PAIR";
	case APPLE5K_TARGET_INVALID: return "INVALID";
	default: return "?";
	}
}

/*
 * Classify one immutable dc_state snapshot once.  Stream membership expresses
 * commit intent; the pipe scan validates that the same root/slave streams have
 * exactly one top-level timing source each.  Consumers must act on this class
 * and the captured pipes instead of reconstructing independent booleans.
 */
static void apple5k_classify_target(struct dc_state *state,
					 struct dc_link *root,
					 struct apple5k_target_snapshot *target)
{
	struct dc_link *slave;
	bool invalid = false;
	int i;

	memset(target, 0, sizeof(*target));
	target->class = APPLE5K_TARGET_INVALID;
	if (!state || !dc_link_has_tiled_root_panel_patch(root))
		return;

	slave = root->tiled_peer;
	for (i = 0; i < state->stream_count; i++) {
		struct dc_stream_state *stream = state->streams[i];
		struct dc_link *link;

		if (!stream || !stream->link) {
			invalid = true;
			continue;
		}
		link = stream->link;
		if (link == root) {
			target->root_stream_count++;
			target->root_stream = stream;
		} else if (apple5k_root_for_link(link) == root) {
			target->slave_stream_count++;
			target->slave_stream = stream;
			if (link != slave || link->tiled_peer != root ||
			    !dc_link_has_tiled_slave_panel_patch(link))
				invalid = true;
		}
	}

	for (i = 0; i < MAX_PIPES; i++) {
		struct pipe_ctx *pipe = &state->res_ctx.pipe_ctx[i];
		struct dc_link *link;

		if (!pipe->stream || pipe->top_pipe || pipe->prev_odm_pipe)
			continue;
		link = pipe->stream->link;
		if (link == root) {
			target->root_pipe_count++;
			target->root_pipe = pipe;
			if (pipe->stream != target->root_stream)
				invalid = true;
		} else if (apple5k_root_for_link(link) == root) {
			target->slave_pipe_count++;
			target->slave_pipe = pipe;
			if (pipe->stream != target->slave_stream || link != slave ||
			    link->tiled_peer != root)
				invalid = true;
		}
	}

	if (target->root_stream_count > 1 || target->slave_stream_count > 1 ||
	    target->root_pipe_count > 1 || target->slave_pipe_count > 1)
		invalid = true;
	if (target->root_stream_count != target->root_pipe_count ||
	    target->slave_stream_count != target->slave_pipe_count)
		invalid = true;
	if (invalid)
		return;

	if (!target->root_stream_count && !target->slave_stream_count) {
		target->class = APPLE5K_TARGET_NONE;
		return;
	}
	if (target->root_stream_count == 1 && !target->slave_stream_count) {
		target->class = APPLE5K_TARGET_ROOT_ONLY;
		return;
	}
	if (!target->root_stream_count && target->slave_stream_count == 1) {
		target->class = APPLE5K_TARGET_SLAVE_ONLY;
		return;
	}
	if (target->root_stream_count != 1 || target->slave_stream_count != 1 ||
	    !slave || !root->local_sink || !slave->local_sink ||
	    !resource_are_streams_timing_synchronizable(target->root_stream,
						       target->slave_stream) ||
	    target->root_stream->timing.h_addressable != 2560 ||
	    target->root_stream->timing.v_addressable != 2880 ||
	    target->slave_stream->timing.h_addressable != 2560 ||
	    target->slave_stream->timing.v_addressable != 2880)
		return;

	target->class = APPLE5K_TARGET_PAIR;
}

static void apple5k_log_target_snapshot(
		struct dc_link *root, const char *stage,
		const struct apple5k_target_snapshot *target)
{
	if (!root || !target ||
	    (!link_apple5k_log_enabled(root->dc, APPLE5K_LOG_SUMMARY) &&
	     target->class != APPLE5K_TARGET_SLAVE_ONLY &&
	     target->class != APPLE5K_TARGET_INVALID))
		return;

	DC_LOGGER_INIT(root->ctx->logger);
	if (target->class == APPLE5K_TARGET_SLAVE_ONLY ||
	    target->class == APPLE5K_TARGET_INVALID)
		DC_LOG_ERROR("APPLE5K-TARGET stage=%s root[%u] class=%s streams=%u/%u pipes=%u/%u\n",
			     stage ? stage : "?", root->link_index,
			     apple5k_target_class_name(target->class),
			     target->root_stream_count,
			     target->slave_stream_count,
			     target->root_pipe_count,
			     target->slave_pipe_count);
	else
		DC_LOG_INFO("APPLE5K-TARGET stage=%s root[%u] class=%s streams=%u/%u pipes=%u/%u\n",
			    stage ? stage : "?", root->link_index,
			    apple5k_target_class_name(target->class),
			    target->root_stream_count,
			    target->slave_stream_count,
			    target->root_pipe_count,
			    target->slave_pipe_count);
}

static const char *apple5k_zero_stream_reason_name(
		enum apple5k_zero_stream_reason reason)
{
	switch (reason) {
	case APPLE5K_ZERO_STREAM_DPMS: return "DPMS";
	case APPLE5K_ZERO_STREAM_SUSPEND: return "SUSPEND";
	default: return "UNSPECIFIED";
	}
}

static enum dc_status apple5k_prepare_zero_stream(
		struct dc_state *state, struct dc_link *root)
{
	struct apple5k_target_snapshot current_target;
	struct dc_link *slave = root->tiled_peer;
	enum apple5k_zero_stream_reason reason =
		state->apple5k_zero_stream_reason;
	enum apple5k_tx_state tx_state;
	enum apple5k_latch_owner owner;
	enum dc_status status;
	bool block_rearm;
	bool clean;

	if (reason == APPLE5K_ZERO_STREAM_UNSPECIFIED)
		return DC_OK;
	if (!slave || slave->tiled_peer != root)
		return DC_ERROR_UNEXPECTED;

	apple5k_classify_target(root->dc->current_state, root,
				 &current_target);
	apple5k_log_target_snapshot(root, "zero-stream:current",
				    &current_target);
	mutex_lock(&root->apple5k.lock);
	tx_state = root->apple5k.state;
	owner = root->apple5k.latch_owner;
	block_rearm = root->apple5k.block_rearm;
	mutex_unlock(&root->apple5k.lock);

	if (link_apple5k_log_enabled(root->dc, APPLE5K_LOG_SUMMARY)) {
		DC_LOGGER_INIT(root->ctx->logger);
		DC_LOG_INFO("APPLE5K-ZERO reason=%s root[%u] current=%s state=%s owner=%s blocked=%d\n",
			    apple5k_zero_stream_reason_name(reason),
			    root->link_index,
			    apple5k_target_class_name(current_target.class),
			    link_apple5k_state_name(tx_state),
			    link_apple5k_latch_owner_name(owner), block_rearm);
	}

	if (current_target.class == APPLE5K_TARGET_PAIR) {
		/* A prior successful neutralization can precede the atomic off. */
		clean = tx_state == APPLE5K_TX_READY &&
			owner == APPLE5K_LATCH_NONE && !block_rearm &&
			apple5k_pipe_is_blanked(current_target.root_pipe) &&
			apple5k_pipe_is_blanked(current_target.slave_pipe);
		if (clean)
			return DC_OK;

		status = apple5k_neutralize_pair(root, slave,
						 current_target.root_pipe,
						 current_target.slave_pipe,
						 reason == APPLE5K_ZERO_STREAM_SUSPEND ?
						 "suspend:neutralize" :
						 "dpms-off:neutralize");
		return status;
	}

	if (current_target.class != APPLE5K_TARGET_NONE &&
	    current_target.class != APPLE5K_TARGET_ROOT_ONLY)
		return DC_ERROR_UNEXPECTED;

	/* Root-only compat and repeated zero-stream commits own no native latch. */
	clean = (tx_state == APPLE5K_TX_IDLE ||
		 tx_state == APPLE5K_TX_READY) &&
		owner == APPLE5K_LATCH_NONE && !block_rearm;
	if (clean)
		return DC_OK;

	mutex_lock(&root->apple5k.lock);
	root->apple5k.state = APPLE5K_TX_BLOCKED;
	root->apple5k.block_rearm = true;
	mutex_unlock(&root->apple5k.lock);
	return DC_ERROR_UNEXPECTED;
}

static bool apple5k_state_get_timing_pair(struct dc_state *state,
					  struct dc_link *root,
					  struct pipe_ctx **root_pipe_out,
					  struct pipe_ctx **slave_pipe_out)
{
	struct apple5k_target_snapshot target;

	if (root_pipe_out)
		*root_pipe_out = NULL;
	if (slave_pipe_out)
		*slave_pipe_out = NULL;
	apple5k_classify_target(state, root, &target);
	if (target.class != APPLE5K_TARGET_PAIR)
		return false;
	if (root_pipe_out)
		*root_pipe_out = target.root_pipe;
	if (slave_pipe_out)
		*slave_pipe_out = target.slave_pipe;
	return true;
}

/* Pair-wide pre-reset transition.  DPMS/suspend targets with no Apple streams
 * neutralize native explicitly; restart keeps its policy-controlled path.
 */
enum dc_status link_apple_5k_prepare_transition(struct dc *dc,
					       struct dc_state *state)
{
	struct apple5k_target_snapshot target;
	struct apple5k_target_snapshot current_target;
	struct apple5k_panel_mode_status st;
	struct dc_link *root;
	struct dc_link *slave;
	enum apple5k_tx_state tx_state;
	enum dc_status status;
	int i;

	if (!dc || !state || !dc->apple5k_policy.enabled)
		return DC_OK;

	if (system_state == SYSTEM_RESTART && !kexec_in_progress &&
	    state->stream_count == 0) {
		link_apple_5k_prepare_shutdown(dc, state);
		return DC_OK;
	}
	if (dc->apple5k_policy.pair_mode == APPLE5K_PAIR_LEGACY)
		return DC_OK;

	for (i = 0; i < dc->link_count; i++) {
		root = dc->links[i];
		if (!dc_link_has_tiled_root_panel_patch(root))
			continue;
		apple5k_classify_target(state, root, &target);
		apple5k_log_target_snapshot(root, "prepare-transition:target",
					    &target);
		if (target.class == APPLE5K_TARGET_NONE) {
			status = apple5k_prepare_zero_stream(state, root);
			if (status != DC_OK)
				return status;
			continue;
		}
		if (target.class == APPLE5K_TARGET_PAIR)
			continue;
		if (target.class != APPLE5K_TARGET_ROOT_ONLY ||
		    !root->tiled_peer)
			return DC_ERROR_UNEXPECTED;
		if (dc->apple5k_policy.pair_mode !=
		    APPLE5K_PAIR_TRANSACTIONAL)
			continue;
		slave = root->tiled_peer;
		mutex_lock(&root->apple5k.lock);
		tx_state = root->apple5k.state;
		if (tx_state != APPLE5K_TX_NATIVE &&
		    tx_state != APPLE5K_TX_DISCOVERING &&
		    tx_state != APPLE5K_TX_ENABLING) {
			mutex_unlock(&root->apple5k.lock);
			continue;
		}
		root->apple5k.state = APPLE5K_TX_QUIESCING;
		root->apple5k.latch_owner = APPLE5K_LATCH_TEARDOWN;
		root->apple5k.block_rearm = true;
		mutex_unlock(&root->apple5k.lock);

		apple5k_classify_target(dc->current_state, root, &current_target);
		apple5k_log_target_snapshot(root, "prepare-transition:current",
					    &current_target);
		if (current_target.class == APPLE5K_TARGET_PAIR) {
			dc->hwss.blank_stream(current_target.root_pipe);
			dc->hwss.blank_stream(current_target.slave_pipe);
			if (!apple5k_pipe_is_blanked(current_target.root_pipe) ||
			    !apple5k_pipe_is_blanked(current_target.slave_pipe))
				return DC_ERROR_UNEXPECTED;
		} else if (current_target.class == APPLE5K_TARGET_ROOT_ONLY) {
			/* A discovery-only root can be balanced after its one source
			 * path is quiet.  Native/enable ownership must never be reduced
			 * to a per-link clear.
			 */
			if (tx_state != APPLE5K_TX_DISCOVERING ||
			    !current_target.root_pipe)
				return DC_ERROR_UNEXPECTED;
			dc->hwss.blank_stream(current_target.root_pipe);
			if (!apple5k_pipe_is_blanked(current_target.root_pipe))
				return DC_ERROR_UNEXPECTED;
		} else if (current_target.class != APPLE5K_TARGET_NONE) {
			return DC_ERROR_UNEXPECTED;
		}

		root->link_state_valid = false;
		slave->link_state_valid = false;

		status = apple5k_write_root_latch(root, 0,
						 "native-to-compat:disarm");
		msleep(10);
		if (status == DC_OK &&
		    apple5k_read_panel_mode_status(root, &st) &&
		    apple5k_panel_status_is_base(&st)) {
			mutex_lock(&root->apple5k.lock);
			root->apple5k.state = APPLE5K_TX_READY;
			root->apple5k.latch_owner = APPLE5K_LATCH_NONE;
			root->apple5k.block_rearm = false;
			mutex_unlock(&root->apple5k.lock);
			return DC_OK;
		}

		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_BLOCKED;
		mutex_unlock(&root->apple5k.lock);
		return DC_ERROR_UNEXPECTED;
	}

	return DC_OK;
}

static bool apple5k_should_defer_pair_finalizer(struct dc_state *state,
						struct pipe_ctx *pipe_ctx)
{
	struct pipe_ctx *root_pipe = NULL;
	struct pipe_ctx *slave_pipe = NULL;
	struct dc_link *root;
	enum apple5k_tx_state tx_state;

	if (!pipe_ctx || !pipe_ctx->stream ||
	    !pipe_ctx->stream->link->dc->apple5k_policy.enabled ||
	    pipe_ctx->stream->link->dc->apple5k_policy.pair_mode ==
						APPLE5K_PAIR_LEGACY ||
	    !apple5k_pipe_is_tile(pipe_ctx))
		return false;

	root = apple5k_root_for_link(pipe_ctx->stream->link);
	if (!apple5k_state_get_timing_pair(state, root, &root_pipe, &slave_pipe))
		return false;
	tx_state = apple5k_get_state(root);
	if (root->dc->apple5k_policy.pair_mode ==
					APPLE5K_PAIR_TRANSACTIONAL &&
	    tx_state != APPLE5K_TX_ENABLING &&
	    tx_state != APPLE5K_TX_NATIVE)
		return false;

	/*
	 * Partial slave retrains are stable on the legacy path; only hold both
	 * tiles for the GSL drain when the pair is actually blanked together.
	 */
	return apple5k_pipe_is_blanked(root_pipe) &&
	       apple5k_pipe_is_blanked(slave_pipe);
}

static void apple5k_latch_stream(struct dc_link *link)
{
	dp_write_tiled_stream_enable_latch(link);
}

void link_apple_5k_prepare_shutdown(struct dc *dc,
				    struct dc_state *new_state)
{
	struct apple5k_target_snapshot current_target;
	struct pipe_ctx *root_pipe = NULL;
	struct pipe_ctx *slave_pipe = NULL;
	struct apple5k_panel_mode_status st;
	struct dc_link *root = NULL;
	enum apple5k_shutdown_mode mode;
	enum dc_status status;
	int i;

	if (!dc || !new_state || !dc->apple5k_policy.enabled ||
	    system_state != SYSTEM_RESTART || kexec_in_progress ||
	    new_state->stream_count != 0)
		return;

	for (i = 0; i < dc->link_count; i++) {
		struct dc_link *candidate = dc->links[i];

		if (!dc_link_has_tiled_root_panel_patch(candidate))
			continue;
		apple5k_classify_target(dc->current_state, candidate,
					 &current_target);
		apple5k_log_target_snapshot(candidate, "shutdown:current",
					    &current_target);
		if (current_target.class == APPLE5K_TARGET_NONE ||
		    current_target.class == APPLE5K_TARGET_ROOT_ONLY)
			continue;
		if (current_target.class != APPLE5K_TARGET_PAIR)
			return;
		root = candidate;
		root_pipe = current_target.root_pipe;
		slave_pipe = current_target.slave_pipe;
		break;
	}
	if (!root || !root_pipe || !slave_pipe)
		return;

	mode = dc->apple5k_policy.shutdown_mode;
	mutex_lock(&root->apple5k.lock);
	root->apple5k.block_rearm = true;
	if (mode != APPLE5K_SHUTDOWN_STOCK &&
	    mode != APPLE5K_SHUTDOWN_OBSERVE)
		root->apple5k.state = APPLE5K_TX_QUIESCING;
	if (mode == APPLE5K_SHUTDOWN_NEUTRALIZE)
		root->apple5k.latch_owner = APPLE5K_LATCH_TEARDOWN;
	mutex_unlock(&root->apple5k.lock);

	link_apple_5k_log_panel_mode(root, "restart:entry");
	if (mode == APPLE5K_SHUTDOWN_STOCK ||
	    mode == APPLE5K_SHUTDOWN_OBSERVE)
		return;

	dc->hwss.blank_stream(root_pipe);
	dc->hwss.blank_stream(slave_pipe);
	if (!apple5k_pipe_is_blanked(root_pipe) ||
	    !apple5k_pipe_is_blanked(slave_pipe)) {
		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_BLOCKED;
		mutex_unlock(&root->apple5k.lock);
		return;
	}

	if (mode != APPLE5K_SHUTDOWN_NEUTRALIZE)
		return;

	status = apple5k_write_root_latch(root, 0,
					 "restart:neutralize");
	msleep(10);
	if (status == DC_OK && apple5k_read_panel_mode_status(root, &st) &&
	    apple5k_panel_status_is_base(&st)) {
		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_READY;
		root->apple5k.latch_owner = APPLE5K_LATCH_NONE;
		mutex_unlock(&root->apple5k.lock);
	} else {
		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_BLOCKED;
		mutex_unlock(&root->apple5k.lock);
	}
}

enum dc_status link_apple_5k_prepare_tiled_pair(struct dc *dc,
					       struct dc_state *state)
{
	struct apple5k_target_snapshot target;
	struct pipe_ctx *root_pipe;
	struct pipe_ctx *slave_pipe;
	struct dc_link *root;
	struct dc_link *slave;
	bool root_blank;
	bool slave_blank;
	enum apple5k_tx_state tx_state;
	int i;

	if (!dc || !state || !dc->apple5k_policy.enabled ||
	    dc->apple5k_policy.pair_mode != APPLE5K_PAIR_TRANSACTIONAL)
		return DC_OK;

	for (i = 0; i < dc->link_count; i++) {
		root = dc->links[i];
		if (!dc_link_has_tiled_root_panel_patch(root))
			continue;
		apple5k_classify_target(state, root, &target);
		apple5k_log_target_snapshot(root, "prepare-pair:target", &target);
		if (target.class == APPLE5K_TARGET_NONE ||
		    target.class == APPLE5K_TARGET_ROOT_ONLY)
			continue;
		if (target.class != APPLE5K_TARGET_PAIR)
			return DC_ERROR_UNEXPECTED;
		root_pipe = target.root_pipe;
		slave_pipe = target.slave_pipe;
		slave = slave_pipe->stream->link;
		root_blank = apple5k_pipe_is_blanked(root_pipe);
		slave_blank = apple5k_pipe_is_blanked(slave_pipe);
		tx_state = apple5k_get_state(root);
		if (!root_blank || !slave_blank) {
			/* An unchanged live native pair needs no reconstruction. */
			if (tx_state == APPLE5K_TX_NATIVE &&
			    !root_blank && !slave_blank &&
			    root->link_state_valid && slave->link_state_valid)
				return DC_OK;

			/* A half-live pair is never a valid transaction boundary. */
			if (root_blank != slave_blank)
				return DC_ERROR_UNEXPECTED;

			/* Quiesce an inherited GOP pair together before clearing it. */
			dc->hwss.blank_stream(root_pipe);
			dc->hwss.blank_stream(slave_pipe);
			if (!apple5k_pipe_is_blanked(root_pipe) ||
			    !apple5k_pipe_is_blanked(slave_pipe))
				return DC_ERROR_UNEXPECTED;
		}

		/* A new transaction must not reuse firmware/half-programmed links. */
		root->link_state_valid = false;
		slave->link_state_valid = false;
		slave->apple5k_source_table_ok = false;
		return apple5k_begin_native_enable(root, slave, root_pipe,
						   slave_pipe) ?
					DC_OK : DC_ERROR_UNEXPECTED;
	}

	return DC_OK;
}

enum dc_status link_apple_5k_finalize_state(struct dc *dc,
					    struct dc_state *state)
{
	struct apple5k_target_snapshot target;
	struct pipe_ctx *root_pipe = NULL;
	struct pipe_ctx *slave_pipe = NULL;
	struct dc_link *root_link;
	struct dc_link *slave_link;
	bool root_blank;
	bool slave_blank;
	bool transaction_started;
	enum apple5k_tx_state tx_state;
	struct dal_logger *dc_logger;
	u64 start_ns;
	u64 mid_ns;
	u64 end_ns;
	int i;

	if (!dc || !state || !dc->apple5k_policy.enabled ||
	    dc->apple5k_policy.pair_mode == APPLE5K_PAIR_LEGACY)
		return DC_OK;

	for (i = 0; i < dc->link_count; i++) {
		struct dc_link *candidate = dc->links[i];

		if (!dc_link_has_tiled_root_panel_patch(candidate))
			continue;
		apple5k_classify_target(state, candidate, &target);
		apple5k_log_target_snapshot(candidate, "finalize:target", &target);
		if (target.class == APPLE5K_TARGET_NONE ||
		    target.class == APPLE5K_TARGET_ROOT_ONLY)
			continue;
		if (target.class != APPLE5K_TARGET_PAIR)
			return DC_ERROR_UNEXPECTED;
		root_pipe = target.root_pipe;
		slave_pipe = target.slave_pipe;
		break;
	}
	if (!root_pipe || !slave_pipe)
		return DC_OK;

	root_link = root_pipe->stream->link;
	slave_link = slave_pipe->stream->link;
	root_blank = apple5k_pipe_is_blanked(root_pipe);
	slave_blank = apple5k_pipe_is_blanked(slave_pipe);
	tx_state = apple5k_get_state(root_link);
	dc_logger = root_link->ctx->logger;

	if (!root_blank || !slave_blank) {
		if (tx_state == APPLE5K_TX_NATIVE)
			return DC_OK;
		DC_LOG_INFO("APPLE5K-UNBLANK pair-finalizer skip root_link[%u] slave_link[%u] root_blank=%d slave_blank=%d reason=not-deferred-pair\n",
			    root_link->link_index, slave_link->link_index,
			    root_blank, slave_blank);
		return DC_ERROR_UNEXPECTED;
	}

	if (dc->apple5k_policy.pair_mode == APPLE5K_PAIR_COORDINATED)
		transaction_started = apple5k_begin_native_enable(root_link,
							 slave_link,
							 root_pipe,
							 slave_pipe);
	else
		transaction_started =
			apple5k_get_state(root_link) == APPLE5K_TX_ENABLING;
	DC_LOG_INFO("APPLE5K-TXN pair-finalizer begin root_link[%u] slave_link[%u] started=%d\n",
		    root_link->link_index, slave_link->link_index,
		    transaction_started);

	if (link_apple5k_log_enabled(dc, APPLE5K_LOG_TIMING))
		DC_LOG_INFO("APPLE5K-UNBLANK pair-finalizer start root_link[%u] slave_link[%u] root_tg=%u slave_tg=%u root_blank=%d slave_blank=%d\n",
			    root_link->link_index, slave_link->link_index,
			    root_pipe->stream_res.tg->inst,
			    slave_pipe->stream_res.tg->inst,
			    root_blank, slave_blank);

	if (!transaction_started)
		return DC_ERROR_UNEXPECTED;

	start_ns = ktime_get_ns();
	if (dc->apple5k_policy.pair_order == APPLE5K_ORDER_SLAVE_FIRST) {
		dc->hwss.unblank_stream(slave_pipe,
			&slave_pipe->stream->link->cur_link_settings);
		mid_ns = ktime_get_ns();
		dc->hwss.unblank_stream(root_pipe,
			&root_pipe->stream->link->cur_link_settings);
	} else {
		dc->hwss.unblank_stream(root_pipe,
			&root_pipe->stream->link->cur_link_settings);
		mid_ns = ktime_get_ns();
		dc->hwss.unblank_stream(slave_pipe,
			&slave_pipe->stream->link->cur_link_settings);
	}
	end_ns = ktime_get_ns();

	if (link_apple5k_log_enabled(dc, APPLE5K_LOG_TIMING))
		DC_LOG_INFO("APPLE5K-UNBLANK pair-finalizer done root_link[%u] slave_link[%u] gap_ns=%llu total_ns=%llu root_blank_after=%d slave_blank_after=%d\n",
			    root_link->link_index, slave_link->link_index,
			    (unsigned long long)(mid_ns - start_ns),
			    (unsigned long long)(end_ns - start_ns),
			    apple5k_pipe_is_blanked(root_pipe),
			    apple5k_pipe_is_blanked(slave_pipe));

	if (root_pipe->stream->sink_patches.delay_ignore_msa > 0)
		msleep(root_pipe->stream->sink_patches.delay_ignore_msa);
	if (slave_pipe->stream->sink_patches.delay_ignore_msa > 0)
		msleep(slave_pipe->stream->sink_patches.delay_ignore_msa);

	apple5k_latch_stream(root_link);
	apple5k_latch_stream(slave_link);
	return apple5k_finish_native_enable(root_link, slave_link,
					    root_pipe, slave_pipe,
					    transaction_started) ?
		DC_OK : DC_ERROR_UNEXPECTED;
}

static bool get_ext_hdmi_settings(struct pipe_ctx *pipe_ctx,
		enum engine_id eng_id,
		struct ext_hdmi_settings *settings)
{
	bool result = false;
	int i = 0;
	struct integrated_info *integrated_info =
			pipe_ctx->stream->ctx->dc_bios->integrated_info;

	if (integrated_info == NULL)
		return false;

	/*
	 * Get retimer settings from sbios for passing SI eye test for DCE11
	 * The setting values are varied based on board revision and port id
	 * Therefore the setting values of each ports is passed by sbios.
	 */

	// Check if current bios contains ext Hdmi settings
	if (integrated_info->gpu_cap_info & 0x20) {
		switch (eng_id) {
		case ENGINE_ID_DIGA:
			settings->slv_addr = integrated_info->dp0_ext_hdmi_slv_addr;
			settings->reg_num = integrated_info->dp0_ext_hdmi_6g_reg_num;
			settings->reg_num_6g = integrated_info->dp0_ext_hdmi_6g_reg_num;
			memmove(settings->reg_settings,
					integrated_info->dp0_ext_hdmi_reg_settings,
					sizeof(integrated_info->dp0_ext_hdmi_reg_settings));
			memmove(settings->reg_settings_6g,
					integrated_info->dp0_ext_hdmi_6g_reg_settings,
					sizeof(integrated_info->dp0_ext_hdmi_6g_reg_settings));
			result = true;
			break;
		case ENGINE_ID_DIGB:
			settings->slv_addr = integrated_info->dp1_ext_hdmi_slv_addr;
			settings->reg_num = integrated_info->dp1_ext_hdmi_6g_reg_num;
			settings->reg_num_6g = integrated_info->dp1_ext_hdmi_6g_reg_num;
			memmove(settings->reg_settings,
					integrated_info->dp1_ext_hdmi_reg_settings,
					sizeof(integrated_info->dp1_ext_hdmi_reg_settings));
			memmove(settings->reg_settings_6g,
					integrated_info->dp1_ext_hdmi_6g_reg_settings,
					sizeof(integrated_info->dp1_ext_hdmi_6g_reg_settings));
			result = true;
			break;
		case ENGINE_ID_DIGC:
			settings->slv_addr = integrated_info->dp2_ext_hdmi_slv_addr;
			settings->reg_num = integrated_info->dp2_ext_hdmi_6g_reg_num;
			settings->reg_num_6g = integrated_info->dp2_ext_hdmi_6g_reg_num;
			memmove(settings->reg_settings,
					integrated_info->dp2_ext_hdmi_reg_settings,
					sizeof(integrated_info->dp2_ext_hdmi_reg_settings));
			memmove(settings->reg_settings_6g,
					integrated_info->dp2_ext_hdmi_6g_reg_settings,
					sizeof(integrated_info->dp2_ext_hdmi_6g_reg_settings));
			result = true;
			break;
		case ENGINE_ID_DIGD:
			settings->slv_addr = integrated_info->dp3_ext_hdmi_slv_addr;
			settings->reg_num = integrated_info->dp3_ext_hdmi_6g_reg_num;
			settings->reg_num_6g = integrated_info->dp3_ext_hdmi_6g_reg_num;
			memmove(settings->reg_settings,
					integrated_info->dp3_ext_hdmi_reg_settings,
					sizeof(integrated_info->dp3_ext_hdmi_reg_settings));
			memmove(settings->reg_settings_6g,
					integrated_info->dp3_ext_hdmi_6g_reg_settings,
					sizeof(integrated_info->dp3_ext_hdmi_6g_reg_settings));
			result = true;
			break;
		default:
			break;
		}

		if (result == true) {
			// Validate settings from bios integrated info table
			if (settings->slv_addr == 0)
				return false;
			if (settings->reg_num > 9)
				return false;
			if (settings->reg_num_6g > 3)
				return false;

			for (i = 0; i < settings->reg_num; i++) {
				if (settings->reg_settings[i].i2c_reg_index > 0x20)
					return false;
			}

			for (i = 0; i < settings->reg_num_6g; i++) {
				if (settings->reg_settings_6g[i].i2c_reg_index > 0x20)
					return false;
			}
		}
	}

	return result;
}

static bool write_i2c(struct pipe_ctx *pipe_ctx,
		uint8_t address, uint8_t *buffer, uint32_t length)
{
	struct i2c_command cmd = {0};
	struct i2c_payload payload = {0};

	memset(&payload, 0, sizeof(payload));
	memset(&cmd, 0, sizeof(cmd));

	cmd.number_of_payloads = 1;
	cmd.engine = I2C_COMMAND_ENGINE_DEFAULT;
	cmd.speed = pipe_ctx->stream->ctx->dc->caps.i2c_speed_in_khz;

	payload.address = address;
	payload.data = buffer;
	payload.length = length;
	payload.write = true;
	cmd.payloads = &payload;

	if (dm_helpers_submit_i2c(pipe_ctx->stream->ctx,
			pipe_ctx->stream->link, &cmd))
		return true;

	return false;
}

static void write_i2c_retimer_setting(
		struct pipe_ctx *pipe_ctx,
		bool is_vga_mode,
		bool is_over_340mhz,
		struct ext_hdmi_settings *settings)
{
	uint8_t slave_address = (settings->slv_addr >> 1);
	uint8_t buffer[2];
	const uint8_t apply_rx_tx_change = 0x4;
	uint8_t offset = 0xA;
	uint8_t value = 0;
	int i = 0;
	bool i2c_success = false;
	DC_LOGGER_INIT(pipe_ctx->stream->ctx->logger);

	memset(&buffer, 0, sizeof(buffer));

	/* Start Ext-Hdmi programming*/

	for (i = 0; i < settings->reg_num; i++) {
		/* Apply 3G settings */
		if (settings->reg_settings[i].i2c_reg_index <= 0x20) {

			buffer[0] = settings->reg_settings[i].i2c_reg_index;
			buffer[1] = settings->reg_settings[i].i2c_reg_val;
			i2c_success = write_i2c(pipe_ctx, slave_address,
						buffer, sizeof(buffer));
			RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
				offset = 0x%x, reg_val= 0x%x, i2c_success = %d\n",
				slave_address, buffer[0], buffer[1], i2c_success?1:0);

			if (!i2c_success)
				goto i2c_write_fail;

			/* Based on DP159 specs, APPLY_RX_TX_CHANGE bit in 0x0A
			 * needs to be set to 1 on every 0xA-0xC write.
			 */
			if (settings->reg_settings[i].i2c_reg_index == 0xA ||
				settings->reg_settings[i].i2c_reg_index == 0xB ||
				settings->reg_settings[i].i2c_reg_index == 0xC) {

				/* Query current value from offset 0xA */
				if (settings->reg_settings[i].i2c_reg_index == 0xA)
					value = settings->reg_settings[i].i2c_reg_val;
				else {
					i2c_success =
						link_query_ddc_data(
						pipe_ctx->stream->link->ddc,
						slave_address, &offset, 1, &value, 1);
					if (!i2c_success)
						goto i2c_write_fail;
				}

				buffer[0] = offset;
				/* Set APPLY_RX_TX_CHANGE bit to 1 */
				buffer[1] = value | apply_rx_tx_change;
				i2c_success = write_i2c(pipe_ctx, slave_address,
						buffer, sizeof(buffer));
				RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
					offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
					slave_address, buffer[0], buffer[1], i2c_success?1:0);
				if (!i2c_success)
					goto i2c_write_fail;
			}
		}
	}

	/* Apply 3G settings */
	if (is_over_340mhz) {
		for (i = 0; i < settings->reg_num_6g; i++) {
			/* Apply 3G settings */
			if (settings->reg_settings[i].i2c_reg_index <= 0x20) {

				buffer[0] = settings->reg_settings_6g[i].i2c_reg_index;
				buffer[1] = settings->reg_settings_6g[i].i2c_reg_val;
				i2c_success = write_i2c(pipe_ctx, slave_address,
							buffer, sizeof(buffer));
				RETIMER_REDRIVER_INFO("above 340Mhz: retimer write to slave_address = 0x%x,\
					offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
					slave_address, buffer[0], buffer[1], i2c_success?1:0);

				if (!i2c_success)
					goto i2c_write_fail;

				/* Based on DP159 specs, APPLY_RX_TX_CHANGE bit in 0x0A
				 * needs to be set to 1 on every 0xA-0xC write.
				 */
				if (settings->reg_settings_6g[i].i2c_reg_index == 0xA ||
					settings->reg_settings_6g[i].i2c_reg_index == 0xB ||
					settings->reg_settings_6g[i].i2c_reg_index == 0xC) {

					/* Query current value from offset 0xA */
					if (settings->reg_settings_6g[i].i2c_reg_index == 0xA)
						value = settings->reg_settings_6g[i].i2c_reg_val;
					else {
						i2c_success =
								link_query_ddc_data(
								pipe_ctx->stream->link->ddc,
								slave_address, &offset, 1, &value, 1);
						if (!i2c_success)
							goto i2c_write_fail;
					}

					buffer[0] = offset;
					/* Set APPLY_RX_TX_CHANGE bit to 1 */
					buffer[1] = value | apply_rx_tx_change;
					i2c_success = write_i2c(pipe_ctx, slave_address,
							buffer, sizeof(buffer));
					RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
						offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
						slave_address, buffer[0], buffer[1], i2c_success?1:0);
					if (!i2c_success)
						goto i2c_write_fail;
				}
			}
		}
	}

	if (is_vga_mode) {
		/* Program additional settings if using 640x480 resolution */

		/* Write offset 0xFF to 0x01 */
		buffer[0] = 0xff;
		buffer[1] = 0x01;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
				offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
				slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;

		/* Write offset 0x00 to 0x23 */
		buffer[0] = 0x00;
		buffer[1] = 0x23;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
			offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
			slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;

		/* Write offset 0xff to 0x00 */
		buffer[0] = 0xff;
		buffer[1] = 0x00;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write to slave_address = 0x%x,\
			offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
			slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;

	}

	return;

i2c_write_fail:
	DC_LOG_DEBUG("Set retimer failed");
}

static void write_i2c_default_retimer_setting(
		struct pipe_ctx *pipe_ctx,
		bool is_vga_mode,
		bool is_over_340mhz)
{
	uint8_t slave_address = (0xBA >> 1);
	uint8_t buffer[2];
	bool i2c_success = false;
	DC_LOGGER_INIT(pipe_ctx->stream->ctx->logger);

	memset(&buffer, 0, sizeof(buffer));

	/* Program Slave Address for tuning single integrity */
	/* Write offset 0x0A to 0x13 */
	buffer[0] = 0x0A;
	buffer[1] = 0x13;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer writes default setting to slave_address = 0x%x,\
		offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;

	/* Write offset 0x0A to 0x17 */
	buffer[0] = 0x0A;
	buffer[1] = 0x17;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
		offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;

	/* Write offset 0x0B to 0xDA or 0xD8 */
	buffer[0] = 0x0B;
	buffer[1] = is_over_340mhz ? 0xDA : 0xD8;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
		offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;

	/* Write offset 0x0A to 0x17 */
	buffer[0] = 0x0A;
	buffer[1] = 0x17;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
		offset = 0x%x, reg_val= 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;

	/* Write offset 0x0C to 0x1D or 0x91 */
	buffer[0] = 0x0C;
	buffer[1] = is_over_340mhz ? 0x1D : 0x91;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
		offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;

	/* Write offset 0x0A to 0x17 */
	buffer[0] = 0x0A;
	buffer[1] = 0x17;
	i2c_success = write_i2c(pipe_ctx, slave_address,
			buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
		offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
		slave_address, buffer[0], buffer[1], i2c_success?1:0);
	if (!i2c_success)
		goto i2c_write_fail;


	if (is_vga_mode) {
		/* Program additional settings if using 640x480 resolution */

		/* Write offset 0xFF to 0x01 */
		buffer[0] = 0xff;
		buffer[1] = 0x01;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
			offset = 0x%x, reg_val = 0x%x, i2c_success = %d\n",
			slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;

		/* Write offset 0x00 to 0x23 */
		buffer[0] = 0x00;
		buffer[1] = 0x23;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write to slave_addr = 0x%x,\
			offset = 0x%x, reg_val= 0x%x, i2c_success = %d\n",
			slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;

		/* Write offset 0xff to 0x00 */
		buffer[0] = 0xff;
		buffer[1] = 0x00;
		i2c_success = write_i2c(pipe_ctx, slave_address,
				buffer, sizeof(buffer));
		RETIMER_REDRIVER_INFO("retimer write default setting to slave_addr = 0x%x,\
			offset = 0x%x, reg_val= 0x%x, i2c_success = %d end here\n",
			slave_address, buffer[0], buffer[1], i2c_success?1:0);
		if (!i2c_success)
			goto i2c_write_fail;
	}

	return;

i2c_write_fail:
	DC_LOG_DEBUG("Set default retimer failed");
}

static void write_i2c_redriver_setting(
		struct pipe_ctx *pipe_ctx,
		bool is_over_340mhz)
{
	uint8_t slave_address = (0xF0 >> 1);
	uint8_t buffer[16];
	bool i2c_success = false;
	DC_LOGGER_INIT(pipe_ctx->stream->ctx->logger);

	memset(&buffer, 0, sizeof(buffer));

	// Program Slave Address for tuning single integrity
	buffer[3] = 0x4E;
	buffer[4] = 0x4E;
	buffer[5] = 0x4E;
	buffer[6] = is_over_340mhz ? 0x4E : 0x4A;

	i2c_success = write_i2c(pipe_ctx, slave_address,
					buffer, sizeof(buffer));
	RETIMER_REDRIVER_INFO("redriver write 0 to all 16 reg offset expect following:\n\
		\t slave_addr = 0x%x, offset[3] = 0x%x, offset[4] = 0x%x,\
		offset[5] = 0x%x,offset[6] is_over_340mhz = 0x%x,\
		i2c_success = %d\n",
		slave_address, buffer[3], buffer[4], buffer[5], buffer[6], i2c_success?1:0);

	if (!i2c_success)
		DC_LOG_DEBUG("Set redriver failed");
}

static void update_psp_stream_config(struct pipe_ctx *pipe_ctx, bool dpms_off)
{
	struct cp_psp *cp_psp = &pipe_ctx->stream->ctx->cp_psp;
	struct link_encoder *link_enc = pipe_ctx->link_res.dio_link_enc;
	struct cp_psp_stream_config config = {0};
	enum dp_panel_mode panel_mode =
			dp_get_panel_mode(pipe_ctx->stream->link);

	if (cp_psp == NULL || cp_psp->funcs.update_stream_config == NULL)
		return;
	if (!pipe_ctx->stream->ctx->dc->config.unify_link_enc_assignment)
		link_enc = link_enc_cfg_get_link_enc(pipe_ctx->stream->link);
	ASSERT(link_enc);
	if (link_enc == NULL)
		return;

	/* otg instance */
	config.otg_inst = (uint8_t) pipe_ctx->stream_res.tg->inst;

	/* dig front end */
	config.dig_fe = (uint8_t) pipe_ctx->stream_res.stream_enc->stream_enc_inst;

	/* stream encoder index */
	config.stream_enc_idx = pipe_ctx->stream_res.stream_enc->id - ENGINE_ID_DIGA;
	if (dp_is_128b_132b_signal(pipe_ctx))
		config.stream_enc_idx =
				pipe_ctx->stream_res.hpo_dp_stream_enc->id - ENGINE_ID_HPO_DP_0;

	/* dig back end */
	config.dig_be = pipe_ctx->stream->link->link_enc_hw_inst;

	/* link encoder index */
	config.link_enc_idx = link_enc->transmitter - TRANSMITTER_UNIPHY_A;
	if (dp_is_128b_132b_signal(pipe_ctx))
		config.link_enc_idx = pipe_ctx->link_res.hpo_dp_link_enc->inst;

	/* dio output index is dpia index for DPIA endpoint & dcio index by default */
	if (pipe_ctx->stream->link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA)
		config.dio_output_idx = pipe_ctx->stream->link->link_id.enum_id - ENUM_ID_1;
	else
		config.dio_output_idx = link_enc->transmitter - TRANSMITTER_UNIPHY_A;


	/* phy index */
	config.phy_idx = resource_transmitter_to_phy_idx(
			pipe_ctx->stream->link->dc, link_enc->transmitter);
	if (pipe_ctx->stream->link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA)
		/* USB4 DPIA doesn't use PHY in our soc, initialize it to 0 */
		config.phy_idx = 0;

	/* stream properties */
	config.assr_enabled = (panel_mode == DP_PANEL_MODE_EDP) ? 1 : 0;
	config.mst_enabled = (pipe_ctx->stream->signal ==
			SIGNAL_TYPE_DISPLAY_PORT_MST) ? 1 : 0;
	config.dp2_enabled = dp_is_128b_132b_signal(pipe_ctx) ? 1 : 0;
	config.usb4_enabled = (pipe_ctx->stream->link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA) ?
			1 : 0;
	config.dpms_off = dpms_off;

	/* dm stream context */
	config.dm_stream_ctx = pipe_ctx->stream->dm_stream_context;

	cp_psp->funcs.update_stream_config(cp_psp->handle, &config);
}

static void set_avmute(struct pipe_ctx *pipe_ctx, bool enable)
{
	struct dc  *dc = pipe_ctx->stream->ctx->dc;

	if (!dc_is_hdmi_signal(pipe_ctx->stream->signal))
		return;

	dc->hwss.set_avmute(pipe_ctx, enable);
}

static void enable_mst_on_sink(struct dc_link *link, bool enable)
{
	unsigned char mstmCntl = 0;

	core_link_read_dpcd(link, DP_MSTM_CTRL, &mstmCntl, 1);
	if (enable)
		mstmCntl |= DP_MST_EN;
	else
		mstmCntl &= (~DP_MST_EN);

	core_link_write_dpcd(link, DP_MSTM_CTRL, &mstmCntl, 1);
}

static void dsc_optc_config_log(struct display_stream_compressor *dsc,
		struct dsc_optc_config *config)
{
	uint32_t precision = 1 << 28;
	uint32_t bytes_per_pixel_int = config->bytes_per_pixel / precision;
	uint32_t bytes_per_pixel_mod = config->bytes_per_pixel % precision;
	uint64_t ll_bytes_per_pix_fraq = bytes_per_pixel_mod;
	DC_LOGGER_INIT(dsc->ctx->logger);

	/* 7 fractional digits decimal precision for bytes per pixel is enough because DSC
	 * bits per pixel precision is 1/16th of a pixel, which means bytes per pixel precision is
	 * 1/16/8 = 1/128 of a byte, or 0.0078125 decimal
	 */
	ll_bytes_per_pix_fraq *= 10000000;
	ll_bytes_per_pix_fraq /= precision;

	DC_LOG_DSC("\tbytes_per_pixel 0x%08x (%d.%07d)",
			config->bytes_per_pixel, bytes_per_pixel_int, (uint32_t)ll_bytes_per_pix_fraq);
	DC_LOG_DSC("\tis_pixel_format_444 %d", config->is_pixel_format_444);
	DC_LOG_DSC("\tslice_width %d", config->slice_width);
}

static bool dp_set_dsc_on_rx(struct pipe_ctx *pipe_ctx, bool enable)
{
	struct dc *dc = pipe_ctx->stream->ctx->dc;
	struct dc_stream_state *stream = pipe_ctx->stream;
	bool result = false;

	if (dc_is_virtual_signal(stream->signal))
		result = true;
	else
		result = dm_helpers_dp_write_dsc_enable(dc->ctx, stream, enable);
	return result;
}

static bool dp_set_hblank_reduction_on_rx(struct pipe_ctx *pipe_ctx)
{
	struct dc *dc = pipe_ctx->stream->ctx->dc;
	struct dc_stream_state *stream = pipe_ctx->stream;
	bool result = false;

	if (dc_is_virtual_signal(stream->signal))
		result = true;
	else
		result = dm_helpers_dp_write_hblank_reduction(dc->ctx, stream);
	return result;
}


/* The stream with these settings can be sent (unblanked) only after DSC was enabled on RX first,
 * i.e. after dp_enable_dsc_on_rx() had been called
 */
void link_set_dsc_on_stream(struct pipe_ctx *pipe_ctx, bool enable)
{
	/* TODO: Move this to HWSS as this is hardware programming sequence not a
	 * link layer sequence
	 */
	struct display_stream_compressor *dsc = pipe_ctx->stream_res.dsc;
	struct dc *dc = pipe_ctx->stream->ctx->dc;
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct pipe_ctx *odm_pipe;
	int opp_cnt = 1;
	struct dccg *dccg = dc->res_pool->dccg;
	/* It has been found that when DSCCLK is lower than 16Mhz, we will get DCN
	 * register access hung. When DSCCLk is based on refclk, DSCCLk is always a
	 * fixed value higher than 16Mhz so the issue doesn't occur. When DSCCLK is
	 * generated by DTO, DSCCLK would be based on 1/3 dispclk. For small timings
	 * with DSC such as 480p60Hz, the dispclk could be low enough to trigger
	 * this problem. We are implementing a workaround here to keep using dscclk
	 * based on fixed value refclk when timing is smaller than 3x16Mhz (i.e
	 * 48Mhz) pixel clock to avoid hitting this problem.
	 */
	bool should_use_dto_dscclk = (dccg->funcs->set_dto_dscclk != NULL) &&
			stream->timing.pix_clk_100hz > 480000;
	DC_LOGGER_INIT(dsc->ctx->logger);

	for (odm_pipe = pipe_ctx->next_odm_pipe; odm_pipe; odm_pipe = odm_pipe->next_odm_pipe)
		opp_cnt++;

	if (enable) {
		struct dsc_config dsc_cfg;
		struct dsc_optc_config dsc_optc_cfg = {0};
		enum optc_dsc_mode optc_dsc_mode;

		/* Enable DSC hw block */
		dsc_cfg.pic_width = (stream->timing.h_addressable + pipe_ctx->dsc_padding_params.dsc_hactive_padding +
				stream->timing.h_border_left + stream->timing.h_border_right) / opp_cnt;
		dsc_cfg.pic_height = stream->timing.v_addressable + stream->timing.v_border_top + stream->timing.v_border_bottom;
		dsc_cfg.pixel_encoding = stream->timing.pixel_encoding;
		dsc_cfg.color_depth = stream->timing.display_color_depth;
		dsc_cfg.is_odm = pipe_ctx->next_odm_pipe ? true : false;
		dsc_cfg.dc_dsc_cfg = stream->timing.dsc_cfg;
		ASSERT(dsc_cfg.dc_dsc_cfg.num_slices_h % opp_cnt == 0);
		dsc_cfg.dc_dsc_cfg.num_slices_h /= opp_cnt;
		dsc_cfg.dsc_padding = 0;

		if (should_use_dto_dscclk)
			dccg->funcs->set_dto_dscclk(dccg, dsc->inst, dsc_cfg.dc_dsc_cfg.num_slices_h);
		dsc->funcs->dsc_set_config(dsc, &dsc_cfg, &dsc_optc_cfg);
		dsc->funcs->dsc_enable(dsc, pipe_ctx->stream_res.opp->inst);
		for (odm_pipe = pipe_ctx->next_odm_pipe; odm_pipe; odm_pipe = odm_pipe->next_odm_pipe) {
			struct display_stream_compressor *odm_dsc = odm_pipe->stream_res.dsc;

			if (should_use_dto_dscclk)
				dccg->funcs->set_dto_dscclk(dccg, odm_dsc->inst, dsc_cfg.dc_dsc_cfg.num_slices_h);
			odm_dsc->funcs->dsc_set_config(odm_dsc, &dsc_cfg, &dsc_optc_cfg);
			odm_dsc->funcs->dsc_enable(odm_dsc, odm_pipe->stream_res.opp->inst);
		}
		dsc_cfg.dc_dsc_cfg.num_slices_h *= opp_cnt;
		dsc_cfg.pic_width *= opp_cnt;
		dsc_cfg.dsc_padding = pipe_ctx->dsc_padding_params.dsc_hactive_padding;

		optc_dsc_mode = dsc_optc_cfg.is_pixel_format_444 ? OPTC_DSC_ENABLED_444 : OPTC_DSC_ENABLED_NATIVE_SUBSAMPLED;

		/* Enable DSC in encoder */
		if (dc_is_dp_signal(stream->signal) && !dp_is_128b_132b_signal(pipe_ctx)) {
			DC_LOG_DSC("Setting stream encoder DSC config for engine %d:", (int)pipe_ctx->stream_res.stream_enc->id);
			dsc_optc_config_log(dsc, &dsc_optc_cfg);
			if (pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_config)
				pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_config(pipe_ctx->stream_res.stream_enc,
										optc_dsc_mode,
										dsc_optc_cfg.bytes_per_pixel,
										dsc_optc_cfg.slice_width);

			/* PPS SDP is set elsewhere because it has to be done after DIG FE is connected to DIG BE */
		}

		/* Enable DSC in OPTC */
		DC_LOG_DSC("Setting optc DSC config for tg instance %d:", pipe_ctx->stream_res.tg->inst);
		dsc_optc_config_log(dsc, &dsc_optc_cfg);
		pipe_ctx->stream_res.tg->funcs->set_dsc_config(pipe_ctx->stream_res.tg,
							optc_dsc_mode,
							dsc_optc_cfg.bytes_per_pixel,
							dsc_optc_cfg.slice_width);
	} else {
		/* disable DSC in OPTC */
		pipe_ctx->stream_res.tg->funcs->set_dsc_config(
				pipe_ctx->stream_res.tg,
				OPTC_DSC_DISABLED, 0, 0);

		/* disable DSC in stream encoder */
		if (dc_is_dp_signal(stream->signal)) {
			if (dp_is_128b_132b_signal(pipe_ctx))
				pipe_ctx->stream_res.hpo_dp_stream_enc->funcs->dp_set_dsc_pps_info_packet(
										pipe_ctx->stream_res.hpo_dp_stream_enc,
										false,
										NULL,
										true);
			else {
				if (pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_config)
					pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_config(
							pipe_ctx->stream_res.stream_enc,
							OPTC_DSC_DISABLED, 0, 0);
				pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_pps_info_packet(
							pipe_ctx->stream_res.stream_enc, false, NULL, true);
			}
		}

		/* disable DSC block */
		for (odm_pipe = pipe_ctx; odm_pipe; odm_pipe = odm_pipe->next_odm_pipe) {
			odm_pipe->stream_res.dsc->funcs->dsc_disconnect(odm_pipe->stream_res.dsc);
			/*
			 * TODO - dsc_disconnect is a double buffered register.
			 * by the time we call dsc_disable, dsc may still remain
			 * connected to OPP. In this case OPTC will no longer
			 * get correct pixel data because DSCC is off. However
			 * we also can't wait for the  disconnect pending
			 * complete, because this function can be called
			 * with/without OTG master lock acquired. When the lock
			 * is acquired we will never get pending complete until
			 * we release the lock later. So there is no easy way to
			 * solve this problem especially when the lock is
			 * acquired. DSC is a front end hw block it should be
			 * programmed as part of front end sequence, where the
			 * commit sequence without lock and update sequence
			 * with lock are completely separated. However because
			 * we are programming dsc as part of back end link
			 * programming sequence, we don't know if front end OPTC
			 * master lock is acquired. The back end should be
			 * agnostic to front end lock. DSC programming shouldn't
			 * belong to this sequence.
			 */
			odm_pipe->stream_res.dsc->funcs->dsc_disable(odm_pipe->stream_res.dsc);
			if (dccg->funcs->set_ref_dscclk)
				dccg->funcs->set_ref_dscclk(dccg, odm_pipe->stream_res.dsc->inst);
		}
	}
}

/*
 * For dynamic bpp change case, dsc is programmed with MASTER_UPDATE_LOCK enabled;
 * hence PPS info packet update need to use frame update instead of immediate update.
 * Added parameter immediate_update for this purpose.
 * The decision to use frame update is hard-coded in function dp_update_dsc_config(),
 * which is the only place where a "false" would be passed in for param immediate_update.
 *
 * immediate_update is only applicable when DSC is enabled.
 */
bool link_set_dsc_pps_packet(struct pipe_ctx *pipe_ctx, bool enable, bool immediate_update)
{
	struct display_stream_compressor *dsc = pipe_ctx->stream_res.dsc;
	struct dc_stream_state *stream = pipe_ctx->stream;

	if (!pipe_ctx->stream->timing.flags.DSC)
		return false;

	if (!dsc)
		return false;

	DC_LOGGER_INIT(dsc->ctx->logger);

	if (enable) {
		struct dsc_config dsc_cfg;
		uint8_t dsc_packed_pps[128];

		memset(&dsc_cfg, 0, sizeof(dsc_cfg));
		memset(dsc_packed_pps, 0, 128);

		/* Enable DSC hw block */
		dsc_cfg.pic_width = stream->timing.h_addressable + stream->timing.h_border_left + stream->timing.h_border_right;
		dsc_cfg.pic_height = stream->timing.v_addressable + stream->timing.v_border_top + stream->timing.v_border_bottom;
		dsc_cfg.pixel_encoding = stream->timing.pixel_encoding;
		dsc_cfg.color_depth = stream->timing.display_color_depth;
		dsc_cfg.is_odm = pipe_ctx->next_odm_pipe ? true : false;
		dsc_cfg.dc_dsc_cfg = stream->timing.dsc_cfg;
		dsc_cfg.dsc_padding = pipe_ctx->dsc_padding_params.dsc_hactive_padding;

		dsc->funcs->dsc_get_packed_pps(dsc, &dsc_cfg, &dsc_packed_pps[0]);
		memcpy(&stream->dsc_packed_pps[0], &dsc_packed_pps[0], sizeof(stream->dsc_packed_pps));
		if (dc_is_dp_signal(stream->signal)) {
			DC_LOG_DSC("Setting stream encoder DSC PPS SDP for engine %d\n", (int)pipe_ctx->stream_res.stream_enc->id);
			if (dp_is_128b_132b_signal(pipe_ctx))
				pipe_ctx->stream_res.hpo_dp_stream_enc->funcs->dp_set_dsc_pps_info_packet(
										pipe_ctx->stream_res.hpo_dp_stream_enc,
										true,
										&dsc_packed_pps[0],
										immediate_update);
			else
				pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_pps_info_packet(
						pipe_ctx->stream_res.stream_enc,
						true,
						&dsc_packed_pps[0],
						immediate_update);
		}
	} else {
		/* disable DSC PPS in stream encoder */
		memset(&stream->dsc_packed_pps[0], 0, sizeof(stream->dsc_packed_pps));
		if (dc_is_dp_signal(stream->signal)) {
			if (dp_is_128b_132b_signal(pipe_ctx))
				pipe_ctx->stream_res.hpo_dp_stream_enc->funcs->dp_set_dsc_pps_info_packet(
										pipe_ctx->stream_res.hpo_dp_stream_enc,
										false,
										NULL,
										true);
			else
				pipe_ctx->stream_res.stream_enc->funcs->dp_set_dsc_pps_info_packet(
						pipe_ctx->stream_res.stream_enc, false, NULL, true);
		}
	}

	return true;
}

bool link_set_dsc_enable(struct pipe_ctx *pipe_ctx, bool enable)
{
	struct display_stream_compressor *dsc = pipe_ctx->stream_res.dsc;
	bool result = false;

	if (!pipe_ctx->stream->timing.flags.DSC)
		goto out;
	if (!dsc)
		goto out;

	if (enable) {
		{
			link_set_dsc_on_stream(pipe_ctx, true);
			result = true;
		}
	} else {
		dp_set_dsc_on_rx(pipe_ctx, false);
		link_set_dsc_on_stream(pipe_ctx, false);
		result = true;
	}
out:
	return result;
}

bool link_update_dsc_config(struct pipe_ctx *pipe_ctx)
{
	struct display_stream_compressor *dsc = pipe_ctx->stream_res.dsc;

	if (!pipe_ctx->stream->timing.flags.DSC)
		return false;
	if (!dsc)
		return false;

	link_set_dsc_on_stream(pipe_ctx, true);
	link_set_dsc_pps_packet(pipe_ctx, true, false);
	return true;
}

static void program_msa_timing_ignore(struct dc_stream_state *stream)
{
	struct dc_link *link = stream->link;
	union down_spread_ctrl old_downspread;
	union down_spread_ctrl new_downspread;
	bool apple5k_dce12_tile;
	bool apple5k_force_msa_ignore;
	enum dc_status read_status;
	enum dc_status write_status = DC_OK;
	bool wrote_downspread = false;

	if (!link)
		return;

	/*
	 * Optional Apple 5K/DCE12 A/B: force sink DPCD 0x107[7] without
	 * setting stream->ignore_msa_timing_param, so DC's timing-sync grouping
	 * still matches the normal Windows-shaped pair flow. Default is off.
	 */
	apple5k_dce12_tile = link->ctx->dce_version == DCE_VERSION_12_0 &&
		apple5k_is_tile(link);
	apple5k_force_msa_ignore = apple5k_dce12_tile &&
		link->dc->apple5k_policy.dce12_force_msa_ignore;

	DC_LOGGER_INIT(link->ctx->logger);

	memset(&old_downspread, 0, sizeof(old_downspread));

	read_status = core_link_read_dpcd(link, DP_DOWNSPREAD_CTRL,
			&old_downspread.raw, sizeof(old_downspread));

	new_downspread.raw = old_downspread.raw;
	new_downspread.bits.IGNORE_MSA_TIMING_PARAM =
			(stream->ignore_msa_timing_param ||
			 apple5k_force_msa_ignore) ? 1 : 0;

	/* Preserve SSC/downspread bits; never synthesize 0x80 after a failed read. */
	if (read_status == DC_OK && new_downspread.raw != old_downspread.raw) {
		wrote_downspread = true;
		write_status = core_link_write_dpcd(link, DP_DOWNSPREAD_CTRL,
			&new_downspread.raw, sizeof(new_downspread));
	}

	if (apple5k_dce12_tile &&
	    link_apple5k_log_enabled(link->dc, APPLE5K_LOG_LINK))
		DC_LOG_INFO("APPLE5K: DCE12 MSA-ignore 0x107 link[%u] force=%d read_status=%d old=0x%02x new=0x%02x write=%d write_status=%d stream_ignore_msa=%u\n",
			    link->link_index, apple5k_force_msa_ignore,
			    read_status, old_downspread.raw, new_downspread.raw,
			    wrote_downspread, write_status,
			    stream->ignore_msa_timing_param);
}

static void enable_stream_features(struct pipe_ctx *pipe_ctx,
				   bool defer_apple5k_latch)
{
	struct dc_stream_state *stream = pipe_ctx->stream;

	if (pipe_ctx->stream->signal != SIGNAL_TYPE_DISPLAY_PORT_MST) {
		struct dc_link *link = stream->link;
		DC_LOGGER_INIT(link->ctx->logger);

		program_msa_timing_ignore(stream);

		if (defer_apple5k_latch && apple5k_is_tile(link))
			DC_LOG_INFO("APPLE5K: defer stream-latch/status link[%u] until pair finalizer\n",
				    link->link_index);
		else
			apple5k_latch_stream(link);
	} else {
		dm_helpers_mst_enable_stream_features(stream);
	}
}

static void log_vcp_x_y(const struct dc_link *link, struct fixed31_32 avg_time_slots_per_mtp)
{
	const uint32_t VCP_Y_PRECISION = 1000;
	uint64_t vcp_x, vcp_y;
	DC_LOGGER_INIT(link->ctx->logger);

	// Add 0.5*(1/VCP_Y_PRECISION) to round up to decimal precision
	avg_time_slots_per_mtp = dc_fixpt_add(
			avg_time_slots_per_mtp,
			dc_fixpt_from_fraction(
				1,
				2*VCP_Y_PRECISION));

	vcp_x = dc_fixpt_floor(
			avg_time_slots_per_mtp);
	vcp_y = dc_fixpt_floor(
			dc_fixpt_mul_int(
				dc_fixpt_sub_int(
					avg_time_slots_per_mtp,
					dc_fixpt_floor(
							avg_time_slots_per_mtp)),
				VCP_Y_PRECISION));


	if (link->type == dc_connection_mst_branch)
		DC_LOG_DP2("MST Update Payload: set_throttled_vcp_size slot X.Y for MST stream "
				"X: %llu "
				"Y: %llu/%d",
				vcp_x,
				vcp_y,
				VCP_Y_PRECISION);
	else
		DC_LOG_DP2("SST Update Payload: set_throttled_vcp_size slot X.Y for SST stream "
				"X: %llu "
				"Y: %llu/%d",
				vcp_x,
				vcp_y,
				VCP_Y_PRECISION);
}

static struct fixed31_32 get_pbn_per_slot(struct dc_stream_state *stream)
{
	struct fixed31_32 mbytes_per_sec;
	uint32_t link_rate_in_mbytes_per_sec = dp_link_bandwidth_kbps(stream->link,
			&stream->link->cur_link_settings);
	link_rate_in_mbytes_per_sec /= 8000; /* Kbits to MBytes */

	mbytes_per_sec = dc_fixpt_from_int(link_rate_in_mbytes_per_sec);

	return dc_fixpt_div_int(mbytes_per_sec, 54);
}

static struct fixed31_32 get_pbn_from_bw_in_kbps(uint64_t kbps)
{
	struct fixed31_32 peak_kbps;
	uint32_t numerator = 0;
	uint32_t denominator = 1;

	/*
	 * The 1.006 factor (margin 5300ppm + 300ppm ~ 0.6% as per spec) is not
	 * required when determining PBN/time slot utilization on the link between
	 * us and the branch, since that overhead is already accounted for in
	 * the get_pbn_per_slot function.
	 *
	 * The unit of 54/64Mbytes/sec is an arbitrary unit chosen based on
	 * common multiplier to render an integer PBN for all link rate/lane
	 * counts combinations
	 * calculate
	 * peak_kbps *= (64/54)
	 * peak_kbps /= (8 * 1000) convert to bytes
	 */

	numerator = 64;
	denominator = 54 * 8 * 1000;
	kbps *= numerator;
	peak_kbps = dc_fixpt_from_fraction(kbps, denominator);

	return peak_kbps;
}

static struct fixed31_32 get_pbn_from_timing(struct pipe_ctx *pipe_ctx)
{
	uint64_t kbps;
	enum dc_link_encoding_format link_encoding;

	if (dp_is_128b_132b_signal(pipe_ctx))
		link_encoding = DC_LINK_ENCODING_DP_128b_132b;
	else
		link_encoding = DC_LINK_ENCODING_DP_8b_10b;

	kbps = dc_bandwidth_in_kbps_from_timing(&pipe_ctx->stream->timing, link_encoding);
	return get_pbn_from_bw_in_kbps(kbps);
}


// TODO - DP2.0 Link: Fix get_lane_status to handle LTTPR offset (SST and MST)
static void get_lane_status(
	struct dc_link *link,
	uint32_t lane_count,
	union lane_status *status,
	union lane_align_status_updated *status_updated)
{
	unsigned int lane;
	uint8_t dpcd_buf[3] = {0};

	if (status == NULL || status_updated == NULL) {
		return;
	}

	core_link_read_dpcd(
			link,
			DP_LANE0_1_STATUS,
			dpcd_buf,
			sizeof(dpcd_buf));

	for (lane = 0; lane < lane_count; lane++) {
		status[lane].raw = dp_get_nibble_at_index(&dpcd_buf[0], lane);
	}

	status_updated->raw = dpcd_buf[2];
}

static bool poll_for_allocation_change_trigger(struct dc_link *link)
{
	/*
	 * wait for ACT handled
	 */
	int i;
	const int act_retries = 30;
	enum act_return_status result = ACT_FAILED;
	enum dc_connection_type display_connected = (link->type != dc_connection_none);
	union payload_table_update_status update_status = {0};
	union lane_status dpcd_lane_status[LANE_COUNT_DP_MAX];
	union lane_align_status_updated lane_status_updated;
	DC_LOGGER_INIT(link->ctx->logger);

	if (!display_connected || link->aux_access_disabled)
		return true;
	for (i = 0; i < act_retries; i++) {
		get_lane_status(link, link->cur_link_settings.lane_count, dpcd_lane_status, &lane_status_updated);

		if (!dp_is_cr_done(link->cur_link_settings.lane_count, dpcd_lane_status) ||
				!dp_is_ch_eq_done(link->cur_link_settings.lane_count, dpcd_lane_status) ||
				!dp_is_symbol_locked(link->cur_link_settings.lane_count, dpcd_lane_status) ||
				!dp_is_interlane_aligned(lane_status_updated)) {
			DC_LOG_ERROR("SST Update Payload: Link loss occurred while "
					"polling for ACT handled.");
			result = ACT_LINK_LOST;
			break;
		}
		core_link_read_dpcd(
				link,
				DP_PAYLOAD_TABLE_UPDATE_STATUS,
				&update_status.raw,
				1);

		if (update_status.bits.ACT_HANDLED == 1) {
			DC_LOG_DP2("SST Update Payload: ACT handled by downstream.");
			result = ACT_SUCCESS;
			break;
		}

		fsleep(5000);
	}

	if (result == ACT_FAILED) {
		DC_LOG_ERROR("SST Update Payload: ACT still not handled after retries, "
				"continue on. Something is wrong with the branch.");
	}

	return (result == ACT_SUCCESS);
}

static void update_mst_stream_alloc_table(
	struct dc_link *link,
	struct stream_encoder *stream_enc,
	struct hpo_dp_stream_encoder *hpo_dp_stream_enc, // TODO: Rename stream_enc to dio_stream_enc?
	const struct dc_dp_mst_stream_allocation_table *proposed_table)
{
	struct link_mst_stream_allocation work_table[MAX_CONTROLLER_NUM] = { 0 };
	struct link_mst_stream_allocation *dc_alloc;

	int i;
	int j;

	/* if DRM proposed_table has more than one new payload */
	ASSERT(proposed_table->stream_count -
			link->mst_stream_alloc_table.stream_count < 2);

	/* copy proposed_table to link, add stream encoder */
	for (i = 0; i < proposed_table->stream_count; i++) {

		for (j = 0; j < link->mst_stream_alloc_table.stream_count; j++) {
			dc_alloc =
			&link->mst_stream_alloc_table.stream_allocations[j];

			if (dc_alloc->vcp_id ==
				proposed_table->stream_allocations[i].vcp_id) {

				work_table[i] = *dc_alloc;
				work_table[i].slot_count = proposed_table->stream_allocations[i].slot_count;
				break; /* exit j loop */
			}
		}

		/* new vcp_id */
		if (j == link->mst_stream_alloc_table.stream_count) {
			work_table[i].vcp_id =
				proposed_table->stream_allocations[i].vcp_id;
			work_table[i].slot_count =
				proposed_table->stream_allocations[i].slot_count;
			work_table[i].stream_enc = stream_enc;
			work_table[i].hpo_dp_stream_enc = hpo_dp_stream_enc;
		}
	}

	/* update link->mst_stream_alloc_table with work_table */
	link->mst_stream_alloc_table.stream_count =
			proposed_table->stream_count;
	for (i = 0; i < MAX_CONTROLLER_NUM; i++)
		link->mst_stream_alloc_table.stream_allocations[i] =
				work_table[i];
}

static void remove_stream_from_alloc_table(
		struct dc_link *link,
		struct stream_encoder *dio_stream_enc,
		struct hpo_dp_stream_encoder *hpo_dp_stream_enc)
{
	int i = 0;
	struct link_mst_stream_allocation_table *table =
			&link->mst_stream_alloc_table;

	if (hpo_dp_stream_enc) {
		for (; i < table->stream_count; i++)
			if (hpo_dp_stream_enc == table->stream_allocations[i].hpo_dp_stream_enc)
				break;
	} else {
		for (; i < table->stream_count; i++)
			if (dio_stream_enc == table->stream_allocations[i].stream_enc)
				break;
	}

	if (i < table->stream_count) {
		i++;
		for (; i < table->stream_count; i++)
			table->stream_allocations[i-1] = table->stream_allocations[i];
		memset(&table->stream_allocations[table->stream_count-1], 0,
				sizeof(struct link_mst_stream_allocation));
		table->stream_count--;
	}
}

static void print_mst_streams(struct dc_link *link)
{
	int i;

	DC_LOGGER_INIT(link->ctx->logger);

	DC_LOG_MST("%s stream_count: %d:\n",
		   __func__,
		   link->mst_stream_alloc_table.stream_count);

	for (i = 0; i < MAX_CONTROLLER_NUM; i++) {
		DC_LOG_MST("stream_enc[%d]: %p\n", i,
			   (void *) link->mst_stream_alloc_table.stream_allocations[i].stream_enc);
		DC_LOG_MST("stream[%d].hpo_dp_stream_enc: %p\n", i,
			   (void *) link->mst_stream_alloc_table.stream_allocations[i].hpo_dp_stream_enc);
		DC_LOG_MST("stream[%d].vcp_id: %d\n", i,
			   link->mst_stream_alloc_table.stream_allocations[i].vcp_id);
		DC_LOG_MST("stream[%d].slot_count: %d\n", i,
			   link->mst_stream_alloc_table.stream_allocations[i].slot_count);
	}
}

static enum dc_status deallocate_mst_payload(struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct dc_dp_mst_stream_allocation_table proposed_table = {0};
	struct fixed31_32 avg_time_slots_per_mtp = dc_fixpt_from_int(0);
	bool mst_mode = (link->type == dc_connection_mst_branch);
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	const struct dc_link_settings empty_link_settings = {0};
	DC_LOGGER_INIT(link->ctx->logger);

	/* deallocate_mst_payload is called before disable link. When mode or
	 * disable/enable monitor, new stream is created which is not in link
	 * stream[] yet. For this, payload is not allocated yet, so de-alloc
	 * should not done. For new mode set, map_resources will get engine
	 * for new stream, so stream_enc->id should be validated until here.
	 */

	/* slot X.Y */
	if (link_hwss->ext.set_throttled_vcp_size)
		link_hwss->ext.set_throttled_vcp_size(pipe_ctx, avg_time_slots_per_mtp);
	if (link_hwss->ext.set_hblank_min_symbol_width)
		link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
				&empty_link_settings,
				avg_time_slots_per_mtp);

	if (mst_mode) {
		/* when link is in mst mode, reply on mst manager to remove
		 * payload
		 */
		if (dm_helpers_dp_mst_write_payload_allocation_table(
				stream->ctx,
				stream,
				&proposed_table,
				false))
			update_mst_stream_alloc_table(
					link,
					pipe_ctx->stream_res.stream_enc,
					pipe_ctx->stream_res.hpo_dp_stream_enc,
					&proposed_table);
		else
			DC_LOG_WARNING("Failed to update MST allocation table for idx %d\n",
					pipe_ctx->pipe_idx);
	} else {
		/* when link is no longer in mst mode (mst hub unplugged),
		 * remove payload with default dc logic
		 */
		remove_stream_from_alloc_table(link, pipe_ctx->stream_res.stream_enc,
				pipe_ctx->stream_res.hpo_dp_stream_enc);
	}

	print_mst_streams(link);

	/* update mst stream allocation table hardware state */
	if (link_hwss->ext.update_stream_allocation_table == NULL ||
			link_dp_get_encoding_format(&link->cur_link_settings) == DP_UNKNOWN_ENCODING) {
		DC_LOG_DEBUG("Unknown encoding format\n");
		return DC_ERROR_UNEXPECTED;
	}

	link_hwss->ext.update_stream_allocation_table(link, &pipe_ctx->link_res,
			&link->mst_stream_alloc_table);

	if (mst_mode)
		dm_helpers_dp_mst_poll_for_allocation_change_trigger(
			stream->ctx,
			stream);

	dm_helpers_dp_mst_update_mst_mgr_for_deallocation(
			stream->ctx,
			stream);

	return DC_OK;
}

/* convert link_mst_stream_alloc_table to dm dp_mst_stream_alloc_table
 * because stream_encoder is not exposed to dm
 */
static enum dc_status allocate_mst_payload(struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct dc_dp_mst_stream_allocation_table proposed_table = {0};
	struct fixed31_32 avg_time_slots_per_mtp;
	struct fixed31_32 pbn;
	struct fixed31_32 pbn_per_slot;
	enum act_return_status ret;
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	DC_LOGGER_INIT(link->ctx->logger);

	/* enable_link_dp_mst already check link->enabled_stream_count
	 * and stream is in link->stream[]. This is called during set mode,
	 * stream_enc is available.
	 */

	/* get calculate VC payload for stream: stream_alloc */
	if (dm_helpers_dp_mst_write_payload_allocation_table(
		stream->ctx,
		stream,
		&proposed_table,
		true))
		update_mst_stream_alloc_table(
					link,
					pipe_ctx->stream_res.stream_enc,
					pipe_ctx->stream_res.hpo_dp_stream_enc,
					&proposed_table);
	else
		DC_LOG_WARNING("Failed to update MST allocation table for idx %d\n",
				pipe_ctx->pipe_idx);

	print_mst_streams(link);

	ASSERT(proposed_table.stream_count > 0);

	/* program DP source TX for payload */
	if (link_hwss->ext.update_stream_allocation_table == NULL ||
			link_dp_get_encoding_format(&link->cur_link_settings) == DP_UNKNOWN_ENCODING) {
		DC_LOG_ERROR("Failure: unknown encoding format\n");
		return DC_ERROR_UNEXPECTED;
	}

	link_hwss->ext.update_stream_allocation_table(link,
			&pipe_ctx->link_res,
			&link->mst_stream_alloc_table);

	/* send down message */
	ret = dm_helpers_dp_mst_poll_for_allocation_change_trigger(
			stream->ctx,
			stream);

	if (ret != ACT_LINK_LOST)
		dm_helpers_dp_mst_send_payload_allocation(
				stream->ctx,
				stream);

	/* slot X.Y for only current stream */
	pbn_per_slot = get_pbn_per_slot(stream);
	if (pbn_per_slot.value == 0) {
		DC_LOG_ERROR("Failure: pbn_per_slot==0 not allowed. Cannot continue, returning DC_UNSUPPORTED_VALUE.\n");
		return DC_UNSUPPORTED_VALUE;
	}
	pbn = get_pbn_from_timing(pipe_ctx);
	avg_time_slots_per_mtp = dc_fixpt_div(pbn, pbn_per_slot);

	log_vcp_x_y(link, avg_time_slots_per_mtp);

	if (link_hwss->ext.set_throttled_vcp_size)
		link_hwss->ext.set_throttled_vcp_size(pipe_ctx, avg_time_slots_per_mtp);
	if (link_hwss->ext.set_hblank_min_symbol_width)
		link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
				&link->cur_link_settings,
				avg_time_slots_per_mtp);

	return DC_OK;
}

struct fixed31_32 link_calculate_sst_avg_time_slots_per_mtp(
		const struct dc_stream_state *stream,
		const struct dc_link *link)
{
	struct fixed31_32 link_bw_effective =
			dc_fixpt_from_int(
					dp_link_bandwidth_kbps(link, &link->cur_link_settings));
	struct fixed31_32 timeslot_bw_effective =
			dc_fixpt_div_int(link_bw_effective, MAX_MTP_SLOT_COUNT);
	struct fixed31_32 timing_bw =
			dc_fixpt_from_int(
					dc_bandwidth_in_kbps_from_timing(&stream->timing,
							dc_link_get_highest_encoding_format(link)));
	struct fixed31_32 avg_time_slots_per_mtp =
			dc_fixpt_div(timing_bw, timeslot_bw_effective);

	return avg_time_slots_per_mtp;
}


static bool write_128b_132b_sst_payload_allocation_table(
		const struct dc_stream_state *stream,
		struct dc_link *link,
		struct link_mst_stream_allocation_table *proposed_table,
		bool allocate)
{
	const uint8_t vc_id = 1; /// VC ID always 1 for SST
	const uint8_t start_time_slot = 0; /// Always start at time slot 0 for SST
	bool result = false;
	uint8_t req_slot_count = 0;
	struct fixed31_32 avg_time_slots_per_mtp = { 0 };
	union payload_table_update_status update_status = { 0 };
	const uint32_t max_retries = 30;
	uint32_t retries = 0;
	enum dc_connection_type display_connected = (link->type != dc_connection_none);
	DC_LOGGER_INIT(link->ctx->logger);

	if (allocate)	{
		avg_time_slots_per_mtp = link_calculate_sst_avg_time_slots_per_mtp(stream, link);
		req_slot_count = dc_fixpt_ceil(avg_time_slots_per_mtp);
		/// Validation should filter out modes that exceed link BW
		ASSERT(req_slot_count <= MAX_MTP_SLOT_COUNT);
		if (req_slot_count > MAX_MTP_SLOT_COUNT)
			return false;
	} else {
		/// Leave req_slot_count = 0 if allocate is false.
	}

	proposed_table->stream_count = 1; /// Always 1 stream for SST
	proposed_table->stream_allocations[0].slot_count = req_slot_count;
	proposed_table->stream_allocations[0].vcp_id = vc_id;

	if (!display_connected || link->aux_access_disabled)
		return true;

	/// Write DPCD 2C0 = 1 to start updating
	update_status.bits.VC_PAYLOAD_TABLE_UPDATED = 1;
	core_link_write_dpcd(
			link,
			DP_PAYLOAD_TABLE_UPDATE_STATUS,
			&update_status.raw,
			1);

	/// Program the changes in DPCD 1C0 - 1C2
	ASSERT(vc_id == 1);
	core_link_write_dpcd(
			link,
			DP_PAYLOAD_ALLOCATE_SET,
			&vc_id,
			1);

	ASSERT(start_time_slot == 0);
	core_link_write_dpcd(
			link,
			DP_PAYLOAD_ALLOCATE_START_TIME_SLOT,
			&start_time_slot,
			1);

	core_link_write_dpcd(
			link,
			DP_PAYLOAD_ALLOCATE_TIME_SLOT_COUNT,
			&req_slot_count,
			1);

	/// Poll till DPCD 2C0 read 1
	/// Try for at least 150ms (30 retries, with 5ms delay after each attempt)

	while (retries < max_retries) {
		if (core_link_read_dpcd(
				link,
				DP_PAYLOAD_TABLE_UPDATE_STATUS,
				&update_status.raw,
				1) == DC_OK) {
			if (update_status.bits.VC_PAYLOAD_TABLE_UPDATED == 1) {
				DC_LOG_DP2("SST Update Payload: downstream payload table updated.");
				result = true;
				break;
			}
		} else {
			union dpcd_rev dpcdRev = {0};

			if (core_link_read_dpcd(
					link,
					DP_DPCD_REV,
					&dpcdRev.raw,
					1) != DC_OK) {
				DC_LOG_ERROR("SST Update Payload: Unable to read DPCD revision "
						"of sink while polling payload table "
						"updated status bit.");
				break;
			}
		}
		retries++;
		fsleep(5000);
	}

	if (!result && retries == max_retries) {
		DC_LOG_ERROR("SST Update Payload: Payload table not updated after retries, "
				"continue on. Something is wrong with the branch.");
		// TODO - DP2.0 Payload: Read and log the payload table from downstream branch
	}

	return result;
}

/*
 * Payload allocation/deallocation for SST introduced in DP2.0
 */
static enum dc_status update_sst_payload(struct pipe_ctx *pipe_ctx,
						 bool allocate)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct link_mst_stream_allocation_table proposed_table = {0};
	struct fixed31_32 avg_time_slots_per_mtp;
	const struct dc_link_settings empty_link_settings = {0};
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	DC_LOGGER_INIT(link->ctx->logger);

	/* slot X.Y for SST payload deallocate */
	if (!allocate) {
		avg_time_slots_per_mtp = dc_fixpt_from_int(0);

		log_vcp_x_y(link, avg_time_slots_per_mtp);

		if (link_hwss->ext.set_throttled_vcp_size)
			link_hwss->ext.set_throttled_vcp_size(pipe_ctx,
					avg_time_slots_per_mtp);
		if (link_hwss->ext.set_hblank_min_symbol_width)
			link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
					&empty_link_settings,
					avg_time_slots_per_mtp);
	}

	/* calculate VC payload and update branch with new payload allocation table*/
	if (!write_128b_132b_sst_payload_allocation_table(
			stream,
			link,
			&proposed_table,
			allocate)) {
		DC_LOG_ERROR("SST Update Payload: Failed to update "
						"allocation table for "
						"pipe idx: %d\n",
						pipe_ctx->pipe_idx);
		return DC_FAIL_DP_PAYLOAD_ALLOCATION;
	}

	proposed_table.stream_allocations[0].hpo_dp_stream_enc = pipe_ctx->stream_res.hpo_dp_stream_enc;

	ASSERT(proposed_table.stream_count == 1);

	//TODO - DP2.0 Logging: Instead of hpo_dp_stream_enc pointer, log instance id
	DC_LOG_DP2("SST Update Payload: hpo_dp_stream_enc: %p      "
		"vcp_id: %d      "
		"slot_count: %d\n",
		(void *) proposed_table.stream_allocations[0].hpo_dp_stream_enc,
		proposed_table.stream_allocations[0].vcp_id,
		proposed_table.stream_allocations[0].slot_count);

	/* program DP source TX for payload */
	link_hwss->ext.update_stream_allocation_table(link, &pipe_ctx->link_res,
			&proposed_table);

	/* poll for ACT handled */
	if (!poll_for_allocation_change_trigger(link)) {
		// Failures will result in blackscreen and errors logged
		BREAK_TO_DEBUGGER();
	}

	/* slot X.Y for SST payload allocate */
	if (allocate && link_dp_get_encoding_format(&link->cur_link_settings) ==
			DP_128b_132b_ENCODING) {
		avg_time_slots_per_mtp = link_calculate_sst_avg_time_slots_per_mtp(stream, link);

		log_vcp_x_y(link, avg_time_slots_per_mtp);

		if (link_hwss->ext.set_throttled_vcp_size)
			link_hwss->ext.set_throttled_vcp_size(pipe_ctx,
					avg_time_slots_per_mtp);
		if (link_hwss->ext.set_hblank_min_symbol_width)
			link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
					&link->cur_link_settings,
					avg_time_slots_per_mtp);
	}

	/* Always return DC_OK.
	 * If part of sequence fails, log failure(s) and show blackscreen
	 */
	return DC_OK;
}

enum dc_status link_reduce_mst_payload(struct pipe_ctx *pipe_ctx, uint32_t bw_in_kbps)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct fixed31_32 avg_time_slots_per_mtp;
	struct fixed31_32 pbn;
	struct fixed31_32 pbn_per_slot;
	struct dc_dp_mst_stream_allocation_table proposed_table = {0};
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	DC_LOGGER_INIT(link->ctx->logger);

	/* decrease throttled vcp size */
	pbn_per_slot = get_pbn_per_slot(stream);
	pbn = get_pbn_from_bw_in_kbps(bw_in_kbps);
	avg_time_slots_per_mtp = dc_fixpt_div(pbn, pbn_per_slot);

	if (link_hwss->ext.set_throttled_vcp_size)
		link_hwss->ext.set_throttled_vcp_size(pipe_ctx, avg_time_slots_per_mtp);
	if (link_hwss->ext.set_hblank_min_symbol_width)
		link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
				&link->cur_link_settings,
				avg_time_slots_per_mtp);

	/* send ALLOCATE_PAYLOAD sideband message with updated pbn */
	dm_helpers_dp_mst_send_payload_allocation(
			stream->ctx,
			stream);

	/* notify immediate branch device table update */
	if (dm_helpers_dp_mst_write_payload_allocation_table(
			stream->ctx,
			stream,
			&proposed_table,
			true)) {
		/* update mst stream allocation table software state */
		update_mst_stream_alloc_table(
				link,
				pipe_ctx->stream_res.stream_enc,
				pipe_ctx->stream_res.hpo_dp_stream_enc,
				&proposed_table);
	} else {
		DC_LOG_WARNING("Failed to update MST allocation table for idx %d\n",
				pipe_ctx->pipe_idx);
	}

	print_mst_streams(link);

	ASSERT(proposed_table.stream_count > 0);

	/* update mst stream allocation table hardware state */
	if (link_hwss->ext.update_stream_allocation_table == NULL ||
			link_dp_get_encoding_format(&link->cur_link_settings) == DP_UNKNOWN_ENCODING) {
		DC_LOG_ERROR("Failure: unknown encoding format\n");
		return DC_ERROR_UNEXPECTED;
	}

	link_hwss->ext.update_stream_allocation_table(link, &pipe_ctx->link_res,
			&link->mst_stream_alloc_table);

	/* poll for immediate branch device ACT handled */
	dm_helpers_dp_mst_poll_for_allocation_change_trigger(
			stream->ctx,
			stream);

	return DC_OK;
}

enum dc_status link_increase_mst_payload(struct pipe_ctx *pipe_ctx, uint32_t bw_in_kbps)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct fixed31_32 avg_time_slots_per_mtp;
	struct fixed31_32 pbn;
	struct fixed31_32 pbn_per_slot;
	struct dc_dp_mst_stream_allocation_table proposed_table = {0};
	enum act_return_status ret;
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	DC_LOGGER_INIT(link->ctx->logger);

	/* notify immediate branch device table update */
	if (dm_helpers_dp_mst_write_payload_allocation_table(
				stream->ctx,
				stream,
				&proposed_table,
				true)) {
		/* update mst stream allocation table software state */
		update_mst_stream_alloc_table(
				link,
				pipe_ctx->stream_res.stream_enc,
				pipe_ctx->stream_res.hpo_dp_stream_enc,
				&proposed_table);
	}

	print_mst_streams(link);

	ASSERT(proposed_table.stream_count > 0);

	/* update mst stream allocation table hardware state */
	if (link_hwss->ext.update_stream_allocation_table == NULL ||
			link_dp_get_encoding_format(&link->cur_link_settings) == DP_UNKNOWN_ENCODING) {
		DC_LOG_ERROR("Failure: unknown encoding format\n");
		return DC_ERROR_UNEXPECTED;
	}

	link_hwss->ext.update_stream_allocation_table(link, &pipe_ctx->link_res,
			&link->mst_stream_alloc_table);

	/* poll for immediate branch device ACT handled */
	ret = dm_helpers_dp_mst_poll_for_allocation_change_trigger(
			stream->ctx,
			stream);

	if (ret != ACT_LINK_LOST) {
		/* send ALLOCATE_PAYLOAD sideband message with updated pbn */
		dm_helpers_dp_mst_send_payload_allocation(
				stream->ctx,
				stream);
	}

	/* increase throttled vcp size */
	pbn = get_pbn_from_bw_in_kbps(bw_in_kbps);
	pbn_per_slot = get_pbn_per_slot(stream);
	avg_time_slots_per_mtp = dc_fixpt_div(pbn, pbn_per_slot);

	if (link_hwss->ext.set_throttled_vcp_size)
		link_hwss->ext.set_throttled_vcp_size(pipe_ctx, avg_time_slots_per_mtp);
	if (link_hwss->ext.set_hblank_min_symbol_width)
		link_hwss->ext.set_hblank_min_symbol_width(pipe_ctx,
				&link->cur_link_settings,
				avg_time_slots_per_mtp);

	return DC_OK;
}

static void disable_link_dp(struct dc_link *link,
		const struct link_resource *link_res,
		enum signal_type signal)
{
	struct dc_link_settings link_settings = link->cur_link_settings;

	if (signal == SIGNAL_TYPE_DISPLAY_PORT_MST &&
			link->mst_stream_alloc_table.stream_count > 0)
		/* disable MST link only when last vc payload is deallocated */
		return;

	dp_disable_link_phy(link, link_res, signal);

	if (link->connector_signal == SIGNAL_TYPE_EDP) {
		if (!link->skip_implict_edp_power_control)
			link->dc->hwss.edp_power_control(link, false);
	}

	if (signal == SIGNAL_TYPE_DISPLAY_PORT_MST && link->sink_count == 0)
		/* set the sink to SST mode after disabling the link */
		enable_mst_on_sink(link, false);

	if (link_dp_get_encoding_format(&link_settings) ==
			DP_8b_10b_ENCODING) {
		dp_set_fec_enable(link, link_res, false);
		dp_set_fec_ready(link, link_res, false);
	}
}

static void disable_link(struct dc_link *link,
		const struct link_resource *link_res,
		enum signal_type signal)
{
	if (dc_is_dp_signal(signal)) {
		disable_link_dp(link, link_res, signal);
	} else if (signal == SIGNAL_TYPE_VIRTUAL) {
		link->dc->hwss.disable_link_output(link, link_res, SIGNAL_TYPE_DISPLAY_PORT);
	} else {
		link->dc->hwss.disable_link_output(link, link_res, signal);
	}

	if (signal == SIGNAL_TYPE_DISPLAY_PORT_MST) {
		/* MST disable link only when no stream use the link */
		if (link->mst_stream_alloc_table.stream_count <= 0)
			link->link_status.link_active = false;
	} else {
		link->link_status.link_active = false;
	}
}

static void enable_link_hdmi(struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	enum dc_color_depth display_color_depth;
	enum engine_id eng_id;
	struct ext_hdmi_settings settings = {0};
	bool is_over_340mhz = false;
	bool is_vga_mode = (stream->timing.h_addressable == 640)
			&& (stream->timing.v_addressable == 480);
	struct dc *dc = pipe_ctx->stream->ctx->dc;
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);

	if (stream->phy_pix_clk == 0)
		stream->phy_pix_clk = stream->timing.pix_clk_100hz / 10;
	if (stream->phy_pix_clk > 340000)
		is_over_340mhz = true;
	if (dc_is_tmds_signal(stream->signal) && stream->phy_pix_clk > 6000000UL) {
		ASSERT(false);
		return;
	}

	if (dc_is_hdmi_signal(pipe_ctx->stream->signal)) {
		unsigned short masked_chip_caps = pipe_ctx->stream->link->chip_caps &
				AMD_EXT_DISPLAY_PATH_CAPS__EXT_CHIP_MASK;
		if (masked_chip_caps == AMD_EXT_DISPLAY_PATH_CAPS__HDMI20_TISN65DP159RSBT) {
			/* DP159, Retimer settings */
			eng_id = pipe_ctx->stream_res.stream_enc->id;

			if (get_ext_hdmi_settings(pipe_ctx, eng_id, &settings)) {
				write_i2c_retimer_setting(pipe_ctx,
						is_vga_mode, is_over_340mhz, &settings);
			} else {
				write_i2c_default_retimer_setting(pipe_ctx,
						is_vga_mode, is_over_340mhz);
			}
		} else if (masked_chip_caps == AMD_EXT_DISPLAY_PATH_CAPS__HDMI20_PI3EQX1204) {
			/* PI3EQX1204, Redriver settings */
			write_i2c_redriver_setting(pipe_ctx, is_over_340mhz);
		}
	}

	if (dc_is_hdmi_signal(pipe_ctx->stream->signal))
		write_scdc_data(
			stream->link->ddc,
			stream->phy_pix_clk,
			stream->timing.flags.LTE_340MCSC_SCRAMBLE);

	memset(&stream->link->cur_link_settings, 0,
			sizeof(struct dc_link_settings));

	display_color_depth = stream->timing.display_color_depth;
	if (stream->timing.pixel_encoding == PIXEL_ENCODING_YCBCR422)
		display_color_depth = COLOR_DEPTH_888;

	/* We need to enable stream encoder for TMDS first to apply 1/4 TMDS
	 * character clock in case that beyond 340MHz.
	 */
	if (dc_is_hdmi_tmds_signal(pipe_ctx->stream->signal) || dc_is_dvi_signal(pipe_ctx->stream->signal))
		link_hwss->setup_stream_encoder(pipe_ctx);

	dc->hwss.enable_tmds_link_output(
			link,
			&pipe_ctx->link_res,
			pipe_ctx->stream->signal,
			pipe_ctx->clock_source->id,
			display_color_depth,
			stream->phy_pix_clk);

	if (dc_is_hdmi_signal(pipe_ctx->stream->signal))
		read_scdc_data(link->ddc);
}

static enum dc_status enable_link_dp(struct dc_state *state,
				     struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	enum dc_status status;
	bool skip_video_pattern;
	struct dc_link *link = stream->link;
	const struct dc_link_settings *link_settings =
			&pipe_ctx->link_config.dp_link_settings;
	bool fec_enable;
	int i;
	bool apply_seamless_boot_optimization = false;
	uint32_t bl_oled_enable_delay = 50; // in ms
	uint32_t post_oui_delay = 30; // 30ms
	/* Reduce link bandwidth between failed link training attempts. */
	bool do_fallback = false;
	int lt_attempts = LINK_TRAINING_ATTEMPTS;

	// Increase retry count if attempting DP1.x on FIXED_VS link
	if (((link->chip_caps & AMD_EXT_DISPLAY_PATH_CAPS__EXT_CHIP_MASK) == AMD_EXT_DISPLAY_PATH_CAPS__DP_FIXED_VS_EN) &&
			link_dp_get_encoding_format(link_settings) == DP_8b_10b_ENCODING)
		lt_attempts = 10;

	// check for seamless boot
	for (i = 0; i < state->stream_count; i++) {
		if (state->streams[i]->apply_seamless_boot_optimization) {
			apply_seamless_boot_optimization = true;
			break;
		}
	}

	/* Train with fallback when enabling DPIA link. Conventional links are
	 * trained with fallback during sink detection.
	 */
	if (link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA &&
			!link->dc->config.enable_dpia_pre_training)
		do_fallback = true;

	/*
	 * Temporary w/a to get DP2.0 link rates to work with SST.
	 * TODO DP2.0 - Workaround: Remove w/a if and when the issue is resolved.
	 */
	if (link_dp_get_encoding_format(link_settings) == DP_128b_132b_ENCODING &&
			pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT &&
			link->dc->debug.set_mst_en_for_sst) {
		enable_mst_on_sink(link, true);
	} else if (link->dpcd_caps.is_mst_capable &&
		pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT) {
		/* disable mst on sink */
		enable_mst_on_sink(link, false);
	}

	if (pipe_ctx->stream->signal == SIGNAL_TYPE_EDP) {
		/*in case it is not on*/
		if (!link->dc->config.edp_no_power_sequencing)
			link->dc->hwss.edp_power_control(link, true);
		link->dc->hwss.edp_wait_for_hpd_ready(link, true);
	}

	if (link_dp_get_encoding_format(link_settings) == DP_128b_132b_ENCODING) {
		/* TODO - DP2.0 HW: calculate 32 symbol clock for HPO encoder */
	} else {
		pipe_ctx->stream_res.pix_clk_params.requested_sym_clk =
				link_settings->link_rate * LINK_RATE_REF_FREQ_IN_KHZ;
		if (state->clk_mgr && !apply_seamless_boot_optimization)
			state->clk_mgr->funcs->update_clocks(state->clk_mgr,
					state, false);
	}

	// during mode switch we do DP_SET_POWER off then on, and OUI is lost
	dpcd_set_source_specific_data(link);
	if (link->dpcd_sink_ext_caps.raw != 0) {
		post_oui_delay += link->panel_config.pps.extra_post_OUI_ms;
		msleep(post_oui_delay);
	}

	// similarly, mode switch can cause loss of cable ID
	dpcd_write_cable_id_to_dprx(link);

	skip_video_pattern = true;

	if (link_settings->link_rate == LINK_RATE_LOW)
		skip_video_pattern = false;

	if (stream->sink_patches.oled_optimize_display_on)
		set_default_brightness_aux(link);

	if (perform_link_training_with_retries(link_settings,
					       skip_video_pattern,
					       lt_attempts,
					       pipe_ctx,
					       pipe_ctx->stream->signal,
					       do_fallback)) {
		status = DC_OK;
	} else {
		status = DC_FAIL_DP_LINK_TRAINING;
	}

	if (link->preferred_training_settings.fec_enable)
		fec_enable = *link->preferred_training_settings.fec_enable;
	else
		fec_enable = true;

	if (link_dp_get_encoding_format(link_settings) == DP_8b_10b_ENCODING)
		dp_set_fec_enable(link, &pipe_ctx->link_res, fec_enable);

	// during mode set we do DP_SET_POWER off then on, aux writes are lost
	if (link->dpcd_sink_ext_caps.bits.oled == 1 ||
		link->dpcd_sink_ext_caps.bits.sdr_aux_backlight_control == 1 ||
		link->dpcd_sink_ext_caps.bits.hdr_aux_backlight_control == 1) {
		if (!stream->sink_patches.oled_optimize_display_on) {
			set_default_brightness_aux(link);
			if (link->dpcd_sink_ext_caps.bits.oled == 1)
				msleep(bl_oled_enable_delay);
			edp_backlight_enable_aux(link, true);
		} else {
			edp_backlight_enable_aux(link, true);
		}
	}

	return status;
}

static enum dc_status enable_link_edp(
		struct dc_state *state,
		struct pipe_ctx *pipe_ctx)
{
	return enable_link_dp(state, pipe_ctx);
}

static void enable_link_lvds(struct pipe_ctx *pipe_ctx)
{
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->link;
	struct dc *dc = stream->ctx->dc;

	if (stream->phy_pix_clk == 0)
		stream->phy_pix_clk = stream->timing.pix_clk_100hz / 10;

	memset(&stream->link->cur_link_settings, 0,
			sizeof(struct dc_link_settings));
	dc->hwss.enable_lvds_link_output(
			link,
			&pipe_ctx->link_res,
			pipe_ctx->clock_source->id,
			stream->phy_pix_clk);

}

static enum dc_status enable_link_dp_mst(
		struct dc_state *state,
		struct pipe_ctx *pipe_ctx)
{
	struct dc_link *link = pipe_ctx->stream->link;
	unsigned char mstm_cntl = 0;

	/* sink signal type after MST branch is MST. Multiple MST sinks
	 * share one link. Link DP PHY is enable or training only once.
	 */
	if (link->link_status.link_active)
		return DC_OK;

	/* clear payload table */
	core_link_read_dpcd(link, DP_MSTM_CTRL, &mstm_cntl, 1);
	if (mstm_cntl & DP_MST_EN)
		dm_helpers_dp_mst_clear_payload_allocation_table(link->ctx, link);

	/* to make sure the pending down rep can be processed
	 * before enabling the link
	 */
	dm_helpers_dp_mst_poll_pending_down_reply(link->ctx, link);

	/* set the sink to MST mode before enabling the link */
	enable_mst_on_sink(link, true);

	return enable_link_dp(state, pipe_ctx);
}

static enum dc_status enable_link_analog(
		struct dc_state *state,
		struct pipe_ctx *pipe_ctx)
{
	struct dc_link *link = pipe_ctx->stream->link;

	link->dc->hwss.enable_analog_link_output(
		link, pipe_ctx->stream->timing.pix_clk_100hz);

	return DC_OK;
}

static enum dc_status enable_link_virtual(struct pipe_ctx *pipe_ctx)
{
	struct dc_link *link = pipe_ctx->stream->link;

	link->dc->hwss.enable_dp_link_output(link,
			&pipe_ctx->link_res,
			SIGNAL_TYPE_DISPLAY_PORT,
			pipe_ctx->clock_source->id,
			&pipe_ctx->link_config.dp_link_settings);
	return DC_OK;
}

static enum dc_status enable_link(
		struct dc_state *state,
		struct pipe_ctx *pipe_ctx)
{
	enum dc_status status = DC_ERROR_UNEXPECTED;
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = NULL;

	if (stream == NULL)
		return DC_ERROR_UNEXPECTED;
	link = stream->link;

	/* There's some scenarios where driver is unloaded with display
	 * still enabled. When driver is reloaded, it may cause a display
	 * to not light up if there is a mismatch between old and new
	 * link settings. Need to call disable first before enabling at
	 * new link settings.
	 */
	if (link->link_status.link_active)
		disable_link(link, &pipe_ctx->link_res, pipe_ctx->stream->signal);

	switch (pipe_ctx->stream->signal) {
	case SIGNAL_TYPE_DISPLAY_PORT:
		status = enable_link_dp(state, pipe_ctx);
		break;
	case SIGNAL_TYPE_EDP:
		status = enable_link_edp(state, pipe_ctx);
		break;
	case SIGNAL_TYPE_DISPLAY_PORT_MST:
		status = enable_link_dp_mst(state, pipe_ctx);
		msleep(200);
		break;
	case SIGNAL_TYPE_DVI_SINGLE_LINK:
	case SIGNAL_TYPE_DVI_DUAL_LINK:
	case SIGNAL_TYPE_HDMI_TYPE_A:
		enable_link_hdmi(pipe_ctx);
		status = DC_OK;
		break;
	case SIGNAL_TYPE_LVDS:
		enable_link_lvds(pipe_ctx);
		status = DC_OK;
		break;
	case SIGNAL_TYPE_RGB:
		status = enable_link_analog(state, pipe_ctx);
		break;
	case SIGNAL_TYPE_VIRTUAL:
		status = enable_link_virtual(pipe_ctx);
		break;
	default:
		break;
	}

	if (status == DC_OK) {
		pipe_ctx->stream->link->link_status.link_active = true;
	}

	return status;
}

static bool allocate_usb4_bandwidth_for_stream(struct dc_stream_state *stream, int bw)
{
	struct dc_link *link = stream->sink->link;
	int req_bw = bw;

	DC_LOGGER_INIT(link->ctx->logger);

	if (!link->dpia_bw_alloc_config.bw_alloc_enabled)
		return false;

	if (stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST) {
		int sink_index = 0;
		int i = 0;

		for (i = 0; i < link->sink_count; i++) {
			if (link->remote_sinks[i] == NULL)
				continue;

			if (stream->sink->sink_id != link->remote_sinks[i]->sink_id)
				req_bw += link->dpia_bw_alloc_config.remote_sink_req_bw[i];
			else
				sink_index = i;
		}

		link->dpia_bw_alloc_config.remote_sink_req_bw[sink_index] = bw;
	}

	link->dpia_bw_alloc_config.dp_overhead = link_dpia_get_dp_overhead(link);
	req_bw += link->dpia_bw_alloc_config.dp_overhead;

	link_dp_dpia_allocate_usb4_bandwidth_for_stream(link, req_bw);

	if (stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST) {
		int i = 0;

		for (i = 0; i < link->sink_count; i++) {
			if (link->remote_sinks[i] == NULL)
				continue;
			DC_LOG_DEBUG("%s, remote_sink=%s, request_bw=%d\n", __func__,
					(const char *)(&link->remote_sinks[i]->edid_caps.display_name[0]),
					link->dpia_bw_alloc_config.remote_sink_req_bw[i]);
		}
	}

	return true;
}

static bool allocate_usb4_bandwidth(struct dc_stream_state *stream)
{
	bool ret;

	int bw = dc_bandwidth_in_kbps_from_timing(&stream->timing,
			dc_link_get_highest_encoding_format(stream->sink->link));

	ret = allocate_usb4_bandwidth_for_stream(stream, bw);

	return ret;
}

static bool deallocate_usb4_bandwidth(struct dc_stream_state *stream)
{
	bool ret;

	ret = allocate_usb4_bandwidth_for_stream(stream, 0);

	return ret;
}

void link_set_dpms_off(struct pipe_ctx *pipe_ctx)
{
	struct dc  *dc = pipe_ctx->stream->ctx->dc;
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->sink->link;
	struct vpg *vpg = pipe_ctx->stream_res.stream_enc->vpg;
	enum dp_panel_mode panel_mode_dp = dp_get_panel_mode(link);

	DC_LOGGER_INIT(pipe_ctx->stream->ctx->logger);

	ASSERT(is_master_pipe_for_link(link, pipe_ctx));

	if (dp_is_128b_132b_signal(pipe_ctx))
		vpg = pipe_ctx->stream_res.hpo_dp_stream_enc->vpg;
	if (dc_is_virtual_signal(pipe_ctx->stream->signal))
		return;

	if (pipe_ctx->stream->sink) {
		if (pipe_ctx->stream->sink->sink_signal != SIGNAL_TYPE_VIRTUAL &&
			pipe_ctx->stream->sink->sink_signal != SIGNAL_TYPE_NONE) {
			DC_LOG_DC("%s pipe_ctx dispname=%s signal=%x link=%d sink_count=%d\n", __func__,
			pipe_ctx->stream->sink->edid_caps.display_name,
			pipe_ctx->stream->signal, link->link_index, link->sink_count);
		}
	}

	if (!pipe_ctx->stream->sink->edid_caps.panel_patch.skip_avmute) {
		if (dc_is_hdmi_signal(pipe_ctx->stream->signal))
			set_avmute(pipe_ctx, true);
	}

	dc->hwss.disable_audio_stream(pipe_ctx);

	update_psp_stream_config(pipe_ctx, true);
	dc->hwss.blank_stream(pipe_ctx);

	if (pipe_ctx->link_config.dp_tunnel_settings.should_use_dp_bw_allocation)
		deallocate_usb4_bandwidth(pipe_ctx->stream);

	if (pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST)
		deallocate_mst_payload(pipe_ctx);
	else if (dc_is_dp_sst_signal(pipe_ctx->stream->signal) &&
			dp_is_128b_132b_signal(pipe_ctx))
		update_sst_payload(pipe_ctx, false);

	if (dc_is_hdmi_signal(pipe_ctx->stream->signal)) {
		struct ext_hdmi_settings settings = {0};
		enum engine_id eng_id = pipe_ctx->stream_res.stream_enc->id;

		unsigned short masked_chip_caps = link->chip_caps &
				AMD_EXT_DISPLAY_PATH_CAPS__EXT_CHIP_MASK;
		//Need to inform that sink is going to use legacy HDMI mode.
		write_scdc_data(
			link->ddc,
			165000,//vbios only handles 165Mhz.
			false);
		if (masked_chip_caps == AMD_EXT_DISPLAY_PATH_CAPS__HDMI20_TISN65DP159RSBT) {
			/* DP159, Retimer settings */
			if (get_ext_hdmi_settings(pipe_ctx, eng_id, &settings))
				write_i2c_retimer_setting(pipe_ctx,
						false, false, &settings);
			else
				write_i2c_default_retimer_setting(pipe_ctx,
						false, false);
		} else if (masked_chip_caps == AMD_EXT_DISPLAY_PATH_CAPS__HDMI20_PI3EQX1204) {
			/* PI3EQX1204, Redriver settings */
			write_i2c_redriver_setting(pipe_ctx, false);
		}
	}

	if (pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT &&
			!dp_is_128b_132b_signal(pipe_ctx)) {

		/* In DP1.x SST mode, our encoder will go to TPS1
		 * when link is on but stream is off.
		 * Disabling link before stream will avoid exposing TPS1 pattern
		 * during the disable sequence as it will confuse some receivers
		 * state machine.
		 * In DP2 or MST mode, our encoder will stay video active
		 */
		disable_link(pipe_ctx->stream->link, &pipe_ctx->link_res, pipe_ctx->stream->signal);
		dc->hwss.disable_stream(pipe_ctx);
	} else {
		dc->hwss.disable_stream(pipe_ctx);
		disable_link(pipe_ctx->stream->link, &pipe_ctx->link_res, pipe_ctx->stream->signal);
	}
	edp_set_panel_assr(link, pipe_ctx, &panel_mode_dp, false);

	if (pipe_ctx->stream->timing.flags.DSC) {
		if (dc_is_dp_signal(pipe_ctx->stream->signal))
			link_set_dsc_enable(pipe_ctx, false);
	}
	if (dp_is_128b_132b_signal(pipe_ctx)) {
		if (pipe_ctx->stream_res.tg->funcs->set_out_mux)
			pipe_ctx->stream_res.tg->funcs->set_out_mux(pipe_ctx->stream_res.tg, OUT_MUX_DIO);
	}

	if (vpg && vpg->funcs->vpg_powerdown)
		vpg->funcs->vpg_powerdown(vpg);

	/* for psp not exist case */
	if (link->connector_signal == SIGNAL_TYPE_EDP && dc->debug.psp_disabled_wa) {
		/* reset internal save state to default since eDP is  off */
		enum dp_panel_mode panel_mode = dp_get_panel_mode(pipe_ctx->stream->link);
		/* since current psp not loaded, we need to reset it to default */
		link->panel_mode = panel_mode;
	}
}

void link_set_dpms_on(
		struct dc_state *state,
		struct pipe_ctx *pipe_ctx)
{
	struct dc *dc = pipe_ctx->stream->ctx->dc;
	struct dc_stream_state *stream = pipe_ctx->stream;
	struct dc_link *link = stream->sink->link;
	enum dc_status status;
	struct link_encoder *link_enc = pipe_ctx->link_res.dio_link_enc;
	enum otg_out_mux_dest otg_out_dest = OUT_MUX_DIO;
	struct vpg *vpg = pipe_ctx->stream_res.stream_enc->vpg;
	const struct link_hwss *link_hwss = get_link_hwss(link, &pipe_ctx->link_res);
	bool apply_edp_fast_boot_optimization =
		pipe_ctx->stream->apply_edp_fast_boot_optimization;

	DC_LOGGER_INIT(pipe_ctx->stream->ctx->logger);

	ASSERT(is_master_pipe_for_link(link, pipe_ctx));

	if (dp_is_128b_132b_signal(pipe_ctx))
		vpg = pipe_ctx->stream_res.hpo_dp_stream_enc->vpg;
	if (dc_is_virtual_signal(pipe_ctx->stream->signal))
		return;

	if (pipe_ctx->stream->sink) {
		if (pipe_ctx->stream->sink->sink_signal != SIGNAL_TYPE_VIRTUAL &&
			pipe_ctx->stream->sink->sink_signal != SIGNAL_TYPE_NONE) {
			DC_LOG_DC("%s pipe_ctx dispname=%s signal=%x link=%d sink_count=%d\n", __func__,
			pipe_ctx->stream->sink->edid_caps.display_name,
			pipe_ctx->stream->signal,
			link->link_index,
			link->sink_count);
		}
	}

	if (!dc->config.unify_link_enc_assignment)
		link_enc = link_enc_cfg_get_link_enc(link);
	ASSERT(link_enc);

	if (!dc_is_virtual_signal(pipe_ctx->stream->signal)
			&& !dp_is_128b_132b_signal(pipe_ctx)) {
		if (link_enc)
			link_enc->funcs->setup(
				link_enc,
				pipe_ctx->stream->signal);
	}

	pipe_ctx->stream->link->link_state_valid = true;

	if (pipe_ctx->stream_res.tg->funcs->set_out_mux) {
		if (dp_is_128b_132b_signal(pipe_ctx))
			otg_out_dest = OUT_MUX_HPO_DP;
		else
			otg_out_dest = OUT_MUX_DIO;
		pipe_ctx->stream_res.tg->funcs->set_out_mux(pipe_ctx->stream_res.tg, otg_out_dest);
	}

	link_hwss->setup_stream_attribute(pipe_ctx);

	pipe_ctx->stream->apply_edp_fast_boot_optimization = false;

	// Enable VPG before building infoframe
	if (vpg && vpg->funcs->vpg_poweron)
		vpg->funcs->vpg_poweron(vpg);

	resource_build_info_frame(pipe_ctx);
	dc->hwss.update_info_frame(pipe_ctx);

	if (dc_is_dp_signal(pipe_ctx->stream->signal))
		dp_trace_source_sequence(link, DPCD_SOURCE_SEQ_AFTER_UPDATE_INFO_FRAME);

	/* Do not touch link on seamless boot optimization. */
	if (pipe_ctx->stream->apply_seamless_boot_optimization) {
		pipe_ctx->stream->dpms_off = false;

		/* Still enable stream features & audio on seamless boot for DP external displays */
		if (pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT) {
			enable_stream_features(pipe_ctx, false);
			dc->hwss.enable_audio_stream(pipe_ctx);
		}

		update_psp_stream_config(pipe_ctx, false);
		return;
	}

	/* eDP lit up by bios already, no need to enable again. */
	if (pipe_ctx->stream->signal == SIGNAL_TYPE_EDP &&
				apply_edp_fast_boot_optimization &&
				!pipe_ctx->stream->timing.flags.DSC &&
				!pipe_ctx->next_odm_pipe) {
		pipe_ctx->stream->dpms_off = false;
		update_psp_stream_config(pipe_ctx, false);

		if (link->is_dds) {
			uint32_t post_oui_delay = 30; // 30ms

			dpcd_set_source_specific_data(link);
			msleep(post_oui_delay);
		}

		return;
	}

	if (pipe_ctx->stream->dpms_off)
		return;

	/* For Dp tunneling link, a pending HPD means that we have a race condition between processing
	 * current link and processing the pending HPD. If we enable the link now, we may end up with a
	 * link that is not actually connected to a sink. So we skip enabling the link in this case.
	 */
	if (link->ep_type == DISPLAY_ENDPOINT_USB4_DPIA && link->is_hpd_pending) {
		DC_LOG_DEBUG("%s, Link%d HPD is pending, not enable it.\n", __func__, link->link_index);
		return;
	}

	/* Have to setup DSC before DIG FE and BE are connected (which happens before the
	 * link training). This is to make sure the bandwidth sent to DIG BE won't be
	 * bigger than what the link and/or DIG BE can handle. VBID[6]/CompressedStream_flag
	 * will be automatically set at a later time when the video is enabled
	 * (DP_VID_STREAM_EN = 1).
	 */
	if (pipe_ctx->stream->timing.flags.DSC) {
		if (dc_is_dp_signal(pipe_ctx->stream->signal) ||
		    dc_is_virtual_signal(pipe_ctx->stream->signal))
			link_set_dsc_enable(pipe_ctx, true);
	}

	if (link->replay_settings.config.replay_supported && !dc_is_embedded_signal(link->connector_signal))
		dp_setup_replay(link, stream);

	status = enable_link(state, pipe_ctx);

	if (status != DC_OK) {
		DC_LOG_WARNING("enabling link %u failed: %d\n",
		pipe_ctx->stream->link->link_index,
		status);

		/* Abort stream enable *unless* the failure was due to
		 * DP link training - some DP monitors will recover and
		 * show the stream anyway. But MST displays can't proceed
		 * without link training.
		 */
		if (status != DC_FAIL_DP_LINK_TRAINING ||
				pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST) {
			if (false == stream->link->link_status.link_active)
				disable_link(stream->link, &pipe_ctx->link_res,
						pipe_ctx->stream->signal);
			BREAK_TO_DEBUGGER();
			return;
		}
	}

	/* turn off otg test pattern if enable */
	if (pipe_ctx->stream_res.tg->funcs->set_test_pattern)
		pipe_ctx->stream_res.tg->funcs->set_test_pattern(pipe_ctx->stream_res.tg,
				CONTROLLER_DP_TEST_PATTERN_VIDEOMODE,
				COLOR_DEPTH_UNDEFINED);

	/* This second call is needed to reconfigure the DIG
	 * as a workaround for the incorrect value being applied
	 * from transmitter control.
	 */
	if (!(dc_is_virtual_signal(pipe_ctx->stream->signal) ||
			dp_is_128b_132b_signal(pipe_ctx))) {

			if (link_enc)
				link_enc->funcs->setup(
					link_enc,
					pipe_ctx->stream->signal);

		}

	dc->hwss.enable_stream(pipe_ctx);

	/* Set DPS PPS SDP (AKA "info frames") */
	if (pipe_ctx->stream->timing.flags.DSC) {
		if (dc_is_dp_signal(pipe_ctx->stream->signal) ||
				dc_is_virtual_signal(pipe_ctx->stream->signal)) {
			dp_set_dsc_on_rx(pipe_ctx, true);
			link_set_dsc_pps_packet(pipe_ctx, true, true);
		}
	}

	if (dc_is_dp_signal(pipe_ctx->stream->signal))
		dp_set_hblank_reduction_on_rx(pipe_ctx);

	if (pipe_ctx->link_config.dp_tunnel_settings.should_use_dp_bw_allocation)
		allocate_usb4_bandwidth(pipe_ctx->stream);

	if (pipe_ctx->stream->signal == SIGNAL_TYPE_DISPLAY_PORT_MST)
		allocate_mst_payload(pipe_ctx);
	else if (dc_is_dp_sst_signal(pipe_ctx->stream->signal) &&
			dp_is_128b_132b_signal(pipe_ctx))
		update_sst_payload(pipe_ctx, true);

	/* Corruption was observed on systems with display mux when stream gets
	 * enabled after the mux switch. Having a small delay between link
	 * training and stream unblank resolves the corruption issue.
	 * This is workaround.
	 */
	if (pipe_ctx->stream->signal == SIGNAL_TYPE_EDP &&
			link->is_display_mux_present)
		msleep(20);

	if (apple5k_should_defer_pair_finalizer(state, pipe_ctx)) {
		DC_LOG_INFO("APPLE5K-UNBLANK defer link[%u] role=%s tg_inst=%u until GSL pair finalizer\n",
			    link->link_index,
			    dc_link_has_tiled_root_panel_patch(link) ? "root" : "slave",
			    pipe_ctx->stream_res.tg->inst);
		if (dc_is_dp_signal(pipe_ctx->stream->signal))
			enable_stream_features(pipe_ctx, true);
	} else {
		dc->hwss.unblank_stream(pipe_ctx,
			&pipe_ctx->stream->link->cur_link_settings);

		if (stream->sink_patches.delay_ignore_msa > 0)
			msleep(stream->sink_patches.delay_ignore_msa);

		if (dc_is_dp_signal(pipe_ctx->stream->signal))
			enable_stream_features(pipe_ctx, false);
	}
	update_psp_stream_config(pipe_ctx, false);

	dc->hwss.enable_audio_stream(pipe_ctx);

	if (dc_is_hdmi_signal(pipe_ctx->stream->signal)) {
		set_avmute(pipe_ctx, false);
	}
}
