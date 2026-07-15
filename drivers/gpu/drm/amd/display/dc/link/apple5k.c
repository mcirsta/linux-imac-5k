// SPDX-License-Identifier: MIT
/*
 * Apple internal 5K tiled-panel policy and private latch ownership.
 *
 * All physical DPCD 0x4F1 writes live here.  Generic detection, training,
 * DPMS and shutdown paths must enter through the pair lifecycle helpers.
 */

#include "dm_services.h"
#include "apple5k.h"
#include "protocols/link_dpcd.h"

#include <linux/delay.h>
#include <linux/dmi.h>

#define APPLE5K_DPCD_PANEL_LATCH 0x4f1

extern int amdgpu_apple5k_enable;
extern int amdgpu_apple5k_profile;
extern int amdgpu_apple5k_pair_mode;
extern int amdgpu_apple5k_wake_mode;
extern int amdgpu_apple5k_boot_mode;
extern int amdgpu_apple5k_shutdown_mode;
extern int amdgpu_apple5k_pair_order;
extern int amdgpu_apple5k_discovery_mode;
extern int amdgpu_apple5k_dce12_force_msa_ignore;
extern uint amdgpu_apple5k_log_mask;

#define DC_LOGGER dc_logger
#define DC_LOGGER_INIT(logger) struct dal_logger *dc_logger = (logger)

static void apple5k_apply_profile(struct apple5k_policy *policy, int profile)
{
	switch (profile) {
	case 0:
		break;
	case 1:
		policy->boot_mode = APPLE5K_BOOT_OBSERVE;
		policy->log_mask = 0x1f;
		break;
	case 2:
		policy->pair_mode = APPLE5K_PAIR_TRANSACTIONAL;
		policy->wake_mode = APPLE5K_WAKE_SCOPED;
		break;
	case 3:
		policy->boot_mode = APPLE5K_BOOT_COLD;
		break;
	case 4:
		policy->pair_mode = APPLE5K_PAIR_TRANSACTIONAL;
		policy->wake_mode = APPLE5K_WAKE_SCOPED;
		policy->boot_mode = APPLE5K_BOOT_COLD;
		break;
	case 5:
		policy->pair_mode = APPLE5K_PAIR_TRANSACTIONAL;
		policy->wake_mode = APPLE5K_WAKE_SCOPED;
		policy->shutdown_mode = APPLE5K_SHUTDOWN_OBSERVE;
		policy->log_mask = 0x1f;
		break;
	case 6:
		policy->pair_mode = APPLE5K_PAIR_TRANSACTIONAL;
		policy->wake_mode = APPLE5K_WAKE_SCOPED;
		policy->shutdown_mode = APPLE5K_SHUTDOWN_PAIR_QUIESCE;
		break;
	case 8:
		policy->pair_mode = APPLE5K_PAIR_TRANSACTIONAL;
		policy->wake_mode = APPLE5K_WAKE_SCOPED;
		policy->shutdown_mode = APPLE5K_SHUTDOWN_NEUTRALIZE;
		break;
	case 9:
		policy->pair_mode = APPLE5K_PAIR_TRANSACTIONAL;
		policy->wake_mode = APPLE5K_WAKE_SCOPED;
		policy->dce12_force_msa_ignore = true;
		break;
	case 10:
		policy->pair_mode = APPLE5K_PAIR_LEGACY;
		policy->wake_mode = APPLE5K_WAKE_LEGACY;
		break;
	case 11:
		policy->enabled = false;
		policy->wake_mode = APPLE5K_WAKE_OFF;
		break;
	default:
		/* Invalid profiles are resolved to the safe profile-0 values. */
		break;
	}
}

void link_apple5k_resolve_policy(struct dc *dc)
{
	struct apple5k_policy *policy;
	const char *sys_vendor;
	const char *product;
	bool invalid_profile = false;
	bool invalid_value = false;
	bool supported_family;
	bool supported_dmi;
	uint raw_log_mask;
	int profile;

	if (!dc)
		return;

	policy = &dc->apple5k_policy;
	memset(policy, 0, sizeof(*policy));
	/* Safe profile-0 defaults.  Raw low-level fields are custom-mode only. */
	policy->enabled = true;
	policy->pair_mode = APPLE5K_PAIR_COORDINATED;
	policy->wake_mode = APPLE5K_WAKE_LEGACY;
	policy->boot_mode = APPLE5K_BOOT_INHERIT;
	policy->shutdown_mode = APPLE5K_SHUTDOWN_STOCK;
	policy->pair_order = APPLE5K_ORDER_ROOT_FIRST;
	policy->discovery_mode = amdgpu_apple5k_discovery_mode;
	policy->dce12_force_msa_ignore = false;
	raw_log_mask = amdgpu_apple5k_log_mask;
	policy->log_mask = raw_log_mask;
	profile = amdgpu_apple5k_profile;
	if (profile < 0 || profile > 11 || profile == 7) {
		invalid_profile = true;
		profile = 0;
	}
	if (profile == 0) {
		policy->pair_mode = amdgpu_apple5k_pair_mode;
		policy->wake_mode = amdgpu_apple5k_wake_mode;
		policy->boot_mode = amdgpu_apple5k_boot_mode;
		policy->shutdown_mode = amdgpu_apple5k_shutdown_mode;
		policy->pair_order = amdgpu_apple5k_pair_order;
		policy->dce12_force_msa_ignore =
			amdgpu_apple5k_dce12_force_msa_ignore > 0;
	} else {
		apple5k_apply_profile(policy, profile);
		/* A non-default command-line mask explicitly overrides a preset. */
		if (raw_log_mask != APPLE5K_LOG_SUMMARY)
			policy->log_mask = raw_log_mask;
	}
	if (amdgpu_apple5k_enable != 0 && amdgpu_apple5k_enable != 1)
		invalid_value = true;
	if (amdgpu_apple5k_enable <= 0)
		policy->enabled = false;
	if (profile == 0 && amdgpu_apple5k_dce12_force_msa_ignore != 0 &&
	    amdgpu_apple5k_dce12_force_msa_ignore != 1)
		invalid_value = true;

	if (policy->pair_mode < APPLE5K_PAIR_LEGACY ||
	    policy->pair_mode > APPLE5K_PAIR_TRANSACTIONAL) {
		invalid_value = true;
		policy->pair_mode = APPLE5K_PAIR_COORDINATED;
	}
	if (policy->wake_mode < APPLE5K_WAKE_LEGACY ||
	    policy->wake_mode > APPLE5K_WAKE_OFF) {
		invalid_value = true;
		policy->wake_mode = APPLE5K_WAKE_LEGACY;
	}
	if (policy->boot_mode != APPLE5K_BOOT_INHERIT &&
	    policy->boot_mode != APPLE5K_BOOT_OBSERVE &&
	    policy->boot_mode != APPLE5K_BOOT_COLD) {
		invalid_value = true;
		policy->boot_mode = APPLE5K_BOOT_INHERIT;
	}
	if (policy->shutdown_mode != APPLE5K_SHUTDOWN_STOCK &&
	    policy->shutdown_mode != APPLE5K_SHUTDOWN_OBSERVE &&
	    policy->shutdown_mode != APPLE5K_SHUTDOWN_PAIR_QUIESCE &&
	    policy->shutdown_mode != APPLE5K_SHUTDOWN_NEUTRALIZE) {
		invalid_value = true;
		policy->shutdown_mode = APPLE5K_SHUTDOWN_STOCK;
	}
	if (policy->pair_order < APPLE5K_ORDER_ROOT_FIRST ||
	    policy->pair_order > APPLE5K_ORDER_PIPE) {
		invalid_value = true;
		policy->pair_order = APPLE5K_ORDER_ROOT_FIRST;
	}
	if (policy->discovery_mode < APPLE5K_DISCOVERY_BYPASS ||
	    policy->discovery_mode > APPLE5K_DISCOVERY_PULSE) {
		invalid_value = true;
		policy->discovery_mode = APPLE5K_DISCOVERY_BOUNDED;
	}

	if (policy->pair_mode == APPLE5K_PAIR_TRANSACTIONAL)
		policy->wake_mode = APPLE5K_WAKE_SCOPED;
	if (policy->enabled && policy->wake_mode == APPLE5K_WAKE_OFF) {
		invalid_value = true;
		policy->wake_mode = APPLE5K_WAKE_LEGACY;
	}

	/* Never activate private panel behavior merely because this GPU has DC. */
	sys_vendor = dmi_get_system_info(DMI_SYS_VENDOR);
	product = dmi_get_system_info(DMI_PRODUCT_NAME);
	supported_dmi = sys_vendor && product &&
		(!strcmp(sys_vendor, "Apple Inc.") ||
		 !strcmp(sys_vendor, "Apple Computer, Inc.")) &&
		!strncmp(product, "iMac", 4);
	supported_family = dc->ctx->dce_version == DCE_VERSION_11_2 ||
			   dc->ctx->dce_version == DCE_VERSION_12_0;
	if (!supported_dmi || !supported_family)
		policy->enabled = false;
	if (!policy->enabled)
		policy->wake_mode = APPLE5K_WAKE_OFF;

	if (policy->log_mask && supported_dmi) {
		DC_LOGGER_INIT(dc->ctx->logger);
		if (invalid_profile)
			DC_LOG_ERROR("APPLE5K-POLICY invalid profile=%d; using profile 0\n",
				     amdgpu_apple5k_profile);
		if (invalid_value)
			DC_LOG_ERROR("APPLE5K-POLICY invalid custom field or combination; "
				     "using safe effective values\n");
		DC_LOG_INFO("APPLE5K-POLICY profile=%d enabled=%d pair=%d wake=%d boot=%d shutdown=%d order=%d discovery=%d msa_ignore=%d log_mask=0x%x\n",
			    profile, policy->enabled, policy->pair_mode,
			    policy->wake_mode, policy->boot_mode,
			    policy->shutdown_mode, policy->pair_order,
			    policy->discovery_mode,
			    policy->dce12_force_msa_ignore, policy->log_mask);
	}
}

bool link_apple5k_log_enabled(const struct dc *dc, uint32_t mask)
{
	return dc && dc->apple5k_policy.enabled &&
	       (dc->apple5k_policy.log_mask & mask);
}

struct dc_link *link_apple5k_root_for_link(struct dc_link *link)
{
	if (dc_link_has_tiled_root_panel_patch(link))
		return link;
	if (dc_link_has_tiled_slave_panel_patch(link) && link->tiled_peer &&
	    dc_link_has_tiled_root_panel_patch(link->tiled_peer))
		return link->tiled_peer;
	return NULL;
}

bool link_apple5k_is_tile(struct dc_link *link)
{
	return link_apple5k_root_for_link(link) != NULL;
}

const char *link_apple5k_state_name(enum apple5k_tx_state state)
{
	switch (state) {
	case APPLE5K_TX_IDLE: return "IDLE";
	case APPLE5K_TX_DISCOVERING: return "DISCOVERING";
	case APPLE5K_TX_READY: return "READY";
	case APPLE5K_TX_ENABLING: return "ENABLING";
	case APPLE5K_TX_NATIVE: return "NATIVE";
	case APPLE5K_TX_QUIESCING: return "QUIESCING";
	case APPLE5K_TX_ABORTING: return "ABORTING";
	case APPLE5K_TX_BLOCKED: return "BLOCKED";
	default: return "?";
	}
}

const char *link_apple5k_latch_owner_name(enum apple5k_latch_owner owner)
{
	switch (owner) {
	case APPLE5K_LATCH_NONE: return "NONE";
	case APPLE5K_LATCH_DISCOVERY: return "DISCOVERY";
	case APPLE5K_LATCH_ENABLE: return "ENABLE";
	case APPLE5K_LATCH_TEARDOWN: return "TEARDOWN";
	default: return "?";
	}
}

static enum dc_status apple5k_write_latch_locked(struct dc_link *link,
						 struct dc_link *root,
						 uint8_t value,
						 const char *stage)
{
	uint8_t readback = 0;
	enum dc_status write_status, read_status;

	lockdep_assert_held(&root->apple5k.lock);
	if (value > 1)
		return DC_ERROR_UNEXPECTED;
	/* Diagnostic modes may not escape into a later native transaction. */
	if (root->dc->apple5k_policy.discovery_mode ==
				APPLE5K_DISCOVERY_BYPASS)
		return DC_ERROR_UNEXPECTED;
	if (root->dc->apple5k_policy.discovery_mode ==
				APPLE5K_DISCOVERY_PULSE &&
	    (link != root ||
	     root->apple5k.state != APPLE5K_TX_DISCOVERING ||
	     root->apple5k.latch_owner != APPLE5K_LATCH_DISCOVERY ||
	     root->apple5k.discovery_pulse_done))
		return DC_ERROR_UNEXPECTED;
	if (value && root->apple5k.block_rearm) {
		return DC_ERROR_UNEXPECTED;
	}
	if (root->dc->apple5k_policy.pair_mode ==
					APPLE5K_PAIR_TRANSACTIONAL) {
		if (root->apple5k.latch_owner == APPLE5K_LATCH_NONE)
			return DC_ERROR_UNEXPECTED;
		if (value &&
		    root->apple5k.state != APPLE5K_TX_DISCOVERING &&
		    root->apple5k.state != APPLE5K_TX_ENABLING)
			return DC_ERROR_UNEXPECTED;
		if (value && link != root &&
		    root->apple5k.latch_owner != APPLE5K_LATCH_ENABLE)
			return DC_ERROR_UNEXPECTED;
	}

	write_status = core_link_write_dpcd(link, APPLE5K_DPCD_PANEL_LATCH,
					   &value, sizeof(value));
	read_status = core_link_read_dpcd(link, APPLE5K_DPCD_PANEL_LATCH,
					  &readback, sizeof(readback));
	if (value)
		root->apple5k.arm_count++;
	else
		root->apple5k.disarm_count++;

	if (link_apple5k_log_enabled(root->dc, APPLE5K_LOG_PANEL) ||
	    write_status != DC_OK || read_status != DC_OK ||
	    readback != value) {
		DC_LOGGER_INIT(root->ctx->logger);
		DC_LOG_INFO("APPLE5K-LATCH gen=%u stage=%s state=%s owner=%s root[%u] target[%u] value=0x%02x write_s=%d read_s=%d readback=0x%02x arms=%u disarms=%u\n",
			    root->apple5k.generation, stage ? stage : "?",
			    link_apple5k_state_name(root->apple5k.state),
			    link_apple5k_latch_owner_name(
					root->apple5k.latch_owner),
			    root->link_index, link->link_index, value,
			    write_status, read_status, readback,
			    root->apple5k.arm_count, root->apple5k.disarm_count);
	}

	if (write_status != DC_OK || read_status != DC_OK || readback != value)
		return DC_ERROR_UNEXPECTED;
	return DC_OK;
}

enum dc_status link_apple5k_write_latch(struct dc_link *link, uint8_t value,
					const char *stage)
{
	struct dc_link *root = link_apple5k_root_for_link(link);
	enum dc_status status;

	if (!link || !root || !link->local_sink ||
	    !root->dc->apple5k_policy.enabled)
		return DC_ERROR_UNEXPECTED;

	mutex_lock(&root->apple5k.lock);
	status = apple5k_write_latch_locked(link, root, value, stage);
	mutex_unlock(&root->apple5k.lock);
	return status;
}

static enum dc_status apple5k_verify_root_latch_locked(struct dc_link *root,
						       const char *stage)
{
	uint8_t value = 0;
	enum dc_status status;

	lockdep_assert_held(&root->apple5k.lock);
	status = core_link_read_dpcd(root, APPLE5K_DPCD_PANEL_LATCH,
				     &value, sizeof(value));
	if (link_apple5k_log_enabled(root->dc, APPLE5K_LOG_PANEL) ||
	    status != DC_OK || value != 1) {
		DC_LOGGER_INIT(root->ctx->logger);
		DC_LOG_INFO("APPLE5K-LATCH verify stage=%s state=%s owner=%s root[%u] status=%d value=0x%02x\n",
			    stage ? stage : "?",
			    link_apple5k_state_name(root->apple5k.state),
			    link_apple5k_latch_owner_name(
					root->apple5k.latch_owner),
			    root->link_index, status, value);
	}
	return status == DC_OK && value == 1 ? DC_OK : DC_ERROR_UNEXPECTED;
}

enum dc_status link_apple5k_verify_root_latch(struct dc_link *root,
					     const char *stage)
{
	enum dc_status status;

	root = link_apple5k_root_for_link(root);
	if (!root || !root->local_sink)
		return DC_ERROR_UNEXPECTED;
	mutex_lock(&root->apple5k.lock);
	status = apple5k_verify_root_latch_locked(root, stage);
	mutex_unlock(&root->apple5k.lock);
	return status;
}

enum dc_status link_apple5k_begin_discovery(struct dc_link *root,
					   const char *stage)
{
	uint8_t r41c = 0, r425 = 0, r4f1 = 0;
	enum dc_status s41c, s425, s4f1;
	bool needs_arm;
	enum dc_status status;

	root = link_apple5k_root_for_link(root);
	if (!root || !root->dc->apple5k_policy.enabled ||
	    root->dc->apple5k_policy.wake_mode == APPLE5K_WAKE_OFF)
		return DC_ERROR_UNEXPECTED;

	if (root->dc->apple5k_policy.wake_mode == APPLE5K_WAKE_LEGACY &&
	    root->dc->apple5k_policy.discovery_mode ==
				APPLE5K_DISCOVERY_BOUNDED)
		return link_apple5k_write_latch(root, 1, stage);

	mutex_lock(&root->apple5k.lock);
	if (root->dc->apple5k_policy.discovery_mode ==
				APPLE5K_DISCOVERY_PULSE &&
	    root->apple5k.discovery_pulse_done) {
		mutex_unlock(&root->apple5k.lock);
		return DC_OK;
	}
	if (root->apple5k.state == APPLE5K_TX_DISCOVERING ||
	    root->apple5k.state == APPLE5K_TX_ENABLING ||
	    root->apple5k.state == APPLE5K_TX_NATIVE) {
		status = apple5k_verify_root_latch_locked(root, stage);
		mutex_unlock(&root->apple5k.lock);
		return status;
	}
	if (root->apple5k.state == APPLE5K_TX_BLOCKED ||
	    root->apple5k.block_rearm) {
		mutex_unlock(&root->apple5k.lock);
		return DC_ERROR_UNEXPECTED;
	}
	mutex_unlock(&root->apple5k.lock);

	s41c = core_link_read_dpcd(root, 0x41c, &r41c, sizeof(r41c));
	s425 = core_link_read_dpcd(root, 0x425, &r425, sizeof(r425));
	s4f1 = core_link_read_dpcd(root, APPLE5K_DPCD_PANEL_LATCH,
				     &r4f1, sizeof(r4f1));
	if (root->dc->apple5k_policy.discovery_mode ==
				APPLE5K_DISCOVERY_BYPASS) {
		if (link_apple5k_log_enabled(root->dc,
					     APPLE5K_LOG_SUMMARY)) {
			DC_LOGGER_INIT(root->ctx->logger);
			DC_LOG_INFO("APPLE5K-TXN discovery bypass root[%u] tuple=%02x(s=%d)/%02x(s=%d)/%02x(s=%d) action=read-only\n",
				    root->link_index, r41c, s41c, r425, s425,
				    r4f1, s4f1);
		}
		return s41c == DC_OK && s425 == DC_OK && s4f1 == DC_OK ?
				DC_OK : DC_ERROR_UNEXPECTED;
	}
	if (s41c != DC_OK || s425 != DC_OK || s4f1 != DC_OK) {
		DC_LOGGER_INIT(root->ctx->logger);
		DC_LOG_ERROR("APPLE5K-TXN discovery blocked: tuple read failed root[%u] 41c_s=%d 425_s=%d 4f1_s=%d\n",
			     root->link_index, s41c, s425, s4f1);
		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_BLOCKED;
		root->apple5k.block_rearm = true;
		mutex_unlock(&root->apple5k.lock);
		return DC_ERROR_UNEXPECTED;
	}
	needs_arm = r41c == 0x05 && r425 == 0x02 && r4f1 == 0x00;
	if (!needs_arm &&
	    !(r41c == 0x15 && r425 == 0x00 && r4f1 == 0x01)) {
		DC_LOGGER_INIT(root->ctx->logger);
		DC_LOG_ERROR("APPLE5K-TXN discovery blocked: mixed/unknown root[%u] tuple=%02x/%02x/%02x\n",
			     root->link_index, r41c, r425, r4f1);
		mutex_lock(&root->apple5k.lock);
		root->apple5k.state = APPLE5K_TX_BLOCKED;
		root->apple5k.block_rearm = true;
		mutex_unlock(&root->apple5k.lock);
		return DC_ERROR_UNEXPECTED;
	}

	mutex_lock(&root->apple5k.lock);
	if (root->apple5k.state == APPLE5K_TX_DISCOVERING ||
	    root->apple5k.state == APPLE5K_TX_ENABLING ||
	    root->apple5k.state == APPLE5K_TX_NATIVE) {
		status = apple5k_verify_root_latch_locked(root, stage);
		mutex_unlock(&root->apple5k.lock);
		return status;
	}
	if (root->apple5k.state == APPLE5K_TX_BLOCKED ||
	    root->apple5k.block_rearm) {
		mutex_unlock(&root->apple5k.lock);
		return DC_ERROR_UNEXPECTED;
	}
	root->apple5k.generation++;
	root->apple5k.state = APPLE5K_TX_DISCOVERING;
	root->apple5k.latch_owner = APPLE5K_LATCH_DISCOVERY;
	status = needs_arm ?
		apple5k_write_latch_locked(root, root, 1, stage) :
		apple5k_verify_root_latch_locked(root, stage);
	if (status == DC_OK) {
		mutex_unlock(&root->apple5k.lock);
		msleep(10);
		if (root->dc->apple5k_policy.discovery_mode ==
					APPLE5K_DISCOVERY_PULSE)
			return link_apple5k_finish_discovery(root,
							      "discovery-pulse");
		return DC_OK;
	}

	/* A failed write/readback leaves the physical latch ambiguous. */
	root->apple5k.state = APPLE5K_TX_BLOCKED;
	root->apple5k.block_rearm = true;
	mutex_unlock(&root->apple5k.lock);
	return status;
}

enum dc_status link_apple5k_finish_discovery(struct dc_link *root,
					    const char *stage)
{
	uint8_t r41c = 0, r425 = 0;
	enum dc_status status;

	root = link_apple5k_root_for_link(root);
	if (!root ||
	    (root->dc->apple5k_policy.wake_mode != APPLE5K_WAKE_SCOPED &&
	     root->dc->apple5k_policy.discovery_mode !=
				APPLE5K_DISCOVERY_PULSE))
		return DC_OK;

	mutex_lock(&root->apple5k.lock);
	if (root->apple5k.state != APPLE5K_TX_DISCOVERING ||
	    root->apple5k.latch_owner != APPLE5K_LATCH_DISCOVERY) {
		mutex_unlock(&root->apple5k.lock);
		return DC_OK;
	}
	status = apple5k_write_latch_locked(root, root, 0, stage);
	msleep(10);
	if (status == DC_OK &&
	    (core_link_read_dpcd(root, 0x41c, &r41c, sizeof(r41c)) != DC_OK ||
	     core_link_read_dpcd(root, 0x425, &r425, sizeof(r425)) != DC_OK ||
	     r41c != 0x05 || r425 != 0x02))
		status = DC_ERROR_UNEXPECTED;
	root->apple5k.state = status == DC_OK ? APPLE5K_TX_READY :
						       APPLE5K_TX_BLOCKED;
	root->apple5k.block_rearm = status != DC_OK;
	if (status == DC_OK) {
		root->apple5k.latch_owner = APPLE5K_LATCH_NONE;
		if (root->dc->apple5k_policy.discovery_mode ==
					APPLE5K_DISCOVERY_PULSE)
			root->apple5k.discovery_pulse_done = true;
	}
	mutex_unlock(&root->apple5k.lock);
	if (status != DC_OK) {
		DC_LOGGER_INIT(root->ctx->logger);
		DC_LOG_ERROR("APPLE5K-TXN discovery cleanup blocked root[%u] tuple=%02x/%02x/00\n",
			     root->link_index, r41c, r425);
	} else if (link_apple5k_log_enabled(root->dc,
						 APPLE5K_LOG_SUMMARY)) {
		DC_LOGGER_INIT(root->ctx->logger);
		DC_LOG_INFO("APPLE5K-TXN discovery closed root[%u] tuple=%02x/%02x/00 state=READY owner=none\n",
			    root->link_index, r41c, r425);
	}
	return status;
}

/*
 * Lower source-DPCD and training helpers may make the slave receiver ready,
 * but transactional mode must already own discovery or native enable before
 * they run.  Legacy/coordinated mode retains the established wake write as a
 * regression control.
 */
enum dc_status link_apple5k_require_wake_scope(struct dc_link *link,
					      const char *stage)
{
	struct dc_link *root = link_apple5k_root_for_link(link);
	enum dc_status status;

	if (!root || !root->dc->apple5k_policy.enabled)
		return DC_ERROR_UNEXPECTED;
	if (root->dc->apple5k_policy.discovery_mode !=
				APPLE5K_DISCOVERY_BOUNDED)
		return DC_ERROR_UNEXPECTED;
	if (root->dc->apple5k_policy.pair_mode !=
					APPLE5K_PAIR_TRANSACTIONAL)
		return link_apple5k_write_latch(root, 1, stage);

	mutex_lock(&root->apple5k.lock);
	if ((root->apple5k.state != APPLE5K_TX_DISCOVERING ||
	     root->apple5k.latch_owner != APPLE5K_LATCH_DISCOVERY) &&
	    (root->apple5k.state != APPLE5K_TX_ENABLING ||
	     root->apple5k.latch_owner != APPLE5K_LATCH_ENABLE)) {
		mutex_unlock(&root->apple5k.lock);
		return DC_ERROR_UNEXPECTED;
	}
	status = apple5k_verify_root_latch_locked(root, stage);
	mutex_unlock(&root->apple5k.lock);
	return status;
}

void link_apple5k_finish_all_discovery(struct dc *dc, const char *stage)
{
	int i;

	if (!dc || dc->apple5k_policy.wake_mode != APPLE5K_WAKE_SCOPED)
		return;
	for (i = 0; i < dc->link_count; i++)
		if (dc_link_has_tiled_root_panel_patch(dc->links[i]))
			link_apple5k_finish_discovery(dc->links[i], stage);
}

void link_apple_5k_finish_detection(struct dc_link *link,
				    enum dc_detect_reason reason)
{
	struct dc_link *root;

	/* Boot detection keeps the scope through the DM root-EDID re-read. */
	if (!link || reason == DETECT_REASON_BOOT ||
	    !dc_link_has_tiled_slave_panel_patch(link))
		return;
	root = link_apple5k_root_for_link(link);
	if (root)
		link_apple5k_finish_discovery(root, "detect-complete");
}
