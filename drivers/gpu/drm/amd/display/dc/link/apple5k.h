/* SPDX-License-Identifier: MIT */
#ifndef AMD_DC_LINK_APPLE5K_H
#define AMD_DC_LINK_APPLE5K_H

#include "dc.h"

#define APPLE5K_LOG_SUMMARY	0x01
#define APPLE5K_LOG_PANEL	0x02
#define APPLE5K_LOG_LINK		0x04
#define APPLE5K_LOG_POWER	0x08
#define APPLE5K_LOG_TIMING	0x10

void link_apple5k_resolve_policy(struct dc *dc);
bool link_apple5k_log_enabled(const struct dc *dc, uint32_t mask);

struct dc_link *link_apple5k_root_for_link(struct dc_link *link);
bool link_apple5k_is_tile(struct dc_link *link);
const char *link_apple5k_state_name(enum apple5k_tx_state state);
const char *link_apple5k_latch_owner_name(enum apple5k_latch_owner owner);

enum dc_status link_apple5k_write_latch(struct dc_link *link, uint8_t value,
					const char *stage);
enum dc_status link_apple5k_verify_root_latch(struct dc_link *root,
					     const char *stage);
enum dc_status link_apple5k_begin_discovery(struct dc_link *root,
					   const char *stage);
enum dc_status link_apple5k_finish_discovery(struct dc_link *root,
					    const char *stage);
enum dc_status link_apple5k_require_wake_scope(struct dc_link *link,
					      const char *stage);
void link_apple5k_finish_all_discovery(struct dc *dc, const char *stage);

#endif /* AMD_DC_LINK_APPLE5K_H */
