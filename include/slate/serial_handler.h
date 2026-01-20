/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Slate Serial Master - Public API
 */

#ifndef SLATE_SERIAL_MASTER_H
#define SLATE_SERIAL_MASTER_H

#include <fusain/fusain.h>
#include <stdint.h>

/**
 * Initialize serial master communication
 *
 * @return 0 on success, negative on error
 */
int serial_master_init(void);

/**
 * Send ping request to Helios ICU
 */
void helios_send_ping(void);

/**
 * Send telemetry config command to Helios ICU
 *
 * @param enabled Enable/disable telemetry broadcasts
 * @param interval_ms Telemetry broadcast interval (100-5000 ms)
 */
void helios_send_telemetry_config(bool enabled, uint32_t interval_ms);

/**
 * Send state command to Helios ICU
 *
 * @param mode Operating mode
 * @param argument Mode-specific argument (e.g., RPM for FAN, pump rate for HEAT)
 */
void helios_set_mode(fusain_mode_t mode, int32_t argument);

/**
 * Get last known Helios state
 *
 * @param state Output state (can be NULL)
 * @param error Output error (can be NULL)
 */
void helios_get_state(fusain_state_t* state, fusain_error_t* error);

/**
 * Serial RX thread entry point
 * Handles UART RX polling and packet decoding only
 * Priority: -2 (highest), Loop: 500µs
 */
int serial_rx_thread(void);

/**
 * Serial TX thread entry point
 * Handles UART TX polling and TX queue processing
 * Priority: 0, Loop: 1ms
 */
int serial_tx_thread(void);

/**
 * Serial processing thread entry point
 * Handles protocol logic, packet processing, timeouts, and retransmissions
 * Priority: 1, Loop: 10ms
 */
int serial_processing_thread(void);

/**
 * Serial handler statistics
 */
struct serial_handler_stats {
  uint32_t bytes_received;
  uint32_t packets_decoded;
  bool ping_response_received;
  bool telemetry_received;
  uint32_t helios_uptime_ms;
};

/**
 * Get serial handler statistics
 */
void serial_handler_get_stats(struct serial_handler_stats* stats);

#endif /* SLATE_SERIAL_MASTER_H */
