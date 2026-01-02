/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Slate Serial Master - Public API
 */

#ifndef SLATE_SERIAL_MASTER_H
#define SLATE_SERIAL_MASTER_H

#include <helios_serial/helios_serial.h>
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
void serial_master_send_ping(void);

/**
 * Send telemetry config command to Helios ICU (protocol v1.2)
 *
 * @param enabled Enable/disable telemetry broadcasts
 * @param interval_ms Telemetry broadcast interval (100-5000 ms)
 * @param mode Telemetry mode (0=bundled, 1=individual)
 */
void serial_master_send_telemetry_config(bool enabled, uint32_t interval_ms,
                                         uint32_t mode);

/**
 * Send set mode command to Helios ICU
 *
 * @param mode Operating mode
 * @param parameter Mode-specific parameter (e.g., pump rate for heating)
 */
void serial_master_set_mode(helios_mode_t mode, uint32_t parameter);

/**
 * Get last known Helios state
 *
 * @param state Output state (can be NULL)
 * @param error Output error (can be NULL)
 */
void serial_master_get_state(helios_state_t* state, helios_error_t* error);

/**
 * Serial TX thread entry point
 * Sends periodic pings to keep Helios connection alive
 */
void serial_tx_thread(void);

/**
 * Serial RX thread entry point
 * Monitors connection status (RX handled by interrupt)
 */
void serial_rx_thread(void);

#endif /* SLATE_SERIAL_MASTER_H */
