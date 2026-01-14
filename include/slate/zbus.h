// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef SLATE_ZBUS_H
#define SLATE_ZBUS_H

#include <fusain/fusain.h>
#include <zephyr/zbus/zbus.h>

/**
 * Slate Zbus Channel Declarations
 *
 * These channels facilitate communication between shell commands
 * and the serial handler for controlling the remote Helios ICU.
 */

//////////////////////////////////////////////////////////////
// Message Types
//////////////////////////////////////////////////////////////

/**
 * Helios state command message (for Zbus IPC)
 *
 * Used by shell commands to request mode changes from serial handler.
 * Fields match fusain_create_state_command() API.
 */
typedef struct {
	fusain_mode_t mode;
	int32_t argument;
} fusain_state_command_msg_t;

/**
 * Helios telemetry data message
 *
 * Published by serial handler when telemetry is received from Helios ICU
 */
typedef struct {
	fusain_state_t state;
	fusain_error_t error;
	float temperature;
	int32_t motor_rpm;
	int32_t motor_target_rpm;
	bool valid;
} helios_telemetry_msg_t;

//////////////////////////////////////////////////////////////
// Channel Declarations
//////////////////////////////////////////////////////////////

/**
 * Helios state command channel
 *
 * Message type: fusain_state_command_msg_t
 * Publishers: Shell commands
 * Subscribers: Serial handler
 */
ZBUS_CHAN_DECLARE(helios_state_command_chan);

/**
 * Helios telemetry data channel
 *
 * Message type: helios_telemetry_msg_t
 * Publishers: Serial handler
 * Subscribers: Display thread
 */
ZBUS_CHAN_DECLARE(helios_telemetry_chan);

#endif /* SLATE_ZBUS_H */
