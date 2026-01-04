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
 *
 * Message structures are defined in fusain/fusain.h:
 * - helios_state_command_msg_t
 */

//////////////////////////////////////////////////////////////
// Message Types
//////////////////////////////////////////////////////////////

/**
 * Helios telemetry data message
 *
 * Published by serial handler when telemetry is received from Helios ICU
 */
typedef struct {
	helios_state_t state;
	helios_error_t error;
	double temperature;
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
 * Message type: helios_state_command_msg_t
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
