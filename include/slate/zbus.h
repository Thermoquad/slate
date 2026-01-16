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

/**
 * WiFi command types
 */
typedef enum {
	WIFI_CMD_CONNECT,           // Connect with SSID/password
	WIFI_CMD_DISCONNECT,        // Disconnect from network
	WIFI_CMD_SET_AUTO_CONNECT,  // Enable/disable auto-connect
	WIFI_CMD_SET_RECONNECT_INTERVAL, // Set reconnect interval
} wifi_command_type_t;

/**
 * WiFi command message
 *
 * Used by shell commands to configure WiFi at runtime
 */
typedef struct {
	wifi_command_type_t type;
	union {
		struct {
			char ssid[33];      // SSID (max 32 + null terminator)
			char password[64];  // Password (max 63 + null terminator)
		} connect;
		bool auto_connect;      // For SET_AUTO_CONNECT
		uint32_t reconnect_interval; // For SET_RECONNECT_INTERVAL (seconds)
	};
} wifi_command_msg_t;

/**
 * WiFi connection state
 */
typedef enum {
	SLATE_WIFI_STATE_DISCONNECTED,
	SLATE_WIFI_STATE_CONNECTING,
	SLATE_WIFI_STATE_CONNECTED,
	SLATE_WIFI_STATE_FAILED,
} slate_wifi_state_t;

/**
 * WiFi status message
 *
 * Published by WiFi thread when connection state changes
 */
typedef struct {
	slate_wifi_state_t state;
	bool auto_connect;
	uint32_t reconnect_interval; // seconds
	char ssid[33];               // Current/target SSID
	char ipv4_address[16];       // IPv4 address (if connected)
	char ipv6_address[46];       // IPv6 address (if connected, prefers global over link-local)
	char hostname[64];           // Device hostname
} wifi_status_msg_t;

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

/**
 * WiFi command channel
 *
 * Message type: wifi_command_msg_t
 * Publishers: Shell commands
 * Subscribers: WiFi thread
 */
ZBUS_CHAN_DECLARE(wifi_command_chan);

/**
 * WiFi status channel
 *
 * Message type: wifi_status_msg_t
 * Publishers: WiFi thread
 * Subscribers: Shell, future network status display
 */
ZBUS_CHAN_DECLARE(wifi_status_chan);

#endif /* SLATE_ZBUS_H */
