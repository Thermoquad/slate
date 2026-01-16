// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef SLATE_SHELL_H
#define SLATE_SHELL_H

#include <zephyr/shell/shell.h>

/**
 * Slate Shell Commands
 *
 * Interactive commands for controlling the remote Helios ICU
 * via serial protocol.
 */

/**
 * Helios root command
 */
int cmd_helios(const struct shell* sh, size_t argc, char** argv);

/**
 * Get Helios state
 *
 * Displays the current state received via telemetry from Helios
 */
int cmd_get_state(const struct shell* sh, size_t argc, char** argv);

/**
 * Set Helios to IDLE mode
 *
 * Sends SET_MODE command with HELIOS_MODE_IDLE
 */
int cmd_set_idle(const struct shell* sh, size_t argc, char** argv);

/**
 * Set Helios to FAN mode
 *
 * Usage: helios state fan <rpm>
 * Sends SET_MODE command with HELIOS_MODE_FAN and target RPM
 */
int cmd_set_fan(const struct shell* sh, size_t argc, char** argv);

/**
 * Set Helios to HEAT mode
 *
 * Usage: helios state heat <pump_rate_ms>
 * Sends SET_MODE command with HELIOS_MODE_HEAT and pump rate parameter
 */
int cmd_set_heat(const struct shell* sh, size_t argc, char** argv);

/**
 * Slate root command
 */
int cmd_slate(const struct shell* sh, size_t argc, char** argv);

/**
 * Slate WiFi configuration command
 */
int cmd_slate_wifi(const struct shell* sh, size_t argc, char** argv);

/**
 * Save WiFi credentials to NVS
 *
 * Usage: slate wifi save <ssid> <password>
 * Stores WiFi network credentials in non-volatile storage
 */
int cmd_wifi_save(const struct shell* sh, size_t argc, char** argv);

/**
 * Show current WiFi settings
 *
 * Displays auto-connect status and reconnect interval
 */
int cmd_wifi_settings(const struct shell* sh, size_t argc, char** argv);

/**
 * Show WiFi connection status
 *
 * Displays detailed WiFi interface status including state, SSID, BSSID,
 * security, signal strength, and link parameters
 */
int cmd_wifi_status(const struct shell* sh, size_t argc, char** argv);

/**
 * Enable WiFi auto-connect
 *
 * Usage: slate wifi auto-connect enable
 * Enable automatic WiFi connection on boot and on disconnect
 */
int cmd_wifi_auto_connect_enable(const struct shell* sh, size_t argc, char** argv);

/**
 * Disable WiFi auto-connect
 *
 * Usage: slate wifi auto-connect disable
 * Disable automatic WiFi connection
 */
int cmd_wifi_auto_connect_disable(const struct shell* sh, size_t argc, char** argv);

/**
 * Set WiFi reconnect interval
 *
 * Usage: slate wifi reconnect-interval <seconds>
 * Set reconnect interval (10-3600 seconds) for WiFi reconnection attempts
 */
int cmd_wifi_reconnect_interval(const struct shell* sh, size_t argc, char** argv);

/**
 * Shell command structure for helios state subcommands
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_helios_state,
    SHELL_CMD(fan, NULL, "Set fan mode with RPM", cmd_set_fan),
    SHELL_CMD(idle, NULL, "Set idle mode (shutdown)", cmd_set_idle),
    SHELL_CMD(heat, NULL, "Set heat mode with pump rate", cmd_set_heat),
    SHELL_SUBCMD_SET_END);

/**
 * Root helios command structure
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_helios,
    SHELL_CMD(state, &sub_helios_state, "Control Helios state", cmd_get_state),
    SHELL_SUBCMD_SET_END);

/**
 * Register helios command with shell
 */
SHELL_CMD_REGISTER(helios, &sub_helios, "Helios ICU control", cmd_helios);

/**
 * WiFi auto-connect subcommands
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_slate_wifi_auto_connect,
    SHELL_CMD(enable, NULL, "Enable WiFi auto-connect", cmd_wifi_auto_connect_enable),
    SHELL_CMD(disable, NULL, "Disable WiFi auto-connect", cmd_wifi_auto_connect_disable),
    SHELL_SUBCMD_SET_END);

/**
 * Slate WiFi subcommands
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_slate_wifi,
    SHELL_CMD(save, NULL, "Save WiFi credentials <ssid> <password>", cmd_wifi_save),
    SHELL_CMD(auto-connect, &sub_slate_wifi_auto_connect, "Configure auto-connect", cmd_slate_wifi),
    SHELL_CMD(reconnect-interval, NULL, "Set reconnect interval (10-3600 seconds)", cmd_wifi_reconnect_interval),
    SHELL_CMD(settings, NULL, "Show WiFi settings", cmd_wifi_settings),
    SHELL_CMD(status, NULL, "Show WiFi connection status", cmd_wifi_status),
    SHELL_SUBCMD_SET_END);

/**
 * Root slate command structure
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_slate,
    SHELL_CMD(wifi, &sub_slate_wifi, "WiFi configuration", cmd_slate_wifi),
    SHELL_SUBCMD_SET_END);

/**
 * Register slate command with shell
 */
SHELL_CMD_REGISTER(slate, &sub_slate, "Slate controller configuration", cmd_slate);

#endif /* SLATE_SHELL_H */
