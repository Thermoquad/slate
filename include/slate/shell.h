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

#endif /* SLATE_SHELL_H */
