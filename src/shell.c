// SPDX-License-Identifier: GPL-2.0-or-later
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <slate/serial_handler.h>
#include <slate/shell.h>
#include <slate/zbus.h>

LOG_MODULE_REGISTER(slate_shell);

#define PUB_TIMEOUT K_SECONDS(1)

// State names for display
static const char* const fusain_state_names[] = {
  "INIT",    // FUSAIN_STATE_INITIALIZING
  "IDLE",    // FUSAIN_STATE_IDLE
  "BLOWING", // FUSAIN_STATE_BLOWING
  "PREHEAT", // FUSAIN_STATE_PREHEAT
  "PRE_ST2", // FUSAIN_STATE_PREHEAT_STAGE_2
  "HEATING", // FUSAIN_STATE_HEATING
  "COOLING", // FUSAIN_STATE_COOLING
  "ERROR",   // FUSAIN_STATE_ERROR
  "E_STOP",  // FUSAIN_STATE_E_STOP
};

// Mode names for display
static const char* const fusain_mode_names[] = {
  "IDLE",      // FUSAIN_MODE_IDLE
  "FAN",       // FUSAIN_MODE_FAN
  "HEAT",      // FUSAIN_MODE_HEAT
  [255] = "EMERGENCY", // FUSAIN_MODE_EMERGENCY
};

//////////////////////////////////////////////////////////////
// Helper Functions
//////////////////////////////////////////////////////////////

/**
 * Send a state command to Helios via Zbus
 *
 * @param sh Shell instance
 * @param cmd Command message to send
 * @return 0 on success, negative error code on failure
 */
static int send_state_command(const struct shell* sh, fusain_state_command_msg_t* cmd)
{
  int ret = zbus_chan_pub(&helios_state_command_chan, cmd, PUB_TIMEOUT);
  if (ret != 0) {
    LOG_ERR("Failed to publish state command: %d", ret);
    shell_print(sh, "Error: Unable to send command to Helios");
    return ret;
  }

  shell_print(sh, "Sending %s command to Helios (parameter: %u)",
      fusain_mode_names[cmd->mode], cmd->parameter);
  return 0;
}

//////////////////////////////////////////////////////////////
// Shell Command Implementations
//////////////////////////////////////////////////////////////

int cmd_helios(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  shell_print(sh, "Slate controller - Helios remote control");
  shell_print(sh, "Use 'helios state' subcommands to control the burner");
  return 0;
}

int cmd_get_state(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  fusain_state_t state;
  fusain_error_t error;

  helios_get_state(&state, &error);

  shell_print(sh, "Helios State: %s (%d)", fusain_state_names[state], state);

  if (error != FUSAIN_ERROR_NONE) {
    shell_print(sh, "Helios Error: %d", error);
  }

  return 0;
}

int cmd_set_idle(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  fusain_state_command_msg_t cmd = {
    .mode = FUSAIN_MODE_IDLE,
    .parameter = 0,
  };

  return send_state_command(sh, &cmd);
}

int cmd_set_fan(const struct shell* sh, size_t argc, char** argv)
{
  if (argc != 2) {
    shell_print(sh, "Usage: helios state fan <rpm>");
    return -EINVAL;
  }

  int rpm = atoi(argv[1]);
  if (rpm < 0) {
    shell_print(sh, "Error: RPM must be positive");
    return -EINVAL;
  }

  fusain_state_command_msg_t cmd = {
    .mode = FUSAIN_MODE_FAN,
    .parameter = (uint32_t)rpm,
  };

  return send_state_command(sh, &cmd);
}

int cmd_set_heat(const struct shell* sh, size_t argc, char** argv)
{
  if (argc != 2) {
    shell_print(sh, "Usage: helios state heat <pump_rate_ms>");
    return -EINVAL;
  }

  int pump_rate = atoi(argv[1]);
  if (pump_rate < 100) {
    shell_print(sh, "Error: Pump rate must be >= 100ms");
    return -EINVAL;
  }

  fusain_state_command_msg_t cmd = {
    .mode = FUSAIN_MODE_HEAT,
    .parameter = (uint32_t)pump_rate,
  };

  return send_state_command(sh, &cmd);
}
