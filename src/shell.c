// SPDX-License-Identifier: GPL-2.0-or-later

#include <slate/shell.h>
#include <slate/zbus.h>
#include <slate/serial_master.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(slate_shell);

#define PUB_TIMEOUT K_SECONDS(1)

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
static int send_state_command(const struct shell* sh, helios_state_command_msg_t* cmd)
{
  int ret = zbus_chan_pub(&helios_state_command_chan, cmd, PUB_TIMEOUT);
  if (ret != 0) {
    LOG_ERR("Failed to publish state command: %d", ret);
    shell_print(sh, "Error: Unable to send command to Helios");
    return ret;
  }

  shell_print(sh, "Sending %s command to Helios (parameter: %u)",
      helios_mode_names[cmd->mode], cmd->parameter);
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

  helios_state_t state;
  helios_error_t error;

  serial_master_get_state(&state, &error);

  shell_print(sh, "Helios State: %s (%d)", helios_state_names[state], state);

  if (error != HELIOS_ERROR_NONE) {
    shell_print(sh, "Helios Error: %d", error);
  }

  return 0;
}

int cmd_set_idle(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  helios_state_command_msg_t cmd = {
    .mode = HELIOS_MODE_IDLE,
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

  helios_state_command_msg_t cmd = {
    .mode = HELIOS_MODE_FAN,
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

  helios_state_command_msg_t cmd = {
    .mode = HELIOS_MODE_HEAT,
    .parameter = (uint32_t)pump_rate,
  };

  return send_state_command(sh, &cmd);
}
