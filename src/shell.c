// SPDX-License-Identifier: GPL-2.0-or-later
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>

#include <slate/serial_handler.h>
#include <slate/shell.h>
#include <slate/wifi.h>
#include <slate/wifi_config.h>
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

  shell_print(sh, "Sending %s command to Helios (argument: %d)",
      fusain_mode_names[cmd->mode], cmd->argument);
  return 0;
}

//////////////////////////////////////////////////////////////
// Helios Commands
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

int cmd_helios_status(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  struct serial_handler_stats stats;
  serial_handler_get_stats(&stats);

  shell_print(sh, "Helios Serial Status:");
  shell_print(sh, "  Bytes received: %u", stats.bytes_received);
  shell_print(sh, "  Packets decoded: %u", stats.packets_decoded);
  shell_print(sh, "  Ping response: %s", stats.ping_response_received ? "yes" : "no");
  shell_print(sh, "  Telemetry: %s", stats.telemetry_received ? "yes" : "no");
  shell_print(sh, "  Helios uptime: %u ms", stats.helios_uptime_ms);

  return 0;
}

int cmd_set_idle(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  fusain_state_command_msg_t cmd = {
    .mode = FUSAIN_MODE_IDLE,
    .argument = 0,
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
    .argument = rpm,
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
    .argument = pump_rate,
  };

  return send_state_command(sh, &cmd);
}

//////////////////////////////////////////////////////////////
// Slate Commands
//////////////////////////////////////////////////////////////

int cmd_slate(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  shell_print(sh, "Slate controller configuration");
  shell_print(sh, "Use 'slate wifi' subcommands for WiFi configuration");
  return 0;
}

int cmd_slate_wifi(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  shell_print(sh, "WiFi configuration");
  shell_print(sh, "Available subcommands: save, auto-connect, reconnect-interval, settings");
  return 0;
}

//////////////////////////////////////////////////////////////
// WiFi Commands
//////////////////////////////////////////////////////////////

int cmd_wifi_save(const struct shell* sh, size_t argc, char** argv)
{
  if (argc != 3) {
    shell_print(sh, "Usage: slate wifi save <ssid> <password>");
    return -EINVAL;
  }

  const char* ssid = argv[1];
  const char* password = argv[2];

  // Validate SSID length
  size_t ssid_len = strlen(ssid);
  if (ssid_len == 0 || ssid_len > WIFI_SSID_MAX_LEN) {
    shell_print(sh, "Error: SSID must be 1-%d characters (got %zu)",
                WIFI_SSID_MAX_LEN, ssid_len);
    return -EINVAL;
  }

  // Validate password length
  size_t password_len = strlen(password);
  if (password_len > WIFI_PASSWORD_MAX_LEN) {
    shell_print(sh, "Error: Password must be 0-%d characters (got %zu)",
                WIFI_PASSWORD_MAX_LEN, password_len);
    return -EINVAL;
  }

  // Save credentials
  int rc = wifi_config_save(ssid, password);
  if (rc != 0) {
    shell_print(sh, "Error: Failed to save WiFi credentials: %d", rc);
    return rc;
  }

  shell_print(sh, "WiFi credentials saved successfully");
  shell_print(sh, "SSID: %s", ssid);
  shell_print(sh, "Password: %s", password_len > 0 ? "***" : "(empty)");

  // Send connect command via Zbus
  shell_print(sh, "Connecting to WiFi...");
  wifi_command_msg_t cmd = {
    .type = WIFI_CMD_CONNECT,
  };
  strncpy(cmd.connect.ssid, ssid, sizeof(cmd.connect.ssid) - 1);
  cmd.connect.ssid[sizeof(cmd.connect.ssid) - 1] = '\0';
  strncpy(cmd.connect.password, password, sizeof(cmd.connect.password) - 1);
  cmd.connect.password[sizeof(cmd.connect.password) - 1] = '\0';

  rc = zbus_chan_pub(&wifi_command_chan, &cmd, PUB_TIMEOUT);
  if (rc != 0) {
    shell_print(sh, "Error: Failed to send WiFi command: %d", rc);
    return rc;
  }

  shell_print(sh, "WiFi connection initiated (check logs for result)");

  return 0;
}

int cmd_wifi_auto_connect_enable(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  // Load current settings
  struct wifi_settings settings;
  wifi_settings_load(&settings);

  // Update auto_connect
  settings.auto_connect = true;

  // Save updated settings
  int rc = wifi_settings_save(settings.auto_connect, settings.reconnect_interval);
  if (rc != 0) {
    shell_print(sh, "Error: Failed to save WiFi settings: %d", rc);
    return rc;
  }

  // Update running state via Zbus
  wifi_command_msg_t cmd = {
    .type = WIFI_CMD_SET_AUTO_CONNECT,
    .auto_connect = true,
  };
  rc = zbus_chan_pub(&wifi_command_chan, &cmd, PUB_TIMEOUT);
  if (rc != 0) {
    shell_print(sh, "Error: Failed to send WiFi command: %d", rc);
    return rc;
  }

  shell_print(sh, "WiFi auto-connect enabled");

  return 0;
}

int cmd_wifi_auto_connect_disable(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  // Load current settings
  struct wifi_settings settings;
  wifi_settings_load(&settings);

  // Update auto_connect
  settings.auto_connect = false;

  // Save updated settings
  int rc = wifi_settings_save(settings.auto_connect, settings.reconnect_interval);
  if (rc != 0) {
    shell_print(sh, "Error: Failed to save WiFi settings: %d", rc);
    return rc;
  }

  // Update running state via Zbus
  wifi_command_msg_t cmd = {
    .type = WIFI_CMD_SET_AUTO_CONNECT,
    .auto_connect = false,
  };
  rc = zbus_chan_pub(&wifi_command_chan, &cmd, PUB_TIMEOUT);
  if (rc != 0) {
    shell_print(sh, "Error: Failed to send WiFi command: %d", rc);
    return rc;
  }

  shell_print(sh, "WiFi auto-connect disabled");

  return 0;
}

int cmd_wifi_reconnect_interval(const struct shell* sh, size_t argc, char** argv)
{
  if (argc != 2) {
    shell_print(sh, "Usage: slate wifi reconnect-interval <seconds>");
    shell_print(sh, "  Valid range: 10-3600 seconds");
    return -EINVAL;
  }

  int reconnect_interval = atoi(argv[1]);
  if (reconnect_interval < 10 || reconnect_interval > 3600) {
    shell_print(sh, "Error: Reconnect interval must be 10-3600 seconds (got %d)",
                reconnect_interval);
    return -EINVAL;
  }

  // Load current settings
  struct wifi_settings settings;
  wifi_settings_load(&settings);

  // Update reconnect_interval
  settings.reconnect_interval = reconnect_interval;

  // Save updated settings
  int rc = wifi_settings_save(settings.auto_connect, settings.reconnect_interval);
  if (rc != 0) {
    shell_print(sh, "Error: Failed to save WiFi settings: %d", rc);
    return rc;
  }

  // Update running state via Zbus
  wifi_command_msg_t cmd = {
    .type = WIFI_CMD_SET_RECONNECT_INTERVAL,
    .reconnect_interval = (uint32_t)reconnect_interval,
  };
  rc = zbus_chan_pub(&wifi_command_chan, &cmd, PUB_TIMEOUT);
  if (rc != 0) {
    shell_print(sh, "Error: Failed to send WiFi command: %d", rc);
    return rc;
  }

  shell_print(sh, "WiFi reconnect interval set to %u seconds", reconnect_interval);

  return 0;
}

int cmd_wifi_settings(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  // Load current settings
  struct wifi_settings settings;
  int rc = wifi_settings_load(&settings);
  if (rc != 0) {
    shell_print(sh, "Error: Failed to load WiFi settings: %d", rc);
    return rc;
  }

  shell_print(sh, "WiFi Settings:");
  shell_print(sh, "  Auto-connect: %s", settings.auto_connect ? "enabled" : "disabled");
  shell_print(sh, "  Reconnect interval: %u seconds", settings.reconnect_interval);

  return 0;
}

int cmd_wifi_status(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  // Get WiFi status from Zbus
  wifi_status_msg_t status_msg;
  int rc = zbus_chan_read(&wifi_status_chan, &status_msg, K_MSEC(100));
  if (rc != 0) {
    shell_print(sh, "Error: Failed to read WiFi status from Zbus: %d", rc);
    return rc;
  }

  // Get WiFi station interface
  struct net_if* iface = net_if_get_wifi_sta();
  if (!iface) {
    shell_print(sh, "Error: WiFi interface not found");
    return -ENODEV;
  }

  // Request WiFi interface status
  struct wifi_iface_status status = { 0 };
  rc = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
                    sizeof(struct wifi_iface_status));
  if (rc != 0) {
    shell_print(sh, "Status request failed: %d", rc);
    return -ENOEXEC;
  }

  // Print status header
  shell_print(sh, "Status: successful");
  shell_print(sh, "==================");
  shell_print(sh, "State: %s", wifi_state_txt(status.state));

  // Display hostname from Zbus
  if (status_msg.hostname[0] != '\0') {
    shell_print(sh, "Hostname: %s", status_msg.hostname);
  }

  // Print detailed info only if associated
  if (status.state >= WIFI_STATE_ASSOCIATED) {
    char bssid_str[18];  // "xx:xx:xx:xx:xx:xx\0"

    shell_print(sh, "Interface Mode: %s", wifi_mode_txt(status.iface_mode));
    shell_print(sh, "Link Mode: %s", wifi_link_mode_txt(status.link_mode));
    shell_print(sh, "SSID: %.32s", status.ssid);

    // Format BSSID manually
    snprintf(bssid_str, sizeof(bssid_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             status.bssid[0], status.bssid[1], status.bssid[2],
             status.bssid[3], status.bssid[4], status.bssid[5]);
    shell_print(sh, "BSSID: %s", bssid_str);

    shell_print(sh, "Band: %s", wifi_band_txt(status.band));
    shell_print(sh, "Channel: %d", status.channel);
    shell_print(sh, "Security: %s %s",
                wifi_wpa3_enterprise_txt(status.wpa3_ent_type),
                wifi_security_txt(status.security));
    shell_print(sh, "MFP: %s", wifi_mfp_txt(status.mfp));

    if (status.iface_mode == WIFI_MODE_INFRA) {
      shell_print(sh, "RSSI: %d", status.rssi);
    }

    shell_print(sh, "Beacon Interval: %d", status.beacon_interval);
    shell_print(sh, "DTIM: %d", status.dtim_period);
    shell_print(sh, "TWT: %s", status.twt_capable ? "Supported" : "Not supported");
    shell_print(sh, "Current PHY TX rate (Mbps) : %.1f", (double)status.current_phy_tx_rate);

    // Display IP addresses from Zbus
    if (status_msg.ipv4_address[0] != '\0') {
      shell_print(sh, "IPv4 Address: %s", status_msg.ipv4_address);
    }

    // Display IPv6 address (hide field if none)
    if (status_msg.ipv6_address[0] != '\0') {
      shell_print(sh, "IPv6 Address: %s", status_msg.ipv6_address);
    }
  }

  return 0;
}
