// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Kaz Walker, Thermoquad

//////////////////////////////////////////////////////////////
// Includes
//////////////////////////////////////////////////////////////

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/zbus/zbus.h>
#include <string.h>

#include <slate/wifi.h>
#include <slate/wifi_config.h>
#include <slate/zbus.h>

LOG_MODULE_REGISTER(wifi, LOG_LEVEL_INF);

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

#define WIFI_THREAD_STACK_SIZE 2048
#define WIFI_THREAD_PRIORITY 6

#define WIFI_LOOP_SLEEP_MS 1000  // Check state every second

//////////////////////////////////////////////////////////////
// State Struct Definition
//////////////////////////////////////////////////////////////

struct wifi_state {
	// Connection state
	bool connected;
	bool connecting;
	bool connection_failed;
	enum wifi_conn_status last_status;

	// Credentials
	char ssid[WIFI_SSID_MAX_LEN + 1];
	char password[WIFI_PASSWORD_MAX_LEN + 1];
	bool has_credentials;

	// Settings
	bool auto_connect;
	uint32_t reconnect_interval;  // seconds

	// Retry tracking
	uint64_t last_connect_attempt_time;  // milliseconds
};

//////////////////////////////////////////////////////////////
// Static Variables
//////////////////////////////////////////////////////////////

static struct net_mgmt_event_callback wifi_mgmt_cb;
static struct net_mgmt_event_callback net_addr_mgmt_cb;
static struct wifi_state wifi_state;
static K_THREAD_STACK_DEFINE(wifi_thread_stack, WIFI_THREAD_STACK_SIZE);
static struct k_thread wifi_thread_data;

//////////////////////////////////////////////////////////////
// Forward Declarations
//////////////////////////////////////////////////////////////

static void wifi_command_listener_cb(const struct zbus_channel *chan);

//////////////////////////////////////////////////////////////
// Zbus Observer
//////////////////////////////////////////////////////////////

// Listener for WiFi command channel (triggered on new publications only)
ZBUS_LISTENER_DEFINE(wifi_command_listener, wifi_command_listener_cb);

//////////////////////////////////////////////////////////////
// Zbus Channels
//////////////////////////////////////////////////////////////

ZBUS_CHAN_DEFINE(wifi_command_chan,
		 wifi_command_msg_t,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS(wifi_command_listener),
		 ZBUS_MSG_INIT(0));

ZBUS_CHAN_DEFINE(wifi_status_chan,
		 wifi_status_msg_t,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

//////////////////////////////////////////////////////////////
// Forward Declarations
//////////////////////////////////////////////////////////////

static void wifi_thread(void *p1, void *p2, void *p3);
static int wifi_connect_internal(void);
static void publish_wifi_status(void);
static void handle_wifi_command(const wifi_command_msg_t *cmd);

//////////////////////////////////////////////////////////////
// Event Handler
//////////////////////////////////////////////////////////////

static void wifi_event_handler(struct net_mgmt_event_callback *cb,
			       uint64_t mgmt_event,
			       struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT: {
		// Get connection status from event info
		const struct wifi_status *status =
			(const struct wifi_status *)cb->info;

		wifi_state.connecting = false;

		if (status && status->conn_status == WIFI_STATUS_CONN_SUCCESS) {
			LOG_INF("WiFi connected to '%s'", wifi_state.ssid);
			wifi_state.connected = true;
			wifi_state.connection_failed = false;
		} else {
			LOG_ERR("WiFi connection failed: %s",
				status ? wifi_conn_status_txt(status->conn_status) : "unknown");
			wifi_state.connected = false;
			wifi_state.connection_failed = true;
			wifi_state.last_status = status ? status->conn_status : WIFI_STATUS_CONN_FAIL;

			// Don't retry if wrong password
			if (status && status->conn_status == WIFI_STATUS_CONN_WRONG_PASSWORD) {
				LOG_ERR("Wrong WiFi password - auto-connect disabled");
				wifi_state.auto_connect = false;
			}
		}

		// Publish status update
		publish_wifi_status();
		break;
	}

	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		LOG_INF("WiFi disconnected");
		wifi_state.connected = false;
		wifi_state.connecting = false;

		// Publish status update
		publish_wifi_status();
		break;

	default:
		break;
	}
}

/**
 * Network address event handler
 *
 * Handles IP address assignment events (IPv4 DHCP, IPv6 SLAAC)
 */
static void net_addr_event_handler(struct net_mgmt_event_callback *cb,
				    uint64_t mgmt_event,
				    struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	// Only process events if WiFi is connected
	if (!wifi_state.connected) {
		return;
	}

	switch (mgmt_event) {
	case NET_EVENT_IPV4_ADDR_ADD:
		LOG_DBG("IPv4 address added");
		publish_wifi_status();
		break;

	case NET_EVENT_IPV6_ADDR_ADD:
		LOG_DBG("IPv6 address added");
		publish_wifi_status();
		break;

	default:
		break;
	}
}

//////////////////////////////////////////////////////////////
// WiFi Thread
//////////////////////////////////////////////////////////////

static void wifi_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("WiFi thread started");

	// Load credentials and settings
	struct wifi_credentials creds;
	struct wifi_settings settings;

	if (wifi_config_load(&creds) == 0) {
		strncpy(wifi_state.ssid, creds.ssid, WIFI_SSID_MAX_LEN);
		strncpy(wifi_state.password, creds.password, WIFI_PASSWORD_MAX_LEN);
		wifi_state.has_credentials = true;
		LOG_INF("WiFi credentials loaded: SSID='%s'", wifi_state.ssid);
	} else {
		wifi_state.has_credentials = false;
		LOG_INF("No WiFi credentials stored");
	}

	// Load settings (uses defaults if not stored)
	if (wifi_settings_load(&settings) == 0) {
		wifi_state.auto_connect = settings.auto_connect;
		wifi_state.reconnect_interval = settings.reconnect_interval;
	}

	// Initial connect if auto_connect enabled and credentials exist
	if (wifi_state.auto_connect && wifi_state.has_credentials) {
		LOG_INF("Auto-connect enabled, connecting to WiFi...");
		wifi_connect_internal();
	}

	// Main loop
	while (true) {
		uint64_t current_time = k_uptime_get();

		// Handle reconnect logic
		if (wifi_state.auto_connect &&
		    wifi_state.has_credentials &&
		    !wifi_state.connected &&
		    !wifi_state.connecting) {

			// Check if enough time has passed since last attempt
			uint64_t elapsed_ms = current_time - wifi_state.last_connect_attempt_time;
			uint64_t reconnect_interval_ms = wifi_state.reconnect_interval * 1000ULL;

			if (elapsed_ms >= reconnect_interval_ms) {
				LOG_INF("Retrying WiFi connection...");
				wifi_connect_internal();
			}
		}

		k_sleep(K_MSEC(WIFI_LOOP_SLEEP_MS));
	}
}

//////////////////////////////////////////////////////////////
// Internal Functions
//////////////////////////////////////////////////////////////

static int wifi_connect_internal(void)
{
	if (!wifi_state.has_credentials) {
		LOG_ERR("Cannot connect: no credentials");
		return -ENOENT;
	}

	if (wifi_state.connecting) {
		LOG_WRN("Connection already in progress");
		return -EALREADY;
	}

	struct net_if *iface = net_if_get_wifi_sta();
	if (!iface) {
		LOG_ERR("WiFi STA interface not available");
		return -ENODEV;
	}

	// Update state
	wifi_state.connecting = true;
	wifi_state.connection_failed = false;
	wifi_state.last_connect_attempt_time = k_uptime_get();

	// Configure connection parameters
	struct wifi_connect_req_params params = {
		.ssid = (const uint8_t *)wifi_state.ssid,
		.ssid_length = strlen(wifi_state.ssid),
		.psk = strlen(wifi_state.password) > 0 ? (const uint8_t *)wifi_state.password : NULL,
		.psk_length = strlen(wifi_state.password),
		.security = strlen(wifi_state.password) > 0 ? WIFI_SECURITY_TYPE_PSK : WIFI_SECURITY_TYPE_NONE,
		.band = WIFI_FREQ_BAND_2_4_GHZ,
		.channel = WIFI_CHANNEL_ANY,
		.timeout = SYS_FOREVER_MS,
	};

	LOG_INF("Connecting to WiFi SSID: %s", wifi_state.ssid);

	int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
	if (ret) {
		LOG_ERR("WiFi connection request failed: %d", ret);
		wifi_state.connecting = false;
		return ret;
	}

	return 0;
}

//////////////////////////////////////////////////////////////
// Helper Functions
//////////////////////////////////////////////////////////////

/**
 * Publish current WiFi status to Zbus
 */
static void publish_wifi_status(void)
{
	wifi_status_msg_t msg = {0};

	// Determine connection state
	if (wifi_state.connected) {
		msg.state = SLATE_WIFI_STATE_CONNECTED;
	} else if (wifi_state.connecting) {
		msg.state = SLATE_WIFI_STATE_CONNECTING;
	} else if (wifi_state.connection_failed) {
		msg.state = SLATE_WIFI_STATE_FAILED;
	} else {
		msg.state = SLATE_WIFI_STATE_DISCONNECTED;
	}

	// Copy settings
	msg.auto_connect = wifi_state.auto_connect;
	msg.reconnect_interval = wifi_state.reconnect_interval;

	// Copy SSID
	strncpy(msg.ssid, wifi_state.ssid, sizeof(msg.ssid) - 1);
	msg.ssid[sizeof(msg.ssid) - 1] = '\0';

	// Get IP addresses if connected
	if (wifi_state.connected) {
		wifi_get_ip_address(msg.ipv4_address, sizeof(msg.ipv4_address));
		// Get IPv6 address (may fail if not configured, which is OK)
		if (wifi_get_ipv6_address(msg.ipv6_address, sizeof(msg.ipv6_address)) != 0) {
			msg.ipv6_address[0] = '\0';
		}

		// Log IP addresses
		if (msg.ipv6_address[0] != '\0') {
			LOG_INF("WiFi addresses: IPv4=%s, IPv6=%s",
				msg.ipv4_address, msg.ipv6_address);
		} else {
			LOG_INF("WiFi address: IPv4=%s (IPv6 not configured)",
				msg.ipv4_address);
		}
	} else {
		msg.ipv4_address[0] = '\0';
		msg.ipv6_address[0] = '\0';
	}

	// Publish to Zbus
	int ret = zbus_chan_pub(&wifi_status_chan, &msg, K_NO_WAIT);
	if (ret != 0) {
		LOG_WRN("Failed to publish WiFi status: %d", ret);
	}
}

/**
 * WiFi command listener callback
 *
 * Triggered when a new command is published to wifi_command_chan
 */
static void wifi_command_listener_cb(const struct zbus_channel *chan)
{
	const wifi_command_msg_t *cmd = zbus_chan_const_msg(chan);
	handle_wifi_command(cmd);
}

/**
 * Handle WiFi command from Zbus
 */
static void handle_wifi_command(const wifi_command_msg_t *cmd)
{
	switch (cmd->type) {
	case WIFI_CMD_CONNECT:
		LOG_INF("Command: Connect to SSID='%s'", cmd->connect.ssid);
		// Update credentials
		strncpy(wifi_state.ssid, cmd->connect.ssid, WIFI_SSID_MAX_LEN);
		wifi_state.ssid[WIFI_SSID_MAX_LEN] = '\0';
		strncpy(wifi_state.password, cmd->connect.password, WIFI_PASSWORD_MAX_LEN);
		wifi_state.password[WIFI_PASSWORD_MAX_LEN] = '\0';
		wifi_state.has_credentials = true;
		// Initiate connection
		wifi_connect_internal();
		break;

	case WIFI_CMD_DISCONNECT:
		LOG_INF("Command: Disconnect");
		wifi_disconnect();
		break;

	case WIFI_CMD_SET_AUTO_CONNECT:
		LOG_INF("Command: Set auto-connect=%d", cmd->auto_connect);
		wifi_state.auto_connect = cmd->auto_connect;
		// Publish updated status
		publish_wifi_status();
		break;

	case WIFI_CMD_SET_RECONNECT_INTERVAL:
		LOG_INF("Command: Set reconnect-interval=%u", cmd->reconnect_interval);
		wifi_state.reconnect_interval = cmd->reconnect_interval;
		// Publish updated status
		publish_wifi_status();
		break;

	default:
		LOG_WRN("Unknown WiFi command type: %d", cmd->type);
		break;
	}
}

//////////////////////////////////////////////////////////////
// Public API
//////////////////////////////////////////////////////////////

int wifi_connect(const char *ssid, const char *password)
{
	if (!ssid) {
		LOG_ERR("Invalid SSID");
		return -EINVAL;
	}

	// Update stored credentials
	strncpy(wifi_state.ssid, ssid, WIFI_SSID_MAX_LEN);
	wifi_state.ssid[WIFI_SSID_MAX_LEN] = '\0';

	if (password) {
		strncpy(wifi_state.password, password, WIFI_PASSWORD_MAX_LEN);
		wifi_state.password[WIFI_PASSWORD_MAX_LEN] = '\0';
	} else {
		wifi_state.password[0] = '\0';
	}

	wifi_state.has_credentials = true;

	return wifi_connect_internal();
}

int wifi_disconnect(void)
{
	struct net_if *iface = net_if_get_wifi_sta();
	if (!iface) {
		LOG_ERR("WiFi STA interface not available");
		return -ENODEV;
	}

	LOG_INF("Disconnecting from WiFi");

	int ret = net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
	if (ret) {
		LOG_ERR("WiFi disconnect request failed: %d", ret);
		return ret;
	}

	return 0;
}

bool wifi_is_connected(void)
{
	return wifi_state.connected;
}

int wifi_get_ip_address(char *addr_str, size_t buf_len)
{
	if (!addr_str || buf_len < 16) {
		return -EINVAL;
	}

	struct net_if *iface = net_if_get_wifi_sta();
	if (!iface) {
		return -ENODEV;
	}

	struct net_if_ipv4 *ipv4 = iface->config.ip.ipv4;
	if (!ipv4) {
		return -ENOENT;
	}

	struct net_if_addr *unicast = &ipv4->unicast[0].ipv4;
	if (!unicast || !unicast->is_used) {
		return -ENOENT;
	}

	net_addr_ntop(AF_INET, &unicast->address.in_addr, addr_str, buf_len);
	return 0;
}

int wifi_get_ipv6_address(char *addr_str, size_t buf_len)
{
	if (!addr_str || buf_len < 46) {
		return -EINVAL;
	}

	struct net_if *iface = net_if_get_wifi_sta();
	if (!iface) {
		return -ENODEV;
	}

	struct net_if_ipv6 *ipv6 = iface->config.ip.ipv6;
	if (!ipv6) {
		return -ENOENT;
	}

	// Find first unicast address that is in use
	for (int i = 0; i < NET_IF_MAX_IPV6_ADDR; i++) {
		if (ipv6->unicast[i].is_used) {
			net_addr_ntop(AF_INET6, &ipv6->unicast[i].address.in6_addr,
				      addr_str, buf_len);
			return 0;
		}
	}

	return -ENOENT;
}

//////////////////////////////////////////////////////////////
// Init Function
//////////////////////////////////////////////////////////////

static int wifi_init(void)
{
	// Register WiFi event callback
	net_mgmt_init_event_callback(&wifi_mgmt_cb,
				     wifi_event_handler,
				     NET_EVENT_WIFI_CONNECT_RESULT |
				     NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_mgmt_cb);

	// Register network address event callback
	net_mgmt_init_event_callback(&net_addr_mgmt_cb,
				     net_addr_event_handler,
				     NET_EVENT_IPV4_ADDR_ADD |
				     NET_EVENT_IPV6_ADDR_ADD);
	net_mgmt_add_event_callback(&net_addr_mgmt_cb);

	// Start WiFi thread
	k_thread_create(&wifi_thread_data, wifi_thread_stack,
			K_THREAD_STACK_SIZEOF(wifi_thread_stack),
			wifi_thread,
			NULL, NULL, NULL,
			WIFI_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(&wifi_thread_data, "wifi");

	return 0;
}

SYS_INIT(wifi_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
