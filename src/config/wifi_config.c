// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Kaz Walker, Thermoquad

//////////////////////////////////////////////////////////////
// Includes
//////////////////////////////////////////////////////////////

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <slate/config.h>
#include <slate/wifi_config.h>

LOG_MODULE_REGISTER(wifi_config, LOG_LEVEL_INF);

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

#define WIFI_CONFIG_VERSION 1
#define WIFI_SETTINGS_VERSION 1

// Default WiFi settings
#define WIFI_DEFAULT_AUTO_CONNECT true
#define WIFI_DEFAULT_RECONNECT_INTERVAL 30  // 30 seconds

// Validation limits
#define WIFI_RECONNECT_INTERVAL_MIN 10
#define WIFI_RECONNECT_INTERVAL_MAX 3600

//////////////////////////////////////////////////////////////
// Public API
//////////////////////////////////////////////////////////////

int wifi_config_save(const char *ssid, const char *password)
{
	if (!ssid || !password) {
		LOG_ERR("Invalid parameters");
		return -EINVAL;
	}

	size_t ssid_len = strlen(ssid);
	size_t password_len = strlen(password);

	if (ssid_len == 0 || ssid_len > WIFI_SSID_MAX_LEN) {
		LOG_ERR("Invalid SSID length: %zu (max %d)", ssid_len, WIFI_SSID_MAX_LEN);
		return -EINVAL;
	}

	if (password_len > WIFI_PASSWORD_MAX_LEN) {
		LOG_ERR("Invalid password length: %zu (max %d)",
			password_len, WIFI_PASSWORD_MAX_LEN);
		return -EINVAL;
	}

	struct wifi_credentials creds;
	memset(&creds, 0, sizeof(creds));

	strncpy(creds.ssid, ssid, WIFI_SSID_MAX_LEN);
	strncpy(creds.password, password, WIFI_PASSWORD_MAX_LEN);

	// Ensure null termination
	creds.ssid[WIFI_SSID_MAX_LEN] = '\0';
	creds.password[WIFI_PASSWORD_MAX_LEN] = '\0';

	int rc = config_save(NVS_ID_WIFI_CREDENTIALS,
			     CONFIG_TYPE_WIFI_CREDENTIALS,
			     WIFI_CONFIG_VERSION,
			     &creds,
			     sizeof(creds));

	if (rc == 0) {
		LOG_INF("WiFi credentials saved: SSID='%s'", ssid);
	} else {
		LOG_ERR("Failed to save WiFi credentials: %d", rc);
	}

	return rc;
}

int wifi_config_load(struct wifi_credentials *creds)
{
	if (!creds) {
		LOG_ERR("Invalid parameter");
		return -EINVAL;
	}

	uint16_t version;
	int rc = config_load(NVS_ID_WIFI_CREDENTIALS,
			     CONFIG_TYPE_WIFI_CREDENTIALS,
			     &version,
			     creds,
			     sizeof(*creds));

	if (rc == 0) {
		// Validate version
		if (version != WIFI_CONFIG_VERSION) {
			LOG_WRN("WiFi config version mismatch (expected %d, got %d)",
				WIFI_CONFIG_VERSION, version);
			// For now, accept version 1 only
			// Future: add migration logic here
			if (version != 1) {
				return -EINVAL;
			}
		}

		// Ensure null termination (defense in depth)
		creds->ssid[WIFI_SSID_MAX_LEN] = '\0';
		creds->password[WIFI_PASSWORD_MAX_LEN] = '\0';

		LOG_INF("WiFi credentials loaded: SSID='%s'", creds->ssid);
	} else if (rc == -ENOENT) {
		LOG_DBG("No WiFi credentials stored");
	} else {
		LOG_ERR("Failed to load WiFi credentials: %d", rc);
	}

	return rc;
}

bool wifi_config_exists(void)
{
	int rc = config_exists(NVS_ID_WIFI_CREDENTIALS);
	return (rc == 1);
}

int wifi_config_delete(void)
{
	int rc = config_delete(NVS_ID_WIFI_CREDENTIALS);
	if (rc == 0) {
		LOG_INF("WiFi credentials deleted");
	}
	return rc;
}

int wifi_settings_save(bool auto_connect, uint32_t reconnect_interval)
{
	// Validate reconnect interval
	if (reconnect_interval < WIFI_RECONNECT_INTERVAL_MIN ||
	    reconnect_interval > WIFI_RECONNECT_INTERVAL_MAX) {
		LOG_ERR("Invalid reconnect interval: %u (must be %u-%u seconds)",
			reconnect_interval, WIFI_RECONNECT_INTERVAL_MIN,
			WIFI_RECONNECT_INTERVAL_MAX);
		return -EINVAL;
	}

	struct wifi_settings settings = {
		.auto_connect = auto_connect,
		.reconnect_interval = reconnect_interval,
	};

	int rc = config_save(NVS_ID_WIFI_SETTINGS,
			     CONFIG_TYPE_WIFI_SETTINGS,
			     WIFI_SETTINGS_VERSION,
			     &settings,
			     sizeof(settings));

	if (rc == 0) {
		LOG_INF("WiFi settings saved: auto_connect=%s, reconnect_interval=%us",
			auto_connect ? "true" : "false", reconnect_interval);
	} else {
		LOG_ERR("Failed to save WiFi settings: %d", rc);
	}

	return rc;
}

int wifi_settings_load(struct wifi_settings *settings)
{
	if (!settings) {
		LOG_ERR("Invalid parameter");
		return -EINVAL;
	}

	uint16_t version;
	int rc = config_load(NVS_ID_WIFI_SETTINGS,
			     CONFIG_TYPE_WIFI_SETTINGS,
			     &version,
			     settings,
			     sizeof(*settings));

	if (rc == 0) {
		// Validate version
		if (version != WIFI_SETTINGS_VERSION) {
			LOG_WRN("WiFi settings version mismatch (expected %d, got %d)",
				WIFI_SETTINGS_VERSION, version);
			if (version != 1) {
				return -EINVAL;
			}
		}

		// Validate loaded values
		if (settings->reconnect_interval < WIFI_RECONNECT_INTERVAL_MIN ||
		    settings->reconnect_interval > WIFI_RECONNECT_INTERVAL_MAX) {
			LOG_ERR("Loaded invalid reconnect_interval: %u, using default",
				settings->reconnect_interval);
			settings->reconnect_interval = WIFI_DEFAULT_RECONNECT_INTERVAL;
		}

		LOG_INF("WiFi settings loaded: auto_connect=%s, reconnect_interval=%us",
			settings->auto_connect ? "true" : "false",
			settings->reconnect_interval);
	} else if (rc == -ENOENT) {
		// No settings stored, use defaults
		LOG_INF("No WiFi settings stored, using defaults");
		settings->auto_connect = WIFI_DEFAULT_AUTO_CONNECT;
		settings->reconnect_interval = WIFI_DEFAULT_RECONNECT_INTERVAL;
		rc = 0;  // Not an error, just use defaults
	} else {
		LOG_ERR("Failed to load WiFi settings: %d", rc);
	}

	return rc;
}

bool wifi_settings_exist(void)
{
	int rc = config_exists(NVS_ID_WIFI_SETTINGS);
	return (rc == 1);
}

int wifi_settings_delete(void)
{
	int rc = config_delete(NVS_ID_WIFI_SETTINGS);
	if (rc == 0) {
		LOG_INF("WiFi settings deleted");
	}
	return rc;
}
