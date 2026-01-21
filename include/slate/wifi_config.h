// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Kaz Walker, Thermoquad

#ifndef SLATE_WIFI_CONFIG_H
#define SLATE_WIFI_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/**
 * WiFi Configuration Management
 *
 * Persistent storage for WiFi credentials using NVS.
 */

//////////////////////////////////////////////////////////////
// Constants
//////////////////////////////////////////////////////////////

#define WIFI_SSID_MAX_LEN     32
#define WIFI_PASSWORD_MAX_LEN 64

//////////////////////////////////////////////////////////////
// WiFi Credential Structure
//////////////////////////////////////////////////////////////

/**
 * WiFi credentials structure
 *
 * Version 1 format - SSID and password as null-terminated strings
 */
struct wifi_credentials {
	char ssid[WIFI_SSID_MAX_LEN + 1];         // +1 for null terminator
	char password[WIFI_PASSWORD_MAX_LEN + 1]; // +1 for null terminator
};

/**
 * WiFi settings structure
 *
 * Version 1 format - Auto-connect and reconnect configuration
 */
struct wifi_settings {
	bool auto_connect;            // Enable auto-connect on boot and on disconnect
	uint32_t reconnect_interval;  // Reconnect interval in seconds (10-3600)
};

//////////////////////////////////////////////////////////////
// Public API
//////////////////////////////////////////////////////////////

/**
 * Save WiFi credentials to NVS
 *
 * @param ssid WiFi network SSID (max 32 characters)
 * @param password WiFi password (max 64 characters)
 * @return 0 on success, negative error code on failure
 */
int wifi_config_save(const char *ssid, const char *password);

/**
 * Load WiFi credentials from NVS
 *
 * @param[out] creds Pointer to credentials structure to populate
 * @return 0 on success, negative error code on failure
 *         -ENOENT if no credentials are stored
 */
int wifi_config_load(struct wifi_credentials *creds);

/**
 * Check if WiFi credentials are stored
 *
 * @return true if credentials exist, false otherwise
 */
bool wifi_config_exists(void);

/**
 * Delete stored WiFi credentials
 *
 * @return 0 on success, negative error code on failure
 */
int wifi_config_delete(void);

/**
 * Save WiFi settings to NVS
 *
 * @param auto_connect Enable auto-connect on boot and on disconnect
 * @param reconnect_interval Reconnect interval in seconds (10-3600)
 * @return 0 on success, negative error code on failure
 */
int wifi_settings_save(bool auto_connect, uint32_t reconnect_interval);

/**
 * Load WiFi settings from NVS
 *
 * @param[out] settings Pointer to settings structure to populate
 * @return 0 on success, negative error code on failure
 *         -ENOENT if no settings are stored
 */
int wifi_settings_load(struct wifi_settings *settings);

/**
 * Check if WiFi settings are stored
 *
 * @return true if settings exist, false otherwise
 */
bool wifi_settings_exist(void);

/**
 * Delete stored WiFi settings
 *
 * @return 0 on success, negative error code on failure
 */
int wifi_settings_delete(void);

#endif /* SLATE_WIFI_CONFIG_H */
