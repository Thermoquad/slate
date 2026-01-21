// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Kaz Walker, Thermoquad

#ifndef SLATE_WIFI_H
#define SLATE_WIFI_H

#include <stdbool.h>
#include <stddef.h>

/**
 * WiFi Connection Management
 *
 * Handles WiFi connection, disconnection, and status monitoring using
 * Zephyr's net_mgmt API.
 */

//////////////////////////////////////////////////////////////
// Public API
//////////////////////////////////////////////////////////////

/**
 * Connect to WiFi network
 *
 * This is an asynchronous call that initiates a WiFi connection.
 * Connection result will be reported via WiFi management events.
 *
 * @param ssid WiFi network SSID (null-terminated string)
 * @param password WiFi network password (null-terminated string, or NULL for open networks)
 * @return 0 on success (connection initiated), negative error code on failure
 */
int wifi_connect(const char *ssid, const char *password);

/**
 * Disconnect from WiFi network
 *
 * @return 0 on success, negative error code on failure
 */
int wifi_disconnect(void);

/**
 * Check if WiFi is currently connected
 *
 * @return true if connected, false otherwise
 */
bool wifi_is_connected(void);

/**
 * Get current IPv4 address
 *
 * @param[out] addr_str Buffer to store IPv4 address string (min 16 bytes)
 * @param buf_len Length of the buffer
 * @return 0 on success, negative error code on failure
 */
int wifi_get_ip_address(char *addr_str, size_t buf_len);

/**
 * Get current IPv6 address
 *
 * @param[out] addr_str Buffer to store IPv6 address string (min 46 bytes)
 * @param buf_len Length of the buffer
 * @return 0 on success, negative error code on failure
 */
int wifi_get_ipv6_address(char *addr_str, size_t buf_len);

#endif /* SLATE_WIFI_H */
