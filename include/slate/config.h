// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Kaz Walker, Thermoquad

#ifndef SLATE_CONFIG_H
#define SLATE_CONFIG_H

#include <stdint.h>
#include <stddef.h>

/**
 * Configuration Storage Module
 *
 * Generic NVS-backed configuration storage with type safety and versioning.
 * Each config item has a type ID and version to support graceful schema
 * evolution and migration.
 */

//////////////////////////////////////////////////////////////
// Config Type IDs
//////////////////////////////////////////////////////////////

/**
 * Config type identifiers
 *
 * These identify the kind of configuration data stored at each NVS ID.
 * Used for type validation when loading configs.
 */
#define CONFIG_TYPE_WIFI_CREDENTIALS  0x0001
#define CONFIG_TYPE_WIFI_SETTINGS     0x0002

//////////////////////////////////////////////////////////////
// NVS Storage IDs
//////////////////////////////////////////////////////////////

/**
 * NVS storage IDs for Slate configuration items
 */
#define NVS_ID_WIFI_CREDENTIALS       0x0001
#define NVS_ID_WIFI_SETTINGS          0x0002

//////////////////////////////////////////////////////////////
// Public API
//////////////////////////////////////////////////////////////

/**
 * Initialize configuration storage
 *
 * Must be called once at boot before any other config operations.
 * Mounts the NVS filesystem from the storage partition.
 *
 * @return 0 on success, negative error code on failure
 */
int config_init(void);

/**
 * Save configuration data to NVS
 *
 * Stores configuration with type and version metadata. Data integrity is
 * provided by NVS built-in CRC-32 (CONFIG_NVS_DATA_CRC).
 *
 * @param id NVS storage ID (e.g., NVS_ID_WIFI_CREDENTIALS)
 * @param type Config type ID (e.g., CONFIG_TYPE_WIFI_CREDENTIALS)
 * @param version Schema version number
 * @param data Pointer to config data structure
 * @param len Size of config data in bytes
 * @return 0 on success, negative error code on failure
 */
int config_save(uint16_t id, uint16_t type, uint16_t version, const void *data, size_t len);

/**
 * Load configuration data from NVS
 *
 * Reads configuration and validates type. Data integrity is verified by NVS
 * built-in CRC-32. The version is returned via the version parameter to allow
 * the caller to handle schema migration.
 *
 * @param id NVS storage ID (e.g., NVS_ID_WIFI_CREDENTIALS)
 * @param expected_type Expected config type ID
 * @param[out] version Pointer to store the loaded version number
 * @param[out] data Pointer to buffer for config data
 * @param len Expected size of config data in bytes
 * @return 0 on success, negative error code on failure
 *         -EINVAL if type mismatch
 *         -ENOENT if config doesn't exist
 */
int config_load(uint16_t id, uint16_t expected_type, uint16_t *version, void *data, size_t len);

/**
 * Check if configuration exists
 *
 * @param id NVS storage ID
 * @return 1 if exists, 0 if not, negative error code on failure
 */
int config_exists(uint16_t id);

/**
 * Delete configuration from NVS
 *
 * @param id NVS storage ID
 * @return 0 on success, negative error code on failure
 */
int config_delete(uint16_t id);

/**
 * Clear all configuration data (factory reset)
 *
 * WARNING: This erases ALL stored configuration. Device will need
 * reconfiguration after this operation.
 *
 * @return 0 on success, negative error code on failure
 */
int config_clear_all(void);

#endif /* SLATE_CONFIG_H */
