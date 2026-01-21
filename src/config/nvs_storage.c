// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 Kaz Walker, Thermoquad

//////////////////////////////////////////////////////////////
// Includes
//////////////////////////////////////////////////////////////

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>

#include <slate/config.h>

LOG_MODULE_REGISTER(config, LOG_LEVEL_INF);

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

#define STORAGE_PARTITION       storage_partition
#define STORAGE_PARTITION_ID    FIXED_PARTITION_ID(STORAGE_PARTITION)
#define STORAGE_PARTITION_DEVICE FIXED_PARTITION_DEVICE(STORAGE_PARTITION)
#define STORAGE_PARTITION_OFFSET FIXED_PARTITION_OFFSET(STORAGE_PARTITION)

// Minimum sectors for NVS wear leveling
#define NVS_SECTOR_COUNT 3

//////////////////////////////////////////////////////////////
// State Struct Definition
//////////////////////////////////////////////////////////////

/**
 * Config header stored with each NVS entry
 *
 * This header is prepended to all config data to provide type safety and
 * versioning. Data integrity is provided by NVS built-in CRC-32.
 */
struct config_header {
	uint16_t type;     // Config type ID
	uint16_t version;  // Schema version
	uint16_t length;   // Data length (without header)
};

//////////////////////////////////////////////////////////////
// Static Variables
//////////////////////////////////////////////////////////////

static struct nvs_fs fs;
static bool initialized = false;

//////////////////////////////////////////////////////////////
// Public API
//////////////////////////////////////////////////////////////

int config_init(void)
{
	struct flash_pages_info info;
	int rc;

	if (initialized) {
		LOG_WRN("Config storage already initialized");
		return 0;
	}

	fs.flash_device = STORAGE_PARTITION_DEVICE;
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("Flash device not ready");
		return -ENODEV;
	}

	fs.offset = STORAGE_PARTITION_OFFSET;

	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		LOG_ERR("Failed to get flash page info: %d", rc);
		return rc;
	}

	fs.sector_size = info.size;
	fs.sector_count = NVS_SECTOR_COUNT;

	rc = nvs_mount(&fs);
	if (rc) {
		LOG_ERR("Failed to mount NVS: %d", rc);
		return rc;
	}

	initialized = true;
	LOG_INF("Config storage initialized (sector_size=%u, sector_count=%u)",
		fs.sector_size, fs.sector_count);

	return 0;
}

int config_save(uint16_t id, uint16_t type, uint16_t version, const void *data, size_t len)
{
	if (!initialized) {
		LOG_ERR("Config storage not initialized");
		return -EINVAL;
	}

	if (!data || len == 0) {
		LOG_ERR("Invalid data or length");
		return -EINVAL;
	}

	// Build header
	struct config_header header = {
		.type = type,
		.version = version,
		.length = len,
	};

	// Allocate buffer for header + data
	size_t total_len = sizeof(header) + len;
	uint8_t *buf = k_malloc(total_len);
	if (!buf) {
		LOG_ERR("Failed to allocate buffer for config save");
		return -ENOMEM;
	}

	// Copy header and data
	memcpy(buf, &header, sizeof(header));
	memcpy(buf + sizeof(header), data, len);

	// Write to NVS
	int rc = nvs_write(&fs, id, buf, total_len);

	k_free(buf);

	if (rc < 0) {
		LOG_ERR("Failed to write config ID 0x%04X: %d", id, rc);
		return rc;
	}

	LOG_DBG("Saved config ID 0x%04X (type=0x%04X, version=%u, len=%zu)",
		id, type, version, len);

	return 0;
}

int config_load(uint16_t id, uint16_t expected_type, uint16_t *version, void *data, size_t len)
{
	if (!initialized) {
		LOG_ERR("Config storage not initialized");
		return -EINVAL;
	}

	if (!data || !version || len == 0) {
		LOG_ERR("Invalid parameters");
		return -EINVAL;
	}

	// Allocate buffer for header + data
	size_t total_len = sizeof(struct config_header) + len;
	uint8_t *buf = k_malloc(total_len);
	if (!buf) {
		LOG_ERR("Failed to allocate buffer for config load");
		return -ENOMEM;
	}

	// Read from NVS
	int rc = nvs_read(&fs, id, buf, total_len);
	if (rc < 0) {
		k_free(buf);
		if (rc == -ENOENT) {
			LOG_DBG("Config ID 0x%04X not found", id);
		} else {
			LOG_ERR("Failed to read config ID 0x%04X: %d", id, rc);
		}
		return rc;
	}

	// Validate read length
	if (rc != total_len) {
		k_free(buf);
		LOG_ERR("Config ID 0x%04X size mismatch (expected %zu, got %d)",
			id, total_len, rc);
		return -EINVAL;
	}

	// Parse header
	struct config_header header;
	memcpy(&header, buf, sizeof(header));

	// Validate type
	if (header.type != expected_type) {
		k_free(buf);
		LOG_ERR("Config ID 0x%04X type mismatch (expected 0x%04X, got 0x%04X)",
			id, expected_type, header.type);
		return -EINVAL;
	}

	// Validate length
	if (header.length != len) {
		k_free(buf);
		LOG_ERR("Config ID 0x%04X data length mismatch (expected %zu, got %u)",
			id, len, header.length);
		return -EINVAL;
	}

	// Copy data to output
	const uint8_t *config_data = buf + sizeof(header);
	memcpy(data, config_data, len);
	*version = header.version;

	k_free(buf);

	LOG_DBG("Loaded config ID 0x%04X (type=0x%04X, version=%u, len=%zu)",
		id, header.type, header.version, len);

	return 0;
}

int config_exists(uint16_t id)
{
	if (!initialized) {
		LOG_ERR("Config storage not initialized");
		return -EINVAL;
	}

	// Try to read just the header
	struct config_header header;
	int rc = nvs_read(&fs, id, &header, sizeof(header));

	if (rc == sizeof(header)) {
		return 1;  // Exists
	} else if (rc == -ENOENT) {
		return 0;  // Doesn't exist
	} else {
		return rc;  // Error
	}
}

int config_delete(uint16_t id)
{
	if (!initialized) {
		LOG_ERR("Config storage not initialized");
		return -EINVAL;
	}

	int rc = nvs_delete(&fs, id);
	if (rc < 0 && rc != -ENOENT) {
		LOG_ERR("Failed to delete config ID 0x%04X: %d", id, rc);
		return rc;
	}

	LOG_INF("Deleted config ID 0x%04X", id);
	return 0;
}

int config_clear_all(void)
{
	if (!initialized) {
		LOG_ERR("Config storage not initialized");
		return -EINVAL;
	}

	int rc = nvs_clear(&fs);
	if (rc) {
		LOG_ERR("Failed to clear NVS: %d", rc);
		return rc;
	}

	LOG_WRN("Factory reset: All configuration cleared");
	return 0;
}
