/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Slate WebSocket Bridge - Fusain Protocol Router
 *
 * Acts as a Fusain router between WebSocket clients and Helios serial interface.
 * Implements discovery, subscription management, and message routing per the
 * Fusain specification.
 *
 * Note: HTTP_SERVICE_DEFINE and HTTP_RESOURCE_DEFINE are in main.c to avoid
 * linker section issues. This file exports ws_fusain_setup() and resource details.
 */

//////////////////////////////////////////////////////////////
// Includes
//////////////////////////////////////////////////////////////

#include <string.h>
#include <strings.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/net/websocket.h>
#include <zephyr/sys/base64.h>
#include <zephyr/zbus/zbus.h>

#include <fusain/fusain.h>
#include <slate/serial_handler.h>
#include <slate/zbus.h>

LOG_MODULE_REGISTER(websocket_bridge, LOG_LEVEL_INF);

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

#define WS_MAX_CLIENTS 4
#define WS_BUFFER_SIZE 256
#define WS_STACK_SIZE 2048
#define WS_RECV_TIMEOUT_MS 100
#define SUBSCRIPTION_TIMEOUT_MS 60000 /* 60 seconds */
#define TELEMETRY_TICK_MS 50 /* Periodic telemetry check interval */
#define DEFAULT_RATE_MS 500 /* Default per-client telemetry rate */

/* Authentication credentials (hardcoded for testing) */
#define AUTH_USERNAME "username"
#define AUTH_PASSWORD "password"
#define AUTH_MAX_CREDENTIAL_LEN 32
/* Buffer for decoded "username:password" = 32 + 1 + 32 + 1 null = 66 bytes */
#define AUTH_DECODED_BUFFER_SIZE (AUTH_MAX_CREDENTIAL_LEN * 2 + 2)

/* Telemetry config constraints */
#define TELEMETRY_MIN_INTERVAL_MS 100

//////////////////////////////////////////////////////////////
// Constant-Time Comparison (Timing Attack Prevention)
//////////////////////////////////////////////////////////////

/**
 * Compare two strings in constant time to prevent timing attacks.
 * Returns 0 if equal, non-zero otherwise.
 * Always compares all bytes to avoid leaking length information.
 */
static int constant_time_strcmp(const char* a, const char* b)
{
  size_t a_len = strlen(a);
  size_t b_len = strlen(b);

  /* XOR the lengths - will be non-zero if different */
  volatile size_t result = a_len ^ b_len;

  /* Compare up to the longer length to avoid timing leak */
  size_t max_len = (a_len > b_len) ? a_len : b_len;

  for (size_t i = 0; i < max_len; i++) {
    /* Use 0 for out-of-bounds to keep constant time */
    uint8_t byte_a = (i < a_len) ? (uint8_t)a[i] : 0;
    uint8_t byte_b = (i < b_len) ? (uint8_t)b[i] : 0;
    result |= byte_a ^ byte_b;
  }

  return (int)result;
}

//////////////////////////////////////////////////////////////
// HTTP Server State
//////////////////////////////////////////////////////////////

static bool http_server_running = false;
static struct k_mutex http_server_mutex;

//////////////////////////////////////////////////////////////
// Subscription Entry
//////////////////////////////////////////////////////////////

struct subscription_entry {
  uint64_t appliance_address; /* Address of appliance */
  int client_slot; /* WebSocket client slot index */
  int64_t last_ping_time; /* Uptime of last PING_REQUEST */
};

//////////////////////////////////////////////////////////////
// WebSocket Client Context
//////////////////////////////////////////////////////////////

struct ws_client_ctx {
  int sock;
  struct k_work_delayable recv_work;
  uint8_t recv_buffer[WS_BUFFER_SIZE];
  fusain_decoder_t decoder;

  /* Per-client telemetry rate limiting */
  uint32_t rate_ms;     /* Telemetry send interval (default: 100ms) */
  int64_t last_send_time; /* Timestamp of last telemetry send */
};

static struct ws_client_ctx clients[WS_MAX_CLIENTS];

//////////////////////////////////////////////////////////////
// Subscription Table
//////////////////////////////////////////////////////////////

#define MAX_SUBSCRIPTIONS 16

static struct subscription_entry subscriptions[MAX_SUBSCRIPTIONS];
static struct k_mutex subscription_mutex;

//////////////////////////////////////////////////////////////
// Helios Device Capabilities
//////////////////////////////////////////////////////////////

/* Helios device capabilities (from Helios CLAUDE.md) */
#define HELIOS_MOTOR_COUNT 1
#define HELIOS_THERMOMETER_COUNT 1
#define HELIOS_PUMP_COUNT 1
#define HELIOS_GLOW_COUNT 1

K_THREAD_STACK_DEFINE(ws_bridge_stack, WS_STACK_SIZE);
static struct k_work_q ws_bridge_queue;

//////////////////////////////////////////////////////////////
// Appliance Cache (Latest Telemetry Per Appliance)
//////////////////////////////////////////////////////////////

/**
 * Telemetry cache entry for a single message type
 *
 * Stores the latest packet and update timestamp. The "valid" flag
 * indicates whether this entry has been populated since last send.
 */
struct telemetry_entry {
  fusain_packet_t packet;
  int64_t update_time;
  bool valid;
};

/**
 * Appliance cache entry
 *
 * Stores the latest telemetry for each message type from a single appliance.
 * The cache uses "latest wins" semantics - newer packets overwrite older ones.
 */
struct appliance_cache_entry {
  uint64_t address;   /* Appliance address (0 = unused slot) */
  int64_t last_seen;  /* Last time any telemetry was received */

  /* Per-message-type telemetry cache */
  struct telemetry_entry state_data;    /* 0x30 */
  struct telemetry_entry motor_data;    /* 0x31 */
  struct telemetry_entry pump_data;     /* 0x32 */
  struct telemetry_entry glow_data;     /* 0x33 */
  struct telemetry_entry temp_data;     /* 0x34 */
  struct telemetry_entry ping_response; /* 0x3F */
};

static struct appliance_cache_entry appliance_cache[CONFIG_SLATE_MAX_APPLIANCES];
static struct k_mutex appliance_cache_mutex;

static struct k_work_delayable telemetry_work;

//////////////////////////////////////////////////////////////
// Subscription Management
//////////////////////////////////////////////////////////////

/* Add or refresh subscription for appliance */
static void subscription_add(uint64_t appliance_address, int client_slot)
{
  k_mutex_lock(&subscription_mutex, K_FOREVER);

  /* Check if subscription already exists */
  for (int i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (subscriptions[i].appliance_address == appliance_address && subscriptions[i].client_slot == client_slot) {
      /* Refresh existing subscription */
      subscriptions[i].last_ping_time = k_uptime_get();
      k_mutex_unlock(&subscription_mutex);
      LOG_DBG("Refreshed subscription for client %d to address 0x%llx",
          client_slot, appliance_address);
      return;
    }
  }

  /* Find free slot */
  for (int i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (subscriptions[i].appliance_address == 0) {
      subscriptions[i].appliance_address = appliance_address;
      subscriptions[i].client_slot = client_slot;
      subscriptions[i].last_ping_time = k_uptime_get();
      k_mutex_unlock(&subscription_mutex);
      LOG_INF("Added subscription for client %d to address 0x%llx",
          client_slot, appliance_address);
      return;
    }
  }

  k_mutex_unlock(&subscription_mutex);
  LOG_WRN("Subscription table full (max %d)", MAX_SUBSCRIPTIONS);
}

/* Remove subscription for specific appliance */
static void subscription_remove(uint64_t appliance_address, int client_slot)
{
  k_mutex_lock(&subscription_mutex, K_FOREVER);

  for (int i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (subscriptions[i].appliance_address == appliance_address &&
        subscriptions[i].client_slot == client_slot) {
      LOG_INF("Removed subscription for client %d to address 0x%llx",
          client_slot, appliance_address);
      subscriptions[i].appliance_address = 0;
      subscriptions[i].client_slot = -1;
      subscriptions[i].last_ping_time = 0;
      k_mutex_unlock(&subscription_mutex);
      return;
    }
  }

  k_mutex_unlock(&subscription_mutex);
  LOG_DBG("Subscription not found for client %d to address 0x%llx",
      client_slot, appliance_address);
}

/* Remove all subscriptions for a client */
static void subscription_remove_client(int client_slot)
{
  k_mutex_lock(&subscription_mutex, K_FOREVER);

  for (int i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (subscriptions[i].client_slot == client_slot) {
      LOG_DBG("Removed subscription for client %d to address 0x%llx",
          client_slot, subscriptions[i].appliance_address);
      subscriptions[i].appliance_address = 0;
      subscriptions[i].client_slot = -1;
      subscriptions[i].last_ping_time = 0;
    }
  }

  k_mutex_unlock(&subscription_mutex);
}

/**
 * Update ping timestamp for subscription(s)
 *
 * If appliance_address is 0 (broadcast), updates all subscriptions for the client.
 * Otherwise, updates only the subscription for the specific appliance.
 */
static void subscription_ping(int client_slot, uint64_t appliance_address)
{
  k_mutex_lock(&subscription_mutex, K_FOREVER);

  int64_t now = k_uptime_get();
  for (int i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (subscriptions[i].client_slot == client_slot) {
      if (appliance_address == 0 ||
          subscriptions[i].appliance_address == appliance_address) {
        subscriptions[i].last_ping_time = now;
      }
    }
  }

  k_mutex_unlock(&subscription_mutex);
}

/**
 * Expire stale subscriptions
 *
 * Removes subscriptions that haven't received a ping within SUBSCRIPTION_TIMEOUT_MS.
 * Should be called periodically.
 */
static void subscription_expire_stale(void)
{
  k_mutex_lock(&subscription_mutex, K_FOREVER);

  int64_t now = k_uptime_get();
  for (int i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (subscriptions[i].appliance_address != 0) {
      int64_t elapsed = now - subscriptions[i].last_ping_time;
      if (elapsed > SUBSCRIPTION_TIMEOUT_MS) {
        LOG_INF("Subscription expired: client %d, appliance 0x%llx (no ping for %lld ms)",
            subscriptions[i].client_slot, subscriptions[i].appliance_address, elapsed);
        subscriptions[i].appliance_address = 0;
        subscriptions[i].client_slot = -1;
        subscriptions[i].last_ping_time = 0;
      }
    }
  }

  k_mutex_unlock(&subscription_mutex);
}

//////////////////////////////////////////////////////////////
// Forward Declarations
//////////////////////////////////////////////////////////////

static int ws_send_packet(int client_slot, const fusain_packet_t* packet);
static void telemetry_work_handler(struct k_work* work);

//////////////////////////////////////////////////////////////
// Serial Telemetry Forwarding (Serial → WebSocket)
//////////////////////////////////////////////////////////////

/**
 * Check if a client has a subscription for a given appliance address
 */
static bool client_has_subscription(int client_slot, uint64_t appliance_address)
{
  bool has_sub = false;

  k_mutex_lock(&subscription_mutex, K_FOREVER);

  for (int i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (subscriptions[i].client_slot == client_slot &&
        subscriptions[i].appliance_address == appliance_address) {
      has_sub = true;
      break;
    }
  }

  k_mutex_unlock(&subscription_mutex);
  return has_sub;
}

/**
 * Forward a packet to all clients subscribed to the appliance
 *
 * Used for immediate forwarding of error messages (0xE0-0xE1).
 */
static void forward_to_subscribers(uint64_t appliance_address,
                                   const fusain_packet_t* packet)
{
  /* Collect client slots while holding mutex (fast) */
  int clients_to_notify[WS_MAX_CLIENTS];
  int client_count = 0;

  k_mutex_lock(&subscription_mutex, K_FOREVER);

  for (int i = 0; i < MAX_SUBSCRIPTIONS && client_count < WS_MAX_CLIENTS; i++) {
    if (subscriptions[i].appliance_address == appliance_address) {
      /* Check if client already in list (avoid duplicates) */
      bool already_added = false;
      for (int j = 0; j < client_count; j++) {
        if (clients_to_notify[j] == subscriptions[i].client_slot) {
          already_added = true;
          break;
        }
      }
      if (!already_added) {
        clients_to_notify[client_count++] = subscriptions[i].client_slot;
      }
    }
  }

  k_mutex_unlock(&subscription_mutex);

  /* Send to clients outside mutex (can block) */
  for (int i = 0; i < client_count; i++) {
    int ret = ws_send_packet(clients_to_notify[i], packet);
    if (ret < 0) {
      LOG_DBG("Failed to forward packet to client %d: %d",
          clients_to_notify[i], ret);
    }
  }
}

/**
 * Find or create appliance cache entry
 *
 * Returns pointer to cache entry for the given address, or NULL if cache is full.
 * Must be called with appliance_cache_mutex held.
 */
static struct appliance_cache_entry* cache_find_or_create(uint64_t address)
{
  struct appliance_cache_entry* free_slot = NULL;

  for (int i = 0; i < CONFIG_SLATE_MAX_APPLIANCES; i++) {
    if (appliance_cache[i].address == address) {
      return &appliance_cache[i];
    }
    if (appliance_cache[i].address == 0 && free_slot == NULL) {
      free_slot = &appliance_cache[i];
    }
  }

  /* Not found - use free slot if available */
  if (free_slot != NULL) {
    free_slot->address = address;
    free_slot->last_seen = k_uptime_get();
    LOG_INF("Added appliance 0x%llx to cache", address);
  }

  return free_slot;
}

/**
 * Update cache entry for a specific message type
 */
static void cache_update_entry(struct appliance_cache_entry* entry,
                               uint8_t msg_type,
                               const fusain_packet_t* packet)
{
  struct telemetry_entry* te = NULL;

  switch (msg_type) {
  case FUSAIN_MSG_STATE_DATA:
    te = &entry->state_data;
    break;
  case FUSAIN_MSG_MOTOR_DATA:
    te = &entry->motor_data;
    break;
  case FUSAIN_MSG_PUMP_DATA:
    te = &entry->pump_data;
    break;
  case FUSAIN_MSG_GLOW_DATA:
    te = &entry->glow_data;
    break;
  case FUSAIN_MSG_TEMP_DATA:
    te = &entry->temp_data;
    break;
  case FUSAIN_MSG_PING_RESPONSE:
    te = &entry->ping_response;
    break;
  default:
    return; /* Unknown message type, ignore */
  }

  te->packet = *packet;
  te->update_time = k_uptime_get();
  te->valid = true;
  entry->last_seen = te->update_time;
}

/**
 * Zbus callback for raw packet reception
 *
 * Called when serial handler publishes a packet to fusain_raw_rx_chan.
 * Updates the appliance cache with the latest telemetry. This callback
 * returns immediately - actual WebSocket sends happen in the periodic
 * telemetry work handler.
 */
void fusain_raw_rx_callback(const struct zbus_channel* chan)
{
  const fusain_raw_packet_msg_t* msg = zbus_chan_const_msg(chan);
  if (msg == NULL) {
    return;
  }

  const fusain_packet_t* packet = &msg->packet;
  uint8_t message_type = packet->msg_type;
  uint64_t appliance_address = packet->address;

  /* Forward error messages (0xE0-0xE1) immediately to subscribers */
  bool is_error = (message_type == FUSAIN_MSG_ERROR_INVALID_CMD ||
                   message_type == FUSAIN_MSG_ERROR_STATE_REJECT);
  if (is_error) {
    LOG_INF("Forwarding error 0x%02X from appliance 0x%llx to subscribers",
        message_type, appliance_address);
    forward_to_subscribers(appliance_address, packet);
    return;
  }

  /* Only cache telemetry data messages (0x30-0x34) and PING_RESPONSE (0x3F) */
  bool is_telemetry = (message_type >= FUSAIN_MSG_STATE_DATA &&
                       message_type <= FUSAIN_MSG_TEMP_DATA);
  bool is_ping_response = (message_type == FUSAIN_MSG_PING_RESPONSE);
  if (!is_telemetry && !is_ping_response) {
    return;
  }

  /* Update appliance cache (fast, non-blocking) */
  k_mutex_lock(&appliance_cache_mutex, K_FOREVER);

  struct appliance_cache_entry* entry = cache_find_or_create(appliance_address);
  if (entry != NULL) {
    cache_update_entry(entry, message_type, packet);
  } else {
    LOG_WRN("Appliance cache full, ignoring telemetry from 0x%llx", appliance_address);
  }

  k_mutex_unlock(&appliance_cache_mutex);
}

/* Listener is defined in main.c to be in same compilation unit as channel */
/* ZBUS_LISTENER_DEFINE(ws_bridge_listener, fusain_raw_rx_callback); */

//////////////////////////////////////////////////////////////
// WiFi Status Monitoring (HTTP Server Lifecycle)
//////////////////////////////////////////////////////////////

/**
 * WiFi status callback
 *
 * Controls HTTP server lifecycle based on WiFi connection state:
 * - Starts HTTP server when WiFi connects
 * - Stops HTTP server when WiFi disconnects
 */
static void wifi_status_callback(const struct zbus_channel* chan)
{
  const wifi_status_msg_t* status = zbus_chan_const_msg(chan);
  if (status == NULL) {
    return;
  }

  k_mutex_lock(&http_server_mutex, K_FOREVER);

  bool should_run = (status->state == SLATE_WIFI_STATE_CONNECTED);

  if (should_run && !http_server_running) {
    /* WiFi connected and IPv4 address assigned - start HTTP server */
    LOG_INF("WiFi connected (%s) - starting HTTP server", status->ipv4_address);
    int ret = http_server_start();
    if (ret == 0) {
      http_server_running = true;
      LOG_INF("HTTP server started on port 80");
    } else {
      LOG_ERR("Failed to start HTTP server: %d", ret);
    }
  } else if (!should_run && http_server_running) {
    /* WiFi disconnected - stop HTTP server */
    LOG_INF("WiFi disconnected - stopping HTTP server");
    int ret = http_server_stop();
    if (ret == 0) {
      http_server_running = false;
      LOG_INF("HTTP server stopped");
    } else {
      LOG_ERR("Failed to stop HTTP server: %d", ret);
    }
  }

  k_mutex_unlock(&http_server_mutex);
}

ZBUS_LISTENER_DEFINE(wifi_status_listener, wifi_status_callback);

//////////////////////////////////////////////////////////////
// Router Message Processing
//////////////////////////////////////////////////////////////

/* Send a Fusain packet to a specific WebSocket client */
static int ws_send_packet(int client_slot, const fusain_packet_t* packet)
{
  if (client_slot < 0 || client_slot >= WS_MAX_CLIENTS) {
    return -EINVAL;
  }

  if (clients[client_slot].sock < 0) {
    return -ENOTCONN;
  }

  uint8_t tx_buffer[FUSAIN_MAX_PACKET_SIZE * 2];
  int len = fusain_encode_packet(packet, tx_buffer, sizeof(tx_buffer));

  if (len < 0) {
    return len;
  }

  return websocket_send_msg(clients[client_slot].sock, tx_buffer, len,
      WEBSOCKET_OPCODE_DATA_BINARY,
      false, true, 100);
}

/**
 * Send cached telemetry to a single client
 *
 * Uses copy-then-send pattern to avoid holding mutex during WebSocket I/O:
 * 1. Hold mutex briefly to copy packets and clear valid flags
 * 2. Release mutex before sending (WebSocket sends can block)
 *
 * This ensures cache updates (from Zbus callback) are never blocked by
 * slow WebSocket operations, fully decoupling serial RX from WebSocket TX.
 */
/* Static buffer for copy-then-send pattern (work queue is single-threaded) */
#define TELEMETRY_SEND_BUFFER_SIZE (CONFIG_SLATE_MAX_APPLIANCES * 6)
static fusain_packet_t telemetry_send_buffer[TELEMETRY_SEND_BUFFER_SIZE];

static void send_cached_telemetry_to_client(int client_slot)
{
  if (clients[client_slot].sock < 0) {
    return;
  }

  int to_send_count = 0;

  /* FAST: Hold mutex only during memory copy */
  k_mutex_lock(&appliance_cache_mutex, K_FOREVER);

  for (int a = 0; a < CONFIG_SLATE_MAX_APPLIANCES; a++) {
    struct appliance_cache_entry* entry = &appliance_cache[a];
    if (entry->address == 0) {
      continue; /* Empty slot */
    }

    /* Check if client is subscribed to this appliance */
    if (!client_has_subscription(client_slot, entry->address)) {
      continue;
    }

    /* Copy each valid telemetry type to local buffer */
    struct telemetry_entry* entries[] = {
      &entry->state_data,
      &entry->motor_data,
      &entry->pump_data,
      &entry->glow_data,
      &entry->temp_data,
      &entry->ping_response,
    };

    for (int t = 0; t < ARRAY_SIZE(entries); t++) {
      if (entries[t]->valid && to_send_count < TELEMETRY_SEND_BUFFER_SIZE) {
        telemetry_send_buffer[to_send_count++] = entries[t]->packet;
        entries[t]->valid = false; /* Mark as consumed */
      }
    }
  }

  k_mutex_unlock(&appliance_cache_mutex);

  /* SLOW: Send packets OUTSIDE mutex - can block without affecting cache updates */
  for (int i = 0; i < to_send_count; i++) {
    int ret = ws_send_packet(client_slot, &telemetry_send_buffer[i]);
    if (ret < 0) {
      LOG_DBG("Failed to send packet to client %d: %d", client_slot, ret);
      /* Packet lost, but next telemetry update will arrive soon (latest-wins) */
    }
  }
}

/**
 * Periodic Telemetry Work Handler
 *
 * Runs every TELEMETRY_TICK_MS. For each connected client, checks if
 * enough time has passed since last send (based on client's rate_ms),
 * and sends any cached telemetry that the client is subscribed to.
 */
static void telemetry_work_handler(struct k_work* work)
{
  ARG_UNUSED(work);

  /* Check for and remove expired subscriptions */
  subscription_expire_stale();

  int64_t now = k_uptime_get();

  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    if (clients[i].sock < 0) {
      continue; /* No client in this slot */
    }

    /* Check if enough time has passed for this client */
    int64_t elapsed = now - clients[i].last_send_time;
    if (elapsed < clients[i].rate_ms) {
      continue; /* Not time yet */
    }

    /* Send cached telemetry to this client */
    send_cached_telemetry_to_client(i);
    clients[i].last_send_time = now;
  }

  /* Reschedule for next tick */
  k_work_reschedule_for_queue(&ws_bridge_queue, &telemetry_work, K_MSEC(TELEMETRY_TICK_MS));
}

/* Handle router-specific messages from WebSocket clients */
static void ws_process_router_message(int client_slot, const fusain_packet_t* packet)
{
  uint8_t message_type = packet->msg_type;

  switch (message_type) {
  case FUSAIN_MSG_DISCOVERY_REQUEST: {
    LOG_INF("Client %d: DISCOVERY_REQUEST", client_slot);

    /* Router responds directly with known appliances - do not forward to serial */
    /* Get tracked Helios address from serial handler */
    uint64_t helios_addr = serial_handler_get_helios_address();

    /* Send DEVICE_ANNOUNCE for Helios if we have its address */
    if (helios_addr != 0) {
      fusain_packet_t helios_announce = {
        .address = helios_addr,
        .length = 12,
      };

      /* CBOR: [0x35, {0:1, 1:1, 2:1, 3:1}] = Helios capabilities */
      uint8_t helios_cbor[] = {
        0x82, /* Array of 2 elements */
        0x18, 0x35, /* uint(0x35) = DEVICE_ANNOUNCE */
        0xA4, /* Map of 4 elements */
        0x00, HELIOS_MOTOR_COUNT, /* Key 0 (motor_count): 1 */
        0x01, HELIOS_THERMOMETER_COUNT, /* Key 1 (thermometer_count): 1 */
        0x02, HELIOS_PUMP_COUNT, /* Key 2 (pump_count): 1 */
        0x03, HELIOS_GLOW_COUNT /* Key 3 (glow_count): 1 */
      };
      memcpy(helios_announce.payload, helios_cbor, sizeof(helios_cbor));

      int ret = ws_send_packet(client_slot, &helios_announce);
      if (ret < 0) {
        LOG_ERR("Failed to send Helios DEVICE_ANNOUNCE: %d", ret);
      } else {
        LOG_INF("Sent DEVICE_ANNOUNCE for Helios (addr=0x%llx, caps=%d/%d/%d/%d)",
            helios_addr, HELIOS_MOTOR_COUNT, HELIOS_THERMOMETER_COUNT,
            HELIOS_PUMP_COUNT, HELIOS_GLOW_COUNT);
      }
    } else {
      LOG_WRN("No Helios address tracked yet - only sending end marker");
    }

    /* Send end-of-discovery marker (all zeros = no more devices) */
    fusain_packet_t end_marker = {
      .address = FUSAIN_ADDRESS_STATELESS,
      .length = 12,
    };

    /* CBOR: [0x35, {0:0, 1:0, 2:0, 3:0}] = end-of-discovery */
    uint8_t end_cbor[] = {
      0x82, /* Array of 2 elements */
      0x18, 0x35, /* uint(0x35) = DEVICE_ANNOUNCE */
      0xA4, /* Map of 4 elements */
      0x00, 0x00, /* Key 0 (motor_count): 0 */
      0x01, 0x00, /* Key 1 (thermometer_count): 0 */
      0x02, 0x00, /* Key 2 (pump_count): 0 */
      0x03, 0x00 /* Key 3 (glow_count): 0 */
    };
    memcpy(end_marker.payload, end_cbor, sizeof(end_cbor));

    int ret = ws_send_packet(client_slot, &end_marker);
    if (ret < 0) {
      LOG_ERR("Failed to send end-of-discovery: %d", ret);
    }
    break;
  }

  case FUSAIN_MSG_DATA_SUBSCRIPTION: {
    /* Extract appliance_address from CBOR payload */
    /* Payload format: [0x14, {0: appliance_address}] */
    /* CBOR: 0x82 (array), 0x18 0x14 (msg type), 0xA1 (map), 0x00 (key), address */

    if (packet->length < 6) {
      LOG_WRN("DATA_SUBSCRIPTION payload too short: %u bytes", packet->length);
      break;
    }

    /* Skip to the appliance_address value (after array, msg type, map, key) */
    /* Expected: 0x82 0x18 0x14 0xA1 0x00 <address> */
    const uint8_t* p = packet->payload + 5; /* Skip to address encoding */
    uint64_t appliance_address = 0;

    /* Parse CBOR uint encoding */
    if (*p < 24) {
      /* Direct value (0-23) */
      appliance_address = *p;
    } else if (*p == 0x18) {
      /* 1-byte uint */
      appliance_address = p[1];
    } else if (*p == 0x19) {
      /* 2-byte uint (big-endian) */
      appliance_address = ((uint64_t)p[1] << 8) | p[2];
    } else if (*p == 0x1A) {
      /* 4-byte uint (big-endian) */
      appliance_address = ((uint64_t)p[1] << 24) | ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 8) | p[4];
    } else if (*p == 0x1B) {
      /* 8-byte uint (big-endian) */
      appliance_address = ((uint64_t)p[1] << 56) | ((uint64_t)p[2] << 48) | ((uint64_t)p[3] << 40) | ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 24) | ((uint64_t)p[6] << 16) | ((uint64_t)p[7] << 8) | p[8];
    } else {
      LOG_WRN("Unsupported CBOR encoding for appliance_address: 0x%02x", *p);
      break;
    }

    LOG_INF("Client %d: DATA_SUBSCRIPTION for appliance 0x%llx",
        client_slot, appliance_address);
    subscription_add(appliance_address, client_slot);
    break;
  }

  case FUSAIN_MSG_DATA_UNSUBSCRIBE: {
    /* Extract appliance_address from CBOR payload */
    /* Payload format: [0x15, {0: appliance_address}] */
    /* CBOR: 0x82 (array), 0x18 0x15 (msg type), 0xA1 (map), 0x00 (key), address */

    if (packet->length < 6) {
      LOG_WRN("DATA_UNSUBSCRIBE payload too short: %u bytes", packet->length);
      break;
    }

    /* Skip to the appliance_address value (after array, msg type, map, key) */
    /* Expected: 0x82 0x18 0x15 0xA1 0x00 <address> */
    const uint8_t* p = packet->payload + 5; /* Skip to address encoding */
    uint64_t appliance_address = 0;

    /* Decode CBOR uint (variable length) */
    if (*p < 24) {
      /* Direct value (0-23) */
      appliance_address = *p;
    } else if (*p == 0x18) {
      /* 1-byte uint */
      appliance_address = p[1];
    } else if (*p == 0x19) {
      /* 2-byte uint (big-endian) */
      appliance_address = ((uint64_t)p[1] << 8) | p[2];
    } else if (*p == 0x1A) {
      /* 4-byte uint (big-endian) */
      appliance_address = ((uint64_t)p[1] << 24) | ((uint64_t)p[2] << 16) |
                          ((uint64_t)p[3] << 8) | p[4];
    } else if (*p == 0x1B) {
      /* 8-byte uint (big-endian) */
      appliance_address = ((uint64_t)p[1] << 56) | ((uint64_t)p[2] << 48) |
                          ((uint64_t)p[3] << 40) | ((uint64_t)p[4] << 32) |
                          ((uint64_t)p[5] << 24) | ((uint64_t)p[6] << 16) |
                          ((uint64_t)p[7] << 8) | p[8];
    } else {
      LOG_WRN("Unsupported CBOR encoding for appliance_address: 0x%02x", *p);
      break;
    }

    LOG_INF("Client %d: DATA_UNSUBSCRIBE for appliance 0x%llx",
        client_slot, appliance_address);
    subscription_remove(appliance_address, client_slot);
    break;
  }

  case FUSAIN_MSG_TELEMETRY_CONFIG: {
    /* Only handle if addressed to stateless (router) */
    if (packet->address != FUSAIN_ADDRESS_STATELESS) {
      LOG_DBG("TELEMETRY_CONFIG not addressed to router, forwarding");
      int ret = serial_handler_send_packet(packet);
      if (ret < 0) {
        LOG_ERR("Failed to forward TELEMETRY_CONFIG: %d", ret);
      }
      break;
    }

    /* Parse CBOR payload: [0x16, {0: enabled, 1: interval_ms}] */
    /* Expected: 0x82 0x16 0xA2 0x00 <enabled> 0x01 <interval_ms> */
    if (packet->length < 5) {
      LOG_WRN("TELEMETRY_CONFIG payload too short: %u bytes", packet->length);
      break;
    }

    const uint8_t* p = packet->payload;

    /* Skip array header (0x82) and message type (0x16) */
    if (p[0] != 0x82 || p[1] != 0x16) {
      LOG_WRN("Invalid TELEMETRY_CONFIG header");
      break;
    }
    p += 2;

    /* Expect map with 2 elements (0xA2) */
    if (*p != 0xA2) {
      LOG_WRN("Expected map in TELEMETRY_CONFIG, got 0x%02x", *p);
      break;
    }
    p++;

    /* Parse key 0 (enabled) - skip for now, just parse past it */
    if (*p != 0x00) {
      LOG_WRN("Expected key 0 in TELEMETRY_CONFIG");
      break;
    }
    p++;

    /* Parse enabled value (bool: 0xF4=false, 0xF5=true, or small uint) */
    bool enabled = false;
    if (*p == 0xF5) {
      enabled = true;
      p++;
    } else if (*p == 0xF4) {
      enabled = false;
      p++;
    } else if (*p < 24) {
      enabled = (*p != 0);
      p++;
    } else {
      LOG_WRN("Unsupported enabled encoding: 0x%02x", *p);
      break;
    }

    /* Parse key 1 (interval_ms) */
    if (*p != 0x01) {
      LOG_WRN("Expected key 1 in TELEMETRY_CONFIG");
      break;
    }
    p++;

    /* Parse interval_ms value */
    uint32_t interval_ms = 0;
    if (*p < 24) {
      interval_ms = *p;
    } else if (*p == 0x18) {
      interval_ms = p[1];
      p++;
    } else if (*p == 0x19) {
      interval_ms = ((uint32_t)p[1] << 8) | p[2];
    } else if (*p == 0x1A) {
      interval_ms = ((uint32_t)p[1] << 24) | ((uint32_t)p[2] << 16) |
                    ((uint32_t)p[3] << 8) | p[4];
    } else {
      LOG_WRN("Unsupported interval_ms encoding: 0x%02x", *p);
      break;
    }

    /* Validate: must be enabled with interval >= minimum */
    if (!enabled || interval_ms < TELEMETRY_MIN_INTERVAL_MS) {
      LOG_WRN("Client %d: TELEMETRY_CONFIG rejected (enabled=%d, interval=%u, min=%d)",
          client_slot, enabled, interval_ms, TELEMETRY_MIN_INTERVAL_MS);

      /* Send ERROR_INVALID_CMD response */
      fusain_packet_t error_pkt = {
        .address = FUSAIN_ADDRESS_STATELESS,
      };

      /* CBOR: [0xE0, {0: 1}] = ERROR_INVALID_CMD with error code 1 (invalid param) */
      uint8_t error_cbor[] = {
        0x82,       /* Array of 2 elements */
        0x18, 0xE0, /* uint(0xE0) = ERROR_INVALID_CMD */
        0xA1,       /* Map of 1 element */
        0x00,       /* Key 0 (error_code) */
        FUSAIN_ERRCODE_INVALID_PARAM, /* Value: 1 = invalid parameter */
      };
      error_pkt.length = sizeof(error_cbor);
      memcpy(error_pkt.payload, error_cbor, error_pkt.length);

      int ret = ws_send_packet(client_slot, &error_pkt);
      if (ret < 0) {
        LOG_ERR("Failed to send ERROR_INVALID_CMD: %d", ret);
      }
      break;
    }

    /* Apply valid configuration */
    clients[client_slot].rate_ms = interval_ms;
    LOG_INF("Client %d: TELEMETRY_CONFIG rate=%u ms", client_slot, interval_ms);
    break;
  }

  case FUSAIN_MSG_PING_REQUEST: {
    uint64_t target_address = packet->address;

    if (target_address == FUSAIN_ADDRESS_BROADCAST ||
        target_address == FUSAIN_ADDRESS_STATELESS) {
      /* Broadcast/stateless ping: update all subscriptions, respond with router uptime */
      LOG_DBG("Client %d: PING_REQUEST (broadcast)", client_slot);
      subscription_ping(client_slot, 0);

      fusain_packet_t pong = {
        .address = FUSAIN_ADDRESS_STATELESS,
      };

      /* Get router uptime in milliseconds */
      int64_t uptime_ms = k_uptime_get();

      /* CBOR: [0x3F, {0: uptime_ms}] */
      uint8_t cbor_payload[10] = {
        0x82, /* Array of 2 elements */
        0x18,
        0x3F, /* uint(0x3F) = PING_RESPONSE */
        0xA1, /* Map of 1 element */
        0x00, /* Key 0 (uptime_ms) */
        0x1A, /* 4-byte uint follows */
        (uint8_t)(uptime_ms >> 24),
        (uint8_t)(uptime_ms >> 16),
        (uint8_t)(uptime_ms >> 8),
        (uint8_t)(uptime_ms & 0xFF),
      };
      pong.length = 10;
      memcpy(pong.payload, cbor_payload, pong.length);

      int ret = ws_send_packet(client_slot, &pong);
      if (ret < 0) {
        LOG_ERR("Failed to send PING_RESPONSE: %d", ret);
      }
    } else {
      /* Addressed ping: update specific subscription, return cached ping_response */
      LOG_DBG("Client %d: PING_REQUEST for appliance 0x%llx", client_slot, target_address);

      if (!client_has_subscription(client_slot, target_address)) {
        LOG_DBG("Client %d not subscribed to 0x%llx, ignoring ping", client_slot, target_address);
        break;
      }

      /* Update ping timestamp for this specific subscription */
      subscription_ping(client_slot, target_address);

      /* Look up cached ping_response for this appliance */
      k_mutex_lock(&appliance_cache_mutex, K_FOREVER);

      bool found = false;
      for (int i = 0; i < CONFIG_SLATE_MAX_APPLIANCES; i++) {
        if (appliance_cache[i].address == target_address &&
            appliance_cache[i].ping_response.valid) {
          /* Send cached ping_response */
          int ret = ws_send_packet(client_slot, &appliance_cache[i].ping_response.packet);
          if (ret < 0) {
            LOG_ERR("Failed to send cached PING_RESPONSE: %d", ret);
          }
          found = true;
          break;
        }
      }

      k_mutex_unlock(&appliance_cache_mutex);

      if (!found) {
        LOG_DBG("No cached ping_response for appliance 0x%llx", target_address);
      }
    }
    break;
  }

  default: {
    /* Forward all other messages to Helios */
    int ret = serial_handler_send_packet(packet);
    if (ret < 0) {
      LOG_ERR("Failed to forward packet 0x%02x to serial: %d", message_type, ret);
    } else {
      LOG_DBG("Forwarded packet 0x%02x to serial", message_type);
    }
    break;
  }
  }
}

//////////////////////////////////////////////////////////////
// WebSocket Receive Handler (WebSocket → Router)
//////////////////////////////////////////////////////////////

static void ws_recv_work_handler(struct k_work* work)
{
  struct k_work_delayable* dwork = k_work_delayable_from_work(work);
  struct ws_client_ctx* ctx = CONTAINER_OF(dwork, struct ws_client_ctx, recv_work);

  /* Find client slot index */
  int client_slot = -1;
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    if (&clients[i] == ctx) {
      client_slot = i;
      break;
    }
  }

  if (client_slot < 0 || ctx->sock < 0) {
    return;
  }

  uint64_t remaining;
  uint32_t message_type;
  int ret;

  /* Non-blocking receive */
  ret = websocket_recv_msg(ctx->sock, ctx->recv_buffer, sizeof(ctx->recv_buffer),
      &message_type, &remaining, WS_RECV_TIMEOUT_MS);

  if (ret < 0) {
    if (ret == -EAGAIN || ret == -EWOULDBLOCK) {
      /* No data available - reschedule */
      goto reschedule;
    }

    /* Connection closed or error - cleanup */
    LOG_INF("WebSocket client disconnected (slot %d, reason %d)", client_slot, ret);
    subscription_remove_client(client_slot);
    websocket_unregister(ctx->sock);
    ctx->sock = -1;
    return;
  }

  if (ret == 0) {
    /* Connection closed gracefully */
    LOG_INF("WebSocket connection closed");
    subscription_remove_client(client_slot);
    websocket_unregister(ctx->sock);
    ctx->sock = -1;
    return;
  }

  /* Process received bytes through Fusain decoder */
  for (int i = 0; i < ret; i++) {
    fusain_packet_t packet;
    fusain_decode_result_t decode_ret = fusain_decode_byte(ctx->recv_buffer[i], &packet, &ctx->decoder);

    if (decode_ret == FUSAIN_DECODE_OK) {
      /* Complete packet received - process as router */
      ws_process_router_message(client_slot, &packet);
    } else if (decode_ret != FUSAIN_DECODE_INCOMPLETE) {
      /* Decode error */
      LOG_WRN("Fusain decode error: %d", decode_ret);
      fusain_reset_decoder(&ctx->decoder);
    }
  }

reschedule:
  /* Reschedule for next receive */
  k_work_reschedule_for_queue(&ws_bridge_queue, &ctx->recv_work, K_MSEC(10));
}

//////////////////////////////////////////////////////////////
// WebSocket Connection Setup
//////////////////////////////////////////////////////////////

/* Register Authorization header for capture */
HTTP_SERVER_REGISTER_HEADER_CAPTURE(auth_header, "Authorization");

static int get_free_slot(void)
{
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    if (clients[i].sock < 0) {
      return i;
    }
  }
  return -1;
}

/* Validate HTTP Basic Authorization header */
static bool validate_authorization(struct http_request_ctx* request_ctx)
{
  const char* auth_value = NULL;

  /* Check if headers were dropped due to buffer overflow */
  if (request_ctx->headers_status == HTTP_HEADER_STATUS_DROPPED) {
    LOG_ERR("Headers were dropped - possible buffer overflow attack");
    return false;
  }

  /* Find Authorization header in captured headers (case-insensitive) */
  for (size_t i = 0; i < request_ctx->header_count; i++) {
    if (strcasecmp(request_ctx->headers[i].name, "Authorization") == 0) {
      auth_value = request_ctx->headers[i].value;
      break;
    }
  }

  if (auth_value == NULL) {
    LOG_WRN("Missing Authorization header");
    return false;
  }

  /* Reject Bearer tokens - not supported */
  if (strncmp(auth_value, "Bearer ", 7) == 0) {
    LOG_WRN("Bearer tokens are not supported, use HTTP Basic auth");
    return false;
  }

  /* HTTP Basic auth: "Basic <base64(username:password)>" */
  if (strncmp(auth_value, "Basic ", 6) != 0) {
    LOG_WRN("Unsupported authorization scheme");
    return false;
  }

  const char* base64_credentials = auth_value + 6;
  size_t base64_len = strlen(base64_credentials);

  /* Decode base64 credentials */
  uint8_t decoded[AUTH_DECODED_BUFFER_SIZE];
  size_t decoded_len;
  int ret = base64_decode(decoded, sizeof(decoded) - 1, &decoded_len,
      (const uint8_t*)base64_credentials, base64_len);

  if (ret == -ENOMEM) {
    LOG_WRN("Credentials too long");
    return false;
  }
  if (ret == -EINVAL) {
    LOG_WRN("Invalid base64 encoding in credentials");
    return false;
  }
  if (ret != 0) {
    LOG_WRN("Base64 decode failed: %d", ret);
    return false;
  }

  /* Null-terminate the decoded string */
  decoded[decoded_len] = '\0';

  /* Find the colon separator between username and password */
  char* colon = strchr((char*)decoded, ':');
  if (colon == NULL) {
    LOG_WRN("Invalid credentials format (missing colon)");
    return false;
  }

  /* Split into username and password */
  *colon = '\0';
  const char* username = (char*)decoded;
  const char* password = colon + 1;

  /* Validate credentials using constant-time comparison to prevent timing attacks */
  int user_match = constant_time_strcmp(username, AUTH_USERNAME);
  int pass_match = constant_time_strcmp(password, AUTH_PASSWORD);
  if (user_match != 0 || pass_match != 0) {
    LOG_WRN("Invalid username or password");
    return false;
  }

  LOG_DBG("HTTP Basic authorization successful");
  return true;
}

int ws_fusain_setup(int ws_socket, struct http_request_ctx* request_ctx,
    void* user_data)
{
  /* Validate authorization */
  if (!validate_authorization(request_ctx)) {
    LOG_WRN("WebSocket connection rejected: authentication failed");
    return -EACCES;
  }

  int slot = get_free_slot();
  if (slot < 0) {
    LOG_ERR("No free WebSocket slots (max %d clients)", WS_MAX_CLIENTS);
    return -ENOENT;
  }

  clients[slot].sock = ws_socket;
  fusain_reset_decoder(&clients[slot].decoder);

  LOG_INF("WebSocket client connected (slot %d, fd %d)", slot, ws_socket);

  /* Start recv_work with small delay to let connection stabilize */
  k_work_reschedule_for_queue(&ws_bridge_queue, &clients[slot].recv_work,
      K_MSEC(100));

  return 0;
}

//////////////////////////////////////////////////////////////
// HTTP Resource Detail (Exported for main.c)
//////////////////////////////////////////////////////////////

/* WebSocket buffer for HTTP server */
uint8_t ws_fusain_buffer[WS_BUFFER_SIZE];

struct http_resource_detail_websocket ws_fusain_resource_detail = {
  .common = {
      .type = HTTP_RESOURCE_TYPE_WEBSOCKET,
      .bitmask_of_supported_http_methods = BIT(HTTP_GET),
  },
  .cb = ws_fusain_setup,
  .data_buffer = ws_fusain_buffer,
  .data_buffer_len = sizeof(ws_fusain_buffer),
};

//////////////////////////////////////////////////////////////
// HTTP Stats Endpoint
//////////////////////////////////////////////////////////////

static uint8_t stats_response_buffer[512];

static int stats_handler(struct http_client_ctx* client, enum http_data_status status,
    const struct http_request_ctx* request_ctx,
    struct http_response_ctx* response_ctx, void* user_data)
{
  if (status != HTTP_SERVER_DATA_FINAL) {
    return 0;
  }

  /* Count connected WebSocket clients */
  int ws_clients = 0;
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    if (clients[i].sock >= 0) {
      ws_clients++;
    }
  }

  int len = snprintf((char*)stats_response_buffer, sizeof(stats_response_buffer),
      "WebSocket Bridge\n"
      "================\n"
      "Connected clients: %d/%d\n",
      ws_clients, WS_MAX_CLIENTS);

  response_ctx->body = stats_response_buffer;
  response_ctx->body_len = len;
  response_ctx->final_chunk = true;
  response_ctx->status = HTTP_200_OK;

  return 0;
}

struct http_resource_detail_dynamic stats_resource_detail = {
  .common = {
      .type = HTTP_RESOURCE_TYPE_DYNAMIC,
      .bitmask_of_supported_http_methods = BIT(HTTP_GET),
  },
  .cb = stats_handler,
  .user_data = NULL,
};

//////////////////////////////////////////////////////////////
// Initialization
//////////////////////////////////////////////////////////////

static int websocket_bridge_init(void)
{
  struct k_work_queue_config cfg = { .name = "ws_bridge" };

  /* Initialize subscription table */
  k_mutex_init(&subscription_mutex);
  for (int i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    subscriptions[i].appliance_address = 0;
    subscriptions[i].client_slot = -1;
    subscriptions[i].last_ping_time = 0;
  }

  /* Initialize appliance cache */
  k_mutex_init(&appliance_cache_mutex);
  for (int i = 0; i < CONFIG_SLATE_MAX_APPLIANCES; i++) {
    appliance_cache[i].address = 0;
    appliance_cache[i].last_seen = 0;
    appliance_cache[i].state_data.valid = false;
    appliance_cache[i].motor_data.valid = false;
    appliance_cache[i].pump_data.valid = false;
    appliance_cache[i].glow_data.valid = false;
    appliance_cache[i].temp_data.valid = false;
    appliance_cache[i].ping_response.valid = false;
  }

  /* Initialize HTTP server control mutex */
  k_mutex_init(&http_server_mutex);

  /* Initialize work queue */
  k_work_queue_init(&ws_bridge_queue);
  k_work_queue_start(&ws_bridge_queue, ws_bridge_stack, WS_STACK_SIZE, 5, &cfg);

  /* Initialize periodic telemetry work */
  k_work_init_delayable(&telemetry_work, telemetry_work_handler);

  /* Initialize client contexts */
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    clients[i].sock = -1;
    clients[i].rate_ms = DEFAULT_RATE_MS;
    clients[i].last_send_time = 0;
    fusain_reset_decoder(&clients[i].decoder);
    k_work_init_delayable(&clients[i].recv_work, ws_recv_work_handler);
  }

  /* Start periodic telemetry work */
  k_work_reschedule_for_queue(&ws_bridge_queue, &telemetry_work, K_MSEC(TELEMETRY_TICK_MS));

  /* Start HTTP server - it needs to be running before WiFi connects */
  http_server_start();
  http_server_running = true;

  LOG_INF("WebSocket router initialized on port 80");
  LOG_INF("Endpoint: ws://<hostname>/fusain");
  LOG_INF("Max clients: %d, Max subscriptions: %d, Max appliances: %d",
      WS_MAX_CLIENTS, MAX_SUBSCRIPTIONS, CONFIG_SLATE_MAX_APPLIANCES);

  return 0;
}

SYS_INIT(websocket_bridge_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
