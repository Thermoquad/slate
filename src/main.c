/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2025 Kaz Walker, Thermoquad
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/zbus/zbus.h>

#include <slate/config.h>
#include <slate/display.h>
#include <slate/serial_handler.h>
#include <slate/wifi_config.h>
#include <slate/zbus.h>

/* WebSocket bridge exports (from websocket_bridge.c) */
extern struct http_resource_detail_websocket ws_fusain_resource_detail;
extern struct http_resource_detail_dynamic stats_resource_detail;
extern int ws_fusain_setup(int ws_socket, struct http_request_ctx *request_ctx,
                           void *user_data);
extern void fusain_raw_rx_callback(const struct zbus_channel *chan);

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

//////////////////////////////////////////////////////////////
// Zbus Observers
//////////////////////////////////////////////////////////////

/* WebSocket bridge listener - callback defined in websocket_bridge.c */
ZBUS_LISTENER_DEFINE(ws_bridge_listener, fusain_raw_rx_callback);

/* Display listener - callback defined in display.c */
ZBUS_LISTENER_DEFINE(display_listener, display_raw_rx_callback);

//////////////////////////////////////////////////////////////
// Zbus Channel Definitions
//////////////////////////////////////////////////////////////

/**
 * Helios state command channel
 *
 * Used by shell commands to request mode changes on the remote
 * Helios ICU. The serial handler subscribes and sends the
 * appropriate SET_MODE protocol messages.
 */
ZBUS_CHAN_DEFINE(helios_state_command_chan,
    fusain_state_command_msg_t,
    NULL, // No validator
    NULL, // No user data
    ZBUS_OBSERVERS(serial_handler_listener), // Serial handler subscribes
    ZBUS_MSG_INIT(.mode = FUSAIN_MODE_IDLE, .argument = 0) // Initialize to idle
);

/**
 * Raw Fusain packet RX channel
 *
 * Published by serial handler when any packet is received from Helios.
 * WebSocket bridge and display subscribe to receive packets.
 */
ZBUS_CHAN_DEFINE(fusain_raw_rx_chan,
    fusain_raw_packet_msg_t,
    NULL, // No validator
    NULL, // No user data
    ZBUS_OBSERVERS(ws_bridge_listener, display_listener),
    ZBUS_MSG_INIT(.timestamp_us = 0) // Initialize with zero timestamp
);

//////////////////////////////////////////////////////////////
// HTTP Service and WebSocket Resource
//////////////////////////////////////////////////////////////

#define HTTP_PORT 80

static uint16_t http_port = HTTP_PORT;

/* HTTP service on port 80 */
HTTP_SERVICE_DEFINE(fusain_bridge_service, NULL, &http_port,
                    CONFIG_HTTP_SERVER_MAX_CLIENTS, 10, NULL, NULL, NULL);

/* WebSocket endpoint at /fusain */
HTTP_RESOURCE_DEFINE(ws_fusain, fusain_bridge_service, "/fusain",
                     &ws_fusain_resource_detail);

/* Stats endpoint at /stats */
HTTP_RESOURCE_DEFINE(http_stats, fusain_bridge_service, "/stats",
                     &stats_resource_detail);

//////////////////////////////////////////////////////////////
// Input Callback
//////////////////////////////////////////////////////////////

static void test_cb(struct input_event* evt, void* p)
{
  if (evt->code == INPUT_REL_WHEEL) {
    printk("z event %d\n", evt->value);
  }
  if (evt->code == INPUT_BTN_1 && evt->value == 1) {
    printk("short press %d\n", evt->value);
  }
  if (evt->code == INPUT_BTN_2 && evt->value == 1) {
    printk("long press %d\n", evt->value);
  }
}

INPUT_CALLBACK_DEFINE(NULL, test_cb, NULL);

/* Thread Definitions */
K_THREAD_DEFINE(serial_rx_id, CONFIG_MAIN_STACK_SIZE, serial_rx_thread, NULL, NULL, NULL, -2, 0, 0); // RX: Highest priority - time-critical UART RX polling
K_THREAD_DEFINE(serial_tx_id, CONFIG_MAIN_STACK_SIZE, serial_tx_thread, NULL, NULL, NULL, 0, 0, 0);  // TX: Priority 0 - UART TX polling and queue processing
K_THREAD_DEFINE(serial_processing_id, CONFIG_MAIN_STACK_SIZE, serial_processing_thread, NULL, NULL, NULL, 1, 0, 0); // Processing: Priority 1 - protocol logic
K_THREAD_DEFINE(display_id, 8192, display_thread, NULL, NULL, NULL, 5, 0, 0); // Lower priority - display can wait

int main(void)
{
  LOG_INF("Slate controller starting");

  // Initialize configuration storage
  int rc = config_init();
  if (rc != 0) {
    LOG_ERR("Failed to initialize config storage: %d", rc);
    return rc;
  }

  // Check if WiFi credentials are stored
  if (wifi_config_exists()) {
    LOG_INF("WiFi credentials found in storage");
  } else {
    LOG_INF("No WiFi credentials stored - use 'wifi_save' command");
  }

  return 0;
}
