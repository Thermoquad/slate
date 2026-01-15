/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Slate Serial Master - Helios Protocol Implementation
 *
 * Implements master-side Helios serial protocol communication.
 * Sends commands and pings to Helios ICU, receives telemetry responses.
 */

//////////////////////////////////////////////////////////////
// Includes
//////////////////////////////////////////////////////////////

#include <fusain/fusain.h>
#include <fusain/generated/cbor_decode.h>
#include <fusain/generated/cbor_types.h>
#include <slate/serial_handler.h>
#include <slate/zbus.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

LOG_MODULE_REGISTER(slate_serial_handler);

#define LOOP_SLEEP_US 500 // No sleep - UART FIFO (32 bytes) fills in 2780us at 115200 baud, poll continuously
#define TELEMETRY_REQUEST_INTERVAL_MS 500
#define PING_INTERVAL_MS 10000
#define TELEMETRY_TIMEOUT_MS 30000
#define EMERGENCY_STOP_RETRANSMIT_MS 250
#define PUB_TIMEOUT K_MSEC(10)

//////////////////////////////////////////////////////////////
// State Struct Definition
//////////////////////////////////////////////////////////////

struct serial_state {
  // Telemetry tracking
  bool telemetry_received;
  uint64_t last_telemetry_time;
  uint64_t last_telemetry_request_time;

  // Emergency stop
  bool emergency_stop_active;
  bool emergency_stop_confirmed;
  uint64_t last_emergency_stop_time;

  // Helios state (from telemetry)
  fusain_state_t helios_state;
  fusain_error_t helios_error;
  uint32_t helios_uptime_ms;

  // Telemetry values (from individual messages)
  float temperature;
  int32_t motor_rpm;
  int32_t motor_target_rpm;

  // Ping tracking
  bool ping_response_received;
  uint64_t last_ping_time;
};

//////////////////////////////////////////////////////////////
// Static Variables
//////////////////////////////////////////////////////////////

/* UART Device */
static const struct device* uart_dev;

/* Decoder State */
static fusain_decoder_t decoder;

/* TX Buffer and State */
static uint8_t tx_buffer[FUSAIN_MAX_PACKET_SIZE * 2]; // 2x for stuffing overhead
static size_t tx_index = 0;
static size_t tx_length = 0;

/* TX Packet Queue - API pushes, thread pops */
K_MSGQ_DEFINE(tx_packet_queue, sizeof(fusain_packet_t), 8, 4);

/* RX Packet Queue - ISR pushes, thread pops */
K_MSGQ_DEFINE(rx_packet_queue, sizeof(fusain_packet_t), 8, 4);

/* Serial State */
static struct serial_state serial_state;

//////////////////////////////////////////////////////////////
// CBOR Helper
//////////////////////////////////////////////////////////////

/**
 * Get CBOR message header length based on msg_type
 *
 * CBOR wire format: [0x82, msg_type, payload_map]
 * - msg_type 0x00-0x17: header is [0x82, type] = 2 bytes
 * - msg_type 0x18-0xFF: header is [0x82, 0x18, type] = 3 bytes
 */
static inline size_t cbor_header_len(uint8_t msg_type)
{
  return (msg_type <= 0x17) ? 2 : 3;
}

//////////////////////////////////////////////////////////////
// Forward Declarations
//////////////////////////////////////////////////////////////

static void process_packet(const fusain_packet_t* packet, struct serial_state* state,
    uint64_t current_micros);
static void send_packet(const fusain_packet_t* packet);
static void fill_transmit_buffer(const fusain_packet_t* packet);
static void poll_uart_rx(void);
static void poll_uart_tx(void);
static void process_rx_packets(struct serial_state* state, uint64_t current_micros);
static void process_tx_queue(void);
static void handle_emergency_stop(struct serial_state* state, uint64_t current_micros);
static void check_telemetry_timeout(struct serial_state* state, uint64_t current_micros);
static void handle_telemetry_config(struct serial_state* state, uint64_t current_micros);
static void handle_ping(struct serial_state* state, uint64_t current_micros);

//////////////////////////////////////////////////////////////
// Public API
//////////////////////////////////////////////////////////////

/**
 * Send Ping Request
 *
 * Queues a ping request packet for transmission to Helios ICU.
 */
void helios_send_ping(void)
{
  fusain_packet_t packet;
  fusain_create_ping_request(&packet, 0); // Broadcast address
  LOG_DBG(">>> SENDING PING REQUEST to Helios");
  send_packet(&packet);
}

/**
 * Send Telemetry Config Command
 *
 * Configures telemetry broadcast settings on Helios ICU.
 *
 * @param enabled Enable/disable telemetry broadcasts
 * @param interval_ms Telemetry broadcast interval (100-5000 ms)
 */
void helios_send_telemetry_config(bool enabled, uint32_t interval_ms)
{
  fusain_packet_t packet;
  fusain_create_telemetry_config(&packet, 0, enabled, interval_ms); // Broadcast address
  send_packet(&packet);
  LOG_DBG("Telemetry config: enabled=%d, interval=%u ms", enabled, interval_ms);
}

/**
 * Send State Command
 *
 * Sends a mode change command to Helios ICU.
 *
 * @param mode Operating mode (IDLE, FAN, HEAT, EMERGENCY)
 * @param argument Mode-specific argument (e.g., RPM for FAN, pump rate for HEAT)
 */
void helios_set_mode(fusain_mode_t mode, int32_t argument)
{
  fusain_packet_t packet;
  fusain_create_state_command(&packet, 0, mode, argument); // Broadcast address
  send_packet(&packet);
  LOG_DBG("Set mode: mode=%u, argument=%d", mode, argument);
}

/**
 * Send Emergency Stop Command
 *
 * Initiates emergency stop procedure.
 * Command will be retransmitted every 250ms until confirmed by Helios.
 */
void helios_send_emergency_stop(void)
{
  serial_state.emergency_stop_active = true;
  serial_state.emergency_stop_confirmed = false;
  LOG_WRN("Emergency stop initiated - retransmitting every %d ms until confirmed",
      EMERGENCY_STOP_RETRANSMIT_MS);
}

/**
 * Get Helios State
 *
 * Returns last known state and error from telemetry.
 *
 * @param state Output state (can be NULL)
 * @param error Output error (can be NULL)
 */
void helios_get_state(fusain_state_t* state, fusain_error_t* error)
{
  if (state) {
    *state = serial_state.helios_state;
  }
  if (error) {
    *error = serial_state.helios_error;
  }
}

//////////////////////////////////////////////////////////////
// Thread Functions
//////////////////////////////////////////////////////////////

/**
 * Serial RX Thread - High-priority UART reception only
 *
 * Dedicated thread for receiving bytes and decoding packets.
 * Runs at highest priority with fast polling to prevent FIFO overflow.
 * Only polls UART and queues packets - does not process them.
 */
int serial_rx_thread(void)
{
  LOG_DBG("Serial RX thread started");

  // Initialize serial communication with Helios (shared initialization)
  int ret = serial_master_init();
  if (ret < 0) {
    LOG_ERR("Failed to initialize serial master: %d", ret);
    return ret;
  }

  // Initialize state
  serial_state.telemetry_received = false;
  serial_state.last_telemetry_time = 0;
  serial_state.last_telemetry_request_time = 0;
  serial_state.emergency_stop_active = false;
  serial_state.emergency_stop_confirmed = false;
  serial_state.last_emergency_stop_time = 0;
  serial_state.helios_state = FUSAIN_STATE_INITIALIZING;
  serial_state.helios_error = FUSAIN_ERROR_NONE;
  serial_state.helios_uptime_ms = 0;
  serial_state.temperature = 0.0;
  serial_state.motor_rpm = 0;
  serial_state.motor_target_rpm = 0;
  serial_state.ping_response_received = false;
  serial_state.last_ping_time = 0;

  while (1) {
    // Poll UART for incoming data and decode (time-critical)
    poll_uart_rx();

    k_sleep(K_USEC(LOOP_SLEEP_US));
  }
}

/**
 * Serial TX Thread - UART transmission only
 *
 * Handles UART TX polling and TX queue processing.
 * Pops packets from queue and fills TX buffer, then polls UART to send bytes.
 */
int serial_tx_thread(void)
{
  LOG_DBG("Serial TX thread started");

  // Wait for RX thread to initialize serial
  k_sleep(K_MSEC(100));

  while (1) {
    // Poll UART for TX readiness and send queued data
    poll_uart_tx();

    // Process one pending TX packet if available (pops from queue, fills buffer)
    process_tx_queue();
    k_sleep(K_MSEC(1)); // TX thread can run slower (1ms)
  }
}

/**
 * Serial Processing Thread - Protocol logic and packet processing
 *
 * Handles all higher-level protocol logic:
 * - Processing received packets from RX queue
 * - Emergency stop retransmission
 * - Telemetry timeout checking
 * - Telemetry configuration requests
 *
 * Runs at moderate priority between RX and TX.
 */
int serial_processing_thread(void)
{
  LOG_DBG("Serial processing thread started");

  // Wait for RX thread to initialize serial and state
  k_sleep(K_MSEC(200));

  while (1) {
    const uint64_t current_micros = k_cyc_to_us_floor64(k_cycle_get_64());

    // Process all pending RX packets (pops from queue, processes)
    process_rx_packets(&serial_state, current_micros);

    // Handle emergency stop (priority - retransmits every 250ms)
    handle_emergency_stop(&serial_state, current_micros);

    // Handle periodic pings (generates packets, puts in TX queue)
    handle_ping(&serial_state, current_micros);

    // Check telemetry timeout (30s)
    check_telemetry_timeout(&serial_state, current_micros);

    // Handle telemetry config requests (sends until first telemetry received)
    handle_telemetry_config(&serial_state, current_micros);

    k_sleep(K_MSEC(10)); // Processing thread runs at 10ms
  }
}

//////////////////////////////////////////////////////////////
// Init Functions
//////////////////////////////////////////////////////////////

/**
 * Initialize Serial Master
 *
 * Initializes UART device and decoder for polling mode.
 * Must be called before starting serial thread.
 *
 * @return 0 on success, negative on error
 */
int serial_master_init(void)
{
  // Get UART device
  uart_dev = DEVICE_DT_GET(DT_ALIAS(helios_uart));
  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not ready");
    return -1;
  }

  LOG_DBG("UART device ready: %s", uart_dev->name);

  // Initialize decoder
  fusain_reset_decoder(&decoder);
  LOG_DBG("Decoder initialized");

  // Flush any pending RX data using polling
  uint8_t discard;
  int flushed = 0;
  while (uart_poll_in(uart_dev, &discard) == 0) {
    flushed++;
  }
  if (flushed > 0) {
    LOG_DBG("Flushed %d bytes from RX buffer", flushed);
  }

  LOG_INF("Serial master initialized on %s (polling mode)", uart_dev->name);

  return 0;
}

//////////////////////////////////////////////////////////////
// Hardware Functions
//////////////////////////////////////////////////////////////

/**
 * Poll UART RX - Decodes incoming bytes and queues complete packets
 *
 * Processes up to 32 bytes per poll to avoid blocking.
 */
static void poll_uart_rx(void)
{
  static uint32_t bytes_read_count = 0;
  static uint32_t packets_decoded_count = 0;
  uint8_t byte;
  int max_bytes = 32; // Limit bytes per poll to keep loop responsive

  while (max_bytes-- > 0 && uart_poll_in(uart_dev, &byte) == 0) {
    bytes_read_count++;
    fusain_packet_t packet;

    // Save state BEFORE decoding for diagnostics
    uint8_t prev_state = decoder.state;
    size_t prev_index = decoder.buffer_index;

    fusain_decode_result_t result = fusain_decode_byte(byte, &packet, &decoder);

    if (result == FUSAIN_DECODE_OK) {
      packets_decoded_count++;
      LOG_DBG("RX: Packet decoded type=0x%02X (total: %u packets, %u bytes)",
          packet.msg_type, packets_decoded_count, bytes_read_count);

      // Packet complete - queue for processing in thread context
      int ret = k_msgq_put(&rx_packet_queue, &packet, K_NO_WAIT);
      if (ret != 0) {
        // Queue full - drop packet and log error
        LOG_ERR("RX queue full, dropping packet type 0x%02X", packet.msg_type);
      }
    } else if (result != FUSAIN_DECODE_INCOMPLETE) {
      // Decode error - reset decoder and continue
      LOG_ERR("DECODE ERROR: result=%d, last_byte=0x%02X",
          result, byte);
      LOG_ERR("  State: prev=%u → curr=%u, Index: prev=%zu → curr=%zu",
          prev_state, decoder.state, prev_index, decoder.buffer_index);
      LOG_ERR("  escape_next=%d", decoder.escape_next);

      // Log last few bytes in decoder buffer to help identify corruption
      if (decoder.buffer_index > 0) {
        size_t log_start = (decoder.buffer_index > 16) ? decoder.buffer_index - 16 : 0;
        LOG_ERR("  Last bytes in buffer (from [%zu]):", log_start);
        for (size_t i = log_start; i < decoder.buffer_index && i < log_start + 16; i += 8) {
          size_t bytes_left = (decoder.buffer_index - i < 8) ? decoder.buffer_index - i : 8;
          if (bytes_left >= 8) {
            LOG_ERR("    [%02zu]: %02X %02X %02X %02X %02X %02X %02X %02X", i,
                decoder.buffer[i + 0], decoder.buffer[i + 1], decoder.buffer[i + 2], decoder.buffer[i + 3],
                decoder.buffer[i + 4], decoder.buffer[i + 5], decoder.buffer[i + 6], decoder.buffer[i + 7]);
          } else {
            // Print remaining bytes with conditional formatting
            LOG_ERR("    [%02zu]: partial (%zu bytes)", i, bytes_left);
          }
        }
      }

      fusain_reset_decoder(&decoder);
    }
  }
}

/**
 * Poll UART TX - Sends buffered data without blocking RX
 *
 * Sends up to 32 bytes per poll to avoid blocking RX polling.
 */
static void poll_uart_tx(void)
{
  if (tx_index < tx_length) {
    // Send up to 32 bytes per poll to keep loop responsive
    int max_bytes = 32;
    while (tx_index < tx_length && max_bytes-- > 0) {
      uart_poll_out(uart_dev, tx_buffer[tx_index]);
      tx_index++;
    }

    // Transmission complete
    if (tx_index >= tx_length) {
      LOG_DBG("TX complete: %zu bytes sent", tx_length);
    }
  }
}

//////////////////////////////////////////////////////////////
// Helper Functions - Packet Processing
//////////////////////////////////////////////////////////////

/**
 * Publish telemetry to Zbus
 *
 * Publishes collected telemetry data to the Zbus channel.
 * Called when we have enough data to send a meaningful update.
 */
static void publish_telemetry(struct serial_state* state)
{
  helios_telemetry_msg_t telemetry_msg = {
    .state = state->helios_state,
    .error = state->helios_error,
    .temperature = state->temperature,
    .motor_rpm = state->motor_rpm,
    .motor_target_rpm = state->motor_target_rpm,
    .valid = true
  };

  int ret = zbus_chan_pub(&helios_telemetry_chan, &telemetry_msg, PUB_TIMEOUT);
  if (ret != 0) {
    LOG_WRN("Failed to publish telemetry to Zbus: %d", ret);
  }

  LOG_DBG("Telemetry published: state=%d, temp=%.1f, rpm=%d",
      telemetry_msg.state, telemetry_msg.temperature, telemetry_msg.motor_rpm);
}

/**
 * Process Received Packet
 *
 * Called from serial thread to process packets queued by UART ISR.
 * Handles ping responses, state data, motor data, temperature data, and error messages.
 */
static void process_packet(const fusain_packet_t* packet, struct serial_state* state,
    uint64_t current_micros)
{
  LOG_DBG("Processing packet: type=0x%02X, length=%u", packet->msg_type,
      packet->length);

  switch (packet->msg_type) {
  case FUSAIN_MSG_PING_RESPONSE: {
    struct ping_response_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_ping_response_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode PING_RESPONSE: %d", ret);
      break;
    }
    state->helios_uptime_ms = decoded.ping_response_payload_timestamp_m;
    state->ping_response_received = true;
    LOG_DBG("Ping response received (uptime=%u ms)", state->helios_uptime_ms);
    break;
  }

  case FUSAIN_MSG_STATE_DATA: {
    struct state_data_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_state_data_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode STATE_DATA: %d", ret);
      break;
    }

    state->helios_state = (fusain_state_t)decoded.state_data_payload_state_m;
    state->helios_error = (fusain_error_t)decoded.state_data_payload_error_code_m;
    LOG_DBG("STATE_DATA: state=%u, error=%u",
        decoded.state_data_payload_state_m, decoded.state_data_payload_error_code_m);

    // Mark that we've received state data (counts as telemetry)
    if (!state->telemetry_received) {
      LOG_INF("State data received - telemetry enabled");
      state->telemetry_received = true;
    }
    state->last_telemetry_time = current_micros;

    // Check for emergency stop confirmation
    if (state->emergency_stop_active && decoded.state_data_payload_state_m == FUSAIN_STATE_E_STOP) {
      LOG_INF("Emergency stop confirmed by Helios - stopping retransmission");
      state->emergency_stop_confirmed = true;
      state->emergency_stop_active = false;
    }

    LOG_DBG("State data: state=%u, error=%u", decoded.state_data_payload_state_m,
        decoded.state_data_payload_error_code_m);

    // Publish updated telemetry
    publish_telemetry(state);
    break;
  }

  case FUSAIN_MSG_MOTOR_DATA: {
    struct motor_data_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_motor_data_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode MOTOR_DATA: %d", ret);
      break;
    }

    // Store motor data (use motor index 0 for display)
    if (decoded.motor_data_payload_motor_index_m == 0) {
      state->motor_rpm = decoded.motor_data_payload_uint2int;
      state->motor_target_rpm = decoded.motor_data_payload_uint3int;

      // Log suspicious high values
      if (state->motor_rpm > 6000 || state->motor_target_rpm > 6000) {
        LOG_ERR("ANOMALY: Suspicious RPM values detected!");
        LOG_ERR("  Motor %u: rpm=%d, target=%d",
            decoded.motor_data_payload_motor_index_m,
            state->motor_rpm, state->motor_target_rpm);
      }

      LOG_DBG("Motor %u: RPM=%d, target=%d",
          decoded.motor_data_payload_motor_index_m,
          state->motor_rpm, state->motor_target_rpm);
    }

    state->last_telemetry_time = current_micros;
    break;
  }

  case FUSAIN_MSG_TEMP_DATA: {
    struct temp_data_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_temp_data_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode TEMP_DATA: %d", ret);
      break;
    }

    // Store temperature data (use thermometer index 0 for display)
    if (decoded.temp_data_payload_thermometer_index_m == 0) {
      state->temperature = decoded.temp_data_payload_uint2float;
      LOG_DBG("Temp %u: %.1f°C", decoded.temp_data_payload_thermometer_index_m,
          decoded.temp_data_payload_uint2float);
    }

    state->last_telemetry_time = current_micros;
    break;
  }

  case FUSAIN_MSG_DEVICE_ANNOUNCE: {
    struct device_announce_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_device_announce_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode DEVICE_ANNOUNCE: %d", ret);
      break;
    }
    LOG_INF("Device announce: motors=%u, temps=%u, pumps=%u, glows=%u",
        decoded.device_announce_payload_uint0uint,
        decoded.device_announce_payload_uint1uint,
        decoded.device_announce_payload_uint2uint,
        decoded.device_announce_payload_uint3uint);
    break;
  }

  case FUSAIN_MSG_ERROR_INVALID_CMD: {
    struct error_invalid_cmd_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_error_invalid_cmd_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode ERROR_INVALID_CMD: %d", ret);
      break;
    }
    LOG_ERR("Helios reported invalid command error: code=%d",
        decoded.error_invalid_cmd_payload_uint0int);
    break;
  }

  case FUSAIN_MSG_ERROR_STATE_REJECT: {
    struct error_state_reject_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_error_state_reject_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode ERROR_STATE_REJECT: %d", ret);
      break;
    }
    LOG_ERR("Helios rejected command: current state=%u",
        decoded.error_state_reject_payload_state_m);
    break;
  }

  default:
    LOG_WRN("Unknown message type: 0x%02X", packet->msg_type);
    break;
  }
}

/**
 * Process RX Packets
 *
 * Drains RX packet queue and processes all pending packets.
 */
static void process_rx_packets(struct serial_state* state, uint64_t current_micros)
{
  fusain_packet_t rx_packet;
  while (k_msgq_get(&rx_packet_queue, &rx_packet, K_NO_WAIT) == 0) {
    process_packet(&rx_packet, state, current_micros);
  }
}

/**
 * Process TX Queue
 *
 * Processes one packet from TX queue if available.
 */
static void process_tx_queue(void)
{
  fusain_packet_t tx_packet;
  if (k_msgq_get(&tx_packet_queue, &tx_packet, K_NO_WAIT) == 0) {
    fill_transmit_buffer(&tx_packet);
  }
}

/**
 * Transmit Packet - Actual UART transmission
 *
 * Encodes packet to buffer. Polling loop will send it.
 * Called from serial thread.
 */
static void fill_transmit_buffer(const fusain_packet_t* packet)
{
  // Encode packet to buffer
  int len = fusain_encode_packet(packet, tx_buffer, sizeof(tx_buffer));
  if (len < 0) {
    LOG_ERR("Encoding failed: %d", len);
    return;
  }

  // Prepare TX state
  tx_index = 0;
  tx_length = (size_t)len;

  LOG_DBG("Starting TX: type=0x%02X, %zu bytes", packet->msg_type, tx_length);
}

/**
 * Queue Packet for Transmission
 *
 * Queues packet for transmission by serial thread.
 * Called from public API functions.
 */
static void send_packet(const fusain_packet_t* packet)
{
  int ret = k_msgq_put(&tx_packet_queue, packet, K_NO_WAIT);
  if (ret != 0) {
    LOG_ERR("TX queue full, dropping packet type 0x%02X", packet->msg_type);
  } else {
    // Log successful queueing of pings for debugging
    if (packet->msg_type == FUSAIN_MSG_PING_REQUEST) {
      LOG_DBG("Ping queued for transmission");
    }
  }
}

//////////////////////////////////////////////////////////////
// Helper Functions - Protocol Handlers
//////////////////////////////////////////////////////////////

/**
 * Handle Emergency Stop
 *
 * Retransmits emergency stop every 250ms until confirmed.
 */
static void handle_emergency_stop(struct serial_state* state, uint64_t current_micros)
{
  if (!state->emergency_stop_active || state->emergency_stop_confirmed) {
    return;
  }

  const uint64_t micros_since_last = current_micros - state->last_emergency_stop_time;
  if (micros_since_last < (EMERGENCY_STOP_RETRANSMIT_MS * 1000)) {
    return;
  }

  fusain_packet_t packet;
  fusain_create_state_command(&packet, 0, FUSAIN_MODE_EMERGENCY, 0); // Broadcast address
  fill_transmit_buffer(&packet);
  state->last_emergency_stop_time = current_micros;
}

/**
 * Check Telemetry Timeout
 *
 * Resets telemetry state if no telemetry received for 30 seconds.
 */
static void check_telemetry_timeout(struct serial_state* state, uint64_t current_micros)
{
  if (!state->telemetry_received || state->last_telemetry_time == 0) {
    return;
  }

  const uint64_t micros_since_telemetry = current_micros - state->last_telemetry_time;
  if (micros_since_telemetry > (TELEMETRY_TIMEOUT_MS * 1000)) {
    LOG_WRN("Telemetry timeout - no telemetry for %d ms, re-enabling", TELEMETRY_TIMEOUT_MS);
    state->telemetry_received = false;
    state->last_telemetry_time = 0;
  }
}

/**
 * Handle Telemetry Config
 *
 * Sends telemetry config every 500ms until first telemetry received.
 * Only starts sending after first ping response is received.
 */
static void handle_telemetry_config(struct serial_state* state, uint64_t current_micros)
{
  if (state->telemetry_received) {
    // Already receiving telemetry - stop sending config
    return;
  }

  // Don't send telemetry config until we've received a ping response
  if (!state->ping_response_received) {
    return;
  }

  // On first call (last_time == 0), send immediately
  if (state->last_telemetry_request_time == 0) {
    LOG_INF("Requesting telemetry from Helios ICU");
    helios_send_telemetry_config(true, 100);
    state->last_telemetry_request_time = current_micros;
    return;
  }

  const uint64_t micros_since_request = current_micros - state->last_telemetry_request_time;
  if (micros_since_request < (TELEMETRY_REQUEST_INTERVAL_MS * 1000)) {
    return;
  }

  LOG_DBG("Retrying telemetry config (still waiting for first telemetry)");
  helios_send_telemetry_config(true, 100);
  state->last_telemetry_request_time = current_micros;
}

/**
 * Handle Ping
 *
 * Sends periodic ping every 10 seconds to maintain connection.
 */
static void handle_ping(struct serial_state* state, uint64_t current_micros)
{
  // On first call (last_time == 0), send immediately
  if (state->last_ping_time == 0) {
    LOG_INF("Starting periodic pings to Helios ICU (every %d seconds)", PING_INTERVAL_MS / 1000);
    helios_send_ping();
    state->last_ping_time = current_micros;
    return;
  }

  const uint64_t micros_since_ping = current_micros - state->last_ping_time;
  if (micros_since_ping < (PING_INTERVAL_MS * 1000)) {
    return;
  }

  LOG_DBG("Sending periodic ping to Helios");
  helios_send_ping();
  state->last_ping_time = current_micros;
}

//////////////////////////////////////////////////////////////
// Zbus Integration
//////////////////////////////////////////////////////////////

/**
 * Zbus Listener - Handles Helios state commands from shell/UI
 *
 * Listens to helios_state_command_chan and forwards commands
 * to Helios ICU via serial protocol.
 */
static void serial_handler_listener_cb(const struct zbus_channel* chan)
{
  // Only handle helios_state_command_chan
  if (chan == &helios_state_command_chan) {
    // In listener callback, channel is already locked - use const_msg
    const fusain_state_command_msg_t* cmd = zbus_chan_const_msg(chan);

    LOG_INF("Received state command: mode=%u, argument=%d", cmd->mode, cmd->argument);

    // Send STATE_COMMAND to Helios via serial protocol
    helios_set_mode(cmd->mode, cmd->argument);
  }
}

ZBUS_LISTENER_DEFINE(serial_handler_listener, serial_handler_listener_cb);
