/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Slate Serial Master - Helios Protocol Implementation
 *
 * Implements master-side Helios serial protocol communication.
 * Sends commands and pings to Helios ICU, receives telemetry responses.
 */

#include <fusain/fusain.h>
#include <slate/serial_handler.h>
#include <slate/zbus.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

LOG_MODULE_REGISTER(slate_serial_handler);

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

#define LOOP_SLEEP_MS 1
#define TELEMETRY_REQUEST_INTERVAL_MS 500
#define PING_INTERVAL_MS 10000
#define TELEMETRY_TIMEOUT_MS 30000
#define EMERGENCY_STOP_RETRANSMIT_MS 250
#define PUB_TIMEOUT K_MSEC(10)

//////////////////////////////////////////////////////////////
// Serial State
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
  helios_state_t helios_state;
  helios_error_t helios_error;
  uint32_t helios_uptime_ms;

  // Ping tracking
  uint64_t last_ping_time;
};

//////////////////////////////////////////////////////////////
// Static Variables
//////////////////////////////////////////////////////////////

/* UART Device */
static const struct device* uart_dev;

/* Decoder State */
static helios_decoder_t decoder;

/* TX Buffer and State */
static uint8_t tx_buffer[HELIOS_MAX_PACKET_SIZE * 2]; // 2x for stuffing overhead
static size_t tx_index = 0;
static size_t tx_length = 0;
K_MUTEX_DEFINE(tx_mutex); // Protects TX buffer and state

/* TX Packet Queue - API pushes, thread pops and transmits */
K_MSGQ_DEFINE(tx_packet_queue, sizeof(helios_packet_t), 8, 4);

/* RX Packet Queue - ISR pushes, thread pops */
K_MSGQ_DEFINE(rx_packet_queue, sizeof(helios_packet_t), 8, 4);

/* Serial State */
static struct serial_state serial_state;

//////////////////////////////////////////////////////////////
// Forward Declarations
//////////////////////////////////////////////////////////////

static void process_packet(const helios_packet_t* packet, struct serial_state* state,
    uint64_t current_micros);
static void send_packet(const helios_packet_t* packet);
static void fill_transmit_buffer(const helios_packet_t* packet);
static void uart_isr(const struct device* dev, void* user_data);
static void process_rx_packets(struct serial_state* state, uint64_t current_micros);
static void process_tx_queue(void);
static void handle_emergency_stop(struct serial_state* state, uint64_t current_micros);
static void check_telemetry_timeout(struct serial_state* state, uint64_t current_micros);
static void handle_telemetry_config(struct serial_state* state, uint64_t current_micros);
static void handle_ping(struct serial_state* state, uint64_t current_micros);

//////////////////////////////////////////////////////////////
// Serial Thread
//////////////////////////////////////////////////////////////

/**
 * Serial Thread - Handles both TX and RX operations
 *
 * Main thread for serial communication with Helios ICU.
 * Processes received packets, transmits queued packets,
 * and handles periodic tasks (ping, telemetry config).
 */
int serial_thread(void)
{
  LOG_DBG("Serial thread started");

  // Initialize serial communication with Helios
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
  serial_state.helios_state = HELIOS_STATE_INITIALIZING;
  serial_state.helios_error = HELIOS_ERROR_NONE;
  serial_state.helios_uptime_ms = 0;
  serial_state.last_ping_time = 0;

  while (1) {
    const uint64_t current_micros = k_cyc_to_us_floor64(k_cycle_get_64());

    // Process all pending RX packets
    process_rx_packets(&serial_state, current_micros);

    // Handle emergency stop (priority)
    handle_emergency_stop(&serial_state, current_micros);

    // Check telemetry timeout
    check_telemetry_timeout(&serial_state, current_micros);

    // Handle telemetry config requests
    handle_telemetry_config(&serial_state, current_micros);

    // Handle periodic pings
    handle_ping(&serial_state, current_micros);

    // Process one pending TX packet if available
    process_tx_queue();

    k_sleep(K_MSEC(LOOP_SLEEP_MS));
  }
}

//////////////////////////////////////////////////////////////
// UART Interrupt Service Routine
//////////////////////////////////////////////////////////////

/**
 * UART ISR - Handles both RX and TX interrupts
 *
 * RX: Decodes incoming bytes and queues complete packets
 * TX: Fills UART FIFO from buffer until transmission complete
 */
static void uart_isr(const struct device* dev, void* user_data)
{
  ARG_UNUSED(user_data);

  uart_irq_update(dev);

  // Handle RX
  if (uart_irq_rx_ready(dev)) {
    uint8_t byte;
    while (uart_fifo_read(dev, &byte, 1) == 1) {
      helios_packet_t packet;
      helios_decode_result_t result = helios_decode_byte(byte, &packet, &decoder);

      if (result == HELIOS_DECODE_OK) {
        // Packet complete - queue for processing in thread context
        int ret = k_msgq_put(&rx_packet_queue, &packet, K_NO_WAIT);
        if (ret != 0) {
          // Queue full - drop packet and log error
          LOG_ERR("RX queue full, dropping packet type 0x%02X", packet.msg_type);
        }
      } else if (result != HELIOS_DECODE_INCOMPLETE) {
        // Decode error - reset decoder and continue
        LOG_WRN("Decode error: %d", result);
        helios_reset_decoder(&decoder);
      }
    }
  }

  // Handle TX
  if (uart_irq_tx_ready(dev)) {
    if (tx_index < tx_length) {
      // Fill FIFO with as much data as possible
      size_t remaining = tx_length - tx_index;
      size_t sent = uart_fifo_fill(dev, &tx_buffer[tx_index], remaining);
      tx_index += sent;

      // Check if transmission complete
      if (tx_index >= tx_length) {
        uart_irq_tx_disable(dev);
        LOG_DBG("TX complete: %zu bytes sent", tx_length);
      }
    } else {
      // No data to send, disable TX interrupt
      uart_irq_tx_disable(dev);
    }
  }
}

//////////////////////////////////////////////////////////////
// Packet Processing
//////////////////////////////////////////////////////////////

/**
 * Process Received Packet
 *
 * Called from serial thread to process packets queued by UART ISR.
 * Handles ping responses, telemetry, state data, and error messages.
 */
static void process_packet(const helios_packet_t* packet, struct serial_state* state,
    uint64_t current_micros)
{
  LOG_DBG("Processing packet: type=0x%02X, length=%u", packet->msg_type,
      packet->length);

  switch (packet->msg_type) {
  case HELIOS_MSG_PING_RESPONSE: {
    helios_data_ping_response_t* response = (helios_data_ping_response_t*)packet->payload;
    state->helios_uptime_ms = response->uptime_ms;
    LOG_DBG("Ping response received (uptime=%u ms)", state->helios_uptime_ms);
    break;
  }

  case HELIOS_MSG_STATE_DATA: {
    helios_data_state_t* data = (helios_data_state_t*)packet->payload;
    state->helios_state = data->state;
    state->helios_error = data->error;

    // Mark that we've received state data (counts as telemetry)
    if (!state->telemetry_received) {
      LOG_INF("State data received - telemetry enabled");
      state->telemetry_received = true;
    }
    state->last_telemetry_time = current_micros;

    LOG_DBG("State data: state=%u, error=%u", state->helios_state, state->helios_error);
    break;
  }

  case HELIOS_MSG_TELEMETRY_BUNDLE: {
    helios_data_telemetry_bundle_t* bundle = (helios_data_telemetry_bundle_t*)packet->payload;

    state->helios_state = bundle->state;
    state->helios_error = bundle->error;

    // Mark that we've received telemetry and update timestamp
    if (!state->telemetry_received) {
      LOG_INF("Telemetry successfully enabled - receiving bundles");
      state->telemetry_received = true;
    }
    state->last_telemetry_time = current_micros;

    // Protocol v1.3: Check for emergency stop confirmation
    if (state->emergency_stop_active && bundle->state == HELIOS_STATE_E_STOP) {
      LOG_INF("Emergency stop confirmed by Helios - stopping retransmission");
      state->emergency_stop_confirmed = true;
      state->emergency_stop_active = false;
    }

    LOG_DBG("Telemetry: state=%u, error=%u, motors=%u, temps=%u", bundle->state,
        bundle->error, bundle->motor_count, bundle->temp_count);

    // Validate packet counts to prevent buffer overruns
    if (bundle->motor_count > 10 || bundle->temp_count > 10) {
      LOG_ERR("Corrupted telemetry packet: motor_count=%u, temp_count=%u (max 10 each)",
              bundle->motor_count, bundle->temp_count);
      LOG_ERR("Rejecting corrupted packet to prevent buffer overrun");
      break;
    }

    // Validate packet length matches expected data size
    size_t expected_payload_size = sizeof(helios_data_telemetry_bundle_t)
        + (bundle->motor_count * sizeof(helios_telemetry_motor_t))
        + (bundle->temp_count * sizeof(helios_telemetry_temperature_t));

    if (packet->length != expected_payload_size) {
      LOG_ERR("Telemetry packet length mismatch: received=%u, expected=%zu",
              packet->length, expected_payload_size);
      LOG_ERR("Rejecting corrupted packet (motor_count=%u, temp_count=%u)",
              bundle->motor_count, bundle->temp_count);
      break;
    }

    // Parse variable-length data
    const uint8_t* ptr = packet->payload + sizeof(helios_data_telemetry_bundle_t);

    // Prepare telemetry message for Zbus
    helios_telemetry_msg_t telemetry_msg = {
      .state = bundle->state,
      .error = bundle->error,
      .temperature = 0.0,
      .motor_rpm = 0,
      .motor_target_rpm = 0,
      .valid = true
    };

    // Read motor data
    for (int i = 0; i < bundle->motor_count; i++) {
      helios_telemetry_motor_t* motor = (helios_telemetry_motor_t*)ptr;
      uint32_t pwm_percent = (motor->pwm_period > 0)
          ? (motor->pwm_duty * 100) / motor->pwm_period
          : 0;
      LOG_DBG("Motor %d: RPM=%u, target=%u, PWM=%u%% (%u/%u ns)", i, motor->rpm,
          motor->target_rpm, pwm_percent, motor->pwm_duty, motor->pwm_period);

      // Store first motor data in telemetry message
      if (i == 0) {
        telemetry_msg.motor_rpm = motor->rpm;
        telemetry_msg.motor_target_rpm = motor->target_rpm;

        // Log suspicious high values
        if (motor->rpm > 6000 || motor->target_rpm > 6000) {
          LOG_WRN("Suspicious high RPM values: rpm=%d target=%d",
                  motor->rpm, motor->target_rpm);
        }

        LOG_DBG("Parsed motor data: rpm=%d target=%d", motor->rpm, motor->target_rpm);
      }

      ptr += sizeof(helios_telemetry_motor_t);
    }

    // Read temperature data
    for (int i = 0; i < bundle->temp_count; i++) {
      helios_telemetry_temperature_t* temp = (helios_telemetry_temperature_t*)ptr;
      LOG_DBG("Temp %d: %.1f°C", i, (double)temp->temperature);

      // Store first temperature reading in telemetry message
      if (i == 0) {
        telemetry_msg.temperature = (double)temp->temperature;
        LOG_DBG("Parsed temperature: %.1f", (double)temp->temperature);
      }

      ptr += sizeof(helios_telemetry_temperature_t);
    }

    // Publish to Zbus
    int ret = zbus_chan_pub(&helios_telemetry_chan, &telemetry_msg, PUB_TIMEOUT);
    if (ret != 0) {
      LOG_WRN("Failed to publish telemetry to Zbus: %d", ret);
    }

    break;
  }

  case HELIOS_MSG_ERROR_INVALID_CRC: {
    helios_error_invalid_crc_t* error = (helios_error_invalid_crc_t*)packet->payload;
    LOG_ERR("Helios reported CRC error: calculated=0x%04X, received=0x%04X",
        error->calculated_crc, error->received_crc);
    break;
  }

  case HELIOS_MSG_ERROR_INVALID_COMMAND: {
    helios_error_invalid_command_t* error = (helios_error_invalid_command_t*)packet->payload;
    LOG_ERR("Helios reported invalid command: 0x%02X", error->invalid_command);
    break;
  }

  case HELIOS_MSG_ERROR_INVALID_LENGTH: {
    helios_error_invalid_length_t* error = (helios_error_invalid_length_t*)packet->payload;
    LOG_ERR("Helios reported invalid length: received=%u, expected=%u",
        error->received_length, error->expected_length);
    break;
  }

  default:
    LOG_WRN("Unknown message type: 0x%02X", packet->msg_type);
    break;
  }
}

/**
 * Transmit Packet - Actual UART transmission
 *
 * Encodes packet and triggers UART ISR to send it.
 * Called from serial thread.
 */
static void fill_transmit_buffer(const helios_packet_t* packet)
{
  // Lock to prevent concurrent transmission attempts
  k_mutex_lock(&tx_mutex, K_FOREVER);

  // Encode packet to buffer
  int len = helios_encode_packet(packet, tx_buffer, sizeof(tx_buffer));
  if (len < 0) {
    LOG_ERR("Encoding failed: %d", len);
    k_mutex_unlock(&tx_mutex);
    return;
  }

  // Prepare TX state
  tx_index = 0;
  tx_length = (size_t)len;

  LOG_DBG("Starting TX: type=0x%02X, %zu bytes", packet->msg_type, tx_length);

  // Enable TX interrupt - this will trigger ISR to start sending
  uart_irq_tx_enable(uart_dev);

  k_mutex_unlock(&tx_mutex);
}

/**
 * Queue Packet for Transmission
 *
 * Queues packet for transmission by serial thread.
 * Called from public API functions.
 */
static void send_packet(const helios_packet_t* packet)
{
  int ret = k_msgq_put(&tx_packet_queue, packet, K_NO_WAIT);
  if (ret != 0) {
    LOG_ERR("TX queue full, dropping packet type 0x%02X", packet->msg_type);
  }
}

//////////////////////////////////////////////////////////////
// Thread Helper Functions
//////////////////////////////////////////////////////////////

/**
 * Process RX Packets
 *
 * Drains RX packet queue and processes all pending packets.
 */
static void process_rx_packets(struct serial_state* state, uint64_t current_micros)
{
  helios_packet_t rx_packet;
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
  helios_packet_t tx_packet;
  if (k_msgq_get(&tx_packet_queue, &tx_packet, K_NO_WAIT) == 0) {
    fill_transmit_buffer(&tx_packet);
  }
}

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

  helios_packet_t packet;
  helios_create_emergency_stop(&packet);
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
 */
static void handle_telemetry_config(struct serial_state* state, uint64_t current_micros)
{
  if (state->telemetry_received) {
    return;
  }

  // On first call (last_time == 0), send immediately
  if (state->last_telemetry_request_time == 0) {
    LOG_DBG("Sending initial telemetry config");
    helios_send_telemetry_config(true, 100, 0);
    state->last_telemetry_request_time = current_micros;
    return;
  }

  const uint64_t micros_since_request = current_micros - state->last_telemetry_request_time;
  if (micros_since_request < (TELEMETRY_REQUEST_INTERVAL_MS * 1000)) {
    return;
  }

  LOG_DBG("Sending telemetry config (waiting for telemetry)");
  helios_send_telemetry_config(true, 100, 0);
  state->last_telemetry_request_time = current_micros;
}

/**
 * Handle Ping
 *
 * Sends periodic ping every 10 seconds to maintain connection.
 */
static void handle_ping(struct serial_state* state, uint64_t current_micros)
{
  if (!state->telemetry_received) {
    return; // Don't ping until telemetry is enabled
  }

  // On first call after telemetry enabled (last_time == 0), send immediately
  if (state->last_ping_time == 0) {
    LOG_DBG("Sending initial ping");
    helios_send_ping();
    state->last_ping_time = current_micros;
    return;
  }

  const uint64_t micros_since_ping = current_micros - state->last_ping_time;
  if (micros_since_ping < (PING_INTERVAL_MS * 1000)) {
    return;
  }

  helios_send_ping();
  state->last_ping_time = current_micros;
}

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
  helios_packet_t packet;
  helios_create_ping_request(&packet);
  LOG_DBG(">>> SENDING PING REQUEST to Helios");
  send_packet(&packet);
}

/**
 * Send Telemetry Config Command
 *
 * Configures telemetry broadcast settings on Helios ICU (protocol v1.2+).
 *
 * @param enabled Enable/disable telemetry broadcasts
 * @param interval_ms Telemetry broadcast interval (100-5000 ms)
 * @param mode Telemetry mode (0=bundled, 1=individual)
 */
void helios_send_telemetry_config(bool enabled, uint32_t interval_ms,
    uint32_t mode)
{
  helios_packet_t packet;
  helios_create_telemetry_config(&packet, enabled, interval_ms, mode);
  send_packet(&packet);
  LOG_INF("Telemetry config: enabled=%d, interval=%u ms, mode=%u", enabled,
      interval_ms, mode);
}

/**
 * Send Set Mode Command
 *
 * Sends a mode change command to Helios ICU.
 *
 * @param mode Operating mode (IDLE, FAN, HEAT, EMERGENCY)
 * @param parameter Mode-specific parameter (e.g., RPM for FAN, pump rate for HEAT)
 */
void helios_set_mode(helios_mode_t mode, uint32_t parameter)
{
  helios_packet_t packet;
  helios_create_set_mode(&packet, mode, parameter);
  send_packet(&packet);
  LOG_INF("Set mode: mode=%u, parameter=%u", mode, parameter);
}

/**
 * Send Emergency Stop Command
 *
 * Initiates emergency stop procedure (protocol v1.3).
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
void helios_get_state(helios_state_t* state, helios_error_t* error)
{
  if (state) {
    *state = serial_state.helios_state;
  }
  if (error) {
    *error = serial_state.helios_error;
  }
}


//////////////////////////////////////////////////////////////
// Initialization
//////////////////////////////////////////////////////////////

/**
 * Initialize Serial Master
 *
 * Initializes UART device, decoder, and interrupt handlers.
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

  // Initialize decoder BEFORE enabling interrupts
  helios_reset_decoder(&decoder);
  LOG_DBG("Decoder initialized");

  // Flush RX FIFO to discard any garbage bytes
  uint8_t discard;
  int flushed = 0;
  while (uart_fifo_read(uart_dev, &discard, 1) == 1) {
    flushed++;
  }
  if (flushed > 0) {
    LOG_DBG("Flushed %d bytes from RX FIFO", flushed);
  }

  // Configure UART for interrupt-driven TX/RX
  uart_irq_callback_set(uart_dev, uart_isr);
  LOG_DBG("UART ISR registered");

  // Enable RX interrupt (after decoder initialized and FIFO flushed)
  uart_irq_rx_enable(uart_dev);
  LOG_DBG("UART RX interrupt enabled");

  // TX interrupt will be enabled on-demand during transmission

  LOG_INF("Serial master initialized on %s", uart_dev->name);

  return 0;
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
    const helios_state_command_msg_t* cmd = zbus_chan_const_msg(chan);

    LOG_INF("Received state command: mode=%u, parameter=%u", cmd->mode, cmd->parameter);

    // Send SET_MODE command to Helios via serial protocol
    helios_set_mode(cmd->mode, cmd->parameter);
  }
}

ZBUS_LISTENER_DEFINE(serial_handler_listener, serial_handler_listener_cb);
