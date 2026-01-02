/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Slate Serial Master - Helios Protocol Implementation
 *
 * Implements master-side Helios serial protocol communication.
 * Sends commands and pings to Helios ICU, receives telemetry responses.
 */

#include <helios_serial/helios_serial.h>
#include <slate/serial_master.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(slate_serial_handler);

/* UART Device */
static const struct device *uart_dev;

/* Decoder State */
static uint8_t decoder_state = 0;
static uint8_t decode_buffer[HELIOS_MAX_PACKET_SIZE];
static size_t decode_buffer_index = 0;
static bool decode_escape_next = false;

/* TX Buffer and State */
static uint8_t tx_buffer[HELIOS_MAX_PACKET_SIZE * 2]; // 2x for stuffing overhead
static size_t tx_index = 0;
static size_t tx_length = 0;
static bool tx_in_progress = false;
K_MUTEX_DEFINE(tx_mutex); // Protects TX buffer and state

/* RX Packet Queue - ISR pushes, thread pops */
K_MSGQ_DEFINE(rx_packet_queue, sizeof(helios_packet_t), 8, 4);

/* Helios State (from telemetry) */
static helios_state_t helios_state = HELIOS_STATE_INITIALIZING;
static helios_error_t helios_error = HELIOS_ERROR_NONE;
static uint32_t helios_uptime_ms = 0;

/* Forward Declarations */
static void process_packet(const helios_packet_t *packet);
static void send_packet(const helios_packet_t *packet);
static void uart_isr(const struct device *dev, void *user_data);

/* UART ISR - Handles both RX and TX interrupts */
static void uart_isr(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);

  uart_irq_update(dev);

  // Handle RX
  if (uart_irq_rx_ready(dev)) {
    uint8_t byte;
    while (uart_fifo_read(dev, &byte, 1) == 1) {
      helios_packet_t packet;
      helios_decode_result_t result =
          helios_decode_byte(byte, &packet, &decoder_state, decode_buffer,
                             &decode_buffer_index, &decode_escape_next);

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
        helios_reset_decoder(&decoder_state, &decode_buffer_index,
                             &decode_escape_next);
      }
    }
  }

  // Handle TX
  if (uart_irq_tx_ready(dev)) {
    if (tx_in_progress && tx_index < tx_length) {
      // Fill FIFO with as much data as possible
      size_t remaining = tx_length - tx_index;
      size_t sent = uart_fifo_fill(dev, &tx_buffer[tx_index], remaining);
      tx_index += sent;

      // Check if transmission complete
      if (tx_index >= tx_length) {
        tx_in_progress = false;
        uart_irq_tx_disable(dev);
        LOG_DBG("TX complete: %zu bytes sent", tx_length);
      }
    } else {
      // No data to send, disable TX interrupt
      uart_irq_tx_disable(dev);
    }
  }
}

/* Process Received Packet (called from RX thread) */
static void process_packet(const helios_packet_t *packet) {
  LOG_DBG("Processing packet: type=0x%02X, length=%u", packet->msg_type,
          packet->length);

  switch (packet->msg_type) {
  case HELIOS_MSG_PING_RESPONSE: {
    helios_data_ping_response_t *response =
        (helios_data_ping_response_t *)packet->payload;
    helios_uptime_ms = response->uptime_ms;
    LOG_DBG("Ping response received (uptime=%u ms)", helios_uptime_ms);
    break;
  }

  case HELIOS_MSG_STATE_DATA: {
    helios_data_state_t *data = (helios_data_state_t *)packet->payload;
    helios_state = data->state;
    helios_error = data->error;
    LOG_DBG("State data: state=%u, error=%u", helios_state, helios_error);
    break;
  }

  case HELIOS_MSG_TELEMETRY_BUNDLE: {
    helios_data_telemetry_bundle_t *bundle =
        (helios_data_telemetry_bundle_t *)packet->payload;
    helios_state = bundle->state;
    helios_error = bundle->error;

    LOG_DBG("Telemetry: state=%u, error=%u, motors=%u, temps=%u", bundle->state,
            bundle->error, bundle->motor_count, bundle->temp_count);

    // Parse variable-length data
    const uint8_t *ptr =
        packet->payload + sizeof(helios_data_telemetry_bundle_t);

    // Read motor data
    for (int i = 0; i < bundle->motor_count; i++) {
      helios_telemetry_motor_t *motor = (helios_telemetry_motor_t *)ptr;
      LOG_DBG("Motor %d: RPM=%u, target=%u, PWM=%u%%", i, motor->rpm,
              motor->target_rpm, motor->pwm_duty);
      ptr += sizeof(helios_telemetry_motor_t);
    }

    // Read temperature data
    for (int i = 0; i < bundle->temp_count; i++) {
      helios_telemetry_temperature_t *temp =
          (helios_telemetry_temperature_t *)ptr;
      LOG_DBG("Temp %d: %.1f°C", i, (double)temp->temperature);
      ptr += sizeof(helios_telemetry_temperature_t);
    }
    break;
  }

  case HELIOS_MSG_ERROR_INVALID_CRC: {
    helios_error_invalid_crc_t *error =
        (helios_error_invalid_crc_t *)packet->payload;
    LOG_ERR("Helios reported CRC error: calculated=0x%04X, received=0x%04X",
            error->calculated_crc, error->received_crc);
    break;
  }

  case HELIOS_MSG_ERROR_INVALID_COMMAND: {
    helios_error_invalid_command_t *error =
        (helios_error_invalid_command_t *)packet->payload;
    LOG_ERR("Helios reported invalid command: 0x%02X", error->invalid_command);
    break;
  }

  case HELIOS_MSG_ERROR_INVALID_LENGTH: {
    helios_error_invalid_length_t *error =
        (helios_error_invalid_length_t *)packet->payload;
    LOG_ERR("Helios reported invalid length: received=%u, expected=%u",
            error->received_length, error->expected_length);
    break;
  }

  default:
    LOG_WRN("Unknown message type: 0x%02X", packet->msg_type);
    break;
  }
}

/* Send Packet via UART using interrupt-driven TX */
static void send_packet(const helios_packet_t *packet) {
  // Lock to prevent concurrent transmission attempts
  k_mutex_lock(&tx_mutex, K_FOREVER);

  // Wait for any previous transmission to complete
  while (tx_in_progress) {
    k_yield();
  }

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
  tx_in_progress = true;

  LOG_DBG("Starting TX: type=0x%02X, %zu bytes", packet->msg_type, tx_length);

  // Enable TX interrupt - this will trigger ISR to start sending
  uart_irq_tx_enable(uart_dev);

  k_mutex_unlock(&tx_mutex);
}

/* Public API - Send Ping Request */
void serial_master_send_ping(void) {
  helios_packet_t packet;
  helios_create_ping_request(&packet);
  LOG_DBG(">>> SENDING PING REQUEST to Helios");
  send_packet(&packet);
}

/* Public API - Send Set Mode Command */
void serial_master_set_mode(helios_mode_t mode, uint32_t parameter) {
  helios_packet_t packet;
  helios_create_set_mode(&packet, mode, parameter);
  send_packet(&packet);
  LOG_INF("Set mode: mode=%u, parameter=%u", mode, parameter);
}

/* Public API - Get Helios State */
void serial_master_get_state(helios_state_t *state, helios_error_t *error) {
  if (state) {
    *state = helios_state;
  }
  if (error) {
    *error = helios_error;
  }
}

/* Initialize Serial Master */
int serial_master_init(void) {
  // Get UART device
  uart_dev = DEVICE_DT_GET(DT_ALIAS(helios_uart));
  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not ready");
    return -1;
  }

  LOG_DBG("UART device ready: %s", uart_dev->name);

  // Initialize decoder BEFORE enabling interrupts
  helios_reset_decoder(&decoder_state, &decode_buffer_index,
                       &decode_escape_next);
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

/* TX Thread - Sends periodic pings */
void serial_tx_thread(void) {
  LOG_DBG("Serial TX thread started");

  while (1) {
    // Send ping every 10 seconds (Helios timeout is 30s)
    k_sleep(K_SECONDS(10));
    serial_master_send_ping();
  }
}

/* RX Thread - Dequeues and processes received packets */
void serial_rx_thread(void) {
  LOG_DBG("Serial RX thread started");

  while (1) {
    helios_packet_t packet;

    // Wait for packet from queue (blocking)
    int ret = k_msgq_get(&rx_packet_queue, &packet, K_FOREVER);
    if (ret == 0) {
      // Process packet in thread context (safe for logging, etc.)
      process_packet(&packet);
    }
  }
}
