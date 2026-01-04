/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2025 Kaz Walker, Thermoquad
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include <slate/serial_handler.h>
#include <slate/zbus.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

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
    helios_state_command_msg_t,
    NULL, // No validator
    NULL, // No user data
    ZBUS_OBSERVERS(serial_handler_listener), // Serial handler subscribes
    ZBUS_MSG_INIT(.mode = HELIOS_MODE_IDLE, .parameter = 0) // Initialize to idle
);

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
K_THREAD_DEFINE(serial_id, CONFIG_MAIN_STACK_SIZE, serial_thread, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
  LOG_INF("Slate controller starting");
  return 0;
}
