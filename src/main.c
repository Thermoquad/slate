/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2025 Kaz Walker, Thermoquad
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <slate/serial_master.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

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
K_THREAD_DEFINE(serial_tx_id, CONFIG_MAIN_STACK_SIZE, serial_tx_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(serial_rx_id, CONFIG_MAIN_STACK_SIZE, serial_rx_thread, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
  LOG_INF("Slate controller starting");

  // Initialize serial communication with Helios
  int ret = serial_master_init();
  if (ret < 0) {
    LOG_ERR("Failed to initialize serial master: %d", ret);
    return ret;
  }

  LOG_INF("Slate initialized successfully");
  LOG_INF("Serial threads started - sending pings to Helios");

  while (true) {
    k_sleep(K_FOREVER);
  };

  return 0;
}
