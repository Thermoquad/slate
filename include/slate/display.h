// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef SLATE_DISPLAY_H
#define SLATE_DISPLAY_H

#include <zephyr/zbus/zbus.h>

/**
 * Display thread entry point
 */
int display_thread(void);

/**
 * Zbus callback for raw Fusain packet processing
 *
 * Called by Zbus when a raw packet is received. Updates the display's
 * appliance cache with decoded telemetry data.
 */
void display_raw_rx_callback(const struct zbus_channel* chan);

#endif
