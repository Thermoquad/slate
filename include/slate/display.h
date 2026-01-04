// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef SLATE_DISPLAY_H
#define SLATE_DISPLAY_H

#include <zephyr/zbus/zbus.h>

int display_thread(void);
void display_telemetry_callback(const struct zbus_channel *chan);

#endif
