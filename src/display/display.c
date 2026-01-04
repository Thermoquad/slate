// SPDX-License-Identifier: GPL-2.0-or-later
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <lvgl.h>
#include <string.h>

#include <slate/zbus.h>

LOG_MODULE_REGISTER(display);

// Configuration
#define LOOP_SLEEP_MS 10
#define DISPLAY_UPDATE_INTERVAL_MS 200
#define PUB_TIMEOUT K_MSEC(10)

// LVGL objects
static lv_obj_t *label_state;
static lv_obj_t *label_temp;
static lv_obj_t *label_rpm;

// Shared telemetry data
static helios_telemetry_msg_t shared_telemetry = {
	.valid = false
};
K_MUTEX_DEFINE(telemetry_mutex);

// Zbus listener callback
void display_telemetry_callback(const struct zbus_channel *chan)
{
	const helios_telemetry_msg_t *msg = zbus_chan_const_msg(chan);

	k_mutex_lock(&telemetry_mutex, K_FOREVER);
	// Copy field by field to ensure atomicity
	shared_telemetry.state = msg->state;
	shared_telemetry.error = msg->error;
	shared_telemetry.temperature = msg->temperature;
	shared_telemetry.motor_rpm = msg->motor_rpm;
	shared_telemetry.motor_target_rpm = msg->motor_target_rpm;
	shared_telemetry.valid = msg->valid;
	k_mutex_unlock(&telemetry_mutex);
}



static void create_home_screen(void)
{
	lv_obj_t *screen = lv_obj_create(NULL);
	lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
	lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

	// Create labels for the 3-line display
	label_state = lv_label_create(screen);
	lv_obj_set_style_text_color(label_state, lv_color_white(), 0);
	lv_obj_set_style_text_font(label_state, &lv_font_unscii_8, 0);
	lv_obj_align(label_state, LV_ALIGN_TOP_LEFT, 0, 0);
	lv_label_set_text(label_state, "HELIOS: INIT");

	label_temp = lv_label_create(screen);
	lv_obj_set_style_text_color(label_temp, lv_color_white(), 0);
	lv_obj_set_style_text_font(label_temp, &lv_font_unscii_8, 0);
	lv_obj_align(label_temp, LV_ALIGN_TOP_LEFT, 0, 16);
	lv_label_set_text(label_temp, "Temp: ---.-C");

	label_rpm = lv_label_create(screen);
	lv_obj_set_style_text_color(label_rpm, lv_color_white(), 0);
	lv_obj_set_style_text_font(label_rpm, &lv_font_unscii_8, 0);
	lv_obj_align(label_rpm, LV_ALIGN_TOP_LEFT, 0, 32);
	lv_label_set_text(label_rpm, "RPM: ----/----");

	lv_screen_load(screen);
}

static void update_display(uint64_t current_micros)
{
	static char state_buf[32];
	static char temp_buf[32];
	static char rpm_buf[32];
	helios_telemetry_msg_t t;

	// Read from mutex-protected shared variable
	k_mutex_lock(&telemetry_mutex, K_FOREVER);
	// Copy field by field to ensure atomicity
	t.state = shared_telemetry.state;
	t.error = shared_telemetry.error;
	t.temperature = shared_telemetry.temperature;
	t.motor_rpm = shared_telemetry.motor_rpm;
	t.motor_target_rpm = shared_telemetry.motor_target_rpm;
	t.valid = shared_telemetry.valid;
	k_mutex_unlock(&telemetry_mutex);

	if (t.valid && t.state <= HELIOS_STATE_E_STOP) {
		snprintf(state_buf, sizeof(state_buf), "HELIOS: %s", helios_state_names[t.state]);
		snprintf(temp_buf, sizeof(temp_buf), "Temp: %.1fC", t.temperature);
		snprintf(rpm_buf, sizeof(rpm_buf), "RPM: %d/%d", t.motor_rpm, t.motor_target_rpm);

		LOG_DBG("Display: state=%d temp=%.1f rpm=%d/%d | \"%s\" \"%s\" \"%s\"",
			t.state, t.temperature, t.motor_rpm, t.motor_target_rpm,
			state_buf, temp_buf, rpm_buf);

		lv_label_set_text(label_state, state_buf);
		lv_label_set_text(label_temp, temp_buf);
		lv_label_set_text(label_rpm, rpm_buf);
	} else {
		lv_label_set_text(label_state, "HELIOS: INIT");
		lv_label_set_text(label_temp, "Temp: ---.-C");
		lv_label_set_text(label_rpm, "RPM: ----/----");
	}
}

int display_thread(void)
{
	LOG_INF("Display thread started");

	const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (!device_is_ready(display_dev)) {
		LOG_ERR("Display device not ready");
		return -1;
	}

	LOG_INF("Display device ready: %s", display_dev->name);

	// Get display capabilities
	struct display_capabilities caps;
	display_get_capabilities(display_dev, &caps);
	LOG_INF("Display: %dx%d, %d bpp", caps.x_resolution, caps.y_resolution,
		caps.current_pixel_format);

	// Set display brightness/contrast
	display_set_brightness(display_dev, 255);
	display_set_contrast(display_dev, 255);

	// Turn on display (disable blanking)
	display_blanking_off(display_dev);

	// Fill display with test pattern
	struct display_buffer_descriptor desc = {
		.buf_size = 128 * 8,  // 128 pixels wide, 8 pages (64 pixels / 8)
		.width = 128,
		.height = 64,
		.pitch = 128,
	};

	uint8_t test_buf[128 * 8];
	memset(test_buf, 0xFF, sizeof(test_buf));  // All pixels on

	int ret = display_write(display_dev, 0, 0, &desc, test_buf);
	LOG_INF("Display write returned: %d", ret);

	k_sleep(K_MSEC(2000));  // Show test pattern for 2 seconds

	memset(test_buf, 0x00, sizeof(test_buf));  // Clear
	display_write(display_dev, 0, 0, &desc, test_buf);

	// Wait for LVGL to fully initialize
	k_sleep(K_MSEC(100));

	// Create UI
	lv_lock();
	create_home_screen();
	lv_timer_handler();
	lv_unlock();

	LOG_INF("Display initialized");

	uint64_t last_update_time = 0;

	while (1) {
		const uint64_t current_micros = k_cyc_to_us_floor64(k_cycle_get_64());
		const uint64_t micros_since_update = current_micros - last_update_time;

		if (micros_since_update >= (DISPLAY_UPDATE_INTERVAL_MS * 1000)) {
			lv_lock();
			update_display(current_micros);
			lv_refr_now(NULL); // Force immediate complete refresh
			lv_unlock();
			last_update_time = current_micros;
		}

		k_sleep(K_MSEC(LOOP_SLEEP_MS));
	}

	return 0;
}
