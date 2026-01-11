// SPDX-License-Identifier: GPL-2.0-or-later
#include <lvgl.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include <slate/zbus.h>

LOG_MODULE_REGISTER(display);

// State names for display
static const char* const fusain_state_names[] = {
  "INIT",    // FUSAIN_STATE_INITIALIZING
  "IDLE",    // FUSAIN_STATE_IDLE
  "BLOWING", // FUSAIN_STATE_BLOWING
  "PREHEAT", // FUSAIN_STATE_PREHEAT
  "PRE_ST2", // FUSAIN_STATE_PREHEAT_STAGE_2
  "HEATING", // FUSAIN_STATE_HEATING
  "COOLING", // FUSAIN_STATE_COOLING
  "ERROR",   // FUSAIN_STATE_ERROR
  "E_STOP",  // FUSAIN_STATE_E_STOP
};

// Custom icon font (1bpp, Font Awesome icons)
LV_FONT_DECLARE(icons_font);
#define ICON_FAN "\xEF\xA1\xA3" // UTF-8 encoding of U+F863 (fan)
#define ICON_TEMP "\xEF\x9D\xA9" // UTF-8 encoding of U+F769 (thermometer)
#define ICON_PUMP "\xEF\x94\xAF" // UTF-8 encoding of U+F52F (gas pump)

// Configuration
#define DISPLAY_UPDATE_INTERVAL_MS 200
#define PUB_TIMEOUT K_MSEC(10)

// Shared telemetry data
static helios_telemetry_msg_t shared_telemetry = {
  .valid = false
};
K_MUTEX_DEFINE(telemetry_mutex);

// Fan icon for animation
static lv_obj_t* fan_icon_label = NULL;
static lv_anim_t fan_anim;
static bool fan_anim_running = false;

// Animation callback for fan icon rotation
static void fan_rotation_anim_cb(void* obj, int32_t value)
{
  lv_obj_set_style_transform_rotation(obj, value, 0);
}

// Zbus listener callback
void display_telemetry_callback(const struct zbus_channel* chan)
{
  const helios_telemetry_msg_t* msg = zbus_chan_const_msg(chan);

  k_mutex_lock(&telemetry_mutex, K_FOREVER);
  // Copy field by field to ensure atomicity
  shared_telemetry.state = msg->state;
  shared_telemetry.error = msg->error;
  shared_telemetry.temperature = msg->temperature;
  shared_telemetry.motor_rpm = msg->motor_rpm;
  shared_telemetry.motor_target_rpm = msg->motor_target_rpm;
  shared_telemetry.valid = msg->valid;
  k_mutex_unlock(&telemetry_mutex);

  LOG_DBG("Telemetry received: state=%d, temp=%.1f, rpm=%d",
      msg->state, msg->temperature, msg->motor_rpm);
}

static void create_home_screen(lv_obj_t** state_label, lv_obj_t** temp_label, lv_obj_t** rpm_label,
    lv_obj_t** pump_label)
{
  lv_obj_t* screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

  // Create labels for the 4-line display
  *state_label = lv_label_create(screen);
  lv_obj_set_style_text_color(*state_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(*state_label, &lv_font_unscii_8, 0);
  lv_obj_align(*state_label, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_label_set_text(*state_label, "INIT");

  // Temperature line: Thermometer icon (custom 1bpp font) + value (unscii_8)
  // Create thermometer icon label
  lv_obj_t* temp_icon_label = lv_label_create(screen);
  lv_obj_set_style_text_color(temp_icon_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(temp_icon_label, &icons_font, 0);
  lv_obj_align(temp_icon_label, LV_ALIGN_TOP_LEFT, 2, 16);
  lv_label_set_text(temp_icon_label, ICON_TEMP);

  // Create temperature value label (positioned after icon)
  *temp_label = lv_label_create(screen);
  lv_obj_set_style_text_color(*temp_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(*temp_label, &lv_font_unscii_8, 0);
  lv_obj_align(*temp_label, LV_ALIGN_TOP_LEFT, 14, 16);
  lv_label_set_text(*temp_label, " --.-C");

  // RPM line: Fan icon (custom 1bpp font) + numbers (unscii_8)
  // Create fan icon label
  fan_icon_label = lv_label_create(screen);
  lv_obj_set_style_text_color(fan_icon_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(fan_icon_label, &icons_font, 0);
  lv_obj_align(fan_icon_label, LV_ALIGN_TOP_LEFT, 0, 32);
  lv_label_set_text(fan_icon_label, ICON_FAN);

  // Set rotation pivot to center of icon and initial rotation
  lv_obj_set_style_transform_pivot_x(fan_icon_label, 6, 0); // Half of 12px width
  lv_obj_set_style_transform_pivot_y(fan_icon_label, 6, 0); // Half of 12px height
  lv_obj_set_style_transform_rotation(fan_icon_label, 450, 0); // Start at 45 degrees

  // Initialize fan icon rotation animation (started/stopped based on RPM)
  lv_anim_init(&fan_anim);
  lv_anim_set_var(&fan_anim, fan_icon_label);
  lv_anim_set_exec_cb(&fan_anim, fan_rotation_anim_cb);
  lv_anim_set_values(&fan_anim, 450, 4050); // 45 to 405 degrees (in decidegrees)
  lv_anim_set_time(&fan_anim, 1000); // 1 second per full rotation
  lv_anim_set_repeat_count(&fan_anim, LV_ANIM_REPEAT_INFINITE);
  // Don't start animation yet - will be started when RPM > 0

  // Create RPM numbers label (positioned after icon)
  *rpm_label = lv_label_create(screen);
  lv_obj_set_style_text_color(*rpm_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(*rpm_label, &lv_font_unscii_8, 0);
  lv_obj_align(*rpm_label, LV_ALIGN_TOP_LEFT, 14, 32);
  lv_label_set_text(*rpm_label, " ----");

  // Pump line: Gas pump icon (custom 1bpp font) + value (unscii_8)
  // Create pump icon label
  lv_obj_t* pump_icon_label = lv_label_create(screen);
  lv_obj_set_style_text_color(pump_icon_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(pump_icon_label, &icons_font, 0);
  lv_obj_align(pump_icon_label, LV_ALIGN_TOP_LEFT, 2, 48);
  lv_label_set_text(pump_icon_label, ICON_PUMP);

  // Create pump value label (positioned after icon)
  *pump_label = lv_label_create(screen);
  lv_obj_set_style_text_color(*pump_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(*pump_label, &lv_font_unscii_8, 0);
  lv_obj_align(*pump_label, LV_ALIGN_TOP_LEFT, 14, 48);
  lv_label_set_text(*pump_label, " 0.0Hz");

  lv_screen_load(screen);
}

static void update_display(lv_obj_t* state_label, lv_obj_t* temp_label, lv_obj_t* rpm_label,
    lv_obj_t* pump_label)
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

  if (t.valid && t.state <= FUSAIN_STATE_E_STOP) {
    // Clear buffers before formatting to prevent artifacts
    memset(state_buf, 0, sizeof(state_buf));
    memset(temp_buf, 0, sizeof(temp_buf));
    memset(rpm_buf, 0, sizeof(rpm_buf));

    snprintf(state_buf, sizeof(state_buf), "%s", fusain_state_names[t.state]);
    // Use leading space for separation from icon
    snprintf(temp_buf, sizeof(temp_buf), " %.1fC", t.temperature);
    snprintf(rpm_buf, sizeof(rpm_buf), " %4d", t.motor_rpm);

    LOG_DBG("Display: state=%d temp=%.1f rpm=%d/%d | \"%s\" \"%s\" \"%s\"",
        t.state, t.temperature, t.motor_rpm, t.motor_target_rpm,
        state_buf, temp_buf, rpm_buf);

    lv_label_set_text(state_label, state_buf);
    lv_label_set_text(temp_label, temp_buf);
    lv_label_set_text(rpm_label, rpm_buf);
    lv_label_set_text(pump_label, " 0.0Hz");

    // Control fan animation based on RPM
    if (t.motor_rpm > 0 && !fan_anim_running) {
      lv_anim_start(&fan_anim);
      fan_anim_running = true;
    } else if (t.motor_rpm == 0 && fan_anim_running) {
      lv_anim_delete(fan_icon_label, fan_rotation_anim_cb);
      lv_obj_set_style_transform_rotation(fan_icon_label, 450, 0); // Reset to 45 degrees
      fan_anim_running = false;
    }
  } else {
    lv_label_set_text(state_label, "INIT");
    lv_label_set_text(temp_label, " --.-C");
    lv_label_set_text(rpm_label, " ----");
    lv_label_set_text(pump_label, " 0.0Hz");

    // Stop fan animation if no valid data
    if (fan_anim_running) {
      lv_anim_delete(fan_icon_label, fan_rotation_anim_cb);
      lv_obj_set_style_transform_rotation(fan_icon_label, 450, 0); // Reset to 45 degrees
      fan_anim_running = false;
    }
  }
}

int display_thread(void)
{
  lv_obj_t* label_state;
  lv_obj_t* label_temp;
  lv_obj_t* label_rpm;
  lv_obj_t* label_pump;
  uint64_t last_update_time = 0;
  int ret;

  LOG_INF("Display thread started");

  const struct device* display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

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

  // Create UI
  lv_lock();
  create_home_screen(&label_state, &label_temp, &label_rpm, &label_pump);
  lv_timer_handler();
  lv_unlock();

  // Turn on display (disable blanking) after UI creation
  ret = display_blanking_off(display_dev);
  if (ret < 0 && ret != -ENOSYS) {
    LOG_ERR("Failed to turn blanking off (error %d)", ret);
    return -1;
  }

  LOG_INF("Display initialized");

  while (1) {
    const uint64_t current_micros = k_cyc_to_us_floor64(k_cycle_get_64());
    const uint64_t micros_since_update = current_micros - last_update_time;
    uint32_t sleep_ms;

    if (micros_since_update >= (DISPLAY_UPDATE_INTERVAL_MS * 1000)) {
      lv_lock();
      update_display(label_state, label_temp, label_rpm, label_pump);
      sleep_ms = lv_timer_handler();
      lv_unlock();
      last_update_time = current_micros;
    } else {
      lv_lock();
      sleep_ms = lv_timer_handler();
      lv_unlock();
    }

    k_sleep(K_MSEC(MIN(sleep_ms, 100)));
  }

  return 0;
}
