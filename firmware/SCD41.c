#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "scd4x_i2c.h"
#include "sensirion_i2c_hal.h"
#include "ws2812.pio.h"
#include <math.h>
#include <stdio.h>


// WS2812 Configuration
#define WS2812_PIN 16
#define NUM_PIXELS 25
#define IS_RGBW false

// LED Target Indices (Corners + Center)
const int TARGET_PIXELS[] = {0, 4, 12, 20, 24};
const int NUM_TARGETS = 5;

// Global PIO variables
PIO pio_ws2812 = pio0;
int sm_ws2812 = 0;

static inline void put_pixel(uint32_t pixel_grb) {
  pio_sm_put_blocking(pio_ws2812, sm_ws2812, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
  // RGB Order
  return ((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b);
}

// Function to set the 5 target LEDs
void set_status_leds(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t color = urgb_u32(r, g, b);
  uint32_t off = urgb_u32(0, 0, 0);

  for (int i = 0; i < NUM_PIXELS; i++) {
    bool is_target = false;
    for (int j = 0; j < NUM_TARGETS; j++) {
      if (i == TARGET_PIXELS[j]) {
        is_target = true;
        break;
      }
    }

    if (is_target) {
      put_pixel(color);
    } else {
      put_pixel(off);
    }
  }
}

int main() {
  stdio_init_all();
  sleep_ms(3000);

  printf("\n=== SCD41 + WS2812 Smart Demo v2 ===\n");

  // 1. Initialize WS2812 PIO
  printf("[Init] Setting up WS2812 on GP%d...\n", WS2812_PIN);
  uint offset = pio_add_program(pio_ws2812, &ws2812_program);
  ws2812_program_init(pio_ws2812, sm_ws2812, offset, WS2812_PIN, 800000,
                      IS_RGBW);

  // Test LED: Blue flash
  printf("[Init] LED Test\n");
  set_status_leds(0, 0, 10);
  sleep_ms(1000);
  set_status_leds(0, 0, 0);
  sleep_ms(100);

  // 2. Initialize SCD41 / I2C
  printf("[Init] I2C/SCD41 Init...\n");
  sensirion_i2c_hal_init();

  // Reset Sequence
  // We removed the initial ASC check which was failing
  printf("[SCD41] WakeUp...\n");
  scd4x_wake_up();

  printf("[SCD41] StopPeriodic...\n");
  scd4x_stop_periodic_measurement();

  printf("[SCD41] Waiting 1000ms...\n");
  sleep_ms(1000);

  printf("[SCD41] Reinit...\n");
  scd4x_reinit();
  sleep_ms(200);

  // Read Serial
  uint16_t serial_0, serial_1, serial_2;
  printf("[SCD41] Reading Serial Number...\n");
  int16_t err = scd4x_get_serial_number(&serial_0, &serial_1, &serial_2);
  if (!err) {
    printf("[SCD41] Serial: 0x%04x%04x%04x\n", serial_0, serial_1, serial_2);
    set_status_leds(0, 10, 0); // Green
    sleep_ms(200);
    set_status_leds(0, 0, 0);
  } else {
    printf("[SCD41] Error reading serial: %d\n", err);
    set_status_leds(10, 0, 0); // Red
    sleep_ms(200);
    set_status_leds(0, 0, 0);
  }

  // Start Measurement
  printf("[SCD41] Starting periodic measurement...\n");
  err = scd4x_start_periodic_measurement();
  if (err)
    printf("[SCD41] Error starting measurement: %d\n", err);

  // Loop Variables
  uint16_t current_co2 = 400;
  uint32_t last_read_time = 0;
  int success_read_count = 0;
  bool asc_checked = false;

  // Breathing params
  const uint8_t MIN_BRIGHT = 5;
  const uint8_t MAX_BRIGHT = 25;
  const uint32_t BREATH_PERIOD = 4000;

  printf("[System] Entering main loop...\n");

  while (true) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // --- ASC Check Task (Run ONCE after 5 successful reads) ---
    if (!asc_checked && success_read_count >= 5) {
      printf("\n[System] logic trigger: Check ASC Status...\n");

      // 1. Stop Measurement
      printf("[SCD41] Stopping measurement...\n");
      int16_t ret = scd4x_stop_periodic_measurement();
      if (ret)
        printf("[SCD41] Stop warning: %d\n", ret);

      // Wait 1000ms for sensor to stop
      sleep_ms(1000);

      // 2. Get/Set ASC
      uint16_t asc_enabled = 0;
      ret = scd4x_get_automatic_self_calibration(&asc_enabled);
      if (!ret) {
        printf("[SCD41] Current ASC Status: %d\n", asc_enabled);
        if (asc_enabled != 1) {
          printf("[SCD41] Enabling ASC...\n");
          ret = scd4x_set_automatic_self_calibration(1);
          if (!ret)
            printf("[SCD41] ASC Enabled Successfully.\n");
          else
            printf("[SCD41] Error enabling ASC: %d\n", ret);
        } else {
          printf("[SCD41] ASC is already Enabled. Good.\n");
        }
      } else {
        printf("[SCD41] Failed to read ASC status: %d\n", ret);
      }

      // 3. Restart Measurement
      printf("[SCD41] Restarting measurement...\n");
      scd4x_start_periodic_measurement();
      asc_checked = true; // Mark done
    }

    // --- Sensor Read Task ---
    if (now - last_read_time > 500) {
      uint16_t ready_status = 0;
      err = scd4x_get_data_ready_status(&ready_status);
      if (!err && (ready_status & 0x7FF)) {
        uint16_t co2;
        int32_t temperature;
        int32_t humidity;
        err = scd4x_read_measurement(&co2, &temperature, &humidity);
        if (!err) {
          current_co2 = co2;
          printf("CO2: %u ppm, T: %.2f C, H: %.2f %%RH\n", co2,
                 temperature / 1000.0f, humidity / 1000.0f);
          last_read_time = now;
          success_read_count++;
        }
      } else if (err) {
        if (now % 2000 < 50)
          printf("[Loop] Read Error: %d\n", err);
      }
    }

    // --- LED Animation Task ---
    uint32_t phase = now % BREATH_PERIOD;
    float factor;
    if (phase < (BREATH_PERIOD / 2)) {
      factor = (float)phase / (BREATH_PERIOD / 2);
    } else {
      factor = (float)(BREATH_PERIOD - phase) / (BREATH_PERIOD / 2);
    }

    uint8_t brightness =
        MIN_BRIGHT + (uint8_t)((MAX_BRIGHT - MIN_BRIGHT) * factor);

    if (current_co2 <= 2000) {
      // Breathing Mode
      if (current_co2 < 800) {
        set_status_leds(0, brightness, 0); // Green
      } else if (current_co2 <= 1200) {
        set_status_leds(brightness, brightness, 0); // Yellow
      } else {
        set_status_leds(brightness, 0, 0); // Red
      }
    } else {
      // Purple Blink Mode (> 2000)
      if ((now % 1000) < 500) {
        set_status_leds(MAX_BRIGHT, 0, MAX_BRIGHT);
      } else {
        set_status_leds(0, 0, 0);
      }
    }

    sleep_ms(20);
  }
  return 0;
}
