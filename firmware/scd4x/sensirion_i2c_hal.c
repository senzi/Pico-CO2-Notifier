/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_config.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

// Configuration: Assume I2C1, GP6 (SDA), GP7 (SCL) as per previous test
// Note: In a real HAL, these might be arguments or user configurable globally.
// Here we hardcode or pick up from global defines if available.
// But since this is a HAL implementation file, we should keep it focused.
// We will assume `i2c1` is initialized elsewhere (in main), or we initialize it
// here. For robustness, let's allow `sensirion_i2c_hal_init` to initialize it.

#define I2C_PORT i2c1
#define I2C_SDA 6
#define I2C_SCL 7
#define I2C_FREQ 100000

void sensirion_i2c_hal_init(void) {
  i2c_init(I2C_PORT, I2C_FREQ);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);
}

void sensirion_i2c_hal_free(void) {
  // Optional: deinit I2C
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t *data, uint16_t count) {
  // Pico SDK: i2c_read_timeout_us returns number of bytes read or error (<0)
  // Use 100ms timeout to avoid hanging
  int ret = i2c_read_timeout_us(I2C_PORT, address, data, count, false, 100000);
  if (ret == count) {
    return 0;
  }
  return -1;
}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t *data,
                               uint16_t count) {
  // Pico SDK: i2c_write_timeout_us returns number of bytes written or error
  // (<0) Use 100ms timeout to avoid hanging
  int ret = i2c_write_timeout_us(I2C_PORT, address, data, count, false, 100000);
  if (ret == count) {
    return 0;
  }
  return -1;
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) { sleep_us(useconds); }
