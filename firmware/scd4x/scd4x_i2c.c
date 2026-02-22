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

#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"

#define SCD4X_I2C_ADDRESS 0x62

int16_t scd4x_start_periodic_measurement(void) {
  int16_t error;
  error = sensirion_i2c_write_cmd(SCD4X_I2C_ADDRESS, 0x21b1);
  return error;
}

int16_t scd4x_read_measurement(uint16_t *co2, int32_t *temperature,
                               int32_t *humidity) {
  int16_t error;
  uint16_t data[3];

  error =
      sensirion_i2c_delayed_read_cmd(SCD4X_I2C_ADDRESS, 0xec05, 1000, data, 3);
  if (error)
    return error;

  *co2 = data[0];
  *temperature = ((21875 * (int32_t)data[1]) >> 13) - 45000;
  *humidity = ((12500 * (int32_t)data[2]) >> 13);
  return 0;
}

int16_t scd4x_stop_periodic_measurement(void) {
  int16_t error;
  error = sensirion_i2c_write_cmd(SCD4X_I2C_ADDRESS, 0x3f86);
  if (error)
    return error;

  sensirion_i2c_hal_sleep_usec(500000);
  return 0;
}

int16_t scd4x_get_serial_number(uint16_t *serial_0, uint16_t *serial_1,
                                uint16_t *serial_2) {
  int16_t error;
  uint16_t data[3];

  error =
      sensirion_i2c_delayed_read_cmd(SCD4X_I2C_ADDRESS, 0x3682, 1000, data, 3);
  if (error)
    return error;

  *serial_0 = data[0];
  *serial_1 = data[1];
  *serial_2 = data[2];
  return 0;
}

int16_t scd4x_get_data_ready_status(uint16_t *data_ready) {
  int16_t error;
  uint16_t data;

  error =
      sensirion_i2c_delayed_read_cmd(SCD4X_I2C_ADDRESS, 0xe4b8, 1000, &data, 1);
  if (error)
    return error;

  *data_ready = data & 0x7ff;
  return 0;
}

int16_t scd4x_wake_up(void) {
  return sensirion_i2c_write_cmd(SCD4X_I2C_ADDRESS, 0x36f6);
}

int16_t scd4x_perform_self_test(uint16_t *sensor_status) {
  int16_t error;
  uint16_t data;

  error = sensirion_i2c_delayed_read_cmd(SCD4X_I2C_ADDRESS, 0x3639, 10000000,
                                         &data, 1);
  if (error)
    return error;

  *sensor_status = data;
  return 0;
}

int16_t scd4x_perform_factory_reset(void) {
  return sensirion_i2c_write_cmd(SCD4X_I2C_ADDRESS, 0x3632);
}

int16_t scd4x_reinit(void) {
  return sensirion_i2c_write_cmd(SCD4X_I2C_ADDRESS, 0x3646);
}

int16_t scd4x_get_automatic_self_calibration(uint16_t *asc_enabled) {
  int16_t error;
  uint16_t data;
  // Commmand 0x2313, Execution time 1ms
  error =
      sensirion_i2c_delayed_read_cmd(SCD4X_I2C_ADDRESS, 0x2313, 1000, &data, 1);
  if (error)
    return error;
  *asc_enabled = data;
  return 0;
}

int16_t scd4x_set_automatic_self_calibration(uint16_t asc_enabled) {
  // Command 0x2416, 1 argument
  return sensirion_i2c_write_cmd_with_args(SCD4X_I2C_ADDRESS, 0x2416,
                                           &asc_enabled, 1);
}
