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

#include "sensirion_i2c.h"
#include "sensirion_common.h"
#include "sensirion_config.h"
#include "sensirion_i2c_hal.h"

uint16_t sensirion_fill_cmd_send_buf(uint8_t *buf, uint16_t cmd,
                                     const uint16_t *args, uint8_t num_args) {
  uint8_t crc;
  uint8_t i;
  uint16_t idx = 0;

  buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
  buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);

  for (i = 0; i < num_args; ++i) {
    buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
    buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);

    crc = sensirion_common_generate_crc((uint8_t *)&buf[idx - 2], 2);
    buf[idx++] = crc;
  }
  return idx;
}

int16_t sensirion_i2c_read_words_as_bytes(uint8_t address, uint8_t *data,
                                          uint16_t num_words) {
  int16_t ret;
  uint16_t i, j;
  uint16_t size = num_words * (2 + 1);
  uint8_t buf[size];

  ret = sensirion_i2c_hal_read(address, buf, size);
  if (ret != 0)
    return ret;

  for (i = 0; i < num_words; ++i) {
    j = i * 3;
    if (sensirion_common_check_crc(&buf[j], 2, buf[j + 2]) != 0)
      return -2; // CRC Fail

    data[i * 2] = buf[j];
    data[i * 2 + 1] = buf[j + 1];
  }

  return 0;
}

int16_t sensirion_i2c_read_words(uint8_t address, uint16_t *data_words,
                                 uint16_t num_words) {
  int16_t ret;
  uint8_t buf[num_words * 2];

  ret = sensirion_i2c_read_words_as_bytes(address, buf, num_words);
  if (ret != 0)
    return ret;

  for (uint16_t i = 0; i < num_words; ++i) {
    data_words[i] = (uint16_t)buf[i * 2] << 8;
    data_words[i] |= (uint16_t)buf[i * 2 + 1];
  }

  return 0;
}

int16_t sensirion_i2c_write_cmd(uint8_t address, uint16_t command) {
  uint8_t buf[2];

  sensirion_fill_cmd_send_buf(buf, command, 0, 0);
  return sensirion_i2c_hal_write(address, buf, 2);
}

int16_t sensirion_i2c_write_cmd_with_args(uint8_t address, uint16_t command,
                                          const uint16_t *data_words,
                                          uint16_t num_words) {
  uint8_t buf[2 + num_words * 3];
  uint16_t buf_size;

  buf_size = sensirion_fill_cmd_send_buf(buf, command, data_words, num_words);
  return sensirion_i2c_hal_write(address, buf, buf_size);
}

int16_t sensirion_i2c_delayed_read_cmd(uint8_t address, uint16_t cmd,
                                       uint32_t delay_us, uint16_t *data_words,
                                       uint16_t num_words) {
  int16_t ret;
  uint8_t buf[2];

  sensirion_fill_cmd_send_buf(buf, cmd, 0, 0);
  ret = sensirion_i2c_hal_write(address, buf, 2);
  if (ret != 0)
    return ret;

  if (delay_us > 0)
    sensirion_i2c_hal_sleep_usec(delay_us);

  return sensirion_i2c_read_words(address, data_words, num_words);
}
