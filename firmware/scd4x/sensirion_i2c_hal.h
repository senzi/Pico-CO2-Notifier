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

#ifndef SENSIRION_I2C_HAL_H
#define SENSIRION_I2C_HAL_H

#include "sensirion_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * sensirion_i2c_hal_init() - Initialize the I2C HAL
 */
void sensirion_i2c_hal_init(void);

/**
 * sensirion_i2c_hal_free() - Release the I2C HAL
 */
void sensirion_i2c_hal_free(void);

/**
 * sensirion_i2c_hal_read() - Execute an I2C read transaction
 *
 * @address:    I2C address to read from
 * @data:       Pointer to the buffer to store the read data
 * @count:      Number of bytes to read
 *
 * Return:      0 on success, an error code otherwise
 */
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t *data, uint16_t count);

/**
 * sensirion_i2c_hal_write() - Execute an I2C write transaction
 *
 * @address:    I2C address to write to
 * @data:       Pointer to the buffer containing the data to write
 * @count:      Number of bytes to write
 *
 * Return:      0 on success, an error code otherwise
 */
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t *data,
                               uint16_t count);

/**
 * sensirion_i2c_hal_sleep_usec() - Sleep for a given number of microseconds
 *
 * @useconds:   Number of microseconds to sleep
 */
void sensirion_i2c_hal_sleep_usec(uint32_t useconds);

#ifdef __cplusplus
}
#endif

#endif /* SENSIRION_I2C_HAL_H */
