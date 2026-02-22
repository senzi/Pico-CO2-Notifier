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

#ifndef SCD4X_I2C_H
#define SCD4X_I2C_H

#include "sensirion_config.h"

#ifdef __cplusplus
extern "C" {
#endif

int16_t scd4x_start_periodic_measurement(void);

int16_t scd4x_read_measurement(uint16_t *co2, int32_t *temperature,
                               int32_t *humidity);

int16_t scd4x_stop_periodic_measurement(void);

int16_t scd4x_get_serial_number(uint16_t *serial_0, uint16_t *serial_1,
                                uint16_t *serial_2);

int16_t scd4x_get_data_ready_status(uint16_t *data_ready);

int16_t scd4x_wake_up(void);

int16_t scd4x_perform_self_test(uint16_t *sensor_status);

int16_t scd4x_perform_factory_reset(void);

int16_t scd4x_reinit(void);

int16_t scd4x_get_automatic_self_calibration(uint16_t *asc_enabled);

int16_t scd4x_set_automatic_self_calibration(uint16_t asc_enabled);

#ifdef __cplusplus
}
#endif

#endif /* SCD4X_I2C_H */
