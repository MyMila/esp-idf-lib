/*
 * Copyright (c) 2016 Jonathan Hartsuiker <https://github.com/jsuiker>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file dht.h
 * @defgroup dht dht
 * @{
 *
 * ESP-IDF driver for DHT11, AM2301 (DHT21, DHT22, AM2302, AM2321), Itead Si7021
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Jonathan Hartsuiker <https://github.com/jsuiker>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>\n
 *
 * BSD Licensed as described in the file LICENSE
 *
 * @note A suitable pull-up resistor should be connected to the selected GPIO line
 *
 */
#ifndef __DHT_H__
#define __DHT_H__

#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Sensor type
 */
typedef enum
{
    DHT_TYPE_DHT11 = 0,   //!< DHT11
    DHT_TYPE_AM2301,      //!< AM2301 (DHT21, DHT22, AM2302, AM2321)
    DHT_TYPE_SI7021       //!< Itead Si7021
} dht_sensor_type_t;

typedef enum
{
    DHT_MODE_AWAIT,
    DHT_MODE_ASYNC
} dht_mode_t;

typedef void *dht_handle_t;

/**
 * @brief Initialize dht sensor struct
 *
 * @param[in] pin GPIO pin
 * @param[in] type Sensor type
 * @param[in] mode Reading mode
 *
 * @return
 *  - dht_handle_t
 *  - NULL
 */
dht_handle_t dht_init(gpio_num_t pin, dht_sensor_type_t type, dht_mode_t mode);

/**
 * @brief Temperature getter
 *
 * Temperature is returned as floats.
 *
 * @param[in] handle Sensor handle
 *
 * @return Temperature, degrees Celsius, nullable
 */
float dht_temperature(dht_handle_t handle);

/**
 * @brief Humidity getter
 *
 * Humidity is returned as floats.
 *
 * @param[in] handle Sensor handle
 *
 * @return Humidity, percents, nullable
 */
float dht_humidity(dht_handle_t handle);

/**
 * @brief Read integer data from sensor on specified pin
 *
 * @param[in] dht_handle_t Sensor handle
 * @return `ESP_OK` on success
 */
esp_err_t dht_read(dht_handle_t handle);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __DHT_H__
