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
 * @file dht.c
 *
 * ESP-IDF driver for DHT11, AM2301 (DHT21, DHT22, AM2302, AM2321), Itead Si7021
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Jonathan Hartsuiker <https://github.com/jsuiker>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>\n
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "dht.h"

#include <freertos/FreeRTOS.h>
#include <string.h>
#include <esp_log.h>

#if CONFIG_IDF_TARGET_ESP32

#include <esp32/rom/ets_sys.h>

#elif CONFIG_IDF_TARGET_ESP32S2
#include <esp32s2/rom/ets_sys.h>
#elif CONFIG_IDF_TARGET_ESP32S3
#include <esp32s3/rom/ets_sys.h>
#elif CONFIG_IDF_TARGET_ESP32C3
#include <esp32c3/rom/ets_sys.h>
#elif CONFIG_IDF_TARGET_ESP32H2
#include <esp32h2/rom/ets_sys.h>
#elif CONFIG_IDF_TARGET_ESP32C2
#include <esp32c2/rom/ets_sys.h>
#endif

#include <esp_idf_lib_helpers.h>

// DHT timer precision in microseconds
#define DHT_TIMER_INTERVAL 2
#define DHT_DATA_BITS 40
#define DHT_DATA_BYTES (DHT_DATA_BITS / 8)
#define DHT_RESPONSE_MAX_TIMING 215
#define DHT_MIN_TIMING 30
#define DHT_MAX_TIMING 185
#define DHT_ONE_TIMING 110


/*
 *  Note:
 *  A suitable pull-up resistor should be connected to the selected GPIO line
 *
 *  __           ______          _______                              ___________________________
 *    \    A    /      \   C    /       \   DHT duration_data_low    /                           \
 *     \_______/   B    \______/    D    \__________________________/   DHT duration_data_high    \__
 *
 *
 *  Initializing communications with the DHT requires four 'phases' as follows:
 *
 *  Phase A - MCU pulls signal low for at least 18000 us
 *  Phase B - MCU allows signal to float back up and waits 20-40us for DHT to pull it low
 *  Phase C - DHT pulls signal low for ~80us
 *  Phase D - DHT lets signal float back up for ~80us
 *
 *  After this, the DHT transmits its first bit by holding the signal low for 50us
 *  and then letting it float back high for a period of time that depends on the data bit.
 *  duration_data_high is shorter than 50us for a logic '0' and longer than 50us for logic '1'.
 *
 *  There are a total of 40 data bits transmitted sequentially. These bits are read into a byte array
 *  of length 5.  The first and third bytes are humidity (%) and temperature (C), respectively.  Bytes 2 and 4
 *  are zero-filled and the fifth is a checksum such that:
 *
 *  byte_5 == (byte_1 + byte_2 + byte_3 + byte_4) & 0xFF
 *
 */

static const char *TAG = "dht";

#if HELPER_TARGET_IS_ESP32
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL() portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL() portEXIT_CRITICAL(&mux)

#elif HELPER_TARGET_IS_ESP8266
#define PORT_ENTER_CRITICAL() portENTER_CRITICAL()
#define PORT_EXIT_CRITICAL() portEXIT_CRITICAL()
#endif

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define IOT_CHECK(tag, a, ret)  if(!(a)) {                                             \
        ESP_LOGE(tag,"%s:%d (%s)", __FILE__, __LINE__, __FUNCTION__);      \
        return (ret);                                                                   \
        }
#define POINT_ASSERT(tag, param, ret)    IOT_CHECK(tag, (param) != NULL, (ret))
#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            PORT_EXIT_CRITICAL(); \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

typedef enum {
    DHT_STATE_STOPPED,
    DHT_STATE_RESPONSE,
    DHT_STATE_DATA,
    DHT_STATE_ACQUIRED,
} dht_state_t;

typedef struct {
    dht_sensor_type_t type;
    gpio_num_t pin;
    dht_mode_t mode;
    uint8_t index;
    uint8_t count;
    uint8_t data[DHT_DATA_BYTES];
    dht_state_t state;
    int16_t humidity;
    int16_t temperature;
    int64_t read_started_at;
} dht_sensor_t;

/**
 * Pack two data bytes into single value and take into account sign bit.
 */
static inline int16_t dht_convert_data(dht_sensor_type_t sensor_type, uint8_t msb, uint8_t lsb)
{
    int16_t data;

    if (sensor_type == DHT_TYPE_DHT11) {
        data = msb * 10;
    } else {
        data = msb & 0x7F;
        data <<= 8;
        data |= lsb;
        if (msb & BIT(7))
            data = -data;       // convert it to negative
    }

    return data;
}

/**
 * @source https://github.com/chaeplin/PietteTech_DHT-8266/blob/master/PietteTech_DHT.cpp
 */
static void isr_handler(void *arg)
{
    dht_sensor_t *sensor = (dht_sensor_t *) arg;

    int64_t current_time = esp_timer_get_time();
    int64_t delta = (current_time - sensor->read_started_at);
    sensor->read_started_at = current_time;

    if (delta > 6000) {
        sensor->state = DHT_STATE_STOPPED;
        gpio_intr_disable(sensor->pin);
        return;
    }
    switch (sensor->state) {
        case DHT_STATE_RESPONSE:            // Spec: 80us LOW followed by 80us HIGH
            if (delta < 65) {      // Spec: 20-200us to first falling edge of response
                sensor->read_started_at -= delta;
                break; //do nothing, it started the response signal
            }
            if (125 < delta && delta < DHT_RESPONSE_MAX_TIMING) {
                sensor->state = DHT_STATE_DATA;
            } else {
                sensor->state = DHT_STATE_STOPPED;
                gpio_intr_disable(sensor->pin);
            }
            break;
        case DHT_STATE_DATA:          // Spec: 50us low followed by high of 26-28us = 0, 70us = 1
            if (DHT_MIN_TIMING < delta && delta < DHT_MAX_TIMING) { //valid in timing
                sensor->data[sensor->index] <<= 1; // shift the data
                if (delta > DHT_ONE_TIMING) //is a one
                    sensor->data[sensor->index] |= 1;
                if (sensor->count == 0) { // we have completed the byte, go to next
                    sensor->count = 7; // restart at MSB
                    if (++sensor->index == 5) { // go to next byte, if we have got 5 bytes stop.
                        gpio_intr_disable(sensor->pin);
                        // Verify checksum
                        uint8_t sum = sensor->data[0] + sensor->data[1] + sensor->data[2] + sensor->data[3];
                        if (sensor->data[4] != sum) {
                            sensor->state = DHT_STATE_STOPPED;
                        } else {
                            sensor->humidity = dht_convert_data(sensor->type, sensor->data[0], sensor->data[1]);
                            sensor->temperature = dht_convert_data(sensor->type, sensor->data[2], sensor->data[3]);
                            sensor->state = DHT_STATE_ACQUIRED;
                        }
                        break;
                    }
                } else sensor->count--;
            } else {
                sensor->state = DHT_STATE_STOPPED;
                gpio_intr_disable(sensor->pin);
            }
            break;
        default:
            break;
    }
}

dht_handle_t dht_init(gpio_num_t pin, dht_sensor_type_t type, dht_mode_t mode)
{
    IOT_CHECK(TAG, pin < GPIO_NUM_MAX, NULL);

    dht_sensor_t *sensor = (dht_sensor_t *) calloc(1, sizeof(dht_sensor_t));
    POINT_ASSERT(TAG, sensor, NULL);

    sensor->pin = pin;
    sensor->type = type;
    sensor->mode = mode;

    esp_err_t res = ESP_OK;
    if (mode == DHT_MODE_ASYNC) {
        res |= gpio_set_intr_type(sensor->pin, GPIO_INTR_NEGEDGE);
        res |= gpio_install_isr_service(0);
        res |= gpio_isr_handler_add(sensor->pin, isr_handler, (void *) sensor);
        res |= gpio_intr_disable(sensor->pin);
    }

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "dht_init: %s", esp_err_to_name(res));
        goto err;
    }

    return (dht_handle_t) sensor;

    err:
    gpio_uninstall_isr_service();
    free(sensor);
    return NULL;
}

/**
 * Wait specified time for pin to go to a specified state.
 * If timeout is reached and pin doesn't go to a requested state
 * false is returned.
 * The elapsed time is returned in pointer 'duration' if it is not NULL.
 */
static esp_err_t dht_await_pin_state(gpio_num_t pin, uint32_t timeout,
                                     int expected_pin_state, uint32_t *duration)
{
    /* XXX dht_await_pin_state() should save pin direction and restore
     * the direction before return. however, the SDK does not provide
     * gpio_get_direction().
     */
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    for (uint32_t i = 0; i < timeout; i += DHT_TIMER_INTERVAL) {
        // need to wait at least a single interval to prevent reading a jitter
        ets_delay_us(DHT_TIMER_INTERVAL);
        if (gpio_get_level(pin) == expected_pin_state) {
            if (duration)
                *duration = i;
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

/**
 * Request data from DHT and read raw bit stream.
 * The function call should be protected from task switching.
 * Return false if error occurred.
 */
static inline esp_err_t
dht_await_fetch_data(dht_sensor_type_t sensor_type, gpio_num_t pin, uint8_t data[DHT_DATA_BYTES])
{
    uint32_t low_duration;
    uint32_t high_duration;

    // Phase 'A' pulling signal low to initiate read sequence
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 0);
    ets_delay_us(sensor_type == DHT_TYPE_SI7021 ? 500 : 20000);
    gpio_set_level(pin, 1);

    // Step through Phase 'B', 40us
    CHECK_LOGE(dht_await_pin_state(pin, 40, 0, NULL),
               "Initialization error, problem in phase 'B'");
    // Step through Phase 'C', 88us
    CHECK_LOGE(dht_await_pin_state(pin, 88, 1, NULL),
               "Initialization error, problem in phase 'C'");
    // Step through Phase 'D', 88us
    CHECK_LOGE(dht_await_pin_state(pin, 88, 0, NULL),
               "Initialization error, problem in phase 'D'");

    // Read in each of the 40 bits of data...
    for (int i = 0; i < DHT_DATA_BITS; i++) {
        CHECK_LOGE(dht_await_pin_state(pin, 65, 1, &low_duration),
                   "LOW bit timeout");
        CHECK_LOGE(dht_await_pin_state(pin, 75, 0, &high_duration),
                   "HIGH bit timeout");

        uint8_t b = i / 8;
        uint8_t m = i % 8;
        if (!m)
            data[b] = 0;

        data[b] |= (high_duration > low_duration) << (7 - m);
    }

    return ESP_OK;
}

static esp_err_t dht_await_read(dht_sensor_t *sensor)
{
    esp_err_t result = ESP_OK;

    gpio_set_direction(sensor->pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(sensor->pin, 1);

    PORT_ENTER_CRITICAL();
    result = dht_await_fetch_data(sensor->type, sensor->pin, sensor->data);
    if (result == ESP_OK)
        PORT_EXIT_CRITICAL();

    /* restore GPIO direction because, after calling dht_fetch_data(), the
     * GPIO direction mode changes */
    gpio_set_direction(sensor->pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(sensor->pin, 1);

    if (result != ESP_OK)
        return result;

    if (sensor->data[4] != ((sensor->data[0] + sensor->data[1] + sensor->data[2] + sensor->data[3]) & 0xFF)) {
        ESP_LOGE(TAG, "Checksum failed, invalid data received from sensor");
        return ESP_ERR_INVALID_CRC;
    }

    sensor->humidity = dht_convert_data(sensor->type, sensor->data[0], sensor->data[1]);
    sensor->temperature = dht_convert_data(sensor->type, sensor->data[2], sensor->data[3]);

    ESP_LOGD(TAG, "Sensor data: humidity=%d, temp=%d", sensor->humidity, sensor->temperature);

    return ESP_OK;
}

static esp_err_t dht_async_read(dht_sensor_t *sensor)
{
    sensor->state = DHT_STATE_RESPONSE;

    // Phase 'A' pulling signal low to initiate read sequence
    gpio_set_direction(sensor->pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(sensor->pin, 0);
    ets_delay_us(sensor->type == DHT_TYPE_SI7021 ? 500 : 20000); // todo: improvement
    gpio_set_direction(sensor->pin, GPIO_MODE_INPUT);

    sensor->read_started_at = esp_timer_get_time();
    gpio_intr_enable(sensor->pin);

    return ESP_OK;
}

esp_err_t dht_read(dht_handle_t handle)
{
    CHECK_ARG(handle);

    dht_sensor_t *sensor = (dht_sensor_t *) handle;

    if (sensor->state != DHT_STATE_STOPPED && sensor->state != DHT_STATE_ACQUIRED) {
        return ESP_FAIL;
    }

    memset(sensor->data, 0, sizeof(sensor->data));
    sensor->count = 7;
    sensor->index = 0;

    return sensor->mode == DHT_MODE_AWAIT
           ? dht_await_read(sensor)
           : dht_async_read(sensor);
}

float dht_temperature(dht_handle_t handle)
{
    POINT_ASSERT(TAG, handle, ESP_FAIL);

    return ((dht_sensor_t *) handle)->temperature / 10.0f;
}

float dht_humidity(dht_handle_t handle)
{
    POINT_ASSERT(TAG, handle, ESP_FAIL);

    return ((dht_sensor_t *) handle)->humidity / 10.0f;
}
