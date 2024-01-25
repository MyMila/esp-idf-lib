//
// Created by Mladen Đurić - Private on 5.3.22..
//

#ifndef ESP_IDF_LIB_PMS9003_H
#define ESP_IDF_LIB_PMS9003_H

#include <stdint.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/uart.h>

typedef enum pms_mode {
    PMS_MODE_ACTIVE,
    PMS_MODE_PASSIVE,
} pms_mode_t;

typedef enum pms_status {
    PMS_STATUS_FAIL = -1,
    PMS_STATUS_OK = 0,
    PMS_STATUS_TOO_MANY_FAN_RESTARTS = BIT0,
    PMS_STATUS_FAN_SPEED_LOW = BIT1,
    PMS_STATUS_ENVIRONMENTAL_INTERFERENCE = BIT2,
} pms_status_t;

typedef struct {
    uint16_t pm1_0_std;
    uint16_t pm2_5_std;
    uint16_t pm10_std;
    uint16_t pm1_0_atm;
    uint16_t pm2_5_atm;
    uint16_t pm10_atm;
    uint16_t pm0_3_par;
    uint16_t pm0_5_par;
    uint16_t pm1_0_par;
    uint16_t pm2_5_par;
    uint16_t pm5_0_par;
    uint16_t pm10_par;
} pms9003_measurement_t;

typedef void *pms9003_handle_t;

pms9003_handle_t
pms9003_init(esp_err_t (*reset_pin)(uint8_t state), esp_err_t (*set_pin)(uint8_t state),
             esp_err_t (*data_size)(uint8_t *size), esp_err_t (*read_data)(uint8_t *data, uint8_t size),
             esp_err_t (*write_data)(const uint8_t *data, uint8_t size));

esp_err_t pms9003_free(pms9003_handle_t handle);

esp_err_t pms9003_set_mode(pms9003_handle_t handle, pms_mode_t mode);

esp_err_t pms9003_sleep(pms9003_handle_t handle, bool sleep);

esp_err_t pms9003_reset(pms9003_handle_t handle);

esp_err_t pms9003_measure(pms9003_handle_t handle, pms9003_measurement_t *measure, uint16_t timeout_ms);

pms_status_t pms9003_status(pms9003_handle_t handle);

#endif //ESP_IDF_LIB_PMS9003_H
