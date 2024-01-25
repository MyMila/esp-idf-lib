//
// Created by Mladen Đurić - Private on 9.1.22..
//

#ifndef ESP_IDF_LIB_ENS210_H
#define ESP_IDF_LIB_ENS210_H

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef    __cplusplus
extern "C" {
#endif

#define ENS210_ADDR 0x43 //!< I2C address

typedef esp_err_t (*ens210_write_reg_t)(uint8_t reg, uint8_t *pbuff, uint16_t bufflen);
typedef esp_err_t (*ens210_read_reg_t)(uint8_t reg, uint8_t *pbuff, uint16_t bufflen);

/**
 * @brief Initialize device descriptor
 *
 * @param dev I2C device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return ESP_OK to indicate success
 */
esp_err_t ens210_init_desc(ens210_write_reg_t ens210_wr_t, ens210_read_reg_t ens210_rr_t);

/**
 * @brief Free device descriptor
 *
 * @return ESP_OK to indicate success
 */
esp_err_t ens210_free_desc(void);

/**
 * @brief Returns ENS210 part id and uid
 *
 * @param part_id Pointer where to store part id
 * @param uid Pointer where to store uid
 * @return  - ESP_OK - success
 *          - ESP_FAIL - fail
 */
esp_err_t ens210_version(uint16_t *part_id, uint64_t *uid);

/**
 * @brief Start single measurement mode
 *
 * @param enable Should single measurement mode be enabled
 * @return  - ESP_OK - success
 *          - ESP_FAIL - fail
 */
esp_err_t ens210_single_measurement_mode(bool enable);

/**
 * @brief Reset
 *
 * @return  - ESP_OK - success
 *          - ESP_FAIL - fail
 */
esp_err_t ens210_reset(void);

/**
 * @brief Measure
 *
 * @param temperature Pointer where to store temperature reading
 * @param humidity Pointer where to store humidity reading
 * @return  - ESP_OK - success
 *          - ESP_FAIL - fail
 */
esp_err_t ens210_measure(float *temperature, float *humidity);

/**
 * @brief Set soldering correction used to compensate temperature difference caused by soldering
 * @note See "Effect of Soldering on Temperature Readout" in "Design-Guidelines"
 *
 * Correction: 50*64/1000
 *
 * @param temperature Pointer where to store temperature reading
 * @param humidity Pointer where to store humidity reading
 * @return  - ESP_OK - success
 *          - ESP_FAIL - fail
 */
void ens210_set_correction(uint8_t correction);

/**
 * @brief Get absolute humidity out of relative humidity and temperature
 *
 * @param temperature Temperature reading in Celsius
 * @param humidity Humidity reading in percentage
 * @return Absolute humidity
 */
float ens210_absolute_humidity(float temperature, float humidity);

/**
 * @brief Get ENS210 availability
 *        ENS210 is determined available if it returns correct Part ID on initialization
 * @return boolean
 */
bool ens210_available();

#ifdef    __cplusplus
}
#endif

#endif //ESP_IDF_LIB_ENS210_H
