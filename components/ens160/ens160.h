//
// Created by Mladen Đurić - Private on 12.2.22..
//

#ifndef ESP_IDF_LIB_ENS160_H
#define ESP_IDF_LIB_ENS160_H

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#ifdef    __cplusplus
extern "C" {
#endif

// Chip constants
#define ENS160_PARTID                0x0160
#define ENS160_BOOTING_MS                50

// 7-bit I2C slave address of the ENS160
#define ENS160_I2CADDR_0            0x52        //ADDR low
#define ENS160_I2CADDR_1            0x53        //ADDR high

// ENS160 registers for version V0
#define ENS160_REG_PART_ID          0x00        // 2 byte register
#define ENS160_REG_OPMODE            0x10
#define ENS160_REG_CONFIG            0x11
#define ENS160_REG_COMMAND            0x12
#define ENS160_REG_TEMP_IN            0x13
#define ENS160_REG_RH_IN            0x15
#define ENS160_REG_DATA_STATUS        0x20
#define ENS160_REG_DATA_AQI            0x21
#define ENS160_REG_DATA_TVOC        0x22
#define ENS160_REG_DATA_ECO2        0x24
#define ENS160_REG_DATA_NO2        0x25
#define ENS160_REG_DATA_BL            0x28
#define ENS160_REG_DATA_T            0x30
#define ENS160_REG_DATA_RH            0x32
#define ENS160_REG_DATA_MISR        0x38
#define ENS160_REG_GPR_WRITE_0        0x40
#define ENS160_REG_GPR_WRITE_1       (ENS160_REG_GPR_WRITE_0 + 1)
#define ENS160_REG_GPR_WRITE_2       (ENS160_REG_GPR_WRITE_0 + 2)
#define ENS160_REG_GPR_WRITE_3       (ENS160_REG_GPR_WRITE_0 + 3)
#define ENS160_REG_GPR_WRITE_4       (ENS160_REG_GPR_WRITE_0 + 4)
#define ENS160_REG_GPR_WRITE_5       (ENS160_REG_GPR_WRITE_0 + 5)
#define ENS160_REG_GPR_WRITE_6       (ENS160_REG_GPR_WRITE_0 + 6)
#define ENS160_REG_GPR_WRITE_7       (ENS160_REG_GPR_WRITE_0 + 7)
#define ENS160_REG_GPR_READ_0        0x48
#define ENS160_REG_GPR_READ_4        (ENS160_REG_GPR_READ_0 + 4)
#define ENS160_REG_GPR_READ_6        (ENS160_REG_GPR_READ_0 + 6)
#define ENS160_REG_GPR_READ_7        (ENS160_REG_GPR_READ_0 + 7)

//ENS160 data register fields
#define ENS160_COMMAND_NOP            0x00
#define ENS160_COMMAND_CLRGPR        0xCC
#define ENS160_COMMAND_GET_APPVER    0x0E
#define ENS160_COMMAND_GET_HWVER 0x01
#define ENS160_COMMAND_SETTH        0x02
#define ENS160_COMMAND_SETSEQ        0xC2

#define ENS160_OPMODE_RESET            0xF0
#define ENS160_OPMODE_DEP_SLEEP        0x00
#define ENS160_OPMODE_IDLE            0x01
#define ENS160_OPMODE_STD            0x02
#define ENS160_OPMODE_INTERMEDIATE    0x03
#define ENS160_OPMODE_CUSTOM        0xC0
#define ENS160_OPMODE_D0            0xD0
#define ENS160_OPMODE_D1            0xD1
#define ENS160_OPMODE_BOOTLOADER    0xB0

#define ENS160_BL_CMD_START            0x02
#define ENS160_BL_CMD_ERASE_APP        0x04
#define ENS160_BL_CMD_ERASE_BLINE    0x06
#define ENS160_BL_CMD_WRITE            0x08
#define ENS160_BL_CMD_VERIFY        0x0A
#define ENS160_BL_CMD_GET_BLVER        0x0C
#define ENS160_BL_CMD_GET_APPVER    0x0E
#define ENS160_BL_CMD_EXITBL        0x12

#define ENS160_SEQ_ACK_NOTCOMPLETE    0x80
#define ENS160_SEQ_ACK_COMPLETE        0xC0

#define IS_ENS160_SEQ_ACK_NOT_COMPLETE(x)    (ENS160_SEQ_ACK_NOTCOMPLETE == (ENS160_SEQ_ACK_NOTCOMPLETE & (x)))
#define IS_ENS160_SEQ_ACK_COMPLETE(x)        (ENS160_SEQ_ACK_COMPLETE == (ENS160_SEQ_ACK_COMPLETE & (x)))

#define ENS160_DATA_STATUS_NEWDAT    0x02
#define ENS160_DATA_STATUS_NEWGPR    0x01

#define IS_NEWDAT(x)            (ENS160_DATA_STATUS_NEWDAT == (ENS160_DATA_STATUS_NEWDAT & (x)))
#define IS_NEWGPR(x)            (ENS160_DATA_STATUS_NEWGPR == (ENS160_DATA_STATUS_NEWGPR & (x)))
#define IS_NEW_DATA_AVAILABLE(x)    (0 != ((ENS160_DATA_STATUS_NEWDAT | ENS160_DATA_STATUS_NEWGPR ) & (x)))

#define CONVERT_RS_RAW2OHMS_I(x)    (1 << ((x) >> 11))
#define CONVERT_RS_RAW2OHMS_F(x)    (pow (2, (float)(x) / 2048))

typedef enum {
    ENS160_AQI_EXCELLENT = 1,
    ENS160_AQI_GOOD,
    ENS160_AQI_MODERATE,
    ENS160_AQI_POOR,
    ENS160_AQI_UNHEALTHY,
} ens160_aqi_t;

typedef enum {
    // Everything is ok
    ENS160_ERR_BL_OK = 0x00,
    // Unable to start application update mode
    ENS160_ERR_BL_START = 0x40,
    // Unable to erase app or baseline
    ENS160_ERR_BL_ERASE = 0x01,
    // Unable to perform FLASH write
    ENS160_ERR_BL_WRITE = 0x04,
    // No erase performed before write
    ENS160_ERR_BL_SEQ = 0x08,
    // App verification failed
    ENS160_ERR_BL_VERIFY = 0x02,
    // Reporting bootloader or app version failed
    ENS160_ERR_BL_VERSION = 0x10,
    // Unable to set OEM magic word
    ENS160_ERR_BL_MAGICSET = 0x80,
    // Unknown command supplied
    ENS160_ERR_BL_UNKNOWN = 0x20,
} ens160_bl_err_t;

/**
* @brief Initialize device descriptor
*
* @param dev I2C device descriptor
* @param addr ENS160 address
* @param bootloader Boot into bootloader after initialization
* @param port I2C port
* @param sda_gpio SDA GPIO
* @param scl_gpio SCL GPIO
* @return ESP_OK to indicate success
*/
esp_err_t ens160_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev I2C device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t ens160_free_desc(i2c_dev_t *dev);

/**
 * @brief Reset
 *
 * @param dev I2C device descriptor
 * @return  - ESP_OK - success
 *          - ESP_FAIL - fail
 */
esp_err_t ens160_reset(i2c_dev_t *dev);

/**
 * @brief Get part id
 *
 * @param dev I2C device descriptor
 * @param part_id Pointer where to store part id
 * @return  - ESP_OK - success
 *          - ESP_FAIL - fail
 */
esp_err_t ens160_part_id(i2c_dev_t *dev, uint16_t *part_id);

/**
 * @brief Get firmware version
 *
 * @param dev I2C device descriptor
 * @param fw_version Pointer where to store firmware version
 * @param hw_version Pointer where to store firmware version
 * @return  - ESP_OK - success
 *          - ESP_FAIL - fail
 */
esp_err_t ens160_versions(i2c_dev_t *dev, uint8_t (*fw_version)[3], uint8_t (*hw_version)[2]);

/**
 * @brief Flash firmware image
 *
 * @param dev I2C device descriptor
 * @param app_img Pointer where application image is stored
 * @param hw_version Size of application image
 * @return  - ESP_OK - success
 *          - ESP_FAIL - fail
 */
esp_err_t ens160_flash_firmware(i2c_dev_t *dev, const uint8_t *app_img, int size);

/**
 * @brief Store environmental data (temperature and humidity), needed for compensation
 * @param dev I2C device descriptor
 * @param temperature Temperature in degrees celsius
 * @param humidity Relative humidity in percentage
 * @return - ESP_OK - success
 *         - ESP_FAIL - fail
 */
esp_err_t ens160_set_environmental_data(i2c_dev_t *dev, float temperature, float humidity);

/**
 * @brief Get TVOC, eCO2, AQI and NO2 readings
 * @param dev I2C device descriptor
 * @param wait_for_new Block on waiting for new data
 * @param aqi AQI reading
 * @param tvoc TVOC reading
 * @param eco2 eCO2 reading
 * @return
 */
esp_err_t
ens160_measure(i2c_dev_t *dev, bool wait_for_new, ens160_aqi_t *aqi, uint16_t *tvoc, uint16_t *eco2,
               uint16_t (*resistance)[4], uint16_t (*baseline)[4]);

/**
 * @brief Reset baseline
 * @param dev I2C device descriptor
 * @param error error response
 */
esp_err_t ens160_reset_baseline(i2c_dev_t *dev, ens160_bl_err_t* err);

/**
 * @brief Set mode
 * @param dev I2C device descriptor
 * @param mode mode in which sensor will be running
 * @return
 */
esp_err_t ens160_set_mode(i2c_dev_t *dev, uint8_t mode);

#ifdef    __cplusplus
}
#endif

#endif //ESP_IDF_LIB_ENS160_H
