//
// Created by Mladen Đurić - Private on 12.2.22..
//
#include <string.h>
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include <math.h>
#include "ens160.h"

#define I2C_FREQ_HZ 100000

#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "ENS160";
static const uint8_t ENS160_BL_MAGIC[4] = {0x53, 0xCE, 0x1A, 0xBF};

// Initialize idle mode and confirms
static esp_err_t ens160_clear_command(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    uint8_t data;
    I2C_DEV_TAKE_MUTEX(dev);

    data = ENS160_COMMAND_NOP;
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_COMMAND, &data, 1));
    data = ENS160_COMMAND_CLRGPR;
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_COMMAND, &data, 1));

    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ens160_set_mode(i2c_dev_t *dev, uint8_t mode)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_OPMODE, &mode, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ens160_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    esp_err_t res = ESP_OK;

    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
    dev->timeout_ticks = 0;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    if (i2c_dev_create_mutex(dev) != ESP_OK) {
        return ESP_FAIL;
    }

    uint16_t part_id;
    if (ens160_part_id(dev, &part_id) != ESP_OK) {
        res = ESP_FAIL;
        goto err;
    }

    if (part_id != ENS160_PARTID) {
        res = ESP_FAIL;
        goto err;
    }

    uint8_t fw_version[3] = { 0 };
    uint8_t hw_version[2] = { 0 };
    if (ens160_versions(dev, &fw_version, &hw_version) == ESP_OK) {
        ESP_LOGI(TAG, "Firmware version: %x.%x.%x", fw_version[0], fw_version[1], fw_version[2]);
        ESP_LOGI(TAG, "Hardware version: %x.%x", hw_version[0], hw_version[1]);
    }

    if (ens160_reset(dev) != ESP_OK) {
        res = ESP_FAIL;
        goto err;
    }

    if (ens160_set_mode(dev, ENS160_OPMODE_STD) != ESP_OK) {
        res = ESP_FAIL;
        goto err;
    }

err:
    if (res == ESP_FAIL) {
        i2c_dev_delete_mutex(dev);
    }
    return res;
}

/**
 * @brief Helper method for sending commands and verifying responses in bootloader mode
 * @param dev
 * @param command
 * @return
 */
static ens160_bl_err_t ens160_bl_write(i2c_dev_t *dev, uint8_t command)
{
    CHECK_ARG(dev);

    uint8_t data;

    // In bootloader, commands should be sent as NOP, followed by command
    I2C_DEV_TAKE_MUTEX(dev);
    data = ENS160_COMMAND_NOP;
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_COMMAND, &data, 1));
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_COMMAND, &command, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    // Wait on command to execute and data to be available for reading
    uint8_t i = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(10));

        I2C_DEV_TAKE_MUTEX(dev);
        I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS160_REG_DATA_STATUS, &data, 1));
        I2C_DEV_GIVE_MUTEX(dev);

        i++;
    } while (!IS_NEWGPR(data) && i < 10);

    if (!IS_NEWGPR(data)) {
        return ENS160_ERR_BL_UNKNOWN;
    }

    I2C_DEV_TAKE_MUTEX(dev);
    uint8_t status[2] = { 0 };
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS160_REG_GPR_READ_6, &data, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    // GPR_READ_6 should be command + 1 if command was successful
    // GPR_READ_7 stored error code in case of error
    return status[1] != (command + 1) ? status[0] : ENS160_ERR_BL_OK;
}

esp_err_t ens160_reset_baseline(i2c_dev_t *dev, ens160_bl_err_t* err)
{
    CHECK_ARG(dev);

    /*
     * Bootloader can be entered only from if it's first issues OPMODE after power-on reset.
     * Because of this we're first performing reset to enter bootloader mode.
     */
    if (ens160_reset(dev) != ESP_OK) {
        return ESP_FAIL;
    }

    if (ens160_set_mode(dev, ENS160_OPMODE_BOOTLOADER) != ESP_OK) {
        return ESP_FAIL;
    }

    // Write magic
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_GPR_WRITE_0, &ENS160_BL_MAGIC, 4));
    I2C_DEV_GIVE_MUTEX(dev);

    // Start bootloader
    if ((*err = ens160_bl_write(dev, ENS160_BL_CMD_START)) != ENS160_ERR_BL_OK) {
        ESP_LOGE(TAG, "ens160_bl_write: bl start cmd failed");
        return ESP_FAIL;
    }

    // Erase baseline
    if ((*err = ens160_bl_write(dev, ENS160_BL_CMD_ERASE_BLINE)) != ENS160_ERR_BL_OK) {
        ESP_LOGE(TAG, "ens160_bl_write: bl erase baseline cmd failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t ens160_versions(i2c_dev_t *dev, uint8_t (*fw_version)[3], uint8_t (*hw_version)[2])
{
    CHECK_ARG(dev);

    // Firmware version is only available in idle mode
    if (ens160_set_mode(dev, ENS160_OPMODE_IDLE) != ESP_OK) {
        return ESP_FAIL;
    }

    // Clear GPR registers for firmware version to be stored
    if (ens160_clear_command(dev) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t data;

    I2C_DEV_TAKE_MUTEX(dev);

    // Get firmware version
    data = ENS160_COMMAND_GET_APPVER;
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_COMMAND, &data, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS160_REG_GPR_READ_4, fw_version, 3));

    // Get hardware version
//    data = ENS160_COMMAND_GET_HWVER;
//    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_COMMAND, &data, 1));
//    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS160_REG_GPR_READ_0, hw_version, 2));

    I2C_DEV_GIVE_MUTEX(dev);

    if (ens160_set_mode(dev, ENS160_OPMODE_STD) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t ens160_write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t data)
{
    return i2c_dev_write_reg(dev, reg, &data, 1);
}

// Flash new firmware to sensor
esp_err_t ens160_flash_firmware(i2c_dev_t *dev, const uint8_t *app_img, int size)
{
    int idx = 0;
    uint8_t data;
    // go to idle mode

    I2C_DEV_TAKE_MUTEX(dev);

    data = ENS160_OPMODE_IDLE;
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_OPMODE, &data, 1));
    vTaskDelay(pdMS_TO_TICKS(ENS160_BOOTING_MS));

    // Enter testmode
    I2C_DEV_CHECK(dev, ens160_write_reg(dev, ENS160_REG_OPMODE, 0x02));
    I2C_DEV_CHECK(dev, ens160_write_reg(dev, ENS160_REG_GPR_WRITE_6, 0xa6));
    I2C_DEV_CHECK(dev, ens160_write_reg(dev, ENS160_REG_GPR_WRITE_4, 0x4a));
    I2C_DEV_CHECK(dev, ens160_write_reg(dev, ENS160_REG_GPR_WRITE_2, 0xeb));
    I2C_DEV_CHECK(dev, ens160_write_reg(dev, ENS160_REG_GPR_WRITE_0, 0x3a));

    vTaskDelay(pdMS_TO_TICKS(100));

    // halt CPU
    I2C_DEV_CHECK(dev, ens160_write_reg(dev, 0x60, 0x21));
    vTaskDelay(pdMS_TO_TICKS(100));
    I2C_DEV_CHECK(dev, ens160_write_reg(dev, 0x60, 0x09));
    vTaskDelay(pdMS_TO_TICKS(100));

    //Mass erase the entire flash except info block
    I2C_DEV_CHECK(dev, ens160_write_reg(dev, 0x65, 0x00));
    I2C_DEV_CHECK(dev, ens160_write_reg(dev, 0x6A, 0x04));
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Write application binary in pieces of 8 bytes
    //uint8_t len = 0;
    uint8_t page = 0;
    uint8_t addr = 0;
    for (idx = 0; idx < size; idx += 4) {
        int len = (size - idx) < 4 ? (size - idx) : 4;
        page = idx / 1024;
        addr = (idx - (page * 1024)) / 4;
        uint8_t ram[4];
        memcpy(ram, app_img, len);  // Copy up to 4 bytes from PROGMEM to RAM
        I2C_DEV_CHECK(dev, ens160_write_reg(dev, 0x6A, addr));
        I2C_DEV_CHECK(dev, ens160_write_reg(dev, 0x65, page));
        I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, 0x66, ram, len));
        I2C_DEV_CHECK(dev, ens160_write_reg(dev, 0x6A, 0x01));
        vTaskDelay(pdMS_TO_TICKS(1));
        I2C_DEV_CHECK(dev, ens160_write_reg(dev, 0x6C, 0x01));
        printf(".");

        app_img += len;
    }
    printf("\n");

    I2C_DEV_GIVE_MUTEX(dev);

    // idx should match sizeof(app_img);
    if ((uint16_t) idx != (uint16_t) size) {
        ESP_LOGI(TAG, "Failed to complete application image update.\t");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Write success of complete application image.");
    }
    return ESP_OK;
}


esp_err_t ens160_reset(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    uint8_t data = ENS160_OPMODE_RESET;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_OPMODE, &data, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    vTaskDelay(pdMS_TO_TICKS(ENS160_BOOTING_MS));

    return ESP_OK;
}

esp_err_t ens160_part_id(i2c_dev_t *dev, uint16_t *part_id)
{
    CHECK_ARG(dev);

    uint8_t data[2];

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS160_REG_PART_ID, data, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    *part_id = data[0] | ((uint16_t) data[1] << 8);

    return ESP_OK;
}


// Writes temperature and humidity in ENS210 format
static esp_err_t ens160_set_env_data_ens210(i2c_dev_t *dev, uint16_t temperature, uint16_t humidity)
{
    CHECK_ARG(dev);
    //uint16_t temp;
    uint8_t trh_in[4];

    //temp = (uint16_t)((t + 273.15f) * 64.0f);
    trh_in[0] = temperature & 0xff;
    trh_in[1] = (temperature >> 8) & 0xff;

    //temp = (uint16_t)(h * 512.0f);
    trh_in[2] = humidity & 0xff;
    trh_in[3] = (humidity >> 8) & 0xff;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS160_REG_TEMP_IN, trh_in, 4));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t ens160_set_environmental_data(i2c_dev_t *dev, float temperature, float humidity)
{
    CHECK_ARG(dev);

    uint16_t t_data = (uint16_t) ((temperature + 273.15f) * 64.0f);
    uint16_t rh_data = (uint16_t) (humidity * 512.0f);

    return ens160_set_env_data_ens210(dev, t_data, rh_data);
}

// Perform measurement and stores result in internal variables
esp_err_t
ens160_measure(i2c_dev_t *dev, bool wait_for_new, ens160_aqi_t *aqi, uint16_t *tvoc, uint16_t *eco2,
               uint16_t (*resistance)[4], uint16_t (*baseline)[4])
{
    CHECK_ARG(dev);

    uint8_t status;
    uint8_t buffer[8];

    if (wait_for_new) {
        do {
            vTaskDelay(pdMS_TO_TICKS(1));

            I2C_DEV_TAKE_MUTEX(dev);
            I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS160_REG_DATA_STATUS, &status, 1));
            I2C_DEV_GIVE_MUTEX(dev);
        } while (!IS_NEWGPR(status));
    } else {
        I2C_DEV_TAKE_MUTEX(dev);
        I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS160_REG_DATA_STATUS, &status, 1));
        I2C_DEV_GIVE_MUTEX(dev);
    }

    I2C_DEV_TAKE_MUTEX(dev);
    // Read predictions
    if (IS_NEWDAT(status)) {
        I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS160_REG_DATA_AQI, &buffer, 5));

        if (aqi) *aqi = buffer[0];
        if (tvoc) *tvoc = buffer[1] | ((uint16_t) buffer[2] << 8);
        if (eco2) *eco2 = buffer[3] | ((uint16_t) buffer[4] << 8);
    }

    // Read raw resistance values
    if (IS_NEWGPR(status) && resistance != NULL) {
        I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS160_REG_GPR_READ_0, buffer, 8));
        for (uint8_t i = 0; i < 4; i++) {
            (*resistance)[i] = (buffer[i * 2] | (buffer[i * 2 + 1] << 8));
        }
    }

    // Read baselines
    if ((IS_NEWGPR(status) || IS_NEWDAT(status)) && baseline != NULL) {
        I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS160_REG_DATA_BL, buffer, 8));
        for (uint8_t i = 0; i < 4; i++) {
            (*baseline)[i] = (buffer[i * 2] | (buffer[i * 2 + 1] << 8));
        }
    }

    I2C_DEV_GIVE_MUTEX(dev);
    return ESP_OK;
}

esp_err_t ens160_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}