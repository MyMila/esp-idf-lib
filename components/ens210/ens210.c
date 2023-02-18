//
// Created by Mladen Đurić - Private on 9.1.22..
//
#include <math.h>
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include "ens210.h"
#include "crc7.h"
#include "esp_crc.h"

#define I2C_FREQ_HZ 100000

#define MOLAR_MASS_OF_WATER     18.01534
#define UNIVERSAL_GAS_CONSTANT  8.21447215

// Chip constants
#define ENS210_PARTID       0x0210 // The expected part id of the ENS210
#define ENS210_BOOTING_MS   10 // Booting time in ms (also after reset, or going to high power)
#define ENS210_THCONV_SINGLE_MS 130 // Conversion time in ms for single shot T/H measurement
#define ENS210_THCONV_CONT_MS   238 // Conversion time in ms for continuous T/H measurement
#define ENS210_SOLDERING_CORRECTION (50 * 64 / 1000)

// Addresses of the ENS210 registers
#define ENS210_REG_PART_ID       0x00
#define ENS210_REG_UID           0x04
#define ENS210_REG_SYS_CTRL      0x10
#define ENS210_REG_SYS_STAT      0x11
#define ENS210_REG_SENS_RUN      0x21
#define ENS210_REG_SENS_START    0x22
#define ENS210_REG_SENS_STOP     0x23
#define ENS210_REG_SENS_STAT     0x24
#define ENS210_REG_T_VAL         0x30
#define ENS210_REG_H_VAL         0x33

#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

struct {
    uint8_t soldering_correction;
    bool single_mode;
    bool available;
} ens210 = {
        .soldering_correction = 0,
        .single_mode = true,
        .available = false,
};

static const char* TAG = "ENS210";

esp_err_t static ens210_low_power(i2c_dev_t *dev, bool enable)
{
    uint8_t data = enable ? 0x01 : 0x00;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS210_REG_SYS_CTRL, &data, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    vTaskDelay(pdMS_TO_TICKS(ENS210_BOOTING_MS));

    return ESP_OK;
}

esp_err_t ens210_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    esp_err_t res = ESP_OK;
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = ENS210_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    if (i2c_dev_create_mutex(dev) != ESP_OK) {
        return ESP_FAIL;
    }

    uint16_t part_id;
    uint64_t uid;

    esp_err_t err = ESP_OK;

    // Restart is needed as ENS210 seems to use previous baseline when starting, causing wrong values
    err = ens210_reset(dev);
    if (err != ESP_OK) {
        res = ESP_FAIL;
        goto err;
    }

    // Wait 10ms for the ENS210 to boot
    vTaskDelay(pdMS_TO_TICKS(100));

    err = ens210_version(dev, &part_id, &uid);
    if (err != ESP_OK) {
        res = ESP_FAIL;
        goto err;
    }

    // Disable low power, to run more stable
    err = ens210_low_power(dev, false);
    if (err != ESP_OK) {
        res = ESP_FAIL;
        goto err;
    }

    ens210.available = part_id == ENS210_PARTID;
err:
    if (res == ESP_FAIL) {
        i2c_dev_delete_mutex(dev);
    }
    return ESP_OK;
}

esp_err_t ens210_reset(i2c_dev_t *dev)
{
    uint8_t data = 0x80;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS210_REG_SYS_CTRL, &data, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    vTaskDelay(pdMS_TO_TICKS(ENS210_BOOTING_MS));

    return ESP_OK;
}

esp_err_t ens210_version(i2c_dev_t *dev, uint16_t *part_id, uint64_t *uid)
{
    if (ens210_low_power(dev, false) != ESP_OK) goto error;

    I2C_DEV_TAKE_MUTEX(dev);

    uint8_t data[8];
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS210_REG_PART_ID, data, 2));

    *part_id = data[1] * 256U + data[0] * 1U;

    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS210_REG_UID, data, 8));

    for (uint8_t i = 0; i < 8; i++) {
        ((uint8_t *) uid)[i] = data[i];
    }

    I2C_DEV_GIVE_MUTEX(dev);

    error:
    if (ens210_low_power(dev, true) != ESP_OK) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t ens210_single_measurement_mode(i2c_dev_t *dev, bool enable)
{
    uint8_t data = enable ? 0x00 : 0x03;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS210_REG_SENS_RUN, &data, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    ens210.single_mode = enable;

    return ESP_OK;
}

static esp_err_t extract_reading(uint32_t in, uint32_t *out)
{
    *out = (in >> 0) & 0xffff;
    bool valid = (in >> 16) & 0x1;
    uint32_t crc = (in >> 17) & 0x7f;
    uint32_t payload = (in >> 0) & 0x1ffff;
    bool crc_ok = crc7(payload) == crc;

    return crc_ok && valid ? ESP_OK : ESP_FAIL;
}

static esp_err_t read(i2c_dev_t *dev, uint32_t *t_data, uint32_t *h_data)
{
    uint8_t data[6];

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, ENS210_REG_T_VAL, data, 6));
    I2C_DEV_GIVE_MUTEX(dev);

    // Retrieve and pack bytes into t_val and h_val
    uint32_t t_val = (uint32_t) ((uint32_t) data[2] << 16 | (uint32_t) data[1] << 8 | (uint32_t) data[0]);
    uint32_t h_val = (uint32_t) ((uint32_t) data[5] << 16 | (uint32_t) data[4] << 8 | (uint32_t) data[3]);

    esp_err_t res = ESP_OK;
    res |= extract_reading(t_val, t_data);
    res |= extract_reading(h_val, h_data);

    return res;
}

static int32_t ens210_to_celsius(uint32_t t_data)
{
    // Force 32 bits
    uint32_t t = t_data & 0xFFFF;
    // Compensate for soldering effect
    t -= ens210.soldering_correction;
    // Return m*C. This equals m*(K-273.15) = m*K - 27315*m/100 = m*t/64 - 27315*m/100
    // Note m is the multiplier, C is temperature in Celsius, K is temperature in Kelvin, t is raw t_data value.
    // Uses C=K-273.15 and K=t/64.
    //return t / 64 - 27315L / 100;
    return ((int32_t)(t) << (16 - 6)) - (int32_t)(273.15*65536.0); // 16 bit fixed-point
}

static int32_t ens210_to_humidity(uint32_t h_data)
{
    // Force 32 bits
    uint32_t h = h_data & 0xFFFF;
    // Return m*H. This equals m*(h/512) = (m*h)/512
    // Note m is the multiplier, H is the relative humidity in %RH, h is raw h_data value.
    // Uses H=h/512.
    //return h / 512;
    return h << (16 - 9); // 16 bit fixed-point
}

esp_err_t ens210_measure(i2c_dev_t *dev, int32_t *temperature, int32_t *humidity)
{
    uint32_t t_val;
    uint32_t h_val;

    if (ens210.single_mode) {
        // Start a single shot measurement
        if (ens210_single_measurement_mode(dev, true) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    uint8_t data = 0x03;
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, ENS210_REG_SENS_START, &data, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    // Wait for measurement to complete
    if (ens210.single_mode) vTaskDelay(pdMS_TO_TICKS(ENS210_THCONV_SINGLE_MS));
    else vTaskDelay(pdMS_TO_TICKS(ENS210_THCONV_CONT_MS));

    // Get the measurement data
    if (read(dev, &t_val, &h_val) != ESP_OK) {
        return ESP_FAIL;
    }

    *temperature = ens210_to_celsius(t_val);
    *humidity = ens210_to_humidity(h_val);

    return ESP_OK;
}

int32_t ens210_absolute_humidity(int32_t temperature, int32_t humidity)
{
    //taken from https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
    //precision is about 0.1°C in range -30 to 35°C
    //August-Roche-Magnus   6.1094 exp(17.625 x T)/(T + 243.04)
    //Buck (1981)     6.1121 exp(17.502 x T)/(T + 240.97)
    //reference https://www.eas.ualberta.ca/jdwilson/EAS372_13/Vomel_CIRES_satvpformulae.html    // Use Buck (1981)

    //return (6.1121 * pow(2.718281828, (17.67 * temperature) / (temperature + 243.5)) * humidity * MOLAR_MASS_OF_WATER) /
    //       ((273.15 + temperature) * UNIVERSAL_GAS_CONSTANT);
    return 0;
}

void ens210_set_correction(i2c_dev_t *dev, uint8_t correction)
{
    ens210.soldering_correction = correction;
}

bool ens210_available()
{
    return ens210.available;
}

esp_err_t ens210_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    ens210.available = false;
    ens210.single_mode = false;
    ens210.soldering_correction = 0;
    return i2c_dev_delete_mutex(dev);
}