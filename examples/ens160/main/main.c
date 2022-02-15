#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ens160.h>
#include <string.h>
#include <esp_log.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO GPIO_NUM_25
#define SCL_GPIO GPIO_NUM_26
#endif

static const char *TAG = "ENS160";

void ens160_test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    if (ens160_init_desc(&dev, ENS160_I2CADDR_0, I2C_NUM_0, SDA_GPIO, SCL_GPIO) == ESP_FAIL) {
        ESP_LOGE(TAG, "Failed to initialize ENS160");
    }

    ens160_aqi_t aqi;
    uint16_t tvoc, eco2;

    while (1) {
        if (ens160_measure(&dev, true, &aqi, &tvoc, &eco2) == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to load measurement data");
        } else {
            ESP_LOGI(TAG, "======================");
            ESP_LOGI(TAG, "AQI: %d", aqi);
            ESP_LOGI(TAG, "TVOC: %d ppb", tvoc);
            ESP_LOGI(TAG, "eCO2: %d ppm", eco2);
            ESP_LOGI(TAG, "NO2: %d ppb", tvoc);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(ens160_test, "ens160_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}

