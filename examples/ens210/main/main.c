#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ens210.h>
#include <string.h>
#include <esp_log.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO GPIO_NUM_25
#define SCL_GPIO GPIO_NUM_26
#endif

static const char *TAG = "ENS210";

void ens210_test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ens210_init_desc(&dev, I2C_NUM_0, SDA_GPIO, SCL_GPIO));

    float temperature, humidity;

    while (1) {
        if (ens210_measure(&dev, &temperature, &humidity) == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f C, Humidity: %.2f %%RH", temperature, humidity);
        } else {
            ESP_LOGE(TAG, "Failed to read sensor measurement.");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(ens210_test, "ens210_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}

