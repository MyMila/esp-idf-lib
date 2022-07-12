#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <dht.h>
#include <esp_log.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
static const gpio_num_t dht_gpio = 4;
#else
static const gpio_num_t dht_gpio = GPIO_NUM_26;
#endif

static const char *TAG = "DHT";

void dht_test(void *pvParameters)
{
    dht_handle_t handle = NULL;

    do {
        handle = dht_init(dht_gpio, DHT_TYPE_AM2301, DHT_MODE_ASYNC);
        if (handle == NULL) {
            ESP_LOGE(TAG, "dht_init: failed to init");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (handle == NULL);

    esp_err_t err;
    while (1) {
        if ((err = dht_read(handle)) == ESP_OK)
            ESP_LOGI(TAG, "dht_read: humidity: %f%% temp: %fC", dht_humidity(handle), dht_temperature(handle));
        else
            ESP_LOGE(TAG, "dht_read: error: %s", esp_err_to_name(err));

        // If you read the sensor data too often, it will heat up
        // http://www.kandrsmith.org/RJS/Misc/Hygrometers/dht_sht_how_fast.html
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main()
{
    xTaskCreate(dht_test, "dht_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}

