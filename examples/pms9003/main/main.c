//
// Created by Mladen Đurić - Private on 5.3.22..
//
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pms9003.h>
#include <string.h>
#include <esp_log.h>

static const char *TAG = "PMS9003";

void pms9003_test(void *pvParameters)
{
    pms9003_handle_t handle;

    do {
        handle = pms9003_init(UART_NUM_1, GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_NC, GPIO_NUM_NC);
        if (handle == NULL) {
            ESP_LOGE(TAG, "pms9003_init: failed to init");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (handle == NULL);

    pms9003_measurement_t measurement;
    int64_t timer;

    while (true) {
        timer = esp_timer_get_time();
        pms9003_measure(handle, &measurement, 1000);
        timer = esp_timer_get_time() - timer;

        ESP_LOGI(TAG, "PM1.0: %d", measurement.pm1_0_std);
        ESP_LOGI(TAG, "PM2.5: %d", measurement.pm2_5_std);
        ESP_LOGI(TAG, "PM10: %d", measurement.pm10_std);
        ESP_LOGI(TAG, "Time difference betweeen measures: %d ms", (int)(timer / 1000));
    }
}

void app_main()
{
    xTaskCreate(pms9003_test, "pms9003_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}
