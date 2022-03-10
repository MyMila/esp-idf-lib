//
// Created by Mladen Đurić - Private on 5.3.22..
//

#include <pms9003.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <machine/endian.h>

#define PMS_START_BYTE_1 0x42
#define PMS_START_BYTE_2 0x4d
#define PMS_CMD_LENGTH 7
#define PMS_MAX_PACKET_LENGTH 32
#define PMS_TIMEOUT_MS 3000

#define IOT_CHECK(tag, a, ret)  if(!(a)) {                                             \
        ESP_LOGE(tag,"%s:%d (%s)", __FILE__, __LINE__, __FUNCTION__);      \
        return (ret);                                                                   \
        }
#define POINT_ASSERT(tag, param, ret)    IOT_CHECK(tag, (param) != NULL, (ret))

typedef struct pms7003_frame {
    uint8_t data[PMS_MAX_PACKET_LENGTH];
    uint16_t expected_length;
    uint16_t length;
} pms_frame_t;

typedef struct {
    uart_port_t port;
    pms_mode_t mode;
    gpio_num_t set_pin;
    gpio_num_t reset_pin;
    pms_frame_t frame;
} pms_device_t;

typedef enum pms_cmd {
    CMD_SLEEP,
    CMD_WAKEUP,
    CMD_PASSIVE_MODE,
    CMD_ACTIVE_MODE,
    CMD_READ_PASSIVE_MODE,
} pms_cmd_t;

/*
 * Commands have following format:
 *
 * +------+------+-----+------+-----+-----------+-----------+
 * | 0x42 | 0x4d | cmd | 0x00 | arg | cksum msb | cksum lsb |
 * +------+------+-----+------+-----+-----------+-----------+
 */
static const uint8_t pms_command_table[][PMS_CMD_LENGTH] = {
        [CMD_SLEEP] = { 0x42, 0x4d, 0xe4, 0x00, 0x00, 0x01, 0x73 },
        [CMD_WAKEUP] = { 0x42, 0x4d, 0xe4, 0x00, 0x01, 0x01, 0x74 },
        [CMD_PASSIVE_MODE] = { 0x42, 0x4d, 0xe1, 0x00, 0x00, 0x01, 0x70 },
        [CMD_ACTIVE_MODE] = { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 },
        [CMD_READ_PASSIVE_MODE] = { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 },
};

static const char *TAG = "PMS9003";


/**
 * @brief Initializes PMS driver
 *
 * @param port uart port
 * @param rx_pin rx gpio pin
 * @param tx_pin tx gpio pin
 * @param set_pin set gpio pin (GPIO_NUM_NC if not used)
 * @param reset_pin reset gpio pin (GPIO_NUM_NC if not used)
 *
 * @return
 *      - pms9003_handle_t  success
 *      - NULL              error occurred
 */
pms9003_handle_t
pms9003_init(uart_port_t port, gpio_num_t rx_pin, gpio_num_t tx_pin, gpio_num_t set_pin, gpio_num_t reset_pin)
{
    IOT_CHECK(TAG, port < UART_NUM_MAX, NULL);
    IOT_CHECK(TAG, rx_pin < GPIO_NUM_MAX, NULL);
    IOT_CHECK(TAG, tx_pin < GPIO_NUM_MAX, NULL);
    IOT_CHECK(TAG, set_pin < GPIO_NUM_MAX, NULL);
    IOT_CHECK(TAG, reset_pin < GPIO_NUM_MAX, NULL);

    pms_device_t *device = (pms_device_t *) calloc(1, sizeof(pms_device_t));
    POINT_ASSERT(TAG, device, NULL);

    device->reset_pin = reset_pin;
    device->set_pin = set_pin;
    device->port = port;
    device->mode = PMS_MODE_ACTIVE;

    uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    esp_err_t status = ESP_OK;
    status |= uart_param_config(device->port, &uart_config);
    status |= uart_set_pin(device->port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    status |= uart_driver_install(device->port, PMS_MAX_PACKET_LENGTH * 8, 0, 0, NULL, 0);

    IOT_CHECK(TAG, status == ESP_OK, NULL);

    if (device->set_pin > GPIO_NUM_NC) {
        gpio_set_direction(device->set_pin, GPIO_MODE_OUTPUT);
        gpio_set_intr_type(device->set_pin, GPIO_INTR_DISABLE);
    }

    pms9003_sleep(device, false); // wake up sensor

    if (device->reset_pin > GPIO_NUM_NC) {
        gpio_set_direction(device->reset_pin, GPIO_MODE_OUTPUT);
        gpio_set_intr_type(device->reset_pin, GPIO_INTR_DISABLE);
        gpio_set_level(device->reset_pin, 1);
    }

    return (pms9003_handle_t) device;
}

/**
 * @brief Set PMS measurement mode
 *
 * Modes available:
 *  - PMS_MODE_ACTIVE
 *  - PMS_MODE_PASSIVE
 *
 * @param handle PMS driver handler.
 * @param mode PMS mode.
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 *      - ESP_ERR_INVALID_ARG
 */
esp_err_t pms9003_set_mode(pms9003_handle_t handle, pms_mode_t mode)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;

    pms_cmd_t cmd = mode == PMS_MODE_ACTIVE ? CMD_ACTIVE_MODE : CMD_PASSIVE_MODE;
    int size = uart_write_bytes(device->port, pms_command_table[cmd], sizeof(pms_command_table[cmd]));
    esp_err_t status = size > 0 ? ESP_OK : ESP_FAIL;

    if (status == ESP_OK) {
        device->mode = mode;
    }

    return status;
}

/**
 * @brief Makes PMS awake or got into sleep
 *
 * @param handle PMS driver handler.
 * @param sleep Set to true for sensor to go to sleep, otherwise to false.
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 *      - ESP_ERR_INVALID_ARG
 */
esp_err_t pms9003_sleep(pms9003_handle_t handle, bool sleep)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;

    if (device->set_pin > GPIO_NUM_NC) {
        return gpio_set_level(device->set_pin, !sleep);
    }

    pms_cmd_t cmd = sleep ? CMD_SLEEP : CMD_WAKEUP;
    int size = uart_write_bytes(device->port, pms_command_table[cmd], sizeof(pms_command_table[cmd]));
    return size > 0 ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Resets PMS
 *
 * @note Available only if reset pin is defined.
 *
 * @param handle PMS driver handler.
 *
 * @return
 *      - ESP_OK
 *      - ESP_ERR_INVALID_ARG
 *      - ESP_ERR_NOT_SUPPORTED
 */
esp_err_t pms9003_reset(pms9003_handle_t handle)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;

    esp_err_t status = ESP_ERR_NOT_SUPPORTED;
    if (device->reset_pin > GPIO_NUM_NC) {
        status |= gpio_set_level(device->reset_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        status |= gpio_set_level(device->reset_pin, 1);
    }


    return status;
}

/**
 * @brief Free device handler
 *
 * @param handle PMS driver handler.
 *
 * @return
 *      - ESP_OK
 *      - ESP_ERR_INVALID_ARG
 */
esp_err_t pms9003_free(pms9003_handle_t handle)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;

    uart_driver_delete(device->port);
    free(device);

    return ESP_OK;
}

/**
 * @brief Sends command for passive read
 *
 * @static
 * @param handle PMS driver handler.
 *
 * @return
 *      - ESP_OK
 *      - ESP_ERR_INVALID_ARG
 *      - ESP_ERR_NOT_SUPPORTED
 */
static esp_err_t pms9003_passive_read(pms9003_handle_t handle)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;

    if (device->mode != PMS_MODE_PASSIVE) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    int size = uart_write_bytes(device->port, pms_command_table[CMD_READ_PASSIVE_MODE],
                                sizeof(pms_command_table[CMD_READ_PASSIVE_MODE]));
    return size > 0 ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Unpacks frame buffer into measurement
 *
 * @static
 * @param frame Frame to unpack.
 * @param measure Pointer to measurement structure where to store the readings.
 *
 * @return
 *      - ESP_OK
 *      - ESP_ERR_INVALID_CRC
 *      - ESP_FAIL
 */
static esp_err_t pms9003_unpack(pms_frame_t* frame, pms9003_measurement_t* measure)
{
    measure->pm1_0_std = (frame->data[4] << 8) | frame->data[5];
    measure->pm2_5_std = (frame->data[6] << 8) | frame->data[7];
    measure->pm10_std = (frame->data[8] << 8) | frame->data[9];
    measure->pm1_0_atm = (frame->data[10] << 8) | frame->data[11];
    measure->pm2_5_atm = (frame->data[12] << 8) | frame->data[13];
    measure->pm10_atm =  (frame->data[14] << 8) | frame->data[15];

    measure->pm0_3_par = (frame->data[16] << 8) | frame->data[17];
    measure->pm0_5_par = (frame->data[18] << 8) | frame->data[19];
    measure->pm1_0_par = (frame->data[20] << 8) | frame->data[21];
    measure->pm2_5_par = (frame->data[22] << 8) | frame->data[23];
    measure->pm5_0_par = (frame->data[24] << 8) | frame->data[25];
    measure->pm10_par = (frame->data[26] << 8) | frame->data[27];

    uint32_t calc_checksum = 0;
    for (uint8_t i = 0; i < 30; i++)
        calc_checksum += frame->data[i];

    uint16_t checksum = (uint16_t)frame->data[30] << 8 | frame->data[31];
    if (calc_checksum != checksum) {
        return ESP_ERR_INVALID_CRC;
    }

    // error code
    if (frame->data[29] > 0) {
        ESP_LOGE(TAG, "pms9003_unpack: pms error code: 0x%02hhx", frame->data[29]);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Performs UART read until frame is obtained
 *
 * Blocking function that waits PMS_TIMEOUT_MS milliseconds until timing out.
 *
 * Frame length: 2x13 + 2
 *
 * @static
 * @param handle PMS driver handler.
 * @param measure Pointer to measurement structure where to store the readings.
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 *      - ESP_ERR_INVALID_CRC
 *      - ESP_ERR_TIMEOUT
 */
static esp_err_t pms9003_read_frame(pms9003_handle_t handle, pms9003_measurement_t* measure)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;
    pms_frame_t *frame = &device->frame;

    uint8_t byte;
    uint16_t expected_length = 0;

    TickType_t entry_time = xTaskGetTickCount();
    do {
        int status = uart_read_bytes(device->port, &byte, 1, pdMS_TO_TICKS(PMS_TIMEOUT_MS));
        if (status < 1) {
            return ESP_FAIL;
        }

        switch (frame->length) {
            case 0:
                if (byte != PMS_START_BYTE_1) {
                    frame->length = 0;
                    continue;
                }
                expected_length = 0;
                break;
            case 1:
                if (byte != PMS_START_BYTE_2) {
                    frame->length = 0;
                    continue;
                }
                break;
            case 2:
                expected_length = (uint16_t) byte << 8;
                break;
            case 3:
                expected_length |= byte;
                // We're discarding anything that's not measurement packet, including responses for commands
                if (expected_length != PMS_MAX_PACKET_LENGTH - 4) {
                    frame->length = 0;
                    continue;
                }

                frame->expected_length = expected_length;
            break;
        }

        frame->data[frame->length++] = byte;
    } while (((frame->length < (frame->expected_length + 4)) || frame->expected_length == 0) &&
             ((xTaskGetTickCount() - entry_time) < pdMS_TO_TICKS(PMS_TIMEOUT_MS)));

    if ((xTaskGetTickCount() - entry_time) >= pdMS_TO_TICKS(PMS_TIMEOUT_MS)) {
        return ESP_ERR_TIMEOUT;
    }

    if ((frame->length != frame->expected_length + 4) || frame->expected_length == 0) {
        return ESP_FAIL;
    }

    return pms9003_unpack(frame, measure);
}

/**
 * @brief Performs single measurement from PMS
 *
 * Blocking function that waits based on the mode sensor is currently running.
 * While in active mode, sensor can send data in two intervals, 2.3s and 200~800ms based on the concentration.
 * Higher the concentration the shorter the interval.
 *
 * While in passive mode, user defines interval based on `timeout_ms` param.
 *
 * @param handle PMS driver handler.
 * @param measure Pointer to measurement structure where to store the readings.
 * @param timeout_ms Delay if in passive mode or reading was unsuccessful.
 * @return
 */
esp_err_t pms9003_measure(pms9003_handle_t handle, pms9003_measurement_t *measure, uint16_t timeout_ms)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;

    esp_err_t status = ESP_FAIL;

    if (device->mode == PMS_MODE_PASSIVE) {
        status = pms9003_passive_read(handle);
        if (status != ESP_OK) {
            return ESP_FAIL;
        }
    }

    status = pms9003_read_frame(handle, measure);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "pms9003_read_frame: error: %s", esp_err_to_name(status));
    }

    if (device->mode == PMS_MODE_PASSIVE || status != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
    }

    return status;
}