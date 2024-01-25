//
// Created by Mladen Đurić - Private on 5.3.22..
//

#include <pms9003.h>
#include <esp_log.h>
#include <string.h>

#define PMS_START_BYTE_1 0x42
#define PMS_START_BYTE_2 0x4d
#define PMS_CMD_LENGTH 7
#define PMS_MAX_PACKET_LENGTH 32
#define PMS_TIMEOUT_MS 3000
#define PMS_ERROR_POSITION 29

#define IOT_CHECK(tag, a, ret)  if(!(a)) {                                             \
        ESP_LOGE(tag,"%s:%d (%s)", __FILE__, __LINE__, __FUNCTION__);      \
        return (ret);                                                                   \
        }
#define POINT_ASSERT(tag, param, ret)    IOT_CHECK(tag, (param) != NULL, (ret))
#define SEMAPHORE_TAKE(device) do { \
        if (!xSemaphoreTake(device->mutex, pdMS_TO_TICKS(1000))) \
        { \
            ESP_LOGE(TAG, "could not take device mutex"); \
            return ESP_ERR_TIMEOUT; \
        } \
        } while (0)
#define SEMAPHORE_GIVE(device) do { \
        if (!xSemaphoreGive(device->mutex)) \
        { \
            ESP_LOGE(TAG, "could not give device mutex"); \
            return ESP_FAIL; \
        } \
        } while (0)

typedef struct pms7003_frame {
    uint8_t data[PMS_MAX_PACKET_LENGTH];
    uint16_t expected_length;
    uint16_t length;
} pms_frame_t;

typedef struct {
    pms_mode_t mode;
    pms_frame_t frame;

    esp_err_t (*reset_pin)(uint8_t state);

    esp_err_t (*set_pin)(uint8_t state);

    esp_err_t (*data_size)(uint8_t *size);

    esp_err_t (*read_data)(uint8_t *data, uint8_t size);

    esp_err_t (*write_data)(const uint8_t *data, uint8_t size);
    SemaphoreHandle_t mutex;
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
        [CMD_ACTIVE_MODE] = { 0x42, 0x4D, 0xe1, 0x00, 0x01, 0x01, 0x71 },
        [CMD_READ_PASSIVE_MODE] = { 0x42, 0x4D, 0xe2, 0x00, 0x00, 0x01, 0x71 },
};

static const char *TAG = "PMS9003";

/**
 * @brief Unpacks frame buffer into measurement
 *
 * @static
 * @param frame Frame to unpack.
 * @param measure Pointer to measurement structure where to store the readings.
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 */
static esp_err_t pms9003_unpack(pms_frame_t *frame, pms9003_measurement_t *measure)
{
    // error code
    if (frame->data[PMS_ERROR_POSITION] > 0) {
        ESP_LOGE(TAG, "pms9003_unpack: pms error code: 0x%02hhx", frame->data[PMS_ERROR_POSITION]);
    }

    measure->pm1_0_std = (frame->data[4] << 8) | frame->data[5];
    measure->pm2_5_std = (frame->data[6] << 8) | frame->data[7];
    measure->pm10_std = (frame->data[8] << 8) | frame->data[9];
    measure->pm1_0_atm = (frame->data[10] << 8) | frame->data[11];
    measure->pm2_5_atm = (frame->data[12] << 8) | frame->data[13];
    measure->pm10_atm = (frame->data[14] << 8) | frame->data[15];

    measure->pm0_3_par = (frame->data[16] << 8) | frame->data[17];
    measure->pm0_5_par = (frame->data[18] << 8) | frame->data[19];
    measure->pm1_0_par = (frame->data[20] << 8) | frame->data[21];
    measure->pm2_5_par = (frame->data[22] << 8) | frame->data[23];
    measure->pm5_0_par = (frame->data[24] << 8) | frame->data[25];
    measure->pm10_par = (frame->data[26] << 8) | frame->data[27];

    return ESP_OK;
}

/**
 * @brief Performs CRC check on frame
 *
 * @static
 * @param frame Pointer to frame struct to perform CRC check on.
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 */
static esp_err_t pms9003_crc_check(pms_frame_t *frame)
{
    if (frame->length > PMS_MAX_PACKET_LENGTH) return ESP_FAIL;

    uint32_t calc_checksum = 0;
    for (uint16_t i = 0; i < frame->length - 2; i++)
        calc_checksum += frame->data[i];

    uint16_t checksum = (uint16_t) frame->data[frame->length - 2] << 8 | frame->data[frame->length - 1];
    if (calc_checksum != checksum) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Performs read until frame is obtained
 *
 * Blocking function that waits PMS_TIMEOUT_MS milliseconds until timing out.
 * Frame can be their ACK for sent command or a measurement.
 *
 * @static
 * @param handle PMS driver handler.
 * @param size Expected frame size.
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 *      - ESP_ERR_INVALID_CRC
 *      - ESP_ERR_TIMEOUT
 *      - ESP_ERR_INVALID_ARG
 */
static esp_err_t pms9003_read_frame(pms9003_handle_t handle, uint8_t size)
{
    esp_err_t status = ESP_FAIL;

    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    IOT_CHECK(TAG, size < PMS_MAX_PACKET_LENGTH, ESP_ERR_INVALID_ARG);

    pms_device_t *device = (pms_device_t *) handle;
    pms_frame_t *frame = &device->frame;

    uint8_t byte;
    uint16_t expected_length = 0;

    // Reset frame before performing another read
    frame->length = 0;
    frame->expected_length = 0;
    memset(frame->data, 0, sizeof(frame->data));

    TickType_t entry_time = xTaskGetTickCount();
    do {
        status = device->data_size(&byte);
        if (status != ESP_OK || byte < 1) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        status = device->read_data(&byte, 1);
        if (status != ESP_OK) {
            return status;
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
                if (expected_length != size) {
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

    if (pms9003_crc_check(frame) != ESP_OK) {
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

/**
 * @brief Method for waiting on ACK after command is sent
 *
 * Blocking function that will try to read ACK until it times out.
 *
 * @static
 * @param handle PMS driver handler.
 * @param cmd Command for the check to be performed against.
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 *      - ESP_ERR_INVALID_CRC
 *      - ESP_ERR_TIMEOUT
 *      - ESP_ERR_INVALID_ARG
 */
static esp_err_t pms9033_wait_ack(pms9003_handle_t handle, uint8_t cmd)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;

    esp_err_t status = pms9003_read_frame(handle, 4);
    if (status != ESP_OK) {
        return status;
    }

    if (device->frame.data[4] != cmd) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Method tries to read measurement from PMS
 *
 * Blocking function that will try to read ACK until it times out.
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
 *      - ESP_ERR_INVALID_ARG
 */
static esp_err_t pms9003_read_measurement(pms9003_handle_t handle, pms9003_measurement_t *measure)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;

    esp_err_t status = pms9003_read_frame(handle, 28);
    if (status != ESP_OK) {
        return status;
    }

    if (device->frame.length != 32) {
        return ESP_FAIL;
    }

    return pms9003_unpack(&device->frame, measure);
}

/**
 * @brief Send command to PMS
 *
 * @static
 * @param handle PMS driver handler.
 * @param wait_ack Wait for PMS response.
 * @param cmd Command to send.
 * @param size Size of command.
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 *      - ESP_ERR_INVALID_CRC
 *      - ESP_ERR_TIMEOUT
 *      - ESP_ERR_INVALID_ARG
 */
static esp_err_t pms9003_do_cmd(pms9003_handle_t handle, bool wait_ack, const uint8_t *cmd, size_t size)
{
    esp_err_t status = ESP_FAIL;

    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);

    pms_device_t *device = (pms_device_t *) handle;

    status = device->write_data(cmd, size);

    if (status == ESP_OK && wait_ack) {
        return pms9033_wait_ack(handle, cmd[2]);
    }

    return status;
}

/**
 * @brief Initializes PMS driver
 *
 * @param reset_pin callback for reset pin
 * @param set_pin callback for set pin
 * @param data_size callback for rx data size
 * @param read_data callback for reading rx data
 *
 * @return
 *      - pms9003_handle_t  success
 *      - NULL              error occurred
 */
pms9003_handle_t
pms9003_init(esp_err_t (*reset_pin)(uint8_t state), esp_err_t (*set_pin)(uint8_t state),
             esp_err_t (*data_size)(uint8_t *size), esp_err_t (*read_data)(uint8_t *data, uint8_t size),
             esp_err_t (*write_data)(const uint8_t *data, uint8_t size))
{
    IOT_CHECK(TAG, reset_pin != NULL, NULL);
    IOT_CHECK(TAG, set_pin != NULL, NULL);
    IOT_CHECK(TAG, data_size != NULL, NULL);
    IOT_CHECK(TAG, read_data != NULL, NULL);
    IOT_CHECK(TAG, write_data != NULL, NULL);

    esp_err_t status = ESP_OK;

    pms_device_t *device = (pms_device_t *) calloc(1, sizeof(pms_device_t));
    POINT_ASSERT(TAG, device, NULL);

    device->mutex = xSemaphoreCreateMutex();
    if (!device->mutex) {
        ESP_LOGE(TAG, "pms9003_init: could not create device mutex");
        goto err;
    }

    device->reset_pin = reset_pin;
    device->set_pin = set_pin;
    device->write_data = write_data;
    device->read_data = read_data;
    device->data_size = data_size;

    status |= pms9003_reset(device);

    vTaskDelay(pdMS_TO_TICKS(1000));
    // Wake up sensor if it was previously in sleep mode for any reason
    status |= pms9003_sleep(device, false);
    // Some time for sensor to react and start responding
    // If delay is too small PMS won't send ACK for mode command
    vTaskDelay(pdMS_TO_TICKS(3000));
    // Force active mode as a start-up mode
    status |= pms9003_set_mode(device, PMS_MODE_ACTIVE);

    if (status != ESP_OK) goto err;

    return (pms9003_handle_t) device;

    err:
    pms9003_free(device);
    return NULL;
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
 *      - ESP_ERR_TIMEOUT
 */
esp_err_t pms9003_set_mode(pms9003_handle_t handle, pms_mode_t mode)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;

    SEMAPHORE_TAKE(device);

    pms_cmd_t cmd = mode == PMS_MODE_ACTIVE ? CMD_ACTIVE_MODE : CMD_PASSIVE_MODE;
    esp_err_t status = pms9003_do_cmd(handle, true, pms_command_table[cmd], sizeof(pms_command_table[cmd]));

    if (status == ESP_OK) {
        device->mode = mode;
    }

    SEMAPHORE_GIVE(device);

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
 *      - ESP_ERR_TIMEOUT
 */
esp_err_t pms9003_sleep(pms9003_handle_t handle, bool sleep)
{
    esp_err_t status = ESP_OK;

    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);

    pms_device_t *device = (pms_device_t *) handle;

    device->set_pin(!sleep);

    SEMAPHORE_TAKE(device);

    pms_cmd_t cmd = sleep ? CMD_SLEEP : CMD_WAKEUP;
    // For some reason PMS only replies with ACK when sensor is put into sleep
    // Therefore we will only wait for it on that command
    bool wait_ack = sleep ? true : false;
    status = pms9003_do_cmd(handle, wait_ack, pms_command_table[cmd], sizeof(pms_command_table[cmd]));

    SEMAPHORE_GIVE(device);

    return status;
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
    esp_err_t status = ESP_ERR_NOT_SUPPORTED;

    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);

    pms_device_t *device = (pms_device_t *) handle;

    status = device->reset_pin(0);
    vTaskDelay(pdMS_TO_TICKS(100));
    status = device->reset_pin(1);

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

    vSemaphoreDelete(device->mutex);
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

    return pms9003_do_cmd(handle, false, pms_command_table[CMD_READ_PASSIVE_MODE],
                          sizeof(pms_command_table[CMD_READ_PASSIVE_MODE]));
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
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 *      - ESP_ERR_INVALID_CRC
 *      - ESP_ERR_TIMEOUT
 *      - ESP_ERR_INVALID_ARG
 */
esp_err_t pms9003_measure(pms9003_handle_t handle, pms9003_measurement_t *measure, uint16_t timeout_ms)
{
    POINT_ASSERT(TAG, handle, ESP_ERR_INVALID_ARG);
    pms_device_t *device = (pms_device_t *) handle;

    esp_err_t status = ESP_FAIL;

    SEMAPHORE_TAKE(device);

    if (device->mode == PMS_MODE_PASSIVE) {
        status = pms9003_passive_read(handle);
        if (status != ESP_OK) {
            ESP_LOGE(TAG, "pms9003_passive_read: %s", esp_err_to_name(status));
            return ESP_FAIL;
        }
    }

    status = pms9003_read_measurement(handle, measure);

    SEMAPHORE_GIVE(device);

    if (device->mode == PMS_MODE_PASSIVE || status != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
    }

    return status;
}

/**
 * @brief Returns error code PMS reported in measurement payload
 *
 * @param handle PMS driver handler.
 *
 * @return
 *      - PMS_STATUS_OK
 *      - PMS_STATUS_TOO_MANY_FAN_RESTARTS
 *      - PMS_STATUS_FAN_SPEED_LOW
 *      - PMS_STATUS_ENVIRONMENTAL_INTERFERENCE
 */
pms_status_t pms9003_status(pms9003_handle_t handle)
{
    POINT_ASSERT(TAG, handle, PMS_STATUS_FAIL);
    pms_device_t *device = (pms_device_t *) handle;

    // If current packet is not measurement payload (most likely ACK), we assume everything is OK
    if (device->frame.length != PMS_MAX_PACKET_LENGTH) {
        return PMS_STATUS_OK;
    }

    return device->frame.data[PMS_ERROR_POSITION];
}