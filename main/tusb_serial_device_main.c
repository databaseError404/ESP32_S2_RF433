/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"
#include "driver/rf_receiver.h"

static const char *TAG = "example";
static uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

#define RF_USB_CDC_PORT TINYUSB_CDC_ACM_0

/**
 * @brief Application Queue
 */
static QueueHandle_t app_queue;
typedef struct {
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];     // Data buffer
    size_t buf_len;                                     // Number of bytes received
    uint8_t itf;                                        // Index of CDC device interface
} app_message_t;

static const char *rf_action_to_str(uint8_t action)
{
    switch (action) {
    case RF_ACTION_START:
        return "=>";
    case RF_ACTION_CONTINUE:
        return "==";
    case RF_ACTION_STOP:
        return "<=";
    default:
        return "unknown";
    }
}

static void rf_publish_to_usb_and_log(const rf_event_t *rfcode)
{
    char line[128];
    const char *action = rf_action_to_str(rfcode->action);

    int len = snprintf(line, sizeof(line), "%s %06llX (protocol: %02hX, %u bits)\r\n",
                       action,
                       rfcode->raw_code,
                       rfcode->protocol,
                       rfcode->bits);
    if (len < 0) {
        ESP_LOGE(TAG, "Failed to format RF message");
        return;
    }

    switch (rfcode->action) {
    case RF_ACTION_START:
        ESP_LOGI(TAG, "start:    %06llX (protocol: %02hX, %u bits)", rfcode->raw_code, rfcode->protocol, rfcode->bits);
        break;
    case RF_ACTION_CONTINUE:
        ESP_LOGI(TAG, "continue: %06llX (protocol: %02hX, %u bits)", rfcode->raw_code, rfcode->protocol, rfcode->bits);
        break;
    case RF_ACTION_STOP:
        ESP_LOGI(TAG, "stop:     %06llX (protocol: %02hX, %u bits)", rfcode->raw_code, rfcode->protocol, rfcode->bits);
        break;
    default:
        ESP_LOGI(TAG, "unknown:  %06llX (protocol: %02hX, %u bits)", rfcode->raw_code, rfcode->protocol, rfcode->bits);
        break;
    }

    size_t write_len = (len < (int)sizeof(line)) ? (size_t)len : (sizeof(line) - 1);
    tinyusb_cdcacm_write_queue(RF_USB_CDC_PORT, (uint8_t *)line, write_len);
    esp_err_t err = tinyusb_cdcacm_write_flush(RF_USB_CDC_PORT, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RF CDC ACM write flush error: %s", esp_err_to_name(err));
    }
}

static void rf_task(void *args)
{
    ESP_LOGI(TAG, "RF task started");

    rf_config_t config = RF_DEFAULT_CONFIG(CONFIG_RF_RECEIVER_GPIO);
    ESP_ERROR_CHECK(rf_config(&config));
    ESP_ERROR_CHECK(rf_driver_install(0));

    QueueHandle_t events = NULL;
    ESP_ERROR_CHECK(rf_get_events_handle(&events));

    rf_event_t rfcode;
    for (;;) {
        if (xQueueReceive(events, &rfcode, portMAX_DELAY)) {
            rf_publish_to_usb_and_log(&rfcode);
        }
    }
}

/**
 * @brief CDC device RX callback
 *
 * CDC device signals, that new data were received
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {

        app_message_t tx_msg = {
            .buf_len = rx_size,
            .itf = itf,
        };

        memcpy(tx_msg.buf, rx_buf, rx_size);
        xQueueSend(app_queue, &tx_msg, 0);
    } else {
        ESP_LOGE(TAG, "Read Error");
    }
}

/**
 * @brief CDC device line change callback
 *
 * CDC device signals, that the DTR, RTS states changed
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}

void app_main(void)
{
    // Create FreeRTOS primitives
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);
    app_message_t msg;

    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = NULL,
        .hs_configuration_descriptor = NULL,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = NULL,
#endif // TUD_OPT_HIGH_SPEED
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    /* the second way to register a callback */
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));

#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_1,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));
#endif

    ESP_LOGI(TAG, "USB initialization DONE");
    xTaskCreate(rf_task, "rf_task", 4096, NULL, 10, NULL);

    while (1) {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            if (msg.buf_len) {

                /* Print received data*/
                ESP_LOGI(TAG, "Data from channel %d:", msg.itf);
                ESP_LOG_BUFFER_HEXDUMP(TAG, msg.buf, msg.buf_len, ESP_LOG_INFO);

                /* write back */
                tinyusb_cdcacm_write_queue(msg.itf, msg.buf, msg.buf_len);
                esp_err_t err = tinyusb_cdcacm_write_flush(msg.itf, 0);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "CDC ACM write flush error: %s", esp_err_to_name(err));
                }
            }
        }
    }
}
