/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "Dac.h"

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "app_main start");
    esp_err_t err = dac_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dac_init failed: %s (0x%x)", esp_err_to_name(err), err);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "dac_init finished");

    while (1)
    {
        ESP_LOGI(TAG, "Hello world");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
