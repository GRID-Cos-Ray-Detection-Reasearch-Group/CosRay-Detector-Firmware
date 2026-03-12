/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_log.h"
#include "esp_err.h"
#include "Preamplifier.h"

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    preamplifier_config_t config;
    preamplifier_get_default_config(&config);
    config.enable_chargein = false;

    esp_err_t err = preamplifier_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "preamplifier_init failed: %s (0x%x)", esp_err_to_name(err), err);
        return;
    }

    ESP_LOGI(TAG, "Preamplifier initialized");
}
