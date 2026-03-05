入口：
    ESP_LOGI(TAG, "app_main start");
    esp_err_t err = dac_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dac_init failed: %s (0x%x)", esp_err_to_name(err), err);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "dac_init finished");


怎么控制电压值：
dac.h:
    #define DAC1_TARGET_VOLTAGE_MV 200
    #define DAC2_TARGET_VOLTAGE_MV 200

DAC1是cathode的输出，单位是mv，经过大约110倍放大
