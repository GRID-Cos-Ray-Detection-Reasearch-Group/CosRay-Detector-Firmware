/*
 * bsp.c
 *
 *  Created on: 2025年10月31日
 *      Author: hp
 */

#include "bsp.h"
#include "esp_log.h"

static const char *TAG = "BSPModule";


esp_err_t BSPInit(void) {
	for (size_t i = 0; i < DATA_BUFFER_SIZE; i++) {
		TxBuffer[i] = 0;
	}
	ESP_LOGI(TAG, "Initializing BlueTooth Peripheral");
	esp_err_t ESPRet = InitBlueTooth();
	if (ESPRet != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize BlueTooth Peripheral");
		return ESPRet;
	}
	ESPRet = InitDataPeripheral();
	if (ESPRet != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize Data Peripheral");
		return ESPRet;
	}
	return ESP_OK;
}
