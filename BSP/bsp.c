/*
 * bsp.c
 *
 *  Created on: 2025年10月31日
 *      Author: hp
 */

#include "bsp.h"
#include "esp_log.h"

static const char *TAG = "BSPModule";

uint16_t CalcCRC(const uint8_t *data, size_t length) {
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < length; i++) {
		crc ^= (uint16_t) data[i] << 8;
		for (int j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

esp_err_t BSPInit(void) {
	for (size_t i = 0; i < CMD_BUFFER_SIZE; i++) {
		RxBuffer[i] = 0;
	}
	for (size_t i = 0; i < DATA_BUFFER_SIZE; i++) {
		TxBuffer[i] = 0;
	}
	ESP_LOGI(TAG, "Initializing BlueTooth Peripheral");
	esp_err_t ESPRet = InitBlueTooth();
	if (ESPRet != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize BlueTooth Peripheral");
		return ESPRet;
	}
	return ESP_OK;
}
