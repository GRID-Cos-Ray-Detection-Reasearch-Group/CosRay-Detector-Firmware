#include "bsp.h"
#include "esp_log.h"

static const char *TAG = "BSPModule";

esp_err_t BSPInit(void) {
	ESP_LOGI(TAG, "Initializing BlueTooth Peripheral");
	esp_err_t ret = InitBlueTooth();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize BlueTooth Peripheral");
		return ret;
	}
	ret = InitDataPeripheral();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize Data Peripheral");
		return ret;
	}
	return ESP_OK;
}
