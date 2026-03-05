#include "bsp.h"
#include "esp_log.h"

static const char *TAG = "BSPModule";

esp_err_t BSPInit(void) {
	// 仅初始化蓝牙；数据外设（ADC/定时器）在 DataStoreTask
	// 中初始化，避免重复初始化
	ESP_LOGI(TAG, "Initializing BlueTooth");
	esp_err_t ret = InitBlueTooth();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize BlueTooth: %d", ret);
		return ret;
	}
	ESP_LOGI(TAG, "BSPInit completed");
	return ESP_OK;
}
