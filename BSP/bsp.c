/*
 * bsp.c
 *
 *  Created on: 2025年10月31日
 *      Author: hp
 */

#include "bsp.h"
#include "esp_log.h"
#include "esp_rom_crc.h" 
#include "sys/param.h"
#include "flashstorage.h"
static const char *TAG = "BSPModule";

esp_err_t BSPInit(void) {
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

esp_err_t SendFlashDataOverBLE(uint32_t start_pkg_idx, uint32_t pkg_count)
{
    if (pkg_count == 0 || start_pkg_idx >= W25N_TOTAL_PAGES * W25N_PAGE_SIZE_MAIN / DATA_PACKAGE_SIZE) {
        ESP_LOGE(TAG, "Invalid flash read params: start=%u, count=%u", start_pkg_idx, pkg_count);
        return ESP_ERR_INVALID_ARG;
    }

    // 堆分配缓冲区（每个包512字节，按单个包分配更节省内存）
    uint8_t *pkg_buf = (uint8_t *)heap_caps_malloc(DATA_PACKAGE_SIZE, MALLOC_CAP_DEFAULT);
    if (!pkg_buf) {
        ESP_LOGE(TAG, "Malloc failed for flash read buffer");
        return ESP_ERR_NO_MEM;
    }

    uint32_t sent_pkgs = 0;
    uint8_t global_total = (uint8_t)pkg_count; // 全局总包数

    // 逐包读取并发送
    for (uint32_t i = 0; i < pkg_count; i++) {
        uint32_t current_global_idx = start_pkg_idx + i;
        uint32_t read_len = 0;

        // 读取单个512字节包
        FlashStatus flash_ret = FlashReadDataPackages(current_global_idx, 1, pkg_buf, &read_len);
        if (flash_ret != FLASH_OK || read_len != DATA_PACKAGE_SIZE) {
            ESP_LOGE(TAG, "Flash read failed for pkg %u (read len: %u)", current_global_idx, read_len);
            continue;
        }

        // 调用4参数的SendNotify，参数为（数据buf, 数据长度, 全局总包数, 当前全局包号）
        int ble_ret = SendNotify(pkg_buf, DATA_PACKAGE_SIZE, global_total, (uint8_t)(i + 1));
        if (ble_ret == 0) {
            sent_pkgs++;
            ESP_LOGI(TAG, "Sent global pkg %d/%d (512 bytes)", i + 1, global_total);
        } else {
            ESP_LOGE(TAG, "BLE send failed for global pkg %d", i + 1);
        }

        // 全局包间隔延迟，避免BLE拥塞
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGI(TAG, "Sent %u/%u packages from flash (start=%u)", sent_pkgs, pkg_count, start_pkg_idx);
    free(pkg_buf);
    return sent_pkgs > 0 ? ESP_OK : ESP_FAIL;
}