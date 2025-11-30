#include "bsp.h"
#include "esp_log.h"

static const char *TAG = "DataPeripheralModule";

// TODO：生成测试数据
static void GenTestPkg(uint8_t *TestBuffer, uint8_t PkgType, uint32_t PkgId) {
	// 根据PkgType生成不同类型的数据包
	if (PkgType == 0x01) {
		// 生成μ子数据包
		MuonDataPkg_t *pkg = (MuonDataPkg_t *)TestBuffer;
		pkg->head[0] = 0xAA;
		pkg->head[1] = 0xBB;
		pkg->head[2] = 0xCC;
		pkg->PkgCnt = PkgId;
		pkg->utc = 0;
		pkg->tail[0] = 0xDD;
		pkg->tail[1] = 0xEE;
		pkg->tail[2] = 0xFF;
		pkg->crc = CalcCRC((uint8_t *)pkg, sizeof(MuonDataPkg_t) - 2);
		ESP_LOGI(TAG, "MuonDataPkg generated with CRC: 0x%04X", pkg->crc);
	}
	else if (PkgType == 0x02) {
		// 生成时间线数据包
		TimeLinePkg_t *pkg = (TimeLinePkg_t *)TestBuffer;
		pkg->head[0] = 0x12;
		pkg->head[1] = 0x34;
		pkg->head[2] = 0x56;
		pkg->PkgCnt = PkgId;
		pkg->tail[0] = 0x78;
		pkg->tail[1] = 0x9A;
		pkg->tail[2] = 0xBC;
		pkg->crc = CalcCRC((uint8_t *)pkg, sizeof(TimeLinePkg_t) - 2);
		ESP_LOGI(TAG, "TimeLinePkg generated with CRC: 0x%04X", pkg->crc);
	}
}

esp_err_t InitDataPeripheral(void) {
	ESP_LOGI(TAG, "Initializing Data Peripheral");
	// TODO: 实现数据外设初始化逻辑
	return ESP_OK;
}

void RunDataPeripheral(void) {
	Command_t cmd;
	while (1) {
		bool cmdProcessed = false;
		if (xQueueReceive(DataQueue, &cmd, portMAX_DELAY) == pdTRUE) {
			ESP_LOGI(TAG, "Received command: 0x%02X%02X", cmd.data[0], cmd.data[1]);
			if (cmd.data[0] == 0x01) { // 发送数据包
				if (xSemaphoreTake(TxBufferMutex, 0) == pdTRUE) {
					if (xSemaphoreTake(TxBufferReadySemaphore, 0) == pdTRUE) {
						uint32_t PkgId = ((uint32_t) cmd.data[2] << 24) |
										((uint32_t) cmd.data[3] << 16) |
										((uint32_t) cmd.data[4] << 8) |
										((uint32_t) cmd.data[5]);
						GenTestPkg(TxBuffer, cmd.data[1], PkgId);
						// 获取信号量，写入TxBuffer
						xSemaphoreGive(TxBufferMutex);
						cmdProcessed = true;
						ESP_LOGI(TAG, "Test data with id %u generated and stored in TxBuffer", (unsigned) PkgId);
					}
					else xSemaphoreGive(TxBufferMutex);
				}
			}
			if (!cmdProcessed) {
				if (xQueueSend(DataQueue, &cmd, 0) != pdTRUE) {
					ESP_LOGE(TAG, "Failed to re-queue command.");
				}
			}
		}
	}
}