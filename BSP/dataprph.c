#include "bsp.h"
#include "esp_log.h"

static const char *TAG = "DataPeripheralModule";

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
			uint8_t opcode = cmd.data[0];
			ESP_LOGI(TAG, "Received command: 0x%02X", opcode);
			if (opcode == START) { // 发送数据包
				ESP_LOGI(TAG, "Starting data transmission...");
				// 生成并发送测试数据包
				uint32_t pkgId = cmd.data[1] << 24 |
								 cmd.data[2] << 16 |
								 cmd.data[3] << 8 |
								 cmd.data[4];
				TxPkg_t TxPkg;
				GenTestPkg(TxPkg.data, cmd.data[5], pkgId); // 生成指定类型的数据包
				TxPkg.length = DATA_PACKAGE_SIZE;
				if (xQueueSend(TxQueue, &TxPkg, pdMS_TO_TICKS(100)) != pdTRUE) {
					ESP_LOGE(TAG, "Failed to enqueue TxPkg");
				}
				cmdProcessed = true;
			}
			else if (cmd.data[0] == STOP) { // 停止发送数据包
				ESP_LOGI(TAG, "Stopping data transmission...");
				cmdProcessed = true;
			}
			else if (cmd.data[0] == ACK) { // 处理ACK
				uint32_t ackedPkgId = cmd.data[1] << 24 |
									  cmd.data[2] << 16 |
									  cmd.data[3] << 8 |
									  cmd.data[4];
				ESP_LOGI(TAG, "Received ACK for package ID: %u", (unsigned) ackedPkgId);
				cmdProcessed = true;
			}
			else if (cmd.data[0] == NACK) { // 处理NACK
				uint32_t nackedPkgId = cmd.data[1] << 24 |
									   cmd.data[2] << 16 |
									   cmd.data[3] << 8 |
									   cmd.data[4];
				TxPkg_t TxPkg;
				GenTestPkg(TxPkg.data, cmd.data[5], nackedPkgId); // 重新生成指定类型的数据包
				if (xQueueSend(TxQueue, &TxPkg, pdMS_TO_TICKS(100)) != pdTRUE) {
					ESP_LOGE(TAG, "Failed to re-enqueue TxPkg for NACKed package ID: %u", (unsigned) nackedPkgId);
				} else {
					ESP_LOGI(TAG, "Re-enqueued TxPkg for NACKed package ID: %u", (unsigned) nackedPkgId);
				}
				cmdProcessed = true;
			}
			if (!cmdProcessed) {
				if (xQueueSend(DataQueue, &cmd, 0) != pdTRUE) {
					ESP_LOGE(TAG, "Failed to re-queue command.");
				}
			}
		}
	}
}