#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "bsp.h"

static const char *TAG = "NimBLEModule";
static uint16_t ConnHandle = BLE_HS_CONN_HANDLE_NONE;
static bool BLEConnected = false;

// GAP事件处理
static int GAPEventCallback(struct ble_gap_event *Event, void *Arg);

// 特征访问回调
static int GenericCharacteristicAccessCallback(uint16_t ConnHandle, uint16_t attr_handle,
							  struct ble_gatt_access_ctxt *ctxt, void *arg);

// 自定义服务UUID
static const ble_uuid128_t MuonServiceUUID = {
	.u = {.type = BLE_UUID_TYPE_128},
	.value = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x02, 0x03,
			  0x04, 0x05, 0x06, 0x07, 0x08}};

// 数据特征UUID
static const ble_uuid128_t DataCharUUID = {
	.u = {.type = BLE_UUID_TYPE_128},
	.value = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x02, 0x03,
			  0x04, 0x05, 0x06, 0x07, 0x09}};

// 命令特征UUID
static const ble_uuid128_t CMDCharUUID = {
	.u = {.type = BLE_UUID_TYPE_128},
	.value = {0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x02, 0x03, 0x04,
			  0x05, 0x06, 0x07, 0x09, 0x01}};

// 特征句柄
static uint16_t DataCharValHandle;
static uint16_t CMDCharValHandle;

// GATT特征定义
static const struct ble_gatt_chr_def DataCharDef = {
	.uuid = &DataCharUUID.u,
	.access_cb = GenericCharacteristicAccessCallback,
	.flags = BLE_GATT_CHR_F_READ,
	.val_handle = &DataCharValHandle,
};
static const struct ble_gatt_chr_def CMDCharDef = {
	.uuid = &CMDCharUUID.u,
	.access_cb = GenericCharacteristicAccessCallback,
	.flags = BLE_GATT_CHR_F_WRITE,
	.val_handle = &CMDCharValHandle,
};

// GATT服务定义
static const struct ble_gatt_svc_def GATTServerServices[] = {
	{.type = BLE_GATT_SVC_TYPE_PRIMARY,
	 .uuid = &MuonServiceUUID.u,
	 .characteristics =
		 (struct ble_gatt_chr_def[]){DataCharDef, CMDCharDef, {0}}},
	{0}};

// 开始广播
static void StartAdvertising(void) {
	struct ble_gap_adv_params AdvParams;
	struct ble_hs_adv_fields Fields;
	int rc;

	// 设置广播数据
	memset(&Fields, 0, sizeof(Fields));
	Fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	Fields.tx_pwr_lvl_is_present = 1;
	Fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

	// 设备名称
	Fields.name = (uint8_t *)"MuonDetector";
	Fields.name_len = strlen("MuonDetector");
	Fields.name_is_complete = 1;

	rc = ble_gap_adv_set_fields(&Fields);
	if (rc != 0) {
		ESP_LOGE(TAG, "Error setting adv data: %d", rc);
		return;
	}

	// 开始广播
	memset(&AdvParams, 0, sizeof(AdvParams));
	AdvParams.conn_mode = BLE_GAP_CONN_MODE_UND;
	AdvParams.disc_mode = BLE_GAP_DISC_MODE_GEN;

	rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
						   &AdvParams, GAPEventCallback, NULL);
	if (rc != 0) {
		ESP_LOGE(TAG, "Error enabling advertising: %d", rc);
	}
}

// GATT 服务初始化
static int InitGATTServer(void) {
	int rc;
	rc = ble_gatts_count_cfg(GATTServerServices);
	if (rc) return rc;
	rc = ble_gatts_add_svcs(GATTServerServices);
	if (rc) return rc;
	return 0;
}

// BLE 同步回调
static void OnSyncCallback(void) {
	int rc;
	// 设置设备名称
	rc = ble_svc_gap_device_name_set("MuonDetector");
	if (rc != 0) {
		ESP_LOGE(TAG, "Error setting device name: %d", rc);
		return;
	}
	// 开始广播
	StartAdvertising();

	ESP_LOGI(TAG, "BLE initialized and advertising");
}

esp_err_t InitBlueTooth(void) {
	// 初始化 NVS Flash
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
		ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// 初始化 NimBLE
	ret = nimble_port_init();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize NimBLE: %d", ret);
		return ret;
	}

	// 初始化 GATT 服务器
	ble_hs_cfg.sync_cb = OnSyncCallback;
	int rc = InitGATTServer();
	if (rc != 0) {
		ESP_LOGE(TAG, "Error initializing GATT server: %d", rc);
		return ESP_FAIL;
	}
	return ESP_OK;
}

// GAP事件处理
static int GAPEventCallback(struct ble_gap_event *Event, void *Arg) {
	switch (Event->type) {
	case BLE_GAP_EVENT_CONNECT:
		ESP_LOGI(TAG, "Device connected");
		ConnHandle = Event->connect.conn_handle;
		BLEConnected = true;
		break;
	case BLE_GAP_EVENT_DISCONNECT:
		ESP_LOGI(TAG, "Device disconnected");
		ConnHandle = BLE_HS_CONN_HANDLE_NONE;
		BLEConnected = false;
		StartAdvertising();
		break;
	default:
		break;
	}
	return 0;
}

// 特征访问回调
static int GenericCharacteristicAccessCallback(uint16_t ConnHandle, uint16_t attr_handle,
							  struct ble_gatt_access_ctxt *ctxt, void *arg) {
	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		// 获取信号量，检查TxBuffer是否正在被写入
		if (xSemaphoreTake(TxBufferMutex, 0) == pdTRUE) {
			if (xSemaphoreTake(TxBufferReadySemaphore, 0) == pdTRUE) {
				// TxBuffer没有有效数据，传递空包
				xSemaphoreGive(TxBufferReadySemaphore);
				xSemaphoreGive(TxBufferMutex);
			}
			else {
				// 否则正常发送数据
				int rc = os_mbuf_append(ctxt->om, TxBuffer, DATA_BUFFER_SIZE);
				for (size_t i = 0; i < DATA_BUFFER_SIZE; i++) {
					ESP_LOGI(TAG, "TxBuffer[%u] = 0x%02X", (unsigned) i, TxBuffer[i]);
					TxBuffer[i] = 0;
				}
				xSemaphoreGive(TxBufferMutex);
				xSemaphoreGive(TxBufferReadySemaphore);
				return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
			}
		} else {
			// TxBuffer正在被写入，传递空包
		}

		NullPkg_t nullPkg = CreateNullPkg();
		int rc = os_mbuf_append(ctxt->om, (uint8_t *) &nullPkg, sizeof(NullPkg_t));
		return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
	} else if (ctxt->op) {
		// 复制数据到RxBuffer
		size_t len = OS_MBUF_PKTLEN(ctxt->om);
		if (len != sizeof(CommandPkg_t)) {
			ESP_LOGE(TAG, "Invalid command length: %d", len);
			return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
		}
		CommandPkg_t cmdPkg;
		int rc = os_mbuf_copydata(ctxt->om, 0, len, (uint8_t*) &cmdPkg);
		if (rc) {
			return BLE_ATT_ERR_UNLIKELY;
		}
		// 校验CRC
		uint16_t calcCrc = CalcCRC((uint8_t *)&cmdPkg.cmd, sizeof(Command_t) - 2);
		if (calcCrc != cmdPkg.crc) {
			ESP_LOGE(TAG, "Invalid command CRC: received 0x%04X, calculated 0x%04X",
					 cmdPkg.crc, calcCrc);
			return BLE_ATT_ERR_UNLIKELY;
		}

		// 将命令放入队列
		if (xQueueSend(CommandQueue, &cmdPkg.cmd, 0) != pdTRUE) {
			ESP_LOGE(TAG, "Failed to enqueue command message");
			return BLE_ATT_ERR_UNLIKELY;
		}
		return 0;
	}
	return 0;
}

void RunBlueToothHost(void) {
	ESP_LOGI(TAG, "BLE Host Started");
	nimble_port_run();
	ESP_LOGI(TAG, "BLE Host Ended");
	nimble_port_freertos_deinit();
	ESP_LOGI(TAG, "NimBLE Port Deinitialized");
}