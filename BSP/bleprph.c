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
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_rom_crc.h"  // ESP32 ROM中定义了MIN/MAX宏
#include "sys/param.h"     // FreeRTOS/BSD风格的MIN/MAX宏

#include "bsp.h"
#include "typedefs.h"

// BLE分包配置
#define BLE_PACKET_MAX_SIZE 22    // 单包总长度固定22字节
#define BLE_PACKET_HEADER_SIZE 2  // 包总数(1)+包号(1)
#define BLE_PACKET_DATA_SIZE (BLE_PACKET_MAX_SIZE - BLE_PACKET_HEADER_SIZE) // 20数据区字节数
static SemaphoreHandle_t ble_tx_mutex = NULL;

/*====解析方式====*/
/*每包 22 字节(MTU23)；
第 1 字节：总包数（固定 26）；
第 2 字节：当前包号（1~26）；
第 3~22 字节：数据区（20 字节），最后一包的后 8 字节是补 0；
拼接所有 26 包的 Data 区后，截取前 512 字节即为原始数据。
*/



static const char *TAG = "NimBLEModule";
static uint16_t ConnHandle = BLE_HS_CONN_HANDLE_NONE;
static bool BLEConnected = false;

extern QueueHandle_t CommandQueue;

// GAP事件处理
static int GAPEventCallback(struct ble_gap_event *Event, void *Arg);

// 特征访问回调
static int GATTControlCharAccessCallback(uint16_t ConnHandle,
										 uint16_t attr_handle,
										 struct ble_gatt_access_ctxt *ctxt,
										 void *arg);
static int GATTDataCharAccessCallback(uint16_t ConnHandle, uint16_t attr_handle,
									  struct ble_gatt_access_ctxt *ctxt,
									  void *arg);

// UUID
static const ble_uuid128_t MuonServiceUUID = {
	.u = {.type = BLE_UUID_TYPE_128},
	.value = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x02, 0x03,
			  0x04, 0x05, 0x06, 0x07, 0x08}};
static const ble_uuid128_t ControlCharUUID = {
	.u = {.type = BLE_UUID_TYPE_128},
	.value = {0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x02, 0x03, 0x04,
			  0x05, 0x06, 0x07, 0x09, 0x01}};
static const ble_uuid128_t DataCharUUID = {
	.u = {.type = BLE_UUID_TYPE_128},
	.value = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x02, 0x03,
			  0x04, 0x05, 0x06, 0x07, 0x09}};

// 特征句柄
static uint16_t DataCharValHandle;
static uint16_t ControlCharValHandle;

// GATT服务定义
static const struct ble_gatt_svc_def GATTServerServices[] = {
	{.type = BLE_GATT_SVC_TYPE_PRIMARY,
	 .uuid = &MuonServiceUUID.u,
	 .characteristics =
		 (struct ble_gatt_chr_def[]){
			 {
				 .uuid = &ControlCharUUID.u,
				 .access_cb = GATTControlCharAccessCallback,
				 .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
				 .val_handle = &ControlCharValHandle,
			 },
			 {
				 .uuid = &DataCharUUID.u,
				 .access_cb = GATTDataCharAccessCallback,
				 .flags = BLE_GATT_CHR_F_NOTIFY,
				 .val_handle = &DataCharValHandle,
			 },
			 {0}}},
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
	if (rc)
		return rc;
	ESP_LOGI(TAG, "GATT services counted");
	rc = ble_gatts_add_svcs(GATTServerServices);
	if (rc)
		return rc;
	ESP_LOGI(TAG, "GATT services added");
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
	esp_log_level_set("NimBLEModule", ESP_LOG_INFO);
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
	ret = InitGATTServer();
	if (ret != 0) {
		ESP_LOGE(TAG, "Error initializing GATT server: %d", ret);
		return ret;
	}
	ble_tx_mutex = xSemaphoreCreateMutex();
    if (ble_tx_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create BLE TX mutex");
        return ESP_FAIL;
    }
	ble_hs_cfg.sync_cb = OnSyncCallback;
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
		Command_t stop;
		stop.data[0] = STOP;
		if (CommandQueue != NULL) {
            xQueueSend(CommandQueue, &stop, 0);
        }
		StartAdvertising();
		break;
	default:
		break;
	}
	return 0;
}

// 特征访问回调
static int GATTControlCharAccessCallback(uint16_t ConnHandle,
										 uint16_t attr_handle,
										 struct ble_gatt_access_ctxt *ctxt,
										 void *arg) {
	if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
		size_t len = OS_MBUF_PKTLEN(ctxt->om);
		if (len != sizeof(CommandPkg_t)) {
			ESP_LOGE(TAG, "Invalid command length: %d", len);
			return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
		}
		CommandPkg_t cmdPkg;
		int rc = os_mbuf_copydata(ctxt->om, 0, len, (uint8_t *)&cmdPkg);
		if (rc) {
			return BLE_ATT_ERR_UNLIKELY;
		}
		// 校验CRC
		uint16_t calcCrc =
			CalcCRC((uint8_t *)&cmdPkg.cmd, sizeof(Command_t) - 2);
		if (calcCrc != cmdPkg.crc) {
			ESP_LOGE(TAG,
					 "Invalid command CRC: received 0x%04X, calculated 0x%04X",
					 cmdPkg.crc, calcCrc);
			return BLE_ATT_ERR_UNLIKELY;
		}

		// 将命令放入队列
		if (CommandQueue != NULL && xQueueSend(CommandQueue, &cmdPkg.cmd, 0) != pdTRUE) {
			ESP_LOGE(TAG, "Failed to enqueue command message");
			return BLE_ATT_ERR_UNLIKELY;
		}
		return 0;
	}
	return 0;
}
static int GATTDataCharAccessCallback(uint16_t ConnHandle, uint16_t attr_handle,
									  struct ble_gatt_access_ctxt *ctxt,
									  void *arg) {
	// 数据特征不支持读写
	return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

// SendNotify函数（固定26包拆分）
int SendNotify(uint8_t *buf, size_t len) {
    if (!BLEConnected || ConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "No BLE connection, cannot send notification");
        return 0;
    }
    // 强制校验：只能传入512字节数据包
    if (len == 0 || buf == NULL || len != DATA_PACKAGE_SIZE) {
        ESP_LOGE(TAG, "Invalid buffer (len=%u, must be %d bytes)", (unsigned)len, DATA_PACKAGE_SIZE);
        return 1;
    }
    if (ble_tx_mutex == NULL) {
        ESP_LOGE(TAG, "BLE TX mutex not initialized");
        return 1;
    }

    // 加锁
    if (xSemaphoreTake(ble_tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take BLE TX mutex（100ms timeout）");
        return 1;
    }

    int ret = 0;
    // 核心：固定26包（512字节 / 20字节 = 25.6 → 向上取整为26）
    const uint8_t TOTAL_PACKETS_FIXED = 26;
    ESP_LOGI(TAG, "Start sending 512-byte pkg in %d fixed 22-byte BLE packets", TOTAL_PACKETS_FIXED);

    // 逐包构建（1~26包）
    for (uint8_t pkt_num = 0; pkt_num < TOTAL_PACKETS_FIXED; pkt_num++) {
        uint8_t pkt_buf[BLE_PACKET_MAX_SIZE] = {0}; // 22字节小包
        pkt_buf[0] = TOTAL_PACKETS_FIXED; // 第1字节：总包数（26）
        pkt_buf[1] = pkt_num + 1;         // 第2字节：包号（1~26）

        // 填充20字节数据区
        size_t offset = pkt_num * BLE_PACKET_DATA_SIZE;
        size_t pkt_data_len = MIN(BLE_PACKET_DATA_SIZE, DATA_PACKAGE_SIZE - offset);
        memcpy(&pkt_buf[2], &buf[offset], pkt_data_len);

        // 发送BLE分包
        struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt_buf, BLE_PACKET_MAX_SIZE);
        if (om == NULL) {
            ESP_LOGE(TAG, "ble_hs_mbuf_from_flat failed for packet %d", pkt_num + 1);
            ret = 1;
            break;
        }

        int rc = ble_gatts_notify_custom(ConnHandle, DataCharValHandle, om);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gatts_notify failed for packet %d: %d", pkt_num + 1, rc);
            os_mbuf_free_chain(om);
            ret = rc;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 分包间隔
        ESP_LOGD(TAG, "Sent BLE packet %d/26 (data len: %d)", pkt_num + 1, pkt_data_len);
    }

    xSemaphoreGive(ble_tx_mutex);

    if (ret == 0) {
        ESP_LOGI(TAG, "512-byte pkg sent as 26 fixed 22-byte BLE packets successfully");
    } else {
        ESP_LOGE(TAG, "Failed to send BLE packets, error: %d", ret);
    }
    return ret;
}

void RunBlueToothHost(void) {
	ESP_LOGI(TAG, "BLE Host Started");
	nimble_port_run();
	ESP_LOGI(TAG, "BLE Host Ended");
	if (ble_tx_mutex != NULL) {
        vSemaphoreDelete(ble_tx_mutex);
        ble_tx_mutex = NULL;
    }

	nimble_port_freertos_deinit();
	ESP_LOGI(TAG, "NimBLE Port Deinitialized");
}