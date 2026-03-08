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
#include "flashstorage.h"
#include "config.h"

// BLE分包配置
#define BLE_PACKET_MAX_SIZE         22      // 单包总长度固定22字节
#define BLE_PACKET_GLOBAL_HEADER    2       // 全局包头（总包数+当前总包号）
#define BLE_PACKET_LOCAL_HEADER     2       // 局部包头（分包数+当前分包号）
#define BLE_PACKET_HEADER_TOTAL     (BLE_PACKET_GLOBAL_HEADER + BLE_PACKET_LOCAL_HEADER) // 4字节总包头
#define BLE_PACKET_DATA_SIZE        (BLE_PACKET_MAX_SIZE - BLE_PACKET_HEADER_TOTAL) // 18字节数据区
#define LOCAL_PACKETS_PER_GLOBAL    29      // 每个512字节总包拆分为30个子包（512/18≈29，向上取整为30）
#define GLOBAL_DATA_LEN             512     // 单个全局包固定512字节

static SemaphoreHandle_t ble_tx_mutex = NULL;



/*====解析方式====*/
/*每包 22 字节(MTU23)；
第 1 字节：总包数；
第 2 字节：当前包号；
第 3 字节：分包数（当前总包内的分包总数，最后一包可能不足30）；
第 4 字节：当前分包号；

第 5~22 字节：数据区（18 字节），最后一包的最后10字节是补 0；
拼接所有 26 包的 Data 区后，截取前 512 字节即为原始数据。
*/



static const char *TAG = "NimBLEModule";
static uint16_t ConnHandle = BLE_HS_CONN_HANDLE_NONE;
static bool BLEConnected = false;

extern QueueHandle_t CommandQueue;
extern FlashGlobalState_t g_flash_state;
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
// NimBLEModule.c 中完整的 GAPEventCallback 函数
// 先声明自动发送任务（放在GAPEventCallback之前）
static void AutoSendFlashDataTask(void *arg);

static int GAPEventCallback(struct ble_gap_event *Event, void *Arg) {
    switch (Event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "Device connected");
            ConnHandle = Event->connect.conn_handle;
            BLEConnected = true;

            // 创建自动发送任务（标准FreeRTOS接口，无兼容性问题）
            BaseType_t task_ret = xTaskCreate(
                AutoSendFlashDataTask,  // 任务函数
                "AutoSendFlashData",    // 任务名
                8192,                   // 栈大小（足够处理Flash读取+BLE发送）
                NULL,                   // 无参数传递
                5,                      // 优先级（与BLE任务同级别）
                NULL                    // 无需保存任务句柄
            );
            if (task_ret != pdPASS) {
                ESP_LOGE(TAG, "Failed to create auto send task");
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Device disconnected");
            ConnHandle = BLE_HS_CONN_HANDLE_NONE;
            BLEConnected = false;
			ESP_LOGI(TAG, "Disconnected, last sent pkg remains: %u", g_flash_state.last_send_pkg);
    
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

// 独立的自动发送任务函数（核心逻辑）
// 替换原AutoSendFlashDataTask函数
static void AutoSendFlashDataTask(void *arg) {
    ESP_LOGI(TAG, "=== Auto send task started (polling flash data) ===");
    
    // 轮询检测，直到断开连接/有数据发送
    while (BLEConnected) {
        // 计算当前已存储的数据包总数
        uint32_t pkg_per_page = W25N_PAGE_SIZE_MAIN / DATA_PACKAGE_SIZE;
        uint32_t total_pkgs = g_flash_state.write_page * pkg_per_page;
        uint32_t unsent_pkgs = total_pkgs - g_flash_state.last_send_pkg;

        if (unsent_pkgs > 0) {
            // 有未发送数据，执行发送
            ESP_LOGI(TAG, "Found %u unsent pkg(s) (total: %u, sent: %u)", 
                     unsent_pkgs, total_pkgs, g_flash_state.last_send_pkg);
            
            esp_err_t ret = SendFlashDataOverBLE(g_flash_state.last_send_pkg, unsent_pkgs);
            if (ret == ESP_OK) {
                g_flash_state.last_send_pkg = total_pkgs;
                ESP_LOGI(TAG, "Send success! Last sent pkg: %u", g_flash_state.last_send_pkg);
                //break; // 发送完成，退出轮询
            } else {
                ESP_LOGE(TAG, "Send failed, retry after 1s (err: %d)", ret);
            }
        }

        // 无数据则1秒后重试
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Auto send task exited");
    vTaskDelete(NULL);
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

int SendNotify(uint8_t *buf, size_t len, uint8_t global_total, uint8_t global_idx) {
    // 1. 基础校验
    if (!BLEConnected || ConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "SendNotify failed: BLE not connected");
        return 0;
    }
    if (len == 0 || buf == NULL || global_total == 0 || global_idx == 0) {
        ESP_LOGE(TAG, "SendNotify invalid params: len=%u, global_total=%u, global_idx=%u", 
                 (unsigned)len, global_total, global_idx);
        return 1;
    }
    if (ble_tx_mutex == NULL) {
        ESP_LOGE(TAG, "SendNotify failed: TX mutex not init");
        return 1;
    }

    // 2. 加锁保护
    if (xSemaphoreTake(ble_tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "SendNotify failed: take mutex timeout");
        return 1;
    }

    int ret = 0;
    const uint8_t local_total = LOCAL_PACKETS_PER_GLOBAL;
    ESP_LOGI(TAG, "Start sending global pkg %d/%d (512 bytes) -> %d x 22-byte BLE packets",
             global_idx, global_total, local_total);

    // 3. 初始化512字节缓冲区（不足补0，超出截断）
    uint8_t *full_global_buf = (uint8_t *)heap_caps_malloc(GLOBAL_DATA_LEN, MALLOC_CAP_DEFAULT);
    if (full_global_buf == NULL) {
        ESP_LOGE(TAG, "SendNotify: malloc full_global_buf failed");
        xSemaphoreGive(ble_tx_mutex);
        return 1;
    }
    memset(full_global_buf, 0, GLOBAL_DATA_LEN); // 初始化
    memcpy(full_global_buf, buf, MIN(len, GLOBAL_DATA_LEN));

    // 4. 逐分包构建并发送
    for (uint8_t local_idx = 0; local_idx < local_total; local_idx++) {
        uint8_t pkt_buf[BLE_PACKET_MAX_SIZE] = {0};
        
        // 填充4字节包头
        pkt_buf[0] = global_total;      // 全局总包数（本次连接需发送的512字节包总数）
        pkt_buf[1] = global_idx;        // 全局当前总包号（1~N）
        pkt_buf[2] = local_total;       // 局部分包数（固定30）
        pkt_buf[3] = local_idx + 1;     // 局部当前分包号（1~30）

        // 填充18字节数据区
        size_t data_offset = local_idx * BLE_PACKET_DATA_SIZE;
        size_t data_len = MIN(BLE_PACKET_DATA_SIZE, GLOBAL_DATA_LEN - data_offset);
        memcpy(&pkt_buf[4], &full_global_buf[data_offset], data_len);

        // 5. 创建BLE消息并发送
        struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt_buf, BLE_PACKET_MAX_SIZE);
        if (om == NULL) {
            ESP_LOGE(TAG, "SendNotify: mbuf create failed (global %d, local %d)",
                     global_idx, local_idx + 1);
            ret = 1;
            break;
        }

        int rc = ble_gatts_notify_custom(ConnHandle, DataCharValHandle, om);
        if (rc != 0) {
            ESP_LOGE(TAG, "SendNotify: notify failed (global %d, local %d): %d",
                     global_idx, local_idx + 1, rc);
            os_mbuf_free_chain(om);
            ret = rc;
            break;
        }

        // 6. 分包发送间隔（避免BLE拥塞）
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        ESP_LOGD(TAG, "SendNotify: sent global %d/%d, local %d/%d (data len: %d)",
                 global_idx, global_total, local_idx + 1, local_total, data_len);
    }
	
    free(full_global_buf);

    // 7. 释放锁并返回
    xSemaphoreGive(ble_tx_mutex);
    if (ret == 0) {
        ESP_LOGI(TAG, "SendNotify: success (global pkg %d/%d, %d BLE packets sent)",
                 global_idx, global_total, local_total);
    } else {
        ESP_LOGE(TAG, "SendNotify: failed (global pkg %d, err: %d)", global_idx, ret);
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