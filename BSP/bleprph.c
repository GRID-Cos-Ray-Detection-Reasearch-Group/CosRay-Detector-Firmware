#include <stdio.h>
#include <string.h>
#include <inttypes.h>  

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
#include "esp_rom_crc.h"
#include "sys/param.h"
#include "freertos/queue.h" 

#include "bsp.h"
#include "typedefs.h"
#include "flashstorage.h"
#include "config.h"

// BLE 发送队列（环形缓冲）
#define BLE_TX_QUEUE_SIZE  60   // 可缓存60包
static QueueHandle_t ble_tx_queue = NULL;
static SemaphoreHandle_t ble_tx_mutex = NULL;

// BLE分包配置
#define BLE_PACKET_MAX_SIZE         22
#define BLE_PACKET_GLOBAL_HEADER    2
#define BLE_PACKET_LOCAL_HEADER     2
#define BLE_PACKET_HEADER_TOTAL     (BLE_PACKET_GLOBAL_HEADER + BLE_PACKET_LOCAL_HEADER)
#define BLE_PACKET_DATA_SIZE        (BLE_PACKET_MAX_SIZE - BLE_PACKET_HEADER_TOTAL)
#define LOCAL_PACKETS_PER_GLOBAL    29
#define GLOBAL_DATA_LEN             512

static const char *TAG = "NimBLEModule";
static uint16_t ConnHandle = BLE_HS_CONN_HANDLE_NONE;
static bool BLEConnected = false;

static void ble_tx_task(void *arg);
extern QueueHandle_t CommandQueue;
extern FlashGlobalState_t g_flash_state;
extern esp_err_t SendFlashDataOverBLE(uint32_t start_pkg_idx, uint32_t pkg_count);

static int GAPEventCallback(struct ble_gap_event *Event, void *Arg);
static int GATTControlCharAccessCallback(uint16_t ConnHandle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int GATTDataCharAccessCallback(uint16_t ConnHandle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

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

static uint16_t DataCharValHandle;
static uint16_t ControlCharValHandle;

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

static void StartAdvertising(void) {
	struct ble_gap_adv_params AdvParams;
	struct ble_hs_adv_fields Fields;
	int rc;

	memset(&Fields, 0, sizeof(Fields));
	Fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	Fields.tx_pwr_lvl_is_present = 1;
	Fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

	Fields.name = (uint8_t *)"MuonDetector";
	Fields.name_len = strlen("MuonDetector");
	Fields.name_is_complete = 1;

	rc = ble_gap_adv_set_fields(&Fields);
	if (rc != 0) {
		ESP_LOGE(TAG, "Error setting adv data: %d", rc);
		return;
	}

	memset(&AdvParams, 0, sizeof(AdvParams));
	AdvParams.conn_mode = BLE_GAP_CONN_MODE_UND;
	AdvParams.disc_mode = BLE_GAP_DISC_MODE_GEN;

	rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
						   &AdvParams, GAPEventCallback, NULL);
	if (rc != 0) {
		ESP_LOGE(TAG, "Error enabling advertising: %d", rc);
	}
}

static int InitGATTServer(void) {
	int rc;
	rc = ble_gatts_count_cfg(GATTServerServices);
	if (rc) return rc;
	rc = ble_gatts_add_svcs(GATTServerServices);
	if (rc) return rc;
	return 0;
}

static void OnSyncCallback(void) {
	int rc;
	rc = ble_svc_gap_device_name_set("MuonDetector");
	if (rc != 0) {
		ESP_LOGE(TAG, "Error setting device name: %d", rc);
		return;
	}
    xTaskCreate(ble_tx_task, "ble_tx_task", 4096, NULL, 6, NULL);
	StartAdvertising();
}

esp_err_t InitBlueTooth(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NimBLE: %d", ret);
        return ret;
    }

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

    ble_tx_queue = xQueueCreate(BLE_TX_QUEUE_SIZE, BLE_PACKET_MAX_SIZE);
    if (ble_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create BLE TX queue");
        return ESP_FAIL;
    }

    ble_hs_cfg.sync_cb = OnSyncCallback;
    return ESP_OK;
}

static void AutoSendFlashDataTask(void *arg);

static int GAPEventCallback(struct ble_gap_event *Event, void *Arg) {
    static TaskHandle_t auto_send_task_handle = NULL;

    switch (Event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "Device connected");
            ConnHandle = Event->connect.conn_handle;
            BLEConnected = true;

            // 创建一次任务
            if (auto_send_task_handle == NULL) {
                BaseType_t task_ret = xTaskCreate(
                    AutoSendFlashDataTask,
                    "AutoSendFlashData",
                    8192,
                    NULL,
                    5,
                    &auto_send_task_handle  // 保存句柄
                );
                if (task_ret != pdPASS) {
                    ESP_LOGE(TAG, "Failed to create auto send task");
                }
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Device disconnected");
            ConnHandle = BLE_HS_CONN_HANDLE_NONE;
            BLEConnected = false;
            ESP_LOGI(TAG, "Disconnected, last sent pkg remains: %" PRIu32, g_flash_state.last_send_pkg);

            //断开时删除任务，防止死循环
            if (auto_send_task_handle != NULL) {
                vTaskDelete(auto_send_task_handle);
                auto_send_task_handle = NULL;
            }

            // 强制释放锁，解决 mutex timeout
            if (ble_tx_mutex != NULL) {
                xSemaphoreGive(ble_tx_mutex);
            }

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


static void AutoSendFlashDataTask(void *arg) {
    ESP_LOGI(TAG, "=== Auto send task started ===");
    
    uint32_t pkg_per_page = W25N_PAGE_SIZE_MAIN / DATA_PACKAGE_SIZE;
    uint32_t total_pkgs = g_flash_state.write_page * pkg_per_page;
    uint32_t unsent_pkgs = total_pkgs - g_flash_state.last_send_pkg;

    if (unsent_pkgs > 0 && BLEConnected) {
        ESP_LOGI(TAG, "Reconnected! Sending %" PRIu32 " unsent packages", unsent_pkgs);
        esp_err_t ret = SendFlashDataOverBLE(g_flash_state.last_send_pkg, unsent_pkgs);
        if (ret == ESP_OK) {
            g_flash_state.last_send_pkg = total_pkgs;
            ESP_LOGI(TAG, "All unsent data sent successfully!");
        } else {
            ESP_LOGE(TAG, "Resend failed: %d", ret);
        }
    }

    while (BLEConnected) {
        uint32_t now_total = g_flash_state.write_page * pkg_per_page;
        uint32_t new_unsent = now_total - g_flash_state.last_send_pkg;
        
        if (new_unsent > 0) {
            esp_err_t ret = SendFlashDataOverBLE(g_flash_state.last_send_pkg, new_unsent);
            if (ret == ESP_OK) {
                g_flash_state.last_send_pkg = now_total;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    vTaskDelete(NULL);
}


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
		uint16_t calcCrc = CalcCRC((uint8_t *)&cmdPkg.cmd, sizeof(Command_t) - 2);
		if (calcCrc != cmdPkg.crc) {
			ESP_LOGE(TAG, "Invalid command CRC");
			return BLE_ATT_ERR_UNLIKELY;
		}

		if (CommandQueue != NULL && xQueueSend(CommandQueue, &cmdPkg.cmd, 0) != pdTRUE) {
			ESP_LOGE(TAG, "Failed to enqueue command");
			return BLE_ATT_ERR_UNLIKELY;
		}
		return 0;
	}
	return 0;
}

static int GATTDataCharAccessCallback(uint16_t ConnHandle, uint16_t attr_handle,
									  struct ble_gatt_access_ctxt *ctxt,
									  void *arg) {
	return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

int SendNotify(uint8_t *buf, size_t len, uint8_t global_total, uint8_t global_idx) {
    if (!BLEConnected || ConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        return 0;
    }

    uint8_t frame_buf[GLOBAL_DATA_LEN] = {0};
    memcpy(frame_buf, buf, MIN(len, GLOBAL_DATA_LEN));

    for (int i = 0; i < LOCAL_PACKETS_PER_GLOBAL; i++) {
        uint8_t pkt[BLE_PACKET_MAX_SIZE] = {0};
        pkt[0] = global_total;
        pkt[1] = global_idx;
        pkt[2] = LOCAL_PACKETS_PER_GLOBAL;
        pkt[3] = i + 1;
        memcpy(&pkt[4], &frame_buf[i * BLE_PACKET_DATA_SIZE], BLE_PACKET_DATA_SIZE);

        xQueueSend(ble_tx_queue, pkt, 0);
    }
    return 0;
}

static void ble_tx_task(void *arg)
{
    uint8_t pkt[BLE_PACKET_MAX_SIZE];
    while (1)
    {
        if (xQueueReceive(ble_tx_queue, pkt, portMAX_DELAY))
        {
            if (BLEConnected && ConnHandle != BLE_HS_CONN_HANDLE_NONE)
            {
                struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, BLE_PACKET_MAX_SIZE);
                if (om) {
                    ble_gatts_notify_custom(ConnHandle, DataCharValHandle, om);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
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