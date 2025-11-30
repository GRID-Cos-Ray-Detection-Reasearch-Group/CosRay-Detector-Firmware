#include "config.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>

#include "bsp.h"

// TAG 变量指向存储在 flash 中的一个字符串字面量
// 见esp_log使用教程：https://docs.espressif.com/projects/esp-idf/zh_CN/stable/esp32/api-reference/system/log.html
static const char *TAG = "MainModule";

// 任务句柄声明
TaskHandle_t dataProcessTaskHandle;
TaskHandle_t bluetoothTaskHandle;
TaskHandle_t bluetoothTxTaskHandle;
TaskHandle_t commandHandlerTaskHandle;
TaskHandle_t dataStoreTaskHandle;
TaskHandle_t telTaskHandle;

//********************GLOBAL VARS*************************//
uint8_t gpsBuffer[256];

// 消息队列声明
QueueHandle_t CommandQueue;
QueueHandle_t DataQueue;
QueueHandle_t TxQueue;

// 信号量声明

static void GpsRxIntTask(void);

static void GpsRxIntSetup(void);

/*!
 * \brief
 *
 */
static void PPSIntTask(void);

static void PPSIntSetup(void);

/*!
 * \brief
 *
 */
static void MuonIntTask(void);

static void MuonInttSetup(void);

/*!
 * \brief
 *
 */
static void AppDataProcess(void *pvParameters);

/*!
 * \brief
 *
 */
static void AppDataStore(void *pvParameters);

/*!
 * \brief
 *
 */
static void BlueToothTxTask(void *pvParameters);

/*!
 * \brief
 *
 */
static void AppBlueTooth(void *pvParameters);

/*!
 * \brief
 *
 */
static void CommandHandlerTask(void *pvParameters);

/*!
 * \brief
 *
 */
void InterruptSetup(void);

/*!
 * \brief
 *
 */
void AppSetup(void);

void app_main(void) {
	ESP_LOGI(TAG, "FreeRTOS Application Starting...");
	AppSetup();
	InterruptSetup();
	ESP_LOGI(TAG, "All tasks created, scheduler will start");
	// 注意：esp-idf 的 freertos不需要用户启动系统任务调度
	// vTaskStartScheduler();  //Enables task scheduling
	while (1) {
		ESP_LOGI(TAG, "Running...");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

static void GpsRxIntTask(void) {}

static void GpsRxIntSetup(void) {}

/*!
 * \brief
 *
 */
static void PPSIntTask(void) {}

static void PPSIntSetup(void) {}

/*!
 * \brief
 *
 */
static void MuonIntTask(void) {}

static void MuonInttSetup(void) {}

/*!
 * \brief
 *
 */
static void AppDataProcess(void *pvParameters) {}

/*!
 * \brief
 *
 */
static void AppDataStore(void *pvParameters) {
	ESP_LOGI(TAG, "Data Store Task Started");
	RunDataPeripheral();
	ESP_LOGI(TAG, "Data Store Task Ended");
}

/*!
 * \brief
 *
 */
// static void AppDataTEL(void *pvParameters) {}

static void CommandHandlerTask(void *pvParameters) {
	ESP_LOGI(TAG, "Command Handler Task Started");
	while (1) {
		Command_t cmdMsg;
		if (xQueueReceive(CommandQueue, &cmdMsg, portMAX_DELAY) == pdTRUE) {
			// 处理命令
			uint8_t opcode = cmdMsg.data[0];
			if (opcode == START || opcode == STOP ||
				opcode == ACK || opcode == NACK) {
				ESP_LOGI(TAG, "Processing command: 0x%02X", opcode);
				// 将命令转发到数据管理任务
				if (xQueueSend(DataQueue, &cmdMsg, pdMS_TO_TICKS(100)) != pdTRUE) {
					ESP_LOGE(TAG, "Failed to forward command to Data Queue");
				}
			} else if (opcode == PING) {
				// TODO
			}
			else if (opcode == STATUS) {
				// TODO
			}
			else {
				ESP_LOGW(TAG, "Unknown command opcode: 0x%02X", opcode);
			}
		}
	}
	ESP_LOGI(TAG, "Command Handler Task Ended");
}

// BLE 发送任务
static void BlueToothTxTask(void *pvParameters) {
	while (1) {
		TxPkg_t TxPkg;
		if (xQueueReceive(TxQueue, &TxPkg, portMAX_DELAY) == pdTRUE) {
			// 发送数据
			size_t dataLen = TxPkg.length;
			ESP_LOGI(TAG, "Sending data notification of length %u", (unsigned) dataLen);
			int rc = SendNotify(TxPkg.data, dataLen);
			if (rc != 0) {
				ESP_LOGE(TAG, "Failed to send data notification");
			}
		}
	}
}

// BLE 模块任务
static void AppBlueTooth(void *pvParameters) {
	ESP_LOGI(TAG, "BLE Task Started");
	RunBlueToothHost();
	ESP_LOGI(TAG, "BLE Task Ended");
}

void InterruptSetup(void) {
	ESP_LOGI(TAG, "Setting up interrupts...");

	GpsRxIntSetup();
	ESP_LOGI(TAG, "GPS interrupt setup completed");

	PPSIntSetup();
	ESP_LOGI(TAG, "PPS interrupt setup completed");

	MuonInttSetup();
	ESP_LOGI(TAG, "Muon interrupt setup completed");

	ESP_LOGI(TAG, "All interrupts setup completed");
}

void AppSetup(void) {
	// 创建信号量

	// 创建消息队列
	CommandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(Command_t));
	if (CommandQueue == NULL) {
		ESP_LOGE(TAG, "Failed to create Command Queue");
		return;
	}
	DataQueue = xQueueCreate(DATA_QUEUE_SIZE, sizeof(Command_t));
	if (DataQueue == NULL) {
		ESP_LOGE(TAG, "Failed to create Data Queue");
		return;
	}
	TxQueue = xQueueCreate(TX_QUEUE_SIZE, sizeof (TxPkg_t));
	if (TxQueue == NULL) {
		ESP_LOGE(TAG, "Failed to create Tx Queue");
		return;
	}

	esp_err_t ESPRet;
	BaseType_t RTOSRet;

	ESPRet = BSPInit();
	if (ESPRet != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize BSP");
		return;
	}

	// 创建GPS接收任务
	// RTOSRet = xTaskCreate(
	// 	AppDataProcess, "DataProcessTask", DATA_PROCESS_TASK_STACK_SIZE, NULL,
	// 	DATA_PROCESS_TASK_PRIORITY, &dataProcessTaskHandle);
	// if (RTOSRet != pdPASS) {
	// 	ESP_LOGE(TAG, "Failed to create Task1");
	// 	return;
	// }
	// ESP_LOGI(TAG, "Task1 created successfully");

	// 创建蓝牙任务
	ESP_LOGI(TAG, "Creating BlueTooth Task");
	RTOSRet =
		xTaskCreate(AppBlueTooth, "BlueToothTask", BLUETOOTH_TASK_STACK_SIZE,
					NULL, BLUETOOTH_TASK_PRIORITY, &bluetoothTaskHandle);
	if (RTOSRet != pdPASS) {
		ESP_LOGE(TAG, "Failed to create BlueTooth Task");
		return;
	}
	ESP_LOGI(TAG, "BlueTooth Task created successfully");

	// 创建命令处理任务
	ESP_LOGI(TAG, "Creating CommandHandlerTask");
	RTOSRet = xTaskCreate(
		CommandHandlerTask, "CommandHandlerTask", COMMAND_HANDLER_TASK_STACK_SIZE,
		NULL, COMMAND_HANDLER_TASK_PRIORITY, &commandHandlerTaskHandle);
	if (RTOSRet != pdPASS) {
		ESP_LOGE(TAG, "Failed to create CommandHandlerTask");
		return;
	}
	ESP_LOGI(TAG, "CommandHandlerTask created successfully");

	// 创建蓝牙发送任务
	ESP_LOGI(TAG, "Creating BlueToothTxTask");
	RTOSRet =
		xTaskCreate(BlueToothTxTask, "BlueToothTxTask", BLUETOOTH_TX_TASK_STACK_SIZE,
					NULL, BLUETOOTH_TX_TASK_PRIORITY, &bluetoothTxTaskHandle);
	if (RTOSRet != pdPASS) {
		ESP_LOGE(TAG, "Failed to create BlueToothTxTask");
		return;
	}
	ESP_LOGI(TAG, "BlueToothTxTask created successfully");

	// 创建数据存储任务
	RTOSRet =
		xTaskCreate(AppDataStore, "DataStoreTask", DATA_STORE_TASK_STACK_SIZE,
					NULL, DATA_STORE_TASK_PRIORITY, &dataStoreTaskHandle);
	if (RTOSRet != pdPASS) {
		ESP_LOGE(TAG, "Failed to create DataStore Task");
		return;
	}
	// ESP_LOGI(TAG, "Task1 created successfully");
	// RTOSRet = xTaskCreate(AppDataTEL, "DataTELTask",
	// DATA_TEL_TASK_STACK_SIZE, 				  NULL, DATA_TEL_TASK_PRIORITY,
	// &telTaskHandle); if (RTOSRet != pdPASS) { 	ESP_LOGE(TAG, "Failed to
	// create Task1"); 	return;
	// }
	// ESP_LOGI(TAG, "Task1 created successfully");
}
