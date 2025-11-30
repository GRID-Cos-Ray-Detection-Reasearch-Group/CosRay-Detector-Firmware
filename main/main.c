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
TaskHandle_t commandHandlerTaskHandle;
TaskHandle_t dataStoreTaskHandle;
TaskHandle_t telTaskHandle;

//********************GLOBAL VARS*************************//
volatile uint8_t TxBuffer[DATA_BUFFER_SIZE];
volatile uint8_t gpsBuffer[256];

// 消息队列声明
QueueHandle_t CommandQueue;
QueueHandle_t DataQueue;

// 信号量声明
SemaphoreHandle_t TxBufferMutex;
SemaphoreHandle_t TxBufferReadySemaphore;

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
static void AppBlueTooth(void *pvParameters);

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
			switch (cmdMsg.data[0]) {
			case 0x01: // 发送数据包
				xQueueSend(DataQueue, &cmdMsg, portMAX_DELAY);
				// TODO: 确认此处阻塞无需修改
				break;
			default:
				break;
			}
		}
	}
	ESP_LOGI(TAG, "Command Handler Task Ended");
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
	TxBufferMutex = xSemaphoreCreateMutex();
	if (TxBufferMutex == NULL) {
		ESP_LOGE(TAG, "Failed to create TxBuffer Mutex");
		return;
	}
	TxBufferReadySemaphore = xSemaphoreCreateBinary();
	if (TxBufferReadySemaphore == NULL) {
		ESP_LOGE(TAG, "Failed to create TxBuffer Ready Semaphore");
		return;
	}
	xSemaphoreGive(TxBufferReadySemaphore);

	// 创建消息队列
	CommandQueue = xQueueCreate(10, sizeof(Command_t));
	if (CommandQueue == NULL) {
		ESP_LOGE(TAG, "Failed to create Command Queue");
		return;
	}
	DataQueue = xQueueCreate(10, sizeof(Command_t));
	if (DataQueue == NULL) {
		ESP_LOGE(TAG, "Failed to create Data Queue");
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
	ESP_LOGI(TAG, "Creating Command Handler Task");
	RTOSRet = xTaskCreate(
		CommandHandlerTask, "CmdHandlerTask", COMMAND_HANDLER_TASK_STACK_SIZE,
		NULL, COMMAND_HANDLER_TASK_PRIORITY, &commandHandlerTaskHandle);
	if (RTOSRet != pdPASS) {
		ESP_LOGE(TAG, "Failed to create Command Handler Task");
		return;
	}
	ESP_LOGI(TAG, "Command Handler Task created successfully");

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
