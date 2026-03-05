#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "bsp.h"
#include "flashstorage.h"
#include "gps_module.h"
#include "typedefs.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"

static const char *TAG = "MainModule";

// 任务句柄
TaskHandle_t dataProcessTaskHandle;
TaskHandle_t bluetoothTxTaskHandle;
TaskHandle_t commandHandlerTaskHandle;
TaskHandle_t dataStoreTaskHandle;
TaskHandle_t telTaskHandle;

uint8_t gpsBuffer[256];

// 消息队列
QueueHandle_t CommandQueue;
QueueHandle_t DataQueue;
QueueHandle_t TxQueue;
QueueHandle_t FlashQueue;

extern esp_err_t InitDataPeripheral(void);
extern void RunDataPeripheral(void);

// 函数声明
static void PPSIntSetup(void);
static void MuonInttSetup(void);
static void TMPAlertIntSetup(void);
static void AppDataStore(void *pvParameters);
static void BlueToothTxTask(void *pvParameters);
static void CommandHandlerTask(void *pvParameters);
void InterruptSetup(void);
void AppSetup(void);

/* ================= ISR 中断处理函数 ================= */

// μ子比较器触发中断（上升沿）
static void IRAM_ATTR trigger_isr(void *arg) {
	BaseType_t hp = pdFALSE;
	Command_t cmd;
	memset(&cmd, 0, sizeof(cmd));
	cmd.data[0] = OPCODE_TRIGGER;
	xQueueSendFromISR(DataQueue, &cmd, &hp);
	if (hp)
		portYIELD_FROM_ISR();
}

// GPS PPS 秒脉冲中断（上升沿）
static void IRAM_ATTR pps_isr(void *arg) {
	BaseType_t hp = pdFALSE;
	Command_t cmd;
	memset(&cmd, 0, sizeof(cmd));
	cmd.data[0] = OPCODE_PPS;
	xQueueSendFromISR(DataQueue, &cmd, &hp);
	if (hp)
		portYIELD_FROM_ISR();
}

// TMP112 温度报警中断（双沿）
static void IRAM_ATTR tmp_alert_isr(void *arg) {
	BaseType_t hp = pdFALSE;
	Command_t cmd;
	memset(&cmd, 0, sizeof(cmd));
	cmd.data[0] = OPCODE_TMP_ALERT;
	xQueueSendFromISR(DataQueue, &cmd, &hp);
	if (hp)
		portYIELD_FROM_ISR();
}

/* ================= 中断设置函数 ================= */

// 设置 μ子触发中断
static void MuonInttSetup(void) {
	gpio_config_t io = {
		.pin_bit_mask = 1ULL << PIN_TRIGGER,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.intr_type = GPIO_INTR_POSEDGE,
	};
	gpio_config(&io);
	gpio_isr_handler_add(PIN_TRIGGER, trigger_isr, NULL);
	ESP_LOGI(TAG, "Muon trigger ISR installed (GPIO%d)", PIN_TRIGGER);
}

// 设置 GPS PPS 中断
static void PPSIntSetup(void) {
	gpio_config_t io = {
		.pin_bit_mask = 1ULL << PIN_PPS,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.intr_type = GPIO_INTR_POSEDGE,
	};
	gpio_config(&io);
	gpio_isr_handler_add(PIN_PPS, pps_isr, NULL);
	ESP_LOGI(TAG, "PPS ISR installed (GPIO%d)", PIN_PPS);
}

// 设置 TMP112 温度报警中断
static void TMPAlertIntSetup(void) {
	gpio_config_t io = {
		.pin_bit_mask = 1ULL << PIN_TMP_ALERT,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.intr_type = GPIO_INTR_ANYEDGE,
	};
	gpio_config(&io);
	gpio_isr_handler_add(PIN_TMP_ALERT, tmp_alert_isr, NULL);
	ESP_LOGI(TAG, "TMP ALERT ISR installed (GPIO%d)", PIN_TMP_ALERT);
}

/* ================= FreeRTOS 任务 ================= */

// 数据采集任务：初始化 ADC/定时器，然后进入数据处理循环
static void AppDataStore(void *pvParameters) {
	ESP_LOGI(TAG, "DataStoreTask started");

	if (InitDataPeripheral() != ESP_OK)
		ESP_LOGE(TAG, "InitDataPeripheral failed");
	else
		ESP_LOGI(TAG, "InitDataPeripheral completed successfully");

	RunDataPeripheral();

	ESP_LOGI(TAG, "DataStoreTask ended");
	vTaskDelete(NULL);
}

// 命令路由任务：将 BLE 收到的命令转发到 DataQueue
static void CommandHandlerTask(void *pvParameters) {
	ESP_LOGI(TAG, "CommandHandlerTask started");

	while (1) {
		Command_t cmdMsg;
		if (xQueueReceive(CommandQueue, &cmdMsg, portMAX_DELAY) == pdTRUE) {
			uint8_t opcode = cmdMsg.data[0];
			ESP_LOGI(TAG, "Command received: 0x%02X", opcode);

			if (opcode == START || opcode == STOP || opcode == ACK ||
				opcode == NACK || opcode == STATUS || opcode == PING) {
				if (xQueueSend(DataQueue, &cmdMsg, pdMS_TO_TICKS(50)) != pdTRUE)
					ESP_LOGE(TAG, "Failed to forward command to DataQueue");
			} else {
				ESP_LOGW(TAG, "Unknown command opcode: 0x%02X", opcode);
			}
		}
	}
}

// 蓝牙发送任务：从 TxQueue 读取数据并通过 BLE notify 发送
static void BlueToothTxTask(void *pvParameters) {
	ESP_LOGI(TAG, "BlueToothTxTask started");

	while (1) {
		TxPkg_t TxPkg;
		if (xQueueReceive(TxQueue, &TxPkg, portMAX_DELAY)) {
			ESP_LOGI(TAG, "Sending BLE data (%u bytes)",
					 (unsigned)TxPkg.length);
			int rc = SendNotify(TxPkg.data, TxPkg.length);
			if (rc != 0)
				ESP_LOGE(TAG, "BLE notify failed (%d)", rc);
		}
	}
}

// 蓝牙主机任务由 nimble_port_freertos_init() 在 InitBlueTooth() 中自动创建，无需手动创建

/* ================= 初始化函数 ================= */

void InterruptSetup(void) {
	ESP_LOGI(TAG, "Setting up interrupts...");

	// 安装 GPIO 中断服务（只调用一次）
	gpio_install_isr_service(0);

	PPSIntSetup();
	ESP_LOGI(TAG, "PPS interrupt setup completed");

	MuonInttSetup();
	ESP_LOGI(TAG, "Muon interrupt setup completed");

	TMPAlertIntSetup();
	ESP_LOGI(TAG, "TMP Alert interrupt setup completed");

	ESP_LOGI(TAG, "All interrupts setup completed");
}

void AppSetup(void) {
	// 创建消息队列
	CommandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(Command_t));
	if (!CommandQueue) {
		ESP_LOGE(TAG, "Failed to create CommandQueue");
		return;
	}

	DataQueue = xQueueCreate(DATA_QUEUE_SIZE, sizeof(Command_t));
	if (!DataQueue) {
		ESP_LOGE(TAG, "Failed to create DataQueue");
		return;
	}

	TxQueue = xQueueCreate(TX_QUEUE_SIZE, sizeof(TxPkg_t));
	if (!TxQueue) {
		ESP_LOGE(TAG, "Failed to create TxQueue");
		return;
	}

	FlashQueue = xQueueCreate(FLASH_QUEUE_SIZE, sizeof(TxPkg_t));
	if (!FlashQueue) {
		ESP_LOGE(TAG, "Failed to create FlashQueue");
		return;
	}

	// 初始化 BSP（BLE + ADC/定时器）
	if (BSPInit() != ESP_OK) {
		ESP_LOGE(TAG, "BSPInit failed");
		return;
	}

	// 启动 GPS 模块（UART + UBX 解析任务）
	gps_start();
	ESP_LOGI(TAG, "GPS module started");

	// NimBLE 主机任务已由 InitBlueTooth() → nimble_port_freertos_init() 自动创建
	ESP_LOGI(TAG, "BLE host task started by nimble_port_freertos_init");

	xTaskCreate(CommandHandlerTask, "CommandHandlerTask",
				COMMAND_HANDLER_TASK_STACK_SIZE, NULL,
				COMMAND_HANDLER_TASK_PRIORITY, &commandHandlerTaskHandle);
	ESP_LOGI(TAG, "CommandHandlerTask created");

	xTaskCreate(BlueToothTxTask, "BlueToothTxTask",
				BLUETOOTH_TX_TASK_STACK_SIZE, NULL, BLUETOOTH_TX_TASK_PRIORITY,
				&bluetoothTxTaskHandle);
	ESP_LOGI(TAG, "BlueToothTxTask created");

	xTaskCreate(AppDataStore, "DataStoreTask", DATA_STORE_TASK_STACK_SIZE, NULL,
				DATA_STORE_TASK_PRIORITY, &dataStoreTaskHandle);
	ESP_LOGI(TAG, "DataStoreTask created");

	// 创建 Flash 存储任务
	xTaskCreate(AppDataStoreTask, "FlashStoreTask", 4096, NULL, 4, NULL);
	ESP_LOGI(TAG, "FlashStoreTask created");
}

void app_main(void) {
	ESP_LOGI(TAG, "CosRay Muon Detector Firmware Starting...");
	AppSetup();
	InterruptSetup();
	ESP_LOGI(TAG, "All tasks created, system running");
	// app_main 任务在初始化完成后挂起；FreeRTOS 调度器继续运行所有其他任务
	vTaskSuspend(NULL);
}
