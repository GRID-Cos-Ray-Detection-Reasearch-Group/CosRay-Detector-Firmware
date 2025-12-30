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
#include "typedefs.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "flashstorage.h"
#include "bsp.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_timer.h"

static const char *TAG = "MainModule";

TaskHandle_t dataProcessTaskHandle;
TaskHandle_t bluetoothTaskHandle;
TaskHandle_t bluetoothTxTaskHandle;
TaskHandle_t commandHandlerTaskHandle;
TaskHandle_t dataStoreTaskHandle;
TaskHandle_t telTaskHandle;

uint8_t gpsBuffer[256];
size_t gpsBufferLen = 0;

QueueHandle_t CommandQueue;
QueueHandle_t DataQueue;
QueueHandle_t TxQueue;
QueueHandle_t FlashQueue;

extern esp_err_t InitDataPeripheral(void);
extern void RunDataPeripheral(void);

// opcode
#define OPCODE_TRIGGER 0xA0
#define OPCODE_PPS 0xA1
#define OPCODE_TMP_ALERT 0xA2
#define OPCODE_GPS 0xA3

// 硬件引脚
#define PIN_SIGNAL 4
#define PIN_TMP_ALERT 5
#define PIN_CATHODE_MON 6
#define PIN_MON 7
#define PIN_CHARGEIN 8
#define PIN_RESTART 9
#define PIN_TRIGGER 10
#define PIN_PPS 11
#define PIN_GPS_RX 17
#define PIN_GPS_TX 18

// 定义flash存储大小
#define FLASH_QUEUE_SIZE 50

// GPS UART
#define GPS_UART_PORT UART_NUM_1
static QueueHandle_t uart_queue = NULL;

// 函数声明
static void GpsRxIntSetup(void);
static void PPSIntSetup(void);
static void MuonInttSetup(void);
static void TMPAlertIntSetup(void);
static void GPSTask(void *arg);
static void AppDataStore(void *pvParameters);
static void BlueToothTxTask(void *pvParameters);
static void AppBlueTooth(void *pvParameters);
static void CommandHandlerTask(void *pvParameters);
void InterruptSetup(void);
void AppSetup(void);

// ISR中断设置
//  触发信号中断（前放板 Triger）
static void IRAM_ATTR trigger_isr(void *arg) {
	BaseType_t hp = pdFALSE;
	Command_t cmd;
	memset(&cmd, 0, sizeof(cmd));
	cmd.data[0] = OPCODE_TRIGGER;
	xQueueSendFromISR(DataQueue, &cmd, &hp);
	if (hp)
		portYIELD_FROM_ISR();
}

// PPS 中断
static void IRAM_ATTR pps_isr(void *arg) {
	BaseType_t hp = pdFALSE;
	Command_t cmd;
	memset(&cmd, 0, sizeof(cmd));
	cmd.data[0] = OPCODE_PPS;
	xQueueSendFromISR(DataQueue, &cmd, &hp);
	if (hp)
		portYIELD_FROM_ISR();
}

// 温度报警中断（TMP112 ALERT）
static void IRAM_ATTR tmp_alert_isr(void *arg) {
	BaseType_t hp = pdFALSE;
	Command_t cmd;
	memset(&cmd, 0, sizeof(cmd));
	cmd.data[0] = OPCODE_TMP_ALERT;
	xQueueSendFromISR(DataQueue, &cmd, &hp);
	if (hp)
		portYIELD_FROM_ISR();
}

// 初始化 UART1 作为 GPS 接口
void GPS_UART_Init(void) {
	uart_config_t uart_config = {.baud_rate = 9600,
								 .data_bits = UART_DATA_8_BITS,
								 .parity = UART_PARITY_DISABLE,
								 .stop_bits = UART_STOP_BITS_1,
								 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

	uart_driver_install(GPS_UART_PORT, 2048, 2048, 20, &uart_queue, 0);
	uart_param_config(GPS_UART_PORT, &uart_config);
	uart_set_pin(GPS_UART_PORT, PIN_GPS_TX, PIN_GPS_RX, UART_PIN_NO_CHANGE,
				 UART_PIN_NO_CHANGE);

	ESP_LOGI(TAG, "GPS UART initialized successfully");
}

// GPS 接收任务
void GPSTask(void *arg) {
	uart_event_t event;
	uint8_t buf[256];

	ESP_LOGI(TAG, "GPSTask started");

	for (;;) {
		if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
			if (event.type == UART_DATA) {
				int len = uart_read_bytes(GPS_UART_PORT, buf, event.size,
										  pdMS_TO_TICKS(50));
				if (len > 0) {

					Command_t cmd;
					memset(&cmd, 0, sizeof(cmd));
					cmd.data[0] = OPCODE_GPS;

					int payload = (len > 254) ? 254 : len;
					if (payload > CMD_LENGTH - 1)
						payload = CMD_LENGTH - 1;
					memcpy(&cmd.data[1], buf, payload);

					if (xQueueSend(DataQueue, &cmd, pdMS_TO_TICKS(10)) !=
						pdTRUE) {
						ESP_LOGW(TAG, "GPS event dropped (DataQueue full)");
					} else {
						ESP_LOGI(TAG, "GPS data enqueued (%d bytes)", payload);
					}
				}
			}
		}
	}
}

// 设置触发中断
void MuonInttSetup(void) {
	gpio_config_t io = {
		.pin_bit_mask = 1ULL << PIN_TRIGGER,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.intr_type = GPIO_INTR_POSEDGE,
	};
	gpio_config(&io);
	gpio_isr_handler_add(PIN_TRIGGER, trigger_isr, NULL);
	ESP_LOGI(TAG, "Muon trigger ISR installed successfully");
}

// 设置 PPS 中断
void PPSIntSetup(void) {
	gpio_config_t io = {
		.pin_bit_mask = 1ULL << PIN_PPS,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.intr_type = GPIO_INTR_POSEDGE,
	};
	gpio_config(&io);
	gpio_isr_handler_add(PIN_PPS, pps_isr, NULL);
	ESP_LOGI(TAG, "PPS ISR installed successfully");
}

// 设置温度报警中断
void TMPAlertIntSetup(void) {
	gpio_config_t io = {
		.pin_bit_mask = 1ULL << PIN_TMP_ALERT,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.intr_type = GPIO_INTR_ANYEDGE,
	};
	gpio_config(&io);
	gpio_isr_handler_add(PIN_TMP_ALERT, tmp_alert_isr, NULL);
	ESP_LOGI(TAG, "TMP ALERT ISR installed successfully");
}

// GPS UART 初始化 + 创建任务
void GpsRxIntSetup(void) {
	GPS_UART_Init();
	xTaskCreate(GPSTask, "GPSTask", 4096, NULL, 5, NULL);
	ESP_LOGI(TAG, "GPSTask created successfully");
}

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

static void CommandHandlerTask(void *pvParameters) {
	ESP_LOGI(TAG, "CommandHandlerTask started");

	while (1) {
		Command_t cmdMsg;
		if (xQueueReceive(CommandQueue, &cmdMsg, portMAX_DELAY) == pdTRUE) {

			uint8_t opcode = cmdMsg.data[0];
			ESP_LOGI(TAG, "Command received: 0x%02X", opcode);

			if (opcode == START || opcode == STOP || opcode == ACK ||
				opcode == NACK) {

				if (xQueueSend(DataQueue, &cmdMsg, pdMS_TO_TICKS(50)) != pdTRUE)
					ESP_LOGE(TAG, "Failed to forward command to DataQueue");
				else
					ESP_LOGI(TAG, "Command forwarded successfully");

			}

			else if (opcode == OPCODE_TRIGGER || opcode == OPCODE_PPS ||
					 opcode == OPCODE_TMP_ALERT || opcode == OPCODE_GPS) {

				if (xQueueSend(DataQueue, &cmdMsg, pdMS_TO_TICKS(20)) != pdTRUE)
					ESP_LOGE(TAG, "Failed to forward special opcode");
			} else {
				ESP_LOGW(TAG, "Unknown opcode: 0x%02X", opcode);
			}
		}
	}
}

// BLE 发送任务

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

static void AppBlueTooth(void *pvParameters) {
	ESP_LOGI(TAG, "BlueToothTask started");
	RunBlueToothHost();
	ESP_LOGI(TAG, "BlueToothTask ended");
	vTaskDelete(NULL);
}

// 中断设置

void InterruptSetup(void) {
	ESP_LOGI(TAG, "Setting up interrupts...");

	gpio_install_isr_service(0);

	GpsRxIntSetup();
	ESP_LOGI(TAG, "GPS interrupt setup completed");
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

	// 初始化 BSP
	if (BSPInit() != ESP_OK) {
		ESP_LOGE(TAG, "BSPInit failed");
		return;
	}

	if (FlashStorageInit() != FLASH_OK) {
        ESP_LOGE(TAG, "Flash storage init failed");
        return;
    }


	// 创建任务
	xTaskCreate(AppBlueTooth, "BlueToothTask", BLUETOOTH_TASK_STACK_SIZE, NULL,
				BLUETOOTH_TASK_PRIORITY, &bluetoothTaskHandle);
	ESP_LOGI(TAG, "BlueToothTask created successfully");

	xTaskCreate(CommandHandlerTask, "CommandHandlerTask",
				COMMAND_HANDLER_TASK_STACK_SIZE, NULL,
				COMMAND_HANDLER_TASK_PRIORITY, &commandHandlerTaskHandle);
	ESP_LOGI(TAG, "CommandHandlerTask created successfully");

	xTaskCreate(BlueToothTxTask, "BlueToothTxTask",
				BLUETOOTH_TX_TASK_STACK_SIZE, NULL, BLUETOOTH_TX_TASK_PRIORITY,
				&bluetoothTxTaskHandle);
	ESP_LOGI(TAG, "BlueToothTxTask created successfully");

	xTaskCreate(AppDataStore, "DataStoreTask", DATA_STORE_TASK_STACK_SIZE, NULL,
				DATA_STORE_TASK_PRIORITY, &dataStoreTaskHandle);
	ESP_LOGI(TAG, "DataStoreTask created successfully");
	
	xTaskCreate(AppDataStoreTask, "FlashStoreTask", 
                4096,  // 栈大小
                NULL, 
                4,     // 任务优先级（合理即可）
                NULL);
    ESP_LOGI(TAG, "FlashStoreTask created");
}

void app_main(void) {
	ESP_LOGI(TAG, "FreeRTOS Application Starting...");
	AppSetup();
	InterruptSetup();
	ESP_LOGI(TAG, "All tasks created, system running");

	while (1) {
		ESP_LOGI(TAG, "Main running...");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
