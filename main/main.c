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
#include <inttypes.h>  // 必须包含：PRIu32 宏定义
#include <stdio.h>
#include <string.h>

// 替换弃用的ADC头文件（解决警告）
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "flashstorage.h"
#include "bsp.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_timer.h"

static const char *TAG = "MainModule";

// 全局变量定义
TaskHandle_t dataProcessTaskHandle;
TaskHandle_t bluetoothTaskHandle;
TaskHandle_t bluetoothTxTaskHandle;
TaskHandle_t commandHandlerTaskHandle;
TaskHandle_t dataStoreTaskHandle;
TaskHandle_t telTaskHandle;
TaskHandle_t dataPeripheralTaskHandle;


uint8_t gpsBuffer[256];
size_t gpsBufferLen = 0;

QueueHandle_t CommandQueue;
QueueHandle_t DataQueue;
QueueHandle_t TxQueue;
QueueHandle_t FlashQueue;

// 外部函数声明
extern esp_err_t InitDataPeripheral(void);
extern void RunDataPeripheral(void *pvParameters);
extern esp_err_t SendFlashDataOverBLE(uint32_t start_pkg_idx, uint32_t pkg_count);
extern FlashGlobalState_t g_flash_state;

// GPS UART 配置
#define GPS_UART_PORT UART_NUM_1
static QueueHandle_t uart_queue = NULL;

// ================= ISR中断处理 =================
static void IRAM_ATTR trigger_isr(void *arg) {
    BaseType_t hp = pdFALSE;
    Command_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.data[0] = OPCODE_TRIGGER;
    xQueueSendFromISR(DataQueue, &cmd, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static void IRAM_ATTR pps_isr(void *arg) {
    BaseType_t hp = pdFALSE;
    Command_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.data[0] = OPCODE_PPS;
    xQueueSendFromISR(DataQueue, &cmd, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static void IRAM_ATTR tmp_alert_isr(void *arg) {
    BaseType_t hp = pdFALSE;
    Command_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.data[0] = OPCODE_TMP_ALERT;
    xQueueSendFromISR(DataQueue, &cmd, &hp);
    if (hp) portYIELD_FROM_ISR();
}

// ================= GPS UART初始化 =================
void GPS_UART_Init(void) {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(GPS_UART_PORT, 2048, 2048, 20, &uart_queue, 0);
    uart_param_config(GPS_UART_PORT, &uart_config);
    uart_set_pin(GPS_UART_PORT, PIN_GPS_TX, PIN_GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "GPS UART initialized successfully");
}

// ================= GPS接收任务 =================
void GPSTask(void *arg) {
    uart_event_t event;
    uint8_t buf[256];

    ESP_LOGI(TAG, "GPSTask started");

    for (;;) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(GPS_UART_PORT, buf, event.size, pdMS_TO_TICKS(50));
                if (len > 0) {
                    Command_t cmd;
                    memset(&cmd, 0, sizeof(cmd));
                    cmd.data[0] = OPCODE_GPS;

                    int payload = (len > (CMD_LENGTH - 1)) ? (CMD_LENGTH - 1) : len;
                    memcpy(&cmd.data[1], buf, payload);

                    if (xQueueSend(DataQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
                        ESP_LOGW(TAG, "GPS event dropped (DataQueue full)");
                    } else {
                        // 修复：使用 PRIu32 格式化 int 类型（兼容uint32_t）
                        ESP_LOGI(TAG, "GPS data enqueued (%" PRIu32 " bytes)", (uint32_t)payload);
                    }
                }
            }
        }
    }
}

// ================= 中断初始化 =================
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

void GpsRxIntSetup(void) {
    GPS_UART_Init();
    xTaskCreate(GPSTask, "GPSTask", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "GPSTask created successfully");
}




// ================= 命令处理任务 =================
static void CommandHandlerTask(void *pvParameters) {
    ESP_LOGI(TAG, "CommandHandlerTask started");

    while (1) {
        Command_t cmdMsg;
        if (xQueueReceive(CommandQueue, &cmdMsg, portMAX_DELAY) == pdTRUE) {
            uint8_t opcode = cmdMsg.data[0];
            ESP_LOGI(TAG, "Command received: 0x%02X", opcode);

            if (opcode == STATUS) {
                ESP_LOGI(TAG, "STATUS command received, sending flash data over BLE");
                // 计算未发送的数据包数（统一使用 PRIu32）
                uint32_t total_pkgs = g_flash_state.write_page * (W25N_PAGE_SIZE_MAIN / DATA_PACKAGE_SIZE);
                uint32_t unsent_pkgs = total_pkgs - g_flash_state.last_send_pkg;
                
                if (unsent_pkgs > 0) {
                    esp_err_t ret = SendFlashDataOverBLE(g_flash_state.last_send_pkg, unsent_pkgs);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Send flash data over BLE failed: %d", ret);
                    } else {
                        ESP_LOGI(TAG, "Sent %" PRIu32 " unsent flash packages over BLE", unsent_pkgs);
                    }
                } else {
                    ESP_LOGI(TAG, "No unsent flash data to send (all packages sent)");
                }
                continue;
            }

            if (opcode == START || opcode == STOP || opcode == ACK || opcode == NACK) {
                if (xQueueSend(DataQueue, &cmdMsg, pdMS_TO_TICKS(50)) != pdTRUE)
                    ESP_LOGE(TAG, "Failed to forward command to DataQueue");
                else
                    ESP_LOGI(TAG, "Command forwarded successfully");
            } else if (opcode == OPCODE_TRIGGER || opcode == OPCODE_PPS ||
                       opcode == OPCODE_TMP_ALERT || opcode == OPCODE_GPS) {
                if (xQueueSend(DataQueue, &cmdMsg, pdMS_TO_TICKS(20)) != pdTRUE)
                    ESP_LOGE(TAG, "Failed to forward special opcode");
            } else {
                ESP_LOGW(TAG, "Unknown opcode: 0x%02X", opcode);
            }
        }
    }
}

// ================= BLE发送任务（修复SendNotify参数） =================
static void BlueToothTxTask(void *pvParameters) {
    ESP_LOGI(TAG, "BlueToothTxTask started (support 512-byte packages)");

    while (1) {
        TxPkg_t TxPkg;
        if (xQueueReceive(TxQueue, &TxPkg, portMAX_DELAY)) {
            // 修复：size_t 使用 %zu 格式化
            ESP_LOGI(TAG, "Sending BLE data (%zu bytes)", TxPkg.length);
            
            // 匹配SendNotify的4个参数原型
            int rc = SendNotify(TxPkg.data, TxPkg.length, 0, 0);
            if (rc != 0)
                ESP_LOGE(TAG, "BLE notify failed (%d)", rc);
            else
                ESP_LOGI(TAG, "BLE data sent successfully (%zu bytes)", TxPkg.length);
        }
    }
}

static void AppBlueTooth(void *pvParameters) {
    ESP_LOGI(TAG, "BlueToothTask started");
    RunBlueToothHost();
    ESP_LOGI(TAG, "BlueToothTask ended");
    vTaskDelete(NULL);
}

// ================= 中断总配置 =================
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

// ================= 系统初始化 =================
void AppSetup(void) {
    // 创建消息队列（使用config.h中的宏，避免重复定义）
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
    
    xTaskCreate(RunDataPeripheral,"DataPeripheralTask", 
        DATA_PERIPHERAL_TASK_STACK_SIZE, NULL, DATA_PERIPHERAL_TASK_PRIORITY, &dataPeripheralTaskHandle);

    xTaskCreate(AppDataStoreTask, "DataStoreTask", DATA_STORE_TASK_STACK_SIZE, NULL,
                DATA_STORE_TASK_PRIORITY, &dataStoreTaskHandle);
    ESP_LOGI(TAG, "DataStoreTask created successfully");
}



// ================= 主函数 =================
void app_main(void) {
    ESP_LOGI(TAG, "FreeRTOS Application Starting...");
    AppSetup();
    InterruptSetup();
    ESP_LOGI(TAG, "All tasks created, system running");

    while (1) {
        // 核心修复：使用 PRIu32 格式化 uint32_t
        ESP_LOGI(TAG, "Main running... Flash write page: %" PRIu32 ", last sent pkg: %" PRIu32, 
                 g_flash_state.write_page, g_flash_state.last_send_pkg);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}