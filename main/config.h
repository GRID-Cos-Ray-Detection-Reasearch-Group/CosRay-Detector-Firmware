#ifndef CONFIG_H
#define CONFIG_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdint.h>  // 引入标准整数类型定义

// ================= 任务优先级定义 =================
#define DATA_PROCESS_TASK_PRIORITY    1
#define BLUETOOTH_TASK_PRIORITY       5
#define DATA_STORE_TASK_PRIORITY      4
#define DATA_TEL_TASK_PRIORITY        2
#define COMMAND_HANDLER_TASK_PRIORITY 4
#define BLUETOOTH_TX_TASK_PRIORITY    7
#define DATA_PERIPHERAL_TASK_PRIORITY 5

// ================= 任务堆栈大小定义 =================
#define DATA_PROCESS_TASK_STACK_SIZE    4096
#define BLUETOOTH_TASK_STACK_SIZE       8192
#define DATA_STORE_TASK_STACK_SIZE      4096
#define DATA_TEL_TASK_STACK_SIZE        3072
#define COMMAND_HANDLER_TASK_STACK_SIZE 4096
#define BLUETOOTH_TX_TASK_STACK_SIZE    8192
#define DATA_PERIPHERAL_TASK_STACK_SIZE 4096
// ================= 队列大小配置 =================
#define COMMAND_QUEUE_SIZE 20
#define DATA_QUEUE_SIZE    50
#define TX_QUEUE_SIZE      50
#define FLASH_QUEUE_SIZE   50

// ================= 数据类型定义 =================
#define DATA_TYPE_GPS    1
#define DATA_TYPE_PPS    2
#define DATA_TYPE_MUON   3
#define DATA_TYPE_OTHER  4

// ================= 缓冲区/指令配置 =================
#define CMD_LENGTH         8
#define DATA_PACKAGE_SIZE  512
#define W25N_PAGE_SIZE_MAIN 2048  // 补充Flash页大小定义

// ================= Opcode 定义 =================
#define OPCODE_TRIGGER    0xA0
#define OPCODE_PPS        0xA1
#define OPCODE_TMP_ALERT  0xA2
#define OPCODE_GPS        0xA3

// ================= 硬件引脚定义 =================
#define PIN_SIGNAL        4
#define PIN_TMP_ALERT     5
#define PIN_CATHODE_MON   6
#define PIN_MON           7
#define PIN_CHARGEIN      8
#define PIN_RESTART       9
#define PIN_TRIGGER       10
#define PIN_PPS           11
#define PIN_GPS_RX        17
#define PIN_GPS_TX        18

// ================= 类型别名（解决格式化问题） =================
// 使用 PRIu32 宏标准化 uint32_t 格式化（需包含 inttypes.h）
#ifndef PRIu32
#define PRIu32 "lu"  // long unsigned int（适配ESP32-S3的uint32_t）
#endif

// ================= 任务句柄声明 =================
extern TaskHandle_t dataProcessTaskHandle;
extern TaskHandle_t bluetoothTaskHandle;
extern TaskHandle_t dataStoreTaskHandle;
extern TaskHandle_t telTaskHandle;
extern TaskHandle_t commandHandlerTaskHandle;
extern TaskHandle_t bluetoothTxTaskHandle;

#endif // CONFIG_H