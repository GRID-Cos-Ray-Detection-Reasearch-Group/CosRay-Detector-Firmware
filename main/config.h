
#ifndef CONFIG_H
#define CONFIG_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// 任务优先级定义
#define DATA_PROCESS_TASK_PRIORITY 1
#define BLUETOOTH_TASK_PRIORITY 4
#define DATA_STORE_TASK_PRIORITY 3
#define DATA_TEL_TASK_PRIORITY 2
#define COMMAND_HANDLER_TASK_PRIORITY 5 // TODO: 设置合适的优先级

// 任务堆栈大小定义
#define DATA_PROCESS_TASK_STACK_SIZE 4096
#define BLUETOOTH_TASK_STACK_SIZE 4096
#define DATA_STORE_TASK_STACK_SIZE 3072
#define DATA_TEL_TASK_STACK_SIZE 3072
#define COMMAND_HANDLER_TASK_STACK_SIZE 3072 // TODO: 设置合适的堆栈大小

// 数据类型定义
#define DATA_TYPE_GPS 1
#define DATA_TYPE_PPS 2
#define DATA_TYPE_MUON 3
#define DATA_TYPE_OTHER 4

// 任务句柄声明
extern TaskHandle_t dataProcessTaskHandle;
extern TaskHandle_t bluetoothTaskHandle;
extern TaskHandle_t dataStoreTaskHandle;
extern TaskHandle_t telTaskHandle;
extern TaskHandle_t commandHandlerTaskHandle;

// 缓冲区大小定义
#define CMD_LENGTH 8
#define DATA_BUFFER_SIZE 512

#endif // CONFIG_H