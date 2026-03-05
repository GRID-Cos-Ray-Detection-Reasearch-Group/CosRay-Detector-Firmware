
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
#define COMMAND_HANDLER_TASK_PRIORITY 5
#define BLUETOOTH_TX_TASK_PRIORITY 4

// 任务堆栈大小定义
#define DATA_PROCESS_TASK_STACK_SIZE 4096
#define BLUETOOTH_TASK_STACK_SIZE 4096
#define DATA_STORE_TASK_STACK_SIZE 3072
#define DATA_TEL_TASK_STACK_SIZE 3072
#define COMMAND_HANDLER_TASK_STACK_SIZE 3072
#define BLUETOOTH_TX_TASK_STACK_SIZE 3072

// 数据类型定义
#define DATA_TYPE_GPS 1
#define DATA_TYPE_PPS 2
#define DATA_TYPE_MUON 3
#define DATA_TYPE_OTHER 4

// 任务句柄声明（NimBLE 主机任务由 nimble_port_freertos_init 内部管理，无需手动句柄）
extern TaskHandle_t dataProcessTaskHandle;
extern TaskHandle_t dataStoreTaskHandle;
extern TaskHandle_t telTaskHandle;
extern TaskHandle_t commandHandlerTaskHandle;

// 队列长度定义
#define COMMAND_QUEUE_SIZE 10
#define DATA_QUEUE_SIZE 10
#define TX_QUEUE_SIZE 15
#define FLASH_QUEUE_SIZE 50

// 命令/数据包参数
#define CMD_LENGTH 8
#define DATA_PACKAGE_SIZE 512

// 内部操作码（ISR → DataQueue 消息标识，与 BSP/dataprph.c 共用）
#define OPCODE_TRIGGER   0xA0  // μ子比较器触发
#define OPCODE_PPS       0xA1  // GPS PPS 秒脉冲
#define OPCODE_TMP_ALERT 0xA2  // TMP112 温度报警
#define OPCODE_GPS       0xA3  // GPS 数据更新通知

// 硬件引脚定义（ESP32-S3 GPIO 编号）
#define PIN_SIGNAL      4   // ADC 信号输入（SiPM 信号幅度）
#define PIN_TMP_ALERT   5   // TMP112 温度报警中断
#define PIN_CATHODE_MON 6   // SiPM 阴极电压监测（ADC）
#define PIN_MON         7   // SiPM 电流监测（ADC）
#define PIN_CHARGEIN    8   // 充电状态检测
#define PIN_RESTART     9   // 复位输出
#define PIN_TRIGGER     10  // μ子比较器触发输入（上升沿中断）
#define PIN_PPS         11  // GPS PPS 秒脉冲输入（上升沿中断）

#endif // CONFIG_H