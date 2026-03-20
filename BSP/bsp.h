#ifndef BSP_H
#define BSP_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "config.h"
#include "typedefs.h"

// 缓冲区定义
extern uint8_t gpsBuffer[256];

// 队列定义
extern QueueHandle_t CommandQueue;
extern QueueHandle_t DataQueue;
extern QueueHandle_t TxQueue;


// 信号量定义
void gps_uart_intr_setup(void);
void gps_uart_intr_disable(void);
void gps_pps_intr_setup(void);
void gps_pps_intr_disable(void);
void gps_all_intr_setup(void);
void gps_poll_task_create(void);
void gps_clear_buffer(void);
void gps_uart_poll(void);
void gps_get_pps_stats(uint32_t *count, uint64_t *last_time_us);

// BSP Interface functions
esp_err_t BSPInit(void);
esp_err_t InitBlueTooth(void);
void RunBlueToothHost(void);
int SendNotify(uint8_t *buf, size_t len, uint8_t global_total, uint8_t global_idx);

esp_err_t InitDataPeripheral(void);
void RunDataPeripheral(void *pvParameters);

esp_err_t SendFlashDataOverBLE(uint32_t start_pkg_idx, uint32_t pkg_count);
#endif // BSP_H