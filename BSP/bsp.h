#ifndef BSP_H
#define BSP_H

#include "config.h"
#include "typedefs.h"

// 缓冲区定义
extern uint8_t gpsBuffer[256];

// 队列定义
extern QueueHandle_t CommandQueue;
extern QueueHandle_t DataQueue;
extern QueueHandle_t TxQueue;

// 信号量定义

// BSP Interface functions
esp_err_t BSPInit(void);

esp_err_t InitBlueTooth(void);
void RunBlueToothHost(void);
int SendNotify(uint8_t *buf, size_t len);

esp_err_t InitDataPeripheral(void);
void RunDataPeripheral(void);

#endif // BSP_H