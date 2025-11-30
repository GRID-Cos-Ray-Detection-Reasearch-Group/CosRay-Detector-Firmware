#ifndef BSP_H
#define BSP_H

#include "config.h"
#include "typedefs.h"

// 缓冲区定义
extern uint8_t TxBuffer[DATA_BUFFER_SIZE];
extern uint8_t gpsBuffer[256];

// 队列定义
extern QueueHandle_t CommandQueue;
extern QueueHandle_t DataQueue;

// 信号量定义
extern SemaphoreHandle_t TxBufferMutex;
extern SemaphoreHandle_t TxBufferReadySemaphore;

// BSP Interface functions
esp_err_t BSPInit(void);

esp_err_t InitBlueTooth(void);
void RunBlueToothHost(void);

esp_err_t InitDataPeripheral(void);
void RunDataPeripheral(void);

#endif // BSP_H