#ifndef BSP_H
#define BSP_H

#include "config.h"
#include "typedefs.h"

// 缓冲区定义
extern volatile uint8_t TxBuffer[DATA_BUFFER_SIZE];
extern volatile uint8_t gpsBuffer[256];

// 队列定义
extern QueueHandle_t CommandQueue;
extern QueueHandle_t DataQueue;

// 信号量定义
extern SemaphoreHandle_t TxBufferMutex;

// BSP Interface functions
esp_err_t BSPInit(void);

esp_err_t InitBlueTooth(void);
void RunBlueToothHost(void);

esp_err_t InitDataPeripheral(void);
void RunDataPeripheral(void);

uint16_t CalcCRC(const uint8_t *data, size_t length);

#endif // BSP_H