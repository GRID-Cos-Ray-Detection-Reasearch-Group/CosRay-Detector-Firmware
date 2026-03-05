#ifndef BSP_H
#define BSP_H

#include "config.h"
#include "typedefs.h"
#include "esp_err.h"

// 缓冲区定义
extern uint8_t gpsBuffer[256];

// 队列定义
extern QueueHandle_t CommandQueue;
extern QueueHandle_t DataQueue;
extern QueueHandle_t TxQueue;
extern QueueHandle_t FlashQueue;

// BSP 初始化（蓝牙 + 数据外设）
esp_err_t BSPInit(void);

// 蓝牙接口
esp_err_t InitBlueTooth(void);
void RunBlueToothHost(void);
int SendNotify(uint8_t *buf, size_t len);

// 数据外设接口（ADC、定时器、muon事件处理）
esp_err_t InitDataPeripheral(void);
void RunDataPeripheral(void);

#endif // BSP_H
