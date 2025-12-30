#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "config.h"
#include "typedefs.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// W25N02KVZEIR 芯片参数
#define W25N02KV_PAGE_SIZE       2048    // 页大小（字节）
#define W25N02KV_BLOCK_SIZE      64      // 每块包含的页数
#define W25N02KV_TOTAL_BLOCKS    16    // 总块数（2Gbit / 128KB每块）
#define W25N02KV_BLOCK_SIZE_BYTES (W25N02KV_PAGE_SIZE * W25N02KV_BLOCK_SIZE) // 128KB

// 操作返回状态
typedef enum {
    FLASH_OK = 0,
    FLASH_INIT_FAILED,
    FLASH_ID_MISMATCH,
    FLASH_WRITE_FAILED,
    FLASH_READ_FAILED,
    FLASH_ERASE_FAILED,
    FLASH_BAD_BLOCK,
    FLASH_INVALID_ADDR
} FlashStatus;

// 外部队列声明（与主程序共享）
extern QueueHandle_t FlashQueue;

// 初始化Flash设备
FlashStatus FlashStorageInit(void);

// 从Flash读取数据
FlashStatus FlashRead(uint32_t page_addr, uint8_t *data, uint32_t len);

// 向Flash写入数据（需确保页已擦除）
FlashStatus FlashWrite(uint32_t page_addr, const uint8_t *data, uint32_t len);

// 擦除指定块（64页）
FlashStatus FlashEraseBlock(uint32_t block_addr);

// 检查块是否为坏块
bool FlashIsBadBlock(uint32_t block_addr);

// Flash存储任务（从队列读取数据并写入Flash）
void AppDataStoreTask(void *pvParameters);

#endif // FLASH_STORAGE_H