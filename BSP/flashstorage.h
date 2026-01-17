#ifndef FLASHSTORAGE_H
#define FLASHSTORAGE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"  // 关键：解决 spi_device_handle_t 未定义问题
#include "../main/typedefs.h"  // 关键：包含已有 TxPkg_t 定义，避免重复

/* ================= 引脚定义（用户硬件配置，不可修改） ================= */
#define PIN_CS           35    // CS引脚（datasheet 6.2）
#define PIN_CLK          48    // SCLK引脚（datasheet 6.5）
#define PIN_MOSI         42    // MOSI(IO0)引脚（datasheet 6.1）
#define PIN_MISO         47    // MISO(IO1)引脚（datasheet 6.1）
#define PIN_WP           36    // WP(IO2)引脚（datasheet 6.3）
#define PIN_HOLD         38    // HOLD(IO3)引脚（datasheet 6.4）

/* ================= 操作状态枚举（覆盖所有可能的操作结果） ================= */
typedef enum {
    FLASH_OK = 0,              // 操作成功
    FLASH_INIT_FAILED,         // 初始化失败
    FLASH_READ_FAILED,         // 读取失败
    FLASH_WRITE_FAILED,        // 写入失败
    FLASH_ERASE_FAILED,        // 擦除失败
    FLASH_INVALID_ADDR,        // 无效地址
    FLASH_INVALID_PARAM,       // 无效参数
    FLASH_ID_MISMATCH,         // 设备ID不匹配
    FLASH_BAD_BLOCK,           // 坏块
    FLASH_TIMEOUT              // 超时
} FlashStatus;

/* ================= 存储架构宏定义（严格遵循datasheet 1. GENERAL DESCRIPTIONS） ================= */
#define W25N_PAGE_SIZE_MAIN      2048    // 页主区大小（字节）
#define W25N_PAGE_SIZE_SPARE     128     // 页备用区大小（字节）
#define W25N_PAGE_SIZE          (W25N_PAGE_SIZE_MAIN + W25N_PAGE_SIZE_SPARE) // 总页大小（2176字节）
#define W25N_PAGES_PER_BLOCK    64      // 每块页数（128KB/块）
#define W25N_TOTAL_BLOCKS       2048    // 总块数（2G-bit）
#define W25N_TOTAL_PAGES        (W25N_TOTAL_BLOCKS * W25N_PAGES_PER_BLOCK) // 总页数（131072）

/* ================= 设备ID宏定义（实际读取为 0xEF AE 21） ================= */
#define W25N_MANUFACTURER_ID    0xEF    // 厂商ID（Winbond，datasheet 10.1.1）
#define W25N_DEVICE_ID_MSB      0xAE    // 设备ID高位（实际读取值）
#define W25N_DEVICE_ID_LSB      0x21    // 设备ID低位（实际读取值）

/* ================= 外部全局变量声明（供外部模块访问） ================= */
extern spi_device_handle_t flash_spi_handle;
extern QueueHandle_t FlashQueue;

/* ================= 函数接口声明（完整功能覆盖） ================= */
/**
 * @brief 初始化Flash存储系统
 * @return FlashStatus 操作状态
 */
FlashStatus FlashStorageInit(void);

/**
 * @brief 检查指定块是否为坏块
 * @param block_addr 块地址（0~2047）
 * @return bool true=坏块，false=正常块
 */
bool FlashIsBadBlock(uint32_t block_addr);

/**
 * @brief 擦除指定块（128KB）
 * @param block_addr 块地址（0~2047）
 * @return FlashStatus 操作状态
 */
FlashStatus FlashEraseBlock(uint32_t block_addr);

/**
 * @brief 写入一页数据（仅主区，最大2048字节）
 * @param page_addr 页地址（0~131071）
 * @param data 待写入数据指针
 * @param len 数据长度（≤2048）
 * @return FlashStatus 操作状态
 */
FlashStatus FlashWrite(uint32_t page_addr, const uint8_t *data, uint32_t len);

/**
 * @brief 读取一页数据（仅主区，最大2048字节）
 * @param page_addr 页地址（0~131071）
 * @param data 接收数据缓冲区指针
 * @param len 读取长度（≤2048）
 * @return FlashStatus 操作状态
 */
FlashStatus FlashRead(uint32_t page_addr, uint8_t *data, uint32_t len);

/**
 * @brief 数据存储任务：从队列读取数据并写入Flash
 * @param pvParameters 任务参数（未使用）
 */
void AppDataStoreTask(void *pvParameters);

#endif // FLASHSTORAGE_H