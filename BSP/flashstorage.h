#ifndef FLASHSTORAGE_H
#define FLASHSTORAGE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "../main/typedefs.h"

/* ================= 引脚定义（用户硬件配置，不可修改） ================= */
#define PIN_CS           35
#define PIN_CLK          48
#define PIN_MOSI         42
//mosi33/42
#define PIN_MISO         47
#define PIN_WP           36
#define PIN_HOLD         38

/* ================= 操作状态枚举（覆盖所有可能的操作结果） ================= */
typedef enum {
    FLASH_OK = 0,
    FLASH_TIMEOUT,
    FLASH_INVALID_PARAM,
    FLASH_ID_MISMATCH,
    FLASH_SPI_ERROR,
    FLASH_SPI_INIT_ERR,
    FLASH_WRITE_ENABLE_ERR,
    FLASH_OP_ERROR,
    FLASH_BAD_BLOCK,
    FLASH_CRC_ERROR,
    FLASH_QUEUE_ERR,
} FlashStatus;

/* ================= 存储架构宏定义（统一修正，解决冲突） ================= */
#define W25N_PAGE_SIZE_MAIN      2048    // 页主区大小
#define W25N_PAGE_SIZE_OOB       64      // 页备用区（OOB）大小（统一为64，与c文件保持一致）
#define W25N_PAGE_SIZE_SPARE     W25N_PAGE_SIZE_OOB // 兼容原有命名，避免冲突
#define W25N_PAGE_TOTAL_SIZE     (W25N_PAGE_SIZE_MAIN + W25N_PAGE_SIZE_OOB)
#define W25N_BLOCK_SIZE_PAGE     128     // W25N01KV 每块128页
#define W25N_PAGES_PER_BLOCK     W25N_BLOCK_SIZE_PAGE // 兼容原有命名
#define W25N_BLOCK_SIZE_BYTES    (W25N_BLOCK_SIZE_PAGE * W25N_PAGE_SIZE_MAIN)
#define W25N_TOTAL_BLOCKS        512     // W25N01KV 总块数（65536页 / 128页/块）
#define W25N_TOTAL_PAGES         (W25N_TOTAL_BLOCKS * W25N_BLOCK_SIZE_PAGE)
#define W25N01KV_TOTAL_PAGES     65536

/* ================= 设备ID宏定义 ================= */
#define W25N_MANUFACTURER_ID    0xEF
#define W25N_DEVICE_ID_MSB      0xAE
#define W25N_DEVICE_ID_LSB      0x21
//W25N0xAA22  AE21 

/* ================= 外部全局变量声明 ================= */
extern spi_device_handle_t flash_spi_handle;
extern QueueHandle_t FlashQueue;

/* ================= 统一驱动接口声明 ================= */

FlashStatus FlashStorageInit(void);
FlashStatus FlashEraseBlock(uint32_t page);
FlashStatus FlashWrite(uint32_t page, const uint8_t *data, uint32_t len);
FlashStatus FlashRead(uint32_t page, uint8_t *data, uint32_t len);
FlashStatus FlashReadOOB(uint32_t page, uint8_t *buf, uint32_t len);
void AppDataStoreTask(void *pv);

#endif 




