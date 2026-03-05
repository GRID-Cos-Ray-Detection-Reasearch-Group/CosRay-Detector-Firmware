#ifndef FLASHSTORAGE_H
#define FLASHSTORAGE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "typedefs.h"

/* ================= 引脚定义（SPI NAND Flash W25N01KV） ================= */
#define PIN_CS   35
#define PIN_CLK  48
#define PIN_MOSI 42
#define PIN_MISO 47
#define PIN_WP   36
#define PIN_HOLD 38

/* ================= 操作状态枚举 ================= */
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

/* ================= 存储架构宏定义（W25N01KV） ================= */
#define W25N_PAGE_SIZE_MAIN   2048  // 页主区大小（字节）
#define W25N_PAGE_SIZE_OOB    64    // 页备用区（OOB）大小（字节）
#define W25N_PAGE_SIZE_SPARE  W25N_PAGE_SIZE_OOB
#define W25N_PAGE_TOTAL_SIZE  (W25N_PAGE_SIZE_MAIN + W25N_PAGE_SIZE_OOB)
#define W25N_BLOCK_SIZE_PAGE  128   // 每块页数
#define W25N_PAGES_PER_BLOCK  W25N_BLOCK_SIZE_PAGE
#define W25N_BLOCK_SIZE_BYTES (W25N_BLOCK_SIZE_PAGE * W25N_PAGE_SIZE_MAIN)
#define W25N_TOTAL_BLOCKS     512   // 总块数（65536页 / 128页/块）
#define W25N_TOTAL_PAGES      (W25N_TOTAL_BLOCKS * W25N_BLOCK_SIZE_PAGE)
#define W25N01KV_TOTAL_PAGES  65536

/* ================= 设备 ID 宏定义 ================= */
#define W25N_MANUFACTURER_ID 0xEF
#define W25N_DEVICE_ID_MSB   0xAE
#define W25N_DEVICE_ID_LSB   0x21

/* ================= 外部全局变量声明 ================= */
extern spi_device_handle_t flash_spi_handle;
extern QueueHandle_t FlashQueue;

/* ================= 驱动接口声明 ================= */

/** 初始化 SPI 总线、Flash 器件，校验 JEDEC ID，关闭写保护 */
FlashStatus FlashStorageInit(void);

/** 擦除指定块（page 必须是块首地址，即 page % 128 == 0） */
FlashStatus FlashEraseBlock(uint32_t page);

/** 写入一页数据（len <= 2048）；块首页写入前会自动擦除 */
FlashStatus FlashWrite(uint32_t page, const uint8_t *data, uint32_t len);

/** 读取一页主区数据（len <= 2048） */
FlashStatus FlashRead(uint32_t page, uint8_t *data, uint32_t len);

/** 读取一页 OOB 备用区数据（len <= 64） */
FlashStatus FlashReadOOB(uint32_t page, uint8_t *buf, uint32_t len);

/** Flash 存储 FreeRTOS 任务：从 FlashQueue 读取数据并写入 Flash */
void AppDataStoreTask(void *pv);

#endif // FLASHSTORAGE_H
