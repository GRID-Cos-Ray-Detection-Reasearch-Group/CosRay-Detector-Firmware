#ifndef FLASHSTORAGE_H
#define FLASHSTORAGE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "../main/typedefs.h"

/* ================= 引脚定义 ================= */
#define PIN_CS           35
#define PIN_CLK          48
#define PIN_MOSI         47     //原理图画反了这里再纠正回来
#define PIN_MISO         42     //测试电路内是42，板子上是33
#define PIN_WP           36
#define PIN_HOLD         38

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

/* ================= 存储架构宏定义（统一修正，解决冲突） ================= */
#define W25N_PAGE_SIZE_MAIN      2048    // 页主区大小
#define W25N_PAGE_SIZE_OOB       64      // 页备用区（OOB）大小
#define W25N_PAGE_SIZE_SPARE     W25N_PAGE_SIZE_OOB // 兼容原有命名
#define W25N_PAGE_TOTAL_SIZE     (W25N_PAGE_SIZE_MAIN + W25N_PAGE_SIZE_OOB)
#define W25N_BLOCK_SIZE_PAGE     128     // W25N01KV 每块128页
#define W25N_PAGES_PER_BLOCK     W25N_BLOCK_SIZE_PAGE // 兼容原有命名
#define W25N_BLOCK_SIZE_BYTES    (W25N_BLOCK_SIZE_PAGE * W25N_PAGE_SIZE_MAIN)
#define W25N_TOTAL_BLOCKS        512     // W25N01KV 总块数
#define W25N_TOTAL_PAGES         (W25N_TOTAL_BLOCKS * W25N_BLOCK_SIZE_PAGE)
#define W25N01KV_TOTAL_PAGES     65536
#define W25N_BAD_BLOCK_MARK     0x00

/* ================= 设备ID宏定义 ================= */
#define JEDEC_MFG_ID    0xEF
#define JEDEC_DEV_MSB      0xAA
#define JEDEC_DEV_LSB      0x22
//02是AA22，01是AE21
/* ================= BLE分包相关常量 ================= */
#define DATA_PACKAGE_SIZE       512     // 单个BLE数据包大小（512字节）
#define FLASH_BATCH_SIZE        4       // 4*512=2048字节（1页）

/* ================= 全局Flash状态结构体（核心新增） ================= */
typedef struct {
    uint32_t write_page;       // 当前写入的Flash页号
    uint32_t last_send_pkg;    // 最后一次成功发送的数据包索引（供BLE同步）
    bool init_ok;              // Flash初始化完成标记
} FlashGlobalState_t;

/* ================= 外部全局变量声明 ================= */
extern spi_device_handle_t flash_spi_handle;
extern QueueHandle_t FlashQueue;
extern FlashGlobalState_t g_flash_state; // 全局Flash状态

/* ================= 统一驱动接口声明 ================= */
FlashStatus FlashStorageInit(void);
FlashStatus FlashEraseBlock(uint32_t page);
FlashStatus FlashWrite(uint32_t page, const uint8_t *data, uint32_t len);
FlashStatus FlashRead(uint32_t page, uint8_t *data, uint32_t len);
FlashStatus FlashReadOOB(uint32_t page, uint8_t *buf, uint32_t len);
void AppDataStoreTask(void *pv);

FlashStatus FlashReadMultiPage(uint32_t start_page, uint32_t page_count, uint8_t *buf, uint32_t *read_len);
FlashStatus FlashReadDataPackages(uint32_t start_pkg_idx, uint32_t pkg_count, uint8_t *out_buf, uint32_t *out_len);

#endif  // FLASHSTORAGE_H