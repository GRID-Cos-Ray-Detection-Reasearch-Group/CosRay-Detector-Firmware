#ifndef FLASHSTORAGE_H
#define FLASHSTORAGE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "../main/typedefs.h"

// ===================== 全 RAM 模式 =====================
#define FLASH_IN_RAM_MODE           1

// 每页 2048 字节（保持兼容）
#define W25N_PAGE_SIZE_MAIN         2048
#define DATA_PACKAGE_SIZE           512
#define FLASH_BATCH_SIZE            4

// 最大支持页数（RAM 大小）
#define RAM_STORAGE_MAX_PAGES       64

typedef enum {
    FLASH_OK = 0,
} FlashStatus;

// ===================== 全局状态 =====================
typedef struct {
    uint32_t write_page;
    uint32_t last_send_pkg;
    bool init_ok;
} FlashGlobalState_t;

extern FlashGlobalState_t g_flash_state;
extern QueueHandle_t FlashQueue;

// ===================== 接口保持不变 =====================
FlashStatus FlashStorageInit(void);
void AppDataStoreTask(void *pv);

FlashStatus FlashReadDataPackages(uint32_t start_pkg_idx, uint32_t pkg_count,
                                  uint8_t *out_buf, uint32_t *out_len);

#endif
