#include "flashstorage.h"
#include "esp_log.h"
#include "string.h"
#include "stdlib.h"

static const char *TAG = "RamStorage";

// ===================== RAM 数组 =====================
static uint8_t s_ram_storage[RAM_STORAGE_MAX_PAGES][W25N_PAGE_SIZE_MAIN];
FlashGlobalState_t g_flash_state = {0};
extern QueueHandle_t FlashQueue;

// ===================== 初始化 =====================
FlashStatus FlashStorageInit(void)
{
    memset(s_ram_storage, 0, sizeof(s_ram_storage));
    g_flash_state.write_page = 0;
    g_flash_state.last_send_pkg = 0;
    g_flash_state.init_ok = true;

    ESP_LOGI(TAG, "✅ RAM Storage initialized (pages: %d)", RAM_STORAGE_MAX_PAGES);
    return FLASH_OK;
}

// ===================== 批量写入 RAM =====================
static esp_err_t RamWriteBatchPage(const uint8_t *data)
{
    if (g_flash_state.write_page >= RAM_STORAGE_MAX_PAGES) {
        ESP_LOGW(TAG, "RAM full, wrap to 0");
        g_flash_state.write_page = 0;
    }

    memcpy(s_ram_storage[g_flash_state.write_page], data, W25N_PAGE_SIZE_MAIN);
    ESP_LOGI(TAG, "Batch write to RAM page %" PRIu32, g_flash_state.write_page);
    g_flash_state.write_page++;
    return ESP_OK;
}

// ===================== 读取数据包（512B） =====================
FlashStatus FlashReadDataPackages(uint32_t start_pkg_idx, uint32_t pkg_count,
                                  uint8_t *out_buf, uint32_t *out_len)
{
    *out_len = 0;
    uint32_t pkts_per_page = W25N_PAGE_SIZE_MAIN / DATA_PACKAGE_SIZE;

    for (uint32_t i = 0; i < pkg_count; i++) {
        uint32_t pkt_idx = start_pkg_idx + i;
        uint32_t page = pkt_idx / pkts_per_page;
        uint32_t off = (pkt_idx % pkts_per_page) * DATA_PACKAGE_SIZE;

        if (page >= RAM_STORAGE_MAX_PAGES) break;

        memcpy(out_buf + i * DATA_PACKAGE_SIZE,
               s_ram_storage[page] + off,
               DATA_PACKAGE_SIZE);
        *out_len += DATA_PACKAGE_SIZE;
    }

    ESP_LOGI(TAG, "Read %" PRIu32 " pkts from RAM", pkg_count);
    return FLASH_OK;
}

// ===================== 存储任务 =====================
void AppDataStoreTask(void *pvParameters)
{
    ESP_LOGI(TAG, "✅ RamStorage task started");
    FlashStorageInit();

    static uint8_t batch_buf[FLASH_BATCH_SIZE * DATA_PACKAGE_SIZE];
    static uint32_t batch_cnt = 0;
    TxPkg_t pkg;

    while (1) {
        if (xQueueReceive(FlashQueue, &pkg, portMAX_DELAY)) {
            if (pkg.length > DATA_PACKAGE_SIZE) continue;

            uint8_t full_pkt[DATA_PACKAGE_SIZE] = {0};
            memcpy(full_pkt, pkg.data, pkg.length);
            memcpy(&batch_buf[batch_cnt * DATA_PACKAGE_SIZE], full_pkt, DATA_PACKAGE_SIZE);
            batch_cnt++;

            if (batch_cnt >= FLASH_BATCH_SIZE) {
                RamWriteBatchPage(batch_buf);
                batch_cnt = 0;
                memset(batch_buf, 0, sizeof(batch_buf));
            }
        }
    }
}