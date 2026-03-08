#include "flashstorage.h"
#include "typedefs.h"
#include "esp_log.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_rom_crc.h"  
#include "sys/param.h"     
#include <string.h>

static const char *TAG = "FlashStorage";
static FlashStatus FlashPageToCache(uint32_t page);

spi_device_handle_t flash_spi_handle;
extern QueueHandle_t FlashQueue;
FlashGlobalState_t g_flash_state = {
    .write_page = 0,          // 初始写入页号为0
    .last_send_pkg = 0,       // 初始数据包索引为0
    .init_ok = false          // 初始化完成标记默认false
};

// 累计多个512字节数据包后再写入
#define FLASH_BATCH_SIZE 4 // 4*512=2048字节（1页）
static uint8_t batch_buf[FLASH_BATCH_SIZE * DATA_PACKAGE_SIZE] = {0};
static size_t batch_count = 0;

/* ================= SPI NAND 指令 ================= */
#define CMD_RESET_ENABLE        0x66
#define CMD_RESET_EXECUTE       0x99
#define CMD_READ_ID             0x9F
#define CMD_WRITE_ENABLE        0x06
#define CMD_PAGE_READ           0x13
#define CMD_READ_CACHE          0x03
#define CMD_PROG_LOAD           0x02
#define CMD_PROG_EXECUTE        0x10
#define CMD_BLOCK_ERASE         0xD8

#define CMD_GET_FEATURE         0x0F
#define CMD_SET_FEATURE         0x1F

/* ================= Feature Register 地址 ================= */
#define REG_STATUS              0xC0
#define REG_PROTECT             0xA0

/* ================= Status Register 位 ================= */
#define STATUS_OIP              0x01
#define STATUS_WEL              0x02
#define STATUS_E_FAIL           0x04
#define STATUS_P_FAIL           0x08
#define STATUS_ECC_MASK         0x30


/* ========================================================= */

static inline void cs_low(void)
{
    gpio_set_level(PIN_CS, 0);
    esp_rom_delay_us(1);
}

static inline void cs_high(void)
{
    esp_rom_delay_us(1);
    gpio_set_level(PIN_CS, 1);
    esp_rom_delay_us(1);
}

/* ================= Feature Register 读写 ================= */

static uint8_t FlashGetFeature(uint8_t reg)
{
    uint8_t tx[3] = {CMD_GET_FEATURE, reg, 0};
    uint8_t rx[3] = {0};
    spi_transaction_t t = {.length = 24, .tx_buffer = tx, .rx_buffer = rx};
    cs_low();
    spi_device_transmit(flash_spi_handle, &t);
    cs_high();
    return rx[2];
}

static FlashStatus FlashSetFeature(uint8_t reg, uint8_t val)
{
    uint8_t tx[3] = {CMD_SET_FEATURE, reg, val};
    spi_transaction_t t = {.length = 24, .tx_buffer = tx};

    cs_low();
    esp_err_t r = spi_device_transmit(flash_spi_handle, &t);
    cs_high();

    return (r == ESP_OK) ? FLASH_OK : FLASH_SPI_ERROR;
}


/* ================= 等待完成 ================= */

static FlashStatus FlashWaitReady(void)
{
    for (int i = 0; i < 5000; i++) {
        uint8_t s = FlashGetFeature(REG_STATUS);
        if (!(s & STATUS_OIP)) {
            if (s & (STATUS_E_FAIL | STATUS_P_FAIL))
                return FLASH_OP_ERROR;
            return FLASH_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return FLASH_TIMEOUT;
}

/* ================= 写使能 ================= */

static FlashStatus FlashWriteEnable(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    spi_transaction_t t = {.length = 8, .tx_buffer = &cmd};
    cs_low();
    spi_device_transmit(flash_spi_handle, &t);
    cs_high();
    if (!(FlashGetFeature(REG_STATUS) & STATUS_WEL))
        return FLASH_WRITE_ENABLE_ERR;

    return FLASH_OK;
}

/* ================= SPI 初始化 ================= */

static esp_err_t SpiInit(void)
{
    static bool initialized = false;
    if (initialized) return ESP_OK;
    initialized = true;

    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << PIN_CS) | (1ULL << PIN_WP) | (1ULL << PIN_HOLD),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    gpio_set_level(PIN_CS, 1);
    gpio_set_level(PIN_WP, 1);
    gpio_set_level(PIN_HOLD, 1);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .max_transfer_sz = 2112 + 16,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));


    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 5,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &flash_spi_handle));

    return ESP_OK;
}


/* ================= 复位 ================= */

static FlashStatus FlashReset(void)
{
    uint8_t cmd;
    spi_transaction_t t = {.length = 8};

    cmd = CMD_RESET_ENABLE;
    t.tx_buffer = &cmd;
    cs_low(); spi_device_transmit(flash_spi_handle, &t); cs_high();

    cmd = CMD_RESET_EXECUTE;
    t.tx_buffer = &cmd;
    cs_low(); spi_device_transmit(flash_spi_handle, &t); cs_high();

    vTaskDelay(pdMS_TO_TICKS(2));
    return FlashWaitReady();
}

/* ================= ID 校验 ================= */

static FlashStatus FlashVerifyId(void)
{
    uint8_t tx[5] = { CMD_READ_ID, 0, 0, 0, 0 };
    uint8_t rx[5] = { 0 };

    spi_transaction_t t = {
        .length = 40,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    cs_low();
    if (spi_device_transmit(flash_spi_handle, &t) != ESP_OK) {
        cs_high();
        ESP_LOGE(TAG, "SPI transmit failed when read JEDEC ID");
        return FLASH_SPI_ERROR;
    }
    cs_high();

    uint8_t mfg = rx[2];
    uint8_t dev_msb = rx[3];
    uint8_t dev_lsb = rx[4];

    ESP_LOGI(TAG, "JEDEC ID = %02X %02X %02X", mfg, dev_msb, dev_lsb);

    if (mfg != JEDEC_MFG_ID || dev_msb != JEDEC_DEV_MSB || dev_lsb != JEDEC_DEV_LSB) {
        ESP_LOGE(TAG, "JEDEC ID mismatch (expected: %02X %02X %02X)",
                 JEDEC_MFG_ID, JEDEC_DEV_MSB, JEDEC_DEV_LSB);
        return FLASH_ID_MISMATCH;
    }

    return FLASH_OK;
}

/* ================= 关闭写保护 ================= */

static FlashStatus FlashDisableWriteProtect(void)
{
    FlashStatus r = FlashWriteEnable();
    if (r != FLASH_OK) return r;

    r = FlashSetFeature(REG_PROTECT, 0x00);
    if (r != FLASH_OK) return r;

    return FlashWaitReady();

}

static FlashStatus FlashPageToCache(uint32_t page)
{
    uint8_t cmd[4] = {
        CMD_PAGE_READ,
        (page >> 16) & 0xFF,
        (page >> 8) & 0xFF,
        page & 0xFF
    };

    spi_transaction_t t = {.length = 32, .tx_buffer = cmd};

    cs_low();
    spi_device_transmit(flash_spi_handle, &t);
    cs_high();

    return FlashWaitReady();
}

FlashStatus FlashRead(uint32_t page, uint8_t *buf, uint32_t len)
{
    if (!buf || len == 0 || len > W25N_PAGE_SIZE_MAIN)
        return FLASH_INVALID_PARAM;

    FlashStatus ret = FlashPageToCache(page);
    if (ret != FLASH_OK) return ret;

    /* column = 0 */
    uint8_t cmd[4] = { CMD_READ_CACHE, 0x00, 0x00, 0x00 };

    cs_low();

    spi_transaction_t t_cmd = {
        .length = 32,
        .tx_buffer = cmd,
        //.flags = SPI_TRANS_CS_KEEP_ACTIVE   
    };
    spi_device_transmit(flash_spi_handle, &t_cmd);

    spi_transaction_t t_data = {
       .length = len * 8,
       .rx_buffer = buf
    };
    spi_device_transmit(flash_spi_handle, &t_data);


    cs_high();

    /* 检查 ECC 状态 */
    uint8_t s = FlashGetFeature(REG_STATUS);
    if ((s & STATUS_ECC_MASK) == 0x20)
        ESP_LOGW(TAG, "ECC corrected (page=%" PRIu32 ")", page);
    else if ((s & STATUS_ECC_MASK) == 0x30){
        ESP_LOGE(TAG, "ECC failure (page=%" PRIu32 ")", page);
        return FLASH_CRC_ERROR;
    }
    return FLASH_OK;
}

FlashStatus FlashReadOOB(uint32_t page, uint8_t *oob_buf, uint32_t len)
{
    if (!oob_buf || len == 0 || len > W25N_PAGE_SIZE_OOB)
        return FLASH_INVALID_PARAM;

    FlashStatus ret = FlashPageToCache(page);
    if (ret != FLASH_OK) return ret;

    /* OOB column = 2048 = 0x0800 */
    uint16_t column = W25N_PAGE_SIZE_MAIN;

    uint8_t cmd[4] = {
        CMD_READ_CACHE,
        (column >> 8) & 0xFF,
        column & 0xFF,
        0x00
    };

    cs_low();

    spi_transaction_t t_cmd = {
        .length = 32,
        .tx_buffer = cmd,
        //.flags = SPI_TRANS_CS_KEEP_ACTIVE
    };
    spi_device_transmit(flash_spi_handle, &t_cmd);

    spi_transaction_t t_data = {
        .length = len * 8,
        .rx_buffer = oob_buf
    };
    spi_device_transmit(flash_spi_handle, &t_data);


    cs_high();

    return FLASH_OK;
}
static FlashStatus FlashCheckBadBlock(uint32_t page) {
    uint32_t block = page / W25N_BLOCK_SIZE_PAGE;
    uint32_t page0 = block * W25N_BLOCK_SIZE_PAGE;
    uint32_t page1 = page0 + 1;

    uint8_t oob[W25N_PAGE_SIZE_OOB];

    // 第一步：读取Block第0页OOB（读取失败则先擦除）
    if (FlashReadOOB(page0, oob, 1) != FLASH_OK) {
        ESP_LOGW(TAG, "Read OOB failed, erase block first (block=%" PRIu32 ")", block);
        FlashEraseBlock(page0); // 先擦除块
        if (FlashReadOOB(page0, oob, 1) != FLASH_OK) {
            ESP_LOGE(TAG, "Bad block check failed (page0=%" PRIu32 ")", page0);
            return FLASH_OP_ERROR;
        }
    }

    // 正确规则：仅当OOB[0] = 0x00 时才是坏块（0xFF是正常，其他值先擦除）
    if (oob[0] == W25N_BAD_BLOCK_MARK) { // 0x00 = 坏块标记
        ESP_LOGW(TAG, "Bad block detected (block=%" PRIu32 ")", block);
        return FLASH_BAD_BLOCK;
    }

    // 非0xFF/非0x00：擦除后再验证
    if (oob[0] != 0xFF) {
        FlashEraseBlock(page0);
        if (FlashReadOOB(page0, oob, 1) != FLASH_OK || oob[0] == W25N_BAD_BLOCK_MARK) {
            ESP_LOGW(TAG, "Bad block after erase (block=%" PRIu32 ")", block);
            return FLASH_BAD_BLOCK;
        }
    }

    // 校验第1页（可选，原厂仅要求校验第0页）
    if (FlashReadOOB(page1, oob, 1) == FLASH_OK && oob[0] == W25N_BAD_BLOCK_MARK) {
        ESP_LOGW(TAG, "Bad block detected (page1=%" PRIu32 ")", page1);
        return FLASH_BAD_BLOCK;
    }

    return FLASH_OK;
}

static FlashStatus FlashProgLoad(const uint8_t *buf, uint32_t len)
{
    uint8_t cmd[3] = { CMD_PROG_LOAD, 0x00, 0x00 };

    cs_low();

    spi_transaction_t t_cmd = {
        .length = 24,
        .tx_buffer = cmd,
        //.flags = SPI_TRANS_CS_KEEP_ACTIVE
    };

    if (spi_device_transmit(flash_spi_handle, &t_cmd) != ESP_OK) {
        cs_high();
        return FLASH_SPI_ERROR;
    }

    spi_transaction_t t_data = {.length = len * 8, .tx_buffer = buf};
    if (spi_device_transmit(flash_spi_handle, &t_data) != ESP_OK) {
        cs_high();
        return FLASH_SPI_ERROR;
    }

    cs_high();
    return FLASH_OK;
}

static FlashStatus FlashProgExecute(uint32_t page)
{
    uint8_t exe[4] = {
        CMD_PROG_EXECUTE,
        (page >> 16) & 0xFF,
        (page >> 8) & 0xFF,
        page & 0xFF
    };

    spi_transaction_t t = {.length = 32, .tx_buffer = exe};

    cs_low();
    if (spi_device_transmit(flash_spi_handle, &t) != ESP_OK) {
        cs_high();
        return FLASH_SPI_ERROR;
    }
    cs_high();

    FlashStatus ret = FlashWaitReady();
    if (ret != FLASH_OK) return ret;

    uint8_t s = FlashGetFeature(REG_STATUS);
    if (s & STATUS_P_FAIL) {
        ESP_LOGE(TAG, "Program failed (page=%" PRIu32 ")", page);
        return FLASH_OP_ERROR;
    }

    return FLASH_OK;
}


FlashStatus FlashEraseBlock(uint32_t page)
{
    if (page % W25N_BLOCK_SIZE_PAGE != 0)
        return FLASH_INVALID_PARAM;

    FlashStatus ret = FlashWriteEnable();
    if (ret != FLASH_OK) return ret;

    uint8_t cmd[4] = {
        CMD_BLOCK_ERASE,
        (page >> 16) & 0xFF,
        (page >> 8) & 0xFF,
        page & 0xFF
    };

    spi_transaction_t t = {.length = 32, .tx_buffer = cmd};

    cs_low();
    spi_device_transmit(flash_spi_handle, &t);
    cs_high();

    ret = FlashWaitReady();
    if (ret != FLASH_OK) return ret;

    uint8_t s = FlashGetFeature(REG_STATUS);
    if (s & STATUS_E_FAIL) {
        ESP_LOGE(TAG, "Erase failed (block page=%" PRIu32 ")", page);
        return FLASH_OP_ERROR;
    }

    ESP_LOGD(TAG, "Block erase OK (%" PRIu32 ")", page);
    return FLASH_OK;
}

FlashStatus FlashWrite(uint32_t page, const uint8_t *buf, uint32_t len) {
    if (!buf || len == 0 || len > W25N_PAGE_SIZE_MAIN)
        return FLASH_INVALID_PARAM;

    uint32_t block_start_page = (page / W25N_BLOCK_SIZE_PAGE) * W25N_BLOCK_SIZE_PAGE;

    // 第一步：检测坏块（检测时会自动擦除非坏块）
    FlashStatus ret = FlashCheckBadBlock(page);
    if (ret != FLASH_OK) {
        ESP_LOGE(TAG, "Write failed: bad block (page=%" PRIu32 ")", page);
        return ret;
    }

    // 第二步：强制擦除块（无论是否块首地址）
    ret = FlashEraseBlock(block_start_page);
    if (ret != FLASH_OK) {
        ESP_LOGE(TAG, "Erase block failed (page=%" PRIu32 ")", block_start_page);
        return ret;
    }

    // 后续逻辑不变
    ret = FlashWriteEnable();
    if (ret != FLASH_OK) return ret;

    ret = FlashProgLoad(buf, len);
    if (ret != FLASH_OK) return ret;

    ret = FlashProgExecute(page);
    if (ret == FLASH_OK)
        ESP_LOGD(TAG, "Page write OK (%" PRIu32 ")", page);

    return ret;
}

FlashStatus FlashStorageInit(void)
{
    FlashStatus ret;

    if (SpiInit() != ESP_OK) {
        ESP_LOGE(TAG, "SPI init failed");
        return FLASH_SPI_INIT_ERR;
    }

    ret = FlashReset();
    if (ret != FLASH_OK) {
        ESP_LOGE(TAG, "Flash reset failed");
        return ret;
    }

    ret = FlashVerifyId();
    if (ret != FLASH_OK) {
        ESP_LOGE(TAG, "JEDEC ID mismatch");
        return ret;
    }

    ret = FlashDisableWriteProtect();
    if (ret != FLASH_OK) {
        ESP_LOGE(TAG, "Write protect disable failed");
        return ret;
    }

    ESP_LOGI(TAG, "Flash init OK");
    return FLASH_OK;
}


// Flash数据存储任务（核心修复：坏块处理+批量写入逻辑）
void AppDataStoreTask(void *pvParameters) {
    ESP_LOGI(TAG, "FlashStoreTask started");

    // 初始化Flash
    FlashStatus ret = FlashStorageInit();
    if (ret != FLASH_OK) {
        ESP_LOGE(TAG, "Flash init failed (ret: %d)", ret);
        vTaskDelete(NULL);
        return;
    }

    TxPkg_t pkg;
    // 重置批量缓存状态（避免脏数据）
    memset(batch_buf, 0, sizeof(batch_buf));
    batch_count = 0;

    while (1) {
        if (xQueueReceive(FlashQueue, &pkg, portMAX_DELAY)) {
            // 校验数据包长度（严格限制512字节）
            if (pkg.length == 0 || pkg.length > DATA_PACKAGE_SIZE) {
                ESP_LOGE(TAG, "Invalid pkg length (size=%zu), must be 1~512 bytes", pkg.length);
                continue;
            }

            // 填充数据包到批量缓冲区（不足512字节的补0）
            uint8_t pkg_full[DATA_PACKAGE_SIZE] = {0};
            memcpy(pkg_full, pkg.data, pkg.length);
            memcpy(&batch_buf[batch_count * DATA_PACKAGE_SIZE], pkg_full, DATA_PACKAGE_SIZE);
            batch_count++;

            // 批量缓冲区满（4个512字节包=2048字节，1个Flash页），执行写入
            if (batch_count >= FLASH_BATCH_SIZE) {
                // 跳过超出Flash总页数的写入（防止越界）
                if (g_flash_state.write_page >= W25N_TOTAL_PAGES) {
                    ESP_LOGE(TAG, "Flash storage full (total pages=%" PRIu32 ")", W25N_TOTAL_PAGES);
                    batch_count = 0;
                    memset(batch_buf, 0, sizeof(batch_buf));
                    continue;
                }

                // 执行Flash页写入
                ret = FlashWrite(g_flash_state.write_page, batch_buf, W25N_PAGE_SIZE_MAIN);
                if (ret == FLASH_OK) {
                    ESP_LOGI(TAG, "Batch write success (4x512) to page %" PRIu32, g_flash_state.write_page);
                    g_flash_state.write_page++; // 正常写入，页号+1
                    batch_count = 0;
                    memset(batch_buf, 0, sizeof(batch_buf));
                } else {
                    ESP_LOGE(TAG, "Batch write failed (page=%" PRIu32 ", ret=%d)", g_flash_state.write_page, ret);
                    // 坏块处理：跳到下一个块的起始页（避免重复检测同一坏块）
                    if (ret == FLASH_BAD_BLOCK) {
                        uint32_t current_block = g_flash_state.write_page / W25N_BLOCK_SIZE_PAGE;
                        g_flash_state.write_page = (current_block + 1) * W25N_BLOCK_SIZE_PAGE;
                        ESP_LOGW(TAG, "Skip bad block %" PRIu32 ", next page: %" PRIu32, 
                                 current_block, g_flash_state.write_page);
                    }
                    // 重置批量缓冲区（无论是否坏块，都清空缓存）
                    batch_count = 0;
                    memset(batch_buf, 0, sizeof(batch_buf));
                }
            }

            ESP_LOGD(TAG, "Cached pkg %zu (total cached: %zu bytes / %zu pages)", 
                     batch_count, batch_count * DATA_PACKAGE_SIZE, 
                     (batch_count * DATA_PACKAGE_SIZE + W25N_PAGE_SIZE_MAIN - 1) / W25N_PAGE_SIZE_MAIN);
        }
    }

    vTaskDelete(NULL);
}

// 多页读取函数（修复：边界检查+内存安全）
FlashStatus FlashReadMultiPage(uint32_t start_page, uint32_t page_count, uint8_t *buf, uint32_t *read_len) {
    // 入参合法性校验
    if (!buf || !read_len || page_count == 0) {
        ESP_LOGE(TAG, "Invalid params for multi-page read (buf=%p, len=%p, count=%" PRIu32 ")", 
                 buf, read_len, page_count);
        return FLASH_INVALID_PARAM;
    }
    // 页号越界检查
    if (start_page >= W25N_TOTAL_PAGES) {
        ESP_LOGE(TAG, "Start page %" PRIu32 " out of range (max=%" PRIu32 ")", 
                 start_page, W25N_TOTAL_PAGES - 1);
        return FLASH_INVALID_PARAM;
    }
    // 修正读取页数（避免超出Flash总页数）
    uint32_t actual_page_count = MIN(page_count, W25N_TOTAL_PAGES - start_page);
    if (actual_page_count != page_count) {
        ESP_LOGW(TAG, "Read page count truncated (requested=%" PRIu32 ", actual=%" PRIu32 ")", 
                 page_count, actual_page_count);
    }

    *read_len = 0;
    uint8_t page_buf[W25N_PAGE_SIZE_MAIN]; // 栈缓冲区（避免堆分配开销）

    for (uint32_t i = 0; i < actual_page_count; i++) {
        uint32_t current_page = start_page + i;
        // 读取单页数据
        FlashStatus ret = FlashRead(current_page, page_buf, W25N_PAGE_SIZE_MAIN);
        if (ret != FLASH_OK) {
            ESP_LOGE(TAG, "Read page %" PRIu32 " failed (ret: %d)", current_page, ret);
            return ret;
        }
        // 拷贝到输出缓冲区（内存安全：避免越界）
        memcpy(buf + i * W25N_PAGE_SIZE_MAIN, page_buf, W25N_PAGE_SIZE_MAIN);
        *read_len += W25N_PAGE_SIZE_MAIN;
    }

    ESP_LOGI(TAG, "Multi-page read success: start=%" PRIu32 ", count=%" PRIu32 ", total bytes=%" PRIu32, 
             start_page, actual_page_count, *read_len);
    return FLASH_OK;
}

// 按512字节数据包粒度读取Flash（修复：偏移计算+内存安全）
FlashStatus FlashReadDataPackages(uint32_t start_pkg_idx, uint32_t pkg_count, uint8_t *out_buf, uint32_t *out_len) {
    // 入参合法性校验
    if (!out_buf || !out_len || pkg_count == 0) {
        ESP_LOGE(TAG, "Invalid params for pkg read (buf=%p, len=%p, count=%" PRIu32 ")", 
                 out_buf, out_len, pkg_count);
        return FLASH_INVALID_PARAM;
    }

    *out_len = 0;
    uint32_t pkg_per_page = W25N_PAGE_SIZE_MAIN / DATA_PACKAGE_SIZE; // 每页4个数据包
    uint32_t total_needed_bytes = pkg_count * DATA_PACKAGE_SIZE;

    // 计算起始页和页内偏移（数据包粒度）
    uint32_t start_page = start_pkg_idx / pkg_per_page;
    uint32_t page_offset = (start_pkg_idx % pkg_per_page) * DATA_PACKAGE_SIZE;
    // 计算需要读取的页数（向上取整）
    uint32_t need_page_count = (total_needed_bytes + page_offset + W25N_PAGE_SIZE_MAIN - 1) / W25N_PAGE_SIZE_MAIN;

    // 堆分配临时缓冲区（自动适配读取页数）
    uint32_t temp_buf_size = need_page_count * W25N_PAGE_SIZE_MAIN;
    uint8_t *page_buf = (uint8_t *)heap_caps_malloc(temp_buf_size, MALLOC_CAP_DEFAULT);
    if (!page_buf) {
        ESP_LOGE(TAG, "Malloc failed for pkg read (need %" PRIu32 " bytes)", temp_buf_size);
        return FLASH_OP_ERROR;
    }

    // 读取多页数据到临时缓冲区
    uint32_t read_bytes = 0;
    FlashStatus ret = FlashReadMultiPage(start_page, need_page_count, page_buf, &read_bytes);
    if (ret != FLASH_OK) {
        free(page_buf);
        ESP_LOGE(TAG, "Multi-page read failed (ret: %d)", ret);
        return ret;
    }

    // 提取目标数据包（跳过页内偏移，避免内存越界）
    uint32_t copy_len = MIN(total_needed_bytes, read_bytes - page_offset);
    memcpy(out_buf, page_buf + page_offset, copy_len);
    *out_len = copy_len;

    // 释放临时缓冲区
    free(page_buf);

    // 校验读取结果（是否满足需求）
    if (*out_len != total_needed_bytes) {
        ESP_LOGW(TAG, "Pkg read incomplete (requested=%" PRIu32 " bytes, actual=%" PRIu32 ")", 
                 total_needed_bytes, *out_len);
    } else {
        ESP_LOGI(TAG, "Pkg read success: start=%" PRIu32 ", count=%" PRIu32 ", total bytes=%" PRIu32, 
                 start_pkg_idx, pkg_count, *out_len);
    }

    return FLASH_OK;
}




