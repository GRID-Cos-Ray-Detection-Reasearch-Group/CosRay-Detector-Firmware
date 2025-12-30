#include "flashstorage.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>
#include <inttypes.h>  // 引入标准整数格式宏
#include "driver/gpio.h"

static const char *TAG = "FlashStorage";

// SPI设备句柄
static spi_device_handle_t flash_spi_handle;

// 当前写入页地址（全局跟踪，掉电后需从NVS恢复，此处简化处理）
static uint32_t s_current_page = 0;

// W25N02KV指令集
#define W25N_CMD_RESET_ENABLE          0x66    //允许复位
#define W25N_CMD_RESET_EXECUTE         0x99    // 软件复位
#define W25N_CMD_READ_ID        0x9F    // 读取设备ID
#define W25N_CMD_PAGE_PROGRAM   0x02    // 页编程
#define W25N_CMD_BLOCK_ERASE    0xD8    // 块擦除（128KB）
#define W25N_CMD_READ_DATA      0x03    // 读取数据
#define W25N_CMD_READ_STATUS    0x05    // 读取状态寄存器
#define W25N_CMD_WRITE_ENABLE   0x06    // 写使能

// 设备ID验证（W25N02KV的制造商ID=0xEF，设备ID=0xAA22）
#define W25N_MANUFACTURER_ID    0xEF
#define W25N_DEVICE_ID_MSB      0xAA
#define W25N_DEVICE_ID_LSB      0x22

// 等待Flash操作完成（轮询状态寄存器）
static FlashStatus FlashWaitReady(void) {
    uint8_t status;
    spi_transaction_t t = {
        .length = 8,          // 1字节指令 + 1字节状态
        .tx_buffer = (uint8_t[]){W25N_CMD_READ_STATUS, 0x00},
        .rx_buffer = &status,
    };

    while (1) {
        if (spi_device_transmit(flash_spi_handle, &t) != ESP_OK) {
            return FLASH_READ_FAILED;
        }
        if (!(status & 0x01)) { // 忙标志位（BIT0）为0表示就绪
            return FLASH_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// 发送写使能指令
static FlashStatus FlashWriteEnable(void) {
    uint8_t cmd = W25N_CMD_WRITE_ENABLE;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };

    if (spi_device_transmit(flash_spi_handle, &t) != ESP_OK) {
        return FLASH_WRITE_FAILED;
    }
    return FlashWaitReady();
}

// 初始化SPI总线
static esp_err_t SpiBusInit(void) {
    spi_bus_config_t bus_cfg = {
        .miso_io_num = 47,    // MISO引脚
        .mosi_io_num = 33,    // MOSI引脚
        .sclk_io_num = 48,    // SCLK引脚
        .quadwp_io_num = -1,  
        .quadhd_io_num = -1,
        .max_transfer_sz = W25N02KV_PAGE_SIZE + 4, // 最大传输：页数据+4字节指令
    };

    // 初始化SPI2主机
    return spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
}

// 初始化SPI设备
static esp_err_t SpiDeviceInit(void) {
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 40MHz（最高支持104MHz）
        .mode = 0,                          // SPI模式0（CPOL=0, CPHA=0）
        .spics_io_num = 35,                  // 根据硬件修改CS引脚
        .queue_size = 10,                   // 事务队列大小
        .pre_cb = NULL,
    };

    return spi_bus_add_device(SPI2_HOST, &dev_cfg, &flash_spi_handle);
}

// 读取设备ID并验证
static FlashStatus FlashVerifyId(void)
{
    uint8_t tx[4] = {
        W25N_CMD_READ_ID,
        0x00,  // dummy
        0x00,  // dummy
        0x00   // dummy
    };

    uint8_t rx[4] = {0};

    spi_transaction_t t = {
        .length = 4 * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(flash_spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "READ ID transmit failed: %s", esp_err_to_name(ret));
        return FLASH_READ_FAILED;
    }

    ESP_LOGI(TAG,
        "Flash ID raw: %02X %02X %02X %02X",
        rx[0], rx[1], rx[2], rx[3]);

    // rx[1..3] 是有效 ID
    if (rx[1] != W25N_MANUFACTURER_ID ||
        rx[2] != W25N_DEVICE_ID_MSB ||
        rx[3] != W25N_DEVICE_ID_LSB) {
        return FLASH_ID_MISMATCH;
    }

    return FLASH_OK;
}




// 软件复位
static FlashStatus FlashReset(void) {

   uint8_t cmd1 = W25N_CMD_RESET_ENABLE;
    spi_transaction_t t1 = {
        .length = 8,
        .tx_buffer = &cmd1,
    };
    if (spi_device_transmit(flash_spi_handle, &t1) != ESP_OK) {
        return FLASH_INIT_FAILED;
    }

    uint8_t cmd2 = W25N_CMD_RESET_EXECUTE;
    spi_transaction_t t2 = {
        .length = 8,
        .tx_buffer = &cmd2,
    };



    if (spi_device_transmit(flash_spi_handle, &t2) != ESP_OK) {
        return FLASH_INIT_FAILED;
    }

    if (FlashWaitReady() != FLASH_OK) {
        ESP_LOGE(TAG, "Flash not ready after reset");
        return FLASH_INIT_FAILED;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 等待复位完成
    return FLASH_OK;
}

// 初始化Flash存储系统
FlashStatus FlashStorageInit(void) {

    // 新增：初始化HOLD（IO38）和WP（IO36）引脚，上拉保持高电平
    gpio_config_t hold_wp_cfg = {
        .pin_bit_mask = (1ULL << 38) | (1ULL << 36),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&hold_wp_cfg);
    gpio_set_level(38, 1); // HOLD = 1
    gpio_set_level(36, 1); // WP   = 1
    
    if (gpio_config(&hold_wp_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HOLD/WP pin init failed");
        return FLASH_INIT_FAILED;
    }

    // 初始化SPI总线
    if (SpiBusInit() != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed");
        return FLASH_INIT_FAILED;
    }

    // 初始化SPI设备
    if (SpiDeviceInit() != ESP_OK) {
        ESP_LOGE(TAG, "SPI device init failed");
        return FLASH_INIT_FAILED;
    }

    // 复位设备
    if (FlashReset() != FLASH_OK) {
        ESP_LOGE(TAG, "Flash reset failed");
        return FLASH_INIT_FAILED;
    }

    // 验证设备ID
    if (FlashVerifyId() != FLASH_OK) {
        ESP_LOGE(TAG, "Flash ID mismatch");
        return FLASH_ID_MISMATCH;
    }

    // 扫描坏块（首次初始化时执行）
    for (uint32_t block = 0; block < W25N02KV_TOTAL_BLOCKS; block++) {
        if (FlashIsBadBlock(block)) {
            // uint32_t类型使用PRIu32格式符
            ESP_LOGW(TAG, "Bad block found: %" PRIu32, block);
        }
    }

    ESP_LOGI(TAG, "Flash storage initialized successfully");
    return FLASH_OK;
}

// 检查块是否为坏块（坏块标记在块的第0页第2047字节）
bool FlashIsBadBlock(uint32_t block_addr) {
    if (block_addr >= W25N02KV_TOTAL_BLOCKS) {
        return true; // 无效块地址视为坏块
    }

    uint32_t page_addr = block_addr * W25N02KV_BLOCK_SIZE;
    uint8_t bad_block_flag;
    FlashStatus ret = FlashRead(page_addr, &bad_block_flag, 1);

    // 坏块标记为0x00（非0xFF）
    return (ret != FLASH_OK) || (bad_block_flag != 0xFF);
}

// 擦除指定块
FlashStatus FlashEraseBlock(uint32_t block_addr) {
    if (block_addr >= W25N02KV_TOTAL_BLOCKS) {
        return FLASH_INVALID_ADDR;
    }
    if (FlashIsBadBlock(block_addr)) {
        return FLASH_BAD_BLOCK;
    }

    // 发送写使能
    if (FlashWriteEnable() != FLASH_OK) {
        return FLASH_WRITE_FAILED;
    }

    // 块擦除指令 + 地址（24位）
    uint8_t cmd[4] = {
        W25N_CMD_BLOCK_ERASE,
        (block_addr >> 16) & 0xFF, // 地址高位（块地址需左移16位，因每块64页=0x40）
        (block_addr >> 8) & 0xFF,
        block_addr & 0xFF
    };

    spi_transaction_t t = {
        .length = 4 * 8,
        .tx_buffer = cmd,
    };

    if (spi_device_transmit(flash_spi_handle, &t) != ESP_OK) {
        return FLASH_ERASE_FAILED;
    }

    // 等待擦除完成（块擦除约需200ms）
    return FlashWaitReady();
}

// 写入一页数据（需确保页所在块已擦除）
FlashStatus FlashWrite(uint32_t page_addr, const uint8_t *data, uint32_t len) {
    if (page_addr >= W25N02KV_TOTAL_BLOCKS * W25N02KV_BLOCK_SIZE) {
        return FLASH_INVALID_ADDR;
    }
    if (len > W25N02KV_PAGE_SIZE) {
        return FLASH_INVALID_ADDR; // 超出页大小
    }

    // 发送写使能
    if (FlashWriteEnable() != FLASH_OK) {
        return FLASH_WRITE_FAILED;
    }

    // 页编程指令 + 地址（24位）
    uint8_t cmd[4] = {
        W25N_CMD_PAGE_PROGRAM,
        (page_addr >> 16) & 0xFF,
        (page_addr >> 8) & 0xFF,
        page_addr & 0xFF
    };

    // 拼接指令和数据（使用DMA传输）
    spi_transaction_t t = {
        .length = 4 * 8 + len * 8, // 指令(4字节) + 数据(len字节)
        .tx_buffer = cmd,
    };

    // 先发送指令，再发送数据（分两步传输）
    if (spi_device_transmit(flash_spi_handle, &t) != ESP_OK) {
        return FLASH_WRITE_FAILED;
    }

    // 发送数据部分
    t.length = len * 8;
    t.tx_buffer = data;
    if (spi_device_transmit(flash_spi_handle, &t) != ESP_OK) {
        return FLASH_WRITE_FAILED;
    }

    // 等待写入完成
    return FlashWaitReady();
}

// 从指定页读取数据
FlashStatus FlashRead(uint32_t page_addr, uint8_t *data, uint32_t len) {
    if (page_addr >= W25N02KV_TOTAL_BLOCKS * W25N02KV_BLOCK_SIZE) {
        return FLASH_INVALID_ADDR;
    }
    if (len > W25N02KV_PAGE_SIZE) {
        return FLASH_INVALID_ADDR; // 超出页大小
    }

    // 读数据指令 + 地址（24位）
    uint8_t cmd[4] = {
        W25N_CMD_READ_DATA,
        (page_addr >> 16) & 0xFF,
        (page_addr >> 8) & 0xFF,
        page_addr & 0xFF
    };

    spi_transaction_t t = {
        .length = 4 * 8 + len * 8, // 指令(4字节) + 数据(len字节)
        .tx_buffer = cmd,
        .rx_buffer = data,
    };

    if (spi_device_transmit(flash_spi_handle, &t) != ESP_OK) {
        return FLASH_READ_FAILED;
    }

    return FLASH_OK;
}

// 数据存储任务：从队列读取数据并写入Flash
void AppDataStoreTask(void *pvParameters) {
    ESP_LOGI(TAG, "Data storage task started");

    // 初始化Flash
    if (FlashStorageInit() != FLASH_OK) {
        ESP_LOGE(TAG, "Flash init failed, task exiting");
        vTaskDelete(NULL);
    }

    uint8_t page_buf[W25N02KV_PAGE_SIZE] = {0}; // 页缓存
    size_t buf_offset = 0;

    while (1) {
        TxPkg_t tx_pkg;
        // 从队列读取待存储数据
        if (xQueueReceive(FlashQueue, &tx_pkg, portMAX_DELAY)) {
            // 检查数据是否超出页缓存
            if (buf_offset + tx_pkg.length > W25N02KV_PAGE_SIZE) {
                // 缓存满，写入当前页
                // uint32_t用PRIu32，size_t用%zu
                ESP_LOGI(TAG, "Writing page %" PRIu32 " (size: %zu bytes)", s_current_page, buf_offset);
                if (FlashWrite(s_current_page, page_buf, buf_offset) != FLASH_OK) {
                    ESP_LOGE(TAG, "Failed to write page %" PRIu32, s_current_page);
                }

                // 移动到下一页，检查是否需要擦除块
                s_current_page++;
                uint32_t current_block = s_current_page / W25N02KV_BLOCK_SIZE;

                // 若为新块的第一页，先擦除块（跳过坏块）
                if (s_current_page % W25N02KV_BLOCK_SIZE == 0) {
                    if (FlashIsBadBlock(current_block)) {
                        ESP_LOGW(TAG, "Skipping bad block %" PRIu32, current_block);
                        s_current_page += W25N02KV_BLOCK_SIZE; // 跳过整个坏块
                        current_block = s_current_page / W25N02KV_BLOCK_SIZE;
                    }
                    ESP_LOGI(TAG, "Erasing block %" PRIu32, current_block);
                    if (FlashEraseBlock(current_block) != FLASH_OK) {
                        ESP_LOGE(TAG, "Failed to erase block %" PRIu32, current_block);
                    }
                }

                // 重置缓存
                memset(page_buf, 0, W25N02KV_PAGE_SIZE);
                buf_offset = 0;
            }

            // 将数据写入缓存
            memcpy(page_buf + buf_offset, tx_pkg.data, tx_pkg.length);
            buf_offset += tx_pkg.length;
            ESP_LOGI(TAG, "Cached data (offset: %zu, total: %zu bytes)", buf_offset, tx_pkg.length);
        }
    }
}