#include "flashstorage.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include <string.h>

static const char *TAG = "FlashStorage";

/* =====================================================
 * 全局对象
 * ===================================================== */
spi_device_handle_t flash_spi_handle;
extern QueueHandle_t FlashQueue;

/* =====================================================
 * W25N01KV / W25N02KV 指令
 * ===================================================== */
#define CMD_RESET_ENABLE        0x66
#define CMD_RESET_EXECUTE       0x99
#define CMD_READ_ID             0x9F
#define CMD_WRITE_ENABLE        0x06
#define CMD_READ_STATUS         0x05
#define CMD_PAGE_READ           0x13
#define CMD_READ_CACHE          0x03
#define CMD_PROG_LOAD           0x02
#define CMD_PROG_EXECUTE        0x10

/* =====================================================
 * JEDEC ID（W25N01KV）
 * ===================================================== */
#define JEDEC_MFG_ID            0xEF
#define JEDEC_DEV_MSB           0xAE
#define JEDEC_DEV_LSB           0x21

/* =====================================================
 * 手动 CS
 * ===================================================== */
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

/* =====================================================
 * 等待 BUSY 清零
 * ===================================================== */
static FlashStatus FlashWaitReady(void)
{
    uint8_t tx[2] = { CMD_READ_STATUS, 0xC0 };
    uint8_t rx[2] = { 0 };

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    int timeout = 1000;
    while (timeout--) {
        cs_low();
        spi_device_transmit(flash_spi_handle, &t);
        cs_high();

        if ((rx[1] & 0x01) == 0) {
            return FLASH_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGE(TAG, "Wait ready timeout");
    return FLASH_TIMEOUT;
}

/* =====================================================
 * 写使能
 * ===================================================== */
static void FlashWriteEnable(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };

    cs_low();
    spi_device_transmit(flash_spi_handle, &t);
    cs_high();
}

/* =====================================================
 * SPI 初始化（只允许初始化一次）
 * ===================================================== */
static esp_err_t SpiInit(void)
{
    static bool initialized = false;
    if (initialized) {
        return ESP_OK;
    }
    initialized = true;

    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << PIN_CS) | (1ULL << PIN_WP) | (1ULL << PIN_HOLD),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_cfg);

    gpio_set_level(PIN_CS, 1);
    gpio_set_level(PIN_WP, 1);
    gpio_set_level(PIN_HOLD, 1);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = W25N_PAGE_SIZE_MAIN + 16,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = 0,   // 必须 FULL DUPLEX
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &flash_spi_handle));

    return ESP_OK;
}

/* =====================================================
 * 软件复位
 * ===================================================== */
static FlashStatus FlashReset(void)
{
    uint8_t cmd;

    cmd = CMD_RESET_ENABLE;
    cs_low();
    spi_device_transmit(flash_spi_handle,
        &(spi_transaction_t){ .length = 8, .tx_buffer = &cmd });
    cs_high();

    cmd = CMD_RESET_EXECUTE;
    cs_low();
    spi_device_transmit(flash_spi_handle,
        &(spi_transaction_t){ .length = 8, .tx_buffer = &cmd });
    cs_high();

    vTaskDelay(pdMS_TO_TICKS(2));
    return FlashWaitReady();
}

/* =====================================================
 * 读取 JEDEC ID（关键修正）
 * ===================================================== */
static FlashStatus FlashVerifyId(void)
{
    uint8_t tx[5] = { CMD_READ_ID, 0, 0, 0, 0 };
    uint8_t rx[5] = { 0 };

    cs_low();
    spi_device_transmit(flash_spi_handle,
        &(spi_transaction_t){
            .length = 40,
            .tx_buffer = tx,
            .rx_buffer = rx,
        });
    cs_high();

    uint8_t mfg = rx[1];
    uint8_t dev_msb = rx[2];
    uint8_t dev_lsb = rx[3];

    ESP_LOGI(TAG, "JEDEC ID = %02X %02X %02X",
             mfg, dev_msb, dev_lsb);

    if (mfg != JEDEC_MFG_ID ||
        dev_msb != JEDEC_DEV_MSB ||
        dev_lsb != JEDEC_DEV_LSB) {
        ESP_LOGE(TAG, "JEDEC mismatch");
        return FLASH_ID_MISMATCH;
    }

    return FLASH_OK;
}

/* =====================================================
 * Page → Cache
 * ===================================================== */
static FlashStatus FlashPageToCache(uint32_t page)
{
    uint8_t cmd[4] = {
        CMD_PAGE_READ,
        (page >> 16) & 0xFF,
        (page >> 8) & 0xFF,
        page & 0xFF
    };

    cs_low();
    spi_device_transmit(flash_spi_handle,
        &(spi_transaction_t){ .length = 32, .tx_buffer = cmd });
    cs_high();

    return FlashWaitReady();
}

/* =====================================================
 * Cache → RAM
 * ===================================================== */
FlashStatus FlashRead(uint32_t page, uint8_t *buf, uint32_t len)
{
    if (!buf || len > W25N_PAGE_SIZE_MAIN) {
        return FLASH_INVALID_PARAM;
    }

    FlashPageToCache(page);

    uint8_t cmd[4] = { CMD_READ_CACHE, 0x00, 0x00, 0x00 };

    cs_low();
    spi_device_transmit(flash_spi_handle,
        &(spi_transaction_t){ .length = 32, .tx_buffer = cmd });

    spi_device_transmit(flash_spi_handle,
        &(spi_transaction_t){ .length = len * 8, .rx_buffer = buf });
    cs_high();

    return FLASH_OK;
}

/* =====================================================
 * 写页
 * ===================================================== */
FlashStatus FlashWrite(uint32_t page, const uint8_t *buf, uint32_t len)
{
    if (!buf || len > W25N_PAGE_SIZE_MAIN) {
        return FLASH_INVALID_PARAM;
    }

    FlashWriteEnable();

    uint8_t cmd[3] = { CMD_PROG_LOAD, 0x00, 0x00 };

    cs_low();
    spi_device_transmit(flash_spi_handle,
        &(spi_transaction_t){ .length = 24, .tx_buffer = cmd });

    spi_device_transmit(flash_spi_handle,
        &(spi_transaction_t){ .length = len * 8, .tx_buffer = buf });
    cs_high();

    FlashWriteEnable();

    uint8_t exe[4] = {
        CMD_PROG_EXECUTE,
        (page >> 16) & 0xFF,
        (page >> 8) & 0xFF,
        page & 0xFF
    };

    cs_low();
    spi_device_transmit(flash_spi_handle,
        &(spi_transaction_t){ .length = 32, .tx_buffer = exe });
    cs_high();

    return FlashWaitReady();
}

/* =====================================================
 * Flash 初始化
 * ===================================================== */
FlashStatus FlashStorageInit(void)
{
    ESP_ERROR_CHECK(SpiInit());
    FlashReset();
    FlashVerifyId();

    if (FlashQueue == NULL) {
        FlashQueue = xQueueCreate(10, sizeof(TxPkg_t));
    }

    ESP_LOGI(TAG, "Flash init OK");
    return FLASH_OK;
}

/* =====================================================
 * 数据存储任务（符号必须存在）
 * ===================================================== */
void AppDataStoreTask(void *arg)
{
    FlashStorageInit();

    TxPkg_t pkg;
    static uint8_t page_buf[W25N_PAGE_SIZE_MAIN];
    static size_t offset = 0;
    static uint32_t page = 0;

    while (1) {
        if (xQueueReceive(FlashQueue, &pkg, portMAX_DELAY)) {
            memcpy(page_buf + offset, pkg.data, pkg.length);
            offset += pkg.length;

            if (offset >= W25N_PAGE_SIZE_MAIN) {
                FlashWrite(page++, page_buf, offset);
                offset = 0;
            }
        }
    }
}
