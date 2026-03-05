#include "flashstorage.h"
#include "typedefs.h"
#include "esp_log.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include <string.h>

static const char *TAG = "FlashStorage";
static FlashStatus FlashPageToCache(uint32_t page);

spi_device_handle_t flash_spi_handle;
extern QueueHandle_t FlashQueue;

/* ================= SPI NAND 指令 ================= */
#define CMD_RESET_ENABLE  0x66
#define CMD_RESET_EXECUTE 0x99
#define CMD_READ_ID       0x9F
#define CMD_WRITE_ENABLE  0x06
#define CMD_PAGE_READ     0x13
#define CMD_READ_CACHE    0x03
#define CMD_PROG_LOAD     0x02
#define CMD_PROG_EXECUTE  0x10
#define CMD_BLOCK_ERASE   0xD8

#define CMD_GET_FEATURE 0x0F
#define CMD_SET_FEATURE 0x1F

/* ================= Feature Register 地址 ================= */
#define REG_STATUS  0xC0
#define REG_PROTECT 0xA0

/* ================= Status Register 位 ================= */
#define STATUS_OIP    0x01
#define STATUS_WEL    0x02
#define STATUS_E_FAIL 0x04
#define STATUS_P_FAIL 0x08
#define STATUS_ECC_MASK 0x30

/* ================= Flash 参数 ================= */
#define W25N_PAGE_SIZE_MAIN_LOCAL  2048
#define W25N_PAGE_SIZE_OOB_LOCAL   64
#define W25N_BLOCK_SIZE_PAGE_LOCAL 128

#define JEDEC_MFG_ID   0xEF
#define JEDEC_DEV_MSB  0xAE
#define JEDEC_DEV_LSB  0x21

#define W25N_BAD_BLOCK_MARK 0x00

/* ========================================================= */

static inline void cs_low(void) {
	gpio_set_level(PIN_CS, 0);
	esp_rom_delay_us(1);
}

static inline void cs_high(void) {
	esp_rom_delay_us(1);
	gpio_set_level(PIN_CS, 1);
	esp_rom_delay_us(1);
}

/* ================= Feature Register 读写 ================= */

static uint8_t FlashGetFeature(uint8_t reg) {
	uint8_t tx[3] = {CMD_GET_FEATURE, reg, 0};
	uint8_t rx[3] = {0};
	spi_transaction_t t = {.length = 24, .tx_buffer = tx, .rx_buffer = rx};
	cs_low();
	spi_device_transmit(flash_spi_handle, &t);
	cs_high();
	return rx[2];
}

static FlashStatus FlashSetFeature(uint8_t reg, uint8_t val) {
	uint8_t tx[3] = {CMD_SET_FEATURE, reg, val};
	spi_transaction_t t = {.length = 24, .tx_buffer = tx};
	cs_low();
	esp_err_t r = spi_device_transmit(flash_spi_handle, &t);
	cs_high();
	return (r == ESP_OK) ? FLASH_OK : FLASH_SPI_ERROR;
}

/* ================= 等待操作完成 ================= */

static FlashStatus FlashWaitReady(void) {
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

static FlashStatus FlashWriteEnable(void) {
	uint8_t cmd = CMD_WRITE_ENABLE;
	spi_transaction_t t = {.length = 8, .tx_buffer = &cmd};
	cs_low();
	spi_device_transmit(flash_spi_handle, &t);
	cs_high();
	if (!(FlashGetFeature(REG_STATUS) & STATUS_WEL))
		return FLASH_WRITE_ENABLE_ERR;
	return FLASH_OK;
}

/* ================= SPI 总线初始化 ================= */

static esp_err_t SpiInit(void) {
	static bool initialized = false;
	if (initialized)
		return ESP_OK;
	initialized = true;

	gpio_config_t io_cfg = {
		.pin_bit_mask =
			(1ULL << PIN_CS) | (1ULL << PIN_WP) | (1ULL << PIN_HOLD),
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
	ESP_ERROR_CHECK(
		spi_bus_add_device(SPI2_HOST, &dev_cfg, &flash_spi_handle));

	return ESP_OK;
}

/* ================= 复位 ================= */

static FlashStatus FlashReset(void) {
	uint8_t cmd;
	spi_transaction_t t = {.length = 8};

	cmd = CMD_RESET_ENABLE;
	t.tx_buffer = &cmd;
	cs_low();
	spi_device_transmit(flash_spi_handle, &t);
	cs_high();

	cmd = CMD_RESET_EXECUTE;
	t.tx_buffer = &cmd;
	cs_low();
	spi_device_transmit(flash_spi_handle, &t);
	cs_high();

	vTaskDelay(pdMS_TO_TICKS(2));
	return FlashWaitReady();
}

/* ================= JEDEC ID 校验 ================= */

static FlashStatus FlashVerifyId(void) {
	uint8_t tx[5] = {CMD_READ_ID, 0, 0, 0, 0};
	uint8_t rx[5] = {0};

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

	if (mfg != JEDEC_MFG_ID || dev_msb != JEDEC_DEV_MSB ||
		dev_lsb != JEDEC_DEV_LSB) {
		ESP_LOGE(TAG, "JEDEC ID mismatch (expected: %02X %02X %02X)",
				 JEDEC_MFG_ID, JEDEC_DEV_MSB, JEDEC_DEV_LSB);
		return FLASH_ID_MISMATCH;
	}
	return FLASH_OK;
}

/* ================= 关闭写保护 ================= */

static FlashStatus FlashDisableWriteProtect(void) {
	FlashStatus r = FlashWriteEnable();
	if (r != FLASH_OK)
		return r;
	r = FlashSetFeature(REG_PROTECT, 0x00);
	if (r != FLASH_OK)
		return r;
	return FlashWaitReady();
}

/* ================= 将页读入缓存 ================= */

static FlashStatus FlashPageToCache(uint32_t page) {
	uint8_t cmd[4] = {CMD_PAGE_READ, (page >> 16) & 0xFF, (page >> 8) & 0xFF,
					  page & 0xFF};
	spi_transaction_t t = {.length = 32, .tx_buffer = cmd};
	cs_low();
	spi_device_transmit(flash_spi_handle, &t);
	cs_high();
	return FlashWaitReady();
}

/* ================= 读取页主区 ================= */

FlashStatus FlashRead(uint32_t page, uint8_t *buf, uint32_t len) {
	if (!buf || len == 0 || len > W25N_PAGE_SIZE_MAIN_LOCAL)
		return FLASH_INVALID_PARAM;

	FlashStatus ret = FlashPageToCache(page);
	if (ret != FLASH_OK)
		return ret;

	uint8_t cmd[4] = {CMD_READ_CACHE, 0x00, 0x00, 0x00};

	cs_low();
	spi_transaction_t t_cmd = {.length = 32, .tx_buffer = cmd};
	spi_device_transmit(flash_spi_handle, &t_cmd);
	spi_transaction_t t_data = {.length = len * 8, .rx_buffer = buf};
	spi_device_transmit(flash_spi_handle, &t_data);
	cs_high();

	uint8_t s = FlashGetFeature(REG_STATUS);
	if ((s & STATUS_ECC_MASK) == 0x20)
		ESP_LOGW(TAG, "ECC corrected (page=%" PRIu32 ")", page);
	else if ((s & STATUS_ECC_MASK) == 0x30) {
		ESP_LOGE(TAG, "ECC failure (page=%" PRIu32 ")", page);
		return FLASH_CRC_ERROR;
	}
	return FLASH_OK;
}

/* ================= 读取页 OOB 备用区 ================= */

FlashStatus FlashReadOOB(uint32_t page, uint8_t *oob_buf, uint32_t len) {
	if (!oob_buf || len == 0 || len > W25N_PAGE_SIZE_OOB_LOCAL)
		return FLASH_INVALID_PARAM;

	FlashStatus ret = FlashPageToCache(page);
	if (ret != FLASH_OK)
		return ret;

	uint16_t column = W25N_PAGE_SIZE_MAIN_LOCAL;
	uint8_t cmd[4] = {CMD_READ_CACHE, (column >> 8) & 0xFF, column & 0xFF,
					  0x00};

	cs_low();
	spi_transaction_t t_cmd = {.length = 32, .tx_buffer = cmd};
	spi_device_transmit(flash_spi_handle, &t_cmd);
	spi_transaction_t t_data = {.length = len * 8, .rx_buffer = oob_buf};
	spi_device_transmit(flash_spi_handle, &t_data);
	cs_high();

	return FLASH_OK;
}

/* ================= 坏块检测 ================= */

static FlashStatus FlashCheckBadBlock(uint32_t page) {
	uint32_t block = page / W25N_BLOCK_SIZE_PAGE_LOCAL;
	uint32_t page0 = block * W25N_BLOCK_SIZE_PAGE_LOCAL;
	uint32_t page1 = page0 + 1;
	uint8_t oob[W25N_PAGE_SIZE_OOB_LOCAL];

	if (FlashReadOOB(page0, oob, W25N_PAGE_SIZE_OOB_LOCAL) != FLASH_OK)
		return FLASH_OP_ERROR;
	if (oob[0] != 0xFF)
		return FLASH_BAD_BLOCK;

	if (FlashReadOOB(page1, oob, W25N_PAGE_SIZE_OOB_LOCAL) != FLASH_OK)
		return FLASH_OP_ERROR;
	if (oob[0] != 0xFF)
		return FLASH_BAD_BLOCK;

	return FLASH_OK;
}

/* ================= 编程加载和执行 ================= */

static FlashStatus FlashProgLoad(const uint8_t *buf, uint32_t len) {
	uint8_t cmd[3] = {CMD_PROG_LOAD, 0x00, 0x00};

	cs_low();
	spi_transaction_t t_cmd = {.length = 24, .tx_buffer = cmd};
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

static FlashStatus FlashProgExecute(uint32_t page) {
	uint8_t exe[4] = {CMD_PROG_EXECUTE, (page >> 16) & 0xFF,
					  (page >> 8) & 0xFF, page & 0xFF};
	spi_transaction_t t = {.length = 32, .tx_buffer = exe};

	cs_low();
	if (spi_device_transmit(flash_spi_handle, &t) != ESP_OK) {
		cs_high();
		return FLASH_SPI_ERROR;
	}
	cs_high();

	FlashStatus ret = FlashWaitReady();
	if (ret != FLASH_OK)
		return ret;

	uint8_t s = FlashGetFeature(REG_STATUS);
	if (s & STATUS_P_FAIL) {
		ESP_LOGE(TAG, "Program failed (page=%" PRIu32 ")", page);
		return FLASH_OP_ERROR;
	}
	return FLASH_OK;
}

/* ================= 块擦除 ================= */

FlashStatus FlashEraseBlock(uint32_t page) {
	if (page % W25N_BLOCK_SIZE_PAGE_LOCAL != 0)
		return FLASH_INVALID_PARAM;

	FlashStatus ret = FlashWriteEnable();
	if (ret != FLASH_OK)
		return ret;

	uint8_t cmd[4] = {CMD_BLOCK_ERASE, (page >> 16) & 0xFF,
					  (page >> 8) & 0xFF, page & 0xFF};
	spi_transaction_t t = {.length = 32, .tx_buffer = cmd};

	cs_low();
	spi_device_transmit(flash_spi_handle, &t);
	cs_high();

	ret = FlashWaitReady();
	if (ret != FLASH_OK)
		return ret;

	uint8_t s = FlashGetFeature(REG_STATUS);
	if (s & STATUS_E_FAIL) {
		ESP_LOGE(TAG, "Erase failed (block page=%" PRIu32 ")", page);
		return FLASH_OP_ERROR;
	}

	ESP_LOGD(TAG, "Block erase OK (%" PRIu32 ")", page);
	return FLASH_OK;
}

/* ================= 写入一页 ================= */

FlashStatus FlashWrite(uint32_t page, const uint8_t *buf, uint32_t len) {
	if (!buf || len == 0 || len > W25N_PAGE_SIZE_MAIN_LOCAL)
		return FLASH_INVALID_PARAM;

	FlashStatus ret = FlashCheckBadBlock(page);
	if (ret != FLASH_OK)
		return ret;

	// 块首页写入前自动擦除
	if (page % W25N_BLOCK_SIZE_PAGE_LOCAL == 0) {
		ret = FlashEraseBlock(page);
		if (ret != FLASH_OK)
			return ret;
	}

	ret = FlashWriteEnable();
	if (ret != FLASH_OK)
		return ret;

	ret = FlashProgLoad(buf, len);
	if (ret != FLASH_OK)
		return ret;

	ret = FlashProgExecute(page);
	if (ret == FLASH_OK)
		ESP_LOGD(TAG, "Page write OK (%" PRIu32 ")", page);

	return ret;
}

/* ================= Flash 存储初始化 ================= */

FlashStatus FlashStorageInit(void) {
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

/* ================= Flash 存储 FreeRTOS 任务 ================= */

void AppDataStoreTask(void *arg) {
	FlashStatus ret = FlashStorageInit();
	if (ret != FLASH_OK) {
		ESP_LOGE(TAG, "FlashStorageInit failed (err=%d), task exit", ret);
		vTaskDelete(NULL);
		return;
	}

	TxPkg_t pkg;
	static uint8_t page_buf[W25N_PAGE_SIZE_MAIN_LOCAL] = {0};
	static size_t offset = 0;
	static uint32_t current_page = 0;

	ESP_LOGI(TAG, "AppDataStoreTask start running");

	while (1) {
		// 等待 FlashQueue 就绪
		while (FlashQueue == NULL) {
			ESP_LOGW(TAG, "Waiting FlashQueue...");
			vTaskDelay(pdMS_TO_TICKS(10));
		}

		if (xQueueReceive(FlashQueue, &pkg, portMAX_DELAY)) {
			if (pkg.length == 0 ||
				pkg.length > (W25N_PAGE_SIZE_MAIN_LOCAL - offset)) {
				ESP_LOGE(TAG,
						 "Invalid pkg data (length=%" PRIu32
						 ", offset=%" PRIu32 ")",
						 (uint32_t)pkg.length, (uint32_t)offset);
				// 重置页缓冲区，防止后续数据错位
				offset = 0;
				memset(page_buf, 0, W25N_PAGE_SIZE_MAIN_LOCAL);
				continue;
			}

			memcpy(page_buf + offset, pkg.data, pkg.length);
			offset += pkg.length;

			ESP_LOGD(TAG,
					 "Received data (len=%" PRIu32 ", offset=%" PRIu32 ")",
					 (uint32_t)pkg.length, (uint32_t)offset);

			if (offset >= W25N_PAGE_SIZE_MAIN_LOCAL) {
				ret = FlashWrite(current_page, page_buf,
								 W25N_PAGE_SIZE_MAIN_LOCAL);

				if (ret == FLASH_BAD_BLOCK) {
					// 跳过坏块
					current_page += W25N_BLOCK_SIZE_PAGE_LOCAL;
					memset(page_buf, 0, W25N_PAGE_SIZE_MAIN_LOCAL);
					offset = 0;
					continue;
				} else if (ret != FLASH_OK) {
					ESP_LOGE(TAG,
							 "FlashWrite failed (page=%" PRIu32
							 ", err=%d)",
							 current_page, ret);
				}

				current_page++;
				memset(page_buf, 0, W25N_PAGE_SIZE_MAIN_LOCAL);
				offset = 0;
			}
		}
	}
}
