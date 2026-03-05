#include "gps_module.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "string.h"

static const char *TAG = "GPS_INTERRUPT";

void gps_poll_task_create(void);

// 中断使能标志
static volatile bool gps_uart_intr_enabled = false;
static volatile bool gps_pps_intr_enabled = false;

// PPS 时间管理（由 main.c 的 PPS ISR 主要负责；此处作辅助统计用）
static uint64_t last_pps_time_us = 0;
static uint32_t pps_count = 0;

// PPS 中断处理函数（备用，与 main.c pps_isr 配合）
static void IRAM_ATTR gps_pps_isr_handler(void *arg) {
	uint64_t pps_time_us = esp_timer_get_time();
	uint32_t pin_level = gpio_get_level(GPS_PPS_PIN);

	if (pin_level == 1) {
		pps_count++;
		last_pps_time_us = pps_time_us;
	}
}

// 设置 UART 接收（创建轮询任务）
void gps_uart_intr_setup(void) {
	ESP_LOGI(TAG, "Setting up GPS UART interrupt");
	gps_poll_task_create();
	gps_uart_intr_enabled = true;
	ESP_LOGI(TAG, "GPS UART interrupt setup completed");
}

void gps_uart_intr_disable(void) {
	if (gps_uart_intr_enabled) {
		gps_uart_intr_enabled = false;
		ESP_LOGI(TAG, "GPS UART interrupt disabled");
	}
}

// 设置 PPS 中断（在 main.c 之外单独使用时调用；主路 PPS ISR 在 main.c 中）
void gps_pps_intr_setup(void) {
	ESP_LOGI(TAG, "Setting up GPS PPS interrupt");
	// gpio_install_isr_service 已在 main.c 中调用，此处忽略重复安装的错误
	esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
	if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
		ESP_LOGE(TAG, "gpio_install_isr_service failed: %d", ret);
	gpio_isr_handler_add(GPS_PPS_PIN, gps_pps_isr_handler, NULL);
	gpio_intr_enable(GPS_PPS_PIN);
	gps_pps_intr_enabled = true;
	ESP_LOGI(TAG, "GPS PPS interrupt setup completed");
}

void gps_pps_intr_disable(void) {
	if (gps_pps_intr_enabled) {
		gpio_isr_handler_remove(GPS_PPS_PIN);
		gpio_intr_disable(GPS_PPS_PIN);
		gps_pps_intr_enabled = false;
		ESP_LOGI(TAG, "GPS PPS interrupt disabled");
	}
}

void gps_clear_buffer(void) {
	ESP_LOGI(TAG, "Clearing GPS ring buffer");
	gps_ring_buffer_reset(&gps_ring_buffer);
}

// 轮询方式读取 UART 数据并写入环形缓冲区
void gps_uart_poll(void) {
	uint8_t data[128];
	int length = 0;

	ESP_ERROR_CHECK(
		uart_get_buffered_data_len(GPS_UART_PORT, (size_t *)&length));

	if (length > 0) {
		if (length > (int)sizeof(data))
			length = (int)sizeof(data);

		length = uart_read_bytes(GPS_UART_PORT, data, length, 0);
		if (length > 0) {
			gps_ring_buffer_write(&gps_ring_buffer, data, length);
			ESP_LOGD(TAG, "POLL: Read %d bytes from UART", length);
		}
	}
}

// GPS UART 轮询任务（每 10 ms 读取一次）
static void gps_uart_poll_task(void *pvParameters) {
	ESP_LOGI(TAG, "GPS UART poll task started");

	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(10);

	while (true) {
		gps_uart_poll();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void gps_all_intr_setup(void) {
	gps_uart_intr_setup();
	gps_pps_intr_setup();
}

void gps_poll_task_create(void) {
	BaseType_t ret = xTaskCreate(gps_uart_poll_task, "GPS_Polling", 2048, NULL,
								 GPS_TASK_PRIORITY - 1, NULL);

	if (ret != pdPASS)
		ESP_LOGE(TAG, "Failed to create GPS poll task");
	else
		ESP_LOGI(TAG, "GPS poll task created successfully");
}

void gps_get_pps_stats(uint32_t *count, uint64_t *last_time_us) {
	if (count != NULL)
		*count = pps_count;
	if (last_time_us != NULL)
		*last_time_us = last_pps_time_us;
}
