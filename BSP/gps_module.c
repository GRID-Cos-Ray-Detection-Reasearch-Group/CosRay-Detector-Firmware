#include "gps_module.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "string.h"

static const char *TAG = "GPS_MODULE";

void gps_uart_intr_setup(void);

// GPS 模块全局变量
gps_ring_buffer_t gps_ring_buffer = {0};
gps_hex_data_t current_gps_hex_data = {0};
SemaphoreHandle_t gps_hex_data_mutex = NULL;

// 环形缓冲区内存
static uint8_t gps_ring_buffer_mem[GPS_RING_BUFFER_SIZE] = {0};
static portMUX_TYPE gps_ring_buffer_spinlock = portMUX_INITIALIZER_UNLOCKED;

// UART 端口句柄
static uart_port_t gps_uart_port = GPS_UART_PORT;

/* ================= 环形缓冲区 ================= */

void gps_ring_buffer_init(gps_ring_buffer_t *rb, size_t size) {
	if (rb == NULL)
		return;

	// size 参数保留供将来扩展；当前使用编译期常量 GPS_RING_BUFFER_SIZE
	(void)size;
	rb->buffer = gps_ring_buffer_mem;
	rb->size = GPS_RING_BUFFER_SIZE;
	rb->head = 0;
	rb->tail = 0;
	rb->mutex = xSemaphoreCreateMutex();
	if (rb->mutex == NULL)
		ESP_LOGE(TAG, "Failed to create ring buffer mutex");
}

void gps_ring_buffer_reset(gps_ring_buffer_t *rb) {
	if (rb == NULL)
		return;

	if (xSemaphoreTake(rb->mutex, portMAX_DELAY) == pdTRUE) {
		rb->head = 0;
		rb->tail = 0;
		memset(rb->buffer, 0, rb->size);
		xSemaphoreGive(rb->mutex);
	}
}

size_t gps_ring_buffer_write(gps_ring_buffer_t *rb, const uint8_t *data,
							 size_t len) {
	if (rb == NULL || data == NULL || len == 0)
		return 0;

	size_t written = 0;
	if (xSemaphoreTake(rb->mutex, portMAX_DELAY) == pdTRUE) {
		for (size_t i = 0; i < len; i++) {
			size_t next_head = (rb->head + 1) % rb->size;
			if (next_head == rb->tail) {
				ESP_LOGW(TAG, "Ring buffer full, dropping data");
				break;
			}
			rb->buffer[rb->head] = data[i];
			rb->head = next_head;
			written++;
		}
		xSemaphoreGive(rb->mutex);
	}
	return written;
}

size_t gps_ring_buffer_write_isr(gps_ring_buffer_t *rb, const uint8_t *data,
								 size_t len) {
	if (rb == NULL || data == NULL || len == 0)
		return 0;

	size_t written = 0;
	portENTER_CRITICAL_ISR(&gps_ring_buffer_spinlock);
	for (size_t i = 0; i < len; i++) {
		size_t next_head = (rb->head + 1) % rb->size;
		if (next_head == rb->tail)
			break;
		rb->buffer[rb->head] = data[i];
		rb->head = next_head;
		written++;
	}
	portEXIT_CRITICAL_ISR(&gps_ring_buffer_spinlock);
	return written;
}

static size_t gps_ring_buffer_available(const gps_ring_buffer_t *rb) {
	if (rb == NULL)
		return 0;

	size_t available = 0;
	if (xSemaphoreTake(rb->mutex, portMAX_DELAY) == pdTRUE) {
		if (rb->head >= rb->tail)
			available = rb->head - rb->tail;
		else
			available = rb->size - rb->tail + rb->head;
		xSemaphoreGive(rb->mutex);
	}
	return available;
}

/* ================= 引脚和 UART 初始化 ================= */

void gps_pin_init(void) {
	ESP_LOGI(TAG, "Initializing GPS pins");

	uart_config_t uart_config = {
		.baud_rate = GPS_UART_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};

	ESP_ERROR_CHECK(uart_param_config(gps_uart_port, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(gps_uart_port, GPS_TX_PIN, GPS_RX_PIN,
								 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	ESP_ERROR_CHECK(uart_driver_install(gps_uart_port, 2 * GPS_MAX_PACKET_SIZE,
										2 * GPS_MAX_PACKET_SIZE, 0, NULL, 0));

	// 配置 PPS 引脚（仅 GPIO 配置，ISR 由 main.c 安装）
	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_POSEDGE,
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = (1ULL << GPS_PPS_PIN),
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE,
	};
	gpio_config(&io_conf);

	ESP_LOGI(TAG, "GPS pins initialized successfully");
}

void gps_uart_init(void) {
	gps_pin_init();
	gps_uart_intr_setup();
}

void gps_module_init(void) {
	ESP_LOGI(TAG, "Initializing GPS module");

	gps_ring_buffer_init(&gps_ring_buffer, GPS_RING_BUFFER_SIZE);

	gps_hex_data_mutex = xSemaphoreCreateMutex();
	if (gps_hex_data_mutex == NULL)
		ESP_LOGE(TAG, "Failed to create GPS hex data mutex");

	memset(&current_gps_hex_data, 0, sizeof(gps_hex_data_t));

	gps_uart_init();

	ESP_LOGI(TAG, "GPS module initialized successfully");
}

/* ================= UBX 数据包解析 ================= */

bool find_ubx_packet(const gps_ring_buffer_t *rb, ubx_packet_info_t *packet) {
	if (rb == NULL || rb->buffer == NULL || packet == NULL)
		return false;

	bool found = false;
	size_t search_pos = rb->tail;

	while (search_pos != rb->head) {
		uint8_t byte1 = rb->buffer[search_pos];
		search_pos = (search_pos + 1) % rb->size;

		if (byte1 == UBX_SYNC_CHAR_1 && search_pos != rb->head) {
			size_t packet_start = (search_pos + rb->size - 1) % rb->size;
			uint8_t byte2 = rb->buffer[search_pos];

			if (byte2 == UBX_SYNC_CHAR_2) {
				size_t pos_after_sync = (search_pos + 1) % rb->size;
				size_t pos_class = pos_after_sync;
				size_t pos_id = (pos_after_sync + 1) % rb->size;

				if (pos_id == rb->head)
					break;

				uint8_t class_id = rb->buffer[pos_class];
				uint8_t msg_id = rb->buffer[pos_id];

				size_t pos_len1 = (pos_id + 1) % rb->size;
				size_t pos_len2 = (pos_len1 + 1) % rb->size;

				if (pos_len2 == rb->head)
					break;

				uint16_t payload_len = (uint16_t)rb->buffer[pos_len1] |
									   ((uint16_t)rb->buffer[pos_len2] << 8);

				// 2(sync) + 1(class) + 1(id) + 2(len) + payload + 2(ck)
				size_t total_packet_size = 8 + payload_len;
				size_t available_data = gps_ring_buffer_available(rb);

				if (available_data == 0)
					break;

				if (total_packet_size <= available_data) {
					packet->class_id = class_id;
					packet->msg_id = msg_id;
					packet->length = payload_len;
					packet->data_offset = (pos_len2 + 1) % rb->size;
					packet->packet_start = packet_start;
					packet->total_length = total_packet_size;

					size_t ck_a_pos =
						(packet->data_offset + payload_len) % rb->size;
					size_t ck_b_pos = (ck_a_pos + 1) % rb->size;
					packet->ck_a = rb->buffer[ck_a_pos];
					packet->ck_b = rb->buffer[ck_b_pos];

					found = true;
					break;
				}
			}
		}
	}

	return found;
}

static bool gps_extract_nav_pvt_hex(const uint8_t *data, size_t len,
									gps_hex_data_t *out) {
	if (data == NULL || out == NULL || len < 40)
		return false;

	// UBX NAV-PVT 偏移（从负载起始）
	out->year[0] = data[4];
	out->year[1] = data[5];
	out->month = data[6];
	out->day = data[7];
	out->hour = data[8];
	out->minute = data[9];
	out->second = data[10];

	out->longitude[0] = data[24];
	out->longitude[1] = data[25];
	out->longitude[2] = data[26];
	out->longitude[3] = data[27];

	out->latitude[0] = data[28];
	out->latitude[1] = data[29];
	out->latitude[2] = data[30];
	out->latitude[3] = data[31];

	out->altitude[0] = data[36];
	out->altitude[1] = data[37];
	out->altitude[2] = data[38];
	out->altitude[3] = data[39];

	return true;
}

static void gps_ring_buffer_set_tail(gps_ring_buffer_t *rb, size_t new_tail) {
	if (rb == NULL)
		return;

	if (xSemaphoreTake(rb->mutex, portMAX_DELAY) == pdTRUE) {
		rb->tail = new_tail;
		xSemaphoreGive(rb->mutex);
	}
}

/* ================= GPS 解析任务 ================= */

void gps_parser_task(void *pvParameters) {
	ESP_LOGI(TAG, "GPS parser task started");

	uint8_t local_buffer[GPS_MAX_PACKET_SIZE] = {0};
	ubx_packet_info_t current_packet = {0};
	gps_hex_data_t parsed_hex = {0};

	while (true) {
		size_t available = gps_ring_buffer_available(&gps_ring_buffer);
		if (available == 0) {
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		if (find_ubx_packet(&gps_ring_buffer, &current_packet)) {
			ESP_LOGD(TAG,
					 "Found UBX packet: class=0x%02X, id=0x%02X, len=%d",
					 current_packet.class_id, current_packet.msg_id,
					 current_packet.length);

			if (current_packet.length < sizeof(local_buffer)) {
				if (current_packet.class_id == UBX_CLASS_NAV &&
					current_packet.msg_id == UBX_NAV_PVT) {
					size_t bytes_copied = 0;
					size_t offset = current_packet.data_offset;

					while (bytes_copied < current_packet.length) {
						local_buffer[bytes_copied] = gps_ring_buffer_mem[offset];
						bytes_copied++;
						offset = (offset + 1) % GPS_RING_BUFFER_SIZE;
					}

					if (gps_extract_nav_pvt_hex(local_buffer,
												current_packet.length,
												&parsed_hex)) {
						if (xSemaphoreTake(gps_hex_data_mutex,
										   pdMS_TO_TICKS(100)) == pdTRUE) {
							current_gps_hex_data = parsed_hex;
							xSemaphoreGive(gps_hex_data_mutex);
						}
						ESP_LOGD(TAG,
								 "GPS: %02X/%02X/%02X %02X:%02X:%02X",
								 parsed_hex.year[0], parsed_hex.month,
								 parsed_hex.day, parsed_hex.hour,
								 parsed_hex.minute, parsed_hex.second);
					}
				}
			}

			gps_ring_buffer_set_tail(
				&gps_ring_buffer,
				(current_packet.packet_start + current_packet.total_length) %
					GPS_RING_BUFFER_SIZE);
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void gps_parser_task_create(void) {
	BaseType_t ret =
		xTaskCreate(gps_parser_task, "GPS_Parser", GPS_TASK_STACK_SIZE, NULL,
					GPS_PARSER_TASK_PRIORITY, NULL);

	if (ret != pdPASS)
		ESP_LOGE(TAG, "Failed to create GPS parser task");
	else
		ESP_LOGI(TAG, "GPS parser task created successfully");
}

void gps_start(void) {
	gps_module_init();
	gps_parser_task_create();
}
