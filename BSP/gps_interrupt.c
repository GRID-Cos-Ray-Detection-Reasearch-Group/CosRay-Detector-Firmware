#include "gps_module.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "string.h"

static const char *TAG = "GPS_INTERRUPT";

void gps_poll_task_create(void);

// 中断标志
static volatile bool gps_uart_intr_enabled = false;
static volatile bool gps_pps_intr_enabled = false;

// PPS时间管理
static uint64_t last_pps_time_us = 0;
static uint32_t pps_count = 0;

// PPS中断处理函数
static void IRAM_ATTR gps_pps_isr_handler(void *arg) {
    // 获取当前时间戳
    uint64_t pps_time_us = esp_timer_get_time();
    uint32_t pin_level = gpio_get_level(GPS_PPS_PIN);

    // 精确时间测量
    if (pin_level == 1) {
        // PPS上升沿
        pps_count++;

        // 计算PPS周期（应该在1秒左右）
        if (last_pps_time_us > 0) {
            uint64_t pps_period_us = pps_time_us - last_pps_time_us;
            uint32_t pps_jitter_us = pps_period_us > 1000000 ?
                                    (uint32_t)(pps_period_us - 1000000) :
                                    (uint32_t)(1000000 - pps_period_us);

            // 如果抖动小于1ms，认为是有效的PPS信号
            if (pps_jitter_us < 1000) {
                // 可以在这里更新系统时间
                // TODO: 增加时间同步逻辑
            }
        }

        last_pps_time_us = pps_time_us;

        // 通知GPS任务处理PPS事件
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // TODO: 发送PPS事件到队列

        // 如果需要上下文切换
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

// 设置UART接收中断
void gps_uart_intr_setup(void) {
    ESP_LOGI(TAG, "Setting up GPS UART interrupt");

    gps_poll_task_create();

    gps_uart_intr_enabled = true;
    ESP_LOGI(TAG, "GPS UART interrupt setup completed");
}

// 禁用UART接收中断
void gps_uart_intr_disable(void) {
    if (gps_uart_intr_enabled) {
        gps_uart_intr_enabled = false;
        ESP_LOGI(TAG, "GPS UART interrupt disabled");
    }
}

// 设置PPS中断
void gps_pps_intr_setup(void) {
    ESP_LOGI(TAG, "Setting up GPS PPS interrupt");

    // 安装GPIO中断服务
    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);

    // 设置中断处理函数
    gpio_isr_handler_add(GPS_PPS_PIN, gps_pps_isr_handler, NULL);

    // 启用中断
    gpio_intr_enable(GPS_PPS_PIN);

    gps_pps_intr_enabled = true;
    ESP_LOGI(TAG, "GPS PPS interrupt setup completed");
}

// 禁用PPS中断
void gps_pps_intr_disable(void) {
    if (gps_pps_intr_enabled) {
        gpio_isr_handler_remove(GPS_PPS_PIN);
        gpio_intr_disable(GPS_PPS_PIN);
        gps_pps_intr_enabled = false;
        ESP_LOGI(TAG, "GPS PPS interrupt disabled");
    }
}

// 清空环形缓冲区
void gps_clear_buffer(void) {
    ESP_LOGI(TAG, "Clearing GPS ring buffer");
    gps_ring_buffer_reset(&gps_ring_buffer);
}

// 轮询方式读取串口数据（作为中断的备份方案）
void gps_uart_poll(void) {
    uint8_t data[128];
    int length = 0;

    // 从UART读取尽可能多的数据
    ESP_ERROR_CHECK(uart_get_buffered_data_len(GPS_UART_PORT, (size_t *)&length));

    if (length > 0) {
        // 限制一次读取的数据量
        if (length > sizeof(data)) {
            length = sizeof(data);
        }

        // 读取UART数据
        length = uart_read_bytes(GPS_UART_PORT, data, length, 0 / portTICK_PERIOD_MS);

        if (length > 0) {
            // 写入环形缓冲区
            gps_ring_buffer_write(&gps_ring_buffer, data, length);

            ESP_LOGD(TAG, "POLL: Read %d bytes from UART", length);
        }
    }
}

// GPS串口接收任务（使用轮询方式，可以根据需要启用）
void gps_uart_poll_task(void *pvParameters) {
    ESP_LOGI(TAG, "GPS UART poll task started");

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 每10ms轮询一次

    // 初始化上一次唤醒时间
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // 轮询读取串口数据
        gps_uart_poll();

        // 等待下一个周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 获取PPS统计信息
void gps_get_pps_stats(uint32_t *count, uint64_t *last_time_us) {
    if (count != NULL) {
        *count = pps_count;
    }

    if (last_time_us != NULL) {
        *last_time_us = last_pps_time_us;
    }
}

// 初始化所有中断
void gps_all_intr_setup(void) {
    // 设置UART接收中断
    gps_uart_intr_setup();

    // 设置PPS中断
    gps_pps_intr_setup();
}

// 创建GPS轮询任务
void gps_poll_task_create(void) {
    BaseType_t ret = xTaskCreate(
        gps_uart_poll_task,
        "GPS_Polling",
        2048,
        NULL,
        GPS_TASK_PRIORITY - 1,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS poll task");
    } else {
        ESP_LOGI(TAG, "GPS poll task created successfully");
    }
}
