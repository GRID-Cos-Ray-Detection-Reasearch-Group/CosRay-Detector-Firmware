#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// GPS 引脚配置宏定义
#define GPS_UART_PORT           UART_NUM_1
#define GPS_TX_PIN 18
#define GPS_RX_PIN              17     // GPS模块RX引脚
#define GPS_PPS_PIN 11
#define GPS_UART_BAUD_RATE      115200
#define GPS_TASK_PRIORITY       5       // GPS任务优先级
#define GPS_TASK_STACK_SIZE     4096    // GPS任务堆栈大小
#define GPS_RING_BUFFER_SIZE    8192    // GPS环形缓冲区大小
#define GPS_MAX_PACKET_SIZE     1024    // GPS最大数据包大小
#define GPS_PARSER_TASK_PRIORITY 4      // GPS解析任务优先级

// UBX协议相关定义
#define UBX_SYNC_CHAR_1         0xB5    // UBX协议同步字符1
#define UBX_SYNC_CHAR_2         0x62    // UBX协议同步字符2
#define UBX_CLASS_NAV           0x01    // NAV类消息

// UBX导航消息ID
#define UBX_NAV_PVT             0x07    // 位置速度时间

// 环形缓冲区结构体
typedef struct {
    uint8_t *buffer;                    // 缓冲区指针
    volatile size_t head;               // 写指针
    volatile size_t tail;               // 读指针
    size_t size;                        // 缓冲区大小
    SemaphoreHandle_t mutex;           // 互斥锁
} gps_ring_buffer_t;

typedef struct {
    uint8_t year[2];
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t latitude[4];
    uint8_t longitude[4];
    uint8_t altitude[4];
} gps_hex_data_t;

// UBX数据包信息结构体
typedef struct {
    uint8_t class_id;                  // 消息类
    uint8_t msg_id;                    // 消息ID
    uint16_t length;                   // 数据长度
    size_t data_offset;                // 数据在缓冲区中的偏移
    size_t packet_start;
    size_t total_length;
    uint8_t ck_a;                      // 校验和A
    uint8_t ck_b;                      // 校验和B
} ubx_packet_info_t;



/**
 * @brief 一键启动GPS模块（初始化 + 创建所有任务）
 */
void gps_start(void);

/**
 * @brief 初始化GPS模块，包括环形缓冲区、互斥锁和UART
 */
void gps_module_init(void);

/**
 * @brief 创建GPS解析任务，从环形缓冲区解析UBX数据包
 */
void gps_parser_task_create(void);


/**
 * @brief 初始化GPS UART接口和中断
 */
void gps_uart_init(void);

/**
 * @brief 初始化GPS相关引脚（UART和PPS）
 */
void gps_pin_init(void);

/**
 * @brief 初始化环形缓冲区
 * @param rb 环形缓冲区对象
 * @param size 缓冲区大小
 */
void gps_ring_buffer_init(gps_ring_buffer_t *rb, size_t size);

/**
 * @brief 复位环形缓冲区（清空数据）
 * @param rb 环形缓冲区对象
 */
void gps_ring_buffer_reset(gps_ring_buffer_t *rb);

/**
 * @brief 向环形缓冲区写入数据（线程安全，使用互斥锁）
 * @param rb 环形缓冲区对象
 * @param data 写入数据指针
 * @param len 写入数据长度
 * @return 实际写入字节数
 */
size_t gps_ring_buffer_write(gps_ring_buffer_t *rb, const uint8_t *data, size_t len);

/**
 * @brief 在中断服务程序中向环形缓冲区写入数据（使用自旋锁）
 * @param rb 环形缓冲区对象
 * @param data 写入数据指针
 * @param len 写入数据长度
 * @return 实际写入字节数
 */
size_t gps_ring_buffer_write_isr(gps_ring_buffer_t *rb, const uint8_t *data, size_t len);

/**
 * @brief GPS解析任务函数实体
 * @param pvParameters 任务参数
 */
void gps_parser_task(void *pvParameters);

/**
 * @brief 在环形缓冲区中查找并验证UBX数据包
 * @param rb 环形缓冲区对象
 * @param packet 数据包信息输出
 * @return 是否找到并通过校验
 */
bool find_ubx_packet(const gps_ring_buffer_t *rb, ubx_packet_info_t *packet);

// 全局变量
extern gps_ring_buffer_t gps_ring_buffer;
extern gps_hex_data_t current_gps_hex_data;
extern SemaphoreHandle_t gps_hex_data_mutex;

#endif // GPS_MODULE_H
