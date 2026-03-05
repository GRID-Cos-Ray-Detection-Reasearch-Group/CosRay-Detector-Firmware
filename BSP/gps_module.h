#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

/* ================= 引脚配置（与主模块 main.c 一致） ================= */
#define GPS_UART_PORT UART_NUM_1
#define GPS_TX_PIN 18			   // ESP32 TX → GPS 模块 RX
#define GPS_RX_PIN 17			   // GPS 模块 TX → ESP32 RX
#define GPS_PPS_PIN 11			   // GPS PPS 脉冲输入（与 PIN_PPS 一致）
#define GPS_UART_BAUD_RATE 9600	   // NMEA/UBX 默认波特率
#define GPS_TASK_PRIORITY 5		   // GPS 任务优先级
#define GPS_TASK_STACK_SIZE 4096   // GPS 任务堆栈大小（字节）
#define GPS_RING_BUFFER_SIZE 8192  // GPS 环形缓冲区大小（字节）
#define GPS_MAX_PACKET_SIZE 1024   // GPS 最大数据包大小（字节）
#define GPS_PARSER_TASK_PRIORITY 4 // GPS 解析任务优先级

/* ================= UBX 协议定义 ================= */
#define UBX_SYNC_CHAR_1 0xB5 // UBX 协议同步字符1
#define UBX_SYNC_CHAR_2 0x62 // UBX 协议同步字符2
#define UBX_CLASS_NAV 0x01	 // NAV 类消息

// UBX 导航消息 ID
#define UBX_NAV_PVT 0x07 // 位置速度时间

/* ================= 数据结构 ================= */

// 环形缓冲区
typedef struct {
	uint8_t *buffer;		 // 缓冲区指针
	volatile size_t head;	 // 写指针
	volatile size_t tail;	 // 读指针
	size_t size;			 // 缓冲区大小
	SemaphoreHandle_t mutex; // 互斥锁
} gps_ring_buffer_t;

// 解析后的 GPS 位置/时间数据（原始字节格式）
typedef struct {
	uint8_t year[2];	  // 年（小端 uint16_t）
	uint8_t month;		  // 月
	uint8_t day;		  // 日
	uint8_t hour;		  // 时
	uint8_t minute;		  // 分
	uint8_t second;		  // 秒
	uint8_t latitude[4];  // 纬度（小端 int32_t，单位 1e-7 度）
	uint8_t longitude[4]; // 经度（小端 int32_t，单位 1e-7 度）
	uint8_t altitude[4];  // 海拔（小端 int32_t，单位 mm）
} gps_hex_data_t;

// UBX 数据包信息
typedef struct {
	uint8_t class_id;	 // 消息类
	uint8_t msg_id;		 // 消息 ID
	uint16_t length;	 // 负载长度
	size_t data_offset;	 // 负载在缓冲区中的起始偏移
	size_t packet_start; // 完整包的起始偏移
	size_t total_length; // 包总字节数（含头尾）
	uint8_t ck_a;		 // 校验和 A
	uint8_t ck_b;		 // 校验和 B
} ubx_packet_info_t;

/* ================= 全局变量 ================= */
extern gps_ring_buffer_t gps_ring_buffer;
extern gps_hex_data_t current_gps_hex_data;
extern SemaphoreHandle_t gps_hex_data_mutex;

/* ================= API ================= */

/**
 * @brief 一键启动 GPS 模块（初始化 + 创建解析任务）
 */
void gps_start(void);

/**
 * @brief 初始化 GPS 模块：环形缓冲区、互斥锁、UART
 */
void gps_module_init(void);

/**
 * @brief 创建 GPS UBX 解析任务
 */
void gps_parser_task_create(void);

/**
 * @brief 初始化 GPS UART 及 PPS 引脚
 */
void gps_uart_init(void);

/**
 * @brief 初始化 GPS 相关 GPIO/UART 引脚
 */
void gps_pin_init(void);

/**
 * @brief 初始化环形缓冲区
 */
void gps_ring_buffer_init(gps_ring_buffer_t *rb, size_t size);

/**
 * @brief 清空环形缓冲区
 */
void gps_ring_buffer_reset(gps_ring_buffer_t *rb);

/**
 * @brief 向环形缓冲区写入数据（线程安全）
 */
size_t gps_ring_buffer_write(gps_ring_buffer_t *rb, const uint8_t *data,
							 size_t len);

/**
 * @brief 在 ISR 中向环形缓冲区写入数据（自旋锁保护）
 */
size_t gps_ring_buffer_write_isr(gps_ring_buffer_t *rb, const uint8_t *data,
								 size_t len);

/**
 * @brief GPS UBX 解析任务函数
 */
void gps_parser_task(void *pvParameters);

/**
 * @brief 在环形缓冲区中查找并解析 UBX 数据包
 */
bool find_ubx_packet(const gps_ring_buffer_t *rb, ubx_packet_info_t *packet);

#endif // GPS_MODULE_H
