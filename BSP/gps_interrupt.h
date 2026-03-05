#ifndef GPS_INTERRUPT_H
#define GPS_INTERRUPT_H

#include <stdint.h>

/**
 * @brief 配置 GPS UART 接收（创建轮询任务）
 */
void gps_uart_intr_setup(void);

/**
 * @brief 禁用 GPS UART 接收
 */
void gps_uart_intr_disable(void);

/**
 * @brief 配置 GPS PPS 中断（安装 ISR 处理函数）
 */
void gps_pps_intr_setup(void);

/**
 * @brief 禁用 GPS PPS 中断
 */
void gps_pps_intr_disable(void);

/**
 * @brief 配置 GPS 所有相关中断（UART + PPS）
 */
void gps_all_intr_setup(void);

/**
 * @brief 创建 GPS UART 轮询任务
 */
void gps_poll_task_create(void);

/**
 * @brief 清空 GPS 环形缓冲区
 */
void gps_clear_buffer(void);

/**
 * @brief 轮询读取 GPS UART 数据到环形缓冲区
 */
void gps_uart_poll(void);

/**
 * @brief 获取 PPS 统计信息
 * @param count        输出：PPS 触发次数
 * @param last_time_us 输出：最近一次 PPS 时间戳（微秒）
 */
void gps_get_pps_stats(uint32_t *count, uint64_t *last_time_us);

#endif // GPS_INTERRUPT_H
