#ifndef GPS_INTERRUPT_H
#define GPS_INTERRUPT_H

#include <stdint.h>

/**
 * @brief 配置GPS UART中断
 */
void gps_uart_intr_setup(void);
/**
 * @brief 禁用GPS UART中断
 */
void gps_uart_intr_disable(void);
/**
 * @brief 配置GPS PPS中断
 */
void gps_pps_intr_setup(void);
/**
 * @brief 禁用GPS PPS中断
 */
void gps_pps_intr_disable(void);
/**
 * @brief 配置GPS所有相关中断
 */
void gps_all_intr_setup(void);
/**
 * @brief 创建GPS轮询任务
 */
void gps_poll_task_create(void);
/**
 * @brief 清空GPS接收缓冲区
 */
void gps_clear_buffer(void);
/**
 * @brief 轮询处理GPS UART数据
 */
void gps_uart_poll(void);
/**
 * @brief 获取PPS统计信息
 * @param count PPS触发次数输出
 * @param last_time_us 最近一次PPS时间戳（微秒）输出
 */
void gps_get_pps_stats(uint32_t *count, uint64_t *last_time_us);

#endif
