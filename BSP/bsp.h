#ifndef BSP_PRESENT
#define BSP_PRESENT

// #include "test.h"
#include "gps_module.h"

// GPS中断处理函数声明
void gps_uart_intr_setup(void);
void gps_uart_intr_disable(void);
void gps_pps_intr_setup(void);
void gps_pps_intr_disable(void);
void gps_all_intr_setup(void);
void gps_poll_task_create(void);
void gps_clear_buffer(void);
void gps_uart_poll(void);
void gps_get_pps_stats(uint32_t *count, uint64_t *last_time_us);

#endif