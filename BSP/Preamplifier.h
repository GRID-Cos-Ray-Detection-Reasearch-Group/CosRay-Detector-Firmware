#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        TickType_t tick_count;
        uint32_t sequence;
    } preamplifier_trigger_context_t;

    typedef struct
    {
        preamplifier_trigger_context_t context;
        int signal_voltage_mv;
        int signal_raw;
    } preamplifier_trigger_sample_t;

    typedef void (*preamplifier_capture_context_cb_t)(const preamplifier_trigger_context_t *context, void *user_ctx);
    typedef void (*preamplifier_trigger_sample_cb_t)(const preamplifier_trigger_sample_t *sample, void *user_ctx);

    typedef struct
    {
        int cathode_voltage_mv;
        int vref_voltage_mv;
        int cathode_monitor_scale;
        bool enable_chargein;
        gpio_num_t trigger_gpio;
        gpio_int_type_t trigger_intr_type;
        uint32_t signal_sample_delay_ms;
        gpio_num_t chargein_gpio;
        uint32_t chargein_period_ms;
        uint32_t chargein_pulse_width_ms;
        gpio_num_t restart_gpio;
        uint32_t restart_pulse_width_ms;
        uint32_t cathode_log_period_ms;
        uint32_t task_interval_ms;
        preamplifier_capture_context_cb_t capture_trigger_context;
        preamplifier_trigger_sample_cb_t on_trigger_sample;
        void *user_ctx;
    } preamplifier_config_t;

    void preamplifier_get_default_config(preamplifier_config_t *config);

    esp_err_t preamplifier_init(const preamplifier_config_t *config);

    esp_err_t preamplifier_read_cathode_monitor_mv(int *monitor_mv, int *raw, int *sipm_input_mv);

    esp_err_t preamplifier_trigger_chargein_pulse(void);

#ifdef __cplusplus
}
#endif