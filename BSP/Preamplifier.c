#include "Preamplifier.h"

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "Dac.h"

static const char *TAG = "Preamplifier";

static const preamplifier_config_t s_default_cfg = {
    .cathode_voltage_mv = 239,
    .vref_voltage_mv = 100,
    .cathode_monitor_scale = 33,
    .enable_chargein = true,
    .trigger_gpio = GPIO_NUM_10,
    .trigger_intr_type = GPIO_INTR_POSEDGE,
    .signal_sample_delay_ms = 1,
    .chargein_gpio = GPIO_NUM_8,
    .chargein_period_ms = 2000,
    .chargein_pulse_width_ms = 50,
    .restart_gpio = GPIO_NUM_9,
    .restart_pulse_width_ms = 50,
    .cathode_log_period_ms = 1000,
    .task_interval_ms = 10,
    .capture_trigger_context = NULL,
    .on_trigger_sample = NULL,
    .user_ctx = NULL,
};

static preamplifier_config_t s_cfg;
static TaskHandle_t s_task_handle;
static volatile uint32_t s_trigger_irq_count;
static uint32_t s_trigger_sequence;
static bool s_inited;
static portMUX_TYPE s_trigger_lock = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR trigger_isr_handler(void *arg)
{
    BaseType_t task_woken = pdFALSE;

    (void)arg;

    portENTER_CRITICAL_ISR(&s_trigger_lock);
    s_trigger_irq_count++;
    portEXIT_CRITICAL_ISR(&s_trigger_lock);

    if (s_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_task_handle, &task_woken);
    }

    if (task_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t pulse_output_init(gpio_num_t gpio_num)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    return gpio_set_level(gpio_num, 0);
}

static esp_err_t trigger_interrupt_init(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << s_cfg.trigger_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = s_cfg.trigger_intr_type,
    };

    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    return gpio_isr_handler_add(s_cfg.trigger_gpio, trigger_isr_handler, NULL);
}

static uint32_t pop_trigger_irq_count(void)
{
    uint32_t pending = 0;

    portENTER_CRITICAL(&s_trigger_lock);
    pending = s_trigger_irq_count;
    s_trigger_irq_count = 0;
    portEXIT_CRITICAL(&s_trigger_lock);

    return pending;
}

static esp_err_t emit_pulse(gpio_num_t gpio_num, uint32_t width_ms)
{
    esp_err_t err = gpio_set_level(gpio_num, 1);
    if (err != ESP_OK)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(width_ms));
    return gpio_set_level(gpio_num, 0);
}

static esp_err_t handle_trigger_event(void)
{
    preamplifier_trigger_context_t trigger_context = {
        .tick_count = xTaskGetTickCount(),
        .sequence = ++s_trigger_sequence,
    };
    preamplifier_trigger_sample_t trigger_sample = {
        .context = trigger_context,
        .signal_voltage_mv = 0,
        .signal_raw = 0,
    };

    ESP_LOGI(TAG, "Trigger captured: seq=%" PRIu32 " tick=%" PRIu32,
             trigger_context.sequence,
             (uint32_t)trigger_context.tick_count);

    if (s_cfg.capture_trigger_context != NULL)
    {
        s_cfg.capture_trigger_context(&trigger_context, s_cfg.user_ctx);
    }

    // Allow the peak-hold/output path to settle before ADC sampling.
    if (s_cfg.signal_sample_delay_ms > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(s_cfg.signal_sample_delay_ms));
    }

    esp_err_t err = adc_io4_get_voltage_mv(&trigger_sample.signal_voltage_mv, &trigger_sample.signal_raw);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_io4_get_voltage_mv failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Signal sampled: raw=%d voltage=%d mV",
             trigger_sample.signal_raw,
             trigger_sample.signal_voltage_mv);

    if (s_cfg.on_trigger_sample != NULL)
    {
        s_cfg.on_trigger_sample(&trigger_sample, s_cfg.user_ctx);
    }

    err = emit_pulse(s_cfg.restart_gpio, s_cfg.restart_pulse_width_ms);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "restart pulse failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Restart pulse emitted on GPIO%d", s_cfg.restart_gpio);
    return ESP_OK;
}

esp_err_t preamplifier_read_cathode_monitor_mv(int *monitor_mv, int *raw, int *sipm_input_mv)
{
    if (monitor_mv == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

    int local_mv = 0;
    int local_raw = 0;
    esp_err_t err = adc_io6_get_voltage_mv(&local_mv, &local_raw);
    if (err != ESP_OK)
    {
        return err;
    }

    *monitor_mv = local_mv;

    if (raw != NULL)
    {
        *raw = local_raw;
    }

    if (sipm_input_mv != NULL)
    {
        *sipm_input_mv = local_mv * s_cfg.cathode_monitor_scale;
    }

    return ESP_OK;
}

esp_err_t preamplifier_trigger_chargein_pulse(void)
{
    if (!s_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_cfg.enable_chargein)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return emit_pulse(s_cfg.chargein_gpio, s_cfg.chargein_pulse_width_ms);
}

static void preamplifier_task(void *arg)
{
    TickType_t next_chargein_tick = xTaskGetTickCount() + pdMS_TO_TICKS(s_cfg.chargein_period_ms);
    TickType_t chargein_high_since = 0;
    TickType_t last_cathode_log_tick = 0;
    bool chargein_high = false;

    (void)arg;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(s_cfg.task_interval_ms));

        uint32_t pending = pop_trigger_irq_count();
        while (pending > 0)
        {
            handle_trigger_event();
            pending--;
        }

        TickType_t now = xTaskGetTickCount();

        if (s_cfg.enable_chargein && s_cfg.chargein_period_ms > 0)
        {
            if (!chargein_high && now >= next_chargein_tick)
            {
                if (gpio_set_level(s_cfg.chargein_gpio, 1) == ESP_OK)
                {
                    chargein_high = true;
                    chargein_high_since = now;
                    next_chargein_tick = now + pdMS_TO_TICKS(s_cfg.chargein_period_ms);
                }
            }
            else if (chargein_high && (now - chargein_high_since) >= pdMS_TO_TICKS(s_cfg.chargein_pulse_width_ms))
            {
                if (gpio_set_level(s_cfg.chargein_gpio, 0) == ESP_OK)
                {
                    chargein_high = false;
                }
            }
        }

        if ((now - last_cathode_log_tick) >= pdMS_TO_TICKS(s_cfg.cathode_log_period_ms))
        {
            int cathode_monitor_mv = 0;
            int cathode_raw = 0;
            int sipm_input_mv = 0;
            esp_err_t err = preamplifier_read_cathode_monitor_mv(&cathode_monitor_mv, &cathode_raw, &sipm_input_mv);
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG,
                         "Cathode monitor raw=%d voltage=%d mV, SiPM input approx=%d mV, DAC1=%d mV, DAC2=%d mV",
                         cathode_raw,
                         cathode_monitor_mv,
                         sipm_input_mv,
                         DAC1_TARGET_VOLTAGE_MV,
                         DAC2_TARGET_VOLTAGE_MV);
            }
            else
            {
                ESP_LOGE(TAG, "adc_io6_get_voltage_mv failed: %s", esp_err_to_name(err));
            }

            last_cathode_log_tick = now;
        }
    }
}

void preamplifier_get_default_config(preamplifier_config_t *config)
{
    if (config != NULL)
    {
        *config = s_default_cfg;
    }
}

esp_err_t preamplifier_init(const preamplifier_config_t *config)
{
    if (config == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (!GPIO_IS_VALID_GPIO(config->trigger_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->restart_gpio) ||
        config->task_interval_ms == 0 ||
        config->cathode_log_period_ms == 0 ||
        (config->enable_chargein && !GPIO_IS_VALID_OUTPUT_GPIO(config->chargein_gpio)) ||
        (config->enable_chargein && config->chargein_pulse_width_ms > config->chargein_period_ms))
    {
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = *config;
    DAC1_TARGET_VOLTAGE_MV = s_cfg.cathode_voltage_mv;
    DAC2_TARGET_VOLTAGE_MV = s_cfg.vref_voltage_mv;

    esp_err_t err = dac_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dac_init failed: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }

    err = adc_io6_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_io6_init failed: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }

    err = trigger_interrupt_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "trigger interrupt init failed: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }

    if (s_cfg.enable_chargein)
    {
        err = pulse_output_init(s_cfg.chargein_gpio);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "chargein output init failed: %s (0x%x)", esp_err_to_name(err), err);
            return err;
        }
    }

    err = pulse_output_init(s_cfg.restart_gpio);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "restart output init failed: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }

    BaseType_t task_ok = xTaskCreate(
        preamplifier_task,
        "preamplifier_task",
        4096,
        NULL,
        tskIDLE_PRIORITY + 1,
        &s_task_handle);
    if (task_ok != pdPASS)
    {
        s_task_handle = NULL;
        return ESP_FAIL;
    }

    s_inited = true;
    ESP_LOGI(TAG,
             "init done: cathode=%d mV vref=%d mV trigger_gpio=%d chargein=%s chargein_gpio=%d restart_gpio=%d",
             s_cfg.cathode_voltage_mv,
             s_cfg.vref_voltage_mv,
             s_cfg.trigger_gpio,
             s_cfg.enable_chargein ? "on" : "off",
             s_cfg.chargein_gpio,
             s_cfg.restart_gpio);
    return ESP_OK;
}