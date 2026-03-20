入口：
    ESP_LOGI(TAG, "app_main start");
    esp_err_t err = dac_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dac_init failed: %s (0x%x)", esp_err_to_name(err), err);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "dac_init finished");


怎么控制电压值：
dac.h:
    #define DAC1_TARGET_VOLTAGE_MV 200
    #define DAC2_TARGET_VOLTAGE_MV 200

DAC1是cathode的输出，单位是mv，经过大约110倍放大


Preamplifier 模块说明

开机后首先使用ADC1和ADC2分别产生Cathode电压（用于为SiPM提供电压，该ADC不能过大，容易烧坏）和Vref参考电压（200mv-500mv，用于为前放板提供触发阈）
首先用ADC测量Cathode mon（IO6），它乘以33等于SiPM引脚输入电压27V。正常调试使用Chargein（IO8），ESP32用IO8向外输出50ms3V3脉冲即可在前放板上模拟信号输入。
在信号输入后，首先是Triger（IO10）收到一个3V3脉冲信号，可以以此作为中断的触发。中断中，先记录GPS触发的时间点和位置信息，然后用ADC采集Signal（IO4），最后使用Restart（IO10）向外输出一个50ms3V3脉冲，将前放板峰保持上的电压释放掉。

1. 模块功能
- 统一完成前置放大器相关初始化：DAC、ADC、触发中断、任务循环。
- 在触发信号到来后采样信号电压，并可通过回调上报采样结果。
- 可选输出 CHARGEIN 周期脉冲，并在触发后输出 RESTART 脉冲。
- 周期读取阴极监测电压，并换算近似 SiPM 输入电压。

2. 入口用法
```c
#include "Preamplifier.h"

void app_main(void)
{
    preamplifier_config_t config;
    preamplifier_get_default_config(&config);

    // 根据实际需求覆盖默认配置
    config.enable_chargein = false;

    esp_err_t err = preamplifier_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE("APP_MAIN", "preamplifier_init failed: %s (0x%x)", esp_err_to_name(err), err);
        return;
    }

    ESP_LOGI("APP_MAIN", "Preamplifier initialized");
}
```

3. 关键配置项（preamplifier_config_t）
- cathode_voltage_mv: 阴极目标电压（写入 DAC1）。
- vref_voltage_mv: 参考目标电压（写入 DAC2）。
- cathode_monitor_scale: 阴极监测电压到 SiPM 输入电压的换算比例。
- trigger_gpio / trigger_intr_type: 外部触发输入和中断触发类型。
- signal_sample_delay_ms: 触发后到 ADC 采样前的等待时间。
- enable_chargein: 是否开启 CHARGEIN 输出。
- chargein_gpio / chargein_period_ms / chargein_pulse_width_ms: CHARGEIN 脉冲参数。
- restart_gpio / restart_pulse_width_ms: RESTART 脉冲参数。
- cathode_log_period_ms: 阴极监测日志周期。
- task_interval_ms: 后台任务轮询周期。
- capture_trigger_context / on_trigger_sample: 触发上下文与采样数据回调。

4. 主要接口
- preamplifier_get_default_config: 获取默认配置。
- preamplifier_init: 初始化模块并启动后台任务。
- preamplifier_read_cathode_monitor_mv: 读取阴极监测电压/原始值/换算值。
- preamplifier_trigger_chargein_pulse: 手动触发一次 CHARGEIN 脉冲（需 enable_chargein=true）。

5. 注意事项
- `preamplifier_init` 仅允许初始化一次，重复调用会返回 `ESP_ERR_INVALID_STATE`。
- 若启用 CHARGEIN，`chargein_pulse_width_ms` 必须小于等于 `chargein_period_ms`。
- 触发引脚需为有效 GPIO，RESTART/CHARGEIN 需为可输出 GPIO。
