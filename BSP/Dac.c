#include "Dac.h"
#include <string.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#ifndef DAC7311_SPI_MAX_BITS
#define DAC7311_SPI_MAX_BITS 16
#endif

static const char *TAG = "DAC";

int DAC1_TARGET_VOLTAGE_MV = 239;
int DAC2_TARGET_VOLTAGE_MV = 200;

static dac7311_t s_dac1 = {
    .host = SPI2_HOST,
    .mosi_io = DAC_MOSI,
    .sclk_io = DAC_SCLK,
    .cs_io = DAC_CS1,
    .clock_hz = DAC_CLOCK_HZ,
    .spi_mode = DAC_SPI_MODE,
};

static dac7311_t s_dac2 = {
    .host = SPI2_HOST,
    .mosi_io = DAC_MOSI,
    .sclk_io = DAC_SCLK,
    .cs_io = DAC_CS2,
    .clock_hz = DAC_CLOCK_HZ,
    .spi_mode = DAC_SPI_MODE,
};

static bool s_dac_inited;

static adc_oneshot_unit_handle_t s_adc1_handle;
static adc_cali_handle_t s_adc1_cali_handle;
static adc_cali_handle_t s_adc1_io4_cali_handle;
static bool s_adc_cali_enabled;
static bool s_adc_io4_cali_enabled;
static bool s_adc_inited;

static const adc_channel_t ADC_IO6_CHANNEL = ADC_CHANNEL_5;
static const adc_atten_t ADC_IO6_ATTEN = ADC_ATTEN_DB_12;
static const adc_channel_t ADC_IO4_CHANNEL = ADC_CHANNEL_3;
static const adc_atten_t ADC_IO4_ATTEN = ADC_ATTEN_DB_12;

static esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    esp_err_t err = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_config, out_handle);
    if (err == ESP_OK)
    {
        return ESP_OK;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_line_fitting(&cali_config, out_handle);
    if (err == ESP_OK)
    {
        return ESP_OK;
    }
#endif

    return err;
}

static inline uint16_t dac7311_build_frame(dac7311_power_mode_t pd, uint16_t code12)
{
    code12 &= 0x0FFF; // 12-bit
    uint16_t frame = 0;
    frame |= ((uint16_t)pd & 0x3) << 14; // PD1:PD0 at bits 15..14
    frame |= (code12 << 2);              // D11..D0 at bits 13..2
    // bits 1..0 don't care
    return frame;
}

static esp_err_t dac7311_write16(dac7311_t *dac, uint16_t frame)
{
    if (!dac || !dac->dev)
    {
        ESP_LOGE(TAG, "dac7311_write16 invalid arg: dac=%p dev=%p", dac, dac ? dac->dev : NULL);
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // SPI driver sends MSB-first by default
    t.length = 16; // in bits
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = (frame >> 8) & 0xFF;
    t.tx_data[1] = frame & 0xFF;

    esp_err_t err = spi_device_transmit(dac->dev, &t);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_device_transmit failed: %s, frame=0x%04X", esp_err_to_name(err), frame);
    }
    else
    {
        ESP_LOGD(TAG, "spi tx ok, frame=0x%04X", frame);
    }
    return err;
}

esp_err_t dac7311_init(dac7311_t *dac)
{
    if (!dac)
    {
        ESP_LOGE(TAG, "dac7311_init invalid arg: dac=NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!GPIO_IS_VALID_OUTPUT_GPIO(dac->mosi_io) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(dac->sclk_io) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(dac->cs_io))
    {
        ESP_LOGE(TAG, "invalid gpio(s): mosi=%d sclk=%d cs=%d", dac->mosi_io, dac->sclk_io, dac->cs_io);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "init start host=%d mosi=%d sclk=%d cs=%d clk=%d mode=%d",
             dac->host, dac->mosi_io, dac->sclk_io, dac->cs_io, dac->clock_hz, dac->spi_mode);

    // 1) Init SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = dac->mosi_io,
        .miso_io_num = -1, // no MISO
        .sclk_io_num = dac->sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
    };
    ESP_LOGI(TAG, "Init SPI bus");
    // NOTE: If SPI bus already initialized elsewhere, spi_bus_initialize will fail.
    // In that case, remove spi_bus_initialize from here and only call spi_bus_add_device.
    esp_err_t err = spi_bus_initialize(dac->host, &buscfg, SPI_DMA_CH_AUTO);
    ESP_LOGI(TAG, "spi_bus_initialize");
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }
    if (err == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "spi bus already initialized, continue");
    }
    ESP_LOGI(TAG, "Add device");
    // 2) Add device (CS controlled by SPI peripheral)
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = dac->clock_hz > 0 ? dac->clock_hz : (10 * 1000 * 1000),
        .mode = dac->spi_mode,      // usually 1
        .spics_io_num = dac->cs_io, // SYNC/CS
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX, // MOSI only is fine
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
    };
    ESP_LOGI(TAG, "Add device success");
    err = spi_bus_add_device(dac->host, &devcfg, &dac->dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "device added on cs=%d", dac->cs_io);

    // Optional: ensure CS idles high (SPI driver does this), and write 0 code once.
    return dac7311_set_code(dac, 0);
}

esp_err_t dac7311_set_code(dac7311_t *dac, uint16_t code)
{
    uint16_t frame = dac7311_build_frame(DAC7311_PD_NORMAL, code);
    ESP_LOGD(TAG, "set code=%u", code & 0x0FFF);
    return dac7311_write16(dac, frame);
}

esp_err_t dac7311_set_millivolts(dac7311_t *dac, uint32_t vout_mv, uint32_t vref_mv)
{
    if (vref_mv == 0)
    {
        ESP_LOGE(TAG, "vref_mv is 0");
        return ESP_ERR_INVALID_ARG;
    }
    if (vout_mv > vref_mv)
    {
        ESP_LOGW(TAG, "vout_mv(%lu) > vref_mv(%lu), clip to vref", (unsigned long)vout_mv, (unsigned long)vref_mv);
        vout_mv = vref_mv;
    }

    // code = round( vout / vref * 4095 )
    // Use integer math with rounding:
    uint32_t numerator = vout_mv * 4095u + (vref_mv / 2u);
    uint16_t code = (uint16_t)(numerator / vref_mv);

    ESP_LOGI(TAG, "set voltage %lumV/%lumV -> code=%u", (unsigned long)vout_mv, (unsigned long)vref_mv, code);

    return dac7311_set_code(dac, code);
}

esp_err_t dac7311_set_power_mode(dac7311_t *dac, dac7311_power_mode_t mode)
{
    // When entering PD, datasheet defines output state by mode.
    // We keep the code as 0 here; you can keep last code if you prefer.
    uint16_t frame = dac7311_build_frame(mode, 0);
    ESP_LOGI(TAG, "set power mode=%d", mode);
    return dac7311_write16(dac, frame);
}

esp_err_t dac_init()
{
    esp_err_t err = ESP_OK;

    if (!s_dac_inited)
    {
        ESP_LOGI(TAG, "dac_init start");

        err = dac7311_init(&s_dac1);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "dac7311_init(dac1) failed: %s (0x%x)", esp_err_to_name(err), err);
            return err;
        }
        ESP_LOGI(TAG, "dac1 init ok");

        err = dac7311_init(&s_dac2);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "dac7311_init(dac2) failed: %s (0x%x)", esp_err_to_name(err), err);
            return err;
        }
        ESP_LOGI(TAG, "dac2 init ok");
        s_dac_inited = true;
    }

    err = dac7311_set_millivolts(&s_dac1, DAC1_TARGET_VOLTAGE_MV, DAC_VREF_MV);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dac7311_set_millivolts(dac1) failed: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }
    ESP_LOGI(TAG, "dac1 set voltage ok: %dmV", DAC1_TARGET_VOLTAGE_MV);

    err = dac7311_set_millivolts(&s_dac2, DAC2_TARGET_VOLTAGE_MV, DAC_VREF_MV);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dac7311_set_millivolts(dac2) failed: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }
    ESP_LOGI(TAG, "dac2 set voltage ok: %dmV", DAC2_TARGET_VOLTAGE_MV);

    ESP_LOGI(TAG, "dac_init done");
    return ESP_OK;
}

esp_err_t dac_set_output_mv(int dac_index, uint32_t vout_mv)
{
    if (!s_dac_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (dac_index == 1)
    {
        DAC1_TARGET_VOLTAGE_MV = (int)vout_mv;
        return dac7311_set_millivolts(&s_dac1, vout_mv, DAC_VREF_MV);
    }
    if (dac_index == 2)
    {
        DAC2_TARGET_VOLTAGE_MV = (int)vout_mv;
        return dac7311_set_millivolts(&s_dac2, vout_mv, DAC_VREF_MV);
    }

    return ESP_ERR_INVALID_ARG;
}

esp_err_t adc_io6_init(void)
{
    if (s_adc_inited)
    {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc1_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_IO6_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc1_handle, ADC_IO6_CHANNEL, &chan_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t io4_chan_cfg = {
        .atten = ADC_IO4_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc1_handle, ADC_IO4_CHANNEL, &io4_chan_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_config_channel(IO4) failed: %s", esp_err_to_name(err));
        return err;
    }

    err = adc_calibration_init(ADC_UNIT_1, ADC_IO6_CHANNEL, ADC_IO6_ATTEN, &s_adc1_cali_handle);
    if (err == ESP_OK)
    {
        s_adc_cali_enabled = true;
        ESP_LOGI(TAG, "adc calibration enabled on GPIO%d", ADC_IO6_GPIO);
    }
    else
    {
        s_adc_cali_enabled = false;
        ESP_LOGW(TAG, "adc calibration unavailable, use approximate conversion");
    }

    err = adc_calibration_init(ADC_UNIT_1, ADC_IO4_CHANNEL, ADC_IO4_ATTEN, &s_adc1_io4_cali_handle);
    if (err == ESP_OK)
    {
        s_adc_io4_cali_enabled = true;
        ESP_LOGI(TAG, "adc calibration enabled on GPIO%d", ADC_IO4_GPIO);
    }
    else
    {
        s_adc_io4_cali_enabled = false;
        ESP_LOGW(TAG, "adc calibration unavailable on GPIO%d, use approximate conversion", ADC_IO4_GPIO);
    }

    s_adc_inited = true;
    ESP_LOGI(TAG, "adc_io6_init done, GPIO=%d channel=%d", ADC_IO6_GPIO, ADC_IO6_CHANNEL);
    return ESP_OK;
}

esp_err_t adc_io4_get_voltage_mv(int *voltage_mv, int *raw)
{
    if (!voltage_mv)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_adc_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

    int raw_code = 0;
    esp_err_t err = adc_oneshot_read(s_adc1_handle, ADC_IO4_CHANNEL, &raw_code);
    if (err != ESP_OK)
    {
        return err;
    }

    int mv = 0;
    if (s_adc_io4_cali_enabled)
    {
        err = adc_cali_raw_to_voltage(s_adc1_io4_cali_handle, raw_code, &mv);
        if (err != ESP_OK)
        {
            return err;
        }
    }
    else
    {
        mv = (raw_code * 3300) / 4095;
    }

    *voltage_mv = mv;
    if (raw)
    {
        *raw = raw_code;
    }
    return ESP_OK;
}

esp_err_t adc_io6_get_voltage_mv(int *voltage_mv, int *raw)
{
    if (!voltage_mv)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_adc_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

    int raw_code = 0;
    esp_err_t err = adc_oneshot_read(s_adc1_handle, ADC_IO6_CHANNEL, &raw_code);
    if (err != ESP_OK)
    {
        return err;
    }

    int mv = 0;
    if (s_adc_cali_enabled)
    {
        err = adc_cali_raw_to_voltage(s_adc1_cali_handle, raw_code, &mv);
        if (err != ESP_OK)
        {
            return err;
        }
    }
    else
    {
        mv = (raw_code * 3300) / 4095;
    }

    *voltage_mv = mv;
    if (raw)
    {
        *raw = raw_code;
    }
    return ESP_OK;
}