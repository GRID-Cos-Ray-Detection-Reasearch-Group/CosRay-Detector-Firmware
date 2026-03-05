#include "Dac.h"
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#ifndef DAC7311_SPI_MAX_BITS
#define DAC7311_SPI_MAX_BITS 16
#endif

static const char *TAG = "DAC";



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
    ESP_LOGI(TAG, "dac_init start");

    dac7311_t dac1 = {
        .host = SPI2_HOST,
        .mosi_io = DAC_MOSI,
        .sclk_io = DAC_SCLK,
        .cs_io = DAC_CS1, // DAC1 SYNC/CS
        .clock_hz = DAC_CLOCK_HZ,
        .spi_mode = DAC_SPI_MODE,
    };

    dac7311_t dac2 = {
        .host = SPI2_HOST,
        .mosi_io = DAC_MOSI,
        .sclk_io = DAC_SCLK,
        .cs_io = DAC_CS2, // DAC2 SYNC/CS（换一个GPIO）
        .clock_hz = DAC_CLOCK_HZ,
        .spi_mode = DAC_SPI_MODE,
    };

    esp_err_t err = dac7311_init(&dac1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dac7311_init(dac1) failed: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }
    ESP_LOGI(TAG, "dac1 init ok");

    err = dac7311_init(&dac2);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dac7311_init(dac2) failed: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }
    ESP_LOGI(TAG, "dac2 init ok");

    err = dac7311_set_millivolts(&dac1, DAC1_TARGET_VOLTAGE_MV, DAC_VREF_MV);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dac7311_set_millivolts(dac1) failed: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }
    ESP_LOGI(TAG, "dac1 set voltage ok: %dmV", DAC1_TARGET_VOLTAGE_MV);

    err = dac7311_set_millivolts(&dac2, DAC2_TARGET_VOLTAGE_MV, DAC_VREF_MV);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dac7311_set_millivolts(dac2) failed: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }
    ESP_LOGI(TAG, "dac2 set voltage ok: %dmV", DAC2_TARGET_VOLTAGE_MV);

    ESP_LOGI(TAG, "dac_init done");
    return ESP_OK;
}