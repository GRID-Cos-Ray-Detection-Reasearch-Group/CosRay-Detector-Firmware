#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C"
{
#endif

    extern int DAC1_TARGET_VOLTAGE_MV;
    extern int DAC2_TARGET_VOLTAGE_MV;
    // #define DAC1_TARGET_VOLTAGE_MV 239
    // #define DAC2_TARGET_VOLTAGE_MV 200

#define DAC_MOSI 33
#define DAC_SCLK 48

#define DAC_CS1 34

#define DAC_CS2 37
#define DAC_CLOCK_HZ (5 * 1000 * 1000)
#define DAC_SPI_MODE 1
#define DAC_VREF_MV 3300

#define ADC_IO6_GPIO 6
#define ADC_IO4_GPIO 4

    typedef enum
    {
        DAC7311_PD_NORMAL = 0,  // PD1:PD0 = 00
        DAC7311_PD_1K_TO_GND,   // 01
        DAC7311_PD_100K_TO_GND, // 10
        DAC7311_PD_HIZ          // 11
    } dac7311_power_mode_t;

    typedef struct
    {
        spi_host_device_t host;  // e.g. SPI2_HOST
        int mosi_io;             // GPIO for MOSI
        int sclk_io;             // GPIO for SCLK
        int cs_io;               // GPIO for SYNC/CS
        int clock_hz;            // e.g. 10*1000*1000
        int spi_mode;            // usually 1 (CPOL=0, CPHA=1)
        spi_device_handle_t dev; // internal
    } dac7311_t;

    /**
     * @brief
     */
    esp_err_t dac_init();

    /**
     * @brief Update DAC output voltage without re-initializing SPI/device.
     * @param dac_index 1 for DAC1, 2 for DAC2.
     */
    esp_err_t dac_set_output_mv(int dac_index, uint32_t vout_mv);

    /**
     * @brief Initialize ADC1 oneshot for GPIO6 input.
     */
    esp_err_t adc_io6_init(void);

    /**
     * @brief Read GPIO6 voltage.
     * @param[out] voltage_mv Measured voltage in mV.
     * @param[out] raw Raw ADC code, optional (can be NULL).
     */
    esp_err_t adc_io6_get_voltage_mv(int *voltage_mv, int *raw);

    /**
     * @brief Read GPIO4 voltage.
     * @param[out] voltage_mv Measured voltage in mV.
     * @param[out] raw Raw ADC code, optional (can be NULL).
     */
    esp_err_t adc_io4_get_voltage_mv(int *voltage_mv, int *raw);

    /**
     * @brief Initialize SPI bus + attach DAC7311 device.
     */
    esp_err_t dac7311_init(dac7311_t *dac);

    /**
     * @brief Set output code (0..4095). Updates VOUT.
     */
    esp_err_t dac7311_set_code(dac7311_t *dac, uint16_t code);

    /**
     * @brief Set output voltage in millivolts given Vref/AVDD in millivolts.
     *        Output is clipped to 0..Vref_mv.
     */
    esp_err_t dac7311_set_millivolts(dac7311_t *dac, uint32_t vout_mv, uint32_t vref_mv);

    /**
     * @brief Put DAC into power-down mode (or back to normal).
     *        When entering PD, output state depends on mode.
     */
    esp_err_t dac7311_set_power_mode(dac7311_t *dac, dac7311_power_mode_t mode);

#ifdef __cplusplus
}
#endif
