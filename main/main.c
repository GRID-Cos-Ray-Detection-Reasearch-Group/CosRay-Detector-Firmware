#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SCL_GPIO 1
#define I2C_SDA_GPIO 2
#define I2C_FREQ_HZ 400000
#define I2C_TIMEOUT_MS 100

#define SC7A20H_ADDR_0 0x18
#define SC7A20H_ADDR_1 0x19

#define SC7A20H_REG_WHO_AM_I 0x0F
#define SC7A20H_WHO_AM_I_VAL 0x11
#define SC7A20H_REG_CTRL1 0x20
#define SC7A20H_REG_CTRL4 0x23
#define SC7A20H_REG_STATUS 0x27
#define SC7A20H_REG_OUT_X_L 0x28
#define SC7A20H_REG_AUTOINC 0x80

static const char *TAG = "acce_check";

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static bool i2c_device_present(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

static esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *out)
{
    return i2c_master_write_read_device(I2C_PORT,
                                        addr,
                                        &reg,
                                        1,
                                        out,
                                        1,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, addr, tx, sizeof(tx), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t i2c_read_regs(uint8_t addr, uint8_t start_reg, uint8_t *out, size_t len)
{
    uint8_t reg = start_reg;
    if (len > 1) {
        reg |= SC7A20H_REG_AUTOINC;
    }

    return i2c_master_write_read_device(I2C_PORT,
                                        addr,
                                        &reg,
                                        1,
                                        out,
                                        len,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static void scan_i2c_bus(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus on SCL=GPIO%d SDA=GPIO%d ...", I2C_SCL_GPIO, I2C_SDA_GPIO);

    int found = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        if (i2c_device_present(addr)) {
            ESP_LOGI(TAG, "Found I2C device at 0x%02X", addr);
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "No I2C devices found. Check power, wiring, pull-up resistors and address pins.");
    } else {
        ESP_LOGI(TAG, "I2C scan complete, %d device(s) detected.", found);
    }
}

static esp_err_t sc7a20h_find_address(uint8_t *out_addr)
{
    const uint8_t candidates[2] = {SC7A20H_ADDR_0, SC7A20H_ADDR_1};

    for (size_t i = 0; i < 2; i++) {
        uint8_t addr = candidates[i];
        if (!i2c_device_present(addr)) {
            continue;
        }

        uint8_t whoami = 0;
        esp_err_t err = i2c_read_reg(addr, SC7A20H_REG_WHO_AM_I, &whoami);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Device at 0x%02X found but WHO_AM_I read failed: %s", addr, esp_err_to_name(err));
            continue;
        }

        if (whoami == SC7A20H_WHO_AM_I_VAL) {
            *out_addr = addr;
            ESP_LOGI(TAG, "SC7A20H detected at 0x%02X, WHO_AM_I=0x%02X", addr, whoami);
            return ESP_OK;
        }

        ESP_LOGW(TAG,
                 "Device at 0x%02X responded, but WHO_AM_I=0x%02X (expect 0x%02X)",
                 addr,
                 whoami,
                 SC7A20H_WHO_AM_I_VAL);
    }

    return ESP_ERR_NOT_FOUND;
}

static esp_err_t sc7a20h_configure(uint8_t addr)
{
    esp_err_t err = i2c_write_reg(addr, SC7A20H_REG_CTRL1, 0x57);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_write_reg(addr, SC7A20H_REG_CTRL4, 0x08);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static esp_err_t sc7a20h_read_xyz(uint8_t addr, int16_t *x, int16_t *y, int16_t *z, uint8_t *status)
{
    esp_err_t err = i2c_read_reg(addr, SC7A20H_REG_STATUS, status);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t raw[6] = {0};
    err = i2c_read_regs(addr, SC7A20H_REG_OUT_X_L, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    *x = ((int16_t)((raw[1] << 8) | raw[0])) >> 4;
    *y = ((int16_t)((raw[3] << 8) | raw[2])) >> 4;
    *z = ((int16_t)((raw[5] << 8) | raw[4])) >> 4;
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting SC7A20H status checker");

    ESP_ERROR_CHECK(i2c_master_init());
    scan_i2c_bus();

    uint8_t sensor_addr = 0;
    ESP_ERROR_CHECK(sc7a20h_find_address(&sensor_addr));
    ESP_ERROR_CHECK(sc7a20h_configure(sensor_addr));
    ESP_LOGI(TAG, "SC7A20H configured: ODR=100Hz, XYZ enabled");

    bool has_prev = false;
    int16_t prev_x = 0;
    int16_t prev_y = 0;
    int16_t prev_z = 0;
    uint32_t no_change_count = 0;

    while (1) {
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;
        uint8_t status = 0;

        esp_err_t err = sc7a20h_read_xyz(sensor_addr, &x, &y, &z, &status);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Read SC7A20H data failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int dx = 0;
        int dy = 0;
        int dz = 0;
        if (has_prev) {
            dx = abs((int)x - (int)prev_x);
            dy = abs((int)y - (int)prev_y);
            dz = abs((int)z - (int)prev_z);
            if ((dx + dy + dz) < 5) {
                no_change_count++;
            } else {
                no_change_count = 0;
            }
        }

        ESP_LOGI(TAG,
                 "SC7A20H OK | STATUS=0x%02X(ZYXDA=%d) | X=%d Y=%d Z=%d | dX=%d dY=%d dZ=%d",
                 status,
                 (status & 0x08) ? 1 : 0,
                 x,
                 y,
                 z,
                 dx,
                 dy,
                 dz);

        if (no_change_count >= 20) {
            ESP_LOGW(TAG, "Data changed very little for a while. Try moving the board to verify dynamic response.");
            no_change_count = 0;
        }

        prev_x = x;
        prev_y = y;
        prev_z = z;
        has_prev = true;

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
