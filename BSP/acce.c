#include "acce.h"

#include <string.h>
#include <stdbool.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ACCE";

#define I2C_PORT        I2C_NUM_0
#define I2C_SCL_GPIO    1
#define I2C_SDA_GPIO    2
#define I2C_FREQ_HZ     400000
#define I2C_TIMEOUT_MS  100

#define SC7A20H_ADDR_0      0x18
#define SC7A20H_ADDR_1      0x19

#define SC7A20H_REG_WHO_AM_I    0x0F
#define SC7A20H_WHO_AM_I_VAL    0x11
#define SC7A20H_REG_CTRL1       0x20
#define SC7A20H_REG_CTRL4       0x23
#define SC7A20H_REG_STATUS      0x27
#define SC7A20H_REG_OUT_X_L     0x28
#define SC7A20H_REG_AUTOINC     0x80

static uint8_t s_sensor_addr = 0;
static bool s_inited = false;

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
                 addr, whoami, SC7A20H_WHO_AM_I_VAL);
    }

    return ESP_ERR_NOT_FOUND;
}

static esp_err_t sc7a20h_configure(uint8_t addr)
{
    // ODR=100Hz, XYZ enabled
    esp_err_t err = i2c_write_reg(addr, SC7A20H_REG_CTRL1, 0x57);
    if (err != ESP_OK) {
        return err;
    }
    // High-resolution mode
    err = i2c_write_reg(addr, SC7A20H_REG_CTRL4, 0x08);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

esp_err_t acce_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

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
    if (err == ESP_ERR_INVALID_STATE) {
        // I2C already initialized by another module, continue normally
        ESP_LOGW(TAG, "I2C driver already installed, continuing");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = sc7a20h_find_address(&s_sensor_addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SC7A20H not found on I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    err = sc7a20h_configure(s_sensor_addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SC7A20H configure failed: %s", esp_err_to_name(err));
        return err;
    }

    s_inited = true;
    ESP_LOGI(TAG, "SC7A20H initialized at addr=0x%02X", s_sensor_addr);
    return ESP_OK;
}

esp_err_t acce_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    if (!x || !y || !z) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[6] = {0};
    esp_err_t err = i2c_read_regs(s_sensor_addr, SC7A20H_REG_OUT_X_L, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    *x = ((int16_t)((raw[1] << 8) | raw[0])) >> 4;
    *y = ((int16_t)((raw[3] << 8) | raw[2])) >> 4;
    *z = ((int16_t)((raw[5] << 8) | raw[4])) >> 4;
    return ESP_OK;
}
