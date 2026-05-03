#ifndef ACCE_H
#define ACCE_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Initialize the SC7A20H accelerometer over I2C.
 *        Assumes I2C_NUM_0 is already initialized (by BSPInit or TMP112 code).
 *        If I2C driver is not yet installed, installs it.
 */
esp_err_t acce_init(void);

/**
 * @brief Read XYZ acceleration values from SC7A20H.
 * @param[out] x  X-axis value (-2048..2047)
 * @param[out] y  Y-axis value
 * @param[out] z  Z-axis value
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t acce_read_xyz(int16_t *x, int16_t *y, int16_t *z);

#endif // ACCE_H
