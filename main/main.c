/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_module.h"
#include "gps_interrupt.h"

void app_main(void) {
	gps_start();

	while (true) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
