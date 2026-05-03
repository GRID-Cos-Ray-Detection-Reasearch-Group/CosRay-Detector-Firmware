#include "config.h"
#include "typedefs.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "flashstorage.h"
#include "Dac.h"
#include "Preamplifier.h"
#include "acce.h"
#include "gps_module.h"

static const char *TAG = "DataPeripheral";

#ifndef SIPM_THRESHOLD_RAW
#define SIPM_THRESHOLD_RAW 2000U
#endif

#define TIMELINE_INTERVAL_US (5ULL * 1000000ULL)
#ifndef MUON_PKG_MAX_EVENTS
#define MUON_PKG_MAX_EVENTS 35
#endif

#define OPCODE_TRIGGER 0xA0
#define OPCODE_PPS 0xA1
#define OPCODE_TMP_ALERT 0xA2
#define OPCODE_GPS 0xA3

// 外部定义
extern QueueHandle_t DataQueue;
extern QueueHandle_t TxQueue;
extern QueueHandle_t FlashQueue;

extern uint16_t CalcCRC(const uint8_t *data, size_t length);

// 初始变量
static esp_timer_handle_t s_timeline_timer = NULL;

static MuonDataPkg_t s_muon_pkg;
static uint8_t s_muon_event_index = 0;
static uint32_t s_muon_pkg_cnt = 0;

static uint32_t s_local_pps_count = 0;
static int32_t s_local_gps_long = 0;
static int32_t s_local_gps_lat = 0;
static uint32_t s_local_gps_utc = 0;
static uint16_t s_local_SiPM_temp = 0;
static uint16_t s_local_MCU_temp = 0;

static bool g_running = false;
static uint32_t s_current_pkg_id = 0;
static uint8_t s_current_pkg_type = 0;

static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

static inline uint16_t adc_raw_to_energy(uint32_t raw) {
    if (raw > 4095U)
        raw = 4095U;
    return (uint16_t)((raw * 65535UL) / 4095UL);
}

static inline uint32_t ParsePkgIDFromCmd(const Command_t *cmd) {
    return ((uint32_t)cmd->data[1] << 24) | ((uint32_t)cmd->data[2] << 16) |
           ((uint32_t)cmd->data[3] << 8) | ((uint32_t)cmd->data[4]);
}

static void timeline_send_now(void) {
    TimeLinePkg_t pkg;
    memset(&pkg, 0, sizeof(pkg));

    pkg.head[0] = 0x12;
    pkg.head[1] = 0x34;
    pkg.head[2] = 0x56;
    pkg.PkgCnt = s_muon_pkg_cnt++;

    TimeLineData_t *td = &pkg.TimeLineData[0];
    td->cpu_time = now_us();
    td->pps = s_local_pps_count;
    td->utc = s_local_gps_utc;
    td->pps_utc = 0;
    td->cputime_pps = td->cpu_time;

    // Read GPS coordinates from gps_hex_data (UBX NAV-PVT little-endian int32_t)
    if (gps_hex_data_mutex != NULL &&
        xSemaphoreTake(gps_hex_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        gps_hex_data_t gd = current_gps_hex_data;
        xSemaphoreGive(gps_hex_data_mutex);

        // Longitude: bytes 0..3, little-endian int32_t (1e-7 deg)
        int32_t raw_lon = (int32_t)(
            ((uint32_t)gd.longitude[0]) |
            ((uint32_t)gd.longitude[1] << 8) |
            ((uint32_t)gd.longitude[2] << 16) |
            ((uint32_t)gd.longitude[3] << 24));

        // Latitude: bytes 0..3, little-endian int32_t (1e-7 deg)
        int32_t raw_lat = (int32_t)(
            ((uint32_t)gd.latitude[0]) |
            ((uint32_t)gd.latitude[1] << 8) |
            ((uint32_t)gd.latitude[2] << 16) |
            ((uint32_t)gd.latitude[3] << 24));

        td->gps_long = raw_lon;
        td->gps_lat = raw_lat;
        td->gps_alt = 0;

        // UTC: encode year/month/day/hour/min/sec as packed BCD-like uint32_t
        // year is 2 bytes LE
        uint16_t year = (uint16_t)(gd.year[0] | ((uint16_t)gd.year[1] << 8));
        // Pack as: [year-2000 (8b)] [month (4b)] [day (5b)] [hour (5b)] [min (6b)] [sec (4b)]
        // Simple packing: yymmddHHMM (shifted)
        uint32_t utc = ((uint32_t)(year & 0xFF) << 24) |
                       ((uint32_t)gd.month << 20) |
                       ((uint32_t)gd.day << 15) |
                       ((uint32_t)gd.hour << 10) |
                       ((uint32_t)gd.minute << 4) |
                       ((uint32_t)gd.second & 0xF);
        td->utc = utc;
        s_local_gps_utc = utc;
        s_local_gps_long = raw_lon;
        s_local_gps_lat = raw_lat;
    } else {
        td->gps_long = s_local_gps_long;
        td->gps_lat = s_local_gps_lat;
        td->gps_alt = 0;
    }

    // SiPM voltage monitor via preamplifier cathode monitor (IO6)
    int monitor_mv = 0;
    if (preamplifier_read_cathode_monitor_mv(&monitor_mv, NULL, NULL) == ESP_OK) {
        td->SiPMVmon = (uint16_t)(monitor_mv < 0 ? 0 : monitor_mv);
    }

    // SiPM current monitor (IO7)
    int v = 0;
    if (adc_io7_get_voltage_mv(&v, NULL) == ESP_OK) {
        td->SiPMImon = (uint16_t)(v < 0 ? 0 : v);
    }

    // Accelerometer
    int16_t ax = 0, ay = 0, az = 0;
    if (acce_read_xyz(&ax, &ay, &az) == ESP_OK) {
        // Clamp to int8_t range (-128..127)
        td->acc_x = (int8_t)(ax < -128 ? -128 : (ax > 127 ? 127 : ax));
        td->acc_y = (int8_t)(ay < -128 ? -128 : (ay > 127 ? 127 : ay));
        td->acc_z = (int8_t)(az < -128 ? -128 : (az > 127 ? 127 : az));
    }

    // Temperature (todo)
    td->SiPMTmp = s_local_SiPM_temp;
    td->MCUTmp = (uint8_t)s_local_MCU_temp;

    pkg.tail[0] = 0x78;
    pkg.tail[1] = 0x9A;
    pkg.tail[2] = 0xBC;

    pkg.crc = CalcCRC((uint8_t *)&pkg, sizeof(TimeLinePkg_t) - 2);

    TxPkg_t tx;
    memset(&tx, 0, sizeof(tx));
    memcpy(tx.data, &pkg, sizeof(pkg));
    tx.length = DATA_PACKAGE_SIZE;

    // 写入Flash
    if (xQueueSend(FlashQueue, &tx, pdMS_TO_TICKS(50)) != pdTRUE)
        ESP_LOGI(TAG, "Timeline packet dropped (FlashQueue full)");
    else
        ESP_LOGI(TAG, "Timeline packet saved to flash, PkgCnt=%u",
                 (unsigned)pkg.PkgCnt);
}

static void timeline_timer_cb(void *arg) {
    (void)arg;
    timeline_send_now();
}

static void flush_muon_pkg_if_any(void) {
    if (s_muon_event_index == 0)
        return;

    s_muon_pkg.utc = s_local_gps_utc;
    s_muon_pkg.tail[0] = 0xDD;
    s_muon_pkg.tail[1] = 0xEE;
    s_muon_pkg.tail[2] = 0xFF;

    s_muon_pkg.crc = CalcCRC((uint8_t *)&s_muon_pkg, sizeof(MuonDataPkg_t) - 2);

    TxPkg_t tx;
    memset(&tx, 0, sizeof(tx));
    memcpy(tx.data, &s_muon_pkg, sizeof(s_muon_pkg));
    tx.length = DATA_PACKAGE_SIZE;

    // 发送到TxQueue
    if (xQueueSend(TxQueue, &tx, pdMS_TO_TICKS(80)) != pdTRUE)
        ESP_LOGI(TAG, "Muon packet dropped (TxQueue full)");
    else
        ESP_LOGI(TAG, "Muon packet queued, PkgCnt=%u events=%u",
                 (unsigned)s_muon_pkg.PkgCnt, (unsigned)s_muon_event_index);

    // 写入Flash
    if (xQueueSend(FlashQueue, &tx, pdMS_TO_TICKS(80)) != pdTRUE)
        ESP_LOGI(TAG, "Muon packet dropped (FlashQueue full)");
    else
        ESP_LOGI(TAG, "Muon packet saved to flash, PkgCnt=%u events=%u",
                 (unsigned)s_muon_pkg.PkgCnt, (unsigned)s_muon_event_index);

    memset(&s_muon_pkg, 0, sizeof(s_muon_pkg));
    s_muon_pkg.head[0] = 0xAA;
    s_muon_pkg.head[1] = 0xBB;
    s_muon_pkg.head[2] = 0xCC;
    s_muon_pkg.PkgCnt = s_muon_pkg_cnt++;
    s_muon_event_index = 0;
}

esp_err_t InitDataPeripheral(void) {
    ESP_LOGI(TAG, "InitDataPeripheral: init ADC and timer");

    // Initialize new ADC for IO4, IO6, IO7
    esp_err_t err = adc_io6_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "adc_io6_init failed (continuing): %s", esp_err_to_name(err));
    }

    const esp_timer_create_args_t targs = {.callback = timeline_timer_cb,
                                           .name = "timeline_timer"};
    if (esp_timer_create(&targs, &s_timeline_timer) != ESP_OK) {
        ESP_LOGI(TAG, "Timeline timer create failed");
        return ESP_FAIL;
    }
    if (esp_timer_start_periodic(s_timeline_timer, TIMELINE_INTERVAL_US) != ESP_OK) {
        ESP_LOGI(TAG, "Timeline timer start failed");
        return ESP_FAIL;
    }

    memset(&s_muon_pkg, 0, sizeof(s_muon_pkg));
    s_muon_pkg.head[0] = 0xAA;
    s_muon_pkg.head[1] = 0xBB;
    s_muon_pkg.head[2] = 0xCC;
    s_muon_pkg.PkgCnt = s_muon_pkg_cnt++;

    g_running = false;

    ESP_LOGI(TAG, "InitDataPeripheral done");
    return ESP_OK;
}

void RunDataPeripheral(void *pvParameters) {
    ESP_LOGI(TAG, "RunDataPeripheral started");
    Command_t cmd;

    for (;;) {
        if (!xQueueReceive(DataQueue, &cmd, portMAX_DELAY))
            continue;

        uint8_t opcode = cmd.data[0];

        // START
        if (opcode == START) {
            s_current_pkg_id = ParsePkgIDFromCmd(&cmd);
            s_current_pkg_type = cmd.data[5];

            memset(&s_muon_pkg, 0, sizeof(s_muon_pkg));
            s_muon_pkg.head[0] = 0xAA;
            s_muon_pkg.head[1] = 0xBB;
            s_muon_pkg.head[2] = 0xCC;
            s_muon_pkg.PkgCnt = s_current_pkg_id;
            s_muon_event_index = 0;

            g_running = true;
            ESP_LOGI(TAG, "Start received. pkgID=%u pkgType=%u",
                     (unsigned)s_current_pkg_id, (unsigned)s_current_pkg_type);
            continue;
        }

        // STOP
        if (opcode == STOP) {
            g_running = false;
            ESP_LOGI(TAG, "Stop received. Flushing muon packet");
            flush_muon_pkg_if_any();
            continue;
        }

        // Trigger - raw ADC value from Preamplifier callback in data[1..2]
        if (opcode == OPCODE_TRIGGER) {
            if (!g_running)
                continue;

            uint64_t t_us = now_us();
            // Reconstruct raw from bytes packed by OnPreamplifierTrigger
            uint16_t raw_signal = ((uint16_t)cmd.data[1] << 8) | (uint16_t)cmd.data[2];

            ESP_LOGI(TAG, "Trigger received. raw=%u", (unsigned)raw_signal);

            if ((uint32_t)raw_signal >= SIPM_THRESHOLD_RAW) {
                MuonData_t ev;
                ev.cpu_time = t_us;
                ev.energy = adc_raw_to_energy(raw_signal);
                ev.pps = s_local_pps_count;

                ESP_LOGI(TAG, "Muon event: energy=%u pps=%u",
                         (unsigned)ev.energy, (unsigned)ev.pps);

                if (s_muon_event_index < MUON_PKG_MAX_EVENTS) {
                    s_muon_pkg.MuonData[s_muon_event_index++] = ev;
                    ESP_LOGI(TAG, "Event stored. index=%u",
                             (unsigned)s_muon_event_index);
                } else {
                    flush_muon_pkg_if_any();
                    s_muon_pkg.MuonData[s_muon_event_index++] = ev;
                    ESP_LOGI(TAG, "New muon pkg started. index=%u",
                             (unsigned)s_muon_event_index);
                }
            }
            continue;
        }

        // PPS
        if (opcode == OPCODE_PPS) {
            s_local_pps_count++;
            ESP_LOGI(TAG, "PPS received. count=%u",
                     (unsigned)s_local_pps_count);
            continue;
        }

        // TMP ALERT
        if (opcode == OPCODE_TMP_ALERT || opcode == STATUS) {
            ESP_LOGI(TAG, "Temp alert or status request. Sending timeline snapshot");
            timeline_send_now();
            continue;
        }

        // GPS
        if (opcode == OPCODE_GPS) {
            ESP_LOGI(TAG, "GPS data received");
            continue;
        }

        // ACK / NACK
        if (opcode == ACK) {
            uint32_t id = ParsePkgIDFromCmd(&cmd);
            ESP_LOGI(TAG, "ACK received for pkgID=%u", (unsigned)id);
            continue;
        }
        if (opcode == NACK) {
            uint32_t id = ParsePkgIDFromCmd(&cmd);
            ESP_LOGI(TAG, "NACK received for pkgID=%u (real data cannot retransmit)",
                     (unsigned)id);
            continue;
        }

        ESP_LOGI(TAG, "Unknown opcode 0x%02X", opcode);
    }
}
