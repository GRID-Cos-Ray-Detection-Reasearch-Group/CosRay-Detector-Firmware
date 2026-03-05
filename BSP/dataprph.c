#include "config.h"
#include "typedefs.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "gps_module.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static const char *TAG = "DataPeripheral";

#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_WIDTH_BITS ADC_WIDTH_BIT_12

static const adc_channel_t ADC_CHANNEL_SIGNAL = ADC_CHANNEL_3;		// IO4
static const adc_channel_t ADC_CHANNEL_CATHODE_MON = ADC_CHANNEL_5; // IO6
static const adc_channel_t ADC_CHANNEL_MON = ADC_CHANNEL_6;			// IO7

// μ子信号判断阈值（可在 sdkconfig 或 CMake 中覆盖）
#ifndef SIPM_THRESHOLD_RAW
#define SIPM_THRESHOLD_RAW 2000U
#endif

// 时间线数据包生成周期（5秒）
#define TIMELINE_INTERVAL_US (5ULL * 1000000ULL)

// 每个μ子数据包最多存储的事件数
#ifndef MUON_PKG_MAX_EVENTS
#define MUON_PKG_MAX_EVENTS 35
#endif

// 内部操作码（与 main.c 中保持一致）
#define OPCODE_TRIGGER 0xA0
#define OPCODE_PPS 0xA1
#define OPCODE_TMP_ALERT 0xA2
#define OPCODE_GPS 0xA3

// 外部队列
extern QueueHandle_t DataQueue;
extern QueueHandle_t TxQueue;
extern QueueHandle_t FlashQueue;

extern uint16_t CalcCRC(const uint8_t *data, size_t length);

// 模块内部状态
static esp_adc_cal_characteristics_t s_adc_chars;
static esp_timer_handle_t s_timeline_timer = NULL;

static MuonDataPkg_t s_muon_pkg;
static uint8_t s_muon_event_index = 0;
static uint32_t s_muon_pkg_cnt = 0;

static uint32_t s_local_pps_count = 0;
static int32_t s_local_gps_long = 0;
static int32_t s_local_gps_lat = 0;
static uint32_t s_local_gps_utc = 0;
static uint16_t s_local_SiPM_temp = 0;
static uint8_t s_local_MCU_temp = 0;

static bool g_running = false;
static uint32_t s_current_pkg_id = 0;
static uint8_t s_current_pkg_type = 0;

// 获取当前 esp_timer 时间（微秒）
static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

// ADC 原始值转换为 16 位能量值
static inline uint16_t adc_raw_to_energy(uint32_t raw) {
	if (raw > 4095U)
		raw = 4095U;
	return (uint16_t)((raw * 65535UL) / 4095UL);
}

// 从命令数据中解析数据包 ID（大端存储）
static inline uint32_t ParsePkgIDFromCmd(const Command_t *cmd) {
	return ((uint32_t)cmd->data[1] << 24) | ((uint32_t)cmd->data[2] << 16) |
		   ((uint32_t)cmd->data[3] << 8) | ((uint32_t)cmd->data[4]);
}

// 从 GPS 模块读取最新位置和时间信息，更新本地缓存
static void UpdateLocalGPSData(void) {
	if (gps_hex_data_mutex == NULL)
		return;

	if (xSemaphoreTake(gps_hex_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
		// 经纬度：UBX NAV-PVT 格式为小端 int32_t，单位为度 * 1e-7
		int32_t lat;
		int32_t lon;
		memcpy(&lat, current_gps_hex_data.latitude, sizeof(int32_t));
		memcpy(&lon, current_gps_hex_data.longitude, sizeof(int32_t));
		s_local_gps_lat = lat;
		s_local_gps_long = lon;

		// UTC 时间编码为当天秒数
		s_local_gps_utc = (uint32_t)current_gps_hex_data.hour * 3600U +
						  (uint32_t)current_gps_hex_data.minute * 60U +
						  (uint32_t)current_gps_hex_data.second;

		xSemaphoreGive(gps_hex_data_mutex);
	}
}

// 生成并发送时间线数据包（通过 BLE 和 Flash）
static void timeline_send_now(void) {
	// 更新 GPS 数据
	UpdateLocalGPSData();

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
	td->gps_long = s_local_gps_long;
	td->gps_lat = s_local_gps_lat;
	td->gps_alt = 0;

	td->acc_x = 0;
	td->acc_y = 0;
	td->acc_z = 0;

	int raw_vmon = adc1_get_raw(ADC_CHANNEL_CATHODE_MON);
	int raw_imon = adc1_get_raw(ADC_CHANNEL_MON);

	// 12位ADC返回0-4095，向uint16_t类型的监测字段写入时截断到合法范围
	td->SiPMVmon = (raw_vmon < 0 ? 0 : (raw_vmon > 4095 ? 4095 : raw_vmon));
	td->SiPMImon = (raw_imon < 0 ? 0 : (raw_imon > 4095 ? 4095 : raw_imon));

	td->SiPMTmp = s_local_SiPM_temp;
	td->MCUTmp = s_local_MCU_temp;

	pkg.tail[0] = 0x78;
	pkg.tail[1] = 0x9A;
	pkg.tail[2] = 0xBC;

	pkg.crc = CalcCRC((uint8_t *)&pkg, sizeof(TimeLinePkg_t) - 2);

	TxPkg_t tx;
	memset(&tx, 0, sizeof(tx));
	memcpy(tx.data, &pkg, sizeof(pkg));
	tx.length = DATA_PACKAGE_SIZE;

	// 发送到蓝牙发送队列
	if (xQueueSend(TxQueue, &tx, pdMS_TO_TICKS(50)) != pdTRUE)
		ESP_LOGW(TAG, "Timeline packet dropped (TxQueue full)");
	else
		ESP_LOGI(TAG, "Timeline packet queued, PkgCnt=%u",
				 (unsigned)pkg.PkgCnt);

	// 写入 Flash 存储队列
	if (xQueueSend(FlashQueue, &tx, pdMS_TO_TICKS(50)) != pdTRUE)
		ESP_LOGW(TAG, "Timeline packet dropped (FlashQueue full)");
	else
		ESP_LOGI(TAG, "Timeline packet saved to flash, PkgCnt=%u",
				 (unsigned)pkg.PkgCnt);
}

// 时间线定时器回调
static void timeline_timer_cb(void *arg) {
	(void)arg;
	timeline_send_now();
}

// 将当前 μ子数据包发送到 BLE 和 Flash，然后重置缓冲区
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

	if (xQueueSend(TxQueue, &tx, pdMS_TO_TICKS(80)) != pdTRUE)
		ESP_LOGW(TAG, "Muon packet dropped (TxQueue full)");
	else
		ESP_LOGI(TAG, "Muon packet queued, PkgCnt=%u events=%u",
				 (unsigned)s_muon_pkg.PkgCnt, (unsigned)s_muon_event_index);

	if (xQueueSend(FlashQueue, &tx, pdMS_TO_TICKS(80)) != pdTRUE)
		ESP_LOGW(TAG, "Muon packet dropped (FlashQueue full)");
	else
		ESP_LOGI(TAG, "Muon packet saved to flash, PkgCnt=%u events=%u",
				 (unsigned)s_muon_pkg.PkgCnt, (unsigned)s_muon_event_index);

	// 初始化新的 μ子数据包
	memset(&s_muon_pkg, 0, sizeof(s_muon_pkg));
	s_muon_pkg.head[0] = 0xAA;
	s_muon_pkg.head[1] = 0xBB;
	s_muon_pkg.head[2] = 0xCC;
	s_muon_pkg.PkgCnt = s_muon_pkg_cnt++;
	s_muon_event_index = 0;
}

// 初始化 ADC 和定时器
esp_err_t InitDataPeripheral(void) {
	ESP_LOGI(TAG, "InitDataPeripheral: init ADC and timer");

	adc1_config_width(ADC_WIDTH_BITS);
	adc1_config_channel_atten(ADC_CHANNEL_SIGNAL, ADC_ATTEN);
	adc1_config_channel_atten(ADC_CHANNEL_CATHODE_MON, ADC_ATTEN);
	adc1_config_channel_atten(ADC_CHANNEL_MON, ADC_ATTEN);

	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BITS, 1100,
							 &s_adc_chars);

	// 创建周期性时间线定时器
	const esp_timer_create_args_t targs = {.callback = timeline_timer_cb,
										   .name = "timeline_timer"};
	if (esp_timer_create(&targs, &s_timeline_timer) != ESP_OK) {
		ESP_LOGE(TAG, "Timeline timer create failed");
		return ESP_FAIL;
	}
	if (esp_timer_start_periodic(s_timeline_timer, TIMELINE_INTERVAL_US) !=
		ESP_OK) {
		ESP_LOGE(TAG, "Timeline timer start failed");
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

// 数据处理主循环（运行于独立 FreeRTOS 任务中）
void RunDataPeripheral(void) {
	ESP_LOGI(TAG, "RunDataPeripheral started");
	Command_t cmd;

	for (;;) {
		if (!xQueueReceive(DataQueue, &cmd, portMAX_DELAY))
			continue;

		uint8_t opcode = cmd.data[0];

		// START：开始采集
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

		// STOP：停止采集，刷新剩余数据包
		if (opcode == STOP) {
			g_running = false;
			ESP_LOGI(TAG, "Stop received. Flushing muon packet");
			flush_muon_pkg_if_any();
			continue;
		}

		// TRIGGER：μ子事件（来自比较器中断）
		if (opcode == OPCODE_TRIGGER) {
			if (!g_running)
				continue;

			uint64_t t_us = now_us();
			int raw_signal = adc1_get_raw(ADC_CHANNEL_SIGNAL);
			int raw_cath = adc1_get_raw(ADC_CHANNEL_CATHODE_MON);
			int raw_mon = adc1_get_raw(ADC_CHANNEL_MON);

			ESP_LOGD(TAG, "Trigger received. sig=%d cath=%d mon=%d", raw_signal,
					 raw_cath, raw_mon);

			if ((uint32_t)raw_signal >= SIPM_THRESHOLD_RAW) {
				MuonData_t ev;
				ev.cpu_time = t_us;
				ev.energy = adc_raw_to_energy(raw_signal);
				ev.pps = s_local_pps_count;

				ESP_LOGI(TAG, "Muon event: energy=%u pps=%u",
						 (unsigned)ev.energy, (unsigned)ev.pps);

				if (s_muon_event_index < MUON_PKG_MAX_EVENTS) {
					s_muon_pkg.MuonData[s_muon_event_index++] = ev;
				} else {
					flush_muon_pkg_if_any();
					s_muon_pkg.MuonData[s_muon_event_index++] = ev;
				}
			}
			continue;
		}

		// PPS：GPS秒脉冲，更新本地 PPS 计数
		if (opcode == OPCODE_PPS) {
			s_local_pps_count++;
			ESP_LOGD(TAG, "PPS received. count=%u",
					 (unsigned)s_local_pps_count);
			continue;
		}

		// GPS：新 GPS 数据到达，更新本地缓存
		if (opcode == OPCODE_GPS) {
			UpdateLocalGPSData();
			ESP_LOGD(TAG, "GPS data updated: lat=%d lon=%u utc=%u",
					 s_local_gps_lat, s_local_gps_long, s_local_gps_utc);
			continue;
		}

		// TMP ALERT / STATUS：生成即时时间线快照
		if (opcode == OPCODE_TMP_ALERT || opcode == STATUS) {
			ESP_LOGI(TAG, "Temp alert or status request");
			timeline_send_now();
			continue;
		}

		// ACK / NACK：处理数据传输确认
		if (opcode == ACK) {
			uint32_t id = ParsePkgIDFromCmd(&cmd);
			ESP_LOGI(TAG, "ACK received for pkgID=%u", (unsigned)id);
			continue;
		}
		if (opcode == NACK) {
			uint32_t id = ParsePkgIDFromCmd(&cmd);
			ESP_LOGI(TAG, "NACK received for pkgID=%u", (unsigned)id);
			continue;
		}

		ESP_LOGW(TAG, "Unknown opcode 0x%02X", opcode);
	}
}
