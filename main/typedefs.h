#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include "config.h"
#include <stdio.h>

uint16_t CalcCRC(const uint8_t *data, size_t length);

// μ子事件数据
#pragma pack(push, 1)
typedef struct {
	uint64_t cpu_time; // CPU时钟（8字节，lifetime counter）
	uint16_t energy;   // μ子能量，2字节（16位ADC测量值）
	uint32_t pps;	   // 本次上电以来的PPS脉冲计数，4字节
} MuonData_t;		   // 每一个μ子事件总共14字节
#pragma pack(pop)

// μ子数据包（最多35个事件）
#pragma pack(push, 1)
typedef struct {
	uint8_t head[3];		 // 0xAA, 0xBB, 0xCC for MuonPackage
	uint32_t PkgCnt;		 // 全局数据包计数，掉电不丢失
	uint32_t utc;			 // 当前包第一个计数写入时的utc时间
	MuonData_t MuonData[35]; // μ子事件，每个数据包最多能填充35个有效值
	uint8_t tail[3];		 // 0xDD, 0xEE, 0xFF for MuonPackage
	uint8_t reserved[6];
	uint16_t crc;
} MuonDataPkg_t;
#pragma pack(pop)

// 时间线辅助数据（每5秒生成一次）
#pragma pack(push, 1)
typedef struct {
	uint64_t cpu_time;	  // 写入当前数据时cpu时钟
	uint32_t pps;		  // 当前PPS脉冲计数，4字节
	uint32_t utc;		  // 最近一次收到的utc时间戳，4字节
	uint32_t pps_utc;	  // 上次记录utc时间时的pps脉冲计数
	uint64_t cputime_pps; // 上次收到pps脉冲时的cpu时钟
	int32_t gps_long;	  // GPS经度（1e-7度单位，有符号，-180°~+180°）
	int32_t gps_lat;	  // GPS纬度（1e-7度单位，有符号，-90°~90°）
	int16_t gps_alt;	  // GPS海拔（米）
	int8_t acc_x;		  // 加速度x值
	int8_t acc_y;		  // 加速度y值
	int8_t acc_z;		  // 加速度z值
	uint16_t SiPMTmp;	  // SiPM附近温度（TMP112）
	uint8_t MCUTmp;		  // ESP32内置温度传感器
	uint16_t SiPMImon;	  // SiPM漏电流监测值
	uint16_t SiPMVmon;	  // SiPM偏压监测值
} TimeLineData_t;		  // 总计48字节
#pragma pack(pop)

// 时间线数据包（最多10个时间线条目）
#pragma pack(push, 1)
typedef struct {
	uint8_t head[3];		  // 0x12, 0x34, 0x56 for timeline package
	uint32_t PkgCnt;		  // 全局timeline数据包计数
	TimeLineData_t TimeLineData[10];
	uint8_t tail[3];		  // 0x78, 0x9A, 0xBC for timeline package
	uint8_t reserved[20];
	uint16_t crc;
} TimeLinePkg_t;
#pragma pack(pop)

// 蓝牙命令结构体
typedef struct {
	uint8_t data[CMD_LENGTH];
} Command_t;

typedef enum {
	START = 0x01,
	STOP = 0x02,
	ACK = 0x03,
	NACK = 0x04,
	STATUS = 0x05,
	PING = 0x06
} CommandOpcode_t;

/*
命令列表
- 0x01 : START 开始传输
    [1..4] : 数据包的 ID，大端存储
    [5] : 传输的数据类型
        0x01 : μ子数据包
        0x02 : 时间线数据包
- 0x02 : STOP 停止传输
- 0x03 : ACK 确认收到数据包
    [1..4] : 已成功收到的数据包的 ID，大端存储
    [5] : 数据类型
- 0x04 : NACK 请求重传数据包
    [1..4] : 需要重传的数据包的 ID，大端存储
    [5] : 数据类型
- 0x05 : STATUS 请求状态信息
- 0x06 : PING 测试连接
*/

// 带CRC的命令包（蓝牙传输用）
#pragma pack(push, 1)
typedef struct {
	Command_t cmd;
	uint16_t crc;
} CommandPkg_t;
#pragma pack(pop)

// 蓝牙/Flash传输数据包
#pragma pack(push, 1)
typedef struct {
	uint8_t data[DATA_PACKAGE_SIZE];
	size_t length;
} TxPkg_t;
#pragma pack(pop)

// 命令帧头部（备用）
#pragma pack(push, 1)
typedef struct {
	uint16_t header;   // 帧头 (固定为 0xAA55)
	uint8_t cmd_type;  // 指令类型
	uint8_t param_len; // 参数长度
	uint8_t params[];  // 可变长度参数
} CommandFrameHeader_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	CommandFrameHeader_t header;
	uint16_t checksum;
	uint16_t trailer; // 帧尾 (固定为 0x55AA)
} CommandFrame_t;
#pragma pack(pop)

#endif // TYPEDEFS_H
