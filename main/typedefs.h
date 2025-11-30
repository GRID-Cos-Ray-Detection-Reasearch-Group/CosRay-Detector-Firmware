#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <stdio.h>
#include "config.h"

uint16_t CalcCRC(const uint8_t *data, size_t length);

// 谬子数据包
#pragma pack(push, 1)
typedef struct {
	uint64_t cpu_time; // CPU时钟（8字节，lifetime counter）
	uint16_t energy;   // μ子能量，2字节（16位ADC测量值）
	uint32_t pps;	   // 本次上电以来的PPS脉冲计数，4字节
} MuonData_t;		   // 每一个μ子事件总共14字节
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	uint8_t head[3];		 // 0xAA, 0xBB, 0xCC for MuonPackage
	uint32_t PkgCnt;		 // 全局数据包计数，掉电不丢失
	uint32_t utc;			 // 当前包第一个计数写入时的utc时间
	MuonData_t MuonData[35]; // μ子事件，每个数据包最多能填充35个有效值
	// 根据μ子计数率估算大约20s写满一个包
	uint8_t tail[3]; // 0xDD, 0xEE, 0xFF for MuonPackage
	uint8_t reserved[6];
	uint16_t crc;
} MuonDataPkg_t; // 平均每秒产生大约25.6B数据
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	uint64_t cpu_time;	  // 写入当前数据时cpu时钟（8字节，lifetime counter）
	uint32_t pps;		  // 当前PPS脉冲计数，4字节
	uint32_t utc;		  // 最近一次收到的utc时间戳，4字节
	uint32_t pps_utc;	  // 上次记录utc时间时的pps脉冲计数
	uint64_t cputime_pps; // 上次收到pps脉冲时的cpu时钟
	uint32_t gps_long;	  // 1m级精度，4字节存储gps经度，无正负，从0°到360°
	int32_t gps_lat;	  // 1m级精度，4字节存储gps纬度，有正负，从-90°到90°
	// 经纬度存储通过自行设计换算方法是可以优化的，基本思想是让2^32的范围尽量都有效
	int16_t gps_alt;   // 1m级精度，2字节，存储gps海拔
	int8_t acc_x;	   // 加速度x值，1字节
	int8_t acc_y;	   // 加速度y值，1字节
	int8_t acc_z;	   // 加速度z值，1字节
	uint16_t SiPMTmp;  // 当前tmp112记录的SiPM附近温度，2字节
	uint8_t MCUTmp;	   // ESP32内置温度传感器记录的温度值
	uint16_t SiPMImon; // SiPM漏电流监测值
	uint16_t SiPMVmon; // SiPM偏压监测值
} TimeLineData_t;	   // 总计48字节，每5秒钟生成一次
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	uint8_t head[3]; // 0x12, 0x34, 0x56 for timeline package
	uint32_t PkgCnt; // 该μ子探测器全局的timeline数据包计数，从0开始，掉电不丢失
	TimeLineData_t
		TimeLineData[10]; // 每5秒生成一次有效timeline数据，每个包最多填充10个
	uint8_t tail[3];	  // 0x78 0x9A 0xBC for timeline package
	uint8_t reserved[20];
	uint16_t crc;
} TimeLinePkg_t; // 平均每秒产生10.24B数据
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	uint8_t head[3]; // 0xFF, 0x00, 0xFF for Null Package
	uint8_t tail[3]; // 0x00, 0xFF, 0x00 for Null Package
	uint16_t crc;
} NullPkg_t;	   // 用于BLE传输时的占位空包，不用于存储
#pragma pack(pop)

NullPkg_t CreateNullPkg();

typedef struct {
	uint8_t data[CMD_LENGTH];
} Command_t;
/*
data[0]: 命令类型
data[0]==0x01: 发送数据包
	data[1]: 数据包类型
		0x01: 谬子数据包
		0x02: 时间线数据包
	data[2]-data[5]: 数据包计数（uint32_t，大端序）
	data[6]-data[7]: 预留
其余预留
*/

#pragma pack(push, 1)
typedef struct {
	Command_t cmd;
	uint16_t crc;
} CommandPkg_t;
#pragma pack(pop)

/*
example command pkg for sending muon data package with PkgCnt=114514:
data: [0x01, 0x01, 0x00, 0x01, 0xBF, 0x52, 0x00, 0x00]
crc: 0xA255
0x 01 01 00 01 BF 52 00 00 55 A2
*/

#endif // TYPEDEFS_H