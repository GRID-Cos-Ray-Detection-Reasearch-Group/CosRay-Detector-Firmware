#include "typedefs.h"
#include "config.h"

uint16_t CalcCRC(const uint8_t *data, size_t length) {
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < length; i++) {
		crc ^= (uint16_t) data[i] << 8;
		for (int j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

NullPkg_t CreateNullPkg() {
	NullPkg_t nullPkg;
	nullPkg.head[0] = 0xFF;
	nullPkg.head[1] = 0x00;
	nullPkg.head[2] = 0xFF;
	nullPkg.tail[0] = 0x00;
	nullPkg.tail[1] = 0xFF;
	nullPkg.tail[2] = 0x00;
	nullPkg.crc = 0x12F3; // 空包计算出的CRC值
	return nullPkg;
}