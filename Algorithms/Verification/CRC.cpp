/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "CRC.h"

uint8_t CRC8Calc(uint8_t *data, uint16_t length) {
    static etl::crc8_ccitt crc8;

    crc8.reset();
    for (uint16_t i = 0; i < length; i++) {
        crc8.add(data[i]);
    }
    return crc8.value();
}

uint16_t CRC16Calc(uint8_t *data, uint16_t length) {
    etl::crc16_modbus crc16;

    for (uint16_t i = 0; i < length; i++) {
        crc16.add(data[i]);
    }
    return crc16.value();
}

uint32_t CRC32Calc(uint8_t *data, uint16_t length) {
    static etl::crc32 crc32;

    crc32.reset();
    for (uint16_t i = 0; i < length; i++) {
        crc32.add(data[i]);
    }
    return crc32.value();
}
