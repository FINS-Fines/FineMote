/*******************************************************************************
* Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/
/*******************************************************************************
* Copyright (c) 2025.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/
#ifndef MPT_45H_H
#define MPT_45H_H

#include "ProjectConfig.h"
#include "DeviceBase.h"
#include "cmath"
#include "EncoderBase.hpp"
#include "RS485_Base.hpp"

volatile uint32_t callback_count = 0;  // 回调计数器

// 辅助函数
int bitCount(uint8_t value) {
    uint8_t count = 0;
    while (value) {
        count += value & 1;
        value >>= 1;
    }
    return count;
}

// CRC(x^8 + 1), 异或校验
uint8_t calcCRC(const uint8_t * buffer, uint8_t length){
    uint8_t temp = *buffer++;
    while(--length){
        temp = *buffer++ ^ temp;
    }
    return temp;
}


// MPT-45H 编码器类
template<uint8_t busID>
class MPT_45H : public EncoderBase {
public:

    /**
     * @brief 解码函数，作为RS485_Agent的回调
     * @param data 接收到的数据
     * @param size 数据长度
    */
    void Decode(const uint8_t* data, size_t size) {
        uint32_t angle_raw = data[4] << 16 | data[3] << 8 | data[2];
        position =  (float)angle_raw / (float)(1 << 24) * 360.0f;
        getPos = true;
        crc_value = data[5];
        data_byte4 = data[3];

        // if(calcCRC(data, 5) == data[5] && _id == data[0]) {
        //     uint32_t angle_raw = data[4] << 16 | data[3] << 8 | data[2];
        //     position = (float)angle_raw / (float)(1 << 24) * 360.0f;
        //     getPos = true;
        // }
    }

    explicit MPT_45H(uint8_t id) :
        rs485Agent(calculateRequestAddress(id), [this](uint8_t* data, size_t size) {
            Decode(data, size);
        })
    {
        SetDivisionFactor(300);
        _id = calculateRequestAddress(id);
        _txbuf = calculateRequestAddress(id);
    }

    void Handle() final {
        messageGenerate();
    }

private:
    RS485_Agent<busID> rs485Agent;

    uint8_t _id;
    uint8_t _txbuf{0};

    void messageGenerate() {
        rs485Agent.Transmit(&_txbuf, 1);
    }

    // 计算请求地址
    static uint8_t calculateRequestAddress(uint8_t id) {
        uint8_t requestAddress = id & 0x7F;
        uint8_t bit_count = bitCount(requestAddress);

        // 计算奇校验位
        if (bit_count % 2 == 1) {
            requestAddress &= ~0x80;
        } else {
            requestAddress |= 0x80;
        }
        return requestAddress;
    }
};
#endif //MPT_45H_H
