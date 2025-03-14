/*******************************************************************************
* Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/


#ifndef DS18B20_485_H
#define DS18B20_485_H

#include "ProjectConfig.h"
#include "DeviceBase.h"
#include "Bus/RS485_Base.h"
#include "cmath"

/**
 * @brief 8 路 ntc 10k B3950 温度传感器
 */
// 寄存器地址 0x00-0x07
#define DS18B20_485_ADDR (0x00)
#define DS18B20_485_NUM (0x08)
// 读写模式 0x03 读 0x06 写
#define DS18B20_485_READ (0x03)
#define DS18B20_485_WRITE (0x06)
// 分频系数
#define DS18B20_485_DIVISION_FACTOR (300)

template<int busID>
class DS18B20_485 : DeviceBase {
public:
    explicit DS18B20_485(uint32_t addr) : rs485Agent(addr) {
        this->SetDivisionFactor(DS18B20_485_DIVISION_FACTOR);
    }

    std::array<float, DS18B20_485_NUM> GetTemperature() const {
        return temperatures;
    }

private:
    std::array<float, DS18B20_485_NUM> temperatures;
    uint8_t addrPosition{0};
    RS485_Agent<busID> rs485Agent;


    /// 地址码(1), 功能码(1), 寄存器地址(2), 读取数量(2), CRC16(2)
    void Require() {
        rs485Agent.txbuf[0] = rs485Agent.addr;                      // 地址码
        rs485Agent.txbuf[1] = 0x03;                     // 功能码 03 读 06 写
        rs485Agent.txbuf[2] = 0x00;                                 // 寄存器启始地址[高]
        rs485Agent.txbuf[3] = 0x00;                     // 寄存器启始地址[低] =0
        rs485Agent.txbuf[4] = 0x00;                                 // 寄存器数量[高]
        rs485Agent.txbuf[5] = 0x08;                      // 寄存器数量[低] *8
        uint16_t crc16 = CRC16Calc(rs485Agent.txbuf, 6);            // CRC16
        rs485Agent.txbuf[6] = crc16 & 0x00FF;                         // CRC16[低]
        rs485Agent.txbuf[7] = (crc16 & 0xFF00) >> 8;                  // CRC16[高]
        rs485Agent.SendMsg(8);                                  // 发送
    }

    /// 地址码(1), 功能码(1), 长度(1), 数据(n), CRC16(2)
    /// @details 每个数据占用2个字节，高位在前，最高位为符号位
    void Update() {
        if (rs485Agent.rxbuf[1] != DS18B20_485_READ) {
            return;
        }
        int data_length = rs485Agent.rxbuf[2];
        int else_length = 3;// 地址码(1), 功能码(1), 长度(1)
        uint16_t crc16 = CRC16Calc(rs485Agent.rxbuf, data_length + else_length);
        uint16_t received_crc = (uint16_t)rs485Agent.rxbuf[data_length + else_length] |
                                ((uint16_t)rs485Agent.rxbuf[data_length + else_length + 1] << 8);
        if (crc16 != received_crc) {
            return;
        }

        for (int i = 0; i < 8; i++) {
            int16_t temp_raw = (
                (rs485Agent.rxbuf[3 + i * 2]<<8) |\
                (rs485Agent.rxbuf[4 + i * 2]));

            if (temp_raw & 0x8000) {
                temp_raw = temp_raw - 65536;
            }

            temperatures[i] = temp_raw / 10.0f;
        }

    }

    void Handle() {
        Require();
        Update();
    }
};

#endif //DS18B20_485_H
