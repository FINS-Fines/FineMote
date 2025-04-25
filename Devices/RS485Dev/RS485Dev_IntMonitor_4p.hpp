#ifndef RS485DEV_IntMonitor_4P_HPP
#define RS485DEV_IntMonitor_4P_HPP

#include "RS485Dev_Monitor.hpp"
// 波特率 115200
// 传感器 Int*4 0-20mV 精度0.10%
template<int busID>
class IntMonitor_4p : public RS485DevMonitor {

#define INT_MONITOR_DIVISION (30)
#define INT_MONITOR_CHANNELS (4)

public:
    explicit IntMonitor_4p(
        uint32_t addr,
        uint32_t div = INT_MONITOR_DIVISION)
    : rs485Agent(addr) {
        this->SetDivisionFactor(div);
    };
    
protected:
    RS485_Agent<busID> rs485Agent;

public:
    std::array<float, INT_MONITOR_CHANNELS> Ints_{};

protected:
    /// 地址码(1), 功能码(1), 寄存器地址(2), 读取数量(2), CRC16(2)
    void Require() override {
        this->rs485Agent.txbuf[0] = this->rs485Agent.addr;                      // 地址码
        this->rs485Agent.txbuf[1] = 0x03;                                       // 功能码 03 读 06 写
        this->rs485Agent.txbuf[2] = 0x00;                                       // 寄存器启始地址[高]
        this->rs485Agent.txbuf[3] = 0x0C;                                       // 寄存器启始地址[低] =0x08
        this->rs485Agent.txbuf[4] = 0x00;                                       // 寄存器数量[高]
        this->rs485Agent.txbuf[5] = static_cast<uint8_t>(INT_MONITOR_CHANNELS); // 寄存器数量[低] *4 =0x0B
        auto crc16 = CRC16Calc(this->rs485Agent.txbuf, 6);                      // CRC16
        this->rs485Agent.txbuf[6] = crc16 & 0x00FF;                             // CRC16[低]
        this->rs485Agent.txbuf[7] = (crc16 & 0xFF00) >> 8;                      // CRC16[高]// 设置发送数据
        this->rs485Agent.SendMsg(8);                                            // 发送数据
    }

    /// 地址码(1), 功能码(1), 长度(1), 数据(n), CRC16(2)
    /// @details 每个数据占用2个字节，高位在前，最高位为符号位
    bool Check() override {
        if (this->rs485Agent.rxbuf[0] != static_cast<uint8_t>(this->rs485Agent.addr)) {
            return false;
        }

        // 检查功能码
        if (this->rs485Agent.rxbuf[1] != 0x03) {
            return false;
        }

        // 检查长度
        if (this->rs485Agent.rxbuf[2] != static_cast<uint8_t>(INT_MONITOR_CHANNELS*2)) {
            return false;
        }

        // crc
        const uint16_t correct_crc = CRC16Calc(this->rs485Agent.rxbuf, INT_MONITOR_CHANNELS*2 + 3);
        const uint16_t receive_crc = this->rs485Agent.rxbuf[INT_MONITOR_CHANNELS*2 + 3] \
                                   | this->rs485Agent.rxbuf[INT_MONITOR_CHANNELS*2 + 3 + 1] << 8;
        if (correct_crc != receive_crc) {
            return false;
        }

        return true;
    }

    /// 地址码(1), 功能码(1), 长度(1), 数据(n), CRC16(2)
    /// @details 每个数据占用2个字节，高位在前，最高位为符号位
    void Update() override {
        for (int i = 0; i < INT_MONITOR_CHANNELS; i++) {
            int16_t vol_raw = (
                (this->rs485Agent.rxbuf[3 + i * 2]<<8) |\
                (this->rs485Agent.rxbuf[4 + i * 2]));

            this->Ints_[i] = vol_raw / 1000.0f;
        }
    }
};

#endif // RS485DEV_IntMonitor_4P_HPP
