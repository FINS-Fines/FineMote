#ifndef RS485DEV_ENV_MONITOR_HPP
#define RS485DEV_ENV_MONITOR_HPP

#include "RS485Dev_Monitor.hpp"
// 波特率 115200
// 传感器 vol*8
template<int busID, int bufSize>
class EnvMonitor : public RS485DevMonitor {

#define DIVISION (30)
#define CHANNELS (2) // 传感器数量

public:
    explicit EnvMonitor(
        uint32_t dev_addr,
        uint32_t div = DIVISION)
    : RS485_Agent<busID, bufSize>(dev_addr, div) {
        this->SetDivisionFactor(div);
    }

protected:
    RS485_Agent<busID, bufSize> rs485Agent;

    /// 地址码(1), 功能码(1), 寄存器地址(2), 读取数量(2), CRC16(2)
    void Require() override {
        this->rs485Agent.txbuf[0] = this->rs485Agent.addr;                      // 地址码
        this->rs485Agent.txbuf[1] = FunctionCode::READ;                         // 功能码 03 读 06 写
        this->rs485Agent.txbuf[2] = 0x00;                                       // 寄存器启始地址[高]
        this->rs485Agent.txbuf[3] = 0x0A;                                       // 寄存器启始地址[低] =0x0A
        this->rs485Agent.txbuf[4] = 0x00;                                       // 寄存器数量[高]
        this->rs485Agent.txbuf[5] = static_cast<uint8_t>(CHANNELS);             // 寄存器数量[低] *2 =0x0B
        auto crc16 = CRC16Calc(this->rs485Agent.txbuf, 6);        // CRC16
        this->rs485Agent.txbuf[6] = crc16 & 0x00FF;                             // CRC16[低]
        this->rs485Agent.txbuf[7] = (crc16 & 0xFF00) >> 8;                      // CRC16[高]// 设置发送数据
        this->rs485Agent.SendMsg(8);                                        // 发送数据
    }

    /// 地址码(1), 功能码(1), 长度(1), 数据(n), CRC16(2)
    /// @details 每个数据占用2个字节，高位在前，最高位为符号位
    bool Check() override {
        if (this->rs485Agent.rxbuf[0] != static_cast<uint8_t>(this->rs485Agent.addr)) {
            return false;
        }

        // 检查功能码
        if (this->rs485Agent.rxbuf[1] != FunctionCode::READ) {
            return false;
        }

        // 检查长度
        if (this->rs485Agent.rxbuf[2] != static_cast<uint8_t>(CHANNELS*2)) {
            return false;
        }

        // crc
        const uint16_t correct_crc = CRC16Calc(this->rs485Agent.rxbuf, CHANNELS*2 + 3);
        const uint16_t receive_crc = this->rs485Agent.rxbuf[CHANNELS*2 + 3] \
                                   | this->rs485Agent.rxbuf[CHANNELS*2 + 3 + 1] << 8;
        if (correct_crc != receive_crc) {
            return false;
        }

        return true;
    }

    /// 地址码(1), 功能码(1), 长度(1), 数据(n), CRC16(2)
    /// @details 每个数据占用2个字节，高位在前，最高位为符号位
    void Update() override {
        int16_t temp_raw = (
            (this->rs485Agent.rxbuf[3 + 0 * 2]<<8) |\
            (this->rs485Agent.rxbuf[4 + 0 * 2]));

        int16_t humi_raw = (
            (this->rs485Agent.rxbuf[3 + 1 * 2]<<8) |\
            (this->rs485Agent.rxbuf[4 + 1 * 2]));

        // 处理 <0 的数据
        if (temp_raw & 0x8000) {
            temp_raw = temp_raw - 65536;
        }

        assert(temp_raw >= -200);
        assert(temp_raw <= 800);
        assert(humi_raw >= 0);
        assert(humi_raw <= 1000);

        this->Temp_ = temp_raw / 10.0f;
        this->Humidity_ = humi_raw / 10.0f;
    }
};

#endif // RS485DEV_ENV_MONITOR_HPP
