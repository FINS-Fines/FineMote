#ifndef RS485DEV_TEMPMONITOR_8P_HPP
#define RS485DEV_TEMPMONITOR_8P_HPP

#include "RS485Dev_Monitor.hpp"
// 波特率 115200
/**
 * @brief 面阵温度测量工具
 * @details ntc 10k B3950 *8
 */
template<uint8_t busID>
class TempMonitor_8p : public RS485DevMonitor {

#define TEMP_MONITOR_DIVISION (50)
#define TEMP_MONITOR_CHANNELS (8)

public:
    explicit TempMonitor_8p(
        uint32_t dev_addr,
        uint32_t div = TEMP_MONITOR_DIVISION)
    : rs485Agent(dev_addr) {
        this->SetDivisionFactor(div);
    }

protected:
    RS485_Agent<busID> rs485Agent;

public:
    // std::array<float, MONITOR_ARR_SIZE> temps_{};

protected:
    /// 地址码(1), 功能码(1), 寄存器地址(2), 读取数量(2), CRC16(2)
    void Require() override {
        this->rs485Agent.txbuf[0] = this->rs485Agent.addr;                                     // 地址码
        this->rs485Agent.txbuf[1] = 0x03;                                                      // 功能码 03 读 06 写
        this->rs485Agent.txbuf[2] = 0x00;                                                      // 寄存器启始地址[高]
        this->rs485Agent.txbuf[3] = 0x00;                                                      // 寄存器启始地址[低] =0
        this->rs485Agent.txbuf[4] = 0x00;                                                      // 寄存器数量[高]
        this->rs485Agent.txbuf[5] = static_cast<uint8_t>(TEMP_MONITOR_CHANNELS);                            // 寄存器数量[低] *8
        auto crc16 = CRC16Calc(this->rs485Agent.txbuf, 6);                       // CRC16
        this->rs485Agent.txbuf[6] = (crc16 & 0x00FF);                                          // CRC16[低]
        this->rs485Agent.txbuf[7] = (crc16 & 0xFF00) >> 8;                                     // CRC16[高]// 设置发送数据
        this->rs485Agent.SendMsg(8);                                                       // 发送数据
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
        if (this->rs485Agent.rxbuf[2] != static_cast<uint8_t>(TEMP_MONITOR_CHANNELS*2)) {
            return false;
        }

        // crc
        const uint16_t correct_crc = CRC16Calc(this->rs485Agent.rxbuf, TEMP_MONITOR_CHANNELS*2 + 3);
        const uint16_t receive_crc = this->rs485Agent.rxbuf[TEMP_MONITOR_CHANNELS*2 + 3] \
                                   | this->rs485Agent.rxbuf[TEMP_MONITOR_CHANNELS*2 + 3 + 1] << 8;
        if (correct_crc != receive_crc) {
            return false;
        }

        return true;
    }

    /// 地址码(1), 功能码(1), 长度(1), 数据(n), CRC16(2)
    /// @details 每个数据占用2个字节，高位在前，最高位为符号位
    void Update() override {
        for (int i = 0; i < TEMP_MONITOR_CHANNELS; i++) {
            int16_t temp_raw = (
                (this->rs485Agent.rxbuf[3 + i * 2]<<8) |\
                (this->rs485Agent.rxbuf[4 + i * 2]));

           if (temp_raw & 0x8000) {
                temp_raw = temp_raw - 65536;
            }

            this->values_[i] = static_cast<float>(temp_raw) / 10.0f;

            // debug
            // this->temps_[i] = this->values_[i];
            // this->values_[7] = 50;
        }
    }
};

#endif //RS485DEV_TEMPMONITOR_8P_HPP