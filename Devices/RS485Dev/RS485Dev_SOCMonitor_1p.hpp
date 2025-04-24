#ifndef RS485DEV_LISOC_1P_HPP
#define RS485DEV_LISOC_1P_HPP

#include "RS485Dev.hpp"

enum class SOCFunctionCode : uint8_t {
    READ = 0x04, // 读取多个寄存器
    WRITE = 0x06 // 修改单个寄存器
};

// 波特率 9600
// 建议命令周期 500ms
/**
 * @brief 电池荷电状态测量工具
 * @details 库伦积分法
 */
template<int busID, int bufSize>
class SOCMonitor_1p : public RS485DevBase<busID, bufSize> {

#define DIVISION (30)
#define CHANNELS (6)

public:
    explicit SOCMonitor_1p(uint32_t addr, uint32_t div = DIVISION)
    : RS485DevBase<busID, bufSize>(addr, div) {}

    [[nodiscard]] float GetPower() const {
        return this->curr_power_;
    }

    [[nodiscard]] float GetSOC() const {
        return this->curr_soc;
    }

protected:
    float curr_power_ = 0.0f;
    float curr_soc = 0.0f;
    float curr_temp_ = 0.0f;
    float curr_current_ = 0.0f;
    float curr_voltage_ = 0.0f;

    /// 地址码(1), 功能码(1), 寄存器地址(2), 读取数量(2), CRC16(2)
    void Require() override {
        this->rs485Agent.txbuf[0] = this->rs485Agent.addr;                                     // 地址码
        this->rs485Agent.txbuf[1] = SOCFunctionCode::READ;                                        // 功能码 03 读 06 写
        this->rs485Agent.txbuf[2] = 0x00;                                                      // 寄存器启始地址[高]
        this->rs485Agent.txbuf[3] = 0x00;                                                      // 寄存器启始地址[低] =0
        this->rs485Agent.txbuf[4] = 0x00;                                                      // 寄存器数量[高]
        this->rs485Agent.txbuf[5] = static_cast<uint8_t>(CHANNELS);                            // 寄存器数量[低] *8
        auto crc16 = CRC16Calc(this->rs485Agent.txbuf, 6);                       // CRC16
        this->rs485Agent.txbuf[6] = crc16 & 0x00FF;                                            // CRC16[低]
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
        if (this->rs485Agent.rxbuf[1] != SOCFunctionCode::READ) {
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
        // SOC
        int16_t soc_raw = (
            (this->rs485Agent.rxbuf[3 + 0 * 2]<<8) |\
            (this->rs485Agent.rxbuf[4 + 0 * 2]));

        // temp
        int16_t temp_raw = (
            (this->rs485Agent.rxbuf[3 + 1 * 2]<<8) |\
            (this->rs485Agent.rxbuf[4 + 1 * 2]));

        // current
        int16_t current_raw = (
            (this->rs485Agent.rxbuf[3 + 2 * 2]<<8) |\
            (this->rs485Agent.rxbuf[4 + 2 * 2]));

        // voltage
        int16_t voltage_raw = (
            (this->rs485Agent.rxbuf[3 + 3 * 2]<<8) |\
            (this->rs485Agent.rxbuf[4 + 3 * 2]));

        // power
        int16_t power_h_raw = (
            (this->rs485Agent.rxbuf[3 + 4 * 2]<<8) |\
            (this->rs485Agent.rxbuf[4 + 4 * 2]));

        int16_t power_l_raw = (
            (this->rs485Agent.rxbuf[3 + 5 * 2]<<8) |\
            (this->rs485Agent.rxbuf[4 + 5 * 2]));
        uint32_t power_raw = (static_cast<uint32_t>(power_h_raw) << 16) | (power_l_raw & 0xFFFF);

        this->curr_soc = static_cast<float>(soc_raw) / 10.0f;
        this->curr_power_ = static_cast<float>(power_raw) / 100.0f;
        this->curr_temp_ = static_cast<float>(temp_raw) / 10.0f;
        this->curr_current_ = static_cast<float>(current_raw) / 10.0f;
        this->curr_voltage_ = static_cast<float>(voltage_raw) / 10.0f;
    }
};

#endif //RS485DEV_LISOC_1P_HPP