#ifndef RS485DEV_PUMP_HPP
#define RS485DEV_PUMP_HPP

#include "RS485Dev.hpp"
// 波特率 1200 2400 4800 9600 19200 38400
// 执行器 开关
template<int busID, int bufSize>
class Pump final : public RS485DevBase<busID, bufSize> {

#define DIVISION (30)
#define CHANNELS (1)

public:
    explicit Pump(uint32_t dev_addr, uint8_t reg_addr, uint32_t div = DIVISION)
    : RS485DevBase<busID, bufSize>(dev_addr, div)
    , dev_addr_(dev_addr)
    , reg_addr_(reg_addr) {
        assert(dev_addr > 0);
        assert(dev_addr < 65535);
        assert(reg_addr > 0);
        assert(reg_addr < 255);
    }

    // 设置占空比, 0-100
    void SetOn() {
        this->curr_cmd_ = 0x01;
    }

    void SetOff() {
        this->curr_cmd_ = 0x00;
    }


protected:
    uint32_t dev_addr_;
    uint8_t reg_addr_;
    uint8_t curr_cmd_ = 0x00;

    /// 地址码(1), 功能码(1), 寄存器地址(2), 读取数量(2), CRC16(2)
    void Require() override {
        this->rs485Agent.txbuf[0] = dev_addr_;                                  // 地址码
        this->rs485Agent.txbuf[1] = FunctionCode::WRITE;                        // 功能码 03 读 06 写
        this->rs485Agent.txbuf[2] = 0x00;                                       // 寄存器启始地址[高]
        this->rs485Agent.txbuf[3] = reg_addr_;                                  // 寄存器启始地址[低] =0x08

        this->rs485Agent.txbuf[4] = 0x00;                                       // 占空比[低]
        this->rs485Agent.txbuf[5] = curr_cmd_;                                  // 占空比[高]

        auto crc16 = CRC16Calc(this->rs485Agent.txbuf, 6);                      // CRC16
        this->rs485Agent.txbuf[6] = crc16 & 0x00FF;                             // CRC16[低]
        this->rs485Agent.txbuf[7] = (crc16 & 0xFF00) >> 8;                      // CRC16[高]// 设置发送数据
        this->rs485Agent.SendMsg(8);                                            // 发送数据
    }

    bool Check() override {
        // todo: 检查是否被正确修改，多设备共享同一设备地址可能导致接收消息在不同设备上混淆
        return true;
    }

    /// 响应帧与请求帧完全一致
    void Update() override {
        // todo:
        return;
    }
};

#endif // RS485DEV_BLOW_HPP
