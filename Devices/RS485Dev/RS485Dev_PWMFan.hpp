#ifndef RS485DEV_FAN_HPP
#define RS485DEV_FAN_HPP

#include <cassert>
#include "ProjectConfig.h"
#include "Bus/PWM.h"
#include "RS485Dev_PWMActor.hpp"
#include "Verification/CRC.h"

// 波特率 115200
// 执行器 fan
template<uint8_t busID>
class Fan final : public RS485DevActor {

#define FAN_DIVISION (50)
#define FAN_CHANNELS (1)
#define FAN_LENGTH (8)

public:
    explicit Fan(
        uint32_t dev_addr,
        uint8_t reg_addr,
        uint32_t div = FAN_DIVISION)
    : rs485Agent(dev_addr)
    , dev_addr_(dev_addr)
    , reg_addr_(reg_addr) {
        this->SetDivisionFactor(div);
    }

protected:
    RS485_Agent<busID> rs485Agent;
    uint32_t dev_addr_;
    uint8_t reg_addr_;

protected:
    /// 地址码(1), 功能码(1), 寄存器地址(2), 读取数量(2), CRC16(2)
    void Require() override {
        this->rs485Agent.txbuf[0] = dev_addr_;                                  // 地址码
        this->rs485Agent.txbuf[1] = 0x06;                                       // 功能码 03 读 06 写
        this->rs485Agent.txbuf[2] = 0x00;                                       // 寄存器启始地址[高]
        this->rs485Agent.txbuf[3] = reg_addr_;                                  // 寄存器启始地址[低] =0x08

        this->rs485Agent.txbuf[4] = (curr_comp_ & 0xFF00) >> 8;                 // 占空比[高]
        this->rs485Agent.txbuf[5] = (curr_comp_ & 0x00FF);                      // 占空比[低]

        auto crc16 = CRC16Calc(this->rs485Agent.txbuf, 6);        // CRC16
        this->rs485Agent.txbuf[6] = (crc16 & 0x00FF);                           // CRC16[低]
        this->rs485Agent.txbuf[7] = (crc16 & 0xFF00) >> 8;                      // CRC16[高]// 设置发送数据
        this->rs485Agent.SendMsg(8);                                        // 发送数据
    }

    /// 地址码(1), 功能码(1), 长度(1), 数据(n), CRC16(2)
    bool Check() override {return true;}

    /// 响应帧与请求帧完全一致
    void Update() override {}
};

#endif // RS485DEV_FAN_HPP
