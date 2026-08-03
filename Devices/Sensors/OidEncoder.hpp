/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_OIDENCODER_HPP
#define FINEMOTE_OIDENCODER_HPP
#include "AngleSensorBase.hpp"
#include "Bus/CAN_Base.hpp"


enum class ENCODER_RESOLUTION {
    _10BIT = 10,
    _15BIT = 15
};

/**
 * @brief OID（欧艾迪）绝对值编码器驱动
 *
 * 该类通过 CAN 总线与 OID 协议的绝对值编码器通讯，支持不同分辨率的编码器。
 * 模板参数指定所使用的 CAN 总线 ID 以及编码器分辨率（10-bit 或 15-bit）。
 * 解析到的原始计数值会转换为角度（0-360°）并保存到基类的 angle 成员。
 *
 * @tparam busID CAN 总线编号，用于实例化内部的 CAN_Agent。
 * @tparam resolution 编码器分辨率，使用枚举 ENCODER_RESOLUTION（_10BIT 或 _15BIT）。
 */

template <size_t busID, ENCODER_RESOLUTION resolution>
class OidEncoder : public AngleSensorBase {
public:
    /**
     * @brief 构造函数
     * @param addr 传感器地址
     * @param divisionFactor 分频系数
     * @param zeroPos 零点位置【0-360】度
     */
    OidEncoder(uint32_t addr, const float zeroPos, const uint32_t divisionFactor = 1) :
        AngleSensorBase(zeroPos,divisionFactor),
        canAgent(addr){
        canAgent.SetDLC(4);
    }

protected:
    /**
     * @brief 更新传感器状态，处理接收数据
     */
    void Update() final {
        if (canAgent.rxbuf[0] == 0x07 && canAgent.rxbuf[1] == canAgent.addr && canAgent.rxbuf[2] == 0x01) {
            // 解析编码器值（24位，小端序）
            uint32_t rawValue  = (canAgent.rxbuf[3]) |
                                 ((canAgent.rxbuf[4]) << 8) |
                                 ((canAgent.rxbuf[5]) << 16) |
                                 ((canAgent.rxbuf[6]) << 24);

            // 计算角度值 1024.0f（10位分辨率） 32768.0f（15位分辨率）
            constexpr int bit = static_cast<int>(resolution);
            constexpr float Res = 1U << bit;
            angle = rawValue * 360.0f / Res;
        }
    }

    /**
     * @brief 处理函数（定期调用）
     */
    void Handle() final {
        AcquireRawangle();
    }

private:
    /**
     * @brief 从can总线发送读取角度指令
     */
    void AcquireRawangle() {
        canAgent[0] = 0x04; // 数据长度
        canAgent[1] = canAgent.addr; //编码器地址
        canAgent[2] = 0x01; //指令码：读取编码器值
        canAgent[3] = 0x00; //保留

        canAgent.Transmit(canAgent.addr);
    }

    CAN_Agent<busID> canAgent;
};

#endif
