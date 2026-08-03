/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_DDSM115_HPP
#define FINEMOTE_DDSM115_HPP

#include "Bus/RS485_Base.hpp"
#include "Control/Clamp.hpp"
#include "Motors/MotorBase.hpp"
#include "Verification/CRC.hpp"

/**
 * @brief DDSM115 直驱伺服电机的 RS-485 驱动适配封装
 * @details
 * 本类将高层控制器输出与 DDSM115 电机的 RS-485 协议对接，负责：
 *  - 将控制器输出（如速度）转换为电机命令，进行限幅、打包并附加 CRC 后发送；
 *  - 解析电机返回的反馈报文并更新 MotorBase::state（位置、速度、力矩、温度等）；
 *  - 通过模板参数 BusID 选择具体的 RS-485 总线实例并注册收发回调以处理接收数据。
 *
 * 设计目标是把通信与协议细节封装在此类中，使上层控制器仅需提供统一的输入/反馈接口。
 */

template <uint8_t BusID>
class DDSM115 : public MotorBase {
public:
    /**
     * @brief 构造函数
     * @tparam controller_type 控制器类型，必须满足 MotorBase::ResetController 的要求（提供 SetFeedbacks, GetOutputs, Calc 等接口）
     * @param params 电机参数，按值传入（通常以 std::move(params) 移动语义传递）
     * @param _controller 对应的控制器引用，驱动周期内会调用其 Calc()/GetOutputs()/SetFeedbacks()
     * @param _addr RS-485 从机地址（设备 ID）
     * @param divisionFactor 传动或分频因子，默认为 1，用于内部单位转换
     */
    template <typename controller_type>
    DDSM115(const Motor_Param_t&& params, controller_type& _controller, uint32_t _addr, uint32_t divisionFactor = 1)
            : MotorBase(std::forward<const Motor_Param_t>(params),divisionFactor),
              addr(_addr),temperature(0),
              rs485Agent(_addr, [this](uint8_t* data, size_t size) {
                  Decode(data, size);
              }) {
        ResetController(_controller);
    }

protected:
    /**
     * @brief 周期处理函数（由框架调用）
     * @details 在每个控制周期中调用控制器的 Calc() 计算输出，然后根据输出生成并发送电机报文。
     */
    void Handle() final {
        controller->Calc();
        MessageGenerate();
    }

    /**
     * @brief 根据当前控制模式设置控制器的反馈来源
     * @details 例如在速度控制模式下，将电机速度地址传递给控制器的 SetFeedbacks。
     */
    void SetFeedback() override {
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Speed:
                controller->SetFeedbacks(&state.speed);
                break;
            default:
                break;
        }
    }

private:
    static constexpr uint8_t TX_BUFFER_SIZE = 10;
    uint8_t txbuf[TX_BUFFER_SIZE] = {};
    uint8_t addr;

    RS485_Agent<BusID> rs485Agent;
    int8_t temperature; //电机温度，单位摄氏度

    /**
     * @brief 根据当前控制器输出构建并发送电机控制报文
     * @details 支持 Speed 控制模式：读取控制器输出（DPS），转换为电机所需的 RPM 单位，
     *          限幅后打包为 RS-485 报文并计算 CRC 后发送。
     */
    void MessageGenerate() {
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Speed: {
                ControllerOutputData output = controller->GetOutputs();
                float targetSpeed = output.dataPtr[0]; // 单位为DPS（度每秒）
                // 转换为RPM：1 RPM = 6 DPS
                float targetSpeedRPM = targetSpeed / 6.0f;
                targetSpeedRPM = Clamp(targetSpeedRPM, -330.0f, 330.0f);
                auto txSpeed = static_cast<int16_t>(targetSpeedRPM);
                    txbuf[0] = addr;
                    txbuf[1] = 0x64;
                    txbuf[2] = (txSpeed >> 8) & 0xFF;
                    txbuf[3] = txSpeed & 0xFF;
                    txbuf[4] = 0x00;
                    txbuf[5] = 0x00;
                    txbuf[6] = 0x00;        //Acce
                    txbuf[7] = 0x00;
                    txbuf[8] = 0x00;
                    txbuf[9] = CRCCalc<crc8_maxim_t>(txbuf, 9u);
                    rs485Agent.Transmit(txbuf, TX_BUFFER_SIZE);
                break;
            }
            default:
                break;
        }
    }

    /**
     * @brief 解析来自电机的状态反馈报文并更新内部状态
     * @param data 指向接收缓冲区
     * @param size 缓冲区大小（期望 10 字节）
     * @details 验证 CRC 并根据功能码解析位置、速度、力矩等数据，更新 state。
     */
    void Decode(uint8_t* data, size_t size) {
        if (size != 10) return;
        CRC_t<crc8_maxim_t> crc;
        if (crc.Calc(data, 9u) != data[9]) return;
        uint8_t funcCode = data[1];
        if (funcCode == 0x02) {
            state.position = (data[6] << 8) | data[7];
            // 电机返回的速度单位是RPM，转换为DPS：1 RPM = 6 DPS
            state.speed = ((data[4] << 8) | data[5]) * 6.0f;
            state.torque = (data[2] << 8) | data[3];
            temperature = 0;
        }
    }
};

#endif
