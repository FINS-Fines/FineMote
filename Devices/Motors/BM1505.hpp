//
// Created by wzj on 25-9-28.
//

#ifndef FINEMOTE_BM1505_HPP
#define FINEMOTE_BM1505_HPP

#include "Motors/MotorBase.hpp"
#include "Bus/CAN_Base.hpp"
#include "BM_grouper.hpp"
#include "Clamp.hpp"

template<int busID>
class BM1505 : public MotorBase {
public:
    template<typename T>
    BM1505(const Motor_Param_t &&params, T &_controller, uint32_t _motor_id, BM_grouper<busID> &_grouper) :
            MotorBase(std::forward<const Motor_Param_t>(params)), canAgent(0x96 + _motor_id), motor_id(_motor_id),
            grouper(&_grouper) {
        ResetController(_controller);
        initTick = HAL_GetTick();
    }

    void Handle() final {
        Update();
        controller->Calc();
        if (HAL_GetTick() - initTick <= 10) { ///todo 测试后若发现控制指令也有回包，考虑初始化把主动反馈关了
            SetEnabled(true);
        } else if (HAL_GetTick() - initTick <= 20) {// 10个有效包触发主动反馈
            ChooseCtrlType();
        } else if (HAL_GetTick() - initTick > 20) {
            MessageGenerate();
        }
    }

    CAN_Agent<busID> canAgent;
    BM_grouper<busID> *grouper;
    uint32_t motor_id;

private:
    uint32_t initTick;
    const float TorqueLimit = 27.f; // 27A
    const float SpeedLimit = 4499.f; // 750RPM -> 4500dps
    const float PositionLimit = 48.f; //只能0-48°，设置零点使得为最长->最短

    void SetFeedback() final {
        switch (params.targetType) {
            case Motor_Ctrl_Type_e::Position:
                controller->SetFeedbacks(&state.position);
                break;
            case Motor_Ctrl_Type_e::Speed:
                controller->SetFeedbacks(&state.speed);
                break;
        }
    }

    void SetEnabled(bool isTrue = true) {
        // 会对总线上所有此类电机生效
        if (isTrue) {
            canAgent[0] = 0x0A;
            canAgent[1] = 0x0A;
            canAgent[2] = 0x0A;
            canAgent[3] = 0x0A;
            canAgent[4] = 0x0A;
            canAgent[5] = 0x0A;
            canAgent[6] = 0x0A;
            canAgent[7] = 0x0A;
        } else {
            canAgent[0] = 0x09;
            canAgent[1] = 0x09;
            canAgent[2] = 0x09;
            canAgent[3] = 0x09;
            canAgent[4] = 0x09;
            canAgent[5] = 0x09;
            canAgent[6] = 0x09;
            canAgent[7] = 0x09;
        }
        canAgent.Transmit(0x105);
    }

    /**
     * Todo: 放到构造函数里面
     */
    void Start() {
        // 默认电机使能
//        canAgent[0] = 0x01;
//        canAgent[1] = 0x01;
//        canAgent[2] = 0x01;
//        canAgent[3] = 0x01;
//        canAgent[4] = 0x01;
//        canAgent[5] = 0x01;
//        canAgent[6] = 0x01;
//        canAgent[7] = 0x01;
//        canAgent.Transmit(0x106); // 设置1ms主动反馈
    }

    /**
     * Todo: 放到构造函数里面
     */
    void ChooseCtrlType() {
        switch (params.ctrlType) {
            //力矩环下，力矩（电流）代表电机在该力矩下运行
            case Motor_Ctrl_Type_e::Torque: {
                canAgent[0] = 0x01;
                canAgent[1] = 0x01;
                canAgent[2] = 0x01;
                canAgent[3] = 0x01;
                canAgent[4] = 0x01;
                canAgent[5] = 0x01;
                canAgent[6] = 0x01;
                canAgent[7] = 0x01;
                break;
            }
                //速度力矩环下，速度命令代表电机运行速度，力矩（电流）代表电机在该速度下运行，能提供的最大电流
            case Motor_Ctrl_Type_e::Speed: {
                canAgent[0] = 0x02;
                canAgent[1] = 0x02;
                canAgent[2] = 0x02;
                canAgent[3] = 0x02;
                canAgent[4] = 0x02;
                canAgent[5] = 0x02;
                canAgent[6] = 0x02;
                canAgent[7] = 0x02;
                break;
            }
        }
        canAgent.Transmit(0x105); // 设置工作模式，需要先使能才能切换模式。对所有同类型本末电机生效
    }

    void MessageGenerate() {
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Position: {
                ControllerOutputData output = controller->GetOutputs();
                float txSpeed = output.dataPtr[0];
                float clampedPosition = Clamp(txSpeed, 0.f, PositionLimit);
                int16_t txPositionCode = static_cast<int16_t>(clampedPosition * 10.f / 6.f); // DPS -> RPM * 10
                /// Todo: 4个指令组合一包发送
                canAgent[0] = (txPositionCode >> 8) & 0xFF;
                canAgent[1] = (txPositionCode) & 0xFF;
                break;
            }
            case Motor_Ctrl_Type_e::Speed: {
                ControllerOutputData output = controller->GetOutputs();
                float txSpeed = output.dataPtr[0];
                float clampedSpeed = Clamp(txSpeed, -SpeedLimit, SpeedLimit);
                int16_t txSpeedCode = static_cast<int16_t>(clampedSpeed * 10.f / 6.f); // DPS -> RPM * 10
                /// Todo: 4个指令组合一包发送
                canAgent[0] = (txSpeedCode >> 8) & 0xFF;
                canAgent[1] = (txSpeedCode) & 0xFF;
                break;
            }
            case Motor_Ctrl_Type_e::Torque: {
                ControllerOutputData output = controller->GetOutputs();
                float txTorque = output.dataPtr[0];
                float clampedTorque = Clamp(txTorque, -TorqueLimit, TorqueLimit);
                int16_t txTorqueCode = static_cast<int16_t>(clampedTorque * 32767.f / 55.f); // 55->32767
                /// Todo: 4个指令组合一包发送
                canAgent[0] = (txTorqueCode >> 8) & 0xFF;
                canAgent[1] = txTorqueCode & 0xFF;
                break;
            }
        }
//      canAgent.Transmit(canAgent.addr);
        if (motor_id >= 1 && motor_id <= 8) {
            grouper->AddMessage(canAgent[0], ((motor_id - 1) % 4) * 2);
            grouper->AddMessage(canAgent[1], ((motor_id - 1) % 4) * 2 + 1);
        }
    }

    void Update() {
        int16_t position_code = (canAgent.rxbuf[4] << 8) | canAgent.rxbuf[5];
        state.position = static_cast<float>(position_code) * 360.f / 32767.f;
        int16_t speed_code = (canAgent.rxbuf[0] << 8) | canAgent.rxbuf[1];
        state.speed = static_cast<float>(speed_code) * 0.6f;
        int16_t torque_code = (canAgent.rxbuf[2] << 8) | canAgent.rxbuf[3];
        state.torque = static_cast<float>(torque_code) * 55.f / 32767.f;
    }
};

#endif //FINEMOTE_BM1505_HPP
