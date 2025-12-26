//
// Created by wzj on 25-9-28.
//

#ifndef FINEMOTE_BM1010_H
#define FINEMOTE_BM1010_H

#include "Motors/MotorBase.hpp"
#include "Bus/CAN_Base.hpp"
#include "Motors/BM_grouper.hpp"
#include "Control/Clamp.hpp"

template<int busID>
class BM1010 : public MotorBase {
public:
    template<typename T>
    BM1010(const Motor_Param_t &&params, T &_controller, uint32_t _motor_id, BM_grouper<busID> &_grouper) :
            MotorBase(std::forward<const Motor_Param_t>(params)), canAgent(0x50 + _motor_id), motor_id(_motor_id),
            grouper(&_grouper) {
        ResetController(_controller);
        initTick = HAL_GetTick();
    }

    void Handle() final {
        Update();
        controller->Calc();
        if (HAL_GetTick() - initTick < 20) {
            ChooseCtrlType(); //失能情况进行模式切换
            SetEnabled();
        } else {
            MessageGenerate();
        }
    }

    CAN_Agent<busID> canAgent;
    BM_grouper<busID> *grouper;
    uint32_t motor_id;

private:
    uint32_t initTick;
    const float TorqueLimit = 74.f; // 75A
    const float SpeedLimit = 9599.f; // 1600RPM -> 9600dps
    const float PosLimit = 17999.f; //50round -> 18000deg

    void SetFeedback() final { //不重要
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
            canAgent[0] = 0x02;
            canAgent[1] = 0x02;
            canAgent[2] = 0x02;
            canAgent[3] = 0x02;
            canAgent[4] = 0x02;
            canAgent[5] = 0x02;
            canAgent[6] = 0x02;
            canAgent[7] = 0x02;
        } else {
            canAgent[0] = 0x01;
            canAgent[1] = 0x01;
            canAgent[2] = 0x01;
            canAgent[3] = 0x01;
            canAgent[4] = 0x01;
            canAgent[5] = 0x01;
            canAgent[6] = 0x01;
            canAgent[7] = 0x01;
        }
        canAgent.Transmit(0x38);
    }

    /**
     * Todo: 放到构造函数里面
     */
    void Start() {
        SetEnabled(true); // 设置使能
    }

    /**
     * Todo: 放到构造函数里面
     */
    void ChooseCtrlType() {
        /// 切换模式时电机需要失能
        switch (params.ctrlType) {
            //力矩环下，力矩（电流）代表电机在该力矩下运行
            case Motor_Ctrl_Type_e::Torque: {
                canAgent[0] = motor_id;
                canAgent[1] = 0x1C;
                canAgent[2] = 0x02;
                canAgent[3] = 0x00;
                canAgent[4] = 0x00;
                canAgent[5] = 0x00;
                canAgent[6] = 0x00;
                canAgent[7] = 0x00;
                break;
            }
                //速度力矩环下，速度命令代表电机运行速度，力矩（电流）代表电机在该速度下运行，能提供的最大电流
            case Motor_Ctrl_Type_e::Speed: {
                canAgent[0] = motor_id;
                canAgent[1] = 0x1C;
                canAgent[2] = 0x03;
                canAgent[3] = 0x00;
                canAgent[4] = 0x00;
                canAgent[5] = 0x00;
                canAgent[6] = 0x00;
                canAgent[7] = 0x00;
                break;
            }
            case Motor_Ctrl_Type_e::Position: {
                canAgent[0] = motor_id;
                canAgent[1] = 0x1C;
                canAgent[2] = 0x04;
                canAgent[3] = 0x00;
                canAgent[4] = 0x00;
                canAgent[5] = 0x00;
                canAgent[6] = 0x00;
                canAgent[7] = 0x00;
                break;
            }
        }
        canAgent.Transmit(0x36); // 设置工作模式，对所有同类型本末电机生效
    }

    void MessageGenerate() {
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Speed: {
                ControllerOutputData output = controller->GetOutputs();
                float txSpeed = output.dataPtr[0];
                float clampedSpeed = Clamp(txSpeed, -SpeedLimit, SpeedLimit);
                int16_t txSpeedCode = static_cast<int16_t>(clampedSpeed * 10.f / 6.f); // DPS -> RPM * 10
                /// Todo: 4个指令组合一包发送
                canAgent[4] = (txSpeedCode >> 8) & 0xFF;
                canAgent[5] = (txSpeedCode) & 0xFF;
                break;
            }
            case Motor_Ctrl_Type_e::Torque: {
                ControllerOutputData output = controller->GetOutputs();
                float txTorque = output.dataPtr[0];
                float clampedTorque = Clamp(txTorque, -TorqueLimit, TorqueLimit);
                int16_t txTorqueCode = static_cast<int16_t>(clampedTorque * 100.f); //
                /// Todo: 4个指令组合一包发送
                canAgent[4] = (txTorqueCode >> 8) & 0xFF;
                canAgent[5] = txTorqueCode & 0xFF;
                break;
            }
            case Motor_Ctrl_Type_e::Position: {
                ControllerOutputData output = controller->GetOutputs();
                float txPos = output.dataPtr[0];
                float clampedPos = Clamp(txPos, -PosLimit, PosLimit);;
                int16_t txPosCode = static_cast<int16_t>(clampedPos / 360.f * 100); //round*100
                /// Todo: 4个指令组合一包发送
                canAgent[4] = (txPosCode >> 8) & 0xFF;
                canAgent[5] = txPosCode & 0xFF;
                break;
            }
        }
//      canAgent.Transmit(canAgent.addr);
        if (motor_id >= 1 && motor_id <= 8) {
            grouper->AddMessage(canAgent[4], ((motor_id - 1) % 4) * 2);
            grouper->AddMessage(canAgent[5], ((motor_id - 1) % 4) * 2 + 1);
        }

    }

    void Update() {
        int16_t position_code = (canAgent.rxbuf[4] << 8) | canAgent.rxbuf[5];
        state.position = static_cast<float>(position_code) * 360.f / 32767.f;
        int16_t speed_code = (canAgent.rxbuf[0] << 8) | canAgent.rxbuf[1];
        state.speed = static_cast<float>(speed_code) * 0.6f; //10rpm->dps
        int16_t torque_code = (canAgent.rxbuf[2] << 8) | canAgent.rxbuf[3];
        state.torque = static_cast<float>(torque_code) / 100.f;//Amp
    }
};

#endif //FINEMOTE_BM1010_H
