//
// Created by wzj on 25-9-28.
//

#ifndef FINEMOTE_DM4310_H
#define FINEMOTE_DM4310_H

#include "Motors/MotorBase.hpp"
#include "Bus/CAN_Base.hpp"
#include "Control/Clamp.hpp"

template<int busID>
class DM4310 : public MotorBase {
public:
    template<typename T>
    DM4310(const Motor_Param_t &&params, T &_controller, uint32_t addr) //addr和接收地址绑定，得填MasterID
            : MotorBase(std::forward<const Motor_Param_t>(params)), canAgent(addr) {
        ResetController(_controller);
        initTick = HAL_GetTick();
    }

    void Handle() final {
        Update();
        controller->Calc();
        if (HAL_GetTick() - initTick < 20) {
            SetEnabled();
        } else {
            MessageGenerate();
        }
    }

    void SetMITCommand(float _pos, float _vel, float _torque, float _kp, float _kd) {
        MIT_cmd.pos_d = _pos;
        MIT_cmd.vel_d = _vel;
        MIT_cmd.torque_d = _torque;
        MIT_cmd.kp = _kp;
        MIT_cmd.kd = _kd;
        MIT_cmd.updated = true;
    }

    CAN_Agent<busID> canAgent;

private:
    uint32_t initTick;

    struct MIT_cmd_t {
        float pos_d = 0.f;
        float vel_d = 0.f;
        float torque_d = 0.f;
        float kp = 0.f;
        float kd = 0.f;
        bool updated = false;
    } MIT_cmd;


    float uint2float(uint8_t bits, uint16_t uint_raw, float min, float max) {
        return ((float) uint_raw * (max - min) / (float) ((1 << bits) - 1)) + min;
    }

    uint16_t float2uint(uint8_t bits, float float_raw, float min, float max) {
        Clamp<float>(float_raw, min, max);
        float tmp = (float_raw - min) * (float) ((1 << bits) - 1) / (max - min);
        return (uint16_t) tmp;
    }

    void SetFeedback() final {
        switch (this->params.targetType) {
            case Motor_Ctrl_Type_e::Position:
                controller->SetFeedbacks(&state.position);
                break;
            case Motor_Ctrl_Type_e::Speed:
                controller->SetFeedbacks(&state.speed);
                break;
            default:
                break;
        }
    }

    void SetEnabled(bool isTrue = true) {
        if (isTrue) {
            canAgent[0] = 0xFF;
            canAgent[1] = 0xFF;
            canAgent[2] = 0xFF;
            canAgent[3] = 0xFF;
            canAgent[4] = 0xFF;
            canAgent[5] = 0xFF;
            canAgent[6] = 0xFF;
            canAgent[7] = 0xFC;
        } else {
            canAgent[0] = 0xFF;
            canAgent[1] = 0xFF;
            canAgent[2] = 0xFF;
            canAgent[3] = 0xFF;
            canAgent[4] = 0xFF;
            canAgent[5] = 0xFF;
            canAgent[6] = 0xFF;
            canAgent[7] = 0xFD;
        }
        canAgent.Transmit(canAgent.addr - 0x100);
    }


    void MessageGenerate() { //MIT mode
        if (!MIT_cmd.updated) { // 未更新指令，不发送
            return;
        }
        uint16_t pos_d_int = float2uint(16, MIT_cmd.pos_d, -3.14159f, 3.14159f);
        uint16_t vel_d_int = float2uint(12, MIT_cmd.vel_d, -30.f, 30.f);
        uint16_t torque_d_int = float2uint(12, MIT_cmd.torque_d, -10.f, 10.f);
        uint16_t kp_int = float2uint(12, MIT_cmd.kp, 0, 500.f);
        uint16_t kd_int = float2uint(12, MIT_cmd.kd, 0, 5.f);
        MIT_cmd.updated = false;

//        ControllerOutputData output = controller->GetOutputs();

        canAgent[0] = pos_d_int >> 8u;
        canAgent[1] = pos_d_int & 0xFF;
        canAgent[2] = vel_d_int >> 4u;
        canAgent[3] = (vel_d_int << 4u & 0xF0) | (kp_int >> 8u & 0x0F);
        canAgent[4] = kp_int & 0xFF;
        canAgent[5] = kd_int >> 4u;
        canAgent[6] = (kd_int << 4u & 0xF0) | (torque_d_int >> 8u & 0x0F);
        canAgent[7] = torque_d_int & 0xFF;

        canAgent.Transmit(canAgent.addr - 0x100);
    }

    void Update() {
        state.position = uint2float(16, (canAgent.rxbuf[2] | (canAgent.rxbuf[1] << 8u)), -3.14159f, 3.14159f);
        state.speed = uint2float(12, (canAgent.rxbuf[4] >> 4u | (canAgent.rxbuf[3] << 4u)), -30.f, 30.f);
        state.torque = uint2float(12, (canAgent.rxbuf[5] | ((canAgent.rxbuf[4] & 0x0f) << 8u)), -10.f, 10.f);
        state.temperature = (canAgent.rxbuf[7]);
    }
};


#endif //FINEMOTE_DM4310_H
