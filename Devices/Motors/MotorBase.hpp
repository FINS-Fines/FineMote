/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MOTORBASE_HPP
#define FINEMOTE_MOTORBASE_HPP

#include "DeviceBase/DeviceBase.hpp"
#include "Control/ImplementControlBase.hpp"

enum class Motor_Ctrl_Type_e : uint8_t {
    Position = 0,
    Speed,
    Torque,
};

typedef struct {
    float position; //单位为度
    float speed; //单位为DPS
    float torque; //转矩电流的相对值，具体值参考电调手册
} Motor_State_t;

using Motor_Param_t = struct Motor_Param_t {
    Motor_Ctrl_Type_e ctrlType; //控制电机的方式
    Motor_Ctrl_Type_e targetType; //控制电机哪个状态
    bool multiTurnSamePosition = false; //多圈电机是否在同一位置
    const float reductionRatio = 1; //减速比
};

class MotorBase: public DeviceBase {
public:
    explicit MotorBase(const Motor_Param_t& params, uint32_t divisionFactor = 1):
        DeviceBase(divisionFactor),
        params(params) {}

    void ResetController(ImplementControllerBase<1, 1>& _controller) {
        controller = &_controller;
        _controller.SetTargets(&target);
        this->SetFeedback();
    }

    void SoftwareStop() {
        switch (params.targetType) {
            case Motor_Ctrl_Type_e::Position:
                SetTarget(state.position);
                break;
            case Motor_Ctrl_Type_e::Speed:
                SetTarget(0);
                break;
            case Motor_Ctrl_Type_e::Torque:
                SetTarget(0);
                break;
        }
    }

    void Enable() {}

    void Disable() {}

    void SetTarget(float _target) {
        switch (params.targetType) {
            case Motor_Ctrl_Type_e::Position:
                target = _target * params.reductionRatio; //多圈目标，减速后

                if (params.multiTurnSamePosition) {
                    while (target - state.position < -180.f * params.reductionRatio) {
                        target += 360.f * params.reductionRatio;
                    }
                    while (target - state.position > 180.f * params.reductionRatio) {
                        target -= 360.f * params.reductionRatio;
                    }
                }
                break;

            case Motor_Ctrl_Type_e::Speed:
                target = _target * params.reductionRatio;
                break;

            case Motor_Ctrl_Type_e::Torque:
                target = _target;
                break;
        }
    }

    void SetState(const Motor_State_t& _state) {
        state = _state;
    }

    [[nodiscard]] const Motor_State_t& GetState() {
        return state;
    }

    [[nodiscard]] float GetMultiTurnPosition() const {
        return state.position / params.reductionRatio;
    }

protected:
    virtual void SetFeedback() = 0;

    float target = 0; //多圈目标，减速后
    Motor_State_t state = { 0, 0, 0 }; //单圈状态，不考虑减速
    Motor_Param_t params;
    ImplementControllerBase<1, 1>* controller = nullptr;
};

#endif
