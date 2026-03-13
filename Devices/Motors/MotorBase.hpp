#ifndef FINEMOTE_MOTORBASE_H
#define FINEMOTE_MOTORBASE_H

#include "../DeviceBase/DeviceBase.hpp"
#include "Control/ImplementControlBase.hpp"
#include "DoubleBuffer.hpp"
#include <cstdint>

enum class Motor_Ctrl_Type_e: uint16_t {
    Position = 0,
    Speed,
    Torque,
};

typedef struct {
    float position; //单位为度
    float speed; //单位为DPS
    float torque; //转矩电流的相对值，具体值参考电调手册
    int8_t temperature; //电机温度，单位摄氏度
} Motor_State_t;

using Motor_Param_t = struct Motor_Param_t {
    Motor_Ctrl_Type_e ctrlType; //控制电机的方式
    Motor_Ctrl_Type_e targetType; //控制电机哪个状态
    bool multiTurnSamePosition = false; //多圈电机是否在同一位置
    const float reductionRatio = 1; //减速比
};

class MotorBase : public DeviceBase {
public:
    explicit MotorBase(const Motor_Param_t& params, uint8_t divisionFactor = 1)
         : DeviceBase(divisionFactor),
           params(params),
           stateBuffer([](uint8_t*, size_t){}) {
    }


    void ResetController(ImplementControllerBase<1,1>& _controller) {

        controller = &_controller;
        _controller.SetTargets(&target);
        this->SetFeedback();
    }

    void Stop() {
        Motor_State_t current_s;
        UpdateSnapshot(current_s);

        switch (params.targetType) {
            case Motor_Ctrl_Type_e::Position:
                SetTargetAngle(current_s.position);
                break;
            case Motor_Ctrl_Type_e::Speed:
                SetTargetSpeed(0);
                break;
            case Motor_Ctrl_Type_e::Torque:
                /** ToDo */
                break;
        }
    }

    void Enable() {

    }

    void Disable() {

    }

    /** Todo: 筛查电机控制类型，不合理调用的Set需要警告 */
    void SetTargetSpeed(float targetSpeed) {
        if(params.targetType != Motor_Ctrl_Type_e::Speed) {
            return;
        }
        target = targetSpeed * params.reductionRatio; //多圈目标，减速后
    }

    void SetTargetAngle(float targetAngle) {
        if(params.targetType != Motor_Ctrl_Type_e::Position) {
            return;
        }
        target = targetAngle * params.reductionRatio; //多圈目标，减速后

        Motor_State_t current_s;
        UpdateSnapshot(current_s);

        if (params.multiTurnSamePosition) {
            while (target - current_s.position < -180.f * params.reductionRatio){
                target += 360.f * params.reductionRatio;
            }
            while (target - current_s.position > 180.f * params.reductionRatio){
                target -= 360.f * params.reductionRatio;
            }
        }
    }

    void UpdateSnapshot(Motor_State_t& dest) {
        memcpy(&dest, stateBuffer.GetBuffer(), sizeof(Motor_State_t));
    }

    Motor_State_t& GetInternalState() {
        return *reinterpret_cast<Motor_State_t*>(stateBuffer.GetBuffer());
    }

    const float GetMultiTurnPosition() {
        Motor_State_t current_s;
        UpdateSnapshot(current_s);
        return current_s.position / params.reductionRatio;    }

protected:
    virtual void SetFeedback() = 0;

    void CommitState(const Motor_State_t& new_state) {
            uint8_t* raw_buf = stateBuffer.GetBuffer();
        memcpy(raw_buf, &new_state, sizeof(Motor_State_t));
        stateBuffer.CommitBuffer(sizeof(Motor_State_t));
    }

    float target = 0; //多圈目标，减速后
    DoubleBuffer<sizeof(Motor_State_t)> stateBuffer; //单圈状态，不考虑减速
    Motor_Param_t params;
    ImplementControllerBase<1,1>* controller = nullptr;
};

#endif
