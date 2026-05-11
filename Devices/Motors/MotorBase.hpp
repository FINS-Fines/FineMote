#ifndef FINEMOTE_MOTORBASE_H
#define FINEMOTE_MOTORBASE_H

#include "../DeviceBase/DeviceBase.hpp"
#include "Control/ImplementControlBase.hpp"
#include "StateSnapshot.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>
#include <sensor_msgs/msg/joint_state.h>

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
        : DeviceBase(divisionFactor), params(params) {

    }

    void ResetController(ImplementControllerBase<1,1>& _controller) {
        controller = &_controller;
        _controller.SetTargets(&target);
        this->SetFeedback();
    }

    void Stop() {
        switch (params.targetType) {
            case Motor_Ctrl_Type_e::Position:
                SetTargetAngle(GetState().position);
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

        Motor_State_t current_s = GetState();

        if (params.multiTurnSamePosition) {
            while (target - current_s.position < -180.f * params.reductionRatio) {
                target += 360.f * params.reductionRatio;
            }
            while (target - current_s.position > 180.f * params.reductionRatio) {
                target -= 360.f * params.reductionRatio;
            }
        }
    }

    Motor_State_t GetState() const {
        return stateSnapshot_.Read();
    }

    const Motor_State_t* GetStatePtr() const {
        return stateSnapshot_.GetPtr();
    }

    float GetMultiTurnPosition() const {
        return GetState().position / params.reductionRatio;
    }

    void CommitState(const Motor_State_t& newState) {
        stateSnapshot_.Commit(newState);
    }

    void UpdateToRos(sensor_msgs__msg__JointState& msg) const {
        if constexpr (WITH_MICRO_ROS) {
            Motor_State_t s = stateSnapshot_.Read();
            uint32_t ticks = xTaskGetTickCount();
            msg.header.stamp.sec = ticks / configTICK_RATE_HZ;
            msg.header.stamp.nanosec = (ticks % configTICK_RATE_HZ) * (1000000000 / configTICK_RATE_HZ);
            if (msg.position.capacity >= 1) {
                msg.position.data[0] = s.position;
                msg.position.size = 1;
            }
            if (msg.velocity.capacity >= 1) {
                msg.velocity.data[0] = s.speed;
                msg.velocity.size = 1;
            }
            if (msg.effort.capacity >= 1) {
                msg.effort.data[0] = s.torque;
                msg.effort.size = 1;
            }
        }
    }

    auto GetRosBinder() {
        return [this](sensor_msgs__msg__JointState& msg) { this->UpdateToRos(msg); };
    }

protected:
    virtual void SetFeedback() = 0;

    float target = 0; //多圈目标，减速后
    Motor_Param_t params;
    ImplementControllerBase<1,1>* controller = nullptr;
    StateSnapshot<Motor_State_t> stateSnapshot_;
};

#endif
