/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
******************************************************************************/

#ifndef FINEMOTE_MOTOR4010_H
#define FINEMOTE_MOTOR4010_H

#include "Motors/MotorBase.hpp"
#include "Bus/CAN_Base.hpp"
#include <sensor_msgs/msg/joint_state.h>
#include "Control/Clamp.hpp"
#include <type_traits>
#include <cstdio>
#include "FreeRTOS.h"
#include "task.h"

/**
 * Todo: Reduction ratio
 */
template <uint8_t busID>
class Motor4010 : public MotorBase {
public:
    template <typename T>
        Motor4010(const Motor_Param_t&& params, T& _controller, uint32_t addr, uint8_t divisionFactor = 1)
            : MotorBase(std::forward<const Motor_Param_t>(params), divisionFactor),
              canAgent(addr)
    {
        ResetController(_controller);
    }

    void Handle() final {
        controller->Calc();
        MessageGenerate();
    }

    auto GetRosBinder()
    {
        return [this](sensor_msgs__msg__JointState& msg)
        {
            this->UpdateToRos(msg);
        };
    }

    CAN_Agent<busID> canAgent;

private:
    void UpdateToRos(sensor_msgs__msg__JointState& msg) {
        if constexpr (!WITH_MICRO_ROS) return;

        Motor_State_t s;
        this->UpdateSnapshot(s);

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

    void SetFeedback() final {
        Motor_State_t& state = GetInternalState();
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

    void MessageGenerate() {
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Torque: {
                ControllerOutputData output = controller->GetOutputs();
                int16_t txTorque = Clamp(1 * output.dataPtr[0], -500.f, 500.f);

                canAgent[0] = 0xA1;
                canAgent[1] = 0x00;
                canAgent[2] = 0x00;
                canAgent[3] = 0x00;
                canAgent[4] = txTorque;
                canAgent[5] = txTorque >> 8;
                canAgent[6] = 0x00;
                canAgent[7] = 0x00;
                break;
            }
            case Motor_Ctrl_Type_e::Position: {
                constexpr uint16_t txSpeed = 0x800;
                ControllerOutputData output = controller->GetOutputs();
                int32_t txAngle = 100 * output.dataPtr[0];

                canAgent[0] = 0xA4;
                canAgent[1] = 0x00;
                canAgent[2] = txSpeed;
                canAgent[3] = txSpeed >> 8;
                canAgent[4] = txAngle;
                canAgent[5] = txAngle >> 8;
                canAgent[6] = txAngle >> 16;
                canAgent[7] = txAngle >> 24;
                break;
            }
        }
        canAgent.Transmit(canAgent.addr);
    }

    void Update() override {
        Motor_State_t newState;
        newState.position = static_cast<int16_t>(canAgent.rxbuf[6] | (canAgent.rxbuf[7] << 8u)) * 360.0f / 16384.0f;
        newState.speed = static_cast<int16_t>(canAgent.rxbuf[4] | (canAgent.rxbuf[5] << 8u));
        newState.torque = static_cast<int16_t>(canAgent.rxbuf[2] | (canAgent.rxbuf[3] << 8u));
        newState.temperature = static_cast<int8_t>(canAgent.rxbuf[1]);

        this->CommitState(newState);
    }
};

#endif