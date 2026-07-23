// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINEMOTE_CHASSISBASE_H
#define FINEMOTE_CHASSISBASE_H

#include <array>
#include <type_traits>
#include "StateSnapshot.hpp"
#include "DeviceBase/DeviceBase.hpp"
#include "MicroROS/MicroROS_Agent.hpp"
#include <geometry_msgs/msg/twist.h>

typedef struct {
    std::array<float, 3> position;
    std::array<float, 3> velocity;
} Chassis_State_t;

template <int DOFs>
class WithoutOdom {
public:
    void SetOdom(const std::array<float, DOFs>& x) {}

    const std::array<float, DOFs>& GetOdom() {
        static std::array<float, DOFs> v = {0};
        return v;
    }

    template<typename T>
    void UpdateOdom(T&& v, uint32_t dt) {}
};

class PlanarOdom {
public:
    void SetOdom(const std::array<float, 3>& x) {
        estimatedX = x;
    }

    const std::array<float, 3>& GetOdom() {
        return estimatedX;
    }

    void UpdateOdom(const std::array<float, 3>& v, uint32_t dt) {
        estimatedX[0] += (v[0] * cosf(estimatedX[2]) - v[1] * sinf(estimatedX[2])) * 0.001f * dt;
        estimatedX[1] += (v[0] * sinf(estimatedX[2]) + v[1] * cosf(estimatedX[2])) * 0.001f * dt;
        estimatedX[2] += v[2] * 0.001f * dt;
    }

private:
    std::array<float, 3> estimatedX = {0};
};

template <typename OdomPolicy>
class ChassisBase : public DeviceBase {
public:
    virtual void InverseKinematics(std::array<float,3>&) = 0; // 底盘到轮组
    virtual void ForwardKinematics() = 0; // 轮组到底盘

    template <typename T>
    void SetVelocity(T&& v) {
        targetV = std::forward<T>(v);
    }

    Chassis_State_t GetChassisState() const {
        return stateSnapshot_.Read();
    }

    const Chassis_State_t* GetChassisStatePtr() const {
        return stateSnapshot_.GetPtr();
    }

    auto GetRosBinder() {
        return [this](geometry_msgs__msg__Twist& msg) {
            this->UpdateToRos(msg);
        };
    }

protected:
    template <typename MsgT>
    void UpdateToRos(MsgT& msg) {
        if constexpr (!microros_supported) return;

        Chassis_State_t state = GetChassisState();

        msg.linear.x = state.velocity[0];
        msg.linear.y = state.velocity[1];
        msg.linear.z = 0.0f;

        msg.angular.x = 0.0f;
        msg.angular.y = 0.0f;
        msg.angular.z = state.velocity[2];
    }

    void CommitChassisState(const Chassis_State_t& newState) {
        stateSnapshot_.Commit(newState);
    }

    OdomPolicy odom;

    std::array<float, 3> targetV = {0};
    std::array<float, 3> estimatedV = {0};

    StateSnapshot<Chassis_State_t> stateSnapshot_;
};

#endif
