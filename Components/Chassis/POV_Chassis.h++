/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_POV_CHASSIS_H
#define FINEMOTE_POV_CHASSIS_H

#include <array>
#include "arm_math.h"

#include "ChassisBase.h++"

using Swerve_t = struct Swerve_t {
    MotorBase* steerMotor;
    MotorBase* driveMotor;
    float lx;
    float ly;
    float zeroPosition;
};

template <size_t N>
class POV_Chassis : public ChassisBase {
public:
    POV_Chassis(const float _wheelDiameter, std::array<Swerve_t, N>&& configs) : modules(std::move(configs)), wheelDiameter(_wheelDiameter) {}

    void InverseKinematics(std::array<float,3>& v) final {
        // Inverse Kinematics
        for(auto& module : modules) {
            float vx = v[0] - module.ly * v[2];
            float vy = v[1] + module.lx * v[2];
            float v = sqrtf(vx * vx + vy * vy);
            float angle = atan2f(vy, vx) * 180 / PI;

            bool isNegative = false;
            while (angle + module.zeroPosition - module.steerMotor->GetMultiTurnPosition() > 100) { //100是换向施密特门
                angle -= 180;
                isNegative = !isNegative;
            }
            while (angle + module.zeroPosition - module.steerMotor->GetMultiTurnPosition() < -100) {
                angle += 180;
                isNegative = !isNegative;
            }
            if (isNegative) {
                v = -v;
            }

            module.steerMotor->SetTargetAngle(angle + module.zeroPosition);
            module.driveMotor->SetTargetSpeed(v / wheelDiameter * 360);
        }
    }

    void ForwardKinematics() final {
        // Forward Kinematics
    }

    void Handle() final {
        ForwardKinematics();
        // Control
        InverseKinematics(targetV);
    }

private:
    std::array<Swerve_t, N> modules;
    const float wheelDiameter;
};

template <typename... Configs>
auto POV_ChassisBuilder(float _wheelDiameter, Configs&&... configs) {
    constexpr size_t N = sizeof...(Configs);
    static_assert((std::is_same<std::decay_t<Configs>, Swerve_t>::value && ...), "All configs must be of type Swerve_t");
    std::array<Swerve_t, N> modules = {std::forward<Configs>(configs)...};
    return POV_Chassis<N>(_wheelDiameter, std::move(modules));
}

#endif //FINEMOTE_POV_CHASSIS_H
