// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef CHASSISBASE_H
#define CHASSISBASE_H

class ChassisBase : public DeviceBase {
public:
    virtual void InverseKinematics(std::array<float,3>&) = 0; // 底盘到轮组
    virtual void ForwardKinematics() = 0; // 轮组到底盘

    template <typename T>
    void SetVelocity(T&& v) {
        targetV = std::forward<T>(v);
    }

    void SetPosition() {
        // Set position
    }

protected:
    std::array<float, 3> targetV = {0};
    std::array<float, 3> estimatedV = {0};
};

using Swerve_t = struct Swerve_t {
    MotorBase* steerMotor;
    MotorBase* driveMotor;
    float lx;
    float ly;
    float zeroPosition;
};

#endif //CHASSISBASE_H
