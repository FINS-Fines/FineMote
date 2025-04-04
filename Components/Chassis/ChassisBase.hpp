// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef CHASSISBASE_H
#define CHASSISBASE_H

class WithoutOdom {
    void SetOdom() {}
    const std::array<float, 3>& GetOdom() {
        static std::array<float, 3> v = {0};
        return v;
    }
    void UpdateOdom(const std::array<float, 3>&) {}
};

class Odom {
public:
    void SetOdom(const std::array<float, 3>& x) {
        estimatedX = x;
    }

    const std::array<float, 3>& GetOdom() {
        return estimatedX;
    }

    void UpdateOdom(const std::array<float, 3>& v){
        estimatedX[0] += (v[0] * cosf(estimatedX[2]) - v[1] * sinf(estimatedX[2])) * 0.001f;
        estimatedX[1] += (v[0] * sinf(estimatedX[2]) + v[1] * cosf(estimatedX[2])) * 0.001f;
        estimatedX[2] += v[2] * 0.001f;
    }

private:
    std::array<float, 3> estimatedX = {0};
};

template <typename Odometer>
class ChassisBase : public DeviceBase, private Odometer{
public:
    virtual void InverseKinematics(std::array<float,3>&) = 0; // 底盘到轮组
    virtual void ForwardKinematics() = 0; // 轮组到底盘

    template <typename T>
    void SetVelocity(T&& v) {
        targetV = std::forward<T>(v);
    }

    using Odometer::SetOdom;
    using Odometer::GetOdom;
    using Odometer::UpdateOdom;

protected:
    std::array<float, 3> targetV = {0};
    std::array<float, 3> estimatedV = {0};
};

#endif //CHASSISBASE_H
