//
// Created by tim67 on 2026/4/4.
//

#ifndef MC_BOARD_02_MAHONY_HPP
#define MC_BOARD_02_MAHONY_HPP

#include <cmath>
#include <tuple>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

class Mahony {
public:
    /**
     * 构造函数
     * @param freq  采样频率 (Hz)，必须与实际调用 update 的频率一致
     * @param kp    比例增益，控制加速度/磁力计误差对陀螺仪的修正强度，典型值 0.1~1.0
     * @param ki    积分增益，用于消除陀螺仪零偏，典型值 0.001~0.05
     */
    Mahony(float freq, float kp, float ki)
        : Kp(kp), Ki(ki), sampleFreq(freq),
          integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f) {
        // 初始化四元数为单位四元数
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
    }

    /**
     * 重置姿态（四元数归零，积分项清零）
     */
    void reset() {
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
        integralFBx = integralFBy = integralFBz = 0.0f;
    }

    /**
     * 动态修改增益
     */
    void setGains(float kp, float ki) {
        Kp = kp;
        Ki = ki;
    }

    /**
     * 动态修改采样频率（注意：必须在 update 前调用，否则 dt 不一致）
     */
    void setSampleFreq(float freq) {
        sampleFreq = freq;
    }

    /**
     * 仅使用加速度计 + 陀螺仪更新姿态 (IMU)
     * @param gx, gy, gz  陀螺仪角速度，单位：rad/s
     * @param ax, ay, az  加速度计值，单位任意（内部会归一化）
     */
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az) {
        float dt = 1.0f / sampleFreq;

        // 检查加速度计是否有效（模长接近 1g，允许一定误差）
        float aNormSq = ax*ax + ay*ay + az*az;
        bool accValid = (aNormSq > 0.1f);  // 模长 > 0.316g，避免自由落体时错误修正

        if (accValid) {
            // 归一化加速度计
            float recipNorm = 1.0f / std::sqrt(aNormSq);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // 估计重力方向（半长向量）
            float halfvx = q1 * q3 - q0 * q2;
            float halfvy = q0 * q1 + q2 * q3;
            float halfvz = q0 * q0 - 0.5f + q3 * q3;

            // 误差 = 测量向量 × 估计向量
            float halfex = ay * halfvz - az * halfvy;
            float halfey = az * halfvx - ax * halfvz;
            float halfez = ax * halfvy - ay * halfvx;

            // 积分反馈
            integralFBx += Ki * halfex * dt;
            integralFBy += Ki * halfey * dt;
            integralFBz += Ki * halfez * dt;

            // 修正陀螺仪
            gx += Kp * halfex + integralFBx;
            gy += Kp * halfey + integralFBy;
            gz += Kp * halfez + integralFBz;
        } else {
            // 加速度无效时，仍然使用陀螺仪积分（不做修正）
            // 积分项保持不变，不累加误差
        }

        // 四元数微分方程
        gx *= 0.5f * dt;
        gy *= 0.5f * dt;
        gz *= 0.5f * dt;

        float nq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz);
        float nq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy);
        float nq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx);
        float nq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx);

        // 归一化四元数
        float recipNorm = 1.0f / std::sqrt(nq0*nq0 + nq1*nq1 + nq2*nq2 + nq3*nq3);
        q0 = nq0 * recipNorm;
        q1 = nq1 * recipNorm;
        q2 = nq2 * recipNorm;
        q3 = nq3 * recipNorm;
    }

    /**
     * 使用加速度计 + 陀螺仪 + 磁力计更新姿态 (MARG)
     * @param gx, gy, gz  陀螺仪角速度，单位：rad/s
     * @param ax, ay, az  加速度计值，单位任意（内部会归一化）
     * @param mx, my, mz  磁力计值，单位任意（内部会归一化）
     */
    void updateMARG(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz) {
        float dt = 1.0f / sampleFreq;

        // 检查加速度计有效性
        float aNormSq = ax*ax + ay*ay + az*az;
        bool accValid = (aNormSq > 0.1f);

        // 检查磁力计有效性
        float mNormSq = mx*mx + my*my + mz*mz;
        bool magValid = (mNormSq > 0.1f);

        // 如果两个传感器都无效，则只做陀螺仪积分
        // if (!accValid && !magValid) {
        //     goto gyro_only;
        // }

        // 归一化有效传感器数据
        if (accValid) {
            float recipNorm = 1.0f / std::sqrt(aNormSq);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        }
        if (magValid) {
            float recipNorm = 1.0f / std::sqrt(mNormSq);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
        }

        // 辅助变量
        float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
        float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
        float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

        // 估计重力方向（半长向量）
        float halfvx = q1q3 - q0q2;
        float halfvy = q0q1 + q2q3;
        float halfvz = q0q0 - 0.5f + q3q3;

        // 磁力计：参考方向在地理系中的投影
        float hx = 2.0f * (mx*(0.5f - q2q2 - q3q3) + my*(q1q2 - q0q3) + mz*(q1q3 + q0q2));
        float hy = 2.0f * (mx*(q1q2 + q0q3) + my*(0.5f - q1q1 - q3q3) + mz*(q2q3 - q0q1));
        float bx = std::sqrt(hx*hx + hy*hy);
        float bz = 2.0f * (mx*(q1q3 - q0q2) + my*(q2q3 + q0q1) + mz*(0.5f - q1q1 - q2q2));

        // 估计磁场方向（半长向量）
        float halfwx = bx*(0.5f - q2q2 - q3q3) + bz*(q1q3 - q0q2);
        float halfwy = bx*(q1q2 - q0q3) + bz*(q0q1 + q2q3);
        float halfwz = bx*(q0q2 + q1q3) + bz*(0.5f - q1q1 - q2q2);

        // 误差初始化
        float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

        // 加速度误差
        if (accValid) {
            halfex += ay * halfvz - az * halfvy;
            halfey += az * halfvx - ax * halfvz;
            halfez += ax * halfvy - ay * halfvx;
        }
        // 磁力计误差
        if (magValid) {
            halfex += my * halfwz - mz * halfwy;
            halfey += mz * halfwx - mx * halfwz;
            halfez += mx * halfwy - my * halfwx;
        }

        // 积分反馈
        integralFBx += Ki * halfex * dt;
        integralFBy += Ki * halfey * dt;
        integralFBz += Ki * halfez * dt;

        // 修正陀螺仪
        gx += Kp * halfex + integralFBx;
        gy += Kp * halfey + integralFBy;
        gz += Kp * halfez + integralFBz;

    gyro_only:
        // 四元数微分方程
        gx *= 0.5f * dt;
        gy *= 0.5f * dt;
        gz *= 0.5f * dt;

        float nq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz);
        float nq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy);
        float nq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx);
        float nq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx);

        // 归一化四元数
        float recipNorm = 1.0f / std::sqrt(nq0*nq0 + nq1*nq1 + nq2*nq2 + nq3*nq3);
        q0 = nq0 * recipNorm;
        q1 = nq1 * recipNorm;
        q2 = nq2 * recipNorm;
        q3 = nq3 * recipNorm;
    }

    /**
     * 获取当前四元数副本
     */
    void getQuaternion(float &w, float &x, float &y, float &z) const {
        w = q0; x = q1; y = q2; z = q3;
    }

    /**
     * 获取欧拉角（单位：度）
     * @param roll  滚转角 (-180..180)
     * @param pitch 俯仰角 (-90..90)
     * @param yaw   偏航角 (-180..180)
     */
    void getEuler(float &roll, float &pitch, float &yaw) const {
        // 从四元数计算欧拉角
        roll  = std::atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 57.295779513f;
        pitch = std::asinf(2.0f*(q0*q2 - q3*q1)) * 57.295779513f;
        yaw   = std::atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 57.295779513f;
    }

private:
    float q0, q1, q2, q3;          // 四元数
    float Kp, Ki;                  // PI 增益
    float sampleFreq;              // 采样频率 (Hz)
    float integralFBx, integralFBy, integralFBz;  // 积分项
};

#endif //MC_BOARD_02_MAHONY_HPP
