/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_ANGLESENSORBASE_HPP
#define FINEMOTE_ANGLESENSORBASE_HPP

#include "DeviceBase/DeviceBase.hpp"

/**
 * @brief 角度传感器类
 * @tparam zeroPos 编码器零点:输入标准为【0-360】度
 * @tparam divisionFactor 分频系数
 */

class AngleSensorBase: public DeviceBase {
public:
    /**
     * @brief 构造 AngleSensorBase 对象
     * @param zeroPos 编码器零点，单位为度（输入可在 [0,360] 范围）。
     *                构造函数会将该值归一化到 (-180,180] 范围并作为内部零点保存。
     * @param divisionFactor 可选的分频/预分频系数，会转发给基类 DeviceBase。
     */
    explicit AngleSensorBase(float zeroPos,uint32_t divisionFactor = 1) :
    DeviceBase(divisionFactor) {
        while (zeroPos < -180)
            zeroPos += 360;
        while (zeroPos >= 180)
            zeroPos -= 360;
        zeroPosition = zeroPos;
    }

    /**
     * @brief 获取归一化到 [0, 360] 范围的角度
     * 返回值为原始角度按配置的零点偏移后映射到闭区间 [0,360] 的结果。
     * @return float 返回的角度，单位为度，范围为 [0,360]
     */
    [[nodiscard]] float GetAngle_360() const {
        if (angle < zeroPosition) {
            return angle + 540 - zeroPosition;
        }
        return angle - zeroPosition + 180;
    }

    /**
     * @brief 获取归一化到 [-180, 180] 范围的角度
     * 返回值为原始角度按配置的零点偏移后映射到对称区间 [-180,180] 的结果。
     * @return float 返回的角度，单位为度，范围为 [-180,180]
     */
    [[nodiscard]] float GetAngle() const {
        if (angle < zeroPosition) {
            return angle + 360 - zeroPosition;
        }
        return angle - zeroPosition;
    }

protected:
    float angle = 0;
    float zeroPosition = 0;
};



#endif
