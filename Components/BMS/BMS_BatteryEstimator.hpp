/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BMS_BATTERY_ESTIMATOR_HPP
#define FINEMOTE_BMS_BATTERY_ESTIMATOR_HPP

#include <cstdint>

#include "BMS_Types.hpp"

class BMS_BatteryEstimator
{
public:
    void Reset();
    void InvalidateSample();
    void Update(uint32_t timestampMs,
                float batteryVoltage,
                float batteryCurrent,
                bool batteryCurrentValid,
                bool charging,
                bool chargeStateValid);

    [[nodiscard]] const BMS_BatteryEstimate& Estimate() const
    {
        return estimate_;
    }

private:
    static float VoltageToSocPercent(float packVoltage);

    BMS_BatteryEstimate estimate_ {};
    bool hasPrevious_ = false;
    bool previousCharging_ = false;
    bool previousStateValid_ = false;
    bool previousCurrentValid_ = false;
    uint32_t previousTimestampMs_ = 0U;
    float previousCurrent_ = 0.0F;
};

#endif // FINEMOTE_BMS_BATTERY_ESTIMATOR_HPP
