/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "BMS_BatteryEstimator.hpp"

#include <cstddef>

namespace
{
constexpr double MaximumIntegrationIntervalSeconds = 5.0;
constexpr float CellCount = 6.0F;
constexpr float MinimumValidPackVoltage = 18.0F;
constexpr float MaximumValidPackVoltage = 26.0F;

constexpr float CellVoltages[] {
    3.00F, 3.20F, 3.40F, 3.50F, 3.60F,
    3.70F, 3.80F, 3.90F, 4.00F, 4.20F,
};
constexpr float SocPercentages[] {
    0.0F, 5.0F, 15.0F, 30.0F, 50.0F,
    65.0F, 80.0F, 92.0F, 97.0F, 100.0F,
};
constexpr size_t SocPointCount = sizeof(CellVoltages) / sizeof(CellVoltages[0]);
}

void BMS_BatteryEstimator::Reset()
{
    estimate_ = {};
    hasPrevious_ = false;
    previousCharging_ = false;
    previousStateValid_ = false;
    previousCurrentValid_ = false;
    previousTimestampMs_ = 0U;
    previousCurrent_ = 0.0F;
}

void BMS_BatteryEstimator::InvalidateSample()
{
    estimate_.sampleValid = false;
    estimate_.batteryCurrentValid = false;
    estimate_.voltageSocValid = false;
    estimate_.chargeStateValid = false;
    hasPrevious_ = false;
    previousCurrentValid_ = false;
    previousStateValid_ = false;
}

void BMS_BatteryEstimator::Update(uint32_t timestampMs,
                                  float batteryVoltage,
                                  float batteryCurrent,
                                  bool batteryCurrentValid,
                                  bool charging,
                                  bool chargeStateValid)
{
    estimate_.batteryVoltage = batteryVoltage;
    estimate_.batteryCurrent = batteryCurrentValid ? batteryCurrent : 0.0F;
    estimate_.batteryCurrentValid = batteryCurrentValid;
    estimate_.voltageSocValid = batteryVoltage >= MinimumValidPackVoltage &&
                                batteryVoltage <= MaximumValidPackVoltage;
    estimate_.voltageSocPercent = estimate_.voltageSocValid
                                      ? VoltageToSocPercent(batteryVoltage)
                                      : 0.0F;
    estimate_.charging = charging;
    estimate_.chargeStateValid = chargeStateValid;
    estimate_.sampleValid = true;
    estimate_.method = batteryCurrentValid
                           ? BMS_EstimateMethod::SessionCharge
                           : BMS_EstimateMethod::Uncalibrated;

    if (charging && chargeStateValid &&
        (!previousCharging_ || !previousStateValid_)) {
        estimate_.sessionChargedAh = 0.0;
    }

    if (hasPrevious_ && batteryCurrentValid && previousCurrentValid_ &&
        chargeStateValid && previousStateValid_) {
        const double elapsedSeconds =
            static_cast<double>(timestampMs - previousTimestampMs_) / 1000.0;
        if (elapsedSeconds > 0.0 &&
            elapsedSeconds <= MaximumIntegrationIntervalSeconds) {
            const double averageCurrent =
                (static_cast<double>(previousCurrent_) + batteryCurrent) * 0.5;
            const double deltaAh = averageCurrent * elapsedSeconds / 3600.0;
            estimate_.bootNetAh += deltaAh;
            if (deltaAh >= 0.0) {
                estimate_.bootChargedAh += deltaAh;
            } else {
                estimate_.bootDischargedAh -= deltaAh;
            }
            if (charging && averageCurrent > 0.0) {
                estimate_.sessionChargedAh += averageCurrent * elapsedSeconds / 3600.0;
            }
            estimate_.integratedSeconds += elapsedSeconds;
        }
    }

    hasPrevious_ = true;
    previousTimestampMs_ = timestampMs;
    previousCurrent_ = batteryCurrent;
    previousCurrentValid_ = batteryCurrentValid;
    previousCharging_ = charging;
    previousStateValid_ = chargeStateValid;
}

float BMS_BatteryEstimator::VoltageToSocPercent(float packVoltage)
{
    const float cellVoltage = packVoltage / CellCount;
    if (cellVoltage <= CellVoltages[0]) return 0.0F;
    if (cellVoltage >= CellVoltages[SocPointCount - 1U]) return 100.0F;

    for (size_t index = 1U; index < SocPointCount; ++index) {
        if (cellVoltage < CellVoltages[index]) {
            const float ratio =
                (cellVoltage - CellVoltages[index - 1U]) /
                (CellVoltages[index] - CellVoltages[index - 1U]);
            return SocPercentages[index - 1U] +
                   ratio * (SocPercentages[index] - SocPercentages[index - 1U]);
        }
    }
    return 100.0F;
}
