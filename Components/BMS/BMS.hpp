/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BMS_HPP
#define FINEMOTE_BMS_HPP

#include <cstddef>
#include <cstdint>
#include <mqueue.h>
#include <pthread.h>

#include "BMS_BatteryEstimator.hpp"
#include "BMS_Types.hpp"

class BMS
{
public:
    static constexpr size_t MaximumCurvePoints = 720U;

    static BMS& GetInstance();

    BMS(const BMS&) = delete;
    BMS& operator=(const BMS&) = delete;

    PlatformErr Start(const BMS_DriverCallbacks& callbacks,
                      const BMS_Config& config = {});
    PlatformErr SubmitSample(const BMS_Sample& sample);

    bool GetLatestSample(BMS_Sample& sample) const;
    bool GetStatus(BMS_Status& status) const;
    bool GetPowerStatistics(BMS_PowerStatistics& statistics) const;
    bool GetBatteryEstimate(BMS_BatteryEstimate& estimate) const;
    size_t GetChargeCurve(BMS_ChargeCurvePoint* points, size_t maximumPoints) const;
    size_t GetPowerCurve(BMS_PowerCurvePoint* points, size_t maximumPoints) const;

private:
    BMS() = default;

    static void* TaskEntry(void* context);
    static BMS_BatteryProfile DetectBatteryProfile(const BMS_AdsMeasurements& values);
    static BMS_ChargeState DecodeChargeState(uint8_t pcfValue);
    static BQ25756_ChargerConfig MakeProfileConfig(BMS_BatteryProfile profile);

    void Run();
    void ProcessSample(const BMS_Sample& sample);
    void UpdateChargeState(const BMS_Sample& sample);
    void UpdatePower(const BMS_Sample& sample);
    void UpdateBatteryEstimate(const BMS_Sample& sample);
    void UpdateChargingPolicy(const BMS_Sample& sample);
    void AddChargeCurvePoint(const BMS_Sample& sample);
    void AddPowerCurvePoint(const BMS_Sample& sample, float power);
    void LogStatus(const BMS_Sample& sample);
    void SetChargeStatusOutput(bool charging);

    mutable pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
    mqd_t sampleReceiveQueue_ = (mqd_t)-1;
    mqd_t sampleSendQueue_ = (mqd_t)-1;
    mqd_t sampleDropQueue_ = (mqd_t)-1;
    pthread_t task_ {};
    BMS_DriverCallbacks callbacks_ {};
    BMS_Config config_ {};

    BMS_Sample latestSample_ {};
    BMS_Status status_ {};
    BMS_PowerStatistics powerStatistics_ {};
    BMS_BatteryEstimator batteryEstimator_ {};

    BMS_ChargeCurvePoint chargeCurve_[MaximumCurvePoints] {};
    BMS_PowerCurvePoint powerCurve_[MaximumCurvePoints] {};
    size_t chargeCurveHead_ = 0U;
    size_t chargeCurveCount_ = 0U;
    size_t powerCurveHead_ = 0U;
    size_t powerCurveCount_ = 0U;

    uint32_t chargeStartTimestampMs_ = 0U;
    uint32_t lastChargeCurveTimestampMs_ = 0U;
    uint32_t lastPowerCurveTimestampMs_ = 0U;
    uint32_t lastPowerTimestampMs_ = 0U;
    uint32_t lastChargeAttemptTimestampMs_ = 0U;
    uint32_t lastWatchdogResetTimestampMs_ = 0U;
    uint32_t lastStatusLogTimestampMs_ = 0U;
    bool chargeCurveRecording_ = false;
    bool hasPowerTimestamp_ = false;
};

#endif // FINEMOTE_BMS_HPP
