/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "BMS.hpp"

#include <algorithm>
#include <cerrno>
#include <fcntl.h>

#include "esp_log.h"

namespace
{
constexpr char Tag[] = "BMS";
constexpr size_t TaskStackSize = 6144U;
constexpr long SampleQueueLength = 16;
constexpr char SampleQueueName[] = "/bms_samples";
constexpr uint32_t MaximumEnergyIntervalMs = 5000U;

class MutexLock
{
public:
    explicit MutexLock(pthread_mutex_t& mutex)
        : mutex_(mutex), locked_(pthread_mutex_lock(&mutex_) == 0)
    {
    }

    ~MutexLock()
    {
        if (locked_) (void)pthread_mutex_unlock(&mutex_);
    }

    [[nodiscard]] bool Locked() const { return locked_; }

private:
    pthread_mutex_t& mutex_;
    bool locked_;
};

void CloseSampleQueues(mqd_t& receiveQueue, mqd_t& sendQueue, mqd_t& dropQueue)
{
    if (dropQueue != (mqd_t)-1) {
        (void)mq_close(dropQueue);
        dropQueue = (mqd_t)-1;
    }
    if (sendQueue != (mqd_t)-1) {
        (void)mq_close(sendQueue);
        sendQueue = (mqd_t)-1;
    }
    if (receiveQueue != (mqd_t)-1) {
        (void)mq_close(receiveQueue);
        receiveQueue = (mqd_t)-1;
    }
    (void)mq_unlink(SampleQueueName);
}
}

BMS& BMS::GetInstance()
{
    static BMS instance;
    return instance;
}

PlatformErr BMS::Start(const BMS_DriverCallbacks& callbacks, const BMS_Config& config)
{
    MutexLock lock(mutex_);
    if (!lock.Locked()) return PLATFORM_FAIL;
    if (status_.started) return PLATFORM_ALREADY_INIT;
    if (callbacks.initializeChargeStatusOutput == nullptr ||
        callbacks.setChargeStatusOutput == nullptr ||
        callbacks.configureCharger == nullptr || callbacks.startCharging == nullptr ||
        callbacks.stopCharging == nullptr || callbacks.resetWatchdog == nullptr ||
        config.chargeStatusGpio < 0) {
        return PLATFORM_INVALID_PARAM;
    }

    const mq_attr queueAttributes {
        .mq_flags = 0,
        .mq_maxmsg = SampleQueueLength,
        .mq_msgsize = static_cast<long>(sizeof(BMS_Sample)),
        .mq_curmsgs = 0,
    };
    sampleReceiveQueue_ =
        mq_open(SampleQueueName, O_CREAT | O_EXCL | O_RDONLY, 0600, &queueAttributes);
    if (sampleReceiveQueue_ != (mqd_t)-1) {
        sampleSendQueue_ = mq_open(SampleQueueName, O_WRONLY | O_NONBLOCK);
        sampleDropQueue_ = mq_open(SampleQueueName, O_RDONLY | O_NONBLOCK);
    }
    if (sampleReceiveQueue_ == (mqd_t)-1 || sampleSendQueue_ == (mqd_t)-1 ||
        sampleDropQueue_ == (mqd_t)-1) {
        const int queueError = errno;
        if (sampleReceiveQueue_ != (mqd_t)-1) {
            CloseSampleQueues(sampleReceiveQueue_, sampleSendQueue_, sampleDropQueue_);
        }
        ESP_LOGE(Tag, "Failed to create BMS sample queue: errno=%d", queueError);
        return PLATFORM_NO_MEMORY;
    }

    if (callbacks.initializeChargeStatusOutput(callbacks.context,
                                               config.chargeStatusGpio) != PLATFORM_OK) {
        CloseSampleQueues(sampleReceiveQueue_, sampleSendQueue_, sampleDropQueue_);
        ESP_LOGE(Tag, "Failed to initialize charge status GPIO %d", config.chargeStatusGpio);
        return PLATFORM_FAIL;
    }

    callbacks_ = callbacks;
    config_ = config;

    pthread_attr_t threadAttributes;
    if (pthread_attr_init(&threadAttributes) != 0) {
        CloseSampleQueues(sampleReceiveQueue_, sampleSendQueue_, sampleDropQueue_);
        ESP_LOGE(Tag, "Failed to initialize BMS pthread attributes");
        return PLATFORM_NO_RESOURCE;
    }
    if (pthread_attr_setdetachstate(&threadAttributes, PTHREAD_CREATE_DETACHED) != 0 ||
        pthread_attr_setstacksize(&threadAttributes, TaskStackSize) != 0) {
        (void)pthread_attr_destroy(&threadAttributes);
        CloseSampleQueues(sampleReceiveQueue_, sampleSendQueue_, sampleDropQueue_);
        ESP_LOGE(Tag, "Failed to configure BMS pthread");
        return PLATFORM_NO_RESOURCE;
    }

    status_.started = true;
    const int createResult = pthread_create(&task_, &threadAttributes, TaskEntry, this);
    (void)pthread_attr_destroy(&threadAttributes);
    if (createResult != 0) {
        status_.started = false;
        task_ = {};
        CloseSampleQueues(sampleReceiveQueue_, sampleSendQueue_, sampleDropQueue_);
        ESP_LOGE(Tag, "Failed to create BMS pthread: err=%d", createResult);
        return PLATFORM_NO_RESOURCE;
    }

    ESP_LOGI(Tag, "BMS pthread started");
    return PLATFORM_OK;
}

PlatformErr BMS::SubmitSample(const BMS_Sample& sample)
{
    {
        MutexLock lock(mutex_);
        if (!lock.Locked()) return PLATFORM_FAIL;
        if (!status_.started || sampleSendQueue_ == (mqd_t)-1) {
            return PLATFORM_NOT_INITIALIZED;
        }
    }
    if (mq_send(sampleSendQueue_,
                reinterpret_cast<const char*>(&sample),
                sizeof(sample),
                0U) == 0) {
        return PLATFORM_OK;
    }
    if (errno != EAGAIN) return PLATFORM_BUSY;

    BMS_Sample discarded {};
    const bool discardedOldSample =
        mq_receive(sampleDropQueue_,
                   reinterpret_cast<char*>(&discarded),
                   sizeof(discarded),
                   nullptr) >= 0;
    if (!discardedOldSample && errno != EAGAIN) return PLATFORM_BUSY;

    if (mq_send(sampleSendQueue_,
                reinterpret_cast<const char*>(&sample),
                sizeof(sample),
                0U) != 0) {
        return PLATFORM_BUSY;
    }

    if (discardedOldSample) {
        MutexLock lock(mutex_);
        if (lock.Locked()) ++status_.droppedSamples;
    }
    return PLATFORM_OK;
}

bool BMS::GetLatestSample(BMS_Sample& sample) const
{
    MutexLock lock(mutex_);
    if (!lock.Locked()) return false;
    sample = latestSample_;
    return true;
}

bool BMS::GetStatus(BMS_Status& status) const
{
    MutexLock lock(mutex_);
    if (!lock.Locked()) return false;
    status = status_;
    return true;
}

bool BMS::GetPowerStatistics(BMS_PowerStatistics& statistics) const
{
    MutexLock lock(mutex_);
    if (!lock.Locked()) return false;
    statistics = powerStatistics_;
    return true;
}

bool BMS::GetBatteryEstimate(BMS_BatteryEstimate& estimate) const
{
    MutexLock lock(mutex_);
    if (!lock.Locked()) return false;
    estimate = batteryEstimator_.Estimate();
    return true;
}

size_t BMS::GetChargeCurve(BMS_ChargeCurvePoint* points, size_t maximumPoints) const
{
    if (points == nullptr || maximumPoints == 0U) return 0U;
    MutexLock lock(mutex_);
    if (!lock.Locked()) return 0U;

    const size_t count = std::min(chargeCurveCount_, maximumPoints);
    const size_t start =
        (chargeCurveHead_ + MaximumCurvePoints - count) % MaximumCurvePoints;
    for (size_t index = 0U; index < count; ++index) {
        points[index] = chargeCurve_[(start + index) % MaximumCurvePoints];
    }
    return count;
}

size_t BMS::GetPowerCurve(BMS_PowerCurvePoint* points, size_t maximumPoints) const
{
    if (points == nullptr || maximumPoints == 0U) return 0U;
    MutexLock lock(mutex_);
    if (!lock.Locked()) return 0U;

    const size_t count = std::min(powerCurveCount_, maximumPoints);
    const size_t start =
        (powerCurveHead_ + MaximumCurvePoints - count) % MaximumCurvePoints;
    for (size_t index = 0U; index < count; ++index) {
        points[index] = powerCurve_[(start + index) % MaximumCurvePoints];
    }
    return count;
}

void* BMS::TaskEntry(void* context)
{
    static_cast<BMS*>(context)->Run();
    return nullptr;
}

void BMS::Run()
{
    for (;;) {
        BMS_Sample sample {};
        if (mq_receive(sampleReceiveQueue_,
                       reinterpret_cast<char*>(&sample),
                       sizeof(sample),
                       nullptr) == static_cast<ssize_t>(sizeof(sample))) {
            ProcessSample(sample);
        }
    }
}

void BMS::ProcessSample(const BMS_Sample& sample)
{
    MutexLock lock(mutex_);
    if (!lock.Locked()) return;

    latestSample_ = sample;
    status_.validSources = sample.validMask;
    UpdateChargeState(sample);
    UpdatePower(sample);
    UpdateBatteryEstimate(sample);
    UpdateChargingPolicy(sample);
    AddChargeCurvePoint(sample);
    LogStatus(sample);
}

void BMS::UpdateChargeState(const BMS_Sample& sample)
{
    if ((sample.updatedMask & BMS_SourcePcf8574) == 0U) return;

    if ((sample.validMask & BMS_SourcePcf8574) == 0U) {
        status_.chargeState = BMS_ChargeState::Unknown;
        SetChargeStatusOutput(false);
        return;
    }

    const BMS_ChargeState newState = DecodeChargeState(sample.pcf8574);
    const bool wasRecording = chargeCurveRecording_;
    status_.chargeState = newState;
    const bool charging = newState == BMS_ChargeState::Charging;
    SetChargeStatusOutput(charging);

    if (charging && !wasRecording) {
        chargeCurveHead_ = 0U;
        chargeCurveCount_ = 0U;
        chargeStartTimestampMs_ = sample.timestampMs;
        lastChargeCurveTimestampMs_ = sample.timestampMs - config_.curveSampleIntervalMs;
        chargeCurveRecording_ = true;
    } else if (newState == BMS_ChargeState::Completed ||
               newState == BMS_ChargeState::NotCharging ||
               newState == BMS_ChargeState::Fault) {
        chargeCurveRecording_ = false;
    }
}

void BMS::UpdatePower(const BMS_Sample& sample)
{
    if ((sample.updatedMask & BMS_SourceAds1115) == 0U ||
        (sample.validMask & BMS_SourceAds1115) == 0U) {
        return;
    }

    float power = sample.ads.outputVoltage * sample.ads.outputCurrent;
    if (power < 0.0F) power = 0.0F;
    const bool chargeStateValid = status_.chargeState != BMS_ChargeState::Unknown;
    const bool charging = status_.chargeState == BMS_ChargeState::Charging;

    if (hasPowerTimestamp_) {
        const uint32_t elapsedMs = sample.timestampMs - lastPowerTimestampMs_;
        if (elapsedMs > 0U && elapsedMs <= MaximumEnergyIntervalMs) {
            const double energyWh =
                static_cast<double>(power) * static_cast<double>(elapsedMs) / 3600000.0;
            if (!chargeStateValid) {
                powerStatistics_.unknownStateEnergyWh += energyWh;
            } else if (charging) {
                powerStatistics_.chargingEnergyWh += energyWh;
            } else {
                powerStatistics_.nonChargingEnergyWh += energyWh;
            }
        }
    }

    hasPowerTimestamp_ = true;
    lastPowerTimestampMs_ = sample.timestampMs;
    powerStatistics_.sampleValid = true;
    powerStatistics_.chargeStateValid = chargeStateValid;
    powerStatistics_.charging = charging;
    powerStatistics_.currentPowerW = power;

    if (powerCurveCount_ == 0U ||
        sample.timestampMs - lastPowerCurveTimestampMs_ >= config_.curveSampleIntervalMs) {
        AddPowerCurvePoint(sample, power);
        lastPowerCurveTimestampMs_ = sample.timestampMs;
    }
}

void BMS::UpdateBatteryEstimate(const BMS_Sample& sample)
{
    if ((sample.updatedMask & BMS_SourceBq25756) == 0U) return;

    if ((sample.validMask & BMS_SourceBq25756) == 0U ||
        (sample.charger.validFields & BQ25756_MeasurementBatteryVoltage) == 0U) {
        batteryEstimator_.InvalidateSample();
        return;
    }

    const bool chargeStateValid = status_.chargeState != BMS_ChargeState::Unknown;
    const bool batteryCurrentValid =
        (sample.charger.validFields & BQ25756_MeasurementBatteryCurrent) != 0U;
    batteryEstimator_.Update(sample.timestampMs,
                             sample.charger.batteryVoltage,
                             sample.charger.batteryCurrent,
                             batteryCurrentValid,
                             status_.chargeState == BMS_ChargeState::Charging,
                             chargeStateValid);
}

void BMS::UpdateChargingPolicy(const BMS_Sample& sample)
{
    if ((sample.validMask & BMS_SourceAds1115) == 0U) return;

    const BMS_BatteryProfile detected = DetectBatteryProfile(sample.ads);
    status_.detectedProfile = detected;
    if (detected == BMS_BatteryProfile::None) {
        status_.configuredProfile = BMS_BatteryProfile::None;
        return;
    }
    if ((sample.validMask & BMS_SourceBq25756) == 0U) return;

    const bool standby = status_.chargeState == BMS_ChargeState::NotCharging;
    if ((detected == BMS_BatteryProfile::Cells3 || standby) &&
        detected != status_.configuredProfile &&
        (lastChargeAttemptTimestampMs_ == 0U ||
         sample.timestampMs - lastChargeAttemptTimestampMs_ >=
             config_.chargeRetryIntervalMs)) {
        lastChargeAttemptTimestampMs_ = sample.timestampMs;
        const BQ25756_ChargerConfig chargerConfig = MakeProfileConfig(detected);
        PlatformErr result = callbacks_.configureCharger(callbacks_.context, chargerConfig);
        if (result == PLATFORM_OK) {
            result = callbacks_.startCharging(callbacks_.context);
        }
        status_.lastDriverResult = result;
        if (result == PLATFORM_OK) {
            status_.configuredProfile = detected;
            lastWatchdogResetTimestampMs_ = sample.timestampMs;
            ESP_LOGI(Tag,
                     "Charge profile applied: %s",
                     detected == BMS_BatteryProfile::Cells6 ? "6S" : "3S");
        } else {
            ESP_LOGE(Tag, "Failed to apply charge profile: err=%d", static_cast<int>(result));
        }
        return;
    }

    if (detected == BMS_BatteryProfile::Cells3 &&
        status_.configuredProfile == BMS_BatteryProfile::Cells3 &&
        sample.timestampMs - lastWatchdogResetTimestampMs_ >=
            config_.watchdogResetIntervalMs) {
        lastWatchdogResetTimestampMs_ = sample.timestampMs;
        status_.lastDriverResult = callbacks_.resetWatchdog(callbacks_.context);
        if (status_.lastDriverResult != PLATFORM_OK) {
            ESP_LOGE(Tag,
                     "Failed to reset charger watchdog: err=%d",
                     static_cast<int>(status_.lastDriverResult));
        }
    }
}

void BMS::AddChargeCurvePoint(const BMS_Sample& sample)
{
    if (!chargeCurveRecording_ ||
        (sample.updatedMask & BMS_SourceBq25756) == 0U ||
        (sample.validMask & BMS_SourceBq25756) == 0U ||
        (sample.charger.validFields & BQ25756_MeasurementBatteryVoltage) == 0U ||
        (sample.charger.validFields & BQ25756_MeasurementBatteryCurrent) == 0U ||
        sample.timestampMs - lastChargeCurveTimestampMs_ < config_.curveSampleIntervalMs) {
        return;
    }

    lastChargeCurveTimestampMs_ = sample.timestampMs;
    chargeCurve_[chargeCurveHead_] = {
        (sample.timestampMs - chargeStartTimestampMs_) / 1000U,
        sample.charger.batteryVoltage,
        sample.charger.batteryCurrent,
    };
    chargeCurveHead_ = (chargeCurveHead_ + 1U) % MaximumCurvePoints;
    if (chargeCurveCount_ < MaximumCurvePoints) ++chargeCurveCount_;
}

void BMS::AddPowerCurvePoint(const BMS_Sample& sample, float power)
{
    powerCurve_[powerCurveHead_] = {
        sample.timestampMs / 1000U,
        sample.ads.outputVoltage,
        sample.ads.outputCurrent,
        power,
    };
    powerCurveHead_ = (powerCurveHead_ + 1U) % MaximumCurvePoints;
    if (powerCurveCount_ < MaximumCurvePoints) ++powerCurveCount_;
}

void BMS::LogStatus(const BMS_Sample& sample)
{
    if (sample.timestampMs - lastStatusLogTimestampMs_ < config_.statusLogIntervalMs) return;
    lastStatusLogTimestampMs_ = sample.timestampMs;
    ESP_LOGI(Tag,
             "sources=0x%02X profile=%u charge=%u power=%.2fW VBAT=%.2fV IBAT=%.2fA drops=%u",
             static_cast<unsigned>(sample.validMask),
             static_cast<unsigned>(status_.detectedProfile),
             static_cast<unsigned>(status_.chargeState),
             static_cast<double>(powerStatistics_.currentPowerW),
             static_cast<double>(sample.charger.batteryVoltage),
             static_cast<double>(sample.charger.batteryCurrent),
             static_cast<unsigned>(status_.droppedSamples));
}

void BMS::SetChargeStatusOutput(bool charging)
{
    if (callbacks_.setChargeStatusOutput(callbacks_.context,
                                         config_.chargeStatusGpio,
                                         charging) != PLATFORM_OK) {
        ESP_LOGE(Tag, "Failed to update charge status GPIO");
    }
}

BMS_BatteryProfile BMS::DetectBatteryProfile(const BMS_AdsMeasurements& values)
{
    if (values.input1Voltage > 10.0F && values.input2Voltage > 18.0F) {
        return BMS_BatteryProfile::Cells6;
    }
    if (values.input1Voltage > 10.0F &&
        values.input2Voltage > 9.0F && values.input2Voltage < 15.0F) {
        return BMS_BatteryProfile::Cells3;
    }
    return BMS_BatteryProfile::None;
}

BMS_ChargeState BMS::DecodeChargeState(uint8_t pcfValue)
{
    const uint8_t stat1 = static_cast<uint8_t>((pcfValue >> 5U) & 0x01U);
    const uint8_t stat2 = static_cast<uint8_t>((pcfValue >> 4U) & 0x01U);
    if (stat1 == 0U && stat2 == 1U) return BMS_ChargeState::Charging;
    if (stat1 == 1U && stat2 == 0U) return BMS_ChargeState::Completed;
    if (stat1 == 1U && stat2 == 1U) return BMS_ChargeState::NotCharging;
    return BMS_ChargeState::Fault;
}

BQ25756_ChargerConfig BMS::MakeProfileConfig(BMS_BatteryProfile profile)
{
    BQ25756_ChargerConfig config {};
    if (profile == BMS_BatteryProfile::Cells3) {
        config.chargeVoltageMv = 12200U;
        config.chargeCurrentMa = 2500U;
        config.inputCurrentLimitMa = 4500U;
        config.inputVoltageLimitMv = 21600U;
        config.prechargeCurrentMa = 500U;
        config.terminationCurrentMa = 2500U;
        config.watchdogTimer = 1U;
    } else {
        config.chargeVoltageMv = 25000U;
        config.chargeCurrentMa = 10000U;
        config.inputCurrentLimitMa = 12000U;
        config.inputVoltageLimitMv = 21600U;
        config.prechargeCurrentMa = 1600U;
        config.terminationCurrentMa = 2000U;
        config.watchdogTimer = 0U;
    }
    return config;
}
