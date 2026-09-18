/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BQ25756_HPP
#define FINEMOTE_BQ25756_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "BQ25756_Registers.hpp"
#include "BQ25756_Types.hpp"
#include "BSP_POSIX.h"
#include "Bus/I2C_Base.hpp"
#include "DeviceBase/DeviceBase.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

class BQ25756_OperationMutex {
public:
    BQ25756_OperationMutex();
    BQ25756_OperationMutex(const BQ25756_OperationMutex&) = delete;
    BQ25756_OperationMutex& operator=(const BQ25756_OperationMutex&) = delete;

    void lock();
    void unlock();

private:
    pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
};

class BQ25756_OperationGuard {
public:
    explicit BQ25756_OperationGuard(BQ25756_OperationMutex& mutex);
    ~BQ25756_OperationGuard();

    BQ25756_OperationGuard(const BQ25756_OperationGuard&) = delete;
    BQ25756_OperationGuard& operator=(const BQ25756_OperationGuard&) = delete;

private:
    BQ25756_OperationMutex& mutex_;
};

inline BQ25756_OperationMutex::BQ25756_OperationMutex() = default;

inline void BQ25756_OperationMutex::lock() {
    (void)pthread_mutex_lock(&mutex_);
}

inline void BQ25756_OperationMutex::unlock() {
    (void)pthread_mutex_unlock(&mutex_);
}

inline BQ25756_OperationGuard::BQ25756_OperationGuard(BQ25756_OperationMutex& mutex): mutex_(mutex) {
    mutex_.lock();
}

inline BQ25756_OperationGuard::~BQ25756_OperationGuard() {
    mutex_.unlock();
}

template<size_t BusId>
class BQ25756 final: public DeviceBase {
public:
    using SampleCallback = void (*)(void* context, PlatformErr result, const BQ25756_Measurements& measurements);

    explicit BQ25756(const BQ25756_DeviceConfig& config = {}, uint32_t divisionFactor = 1U);

    BQ25756(const BQ25756&) = delete;
    BQ25756& operator=(const BQ25756&) = delete;

    PlatformErr Configure(const BQ25756_ChargerConfig& config);

    PlatformErr InitializeMeasurements();

    PlatformErr StartCharging();

    PlatformErr StopCharging();

    PlatformErr SetChargeVoltage(uint16_t voltageMv);

    PlatformErr SetFastChargeCurrent(uint16_t currentMa);

    PlatformErr SetInputCurrentLimit(uint16_t currentMa);

    PlatformErr SetInputVoltageLimit(uint16_t voltageMv);

    PlatformErr SetPrechargeCurrentLimit(uint16_t currentMa);

    PlatformErr SetTerminationCurrent(uint16_t currentMa);

    PlatformErr ConfigurePrechargeTermination(
        bool terminationEnabled,
        BQ25756_FastChargeThreshold threshold,
        bool prechargeEnabled
    );

    PlatformErr SetTopOffTimer(uint8_t timer);

    PlatformErr SetWatchdogTimer(uint8_t timer);

    PlatformErr ConfigureSafetyTimer(bool enabled, uint8_t timer, bool slowInDpm);

    PlatformErr ConfigureConstantVoltageTimer(uint8_t timer);

    PlatformErr SetAutoRechargeThreshold(BQ25756_AutoRechargeThreshold threshold);

    PlatformErr ResetWatchdog();

    PlatformErr SetCePinEnabled(bool enabled);

    PlatformErr SetChargeOnWatchdogExpiry(bool enabled);

    PlatformErr SetHighImpedance(bool enabled);

    PlatformErr SetBatteryLoad(bool enabled);

    PlatformErr SetChargeEnabled(bool enabled);

    PlatformErr ConfigurePins(bool ichgEnabled, bool ilimHizEnabled, bool powerGoodEnabled, bool statusEnabled);

    PlatformErr ResetRegisters();

    PlatformErr SetInputLoad(bool enabled);

    PlatformErr SetPfmMode(bool enabled);

    PlatformErr SetReverseMode(bool enabled);

    PlatformErr SetMpptEnabled(bool enabled);

    PlatformErr SetTemperaturePinEnabled(bool enabled);

    PlatformErr ConfigureAdc(
        bool enabled,
        bool oneShot,
        BQ25756_AdcSampleSpeed sampleSpeed,
        bool runningAverage,
        bool initializeAverage
    );

    PlatformErr ConfigureAdcChannels(
        bool inputCurrentEnabled,
        bool batteryCurrentEnabled,
        bool inputVoltageEnabled,
        bool batteryVoltageEnabled,
        bool temperatureEnabled,
        bool feedbackVoltageEnabled
    );

    PlatformErr ReadBatteryVoltage(float& voltageV);

    PlatformErr ReadChargeCurrent(float& currentA);

    PlatformErr ReadInput(float& voltageV, float& currentA);

    PlatformErr ReadBatteryTemperature(float& temperatureC);

    PlatformErr ReadFeedbackVoltage(float& voltageV);

    PlatformErr ReadChargeState(BQ25756_ChargeState& state);

    PlatformErr ReadFaultStatus(uint8_t& faultFlags);

    PlatformErr ReadWatchdogExpired(bool& expired);

    PlatformErr ReadMeasurements(BQ25756_Measurements& measurements);

    [[nodiscard]] const BQ25756_Measurements& Measurements() const;

    [[nodiscard]] PlatformErr GpioInitResult() const;

    [[nodiscard]] PlatformErr LastResult() const;

    void SetSampleCallback(SampleCallback callback, void* context = nullptr);

    void SetFailureRetryCycles(uint32_t cycles);

    void Update() final;

    void Handle() final;

private:
    static constexpr const char* Tag = "BQ25756";

    PlatformErr StoreResult(PlatformErr result);

    PlatformErr ConfigureMeasurementsUnlocked(const BQ25756_ChargerConfig& config);

    PlatformErr SetCurrentLimit(
        BQ25756_Registers::Register reg,
        uint16_t mask,
        uint16_t currentMa,
        uint16_t minimumMa,
        uint16_t maximumMa
    );

    PlatformErr SetRegisterFlag(BQ25756_Registers::Register reg, uint8_t mask, bool enabled);

    PlatformErr UpdateRegister8(BQ25756_Registers::Register reg, uint8_t mask, uint8_t value);

    PlatformErr UpdateRegister16(BQ25756_Registers::Register reg, uint16_t mask, uint16_t value);

    PlatformErr WriteRegister8(BQ25756_Registers::Register reg, uint8_t value);

    PlatformErr ReadRegister8(BQ25756_Registers::Register reg, uint8_t& value);

    PlatformErr WriteRegister16(BQ25756_Registers::Register reg, uint16_t value);

    PlatformErr ReadRegister16(BQ25756_Registers::Register reg, uint16_t& value);

    BQ25756_DeviceConfig config_;
    BQ25756_ChargerConfig chargerConfig_ {};
    BQ25756_OperationMutex operationMutex_;
    I2C_Agent<BusId> i2cAgent_;
    BQ25756_Measurements measurements_ {};
    PlatformErr gpioInitResult_ = PLATFORM_NOT_INITIALIZED;
    PlatformErr lastResult_ = PLATFORM_NOT_INITIALIZED;
    SampleCallback sampleCallback_ = nullptr;
    void* sampleContext_ = nullptr;
    BQ25756_Measurements scheduledMeasurements_ {};
    PlatformErr scheduledResult_ = PLATFORM_NOT_INITIALIZED;
    uint32_t failureRetryCycles_ = 0U;
    uint32_t retryCyclesRemaining_ = 0U;
    bool measurementInitializationPending_ = true;
    bool samplePending_ = false;
};

// Template method implementations

template<size_t BusId>
BQ25756<BusId>::BQ25756(const BQ25756_DeviceConfig& config, uint32_t divisionFactor):
    DeviceBase(divisionFactor),
    config_(config),
    i2cAgent_(config.i2c) {
    if (config_.readyPin < 0) {
        gpioInitResult_ = PLATFORM_INVALID_PARAM;
        ESP_EARLY_LOGE(Tag, "Invalid READY GPIO: %d", static_cast<int>(config_.readyPin));
        return;
    }

    gpio_config_t ioConfig {};
    ioConfig.pin_bit_mask = 1ULL << static_cast<uint32_t>(config_.readyPin);
    ioConfig.mode = GPIO_MODE_OUTPUT;
    ioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    ioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
    ioConfig.intr_type = GPIO_INTR_DISABLE;

    gpioInitResult_ = gpio_config(&ioConfig) == ESP_OK ? PLATFORM_OK : PLATFORM_FAIL;
    if (gpioInitResult_ == PLATFORM_OK && gpio_set_level(config_.readyPin, 0U) != ESP_OK) {
        gpioInitResult_ = PLATFORM_FAIL;
    }
    if (gpioInitResult_ == PLATFORM_OK) {
        ESP_EARLY_LOGI(
            Tag,
            "Initialized on I2C%u, address=0x%02X, READY GPIO=%d",
            static_cast<unsigned>(BusId),
            static_cast<unsigned>(config_.i2c.deviceAddress),
            static_cast<int>(config_.readyPin)
        );
    } else {
        ESP_EARLY_LOGE(Tag, "Failed to initialize READY GPIO %d", static_cast<int>(config_.readyPin));
    }
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::Configure(const BQ25756_ChargerConfig& config) {
    BQ25756_OperationGuard operationGuard(operationMutex_);
    PlatformErr result = SetChargeVoltage(config.chargeVoltageMv);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetFastChargeCurrent(config.chargeCurrentMa);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetPrechargeCurrentLimit(config.prechargeCurrentMa);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result =
        ConfigurePrechargeTermination(config.terminationEnabled, config.fastChargeThreshold, config.prechargeEnabled);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetTerminationCurrent(config.terminationCurrentMa);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetAutoRechargeThreshold(config.autoRechargeThreshold);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetInputCurrentLimit(config.inputCurrentLimitMa);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetInputVoltageLimit(config.inputVoltageLimitMv);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetTopOffTimer(config.topOffTimer);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetWatchdogTimer(config.watchdogTimer);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = ConfigureSafetyTimer(config.safetyTimerEnabled, config.safetyTimer, config.slowSafetyTimerInDpm);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = ConfigureConstantVoltageTimer(config.constantVoltageTimer);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetChargeOnWatchdogExpiry(config.chargeOnWatchdogExpiry);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetCePinEnabled(config.cePinEnabled);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = ConfigurePins(
        config.ichgPinEnabled,
        config.ilimHizPinEnabled,
        config.powerGoodPinEnabled,
        config.statusPinsEnabled
    );
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetHighImpedance(config.highImpedanceEnabled);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetBatteryLoad(config.batteryLoadEnabled);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetInputLoad(config.inputLoadEnabled);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetPfmMode(config.pfmEnabled);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetReverseMode(config.reverseModeEnabled);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetMpptEnabled(config.mpptEnabled);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = SetTemperaturePinEnabled(config.temperaturePinEnabled);
    if (result != PLATFORM_OK)
        return StoreResult(result);
    result = ConfigureMeasurementsUnlocked(config);
    if (result != PLATFORM_OK)
        return StoreResult(result);

    chargerConfig_ = config;
    measurementInitializationPending_ = false;
    ESP_LOGI(
        Tag,
        "Configured: charge=%umV/%umA, input limit=%umV/%umA",
        static_cast<unsigned>(config.chargeVoltageMv),
        static_cast<unsigned>(config.chargeCurrentMa),
        static_cast<unsigned>(config.inputVoltageLimitMv),
        static_cast<unsigned>(config.inputCurrentLimitMa)
    );
    return StoreResult(PLATFORM_OK);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::InitializeMeasurements() {
    BQ25756_OperationGuard operationGuard(operationMutex_);
    const PlatformErr result = ConfigureMeasurementsUnlocked(chargerConfig_);
    measurementInitializationPending_ = result != PLATFORM_OK;
    return StoreResult(result);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::StartCharging() {
    BQ25756_OperationGuard operationGuard(operationMutex_);
    PlatformErr result = UpdateRegister8(
        BQ25756_Registers::Register::ChargerControl,
        static_cast<uint8_t>(BQ25756_Registers::CePinDisable | BQ25756_Registers::ChargeEnable),
        BQ25756_Registers::ChargeEnable
    );
    if (result != PLATFORM_OK) {
        ESP_LOGE(Tag, "Failed to enable charging: err=%d", static_cast<int>(result));
        return StoreResult(result);
    }
    if (gpioInitResult_ != PLATFORM_OK || gpio_set_level(config_.readyPin, 1U) != ESP_OK) {
        ESP_LOGE(Tag, "Failed to assert READY GPIO %d", static_cast<int>(config_.readyPin));
        return StoreResult(PLATFORM_FAIL);
    }
    ESP_LOGI(Tag, "Charging started");
    return StoreResult(PLATFORM_OK);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::StopCharging() {
    BQ25756_OperationGuard operationGuard(operationMutex_);
    const bool gpioOk = gpioInitResult_ == PLATFORM_OK && gpio_set_level(config_.readyPin, 0U) == ESP_OK;
    const PlatformErr result =
        UpdateRegister8(BQ25756_Registers::Register::ChargerControl, BQ25756_Registers::ChargeEnable, 0U);
    if (result != PLATFORM_OK) {
        ESP_LOGE(Tag, "Failed to disable charging: err=%d", static_cast<int>(result));
        return StoreResult(result);
    }
    if (!gpioOk) {
        ESP_LOGE(Tag, "Failed to clear READY GPIO %d", static_cast<int>(config_.readyPin));
        return StoreResult(PLATFORM_FAIL);
    }
    ESP_LOGI(Tag, "Charging stopped");
    return StoreResult(PLATFORM_OK);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetChargeVoltage(uint16_t voltageMv) {
    float dividerRatio = 0.0F;
    if (voltageMv >= 12200U && voltageMv <= 12700U) {
        dividerRatio = config_.feedbackDividerRatio3S;
    } else if (voltageMv >= 24500U && voltageMv <= 25500U) {
        dividerRatio = config_.feedbackDividerRatio6S;
    } else {
        ESP_LOGE(Tag, "Unsupported charge voltage: %u mV", static_cast<unsigned>(voltageMv));
        return PLATFORM_INVALID_PARAM;
    }
    if (dividerRatio <= 0.0F) {
        ESP_LOGE(Tag, "Invalid feedback divider ratio");
        return PLATFORM_INVALID_PARAM;
    }
    float feedbackMv = static_cast<float>(voltageMv) / dividerRatio;
    if (feedbackMv < 1504.0F) {
        ESP_LOGW(
            Tag,
            "Charge voltage %u mV is below the hardware range; clamped to minimum",
            static_cast<unsigned>(voltageMv)
        );
        feedbackMv = 1504.0F;
    } else if (feedbackMv > 1566.0F) {
        ESP_LOGW(
            Tag,
            "Charge voltage %u mV is above the hardware range; clamped to maximum",
            static_cast<unsigned>(voltageMv)
        );
        feedbackMv = 1566.0F;
    }
    const uint16_t value = static_cast<uint16_t>((feedbackMv - 1504.0F) / 2.0F) & BQ25756_Registers::ChargeVoltageMask;
    return WriteRegister16(BQ25756_Registers::Register::ChargeVoltageLimit, value);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetFastChargeCurrent(uint16_t currentMa) {
    return SetCurrentLimit(
        BQ25756_Registers::Register::ChargeCurrentLimit,
        BQ25756_Registers::ChargeCurrentMask,
        currentMa,
        400U,
        20000U
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetInputCurrentLimit(uint16_t currentMa) {
    return SetCurrentLimit(
        BQ25756_Registers::Register::InputCurrentDpmLimit,
        BQ25756_Registers::InputCurrentMask,
        currentMa,
        400U,
        20000U
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetInputVoltageLimit(uint16_t voltageMv) {
    if (voltageMv < 4200U || voltageMv > 36000U) {
        ESP_LOGE(Tag, "Input voltage limit out of range: %u mV", static_cast<unsigned>(voltageMv));
        return PLATFORM_INVALID_PARAM;
    }
    const uint16_t value = static_cast<uint16_t>((voltageMv / 20U) << 2U);
    return UpdateRegister16(
        BQ25756_Registers::Register::InputVoltageDpmLimit,
        BQ25756_Registers::InputVoltageMask,
        value
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetPrechargeCurrentLimit(uint16_t currentMa) {
    return SetCurrentLimit(
        BQ25756_Registers::Register::PrechargeCurrentLimit,
        BQ25756_Registers::PrechargeCurrentMask,
        currentMa,
        250U,
        10000U
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetTerminationCurrent(uint16_t currentMa) {
    return SetCurrentLimit(
        BQ25756_Registers::Register::TerminationCurrentLimit,
        BQ25756_Registers::TerminationCurrentMask,
        currentMa,
        250U,
        10000U
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ConfigurePrechargeTermination(
    bool terminationEnabled,
    BQ25756_FastChargeThreshold threshold,
    bool prechargeEnabled
) {
    const uint8_t thresholdValue = static_cast<uint8_t>(threshold);
    if (thresholdValue > 3U) {
        return PLATFORM_INVALID_PARAM;
    }
    const uint8_t value = static_cast<uint8_t>(
        (terminationEnabled ? BQ25756_Registers::TerminationEnable : 0U) | (thresholdValue << 1U)
        | (prechargeEnabled ? BQ25756_Registers::PrechargeEnable : 0U)
    );
    return UpdateRegister8(
        BQ25756_Registers::Register::PrechargeTerminationControl,
        static_cast<uint8_t>(
            BQ25756_Registers::TerminationEnable | BQ25756_Registers::FastChargeThresholdMask
            | BQ25756_Registers::PrechargeEnable
        ),
        value
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetTopOffTimer(uint8_t timer) {
    if (timer > 3U) {
        ESP_LOGE(Tag, "Invalid top-off timer value: %u", static_cast<unsigned>(timer));
        return PLATFORM_INVALID_PARAM;
    }
    return UpdateRegister8(
        BQ25756_Registers::Register::TimerControl,
        BQ25756_Registers::TopOffTimerMask,
        static_cast<uint8_t>(timer << 6U)
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetWatchdogTimer(uint8_t timer) {
    if (timer > 3U) {
        ESP_LOGE(Tag, "Invalid watchdog timer value: %u", static_cast<unsigned>(timer));
        return PLATFORM_INVALID_PARAM;
    }
    return UpdateRegister8(
        BQ25756_Registers::Register::TimerControl,
        BQ25756_Registers::WatchdogTimerMask,
        static_cast<uint8_t>(timer << 4U)
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ConfigureSafetyTimer(bool enabled, uint8_t timer, bool slowInDpm) {
    if (timer > 3U) {
        ESP_LOGE(Tag, "Invalid safety timer value: %u", static_cast<unsigned>(timer));
        return PLATFORM_INVALID_PARAM;
    }
    const uint8_t value = static_cast<uint8_t>(
        (enabled ? BQ25756_Registers::SafetyTimerEnable : 0U) | (timer << 1U)
        | (slowInDpm ? BQ25756_Registers::SafetyTimerSlowInDpm : 0U)
    );
    return UpdateRegister8(
        BQ25756_Registers::Register::ThreeStageChargeControl,
        static_cast<uint8_t>(
            BQ25756_Registers::SafetyTimerEnable | BQ25756_Registers::SafetyTimerMask
            | BQ25756_Registers::SafetyTimerSlowInDpm
        ),
        value
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ConfigureConstantVoltageTimer(uint8_t timer) {
    if (timer > 3U) {
        ESP_LOGE(Tag, "Invalid constant voltage timer value: %u", static_cast<unsigned>(timer));
        return PLATFORM_INVALID_PARAM;
    }
    return UpdateRegister8(
        BQ25756_Registers::Register::ThreeStageChargeControl,
        BQ25756_Registers::ConstantVoltageTimerMask,
        timer
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetAutoRechargeThreshold(BQ25756_AutoRechargeThreshold threshold) {
    const uint8_t value = static_cast<uint8_t>(threshold);
    if (value > 3U)
        return PLATFORM_INVALID_PARAM;
    return UpdateRegister8(
        BQ25756_Registers::Register::ChargerControl,
        BQ25756_Registers::AutoRechargeThresholdMask,
        static_cast<uint8_t>(value << 6U)
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ResetWatchdog() {
    BQ25756_OperationGuard operationGuard(operationMutex_);
    return UpdateRegister8(
        BQ25756_Registers::Register::ChargerControl,
        BQ25756_Registers::WatchdogReset,
        BQ25756_Registers::WatchdogReset
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetCePinEnabled(bool enabled) {
    return SetRegisterFlag(BQ25756_Registers::Register::ChargerControl, BQ25756_Registers::CePinDisable, !enabled);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetChargeOnWatchdogExpiry(bool enabled) {
    return SetRegisterFlag(
        BQ25756_Registers::Register::ChargerControl,
        BQ25756_Registers::ChargeOnWatchdogExpiry,
        enabled
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetHighImpedance(bool enabled) {
    return SetRegisterFlag(
        BQ25756_Registers::Register::ChargerControl,
        BQ25756_Registers::HighImpedanceEnable,
        enabled
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetBatteryLoad(bool enabled) {
    return SetRegisterFlag(BQ25756_Registers::Register::ChargerControl, BQ25756_Registers::BatteryLoadEnable, enabled);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetChargeEnabled(bool enabled) {
    return SetRegisterFlag(BQ25756_Registers::Register::ChargerControl, BQ25756_Registers::ChargeEnable, enabled);
}

template<size_t BusId>
PlatformErr
BQ25756<BusId>::ConfigurePins(bool ichgEnabled, bool ilimHizEnabled, bool powerGoodEnabled, bool statusEnabled) {
    const uint8_t value = static_cast<uint8_t>(
        (ichgEnabled ? BQ25756_Registers::IchgPinEnable : 0U)
        | (ilimHizEnabled ? BQ25756_Registers::IlimHizPinEnable : 0U)
        | (!powerGoodEnabled ? BQ25756_Registers::PowerGoodPinDisable : 0U)
        | (!statusEnabled ? BQ25756_Registers::StatusPinsDisable : 0U)
    );
    return UpdateRegister8(
        BQ25756_Registers::Register::PinControl,
        static_cast<uint8_t>(
            BQ25756_Registers::IchgPinEnable | BQ25756_Registers::IlimHizPinEnable
            | BQ25756_Registers::PowerGoodPinDisable | BQ25756_Registers::StatusPinsDisable
        ),
        value
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ResetRegisters() {
    return SetRegisterFlag(
        BQ25756_Registers::Register::PowerPathReverseModeControl,
        BQ25756_Registers::RegisterReset,
        true
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetInputLoad(bool enabled) {
    return SetRegisterFlag(
        BQ25756_Registers::Register::PowerPathReverseModeControl,
        BQ25756_Registers::InputLoadEnable,
        enabled
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetPfmMode(bool enabled) {
    return SetRegisterFlag(
        BQ25756_Registers::Register::PowerPathReverseModeControl,
        BQ25756_Registers::PfmEnable,
        enabled
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetReverseMode(bool enabled) {
    return SetRegisterFlag(
        BQ25756_Registers::Register::PowerPathReverseModeControl,
        BQ25756_Registers::ReverseModeEnable,
        enabled
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetMpptEnabled(bool enabled) {
    return SetRegisterFlag(BQ25756_Registers::Register::MpptControl, BQ25756_Registers::MpptEnable, enabled);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetTemperaturePinEnabled(bool enabled) {
    return SetRegisterFlag(
        BQ25756_Registers::Register::TsChargingRegionBehaviorControl,
        BQ25756_Registers::TemperaturePinEnable,
        enabled
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ConfigureAdc(
    bool enabled,
    bool oneShot,
    BQ25756_AdcSampleSpeed sampleSpeed,
    bool runningAverage,
    bool initializeAverage
) {
    const uint8_t speed = static_cast<uint8_t>(sampleSpeed);
    if (speed > 2U) {
        ESP_LOGE(Tag, "Invalid ADC sample speed: %u", static_cast<unsigned>(speed));
        return PLATFORM_INVALID_PARAM;
    }
    const uint8_t value = static_cast<uint8_t>(
        (enabled ? BQ25756_Registers::AdcEnable : 0U) | (oneShot ? BQ25756_Registers::AdcOneShot : 0U) | (speed << 4U)
        | (runningAverage ? BQ25756_Registers::AdcRunningAverage : 0U)
        | (initializeAverage ? BQ25756_Registers::AdcInitializeAverage : 0U)
    );
    return WriteRegister8(BQ25756_Registers::Register::AdcControl, value);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ConfigureAdcChannels(
    bool inputCurrentEnabled,
    bool batteryCurrentEnabled,
    bool inputVoltageEnabled,
    bool batteryVoltageEnabled,
    bool temperatureEnabled,
    bool feedbackVoltageEnabled
) {
    const uint8_t value = static_cast<uint8_t>(
        (!inputCurrentEnabled ? BQ25756_Registers::InputCurrentAdcDisable : 0U)
        | (!batteryCurrentEnabled ? BQ25756_Registers::BatteryCurrentAdcDisable : 0U)
        | (!inputVoltageEnabled ? BQ25756_Registers::InputVoltageAdcDisable : 0U)
        | (!batteryVoltageEnabled ? BQ25756_Registers::BatteryVoltageAdcDisable : 0U)
        | (!temperatureEnabled ? BQ25756_Registers::TemperatureAdcDisable : 0U)
        | (!feedbackVoltageEnabled ? BQ25756_Registers::FeedbackVoltageAdcDisable : 0U)
    );
    return WriteRegister8(BQ25756_Registers::Register::AdcChannelControl, value);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadBatteryVoltage(float& voltageV) {
    uint16_t raw = 0U;
    const PlatformErr result = ReadRegister16(BQ25756_Registers::Register::BatteryVoltageAdc, raw);
    if (result == PLATFORM_OK)
        voltageV = static_cast<float>(raw) * 0.002F;
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadChargeCurrent(float& currentA) {
    if (config_.batterySenseResistanceOhm <= 0.0F) {
        ESP_LOGE(Tag, "Invalid battery current sense resistance");
        return PLATFORM_INVALID_PARAM;
    }
    uint16_t raw = 0U;
    const PlatformErr result = ReadRegister16(BQ25756_Registers::Register::BatteryCurrentAdc, raw);
    if (result == PLATFORM_OK) {
        currentA =
            static_cast<float>(static_cast<int16_t>(raw)) * 0.002F * (0.005F / config_.batterySenseResistanceOhm);
    }
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadInput(float& voltageV, float& currentA) {
    if (config_.inputSenseResistanceOhm <= 0.0F) {
        ESP_LOGE(Tag, "Invalid input current sense resistance");
        return PLATFORM_INVALID_PARAM;
    }
    uint16_t voltageRaw = 0U;
    PlatformErr result = ReadRegister16(BQ25756_Registers::Register::InputVoltageAdc, voltageRaw);
    if (result != PLATFORM_OK)
        return result;
    uint16_t currentRaw = 0U;
    result = ReadRegister16(BQ25756_Registers::Register::InputCurrentAdc, currentRaw);
    if (result == PLATFORM_OK) {
        voltageV = static_cast<float>(voltageRaw) * 0.002F;
        currentA =
            static_cast<float>(static_cast<int16_t>(currentRaw)) * 0.0008F * (0.002F / config_.inputSenseResistanceOhm);
    }
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadBatteryTemperature(float& temperatureC) {
    uint16_t raw = 0U;
    const PlatformErr result = ReadRegister16(BQ25756_Registers::Register::TemperatureAdc, raw);
    if (result != PLATFORM_OK)
        return result;

    constexpr float RegulatorVoltage = 5.0F;
    constexpr float SeriesResistance = 5240.0F;
    constexpr float ParallelResistance = 30310.0F;
    constexpr float NominalResistance = 10000.0F;
    constexpr float NominalTemperatureK = 298.15F;
    constexpr float Beta = 3435.0F;

    const float percent = static_cast<float>(raw & BQ25756_Registers::TemperatureAdcMask) * 0.09765625F;
    const float voltage = RegulatorVoltage * percent / 100.0F;
    if (voltage <= 0.0F || voltage >= RegulatorVoltage) {
        ESP_LOGD(Tag, "TS ADC sample unavailable: %.3fV", static_cast<double>(voltage));
        return PLATFORM_FAIL;
    }
    const float resistance = ParallelResistance * voltage / (RegulatorVoltage - voltage) - SeriesResistance;
    if (resistance <= 0.0F) {
        ESP_LOGD(Tag, "NTC sample unavailable: %.1f ohm", static_cast<double>(resistance));
        return PLATFORM_FAIL;
    }

    const float temperatureK =
        1.0F / ((1.0F / NominalTemperatureK) + (std::log(resistance / NominalResistance) / Beta));
    temperatureC = temperatureK - 273.15F;
    return PLATFORM_OK;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadFeedbackVoltage(float& voltageV) {
    uint16_t raw = 0U;
    const PlatformErr result = ReadRegister16(BQ25756_Registers::Register::FeedbackVoltageAdc, raw);
    if (result == PLATFORM_OK)
        voltageV = static_cast<float>(raw) * 0.001F;
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadChargeState(BQ25756_ChargeState& state) {
    uint8_t value = 0U;
    const PlatformErr result = ReadRegister8(BQ25756_Registers::Register::ChargerStatus1, value);
    if (result == PLATFORM_OK) {
        state = static_cast<BQ25756_ChargeState>(value & BQ25756_Registers::ChargeStateMask);
    }
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadFaultStatus(uint8_t& faultFlags) {
    const PlatformErr result = ReadRegister8(BQ25756_Registers::Register::FaultStatus, faultFlags);
    if (result == PLATFORM_OK && faultFlags != BQ25756_FaultNone) {
        ESP_LOGW(Tag, "Charger fault flags=0x%02X", static_cast<unsigned>(faultFlags));
    }
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadWatchdogExpired(bool& expired) {
    uint8_t value = 0U;
    const PlatformErr result = ReadRegister8(BQ25756_Registers::Register::ChargerStatus1, value);
    if (result == PLATFORM_OK) {
        expired = (value & BQ25756_Registers::WatchdogExpired) != 0U;
    }
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadMeasurements(BQ25756_Measurements& measurements) {
    BQ25756_OperationGuard operationGuard(operationMutex_);
    if (measurementInitializationPending_) {
        const PlatformErr initializationResult = ConfigureMeasurementsUnlocked(chargerConfig_);
        if (initializationResult != PLATFORM_OK)
            return StoreResult(initializationResult);
        measurementInitializationPending_ = false;
    }

    BQ25756_Measurements updated {};
    PlatformErr firstFailure = PLATFORM_OK;
    const auto recordResult = [&updated, &firstFailure](PlatformErr result, uint16_t field) {
        if (result == PLATFORM_OK) {
            updated.validFields |= field;
        } else if (firstFailure == PLATFORM_OK) {
            firstFailure = result;
        }
    };

    recordResult(ReadBatteryVoltage(updated.batteryVoltageV),
                 BQ25756_MeasurementBatteryVoltage);
    recordResult(ReadChargeCurrent(updated.batteryCurrentA),
                 BQ25756_MeasurementBatteryCurrent);
    recordResult(ReadInput(updated.inputVoltageV, updated.inputCurrentA),
                 BQ25756_MeasurementInput);
    recordResult(ReadBatteryTemperature(updated.batteryTemperatureC),
                 BQ25756_MeasurementBatteryTemperature);
    recordResult(ReadFeedbackVoltage(updated.feedbackVoltageV),
                 BQ25756_MeasurementFeedbackVoltage);
    recordResult(ReadChargeState(updated.chargeState),
                 BQ25756_MeasurementChargeState);
    recordResult(ReadFaultStatus(updated.faultFlags),
                 BQ25756_MeasurementFaultStatus);
    recordResult(ReadWatchdogExpired(updated.watchdogExpired),
                 BQ25756_MeasurementWatchdog);

    if (updated.validFields == 0U) {
        measurementInitializationPending_ = true;
        return StoreResult(firstFailure == PLATFORM_OK ? PLATFORM_FAIL : firstFailure);
    }

    measurements_ = updated;
    measurements = updated;
    ESP_LOGD(
        Tag,
        "Measurements: valid=0x%02X, VBAT=%.3fV, IBAT=%.3fA, VAC=%.3fV, IAC=%.3fA, temp=%.1fC",
        static_cast<unsigned>(updated.validFields),
        static_cast<double>(updated.batteryVoltageV),
        static_cast<double>(updated.batteryCurrentA),
        static_cast<double>(updated.inputVoltageV),
        static_cast<double>(updated.inputCurrentA),
        static_cast<double>(updated.batteryTemperatureC)
    );
    return StoreResult(PLATFORM_OK);
}

template<size_t BusId>
[[nodiscard]] const BQ25756_Measurements& BQ25756<BusId>::Measurements() const {
    return measurements_;
}

template<size_t BusId>
[[nodiscard]] PlatformErr BQ25756<BusId>::GpioInitResult() const {
    return gpioInitResult_;
}

template<size_t BusId>
[[nodiscard]] PlatformErr BQ25756<BusId>::LastResult() const {
    return lastResult_;
}

template<size_t BusId>
void BQ25756<BusId>::SetSampleCallback(typename BQ25756<BusId>::SampleCallback callback, void* context) {
    sampleCallback_ = callback;
    sampleContext_ = context;
}

template<size_t BusId>
void BQ25756<BusId>::SetFailureRetryCycles(uint32_t cycles) {
    failureRetryCycles_ = cycles;
}

template<size_t BusId>
void BQ25756<BusId>::Update() {
    samplePending_ = false;
    if (sampleCallback_ == nullptr)
        return;
    if (retryCyclesRemaining_ > 0U) {
        --retryCyclesRemaining_;
        return;
    }

    scheduledResult_ = ReadMeasurements(scheduledMeasurements_);
    samplePending_ = true;
    if (scheduledResult_ != PLATFORM_OK) {
        retryCyclesRemaining_ = failureRetryCycles_;
    }
}

template<size_t BusId>
void BQ25756<BusId>::Handle() {
    if (!samplePending_ || sampleCallback_ == nullptr)
        return;
    sampleCallback_(sampleContext_, scheduledResult_, scheduledMeasurements_);
    samplePending_ = false;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::StoreResult(PlatformErr result) {
    lastResult_ = result;
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ConfigureMeasurementsUnlocked(const BQ25756_ChargerConfig& config) {
    PlatformErr result = ConfigureAdcChannels(
        config.inputCurrentAdcEnabled,
        config.batteryCurrentAdcEnabled,
        config.inputVoltageAdcEnabled,
        config.batteryVoltageAdcEnabled,
        config.temperatureAdcEnabled,
        config.feedbackVoltageAdcEnabled
    );
    if (result != PLATFORM_OK)
        return result;
    return ConfigureAdc(
        config.adcEnabled,
        config.adcOneShot,
        config.adcSampleSpeed,
        config.adcRunningAverage,
        config.adcInitializeAverage
    );
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetCurrentLimit(
    BQ25756_Registers::Register reg,
    uint16_t mask,
    uint16_t currentMa,
    uint16_t minimumMa,
    uint16_t maximumMa
) {
    if (currentMa < minimumMa || currentMa > maximumMa) {
        ESP_LOGE(
            Tag,
            "Current limit out of range: %u mA (expected %u..%u mA)",
            static_cast<unsigned>(currentMa),
            static_cast<unsigned>(minimumMa),
            static_cast<unsigned>(maximumMa)
        );
        return PLATFORM_INVALID_PARAM;
    }
    const uint16_t value = static_cast<uint16_t>((currentMa / 50U) << 2U);
    return UpdateRegister16(reg, mask, value);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::SetRegisterFlag(BQ25756_Registers::Register reg, uint8_t mask, bool enabled) {
    return UpdateRegister8(reg, mask, enabled ? mask : 0U);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::UpdateRegister8(BQ25756_Registers::Register reg, uint8_t mask, uint8_t value) {
    uint8_t current = 0U;
    PlatformErr result = ReadRegister8(reg, current);
    if (result != PLATFORM_OK)
        return result;
    current = static_cast<uint8_t>((current & static_cast<uint8_t>(~mask)) | (value & mask));
    return WriteRegister8(reg, current);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::UpdateRegister16(BQ25756_Registers::Register reg, uint16_t mask, uint16_t value) {
    uint16_t current = 0U;
    PlatformErr result = ReadRegister16(reg, current);
    if (result != PLATFORM_OK)
        return result;
    current = static_cast<uint16_t>((current & static_cast<uint16_t>(~mask)) | (value & mask));
    return WriteRegister16(reg, current);
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::WriteRegister8(BQ25756_Registers::Register reg, uint8_t value) {
    const PlatformErr result =
        i2cAgent_.MemWrite(static_cast<uint8_t>(reg), 1U, I2C_MemoryAddressEndian::BigEndian, &value, 1U);
    if (result != PLATFORM_OK) {
        ESP_LOGE(Tag, "Register 0x%02X write failed: err=%d", static_cast<unsigned>(reg), static_cast<int>(result));
    }
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadRegister8(BQ25756_Registers::Register reg, uint8_t& value) {
    const PlatformErr result =
        i2cAgent_.MemRead(static_cast<uint8_t>(reg), 1U, I2C_MemoryAddressEndian::BigEndian, &value, 1U);
    if (result != PLATFORM_OK) {
        ESP_LOGE(Tag, "Register 0x%02X read failed: err=%d", static_cast<unsigned>(reg), static_cast<int>(result));
    }
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::WriteRegister16(BQ25756_Registers::Register reg, uint16_t value) {
    const uint8_t data[2] {
        static_cast<uint8_t>(value & 0xFFU),
        static_cast<uint8_t>(value >> 8U),
    };
    const PlatformErr result =
        i2cAgent_.MemWrite(static_cast<uint8_t>(reg), 1U, I2C_MemoryAddressEndian::BigEndian, data, sizeof(data));
    if (result != PLATFORM_OK) {
        ESP_LOGE(Tag, "Register 0x%02X write failed: err=%d", static_cast<unsigned>(reg), static_cast<int>(result));
    }
    return result;
}

template<size_t BusId>
PlatformErr BQ25756<BusId>::ReadRegister16(BQ25756_Registers::Register reg, uint16_t& value) {
    uint8_t data[2] {};
    const PlatformErr result =
        i2cAgent_.MemRead(static_cast<uint8_t>(reg), 1U, I2C_MemoryAddressEndian::BigEndian, data, sizeof(data));
    if (result == PLATFORM_OK) {
        value = static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8U));
    } else {
        ESP_LOGE(Tag, "Register 0x%02X read failed: err=%d", static_cast<unsigned>(reg), static_cast<int>(result));
    }
    return result;
}

#endif // FINEMOTE_BQ25756_HPP
